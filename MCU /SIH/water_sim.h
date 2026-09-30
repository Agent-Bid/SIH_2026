/*
  water_sim.h  --  simulated sensors: realistic, changing water conditions for testing the
  whole chain without real sensors. Pure C++ (no Arduino calls), deterministic from its seed.

  The scene: an underwater vehicle dives and climbs between 20 and 350 m (a "yo-yo" survey)
  through a thermocline, while tracking three contacts; each ping goes to the next contact.
    - temperature: warm mixed layer over a thermocline (22 C at the surface, 5 C deep)
    - salinity:    a halocline, 34.4 ppt near the surface to 35.2 ppt deep
    - turbidity:   a few NTU of background, a muddy layer near the bottom (below 300 m),
                   and drifting sediment plumes
    - battery:     drains with time, recharged at 15 %
    - contacts:    A near and moving (a diver or small vehicle), B mid-range and nearly still
                   (a moored object), C far (450-1000 m) and moving (a ship); range and radial speed evolve
                   as random but smooth motions, and the speed is the rate the range changes
  Every sensor reading has a little noise. Mission time runs SIM_SPEEDUP x faster than real
  time, so the dive and the battery change visibly within minutes.
*/
#pragma once
#include <math.h>
#include <stdint.h>
#include "sonar_config.h"

#ifndef SIM_SPEEDUP
#define SIM_SPEEDUP 10.0f
#endif

namespace watersim {

struct Contact {
  const char *name;
  float rangeM, velMs;          // range, and its rate of change (+ = moving away)
  float rMin, rMax;             // where it stays
  float velSigma;               // how lively its motion is, m/s
  float velTau;                 // how long a motion lasts, s
};

struct Reading {                // one ping's inputs
  float tempC, salinityPpt, depthM, turbidityNtu, batteryPct;
  float rangeM, speedMs;
  int   contact;
  float missionS;
};

struct Sim {
  uint32_t rng;
  float    t;                   // mission time, s
  float    battery;             // %
  float    plume;               // sediment plume, NTU (slowly varying)
  int      next;                // contact of the next ping
  Contact  c[3];
};

inline float uniform01(Sim &s) {                     // xorshift32
  s.rng ^= s.rng << 13; s.rng ^= s.rng >> 17; s.rng ^= s.rng << 5;
  return (s.rng >> 8) * (1.0f / 16777216.0f);
}
inline float gauss(Sim &s) {                         // Box-Muller
  const float u = uniform01(s) + 1e-7f, v = uniform01(s);
  return sqrtf(-2.0f * logf(u)) * cosf(6.2831853f * v);
}
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline void init(Sim &s, uint32_t seed = 12345u) {
  s.rng = seed ? seed : 1u;
  s.t = 0.0f;
  s.battery = 100.0f;
  s.plume = 5.0f;
  s.next = 0;
  s.c[0] = { "A",   60.0f,  1.5f,   15.0f,  180.0f, 2.0f,  20.0f };
  s.c[1] = { "B",  400.0f,  0.02f, 250.0f,  600.0f, 0.05f, 60.0f };
  s.c[2] = { "C",  800.0f, -3.0f,  450.0f, 1000.0f, 2.5f,  40.0f };
}

// Depth of the yo-yo profile: 20 -> 350 -> 20 m every 20 minutes of mission time
inline float depthAt(float t) {
  const float period = 1200.0f, x = fmodf(t, period) / period;
  return 20.0f + 330.0f * (x < 0.5f ? 2.0f * x : 2.0f - 2.0f * x);
}
inline float temperatureAt(float d) { return 5.0f + 17.0f / (1.0f + expf((d - 80.0f) / 20.0f)); }
inline float salinityAt(float d)    { return 34.8f + 0.4f * tanhf((d - 60.0f) / 40.0f); }

// Advance the scene by dt seconds of real time and read the inputs for the next ping.
inline Reading step(Sim &s, float dtReal) {
  const float dt = dtReal * SIM_SPEEDUP;
  s.t += dt;
  s.battery -= 0.005f * dt;                                         // 100 % -> 15 % in ~4.7 h mission,
  if (s.battery < 15.0f) s.battery = 100.0f;                        // then it docks and recharges

  // plume: Ornstein-Uhlenbeck around 5 NTU, occasionally big
  const float a = expf(-dt / 120.0f);
  s.plume = 5.0f + (s.plume - 5.0f) * a + 12.0f * sqrtf(1.0f - a * a) * gauss(s);
  s.plume = clampf(s.plume, 0.0f, 60.0f);

  // every contact moves; its radial speed wanders smoothly (Ornstein-Uhlenbeck)
  for (Contact &c : s.c) {
    const float b = expf(-dt / c.velTau);
    c.velMs = c.velMs * b + c.velSigma * sqrtf(1.0f - b * b) * gauss(s);
    c.rangeM += c.velMs * dt;
    if (c.rangeM < c.rMin) { c.rangeM = 2.0f * c.rMin - c.rangeM; c.velMs = fabsf(c.velMs); }
    if (c.rangeM > c.rMax) { c.rangeM = 2.0f * c.rMax - c.rangeM; c.velMs = -fabsf(c.velMs); }
  }

  Reading r;
  const float d = depthAt(s.t);
  const float nepheloid = d > 300.0f ? 40.0f * (d - 300.0f) / 50.0f : 0.0f;
  r.depthM       = clampf(d + 0.5f * gauss(s), DEPTH_MIN_M, DEPTH_MAX_M);
  r.tempC        = clampf(temperatureAt(d) + 0.05f * gauss(s), TEMP_MIN_C, TEMP_MAX_C);
  r.salinityPpt  = clampf(salinityAt(d) + 0.02f * gauss(s), SALINITY_MIN_PPT, SALINITY_MAX_PPT);
  r.turbidityNtu = clampf(3.0f + s.plume + nepheloid + 0.5f * gauss(s), TURBIDITY_MIN_NTU, TURBIDITY_MAX_NTU);
  r.batteryPct   = s.battery;
  r.contact      = s.next;
  const Contact &c = s.c[s.next];
  r.rangeM       = clampf(c.rangeM, (float)MIN_TARGET_RANGE_M, (float)MAX_TARGET_RANGE_M);
  r.speedMs      = clampf(fabsf(c.velMs), SPEED_MIN_MS, SPEED_MAX_MS);
  r.missionS     = s.t;
  s.next = (s.next + 1) % 3;
  return r;
}

}  // namespace watersim
