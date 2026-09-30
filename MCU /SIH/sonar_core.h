/*
  sonar_core.h  --  physics + decision logic + FPGA packet.
  Pure C++ (no Arduino / FreeRTOS calls) so it can be unit-tested on a PC.

  Chain (matches the team's intelligence-layer doc):
    sensors -> ocean state (sound speed, pressure)
            -> propagation (absorption, turbidity loss, spreading)
            -> channel (transmission loss, noise, SNR)
            -> constraints (band, battery energy, required SNR)
            -> output waveform (fc, bandwidth, pulse, amplitude, modulation, SF)
*/
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "sonar_config.h"

namespace sonar {

/* ---------------------------------------------------------------- types */
struct Env {                 // the 5 sensor inputs (engineering units)
  float tempC;
  float salinityPpt;
  float depthM;
  float turbidityNtu;
  float batteryPct;
};

enum PacketFlags : uint8_t {
  FLAG_LINK_OK        = 1 << 0,   // chosen profile meets the SNR requirement
  FLAG_LOW_BATTERY    = 1 << 1,
  FLAG_INPUT_CLAMPED  = 1 << 2,   // a sensor was outside its valid range / NaN
  FLAG_ML_USED        = 1 << 3,   // ML suggestion accepted (physics-verified)
  FLAG_BEST_EFFORT    = 1 << 4,   // nothing closes the link -> fallback profile
  FLAG_ML_OVERRIDDEN  = 1 << 5,   // ML suggestion rejected by physics
  FLAG_ML_AMP_RAISED  = 1 << 6    // ML amplitude too low to close the link: physics raised it
};

struct Decision {
  int      profile;            // index into PROFILES
  uint8_t  flags;
  float    ampFrac;            // 0..1 drive amplitude
  float    txPowerW;           // electrical power during the pulse
  float    energyJ;            // electrical energy for one ping
  float    snrOutDb;           // predicted output SNR at target range
  float    marginDb;           // snrOutDb - DETECTION_THRESHOLD_DB
  float    maxRangeM;          // reach of this profile at max allowed power
  float    soundSpeed;         // m/s
  float    wavelengthM;
  float    delayS;             // one-way propagation delay to target range
  float    alphaWaterDbKm;     // Francois-Garrison absorption
  float    alphaTurbDbKm;      // turbidity loss
  float    tlDb;               // one-way transmission loss
  float    n0Db;               // noise spectral density (incl. noise figure)
  float    energyBudgetJ;
  float    energyMinJ;         // energy of the cheapest profile that closes the link (0 if none)
  float    energyAllowanceJ;   // most energy the chosen profile may use (the detail/energy rule)
};

/* ------------------------------------------------------------- helpers */
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float db2pow(float db)   { return powf(10.0f, db / 10.0f); }
inline float pow2db(float p)    { return 10.0f * log10f(p); }

// Clamp sensors to the range where the models are valid. Returns true if
// anything had to be corrected (NaN, or out of range).
inline bool sanitizeEnv(Env &e) {
  bool changed = false;
  auto fix = [&](float &v, float lo, float hi) {
    if (v != v)      { v = 0.5f * (lo + hi); changed = true; }   // NaN
    else if (v < lo) { v = lo; changed = true; }
    else if (v > hi) { v = hi; changed = true; }
  };
  fix(e.tempC,        TEMP_MIN_C,      TEMP_MAX_C);
  fix(e.salinityPpt,  SALINITY_MIN_PPT, SALINITY_MAX_PPT);
  fix(e.depthM,       DEPTH_MIN_M,     DEPTH_MAX_M);
  fix(e.turbidityNtu, TURBIDITY_MIN_NTU, TURBIDITY_MAX_NTU);
  fix(e.batteryPct,   BATTERY_MIN_PCT, BATTERY_MAX_PCT);
  return changed;
}

/* ------------------------------------------------ 2. OCEAN STATE layer */

// Mackenzie (1981) sound speed. T [C], S [ppt], D [m] -> m/s.
inline float soundSpeedMackenzie(float T, float S, float D) {
  return 1448.96f + 4.591f * T - 5.304e-2f * T * T + 2.374e-4f * T * T * T
       + 1.340f * (S - 35.0f) + 1.630e-2f * D + 1.675e-7f * D * D
       - 1.025e-2f * T * (S - 35.0f) - 7.139e-13f * T * D * D * D;
}

// Absolute hydrostatic pressure [kPa]. (Francois-Garrison uses depth directly;
// this is reported for logging / the doc's "pressure as function of depth".)
inline float pressureKPa(float depthM) {
  return 101.325f + 1025.0f * 9.81f * depthM / 1000.0f;
}

inline float wavelengthM(float c, float fHz)  { return c / fHz; }
inline float propagationDelayS(float R, float c) { return R / c; }

/* ------------------------------------------- 3. ACOUSTIC PROPAGATION */

// Francois & Garrison (1982) total absorption, dB/km.
// f in kHz, T [C], S [ppt], D [m], pH, c [m/s]. Valid 0.2 kHz - 1 MHz.
inline float fgAbsorptionDbKm(float fk, float T, float S, float D, float pH, float c) {
  const float Tk = T + 273.0f;
  // boric acid (matters < 10 kHz)
  const float A1 = 8.86f / c * powf(10.0f, 0.78f * pH - 5.0f);
  const float f1 = 2.8f * sqrtf(S / 35.0f) * powf(10.0f, 4.0f - 1245.0f / Tk);
  // magnesium sulphate
  const float A2 = 21.44f * S / c * (1.0f + 0.025f * T);
  const float P2 = 1.0f - 1.37e-4f * D + 6.2e-9f * D * D;
  const float f2 = 8.17f * powf(10.0f, 8.0f - 1990.0f / Tk) / (1.0f + 0.0018f * (S - 35.0f));
  // pure water (viscous) -- dominant at 100-500 kHz
  float A3;
  if (T <= 20.0f) A3 = 4.937e-4f - 2.59e-5f * T + 9.11e-7f * T * T - 1.50e-8f * T * T * T;
  else            A3 = 3.964e-4f - 1.146e-5f * T + 1.45e-7f * T * T - 6.5e-10f * T * T * T;
  const float P3 = 1.0f - 3.83e-5f * D + 4.9e-10f * D * D;

  const float fsq = fk * fk;
  return A1 * f1 * fsq / (fsq + f1 * f1)
       + A2 * P2 * f2 * fsq / (fsq + f2 * f2)
       + A3 * P3 * fsq;
}

// Extra loss from suspended particles (empirical, see config note).
inline float turbidityAttenDbKm(float fk, float ntu) {
  return TURB_ATTEN_DB_KM_PER_NTU_AT_100K * ntu * powf(fk / 100.0f, TURB_FREQ_EXPONENT);
}

inline float spreadingLossDb(float R) {
  return SPREADING_COEFF * log10f(R < 1.0f ? 1.0f : R);
}

/* ------------------------------------------------------ 4. CHANNEL layer */

// Ambient noise spectral density, dB re 1 uPa^2/Hz, f in kHz.
// Thermal (Mellen): -15 + 20 log f.  Wind: 50 + 7.5 sqrt(w) + 20 log f - 40 log(f+0.4).
inline float ambientNoiseDb(float fk) {
  const float th = -15.0f + 20.0f * log10f(fk);
  const float wd = 50.0f + 7.5f * sqrtf(ASSUMED_WIND_SPEED_MS) + 20.0f * log10f(fk)
                 - 40.0f * log10f(fk + 0.4f);
  return pow2db(db2pow(th) + db2pow(wd));
}
inline float noiseDensityDb(float fk) { return ambientNoiseDb(fk) + RX_NOISE_FIGURE_DB; }

// Source level, dB re 1 uPa @ 1 m:  SL = 170.8 + 10 log(P_acoustic) + DI
inline float sourceLevelDb(float pElecW) {
  return 170.8f + pow2db(TX_EFFICIENCY * pElecW) + TX_DIRECTIVITY_DB;
}

/* ------------------------------------------------ 5. CONSTRAINTS layer */

inline float energyBudgetJ(float socPct) {
  return ENERGY_PER_PING_MAX_J * clampf(socPct / 100.0f, SOC_FLOOR_PCT / 100.0f, 1.0f);
}

// A profile in the FPGA's units. FTW = f x 2^32 / FPGA_CLK_HZ.
struct FpgaWave {
  uint32_t lenClk;       // pulse length, FPGA clocks (BPSK: rounded to 13 whole chips)
  uint32_t ftwStart;     // start frequency: fc for CW and BPSK, fc - bw/2 for the sweeps
  int32_t  ftwStep;      // LFM: FTW change per clock x 2^16.  GEOMETRIC: (ratio - 1) x 2^32,
                         // applied every 32 clocks.  CW, BPSK: 0
  uint32_t winStep;      // 2^32 / lenClk: the Hann window makes one lap per pulse
  uint16_t code;         // BPSK: phase code (chip i = bit i, 1 = flip), else 0
  uint32_t chipLen;      // BPSK: clocks per chip, else 0
  bool     fits;         // every value fits its field
};

inline FpgaWave fpgaWave(const WaveProfile &p) {
  const double two32 = 4294967296.0;
  const double K = two32 / FPGA_CLK_HZ;                     // FTW per Hz
  const bool sweep = (p.mod == MOD_LFM_CHIRP || p.mod == MOD_GEOMETRIC);
  const bool bpsk  = (p.mod == MOD_BPSK);
  double len = floor((double)p.pulseS * FPGA_CLK_HZ + 0.5);
  double chip = 0.0;
  if (bpsk) {
    chip = floor(len / BARKER13_CHIPS + 0.5);
    len  = chip * BARKER13_CHIPS;
  }
  const double f0 = sweep ? (double)p.fcHz - 0.5 * (double)p.bwHz : (double)p.fcHz;
  const double f1 = sweep ? (double)p.fcHz + 0.5 * (double)p.bwHz : (double)p.fcHz;
  const double updates = floor(len / 32.0);                 // geometric: one update per 32 clocks
  double step = 0.0;
  if (p.mod == MOD_LFM_CHIRP)
    step = floor((f1 - f0) * K / len * (1 << FPGA_STEP_FRAC_BITS) + 0.5);
  else if (p.mod == MOD_GEOMETRIC && updates >= 1.0 && f0 > 0.0)
    step = floor((pow(f1 / f0, 1.0 / updates) - 1.0) * two32 + 0.5);
  FpgaWave w;
  w.fits = len >= 64.0 && len <= (double)FPGA_LEN_MAX_CLK && chip <= (double)FPGA_LEN_MAX_CLK
        && f0 > 0.0 && f1 * K < 0.5 * two32 && step >= 0.0 && step < 2147483647.0;
  w.lenClk   = w.fits ? (uint32_t)len : 64u;
  w.ftwStart = w.fits ? (uint32_t)floor(f0 * K + 0.5) : 0u;
  w.ftwStep  = w.fits ? (int32_t)step : 0;
  w.winStep  = (uint32_t)floor(two32 / w.lenClk + 0.5);
  w.code     = (w.fits && bpsk) ? (uint16_t)BARKER13_CODE : (uint16_t)0;
  w.chipLen  = (w.fits && bpsk) ? (uint32_t)chip : 0u;
  return w;
}

// The same profile with another modulation (the operator's M command); mod < 0 keeps it.
inline WaveProfile withModulation(const WaveProfile &p, int mod) {
  WaveProfile q = p;
  if (mod >= MOD_CW && mod <= MOD_BPSK) q.mod = (Modulation)mod;
  return q;
}

inline bool profileInBand(const WaveProfile &p) {
  return p.bwHz > 0.0f && p.pulseS > 0.0f
      && (p.fcHz - 0.5f * p.bwHz) >= BAND_MIN_HZ
      && (p.fcHz + 0.5f * p.bwHz) <= BAND_MAX_HZ
      && fpgaWave(p).fits;
}

/* -------------------------------------- 6. OUTPUT: evaluate one profile */
/*
  Output SNR after pulse compression (matched filter):
      SNR_out = SL + 10 log10(T) - PathLoss - N0            (+ TS in echo mode)
  In-band noise (N0 + 10 log B) and compression gain (10 log B*T) cancel, so
  SNR depends on pulse ENERGY, not bandwidth. Bandwidth only buys resolution.
*/
struct Eval {
  bool  feasible;
  float pReqW, pCapW, pUseW, ampFrac, snrOutDb;
  float tlDb, n0Db, alphaWaterDbKm, alphaTurbDbKm;
};

inline Eval evaluateProfile(const WaveProfile &p, const Env &e, float c, float R,
                            float eBudgetJ, float extraDb) {
  Eval ev;
  memset(&ev, 0, sizeof(ev));
  const float fk = p.fcHz / 1000.0f;
  ev.alphaWaterDbKm = fgAbsorptionDbKm(fk, e.tempC, e.salinityPpt, e.depthM, ASSUMED_PH, c);
  ev.alphaTurbDbKm  = turbidityAttenDbKm(fk, e.turbidityNtu);
  const float Rm = R < 1.0f ? 1.0f : R;
  ev.tlDb  = spreadingLossDb(Rm) + (ev.alphaWaterDbKm + ev.alphaTurbDbKm) * Rm / 1000.0f;
  ev.n0Db  = noiseDensityDb(fk);

#if ACTIVE_ECHO_MODE
  const float path = 2.0f * ev.tlDb - TARGET_STRENGTH_DB;
#else
  const float path = ev.tlDb;
#endif

  // Solve  170.8 + DI + 10log(eta*E) - path - N0 = need   for electrical energy E
  const float need = DETECTION_THRESHOLD_DB + LINK_MARGIN_DB + extraDb;
  const float L    = need + path + ev.n0Db - 170.8f - TX_DIRECTIVITY_DB;
  const float eReq = db2pow(L) / TX_EFFICIENCY;              // may be +inf: fine
  ev.pReqW = eReq / p.pulseS;
  ev.pCapW = fminf(TX_ELEC_POWER_MAX_W, eBudgetJ / p.pulseS);

  bool ok = profileInBand(p) && (ev.pReqW <= ev.pCapW);
#if ACTIVE_ECHO_MODE
  if (p.pulseS > 2.0f * Rm / c) ok = false;                  // pulse must end before echo returns
#endif
  ev.feasible = ok;

  // Power actually used: just enough (with a floor); if infeasible, everything allowed.
  const float pFloor = AMP_MIN_FRAC * AMP_MIN_FRAC * TX_ELEC_POWER_MAX_W;
  float pUse = ok ? fmaxf(ev.pReqW, pFloor) : ev.pCapW;
  pUse = fminf(pUse, ev.pCapW);
  ev.pUseW   = pUse;
  ev.ampFrac = sqrtf(pUse / TX_ELEC_POWER_MAX_W);
  ev.snrOutDb = 170.8f + TX_DIRECTIVITY_DB + pow2db(TX_EFFICIENCY * pUse * p.pulseS)
              - path - ev.n0Db;
  return ev;
}

// Farthest range at which this profile still closes the link at max allowed power.
inline float maxRangeM(const WaveProfile &p, const Env &e, float c, float eBudgetJ) {
  float lo = 1.0f, hi = 20000.0f;
  if (!evaluateProfile(p, e, c, lo, eBudgetJ, 0.0f).feasible) return 0.0f;
  if (evaluateProfile(p, e, c, hi, eBudgetJ, 0.0f).feasible) return hi;
  for (int i = 0; i < 30; i++) {
    const float mid = 0.5f * (lo + hi);
    if (evaluateProfile(p, e, c, mid, eBudgetJ, 0.0f).feasible) lo = mid; else hi = mid;
  }
  return lo;
}

/* --------------------------------------------------- the decision step */
/*
  1. Evaluate every profile: can it close the link, and how much energy would one ping use?
     A profile finer than the previous choice must clear an extra SWITCH_UP_HYSTERESIS_DB
     (anti flip-flop), which also raises the energy it is charged with.
  2. allowance = max(DETAIL_ENERGY_FACTOR x the cheapest working profile's energy,
                     DETAIL_FREE_ENERGY_FRAC x this ping's energy budget)
  3. Take the finest (earliest) profile that closes the link within the allowance.
     If none closes the link, use the last profile at maximum allowed power (FLAG_BEST_EFFORT).

  mlHint / mlAmp: the ML model's profile and amplitude (-1 = none). The ML's profile is used
  when it passes the same test as step 3 (it may choose a cheaper, coarser profile than the
  physics, never one that fails the link or costs more than the allowance); otherwise the
  physics' choice replaces it. The ML's amplitude is used for its own profile, raised to what
  the link needs if it is too low (FLAG_ML_AMP_RAISED) and capped at what is allowed.
*/
inline Decision decide(Env env, float rangeM, int prevProfile, int mlHint, float mlAmp = -1.0f) {
  Decision d;
  memset(&d, 0, sizeof(d));

  if (sanitizeEnv(env)) d.flags |= FLAG_INPUT_CLAMPED;
  rangeM = clampf(rangeM, (float)MIN_TARGET_RANGE_M, (float)MAX_TARGET_RANGE_M);

  const float c  = soundSpeedMackenzie(env.tempC, env.salinityPpt, env.depthM);
  const float eb = energyBudgetJ(env.batteryPct);
  if (env.batteryPct < LOW_BATTERY_PCT) d.flags |= FLAG_LOW_BATTERY;

  // 1. every profile, with and without the switch-up margin
  Eval  ev[NUM_PROFILES], plain[NUM_PROFILES];
  float energy[NUM_PROFILES];
  float eMin = INFINITY;
  for (int i = 0; i < NUM_PROFILES; i++) {
    const float extra = (prevProfile >= 0 && i < prevProfile) ? SWITCH_UP_HYSTERESIS_DB : 0.0f;
    plain[i]  = evaluateProfile(PROFILES[i], env, c, rangeM, eb, 0.0f);
    ev[i]     = (extra > 0.0f) ? evaluateProfile(PROFILES[i], env, c, rangeM, eb, extra) : plain[i];
    energy[i] = ev[i].pUseW * PROFILES[i].pulseS;
    if (plain[i].feasible) eMin = fminf(eMin, plain[i].pUseW * PROFILES[i].pulseS);
  }
  // 2. how much energy more detail may cost
  const float allowance = fmaxf(DETAIL_ENERGY_FACTOR * eMin, DETAIL_FREE_ENERGY_FRAC * eb);
  d.energyAllowanceJ = (eMin < INFINITY) ? allowance : 0.0f;

  // 3. the ML's pick if it passes, else the finest profile that does
  int  chosen = -1;
  Eval chosenEv;
  memset(&chosenEv, 0, sizeof(chosenEv));
  if (mlHint >= 0 && mlHint < NUM_PROFILES) {
    if (ev[mlHint].feasible && energy[mlHint] <= allowance) {
      chosen = mlHint; chosenEv = ev[mlHint]; d.flags |= FLAG_ML_USED;
    } else {
      d.flags |= FLAG_ML_OVERRIDDEN;
    }
  }
  for (int i = 0; chosen < 0 && i < NUM_PROFILES; i++)
    if (ev[i].feasible && energy[i] <= allowance) { chosen = i; chosenEv = ev[i]; }
  // the switch-up margin can rule out every working profile: then choose without it
  for (int i = 0; chosen < 0 && eMin < INFINITY && i < NUM_PROFILES; i++)
    if (plain[i].feasible && plain[i].pUseW * PROFILES[i].pulseS <= allowance) { chosen = i; chosenEv = plain[i]; }

  if (chosen >= 0) {
    d.flags |= FLAG_LINK_OK;
  } else {
    chosen   = NUM_PROFILES - 1;
    chosenEv = plain[chosen];
    d.flags |= FLAG_BEST_EFFORT;
  }

  // amplitude: the physics' "just enough", or the ML's for its own profile
  const WaveProfile &p = PROFILES[chosen];
  float pUse = chosenEv.pUseW;
  if ((d.flags & FLAG_ML_USED) && mlAmp >= 0.0f) {
    const float aNeed = chosenEv.ampFrac;                                 // just enough (with the floor)
    const float aCap  = sqrtf(chosenEv.pCapW / TX_ELEC_POWER_MAX_W);
    float a = clampf(mlAmp, AMP_MIN_FRAC, aCap);
    if (a < aNeed) { a = aNeed; d.flags |= FLAG_ML_AMP_RAISED; }
    pUse = a * a * TX_ELEC_POWER_MAX_W;
  }
  d.profile        = chosen;
  d.ampFrac        = sqrtf(pUse / TX_ELEC_POWER_MAX_W);
  d.txPowerW       = pUse;
  d.energyJ        = pUse * p.pulseS;
  d.snrOutDb       = chosenEv.snrOutDb + pow2db(pUse / chosenEv.pUseW);
  d.marginDb       = d.snrOutDb - DETECTION_THRESHOLD_DB;
  d.soundSpeed     = c;
  d.wavelengthM    = wavelengthM(c, p.fcHz);
  d.delayS         = propagationDelayS(rangeM, c);
  d.alphaWaterDbKm = chosenEv.alphaWaterDbKm;
  d.alphaTurbDbKm  = chosenEv.alphaTurbDbKm;
  d.tlDb           = chosenEv.tlDb;
  d.n0Db           = chosenEv.n0Db;
  d.energyBudgetJ  = eb;
  d.energyMinJ     = (eMin < INFINITY) ? eMin : 0.0f;
  d.maxRangeM      = maxRangeM(p, env, c, eb);
  return d;
}

// Boot-time sanity check of the profile table: band limits, and every profile fits the
// FPGA's fields with every modulation the operator can choose.
inline bool validateProfiles(int *badIndex) {
  for (int i = 0; i < NUM_PROFILES; i++) {
    bool ok = profileInBand(PROFILES[i]);
    for (int m = MOD_CW; m <= MOD_BPSK; m++)
      ok = ok && fpgaWave(withModulation(PROFILES[i], m)).fits;
    if (!ok) { if (badIndex) *badIndex = i; return false; }
  }
  return true;
}

/* -------------------------------------------------------- FPGA packet */
#pragma pack(push, 1)
struct SonarPacket {            // little-endian, 57 bytes, sent over SPI
  uint8_t  sync;                // 0xA5
  uint8_t  version;             // 3 (v2 added the FPGA-unit fields, v3 BPSK/geometric + period)
  uint16_t seq;                 // increments per new decision
  uint8_t  profileId;
  uint8_t  modulation;          // Modulation: 0 CW, 1 LFM, 2 geometric, 3 BPSK (the FPGA uses this)
  uint8_t  spreadingFactor;     // round(log2(bw*T)) for chirps, 0 for CW
  uint8_t  flags;               // PacketFlags
  uint32_t centerFreqHz;
  uint32_t bandwidthHz;
  uint32_t pulseUs;
  uint16_t amplitude;           // 0..65535 = 0..100 % drive
  uint16_t txPowerMilliW;       // electrical, informational
  uint32_t propDelayUs;         // one-way delay to target range
  // --- what the FPGA actually uses (see FpgaWave) ---
  uint32_t lenClk;              // pulse length, FPGA clocks
  uint32_t ftwStart;            // start frequency, FTW
  int32_t  ftwStep;             // LFM: FTW/clock x 2^16; geometric: (ratio - 1) x 2^32 per 32 clocks
  uint32_t winStep;             // 2^32 / lenClk
  uint16_t ampQ12;              // amplitude, 4096 = 1.0
  uint16_t code;                // BPSK phase code, chip i = bit i (0 = no flips)
  uint32_t chipLen;             // BPSK clocks per chip
  uint32_t periodClk;           // FPGA clocks from one pulse start to the next
  uint8_t  crc8;                // CRC-8 (poly 0x07, init 0) over all previous bytes
};
#pragma pack(pop)
static_assert(sizeof(SonarPacket) == 57, "SonarPacket must be 57 bytes");

inline uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// mod: the operator's modulation override (M command), or -1 for the profile's own.
inline SonarPacket buildPacket(const Decision &d, uint16_t seq, int mod = -1) {
  const WaveProfile p = withModulation(PROFILES[d.profile], mod);
  SonarPacket k;
  memset(&k, 0, sizeof(k));
  k.sync       = 0xA5;
  k.version    = 3;
  k.seq        = seq;
  k.profileId  = (uint8_t)d.profile;
  k.modulation = (uint8_t)p.mod;
  k.spreadingFactor = (p.mod == MOD_LFM_CHIRP || p.mod == MOD_GEOMETRIC)
                    ? (uint8_t)lroundf(log2f(p.bwHz * p.pulseS)) : 0;
  k.flags      = d.flags;
  k.centerFreqHz  = (uint32_t)lroundf(p.fcHz);
  k.bandwidthHz   = (uint32_t)lroundf(p.bwHz);
  k.pulseUs       = (uint32_t)lroundf(p.pulseS * 1.0e6f);
  k.amplitude     = (uint16_t)lroundf(clampf(d.ampFrac, 0.0f, 1.0f) * 65535.0f);
  k.txPowerMilliW = (uint16_t)lroundf(clampf(d.txPowerW * 1000.0f, 0.0f, 65535.0f));
  k.propDelayUs   = (uint32_t)lroundf(d.delayS * 1.0e6f);
  const FpgaWave w = fpgaWave(p);
  k.lenClk        = w.lenClk;
  k.ftwStart      = w.ftwStart;
  k.ftwStep       = w.ftwStep;
  k.winStep       = w.winStep;
  k.ampQ12        = (uint16_t)lroundf(clampf(d.ampFrac, 0.0f, 1.0f) * 4096.0f);
  k.code          = w.code;
  k.chipLen       = w.chipLen;
  double periodS = (double)p.pulseS;
#if ACTIVE_ECHO_MODE
  periodS += 2.0 * (double)d.delayS;                        // wait for the echo
#endif
  if (periodS < (double)PING_PERIOD_MIN_S) periodS = (double)PING_PERIOD_MIN_S;
  const double periodClk = floor(periodS * FPGA_CLK_HZ + 0.5);
  k.periodClk = periodClk > (double)w.lenClk ? (uint32_t)periodClk : w.lenClk + 1u;
  k.crc8 = crc8((const uint8_t *)&k, sizeof(k) - 1);
  return k;
}

}  // namespace sonar
