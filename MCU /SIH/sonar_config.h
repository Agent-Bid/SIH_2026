/*
  sonar_config.h  --  ALL team-editable numbers live here.
  Physics/decision code is in sonar_core.h (please don't edit that unless
  the physics itself is being changed).

  Anything marked  [ASSUMPTION]  is a value nobody has confirmed yet.
  See README_adaptive_sonar.md -> "Assumptions" for the reasoning and the
  person who should confirm each one.
*/
#pragma once
#include <stdint.h>

/* ========================================================================
   1. OPERATING BAND + TRANSMITTER HARDWARE
   ======================================================================== */
#define BAND_MIN_HZ            100000.0f   // team requirement: 100 kHz ...
#define BAND_MAX_HZ            500000.0f   // ... to 500 kHz (whole chirp must fit inside)

#define TX_ELEC_POWER_MAX_W    20.0f       // [ASSUMPTION] peak electrical power into transducer
#define TX_EFFICIENCY          0.50f       // [ASSUMPTION] electrical -> acoustic efficiency
#define TX_DIRECTIVITY_DB      0.0f        // [ASSUMPTION] 0 = omnidirectional (conservative)
#define RX_NOISE_FIGURE_DB     6.0f        // [ASSUMPTION] receiver noise figure added to ocean noise

/* ========================================================================
   1b. FPGA WAVEFORM ENGINE  (must match the FPGA build -- ask the FPGA owner)
   The MCU converts each profile into the FPGA's own units (FTW = fraction of
   a sine-table lap per FPGA clock, as a 32-bit number) so the FPGA never divides.
   ======================================================================== */
#define FPGA_CLK_HZ            (27.0e6 * 13.0 / 7.0)   // 27 MHz x PLL 13/7 = 50.142857 MHz
#define FPGA_STEP_FRAC_BITS    16                      // ftwStepQ16 = FTW change per clock x 2^16
#define FPGA_LEN_MAX_CLK       16777215UL              // 24-bit pulse length and chip length (~335 ms)

// Phase code for BPSK: Barker-13, chip i = bit i, 1 = flip:  + + + + + - - + + - + - +
#define BARKER13_CODE          0x0A60
#define BARKER13_CHIPS         13

// Time between pulse starts. One-way link: this minimum. Echo mode: at least
// pulse + round trip (2 x range / c), so a pulse never starts before the echo is back.
#define PING_PERIOD_MIN_S      0.1f                    // [ASSUMPTION] 10 pings per second at most

/* ========================================================================
   2. WATER / CHANNEL MODEL SETTINGS (things we do NOT have a sensor for)
   ======================================================================== */
#define ASSUMED_PH             8.0f        // only matters < 10 kHz, harmless at 100-500 kHz
#define ASSUMED_WIND_SPEED_MS  5.0f        // [ASSUMPTION] for wind noise (small vs thermal above ~200 kHz)
#define SPREADING_COEFF        20.0f       // 20 = spherical spreading, 15 = "practical" spreading

// Turbidity -> extra attenuation. [ASSUMPTION - NEEDS CALIBRATION]
// Anchored to Richards, Heathershaw & Thorne (JASA 1996): ~3 dB over 100 m at
// 100 kHz for 0.2 kg/m^3 of suspended sediment  =>  ~0.15 dB/km per (mg/L).
// We treat 1 NTU ~ 1 mg/L (site dependent!). Set to 0 to switch turbidity loss off.
#define TURB_ATTEN_DB_KM_PER_NTU_AT_100K  0.15f
#define TURB_FREQ_EXPONENT                1.0f   // loss grows ~ f^n

/* ========================================================================
   3. WHAT DOES THE SONAR DO?   (decide with the team!)
   ======================================================================== */
// 0 = one-way link (transmit -> receiver). Same convention as the ML dataset.
// 1 = active echo sonar (out-and-back path, needs target strength).
#define ACTIVE_ECHO_MODE       0
#define TARGET_STRENGTH_DB     (-20.0f)    // used only if ACTIVE_ECHO_MODE = 1

/* ========================================================================
   4. LINK REQUIREMENTS
   ======================================================================== */
#define DETECTION_THRESHOLD_DB 12.0f       // required output SNR (same 12 dB as ML dataset)
#define LINK_MARGIN_DB         3.0f        // extra safety for refraction, multipath, filter loss...
#define SWITCH_UP_HYSTERESIS_DB 2.0f       // need this much EXTRA margin before moving to a
                                           // finer-resolution profile (stops flip-flopping)
#define AMP_MIN_FRAC           0.05f       // never drive below 5 % amplitude

/* ========================================================================
   4b. DETAIL vs ENERGY: WHICH PROFILE TO USE   (decide with the team!)
   ------------------------------------------------------------------------
   Among the profiles that close the link, take the one with the finest detail
   (earliest in the table) whose energy per ping is at most
       DETAIL_ENERGY_FACTOR x the energy of the cheapest profile that works,
   or, if that is more, DETAIL_FREE_ENERGY_FRAC of this ping's energy budget
   (energy that small is not worth saving; the budget shrinks with the battery,
   so a low battery leans towards saving energy).
   Lower frequencies are always cheaper (less absorption, less thermal noise)
   but give less detail: these two numbers set the trade-off.
     DETAIL_ENERGY_FACTOR 1       -> always the cheapest profile
     DETAIL_ENERGY_FACTOR 1e9     -> always the finest profile that works
   ======================================================================== */
#define DETAIL_ENERGY_FACTOR     4.0f       // [ASSUMPTION] pay up to 4x the minimum energy for more detail
#define DETAIL_FREE_ENERGY_FRAC  0.01f      // [ASSUMPTION] up to 1 % of the ping's budget always counts as affordable

/* ========================================================================
   5. BATTERY  ->  ENERGY ALLOWED PER PING
   ======================================================================== */
#define ENERGY_PER_PING_MAX_J  0.4f        // [ASSUMPTION] electrical energy allowed per ping at 100 % SOC
#define SOC_FLOOR_PCT          10.0f       // budget stops shrinking below this SOC (never fully silent)
#define LOW_BATTERY_PCT        20.0f       // sets the LOW_BATTERY flag in the packet

/* ========================================================================
   6. TARGET RANGE  (not a sensor: it is a mission setting)
   ======================================================================== */
#define DEFAULT_TARGET_RANGE_M 300
#define MIN_TARGET_RANGE_M     10
#define MAX_TARGET_RANGE_M     2000        // can be changed at runtime from Serial: "R 450"

/* ========================================================================
   7. SENSOR VALID RANGES (also the potentiometer end-stops)
      Limits follow the validity of the Mackenzie sound-speed equation.
   ======================================================================== */
#define TEMP_MIN_C             2.0f
#define TEMP_MAX_C             30.0f
#define SALINITY_MIN_PPT       25.0f
#define SALINITY_MAX_PPT       40.0f
#define DEPTH_MIN_M            0.0f
#define DEPTH_MAX_M            1000.0f
#define TURBIDITY_MIN_NTU      0.0f
#define TURBIDITY_MAX_NTU      100.0f
#define BATTERY_MIN_PCT        0.0f
#define BATTERY_MAX_PCT        100.0f

/* ========================================================================
   8. WAVEFORM PROFILES
   ------------------------------------------------------------------------
   Ordered from MOST preferred (finest range resolution) to LEAST preferred
   (longest reach). The decision code walks down the list and takes the
   first profile whose link closes. The LAST entry is the "best effort"
   fallback, so keep the longest-reach profile last.

   Each profile is a linear-FM chirp: centre frequency, bandwidth, duration.
   Whole band (fc +/- bw/2) must sit inside BAND_MIN_HZ..BAND_MAX_HZ
   (checked at boot by validateProfiles()).

   Range resolution = c / (2 * bandwidth):
     X-tier 120 kHz -> ~0.6 cm   H-tier 60 kHz -> ~1.3 cm
     M-tier  40 kHz -> ~1.9 cm   L-tier 40 kHz -> ~1.9 cm
   ======================================================================== */
// The profile table below is all LFM; the operator can switch every profile to another
// modulation with the serial command  M lfm | geo | bpsk | cw | auto  (auto = the table's).
//   GEOMETRIC: the frequency is multiplied by the same ratio every step (fc-bw/2 -> fc+bw/2);
//              tolerant of motion (Doppler).
//   BPSK:      a tone at fc, phase-flipped by the Barker-13 code; 13 chips fill the pulse.
//              Sharp and clean when nothing moves, but breaks down with motion.
enum Modulation : uint8_t { MOD_CW = 0, MOD_LFM_CHIRP = 1, MOD_GEOMETRIC = 2, MOD_BPSK = 3 };

struct WaveProfile {
  const char* name;
  float       fcHz;      // centre frequency
  float       bwHz;      // chirp bandwidth (sweep width)
  float       pulseS;    // pulse duration in seconds
  Modulation  mod;
};

static const WaveProfile PROFILES[] = {
  //  name    fc [Hz]   bw [Hz]   pulse [s]  modulation
  { "X-S",  440000.0f, 120000.0f, 0.001f, MOD_LFM_CHIRP },   //  0  380-500 kHz, 1 ms
  { "X-M",  440000.0f, 120000.0f, 0.005f, MOD_LFM_CHIRP },   //  1
  { "X-L",  440000.0f, 120000.0f, 0.020f, MOD_LFM_CHIRP },   //  2
  { "H-S",  300000.0f,  60000.0f, 0.001f, MOD_LFM_CHIRP },   //  3  270-330 kHz
  { "H-M",  300000.0f,  60000.0f, 0.005f, MOD_LFM_CHIRP },   //  4
  { "H-L",  300000.0f,  60000.0f, 0.020f, MOD_LFM_CHIRP },   //  5
  { "M-S",  200000.0f,  40000.0f, 0.001f, MOD_LFM_CHIRP },   //  6  180-220 kHz
  { "M-M",  200000.0f,  40000.0f, 0.005f, MOD_LFM_CHIRP },   //  7
  { "M-L",  200000.0f,  40000.0f, 0.020f, MOD_LFM_CHIRP },   //  8
  { "L-S",  120000.0f,  40000.0f, 0.001f, MOD_LFM_CHIRP },   //  9  100-140 kHz
  { "L-M",  120000.0f,  40000.0f, 0.005f, MOD_LFM_CHIRP },   // 10
  { "L-L",  120000.0f,  40000.0f, 0.020f, MOD_LFM_CHIRP },   // 11  <- best-effort fallback
};
#define NUM_PROFILES ((int)(sizeof(PROFILES) / sizeof(PROFILES[0])))
