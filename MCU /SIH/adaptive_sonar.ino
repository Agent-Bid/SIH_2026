/*
  ============================================================================
  Adaptive Sonar Parameter Engine  --  ESP32-S3 (N16R8) + FreeRTOS
  ============================================================================
  Takes the conditions (by default from a simulation of changing water conditions,
  water_sim.h; or typed on the serial console, or 5 potentiometers), runs the physics
  model and the neural networks, picks a waveform for every ping (profile 100-500 kHz,
  amplitude, and LFM / geometric / BPSK from the motion), 6 pings a second, and sends
  it to the FPGA over SPI.

  Files:  sonar_config.h  = every tunable number + the profile table
          sonar_core.h    = physics, decision logic, FPGA packet (PC-testable)
          this file       = hardware, FreeRTOS tasks, SPI, serial
  See README_adaptive_sonar.md for the plain-language walkthrough.

  Arduino IDE board settings (ESP32S3 Dev Module):
    Flash size 16MB | PSRAM "Disabled" (not needed) | USB CDC On Boot: Enabled
    (if you use the native USB port for Serial)

  Serial commands (115200 baud; host/esp_console.py in the FPGA project is a terminal that
  does not reset the board):
      A          -> automatic: simulated water conditions (the default); prints one PING
                    line per ping
      C [amp]    -> DAC view: records the DAC filter output on GPIO 1 during a slow-motion copy
                    of the current ping, optionally at another amplitude 0..1 (host/dac_view.py
                    in the FPGA project plots it)
      Enter or W -> type new inputs: it asks for each value, then prints the decision
      I 15 35 50 20 100 300 0.5 -> all seven inputs on one line (temperature, salinity, depth,
                    turbidity, battery, range, speed); the report ends with a DECISION line
                    for programs (host/demo.py)
      R 450      -> set target range to 450 m
      M geo      -> modulation: auto | lfm | geo | bpsk | cw
      P          -> inputs from the potentiometers <-> typed inputs
      L          -> live status line on/off
      ?          -> help
  ============================================================================
*/
#include <Arduino.h>
#include <SPI.h>
#include "sonar_config.h"
#include "sonar_core.h"
#include "water_sim.h"
#if __has_include("esp_adc/adc_continuous.h")
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#define HAVE_ADC_CAPTURE 1
#else
#define HAVE_ADC_CAPTURE 0
#endif

// 1 = the ML model picks the profile, its amplitude and the modulation (ml_model.h, see ml/); the
//     physics keeps the profile when it closes the link within the energy allowance and overrides
//     it otherwise (FLAG_ML_USED / FLAG_ML_OVERRIDDEN), raises the amplitude if it is too low, and
//     replaces a modulation that cannot take the motion (FLAG_ML_MOD_OVERRIDDEN)
// 0 = physics only
#ifndef USE_ML_MODEL
#define USE_ML_MODEL 1
#endif

/* ------------------------------------------------------------ board pins */
// Pots on ADC1 pins (ADC2 is unusable when WiFi is on, so we stay on ADC1).
// Avoid GPIO 26-37 (flash/PSRAM on N16R8), 19/20 (USB), 0/3/45/46 (strapping).
#define PIN_POT_TEMP        1
#define PIN_POT_SALINITY    2
#define PIN_POT_DEPTH       3
#define PIN_POT_TURBIDITY   4
#define PIN_POT_BATTERY     5

// SPI to the FPGA (Tang Nano pins in brackets). The SPI signals go through the GPIO matrix,
// so any GPIO works; 2 MHz is far below its limit.
#define PIN_FPGA_CS         14    // -> FPGA pin 36
#define PIN_FPGA_MOSI       12    // -> FPGA pin 37
#define PIN_FPGA_SCK        13    // -> FPGA pin 38
#define PIN_FPGA_MISO       11    // -> FPGA pin 39 (not driven by the FPGA; unused)
#define FPGA_SPI_HZ         2000000UL     // [ASSUMPTION] confirm with FPGA owner
#define FPGA_SPI_MODE       SPI_MODE0     // [ASSUMPTION] confirm with FPGA owner

#define POT_FULLSCALE_MV    3100          // ESP32-S3 ADC @11 dB tops out near 3.1 V
#define ADC_OVERSAMPLE      16

/* ------------------------------------------------------------- RTOS setup */
#define SENSOR_PERIOD_MS    20            // 50 Hz sensor sampling
// decisions: PING_RATE_HZ (sonar_config.h), one per ping
#define SPI_PERIOD_MS       20            // 50 Hz packet refresh to FPGA (a new decision reaches it fast)
#define LOG_PERIOD_MS       500
#define SENSOR_STALE_MS     500           // decision refuses data older than this
#define SENSOR_FILTER_ALPHA 0.2f          // EMA smoothing of pot noise

// priorities (higher = more urgent): SPI > decision > sensor > console
#define PRIO_SPI            4
#define PRIO_DECISION       3
#define PRIO_SENSOR         2
#define PRIO_CONSOLE        1
#define CORE_A              0             // SPI + console
#define CORE_B              1             // sensor + decision

/* ----------------------------------------------------------- shared data */
// version: bumped by every typed change, so the decision starts fresh and the console can wait for it
struct SensorSample { sonar::Env env; uint32_t stampMs; uint32_t version; };
struct MlPick       { int profile; float amp; int mod; };   // the ML's own picks (-1 = none)
struct Telemetry    { sonar::Env env; sonar::Decision dec; sonar::SonarPacket pkt; uint32_t rangeM;
                      uint32_t staleCount; int32_t forcedMod; uint32_t version; MlPick ml;
                      int contact; float missionS; };           // contact: the simulator's, -1 = none

static QueueHandle_t qSensor = NULL;   // length 1, always holds the LATEST sample
static QueueHandle_t qPacket = NULL;   // length 1, always holds the LATEST packet
static QueueHandle_t qTelem  = NULL;   // length 1, latest telemetry for the console
static QueueHandle_t qPing   = NULL;   // every decision, for the PING log

static volatile uint32_t gTargetRangeM = DEFAULT_TARGET_RANGE_M;  // aligned 32-bit write is atomic
static volatile int32_t  gModulation   = -1;   // M command: -1 = auto (from the motion), else Modulation
static volatile bool     gSim          = true;   // A command: simulated water conditions (water_sim.h)
static volatile bool     gCapture      = false;  // C command running: the decision task leaves the packet alone
static volatile bool     gUsePots      = false;  // P command: potentiometers, else typed inputs
static volatile bool     gLive         = false;  // L command: status line every LOG_PERIOD_MS

// The typed inputs (the speed is typed in both modes: there is no sensor for it yet)
static portMUX_TYPE      gInputLock    = portMUX_INITIALIZER_UNLOCKED;
static sonar::Env        gTyped        = { 15.0f, 35.0f, 50.0f, 20.0f, 100.0f, DEFAULT_SPEED_MS };
static uint32_t          gInputVersion = 0;

static const char *const MOD_NAMES[] = { "cw", "lfm", "geo", "bpsk" };
static SPIClass spiFpga(FSPI);

#if USE_ML_MODEL
#include "ml_model.h"
// The ML model's choice (two small neural networks trained by ml/train.py on the physics'
// own decisions): the profile, and the amplitude for that profile. It always answers;
// sonar::decide() checks it with the physics.
static MlPick mlSelect(const sonar::Env &env, uint32_t rangeM) {
  sonar::Env e = env;
  sonar::sanitizeEnv(e);                              // the ranges the model was trained on
  const float r = sonar::clampf((float)rangeM, (float)MIN_TARGET_RANGE_M, (float)MAX_TARGET_RANGE_M);
  const float in[ml::ML_IN] = { e.tempC, e.salinityPpt, e.depthM, e.turbidityNtu, e.batteryPct, r };
  const int k = ml::ml_predict_profile(in);
  return { k, ml::ml_predict_amplitude(in, k), -1 };
}
// The modulation for the profile the physics settled on
static int mlModulation(const sonar::Env &env, int profile) {
  sonar::Env e = env;
  sonar::sanitizeEnv(e);
  const float in[4] = { e.tempC, e.salinityPpt, e.depthM, e.speedMs };
  return ml::ml_predict_modulation(in, profile);
}
#endif

/* ------------------------------------------------------------- utilities */
static float readPotFraction(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) acc += analogReadMilliVolts(pin);
  const float mv = (float)acc / ADC_OVERSAMPLE;
  return sonar::clampf(mv / (float)POT_FULLSCALE_MV, 0.0f, 1.0f);
}
static float scaleToRange(float lo, float hi, float t) { return lo + t * (hi - lo); }

/* ================================================================ TASKS */

// 1) SENSOR TASK: fixed-rate ADC read + smoothing (or the typed inputs) -> latest sample queue.
static void sensorTask(void *) {
  sonar::Env f = {};
  bool first = true;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    portENTER_CRITICAL(&gInputLock);
    const sonar::Env typed = gTyped;
    const uint32_t version = gInputVersion;
    portEXIT_CRITICAL(&gInputLock);
    if (!gUsePots) {
      f = typed;                              // exact, no smoothing
      first = true;
      const SensorSample s = { f, millis(), version };
      xQueueOverwrite(qSensor, &s);
      vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
      continue;
    }
    sonar::Env raw;
    raw.tempC        = scaleToRange(TEMP_MIN_C,      TEMP_MAX_C,      readPotFraction(PIN_POT_TEMP));
    raw.salinityPpt  = scaleToRange(SALINITY_MIN_PPT, SALINITY_MAX_PPT, readPotFraction(PIN_POT_SALINITY));
    raw.depthM       = scaleToRange(DEPTH_MIN_M,     DEPTH_MAX_M,     readPotFraction(PIN_POT_DEPTH));
    raw.turbidityNtu = scaleToRange(TURBIDITY_MIN_NTU, TURBIDITY_MAX_NTU, readPotFraction(PIN_POT_TURBIDITY));
    raw.batteryPct   = scaleToRange(BATTERY_MIN_PCT, BATTERY_MAX_PCT, readPotFraction(PIN_POT_BATTERY));
    raw.speedMs      = typed.speedMs;

    if (first) { f = raw; first = false; }
    else {
      const float a = SENSOR_FILTER_ALPHA;
      f.tempC        += a * (raw.tempC        - f.tempC);
      f.salinityPpt  += a * (raw.salinityPpt  - f.salinityPpt);
      f.depthM       += a * (raw.depthM       - f.depthM);
      f.turbidityNtu += a * (raw.turbidityNtu - f.turbidityNtu);
      f.batteryPct   += a * (raw.batteryPct   - f.batteryPct);
      f.speedMs       = raw.speedMs;
    }
    const SensorSample s = { f, millis(), version };
    xQueueOverwrite(qSensor, &s);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// 2) DECISION TASK: physics + ML -> profile, amplitude, modulation -> packet.
//    Never touches SPI or Serial.
static void decisionTask(void *) {
  int prev[4] = { -1, -1, -1, -1 };    // anti flip-flop memory per target: the simulator's 3 contacts, or 1
  uint32_t lastVersion = 0;
  uint16_t seq = 0;
  uint32_t stale = 0;
  watersim::Sim sim;
  watersim::init(sim);
  const TickType_t t0 = xTaskGetTickCount();
  for (uint32_t k = 1;; k++) {
    sonar::Env env;
    uint32_t r = 0, version = 0;
    int who = -1;
    float missionS = 0.0f;
    bool fresh = false;
    if (gSim) {
      const watersim::Reading w = watersim::step(sim, 1.0f / PING_RATE_HZ);
      env = { w.tempC, w.salinityPpt, w.depthM, w.turbidityNtu, w.batteryPct, w.speedMs };
      r = (uint32_t)lroundf(w.rangeM);
      who = w.contact;
      missionS = w.missionS;
      version = gInputVersion;
      fresh = true;
    } else {
      SensorSample s;
      if (xQueuePeek(qSensor, &s, 0) == pdTRUE && (millis() - s.stampMs) <= SENSOR_STALE_MS) {
        env = s.env;
        r = gTargetRangeM;
        version = s.version;
        fresh = true;
      }
    }
    if (fresh) {
      if (version != lastVersion) {                       // new inputs: fresh start
        for (int &p : prev) p = -1;
        lastVersion = version;
      }
      int &last = prev[who >= 0 ? who : 3];
      MlPick ml = { -1, -1.0f, -1 };
#if USE_ML_MODEL
      ml = mlSelect(env, r);
#endif
      sonar::Decision d = sonar::decide(env, (float)r, last, ml.profile, ml.amp);
#if USE_ML_MODEL
      ml.mod = mlModulation(env, d.profile);
#endif
      sonar::chooseModulation(d, env, ml.mod);
      last = d.profile;
      const int32_t forced = gModulation;
      const sonar::SonarPacket pkt = sonar::buildPacket(d, seq, forced);
      if (!gCapture) xQueueOverwrite(qPacket, &pkt);
      const Telemetry t = { env, d, pkt, r, stale, forced, version, ml, who, missionS };
      xQueueOverwrite(qTelem, &t);
      xQueueSend(qPing, &t, 0);
      seq++;
    } else {
      stale++;                       // no fresh data: keep the last good packet going
    }
    // PING_RATE_HZ exactly on average (1000 / 6 ms is not a whole number of ticks)
    const TickType_t next = t0 + pdMS_TO_TICKS((uint64_t)k * 1000u / PING_RATE_HZ);
    const TickType_t now = xTaskGetTickCount();
    if ((int32_t)(next - now) > 0) vTaskDelay(next - now);
  }
}

// 3) SPI TASK: highest priority, fixed period, only job is to push the latest packet.
static void spiTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    sonar::SonarPacket pkt;
    if (xQueuePeek(qPacket, &pkt, 0) == pdTRUE) {
      spiFpga.beginTransaction(SPISettings(FPGA_SPI_HZ, MSBFIRST, FPGA_SPI_MODE));  // set the mode first,
      digitalWrite(PIN_FPGA_CS, LOW);                                                // then select the FPGA
      spiFpga.writeBytes((const uint8_t *)&pkt, sizeof(pkt));
      digitalWrite(PIN_FPGA_CS, HIGH);
      spiFpga.endTransaction();
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SPI_PERIOD_MS));
  }
}

// 4) CONSOLE TASK: lowest priority. Serial commands, the typed-input prompts, the decision report
//    and the optional live status line.

// Console output in small pieces, each sent before the next is queued: the USB-Serial/JTAG driver
// loses text when it is queued faster than the host collects it, and Serial.flush() can discard it
static void out(const char *fmt, ...) {
  static int emptyRoom = 0;                 // free space of the empty transmit buffer
  char buf[320];
  va_list a;
  va_start(a, fmt);
  const int n = vsnprintf(buf, sizeof buf, fmt, a);
  va_end(a);
  for (int i = 0; i < n && i < (int)sizeof buf - 1; i += 32) {
    Serial.write((const uint8_t *)buf + i, (n - i < 32) ? n - i : 32);
    for (int w = 0; w < 20; w++) {
      const int room = Serial.availableForWrite();
      if (room > emptyRoom) emptyRoom = room;
      if (room >= emptyRoom) break;
      vTaskDelay(1);
    }
    vTaskDelay(1);                          // the host picks it up from the chip
  }
}

// The values the W command asks for, in order
struct Field { const char *name; const char *unit; float lo, hi; };
static const Field FIELDS[] = {
  { "Temperature",    "C",   TEMP_MIN_C,                TEMP_MAX_C },
  { "Salinity",       "ppt", SALINITY_MIN_PPT,          SALINITY_MAX_PPT },
  { "Depth",          "m",   DEPTH_MIN_M,               DEPTH_MAX_M },
  { "Turbidity",      "NTU", TURBIDITY_MIN_NTU,         TURBIDITY_MAX_NTU },
  { "Battery",        "%",   BATTERY_MIN_PCT,           BATTERY_MAX_PCT },
  { "Target range",   "m",   (float)MIN_TARGET_RANGE_M, (float)MAX_TARGET_RANGE_M },
  { "Relative speed", "m/s", SPEED_MIN_MS,              SPEED_MAX_MS },
};
static const int NUM_FIELDS = (int)(sizeof(FIELDS) / sizeof(FIELDS[0]));

static float    gDraft[NUM_FIELDS];
static int      gAsk = -1;               // the field being asked for, -1 = not asking
static uint32_t gWaitVersion = 0;        // print the report of the decision with this input version
static uint32_t gWaitStartMs = 0;
static bool     gWaiting = false;
static bool     gMachine = false;        // the I command: end the report with a DECISION line

static void printPrompt() {
  if (!gSim) out("\nEnter = new inputs, ? = help > ");
}

static void printHelp() {
  out("\n  A            automatic: simulated water conditions, one PING line per ping\n");
  out("  C [amp]      DAC view: record GPIO 1 during a slow-motion copy of the current ping\n");
  out("  Enter or W   type new inputs (Enter keeps a value, q cancels)\n");
  out("  R <metres>   target range, e.g. R 450\n");
  out("  M <mod>      modulation: auto (from the motion) | lfm | geo | bpsk | cw\n");
  out("  P            inputs from the potentiometers <-> typed inputs\n");
  out("  L            live status line on/off\n");
}

// Apply a change and wait for the decision that uses it
static void inputsChanged() {
  portENTER_CRITICAL(&gInputLock);
  gWaitVersion = ++gInputVersion;
  portEXIT_CRITICAL(&gInputLock);
  gWaiting = true;
  gWaitStartMs = millis();
}

static void askField() {
  const Field &f = FIELDS[gAsk];
  out("%s (%g..%g %s) [%g]: ", f.name, f.lo, f.hi, f.unit, gDraft[gAsk]);
}

static void startAsking() {
  portENTER_CRITICAL(&gInputLock);
  const sonar::Env e = gTyped;
  portEXIT_CRITICAL(&gInputLock);
  const float now[NUM_FIELDS] = { e.tempC, e.salinityPpt, e.depthM, e.turbidityNtu, e.batteryPct,
                                  (float)gTargetRangeM, e.speedMs };
  memcpy(gDraft, now, sizeof(gDraft));
  out("%s\n", gUsePots ? "\nPotentiometers are on (P): only the range and the speed are used from here."
                          : "\nNew inputs (Enter keeps the value in brackets, q cancels):");
  gAsk = 0;
  askField();
}

static void finishAsking() {
  gAsk = -1;
  gSim = false;
  gMachine = false;
  portENTER_CRITICAL(&gInputLock);
  gTyped.tempC        = gDraft[0];
  gTyped.salinityPpt  = gDraft[1];
  gTyped.depthM       = gDraft[2];
  gTyped.turbidityNtu = gDraft[3];
  gTyped.batteryPct   = gDraft[4];
  gTyped.speedMs      = gDraft[6];
  gTargetRangeM       = (uint32_t)lroundf(gDraft[5]);
  portEXIT_CRITICAL(&gInputLock);
  inputsChanged();
}

static void answerField(const char *w) {
  if (*w == 'q' || *w == 'Q') { gAsk = -1; out("cancelled\n"); printPrompt(); return; }
  if (*w) {
    char *end;
    const float v = strtof(w, &end);
    const Field &f = FIELDS[gAsk];
    if (end == w || *end || !(v >= f.lo && v <= f.hi)) {
      out("  \"%s\": must be a number from %g to %g\n", w, f.lo, f.hi);
      askField();
      return;
    }
    gDraft[gAsk] = v;
  }
  if (++gAsk < NUM_FIELDS) askField();
  else finishAsking();
}

// DAC view (C command): records the DAC filter's output on GPIO 1 with the ESP32's ADC. The ADC
// (80 kS/s at most) is far too slow for 100-500 kHz, so the FPGA is sent a slow-motion copy of the
// current waveform: every frequency divided by N and the pulse N times longer (N up to 100), which it
// plays through the real sigma-delta and filter. host/dac_view.py plots it against the FPGA's samples.
#define DAC_VIEW_ADC_CHANNEL  0          // GPIO 1 = ADC1 channel 0
#define DAC_VIEW_MAX_SAMPLES  50000
#define DAC_VIEW_MAX_RATE     80000

static bool slowPacket(const Telemetry &t, float amp, sonar::SonarPacket &k, int &n) {
  WaveProfile p = sonar::withModulation(PROFILES[t.dec.profile], t.pkt.modulation);
  n = (int)fmin(100.0, floor((double)FPGA_LEN_MAX_CLK / (p.pulseS * FPGA_CLK_HZ * 1.02)));
  if (n < 2) return false;
  p.fcHz /= n;
  p.bwHz /= n;
  p.pulseS *= n;
  const sonar::FpgaWave w = sonar::fpgaWave(p);
  if (!w.fits) return false;
  k = t.pkt;
  k.lenClk    = w.lenClk;
  k.ftwStart  = w.ftwStart;
  k.ftwStep   = w.ftwStep;
  k.winStep   = w.winStep;
  k.code      = w.code;
  k.chipLen   = w.chipLen;
  k.periodClk = w.lenClk + w.lenClk / 2;
  if (amp > 0.0f) k.ampQ12 = (uint16_t)lroundf(sonar::clampf(amp, 0.0f, 1.0f) * 4096.0f);
  k.crc8      = sonar::crc8((const uint8_t *)&k, sizeof(k) - 1);
  return true;
}

static void captureDac(float amp) {
#if HAVE_ADC_CAPTURE
  Telemetry t;
  sonar::SonarPacket k;
  int n;
  if (xQueuePeek(qTelem, &t, 0) != pdTRUE) { out("CAPERR no decision yet\n"); return; }
  if (!slowPacket(t, amp, k, n)) { out("CAPERR this waveform does not fit in slow motion\n"); return; }
  const double window = 2.7 * k.lenClk / FPGA_CLK_HZ;        // a whole pulse fits, whatever its phase
  const uint32_t rate = (uint32_t)fmin((double)DAC_VIEW_MAX_RATE, DAC_VIEW_MAX_SAMPLES / window);
  const uint32_t want = (uint32_t)(rate * window);
  uint16_t *mv = (uint16_t *)malloc(want * sizeof(uint16_t));
  if (!mv) { out("CAPERR out of memory\n"); return; }

  gCapture = true;                       // the slow copy replaces the packet; the FPGA takes it when idle
  xQueueOverwrite(qPacket, &k);
  vTaskDelay(pdMS_TO_TICKS(60));

  adc_continuous_handle_t adc = NULL;
  adc_cali_handle_t cali = NULL;
  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = 8192;
  hc.conv_frame_size = 256;
  adc_digi_pattern_config_t pat = {};
  pat.atten = ADC_ATTEN_DB_12;
  pat.channel = DAC_VIEW_ADC_CHANNEL;
  pat.unit = ADC_UNIT_1;
  pat.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  adc_continuous_config_t cc = {};
  cc.pattern_num = 1;
  cc.adc_pattern = &pat;
  cc.sample_freq_hz = rate;
  cc.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cc.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  adc_cali_curve_fitting_config_t cf = {};
  cf.unit_id = ADC_UNIT_1;
  cf.chan = (adc_channel_t)DAC_VIEW_ADC_CHANNEL;
  cf.atten = ADC_ATTEN_DB_12;
  cf.bitwidth = ADC_BITWIDTH_12;
  bool ok = adc_continuous_new_handle(&hc, &adc) == ESP_OK && adc_continuous_config(adc, &cc) == ESP_OK
         && adc_cali_create_scheme_curve_fitting(&cf, &cali) == ESP_OK;
  const bool started = ok && adc_continuous_start(adc) == ESP_OK;
  ok = started;
  uint32_t got = 0;
  const int64_t t0 = esp_timer_get_time();
  uint8_t frame[256];
  while (ok && got < want) {
    uint32_t len = 0;
    if (adc_continuous_read(adc, frame, sizeof frame, &len, 1000) != ESP_OK) { ok = false; break; }
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= len && got < want; i += SOC_ADC_DIGI_RESULT_BYTES) {
      const adc_digi_output_data_t *d = (const adc_digi_output_data_t *)&frame[i];
      if (d->type2.channel != DAC_VIEW_ADC_CHANNEL) continue;
      int v = 0;
      adc_cali_raw_to_voltage(cali, d->type2.data, &v);
      mv[got++] = (uint16_t)v;
    }
  }
  const double secs = (esp_timer_get_time() - t0) / 1e6;
  if (started) adc_continuous_stop(adc);
  if (adc) adc_continuous_deinit(adc);
  if (cali) adc_cali_delete_scheme_curve_fitting(cali);
  gCapture = false;
  if (!ok) { free(mv); out("CAPERR ADC failed (was potentiometer mode used since power-on? reset the ESP32)\n"); return; }

  out("CAP n=%lu rate=%lu meas=%.1f slow=%d mod=%u len=%lu ftw_start=%lu ftw_step=%ld win_step=%lu amp=%u"
      " code=%u chip=%lu period=%lu profile=%s\n", (unsigned long)got, (unsigned long)rate, got / secs, n,
      k.modulation, (unsigned long)k.lenClk, (unsigned long)k.ftwStart, (long)k.ftwStep,
      (unsigned long)k.winStep, k.ampQ12, k.code, (unsigned long)k.chipLen, (unsigned long)k.periodClk,
      PROFILES[t.dec.profile].name);
  char line[4 + 3 * 32];
  for (uint32_t i = 0; i < got; i += 32) {
    int p = snprintf(line, sizeof line, "D ");
    for (uint32_t j = i; j < got && j < i + 32; j++) p += snprintf(line + p, sizeof line - p, "%03X", mv[j] & 0xFFF);
    out("%s\n", line);
  }
  out("END\n");
  free(mv);
#else
  (void)amp;
  out("CAPERR not supported on this build\n");
#endif
}

static void handleCommand(char *w) {
  const char c = *w;
  if (c == 0 && gSim) return;                              // Enter alone does not stop the simulation
  if (c == 0 || c == 'W' || c == 'w') { startAsking(); return; }
  if (c == 'C' || c == 'c') {
    captureDac(strtof(w + 1, nullptr));                     // no number: 0 = keep the ping's own amplitude
    return;
  }
  if (c == 'A' || c == 'a') {
    gSim = true;
    gAsk = -1;
    out("  automatic: simulated water conditions, %d pings per second\n", PING_RATE_HZ);
    inputsChanged();
    return;
  }
  w++;
  while (*w == ' ') w++;
  if (c == 'I' || c == 'i') {
    float v[NUM_FIELDS];
    const char *p = w;
    for (int i = 0; i < NUM_FIELDS; i++) {
      char *end;
      v[i] = strtof(p, &end);
      if (end == p || !(v[i] >= FIELDS[i].lo && v[i] <= FIELDS[i].hi)) {
        out("  ERROR input %d (%s) must be %g..%g %s\n", i + 1, FIELDS[i].name, FIELDS[i].lo, FIELDS[i].hi, FIELDS[i].unit);
        printPrompt();
        return;
      }
      p = end;
    }
    memcpy(gDraft, v, sizeof(gDraft));
    finishAsking();
    gMachine = true;
    return;
  }
  if (c == 'R' || c == 'r') {
    const long v = atol(w);
    if (v >= MIN_TARGET_RANGE_M && v <= MAX_TARGET_RANGE_M) {
      gTargetRangeM = (uint32_t)v;
      inputsChanged();
      return;
    }
    out("  range must be %d..%d m\n", MIN_TARGET_RANGE_M, MAX_TARGET_RANGE_M);
  } else if (c == 'M' || c == 'm') {
    int m = -2;
    if (!strcasecmp(w, "auto")) m = -1;
    for (int i = 0; i < 4; i++) if (!strcasecmp(w, MOD_NAMES[i])) m = i;
    if (m >= -1) {
      gModulation = m;
      inputsChanged();
      return;
    }
    out("  modulation must be one of: auto lfm geo bpsk cw\n");
  } else if (c == 'P' || c == 'p') {
    gUsePots = !gUsePots;
    gSim = false;
    out("%s\n", gUsePots ? "  inputs from the potentiometers (speed and range stay typed)" : "  typed inputs");
    inputsChanged();
    return;
  } else if (c == 'L' || c == 'l') {
    gLive = !gLive;
    out("%s\n", gLive ? "  live status on" : "  live status off");
  } else {
    printHelp();
  }
  printPrompt();
}

static void handleSerial() {
  static char buf[64];
  static uint8_t n = 0;
  static bool lastCR = false;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && lastCR) { lastCR = false; continue; }      // CR LF = one line end
    lastCR = (ch == '\r');
    if (ch == '\n' || ch == '\r') {
      buf[n] = 0;
      n = 0;
      char *w = buf;
      while (*w == ' ') w++;
      for (char *e = w + strlen(w); e > w && e[-1] == ' '; ) *--e = 0;
      if (gAsk >= 0) answerField(w);
      else if (!gWaiting) handleCommand(w);
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

static const char *modName(int m) { return (m >= 0 && m <= 3) ? MOD_NAMES[m] : "-"; }

static void printReport(const Telemetry &t) {
  const sonar::Decision &d = t.dec;
  const sonar::SonarPacket &k = t.pkt;
  const WaveProfile &p = PROFILES[d.profile];
  const int mod = k.modulation;
  const double clkUs = 1.0e6 / FPGA_CLK_HZ;
  out("\n-----------------------------------------------------------------------------\n");
  out(" Inputs     %.1f C, %.1f ppt, depth %.0f m, turbidity %.0f NTU, battery %.0f %%%s\n",
                t.env.tempC, t.env.salinityPpt, t.env.depthM, t.env.turbidityNtu, t.env.batteryPct,
                gUsePots ? " (potentiometers)" : "");
  out("            target range %lu m, relative speed %.2f m/s\n", (unsigned long)t.rangeM, t.env.speedMs);
  out(" Physics    sound speed %.1f m/s, absorption %.1f + turbidity %.1f dB/km at %.0f kHz,\n",
                d.soundSpeed, d.alphaWaterDbKm, d.alphaTurbDbKm, p.fcHz / 1000.0f);
  out("            loss %.1f dB, noise %.1f dB/Hz, Doppler drift %.2f cycles over the pulse\n",
                d.tlDb, d.n0Db, d.dopplerCycles);
#if USE_ML_MODEL
  out(" Neural network picks, checked by the physics:\n");
  if (d.flags & sonar::FLAG_ML_USED)
    out("   profile     %-5s kept: closes the link within the energy allowance\n", PROFILES[t.ml.profile].name);
  else
    out("   profile     %-5s replaced by %s: %s\n", t.ml.profile >= 0 ? PROFILES[t.ml.profile].name : "-",
                  p.name, (d.flags & sonar::FLAG_BEST_EFFORT) ? "no profile closes the link"
                                                               : "fails the link or costs more than allowed");
  if (!(d.flags & sonar::FLAG_ML_USED) || t.ml.amp < 0.0f)
    out("   amplitude   -     set by the physics: %.3f\n", d.ampFrac);
  else if (d.flags & sonar::FLAG_ML_AMP_RAISED)
    out("   amplitude   %.3f raised to %.3f: too low for the link\n", t.ml.amp, d.ampFrac);
  else if (t.ml.amp < d.ampFrac - 5e-4f)
    out("   amplitude   %.3f raised to %.3f: the minimum drive\n", t.ml.amp, d.ampFrac);
  else if (t.ml.amp > d.ampFrac + 5e-4f)
    out("   amplitude   %.3f lowered to %.3f: the most this ping's energy budget allows\n", t.ml.amp, d.ampFrac);
  else
    out("   amplitude   %.3f kept: enough for the link\n", t.ml.amp);
  if (t.forcedMod >= 0)
    out("   modulation  %-5s not used: forced to %s by the M command (M auto to undo)\n",
                  modName(t.ml.mod), modName(t.forcedMod));
  else if (d.flags & sonar::FLAG_ML_MOD_OVERRIDDEN)
    out("   modulation  %-5s replaced by %s: it cannot take %.2f cycles of Doppler drift\n",
                  modName(t.ml.mod), modName(d.mod), d.dopplerCycles);
  else
    out("   modulation  %-5s kept: it takes %.2f cycles of Doppler drift\n", modName(t.ml.mod), d.dopplerCycles);
#else
  out(" Physics only (USE_ML_MODEL 0)\n");
#endif
  out(" Result     %s %s, %.0f ms, amplitude %.3f (%.2f W), energy %.2f mJ\n", p.name, modName(mod),
                p.pulseS * 1000.0f, d.ampFrac, d.txPowerW, d.energyJ * 1000.0f);
  if (mod == MOD_BPSK)
    out("            %.0f kHz, Barker-13: 13 chips of %.1f us\n", p.fcHz / 1000.0f, k.chipLen * clkUs);
  else if (mod == MOD_CW)
    out("            %.0f kHz tone\n", p.fcHz / 1000.0f);
  else
    out("            sweep %.0f -> %.0f kHz\n", (p.fcHz - 0.5f * p.bwHz) / 1000.0f, (p.fcHz + 0.5f * p.bwHz) / 1000.0f);
  out("            SNR %.1f dB (needs %.1f), reaches %.0f m; energy: cheapest %.2f mJ, allowed %.2f mJ\n",
                d.snrOutDb, DETECTION_THRESHOLD_DB + LINK_MARGIN_DB, d.maxRangeM,
                d.energyMinJ * 1000.0f, d.energyAllowanceJ * 1000.0f);
  out("            %s%s%s\n", (d.flags & sonar::FLAG_LINK_OK) ? "LINK OK" : "LINK FAILS (best effort)",
                (d.flags & sonar::FLAG_LOW_BATTERY) ? ", LOW BATTERY" : "",
                (d.flags & sonar::FLAG_INPUT_CLAMPED) ? ", an input was clamped" : "");
  out(" To FPGA    len %lu clk, ftw_start %lu, ftw_step %ld, win_step %lu, amp %u/4096,\n",
                (unsigned long)k.lenClk, (unsigned long)k.ftwStart, (long)k.ftwStep, (unsigned long)k.winStep, k.ampQ12);
  out("            code 0x%04X, chip %lu clk, a pulse every %lu clk (%.0f ms); sent 10 times a second\n",
                k.code, (unsigned long)k.chipLen, (unsigned long)k.periodClk, k.periodClk * clkUs / 1000.0);
  out("-----------------------------------------------------------------------------\n");
  if (gMachine)
    out("DECISION profile=%s mod=%s amp=%.4f ml_profile=%s ml_mod=%s ml_amp=%.4f flags=0x%02X doppler=%.3f"
        " len=%lu ftw_start=%lu ftw_step=%ld win_step=%lu amp_q12=%u code=%u chip=%lu period=%lu\n",
        p.name, modName(mod), d.ampFrac, t.ml.profile >= 0 ? PROFILES[t.ml.profile].name : "-", modName(t.ml.mod),
        t.ml.amp, d.flags, d.dopplerCycles, (unsigned long)k.lenClk, (unsigned long)k.ftwStart, (long)k.ftwStep,
        (unsigned long)k.winStep, k.ampQ12, k.code, (unsigned long)k.chipLen, (unsigned long)k.periodClk);
}

static void printStatus(const Telemetry &t) {
  const WaveProfile &p = PROFILES[t.dec.profile];
  out("T=%.1fC S=%.1f D=%.0fm Turb=%.0f Bat=%.0f%% R=%lum v=%.2f | %s %s amp=%.3f E=%.2fmJ SNR=%.1f"
                " dop=%.2f | %s%s%s%s stale=%lu\n",
                t.env.tempC, t.env.salinityPpt, t.env.depthM, t.env.turbidityNtu, t.env.batteryPct,
                (unsigned long)t.rangeM, t.env.speedMs, p.name, modName(t.pkt.modulation), t.dec.ampFrac,
                t.dec.energyJ * 1000.0f, t.dec.snrOutDb, t.dec.dopplerCycles,
                (t.dec.flags & sonar::FLAG_LINK_OK) ? "LINK_OK " : "LINK_FAIL ",
                (t.dec.flags & sonar::FLAG_ML_USED) ? "ML " : (t.dec.flags & sonar::FLAG_ML_OVERRIDDEN) ? "ML_OVERRIDDEN " : "PHYSICS ",
                (t.dec.flags & sonar::FLAG_ML_AMP_RAISED) ? "AMP_RAISED " : "",
                (t.dec.flags & sonar::FLAG_ML_MOD_OVERRIDDEN) ? "MOD_OVERRIDDEN " : "",
                (unsigned long)t.staleCount);
}

// One line per ping in automatic mode, key=value, for people and for host/demo.py
static void printPing(const Telemetry &t) {
  const sonar::Decision &d = t.dec;
  out("PING seq=%u contact=%c t=%.1f T=%.2f S=%.2f D=%.1f turb=%.1f bat=%.1f R=%lu v=%.2f"
      " profile=%s mod=%s amp=%.3f ml_profile=%s ml_mod=%s ml_amp=%.3f flags=0x%02X doppler=%.2f\n",
      t.pkt.seq, t.contact >= 0 ? "ABC"[t.contact] : '-', t.missionS, t.env.tempC, t.env.salinityPpt,
      t.env.depthM, t.env.turbidityNtu, t.env.batteryPct, (unsigned long)t.rangeM, t.env.speedMs,
      PROFILES[d.profile].name, modName(t.pkt.modulation), d.ampFrac,
      t.ml.profile >= 0 ? PROFILES[t.ml.profile].name : "-", modName(t.ml.mod), t.ml.amp, d.flags, d.dopplerCycles);
}

static void consoleTask(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t tick = 0;
  for (;;) {
    handleSerial();
    Telemetry t;
    while (xQueueReceive(qPing, &t, 0) == pdTRUE)
      if (gSim && t.contact >= 0 && gAsk < 0) printPing(t);
    const bool have = xQueuePeek(qTelem, &t, 0) == pdTRUE;
    if (gWaiting && gSim) gWaiting = false;
    if (gWaiting) {
      if (have && t.version == gWaitVersion) {
        gWaiting = false;
        printReport(t);
        printPrompt();
      } else if (millis() - gWaitStartMs > 2000) {
        gWaiting = false;
        out("\n  no decision within 2 s (sensor data stale?)\n");
        printPrompt();
      }
    }
    tick += 50;
    if (tick >= LOG_PERIOD_MS) {
      tick = 0;
      if (gLive && have && gAsk < 0 && !gWaiting) printStatus(t);
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

/* ================================================================ SETUP */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Adaptive Sonar Engine (ESP32-S3 + FreeRTOS) ===");

  int bad = -1;
  if (!sonar::validateProfiles(&bad)) {
    Serial.printf("FATAL: profile %d is outside %.0f-%.0f kHz band. Fix sonar_config.h.\n",
                  bad, BAND_MIN_HZ / 1000.0f, BAND_MAX_HZ / 1000.0f);
    for (;;) delay(1000);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_FPGA_CS, OUTPUT);
  digitalWrite(PIN_FPGA_CS, HIGH);
  spiFpga.begin(PIN_FPGA_SCK, PIN_FPGA_MISO, PIN_FPGA_MOSI, -1);   // CS handled manually

  qSensor = xQueueCreate(1, sizeof(SensorSample));
  qPacket = xQueueCreate(1, sizeof(sonar::SonarPacket));
  qTelem  = xQueueCreate(1, sizeof(Telemetry));
  qPing   = xQueueCreate(8, sizeof(Telemetry));
  if (!qSensor || !qPacket || !qTelem || !qPing) {
    Serial.println("FATAL: queue allocation failed");
    for (;;) delay(1000);
  }

  xTaskCreatePinnedToCore(sensorTask,   "sensor",   3072, NULL, PRIO_SENSOR,   NULL, CORE_B);
  xTaskCreatePinnedToCore(decisionTask, "decision", 6144, NULL, PRIO_DECISION, NULL, CORE_B);
  xTaskCreatePinnedToCore(spiTask,      "spi",      3072, NULL, PRIO_SPI,      NULL, CORE_A);
  xTaskCreatePinnedToCore(consoleTask,  "console",  6144, NULL, PRIO_CONSOLE,  NULL, CORE_A);

  Serial.printf("Profiles: %d | band %.0f-%.0f kHz | %s\n", NUM_PROFILES, BAND_MIN_HZ / 1000.0f,
                BAND_MAX_HZ / 1000.0f, USE_ML_MODEL ? "neural networks + physics check" : "physics only");
  Serial.printf("Automatic: simulated water conditions, %d pings per second (? for commands)\n", PING_RATE_HZ);
  printPrompt();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // all work happens in the RTOS tasks
}
