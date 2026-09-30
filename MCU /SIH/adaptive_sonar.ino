/*
  ============================================================================
  Adaptive Sonar Parameter Engine  --  ESP32-S3 (N16R8) + FreeRTOS
  ============================================================================
  Reads 5 "sensors" (potentiometers), runs the physics model, picks a waveform
  profile (100-500 kHz), and sends the result to the FPGA over SPI.

  Files:  sonar_config.h  = every tunable number + the profile table
          sonar_core.h    = physics, decision logic, FPGA packet (PC-testable)
          this file       = hardware, FreeRTOS tasks, SPI, serial
  See README_adaptive_sonar.md for the plain-language walkthrough.

  Arduino IDE board settings (ESP32S3 Dev Module):
    Flash size 16MB | PSRAM "Disabled" (not needed) | USB CDC On Boot: Enabled
    (if you use the native USB port for Serial)

  Serial commands (115200 baud):   R 450   -> set target range to 450 m
                                   M geo   -> waveform: auto | lfm | geo | bpsk | cw
                                   ?       -> help
  ============================================================================
*/
#include <Arduino.h>
#include <SPI.h>
#include "sonar_config.h"
#include "sonar_core.h"

// 1 = the ML model picks the profile and its amplitude (ml_model.h, see ml/); the physics keeps
//     the pick when it closes the link within the energy allowance and overrides it otherwise
//     (FLAG_ML_USED / FLAG_ML_OVERRIDDEN), and raises the amplitude if it is too low
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
#define DECISION_PERIOD_MS  100           // 10 Hz decisions
#define SPI_PERIOD_MS       100           // 10 Hz packet refresh to FPGA
#define LOG_PERIOD_MS       500
#define SENSOR_STALE_MS     500           // decision refuses data older than this
#define SENSOR_FILTER_ALPHA 0.2f          // EMA smoothing of pot noise

// priorities (higher = more urgent): SPI > decision > sensor > logger
#define PRIO_SPI            4
#define PRIO_DECISION       3
#define PRIO_SENSOR         2
#define PRIO_LOGGER         1
#define CORE_A              0             // SPI + logger
#define CORE_B              1             // sensor + decision

/* ----------------------------------------------------------- shared data */
struct SensorSample { sonar::Env env; uint32_t stampMs; };
struct Telemetry    { sonar::Env env; sonar::Decision dec; uint32_t rangeM; uint16_t seq; uint32_t staleCount; int32_t mod; };

static QueueHandle_t qSensor = NULL;   // length 1, always holds the LATEST sample
static QueueHandle_t qPacket = NULL;   // length 1, always holds the LATEST packet
static QueueHandle_t qTelem  = NULL;   // length 1, latest telemetry for the logger

static volatile uint32_t gTargetRangeM = DEFAULT_TARGET_RANGE_M;  // aligned 32-bit write is atomic
static volatile int32_t  gModulation   = -1;   // M command: -1 = auto (the profile's own), else Modulation

static const char *const MOD_NAMES[] = { "cw", "lfm", "geo", "bpsk" };
static SPIClass spiFpga(FSPI);

#if USE_ML_MODEL
#include "ml_model.h"
// The ML model's choice (two small neural networks trained by ml/train.py on the physics'
// own decisions): the profile, and the amplitude for that profile. It always answers;
// sonar::decide() checks it with the physics.
struct MlChoice { int profile; float amp; };
static MlChoice mlSelect(const sonar::Env &env, uint32_t rangeM) {
  sonar::Env e = env;
  sonar::sanitizeEnv(e);                              // the ranges the model was trained on
  const float r = sonar::clampf((float)rangeM, (float)MIN_TARGET_RANGE_M, (float)MAX_TARGET_RANGE_M);
  const float in[ml::ML_IN] = { e.tempC, e.salinityPpt, e.depthM, e.turbidityNtu, e.batteryPct, r };
  const int k = ml::ml_predict_profile(in);
  return { k, ml::ml_predict_amplitude(in, k) };
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

// 1) SENSOR TASK: fixed-rate ADC read + smoothing -> latest sample queue.
static void sensorTask(void *) {
  sonar::Env f = {};
  bool first = true;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    sonar::Env raw;
    raw.tempC        = scaleToRange(TEMP_MIN_C,      TEMP_MAX_C,      readPotFraction(PIN_POT_TEMP));
    raw.salinityPpt  = scaleToRange(SALINITY_MIN_PPT, SALINITY_MAX_PPT, readPotFraction(PIN_POT_SALINITY));
    raw.depthM       = scaleToRange(DEPTH_MIN_M,     DEPTH_MAX_M,     readPotFraction(PIN_POT_DEPTH));
    raw.turbidityNtu = scaleToRange(TURBIDITY_MIN_NTU, TURBIDITY_MAX_NTU, readPotFraction(PIN_POT_TURBIDITY));
    raw.batteryPct   = scaleToRange(BATTERY_MIN_PCT, BATTERY_MAX_PCT, readPotFraction(PIN_POT_BATTERY));

    if (first) { f = raw; first = false; }
    else {
      const float a = SENSOR_FILTER_ALPHA;
      f.tempC        += a * (raw.tempC        - f.tempC);
      f.salinityPpt  += a * (raw.salinityPpt  - f.salinityPpt);
      f.depthM       += a * (raw.depthM       - f.depthM);
      f.turbidityNtu += a * (raw.turbidityNtu - f.turbidityNtu);
      f.batteryPct   += a * (raw.batteryPct   - f.batteryPct);
    }
    SensorSample s = { f, millis() };
    xQueueOverwrite(qSensor, &s);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// 2) DECISION TASK: physics -> profile -> packet. Never touches SPI or Serial.
static void decisionTask(void *) {
  int prev = -1;
  uint16_t seq = 0;
  uint32_t stale = 0;
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    SensorSample s;
    if (xQueuePeek(qSensor, &s, 0) == pdTRUE && (millis() - s.stampMs) <= SENSOR_STALE_MS) {
      const uint32_t r = gTargetRangeM;
      int hint = -1;
      float hintAmp = -1.0f;
#if USE_ML_MODEL
      const MlChoice ml = mlSelect(s.env, r);
      hint = ml.profile;
      hintAmp = ml.amp;
#endif
      const sonar::Decision d = sonar::decide(s.env, (float)r, prev, hint, hintAmp);
      prev = d.profile;
      const sonar::SonarPacket pkt = sonar::buildPacket(d, seq, gModulation);
      xQueueOverwrite(qPacket, &pkt);
      Telemetry t = { s.env, d, r, seq, stale, gModulation };
      xQueueOverwrite(qTelem, &t);
      seq++;
    } else {
      stale++;                       // no fresh data: keep the last good packet going
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(DECISION_PERIOD_MS));
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

// 4) LOGGER TASK: lowest priority. Prints status and handles serial commands.
static void handleSerial() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      buf[n] = 0;
      if (n > 0) {
        if (buf[0] == 'R' || buf[0] == 'r') {
          const long v = atol(buf + 1);
          if (v >= MIN_TARGET_RANGE_M && v <= MAX_TARGET_RANGE_M) {
            gTargetRangeM = (uint32_t)v;
            Serial.printf(">> target range = %ld m\n", v);
          } else {
            Serial.printf(">> range must be %d..%d m\n", MIN_TARGET_RANGE_M, MAX_TARGET_RANGE_M);
          }
        } else if (buf[0] == 'M' || buf[0] == 'm') {
          const char *w = buf + 1;
          while (*w == ' ') w++;
          int m = -2;
          if (!strcasecmp(w, "auto")) m = -1;
          for (int i = 0; i < 4; i++) if (!strcasecmp(w, MOD_NAMES[i])) m = i;
          if (m >= -1) {
            gModulation = m;
            Serial.printf(">> waveform = %s\n", m < 0 ? "auto (profile table)" : MOD_NAMES[m]);
          } else {
            Serial.println(">> waveform must be one of: auto lfm geo bpsk cw");
          }
        } else {
          Serial.println(">> commands:  R <metres>   e.g.  R 450");
          Serial.println(">>            M <waveform> auto | lfm | geo | bpsk | cw");
        }
      }
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

static void loggerTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    handleSerial();
    static uint32_t tick = 0;
    tick += 50;
    if (tick >= LOG_PERIOD_MS) {
      tick = 0;
      Telemetry t;
      if (xQueuePeek(qTelem, &t, 0) == pdTRUE) {
        const WaveProfile &p = PROFILES[t.dec.profile];
        Serial.printf("T=%.1fC S=%.1f D=%.0fm Turb=%.0f Bat=%.0f%% R=%lum | c=%.1f aW=%.1f aT=%.1fdB/km TL=%.1f N0=%.1f"
                      " | %s %s fc=%.0fk bw=%.0fk T=%.0fms amp=%.2f P=%.2fW E=%.4fJ SNR=%.1f mrg=%+.1f reach=%.0fm"
                      " Emin=%.4fJ allow=%.4fJ | %s%s%s%s%s%s stale=%lu\n",
                      t.env.tempC, t.env.salinityPpt, t.env.depthM, t.env.turbidityNtu, t.env.batteryPct,
                      (unsigned long)t.rangeM, t.dec.soundSpeed, t.dec.alphaWaterDbKm, t.dec.alphaTurbDbKm,
                      t.dec.tlDb, t.dec.n0Db, p.name,
                      MOD_NAMES[t.mod >= 0 ? t.mod : (int)p.mod], p.fcHz / 1000.0f, p.bwHz / 1000.0f, p.pulseS * 1000.0f,
                      t.dec.ampFrac, t.dec.txPowerW, t.dec.energyJ, t.dec.snrOutDb, t.dec.marginDb, t.dec.maxRangeM,
                      t.dec.energyMinJ, t.dec.energyAllowanceJ,
                      (t.dec.flags & sonar::FLAG_LINK_OK)       ? "LINK_OK " : "LINK_FAIL ",
                      (t.dec.flags & sonar::FLAG_BEST_EFFORT)   ? "BEST_EFFORT " : "",
                      (t.dec.flags & sonar::FLAG_LOW_BATTERY)   ? "LOW_BAT " : "",
                      (t.dec.flags & sonar::FLAG_INPUT_CLAMPED) ? "CLAMPED " : "",
                      (t.dec.flags & sonar::FLAG_ML_USED)       ? "ML "
                      : (t.dec.flags & sonar::FLAG_ML_OVERRIDDEN) ? "ML_OVERRIDDEN " : "PHYSICS ",
                      (t.dec.flags & sonar::FLAG_ML_AMP_RAISED) ? "AMP_RAISED " : "",
                      (unsigned long)t.staleCount);
      }
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
  if (!qSensor || !qPacket || !qTelem) {
    Serial.println("FATAL: queue allocation failed");
    for (;;) delay(1000);
  }

  xTaskCreatePinnedToCore(sensorTask,   "sensor",   3072, NULL, PRIO_SENSOR,   NULL, CORE_B);
  xTaskCreatePinnedToCore(decisionTask, "decision", 6144, NULL, PRIO_DECISION, NULL, CORE_B);
  xTaskCreatePinnedToCore(spiTask,      "spi",      3072, NULL, PRIO_SPI,      NULL, CORE_A);
  xTaskCreatePinnedToCore(loggerTask,   "logger",   5120, NULL, PRIO_LOGGER,   NULL, CORE_A);

  Serial.printf("Profiles: %d | band %.0f-%.0f kHz | default range %d m | type '?' for commands\n",
                NUM_PROFILES, BAND_MIN_HZ / 1000.0f, BAND_MAX_HZ / 1000.0f, DEFAULT_TARGET_RANGE_M);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // all work happens in the RTOS tasks
}
