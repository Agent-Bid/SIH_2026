/*
 * Real-speed recorder for the sonar DAC: the BlackPill's ADC samples the filtered DAC output at
 * 2.4 MS/s, so the real 100-500 kHz pulse can be seen on the laptop without slowing it down
 * (host/real_speed.py plots it against the FPGA's own samples).
 *
 * Wiring:
 *   PA1  <- the OPA340's output (LASC board J6 pin 7, channel 2 / U3); P 0 switches to PA0 (J6 pin 6)
 *   PB0  <- FPGA pin 41 (high for as long as each pulse lasts)
 *   GND  <- FPGA GND
 *
 * The ADC runs continuously into a circular buffer through DMA. A rising edge on PB0 marks where
 * the pulse starts; the recording keeps `pre` samples from before it and stops after the rest.
 *
 * Commands over USB (one per line):
 *   C [n] [pre]          the next pulse: n samples (default 4800 = 2 ms), pre of them before it
 *   S [k] [n] [pre]      the next k consecutive pulses (default 6), each sent as soon as it is in
 *   P <0|1>              record PA0 or PA1
 *   T <0..7>             ADC sampling time: 3, 15, 28, 56, 84, 112, 144, 480 ADC clocks (default 0 = 3,
 *                        2.4 MS/s; longer is slower but gives the input more time to settle)
 *   ?                    status
 * Each recording is a text line "CAP i=.. n=.. pre=.. rate=.. ch=.. t_us=..", then n
 * little-endian 16-bit samples (12-bit ADC counts), then "\nEND\n". A sequence ends with "SEQEND".
 */
#include <Arduino.h>

#define BUF_N     40000u                // 80 KB of RAM: 16.6 ms at 2.4 MS/s
#define TRIG_PIN  PB0

static uint16_t buf[BUF_N] __attribute__((aligned(4)));
static volatile int32_t trigAt = -1;    // buffer index at the trigger, -1 = not yet
static volatile bool armed = false;
static uint8_t channel = 1;
static uint8_t smp = 0;                 // sampling time code, see the T command
static float rate = 0;                  // measured samples per second

// 72 MHz from the 25 MHz crystal (USB still gets its 48 MHz); APB2 = 72 MHz, ADC = APB2 / 2 = 36 MHz
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {};
  RCC_ClkInitTypeDef clk = {};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 25;
  osc.PLL.PLLN = 144;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static inline uint32_t writePos() { return BUF_N - DMA2_Stream0->NDTR; }

static void adcStop() {
  ADC1->CR2 &= ~(ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA);
  DMA2_Stream0->CR &= ~DMA_SxCR_EN;
  while (DMA2_Stream0->CR & DMA_SxCR_EN) {}
}

static void setSampling(uint8_t code) {
  smp = code & 7;
  uint32_t r = 0;
  for (int ch = 0; ch < 10; ch++) r |= (uint32_t)smp << (3 * ch);
  ADC1->SMPR2 = r;
}

static void adcInit() {
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= (3u << 0) | (3u << 2);                  // PA0, PA1 analog
  ADC->CCR = ADC->CCR & ~ADC_CCR_ADCPRE;                   // ADC clock = PCLK2 / 2
  ADC1->CR1 = 0;                                           // 12 bits
  setSampling(0);                                          // 3-cycle sampling: 15 clocks a sample
  ADC1->SQR1 = 0;                                          // one channel
}

// ADC1 -> DMA2 stream 0 (channel 0), circular over the whole buffer
static void adcStart() {
  adcStop();
  DMA2->LIFCR = 0x3D;                                      // clear stream 0's flags
  DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;
  DMA2_Stream0->M0AR = (uint32_t)buf;
  DMA2_Stream0->NDTR = BUF_N;
  DMA2_Stream0->FCR = 0;
  DMA2_Stream0->CR = DMA_SxCR_PL | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_CIRC;
  DMA2_Stream0->CR |= DMA_SxCR_EN;
  ADC1->SR = 0;
  ADC1->SQR3 = channel;
  ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS;
  delayMicroseconds(5);                                    // the ADC's start-up time
  ADC1->CR2 |= ADC_CR2_SWSTART;
}

static void measureRate() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  adcStart();
  delay(2);
  const uint32_t c0 = DWT->CYCCNT, p0 = writePos();
  while (DWT->CYCCNT - c0 < SystemCoreClock / 100) {}     // 10 ms, fewer samples than the buffer
  const uint32_t c1 = DWT->CYCCNT, p1 = writePos();
  adcStop();
  rate = (float)((p1 + BUF_N - p0) % BUF_N) * SystemCoreClock / (c1 - c0);
}

static void onTrigger() {
  if (armed) {
    trigAt = writePos();
    armed = false;
  }
}

// Wait for the next pulse and record around it. false if none comes within timeoutMs.
static bool record(uint32_t n, uint32_t pre, uint32_t timeoutMs, uint32_t &tUs) {
  adcStart();
  delayMicroseconds((uint32_t)((uint64_t)pre * 1000000u / (uint32_t)rate) + 20);   // the samples before it
  trigAt = -1;
  armed = true;
  const uint32_t t0 = millis();
  while (trigAt < 0) {
    if (millis() - t0 > timeoutMs) {
      armed = false;
      adcStop();
      return false;
    }
  }
  tUs = micros();
  const uint32_t start = (uint32_t)trigAt, after = n - pre;
  while ((writePos() + BUF_N - start) % BUF_N < after) {}
  adcStop();
  return true;
}

static void send(uint32_t index, uint32_t n, uint32_t pre, uint32_t tUs) {
  Serial.printf("CAP i=%lu n=%lu pre=%lu rate=%lu ch=%u t_us=%lu\n", (unsigned long)index,
                (unsigned long)n, (unsigned long)pre, (unsigned long)lroundf(rate), channel, (unsigned long)tUs);
  const uint32_t first = ((uint32_t)trigAt + BUF_N - pre) % BUF_N;
  const uint32_t part = min(n, BUF_N - first);
  Serial.write((const uint8_t *)&buf[first], part * 2);
  if (part < n) Serial.write((const uint8_t *)buf, (n - part) * 2);
  Serial.print("\nEND\n");
}

static void clampSizes(uint32_t &n, uint32_t &pre) {
  if (n < 16) n = 4800;
  if (n > BUF_N - 2000) n = BUF_N - 2000;                   // room for the time it takes to stop
  if (pre >= n) pre = n / 10;
}

static void handle(char *line) {
  char *p = line + 1;
  const char c = line[0];
  if (c == 'C' || c == 'c' || c == 'S' || c == 's') {
    uint32_t k = 1;
    if (c == 'S' || c == 's') {
      k = strtoul(p, &p, 10);
      if (k < 1) k = 6;
    }
    uint32_t n = strtoul(p, &p, 10), pre = strtoul(p, &p, 10);
    if (pre == 0 && n == 0) pre = 480;
    clampSizes(n, pre);
    for (uint32_t i = 0; i < k; i++) {
      uint32_t tUs;
      if (!record(n, pre, 2000, tUs)) {
        Serial.print("CAPERR no pulse on PB0 within 2 s (is FPGA pin 41 wired, and is the ESP32 sending?)\n");
        return;
      }
      send(i, n, pre, tUs);
    }
    if (c == 'S' || c == 's') Serial.print("SEQEND\n");
  } else if (c == 'T' || c == 't') {
    static const uint16_t CYCLES[8] = {3, 15, 28, 56, 84, 112, 144, 480};
    setSampling(strtoul(p, nullptr, 10));
    measureRate();
    Serial.printf("sampling %u ADC clocks, rate=%lu\n", CYCLES[smp], (unsigned long)lroundf(rate));
  } else if (c == 'P' || c == 'p') {
    channel = strtoul(p, nullptr, 10) ? 1 : 0;
    Serial.printf("recording PA%u\n", channel);
  } else {
    Serial.printf("BPADC rate=%lu ch=PA%u buf=%u clk=%lu\n", (unsigned long)lroundf(rate), channel, BUF_N, (unsigned long)SystemCoreClock);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, INPUT_PULLDOWN);
  adcInit();
  measureRate();
  attachInterrupt(digitalPinToInterrupt(TRIG_PIN), onTrigger, RISING);
}

void loop() {
  static char line[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = 0;
        handle(line);
        len = 0;
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
