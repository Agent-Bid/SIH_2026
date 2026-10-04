# Adaptive sonar waveform generator: project guide for agents

This file describes the whole project: what it does, how the pieces fit, how to build,
test and run it, what has been verified, and what is still open. Read it before changing
anything. Keep it up to date when you change the design (see "Rules for changes" at the end).

Last updated: 2026-09-30 (simulated water conditions, 6 pings/s, Doppler-based modulation with a third
network, streamed captures at 3 Mbaud, `host/demo.py` verified on the boards).

---

## 1. What the system does

An underwater sonar transmitter that adapts its waveform to the water conditions.

```
 conditions ─► ESP32-S3 ───────────────── SPI (57-byte packet per ping) ──► Tang Nano 9K FPGA ──► 8-bit DAC ──► amplifier ──► transducer
 (simulated    neural networks pick the                                     generates the pulse
  for now)     waveform, physics checks it                                  sample by sample at 50.14 MHz
                                                     PC (USB UART) ──────►  (can stand in for the ESP32,
                                                                   ◄──────   and streams captured pulses back)
```

- **ESP32-S3 (`MCU /SIH/`)**: takes temperature, salinity, depth, turbidity, battery, target
  range and relative speed (by default from a built-in simulation of changing water
  conditions, `water_sim.h`), runs an underwater acoustics model and three small neural
  networks, and for every ping picks a profile (100–500 kHz, 1/5/20 ms), the drive amplitude
  (just enough to close the link) and the modulation (BPSK, LFM or geometric, from the Doppler
  drift the motion causes); the physics checks each pick. It converts everything into the
  FPGA's own units and sends a packet over SPI, 6 pings a second.
- **FPGA (the Verilog in this folder)**: turns the packet into windowed pulses. Four
  modulations: CW tone, LFM chirp, geometric (exponential) sweep, and BPSK with the Barker-13
  code. Every pulse gets a Hann window (to keep its spectrum tight) and an amplitude.
  It drives an 8-bit parallel DAC.
- **PC (`host/`)**: can send the same packet over the Tang Nano's USB UART (stands in for the
  ESP32), and can ask the FPGA to capture one pulse or every pulse and send it back; the samples
  are checked against a bit-exact Python golden model. `host/demo.py` shows one second of the
  running system: 6 pings on one graph with the conditions and choices behind each.

---

## 2. Current status (2026-09-30)

| Part | State |
|---|---|
| FPGA design | All 11 simulations pass (`sim/all`). Built: 49 % DFF, 18/26 BSRAM, 68 MHz (needs 50.14). **Written to the FPGA's flash (2026-09-30)**: it starts on power-up. |
| FPGA on the board, PC path (UART, 3 Mbaud) | **Verified**: chirp, geometric, BPSK and tone captured bit-exact against the golden model (16,383 samples each). |
| ESP32 → FPGA over SPI | **Verified on the boards (2026-09-30)**: one new packet per ping, 6 per second, 0 bad; framed **without CS** (see section 5): the CS wire (GPIO14 → pin 36) never carried a clean signal. |
| Whole system, `host/demo.py` | **Verified (2026-09-30)**: 6 consecutive pings (streamed captures, whole pulses), each a new ESP32 decision (packet numbers one apart), all bit-exact; the conditions recomputed on the PC match the ESP32's log. |
| ESP32 firmware v3 + ML | 43/43 host tests pass; **flashed and running** (native USB build, `CDCOnBoot=cdc`); starts in automatic mode (simulated conditions) at power-on, with or without a USB host. |
| Wiring ESP32 ↔ FPGA | Done (pin table in section 6). SCK, MOSI verified with an edge-counting probe; CS picks up only noise (cause not found: ESP32 pin, wire or FPGA pin). |
| DAC | The DAC0808 board is shelved. **In use: the 1-bit sigma-delta** on pin 40 → 1 kΩ + 100 pF → 10 kΩ + 10 pF → ESP32 GPIO 1. **Verified without a scope (2026-10-04)**: `host/dac_view.py` records the filtered output with the ESP32's ADC during a slow-motion copy of a ping; it matches the FPGA's samples (correlation 0.998 at full amplitude, BPSK phase flips visible). An 8-bit R-2R ladder on `dac_d[7:0]` (R = 1 kΩ, 25 resistors) is the cleaner next step. **Now (2026-10-05)**: RC → OPA340 Sallen-Key on the LASC board, a second-order sigma-delta, and a BlackPill recording at 2.4 MS/s (`host/real_speed.py`). Against first order on the same fixed pings: leftover noise 28 → 22, 16 → 13 and 17 → 12.5 mV rms (120, 300, 440 kHz); the 440 kHz amp-0.05 ping's match 0.87 → 0.93. The BlackPill's own floor is about 9–10 mV rms. |
| Sensors | Simulated (`MCU /SIH/water_sim.h`). No potentiometers wired: in pot mode (`P`) the ESP32's ADC inputs float. |
| ML model | Three networks in `MCU /SIH/ml_model.h` (profile 6-48-48-6, amplitude 12-48-48-1, modulation 16-32-32-3), `USE_ML_MODEL 1`. Profile agrees with the physics on 99.2–99.4 %, 0.5 % overridden; amplitude within 1.8 % (median), 1.6–3.4 % extra energy; modulation agrees on 99.7 %, 0.2 % overridden. **Running on the ESP32.** |
| Git | Committed. |

A board incident on 2026-09-29: with the ESP32 wired in, the FPGA's UART went silent and
JTAG read a wrong IDCODE (`0x0100581B`), so bitstreams would not load. A power cycle cleared
it; a loose jumper wire is the likely cause. If it comes back, suspect **pin 36** first: it is
`IOB29B`, the other half of the I/O pair whose `IOB29A` (pin 35) drives `dac_clk`. Moving
`spi_cs_n` to pin 40 (a free 3.3 V pin) builds fine.

---

## 3. Repository layout

Run everything from this folder (the project root): scripts and `$readmemh` use relative paths.

| Path | What it is |
|---|---|
| `top.v` | Board top: PLL, reset, buttons, SPI + UART packet input, settings double-buffer, pulse timer, engine, DAC output, capture, LEDs |
| `pulse_dds.v` | The waveform engine: one windowed, swept, phase-coded pulse per start |
| `pulse_ctrl.v` | `active` for `len` clocks after a start |
| `sweep.v` | Frequency sweep: LFM (add) or geometric (multiply, shift-and-add) on a 48-bit register |
| `phase_code.v` | BPSK chip counter: `flip = code[chip]` |
| `dds.v`, `phase_acc.v` | Phase accumulator + 1024-entry sine table (with a phase offset input for BPSK) |
| `scale.v` | Fixed-point multiply: `out = in × factor / 4096`, 2 clocks (split into two small products) |
| `sigma_delta.v` | 1-bit DAC of the waveform: second-order sigma-delta by default (input limited to ±95 %), `ORDER=1` for the original first-order one |
| `spi_rx.v` | SPI slave (mode 0) that collects one packet, framed by CS or (as used now) by the pause after each burst |
| `pkt_check.v` | Checks sync, version and CRC-8 of a packet, one byte per clock |
| `uart_rx.v`, `uart_tx.v`, `cmd_rx.v` | USB UART (3 Mbaud 8N1) and its command frames |
| `capture.v` | Records 2^14 samples of the next pulse (one in 2^shift, so the whole pulse fits) into BSRAM, then sends header + samples over the UART; stream mode re-arms after each |
| `pll.v`, `button.v` | 27 → 50.142857 MHz PLL; debounced buttons |
| `sine.hex`, `hann.hex` | ROM contents (made by `host/gen_sine.py`, `host/gen_window.py`) |
| `tangnano9k.cst` | Pin constraints |
| `tn9k.conf` | Build settings read by the `tn9k` script (`FREQ=50`, `SYNTH_FLAGS=-nodsp`) |
| `host/model.py` | Bit-exact golden model of `pulse_dds`, plus `geo_ftw` / `lfm_ftw` sweep tracks |
| `host/mcu_packet.py` | The ESP32 packet in Python: build, parse, CRC-8, unit conversion (same maths as the firmware) |
| `host/wavegen.py` | PC as the ESP32: sends a packet over the UART, optionally captures and checks the pulse |
| `host/capture.py` | Receives captures (board or simulation dump), checks them, plots waveform + spectrogram; `open_board()` opens the UART |
| `host/demo.py` | One second of the running system: streams 6 consecutive pings, checks them, plots them on one graph with a table of conditions and choices (`sim/out/demo.png`) |
| `host/water_sim.py` | Python twin of the ESP32's simulated conditions: the conditions behind a packet, from its sequence number |
| `host/esp_console.py` | Terminal for the ESP32's serial console that does not reset it |
| `host/dac_view.py` | Before vs after the DAC without a scope: ESP32 ADC recording of the filtered sigma-delta output (slow motion) against the golden model |
| `host/dac_sequence.py` | The same for one second of pings: the ESP32's next 6 decisions, each recorded 4× and averaged, on one broken-time-axis graph with close-ups and a conditions table (`--replot` redraws the saved recording) |
| `host/real_speed.py` | Pings at full speed after the DAC: the BlackPill (`blackpill/src/main.cpp`, ADC at 2.4 MS/s on PA1, trigger pin 41 on PB0) records 6 consecutive pulses while the FPGA streams their headers; each recording is paired with its pulse by correlation and plotted over the golden model, with no slow motion or averaging |
| `host/check_*.py` | Checkers for the unit testbenches; `check_water_sim.py` compares `water_sim.py` with the C++ |
| `sim/run`, `sim/all` | Run one testbench / all testbenches with their checkers |
| `sim/*_tb.v` | Testbenches (see section 9) |
| `sim/top_frames.hex`, `sim/top_spi.hex` | Stimulus for `top_tb`, made by `wavegen.py --frames-out / --spi-out` |
| `MCU /SIH/` | ESP32-S3 firmware (**the folder name ends in a space: quote paths**) — see section 8 |

Generated, git-ignored: `build/` (bitstream, logs), `sim/out/` (waves, sample logs, plots).

---

## 4. FPGA design

### Clock and reset
27 MHz crystal → `rPLL` (IDIV 7, FBDIV 13, ODIV 16, VCO 802 MHz) → **F_CLK = 50,142,857 Hz**.
Reset is held for 8 clocks after the PLL locks.

### Units
- **FTW** (frequency tuning word): `f × 2³² / F_CLK`. 1 Hz ≈ 85.65 FTW.
- Phase is a 32-bit fraction of a cycle; the sine table is indexed by `phase[31:22]`.
- Amplitudes and window values are fixed-point with **4096 = 1.0**.

### Signal chain (`pulse_dds.v`)
```
start ─► pulse_ctrl ─► active (len clocks);  run_rst = rst | ~active holds everything at its start value
          sweep ──ftw──► dds: phase += ftw; sample = sine[(phase + (flip ? 2^31 : 0))[31:22]]   (12-bit, ±2047)
          phase_code ──flip (BPSK: half a cycle = 180°)──┘
          window phase += win_step; win = hann[win_phase[31:22]]                              (13-bit, 0..4096)
          pipeline register (tone_r, win_r) ─► scale (× window) ─► scale (× amp) ─► out (12-bit signed)
top: dac_d = {~out[11], out[10:4]} (offset binary, top 8 bits); dac_clk = ~clk (DAC latches mid-sample)
     scope_sd = sigma_delta(out); scope_trig = active
```
- **LFM**: the sweep register holds FTW × 2¹⁶ (48 bits); every clock it adds the signed 32-bit
  `ftw_step` (FTW per clock × 2¹⁶). The fraction bits let long, narrow chirps sweep exactly.
- **Geometric**: every 32 clocks the register grows by `register × ftw_step / 2³²` (rounded down),
  i.e. the frequency is multiplied by `1 + ftw_step / 2³²`. The product is computed by a
  shift-and-add multiplier, one bit of `ftw_step` per clock, so no wide multiplier is needed.
- **BPSK**: `phase_code` advances a 4-bit chip counter every `chip_len` clocks; `flip = code[chip]`.
  Barker-13 = `0x0A60` (chips `+ + + + + - - + + - + - +`, bit i = chip i, 1 = flip).
- **Window**: a second phase accumulator makes exactly one lap per pulse (`win_step = 2³² / len`).
- The pipeline register before the first multiply is needed for timing (multipliers are built
  from LUTs, see section 7). Each `scale` takes 2 clocks: `factor` is split into its low 7 and
  high 6 bits, both partial products are registered, then added and shifted. `host/model.py`
  mirrors this delay exactly.

### Limits
| Quantity | Limit |
|---|---|
| Pulse length `len` | 24 bits: up to 16,777,215 clocks ≈ 335 ms |
| Period between pulse starts | 32 bits of clocks (≈ 85 s) |
| LFM sweep rate | `ftw_step` < 2³¹ → up to ~19 GHz/s (the sonar profiles need ≤ 0.12 GHz/s) |
| Chips per code | 16 (Barker-13 used); `chip_len` 24 bits |
| Frequency | keep ≤ 0.4 × F_CLK ≈ 20 MHz numerically; the sonar band is 100–500 kHz |
| Amplitude | 0..4096 (larger values are clamped to 4096) |

### Settings path (`top.v`)
1. A packet arrives from the ESP32 (`spi_rx`) or from the PC inside a UART frame (`cmd_rx`).
2. `pkt_check` verifies sync `0xA5`, version 3 and the CRC-8; good packets load `pending`.
3. `pending` is copied to `cur` only while no pulse is playing (`!active && !start`), so a
   pulse is never changed halfway through (double buffering).
4. A 32-bit timer fires `start` every `period` clocks.

Power-up settings and **S1** presets (13,000-clock pulses every 25,000 clocks):
LFM chirp 150 → 500 kHz, geometric sweep 150 → 500 kHz, BPSK Barker-13 at 300 kHz (1,000
clocks per chip). A connected ESP32 replaces them within 20 ms.

The packet's `seq`, `profileId` and `flags` travel with its settings (`cur_tag`), and a 16-bit
counter numbers the pulses, so every capture says which packet and which pulse it holds.

### Capture (`capture.v`)
Armed by **S2** or the UART command `A5 02 02`; `A5 03 03` turns on **stream mode** (re-armed
after every capture, so every pulse that starts after the previous capture has been sent is
captured) and `A5 04 04` turns it off. Records 2^14 = 16,384 samples from the next pulse start,
keeping **one sample in 2^shift**, where `shift` is the smallest that fits the whole pulse
(`fit_shift` in `top.v`: 0 up to 327 µs, 2 for 1 ms pulses, 4 for 5 ms, 6 for 20 ms), then
sends a 43-byte header and the samples as little-endian int16 over the UART (~0.11 s at 3
Mbaud: 6 pings a second fit). The header is frozen at the pulse start, so it always describes
the captured pulse. At one sample in 64 (20 ms pulses) the capture rate is 783 kS/s: fine for
the L profiles (100–140 kHz) that use 20 ms, not for an X profile at 20 ms (never chosen).

Header (little-endian): `A5 5A`, `{shift[3:0], 00, mod[1:0]}` u8, `N` u16, `len` u32,
`ftw_start` u32, `ftw_step` u32, `win_step` u32, `code` u16, `chip_len` u32, `amp` u16,
`period` u32, good SPI packets u16, bad SPI packets u16, packet `seq` u16, pulse number u16,
packet `profileId` u8, packet `flags` u8 (`HDR = "<BHIIIIHIHIHHHHBB"` after the two sync bytes
in `host/capture.py`).

The buffer is 16 BSRAM banks of 1024 × 12 bits (see section 7 for why).

---

## 5. The packet (SonarPacket v3, 57 bytes, little-endian)

Defined in `MCU /SIH/sonar_core.h` and mirrored in `host/mcu_packet.py`. The FPGA uses
byte 5 and bytes 28–55; the rest is telemetry.

| Bytes | Field | Meaning |
|---|---|---|
| 0 | `sync` | `0xA5` |
| 1 | `version` | 3 |
| 2–3 | `seq` | increments per new decision |
| 4 | `profileId` | 0–11 (0xFF from the PC) |
| 5 | `modulation` | **FPGA**: 0 CW, 1 LFM, 2 geometric, 3 BPSK |
| 6 | `spreadingFactor` | round(log2(bw × T)) for sweeps |
| 7 | `flags` | bit0 link OK, bit1 low battery, bit2 input clamped, bit3 ML profile used, bit4 best effort, bit5 ML profile overridden, bit6 ML amplitude raised, bit7 ML modulation overridden |
| 8–27 | `centerFreqHz`, `bandwidthHz`, `pulseUs`, `amplitude` (0–65535), `txPowerMilliW`, `propDelayUs` | telemetry |
| 28–31 | `lenClk` | **FPGA**: pulse length, clocks (BPSK: 13 × `chipLen`) |
| 32–35 | `ftwStart` | **FPGA**: start FTW (fc for CW/BPSK, fc − bw/2 for sweeps) |
| 36–39 | `ftwStep` | **FPGA**: LFM: FTW per clock × 2¹⁶ (signed). Geometric: (ratio − 1) × 2³² per 32 clocks, ratio = (f_end / f_start)^(1 / ⌊lenClk / 32⌋). Else 0 |
| 40–43 | `winStep` | **FPGA**: 2³² / lenClk |
| 44–45 | `ampQ12` | **FPGA**: 4096 = 1.0 |
| 46–47 | `code` | **FPGA**: BPSK code (0x0A60), else 0 |
| 48–51 | `chipLen` | **FPGA**: BPSK clocks per chip, else 0 |
| 52–55 | `periodClk` | **FPGA**: clocks between pulse starts (1/6 s minimum = `PING_PERIOD_MIN_S`; + round trip in echo mode) |
| 56 | `crc8` | CRC-8, poly 0x07, init 0, over bytes 0–55 (check value of "123456789" = 0xF4) |

**SPI**: mode 0 (SCK idles low, sample on rising edge), MSB first, 2 MHz. The ESP32 sends the
latest packet as one continuous 57-byte burst (228 µs) every 20 ms, with CS low around it.
**Framing on the current boards: without CS** (`spi_rx` parameter `USE_CS = 0` in `top.v`): a
frame ends when SCK has been quiet for 100 µs (`GAP` = 5000 clocks). This was needed because
the CS connection (ESP32 GPIO14 → FPGA pin 36) only ever showed noise on the FPGA side (an
edge-counting probe saw 26–201 random edges/s instead of 10/s), while SCK (exactly 4,560 edges/s)
and MOSI were clean. If CS is ever fixed, set `USE_CS(1)` to frame by CS again. Either way, any
byte count other than 57, a bad CRC, a wrong sync or version → rejected and counted.

**UART (PC)**: 3 Mbaud 8N1 on the Tang Nano's second FTDI channel (the FPGA runs at
F_CLK / 17 = 2.95 Mbaud, 1.7 % from the PC's 3 Mbaud, within UART tolerance).
- `A5 01 <57-byte packet> <xor of 01 and the 57 bytes>`: set the waveform.
- `A5 02 02`: capture the next pulse. `A5 03 03` / `A5 04 04`: stream mode on / off.
- A frame whose bytes stop arriving for 1 ms is dropped.
- **Open the port twice**: the first open after the board is plugged in or loaded does not take
  the 3 Mbaud setting and nothing gets through; `host/capture.py:open_board()` opens and closes
  it once first.

---

## 6. Hardware

**Board**: Sipeed Tang Nano 9K, GW1NR-LV9QN88PC6/I5 (GW1NR-9C). IO banks: bank 1 (pins 48–77)
and bank 2 (17–47) are 3.3 V; **bank 3 (3–16, 79–88) is 1.8 V** — never connect 3.3 V signals
there. Pins 59–62 are the on-board flash.

| Signal | FPGA pin | Other end |
|---|---|---|
| `clk` | 52 | 27 MHz oscillator |
| `btn1` / `btn2` | 3 / 4 | S1 (presets) / S2 (capture), active low |
| `led[5:0]` | 10, 11, 13, 14, 15, 16 | on-board LEDs, active low |
| `uart_tx` / `uart_rx` | 17 / 18 | USB UART (`/dev/ttyUSB1`) |
| `dac_d[7:0]` | 25, 26, 27, 28, 29, 30, 33, 34 | DAC data (offset binary) |
| `dac_clk` | 35 | DAC clock (= inverted system clock) |
| `spi_cs_n` | 36 (pull-up) | ESP32 GPIO14 |
| `spi_mosi` | 37 | ESP32 GPIO12 |
| `spi_sck` | 38 (pull-down) | ESP32 GPIO13 |
| (unused) | 39 | ESP32 GPIO11 (MISO) |
| `scope_sd` | 40 | 1 kΩ + 100 pF, then 10 kΩ + 10 pF (two RC stages) → ESP32 GPIO 1 (ADC) and/or a scope probe |
| `scope_trig` | 41 | oscilloscope trigger: high during each pulse |
| GND | GND | ESP32 GND — required |

**Oscilloscope without a DAC**: `scope_sd` is the waveform as a 1-bit stream at 50 MHz (a
second-order sigma-delta since 2026-10-05: the share of 1s follows the sample). Through 1 kΩ + 100 pF (cut-off
≈ 1.6 MHz) it becomes the analog waveform, window and amplitude included (simulated ripple
≈ 11 % of the amplitude). Unfiltered, it looks like a band of fast pulses; a DSO's averaging
(triggered on `scope_trig`) may still show the shape. With no parts at all, `dac_d[7]`
(pin 34, the sign bit) is a square wave at the instantaneous frequency: it shows the sweep and
the BPSK phase jumps, but not the window or the amplitude.

**LEDs** (lit = true): 0 BPSK, 1 capture armed, 2 recording or sending, 3 toggles on every good
SPI packet, 4 pulse active, 5 geometric sweep.

**Power**: each board on its own USB-C. Share GND only; never tie the 3.3 V or 5 V rails
together. Power the Tang Nano first (or both together): an ESP32 driving 3.3 V into an
unpowered FPGA back-feeds it through the pin protection.

**USB devices**: Tang Nano = `/dev/ttyUSB0` (JTAG) + `/dev/ttyUSB1` (UART), FTDI 0403:6010.
ESP32-S3 = `/dev/ttyACM0` (USB-Serial/JTAG, 303a:1001). If another USB-serial adapter is
plugged in, the numbers can shift: check with `ls /dev/ttyUSB* /dev/ttyACM*`, pass `--port`.
The ESP32 runs without a USB host (e.g. from a power bank); only the FPGA must be on the PC
for `host/demo.py`.

---

## 7. Toolchain and its quirks

| Tool | Version / location |
|---|---|
| yosys | 0.66 |
| nextpnr-himbaechel | 0.11.1 (Gowin) |
| Apycula (`gowin_pack`) | 0.33 in `~/.local/share/apycula-venv`, **patched** with upstream commit 81b3a9e (undo: `pip install --force-reinstall apycula==0.33` in that venv) |
| openFPGALoader | loads SRAM / flash over the on-board JTAG |
| `tn9k` | `~/.local/bin/tn9k`: `tn9k build`, `tn9k load` (SRAM, lost at power-off), `tn9k flash`, `tn9k clean` (`openFPGALoader -b tangnano9k -f build/top.fs` writes flash directly). Compiles every `.v` in the folder, top = `top`, reads `tn9k.conf` (`FREQ`, `SYNTH_FLAGS`, `SRC`) |
| iverilog / vvp, GTKWave | simulation (`sim/run <tb> -w` opens GTKWave) |
| Python 3 | numpy, scipy, matplotlib, pyserial |
| arduino-cli | 1.4.1, core `esp32:esp32` 3.3.7; esptool v5.1.0 |

Findings on the real chip — **do not undo these**:
- **Hardware multipliers (DSP)**: Apycula 0.33 crashed packing MULT18X18 (fixed by the patch
  above), and then the multipliers computed **unsigned** on the chip. The project builds with
  `-nodsp`; all multipliers are LUT-based, hence the pipeline register in `pulse_dds.v`.
- **Block RAM**: the deep, narrow modes (16K × 1, 4K × 4) store data at wrong addresses on the
  chip. Only 1K × 18 and 2K × 9 were verified. `capture.v` therefore uses 1024-deep banks.
- **The timing report can pass while the chip fails.** With the sigma-delta added, a single-clock
  12 × 13-bit LUT multiply in `scale.v` gave wrong products on a few samples per capture, though
  nextpnr reported slack. Splitting it into two registered partial products (2 clocks) fixed
  it (5/5 captures bit-exact). Keep LUT multipliers small and registered.
- The router prints `Failed to route net 'clk50' ... using dedicated routing` (the inverted
  clock to `dac_clk`); it is expected and harmless so far.

---

## 8. ESP32-S3 firmware (`MCU /SIH/`)

Files: `adaptive_sonar.ino` (pins, ADC, FreeRTOS tasks, SPI, serial console), `sonar_core.h`
(physics, decision, Doppler modulation rule, FPGA unit conversion, packet — pure C++,
PC-testable), `water_sim.h` (simulated water conditions), `sonar_config.h` (every
tunable number and the profile table), `physics_reference.py` (Python twin of the physics,
also makes ML datasets), `README_adaptive_sonar.md` (the team's full write-up: equations,
assumptions, sources), `adaptive_sonar_project.zip` (the same files in Arduino layout plus
`tools/`: `test_core.cpp`, `run_tests.sh`, `make_vectors.py`, mock Arduino headers; and `ml/`),
`ml_model.h` (the ML model as C, generated), `ml/` (training code, report, test vectors).

**The loose files and the zip must match.** The tests exist only in the zip. After editing a
loose file: extract the zip to a temporary folder, copy the edited files into
`adaptive_sonar/`, run `bash tools/run_tests.sh`, then rebuild the zip (there is no `zip`
command on this machine; use Python's `zipfile`). The Arduino sketch folder must be named
`adaptive_sonar`.

**What it does** (per ping, 6 a second): Mackenzie sound speed → Francois–Garrison absorption + turbidity
loss + spherical spreading → thermal + wind noise → output SNR after pulse compression
(`SL + 10·log T − TL − N0`, i.e. set by pulse energy) → for every profile: does it close the
link (12 dB threshold + 3 dB margin) and how much energy per ping does it need (just enough
power, minimum 5 % amplitude) → **detail-vs-energy rule**: take the finest profile whose energy
is at most max(`DETAIL_ENERGY_FACTOR` (4) × the cheapest working profile's energy,
`DETAIL_FREE_ENERGY_FRAC` (1 %) × this ping's energy budget); a profile finer than the
previous one needs 2 dB extra margin (anti flip-flop) → if none closes the link, L-L at
maximum allowed power (best effort) → packet. Lower frequencies always need less energy, so
the rule only ever picks X-S, H-S, M-S, L-S, L-M or L-L. Then the **modulation from the
motion** (`chooseModulation()`, `sonar_config.h` section 4c): the Doppler drift over the pulse
is fd·T cycles with fd = speed / c × fc (× 2 in echo mode); BPSK while it is ≤ 0.25, LFM up to
1, geometric beyond (BPSK is the cleanest when still, geometric tolerates the most motion).
Four FreeRTOS tasks: sensor 50 Hz (pots / typed inputs), decision 6 Hz (one per ping, exactly 6
per second on average; steps the simulation in automatic mode; anti flip-flop memory per
contact), SPI 50 Hz (highest priority, sends the latest packet), console.

**Simulated conditions** (`water_sim.h`, automatic mode, the default at power-on): a vehicle
dives and climbs 20 → 350 m every 20 mission-minutes through a thermocline (22 → 5 °C) and
halocline (34.4 → 35.2 ppt); turbidity = background + drifting plumes + a muddy layer below
300 m; battery drains and recharges at 15 %; three contacts, pinged in turn: A 15–180 m and
lively, B 250–600 m and nearly still, C 450–1000 m and moving (range and radial speed follow
smooth random motions; the speed is the range rate). Sensor noise on every reading; mission
time runs 10× real time. Deterministic from its seed: `host/water_sim.py` reproduces it
(`host/check_water_sim.py`: within 0.001 over 20,000 pings).

**Profiles** (LFM in the table; the modulation is chosen per ping): X 380–500 kHz,
H 270–330 kHz, M 180–220 kHz, L 100–140 kHz, each with 1 / 5 / 20 ms pulses.

**Serial console** (115200 on `/dev/ttyACM0`; `host/esp_console.py` opens it without resetting):
- `A`: automatic (simulated conditions); prints one `PING seq=… contact=… T=… S=… D=… turb=…
  bat=… R=… v=… profile=… mod=… amp=… ml_profile=… ml_mod=… ml_amp=… flags=… doppler=…`
  line per ping.
- `W` (or Enter when not automatic): asks for the seven inputs one by one, then prints a full
  report (network picks and the physics check, result, FPGA values). `I T S D turb bat R v`:
  the same on one line, ending with a `DECISION` line.
- `R <metres>`, `M lfm | geo | bpsk | cw | auto` (force a modulation; `auto` = from the
  motion), `P` (potentiometers), `L` (status line), `?`.
- `C [amp]`: **DAC view.** Sends the FPGA a slow-motion copy of the current ping (every
  frequency ÷ N, pulse × N, N ≤ 100 so the length fits 24 bits; period 1.5 × length; optional
  amplitude), holds it (`gCapture`) while recording GPIO 1 with the ADC in continuous mode (≤ 80
  kS/s, ≤ 50,000 samples, calibrated to mV, a window of 2.7 pulse lengths so one whole pulse is
  inside), then prints `CAP key=value …` (the packet's FPGA fields), `D` lines of 3-digit hex mV,
  and `END`. The ADC continuous driver cannot start if the potentiometer mode (one-shot ADC)
  was used since power-on: reset the ESP32 first. `host/dac_view.py [--amp 1]` runs it and plots.
- `S [n] [r]`: the next n decisions (default 6) as they are made, each captured r times (default
  4) as above, each block preceded by a `COND` line with the ping's conditions; ends with
  `SEQEND`. `host/dac_sequence.py` runs it (about 2.5 minutes) and plots.
- The console writes in 32-byte pieces and waits for each to leave: the USB-Serial/JTAG
  driver loses text when it is queued faster, and `Serial.flush()` can discard it.

**Build and flash**:
```
cp "MCU /SIH/"{adaptive_sonar.ino,sonar_core.h,sonar_config.h,water_sim.h,ml_model.h} <tmp>/adaptive_sonar/
arduino-cli compile -b "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=disabled" --output-dir <tmp>/build <tmp>/adaptive_sonar
arduino-cli upload  -p /dev/ttyACM0 -b "<same fqbn>" --input-dir <tmp>/build <tmp>/adaptive_sonar
esptool --chip esp32s3 -p /dev/ttyACM0 --before no-reset --after watchdog-reset read-mac   # start the program
```
- If the upload cannot connect (a running program owns the USB port): hold BOOT, tap RESET,
  release BOOT, upload again.
- After an upload the "hard reset via RTS" boots back into download mode; the esptool
  watchdog reset above starts the firmware.
- Open the serial port with DTR and RTS **off**: on this USB port they drive BOOT and RESET.

**ML model** (required by the project). The ML chooses the waveform; the physics checks it.
- **What runs** (`MCU /SIH/ml_model.h`, 30 KB of weights, generated by `MCU /SIH/ml/train.py`,
  `USE_ML_MODEL 1` by default), called by `mlSelect()` and `mlModulation()` in the sketch:
  - profile network 6 → 48 → 48 → 6 (ReLU): one score per profile the rule ever picks
    (`ML_CLASSES` maps outputs to profile numbers);
  - amplitude network 12 → 48 → 48 → 1: the 6 inputs + the chosen profile (one-hot) →
    log(amplitude that just closes the link, before the 5 % floor) + a +2.3 % safety margin;
  - modulation network 16 → 32 → 32 → 3: temperature, salinity, depth, log10(speed + 0.01)
    + the profile the physics settled on (one-hot of 12) → LFM / geometric / BPSK.
  Inputs: temperature, salinity, depth, turbidity, battery, range (as log10), standardised,
  clamped to the sensor ranges first.
- **Roles** (`sonar::decide(env, range, prev, mlProfile, mlAmp)`): the ML's profile is kept
  when it closes the link within the energy allowance, with the anti flip-flop margin
  (`FLAG_ML_USED`), else the physics chooses (`FLAG_ML_OVERRIDDEN`). The ML's amplitude is used
  for the ML's profile, raised to what the link needs if short (`FLAG_ML_AMP_RAISED`, flags
  bit 6) and capped at what is allowed. The ML's modulation is kept when it can take the
  Doppler drift (a more tolerant one than needed is fine), else the rule's replaces it
  (`FLAG_ML_MOD_OVERRIDDEN`, bit 7).
- **Data**: 300,000 scenarios labelled by `physics_reference.py` (half with log-uniform
  range), inputs rounded to float32; the amplitude network trains on the (scenario, profile)
  pairs whose needed amplitude is in the range that matters. Tested on 2 × 20,000 scenarios
  with other seeds.
- **Results** (`ml/REPORT.md`): profile agreement 99.2 % (uniform range) / 99.4 % (log
  range), 0.5 % overridden (XGBoost: 97–99 % with ~20k tree nodes; decision tree 94–96 %).
  Amplitude: 1.8 % median error, 1.6–3.4 % more energy than the minimum, raised in 1–2 % of
  cases. With random sensor values at fixed ranges 50–2000 m: 97.8–100 % the same profile as
  the physics. Modulation: 99.7 % agreement, 0.2 % overridden, all disagreements within ~1 % of
  a threshold. The C code matches the Python models on all test vectors (6,000 each).
- **Limit**: trained on the physics, so it can only copy it. It beats the physics only when
  retrained on measured results (see open items).
- **Retrain** after any `sonar_config.h` change that affects the choice (profiles, power,
  thresholds, the detail-vs-energy knobs, sensor ranges):
  `python3 -m venv ml/.venv && ml/.venv/bin/pip install -r ml/requirements.txt`, then
  `ml/.venv/bin/python ml/train.py` (~5 min on a laptop CPU, no GPU), then the firmware tests
  and the zip rebuild. `ml/.venv/` (700 MB) is not part of the project.
- `physics_reference.py` finds `sonar_config.h` next to itself or in `../adaptive_sonar/`.

---

## 9. Build, test, run

```
sim/all                                  # all 11 testbenches + checkers, prints ALL PASS
sim/run top_tb -w                        # one testbench, then GTKWave
tn9k load                                # build + load the FPGA (SRAM)
python3 host/demo.py                     # one second of the running system (ESP32 + FPGA), 6 pings
python3 host/check_water_sim.py          # the PC's copy of the simulated conditions matches the C++
python3 host/dac_view.py --amp 1         # the sigma-delta DAC's filtered output vs the FPGA's samples (no scope)
python3 host/dac_sequence.py             # one second of pings through the DAC, measured (~2.5 min)
python3 host/real_speed.py               # 6 pings at full speed after the DAC (BlackPill ADC, 2.4 MS/s)
python3 host/wavegen.py geo --fc 325e3 --bw 350e3 --capture   # PC sets a waveform, captures, checks
python3 host/capture.py --request        # capture whatever is playing now (e.g. the ESP32's choice)
python3 host/capture.py                  # wait for S2 presses
```
`wavegen.py` kinds: `tone`, `chirp`, `geo`, `bpsk`; options `--fc`, `--bw`, `--amp 0..1`,
`--period <clocks>`, `--capture`, `--port`, `--dry-run`, `--frames-out`, `--spi-out`. Its
pulses are always 13,000 clocks, sweeps always go up, BPSK is always Barker-13.
Plots go to `sim/out/board_capture_<kind>.png` (waveform + spectrogram with the expected sweep).

| Testbench | Checks |
|---|---|
| `phase_acc_tb`, `pulse_ctrl_tb`, `phase_code_tb` | module behaviour (self-checking) |
| `capture_tb` | capture on its own: header, one sample in 1 / 4 / 1024 (a counter as input), and two streamed captures after one arm |
| `dds_tb` | tone vs golden model + FFT peak |
| `pulse_dds_tb` | windowed pulses vs golden model; window narrows the spectrum |
| `chirp_tb` | LFM up and down, 3 ↔ 7 MHz, bit-exact + measured sweep |
| `geo_tb` | geometric 2 → 8 MHz, bit-exact + measured: exactly two octaves |
| `bpsk_tb` | Barker-13, bit-exact + code read back from the samples |
| `sigma_delta_tb` | scope output: first- and second-order bit streams of a chirp both bit-exact against their models; after a simulated 1 kΩ + 100 pF it follows the waveform (ripple < 15 %) |
| `top_tb` | whole board: UART chirp, SPI packet with bad CRC (rejected) + good geometric packet, UART BPSK; three captures checked by `capture.py` |

---

## 10. Open items

1. **CS line**: the ESP32 → FPGA link works without it (framed by the pause after each
   burst). Finding out why GPIO14 → pin 36 carries only noise (ESP32 pin, jumper or FPGA pin; a
   GPIO14 toggle test with the pin probe would tell) is optional.
2. **Choose the DAC**: the DAC0808 + TL084 Butterworth board needs a −5 V charge pump, faster
   op-amps (OPA4350), U5A's feedback moved to its − input, a 13-clock sample hold in the FPGA
   (3.857 MS/s) and a bit-order check; shelved for now. Meanwhile the sigma-delta (pin 40) works,
   and an 8-bit R-2R on `dac_d[7:0]` would be cleaner. For the transmitter, a single-supply
   high-speed DAC (DAC908 / AD9708 class) fits `dac_d` + `dac_clk` directly.
   `dac_view.py` shows the DAC output's shape but not the filter's effect at 100–500 kHz (the slow
   copy is at 1–5 kHz); that still needs a scope.
3. **Doppler thresholds** (`DOPPLER_BPSK_MAX_CYCLES` 0.25, `DOPPLER_LFM_MAX_CYCLES` 1): set
   from rules of thumb; confirm with the team (and retrain after a change). The speed input is
   simulated; a real one needs a DVL or a Doppler estimate from echoes.
4. **One-way link or echo sonar** (`ACTIVE_ECHO_MODE`): changes the loss maths and the period
   (echo mode waits for the round trip).
5. Sensors: real sensors (or the potentiometers, GPIO 1–5) instead of the simulation.
6. Captures of 20 ms pulses keep one sample in 64 (783 kS/s): enough for 100–140 kHz, but a
   20 ms pulse above ~390 kHz would alias (no such profile is chosen today).
7. MISO (pin 39 ↔ GPIO11) is wired but unused; it could return status to the ESP32.
8. From the firmware's own list: transducer coverage of 100–500 kHz, turbidity loss
   calibration, where the target range comes from.
9. **Tune the detail-vs-energy rule** with the team: `DETAIL_ENERGY_FACTOR` (4) and
    `DETAIL_FREE_ENERGY_FRAC` (0.01) in `sonar_config.h`; retrain the ML after changing them.
10. **Close the loop** for ML that can beat the physics: measure the received level per ping
    (hydrophone → amp → log detector → ESP32 ADC) and log it with the inputs and the choice.
    Later: a speed (Doppler) estimate from echoes would replace the simulated speed.
11. The ESP32 and FPGA ping clocks are independent (both 6 Hz from their own crystals): about
    once every few hours a ping may repeat or skip a decision. A MISO or trigger line from the
    FPGA could synchronise them.

---

## 11. Rules for changes

- **The golden model is the specification.** Any change to the datapath (`pulse_dds.v`,
  `sweep.v`, `phase_code.v`, `dds.v`, `scale.v`, `pulse_ctrl.v`) must be mirrored in
  `host/model.py`, and `sim/all` must stay ALL PASS. On the board, `capture.py` must report
  PASS (bit-exact).
- **Changing the packet** touches all of: `MCU /SIH/sonar_core.h` (+ its tests, README and the
  zip), `host/mcu_packet.py`, the field extraction in `top.v` (`pkt_settings`), the capture
  header if new fields must be visible (`top.v`, `host/capture.py`), and the `top_tb`
  stimulus (regenerate with `wavegen.py --frames-out sim/top_frames.hex` and
  `--spi-out sim/top_spi.hex`; delete the old files first, the options append).
- Keep `FPGA_CLK_HZ` (firmware), `F_CLK` (`host/model.py`, `host/mcu_packet.py`) and the PLL
  settings in agreement.
- Keep `-nodsp` and 1K-deep BSRAM banks (section 7).
- Put nothing 1.8 V-incompatible on bank 3 pins.
- Update this file (status, limits, pins, open items) with any change that affects them.
