# Adaptive sonar waveform generator: project guide for agents

This file describes the whole project: what it does, how the pieces fit, how to build,
test and run it, what has been verified, and what is still open. Read it before changing
anything. Keep it up to date when you change the design (see "Rules for changes" at the end).

Last updated: 2026-09-30 (ESP32 → FPGA link verified end to end on the boards; SPI framed without CS).

---

## 1. What the system does

An underwater sonar transmitter that adapts its waveform to the water conditions.

```
 5 sensors ─► ESP32-S3 ────────────────── SPI (57-byte packet, 10 Hz) ──► Tang Nano 9K FPGA ──► 8-bit DAC ──► amplifier ──► transducer
 (pots for    physics model picks a                                        generates the pulse
  now)        waveform profile and power                                   sample by sample at 50.14 MHz
                                                     PC (USB UART) ──────►  (can stand in for the ESP32,
                                                                   ◄──────   and read captured pulses back)
```

- **ESP32-S3 (`MCU /SIH/`)**: reads temperature, salinity, depth, turbidity and battery, runs
  an underwater acoustics model, picks one of 12 chirp profiles (100–500 kHz, 1/5/20 ms), sets
  the drive amplitude just high enough to close the link, converts everything into the FPGA's
  own units and sends a packet over SPI ten times a second.
- **FPGA (the Verilog in this folder)**: turns the packet into windowed pulses. Four
  modulations: CW tone, LFM chirp, geometric (exponential) sweep, and BPSK with the Barker-13
  code. Every pulse gets a Hann window (to keep its spectrum tight) and an amplitude.
  It drives an 8-bit parallel DAC.
- **PC (`host/`)**: can send the same packet over the Tang Nano's USB UART (stands in for the
  ESP32), and can ask the FPGA to capture a pulse and send it back; the samples are checked
  against a bit-exact Python golden model.

---

## 2. Current status (2026-09-30)

| Part | State |
|---|---|
| FPGA design | All 9 simulations pass (`sim/all`). Built: 44 % LUT, 45 % DFF, 18/26 BSRAM, 65 MHz (needs 50.14). |
| FPGA on the board, PC path (UART) | **Verified**: chirp, geometric, BPSK and tone captured bit-exact against the golden model (16,383 samples each), with packet v3. |
| ESP32 → FPGA over SPI | **Verified on the boards (2026-09-30)**: 10 good packets/s, 0 bad; the FPGA plays the ESP32's choice bit-exact (H-S chirp at 300 m; M-S geometric sweep after `R 600` + `M geo`). Framed **without CS** (see section 5): the CS wire (GPIO14 → pin 36) never carried a clean signal. |
| ESP32 firmware v3 + ML | 34/34 host tests pass; **flashed and running** (native USB build, `CDCOnBoot=cdc`); the status line shows `ML` decisions. |
| Wiring ESP32 ↔ FPGA | Done (pin table in section 6). SCK, MOSI verified with an edge-counting probe; CS picks up only noise (cause not found: ESP32 pin, wire or FPGA pin). |
| DAC | Not chosen / not connected. The FPGA already drives `dac_d[7:0]` and `dac_clk`. |
| Sensors | No potentiometers wired: the ESP32's ADC inputs float, so its readings are noise. |
| ML model | Built (2026-09-30): two networks in `MCU /SIH/ml_model.h` (profile 6-48-48-6, amplitude 12-48-48-1), `USE_ML_MODEL 1`. Profile agrees with the physics on 99.2–99.4 %, 0.5 % overridden; amplitude within 1.8 % (median), 1.6–3.4 % extra energy. **Running on the ESP32.** |
| Git | The SPI / packet-v3 work is not committed yet, and `MCU /` is untracked. |

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
| `scale.v` | Fixed-point multiply: `out = in × factor / 4096` |
| `spi_rx.v` | SPI slave (mode 0) that collects one packet, framed by CS or (as used now) by the pause after each burst |
| `pkt_check.v` | Checks sync, version and CRC-8 of a packet, one byte per clock |
| `uart_rx.v`, `uart_tx.v`, `cmd_rx.v` | USB UART (115200 8N1) and its command frames |
| `capture.v` | Records 2^14 samples from the next pulse into BSRAM, then sends header + samples over the UART |
| `pll.v`, `button.v` | 27 → 50.142857 MHz PLL; debounced buttons |
| `sine.hex`, `hann.hex` | ROM contents (made by `host/gen_sine.py`, `host/gen_window.py`) |
| `tangnano9k.cst` | Pin constraints |
| `tn9k.conf` | Build settings read by the `tn9k` script (`FREQ=50`, `SYNTH_FLAGS=-nodsp`) |
| `host/model.py` | Bit-exact golden model of `pulse_dds`, plus `geo_ftw` / `lfm_ftw` sweep tracks |
| `host/mcu_packet.py` | The ESP32 packet in Python: build, parse, CRC-8, unit conversion (same maths as the firmware) |
| `host/wavegen.py` | PC as the ESP32: sends a packet over the UART, optionally captures and checks the pulse |
| `host/capture.py` | Receives captures (board or simulation dump), checks them, plots waveform + spectrogram |
| `host/check_*.py` | Checkers for the unit testbenches |
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
  from LUTs, see section 7).

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
clocks per chip). A connected ESP32 replaces them within 100 ms.

### Capture (`capture.v`)
Armed by **S2** or the UART command `A5 02 02`. Records 2^14 = 16,384 samples starting at the
next pulse start (327 µs; longer pulses are captured only in part), then sends a 37-byte
header and the samples as little-endian int16 over the UART (~3 s at 115200 baud). The header
is frozen at the pulse start, so it always describes the captured pulse.

Header (little-endian): `A5 5A`, `mod` u8, `N` u16, `len` u32, `ftw_start` u32, `ftw_step` u32,
`win_step` u32, `code` u16, `chip_len` u32, `amp` u16, `period` u32, good SPI packets u16,
bad SPI packets u16 (`HDR = "<BHIIIIHIHIHH"` after the two sync bytes in `host/capture.py`).

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
| 7 | `flags` | bit0 link OK, bit1 low battery, bit2 input clamped, bit3 ML used, bit4 best effort, bit5 ML overridden, bit6 ML amplitude raised |
| 8–27 | `centerFreqHz`, `bandwidthHz`, `pulseUs`, `amplitude` (0–65535), `txPowerMilliW`, `propDelayUs` | telemetry |
| 28–31 | `lenClk` | **FPGA**: pulse length, clocks (BPSK: 13 × `chipLen`) |
| 32–35 | `ftwStart` | **FPGA**: start FTW (fc for CW/BPSK, fc − bw/2 for sweeps) |
| 36–39 | `ftwStep` | **FPGA**: LFM: FTW per clock × 2¹⁶ (signed). Geometric: (ratio − 1) × 2³² per 32 clocks, ratio = (f_end / f_start)^(1 / ⌊lenClk / 32⌋). Else 0 |
| 40–43 | `winStep` | **FPGA**: 2³² / lenClk |
| 44–45 | `ampQ12` | **FPGA**: 4096 = 1.0 |
| 46–47 | `code` | **FPGA**: BPSK code (0x0A60), else 0 |
| 48–51 | `chipLen` | **FPGA**: BPSK clocks per chip, else 0 |
| 52–55 | `periodClk` | **FPGA**: clocks between pulse starts (0.1 s minimum; + round trip in echo mode) |
| 56 | `crc8` | CRC-8, poly 0x07, init 0, over bytes 0–55 (check value of "123456789" = 0xF4) |

**SPI**: mode 0 (SCK idles low, sample on rising edge), MSB first, 2 MHz. The ESP32 sends each
packet as one continuous 57-byte burst (228 µs) every 100 ms, with CS low around it.
**Framing on the current boards: without CS** (`spi_rx` parameter `USE_CS = 0` in `top.v`): a
frame ends when SCK has been quiet for 100 µs (`GAP` = 5000 clocks). This was needed because
the CS connection (ESP32 GPIO14 → FPGA pin 36) only ever showed noise on the FPGA side (an
edge-counting probe saw 26–201 random edges/s instead of 10/s), while SCK (exactly 4,560 edges/s)
and MOSI were clean. If CS is ever fixed, set `USE_CS(1)` to frame by CS again. Either way, any
byte count other than 57, a bad CRC, a wrong sync or version → rejected and counted.

**UART (PC)**: 115200 8N1 on the Tang Nano's second FTDI channel.
- `A5 01 <57-byte packet> <xor of 01 and the 57 bytes>`: set the waveform.
- `A5 02 02`: capture the next pulse.
- A frame whose bytes stop arriving for 1 ms is dropped.

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
| GND | GND | ESP32 GND — required |

**LEDs** (lit = true): 0 BPSK, 1 capture armed, 2 recording or sending, 3 toggles on every good
SPI packet, 4 pulse active, 5 geometric sweep.

**Power**: each board on its own USB-C. Share GND only; never tie the 3.3 V or 5 V rails
together. Power the Tang Nano first (or both together): an ESP32 driving 3.3 V into an
unpowered FPGA back-feeds it through the pin protection.

**USB devices**: Tang Nano = `/dev/ttyUSB0` (JTAG) + `/dev/ttyUSB1` (UART), FTDI 0403:6010.
ESP32-S3 = `/dev/ttyACM0` (USB-Serial/JTAG, 303a:1001). If another USB-serial adapter is
plugged in, the numbers can shift: check with `ls /dev/ttyUSB* /dev/ttyACM*`, pass `--port`.

---

## 7. Toolchain and its quirks

| Tool | Version / location |
|---|---|
| yosys | 0.66 |
| nextpnr-himbaechel | 0.11.1 (Gowin) |
| Apycula (`gowin_pack`) | 0.33 in `~/.local/share/apycula-venv`, **patched** with upstream commit 81b3a9e (undo: `pip install --force-reinstall apycula==0.33` in that venv) |
| openFPGALoader | loads SRAM / flash over the on-board JTAG |
| `tn9k` | `~/.local/bin/tn9k`: `tn9k build`, `tn9k load` (SRAM, lost at power-off), `tn9k flash`, `tn9k clean`. Compiles every `.v` in the folder, top = `top`, reads `tn9k.conf` (`FREQ`, `SYNTH_FLAGS`, `SRC`) |
| iverilog / vvp, GTKWave | simulation (`sim/run <tb> -w` opens GTKWave) |
| Python 3 | numpy, scipy, matplotlib, pyserial |
| arduino-cli | 1.4.1, core `esp32:esp32` 3.3.7; esptool v5.1.0 |

Findings on the real chip — **do not undo these**:
- **Hardware multipliers (DSP)**: Apycula 0.33 crashed packing MULT18X18 (fixed by the patch
  above), and then the multipliers computed **unsigned** on the chip. The project builds with
  `-nodsp`; all multipliers are LUT-based, hence the pipeline register in `pulse_dds.v`.
- **Block RAM**: the deep, narrow modes (16K × 1, 4K × 4) store data at wrong addresses on the
  chip. Only 1K × 18 and 2K × 9 were verified. `capture.v` therefore uses 1024-deep banks.
- The router prints `Failed to route net 'clk50' ... using dedicated routing` (the inverted
  clock to `dac_clk`); it is expected and harmless so far.

---

## 8. ESP32-S3 firmware (`MCU /SIH/`)

Files: `adaptive_sonar.ino` (pins, ADC, FreeRTOS tasks, SPI, serial), `sonar_core.h` (physics,
decision, FPGA unit conversion, packet — pure C++, PC-testable), `sonar_config.h` (every
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

**What it does** (10 Hz): Mackenzie sound speed → Francois–Garrison absorption + turbidity
loss + spherical spreading → thermal + wind noise → output SNR after pulse compression
(`SL + 10·log T − TL − N0`, i.e. set by pulse energy) → for every profile: does it close the
link (12 dB threshold + 3 dB margin) and how much energy per ping does it need (just enough
power, minimum 5 % amplitude) → **detail-vs-energy rule**: take the finest profile whose energy
is at most max(`DETAIL_ENERGY_FACTOR` (4) × the cheapest working profile's energy,
`DETAIL_FREE_ENERGY_FRAC` (1 %) × this ping's energy budget); a profile finer than the
previous one needs 2 dB extra margin (anti flip-flop) → if none closes the link, L-L at
maximum allowed power (best effort) → packet. Lower frequencies always need less energy, so
the rule only ever picks X-S, H-S, M-S, L-S, L-M or L-L. Four FreeRTOS tasks: sensor 50 Hz,
decision 10 Hz, SPI 10 Hz (highest priority), logger (status line every 500 ms).

**Profiles** (all LFM in the table): X 380–500 kHz, H 270–330 kHz, M 180–220 kHz,
L 100–140 kHz, each with 1 / 5 / 20 ms pulses.

**Serial commands** (115200 on `/dev/ttyACM0`): `R <metres>` target range (10–2000, default
300); `M lfm | geo | bpsk | cw | auto` forces the modulation for whatever profile the physics
picks (`auto` = the table's LFM).

**Build and flash**:
```
cp "MCU /SIH/"{adaptive_sonar.ino,sonar_core.h,sonar_config.h} <tmp>/adaptive_sonar/
arduino-cli compile -b "esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=16M,PSRAM=disabled" --output-dir <tmp>/build <tmp>/adaptive_sonar
arduino-cli upload  -p /dev/ttyACM0 -b "<same fqbn>" --input-dir <tmp>/build <tmp>/adaptive_sonar
esptool --chip esp32s3 -p /dev/ttyACM0 --before no-reset --after watchdog-reset read-mac   # start the program
```
- If the upload cannot connect (a running program owns the USB port): hold BOOT, tap RESET,
  release BOOT, upload again.
- After an upload the "hard reset via RTS" boots back into download mode; the esptool
  watchdog reset above starts the firmware.
- Open the serial port with DTR and RTS **off**: on this USB port they drive BOOT and RESET.

**ML model** (required by the project; built 2026-09-30). The ML chooses the waveform; the
physics checks it.
- **What runs** (`MCU /SIH/ml_model.h`, 23.5 KB, generated by `MCU /SIH/ml/train.py`,
  `USE_ML_MODEL 1` by default), called by `mlSelect()` in the sketch:
  - profile network 6 → 48 → 48 → 6 (ReLU): one score per profile the rule ever picks
    (`ML_CLASSES` maps outputs to profile numbers);
  - amplitude network 12 → 48 → 48 → 1: the 6 inputs + the chosen profile (one-hot) →
    log(amplitude that just closes the link, before the 5 % floor) + a +2.3 % safety margin.
  Inputs: temperature, salinity, depth, turbidity, battery, range (as log10), standardised,
  clamped to the sensor ranges first.
- **Roles** (`sonar::decide(env, range, prev, mlProfile, mlAmp)`): the ML's profile is kept
  when it closes the link within the energy allowance, with the anti flip-flop margin
  (`FLAG_ML_USED`), else the physics chooses (`FLAG_ML_OVERRIDDEN`). The ML's amplitude is used
  for the ML's profile, raised to what the link needs if short (`FLAG_ML_AMP_RAISED`, flags
  bit 6) and capped at what is allowed. The status line shows `ML` / `ML_OVERRIDDEN` /
  `PHYSICS`, `AMP_RAISED`, and the energy numbers (`E`, `Emin`, `allow`).
- **Data**: 300,000 scenarios labelled by `physics_reference.py` (half with log-uniform
  range), inputs rounded to float32; the amplitude network trains on the (scenario, profile)
  pairs whose needed amplitude is in the range that matters. Tested on 2 × 20,000 scenarios
  with other seeds.
- **Results** (`ml/REPORT.md`): profile agreement 99.2 % (uniform range) / 99.4 % (log
  range), 0.5 % overridden (XGBoost: 97–99 % with ~20k tree nodes; decision tree 94–96 %).
  Amplitude: 1.8 % median error, 1.6–3.4 % more energy than the minimum, raised in 1–2 % of
  cases. With random sensor values at fixed ranges 50–2000 m: 97.8–100 % the same profile as
  the physics. The C code matches the Python models on all 6,000 test vectors.
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
sim/all                                  # all 9 testbenches + checkers, prints ALL PASS
sim/run top_tb -w                        # one testbench, then GTKWave
tn9k load                                # build + load the FPGA (SRAM)
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
| `dds_tb` | tone vs golden model + FFT peak |
| `pulse_dds_tb` | windowed pulses vs golden model; window narrows the spectrum |
| `chirp_tb` | LFM up and down, 3 ↔ 7 MHz, bit-exact + measured sweep |
| `geo_tb` | geometric 2 → 8 MHz, bit-exact + measured: exactly two octaves |
| `bpsk_tb` | Barker-13, bit-exact + code read back from the samples |
| `top_tb` | whole board: UART chirp, SPI packet with bad CRC (rejected) + good geometric packet, UART BPSK; three captures checked by `capture.py` |

---

## 10. Open items

1. **CS line**: the ESP32 → FPGA link works without it (framed by the pause after each
   burst). Finding out why GPIO14 → pin 36 carries only noise (ESP32 pin, jumper or FPGA pin; a
   GPIO14 toggle test with the pin probe would tell) is optional.
2. **Choose the DAC**: it must take 8-bit parallel data at 50 MS/s (or the FPGA must slow its
   output rate), and its latch edge must match `dac_clk`.
3. **Automatic choice between LFM, geometric and BPSK**: the physics scores waveforms by pulse
   energy only, so it cannot tell them apart. It needs a relative-speed (Doppler) input:
   geometric sweeps tolerate motion, BPSK needs a nearly still scene (≈ 0.6 m/s at 300 kHz
   with a 1 ms pulse, ≈ 3 cm/s with 20 ms). Until then the operator uses `M`.
4. **One-way link or echo sonar** (`ACTIVE_ECHO_MODE`): changes the loss maths and the period
   (echo mode waits for the round trip).
5. Sensors: wire the potentiometers (GPIO 1–5) or real sensors.
6. Capturing whole long pulses: the buffer holds 327 µs; 1–20 ms pulses need decimation or a
   bigger/external buffer to be seen in full.
7. MISO (pin 39 ↔ GPIO11) is wired but unused; it could return status to the ESP32.
8. From the firmware's own list: transducer coverage of 100–500 kHz, turbidity loss
   calibration, where the target range comes from.
9. **Tune the detail-vs-energy rule** with the team: `DETAIL_ENERGY_FACTOR` (4) and
    `DETAIL_FREE_ENERGY_FRAC` (0.01) in `sonar_config.h`; retrain the ML after changing them.
10. **Close the loop** for ML that can beat the physics: measure the received level per ping
    (hydrophone → amp → log detector → ESP32 ADC) and log it with the inputs and the choice.
    Later: a speed (Doppler) estimate from echoes would also enable automatic LFM / geometric /
    BPSK choice.
11. Commit the current work (SPI receiver, packet v3, ML, `MCU /`), and write the FPGA
    bitstream to flash (`tn9k flash`) so it survives power-off.

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
