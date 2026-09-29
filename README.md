# Waveform generator: finished reference (Tang Nano 9K)

A working copy of the learning project in `../Basic`, with every piece filled in.
The PC stands in for the MCU: it sends the waveform settings over the USB UART,
the FPGA generates windowed pulses, and it can capture one pulse and send it back
to be checked against the Python golden model sample for sample.

Verified on the board on 2026-09-29: LFM chirp, geometric sweep (150 -> 500 kHz and
100 -> 400 kHz at 0.6 amplitude), BPSK Barker-13 and a tone all matched the golden model
on all 16,383 samples.

## Use it

```
tn9k load                                   # build + load into the FPGA (volatile)

python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --capture    # LFM 150 -> 500 kHz
python3 host/wavegen.py geo   --fc 325e3 --bw 350e3 --capture    # geometric 150 -> 500 kHz
python3 host/wavegen.py bpsk  --fc 300e3 --capture               # Barker-13 at 300 kHz
python3 host/wavegen.py tone  --fc 200e3 --amp 0.75 --capture
```

- Every pulse is 13,000 clocks (259 us). Sweeps always go up, from `fc - bw/2` to
  `fc + bw/2`. BPSK always uses Barker-13 at 1,000 clocks per chip (13 chips = the pulse).
- `--capture` asks the board for the next pulse, checks it and saves a plot in
  `sim/out/board_capture_<kind>.png` (spectrogram, with the expected sweep dashed).
  Without it, the settings are just sent.
- Other options: `--amp 0..1`, `--period` (clocks between pulse starts), `--dry-run`.
- On the board: **S1** steps through three built-in presets (chirp 150 -> 500 kHz,
  geometric 150 -> 500 kHz, BPSK at 300 kHz). **S2** captures the next pulse
  (listen with `python3 host/capture.py`).
- LEDs: 0 BPSK, 1 armed, 2 recording, 3 sending, 4 pulse active, 5 geometric sweep.
- DAC pins (for later): `dac_d[7:0]` on pins 25-30, 33, 34 and `dac_clk` on 35 (3.3 V).

## Geometric sweep

An LFM chirp adds the same number of Hz every clock (a straight line in frequency).
A geometric sweep **multiplies** the frequency by the same ratio every step, so it spends
equal time in each octave (a straight line on a log-frequency axis, a curve on a linear one).

Multiplying `ftw` by a ratio every clock would need a 32 x 32 multiplier, and this
toolchain can't use the chip's hardware multipliers. So `sweep.v` updates the frequency
every **32 clocks** (0.64 us, far quicker than one cycle of the wave), and works out
`ftw x step / 2^32` in the meantime with a **shift-and-add multiplier**: one bit of `step`
per clock, just an adder. For a geometric sweep `ftw_step` holds `(ratio - 1) x 2^32`,
where `ratio = (f_end / f_start) ^ (1 / (len / 32))`. `host/model.py` does the same
arithmetic, so the board matches it bit for bit.

## Test in simulation

```
sim/all        # 9 testbenches
```

`sim/geo_tb.v` checks the geometric sweep on its own (2 -> 8 MHz, exact match, and the
measured frequency climbs exactly two octaves). `sim/top_tb.v` simulates the whole board:
it sends the frames in `sim/top_frames.hex` (made by `wavegen.py --frames-out`: a chirp,
a geometric sweep, BPSK) and checks the three captures it gets back.

## Settings struct (26 bytes, little-endian)

The PC sends `A5 01 <26 bytes> <xor checksum>`, or `A5 02 02` to capture.
The MCU will later send the same struct over SPI.

| Bytes | Field | Meaning |
| --- | --- | --- |
| 0-1 | len | pulse length in clocks (max 65535 = 1.3 ms) |
| 2-5 | ftw_start | starting frequency: f x 2^32 / 50.142857 MHz |
| 6-9 | ftw_step | LFM: frequency change per clock (0 = tone). Geometric: (ratio - 1) x 2^32 |
| 10-13 | win_step | 2^32 / len (Hann window, one lap per pulse) |
| 14-15 | code | phase code, bit i = chip i, 1 = flip |
| 16-17 | chip_len | clocks per chip |
| 18-19 | amp | amplitude, 4096 = 1.0 |
| 20-23 | period | clocks between pulse starts |
| 24-25 | mode | bit 0: 1 = geometric sweep |

New settings wait until no pulse is playing (double buffering), so a pulse is never
changed halfway through.

## What differs from the learning folder

| File | Change |
| --- | --- |
| `pulse_dds.v` | pipeline register before the first multiply |
| `uart_tx.v` | filled in |
| `uart_rx.v`, `cmd_rx.v` | new: receive settings and capture commands |
| `top.v` | settings from the PC, double-buffered, three presets |
| `sweep.v` | geometric mode (shift-and-add multiplier, update every 32 clocks) |
| `capture.v` | header frozen at pulse start (same banked RAM as the learning folder) |
| `host/wavegen.py` | new: real-world units -> settings struct -> board |
| `sim/geo_tb.v`, `host/check_geo.py` | new: geometric sweep test |

## Toolchain notes (found on the real chip)

- **Hardware multipliers (DSP):** Apycula 0.33 crashes packing them; the upstream fix
  (commit 81b3a9e) is patched into the installed Apycula. They then pack, but compute
  **unsigned** on the chip, so this project builds with `-nodsp` (`tn9k.conf`).
- **Block RAM:** the deep, narrow modes (16K x 1, 4K x 4) store samples in the wrong
  places on the chip. The capture buffer is built from 1K banks instead. The 1K x 18 and
  2K x 9 modes were checked on the chip with a counter pattern and are fine.
