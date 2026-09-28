# Waveform generator: finished reference (Tang Nano 9K)

A working copy of the learning project in `../Basic`, with every piece filled in.
The PC stands in for the MCU: it sends the waveform settings over the USB UART,
the FPGA generates windowed pulses, and it can capture one pulse and send it back
to be checked against the Python golden model sample for sample.

Verified on the board on 2026-09-28: chirp up/down, BPSK Barker-13 and Barker-7,
and a tone at 0.75 amplitude all matched the golden model on all 16,383 samples.

## Use it

```
tn9k load                                   # build + load into the FPGA (volatile)

python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --len 16000 --capture
python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --down --capture
python3 host/wavegen.py bpsk  --fc 300e3 --chip-len 1000 --code barker13 --capture
python3 host/wavegen.py bpsk  --fc 250e3 --chip-len 150 --code barker7 --amp 0.5 --capture
python3 host/wavegen.py tone  --fc 200e3 --len 12000 --amp 0.75 --capture
```

- `--capture` asks the board for the next pulse, checks it and saves a plot in
  `sim/out/board_capture_<kind>.png`. Without it, the settings are just sent.
- Other options: `--amp 0..1`, `--period` (clocks between pulse starts),
  `--code` (barker2..barker13, or a string like `++-+`), `--dry-run`.
- On the board: **S1** switches between two built-in presets (chirp 150 -> 500 kHz,
  BPSK at 300 kHz). **S2** captures the next pulse (listen with `python3 host/capture.py`).
- LEDs: 0 BPSK, 1 armed, 2 recording, 3 sending, 4 pulse active, 5 PLL locked.
- DAC pins (for later): `dac_d[7:0]` on pins 25-30, 33, 34 and `dac_clk` on 35 (3.3 V).

## Test in simulation

```
sim/all        # 8 testbenches
```

`sim/top_tb.v` simulates the whole board: it sends the frames in `sim/top_frames.hex`
(made by `wavegen.py --frames-out`) and checks both captures it gets back.

## Settings struct (24 bytes, little-endian)

The PC sends `A5 01 <24 bytes> <xor checksum>`, or `A5 02 02` to capture.
The MCU will later send the same struct over SPI.

| Bytes | Field | Meaning |
| --- | --- | --- |
| 0-1 | len | pulse length in clocks (max 65535 = 1.3 ms) |
| 2-5 | ftw_start | starting frequency: f x 2^32 / 50.142857 MHz |
| 6-9 | ftw_step | frequency change per clock (two's complement; 0 = tone) |
| 10-13 | win_step | 2^32 / len (Hann window, one lap per pulse) |
| 14-15 | code | phase code, bit i = chip i, 1 = flip |
| 16-17 | chip_len | clocks per chip |
| 18-19 | amp | amplitude, 4096 = 1.0 |
| 20-23 | period | clocks between pulse starts |

New settings wait until no pulse is playing (double buffering), so a pulse is never
changed halfway through.

## What differs from the learning folder

| File | Change |
| --- | --- |
| `pulse_dds.v` | pipeline register before the first multiply |
| `uart_tx.v` | filled in |
| `uart_rx.v`, `cmd_rx.v` | new: receive settings and capture commands |
| `top.v` | settings from the PC, double-buffered, two presets |
| `capture.v` | header frozen at pulse start (same banked RAM as the learning folder) |
| `host/wavegen.py` | new: real-world units -> settings struct -> board |

## Toolchain notes (found on the real chip)

- **Hardware multipliers (DSP):** Apycula 0.33 crashes packing them; the upstream fix
  (commit 81b3a9e) is patched into the installed Apycula. They then pack, but compute
  **unsigned** on the chip, so this project builds with `-nodsp` (`tn9k.conf`).
- **Block RAM:** the deep, narrow modes (16K x 1, 4K x 4) store samples in the wrong
  places on the chip. The capture buffer is built from 1K banks instead. The 1K x 18 and
  2K x 9 modes were checked on the chip with a counter pattern and are fine.
