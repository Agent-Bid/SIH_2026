# Waveform generator — finished demo (Tang Nano 9K)

A working copy of the learning project with every piece filled in. The PC plays the MCU:
`host/wavegen.py` turns real-world settings into the settings struct, sends it over the USB
UART, and can capture one pulse back and check it sample-for-sample against the golden model.

Verified on the board on 2026-09-28: chirp up and down, BPSK Barker-13 and Barker-7, and a
plain tone, each 16,383 captured samples bit-exact.

## Use it

```
tn9k load                                   # build and load (SRAM; lost at power-off)
python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --len 16000 --capture
python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --down --capture
python3 host/wavegen.py bpsk  --fc 300e3 --chip-len 1000 --code barker13 --capture
python3 host/wavegen.py bpsk  --fc 250e3 --chip-len 150 --code barker7 --amp 0.5 --capture
python3 host/wavegen.py tone  --fc 200e3 --len 12000 --capture
```

- `--capture` records the next pulse, downloads it (~3 s), prints PASS/FAIL and saves a plot
  in `sim/out/board_capture_<kind>.png`. Without it, the settings are just sent.
- `--dry-run` prints the frame without sending. `--help` lists every option.
- Buttons: S1 switches between two built-in presets (chirp 150→500 kHz / BPSK 300 kHz);
  S2 captures the next pulse (then run `python3 host/capture.py`).
- LEDs: 0 BPSK, 1 armed, 2 recording, 3 sending, 4 pulse active, 5 PLL locked.
- DAC pins (`dac_d[7:0]` 25–30, 33, 34; `dac_clk` 35) carry the offset-binary samples.

## Settings struct (24 bytes, little-endian) — sent as `A5 01 <struct> <xor>`

| Field | Bytes | Meaning |
| --- | --- | --- |
| len | 2 | pulse length, clocks |
| ftw_start | 4 | starting FTW = f × 2^32 / 50,142,857 |
| ftw_step | 4 | FTW change per clock (two's complement; 0 = tone) |
| win_step | 4 | 2^32 / len |
| code | 2 | phase code, chip i = bit i, 1 = flip |
| chip_len | 2 | clocks per chip |
| amp | 2 | amplitude, 4096 = 1.0 |
| period | 4 | clocks between pulse starts |

`A5 02 02` = capture the next pulse. New settings take effect between pulses.

## Tests

`sim/all` runs every testbench. `sim/top_tb.v` simulates the whole board: it sends the frames
from `sim/top_frames.hex` (made by `wavegen.py --frames-out`) and checks both captures.

## Toolchain notes (Apycula 0.33)

- `gowin_pack` crashes on the hardware multipliers. The upstream fix (apicula 81b3a9e) is
  patched into the installed copy, but on the chip the multipliers then come out **unsigned**,
  so this project builds multiplies from logic: `SYNTH_FLAGS=-nodsp` in `tn9k.conf`.
- Block RAM in the deep, narrow modes (16K×1, 4K×4) corrupts data on the chip, while 1K×18
  and 2K×9 work. `capture.v` therefore builds its buffer from 1K banks.
