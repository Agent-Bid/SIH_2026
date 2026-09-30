# Adaptive sonar waveform generator (Tang Nano 9K + ESP32-S3)

An ESP32-S3 reads the water conditions, runs an acoustics model and picks a sonar waveform;
it sends the choice over SPI to a Tang Nano 9K FPGA, which generates the pulses sample by
sample at 50.14 MHz for an 8-bit DAC. Four modulations, 100–500 kHz: CW tone, LFM chirp,
geometric sweep and BPSK (Barker-13), each with a Hann window and an amplitude.

A PC can stand in for the ESP32 over the Tang Nano's USB UART, and can capture pulses back
from the FPGA to check them sample for sample against a Python golden model.

**Full project details** (design, packet format, pins, toolchain quirks, status, open items):
[`AGENTS.md`](AGENTS.md). ESP32 firmware and its physics write-up: `MCU /SIH/`.

## Use it

```
tn9k load                                                        # build + load the FPGA

python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --capture    # LFM 150 -> 500 kHz
python3 host/wavegen.py geo   --fc 325e3 --bw 350e3 --capture    # geometric 150 -> 500 kHz
python3 host/wavegen.py bpsk  --fc 300e3 --capture               # Barker-13 at 300 kHz
python3 host/wavegen.py tone  --fc 200e3 --amp 0.75 --capture

python3 host/capture.py --request                                # capture what is playing now
```

- `--capture` asks the board for the next pulse, checks it against the golden model and saves
  a plot in `sim/out/board_capture_<kind>.png` (waveform + spectrogram with the expected sweep).
- `wavegen.py` pulses are 13,000 clocks (259 µs); sweeps go up from `fc - bw/2` to
  `fc + bw/2`; BPSK uses Barker-13 at 1,000 clocks per chip. Other options: `--amp 0..1`,
  `--period <clocks>`, `--port`, `--dry-run`.
- With the ESP32 connected, its packets (10 per second) replace the PC's settings; use
  `capture.py --request` to see what it chose. On the ESP32's serial port, `M geo`,
  `M bpsk`, `M lfm`, `M cw` or `M auto` picks the modulation, `R 450` the target range.
- On the board: **S1** steps through three presets (chirp, geometric, BPSK), **S2** captures
  the next pulse (listen with `python3 host/capture.py`).
- **Oscilloscope** (no DAC needed): pin 40 → 1 kΩ → probe point, 100 pF from the probe point
  to GND; trigger on pin 41 (high during each pulse). Without any parts, pin 34 shows a square
  wave at the waveform's frequency (sweep and BPSK phase jumps visible, window not).
- LEDs: 0 BPSK, 1 armed, 2 recording/sending, 3 blinks per good SPI packet, 4 pulse active,
  5 geometric.

## Test in simulation

```
sim/all        # 10 testbenches, ALL PASS
```

`sim/top_tb.v` simulates the whole board: settings over the UART, a pretend ESP32 on SPI
(including a packet with a broken CRC, which must be rejected), and three captures checked
by `host/capture.py`.

## Verified on the board

- 2026-09-29: with the 57-byte packet format, LFM chirp, geometric sweep, BPSK and a tone at
  half amplitude all matched the golden model on all 16,383 captured samples (settings sent
  from the PC over the UART).
- 2026-09-30: the full chain on the boards. The ESP32 (physics + neural network) sends its
  choice over SPI 10 times a second (0 bad packets), and the FPGA plays it bit-exact: an H-S
  chirp at 300 m, then an M-S geometric sweep after `R 600` and `M geo`. Packets are framed by
  the pause after each burst instead of CS (the CS wire does not work on these boards; see
  `AGENTS.md`).
- 2026-09-30: with the oscilloscope outputs added and `scale` split into two clocks (the
  single-clock multiply failed on the chip), ESP32 and PC captures were all bit-exact. This
  build is in the FPGA's flash.
