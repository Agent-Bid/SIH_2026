# Adaptive sonar waveform generator (Tang Nano 9K + ESP32-S3)

An ESP32-S3 takes the water conditions (simulated for now), runs an acoustics model and three
small neural networks, and picks a sonar waveform for every ping, 6 a second: the frequency
band and pulse length, the amplitude, and the modulation (BPSK, LFM or geometric, from how fast
the target moves). It sends the choice over SPI to a Tang Nano 9K FPGA, which generates the
pulses sample by sample at 50.14 MHz for an 8-bit DAC, each with a Hann window.

A PC can stand in for the ESP32 over the Tang Nano's USB UART, and can capture pulses back
from the FPGA (one, or every ping) to check them sample for sample against a Python golden
model.

**Full project details** (design, packet format, pins, toolchain quirks, status, open items):
[`AGENTS.md`](AGENTS.md). ESP32 firmware and its physics write-up: `MCU /SIH/`.

## Use it

**Demo**: power the ESP32 (it starts pinging on its own), plug the Tang Nano into the PC, then

```
python3 host/demo.py                  # one second: 6 pings on one graph, with the conditions behind each
```

It streams 6 consecutive pings from the FPGA, checks each against the golden model, and plots
them at their real times over one second (grey = baseline, no output), with a table of the
water conditions, the waveform chosen, what the neural networks picked, and the check result
(`sim/out/demo.png`). The ESP32's USB log is used if it is connected; otherwise the conditions
are recomputed from each packet's number (`host/water_sim.py`).

**Other tools**:

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
  Long pulses are captured whole by keeping one sample in 4, 16 or 64.
- `wavegen.py` pulses are 13,000 clocks (259 µs); sweeps go up from `fc - bw/2` to
  `fc + bw/2`; BPSK uses Barker-13 at 1,000 clocks per chip. Other options: `--amp 0..1`,
  `--period <clocks>`, `--port`, `--dry-run`.
- With the ESP32 connected, its packets (one per ping) replace the PC's settings; use
  `capture.py --request` to see what it chose. The ESP32's serial console
  (`python3 host/esp_console.py`): `A` automatic (simulated conditions, the default),
  `W` type the inputs, `M geo|bpsk|lfm|cw|auto` force a modulation, `R 450` the target range.
- The PC talks to the FPGA at 3 Mbaud (`/dev/ttyUSB1`).
- **The DAC output without a scope**: wire pin 40 → 1 kΩ + 100 pF → 10 kΩ + 10 pF → ESP32
  GPIO 1, then `python3 host/dac_view.py --amp 1`. The ESP32 records the filtered voltage while
  the FPGA plays a slow-motion copy of the current ping, and the plot lays it over the FPGA's own
  samples (`sim/out/dac_view.png`). `python3 host/dac_sequence.py` does it for one second of
  pings (6 decisions, each recorded 4× and averaged) on one graph (`sim/out/dac_sequence.png`).
- On the board: **S1** steps through three presets (chirp, geometric, BPSK), **S2** captures
  the next pulse (listen with `python3 host/capture.py`).
- **Oscilloscope** (no DAC needed): pin 40 → 1 kΩ → probe point, 100 pF from the probe point
  to GND; trigger on pin 41 (high during each pulse). Without any parts, pin 34 shows a square
  wave at the waveform's frequency (sweep and BPSK phase jumps visible, window not).
- LEDs: 0 BPSK, 1 armed, 2 recording/sending, 3 blinks per good SPI packet, 4 pulse active,
  5 geometric.

## Test in simulation

```
sim/all        # 11 testbenches, ALL PASS
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
  single-clock multiply failed on the chip), ESP32 and PC captures were all bit-exact.
- 2026-09-30: the demo: simulated conditions on the ESP32, 6 pings a second, each a new
  decision; 6 consecutive pings streamed from the FPGA, all bit-exact. This build is in the
  FPGA's flash.
- 2026-10-04: the sigma-delta DAC's filtered output, recorded by the ESP32's ADC in slow motion,
  matches the FPGA's samples (correlation 0.998 at full amplitude; BPSK phase flips visible).
