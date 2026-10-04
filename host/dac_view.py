"""Before and after the DAC, without an oscilloscope.

The ESP32 records the sigma-delta DAC's filtered output (pin 40 -> 1 kOhm + 100 pF -> 10 kOhm +
10 pF -> ESP32 GPIO 1) with its ADC while the FPGA plays a slow-motion copy of the current ping
(every frequency divided by N, the pulse N times longer; serial command C, see adaptive_sonar.ino).
This program plots that measured voltage ("after the DAC") over the FPGA's own samples for the
same packet from the golden model ("before the DAC"), lined up.

  python3 host/dac_view.py              # ESP32 on /dev/ttyACM0
  python3 host/dac_view.py --no-show    # only save sim/out/dac_view.png

The time axis is the real pulse's time; the recording itself ran N times slower. The filter's
effect at the real 100-500 kHz is not shown (at the slow frequencies it passes everything).
"""
import argparse
import os
import sys
import time
import numpy as np
import serial
from scipy.signal import correlate

from model import F_CLK, golden

VDD = 3.3
MOD_NAME = {0: "tone", 1: "LFM chirp", 2: "geometric sweep", 3: "BPSK (Barker-13)"}


def capture(port, amp):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = s.rts = False                                  # toggling these resets the ESP32-S3
    s.open()
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(f"C {amp:g}\n".encode() if amp else b"C\n")
    text, deadline = "", time.time() + 30
    while time.time() < deadline and "\nEND" not in text:
        text += s.read(65536).decode(errors="replace")
        if "CAPERR" in text:
            raise SystemExit("ESP32: " + text[text.find("CAPERR"):].splitlines()[0])
    s.close()
    if "\nEND" not in text:
        raise SystemExit("no capture from the ESP32 within 30 s (is it running the C command build?)")
    head = next(l for l in text.splitlines() if l.startswith("CAP "))
    info = dict(kv.split("=", 1) for kv in head.split()[1:])
    hexes = "".join(l[2:].strip() for l in text.splitlines() if l.startswith("D "))
    mv = np.array([int(hexes[i:i + 3], 16) for i in range(0, len(hexes), 3)], dtype=float)
    return info, mv


def fpga_samples(info, step):
    """The FPGA's output for this packet, one value every `step` clocks, as the voltage the DAC aims for."""
    length = int(info["len"])
    p = (int(info["ftw_start"]), int(info["ftw_step"]) % 2**32, min(int(info["amp"]), 4096),
         int(info["code"]), int(info["chip"]))
    ticks = ((1 if k == 0 else 0,) + p for k in range(length + 8))
    out, _ = golden(ticks, length, int(info["win_step"]), geo=(info["mod"] == "2"), every=step)
    return (np.asarray(out, dtype=float) + 2048) / 4096 * VDD


def align(after, before, before_dt, rate, t_pulse):
    """Find the ADC's true sample rate (within +-5 %) and where the pulse starts in the recording,
    by correlating the measured voltage with the expected one."""
    a = after - after.mean()
    best = None
    for r in rate * np.linspace(0.95, 1.05, 101):
        m = int(t_pulse * r)
        if m >= len(a):
            continue
        tmpl = np.interp(np.arange(m) / r, np.arange(len(before)) * before_dt, before) - VDD / 2
        c = correlate(a, tmpl, mode="valid", method="fft")
        lag = int(np.argmax(c))
        seg = a[lag:lag + m] - a[lag:lag + m].mean()
        score = float(np.dot(seg, tmpl) / (np.linalg.norm(seg) * np.linalg.norm(tmpl) + 1e-12))
        if best is None or score > best[0]:
            best = (score, r, lag, m)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp", default="/dev/ttyACM0", help="the ESP32's USB port")
    ap.add_argument("--amp", type=float, help="play the slow copy at this amplitude 0..1 (default: the ESP32's own)")
    ap.add_argument("--no-show", action="store_true", help="only save the picture")
    a = ap.parse_args()

    print("recording the DAC output on the ESP32 (slow motion)...")
    info, mv = capture(a.esp, a.amp)
    n_slow, rate = int(info["slow"]), float(info["meas"])
    t_pulse = int(info["len"]) / F_CLK                     # seconds, slow motion
    step = max(1, int(F_CLK / (4 * rate)))                 # 4 FPGA samples per ADC sample is plenty
    print(f"{len(mv)} ADC samples at ~{rate:.0f} S/s; computing the FPGA's samples for the same packet...")
    before = fpga_samples(info, step)
    score, r, lag, m = align(mv / 1000, before, step / F_CLK, rate, t_pulse)
    after = mv[lag:lag + m] / 1000
    t_us = np.arange(m) / r / n_slow * 1e6                # real-pulse time
    expect = np.interp(np.arange(m) / r, np.arange(len(before)) * step / F_CLK, before)
    offset = after.mean() - expect.mean()                  # the ESP32 ADC's own DC error
    after = after - offset
    rms = float(np.sqrt(np.mean((after - after.mean() - (expect - expect.mean())) ** 2))) * 1000
    print(f"match: correlation {score:.3f}, difference {rms:.0f} mV rms; "
          f"{info['profile']} {MOD_NAME.get(int(info['mod']), '?')}, amplitude {int(info['amp']) / 4096:.2f}, "
          f"slow motion x{n_slow}")

    import matplotlib
    if a.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), gridspec_kw=dict(height_ratios=[1.2, 1]))
    c_before, c_after = "#2a78d6", "#eb6834"
    ax1.plot(t_us, expect, lw=1.0, color=c_before, label="before the DAC: the FPGA's samples (golden model)")
    ax1.plot(t_us, after, lw=0.5, color=c_after, alpha=0.75,
             label=f"after the DAC: measured on ESP32 GPIO 1 (ADC offset {offset * 1000:+.0f} mV removed)")
    lo, hi = min(after.min(), expect.min()), max(after.max(), expect.max())
    ax1.set_ylim(lo - 0.05 * (hi - lo), hi + 0.3 * (hi - lo))
    ax1.set(xlabel="time in the real pulse (µs)", ylabel="volts",
            title=f"{info['profile']} {MOD_NAME.get(int(info['mod']), '?')}, amplitude {int(info['amp']) / 4096:.2f}: "
                  f"before vs after the 1-bit DAC + RC filter  (recorded {n_slow}× slowed)")
    ax1.legend(loc="upper right", fontsize=9)
    ax1.text(0.01, 0.04, f"correlation {score:.3f} · difference {rms:.0f} mV rms", transform=ax1.transAxes,
             fontsize=9, color="#52514e")
    # close-up: the middle of the pulse (for BPSK, around the first phase flip)
    chip_us = int(info["chip"]) / F_CLK / n_slow * 1e6
    centre = 5 * chip_us if int(info["code"]) else t_us[-1] / 2
    f0 = int(info["ftw_start"]) / 2**32 * F_CLK * n_slow   # real start frequency
    half = 6 / f0 * 1e6
    k = (t_us > centre - half) & (t_us < centre + half)
    ax2.plot(t_us[k], after[k], ".-", lw=0.8, ms=3, color=c_after, label="after the DAC (measured)")
    ax2.plot(t_us[k], expect[k], lw=1.4, color=c_before, label="before the DAC (FPGA samples)")
    ax2.set(xlabel="time in the real pulse (µs)", ylabel="volts",
            title="close-up" + (" at a BPSK phase flip" if int(info["code"]) else " mid-pulse"))
    lo, hi = min(after[k].min(), expect[k].min()), max(after[k].max(), expect[k].max())
    ax2.set_ylim(lo - 0.05 * (hi - lo), hi + 0.35 * (hi - lo))
    ax2.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    os.makedirs("sim/out", exist_ok=True)
    fig.savefig("sim/out/dac_view.png", dpi=110)
    print("plot saved to sim/out/dac_view.png")
    if not a.no_show:
        import signal
        signal.signal(signal.SIGINT, signal.SIG_DFL)          # Tk would otherwise swallow Ctrl+C
        print("close the plot window, press q in it, or Ctrl+C here to exit")
        plt.show()


if __name__ == "__main__":
    main()
