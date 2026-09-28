"""Check the DDS simulation against a Python golden model, and plot it.

1. Reads sim/out/dds_samples.txt (written by sim/dds_tb.v).
2. Works out what every sample *should* be, using the same maths as the hardware.
3. Says PASS only if every sample matches exactly.
4. Saves sim/out/dds.png: the waveform, and its spectrum (FFT).

Run from the project root, after `sim/run dds_tb`:   python3 host/check_dds.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")                      # draw to a file, no window needed
import matplotlib.pyplot as plt
from gen_sine import sine_table

F_CLK = 27e6 * 13 / 7                      # ~50.142857 MHz (the PLL we'll build)

def golden_dds(ftw, n):
    """What the hardware should output: table[top 10 bits of (k * ftw mod 2^32)]."""
    k = np.arange(n, dtype=np.uint64)
    phase = (k * ftw) % 2**32
    return sine_table()[phase >> 22]

# ---- 1. read the simulation output
with open("sim/out/dds_samples.txt") as f:
    ftw = int(f.readline().split("=")[1])
    lines = [line.strip() for line in f]
if any(not v.lstrip("-").isdigit() for v in lines):
    first = next(i for i, v in enumerate(lines) if not v.lstrip("-").isdigit())
    print(f"FAIL: sample {first} is '{lines[first]}' (x = unknown: nothing is driving 'sample' yet)")
    sys.exit(1)
sim = np.array([int(v) for v in lines])

# ---- 2 + 3. compare with the golden model
gold = golden_dds(ftw, len(sim))
bad = np.nonzero(sim != gold)[0]
f_expected = ftw * F_CLK / 2**32

# ---- 4. spectrum: which frequencies are in the signal?
win = np.hanning(len(sim))                 # softens the edges of the capture for a cleaner FFT
spec = np.abs(np.fft.rfft(sim * win))
spec_db = 20 * np.log10(spec / spec.max() + 1e-12)
freqs = np.fft.rfftfreq(len(sim), 1 / F_CLK)
f_peak = freqs[np.argmax(spec)]
bin_width = F_CLK / len(sim)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
t_us = np.arange(200) / F_CLK * 1e6
ax1.plot(t_us, sim[:200], ".-", label="simulation")
ax1.plot(t_us, gold[:200], "--", alpha=0.6, label="golden model")
ax1.set(title=f"DDS output, ftw={ftw} -> {f_expected/1e6:.6f} MHz",
        xlabel="time (µs)", ylabel="sample value")
ax1.legend()
ax2.plot(freqs / 1e6, spec_db)
ax2.set(title=f"Spectrum: peak at {f_peak/1e6:.4f} MHz", xlabel="frequency (MHz)",
        ylabel="dB (0 = peak)", ylim=(-120, 5))
ax2.grid(alpha=0.3)
fig.tight_layout()
fig.savefig("sim/out/dds.png", dpi=110)

print(f"samples: {len(sim)}   expected f_out: {f_expected/1e6:.6f} MHz   "
      f"FFT peak: {f_peak/1e6:.4f} MHz")
print("plot saved to sim/out/dds.png")
if len(bad) == 0 and abs(f_peak - f_expected) <= bin_width:
    print("\nPASS: every sample matches the golden model")
else:
    for i in bad[:10]:
        print(f"  sample {i}: sim={sim[i]}  expected={gold[i]}")
    print(f"\nFAIL: {len(bad)} of {len(sim)} samples differ")
    sys.exit(1)
