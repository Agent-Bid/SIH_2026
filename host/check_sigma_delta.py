"""Check the 1-bit DAC (sigma_delta.v): both bit streams (first and second order) are exactly
the modulators' models, and after a filter they follow the waveform. Saves sim/out/sigma_delta.png.

Run from the project root, after `sim/run sigma_delta_tb`:   python3 host/check_sigma_delta.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from model import F_CLK

rows = np.loadtxt("sim/out/sigma_delta.txt", dtype=int)
sample, bits1, bits2 = rows[:, 0], rows[:, 1], rows[:, 2]


def wrap(v, n):
    return (v + (1 << (n - 1))) % (1 << n) - (1 << (n - 1))


# 1. bit-exact. Row j is logged after clock j; row 0 is the moment the reset ends (state 0).
#    First order, clock j: out <= acc[12]; acc <= acc[11:0] + level(sample logged in row j-1).
level = sample + 2048
acc, expect1 = 0, [0]
for j in range(1, len(level)):
    expect1.append(acc >> 12)
    acc = (acc & 0xFFF) + level[j - 1]
#    Second order, clock j: x = the sample of row j-1 limited to +-1945, fb = +-2048 for the last
#    bit; i1 += x - fb; i2 += i1 - fb; out = (i2 >= 0)
i1 = i2 = 0
expect2 = [0]
for j in range(1, len(sample)):
    x = max(-1945, min(1945, int(sample[j - 1])))
    fb = 2048 if expect2[-1] else -2048
    i1 = wrap(i1 + x - fb, 16)
    i2 = wrap(i2 + i1 - fb, 20)
    expect2.append(int(i2 >= 0))
bad1 = int(np.sum(np.array(expect1) != bits1))
bad2 = int(np.sum(np.array(expect2) != bits2))

# 2. the bench filter: first-order RC, tau = 1 kOhm x 100 pF = 100 ns, on a 3.3 V output
VDD, tau = 3.3, 1e3 * 100e-12
a = np.exp(-1 / (F_CLK * tau))


def rc(bits, stages):
    """The bits (0 / 3.3 V) through `stages` identical 1 kOhm + 100 pF stages (unloaded)."""
    v = VDD * bits.astype(float)
    for _ in range(stages):
        out, y = np.empty_like(v), VDD / 2
        for i, x in enumerate(v):
            y = a * y + (1 - a) * x
            out[i] = y
        v = out
    return v


ideal = VDD * level / 4096.0
k = np.arange(520, 15520)                      # where the pulse is
peak = np.max(np.abs(ideal[k] - VDD / 2))
print(f"bit streams: first order {bad1}, second order {bad2} of {len(bits1)} bits differ from the models")
ripple = {}
for stages in (1, 2):
    for order, bits in ((1, bits1), (2, bits2)):
        f = rc(bits, stages)
        ref = rc(ideal / VDD, stages)          # the ideal waveform through the same filter
        e = f[k] - ref[k]
        ripple[(order, stages)] = np.sqrt(np.mean(e ** 2))
    print(f"after {stages} x (1 kOhm + 100 pF): residual noise, first order "
          f"{ripple[(1, stages)] * 1000:.0f} mV rms, second order {ripple[(2, stages)] * 1000:.0f} mV rms "
          f"(on a {2 * peak:.2f} V p-p waveform)")
filt, bits = rc(bits2, 1), bits2
rms = ripple[(2, 1)]

t = np.arange(len(bits)) / F_CLK * 1e6
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
ax1.plot(t, filt, lw=0.5, label="second order after 1 kOhm + 100 pF")
ax1.plot(t, ideal, lw=0.5, alpha=0.7, label="ideal waveform (for comparison)")
ax1.set(title="What the oscilloscope shows: windowed LFM chirp 150 -> 500 kHz", xlabel="time (µs)", ylabel="V")
ax1.legend(loc="upper right")
z = (t > 150) & (t < 170)
ax2.plot(t[z], filt[z], lw=1, label="filtered")
ax2.plot(t[z], ideal[z], lw=1, alpha=0.7, label="ideal")
ax2.step(t[z], 0.2 + 0.3 * bits[z], lw=0.4, label="raw bits (scaled)")
ax2.set(title="Zoom: 20 µs", xlabel="time (µs)", ylabel="V")
ax2.legend(loc="upper right")
fig.tight_layout()
fig.savefig("sim/out/sigma_delta.png", dpi=110)
print("plot saved to sim/out/sigma_delta.png")

if bad1 == 0 and bad2 == 0 and rms / peak < 0.15:
    print("\nPASS: both modulators bit-exact, and the filtered output follows the waveform")
else:
    print("\nFAIL")
    sys.exit(1)
