"""Check the oscilloscope output (sigma_delta.v): the bit stream is exactly the first-order
sigma-delta of the samples, and after the bench filter (1 kOhm + 100 pF) it follows the
waveform. Saves sim/out/sigma_delta.png.

Run from the project root, after `sim/run sigma_delta_tb`:   python3 host/check_sigma_delta.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from model import F_CLK

rows = np.loadtxt("sim/out/sigma_delta.txt", dtype=int)
sample, bits = rows[:, 0], rows[:, 1]

# 1. bit-exact. Row j is logged after clock j; row 0 is the moment the reset ends (acc = 0).
#    Clock j does: out <= acc[12]; acc <= acc[11:0] + level(sample logged in row j-1).
level = sample + 2048
acc, expect = 0, [0]
for j in range(1, len(level)):
    expect.append(acc >> 12)
    acc = (acc & 0xFFF) + level[j - 1]
bad = int(np.sum(np.array(expect) != bits))

# 2. the bench filter: first-order RC, tau = 1 kOhm x 100 pF = 100 ns, on a 3.3 V output
VDD, tau = 3.3, 1e3 * 100e-12
a = np.exp(-1 / (F_CLK * tau))
v, filt = VDD / 2, []
for b in bits:
    v = a * v + (1 - a) * VDD * b
    filt.append(v)
filt = np.array(filt)
ideal = VDD * level / 4096.0
# compare where the pulse is, after the filter's own delay (~ tau)
lag = int(round(F_CLK * tau))
k = np.arange(520, 15520)
err = filt[k + lag] - ideal[k]
rms, peak = np.sqrt(np.mean(err ** 2)), np.max(np.abs(ideal[k] - VDD / 2))
print(f"bit stream: {bad} of {len(bits)} bits differ from the sigma-delta model")
print(f"after 1 kOhm + 100 pF: residual ripple {rms * 1000:.0f} mV rms on a {2 * peak:.2f} V p-p waveform "
      f"({rms / peak:.1%} of its amplitude)")

t = np.arange(len(bits)) / F_CLK * 1e6
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
ax1.plot(t, filt, lw=0.5, label="scope_sd after 1 kOhm + 100 pF")
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

if bad == 0 and rms / peak < 0.15:
    print("\nPASS: bit-exact sigma-delta, and the filtered output follows the waveform")
else:
    print("\nFAIL")
    sys.exit(1)
