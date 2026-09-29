"""Check the geometric sweep simulation: exact match with the golden model, and the
frequency measured from the samples grows by the same ratio every step (a straight
line on a log-frequency axis). Saves sim/out/geo.png.

Run from the project root, after `sim/run geo_tb`:   python3 host/check_geo.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from scipy.signal import hilbert
from model import F_CLK, hz, read_log, golden, geo_ftw, pulses, compare

length, win_step, inputs, sim_out, sim_act = read_log("sim/out/geo_samples.txt")
gold_out, gold_act = golden(inputs, length, win_step, geo=True)
n_bad = compare(sim_out, sim_act, gold_out, gold_act)
found = pulses(sim_act)

# Independent check: measure the frequency from the samples (slope of the analytic
# signal's phase), smoothed over 64 ticks, in the middle 80% of the pulse.
p0, n = found[0]
ftw_start, ratio = inputs[p0][1], inputs[p0][2]
expect = hz(geo_ftw(ftw_start, ratio, n))
seg = sim_out[p0:p0 + n].astype(float)
inst_f = np.diff(np.unwrap(np.angle(hilbert(seg)))) * F_CLK / (2 * np.pi)
inst_f = np.convolve(inst_f, np.ones(64) / 64, mode="same")
k = np.arange(int(0.1 * n), int(0.9 * n))
err = np.max(np.abs(inst_f[k] / expect[k] - 1))
slope = np.polyfit(k, np.log2(inst_f[k]), 1)[0]           # octaves per tick
octaves = slope * n
ok_sweep = err < 0.02 and abs(octaves - 2) < 0.05
print(f"expected {expect[0]/1e6:.3f} -> {expect[-1]/1e6:.3f} MHz (2 octaves); "
      f"measured {octaves:.3f} octaves, worst frequency error {err*100:.2f}%  "
      f"{'ok' if ok_sweep else 'WRONG'}")

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
t_us = np.arange(n) / F_CLK * 1e6
ax1.plot(t_us, seg, lw=0.4)
ax1.set(title="Windowed geometric sweep, 2 -> 8 MHz", xlabel="time (µs)", ylabel="sample value")
ax2.semilogy(t_us[k], inst_f[k] / 1e6, lw=1, label="measured from the samples")
ax2.semilogy(t_us, expect / 1e6, "k--", lw=0.8, label="expected")
ax2.set(title="Frequency (log axis: a geometric sweep is a straight line)",
        xlabel="time (µs)", ylabel="frequency (MHz)")
ax2.yaxis.set_major_formatter(ScalarFormatter())
ax2.yaxis.set_minor_formatter(ScalarFormatter())
ax2.legend()
fig.tight_layout()
fig.savefig("sim/out/geo.png", dpi=110)
print("plot saved to sim/out/geo.png")

if n_bad == 0 and ok_sweep and [m for _, m in found] == [length]:
    print("\nPASS: every sample matches the golden model, and the sweep is geometric")
else:
    print(f"\nFAIL: {n_bad} ticks differ; pulses {found}; sweep ok: {ok_sweep}")
    sys.exit(1)
