"""Check the chirp simulation: exact match with the golden model, and the frequency
really sweeps in a straight line from start to end. Saves sim/out/chirp.png.

Run from the project root, after `sim/run chirp_tb`:   python3 host/check_chirp.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import hilbert, spectrogram
from model import F_CLK, hz, read_log, golden, pulses, compare

length, win_step, inputs, sim_out, sim_act = read_log("sim/out/chirp_samples.txt")
gold_out, gold_act = golden(inputs, length, win_step)
n_bad = compare(sim_out, sim_act, gold_out, gold_act)
found = pulses(sim_act)

# Independent check: measure the frequency from the samples themselves.
# The phase of the analytic signal climbs at 2*pi*f per second; its slope is the frequency.
# Only the middle 80% of each pulse is used (the window makes the edges too quiet to measure).
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
ok_sweep = True
for p0, n in found:
    ftw_start, ftw_step = inputs[p0][1], inputs[p0][2]
    ftw_step = ftw_step - 2**32 if ftw_step >= 2**31 else ftw_step
    f_start, f_end = hz(ftw_start), hz(ftw_start + ftw_step * n / 2**16)   # step is FTW/clock x 2^16
    seg = sim_out[p0:p0 + n].astype(float)
    k = np.arange(int(0.1 * n), int(0.9 * n))
    inst_f = np.diff(np.unwrap(np.angle(hilbert(seg)))) * F_CLK / (2 * np.pi)
    slope, f0 = np.polyfit(k, inst_f[k], 1)
    m_start, m_end = f0, f0 + slope * n
    ok = abs(m_start - f_start) < 50e3 and abs(m_end - f_end) < 50e3
    ok_sweep &= ok
    print(f"pulse at tick {p0}: expected {f_start/1e6:.3f} -> {f_end/1e6:.3f} MHz, "
          f"measured {m_start/1e6:.3f} -> {m_end/1e6:.3f} MHz  {'ok' if ok else 'WRONG'}")
    t = (p0 + np.array([0, n])) / F_CLK * 1e6
    ax2.plot(t, [f_start / 1e6, f_end / 1e6], "w--", lw=1)

t_us = np.arange(len(sim_out)) / F_CLK * 1e6
ax1.plot(t_us, sim_out, lw=0.4)
ax1.set(title="Two windowed LFM chirps: up (3 -> 7 MHz), then down (7 -> 3 MHz)",
        xlabel="time (µs)", ylabel="sample value")
f, t, S = spectrogram(sim_out.astype(float), fs=F_CLK, window="hann",
                      nperseg=256, noverlap=248, nfft=1024)
S_db = 10 * np.log10(S / S.max() + 1e-12)
ax2.pcolormesh(t * 1e6, f / 1e6, S_db, shading="auto", vmin=-30, vmax=0, cmap="magma")
ax2.set(title="Spectrogram (dashed = expected)", xlabel="time (µs)",
        ylabel="frequency (MHz)", ylim=(0, 10))
fig.tight_layout()
fig.savefig("sim/out/chirp.png", dpi=110)
print("plot saved to sim/out/chirp.png")

lengths = [n for _, n in found]
if n_bad == 0 and ok_sweep and lengths == [length, length]:
    print("\nPASS: every sample matches the golden model, and both sweeps are correct")
else:
    print(f"\nFAIL: {n_bad} ticks differ; pulse lengths {lengths}; sweeps ok: {ok_sweep}")
    sys.exit(1)
