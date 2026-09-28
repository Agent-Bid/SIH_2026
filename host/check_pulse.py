"""Check the windowed-pulse simulation against a Python golden model, and plot it.

1. Reads sim/out/pulse_samples.txt (written by sim/pulse_dds_tb.v).
2. Replays the same inputs through a tick-by-tick Python copy of the hardware.
3. Says PASS only if every output sample matches exactly.
4. Saves sim/out/pulse.png: the two pulses, and the spectrum with vs without the window.

Run from the project root, after `sim/run pulse_dds_tb`:   python3 host/check_pulse.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from model import F_CLK, read_log, golden, pulses, compare

def spectrum_db(x, n_fft=1 << 16):
    s = np.abs(np.fft.rfft(x, n_fft))
    return np.fft.rfftfreq(n_fft, 1 / F_CLK), 20 * np.log10(s / s.max() + 1e-12), s ** 2

def occupied_bw(freqs, power, frac=0.99):
    """Width of the frequency band that holds 99% of the energy."""
    c = np.cumsum(power) / power.sum()
    lo = freqs[np.searchsorted(c, (1 - frac) / 2)]
    hi = freqs[np.searchsorted(c, 1 - (1 - frac) / 2)]
    return hi - lo

# ---- 1. read the simulation output
length, win_step, inputs, sim_out, sim_act = read_log("sim/out/pulse_samples.txt")

# ---- 2 + 3. compare with the golden model
gold_out, gold_act = golden(inputs, length, win_step)
n_bad = compare(sim_out, sim_act, gold_out, gold_act)
found = pulses(sim_act)
starts = [p for p, _ in found]
ends = [p + n for p, n in found]
lengths = [n for _, n in found]

# ---- 4. with vs without the window (same pulse, window forced to 1.0 everywhere)
rect_out, _ = golden(inputs, length, win_step, window=np.full(1024, 4096))
p1 = slice(starts[0], starts[0] + length + 10) if len(starts) else slice(0, 0)
f, db_win, pw_win = spectrum_db(sim_out[p1])
_, db_rect, pw_rect = spectrum_db(rect_out[p1])
bw_win, bw_rect = occupied_bw(f, pw_win), occupied_bw(f, pw_rect)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
t_us = np.arange(len(sim_out)) / F_CLK * 1e6
ax1.plot(t_us, sim_out, lw=0.6, label="simulation")
ax1.plot(t_us, sim_act * 2200, "--", lw=1, label="active (scaled)")
ax1.set(title=f"Windowed pulses: {length} ticks each, amp 1.0 then 0.5",
        xlabel="time (µs)", ylabel="sample value")
ax1.legend(loc="upper right")
ax2.plot(f / 1e6, db_rect, lw=0.8, color="tab:red", alpha=0.7,
         label=f"no window   (Energy in {bw_rect/1e3:.0f} kHz)")
ax2.plot(f / 1e6, db_win, lw=1.2, color="tab:blue",
         label=f"Hann window (Energy in {bw_win/1e3:.0f} kHz)")
ax2.set(title="Spectrum of one pulse", xlabel="frequency (MHz)", ylabel="dB (0 = peak)",
        xlim=(0, 2), ylim=(-100, 5))
ax2.grid(alpha=0.3)
ax2.legend(loc="upper right")
fig.tight_layout()
fig.savefig("sim/out/pulse.png", dpi=110)

print(f"ticks: {len(sim_out)}   pulses seen: {len(lengths)}   lengths: {lengths}")
print(f"peak |out| per pulse: {[int(np.abs(sim_out[s:e+4]).max()) for s, e in zip(starts, ends)]}")
print(f"99% bandwidth: no window {bw_rect/1e3:.0f} kHz  ->  Hann window {bw_win/1e3:.0f} kHz")
print("plot saved to sim/out/pulse.png")
if n_bad == 0 and lengths == [length, length]:
    print("\nPASS: every sample matches the golden model")
else:
    print(f"\nFAIL: {n_bad} ticks differ; pulse lengths {lengths}, expected [{length}, {length}]")
    sys.exit(1)
