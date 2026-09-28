"""Check the BPSK simulation: exact match with the golden model, and the code can be
read back from the samples. Saves sim/out/bpsk.png.

Run from the project root, after `sim/run bpsk_tb`:   python3 host/check_bpsk.py
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import hilbert
from model import F_CLK, hz, read_log, golden, pulses, compare

length, win_step, inputs, sim_out, sim_act = read_log("sim/out/bpsk_samples.txt")
gold_out, gold_act = golden(inputs, length, win_step)
n_bad = compare(sim_out, sim_act, gold_out, gold_act)
found = pulses(sim_act)
if len(found) != 1:
    sys.exit(f"FAIL: expected 1 pulse, saw {len(found)}")
p0, n = found[0]
_, ftw, _, _, code, chip_len = inputs[p0]
n_chips = n // chip_len
sent = [(code >> i) & 1 for i in range(n_chips)]

# Independent check: remove the carrier and look at what phase is left.
# Unflipped chips sit near 0 degrees, flipped chips near 180.
seg = sim_out[p0:p0 + n + 3].astype(float)
t = np.arange(len(seg))
resid = np.angle(hilbert(seg) * np.exp(-2j * np.pi * hz(ftw) / F_CLK * t))
resid = np.angle(np.exp(1j * (resid - resid[chip_len // 2])))      # chip 0 = reference
read = []
for c in range(n_chips):
    mid = resid[c * chip_len + chip_len // 4: (c + 1) * chip_len - chip_len // 4]
    read.append(int(np.mean(np.cos(mid)) < 0))

# What the receiver sees: correlate with the code (a matched filter)
chips = np.array([-1 if b else 1 for b in read])
corr = np.correlate(chips, chips, mode="full")

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))
t_us = np.arange(len(seg)) / F_CLK * 1e6
ax1.plot(t_us, seg, lw=0.5)
for c in range(1, n_chips):
    ax1.axvline(c * chip_len / F_CLK * 1e6, color="gray", lw=0.5, ls=":")
ax1.set(title="BPSK pulse: 5 MHz, Barker-13 (dotted = chip boundaries)",
        xlabel="time (µs)", ylabel="sample value")
ax2.plot(t_us, np.mod(np.degrees(resid) + 90, 360) - 90, lw=0.8)    # -90..270, so 180 isn't split
ax2.set(title=f"Phase left after removing the carrier.  Sent: {''.join('-' if b else '+' for b in sent)}"
              f"   Read back: {''.join('-' if b else '+' for b in read)}",
        xlabel="time (µs)", ylabel="degrees", yticks=[-90, 0, 90, 180, 270])
ax3.stem(np.arange(-n_chips + 1, n_chips), corr)
ax3.set(title="Matched filter output: one tall peak (13), tiny sidelobes (1)",
        xlabel="shift (chips)", ylabel="correlation")
fig.tight_layout()
fig.savefig("sim/out/bpsk.png", dpi=110)

print(f"pulse: {n} ticks, {n_chips} chips of {chip_len} ticks, carrier {hz(ftw)/1e6:.3f} MHz")
print(f"sent:      {''.join('-' if b else '+' for b in sent)}")
print(f"read back: {''.join('-' if b else '+' for b in read)}")
print("plot saved to sim/out/bpsk.png")
if n_bad == 0 and read == sent and n == length:
    print("\nPASS: every sample matches the golden model, and the code reads back correctly")
else:
    print(f"\nFAIL: {n_bad} ticks differ; code read back {'ok' if read == sent else 'WRONG'}")
    sys.exit(1)
