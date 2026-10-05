"""Energy per ping: the adaptive choice against fixed sonars, over the ESP32's simulated mission.

Runs the simulated water conditions (host/water_sim.py, the same as the firmware) for one full
battery cycle (100 % down to 15 %, about 10,200 pings at 6 a second) and, for every ping,
compares the energy the adaptive rule spends (MCU /SIH/physics_reference.py, the same rule as
the firmware without the neural networks) with two fixed sonars at full allowed power:
L-L (120 kHz, 20 ms, the longest reach) and H-S (300 kHz, 1 ms).

  python3 host/energy_compare.py
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "MCU ", "SIH"))
import physics_reference as pr                              # noqa: E402
from water_sim import WaterSim                              # noqa: E402

P, C = pr.PROFILES, pr.C
names = [p["name"] for p in P]


def main():
    sim, prev = WaterSim(), {}
    fixed = {"L-L": names.index("L-L"), "H-S": names.index("H-S")}
    energy = {k: [] for k in ["adaptive", *fixed]}
    closes = {k: [] for k in energy}
    picks = {}
    started = False
    while True:
        r = sim.step()
        if started and r["bat"] > 99:                      # recharged: one whole battery cycle done
            break
        started = started or r["bat"] < 99
        env = dict(T=r["T"], S=r["S"], D=r["D"], turb=r["turb"], soc=r["bat"])
        i, ok, ev, c = pr.decide(env, r["R"], prev.get(r["contact"], -1))
        prev[r["contact"]] = i
        energy["adaptive"].append(ev["p_use"] * P[i]["T"])
        closes["adaptive"].append(ok)
        picks[names[i]] = picks.get(names[i], 0) + 1
        eb = pr.energy_budget_j(env["soc"])
        rng = min(max(r["R"], C["MIN_TARGET_RANGE_M"]), C["MAX_TARGET_RANGE_M"])
        for name, k in fixed.items():
            e = pr.evaluate(P[k], env, c, rng, eb)
            energy[name].append(e["p_cap"] * P[k]["T"])     # full allowed power, every ping
            closes[name].append(e["p_req"] <= e["p_cap"])
    n = len(energy["adaptive"])
    print(f"{n} pings ({n / 6 / 60:.1f} minutes at 6 a second), one full battery cycle")
    for k in energy:
        print(f"  {k + ' (fixed, full power)' if k != 'adaptive' else 'adaptive':26s} "
              f"{np.mean(energy[k]) * 1e3:8.2f} mJ per ping, link closes on {100 * np.mean(closes[k]):5.1f} % of pings")
    print(f"  fixed L-L uses {np.mean(energy['L-L']) / np.mean(energy['adaptive']):.0f}x the adaptive energy")
    print("  adaptive choices: " + ", ".join(f"{k} {100 * v / n:.1f} %" for k, v in sorted(picks.items())))


if __name__ == "__main__":
    main()
