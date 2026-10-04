"""One second of pings as they come out of the DAC: the ESP32 takes its next 6 decisions (one
second of the simulated water conditions), plays each to the FPGA in slow motion and records the
sigma-delta DAC's filtered output on GPIO 1 several times (serial command S). Each ping's
recordings are lined up with the FPGA's own samples and averaged, and the six are drawn on one
graph whose time axis skips the baseline between pings.

  python3 host/dac_sequence.py                 # 6 pings, 4 recordings each (~2 minutes)
  python3 host/dac_sequence.py --reps 1        # faster, noisier
  python3 host/dac_sequence.py --no-show       # only save sim/out/dac_sequence.png
  python3 host/dac_sequence.py --replot        # redraw the last recording (sim/out/dac_sequence.pkl)

Wiring as for host/dac_view.py: pin 40 -> 1 kOhm + 100 pF -> 10 kOhm + 10 pF -> ESP32 GPIO 1.
"""
import argparse
import os
import pickle
import sys
import time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import serial

from model import F_CLK, hz, lfm_ftw, geo_ftw
from dac_view import fpga_samples, align, VDD

MOD_NAME = {"cw": "tone", "lfm": "LFM", "geo": "geometric", "bpsk": "BPSK"}
FLAG_LINK_OK, FLAG_ML_USED, FLAG_ML_AMP_RAISED, FLAG_ML_MOD_OVERRIDDEN = 1, 1 << 3, 1 << 6, 1 << 7
PING_PERIOD_MS = 1000 / 6


def record(port, count, reps):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = s.rts = False                                  # toggling these resets the ESP32-S3
    s.open()
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write(f"S {count} {reps}\n".encode())
    text, shown, deadline = "", -1, time.time() + 60 + 20 * count * reps
    while time.time() < deadline and "SEQEND" not in text:
        chunk = s.read(65536).decode(errors="replace")
        text += chunk
        if "CAPERR" in text:
            raise SystemExit("ESP32: " + text[text.find("CAPERR"):].splitlines()[0])
        done = text.count("\nEND")
        if done != shown:
            print(f"  {done} of {count * reps} recordings received", flush=True)
            shown = done
    s.close()
    if "SEQEND" not in text:
        raise SystemExit("the ESP32 did not finish the sequence (is it running the S command build?)")
    pings, cond, info, data = {}, None, None, []
    for line in text.splitlines():
        if line.startswith("COND "):
            cond = dict(kv.split("=", 1) for kv in line.split()[1:])
        elif line.startswith("CAP "):
            info, data = dict(kv.split("=", 1) for kv in line.split()[1:]), []
        elif line.startswith("D ") and info is not None:
            data.append(line[2:].strip())
        elif line.startswith("END") and info is not None and cond is not None:
            hexes = "".join(data)
            mv = np.array([int(hexes[i:i + 3], 16) for i in range(0, len(hexes), 3)], dtype=float)
            p = pings.setdefault(int(cond["idx"]), dict(cond=cond, info=info, recs=[]))
            p["recs"].append((float(info["meas"]), mv))
            info = None
    return [pings[k] for k in sorted(pings)]


def golden_for(info):
    rate = float(info["meas"])
    step = max(1, int(F_CLK / (4 * rate)))
    return step, fpga_samples(info, step)


def band(info):
    """The real pulse's frequency or sweep, from the slow-motion packet (x N)."""
    n, start, step, length = int(info["slow"]), int(info["ftw_start"]), int(info["ftw_step"]) % 2**32, int(info["len"])
    f0 = hz(start) * n / 1e3
    if info["mod"] == "1":
        return f"{f0:.0f}-{hz(float(lfm_ftw(start, step, length)[-1])) * n / 1e3:.0f} kHz"
    if info["mod"] == "2":
        return f"{f0:.0f}-{hz(float(geo_ftw(start, step, length)[-1])) * n / 1e3:.0f} kHz"
    return f"{f0:.0f} kHz"


def average(ping, step, before):
    """Line every recording up with the FPGA's samples and average them on one time grid."""
    info = ping["info"]
    t_pulse = int(info["len"]) / F_CLK
    grid_rate = float(info["meas"])
    m = int(t_pulse * grid_rate)
    tg = np.arange(m) / grid_rate                          # slow-motion seconds
    expect = np.interp(tg, np.arange(len(before)) * step / F_CLK, before)
    acc = []
    for rate, mv in ping["recs"]:
        score, r, lag, mm = align(mv / 1000, before, step / F_CLK, rate, t_pulse)
        seg = mv[lag:lag + mm] / 1000
        y = np.interp(tg, np.arange(mm) / r, seg)
        acc.append(y - (y.mean() - expect.mean()))         # the ADC's DC offset removed
    after = np.mean(acc, axis=0)
    a, e = after - after.mean(), expect - expect.mean()
    corr = float(np.dot(a, e) / (np.linalg.norm(a) * np.linalg.norm(e) + 1e-12))
    t_ms = tg / int(info["slow"]) * 1e3                    # real-pulse time
    return t_ms, expect, after, corr


def plot(pings, results, reps, path, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(pings)
    c_before, c_after = "#2a78d6", "#eb6834"
    fig = plt.figure(figsize=(18, 11))
    gs = fig.add_gridspec(2, n, left=0.05, right=0.99, top=0.9, bottom=0.33, wspace=0.22, hspace=0.45,
                          height_ratios=[2.1, 1])
    dev = max(np.abs(np.concatenate([r[1], r[2]]) - VDD / 2).max() for r in results)
    axes = []
    for i, (p, (t_ms, expect, after, corr)) in enumerate(zip(pings, results)):
        ax = fig.add_subplot(gs[0, i], sharey=axes[0] if axes else None)
        axes.append(ax)
        ax.plot(t_ms, expect, lw=0.8, color=c_before, label="before the DAC: FPGA samples")
        ax.plot(t_ms, after, lw=0.6, color=c_after, alpha=0.85, label="after the DAC: measured")
        c = p["cond"]
        ax.set_title(f"ping {i + 1} · {i * PING_PERIOD_MS:.0f} ms", fontsize=11, fontweight="bold")
        ax.text(0.03, 0.97, f"{c['contact']}: {c['R']} m, {float(c['v']):.1f} m/s\n"
                            f"{c['profile']} {MOD_NAME.get(c['mod'], c['mod'])}, amp {float(c['amp']):.2f}",
                transform=ax.transAxes, va="top", fontsize=8.5)
        ax.set_xlim(0, t_ms[-1])
        ax.set_xlabel("ms")
        if i:
            ax.tick_params(labelleft=False)
        # close-up underneath: a few carrier cycles mid-pulse (BPSK: around the first phase flip)
        info = p["info"]
        f0 = hz(int(info["ftw_start"])) * int(info["slow"])
        chip_ms = int(info["chip"]) / F_CLK / int(info["slow"]) * 1e3
        centre = 5 * chip_ms if int(info["code"]) else t_ms[-1] / 2
        half = 4 / f0 * 1e3
        k = (t_ms > centre - half) & (t_ms < centre + half)
        axz = fig.add_subplot(gs[1, i])
        axz.plot(t_ms[k] * 1e3, expect[k], lw=1.2, color=c_before)
        axz.plot(t_ms[k] * 1e3, after[k], ".-", lw=0.7, ms=2.5, color=c_after)
        axz.set_title("close-up" + (" at a phase flip" if int(info["code"]) else ""), fontsize=9)
        axz.set_xlabel("µs", fontsize=8)
        axz.tick_params(labelsize=7)
        axz.xaxis.set_major_locator(plt.MaxNLocator(4))
        if i < n - 1:                                      # the skipped baseline between pings
            for x in (1.0, 1.08):
                ax.plot([x - 0.015, x + 0.015], [-0.03, 0.03], transform=ax.transAxes, color="#52514e",
                        lw=1.2, clip_on=False)
    axes[0].set_ylim(VDD / 2 - 1.25 * dev, VDD / 2 + 1.6 * dev)
    axes[0].set_ylabel("volts")
    axes[0].legend(loc="lower left", fontsize=8)
    fig.suptitle(f"One second of pings through the 1-bit DAC + RC filter (measured, {reps}× averaged, recorded in "
                 f"slow motion)   // = ~166 ms of baseline skipped", fontsize=12)

    cols = ["ping", "t (ms)", "contact", "range m", "speed m/s", "temp C", "depth m", "turb NTU", "battery %",
            "waveform", "frequency", "length", "amplitude", "network", "match (corr.)"]
    rows = []
    for i, (p, r) in enumerate(zip(pings, results)):
        c, info = p["cond"], p["info"]
        flags = int(c["flags"], 16)
        net = ("profile kept" if flags & FLAG_ML_USED else "profile replaced") + \
              (", mod replaced" if flags & FLAG_ML_MOD_OVERRIDDEN else ", mod kept")
        rows.append([str(i + 1), f"{i * PING_PERIOD_MS:.0f}", c["contact"], c["R"], c["v"], c["T"], c["D"],
                     c["turb"], c["bat"], f"{c['profile']} {MOD_NAME.get(c['mod'], c['mod'])}", band(info),
                     f"{int(info['len']) / F_CLK / int(info['slow']) * 1e3:.0f} ms", c["amp"], net, f"{r[3]:.3f}"])
    axt = fig.add_axes([0.01, 0.01, 0.98, 0.24])
    axt.axis("off")
    tab = axt.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(8.5)
    tab.auto_set_column_width(list(range(len(cols))))
    tab.scale(1, 1.6)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    if show:
        import signal
        signal.signal(signal.SIGINT, signal.SIG_DFL)      # Tk would otherwise swallow Ctrl+C
        print("close the plot window, press q in it, or Ctrl+C here to exit")
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp", default="/dev/ttyACM0", help="the ESP32's USB port")
    ap.add_argument("--pings", type=int, default=6)
    ap.add_argument("--reps", type=int, default=4, help="recordings per ping, averaged")
    ap.add_argument("--no-show", action="store_true", help="only save the picture")
    ap.add_argument("--replot", action="store_true", help="redraw the last recording without recording again")
    a = ap.parse_args()
    saved = "sim/out/dac_sequence.pkl"

    if a.replot:
        with open(saved, "rb") as f:
            pings, results, reps = pickle.load(f)
        plot(pings, results, reps, "sim/out/dac_sequence.png", show=not a.no_show)
        print("plot saved to sim/out/dac_sequence.png")
        return
    print(f"recording {a.pings} pings x {a.reps} in slow motion on the ESP32...", flush=True)
    pings = record(a.esp, a.pings, a.reps)
    print("computing the FPGA's samples and lining the recordings up...", flush=True)
    with ProcessPoolExecutor() as pool:
        golds = list(pool.map(golden_for, [p["info"] for p in pings]))
    results = [average(p, step, before) for p, (step, before) in zip(pings, golds)]
    os.makedirs("sim/out", exist_ok=True)
    with open(saved, "wb") as f:
        pickle.dump((pings, results, a.reps), f)
    for i, (p, r) in enumerate(zip(pings, results)):
        c = p["cond"]
        print(f"  ping {i + 1}: {c['contact']} at {c['R']} m, {float(c['v']):.2f} m/s -> {c['profile']} "
              f"{MOD_NAME.get(c['mod'], c['mod'])}, amp {float(c['amp']):.2f}; match {r[3]:.3f}")
    plot(pings, results, a.reps, "sim/out/dac_sequence.png", show=not a.no_show)
    print("plot saved to sim/out/dac_sequence.png")


if __name__ == "__main__":
    main()
