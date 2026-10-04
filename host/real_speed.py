"""Pings at their real speed after the DAC, recorded by the BlackPill's ADC at 2.4 MS/s
(blackpill/src/main.cpp), next to the FPGA's own samples of the same pings.

The ESP32 chooses 6 pings a second as usual. The BlackPill records the filtered DAC output around
each pulse (triggered by FPGA pin 41) while the FPGA streams its capture header for every pulse
(stream mode, as in host/demo.py); each recording is paired with the pulse it holds, the FPGA's
samples for that packet come from the golden model, and both are plotted on the real time axis:
no slow motion and no averaging.

  python3 host/real_speed.py                  # 6 consecutive pings
  python3 host/real_speed.py --pings 1        # the next ping only
  python3 host/real_speed.py --no-show        # only save sim/out/real_speed.png

Wiring: OPA340 output -> BlackPill PA1, FPGA pin 41 -> PB0, grounds joined. Pulses longer than
about 14 ms (the L profiles' 20 ms) are recorded only in part: the BlackPill keeps 16.6 ms.
"""
import argparse
import os
import threading
import time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import sys
import serial
from serial.tools import list_ports

from model import F_CLK
from capture import open_board, read_capture, unpack
from dac_view import fpga_samples, align, VDD
from demo import band, from_header, MOD_OF, MOD_NAME
from water_sim import conditions

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "MCU ", "SIH"))
from physics_reference import PROFILES                     # noqa: E402

STEP = 5                                                   # FPGA samples every 5 clocks (10 MS/s)
WINDOW_S, PRE_S = 0.0145, 0.0002                           # recorded per pulse, and before it


def blackpill_port():
    for p in list_ports.comports():
        if (p.vid, p.pid) == (0x0483, 0x5740):
            return p.device
    raise SystemExit("no BlackPill found (USB 0483:5740); is it plugged in and running blackpill/?")


def read_recordings(bp, count, n, pre):
    """The BlackPill's recordings of the next `count` pulses: [(header dict, counts)]."""
    bp.reset_input_buffer()
    bp.write(f"S {count} {n} {pre}\n".encode())
    out = []
    while True:
        line = bp.readline().decode(errors="replace").strip()
        if not line:
            raise SystemExit("the BlackPill stopped answering")
        if line.startswith("CAPERR"):
            raise SystemExit("BlackPill: " + line)
        if line == "SEQEND":
            return out
        if line.startswith("CAP "):
            h = dict(kv.split("=", 1) for kv in line.split()[1:])
            raw = bp.read(2 * int(h["n"]))
            bp.readline()
            bp.readline()                                  # "\nEND\n"
            out.append((h, np.frombuffer(raw, dtype="<u2").astype(float)))


def fpga_stream(fpga, caps, stop):
    """Collect the FPGA's capture headers (and samples) for every pulse until told to stop."""
    fpga.reset_input_buffer()
    fpga.write(bytes([0xA5, 0x03, 0x03]))                  # stream on
    while not stop.is_set():
        try:
            fields, raw = read_capture(fpga, quiet=True)
        except TimeoutError:
            fpga.write(bytes([0xA5, 0x03, 0x03]))
            continue
        f, samples = unpack(fields, raw)
        if samples is not None:
            caps.append(f)
    fpga.write(bytes([0xA5, 0x04, 0x04]))                  # stream off


def info_of(f):
    return dict(len=f["len"], ftw_start=f["ftw_start"], ftw_step=f["ftw_step"], amp=f["amp"], code=f["code"],
                chip=f["chip_len"], win_step=f["win_step"], mod=str(f["mod"]))


def match(args):
    """Line one recording up with one packet's FPGA samples: (score, t_ms, expect, after, fine), where
    expect is the FPGA's output at the ADC's sample times and fine = (t_ms, volts) every STEP clocks."""
    f, h, counts = args
    rate = float(h["rate"])
    before = fpga_samples(info_of(f), STEP)
    t_pulse = min(f["len"] / F_CLK, (len(counts) - 1) / rate * 0.95)
    volts = counts / 4095 * VDD
    score, r, lag, m = align(volts, before, STEP / F_CLK, rate, t_pulse)
    after = volts[lag:lag + m]
    t = np.arange(m) / r
    expect = np.interp(t, np.arange(len(before)) * STEP / F_CLK, before)
    after = after - (after.mean() - expect.mean())         # the ADC's DC error
    tb = np.arange(len(before)) * STEP / F_CLK
    keep = tb <= t[-1]
    return score, t * 1e3, expect, after, (tb[keep] * 1e3, before[keep])


def pair(recs, caps):
    """Which consecutive run of FPGA pulses the recordings hold: the offset with the best match."""
    k, best = len(recs), None
    with ProcessPoolExecutor() as pool:
        for j in range(0, len(caps) - k + 1):
            if any(caps[j + i]["pulse"] != (caps[j]["pulse"] + i) % 65536 for i in range(k)):
                continue
            res = list(pool.map(match, [(caps[j + i], h, c) for i, (h, c) in enumerate(recs)]))
            total = sum(r[0] for r in res)
            if best is None or total > best[0]:
                best = (total, j, res)
    if best is None:
        raise SystemExit("the FPGA's headers do not cover the recorded pulses")
    return [caps[best[1] + i] for i in range(k)], best[2]


def plot(fs, res, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    n = len(fs)
    c_before, c_after = "#2a78d6", "#eb6834"
    fig = plt.figure(figsize=(max(7, 3 * n), 10.5))
    gs = fig.add_gridspec(2, n, left=0.06, right=0.98, top=0.87, bottom=0.27, wspace=0.25, hspace=0.45,
                          height_ratios=[2, 1])
    dev = max(np.abs(np.r_[e, a] - VDD / 2).max() for _, _, e, a, _ in res)
    axes = []
    for i, (f, (score, t, e, a, (tb, b))) in enumerate(zip(fs, res)):
        ax = fig.add_subplot(gs[0, i], sharey=axes[0] if axes else None)
        axes.append(ax)
        ax.plot(tb, b, lw=0.8, color=c_before)
        ax.plot(t, a, lw=0.6, color=c_after, alpha=0.85)
        prof = PROFILES[f["profile"]]["name"] if f["profile"] < len(PROFILES) else "?"
        ax.set_title(f"{i * f['period'] / F_CLK * 1e3:.0f} ms", fontsize=11, fontweight="bold")
        ax.text(0.03, 0.97, f"{prof} {MOD_NAME[MOD_OF[f['mod']]]} · {f['amp'] / 4096:.2f}", transform=ax.transAxes,
                va="top", fontsize=9)
        ax.set_xlim(0, t[-1])
        ax.set_xlabel("ms")
        if i:
            ax.tick_params(labelleft=False)
        f0 = f["ftw_start"] / 2**32 * F_CLK
        chip_ms = f["chip_len"] / F_CLK * 1e3
        centre = 5 * chip_ms if f["code"] else t[-1] / 2
        half = 4 / f0 * 1e3
        k = (t > centre - half) & (t < centre + half)
        axz = fig.add_subplot(gs[1, i])
        kb = (tb > centre - half) & (tb < centre + half)
        axz.plot(tb[kb] * 1e3, b[kb], lw=1.2, color=c_before)
        axz.plot(t[k] * 1e3, a[k], ".-", lw=0.7, ms=3, color=c_after)
        axz.set_title("close-up", fontsize=9)
        axz.set_xlabel("µs", fontsize=8)
        axz.tick_params(labelsize=7)
        axz.xaxis.set_major_locator(plt.MaxNLocator(4))
    axes[0].set_ylim(VDD / 2 - 1.25 * dev, VDD / 2 + 1.6 * dev)
    axes[0].set_ylabel("volts")
    fig.suptitle("Pings at full speed through the DAC", fontsize=14, fontweight="bold", y=0.985)
    fig.legend([Line2D([], [], color=c_before, lw=3), Line2D([], [], color=c_after, lw=3)],
               ["Blue: FPGA output (before the DAC)", "Orange: measured DAC output, BlackPill ADC at 2.4 MS/s"],
               loc="upper center", bbox_to_anchor=(0.5, 0.95), ncol=2 if n > 2 else 1, fontsize=11, frameon=False)
    return fig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bp", help="the BlackPill's USB port (found by its USB id otherwise)")
    ap.add_argument("--fpga", default="/dev/ttyUSB1", help="the Tang Nano's UART")
    ap.add_argument("--pings", type=int, default=6)
    ap.add_argument("--no-show", action="store_true", help="only save the picture")
    a = ap.parse_args()

    bp = serial.Serial(a.bp or blackpill_port(), 115200, timeout=5)
    bp.write(b"?\n")
    status = bp.readline().decode(errors="replace").strip()
    if not status.startswith("BPADC"):
        raise SystemExit(f"unexpected answer from the BlackPill: {status!r}")
    rate = float(dict(kv.split("=", 1) for kv in status.split()[1:])["rate"])
    n, pre = int(WINDOW_S * rate), int(PRE_S * rate)

    fpga = open_board(a.fpga, timeout=1)
    caps, stop = [], threading.Event()
    th = threading.Thread(target=fpga_stream, args=(fpga, caps, stop), daemon=True)
    th.start()
    time.sleep(0.3)                                        # the FPGA's stream is running first
    recs = read_recordings(bp, a.pings, n, pre)
    time.sleep(0.4)
    stop.set()
    th.join()
    fs, res = pair(recs, caps)

    sim = conditions([f["seq"] for f in fs])
    rows = []
    for i, (f, (score, t, e, a_, _)) in enumerate(zip(fs, res)):
        p = from_header(f, sim[f["seq"]])
        noise = np.sqrt(np.mean(((a_ - a_.mean()) - (e - e.mean())) ** 2)) * 1e3
        print(f"  ping {i + 1}: {p['contact']} at {p['R']} m, {float(p['v']):.2f} m/s -> {p['profile']} "
              f"{MOD_NAME[p['mod']]} {band(f)}, amp {float(p['amp']):.2f}; match {score:.3f}, "
              f"difference {noise:.0f} mV rms")
        rows.append([str(i + 1), p["contact"], p["R"], p["v"], f"{p['profile']} {MOD_NAME[p['mod']]}", band(f),
                     f"{float(p['amp']):.2f}", f"{score:.3f}", f"{noise:.0f}"])
    fig = plot(fs, res, show=not a.no_show)
    axt = fig.add_axes([0.02, 0.01, 0.96, 0.18])
    axt.axis("off")
    tab = axt.table(cellText=rows, colLabels=["ping", "contact", "range m", "speed m/s", "waveform", "frequency",
                                              "amplitude", "match (corr.)", "difference mV rms"],
                    loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(9)
    tab.auto_set_column_width(list(range(9)))
    tab.scale(1, 1.5)
    os.makedirs("sim/out", exist_ok=True)
    fig.savefig("sim/out/real_speed.png", dpi=300)
    fig.savefig("sim/out/real_speed.svg")
    print("plot saved to sim/out/real_speed.png")
    if not a.no_show:
        import signal
        import matplotlib.pyplot as plt
        signal.signal(signal.SIGINT, signal.SIG_DFL)      # Tk would otherwise swallow Ctrl+C
        plt.show()


if __name__ == "__main__":
    main()
