"""One second of the whole system: 6 pings, each chosen by the ESP32 for the water conditions of
that moment, as the FPGA actually output them, on one graph.

The ESP32 simulates changing water conditions (MCU /SIH/water_sim.h) and decides a waveform for
every ping, 6 a second, printing a PING line for each. The FPGA captures every pulse it plays
(stream mode) and sends it over its USB UART; each capture carries the ping number and the
packet's sequence number, profile and flags. This program collects 6 consecutive pings, checks
every one sample for sample against the golden model, and plots them at their real times over one
second, with the baseline (no output) in between.

The water conditions come from the ESP32's PING lines when its USB is connected; otherwise they
are worked out from the packet's sequence number by host/water_sim.py (the same simulation,
counted from the ESP32's power-on, in automatic mode throughout).

  python3 host/demo.py                  # both boards connected
  python3 host/demo.py --no-show        # only save sim/out/demo.png

Short pulses (1 ms) are thin lines at this time scale; their height is the amplitude.
"""
import argparse
import os
import sys
import threading
import time
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import serial

from model import F_CLK, hz, geo_ftw, lfm_ftw
from capture import open_board, read_capture, unpack, golden_capture, align
from water_sim import conditions

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "MCU ", "SIH"))
from physics_reference import PROFILES                     # noqa: E402  (the profile names)

PINGS = 6
MOD_NAME = {"cw": "tone", "lfm": "LFM", "geo": "geometric", "bpsk": "BPSK"}
MOD_OF = ["cw", "lfm", "geo", "bpsk"]                        # the capture header's modulation field
FLAG_LINK_OK, FLAG_ML_USED, FLAG_ML_AMP_RAISED, FLAG_ML_MOD_OVERRIDDEN = 1, 1 << 3, 1 << 6, 1 << 7


def esp_reader(esp, pings, stop):
    """Collect the ESP32's PING lines: seq -> {key: value}."""
    buf = ""
    while not stop.is_set():
        buf += esp.read(4096).decode(errors="replace")
        *lines, buf = buf.split("\n")
        for line in lines:
            if line.startswith("PING "):
                d = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
                if "seq" in d:
                    pings[int(d["seq"])] = d


def collect(fpga, timeout=10.0):
    """Stream captures until PINGS of them come from consecutive pulses."""
    got = []
    fpga.reset_input_buffer()
    fpga.write(bytes([0xA5, 0x03, 0x03]))                  # stream on
    deadline = time.time() + timeout
    try:
        while time.time() < deadline:
            try:
                fields, raw = read_capture(fpga, quiet=True)
            except TimeoutError:                           # nothing came back: ask again
                fpga.write(bytes([0xA5, 0x03, 0x03]))
                continue
            f, samples = unpack(fields, raw)
            if samples is None:
                continue
            if got and f["pulse"] != (got[-1][0]["pulse"] + 1) % 65536:
                got = []                                    # a pulse was missed: start again
            got.append((f, samples))
            if len(got) == PINGS:
                return got
    finally:
        fpga.write(bytes([0xA5, 0x04, 0x04]))              # stream off
    raise SystemExit(f"no {PINGS} consecutive pings within {timeout:.0f} s "
                     f"(got {len(got)}); is the ESP32 sending and the UART at 3 Mbaud?")


def verify(f, samples):
    _, a, b = align(samples, golden_capture(f), f["dec"])
    return bool(np.all(a == b) and np.any(a != 0))


def band(f):
    """What the FPGA played, from the capture header: the frequency or the sweep, in kHz."""
    mod = MOD_OF[f["mod"]]
    if mod == "lfm":
        f1 = hz(float(lfm_ftw(f["ftw_start"], f["ftw_step"], f["len"])[-1]))
    elif mod == "geo":
        f1 = hz(float(geo_ftw(f["ftw_start"], f["ftw_step"], f["len"])[-1]))
    else:
        return f"{hz(f['ftw_start']) / 1e3:.0f} kHz"
    return f"{hz(f['ftw_start']) / 1e3:.0f}-{f1 / 1e3:.0f} kHz"


def from_header(f, sim):
    """A ping's details without the ESP32's log: the conditions from the simulation twin, the
    choice and the flags from the capture header."""
    return dict(contact=sim["contact"], R=f"{sim['R']:.0f}", v=f"{sim['v']:.2f}", T=f"{sim['T']:.2f}",
                S=f"{sim['S']:.2f}", D=f"{sim['D']:.1f}", turb=f"{sim['turb']:.1f}", bat=f"{sim['bat']:.1f}",
                profile=PROFILES[f["profile"]]["name"] if f["profile"] < len(PROFILES) else "?",
                mod=MOD_OF[f["mod"]], amp=f"{f['amp'] / 4096:.3f}", flags=f"0x{f['flags']:02X}")


def label(i, p):
    return f"{i}: {p['contact']} {p['R']} m\n{p['profile']} {MOD_NAME.get(p['mod'], p['mod'])}"


def nn_text(p):
    """What the network picked and what the physics did with it (the picks themselves only with the log)."""
    flags = int(p["flags"], 16)
    prof = (p.get("ml_profile", "") + " ").lstrip() + ("kept" if flags & FLAG_ML_USED else "replaced")
    mod = (p.get("ml_mod", "") + " ").lstrip() + ("replaced" if flags & FLAG_ML_MOD_OVERRIDDEN else "kept")
    amp = (f"{float(p['ml_amp']):.2f} " if "ml_amp" in p else "") + ("raised" if flags & FLAG_ML_AMP_RAISED else "kept")
    return f"profile {prof}, mod {mod}, amp {amp}"


def plot(caps, pings, oks, path, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    period = caps[0][0]["period"] / F_CLK
    first = caps[0][0]["pulse"]
    fig = plt.figure(figsize=(18, 9.5))
    ax = fig.add_axes([0.05, 0.40, 0.93, 0.50])
    ax.axhline(0, color="grey", lw=0.8, zorder=0)             # the baseline: no output between pulses
    colours = plt.cm.tab10.colors
    rows = []
    for i, ((f, s), ok) in enumerate(zip(caps, oks)):
        t0 = ((f["pulse"] - first) % 65536) * period
        t = t0 + np.arange(len(s)) * f["dec"] / F_CLK
        y = s / 2048.0
        c = colours[i % 10]
        ax.plot(t, y, lw=0.6, color=c)
        p = pings[f["seq"]]
        ax.annotate(label(i + 1, p), (t0 + f["len"] / F_CLK / 2, np.abs(y).max()), xytext=(0, 8),
                    textcoords="offset points", ha="center", va="bottom", fontsize=8.5, color=c, fontweight="bold")
        rows.append([f"{i + 1}", f"{t0 * 1e3:.0f}", p["contact"], p["R"], f"{float(p['v']):.2f}", p["T"], p["S"],
                     p["D"], p["turb"], p["bat"], f"{p['profile']} {MOD_NAME.get(p['mod'], p['mod'])}", band(f),
                     f"{f['len'] / F_CLK * 1e3:.0f}", f"{float(p['amp']):.3f}", nn_text(p),
                     "yes" if int(p["flags"], 16) & FLAG_LINK_OK else "NO", "bit-exact" if ok else "MISMATCH"])
    peak = max(np.abs(s).max() for _, s in caps) / 2048.0
    ax.set_xlim(-0.03, 1.0)
    ax.set_ylim(-1.15 * peak, 1.6 * peak)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("FPGA output (fraction of full scale)")
    ax.set_title(f"{PINGS} pings in one second, as output by the FPGA", fontsize=11)

    cols = ["ping", "t (ms)", "contact", "range m", "speed m/s", "temp C", "salinity", "depth m", "turb NTU",
            "battery %", "waveform", "frequency", "ms", "amplitude", "neural network pick, after the physics check",
            "link", "FPGA output"]
    axt = fig.add_axes([0.01, 0.02, 0.98, 0.28])
    axt.axis("off")
    tab = axt.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(8)
    tab.auto_set_column_width(list(range(len(cols))))
    tab.scale(1, 1.6)
    for (r, col), cell in tab.get_celld().items():
        if r > 0:
            cell.set_text_props(color=colours[(r - 1) % 10] if col == 0 else
                                ("green" if rows[r - 1][-1] == "bit-exact" else "red") if col == len(cols) - 1
                                else "black")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp", default="/dev/ttyACM0", help="the ESP32's USB port")
    ap.add_argument("--fpga", default="/dev/ttyUSB1", help="the Tang Nano's UART")
    ap.add_argument("--no-show", action="store_true", help="only save the picture")
    a = ap.parse_args()

    pings, stop, reader = {}, threading.Event(), None
    try:
        esp = serial.Serial()
        esp.port, esp.baudrate, esp.timeout = a.esp, 115200, 0.1
        esp.dtr = esp.rts = False                          # toggling these resets the ESP32-S3
        esp.open()
        esp.write(b"A\n")                                  # automatic: simulated water conditions
        reader = threading.Thread(target=esp_reader, args=(esp, pings, stop), daemon=True)
        reader.start()
        time.sleep(0.5)
    except serial.SerialException:
        pass                                               # conditions from the packet numbers instead

    fpga = open_board(a.fpga, timeout=1)
    caps = collect(fpga)
    if reader:
        time.sleep(0.5)                                    # the last PING lines
        stop.set()
        reader.join()
    missing = [f["seq"] for f, _ in caps if f["seq"] not in pings]
    if missing:
        sim = conditions(missing)
        for f, _ in caps:
            if f["seq"] in missing:
                pings[f["seq"]] = from_header(f, sim[f["seq"]])

    with ProcessPoolExecutor() as pool:
        oks = list(pool.map(verify, [f for f, _ in caps], [s for _, s in caps]))
    period = caps[0][0]["period"] / F_CLK
    for i, ((f, s), ok) in enumerate(zip(caps, oks)):
        p = pings[f["seq"]]
        what = (f"contact {p['contact']} at {p['R']} m, {float(p['v']):.2f} m/s -> {p['profile']} "
                f"{MOD_NAME.get(p['mod'], p['mod'])} {band(f)}, amp {float(p['amp']):.3f}")
        print(f"  ping {i + 1} at {i * period * 1e3:4.0f} ms, packet {f['seq']}: {what}: "
              + ("bit-exact" if ok else "MISMATCH"))
    plot(caps, pings, oks, "sim/out/demo.png", show=not a.no_show)
    print("plot saved to sim/out/demo.png")
    sys.exit(0 if all(oks) else 1)


if __name__ == "__main__":
    main()
