"""Receive captured pulses from the board (or the top_tb simulation) and check them.

Board:       python3 host/capture.py            then press S2 on the board
             (reads /dev/ttyUSB1 at 115200 baud; --port to change)
Simulation:  python3 host/capture.py --file sim/out/top_uart.txt

Each capture = a 29-byte header (the settings used) + N samples. The samples are
compared with the golden model for those settings, and plotted to sim/out/.
"""
import argparse
import struct
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import spectrogram
from model import F_CLK, hz, golden, geo_ftw

HDR = "<BHHIIIHHHI"           # mode, N, len, ftw_start, ftw_step, win_step, code, chip_len, amp, period
HDR_LEN = struct.calcsize(HDR)
MODE_BPSK, MODE_GEO = 1, 2                  # header byte 0


def kind(mode, ftw_step):
    return "bpsk" if mode & MODE_BPSK else "geo" if mode & MODE_GEO else "chirp" if ftw_step else "tone"


def captures_from_file(path):
    data = bytes(int(line, 16) for line in open(path) if line.strip())
    i = 0
    while (i := data.find(b"\xa5\x5a", i)) >= 0:
        fields = struct.unpack(HDR, data[i + 2:i + 2 + HDR_LEN])
        n = fields[1]
        start = i + 2 + HDR_LEN
        yield fields, data[start:start + 2 * n]
        i = start + 2 * n


def read_capture(s):
    """Wait for one capture on an open serial port; returns (header fields, sample bytes)."""
    while True:
        b = s.read(1)
        if not b:
            raise TimeoutError("no capture arrived")
        if b != b"\xa5" or s.read(1) != b"\x5a":
            continue
        fields = struct.unpack(HDR, s.read(HDR_LEN))
        n = fields[1]
        print(f"receiving {kind(fields[0], fields[4])} capture: {n} samples...")
        return fields, s.read(2 * n)


def captures_from_port(port, baud):
    import serial
    with serial.Serial(port, baud, timeout=30) as s:
        s.reset_input_buffer()
        print(f"listening on {port} at {baud} baud -- press S2 on the board (Ctrl+C to stop)")
        while True:
            try:
                yield read_capture(s)
            except TimeoutError:
                continue


def check(fields, raw, tag):
    mode, n, length, ftw_start, ftw_step, win_step, code, chip_len, amp, period = fields
    if len(raw) != 2 * n:
        print(f"FAIL: expected {2 * n} sample bytes, got {len(raw)}")
        return False
    got = np.array(struct.unpack(f"<{n}h", raw))
    inputs = [(int(k % period == 0), ftw_start, ftw_step, amp, code, chip_len)
              for k in range(n + 8)]            # a start every `period` clocks
    gold, _ = golden(inputs, length, win_step, geo=bool(mode & MODE_GEO))

    # The capture starts on the same clock as the pulse; find how the two line up.
    best = max(range(-4, 5), key=lambda d: np.sum(got[max(d, 0):n + min(d, 0)] ==
                                                    gold[max(-d, 0):n - max(d, 0)]))
    a, b = got[max(best, 0):n + min(best, 0)], gold[max(-best, 0):n - max(best, 0)]
    bad = np.nonzero(a != b)[0]

    name = kind(mode, ftw_step)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    t_us = np.arange(n) / F_CLK * 1e6
    ax1.plot(t_us, got, lw=0.5, label="captured")
    ax1.set(title=f"{tag}: {name}, {n} samples", ylabel="sample value")
    seg = min(1024, n // 4)
    padded = np.concatenate([np.zeros(seg // 2), got, np.zeros(seg // 2)]).astype(float)
    f, t, S = spectrogram(padded, fs=F_CLK, window="hann",
                          nperseg=seg, noverlap=seg - seg // 16, nfft=8 * seg)
    ax2.pcolormesh((t - seg / 2 / F_CLK) * 1e6, f / 1e3, 10 * np.log10(S / S.max() + 1e-12),
                   shading="auto", vmin=-30, vmax=0, cmap="magma")
    if ftw_step and not mode & MODE_BPSK:
        k = np.arange(min(n, length))
        track = geo_ftw(ftw_start, ftw_step, len(k)) if mode & MODE_GEO else ftw_start + ftw_step * k
        ax2.plot((k + best) / F_CLK * 1e6, hz(track) / 1e3, "c--", lw=1,
                 label="expected " + ("(geometric)" if mode & MODE_GEO else "(linear)"))
        ax2.legend(loc="upper left")
    ax2.set(title="Spectrogram", xlabel="time (µs)", ylabel="frequency (kHz)", ylim=(0, 800),
            xlim=(0, t_us[-1]))
    fig.tight_layout()
    out = f"sim/out/{tag}_{name}.png"
    fig.savefig(out, dpi=110)

    f0 = hz(ftw_start)
    f1 = hz(geo_ftw(ftw_start, ftw_step, length)[-1] if mode & MODE_GEO
            else (ftw_start + ftw_step * length) % 2**32)
    if f1 > F_CLK / 2:
        f1 -= F_CLK
    print(f"{name}: {f0/1e3:.1f} -> {f1/1e3:.1f} kHz, len {length}, code {code:#06x}, "
          f"amp {amp}; aligned at offset {best}; plot {out}")
    if len(bad) == 0 and np.any(a != 0):
        print(f"PASS: all {len(a)} samples match the golden model")
        return True
    for i in bad[:10]:
        print(f"  sample {i}: got {a[i]}, expected {b[i]}")
    print(f"FAIL: {len(bad)} of {len(a)} samples differ")
    return False


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", help="hex dump from sim/top_tb.v instead of the board")
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    source = captures_from_file(args.file) if args.file else captures_from_port(args.port, args.baud)
    tag = "sim_capture" if args.file else "board_capture"
    results = [check(fields, raw, tag) for fields, raw in source]
    if args.file:
        print(f"\n{'PASS' if results and all(results) else 'FAIL'}: {len(results)} capture(s) checked")
        sys.exit(0 if results and all(results) else 1)
