"""Receive captured pulses from the board (or the top_tb simulation) and check them.

Board:       python3 host/capture.py              then press S2 on the board
             python3 host/capture.py --request    ask for one capture over the UART instead
             (reads /dev/ttyUSB1 at 3 Mbaud; --port to change)
Simulation:  python3 host/capture.py --file sim/out/top_uart.txt

Each capture = a 43-byte header (the settings used, the good/bad SPI packet counts, the pulse
number, and the packet's sequence number, profile and flags)
+ N samples. A long pulse is captured with every 2^shift-th sample (shift = top 4 bits of the
mod byte), so the whole pulse fits. The samples are compared with the golden model for those
settings, and plotted to sim/out/.
"""
import argparse
import os
import struct
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import spectrogram
from model import F_CLK, hz, golden, geo_ftw, lfm_ftw

# {shift, mod}, N, len, ftw_start, ftw_step, win_step, code, chip_len, amp, period, spi_ok, spi_bad,
# seq, pulse, profile, flags
HDR = "<BHIIIIHIHIHHHHBB"
HDR_LEN = struct.calcsize(HDR)
KINDS = {0: "tone", 1: "chirp", 2: "geo", 3: "bpsk"}
MOD_LFM, MOD_GEO = 1, 2


def captures_from_file(path):
    data = bytes(int(line, 16) for line in open(path) if line.strip())
    i = 0
    while (i := data.find(b"\xa5\x5a", i)) >= 0:
        fields = struct.unpack(HDR, data[i + 2:i + 2 + HDR_LEN])
        n = fields[1]
        start = i + 2 + HDR_LEN
        yield fields, data[start:start + 2 * n]
        i = start + 2 * n


def open_board(port, baud=3000000, timeout=30):
    """The Tang Nano's UART. Opened and closed once first: the first open after the board was
    plugged in or loaded does not take the 3 Mbaud setting, so nothing gets through."""
    import serial
    serial.Serial(port, baud).close()
    return serial.Serial(port, baud, timeout=timeout)


def read_capture(s, quiet=False):
    """Wait for one capture on an open serial port; returns (header fields, sample bytes)."""
    while True:
        b = s.read(1)
        if not b:
            raise TimeoutError("no capture arrived")
        if b != b"\xa5" or s.read(1) != b"\x5a":
            continue
        fields = struct.unpack(HDR, s.read(HDR_LEN))
        n = fields[1]
        if not quiet:
            print(f"receiving {KINDS[fields[0] & 3]} capture: {n} samples...")
        return fields, s.read(2 * n)


def captures_from_port(port, baud, request):
    with open_board(port, baud) as s:
        s.reset_input_buffer()
        if request:
            s.write(bytes([0xA5, 0x02, 0x02]))
            yield read_capture(s)
            return
        print(f"listening on {port} at {baud} baud -- press S2 on the board (Ctrl+C to stop)")
        while True:
            try:
                yield read_capture(s)
            except TimeoutError:
                continue


def unpack(fields, raw):
    """(settings dict, samples) from a capture, or None if the sample count is wrong."""
    keys = "mod n len ftw_start ftw_step win_step code chip_len amp period spi_ok spi_bad seq pulse profile flags".split()
    f = dict(zip(keys, fields))
    f["dec"] = 1 << (f["mod"] >> 4)
    f["mod"] &= 3
    if len(raw) != 2 * f["n"]:
        return f, None
    return f, np.array(struct.unpack(f"<{f['n']}h", raw))


def golden_capture(f):
    """The samples the capture should hold: the golden model, every dec-th tick, with a margin
    of 8 captured samples on each side for the alignment search."""
    dec, n = f["dec"], f["n"]
    inputs = [(int(k % f["period"] == 0), f["ftw_start"], f["ftw_step"], f["amp"], f["code"], f["chip_len"])
              for k in range((n + 8) * dec)]    # a start every `period` clocks
    gold, _ = golden(inputs, f["len"], f["win_step"], geo=(f["mod"] == MOD_GEO))
    return gold


def align(got, gold, dec):
    """Line the capture up with the golden model: captured sample i is golden tick i x dec - lag,
    for a lag of a few clocks (the capture starts on the pulse's first clock, give or take the
    pipeline). Returns (lag, captured, expected) over the samples that overlap."""
    n, best = len(got), None
    for lag in range(-4, 5):
        i0 = max(0, -(-lag // dec))                 # first i with i x dec - lag >= 0
        idx = np.arange(i0, n) * dec - lag
        idx = idx[idx < len(gold)]
        a, b = got[i0:i0 + len(idx)], gold[idx]
        m = np.sum(a == b)
        if best is None or m > best[0]:
            best = (m, lag, a, b)
    return best[1:]


def check(fields, raw, tag):
    f, got = unpack(fields, raw)
    mod, n, dec, length = f["mod"], f["n"], f["dec"], f["len"]
    ftw_start, ftw_step, code, amp, period = f["ftw_start"], f["ftw_step"], f["code"], f["amp"], f["period"]
    spi_ok, spi_bad = f["spi_ok"], f["spi_bad"]
    if got is None:
        print(f"FAIL: expected {2 * n} sample bytes, got {len(raw)}")
        return False
    best, a, b = align(got, golden_capture(f), dec)
    bad = np.nonzero(a != b)[0]

    name = KINDS[mod]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fs = F_CLK / dec
    t_us = np.arange(n) / fs * 1e6
    ax1.plot(t_us, got, lw=0.5, label="captured")
    ax1.set(title=f"{tag}: {name}, {n} samples" + (f", every {dec}th clock" if dec > 1 else ""),
            ylabel="sample value")
    seg = min(1024, n // 4)
    padded = np.concatenate([np.zeros(seg // 2), got, np.zeros(seg // 2)]).astype(float)
    fr, t, S = spectrogram(padded, fs=fs, window="hann",
                           nperseg=seg, noverlap=seg - seg // 16, nfft=8 * seg)
    ax2.pcolormesh((t - seg / 2 / fs) * 1e6, fr / 1e3, 10 * np.log10(S / S.max() + 1e-12),
                   shading="auto", vmin=-30, vmax=0, cmap="magma")
    shown = min(n * dec, length)
    track = (geo_ftw(ftw_start, ftw_step, shown) if mod == MOD_GEO else
             lfm_ftw(ftw_start, ftw_step, shown) if mod == MOD_LFM else None)
    if track is not None:
        ax2.plot((np.arange(shown) + best) / F_CLK * 1e6, hz(track.astype(float)) / 1e3, "c--", lw=1,
                 label="expected " + ("(geometric)" if mod == MOD_GEO else "(linear)"))
        ax2.legend(loc="upper left")
    ax2.set(title="Spectrogram", xlabel="time (µs)", ylabel="frequency (kHz)", ylim=(0, min(800, fs / 2e3)),
            xlim=(0, t_us[-1]))
    fig.tight_layout()
    os.makedirs("sim/out", exist_ok=True)
    out = f"sim/out/{tag}_{name}.png"
    fig.savefig(out, dpi=110)

    full = (geo_ftw(ftw_start, ftw_step, length) if mod == MOD_GEO else
            lfm_ftw(ftw_start, ftw_step, length) if mod == MOD_LFM else [ftw_start])
    f0, f1 = hz(ftw_start), hz(float(full[-1]))
    shown_note = "" if length <= n * dec else f" (the capture shows the first {n * dec / length:.0%})"
    print(f"{name}: {f0/1e3:.1f} -> {f1/1e3:.1f} kHz, len {length} ({length / F_CLK * 1e3:.2f} ms)"
          f"{shown_note}, code {code:#06x}, amp {amp}, period {period / F_CLK * 1e3:.1f} ms; "
          f"SPI packets good {spi_ok} bad {spi_bad}; aligned at offset {best}; plot {out}")
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
    ap.add_argument("--baud", type=int, default=3000000)
    ap.add_argument("--request", action="store_true", help="ask the board for one capture (no S2 press)")
    args = ap.parse_args()

    source = (captures_from_file(args.file) if args.file else
              captures_from_port(args.port, args.baud, args.request))
    tag = "sim_capture" if args.file else "board_capture"
    results = [check(fields, raw, tag) for fields, raw in source]
    if args.file or args.request:
        print(f"\n{'PASS' if results and all(results) else 'FAIL'}: {len(results)} capture(s) checked")
        sys.exit(0 if results and all(results) else 1)
