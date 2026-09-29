"""Send waveform settings to the board over the USB UART (the PC plays the MCU),
and optionally capture one pulse and check it against the golden model.

  python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --capture     # LFM, 150 -> 500 kHz
  python3 host/wavegen.py geo   --fc 325e3 --bw 350e3 --capture     # geometric, 150 -> 500 kHz
  python3 host/wavegen.py bpsk  --fc 300e3 --capture                # Barker-13 at 300 kHz
  python3 host/wavegen.py tone  --fc 200e3

Every pulse is LEN = 13000 clocks (259 us); sweeps go up, from fc - bw/2 to fc + bw/2;
BPSK always uses Barker-13, 1000 clocks per chip.
Common options: --amp 0..1 (default 1.0), --period clocks between pulse starts,
--port (default /dev/ttyUSB1), --dry-run (print the frame, don't send).
"""
import argparse
import struct
import sys
from model import F_CLK, K, geo_ftw

LEN = 13000
BARKER13 = "+++++--++-+-+"                  # + = normal, - = flipped; chip i = bit i
CODE = sum(1 << i for i, c in enumerate(BARKER13) if c == "-")
CHIP_LEN = LEN // len(BARKER13)
MODE_GEO = 1
STRUCT = "<HIIIHHHIH"                       # len ftw_start ftw_step win_step code chip_len amp period mode
F_MAX = 0.4 * F_CLK                         # keep well under Nyquist (F_CLK / 2)


def settings(args):
    """Real-world units -> the 26-byte settings struct (same fields as top.v)."""
    code, chip_len, step, mode = 0, CHIP_LEN, 0, 0
    f0 = args.fc - args.bw / 2 if args.kind in ("chirp", "geo") else args.fc
    start = round(f0 * K)
    if args.kind == "chirp":
        step = round(args.bw * K / LEN)
    elif args.kind == "geo":
        mode = MODE_GEO
        step = round(((args.fc + args.bw / 2) / f0) ** (1 / (LEN // 32)) * 2**32 - 2**32)   # LEN // 32 updates
    elif args.kind == "bpsk":
        code = CODE
    end = geo_ftw(start, step, LEN)[-1] if mode else start + step * LEN
    if start <= 0 or end / K > F_MAX or not 0 <= step < 2**32:
        sys.exit(f"frequencies must stay between 0 and {F_MAX/1e6:.1f} MHz")
    period = args.period or LEN + 12000
    if period <= LEN:
        sys.exit(f"--period ({period}) must be longer than the pulse ({LEN})")
    amp = max(0, min(4096, round(args.amp * 4096)))
    win_step = round(2**32 / LEN)
    fields = (LEN, start, step, win_step, code, chip_len, amp, period, mode)
    return fields, struct.pack(STRUCT, *fields)


def frame(cmd, payload=b""):
    chk = cmd
    for b in payload:
        chk ^= b
    return bytes([0xA5, cmd]) + payload + bytes([chk])


def describe(fields):
    length, start, step, win_step, code, chip_len, amp, period, mode = fields
    end = geo_ftw(start, step, length)[-1] if mode & MODE_GEO else start + step * length
    kind = ("geometric sweep" if mode & MODE_GEO else "LFM chirp" if step else
            "BPSK" if code else "tone")
    print(f"{kind}: {start/K/1e3:.2f} -> {end/K/1e3:.2f} kHz, pulse {length} clocks "
          f"({length / F_CLK * 1e6:.1f} us), every {period} clocks; amp {amp / 4096:.3f}")
    if code:
        print(f"phase code Barker-13 ({code:#06x}), {chip_len} clocks per chip")
    print(f"ftw_start {start}  ftw_step {step}  win_step {win_step}  mode {mode}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["tone", "chirp", "geo", "bpsk"])
    ap.add_argument("--fc", type=float, required=True, help="centre / carrier frequency, Hz")
    ap.add_argument("--bw", type=float, default=0, help="sweep bandwidth, Hz (chirp, geo)")
    ap.add_argument("--amp", type=float, default=1.0)
    ap.add_argument("--period", type=int, default=0, help="clocks between pulse starts")
    ap.add_argument("--capture", action="store_true", help="capture the next pulse and check it")
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--dry-run", action="store_true", help="print the frames instead of sending")
    ap.add_argument("--frames-out", help="append the frames as hex lines to this file (for top_tb)")
    args = ap.parse_args()
    if args.kind in ("chirp", "geo") and args.bw <= 0:
        sys.exit(f"a {args.kind} sweep needs --bw")

    fields, payload = settings(args)
    describe(fields)
    frames = frame(0x01, payload) + (frame(0x02) if args.capture else b"")

    if args.frames_out:
        with open(args.frames_out, "a") as f:
            f.writelines(f"{b:02x}\n" for b in frames)
    if args.dry_run or args.frames_out:
        print("frames:", frames.hex(" "))
        sys.exit(0)

    import serial
    from capture import read_capture, check
    with serial.Serial(args.port, 115200, timeout=10) as s:
        s.reset_input_buffer()
        s.write(frames)
        print(f"sent to {args.port}")
        if args.capture:
            ok = check(*read_capture(s), "board_capture")
            sys.exit(0 if ok else 1)
