"""Send waveform settings to the board over the USB UART (the PC plays the MCU),
and optionally capture one pulse and check it against the golden model.

  python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --len 16000 --capture
  python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --down
  python3 host/wavegen.py bpsk  --fc 300e3 --chip-len 1000 --code barker13 --capture
  python3 host/wavegen.py tone  --fc 200e3 --len 20000

Common options: --amp 0..1 (default 1.0), --period clocks between pulse starts,
--port (default /dev/ttyUSB1), --dry-run (print the frame, don't send).
"""
import argparse
import struct
import sys
from model import F_CLK, K

CODES = {                                   # + = normal, - = flipped
    "barker2":  "+-",
    "barker3":  "++-",
    "barker4":  "++-+",
    "barker5":  "+++-+",
    "barker7":  "+++--+-",
    "barker11": "+++---+--+-",
    "barker13": "+++++--++-+-+",
}
F_MAX = 0.4 * F_CLK                         # keep well under Nyquist (F_CLK / 2)


def code_bits(name):
    """Code name or a string of +/- -> (bits, number of chips). Chip i = bit i, 1 = flip."""
    chips = CODES.get(name.lower(), name)
    if set(chips) - set("+-") or not 1 <= len(chips) <= 16:
        sys.exit(f"code must be one of {', '.join(CODES)} or up to 16 of + and -")
    return sum(1 << i for i, c in enumerate(chips) if c == "-"), len(chips)


def settings(args):
    """Real-world units -> the 24-byte settings struct (same fields as top.v)."""
    code, chip_len, step = 0, 1000, 0
    if args.kind == "tone":
        length, start = args.len, round(args.fc * K)
    elif args.kind == "chirp":
        length = args.len
        f0 = args.fc + args.bw / 2 if args.down else args.fc - args.bw / 2
        start = round(f0 * K)
        step = round(args.bw * K / length) * (-1 if args.down else 1)
    else:
        code, chips = code_bits(args.code)
        chip_len = args.chip_len
        length = chips * chip_len
        start = round(args.fc * K)
    lo, hi = sorted([start, start + step * length])
    if lo < 0 or hi / K > F_MAX:
        sys.exit(f"frequencies must stay between 0 and {F_MAX/1e6:.1f} MHz")
    if not 1 <= length <= 65535:
        sys.exit(f"pulse length {length} clocks must be 1..65535 (1.3 ms)")
    period = args.period or max(25000, length + 1000)
    if period <= length:
        sys.exit(f"--period ({period}) must be longer than the pulse ({length})")
    amp = max(0, min(4096, round(args.amp * 4096)))
    win_step = round(2**32 / length)
    fields = (length, start % 2**32, step % 2**32, win_step, code, chip_len, amp, period)
    return fields, struct.pack("<HIIIHHHI", *fields)


def frame(cmd, payload=b""):
    chk = cmd
    for b in payload:
        chk ^= b
    return bytes([0xA5, cmd]) + payload + bytes([chk])


def describe(fields):
    length, start, step, win_step, code, chip_len, amp, period = fields
    step_s = step - 2**32 if step >= 2**31 else step
    f0, f1 = start / K, (start + step_s * length) / K
    print(f"pulse {length} clocks ({length / F_CLK * 1e6:.1f} us), every {period} clocks; "
          f"{f0/1e3:.2f} -> {f1/1e3:.2f} kHz; amp {amp / 4096:.3f}")
    if code:
        print(f"phase code {code:#06x}, {chip_len} clocks per chip")
    print(f"ftw_start {start}  ftw_step {step_s}  win_step {win_step}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["tone", "chirp", "bpsk"])
    ap.add_argument("--fc", type=float, required=True, help="centre / carrier frequency, Hz")
    ap.add_argument("--bw", type=float, default=0, help="chirp bandwidth, Hz")
    ap.add_argument("--down", action="store_true", help="chirp from high to low")
    ap.add_argument("--len", type=int, default=16000, help="pulse length, clocks (tone, chirp)")
    ap.add_argument("--code", default="barker13", help="bpsk code name, or a string of + and -")
    ap.add_argument("--chip-len", type=int, default=1000, help="bpsk clocks per chip")
    ap.add_argument("--amp", type=float, default=1.0)
    ap.add_argument("--period", type=int, default=0, help="clocks between pulse starts")
    ap.add_argument("--capture", action="store_true", help="capture the next pulse and check it")
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--dry-run", action="store_true", help="print the frames instead of sending")
    ap.add_argument("--frames-out", help="append the frames as hex lines to this file (for top_tb)")
    args = ap.parse_args()
    if args.kind == "chirp" and args.bw <= 0:
        sys.exit("a chirp needs --bw")

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
