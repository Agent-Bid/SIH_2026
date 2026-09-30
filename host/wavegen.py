"""Send waveform settings to the board over the USB UART (the PC plays the ESP32), and
optionally capture one pulse and check it against the golden model. The settings go as a
real ESP32 packet (host/mcu_packet.py), inside the UART frame A5 01 <57 bytes> <xor>.

  python3 host/wavegen.py chirp --fc 325e3 --bw 350e3 --capture     # LFM, 150 -> 500 kHz
  python3 host/wavegen.py geo   --fc 325e3 --bw 350e3 --capture     # geometric, 150 -> 500 kHz
  python3 host/wavegen.py bpsk  --fc 300e3 --capture                # Barker-13 at 300 kHz
  python3 host/wavegen.py tone  --fc 200e3

Every pulse is LEN = 13000 clocks (259 us); sweeps go up, from fc - bw/2 to fc + bw/2;
BPSK always uses Barker-13, 1000 clocks per chip.
Common options: --amp 0..1 (default 1.0), --period clocks between pulse starts,
--port (default /dev/ttyUSB1), --dry-run (print the frame, don't send).
If the ESP32 is connected it sends its own packet 10 times a second, which replaces these
settings within 100 ms: to capture what the ESP32 asked for, use  host/capture.py --request.
"""
import argparse
import sys
import mcu_packet as mp
from model import geo_ftw, lfm_ftw

LEN = 13000
KINDS = {"tone": mp.CW, "chirp": mp.LFM, "geo": mp.GEO, "bpsk": mp.BPSK}
F_MAX = 0.4 * mp.F_CLK                      # keep well under Nyquist (F_CLK / 2)


def frame(cmd, payload=b""):
    chk = cmd
    for b in payload:
        chk ^= b
    return bytes([0xA5, cmd]) + payload + bytes([chk])


def describe(p):
    mod, length, start, step = p["modulation"], p["len_clk"], p["ftw_start"], p["ftw_step"]
    end = (geo_ftw(start, step, length) if mod == mp.GEO else lfm_ftw(start, step, length))[-1]
    print(f"{mp.MOD_NAMES[mod]}: {start/mp.K/1e3:.2f} -> {end/mp.K/1e3:.2f} kHz, pulse {length} clocks "
          f"({length / mp.F_CLK * 1e6:.1f} us), every {p['period_clk']} clocks; amp {p['amp_q12'] / 4096:.3f}")
    if p["code"]:
        print(f"phase code Barker-13 ({p['code']:#06x}), {p['chip_len']} clocks per chip")
    print(f"ftw_start {start}  ftw_step {step}  win_step {p['win_step']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=list(KINDS))
    ap.add_argument("--fc", type=float, required=True, help="centre / carrier frequency, Hz")
    ap.add_argument("--bw", type=float, default=0, help="sweep bandwidth, Hz (chirp, geo)")
    ap.add_argument("--amp", type=float, default=1.0)
    ap.add_argument("--period", type=int, default=LEN + 12000, help="clocks between pulse starts")
    ap.add_argument("--capture", action="store_true", help="capture the next pulse and check it")
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--dry-run", action="store_true", help="print the frames instead of sending")
    ap.add_argument("--frames-out", help="append the UART frames as hex lines to this file (for top_tb)")
    ap.add_argument("--spi-out", help="append the bare packet as hex lines to this file (top_tb's ESP32)")
    args = ap.parse_args()
    mod = KINDS[args.kind]
    if mod in (mp.LFM, mp.GEO) and args.bw <= 0:
        sys.exit(f"a {args.kind} sweep needs --bw")
    if args.period <= LEN:
        sys.exit(f"--period ({args.period}) must be longer than the pulse ({LEN})")
    if (args.fc + args.bw / 2) > F_MAX or args.fc - args.bw / 2 <= 0:
        sys.exit(f"frequencies must stay between 0 and {F_MAX/1e6:.1f} MHz")

    packet = mp.build(mod, args.fc, args.bw, LEN / mp.F_CLK, args.amp, args.period / mp.F_CLK)
    describe(mp.parse(packet))
    frames = frame(0x01, packet) + (frame(0x02) if args.capture else b"")

    if args.spi_out:
        with open(args.spi_out, "a") as f:
            f.writelines(f"{b:02x}\n" for b in packet)
    if args.frames_out:
        with open(args.frames_out, "a") as f:
            f.writelines(f"{b:02x}\n" for b in frames)
    if args.dry_run or args.frames_out or args.spi_out:
        print("frames:", frames.hex(" "))
        sys.exit(0)

    from capture import open_board, read_capture, check
    with open_board(args.port, timeout=10) as s:
        s.reset_input_buffer()
        s.write(frames)
        print(f"sent to {args.port}")
        if args.capture:
            ok = check(*read_capture(s), "board_capture")
            sys.exit(0 if ok else 1)
