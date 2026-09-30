"""The ESP32's packet to the FPGA (SonarPacket v3 in MCU /SIH/sonar_core.h), in Python.

57 bytes, little-endian, CRC-8 (poly 0x07, init 0) over the first 56. The FPGA reads it
from SPI (the ESP32) or inside a UART frame A5 01 <packet> <xor> (the PC, host/wavegen.py).
fpga_wave() does the same unit conversion as fpgaWave() in sonar_core.h.
"""
import math
import struct

F_CLK = 27e6 * 13 / 7                       # FPGA_CLK_HZ
TWO32 = 2**32
K = TWO32 / F_CLK                           # FTW per Hz
LEN_MAX = 2**24 - 1                         # FPGA_LEN_MAX_CLK
BARKER13_CODE, BARKER13_CHIPS = 0x0A60, 13
PING_PERIOD_MIN_S = 0.1
CW, LFM, GEO, BPSK = 0, 1, 2, 3
MOD_NAMES = {CW: "cw", LFM: "lfm", GEO: "geo", BPSK: "bpsk"}
VERSION = 3

LAYOUT = "<BBHBBBBIIIHHIIIiIHHII"
FIELDS = ("sync version seq profile_id modulation spreading_factor flags "
          "center_freq_hz bandwidth_hz pulse_us amplitude tx_power_mw prop_delay_us "
          "len_clk ftw_start ftw_step win_step amp_q12 code chip_len period_clk").split()
SIZE = struct.calcsize(LAYOUT) + 1          # + crc8
assert SIZE == 57


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _round(x):
    return math.floor(x + 0.5)


def fpga_wave(mod, fc, bw, pulse_s):
    """(len_clk, ftw_start, ftw_step, win_step, code, chip_len), the same as fpgaWave()."""
    sweep = mod in (LFM, GEO)
    length = _round(pulse_s * F_CLK)
    chip = 0
    if mod == BPSK:
        chip = _round(length / BARKER13_CHIPS)
        length = chip * BARKER13_CHIPS
    f0 = fc - bw / 2 if sweep else fc
    f1 = fc + bw / 2 if sweep else fc
    step = 0
    if mod == LFM:
        step = _round((f1 - f0) * K / length * 2**16)
    elif mod == GEO:
        step = _round(((f1 / f0) ** (1 / (length // 32)) - 1) * TWO32)
    if not (64 <= length <= LEN_MAX and chip <= LEN_MAX and f0 > 0 and f1 * K < TWO32 / 2
            and 0 <= step < 2**31 - 1):
        raise ValueError(f"does not fit the FPGA: mod {mod} fc {fc} bw {bw} pulse {pulse_s}")
    return (length, _round(f0 * K), step, _round(TWO32 / length),
            BARKER13_CODE if mod == BPSK else 0, chip)


def period_clk(pulse_s, length, period_s=None):
    period_s = max(PING_PERIOD_MIN_S, pulse_s) if period_s is None else period_s
    return max(_round(period_s * F_CLK), length + 1)


def build(mod, fc, bw, pulse_s, amp=1.0, period_s=None, seq=0, profile_id=0xFF):
    """A complete 57-byte packet, as the ESP32 would send it."""
    length, ftw_start, step, win_step, code, chip = fpga_wave(mod, fc, bw, pulse_s)
    amp = min(max(amp, 0.0), 1.0)
    sf = _round(math.log2(bw * pulse_s)) if mod in (LFM, GEO) and bw > 0 else 0
    body = struct.pack(LAYOUT, 0xA5, VERSION, seq, profile_id, mod, sf, 0,
                       _round(fc), _round(bw), _round(pulse_s * 1e6), _round(amp * 65535), 0, 0,
                       length, ftw_start, step, win_step, _round(amp * 4096), code, chip,
                       period_clk(pulse_s, length, period_s))
    return body + bytes([crc8(body)])


def parse(packet):
    """57 bytes -> dict of fields; raises ValueError if the sync, version or CRC is wrong."""
    if len(packet) != SIZE or packet[0] != 0xA5 or packet[1] != VERSION:
        raise ValueError("not a v3 packet")
    if crc8(packet[:-1]) != packet[-1]:
        raise ValueError("bad CRC")
    return dict(zip(FIELDS, struct.unpack(LAYOUT, packet[:-1])))
