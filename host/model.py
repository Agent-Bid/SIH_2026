"""Golden model of pulse_dds, tick by tick, plus helpers shared by the checkers.

Every register updates at the same instant, so each tick we work out all the next
values from the current ones, then swap them in.
"""
import numpy as np
from gen_sine import sine_table
from gen_window import hann_table

F_CLK = 27e6 * 13 / 7                  # ~50.142857 MHz
K = 2**32 / F_CLK                      # FTW per Hz


def hz(ftw):
    return ftw / K


def read_log(path):
    """Read a pulse log: header '# len=.. win_step=..', then rows of
    'start ftw_start ftw_step amp code chip_len out active'."""
    with open(path) as f:
        hdr = dict(kv.split("=") for kv in f.readline()[1:].split())
        rows = [line.split() for line in f]
    for i, r in enumerate(rows):
        if not r[6].lstrip("-").isdigit() or r[7] not in ("0", "1"):
            raise SystemExit(f"FAIL: tick {i} has out='{r[6]}' active='{r[7]}' "
                             "(x = unknown: something isn't driven yet)")
    inputs = [tuple(int(v) for v in r[:6]) for r in rows]
    out = np.array([int(r[6]) for r in rows])
    act = np.array([int(r[7]) for r in rows])
    return int(hdr["len"]), int(hdr["win_step"]), inputs, out, act


def golden(inputs, length, win_step, window=None, geo=False):
    """inputs: one (start, ftw_start, ftw_step, amp, code, chip_len) per tick.
    geo: geometric sweep instead of LFM. Returns (out, active)."""
    sine = sine_table()
    window = hann_table() if window is None else window
    active = count = ftw = phase = winph = tone = win = tone_r = win_r = shaped = out = 0
    tick_in_chip = chip = 0
    geo_n = geo_acc = acc = 0
    outs, acts = [], []
    for start, ftw_start, ftw_step, amp, code, chip_len in inputs:
        run_rst = not active
        flip = (code >> chip) & 1
        if start and not active:
            n_active, n_count = 1, 0
        elif active:
            n_active = 0 if count == length - 1 else 1
            n_count = (count + 1) % 2**24
        else:
            n_active, n_count = active, count
        # the sweep register holds FTW x 2^16; the engine uses its top 32 bits
        n_geo_n, n_geo_acc = geo_n, geo_acc
        if run_rst:
            n_acc, n_geo_n, n_geo_acc = ftw_start << 16, 0, 0
        elif not geo:
            step = ftw_step - 2**32 if ftw_step >= 2**31 else ftw_step
            n_acc = (acc + step) % 2**48
        else:                                       # shift-and-add: one bit of step per tick
            total = geo_acc + (acc if (ftw_step >> geo_n) & 1 else 0)
            n_geo_n = (geo_n + 1) % 32
            if geo_n == 31:
                n_acc, n_geo_acc = (acc + (total >> 1)) % 2**48, 0
            else:
                n_acc, n_geo_acc = acc, total >> 1
        ftw = acc >> 16
        n_phase = 0 if run_rst else (phase + ftw) % 2**32
        n_winph = 0 if run_rst else (winph + win_step) % 2**32
        if run_rst:
            n_tick, n_chip = 0, 0
        elif tick_in_chip == (chip_len - 1) % 2**24:
            n_tick, n_chip = 0, (chip + 1) % 16
        else:
            n_tick, n_chip = tick_in_chip + 1, chip
        n_tone = sine[((phase + (flip << 31)) % 2**32) >> 22]
        n_win = window[winph >> 22]
        n_tone_r, n_win_r = tone, win               # pipeline register before the multiply
        n_shaped = (tone_r * win_r) >> 12
        n_out = (shaped * amp) >> 12
        active, count, acc, phase, winph = n_active, n_count, n_acc, n_phase, n_winph
        tick_in_chip, chip = n_tick, n_chip
        geo_n, geo_acc = n_geo_n, n_geo_acc
        tone, win, tone_r, win_r, shaped, out = n_tone, n_win, n_tone_r, n_win_r, n_shaped, n_out
        outs.append(out)
        acts.append(active)
    return np.array(outs), np.array(acts)


def geo_ftw(ftw_start, ftw_step, length):
    """The ftw a geometric sweep has reached after each clock of a pulse (for plots)."""
    track, acc = [], ftw_start << 16
    for k in range(length):
        track.append(acc >> 16)
        if k % 32 == 31:
            acc = (acc + acc * ftw_step // 2**32) % 2**48
    return np.array(track)


def lfm_ftw(ftw_start, ftw_step, length):
    """The ftw an LFM sweep has reached after each clock (ftw_step = FTW per clock x 2^16)."""
    step = ftw_step - 2**32 if ftw_step >= 2**31 else ftw_step
    return ((ftw_start << 16) + step * np.arange(length, dtype=object)) % 2**48 >> 16


def pulses(act):
    """(first_tick, length) of every run of active = 1."""
    edges = np.diff(np.concatenate([[0], act, [0]]))
    starts, ends = np.nonzero(edges == 1)[0], np.nonzero(edges == -1)[0]
    return [(int(s), int(e - s)) for s, e in zip(starts, ends)]


def compare(sim_out, sim_act, gold_out, gold_act):
    bad = np.nonzero((sim_out != gold_out) | (sim_act != gold_act))[0]
    for i in bad[:10]:
        print(f"  tick {i}: sim out={sim_out[i]} active={sim_act[i]}   "
              f"expected out={gold_out[i]} active={gold_act[i]}")
    return len(bad)
