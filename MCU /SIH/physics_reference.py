#!/usr/bin/env python3
"""
physics_reference.py -- Python twin of adaptive_sonar/sonar_core.h

Purpose
  1. Give the ML owner the EXACT same physics/labels as the ESP32 firmware, so the
     dataset and the firmware can never disagree again.
  2. Let the team cross-check the C++ (see test_core.cpp / run_tests.sh).

It reads every number and the profile table straight from sonar_config.h, so
editing the config changes firmware and dataset together.

Usage
  python3 physics_reference.py --n 20000 --out sonar_dataset_v2.csv
"""
import argparse, math, os, re, random, csv

HERE = os.path.dirname(os.path.abspath(__file__))
# sonar_config.h sits next to this file (flat folder) or in ../adaptive_sonar (the zip's layout)
CONFIG = next((c for c in (os.path.join(HERE, "sonar_config.h"),
                           os.path.join(HERE, "..", "adaptive_sonar", "sonar_config.h"))
               if os.path.exists(c)), os.path.join(HERE, "sonar_config.h"))

# ----------------------------------------------------------- config parsing
def _num(tok):
    tok = tok.strip().rstrip("fFuUlL")
    return float(tok)

def load_config(path=CONFIG):
    txt = open(path).read()
    cfg = {}
    for m in re.finditer(r"^\s*#define\s+([A-Z0-9_]+)\s+\(?\s*(-?[0-9.]+(?:[eE][-+]?[0-9]+)?[fFuUlL]*)\s*\)?", txt, re.M):
        cfg[m.group(1)] = _num(m.group(2))
    profs = []
    for m in re.finditer(r'\{\s*"([^"]+)"\s*,\s*([0-9.eE+-]+)f?\s*,\s*([0-9.eE+-]+)f?\s*,\s*([0-9.eE+-]+)f?\s*,\s*(MOD_\w+)\s*\}', txt):
        profs.append(dict(name=m.group(1), fc=float(m.group(2)), bw=float(m.group(3)),
                          T=float(m.group(4)), mod=m.group(5)))
    return cfg, profs

CFG, PROFILES = load_config()
C = CFG  # shorthand

# ---------------------------------------------------------------- physics
def mackenzie(T, S, D):
    return (1448.96 + 4.591*T - 5.304e-2*T**2 + 2.374e-4*T**3 + 1.340*(S-35)
            + 1.630e-2*D + 1.675e-7*D**2 - 1.025e-2*T*(S-35) - 7.139e-13*T*D**3)

def fg_absorption_db_km(fk, T, S, D, pH, c):
    Tk = T + 273.0
    A1 = 8.86/c * 10**(0.78*pH - 5); f1 = 2.8*math.sqrt(S/35)*10**(4 - 1245/Tk)
    A2 = 21.44*S/c*(1 + 0.025*T);    P2 = 1 - 1.37e-4*D + 6.2e-9*D*D
    f2 = 8.17*10**(8 - 1990/Tk)/(1 + 0.0018*(S-35))
    if T <= 20: A3 = 4.937e-4 - 2.59e-5*T + 9.11e-7*T**2 - 1.50e-8*T**3
    else:       A3 = 3.964e-4 - 1.146e-5*T + 1.45e-7*T**2 - 6.5e-10*T**3
    P3 = 1 - 3.83e-5*D + 4.9e-10*D*D
    f_sq = fk*fk
    return A1*f1*f_sq/(f_sq+f1*f1) + A2*P2*f2*f_sq/(f_sq+f2*f2) + A3*P3*f_sq

def turb_atten_db_km(fk, ntu):
    return C["TURB_ATTEN_DB_KM_PER_NTU_AT_100K"]*ntu*(fk/100.0)**C["TURB_FREQ_EXPONENT"]

def ambient_noise_db(fk):
    th = -15 + 20*math.log10(fk)
    wd = (50 + 7.5*math.sqrt(C["ASSUMED_WIND_SPEED_MS"]) + 20*math.log10(fk)
          - 40*math.log10(fk + 0.4))
    return 10*math.log10(10**(th/10) + 10**(wd/10))

def energy_budget_j(soc):
    return C["ENERGY_PER_PING_MAX_J"]*min(max(soc/100.0, C["SOC_FLOOR_PCT"]/100.0), 1.0)

def in_band(p):
    return p["bw"] > 0 and p["T"] > 0 and p["fc"]-p["bw"]/2 >= C["BAND_MIN_HZ"] and p["fc"]+p["bw"]/2 <= C["BAND_MAX_HZ"]

def evaluate(p, env, c, R, e_budget, extra_db=0.0):
    fk = p["fc"]/1000.0
    aw = fg_absorption_db_km(fk, env["T"], env["S"], env["D"], C["ASSUMED_PH"], c)
    at = turb_atten_db_km(fk, env["turb"])
    Rm = max(R, 1.0)
    tl = C["SPREADING_COEFF"]*math.log10(Rm) + (aw + at)*Rm/1000.0
    n0 = ambient_noise_db(fk) + C["RX_NOISE_FIGURE_DB"]
    echo = int(C.get("ACTIVE_ECHO_MODE", 0))
    path = (2*tl - C["TARGET_STRENGTH_DB"]) if echo else tl
    need = C["DETECTION_THRESHOLD_DB"] + C["LINK_MARGIN_DB"] + extra_db
    L = need + path + n0 - 170.8 - C["TX_DIRECTIVITY_DB"]
    try: e_req = 10**(L/10)/C["TX_EFFICIENCY"]
    except OverflowError: e_req = float("inf")
    p_req = e_req/p["T"]
    p_cap = min(C["TX_ELEC_POWER_MAX_W"], e_budget/p["T"])
    ok = in_band(p) and p_req <= p_cap
    if echo and p["T"] > 2*Rm/c: ok = False
    p_floor = C["AMP_MIN_FRAC"]**2*C["TX_ELEC_POWER_MAX_W"]
    p_use = max(p_req, p_floor) if ok else p_cap
    p_use = min(p_use, p_cap)
    snr = 170.8 + C["TX_DIRECTIVITY_DB"] + 10*math.log10(C["TX_EFFICIENCY"]*p_use*p["T"]) - path - n0
    return dict(feasible=ok, p_req=p_req, p_cap=p_cap, p_use=p_use,
                amp=math.sqrt(p_use/C["TX_ELEC_POWER_MAX_W"]), snr=snr, tl=tl, n0=n0, aw=aw, at=at)

def decide(env, R, prev=-1):
    """Same rule as sonar::decide() without an ML hint: the finest profile that closes the link
    and uses at most max(DETAIL_ENERGY_FACTOR x the cheapest working profile's energy,
    DETAIL_FREE_ENERGY_FRAC x the ping's energy budget). Profiles finer than prev must clear
    the switch-up margin. Returns (profile, link_ok, evaluation, sound speed)."""
    R = min(max(R, C["MIN_TARGET_RANGE_M"]), C["MAX_TARGET_RANGE_M"])
    c = mackenzie(env["T"], env["S"], env["D"])
    eb = energy_budget_j(env["soc"])
    evs, plain = [], []
    for i, p in enumerate(PROFILES):
        extra = C["SWITCH_UP_HYSTERESIS_DB"] if (prev >= 0 and i < prev) else 0.0
        plain.append(evaluate(p, env, c, R, eb, 0.0))
        evs.append(evaluate(p, env, c, R, eb, extra) if extra > 0 else plain[-1])
    energies = [e["p_use"] * p["T"] for e, p in zip(evs, PROFILES)]
    works = [e["p_use"] * p["T"] for e, p in zip(plain, PROFILES) if e["feasible"]]
    if works:
        allowance = max(C["DETAIL_ENERGY_FACTOR"] * min(works), C["DETAIL_FREE_ENERGY_FRAC"] * eb)
        for i, (e, en) in enumerate(zip(evs, energies)):
            if e["feasible"] and en <= allowance:
                return i, True, e, c
        for i, (e, p) in enumerate(zip(plain, PROFILES)):        # the margin ruled everything out
            if e["feasible"] and e["p_use"] * p["T"] <= allowance:
                return i, True, e, c
    i = len(PROFILES) - 1
    return i, False, plain[i], c


def amplitude_needed(env, R):
    """For every profile: the amplitude that just closes the link, sqrt(P_needed / P_max), before
    the AMP_MIN_FRAC floor (the firmware applies the floor) -- above 1 when even full power is not
    enough. A smooth function of the inputs; what the ML's amplitude output learns."""
    R = min(max(R, C["MIN_TARGET_RANGE_M"]), C["MAX_TARGET_RANGE_M"])
    c = mackenzie(env["T"], env["S"], env["D"])
    eb = energy_budget_j(env["soc"])
    return [math.sqrt(evaluate(p, env, c, R, eb)["p_req"] / C["TX_ELEC_POWER_MAX_W"]) for p in PROFILES]


# ------------------------------------------------ modulation from motion
MOD_CW, MOD_LFM, MOD_GEO, MOD_BPSK = 0, 1, 2, 3
MOD_NAMES = {MOD_CW: "cw", MOD_LFM: "lfm", MOD_GEO: "geo", MOD_BPSK: "bpsk"}

def doppler_cycles(p, v, c):
    """Phase drift over one pulse from motion: fd x T, fd = k x speed / c x fc (k = 2 in echo mode)."""
    k = 2.0 if int(C.get("ACTIVE_ECHO_MODE", 0)) else 1.0
    return k * abs(v) / c * p["fc"] * p["T"]

def tolerates(mod, cycles):
    if mod == MOD_BPSK: return cycles <= C["DOPPLER_BPSK_MAX_CYCLES"]
    if mod == MOD_LFM:  return cycles <= C["DOPPLER_LFM_MAX_CYCLES"]
    return mod == MOD_GEO

def modulation_for(cycles):
    """Same as sonar::modulationFor(): the least tolerant modulation that takes the drift."""
    for m in (MOD_BPSK, MOD_LFM):
        if tolerates(m, cycles):
            return m
    return MOD_GEO


# ------------------------------------------------------------ dataset maker
def generate_dataset(n, seed, out_csv):
    """Uniform random scenarios over the sensor ranges + target range -> physics label.
       (Range is a required ML input: it is not a sensor but it decides the answer.)"""
    rnd = random.Random(seed)
    cols = ["temperature","salinity","depth","turbidity","battery","range",
            "sound_speed","selected_profile","profile_name","link_ok","fc_hz","bw_hz","pulse_s",
            "amplitude_frac","tx_power_w","snr_out_db","speed","modulation"]
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f); w.writerow(cols)
        for _ in range(n):
            env = dict(T=rnd.uniform(C["TEMP_MIN_C"], C["TEMP_MAX_C"]),
                       S=rnd.uniform(C["SALINITY_MIN_PPT"], C["SALINITY_MAX_PPT"]),
                       D=rnd.uniform(C["DEPTH_MIN_M"], C["DEPTH_MAX_M"]),
                       turb=rnd.uniform(C["TURBIDITY_MIN_NTU"], C["TURBIDITY_MAX_NTU"]),
                       soc=rnd.uniform(C["BATTERY_MIN_PCT"], C["BATTERY_MAX_PCT"]))
            R = rnd.uniform(C["MIN_TARGET_RANGE_M"], C["MAX_TARGET_RANGE_M"])
            v = rnd.uniform(C["SPEED_MIN_MS"], C["SPEED_MAX_MS"])
            i, ok, ev, c = decide(env, R, prev=-1)
            p = PROFILES[i]
            w.writerow([f"{env['T']:.4f}", f"{env['S']:.4f}", f"{env['D']:.3f}", f"{env['turb']:.4f}",
                        f"{env['soc']:.4f}", f"{R:.3f}", f"{c:.4f}", i, p["name"], int(ok),
                        int(p["fc"]), int(p["bw"]), p["T"], f"{ev['amp']:.5f}", f"{ev['p_use']:.5f}",
                        f"{ev['snr']:.4f}", f"{v:.4f}",
                        MOD_NAMES[modulation_for(doppler_cycles(p, v, c))]])
    print(f"wrote {n} rows -> {out_csv}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="sonar_dataset_v2.csv")
    a = ap.parse_args()
    print(f"loaded {len(CFG)} config values and {len(PROFILES)} profiles from sonar_config.h")
    generate_dataset(a.n, a.seed, a.out)
