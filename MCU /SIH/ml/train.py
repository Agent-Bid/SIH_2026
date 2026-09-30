"""Train the ESP32's waveform-choice model and export it to C.

The labels come from the physics twin (../physics_reference.py, which reads the same
sonar_config.h as the firmware). For each random scenario the physics gives:
  - the profile it picks: the finest one that closes the link within the energy allowance
    (the detail-vs-energy rule in sonar_config.h, section 4b);
  - for every profile, the amplitude that just closes the link.
Two small neural networks learn these from the six values the firmware hands to mlSelect():
temperature, salinity, depth, turbidity, battery and target range.
  classifier: 6 -> 48 -> 48 -> one score per profile the rule ever picks; the highest wins;
  amplitude:  6 inputs + the profile (one-hot) -> 48 -> 48 -> log(amplitude that just closes
              the link, before the 5 % floor), trained only where that amplitude matters.
XGBoost and a decision tree are trained on the same classification data as a comparison.

  ml/.venv/bin/python ml/train.py              # full run (~5 minutes on a laptop CPU)
  ml/.venv/bin/python ml/train.py --quick      # small data, for a fast check

Writes ../ml_model.h (both networks as C, next to the sketch), ml/test_vectors.csv (inputs,
expected class, score margin and amplitude, for tools/test_core.cpp) and ml/REPORT.md.
Later, the same code can learn from measured results: replace make_set()'s labels with logged
field data.
"""
import argparse
import math
import os
import random
import sys
import time
import warnings
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import physics_reference as phys                                 # noqa: E402

from sklearn.exceptions import ConvergenceWarning                 # noqa: E402
from sklearn.neural_network import MLPClassifier, MLPRegressor    # noqa: E402
from sklearn.preprocessing import StandardScaler                  # noqa: E402
from sklearn.tree import DecisionTreeClassifier                   # noqa: E402
import xgboost as xgb                                             # noqa: E402

C = phys.C
N_CLASSES = len(phys.PROFILES)
NAMES = [p["name"] for p in phys.PROFILES]
SEED_TRAIN, SEED_TEST_UNIFORM, SEED_TEST_LOG = 11, 22, 33


# ------------------------------------------------------------------ data
def scenario(rnd, log_range):
    env = dict(T=rnd.uniform(C["TEMP_MIN_C"], C["TEMP_MAX_C"]),
               S=rnd.uniform(C["SALINITY_MIN_PPT"], C["SALINITY_MAX_PPT"]),
               D=rnd.uniform(C["DEPTH_MIN_M"], C["DEPTH_MAX_M"]),
               turb=rnd.uniform(C["TURBIDITY_MIN_NTU"], C["TURBIDITY_MAX_NTU"]),
               soc=rnd.uniform(C["BATTERY_MIN_PCT"], C["BATTERY_MAX_PCT"]))
    lo, hi = C["MIN_TARGET_RANGE_M"], C["MAX_TARGET_RANGE_M"]
    R = math.exp(rnd.uniform(math.log(lo), math.log(hi))) if log_range else rnd.uniform(lo, hi)
    return env, R


def make_set(n, seed, log_range_share):
    """n scenarios labelled by the physics (profile, link ok, amplitude each profile needs). A share of them draw the range log-uniformly (more
    short ranges, where the fine profiles live), the rest uniformly (the generator's default).
    Inputs are rounded to float32 first, as the firmware sees them."""
    rnd = random.Random(seed)
    X = np.empty((n, 6), dtype=np.float32)
    y = np.empty(n, dtype=np.int32)
    link_ok = np.empty(n, dtype=bool)
    amp = np.empty((n, N_CLASSES), dtype=np.float32)
    f = lambda v: float(np.float32(v))
    for i in range(n):
        env, R = scenario(rnd, rnd.random() < log_range_share)
        env = {k: f(v) for k, v in env.items()}
        R = f(R)
        k, ok, _, _ = phys.decide(env, R, prev=-1)
        X[i] = (env["T"], env["S"], env["D"], env["turb"], env["soc"], R)
        y[i], link_ok[i] = k, ok
        amp[i] = phys.amplitude_needed(env, R)
    return X, y, link_ok, amp


def acceptable(x, k):
    """Would the firmware keep profile k for inputs x? It must close the link and use no more
    energy than the allowance (what sonar::decide() checks for an ML pick, from a fresh start)."""
    env = dict(T=float(x[0]), S=float(x[1]), D=float(x[2]), turb=float(x[3]), soc=float(x[4]))
    c = phys.mackenzie(env["T"], env["S"], env["D"])
    eb = phys.energy_budget_j(env["soc"])
    evs = [phys.evaluate(p, env, c, float(x[5]), eb) for p in phys.PROFILES]
    works = [e["p_use"] * p["T"] for e, p in zip(evs, phys.PROFILES) if e["feasible"]]
    if not works:
        return False
    allowance = max(C["DETAIL_ENERGY_FACTOR"] * min(works), C["DETAIL_FREE_ENERGY_FRAC"] * eb)
    return evs[k]["feasible"] and evs[k]["p_use"] * phys.PROFILES[k]["T"] <= allowance


# ------------------------------------------------------------------ metrics
def metrics(X, y, link_ok, pred):
    ok_idx = np.nonzero(link_ok)[0]
    kept = np.array([acceptable(X[i], pred[i]) for i in ok_idx])
    be_idx = np.nonzero(~link_ok)[0]
    return dict(
        agreement=float(np.mean(pred == y)),
        kept=float(kept.mean()),                                   # a profile works and the ML's is kept
        overridden=1 - float(kept.mean()),                         # fails the link or the energy allowance
        coarser=float(np.mean(pred[ok_idx][kept] > y[ok_idx][kept])),  # kept, but coarser than the physics'
        best_effort_same=float(np.mean(pred[be_idx] == N_CLASSES - 1)),
        per_class={NAMES[k]: float(np.mean(pred[y == k] == k)) for k in range(N_CLASSES) if np.any(y == k)})


class Relabelled:
    """XGBoost wants classes 0..n-1; the rule only ever picks some of the 12 profiles."""
    def __init__(self, model):
        self.model = model
    def fit(self, X, y):
        self.classes_ = np.unique(y)
        self.model.fit(X, np.searchsorted(self.classes_, y))
        return self
    def predict(self, X):
        return self.classes_[self.model.predict(X)]
    def get_booster(self):
        return self.model.get_booster()


# ------------------------------------------------------------------ inputs of the network
def net_inputs(X):
    """Range as log10(range): the loss grows with log(range) and range spans 10..2000 m."""
    Z = X.astype(np.float64).copy()
    Z[:, 5] = np.log10(Z[:, 5])
    return Z


def product_features(X):
    """For the XGBoost comparison: simple products that let trees follow the curved boundaries."""
    t, s, d, tb, b, r = X.T.astype(np.float64)
    return np.column_stack([t, s, d, tb, b, r, np.log10(r), r * t, r * tb, r * d, r * s, r * t * t])


# ------------------------------------------------------------------ the networks in float32, as in C
AMP_FLOOR = C["AMP_MIN_FRAC"]                     # the firmware never drives below this


def forward32(model, scaler, X):
    """Outputs computed in float32, as the C code does."""
    x = ((net_inputs(X) - scaler.mean_) / scaler.scale_).astype(np.float32)
    for W, b in zip(model.coefs_[:-1], model.intercepts_[:-1]):
        x = np.maximum(x @ W.astype(np.float32) + b.astype(np.float32), np.float32(0))
    return x @ model.coefs_[-1].astype(np.float32) + model.intercepts_[-1].astype(np.float32)


def amp_inputs(scaler, X, k, used):
    """The amplitude network's inputs: the 6 normalised inputs + a one-hot of the profile."""
    oh = np.zeros((len(X), len(used)), dtype=np.float32)
    oh[np.arange(len(X)), np.searchsorted(used, k)] = 1
    return np.hstack([((net_inputs(X) - scaler.mean_) / scaler.scale_).astype(np.float32), oh])


def amp_rows(scaler, X, A, used):
    """Training rows for the amplitude network: every (scenario, profile) pair whose needed
    amplitude is in the range that matters (a bit below the floor up to a bit above full power)."""
    Z, t = [], []
    for k in used:
        keep = (A[:, k] >= 0.3 * AMP_FLOOR) & (A[:, k] <= 1.5)
        Z.append(amp_inputs(scaler, X[keep], np.full(keep.sum(), k), used))
        t.append(np.log(A[keep, k]))
    return np.vstack(Z), np.concatenate(t)


def amp_forward32(amp_model, scaler, X, k, used):
    x = amp_inputs(scaler, X, k, used)
    for W, b in zip(amp_model.coefs_[:-1], amp_model.intercepts_[:-1]):
        x = np.maximum(x @ W.astype(np.float32) + b.astype(np.float32), np.float32(0))
    return (x @ amp_model.coefs_[-1].astype(np.float32) + amp_model.intercepts_[-1].astype(np.float32))[:, 0]


def amp_out(amp_model, scaler, X, k, used, margin):
    """The amplitude the ESP32 will send for profile k: exp(network output + safety margin)."""
    return np.exp(amp_forward32(amp_model, scaler, X, k, used) + np.float32(margin)).astype(np.float32)


def export_c(clf, amp_model, amp_margin, scaler, path, info):
    def fl(v):                                        # a C float literal: 3.0f, not 3f
        t = f"{np.float32(v).item():.9g}"
        return (t if any(c in t for c in ".en") else t + ".0") + "f"
    def arr(name, a):
        a = np.asarray(a, dtype=np.float32)
        if a.ndim == 1:
            return f"static const float {name}[{a.shape[0]}] = {{ {', '.join(fl(v) for v in a)} }};\n"
        rows = ",\n".join("  { " + ", ".join(fl(v) for v in row) + " }" for row in a)
        return f"static const float {name}[{a.shape[0]}][{a.shape[1]}] = {{\n{rows}\n}};\n"
    sizes = [clf.coefs_[0].shape[0]] + [W.shape[1] for W in clf.coefs_]
    asizes = [amp_model.coefs_[0].shape[0]] + [W.shape[1] for W in amp_model.coefs_]
    assert len(sizes) == 4 and asizes == [sizes[0] + sizes[3], sizes[1], sizes[2], 1]
    with open(path, "w") as f:
        f.write(f"""// Generated by ml/train.py -- do not edit. {info}
// Waveform-choice model: two neural networks, {' -> '.join(map(str, sizes))} (profile) and
// {' -> '.join(map(str, asizes))} (amplitude), ReLU, trained on the physics twin's decisions.
// Inputs in the firmware's units:
//   temperature C, salinity ppt, depth m, turbidity NTU, battery %, target range m.
// ml_predict_profile()   -> the profile 0..{N_CLASSES - 1} (the classifier's highest score)
// ml_predict_amplitude() -> the drive amplitude 0..1 for that profile (the physics raises it if short)
#pragma once
#include <math.h>

namespace ml {{

static const int ML_IN = {sizes[0]}, ML_H1 = {sizes[1]}, ML_H2 = {sizes[2]};
// The classifier scores only the profiles the rule ever picks; ML_CLASSES maps its outputs to profiles.
// The amplitude network takes the 6 inputs plus a one-hot of the profile (its position in ML_CLASSES).
static const int ML_NCLASS = {sizes[3]}, MA_IN = ML_IN + ML_NCLASS;
static const int ML_CLASSES[ML_NCLASS] = {{ {", ".join(str(int(c)) for c in clf.classes_)} }};
// Input normalisation (both networks): input 5 (range) is log10(range) first, then every input is
// (x - ML_MEAN) / ML_SCALE.
""")
        f.write(arr("ML_MEAN", scaler.mean_) + arr("ML_SCALE", scaler.scale_))
        f.write("\n// Classifier: one score per entry of ML_CLASSES\n")
        for i, (W, b) in enumerate(zip(clf.coefs_, clf.intercepts_), 1):
            f.write(arr(f"ML_W{i}", W) + arr(f"ML_B{i}", b))
        f.write("\n// Amplitude network: log(amplitude that just closes the link); the margin makes it rarely short\n")
        for i, (W, b) in enumerate(zip(amp_model.coefs_, amp_model.intercepts_), 1):
            f.write(arr(f"MA_W{i}", W) + arr(f"MA_B{i}", b))
        f.write(f"static const float MA_MARGIN = {fl(amp_margin)};\n")
        f.write("""
// one network: NIN normalised inputs -> N outputs
template <int NIN, int N>
inline void ml_forward(const float x[NIN],
                       const float W1[NIN][ML_H1], const float B1[ML_H1],
                       const float W2[ML_H1][ML_H2], const float B2[ML_H2],
                       const float W3[ML_H2][N], const float B3[N], float out[N]) {
  float h1[ML_H1], h2[ML_H2];
  for (int j = 0; j < ML_H1; j++) {
    float s = B1[j];
    for (int i = 0; i < NIN; i++) s += x[i] * W1[i][j];
    h1[j] = s > 0.0f ? s : 0.0f;
  }
  for (int j = 0; j < ML_H2; j++) {
    float s = B2[j];
    for (int i = 0; i < ML_H1; i++) s += h1[i] * W2[i][j];
    h2[j] = s > 0.0f ? s : 0.0f;
  }
  for (int j = 0; j < N; j++) {
    float s = B3[j];
    for (int i = 0; i < ML_H2; i++) s += h2[i] * W3[i][j];
    out[j] = s;
  }
}

inline void ml_normalise(const float in[ML_IN], float x[ML_IN]) {
  for (int i = 0; i < ML_IN; i++)
    x[i] = ((i == 5 ? log10f(in[i]) : in[i]) - ML_MEAN[i]) / ML_SCALE[i];
}

inline int ml_predict_profile(const float in[ML_IN]) {
  float x[ML_IN], score[ML_NCLASS];
  ml_normalise(in, x);
  ml_forward<ML_IN, ML_NCLASS>(x, ML_W1, ML_B1, ML_W2, ML_B2, ML_W3, ML_B3, score);
  int best = 0;
  for (int j = 1; j < ML_NCLASS; j++) if (score[j] > score[best]) best = j;
  return ML_CLASSES[best];
}

// -1 for a profile the model does not know (then the physics sets the amplitude)
inline float ml_predict_amplitude(const float in[ML_IN], int profile) {
  float x[MA_IN], logamp[1];
  ml_normalise(in, x);
  int c = -1;
  for (int j = 0; j < ML_NCLASS; j++) { x[ML_IN + j] = 0.0f; if (ML_CLASSES[j] == profile) c = j; }
  if (c < 0) return -1.0f;
  x[ML_IN + c] = 1.0f;
  ml_forward<MA_IN, 1>(x, MA_W1, MA_B1, MA_W2, MA_B2, MA_W3, MA_B3, logamp);
  const float a = expf(logamp[0] + MA_MARGIN);
  return a > 1.0f ? 1.0f : a;
}

}  // namespace ml
""")
    count = lambda m: sum(W.size for W in m.coefs_) + sum(b.size for b in m.intercepts_)
    return count(clf), count(amp_model)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    n_train, n_test = (40_000, 5_000) if a.quick else (300_000, 20_000)
    warnings.filterwarnings("ignore", category=ConvergenceWarning)
    report = []
    log = lambda s="": (print(s, flush=True), report.append(s))

    t0 = time.time()
    X, y, ok, amp = make_set(n_train, SEED_TRAIN, log_range_share=0.5)
    tests = {"uniform range": make_set(n_test, SEED_TEST_UNIFORM, 0.0),     # like the default generator
             "log range": make_set(n_test, SEED_TEST_LOG, 1.0)}             # more short ranges
    log(f"Data: {n_train:,} training scenarios (half with log-uniform range), 2 x {n_test:,} test "
        f"scenarios with other seeds; labelled by the physics twin in {time.time() - t0:.0f} s.")
    log(f"Rule: finest profile that closes the link using at most max({C['DETAIL_ENERGY_FACTOR']:g} x the "
        f"cheapest working profile's energy, {C['DETAIL_FREE_ENERGY_FRAC']:.0%} of the ping's energy budget).")
    counts = np.bincount(y, minlength=N_CLASSES)
    log()
    log("Training labels: " + ", ".join(f"{NAMES[k]} {counts[k] / len(y):.1%}" for k in range(N_CLASSES))
        + f". In {np.mean(~ok):.1%} of them no profile closes the link (best effort, {NAMES[-1]}).")

    # ---- profile: classifier comparison
    scaler = StandardScaler().fit(net_inputs(X))
    nets = {}
    candidates = [
        ("decision tree, depth 14", lambda: DecisionTreeClassifier(max_depth=14, random_state=0), None),
        ("XGBoost depth 5, 60 rounds", lambda: Relabelled(xgb.XGBClassifier(
            n_estimators=60, max_depth=5, learning_rate=0.3, tree_method="hist", n_jobs=-1, random_state=0)), None),
        ("XGBoost depth 5, 60 rounds + product inputs", lambda: Relabelled(xgb.XGBClassifier(
            n_estimators=60, max_depth=5, learning_rate=0.3, tree_method="hist", n_jobs=-1, random_state=0)),
         product_features),
        ("neural net 6-48-48-n", lambda: MLPClassifier(
            hidden_layer_sizes=(48, 48), max_iter=400, early_stopping=True, n_iter_no_change=20,
            random_state=0), "net"),
    ]
    log()
    log("## Profile choice")
    log()
    log("| Model | Size | Agreement (uniform / log range) | Overridden | Coarser | Train |")
    log("|---|---|---|---|---|---|")
    for name, make, feats in candidates:
        model = make()
        prep = (lambda Z: scaler.transform(net_inputs(Z))) if feats == "net" else (feats or (lambda Z: Z))
        t0 = time.time()
        model.fit(prep(X), y)
        dt = time.time() - t0
        m = {k: metrics(Xt, yt, okt, model.predict(prep(Xt))) for k, (Xt, yt, okt, _) in tests.items()}
        if feats == "net":
            size = f"{sum(W.size for W in model.coefs_) + sum(b.size for b in model.intercepts_):,} weights"
            nets[name] = (model, m)
        elif isinstance(model, DecisionTreeClassifier):
            size = f"{model.tree_.node_count:,} nodes"
        else:
            size = f"{sum(d.count(chr(10)) + 1 for d in model.get_booster().get_dump()):,} nodes"
        worst = lambda key: max(m[k][key] for k in m)
        log(f"| {name} | {size} | {m['uniform range']['agreement']:.2%} / {m['log range']['agreement']:.2%} | "
            f"{worst('overridden'):.2%} | {worst('coarser'):.2%} | {dt:.0f} s |")
    log()
    log("Agreement: same profile as the physics. Overridden: a profile can close the link but the ML's")
    log("pick either cannot or uses more energy than the allowance, so the physics replaces it")
    log("(FLAG_ML_OVERRIDDEN). Coarser: the ML's pick is kept but gives less detail than the physics'")
    log("(it then also uses less energy). Worst of the two test sets.")

    name, (clf, m) = max(nets.items(), key=lambda kv: sum(v["agreement"] for v in kv[1][1].values()))
    log()
    log(f"On the ESP32: {name}.")
    log("Per-class recall, uniform / log range: " + ", ".join(
        f"{k} {m['uniform range']['per_class'][k]:.0%}/{m['log range']['per_class'][k]:.0%}"
        for k in NAMES if k in m['uniform range']['per_class'] and k in m['log range']['per_class'])
        + f" (the other {N_CLASSES - len(clf.classes_)} profiles are never the rule's choice).")
    log(f"Best-effort cases where the ML also picks {NAMES[-1]}: "
        f"{m['uniform range']['best_effort_same']:.2%} / {m['log range']['best_effort_same']:.2%}.")

    # ---- amplitude: log(amplitude needed) for all 12 profiles at once
    used = clf.classes_
    Za, ta = amp_rows(scaler, X, amp, used)
    t0 = time.time()
    amp_model = MLPRegressor(hidden_layer_sizes=(48, 48), max_iter=400, early_stopping=True,
                             n_iter_no_change=20, random_state=0)
    amp_model.fit(Za, ta)
    dt = time.time() - t0
    # safety margin: from the uniform test set, so that the ML's amplitude is rarely below what is needed
    Xu, yu, oku, Au = tests["uniform range"]
    ku = clf.predict(scaler.transform(net_inputs(Xu)))
    need_u = Au[np.arange(len(Xu)), ku]
    usable = oku & (need_u <= 1.0) & (need_u >= AMP_FLOOR)     # below the floor, the floor decides
    err = (np.log(need_u) - amp_forward32(amp_model, scaler, Xu, ku, used))[usable]
    amp_margin = float(np.quantile(err, 0.98))
    log()
    log("## Amplitude")
    log()
    log(f"Network {6 + len(used)}-48-48-1 (6 inputs + the profile, one-hot) predicting log(amplitude that just "
        f"closes the link, before the {AMP_FLOOR:.0%} floor), trained on the {len(ta):,} (scenario, profile) pairs "
        f"where that amplitude matters, in {dt:.0f} s; "
        f"safety margin {amp_margin:+.4f} in log (x{math.exp(amp_margin):.3f}), set so that 98 % of the "
        "uniform-range test picks above the floor need no raise. The firmware applies the floor.")
    log()
    log("| Test set | Amplitude error (median / 95th pct) | Raised by the physics | Extra energy vs just enough (median / mean) |")
    log("|---|---|---|---|")
    for tname, (Xt, yt, okt, At) in tests.items():
        k = clf.predict(scaler.transform(net_inputs(Xt)))
        need = At[np.arange(len(Xt)), k]
        sel = okt & (need <= 1.0)
        a_ml = np.maximum(amp_out(amp_model, scaler, Xt, k, used, amp_margin)[sel], np.float32(AMP_FLOOR))
        need = np.maximum(need[sel], np.float32(AMP_FLOOR))
        rel = np.abs(a_ml / need - 1)
        raised = a_ml < need
        extra = (np.maximum(a_ml, need) / need) ** 2 - 1
        log(f"| {tname} | {np.median(rel):.2%} / {np.quantile(rel, 0.95):.2%} | {raised.mean():.2%} | "
            f"{np.median(extra):.2%} / {extra.mean():.2%} |")
    log()
    log("Amplitude error: |ML amplitude / amplitude that just closes the link - 1|, for the ML's own pick.")
    log("Raised: the ML's amplitude was short, so the physics raised it (FLAG_ML_AMP_RAISED). Extra energy:")
    log("what the ML's amplitude costs above the minimum (energy goes with amplitude squared).")

    # ---- export and the C test vectors
    Xt = np.concatenate([tests["uniform range"][0][:3000], tests["log range"][0][:3000]])
    s32 = forward32(clf, scaler, Xt)
    k32 = clf.classes_[s32.argmax(1)]
    same = np.mean(k32 == clf.predict(scaler.transform(net_inputs(Xt))))
    top2 = np.sort(s32, axis=1)[:, -2:]
    a32 = np.minimum(amp_out(amp_model, scaler, Xt, k32, used, amp_margin), 1.0)  # as ml_predict_amplitude()
    log()
    log(f"float32 (as on the ESP32) vs float64: same profile on {same:.2%} of {len(Xt):,} vectors.")
    info = (f"{name}; profile agreement with the physics {m['uniform range']['agreement']:.2%} (uniform range) / "
            f"{m['log range']['agreement']:.2%} (log range)")
    n1, n2 = export_c(clf, amp_model, amp_margin, scaler, os.path.join(HERE, "..", "ml_model.h"), info)
    with open(os.path.join(HERE, "test_vectors.csv"), "w") as f:
        f.write("# temperature,salinity,depth,turbidity,battery,range,profile,score margin,amplitude (float32)\n")
        for x, k, t, av in zip(Xt, k32, top2, a32):
            f.write(",".join(f"{np.float32(v).item():.9g}" for v in x) + f",{k},{t[1] - t[0]:.6g},{av:.7g}\n")
    log(f"Wrote ml_model.h ({n1 + n2:,} weights = {(n1 + n2) * 4 / 1024:.1f} KB) and ml/test_vectors.csv "
        f"({len(Xt):,} vectors).")

    with open(os.path.join(HERE, "REPORT.md"), "w") as f:
        f.write("# Waveform-choice model: training report\n\nGenerated by `ml/train.py`.\n\n")
        f.write("\n".join(report) + "\n")


if __name__ == "__main__":
    main()
