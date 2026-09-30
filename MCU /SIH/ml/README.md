# Waveform-choice model

The ESP32 chooses the waveform with two small neural networks, compiled into the firmware as
`../ml_model.h` (23.5 KB):

- **profile network** 6 → 48 → 48 → 6 (ReLU): one score for each profile the rule ever picks
  (X-S, H-S, M-S, L-S, L-M, L-L); the highest wins.
- **amplitude network** 12 → 48 → 48 → 1: the 6 inputs plus the chosen profile (one-hot) →
  log(amplitude that just closes the link), plus a small safety margin.

**Inputs**: temperature (°C), salinity (ppt), depth (m), turbidity (NTU), battery (%), target
range (m); the range is fed as log10(range), and every input is standardised.

**What it learns**: the physics' choice under the detail-vs-energy rule (`sonar_config.h`
section 4b): the finest profile that closes the link using at most 4 × the energy of the
cheapest working profile, or 1 % of the ping's energy budget, whichever is more; and the
amplitude that just closes the link for a profile. The physics still checks every answer:
the profile is kept when it passes the same test (`FLAG_ML_USED`), else replaced
(`FLAG_ML_OVERRIDDEN`); the amplitude is raised if it is too low (`FLAG_ML_AMP_RAISED`).
Set `USE_ML_MODEL 0` in `adaptive_sonar.ino` for physics only.

**Results** (`REPORT.md`): the profile agrees with the physics on about 99.2–99.4 % of
scenarios and 0.5 % of picks are overridden; the amplitude is within 1.8 % (median) of what is
needed and costs 1.6–3.4 % more energy than the minimum. XGBoost and a decision tree are
trained on the same data for comparison: the boundaries are smooth curves, which the network
fits with far fewer parameters. The labels come from the physics, so the model reproduces the
physics; it cannot be more accurate than it.

**Retrain** (about 5 minutes on a laptop CPU; no GPU needed):
```
python3 -m venv ml/.venv && ml/.venv/bin/pip install -r ml/requirements.txt
ml/.venv/bin/python ml/train.py          # writes ../ml_model.h, test_vectors.csv, REPORT.md
```
Retrain after any change to `sonar_config.h` that affects the choice (profiles, power,
thresholds, the detail-vs-energy knobs, sensor ranges), then run the firmware tests:
`tools/test_core.cpp` checks that the C networks give the Python models' profile and amplitude
on `test_vectors.csv`.

**Later, real data**: the model only beats the physics once it learns from measurements.
Log the inputs, the chosen profile and the measured result of each ping (for example the
received level), and train on that instead of the physics labels (`make_set()` in `train.py`).
