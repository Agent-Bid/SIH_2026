"""Check host/water_sim.py against the C++ simulation the ESP32 runs (MCU /SIH/water_sim.h):
compiles a small program with g++, steps both for 20,000 pings and compares every reading.

  python3 host/check_water_sim.py
"""
import os
import subprocess
import sys
import tempfile
from water_sim import WaterSim

HERE = os.path.dirname(os.path.abspath(__file__))
SKETCH = os.path.join(HERE, "..", "MCU ", "SIH")
N = 20000
SRC = r"""
#include <stdio.h>
#include "water_sim.h"
int main() {
  watersim::Sim s; watersim::init(s);
  for (int i = 0; i < %d; i++) {
    watersim::Reading r = watersim::step(s, 1.0f / PING_RATE_HZ);
    printf("%%.4f %%.4f %%.3f %%.4f %%.4f %%.3f %%.4f %%d\n", r.tempC, r.salinityPpt, r.depthM, r.turbidityNtu,
           r.batteryPct, r.rangeM, r.speedMs, r.contact);
  }
}
""" % N

with tempfile.TemporaryDirectory() as tmp:
    with open(os.path.join(tmp, "sim.cpp"), "w") as f:
        f.write(SRC)
    subprocess.run(["g++", "-O1", "-I", SKETCH, "-o", os.path.join(tmp, "sim"), os.path.join(tmp, "sim.cpp")],
                   check=True)
    lines = subprocess.run([os.path.join(tmp, "sim")], capture_output=True, text=True, check=True).stdout.split("\n")

sim = WaterSim()
keys = ["T", "S", "D", "turb", "bat", "R", "v"]
tol = dict(T=0.01, S=0.01, D=0.1, turb=0.05, bat=0.01, R=0.5, v=0.01)     # float (C++) vs double (Python)
worst = {k: 0.0 for k in keys}
bad = 0
for i in range(N):
    c = lines[i].split()
    r = sim.step()
    for k, v in zip(keys, c):
        worst[k] = max(worst[k], abs(float(v) - r[k]))
    if int(c[7]) != "ABC".index(r["contact"]) or any(abs(float(v) - r[k]) > tol[k] for k, v in zip(keys, c)):
        bad += 1
print(f"{N} pings, largest differences: " + ", ".join(f"{k} {worst[k]:.2g}" for k in keys))
if bad == 0:
    print("PASS: the Python twin gives the ESP32's conditions")
else:
    print(f"FAIL: {bad} pings differ")
    sys.exit(1)
