"""Python twin of the ESP32's simulated water conditions (MCU /SIH/water_sim.h).

The simulation is deterministic from its seed and takes one step per decision, and the ESP32
numbers its decisions (the packet's seq) from 0 at power-on. So the conditions behind any
packet can be worked out here from its seq alone, without the ESP32's serial log.
Check against the C++: python3 host/check_water_sim.py
"""
import numpy as np

F = np.float32                       # the ESP32 computes in float: so does this, step for step
SEED = 12345
SPEEDUP = F(10.0)                    # SIM_SPEEDUP
PING_RATE_HZ = 6
LIMITS = dict(T=(2.0, 30.0), S=(25.0, 40.0), D=(0.0, 1000.0), turb=(0.0, 100.0), R=(10.0, 2000.0), v=(0.0, 10.0))
CONTACTS = "ABC"


def clamp(x, lim):
    return F(min(max(x, F(lim[0])), F(lim[1])))


class WaterSim:
    def __init__(self, seed=SEED):
        self.rng = seed or 1
        self.t, self.battery, self.plume, self.next = F(0), F(100), F(5), 0
        # range, radial speed, min, max range, speed sigma, speed time constant
        self.c = [[F(v) for v in c] for c in ((60.0, 1.5, 15.0, 180.0, 2.0, 20.0),
                                              (400.0, 0.02, 250.0, 600.0, 0.05, 60.0),
                                              (800.0, -3.0, 450.0, 1000.0, 2.5, 40.0))]

    def uniform01(self):
        r = self.rng
        r ^= (r << 13) & 0xFFFFFFFF
        r ^= r >> 17
        r ^= (r << 5) & 0xFFFFFFFF
        self.rng = r
        return F(r >> 8) * F(1.0 / 16777216.0)

    def gauss(self):
        u = self.uniform01() + F(1e-7)
        v = self.uniform01()
        return np.sqrt(F(-2) * np.log(u)) * np.cos(F(6.2831853) * v)

    def step(self, dt_real=F(1) / F(PING_RATE_HZ)):
        dt = F(dt_real) * SPEEDUP
        self.t += dt
        self.battery -= F(0.005) * dt
        if self.battery < 15:
            self.battery = F(100)
        a = np.exp(-dt / F(120))
        self.plume = F(5) + (self.plume - F(5)) * a + F(12) * np.sqrt(F(1) - a * a) * self.gauss()
        self.plume = clamp(self.plume, (0.0, 60.0))
        for c in self.c:
            b = np.exp(-dt / c[5])
            c[1] = c[1] * b + c[4] * np.sqrt(F(1) - b * b) * self.gauss()
            c[0] += c[1] * dt
            if c[0] < c[2]:
                c[0], c[1] = F(2) * c[2] - c[0], abs(c[1])
            if c[0] > c[3]:
                c[0], c[1] = F(2) * c[3] - c[0], -abs(c[1])
        period = F(1200)
        x = np.fmod(self.t, period) / period
        d = F(20) + F(330) * (F(2) * x if x < 0.5 else F(2) - F(2) * x)
        nepheloid = F(40) * (d - F(300)) / F(50) if d > 300 else F(0)
        r = dict(D=clamp(d + F(0.5) * self.gauss(), LIMITS["D"]),
                 T=clamp(F(5) + F(17) / (F(1) + np.exp((d - F(80)) / F(20))) + F(0.05) * self.gauss(), LIMITS["T"]),
                 S=clamp(F(34.8) + F(0.4) * np.tanh((d - F(60)) / F(40)) + F(0.02) * self.gauss(), LIMITS["S"]),
                 turb=clamp(F(3) + self.plume + nepheloid + F(0.5) * self.gauss(), LIMITS["turb"]),
                 bat=self.battery, contact=CONTACTS[self.next], t=self.t)
        c = self.c[self.next]
        r["R"] = clamp(c[0], LIMITS["R"])
        r["v"] = clamp(abs(c[1]), LIMITS["v"])
        self.next = (self.next + 1) % 3
        r = {k: (float(v) if k != "contact" else v) for k, v in r.items()}
        return r


def conditions(seqs):
    """{seq: conditions} for packet numbers counted from power-on (seq k = step k + 1)."""
    want = sorted(set(seqs))
    sim, out = WaterSim(), {}
    for k in range(want[-1] + 1 if want else 0):
        r = sim.step()
        if k in want:
            out[k] = r
    return out
