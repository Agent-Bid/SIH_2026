"""Make sine.hex: one full sine cycle as 1024 signed 12-bit numbers.

Entry i holds round(2047 * sin(2*pi*i/1024)), stored as 3 hex digits in two's
complement (e.g. -1 -> FFF). Verilog loads it with $readmemh("sine.hex", ...).

Run from the project root:   python3 host/gen_sine.py
"""
import numpy as np

ENTRIES = 1024      # table size -> address is the top 10 bits of phase
AMPL = 2047         # largest 12-bit signed value

def sine_table():
    i = np.arange(ENTRIES)
    return np.round(AMPL * np.sin(2 * np.pi * i / ENTRIES)).astype(int)

if __name__ == "__main__":
    with open("sine.hex", "w") as f:
        for v in sine_table():
            f.write(f"{v & 0xFFF:03X}\n")
    print(f"wrote sine.hex ({ENTRIES} entries, +/-{AMPL})")
