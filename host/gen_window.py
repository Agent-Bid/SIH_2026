"""Make hann.hex: a Hann window (a smooth fade-in / fade-out) as 1024 numbers.

Entry i holds round(4096 * sin^2(pi*i/1024)): 0 at the edges, 4096 (= 1.0) in the
middle. Stored as 4 hex digits. Verilog loads it with $readmemh("hann.hex", ...).

Run from the project root:   python3 host/gen_window.py
"""
import numpy as np

ENTRIES = 1024      # table size -> address is the top 10 bits of the window phase
ONE = 4096          # the number that means "1.0" (multiply by 4096, then shift right 12)

def hann_table():
    i = np.arange(ENTRIES)
    return np.round(ONE * np.sin(np.pi * i / ENTRIES) ** 2).astype(int)

if __name__ == "__main__":
    with open("hann.hex", "w") as f:
        for v in hann_table():
            f.write(f"{v:04X}\n")
    print(f"wrote hann.hex ({ENTRIES} entries, 0..{ONE})")
