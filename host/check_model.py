#!/usr/bin/env python3
"""check_model.py -- compare host/tile_model.py with the RTL simulation.

tb/tb_sm.v dumps the 64 results of the first two RUN_ONE blocks of the
power-on BRAM contents (K = 256) of one SM to tb_out/tile_poweron.txt. This script
recomputes them with tile_model.py from the seeds alone. A match proves the
Python model (used by host/verify.py on the board) and the RTL agree bit
for bit, including the power-on BRAM initialisation.

    python host/check_model.py [tb_out/tile_poweron.txt]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tile_model import TileModel, power_on_A, power_on_B, self_test, NRES, COLS   # noqa: E402


def main(path="tb_out/tile_poweron.txt"):
    seeds, got = None, {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if p[0] == "seed_a":
                seeds = (int(p[1]), int(p[3]))
            elif p[0] == "res":
                got[(int(p[1]), int(p[2]))] = int(p[3])
    m = TileModel(A=power_on_A(seeds[0]), B=power_on_B(seeds[1]))
    bad = 0
    for b in (0, 1):
        exp = m.results(b, 256)
        for r in range(NRES):
            if got[(b, r)] != exp[r]:
                if bad < 5:
                    print(f"MISMATCH block {b} C[{r // COLS}][{r % COLS}]: rtl {got[(b, r)]} model {exp[r]}")
                bad += 1
    self_test()
    print(f"check_model: {len(got)} RTL results vs tile_model.py -> "
          + ("PASS (bit-exact)" if bad == 0 else f"FAIL ({bad} mismatches)"))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
