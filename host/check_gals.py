#!/usr/bin/env python3
"""check_gals.py -- self-test of verify.py --gals without a board.

A fake board produces 1 ms window records in real time from a clock model:
every SM counts f x duty cycles per window, and SET_FREQ stops the selected
SM's clock for STOP_US, then restarts it at the new divider.

  * GALS board (10_6): only the re-programmed SM stops    -> verify must PASS
    and report a stop close to STOP_US
  * shared-MMCM board (v7.3 style): every SM stops         -> verify must FAIL

    python host/check_gals.py
"""
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import verify                                  # noqa: E402
from gals_host import default_off              # noqa: E402

STOP_US = 70.0
NUM_SM = 24
VCO = 1200


class FakeBoard:
    def __init__(self, shared=False, seed=1):
        self.shared = shared
        self.rng = random.Random(seed)
        self.div = [6] * NUM_SM
        self.seg = [[(0.0, VCO / 6)] for _ in range(NUM_SM)]   # (start ms, MHz; 0 = stopped)
        self.en = 0
        self.t0 = None
        self.next_win = 0

    def _now(self):
        return (time.time() - self.t0) * 1000.0 if self.t0 is not None else 0.0

    def info(self):
        return {"vco_mhz": VCO, "num_sm": NUM_SM}

    def get_freq(self, t):
        return self.div[t], {"locked": True}

    def set_en(self, mask):
        self.en = mask

    def seq_cfg(self, run, clear=False, seq_len=0, win_cycles=0):
        if run:
            self.t0 = time.time()
            self.next_win = 0
            self.seg = [[(0.0, VCO / d)] for d in self.div]
        else:
            self.stop_at = self._now()

    def set_freq(self, t, d):
        now = self._now()
        start = now + self.rng.uniform(0.2, 1.5)               # USB + command latency
        for s in (range(NUM_SM) if self.shared else [t]):
            new = VCO / d if s == t else VCO / self.div[s]
            self.seg[s] += [(start, 0.0), (start + STOP_US / 1000, new)]
        self.div[t] = d
        time.sleep(0.002)
        return True

    def _cycles(self, s, k):
        """cycles SM s ran in window k = [k, k+1) ms (before the duty pattern)"""
        tot, seg = 0.0, self.seg[s]
        for i, (a, f) in enumerate(seg):
            b = seg[i + 1][0] if i + 1 < len(seg) else float("inf")
            lo, hi = max(a, k), min(b, k + 1)
            if hi > lo:
                tot += f * 1000 * (hi - lo)
        return tot

    def read_windows(self, max_records=255):
        end = self._now() if not hasattr(self, "stop_at") or self.stop_at is None else self.stop_at
        recs = []
        while self.next_win + 1 <= end and len(recs) < max_records:
            k = self.next_win
            nk = []
            for s in range(NUM_SM):
                if self.en >> s & 1:
                    duty = (100 - default_off(s, k % verify.SEQ_LEN)) / 100
                    nk.append(int(self._cycles(s, k) * duty))
                else:
                    nk.append(0)
            recs.append((k, nk))
            self.next_win += 1
        return True, False, recs


def run(shared):
    b = FakeBoard(shared=shared)
    b.stop_at = None
    return verify.gals_test(b, 1, 0, seconds=0.3)


def main():
    print("--- GALS board (each SM has its own MMCM / PLL) ---")
    ok_gals = run(False)
    print("--- shared-MMCM board (every output stops together) ---")
    ok_shared = run(True)
    if ok_gals and not ok_shared:
        print("PASS: check_gals (GALS board passes, shared-clock board is caught)")
        return 0
    print(f"FAIL: check_gals (GALS board {'PASS' if ok_gals else 'FAIL'}, "
          f"shared board {'PASS' if ok_shared else 'FAIL'})")
    return 1


if __name__ == "__main__":
    sys.exit(main())
