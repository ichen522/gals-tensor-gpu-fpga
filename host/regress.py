#!/usr/bin/env python3
"""regress.py -- the professor's sliding-window fit: split measured power
into a static part (leakage) and each SM's dynamic part.

10_6: works for any number of SMs (columns N_sm0..N_sm(T-1), or the older
N_tile0.. of 110_3, in windows.csv); --synthetic defaults to the 24-SM
build and its power-on sequences (rtl/win_ctrl.v, gals_host.default_off).
The default span grows with the number of SMs (4 x unknowns, >= 40): with
24 W's + P_static a 40-window fit has too few spare equations.

Model, one equation per 1 ms window k (9/28 meeting):

    P_k = P_static + sum_t W_t * a_{t,k}          a_{t,k} = N_{t,k} / cycles_per_window

  N_{t,k}  cycles tile t actually worked in window k (from the FPGA:
           `gals_host.py log` -> windows.csv)
  P_k      measured board/rail power in the same window (from the power
           sensor -- NOT produced by the FPGA)
  W_t      tile t's dynamic power at 100% activity (W); energy per active
           cycle = W_t / f_tile
  P_static leakage + clocks + everything that does not follow N

Sliding window: P_static and W_t are assumed constant over M consecutive
samples (M ms -- temperature barely moves in that time), so each group of
M samples gives one least-squares solution. Slide the group forward one
sample at a time and you get P_static(t) and W_t(t) as the chip heats up:
leakage should rise with temperature while W stays roughly flat. This only
works because every window has a DIFFERENT count (the 100%..80% sequence):
with constant counts the equations would all be identical.

Usage:
    python host\\regress.py --windows windows.csv --power power.csv --mhz 200
    python host\\regress.py --synthetic                 # self-test with made-up power

power.csv: columns `window,P` (P in watts, one row per 1 ms window, same
window index as windows.csv). `--lag K` shifts the power samples by K
windows if the two recordings are offset.
Writes regress_out.csv: window, P_static, W_tile0..W_tile(T-1), rms_residual.
"""
import argparse
import csv
import sys

import os

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gals_host import default_off            # noqa: E402

SEQ_LEN = 256


def read_windows(path):
    idx, N = [], []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        cols = [c for c in r.fieldnames if c.startswith("N_sm") or c.startswith("N_tile")]
        for row in r:
            idx.append(int(row["window"]))
            N.append([int(row[c]) for c in cols])
    return np.array(idx), np.array(N, dtype=float)


def read_power(path):
    d = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            d[int(row["window"])] = float(row["P"])
    return d


def sliding_fit(A, P, M):
    """A: (K, T) activities 0..1, P: (K,) watts
    -> list of (k_center, Ps, W[T], rms, se_W[T]); se = standard error of each W."""
    K, T = A.shape
    active = [t for t in range(T) if np.ptp(A[:, t]) > 1e-6]   # tiles that varied
    out = []
    for s in range(0, K - M + 1):
        X = np.column_stack([np.ones(M)] + [A[s:s + M, t] for t in active])
        y = P[s:s + M]
        if np.linalg.matrix_rank(X) < X.shape[1]:
            continue
        coef, *_ = np.linalg.lstsq(X, y, rcond=None)
        res = X @ coef - y
        rms = float(np.sqrt(np.mean(res ** 2)))
        dof = max(M - X.shape[1], 1)
        cov = (res @ res / dof) * np.linalg.inv(X.T @ X)
        W = np.full(T, np.nan)
        se = np.full(T, np.nan)
        W[active] = coef[1:]
        se[active] = np.sqrt(np.diag(cov))[1:]
        out.append((s + M // 2, coef[0], W, rms, se))
    return out


def default_span(T):
    """windows per fit. The paired sequences (x, 20 - x) give every pair of
    windows the same total, so only ~M/2 equations are independent: the span
    must be >= 2 x (T + 1); 4 x (T + 1) leaves room for the noise."""
    return max(40, 4 * (T + 1))


def synthetic(mhz=200.0, K=3000, M=None, seed=1, tiles=24):
    """Fake experiment: every SM on its power-on sequence, leakage drifting
    up with temperature, sensor noise. Checks that the fit recovers the truth."""
    M = M or default_span(tiles)
    rng = np.random.default_rng(seed)
    cyc = mhz * 1e3
    N = np.array([[cyc * (100 - default_off(t, k % SEQ_LEN)) / 100 for t in range(tiles)]
                  for k in range(K)])
    A = N / cyc
    X = np.column_stack([np.ones(M)] + [A[:M, t] for t in range(tiles)])
    rank = np.linalg.matrix_rank(X)
    W_true = 0.28 + 0.12 * rng.random(tiles)                 # W per 64-MAC SM at 100% (200 MHz)
    Ps_true = 6.0 + 1.5 * (1 - np.exp(-np.arange(K) / 1500))  # leakage warming up
    P = Ps_true + A @ W_true + rng.normal(0, 0.01, K)         # 10 mW sensor noise
    fits = sliding_fit(A, P, M)
    if not fits:
        print(f"synthetic: no solvable fit with span {M} (rank {rank}, need {tiles + 1}). With paired "
              f"sequences (x, 20-x) the span must be >= 2 x (SMs + 1) = {2 * (tiles + 1)}.")
        return False
    Ws = np.array([f[2] for f in fits])
    SEs = np.array([f[4] for f in fits])
    bias = np.abs(Ws.mean(axis=0) - W_true)
    z = np.abs(Ws - W_true) / SEs
    errPs = np.array([ps - Ps_true[k] for k, ps, *_ in fits])
    print(f"synthetic: {tiles} SMs on their power-on sequences (pairs (x, 20-x), 256 windows),")
    print(f"           rank of [1, a_0 .. a_{tiles - 1}] over {M} windows = {rank} (need {tiles + 1}),")
    print(f"           {len(fits)} sliding fits, 10 mW sensor noise, leakage {Ps_true[0]:.2f} -> {Ps_true[-1]:.2f} W")
    print(f"  true W              = {np.round(W_true, 3)}")
    print(f"  average of all fits = {np.round(Ws.mean(axis=0), 3)}   (max bias {bias.max() * 1000:.1f} mW)")
    print(f"  one fit's uncertainty (1 sigma) ~ {np.median(SEs) * 1000:.0f} mW per SM; "
          f"{np.mean(z < 2) * 100:.0f}% of fits within 2 sigma")
    print(f"  P_static tracking error: mean {errPs.mean() * 1000:.1f} mW, rms {errPs.std() * 1000:.1f} mW")
    return bool(rank == tiles + 1 and np.all(bias < 0.02) and np.mean(z < 2) > 0.9 and abs(errPs.mean()) < 0.05)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--windows")
    ap.add_argument("--power")
    ap.add_argument("--mhz", type=float, default=200.0, help="SM clock")
    ap.add_argument("--win-ms", type=float, default=1.0)
    ap.add_argument("--span", type=int, default=0, help="samples per sliding fit (M); 0 = max(40, 4 x (SMs + 1))")
    ap.add_argument("--lag", type=int, default=0, help="power sample k pairs with window k+lag")
    ap.add_argument("--out", default="regress_out.csv")
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--sms", "--tiles", dest="tiles", type=int, default=24, help="SMs in the --synthetic test")
    a = ap.parse_args(argv)

    if a.synthetic:
        ok = synthetic(a.mhz, M=a.span or None, tiles=a.tiles)
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1
    if not (a.windows and a.power):
        ap.error("--windows and --power are required (or use --synthetic)")

    idx, N = read_windows(a.windows)
    a.span = a.span or default_span(N.shape[1])
    pw = read_power(a.power)
    keep = [i for i, w in enumerate(idx) if (w + a.lag) in pw]
    if len(keep) < a.span:
        print(f"only {len(keep)} windows have a matching power sample (need >= {a.span})")
        return 1
    idx, N = idx[keep], N[keep]
    P = np.array([pw[w + a.lag] for w in idx])
    A = N / (a.mhz * 1e3 * a.win_ms)
    fits = sliding_fit(A, P, a.span)
    T = N.shape[1]
    if not fits:
        print(f"no solvable fit with span {a.span}: use --span >= {2 * (T + 1)} (paired sequences)")
        return 1
    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["window", "P_static"] + [f"W_sm{t}" for t in range(T)] + ["rms_residual"])
        for k, ps, W, rms, _ in fits:
            w.writerow([int(idx[k]), f"{ps:.5f}"] + [f"{x:.5f}" for x in W] + [f"{rms:.5f}"])
    Ws = np.array([f[2] for f in fits])
    SEs = np.array([f[4] for f in fits])
    print(f"{len(fits)} fits of {a.span} windows -> {a.out}")
    print(f"P_static: {fits[0][1]:.3f} W (start) -> {fits[-1][1]:.3f} W (end)")
    for t in range(T):
        col = Ws[:, t]
        if np.all(np.isnan(col)):
            print(f"SM {t:2d}: not active")
        else:
            print(f"SM {t:2d}: W = {np.nanmean(col):.3f} W at 100% (spread {np.nanstd(col):.3f}, "
                  f"one fit +-{np.nanmedian(SEs[:, t]):.3f}), "
                  f"{np.nanmean(col) / (a.mhz * 1e6) * 1e9:.2f} nJ per active cycle")
    print(f"typical residual: {np.median([f[3] for f in fits]) * 1000:.1f} mW "
          "(should be close to the sensor noise; much larger = model or alignment wrong, try --lag)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
