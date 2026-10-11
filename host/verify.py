#!/usr/bin/env python3
"""verify.py -- check every SM of the 10_6 GPU bitstream on the VC707 against
the independent model host/tile_model.py.

Before running: press CPU_RESET and set ALL DIP switches OFF.

    python host\\verify.py --port COM9                    every SM, power-on data
    python host\\verify.py --port COM9 --sms 0,13 --load 123
    python host\\verify.py --port COM9 --sms gpc:2        the 6 SMs of GPC 2
    python host\\verify.py --port COM9 --freq             + per-SM clock sweep (MMCM and PLL SMs)
    python host\\verify.py --port COM9 --together         + all selected SMs at once
    python host\\verify.py --port COM9 --sms 0,1 --gals   + GALS: change one SM's clock, the other must not notice

Per SM (one at a time unless --together):
  [one]    TILE_CFG K, RUN_ONE, READ_RES: all 64 values of C = A x B must
           equal the model (8 x 8 x K multiply-adds, 48-bit exact), and
           READ_STAT must give blk_done = 1 with the matching ReLU checksum.
  [cont]   continuous run (SET_EN) for --seconds; READ_STAT -> the checksum
           of exactly blk_done blocks must equal the model -- this covers
           millions of blocks at full speed.
  [window] 1 ms window engine with the power-on sequence of that SM:
           N = f_sm / 1000 * (100 - OFF) / 100 (+-3) in every window, with
           f_sm from GET_FREQ -> proves the SM clock (MMCM or PLL) and the
           enable FSM.
  [freq]   (--freq) the window check again at D = 12, 8, 6 (100 / 150 /
           200 MHz), then back to the boot divider.
  [gals]   (--gals, first two selected SMs, each way round) both SMs run the
           window engine; the victim's MMCM / PLL is re-programmed at the
           same D, then D 12, then back. Every bystander window must still be
           exact (+-3), the victim must show old | stop | old | new | old, and
           the same-D step measures how long one SET_FREQ stops an SM (us).
  --load SEED writes random A / B for that SM first (512 x WRITE_A + 512 x
  WRITE_B, a few seconds) and checks against those.

Needs pyserial and numpy.
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gals_host import Board, default_off, parse_mask, sm_label   # noqa: E402
from tile_model import TileModel, DEPTH, NROW, COLS, NRES         # noqa: E402

SEQ_LEN = 256                     # power-on sequences (rtl/win_ctrl.v)
CLK_MEM_HZ = 200_000_000


def load_tile(b, t, seed):
    rng = np.random.default_rng(seed * 64 + t)
    A = rng.integers(-32768, 32768, (NROW, DEPTH))
    B = rng.integers(-32768, 32768, (DEPTH, COLS))
    t0 = time.time()
    for a in range(DEPTH):
        b.write_a(t, a, A[:, a])
        b.write_b(t, a, B[a, :])
        if a % 64 == 63:
            print(f"\r    wrote {a + 1}/{DEPTH} addresses", end="", flush=True)
    print(f"\r    wrote A and B ({DEPTH} addresses) in {time.time() - t0:.1f} s")
    return TileModel(t, A=A, B=B)


def one_test(b, t, m, K):
    b.tile_cfg(1 << t, K)
    b.run_one(t)
    got = b.read_res(t)
    exp = m.results(0, K)
    bad = [i for i in range(NRES) if got[i] != exp[i]]
    act, blk, chk = b.read_stat(t)
    ok = not bad and blk == 1 and chk == m.checksum(1, K)
    print(f"    C[0][0..3] board {got[:4]}")
    print(f"               model {exp[:4]}   -> {NRES - len(bad)}/{NRES} exact, blk_done {blk}, "
          f"chk {'match' if chk == m.checksum(1, K) else 'MISMATCH'}")
    return ok


def cont_tests(b, tiles, models, K, seconds):
    mask = sum(1 << t for t in tiles)
    b.tile_cfg(mask, K)
    b.set_en(mask)
    time.sleep(seconds)
    b.set_en(0)
    time.sleep(0.02)
    res = {}
    for t in tiles:
        act, blk, chk = b.read_stat(t)
        exp = models[t].checksum(blk, K)
        ok = blk > 0 and chk == exp and act >= blk * K
        print(f"    {sm_label(t)}: {act:,} active cycles, {blk:,} blocks x {NRES} x {K} MACs, "
              f"chk 0x{chk:016x} -> {'MATCH' if chk == exp else 'MISMATCH (model 0x%016x)' % exp}")
        res[t] = ok
    return res


def restore_sequences(b, tiles):
    """write the power-on sequence back (only needed after `seq --random`)"""
    for t in tiles:
        for i in range(SEQ_LEN):
            b.seq_write(t, i, default_off(t, i))


def window_test(b, t, label=""):
    mhz = b.tile_mhz(t)
    per_win = int(round(mhz * 1e6 / 1000))
    b.set_en(1 << t)
    b.seq_cfg(True, clear=True, seq_len=SEQ_LEN, win_cycles=CLK_MEM_HZ // 1000)
    recs, t_end = [], time.time() + 0.3
    while time.time() < t_end or len(recs) < 50:
        _, _, r = b.read_windows(255)
        recs += r
        if not r:
            time.sleep(0.02)
        if time.time() > t_end + 2:
            break
    b.seq_cfg(False)
    b.set_en(0)
    if not recs:
        print("    !! no windows recorded")
        return False
    worst, other, est = 0, 0, []
    for idx, nk in recs:
        off = default_off(t, idx % SEQ_LEN)
        worst = max(worst, abs(nk[t] - per_win * (100 - off) // 100))
        est.append(nk[t] / ((100 - off) / 100) * 1000 / 1e6)
        other += sum(1 for u in range(len(nk)) if u != t and nk[u] != 0)
    print(f"    {label}{len(recs)} windows at {mhz:.1f} MHz: worst |N - expected| = {worst}, "
          f"measured {np.mean(est):.3f} MHz" + (f"  !! {other} counts from other SMs (DIP on?)" if other else ""))
    return worst <= 3 and other == 0


# ---- GALS check: one SM's clock changes, its neighbour must not notice ----------------
def _collect(b, seconds, recs):
    t_end = time.time() + seconds
    ovf = False
    while time.time() < t_end:
        _, o, r = b.read_windows(255)
        ovf |= o
        recs += r
        if not r:
            time.sleep(0.01)
    return ovf


def analyse_gals(recs, victim, bystander, pw_a, pw_b, pw_u, tol=3):
    """recs: [(window, [N_sm...])]. pw_* = SM cycles per 1 ms window at the victim's
    boot clock (a), the victim's low clock (b) and the bystander's clock (u).
    Returns (ok, report lines, stop_us)."""
    lines, worst_u, other = [], 0, 0
    runs = []                                  # [label, count, deficit cycles, duty sum]
    for idx, nk in recs:
        off_u = default_off(bystander, idx % SEQ_LEN)
        worst_u = max(worst_u, abs(nk[bystander] - pw_u * (100 - off_u) // 100))
        other += sum(1 for s in range(len(nk)) if s not in (victim, bystander) and nk[s] != 0)
        off_v = default_off(victim, idx % SEQ_LEN)
        ea, eb, n = pw_a * (100 - off_v) // 100, pw_b * (100 - off_v) // 100, nk[victim]
        lab = "A" if abs(n - ea) <= tol else "B" if abs(n - eb) <= tol else "X"
        if runs and runs[-1][0] == lab:
            runs[-1][1] += 1
        else:
            runs.append([lab, 1, 0, 0.0])
        if lab == "X":
            runs[-1][2] += ea - n
            runs[-1][3] += (100 - off_v) / 100
    gaps = [i for i in range(1, len(recs)) if recs[i][0] != recs[i - 1][0] + 1]

    by_ok = worst_u <= tol and other == 0
    lines.append(f"{len(recs)} windows; bystander SM {bystander}: worst |N - expected| = {worst_u}"
                 + ("  -> never lost a cycle" if by_ok else "  !! DISTURBED")
                 + (f"  !! {other} counts from other SMs (DIP on?)" if other else ""))
    mhz = {"A": pw_a / 1000, "B": pw_b / 1000}
    lines.append(f"victim SM {victim}: " + " | ".join(
        f"{mhz[r[0]]:.1f} MHz x {r[1]}" if r[0] != "X" else f"{r[1]} window(s) off" for r in runs))
    if gaps:
        lines.append(f"!! {len(gaps)} gap(s) in the window index (board FIFO overflowed?)")

    shape = [r[0] for r in runs]
    want = ["A", "X", "A", "X", "B", "X", "A"]
    stop_us = None
    if shape == want:
        x = runs[1]                                     # re-program at the same divider
        stop_us = x[2] / (pw_a / 1000) / (x[3] / x[1])  # missing cycles / (cycles per us x duty)
        lines.append(f"re-programming SM {victim} at the same D stopped its clock for about "
                     f"{stop_us:.0f} us ({x[2]:,} cycles missing); "
                     + ("its neighbour kept running" if by_ok else "!! its neighbour was disturbed too"))
    else:
        lines.append(f"!! expected the pattern {' '.join(want)}, got {' '.join(shape)}")
    ok = by_ok and stop_us is not None and 0 < stop_us < 1000 and not gaps
    return ok, lines, stop_us


def gals_test(b, victim, bystander, seconds=0.3):
    """The professor's GALS requirement on real silicon. Both SMs run the window
    engine with their power-on sequences; meanwhile the victim's MMCM / PLL is
    re-programmed three times (same D, D 12, back). Every one of the bystander's
    1 ms windows must still match its own clock exactly (+-3 cycles), and the
    victim must show: old clock | short stop | old clock | ... | new clock | ... | old."""
    vco = b.info()["vco_mhz"]
    d_v, _ = b.get_freq(victim)
    d_u, _ = b.get_freq(bystander)
    d_low = 12 if d_v != 12 else 8
    pw = lambda d: int(round(vco * 1e6 / d / 1000))      # noqa: E731  cycles per 1 ms
    print(f"  [gals] re-program {sm_label(victim)} while {sm_label(bystander)} runs")
    b.set_en((1 << victim) | (1 << bystander))
    b.seq_cfg(True, clear=True, seq_len=SEQ_LEN, win_cycles=CLK_MEM_HZ // 1000)
    recs, acks = [], []
    ovf = _collect(b, seconds, recs)
    for d in (d_v, d_low, d_v):
        acks.append(b.set_freq(victim, d))
        ovf |= _collect(b, seconds, recs)
    b.seq_cfg(False)
    for _ in range(5):                       # drain what is still in the FIFO
        _, o, r = b.read_windows(255)
        recs += r
        if not r:
            break
    b.set_en(0)
    ok, lines, _ = analyse_gals(recs, victim, bystander, pw(d_v), pw(d_low), pw(d_u))
    for ln in lines:
        print("    " + ln)
    if not all(acks):
        print(f"    !! SET_FREQ refused: {acks}")
    if ovf:
        print("    !! board FIFO overflowed")
    print(f"    -> {'PASS' if ok and all(acks) else 'FAIL'}")
    return ok and all(acks)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--sms", "--tiles", dest="tiles", default=None, help="e.g. 0,1,13 or gpc:0,2 (default: all)")
    ap.add_argument("--k", type=int, default=256)
    ap.add_argument("--load", type=int, default=None, metavar="SEED")
    ap.add_argument("--seconds", type=float, default=0.2, help="continuous run per SM")
    ap.add_argument("--freq", action="store_true", help="also check D = 12, 8, 6")
    ap.add_argument("--together", action="store_true", help="also run all selected SMs at once")
    ap.add_argument("--gals", action="store_true",
                    help="re-program one SM's clock while its neighbour runs (first two selected SMs, both ways)")
    ap.add_argument("--restore-seq", action="store_true",
                    help="first write the power-on duty sequences back (after `gals_host.py seq --random`)")
    a = ap.parse_args(argv)

    b = Board(a.port)
    v = b.ping()
    info = b.info()
    nt = info["num_sm"]
    print(f"link OK: GALS build 0x{v:02X}, {nt} SMs ({nt // info['sm_per_gpc']} GPCs), "
          f"{info['n_sub']} sub-partitions x {info['rows']} x {info['cols']} MACs per SM")
    if a.tiles is None:
        tiles = list(range(nt))
    elif a.tiles.lower().startswith("gpc:") or a.tiles.lower() == "all":
        m = parse_mask(a.tiles, nt)
        tiles = [t for t in range(nt) if m >> t & 1]
    else:
        tiles = [int(x) for x in a.tiles.split(",")]
    b.seq_cfg(False)
    b.set_en(0)
    if a.restore_seq:
        restore_sequences(b, tiles)

    models, results = {}, {}
    for t in tiles:
        d, st = b.get_freq(t)
        print(f"\n=== {sm_label(t)}  (D {d} = {info['vco_mhz'] / d:.1f} MHz, "
              f"{'locked' if st['locked'] else 'NOT LOCKED'}) ===")
        if a.load is not None:
            print(f"  [load] seed {a.load}")
            models[t] = load_tile(b, t, a.load)
        else:
            models[t] = TileModel(t)
        print(f"  [one] RUN_ONE, K = {a.k}")
        ok = one_test(b, t, models[t], a.k)
        print(f"  [cont] {a.seconds} s continuous")
        ok &= cont_tests(b, [t], models, a.k, a.seconds)[t]
        print("  [window] 1 ms windows")
        ok &= window_test(b, t)
        if a.freq:
            boot_d = d
            for dd in (12, 8, 6):
                if not b.set_freq(t, dd):
                    print(f"    !! SET_FREQ D {dd} refused"); ok = False; continue
                ok &= window_test(b, t, label=f"D {dd}: ")
            b.set_freq(t, boot_d)
        results[t] = ok
        print(f"  SM {t}: {'PASS' if ok else 'FAIL'}")

    if a.gals:
        if len(tiles) < 2:
            print("\n--gals needs two SMs (e.g. --sms 0,1)")
        else:
            u, v = tiles[0], tiles[1]
            print(f"\n=== GALS: SM {v} and SM {u} ===")
            for vic, by in ((v, u), (u, v)):
                g = gals_test(b, vic, by)
                results[vic] &= g
                results[by] &= g

    if a.together and len(tiles) > 1:
        print(f"\n=== SMs {tiles} at once, {a.seconds} s ===")
        res = cont_tests(b, tiles, models, a.k, a.seconds)
        for t in tiles:
            results[t] &= res[t]

    print("\nsummary: " + "  ".join(f"SM {t} {'PASS' if ok else 'FAIL'}" for t, ok in results.items()))
    if a.load is not None:
        print(f"note: the written rows stay in the BRAMs until the FPGA is re-programmed "
              f"(run later checks with --load {a.load} again).")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
