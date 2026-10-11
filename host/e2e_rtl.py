#!/usr/bin/env python3
"""e2e_rtl.py -- end-to-end check: host/gals_host.py's byte encoders and
decoders + the RTL (simulated with iverilog) + host/tile_model.py.

  1. builds a command script with gals_host.Proto (the exact bytes Board
     sends on the real board) -> tb_out/e2e_cmds.txt
  2. runs tb/tb_e2e.v (vc707_top, 3 SMs: SM 0 / 2 on MMCMs, SM 1 on a
     PLL, sim/sim_prims.v) which plays the bytes into the UART pin and
     records every reply
  3. decodes the replies with gals_host.Proto and checks them against
     tile_model.py: READ_RES of a RUN_ONE block, checksums after a
     continuous run, frequency read-back (MMCM and PLL SMs), INFO, PING,
     READ_WIN records (3-byte N_k) against the power-on sequences.

    python host/e2e_rtl.py            (from the 10_6 folder; needs iverilog)
"""
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from gals_host import Proto, default_off      # noqa: E402
from tile_model import TileModel, NROW, COLS   # noqa: E402

K = 64
NT = 3
MBT = (NT + 7) // 8           # mask bytes of the 3-SM test build


def main():
    os.chdir(ROOT)
    os.makedirs("tb_out", exist_ok=True)
    rng = random.Random(7)
    models = [TileModel(t) for t in range(NT)]
    steps = []                         # (name, encoded or None, extra)

    def T(name, enc):
        steps.append((name, enc))

    T("ping", Proto.ping())
    T("info", Proto.info())
    for t in range(NT):
        T(f"get_freq boot {t}", Proto.get_freq(t))
    T("tile_cfg", Proto.tile_cfg(0x7, K, MBT))
    col = [rng.randint(-32768, 32767) for _ in range(NROW)]
    row = [rng.randint(-32768, 32767) for _ in range(COLS)]
    col[0], row[0] = -32768, -32768
    T("write_a", Proto.write_a(1, 3, col))
    T("write_b", Proto.write_b("all", 3, row))
    models[1].write_a(3, col)
    for m in models:
        m.write_b(3, row)
    T("run_one 1", Proto.run_one(1))
    T("read_res 1", Proto.read_res(1))
    T("read_stat 1", Proto.read_stat(1))
    T("set_en 2", Proto.set_en(0b100, MBT))
    steps.append(("wait", 40000))
    T("set_en 0", Proto.set_en(0, MBT))
    steps.append(("wait", 3000))
    T("read_stat 2", Proto.read_stat(2))
    T("read_act 2", Proto.read_act(2))
    T("read_chk 2", Proto.read_chk(2))
    T("set_freq 0 24", Proto.set_freq(0, 24))
    T("get_freq 0", Proto.get_freq(0))
    T("set_freq 1 8", Proto.set_freq(1, 8))          # SM 1 runs on a PLL
    T("get_freq 1", Proto.get_freq(1))
    T("set_freq 0 2", Proto.set_freq(0, 2))
    # window engine: power-on sequences, short windows, all 3 SMs on
    WIN = 4000                                        # clk_mem cycles = 20 us
    T("set_en all", Proto.set_en(0b111, MBT))
    T("seq_cfg run", Proto.seq_cfg(True, True, 0, WIN))
    steps.append(("wait", 140000))
    T("seq_cfg stop", Proto.seq_cfg(False))
    T("set_en off", Proto.set_en(0, MBT))
    T("read_win", (bytes([0x08, 4]), 2 + 4 * Proto.win_record_bytes(NT)))

    with open("tb_out/e2e_cmds.txt", "w") as f:
        for name, enc in steps:
            if name == "wait":
                f.write(f"W {enc}\n")
            else:
                cmd, n = enc
                f.write(f"T {len(cmd)} " + " ".join(f"{x:02x}" for x in cmd) + f" {n}\n")

    build = "/tmp/claude-0/b" if os.path.isdir("/tmp/claude-0") else "tb_out"
    os.makedirs(build, exist_ok=True)
    vvp = os.path.join(build, "e2e.vvp")
    srcs = ["sim/sim_prims.v"] + sorted(os.path.join("rtl", x) for x in os.listdir("rtl") if x.endswith(".v")) + ["tb/tb_e2e.v"]
    subprocess.run(["iverilog", "-g2012", "-o", vvp, "-s", "tb_e2e"] + srcs, check=True)
    out = subprocess.run(["vvp", "-n", vvp], capture_output=True, text=True, check=True).stdout
    print(out.strip().splitlines()[-1])

    replies = []
    with open("tb_out/e2e_replies.txt") as f:
        for line in f:
            p = line.split()
            replies.append(bytes(int(x, 16) for x in p[2:2 + int(p[1])]))
    named = [s for s in steps if s[0] != "wait"]
    assert len(replies) == len(named), f"{len(replies)} replies for {len(named)} commands"
    assert len({n for n, _ in named}) == len(named), "command names must be unique"
    R = {name: r for (name, _), r in zip(named, replies)}

    errors = []

    def check(cond, msg):
        print(("OK   " if cond else "FAIL ") + msg)
        if not cond:
            errors.append(msg)

    check(Proto.d_ping(R["ping"]) == 0x90, "PING -> build 0x90")
    info = Proto.d_info(R["info"])
    check(info == dict(num_sm=NT, n_sub=4, rows=2, cols=8, addr_bits=9, vco_mhz=1200,
                       div_min=3, div_max=126, k_min=40, sm_per_gpc=6, nk_bytes=3), f"INFO {info}")
    for t in range(NT):
        d, st = Proto.d_freq(R[f"get_freq boot {t}"])
        check(d == 12 and st["locked"] and st["running"] and st["boot_done"],
              f"GET_FREQ SM {t} ({'PLL' if t % 2 else 'MMCM'}): D {d} {st}")
    for n in ("tile_cfg", "write_a", "write_b", "run_one 1", "set_en 2", "set_en 0", "set_freq 0 24",
              "set_freq 1 8", "seq_cfg run", "set_en all", "seq_cfg stop", "set_en off"):
        check(R[n] == bytes([0xA0]), f"{n} -> {R[n].hex()}")
    res = Proto.d_res(R["read_res 1"])
    exp = models[1].results(0, K)
    check(res == exp, f"READ_RES SM 1 == tile_model block 0 ({len(exp)} values, incl. the written A column / B row)")
    act, blk, chk = Proto.d_stat(R["read_stat 1"])
    check(blk == 1 and chk == models[1].checksum(1, K), f"READ_STAT SM 1: blk {blk}, chk 0x{chk:016x}")
    act, blk, chk = Proto.d_stat(R["read_stat 2"])
    check(blk > 10 and chk == models[2].checksum(blk, K) and act >= blk * K,
          f"READ_STAT SM 2 after a continuous run: act {act}, blk {blk}, chk 0x{chk:016x} == model")
    check(Proto.d_u32(R["read_act 2"]) == act, "READ_ACT SM 2 == activity in READ_STAT")
    check(Proto.d_u64(R["read_chk 2"]) == chk, "READ_CHK SM 2 == chk in READ_STAT")
    d, st = Proto.d_freq(R["get_freq 0"])
    check(d == 24, f"after SET_FREQ SM 0 (MMCM) -> D 24: GET_FREQ says D {d}")
    d, st = Proto.d_freq(R["get_freq 1"])
    check(d == 8 and st["locked"] and st["running"], f"after SET_FREQ SM 1 (PLL) -> D 8: GET_FREQ says D {d} {st}")
    check(R["set_freq 0 2"] == bytes([0xE1]), "SET_FREQ D 2 (< DIV_MIN) refused with E1")
    # READ_WIN: N_k of each SM = f_sm * window * (100 - OFF) / 100, OFF = power-on sequence
    rw = R["read_win"]
    st, cnt = rw[0], rw[1]
    recs = Proto.d_win_records(rw[2:], cnt, NT)
    ok = cnt >= 3
    worst = 0
    for idx, nk in recs:
        for t, mhz in ((0, 1200 / 24), (1, 1200 / 8), (2, 1200 / 12)):
            per = mhz * 1e6 * WIN / 200e6
            exp_n = per * (100 - default_off(t, idx)) / 100
            worst = max(worst, abs(nk[t] - exp_n))
    ok &= worst <= 3
    check(ok, f"READ_WIN: {cnt} records, N_k of SM 0/1/2 (50 / 150 / 100 MHz) vs the power-on "
              f"sequences: worst |N - expected| = {worst:.1f}  (first: {recs[0] if recs else None})")

    print("PASS: e2e_rtl (gals_host.py <-> RTL <-> tile_model.py)" if not errors
          else f"FAIL: e2e_rtl, {len(errors)} error(s)")
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
