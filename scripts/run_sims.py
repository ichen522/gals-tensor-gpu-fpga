#!/usr/bin/env python3
"""run_sims.py -- run every 10_6 simulation (needs Icarus Verilog 12 in PATH).

    python scripts/run_sims.py            all (about 15 minutes)
    python scripts/run_sims.py sm clk     just these

Tests (tb/):
  sm         tb_sm.v         one SM vs a GEMM reference (RUN_ONE, writes,
                             continuous with random enable, duty FSM, clear)
             + host/check_model.py (RTL results == host/tile_model.py)
  clk        tb_clk.v        per-SM MMCM / PLL + DRP + glitch-free clock gate
  system     tb_system.v     vc707_top, 3 SMs (MMCM, PLL, MMCM), every UART command
  system24   tb_system24.v   vc707_top, 24 SMs (the default build: 4 GPC x 3 TPC x 2 SM)
  e2e        host/e2e_rtl.py gals_host.py encoders <-> RTL <-> tile_model.py
  gals       host/check_gals.py  verify.py --gals against a fake GALS board
                             (must PASS) and a fake shared-MMCM board (must FAIL)
  uart, en_gen, win_ctrl     v7 testbenches (win_ctrl: 24-bit N_k, power-on sequences)
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RTL = sorted(os.path.join("rtl", f) for f in os.listdir(os.path.join(ROOT, "rtl")) if f.endswith(".v"))
SIM = ["sim/sim_prims.v"]

TESTS = {
    "sm":       ("tb_sm",       SIM + RTL),
    "clk":      ("tb_clk",      SIM + ["rtl/clk_ctrl.v", "rtl/sm_clk.v"]),
    "system":   ("tb_system",   SIM + RTL),
    "system24": ("tb_system24", SIM + RTL),
    "uart":     ("tb_uart",     ["rtl/uart.v"]),
    "en_gen":   ("tb_en_gen",   ["rtl/en_gen.v"]),
    "win_ctrl": ("tb_win_ctrl", ["rtl/win_ctrl.v", "rtl/en_gen.v"]),
}


def run_tb(name):
    top, srcs = TESTS[name]
    os.makedirs("tb_out", exist_ok=True)
    vvp = os.path.join("tb_out", top + ".vvp")
    c = subprocess.run(["iverilog", "-g2012", "-o", vvp, "-s", top] + srcs + [f"tb/{top}.v"],
                       capture_output=True, text=True)
    if c.returncode != 0:
        print(f"FAIL  {name:9s} (compile error)")
        print(c.stdout + c.stderr)
        return False
    out = subprocess.run(["vvp", "-n", vvp], capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines() if "PASS" in l or "FAIL" in l]
    ok = any("PASS" in l for l in lines) and not any(l.startswith("FAIL") for l in lines)
    print(f"{'PASS' if ok else 'FAIL'}  {name:9s} {lines[-1] if lines else '(no result line)'}")
    if not ok:
        print(out)
    return ok


def main(argv):
    os.chdir(ROOT)
    names = argv or list(TESTS) + ["e2e", "gals"]
    ok = True
    for n in names:
        if n in ("e2e", "gals"):
            script = "host/e2e_rtl.py" if n == "e2e" else "host/check_gals.py"
            r = subprocess.run([sys.executable, script], capture_output=True, text=True)
            good = r.returncode == 0
            print(f"{'PASS' if good else 'FAIL'}  {n:9s} {r.stdout.strip().splitlines()[-1]}")
            ok &= good
        else:
            ok &= run_tb(n)
            if n == "sm":
                r = subprocess.run([sys.executable, "host/check_model.py"], capture_output=True, text=True)
                good = r.returncode == 0
                print(f"{'PASS' if good else 'FAIL'}  model     {r.stdout.strip().splitlines()[-1]}")
                ok &= good
    print("ALL PASS" if ok else "SOME TESTS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
