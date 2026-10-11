#!/usr/bin/env python3
"""uart_diag.py -- raw-byte check of the VC707 UART link for the 10_6 GPU build (921600 8-N-1).

Usage:  python host\\uart_diag.py COM9            normal check
        python host\\uart_diag.py COM9 --sweep    also try lower baud rates

Keeps ONE port open, sends a few safe commands and prints every byte that
comes back -- extra or missing bytes included -- so a link problem can be
told apart from a protocol problem. Only SM 0 is ever switched on, and
only briefly. Never sends SET_FREQ.

How to read the result:
  * PING answers 47 41 4C 53 90 ("GALS" build 0x90)  -> link is fine
  * PING answers ... 80 / 71                           -> the 110_3 (12 tiles) / v7.x bitstream
  * replies all 00 / wrong length -> baud mismatch: LED 5 must blink exactly
    once per second (clk_mem = the board's 200 MHz), try --sweep
  * nothing at all -> wrong COM port, bitstream not loaded
"""
import sys
import time

import serial

PING = bytes([0x09])
PING_OK = bytes([0x47, 0x41, 0x4C, 0x53])


def open_port(port, baud):
    s = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1,
                      timeout=0.25, rtscts=False, xonxoff=False, dsrdtr=False)
    time.sleep(0.05)          # > 10 ms: the board drops any half-received command
    s.reset_input_buffer()
    return s


def cmd(s, payload, label, expect):
    s.reset_input_buffer()
    s.write(bytes(payload))
    s.flush()
    time.sleep(0.15)
    got = s.read(4096)
    ok = "OK " if len(got) == expect else "?? "
    print(f"{ok}{label:<34} sent [{bytes(payload).hex(' ')}]  expect {expect} byte(s), "
          f"got {len(got)}: [{got[:24].hex(' ')}{' ...' if len(got) > 24 else ''}]")
    return got


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else "COM9"
    sweep = "--sweep" in sys.argv

    s = open_port(port, 921600)
    time.sleep(0.3)
    idle = s.read(4096)
    print(f"port {port} open @921600. bytes arriving while idle (should be none): {len(idle)}"
          + (f"  -> {idle[:32].hex(' ')}" if idle else ""))

    p = cmd(s, PING, "PING", 5)
    if p[:4] == PING_OK and len(p) >= 5:
        print(f"    -> link OK, bitstream build 0x{p[4]:02X}" + ("" if p[4] == 0x90 else "  (not the 10_6 GPU build!)"))
    elif not p:
        print("    -> no answer: link down or no bitstream")
    else:
        print("    -> WRONG bytes: link problem (baud / clk_mem frequency) -- see --sweep")
    info = cmd(s, [0x0F], "INFO", 12)
    mb = 3
    if len(info) == 12:
        mb = (info[0] + 7) // 8
        print(f"    -> {info[0]} SMs ({info[0] // max(info[10], 1)} GPCs x {info[10]}), "
              f"{info[1]} sub-partitions x {info[2]} x {info[3]} MACs, VCO {(info[5] << 8) | info[6]} MHz, "
              f"D {info[7]}..{info[8]}")
    f0 = cmd(s, [0x10, 0x00], "GET_FREQ SM 0", 2)
    if len(f0) == 2:
        print(f"    -> SM 0: D {f0[0]} ({1200 / max(f0[0], 1):.0f} MHz), status {f0[1]:04b} "
              "(bit0 locked, bit1 running, bit3 boot done)")
    off = [0x04] + [0x00] * mb
    on0 = [0x04] + [0x00] * (mb - 1) + [0x01]
    cmd(s, off, "SET_EN none (expect A0)", 1)
    cmd(s, [0x02, 0x00], "READ_ACT SM 0", 4)
    cmd(s, on0, "SET_EN SM 0 only", 1)
    time.sleep(0.2)
    a1 = cmd(s, [0x02, 0x00], "READ_ACT SM 0", 4)
    time.sleep(0.2)
    a2 = cmd(s, [0x02, 0x00], "READ_ACT SM 0 (+0.2 s)", 4)
    cmd(s, off, "SET_EN none (off again)", 1)
    if len(a1) == 4 and len(a2) == 4:
        d = (int.from_bytes(a2, "big") - int.from_bytes(a1, "big")) & 0xFFFFFFFF
        print(f"    -> SM 0 ran {d:,} cycles between the two reads "
              f"(~{d / 0.35 / 1e6:.0f} MHz, rough: includes USB latency; 0 = not running)")
    st = cmd(s, [0x0C, 0x00], "READ_STAT SM 0", 16)
    if len(st) == 16:
        print(f"    -> blocks {int.from_bytes(st[4:8], 'big'):,}, checksum 0x{st[8:].hex()}")
    cmd(s, [0x02, 0x63], "READ_ACT SM 99 (no such SM -> E1)", 1)
    s.close()

    if sweep:
        print("\n--sweep: PING at lower baud rates (finds a clk_mem that is too slow)")
        for baud in (921600, 614400, 460800, 307200, 230400, 115200):
            try:
                s = open_port(port, baud)
            except Exception as e:  # noqa: BLE001 - report and keep going
                print(f"  {baud:>7}: cannot open ({e})")
                continue
            got = cmd(s, PING, f"PING @ {baud}", 5)
            s.close()
            if got[:4] == PING_OK:
                print(f"  -> board answers correctly at {baud} baud: clk_mem is about "
                      f"{200 * baud / 921600:.0f} MHz instead of 200 MHz")
                break
            time.sleep(0.05)


if __name__ == "__main__":
    main()
