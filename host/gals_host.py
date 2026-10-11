#!/usr/bin/env python3
"""gals_host.py -- talk to the 10_6 GPU (24 SMs, build 0x90) over the VC707 USB-UART.

Protocol: rtl/host_if.v (921600 baud, 8-N-1, no flow control). Needs
pyserial (pip install pyserial) and numpy.

The GPU: 4 GPCs x 3 TPCs x 2 SMs = 24 SMs; SM s = GPC s//6, TPC (s%6)//2,
slot s%2 (slot 0 clocked by its region's MMCM, slot 1 by its PLL).
SET masks: "all", "gpc:0,2" (every SM of those GPCs), "sm:0,5,13" (just
those SMs), 0 (none) or an integer bit mask (bit s = SM s, e.g. 0x3F).

Windows: the port is COMx ("Silicon Labs CP210x USB to UART Bridge" in
Device Manager). Examples:

  python host\\gals_host.py --port COM9 ping                 link check -> GALS build 0x90
  python host\\gals_host.py --port COM9 info                 SMs, GPCs, MAC grid, divider range
  python host\\gals_host.py --port COM9 freq                 every SM's clock (GPC / TPC / MMCM-PLL)
  python host\\gals_host.py --port COM9 freq 13 8            SM 13 -> 1200/8 = 150 MHz
  python host\\gals_host.py --port COM9 cfg all --k 256      clear all 24 SMs, K = 256
  python host\\gals_host.py --port COM9 one 0                SM 0: one block, print C (8 x 8)
  python host\\gals_host.py --port COM9 en gpc:0             the 6 SMs of GPC 0 run continuously
  python host\\gals_host.py --port COM9 stat                 activity / blocks / checksum per SM
  python host\\gals_host.py --port COM9 log --seconds 5 --out windows.csv
  python host\\gals_host.py --port COM9 seq --random 1       new random duty sequences
  python host\\gals_host.py --port COM9 seq --random 1 --avg 50   same, 50 % average activity
  python host\\gals_host.py --port COM9 freq all 3          every SM -> 400 MHz, one at a time

Power: the VC707 cuts VCCINT at 20 A. Every SM boots at 200 MHz. Raise
frequencies / enable many SMs step by step (see README "power").
"""
import argparse
import csv
import struct
import sys
import time

ACK, NACK = 0xA0, 0xE1
CLK_MEM_HZ = 200_000_000
NUM_SM = 24
NROW, NCOL = 8, 8                # C block of one SM
NRES = NROW * NCOL               # 64 results per block
MB = (NUM_SM + 7) // 8           # mask bytes (3)
NK_BYTES = 3                     # N_k bytes per SM in a READ_WIN record
SM_PER_TPC, TPC_PER_GPC = 2, 3
SM_PER_GPC = SM_PER_TPC * TPC_PER_GPC


def sm_place(sm):
    """(gpc, tpc, slot) of SM sm; slot 0 = MMCM, 1 = PLL"""
    return sm // SM_PER_GPC, (sm % SM_PER_GPC) // SM_PER_TPC, sm % SM_PER_TPC


def sm_label(sm):
    g, t, s = sm_place(sm)
    return f"SM {sm:2d} (GPC {g} TPC {t} {'PLL ' if s else 'MMCM'})"


def parse_mask(text, num_sm=NUM_SM):
    """'0x3F' / '63' -> int; 'all' -> every SM; 'gpc:0,2' -> SMs of GPC 0 and 2;
    'sm:0,5,13' -> just those SMs"""
    text = str(text).strip().lower()
    if text == "all":
        return (1 << num_sm) - 1
    if text.startswith("sm:"):
        m = 0
        for x in text[3:].split(","):
            if not 0 <= int(x) < num_sm:
                raise ValueError(f"no SM {x} (0..{num_sm - 1})")
            m |= 1 << int(x)
        return m
    if text.startswith("gpc:"):
        m = 0
        for g in text[4:].split(","):
            g = int(g)
            for s in range(g * SM_PER_GPC, min((g + 1) * SM_PER_GPC, num_sm)):
                m |= 1 << s
        return m
    return int(text, 0)


class ProtocolError(IOError):
    pass


def _tile_byte(tile):
    return 0xFF if tile in ("all", None, 0xFF) else int(tile) & 0xFF


def _s48(v):
    return v - (1 << 48) if v & (1 << 47) else v


class Proto:
    """Byte encoding / decoding of every command (no I/O), so the same code
    can be checked against the RTL in simulation (host/e2e_rtl.py)."""

    # ---- encoders: return (command bytes, reply length) -------------------
    @staticmethod
    def ping():
        return bytes([0x09]), 5

    @staticmethod
    def info():
        return bytes([0x0F]), 12

    @staticmethod
    def write_a(tile, addr, col):
        """col: A[0..NROW-1][addr] (8 values); word bits [i*16 +: 16] = A[i]; sent MSB first"""
        if len(col) != NROW:
            raise ValueError(f"an A column has {NROW} values (rows 0..{NROW - 1})")
        word = 0
        for i, v in enumerate(col):
            word |= (int(v) & 0xFFFF) << (16 * i)
        return bytes([0x01, _tile_byte(tile), (addr >> 8) & 0xFF, addr & 0xFF]) + word.to_bytes(2 * NROW, "big"), 1

    @staticmethod
    def write_b(tile, addr, row8):
        """row8: B[addr][0..7]"""
        if len(row8) != 8:
            raise ValueError("a B row has 8 values (columns 0..7)")
        word = 0
        for j, v in enumerate(row8):
            word |= (int(v) & 0xFFFF) << (16 * j)
        return bytes([0x0B, _tile_byte(tile), (addr >> 8) & 0xFF, addr & 0xFF]) + word.to_bytes(16, "big"), 1

    @staticmethod
    def read_act(tile):
        return bytes([0x02, tile]), 4

    @staticmethod
    def read_chk(tile):
        return bytes([0x03, tile]), 8

    @staticmethod
    def read_stat(tile):
        return bytes([0x0C, tile]), 16

    @staticmethod
    def set_en(mask, mb=MB):
        """mb = mask bytes = ceil(NUM_SM / 8) of the bitstream (3 for 24 SMs)"""
        return bytes([0x04]) + int(mask).to_bytes(mb, "big"), 1

    @staticmethod
    def set_freq(tile, div):
        return bytes([0x05, tile, div]), 1

    @staticmethod
    def get_freq(tile):
        return bytes([0x10, tile]), 2

    @staticmethod
    def tile_cfg(mask, k, mb=MB):
        return bytes([0x0A]) + int(mask).to_bytes(mb, "big") + bytes([(k >> 8) & 0xFF, k & 0xFF]), 1

    @staticmethod
    def run_one(tile):
        return bytes([0x0D, tile]), 1

    @staticmethod
    def read_res(tile):
        return bytes([0x0E, tile]), 6 * NRES

    @staticmethod
    def seq_write(tile, index, off):
        if not 0 <= off <= 100:
            raise ValueError("OFF must be 0..100 (cycles off per 100)")
        return bytes([0x06, tile]) + struct.pack(">HB", index, off), 1

    @staticmethod
    def seq_cfg(run, clear=False, seq_len=0, win_cycles=0):
        flags = (1 if run else 0) | (2 if clear else 0)
        return bytes([0x07, flags]) + struct.pack(">HI", seq_len, win_cycles), 1

    # ---- decoders ------------------------------------------------------------
    @staticmethod
    def check_ack(r, what):
        if r[0] != ACK:
            raise ProtocolError(f"{what}: expected A0, got {r.hex(' ')}")

    @staticmethod
    def d_ping(r):
        if r[:4] != b"GALS":
            raise ProtocolError(f"PING: expected 47 41 4C 53 xx, got {r.hex(' ')} (UART link problem)")
        return r[4]

    @staticmethod
    def d_info(r):
        return dict(num_sm=r[0], n_sub=r[1], rows=r[2], cols=r[3], addr_bits=r[4],
                    vco_mhz=(r[5] << 8) | r[6], div_min=r[7], div_max=r[8], k_min=r[9],
                    sm_per_gpc=r[10], nk_bytes=r[11])

    @staticmethod
    def win_record_bytes(num_sm, nk_bytes=NK_BYTES):
        return 4 + nk_bytes * num_sm

    @staticmethod
    def d_win_records(body, count, num_sm, nk_bytes=NK_BYTES):
        """[(window_index, [N_sm0 .. N_sm(T-1)]), ...] from READ_WIN record bytes"""
        rec = Proto.win_record_bytes(num_sm, nk_bytes)
        out = []
        for i in range(count):
            r = body[i * rec:(i + 1) * rec]
            idx = int.from_bytes(r[0:4], "big")
            nk = [int.from_bytes(r[4 + nk_bytes * t:4 + nk_bytes * (t + 1)], "big") for t in range(num_sm)]
            out.append((idx, nk))
        return out

    @staticmethod
    def d_u32(r):
        return int.from_bytes(r[:4], "big")

    @staticmethod
    def d_u64(r):
        return int.from_bytes(r[:8], "big")

    @staticmethod
    def d_stat(r):
        return (int.from_bytes(r[0:4], "big"), int.from_bytes(r[4:8], "big"), int.from_bytes(r[8:16], "big"))

    @staticmethod
    def d_freq(r):
        st = r[1]
        return r[0], dict(locked=bool(st & 1), running=bool(st & 2), busy=bool(st & 4), boot_done=bool(st & 8))

    @staticmethod
    def d_res(r):
        return [_s48(int.from_bytes(r[6 * i:6 * i + 6], "big")) for i in range(NRES)]


class Board:
    def __init__(self, port, baud=921_600, timeout=1.0, ser=None):
        if ser is None:
            import serial  # pyserial
            ser = serial.Serial(port, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                                timeout=timeout, rtscts=False, xonxoff=False)
        self.ser = ser
        time.sleep(0.05)                 # > 10 ms of silence: the board drops any half command
        self.ser.reset_input_buffer()
        self._info = None

    # ---- transport -------------------------------------------------------------
    def _xfer(self, cmd, n_reply):
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        data = self.ser.read(n_reply)
        if len(data) == 1 and n_reply > 1 and data[0] == NACK:
            raise ProtocolError(f"command {cmd[:4].hex(' ')}: board answered E1 (bad tile index / busy / timeout)")
        if len(data) != n_reply:
            raise ProtocolError(f"expected {n_reply} reply bytes, got {len(data)}: {data[:16].hex(' ')} "
                                "(check port, 921600 baud, bitstream loaded; host/uart_diag.py checks the raw link)")
        return data

    def _ack(self, enc, what):
        cmd, n = enc
        Proto.check_ack(self._xfer(cmd, n), what)

    # ---- commands ----------------------------------------------------------------
    def ping(self):
        return Proto.d_ping(self._xfer(*Proto.ping()))

    def info(self):
        if self._info is None:
            raw = self._xfer(*Proto.info())
            inf = Proto.d_info(raw)
            if not (1 <= inf["num_sm"] <= 64 and inf["vco_mhz"] >= 600 and inf["nk_bytes"] in (3, 4)):
                raise ProtocolError(
                    f"INFO answered {raw.hex(' ')} -- not the GPU bitstream talking. All 00 = the "
                    "board's UART line is held low: FPGA not configured (power tripped or reset?). "
                    "Check LED 4 / LED 5, re-program the device, then run host/uart_diag.py")
            self._info = inf
        return self._info

    @property
    def num_tiles(self):
        return self.info()["num_sm"]

    num_sm = num_tiles

    @property
    def mask_bytes(self):
        return (self.num_tiles + 7) // 8

    def write_a(self, tile, addr, col):
        self._ack(Proto.write_a(tile, addr, col), "WRITE_A")

    def write_b(self, tile, addr, row8):
        self._ack(Proto.write_b(tile, addr, row8), "WRITE_B")

    def read_act(self, tile):
        return Proto.d_u32(self._xfer(*Proto.read_act(tile)))

    def read_chk(self, tile):
        return Proto.d_u64(self._xfer(*Proto.read_chk(tile)))

    def read_stat(self, tile):
        """(activity_count, blk_done, chk) -- one consistent snapshot"""
        return Proto.d_stat(self._xfer(*Proto.read_stat(tile)))

    def set_en(self, mask):
        self._ack(Proto.set_en(mask, self.mask_bytes), "SET_EN")

    def set_freq(self, tile, div):
        """tile clock = vco / div. Returns True on success (A0)."""
        cmd, n = Proto.set_freq(tile, div)
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        old = self.ser.timeout
        self.ser.timeout = max(old or 0, 1.0)
        r = self.ser.read(1)
        self.ser.timeout = old
        if len(r) != 1:
            raise ProtocolError("SET_FREQ: no reply")
        return r[0] == ACK

    def get_freq(self, tile):
        return Proto.d_freq(self._xfer(*Proto.get_freq(tile)))

    def tile_mhz(self, tile):
        div, _ = self.get_freq(tile)
        return self.info()["vco_mhz"] / div

    def tile_cfg(self, mask, k):
        self._ack(Proto.tile_cfg(mask, k, self.mask_bytes), "TILE_CFG")

    def run_one(self, tile):
        self._ack(Proto.run_one(tile), "RUN_ONE")

    def read_res(self, tile):
        """the 64 raw C values of the last RUN_ONE block, row-major (C[0][0], C[0][1], ...)"""
        return Proto.d_res(self._xfer(*Proto.read_res(tile)))

    def seq_write(self, tile, index, off):
        self._ack(Proto.seq_write(tile, index, off), "SEQ_WR")

    def seq_cfg(self, run, clear=False, seq_len=0, win_cycles=0):
        self._ack(Proto.seq_cfg(run, clear, seq_len, win_cycles), "SEQ_CFG")

    def read_windows(self, max_records=255):
        """(running, overflow, [(window_index, [N_sm0 .. N_sm(T-1)]), ...])"""
        nt = self.num_tiles
        nkb = self.info()["nk_bytes"]
        rec = Proto.win_record_bytes(nt, nkb)
        self.ser.reset_input_buffer()
        self.ser.write(bytes([0x08, max_records & 0xFF]))
        hdr = self.ser.read(2)
        if len(hdr) != 2:
            raise ProtocolError("READ_WIN: no header")
        status, count = hdr[0], hdr[1]
        body = self.ser.read(count * rec)
        if len(body) != count * rec:
            raise ProtocolError(f"READ_WIN: expected {count} records, got {len(body)} bytes")
        return bool(status & 1), bool(status & 2), Proto.d_win_records(body, count, nt, nkb)


def load_seq_csv(path, num_tiles, activity=False):
    rows = []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            row = [c.strip() for c in row if c.strip() != ""]
            if not row or row[0].startswith("#"):
                continue
            vals = [int(float(c)) for c in row]
            if len(vals) != num_tiles:
                raise ValueError(f"each row needs {num_tiles} values, got {row}")
            rows.append([100 - v if activity else v for v in vals])
    return rows


M32 = 0xFFFFFFFF


def _xs32(x):
    x ^= (x << 13) & M32
    x ^= x >> 17
    x ^= (x << 5) & M32
    return x & M32


def _hash3(seed, a, b):
    x = (seed ^ ((a * 0x9E3779B9) & M32) ^ ((b * 0x85EBCA6B) & M32) ^ 1) & M32
    if x == 0:
        x = 1
    return _xs32(_xs32(_xs32(x)))


def default_off(sm, i):
    """power-on OFF value of SM sm, window i (rtl/win_ctrl.v def_off)"""
    x = _hash3(0x5EC0D0FF, sm, i >> 1) % 21
    return 20 - x if i & 1 else x


def default_sequences(num_sm=NUM_SM, length=256):
    """rows[window][sm] = OFF, exactly what the board holds after power-up"""
    return [[default_off(t, i) for t in range(num_sm)] for i in range(length)]


def random_sequences(num_tiles, length=256, seed=0, avg=90):
    """rows[window][tile] = OFF; per tile pairs (m-10+x, m+10-x), x uniform 0..20,
    m = 100 - avg, so every pair averages exactly `avg` % activity (avg 90 ->
    pairs (x, 20 - x), the power-on pattern). With 12 tiles this halves the
    per-tile uncertainty of host/regress.py compared with the default 20-entry
    pattern (synthetic test: ~45 vs ~93 mW)."""
    import random
    m = 100 - avg
    if not 10 <= m <= 90:
        raise ValueError("--avg must be between 10 and 90 (%)")
    rng = random.Random(seed)
    length -= length % 2
    cols = []
    for _ in range(num_tiles):
        col = []
        for _ in range(length // 2):
            x = rng.randint(0, 20)
            col += [m - 10 + x, m + 10 - x]
        cols.append(col)
    return [[cols[t][i] for t in range(num_tiles)] for i in range(length)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial port, e.g. COM9 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921_600)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("info")
    p = sub.add_parser("freq", help="show every tile's clock, or set one: freq TILE DIV, or all: freq all DIV")
    p.add_argument("tile", nargs="?", help="SM number, or 'all' (one SM at a time, 0.2 s apart)")
    p.add_argument("div", type=int, nargs="?")
    p = sub.add_parser("en", help="SET_EN: MASK = 'all', 'gpc:0,2', 'sm:0,5,13', 0 (none) or a bit mask like 0x3F")
    p.add_argument("mask")
    p = sub.add_parser("cfg", help="TILE_CFG: clear the SMs in MASK and set K")
    p.add_argument("mask"); p.add_argument("--k", type=int, default=256)
    p = sub.add_parser("one", help="run one block on SM TILE and print C")
    p.add_argument("tile", type=int)
    p = sub.add_parser("stat"); p.add_argument("tile", type=int, nargs="?")
    p = sub.add_parser("seq", help="load duty sequences: --csv FILE, or --random SEED")
    p.add_argument("--csv")
    p.add_argument("--activity", action="store_true", help="CSV holds activity %% instead of OFF counts")
    p.add_argument("--random", type=int, metavar="SEED",
                   help="random per-SM sequences of --len windows, OFF in pairs (x, 20-x): "
                        "every pair still averages 90%% (flat temperature), the SMs are "
                        "uncorrelated (the power-on sequences are already like this)")
    p.add_argument("--len", type=int, default=256)
    p.add_argument("--avg", type=int, default=90,
                   help="with --random: average activity in %% (pairs avg-10..avg+10), e.g. 50 for 400 MHz on every SM")
    p = sub.add_parser("start")
    p.add_argument("--win-ms", type=float, default=1.0)
    p.add_argument("--seq-len", type=int, default=0, help="0 = keep current (power-on: 256)")
    p.add_argument("--clear", action="store_true")
    sub.add_parser("stop")
    p = sub.add_parser("log")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--out", default="windows.csv")
    p.add_argument("--win-ms", type=float, default=1.0)
    p.add_argument("--keep-running", action="store_true")
    a = ap.parse_args(argv)

    b = Board(a.port, a.baud)
    if a.cmd == "ping":
        v = b.ping(); print(f"GALS build 0x{v:02X} -- UART link OK")
    elif a.cmd == "info":
        for k, v in b.info().items():
            print(f"{k:10s} {v}")
    elif a.cmd == "freq":
        if a.tile is not None and a.tile.lower() == "all":
            if a.div is None:
                ap.error("freq all needs a divider, e.g. freq all 3")
            for t in range(b.num_tiles):          # one SM at a time: the current ramps up
                ok = b.set_freq(t, a.div)
                print(("ok: " if ok else "REFUSED (E1): ") + f"SM {t} -> D {a.div} = {b.info()['vco_mhz'] / a.div:.1f} MHz")
                time.sleep(0.2)
            a.tile = None
        elif a.tile is not None:
            a.tile = int(a.tile)
        if a.tile is not None and a.div is not None:
            ok = b.set_freq(a.tile, a.div)
            print(("ok: " if ok else "REFUSED (E1): ") + f"SM {a.tile} -> D {a.div} = {b.info()['vco_mhz'] / a.div:.1f} MHz")
        tiles = [a.tile] if a.tile is not None else range(b.num_tiles)
        for t in tiles:
            d, st = b.get_freq(t)
            print(f"{sm_label(t)}: D {d:3d} = {b.info()['vco_mhz'] / d:7.2f} MHz  "
                  + " ".join(k for k, v in st.items() if v))
    elif a.cmd == "en":
        b.set_en(parse_mask(a.mask, b.num_tiles)); print("ok")
    elif a.cmd == "cfg":
        b.tile_cfg(parse_mask(a.mask, b.num_tiles), a.k); print("ok")
    elif a.cmd == "one":
        b.run_one(a.tile)
        c = b.read_res(a.tile)
        for i in range(NROW):
            print(" ".join(f"{v:14d}" for v in c[i * NCOL:(i + 1) * NCOL]))
    elif a.cmd == "stat":
        tiles = [a.tile] if a.tile is not None else range(b.num_tiles)
        for t in tiles:
            act, blk, chk = b.read_stat(t)
            print(f"{sm_label(t)}: activity {act:12,d}  blocks {blk:10,d}  chk 0x{chk:016x}")
    elif a.cmd == "seq":
        nt = b.num_tiles
        if a.random is not None:
            rows = random_sequences(nt, a.len, a.random, a.avg)
        elif a.csv:
            rows = load_seq_csv(a.csv, nt, a.activity)
        else:
            ap.error("seq needs --csv FILE or --random SEED")
        for i, row in enumerate(rows):
            for t, off in enumerate(row):
                b.seq_write(t, i, off)
        avg = [100 - sum(r[t] for r in rows) / len(rows) for t in range(nt)]
        print(f"loaded {len(rows)} windows; average activity per tile: " + ", ".join(f"{x:.1f}%" for x in avg))
        print(f"start with:  start --seq-len {len(rows)}")
    elif a.cmd == "start":
        b.seq_cfg(True, a.clear, a.seq_len, int(round(a.win_ms * 1e-3 * CLK_MEM_HZ))); print("running")
    elif a.cmd == "stop":
        b.seq_cfg(False); print("stopped")
    elif a.cmd == "log":
        nt = b.num_tiles
        was_running, _, _ = b.read_windows(0)
        if was_running:
            # already running (e.g. `start` before `en all` at 400 MHz): never switch it
            # off, even for a moment -- enabled SMs would jump to 100 %. Drop old records.
            while b.read_windows(255)[2]:
                pass
            print("window engine already running: logging it as it is (and leaving it running)")
        else:
            b.seq_cfg(True, True, 0, int(round(a.win_ms * 1e-3 * CLK_MEM_HZ)))
        t_end = time.time() + a.seconds
        n = 0; overflowed = False
        with open(a.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["window"] + [f"N_sm{t}" for t in range(nt)])
            while time.time() < t_end:
                _, ovf, recs = b.read_windows(255)
                overflowed |= ovf
                for idx, nk in recs:
                    w.writerow([idx] + nk); n += 1
                if not recs:
                    time.sleep(0.02)
        if not a.keep_running and not was_running:
            b.seq_cfg(False)
        print(f"wrote {n} windows to {a.out}"
              + ("  (WARNING: board FIFO overflowed, some windows lost)" if overflowed else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
