#!/usr/bin/env python3
"""tile_model.py -- bit-exact software model of one SM of the 10_6 GPU (rtl/sm_core.v).

(File name kept from 110_3, where an SM was called a "tile".)

One SM computes, block after block, with its 4 sub-partitions together,

    C_b (8 x 8) = A[:, addrs] @ B[addrs, :]       addrs = (b*K + k) mod 512, k = 0..K-1

with 16-bit signed A / B values from the SM's BRAMs and exact 48-bit sums
(|C| < 2^40, so int64 is exact). The board reports

    blk_done = number of finished blocks since the last TILE_CFG
    chk      = sum over those blocks of sum_ij max(C_b[i][j], 0)   mod 2^64
    READ_RES = the 64 raw C values of the last RUN_ONE block (row-major)

BRAM layout (see rtl/sm_core.v):
    A: sm_core SUBP[s].mem[addr] lane r = A[s*2 + r][addr]    (s = 0..3 sub-partition, r = 0..1)
    B: sm_core bmem[addr]        lane c = B[addr][c]          (c = 0..7)
Power-on contents are seeded hashes (the same constant functions the RTL
uses); A differs per SM, B (the weights) is the same in every SM.

Needs numpy.
"""
from math import gcd

import numpy as np

M32 = 0xFFFFFFFF
M64 = (1 << 64) - 1
N_SUB, ROWS, COLS = 4, 2, 8
NROW = N_SUB * ROWS          # 8 rows of C per SM
DEPTH = 512                  # BRAM depth
K_MIN = 40
NRES = NROW * COLS           # 64 results per block
SM_PER_TPC, TPC_PER_GPC = 2, 3
SM_PER_GPC = SM_PER_TPC * TPC_PER_GPC   # 6


def sm_place(sm):
    """(gpc, tpc, slot) of SM sm, as in rtl/vc707_top.v; slot 0 = MMCM, 1 = PLL."""
    return sm // SM_PER_GPC, (sm % SM_PER_GPC) // SM_PER_TPC, sm % SM_PER_TPC


def tile_seeds(t):
    """(SEED_A, SEED_B) of SM t, as in rtl/vc707_top.v."""
    return (0xA5A50000 ^ ((t * 0x9E3779B9) & M32)) & M32, 0x5A5A1234


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


def _s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def power_on_A(seed_a):
    A = np.zeros((NROW, DEPTH), dtype=np.int64)
    for s in range(N_SUB):
        sa = (seed_a ^ (((s + 1) * 0x7F4A7C15) & M32)) & M32
        for a in range(DEPTH):
            for r in range(ROWS):
                A[s * ROWS + r, a] = _s16(_hash3(sa, a, r))
    return A


def power_on_B(seed_b):
    B = np.zeros((DEPTH, COLS), dtype=np.int64)
    for a in range(DEPTH):
        for c in range(COLS):
            B[a, c] = _s16(_hash3(seed_b, a, c))
    return B


class TileModel:
    """One SM. A: 8 x 512, B: 512 x 8 (int64 holding int16 values)."""

    def __init__(self, tile=0, A=None, B=None):
        sa, sb = tile_seeds(tile)
        self.A = power_on_A(sa) if A is None else np.array(A, dtype=np.int64)
        self.B = power_on_B(sb) if B is None else np.array(B, dtype=np.int64)
        self._cache = {}

    # ---- host writes ---------------------------------------------------
    def write_a(self, addr, col):
        """col: NROW (8) values A[0..7][addr]"""
        self.A[:, addr] = [_s16(int(v)) for v in col]
        self._cache.clear()

    def write_b(self, addr, row):
        """row: 8 values B[addr][0..7]"""
        self.B[addr, :] = [_s16(int(v)) for v in row]
        self._cache.clear()

    # ---- what the tile computes -----------------------------------------
    @staticmethod
    def k_eff(K):
        return max(K_MIN, min(int(K), 1023))

    def block(self, b, K):
        """C of block b (8 x 8, int64)."""
        K = self.k_eff(K)
        addrs = (b * K + np.arange(K)) % DEPTH
        return self.A[:, addrs] @ self.B[addrs, :]

    def relu_sum(self, b, K):
        K = self.k_eff(K)
        key = (K, (b * K) % DEPTH)
        if key not in self._cache:
            C = self.block(b, K)
            self._cache[key] = int(np.maximum(C, 0).sum())
        return self._cache[key]

    def checksum(self, nblocks, K):
        """chk after nblocks finished blocks since TILE_CFG (mod 2^64)."""
        K = self.k_eff(K)
        period = DEPTH // gcd(K, DEPTH)              # blocks until the address pattern repeats
        per = sum(self.relu_sum(b, K) for b in range(min(period, nblocks)))
        full, part = divmod(nblocks, period)
        total = full * per + sum(self.relu_sum(b, K) for b in range(part)) if nblocks >= period else per
        return total & M64

    def results(self, b, K):
        """READ_RES order: C[0][0], C[0][1], ... C[7][7] as Python ints."""
        return [int(v) for v in self.block(b, K).reshape(-1)]


def self_test():
    m = TileModel(0)
    K = 256
    direct = sum(m.relu_sum(b, K) for b in range(7)) & M64
    assert m.checksum(7, K) == direct
    K = 40                                         # period 64 blocks
    direct = sum(int(np.maximum(m.block(b, K), 0).sum()) for b in range(130)) & M64
    assert m.checksum(130, K) == direct
    assert abs(int(m.block(0, 1023).max())) < 2 ** 40
    return True


if __name__ == "__main__":
    print("tile_model self-test:", "PASS" if self_test() else "FAIL")
