# GALS Tensor GPU on FPGA

**A scaled-down NVIDIA-style tensor-core GPU on a Virtex-7 FPGA, where every SM runs on its own clock.**

24 streaming multiprocessors (SMs) are organized like a data-center GPU (4 GPC × 3 TPC × 2 SM). Each SM has four tensor-core sub-partitions, for 1,536 int16 MACs in total. Unlike a real GPU, every SM has its own MMCM or PLL and can be re-clocked live without disturbing the others (GALS: globally asynchronous, locally synchronous). Every SM also counts its active cycles in 1 ms windows and streams the counts to a PC, so per-SM power can be fitted against measured board power.

**Target:** Xilinx VC707 (XC7VX485T-2) · Verilog-2001 · Vivado 2025.2 · Icarus Verilog 12 · Python 3

---

## Highlights

| | |
|---|---|
| Compute | 24 SMs · 96 tensor cores · 1,536 MACs on DSP48E1, int16 × int16 → 48-bit accumulate |
| Peak throughput | 614 GMAC/s at 400 MHz |
| Clocks | 24 independent SM clocks (12 MMCM + 12 PLL), 1200 MHz ÷ D, changed live per SM over DRP; only the re-clocked SM pauses (about 100 µs) |
| Timing | All 24 SM clocks closed at 400 MHz (WNS +0.019 ns, WHS +0.032 ns); 200 MHz front end |
| Floorplan | One SM per half clock region, one TPC per clock region, one GPC per die corner |
| Instrumentation | Per-SM duty-cycle enable (80–100 % active per window); 24-bit active-cycle count per SM per 1 ms window, logged over UART |
| Verification | All simulation and co-simulation tests pass; RTL results are bit-exact against a Python model |

---

## Architecture

### Hierarchy

```
GPU (XC7VX485T)
├── Front end (clk_mem, 200 MHz): UART host interface, window engine, clock controller
└── 4 GPC   one per die corner
    └── 3 TPC   one clock region each = one CMT (1 MMCM + 1 PLL)
        └── 2 SM   half a clock region each; slot 0 → MMCM clock, slot 1 → PLL clock
            └── 4 sub-partitions, one tensor core each
                └── 2 × 8 MAC array (16 × DSP48E1)
```

SM index: `sm = 6·gpc + 2·tpc + slot`.

### Floorplan

```
        X0       X1
  Y6   GPC 2    GPC 3    TPC 0 (corner)
  Y5   GPC 2    GPC 3    TPC 1
  Y4   GPC 2    GPC 3    TPC 2
  Y3   front end + spine registers
  Y2   GPC 0    GPC 1    TPC 2
  Y1   GPC 0    GPC 1    TPC 1
  Y0   GPC 0    GPC 1    TPC 0 (corner)
```

Each SM is locked into half a clock region by a Pblock (`constraints/pblocks.tcl`), so heating one corner of the die mostly affects one GPC. GPC and TPC are placement and control groupings (enable masks such as `en gpc:0`); there is no GPC-level logic, and SMs do not communicate with each other.

### Module hierarchy

```
vc707_top
├── IBUFDS + BUFG                  board 200 MHz → clk_mem
├── host_if                        UART command decoder
│   └── uart
├── win_ctrl                       1 ms windows, duty sequences, N_k log
├── clk_ctrl                       boot-time clock setup, live SET_FREQ over DRP
└── SM[i]  (generate × 24)
    ├── spine registers            one clk_mem register stage between front end and SM
    ├── sm_clk                     MMCME2_ADV (slot 0) or PLLE2_ADV (slot 1)
    ├── BUFH / BUFHCE              regional clock buffers (free-running / gated SM clock)
    └── sm_top                     everything in the SM's Pblock
        ├── en_gen                 duty-cycle enable, active-cycle counters
        ├── tog_cdc × 2            TILE_CFG / RUN_ONE handshakes
        ├── stat_cdc               consistent status snapshot
        └── sm_core                controller, BRAMs, ReLU → checksum
            ├── B BRAM             shared by the 4 sub-partitions
            ├── SUBP[s] × 4        sub-partition: A BRAM + tensor_core
            │   └── tensor_core    2 × 8 mac_cell + readout chain
            │       └── mac_cell × 16   one DSP48E1
            └── result LUTRAM × 4  raw C of RUN_ONE blocks
```

### What one SM computes

Each SM repeatedly computes one GEMM tile:

```
C (8 × 8) = A (8 × K) × B (K × 8),    K = 40 … 1023 (default 256)
```

- **Output-stationary:** each DSP48E1 owns one `C[i][j]` and accumulates `P += A[i][k] · B[k][j]` over K steps.
- **Data reuse:** sub-partition *s* keeps rows 2*s* and 2*s*+1 of A in its own BRAM. B is read from one shared BRAM and broadcast to all four tensor cores. This mirrors the Hopper `wgmma` split, where the four warps of a warpgroup take different M rows and share B.
- **Throughput:** one k-step per enabled cycle, i.e. 64 MACs per SM per cycle.
- **Epilogue:** after the last k-step, all 64 accumulators are copied into readout chains in one cycle, so the DSPs start the next block immediately. The readout applies ReLU and folds the values into a 64-bit checksum.
- **RUN_ONE:** computes a single block and keeps the 64 raw results for readback.
- **Data:** A (activations) differs per SM through a per-SM seed; B (weights) is the same in all SMs. BRAM contents are generated at power-up and can be rewritten over UART.

### Industry architecture vs. what this project adds

| NVIDIA data-center GPU (Hopper / Blackwell) | This design |
|---|---|
| GPU → GPC → TPC (2 SMs) → SM | 4 GPC × 3 TPC × 2 SM |
| 4 sub-partitions per SM, one tensor core each | Same: `sm_core.v` → 4 × `tensor_core.v` |
| Warpgroup MMA: 4 warps split the M rows and share B | 4 sub-partitions own 2 rows of C each; B broadcast from one BRAM |
| Accumulators in tensor memory, epilogue afterwards | Accumulators in DSP P registers; readout chain → ReLU → checksum |
| FP16 / BF16 / FP8 inputs, FP32 accumulate | int16 inputs, 48-bit integer accumulate (the DSP48E1 is an integer multiplier) |
| One SM clock domain shared by all SMs; per-SM clock gating | **Added:** each SM has its own MMCM/PLL and regional clock tree, live per-SM frequency change, and per-SM clock gating |

Publicly documented NVIDIA GPUs run all SMs in a single SM clock domain. Per-SM clocks are the power/thermal mechanism this project adds on top of an otherwise industry-style compute array. Not modeled: CUDA cores, warp schedulers, an instruction set, L2, and HBM; A and B come from on-chip BRAM.

### Clocking (GALS)

```
board 200 MHz → IBUFDS → BUFG → clk_mem ─┬─ front end (host_if, win_ctrl, clk_ctrl)
                                          └─ per SM: MMCM or PLL (VCO 1200 MHz, ÷ D)
                                                ├─ BUFHCE → SM clock (gated, regional)
                                                └─ BUFH   → clk_free (drives the CE shift register)
```

- **Frequencies:** D = 3 … 126 gives 400 MHz … 9.5 MHz. The bitstream is timed at D = 3 (400 MHz); every SM boots at D = 6 (200 MHz).
- **SET_FREQ for one SM:** stop its BUFHCE cleanly → reset its MMCM/PLL → rewrite the divider over DRP → wait for lock → restart. The SM keeps its state, and its TPC neighbour (the other clock of the same CMT) keeps running.
- **Regional clock trees:** BUFH/BUFHCE instead of global BUFGs; each SM clock stays inside its own clock region. Only 1 of the 32 global buffers is used.
- **Clock-domain crossings:** 25 asynchronous domains. Host writes use dual-clock BRAMs; commands use toggle handshakes (`tog_cdc`); status uses consistent snapshots (`stat_cdc`); a spine register stage sits between the front end and each SM.

### Power instrumentation

- `en_gen` gates the datapath cycle by cycle following a per-SM duty sequence (80–100 % active per 1 ms window). An idle cycle reads no BRAM and toggles no datapath register.
- Each SM counts its active cycles N_k per window (24-bit). `win_ctrl` packs all 24 counts into a 76-byte record per window, streamed over UART at 921,600 baud.
- `host/regress.py` fits per-SM power coefficients from N_k and externally measured board power (sliding-window regression).
- `scripts/temp_log.tcl` logs on-chip temperature and supply voltages from the XADC over JTAG.

---

## Results

### Implementation (Vivado 2025.2, xc7vx485tffg1761-2)

| Resource | Used | Device |
|---|---:|---:|
| LUT | 88,632 | 29 % |
| FF | 159,906 | 26 % |
| DSP48E1 | 1,536 | 55 % |
| Block RAM tiles | 115 | 11 % |
| MMCM / PLL | 12 / 12 | 86 % / 86 % |
| Global clock buffers (BUFGCTRL) | 1 | 3 % |

- All timing constraints met: WNS +0.019 ns, WHS +0.032 ns, with every SM clock checked at 400 MHz.
- Each SM clock net reaches only its own clock region.
- Vectorless power estimate: 7.8 W (low confidence). With all 24 SMs computing at 400 MHz and 50 % toggle, the estimate reaches about 24 A on VCCINT, above the VC707's 20 A cut-off, so frequency and duty are budgeted per SM. Rule of thumb from the estimate: VCCINT ≈ 1.5 A + 0.0024 A/MHz × Σ(MHz of active SMs).

### Simulation

`python scripts/run_sims.py` (Icarus Verilog 12). All tests pass.

| Test | What it checks |
|---|---|
| `tb_sm` + `check_model.py` | One SM at 400 MHz against a GEMM reference: single blocks (64/64 exact), host writes including −32768², continuous runs with random enable, duty FSM, clear mid-run. RTL results are bit-exact with `host/tile_model.py`. |
| `tb_clk` | MMCM and PLL SMs boot at 200 / 400 / 150 MHz and are re-clocked live to 100 / 300 / 240 MHz; the neighbouring SM never stops. |
| `tb_system` | Full top level with 3 SMs, driven over UART: every command, live re-clocking, window counts within ±2 cycles. |
| `tb_system24` | Default 24-SM build: all 24 clocks lock, per-GPC enable, 76-byte window records. |
| `e2e_rtl.py` | Host encoder → RTL → host decoder → Python model, end to end. |
| `check_gals.py` | The on-board GALS test passes on a simulated GALS board and correctly fails on a simulated shared-MMCM board. |
| `tb_uart`, `tb_en_gen`, `tb_win_ctrl` | Unit tests. |

### On the board (bring-up in progress)

- **Window log:** 4,993 consecutive 1 ms windows recorded from GPC 0 (6 SMs: 3 MMCM-clocked, 3 PLL-clocked) with no missing or repeated records. Every count is a whole-percent duty of 200,000 cycles (±1 cycle), consistent with each SM clock running at exactly 200 MHz.
- **Thermal logging:** XADC temperature and supply voltages logged at 10 samples/s.

**Next:** per-SM functional sweep on hardware (`verify.py`), live re-clocking and GALS isolation on hardware (`verify.py --freq`, `--gals`), external power measurement with per-SM regression, and heat-gun localization by GPC.

---

## Repository layout

```
rtl/            synthesizable Verilog (13 modules, no vendor IP)
sim/            simulation-only models of IBUFDS / BUFG / BUFH / BUFHCE / MMCME2_ADV / PLLE2_ADV
tb/             testbenches
constraints/    pins, clock groups, per-SM Pblocks
scripts/        Vivado project / reports / floorplan coloring / power estimate, simulation runner, XADC logger
host/           Python host tools: board driver, on-board verification, bit-exact model, regression
reports/        Vivado implementation reports
```

---

## Getting started

### Simulate

```bash
pip install pyserial numpy
python scripts/run_sims.py            # all tests (about 15 minutes)
python scripts/run_sims.py sm clk     # selected tests
```

### Build

In the Vivado Tcl console:

```tcl
source <repo>/scripts/create_project.tcl   ;# creates the Vivado project under vivado/ (xc7vx485tffg1761-2)
# Generate Bitstream, then:
source <repo>/scripts/reports.tcl          ;# timing / utilization / clocks / power → reports/
```

Do not add `sim/` to the Vivado project; those files only stand in for Xilinx primitives in Icarus Verilog.

### Run on a VC707

> **Power:** the VC707 cuts VCCINT at 20 A. All SMs boot at 200 MHz and idle. Turn DIP 5 (all SMs on) off before programming, and check `scripts/power_realistic.tcl` before raising many SMs to 400 MHz.

```bash
python host/uart_diag.py COM9                          # link check: PING → "GALS" 0x90
python host/gals_host.py --port COM9 freq              # every SM's clock and lock state
python host/verify.py    --port COM9 --sms 0,1         # one TPC (MMCM SM + PLL SM) vs. the model
python host/verify.py    --port COM9 --sms 0,1 --gals  # re-clock one SM; its neighbour must not notice
python host/gals_host.py --port COM9 en gpc:0          # run GPC 0 continuously
python host/gals_host.py --port COM9 log --seconds 5 --out windows.csv
```

Replace `COM9` with your serial port (for example `/dev/ttyUSB0` on Linux).

Board I/O: DIP 0–3 run GPC 0–3, DIP 5 runs every SM, DIP 4 starts the window engine. LED 0–3 show GPC 0–3 computing, LED 4 = boot done and all 24 clocks locked, LED 5 = 1 Hz heartbeat, LED 6 = window engine running.

<details>
<summary>UART protocol (921,600 baud, 8-N-1, build 0x90)</summary>

`TT` = SM index 0–23 (`FF` = all SMs for WRITE_A / WRITE_B). `MASK` = 3 bytes, most significant byte first, bit *s* = SM *s*. Any error returns `E1`.

| Op | Name | Request | Response |
|---|---|---|---|
| 01 | WRITE_A | `01 TT AH AL` + 16 bytes | `A0`; one column of A (8 values) |
| 0B | WRITE_B | `0B TT AH AL` + 16 bytes | `A0`; one row of B (8 values) |
| 02 | READ_ACT | `02 TT` | 4 bytes: activity count |
| 03 | READ_CHK | `03 TT` | 8 bytes: checksum |
| 0C | READ_STAT | `0C TT` | 16 bytes: activity, blocks done, checksum |
| 04 | SET_EN | `04 M2 M1 M0` | `A0`; SMs in the mask run continuously (ORed with DIP) |
| 05 | SET_FREQ | `05 TT DD` | `A0` / `E1`; f = 1200 / DD MHz, 3 ≤ DD ≤ 126 |
| 10 | GET_FREQ | `10 TT` | `DD ST`: locked, running, busy, boot done |
| 0A | TILE_CFG | `0A M2 M1 M0 KH KL` | `A0`; set K and clear address / checksum / block count |
| 0D | RUN_ONE | `0D TT` | `A0` after one block completes |
| 0E | READ_RES | `0E TT` | 384 bytes: C[0][0] … C[7][7], 6 bytes each (48-bit, before ReLU) |
| 06 | SEQ_WR | `06 TT IH IL VV` | `A0`; write one duty-sequence entry |
| 07 | SEQ_CFG | `07 FF LH LL W3 W2 W1 W0` | `A0`; flags, sequence length, window length |
| 08 | READ_WIN | `08 MM` | `ST CC` + CC records of 76 bytes (window index + 24 × 24-bit N_k) |
| 09 | PING | `09` | `47 41 4C 53 90` ("GALS", build 0x90) |
| 0F | INFO | `0F` | 12 bytes: build parameters |

</details>

---

## Acknowledgments

Developed as part of FPGA power and thermal research with Prof. Young H. Cho at the University of Southern California.

**Author:** Ian Chen
