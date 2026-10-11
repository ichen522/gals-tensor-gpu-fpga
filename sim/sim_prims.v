// sim/sim_prims.v -- SIMULATION-ONLY stand-ins for the Xilinx primitives
// used by the 10_6 RTL (iverilog has no unisim library). DO NOT add this
// file to the Vivado project: Vivado uses the real primitives.
//
//   IBUFDS, BUFG     : wires
//   BUFGCE, BUFHCE   : glitch-free gate (CE latched while I is low), like
//                      the real buffers with CE_TYPE = SYNC; BUFH = wire
//   PLLE2_ADV        : same model as MMCME2_ADV below (same DRP fields),
//                      prints "PLL-ERROR"
//   MMCME2_ADV       : just the parts 110_3 relies on --
//     * CLKOUT1 = VCO / D1, VCO = CLKFBOUT_MULT_F / (CLKIN1_PERIOD * DIVCLK_DIVIDE)
//     * RST high: outputs stop (low), LOCKED low
//     * after RST is released: SIM_LOCK_NS of GARBAGE on CLKOUT1 (fast,
//       irregular toggling, like a real MMCM before lock), then LOCKED = 1
//       and a clean clock at the CURRENT divider. The garbage makes a
//       missing / wrong clock gate visible in the testbenches.
//     * DRP: DEN/DWE/DADDR/DI -> DRDY 3 DCLK cycles later (DO on reads),
//       a 128 x 16 register file. CLKOUT1's divider is decoded from
//       ClkReg1 0x0A {PHASE_MUX[15:13], rsvd[12], HIGH[11:6], LOW[5:0]}
//       and ClkReg2 0x0B {rsvd[15:10], MX[9:8], EDGE[7], NO_COUNT[6], DELAY[5:0]}
//   Self-checks print "MMCM-ERROR" (the testbenches count them): DRP write
//   while not in reset, DEN while busy, HIGH/LOW/EDGE inconsistent, reserved
//   bits not preserved by the read-modify-write (they start as 1 / 101010),
//   phase / MX / delay not zero, POWER register 0x28 not written FFFF.
`timescale 1ns/1ps

module IBUFDS (input wire I, input wire IB, output wire O);
  assign O = I;
endmodule

module BUFG (input wire I, output wire O);
  assign O = I;
endmodule

module BUFGCE (input wire I, input wire CE, output wire O);
  reg ce_l = 1'b0;
  always @(I or CE) if (!I) ce_l = CE;      // transparent while I is low
  assign O = I & ce_l;
endmodule

// regional versions (one clock region); same behaviour in simulation
module BUFH (input wire I, output wire O);
  assign O = I;
endmodule

module BUFHCE #(
  parameter       CE_TYPE  = "SYNC",
  parameter [0:0] INIT_OUT = 1'b0
)(input wire I, input wire CE, output wire O);
  reg ce_l = 1'b0;
  always @(I or CE) if (!I) ce_l = CE;      // SYNC: transparent while I is low
  assign O = I & ce_l;
endmodule

module MMCME2_ADV #(
  parameter          BANDWIDTH          = "OPTIMIZED",
  parameter          COMPENSATION       = "ZHOLD",
  parameter          STARTUP_WAIT       = "FALSE",
  parameter real     CLKIN1_PERIOD      = 5.0,
  parameter integer  DIVCLK_DIVIDE      = 1,
  parameter real     CLKFBOUT_MULT_F    = 6.0,
  parameter real     CLKFBOUT_PHASE     = 0.0,
  parameter real     CLKOUT0_DIVIDE_F   = 6.0,
  parameter integer  CLKOUT1_DIVIDE     = 3,
  parameter real     CLKOUT1_DUTY_CYCLE = 0.5,
  parameter real     CLKOUT1_PHASE      = 0.0,
  parameter real     SIM_LOCK_NS        = 500.0
)(
  input  wire        CLKIN1,
  input  wire        CLKIN2,
  input  wire        CLKINSEL,
  input  wire        CLKFBIN,
  output wire        CLKFBOUT,
  output wire        CLKFBOUTB,
  output wire        CLKOUT0,
  output wire        CLKOUT0B,
  output reg         CLKOUT1,
  output wire        CLKOUT1B,
  output wire        CLKOUT2,
  output wire        CLKOUT2B,
  output wire        CLKOUT3,
  output wire        CLKOUT3B,
  output wire        CLKOUT4,
  output wire        CLKOUT5,
  output wire        CLKOUT6,
  output reg         LOCKED,
  output wire        CLKINSTOPPED,
  output wire        CLKFBSTOPPED,
  input  wire        RST,
  input  wire        PWRDWN,
  input  wire        DCLK,
  input  wire        DEN,
  input  wire        DWE,
  input  wire [6:0]  DADDR,
  input  wire [15:0] DI,
  output reg  [15:0] DO,
  output reg         DRDY,
  input  wire        PSCLK,
  input  wire        PSEN,
  input  wire        PSINCDEC,
  output wire        PSDONE
);

  assign CLKFBOUT = 1'b0;  assign CLKFBOUTB = 1'b1;
  assign CLKOUT0  = 1'b0;  assign CLKOUT0B  = 1'b1;  assign CLKOUT1B = ~CLKOUT1;
  assign CLKOUT2  = 1'b0;  assign CLKOUT2B  = 1'b1;
  assign CLKOUT3  = 1'b0;  assign CLKOUT3B  = 1'b1;
  assign CLKOUT4  = 1'b0;  assign CLKOUT5   = 1'b0;  assign CLKOUT6  = 1'b0;
  assign CLKINSTOPPED = 1'b0;  assign CLKFBSTOPPED = 1'b0;  assign PSDONE = 1'b0;

  real    vco_ns, h1, hg;
  integer div1;
  reg     run, garb;
  reg [15:0] r [0:127];
  integer gen, i, busy_cnt, nerr;
  reg     pwr_written, any_write;

  task decode_and_check;
    integer hi, lo, d;
    begin
      hi = r[7'h0A][11:6];  if (hi == 0) hi = 64;
      lo = r[7'h0A][5:0];   if (lo == 0) lo = 64;
      d  = r[7'h0B][6] ? 1 : hi + lo;
      if (any_write) begin
        if (!pwr_written) begin
          $display("MMCM-ERROR %m: POWER register 0x28 not written FFFF"); nerr = nerr + 1;
        end
        if (r[7'h0A][12] !== 1'b1 || r[7'h0B][15:10] !== 6'b101010) begin
          $display("MMCM-ERROR %m: reserved bits not preserved (0A=%h 0B=%h)", r[7'h0A], r[7'h0B]); nerr = nerr + 1;
        end
        if (hi != d / 2 || lo != d - d / 2 || r[7'h0B][7] != (d % 2)) begin
          $display("MMCM-ERROR %m: inconsistent divider fields hi=%0d lo=%0d edge=%b", hi, lo, r[7'h0B][7]); nerr = nerr + 1;
        end
        if (r[7'h0A][15:13] != 3'b000 || r[7'h0B][9:8] != 2'b00 || r[7'h0B][5:0] != 6'd0) begin
          $display("MMCM-ERROR %m: phase / MX / delay fields not zero"); nerr = nerr + 1;
        end
      end
      div1 = d;
      h1   = vco_ns * div1 / 2.0;
      hg   = h1 * 0.29;                       // garbage: ~3.4x too fast
    end
  endtask

  initial begin
    nerr   = 0;
    vco_ns = CLKIN1_PERIOD * DIVCLK_DIVIDE / CLKFBOUT_MULT_F;
    for (i = 0; i < 128; i = i + 1) r[i] = 16'h0000;
    div1 = CLKOUT1_DIVIDE;
    r[7'h0A] = {3'b000, 1'b1, 6'd0, 6'd0};
    r[7'h0A][11:6] = div1 / 2;
    r[7'h0A][5:0]  = div1 - div1 / 2;
    r[7'h0B] = {6'b101010, 2'b00, (div1 % 2) ? 1'b1 : 1'b0, 1'b0, 6'd0};
    pwr_written = 1'b0; any_write = 1'b0;
    decode_and_check;
    run = 1'b0; garb = 1'b0; LOCKED = 1'b0; CLKOUT1 = 1'b0;
    DO = 16'h0; DRDY = 1'b0; busy_cnt = 0; gen = 0;
  end

  // ---- reset / lock: one polling process (robust against X->0 edges) ----
  initial begin : lockproc
    real t0;
    #0.1;
    forever begin
      while (RST !== 1'b0) begin run = 1'b0; garb = 1'b0; LOCKED = 1'b0; #1; end
      garb = 1'b1;                             // released: garbage until lock
      t0   = $realtime;
      while (RST === 1'b0 && ($realtime - t0) < SIM_LOCK_NS) #1;
      if (RST === 1'b0) begin
        decode_and_check;
        garb = 1'b0; run = 1'b1; LOCKED = 1'b1;
        wait (RST !== 1'b0);
      end
      run = 1'b0; garb = 1'b0; LOCKED = 1'b0;
    end
  end

  // ---- CLKOUT1 ----
  always begin
    if (run)       begin CLKOUT1 = 1'b1; #(h1); CLKOUT1 = 1'b0; #(h1); end
    else if (garb) begin CLKOUT1 = ~CLKOUT1; #(hg * (1.0 + 0.3 * ($random % 2))); end
    else           begin CLKOUT1 = 1'b0; #(1.0); end
  end

  // ---- DRP ----
  always @(posedge DCLK) begin
    DRDY <= 1'b0;
    if (busy_cnt > 0) begin
      busy_cnt = busy_cnt - 1;
      if (busy_cnt == 0) DRDY <= 1'b1;
    end
    if (DEN) begin
      if (busy_cnt != 0) begin
        $display("MMCM-ERROR %m: DEN while a DRP access is pending"); nerr = nerr + 1;
      end
      busy_cnt = 3;
      if (DWE) begin
        if (!RST) begin
          $display("MMCM-ERROR %m: DRP write while the MMCM is not in reset"); nerr = nerr + 1;
        end
        r[DADDR] = DI;
        any_write = 1'b1;
        if (DADDR == 7'h28 && DI == 16'hFFFF) pwr_written = 1'b1;
      end else begin
        DO <= r[DADDR];
      end
    end
  end

endmodule

module PLLE2_ADV #(
  parameter          BANDWIDTH          = "OPTIMIZED",
  parameter          COMPENSATION       = "ZHOLD",
  parameter          STARTUP_WAIT       = "FALSE",
  parameter real     CLKIN1_PERIOD      = 5.0,
  parameter integer  DIVCLK_DIVIDE      = 1,
  parameter integer  CLKFBOUT_MULT      = 6,
  parameter real     CLKFBOUT_PHASE     = 0.0,
  parameter integer  CLKOUT0_DIVIDE     = 6,
  parameter integer  CLKOUT1_DIVIDE     = 3,
  parameter real     CLKOUT1_DUTY_CYCLE = 0.5,
  parameter real     CLKOUT1_PHASE      = 0.0,
  parameter real     SIM_LOCK_NS        = 400.0
)(
  input  wire        CLKIN1,
  input  wire        CLKIN2,
  input  wire        CLKINSEL,
  input  wire        CLKFBIN,
  output wire        CLKFBOUT,
  output wire        CLKOUT0,
  output reg         CLKOUT1,
  output wire        CLKOUT2,
  output wire        CLKOUT3,
  output wire        CLKOUT4,
  output wire        CLKOUT5,
  output reg         LOCKED,
  input  wire        RST,
  input  wire        PWRDWN,
  input  wire        DCLK,
  input  wire        DEN,
  input  wire        DWE,
  input  wire [6:0]  DADDR,
  input  wire [15:0] DI,
  output reg  [15:0] DO,
  output reg         DRDY
);

  // same behaviour as the MMCME2_ADV model above (PLLE2 uses the same DRP
  // addresses / fields for CLKOUT1 and POWER), slightly faster lock
  assign CLKFBOUT = 1'b0;  assign CLKOUT0 = 1'b0;  assign CLKOUT2 = 1'b0;
  assign CLKOUT3  = 1'b0;  assign CLKOUT4 = 1'b0;  assign CLKOUT5 = 1'b0;

  real    vco_ns, h1, hg;
  integer div1;
  reg     run, garb;
  reg [15:0] r [0:127];
  integer gen, i, busy_cnt, nerr;
  reg     pwr_written, any_write;

  task decode_and_check;
    integer hi, lo, d;
    begin
      hi = r[7'h0A][11:6];  if (hi == 0) hi = 64;
      lo = r[7'h0A][5:0];   if (lo == 0) lo = 64;
      d  = r[7'h0B][6] ? 1 : hi + lo;
      if (any_write) begin
        if (!pwr_written) begin
          $display("PLL-ERROR %m: POWER register 0x28 not written FFFF"); nerr = nerr + 1;
        end
        if (r[7'h0A][12] !== 1'b1 || r[7'h0B][15:10] !== 6'b101010) begin
          $display("PLL-ERROR %m: reserved bits not preserved (0A=%h 0B=%h)", r[7'h0A], r[7'h0B]); nerr = nerr + 1;
        end
        if (hi != d / 2 || lo != d - d / 2 || r[7'h0B][7] != (d % 2)) begin
          $display("PLL-ERROR %m: inconsistent divider fields hi=%0d lo=%0d edge=%b", hi, lo, r[7'h0B][7]); nerr = nerr + 1;
        end
        if (r[7'h0A][15:13] != 3'b000 || r[7'h0B][9:8] != 2'b00 || r[7'h0B][5:0] != 6'd0) begin
          $display("PLL-ERROR %m: phase / MX / delay fields not zero"); nerr = nerr + 1;
        end
      end
      div1 = d;
      h1   = vco_ns * div1 / 2.0;
      hg   = h1 * 0.29;                       // garbage: ~3.4x too fast
    end
  endtask

  initial begin
    nerr   = 0;
    vco_ns = CLKIN1_PERIOD * DIVCLK_DIVIDE / CLKFBOUT_MULT;
    for (i = 0; i < 128; i = i + 1) r[i] = 16'h0000;
    div1 = CLKOUT1_DIVIDE;
    r[7'h0A] = {3'b000, 1'b1, 6'd0, 6'd0};
    r[7'h0A][11:6] = div1 / 2;
    r[7'h0A][5:0]  = div1 - div1 / 2;
    r[7'h0B] = {6'b101010, 2'b00, (div1 % 2) ? 1'b1 : 1'b0, 1'b0, 6'd0};
    pwr_written = 1'b0; any_write = 1'b0;
    decode_and_check;
    run = 1'b0; garb = 1'b0; LOCKED = 1'b0; CLKOUT1 = 1'b0;
    DO = 16'h0; DRDY = 1'b0; busy_cnt = 0; gen = 0;
  end

  // ---- reset / lock: one polling process (robust against X->0 edges) ----
  initial begin : lockproc
    real t0;
    #0.1;
    forever begin
      while (RST !== 1'b0) begin run = 1'b0; garb = 1'b0; LOCKED = 1'b0; #1; end
      garb = 1'b1;                             // released: garbage until lock
      t0   = $realtime;
      while (RST === 1'b0 && ($realtime - t0) < SIM_LOCK_NS) #1;
      if (RST === 1'b0) begin
        decode_and_check;
        garb = 1'b0; run = 1'b1; LOCKED = 1'b1;
        wait (RST !== 1'b0);
      end
      run = 1'b0; garb = 1'b0; LOCKED = 1'b0;
    end
  end

  // ---- CLKOUT1 ----
  always begin
    if (run)       begin CLKOUT1 = 1'b1; #(h1); CLKOUT1 = 1'b0; #(h1); end
    else if (garb) begin CLKOUT1 = ~CLKOUT1; #(hg * (1.0 + 0.3 * ($random % 2))); end
    else           begin CLKOUT1 = 1'b0; #(1.0); end
  end

  // ---- DRP ----
  always @(posedge DCLK) begin
    DRDY <= 1'b0;
    if (busy_cnt > 0) begin
      busy_cnt = busy_cnt - 1;
      if (busy_cnt == 0) DRDY <= 1'b1;
    end
    if (DEN) begin
      if (busy_cnt != 0) begin
        $display("PLL-ERROR %m: DEN while a DRP access is pending"); nerr = nerr + 1;
      end
      busy_cnt = 3;
      if (DWE) begin
        if (!RST) begin
          $display("PLL-ERROR %m: DRP write while the MMCM is not in reset"); nerr = nerr + 1;
        end
        r[DADDR] = DI;
        any_write = 1'b1;
        if (DADDR == 7'h28 && DI == 16'hFFFF) pwr_written = 1'b1;
      end else begin
        DO <= r[DADDR];
      end
    end
  end

endmodule
