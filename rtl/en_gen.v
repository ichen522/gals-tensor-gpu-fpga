// en_gen.v
//
// Per-tile enable generator ("the small FSM" from the 9/28 meeting) plus
// the per-window activity counter. One instance per tile, running on that
// tile's own clock.
//
// ---- what it does ------------------------------------------------------
// The professor's method needs every 1 ms sample window to contain a
// DIFFERENT, KNOWN amount of tile work (so P_k = P_static + W*N_k can be
// solved for P_static and W), while keeping the chip's temperature flat
// (average activity constant, e.g. 90%). The clock frequency stays fixed;
// instead, individual clock cycles are skipped by pulling `en` low.
//
//   duty pattern : "turn off OFF cycles out of every DUTY_DEN", spread as
//                  evenly as possible. OFF comes from the duty sequence in
//                  win_ctrl.v and changes once per window.
//                    OFF = 1  -> 1 off per 100   -> 99%
//                    OFF = 10 -> every 10th off  -> 90%
//                    OFF = 20 -> every 5th off   -> 80%
//                    OFF = 3  -> 3 per 100, ~every 33rd -> 97%
//                  This is exactly the professor's "1 off every N"
//                  (OFF = 100/N), generalized so that values like 97% or
//                  89% that don't divide evenly are still exact.
//   activity     : activity_count counts every cycle `en` was high (the
//                  same counter OP_READ_ACT always read); win_cnt counts
//                  the same thing but restarts at every window boundary,
//                  and nk_hold keeps the finished window's total (N_k) so
//                  win_ctrl.v can log it.
//
// ---- how the spreading works (first-order sigma-delta / Bresenham) -----
//   each cycle:  if (acc >= DUTY_DEN-OFF):  acc -= DUTY_DEN-OFF ; en = 0
//                else:                      acc += OFF          ; en = 1
// acc always stays in [0, DUTY_DEN-1], and over any DUTY_DEN consecutive
// cycles exactly OFF of them are off. OFF=10 gives 1,1,1,1,1,1,1,1,1,0
// repeating -- identical to a "count to 10, drop the last one" counter.
//
// ---- clock-domain crossings (all from clk_mem / async) ----------------
//   tile_en_in    : DIP switch | host override. 2-flop synchronized.
//   seq_run_in    : level from win_ctrl.v. 2-flop synchronized.
//   win_toggle_in : flips once per window boundary in clk_mem. 2-flop
//                   synchronized + edge detect = `tick` in this domain.
//   duty_off_in   : multi-bit, but quasi-static: win_ctrl.v sets it at
//                   least 64 clk_mem cycles before flipping win_toggle and
//                   holds it at least 64 cycles after, and this module
//                   only samples it on `tick`, which by construction comes
//                   2-3 tile cycles AFTER the toggle -> always stable when
//                   sampled, no per-bit synchronizer needed.
//   nk_hold (out) : updated on `tick`, then held for a whole window;
//                   win_ctrl.v reads it half a window later -> stable.
//
// When seq_run is low, en = tile_en (100% while on) -- identical to the
// pre-v6 behaviour, so the DIP-switch on/off experiment still works.

module en_gen #(
  parameter integer DUTY_DEN = 100,     // OFF is "cycles off per DUTY_DEN"
  parameter integer DUTY_W   = 8
)(
  input  wire              clk,            // this tile's clock
  input  wire              rst_n,          // this tile's reset

  input  wire              tile_en_in,     // async / quasi-static
  input  wire              seq_run_in,     // clk_mem level
  input  wire              win_toggle_in,  // clk_mem toggle
  input  wire [DUTY_W-1:0] duty_off_in,    // quasi-static, see header

  output reg               en,             // registered: this cycle is a working cycle
  output reg  [31:0]       activity_count, // total working cycles since reset (wraps)
  output reg  [31:0]       nk_hold,        // working cycles in the last finished window
  output reg  [DUTY_W-1:0] duty_cur,       // OFF value in effect (debug)
  output wire              tick            // 1-cycle pulse at each window boundary (debug/TB)
);

  localparam integer AW = 9;   // acc width; supports DUTY_DEN <= 256
  localparam [AW-1:0] DEN_V = DUTY_DEN;

  // v7.2: SYNCHRONOUS reset. rst_n already arrives synchronized to clk
  // and is held low >= 64 cycles after the MMCM locks (vc707_top.v), so a
  // registered, fanout-replicated copy replaces the old asynchronous CLR
  // net. At 400 MHz that one net could not reach the ~1,400 flops per
  // tile within 2.5 ns (Vivado "async_default" recovery failures).
  // Pure pipelines (enable / valid chains) have no reset at all any more:
  // they flush with zeros during that 64-cycle reset.
  (* max_fanout = 32 *) reg srst = 1'b1;
  always @(posedge clk) srst <= ~rst_n;

  // ---- synchronizers -------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) reg [1:0] en_s;
  (* ASYNC_REG = "TRUE" *) reg [1:0] run_s;
  (* ASYNC_REG = "TRUE" *) reg [1:0] tog_s;
  reg tog_d;
  always @(posedge clk) begin
    if (srst) begin
      en_s <= 2'b00; run_s <= 2'b00; tog_s <= 2'b00; tog_d <= 1'b0;
    end else begin
      en_s  <= {en_s[0],  tile_en_in};
      run_s <= {run_s[0], seq_run_in};
      tog_s <= {tog_s[0], win_toggle_in};
      tog_d <= tog_s[1];
    end
  end
  assign tick = tog_s[1] ^ tog_d;

  // ---- duty pattern generator ---------------------------------------------
  // v7.2: same Bresenham sequence as before (acc in [0, DUTY_DEN), an OFF
  // cycle whenever acc >= k = DUTY_DEN - OFF), but the register now holds
  // d = acc - k as a signed value, so "acc >= k" is just d's sign bit and
  // both possible next values are computed in parallel:
  //     off cycle (d >= 0): d' = d - k       (acc' = acc - k)
  //     on  cycle (d <  0): d' = d + OFF     (acc' = acc + OFF)
  // The old form (9-bit compare, then add-or-subtract, then mux) was two
  // carry chains in series: -0.33 ns at 400 MHz.
  localparam integer DW = AW + 1;     // signed: d in [-DUTY_DEN, DUTY_DEN)
  wire [AW-1:0] off_clamped = ({{(AW-DUTY_W){1'b0}}, duty_off_in} > DEN_V) ? DEN_V
                                                       : {{(AW-DUTY_W){1'b0}}, duty_off_in};
  reg  [DW-1:0] off_q;     // +OFF for the current window (sign-extended)
  reg  [DW-1:0] negk_q;    // -k = OFF - DUTY_DEN
  reg  [DW-1:0] d;

  wire          drop     = ~d[DW-1];                 // d >= 0  <=>  acc >= k
  wire          want_on  = en_s[1] & (~run_s[1] | ~drop);
  wire [DW-1:0] d_off    = d + negk_q;               // both in parallel,
  wire [DW-1:0] d_on     = d + off_q;                // selected by d's sign

  always @(posedge clk) begin
    if (srst) begin
      off_q    <= {DW{1'b0}};
      negk_q   <= {DW{1'b0}} - {1'b0, DEN_V};
      d        <= {DW{1'b0}} - {1'b0, DEN_V};       // acc = 0, k = DUTY_DEN
      duty_cur <= {DUTY_W{1'b0}};
      en       <= 1'b0;
    end else begin
      en <= want_on;
      if (tick) begin
        off_q    <= {1'b0, off_clamped};
        negk_q   <= {1'b0, off_clamped} - {1'b0, DEN_V};
        d        <= {1'b0, off_clamped} - {1'b0, DEN_V};   // acc = 0 with the new k
        duty_cur <= off_clamped[DUTY_W-1:0];
      end else begin
        d <= drop ? d_off : d_on;
      end
    end
  end

  // ---- activity counters ----------------------------------------------------
  reg [31:0] win_cnt;
  always @(posedge clk) begin
    if (srst) begin
      activity_count <= 32'd0;
      win_cnt        <= 32'd0;
      nk_hold        <= 32'd0;
    end else begin
      activity_count <= activity_count + {31'd0, en};
      if (tick) begin
        nk_hold <= win_cnt + {31'd0, en};   // this cycle still belongs to the old window
        win_cnt <= 32'd0;
      end else begin
        win_cnt <= win_cnt + {31'd0, en};
      end
    end
  end

endmodule
