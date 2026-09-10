`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// nes_chrsplit_capture -- per-frame CHR raster-split (CMD_CHR_SPLITS, opcode
// 0x13) capture, protocol v2.3.  STRUCTURAL MIRROR of nes_split_capture.v (the
// scroll CMD_SPLITS capture): same reset/frame_tick reseed, same entry-0 seed at
// the first rendering ce, same <=1-scanline coalescence CHAIN, same
// shortest-strip eviction with entry 0 NEVER evicted.  Only the payload differs
// (one CHR slot-0 bank byte instead of loopy_T/fine_x) plus the two deltas
// documented below (NO w-gating, NEW poison valve).
//
// WHY IT EXISTS
// -------------------------------------------------------------------------------
//   CMD_CHR_STATE (0x12) publishes the ABSOLUTE CHR-bank state ONCE per frame.
//   Games that swap the slot-0 4K bank MID-DISPLAY every frame (RoboCop 2, MMC1
//   4K: HUD scanlines 0-31 from bank 2, playfield from bank 0; title screen
//   splits at scanline 119) are not representable by one state per frame -- the
//   renderer draws the whole frame from whichever bank happened to be live at
//   the close, and the other region is garbled.  This module captures WHERE the
//   bank changed so the renderer can raster-split CHR via HDMA $210B.
//
// COMMAND FORMAT (serialized by nes_bridge.v, S_CSPL_*):
//   0x13 | hdr(1: bit7=overflow, bits[3:0]=cnt 2..4) | cnt x [scanline(1) bank(1)]
//   Entry 0 is the slot-0 bank at DISPLAY START; every entry is valid until the
//   next entry's scanline (the last one until scanline 240).
//
// ADDITIVE GATE (byte-identical goldens): the bridge emits the command ONLY when
//   cnt>=2 AND !poison.  A frame with no mid-display bank change leaves cnt==1
//   and NOTHING is emitted, so the existing per-frame byte stream of every
//   non-splitting game/frame is unchanged.
//
// NO w-GATING (delta vs nes_split_capture) -- deliberate, do not "fix":
//   The scroll capture gates its change detection on ppu_tap_loopy_w==0 because
//   loopy_T is written by a TWO-WRITE $2005/$2006 pair and is observable in an
//   INTERMEDIATE state between the two writes (sampling it emitted phantom
//   entries).  The CHR bank has no such window: MMC1's serial port commits
//   atomically on the FIFTH write (mmu.v ~l.133-136: the shift register only
//   drives `chr_bank_0 <= {prg_din[0], shift[4:1]}` in the `if (shift[0])`
//   branch -- the four preceding writes only shift), and the tap this module
//   consumes is itself REGISTERED under `ce` inside MultiMapper (mmu.v
//   ~l.464-475).  s0_bank therefore only ever presents COMMITTED values; there
//   is no intermediate state to gate away.  Mapper28-family CHR (a53chr) is a
//   single-write register -- atomic by construction.
//
// POISON (new here, no counterpart in the scroll capture):
//   A mid-frame change of `s1_present` (MMC1 8K<->4K CHR mode) invalidates the
//   very meaning of "slot 0 bank" for the strips already captured (in 8K mode
//   slot 0 covers both pattern tables).  Rather than ship a split list the
//   renderer cannot interpret, the frame is POISONED: capture is frozen for the
//   rest of the frame, cnt falls back to 1 and ovf is cleared, and the bridge
//   suppresses the command entirely (its gate is cnt>=2 && !poison).  The test
//   runs on every display-window ce AFTER the seed, REGARDLESS of the rendering
//   gate: a mode switch performed inside a mid-frame forced blank that never
//   re-enables rendering this frame must also poison (the entries were captured
//   in the old mode while the close-time CMD_CHR_STATE carries the new one).
//   Pre-seed changes never poison -- the seed re-baselines `last_s1p`.
//   Correctness valve only -- measured 0 occurrences across the trace corpus.
//   While poisoned, cnt_o/ovf_o are unobservable downstream by construction.
//   Cleared at frame_tick, like every other per-frame register here.
//
// COST: ~90 FFs (4x8 scanline + 4x8 bank + cnt/ovf/frozen/poison + the 8+8+1
// "last change" registers), ZERO M9K.  The change test is an 8-bit compare
// against a single register (never a scan over the array); array writes are
// 4-deep decodes/shifts; the eviction cone is consumed ONLY in the cnt==4
// branch.  Mirror of the bridge_sim chrsplit bookkeeping.
//////////////////////////////////////////////////////////////////////////////////

module nes_chrsplit_capture(
  input         CLK,
  input         RST,
  input         ce,              // core tick (nes_wrap ce_pulse_r)
  input         frame_tick,      // frame close (nes_wrap frame_tick_r)
  input  [8:0]  scanline,        // core scanline
  input  [7:0]  s0_bank,         // chr_snap_s0_bank (registered under ce in mmu.v)
  input  [7:0]  s1_bank,         // chr_snap_s1_bank  (same tap the 0x12 uses)
  input         s1_present,      // chr_snap_s1_present (8K=0 / 4K=1)
  input  [7:0]  ppumask,         // ppu_tap_ppumask
  output [2:0]  cspl_cnt_o,
  output        cspl_ovf_o,
  output        cspl_poison_o,
  output [31:0] cspl_sl_flat,    // 4 x scanline[7:0]  (window is 0..239 -> 8 bits)
  output [63:0] cspl_bank_flat   // 4 x {s1_eff[7:0], s0_bank[7:0]}
);
  reg [7:0]  cspl_sl [0:3];
  reg [15:0] cspl_bk [0:3];
  reg [2:0]  cspl_cnt;
  reg        cspl_ovf;
  reg        cspl_frozen;
  reg        cspl_poison;
  reg [7:0]  cspl_last_sl;    // scanline of the LAST CHANGE (coalesce chain)
  reg [15:0] cspl_last_bk;
  reg        cspl_last_s1p;   // 8K/4K mode baseline for the poison test
  // ---- THE KEY IS THE PAIR, not slot 0 alone ----------------------------
  // A game that raster-splits on the SLOT 1 half was invisible: the change test
  // only looked at slot 0, so nothing was ever captured and no 0x13 was emitted
  // for the frame.  Measured: Goemon 1 gains a 0x13 in 561 frames of its
  // gameplay trace (frame 170 = [(0, 00, 19), (42, 00, 01)] -- s0 IDENTICAL in
  // both entries), and jajaninpou goes from 5 frames with a 0x13 to 665.
  // s1 is gated EXACTLY like the 0x12's payload (`s1_present ? s1_bank : 0`),
  // so a 0x13 entry and a CMD_CHR_STATE payload are the same tuple -- lockstep
  // with bridge_sim's note_chr_state (`_chr_s1 = s1_bank if s1_present else 0`).
  wire [7:0]  s1_eff    = s1_present ? s1_bank : 8'd0;
  wire [15:0] cspl_cur  = {s1_eff, s0_bank};
  wire cspl_changed = (cspl_cur != cspl_last_bk);
  wire cspl_s1p_chg = (s1_present != cspl_last_s1p);
  // RENDERING GATE (ppumask BG|OBJ): a bank change while rendering is OFF is not
  // a visible CHR split (games re-bank freely in vblank / forced blank; the
  // frame-close CMD_CHR_STATE already carries that).  Keeps non-splitting games
  // at cnt<=1 so their byte stream never changes.
  wire cspl_render = ppumask[3] | ppumask[4];
  // No w-gating: the tap is committed+registered (see the header).
  wire cspl_do_change = cspl_changed;
  // eviction cones (consumed ONLY in the cnt==4 overflow branch; reg-to-reg,
  // 8-bit subtract + compare tree, single-cycle at CLK2 with room to spare)
  wire [7:0] cspl_sl_now = scanline[7:0];
  wire [7:0] cspl_d1 = cspl_sl[2] - cspl_sl[1];
  wire [7:0] cspl_d2 = cspl_sl[3] - cspl_sl[2];
  wire [7:0] cspl_d3 = cspl_sl_now - cspl_sl[3];
  wire [7:0] cspl_dn = 8'd240 - cspl_sl_now;
  wire cspl_ev1 = (cspl_d1 <= cspl_d2) && (cspl_d1 <= cspl_d3) && (cspl_d1 <= cspl_dn);
  wire cspl_ev2 = !cspl_ev1 && (cspl_d2 <= cspl_d3) && (cspl_d2 <= cspl_dn);
  wire cspl_ev3 = !cspl_ev1 && !cspl_ev2 && (cspl_d3 <= cspl_dn);
  // ---- REGISTERED DECISION (timing; a 1-CYCLE DEFERRAL, semantically exact) -
  // Same shape as nes_split_capture / nes_chrwin_capture: the change compare,
  // the <=1-scanline chain test and the eviction tree pick an ACTION in the ce
  // cycle; the arrays move the cycle after.  The narrow state (cspl_cnt,
  // cspl_ovf, cspl_frozen, cspl_poison and the cspl_last_* chain) still updates
  // in cycle 1 because it feeds the next comparison.  The apply is placed FIRST
  // so a frame close landing on a pending write comes out in the same order the
  // immediate module would produce.
  localparam [2:0] CACT_WR   = 3'd1,
                   CACT_COAL = 3'd2,
                   CACT_EV1  = 3'd3,
                   CACT_EV2  = 3'd4,
                   CACT_EV3  = 3'd5;
  reg        cdp_v;
  reg [2:0]  cdp_act;
  reg [2:0]  cdp_idx;
  reg [7:0]  cdp_sl;
  reg [15:0] cdp_bk;

  always @(posedge CLK) begin
    // ---- cycle 2 of every array update: apply the parked decision -----------
    if (cdp_v & ~RST) begin
      cdp_v <= 1'b0;
      case (cdp_act)
        CACT_WR:   begin cspl_sl[cdp_idx[1:0]]<=cdp_sl; cspl_bk[cdp_idx[1:0]]<=cdp_bk; end
        CACT_COAL: begin cspl_bk[cdp_idx[1:0]]<=cdp_bk; end
        CACT_EV1: begin
          cspl_sl[1]<=cspl_sl[2]; cspl_bk[1]<=cspl_bk[2];
          cspl_sl[2]<=cspl_sl[3]; cspl_bk[2]<=cspl_bk[3];
          cspl_sl[3]<=cdp_sl;     cspl_bk[3]<=cdp_bk;
        end
        CACT_EV2: begin
          cspl_sl[2]<=cspl_sl[3]; cspl_bk[2]<=cspl_bk[3];
          cspl_sl[3]<=cdp_sl;     cspl_bk[3]<=cdp_bk;
        end
        default: begin
          cspl_sl[3]<=cdp_sl;     cspl_bk[3]<=cdp_bk;
        end
      endcase
    end

    if (RST) begin
      cspl_cnt<=3'd1; cspl_ovf<=1'b0; cspl_frozen<=1'b0; cspl_poison<=1'b0;
      cspl_sl[0]<=8'd0; cspl_bk[0]<=16'd0;
      cspl_last_sl<=8'd0; cspl_last_bk<=16'd0; cspl_last_s1p<=1'b0;
      cdp_v<=1'b0; cdp_act<=3'd0; cdp_idx<=3'd0; cdp_sl<=8'd0; cdp_bk<=16'd0;
    end else if (frame_tick) begin
      // re-seed entry0 (fallback = close-time bank) + re-arm capture for next frame
      cspl_frozen<=1'b0; cspl_ovf<=1'b0; cspl_cnt<=3'd1; cspl_poison<=1'b0;
      cspl_sl[0]<=8'd0; cspl_bk[0]<=cspl_cur;
      cspl_last_sl<=8'd0; cspl_last_bk<=cspl_cur; cspl_last_s1p<=s1_present;
    end else if (ce && scanline <= 9'd239 && !cspl_poison) begin
      if (cspl_frozen && cspl_s1p_chg) begin
        // 8K<->4K mid-display: the captured strips lose their meaning -> drop the
        // whole frame's list (the bridge gates emission on !poison).  Tested
        // OUTSIDE the rendering gate on purpose: a mode switch performed during a
        // MID-FRAME blank that never re-enables rendering this frame would
        // otherwise slip through (entries captured in the old mode + close-state
        // CMD_CHR_STATE in the new one = uninterpretable list).  Pre-seed
        // (cspl_frozen==0) changes never poison -- the seed re-baselines.
        cspl_poison<=1'b1; cspl_cnt<=3'd1; cspl_ovf<=1'b0;
      end else if (cspl_render) begin
      if (!cspl_frozen) begin
        // entry 0 = display-start state (chain anchored at scanline 0)
        cdp_v<=1'b1; cdp_act<=CACT_WR; cdp_idx<=3'd0;
        cdp_sl<=8'd0; cdp_bk<=cspl_cur;
        cspl_cnt<=3'd1; cspl_frozen<=1'b1;
        cspl_last_sl<=8'd0; cspl_last_bk<=cspl_cur; cspl_last_s1p<=s1_present;
      end else if (cspl_do_change) begin
        if ((cspl_sl_now - cspl_last_sl) <= 8'd1) begin
          // coalesce (same/adjacent scanline): entry keeps its ORIGINAL
          // scanline; the chain advances so a burst may continue next line
          cdp_v<=1'b1; cdp_act<=CACT_COAL; cdp_idx<=cspl_cnt-3'd1;
          cdp_sl<=cspl_sl_now; cdp_bk<=cspl_cur;
          cspl_last_sl<=cspl_sl_now; cspl_last_bk<=cspl_cur;
        end else if (cspl_cnt >= 3'd4) begin
          cspl_ovf<=1'b1;
          if (cspl_ev1 | cspl_ev2 | cspl_ev3) begin
            cdp_v<=1'b1;
            cdp_act<=cspl_ev1 ? CACT_EV1 : (cspl_ev2 ? CACT_EV2 : CACT_EV3);
            cdp_idx<=3'd3;
            cdp_sl<=cspl_sl_now; cdp_bk<=cspl_cur;
            cspl_last_sl<=cspl_sl_now; cspl_last_bk<=cspl_cur;
          end
          // else: the new entry is the shortest strip -> dropped, chain frozen
        end else begin
          cdp_v<=1'b1; cdp_act<=CACT_WR; cdp_idx<=cspl_cnt;
          cdp_sl<=cspl_sl_now; cdp_bk<=cspl_cur;
          cspl_cnt<=cspl_cnt+3'd1;
          cspl_last_sl<=cspl_sl_now; cspl_last_bk<=cspl_cur;
        end
      end
      end // cspl_render
    end
  end
  assign cspl_sl_flat   = {cspl_sl[3],cspl_sl[2],cspl_sl[1],cspl_sl[0]};
  assign cspl_bank_flat = {cspl_bk[3],cspl_bk[2],cspl_bk[1],cspl_bk[0]};
  assign cspl_cnt_o     = cspl_cnt;
  assign cspl_ovf_o     = cspl_ovf;
  assign cspl_poison_o  = cspl_poison;
endmodule
