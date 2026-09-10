`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// nes_ppusplit_capture -- per-frame PPU raster-split (CMD_PPU_SPLITS, opcode
// 0x16) capture, protocol Fase 3.
//
// ⚠️ NO LONGER A STRUCTURAL MIRROR OF ITS SIBLINGS, in two ways, and both are
// deliberate: (1) K = 3 instead of 4 and the payload is stored in 5 bits (area
// -- see the notes below); (2) it REGISTERS ITS INPUTS before comparing
// anything, which the others do not.  (2) was forced by the fit: consumed
// combinationally, this module owned the eleven worst setup paths of the whole
// device at -1.438 ns.  If a sibling ever shows up in that position, this is
// the pattern to copy -- but check the cone first, because the one that DID
// appear next (nes_chrwin_capture at -0.936) is violating on an INTERNAL
// reg-to-reg path (its eviction tree over the 64-bit vector), which an input
// pipeline would not touch.
//
// Originally a mirror of nes_chrsplit_capture.v
// (which is itself the mirror of nes_split_capture.v): same reset/frame_tick
// reseed, same entry-0 seed at the first rendering ce, same <=1-scanline
// coalescence CHAIN, same shortest-strip eviction with entry 0 NEVER evicted.
// Only the payload differs (one COMPOSED byte instead of a CHR slot-0 bank),
// plus the two deltas documented below (NO w-gating, NO poison).
//
// WHY IT EXISTS
// -------------------------------------------------------------------------------
//   Two PPU quantities the protocol publishes ONCE per frame are routinely
//   changed MID-DISPLAY, and both are sampled at the worst possible instant:
//     * the nametable ARRANGEMENT (FRAME_HDR.flags[5:4]), sampled at the frame
//       CLOSE -- Dragon Buster (mapper 95) runs 702/908 gameplay frames with
//       (display start, close) = (1A, 1B), so the renderer draws the WHOLE
//       frame with the status-bar page and the map disappears;
//     * PPUCTRL bit 4, the BG pattern table (CMD_REGS.ppuctrl), sampled
//       mid-display at scanline 120 -- Dudes with Attitude flips it at 119,
//       Bee 52 at 53, Fantastic Dizzy twice (144 and 235).
//   This is the v1.2 "the frame-close snapshot grabs the HUD" bug in two more
//   dimensions.  This module captures WHERE each changed so the renderer can
//   raster-split BG1SC ($2107, HDMA ch5) and the BG tile base ($210B, the ch3
//   table that already exists).
//
// COMMAND FORMAT (serialized by nes_bridge.v, S_PSP_*):
//   0x16 | hdr(1: bit7=overflow, bits[3:0]=cnt 1..4) | cnt x [scanline(1) payload(1)]
//   payload = {3'b0, ppuctrl[4], ntcode[3:0]}
//   Entry 0 is the state at DISPLAY START; every entry is valid until the next
//   entry's scanline (the last one until scanline 240).
//
// THE DETECTION MASK IS {ppuctrl[4], ntcode}, NOT THE RAW $2000 BYTE
// -------------------------------------------------------------------------------
//   $2000 is REWRITTEN EVERY FRAME by the scroll code (bits [1:0] are the
//   nametable select half of the $2005/$2006 dance).  Comparing the whole
//   PPUCTRL byte would emit one entry per scroll write and blow K=4 open on
//   every scrolling game.  Only the two quantities the renderer can actually
//   act on per strip are in the mask.  (Same class of hole as the fast-skip
//   $15 measurement bug -- it cost two measurement rounds there.)
//
// ppuctrl[3] AND ppuctrl[5] ARE OUT OF THE PAYLOAD AND OUT OF THE MASK, ON
// PURPOSE -- this is a decision, not an oversight:
//   * bit 3 (OBJ pattern table) would need OBSEL per strip, and $2101's
//     name_base granularity is 8 K-WORD = 16 KB = one whole OBJ region, and
//     there is no free 16 KB in VRAM;
//   * bit 5 (8x16 sprites) is decided PER SPRITE during OAM conversion.
//   Including either would only manufacture entries capable of EVICTING the
//   useful ones.
//
// NO w-GATING (delta vs nes_split_capture) -- deliberate, do not "fix":
//   The scroll capture gates on ppu_tap_loopy_w==0 because loopy_T is written
//   by a TWO-WRITE $2005/$2006 pair and is observable in an INTERMEDIATE state
//   between the two.  Neither source here has such a window: $2000 is a
//   SINGLE-write register (ppu.v latches tap_ppuctrl from ppuctrl_full), and
//   the mapper's nt_snap_code tap is REGISTERED under `ce` inside MultiMapper.
//   Both therefore only ever present COMMITTED values.
//
// NO POISON (delta vs nes_chrsplit_capture) -- also deliberate:
//   The CHR capture poisons a frame whose 8K<->4K mode flips mid-display,
//   because that changes the MEANING of "slot 0 bank" for the strips already
//   captured.  Every composition of {b4, ntcode} is interpretable on its own
//   (the renderer resolves each strip independently, and a non-classic code
//   falls back through legacy_of() to the nearest classic arrangement), so
//   there is no state that can invalidate the list.  Same reasoning as the
//   $15 window-vector capture, which has no poison either.
//
// THE COALESCENCE CHAIN IS LOAD-BEARING, not an optimisation.  Mapper 95
// passes through code 0x3 (H with the pages swapped) as a TRANSIENT between
// the write of R0 and the write of R1 -- 6.319 such writes in the Dragon
// Buster trace.  The <=1-scanline chain absorbs the whole burst and cnt stays
// 2; without it every one of those frames would be cnt 3 with a phantom strip.
//
// cnt == 1 IS LEGAL HERE, AND IT HAPPENS.  This is the ONE place this command
// differs from its siblings $11/$13/$15, and it is in the protocol on purpose:
// the bridge's emission gate is
//        emit  <=>  cnt >= 2  OR  entry0's code is NOT classic
// because flags[5:4] only has two bits, so a frame whose code is non-classic
// but STATIC has no other channel.  Measured: l3_finallap (Namco 163) emits
// cnt==1 with code 0x2 in 1497/1500 frames.  Every opcode walker that assumed
// cnt>=2 had to be taught this.
//
// K = 3, ONE LESS THAN ITS SIBLINGS -- lockstep with bridge_sim/mailbox.py
// PPUSPLIT_MAX.  Two facts justify it and both are measured, not assumed:
//   * across the 149-trace corpus cnt NEVER exceeds 3 and ovf is 0 everywhere,
//     i.e. no frame ever accepted a FOURTH change, so dropping the ceiling from
//     4 to 3 leaves every golden BYTE-IDENTICAL (proven by a full re-simulation
//     of all 149, not by the argument alone);
//   * area.  The K=4 version measured +350 LEs on a device the Lote 3 already
//     leaves at 95 %.  K=3 removes one entry AND one level of the eviction
//     compare tree.
// The eviction BEHAVIOUR is unchanged (shortest strip, entry 0 never evicted,
// tie -> lowest index, victim == the new one -> dropped without advancing the
// chain); only the ceiling moved.
//
// THE PAYLOAD IS STORED IN 5 BITS, not 8.  The serialized byte is
// {3'b000, ppuctrl[4], ntcode[3:0]} and bits 7:5 are protocol padding, so
// keeping them in the registers only bought three constant-zero FFs per entry
// (Quartus reported them as "Stuck at GND", which is how they were found).  The
// bridge widens back to 8 at emission.
//
// COST: ~77 FFs (the 17-FF input pipeline stage + 3x8 scanline + 3x5 payload + cnt/ovf/frozen + the 8+5 "last
// change" registers), ZERO M9K.  The change test is a 5-bit compare against a
// single register (never a scan over the array); array writes are 3-deep
// decodes/shifts; the eviction cone is consumed ONLY in the cnt==3 branch.
// Mirror of the bridge_sim ppustate._ppu_capture_tick bookkeeping.
//////////////////////////////////////////////////////////////////////////////////

module nes_ppusplit_capture(
  input         CLK,
  input         RST,
  input         ce,              // core tick (nes_wrap ce_pulse_r)
  input         frame_tick,      // frame close (nes_wrap frame_tick_r)
  input  [8:0]  scanline,        // core scanline
  input  [3:0]  ntcode,          // nt_snap_code (registered under ce in mmu.v)
  input  [7:0]  ppuctrl,         // ppu_tap_ppuctrl (registered in ppu.v)
  input  [7:0]  ppumask,         // ppu_tap_ppumask
  output [2:0]  psp_cnt_o,
  output        psp_ovf_o,
  output        psp_frozen_o,    // entry 0 was captured with rendering ON
  output [23:0] psp_sl_flat,     // 3 x scanline[7:0]  (window is 0..239 -> 8 bits)
  output [14:0] psp_pay_flat     // 3 x payload[4:0]   (bits 7:5 are padding)
);
  reg [7:0]  psp_sl [0:2];
  reg [4:0]  psp_py [0:2];
  reg [2:0]  psp_cnt;
  reg        psp_ovf;
  reg        psp_frozen;
  reg [7:0]  psp_last_sl;    // scanline of the LAST CHANGE (coalesce chain)
  reg [4:0]  psp_last_py;

  // ---- INPUT PIPELINE STAGE (timing; SEMANTICALLY A NO-OP) -----------------
  // Every input is re-registered once before ANY comparison, and the whole body
  // below reads only the _q copies.  This is not a style choice, it is what the
  // fit demanded: with the inputs consumed combinationally, the ELEVEN WORST
  // setup paths of the whole device were
  //     PPU|ClockGen|scanline[0] -> psp_cap|psp_last_py[*] / psp_sl[2][*]
  // at -1.438 ns, and none of 12 seeds closed at 85 C.  The cone was
  // {scanline -> 8-bit subtract -> deadband/coalescence compare -> the array
  // write enables} plus the payload compare, all inside one CLK2 of 11.9 ns,
  // with the source on the far side of the chip inside the PPU.
  //
  // WHY IT IS EXACT, and not "close enough": every input here moves ONLY on a
  // core `ce`, and the pacer guarantees >= 13 CLK2 between two of them
  // (nes_wrap.v header).  Delaying `ce` by the SAME one cycle as the data keeps
  // each pair together, so the automaton observes exactly the values it
  // observed before, one CLK2 later.  Delaying `frame_tick` with them keeps the
  // reseed paired with the payload it must capture -- and it also removes a
  // race the undelayed version had, because the bridge latches this module's
  // outputs on the UNDELAYED frame_tick, i.e. now strictly BEFORE the reseed
  // rewrites them.  The byte-exact gate over all 149 traces is the proof: a
  // one-scanline shift would show up in it immediately (the deadband is +-1).
  //
  // Composed BEFORE the register on purpose: the payload and the rendering gate
  // are a concat and an OR of bits already registered in ppu.v/mmu.v, so
  // putting them in front costs no depth and saves 8 flops against registering
  // ppuctrl+ppumask+ntcode whole.  Cost: 17 FFs.
  wire [4:0] psp_cur_w  = {ppuctrl[4], ntcode};
  wire       psp_rend_w = ppumask[3] | ppumask[4];
  reg        q_ce, q_ft, q_rend;
  reg [8:0]  q_sl;
  reg [4:0]  q_cur;
  always @(posedge CLK) begin
    if (RST) begin
      q_ce<=1'b0; q_ft<=1'b0; q_rend<=1'b0; q_sl<=9'd0; q_cur<=5'd0;
    end else begin
      q_ce<=ce; q_ft<=frame_tick; q_rend<=psp_rend_w;
      q_sl<=scanline; q_cur<=psp_cur_w;
    end
  end

  // THE payload, 5 bits.  See the mask note in the header: bits 1:0/3/5 of
  // $2000 are out of it on purpose, so a scroll write can never look like a
  // split; and bits 7:5 of the SERIALIZED byte are protocol padding, so they
  // are not stored (the bridge re-adds them).
  wire [4:0] psp_cur = q_cur;
  wire psp_changed = (psp_cur != psp_last_py);
  // RENDERING GATE (ppumask BG|OBJ): an arrangement/pattern change while
  // rendering is OFF is not a visible split (games re-point freely in vblank /
  // forced blank; the frame-close FRAME_HDR/CMD_REGS already carry that).
  // Keeps non-splitting games at cnt<=1 so their byte stream never changes.
  wire psp_render = q_rend;
  // No w-gating: both sources are committed+registered (see the header).
  wire psp_do_change = psp_changed;
  // eviction cones (consumed ONLY in the cnt==4 overflow branch; reg-to-reg,
  // 8-bit subtract + compare tree, single-cycle at CLK2 with room to spare)
  wire [7:0] psp_sl_now = q_sl[7:0];
  wire [7:0] psp_d1 = psp_sl[2]  - psp_sl[1];   // strip of entry 1
  wire [7:0] psp_d2 = psp_sl_now - psp_sl[2];   // strip of entry 2
  wire [7:0] psp_dn = 8'd240     - psp_sl_now;  // strip the NEW one would get
  wire psp_ev1 = (psp_d1 <= psp_d2) && (psp_d1 <= psp_dn);
  wire psp_ev2 = !psp_ev1 && (psp_d2 <= psp_dn);
  always @(posedge CLK) begin
    if (RST) begin
      psp_cnt<=3'd1; psp_ovf<=1'b0; psp_frozen<=1'b0;
      psp_sl[0]<=8'd0; psp_py[0]<=5'd0;
      psp_last_sl<=8'd0; psp_last_py<=5'd0;
    end else if (q_ft) begin
      // re-seed entry0 (fallback = close-time payload) + re-arm for next frame.
      // NOTE the bridge latches the PRE-reseed array on this same edge, and
      // when psp_frozen==0 it OVERRIDES entry 0 with the live close-time
      // payload -- see the l_psp_pay0 note in nes_bridge.v (spec SS2.2).
      psp_frozen<=1'b0; psp_ovf<=1'b0; psp_cnt<=3'd1;
      psp_sl[0]<=8'd0; psp_py[0]<=psp_cur;
      psp_last_sl<=8'd0; psp_last_py<=psp_cur;
    end else if (q_ce && q_sl <= 9'd239 && psp_render) begin
      if (!psp_frozen) begin
        // entry 0 = display-start state (chain anchored at scanline 0)
        psp_sl[0]<=8'd0; psp_py[0]<=psp_cur;
        psp_cnt<=3'd1; psp_frozen<=1'b1;
        psp_last_sl<=8'd0; psp_last_py<=psp_cur;
      end else if (psp_do_change) begin
        if ((psp_sl_now - psp_last_sl) <= 8'd1) begin
          // coalesce (same/adjacent scanline): entry keeps its ORIGINAL
          // scanline; the chain advances so a burst may continue next line.
          // This is the branch that swallows the mapper-95 0x3 transient.
          psp_py[psp_cnt-3'd1]<=psp_cur;
          psp_last_sl<=psp_sl_now; psp_last_py<=psp_cur;
        end else if (psp_cnt >= 3'd3) begin
          psp_ovf<=1'b1;
          if (psp_ev1) begin        // evict e1: shift e2 down, new at [2]
            psp_sl[1]<=psp_sl[2]; psp_py[1]<=psp_py[2];
            psp_sl[2]<=psp_sl_now; psp_py[2]<=psp_cur;
            psp_last_sl<=psp_sl_now; psp_last_py<=psp_cur;
          end else if (psp_ev2) begin
            psp_sl[2]<=psp_sl_now; psp_py[2]<=psp_cur;
            psp_last_sl<=psp_sl_now; psp_last_py<=psp_cur;
          end
          // else: the new entry is the shortest strip -> dropped, chain frozen
        end else begin
          psp_sl[psp_cnt]<=psp_sl_now;
          psp_py[psp_cnt]<=psp_cur;
          psp_cnt<=psp_cnt+3'd1;
          psp_last_sl<=psp_sl_now; psp_last_py<=psp_cur;
        end
      end
    end
  end
  assign psp_sl_flat  = {psp_sl[2],psp_sl[1],psp_sl[0]};
  assign psp_pay_flat = {psp_py[2],psp_py[1],psp_py[0]};
  assign psp_cnt_o    = psp_cnt;
  assign psp_ovf_o    = psp_ovf;
  assign psp_frozen_o = psp_frozen;
endmodule
