`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// nes_bridge -- NES->SNES video bridge v1 (Fase 1a).  Golden: the byte-exact
// mailbox command stream produced by nes-tests/bridge-sim (`out/<trace>/mailbox.bin`).
//
// WHAT THIS MODULE IS
// -------------------------------------------------------------------------------
//   It is the ENCODER + dirty-tracking + double-buffered mailbox + control block.
//   Its INPUTS are the "taps" (NES-BRIDGE-SPEC.md SS2):
//     * Class A (PSRAM bus, wired from nes_wrap.v): nametable writes (post-mirror
//       physical CIRAM offset) and CHR-RAM dirty tiles.
//     * Class B (new ppu.v ports, threaded via nes.v; molde dbg_cpu): palette
//       writes (idx,val), OAM bytes, and a per-frame snapshot latched at
//       scanline==241 (loopy_t + fine_x + PPUCTRL + PPUMASK + chr bank per slot).
//   The gate testbench drives these ports DIRECTLY from a Python stimulus
//   generator that replays each Mesen trace through bridge_sim; in hardware
//   nes_wrap.v wires the real taps to the same ports.
//
// DELIBERATE FEED-INS (documented spec/sim/RTL boundaries)
// -------------------------------------------------------------------------------
//   * SCROLL from loopy_T (not loopy_V): bridge_sim/ppustate.py::current_scroll
//     derives scroll from loopy_T (the reload reg).  At vblank loopy_V holds the
//     STALE last-rendered VRAM address; loopy_T holds scroll intent.  So the ppu.v
//     tap exports loopy_t_out (contradicting spec SS2.2 which names loopy_v).
//   * CHR bank: per-slot bank tap; the bridge diffs vs prev (matches chr_slots()).
//   * forced_blank flag: across the whole 15-trace golden dataset the NON-CHR vram
//     total is structurally <=4448 B (< 6000 budget; SS13.4 proves <=4456), so
//     forced_blank is EXCLUSIVELY driven by chr_bank_reupload -- an SNES-VRAM-LRU
//     residency quantity the FPGA cannot observe.  It is FED as `snap_fb_hint`.
//     The other three flags (full_redraw, palette_present, chr_present) ARE
//     computed here from the bridge's own dirty state.
//
// Byte layout is byte-exact with bridge_sim/mailbox.py (SS4.3); canonical order
// with bridge_sim/encoder.py (SS4.2).  Multi-byte little-endian.
//
// MEMORY TIMING: shadow BRAMs use a COMBINATIONAL read-address mux (nt_rd_a/
// chr_rd_a/oam_rd_a) feeding a registered read (1-cycle latency).  Every scan is
// a 2-state request/consume loop (S_*A sets the address via the FSM reg it maps
// to, S_*B consumes the registered read output).  Runs are built streaming so the
// run length is known at close -- no backpatch, inline XOR.
//
// M9K INFERENCE (Quartus, cost the first synthesis run -- don't regress):
//   * Every array read must be SYNCHRONOUS and land DIRECTLY in its own register
//     (`q <= mem[addr]`).  Muxing two async array reads in front of one register
//     ("win_data <= sel ? mbox1[a] : mbox0[a]") is "asynchronous read logic" to
//     quartus_map: both arrays uninfer (Info 276007) and the 2x8KiB mailbox
//     falls back to ~131k FFs => Error 276003 aborts the fit.  The window read
//     is therefore one dedicated q-register PER buffer + comb mux AFTER them
//     (see "window read" block).  ciram/ntdirty/oam and the v2.7 CHR arrays
//     (chrmem/chrdsc/chrpend) already follow the sync-read template and infer.
//   * `pal` (32x6 = 192 FFs) is DELIBERATELY left as an uninferred register
//     array: same acceptable pattern as ppu.v's oam/sprtemp/palette arrays,
//     uninferred since Fase -1.  Its READ is nonetheless REGISTERED (pal_q,
//     request/consume S_PAL*_RD/_WR pair) -- see the SERIALIZER TIMING note.
//
// CHR-RAM DELIVERY (v2.7): CMD_CHR_RUN 0x41 is AT-LEAST-ONCE, and the store
// behind it is an 8 KiB MIRROR of the CHR-RAM plus a 512-entry DIRTY-TILE
// bitmap in two generations -- the ntpend algebra applied to CHR.  A recovery
// frame re-emits the unconfirmed tiles from the mirror; a run is idempotent at
// the renderer (shadow, last-wins), so a redundant re-emission is a no-op and
// tiles over the per-frame budget are DEFERRED, never discarded.  This replaces
// the v2.6 ack-gated payload RING, whose capacity valve gave up by discarding
// (measured: 32-460 B of CHR-RAM lost for good with the consumer parked inside
// a dump).  Full derivation and the byte-identity argument: the CHR MIRROR +
// PENDING block.  Nothing about it is visible to the byte-exact golden gate.
//
// SERIALIZER TIMING (cost the second STA run -- do not regress): every value
// that lands in mb_wdata/xor_acc must come from a REGISTER (pal_q/ciram_q/
// oam_q/pal_cnt_r/FSM regs), never from {scan -> async array mux} combinational
// chains.  The original S_PAL chained {popcnt32(pal_dirty) -> use_full select
// -> pal[idx] 192-FF mux -> mb_wdata -> xor_acc} in one cycle: setup -4.85ns
// (worst paths pal_dirty[*] -> xor_acc/mb_wdata).  Fixes: pal_cnt_r is
// maintained incrementally at the tap (popcount deleted); the pal read gets a
// request/consume state pair like every other array.  NT/OAM/CHR already
// consumed registered reads (ciram_q/oam_q/ntdirty_q/chrmem_q/chrdsc_q/chrpend_q).  Extra cost:
// +1 cycle per LIST entry, 2 cycles/byte in FULL (~ +34 CLK2 worst = ~0.4us,
// vs a ~1.27ms NES vblank -- noise).
//
// DEFERRED CAPTURE WRITES vs tick_accept (why the four raster capture modules
// are allowed to land their array writes one CLK2 late).  Three of them defer:
// nes_split_capture and nes_ppusplit_capture register their INPUTS, and
// nes_split_capture and nes_chrwin_capture also register the array-write
// DECISION.  Each costs at most one CLK2 between a core `ce` and the array
// settling.  Two readers exist, and both have far more room than that:
//
//   1. THE NEXT `ce`, which re-reads the arrays through the eviction tree and
//      the coalescence compare.  The nes_wrap pacer keeps consecutive `ce`
//      pulses >= 13 CLK2 apart (derivation in the nes_wrap header).  MEASURED
//      by run_nestest on the real SYSCLK pacer: min 15, max 23 CLK2 over 79,665
//      pulses.  So >= 12 cycles of the floor remain after the deferral, and >= 14
//      in practice.  run_split_equiv.sh and run_chrwin_equiv.sh turn this into
//      an assertion rather than an argument: they sample the cycle BEFORE every
//      ce and require the deferred module to already equal an immediate
//      reference (6,312 samples each, zero not-settled).
//
//   2. THIS FILE'S S_IDLE ACCEPT, which latches snap_spl_*/snap_cspl_*/
//      snap_cwin_*/snap_psp_* into l_spl_*/l_cspl_*/l_cwsp_*/l_psp_*.  It is
//      much further away, and by construction rather than by luck:
//        - the modules gate their array writes on scanline <= 239, so the last
//          write of a frame is a tick on line 239;
//        - nes_wrap sets frame_tick_r on the `ce` where scanline reaches 241,
//          i.e. a full scanline of core ticks later;
//        - only then can S_IDLE accept, and it is deferred further whenever the
//          close coincides with an nt/chr write.
//      NOT DIRECTLY MEASURED end to end: the only full-system testbench
//      (tb_nestest) stops after ~1.27M CLK2 -- less than one NES frame -- so
//      scanline never passes 233 there and tick_accept never fires.  The leg
//      rests on the reads above, not on a measurement; if that ever needs
//      closing, it wants a rendering ROM run for >= 2 frames through nes_wrap.
//
//   The OTHER direction is safe by construction and must stay that way: the
//   capture modules take their frame_tick from tick_accept, and this FSM sets
//   tick_accept in the SAME cycle it latches the arrays.  Both are non-blocking,
//   so the latch reads the pre-reseed arrays; the reseed then lands two cycles
//   later (one for the tick_accept register, one for the module's own input
//   pipeline).  Reseeding on the raw frame_tick instead threw the entry list
//   away before this FSM read it -- measured as a whole CMD_CHR_SPLITS8 lost in
//   l3_minelvaton frame 228.
//////////////////////////////////////////////////////////////////////////////////

module nes_bridge(
  input         clk,
  input         rst,

  // ---- Class A tap: nametable write (physical CIRAM offset, post-mirror) ----
  input         nt_we,
  input  [10:0] nt_addr,           // 0..0x7FF (2 KiB CIRAM; H/V/1-screen)
  input  [7:0]  nt_data,

  // ---- Class A tap: CHR-RAM write (offset + DATA) -- v2.4 ---------------------
  // Fase 2.2-lite redefined CMD_CHR_RUN 0x41 to carry the payload INLINE
  // (chr_off(2) len(1) data(len), the CMD_NT_RUN shape), so the tap now has to
  // deliver the WRITTEN BYTE, not just "tile N is dirty".  chr_off is the BYTE
  // offset inside the (<=8 KiB) CHR-RAM, i.e. the mapper-resolved flat address
  // -- the same quantity the old chr_tile carried shifted right by 4
  // (nes_wrap: tapA_addr_w[12:0] instead of tapA_addr_w[16:4]).  CHR-RAM > 8 KiB
  // is MENU_ERR_NOIMPL in the loader (design SS5.1), so 13 bits is the whole space.
  input         chr_we,
  input  [12:0] chr_off,           // byte offset in CHR-RAM (mapper-resolved)
  input  [7:0]  chr_data,          // the byte written

  // ---- Class B tap: palette write (raw idx 0..31; bridge applies NES mirror) ----
  input         pal_we,
  input  [4:0]  pal_idx,
  input  [5:0]  pal_data,

  // ---- Class B tap: OAM byte ----
  input         oam_we,
  input  [7:0]  oam_addr,
  input  [7:0]  oam_data,
  // ---- OAM freeze trigger (v1.4b OAM-tear fix) ------------------------------
  // A 1-cycle pulse that snapshots the live oam[] into oam_frz[] (257 cycles).
  // nes_wrap pulses it MID-DISPLAY (~scanline 120), where OAM is provably stable
  // -- games only rewrite OAM via DMA in vblank, so OAM@120 == OAM@frame-close.
  // Freezing there removes the snapshot from the vblank/OAM-DMA window entirely
  // (the copy-at-tick path snapshotted IN vblank, next to the game's ~dot-253
  // OAM-DMA).  Testbenches tie it to frame_tick (copy-at-tick), byte-identical
  // to the old S_CPY: the NT scan alone is >=4096 cycles before OAM is read.
  input         oam_freeze,

  // ---- Frame close (scanline==241) + snapshot ----
  input         frame_tick,
  input  [15:0] snap_frame,
  input  [14:0] snap_loopy_t,
  input  [2:0]  snap_fine_x,
  input  [7:0]  snap_ppuctrl,
  input  [7:0]  snap_ppumask,
  input         snap_fb_hint,
  // NT arrangement (protocol v1.1, FRAME_HDR.flags[5:4] -- lockstep with
  // bridge_sim/mailbox.py NTARR_*): 0=horizontal mirroring (CIRAM A10=PPU
  // A11, DK), 1=vertical (A10=PPU A10, SMB1), 2=single-screen low,
  // 3=single-screen high.  Sampled per frame at the tick like every other
  // snap_* -- the field already carries DYNAMIC mirroring; v1.1 wiring feeds
  // the static mapper_flags[14] (see nes_wrap), v1.2 swaps in a live mmu tap
  // without touching this module or the protocol.
  input  [1:0]  snap_ntarr,
  input         snap_s0_present,
  input  [7:0]  snap_s0_bank,
  input         snap_s1_present,
  input  [7:0]  snap_s1_bank,

  // ---- v1.3 multi-scroll (CMD_SPLITS, opcode 0x11) -- ADDITIVE ----------------
  // Up to K=4 per-frame scroll entries latched at the tick (snapshot inputs, same
  // discipline as snap_loopy_t).  Emitted ONLY when cnt>=2: split-less games (DK)
  // feed cnt<=1 and NOTHING is emitted -> their command stream (CMD_REGS keeping
  // the mid-display fallback) is byte-identical to v1.2.  Each entry is RAW
  // (scanline, loopy_t, fine_x); sx/sy/ntsel are derived HERE at the tick exactly
  // like CMD_REGS (l_sx={T[4:0],fx}, l_sy={T[9:5],T[14:12]}, ntsel=T[11:10]), so
  // nes_wrap only has to CAPTURE raw taps (mirror of loopy_mid_r).  Flat packed
  // vectors: entry i = [i*8 +:8] / [i*15 +:15] / [i*3 +:3].
  input  [2:0]  snap_split_cnt,   // 0..4 valid entries
  input         snap_split_ovf,   // >4 changes this frame: keep first K, flag it
  input  [31:0] snap_spl_sl,      // 4x scanline[7:0]
  input  [59:0] snap_spl_t,       // 4x loopy_t[14:0]
  input  [11:0] snap_spl_fx,      // 4x fine_x[2:0]

  // ---- v2.3 CHR raster-split (CMD_CHR_SPLITS, opcode 0x13) -- ADDITIVE --------
  // Up to K=4 per-frame (scanline, CHR slot-0 bank) entries, captured by
  // nes_chrsplit_capture.v with the SAME discipline as the scroll splits above
  // (entry 0 = display start, <=1-line coalescence, shortest-strip eviction).
  // Motivation: CMD_CHR_STATE (0x12) is one ABSOLUTE state per frame, which
  // cannot describe games that re-bank MID-DISPLAY every frame (RoboCop 2, MMC1
  // 4K).  Emitted ONLY when cnt>=2 AND !poison: everything else feeds cnt<=1 and
  // NOTHING is emitted, so those frames stay byte-identical to v2.2.
  // `poison` = a mid-frame 8K<->4K (s1_present) flip made the captured strips
  // meaningless; suppress rather than ship an uninterpretable list.
  // Flat packed vectors: entry i = [i*8 +:8] in both.
  input  [2:0]  snap_cspl_cnt,    // 0..4 valid entries
  input         snap_cspl_ovf,    // >4 changes this frame: evicted, flag it
  input         snap_cspl_poison, // frame invalidated (CHR mode changed mid-frame)
  input  [31:0] snap_cspl_sl,     // 4x scanline[7:0]
  input  [63:0] snap_cspl_bank,   // 4x {s1_eff[7:0], s0_bank[7:0]}

  // ---- v2.5 CHR WINDOW VECTOR, mapper 4 / MMC3 (CMD_CHR_STATE8 0x14 +
  //      CMD_CHR_SPLITS8 0x15) -- ADDITIVE ---------------------------------------
  // MMC3 replaces the "pair of 4KB halves" CHR model with a VECTOR OF EIGHT 1KB
  // WINDOWS, which the slot0/slot1 encoding of CMD_CHR_STATE 0x12 cannot express.
  // The vector arrives here already NORMALIZED and SIZE-MASKED by the mmu tap
  // (chr_snap_win): window k = the physical 1KB bank the PPU fetches for CHR
  // addresses k*1024..k*1024+1023, in [k*8 +: 8].
  //
  //   snap_chr_win_en  -- the ACTIVE mapper publishes a window vector (mappers 4,
  //                       69 and the Namco 108 family 206/88/95/154).  It is the ONE gate: 0x14 is emitted on EVERY
  //                       frame while it is set and NEVER while it is clear, so
  //                       every frame of every mapper 0/1/2/3/7/28 game stays
  //                       byte-identical (the same additivity clause that closed
  //                       v1.3/v2.3).
  //   snap_chr_win_flags -- the 0x14 flags BYTE verbatim (bit0 = CHR-RAM, bits
  //                       7:1 reserved zero).  Carried as a byte, not a bit, so
  //                       a future flag costs no new port; LOCKSTEP with
  //                       bridge_sim/mailbox.py CHR8_FLAG_CHR_RAM.
  //   snap_cwin_*      -- K=4 raster entries from nes_chrwin_capture.v (entry 0 =
  //                       display start, <=1-line coalescence, shortest-strip
  //                       eviction).  NO POISON port: the window vector has no
  //                       mode flip that can invalidate captured strips, so that
  //                       valve has no trigger here (it stays on the 0x13 path).
  input         snap_chr_win_en,
  input  [63:0] snap_chr_win,
  input  [7:0]  snap_chr_win_flags,
  input  [2:0]  snap_cwin_cnt,    // 0..4 valid entries
  input         snap_cwin_ovf,    // >4 changes this frame: evicted, flag it
  input  [31:0] snap_cwin_sl,     // 4x scanline[7:0]
  input  [255:0] snap_cwin_win,   // 4x win[63:0]

  // ---- Fase 3: PPU raster split (CMD_PPU_SPLITS 0x16) -----------------------
  // K=4 (scanline, payload) entries from nes_ppusplit_capture.v, payload =
  // {3'b0, ppuctrl[4], ntcode[3:0]}.  Three deltas vs the 0x13/0x15 ports, all
  // deliberate:
  //   snap_psp_frozen -- "entry 0 was captured with rendering ON".  When it is
  //                      CLEAR the frame ran its whole display window with
  //                      rendering off, entry 0 still holds the value seeded at
  //                      the PREVIOUS frame_tick (a 1-frame lag), and this
  //                      module substitutes the LIVE close-time payload.  See
  //                      the l_psp_pay0 note at the tick.
  //   snap_ntcode     -- the mmu's live 4-bit nametable code at the close, the
  //                      other half of that substitution (snap_ppuctrl already
  //                      carries the live $2000).  It is ALSO the reason
  //                      FRAME_HDR.flags[5:4] and the 0x16 can never disagree:
  //                      both are derived from the SAME latched byte.
  //   no poison port  -- every {b4, ntcode} composition is interpretable, so
  //                      there is nothing to invalidate a captured list (same
  //                      as the 0x15 path).
  // K is 3 here, not the 4 of the 0x11/0x13/0x15 ports, and the payload arrives
  // in 5 bits: bits 7:5 of the serialized byte are protocol padding and are
  // re-added at emission.  Both are area decisions taken with a measured fit --
  // see the nes_ppusplit_capture.v header.
  input  [2:0]  snap_psp_cnt,     // 1..3 valid entries (cnt==1 is LEGAL here)
  input         snap_psp_ovf,     // >3 changes this frame: evicted, flag it
  input         snap_psp_frozen,  // entry 0 seeded with rendering ON
  input  [23:0] snap_psp_sl,      // 3x scanline[7:0]
  input  [14:0] snap_psp_pay,     // 3x payload[4:0]  (bits 7:5 are padding)
  input  [3:0]  snap_ntcode,      // live nt_snap_code at the frame close

  // ---- Frame-close ACCEPT pulse (feeds the raster capture modules) ---------
  // The bridge does NOT latch snap_* on the raw frame_tick: it latches on the
  // ACCEPT in S_IDLE, which is additionally gated on !nt_rmw && !nt_we &&
  // !chr_we (a CIRAM/CHR-RAM write landing on the same cycle would snapshot the
  // ring pointers before that byte's bump).  Those taps fire on exactly the
  // cycle a game writes, and a frame close CAN coincide with one, so the accept
  // is routinely deferred 1-2+ cycles past the tick.
  //
  // The capture modules used to reseed on the RAW frame_tick, so in a deferred
  // frame they had ALREADY thrown their entry list away by the time the bridge
  // read it -- the list collapsed to the reseeded cnt==1 and the whole command
  // vanished for that frame.  That is a REAL, MEASURED loss, not a theoretical
  // one: l3_minelvaton_jp_gameplay frame 228 drops its CMD_CHR_SPLITS8 (0x15,
  // 20 bytes) in the end-to-end run_maptap gate while the simulator emits it.
  // bridge_sim cannot see the class at all -- it models the tick as an atomic
  // copy, with no accept and no tap collisions.
  //
  // So the modules are reseeded from THIS pulse instead.  It is registered (one
  // cycle after the accept) on purpose: the bridge has then provably already
  // sampled the arrays, and no combinational path is created from nt_we/chr_we
  // into the capture modules.
  output reg    tick_accept,

  // ---- Control block outputs ----
  output reg [15:0] frame_seq_o,
  output reg [15:0] frame_len_o,
  output reg [7:0]  status_o,
  output reg        frame_done_o,

  // ---- Control block inputs ----
  input  [15:0] frame_ack_i,
  input         buf_sel_i,
  // Full-state resync enable (hardware ties 1; the byte-exact gate tb ties 0
  // -- its golden assumes perfect consumption and never writes ACK, which
  // would otherwise read as "renderer never caught up" and force full frames).
  // When enabled, a frame is serialized FULL-STATE (whole CIRAM as NT runs +
  // PALETTE_FULL + re-emitted CHR banks + FULL_REDRAW|PAL flags) whenever:
  //   (a) no ACK has been seen since reset (boot: the renderer's first
  //       consumed frame is always complete -- the lost-boot-frames fix), or
  //   (b) the renderer fell >=2 frames behind at frame close (a never-ACKed
  //       buffer is about to be overwritten = real loss; the next frame
  //       re-carries everything).
  // OAM is unconditional already; CHR_DIRTY is NOT forced (CHR-RAM resync is
  // Fase 2; v1 renderer parses it as no-op anyway).
  input         resync_en,

  // ---- SNES-facing mailbox window read ($6000-$7FFF), 1-cycle registered ----
  input  [12:0] win_addr,
  output [7:0]  win_data,

  // ---- Joypad (deliverable f) ----
  input  [15:0] ctrl_p1_i,
  input  [15:0] ctrl_p2_i,
  // Core-tick qualifier for the joypad RELOAD (see the joypad block).  Tie 1'b1
  // to get the pre-fix behaviour (reload every CLK2) -- which is what the
  // injected-tap testbenches do, so their byte streams are unchanged.
  input         ce_tick,
  input         joy_strobe,
  input  [1:0]  joy_clock,
  output [1:0]  joypad_data_o,

  // ---- Breadcrumb band counters ----
  // palette liveness fingerprint (device "bad boot loses palette" probe;
  // group 0x04 idx22/23 via nes_wrap, NDBG v3 +28/+29):
  //   dbg_pal_sum  = sum mod 256 of the 32 live pal[] entries (registered;
  //                  SUM -- not XOR -- so identical-pair writes don't cancel;
  //                  the EXPECTED value is trivially computed from Mesen/sim)
  //   dbg_pal_wcnt = rolling count of tapped palette writes since reset
  output reg [7:0] dbg_pal_sum,
  output reg [7:0] dbg_pal_wcnt,
  output reg [15:0] bc_bytes_last,
  output reg [15:0] bc_frames,
  output reg [15:0] bc_overruns
);

  // ============================================================ opcodes / flags
  localparam [7:0] OP_FRAME_HDR = 8'h01;
  localparam [7:0] OP_REGS      = 8'h10;
  localparam [7:0] OP_SPLITS    = 8'h11;   // v1.3 multi-scroll (CMD_SPLITS)
  // v2.2 CMD_CHR_STATE: ABSOLUTE current CHR-bank state, emitted EVERY frame
  // right after CMD_REGS (fixed offset 14 = HDR 6 + REGS 8).  Design decision
  // after 3 device failures in the event-driven CHR-bank family (first-valid
  // swallow, resync re-announce runaway, 4K-mode inference): follow the
  // hardware -- publish STATE, renderer reconciles idempotently.  4 bytes:
  //   +0 0x12 | +1 s0_bank | +2 (s1_present ? s1_bank : 0) | +3 {7'd0, s1_present}
  // s1_present is the 8K/4K discriminator that never travelled before.  The
  // event CMD_CHR_BANK ($40) stays as an ADVISORY timing/forced-blank hint
  // (drives FLAG_CHR_PRESENT); the renderer MUST reconcile from CHR_STATE.
  localparam [7:0] OP_CHR_STATE = 8'h12;
  // v2.3 CMD_CHR_SPLITS: WHERE the CHR slot-0 bank changed inside the display.
  // ADDITIVE on top of CHR_STATE (which stays unconditional at fixed offset 14);
  // this one is emitted right after it, ONLY when cnt>=2 && !poison:
  //   +0 0x13 | +1 hdr {ovf, 4'd0, cnt[2:0]} | then cnt x [scanline s0 s1]
  // Entry 0 = the bank at display start; each entry is valid until the next
  // entry's scanline (the last one until scanline 240).  The renderer
  // raster-splits CHR via HDMA $210B from this list and still reconciles the
  // absolute state from CHR_STATE, so a frame without the command (the common
  // case) behaves exactly as in v2.2.
  localparam [7:0] OP_CHR_SPLITS= 8'h13;
  // v2.5 CMD_CHR_STATE8 / CMD_CHR_SPLITS8 -- the mapper-4 (MMC3) CHR model.
  // BYTE LAYOUT (this block is the CONTRACT; bridge_sim/mailbox.py mirrors it):
  //
  //   CMD_CHR_STATE8  0x14 : 14 win0 win1 win2 win3 win4 win5 win6 win7 flags
  //                          10 bytes, FIXED FRAME OFFSET 18 (= HDR 6 + REGS 8 +
  //                          CHR_STATE 4), emitted UNCONDITIONALLY on every
  //                          mapper-4 frame.  win[k] = the physical 1KB bank the
  //                          PPU fetches for CHR k*1024..k*1024+1023 (already
  //                          normalized for chr_a12_invert and masked by the CHR
  //                          size class).  flags bit0 = CHR-RAM (the renderer
  //                          picks the window SOURCE from it: pre-converted
  //                          CHR-ROM in PSRAM vs the converted CHR-RAM mirror in
  //                          WRAM); bits 7:1 RESERVED ZERO.
  //   CMD_CHR_SPLITS8 0x15 : 15 hdr cnt x [scanline win0..win7]
  //                          hdr = {ovf, 4'd0, cnt[2:0]}; 2 + cnt*9 bytes, at
  //                          FIXED FRAME OFFSET 28 (immediately after the 0x14),
  //                          emitted ONLY when cnt>=2.  Each entry is an
  //                          ABSOLUTE snapshot of the whole vector, valid until
  //                          the next entry's scanline (the last until 240).
  //
  // THE TWO HARD RULES, and how each one is enforced structurally:
  //   (1) 0x14/0x15 NEVER appear outside a window-vector mapper (4, 69, 206/88/95/154) --
  //       snap_chr_win_en is the only path into S_CWST, and the mmu drives it
  //       from flags[7:0] (4, 69 and the Namco 108 family 206/88/95/154).
  //   (2) 0x13 NEVER appears IN those mappers      -- twice over: the legacy
  //       chr_snap_s0/s1 tap is a CONSTANT for them (so its capture can
  //       never reach cnt>=2, see mmu.v), AND the state chain below routes
  //       S_CHRST -> S_CWST -> S_CWSP_* -> S_SPL_*, never touching S_CSPL_OP
  //       when the window vector is enabled.
  localparam [7:0] OP_CHR_STATE8 = 8'h14;
  localparam [7:0] OP_CHR_SPLITS8= 8'h15;
  // Fase 3 -- CMD_PPU_SPLITS 0x16 (NES-MIDFRAME-SPEC.md SS2.1)
  //   16 hdr cnt x [scanline payload], hdr = {ovf, 4'd0, cnt[2:0]},
  //   payload = {3'b0, ppuctrl[4], ntcode[3:0]}.  Canonical position:
  //   immediately AFTER the 0x11, in the raster block (NOT a fixed offset --
  //   only 0x12/0x14 are; the parser is per-opcode).
  //
  //   EMISSION GATE, and it is the ONE place this command differs from its
  //   siblings 0x11/0x13/0x15:
  //        emit  <=>  cnt >= 2   OR   entry0's code is NOT classic
  //   The second clause exists because flags[5:4] has only TWO bits: a frame
  //   whose code is non-classic but STATIC (mapper 118 with chr_a12_invert, the
  //   (1,0) of mapper 95, Namco 163's independent quadrant registers) has no
  //   other channel at all.  So cnt==1 IS a legal 0x16 -- measured on
  //   l3_finallap, 1497/1500 frames.  Every opcode walker must handle it.
  //
  //   PRECEDENCE: while a 0x16 is present it is AUTHORITATIVE and both
  //   FRAME_HDR.flags[5:4] and CMD_REGS.ppuctrl[4] are to be ignored (the same
  //   rule as "0x14 present => ignore the 0x12").
  localparam [7:0] OP_PPU_SPLITS = 8'h16;
  localparam [7:0] OP_PALETTE   = 8'h20;
  localparam [7:0] OP_PAL_FULL  = 8'h21;
  localparam [7:0] OP_NT_RUN    = 8'h30;
  localparam [7:0] OP_CHR_BANK  = 8'h40;
  // v2.4 CMD_CHR_RUN (was CMD_CHR_DIRTY): the opcode BYTE is unchanged (0x41)
  // but the payload is now INLINE -- `41 off_lo off_hi len data[len]`, exactly
  // the CMD_NT_RUN shape, with off = BYTE offset in CHR-RAM (0..0x1FFF) and
  // len 1..255.  The old form (`41 base_tile(2) count(1)`, a POINTER into
  // PSRAM) forced the renderer to read CHR-RAM back over the core's own PSRAM
  // bus and the bridge to keep an 8 KiB M9K shadow; W6's measurement
  // (chrram-study: 99.6-99.9% of the writes sequential, p99 = 1 run/frame,
  // worst frame 2120 B) makes carrying the bytes strictly cheaper.  Only
  // CHR-RAM games are affected: chr_we can only fire when the mapper allows
  // CHR writes, so every CHR-ROM golden stays byte-identical.
  localparam [7:0] OP_CHR_RUN   = 8'h41;
  localparam [7:0] OP_OAM       = 8'h50;
  localparam [7:0] OP_FRAME_DONE= 8'hF0;

  localparam [7:0] FLAG_FORCED_BLANK    = 8'h01;
  localparam [7:0] FLAG_FULL_REDRAW     = 8'h02;
  localparam [7:0] FLAG_PALETTE_PRESENT = 8'h04;
  localparam [7:0] FLAG_CHR_PRESENT     = 8'h08;

  localparam NT_SIZE   = 2048;
  // v2.4 CHR-RAM capture ring (replaces the 512-entry dirty bitmap + its pend
  // generations, all three of which became dead the moment 0x41 started
  // carrying data -- see the CHR RING block below for the sizing argument).
  localparam CB_SZ     = 8192;   // CHR-RAM mirror, bytes (the whole address space)
  localparam DSC_N     = 256;    // run-descriptor ring, entries (ONE frame's worth)
  // MAILBOX CAPACITY VALVE for the CHR drain (v2.6).  A mailbox buffer is 8192
  // bytes and wptr is 13 bits: it WRAPS silently.  Before ack-gating, one
  // frame could only ever carry one frame's worth of CHR (<=2120 B measured,
  // worst golden frame 2721 B total), so the wrap was unreachable in practice.
  // With at-least-once the ring can hold ~3 frames of unconfirmed payload and a
  // single recovery frame would try to ship all of it, on top of a possibly
  // large NT union -> wrap = a corrupt frame, the one thing this module must
  // never produce.  So a run is only STARTED while wptr < CHR_WSTOP; whatever
  // does not fit stays in the ring (the drain cursor simply stops there) and is
  // shipped by the next frame -- at-least-once is preserved, the frame is not.
  //   worst tail after the last accepted start = 4 (run header) + 255 (payload)
  //   + 257 (OAM) + 2 (FRAME_DONE) = 518  =>  CHR_WSTOP <= 8192-518 = 7674.
  // 7600 leaves 74 B of slack.  INERT for every golden (max frame 2721 B).
  localparam [12:0] CHR_WSTOP = 13'd7600;
  // ---- CHR-RAM RE-SEND BUDGET (v2.7: MIRROR + DIRTY-TILE BITMAP) -----------
  //
  // WHAT REPLACED WHAT.  Until v2.6 the CHR payload lived in a CIRCULAR RING
  // and retransmission meant HOLDING that ring until an ACK proved delivery.
  // That is why this file used to carry RETX_WINDOW / RETX_CHUNK / RETX_DSC /
  // RING_HIWATER / DSC_HIWATER, a committed tail (cb_cp/dsc_cp), an 8-deep
  // publish history, chain_broken, a rewind and an all-or-nothing TRIM:
  // CAPACITY, not policy, was the thing being managed, and the trim's GIVE-UP
  // was a PERMANENT loss of CHR-RAM -- the one piece of state that had no
  // idempotent mirror.  Measured on the real trace with the consumer parked
  // INSIDE a 512-tile dump (Fantasy Zone JP, mapper 93, rendering off,
  // rtl-tb/run_chrburst.sh): 245 B lost for good at seq 830 x 30 frames before
  // the ack_win_q fix, and still 32 B at 840 x 30, 245 B at 880 x 30 and 460 B
  // at 880 x 60 after it -- all of them the trim giving up.
  //
  // The store is now ABSOLUTE.  chrmem is an 8 KiB MIRROR of the CHR-RAM,
  // indexed by physical offset, so a byte can never be aged out of it; what is
  // unconfirmed is described by a 512-entry DIRTY-TILE bitmap in TWO
  // GENERATIONS (the ntpend algebra), and a recovery frame re-emits those tiles
  // straight from the mirror.  There is no capacity to run out of, so there is
  // nothing to give up: tiles over this frame's budget are DEFERRED (their bits
  // stay set) instead of DISCARDED.
  //
  // PER-FRAME RE-SEND BUDGET -- ARITHMETIC ON THE CONSUMER, NOT A TUNING KNOB.
  // The renderer stages CHR through NES_CHRQ_TILES_MAX = 192 tiles = 3072 BYTES
  // per frame (snes/nes/nes_equates.i65).  Past that it escalates to
  // nes_chrq_full, a whole-shadow rebuild -- and a renderer that never finishes
  // a frame never ACKs, which is exactly the W16e field signature (publisher
  // alive, 6502 alive, overruns advancing 1:1 with frames, zero frames
  // consumed).  A frame carries FRESH + RESEND, and fresh alone reaches the
  // 2120 B plateau on a level-entry dump, so:
  //     resend_allowance  =  (CHR_TILE_CAP - fresh_tiles_this_frame) * 16
  // which is 952 B at the 2120 B plateau and the whole 3072 when the dump is
  // over -- the arithmetic the ring-era RETX_CHUNK approximated with a constant.
  //
  // *** ABI LOCKSTEP -- snes/nes/nes_equates.i65 NES_CHRQ_TILES_MAX ***
  //     CHR_TILE_CAP  ==  NES_CHRQ_TILES_MAX
  // Break it and NOTHING fails offline -- the byte-exact goldens contain no
  // retransmission at all, so they cannot see it; only the renderer notices, by
  // escalating to a whole-shadow rebuild and never ACKing.  Change either side
  // and re-check the other.
  // The budget is DYNAMIC, not the fixed 768 an earlier cut used: it is what is
  // left of the renderer's own per-frame ceiling after this frame's FRESH bytes.
  // Fixed-768 was measured to be too mean on the real trace -- a level-entry
  // dump makes the whole 512-tile bitmap pending at once, 768 B/frame is 48
  // tiles, and the handful of recovery frames a skip produces cannot drain 512
  // of them (run_chrburst, Fantasy Zone 820x120: 30 tiles never re-sent).  Right
  // AFTER a dump the fresh load is ~0, so the allowance opens to the full 192
  // and the same debt drains in three frames.  Fresh is NEVER delayed by it: the
  // allowance is the REMAINDER, so a 192-TILE fresh frame leaves zero.
  // (The two lines above used to say "3072 bytes", which contradicted the
  // paragraph immediately below -- the ceiling has been in TILES since v2.8.)
  // THE CEILING IS IN TILES, NOT BYTES (v2.8; the v2.7 byte form was an ABI bug).
  // The renderer stages CHR through NES_CHRQ_TILES_MAX = 192 TILE DESCRIPTORS per
  // frame (snes/nes/nes_equates.i65, consumed in nes_render.a65) -- not through
  // 3072 bytes.  The two only agree when every touched tile is written WHOLE: a
  // game that writes part of a tile pays a full descriptor for a fraction of the
  // 16 bytes, so counting bytes lets the frame pass the byte test and still blow
  // the tile ceiling (l2_senjou with rendering ON reaches 207 tiles inside 3072
  // bytes -> ncst_overflow -> nes_chrq_drop, and the renderer's whole-shadow
  // rebuild then HIDES the miss).  So the tap counts TILES and the allowance is
  // what is left of the 192 after this frame's fresh capture.
  localparam [8:0]  CHR_TILE_CAP    = 9'd192;   // == NES_CHRQ_TILES_MAX
  // Longest re-send run, in TILES.  The 0x41 len field is ONE BYTE, so 15 tiles
  // (240 B) is the largest whole-tile run that fits.  Keeping every re-send run
  // tile-aligned is what lets the scan charge its budget in units of 16 without
  // a multiplier and lets the header derive off/len by pure bit-slicing.
  localparam [4:0]  CHR_PEND_MAXT   = 5'd15;
  // 8 KiB of CHR-RAM / 16 B per NES tile.  This is the renderer's unit too
  // (nes_chr_handle_run reconverts the touched TILES), so a coarser bitmap
  // would re-send tiles the renderer would have to reconvert anyway.
  localparam        CHR_TILES       = 512;

  // ============================================================ shadow memories
  //
  // SNAPSHOT / PING-PONG (pre-hardware fix; do not regress) -- taps are accepted
  // ALWAYS (no longer gated on S_IDLE): on live hardware frame-N+1 events (the
  // game's NMI OAM-DMA lands EARLY in vblank, DURING frame N's serialization)
  // must keep accumulating while the serializer reads FROZEN state.  Per
  // category:
  //   * dirty bitmaps + per-frame counters (ntdirty/pal_dirty/counts, and the
  //     chr_any flag):
  //     PING-PONG (2 banks: taps write bank `live`, serializer scans + clears
  //     bank ~live, flip at frame_tick).  Correct because they RESET per frame.
  //   * OAM: COPY-AT-TICK (oam live + oam_frz; S_CPY, pipelined 1B/cycle, 258
  //     cycles).  NOT ping-pong: OAM is PERSISTENT -- ping-pong would emit
  //     2-frame-old bytes whenever a game skips its OAM-DMA (lag frames do);
  //     the copy freezes the tick-instant contents.  Residual race: a DMA byte
  //     landing inside the ~3us copy window may be captured one frame early --
  //     same benign class as the trace-F/scanline-241 offset (spec SS13.3).
  //   * ciram + pal[] VALUES: SINGLE, read LIVE by the serializer.  A frame-N+1
  //     write read early is emitted with the NEW value in frame N AND re-emitted
  //     in frame N+1 (its dirty bit is in the live bank) -- the renderer
  //     converges; saves a 2KiB BRAM copy.
  // M9K delta of that fix: ntdirty x2 (+2Kb) + oam_frz (+2Kb).  The CHR dirty
  // bitmap and its pend generations that used to live here are GONE since v2.4:
  // 0x41 carries data now, so a bitmap cannot describe what to send.
  // A frame_tick that collides with an in-flight nt RMW (or, theoretically,
  // lands outside S_IDLE) is latched in tick_pend and accepted at the next
  // S_IDLE cycle -- ticks are never silently dropped anymore.
  reg [7:0] ciram    [0:NT_SIZE-1];    // single, read LIVE (see note)
  reg       ntdirty0 [0:NT_SIZE-1];    // ping-pong bank 0
  reg       ntdirty1 [0:NT_SIZE-1];    // ping-pong bank 1
  reg [7:0] oam      [0:255];          // live (taps)
  reg [7:0] oam_frz  [0:255];          // frozen copy (S_CPY at tick)
  // ================================= CHR MIRROR + PENDING (v2.7, CMD_CHR_RUN)
  // MICROARCHITECTURE (the one decision this block exists to record):
  //
  //   chrmem  = an 8 KiB MIRROR of the CHR-RAM, indexed by the mapper-resolved
  //             PHYSICAL OFFSET.  The tap writes it unconditionally, one byte
  //             per cycle, last-wins; the serializer reads it.  It is ABSOLUTE
  //             state, exactly like ciram: nothing can be aged out of it, so
  //             there is no retransmission window, no high-water and no
  //             give-up.  It occupies the eight M9K blocks the payload ring
  //             used to hold, so this is a re-interpretation of the same
  //             memory, not new memory.
  //   chrdsc  = a ring of RUN DESCRIPTORS {off[12:0], cnt[12:0]}, ONE write, at
  //             run OPEN only.  The run's LENGTH is never stored: it is
  //             `next_descriptor.cnt - this.cnt` (and, for the last run of a
  //             frame, `l_cb_end - this.cnt`), where cnt is a free-running
  //             count of accepted tap bytes.  Nothing is back-patched and the
  //             tap never needs more than ONE write port on either array in a
  //             cycle -- that is what keeps it a single-cycle event with no
  //             micro-sequencer and no assumption about how fast the NES can
  //             hit $2007.  Since v2.7 the descriptors carry ONE FRAME (they
  //             are drained every frame); they are NOT a retransmission store.
  //   chrpend = 512 x 2 bits, ONE ENTRY PER TILE: generations {A,B} with the
  //             ntpend algebra (bit1 = A "sealed by the last recovery",
  //             bit0 = B "young").  Written ONLY by the FSM -- the fresh walk
  //             marks a tile B as it ships it (S_CRB), the pend scan does the
  //             deliver/seal/materialise rewrite (S_CP1) -- so the array never
  //             needs a second write port and the TAP NEVER TOUCHES IT.  That
  //             single-writer discipline is the reason the per-frame dirty
  //             bitmap that NT needs (ntdirty0/1, ping-ponged) does not exist
  //             here: the descriptors already are this frame's dirty list.
  //
  // WHY THE WRITE-ORDER DESCRIPTORS SURVIVE THE MIRROR (byte identity).  The
  // golden stream emits a run's bytes AS WRITTEN, in write order; a mirror only
  // holds the LAST value of an offset.  Measured over the whole 95 900-frame
  // golden corpus (541 854 CHR bytes), SIX frames write the same offset twice
  // with different values inside one frame -- battletoads 2, rockman4ex 2,
  // metroid_deluxe_explore 1, l2_senjou 1 -- and emitting those runs from the
  // mirror would ship the final value twice.  That is semantically identical
  // (the renderer's 0x41 handler is last-wins over an 8 KiB shadow, so the
  // post-frame state cannot differ) but it is NOT byte-identical.  Keeping the
  // write-order descriptors keeps the run BOUNDARIES and the run ORDER exactly
  // as bridge_sim/ppustate.chr_runs produces them, so the divergence is
  // confined to those six frames instead of to every CHR-RAM frame in the
  // corpus (which is what an address-ordered, tile-granular emission would
  // have cost).  Do not "simplify" this into a bitmap-only encoder.
  //
  // SIZING.  The descriptors now only have to hold ONE frame plus whatever
  // CHR_WSTOP defers: the worst measured is ~90/frame (scattered writes) out of
  // 256, where v2.6 had to share the same 256 with several frames of
  // unconfirmed history.  A descriptor overflow no longer loses the DATA -- the
  // mirror has it -- it drops the RUN, latches cb_ovf for the breadcrumb and
  // sets chr_forceall, which makes the next pend scan mark EVERY tile pending
  // so the next recovery re-ships the whole mirror, 48 tiles per frame.
  // M9K BUDGET (measured in output_files/main.fit.rpt, 55/56 before this
  // change): chrmem reuses chrbuf's eight blocks, chrdsc keeps its one
  // (256x26 in a 256x36 config), chrpend takes the last free one (512x2).  If
  // quartus_map ever refuses to infer one of them, do NOT mux two async array
  // reads in front of its q-register -- see M9K INFERENCE in the header.
  reg [7:0]  chrmem  [0:CB_SZ-1];      // M9K (8 blocks): the CHR-RAM mirror
  reg [25:0] chrdsc  [0:DSC_N-1];      // M9K (1 block): {off[12:0], cnt[12:0]}
  reg [2:0]  chrpend [0:CHR_TILES-1];  // M9K (1 block): {D,A,B} per tile
  // PENDING accumulators (chain-breaker; cost a hardware iteration -- see the
  // LOSS THRESHOLD note): union of the dirty bits of every serialized-but-not-
  // yet-confirmed frame.  A loss no longer emits a FULL frame (~2.4KB, whose
  // apply on the real renderer takes 2-3 frame periods and re-triggers loss =
  // the self-sustaining degeneration seen on hardware); it emits frozen|pending
  // = the REAL delta since the last consumed frame (typically tens of bytes).
  // The scan REWRITES every visited offset each serialization
  // (pend <= frz | (pend & epoch)), so "clearing" is just dropping the epoch
  // bit -- no extra pass.  Cleared when ACK catches up (tick with ack==seq
  // and no unprocessed skip) or, per generation, when the recovery frame
  // that sealed it is confirmed consumed (ack >= recov_seq).  Worst case
  // (ACK never in time): the union saturates toward full = degrades to the
  // OLD behavior, never worse.  Boot (~saw_ack) still uses true FULL frames.
  // TWO GENERATIONS per cell (bit1 = A "sealed", bit0 = B "young") -- see the
  // GENERATION note at the pend registers below.
  // SINCE v2.7 CHR SHARES THIS ALGEBRA (chrpend above), including the valid
  // flags pend_a_valid/pend_valid and their tick latches pend_avf/pend_bvf.
  // The one CHR-specific term is DELIVERY: the pend scan has a per-frame byte
  // budget, and a tile it did not deliver must NOT be sealed into A -- sealing
  // means "this recovery carried it", and a later confirm would then drop a
  // tile that was never sent.  Un-delivered tiles are demoted to B instead, so
  // they survive the A-kill and go out in the next recovery.
  reg [1:0] ntpend  [0:NT_SIZE-1];
  reg [7:0] mbox0   [0:8191];   // M9K (inferred; see M9K INFERENCE in the header)
  reg [7:0] mbox1   [0:8191];   // M9K (inferred)
  reg [5:0] pal     [0:31];     // register array ON PURPOSE (192 FFs); read via pal_q reg

  integer k, kk;
  initial begin
    for (k=0;k<NT_SIZE;k=k+1)   begin ciram[k]=8'h00; ntdirty0[k]=1'b0; ntdirty1[k]=1'b0; ntpend[k]=2'b00; end
    for (k=0;k<256;k=k+1)       begin oam[k]=8'h00; oam_frz[k]=8'h00; end
    // GOTCHA (cost a synthesis run when CB_SZ went 4096 -> 8192): quartus_map
    // refuses a constant loop of more than 5000 iterations -- "Error (10106):
    // Verilog HDL Loop error ... loop must terminate within 5000 iterations",
    // and it kills ELABORATION of the whole module, so nothing downstream even
    // runs.  iverilog does not care.  Clearing the 8192-byte ring therefore has
    // to be a NEST (64 x 128), not a flat sweep.
    for (k=0;k<CB_SZ/128;k=k+1)
      for (kk=0;kk<128;kk=kk+1) chrmem[k*128+kk]=8'h00;
    for (k=0;k<DSC_N;k=k+1)     chrdsc[k]=26'd0;
    for (k=0;k<CHR_TILES;k=k+1) chrpend[k]=3'b000;
    for (k=0;k<32;k=k+1)        pal[k]=6'h00;
  end

  // ============================================================ dirty bookkeeping
  // All per-frame accumulators ping-ponged by `live`; the serializer reads the
  // frozen views *_frz.  pal counts stay INCREMENTAL registers (the serializer-
  // timing fix: no popcount -- see SERIALIZER TIMING in the header).
  reg        live;         // bank the TAPS write; ~live = frozen (serialized)
  reg        tick_pend;    // frame_tick latched until accepted in S_IDLE
  reg        saw_ack;      // any nonzero NES_FRAME_ACK seen since reset
  reg        lost_r;       // registered (seq-ack)>=3 (genuine loss)
  reg        force_full;   // BOOT ONLY now: serialize true full state
  reg        lost_hold;    // this frame is a RECOVERY: scan adds the pending union
  reg        seal_hold;    // this recovery SEALS generation A (A was empty/confirmed)
  // GENERATIONS (cost the tb_handshake "bootskip" scenario -- a self-
  // sustaining degenerate chain REBORN through the union): with a single
  // epoch, the only safe clear is a tick with ack==seq, which a phase-adverse
  // consumer NEVER hits while recovery frames are large -- a big burst that
  // was already delivered and CONFIRMED stays in the union forever, every
  // recovery re-carries ~2.4KB, the apply exceeds one period, the consumer
  // skips again -> ack_skip fires again -> chain.  Fix: the union lives in
  // TWO generations per cell.  New dirty accumulates in B (young).  A
  // recovery frame delivers frz|A|B and SEALS: A := frz|A|B, B := 0 (done by
  // the scan's rewrite pass, no extra sweep).  When the consumer confirms a
  // frame at/after the last recovery (ack >= recov_seq, mod 2^16), everything
  // sealed in A was delivered IN that recovery -> kill A via its valid bit
  // (no sweep), regardless of lag or later skips.  Dirty of frames after the
  // recovery is in B and survives.  The chain now dies in ~2 recoveries: the
  // second one carries only B (small).
  reg        pend_valid;   // generation B (young) holds content
  reg        pend_a_valid; // generation A (sealed at the last recovery) holds content
  reg        pend_avf;     // pend_a_valid latched at the tick (stable for the scan)
  reg        pend_bvf;     // pend_valid   latched at the tick
  reg [31:0] pal_pend_a, pal_pend_b;
  reg        recov_active; // a recovery frame is in flight, unconfirmed
  reg [15:0] recov_seq;    // seq of the LAST recovery frame published
  // ACK-SKIP detector (cost a hardware iteration -- "EVERY death transition
  // loses the background"): a 1-2 frame loss never raises seq-ack to 3 (the
  // renderer catches right back up), so the >=3 lag trigger misses it and the
  // pending union is never emitted -- the skipped transition burst is lost
  // until the next burst.  The EXACT, false-positive-free loss signal is the
  // ACK SEQUENCE itself: the renderer ACKs every frame it consumes, so an ACK
  // that ADVANCES BY >=2 in one step proves the intermediate frames were
  // skipped.  Arm a recovery for the next tick when that happens.
  reg [15:0] ack_prev;
  reg        ack_skip;     // sticky until consumed by a tick
  reg [11:0] nt_cnt0, nt_cnt1;
  reg        chr_any0, chr_any1;
  reg [31:0] pal_dirty0, pal_dirty1;
  reg [5:0]  pal_cnt0, pal_cnt1;

  wire       pal_mirror = pal_idx[4] & ~pal_idx[1] & ~pal_idx[0];
  wire [4:0] pal_eff    = pal_mirror ? {1'b0, pal_idx[3:0]} : pal_idx;

  wire [11:0] nt_cnt_frz    = live ? nt_cnt0    : nt_cnt1;
  wire        chr_any_frz   = live ? chr_any0   : chr_any1;
  wire [31:0] pal_dirty_frz = live ? pal_dirty0 : pal_dirty1;
  wire [5:0]  pal_cnt_frz   = live ? pal_cnt0   : pal_cnt1;
  // seal_hold term: a SEALING recovery moves pal_pend_b into pal_pend_a and
  // drops pend_avf on the SAME tick edge, so during that very frame's
  // serialization both register terms below read 0 and the union this
  // recovery exists to carry is never emitted -- then the next confirm kills
  // A and the entries are lost for good.  The NT path survives the same
  // algebra because its scan reads ntpend_q per cell AFTER the seal rewrite;
  // the palette pend is plain registers sampled BEFORE the scan.  Palette has
  // no shadow re-emission and no scrub, so one swallowed sealing recovery is
  // a permanently wrong CGRAM (stuck mid-fade title after a warm reset --
  // silicon-reproduced ~50% on Bomberman; tb_handshake mode palfade is the
  // distilled regression).  seal_hold is latched at the tick and only ever
  // set under resync_en, so the byte-exact goldens (resync_en=0) are
  // untouched.
  wire        pal_pend_any  = lost_hold & ((pend_avf & (pal_pend_a != 32'd0))
                                         | (pend_bvf & (pal_pend_b != 32'd0))
                                         | (seal_hold & (pal_pend_a != 32'd0)));
  wire        pal_present   = (pal_cnt_frz != 6'd0) | force_full | pal_pend_any;
  // recovery always uses PALETTE_FULL when palette data is owed: emitting the
  // 33-byte full palette avoids a popcount over frz|pend (the serializer-
  // timing rule) and is cheap
  wire        use_full      = (pal_cnt_frz > 6'd16) | force_full | pal_pend_any;

  // ============================================================ FSM state decl
  // WIDTH: 6 bits since v2.3 (the CMD_CHR_SPLITS states pushed the count past
  // 32).  Every state literal below MUST stay 6'dN -- a leftover 5'dN silently
  // zero-extends in comparisons but truncates in assignments.
  // v2.5 added FIVE states (S_CWST + S_CWSP_*), then v2.7 the five S_CP*
  // (6'd44..48) and Fase 3 the four S_PSP_* (6'd49..52), taking the highest
  // literal to 6'd52 of the 63 a 6-bit register holds -- still 11 spare, no
  // width change.
  localparam [5:0]
    S_IDLE  = 6'd0,  S_SETUP = 6'd1,  S_HDR  = 6'd2,  S_REGS = 6'd3,
    S_PAL   = 6'd4,  S_NTA   = 6'd5,  S_NTB  = 6'd6,  S_NTHDR= 6'd7,
    S_NTDA  = 6'd8,  S_NTDB  = 6'd9,  S_CBANK= 6'd10,
    // v2.4 CMD_CHR_RUN emission (replaces the old S_CA/S_CB/S_CHDR dirty-bitmap
    // scan): walk the frozen slice of the descriptor ring, then stream each
    // run's bytes out of the CHR mirror (v2.7; the payload ring before that).
    // Same request/consume
    // shape as S_NTA/S_NTB and S_NTDA/S_NTDB (every array read is REGISTERED
    // and mb_wdata only ever sees a register -- SERIALIZER TIMING).
    S_CR0   = 6'd11,  // descriptor cursor test; request chrdsc[dsc_i]
    S_CR1   = 6'd12,  // consume descriptor -> cur_coff/cur_ccnt; advance dsc_i
    S_CR2   = 6'd13,  // wait state: chrdsc[dsc_i+1] read registers
    S_CR3   = 6'd35,  // consume NEXT cnt -> cur_clen; park chr_ra_r at cur_coff
    S_CRH   = 6'd36,  // emit 41 off_lo off_hi len
    S_CRA   = 6'd37,  // request chrmem[chr_ra_r]
    S_CRB   = 6'd38,  // emit payload byte (from the mirror)
    // v2.7 CHR PENDING RE-SEND emission (tile-granular scan of chrpend, payload
    // from chrmem).  Same request/consume shape as S_NTA/S_NTB: S_CP0 sets the
    // address, S_CP1 consumes the registered read and does the ONE rewrite of
    // that cell.  Runs against the 0x41 opcode the fresh walk uses, so the
    // renderer sees no new command.
    S_CP0   = 6'd44,  // tile cursor test; request chrpend[cp_i]
    S_CP1   = 6'd45,  // consume {A,B}; deliver/seal/materialise; extend run
    S_CPH   = 6'd46,  // emit 41 off_lo off_hi len for the tile run
    S_CPA   = 6'd47,  // request chrmem[chr_ra_r]
    S_CPB   = 6'd48,  // emit payload byte
    S_OAMA  = 6'd14, S_OAMB = 6'd15,
    S_DONE  = 6'd16, S_CLEAR = 6'd17, S_FINISH=6'd18,
    // palette emission, pipelined (request/consume like NT/OAM/CHR -- the pal
    // array read is REGISTERED into pal_q; mb_wdata/xor_acc consume registers):
    S_PALL_CNT = 6'd19,  // LIST: emit count byte (pal_cnt_r)
    S_PALL_FIND= 6'd20,  // LIST: scan pal_dirty, emit idx byte on hit
    S_PALL_RD  = 6'd21,  // LIST: dead cycle, pal_q <= pal[pal_i]
    S_PALL_WR  = 6'd22,  // LIST: emit value byte (pal_q)
    S_PALF_RD  = 6'd23,  // FULL: dead cycle, pal_q <= pal[pal_i]
    S_PALF_WR  = 6'd24,  // FULL: emit value byte (pal_q)
    S_CPY      = 6'd25,  // OAM copy-at-tick (live -> frz, pipelined 1B/cycle)
    // v1.3 CMD_SPLITS emission (after S_CHRST/S_CSPL_*, before S_PAL/S_NTA):
    S_SPL_OP   = 6'd26,  // emit opcode 0x11
    S_SPL_HDR  = 6'd27,  // emit hdr byte {ovf, cnt}
    S_SPL_LD   = 6'd28,  // consume: cur_* <= l_spl_*[spl_i]  (idx mux off the mb path)
    S_SPL_E    = 6'd29,  // emit the 4 bytes of entry spl_i (sl, sx, sy, ntsel)
    // v2.2 CMD_CHR_STATE emission (UNCONDITIONAL, right after S_REGS -- fixed
    // frame offset 14 = HDR(6)+REGS(8); absolute CURRENT bank state every
    // frame, the renderer reconciles idempotently -- see the localparam
    // OP_CHR_STATE comment for the design rationale):
    S_CHRST    = 6'd30,  // emit 12 s0_bank (s1p?s1_bank:0) {7'd0,s1p}
    // v2.3 CMD_CHR_SPLITS emission (right after S_CHRST, BEFORE S_SPL_*), same
    // request/consume shape as S_SPL_*: S_CSPL_LD moves the indexed array read
    // off the mb_wdata/xor_acc cone, S_CSPL_E emits from single registers.
    S_CSPL_OP  = 6'd31,  // emit opcode 0x13
    S_CSPL_HDR = 6'd32,  // emit hdr byte {ovf, cnt}
    S_CSPL_LD  = 6'd33,  // consume: cur_c* <= l_cspl_*[cspl_i]
    S_CSPL_E   = 6'd34,  // emit the 2 bytes of entry cspl_i (scanline, bank)
    // v2.5 CMD_CHR_STATE8 / CMD_CHR_SPLITS8 emission (mapper 4).  The 0x14 sits
    // between S_CHRST and the split states so its 10 bytes land at the fixed
    // frame offset 18; the 0x15 states copy the S_CSPL_* request/consume split
    // one for one (S_CWSP_LD moves the indexed 64-bit array read off the
    // mb_wdata/xor_acc cone; S_CWSP_E emits from single registers).
    S_CWST     = 6'd39,  // emit 14 win0..win7 flags   (10 bytes, sub 0..9)
    S_CWSP_OP  = 6'd40,  // emit opcode 0x15
    S_CWSP_HDR = 6'd41,  // emit hdr byte {ovf, cnt}
    S_CWSP_LD  = 6'd42,  // consume: cur_cw* <= l_cwsp_*[cwsp_i]
    S_CWSP_E   = 6'd43,  // emit the 9 bytes of entry cwsp_i (scanline + vector)
    // Fase 3 CMD_PPU_SPLITS emission, chained right AFTER S_SPL_* (canonical
    // position: immediately after the 0x11).  Copies the S_CSPL_* shape one for
    // one -- S_PSP_LD moves the indexed array read off the mb_wdata/xor_acc
    // cone (SERIALIZER TIMING), S_PSP_E emits from single registers.
    S_PSP_OP   = 6'd49,  // emit opcode 0x16
    S_PSP_HDR  = 6'd50,  // emit hdr byte {ovf, cnt}
    S_PSP_LD   = 6'd51,  // consume: cur_p* <= l_psp_*[psp_i]
    S_PSP_E    = 6'd52;  // emit the 2 bytes of entry psp_i (scanline, payload)

  reg [5:0]  st;
  reg [3:0]  sub;
  reg [12:0] wptr;
  reg [7:0]  xor_acc;
  reg [15:0] new_seq;
  reg        mb_wbuf;

  // latched snapshot
  reg [15:0] l_frame;
  reg [7:0]  l_ppuctrl, l_ppumask, l_flags, l_sx, l_sy;
  reg [1:0]  l_ntarr;
  reg [1:0]  l_ntsel;   // REGS byte 7: NT select = loopy_T[11:10] of the mid
                        // snapshot (v1.2) -- NOT ppuctrl[1:0] of the close,
                        // which in split games is the HUD's (always 0)
  reg        l_s0p, l_s1p, l_s0chg, l_s1chg;
  reg [7:0]  l_s0b, l_s1b;

  // v1.3 CMD_SPLITS: entries derived + latched at the tick (single registers,
  // read during emission via a request/consume LD state so the mb_wdata/xor
  // cone is a pure single-register mux -- SERIALIZER TIMING).
  reg [2:0]  l_split_cnt;         // 0..4
  reg [7:0]  l_split_hdr;         // {ovf, 4'd0, cnt[2:0]}
  reg [7:0]  l_spl_sl [0:3];
  reg [7:0]  l_spl_sx [0:3];
  reg [7:0]  l_spl_sy [0:3];
  reg [1:0]  l_spl_nt [0:3];
  reg [2:0]  spl_i;               // emission entry cursor
  reg [7:0]  cur_sl, cur_sx, cur_sy;   // consumed entry (loaded in S_SPL_LD)
  reg [1:0]  cur_nt;
  integer    si;
  integer    pi;                  // Fase 3: the 0x16 arrays are 3 deep, not 4

  // v2.3 CMD_CHR_SPLITS: same latch-at-the-tick + request/consume discipline.
  // No derivation needed (the payload is already a raw scanline + bank byte).
  reg [2:0]  l_cspl_cnt;          // 0..4
  reg [7:0]  l_cspl_hdr;          // {ovf, 4'd0, cnt[2:0]}
  reg        l_cspl_go;           // cnt>=2 && !poison -> emit this frame
  reg [7:0]  l_cspl_sl [0:3];
  reg [15:0] l_cspl_bk [0:3];
  reg [2:0]  cspl_i;              // emission entry cursor
  reg [7:0]  cur_csl;             // consumed entry (loaded in S_CSPL_LD)
  reg [15:0] cur_cbk;             // {s1, s0} -- the 0x13 carries BOTH (v2.9)

  // v2.5 CMD_CHR_STATE8 / CMD_CHR_SPLITS8 (mapper 4): same latch-at-the-tick +
  // request/consume discipline, payload 8x wider.  l_cwsp_w is the frozen copy
  // the area budget calls out (4 x 64 b = 256 FF); it exists for the same reason
  // l_cspl_bk does -- the serializer must read a stable list while the capture
  // keeps accumulating the NEXT frame.
  reg        l_cwin_en;           // window-vector mapper (4/69/Namco 108) this frame -> emit 0x14
  reg [63:0] l_cwin;              // the frame-close window vector
  reg [7:0]  l_cwin_flags;        // 0x14 flags byte (bit0 = CHR-RAM)
  reg [2:0]  l_cwsp_cnt;          // 0..4
  reg [7:0]  l_cwsp_hdr;          // {ovf, 4'd0, cnt[2:0]}
  reg        l_cwsp_go;           // cnt>=2 -> emit 0x15 this frame
  reg [7:0]  l_cwsp_sl [0:3];
  reg [63:0] l_cwsp_w  [0:3];
  reg [2:0]  cwsp_i;              // emission entry cursor
  reg [7:0]  cur_cwsl;            // consumed entry scanline (S_CWSP_LD)
  reg [63:0] cur_cwin;            // consumed entry vector   (S_CWSP_LD)

  // Fase 3 CMD_PPU_SPLITS 0x16: same latch-at-the-tick + request/consume
  // discipline as the 0x13.  l_psp_pay[0] is NOT a plain copy of the capture's
  // entry 0 -- see the tick.
  reg [2:0]  l_psp_cnt;           // 1..3  (cnt==1 is LEGAL for this command)
  reg [7:0]  l_psp_hdr;           // {ovf, 4'd0, cnt[2:0]}
  reg        l_psp_go;            // cnt>=2 || entry0 code non-classic
  reg [7:0]  l_psp_sl [0:2];
  reg [4:0]  l_psp_py [0:2];      // 5 bits: 7:5 of the byte are padding
  reg [2:0]  psp_i;               // emission entry cursor
  reg [7:0]  cur_psl;             // consumed entry (loaded in S_PSP_LD)
  reg [4:0]  cur_ppy;

  // 4-bit nametable code -> the 2-bit FRAME_HDR.flags[5:4] enum (0=H, 1=V,
  // 2=1A, 3=1B).  ONE HAND-WRITTEN TABLE, NOT derived at runtime, and mirrored
  // in bridge_sim/mappers.py NTCODE_LEGACY, mmu.v and nes_render.a65.  The
  // naive rule "the arrangement that gets the most quadrants right" sends the
  // H-inverted code 0x3 to V, and swapping the AXIS is worse than swapping the
  // PAGE -- hence 0x3 -> H and 0x5 -> V by hand.
  function [1:0] psp_legacy_of;
    input [3:0] c;
    case (c)
      4'h0: psp_legacy_of = 2'd2;   // 1A
      4'h1: psp_legacy_of = 2'd2;
      4'h2: psp_legacy_of = 2'd2;
      4'h3: psp_legacy_of = 2'd0;   // H inverted -> H (own axis)
      4'h4: psp_legacy_of = 2'd2;
      4'h5: psp_legacy_of = 2'd1;   // V inverted -> V (own axis)
      4'h6: psp_legacy_of = 2'd0;
      4'h7: psp_legacy_of = 2'd3;
      4'h8: psp_legacy_of = 2'd2;
      4'h9: psp_legacy_of = 2'd0;
      4'ha: psp_legacy_of = 2'd1;   // V
      4'hb: psp_legacy_of = 2'd3;
      4'hc: psp_legacy_of = 2'd0;   // H
      4'hd: psp_legacy_of = 2'd3;
      4'he: psp_legacy_of = 2'd3;
      4'hf: psp_legacy_of = 2'd3;   // 1B
    endcase
  endfunction

  // Entry 0 of the 0x16, AFTER the rendering-off fallback (spec SS2.2).  The
  // capture reseeds its entry 0 at every frame_tick with the CLOSE-time
  // payload, so when the display window never ran with rendering on
  // (snap_psp_frozen == 0) the array still holds the value seeded at the
  // PREVIOUS tick -- the previous frame's close, i.e. a ONE-FRAME LAG.  The
  // simulator hits the same hole for a different reason (it ticks per EVENT,
  // not per dot, so a rendering-ON frame with zero events in 0..239 also never
  // seeds) and closes it in finalize_ppu(); measured on l3_finallap frame 5,
  // whose only two writes in the window are both in vblank and whose published
  // b4 came out 0 while the display ran with 1.  Substituting the LIVE payload
  // here is the RTL half of that fix, and it is what makes the fallback and the
  // command agree by construction.
  wire [4:0] psp_pay0_live = {snap_ppuctrl[4], snap_ntcode};
  wire [4:0] psp_pay0_eff  = snap_psp_frozen ? snap_psp_pay[4:0] : psp_pay0_live;
  // "classic" = one of the four arrangements flags[5:4] can name.  Anything
  // else has NO other channel, which is the whole reason cnt==1 can emit.
  wire psp_classic0 = (psp_pay0_eff[3:0] == 4'h0) | (psp_pay0_eff[3:0] == 4'ha)
                    | (psp_pay0_eff[3:0] == 4'hc) | (psp_pay0_eff[3:0] == 4'hf);

  // chr bank prev tracking
  reg        s0_valid, s1_valid;
  reg [7:0]  s0_prev, s1_prev;

  // scan cursors / run builders
  reg [11:0] nt_i;         // 0..NT_SIZE
  reg        nt_inrun;
  reg [10:0] run_start;
  reg [8:0]  run_len, run_k;
  reg [8:0]  oam_i;        // 0..256
  reg [5:0]  pal_i;

  // clear-pass cursor (NT dirty bitmap only since v2.4)
  reg [11:0] clr_nt;

  // ---- CHR capture / emission state (see the CHR MIRROR + PENDING block) ----
  // TAP side (all updated in ONE cycle per chr_we, no sequencer):
  reg [12:0] cb_wp;        // free-running count of ACCEPTED tap bytes
  reg [7:0]  dsc_wp;       // descriptor ring head
  reg        cb_open;      // a run is open (cleared at every frame tick)
  reg [7:0]  cb_len;       // bytes in the open run (slice at 255)
  reg [12:0] cb_next_off;  // CHR offset a sequential write must carry
  reg        cb_ovf;       // sticky: a RUN was dropped (descriptor ring full)
  // FROZEN slice, latched at the tick:
  reg [7:0]  l_dsc_end;    // one past the last descriptor of the closing frame
  reg [12:0] l_cb_end;     // accepted-byte count at the close of the frame
  // SERIALIZER side -- FRESH walk (write-order descriptors, byte-exact):
  reg [7:0]  dsc_i;        // descriptor cursor (== the ring tail between frames)
  reg [12:0] cur_coff;     // consumed descriptor: CHR offset
  reg [12:0] cur_ccnt;     // consumed descriptor: byte count at run OPEN
  reg [12:0] cur_clen;     // derived length (next cnt - this cnt), <=255
  reg [7:0]  crun_k;       // payload byte cursor inside the current run
  reg [12:0] chr_emit;     // CHR payload bytes shipped by THIS frame (breadcrumb)
  // The mirror is read at a REGISTERED address, shared by both walks.  Nothing
  // combinational reaches chrmem's address port (SERIALIZER TIMING) and the two
  // walks can never be active at once, so one cursor is enough.
  reg [12:0] chr_ra_r;
  // SERIALIZER side -- PENDING re-send scan (tile-granular, from the mirror):
  reg [9:0]  cp_i;         // tile cursor, 0..CHR_TILES
  reg        cp_inrun;     // a re-send run is open
  reg [8:0]  cp_start;     // first tile of the open run
  reg [4:0]  cp_len;       // open run length in TILES (1..CHR_PEND_MAXT)
  reg [7:0]  cp_bk;        // payload byte cursor inside the re-send run
  // CIRCULAR START.  The scan walks tiles in ADDRESS order, and a budget that
  // stops at 48 tiles would re-send THE SAME LOWEST 48 on every recovery: while
  // the consumer is parked the ACK never confirms, generation A never dies, and
  // the tiles above them are never reached.  That is head-of-line starvation --
  // the exact failure mode that blacked out the device with the ring (the
  // "re-ships the same oldest bytes for ever" note).  So the pass STARTS at
  // cp_base -- where the previous one ran out of budget -- and WRAPS, visiting
  // all CHR_TILES cells exactly once.  Two properties, and the gate needs both:
  //   * fairness: successive recoveries cover DIFFERENT tiles, so a whole-bitmap
  //     debt drains in ceil(512/48) = 11 recovery frames instead of never;
  //   * COMPLETENESS: a debt that FITS in one budget is delivered by ONE pass,
  //     wherever it sits.  An earlier cut gated delivery on `tile >= cp_base`
  //     instead of wrapping, and that lost 27 contiguous tiles on the real trace
  //     (Fantasy Zone, no stall): the debt was below the cursor, the pass
  //     refused it, and no further recovery ever came to pick it up.
  reg [8:0]  cp_base;      // first tile of THIS pass (wraps)
  reg [8:0]  cp_nresume;   // ... where the next pass will pick up
  reg [12:0] chr_pend_emit;// bytes CHARGED to this frame's re-send budget
  reg [12:0] chr_pend_allow;// ... and the allowance, latched at the tick
  // ⚠️ REDUNDANT WITH pend_owed since v2.8, and kept on purpose as defence in
  // depth -- NOT because it is still load-bearing.  pend_owed (resync_en &
  // saw_ack & chr_debt) already covers every frame this term fires on; the two
  // are ORed into cp_deliver, so the older term can only ever agree.  If one of
  // them is ever removed, remove THIS one, and do not read its survival as
  // evidence that the debt logic needs two triggers.
  reg        pend_drain;   // deliver on a NON-recovery frame (debt unconfirmed)
  // v2.8 DEBT.  chr_debt is "some tile still has D set", maintained EXACTLY (the
  // scan visits every cell, so the pass recomputes it from cp_dany).  It is what
  // ends the drain: the window used to be the ACK's (recov_active, killed by
  // caught_w/confirm_a_w), and with an ACK one frame behind -- the NORMAL regime
  // -- that window closed after one recovery plus one drain with 128 tiles still
  // marked and never sent.  Now the drain ends when the DEBT is paid, not when
  // the ACK bookkeeping says so.
  reg        chr_debt;     // some tile is owed-but-unsent (D set somewhere)
  reg        cp_dany;      // this pass left a D set (recomputes chr_debt)
  // v2.8 DEFERRED-FRESH backlog.  CHR_WSTOP can cut the FRESH walk short; the
  // bytes keep their place in the descriptor ring and go out next frame, but
  // l_cb_end has already moved, so the next frame's allowance only discounts the
  // NEW capture and the frame then ships deferred + new + resend against a
  // budget sized for one of the three.  Sticky flag -> the next frame's
  // allowance is ZERO, which is the honest answer (that frame owes the renderer
  // a whole backlog already).
  reg        chr_defer_r;
  // TILE accounting for the allowance (see CHR_TILE_CAP).  Counting at the tap
  // is what makes a partial-tile write cost a whole descriptor, exactly as it
  // does at the renderer.  A tile touched by two separate runs counts twice --
  // over-counting shrinks the allowance, which is the safe direction.
  reg [8:0]  cb_tiles;     // tiles touched since the last tick (saturates)
  reg [8:0]  cb_last_tile; // ... last tile seen, to detect the transition
  reg        cb_tile_vld;
  reg        cp_any;       // some pend bit survived this pass (drives chr_pend_nz)
  reg        chr_pend_nz;  // chrpend may hold a set bit -- when 0 the scan is skipped
  reg        chr_scrub_r;  // a generation was invalidated -> materialise it
  reg        chr_forceall; // descriptor overflow -> mark EVERY tile pending
  reg        l_forceall;   // ... latched for the frame the scan runs in
  reg        chr_scan_go;  // run the pend scan in THIS frame (latched at the tick)
  // ================== v2.7 CHR RE-SEND FROM THE MIRROR ======================
  // WHY.  Until v2.5 the drain cursor advanced as the serializer emitted, so a
  // frame the renderer never applied took its CHR bytes with it: NT and palette
  // had pend/recovery, CHR had NOTHING.  Gate 2.4 hardware: Mega Man (mapper 2,
  // pure CHR-RAM) drew a permanently corrupt background with 79 overruns in
  // ~33 s -- every skipped frame silently deleted tiles, and a ONE-SHOT boot
  // dump never comes back.  v2.6 answered with an ACK-GATED RING: hold the
  // payload until an ACK proves delivery, rewind on a recovery.  That worked
  // for ordinary skips and FAILED for the case it was written for -- a consumer
  // parked INSIDE a dump -- because holding costs CAPACITY, and the valve that
  // protects capacity (the window TRIM) gives up by DISCARDING.  Loss was
  // therefore still permanent, just rarer: run_chrburst reports 32-460 B gone
  // in four cells of the 820-900 x 6/30/60 matrix.
  //
  // MECHANISM.  The payload is no longer queued, it is MIRRORED (chrmem), and
  // what is owed is a BITMAP, not bytes:
  //   * the FRESH walk is unchanged in shape and in output -- it follows the
  //     write-order descriptors of the closing frame and emits their bytes,
  //     which is what keeps every golden byte-identical.  As it ships a byte it
  //     marks that byte's TILE in chrpend generation B: "the renderer owes me
  //     an ACK for this tile".
  //   * the PEND SCAN (S_CP*) walks the 512 tiles in address order.  On a
  //     RECOVERY frame (the same recovery_now_w = lost_r | ack_skip that arms
  //     the NT union) it emits every pending tile straight from the mirror, up
  //     to the frame's remaining allowance, and DEFERS the rest by leaving bits
  //     set.  On any other frame it only MATERIALISES an invalidation.
  //   * confirmation is the ntpend algebra, shared registers and all: a tick
  //     that catches up (ack==seq, no skip) kills B, an ack at/after the
  //     sealing recovery kills A.
  // Runs are idempotent at the renderer (nes_render.a65 nes_chr_handle_run:
  // MVN into nes_chr_shadow, then reconvert the touched tiles), so a redundant
  // re-emission is a no-op and the re-send needs no ordering guarantee at all.
  //
  // WHY THIS CANNOT LOSE WHAT THE RING LOST.  The ring lost bytes because it
  // had to choose between keeping them and having room for new ones.  The
  // mirror has no such choice: a tile's bit stays set until an ACK proves
  // delivery, and the DATA is in an absolute store that the tap keeps current.
  // Deferring costs latency (48 tiles per recovery frame), never content.
  //
  // BYTE IDENTITY.  Delivery is gated on lost_hold, which needs
  // recovery_now_w = resync_en & saw_ack & (lost_r | ack_skip); the byte-exact
  // gate ties resync_en=0 and never ACKs, so no tile is ever delivered and the
  // scan itself is skipped (chr_scan_go stays 0 -- nothing invalidates while
  // ack is frozen at 0), which means the goldens do not even pay its cycles.
  // The fresh walk emits exactly what v2.5/v2.6 emitted with the tail following
  // the drain, which is what resync_en=0 already did.
  //
  // ACK TEAR: the renderer writes NES_FRAME_ACK as TWO byte stores ($2BD4 lo,
  // $2BD5 hi -> main.v nes_frame_ack[7:0]/[15:8]), so a read landing between
  // them samples {old_hi, new_lo}.  Nothing here indexes the ack any more (the
  // publish history is gone), so the only consumer is the plausibility filter
  // below, which rejects the tear as an out-of-range jump.
  //
  // ROOM.  The mirror cannot run out -- it is indexed by offset -- so the only
  // capacity left is the DESCRIPTOR ring, and it only has to hold ONE frame.
  // The margin covers the 1-descriptor look-ahead dsc_i takes while emitting.
  wire [7:0]  dsc_used  = dsc_wp - dsc_i;
  wire        cb_room   = (dsc_used < (DSC_N - 4));
  // reported occupancy (breadcrumb/tb only; nothing in the logic reads it)
  wire [12:0] cb_used   = {5'd0, dsc_used};
  // A write opens a NEW run when there is none, when it is not the sequential
  // successor of the last one, when the open run already hit the 255-byte
  // ceiling of the len field, or when the offset WRAPPED to 0.
  //
  // PROTOCOL INVARIANT (do not regress): **off + len <= 0x2000, ALWAYS.**
  // `cb_next_off <= chr_off + 1` is 13 bits, so a write at 0x1FFF leaves
  // cb_next_off == 0x0000 and a following write at 0x0000 looked SEQUENTIAL --
  // the run then described bytes PAST the end of the 8 KiB CHR-RAM and the
  // renderer would apply the tail somewhere it must not (measured in the first
  // v2.4 goldens: battletoads frame 8 shipped `off=0x1FEF len=255`, 238 bytes
  // past the end; castlevania 121 bytes).  0x1FFF->0x0000 is the ONLY wrap
  // point of a monotonically incrementing 13-bit offset, so testing
  // `chr_off == 0` while a run is open covers the general case -- it is not a
  // special case of the len==255 slicing.  Mirrored EXACTLY by
  // bridge_sim/ppustate.py (which breaks the run when flat wraps to 0).
  wire        cb_newrun = ~cb_open | (chr_off != cb_next_off)
                        | (chr_off == 13'd0) | (cb_len == 8'd255);

  // OAM copy cursor + engine (decoupled from the serialize FSM -- v1.4b).
  reg [8:0]  cpy_i;
  reg        cpy_run;    // a live->frz copy is in flight (triggered by oam_freeze)

  // ============================================================ read plumbing
  // Live and frozen banks have SEPARATE read addresses (they can be active in
  // the same cycle: tap RMW on the live bank while the serializer scans the
  // frozen one).  All reads registered (M9K template).
  reg        ntdirty0_q, ntdirty1_q;
  reg [1:0]  ntpend_q;
  reg [7:0]  ciram_q, oam_q, oam_frz_q;
  reg [5:0]  pal_q;        // registered read of pal[pal_i] (S_PAL*_RD/_WR pair)
  reg [25:0] chrdsc_q;     // registered read of chrdsc[dsc_i]  (S_CR0/1, S_CR2/3)
  reg [7:0]  chrmem_q;     // registered read of chrmem[chr_ra_r] (S_CR*/S_CP*)
  // NOTE (fragile property, written down on purpose): chrpend_q is consumed
  // ONLY in S_CP1, one cycle after S_CP0 issued the address, and S_CP0 itself
  // never looks at it -- so the read has exactly one consumer and no bypass is
  // needed.  If a future state ever reads chrpend_q in the same cycle a scan
  // write lands on the same cell, add the read-during-write bypass that chrmem
  // uses below.
  reg [2:0]  chrpend_q;    // registered read of chrpend[cp_a]  (S_CP0/S_CP1)

  // nametable RMW tap phase
  reg        nt_rmw;
  reg [10:0] nt_rmw_addr;
  reg [7:0]  nt_rmw_data;

  wire [10:0] nt_tap_ra  = nt_we ? nt_addr : nt_rmw_addr;   // live-bank read (RMW check)
  wire [10:0] nt_scan_ra = nt_i[10:0];                      // frozen-bank read (scan)
  wire [10:0] ciram_ra   = run_start + run_k[7:0];          // serializer run data
  wire        ntdirty_live_q = live ? ntdirty1_q : ntdirty0_q;
  wire        ntdirty_frz_q  = live ? ntdirty0_q : ntdirty1_q;
  wire        nt_pend_eff    = lost_hold & ((pend_avf & ntpend_q[1])  | (pend_bvf & ntpend_q[0]));

  // ---- tick-time generation decisions (values are stable in the accepting
  // S_IDLE cycle; all consumers are registered there).  recov_seq tracks the
  // FIRST unconfirmed recovery: later recoveries deliver A|B WITHOUT
  // re-sealing (seal only when A is empty/just-confirmed), otherwise the ack
  // -- always behind while frames are big -- chases a recov_seq that re-arms
  // every tick and A never dies (the degenerate chain reborn; found by mode
  // "bootskip").  confirm_a is guarded by ~ack_skip: an ack that JUMPED past
  // a skipped recovery must not confirm it (the union it carried was never
  // applied); the skip arms a fresh recovery on this same tick instead.
  wire [15:0] ack_minus_recov = frame_ack_i - recov_seq;
  wire        confirm_a_w  = recov_active & ~ack_skip & ~ack_minus_recov[15];
  wire        caught_w     = (frame_ack_i == frame_seq_o) & ~ack_skip;
  wire        avf_next_w   = pend_a_valid & ~caught_w & ~confirm_a_w;
  wire        bvf_next_w   = pend_valid   & ~caught_w;
  wire        recovery_now_w = resync_en & saw_ack & (lost_r | ack_skip);
  wire        seal_now_w   = recovery_now_w & ~avf_next_w;
  // v2.7: CHR now shares these three terms verbatim -- recovery_now_w arms the
  // pend delivery, seal_now_w decides whether it seals generation A, and the
  // avf/bvf pair is latched at the tick for BOTH scans.  The v2.6 commit /
  // rewind / trim decisions that used to be derived here are gone with the
  // ring: there is no payload to retire, so an ACK has nothing to authorise
  // beyond killing a generation.
  wire [15:0] ack_fwd_w    = frame_ack_i - ack_prev;   // sized: modular
  // PLAUSIBILITY IS A WINDOW, NOT A CONSTANT.  The first cut tested "moved
  // forward by <= 64 and sits within 64 of the published seq", and that constant
  // is a LATCH-UP: one ACK step past 64 frames (a renderer paused >1.07 s -- the
  // in-game shell, a savestate, staging a manual page, any SD operation, all of
  // which the NES core free-runs through) leaves ack_prev frozen, every later
  // ACK even further away, and the test can never become true again short of a
  // reset.  Measured on a 200-frame stall: ack_prev frozen, 240000 rejects, and
  // -- far worse than the CHR path -- ack_skip permanently dead, which disarms
  // the skip branch of the NT recovery algebra that IS hardware-validated.
  // The legal range of an ACK is [ack_prev, frame_seq_o]: it never runs backwards
  // and never gets ahead of what has been published.  Expressed that way the
  // window GROWS as frames are published, so any genuine value -- a 200-frame
  // catch-up included -- falls inside it, while the {old_hi,new_lo} tear of a
  // high-byte rollover reads as ~255 BACKWARDS and stays outside.
  wire [15:0] ack_span_w   = frame_seq_o - ack_prev;
  wire        ack_plaus_w  = (ack_fwd_w != 16'd0) & (ack_fwd_w <= ack_span_w);
  // ONE SAMPLE, ONE PIPELINE.  NES_FRAME_ACK is written by the renderer as TWO
  // byte stores, so a value read between them is a TEAR.  The ack is sampled
  // into a 3-deep pipeline and only acted on while every stage agrees
  // (ack_stable_w); together with the plausibility window below that is what
  // makes a tear be DISCARDED instead of scored as a skip.  v2.6 also fed an
  // 8-deep publish history and a commit/trim decision off this pipeline; that
  // whole cone is gone with the ring, and with it the -3.383 ns path
  // nes_frame_ack[*] -> cb_rp[*] the pipeline was introduced to break.
  reg [15:0] ack_s0, ack_s1, ack_s2;
  // NB: this block owns ack_s* COMPLETELY, reset included.  Splitting a reg's
  // reset into the main FSM block and its updates into this one is two always
  // blocks driving one reg: iverilog simulates it happily and quartus_map
  // refuses it outright ("Can't resolve multiple constant drivers"), which is
  // the house gotcha this file already documents for debug taps.
  always @(posedge clk) if (rst) begin
    ack_s0<=16'd0; ack_s1<=16'd0; ack_s2<=16'd0;
  end else begin
    ack_s0 <= frame_ack_i;
    ack_s1 <= ack_s0;
    ack_s2 <= ack_s1;
  end
  // the whole pipeline must describe one settled ack value
  wire        ack_stable_w = (frame_ack_i == ack_s0) & (ack_s0 == ack_s1)
                           & (ack_s1 == ack_s2);
  // ---- pending-scan cell algebra (registers only -- SERIALIZER TIMING) ------
  // A_eff/B_eff materialise the tick-latched generation validity into the two
  // stored bits; l_forceall is the descriptor-overflow escape, which marks
  // every tile as young.  cp_send is the DELIVERY decision and it is what the
  // seal is qualified on: a tile the budget did not carry must not be sealed.
  // The tile this step of the scan is on: the pass starts at cp_base and wraps
  // (cp_i is a COUNT, not an index), which is what makes one pass visit every
  // cell exactly once while still starting where the last budget ran out.
  wire [8:0]  cp_a      = cp_base + cp_i[8:0];
  // D is deliberately NOT gated by pend_avf/pend_bvf: it is not an ACK state.
  wire        cp_d_eff  = chrpend_q[2];
  wire        cp_a_eff  = pend_avf & chrpend_q[1];
  wire        cp_b_eff  = (pend_bvf & chrpend_q[0]) | l_forceall;
  wire        cp_eff    = cp_d_eff | cp_a_eff | cp_b_eff;
  // ROOM.  Two v2.8 corrections, both measured:
  //  * `emit < allow` OVERSHOOTS by one tile -- it lets emit reach allow+15, and
  //    that is how a 2120 B fresh frame plus a 960 B resend made 3080 > the
  //    ceiling.  The test has to be "does the NEXT tile still fit".
  //  * the mailbox floor is a RESERVE, not the raw CHR_WSTOP: a recovery with a
  //    large NT union (<=4448 B) plus a full 3072 B resend plus headers reaches
  //    ~7728 > 7600, so the resend would push the whole FRESH slice past the
  //    valve -- and the fresh slice is what fills the descriptor ring.  1 KiB is
  //    above the worst fresh frame the corpus produces after the NT union.
  wire        cp_room   = ((chr_pend_emit + 13'd16) <= chr_pend_allow)
                        & (wptr < (CHR_WSTOP - 13'd1024));
  // DELIVERY IS NOT LIMITED TO RECOVERY FRAMES.  A recovery arms on an edge
  // (lost_r / ack_skip) and a big loss outlives it: the sealed debt would sit in
  // generation A with nothing left to ship it.  So delivery also runs while that
  // seal is UNCONFIRMED (recov_active), which is a self-terminating window --
  // the confirmation that kills A is exactly what ends it.
  // A FRAME DELIVERS while a recovery is armed, while a sealed recovery is
  // unconfirmed, OR while there is DEBT.  The third term is the v2.8 fix: the
  // first two are ACK windows and they close long before a multi-budget debt is
  // paid.  All three are resync_en-gated (lost_hold/pend_drain through
  // recovery_now_w, pend_owed explicitly), so the byte-exact gate never
  // delivers and never even sets D.
  wire        pend_owed  = resync_en & saw_ack & chr_debt;
  wire        cp_deliver = lost_hold | pend_drain | pend_owed;
  wire        cp_send    = cp_deliver & cp_eff & cp_room;
  // The cell rewrite, as three wires so the two S_CP1 branches cannot drift.
  // D: cleared by delivery, otherwise kept -- and SET when this frame could
  //    deliver, the tile was owed, and the budget said no.  l_forceall enters
  //    here too, which is what makes the descriptor-overflow escape actually
  //    ship (in v2.7 it marked 512 tiles that nothing was left to deliver).
  wire        cp_dnext   = cp_send ? 1'b0
                         : (cp_d_eff | l_forceall | (cp_deliver & cp_eff));
  wire        cp_anext   = seal_hold ? cp_send : cp_a_eff;
  wire        cp_bnext   = seal_hold ? 1'b0    : cp_b_eff;
  // ---- pending-scan SCHEDULING (consumed at the tick) ----------------------
  // chr_inval_w = a generation just lost its validity, so the bits that carry
  // it have to be physically rewritten before S_FINISH re-arms the flag.
  wire        chr_inval_w = (pend_a_valid & ~avf_next_w)
                          | (pend_valid   & ~bvf_next_w);
  // the drain window, evaluated at the tick like every other generation term
  wire        pend_drain_w = resync_en & saw_ack & recov_active
                           & ~caught_w & ~confirm_a_w;
  wire        chr_scan_w  = (chr_pend_nz & (recovery_now_w | pend_drain_w
                                            | chr_scrub_r | chr_inval_w))
                          | chr_forceall | pend_owed;

  // port-A write nets to shadow BRAMs.
  // Per ping-pong bank there is exactly ONE writer per cycle: the tap writes
  // the live bank, the clear pass writes the frozen bank -- the `live` muxes
  // below can never collide.
  reg [10:0] ntb_addr;  reg ntb_cwe; reg [7:0] ntb_cwr; reg nttap_dwe;
  // Clear-pass strobes AND addresses registered TOGETHER (do not regress): the
  // strobe fires one cycle after S_CLEAR schedules it, so using the (already
  // incremented) clr_* counters combinationally in the write mux cleared
  // addresses 1..N instead of 0..N-1.  On the NT bank the counter wrap
  // (2048[10:0]==0) hid it by luck; the SAME bug on the (now removed) CHR
  // dirty bitmap left index 512 off the array so TILE 0 WAS NEVER CLEARED and
  // a stale CMD_CHR_DIRTY of tile 0 was re-emitted forever (caught by the
  // megaman2/metroid gate FAILs right after the ping-pong refactor).  Kept
  // written down because the NT half of the pattern is still live.
  reg        ntclr_we;
  reg [10:0] ntclr_a;

  // bank-0/1 single write ports (mux by role)
  wire        nt0_we = live ? ntclr_we   : nttap_dwe;
  wire [10:0] nt0_wa = live ? ntclr_a    : ntb_addr;
  wire        nt0_wd = live ? 1'b0       : 1'b1;
  wire        nt1_we = live ? nttap_dwe  : ntclr_we;
  wire [10:0] nt1_wa = live ? ntb_addr   : ntclr_a;
  wire        nt1_wd = live ? 1'b1       : 1'b0;

  // mailbox write
  reg        mb_we; reg [12:0] mb_waddr; reg [7:0] mb_wdata;

  // ------------- helper: what byte to write this cycle (drives mb_* + xor) -----
  // We use a task-like inline via mb_put(byte): set mb_we/mb_waddr/mb_wdata, bump
  // wptr and xor.  Implemented by assigning in each state (non-blocking).
  // (kept inline below for clarity.)

  always @(posedge clk) begin
    // default strobes
    mb_we        <= 1'b0;
    ntb_cwe      <= 1'b0;
    nttap_dwe    <= 1'b0;
    ntclr_we     <= 1'b0;
    frame_done_o <= 1'b0;

    // registered reads (comb address; live/frozen banks read in parallel)
    ntdirty0_q  <= ntdirty0[live ? nt_scan_ra : nt_tap_ra];
    ntdirty1_q  <= ntdirty1[live ? nt_tap_ra  : nt_scan_ra];
    ntpend_q    <= ntpend[nt_scan_ra];
    ciram_q     <= ciram[ciram_ra];
    oam_q       <= oam[cpy_i[7:0]];        // copy source (S_CPY)
    oam_frz_q   <= oam_frz[oam_i[7:0]];    // serializer source (S_OAM*)
    pal_q       <= pal[pal_i[4:0]];        // register-array mux lands in a register
    // CHR arrays, simple-dual-port template (sync read here, sync write in the
    // tap / the FSM below).  chrmem is read at a REGISTERED address, so nothing
    // combinational reaches its address port; chrdsc is read one descriptor
    // ahead of the emitting cursor (the look-ahead at dsc_i==l_dsc_end is
    // discarded in favour of l_cb_end); chrpend follows the S_NTA/S_NTB
    // request/consume shape.  The tap writes chrmem/chrdsc and the FSM writes
    // chrpend -- one writer per array per cycle, never two.
    chrdsc_q    <= chrdsc[dsc_i];
    // READ-DURING-WRITE, made DETERMINISTIC (v2.8).  The tap writes chrmem in
    // every cycle it fires and the serializer reads it in the same cycle; on
    // hardware the M9K's mixed-port behaviour for that collision is "old data",
    // while iverilog hands back new data -- a simulation/silicon split on a byte
    // that is being re-sent.  The bypass makes both say NEW, which is also the
    // semantically right answer for a last-wins mirror.  This is the classic
    // single-array bypass template: it does NOT mux two array reads, so the M9K
    // INFERENCE rule in the header still holds.
    chrmem_q    <= (chr_we && (chr_off == chr_ra_r)) ? chr_data
                                                    : chrmem[chr_ra_r];
    chrpend_q   <= chrpend[cp_a];

    if (rst) begin
      st<=S_IDLE; frame_seq_o<=0; frame_len_o<=0; status_o<=0;
      live<=0; tick_pend<=0; tick_accept<=0; saw_ack<=0; lost_r<=0; force_full<=0;
      lost_hold<=0; seal_hold<=0; pend_valid<=0; pend_a_valid<=0;
      pend_avf<=0; pend_bvf<=0; pal_pend_a<=0; pal_pend_b<=0;
      recov_active<=0; recov_seq<=0; ack_prev<=0; ack_skip<=0;
      nt_cnt0<=0; nt_cnt1<=0; chr_any0<=0; chr_any1<=0;
      pal_dirty0<=0; pal_dirty1<=0; pal_cnt0<=0; pal_cnt1<=0; nt_rmw<=0;
      s0_valid<=0; s1_valid<=0; bc_bytes_last<=0; bc_frames<=0; bc_overruns<=0;
      dbg_pal_sum<=0; dbg_pal_wcnt<=0;
      l_split_cnt<=3'd0; l_split_hdr<=8'd0; spl_i<=3'd0;
      l_cspl_cnt<=3'd0; l_cspl_hdr<=8'd0; l_cspl_go<=1'b0; cspl_i<=3'd0;
      l_psp_cnt<=3'd0; l_psp_hdr<=8'd0; l_psp_go<=1'b0; psp_i<=3'd0;
      l_cwin_en<=1'b0; l_cwin<=64'd0; l_cwin_flags<=8'd0;
      l_cwsp_cnt<=3'd0; l_cwsp_hdr<=8'd0; l_cwsp_go<=1'b0; cwsp_i<=3'd0;
      cpy_run<=0; cpy_i<=9'd0;
      cb_wp<=13'd0; dsc_wp<=8'd0; dsc_i<=8'd0;
      cb_open<=1'b0; cb_len<=8'd0; cb_next_off<=13'd0; cb_ovf<=1'b0;
      l_dsc_end<=8'd0; l_cb_end<=13'd0; crun_k<=8'd0; chr_ra_r<=13'd0;
      cur_coff<=13'd0; cur_ccnt<=13'd0; cur_clen<=13'd0;
      cp_i<=10'd0; cp_inrun<=1'b0; cp_start<=9'd0; cp_len<=5'd0; cp_bk<=8'd0;
      cp_base<=9'd0; cp_nresume<=9'd0;
      chr_pend_emit<=13'd0; chr_pend_allow<=13'd0; pend_drain<=1'b0; cp_any<=1'b0;
      chr_debt<=1'b0; cp_dany<=1'b0; chr_defer_r<=1'b0;
      cb_tiles<=9'd0; cb_last_tile<=9'd0; cb_tile_vld<=1'b0;
      // THE PENDING BITMAP MUST BE MATERIALLY CLEARED, not just declared empty.
      // rst pulses on the SNES reset strobe / IGR, which zeroes frame_seq_o and
      // the renderer-written ack; stale tile bits would make the first
      // post-reset recovery re-ship tiles from another life.  The array is an
      // M9K, so it CANNOT be swept with a for-loop in this reset branch (a
      // 512-wide simultaneous write uninfers the RAM into 1024 FFs -- see M9K
      // INFERENCE).  Instead chr_scrub_r is armed, which makes the first frame
      // after reset run the scan once with both generations invalid: every cell
      // is rewritten to {0,0} and chr_pend_nz falls back to 0.  It costs ~1k
      // cycles, once, and emits nothing.
      chr_pend_nz<=1'b1; chr_scrub_r<=1'b1; chr_forceall<=1'b0;
      l_forceall<=1'b0; chr_scan_go<=1'b0;
      chr_emit<=13'd0;
    end else begin
      tick_accept <= 1'b0;                 // 1-cycle pulse (set at the accept)
      if (frame_tick) tick_pend <= 1'b1;   // never drop a tick (cleared on accept)
      if (frame_ack_i != 16'd0) saw_ack <= 1'b1;
      // LOSS THRESHOLD IS >=3, NOT >=2 (cost a hardware iteration -- periodic
      // ~200ms stutter bursts, +12 overruns every ~3s): the renderer marks a
      // frame "seen" at the poll but only writes ACK at ITS vblank, so in
      // NORMAL operation the observed lag oscillates 1<->2 purely with the
      // (slowly drifting) phase between the bridge tick and the SNES vblank.
      // A >=2 trigger fires on that false positive; each full frame (~2.4KB)
      // lengthens the renderer's apply, which HOLDS the lag at 2 -> a CASCADE
      // of full frames until the phase slips back = the burst.  lag==2
      // sustained means "everything consumed, ACK one vblank behind" (no loss);
      // lag>=3 means a never-seen buffer is about to be overwritten = genuine
      // loss.  Kept CONTINUOUS while >=3 (no one-shot): under newest-wins the
      // renderer consumes the NEWEST frame, so during genuine loss every
      // published frame must be full or the consumed one rebuilds partial
      // state.  A real 1-frame stutter now costs 1-2 full frames, not a storm.
      // Validated by tb_handshake mode "phase" (adversarial ACK phase: zero
      // spurious fulls) + mode "slow" (sustained loss: all consumed are full).
      lost_r <= ((frame_seq_o - frame_ack_i) >= 16'd3);   // registered: tick uses FFs only
      // PLAUSIBILITY FILTER (v2.6c).  NES_FRAME_ACK is written as TWO byte
      // stores ($2BD4 lo, $2BD5 hi -> main.v nes_frame_ack[7:0]/[15:8]), so
      // every time the HIGH byte rolls over the bridge sees the intermediate
      // {old_hi, new_lo} for as long as the renderer takes between the two
      // stores.  Modulo 2^16 that intermediate reads as a jump of 65281, which
      // the bare ">=2" detector scored as a skip -- setting ack_skip AND
      // chain_broken twice, deterministically, every 256 frames (~4.3 s), each
      // one forcing a recovery + rewind nobody asked for.  A real ACK only ever
      // moves FORWARD by a few frames and never runs ahead of what has been
      // published, so anything outside that envelope is discarded whole: no
      // skip, no chain break, and ack_prev is NOT poisoned with the torn value.
      // ack_prev tracks every STABLE, in-range ACK -- including a huge catch-up
      // after a long pause.  A tear is a TRANSIENT and out-of-range value, so it
      // is rejected on both counts; a stall is a stable, in-range one, so it is
      // accepted and scored as the skip it really is.
      if ((frame_ack_i != ack_prev) && ack_stable_w && ack_plaus_w) begin
        // v2.7: this used to break the CHR "everything before the committed
        // tail was applied" chain as well.  There is no chain any more -- the
        // tiles a skipped frame owed are still marked in chrpend and the bytes
        // are still in the mirror -- so a skip only has to ARM the recovery.
        if (saw_ack && (ack_fwd_w >= 16'd2)) ack_skip <= 1'b1;
        ack_prev <= frame_ack_i;
      end
      // (generation-A confirmation is processed AT THE TICK, guarded by
      // ~ack_skip -- see confirm_a_w below.  An async drop here was wrong
      // twice: (1) HISTORY: a single-epoch drop discarded the pending dirtys
      // of frames serialized after the recovery ("background vanishes until
      // the next death") -- generation B now preserves those; (2) an ack for
      // a NORMAL frame past a SKIPPED recovery must not confirm A -- only
      // the tick sees ack_skip and can tell the difference.)

      // -------- port-A writes to shadow BRAMs (bank-muxed single ports) ------
      if (ntb_cwe)  ciram[ntb_addr]  <= ntb_cwr;
      if (nt0_we)   ntdirty0[nt0_wa] <= nt0_wd;
      if (nt1_we)   ntdirty1[nt1_wa] <= nt1_wd;
      if (oam_we)   oam[oam_addr]    <= oam_data;
      // -------- OAM freeze copy engine (v1.4b -- decoupled from the FSM) ------
      // oam_freeze pulses live oam[] -> oam_frz[] over 257 cycles (oam_q is the
      // 1-cycle-registered read of oam[cpy_i]; write lags by one, exactly the
      // old S_CPY pipeline).  In hardware the pulse lands MID-DISPLAY (scanline
      // 120) so the copy is DONE long before the frame tick -> the serializer
      // reads a stable, race-free oam_frz and never overlaps the vblank OAM-DMA.
      if (oam_freeze && !cpy_run) begin cpy_run<=1'b1; cpy_i<=9'd0; end
      else if (cpy_run) begin
        cpy_i <= cpy_i + 9'd1;
        if (cpy_i == 9'd256) cpy_run <= 1'b0;
      end
      if (cpy_run && cpy_i != 9'd0) oam_frz[cpy_i[7:0]-8'd1] <= oam_q;
      if (mb_we) begin
        if (mb_wbuf) mbox1[mb_waddr] <= mb_wdata; else mbox0[mb_waddr] <= mb_wdata;
      end

      // -------- palette fingerprint: INCREMENTAL sum-mod-256 on write ------
      // (synthesis #21: the 32-entry adder tree missed setup by -1.4ns; the
      // delta form is 1 sub + 1 add of 8 bits.  pal[] is deliberately an FF
      // array, so the combinational read below returns the OLD value in the
      // same cycle the tap write lands -- exact running sum of the 32 live
      // entries, same semantics as the tree.)
      if (pal_we) begin
        dbg_pal_sum  <= dbg_pal_sum + {2'b00, (pal_data & 6'h3F)}
                                    - {2'b00, pal[pal_eff]};
        dbg_pal_wcnt <= dbg_pal_wcnt + 8'd1;
      end

      // -------- tap accumulation (ALWAYS ON -- see SNAPSHOT note) --------
      begin
        if (pal_we) begin
          pal[pal_eff] <= pal_data & 6'h3F;
          if (live) begin
            pal_dirty1[pal_eff] <= 1'b1;
            if (!pal_dirty1[pal_eff]) pal_cnt1 <= pal_cnt1 + 6'd1;
          end else begin
            pal_dirty0[pal_eff] <= 1'b1;
            if (!pal_dirty0[pal_eff]) pal_cnt0 <= pal_cnt0 + 6'd1;
          end
        end
        // -------- CHR-RAM tap (v2.7): mirror + run descriptors, 1 cycle ------
        // Everything below is a single-cycle event: at most ONE chrmem write
        // (the data byte) and ONE chrdsc write (only when a run opens), on two
        // DIFFERENT arrays.  That is the whole point of storing `cnt` instead
        // of `len` in the descriptor -- see the CHR MIRROR + PENDING block.
        // A frame_tick is DEFERRED while chr_we is high (S_IDLE accept), so a
        // tap and a tick never race for the same cycle and the byte always
        // belongs to the CLOSING frame -- the same convention nt_rmw uses, and
        // the one the stimulus order (`4` lines before the `5` line) encodes.
        // GONE WITH THE RING: the per-cycle window clamp that used to sit here
        // (outside the `if (chr_we)`, because the window also grew while the
        // SERIALIZER walked).  It existed to stop cb_used_h aliasing past
        // CB_SZ between ticks and to bound a retransmission window this design
        // no longer has -- there is no window, no committed tail and no
        // occupancy that a tap can push past.
        if (chr_we) begin
          if (live) chr_any1 <= 1'b1; else chr_any0 <= 1'b1;
          // THE MIRROR ALWAYS TAKES THE BYTE, room or no room.  It is indexed
          // by offset, so it has no capacity to run out of and no ordering to
          // preserve; this unconditional write is what makes the CHR state
          // idempotent and the re-send lossless.
          chrmem[chr_off] <= chr_data;
          // TILE ACCOUNTING for the resend allowance (see CHR_TILE_CAP).  One
          // count per TRANSITION of chr_off[12:4] -- OR PER NEW RUN, and the
          // second term is not decoration: the renderer accumulates nes_cq_nt
          // PER RUN (nes_render.a65 ~l.8055, the ((off+cl-1)>>4)-(off>>4)+1
          // form at ~l.7874-7887), so a tile that TWO runs straddle costs it
          // TWO descriptor entries.  Counting only transitions of the tile
          // index made this side under-count exactly that case and let the
          // frame exceed the renderer's ceiling: measured 201 tiles against a
          // cap of 192 on l2_senjou with rendering ON, and 200 on megaman1's
          // S1.  cb_newrun is already computed alongside (this adds fan-out,
          // not depth).  Expected effect: 201 -> 192 and 200 -> 192, with
          // loss/latency/bandwidth byte-identical.
          if (!cb_tile_vld || (chr_off[12:4] != cb_last_tile) || cb_newrun) begin
            if (cb_tiles != 9'd511) cb_tiles <= cb_tiles + 9'd1;
            cb_last_tile <= chr_off[12:4];
            cb_tile_vld  <= 1'b1;
          end
          if (!cb_room) begin
            // DESCRIPTOR ring full: drop the RUN (so a loss shows up as a
            // missing run, never as a hole inside one), latch the breadcrumb,
            // and force the next pend scan to mark every tile -- the data is in
            // the mirror, so the next recovery re-ships it instead of losing
            // it, which is strictly better than the v2.6 byte drop.
            // Unreachable in the corpus: worst measured is ~90 descriptors per
            // frame against 256, and they are drained every frame now.
            cb_ovf       <= 1'b1;
            cb_open      <= 1'b0;
            chr_forceall <= 1'b1;
          end else begin
            cb_wp <= cb_wp + 13'd1;
            if (cb_newrun) begin
              chrdsc[dsc_wp] <= {chr_off, cb_wp};   // cnt = PRE-increment count
              dsc_wp         <= dsc_wp + 8'd1;
              cb_len         <= 8'd1;
            end else cb_len <= cb_len + 8'd1;
            cb_open     <= 1'b1;
            cb_next_off <= chr_off + 13'd1;
          end
        end
        if (nt_we && !nt_rmw) begin
          nt_rmw<=1'b1; nt_rmw_addr<=nt_addr; nt_rmw_data<=nt_data;
        end else if (nt_rmw) begin
          ntb_addr<=nt_rmw_addr; ntb_cwr<=nt_rmw_data; ntb_cwe<=1'b1;
          nttap_dwe<=1'b1;
          if (!ntdirty_live_q) begin
            if (live) nt_cnt1 <= nt_cnt1 + 12'd1;
            else      nt_cnt0 <= nt_cnt0 + 12'd1;
          end
          nt_rmw<=1'b0;
        end
      end

      // ============================ serialize FSM =============================
      case (st)
        // tick accept: deferred while an nt RMW is in flight (a coincident tap
        // belongs to the CLOSING frame and must land in the pre-flip bank).
        // v2.4 adds !chr_we for the same reason on the CHR ring: accepting in
        // the same cycle a CHR byte lands would snapshot l_cb_end/l_dsc_end
        // BEFORE that byte's pointer bump, orphaning it between two frames.
        S_IDLE: if ((frame_tick | tick_pend) && !nt_rmw && !nt_we && !chr_we) begin
          tick_pend<= 1'b0;
          tick_accept <= 1'b1;      // reseeds the capture modules, one cycle
                                    // after this latch (see the port comment)
          // v2.4: freeze the CHR payload/descriptor slice of the closing frame
          // and force the next write to open a fresh run (runs NEVER span
          // frames -- lockstep with bridge_sim begin_frame()).
          l_dsc_end<= dsc_wp;
          l_cb_end <= cb_wp;
          cb_open  <= 1'b0;
          // v2.7 PENDING-SCAN SCHEDULING.  The scan (S_CP*) has three jobs: it
          // MATERIALISES a generation invalidation into the bitmap, it DELIVERS
          // the unconfirmed tiles on a recovery frame, and it applies the
          // descriptor-overflow force-mark.  It is skipped whenever there is
          // none of the three to do -- which is EVERY frame of the byte-exact
          // gate (resync_en=0 never recovers, and an ack frozen at 0 never
          // invalidates a generation), so the goldens do not even pay its
          // cycles.
          //
          // THE INVALIDATION MUST BE MATERIALISED IN THE SAME FRAME IT HAPPENS.
          // The valid flags are re-armed unconditionally at S_FINISH
          // (pend_valid <= 1), exactly as NT does, and NT gets away with it
          // because its scan rewrites all 2048 cells EVERY frame.  The CHR scan
          // is conditional, so a generation dropped at the tick and re-armed at
          // S_FINISH would RESURRECT every stale bit still in the array the
          // next time a fresh mark set the flag -- the union would grow without
          // bound, which is precisely the degeneration the two generations
          // exist to prevent.  chr_scrub_r is what carries the request when the
          // bitmap is empty at the moment of the invalidation.
          chr_scan_go <= chr_scan_w;
          chr_scrub_r <= chr_scan_w ? 1'b0 : (chr_scrub_r | chr_inval_w);
          l_forceall   <= chr_forceall;
          chr_forceall <= 1'b0;
          cp_i     <= 10'd0;
          cp_inrun <= 1'b0;
          cp_any   <= 1'b0;
          cp_nresume    <= cp_base;     // no delivery this pass -> same start
          chr_pend_emit <= 13'd0;
          pend_drain    <= pend_drain_w & ~recovery_now_w;
          // What is left of the renderer's TILE ceiling after this frame's
          // fresh capture, in bytes.  Zero when the previous frame left a
          // CHR_WSTOP backlog: that frame already owes the renderer a full
          // slice, so it gets no resend on top.
          chr_pend_allow <= (chr_defer_r | (cb_tiles >= CHR_TILE_CAP)) ? 13'd0
                          : {(CHR_TILE_CAP - cb_tiles), 4'd0};
          cb_tiles     <= 9'd0;
          cb_tile_vld  <= 1'b0;
          chr_defer_r  <= 1'b0;
          cp_dany      <= 1'b0;
          live     <= ~live;                 // flip: filled bank becomes frozen
          new_seq  <= frame_seq_o + 16'd1;
          mb_wbuf  <= ~frame_seq_o[0];
          l_frame  <= snap_frame;
          // FRAME_HDR.flags[5:4], Fase 3 SS2.2: the field now comes from the
          // START OF THE DISPLAY, not the frame CLOSE.  Change of SEMANTICS,
          // not of format -- and it is the v1.2 "the close-time snapshot grabs
          // the HUD" fix in the arrangement dimension: Dragon Buster runs 702
          // of 908 gameplay frames with (display start, close) = (1A, 1B), so
          // the close-time value made the renderer draw the whole frame with
          // the status-bar page.  It is wrong even when cnt == 1, which is why
          // this is NOT gated on the command being emitted.
          //   Fallback when the frame ran its whole display with rendering off
          //   (snap_psp_frozen == 0): the CLOSE-time snap_ntarr, which is
          //   literally today's behaviour => byte-identical.
          //   Residual, documented: in hardware snap_ntarr comes from the mmu's
          //   ntarr_of(raw 2-bit mirror), and for the eight-1KB-window family
          //   (95/118/154/163) that partition rule can name a different
          //   arrangement than psp_legacy_of(nt_snap_code) would.  It only
          //   shows in this rendering-off branch, where nothing is displayed;
          //   and if the live code is NON-CLASSIC the 0x16 is emitted anyway
          //   (psp_classic0 is computed from the SAME psp_pay0_eff) and takes
          //   precedence over the field.
          l_ntarr  <= snap_psp_frozen ? psp_legacy_of(psp_pay0_eff[3:0])
                                      : snap_ntarr;
          l_ntsel  <= snap_loopy_t[11:10];
          l_ppuctrl<= snap_ppuctrl; l_ppumask<= snap_ppumask;
          l_sx <= {snap_loopy_t[4:0],  snap_fine_x};
          l_sy <= {snap_loopy_t[9:5],  snap_loopy_t[14:12]};
          l_s0p<=snap_s0_present; l_s0b<=snap_s0_bank;
          l_s1p<=snap_s1_present; l_s1b<=snap_s1_bank;
          // v2.0a first-valid fix (device-proven on Tetris USA, MMC1-4K): the old
          // `s_valid & (bank!=prev)` SWALLOWED the first sample (present-rise
          // loaded prev without emitting) -- a game entering 4K mode after the
          // ~1-frame boot resync window (Tetris USA: 4K at frame 2, banks (0,0),
          // first real switch only at frame 801) never announced slot 1 -> PT1
          // rendered from the wrong half for ~13s.
          //   slot 1: presence-RISE is itself information (8K->4K; PT1 remaps
          //     even at bank 0) -> ~s1_valid counts as change, ALWAYS.
          //   slot 0: always present; its first-valid compares against the BOOT
          //     BASELINE bank 0 (power-on state of every v0 mapper register ==
          //     the renderer's boot full-CHR upload) -- emits only if the game
          //     already switched during frame 1.  An unconditional ~s0_valid
          //     would inject CHR_BANK(0,0) into frame 1 of EVERY game (goldens
          //     of no-switch games must stay byte-identical).
          // Lockstep: bridge_sim/encoder.py first-seen rule (same two cases).
          l_s0chg<= snap_s0_present & ((s0_valid ? (snap_s0_bank!=s0_prev)
                                                 : (snap_s0_bank!=8'd0))
                                        | (s0_valid & resync_en & (~saw_ack | lost_r | ack_skip)));
          l_s1chg<= snap_s1_present & (~s1_valid | (snap_s1_bank!=s1_prev)
                                        | (resync_en & (~saw_ack | lost_r | ack_skip)));
          // v1.3 CMD_SPLITS: derive sx/sy/ntsel per entry from the raw snapshot
          // (bit-slicing identical to CMD_REGS's l_sx/l_sy) -- pure wiring, no
          // arithmetic; done once/frame here so emission reads registers only.
          l_split_cnt <= snap_split_cnt;
          l_split_hdr <= {snap_split_ovf, 4'd0, snap_split_cnt};
          // v2.3 CMD_CHR_SPLITS: raw (scanline, bank) pairs -- nothing to derive.
          // The EMISSION GATE is latched here too so the FSM branch reads a
          // single register: cnt>=2 (a lone display-start entry is what every
          // non-splitting frame produces) AND !poison (a mid-frame 8K<->4K flip
          // invalidated the strips -- see nes_chrsplit_capture.v).
          l_cspl_cnt <= snap_cspl_cnt;
          l_cspl_hdr <= {snap_cspl_ovf, 4'd0, snap_cspl_cnt};
          l_cspl_go  <= (snap_cspl_cnt >= 3'd2) & ~snap_cspl_poison;
          // v2.5 CMD_CHR_STATE8/SPLITS8 (mapper 4): the 0x14 gate is the mapper
          // itself (unconditional while enabled), the 0x15 gate is cnt>=2 -- no
          // poison term, the window vector has no mode flip to invalidate it.
          l_cwin_en  <= snap_chr_win_en;
          l_cwin     <= snap_chr_win;
          l_cwin_flags <= snap_chr_win_flags;
          l_cwsp_cnt <= snap_cwin_cnt;
          l_cwsp_hdr <= {snap_cwin_ovf, 4'd0, snap_cwin_cnt};
          l_cwsp_go  <= snap_chr_win_en & (snap_cwin_cnt >= 3'd2);
          // Fase 3 CMD_PPU_SPLITS 0x16: raw (scanline, payload) pairs, nothing
          // to derive.  The EMISSION GATE is latched here so the FSM branch
          // reads a single register:
          //     cnt >= 2  OR  entry0's code is NOT classic
          // The second clause is what makes cnt == 1 a legal command -- see the
          // OP_PPU_SPLITS comment.  Both terms use psp_pay0_eff, the
          // rendering-off-corrected entry 0, so the command, its payload and
          // FRAME_HDR.flags[5:4] can never disagree about which value the frame
          // started on.
          l_psp_cnt <= snap_psp_cnt;
          l_psp_hdr <= {snap_psp_ovf, 4'd0, snap_psp_cnt};
          l_psp_go  <= (snap_psp_cnt >= 3'd2) | ~psp_classic0;
          for (si=0; si<4; si=si+1) begin
            l_spl_sl[si] <= snap_spl_sl[si*8 +: 8];
            l_spl_sx[si] <= {snap_spl_t[si*15 +: 5],    snap_spl_fx[si*3 +: 3]};
            l_spl_sy[si] <= {snap_spl_t[si*15+5 +: 5],  snap_spl_t[si*15+12 +: 3]};
            l_spl_nt[si] <= snap_spl_t[si*15+10 +: 2];
            l_cspl_sl[si]<= snap_cspl_sl[si*8 +: 8];
            l_cspl_bk[si]<= snap_cspl_bank[si*16 +: 16];
            l_cwsp_sl[si]<= snap_cwin_sl[si*8 +: 8];
            l_cwsp_w[si] <= snap_cwin_win[si*64 +: 64];
          end
          // The 0x16 arrays are 3 deep (K=3), so they get their own loop rather
          // than riding the 4-deep one above.  Entry 0 takes the
          // rendering-off-corrected payload; 1..2 are raw (they only EXIST when
          // the capture froze, so the correction is a no-op for them by
          // construction -- writing it as a select keeps the loop a single
          // indexed assignment).
          for (pi=0; pi<3; pi=pi+1) begin
            l_psp_sl[pi] <= snap_psp_sl[pi*8 +: 8];
            l_psp_py[pi] <= (pi == 0) ? psp_pay0_eff : snap_psp_pay[pi*5 +: 5];
          end
          force_full <= resync_en & ~saw_ack;          // BOOT only: true full
          lost_hold  <= recovery_now_w;                // recovery: delta union
          seal_hold  <= seal_now_w;                    // this recovery seals A
          if (ack_skip) ack_skip <= 1'b0;              // consumed by this tick
          // Epoch handling (see the GENERATION note + the wires above).
          // caught_w is gated on ~ack_skip: a skip whose catch-up lands
          // exactly on a caught-up tick still needs THIS recovery to carry
          // the union -- ungated, the recovery would be emitted EMPTY and
          // the union dropped = permanent loss of the skipped delta.
          // The clears must be visible to THIS frame's scan (an earlier
          // revision sampled the frame-latch PRE-clear: the union never
          // actually cleared and grew forever).
          pend_avf <= avf_next_w;
          pend_bvf <= bvf_next_w;
          if (caught_w) pend_valid <= 1'b0;
          if (caught_w | confirm_a_w) begin
            pend_a_valid <= 1'b0;                      // re-set by S_FINISH if sealing
            recov_active <= 1'b0;
          end
          // palette pending (plain registers -- same generation algebra,
          // done here since there is no sweep):
          if (seal_now_w) begin
            pal_pend_a <= (bvf_next_w ? pal_pend_b : 32'd0)
                        | (live ? pal_dirty1 : pal_dirty0);
            pal_pend_b <= 32'd0;
          end else begin
            pal_pend_a <= (avf_next_w ? pal_pend_a : 32'd0);
            pal_pend_b <= ((bvf_next_w ? pal_pend_b : 32'd0)
                        | (live ? pal_dirty1 : pal_dirty0));
          end
          // OAM was already frozen by the oam_freeze copy engine (mid-display in
          // hardware; at the tick in the tbs) -> go straight to serialize.
          chr_emit<=13'd0;                   // per-frame CHR shipped counter
          wptr<=0; xor_acc<=0; sub<=0; st<=S_SETUP;
        end

        S_SETUP: begin
          l_flags <= (snap_fb_hint ? FLAG_FORCED_BLANK : 8'd0)
                   | (((nt_cnt_frz > 12'd1536) | force_full) ? FLAG_FULL_REDRAW : 8'd0)
                   | (pal_present ? FLAG_PALETTE_PRESENT : 8'd0)
                   | ((l_s0chg|l_s1chg|chr_any_frz) ? FLAG_CHR_PRESENT : 8'd0)
                   | {2'd0, l_ntarr, 4'd0};   // flags[5:4] = NT arrangement (v1.1)
          st<=S_HDR; sub<=0;
        end

        // -------- FRAME_HDR: 01 seq(2) frame(2) flags(1) --------
        S_HDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_FRAME_HDR;   xor_acc<=xor_acc^OP_FRAME_HDR;   end
            1: begin mb_wdata<=new_seq[7:0];   xor_acc<=xor_acc^new_seq[7:0];   end
            2: begin mb_wdata<=new_seq[15:8];  xor_acc<=xor_acc^new_seq[15:8];  end
            3: begin mb_wdata<=l_frame[7:0];   xor_acc<=xor_acc^l_frame[7:0];   end
            4: begin mb_wdata<=l_frame[15:8];  xor_acc<=xor_acc^l_frame[15:8];  end
            5: begin mb_wdata<=l_flags;        xor_acc<=xor_acc^l_flags;        end
          endcase
          if (sub==5) begin st<=S_REGS; sub<=0; end else sub<=sub+4'd1;
        end

        // -------- REGS: 10 sx(2) sy(2) ctrl mask ntsel --------
        S_REGS: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_REGS;               xor_acc<=xor_acc^OP_REGS; end
            1: begin mb_wdata<=l_sx;                  xor_acc<=xor_acc^l_sx; end
            2: begin mb_wdata<=8'h00;                 xor_acc<=xor_acc^8'h00; end
            3: begin mb_wdata<=l_sy;                  xor_acc<=xor_acc^l_sy; end
            4: begin mb_wdata<=8'h00;                 xor_acc<=xor_acc^8'h00; end
            5: begin mb_wdata<=l_ppuctrl;             xor_acc<=xor_acc^l_ppuctrl; end
            6: begin mb_wdata<=l_ppumask;             xor_acc<=xor_acc^l_ppumask; end
            7: begin mb_wdata<={6'd0,l_ntsel};        xor_acc<=xor_acc^{6'd0,l_ntsel}; end
          endcase
          if (sub==7) begin st<=S_CHRST; sub<=0; end   // v2.2: CHR_STATE sempre
          else sub<=sub+4'd1;
        end

        // -------- CMD_CHR_STATE (v2.2): 12 s0_bank s1_bank {7'd0,s1p} --------
        // UNCONDITIONAL, fixed frame offset 14 -- absolute current bank state;
        // the renderer reconciles (see OP_CHR_STATE comment).  s1_bank is
        // gated by presence (l_s1p ? l_s1b : 0) so the injected-snapshot tb
        // and the real-mmu path (which drives chr_bank_1 raw even in 8K mode)
        // emit IDENTICAL bytes.  mb_wdata reads registers only (l_*).
        S_CHRST: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_CHR_STATE;          xor_acc<=xor_acc^OP_CHR_STATE; end
            1: begin mb_wdata<=l_s0b;                 xor_acc<=xor_acc^l_s0b; end
            2: begin mb_wdata<=(l_s1p ? l_s1b : 8'd0);xor_acc<=xor_acc^(l_s1p ? l_s1b : 8'd0); end
            3: begin mb_wdata<={7'd0,l_s1p};          xor_acc<=xor_acc^{7'd0,l_s1p}; end
          endcase
          if (sub==3) begin
            // v2.5: in mapper 4 the chain goes to CMD_CHR_STATE8 and NEVER to
            // S_CSPL_OP -- that is hard rule (2) of the OP_CHR_STATE8 block,
            // enforced here by the ORDER of this if-chain (and, independently,
            // by the constant legacy tap in mmu.v, which keeps l_cspl_go at 0).
            if (l_cwin_en) begin st<=S_CWST; sub<=0; end  // v2.5: CMD_CHR_STATE8
            else if (l_cspl_go) st<=S_CSPL_OP;            // v2.3: emit CMD_CHR_SPLITS
            else if (l_split_cnt >= 3'd2) st<=S_SPL_OP;   // v1.3: emit CMD_SPLITS
            else if (l_psp_go) st<=S_PSP_OP;             // Fase 3: emit CMD_PPU_SPLITS
            else if (pal_present) st<=S_PAL;
            else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
          end else sub<=sub+4'd1;
        end

        // -------- CMD_CHR_STATE8 (v2.5): 14 win0..win7 flags -----------------
        // UNCONDITIONAL in mapper 4, fixed frame offset 18 (right after the
        // 0x12, which keeps ITS fixed offset 14 so the parser never moves; the
        // renderer ignores the 0x12 once it has seen a 0x14).  mb_wdata reads
        // registers only: a 10-way case over slices of the SINGLE latched
        // register l_cwin (byte-select mux, same class as the 8-way S_REGS one)
        // -- SERIALIZER TIMING.
        S_CWST: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_CHR_STATE8;      xor_acc<=xor_acc^OP_CHR_STATE8; end
            1: begin mb_wdata<=l_cwin[7:0];        xor_acc<=xor_acc^l_cwin[7:0]; end
            2: begin mb_wdata<=l_cwin[15:8];       xor_acc<=xor_acc^l_cwin[15:8]; end
            3: begin mb_wdata<=l_cwin[23:16];      xor_acc<=xor_acc^l_cwin[23:16]; end
            4: begin mb_wdata<=l_cwin[31:24];      xor_acc<=xor_acc^l_cwin[31:24]; end
            5: begin mb_wdata<=l_cwin[39:32];      xor_acc<=xor_acc^l_cwin[39:32]; end
            6: begin mb_wdata<=l_cwin[47:40];      xor_acc<=xor_acc^l_cwin[47:40]; end
            7: begin mb_wdata<=l_cwin[55:48];      xor_acc<=xor_acc^l_cwin[55:48]; end
            8: begin mb_wdata<=l_cwin[63:56];      xor_acc<=xor_acc^l_cwin[63:56]; end
            9: begin mb_wdata<=l_cwin_flags;       xor_acc<=xor_acc^l_cwin_flags; end
          endcase
          if (sub==4'd9) begin
            if (l_cwsp_go) st<=S_CWSP_OP;                 // v2.5: CMD_CHR_SPLITS8
            else if (l_split_cnt >= 3'd2) st<=S_SPL_OP;   // v1.3: CMD_SPLITS
            else if (l_psp_go) st<=S_PSP_OP;             // Fase 3: CMD_PPU_SPLITS
            else if (pal_present) st<=S_PAL;
            else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
          end else sub<=sub+4'd1;
        end

        // -------- CMD_CHR_SPLITS8 (v2.5): 15 hdr cnt x [sl win0..win7] --------
        // Emitted only when l_cwsp_go (mapper 4 && cnt>=2), so a mapper-4 frame
        // with no mid-display window change walks straight from S_CWST to the
        // pre-v2.5 chain.  Same request/consume split as S_CSPL_*: S_CWSP_LD
        // consumes l_cwsp_*[cwsp_i] into cur_cw* (the 4-deep indexed read of a
        // 64-bit array stays OFF the mb_wdata/xor cone), S_CWSP_E emits the 9
        // bytes as a byte-select mux over those single registers.
        S_CWSP_OP: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=OP_CHR_SPLITS8; xor_acc<=xor_acc^OP_CHR_SPLITS8;
          st<=S_CWSP_HDR;
        end
        S_CWSP_HDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=l_cwsp_hdr; xor_acc<=xor_acc^l_cwsp_hdr;
          cwsp_i<=3'd0; sub<=4'd0; st<=S_CWSP_LD;
        end
        S_CWSP_LD: begin
          cur_cwsl<=l_cwsp_sl[cwsp_i[1:0]]; cur_cwin<=l_cwsp_w[cwsp_i[1:0]];
          st<=S_CWSP_E;
        end
        S_CWSP_E: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=cur_cwsl;        xor_acc<=xor_acc^cur_cwsl; end
            1: begin mb_wdata<=cur_cwin[7:0];   xor_acc<=xor_acc^cur_cwin[7:0]; end
            2: begin mb_wdata<=cur_cwin[15:8];  xor_acc<=xor_acc^cur_cwin[15:8]; end
            3: begin mb_wdata<=cur_cwin[23:16]; xor_acc<=xor_acc^cur_cwin[23:16]; end
            4: begin mb_wdata<=cur_cwin[31:24]; xor_acc<=xor_acc^cur_cwin[31:24]; end
            5: begin mb_wdata<=cur_cwin[39:32]; xor_acc<=xor_acc^cur_cwin[39:32]; end
            6: begin mb_wdata<=cur_cwin[47:40]; xor_acc<=xor_acc^cur_cwin[47:40]; end
            7: begin mb_wdata<=cur_cwin[55:48]; xor_acc<=xor_acc^cur_cwin[55:48]; end
            8: begin mb_wdata<=cur_cwin[63:56]; xor_acc<=xor_acc^cur_cwin[63:56]; end
          endcase
          if (sub==4'd8) begin
            // exit = the SAME decision S_CWST took after the last 0x14 byte
            if (cwsp_i+3'd1 >= l_cwsp_cnt) begin
              if (l_split_cnt >= 3'd2) st<=S_SPL_OP;
              else if (l_psp_go) st<=S_PSP_OP;
              else if (pal_present) st<=S_PAL;
              else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
            end else begin cwsp_i<=cwsp_i+3'd1; sub<=4'd0; st<=S_CWSP_LD; end
          end else sub<=sub+4'd1;
        end

        // -------- CMD_CHR_SPLITS (v2.3): 13 hdr(ovf|cnt) cnt x [sl bank] ------
        // Emitted only when l_cspl_go (cnt>=2 && !poison), so a frame with no
        // mid-display CHR bank change walks EXACTLY the pre-v2.3 state chain and
        // its bytes are unchanged.  Same request/consume split as S_SPL_*:
        // S_CSPL_LD consumes l_cspl_*[cspl_i] into cur_c* (indexed array read
        // kept OFF the mb_wdata/xor cone -- SERIALIZER TIMING), S_CSPL_E emits
        // the 2 bytes from those single registers.
        S_CSPL_OP: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=OP_CHR_SPLITS; xor_acc<=xor_acc^OP_CHR_SPLITS;
          st<=S_CSPL_HDR;
        end
        S_CSPL_HDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=l_cspl_hdr; xor_acc<=xor_acc^l_cspl_hdr;
          cspl_i<=3'd0; sub<=4'd0; st<=S_CSPL_LD;
        end
        S_CSPL_LD: begin
          cur_csl<=l_cspl_sl[cspl_i[1:0]]; cur_cbk<=l_cspl_bk[cspl_i[1:0]];
          st<=S_CSPL_E;
        end
        S_CSPL_E: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          // v2.9: THREE bytes per entry -- scanline, slot0 bank, slot1 bank.
          // s1 is ALWAYS emitted, and in 8K mode it carries the same value the
          // 0x12 publishes there (the capture applies `s1_present ? s1 : 0`),
          // so a 0x13 entry and a CMD_CHR_STATE payload are the same tuple.
          // Before this the command only carried slot 0 and a game that split
          // on the slot-1 half emitted nothing at all.
          case (sub)
            0: begin mb_wdata<=cur_csl;         xor_acc<=xor_acc^cur_csl; end
            1: begin mb_wdata<=cur_cbk[7:0];    xor_acc<=xor_acc^cur_cbk[7:0]; end
            2: begin mb_wdata<=cur_cbk[15:8];   xor_acc<=xor_acc^cur_cbk[15:8]; end
          endcase
          if (sub==4'd2) begin
            // exit = the SAME decision the pre-v2.3 chain took at the end of
            // S_CHRST (scroll splits -> palette -> nametable)
            if (cspl_i+3'd1 >= l_cspl_cnt) begin
              if (l_split_cnt >= 3'd2) st<=S_SPL_OP;
              else if (l_psp_go) st<=S_PSP_OP;
              else if (pal_present) st<=S_PAL;
              else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
            end else begin cspl_i<=cspl_i+3'd1; sub<=4'd0; st<=S_CSPL_LD; end
          end else sub<=sub+4'd1;
        end

        // -------- CMD_SPLITS (v1.3): 11 hdr(ovf|cnt) cnt x [sl sx sy ntsel] ----
        // Emitted only when cnt>=2 (guaranteed by the S_REGS branch).  Entries
        // walked one at a time: S_SPL_LD consumes l_spl_*[spl_i] into cur_*
        // (the idx mux stays OFF the mb_wdata/xor cone), S_SPL_E emits the 4
        // bytes from cur_* (single-register case mux, like S_HDR/S_REGS).
        S_SPL_OP: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=OP_SPLITS; xor_acc<=xor_acc^OP_SPLITS;
          st<=S_SPL_HDR;
        end
        S_SPL_HDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=l_split_hdr; xor_acc<=xor_acc^l_split_hdr;
          spl_i<=3'd0; sub<=4'd0; st<=S_SPL_LD;
        end
        S_SPL_LD: begin
          cur_sl<=l_spl_sl[spl_i[1:0]]; cur_sx<=l_spl_sx[spl_i[1:0]];
          cur_sy<=l_spl_sy[spl_i[1:0]]; cur_nt<=l_spl_nt[spl_i[1:0]];
          st<=S_SPL_E;
        end
        S_SPL_E: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=cur_sl;          xor_acc<=xor_acc^cur_sl; end
            1: begin mb_wdata<=cur_sx;          xor_acc<=xor_acc^cur_sx; end
            2: begin mb_wdata<=cur_sy;          xor_acc<=xor_acc^cur_sy; end
            3: begin mb_wdata<={6'd0,cur_nt};   xor_acc<=xor_acc^{6'd0,cur_nt}; end
          endcase
          if (sub==4'd3) begin
            if (spl_i+3'd1 >= l_split_cnt) begin
              if (l_psp_go) st<=S_PSP_OP;
              else if (pal_present) st<=S_PAL;
              else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
            end else begin spl_i<=spl_i+3'd1; sub<=4'd0; st<=S_SPL_LD; end
          end else sub<=sub+4'd1;
        end

        // -------- CMD_PPU_SPLITS (Fase 3): 16 hdr(ovf|cnt) cnt x [sl pay] -----
        // Emitted only when l_psp_go, which is `cnt>=2 || entry0 non-classic`,
        // so every frame whose arrangement AND PPUCTRL[4] are classic and
        // static walks EXACTLY the pre-Fase-3 state chain and its bytes are
        // unchanged.  UNLIKE its siblings this command CAN carry cnt==1 (see
        // the OP_PPU_SPLITS comment) -- a walker that assumes cnt>=2 mis-parses
        // the stream from that byte on.
        // Same request/consume split as S_CSPL_*: S_PSP_LD consumes
        // l_psp_*[psp_i] into cur_p* (the 4-deep indexed array read stays OFF
        // the mb_wdata/xor cone -- SERIALIZER TIMING), S_PSP_E emits the 2
        // bytes from those single registers.
        S_PSP_OP: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=OP_PPU_SPLITS; xor_acc<=xor_acc^OP_PPU_SPLITS;
          st<=S_PSP_HDR;
        end
        S_PSP_HDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<=l_psp_hdr; xor_acc<=xor_acc^l_psp_hdr;
          psp_i<=3'd0; sub<=4'd0; st<=S_PSP_LD;
        end
        S_PSP_LD: begin
          // INVARIANT: l_psp_cnt <= 3, so psp_i only ever reaches 2 and the
          // 2-bit index never addresses a 4th element of these 3-deep arrays.
          // The capture cannot produce more (psp_cnt stops incrementing at 3
          // and the overflow branch never increments), and every tb clamps to
          // 3.  If K is ever raised, RAISE THE ARRAYS FIRST -- widening cnt
          // alone would read an element that does not exist.
          cur_psl<=l_psp_sl[psp_i[1:0]]; cur_ppy<=l_psp_py[psp_i[1:0]];
          st<=S_PSP_E;
        end
        S_PSP_E: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=cur_psl; xor_acc<=xor_acc^cur_psl; end
            // bits 7:5 of the payload byte are protocol padding; the
            // capture does not store them (see its header), so they are
            // re-added here
            1: begin mb_wdata<={3'b000,cur_ppy};
                     xor_acc<=xor_acc^{3'b000,cur_ppy}; end
          endcase
          if (sub==4'd1) begin
            // exit = the SAME decision S_SPL_E takes after its last entry
            if (psp_i+3'd1 >= l_psp_cnt) begin
              if (pal_present) st<=S_PAL;
              else begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
            end else begin psp_i<=psp_i+3'd1; sub<=4'd0; st<=S_PSP_LD; end
          end else sub<=sub+4'd1;
        end

        // -------- PALETTE / PALETTE_FULL (pipelined; see pal_cnt_r comment) ---
        // Byte stream identical to the monolithic version (opcode, [count],
        // then values/(idx,val) pairs ascending); only cycle count changed
        // (+1/dirty entry in LIST, 2/byte in FULL) -- the tb compares bytes.
        S_PAL: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          if (use_full) begin
            mb_wdata<=OP_PAL_FULL; xor_acc<=xor_acc^OP_PAL_FULL;
            pal_i<=6'd0; st<=S_PALF_RD;
          end else begin
            mb_wdata<=OP_PALETTE; xor_acc<=xor_acc^OP_PALETTE;
            st<=S_PALL_CNT;
          end
        end
        S_PALL_CNT: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<={2'd0,pal_cnt_frz}; xor_acc<=xor_acc^{2'd0,pal_cnt_frz};
          pal_i<=6'd0; st<=S_PALL_FIND;
        end
        S_PALL_FIND: begin
          if (pal_i>=6'd32) begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
          else if (pal_dirty_frz[pal_i[4:0]]) begin
            mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
            mb_wdata<={3'd0,pal_i[4:0]}; xor_acc<=xor_acc^{3'd0,pal_i[4:0]};
            st<=S_PALL_RD;
          end else pal_i<=pal_i+6'd1;
        end
        S_PALL_RD: st<=S_PALL_WR;   // pal_q <= pal[pal_i] this edge
        S_PALL_WR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<={2'd0,pal_q}; xor_acc<=xor_acc^{2'd0,pal_q};
          pal_i<=pal_i+6'd1; st<=S_PALL_FIND;
        end
        S_PALF_RD: st<=S_PALF_WR;   // pal_q <= pal[pal_i] this edge
        S_PALF_WR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          mb_wdata<={2'd0,pal_q}; xor_acc<=xor_acc^{2'd0,pal_q};
          if (pal_i==6'd31) begin st<=S_NTA; nt_i<=0; nt_inrun<=1'b0; end
          else begin pal_i<=pal_i+6'd1; st<=S_PALF_RD; end
        end

        // -------- NT runs: streaming 2-state scan (S_NTA req, S_NTB consume) --
        S_NTA: begin
          if (nt_i >= NT_SIZE) begin
            if (nt_inrun) begin st<=S_NTHDR; sub<=0; end   // close final run
            else begin st<=S_CBANK; sub<=0; end
          end else st<=S_NTB;   // nt_rd_a==nt_i this cycle -> ntdirty_q next cycle
        end
        S_NTB: begin
          // pending union rewrite: pend accumulates the dirty of EVERY
          // serialized frame until confirmation drops the epoch (see the
          // PENDING note).
          // generation rewrite: SEAL (A := frz|A|B, B := 0) only when this
          // recovery seals (seal_hold); otherwise -- normal frames AND
          // recoveries with A still unconfirmed -- keep the generations
          // separate and accumulate in B.  Delivery (nt_pend_eff) is A|B
          // either way; delivery != rewrite.  The tick-latched pend_avf/bvf
          // materialize any pending invalidation into the stored bits.
          //
          // *** THE REWRITE IS NOT IDEMPOTENT ON A RE-VISIT, AND THE SCAN
          //     RE-VISITS.  (v2.7 fix; this cost the whole "mecanismo 3" class.)
          // A visit that CLOSES a run -- run_len hit the 255 ceiling, or the cell
          // is not deliverable -- leaves nt_i WHERE IT IS on purpose, so after
          // the payload the scan comes back to the SAME offset and evaluates it
          // again.  The old code rewrote ntpend on BOTH visits.  On a SEALING
          // recovery (`seal_now_w = recovery_now_w & ~avf_next_w`, so pend_avf
          // is 0 by construction) the first rewrite moves the cell into
          // generation A and clears B; the second visit then computes
          // nt_pend_eff = (pend_avf & A) | (pend_bvf & B) = (0 & 1) | (x & 0) = 0
          // and the cell is judged NOT PENDING -- so no new run opens on it and
          // it is DROPPED, permanently, from a delta the consumer never saw.
          // Signature: a contiguous pending span emits runs at a stride of 256
          // with length 255, i.e. cells $00FF, $01FF, $02FF, $03FF ... vanish.
          // Measured on megaman1_usa_attract, recovery frames: runs
          // (0,255) (256,255) (512,255) (768,255) -- cells 255/511/767/1023 gone.
          // It never showed up in force_full frames (runs at a stride of 255,
          // contiguous) nor in any byte-exact golden, because a LOCKSTEP consumer
          // never takes the recovery path at all.
          // FIX: rewrite exactly ONCE per cell per frame, on the visit that
          // CONSUMES it (the branches that advance nt_i).
          if (!nt_inrun) begin
            ntpend[nt_i[10:0]] <= seal_hold
              ? {ntdirty_frz_q | (pend_avf & ntpend_q[1]) | (pend_bvf & ntpend_q[0]), 1'b0}
              : {(pend_avf & ntpend_q[1]), (pend_bvf & ntpend_q[0]) | ntdirty_frz_q};
            if (ntdirty_frz_q | force_full | nt_pend_eff) begin
                                 run_start<=nt_i[10:0]; run_len<=9'd1; nt_inrun<=1'b1;
                                 nt_i<=nt_i+12'd1; st<=S_NTA; end
            else begin nt_i<=nt_i+12'd1; st<=S_NTA; end
          end else begin
            if ((ntdirty_frz_q | force_full | nt_pend_eff) && run_len<9'd255) begin
              ntpend[nt_i[10:0]] <= seal_hold
                ? {ntdirty_frz_q | (pend_avf & ntpend_q[1]) | (pend_bvf & ntpend_q[0]), 1'b0}
                : {(pend_avf & ntpend_q[1]), (pend_bvf & ntpend_q[0]) | ntdirty_frz_q};
              run_len<=run_len+9'd1; nt_i<=nt_i+12'd1; st<=S_NTA;
            end else begin
              st<=S_NTHDR; sub<=0;   // close run (nt_i points at the breaking
                                      // offset; it is re-visited, so it must NOT
                                      // have been rewritten above)
            end
          end
        end
        S_NTHDR: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_NT_RUN;            xor_acc<=xor_acc^OP_NT_RUN; end
            1: begin mb_wdata<=run_start[7:0];       xor_acc<=xor_acc^run_start[7:0]; end
            2: begin mb_wdata<={5'd0,run_start[10:8]};xor_acc<=xor_acc^{5'd0,run_start[10:8]}; end
            3: begin mb_wdata<=run_len[7:0];         xor_acc<=xor_acc^run_len[7:0]; end
          endcase
          if (sub==3) begin st<=S_NTDA; run_k<=9'd0; end else sub<=sub+4'd1;
        end
        S_NTDA: st<=S_NTDB;   // nt_rd_a==run_start+run_k -> ciram_q next cycle
        S_NTDB: begin
          mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=ciram_q; wptr<=wptr+13'd1;
          xor_acc<=xor_acc^ciram_q;
          if (run_k+9'd1==run_len) begin nt_inrun<=1'b0; st<=S_NTA; end  // resume scan at nt_i
          else begin run_k<=run_k+9'd1; st<=S_NTDA; end
        end

        // -------- CHR_BANK (0..2): 40 slot bank --------
        // -------- CHR_BANK (0..2): 40 slot bank --------
        // The CHR section that follows is: [pend re-send runs] [fresh runs].
        // The pend scan goes FIRST so that, for a tile touched by both, the
        // FRESH run -- the write-order, byte-exact one -- is what lands last at
        // the renderer's last-wins shadow.
        S_CBANK: begin
          case (sub)
            0: if (l_s0chg) begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=OP_CHR_BANK;
                     xor_acc<=xor_acc^OP_CHR_BANK; wptr<=wptr+13'd1; sub<=4'd1; end
               else sub<=4'd3;
            1: begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=8'd0; xor_acc<=xor_acc^8'd0;
                     wptr<=wptr+13'd1; sub<=4'd2; end
            2: begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=l_s0b; xor_acc<=xor_acc^l_s0b;
                     wptr<=wptr+13'd1; sub<=4'd3; end
            3: if (l_s1chg) begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=OP_CHR_BANK;
                     xor_acc<=xor_acc^OP_CHR_BANK; wptr<=wptr+13'd1; sub<=4'd4; end
               else st<= chr_scan_go ? S_CP0 : S_CR0;
            4: begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=8'd1; xor_acc<=xor_acc^8'd1;
                     wptr<=wptr+13'd1; sub<=4'd5; end
            5: begin mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=l_s1b; xor_acc<=xor_acc^l_s1b;
                     wptr<=wptr+13'd1; st<= chr_scan_go ? S_CP0 : S_CR0; end
          endcase
        end
        // -------- CMD_CHR_RUN, PENDING RE-SEND (v2.7): 41 off_lo off_hi len --
        // A TILE-GRANULAR scan of chrpend in ADDRESS order, payload read from
        // the mirror.  This is the CHR analogue of the NT pending union: same
        // two generations, same materialise-by-rewrite, same "delivery is not
        // the same thing as rewrite" split.  What it does NOT share is a
        // capacity limit -- the bytes live in chrmem, so a tile that does not
        // fit this frame's allowance simply keeps its bit and is DEFERRED
        // to the next recovery instead of being discarded.  That is the whole
        // difference from the ring the design replaces.
        //
        // ONE REWRITE PER CELL PER FRAME, ON THE VISIT THAT CONSUMES IT -- the
        // v2.7 NT lesson, and it bites here for the same reason: the visit that
        // CLOSES a run leaves cp_i where it is, so the scan comes back to the
        // same tile.  Rewriting on both visits would move the cell into A on
        // the first and then read it as not-pending on the second, dropping it
        // from a delta the consumer never saw.
        S_CP0: begin
          if (cp_i >= CHR_TILES) begin
            if (cp_inrun) begin st<=S_CPH; sub<=0; end   // close the final run
            else begin
              // chr_pend_nz is the scan's own gate for the NEXT frame: if this
              // pass left every cell clear there is nothing to walk again.
              chr_pend_nz <= cp_any;
              // The pass visited EVERY cell, so it recomputes the debt exactly.
              chr_debt    <= cp_dany;
              // The pass visited every tile, so the only thing to carry over is
              // WHERE THE BUDGET RAN OUT.  A pass that could not DELIVER (a
              // plain materialise) leaves the cursor alone -- but EVERY
              // delivering frame must advance it, drain frames included: gating
              // this on lost_hold alone made each drain restart at the same
              // cp_base and re-offer the same tiles (head-of-line starvation).
              if (cp_deliver) cp_base <= cp_nresume;
              st <= S_CR0;
            end
          end else st<=S_CP1;   // chrpend_ra==cp_i now -> chrpend_q next cycle
        end
        S_CP1: begin
          if (!cp_inrun) begin
            // REWRITE.  Not sealing: just materialise the tick-latched validity
            // (and the force-mark).  Sealing: a DELIVERED tile moves into A,
            // and one the budget did not carry stays owed -- sealing it would
            // let the confirmation of this recovery drop a tile that was never
            // sent, which is exactly the loss this rewrite exists to prevent.
            // ⚠️ TWO CORRECTIONS to what this comment used to say (v2.8):
            //   * the undelivered tile now goes to the DEBT generation (D), not
            //     to B -- chr_debt is what carries it across frames;
            //   * "Mutation: seal unconditionally -> run_chrburst FAILs" is NO
            //     LONGER TRUE.  That mutant is INERT against run_chrburst,
            //     because the megaman1 cells there never reach caught_w at the
            //     instant of the skip (the tb_skipburst consumer writes its ACK
            //     at a fixed phase from a last_seq updated only after the whole
            //     apply).  The gate that DOES discriminate this policy is the
            //     synthetic lock-step one -- do not trust the old claim.
            chrpend[cp_a] <= {cp_dnext, cp_anext, cp_bnext};
            if (cp_dnext | cp_anext | cp_bnext) cp_any  <= 1'b1;
            if (cp_dnext)                       cp_dany <= 1'b1;
            if (cp_send) begin
              cp_start      <= cp_a;
              cp_len        <= 5'd1;
              cp_inrun      <= 1'b1;
              cp_nresume    <= cp_a + 9'd1;
              chr_pend_emit <= chr_pend_emit + 13'd16;
            end
            cp_i<=cp_i+10'd1; st<=S_CP0;
          end else begin
            // ... and NEVER across the wrap: a run is a contiguous OFFSET
            // range, so tile 0 always opens a new one (the same rule the tap
            // applies to chr_off == 0).
            if (cp_send && (cp_len < CHR_PEND_MAXT) && (cp_a != 9'd0)) begin
              chrpend[cp_a] <= {cp_dnext, cp_anext, cp_bnext};
              if (cp_dnext | cp_anext | cp_bnext) cp_any  <= 1'b1;
              if (cp_dnext)                       cp_dany <= 1'b1;
              cp_len        <= cp_len + 5'd1;
              cp_nresume    <= cp_a + 9'd1;
              chr_pend_emit <= chr_pend_emit + 13'd16;
              cp_i<=cp_i+10'd1; st<=S_CP0;
            end else begin
              st<=S_CPH; sub<=0;   // close run (cp_i points at the breaking
                                   // tile; it is re-visited, so it must NOT
                                   // have been rewritten above)
            end
          end
        end
        S_CPH: begin
          // off = tile*16 and len = tiles*16, both by pure bit-slicing: that is
          // why the run is capped at 15 tiles (CHR_PEND_MAXT) -- 240 is the
          // largest whole-tile length the one-byte len field holds.
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_CHR_RUN;            xor_acc<=xor_acc^OP_CHR_RUN; end
            1: begin mb_wdata<={cp_start[3:0],4'd0};  xor_acc<=xor_acc^{cp_start[3:0],4'd0}; end
            2: begin mb_wdata<={3'd0,cp_start[8:4]};  xor_acc<=xor_acc^{3'd0,cp_start[8:4]}; end
            3: begin mb_wdata<={cp_len[3:0],4'd0};    xor_acc<=xor_acc^{cp_len[3:0],4'd0}; end
          endcase
          if (sub==3) begin
            st<=S_CPA; cp_bk<=8'd0; chr_ra_r<={cp_start,4'd0};
          end else sub<=sub+4'd1;
        end
        S_CPA: st<=S_CPB;            // chrmem_ra==chr_ra_r -> chrmem_q next cycle
        S_CPB: begin
          mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=chrmem_q; wptr<=wptr+13'd1;
          xor_acc<=xor_acc^chrmem_q;
          chr_ra_r <= chr_ra_r + 13'd1;
          chr_emit <= chr_emit + 13'd1;
          // TERMINATION IS NOT ALLOWED TO DEPEND ON AN INVARIANT (house rule
          // #1: never wedge).  cp_len is 1..15 by construction, so the equality
          // always fires at 239 at the latest; the second exit makes that a
          // hard ceiling instead of an assumption.
          if ((cp_bk + 8'd1 == {cp_len[3:0],4'd0}) || (cp_bk == 8'd239)) begin
            cp_inrun<=1'b0; st<=S_CP0;
          end else begin cp_bk<=cp_bk+8'd1; st<=S_CPA; end
        end

        // -------- CMD_CHR_RUN, FRESH (v2.4 shape): 41 off_lo off_hi len data --
        // Walk the frozen descriptor slice [dsc_i, l_dsc_end) in ARRIVAL order
        // (NOT sorted/deduplicated as the old dirty-bitmap scan was: two writes
        // to the same byte in one frame ship twice and the renderer's last-wins
        // apply converges -- 99.6-99.9% of real writes are sequential, so the
        // cost is noise).  Emitted for CHR-RAM only: with CHR-ROM the tap never
        // fires, l_dsc_end==dsc_i, and this whole block is a single cycle that
        // adds ZERO bytes (byte-identity of every CHR-ROM golden).
        // The walk stops on the MAILBOX valve only (CHR_WSTOP: a buffer is
        // 8192 B and wptr wraps silently).  There is deliberately NO per-frame
        // byte cap on the drain: capping it drops data with a perfectly healthy
        // consumer (measured: 6585 drops at 2500 B/frame, 49955 at 3700, where
        // the uncapped drain dropped ZERO).  Whatever CHR_WSTOP holds back
        // keeps its place -- dsc_i stays where it is and the next frame's slice
        // starts there -- so it is DEFERRED, never lost.
        S_CR0: begin
          if ((dsc_i == l_dsc_end) || (wptr >= CHR_WSTOP)) begin
            // The valve cut the FRESH slice short: record it so the NEXT frame
            // spends its whole ceiling on the backlog instead of on a resend.
            if (dsc_i != l_dsc_end) chr_defer_r <= 1'b1;
            st<=S_OAMA; oam_i<=0;
          end
          else st<=S_CR1;          // chrdsc_ra==dsc_i now -> chrdsc_q next cycle
        end
        S_CR1: begin
          cur_coff <= chrdsc_q[25:13];
          cur_ccnt <= chrdsc_q[12:0];
          dsc_i    <= dsc_i + 8'd1;   // look ahead at the NEXT descriptor
          st       <= S_CR2;
        end
        S_CR2: st<=S_CR3;            // chrdsc_ra==dsc_i(+1) -> chrdsc_q next cycle
        S_CR3: begin
          // LENGTH IS DERIVED, never stored: the next run starts where this one
          // ends, and the LAST run of the frame ends at the frozen byte count.
          // The 13-bit subtraction wraps with the counter, so a run straddling
          // the wrap measures correctly.
          cur_clen <= ((dsc_i == l_dsc_end) ? l_cb_end : chrdsc_q[12:0]) - cur_ccnt;
          chr_ra_r <= cur_coff;      // park the mirror cursor at the run start
          sub      <= 4'd0;
          st       <= S_CRH;
        end
        S_CRH: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          case (sub)
            0: begin mb_wdata<=OP_CHR_RUN;             xor_acc<=xor_acc^OP_CHR_RUN; end
            1: begin mb_wdata<=cur_coff[7:0];          xor_acc<=xor_acc^cur_coff[7:0]; end
            2: begin mb_wdata<={3'd0,cur_coff[12:8]};  xor_acc<=xor_acc^{3'd0,cur_coff[12:8]}; end
            3: begin mb_wdata<=cur_clen[7:0];          xor_acc<=xor_acc^cur_clen[7:0]; end
          endcase
          if (sub==3) begin st<=S_CRA; crun_k<=8'd0; end else sub<=sub+4'd1;
        end
        S_CRA: st<=S_CRB;            // chrmem_ra==chr_ra_r -> chrmem_q next cycle
        S_CRB: begin
          mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=chrmem_q; wptr<=wptr+13'd1;
          xor_acc<=xor_acc^chrmem_q;
          chr_ra_r <= chr_ra_r + 13'd1;
          chr_emit <= chr_emit + 13'd1;
          // MARK THE TILE AS OWED.  Generation B (young) is set for the tile
          // this byte belongs to; A is cleared ON PURPOSE -- a fresh write
          // supersedes any older delivery of the same tile, and B is the
          // conservative home (it only dies on a full catch-up, while A dies on
          // the confirmation of a recovery that no longer describes this data).
          // This and S_CP1 are the ONLY writers of chrpend, and they are in
          // different states, so the array keeps a single writer per cycle.
          // {D,A,B} = 001: D and A cleared because THIS run carries the tile
          // and supersedes any older delivery of it; B is the conservative home.
          chrpend[chr_ra_r[12:4]] <= 3'b001;
          chr_pend_nz <= 1'b1;
          // TERMINATION IS NOT ALLOWED TO DEPEND ON AN INVARIANT (house rule
          // #1: never wedge).  cur_clen is 13 bits and crun_k is 8; the tap
          // guarantees 1 <= cur_clen <= 255, but if that ever broke (a
          // descriptor corrupted by a ring bug, cur_clen[12:8]!=0) the equality
          // alone would NEVER fire -- crun_k wraps at 256 and the FSM would
          // emit payload for ever, overrunning the mailbox and hanging the
          // frame.  crun_k==254 is the LAST legal value (len<=255 => last byte
          // is index 254), so this second exit is unreachable for every legal
          // run and is a hard ceiling for an illegal one.
          if (({5'd0,crun_k} + 13'd1 == cur_clen) || (crun_k == 8'd254)) st<=S_CR0;
          else begin crun_k<=crun_k+8'd1; st<=S_CRA; end
        end

        // -------- OAM: 50 + 256 bytes --------
        S_OAMA: begin
          if (oam_i==9'd0) begin
            mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=OP_OAM; xor_acc<=xor_acc^OP_OAM;
            wptr<=wptr+13'd1; oam_i<=9'd1; st<=S_OAMB;   // oam_rd_a==0 -> oam_q next cyc
          end
        end
        S_OAMB: begin
          mb_we<=1'b1; mb_waddr<=wptr; mb_wdata<=oam_frz_q; wptr<=wptr+13'd1;
          xor_acc<=xor_acc^oam_frz_q;
          if (oam_i==9'd256) begin st<=S_DONE; sub<=0; end
          else oam_i<=oam_i+9'd1;   // oam_rd_a follows oam_i
        end

        // -------- FRAME_DONE: F0 xor(body) --------
        S_DONE: begin
          mb_we<=1'b1; mb_waddr<=wptr; wptr<=wptr+13'd1;
          if (sub==0) begin mb_wdata<=OP_FRAME_DONE; sub<=4'd1; end
          else begin mb_wdata<=xor_acc; st<=S_CLEAR; clr_nt<=0; end
        end

        // -------- clear the NT dirty bitmap (the CHR one is gone in v2.4) -----
        S_CLEAR: begin
          // clears target the FROZEN banks (the nt0/1_w* muxes route them);
          // taps keep writing the live banks in parallel, race-free
          if (clr_nt < NT_SIZE)   begin ntclr_we<=1'b1;  ntclr_a<=clr_nt[10:0];   clr_nt<=clr_nt+12'd1; end
          else st<=S_FINISH;
        end

        // -------- publish + reset per-frame --------
        S_FINISH: begin
          frame_len_o   <= {3'd0, wptr};
          frame_seq_o   <= new_seq;
          bc_bytes_last <= {3'd0, wptr};
          bc_frames     <= bc_frames + 16'd1;
          if ((frame_seq_o - frame_ack_i) > 16'd1) begin
            bc_overruns <= bc_overruns + 16'd1; status_o[0] <= 1'b1;
          end
          // v2.4: sticky "a CHR-RAM byte was dropped because the capture ring
          // was full".  Unreachable in the corpus; a device seeing this bit set
          // is telling you the ring sizing assumption broke, not that the
          // renderer is behind (that is bit 0).
          if (cb_ovf) status_o[1] <= 1'b1;
          if (l_s0p) begin s0_prev<=l_s0b; s0_valid<=1'b1; end else s0_valid<=1'b0;
          if (l_s1p) begin s1_prev<=l_s1b; s1_valid<=1'b1; end else s1_valid<=1'b0;
          if (live) begin nt_cnt0<=0; chr_any0<=0; pal_dirty0<=0; pal_cnt0<=0; end
          else      begin nt_cnt1<=0; chr_any1<=0; pal_dirty1<=0; pal_cnt1<=0; end
          pend_valid <= 1'b1;   // pending arrays fully rewritten by this scan
          if (lost_hold & seal_hold) begin
            // this recovery SEALED generation A: everything it delivered now
            // lives in A -- arm the confirmation watch (ack >= recov_seq).
            // Non-sealing recoveries keep recov_seq at the FIRST unconfirmed
            // recovery (see the tick-wire note).
            pend_a_valid <= 1'b1;
            recov_active <= 1'b1;
            recov_seq    <= new_seq;
          end
          frame_done_o<=1'b1; st<=S_IDLE;
        end

        default: st<=S_IDLE;
      endcase
    end
  end

  // ============================================================ window read
  // One registered read PER mailbox buffer + combinational mux AFTER the
  // registers.  Quartus M9K inference REQUIRES the RAM read itself to be
  // synchronous: the previous `win_data <= sel ? mbox1[a] : mbox0[a]` (mux of
  // two async array reads in front of one register) uninfers BOTH arrays
  // (quartus_map Info 276007) and the 2x8KiB fall back to ~131k FFs (Error
  // 276003).  Each block below is a clean simple-dual-port template (sync
  // write in the FSM block, sync read here).  Total window latency is
  // UNCHANGED: win_data valid 1 cycle after win_addr (buf_sel_i is stable for
  // the whole drain, so the post-register mux adds no cycle).  The two buffers
  // never read the address being written (renderer reads the completed buffer;
  // the FSM writes the other one) => read-during-write don't-care.
  reg [7:0] mbox0_q, mbox1_q;
  always @(posedge clk) begin
    mbox0_q <= mbox0[win_addr];
    mbox1_q <= mbox1[win_addr];
  end
  assign win_data = buf_sel_i ? mbox1_q : mbox0_q;

  // ============================================================ joypad
  // joy_strobe/joy_clock are LEVELS (in hardware: nes.v's ce-registered
  // tapJ_strobe/tapJ_clock; a $4016/$4017 read holds joypad_clock high for a
  // whole CPU cycle = 3 ce ticks = dozens of CLK2 cycles).  The controller
  // model shifts on the FALLING edge of the clock level -- one shift per read
  // -- exactly like the canonical fpganes top-level consumer (NES_Nexys4:
  // `if (!joypad_clock[0] && last_joypad_clock[0]) shift`).  Level-shifting
  // (the first version of this block) shifted every CLK2 cycle the level was
  // high = dozens of shifts per read: latent functional bug, never exercised
  // by the byte-exact gate (the tb ties joypad off).  The core samples
  // joypad_data DURING the high level (pre-shift bit); the falling edge then
  // advances to the next bit -- correct NES serial semantics.
  // BIT ORDER (cost a hardware iteration -- "erratic controls"): the shift is
  // MSB-FIRST.  The renderer packs A=bit7, B=bit6, Select=bit5, Start=bit4,
  // Up=bit3, Down=bit2, Left=bit1, Right=bit0 ("classic serial order",
  // nes_transport_device.a65) and the NES reads A on the FIRST $4016 read --
  // so bit7 must come out first.  The original LSB-first shift returned the
  // byte TRANSPOSED (Right read as A, Left as B, Up as Start...): the game
  // responded, but to the wrong buttons.  Post-shift fill = 1s, so reads 9+
  // return 1 (real NES controller convention; games may depend on it).
  // Validated end-to-end by tb/run_joypad.sh (real 6502 micro-ROM doing
  // strobe + 10 reads against the full core + bridge).
  // Byte-tearing note: all 8 buttons live in CTRL_P1's LOW byte, written
  // atomically by ONE 8-bit bus write ($2BDA); the high byte is zero padding
  // -- cross-frame button tearing is impossible by construction.
  reg [7:0] sr1, sr2;
  reg [1:0] joy_clock_prev;
  always @(posedge clk) begin
    if (rst) begin sr1<=8'hFF; sr2<=8'hFF; joy_clock_prev<=2'b00; end
    // THE WHOLE BLOCK IS PACED BY ce_tick, not just the reload.  Gating only the
    // reload left the SHIFT unpaced: joy_clock_prev updated every CLK2, so the
    // falling-edge detect went true one CLK2 AFTER the edge and sr* moved at
    // launch+1 -- while the SDC's rule 4c hands that path launch+2 (23.8 ns for
    // an 11.9 ns relationship).  The -0.318 ns violation on
    // bridge|sr2[7] -> NES_ROM_DATAr[5] stayed uncovered and the fit's PASS on
    // that edge was empty.  With the block paced, BOTH edges on which sr* can
    // change are ce edges, so 4c is exactly 2 and 4b is >= 13.
    // Semantics are untouched: joy_clock is tapJ_clock, itself ce-registered,
    // so comparing it against its value at the PREVIOUS ce still yields exactly
    // one shift per falling edge; and the core can only observe joypad_data at
    // a ce, with >= 9 ce between the clock falling and the next $4016 read.
    // run_joypad (a real 6502 micro-ROM doing strobe + 10 reads) is the proof.
    else if (ce_tick) begin
      joy_clock_prev <= joy_clock;
      // TWO measured violations sit behind the pacing above.  (1) The RELOAD:
      // ctrl_p1_i/ctrl_p2_i are the control-block registers the SNES writes on
      // SNES_WR_end -- an edge with NO relation to the core's `ce`.  Reloading
      // on every CLK2 while the strobe level is high made sr1/sr2 change on
      // that arbitrary edge, so sr* -> NES core was genuinely SINGLE-CYCLE and
      // the SDC's rule 4b (`-setup 2`, justified by "the sr* only move one CLK2
      // after a ce") claimed a relationship the RTL did not provide: -0.058 ns,
      // hidden by the constraint.  (2) The SHIFT: with joy_clock_prev updating
      // every CLK2 the edge-detect fired one cycle AFTER the falling edge, so
      // sr* moved at launch+1 while rule 4c gives that path launch+2 --
      // -0.318 ns on sr2[7] -> NES_ROM_DATAr[5], left uncovered.
      // With the pacing the sr* only move on a core tick and rule 4b is true.
      // Semantics are preserved: the OUT0 strobe still reloads CONTINUOUSLY
      // while high (just at ce granularity), still suppresses shifting while
      // high, and the core can only observe the register at a ce anyway -- so
      // it sees the same byte it saw before.
      if (joy_strobe) begin sr1<=ctrl_p1_i[7:0]; sr2<=ctrl_p2_i[7:0]; end
      else begin
        if (!joy_clock[0] && joy_clock_prev[0]) sr1<={sr1[6:0],1'b1};
        if (!joy_clock[1] && joy_clock_prev[1]) sr2<={sr2[6:0],1'b1};
      end
    end
  end
  assign joypad_data_o = {sr2[7], sr1[7]};

endmodule
