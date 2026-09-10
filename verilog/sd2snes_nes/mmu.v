// Copyright (c) 2012-2013 Ludvig Strigeus
// This program is GPL Licensed. See COPYING for the full license.
//
// ---------------------------------------------------------------------------
// PRUNED for sd2snes_nes (started at Phase -1, nestest gate only; MMC3 added
// back in Phase 2.1; the discrete-mapper batch -- 11/66/79/113/87/34/71/232/69
// -- added after that).
//
// Vendored from strigeus/fpganes `src/mmu.v` (1757 lines). This file originally
// contained the `MultiMapper` dispatcher plus EVERY supported mapper (MMC0..MMC5,
// Rambo1, Mapper13/15/28/34/41/66/68/69/71/79/228/234, NesEvent). Per
// NES-FPGANES-ANALYSIS.md #5 and NES-CORE-CONTRACT.md #9, Phase -1 only needed to
// exercise CPU+DMA+DMC+mapper 0 (nestest.nes is an NROM/mapper-0 image), so this
// file keeps ONLY:
//   - MMC0     (mmu.v:5-21 upstream)   -- trivial NROM fallback, kept as `default`
//   - MMC1     (mmu.v:25-133 upstream) -- mapper 1 (not exercised by nestest, but
//                                          cheap and explicitly requested to stay)
//   - Mapper28 (mmu.v:946-1036 upstream) -- mappers 0, 2, 3, 7, 28 (nestest.nes is
//                                          mapper 0, dispatched here)
//   - MMC3     (mmu.v:251-395 upstream) -- mapper 4 (Phase 2.1; the 47/118/119
//                                          variants that share the upstream module
//                                          are pruned -- see that module's header)
//                                          plus, as MODES of the same module, the
//                                          NAMCO 108 FAMILY (206, 88, 95, 154)
//                                          the TAITO TC0190/TC0690 pair (33,
//                                          48), the MMC6 ($F4) and the TAITO
//                                          X1-005/X1-017 pair (80, 82) -- all
//                                          four reuse its register file.
//   - MapperDiscrete -- NOT an upstream module: ONE module covering mappers 11,
//                       66, 79, 113, 87, 34 and 71/232, i.e. what upstream spread
//                       over Mapper66 (mmu.v:1040), Mapper34 (1073), Mapper79
//                       (1358) and Mapper71 (1312), plus mapper 87 (Jaleco JF-0x,
//                       which upstream never had), and then the second discrete
//                       batch -- 93, 89, 94, 97, 180, 184, 70, 152, 78 (+ the $F8
//                       Cosmo Carrier alias), 86 and 140, none of which upstream
//                       ever had either, and finally 72 (Jaleco JF-17) and 185
//                       (CNROM with CHR disable). See that module's header for
//                       why they are fused and for the per-mapper deviations.
//   - Mapper1K -- NOT an upstream module: the eight-1KB-window family, i.e.
//                       iNES 18, 19, 32 (+ the $F1 Major League alias), 65, 75
//                       and 210, written from the nesdev wiki; see its header
//                       for why six mappers share one module.
//   - Mapper69 (mmu.v:1221-1310 upstream) -- mapper 69 / Sunsoft FME-7, ported
//                       verbatim except for the PRG-RAM base address and the
//                       `prgout` default; see that module's header.
//   - VRC24 -- NOT an upstream module: Konami VRC2/VRC4, i.e. iNES mappers 21,
//                       22, 23 and 25, written from the nesdev wiki.  One
//                       module, four register-line wirings; see its header.
//   - MultiMapper, reduced to a 7-way one-hot select (1 / 4 / 69 / VRC / the
//     discrete batch / {0,2,3,7,28} / default) plus the mapper-agnostic
//     prg_mask/chr_mask + CHR-VRAM/CPU-RAM remap tail, which is unchanged from
//     upstream (mmu.v:1670-1749).
//
// REMOVED entirely (modules + their instantiation + case arms in MultiMapper):
//   MMC2 (mapper 9), MMC5 (5, incl. its 1KB ExRAM + mul8x8 +
//   vsplit + scanline IRQ), Rambo1 (64/158), Mapper13 (13), Mapper15 (15),
//   Mapper41 (41), Mapper68 (68), Mapper228 (228), Mapper234 (234),
//   NesEvent (105, which upstream reaches INTO MMC1's internals -- mmc1_chr/
//   mmc1_aout -- so it could not have been pruned independently of MMC1 anyway).
//
// Consequences of the prune:
//   - `irq` is driven ONLY by MMC3 (mapper 4 -- the Namco 108 family shares that
//     module but "has no IRQs" and cannot reach irq_enable), Mapper69 and VRC24
//     (mapper 22 shares that module but is a VRC2, which has no IRQ device and
//     cannot reach the $F00x block). For every other mapper here the upstream
//     `irq=0` default in the combinational block is never overridden, exactly as
//     before.  Mapper1K is the fourth source (mappers 18, 19 and 65 each have a
//     CPU-cycle counter; 32, 75 and 210 have no IRQ device and contribute a
//     constant 0), and the MMC3 arm gained the Taito TC0690 (48) on its
//     existing A12 counter.
//   - `has_chr_dout`/`chr_dout`/`prg_dout` (used upstream only by MMC5's ExRAM
//     readback) are always the inert defaults (0 / 0 / 8'hff).  That is a choice,
//     not a limitation: nes.v DOES honour both overrides (`chr_to_ppu =
//     has_chr_from_ppu_mapper ? chr_from_ppu_mapper : memory_din_ppu`, and
//     `from_data_bus = prg_dout_mapper` whenever prg_allow is low), so a future
//     mapper that has to answer reads itself -- 185, or MMC6's "disabled half
//     reads back as zero" -- can use them without touching nes.v.
//   - `ppu_ce` (2nd clock-enable input, used upstream by MMC2/MMC3/MMC5/Rambo1 to
//     react every PPU dot instead of only on cart_ce) is consumed by MMC3 since
//     Phase 2.1 and by MapperDiscrete since the mapper-185 batch; nes.v has always
//     fed it the core `ce` (nes.v: `MultiMapper multi_mapper(clk, cart_ce, ce,
//     ...)`), so nothing outside this file changed.  EVERY sequential element in
//     this file is gated by one of `ce` / `ppu_ce` -- `main.sdc` states that as
//     the foundation of its blanket `set_multicycle_path -setup 4` on
//     `*|NES:core|*`, so a free-running register here would be a constraint lie.
//
// FUNCTIONAL DEVIATIONS from upstream (besides the prune):
//   - Mapper28 maps 8KB of PRG-RAM at $6000-$7FFF (upstream had none there) --
//     see the comment block at the end of module Mapper28 for the full rationale
//     (blargg test harness / Family-Basic-style NROM / standard emulator behavior).
//   - MMC3 forces the PRG-RAM enable bit on (`ram_enable_eff`) for the SAME
//     reason -- see the DEVIATION block in module MMC3.
//   - MapperDiscrete replaces upstream Mapper71's unconditional "header-H means
//     single-screen" rule with a latch armed by the first $9000-$9FFF write --
//     see the FIRE HAWK block in that module.
//   - Mapper69 puts its PRG-RAM in the CART-RAM window (0x3C0000) instead of
//     upstream's 0x100000, which is not part of this core's PSRAM map.
// ---------------------------------------------------------------------------

// No mapper chip
module MMC0(input clk, input ce,
            input [31:0] flags,
            input [15:0] prg_ain, output [21:0] prg_aout,
            input prg_read, prg_write,                   // Read / write signals
            input [7:0] prg_din,
            output prg_allow,                            // Enable access to memory for the specified operation.
            input [13:0] chr_ain, output [21:0] chr_aout,
            output chr_allow,                      // Allow write
            output vram_a10,                             // Value for A10 address line
            output vram_ce);                             // True if the address should be routed to the internal 2kB VRAM.
  assign prg_aout = {7'b00_0000_0, prg_ain[14:0]};
  assign prg_allow = prg_ain[15] && !prg_write;
  assign chr_allow = flags[15];
  assign chr_aout = {9'b10_0000_000, chr_ain[12:0]};
  assign vram_ce = chr_ain[13];
  assign vram_a10 = flags[14] ? chr_ain[10] : chr_ain[11];
endmodule

// MMC1 mapper chip. Maps prg or chr addresses into a linear address.
// If vram_ce is set, {vram_a10, chr_aout[9:0]} are used to access the NES internal VRAM instead.
module MMC1(input clk, input ce, input reset,
            input [31:0] flags,
            input [15:0] prg_ain, output [21:0] prg_aout,
            input prg_read, prg_write,                   // Read / write signals
            input [7:0] prg_din,
            output prg_allow,                            // Enable access to memory for the specified operation.
            input [13:0] chr_ain, output [21:0] chr_aout,
            output chr_allow,                      // Allow write
            output vram_a10,                             // Value for A10 address line
            output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
            // sd2snes video-bridge CHR-bank tap (raw register state; MultiMapper
            // derives the per-slot snapshot -- see chr_snap_* there)
            output [4:0] chr_snap_bank0,
            output [4:0] chr_snap_bank1,
            output chr_snap_4k,
            // sd2snes video-bridge NT-arrangement tap (v2.0a): raw MMC1 mirror
            // control control[1:0] (00=1-screen lower, 01=1-screen upper,
            // 10=vertical, 11=horizontal -- same field vram_a10_t decodes).
            // MultiMapper maps it to the protocol NTARR code (see ntarr_of).
            output [1:0] snap_mirror);
  reg [4:0] shift;

// CPPMM
// |||||
// |||++- Mirroring (0: one-screen, lower bank; 1: one-screen, upper bank;
// |||               2: vertical; 3: horizontal)
// |++--- PRG ROM bank mode (0, 1: switch 32 KB at $8000, ignoring low bit of bank number;
// |                         2: fix first bank at $8000 and switch 16 KB bank at $C000;
// |                         3: fix last bank at $C000 and switch 16 KB bank at $8000)
// +----- CHR ROM bank mode (0: switch 8 KB at a time; 1: switch two separate 4 KB banks)
  reg [4:0] control;

// CCCCC
// |||||
// +++++- Select 4 KB or 8 KB CHR bank at PPU $0000 (low bit ignored in 8 KB mode)
  reg [4:0] chr_bank_0;

// CCCCC
// |||||
// +++++- Select 4 KB CHR bank at PPU $1000 (ignored in 8 KB mode)
  reg [4:0] chr_bank_1;

// RPPPP
// |||||
// |++++- Select 16 KB PRG ROM bank (low bit ignored in 32 KB mode)
// +----- PRG RAM chip enable (0: enabled; 1: disabled; ignored on MMC1A)
  reg [4:0] prg_bank;

  // Update shift register
  always @(posedge clk) if (reset) begin
    shift <= 1;
    control <= 'hC;
    // sd2snes_nes: chr_bank_0/1 had NO reset upstream (X in simulation until
    // first written; 0 in hardware).  Reset added for sim determinism -- the
    // bridge CHR-bank tap reads them every frame; same precedent as Mapper28's
    // a53chr.  Hardware behavior unchanged (registers power up 0), and the
    // golden simulator's power-on state is bank 0 too (mappers.py).
    chr_bank_0 <= 0;
    chr_bank_1 <= 0;
  end else if (ce) begin
    if (prg_write && prg_ain[15]) begin
      if (prg_din[7]) begin
        shift <= 5'b10000;
        control <= control | 'hC;
      end else begin
        if (shift[0]) begin
          casez(prg_ain[14:13])
          0: control    <= {prg_din[0], shift[4:1]};
          1: chr_bank_0 <= {prg_din[0], shift[4:1]};
          2: chr_bank_1 <= {prg_din[0], shift[4:1]};
          3: prg_bank   <= {prg_din[0], shift[4:1]};
          endcase
          shift <= 5'b10000;
        end else begin
          shift <= {prg_din[0], shift[4:1]};
        end
      end
    end
  end

  // The PRG bank to load. Each increment here is 16kb. So valid values are 0..15.
  reg [3:0] prgsel;
  always @* begin
    casez({control[3:2], prg_ain[14]})
    3'b0?_?: prgsel = {prg_bank[3:1], prg_ain[14]};
    3'b10_0: prgsel = 4'b0000;
    3'b10_1: prgsel = prg_bank[3:0];
    3'b11_0: prgsel = prg_bank[3:0];
    3'b11_1: prgsel = 4'b1111;
    endcase
  end
  wire [21:0] prg_aout_tmp = {4'b00_00,  prgsel, prg_ain[13:0]};

  // The CHR bank to load. Each increment here is 4 kb. So valid values are 0..31.
  reg [4:0] chrsel;
  always @* begin
    casez({control[4], chr_ain[12]})
    2'b0_?: chrsel = {chr_bank_0[4:1], chr_ain[12]};
    2'b1_0: chrsel = chr_bank_0;
    2'b1_1: chrsel = chr_bank_1;
    endcase
  end
  assign chr_aout = {5'b100_00, chrsel, chr_ain[11:0]};

  // The a10 VRAM address line. (Used for mirroring)
  reg vram_a10_t;
  always @* begin
    casez(control[1:0])
    2'b00: vram_a10_t = 0;             // One screen, lower bank
    2'b01: vram_a10_t = 1;             // One screen, upper bank
    2'b10: vram_a10_t = chr_ain[10];   // One screen, vertical
    2'b11: vram_a10_t = chr_ain[11];   // One screen, horizontal
    endcase
  end
  assign vram_a10 = vram_a10_t;
  assign vram_ce = chr_ain[13];

  wire prg_is_ram = prg_ain >= 'h6000 && prg_ain < 'h8000;
  assign prg_allow = prg_ain[15] && !prg_write || prg_is_ram;
  wire [21:0] prg_ram = {9'b11_1100_000, prg_ain[12:0]};

  assign prg_aout = prg_is_ram ? prg_ram : prg_aout_tmp;
  assign chr_allow = flags[15];

  // sd2snes bridge tap: raw CHR-bank register state (register outputs only)
  assign chr_snap_bank0 = chr_bank_0;
  assign chr_snap_bank1 = chr_bank_1;
  assign chr_snap_4k    = control[4];
  // sd2snes bridge tap (v2.0a): raw MMC1 mirror control (dynamic; power-on 0x0C
  // -> control[1:0]=00 = 1-screen lower, matching the golden mappers.py MMC1
  // reset state and the vram_a10_t decode above).
  assign snap_mirror    = control[1:0];
endmodule

// Mapper28 -- covers mappers 0 (NROM), 2 (UNROM), 3 (CNROM), 7 (AxROM) and 28
// (homebrew "mapper 28" multi-discrete). nestest.nes is mapper 0, dispatched here.
module Mapper28(input clk, input ce, input reset,
                input [31:0] flags,
                input [15:0] prg_ain, output [21:0] prg_aout,
                input prg_read, prg_write,                   // Read / write signals
                input [7:0] prg_din,
                output prg_allow,                            // Enable access to memory for the specified operation.
                input [13:0] chr_ain, output [21:0] chr_aout,
                output chr_allow,                      // Allow write
                output reg vram_a10,                         // Value for A10 address line
                output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
                // sd2snes video-bridge CHR-bank tap (raw CNROM latch)
                output [1:0] chr_snap_a53chr,
                // sd2snes video-bridge NT-arrangement tap (v2.0a): raw mode[1:0]
                // (00=1-screen lower, 01=1-screen upper, 10=vertical,
                // 11=horizontal -- same field vram_a10 decodes; covers AxROM's
                // dynamic single-screen page select via mode[0]).
                output [1:0] snap_mirror);
    reg [6:0] a53prg;    // output PRG ROM (A14-A20 on ROM)
    reg [1:0] a53chr;    // output CHR RAM (A13-A14 on RAM)

    reg [3:0] inner;    // "inner" bank at 01h
    reg [5:0] mode;     // mode register at 80h
    reg [5:0] outer;    // "outer" bank at 81h
    reg [1:0] selreg;   // selector register

    // Allow writes to 0x5000 only when launching through the proper mapper ID.
    wire [7:0] mapper = flags[7:0];
    wire allow_select = (mapper == 8'd28);

    always @(posedge clk) if (reset) begin
      mode[5:2] <= 0;         // NROM mode, 32K mode
      outer[5:0] <= 6'h3f;    // last bank
      inner <= 0;
      selreg <= 1;
      // sd2snes: a53chr had NO reset upstream (relied on FPGA power-up-0;
      // X in simulation for mappers that never write it -- 0/2/7).  Reset it
      // for sim determinism (the bridge CHR-bank tap reads it); hardware
      // behavior unchanged (registers power up 0 anyway), and CNROM's
      // power-on latch state is bank 0 by the same convention the golden
      // simulator uses (mappers.py chr_bank=0 initial).
      a53chr <= 0;

      // Set value for mirroring
      if (mapper == 2 || mapper == 0 || mapper == 3)
        mode[1:0] <= flags[14] ? 2'b10 : 2'b11;

      // UNROM #2 - Current bank in $8000-$BFFF and fixed top half of outer bank in $C000-$FFFF
      if (mapper == 2)
        mode[5:2] <= 4'b1111;

      // CNROM #3 - Fixed PRG bank, switchable CHR bank.
      if (mapper == 3)
        selreg <= 0;

      // AxROM #7 - Switch 32kb rom bank + switchable nametables
      if (mapper == 7) begin
        mode[1:0] <= 2'b00;   // Switchable VRAM page.
        mode[5:2] <= 4'b1100; // 256K banks, (B)NROM mode
      end
    end else if (ce) begin
      if ((prg_ain[15:12] == 4'h5) & prg_write && allow_select) selreg <= {prg_din[7], prg_din[0]};        // select register
      if (prg_ain[15] & prg_write) begin
        case (selreg)
        2'h0:  {mode[0], a53chr}  <= {(mode[1] ? mode[0] : prg_din[4]), prg_din[1:0]};  // CHR RAM bank
        2'h1:  {mode[0], inner}   <= {(mode[1] ? mode[0] : prg_din[4]), prg_din[3:0]};  // "inner" bank
        2'h2:  {mode}             <= {prg_din[5:0]};                                    // mode register
        2'h3:  {outer}            <= {prg_din[5:0]};                                    // "outer" bank
        endcase
      end
    end

    always @* begin
      // mirroring mode
      casez(mode[1:0])
      2'b0?   :   vram_a10 = {mode[0]};        // 1 screen lower
      2'b10   :   vram_a10 = {chr_ain[10]};    // vertical
      2'b11   :   vram_a10 = {chr_ain[11]};    // horizontal
      endcase

      // PRG ROM bank size select
      casez({mode[5:2], prg_ain[14]})
      5'b00_0?_?  :  a53prg = {outer[5:0],             prg_ain[14]};  // 32K banks, (B)NROM mode
      5'b01_0?_?  :  a53prg = {outer[5:1], inner[0],   prg_ain[14]};  // 64K banks, (B)NROM mode
      5'b10_0?_?  :  a53prg = {outer[5:2], inner[1:0], prg_ain[14]};  // 128K banks, (B)NROM mode
      5'b11_0?_?  :  a53prg = {outer[5:3], inner[2:0], prg_ain[14]};  // 256K banks, (B)NROM mode

      5'b00_10_1,
      5'b00_11_0  :  a53prg = {outer[5:0], inner[0]};             // 32K banks, UNROM mode
      5'b01_10_1,
      5'b01_11_0  :  a53prg = {outer[5:1], inner[1:0]};           // 64K banks, UNROM mode
      5'b10_10_1,
      5'b10_11_0  :  a53prg = {outer[5:2], inner[2:0]};           // 128K banks, UNROM mode
      5'b11_10_1,
      5'b11_11_0  :  a53prg = {outer[5:3], inner[3:0]};           // 256K banks, UNROM mode

      default     :  a53prg = {outer[5:0],             prg_ain[14]};  // 16K fixed bank
      endcase
    end

  assign vram_ce = chr_ain[13];
  // DEVIATION from upstream fpganes (documented; found by the blargg
  // interrupt-suite bring-up): upstream Mapper28 exposes NO PRG-RAM -- reads
  // and writes at $6000-$7FFF are dead (prg_allow=0, writes dropped, reads
  // return the mapper's 8'hff). But blargg's test harness (and Family-Basic-
  // style NROM boards, and effectively every emulator's mapper-0 behavior)
  // expects 8KB of PRG-RAM at $6000-$7FFF: the tests write their status byte
  // to $6000, the DE B0 61 magic to $6001-6003 and result text to $6004+, and
  // the harness reads the region back too. Mirror MMC1's prg_is_ram mapping
  // exactly (CART-RAM window at PSRAM 0x3C0000, ANALYSIS SS2.4) for all
  // mappers this module serves (0/2/3/7/28). Providing RAM where a real board
  // had none is harmless for well-behaved ROMs (they never touch it); a ROM
  // relying on open-bus reads at $6000 would now see RAM instead --
  // acceptable, matches common emulator behavior.
  wire prg_is_ram = prg_ain >= 'h6000 && prg_ain < 'h8000;
  wire [21:0] prg_ram = {9'b11_1100_000, prg_ain[12:0]};
  assign prg_aout = prg_is_ram ? prg_ram : {1'b0, (a53prg & 7'b0011111), prg_ain[13:0]};
  assign prg_allow = (prg_ain[15] && !prg_write) || prg_is_ram;
  assign chr_allow = flags[15];
  assign chr_aout = {7'b10_0000_0, a53chr, chr_ain[12:0]};

  // sd2snes bridge tap: raw CNROM CHR latch (register output only)
  assign chr_snap_a53chr = a53chr;
  // sd2snes bridge tap (v2.0a): raw mirror mode (dynamic for AxROM/#7; static
  // header value for 0/2/3 -- set from flags[14] at reset).
  assign snap_mirror     = mode[1:0];
endmodule

// MMC3 mapper chip -- mapper 4 (TxROM boards: TSROM/TLROM/TKROM/TGROM/...) and,
// since the second discrete batch, the NAMCO 108 FAMILY as a MODE of the same
// module: mappers 206 (Namco 108/109/118/119, Tengen MIMIC-1, NES-DxROM), 88,
// 95 and 154. Ported for Phase 2.1 from strigeus/fpganes `src/mmu.v:251-395`,
// which served mappers 4/47/118/119 from this one module. Only mapper 4 (plus
// the Namco family below) survives here.
//
// NAMCO 108 FAMILY (`n108` = mapper 206/88/95/154) -- NESdev INES_Mapper_206:
// "the simpler predecessor of the MMC3". It IS this module with three things
// taken away and, per variant, one added:
//   - "There are no IRQs" and "There are no control registers in the $A000-$FFFF
//     range": register mask $E001, i.e. only $8000-$9FFF decodes. Both facts are
//     one gate here (`n108_drop`): the $A000/$C000/$E000 arms of the write case
//     never run, so `mirroring`, `irq_latch`, `irq_reload` and -- the point --
//     `irq_enable` can never leave their reset value. `irq` is therefore 0 by
//     CONSTRUCTION (the a12 counter still runs; the only assignment that raises
//     `irq` is guarded by `irq_enable`), which is why no separate irq gate is
//     needed and none was added to the critical path.
//   - Bank select ($8000, even) is `xxxx xRRR`: "note the absence of any control
//     bits in the upper five bits of this register". d[7] (CHR A12 invert) and
//     d[6] (PRG ROM bank mode) DO NOT EXIST -- masked to 0 here, which is what
//     pins the family to "PRG always has the last two 8KiB banks fixed to the
//     end" and "the left pattern table gets the two 2KiB banks, the right one
//     the four 1KiB banks".
//   - Bank data ($8001, odd) is narrower: "only bits 5-1 exist for the two 2 KiB
//     CHR banks, only bits 5-0 exist for the four 1 KiB CHR banks, and only bits
//     3-0 exist for the two 8 KiB PRG banks" => 64KiB CHR / 128KiB PRG. The three
//     `n_mask*` constants below are exactly those three widths, applied on the D
//     input of the same registers mapper 4 uses.
// Per variant, from each mapper's own page:
//   206  hardwired mirroring (iNES header). Four-screen (Gauntlet) is rejected
//        by the loader, as it already was.
//   88   "CHR support is increased to 128 KiB by connecting PPU's A12 line to
//        the CHR ROM's A16 line", i.e. `$0000-$0FFF can only address CHR from
//        the first 64 KiB, $1000-$1FFF ... the second 64 KiB half`. The page's
//        own suggested implementation -- "mask the 1 KiB CHR-ROM bank output by
//        ANDing with $3F, and then OR it with $40 for Namco 108 registers 2, 3,
//        4, and 5" -- is what `n108_a16` does in chr_win_of: the AND $3F is the
//        n_mask25/n_mask01 width above, and the OR $40 keys off the PHYSICAL
//        window (k[2] = PPU A12), which is the same set as "registers 2..5"
//        because the family has no A12 invert. Mirroring: header.
//   154  "identical to Mapper 88, but with the addition of a single bit allowing
//        for one-screen nametable control ... $8000-$FFFF: [.N.. ....] 0 = 1ScA,
//        1 = 1ScB". NOTE the range: "the nametable control bit is present over
//        the entire 32kB range; unlike the associated Namco 108 which is only
//        present at $8000-$9FFF" -- so `ss_page` is latched OUTSIDE `n108_drop`,
//        on ANY $8000-$FFFF write (bank data writes included; the latch does not
//        see A0 either).
//   95   "CHR A15 directly controlling CIRAM A10, just as CHR A17 controls CIRAM
//        A10 on TxSROM", with bank register 0 also selecting "nametable at PPU
//        $2000-$27FF" and register 1 "$2800-$2FFF". The page's pseudocode is
//        `ciram_addr = ((namco108_chrmap(ppu_addr&0x1FFF)>>15)<<10) | ...`, i.e.
//        CIRAM A10 is bit 5 of the 1KB bank id the ordinary decode already
//        produces => `vram_a10 = chrsel[5]`, upstream's TxSROM `chrsel[7]` one
//        bit down. It is bit 5 of chr_bank_0/chr_bank_1 shifted by the 2KB pair
//        form, which is why the snapshot reads chr_bank_{0,1}[4].
//        DIVERGENCE from NES-MAPPERS-LOTE2.md 1.1, which says the bit is masked
//        out of the CHR id: the page says the opposite -- `chr_rom_addr =
//        namco108_chrmap(...) & 0xFFFF` KEEPS A15, and "homebrew using this
//        mapper could use the full 64 KiB". Nothing is masked here; Dragon
//        Buster's 32KiB image loses the bit through the ordinary size mask.
//
// CLOCKING: unlike MMC1/Mapper28 (clocked by cart_ce = 1 tick every 3 dots),
// MMC3 is clocked by `ppu_ce` = ONE PPU DOT, exactly as upstream instantiates it
// (fpganes/src/mmu.v:1592 `MMC3 mmc3(clk, ppu_ce, ...)`). The A12 scanline
// counter needs dot resolution: its 15-dot hysteresis filter (a12_ctr) is what
// makes a PPU fetch pattern look like one scanline tick. MultiMapper already
// carried ppu_ce in its signature and nes.v has always fed it the core `ce`
// (one PPU dot), so this port changes nothing outside this file. The pacer
// stretches WALL TIME, not the dot count, so the filter keeps its hardware
// meaning (15 dots = 5 M2 cycles).
//
// PRUNED (all three were served by the same upstream module; none is reachable
// here, and each one costs area or actively conflicts):
//   - mapper 47 (multicart): its block-select register is written by ANY write
//     to $6000-$7FFF, which collides with the PRG-RAM window, and it is the only
//     reason upstream's prg_allow/prg_aout carry `&& !mapper47`.
//   - TQROM (mapper 119): 8KB of CHR-RAM at a SECOND address space
//     (9'b11_1111_111); this core's CHR-RAM path is the 0x200000 one, and a
//     second one with no target game is pure debt.
//   - TxSROM (mapper 118): drives CIRAM A10 from chrsel[7] (a form of
//     four-screen), which the loader already rejects (src/nes.c) and which
//     nt_snap_arr cannot encode.
//   - `mmc3_alt_behavior`: upstream declares `wire mmc3_alt_behavior = 0`
//     (mmu.v:277) = MMC3C / Sharp MMC3B "normal" IRQ semantics. Kept at 0, so
//     the guard it controls is constant-true and is pruned to the constant (see
//     the IRQ block). CONSEQUENCE, by design: the "alternate"/rev-A IRQ test
//     ROMs (mmc3_test_2/6-MMC3_alt, mmc3_irq_tests/5.MMC3_rev_A) FAIL -- they
//     are declared expected-fails in run_mmc3.sh / run_mmc3_irq.sh. mapper_flags
//     bit [14] (the iNES mirroring bit, which MMC3 ignores because $A000 owns
//     mirroring) is the natural place to expose the variant one day -- RESERVED,
//     not used.
//
// TAPS: MMC3's CHR view is a VECTOR OF EIGHT 1KB WINDOWS, which the slot0/slot1
// pair of the legacy tap (chr_snap_*, CMD_CHR_STATE $12) cannot represent, so
// mapper 4 gets its OWN tap here (snap_chr_win, 8x8 bits) feeding the new
// CMD_CHR_STATE8 $14 / CMD_CHR_SPLITS8 $15 opcodes. The legacy pair stays at a
// CONSTANT sentinel for mapper 4 -- see the snapshot block in MultiMapper for
// why that constant IS the mechanism that suppresses $13. The mirror tap IS
// wired, because MMC3 owns mirroring dynamically via $A000 and the static
// header bit is meaningless for it.
module MMC3(input clk, input ce, input reset,
            input [31:0] flags,
            input [15:0] prg_ain, output [21:0] prg_aout,
            input prg_read, prg_write,                   // Read / write signals
            input [7:0] prg_din,
            output prg_allow,                            // Enable access to memory for the specified operation.
            input [13:0] chr_ain, output [21:0] chr_aout,
            output chr_allow,                            // Allow write
            output vram_a10,                             // Value for A10 address line
            output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
            output reg irq,
            // MMC6 only ($F4): the disabled half of its 1KB PRG-RAM reads back
            // as ZERO rather than as open bus, so the mapper answers that read
            // itself (nes.v: `from_data_bus = prg_dout_mapper` when prg_allow
            // is low).
            output [7:0] prg_dout,
            output has_prg_dout,
            // sd2snes video-bridge NT-arrangement tap (v2.0a molde): raw mirror
            // control in the SAME encoding MMC1/Mapper28 export
            // (00=1scr-lo, 01=1scr-hi, 10=vertical, 11=horizontal). MMC3 only
            // does V/H, so the high bit is tied 1 -- see the assign at the end.
            output [1:0] snap_mirror,
            // sd2snes video-bridge CHR WINDOW VECTOR tap (protocol v2.5): the
            // eight 1KB windows the PPU currently sees, window k in [k*8 +: 8].
            // UNMASKED (the size mask is applied where every other tap masks --
            // in MultiMapper); combinational here, registered under ce there.
            output [63:0] snap_chr_win,
                // sd2snes video-bridge NT-CODE tap (Fase 3, CMD_PPU_SPLITS
                // 0x16): the four per-quadrant CIRAM pages, bit k = the page of
                // logical nametable k.  snap_mirror above COLLAPSES them into
                // the 2-bit legacy code, and for mapper 95 that collapse is
                // LOSSY by construction -- R0.5 owns $2000-$27FF and R1.5 owns
                // $2800-$2FFF independently, so (1,0) is the inverted
                // horizontal arrangement the 4-code protocol has no name for
                // and snap_mirror has to report as plain horizontal.  Here it
                // is code 0x3, and distinguishable.  Appended at the END of the
                // port list on purpose: the instantiation is POSITIONAL.
                output [3:0] snap_ntpages);
  reg [2:0] bank_select;             // Register to write to next
  reg prg_rom_bank_mode;             // Mode for PRG banking
  reg chr_a12_invert;                // Mode for CHR banking
  reg mirroring;                     // 0 = vertical, 1 = horizontal
  reg irq_enable, irq_reload;        // IRQ enabled, and IRQ reload requested
  reg [7:0] irq_latch, counter;      // IRQ latch value and current counter
  reg ram_enable, ram_protect;       // RAM protection bits
  reg [6:0] chr_bank_0, chr_bank_1;  // Selected CHR banks
  reg [7:0] chr_bank_2, chr_bank_3, chr_bank_4, chr_bank_5;
  reg [5:0] prg_bank_0, prg_bank_1;  // Selected PRG banks
  reg       ss_page;                 // mapper 154 only: one-screen page ($8000 d6)
  wire prg_is_ram;

  // --- Namco 108 family mode (see the module header) -------------------------
  wire [7:0] mnum = flags[7:0];
  wire m206 = (mnum == 8'd206);
  wire m88  = (mnum == 8'd88);
  wire m95  = (mnum == 8'd95);
  wire m154 = (mnum == 8'd154);
  wire n108 = m206 | m88 | m95 | m154;
  // --- Taito TC0190 (33) / TC0690 (48) mode ---------------------------------
  // A SECOND mode of this module, for the same reason the Namco 108 family is
  // one: the Taito register file IS the MMC3's.  Two 2 KiB CHR banks at
  // $0000/$0800, four 1 KiB banks at $1000-$1FFF, two 8 KiB PRG banks at
  // $8000/$A000 with the last two fixed, V/H mirroring, and (on the 48) a
  // scanline IRQ that INES_Mapper_048 describes as "exactly like MMC3's" with
  // two documented exceptions.  Everything below -- prgsel, chr_win_of, the
  // A12 counter, vram_a10, both taps -- is reused unchanged; only the write
  // decoder is new.
  //
  // Not reproduced, and why: exception 2 of the page, "the IRQ seems to trip
  // about 4 CPU cycles later than on MMC3".  Reproducing a four-cycle skew
  // would mean a delay line on the one signal in this file that already sits
  // on the timing-critical path, for an effect the page itself calls "shaking
  // and other graphical quirks in some games" -- i.e. a raster-position
  // wobble, on a core whose bridge samples the CHR state once per frame.
  wire t33 = (mnum == 8'd33);
  wire t48 = (mnum == 8'd48);
  wire taito = t33 | t48;

  // --- MMC6 mode (internal alias $F4) ---------------------------------------
  // "The MMC3C and the MMC6 are alike, except the MMC6 has 1 KB of internal
  // PRG RAM with different write/enable controls."  So it is a THIRD mode of
  // this module and the entire bank/IRQ/mirroring machinery is shared; only
  // the RAM window changes.  NES 2.0 calls it mapper 4 submapper 1 and
  // `mapper_flags` has no submapper field, so nes.c translates it into the
  // internal byte $F4 -- the mechanism NES-MAPPERS-LOTE2.md 1.3 established.
  // THREE MIRRORS: src/nes.c, this file and bridge-sim/bridge_sim/mappers.py.
  wire mmc6 = (mnum == 8'hF4);
  reg  m6_rd_hi, m6_wr_hi, m6_rd_lo, m6_wr_lo;

  // --- Taito X1-005 (80) / X1-017 (82) mode ---------------------------------
  // A FOURTH mode, and the same argument as the TC0190: two 2 KiB CHR banks,
  // four 1 KiB banks, V/H mirroring and (on the X1-017) a CHR A12 inversion
  // that IS this module's chr_a12_invert.  What they add over the MMC3 is a
  // THIRD switchable 8 KiB PRG bank -- $8000/$A000/$C000 all move and only
  // $E000 is fixed -- and a register file that lives at $7EF0-$7EFF, i.e.
  // INSIDE the PRG-RAM window.
  //
  // "All registers exist at $7EFx and mirrored at $7E7x because CPU A7 line is
  // ignored", hence the A7-blind decode.
  //
  // MIRRORING POLARITY -- SETTLED BY MEASUREMENT, not by the wiki's wording.
  // Both X1 pages annotate the line as "0: vertical (A11); 1: horizontal
  // (A10)", and that pairing is BACKWARDS against every other page on the same
  // wiki: mapper 18 says "0: Horizontal (A11) / 1: Vertical (A10)" and the MMC3
  // says "0: horizontal (A10)" -- i.e. everywhere else vertical goes with PPU
  // A10.  So one of the X1 page's two halves has to be wrong, and the corpus
  // says it is the WORD, not the parenthesised physical line: measured in
  // Mesen, `harikiristadium_jp` and `harikirikoushien_jp` both write
  // $7EF6 = $01 exactly once and then render VERTICAL ($2000 -> page 0,
  // $2400 -> page 1, $2800 -> page 0) -- which is the "(A10)" of that page's
  // "1: horizontal (A10)".
  //   $7EF6 d0 = 1 -> CIRAM A10 follows PPU A10 = VERTICAL
  //   $7EF6 d0 = 0 -> CIRAM A10 follows PPU A11 = HORIZONTAL
  // which is `mirroring <= ~prg_din[0]` in this module's encoding (0 = A10).
  //
  // RESET comes from the iNES HEADER, not from a guessed latch value: the X1
  // register file is write-only and `minelvaton_jp` never touches $7EF6 for the
  // whole attract (400+ frames), so the only defensible arrangement until the
  // first write is the cartridge's own -- and its header says horizontal.  The
  // golden simulator latches the same way (mappers.py `mirroring_h = None`).
  //
  // NOT reproduced: the RAM permission registers ($7EF8/$7EF9 on the X1-005,
  // $7EF7/$7EF8/$7EF9 on the X1-017), which enable the internal RAM only after
  // a magic byte ($A3, or $CA/$69/$84).  The whole $6000-$7FFF window is
  // CART-RAM here and always live -- the same deviation ram_enable_eff already
  // is -- so the X1-005's 128 bytes at $7F00 and the X1-017's 5 KiB at
  // $6000-$73FF simply work.  Consequence to document rather than fix: a read
  // of $7EFx returns whatever was last written there instead of the zero the
  // ASIC's pull-downs give, because the register writes also land in the RAM.
  // No game in the compatibility sheet reads its own write-only registers.
  //
  // ALSO not reproduced: the X1-017's IRQ, which "was not used by the
  // commercial games and only reverse-engineered in January 2020".
  wire x80 = (mnum == 8'd80);
  wire x82 = (mnum == 8'd82);
  wire x1  = x80 | x82;
  reg [5:0] prg_bank_2;              // the third 8 KiB window, X1 only
  // The A7-BLIND mirror is an X1-005 property only: "all registers exist at
  // $7EFx and mirrored at $7E7x because CPU A7 line is ignored" appears on the
  // X1-005 page and NOT on the X1-017's, so the 82 decodes $7EF0-$7EFF exactly
  // and a write to $7E7x there is an ordinary RAM write.
  wire      x1_wr = prg_write && x1 && (prg_ain[15:8] == 8'h7e)
                                    && (prg_ain[6:4] == 3'b111)
                                    && (x80 || prg_ain[7]);
  wire [3:0] x1_r = prg_ain[3:0];
  // 88/154: PPU A12 is wired to CHR ROM A16 (INES_Mapper_088 "OR it with $40 for
  // Namco 108 registers 2, 3, 4, and 5" -- the $1000-$1FFF windows).
  wire n108_a16 = m88 | m154;
  // Register mask $E001: only $8000-$9FFF decodes ("There are no control
  // registers in the $A000-$FFFF range"). This one gate removes mirroring, the
  // IRQ latch/reload AND irq_enable -- which is why `irq` is 0 by construction.
  wire n108_drop = n108 & (prg_ain[14] | prg_ain[13]);
  // Bank data widths, verbatim from INES_Mapper_206 (bits 5-1 / 5-0 / 3-0).
  wire [6:0] n_mask01 = n108 ? 7'h1f : 7'h7f;   // R0/R1, held as d[7:1]
  wire [7:0] n_mask25 = n108 ? 8'h3f : 8'hff;   // R2-R5, held as d[7:0]
  wire [5:0] n_mask67 = n108 ? 6'h0f : 6'h3f;   // R6/R7, held as d[5:0]

  // DEVIATION from upstream fpganes (documented; same class and same cause as
  // the Mapper28 PRG-RAM deviation at the end of that module): the PRG-RAM
  // enable bit is FORCED ON. Upstream honors `ram_enable` ($A001 bit 7), which
  // resets to 0, so $6000-$7FFF is dead until the ROM programs it.
  // Measured in the fixtures: mmc3_test/5-MMC3.nes and
  // mmc3_test_2/rom_singles/1-clocking.nes each contain FIVE `STA $6000`-family
  // opcodes and ZERO `STA $A001` (8D 01 A0) -- i.e. the blargg harness writes
  // its status byte/magic/result text to the PRG-RAM window but never enables
  // it. With upstream semantics every one of those writes is dropped and the
  // testbench is blind (it polls CART-RAM at PSRAM 0x3C0000). Real TxROM boards
  // (TSROM/TLROM/TKROM) DO have the RAM chip -- only the enable is a register --
  // so handing a well-behaved ROM an already-enabled RAM is inert (it programs
  // $A001 before using it anyway). `ram_protect` ($A001 bit 6) stays FULLY
  // FUNCTIONAL, so write-protection semantics are unchanged. `ram_enable`
  // itself is still latched (kept for fidelity and for a future savestate of
  // the mapper state); it simply has no consumer while this deviation stands.
  wire ram_enable_eff = 1'b1;

  wire [7:0] new_counter = (counter == 0 || irq_reload) ? irq_latch : counter - 1;
  reg [3:0] a12_ctr;

  always @(posedge clk) if (reset) begin
    irq <= 0;
    bank_select <= 0;
    prg_rom_bank_mode <= 0;
    chr_a12_invert <= 0;
    // The X1 pair takes its arrangement from the HEADER until the first $7EF6
    // write (see the POLARITY note); every other member of this module resets
    // its own register to zero, which is vertical in this encoding.
    mirroring <= x1 ? ~flags[14] : 1'b0;
    {irq_enable, irq_reload} <= 0;
    {irq_latch, counter} <= 0;
    {ram_enable, ram_protect} <= 0;
    {chr_bank_0, chr_bank_1} <= 0;
    {chr_bank_2, chr_bank_3, chr_bank_4, chr_bank_5} <= 0;
    {prg_bank_0, prg_bank_1} <= 0;
    a12_ctr <= 0;
    ss_page <= 0;
    {m6_rd_hi, m6_wr_hi, m6_rd_lo, m6_wr_lo} <= 4'b0000;
    prg_bank_2 <= 0;
  end else if (ce) begin
    // Mapper 154's one-screen latch sits OUTSIDE the $E001 register mask: its
    // page says the bit "is present over the entire 32kB range", and the latch
    // does not decode A0 either, so a bank DATA write updates it as well.
    if (prg_write && prg_ain[15] && m154) ss_page <= prg_din[6];
    // --- Taito TC0190 / TC0690 decoder (see the mode block at the top) ------
    // Range/mask is $A003 on the 33 ("$8000-$BFFF") and $E003 on the 48
    // ("$8000-$FFFF"), so the register number is {A14, A13, A1, A0} and the 33
    // simply has no $C000/$E000 blocks -- which is why every arm for those two
    // blocks is gated on t48.
    if (prg_write && prg_ain[15] && taito) begin
      case ({prg_ain[14:13], prg_ain[1:0]})
      4'b00_00: begin prg_bank_0 <= prg_din[5:0];       // $8000 [.MPP PPPP]
                      if (t33) mirroring <= prg_din[6]; end
      4'b00_01: prg_bank_1 <= prg_din[5:0];             // $8001 [..PP PPPP]
      // "Unlike the MMC3, the value written for the two 2 KiB CHR banks does
      // not drop the LSB; the number written specifies the offset into CHR as
      // a multiple of 2 KiB."  That is exactly the register shape chr_win_of
      // already uses for the MMC3's R0/R1 ({register, j[0]}), so only the
      // SLICE of the written byte differs: d[6:0] here, d[7:1] there.  d[7] is
      // dropped -- the page says the MSB is implemented but "no games ever
      // used it", and a ninth window bit does not fit the byte the video
      // bridge publishes (the four 1 KiB registers are 256 KiB either way).
      4'b00_10: chr_bank_0 <= prg_din[6:0];             // 2 KiB @ $0000
      4'b00_11: chr_bank_1 <= prg_din[6:0];             // 2 KiB @ $0800
      4'b01_00: chr_bank_2 <= prg_din;                  // 1 KiB @ $1000
      4'b01_01: chr_bank_3 <= prg_din;                  // 1 KiB @ $1400
      4'b01_10: chr_bank_4 <= prg_din;                  // 1 KiB @ $1800
      4'b01_11: chr_bank_5 <= prg_din;                  // 1 KiB @ $1C00
      // "$C000 -> MMC3 $C000 (XOR written value with $FF), $C001 -> $C001,
      // $C002 -> $E001, $C003 -> $E000."
      4'b10_00: if (t48) irq_latch  <= ~prg_din;
      4'b10_01: if (t48) irq_reload <= 1;
      4'b10_10: if (t48) irq_enable <= 1;
      4'b10_11: if (t48) begin irq_enable <= 0; irq <= 0; end
      // $E000-$E003 [.M.. ....] -- the 48 puts mirroring here instead.
      default:  if (t48) mirroring <= prg_din[6];
      endcase
    end
    // --- Taito X1-005 / X1-017 decoder (see the mode block at the top) ------
    if (x1_wr) begin
      // "Bit 0 of written bank number is ignored when switching 2K banks
      // ($7EF0, $7EF1), just like other common mappers like the MMC3" -- so
      // unlike the TC0190 these ARE the MMC3's own d[7:1] registers.
      if (x1_r == 4'd0) chr_bank_0 <= prg_din[7:1];   // 2 KiB @ $0000
      if (x1_r == 4'd1) chr_bank_1 <= prg_din[7:1];   // 2 KiB @ $0800
      if (x1_r == 4'd2) chr_bank_2 <= prg_din;        // 1 KiB @ $1000
      if (x1_r == 4'd3) chr_bank_3 <= prg_din;
      if (x1_r == 4'd4) chr_bank_4 <= prg_din;
      if (x1_r == 4'd5) chr_bank_5 <= prg_din;
      // $7EF6.  "This register is not mirrored at $7EF7."  On the X1-017 bit 1
      // is the CHR A12 inversion, which is exactly chr_a12_invert.
      if (x1_r == 4'd6) begin
        mirroring <= ~prg_din[0];                     // see the POLARITY note
        if (x82) chr_a12_invert <= prg_din[1];
      end
      // PRG.  X1-005: $7EFA/$7EFC/$7EFE, "mirrored at right next respective
      // addresses", six bits.  X1-017: $7EFA/$7EFB/$7EFC, and the mask ROM's
      // address lines are reversed so iNES 082 reads the bank out of d[5:2]
      // ("xxDC BAxx", PRG A13..A16 = bits 2..5) -- which is d[5:2] taken as a
      // plain number, capped at 128 KiB exactly as the page says.
      if (x80) begin
        if (x1_r[3:1] == 3'd5) prg_bank_0 <= prg_din[5:0];   // $7EFA/$7EFB
        if (x1_r[3:1] == 3'd6) prg_bank_1 <= prg_din[5:0];   // $7EFC/$7EFD
        if (x1_r[3:1] == 3'd7) prg_bank_2 <= prg_din[5:0];   // $7EFE/$7EFF
      end
      if (x82) begin
        if (x1_r == 4'd10) prg_bank_0 <= {2'b00, prg_din[5:2]};
        if (x1_r == 4'd11) prg_bank_1 <= {2'b00, prg_din[5:2]};
        if (x1_r == 4'd12) prg_bank_2 <= {2'b00, prg_din[5:2]};
      end
    end
    if (prg_write && prg_ain[15] && !n108_drop && !taito && !x1) begin
      case({prg_ain[14], prg_ain[13], prg_ain[0]})
      // Bank select ($8000-$9FFE, even).  d[7]/d[6] do not exist on the Namco
      // 108 family ("the absence of any control bits in the upper five bits").
      3'b00_0: begin
        {chr_a12_invert, prg_rom_bank_mode, bank_select} <=
          {prg_din[7] & ~n108, prg_din[6] & ~n108, prg_din[2:0]};
        // MMC6 [CPMx xRRR]: bit 5 is the PRG-RAM enable.  That bit does not
        // exist on the MMC3, whose RAM enable is $A001 bit 7 instead.
        if (mmc6) ram_enable <= prg_din[5];
      end
      3'b00_1: begin // Bank data ($8001-$9FFF, odd)
        case (bank_select)
        0: chr_bank_0 <= prg_din[7:1] & n_mask01;  // Select 2 KB CHR bank at PPU $0000-$07FF (or $1000-$17FF);
        1: chr_bank_1 <= prg_din[7:1] & n_mask01;  // Select 2 KB CHR bank at PPU $0800-$0FFF (or $1800-$1FFF);
        2: chr_bank_2 <= prg_din & n_mask25;       // Select 1 KB CHR bank at PPU $1000-$13FF (or $0000-$03FF);
        3: chr_bank_3 <= prg_din & n_mask25;       // Select 1 KB CHR bank at PPU $1400-$17FF (or $0400-$07FF);
        4: chr_bank_4 <= prg_din & n_mask25;       // Select 1 KB CHR bank at PPU $1800-$1BFF (or $0800-$0BFF);
        5: chr_bank_5 <= prg_din & n_mask25;       // Select 1 KB CHR bank at PPU $1C00-$1FFF (or $0C00-$0FFF);
        6: prg_bank_0 <= prg_din[5:0] & n_mask67;  // Select 8 KB PRG ROM bank at $8000-$9FFF (or $C000-$DFFF);
        7: prg_bank_1 <= prg_din[5:0] & n_mask67;  // Select 8 KB PRG ROM bank at $A000-$BFFF
        endcase
      end
      3'b01_0: mirroring <= prg_din[0];                   // Mirroring ($A000-$BFFE, even)
      // PRG RAM protect ($A001-$BFFF, odd).  On the MMC6 the same address
      // carries FOUR bits instead [HhLl xxxx] -- read/write enables for the
      // two 512-byte halves -- and "when PRG RAM is disabled via $8000, the
      // mapper continuously sets $A001 to $00, and so all writes to $A001 are
      // ignored".
      3'b01_1: if (mmc6) begin
                 if (ram_enable)
                   {m6_rd_hi, m6_wr_hi, m6_rd_lo, m6_wr_lo} <= prg_din[7:4];
               end else
                 {ram_enable, ram_protect} <= prg_din[7:6];
      3'b10_0: irq_latch <= prg_din;                      // IRQ latch ($C000-$DFFE, even)
      3'b10_1: irq_reload <= 1;                           // IRQ reload ($C001-$DFFF, odd)
      3'b11_0: begin irq_enable <= 0; irq <= 0; end       // IRQ disable ($E000-$FFFE, even)
      3'b11_1: irq_enable <= 1;                           // IRQ enable ($E001-$FFFF, odd)
      endcase
    end

    // Trigger IRQ counter on rising edge of chr_ain[12]
    // All MMC3A's and non-Sharp MMC3B's will generate only a single IRQ when $C000 is $00.
    // This is because this version of the MMC3 generates IRQs when the scanline counter is decremented to 0.
    // In addition, writing to $C001 with $C000 still at $00 will result in another single IRQ being generated.
    // In the community, this is known as the "alternate" or "old" behavior.
    // All MMC3C's and Sharp MMC3B's will generate an IRQ on each scanline while $C000 is $00.
    // This is because this version of the MMC3 generates IRQs when the scanline counter is equal to 0.
    // In the community, this is known as the "normal" or "new" behavior.
    if (chr_ain[12] && a12_ctr == 0) begin
      counter <= new_counter;
      // sd2snes_nes: upstream guards this with
      //   (!mmc3_alt_behavior || counter != 0 || irq_reload)
      // and ties mmc3_alt_behavior to 0, making the guard constant-true. Pruned
      // to the constant (we are MMC3C / Sharp MMC3B, "normal" behavior); see the
      // module header for the expected-fail consequence on the rev-A test ROMs.
      if (new_counter == 0 && irq_enable) begin
        irq <= 1;
      end
      irq_reload <= 0;
    end
    a12_ctr <= chr_ain[12] ? 4'b1111 : (a12_ctr != 0) ? a12_ctr - 4'b0001 : a12_ctr;
  end

  // The PRG bank to load. Each increment here is 8kb. So valid values are 0..63.
  reg [5:0] prgsel;
  always @* begin
    casez({prg_ain[14:13], prg_rom_bank_mode})
    3'b00_0: prgsel = prg_bank_0;  // $8000 mode 0
    3'b00_1: prgsel = 6'b111110;   // $8000 fixed to second last bank
    3'b01_?: prgsel = prg_bank_1;  // $A000 mode 0,1
    3'b10_0: prgsel = 6'b111110;   // $C000 fixed to second last bank
    3'b10_1: prgsel = prg_bank_0;  // $C000 mode 1
    3'b11_?: prgsel = 6'b111111;   // $E000 fixed to last bank
    endcase
    // The X1 pair switches THREE windows and fixes only the last one; written
    // as an override so the MMC3's own casez keeps its exact shape.
    if (x1) case (prg_ain[14:13])
            2'b00: prgsel = prg_bank_0;    // $8000
            2'b01: prgsel = prg_bank_1;    // $A000
            2'b10: prgsel = prg_bank_2;    // $C000
            2'b11: prgsel = 6'b111111;     // $E000 = last
            endcase
  end

  // The CHR bank to load. Each increment here is 1kb. So valid values are 0..255.
  //
  // SINGLE IMPLEMENTATION (do not fork this): upstream writes the selection as a
  // casez over {chr_ain[12]^inv, chr_ain[11], chr_ain[10]} (fpganes mmu.v:370-377).
  // The sd2snes bridge needs the SAME function evaluated for all eight windows at
  // once (snap_chr_win, below), so the body moved into `chr_win_of` and both the
  // address path and the tap call it. Forking them would let the tap drift from
  // what the PPU actually fetches -- the one failure mode this arrangement makes
  // impossible.
  //
  //   j = {k[2] ^ chr_a12_invert, k[1], k[0]}     (k = the PPU's chr_ain[12:10])
  //   j[2]==0 : win = {(j[1] ? chr_bank_1 : chr_bank_0), j[0]}   (2KB pair)
  //   j[2]==1 : win = chr_bank_{2 + j[1:0]}                      (1KB slot)
  //
  // j[0]==k[0] (the XOR only touches bit 2), which is why the low bit of a 2KB
  // pair is the raw PPU address bit exactly as in the upstream casez. 8 bits =
  // 256 x 1KB = 256KB = the full width of chrsel, so the vector is loss-less.
  function [7:0] chr_win_of;
    input [2:0] k;
    reg [2:0] j;
    reg [7:0] v;
    begin
      j = {k[2] ^ chr_a12_invert, k[1], k[0]};
      if (!j[2]) v = {(j[1] ? chr_bank_1 : chr_bank_0), j[0]};
      else case (j[1:0])
        2'd0: v = chr_bank_2;
        2'd1: v = chr_bank_3;
        2'd2: v = chr_bank_4;
        2'd3: v = chr_bank_5;
      endcase
      // Mappers 88/154: PPU A12 drives CHR ROM A16, so the right pattern table
      // comes from the second 64KiB half.  Keyed off k[2] (the PHYSICAL window
      // the PPU is fetching = PPU A12), which is what the wire on the board is;
      // the family has no A12 invert, so k[2] == j[2] and this is also exactly
      // "registers 2, 3, 4 and 5" as INES_Mapper_088 words it.  The page's
      // companion "AND with $3F" is the n_mask01/n_mask25 register width.
      chr_win_of = v | {1'b0, n108_a16 & k[2], 6'd0};
    end
  endfunction

  reg [7:0]  chrsel;
  reg [63:0] snap_win_r;
  // EXPLICIT sensitivity list, not @*: the right-hand sides are FUNCTION CALLS,
  // and whether @* looks through a function body at the variables it reads is
  // exactly the kind of tool-dependent corner that costs a debugging session
  // (Icarus left the vector at X). Listing the six bank registers + the invert
  // bit + chr_ain + n108_a16 is complete by construction -- chr_win_of reads
  // nothing else.  n108_a16 is quasi-static (it only moves when the loader
  // changes `flags`), but LEAVING IT OUT would freeze the vector at the value
  // it had for the previous ROM in simulation, which is precisely the class of
  // bug this list exists to prevent.
  always @(chr_ain or chr_a12_invert or n108_a16 or chr_bank_0 or chr_bank_1
           or chr_bank_2 or chr_bank_3 or chr_bank_4 or chr_bank_5) begin
    chrsel     = chr_win_of(chr_ain[12:10]);
    // sd2snes bridge tap: the whole window vector (see the port comment).
    snap_win_r = {chr_win_of(3'd7), chr_win_of(3'd6),
                  chr_win_of(3'd5), chr_win_of(3'd4),
                  chr_win_of(3'd3), chr_win_of(3'd2),
                  chr_win_of(3'd1), chr_win_of(3'd0)};
  end
  assign snap_chr_win = snap_win_r;

  wire [21:0] prg_aout_tmp = {3'b00_0,  prgsel, prg_ain[12:0]};

  assign {chr_allow, chr_aout} = {flags[15], 4'b10_00, chrsel, chr_ain[9:0]};

  // PRG-RAM window: same CART-RAM mapping MMC1/Mapper28 use (PSRAM 0x3C0000).
  // `ram_enable_eff` is the deviation documented at the top of this module;
  // `ram_protect` is honored as upstream does.  For the Namco 108 family the
  // board has NO PRG-RAM at all ("There is no PRG-RAM support, except for
  // Magical Puzzle Popils"), and `ram_protect` can no longer be written, so the
  // window is unconditionally live there.  Same class of deviation as the one
  // at the end of module Mapper28: handing a well-behaved ROM RAM it never
  // touches is inert, and gating it would cost logic on `prg_allow`, which is
  // the one output of this file that sits on the critical path.
  // MMC6: 1 KB of RAM at $7000-$7FFF, MIRRORED (so the address is prg_ain[9:0]
  // and the two protection halves are picked by A9), and NOTHING at
  // $6000-$6FFF.  A half that is not read-enabled is not accessible at all;
  // if the OTHER half is enabled it reads back as zero, and if neither is the
  // window is open bus.  "The write-enable bits only have effect if that bank
  // is enabled for reading."
  wire m6_win = mmc6 && (prg_ain[15:12] == 4'h7);
  wire m6_rd  = ram_enable & (prg_ain[9] ? m6_rd_hi : m6_rd_lo);
  wire m6_wre = ram_enable & (prg_ain[9] ? m6_wr_hi : m6_wr_lo);
  wire m6_ok  = m6_win && (prg_write ? (m6_rd & m6_wre) : m6_rd);
  wire mmc3_is_ram = !mmc6 && prg_ain >= 'h6000 && prg_ain < 'h8000
                     && ram_enable_eff && !(ram_protect && prg_write);
  assign prg_is_ram = mmc3_is_ram | m6_ok;
  assign prg_allow = prg_ain[15] && !prg_write || prg_is_ram;
  wire [21:0] prg_ram = mmc6 ? {9'b11_1100_000, 3'b000, prg_ain[9:0]}
                             : {9'b11_1100_000, prg_ain[12:0]};
  assign prg_aout = prg_is_ram ? prg_ram : prg_aout_tmp;
  assign prg_dout = 8'h00;
  assign has_prg_dout = m6_win && !m6_rd && (ram_enable & (m6_rd_hi | m6_rd_lo));
  assign vram_ce = chr_ain[13];

  // --- Mirroring ------------------------------------------------------------
  // Mapper 4 owns it through $A000; the Namco 108 family has no $A000 at all,
  // so each variant gets it from where its own page says:
  //   206/88  the iNES header (hardwired on the board)
  //   154     the $8000 d6 latch (0 = 1ScA, 1 = 1ScB)
  //   95      CHR A15 = bit 5 of the 1KB bank id the ordinary decode produces,
  //           i.e. chrsel[5] -- upstream's TxSROM `chrsel[7]`, one bit down,
  //           and exactly the page's `(namco108_chrmap(...)>>15)`.  A12 is
  //           carried along for free: at $2000-$2FFF the PPU has A12=0 so
  //           chrsel comes from R0/R1 selected by A11, which is what "register
  //           0 also selects the nametable at $2000-$27FF" means.
  wire mmc3_a10 = mirroring  ? chr_ain[11] : chr_ain[10];
  wire hdr_a10  = flags[14]  ? chr_ain[10] : chr_ain[11];
  assign vram_a10 = !n108 ? mmc3_a10
                  : m95   ? chrsel[5]
                  : m154  ? ss_page
                  :         hdr_a10;

  // sd2snes bridge tap (v2.0a molde): MMC3 mirroring is DYNAMIC ($A000 bit 0)
  // and the iNES header bit does not reflect it, so nt_snap_arr must come from
  // here. Encoding: {1'b1, mirroring} -> 2'b10 = vertical, 2'b11 = horizontal,
  // which is exactly what vram_a10 decodes above and what ntarr_of() expects.
  //
  // Mapper 95 is the one member the 4-code protocol cannot describe exactly:
  // R0.5 owns $2000-$27FF and R1.5 owns $2800-$2FFF independently, which is
  // four states where NTARR has three reachable ones.  (0,0) = both halves on
  // page A = 1ScA; (1,1) = 1ScB; (0,1) = CIRAM A10 follows PPU A11 = plain
  // horizontal.  (1,0) is the INVERTED horizontal arrangement and has no code
  // -- reported as horizontal (documented in NES-MAPPERS-LOTE2.md 1.1; Dragon
  // Buster is the only title and never uses it).  R0.5 is chr_bank_0[4] because
  // the 2KB pair form shifts the register left by one to make the 1KB id.
  wire n95_r0 = chr_bank_0[4];
  wire n95_r1 = chr_bank_1[4];
  wire [1:0] n108_raw = m95  ? {n95_r0 ^ n95_r1, n95_r0 | n95_r1}
                      : m154 ? {1'b0, ss_page}
                      :        {1'b1, ~flags[14]};
  assign snap_mirror = n108 ? n108_raw : {1'b1, mirroring};
  // The same four decisions vram_a10 makes above, published as pages instead of
  // as a 2-bit approximation.  Mapper 4: `mirroring` 1 = A10 follows PPU A11 =
  // horizontal = (0,0,1,1); 0 = A10 follows PPU A10 = vertical = (0,1,0,1).
  // 206/88: the hardwired header bit, same two codes.  154: both halves on the
  // latched page.  95: R0.5 for quadrants 0/1, R1.5 for 2/3 -- the exact
  // hardware form, lockstep with bridge_sim/mappers.py Mapper95.nt_pages().
  assign snap_ntpages = !n108 ? (mirroring ? 4'hc : 4'ha)
                      : m95   ? {n95_r1, n95_r1, n95_r0, n95_r0}
                      : m154  ? {4{ss_page}}
                      :         (flags[14] ? 4'ha : 4'hc);
endmodule

// MapperDiscrete -- the whole discrete-mapper batch in ONE module: mappers 11
// (Color Dreams), 66 (GxROM), 79 (NINA-03/06), 113 (NINA-06 with mirroring
// control), 87 (Jaleco JF-05/06/07/08/09/10), 34 (BNROM *and* NINA-001), 71
// (Camerica BF9093/BF9097), 232 (Camerica Quattro) and -- second batch --
// 93 (Sunsoft-3R), 89 (Sunsoft-3), 94 (HVC-UN1ROM), 97 (Irem TAM-S1),
// 180 (UNROM inverted / Crazy Climber), 184 (Sunsoft-1), 70 (Bandai 74161),
// 152 (Bandai 74161 + one-screen), 78 (Holy Diver / Cosmo Carrier),
// 86 (Jaleco JF-13) and 140 (Jaleco JF-11/JF-14).
//
// SECOND BATCH, one line per board, all from the NESdev page of the same
// number (register diagrams quoted verbatim; d = the byte written):
//   93   $8000-$FFFF [.PPP ...E]  P = 16K @ $8000 (d[6:4]), last 16K fixed at
//        $C000; CHR is 8K RAM and E (d[0]) is its WRITE ENABLE -- "0 = RAM is
//        disabled; writes are ignored and reads are open bus". Mirroring: board.
//   89   $8000-$FFFF [CPPP MCCC]  C = 8K CHR ({d[7], d[2:0]}), P = 16K @ $8000
//        (d[5:3]), M (d[6]) = one screen, 0 = 1ScA / 1 = 1ScB.
//   94   $8000-$FFFF xxxP PPxx    16K @ $8000 = d[4:2] ("very similar to UxROM,
//        but the register is shifted by two bits"); CHR 8K RAM; board mirroring.
//   97   $8000-$BFFF [M..P PPPP]  P = 16K @ $C000 (d[4:0]) with $8000 fixed to
//        the LAST bank -- the inverted layout; M (d[7]) = 0 Horz / 1 Vert.
//   180  $8000-$FFFF xxxx xPPP    16K @ $C000 = d[2:0] with $8000 fixed to the
//        FIRST bank ("uses AND logic instead of OR logic"); CHR 8K RAM.
//   184  $6000-$7FFF [.1HH .LLL]  two 4K CHR banks: $0000 = d[2:0], $1000 =
//        {1'b1, d[5:4]} -- the diagram's hardwired '1' is the "only banks 4-7
//        selectable" note. PRG is a fixed 32K. The register lives in the RAM
//        window, so "this mapper precludes iNES-default RAM".
//   70   $8000-$FFFF [PPPP CCCC]  P = 16K @ $8000, C = 8K CHR; board mirroring.
//   152  $8000-$FFFF [MPPP CCCC]  = 70 plus M (d[7]) one screen (0 = 1ScA).
//   78   $8000-$FFFF [CCCC MPPP]  C = 8K CHR, P = 16K @ $8000, M = d[3]. The
//        two boards read M DIFFERENTLY and are told apart by the NES 2.0
//        submapper, which mapper_flags has no room for -- see the ALIAS note.
//   86   $6000-$6FFF [.CPP ..CC]  32K PRG = d[5:4], 8K CHR = {d[6], d[1:0]}.
//        ($7000-$7FFF is the uPD7756C speech chip -- no audio here, ignored.)
//   140  $6000-$7FFF [..PP CCCC]  32K PRG = d[5:4], 8K CHR = d[3:0].
//
// ALIAS $F8 = mapper 78 submapper 1 (Uchuusen - Cosmo Carrier). There is no
// submapper bit in mapper_flags (all 16 are spoken for), so the loader folds
// (mapper, submapper) into a private byte in the $F0-$FE range that is never a
// real iNES number. THREE PLACES SHARE THIS TABLE and must move together:
// this file, src/nes.c and bridge-sim/mappers.py. iNES 1.0 78 (no submapper)
// stays Holy Diver, the famous title. On Holy Diver d[3] picks H/V through a
// 74HC00 mux of PPU A10/A11; on Cosmo Carrier the same latch bit "is connected
// directly to CIRAM A10, as in AxROM" = one screen A/B.
//
// BUS CONFLICTS are not emulated for any member (upstream fpganes does not
// either, and neither did the first batch): every one of these boards ANDs the
// written value with the ROM byte at that address. Commercial software writes
// the value that matches, which is why this has never mattered; a homebrew that
// relies on the conflict would misbehave.
//
// WHY ONE MODULE (and not five ports of upstream's Mapper66/34/79/71 + a new
// Mapper87): every arm added to MultiMapper's `case(flags[7:0])` is another
// input to a 48-bit-wide mux (prg_aout + chr_aout + the five control bits), and
// every separate module carries its own copy of registers that are dead for all
// the other mappers. These eight mappers are all "latch a few bits, index a 32KB
// (or 16KB) PRG window and an 8KB (or 4KB) CHR window", so they collapse into
// ONE register file -- prg_bank/chr_bank0/chr_bank1/mirr/ss_mode, 14 flip-flops
// total -- and ONE output stage, reached through ONE case arm. Only the write
// decode is per-mapper, and that is a handful of comparators on the quasi-static
// mapper number.
//
// Semantics are those of the upstream modules, which stay the reference:
//   11/66  -> fpganes src/mmu.v:1040 (Mapper66)
//   34     -> fpganes src/mmu.v:1073 (Mapper34)   [BNROM + NINA-001]
//   71/232 -> fpganes src/mmu.v:1312 (Mapper71)
//   79/113 -> fpganes src/mmu.v:1358 (Mapper79)
//   87     -> no upstream module; NESdev (JF-05..JF-10): a write to $6000-$7FFF
//             selects the 8KB CHR bank with the two low data bits SWAPPED,
//             bank = {d[0], d[1]}. PRG is fixed (NROM); a 16KB image is mirrored
//             by MultiMapper's prg_mask, exactly as MMC0 does it.
//
// Registers (shared across all nineteen mappers; only one is ever active):
//   prg_bank[4:0]  32KB bank for 11/66/79/113/34/86/140 (low bits only), 16KB
//                  bank for everything else (the 232 "outer" pair lives in
//                  [3:2]).  FIVE bits because mapper 97 is the only member with
//                  a 5-bit selector; the widest 32KB user needs 3.
//   chr_bank0[3:0] 8KB CHR bank for 11/66/79/113/87/89/70/152/78/86/140; the
//                  $0000 4KB bank for 34/NINA and 184. Stays 0 for the fixed
//                  CHR-RAM members (71/232/93/94/97/180) -- that is what lets
//                  the 8KB output form below serve them too.
//   chr_bank1[3:0] the $1000 4KB bank for 34/NINA and 184. RESET TO 1 (upstream
//                  Mapper34 does the same) so that 34/BNROM, whose CHR is a
//                  fixed 8KB RAM, reads out as the linear 0/1 pair.
//   mirr           the ONE dynamic mirroring bit: 113 ($4100-$5FFF d[7]) and 78
//                  /97 (1 = vertical) on one side, and the single-screen PAGE
//                  for 71 (armed), 89, 152 and $F8 on the other.  The two
//                  readings never coexist because only one mapper is active;
//                  `ss_sel` below picks which one applies.  Note that the
//                  polarity is shared: every V/H member has 1 = vertical.
//   ss_mode        71's single-screen latch ($9000-$9FFF) -- see FIRE HAWK below.
//   wren           93's CHR-RAM write enable (d[0]).  Its power-on state is NOT
//                  documented on the page, so it resets to 1 = "normal
//                  operation": a ROM that never writes the register then behaves
//                  like plain CHR-RAM instead of being invisible.  Only the
//                  write half is reproduced; the page's "reads are open bus" is
//                  deliberately NOT, because the page also says the feature is
//                  "like iNES Mapper 185 except no games use this" -- reads
//                  return whatever the RAM holds.  NOTE for whoever revisits
//                  mapper 185: the override path this would need DOES exist and
//                  IS live (nes.v `chr_to_ppu = has_chr_from_ppu_mapper ?
//                  chr_from_ppu_mapper : memory_din_ppu`), it is simply held at
//                  the inert default by every mapper in this file.  The claim in
//                  NES-MAPPERS-LOTE1.md 0 that `has_chr_dout` is "not consumed
//                  by nes.v" no longer holds.
module MapperDiscrete(input clk, input ce, input ppu_ce, input reset,
                      input [31:0] flags,
                      input [15:0] prg_ain, output [21:0] prg_aout,
                      input prg_read, prg_write,               // Read / write signals
                      input [7:0] prg_din,
                      output prg_allow,                        // Enable access to memory for the specified operation.
                      input chr_read,                          // PPU is reading VRAM (mapper 185 only)
                      input [13:0] chr_ain, output [21:0] chr_aout,
                      output chr_allow,                        // Allow write
                      // Mapper 185 only: the CHR-ROM chip is deselected, so a
                      // pattern-table read must NOT come from memory.  See the
                      // MAPPER 185 block below.
                      output chr_dis,
                      output vram_a10,                         // Value for A10 address line
                      output vram_ce,                          // True if the address should be routed to the internal 2kB VRAM.
                      // sd2snes video-bridge CHR-bank tap (raw register state;
                      // MultiMapper derives the per-slot snapshot and applies
                      // the size mask -- see chr_snap_* there).
                      output [3:0] snap_chr0,
                      output [3:0] snap_chr1,
                      // sd2snes video-bridge NT-arrangement tap: the raw mirror
                      // code every other mapper here exports (00=1scr-lo,
                      // 01=1scr-hi, 10=vertical, 11=horizontal), which is the
                      // same field vram_a10 decodes below.
                      output [1:0] snap_mirror);
  reg [4:0] prg_bank;
  reg [3:0] chr_bank0, chr_bank1;
  reg       mirr;
  reg       ss_mode;
  reg       wren;
  reg [1:0] lat72;                 // mapper 72: previous state of the P and C bits
  reg [1:0] rd185;                 // mapper 185: pattern-table reads seen since reset
  reg       chr_read_q;

  wire [7:0] mapper = flags[7:0];
  wire m11  = (mapper == 8'd11);
  wire m66  = (mapper == 8'd66);
  wire m79  = (mapper == 8'd79);
  wire m113 = (mapper == 8'd113);
  wire m87  = (mapper == 8'd87);
  wire m34  = (mapper == 8'd34);
  wire m71  = (mapper == 8'd71);
  wire m232 = (mapper == 8'd232);
  wire m93  = (mapper == 8'd93);
  wire m89  = (mapper == 8'd89);
  wire m94  = (mapper == 8'd94);
  wire m97  = (mapper == 8'd97);
  wire m180 = (mapper == 8'd180);
  wire m184 = (mapper == 8'd184);
  wire m70  = (mapper == 8'd70);
  wire m152 = (mapper == 8'd152);
  wire m78  = (mapper == 8'd78);
  wire mF8  = (mapper == 8'hF8);   // 78 submapper 1 (Cosmo Carrier) -- see ALIAS
  wire m86  = (mapper == 8'd86);
  wire m140 = (mapper == 8'd140);
  wire m72  = (mapper == 8'd72);
  wire m185 = (mapper == 8'd185);
  wire m78any = m78 | mF8;
  // Upstream Mapper34's rule verbatim: NINA-001 is assumed whenever the image
  // carries more than 8KB of CHR. A 34 with exactly 8KB of CHR-ROM therefore
  // loads as BNROM; no known title is in that case (documented in the loader).
  wire nina = m34 && (flags[13:11] != 3'd0);

  always @(posedge clk) if (reset) begin
    prg_bank  <= 5'd0;
    chr_bank0 <= 4'd0;
    // upstream Mapper34: "To be compatible with BxROM" -- but on the Sunsoft-1
    // the diagram's hardwired '1' is a WIRE, not a latch bit ("only banks 4-7
    // selectable"), so bank 4 is already on the bus at power-on with the latch
    // still zero.  Getting this wrong is invisible until a 184 game draws its
    // first frame from the high pattern table.
    chr_bank1 <= m184 ? 4'd4 : 4'd1;
    mirr      <= 1'b0;
    ss_mode   <= 1'b0;
    wren      <= 1'b1;   // mapper 93: undocumented at power-on, see the header
    lat72     <= 2'b00;
  end else if (ce && prg_write) begin
    // 11 / 66 -- any write to $8000-$FFFF. Color Dreams puts CHR in d[7:4] and
    // PRG in d[1:0]; GxROM puts PRG in d[5:4] and CHR in d[1:0].
    if ((m11 || m66) && prg_ain[15]) begin
      prg_bank  <= m66 ? {3'b000, prg_din[5:4]} : {3'b000, prg_din[1:0]};
      chr_bank0 <= m66 ? {2'b00, prg_din[1:0]} : prg_din[7:4];
    end
    // 79 / 113 -- $4100-$5FFF with A8 set. One byte carries everything:
    // {mirroring, chr[3], prg[2:0], chr[2:0]}.
    if ((m79 || m113) && prg_ain[15:13] == 3'b010 && prg_ain[8]) begin
      prg_bank  <= {2'b00, prg_din[5:3]};
      chr_bank0 <= {prg_din[6], prg_din[2:0]};
      mirr      <= prg_din[7];
    end
    // 87 -- $6000-$7FFF, and the two CHR bits arrive SWAPPED.
    if (m87 && prg_ain[15:13] == 3'b011)
      chr_bank0 <= {2'b00, prg_din[0], prg_din[1]};

    // --- second batch, the $8000-$FFFF single-latch boards ------------------
    // One `if` per board so the bit slice reads like the page's diagram; the
    // mapper comparators are quasi-static, so this collapses into one write
    // enable plus one data mux per register.
    if (prg_ain[15]) begin
      // 93 [.PPP ...E]: PRG and the CHR-RAM write enable in the same byte.
      if (m93)  begin prg_bank <= {2'b00, prg_din[6:4]}; wren <= prg_din[0]; end
      // 89 [CPPP MCCC]: the CHR bank is SPLIT (its top bit is d[7], three bits
      // away from the other three) and the one-screen page is d[3].
      if (m89)  begin prg_bank  <= {2'b00, prg_din[6:4]};
                      chr_bank0 <= {prg_din[7], prg_din[2:0]};
                      mirr      <= prg_din[3]; end
      // 94 xxxPPPxx: UxROM shifted two bits left.
      if (m94)  prg_bank <= {2'b00, prg_din[4:2]};
      // 180 xxxxxPPP: the switchable half is the HIGH one (see prgout16).
      if (m180) prg_bank <= {2'b00, prg_din[2:0]};
      // 70 [PPPP CCCC] and 152 [MPPP CCCC] -- the same 74161 latch, 152
      // spending the top bit on a one-screen page instead of PRG A17.
      if (m70)  begin prg_bank <= {1'b0, prg_din[7:4]}; chr_bank0 <= prg_din[3:0]; end
      if (m152) begin prg_bank <= {2'b00, prg_din[6:4]};
                      chr_bank0 <= prg_din[3:0];
                      mirr      <= prg_din[7]; end
      // 78 / $F8 [CCCC MPPP] -- identical latch on both boards; only the
      // reading of `mirr` differs (see ss_sel below).
      if (m78any) begin prg_bank <= {2'b00, prg_din[2:0]};
                        chr_bank0 <= prg_din[7:4];
                        mirr      <= prg_din[3]; end
      // 97 [M..PPPPP].  DECODE RANGE, documented divergence: Disch's notes on
      // the page head the register "$8000-BFFF", but that is where the FIXED
      // bank -- and therefore every one of this game's bankswitch instructions
      // -- lives, not what the TAM-S1 decodes; the board has no bus conflicts,
      // so nothing else ever writes cartridge space. The whole $8000-$FFFF
      // range is what the golden simulator (bridge-sim/mappers.py) implements,
      // and RTL and golden diverging is the one failure this project cannot
      // absorb (every byte-exact maptap trace would drift). Widening also
      // cannot break a game the narrow decode would run: the two differ only
      // for writes into the SWITCHABLE half, which a bankswitch never targets.
      if (m97) begin prg_bank <= prg_din[4:0]; mirr <= prg_din[7]; end
      // 72 (Jaleco JF-17) [PCxx DDDD]: the bank number does NOT take effect on
      // the write that carries it.  "When a 1 is written after a 0 was
      // previously written, load the bank specified by D" -- the '161 latch
      // clocks a '174 on the RISING EDGE of P (PRG) or C (CHR), so the game
      // writes the value with the bit set and then clears it to re-arm.  The
      // two previous-state bits are the whole mapper.
      if (m72) begin
        if (prg_din[7] && !lat72[1]) prg_bank  <= {1'b0, prg_din[3:0]};
        if (prg_din[6] && !lat72[0]) chr_bank0 <= prg_din[3:0];
        lat72 <= prg_din[7:6];
      end
      // 185 (CNROM with CHR disable) [..DC ..BA]: on mapper 3 the A/B bits are
      // CHR A13/A14; on 185 the ROM is a Sharp LH2367 whose pins 26/27 became
      // programmable chip selects, so the same two bits decide whether the
      // chip answers at all.  The bank is kept for fidelity -- every 185 board
      // mounts exactly 8 KiB of CHR, so chr_mask folds it to zero anyway.
      if (m185) chr_bank0 <= {2'b00, prg_din[1:0]};
    end

    // --- second batch, the boards whose port replaced the PRG-RAM window ----
    // 184 [.1HH .LLL] at $6000-$7FFF: two 4K CHR banks, the high one hardwired
    // into banks 4-7 by the diagram's constant '1'.
    if (m184 && prg_ain[15:13] == 3'b011) begin
      chr_bank0 <= {1'b0, prg_din[2:0]};
      chr_bank1 <= {1'b0, 1'b1, prg_din[5:4]};
    end
    // 86 [.CPP ..CC] at $6000-$6FFF ONLY ($7000-$7FFF is the uPD7756C speech
    // port).  The board also aliases both ports at $E000-$FFFF because /ROMSEL
    // is high while M2 is low; not reproduced -- it is a wiring flaw of the
    // JF-13, no software depends on it, and honoring it would make every ROM
    // write in the top 8K a bankswitch.
    if (m86 && prg_ain[15:12] == 4'h6) begin
      prg_bank  <= {3'b000, prg_din[5:4]};
      chr_bank0 <= {1'b0, prg_din[6], prg_din[1:0]};
    end
    // 140 [..PP CCCC] at $6000-$7FFF.
    if (m140 && prg_ain[15:13] == 3'b011) begin
      prg_bank  <= {3'b000, prg_din[5:4]};
      chr_bank0 <= prg_din[3:0];
    end
    // 34 -- BNROM banks on any $8000-$FFFF write; NINA-001 has three byte-wide
    // registers at the top of the PRG-RAM window (which those writes ALSO
    // reach, because prg_is_ram below covers $6000-$7FFF for NINA).
    if (m34) begin
      if (!nina) begin
        if (prg_ain[15]) prg_bank <= {3'b000, prg_din[1:0]};
      end else begin
        if (prg_ain == 16'h7ffd) prg_bank  <= {3'b000, prg_din[1:0]};
        if (prg_ain == 16'h7ffe) chr_bank0 <= prg_din[3:0];
        if (prg_ain == 16'h7fff) chr_bank1 <= prg_din[3:0];
      end
    end
    // 71 / 232 -- Camerica.
    if ((m71 || m232) && prg_ain[15]) begin
      // $8000-$BFFF: outer bank (232 only).
      if (m232 && !prg_ain[14]) prg_bank[3:2] <= prg_din[4:3];
      // FIRE HAWK ($9000-$9FFF, mapper 71 only). Upstream ties this to the
      // header instead -- `vram_a10 = flags[14] ? chr_ain[10] : ciram_select`
      // (fpganes src/mmu.v:1352) -- which turns EVERY header-H mapper-71 cart
      // into single-screen; Battle Kid 2 is 71/H and would break. There is no
      // submapper bit to tell them apart (mapper_flags is full), so the board
      // variant is inferred from behaviour: only the BF9097 board (Fire Hawk)
      // ever writes its mirroring register, and it does so at $9000 (14+ sites,
      // all `STA $9000`) -- so the FIRST such write arms the latch, and until
      // then the header rules.  The decode is $9000-$9FFF, NOT the wider
      // $8000-$9FFF upstream uses: several BF9093 titles (Micro Machines, Stunt
      // Kids, Ultimate Stuntman, Big Nose the Caveman, Fantastic Adventures of
      // Dizzy) carry a leftover Quattro outer-bank `STA $8000` in their reset
      // path, inert on the real board but fatal to a latch that listens there
      // (measured: Micro Machines collapsed to single-screen A at boot).  This
      // is the same (addr & $F000) == $9000 decode FCEUX/Mesen use for the
      // BF9097 register.  Mapper 232 writes $8000-$BFFF for its OUTER bank and
      // must never arm it.
      if (m71 && prg_ain[14:12] == 3'b001) begin
        mirr    <= prg_din[4];
        ss_mode <= 1'b1;
      end
      // $C000-$FFFF: bank select. On 232 the outer pair is kept.
      if (prg_ain[14]) prg_bank <= {1'b0, m232 ? prg_bank[3:2] : prg_din[3:2], prg_din[1:0]};
    end
  end

  // --- MAPPER 185: which chip-select value enables the CHR-ROM ---------------
  // The submapper is what really answers this (4/5/6/7 = CS 0/1/2/3), and
  // `mapper_flags` has no room for one.  A value-based guess is NOT available
  // either: the page's own table has Bird Week and B-Wings requiring CS=3 to
  // ENABLE while Spy vs Spy requires CS=3 to DISABLE, and Seicross requiring
  // CS=0 to enable while four other titles require CS=0 to disable.  No
  // function of the latch can satisfy all of them.
  //
  // So this implements the page's OWN fallback for submapper 0, which is what
  // every iNES 1.0 image is: "Disable CHR-ROM for the first two reads from
  // $2007 after a reset and then enable it will work with all known games."
  // The two reads are counted as pattern-table reads (chr_ain[13] low) -- the
  // check runs in the game's init with rendering off, so the PPU issues no
  // fetches of its own, and nametable reads are excluded so they cannot burn a
  // count.  Edge-detected under `ppu_ce`, NOT under `ce`: `ce` here is the CART
  // (CPU) cycle and PPU reads are not on it, but `main.sdc` states as its
  // foundation that EVERY sequential element inside the NES hierarchy is gated
  // by a clock enable (that is what makes the blanket
  // `set_multicycle_path -setup 4` on `*|NES:core|*` sound), so a free-running
  // register here would be a constraint lie: `chr_read` is the PPU's
  // combinational `vram_r` (ppu.v), and a capture on every CLK2 would be given
  // 4x its real budget by the STA.  `ppu_ce` is one PPU dot, which is exactly
  // the rate this edge detector needs.
  //
  // What the disabled chip returns: the page says "reading from the pattern
  // tables returns open bus.  Theoretically this should return the LSB of the
  // address read, but real-world behavior varies, and the earlier revision of
  // Mighty Bomb Jack in fact relies on open bus at PPU $0000 being something
  // other than $00".  $FF satisfies every "must differ from" cell in that
  // table, including Mighty Bomb Jack's.
  always @(posedge clk) if (reset) begin
    rd185      <= 2'd0;
    chr_read_q <= 1'b0;
  end else if (ppu_ce) begin
    chr_read_q <= chr_read;
    if (m185 && chr_read && !chr_read_q && !chr_ain[13] && rd185 != 2'd2)
      rd185 <= rd185 + 2'd1;
  end
  assign chr_dis = m185 && (rd185 != 2'd2) && !chr_ain[13];

  // --- PRG -----------------------------------------------------------------
  // Three 16KB layouts and one 32KB one:
  //   "low"  (the usual UxROM shape, 71/93/89/94/70/152/78) switchable at
  //          $8000, LAST bank fixed at $C000;
  //   232    switchable at $8000, the outer bank's top 16KB fixed at $C000;
  //   "high" switchable at $C000 with $8000 fixed -- to the LAST bank on 97 and
  //          to the FIRST on 180, which is the whole difference between them
  //          (74HC08 instead of 74HC32).
  // "Last bank" is written as all-ones and MultiMapper's prg_mask folds it onto
  // the real last bank of the image, which is why no size ever appears here.
  wire hi_sw = m97 | m180;
  reg [4:0] prgout16;
  always @* begin
    if (m232)       prgout16 = prg_ain[14] ? {1'b0, prg_bank[3:2], 2'b11} : prg_bank;
    else if (hi_sw) prgout16 = prg_ain[14] ? prg_bank : (m180 ? 5'b00000 : 5'b11111);
    else            prgout16 = prg_ain[14] ? 5'b11111 : prg_bank;
  end
  // 72 is the ordinary UxROM shape (switchable at $8000, last fixed at $C000);
  // 185 is fixed 32 KiB, so it stays out of this list exactly like mapper 87.
  wire is16k = m71 | m232 | m93 | m89 | m94 | m97 | m180 | m70 | m152 | m78any | m72;
  // NINA-001 is the only member with PRG-RAM; same CART-RAM window MMC1 /
  // Mapper28 / MMC3 use (PSRAM 0x3C0000), which is also what upstream
  // Mapper34 already did.
  // 184, 86 and 140 map their PORT there instead, so they must NOT get RAM.
  wire prg_is_ram = nina && (prg_ain[15:13] == 3'b011);
  wire [21:0] prg_ram  = {9'b11_1100_000, prg_ain[12:0]};
  wire [21:0] prg_rom  = is16k ? {3'b000,  prgout16,      prg_ain[13:0]}
                               : {4'b0000, prg_bank[2:0], prg_ain[14:0]};
  assign prg_aout  = prg_is_ram ? prg_ram : prg_rom;
  assign prg_allow = (prg_ain[15] && !prg_write) || prg_is_ram;

  // --- CHR -----------------------------------------------------------------
  // NINA-001 (and, by way of the chr_bank1=1 reset, BNROM) is the 4KB-pair
  // form; every other member is one 8KB window. 71/232 have fixed 8KB CHR-RAM
  // and never write chr_bank0, so the 8KB form degenerates to the linear
  // mapping for them at zero cost.
  wire is4k = m34 | m184;
  assign chr_aout  = is4k ? {6'b10_0000, (chr_ain[12] ? chr_bank1 : chr_bank0), chr_ain[11:0]}
                          : {5'b10_000,  chr_bank0, chr_ain[12:0]};
  // Mapper 93 is the only member that can switch its CHR-RAM off (d[0]).
  assign chr_allow = flags[15] & (~m93 | wren);
  assign vram_ce   = chr_ain[13];

  // --- Mirroring ------------------------------------------------------------
  // Two readings of the one `mirr` bit, and the header for everyone else:
  //   ss_sel  the bit is a single-screen PAGE (89, 152, Cosmo Carrier, and 71
  //           once the Fire Hawk latch is armed);
  //   mdyn    the bit is a V/H select, 1 = vertical on all three (113's
  //           $4100-$5FFF d[7], Holy Diver's d[3], 97's d[7] "%1 = Vert").
  wire ss_sel = m89 | m152 | mF8 | (m71 & ss_mode);
  wire mdyn   = m113 | m78 | m97;
  wire mirr_v = mdyn ? mirr : flags[14];
  assign vram_a10 = ss_sel ? mirr : (mirr_v ? chr_ain[10] : chr_ain[11]);

  // sd2snes bridge taps.
  assign snap_chr0   = chr_bank0;
  assign snap_chr1   = chr_bank1;
  // {1,0} = vertical, {1,1} = horizontal, {0,page} = single screen -- the same
  // encoding MMC1/Mapper28/MMC3 export and ntarr_of() decodes.
  assign snap_mirror = ss_sel ? {1'b0, mirr} : {1'b1, ~mirr_v};
endmodule

// Mapper69 -- Sunsoft FME-7 (mapper 69).
// Ported from strigeus/fpganes `src/mmu.v:1221-1310`. The register model, the
// $8000/$A000 command-then-parameter protocol, the 8x1KB CHR window vector, the
// 4x8KB PRG window vector, the four mirroring modes and the 16-bit IRQ
// countdown are all VERBATIM upstream. Deviations, both forced by this core:
//
//   1. PRG-RAM ADDRESS. Upstream forms `prg_aout = {1'b0, ram_cs, 2'b00,
//      prgout, prg_ain[12:0]}`, i.e. it puts the cartridge RAM at PSRAM
//      0x100000+. That region is PRG-ROM in this core's map (nes_wrap.v: PRG
//      0x000000-0x0FFFFF, CHR 0x200000, CIRAM 0x300000, CPU-RAM 0x380000,
//      CART-RAM 0x3C0000-0x3C1FFF), so the RAM window is remapped to the
//      CART-RAM one that MMC1/Mapper28/MMC3/MapperDiscrete already use.
//      `ram_enable`/`ram_select` keep their upstream meaning -- unlike MMC3,
//      this mapper's enable bit is NOT forced on: the FME-7 boot code programs
//      $8000=8/$A000 before touching $6000, and games with no RAM chip
//      (Gimmick! aside) must keep seeing ROM there.
//   2. `prgout` DEFAULT. Upstream leaves it 5'bxxxxx for prg_ain < $6000,
//      which is X in simulation. $0000-$1FFF is overridden by MultiMapper's
//      CPU-RAM tail; $4018-$5FFF IS reachable on this core (nes.v asserts
//      prg_read there, apu_cs only covers $4000-$4017), and there both upstream
//      and this port answer with `prg_allow=1` (reads) -- upstream with an X
//      bank, here with PRG bank 0.  Real FME-7 boards leave that range open
//      bus; no known title reads it.  A defined 0 keeps the testbenches (and
//      the bridge taps that watch this module) free of X propagation.  Same
//      class of change as Mapper28's a53chr reset.
//
// The CHR bank registers are eight NAMED registers rather than upstream's
// `reg [7:0] chr_bank[0:7]`: the tap below needs all eight at once, and an
// array with an asynchronous variable-index read is exactly the shape that
// tempts a tool into a LUT-RAM it cannot reset. MMC3 in this file already
// spells its six banks out for the same reason.
//
// TAPS: like MMC3, the FME-7's CHR view is a VECTOR OF EIGHT 1KB WINDOWS, which
// the slot0/slot1 pair of the legacy tap cannot represent -- so it publishes
// snap_chr_win (CMD_CHR_STATE8 $14) and MultiMapper holds the legacy pair at
// the same constant sentinel it uses for mapper 4. Mirroring is dynamic (R12)
// and the header bit is meaningless for it, so snap_mirror is wired too.
module Mapper69(input clk, input ce, input reset,
                input [31:0] flags,
                input [15:0] prg_ain, output [21:0] prg_aout,
                input prg_read, prg_write,                   // Read / write signals
                input [7:0] prg_din,
                output prg_allow,                            // Enable access to memory for the specified operation.
                input [13:0] chr_ain, output [21:0] chr_aout,
                output chr_allow,                            // Allow write
                output reg vram_a10,                         // Value for A10 address line
                output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
                output reg irq,
                // sd2snes video-bridge NT-arrangement tap: R12 recoded into the
                // (00=1scr-lo, 01=1scr-hi, 10=V, 11=H) form ntarr_of() decodes.
                output [1:0] snap_mirror,
                // sd2snes video-bridge CHR WINDOW VECTOR tap (protocol v2.5),
                // window k in [k*8 +: 8]. UNMASKED here; MultiMapper applies the
                // size mask where every other tap does.
                output [63:0] snap_chr_win);
  reg [7:0] chr_bank0, chr_bank1, chr_bank2, chr_bank3;
  reg [7:0] chr_bank4, chr_bank5, chr_bank6, chr_bank7;
  reg [4:0] prg_bank0, prg_bank1, prg_bank2, prg_bank3;
  reg [1:0] mirroring;
  reg irq_countdown, irq_trigger;
  reg [15:0] irq_counter;
  reg [3:0] addr;
  reg ram_enable, ram_select;
  wire [16:0] new_irq_counter = irq_counter - {15'b0, irq_countdown};

  always @(posedge clk) if (reset) begin
    chr_bank0 <= 0; chr_bank1 <= 0; chr_bank2 <= 0; chr_bank3 <= 0;
    chr_bank4 <= 0; chr_bank5 <= 0; chr_bank6 <= 0; chr_bank7 <= 0;
    prg_bank0 <= 0; prg_bank1 <= 0; prg_bank2 <= 0; prg_bank3 <= 0;
    mirroring <= 0;
    irq_countdown <= 0;
    irq_trigger <= 0;
    irq_counter <= 0;
    addr <= 0;
    ram_enable <= 0;
    ram_select <= 0;
    irq <= 0;
  end else if (ce) begin
    irq_counter <= new_irq_counter[15:0];
    if (irq_trigger && new_irq_counter[16]) irq <= 1;
    if (!irq_trigger) irq <= 0;

    if (prg_ain[15] & prg_write) begin
      case (prg_ain[14:13])
      2'd0: addr <= prg_din[3:0];                             // $8000-$9FFF command
      2'd1: begin                                             // $A000-$BFFF parameter
          case (addr)
          4'd0:  chr_bank0 <= prg_din;
          4'd1:  chr_bank1 <= prg_din;
          4'd2:  chr_bank2 <= prg_din;
          4'd3:  chr_bank3 <= prg_din;
          4'd4:  chr_bank4 <= prg_din;
          4'd5:  chr_bank5 <= prg_din;
          4'd6:  chr_bank6 <= prg_din;
          4'd7:  chr_bank7 <= prg_din;
          4'd8:  prg_bank0 <= prg_din[4:0];
          4'd9:  prg_bank1 <= prg_din[4:0];
          4'd10: prg_bank2 <= prg_din[4:0];
          4'd11: prg_bank3 <= prg_din[4:0];
          4'd12: mirroring <= prg_din[1:0];
          4'd13: {irq_countdown, irq_trigger} <= {prg_din[7], prg_din[0]};
          4'd14: irq_counter[7:0]  <= prg_din;
          4'd15: irq_counter[15:8] <= prg_din;
          endcase
          if (addr == 4'd8) {ram_enable, ram_select} <= prg_din[7:6];
        end
      endcase
    end
  end

  always @* begin
    casez(mirroring[1:0])
    2'b00   :   vram_a10 = chr_ain[10];    // vertical
    2'b01   :   vram_a10 = chr_ain[11];    // horizontal
    2'b1?   :   vram_a10 = mirroring[0];   // one screen, lower / upper
    endcase
  end

  reg [4:0] prgout;
  reg [7:0] chrout;
  always @* begin
    casez(prg_ain[15:13])
    3'b011:  prgout = prg_bank0;
    3'b100:  prgout = prg_bank1;
    3'b101:  prgout = prg_bank2;
    3'b110:  prgout = prg_bank3;
    3'b111:  prgout = 5'b11111;
    default: prgout = 5'b00000;   // see DEVIATION 2 in the module header
    endcase
    case (chr_ain[12:10])
    3'd0: chrout = chr_bank0;
    3'd1: chrout = chr_bank1;
    3'd2: chrout = chr_bank2;
    3'd3: chrout = chr_bank3;
    3'd4: chrout = chr_bank4;
    3'd5: chrout = chr_bank5;
    3'd6: chrout = chr_bank6;
    3'd7: chrout = chr_bank7;
    endcase
  end

  wire ram_cs = (prg_ain[15] == 0 && ram_select);
  wire [21:0] prg_ram = {9'b11_1100_000, prg_ain[12:0]};   // DEVIATION 1
  assign prg_aout  = ram_cs ? prg_ram : {4'b00_00, prgout[4:0], prg_ain[12:0]};
  assign prg_allow = ram_cs ? ram_enable : !prg_write;
  assign chr_allow = flags[15];
  assign chr_aout  = {4'b10_00, chrout, chr_ain[9:0]};
  assign vram_ce   = chr_ain[13];

  // sd2snes bridge taps. R12: 0=V, 1=H, 2=1-screen A, 3=1-screen B maps to the
  // shared raw code as {~mirroring[1], mirroring[0]} (0->10 V, 1->11 H,
  // 2->00 1A, 3->01 1B) -- exactly what vram_a10 decodes above.
  assign snap_mirror  = {~mirroring[1], mirroring[0]};
  assign snap_chr_win = {chr_bank7, chr_bank6, chr_bank5, chr_bank4,
                         chr_bank3, chr_bank2, chr_bank1, chr_bank0};
endmodule

// VRC24 -- Konami VRC2 and VRC4 (iNES mappers 21, 22, 23, 25).
//
// NOT an upstream fpganes module: fpganes never had any VRC.  Written from
// nesdev.org "VRC2 and VRC4" and "VRC IRQ" (raw wikitext, 5 Sep 2026).
//
// ONE module for all four iNES numbers.  The chips are identical; what differs
// between boards is WHICH TWO CPU ADDRESS LINES feed the mapper's two register
// select inputs, and the wiki's table is what this decode implements:
//
//   nickname  PCB      reg A0  reg A1   iNES
//   VRC2a     351618   A1      A0       22
//   VRC2b     many     A0      A1       23
//   VRC2c     351948   A1      A0       25
//   VRC4a     352398   A1      A2       21
//   VRC4b     351406   A1      A0       25
//   VRC4c     352889   A6      A7       21
//   VRC4d     352400   A3      A2       25
//   VRC4e     352396   A2      A3       23
//   VRC4f     -        A0      A1       23
//
// "iNES mappers 21, 23 and 25 each implement TWO address mappings at the same
// time ... Because the address pairings do not overlap, and the games appear to
// use these registers in a well behaved manner, it is presumed sufficient for
// compatibility in most cases."  So each number ORs its two wirings:
//
//   21: reg A0 = A1|A6, reg A1 = A2|A7      (VRC4a + VRC4c)
//   22: reg A0 = A1,    reg A1 = A0         (VRC2a alone)
//   23: reg A0 = A0|A2, reg A1 = A1|A3      (VRC2b/VRC4f + VRC4e)
//   25: reg A0 = A1|A3, reg A1 = A0|A2      (VRC2c/VRC4b + VRC4d)
//
// NOTE for whoever re-reads the contract: NES-MAPPERS-LOTE3.md 0 writes mapper
// 25 as "A1|A0 and A0|A1", which is the same term twice and cannot be right.
// The wiki table above is what is implemented (25 = A1|A3 and A0|A2).
//
// VRC2 vs VRC4: only mapper 22 is treated as a VRC2 here, because the wiki says
// "the VRC4 is always presumed for these three mappers" (21/23/25).  The VRC2
// differences that survive that choice are the ones mapper 22 needs: no IRQ, no
// PRG swap mode, CHR bank value right-shifted by one, and mirroring with bit 1
// ignored.  Consequence, documented rather than fixed: a VRC2b/VRC2c game that
// writes a value with bit 1 set to $9000 gets one-screen where the real board
// gives horizontal.  The wiki names exactly one instance of that (Wai Wai World
// writing $FF once) and every VRC4-presuming emulator has it too.
//
// DEVIATIONS from the page, both deliberate:
//   1. CHR REGISTERS ARE 8 BITS, not the VRC4's 9.  The high half register is
//      "...H HHHH" (5 bits) on VRC4 and "only 4 high bits" on VRC2; the sd2snes
//      video bridge carries ONE BYTE per 1KB window (chr_snap_win), so a ninth
//      bit could not be published even if it were latched, and the renderer
//      would fetch from the wrong half of PSRAM.  CHR is therefore capped at
//      256 KiB -- which is the VRC2's own limit, and covers every licensed
//      VRC4 title in the compatibility sheet.  The loader guard in nes.c is
//      what makes a bigger image NOIMPL instead of silently aliasing.
//   2. PRG-RAM AT $6000-$7FFF IS ALWAYS LIVE.  Same class of deviation as the
//      one at the top of module MMC3 and the end of module Mapper28: the
//      VRC4's 'W' bit ($9002 bit 0) is ignored and the CART-RAM window answers
//      unconditionally.  This is not just convenience -- it is what makes the
//      VRC2 1-bit latch at $6000-$6FFF work.  Contra (J) and Ganbare Goemon 2
//      write that latch and read it back, and the wiki is explicit that
//      returning open bus OR zero locks them up ("Emulators that use the same
//      VRC4 core (and its PRG RAM) for VRC2 emulation will have the effect
//      simulated for them").  A byte of RAM returns the value that was written,
//      which is a superset of the one bit those games look at.
//
// The mirroring code (0=V, 1=H, 2=1ScA, 3=1ScB) is bit-for-bit the FME-7's R12,
// so vram_a10 and the snap_mirror recoding are copied from module Mapper69.
module VRC24(input clk, input ce, input reset,
             input [31:0] flags,
             input [15:0] prg_ain, output [21:0] prg_aout,
             input prg_read, prg_write,                   // Read / write signals
             input [7:0] prg_din,
             output prg_allow,                            // Enable access to memory for the specified operation.
             input [13:0] chr_ain, output [21:0] chr_aout,
             output chr_allow,                            // Allow write
             output reg vram_a10,                         // Value for A10 address line
             output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
             output reg irq,
             // sd2snes video-bridge NT-arrangement tap, same recoding Mapper69
             // uses (the register field is identical).
             output [1:0] snap_mirror,
             // sd2snes video-bridge CHR WINDOW VECTOR tap (protocol v2.5),
             // window k in [k*8 +: 8].  UNMASKED here; MultiMapper applies the
             // size mask where every other tap does.
             output [63:0] snap_chr_win);
  reg [7:0] chr_bank0, chr_bank1, chr_bank2, chr_bank3;
  reg [7:0] chr_bank4, chr_bank5, chr_bank6, chr_bank7;
  reg [4:0] prg_bank0, prg_bank1;
  reg [1:0] mirroring;
  reg       prg_swap;
  reg [7:0] irq_latch, irq_counter;
  reg [8:0] prescaler;
  reg       irq_enable, irq_enable_ack, irq_mode;

  wire [7:0] mnum = flags[7:0];
  wire m21 = (mnum == 8'd21);
  wire m22 = (mnum == 8'd22);
  wire m23 = (mnum == 8'd23);
  wire m25 = (mnum == 8'd25);
  // Mapper 22 is the only VRC2 (see the module header).
  wire vrc2 = m22;

  // The two register-select lines, per the table in the header.
  wire sel0 = m21 ? (prg_ain[1] | prg_ain[6])
            : m22 ?  prg_ain[1]
            : m23 ? (prg_ain[0] | prg_ain[2])
            :       (prg_ain[1] | prg_ain[3]);   // 25
  wire sel1 = m21 ? (prg_ain[2] | prg_ain[7])
            : m22 ?  prg_ain[0]
            : m23 ? (prg_ain[1] | prg_ain[3])
            :       (prg_ain[0] | prg_ain[2]);   // 25
  wire [1:0] ridx = {sel1, sel0};

  wire       wr   = ce & prg_write & prg_ain[15];
  wire [2:0] blk  = prg_ain[14:12];
  // $B000/$C000/$D000/$E000 hold CHR selects 0-1/2-3/4-5/6-7; blk[1:0]+1 turns
  // 011,100,101,110 into 0,1,2,3 (the "CHR Select 2...7" table on the page).
  wire       chr_blk = (blk == 3'b011) | (blk == 3'b100) |
                       (blk == 3'b101) | (blk == 3'b110);
  wire [2:0] cidx = {blk[1:0] + 2'b01, ridx[1]};
  wire       cwr  = wr & chr_blk;
  wire       chi  = ridx[0];                 // 1 = the "high" half register
  wire [3:0] cdat = prg_din[3:0];

  // VRC IRQ (nesdev "VRC IRQ").  Scanline mode divides CPU cycles by
  // 114/114/113 through a prescaler that starts at 341 and loses 3 per cycle;
  // cycle mode bypasses it.  The counter counts UP and reloads from the latch
  // when it rolls over from $FF.
  wire       irq_tick_presc = (prescaler <= 9'd3);
  wire       irq_tick = irq_enable & (irq_mode | irq_tick_presc);

  always @(posedge clk) if (reset) begin
    chr_bank0 <= 0; chr_bank1 <= 0; chr_bank2 <= 0; chr_bank3 <= 0;
    chr_bank4 <= 0; chr_bank5 <= 0; chr_bank6 <= 0; chr_bank7 <= 0;
    prg_bank0 <= 0; prg_bank1 <= 0;
    mirroring <= 0;
    prg_swap  <= 0;
    irq_latch <= 0; irq_counter <= 0;
    prescaler <= 9'd341;
    irq_enable <= 0; irq_enable_ack <= 0; irq_mode <= 0;
    irq <= 0;
  end else if (ce) begin
    // --- IRQ, clocked FIRST so a write in the same cart cycle overrides it ---
    if (irq_enable) begin
      if (!irq_mode)
        prescaler <= irq_tick_presc ? (prescaler + 9'd338) : (prescaler - 9'd3);
      if (irq_tick) begin
        if (irq_counter == 8'hff) begin
          irq_counter <= irq_latch;
          irq <= 1'b1;
        end else
          irq_counter <= irq_counter + 8'd1;
      end
    end

    if (wr) begin
      case (blk)
      3'b000: prg_bank0 <= prg_din[4:0];              // $8000-$8003 PRG select 0
      3'b001: begin                                   // $9000-$9003
        // VRC4: index 0 and index 1 are the nametable arrangement, index 2 is
        // "PRG Swap Mode/WRAM control ($9002)" and index 3 is the "VRC4
        // External Select ($9003)", which drives an off-chip signal and
        // changes nothing inside the mapper ("No Konami games made use of
        // this").  So a $9003 write must NOT move the swap mode.
        // VRC2: the heading is literal -- "Nametable arrangement ($9000,
        // $9001, $9002, $9003)" -- because there is no swap-mode register to
        // share the block with, so all four indices write it and bit 1 is
        // ignored.
        if (vrc2)                        mirroring <= {1'b0, prg_din[0]};
        else if (!ridx[1])               mirroring <= prg_din[1:0];
        else if (!ridx[0])               prg_swap  <= prg_din[1];
      end
      3'b010: prg_bank1 <= prg_din[4:0];              // $A000-$A003 PRG select 1
      3'b111: if (!vrc2) begin                        // $F000-$F003 IRQ
        case (ridx)
        2'd0: irq_latch[3:0] <= prg_din[3:0];
        2'd1: irq_latch[7:4] <= prg_din[3:0];
        2'd2: begin
          // "Any write to this register will acknowledge the pending IRQ and
          // reset the prescaler.  If this register is written to with 'E' set,
          // the IRQ counter is reloaded with the latch value."
          {irq_mode, irq_enable, irq_enable_ack} <= prg_din[2:0];
          irq       <= 1'b0;
          prescaler <= 9'd341;
          if (prg_din[1]) irq_counter <= irq_latch;
        end
        2'd3: begin
          // "Any write to this register will acknowledge the pending IRQ.  In
          // addition, the 'A' control bit is copied to the 'E' control bit."
          irq        <= 1'b0;
          irq_enable <= irq_enable_ack;
        end
        endcase
      end
      default: ;
      endcase
      if (cwr) begin
        // Each 1KB bank is two nibble-wide registers (low 4 / high 4).
        if (cidx == 3'd0) begin if (chi) chr_bank0[7:4] <= cdat; else chr_bank0[3:0] <= cdat; end
        if (cidx == 3'd1) begin if (chi) chr_bank1[7:4] <= cdat; else chr_bank1[3:0] <= cdat; end
        if (cidx == 3'd2) begin if (chi) chr_bank2[7:4] <= cdat; else chr_bank2[3:0] <= cdat; end
        if (cidx == 3'd3) begin if (chi) chr_bank3[7:4] <= cdat; else chr_bank3[3:0] <= cdat; end
        if (cidx == 3'd4) begin if (chi) chr_bank4[7:4] <= cdat; else chr_bank4[3:0] <= cdat; end
        if (cidx == 3'd5) begin if (chi) chr_bank5[7:4] <= cdat; else chr_bank5[3:0] <= cdat; end
        if (cidx == 3'd6) begin if (chi) chr_bank6[7:4] <= cdat; else chr_bank6[3:0] <= cdat; end
        if (cidx == 3'd7) begin if (chi) chr_bank7[7:4] <= cdat; else chr_bank7[3:0] <= cdat; end
      end
    end
  end

  // --- PRG ------------------------------------------------------------------
  // Swap mode 0: $8000 = reg0, $C000 = second-to-last.  Mode 1 exchanges them.
  // $A000 = reg1 and $E000 = last in both.  The VRC2's fixed 16KB at $C000 is
  // exactly mode 0 (second-to-last then last), so `prg_swap` simply never
  // leaves 0 there.  "Last bank" is all-ones; MultiMapper's prg_mask folds it.
  reg [4:0] prgsel;
  always @* begin
    case (prg_ain[14:13])
    2'b00: prgsel = prg_swap ? 5'b11110 : prg_bank0;   // $8000-$9FFF
    2'b01: prgsel = prg_bank1;                         // $A000-$BFFF
    2'b10: prgsel = prg_swap ? prg_bank0 : 5'b11110;   // $C000-$DFFF
    2'b11: prgsel = 5'b11111;                          // $E000-$FFFF
    endcase
  end

  // --- CHR ------------------------------------------------------------------
  // SINGLE IMPLEMENTATION, same discipline as MMC3's chr_win_of: the address
  // path and the window-vector tap call the SAME function, so the tap cannot
  // drift from what the PPU fetches.
  function [7:0] chr_win_of;
    input [2:0] k;
    reg [7:0] v;
    begin
      case (k)
      3'd0: v = chr_bank0;  3'd1: v = chr_bank1;
      3'd2: v = chr_bank2;  3'd3: v = chr_bank3;
      3'd4: v = chr_bank4;  3'd5: v = chr_bank5;
      3'd6: v = chr_bank6;  3'd7: v = chr_bank7;
      endcase
      // "On VRC2a (mapper 22), the low bit is ignored (right shift value by 1)."
      chr_win_of = vrc2 ? {1'b0, v[7:1]} : v;
    end
  endfunction

  reg [7:0]  chrsel;
  reg [63:0] snap_win_r;
  // EXPLICIT sensitivity list for the same tool-portability reason module MMC3
  // spells its own out: the right-hand sides are function calls.
  always @(chr_ain or vrc2 or chr_bank0 or chr_bank1 or chr_bank2 or chr_bank3
           or chr_bank4 or chr_bank5 or chr_bank6 or chr_bank7) begin
    chrsel     = chr_win_of(chr_ain[12:10]);
    snap_win_r = {chr_win_of(3'd7), chr_win_of(3'd6),
                  chr_win_of(3'd5), chr_win_of(3'd4),
                  chr_win_of(3'd3), chr_win_of(3'd2),
                  chr_win_of(3'd1), chr_win_of(3'd0)};
  end

  // --- Buses ----------------------------------------------------------------
  wire        prg_is_ram = (prg_ain[15:13] == 3'b011);   // DEVIATION 2
  wire [21:0] prg_ram    = {9'b11_1100_000, prg_ain[12:0]};
  assign prg_aout  = prg_is_ram ? prg_ram
                                : {4'b00_00, prgsel, prg_ain[12:0]};
  assign prg_allow = (prg_ain[15] && !prg_write) || prg_is_ram;
  assign chr_allow = flags[15];
  assign chr_aout  = {4'b10_00, chrsel, chr_ain[9:0]};
  assign vram_ce   = chr_ain[13];

  always @* begin
    casez (mirroring)
    2'b00:   vram_a10 = chr_ain[10];    // vertical
    2'b01:   vram_a10 = chr_ain[11];    // horizontal
    2'b1?:   vram_a10 = mirroring[0];   // one screen, lower / upper
    endcase
  end

  // Same field, same recoding as module Mapper69's R12.
  assign snap_mirror  = {~mirroring[1], mirroring[0]};
  assign snap_chr_win = snap_win_r;
endmodule

// Mapper1K -- the "eight 1 KiB CHR windows + up to three 8 KiB PRG registers"
// family, in ONE module: iNES mappers 18, 19, 32, 65, 75 and 210.
//
//   18   Jaleco SS 88006          nibble-pair registers, 16-bit down IRQ
//   19   Namco 163 / 129          $800-block registers, 15-bit up IRQ, NT select
//   32   Irem G-101               $F000 mask, PRG swap mode           (+ alias $F1)
//   65   Irem H3001               $F007 mask, 16-bit down IRQ
//   75   Konami VRC1              two 4 KiB CHR banks, three PRG banks
//   210  Namco 175 / 340          = 19 without IRQ / NT select / internal RAM
//
// NOT an upstream fpganes module: fpganes has none of these.  Written from the
// nesdev pages INES_Mapper_018, INES_Mapper_019, INES_Mapper_032,
// INES_Mapper_065, VRC1 and INES_Mapper_210 (raw wikitext, 5 Sep 2026).
//
// WHY ONE MODULE.  Measured on this fit: every additional arm of MultiMapper's
// one-hot AND-OR costs ~50 LEs on top of the mapper itself (48 bus bits gain a
// term, plus the window-vector source and the NTARR arm).  These six mappers
// are the SAME MACHINE with six different write decoders -- eight byte-wide
// 1 KiB CHR registers, three 6-bit 8 KiB PRG registers with the last bank
// fixed, a per-quadrant CIRAM page, and a 16-bit CPU-cycle counter -- so fusing
// them pays that ~50 LEs once instead of six times AND shares 124 flip-flops.
// The decoders themselves are quasi-static (`flags[7:0]` comparators), so they
// collapse into one write enable plus one data mux per register, exactly the
// way module MapperDiscrete fuses the discrete boards.
//
// CIRAM PAGE PER QUADRANT is the state this module keeps for mirroring, not a
// two-bit "mode": the Namco 163 really does have four independent nametable
// registers, and every other member's mirroring field is just a canned pattern
// of those four bits (V = 0,1,0,1 / H = 0,0,1,1 / 1ScA = 0,0,0,0 / 1ScB =
// 1,1,1,1).  One 4:1 mux drives vram_a10 for all six, and one decode turns the
// four bits back into the protocol's NTARR code.
//
// NOT SUPPORTED, and why (both are the same wall -- the sd2snes video bridge
// only ever sees CIRAM, NES-MAPPERS-LOTE3.md 0):
//   * Namco 163 NAMETABLES FROM CHR-ROM.  A value BELOW $E0 in one of the four
//     $C000-$DFFF registers selects a 1 KiB page of CHR-ROM as that nametable.
//     Here the low bit is taken as a CIRAM page regardless, so such a title
//     renders the wrong nametable rather than hanging.
//   * Namco 163 CHR-RAM FROM NAMETABLE RAM ($E800 bits 6/7 clear).  Always
//     treated as if both bits were set, i.e. pages $E0-$FF are ordinary
//     CHR-ROM.  All commercial-era titles ship CHR-ROM only.
//   * Namco 163 128-byte INTERNAL RAM (the $4800 port).  It is the expansion
//     audio's register file, and for six battery titles also their save area.
//     There is no expansion audio in this core; a game that saves there sees
//     an uninitialised save and re-initialises it.
//   * The uPD7756C ADPCM port of mapper 18 ($F003) -- no expansion audio.
//
// CHR CEILING, and which tap each member uses.  Five of the six announce a
// WINDOW VECTOR (chr_snap_win / CMD_CHR_STATE8 $14).  Mapper 75 is the
// exception: its granularity is 4 KiB, so it announces the legacy $12 PAIR,
// with FIVE-bit bank ids -- all three VRC1 titles carry 128 KiB of CHR, i.e.
// 32 banks, and the four-bit pair mask NINA-001/Sunsoft-1 use would have hidden
// half of it from the renderer while the address path still reached it.  That
// is the Sunsoft-1 failure of NES-MAPPERS-LOTE2.md 5 seen from the other side,
// and it is why MultiMapper's pair mask is {chr_mask[3:0], 1'b1}.
// Because a WINDOW id is ONE BYTE, every window mapper is capped at 256 KiB of
// CHR; the registers here are sized to match, so nothing can address CHR the
// tap cannot name.  On paper the VRC4 (9-bit CHR registers) and the two 2 KiB
// Taito registers reach 512 KiB -- no licensed title does, and the loader
// guard in nes.c is what keeps a bigger image from aliasing in silence.
//
// DEVIATION shared with modules MMC3/Mapper28/VRC24: the $6000-$7FFF CART-RAM
// window is ALWAYS live.  Mappers 18, 19 and 210/175 all gate their WRAM behind
// an enable register ($9002, $F800, $C000 respectively); handing a well-behaved
// ROM RAM it has not enabled yet is inert, and 32/65/75 have no RAM chip at all
// so nothing there ever reads the window.
module Mapper1K(input clk, input ce, input reset,
                input [31:0] flags,
                input [15:0] prg_ain, output [21:0] prg_aout,
                input prg_read, prg_write,                   // Read / write signals
                input [7:0] prg_din, output reg [7:0] prg_dout,
                output prg_allow,                            // Enable access to memory for the specified operation.
                output has_prg_dout,                         // This mapper answers the read itself
                input [13:0] chr_ain, output [21:0] chr_aout,
                output chr_allow,                            // Allow write
                output reg vram_a10,                         // Value for A10 address line
                output vram_ce,                              // True if the address should be routed to the internal 2kB VRAM.
                output reg irq,
                // sd2snes video-bridge NT-arrangement tap, in the shared raw
                // code (00 = 1ScA, 01 = 1ScB, 10 = V, 11 = H).
                output [1:0] snap_mirror,
                // sd2snes video-bridge CHR WINDOW VECTOR tap (protocol v2.5),
                // window k in [k*8 +: 8].  UNMASKED here; MultiMapper masks.
                // NOT used by mapper 75, which announces the pair below.
                output [63:0] snap_chr_win,
                // sd2snes video-bridge legacy $12 PAIR tap -- mapper 75 only,
                // 5-bit 4 KiB bank ids (UNMASKED; MultiMapper applies the
                // five-bit form of the pair mask).
                output [4:0] snap_pair0,
                output [4:0] snap_pair1,
                // sd2snes video-bridge NT-CODE tap (Fase 3, CMD_PPU_SPLITS
                // 0x16): the FOUR per-quadrant CIRAM pages, bit k = the page
                // of logical nametable k ($2000/$2400/$2800/$2C00).  This is
                // the same nt_page the vram_a10 mux above reads, so the tap
                // and the address path can never diverge.  It exists because
                // snap_mirror collapses those four bits into the 2-bit legacy
                // code, which cannot name the (1,0) of mapper 95 nor the ten
                // patterns only the Namco 163 / 118-with-inversion reach.
                // Appended at the END of the port list on purpose: the
                // instantiation in MultiMapper is POSITIONAL.
                output [3:0] snap_ntpages);
  reg [7:0] chr_bank0, chr_bank1, chr_bank2, chr_bank3;
  reg [7:0] chr_bank4, chr_bank5, chr_bank6, chr_bank7;
  reg [5:0] prg_bank0, prg_bank1, prg_bank2;
  reg [3:0] nt_page;                 // CIRAM A10 per nametable quadrant
  reg       prg_mode;                // 32/65: the $8000 <-> $C000 swap
  reg [15:0] irq_counter, irq_latch;
  reg       irq_enable;
  reg [2:0] irq_width;               // 18 only: the F/E/T size bits

  wire [7:0] mnum = flags[7:0];
  wire m18  = (mnum == 8'd18);
  wire m19  = (mnum == 8'd19);
  wire m32  = (mnum == 8'd32);
  wire mF1  = (mnum == 8'hF1);       // 32 submapper 1 (Major League) -- see ALIAS
  wire m65  = (mnum == 8'd65);
  wire m75  = (mnum == 8'd75);
  wire m210 = (mnum == 8'd210);
  wire m32any = m32 | mF1;
  // ALIAS $F1 (the mechanism NES-MAPPERS-LOTE2.md 1.3 established for $F8 and
  // $F4): Major League is a mapper 32 whose board ties CIRAM A10 to +5V and
  // disables the $9000 register entirely, and NES 2.0 assigns it submapper 1.
  // `mapper_flags` has no room for a submapper, so nes.c translates (32, 1)
  // into the internal byte $F1.  THREE MIRRORS: src/nes.c, this file and
  // bridge-sim/bridge_sim/mappers.py.

  // The Namco pair share one decoder: 210 IS a 163 with the IRQ, the nametable
  // selects and the internal RAM removed.
  wire namco = m19 | m210;
  // 19/210 register blocks are $800 wide; 18 uses A12-A14 + A0-A1; 32/75 use
  // A12-A14 only; 65 uses A12-A14 + A0-A2.
  wire [3:0] nblk = prg_ain[14:11];
  wire [2:0] blk  = prg_ain[14:12];
  wire [1:0] jidx = prg_ain[1:0];
  wire [2:0] iidx = prg_ain[2:0];
  wire       wr   = ce & prg_write & prg_ain[15];

  // --- IRQ ------------------------------------------------------------------
  // One counter, three rules:
  //   19  15-bit UP, +1 per CPU cycle, fires and STOPS at $7FFF
  //   18  16-bit DOWN, and the F/E/T bits stop the borrow at bit 4/8/12 and
  //       assert IRQ there instead ("the high bits are not altered")
  //   65  16-bit DOWN, fires at 0 and STOPS ("does not wrap, isn't reloaded")
  wire [15:0] cnt_dec = irq_counter - 16'd1;
  wire [15:0] cnt_inc = irq_counter + 16'd1;
  // 18's borrow points.  F overrides E overrides T.
  wire w4  = irq_width[2];
  wire w8  = ~irq_width[2] &  irq_width[1];
  wire w12 = ~irq_width[2] & ~irq_width[1] &  irq_width[0];
  wire w16 = (irq_width == 3'd0);
  wire b18 = (w4  & (irq_counter[3:0]  == 4'd0))
           | (w8  & (irq_counter[7:0]  == 8'd0))
           | (w12 & (irq_counter[11:0] == 12'd0))
           | (w16 & (irq_counter       == 16'd0));
  wire n19_top  = (irq_counter[14:0] == 15'h7fff);
  wire m65_last = (irq_counter == 16'd1);

  always @(posedge clk) if (reset) begin
    chr_bank0 <= 0; chr_bank1 <= 0; chr_bank2 <= 0; chr_bank3 <= 0;
    chr_bank4 <= 0; chr_bank5 <= 0; chr_bank6 <= 0; chr_bank7 <= 0;
    // "On powerup, it appears as though PRG regs are inited to specific values:
    // $8000 = $00, $A000 = $01.  Games do rely on this and will crash
    // otherwise." (INES_Mapper_065).  Harmless for the other five, which all
    // program both banks before jumping anywhere.
    prg_bank0 <= 6'd0; prg_bank1 <= 6'd1; prg_bank2 <= 6'd0;
    // POWER-ON MIRRORING.  None of these pages documents a power-on state, and
    // every game programs the register before it renders -- but the video
    // bridge snapshots the arrangement once per frame, so the RTL and the
    // golden simulator have to agree on the pre-write value or the first
    // frames diverge.  The rule, and it is the only one that generalises: the
    // mapper's own mirroring REGISTER resets to zero, decoded by that mapper's
    // own encoding.  Zero is one-screen A on the Namco pair, horizontal on the
    // SS 88006 and vertical on the two Irems and the VRC1.
    // $F1 is the exception on purpose: Major League has no register at all and
    // its board ties CIRAM A10 to +5V, i.e. one-screen B forever.
    nt_page   <= mF1  ? 4'b1111
               : m18  ? 4'b1100
               : (m32 | m65 | m75) ? 4'b1010
               :        4'b0000;
    prg_mode  <= 1'b0;
    irq_counter <= 0; irq_latch <= 0; irq_enable <= 0; irq_width <= 0;
    irq <= 0;
  end else if (ce) begin
    // Counter first, so a register write in the same cart cycle wins.
    if (irq_enable) begin
      if (m19) begin
        if (n19_top) irq <= 1'b1;
        else         irq_counter <= cnt_inc;
      end else if (m65) begin
        if (irq_counter != 16'd0) begin
          irq_counter <= cnt_dec;
          if (m65_last) irq <= 1'b1;
        end
      end else if (m18) begin
        irq_counter[3:0] <= cnt_dec[3:0];
        if (!w4)          irq_counter[7:4]   <= cnt_dec[7:4];
        if (w12 | w16)    irq_counter[11:8]  <= cnt_dec[11:8];
        if (w16)          irq_counter[15:12] <= cnt_dec[15:12];
        if (b18) irq <= 1'b1;
      end
    end

    // --- Namco 163 IRQ registers live BELOW $8000 -------------------------
    // "$5000-$57FF all refers to one register, $5800-$5FFF to another."  These
    // are direct access to the counter, not a reload value, and a write to
    // either acknowledges.
    if (m19 && prg_write && (prg_ain[15:11] == 5'b01010)) begin
      irq_counter[7:0] <= prg_din;
      irq <= 1'b0;
    end
    if (m19 && prg_write && (prg_ain[15:11] == 5'b01011)) begin
      irq_counter[14:8] <= prg_din[6:0];
      irq_enable        <= prg_din[7];
      irq <= 1'b0;
    end

    if (wr) begin
      // --- 19 / 210 -------------------------------------------------------
      // $8000-$BFFF: eight 1 KiB CHR selects, one per $800 block.
      // $C000-$DFFF: the four nametable selects (163 only; on the 175/340 the
      //              $C000 block is the PRG-RAM enable, which this core
      //              ignores -- see the DEVIATION note in the header).
      // $E000/$E800/$F000: the three 8 KiB PRG selects, six bits each.
      if (namco) begin
        if (nblk == 4'd0)  chr_bank0 <= prg_din;
        if (nblk == 4'd1)  chr_bank1 <= prg_din;
        if (nblk == 4'd2)  chr_bank2 <= prg_din;
        if (nblk == 4'd3)  chr_bank3 <= prg_din;
        if (nblk == 4'd4)  chr_bank4 <= prg_din;
        if (nblk == 4'd5)  chr_bank5 <= prg_din;
        if (nblk == 4'd6)  chr_bank6 <= prg_din;
        if (nblk == 4'd7)  chr_bank7 <= prg_din;
        if (m19 && nblk == 4'd8)  nt_page[0] <= prg_din[0];
        if (m19 && nblk == 4'd9)  nt_page[1] <= prg_din[0];
        if (m19 && nblk == 4'd10) nt_page[2] <= prg_din[0];
        if (m19 && nblk == 4'd11) nt_page[3] <= prg_din[0];
        if (nblk == 4'd12) begin
          prg_bank0 <= prg_din[5:0];
          // "Namco 340 only: select mirroring -- 0 one-screen A, 1 vertical,
          // 2 one-screen B, 3 horizontal."  Applied to every iNES 210 image:
          // the page says "all the commercial Namco 175 games set the upper
          // bits of $E000 to match their respective hardwired nametable
          // mirroring", so reading them as a 340 is right for both ASICs and
          // no submapper alias is needed.
          if (m210) case (prg_din[7:6])
                    2'd0: nt_page <= 4'b0000;   // 1-screen A
                    2'd1: nt_page <= 4'b1010;   // vertical
                    2'd2: nt_page <= 4'b1111;   // 1-screen B
                    2'd3: nt_page <= 4'b1100;   // horizontal
                    endcase
        end
        if (nblk == 4'd13) prg_bank1 <= prg_din[5:0];
        if (nblk == 4'd14) prg_bank2 <= prg_din[5:0];
      end

      // --- 18 (SS 88006) --------------------------------------------------
      // "This mapper is connected only to A12-A14, A0-A1, and D0-D3, so the
      // PRG bank and CHR bank numbers are split over two sequential addresses."
      // PRG high halves are only TWO bits (".... ..HH"), CHR high halves four.
      if (m18) begin
        if (blk == 3'd0) begin
          if (jidx == 2'd0) prg_bank0[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) prg_bank0[5:4] <= prg_din[1:0];
          if (jidx == 2'd2) prg_bank1[3:0] <= prg_din[3:0];
          if (jidx == 2'd3) prg_bank1[5:4] <= prg_din[1:0];
        end
        if (blk == 3'd1) begin
          if (jidx == 2'd0) prg_bank2[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) prg_bank2[5:4] <= prg_din[1:0];
          // jidx 2 = PRG RAM protect: see the DEVIATION note in the header.
        end
        if (blk == 3'd2) begin
          if (jidx == 2'd0) chr_bank0[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) chr_bank0[7:4] <= prg_din[3:0];
          if (jidx == 2'd2) chr_bank1[3:0] <= prg_din[3:0];
          if (jidx == 2'd3) chr_bank1[7:4] <= prg_din[3:0];
        end
        if (blk == 3'd3) begin
          if (jidx == 2'd0) chr_bank2[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) chr_bank2[7:4] <= prg_din[3:0];
          if (jidx == 2'd2) chr_bank3[3:0] <= prg_din[3:0];
          if (jidx == 2'd3) chr_bank3[7:4] <= prg_din[3:0];
        end
        if (blk == 3'd4) begin
          if (jidx == 2'd0) chr_bank4[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) chr_bank4[7:4] <= prg_din[3:0];
          if (jidx == 2'd2) chr_bank5[3:0] <= prg_din[3:0];
          if (jidx == 2'd3) chr_bank5[7:4] <= prg_din[3:0];
        end
        if (blk == 3'd5) begin
          if (jidx == 2'd0) chr_bank6[3:0] <= prg_din[3:0];
          if (jidx == 2'd1) chr_bank6[7:4] <= prg_din[3:0];
          if (jidx == 2'd2) chr_bank7[3:0] <= prg_din[3:0];
          if (jidx == 2'd3) chr_bank7[7:4] <= prg_din[3:0];
        end
        // $E000-$EFFF: the 16-bit reload value, "least significant four bits
        // first".
        if (blk == 3'd6) begin
          if (jidx == 2'd0) irq_latch[3:0]   <= prg_din[3:0];
          if (jidx == 2'd1) irq_latch[7:4]   <= prg_din[3:0];
          if (jidx == 2'd2) irq_latch[11:8]  <= prg_din[3:0];
          if (jidx == 2'd3) irq_latch[15:12] <= prg_din[3:0];
        end
        if (blk == 3'd7) begin
          // $F000 reload (full 16 bits, whatever the current width) + ack.
          if (jidx == 2'd0) begin irq_counter <= irq_latch; irq <= 1'b0; end
          // $F001 [.... FETC] -- also acknowledges.
          if (jidx == 2'd1) begin
            irq_width  <= prg_din[3:1];
            irq_enable <= prg_din[0];
            irq <= 1'b0;
          end
          // $F002 [.... ..MM] -- 0 Horizontal, 1 Vertical, 2 1ScA, 3 1ScB.
          // NOTE the order: horizontal FIRST, the opposite of nearly every
          // other mapper in this file.
          if (jidx == 2'd2) case (prg_din[1:0])
                            2'd0: nt_page <= 4'b1100;   // horizontal
                            2'd1: nt_page <= 4'b1010;   // vertical
                            2'd2: nt_page <= 4'b0000;   // 1-screen A
                            2'd3: nt_page <= 4'b1111;   // 1-screen B
                            endcase
        end
      end

      // --- 32 (Irem G-101) and its $F1 alias ------------------------------
      if (m32any) begin
        if (blk == 3'd0) prg_bank0 <= {1'b0, prg_din[4:0]};
        // "Major League wants hardwired 1-screen mirroring (CIRAM A10 is tied
        // to +5V on this game).  Additionally, the register at $9000 is
        // entirely disabled: the game can only request PRG mode 0."
        if (blk == 3'd1 && !mF1) begin
          nt_page  <= prg_din[0] ? 4'b1100 : 4'b1010;   // 0 vertical, 1 horizontal
          prg_mode <= prg_din[1];
        end
        if (blk == 3'd2) prg_bank1 <= {1'b0, prg_din[4:0]};
        // $B000-$BFFF, mask $F007: eight 1 KiB CHR selects picked by A2-A0.
        if (blk == 3'd3) begin
          if (iidx == 3'd0) chr_bank0 <= prg_din;
          if (iidx == 3'd1) chr_bank1 <= prg_din;
          if (iidx == 3'd2) chr_bank2 <= prg_din;
          if (iidx == 3'd3) chr_bank3 <= prg_din;
          if (iidx == 3'd4) chr_bank4 <= prg_din;
          if (iidx == 3'd5) chr_bank5 <= prg_din;
          if (iidx == 3'd6) chr_bank6 <= prg_din;
          if (iidx == 3'd7) chr_bank7 <= prg_din;
        end
      end

      // --- 65 (Irem H3001) ------------------------------------------------
      if (m65) begin
        if (blk == 3'd0) prg_bank0 <= prg_din[5:0];
        if (blk == 3'd1) begin
          // $9000 [X... ....] PRG bank layout; $9001 [MM.. ....] mirroring
          // (%00 vert, %10 horz, %01 and %11 1ScA); $9003 [E... ....] IRQ
          // enable; $9004 reload; $9005 high 8 bits; $9006 LOW 8 bits.
          if (iidx == 3'd0) prg_mode <= prg_din[7];
          if (iidx == 3'd1) nt_page  <= prg_din[6] ? 4'b0000
                                      : prg_din[7] ? 4'b1100 : 4'b1010;
          if (iidx == 3'd3) begin irq_enable <= prg_din[7]; irq <= 1'b0; end
          if (iidx == 3'd4) begin irq_counter <= irq_latch; irq <= 1'b0; end
          if (iidx == 3'd5) irq_latch[15:8] <= prg_din;
          if (iidx == 3'd6) irq_latch[7:0]  <= prg_din;
        end
        if (blk == 3'd2) prg_bank1 <= prg_din[5:0];
        if (blk == 3'd3) begin
          if (iidx == 3'd0) chr_bank0 <= prg_din;
          if (iidx == 3'd1) chr_bank1 <= prg_din;
          if (iidx == 3'd2) chr_bank2 <= prg_din;
          if (iidx == 3'd3) chr_bank3 <= prg_din;
          if (iidx == 3'd4) chr_bank4 <= prg_din;
          if (iidx == 3'd5) chr_bank5 <= prg_din;
          if (iidx == 3'd6) chr_bank6 <= prg_din;
          if (iidx == 3'd7) chr_bank7 <= prg_din;
        end
      end

      // --- 75 (Konami VRC1) -----------------------------------------------
      // Two 4 KiB CHR banks of five bits each: the low four live in $E000 /
      // $F000, the fifth in $9000 bits 1 and 2.  They are kept in chr_bank0
      // and chr_bank4 and expanded into eight 1 KiB windows below.
      if (m75) begin
        if (blk == 3'd0) prg_bank0 <= {2'b00, prg_din[3:0]};
        if (blk == 3'd1) begin
          nt_page       <= prg_din[0] ? 4'b1100 : 4'b1010;  // 0 vert, 1 horz
          chr_bank0[4]  <= prg_din[1];
          chr_bank4[4]  <= prg_din[2];
        end
        if (blk == 3'd2) prg_bank1 <= {2'b00, prg_din[3:0]};
        if (blk == 3'd4) prg_bank2 <= {2'b00, prg_din[3:0]};
        if (blk == 3'd6) chr_bank0[3:0] <= prg_din[3:0];
        if (blk == 3'd7) chr_bank4[3:0] <= prg_din[3:0];
      end
    end
  end

  // --- PRG ------------------------------------------------------------------
  // 18/19/75/210: three switchable 8 KiB banks then the last one fixed.
  // 32/65: two switchable banks, the second-to-last fixed, and a mode bit that
  // exchanges $8000 with $C000 (the same swap the VRC4 has).
  wire irem = m32any | m65;
  reg [5:0] prgsel;
  always @* begin
    case (prg_ain[14:13])
    2'b00: prgsel = (irem && prg_mode) ? 6'b111110 : prg_bank0;   // $8000
    2'b01: prgsel = prg_bank1;                                    // $A000
    2'b10: prgsel = irem ? (prg_mode ? prg_bank0 : 6'b111110)     // $C000
                         : prg_bank2;
    2'b11: prgsel = 6'b111111;                                    // $E000 = last
    endcase
  end

  // --- CHR ------------------------------------------------------------------
  // SINGLE IMPLEMENTATION (module MMC3's discipline): the address path and the
  // window-vector tap call the SAME function.
  function [7:0] chr_win_of;
    input [2:0] k;
    reg [7:0] v;
    begin
      case (k)
      3'd0: v = chr_bank0;  3'd1: v = chr_bank1;
      3'd2: v = chr_bank2;  3'd3: v = chr_bank3;
      3'd4: v = chr_bank4;  3'd5: v = chr_bank5;
      3'd6: v = chr_bank6;  3'd7: v = chr_bank7;
      endcase
      chr_win_of = v;
    end
  endfunction

  // VRC1 is the one member with 4 KiB granularity, and it announces the legacy
  // $12 PAIR rather than a window vector (five-bit bank ids -- see the CHR
  // CEILING note in the module header).  The address path needs nothing beyond
  // picking the right pair register: {bank[4:0], chr_ain[11:10]} sitting in the
  // window field is bit-for-bit {bank, chr_ain[11:0]} in the byte field, which
  // is exactly what a 4 KiB window is.
  wire [4:0] pair_bank = chr_ain[12] ? chr_bank4[4:0] : chr_bank0[4:0];

  reg [7:0]  chrsel;
  reg [63:0] snap_win_r;
  always @(chr_ain or m75 or pair_bank or chr_bank0 or chr_bank1 or chr_bank2
           or chr_bank3 or chr_bank4 or chr_bank5 or chr_bank6 or chr_bank7) begin
    chrsel     = m75 ? {1'b0, pair_bank, chr_ain[11:10]}
                     : chr_win_of(chr_ain[12:10]);
    snap_win_r = {chr_win_of(3'd7), chr_win_of(3'd6),
                  chr_win_of(3'd5), chr_win_of(3'd4),
                  chr_win_of(3'd3), chr_win_of(3'd2),
                  chr_win_of(3'd1), chr_win_of(3'd0)};
  end

  // --- Buses ----------------------------------------------------------------
  wire        prg_is_ram = (prg_ain[15:13] == 3'b011);
  wire [21:0] prg_ram    = {9'b11_1100_000, prg_ain[12:0]};
  assign prg_aout  = prg_is_ram ? prg_ram
                                : {3'b000, prgsel, prg_ain[12:0]};
  assign chr_allow = flags[15];
  assign chr_aout  = {4'b10_00, chrsel, chr_ain[9:0]};
  assign vram_ce   = chr_ain[13];

  // The Namco 163's two IRQ registers are READ/WRITE ("Games can read and
  // write to these registers in realtime"), and they sit at $5000/$5800 where
  // prg_allow is low -- so this mapper has to answer the read itself.  That is
  // what has_prg_dout/prg_dout are for (nes.v: `from_data_bus = prg_dout_mapper`
  // whenever prg_allow is low).
  wire n163_irq_lo = m19 & (prg_ain[15:11] == 5'b01010);
  wire n163_irq_hi = m19 & (prg_ain[15:11] == 5'b01011);
  assign has_prg_dout = n163_irq_lo | n163_irq_hi;
  always @* begin
    if (n163_irq_hi) prg_dout = {irq_enable, irq_counter[14:8]};
    else             prg_dout = irq_counter[7:0];
  end
  assign prg_allow = (prg_ain[15] && !prg_write) || prg_is_ram;

  // --- Mirroring ------------------------------------------------------------
  // One 4:1 mux over the per-quadrant CIRAM pages, for all six mappers.
  always @* begin
    case (chr_ain[11:10])
    2'd0: vram_a10 = nt_page[0];
    2'd1: vram_a10 = nt_page[1];
    2'd2: vram_a10 = nt_page[2];
    2'd3: vram_a10 = nt_page[3];
    endcase
  end

  // ... and one decode back into the four-code protocol.  The Namco 163's four
  // registers are INDEPENDENT, so it reaches sixteen patterns where NTARR has
  // four names; the decode below reports the PARTITION rather than looking for
  // an exact match:
  //     p0 != p1            -> vertical    (the halves split left/right)
  //     else p0 != p2       -> horizontal  (they split top/bottom)
  //     else                -> one screen, page p0
  // The four canned patterns every other member of this module writes land on
  // exactly the code they mean, so this costs them nothing.  For the 163 it is
  // measurably better than "no exact match -> horizontal": Final Lap spends
  // 34.673 of its 34.828 nametable writes in (0,1,0,0), where the partition
  // rule gets three of the four pages right and the exact-match rule got one.
  assign snap_mirror = (nt_page[0] != nt_page[1]) ? 2'b10      // vertical
                     : (nt_page[0] != nt_page[2]) ? 2'b11      // horizontal
                     :                     {1'b0, nt_page[0]}; // 1ScA / 1ScB
  assign snap_chr_win = snap_win_r;
  assign snap_pair0   = chr_bank0[4:0];
  assign snap_pair1   = chr_bank4[4:0];
  assign snap_ntpages = nt_page;
endmodule

module MultiMapper(input clk, input ce, input ppu_ce, input reset,
                   input [19:0] ppuflags,                           // Misc flags from PPU for MMC5 cheating (unused, kept for interface compat)
                   input [31:0] flags,                              // Misc flags from ines header {prg_size(3), chr_size(3), mapper(8)}
                   input [15:0] prg_ain, output reg [21:0] prg_aout,// PRG Input / Output Address Lines
                   input prg_read, prg_write,                       // PRG Read / write signals
                   input [7:0] prg_din, output reg [7:0] prg_dout,  // PRG Data
                   input [7:0] prg_from_ram,                        // PRG Data from RAM (unused by MMC1/Mapper28/MMC0)
                   output reg prg_allow,                            // PRG Allow write access
                   input chr_read,                                  // Read from CHR
                   input [13:0] chr_ain, output reg [21:0] chr_aout,// CHR Input / Output Address Lines
                   output reg [7:0] chr_dout,                       // Value to override CHR data with
                   output reg has_chr_dout,                         // True if CHR data should be overridden
                   output reg chr_allow,                            // CHR Allow write
                   output reg vram_a10,                             // CHR Value for A10 address line
                   output reg vram_ce,                              // CHR True if the address should be routed to the internal 2kB VRAM.
                   output reg irq,
                   // sd2snes video-bridge CHR-bank snapshot (molde dbg_cpu:
                   // registered in-core, threaded out through nes.v).  Encodes
                   // the SAME per-slot state bridge_sim/mappers.py::chr_slots()
                   // reports (the golden's CMD_CHR_BANK source):
                   //   MMC1 8KB mode: slot0 = chr_bank_0>>1, no slot1;
                   //   MMC1 4KB mode: slot0 = chr_bank_0, slot1 = chr_bank_1;
                   //   Mapper28 family (0/2/3/7/28): slot0 = a53chr masked by
                   //     the CHR size (the simulator masks in cpu_write --
                   //     chr_mask[1:0] is that same mask), no slot1.
                   // slot0 is ALWAYS present (every mapper reports it): the
                   // consumer ties s0_present=1.  Registered under ce (cart
                   // cycle) -> NES:core multicycle covers the shallow cones.
                   output reg        chr_snap_s1_present,
                   output reg [7:0]  chr_snap_s0_bank,
                   output reg [7:0]  chr_snap_s1_bank,
                   // sd2snes video-bridge CHR WINDOW VECTOR snapshot (protocol
                   // v2.5, CMD_CHR_STATE8 $14) -- the MMC3 successor to the
                   // slot0/slot1 pair above.  Window k (= the 1KB slice of the
                   // PPU's 8KB CHR view starting at k*1024) in [k*8 +: 8], SIZE
                   // MASKED here exactly like every other tap masks (the
                   // Mapper28 arm's `& chr_mask[1:0]` is the precedent), so the
                   // byte is the physical 1KB bank index the PPU really fetches.
                   // Lockstep with bridge_sim/mappers.py::Mapper4.chr_windows().
                   //   chr_snap_win_en    = this mapper publishes a window
                   //                        vector (mappers 4 and 69 -- the
                   //                        bridge uses it as the emission gate
                   //                        for $14/$15)
                   //   chr_snap_win_flags = the $14 flags BYTE, verbatim.
                   //     bit0     = CHR-RAM (mapper_flags[15]): the renderer
                   //                picks the window SOURCE from it (CHR-ROM
                   //                pre-converted in PSRAM vs the converted
                   //                CHR-RAM mirror in WRAM, design SS5.2).
                   //     bits 7:1 = RESERVED, must stay 0.
                   //   LOCKSTEP with bridge_sim/mailbox.py CHR8_FLAG_CHR_RAM --
                   //   the design fixes the SIZE of this byte, not its bits, so
                   //   this is the one field where RTL and golden had to agree
                   //   by hand.  It is a BYTE, not a bit, precisely so a new
                   //   flag never costs a port again.
                   // Registered under ce (cart cycle), same shallow-cone contract
                   // as chr_snap_* -> NES:core multicycle covers it.
                   output reg [63:0] chr_snap_win,
                   output reg [7:0]  chr_snap_win_flags,
                   output reg        chr_snap_win_en,
                   // sd2snes video-bridge NT-arrangement snapshot (v2.0a).
                   // Registered (molde chr_snap_*): the protocol NTARR code
                   // (FRAME_HDR.flags[5:4]) derived from the ACTIVE mapper's
                   // DYNAMIC mirror control.  Replaces the static
                   // mapper_flags[14] the nes_wrap used to feed snap_ntarr --
                   // MMC1/AxROM change mirroring at runtime and the iNES flag
                   // does not reflect it (proven: metroid golden = 1794 V + 6
                   // 1A frames on a header-`h` cart).  Encodes exactly what
                   // bridge_sim/mailbox.py NTARR_BY_MODE[mapper.mirror_mode]
                   // reports (the golden's FRAME_HDR ntarr source).
                   output reg [1:0]  nt_snap_arr,
                   // sd2snes video-bridge NT-CODE snapshot (Fase 3,
                   // CMD_PPU_SPLITS 0x16).  Bit k = the physical CIRAM page of
                   // logical nametable k.  A SUPERSET of nt_snap_arr: the four
                   // classic arrangements are four of the sixteen codes
                   // (1A=0x0, 1B=0xF, V=0xA, H=0xC), and the other twelve are
                   // exactly what the eight-1KB-window family can produce and
                   // the 2-bit field cannot name.  Same ce registration and
                   // shallow-cone contract as nt_snap_arr.
                   output reg [3:0]  nt_snap_code);
  // Raw mirror control (00=1scr-lo,01=1scr-hi,10=V,11=H) -> protocol NTARR
  // (H=0,V=1,1A/lo=2,1B/hi=3); lockstep with mappers.py MIRROR_TABLE ->
  // NTARR_BY_MODE.  Same field vram_a10 decodes in each mapper.
  function [1:0] ntarr_of;
    input [1:0] mc;
    case (mc)
      2'b00: ntarr_of = 2'd2;   // one-screen lower  -> 1A
      2'b01: ntarr_of = 2'd3;   // one-screen upper  -> 1B
      2'b10: ntarr_of = 2'd1;   // vertical          -> V
      2'b11: ntarr_of = 2'd0;   // horizontal        -> H
    endcase
  endfunction

  wire mmc0_prg_allow, mmc0_vram_a10, mmc0_vram_ce, mmc0_chr_allow;
  wire [21:0] mmc0_prg_addr, mmc0_chr_addr;
  MMC0 mmc0(clk, ce, flags, prg_ain, mmc0_prg_addr, prg_read, prg_write, prg_din, mmc0_prg_allow,
                            chr_ain, mmc0_chr_addr, mmc0_chr_allow, mmc0_vram_a10, mmc0_vram_ce);

  wire mmc1_prg_allow, mmc1_vram_a10, mmc1_vram_ce, mmc1_chr_allow;
  wire [21:0] mmc1_prg_addr, mmc1_chr_addr;
  wire [4:0] mmc1_snap_bank0, mmc1_snap_bank1;
  wire mmc1_snap_4k;
  // Raw 2-bit mirror control -> the 4-bit NT code, for every mapper that does
  // NOT publish its four pages directly.  Bit-for-bit the classic table:
  // 1A = all quadrants page 0, 1B = all page 1, V = A10 follows PPU A10
  // (0,1,0,1), H = A10 follows PPU A11 (0,0,1,1).  Lockstep with
  // bridge_sim/mappers.py NTCODE_BY_MODE.
  function [3:0] ntcode_of;
    input [1:0] mc;
    case (mc)
      2'b00: ntcode_of = 4'h0;   // one-screen lower  -> 1A
      2'b01: ntcode_of = 4'hf;   // one-screen upper  -> 1B
      2'b10: ntcode_of = 4'ha;   // vertical
      2'b11: ntcode_of = 4'hc;   // horizontal
    endcase
  endfunction

  wire [1:0] mmc1_snap_mirror;
  MMC1 mmc1(clk, ce, reset, flags, prg_ain, mmc1_prg_addr, prg_read, prg_write, prg_din, mmc1_prg_allow,
                                   chr_ain, mmc1_chr_addr, mmc1_chr_allow, mmc1_vram_a10, mmc1_vram_ce,
                                   mmc1_snap_bank0, mmc1_snap_bank1, mmc1_snap_4k, mmc1_snap_mirror);

  wire map28_prg_allow, map28_vram_a10, map28_vram_ce, map28_chr_allow;
  wire [21:0] map28_prg_addr, map28_chr_addr;
  wire [1:0] map28_snap_a53chr;
  wire [1:0] map28_snap_mirror;
  Mapper28 map28(clk, ce, reset, flags, prg_ain, map28_prg_addr, prg_read, prg_write, prg_din, map28_prg_allow,
                                        chr_ain, map28_chr_addr, map28_chr_allow, map28_vram_a10, map28_vram_ce,
                                        map28_snap_a53chr, map28_snap_mirror);

  // MMC3 (mapper 4) -- NOTE the 2nd port: `ppu_ce`, not `ce`. This is the only
  // mapper here clocked per PPU dot (its A12 scanline counter needs it); see the
  // CLOCKING note in module MMC3. Same instantiation upstream uses
  // (fpganes/src/mmu.v:1592).
  wire mmc3_prg_allow, mmc3_vram_a10, mmc3_vram_ce, mmc3_chr_allow, mmc3_irq;
  wire [21:0] mmc3_prg_addr, mmc3_chr_addr;
  wire [1:0] mmc3_snap_mirror;
  wire [63:0] mmc3_snap_win;
  wire [3:0] mmc3_snap_pages;   // Fase 3: the four per-quadrant CIRAM pages
  wire [7:0] mmc3_prg_dout;
  wire mmc3_has_dout;
  MMC3 mmc3(clk, ppu_ce, reset, flags, prg_ain, mmc3_prg_addr, prg_read, prg_write, prg_din, mmc3_prg_allow,
                                       chr_ain, mmc3_chr_addr, mmc3_chr_allow, mmc3_vram_a10, mmc3_vram_ce,
                                       mmc3_irq, mmc3_prg_dout, mmc3_has_dout,
                                       mmc3_snap_mirror, mmc3_snap_win,
                                       mmc3_snap_pages);

  // The discrete batch (11/66/79/113/87/34/71/232) -- ONE module, ONE case arm.
  wire disc_prg_allow, disc_vram_a10, disc_vram_ce, disc_chr_allow;
  wire [21:0] disc_prg_addr, disc_chr_addr;
  wire [3:0] disc_snap_chr0, disc_snap_chr1;
  wire [1:0] disc_snap_mirror;
  wire disc_chr_dis;
  // NOTE the 2nd clock enable, `ppu_ce`: the mapper-185 read counter below
  // samples a PPU-side signal and must tick on a PPU dot, not on a cart cycle.
  MapperDiscrete disc(clk, ce, ppu_ce, reset, flags, prg_ain, disc_prg_addr, prg_read, prg_write, prg_din, disc_prg_allow,
                                             chr_read, chr_ain, disc_chr_addr, disc_chr_allow, disc_chr_dis,
                                             disc_vram_a10, disc_vram_ce,
                                             disc_snap_chr0, disc_snap_chr1, disc_snap_mirror);
  // NINA-001 vs BNROM, decided by CHR size exactly as inside MapperDiscrete.
  wire disc_nina = (flags[7:0] == 8'd34) && (flags[13:11] != 3'd0);

  // Mapper 69 (Sunsoft FME-7) -- the second `irq` source in this file.
  wire map69_prg_allow, map69_vram_a10, map69_vram_ce, map69_chr_allow, map69_irq;
  wire [21:0] map69_prg_addr, map69_chr_addr;
  wire [1:0] map69_snap_mirror;
  wire [63:0] map69_snap_win;
  Mapper69 map69(clk, ce, reset, flags, prg_ain, map69_prg_addr, prg_read, prg_write, prg_din, map69_prg_allow,
                                        chr_ain, map69_chr_addr, map69_chr_allow, map69_vram_a10, map69_vram_ce,
                                        map69_irq, map69_snap_mirror, map69_snap_win);

  // Konami VRC2/VRC4 (21/22/23/25) -- the third `irq` source.  Clocked from
  // `ce` (the cart/CPU cycle), NOT ppu_ce: the VRC IRQ counts CPU cycles, and
  // its "scanline mode" is a 114/114/113 prescaler over those cycles, not a
  // real A12 counter.  Same port `ce` the FME-7 uses for its own CPU-cycle
  // counter.
  wire vrc_prg_allow, vrc_vram_a10, vrc_vram_ce, vrc_chr_allow, vrc_irq;
  wire [21:0] vrc_prg_addr, vrc_chr_addr;
  wire [1:0] vrc_snap_mirror;
  wire [63:0] vrc_snap_win;
  VRC24 vrc24(clk, ce, reset, flags, prg_ain, vrc_prg_addr, prg_read, prg_write, prg_din, vrc_prg_allow,
                                     chr_ain, vrc_chr_addr, vrc_chr_allow, vrc_vram_a10, vrc_vram_ce,
                                     vrc_irq, vrc_snap_mirror, vrc_snap_win);

  // The eight-1KB-window family (18/19/32/65/75/210) -- ONE module, ONE arm.
  // Clocked from `ce`: every IRQ in it counts CPU cycles.
  wire m1k_prg_allow, m1k_vram_a10, m1k_vram_ce, m1k_chr_allow, m1k_irq, m1k_has_dout;
  wire [21:0] m1k_prg_addr, m1k_chr_addr;
  wire [7:0]  m1k_prg_dout;
  wire [1:0]  m1k_snap_mirror;
  wire [63:0] m1k_snap_win;
  wire [4:0] m1k_pair0, m1k_pair1;
  wire [3:0] m1k_snap_pages;   // Fase 3: the four per-quadrant CIRAM pages
  Mapper1K m1k(clk, ce, reset, flags, prg_ain, m1k_prg_addr, prg_read, prg_write, prg_din,
                                      m1k_prg_dout, m1k_prg_allow, m1k_has_dout,
                                      chr_ain, m1k_chr_addr, m1k_chr_allow, m1k_vram_a10, m1k_vram_ce,
                                      m1k_irq, m1k_snap_mirror, m1k_snap_win, m1k_pair0, m1k_pair1,
                                      m1k_snap_pages);

  // ---------------------------------------------------------------------------
  // MAPPER SELECT -- one-hot, and deliberately NOT a `case(flags[7:0])`.
  //
  // Upstream (and this file, until the discrete batch) wrote the selection as a
  // case over the mapper number.  With four arms Quartus turned that into a
  // short chain and nobody noticed; with SIXTEEN case values it decomposed the
  // case into a PRIORITY CHAIN, and the measured result was an eight-LUT ripple
  // (Selector22~6 .. ~23) sitting directly in front of `prg_allow`.  That is on
  // the design's critical path -- DmaController/CPU -> the mapper -> main.v's
  // PSRAM state machine (ST_MEM_DELAY/STATE) -- and it cost 2.4 ns of setup
  // slack, i.e. the whole margin plus some (Slow 85C went +0.594 -> -1.819).
  //
  // The late-arriving input on that path is `prg_write` (it reaches the mapper
  // through nes.v's apu_cs), so the number of LUT levels between a module's
  // output and this block's output is what matters.  Written as an AND-OR over
  // a one-hot select that depends ONLY on `flags` -- a load-time register, off
  // every data path -- each output bit is a balanced 6-term OR, two LUT levels,
  // no matter how many mappers the file grows.  Do not "simplify" this back
  // into a case.
  wire [7:0] mnum = flags[7:0];
  wire h_mmc1 = (mnum == 8'd1);
  // Mapper 4 AND the Namco 108 family, which is a MODE of module MMC3 (206/88/
  // 95/154 -- see that module's header).  They share this arm, so they also
  // share the window-vector tap and the dynamic mirror tap below.
  wire h_mmc3 = (mnum == 8'd4)  || (mnum == 8'd206) || (mnum == 8'd88) ||
                (mnum == 8'd95) || (mnum == 8'd154) ||
                // ... and the Taito TC0190/TC0690 pair, a SECOND mode of the
                // same module (its register file is the MMC3's -- see the
                // Taito block in module MMC3).  Riding this arm is the point:
                // a mapper family that needs no new module also needs no new
                // term in the one-hot AND-OR below, which is where the ~50 LEs
                // per arm are actually spent.
                (mnum == 8'd33) || (mnum == 8'd48) ||
                // ... the MMC6 ($F4), a THIRD mode of the same module ...
                (mnum == 8'hF4) ||
                // ... and the Taito X1-005/X1-017 pair, a FOURTH.
                (mnum == 8'd80) || (mnum == 8'd82);
  wire h_m28  = (mnum == 8'd0) || (mnum == 8'd2) || (mnum == 8'd3) ||
                (mnum == 8'd7) || (mnum == 8'd28);
  wire h_m69  = (mnum == 8'd69);
  wire h_disc = (mnum == 8'd11)  || (mnum == 8'd34) || (mnum == 8'd66) ||
                (mnum == 8'd71)  || (mnum == 8'd79) || (mnum == 8'd87) ||
                (mnum == 8'd113) || (mnum == 8'd232) ||
                (mnum == 8'd70)  || (mnum == 8'd78) || (mnum == 8'd86) ||
                (mnum == 8'd89)  || (mnum == 8'd93) || (mnum == 8'd94) ||
                (mnum == 8'd97)  || (mnum == 8'd140) || (mnum == 8'd152) ||
                (mnum == 8'd180) || (mnum == 8'd184) || (mnum == 8'hF8) ||
                (mnum == 8'd72)  || (mnum == 8'd185);
  // Konami VRC2/VRC4: the four iNES numbers are ONE chip with four different
  // register-line wirings, all decoded inside module VRC24.
  wire h_vrc  = (mnum == 8'd21) || (mnum == 8'd22) ||
                (mnum == 8'd23) || (mnum == 8'd25);
  // The eight-1KB-window family; $F1 is the internal alias for mapper 32
  // submapper 1 (Major League), see module Mapper1K.
  wire h_m1k  = (mnum == 8'd18) || (mnum == 8'd19) || (mnum == 8'd32) ||
                (mnum == 8'hF1) || (mnum == 8'd65) || (mnum == 8'd75) ||
                (mnum == 8'd210);
  // Mapper 75 rides the Mapper1K arm for the BUS but is the only member with
  // 4 KiB CHR granularity, so on the TAP side it announces the legacy $12 pair
  // instead of a window vector.
  wire h_m75  = (mnum == 8'd75);
  // Everything else keeps falling back to MMC0, exactly as the old `default`
  // arm did (the loader rejects those images anyway).
  wire h_mmc0 = !(h_mmc1 | h_mmc3 | h_m28 | h_m69 | h_disc | h_vrc | h_m1k);

  // Mask
  reg [5:0] prg_mask;
  reg [6:0] chr_mask;
  integer wi;   // loop var of the CHR window-vector mask (elaboration-time)

  always @* begin
    case(flags[10:8])
    0: prg_mask = 6'b000000;
    1: prg_mask = 6'b000001;
    2: prg_mask = 6'b000011;
    3: prg_mask = 6'b000111;
    4: prg_mask = 6'b001111;
    5: prg_mask = 6'b011111;
    default: prg_mask = 6'b111111;
    endcase

    case(flags[13:11])
    0: chr_mask = 7'b0000000;
    1: chr_mask = 7'b0000001;
    2: chr_mask = 7'b0000011;
    3: chr_mask = 7'b0000111;
    4: chr_mask = 7'b0001111;
    5: chr_mask = 7'b0011111;
    6: chr_mask = 7'b0111111;
    7: chr_mask = 7'b1111111;
    endcase

    irq = 0;
    // Default: this file answers no read itself, so the CPU sees the open-bus
    // stand-in nes.v substitutes whenever prg_allow is low.  The one exception
    // so far is the Namco 163's readable IRQ counter ($5000/$5800) -- see
    // module Mapper1K.  Written as an override of the default rather than as
    // another one-hot AND-OR because prg_dout feeds only the CPU data-bus mux,
    // never prg_allow, so it is off the critical path this block is shaped for.
    prg_dout = 8'hff;
    // Mapper 185's deselected CHR-ROM is the one place in this file that has to
    // answer a PPU read itself; nes.v honours the override (`chr_to_ppu =
    // has_chr_from_ppu_mapper ? chr_from_ppu_mapper : memory_din_ppu`).  The
    // module already restricts `chr_dis` to pattern-table addresses, so
    // nametable reads are untouched.
    has_chr_dout = 0;
    chr_dout = 8'hff;

    // The selection itself: six sources, one-hot, AND-OR (see the MAPPER SELECT
    // block above for why it is not a case).  Every unselected term is ANDed
    // with a hard zero, so an X in a mapper that is not running can never leak
    // into the bus -- the property the old case gave for free.
    prg_aout  = (mmc1_prg_addr  & {22{h_mmc1}}) | (mmc3_prg_addr  & {22{h_mmc3}})
              | (map28_prg_addr & {22{h_m28}})  | (map69_prg_addr & {22{h_m69}})
              | (disc_prg_addr  & {22{h_disc}}) | (mmc0_prg_addr  & {22{h_mmc0}})
              | (vrc_prg_addr   & {22{h_vrc}})  | (m1k_prg_addr & {22{h_m1k}});
    chr_aout  = (mmc1_chr_addr  & {22{h_mmc1}}) | (mmc3_chr_addr  & {22{h_mmc3}})
              | (map28_chr_addr & {22{h_m28}})  | (map69_chr_addr & {22{h_m69}})
              | (disc_chr_addr  & {22{h_disc}}) | (mmc0_chr_addr  & {22{h_mmc0}})
              | (vrc_chr_addr   & {22{h_vrc}})  | (m1k_chr_addr & {22{h_m1k}});
    prg_allow = (mmc1_prg_allow & h_mmc1) | (mmc3_prg_allow & h_mmc3)
              | (map28_prg_allow & h_m28) | (map69_prg_allow & h_m69)
              | (disc_prg_allow & h_disc) | (mmc0_prg_allow & h_mmc0)
              | (vrc_prg_allow & h_vrc) | (m1k_prg_allow & h_m1k);
    chr_allow = (mmc1_chr_allow & h_mmc1) | (mmc3_chr_allow & h_mmc3)
              | (map28_chr_allow & h_m28) | (map69_chr_allow & h_m69)
              | (disc_chr_allow & h_disc) | (mmc0_chr_allow & h_mmc0)
              | (vrc_chr_allow & h_vrc) | (m1k_chr_allow & h_m1k);
    vram_a10  = (mmc1_vram_a10 & h_mmc1) | (mmc3_vram_a10 & h_mmc3)
              | (map28_vram_a10 & h_m28) | (map69_vram_a10 & h_m69)
              | (disc_vram_a10 & h_disc) | (mmc0_vram_a10 & h_mmc0)
              | (vrc_vram_a10 & h_vrc) | (m1k_vram_a10 & h_m1k);
    vram_ce   = (mmc1_vram_ce & h_mmc1) | (mmc3_vram_ce & h_mmc3)
              | (map28_vram_ce & h_m28) | (map69_vram_ce & h_m69)
              | (disc_vram_ce & h_disc) | (mmc0_vram_ce & h_mmc0)
              | (vrc_vram_ce & h_vrc) | (m1k_vram_ce & h_m1k);
    // MMC3 and the FME-7 are the only sources of `irq` (upstream mmu.v:1706/
    // 1721 do the same, MMC3 sharing its arm with 47/118/119 -- pruned here);
    // every other mapper contributes nothing and the line stays at the 0 set
    // above.  The Namco 108 family rides the MMC3 arm and "has no IRQs": no
    // extra gate is needed here because that module can never set irq_enable
    // in n108 mode (module MMC3, n108_drop).  tb_mapeq proves it by hammering
    // $C000/$C001/$E000/$E001 and requiring irq to stay low.
    // The VRC2/VRC4 arm is the third source; mapper 22 (the one VRC2 here)
    // cannot reach the $F00x block at all, so it contributes a constant 0 the
    // same way the Namco 108 family does on the MMC3 arm.
    irq       = (mmc3_irq & h_mmc3) | (map69_irq & h_m69) | (vrc_irq & h_vrc)
              | (m1k_irq & h_m1k);
    if (h_m1k && m1k_has_dout)   prg_dout = m1k_prg_dout;
    if (h_mmc3 && mmc3_has_dout) prg_dout = mmc3_prg_dout;
    has_chr_dout = h_disc & disc_chr_dis;

    if (prg_aout[21:20] == 2'b00)
      prg_aout[19:0] = {prg_aout[19:14] & prg_mask, prg_aout[13:0]};
    if (chr_aout[21:20] == 2'b10)
      chr_aout[19:0] = {chr_aout[19:13] & chr_mask, chr_aout[12:0]};
    // Remap the CHR address into VRAM, if needed.
    chr_aout = vram_ce ? {11'b11_0000_0000_0, vram_a10, chr_ain[9:0]} : chr_aout;
    prg_aout = (prg_ain < 'h2000) ? {11'b11_1000_0000_0, prg_ain[10:0]} : prg_aout;
    prg_allow = prg_allow || (prg_ain < 'h2000);
  end

  // sd2snes video-bridge CHR-bank snapshot (see port comment).  Registered
  // output stage: source cones are pure mapper registers + the quasi-static
  // chr_mask decode of flags -- shallow, and the destinations are ce-gated
  // (NES:core multicycle umbrella).
  // Which SHAPE the active mapper's legacy $12 pair takes.  These are one-hot
  // over `flags` (quasi-static, off every data path) for the same reason the
  // MAPPER SELECT block above is: a `case(flags[7:0])` here would now carry
  // thirty values, and the sixteen-value one that used to live in this block
  // was already the widest decode in the file.
  //   tap_pair  4KB PAIR announcers: NINA-001 and Sunsoft-1 (184), the only two
  //             members of the discrete batch with two independent 4KB windows.
  //   tap_8k    single 8KB window, size masked.
  //   tap_none  nothing to announce: the window-vector mappers (which publish
  //             chr_snap_win instead and need the constant sentinel -- see the
  //             block below) and every fixed-CHR-RAM board, whose bank register
  //             is never written and must not be reported as if it were.
  wire tap_pair = disc_nina || (mnum == 8'd184) || h_m75;
  // The pair source: four-bit ids from the discrete batch, FIVE-bit ones from
  // the VRC1 (32 x 4 KiB = its full 128 KiB).  Zero-extended so one mask
  // covers both -- see the mask comment on the tap_pair arm below.
  wire [4:0] pair0_src = h_m75 ? m1k_pair0 : {1'b0, disc_snap_chr0};
  wire [4:0] pair1_src = h_m75 ? m1k_pair1 : {1'b0, disc_snap_chr1};
  wire tap_8k   = (mnum == 8'd11)  || (mnum == 8'd66)  || (mnum == 8'd79)  ||
                  (mnum == 8'd113) || (mnum == 8'd87)  || (mnum == 8'd89)  ||
                  (mnum == 8'd70)  || (mnum == 8'd152) || (mnum == 8'd78)  ||
                  (mnum == 8'hF8)  || (mnum == 8'd86)  || (mnum == 8'd140) ||
                  (mnum == 8'd72)  || (mnum == 8'd185);
  wire tap_none = h_mmc3 || h_m69 || h_vrc || (h_m1k && !h_m75) ||
                  (mnum == 8'd71) || (mnum == 8'd232) || (mnum == 8'd93) ||
                  (mnum == 8'd94) || (mnum == 8'd97)  || (mnum == 8'd180) ||
                  ((mnum == 8'd34) && !disc_nina);

  // ---------------------------------------------------------------------------
  // WINDOW-VECTOR SOURCE -- one-hot AND-OR over the mappers that publish eight
  // 1KB windows, feeding ONE size-mask loop below.  Every one of them forms
  // chr_aout as {4'b1000, <8-bit bank>, chr_ain[9:0]}, so the bank sits in
  // [17:10] and MultiMapper's own tail masks chr_aout[19:13] -- i.e. the top
  // FIVE bits of the byte.  Because the shape is identical for all of them,
  // the mask is written once here instead of once per arm; adding a window
  // mapper is then one term, not another copy of the loop.
  wire        win_any = h_mmc3 | h_m69 | h_vrc | (h_m1k & ~h_m75);
  wire [63:0] win_src = (mmc3_snap_win  & {64{h_mmc3}})
                      | (map69_snap_win & {64{h_m69}})
                      | (vrc_snap_win   & {64{h_vrc}})
                      | (m1k_snap_win   & {64{h_m1k & ~h_m75}});

  always @(posedge clk) begin
    if (ce) begin
      if (h_mmc1) begin  // MMC1
        chr_snap_s1_present <= mmc1_snap_4k;
        chr_snap_s0_bank    <= mmc1_snap_4k ? {3'b000, mmc1_snap_bank0}
                                            : {4'b0000, mmc1_snap_bank0[4:1]};
        chr_snap_s1_bank    <= {3'b000, mmc1_snap_bank1};
      end
      else if (tap_none) begin  // MMC3 / FME-7 -- CONSTANT sentinel, ON PURPOSE
        // MMC3's CHR view is a vector of EIGHT 1KB windows, which the slot0/slot1
        // pair this tap encodes (CMD_CHR_STATE $12) cannot represent. Mapper 4
        // publishes it through chr_snap_win / CMD_CHR_STATE8 $14 instead, and the
        // legacy pair stays a CONSTANT, DEFINED sentinel. That constant is not a
        // placeholder -- it is the MECHANISM behind two protocol rules:
        //   1. the per-frame $12 carries an inert value, which is exactly what
        //      the contract asks for ("the renderer ignores the $12 when it saw
        //      the $14"; $12 only keeps its fixed offset 14 so the parser does
        //      not break);
        //   2. because the tap NEVER changes, nes_chrsplit_capture can never see
        //      a mid-display bank change, so its cnt stays 1 and CMD_CHR_SPLITS
        //      $13 is STRUCTURALLY unreachable in mapper 4 -- the hard rule "$13
        //      must not appear in mapper 4", enforced by construction rather than
        //      by a gate that could be wired wrong.  (nes_bridge ALSO orders its
        //      state chain so the $13 branch is not reachable when the window
        //      vector is enabled: belt and braces, zero extra logic.)
        // Written out explicitly instead of falling through to the Mapper28 arm:
        // a53chr does happen to stay 0 for mapper 4 (Mapper28's selreg resets to
        // 1, so only `inner`/`mode` ever get written), but relying on another
        // mapper's incidental state would be a trap.
        chr_snap_s1_present <= 1'b0;
        chr_snap_s0_bank    <= 8'd0;
        chr_snap_s1_bank    <= 8'd0;
      end
      // ... and the same reasoning covers every fixed-CHR-RAM board (71/232/93/
      // 94/97/180 and 34/BNROM): they have nothing to announce, ever, and the
      // defined zero keeps the tap from carrying another mapper's register
      // state.  The Namco 108 family rides the mapper-4 sentinel for the same
      // reason mapper 4 does -- it publishes a WINDOW VECTOR below.
      // NINA-001 and Sunsoft-1 (184) are the 4KB PAIR announcers: their banks
      // are 4KB IDs, SIZE MASKED with the 4KB form of chr_mask ({chr_mask[2:0],
      // 1} = the width that survives in chr_aout[15:12]).  The mask matters on
      // real carts: Sunsoft-1 hardwires the high window to banks 4-7, so on a
      // 16KB CHR board (Atlantis no Nazo) the raw id is ALWAYS out of range
      // while the address path aliases it to 0-3 -- an unmasked id would send
      // the renderer to un-converted PSRAM (the previous game's CHR).
      // The mask is the FIVE-bit form of the 4KB width, {chr_mask[3:0], 1'b1}.
      // For the four-bit ids of NINA-001/Sunsoft-1 that is byte-identical to
      // the old {chr_mask[2:0], 1'b1} -- the extra mask bit lands where their
      // id has no bit -- while the VRC1's five-bit id needs it: on a 128 KiB
      // board (all three VRC1 titles) the old mask would have cut bank 16-31
      // out of the tap while the address path still reached them, which is the
      // Sunsoft-1 failure of NES-MAPPERS-LOTE2.md 5 in the other direction.
      else if (tap_pair) begin
        chr_snap_s1_present <= 1'b1;
        chr_snap_s0_bank    <= {3'b000, pair0_src & {chr_mask[3:0], 1'b1}};
        chr_snap_s1_bank    <= {3'b000, pair1_src & {chr_mask[3:0], 1'b1}};
      end
      // The 8KB-window members of the discrete batch.  SIZE MASKED here, the
      // Mapper28 arm's `& chr_mask[1:0]` being the precedent: the bank sits in
      // chr_aout[16:13], so chr_mask[3:0] is the surviving width.
      else if (tap_8k) begin
        chr_snap_s1_present <= 1'b0;
        chr_snap_s0_bank    <= {4'b0000, disc_snap_chr0 & chr_mask[3:0]};
        chr_snap_s1_bank    <= 8'd0;
      end
      else begin                     // Mapper28 family (0/2/3/7/28) / MMC0
        chr_snap_s1_present <= 1'b0;
        chr_snap_s0_bank    <= {6'b000000, map28_snap_a53chr & chr_mask[1:0]};
        chr_snap_s1_bank    <= 8'd0;
      end
      // v2.5 CHR window vector (see the port comment).  SIZE MASK applied here,
      // where every other tap masks: the address path does
      // `chr_aout[19:13] & chr_mask` with chr_aout[19:18]=2'b00 and
      // chr_aout[17:10]=chrsel, so the surviving bits of an 8-bit window are
      // {chrsel[7:3] & chr_mask[4:0], chrsel[2:0]} -- e.g. CHR-RAM (class 0,
      // chr_mask=0) confines all eight windows to 0..7 = the single 8KB page,
      // exactly what the PPU really fetches.  The mask AND and the mapper-4
      // gate collapse into ONE LUT level per bit, so publishing a defined zero
      // outside mapper 4 is free (and keeps the tap from carrying MMC3
      // registers that some OTHER mapper's writes happened to move).
      // The Namco 108 family (h_mmc3) publishes the vector too, and it MUST:
      // its four 1KB windows move independently, so the $12 pair could not
      // describe it either.  88/154 already carry their A16 bit inside the
      // window byte (module MMC3, chr_win_of), so nothing special happens here.
      // SOURCE is the one-hot `win_src` built next to the MAPPER SELECT block,
      // so this loop is written ONCE no matter how many window mappers exist
      // (every one of them puts the bank in chr_aout[17:10]).
      if (win_any) begin
        for (wi=0; wi<8; wi=wi+1)
          chr_snap_win[wi*8 +: 8] <= {win_src[wi*8+3 +: 5] & chr_mask[4:0],
                                      win_src[wi*8 +: 3]};
        chr_snap_win_flags <= {7'd0, flags[15]};   // bit0 = CHR-RAM
        chr_snap_win_en    <= 1'b1;
      end else begin
        chr_snap_win       <= 64'd0;
        chr_snap_win_flags <= 8'd0;
        chr_snap_win_en    <= 1'b0;
      end
      // NT arrangement (v2.0a): from the ACTIVE mapper's dynamic mirror control.
      // Same ce-registration + shallow-cone contract as chr_snap_* (main.sdc
      // {*|NES:core|*} multicycle covers it).  True default (mappers outside the
      // v0 set) is dead -- those are rejected NOIMPL at load; fall back to the
      // static header mirror there.
      // Reuses the MAPPER SELECT one-hot instead of re-decoding the mapper
      // number: same result, and the decode it used to spell out had grown to
      // thirty values.
      if      (h_mmc1) nt_snap_arr <= ntarr_of(mmc1_snap_mirror);
      // MMC3 owns mirroring dynamically through $A000 and IGNORES the iNES
      // header bit, so the static fallback below would be wrong for it.  The
      // Namco 108 family shares this arm and each of its four variants resolves
      // its own source inside module MMC3 (header / $8000 d6 / CHR A15).
      else if (h_mmc3) nt_snap_arr <= ntarr_of(mmc3_snap_mirror);
      // FME-7 owns mirroring dynamically through R12 (four modes, so the
      // single-screen codes are reachable here as well).
      else if (h_m69)  nt_snap_arr <= ntarr_of(map69_snap_mirror);
      // VRC2/VRC4: the $9000 register, in the SAME field encoding as R12 (see
      // module VRC24's snap_mirror).  Dynamic on every one of the four numbers,
      // so the static header fallback would be wrong here too.
      else if (h_vrc)  nt_snap_arr <= ntarr_of(vrc_snap_mirror);
      // The eight-1KB-window family derives the raw code from its four
      // per-quadrant CIRAM pages (module Mapper1K, snap_mirror).
      else if (h_m1k)  nt_snap_arr <= ntarr_of(m1k_snap_mirror);
      // The discrete batch reports one raw code for all of its members: the
      // static header for most, the V/H bit of 113/78/97, and the single-screen
      // page of 89/152/Cosmo Carrier (and of 71 once its Fire Hawk latch arms).
      else if (h_disc) nt_snap_arr <= ntarr_of(disc_snap_mirror);
      else if (h_m28)  nt_snap_arr <= ntarr_of(map28_snap_mirror);
      // True default (mappers outside the supported set) is dead -- those are
      // rejected NOIMPL at load; fall back to the static header mirror there.
      else             nt_snap_arr <= flags[14] ? 2'd1 : 2'd0;
    end
  end

  // NT CODE (Fase 3, CMD_PPU_SPLITS 0x16) -- its OWN always block so the
  // pre-existing nt_snap_arr chain above is not touched at all.  Same ce
  // registration, same shallow cone, same priority order; the ONLY member that
  // does not go through ntcode_of() is the eight-1KB-window family, which
  // publishes its four pages directly (mappers 95/118/154/163 reach codes the
  // 2-bit legacy field cannot name -- that is the entire point of this tap).
  always @(posedge clk) begin
    if (reset) nt_snap_code <= 4'hc;    // header default is H, as above
    else if (ce) begin
      if      (h_m1k)  nt_snap_code <= m1k_snap_pages;
      else if (h_mmc3) nt_snap_code <= mmc3_snap_pages;
      else if (h_mmc1) nt_snap_code <= ntcode_of(mmc1_snap_mirror);
      else if (h_m69)  nt_snap_code <= ntcode_of(map69_snap_mirror);
      else if (h_vrc)  nt_snap_code <= ntcode_of(vrc_snap_mirror);
      else if (h_disc) nt_snap_code <= ntcode_of(disc_snap_mirror);
      else if (h_m28)  nt_snap_code <= ntcode_of(map28_snap_mirror);
      else             nt_snap_code <= flags[14] ? 4'ha : 4'hc;
    end
  end
endmodule

// PRG       = 0....
// CHR       = 10...
// CHR-VRAM  = 1100
// CPU-RAM   = 1110
// CARTRAM   = 1111
