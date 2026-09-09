// sd2snes SMS core -- YM2413/OPLL register file (M-OPLL.1: plumbing only).
//
// STATUS: this milestone implements the chip's WRITE-ONLY register interface
// exactly (address latch + data write, same two-port protocol as the real
// part) so port decode, FM detection and the audio-control mute logic can be
// built and tested end-to-end. MIX is hardwired to silence -- the phase
// generator, envelope generator, sine/log ROMs and the 15-patch instrument
// ROM are M-OPLL.2+ and are NOT in this file yet. Written from Yamaha's
// YM2413 Application Manual and the smspower.org YM2413/AudioControlPort
// pages (public documentation); no code from any other OPLL implementation
// was used, to keep this file's license the same as the rest of this core.
//
// REGISTER MAP (from the app manual -- this part is just bookkeeping, no
// audio-generation logic depends on getting the bit LAYOUT right yet, only
// on latching the right byte at the right address):
//   $00-$07  custom instrument (slot 0's "user" patch) -- 8 bytes, RAM, and
//            the only patch that's writable; instruments 1-15 are a fixed
//            ROM table inside the real chip (M-OPLL.2 territory).
//   $0E      rhythm control: bit5 = rhythm mode enable, bits4:0 = BD/SD/TOM/
//            TC/HH key-on/off when rhythm mode is on.
//   $0F      test register (bring-up only on real hardware; stored, unused).
//   $10-$18  ch0..ch8 F-Number, low 8 bits.
//   $20-$28  ch0..ch8: bit5=sustain, bit4=key-on, bits3:1=block, bit0=F-Num
//            bit8.
//   $30-$38  ch0..ch8: bits7:4=instrument number (0=custom, 1-15=ROM),
//            bits3:0=volume (0=loudest, 15=silent, same sense as the PSG).
// Everything else in $00-$3F is unused on the real chip and just drops the
// write (matches real hardware: OPLL has no register read-back at all, so
// there's no observable difference from actually storing it).
//
// CLOCKING. CE is the same ~3.579545MHz SMS clock enable the PSG and Z80
// run from. The real chip divides that by 72 for its internal sample clock
// (~49.7kHz, the commonly quoted YM2413 native rate) -- TICK below is that
// divider, wired up now so M-OPLL.2 can hang the phase/envelope generators
// off it directly without touching the clock-domain plumbing again.
module opll (
  input             CLK,
  input             RST,
  input             CE,        // SMS clock enable (~3.579545 MHz, 1-CLK pulse)
  input             WE_ADDR,   // write strobe to $F0 (register address latch)
  input             WE_DATA,   // write strobe to $F1 (register data)
  input      [7:0]  D,         // byte the Z80 wrote
  // Bipolar, same 12-bit signed width sms_core.v already captures the PSG
  // into (au_x_r) -- keeps the two sources drop-in swappable in the mixer
  // with no rescaling. M-OPLL.1: always 0.
  output signed [11:0] MIX,
  output reg        TICK       // 1-CLK pulse at the ~49.7kHz internal rate
);

  // ---------------- /72 prescaler ----------------
  reg [6:0] presc;
  always @(posedge CLK) begin
    TICK <= 1'b0;
    if (RST) presc <= 7'd0;
    else if (CE) begin
      if (presc == 7'd71) begin presc <= 7'd0; TICK <= 1'b1; end
      else                      presc <= presc + 7'd1;
    end
  end

  // ---------------- register address latch ($F0) ----------------
  reg [5:0] ra;
  always @(posedge CLK) begin
    if (RST) ra <= 6'd0;
    else if (WE_ADDR) ra <= D[5:0];
  end

  // ---------------- register file ($F1 data writes) ----------------
  reg [7:0] cust      [0:7];   // $00-$07 custom (instrument 0) patch, RAM
  reg [7:0] rhythm_reg;        // $0E
  reg [7:0] test_reg;          // $0F
  reg [7:0] fnum_lo   [0:8];   // $10-$18
  reg [7:0] ch_ctrl   [0:8];   // $20-$28 (sustain/key-on/block/fnum hi)
  reg [7:0] ch_inst_vol[0:8];  // $30-$38 (instrument#/volume)

  integer k;
  always @(posedge CLK) begin
    if (RST) begin
      for (k = 0; k < 8; k = k + 1) cust[k] <= 8'd0;
      rhythm_reg <= 8'd0;
      test_reg   <= 8'd0;
      for (k = 0; k < 9; k = k + 1) begin
        fnum_lo[k] <= 8'd0; ch_ctrl[k] <= 8'd0; ch_inst_vol[k] <= 8'd0;
      end
    end else if (WE_DATA) begin
      if      (ra <= 6'h07)                    cust[ra[2:0]] <= D;
      else if (ra == 6'h0E)                    rhythm_reg    <= D;
      else if (ra == 6'h0F)                    test_reg      <= D;
      else if (ra >= 6'h10 && ra <= 6'h18)      fnum_lo[ra-6'h10]    <= D;
      else if (ra >= 6'h20 && ra <= 6'h28)      ch_ctrl[ra-6'h20]    <= D;
      else if (ra >= 6'h30 && ra <= 6'h38)      ch_inst_vol[ra-6'h30]<= D;
      // else: unused register range, real chip drops it -- so do we
    end
  end

  // M-OPLL.1: no phase/envelope generator yet -- silence.
  assign MIX = 12'sd0;

endmodule
