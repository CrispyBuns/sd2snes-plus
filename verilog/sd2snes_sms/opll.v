// sd2snes SMS core -- YM2413 (OPLL) FM synthesizer, Sega Mark III / Japanese
// SMS "FM Sound Unit" add-on.
//
// STATUS: first working draft, NOT yet verified against real silicon or a
// golden reference (unlike sms_translate.v's SPC7110 core, which was
// cross-checked against ares). Treat the numbers below as "architecturally
// correct, calibration TBD" -- see the TODO_OPLL_COMPAT markers. Register
// decode, the instrument ROM table and the audio-control mute/detect
// semantics ARE verified against public documentation (see credits below)
// and should not need touching; the envelope-generator RATE CURVES and the
// overall attenuation SCALE are first-pass approximations that will want
// ear-tuning against a real FM Sound Unit -- which is exactly the hardware
// you have, so that loop is now closed.
//
// CREDITS / SOURCES:
//   * Register map (user-instrument bytes 00-07, system reg 0E, per-channel
//     10-18/20-28/30-38) verified against the documented layout in
//     aaronsgiles/ymfm (BSD-3-Clause, (c) 2021 Aaron Giles), src/ymfm_opl.h.
//   * The 15 melodic + 3 rhythm factory patches (instrument ROM table below)
//     are the YM2413 default table from the same project
//     (src/ymfm_opl.cpp, credited there to David Viens, tweaked by
//     Hubert Lamontagne). Reproduced here as plain register data (not
//     source code) under the same BSD-3-Clause terms.
//   * $F0/$F1 (address/data) and $F2 (audio control: mute bits + detection
//     readback) semantics from smspower.org/Development/YM2413 and
//     smspower.org/Development/AudioControlPort.
//   * logsin/exp waveform-reconstruction tables: generated from the closed-
//     form OPL formulas (logsin[i] = round(-log2(sin((i+0.5)*pi/512))*256),
//     exp[i] = round((2^(i/256)-1)*1024)) used by essentially every
//     open-source OPL-family emulator; not copied from any one project.
//
// ARCHITECTURE. Unlike psg.v's fully-parallel channels, this is still small
// enough (9 channels x 2 operators = 18 "slots") to do in parallel with
// plain Verilog arrays + a `for` loop per always block, the same idiom
// psg.v already uses for its 3 tone channels -- no time-multiplexed FSM,
// so there is one copy of the accumulator/envelope logic per slot rather
// than 18 passes through a shared datapath. Simpler to read and to debug
// in simulation; if LE count on the Mk.III Cyclone V ever becomes tight,
// multiplexing 18 slots through one datapath (like the real chip does) is
// the obvious place to claw it back.
//
// SAMPLE RATE. One output sample is produced per OPLL_TICK (see below),
// derived from CE (~3.579545 MHz) divided by OPLL_TICK_DIV. The real chip
// runs its internal EG/phase update at clock/72; we use a simpler divider
// (defaulted to 72 to match) -- TODO_OPLL_COMPAT: this has not been compared
// against a real unit's pitch reference, only checked for internal
// consistency (doubling per octave, correct relative multiplier ratios).

module opll (
  input             CLK,      // CLK2 (96 MHz, same domain as psg.v)
  input             RST,
  input             CE,       // SMS clock-enable (~3.579545 MHz, 1-CLK pulse)

  // Z80-side register interface (decoded in sms.v: A[7:0]==$F0/$F1)
  input             ADDR_WE,  // 1-CLK pulse: write to $F0 (latch register #)
  input             DATA_WE,  // 1-CLK pulse: write to $F1 (write latched reg)
  input      [7:0]  D,        // byte the Z80 wrote

  output reg signed [14:0] MIX,   // raw 9-channel sum, NOT pre-saturated -- see below
  output                    TICK   // 1-CLK pulse when MIX updates (end of the read-back pipeline below)
);

  localparam OPLL_TICK_DIV = 7'd72;  // CE ticks per sample (matches real EG rate)

  // ---------------- register file ----------------
  reg [7:0] addr_latch;
  always @(posedge CLK) begin
    if (RST) addr_latch <= 8'd0;
    else if (ADDR_WE) addr_latch <= D;
  end

  reg [7:0] user_inst [0:7];      // $00-$07: user (instrument 0) patch bytes
  reg       rhy_enable;            // $0E bit5
  reg [4:0] rhy_keyon;             // $0E bits 4:0 = {BD,SD,TOM,CYM,HH} key-on
  reg [7:0] fnum_lo   [0:8];       // $10-$18
  reg [7:0] chreg2    [0:8];       // $20-$28 raw: {2'b0,sustain,keyon,block[2:0],fnum_hi}
  reg [7:0] chreg3    [0:8];       // $30-$38 raw: {inst[3:0],vol[3:0]}

  integer ri;
  always @(posedge CLK) begin
    if (RST) begin
      for (ri = 0; ri < 8; ri = ri + 1) user_inst[ri] <= 8'd0;
      rhy_enable <= 1'b0; rhy_keyon <= 5'd0;
      for (ri = 0; ri < 9; ri = ri + 1) begin
        fnum_lo[ri] <= 8'd0; chreg2[ri] <= 8'd0; chreg3[ri] <= 8'd0;
      end
    end else if (DATA_WE) begin
      if (addr_latch < 8'h08)
        user_inst[addr_latch[2:0]] <= D;
      else if (addr_latch == 8'h0E) begin
        rhy_enable <= D[5];
        rhy_keyon  <= D[4:0];
      end else if (addr_latch >= 8'h10 && addr_latch <= 8'h18)
        fnum_lo[addr_latch[3:0]] <= D;
      else if (addr_latch >= 8'h20 && addr_latch <= 8'h28)
        chreg2[addr_latch - 8'h20] <= D;
      else if (addr_latch >= 8'h30 && addr_latch <= 8'h38)
        chreg3[addr_latch - 8'h30] <= D;
      // other addresses: real chip ignores them too
    end
  end

  // ---------------- instrument ROM: 15 melodic + 3 rhythm, 8 bytes each ----
  // byte0: op0(modulator) {AM,PM,EGtype,KSR,MULT[3:0]}
  // byte1: op1(carrier)   {AM,PM,EGtype,KSR,MULT[3:0]}
  // byte2: {KSL_carrier[1:0], TL_modulator[5:0]}
  // byte3: {KSL_modulator[1:0], rect_carrier, rect_modulator, feedback[2:0]}
  // byte4: {AR_modulator[3:0], DR_modulator[3:0]}
  // byte5: {AR_carrier[3:0],   DR_carrier[3:0]}
  // byte6: {SL_modulator[3:0], RR_modulator[3:0]}
  // byte7: {SL_carrier[3:0],   RR_carrier[3:0]}
  // (instrument ROM lookup is the case-statement inst_rom/rhy_rom below,
  // matching psg.v's atten2lvl idiom -- synthesis-safe, no local arrays)

  // OPL-standard multiplier table, x2 fixed point (entry 0 = x0.5)
  function [5:0] mult_x2(input [3:0] m);
    case (m)
      4'd0: mult_x2=6'd1;  4'd1: mult_x2=6'd2;  4'd2: mult_x2=6'd4;  4'd3: mult_x2=6'd6;
      4'd4: mult_x2=6'd8;  4'd5: mult_x2=6'd10; 4'd6: mult_x2=6'd12; 4'd7: mult_x2=6'd14;
      4'd8: mult_x2=6'd16; 4'd9: mult_x2=6'd18; 4'd10:mult_x2=6'd20; 4'd11:mult_x2=6'd20;
      4'd12:mult_x2=6'd24; 4'd13:mult_x2=6'd24; 4'd14:mult_x2=6'd30; default: mult_x2=6'd30;
    endcase
  endfunction

  // one 64-bit word per instrument = all 8 bytes in a single memory read
  reg [63:0] inst_mem [0:14];
  reg [63:0] rhy_mem  [0:2];
  initial begin
    inst_mem[0] = 64'h71611E17EF7F0017;
    inst_mem[1] = 64'h13411A0DF8F72313;
    inst_mem[2] = 64'h13019900F2C41123;
    inst_mem[3] = 64'h31610E0798647027;
    inst_mem[4] = 64'h22211E06BF760028;
    inst_mem[5] = 64'h31221605E0710F18;
    inst_mem[6] = 64'h21611D07828F1007;
    inst_mem[7] = 64'h23212D14FF7F0007;
    inst_mem[8] = 64'h41611B0664651017;
    inst_mem[9] = 64'h61610B1885FF8107;
    inst_mem[10] = 64'h13018311FAE41004;
    inst_mem[11] = 64'h17812307F8F82212;
    inst_mem[12] = 64'h61500C05F2F52942;
    inst_mem[13] = 64'h01015403C3920302;
    inst_mem[14] = 64'h41418903F1E51113;
    rhy_mem[0]  = 64'h0101180FDFF86A6D;
    rhy_mem[1]  = 64'h01010000C8D8A748;
    rhy_mem[2]  = 64'h05010000F8AA5955;
  end
  // ---------------- per-channel decode ----------------
  wire [8:0] ch_fnum   [0:8];
  wire [2:0] ch_block  [0:8];
  wire       ch_sustain[0:8];
  wire       ch_keyon_reg[0:8];   // raw $20-$28 bit4 (melodic key-on)
  wire [3:0] ch_instsel[0:8];
  wire [3:0] ch_vol    [0:8];
  genvar gc;
  generate
    for (gc = 0; gc < 9; gc = gc + 1) begin : CHDEC
      assign ch_fnum[gc]      = {chreg2[gc][0], fnum_lo[gc]};
      assign ch_block[gc]     = chreg2[gc][3:1];
      assign ch_sustain[gc]   = chreg2[gc][5];
      assign ch_keyon_reg[gc] = chreg2[gc][4];
      assign ch_instsel[gc]   = chreg3[gc][7:4];
      assign ch_vol[gc]       = chreg3[gc][3:0];
    end
  endgenerate

  // rhythm routing: which channels are diverted to fixed rhythm patches/keyon
  // (channels 6,7,8 zero-indexed == registers' "channel 7/8/9")
  wire is_rhy_ch6 = rhy_enable; // BD, both ops
  wire is_rhy_ch7 = rhy_enable; // mod=HH, car=SD
  wire is_rhy_ch8 = rhy_enable; // mod=TOM, car=CYM

  // ---------------- 18 slots: 2 per channel (0=modulator,1=carrier) --------
  reg  [19:0] phase   [0:17];
  reg  [8:0]  eg_lvl  [0:17];   // 0 = loudest .. 511 = silent
  reg  [1:0]  eg_st   [0:17];   // 0=ATTACK 1=DECAY 2=SUSTAIN 3=RELEASE
  reg         key_d   [0:17];   // previous key-on, for edge detect
  reg  signed [10:0] op_out [0:17]; // this slot's last sample (for feedback + FM)
  reg  signed [10:0] op_out_d[0:17]; // one-tick-older (2-tap feedback average)

  // per-slot: instrument fields, resolved combinationally each tick
  reg [3:0] f_mult   [0:17];
  reg       f_ksr     [0:17];
  reg       f_egsus   [0:17];   // instrument's EG-type bit (1=sustained/organ)
  reg [5:0] f_tl      [0:17];   // modulator only; carrier ignores (uses ch_vol)
  reg [2:0] f_fb      [0:17];   // modulator only
  reg       f_rect    [0:17];   // "half sine" waveform select
  reg [3:0] f_ar      [0:17];
  reg [3:0] f_dr      [0:17];
  reg [3:0] f_sl      [0:17];
  reg [3:0] f_rr      [0:17];
  reg       f_key     [0:17];
  reg [8:0] f_fnum    [0:17];
  reg [2:0] f_block   [0:17];

  // Per-channel/operator field resolution (combinational). Written as an
  // explicit function of (chan, isCarrier) rather than the loop stub above,
  // because instrument-byte layout differs by operator (see byte-layout doc
  // above inst_mem). Reading is now ONE 64-bit word per channel (not 8
  // separate case-table lookups) -- Quartus can trivially map a 15/3-entry
  // array read to a handful of LEs or a tiny memory, unlike the 120-entry
  // decoder this replaced.
  integer c2, op;
  reg [7:0] rb0, rb1, rb2, rb3, rb4, rb5, rb6, rb7; // resolved instrument bytes for this channel
  reg [63:0] iw; // whichever instrument word applies to this channel
  always @* begin
    for (c2 = 0; c2 < 9; c2 = c2 + 1) begin
      // pick the instrument word: user (0), ROM (1-15), or rhythm override
      if ((c2 == 6) && is_rhy_ch6)            iw = rhy_mem[0];
      else if ((c2 == 7) && is_rhy_ch7)       iw = rhy_mem[1];
      else if ((c2 == 8) && is_rhy_ch8)       iw = rhy_mem[2];
      else if (ch_instsel[c2] == 4'd0)        iw = {user_inst[0],user_inst[1],user_inst[2],user_inst[3],
                                                     user_inst[4],user_inst[5],user_inst[6],user_inst[7]};
      else                                    iw = inst_mem[ch_instsel[c2] - 4'd1];
      {rb0,rb1,rb2,rb3,rb4,rb5,rb6,rb7} = iw;

      // modulator = slot 2*c2, carrier = slot 2*c2+1
      f_mult[2*c2]   = rb0[3:0];      f_mult[2*c2+1]   = rb1[3:0];
      f_ksr[2*c2]    = rb0[4];        f_ksr[2*c2+1]    = rb1[4];
      f_egsus[2*c2]  = rb0[5];        f_egsus[2*c2+1]  = rb1[5];
      f_tl[2*c2]     = rb2[5:0];      f_tl[2*c2+1]     = 6'd0; // carrier: n/a
      f_fb[2*c2]     = rb3[2:0];      f_fb[2*c2+1]     = 3'd0;
      f_rect[2*c2]   = rb3[3];        f_rect[2*c2+1]   = rb3[4];
      f_ar[2*c2]     = rb4[7:4];      f_ar[2*c2+1]     = rb5[7:4];
      f_dr[2*c2]     = rb4[3:0];      f_dr[2*c2+1]     = rb5[3:0];
      f_sl[2*c2]     = rb6[7:4];      f_sl[2*c2+1]     = rb7[7:4];
      f_rr[2*c2]     = rb6[3:0];      f_rr[2*c2+1]     = rb7[3:0];
      f_fnum[2*c2]   = ch_fnum[c2];   f_fnum[2*c2+1]   = ch_fnum[c2];
      f_block[2*c2]  = ch_block[c2];  f_block[2*c2+1]  = ch_block[c2];

      // key-on routing
      if ((c2==6) && is_rhy_ch6) begin
        f_key[2*c2] = rhy_keyon[4]; f_key[2*c2+1] = rhy_keyon[4]; // BD both ops
      end else if ((c2==7) && is_rhy_ch7) begin
        f_key[2*c2] = rhy_keyon[0]; f_key[2*c2+1] = rhy_keyon[3]; // mod=HH car=SD
      end else if ((c2==8) && is_rhy_ch8) begin
        f_key[2*c2] = rhy_keyon[2]; f_key[2*c2+1] = rhy_keyon[1]; // mod=TOM car=CYM
      end else begin
        f_key[2*c2] = ch_keyon_reg[c2]; f_key[2*c2+1] = ch_keyon_reg[c2];
      end
    end
  end

  // Real memory arrays (not case-statement decoders) so Quartus can map
  // these to M10K block RAM instead of ~256-way combinational logic --
  // this pair, called once per operator per sample (18x), was the single
  // biggest source of the "combinational node" budget overflow.
  reg [11:0] logsin_mem [0:255];
  reg [9:0]  exp_mem    [0:255];
  initial begin
    logsin_mem[0] = 12'd2137;
    logsin_mem[1] = 12'd1731;
    logsin_mem[2] = 12'd1543;
    logsin_mem[3] = 12'd1419;
    logsin_mem[4] = 12'd1326;
    logsin_mem[5] = 12'd1252;
    logsin_mem[6] = 12'd1190;
    logsin_mem[7] = 12'd1137;
    logsin_mem[8] = 12'd1091;
    logsin_mem[9] = 12'd1050;
    logsin_mem[10] = 12'd1013;
    logsin_mem[11] = 12'd979;
    logsin_mem[12] = 12'd949;
    logsin_mem[13] = 12'd920;
    logsin_mem[14] = 12'd894;
    logsin_mem[15] = 12'd869;
    logsin_mem[16] = 12'd846;
    logsin_mem[17] = 12'd825;
    logsin_mem[18] = 12'd804;
    logsin_mem[19] = 12'd785;
    logsin_mem[20] = 12'd767;
    logsin_mem[21] = 12'd749;
    logsin_mem[22] = 12'd732;
    logsin_mem[23] = 12'd717;
    logsin_mem[24] = 12'd701;
    logsin_mem[25] = 12'd687;
    logsin_mem[26] = 12'd672;
    logsin_mem[27] = 12'd659;
    logsin_mem[28] = 12'd646;
    logsin_mem[29] = 12'd633;
    logsin_mem[30] = 12'd621;
    logsin_mem[31] = 12'd609;
    logsin_mem[32] = 12'd598;
    logsin_mem[33] = 12'd587;
    logsin_mem[34] = 12'd576;
    logsin_mem[35] = 12'd566;
    logsin_mem[36] = 12'd556;
    logsin_mem[37] = 12'd546;
    logsin_mem[38] = 12'd536;
    logsin_mem[39] = 12'd527;
    logsin_mem[40] = 12'd518;
    logsin_mem[41] = 12'd509;
    logsin_mem[42] = 12'd501;
    logsin_mem[43] = 12'd492;
    logsin_mem[44] = 12'd484;
    logsin_mem[45] = 12'd476;
    logsin_mem[46] = 12'd468;
    logsin_mem[47] = 12'd461;
    logsin_mem[48] = 12'd453;
    logsin_mem[49] = 12'd446;
    logsin_mem[50] = 12'd439;
    logsin_mem[51] = 12'd432;
    logsin_mem[52] = 12'd425;
    logsin_mem[53] = 12'd418;
    logsin_mem[54] = 12'd411;
    logsin_mem[55] = 12'd405;
    logsin_mem[56] = 12'd399;
    logsin_mem[57] = 12'd392;
    logsin_mem[58] = 12'd386;
    logsin_mem[59] = 12'd380;
    logsin_mem[60] = 12'd375;
    logsin_mem[61] = 12'd369;
    logsin_mem[62] = 12'd363;
    logsin_mem[63] = 12'd358;
    logsin_mem[64] = 12'd352;
    logsin_mem[65] = 12'd347;
    logsin_mem[66] = 12'd341;
    logsin_mem[67] = 12'd336;
    logsin_mem[68] = 12'd331;
    logsin_mem[69] = 12'd326;
    logsin_mem[70] = 12'd321;
    logsin_mem[71] = 12'd316;
    logsin_mem[72] = 12'd311;
    logsin_mem[73] = 12'd307;
    logsin_mem[74] = 12'd302;
    logsin_mem[75] = 12'd297;
    logsin_mem[76] = 12'd293;
    logsin_mem[77] = 12'd289;
    logsin_mem[78] = 12'd284;
    logsin_mem[79] = 12'd280;
    logsin_mem[80] = 12'd276;
    logsin_mem[81] = 12'd271;
    logsin_mem[82] = 12'd267;
    logsin_mem[83] = 12'd263;
    logsin_mem[84] = 12'd259;
    logsin_mem[85] = 12'd255;
    logsin_mem[86] = 12'd251;
    logsin_mem[87] = 12'd248;
    logsin_mem[88] = 12'd244;
    logsin_mem[89] = 12'd240;
    logsin_mem[90] = 12'd236;
    logsin_mem[91] = 12'd233;
    logsin_mem[92] = 12'd229;
    logsin_mem[93] = 12'd226;
    logsin_mem[94] = 12'd222;
    logsin_mem[95] = 12'd219;
    logsin_mem[96] = 12'd215;
    logsin_mem[97] = 12'd212;
    logsin_mem[98] = 12'd209;
    logsin_mem[99] = 12'd205;
    logsin_mem[100] = 12'd202;
    logsin_mem[101] = 12'd199;
    logsin_mem[102] = 12'd196;
    logsin_mem[103] = 12'd193;
    logsin_mem[104] = 12'd190;
    logsin_mem[105] = 12'd187;
    logsin_mem[106] = 12'd184;
    logsin_mem[107] = 12'd181;
    logsin_mem[108] = 12'd178;
    logsin_mem[109] = 12'd175;
    logsin_mem[110] = 12'd172;
    logsin_mem[111] = 12'd169;
    logsin_mem[112] = 12'd167;
    logsin_mem[113] = 12'd164;
    logsin_mem[114] = 12'd161;
    logsin_mem[115] = 12'd159;
    logsin_mem[116] = 12'd156;
    logsin_mem[117] = 12'd153;
    logsin_mem[118] = 12'd151;
    logsin_mem[119] = 12'd148;
    logsin_mem[120] = 12'd146;
    logsin_mem[121] = 12'd143;
    logsin_mem[122] = 12'd141;
    logsin_mem[123] = 12'd138;
    logsin_mem[124] = 12'd136;
    logsin_mem[125] = 12'd134;
    logsin_mem[126] = 12'd131;
    logsin_mem[127] = 12'd129;
    logsin_mem[128] = 12'd127;
    logsin_mem[129] = 12'd125;
    logsin_mem[130] = 12'd122;
    logsin_mem[131] = 12'd120;
    logsin_mem[132] = 12'd118;
    logsin_mem[133] = 12'd116;
    logsin_mem[134] = 12'd114;
    logsin_mem[135] = 12'd112;
    logsin_mem[136] = 12'd110;
    logsin_mem[137] = 12'd108;
    logsin_mem[138] = 12'd106;
    logsin_mem[139] = 12'd104;
    logsin_mem[140] = 12'd102;
    logsin_mem[141] = 12'd100;
    logsin_mem[142] = 12'd98;
    logsin_mem[143] = 12'd96;
    logsin_mem[144] = 12'd94;
    logsin_mem[145] = 12'd92;
    logsin_mem[146] = 12'd91;
    logsin_mem[147] = 12'd89;
    logsin_mem[148] = 12'd87;
    logsin_mem[149] = 12'd85;
    logsin_mem[150] = 12'd83;
    logsin_mem[151] = 12'd82;
    logsin_mem[152] = 12'd80;
    logsin_mem[153] = 12'd78;
    logsin_mem[154] = 12'd77;
    logsin_mem[155] = 12'd75;
    logsin_mem[156] = 12'd74;
    logsin_mem[157] = 12'd72;
    logsin_mem[158] = 12'd70;
    logsin_mem[159] = 12'd69;
    logsin_mem[160] = 12'd67;
    logsin_mem[161] = 12'd66;
    logsin_mem[162] = 12'd64;
    logsin_mem[163] = 12'd63;
    logsin_mem[164] = 12'd62;
    logsin_mem[165] = 12'd60;
    logsin_mem[166] = 12'd59;
    logsin_mem[167] = 12'd57;
    logsin_mem[168] = 12'd56;
    logsin_mem[169] = 12'd55;
    logsin_mem[170] = 12'd53;
    logsin_mem[171] = 12'd52;
    logsin_mem[172] = 12'd51;
    logsin_mem[173] = 12'd49;
    logsin_mem[174] = 12'd48;
    logsin_mem[175] = 12'd47;
    logsin_mem[176] = 12'd46;
    logsin_mem[177] = 12'd45;
    logsin_mem[178] = 12'd43;
    logsin_mem[179] = 12'd42;
    logsin_mem[180] = 12'd41;
    logsin_mem[181] = 12'd40;
    logsin_mem[182] = 12'd39;
    logsin_mem[183] = 12'd38;
    logsin_mem[184] = 12'd37;
    logsin_mem[185] = 12'd36;
    logsin_mem[186] = 12'd35;
    logsin_mem[187] = 12'd34;
    logsin_mem[188] = 12'd33;
    logsin_mem[189] = 12'd32;
    logsin_mem[190] = 12'd31;
    logsin_mem[191] = 12'd30;
    logsin_mem[192] = 12'd29;
    logsin_mem[193] = 12'd28;
    logsin_mem[194] = 12'd27;
    logsin_mem[195] = 12'd26;
    logsin_mem[196] = 12'd25;
    logsin_mem[197] = 12'd24;
    logsin_mem[198] = 12'd23;
    logsin_mem[199] = 12'd23;
    logsin_mem[200] = 12'd22;
    logsin_mem[201] = 12'd21;
    logsin_mem[202] = 12'd20;
    logsin_mem[203] = 12'd20;
    logsin_mem[204] = 12'd19;
    logsin_mem[205] = 12'd18;
    logsin_mem[206] = 12'd17;
    logsin_mem[207] = 12'd17;
    logsin_mem[208] = 12'd16;
    logsin_mem[209] = 12'd15;
    logsin_mem[210] = 12'd15;
    logsin_mem[211] = 12'd14;
    logsin_mem[212] = 12'd13;
    logsin_mem[213] = 12'd13;
    logsin_mem[214] = 12'd12;
    logsin_mem[215] = 12'd12;
    logsin_mem[216] = 12'd11;
    logsin_mem[217] = 12'd10;
    logsin_mem[218] = 12'd10;
    logsin_mem[219] = 12'd9;
    logsin_mem[220] = 12'd9;
    logsin_mem[221] = 12'd8;
    logsin_mem[222] = 12'd8;
    logsin_mem[223] = 12'd7;
    logsin_mem[224] = 12'd7;
    logsin_mem[225] = 12'd7;
    logsin_mem[226] = 12'd6;
    logsin_mem[227] = 12'd6;
    logsin_mem[228] = 12'd5;
    logsin_mem[229] = 12'd5;
    logsin_mem[230] = 12'd5;
    logsin_mem[231] = 12'd4;
    logsin_mem[232] = 12'd4;
    logsin_mem[233] = 12'd4;
    logsin_mem[234] = 12'd3;
    logsin_mem[235] = 12'd3;
    logsin_mem[236] = 12'd3;
    logsin_mem[237] = 12'd2;
    logsin_mem[238] = 12'd2;
    logsin_mem[239] = 12'd2;
    logsin_mem[240] = 12'd2;
    logsin_mem[241] = 12'd1;
    logsin_mem[242] = 12'd1;
    logsin_mem[243] = 12'd1;
    logsin_mem[244] = 12'd1;
    logsin_mem[245] = 12'd1;
    logsin_mem[246] = 12'd1;
    logsin_mem[247] = 12'd1;
    logsin_mem[248] = 12'd0;
    logsin_mem[249] = 12'd0;
    logsin_mem[250] = 12'd0;
    logsin_mem[251] = 12'd0;
    logsin_mem[252] = 12'd0;
    logsin_mem[253] = 12'd0;
    logsin_mem[254] = 12'd0;
    logsin_mem[255] = 12'd0;
    exp_mem[0] = 10'd0;
    exp_mem[1] = 10'd1;
    exp_mem[2] = 10'd3;
    exp_mem[3] = 10'd4;
    exp_mem[4] = 10'd6;
    exp_mem[5] = 10'd7;
    exp_mem[6] = 10'd8;
    exp_mem[7] = 10'd10;
    exp_mem[8] = 10'd11;
    exp_mem[9] = 10'd13;
    exp_mem[10] = 10'd14;
    exp_mem[11] = 10'd15;
    exp_mem[12] = 10'd17;
    exp_mem[13] = 10'd18;
    exp_mem[14] = 10'd20;
    exp_mem[15] = 10'd21;
    exp_mem[16] = 10'd23;
    exp_mem[17] = 10'd24;
    exp_mem[18] = 10'd26;
    exp_mem[19] = 10'd27;
    exp_mem[20] = 10'd28;
    exp_mem[21] = 10'd30;
    exp_mem[22] = 10'd31;
    exp_mem[23] = 10'd33;
    exp_mem[24] = 10'd34;
    exp_mem[25] = 10'd36;
    exp_mem[26] = 10'd37;
    exp_mem[27] = 10'd39;
    exp_mem[28] = 10'd40;
    exp_mem[29] = 10'd42;
    exp_mem[30] = 10'd43;
    exp_mem[31] = 10'd45;
    exp_mem[32] = 10'd46;
    exp_mem[33] = 10'd48;
    exp_mem[34] = 10'd49;
    exp_mem[35] = 10'd51;
    exp_mem[36] = 10'd52;
    exp_mem[37] = 10'd54;
    exp_mem[38] = 10'd55;
    exp_mem[39] = 10'd57;
    exp_mem[40] = 10'd59;
    exp_mem[41] = 10'd60;
    exp_mem[42] = 10'd62;
    exp_mem[43] = 10'd63;
    exp_mem[44] = 10'd65;
    exp_mem[45] = 10'd66;
    exp_mem[46] = 10'd68;
    exp_mem[47] = 10'd69;
    exp_mem[48] = 10'd71;
    exp_mem[49] = 10'd73;
    exp_mem[50] = 10'd74;
    exp_mem[51] = 10'd76;
    exp_mem[52] = 10'd77;
    exp_mem[53] = 10'd79;
    exp_mem[54] = 10'd81;
    exp_mem[55] = 10'd82;
    exp_mem[56] = 10'd84;
    exp_mem[57] = 10'd85;
    exp_mem[58] = 10'd87;
    exp_mem[59] = 10'd89;
    exp_mem[60] = 10'd90;
    exp_mem[61] = 10'd92;
    exp_mem[62] = 10'd94;
    exp_mem[63] = 10'd95;
    exp_mem[64] = 10'd97;
    exp_mem[65] = 10'd99;
    exp_mem[66] = 10'd100;
    exp_mem[67] = 10'd102;
    exp_mem[68] = 10'd104;
    exp_mem[69] = 10'd105;
    exp_mem[70] = 10'd107;
    exp_mem[71] = 10'd109;
    exp_mem[72] = 10'd110;
    exp_mem[73] = 10'd112;
    exp_mem[74] = 10'd114;
    exp_mem[75] = 10'd115;
    exp_mem[76] = 10'd117;
    exp_mem[77] = 10'd119;
    exp_mem[78] = 10'd120;
    exp_mem[79] = 10'd122;
    exp_mem[80] = 10'd124;
    exp_mem[81] = 10'd126;
    exp_mem[82] = 10'd127;
    exp_mem[83] = 10'd129;
    exp_mem[84] = 10'd131;
    exp_mem[85] = 10'd132;
    exp_mem[86] = 10'd134;
    exp_mem[87] = 10'd136;
    exp_mem[88] = 10'd138;
    exp_mem[89] = 10'd140;
    exp_mem[90] = 10'd141;
    exp_mem[91] = 10'd143;
    exp_mem[92] = 10'd145;
    exp_mem[93] = 10'd147;
    exp_mem[94] = 10'd148;
    exp_mem[95] = 10'd150;
    exp_mem[96] = 10'd152;
    exp_mem[97] = 10'd154;
    exp_mem[98] = 10'd156;
    exp_mem[99] = 10'd157;
    exp_mem[100] = 10'd159;
    exp_mem[101] = 10'd161;
    exp_mem[102] = 10'd163;
    exp_mem[103] = 10'd165;
    exp_mem[104] = 10'd167;
    exp_mem[105] = 10'd168;
    exp_mem[106] = 10'd170;
    exp_mem[107] = 10'd172;
    exp_mem[108] = 10'd174;
    exp_mem[109] = 10'd176;
    exp_mem[110] = 10'd178;
    exp_mem[111] = 10'd180;
    exp_mem[112] = 10'd181;
    exp_mem[113] = 10'd183;
    exp_mem[114] = 10'd185;
    exp_mem[115] = 10'd187;
    exp_mem[116] = 10'd189;
    exp_mem[117] = 10'd191;
    exp_mem[118] = 10'd193;
    exp_mem[119] = 10'd195;
    exp_mem[120] = 10'd197;
    exp_mem[121] = 10'd198;
    exp_mem[122] = 10'd200;
    exp_mem[123] = 10'd202;
    exp_mem[124] = 10'd204;
    exp_mem[125] = 10'd206;
    exp_mem[126] = 10'd208;
    exp_mem[127] = 10'd210;
    exp_mem[128] = 10'd212;
    exp_mem[129] = 10'd214;
    exp_mem[130] = 10'd216;
    exp_mem[131] = 10'd218;
    exp_mem[132] = 10'd220;
    exp_mem[133] = 10'd222;
    exp_mem[134] = 10'd224;
    exp_mem[135] = 10'd226;
    exp_mem[136] = 10'd228;
    exp_mem[137] = 10'd230;
    exp_mem[138] = 10'd232;
    exp_mem[139] = 10'd234;
    exp_mem[140] = 10'd236;
    exp_mem[141] = 10'd238;
    exp_mem[142] = 10'd240;
    exp_mem[143] = 10'd242;
    exp_mem[144] = 10'd244;
    exp_mem[145] = 10'd246;
    exp_mem[146] = 10'd248;
    exp_mem[147] = 10'd250;
    exp_mem[148] = 10'd252;
    exp_mem[149] = 10'd254;
    exp_mem[150] = 10'd257;
    exp_mem[151] = 10'd259;
    exp_mem[152] = 10'd261;
    exp_mem[153] = 10'd263;
    exp_mem[154] = 10'd265;
    exp_mem[155] = 10'd267;
    exp_mem[156] = 10'd269;
    exp_mem[157] = 10'd271;
    exp_mem[158] = 10'd273;
    exp_mem[159] = 10'd275;
    exp_mem[160] = 10'd278;
    exp_mem[161] = 10'd280;
    exp_mem[162] = 10'd282;
    exp_mem[163] = 10'd284;
    exp_mem[164] = 10'd286;
    exp_mem[165] = 10'd288;
    exp_mem[166] = 10'd291;
    exp_mem[167] = 10'd293;
    exp_mem[168] = 10'd295;
    exp_mem[169] = 10'd297;
    exp_mem[170] = 10'd299;
    exp_mem[171] = 10'd301;
    exp_mem[172] = 10'd304;
    exp_mem[173] = 10'd306;
    exp_mem[174] = 10'd308;
    exp_mem[175] = 10'd310;
    exp_mem[176] = 10'd313;
    exp_mem[177] = 10'd315;
    exp_mem[178] = 10'd317;
    exp_mem[179] = 10'd319;
    exp_mem[180] = 10'd322;
    exp_mem[181] = 10'd324;
    exp_mem[182] = 10'd326;
    exp_mem[183] = 10'd328;
    exp_mem[184] = 10'd331;
    exp_mem[185] = 10'd333;
    exp_mem[186] = 10'd335;
    exp_mem[187] = 10'd337;
    exp_mem[188] = 10'd340;
    exp_mem[189] = 10'd342;
    exp_mem[190] = 10'd344;
    exp_mem[191] = 10'd347;
    exp_mem[192] = 10'd349;
    exp_mem[193] = 10'd351;
    exp_mem[194] = 10'd354;
    exp_mem[195] = 10'd356;
    exp_mem[196] = 10'd358;
    exp_mem[197] = 10'd361;
    exp_mem[198] = 10'd363;
    exp_mem[199] = 10'd366;
    exp_mem[200] = 10'd368;
    exp_mem[201] = 10'd370;
    exp_mem[202] = 10'd373;
    exp_mem[203] = 10'd375;
    exp_mem[204] = 10'd378;
    exp_mem[205] = 10'd380;
    exp_mem[206] = 10'd382;
    exp_mem[207] = 10'd385;
    exp_mem[208] = 10'd387;
    exp_mem[209] = 10'd390;
    exp_mem[210] = 10'd392;
    exp_mem[211] = 10'd395;
    exp_mem[212] = 10'd397;
    exp_mem[213] = 10'd399;
    exp_mem[214] = 10'd402;
    exp_mem[215] = 10'd404;
    exp_mem[216] = 10'd407;
    exp_mem[217] = 10'd409;
    exp_mem[218] = 10'd412;
    exp_mem[219] = 10'd414;
    exp_mem[220] = 10'd417;
    exp_mem[221] = 10'd419;
    exp_mem[222] = 10'd422;
    exp_mem[223] = 10'd424;
    exp_mem[224] = 10'd427;
    exp_mem[225] = 10'd430;
    exp_mem[226] = 10'd432;
    exp_mem[227] = 10'd435;
    exp_mem[228] = 10'd437;
    exp_mem[229] = 10'd440;
    exp_mem[230] = 10'd442;
    exp_mem[231] = 10'd445;
    exp_mem[232] = 10'd448;
    exp_mem[233] = 10'd450;
    exp_mem[234] = 10'd453;
    exp_mem[235] = 10'd455;
    exp_mem[236] = 10'd458;
    exp_mem[237] = 10'd461;
    exp_mem[238] = 10'd463;
    exp_mem[239] = 10'd466;
    exp_mem[240] = 10'd469;
    exp_mem[241] = 10'd471;
    exp_mem[242] = 10'd474;
    exp_mem[243] = 10'd477;
    exp_mem[244] = 10'd479;
    exp_mem[245] = 10'd482;
    exp_mem[246] = 10'd485;
    exp_mem[247] = 10'd487;
    exp_mem[248] = 10'd490;
    exp_mem[249] = 10'd493;
    exp_mem[250] = 10'd495;
    exp_mem[251] = 10'd498;
    exp_mem[252] = 10'd501;
    exp_mem[253] = 10'd504;
    exp_mem[254] = 10'd506;
    exp_mem[255] = 10'd509;
  end
  // ---------------- sample-rate tick ----------------
  reg [6:0] tick_div;
  reg       tick_start;   // internal: "begin computing the next sample" pulse
  always @(posedge CLK) begin
    tick_start <= 1'b0;
    if (RST) tick_div <= 7'd0;
    else if (CE) begin
      if (tick_div == OPLL_TICK_DIV - 7'd1) begin tick_div <= 7'd0; tick_start <= 1'b1; end
      else tick_div <= tick_div + 7'd1;
    end
  end

  // ---------------- rate-to-step approximation ----------------
  // TODO_OPLL_COMPAT: geometric first pass, not the real KSR-scaled curve.
  // AR=15 is special-cased to an immediate jump (matches real chip behaviour).
  function [8:0] rate_step(input [3:0] r);
    case (r)
      4'd0: rate_step=9'd1;   4'd1: rate_step=9'd1;   4'd2: rate_step=9'd2;   4'd3: rate_step=9'd2;
      4'd4: rate_step=9'd3;   4'd5: rate_step=9'd4;   4'd6: rate_step=9'd6;   4'd7: rate_step=9'd8;
      4'd8: rate_step=9'd11;  4'd9: rate_step=9'd16;  4'd10:rate_step=9'd22;  4'd11:rate_step=9'd32;
      4'd12:rate_step=9'd45;  4'd13:rate_step=9'd64;  4'd14:rate_step=9'd90;  default: rate_step=9'd128;
    endcase
  endfunction

  wire [8:0] sl_atten [0:17]; // sustain-level target, in our 0..511 atten domain
  generate
    for (gc = 0; gc < 18; gc = gc + 1) begin : SLATT
      // SL 0-14 -> ~32 units/step; SL 15 -> decay all the way to silence
      assign sl_atten[gc] = (f_sl[gc] == 4'hF) ? 9'd511 : {f_sl[gc], 5'd0};
    end
  endgenerate

  // ---------------- per-slot phase + envelope + waveform (pipelined) --------
  // Rewritten as a 5-stage pipeline so logsin_mem/exp_mem are each read from
  // exactly ONE point in the code (still 18-wide arrays, i.e. still 18
  // physical read ports/instances -- this does NOT serialize down to 1
  // instance total) but as PLAIN ARRAY INDEXING on a registered address,
  // which is the idiom Quartus reliably maps to M10K block RAM instead of a
  // 256-way combinational decoder. That decoder replication (18x logsin +
  // 18x exp, each originally a ~256-entry case statement) was the actual
  // cause of the "combinational node" budget overflow -- the arithmetic
  // around it (adders/comparators/muxes) was never the expensive part.
  //
  // Latency: 5 CLK cycles from tick_start to the sample being valid on MIX/
  // TICK. At a ~1929-CLK sample period (72 CE cycles @ CLK/CE~27) this is
  // under 0.3% overhead -- inaudible, and nowhere close to violating the
  // "next tick_start arrives before this one finishes" assumption feedback
  // and modulator/carrier coupling rely on.
  integer k;
  reg  [19:0] phase_inc;
  reg  signed [10:0] mod_in;
  reg  signed [11:0] mod_scaled;
  reg  [9:0]  sine_idx;
  reg  [11:0] comb_atten;
  reg  [9:0]  emag;
  reg  signed [10:0] slot_sample;

  // pipeline-carry registers (indexed by the SAME slot k throughout)
  reg [7:0]  quart_idx_p [0:17];
  reg [1:0]  quad_p      [0:17];
  reg [8:0]  tot_atten_p [0:17];
  reg        frect_p     [0:17];
  reg [11:0] ls_p        [0:17];
  reg [3:0]  eshift_p    [0:17];
  reg [7:0]  efrac_p     [0:17];
  reg [9:0]  expval_p    [0:17];

  reg tick_p1, tick_p2, tick_p3, tick_p4, tick_p5;
  always @(posedge CLK) begin
    if (RST) begin tick_p1<=0; tick_p2<=0; tick_p3<=0; tick_p4<=0; tick_p5<=0; end
    else begin
      tick_p1 <= tick_start;
      tick_p2 <= tick_p1;
      tick_p3 <= tick_p2;
      tick_p4 <= tick_p3;
      tick_p5 <= tick_p4;
    end
  end
  assign TICK = tick_p5;

  // ---- Stage 0 (tick_start): envelope/phase state advance + sine index ----
  always @(posedge CLK) begin
    if (RST) begin
      for (k = 0; k < 18; k = k + 1) begin
        phase[k] <= 20'd0; eg_lvl[k] <= 9'd511; eg_st[k] <= 2'd3; key_d[k] <= 1'b0;
        quart_idx_p[k] <= 8'd0; quad_p[k] <= 2'd0; tot_atten_p[k] <= 9'd0; frect_p[k] <= 1'b0;
      end
    end else if (tick_start) begin
      for (k = 0; k < 18; k = k + 1) begin
        // key edge: retrigger attack, reset phase
        if (f_key[k] & ~key_d[k]) begin
          eg_st[k] <= 2'd0; phase[k] <= 20'd0;
        end else if (~f_key[k] & key_d[k]) begin
          eg_st[k] <= 2'd3;
        end
        key_d[k] <= f_key[k];

        case (eg_st[k])
          2'd0: begin // ATTACK
            if (f_ar[k] == 4'hF) eg_lvl[k] <= 9'd0;
            else if (eg_lvl[k] <= rate_step(f_ar[k])) eg_lvl[k] <= 9'd0;
            else eg_lvl[k] <= eg_lvl[k] - rate_step(f_ar[k]);
            if (eg_lvl[k] == 9'd0) eg_st[k] <= 2'd1;
          end
          2'd1: begin // DECAY
            if (eg_lvl[k] + rate_step(f_dr[k]) >= sl_atten[k]) begin
              eg_lvl[k] <= sl_atten[k]; eg_st[k] <= 2'd2;
            end else eg_lvl[k] <= eg_lvl[k] + rate_step(f_dr[k]);
          end
          2'd2: begin // SUSTAIN (hold if egsus, else keep decaying via RR)
            if (~f_egsus[k]) begin
              if (eg_lvl[k] + rate_step(f_rr[k]) >= 9'd511) eg_lvl[k] <= 9'd511;
              else eg_lvl[k] <= eg_lvl[k] + rate_step(f_rr[k]);
            end
          end
          default: begin // RELEASE
            if (eg_lvl[k] + rate_step(f_rr[k]) >= 9'd511) eg_lvl[k] <= 9'd511;
            else eg_lvl[k] <= eg_lvl[k] + rate_step(f_rr[k]);
          end
        endcase

        // phase accumulate (fnum/block/mult; KSR/KSL not yet modelled)
        phase_inc = ({11'd0, f_fnum[k]} << f_block[k]) * mult_x2(f_mult[k]);
        // modulator (even k) self-feedback; carrier (odd k) gets FM from its
        // paired modulator's PREVIOUS SAMPLE output (one-sample latency,
        // same simplification the original single-cycle version made --
        // real hardware's serial pipeline lets carrier see the current
        // sample's modulator output; this is a documented approximation,
        // not something the pipelining below changed).
        mod_in = k[0] ? op_out[k-1]
                       : (f_fb[k] == 3'd0) ? 11'sd0
                                           : ($signed(op_out[k]) + $signed(op_out_d[k])) >>> (4'd9 - {1'b0,f_fb[k]});
        phase[k] <= phase[k] + phase_inc;

        mod_scaled = $signed(mod_in) >>> 1;   // FM depth knob: >>>1 is a first guess
        sine_idx   = phase[k][19:10] + mod_scaled[9:0];

        quad_p[k]       <= sine_idx[9:8];
        quart_idx_p[k]  <= sine_idx[8] ? ~sine_idx[7:0] : sine_idx[7:0];
        frect_p[k]      <= f_rect[k];
        tot_atten_p[k]  <= (eg_lvl[k] + (k[0] ? {ch_vol[k>>1], 5'd0} : {f_tl[k], 3'd0}) > 9'd511) ? 9'd511
                         : (eg_lvl[k] + (k[0] ? {ch_vol[k>>1], 5'd0} : {f_tl[k], 3'd0}));
      end
    end
  end

  // ---- Stage 1 (tick_p1): logsin_mem read (the actual memory access) ----
  always @(posedge CLK) begin
    if (tick_p1) for (k = 0; k < 18; k = k + 1) ls_p[k] <= logsin_mem[quart_idx_p[k]];
  end

  // ---- Stage 2 (tick_p2): combine waveform atten with envelope/TL/vol ----
  // TODO_OPLL_COMPAT: ls_p (0..2137) and tot_atten_p (0..511) are treated as
  // the same "256 units/octave" scale and simply added -- internally
  // consistent, not yet checked against real chip dB steps (see README).
  always @(posedge CLK) begin
    if (tick_p2) for (k = 0; k < 18; k = k + 1) begin
      comb_atten    = ls_p[k] + {3'd0, tot_atten_p[k]};
      eshift_p[k]  <= comb_atten[11:8];
      efrac_p[k]   <= comb_atten[7:0];
    end
  end

  // ---- Stage 3 (tick_p3): exp_mem read (the other actual memory access) ----
  always @(posedge CLK) begin
    if (tick_p3) for (k = 0; k < 18; k = k + 1) expval_p[k] <= exp_mem[~efrac_p[k]];
  end

  // ---- Stage 4 (tick_p4): reconstruct linear sample, apply sign/rectify ----
  always @(posedge CLK) begin
    if (RST) begin
      for (k = 0; k < 18; k = k + 1) begin op_out[k] <= 11'sd0; op_out_d[k] <= 11'sd0; end
    end else if (tick_p4) begin
      for (k = 0; k < 18; k = k + 1) begin
        emag = (eshift_p[k] > 4'hB) ? 10'd0 : ((expval_p[k] | 10'h200) >> eshift_p[k]);
        slot_sample = (frect_p[k] & quad_p[k][1]) ? 11'sd0
                    : quad_p[k][1] ? -$signed({1'b0, emag})
                                   :  $signed({1'b0, emag});
        op_out_d[k] <= op_out[k];
        op_out[k]   <= slot_sample;
      end
    end
  end

  // ---- Stage 5 (tick_p5): mix carrier slots (odd k), publish MIX/TICK ----
  always @(posedge CLK) begin
    if (RST) MIX <= 15'sd0;
    else if (tick_p5) begin
      // Explicit sign-extension to 15 bits on every term before summing --
      // deliberately NOT relying on Verilog's context-width propagation
      // through a 9-term chain, which behaves consistently across tools
      // only once you've checked it does; this way it's unambiguous.
      MIX <= {{4{op_out[1][10]}},op_out[1]}   + {{4{op_out[3][10]}},op_out[3]}   +
             {{4{op_out[5][10]}},op_out[5]}   + {{4{op_out[7][10]}},op_out[7]}   +
             {{4{op_out[9][10]}},op_out[9]}   + {{4{op_out[11][10]}},op_out[11]} +
             {{4{op_out[13][10]}},op_out[13]} + {{4{op_out[15][10]}},op_out[15]} +
             {{4{op_out[17][10]}},op_out[17]};
    end
  end

endmodule
