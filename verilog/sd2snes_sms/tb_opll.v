`timescale 1ns/1ns
module tb_opll;
  reg CLK=0, RST=1, CE=0;
  reg ADDR_WE=0, DATA_WE=0;
  reg [7:0] D=0;
  wire signed [14:0] MIX;
  wire TICK;

  opll dut(.CLK(CLK), .RST(RST), .CE(CE), .ADDR_WE(ADDR_WE), .DATA_WE(DATA_WE), .D(D), .MIX(MIX), .TICK(TICK));

  always #5 CLK = ~CLK; // 100MHz sim clock

  // fake CE pulse every ~28 CLKs (~3.58MHz-ish, doesn't need to be exact for a smoke test)
  integer ce_div = 0;
  always @(posedge CLK) begin
    ce_div <= ce_div + 1;
    CE <= (ce_div % 14 == 0);
  end

  task wr(input [7:0] addr, input [7:0] data);
    begin
      @(posedge CLK); ADDR_WE=1; D=addr; @(posedge CLK); ADDR_WE=0;
      @(posedge CLK); DATA_WE=1; D=data; @(posedge CLK); DATA_WE=0;
    end
  endtask

  integer sample_count = 0;
  integer nonzero_count = 0;
  reg signed [14:0] last_mix = 0;
  integer transitions = 0;

  always @(posedge CLK) begin
    if (TICK) begin
      sample_count = sample_count + 1;
      if (MIX != 0) nonzero_count = nonzero_count + 1;
      if (MIX != last_mix) transitions = transitions + 1;
      last_mix = MIX;
      if (sample_count < 40 || (sample_count % 500)==0)
        $display("t=%0t sample#%0d MIX=%0d", $time, sample_count, MIX);
    end
  end

  initial begin
    #100;
    RST = 0;

    // channel 0: instrument 3 (Piano), fnum/block for a mid note, key on
    wr(8'h10, 8'h50);      // fnum lo
    wr(8'h20, 8'h14);      // sustain=0,keyon=1,block=2,fnum_hi=0 -> 0b0_0_1_010_0? recompute below
    wr(8'h30, 8'h35);      // instrument=3, volume=5

    // let it run for a bunch of samples (attack/decay/sustain)
    repeat (400000) @(posedge CLK);

    // key off
    wr(8'h20, 8'h04);      // keyon=0, same block/fnum_hi
    repeat (400000) @(posedge CLK);

    $display("=== summary: %0d samples, %0d nonzero, %0d transitions ===", sample_count, nonzero_count, transitions);
    if (nonzero_count == 0) begin
      $display("FAIL: OPLL produced only silence");
      $finish;
    end
    if (transitions < 10) begin
      $display("FAIL: OPLL output barely changes (stuck?)");
      $finish;
    end
    $display("PASS: OPLL produced a varying, non-silent waveform");
    $finish;
  end
endmodule
