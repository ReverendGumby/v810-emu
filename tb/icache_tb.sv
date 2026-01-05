// Instruction cache (I$) development testbench
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ns

module icache_tb();

bit             clk, ce, res;
bit             halted;

wire [31:0]     cpu_a;
logic [31:0]    cpu_d_i;
wire [31:0]     cpu_d_o;
wire [3:0]      cpu_ben;
wire [1:0]      cpu_st;
wire            cpu_dan;
wire            cpu_mrqn;
wire            cpu_rw;
wire            cpu_bcystn;
wire            cpu_readyn;
wand            cpu_szrqn;

logic           imem_cen;
logic [31:0]    imem_do;

logic           dmem_cen;
wire [31:0]     dmem_do;

logic           unk_cen;

initial begin
    $timeformat(-9, 0, " ns", 1);

    $dumpfile("icache_tb.vcd");
    $dumpvars();
end

v810 dut
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .A(cpu_a),
   .D_I(cpu_d_i),
   .D_O(cpu_d_o),
   .BEn(cpu_ben),
   .ST(cpu_st),
   .DAn(cpu_dan),
   .MRQn(cpu_mrqn),
   .RW(cpu_rw),
   .BCYSTn(cpu_bcystn),
   .READYn(cpu_readyn),
   .SZRQn(cpu_szrqn),

   .INT('0),
   .INTVn('1),
   .NMIn('1)
   );

always @* begin
    if (~imem_cen)
        cpu_d_i = imem_do;
    else if (~dmem_cen)
        cpu_d_i = dmem_do;
    else
        cpu_d_i = '0;
end

assign cpu_readyn = unk_cen & imem_cen & dmem_cen;
assign cpu_szrqn = '1;

ram #(10, 32) imem
  (
   .CLK(clk),
   .nCE(imem_cen),
   .nWE('1),
   .nOE('0),
   .nBE(cpu_ben),
   .A(cpu_a[11:2]),
   .DI('Z),
   .DO(imem_do)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE(dmem_cen),
   .nWE(cpu_rw),
   .nOE(~cpu_rw),
   .nBE(cpu_ben),
   .A(cpu_a[11:2]),
   .DI(cpu_d_o),
   .DO(dmem_do)
   );

assign dmem_cen = ~(~cpu_mrqn & ~cpu_a[31]);
assign imem_cen = ~(~cpu_mrqn & cpu_a[31]);
assign unk_cen = ~(dmem_cen & imem_cen);

initial begin
    halted = 0;
    res = 1;
    ce = 0;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk; // 50 MHz
end

always @(posedge clk)
    ce <= ~ce;

event test_started;

task start_test;
    for (int i = 0; i < (1<<dut.icache.DATAW); i++)
        dut.icache.idataram.mem[i] = 'X;

    -> test_started;

    repeat (10) @(posedge clk) ;
    while (ce) @(posedge clk) ;
    res <= 0;
endtask

task end_test;
    @(posedge halted) ;         // wait for HALT instruction

    disable emergency_exit;

    repeat (10) @(posedge clk) ;
    while (ce) @(posedge clk) ;
    res <= 1;
endtask

task test_clear;
    imem.load_hex16("icache_imem_clear.hex");

    start_test;

    // Verify cache clear has started
    #3 assert(dut.icache.chcw.icc == '1);
    assert(dut.icache.chcw.cen != '0);
    assert(dut.icache.chcw.cec != '0);

    end_test;

    // Verify cache clear has completed
    assert(dut.icache.chcw.icc == '0);
    assert(dut.icache.chcw.cen == '0);
    assert(dut.icache.chcw.cec == '0);

    // Verify tag RAM is clear
    for (int i = 0; i < $size(dut.icache.itagram.mem); i++)
        assert(dut.icache.itagram.mem[i] == '0);
endtask

task test_loop;
    imem.load_hex16("icache_imem_loop.hex");

    start_test;
    end_test;

    // Verify cache clear has been enabled
    assert(dut.icache.chcw.ice == '1);

    // Verify the loop and a bit has been loaded into cache
    assert(dut.icache.itagram.mem['h7E] == {4'b0, 2'b10, 22'h200000});
    assert(dut.icache.idataram.mem['hFD] == 32'h9A00445F);
    assert(dut.icache.itagram.mem['h7F] == {4'b0, 2'b11, 22'h200000});
    assert(dut.icache.idataram.mem['hFE] == 32'h680095FC);
    assert(dut.icache.idataram.mem['hFF] == 32'h00000000);
endtask

task test_disable;
    imem.load_hex16("icache_imem_disable.hex");

    start_test;

    // Verify cache clear has been enabled
    #8 assert(dut.icache.chcw.icc == '1);

    end_test;

    // Verify cache clear has been disabled
    assert(dut.icache.chcw.ice == '0);

    // Verify the loop and a bit has been loaded into cache
    assert(dut.icache.itagram.mem['h02] == {4'b0, 2'b11, 22'h200000});
    assert(dut.icache.idataram.mem['h04] == 32'h9A00445F);
    assert(dut.icache.idataram.mem['h05] == 32'h9A0095FC);
    assert(dut.icache.itagram.mem['h03] == {4'b0, 2'b11, 22'h200000});
    assert(dut.icache.idataram.mem['h06] == 32'h9A007018);
    assert(dut.icache.idataram.mem['h07] == 32'h9A009A00);
endtask

initial #0 begin
    test_clear;
    test_loop;
    test_disable;

    $display("Done!");
    $finish();
end

always @test_started begin
    begin :emergency_exit
        #100 ;
        // If we reach this point, halted didn't assert in time.
        $error("Emergency exit!");
        $fatal(1);
    end
end

always @(posedge clk) if (ce) begin
    assert(~dut.icache.icmaint | ~(dut.euireq | dut.euiack));
end

always @(posedge clk) if (ce) begin
    if (res)
        halted <= '0;
    else begin
        if (~cpu_dan & ~cpu_readyn & cpu_mrqn & cpu_st[0])
            halted <= '1;
    end
end

always @(posedge clk) if (0 & ce) begin
    if (~(res | dut.eu.ex_stall | dut.eu.ex_flush | dut.eu.idex_exc | ~|dut.eu.idex_ctl.ex))
        $display("{EX} PC=%x IR=%x", dut.eu.idex_pc, dut.eu.idex_ir);
end

endmodule

//////////////////////////////////////////////////////////////////////

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s icache_tb -o icache_tb.vvp -f v810.files ram.sv icache_tb.sv && ./icache_tb.vvp"
// End:
