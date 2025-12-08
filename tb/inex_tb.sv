// Interrupt development testbench
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ns

module inex_tb();

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

logic           cpu_int;
logic [3:0]     cpu_intv;
logic           cpu_nmi;

logic           imem_cen;
logic [31:0]    imem_do;

logic           dmem_cen;
wire [31:0]     dmem_do;

logic           unk_cen;

initial begin
    $timeformat(-9, 0, " ns", 1);

    $dumpfile("inex_tb.vcd");
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

   .INT(cpu_int),
   .INTVn(~cpu_intv),
   .NMIn(~cpu_nmi)
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
    ce = 1;
    cpu_int = 0;
    cpu_intv = 0;
    cpu_nmi = 0;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

task start_test;
    repeat (5) @(posedge clk) ;
    res <= 0;
    cpu_nmi <= 0;
    cpu_int <= 0;
    cpu_intv <= 0;
    //repeat (11) @(posedge clk) ; // wait for IF fetching first ins.
    repeat (1) @(posedge clk) ; // wait for reset interrupt latched
endtask

task end_test;
    @(posedge halted) ;         // wait for HALT instruction

    repeat (10) @(posedge clk) ;
    res <= 1;
endtask

task test_nmi;
    imem.load_hex16("inex_imem_nmi.hex");

    for (int i = 0; i < 30; i++) begin
        $display("%t: NMI at RESET + %1dcy", $realtime, i);
        start_test;

        repeat (i) @(posedge clk) ;
        cpu_nmi <= '1;
        @(posedge clk) ;
        cpu_nmi <= '0;

        end_test;

        assert(dut.sr.psw == 32'h00009000);
        assert(dut.sr.eipsw == 32'h00000000);
        assert(dut.sr.eipc == 32'h80000002);
        assert(dut.sr.ecr.fecc == 16'hffd0);
        assert(dut.sr.ecr.eicc == 16'hfff0);
    end
endtask

task test_int8;
    imem.load_hex16("inex_imem_int8.hex");

    for (int i = 0; i < 30; i++) begin
        $display("%t: INT8 at RESET + %1dcy", $realtime, i);
        start_test;

        repeat (i) @(posedge clk) ;
        cpu_int <= '1;
        cpu_intv <= 4'd8;

        end_test;

        assert(dut.sr.psw == 32'h00095000);
        assert(dut.sr.eipsw == 32'h00080000);
        assert(dut.sr.eipc == 32'h80000008);
        assert(dut.sr.ecr.fecc == '0);
        assert(dut.sr.ecr.eicc == 16'hfe80);
    end
endtask

task test_int7_dis;
    imem.load_hex16("inex_imem_int7_dis.hex");

    for (int i = 0; i < 30; i++) begin
        $display("%t: INT7 (disabled) at RESET + %1dcy", $realtime, i);
        start_test;

        repeat (i) @(posedge clk) ;
        cpu_int <= '1;
        cpu_intv <= 4'd7;

        end_test;

        assert(dut.sr.psw == 32'h00080000);
        assert(dut.sr.eipsw == '0);
        assert(dut.sr.ecr.eicc == 16'hfff0);
    end
endtask

initial #0 begin
    //test_nmi;
    //test_int8;
    test_int7_dis;

    $display("Done!");
    $finish();
end

initial #200 begin
    $error("Emergency exit!");
    $fatal(1);
end

always @(posedge clk) if (ce) begin
    if (res)
        halted <= '0;
    else begin
        if (~cpu_dan & ~cpu_readyn & cpu_mrqn & cpu_st[0])
            halted <= '1;
    end
end

always @(posedge clk) if (ce) begin
    if (~(res | dut.eu.ex_stall | dut.eu.ex_flush | dut.eu.idex_exc | ~|dut.eu.idex_ctl.ex))
        $display("{EX} PC=%x IR=%x", dut.eu.idex_pc, dut.eu.idex_ir);
end

endmodule

//////////////////////////////////////////////////////////////////////

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s inex_tb -o inex_tb.vvp -f v810.files ram.sv inex_tb.sv && ./inex_tb.vvp"
// End:
