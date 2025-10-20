`timescale 1us / 1ps

module dev_tb();

bit             clk, ce, res;
logic [31:0]    ia, da;
logic [15:0]    dut_id;
logic [31:0]    dut_dd_i, dut_dd_o;
logic           rw;

initial begin
    $timeformat(-6, 0, " us", 1);

    $dumpfile("dev_tb.vcd");
    $dumpvars();
end

v810_exec dut
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .IA(ia),
   .ID(dut_id),

   .DA(da),
   .DD_I(dut_dd_i),
   .DD_O(dut_dd_o),
   .DD_OE(),
   .BEn(),

   .ST(),
   .DAn(),
   .MRQn(),
   .RW(rw),
   .BCYSTn()
   );

ram #(10, 16) imem
  (
   .CLK(clk),
   .nCE('0),
   .nWE(rw),
   .nOE('0),
   .A(ia[10:1]),
   .DI('Z),
   .DO(dut_id)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE('0),
   .nWE(rw),
   .nOE('0),
   .A(da[11:2]),
   .DI(dut_dd_o),
   .DO(dut_dd_i)
   );

initial begin
    res = 1;
    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

task start_test;
    repeat (4) @(posedge clk) ;
    res <= 0;
endtask

task end_test;
    @(dut.halt) ;               // wait for HALT instruction

    repeat (6) @(posedge clk) ;
    res <= 1;
endtask

initial #0 begin
    test_mov_rr;
    test_alu0;

    $display("Done!");
    $finish();
end

initial #100 begin
    $error("Emergency exit!");
    $fatal(1);
end

task test_mov_rr;
    imem.load_hex("dev_imem_mov_rr.hex");
    start_test;
    end_test;

    for (int i = 1; i < 32; i++)
        ;//assert(dut.rmem[i] == 32'h0);
endtask

task test_alu0;
    imem.load_hex("dev_imem_alu0.hex");
    start_test;
    end_test;

    assert(dut.rmem[3] == 32'h1);
    assert(dut.rmem[5] == 32'h4);
    assert(dut.rmem[7] == 32'h7);
    assert(dut.rmem[9] == 32'h11);
endtask

endmodule

//////////////////////////////////////////////////////////////////////

module ram
  #(parameter AW,
    parameter DW)
  (
   input           CLK,
   input           nCE,
   input           nWE,
   input           nOE,
   input [AW-1:0]  A,
   input [DW-1:0]  DI,
   output [DW-1:0] DO
 );

localparam SIZE = 1 << AW;

bit [DW-1:0]    mem [0:SIZE-1];
bit [DW-1:0]    dor;

task clear;
    for (int i = 0; i < SIZE; i++)
        mem[i] = '0;
endtask

task load_hex(string fn);
    clear();
    $readmemh(fn, mem);
endtask

task load_bin(string fn);
integer fin, code;
    begin
        clear();
        fin = $fopen(fn, "r");
        assert(fin != 0) else $fatal(1, "unable to open file");
        code = $fread(mem, fin, 0, SIZE);
        assert(code == SIZE) else $fatal(1, "file too short");
    end
endtask

always @(negedge CLK) begin
    dor <= mem[A];
end

assign DO = ~(nCE | nOE) ? dor : {DW{1'bz}};

always @(posedge CLK) begin
    if (~(nCE | nWE)) begin
        mem[A] <= DI;
    end
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp ../v810_exec.sv dev_tb.sv && ./dev_tb.vvp"
// End:
