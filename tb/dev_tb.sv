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
    $readmemh("dev_imem.hex", imem.mem);

    res = 1;
    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

initial #0 begin
    repeat (4) @(posedge clk) ;
    res <= 0;

    #10 $error("Emergency exit!");
    $fatal(1);
end

always @(dut.halt) begin
    $display("Decoding HALT instruction");
    #0.2 $finish;
end

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

task load(string fn);
integer fin, code;
    begin
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
