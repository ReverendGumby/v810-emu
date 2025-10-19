`timescale 1us / 1ps

module dev_tb();

bit             clk, ce;
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
   .CLK(clk),
   .CE(ce),

   .IA(ia),
   .ID(dut_id),

   .DA(da),
   .DD_I(dut_dd_i),
   .DD_O(dut_dd_o),
   .DD_OE(),
   .BEN(),

   .ST(),
   .DAN(),
   .MRQN(),
   .RW(rw),
   .BCYSTN()
   );

ram #(10, 16) imem
  (
   .CLK(clk),
   .nCE('0),
   .nWE(rw),
   .nOE('0),
   .A(ia[9:0]),
   .DI('Z),
   .DO(dut_id)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE('0),
   .nWE(rw),
   .nOE('0),
   .A(da[9:0]),
   .DI(dut_dd_o),
   .DO(dut_dd_i)
   );

initial begin
    $readmemh("dev_imem.hex", imem.mem);

    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

initial #0 begin
    #10 $finish;
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

always @(posedge CLK) begin
    dor <= mem[A];
end

assign DO = ~(nCE | nOE) ? dor : {DW{1'bz}};

always @(negedge CLK) begin
    if (~(nCE | nWE)) begin
        mem[A] <= DI;
    end
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp ../v810_exec.sv dev_tb.sv && ./dev_tb.vvp"
// End:
