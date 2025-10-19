`timescale 1us / 1ns

module v810_exec
  (
   input         CLK,
   input         CE, // global clock enable

   // Instruction bus
   output [31:0] IA,
   input [15:0]  ID,

   // Data bus
   output [31:0] DA,
   input [31:0]  DD_I,
   output [31:0] DD_O,
   output        DD_OE,
   output [3:0]  BEN, // Byte Enable

   output [1:0]  ST, // Status
   output        DAN, // Data Access
   output        MRQN, // Memory ReQuest
   output        RW, // Read / Write
   output        BCYSTN // Bus CYcle STart
   );


//////////////////////////////////////////////////////////////////////
// Instruction memory interface

logic [31:0]    imi_a;
logic [15:0]    imi_d;

assign IA = imi_a;
assign imi_d = ID;


//////////////////////////////////////////////////////////////////////
// Program Counter

logic [31:1]    pc;

initial begin
    pc = '0;
end

always @(posedge CLK) if (CE) begin
    pc <= pc + 1;
end

assign imi_a = {pc, 1'b0};


//////////////////////////////////////////////////////////////////////
// Register file

logic [31:0]    rmem [32];

initial begin
    rmem[0] = '0;
end

always @(posedge CLK) if (CE) begin
end

// A debugging aid for iverilog
logic [31:0] rmem00, rmem01, rmem02, rmem03, rmem04, rmem05, rmem06, rmem07, 
             rmem08, rmem09, rmem10, rmem11, rmem12, rmem13, rmem14, rmem15, 
             rmem16, rmem17, rmem18, rmem19, rmem20, rmem21, rmem22, rmem23,
             rmem24, rmem25, rmem26, rmem27, rmem28, rmem29, rmem30, rmem31;

assign rmem00 = rmem[ 0]; assign rmem01 = rmem[ 1]; assign rmem02 = rmem[ 2];
assign rmem03 = rmem[ 3]; assign rmem04 = rmem[ 4]; assign rmem05 = rmem[ 5];
assign rmem06 = rmem[ 6]; assign rmem07 = rmem[ 7]; assign rmem08 = rmem[ 8];
assign rmem09 = rmem[ 9]; assign rmem10 = rmem[10]; assign rmem11 = rmem[11];
assign rmem12 = rmem[12]; assign rmem13 = rmem[13]; assign rmem14 = rmem[14];
assign rmem15 = rmem[15]; assign rmem16 = rmem[16]; assign rmem17 = rmem[17];
assign rmem18 = rmem[18]; assign rmem19 = rmem[19]; assign rmem20 = rmem[20];
assign rmem21 = rmem[21]; assign rmem22 = rmem[22]; assign rmem23 = rmem[23];
assign rmem24 = rmem[24]; assign rmem25 = rmem[25]; assign rmem26 = rmem[26];
assign rmem27 = rmem[27]; assign rmem28 = rmem[28]; assign rmem29 = rmem[29];
assign rmem30 = rmem[30]; assign rmem31 = rmem[31];

endmodule
