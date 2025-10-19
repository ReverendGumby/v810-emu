`timescale 1us / 1ns

module v810
  (
   input         CLK,
   input         CE, // global clock enable

   output [31:0] A,
   input [31:0]  D_I,
   input [31:0]  D_O,
   output        D_OE,
   output [3:0]  BEN, // Byte Enable

   output [1:0]  ST, // Status
   output        DAN, // Data Access
   output        MRQN, // Memory ReQuest
   output        RW, // Read / Write
   output        BCYSTN // Bus CYcle STart
   );



endmodule
