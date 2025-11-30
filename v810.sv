`timescale 1us / 1ns

module v810
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   output [31:0] A,
   input [31:0]  D_I,
   output [31:0] D_O,
   output [3:0]  BEn, // Byte Enable

   output [1:0]  ST, // Status
   output        DAn, // Data Access
   output        MRQn, // Memory ReQuest
   output        RW, // Read / not Write
   output        BCYSTn, // Bus CYcle STart
   input         READYn,
   input         SZRQn // Bus SiZing ReQuest
   );

logic [31:0]    euia, euda;
logic [31:0]    euid;
logic           euireq, euiack;
wire [31:0]     eudd_i, eudd_o;
wire [1:0]      eudbc;
wire [3:0]      eudbe;
wire            eudwr;
wire            eudmrq;
wire [1:0]      eudst;
wire            eudreq, eudack;

v810_exec eu
  (
   .RESn(RESn),
   .CLK(CLK),
   .CE(CE),

   .IA(euia),
   .ID(euid),
   .IREQ(euireq),
   .IACK(euiack),

   .DA(euda),
   .DD_I(eudd_i),
   .DD_O(eudd_o),
   .DBC(eudbc),
   .DBE(eudbe),
   .DWR(eudwr),
   .DMRQ(eudmrq),
   .DST(eudst),
   .DREQ(eudreq),
   .DACK(eudack)
   );

v810_mem mem
  (
   .RESn(RESn),
   .CLK(CLK),
   .CE(CE),

   .EUDA(euda),
   .EUDD_I(eudd_i),
   .EUDD_O(eudd_o),
   .EUDBC(eudbc),
   .EUDBE(eudbe),
   .EUDWR(eudwr),
   .EUDMRQ(eudmrq),
   .EUDST(eudst),
   .EUDREQ(eudreq),
   .EUDACK(eudack),

   .EUIA(euia),
   .EUID(euid),
   .EUIREQ(euireq),
   .EUIACK(euiack),

   .A(A),
   .D_I(D_I),
   .D_O(D_O),
   .BEn(BEn),
   .ST(ST),
   .DAn(DAn),
   .MRQn(MRQn),
   .RW(RW),
   .BCYSTn(BCYSTn),
   .READYn(READYn),
   .SZRQn(SZRQn)
   );

endmodule
