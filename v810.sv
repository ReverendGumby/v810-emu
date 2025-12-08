// V810 CPU top-level module
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

import v810_pkg::*;

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
   input         SZRQn, // Bus SiZing ReQuest

   input         INT, // Interrupt
   input [3:0]   INTVn, // Interrupt Level
   input         NMIn // Non-maskable Interrupt
   );

wire            inex_euf;
wire [15:0]     inex_euccb;
wire [4:0]      inex_eucco;
wire            inex_adtrf;
wire            inex_if;
wire [15:0]     inex_cc;
wire            inex_np;
wire [3:0]      inex_iel;
wire [31:0]     inex_ha;
wire            inex_ack;

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

wire [4:0]      sr_ra, sr_wa;
wire [31:0]     sr_rd, sr_wd;
wire            sr_we;
psw_t           psw, psw_reset, psw_set;
wire            ecr_set_eicc, ecr_set_fecc;

v810_inex inex
  (
   .RESn(RESn),
   .CLK(CLK),
   .CE(CE),

   .PSW(psw),

   .INT(INT),
   .INTVn(INTVn),
   .NMIn(NMIn),

   .EUF(inex_euf),
   .EUCCB(inex_euccb),
   .EUCCO(inex_eucco),
   .ADTRF(inex_adtrf),
   .IF(inex_if),
   .CC(inex_cc),
   .NP(inex_np),
   .IEL(inex_iel),
   .HA(inex_ha),
   .ACK(inex_ack)
   );

v810_exec eu
  (
   .RESn(RESn),
   .CLK(CLK),
   .CE(CE),

   .INEX_EUF(inex_euf),
   .INEX_EUCCB(inex_euccb),
   .INEX_EUCCO(inex_eucco),
   .INEX_ADTRF(inex_adtrf),
   .INEX_IF(inex_if),
   .INEX_NP(inex_np),
   .INEX_IEL(inex_iel),
   .INEX_HA(inex_ha),
   .INEX_ACK(inex_ack),

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
   .DACK(eudack),

   .SR_RA(sr_ra),
   .SR_RD(sr_rd),
   .SR_WA(sr_wa),
   .SR_WD(sr_wd),
   .SR_WE(sr_we),
   .PSW(psw),
   .PSW_RESET(psw_reset),
   .PSW_SET(psw_set),
   .ECR_SET_EICC(ecr_set_eicc),
   .ECR_SET_FECC(ecr_set_fecc)
   );

v810_sysreg sr
  (
   .RESn(RESn),
   .CLK(CLK),
   .CE(CE),

   .RA(sr_ra),
   .RD(sr_rd),

   .WA(sr_wa),
   .WD(sr_wd),
   .WE(sr_we),

   .PSW(psw),
   .PSW_RESET(psw_reset),
   .PSW_SET(psw_set),
   .ECR_CC(inex_cc),
   .ECR_SET_EICC(ecr_set_eicc),
   .ECR_SET_FECC(ecr_set_fecc)
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
