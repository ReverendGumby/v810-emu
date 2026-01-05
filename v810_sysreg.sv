// System registers
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// TODO: System registers PIR, TKCW, ADTRE

module v810_sysreg
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Multiplexed read/write interface
   input [4:0]   RA,
   output [31:0] RD,

   input [4:0]   WA,
   input [31:0]  WD,
   input         WE,

   // Dedicated register interface
   output        psw_t PSW,
   input         psw_t PSW_RESET,
   input         psw_t PSW_SET,

   input [15:0]  ECR_CC,
   input         ECR_SET_EICC,
   input         ECR_SET_FECC,

   // External register interface
   input [31:0]  CHCW,
   output [31:0] CHCW_WD,
   output        CHCW_WE
   );

//////////////////////////////////////////////////////////////////////
// The registers

logic [31:0]    eipc, fepc;
psw_t           eipsw, fepsw, psw;
ecr_t           ecr;

//////////////////////////////////////////////////////////////////////
// Read/write interface

logic [31:0]    rd;

always @* begin
    case (RA)
        SRSEL_EIPC:     rd = eipc;
        SRSEL_EIPSW:    rd = eipsw;
        SRSEL_FEPC:     rd = fepc;
        SRSEL_FEPSW:    rd = fepsw;
        SRSEL_ECR:      rd = ecr;
        SRSEL_PSW:      rd = psw;
        SRSEL_CHCW:     rd = CHCW;
        default:        rd = 'X;
    endcase
end

assign RD = rd;

always @(posedge CLK) if (CE) begin
    if (~RESn)
        psw <= '0;
    else begin
        if (WE & (WA == SRSEL_PSW)) begin
            psw <= WD;
        end
        else
            psw <= (psw & ~PSW_RESET) | PSW_SET;
        psw.rfu20 <= '0;
        psw.rfu10 <= '0;
    end
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        ecr <= '0;
    end
    else begin
        if (ECR_SET_EICC)
            ecr.eicc <= ECR_CC;
        else if (ECR_SET_FECC)
            ecr.fecc <= ECR_CC;
    end
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        eipc <= '0;
        eipsw <= '0;
        fepc <= '0;
        fepsw <= '0;
    end
    else begin
        if (WE) begin
            case (WA)
                SRSEL_EIPC:     eipc <= WD;
                SRSEL_EIPSW:    eipsw <= WD;
                SRSEL_FEPC:     fepc <= WD;
                SRSEL_FEPSW:    fepsw <= WD;
                default:        ;
            endcase

            // Some register bits are fixed at 0
            eipc[0] <= '0;
            eipsw.rfu20 <= '0;
            eipsw.rfu10 <= '0;
            fepc[0] <= '0;
            fepsw.rfu20 <= '0;
            fepsw.rfu10 <= '0;
        end
    end
end

//////////////////////////////////////////////////////////////////////
// Dedicated read/write ports

assign PSW = psw;

assign CHCW_WD = WD;
assign CHCW_WE = CE & WE & (WA == SRSEL_CHCW);

endmodule
