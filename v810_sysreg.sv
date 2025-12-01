// System registers
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// TODO: System registers EIPC, EIPSW, FEPC, FEPSW, ECR, PIR, TKCW, ADTRE

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
   input [3:0]   PSW_ALU_FL_RESET,
   input [3:0]   PSW_ALU_FL_SET,
   output [3:0]  PSW_ALU_FL
   );

//////////////////////////////////////////////////////////////////////
// The registers

psw_t           psw;
logic [31:0]    chcw;

//////////////////////////////////////////////////////////////////////
// Read/write interface

logic [31:0]    rd;

always @* begin
    case (RA)
        SRSEL_PSW:      rd = psw;
        SRSEL_CHCW:		rd = chcw;
        default:        rd = 'X;
    endcase
end

assign RD = rd;

always @(posedge CLK) if (CE) begin
    if (~RESn)
        psw <= 32'h00008000;
    else begin
        if (WE & (WA == SRSEL_PSW))
            psw <= WD;
        else
            psw.alu_fl <= (psw.alu_fl & ~PSW_ALU_FL_RESET) | PSW_ALU_FL_SET;
    end

end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        chcw <= '0;
    end
    else begin
        if (WE) begin
            case (WA)
                SRSEL_CHCW:     chcw <= WD;
                default:        ;
            endcase
        end
    end
end

//////////////////////////////////////////////////////////////////////
// Dedicated read ports

assign PSW_ALU_FL = psw.alu_fl;

endmodule
