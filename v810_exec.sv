// Instruction execution unit
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// TODO: These instructions:
// - MUL(U), DIV(U)
// - Bit string manipulation (Bstr)
// - Floating-point operation (Fpp)
// - CAXI

import v810_pkg::*;

module v810_exec
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Interrupt / exception interface
   output        INEX_EUF, // execution exception flag
   output [15:0] INEX_EUCCB, // execution exception code base
   output [4:0]  INEX_EUCCO, // execution exception code offset
   output        INEX_ADTRF, // address trap exception flag
   input         INEX_IF, // Interrupt / Exception Flag
   input         INEX_NP, // Non-maskable int. or duplexed exc.
   input [3:0]   INEX_IEL, // Maskable int. enable level (for PSW.I)
   input [31:0]  INEX_HA, // Handler Address
   output        INEX_ACK, // execution unit acknowledge

   // Instruction bus
   output [31:0] IA,
   input [31:0]  ID,
   output        IREQ, // Access request
   input         IACK, // Access acknowledge

   // Data bus
   output [31:0] DA,
   input [31:0]  DD_I,
   output [31:0] DD_O,
   output [1:0]  DBC, // Byte Count - 1 (0=1, 1=2, 3=4)
   output [3:0]  DBE, // Byte Enable
   output        DWR, // Write / not Read
   output        DMRQ, // Memory Request
   output [1:0]  DST, // Bus Status
   output        DREQ, // Access request
   input         DACK, // Access acknowledge

   // System registers
   output [4:0]  SR_RA,
   input [31:0]  SR_RD,
   output [4:0]  SR_WA,
   output [31:0] SR_WD,
   output        SR_WE,
   input         psw_t PSW,
   output        psw_t PSW_RESET,
   output        psw_t PSW_SET,
   output        ECR_SET_EICC,
   output        ECR_SET_FECC
   );


//////////////////////////////////////////////////////////////////////
// Forward declarations

typedef enum bit [2:0] {
    BCOND_V   = 3'b000,
    BCOND_C   = 3'b001,
    BCOND_Z   = 3'b010,
    BCOND_NH  = 3'b011,
    BCOND_S   = 3'b100,
    BCOND_T   = 3'b101,
    BCOND_LT  = 3'b110,
    BCOND_LTE = 3'b111
} bcond_t;

typedef enum bit [3:0] {
    ALUOP_MOV = 4'b0000,
    ALUOP_ADD = 4'b0001,
    ALUOP_SUB = 4'b0010,
    ALUOP_SHL = 4'b0100,
    ALUOP_SHR = 4'b0101,
    ALUOP_SAR = 4'b0111,
    ALUOP_OR  = 4'b1100,
    ALUOP_AND = 4'b1101,
    ALUOP_XOR = 4'b1110,
    ALUOP_NOT = 4'b1111
} aluop_t;


//////////////////////////////////////////////////////////////////////
// Control registers

wor             if_stall, if_flush;
wor             id_stall, id_flush;
wor             ex_stall, ex_flush;
wor             ma_flush;

logic           halted;         // End of the line


//////////////////////////////////////////////////////////////////////
// Pipeline registers

typedef enum bit [2:0] {
    ALUSRC1_RF_RD1 = 3'd0,
    ALUSRC1_IMM5,
    ALUSRC1_PC,
    ALUSRC1_BMATCH,
    ALUSRC1_SR_RD,
    ALUSRC1_EXC_A
} alu_src1_t;

typedef enum bit [2:0] {
    ALUSRC2_RF_RD2 = 3'd0,
    ALUSRC2_DISP9,
    ALUSRC2_IMM16,
    ALUSRC2_IMM16_HI,
    ALUSRC2_DISP26,
    ALUSRC2_CONST_4
} alu_src2_t;

typedef struct packed {
    logic       Extend; // ins. has multiple EX cycles
    logic [3:0] ALUOp;
    alu_src1_t  ALUSrc1;
    alu_src2_t  ALUSrc2;
    logic       Branch;
    logic [3:0] Bcond;
    sr_sel_t    SRSelRead;
    logic       Halt; // TODO: there's gotta be a better way
    logic       AckExcept;
} ctl_ex_t;

typedef struct packed {
    logic       Write;
    logic       MemReq;
    logic       IOReq;
    logic       FaultReq;
    aluflags_t  FlagMask;
    sr_sel_t    SRSelWrite;
    logic       SRWrite;
    logic       SRtoMem;
    logic       SRtoReg;
} ctl_ma_t;

typedef struct packed {
    logic       RegWrite;
    logic       MemtoReg;
    logic       IOtoReg;
    logic [1:0] MemWidth;
} ctl_wb_t;

// IF/ID
logic [31:0]    ifid_pc;
logic [31:0]    ifid_ir;
logic           ifid_exc;

// ID/EX
logic [31:0]    idex_pc;
logic [31:0]    idex_ir;
logic           idex_exc;
logic [31:0]    idex_rf_rd1, idex_rf_rd2;
logic [4:0]     idex_rf_wa;
struct packed {
    ctl_ex_t    ex;
    ctl_ma_t    ma;
    ctl_wb_t    wb;
} idex_ctl;

// EX/MA
logic [4:0]     exma_rf_wa;
logic [31:0]    exma_rf_rd2;
logic [31:0]    exma_alu_out;
aluflags_t      exma_alu_fl;
logic [31:0]    exma_sr_rd;
struct packed {
    ctl_ma_t    ma;
    ctl_wb_t    wb;
} exma_ctl;

// MA/WB
logic [4:0]     mawb_rf_wa;
logic [31:0]    mawb_alu_out;
logic [31:0]    mawb_mem_rd;
logic [1:0]     mawb_mem_bsel;
struct packed {
    ctl_wb_t    wb;
} mawb_ctl;


//////////////////////////////////////////////////////////////////////
// IF - Instruction Fetch stage
//////////////////////////////////////////////////////////////////////

logic           resh;           // reset exception is being handled
logic           if_ins32_fetch_hi;
wand            if_fetch_en;
logic           if_exc_done;

assign if_fetch_en = ~halted;

//////////////////////////////////////////////////////////////////////
// Instruction memory interface

logic           imi_en;
logic [31:0]    imi_a, imi_an;
logic [31:0]    imi_d, imi_dbuf;
logic [15:0]    idrh;   // last high halfword fetched
logic           imi_a_new;
logic           imi_complete, imi_incomplete;

assign IA = imi_a;
assign IREQ = RESn & imi_a_new;

always @* begin
    imi_d = imi_dbuf;
    if (IACK)
        imi_d = ID;
end

assign imi_en = ~resh;

// Compare word address, to avoid fetching the same word.
// TODO: Compare halfword address if SIZ16B=1
wire imi_a_eq_an = imi_a[31:2] == imi_an[31:2];
wire imi_a_new_p = ~RESn | ~(imi_a_eq_an | if_exc) | (imi_a_new & ~IACK);

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        imi_a <= '0;
        imi_a_new <= '0;
    end
    else if (imi_en & ~imi_incomplete) begin
        imi_a <= imi_an;
        imi_a_new <= imi_a_new_p;
    end

    if (~RESn)
        imi_dbuf <= '0;
    else if (IACK)
        imi_dbuf <= imi_d;
end

assign if_fetch_en = imi_en;

always @(posedge CLK) if (CE) begin
    if (if_fetch_en) begin
        idrh <= imi_d[31:16];
    end
end

assign imi_complete = IREQ & IACK;
assign imi_incomplete = IREQ & ~IACK;

// Disable fetching while memory access completes
assign if_fetch_en = ~imi_incomplete;

// Stall pipeline while memory access completes
assign if_stall = imi_incomplete;
assign id_stall = imi_incomplete;
assign ex_flush = imi_incomplete;

//////////////////////////////////////////////////////////////////////
// Program Counter

logic           if_pc_inc, if_pc_inc4, if_pc_set;
logic           if_pc_set_ins_fetch;
logic [31:0]    if_pc_set_val;

logic [31:0]    pc, pci, pci2, pci4, pcn;

always @* begin
    pci2 = pc + 32'd2;
    pci4 = pc + 32'd4;
    pci = if_pc_inc4 ? pci4 : pci2;
end

always @* begin
    pcn = pc;
    if (if_pc_set)
        pcn = if_pc_set_val;
    else if (if_pc_inc)
        pcn = pci;

    pcn[0] = '0; // PC[0] shall never be set.
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        pc <= 'X;
    end
    else if (if_fetch_en) begin
        pc <= pcn;
    end
end

//////////////////////////////////////////////////////////////////////
// Instruction Pre-Decode

logic [31:0]    pd;
logic           if_imi_a2;
logic           if_pdlo_ins32, if_pdhi_ins32;

always @* begin
    if (~RESn) begin
        pd = '0;
    end
    else begin
        pd = imi_d;
    end
end

function if_is_ins32(input [15:0] ins);
    if_is_ins32 = (ins[15:13] == 3'b101) | // Format IV, V
                  ((ins[15:13] == 3'b110) &
                   (~ins[11] | &ins[11:10])) | // Format VI
                  (ins[15:12] == 4'b1110) |    // Format VI
                  (ins[15:11] == 5'b11110) |   // Format VI
                  (ins[15:10] == 6'b111111);   // Format VI
endfunction

always @* begin
    imi_an = imi_a;
    if (~RESn)
        imi_an = pc;
    else if (if_fetch_en) begin
        if (if_imi_a2 & ~if_pc_set)
            imi_an = pcn + 32'd2;
        else
            imi_an = pcn;
    end
end

assign if_pdlo_ins32 = if_is_ins32(pd[15:0]);
assign if_pdhi_ins32 = if_is_ins32(pd[31:16]);

//////////////////////////////////////////////////////////////////////
// 32-bit fetch handling

// Pre-fetch wrapped 32-bit ins. whenever possible.  Stall if two
// fetch cycles needed, e.g., on branching to a wrapped 32-bit ins.

logic           if_wrap, if_wrapped;
logic           if_ir_swap;

always @* begin
    if_pc_inc4 = '0;
    if_wrap = '0;
    if_ins32_fetch_hi = '0;

    casez ({pc[1], if_wrapped, if_pdhi_ins32, if_pdlo_ins32})
        4'b0000,
        4'b100?:
            ;
        4'b00?1,
        4'b110?:    begin
            if_pc_inc4 = '1;
        end
        4'b0010:    begin
            if_wrap = '1;
        end
        4'b101?:    begin
            if_wrap = '1;
            if_ins32_fetch_hi = '1;
        end
        4'b111?:    begin
            if_pc_inc4 = '1;
            if_wrap = '1;
        end
        default: ;              // invalid state
    endcase
end

assign if_ir_swap = pc[1];
assign if_imi_a2 = if_wrap;
assign if_pc_inc = ~(if_ins32_fetch_hi | imi_incomplete);

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        if_wrapped <= '0;
    end
    else begin
        if (if_fetch_en) begin
            if_wrapped <= if_wrap & ~if_pc_set;
        end
    end
end

// Stall pipeline while 32-bit instruction fetch completes.
assign if_stall = if_ins32_fetch_hi;
assign id_stall = if_ins32_fetch_hi;
assign ex_flush = if_ins32_fetch_hi;

//////////////////////////////////////////////////////////////////////
// Instruction Register

logic [31:0]    ir;
logic           ir_valid, ir_valid_d;

always @* begin
    ir = pd;
    if (if_ir_swap)
        ir = {ir[15:0], ir[31:16]};
    if (if_wrapped)
        ir[15:0] = idrh;
    if (if_flush)
        ir = '0;
end

always @* begin
    ir_valid = ir_valid_d;
    if (~RESn | ~imi_en | if_pc_set | imi_incomplete | if_ins32_fetch_hi)
        ir_valid = '0;
    else if (~imi_a_new | imi_complete)
        ir_valid = '1;
end

always @(posedge CLK) if (CE) begin
    ir_valid_d <= ir_valid;
end

// Disable fetching if the next instruction is ready but cannot be output.
assign if_fetch_en = ~(ir_valid & if_stall);

//////////////////////////////////////////////////////////////////////
// Interrupt / Exception Ingress

logic           if_exc;

assign if_exc = INEX_IF & ~if_flush;

always @(posedge CLK) if (CE) begin
    resh <= (resh | ~RESn) & ~if_exc_done;
end

//assign if_stall = ifid_exc & ~if_exc_done;

//////////////////////////////////////////////////////////////////////
// IF/ID pipeline register

always @(posedge CLK) if (CE) begin
    if (~RESn | ~if_stall) begin
        ifid_pc <= pc;
        ifid_ir <= ir;
        ifid_exc <= if_exc;
    end
end


//////////////////////////////////////////////////////////////////////
// ID - Instruction Decode / Register Fetch stage
//////////////////////////////////////////////////////////////////////

logic           id_done;

//////////////////////////////////////////////////////////////////////
// Instruction decoder

logic [4:0]     id_rf_ra1, id_rf_ra2, id_rf_wa;
ctl_ex_t        id_ctl_ex;
ctl_ma_t        id_ctl_ma;
ctl_wb_t        id_ctl_wb;
logic           id_exc_set_ecr, id_exc_set_psw;
logic           id_trap;
logic [5:0]     id_ccnt;
logic           id_invalid;

always @* begin
    id_rf_ra1 = '0;
    id_rf_ra2 = '0;
    id_rf_wa = '0;
    id_ctl_ex = '0;
    id_ctl_ma = '0;
    id_ctl_wb = '0;
    id_exc_set_ecr = '0;
    id_exc_set_psw = '0;
    id_trap = '0;
    id_invalid = '0;

    if (ifid_exc) begin
        // Exception processing
        case (id_ccnt)
            'd0: begin
                id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                id_ctl_ex.ALUOp = ALUOP_MOV;
                id_ctl_ex.AckExcept = '1;
                id_ctl_ma.SRSelWrite = PSW.ep ? SRSEL_FEPC : SRSEL_EIPC;
                id_ctl_ma.SRWrite = '1;
                id_ctl_ex.Extend = '1;
            end
            'd1: begin
                id_ctl_ex.ALUSrc1 = ALUSRC1_SR_RD;
                id_ctl_ex.ALUOp = ALUOP_MOV;
                id_ctl_ex.SRSelRead = SRSEL_PSW;
                id_ctl_ma.SRSelWrite = PSW.ep ? SRSEL_FEPSW : SRSEL_EIPSW;
                id_ctl_ma.SRWrite = '1;
                id_ctl_ex.Extend = '1;
            end
            'd2: begin
                id_exc_set_ecr = '1;
                id_ctl_ex.Extend = '1;
            end
            'd3: begin
                id_exc_set_psw = '1;
                id_ctl_ex.Extend = '1;
            end
            'd4: begin
                id_ctl_ex.ALUSrc1 = ALUSRC1_EXC_A;
                id_ctl_ex.ALUOp = ALUOP_MOV;
                id_ctl_ex.Branch = '1;
                id_ctl_ex.Bcond[2:0] = BCOND_T;
            end
            default: ;
        endcase
    end
    else
        casez (ifid_ir[15:10])
            6'b0?0_00?,             // MOV, ADD
            6'b000_010,             // SUB
            6'b0?0_10?,             // SHL, SHR
            6'b0?0_111,             // SAR
            6'b001_10?,             // OR, AND
            6'b001_110,             // XOR
            6'b001_111:             // NOT
                begin
                    if (~(~ifid_ir[15] &
                          ((ifid_ir[14:10] == 5'b01111) // NOT
                           | (ifid_ir[13:10] == 4'b0000)))) // MOV
                        id_rf_ra2 = ifid_ir[9:5];
                    id_rf_wa = ifid_ir[9:5];
                    if (ifid_ir[14])
                        id_ctl_ex.ALUSrc1 = ALUSRC1_IMM5;
                    else
                        id_rf_ra1 = ifid_ir[4:0];
                    id_ctl_ex.ALUOp = ifid_ir[13:10];
                    id_ctl_wb.RegWrite = '1;
                    if (id_ctl_ex.ALUOp != ALUOP_MOV) begin
                        id_ctl_ma.FlagMask = '1;
                        id_ctl_ma.FlagMask.Carry = (id_ctl_ex.ALUOp == ALUOP_ADD)
                            | (id_ctl_ex.ALUOp == ALUOP_SUB);
                    end
                end
            6'b000_110:             // JMP
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_ctl_ex.ALUOp = ALUOP_MOV;
                    id_ctl_ex.Branch = '1;
                    id_ctl_ex.Bcond[2:0] = BCOND_T;
                end
            6'b0?0_011:             // CMP
                begin
                    id_rf_ra2 = ifid_ir[9:5];
                    if (ifid_ir[14])
                        id_ctl_ex.ALUSrc1 = ALUSRC1_IMM5;
                    else
                        id_rf_ra1 = ifid_ir[4:0];
                    id_ctl_ex.ALUOp = ALUOP_SUB;
                    id_ctl_ma.FlagMask = '1;
                end
            6'b010_010:             // SETF
                begin
                    id_rf_wa = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc1 = ALUSRC1_BMATCH;
                    id_ctl_ex.ALUOp = ALUOP_MOV;
                    id_ctl_ex.Bcond = ifid_ir[3:0];
                    id_ctl_wb.RegWrite = '1;
                end
            6'b011_000:             // TRAP
                begin
                    id_trap = '1;
                end
            6'b011_001:             // RETI
                begin
                    if (id_ccnt == 'd0) begin
                        id_ctl_ex.ALUSrc1 = ALUSRC1_SR_RD;
                        id_ctl_ex.ALUOp = ALUOP_MOV;
                        id_ctl_ex.SRSelRead = PSW.np ? SRSEL_FEPC : SRSEL_EIPC;
                        id_ctl_ex.Branch = '1;
                        id_ctl_ex.Bcond[2:0] = BCOND_T;
                        id_ctl_ex.Extend = '1;
                    end
                    else if (id_ccnt == 'd1) begin
                        id_ctl_ex.ALUSrc1 = ALUSRC1_SR_RD;
                        id_ctl_ex.ALUOp = ALUOP_MOV;
                        id_ctl_ex.SRSelRead = PSW.np ? SRSEL_FEPSW : SRSEL_EIPSW;
                        id_ctl_ma.SRSelWrite = SRSEL_PSW;
                        id_ctl_ma.SRWrite = '1;
                    end
                end
            6'b011_010:             // HALT
                begin
                    id_rf_ra2 = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                    id_ctl_ex.ALUOp = ALUOP_MOV;
                    id_ctl_ex.SRSelRead = SRSEL_PSW;
                    id_ctl_ex.Halt = '1;
                    id_ctl_ma.FaultReq = '1;
                    id_ctl_ma.Write = '1;
                    id_ctl_ma.SRtoMem = '1;
                    id_ctl_wb.MemWidth = 2'd1; // halfword
                end
            6'b011_100:             // LDSR
                begin
                    id_rf_ra1 = ifid_ir[9:5];
                    id_ctl_ex.ALUOp = ALUOP_MOV;
                    id_ctl_ma.SRSelWrite = sr_sel_t'(ifid_ir[4:0]);
                    id_ctl_ma.SRWrite = '1;
                end
            6'b011_101:             // STSR
                begin
                    id_rf_wa = ifid_ir[9:5];
                    id_ctl_ex.SRSelRead = sr_sel_t'(ifid_ir[4:0]);
                    id_ctl_ma.SRtoReg = '1;
                    id_ctl_wb.RegWrite = '1;
                end
            6'b100_???:             // Bcond (branch)
                begin
                    id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                    id_ctl_ex.ALUSrc2 = ALUSRC2_DISP9;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ex.Branch = '1;
                    id_ctl_ex.Bcond = ifid_ir[12:9];
                end
            6'b101_000,             // MOVEA
                6'b101_001:             // ADDI
                    begin
                        id_rf_ra1 = ifid_ir[4:0];
                        id_rf_wa = ifid_ir[9:5];
                        id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                        id_ctl_ex.ALUOp = ALUOP_ADD;
                        id_ctl_wb.RegWrite = '1;
                        if (ifid_ir[10]) // ADDI
                            id_ctl_ma.FlagMask = '1;
                    end
            6'b101_10?,             // ORI, ANDI
                6'b101_110:             // XORI
                    begin
                        id_rf_ra1 = ifid_ir[4:0];
                        id_rf_wa = ifid_ir[9:5];
                        id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                        id_ctl_ex.ALUOp = ifid_ir[13:10];
                        id_ctl_wb.RegWrite = '1;
                        id_ctl_ma.FlagMask = '1;
                        id_ctl_ma.FlagMask.Carry = '0;
                    end
            6'b101_010:             // JR
                begin
                    id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                    id_ctl_ex.ALUSrc2 = ALUSRC2_DISP26;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ex.Branch = '1;
                    id_ctl_ex.Bcond[2:0] = BCOND_T;
                end
            6'b101_011:             // JAL
                begin
                    if (id_ccnt == 'd0) begin
                        id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                        id_ctl_ex.ALUSrc2 = ALUSRC2_DISP26;
                        id_ctl_ex.ALUOp = ALUOP_ADD;
                        id_ctl_ex.Branch = '1;
                        id_ctl_ex.Bcond[2:0] = BCOND_T;
                        id_ctl_ex.Extend = '1;
                    end
                    else if (id_ccnt == 'd1) begin
                        id_rf_wa = 5'd31;
                        id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                        id_ctl_ex.ALUSrc2 = ALUSRC2_CONST_4;
                        id_ctl_ex.ALUOp = ALUOP_ADD;
                        id_ctl_wb.RegWrite = '1;
                    end
                end
            6'b101_111:             // MOVHI
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_rf_wa = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16_HI;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_wb.RegWrite = '1;
                end
            6'b110_0??:             // LD
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_rf_wa = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ma.MemReq = '1;
                    id_ctl_wb.MemWidth = ifid_ir[11:10];
                    id_ctl_wb.MemtoReg = '1;
                    id_ctl_wb.RegWrite = '1;
                end
            6'b110_1??:             // ST
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_rf_ra2 = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ma.MemReq = '1;
                    id_ctl_ma.Write = '1;
                    id_ctl_wb.MemWidth = ifid_ir[11:10];
                end
            6'b111_0??:             // IN
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_rf_wa = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ma.IOReq = '1;
                    id_ctl_wb.MemWidth = ifid_ir[11:10];
                    id_ctl_wb.IOtoReg = '1;
                    id_ctl_wb.RegWrite = '1;
                end
            6'b111_1??:             // OUT
                begin
                    id_rf_ra1 = ifid_ir[4:0];
                    id_rf_ra2 = ifid_ir[9:5];
                    id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                    id_ctl_ex.ALUOp = ALUOP_ADD;
                    id_ctl_ma.IOReq = '1;
                    id_ctl_ma.Write = '1;
                    id_ctl_wb.MemWidth = ifid_ir[11:10];
                end
            default:
                id_invalid = '1;
        endcase

    if (id_rf_wa == '0)
        id_ctl_wb.RegWrite = '0; // no point in trying
end

assign id_done = ~id_flush & ~id_ctl_ex.Extend;

// Handle ins. with multiple EX cycles
always @(posedge CLK) if (CE) begin
    if (~RESn)
        id_ccnt <= '0;
    else if (~(id_stall | id_flush)) begin
        if (id_done)
            id_ccnt <= '0;
        else
            id_ccnt <= id_ccnt + 1'd1;
    end
end

// Decoded an invalid (or unimplemented) instruction
wire id_invalid_ins = id_invalid & ~id_stall;

assign if_stall = id_ctl_ex.Extend & ~(id_stall | id_flush);

// Avoid an IF fetch happening post-HALT.
assign if_stall = halted;
assign if_flush = id_ctl_ex.Halt | halted;
assign id_flush = halted;

//////////////////////////////////////////////////////////////////////
// Execution Unit exception generation

wor             ex_euf;
logic [15:0]    ex_euccb;
logic [4:0]     ex_eucco;

always @* begin
    ex_euccb = '0;
    ex_eucco = '0;

    if (id_trap) begin
        ex_euccb = 16'hFFA0;
        ex_eucco = ifid_ir[4:0];
    end
end

assign INEX_EUF = ex_euf;
assign INEX_EUCCB = ex_euccb;
assign INEX_EUCCO = ex_eucco;
assign INEX_ADTRF = '0; // TODO

//////////////////////////////////////////////////////////////////////
// Exception handling

wire id_exc_set_ecr_go = id_exc_set_ecr & ~(id_stall | id_flush);
assign ECR_SET_EICC = id_exc_set_ecr_go & (resh | ~INEX_NP) & (~PSW.ep & ~PSW.np);
assign ECR_SET_FECC = id_exc_set_ecr_go & ((~resh & INEX_NP) | (PSW.ep & ~PSW.np));

wire id_exc_set_psw_go = id_exc_set_psw & ~(id_stall | id_flush);
assign PSW_RESET.id = '0;
assign PSW_RESET.ae = id_exc_set_psw_go;
assign PSW_RESET.ep = '0;
assign PSW_RESET.np = '0;
assign PSW_RESET.i = {4{PSW_SET.ep}};
assign PSW_SET.id = id_exc_set_psw_go & ~resh; // Why is ID=0 on reset?
assign PSW_SET.ae = '0;
assign PSW_SET.ep = id_exc_set_psw_go & ~INEX_NP;
assign PSW_SET.np = id_exc_set_psw_go & INEX_NP;
assign PSW_SET.i = {4{PSW_SET.ep}} & INEX_IEL;

assign ex_euf = id_trap;
assign if_exc_done = ifid_exc & id_done;

//////////////////////////////////////////////////////////////////////
// Register file

logic [31:0]    id_rf_rd1, id_rf_rd2;
logic [4:0]     wb_rf_wa;
logic [31:0]    wb_rf_wd;
logic           wb_rf_we;

v810_regfile rf
    (
     .CLK(CLK),
     .CE(CE),

     .RA1(id_rf_ra1),
     .RD1(id_rf_rd1),

     .RA2(id_rf_ra2),
     .RD2(id_rf_rd2),

     .WA(wb_rf_wa),
     .WD(wb_rf_wd),
     .WE(wb_rf_we)
     );

//////////////////////////////////////////////////////////////////////
// ID/EX pipeline register

wire idex_ctl_zero = RESn & (id_flush);

always @(posedge CLK) if (CE) begin
    if (~RESn | ~id_stall) begin
        idex_pc <= ifid_pc;
        idex_ir <= ifid_ir;
        idex_exc <= ifid_exc;
        idex_rf_wa <= id_rf_wa;
        idex_rf_rd1 <= id_rf_rd1;
        idex_rf_rd2 <= id_rf_rd2;
        idex_ctl.ex <= idex_ctl_zero ? '0 : id_ctl_ex;
        idex_ctl.ma <= idex_ctl_zero ? '0 : id_ctl_ma;
        idex_ctl.wb <= idex_ctl_zero ? '0 : id_ctl_wb;
    end
end


//////////////////////////////////////////////////////////////////////
// EX - Execute / address calculation
//////////////////////////////////////////////////////////////////////

logic           bcond_match;
logic [31:0]    ex_sr_rd;

//////////////////////////////////////////////////////////////////////
// ALU

logic [31:0]    alu_in1, alu_in2;
logic [31:0]    alu_out;
aluop_t         alu_op;
aluflags_t      alu_fl;

wire            alu_se = (alu_op < ALUOP_SHL);

wire [4:0]      idex_imm5 = idex_ir[4:0];
wire [15:0]     idex_imm16 = idex_ir[31:16];

wire [31:0]     idex_disp9 = 32'($signed(idex_ir[8:0]));
wire [31:0]     idex_imm5_ext = alu_se ? 32'($signed(idex_imm5)) : 32'(idex_imm5);
wire [31:0]     idex_imm16_hi = {idex_ir[31:16], 16'b0};
wire [31:0]     idex_imm16_ext = alu_se ? 32'($signed(idex_imm16)) : 32'(idex_imm16);
wire [31:0]     idex_disp26 = 32'($signed({idex_ir[9:0], idex_ir[31:16]}));

always @* begin
    alu_in1 = 'X;
    case (idex_ctl.ex.ALUSrc1)
        ALUSRC1_RF_RD1: alu_in1 = idex_rf_rd1;
        ALUSRC1_IMM5:   alu_in1 = idex_imm5_ext;
        ALUSRC1_PC:     alu_in1 = idex_pc;
        ALUSRC1_BMATCH: alu_in1 = {31'b0, bcond_match};
        ALUSRC1_SR_RD:  alu_in1 = ex_sr_rd;
        ALUSRC1_EXC_A:  alu_in1 = INEX_HA;
        default: ;
    endcase
end

always @* begin
    alu_in2 = 'X;
    case (idex_ctl.ex.ALUSrc2)
        ALUSRC2_RF_RD2:     alu_in2 = idex_rf_rd2;
        ALUSRC2_DISP9:      alu_in2 = idex_disp9;
        ALUSRC2_IMM16:      alu_in2 = idex_imm16_ext;
        ALUSRC2_IMM16_HI:   alu_in2 = idex_imm16_hi;
        ALUSRC2_DISP26:     alu_in2 = idex_disp26;
        ALUSRC2_CONST_4:    alu_in2 = 32'd4;
        default: ;
    endcase
end

assign alu_op = aluop_t'(idex_ctl.ex.ALUOp);

always @* begin
    alu_fl = '0;
    case (alu_op)
        ALUOP_MOV:
            alu_out = alu_in1;
        ALUOP_ADD: begin
            {alu_fl.Carry, alu_out} = alu_in2 + alu_in1;
            alu_fl.Over = (alu_in1[31] == alu_in2[31])
                & (alu_in2[31] != alu_out[31]);
        end
        ALUOP_SUB: begin
            {alu_fl.Carry, alu_out} = alu_in2 - alu_in1;
            alu_fl.Over = (alu_in1[31] != alu_in2[31])
                & (alu_in2[31] != alu_out[31]);
        end
        ALUOP_SHL:
            {alu_fl.Carry, alu_out} = 33'(alu_in2) << alu_in1;
        ALUOP_SHR:
            {alu_out, alu_fl.Carry} = {alu_in2, 1'b0} >> alu_in1;
        ALUOP_SAR:
            {alu_out, alu_fl.Carry} = $signed({alu_in2, 1'b0}) >>> alu_in1;
        ALUOP_OR:
            alu_out = alu_in1 | alu_in2;
        ALUOP_AND:
            alu_out = alu_in1 & alu_in2;
        ALUOP_XOR:
            alu_out = alu_in1 ^ alu_in2;
        ALUOP_NOT:
            alu_out = ~alu_in1;
        default:
            alu_out = 'X;
    endcase
    alu_fl.Zero = ~|alu_out;
    alu_fl.Sign = alu_out[31];
end

//////////////////////////////////////////////////////////////////////
// Branch condition test

aluflags_t psw_alu_fl;
assign psw_alu_fl = PSW.alu_fl;

always @* begin
    case (idex_ctl.ex.Bcond[2:0])
        BCOND_V:               // Overflow
            bcond_match = psw_alu_fl.Over;
        BCOND_C:               // Carry / Lower
            bcond_match = psw_alu_fl.Carry;
        BCOND_Z:               // Zero / Equal
            bcond_match = psw_alu_fl.Zero;
        BCOND_NH:              // Not higher
            bcond_match = psw_alu_fl.Carry | psw_alu_fl.Zero;
        BCOND_S:               // Negative
            bcond_match = psw_alu_fl.Sign;
        BCOND_T:               // Always
            bcond_match = '1; 
        BCOND_LT:              // Less than signed
            bcond_match = psw_alu_fl.Sign ^ psw_alu_fl.Over;
        BCOND_LTE:             // Less than or equal signed
            bcond_match = (psw_alu_fl.Sign ^ psw_alu_fl.Over) | psw_alu_fl.Zero;
    endcase
    // MSB inverts the test.
    bcond_match ^= idex_ctl.ex.Bcond[3];
end

// On branch taken, set PC and flush pipeline before EX.
wire branch_taken = idex_ctl.ex.Branch & bcond_match;
// For JAL: EX-1 takes branch, EX-2 computes r31. So, don't flush ID on branch.
wire branch_no_id_flush = idex_ctl.ex.Extend;
assign if_pc_set = branch_taken & ~ex_stall;
assign if_pc_set_val = alu_out;
assign if_flush = branch_taken;
assign id_flush = branch_taken & ~branch_no_id_flush;

//////////////////////////////////////////////////////////////////////
// Special stuff

// System register read
assign SR_RA = idex_ctl.ex.SRSelRead;
assign ex_sr_rd = SR_RD;

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        halted <= '0;
    end
    else begin
        if (idex_ctl.ex.Halt) begin
            halted <= '1;
        end
    end
end

// Avoid an IF fetch happening post-HALT.
assign if_stall = idex_ctl.ex.Halt;

// Start of exception execution
assign INEX_ACK = idex_ctl.ex.AckExcept;


//////////////////////////////////////////////////////////////////////
// EX/MA pipeline register

wire exma_ctl_flush = RESn & (ex_flush);

always @(posedge CLK) if (CE) begin
    if (~RESn | ~ex_stall) begin
        exma_rf_wa <= idex_rf_wa;
        exma_rf_rd2 <= idex_rf_rd2;
        exma_alu_out <= alu_out;
        exma_alu_fl <= alu_fl;
        exma_sr_rd <= ex_sr_rd;
        exma_ctl.ma <= exma_ctl_flush ? '0 : idex_ctl.ma;
        exma_ctl.wb <= exma_ctl_flush ? '0 : idex_ctl.wb;
    end
end


//////////////////////////////////////////////////////////////////////
// MA - Memory Access
//////////////////////////////////////////////////////////////////////

logic [31:0]    ma_di, ma_do;
logic [3:0]     ma_be;

wire [1:0]  ma_bsel = DA[1:0];
wire        ma_hsel = DA[1];
wire        ma_bes = DREQ;

always @* begin
    ma_be = '0;
    case (exma_ctl.wb.MemWidth)
        2'b00:
            ma_be[ma_bsel] = ma_bes;
        2'b01:
            ma_be[ma_hsel*2+:2] = {2{ma_bes}};
        default: // 2'b11
            ma_be = {4{ma_bes}};
    endcase
end

always @* begin
    case (exma_ctl.wb.MemWidth)
        2'b00:
            ma_do = {4{exma_rf_rd2[7:0]}};
        2'b01:
            ma_do = {2{exma_rf_rd2[15:0]}};
        default: // 2'b11
            ma_do = exma_rf_rd2;
    endcase
end

assign DA = exma_ctl.ma.SRtoMem ? exma_sr_rd : exma_alu_out;
assign ma_di = DD_I;
assign DD_O = ma_do;
assign DBC = exma_ctl.wb.MemWidth;
assign DBE = ma_be;

assign DWR = DREQ & exma_ctl.ma.Write;
assign DMRQ = exma_ctl.ma.MemReq;
assign DST[0] = exma_ctl.ma.FaultReq;
assign DST[1] = exma_ctl.ma.MemReq | exma_ctl.ma.IOReq |
                exma_ctl.ma.FaultReq & (halted /*& ~fault*/);

assign DREQ = (exma_ctl.ma.MemReq | exma_ctl.ma.IOReq | exma_ctl.ma.FaultReq);

// Stall pipeline while memory access completes
wire ma_incomplete = DREQ & ~DACK;
assign if_stall = ma_incomplete;
assign id_stall = ma_incomplete;
assign ex_stall = ma_incomplete;
assign ma_flush = ma_incomplete;

//////////////////////////////////////////////////////////////////////
// System Register Write back

aluflags_t psw_reset_alu_fl, psw_set_alu_fl;

// PSW ALU flags
assign PSW_RESET.alu_fl = exma_ctl.ma.FlagMask & ~exma_alu_fl;
assign PSW_SET.alu_fl   = exma_ctl.ma.FlagMask & exma_alu_fl;

// Placeholder for remaining PSW fields
assign PSW_RESET.rfu20 = '0;
assign PSW_RESET.rfu10 = '0;
assign PSW_RESET.float_fl = '0;
assign PSW_SET.rfu20 = '0;
assign PSW_SET.rfu10 = '0;
assign PSW_SET.float_fl = '0;

// System register write
assign SR_WA = exma_ctl.ma.SRSelWrite;
assign SR_WD = exma_alu_out;
assign SR_WE = exma_ctl.ma.SRWrite;

//////////////////////////////////////////////////////////////////////
// MA/WB pipeline register

wire mawb_ctl_flush = RESn & (ma_flush);

always @(posedge CLK) if (CE) begin
    mawb_rf_wa <= exma_rf_wa;
    mawb_alu_out <= exma_ctl.ma.SRtoReg ? exma_sr_rd : exma_alu_out;
    mawb_mem_rd <= ma_di;
    mawb_mem_bsel <= ma_bsel;
    mawb_ctl.wb <= mawb_ctl_flush ? '0 : exma_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// WB - Write back
//////////////////////////////////////////////////////////////////////

logic [31:0]    wb_ldmem;

wire [1:0] wb_mbsel = mawb_mem_bsel;
wire       wb_mhsel = wb_mbsel[1];

// Sign-extend memory read, zero-extend IO read
always @* begin
    case (mawb_ctl.wb.MemWidth)
        2'b00: begin
            wb_ldmem[7:0] = mawb_mem_rd[wb_mbsel*8+:8];
            wb_ldmem[31:8] = {24{wb_ldmem[7] && mawb_ctl.wb.MemtoReg}};
        end
        2'b01: begin
            wb_ldmem[15:0] = mawb_mem_rd[wb_mhsel*16+:16];
            wb_ldmem[31:16] = {16{wb_ldmem[15] && mawb_ctl.wb.MemtoReg}};
        end
        default: // 2'b11
            wb_ldmem = mawb_mem_rd;
    endcase
end

wire wb_memio_to_reg = mawb_ctl.wb.MemtoReg | mawb_ctl.wb.IOtoReg;

assign wb_rf_wa = mawb_rf_wa;
assign wb_rf_wd = wb_memio_to_reg ? wb_ldmem : mawb_alu_out;
assign wb_rf_we = mawb_ctl.wb.RegWrite & |wb_rf_wa;


//////////////////////////////////////////////////////////////////////
// Hazard Detection
//////////////////////////////////////////////////////////////////////

// Data / Register Hazard
wire haz_data_ex = idex_ctl.wb.RegWrite & |idex_rf_wa &
     ((idex_rf_wa == id_rf_ra1) | (idex_rf_wa == id_rf_ra2));

wire haz_data_ma = exma_ctl.wb.RegWrite & |exma_rf_wa &
     ((exma_rf_wa == id_rf_ra1) | (exma_rf_wa == id_rf_ra2));

// Note: Because reg. writes in WB are forwarded to reads in ID, there
// is no data hazard in WB.

wire haz_data = haz_data_ex | haz_data_ma;

// Flag Hazard
wire haz_bcond = id_ctl_ex.Branch & (id_ctl_ex.Bcond[2:0] != BCOND_T);
wire haz_setf = id_ctl_ex.ALUSrc1 == ALUSRC1_BMATCH;
wire haz_flag_ex = |idex_ctl.ma.FlagMask & (haz_bcond | haz_setf);

wire haz_flag = haz_flag_ex;

// System Hazard
//
// (1) Because LDSR takes effect immediately, i.e., is observed by the
// next instruction, there is a system hazard until LDSR clears MA.

wire haz_sys1_ex = idex_ctl.ma.SRWrite;
wire haz_sys1_ma = exma_ctl.ma.SRWrite;
wire haz_sys1 = haz_sys1_ex | haz_sys1_ma;

// (2) IN must reach WB before EX can start the next instruction. In
// other words, IN must be atomic for interrupts / exceptions;
// restarting it could have unwanted side effects.

wire haz_sys2_ex = idex_ctl.wb.IOtoReg;
wire haz_sys2_ma = exma_ctl.wb.IOtoReg;
wire haz_sys2 = haz_sys2_ex | haz_sys2_ma;

wire haz_sys = haz_sys1 | haz_sys2;

wire branch_incomplete = if_pc_set/* | if_pc_set_ins_fetch*/;
assign if_stall = (haz_data | haz_flag | haz_sys) & ~branch_incomplete;
assign id_flush = (haz_data | haz_flag | haz_sys);

endmodule
