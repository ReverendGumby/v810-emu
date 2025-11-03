`timescale 1us / 1ns

// Instruction execution unit

// TODO: These instructions:
// - MUL(U), DIV(U)
// - TRAP, RETI, HALT
// - Bit string manipulation (Bstr)
// - LDSR, STSR
// - Floating-point operation (Fpp)
// - IN, OUT
// - CAXI

module v810_exec
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Instruction bus
   output [31:0] IA,
   input [31:0]  ID,
   output        IREQ, // Access request
   input         IACK, // Access acknowledge

   // Data bus
   output [31:0] DA,
   input [31:0]  DD_I,
   output [31:0] DD_O,
   output [3:0]  DBE, // Byte Enable
   output        DWR, // Write / not Read
   output        DREQ, // Access request
   input         DACK, // Access acknowledge

   output [1:0]  ST // Status
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

typedef struct packed {
    logic Carry;
    logic Over;
    logic Sign;
    logic Zero;
} aluflags_t;

wor             if_stall, if_flush;
wor             id_stall, id_flush;
wor             ex_stall, ex_flush;
wor             ma_flush;

event halt;


//////////////////////////////////////////////////////////////////////
// System Registers

aluflags_t      psw;    // TODO: More flags to come


//////////////////////////////////////////////////////////////////////
// Pipeline registers

typedef enum bit [1:0] {
    ALUSRC1_RF_RD1 = 2'd0,
    ALUSRC1_IMM5,
    ALUSRC1_PC,
    ALUSRC1_BMATCH
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
    logic       Halt; // TODO: there's gotta be a better way
} ctl_ex_t;

typedef struct packed {
    logic       MemRead;
    logic       MemWrite;
    aluflags_t  FlagMask;
} ctl_ma_t;

typedef struct packed {
    logic       RegWrite;
    logic       MemtoReg;
    logic [1:0] MemWidth;
} ctl_wb_t;

// IF/ID
logic [31:0]    ifid_pc;
logic [31:0]    ifid_ir;

// ID/EX
logic [31:0]    idex_pc;
logic [31:0]    idex_ir;
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

logic           if_ins32_fetch_hi;

//////////////////////////////////////////////////////////////////////
// Instruction memory interface

logic [31:0]    imi_a;
logic [31:0]    imi_d;
logic [15:0]    idrh;   // last high halfword fetched

assign IA = imi_a;
assign imi_d = ID;

always @(posedge CLK) if (CE) begin
    if (~if_stall)
        idrh <= imi_d[31:16];
end

//////////////////////////////////////////////////////////////////////
// Program Counter

logic           if_pc_inc, if_pc_inc4, if_pc_set;
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
        pc <= 32'hFFFFFFF0;
    end
    else if (~(if_stall | if_ins32_fetch_hi)) begin
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
    casex (ins[15:10])
        6'b101xxx,              // Format IV, V
        6'b110x0x, 6'b110x11,   // Format VI
        6'b1110xx, 6'b11110x,   // Format VI
        6'b111111:              // Format VI
            if_is_ins32 = '1;
        default:
            if_is_ins32 = '0;
    endcase
endfunction

always @(posedge CLK) if (CE) begin
    if (~RESn)
        imi_a <= pc;
    else if (~if_stall) begin
        if (if_imi_a2 & ~if_pc_set)
            imi_a <= pcn + 2'd2;
        else
            imi_a <= pcn;
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

    casex ({pc[1], if_wrapped, if_pdhi_ins32, if_pdlo_ins32})
        4'b0000,
        4'b1000,
        4'b110x:
            ;
        4'b00x1:    begin
            if_pc_inc4 = '1;
        end
        4'b0010:    begin
            if_wrap = '1;
        end
        4'b101x:    begin
            if_wrap = '1;
            if_ins32_fetch_hi = '1;
        end
        4'b111x:    begin
            if_pc_inc4 = '1;
            if_wrap = '1;
        end
        default:    // invalid state
            assert(0);
    endcase
end

assign if_ir_swap = pc[1];
assign if_imi_a2 = if_wrap;
assign if_pc_inc = ~if_ins32_fetch_hi;

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        if_wrapped <= '0;
    end
    else if (~if_stall) begin
        if_wrapped <= if_wrap & ~if_pc_set;
    end
end

//////////////////////////////////////////////////////////////////////
// Instruction Register

logic [31:0]    ir;

always @* begin
    ir = pd;
    if (if_ir_swap)
        ir = {ir[15:0], ir[31:16]};
    if (if_wrapped)
        ir[15:0] = idrh;
    if (if_flush)
        ir = '0;
end

//////////////////////////////////////////////////////////////////////
// IF/ID pipeline register

always @(posedge CLK) if (CE) begin
    if (~RESn | ~if_stall) begin
        ifid_pc <= pc;
        ifid_ir <= ir;
    end
end


//////////////////////////////////////////////////////////////////////
// ID - Instruction Decode / Register Fetch stage
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Instruction decoder

logic [4:0]     id_rf_ra1, id_rf_ra2, id_rf_wa;
ctl_ex_t        id_ctl_ex;
ctl_ma_t        id_ctl_ma;
ctl_wb_t        id_ctl_wb;
logic [5:0]     id_ccnt;

always @* begin
    id_rf_ra1 = '0;
    id_rf_ra2 = '0;
    id_rf_wa = '0;
    id_ctl_ex = '0;
    id_ctl_ma = '0;
    id_ctl_wb = '0;

    casex (ifid_ir[15:10])
        6'b0x0_00x,             // MOV, ADD
        6'b000_010,             // SUB
        6'b0x0_10x,             // SHL, SHR
        6'b0x0_111,             // SAR
        6'b001_10x,             // OR, AND
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
        6'b0x0_011:             // CMP
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
        6'b011_010:             // HALT
            id_ctl_ex.Halt = '1;
        6'b100_xxx:             // Bcond (branch)
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
        6'b101_10x,             // ORI, ANDI
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
        6'b110_0xx:             // LD
            begin
                id_rf_ra1 = ifid_ir[4:0];
                id_rf_wa = ifid_ir[9:5];
                id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_ma.MemRead = '1;
                id_ctl_wb.MemWidth = ifid_ir[11:10];
                id_ctl_wb.MemtoReg = '1;
                id_ctl_wb.RegWrite = '1;
            end
        6'b110_1xx:             // ST
            begin
                id_rf_ra1 = ifid_ir[4:0];
                id_rf_ra2 = ifid_ir[9:5];
                id_ctl_ex.ALUSrc2 = ALUSRC2_IMM16;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_ma.MemWrite = '1;
                id_ctl_wb.MemWidth = ifid_ir[11:10];
            end
        default: ;
    endcase
    if (id_rf_wa == '0)
        id_ctl_wb.RegWrite = '0; // no point in trying
end

// Handle ins. with multiple EX cycles
always @(posedge CLK) if (CE) begin
    if (~RESn)
        id_ccnt <= '0;
    else if (~id_stall) begin
        if (~id_ctl_ex.Extend)
            id_ccnt <= '0;
        else
            id_ccnt <= id_ccnt + 1'd1;
    end
end

assign if_stall = id_ctl_ex.Extend;

//////////////////////////////////////////////////////////////////////
// Register file

logic [31:0]    rmem [32];
logic [4:0]     rf_ra1, rf_ra2;
logic [31:0]    rf_rd1, rf_rd2;
logic           rf_re1, rf_re2;
logic [4:0]     rf_wa;
logic [31:0]    rf_wd;
logic           rf_we;

initial begin
    rmem[0] = '0;
end

assign rf_ra1 = id_rf_ra1;
assign rf_ra2 = id_rf_ra2;

// Use asynch. logic to forward writes to the read ports.
always @* begin
    if (rf_we & (rf_ra1 == rf_wa))
        rf_rd1 = rf_wd;
    else
        rf_rd1 = rmem[rf_ra1];
end

always @* begin
    if (rf_we & (rf_ra2 == rf_wa))
        rf_rd2 = rf_wd;
    else
        rf_rd2 = rmem[rf_ra2];
end

always @(posedge CLK) if (CE) begin
    if (rf_we)
        rmem[rf_wa] <= rf_wd;
end

// Debugging aid for .vcd
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

//////////////////////////////////////////////////////////////////////
// ID/EX pipeline register

wire idex_ctl_zero = RESn & (id_flush);

always @(posedge CLK) if (CE) begin
    if (~RESn | ~id_stall) begin
        idex_pc <= ifid_pc;
        idex_ir <= ifid_ir;
        idex_rf_wa <= id_rf_wa;
        idex_rf_rd1 <= rf_rd1;
        idex_rf_rd2 <= rf_rd2;
        idex_ctl.ex <= idex_ctl_zero ? '0 : id_ctl_ex;
        idex_ctl.ma <= idex_ctl_zero ? '0 : id_ctl_ma;
        idex_ctl.wb <= idex_ctl_zero ? '0 : id_ctl_wb;
    end
end


//////////////////////////////////////////////////////////////////////
// EX - Execute / address calculation
//////////////////////////////////////////////////////////////////////

logic           bcond_match;

assign ex_flush = '0;

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

always @* begin
    case (idex_ctl.ex.Bcond[2:0])
        BCOND_V:               // Overflow
            bcond_match = psw.Over;
        BCOND_C:               // Carry / Lower
            bcond_match = psw.Carry;
        BCOND_Z:               // Zero / Equal
            bcond_match = psw.Zero;
        BCOND_NH:              // Not higher
            bcond_match = psw.Carry | psw.Zero;
        BCOND_S:               // Negative
            bcond_match = psw.Sign;
        BCOND_T:               // Always
            bcond_match = '1; 
        BCOND_LT:              // Less than signed
            bcond_match = psw.Sign ^ psw.Over;
        BCOND_LTE:             // Less than or equal signed
            bcond_match = (psw.Sign ^ psw.Over) | psw.Zero;
    endcase
    // MSB inverts the test.
    bcond_match ^= idex_ctl.ex.Bcond[3];
end

// On branch taken, set PC and flush pipeline before EX.
wire branch_taken = idex_ctl.ex.Branch & bcond_match;
// For JAL: EX-1 takes branch, EX-2 computes r31. So, don't flush ID on branch.
wire branch_no_id_flush = idex_ctl.ex.Extend;
assign if_pc_set = branch_taken;
assign if_pc_set_val = alu_out;
assign if_flush = branch_taken;
assign id_flush = branch_taken & ~branch_no_id_flush;

//////////////////////////////////////////////////////////////////////
// Special stuff

always @(posedge CLK) if (CE) begin
    if (idex_ctl.ex.Halt) begin
        // TODO: Emit a halt acknowledge cycle
        -> halt;
    end
end


//////////////////////////////////////////////////////////////////////
// EX/MA pipeline register

wire exma_ctl_flush = RESn & (ex_flush);

always @(posedge CLK) if (CE) begin
    if (~RESn | ~ex_stall) begin
        exma_rf_wa <= idex_rf_wa;
        exma_rf_rd2 <= idex_rf_rd2;
        exma_alu_out <= alu_out;
        exma_alu_fl <= alu_fl;
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

assign DA = exma_alu_out;
assign ma_di = DD_I;
assign DD_O = ma_do;
assign DBE = ma_be;

assign DREQ = exma_ctl.ma.MemRead | exma_ctl.ma.MemWrite;
assign DWR = exma_ctl.ma.MemWrite;

always @(posedge CLK) if (CE) begin
    if (~RESn)
        psw <= '0;
    else
        psw <= (psw & ~exma_ctl.ma.FlagMask) |
               (exma_alu_fl & exma_ctl.ma.FlagMask);
end

// Stall pipeline while memory access completes
wire ma_incomplete = DREQ & ~DACK;
assign if_stall = ma_incomplete;
assign id_stall = ma_incomplete;
assign ex_stall = ma_incomplete;
assign ma_flush = ma_incomplete;

//////////////////////////////////////////////////////////////////////
// MA/WB pipeline register

wire mawb_ctl_flush = RESn & (ma_flush);

always @(posedge CLK) if (CE) begin
    mawb_rf_wa <= exma_rf_wa;
    mawb_alu_out <= exma_alu_out;
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

// Sign-extend memory read
always @* begin
    case (mawb_ctl.wb.MemWidth)
        2'b00: begin
            wb_ldmem[7:0] = mawb_mem_rd[wb_mbsel*8+:8];
            wb_ldmem[31:8] = {24{wb_ldmem[7]}};
        end
        2'b01: begin
            wb_ldmem[15:0] = mawb_mem_rd[wb_mhsel*16+:16];
            wb_ldmem[31:16] = {16{wb_ldmem[15]}};
        end
        default: // 2'b11
            wb_ldmem = mawb_mem_rd;
    endcase
end

assign rf_wa = mawb_rf_wa;
assign rf_wd = mawb_ctl.wb.MemtoReg ? wb_ldmem : mawb_alu_out;
assign rf_we = mawb_ctl.wb.RegWrite & |rf_wa;


//////////////////////////////////////////////////////////////////////
// Hazard Detection
//////////////////////////////////////////////////////////////////////

// Data / Register Hazard
wire haz_data_ex = idex_ctl.wb.RegWrite & |idex_rf_wa &
     ((idex_rf_wa == rf_ra1) | (idex_rf_wa == rf_ra2));

wire haz_data_ma = exma_ctl.wb.RegWrite & |exma_rf_wa &
     ((exma_rf_wa == rf_ra1) | (exma_rf_wa == rf_ra2));

// Note: Because reg. writes in WB are forwarded to reads in ID, there
// is no data hazard in WB.

wire haz_data = haz_data_ex | haz_data_ma;

// Flag Hazard
wire haz_bcond = id_ctl_ex.Branch & (id_ctl_ex.Bcond[2:0] != BCOND_T);
wire haz_setf = id_ctl_ex.ALUSrc1 == ALUSRC1_BMATCH;
wire haz_flag_ex = |idex_ctl.ma.FlagMask & (haz_bcond | haz_setf);

wire haz_flag = haz_flag_ex;

assign if_stall = (haz_data | haz_flag) & ~branch_taken;
assign id_flush = (haz_data | haz_flag);

endmodule
