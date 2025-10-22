`timescale 1us / 1ns

module v810_exec
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Instruction bus
   output [31:0] IA,
   input [31:0]  ID,

   // Data bus
   output [31:0] DA,
   input [31:0]  DD_I,
   output [31:0] DD_O,
   output [3:0]  BEn, // Byte Enable

   output [1:0]  ST, // Status
   output        MRQn, // Memory ReQuest
   output        RW // Read / Write
   );


//////////////////////////////////////////////////////////////////////
// Forward declarations

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

event halt;


//////////////////////////////////////////////////////////////////////
// Pipeline registers

typedef enum bit [1:0] {
    ALUSRC1_RF_RD1 = 2'd0,
    ALUSRC1_IMM5
} alu_src1_t;

typedef enum bit [1:0] {
    ALUSRC2_RF_RD2 = 2'd0,
    ALUSRC2_DISP16
} alu_src2_t;

typedef struct packed {
    logic RegDst;
    logic [3:0] ALUOp;
    alu_src1_t ALUSrc1;
    alu_src2_t ALUSrc2;
} ctl_ex_t;

typedef struct packed {
    logic Branch;
    logic MemRead;
    logic MemWrite;
} ctl_mem_t;

typedef struct packed {
    logic RegWrite;
    logic MemtoReg;
} ctl_wb_t;

// IF/ID
logic [31:0]    ifid_pc;
logic [31:0]    ifid_ir;

// ID/EX
logic [31:0]    idex_pc;
logic [31:0]    idex_imm;
logic [15:0]    idex_disp16;
logic [31:0]    idex_rf_rd1, idex_rf_rd2;
logic [4:0]     idex_rf_wa;
struct packed {
    ctl_ex_t    ex;
    ctl_mem_t   mem;
    ctl_wb_t    wb;
} idex_ctl;

// EX/MEM
logic [4:0]     exmem_rf_wa;
logic [31:0]    exmem_rf_rd2;
logic [31:0]    exmem_alu_out;
struct packed {
    ctl_mem_t   mem;
    ctl_wb_t    wb;
} exmem_ctl;

// MEM/WB
logic [4:0]     memwb_rf_wa;
logic [31:0]    memwb_alu_out;
logic [31:0]    memwb_mem_rd;
struct packed {
    ctl_wb_t    wb;
} memwb_ctl;


//////////////////////////////////////////////////////////////////////
// IF - Instruction Fetch stage
//////////////////////////////////////////////////////////////////////

logic           if_ins32;
logic           if_ins32_wrap;
logic           if_ins32_fetch_hi;

//////////////////////////////////////////////////////////////////////
// Instruction memory interface

logic [31:0]    imi_a;
logic [31:0]    imi_d;

assign IA = imi_a;
assign imi_d = ID;

//////////////////////////////////////////////////////////////////////
// Program Counter

logic           if_pc_inc, if_pc_inc4, if_pc_set;
logic [31:0]    if_pc_set_val;

logic [31:0]    pc, pci, pcn;

assign if_pc_inc4 = if_ins32;

always @* begin
    pci = pc + (if_pc_inc4 ? 32'd4 : 32'd2);
end

always @* begin
    pcn = pc;
    if (if_pc_inc)
        pcn = pci;
    else if (if_pc_set)
        pcn = if_pc_set_val;
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        pc <= '0;
    end
    else begin
        pc <= pcn;
    end
end

assign imi_a = if_ins32_fetch_hi ? pci : pc;

//////////////////////////////////////////////////////////////////////
// Instruction Pre-Decode

logic [31:0]    pd;

always @* begin
    if (~RESn) begin
        pd = 16'h9A00;          // NOP
    end
    else begin
        pd = imi_d;
        if (if_ins32_fetch_hi)
            pd = {pd[15:0], ifid_ir[31:16]};
        else if (pc[1])
            pd = {pd[15:0], pd[31:16]};
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

assign if_ins32 = if_is_ins32(pd[15:0]);

//////////////////////////////////////////////////////////////////////
// 32-bit fetch handling

// TODO: Silicon probably optimizes this to pre-fetch wrapped 32-bit ins.

assign if_ins32_wrap = if_ins32 & imi_a[1];
assign if_pc_inc = ~(if_ins32_wrap & ~if_ins32_fetch_hi);

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        if_ins32_fetch_hi <= '0;
    end
    else begin
        if_ins32_fetch_hi <= if_ins32_wrap & ~if_ins32_fetch_hi;
    end
end

//////////////////////////////////////////////////////////////////////
// Instruction Register

logic [31:0]    ir;

always @* begin
    ir = pd;
    if (~if_pc_inc)
        ir = {ir[15:0], 16'h9A00}; // save LO in IF/ID reg
end

//////////////////////////////////////////////////////////////////////
// IF/ID pipeline register

always @(posedge CLK) if (CE) begin
    ifid_pc <= pc;
    ifid_ir <= ir;
end


//////////////////////////////////////////////////////////////////////
// ID - Instruction Decode / Register Fetch stage
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Instruction decoder

logic [4:0]     id_rf_ra1, id_rf_ra2, id_rf_wa;
ctl_ex_t        id_ctl_ex;
ctl_mem_t       id_ctl_mem;
ctl_wb_t        id_ctl_wb;

assign id_rf_ra1 = ifid_ir[4:0];
assign id_rf_ra2 = ifid_ir[9:5];
assign id_rf_wa = ifid_ir[9:5];

always @* begin
    id_ctl_ex = '0;
    id_ctl_mem = '0;
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
                if (ifid_ir[14])
                    id_ctl_ex.ALUSrc1 = ALUSRC1_IMM5;
                id_ctl_ex.ALUOp = ifid_ir[13:10];
                id_ctl_wb.RegWrite = '1;
            end
        6'b011_010:             // HALT
            // TODO: Emit a halt acknowledge cycle
            -> halt;
        6'b110_0xx:             // LD
            begin
                id_ctl_ex.ALUSrc2 = ALUSRC2_DISP16;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_mem.MemRead = '1;
                id_ctl_wb.MemtoReg = '1;
                id_ctl_wb.RegWrite = '1;
            end
        6'b110_1xx:             // ST
            begin
                id_ctl_ex.ALUSrc2 = ALUSRC2_DISP16;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_mem.MemWrite = '1;
            end
        default: ;
    endcase
end

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

always @(posedge CLK) if (CE) begin
    rf_rd1 <= rmem[rf_ra1];
end

always @(posedge CLK) if (CE) begin
    rf_rd2 <= rmem[rf_ra2];
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

always @(posedge CLK) if (CE) begin
    idex_pc <= ifid_pc;
    idex_imm <= 32'($signed(ifid_ir[4:0]));
    idex_disp16 <= 16'($signed(ifid_ir[31:16]));
    idex_rf_wa <= id_rf_wa;
    idex_ctl.ex <= id_ctl_ex;
    idex_ctl.mem <= id_ctl_mem;
    idex_ctl.wb <= id_ctl_wb;
end

always @* begin
    idex_rf_rd1 = rf_rd1;
    idex_rf_rd2 = rf_rd2;
end


//////////////////////////////////////////////////////////////////////
// EX - Execute / address calculation
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// ALU

logic [31:0]    alu_in1, alu_in2;
logic [31:0]    alu_out;
aluop_t         alu_op;

always @* begin
    alu_in1 = 'X;
    case (idex_ctl.ex.ALUSrc1)
        ALUSRC1_RF_RD1: alu_in1 = idex_rf_rd1;
        ALUSRC1_IMM5:   alu_in1 = idex_imm;
        default: ;
    endcase
end

always @* begin
    alu_in2 = 'X;
    case (idex_ctl.ex.ALUSrc2)
        ALUSRC2_RF_RD2: alu_in2 = idex_rf_rd2;
        ALUSRC2_DISP16: alu_in2 = idex_disp16;
        default: ;
    endcase
end

assign alu_op = aluop_t'(idex_ctl.ex.ALUOp);

always @* begin
    case (alu_op)
        ALUOP_MOV:
            alu_out = alu_in1;
        ALUOP_ADD:
            alu_out = alu_in2 + alu_in1;
        ALUOP_SUB:
            alu_out = alu_in2 - alu_in1;
        ALUOP_SHL:
            alu_out = alu_in2 << alu_in1;
        ALUOP_SHR:
            alu_out = alu_in2 >> alu_in1;
        ALUOP_SAR:
            alu_out = alu_in2 >>> alu_in1;
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
end

//////////////////////////////////////////////////////////////////////
// EX/MEM pipeline register

always @(posedge CLK) if (CE) begin
    exmem_rf_wa <= idex_rf_wa;
    exmem_rf_rd2 <= idex_rf_rd2;
    exmem_alu_out <= alu_out;
    exmem_ctl.mem <= idex_ctl.mem;
    exmem_ctl.wb <= idex_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// MEM - Memory access
//////////////////////////////////////////////////////////////////////

logic [31:0]    mem_di;

assign if_pc_set = '0;
assign if_pc_set_val = exmem_alu_out;

assign DA = exmem_alu_out;
assign mem_di = DD_I;
assign DD_O = exmem_rf_rd2;

assign MRQn = ~(exmem_ctl.mem.MemRead | exmem_ctl.mem.MemWrite);
assign RW = MRQn | exmem_ctl.mem.MemRead;

//////////////////////////////////////////////////////////////////////
// MEM/WB pipeline register

always @(posedge CLK) if (CE) begin
    memwb_rf_wa <= exmem_rf_wa;
    memwb_alu_out <= exmem_alu_out;
    memwb_mem_rd <= mem_di;
    memwb_ctl.wb <= exmem_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// WB - Write back
//////////////////////////////////////////////////////////////////////

assign rf_wa = memwb_rf_wa;
assign rf_wd = memwb_ctl.wb.MemtoReg ? memwb_mem_rd : memwb_alu_out;
assign rf_we = memwb_ctl.wb.RegWrite & |rf_wa;

endmodule
