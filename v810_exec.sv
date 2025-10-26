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

typedef struct packed {
    logic Carry;
    logic Over;
    logic Sign;
    logic Zero;
} aluflags_t;

aluflags_t          psw;    // TODO: More flags to come

event halt;


//////////////////////////////////////////////////////////////////////
// Pipeline registers

typedef enum bit [1:0] {
    ALUSRC1_RF_RD1 = 2'd0,
    ALUSRC1_IMM5,
    ALUSRC1_PC
} alu_src1_t;

typedef enum bit [1:0] {
    ALUSRC2_RF_RD2 = 2'd0,
    ALUSRC2_DISP16,
    ALUSRC2_DISP9
} alu_src2_t;

typedef struct packed {
    logic [3:0] ALUOp;
    alu_src1_t ALUSrc1;
    alu_src2_t ALUSrc2;
    logic Branch;
    logic [3:0] Bcond;
} ctl_ex_t;

typedef struct packed {
    logic MemRead;
    logic MemWrite;
    logic [3:0] FlagMask;
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
logic [31:0]    idex_disp9;
logic [31:0]    idex_disp16;
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
aluflags_t      exmem_alu_fl;
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
logic           if_flush;

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
logic           if_pc_en;

logic [31:0]    pc, pci, pcn;

assign if_pc_inc4 = if_ins32;

always @* begin
    pci = pc + (if_pc_inc4 ? 32'd4 : 32'd2);
end

always @* begin
    pcn = pc;
    if (if_pc_set)
        pcn = if_pc_set_val;
    else if (if_pc_inc)
        pcn = pci;
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        pc <= '0;
    end
    else if (if_pc_en) begin
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
    else if (if_pc_en) begin
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
    if (if_flush)
        ir = 16'h9A00;
end

//////////////////////////////////////////////////////////////////////
// IF/ID pipeline register

logic           ifid_en;

always @(posedge CLK) if (CE) begin
    if (~RESn | ifid_en) begin
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
ctl_mem_t       id_ctl_mem;
ctl_wb_t        id_ctl_wb;

always @* begin
    id_rf_ra1 = '0;
    id_rf_ra2 = '0;
    id_rf_wa = '0;
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
            end
        6'b0x0_011:             // CMP
            begin
                id_rf_ra2 = ifid_ir[9:5];
                if (ifid_ir[14])
                    id_ctl_ex.ALUSrc1 = ALUSRC1_IMM5;
                else
                    id_rf_ra1 = ifid_ir[4:0];
                id_ctl_ex.ALUOp = ALUOP_SUB;
                id_ctl_mem.FlagMask = '1;
            end
        6'b011_010:             // HALT
            // TODO: Emit a halt acknowledge cycle
            -> halt;
        6'b100_xxx:             // Bcond (branch)
            begin
                id_ctl_ex.ALUSrc1 = ALUSRC1_PC;
                id_ctl_ex.ALUSrc2 = ALUSRC2_DISP9;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_ex.Branch = '1;
                id_ctl_ex.Bcond = ifid_ir[12:9];
            end
        6'b110_0xx:             // LD
            begin
                id_rf_ra1 = ifid_ir[4:0];
                id_rf_wa = ifid_ir[9:5];
                id_ctl_ex.ALUSrc2 = ALUSRC2_DISP16;
                id_ctl_ex.ALUOp = ALUOP_ADD;
                id_ctl_mem.MemRead = '1;
                id_ctl_wb.MemtoReg = '1;
                id_ctl_wb.RegWrite = '1;
            end
        6'b110_1xx:             // ST
            begin
                id_rf_ra1 = ifid_ir[4:0];
                id_rf_ra2 = ifid_ir[9:5];
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

// Use asynch. logic to ensure that a write will be read in the next clock cycle.
always @* begin
    rf_rd1 = rmem[rf_ra1];
end

always @* begin
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

logic           id_ctl_en;
logic           id_flush;

wire idex_ctl_zero = RESn & (~id_ctl_en | id_flush);

always @(posedge CLK) if (CE) begin
    idex_pc <= ifid_pc;
    idex_imm <= 32'($signed(ifid_ir[4:0]));
    idex_disp9 <= 32'($signed(ifid_ir[8:0]));
    idex_disp16 <= 32'($signed(ifid_ir[31:16]));
    idex_rf_wa <= id_rf_wa;
    idex_rf_rd1 <= rf_rd1;
    idex_rf_rd2 <= rf_rd2;
    idex_ctl.ex <= idex_ctl_zero ? '0 : id_ctl_ex;
    idex_ctl.mem <= idex_ctl_zero ? '0 : id_ctl_mem;
    idex_ctl.wb <= idex_ctl_zero ? '0 : id_ctl_wb;
end


//////////////////////////////////////////////////////////////////////
// EX - Execute / address calculation
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// ALU

logic [31:0]    alu_in1, alu_in2;
logic [31:0]    alu_out;
aluop_t         alu_op;
aluflags_t      alu_fl;

always @* begin
    alu_in1 = 'X;
    case (idex_ctl.ex.ALUSrc1)
        ALUSRC1_RF_RD1: alu_in1 = idex_rf_rd1;
        ALUSRC1_IMM5:   alu_in1 = idex_imm;
        ALUSRC1_PC:     alu_in1 = idex_pc;
        default: ;
    endcase
end

always @* begin
    alu_in2 = 'X;
    case (idex_ctl.ex.ALUSrc2)
        ALUSRC2_RF_RD2: alu_in2 = idex_rf_rd2;
        ALUSRC2_DISP16: alu_in2 = idex_disp16;
        ALUSRC2_DISP9:  alu_in2 = idex_disp9;
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
    alu_fl.Zero = ~|alu_out;
    alu_fl.Sign = alu_out[31];
end

//////////////////////////////////////////////////////////////////////
// Branch condition test

logic           bcond_match;

always @* begin
    case (idex_ctl.ex.Bcond[2:0])
        3'b000:                // Overflow
            bcond_match = psw.Over;
        3'b001:                // Carry / Lower
            bcond_match = psw.Carry;
        3'b010:                // Zero / Equal
            bcond_match = psw.Zero;
        3'b011:                // Not higher
            bcond_match = psw.Carry | psw.Zero;
        3'b100:                // Negative
            bcond_match = psw.Sign;
        3'b101:                // Always
            bcond_match = '1; 
        3'b110:                // Less than signed
            bcond_match = psw.Sign ^ psw.Over;
        3'b111:                // Less than or equal signed
            bcond_match = (psw.Sign ^ psw.Over) | psw.Zero;
    endcase
    // MSB inverts the test.
    bcond_match ^= idex_ctl.ex.Bcond[3];
end

// On branch taken, set PC and flush pipeline before MEM.
wire branch_taken = idex_ctl.ex.Branch & bcond_match;
assign if_pc_set = branch_taken;
assign if_pc_set_val = alu_out;
assign if_flush = branch_taken;
assign id_flush = branch_taken;
//assign ex_flush = branch_taken;

//////////////////////////////////////////////////////////////////////
// EX/MEM pipeline register

logic           ex_flush;

wire exmem_ctl_flush = '0;//RESn & (ex_flush);

always @(posedge CLK) if (CE) begin
    exmem_rf_wa <= idex_rf_wa;
    exmem_rf_rd2 <= idex_rf_rd2;
    exmem_alu_out <= alu_out;
    exmem_alu_fl <= alu_fl;
    exmem_ctl.mem <= exmem_ctl_flush ? '0 : idex_ctl.mem;
    exmem_ctl.wb <= exmem_ctl_flush ? '0 : idex_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// MEM - Memory access
//////////////////////////////////////////////////////////////////////

logic [31:0]    mem_di;

assign DA = exmem_alu_out;
assign mem_di = DD_I;
assign DD_O = exmem_rf_rd2;

assign MRQn = ~(exmem_ctl.mem.MemRead | exmem_ctl.mem.MemWrite);
assign RW = MRQn | exmem_ctl.mem.MemRead;

always @(posedge CLK) if (CE) begin
    if (~RESn)
        psw <= '0;
    else
        psw <= (psw & ~exmem_ctl.mem.FlagMask) |
               (exmem_alu_fl & exmem_ctl.mem.FlagMask);
end

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


//////////////////////////////////////////////////////////////////////
// Hazard Detection
//////////////////////////////////////////////////////////////////////

// Data / Register Hazard
wire haz_data_ex = idex_ctl.wb.RegWrite & |idex_rf_wa &
     ((idex_rf_wa == rf_ra1) | (idex_rf_wa == rf_ra2));

wire haz_data_mem = exmem_ctl.wb.RegWrite & |exmem_rf_wa &
     ((exmem_rf_wa == rf_ra1) | (exmem_rf_wa == rf_ra2));

// TODO: Eliminate this hazard by forwarding reg. writes to same-cycle reads.
wire haz_data_wb = memwb_ctl.wb.RegWrite & |memwb_rf_wa &
     ((memwb_rf_wa == rf_ra1) | (memwb_rf_wa == rf_ra2));

wire haz_data = haz_data_ex | haz_data_mem | haz_data_wb;

// Flag Hazard
// TODO: Add SETF
wire haz_flag_ex = |idex_ctl.mem.FlagMask & id_ctl_ex.Branch;

wire haz_flag = haz_flag_ex;

assign if_pc_en = ~(haz_data | haz_flag);
assign ifid_en = ~(haz_data | haz_flag);
assign id_ctl_en = ~(haz_data | haz_flag);

endmodule
