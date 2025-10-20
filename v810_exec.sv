`timescale 1us / 1ns

module v810_exec
  (
   input         RESn,
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
   output [3:0]  BEn, // Byte Enable

   output [1:0]  ST, // Status
   output        DAn, // Data Access
   output        MRQn, // Memory ReQuest
   output        RW, // Read / Write
   output        BCYSTn // Bus CYcle STart
   );


//////////////////////////////////////////////////////////////////////
// Pipeline registers

typedef struct packed {
    logic RegDst;
    logic [3:0] ALUOp;          // placeholder
    logic ALUSrc;
} ctl_ex_t;

typedef struct packed {
    logic Branch;
    logic MemRead;
    logic MemWrite;
} ctl_mem_t;

typedef struct packed {
    logic RegWrite;
} ctl_wb_t;

// IF/ID
logic [31:0]    ifid_pc;
logic [15:0]    ifid_ir;

// ID/EX
logic [31:0]    idex_pc;
logic [31:0]    idex_rf_rd1, idex_rf_rd2;
logic [4:0]     idex_rf_wa;
struct packed {
    ctl_ex_t    ex;
    ctl_mem_t   mem;
    ctl_wb_t    wb;
} idex_ctl;

// EX/MEM
logic [4:0]     exmem_rf_wa;
logic [31:0]    exmem_alu_out;
struct packed {
    ctl_mem_t   mem;
    ctl_wb_t    wb;
} exmem_ctl;

// MEM/WB
logic [4:0]     memwb_rf_wa;
logic [31:0]    memwb_alu_out;
struct packed {
    ctl_wb_t    wb;
} memwb_ctl;


//////////////////////////////////////////////////////////////////////
// IF - Instruction Fetch stage
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// Instruction memory interface

logic [31:0]    imi_a;
logic [15:0]    imi_d;

assign IA = imi_a;
assign imi_d = ID;

//////////////////////////////////////////////////////////////////////
// Program Counter

logic [31:1]    pc;

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        pc <= '0;
    end
    else begin
        pc <= pc + 1;
    end
end

assign imi_a = {pc, 1'b0};

//////////////////////////////////////////////////////////////////////
// Instruction Register

logic [31:0]    ir;

always @* begin
    if (~RESn) begin
        ir = 16'h9A00;          // NOP
    end
    else begin
        ir = {16'b0, imi_d};
    end
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
        6'b0x0_000,             // MOV, ADD
        6'b000_010,             // SUB
        6'b0x0_10x,             // SHL, SHR
        6'b001_110,             // XOR
        6'b0x0_111,             // SAR
        6'b001_111:             // NOT
            id_ctl_wb.RegWrite = '1;
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

logic [31:0] alu_out;

always @* begin
    alu_out = idex_rf_rd1;
end

//////////////////////////////////////////////////////////////////////
// EX/MEM pipeline register

always @(posedge CLK) if (CE) begin
    exmem_rf_wa <= idex_rf_wa;
    exmem_alu_out <= alu_out;
    exmem_ctl.mem <= idex_ctl.mem;
    exmem_ctl.wb <= idex_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// MEM - Memory access
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// MEM/WB pipeline register

always @(posedge CLK) if (CE) begin
    memwb_rf_wa <= exmem_rf_wa;
    memwb_alu_out <= exmem_alu_out;
    memwb_ctl.wb <= exmem_ctl.wb;
end


//////////////////////////////////////////////////////////////////////
// WB - Write back
//////////////////////////////////////////////////////////////////////

assign rf_wa = memwb_rf_wa;
assign rf_wd = memwb_alu_out;
assign rf_we = memwb_ctl.wb.RegWrite;

endmodule
