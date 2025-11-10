`timescale 1us / 1ns

module dev_tb();

bit             clk, ce, res;
logic [31:0]    dut_ia, dut_da;
logic [31:0]    dut_id;
logic           dut_ireq, dut_iack;
wire [31:0]     dut_dd_i, dut_dd_o;
wire [1:0]      dut_dbc;
wire [3:0]      dut_dbe;
wire            dut_dwr;
wire            dut_dreq, dut_dack;

wire [31:0]     dut_mem_a;
logic [31:0]    dut_mem_euia;
wire [31:0]     dut_mem_d_i, dut_mem_d_o, dut_mem_euid;
logic           dut_mem_euireq, dut_mem_euiack;
wire [3:0]      dut_mem_ben;
wire            dut_mem_dan;
wire            dut_mem_mrqn;
wire            dut_mem_rw;
wire            dut_mem_bcystn;
wand            dut_mem_readyn;
logic           dut_mem_szrqn;

logic [31:0]    imem_a, imem_do;
int             imem_ws, imem_dw;
logic [3:0]     imem_ben;
logic           imem_cen;
int             dmem_ws, dmem_dw;
wire [31:0]     dmem_di, dmem_do;
logic [3:0]     dmem_ben;
logic           dmem_cen;

logic [31:0]    idbr_mem_do;

bit             eu_bypass_mau;

initial begin
    $timeformat(-9, 0, " ns", 1);

    $dumpfile("dev_tb.vcd");
    $dumpvars();
end

v810_exec dut
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .IA(dut_ia),
   .ID(dut_id),
   .IREQ(dut_ireq),
   .IACK(dut_iack),

   .DA(dut_da),
   .DD_I(dut_dd_i),
   .DD_O(dut_dd_o),
   .DBC(dut_dbc),
   .DBE(dut_dbe),
   .DWR(dut_dwr),
   .DREQ(dut_dreq),
   .DACK(dut_dack),

   .ST()
   );

v810_mem dut_mem
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .EUDA(dut_da),
   .EUDD_I(dut_dd_i),
   .EUDD_O(dut_dd_o),
   .EUDBC(dut_dbc),
   .EUDBE(dut_dbe),
   .EUDWR(dut_dwr),
   .EUDREQ(dut_dreq),
   .EUDACK(dut_dack),

   .EUIA(dut_mem_euia),
   .EUID(dut_mem_euid),
   .EUIREQ(dut_mem_euireq),
   .EUIACK(dut_mem_euiack),

   .A(dut_mem_a),
   .D_I(dut_mem_d_i),
   .D_O(dut_mem_d_o),
   .BEn(dut_mem_ben),
   .DAn(dut_mem_dan),
   .MRQn(dut_mem_mrqn),
   .RW(dut_mem_rw),
   .BCYSTn(dut_mem_bcystn),
   .READYn(dut_mem_readyn),
   .SZRQn(dut_mem_szrqn)
   );

// Route EU ins. fetch to MAU or ROM direct
always @* begin
    if (eu_bypass_mau) begin
        dut_id = imem_do;
        dut_iack = dut_ireq;

        dut_mem_euireq = '0;
        dut_mem_euia = 'Z;

        imem_ws = 0;
        imem_dw = 32;
        imem_a = dut_ia;
        imem_ben = '0;
        imem_cen = ~dut_ireq;

        idbr_mem_do = 'Z;
    end
    else begin
        dut_id = dut_mem_euid;
        dut_iack = dut_mem_euiack;

        dut_mem_euireq = dut_ireq;
        dut_mem_euia = dut_ia;

        imem_ws = dmem_ws;
        imem_dw = dmem_dw;
        imem_a = dut_mem_a;
        imem_ben = dut_mem_ben;
        imem_cen = dut_mem_mrqn;

        idbr_mem_do = imem_do;
    end
end

assign dut_mem_szrqn = ~((dmem_dw == 16) & ~dut_mem_readyn);

assign dmem_ben = dut_mem_ben;
assign dmem_cen = dut_mem_mrqn;

data_bus_resizer idbr
  (
   .WS(imem_ws),
   .DW(imem_dw),
   .CLK(clk),
   .CE(ce),
   .CTLR_DAn(dut_mem_dan),
   .CTLR_BEn(dut_mem_ben),
   .CTLR_READYn(dut_mem_readyn),
   .CTLR_DI(dut_mem_d_i),
   .CTLR_DO(),
   .MEM_DI(),
   .MEM_DO(idbr_mem_do)
   );

ram #(10, 32) imem
  (
   .CLK(clk),
   .nCE(imem_cen | ~imem_a[31]),
   .nWE('1),
   .nOE('0),
   .nBE(imem_ben),
   .A(imem_a[11:2]),
   .DI('Z),
   .DO(imem_do)
   );

data_bus_resizer ddbr
  (
   .WS(dmem_ws),
   .DW(dmem_dw),
   .CLK(clk),
   .CE(ce),
   .CTLR_DAn(dut_mem_dan),
   .CTLR_BEn(dut_mem_ben),
   .CTLR_READYn(dut_mem_readyn),
   .CTLR_DI(dut_mem_d_i),
   .CTLR_DO(dut_mem_d_o),
   .MEM_DI(dmem_di),
   .MEM_DO(dmem_do)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE(dmem_cen | dut_mem_a[31]),
   .nWE(dut_mem_rw),
   .nOE(~dut_mem_rw),
   .nBE(dmem_ben),
   .A(dut_mem_a[11:2]),
   .DI(dmem_di),
   .DO(dmem_do)
   );

task imem_load_boot;
    imem.mem['h3FC] = 32'h8000BFE0; // MOVHI #0x8000, r0, r31
    imem.mem['h3FD] = 32'h0000181F; // JMP LP
endtask

initial begin
    res = 1;
    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

task start_test;
    imem_load_boot;

    repeat (5) @(posedge clk) ;
    res <= 0;
    @(posedge clk) ;
endtask

task end_test;
    @(dut.halt) ;               // wait for HALT instruction

    repeat (10) @(posedge clk) ;
    //assert(dut_mem_dan & dut_mem_mrqn); // reset while bus is busy == bad
    res <= 1;
endtask

initial #0 begin
    test_all_modes;

    $display("Done!");
    $finish();
end

initial #800 begin
    $error("Emergency exit!");
    $fatal(1);
end

always @dut_ia begin
    if (dut_ia[0]) begin
        $error("Invalid instruction address!");
        $fatal(1);
    end
end

task test_mov_rr;
    imem.load_hex16("dev_imem_mov_rr.hex");
    start_test;
    end_test;

    for (int i = 1; i < 32; i++)
        ;//assert(dut.rmem[i] == 32'h0);
endtask

task test_alu0;
    imem.load_hex16("dev_imem_alu0.hex");
    start_test;
    end_test;

    assert(dut.rmem[3] == 32'h1);
    assert(dut.rmem[5] == 32'h4);
    assert(dut.rmem[7] == 32'h7);
    assert(dut.rmem[9] == 32'h11);
endtask

task test_alu1;
    imem.load_hex16("dev_imem_alu1.hex");
    start_test;
    end_test;

    assert(dut.rmem[10] == 32'hF0007);
    assert(dut.rmem[11] == 32'h1);
    assert(dut.rmem[12] == 32'h7);
    assert(dut.rmem[13] == 32'h1F);
    assert(dut.rmem[14] == 32'hE);
endtask

task test_alu2;
    imem.load_hex16("dev_imem_alu2.hex");
    start_test;
    end_test;

    assert(dut.rmem[15] == 32'h0);
endtask

task test_ldst0;
    imem.load_hex16("dev_imem_ldst0.hex");
    dmem.load_hex("dev_dmem_ldst0.hex");
    start_test;
    end_test;

    assert(dmem.mem[3] == dmem.mem[1]);
    assert(dmem.mem[4] == dmem.mem[2]);
    assert(dut.rmem[8] == dmem.mem[3]);
    assert(dut.rmem[9] == dmem.mem[4]);
endtask

task test_ldst1;
    imem.load_hex16("dev_imem_ldst1.hex");
    dmem.load_hex("dev_dmem_ldst1.hex");
    start_test;
    end_test;

    assert(dmem.mem[4][31:16] == dmem.mem[1][15:0]);
    assert(dmem.mem[4][15:0] == dmem.mem[1][31:16]);
    assert(dmem.mem[3][31:16] == dmem.mem[2][15:0]);
    assert(dmem.mem[3][15:0] == dmem.mem[2][31:16]);
    assert(dut.rmem[8] == dmem.mem[3]);
    assert(dut.rmem[9] == dmem.mem[4]);
endtask

task test_ldst2;
    imem.load_hex16("dev_imem_ldst2.hex");
    dmem.load_hex("dev_dmem_ldst2.hex");
    start_test;
    end_test;

    assert(dmem.mem[4][31:24] == dmem.mem[1][7:0]);
    assert(dmem.mem[4][23:16] == dmem.mem[1][15:8]);
    assert(dmem.mem[4][15:0] == '0);
    assert(dmem.mem[3][15:8] == dmem.mem[2][23:16]);
    assert(dmem.mem[3][7:0] == dmem.mem[2][31:24]);
    assert(dmem.mem[3][31:16] == '0);
    assert(dut.rmem[8] == dmem.mem[3]);
    assert(dut.rmem[9] == dmem.mem[4]);
endtask

task test_data_hazard0;
    imem.load_hex16("dev_imem_data_hazard0.hex");
    start_test;
    end_test;

    assert(dut.rmem[2] == 32'd10);
    assert(dut.rmem[5] == 32'd1);
    assert(dmem.mem[112>>2] == 32'd9);
endtask

task test_bcond0;
    imem.load_hex16("dev_imem_bcond0.hex");
    start_test;
    end_test;

    assert(dut.rmem[1] == 32'd0);
endtask

task test_setf;
    imem.load_hex16("dev_imem_setf.hex");
    start_test;
    end_test;

    assert(dut.rmem[2] == 32'd0);
endtask

task test_jmp_jr_jal;
    imem.load_hex16("dev_imem_jmp_jr_jal.hex");
    start_test;
    end_test;

    assert(dut.rmem[2] == 32'h5);
endtask

task test_all;
    test_mov_rr;
    test_alu0;
    test_ldst0;
    test_data_hazard0;
    test_bcond0;
    test_setf;
    test_jmp_jr_jal;
    test_alu1;
    test_alu2;
    test_ldst1;
    test_ldst2;
endtask

task test_all_ram_modes;
    $display("%t: RAM: Bypass T1 state, 32-bit, 0-wait", $realtime);
    dut_mem.dbg_bypass_ebi_t1 = '1;
    dmem_ws = 0;
    dmem_dw = 32;
    test_all;
    dut_mem.dbg_bypass_ebi_t1 = '0;

    $display("%t: RAM: 32-bit, 0-wait", $realtime);
    dmem_ws = 0;
    dmem_dw = 32;
    test_all;

    $display("%t: RAM: 32-bit, 1-wait", $realtime);
    dmem_ws = 1;
    dmem_dw = 32;
    test_all;

    $display("%t: RAM: 16-bit, 0-wait", $realtime);
    dmem_ws = 0;
    dmem_dw = 16;
    test_all;

    $display("%t: RAM: 16-bit, 1-wait", $realtime);
    dmem_ws = 1;
    dmem_dw = 16;
    test_all;
endtask

task test_all_wb_modes;
    $display("MAU: WB bypassed");
    dut_mem.dbg_bypass_wb = '1;
    test_all_ram_modes;

    $display("MAU: WB enabled");
    dut_mem.dbg_bypass_wb = '0;
    test_all_ram_modes;
endtask

task test_all_modes;
    $display("EU: Fetch ins. directly from ROM (emulate I$ always hit)");
    eu_bypass_mau = '1;
    test_all_wb_modes;

    $display("EU: Fetch ins. from MAU");
    eu_bypass_mau = '0;
    test_all_wb_modes;
endtask

endmodule

//////////////////////////////////////////////////////////////////////

module ram
  #(parameter AW,
    parameter DW)
  (
   input            CLK,
   input            nCE,
   input            nWE,
   input            nOE,
   input [DW/8-1:0] nBE,
   input [AW-1:0]   A,
   input [DW-1:0]   DI,
   output [DW-1:0]  DO
 );

localparam SIZE = 1 << AW;

bit [DW-1:0]    mem [0:SIZE-1];
bit [DW-1:0]    dor;

task clear;
    for (int i = 0; i < SIZE; i++)
        mem[i] = '0;
endtask

task load_hex(string fn);
    clear();
    $readmemh(fn, mem);
endtask

task load_hex16(string fn);
bit [15:0] tmp [SIZE * 2];
    assert(DW / 16 == 2);
    //$display("Loading %s", fn);
    for (int i = 0; i < SIZE * 2; i++)
        tmp[i] = '0;
    $readmemh(fn, tmp);
    for (int i = 0; i < SIZE * 2; i++)
        mem[i / 2][i[0]*16+:16] = tmp[i];
endtask

task load_bin(string fn);
integer fin, code;
    begin
        clear();
        fin = $fopen(fn, "r");
        assert(fin != 0) else $fatal(1, "unable to open file");
        code = $fread(mem, fin, 0, SIZE);
        assert(code == SIZE) else $fatal(1, "file too short");
    end
endtask

always @(negedge CLK) begin
    dor <= mem[A];
end

assign DO = ~(nCE | nOE) ? dor : {DW{1'bz}};

always @(posedge CLK) begin
    for (int i = 0; i < DW/8; i++)
        if (~(nCE | nWE | nBE[i]))
            mem[A][i*8+:8] <= DI[i*8+:8];
end

endmodule

//////////////////////////////////////////////////////////////////////

module data_bus_resizer
  (
   input int           WS,
   input int           DW,

   input               CLK,
   input               CE,

   input               CTLR_DAn,
   input [3:0]         CTLR_BEn,
   output logic        CTLR_READYn,
   output [31:0]       CTLR_DI,
   input [31:0]        CTLR_DO,

   output logic [31:0] MEM_DI,
   input [31:0]        MEM_DO
   );

logic           ready_w0, ready_w1;
logic [31:0]    ctlr_di;

// Emulate memory with one wait state
always @(posedge CLK) if (CE) begin
    ready_w1 <= ~ready_w1 & ~CTLR_DAn;
end

// Emulate memory with no wait states
assign ready_w0 = ~CTLR_DAn;

always @* begin
    case (WS)
        0: CTLR_READYn = ~ready_w0;
        1: CTLR_READYn = ~ready_w1;
        default: CTLR_READYn = 'X;
    endcase
end

always @* begin
    if (DW == 32) begin
        MEM_DI = CTLR_DO;
        ctlr_di = MEM_DO;
    end
    else begin
        case (CTLR_BEn)
            4'b1110, 4'b1101, 4'b1100, 4'b0000: begin
                MEM_DI = {16'bz, CTLR_DO[15:0]};
                ctlr_di = {16'bz, MEM_DO[15:0]};
            end
            default: begin
                MEM_DI = {CTLR_DO[15:0], 16'bz};
                ctlr_di = {16'bz, MEM_DO[31:16]};
            end
        endcase
    end
end

assign CTLR_DI = ctlr_di;

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp ../v810_exec.sv ../v810_mem.sv dev_tb.sv && ./dev_tb.vvp"
// End:
