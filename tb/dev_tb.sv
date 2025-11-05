`timescale 1us / 1ps

module dev_tb();

bit             clk, ce, res;
logic [31:0]    ia, dut_da;
logic [31:0]    dut_id;
logic           dut_ireq, dut_iack;
wire [31:0]     dut_dd_i, dut_dd_o;
wire [1:0]      dut_dbc;
wire [3:0]      dut_dbe;
wire            dut_dwr;
wire            dut_dreq, dut_dack;

wire [31:0]     dut_mem_a;
logic [31:0]    dut_mem_d_i, dut_mem_d_o;
wire [3:0]      dut_mem_ben;
wire            dut_mem_dan;
wire            dut_mem_mrqn;
wire            dut_mem_rw;
wire            dut_mem_bcystn;
logic           dut_mem_readyn;
logic           dut_mem_szrqn;

logic [31:0]    dmem_di, dmem_do;
logic [3:0]     dmem_ben;

logic           ready_w0, ready_w1;
int             dmem_ws, dmem_dw;

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

   .IA(ia),
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

   .EDA(dut_da),
   .EDD_I(dut_dd_i),
   .EDD_O(dut_dd_o),
   .EDBC(dut_dbc),
   .EDBE(dut_dbe),
   .EDWR(dut_dwr),
   .EDREQ(dut_dreq),
   .EDACK(dut_dack),

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

assign dut_iack = dut_ireq;

assign dut_mem_szrqn = ~((dmem_dw == 16) & ~dut_mem_readyn);

// Emulate memory with one wait state
always @(posedge clk) if (ce) begin
    ready_w1 <= ~ready_w1 & ~dut_mem_dan;
end

// Emulate memory with no wait states
assign ready_w0 = ~dut_mem_dan;

always @* begin
    case (dmem_ws)
        0: dut_mem_readyn = ~ready_w0;
        1: dut_mem_readyn = ~ready_w1;
        default: dut_mem_readyn = 'X;
    endcase
end

assign dmem_ben = dut_mem_ben;

always @* begin
    if (dmem_dw == 32) begin
        dmem_di = dut_mem_d_o;
        dut_mem_d_i = dmem_do;
    end
    else begin
        case (dut_mem_ben)
            4'b1110, 4'b1101, 4'b1100, 4'b0000: begin
                dmem_di = {16'bz, dut_mem_d_o[15:0]};
                dut_mem_d_i = {16'bz, dmem_do[15:0]};
            end
            default: begin
                dmem_di = {dut_mem_d_o[15:0], 16'bz};
                dut_mem_d_i = {16'bz, dmem_do[31:16]};
            end
        endcase
    end
end

ram #(10, 32) imem
  (
   .CLK(clk),
   .nCE('0),
   .nWE('1),
   .nOE('0),
   .nBE('1),
   .A(ia[11:2]),
   .DI('Z),
   .DO(dut_id)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE(dut_mem_mrqn),
   .nWE(dut_mem_rw),
   .nOE(~dut_mem_rw),
   .nBE(dmem_ben),
   .A(dut_mem_a[11:2]),
   .DI(dmem_di),
   .DO(dmem_do)
   );

initial begin
    res = 1;
    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

task start_test;
    repeat (5) @(posedge clk) ;
    res <= 0;
    @(posedge clk) ;
endtask

task end_test;
    @(dut.halt) ;               // wait for HALT instruction

    repeat (6) @(posedge clk) ;
    res <= 1;
endtask

initial #0 begin
    $display("%t: RAM: Bypass T1 state, 32-bit, 0-wait", $realtime);
    dut_mem.dbg_bypass = '1;
    dmem_ws = 0;
    dmem_dw = 32;
    test_all;
    dut_mem.dbg_bypass = '0;

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

    $display("Done!");
    $finish();
end

initial #500 begin
    $error("Emergency exit!");
    $fatal(1);
end

always @ia begin
    if (ia[0]) begin
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

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp ../v810_exec.sv ../v810_mem.sv dev_tb.sv && ./dev_tb.vvp"
// End:
