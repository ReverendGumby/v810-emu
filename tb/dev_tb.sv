`timescale 1us / 1ps

module dev_tb();

bit             clk, ce, res;
logic [31:0]    ia, da;
logic [31:0]    dut_id;
logic           dut_ireq, dut_iack;
logic [31:0]    dut_dd_i, dut_dd_o;
logic [3:0]     dut_dbe;
logic           dut_dwr;
logic           dut_dreq, dut_dack;
logic           dack_w0, dack_w1;
int             dmem_ws;

initial begin
    $timeformat(-6, 0, " us", 1);

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

   .DA(da),
   .DD_I(dut_dd_i),
   .DD_O(dut_dd_o),
   .DBE(dut_dbe),
   .DWR(dut_dwr),
   .DREQ(dut_dreq),
   .DACK(dut_dack),

   .ST()
   );

assign dut_iack = dut_ireq;

// Emulate memory with one wait state
always @(posedge clk) if (ce) begin
    dack_w1 <= ~dut_dack & dut_dreq;
end

// Emulate memory with no wait states
assign dack_w0 = dut_dreq;

always @* begin
    case (dmem_ws)
        0: dut_dack = dack_w0;
        1: dut_dack = dack_w1;
        default: dut_dack = 'Z;
    endcase
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
   .nCE(~dut_dack),
   .nWE(~dut_dwr),
   .nOE(dut_dwr),
   .nBE(~dut_dbe),
   .A(da[11:2]),
   .DI(dut_dd_o),
   .DO(dut_dd_i)
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
    dmem_ws = 0;
    test_all;

    dmem_ws = 1;
    test_all;

    $display("Done!");
    $finish();
end

initial #100 begin
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
    $display("Loading %s", fn);
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
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp ../v810_exec.sv dev_tb.sv && ./dev_tb.vvp"
// End:
