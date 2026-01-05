// Pipeline development testbench
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ns

module dev_tb();

bit             clk, ce, res;
bit             halted;

logic [31:0]    dut_ia, dut_da;
logic [31:0]    dut_id;
logic           dut_ireq, dut_iack;
wire [31:0]     dut_dd_i, dut_dd_o;
wire [1:0]      dut_dbc;
wire [3:0]      dut_dbe;
wire            dut_dwr;
wire            dut_dmrq;
wire [1:0]      dut_dst;
wire            dut_dreq, dut_dack;

wire [31:0]     dut_mem_a;
logic [31:0]    dut_mem_icia;
wire [31:0]     dut_mem_d_i, dut_mem_d_o, dut_mem_icid;
logic           dut_mem_icireq, dut_mem_iciack;
wire [3:0]      dut_mem_ben;
wire [1:0]      dut_mem_st;
wire            dut_mem_dan;
wire            dut_mem_mrqn;
wire            dut_mem_rw;
wire            dut_mem_bcystn;
wor             dut_mem_readyn;
wand            dut_mem_szrqn;

wire            inex_euf;
wire [15:0]     inex_euccb;
wire [4:0]      inex_eucco;
wire            inex_adtrf;
wire            inex_if;
wire            inex_np;
wire [3:0]      inex_iel;
wire [15:0]     inex_cc;
wire [31:0]     inex_ha;
wire            inex_ack;

wire [4:0]      sr_ra, sr_wa;
wire [31:0]     sr_rd, sr_wd;
wire            sr_we;
psw_t           psw, psw_reset, psw_set;
wire            ecr_set_eicc, ecr_set_fecc;

logic [31:0]    imem_a, imem_do;
int             imem_ws, imem_dw;
logic [3:0]     imem_ben;
logic           imem_cen;
int             dmem_ws, dmem_dw;
wire [31:0]     dmem_di, dmem_do;
logic [3:0]     dmem_ben;
logic           dmem_cen;
logic           dmem_as_io = 0;

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

   .INEX_EUF(inex_euf),
   .INEX_EUCCB(inex_euccb),
   .INEX_EUCCO(inex_eucco),
   .INEX_ADTRF(inex_adtrf),
   .INEX_IF(inex_if),
   .INEX_NP(inex_np),
   .INEX_IEL(inex_iel),
   .INEX_HA(inex_ha),
   .INEX_ACK(inex_ack),

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
   .DMRQ(dut_dmrq),
   .DST(dut_dst),
   .DREQ(dut_dreq),
   .DACK(dut_dack),

   .SR_RA(sr_ra),
   .SR_RD(sr_rd),
   .SR_WA(sr_wa),
   .SR_WD(sr_wd),
   .SR_WE(sr_we),
   .PSW(psw),
   .PSW_RESET(psw_reset),
   .PSW_SET(psw_set),
   .ECR_SET_EICC(ecr_set_eicc),
   .ECR_SET_FECC(ecr_set_fecc),

   .ICMAINT('0)
   );

v810_inex dut_inex
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .PSW(psw),

   .INT('0),
   .INTVn('1),
   .NMIn('1),

   .EUF(inex_euf),
   .EUCCB(inex_euccb),
   .EUCCO(inex_eucco),
   .ADTRF(inex_adtrf),
   .IF(inex_if),
   .NP(inex_np),
   .IEL(inex_iel),
   .CC(inex_cc),
   .HA(inex_ha),
   .ACK(inex_ack)
   );

v810_sysreg dut_sr
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .RA(sr_ra),
   .RD(sr_rd),

   .WA(sr_wa),
   .WD(sr_wd),
   .WE(sr_we),

   .PSW(psw),
   .PSW_RESET(psw_reset),
   .PSW_SET(psw_set),
   .ECR_CC(inex_cc),
   .ECR_SET_EICC(ecr_set_eicc),
   .ECR_SET_FECC(ecr_set_fecc),

   .CHCW('0),
   .CHCW_WD(),
   .CHCW_WE()
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
   .EUDMRQ(dut_dmrq),
   .EUDST(dut_dst),
   .EUDREQ(dut_dreq),
   .EUDACK(dut_dack),

   .ICIA(dut_mem_icia),
   .ICID(dut_mem_icid),
   .ICIREQ(dut_mem_icireq),
   .ICIACK(dut_mem_iciack),

   .A(dut_mem_a),
   .D_I(dut_mem_d_i),
   .D_O(dut_mem_d_o),
   .BEn(dut_mem_ben),
   .ST(dut_mem_st),
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

        dut_mem_icireq = '0;
        dut_mem_icia = 'Z;

        imem_ws = 0;
        imem_dw = 32;
        imem_a = dut_ia;
        imem_ben = '0;
        imem_cen = ~dut_ireq;

        idbr_mem_do = 'Z;
    end
    else begin
        dut_id = dut_mem_icid;
        dut_iack = dut_mem_iciack;

        dut_mem_icireq = dut_ireq;
        dut_mem_icia = dut_ia;

        imem_ws = dmem_ws;
        imem_dw = dmem_dw;
        imem_a = dut_mem_a;
        imem_ben = dut_mem_ben;
        imem_cen = dut_mem_mrqn;

        idbr_mem_do = imem_do;
    end
end

assign dut_mem_readyn = '0;
assign dut_mem_szrqn = '1;

wire dut_mem_iorqn = ~(dut_mem_mrqn & ~dut_mem_dan & (dut_mem_st == 2'b10));

assign dmem_ben = dut_mem_ben;
assign dmem_cen = dmem_as_io ? dut_mem_iorqn : dut_mem_mrqn;

data_bus_resizer idbr
  (
   .WS(imem_ws),
   .DW(imem_dw),
   .CLK(clk),
   .CE(ce),
   .CTLR_DAn(dut_mem_dan),
   .CTLR_BEn(dut_mem_ben),
   .CTLR_READYn(dut_mem_readyn),
   .CTLR_SZRQn(dut_mem_szrqn),
   .CTLR_DI(dut_mem_d_i),
   .CTLR_DO(),
   .MEM_nCE('0),
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
   .CTLR_SZRQn(dut_mem_szrqn),
   .CTLR_DI(dut_mem_d_i),
   .CTLR_DO(dut_mem_d_o),
   .MEM_nCE('0),
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
    halted = 0;
    res = 1;
    ce = 1;
    clk = 1;
end

always begin :ckgen
    #0.02 clk = ~clk;
end

event test_started;

task start_test;
    imem_load_boot;

    for (int i = 1; i < 32; i++)
        dut.rf.rmem[i] = 'X;

    -> test_started;

    repeat (5) @(posedge clk) ;
    res <= 0;
    @(posedge clk) ;
endtask

task end_test;
    @(posedge halted) ;         // wait for HALT instruction

    disable emergency_exit;

    repeat (10) @(posedge clk) ;
    //assert(dut_mem_dan & dut_mem_mrqn); // reset while bus is busy == bad
    res <= 1;
endtask

initial #0 begin
    test_all_modes;

    $display("Done!");
    $finish();
end

always @test_started begin
    begin :emergency_exit
        #40 ;
        // If we reach this point, halted didn't assert in time.
        $error("Emergency exit!");
        $fatal(1);
    end
end

always @dut_ia begin
    if (dut_ia[0]) begin
        $error("Invalid instruction address!");
        $fatal(1);
    end
end

// Assert that the bus stays idle after the fault/halt acknowledge.
always @(posedge clk) if (ce) begin
    if (res)
        halted <= '0;
    else begin
        if (~dut_mem_dan & ~dut_mem_readyn & dut_mem_mrqn & dut_mem_st[0])
            halted <= '1;
        else if (halted) begin
            assert(dut_mem_dan & dut_mem_bcystn);
            else
                @(posedge clk) $fatal(1, "Bus must stay idle after fault/halt");
        end
    end
end

task test_mov_rr;
    imem.load_hex16("dev_imem_mov_rr.hex");
    start_test;
    end_test;

    for (int i = 1; i < 32; i++)
        assert(dut.rf.rmem[i] == 32'h0);
endtask

task test_alu0;
    imem.load_hex16("dev_imem_alu0.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[3] == 32'h1);
    assert(dut.rf.rmem[5] == 32'h4);
    assert(dut.rf.rmem[7] == 32'h7);
    assert(dut.rf.rmem[9] == 32'h11);
endtask

task test_alu1;
    imem.load_hex16("dev_imem_alu1.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[10] == 32'hF0007);
    assert(dut.rf.rmem[11] == 32'h1);
    assert(dut.rf.rmem[12] == 32'h7);
    assert(dut.rf.rmem[13] == 32'h1F);
    assert(dut.rf.rmem[14] == 32'hE);
endtask

task test_alu2;
    imem.load_hex16("dev_imem_alu2.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[15] == 32'h0);
endtask

task test_ldst0;
    imem.load_hex16("dev_imem_ldst0.hex");
    dmem.load_hex("dev_dmem_ldst0.hex");
    start_test;
    end_test;

    assert(dmem.mem[3] == dmem.mem[1]);
    assert(dmem.mem[4] == dmem.mem[2]);
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
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
    assert(dut.rf.rmem[2] == {{16{dmem.mem[1][15]}}, dmem.mem[1][15:0]});
    assert(dut.rf.rmem[3] == {{16{dmem.mem[1][31]}}, dmem.mem[1][31:16]});
    assert(dut.rf.rmem[4] == {{16{dmem.mem[2][15]}}, dmem.mem[2][15:0]});
    assert(dut.rf.rmem[5] == {{16{dmem.mem[2][31]}}, dmem.mem[2][31:16]});
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
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
    assert(dut.rf.rmem[2] == {{24{dmem.mem[1][7]}}, dmem.mem[1][7:0]});
    assert(dut.rf.rmem[3] == {{24{dmem.mem[1][15]}}, dmem.mem[1][15:8]});
    assert(dut.rf.rmem[4] == {{24{dmem.mem[2][23]}}, dmem.mem[2][23:16]});
    assert(dut.rf.rmem[5] == {{24{dmem.mem[2][31]}}, dmem.mem[2][31:24]});
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
endtask

task test_data_hazard0;
    imem.load_hex16("dev_imem_data_hazard0.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[2] == 32'd10);
    assert(dut.rf.rmem[5] == 32'd1);
    assert(dmem.mem[112>>2] == 32'd9);
endtask

task test_bcond0;
    imem.load_hex16("dev_imem_bcond0.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[1] == 32'd0);
endtask

task test_setf;
    imem.load_hex16("dev_imem_setf.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[2] == 32'd0);
endtask

task test_jmp_jr_jal;
    imem.load_hex16("dev_imem_jmp_jr_jal.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[2] == 32'h5);
endtask

task test_jal1;
    imem.load_hex16("dev_imem_jal1.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[1] == 32'h9);
endtask

task test_jal2;
    imem.load_hex16("dev_imem_jal2.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[1] == 32'h80000020);
    assert(dut.rf.rmem[31] == 32'h80000010);
endtask

task test_ldsr0;
    imem.load_hex16("dev_imem_ldsr0.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[15] == 32'd15);
endtask

task test_ldsr1;
    imem.load_hex16("dev_imem_ldsr1.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h00001000);
endtask

task test_stsr0;
    imem.load_hex16("dev_imem_stsr0.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[15] == 32'd15);
endtask

task test_stsr1;
    imem.load_hex16("dev_imem_stsr1.hex");
    start_test;
    end_test;

    assert(dut.rf.rmem[15] == 32'd15);
endtask

task test_inout0;
    imem.load_hex16("dev_imem_inout0.hex");
    dmem.load_hex("dev_dmem_inout0.hex");
    dmem_as_io = 1;
    start_test;
    end_test;
    dmem_as_io = 0;

    assert(dmem.mem[3] == dmem.mem[1]);
    assert(dmem.mem[4] == dmem.mem[2]);
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
endtask

task test_inout1;
    imem.load_hex16("dev_imem_inout1.hex");
    dmem.load_hex("dev_dmem_inout1.hex");
    dmem_as_io = 1;
    start_test;
    end_test;
    dmem_as_io = 0;

    assert(dmem.mem[4][31:16] == dmem.mem[1][15:0]);
    assert(dmem.mem[4][15:0] == dmem.mem[1][31:16]);
    assert(dmem.mem[3][31:16] == dmem.mem[2][15:0]);
    assert(dmem.mem[3][15:0] == dmem.mem[2][31:16]);
    assert(dut.rf.rmem[2] == {16'b0, dmem.mem[1][15:0]});
    assert(dut.rf.rmem[3] == {16'b0, dmem.mem[1][31:16]});
    assert(dut.rf.rmem[4] == {16'b0, dmem.mem[2][15:0]});
    assert(dut.rf.rmem[5] == {16'b0, dmem.mem[2][31:16]});
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
endtask

task test_inout2;
    imem.load_hex16("dev_imem_inout2.hex");
    dmem.load_hex("dev_dmem_inout2.hex");
    dmem_as_io = 1;
    start_test;
    end_test;
    dmem_as_io = 0;

    assert(dmem.mem[4][31:24] == dmem.mem[1][7:0]);
    assert(dmem.mem[4][23:16] == dmem.mem[1][15:8]);
    assert(dmem.mem[4][15:0] == '0);
    assert(dmem.mem[3][15:8] == dmem.mem[2][23:16]);
    assert(dmem.mem[3][7:0] == dmem.mem[2][31:24]);
    assert(dmem.mem[3][31:16] == '0);
    assert(dut.rf.rmem[2] == {24'b0, dmem.mem[1][7:0]});
    assert(dut.rf.rmem[3] == {24'b0, dmem.mem[1][15:8]});
    assert(dut.rf.rmem[4] == {24'b0, dmem.mem[2][23:16]});
    assert(dut.rf.rmem[5] == {24'b0, dmem.mem[2][31:24]});
    assert(dut.rf.rmem[8] == dmem.mem[3]);
    assert(dut.rf.rmem[9] == dmem.mem[4]);
endtask

task test_trap0;
    imem.load_hex16("dev_imem_trap0.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h00005001);
    assert(dut_sr.eipc == 32'h80000008); // next PC
    assert(dut_sr.eipsw == 32'h00000001);
    assert(dut_sr.ecr.fecc == '0);
    assert(dut_sr.ecr.eicc == 16'hffb8);
endtask

task test_trap1;
    imem.load_hex16("dev_imem_trap1.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h0000d001);
    assert(dut_sr.fepc == 32'h80000008); // next PC
    assert(dut_sr.fepsw == 32'h00004001);
    assert(dut_sr.ecr.fecc == 16'hffa8);
    assert(dut_sr.ecr.eicc == 16'hfff0);
endtask

task test_reti0;
    imem.load_hex16("dev_imem_reti0.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h00000001);
endtask

task test_reti1;
    imem.load_hex16("dev_imem_reti1.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h00000001);
endtask

task test_invalid_op;
    imem.load_hex16("dev_imem_invalid_op.hex");
    start_test;
    end_test;

    assert(dut_sr.psw == 32'h00005001);
    assert(dut_sr.eipc == 32'h80000006); // current PC
    assert(dut_sr.eipsw == 32'h00000001);
    assert(dut_sr.ecr.fecc == '0);
    assert(dut_sr.ecr.eicc == 16'hff90);
endtask

task test_mul;
    imem.load_hex16("dev_imem_mul.hex");
    dmem.load_hex("dev_dmem_mul.hex");

    start_test;
    end_test;

    assert(dut.rf.rmem[16] == 32'd0);
endtask

task test_mulu;
    imem.load_hex16("dev_imem_mulu.hex");
    dmem.load_hex("dev_dmem_mulu.hex");

    start_test;
    end_test;

    assert(dut.rf.rmem[16] == 32'd0);
endtask

task test_all;
    test_mov_rr;
    test_alu0;
    test_ldst0;
    test_data_hazard0;
    test_bcond0;
    test_setf;
    test_jmp_jr_jal;
    test_jal1;
    test_jal2;
    test_alu1;
    test_alu2;
    test_ldst1;
    test_ldst2;
    test_ldsr0;
    test_ldsr1;
    test_stsr1;
    test_stsr0;
    test_inout0;
    test_inout1;
    test_inout2;
    test_trap0;
    test_trap1;
    test_reti0;
    test_reti1;
    test_invalid_op;
    test_mul;
    test_mulu;
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

/* -----\/----- EXCLUDED -----\/-----
always @(posedge clk) if (ce) begin
    if (~(res | dut.id_stall | dut.id_flush | dut.ifid_exc))
        $display("PC=%x IR=%x", dut.ifid_pc, dut.ifid_ir);
end
 -----/\----- EXCLUDED -----/\----- */

endmodule

//////////////////////////////////////////////////////////////////////

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s dev_tb -o dev_tb.vvp -f v810.files ram.sv data_bus_resizer.sv dev_tb.sv && ./dev_tb.vvp"
// End:
