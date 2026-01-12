// Run ROM BIOS
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ns

module bios_tb();

bit             clk, ce, res;
int             rescnt;

wire [31:0]     cpu_a;
logic [31:0]    cpu_d_i;
wire [31:0]     cpu_d_o;
wire [3:0]      cpu_ben;
wire [1:0]      cpu_st;
wire            cpu_dan;
wire            cpu_mrqn;
wire            cpu_rw;
wire            cpu_bcystn;
wire            cpu_readyn;
wand            cpu_szrqn;

wire [31:0]     rom_dbr_ctlr_di;
wire            rom_dbr_readyn;
wire            rom_dbr_szrqn;

int             rom_ws, rom_dw;
logic           rom_cen;
logic [31:0]    rom_do;

logic           ram_cen;
wire [31:0]     ram_do;

logic           unk_cen;

initial begin
    $timeformat(-9, 0, " ns", 1);

`ifndef VERILATOR
    $dumpfile("bios_tb.vcd");
    $dumpvars();
`else
    $dumpfile("bios_tb.verilator.fst");
    $dumpvars();
`endif
end

v810 dut
  (
   .RESn(~res),
   .CLK(clk),
   .CE(ce),

   .A(cpu_a),
   .D_I(cpu_d_i),
   .D_O(cpu_d_o),
   .BEn(cpu_ben),
   .ST(cpu_st),
   .DAn(cpu_dan),
   .MRQn(cpu_mrqn),
   .RW(cpu_rw),
   .BCYSTn(cpu_bcystn),
   .READYn(cpu_readyn),
   .SZRQn(cpu_szrqn),

   .INT('0),
   .INTVn('1),
   .NMIn('1)
   );

always @* begin
    if (~rom_cen)
        cpu_d_i = rom_dbr_ctlr_di;
    else if (~ram_cen)
        cpu_d_i = ram_do;
    else
        cpu_d_i = '0;
end

assign cpu_readyn = unk_cen & rom_dbr_readyn & ram_cen;
assign cpu_szrqn = ~unk_cen | rom_dbr_szrqn;

data_bus_resizer rom_dbr
  (
   .WS(rom_ws),
   .DW(rom_dw),
   .CLK(clk),
   .CE(ce),
   .CTLR_A1(cpu_a[1]),
   .CTLR_DAn(cpu_dan),
   .CTLR_BEn(cpu_ben),
   .CTLR_READYn(rom_dbr_readyn),
   .CTLR_SZRQn(rom_dbr_szrqn),
   .CTLR_DI(rom_dbr_ctlr_di),
   .CTLR_DO(),
   .MEM_nCE(rom_cen),
   .MEM_DI(),
   .MEM_DO(rom_do)
   );

ram #(18, 32) rombios
  (
   .CLK(clk),
   .nCE(rom_cen),
   .nWE('1),
   .nOE('0),
   .nBE(cpu_ben),
   .A(cpu_a[19:2]),
   .DI('Z),
   .DO(rom_do)
   );

ram #(10, 32) dmem
  (
   .CLK(clk),
   .nCE(ram_cen),
   .nWE(cpu_rw),
   .nOE(~cpu_rw),
   .nBE(cpu_ben),
   .A(cpu_a[11:2]),
   .DI(cpu_d_o),
   .DO(ram_do)
   );

assign ram_cen = ~(~cpu_mrqn & ~cpu_a[31]);
assign rom_cen = ~(~cpu_mrqn & (cpu_a[31:20] == 12'hFFF));
assign unk_cen = ~(ram_cen & rom_cen);

initial begin
    rescnt = 0;
    res = 1;
    ce = 0;
    clk = 1;
end

always begin :ckgen
    #0.01 clk = ~clk; // 50 MHz
end

always @(posedge clk)
    ce <= ~ce;

initial #0 begin
    rom_ws = 0;
    rom_dw = 16;
    rombios.load_hex16("rom.hex");
end

always @(posedge clk) if (ce) begin
    if (res) begin
        if (rescnt == 6)
            res <= 0;
        else
            rescnt <= rescnt + 1;
    end
end

initial #(20e3) begin
    $finish();
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s bios_tb -o bios_tb.vvp -f v810.files ram.sv data_bus_resizer.sv bios_tb.sv && ./bios_tb.vvp"
// End:
