// Register file
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

module v810_regfile
  (
   input         CLK,
   input         CE, // global clock enable

   input [4:0]   RA1,
   output [31:0] RD1,

   input [4:0]   RA2,
   output [31:0] RD2,

   input [4:0]   WA,
   input [31:0]  WD,
   input         WE
   );

logic [31:0]    rmem [32];
logic [31:0]    rd1, rd2;

initial begin
    rmem[0] = '0;
end

// Use asynch. logic to forward writes to the read ports.
always @* begin
    if (WE & (RA1 == WA))
        rd1 = WD;
    else
        rd1 = rmem[RA1];
end

always @* begin
    if (WE & (RA2 == WA))
        rd2 = WD;
    else
        rd2 = rmem[RA2];
end

always @(posedge CLK) if (CE) begin
    if (WE)
        rmem[WA] <= WD;
end

assign RD1 = rd1;
assign RD2 = rd2;

`ifdef __ICARUS__
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
`endif

endmodule
