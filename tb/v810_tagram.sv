// Cache tag RAM: Simple dual-port RAM with asynchronous read, testbench version
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

module v810_tagram
  #(parameter int    addr_width = 8,
	parameter int    data_width = 8
	)
   (
    input [addr_width-1:0]  rd_address,
    output [data_width-1:0] rd_data,

    input                   wr_clock,
    input                   wr_en,
    input [addr_width-1:0]  wr_address,
    input [data_width-1:0]  wr_data
    );

reg [data_width-1:0] mem [0:(1<<addr_width)-1];

always @(posedge wr_clock) begin
    if (wr_en) begin
        mem[wr_address] <= wr_data;
    end
end

assign rd_data = mem[rd_address];

endmodule
