// Cache data RAM: Simple dual-port RAM, testbench version
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

module v810_dataram
  #(parameter int    addr_width = 8,
	parameter int    data_width = 8
	)
   (
    input                   clock,

    input [addr_width-1:0]  rd_address,
    output [data_width-1:0] rd_data,

    input                   wr_en,
    input [addr_width-1:0]  wr_address,
    input [data_width-1:0]  wr_data
    );

reg [data_width-1:0] mem [0:(1<<addr_width)-1];
reg [data_width-1:0] rd_buf;

always @(posedge clock) begin
    rd_buf <= mem[rd_address];
    if (wr_en) begin
        mem[wr_address] <= wr_data;
    end
end

assign rd_data = rd_buf;

endmodule
