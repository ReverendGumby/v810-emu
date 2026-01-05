// Cache data RAM: Simple dual-port RAM
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

module v810_tagram
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

altdpram
   #(
    .indata_reg("INCLOCK"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altdpram"),
    .outdata_reg("INCLOCK"),
    .ram_block_type("MLAB"),
    .rdaddress_reg("INCLOCK"),
    .rdcontrol_reg("INCLOCK"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .width(data_width),
    .widthad(addr_width),
    .wrcontrol_reg("INCLOCK")
    )
   altdpram
   (
	.data(wr_data),
	.inclock(clock),
	.rdaddress(rd_address),
	.wraddress(wr_address),
	.wren(wr_en),
	.q(rd_data)
	);

endmodule
