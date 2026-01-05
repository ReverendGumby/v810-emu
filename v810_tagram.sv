// Cache tag RAM: Simple dual-port RAM with asynchronous read
//
// Adapted from https://github.com/MiSTer-devel/PSX_MiSTer/blob/75144be73c6da78b76c87ec4141262b07f8e2480/rtl/RamMLAB.vhd
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

altdpram
   #(
    .indata_reg("INCLOCK"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altdpram"),
    .outdata_reg("UNREGISTERED"),
    .ram_block_type("MLAB"),
    .rdaddress_reg("UNREGISTERED"),
    .rdcontrol_reg("UNREGISTERED"),
    .read_during_write_mode_mixed_ports("DONT_CARE"),
    .width(data_width),
    .widthad(addr_width),
    .wrcontrol_reg("INCLOCK")
    )
   altdpram
   (
	.data(wr_data),
	.inclock(wr_clock),
	.rdaddress(rd_address),
	.wraddress(wr_address),
	.wren(wr_en),
	.q(rd_data)
	);

endmodule
