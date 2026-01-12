// Instruction cache
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

// TODO:
// - dump (ICD)
// - restore (ICR)

import v810_pkg::*;

module v810_icache
   (
    input             RESn,
    input             CLK,
    input             CE, // global clock enable

    // Register interface
    output [31:0]     CHCW,
    input [31:0]      CHCW_WD,
    input             CHCW_WE,

    // Global instruction cache state
    output            ICMAINT,

    // Execution unit (EU) instruction bus
    input [31:0]      EUIA,
    output reg [31:0] EUID,
    input             EUIREQ, // Access request
    output reg        EUIACK, // Access acknowledge

    // Memory access unit instruction bus
    output [31:0]     ICIA,
    input [31:0]      ICID,
    output reg        ICIREQ, // Access request
    input             ICIACK // Access acknowledge
    );

typedef struct packed {
    logic [3:0] necrv;          // (reserved by NEC)
    logic [1:0] v;              // valid (1 bit per 4 bytes)
    logic [21:0] tag;
} tag_t;

chcw_t              chcw;
logic               tag_eq, hit, miss;
logic               fill;

//////////////////////////////////////////////////////////////////////
// Map address to tag / data memory index and subblock

// Tag = A[31:10], Index = A[9:3]
localparam int TAGW = 22;
localparam int TAGS = 10;
localparam int IDXW = 7;
localparam int IDXS = 3;
localparam int SUBW = 1;
localparam int SUBS = 2;

wire [TAGW-1:0]     euia_tag;
logic [IDXW-1:0]    euia_idx;
logic               euia_sub;

assign euia_tag = EUIA[TAGS+:TAGW];
assign euia_idx = EUIA[IDXS+:IDXW];
assign euia_sub = EUIA[SUBS];

//////////////////////////////////////////////////////////////////////
// Tag memory
//
// Tag memory is 128 tag entries.  Each entry corresponds to one data
// block or two data subblocks (2x 4 bytes) in data memory.

tag_t               tag_rd, tag_wd;
logic               tag_we, tag_we_fill;
logic [IDXW-1:0]    tag_ra, tag_wa;

localparam TAGRAM_DEPTH = 1<<IDXW;

v810_tagram #(.addr_width(IDXW), .data_width(28)) itagram
   (
    .rd_address(tag_ra),
    .rd_data(tag_rd),

    .wr_clock(CLK),
    .wr_en(tag_we),
    .wr_address(tag_wa),
    .wr_data(tag_wd)
    );

assign tag_ra = euia_idx;

wire [1:0] tag_rd_v = tag_rd.v;

always @* begin
    if (chcw.icc) begin
        tag_we = '1;
        tag_wa = chcw.cen[IDXW-1:0];
        tag_wd = '0;
    end
    else begin
        tag_we = tag_we_fill;
        tag_wa = tag_ra;
        tag_wd.tag = euia_tag;
        tag_wd.v = ({2{tag_eq}} & tag_rd_v) | {euia_sub, ~euia_sub};
        tag_wd.necrv = '0;
    end
end

// Cacheline hit test
assign tag_eq = (tag_rd.tag == euia_tag);
assign hit = EUIREQ & tag_eq & tag_rd_v[euia_sub];
assign miss = EUIREQ & ~hit;

//////////////////////////////////////////////////////////////////////
// Data memory
//
// Data memory is 256 subblocks of 4 bytes.

localparam DATAW = IDXW + SUBW;

logic [DATAW-1:0]   data_a;
logic [31:0]        data_rd, data_wd;
logic               data_we;

v810_dataram #(.addr_width(DATAW), .data_width(32)) idataram
   (
    .clock(CLK),

    .rd_address(data_a),
    .rd_data(data_rd),

    .wr_en(data_we),
    .wr_address(data_a),
    .wr_data(data_wd)
    );

assign data_a = {euia_idx, euia_sub};
assign data_wd = ICID;

//////////////////////////////////////////////////////////////////////
// Cacheline fill engine

always @* begin
    tag_we_fill = '0;
    data_we = '0;

    if (chcw.ice) begin
        if (CE) begin
            if (ICIREQ & ICIACK) begin
                tag_we_fill = fill;
                data_we = fill;
            end
        end
    end
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        fill <= '0;
    end
    else begin
        if (~fill & chcw.ice & miss)
            fill <= '1;
        else if (fill & ICIREQ & ICIACK)
            fill <= '0;
    end
end

//////////////////////////////////////////////////////////////////////
// Cache control

wire icmaint = chcw.icc | chcw.icd | chcw.icr;

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        chcw <= '0;
    end
    else begin
        if (chcw.icc) begin
            if ((chcw.cen >= 12'(TAGRAM_DEPTH-1)) | (chcw.cec <= 12'd1)) begin
                chcw.cen <= '0;
                chcw.cec <= '0;
                chcw.icc <= '0;
            end
            else begin
                chcw.cec <= chcw.cec - 1'd1;
                chcw.cen <= chcw.cen + 1'd1;
            end
        end

        if (CHCW_WE) begin
            chcw <= CHCW_WD;
        end
    end
end

assign CHCW = chcw;
assign ICMAINT = icmaint;

//////////////////////////////////////////////////////////////////////
// Instruction bus interfaces

always @* begin
    if (chcw.ice) begin
        EUID = data_rd[31:0];
        ICIREQ = fill;
        EUIACK = ~fill & hit;
    end
    else begin
        EUID = ICID;
        ICIREQ = EUIREQ;
        EUIACK = ICIACK;
    end
end

assign ICIA = EUIA;

endmodule
