`timescale 1us / 1ns

// Memory access unit

// TODO:
// - Implement bus hold / TH(S) state
// - 16-bit Bus Fixed Mode (SIZ16B=1)
// - Add port for instruction fetch

module v810_mem
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Execution unit (EU) data bus
   input [31:0]  EDA,
   output [31:0] EDD_I,
   input [31:0]  EDD_O,
   input [1:0]   EDBC, // Byte Count - 1 (0=1, 1=2, 3=4)
   input [3:0]   EDBE, // Byte Enable
   input         EDWR, // Write / not Read
   input         EDREQ, // Access request
   output        EDACK, // Access acknowledge

   // External bus
   output [31:0] A,
   input [31:0]  D_I,
   output [31:0] D_O,
   output [3:0]  BEn, // Byte Enable
   output        DAn, // Data Access
   output        MRQn, // Memory ReQuesst
   output        RW, // Read / not Write
   output        BCYSTn, // Bus CYcle STart
   input         READYn,
   input         SZRQn // Bus SiZing ReQuest
);


//////////////////////////////////////////////////////////////////////
// Write buffer (WB): 2-slot FIFO
//
// EU writes first go into the write buffer, then to the bus MUX.

localparam [1:0] WB_NUM_SLOTS = 2'd2;

typedef struct packed {
    logic [31:0]    a;
    logic [31:0]    d;
    logic [1:0]     bc;
    logic [3:0]     be;
} wb_slot_t;

wb_slot_t       wb_slot [WB_NUM_SLOTS];
wb_slot_t       wb_in, wb_out;
logic           wb_rptr, wb_wptr, wb_rptr_n, wb_wptr_n;
logic           wb_read, wb_write;
logic           wb_full, wb_empty;

assign wb_wptr_n = wb_write ? wb_wptr + 1'd1 : wb_wptr;
assign wb_rptr_n = wb_read ? wb_rptr + 1'd1 : wb_rptr;

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        wb_rptr <= '0;
        wb_wptr <= '0;
        wb_full <= '0;
    end
    else begin
        wb_rptr <= wb_rptr_n;
        wb_wptr <= wb_wptr_n;
        wb_full <= (wb_wptr_n == wb_rptr_n) & (wb_full | wb_write);
    end
end

assign wb_empty = ~wb_full & (wb_wptr == wb_rptr);

always @(posedge CLK) if (CE) begin
    if (wb_write) begin
        wb_slot[wb_wptr] <= wb_in;
    end
end

always @* begin
    wb_out = wb_slot[wb_rptr];
end


//////////////////////////////////////////////////////////////////////
// Bus MUX
//
// In priority order:
// 1. The write buffer drains to the EBI
// 2. EU reads go straight to the EBI
//
// EU reads effectively flush the write buffer before starting.

// Set to bypass the write buffer.
bit             dbg_bypass_wb = '0;

logic           bm_sel_wb;      // 0=WB, 1=EU

logic           bm_eu_dack;

logic [31:0]    bm_ebi_a;
logic [31:0]    bm_ebi_do;
logic [1:0]     bm_ebi_bc;
logic [3:0]     bm_ebi_be;
logic           bm_ebi_wr;
logic           bm_ebi_req;
logic           bm_ebi_ack;

assign wb_write = ~dbg_bypass_wb & RESn & EDREQ & EDWR & ~wb_full;
assign wb_in = {EDA, EDD_O, EDBC, EDBE};

assign bm_sel_wb = ~wb_empty;

always @* begin
    if (bm_sel_wb) begin
        bm_ebi_a = wb_out.a;
        bm_ebi_do = wb_out.d;
        bm_ebi_bc = wb_out.bc;
        bm_ebi_be = wb_out.be;
        bm_ebi_wr = '1;
        bm_ebi_req = '1;

        wb_read = bm_ebi_ack;
        bm_eu_dack = wb_write;
    end
    else begin
        bm_ebi_a = EDA;
        bm_ebi_do = EDD_O;
        bm_ebi_bc = EDBC;
        bm_ebi_be = EDBE;
        bm_ebi_wr = EDWR;
        bm_ebi_req = EDREQ & (dbg_bypass_wb | ~EDWR);

        wb_read = '0;
        bm_eu_dack = (~dbg_bypass_wb & EDWR) ? wb_write : bm_ebi_ack;
    end
end

assign EDACK = bm_eu_dack;


//////////////////////////////////////////////////////////////////////
// External bus interface (EBI)

// External bus state
typedef enum bit [2:0] {
    // Cycles for byte/word or 1st halfword
    EBST_TI = 3'd0,             // idle
    EBST_T1,                    // address
    EBST_T2,                    // data
    EBST_TH,                    // hold
    // Cycles for 2nd halfword (fixed or dynamic 16-bit bus sizing only)
    EBST_TIS,                   // idle
    EBST_T1S,                   // address
    EBST_T2S,                   // data
    EBST_THS                    // hold
} ebst_t;

// Set to effectively make this transparent; T1 is skipped.
bit             dbg_bypass_ebi_t1 = '0;

ebst_t          ebstp, ebst;
logic           eb_word1, eb_word2, eb_words;
logic           eb_next_word;  // Last cycle of access
logic           eb_two_half;   // Access will take two halfword cycles
logic [3:0]     eb_be;
logic           eb_readyn_d;
logic           eb_two_half_d;
logic [15:0]    eb_rbuf1;
logic [31:0]    edd_i, d_o;
logic           eb_halfword_byte_in_upper_half;

assign eb_two_half = (bm_ebi_bc == 2'd3) & ~SZRQn;

// External bus state machine
always @* begin
    ebst = ebstp;

    case (ebstp)
        EBST_TI: begin
            if (bm_ebi_req)
                ebst = dbg_bypass_ebi_t1 ? EBST_T2 : EBST_T1;
        end
        EBST_T1: begin
            ebst = EBST_T2;
        end
        EBST_T2: begin
            if (~eb_readyn_d) begin
                if (eb_two_half_d)
                    ebst = dbg_bypass_ebi_t1 ? EBST_T2S : EBST_T1S;
                else if (bm_ebi_req)
                    ebst = dbg_bypass_ebi_t1 ? EBST_T2 : EBST_T1;
                else
                    ebst = EBST_TI;
            end
        end
        EBST_T1S: begin
            ebst = EBST_T2S;
        end
        EBST_T2S: begin
            if (~eb_readyn_d) begin
                if (bm_ebi_req)
                    ebst = dbg_bypass_ebi_t1 ? EBST_T2 : EBST_T1;
                else
                    ebst = EBST_TI;
            end
        end
    endcase
end

always @(posedge CLK) if (CE) begin
    if (~RESn)
        ebstp <= EBST_TI;
    else
        ebstp <= ebst;
end

always @(posedge CLK) if (CE) begin
    eb_readyn_d <= READYn;
    eb_two_half_d <= eb_two_half;
end

assign eb_word1 = (ebst == EBST_T1) | (ebst == EBST_T2);
assign eb_word2 = (ebst == EBST_T1S) | (ebst == EBST_T2S);
assign eb_words = eb_word1 | eb_word2;
assign eb_next_word = (((ebst == EBST_T2) & ~eb_two_half) |
                       (ebst == EBST_T2S)) & ~READYn;

// For dynamic sizing, all data moves through D[15:0].

always @* begin
    eb_be = bm_ebi_be;
    if (eb_word2)
        // Dynamic sizing: disable upper halfword BE on second bus cycle.
        eb_be[1:0] = '0;
end

always @(posedge CLK) if (CE) begin
    if (eb_word1 & eb_two_half & ~READYn)
        // Dynamic sizing: Buffer lower halfword in first bus cycle.
        eb_rbuf1 <= D_I[15:0];
end

assign eb_halfword_byte_in_upper_half = ~(bm_ebi_bc == 2'd3) & ~SZRQn &
                                        |bm_ebi_be[3:2];

always @* begin
    edd_i = D_I;
    if (eb_word2)
        // Dynamic sizing: Combine two halfwords, present to CPU.
        edd_i = {D_I[15:0], eb_rbuf1};
    else if (eb_halfword_byte_in_upper_half)
        // Dynamic sizing: Shift lower (input) to upper halfword for reads.
        edd_i[31:16] = D_I[15:0];
end

always @* begin
    d_o = bm_ebi_do;
    if (eb_word2 | eb_halfword_byte_in_upper_half)
        // Dynamic sizing: Shift upper to lower (output) halfword for writes.
        d_o[15:0] = bm_ebi_do[31:16];
end

assign A[31:2] = bm_ebi_a[31:2];
assign A[1] = eb_word2;
assign A[0] = '0;
assign D_O = d_o;
assign BEn = ~eb_be;
assign DAn = ~((ebst == EBST_T2) | (ebst == EBST_T2S));
assign MRQn = ~bm_ebi_req;
assign RW = MRQn | ~bm_ebi_wr;
assign BCYSTn = ~((ebst == EBST_T1) | (ebst == EBST_T1S));

assign EDD_I = edd_i;
assign bm_ebi_ack = eb_next_word;

endmodule
