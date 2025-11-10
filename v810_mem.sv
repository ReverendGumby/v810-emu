`timescale 1us / 1ns

// Memory access unit

// TODO:
// - Implement bus hold / TH(S) state
// - 16-bit Bus Fixed Mode (SIZ16B=1)

module v810_mem
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   // Execution unit (EU) instruction bus
   input [31:0]  EUIA,
   output [31:0] EUID,
   input         EUIREQ, // Access request
   output        EUIACK, // Access acknowledge

   // Execution unit (EU) data bus
   input [31:0]  EUDA,
   output [31:0] EUDD_I,
   input [31:0]  EUDD_O,
   input [1:0]   EUDBC, // Byte Count - 1 (0=1, 1=2, 3=4)
   input [3:0]   EUDBE, // Byte Enable
   input         EUDWR, // Write / not Read
   input         EUDREQ, // Access request
   output        EUDACK, // Access acknowledge

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
// 2. EU data reads go straight to the EBI
// 3. EU instruction reads go straight to the EBI
//
// EU reads effectively flush the write buffer before starting.

// Set to bypass the write buffer.
bit             dbg_bypass_wb = '0;

logic           bm_sel_wb;      // 1=WB, 0=EU
logic           bm_sel_eud, bm_sel_eud_d;
logic           bm_sel_eui, bm_sel_eui_d;

logic [31:0]    bm_ebi_a;
logic [31:0]    bm_ebi_di, bm_ebi_do;
logic [1:0]     bm_ebi_bc;
logic [3:0]     bm_ebi_be;
logic           bm_ebi_wr;
logic           bm_ebi_req;
logic           bm_ebi_ack;

assign wb_write = ~dbg_bypass_wb & RESn & EUDREQ & EUDWR & ~wb_full;
assign wb_in = {EUDA, EUDD_O, EUDBC, EUDBE};
assign wb_read = bm_sel_wb & bm_ebi_ack;

assign bm_sel_wb = ~wb_empty;
assign bm_sel_eud = RESn & (bm_sel_eud_d | (~bm_sel_eui_d & EUDREQ));
assign bm_sel_eui = RESn & (bm_sel_eui_d | (~bm_sel_eud_d & ~EUDREQ & EUIREQ));

always @(posedge CLK) if (CE) begin
    bm_sel_eud_d <= bm_sel_eud & ~EUDACK;
    bm_sel_eui_d <= bm_sel_eui & ~EUIACK;
end

always @* begin
    if (bm_sel_wb) begin
        bm_ebi_a = wb_out.a;
        bm_ebi_do = wb_out.d;
        bm_ebi_bc = wb_out.bc;
        bm_ebi_be = wb_out.be;
        bm_ebi_wr = '1;
        bm_ebi_req = '1;
    end
    else if (bm_sel_eud) begin
        bm_ebi_a = EUDA;
        bm_ebi_do = EUDD_O;
        bm_ebi_bc = EUDBC;
        bm_ebi_be = EUDBE;
        bm_ebi_wr = EUDWR;
        bm_ebi_req = EUDREQ & (dbg_bypass_wb | ~EUDWR);
    end
    else /*if (bm_sel_eui)*/ begin
        bm_ebi_a = EUIA;
        bm_ebi_do = 'X;
        bm_ebi_bc = 2'd3;
        bm_ebi_be = '1;
        bm_ebi_wr = '0;
        bm_ebi_req = EUIREQ;
    end
end

assign EUIACK = ~(bm_sel_wb | bm_sel_eud) & bm_ebi_ack;
assign EUID = ~(bm_sel_wb | bm_sel_eud) ? bm_ebi_di : 'X;

assign EUDACK = (bm_sel_wb | (~dbg_bypass_wb & EUDWR)) ?
                wb_write : EUDREQ & bm_ebi_ack;
assign EUDD_I = (~bm_sel_wb & bm_sel_eud) ? bm_ebi_di : 'X;


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
logic [31:0]    eb_di, eb_do;
logic           eb_halfword_byte_in_upper_half;
logic [31:0]    eb_a_cur;

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
    eb_di = D_I;
    if (eb_word2)
        // Dynamic sizing: Combine two halfwords, present to CPU.
        eb_di = {D_I[15:0], eb_rbuf1};
    else if (eb_halfword_byte_in_upper_half)
        // Dynamic sizing: Shift lower (input) to upper halfword for reads.
        eb_di[31:16] = D_I[15:0];
end

always @* begin
    eb_do = bm_ebi_do;
    if (eb_word2 | eb_halfword_byte_in_upper_half)
        // Dynamic sizing: Shift upper to lower (output) halfword for writes.
        eb_do[15:0] = bm_ebi_do[31:16];
end

assign A[31:2] = bm_ebi_a[31:2];
assign A[1] = eb_word2;
assign A[0] = '0;
assign D_O = eb_do;
assign BEn = ~eb_be;
assign DAn = ~((ebst == EBST_T2) | (ebst == EBST_T2S));
assign MRQn = ~bm_ebi_req;
assign RW = MRQn | ~bm_ebi_wr;
assign BCYSTn = ~((ebst == EBST_T1) | (ebst == EBST_T1S));

assign bm_ebi_ack = eb_next_word;
assign bm_ebi_di = eb_di;

// Debugging assertions
always @(posedge CLK) if (CE) begin
    if (~dbg_bypass_ebi_t1 & ~MRQn) begin
        if (~BCYSTn)
            eb_a_cur <= A;
        else begin
            assert(A === eb_a_cur);
            else
                @(posedge CLK) $fatal(1, "A must not change mid-access");
        end
    end
end

endmodule
