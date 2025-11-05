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

   // Execution unit data bus
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
bit             dbg_bypass = '0;

ebst_t          stp, st;
wire            eb_word1, eb_word2, eb_words;
wire            eb_next_word;  // Last cycle of access
wire            eb_two_half;   // Access will take two halfword cycles
logic [3:0]     eb_be;
logic           eb_readyn_d;
logic           eb_two_half_d;
logic [15:0]    eb_rbuf1;
logic [31:0]    edd_i, d_o;
wire            eb_halfword_byte_in_upper_half;

assign eb_two_half = (EDBC == 2'd3) & ~SZRQn;

// External bus state machine
always @* begin
    st = stp;

    case (stp)
        EBST_TI: begin
            if (EDREQ)
                st = dbg_bypass ? EBST_T2 : EBST_T1;
        end
        EBST_T1: begin
            st = EBST_T2;
        end
        EBST_T2: begin
            if (~eb_readyn_d) begin
                if (eb_two_half_d)
                    st = dbg_bypass ? EBST_T2S : EBST_T1S;
                else if (EDREQ)
                    st = dbg_bypass ? EBST_T2 : EBST_T1;
                else
                    st = EBST_TI;
            end
        end
        EBST_T1S: begin
            st = EBST_T2S;
        end
        EBST_T2S: begin
            if (~eb_readyn_d) begin
                if (EDREQ)
                    st = dbg_bypass ? EBST_T2 : EBST_T1;
                else
                    st = EBST_TI;
            end
        end
    endcase
end

always @(posedge CLK) if (CE) begin
    if (~RESn)
        stp <= EBST_TI;
    else
        stp <= st;
end

always @(posedge CLK) if (CE) begin
    eb_readyn_d <= READYn;
    eb_two_half_d <= eb_two_half;
end

assign eb_word1 = (st == EBST_T1) | (st == EBST_T2);
assign eb_word2 = (st == EBST_T1S) | (st == EBST_T2S);
assign eb_words = eb_word1 | eb_word2;
assign eb_next_word = (((st == EBST_T2) & ~eb_two_half) |
                       (st == EBST_T2S)) & ~READYn;

// For dynamic sizing, all data moves through D[15:0].

always @* begin
    eb_be = EDBE;
    if (eb_word2)
        // Dynamic sizing: disable upper halfword BE on second bus cycle.
        eb_be[1:0] = '0;
end

always @(posedge CLK) if (CE) begin
    if (eb_word1 & eb_two_half & ~READYn)
        // Dynamic sizing: Buffer lower halfword in first bus cycle.
        eb_rbuf1 <= D_I[15:0];
end

assign eb_halfword_byte_in_upper_half = ~(EDBC == 2'd3) & ~SZRQn & |EDBE[3:2];

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
    d_o = EDD_O;
    if (eb_word2 | eb_halfword_byte_in_upper_half)
        // Dynamic sizing: Shift upper to lower (output) halfword for writes.
        d_o[15:0] = EDD_O[31:16];
end

assign A[31:2] = EDA[31:2];
assign A[1] = eb_word2;
assign A[0] = '0;
assign D_O = d_o;
assign BEn = ~eb_be;
assign DAn = ~((st == EBST_T2) | (st == EBST_T2S));
assign MRQn = ~EDREQ;
assign RW = MRQn | ~EDWR;
assign BCYSTn = ~((st == EBST_T1) | (st == EBST_T1S));

assign EDD_I = edd_i;
assign EDACK = eb_next_word;

endmodule
