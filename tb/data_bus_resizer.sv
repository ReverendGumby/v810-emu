module data_bus_resizer
  (
   input int           WS,
   input int           DW,

   input               CLK,
   input               CE,

   input               CTLR_DAn,
   input [3:0]         CTLR_BEn,
   output logic        CTLR_READYn,
   output              CTLR_SZRQn,
   output [31:0]       CTLR_DI,
   input [31:0]        CTLR_DO,

   input               MEM_nCE,
   output logic [31:0] MEM_DI,
   input [31:0]        MEM_DO
   );

logic           ready_w0, ready_w1;
logic [31:0]    ctlr_di;

// Emulate memory with one wait state
always @(posedge CLK) if (CE) begin
    ready_w1 <= ~ready_w1 & ~CTLR_DAn & ~MEM_nCE;
end

// Emulate memory with no wait states
assign ready_w0 = ~CTLR_DAn & ~MEM_nCE;

always @* begin
    case (WS)
        0: CTLR_READYn = ~ready_w0;
        1: CTLR_READYn = ~ready_w1;
        default: CTLR_READYn = 'X;
    endcase
end

assign CTLR_SZRQn = ~((DW == 16) & ~CTLR_READYn);

always @* begin
    if (DW == 32) begin
        MEM_DI = CTLR_DO;
        ctlr_di = MEM_DO;
    end
    else begin
        case (CTLR_BEn)
            4'b1110, 4'b1101, 4'b1100, 4'b0000: begin
                MEM_DI = {16'bz, CTLR_DO[15:0]};
                ctlr_di = {16'bz, MEM_DO[15:0]};
            end
            default: begin
                MEM_DI = {CTLR_DO[15:0], 16'bz};
                ctlr_di = {16'bz, MEM_DO[31:16]};
            end
        endcase
    end
end

assign CTLR_DI = ctlr_di;

endmodule
