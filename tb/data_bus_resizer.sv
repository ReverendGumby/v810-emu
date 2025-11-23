module data_bus_resizer
  (
   input int           WS,
   input int           DW,

   input               CLK,
   input               CE,

   input               CTLR_DAn,
   input [3:0]         CTLR_BEn,
   output              CTLR_READYn,
   output              CTLR_SZRQn,
   output [31:0]       CTLR_DI,
   input [31:0]        CTLR_DO,

   input               MEM_nCE,
   output logic [31:0] MEM_DI,
   input [31:0]        MEM_DO
   );

logic           ready, ready_w0, ready_w1;
logic [31:0]    ctlr_di;

// Emulate memory with one wait state
always @(posedge CLK) if (CE) begin
    ready_w1 <= ~ready_w1 & ~CTLR_DAn;
end

// Emulate memory with no wait states
assign ready_w0 = '1;

always @* begin
    case (WS)
        0: ready = ready_w0;
        1: ready = ready_w1;
        default: ready = 'X;
    endcase
    ready &= ~CTLR_DAn;
end

assign CTLR_READYn = MEM_nCE | ~ready;
assign CTLR_SZRQn = MEM_nCE | ~((DW == 16) & ready);

always @* begin
    if (DW == 32) begin
        MEM_DI = CTLR_DO;
        ctlr_di = MEM_DO;
    end
    else begin
        case (CTLR_BEn)
            4'b1110, 4'b1101, 4'b1100, 4'b0000: begin
                MEM_DI = {16'bx, CTLR_DO[15:0]};
                ctlr_di = {16'bx, MEM_DO[15:0]};
            end
            default: begin
                MEM_DI = {CTLR_DO[15:0], 16'bx};
                ctlr_di = {16'bx, MEM_DO[31:16]};
            end
        endcase
    end
end

assign CTLR_DI = ctlr_di;

endmodule
