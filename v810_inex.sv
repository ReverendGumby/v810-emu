// Interrupt / exception multiplexer
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

import v810_pkg::*;

module v810_inex
  (
   input         RESn,
   input         CLK,
   input         CE, // global clock enable

   input         psw_t PSW, // Processor Status Word

   input         INT, // Interrupt
   input [3:0]   INTVn, // Interrupt Level
   input         NMIn, // Non-maskable Interrupt

   input         EUF, // execution exception flag
   input [15:0]  EUCCB, // execution exception code base
   input [4:0]   EUCCO, // execution exception code offset
   input         ADTRF, // address trap exception flag
   output        IF, // Interrupt / Exception Flag
   output        NP, // Non-maskable int. or duplexed exc.
   output [3:0]  IEL, // Maskable int. enable level (for PSW.I)
   output [15:0] CC, // Exception Code
   output [31:0] HA, // Handler Address
   input         ACK // execution unit acknowledge
   );

logic           ifl;
logic           resn_d, nmin_d;
logic           resp, nmip, intp, adtrp, eup; // pending
logic           resc, nmic, intc, adtrc, euc; // clear flag
logic           resf, nmif, intf, adtrf, euf; // flag (active)
logic [3:0]     intv;
logic [15:0]    eucc;
logic [15:0]    cc, cc_d;
logic           np, np_d;
logic [3:0]     iel, iel_d;
logic [31:0]    ha, ha_d;

always @(posedge CLK) if (CE) begin
    resn_d <= RESn;
    nmin_d <= NMIn;

    if (~RESn) begin
        resp <= '0;
        nmip <= '0;
    end
    else begin
        // RESET exception is on RESn rising edge.
        resp <= (resp | (~resn_d & RESn)) & ~resc;
        // NMI interrupt is on NMIn falling edge.
        nmip <= (nmip | (nmin_d & ~NMIn)) & ~nmic;
    end
end

// A non-maskable interrupt request that occurs while NP=1 is
// internally held by the processor.  Nothing masks a reset.
assign resf = resp;
assign nmif = nmip & ~(resp | PSW.np);

// Interrupt request is detected when all three of the following
// conditions are true:
wire inte = ~(PSW.np | PSW.ep | PSW.id); // All these PSW flags are '0'
wire intl = ~INTVn >= PSW.i; // Level meets or exceeds PSW enable level
wire inti = INT;             // INT pin is active
wire intr = inte & intl & inti;

// The interrupt request and level are latched on detection and held
// while the PSW test is true.
always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        intp <= '0;
        intv <= '0;
    end
    else begin
        if (~intp & intr) begin
            intp <= '1;
            intv <= ~INTVn;
        end
        else if (intp & ~inte) begin
            intp <= '0;
            intv <= '0;
        end
    end
end

assign intf = intp;
wire [15:0] intcc = {8'hFE, intv, 4'h0};

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        adtrf <= '0;
        euf <= '0;
        eucc <= '0;
    end
    else begin
        // EU raises these exceptions.
        adtrf <= (adtrf | ADTRF) & ~adtrc;
        euf <= (euf | EUF) & ~euc;
        if (EUF)
            eucc <= EUCCB + 16'(EUCCO);
    end
end

// Merge all interrupt / exception flags
assign IF = resf | intf | nmif | adtrf | euf;

// Priority encoder yields condition code and clear flag signal
always @* begin
    {resc, nmic, intc, adtrc, euc} = '0;
    cc = '0;

    casez ({resf, nmif, intf, adtrf, euf})
        5'b1????: begin         // resf
            cc = 16'hFFF0;
            resc = resf & ACK;
        end
        5'b01???: begin         // nmif
            cc = 16'hFFD0;
            nmic = nmif & ACK;
        end
        5'b001??: begin         // intf
            cc = intcc;
            intc = intf & ACK;
        end
        5'b0001?: begin         // adtrf
            cc = 16'hFFC0;
            adtrc = adtrf & ACK;
        end
        5'b00001: begin         // euf
            cc = eucc;
            euc = euf & ACK;
        end
        default: ;
    endcase
end

always @* begin
    ha = {16'hFFFF, cc};
    if (PSW.ep)                 // duplexed exception
        ha[15:0] = 16'hFFD0;
    else if (ha[7:4] == 4'h7)   // codes FF6x-FF7x use address 'FF60
        ha[7:4] = 4'h6;
    ha[3:0] = 4'h0;
end

assign np = resf | nmif | PSW.ep;

always @* begin
    iel = '0;
    if (intf)
        iel = (intv == 4'd15) ? 4'd15 : (intv + 1'd1);
end

always @(posedge CLK) if (CE) begin
    if (~RESn) begin
        cc_d <= '0;
        np_d <= '0;
        iel_d <= '0;
        ha_d <= '0;
    end
    else if (IF & ACK) begin
        cc_d <= cc;
        np_d <= np;
        iel_d <= iel;
        ha_d <= ha;
    end
end

assign CC = cc_d;
assign NP = np_d;
assign IEL = iel_d;
assign HA = ha_d;

endmodule
