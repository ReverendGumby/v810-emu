`timescale 1us / 1us

module mulu_test;

logic [31:0] a, b;
logic [63:0] c;
logic        ov;

task mulu;
logic [31:0] x;
logic        cy;
    // Perform long multiplication on 16-bit groups:
    //
    //  ahal * bhbl = (al*bl) + ((ah*bh) << 32)
    //              + ((ah*bl) << 16) + ((al*bh) << 16)
    //
    #1 c = '0;
    #1 x = a[0+:16] * b[0+:16];
    #1 c[0+:32] = x;
    #1 x = a[16+:16] * b[16+:16];
    #1 c[32+:32] = x;
    #1 x = a[0+:16] * b[16+:16];
    //#1 c[16+:48] += {16'b0, x};
    #1 {cy, c[0+:32]} = c[0+:32] + {x[0+:16], 16'b0};
    #1 {cy, c[32+:32]} = c[32+:32] + {16'b0, x[16+:16]} + cy;
    #1 x = a[16+:16] * b[0+:16];
    //#1 c[16+:48] += {16'b0, x};
    #1 {cy, c[0+:32]} = c[0+:32] + {x[0+:16], 16'b0};
    #1 {cy, c[32+:32]} = c[32+:32] + {16'b0, x[16+:16]} + cy;
    ov = |c[32+:32];
endtask

task test(input [31:0] in_a, input [31:0] in_b);
logic [63:0] want_c;
    want_c = in_a * in_b;
    a = in_a;
    b = in_b;
    mulu();
    assert(c == want_c);
endtask

initial #0 begin
    c = '0;
    ov = '0;

    $dumpfile("mulu_test.vcd");
    $dumpvars();

    test(32'd1, 32'd1);
    test(32'd2, 32'd1);
    test(32'd1, 32'd2);
    test(32'd3, 32'd100);
    test(32'h7fff, 32'h7fff);
    test(32'hffff, 32'hffff);
    test(32'h10000, 32'h12345);
    test(32'h01234567, 32'h89abcdef);
    test(32'h89abcdef, 32'h01234567);
    test(32'h76543210, 32'hfedcba98);
    test(32'h7fffffff, 32'h7fffffff);
    test(32'hffffffff, 32'hffffffff);
    $finish();
end

endmodule

// Local Variables:
// compile-command: "iverilog -g2012 -grelative-include -s mulu_test -o mulu_test.vvp mulu_test.sv && ./mulu_test.vvp"
// End:
