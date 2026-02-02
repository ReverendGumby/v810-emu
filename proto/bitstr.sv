// MOVBSU prototype test
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

`timescale 1us / 1ns

module bitstr;

localparam BUFLEN = 8;

logic [4:0]     dbo, sbo;
int             cnt, dst, src;
logic [31:0]    tbuf[BUFLEN], rbuf[BUFLEN];

task ld(int i, output logic [31:0] v);
    #1 v = tbuf[i];
endtask

task st(int i, logic [31:0] v);
    #1 tbuf[i] = v;
endtask

task init_buf();
    tbuf[0] = 32'h01234567;
    tbuf[1] = 32'h89abcdef;
    tbuf[2] = 32'hfedcba98;
    tbuf[3] = 32'h76543210;
    tbuf[4] = 32'h01234567;
    tbuf[5] = 32'h89abcdef;
    tbuf[6] = 32'hfedcba98;
    tbuf[7] = 32'h76543210;
endtask

task calc_rbuf();
    init_buf();

    while (cnt) begin
    logic sb;
    logic [31:0] sw, dw;
        sw = tbuf[src];
        sb = (sw >> sbo) & 1;
        dw = tbuf[dst];
        dw &= ~(1 << dbo);
        dw |= (sb << dbo);
        tbuf[dst] = dw;

        if (&sbo) begin
            sbo = 0;
            src ++;
        end
        else
            sbo ++;

        if (&dbo) begin
            dbo = 0;
            dst ++;
        end
        else
            dbo ++;

        cnt--;
    end

    for (int i = 0; i < BUFLEN; i++)
        rbuf[i] = tbuf[i];
endtask

task calc_tbuf;
logic [31:0] sw, dw, mask;
logic [63:0] sr;
int          n;
    init_buf();

    if (sbo >= dbo) begin
        ld(src, sw);
        src ++;
        sr = {sw, sr[32+:32]};
    end

    // Partial first dest. word
    if (dbo) begin
        ld(src, sw);
        src ++;
        sr = {sw, sr[32+:32]};

        n = 32 - dbo;
        mask = -32'd1 >> n;
        if (n > cnt) begin
            n = cnt;
            mask |= (-32'd1 << (dbo + cnt));
        end
        ld(dst, dw);
        // (sbo-dbo) should be treated as unsigned.
        dw = (dw & mask) | (sr[5'(sbo-dbo)+:32] & ~mask);
        st(dst, dw);

        cnt -= n;
        sbo += n;
        dbo += n;
        dst ++;
    end

    while (cnt >= 32) begin
        ld(src, sw);
        src ++;
        sr = {sw, sr[32+:32]};

        assert(dbo == 0);
        ld(dst, dw);
        dw = sr[sbo+:32];
        st(dst, dw);

        cnt -= 32;
        dst ++;
    end

    // Partial last dest. word
    if (cnt) begin
        ld(src, sw);
        sr = {sw, sr[32+:32]};

        mask = -32'd1 << cnt;
        ld(dst, dw);
        dw = (dw & mask) | (sr[sbo+:32] & ~mask);
        st(dst, dw);

        cnt = 0;
    end
endtask


task test_one(input [4:0] in_dbo, input [4:0] in_sbo, input int in_cnt,
              input int in_dst, input int in_src);
bit diff;
    {dbo, sbo, cnt, dst, src} = {in_dbo, in_sbo, in_cnt, in_dst, in_src};
    calc_rbuf();
    //pdebug = 1;
    {dbo, sbo, cnt, dst, src} = {in_dbo, in_sbo, in_cnt, in_dst, in_src};
    calc_tbuf();

    diff = 0;
    for (int i = 0; i < BUFLEN; i++) begin
        $display("%x %x %d", rbuf[i], tbuf[i], rbuf[i] == tbuf[i]);
        assert(rbuf[i] == tbuf[i]);
        else
            diff = 1;
    end
    if (diff)
        $fatal(1, "Mismatch!");
endtask

task load_regs_all(int start, int bend, int num);
    src = start / 32;
    sbo = start[4:0];
    dst = bend / 32 + (BUFLEN / 2);
    dbo = bend[4:0];
    cnt = num;
endtask

task test_all;
bit diff;
static int iter = 0;
static int total = (BUFLEN * 32) / 2;
    for (int num = 1; num < total; num++) begin
        for (int start = 0; start < total - num + 1; start++) begin
            for (int bend = 0; bend < total - num + 1; bend++) begin
                iter++;
                load_regs_all(start, bend, num);
                calc_rbuf();
                load_regs_all(start, bend, num);
                calc_tbuf();

                diff = 0;
                for (int i = 0; i < BUFLEN; i++) begin
                    assert(rbuf[i] == tbuf[i]);
                    else
                        diff = 1;
                end
                if (diff) begin
                    load_regs_all(start, bend, num);
                    $display("dbo,sbo,cnt,dst,src: %1d %1d %1d %1d %1d", dbo, sbo, cnt, dst, src);
                    $fatal(1, "Mismatch!");
                end
            end
        end
    end
    $display("Passed %d tests", iter);
endtask

initial #0 begin
    $dumpfile("bitstr.vcd");
    //$dumpvars();

    //test_one(8, 4, 64, 5, 0);
    test_all();
end

endmodule


// Local Variables:
// compile-command: "iverilog -g2012 -s bitstr -o bitstr.vvp bitstr.sv && ./bitstr.vvp"
// End:
