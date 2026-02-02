// MOVBSU prototype test
//
// Copyright (c) 2026 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define BUFLEN 8

const uint32_t cbuf[BUFLEN] = {
    0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210,
    0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210,
    //0x02468ace, 0x13579bdf, 0xfdb97531, 0xeca86420,
    //0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    //0x00000000, 0x00000000, 0x00000000, 0x00000000,
};

uint32_t rbuf[BUFLEN];
uint32_t tbuf[BUFLEN];
uint32_t *buf;

uint32_t dbo, sbo, cnt, dst, src;

int pdebug = 0;

uint32_t reverse_bits(uint32_t in)
{
    uint32_t out = 0;
    for (int i = 0; i < 32; i++) {
        out = (out << 1) | (in & 1);
        in >>= 1;
    }
    return out;
}

void init_buf(uint32_t *p)
{
    buf = p;
    for (int i = 0; i < BUFLEN; i++) {
        buf[i] = reverse_bits(cbuf[i]);
    }
}

uint32_t ld(uint32_t a)
{
    assert (a < BUFLEN);
    if (pdebug)
        printf("ld(%d)\n", a);
    return buf[a];
}

void st(uint32_t a, uint32_t v)
{
    assert (a < BUFLEN);
    if (pdebug)
        printf("st(%d, 0x%08x)\n", a, v);
    buf[a] = v;
}

void calc_rbuf()
{
    init_buf(rbuf);

    while (cnt--) {
        uint32_t sb = (ld(src) >> sbo) & 1;
        uint32_t dw = ld(dst);
        dw &= ~(1 << dbo);
        dw |= (sb << dbo);
        st(dst, dw);

        if (++sbo == 32) {
            sbo = 0;
            src ++;
        }
        if (++dbo == 32) {
            dbo = 0;
            dst ++;
        }
    }
}

void calc_tbuf()
{
    init_buf(tbuf);

    uint32_t sw, dw;
    int first_word = 1;

    while (cnt) {
        int last_word = cnt + dbo < 32;

        if (first_word || !sbo) {
            sw = ld(src);
            src ++;
        }
        if (first_word) {
            dw = ld(dst);
        }
        else if (last_word && !dbo) {
            dw = ld(dst);
        }
        first_word = 0;

        if (pdebug)
            printf("%2d %2d %2d %2d %2d : %08x %08x\n",
                   dbo, sbo, cnt, dst, src,
                   reverse_bits(sw), reverse_bits(dw));

        int n = 32 - sbo;
        if (n > 32 - dbo)
            n = 32 - dbo;
        if (n > cnt)
            n = cnt;
        uint32_t m = n == 32 ? -1 : ((1 << n) - 1) << dbo;
        uint32_t v = (sw >> sbo) << dbo;
        if (pdebug)
            printf("m=%08x v=%08x\n", reverse_bits(m), reverse_bits(v));
        dw &= ~m;
        dw |= (v & m);
        sbo += n;
        dbo += n;
        cnt -= n;

        if (sbo == 32) {
            sbo = 0;
        }
        if ((dbo == 32) || (!cnt && dbo)) {
            st(dst, dw);
            dst ++;
            dbo = 0;
        }
    }
}

void dump()
{
    // Dump data, but print bits in reverse
    for (int i = 0; i < BUFLEN; i++) {
        printf("%08x ", reverse_bits(buf[i]));
        if ((i + 1) % 4 == 0)
            printf("\n");
    }
    printf("\n");
}

void load_regs_one(int argc, char *argv[])
{
    dbo = strtoul(argv[1], NULL, 0);       // r26: string dest. start bit offset
    sbo = strtoul(argv[2], NULL, 0);       // r27: string src. start bit offset
    cnt = strtoul(argv[3], NULL, 0);       // r28: string length (bits)
    dst = strtoul(argv[4], NULL, 0);       // r29: string dest. addr.
    src = strtoul(argv[5], NULL, 0);       // r30: string src. addr.
}

int test_one(int argc, char *argv[])
{
    load_regs_one(argc, argv);
    calc_rbuf();
    dump();
    pdebug = 1;
    load_regs_one(argc, argv);
    calc_tbuf();
    dump();

    if (memcmp(rbuf, tbuf, BUFLEN * sizeof(uint32_t)) != 0) {
        printf("Mismatch!\n");
        return 1;
    }

    printf("Match.\n");
    return 0;
}

void load_regs_all(uint32_t start, uint32_t end, uint32_t num)
{
    src = start / 32;
    sbo = start % 32;
    dst = end / 32 + (BUFLEN / 2);
    dbo = end % 32;
    cnt = num;
}

int test_all(void)
{
    int iter = 0;
    const uint32_t total = (BUFLEN * 32) / 2;
    for (uint32_t num = 1; num < total; num++) {
        for (uint32_t start = 0; start < total - num + 1; start++) {
            for (uint32_t end = 0; end < total - num + 1; end++) {
                iter++;
                load_regs_all(start, end, num);
                calc_rbuf();
                load_regs_all(start, end, num);
                calc_tbuf();

                if (memcmp(rbuf, tbuf, BUFLEN * sizeof(uint32_t)) != 0) {
                    load_regs_all(start, end, num);
                    printf("Mismatch!\n");
                    printf("dbo,sbo,cnt,dst,src: %d %d %d %d %d\n", dbo, sbo, cnt, dst, src);
                    return 1;
                }
            }
        }
    }
    printf("Passed %d tests\n", iter);
    return 0;
}

int main(int argc, char *argv[])
{
    if (argc > 5)
        return test_one(argc, argv);
    else
        return test_all();
}


// Local Variables:
// compile-command: "gcc -o bitstr bitstr.c && ./bitstr 0 0 32 4 0"
// End:
