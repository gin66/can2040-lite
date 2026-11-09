// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_test_units.c — PC-based host tests for the portable CAN protocol core
// and the frame builder.
//
// Build and run: make -C tests/pc_based test

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "can2040_lite_proto.h"
#include "can2040_lite.h"
#include "can_frame_build.h"

static int g_passed;
static int g_failed;
static int g_test_failed;

#define TEST_BEGIN(name) do { \
    printf("  TEST: %s ... ", (name)); \
    fflush(stdout); \
    g_test_failed = 0; \
} while (0)

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("\n    FAIL: %s (line %d)\n", #cond, __LINE__); \
        g_test_failed = 1; \
    } \
} while (0)

#define TEST_END() do { \
    if (g_test_failed) { \
        g_failed++; \
        printf("    -> FAILED\n"); \
    } else { \
        g_passed++; \
        printf("PASS\n"); \
    } \
} while (0)

static uint32_t get_field(const uint8_t *buf, uint32_t start, uint32_t nbits) {
    uint32_t v = 0;
    uint32_t i;
    for (i = 0; i < nbits; i++)
        v = (v << 1) | bits_get(buf, start + i);
    return v;
}

static int max_run(const uint8_t *buf, uint32_t nbits) {
    uint32_t run = 0;
    uint32_t best = 0;
    uint8_t last = 0;
    uint32_t i;
    for (i = 0; i < nbits; i++) {
        uint8_t bit = bits_get(buf, i);
        if (run > 0 && bit == last)
            run++;
        else {
            run = 1;
            last = bit;
        }
        if (run > best)
            best = run;
    }
    return (int)best;
}

static void fill_std(can_frame_t *f, uint32_t id, uint8_t dlc,
                     const uint8_t *data) {
    memset(f, 0, sizeof(*f));
    f->id = id;
    f->dlc = dlc;
    if (data)
        memcpy(f->data, data, dlc);
}

/****************************************************************
 * 2.1 Bit helpers
 ****************************************************************/
static void test_bit_helpers(void) {
    uint8_t buf[4];
    uint32_t i;
    int ok;

    TEST_BEGIN("bit get/put roundtrip across byte boundaries");
    memset(buf, 0, sizeof(buf));
    for (i = 0; i < 32; i++)
        bits_put(buf, i, (uint8_t)(((i * 7u) + (i / 3u)) & 1u));
    ok = 1;
    for (i = 0; i < 32; i++)
        if (bits_get(buf, i) != (uint8_t)(((i * 7u) + (i / 3u)) & 1u))
            ok = 0;
    CHECK(ok);
    TEST_END();

    TEST_BEGIN("bit 0 is MSB of byte 0");
    memset(buf, 0, sizeof(buf));
    bits_put(buf, 0, 1);
    bits_put(buf, 8, 1);
    bits_put(buf, 15, 1);
    CHECK(buf[0] == 0x80);
    CHECK(buf[1] == 0x81);
    TEST_END();

    TEST_BEGIN("bits_clear");
    memset(buf, 0xff, sizeof(buf));
    bits_clear(buf, 32);
    CHECK(buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0);
    TEST_END();
}

/****************************************************************
 * 2.2 Stuffing
 ****************************************************************/
static void test_stuffing(void) {
    uint8_t in[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t out[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t back[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t n;
    uint32_t u;
    uint32_t i;

    TEST_BEGIN("stuff after five identical bits");
    bits_clear(in, 8);
    for (i = 0; i < 5; i++)
        bits_put(in, i, 0);
    bits_put(in, 5, 1);
    n = bit_stuff(in, 6, out, 32);
    CHECK(n == 7);
    CHECK(bits_get(out, 0) == 0 && bits_get(out, 4) == 0);
    CHECK(bits_get(out, 5) == 1); // inserted stuff bit
    CHECK(bits_get(out, 6) == 1); // original data bit
    TEST_END();

    TEST_BEGIN("ten zeros get two stuff bits");
    bits_clear(in, 16);
    n = bit_stuff(in, 10, out, 32);
    CHECK(n == 12);
    CHECK(bits_get(out, 5) == 1);
    CHECK(bits_get(out, 11) == 1);
    TEST_END();

    TEST_BEGIN("alternating bits need no stuffing");
    for (i = 0; i < 16; i++)
        bits_put(in, i, (uint8_t)(i & 1u));
    n = bit_stuff(in, 16, out, 32);
    CHECK(n == 16);
    TEST_END();

    TEST_BEGIN("stuff/unstuff roundtrip (std/ext/RTR)");
    {
        static const uint8_t d[8] = {0x00, 0x00, 0xff, 0xff,
                                     0x55, 0xaa, 0x12, 0x34};
        can_frame_t f;
        can_build_info_t info;
        uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
        int ok = 1;

        fill_std(&f, 0x123, 8, d);
        can_frame_build(&f, VALID_FRAME, 0, bits,
                        CAN2040_LITE_MAX_FRAME_BITS, &info);
        u = bit_unstuff(bits, info.stuffed_bits, back,
                        CAN2040_LITE_MAX_FRAME_BITS);
        ok = ok && (u == info.unstuffed_bits);

        fill_std(&f, 0x18febb11, 8, d);
        f.flags = CAN_FRAME_EXTENDED;
        can_frame_build(&f, VALID_FRAME, 0, bits,
                        CAN2040_LITE_MAX_FRAME_BITS, &info);
        u = bit_unstuff(bits, info.stuffed_bits, back,
                        CAN2040_LITE_MAX_FRAME_BITS);
        ok = ok && (u == info.unstuffed_bits);

        fill_std(&f, 0x456, 0, NULL);
        f.flags = CAN_FRAME_RTR;
        can_frame_build(&f, VALID_FRAME, 0, bits,
                        CAN2040_LITE_MAX_FRAME_BITS, &info);
        u = bit_unstuff(bits, info.stuffed_bits, back,
                        CAN2040_LITE_MAX_FRAME_BITS);
        ok = ok && (u == info.unstuffed_bits);

        CHECK(ok);
    }
    TEST_END();

    TEST_BEGIN("six identical bits are a stuff violation");
    bits_clear(in, 8);
    n = 6; // all zeros
    u = bit_unstuff(in, n, back, CAN2040_LITE_MAX_FRAME_BITS);
    CHECK(u == CAN_BITS_ERROR);
    TEST_END();

    TEST_BEGIN("bit_stuff overflow is reported");
    bits_clear(in, 16);
    n = bit_stuff(in, 16, out, 5); // cannot fit
    CHECK(n == CAN_BITS_ERROR);
    TEST_END();
}

/****************************************************************
 * 2.3 CRC
 ****************************************************************/
static void test_crc(void) {
    static const uint8_t d1[8] = {0x12, 0x34, 0x56, 0x78,
                                   0x9a, 0xbc, 0xde, 0xf0};
    static const uint8_t d2[8] = {0xde, 0xad, 0xbe, 0xef,
                                   0x12, 0x34, 0x56, 0x78};
    uint8_t buf[CAN2040_LITE_MAX_FRAME_BYTES];
    can_frame_t f;
    can_build_info_t info;
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];

    TEST_BEGIN("can2040 table recurrence");
    CHECK(crc_can15_update(0, 0x00) == 0x0000);
    CHECK(crc_can15_update(0, 0x01) == 0x4599);
    TEST_END();

    TEST_BEGIN("bit-level CRC equals byte-wise reference");
    {
        uint32_t n;
        int ok = 1;
        for (n = 1; n <= 8; n++) {
            uint32_t i;
            uint16_t cb;
            uint16_t cbit;
            for (i = 0; i < n; i++)
                bits_put(buf, i * 8u, 0); // ensure defined
            memset(buf, 0, sizeof(buf));
            for (i = 0; i < n; i++) {
                uint8_t v = (uint8_t)(d1[i] ^ (uint8_t)(n * 0x11u));
                uint32_t b;
                for (b = 0; b < 8; b++)
                    bits_put(buf, i * 8u + b,
                             (uint8_t)((v >> (7 - b)) & 1u));
            }
            cb = crc_can15_calc(buf, n);
            cbit = crc_can15_bits(0, buf, n * 8u);
            ok = ok && (cb == cbit);
        }
        CHECK(ok);
    }
    TEST_END();

    TEST_BEGIN("golden CRC: std 0x123 dlc 8");
    fill_std(&f, 0x123, 8, d1);
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(info.crc == 0x1738);
    TEST_END();

    TEST_BEGIN("golden CRC: ext 0x18FEBB11 dlc 8");
    fill_std(&f, 0x18febb11, 8, d2);
    f.flags = CAN_FRAME_EXTENDED;
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(info.crc == 0x4e1a);
    TEST_END();
}

/****************************************************************
 * 2.4 Frame layout
 ****************************************************************/
static void test_frame_layout(void) {
    static const uint8_t d[8] = {0x12, 0x34, 0x56, 0x78,
                                  0x9a, 0xbc, 0xde, 0xf0};
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t un[CAN2040_LITE_MAX_FRAME_BYTES];
    can_frame_t f;
    can_build_info_t info;
    uint32_t ulen;
    uint32_t i;

    TEST_BEGIN("standard frame field layout");
    fill_std(&f, 0x123, 8, d);
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    ulen = bit_unstuff(bits, info.stuffed_bits, un,
                       CAN2040_LITE_MAX_FRAME_BITS);
    CHECK(ulen == info.unstuffed_bits);
    CHECK(bits_get(un, 0) == 0);                    // SOF dominant
    CHECK(get_field(un, 1, 11) == 0x123);           // ID
    CHECK(bits_get(un, 12) == 0);                   // RTR
    CHECK(bits_get(un, 13) == 0);                   // IDE
    CHECK(bits_get(un, 14) == 0);                   // r0
    CHECK(get_field(un, 15, 4) == 8);               // DLC
    TEST_END();

    TEST_BEGIN("extended frame field layout");
    fill_std(&f, 0x18febb11, 8, d);
    f.flags = CAN_FRAME_EXTENDED;
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    ulen = bit_unstuff(bits, info.stuffed_bits, un,
                       CAN2040_LITE_MAX_FRAME_BITS);
    CHECK(ulen == info.unstuffed_bits);
    CHECK(bits_get(un, 0) == 0);
    CHECK(get_field(un, 1, 11) == ((0x18febb11u >> 18) & 0x7ffu));
    CHECK(bits_get(un, 12) == 1);                   // SRR
    CHECK(bits_get(un, 13) == 1);                   // IDE
    CHECK(get_field(un, 14, 18) == (0x18febb11u & 0x3ffffu));
    CHECK(bits_get(un, 32) == 0);                   // RTR
    CHECK(bits_get(un, 33) == 0);                   // r1
    CHECK(bits_get(un, 34) == 0);                   // r0
    CHECK(get_field(un, 35, 4) == 8);               // DLC
    TEST_END();

    TEST_BEGIN("DLC sweep 0..8 (std/ext/RTR)");
    {
        int ok = 1;
        uint8_t dlc;
        for (dlc = 0; dlc <= 8; dlc++) {
            can_parsed_t p;
            int pass;
            for (pass = 0; pass < 3; pass++) {
                fill_std(&f, 0x155, dlc, d);
                f.flags = 0;
                if (pass == 1)
                    f.flags = CAN_FRAME_EXTENDED;
                if (pass == 2)
                    f.flags = CAN_FRAME_RTR;
                can_frame_build(&f, VALID_FRAME, 0, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
                ok = ok && (info.total_bits == info.stuffed_bits + 13u);
                ok = ok && (can_frame_decode(bits, info.total_bits, &p)
                            == VALID_FRAME);
                ok = ok && (p.id == f.id);
                ok = ok && (p.dlc == dlc);
                ok = ok && (p.is_extended ==
                            ((f.flags & CAN_FRAME_EXTENDED) ? 1 : 0));
                ok = ok && (p.is_rtr ==
                            ((f.flags & CAN_FRAME_RTR) ? 1 : 0));
            }
        }
        CHECK(ok);
    }
    TEST_END();

    TEST_BEGIN("no run > 5 in the stuffed region");
    fill_std(&f, 0x000, 8, d);
    f.flags = CAN_FRAME_EXTENDED;
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(max_run(bits, info.stuffed_bits) <= 5);
    TEST_END();

    TEST_BEGIN("tail is recessive except the ACK slot");
    fill_std(&f, 0x123, 8, d);
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    for (i = info.stuffed_bits; i < info.total_bits; i++)
        CHECK(bits_get(bits, i) == 1);
    TEST_END();

    TEST_BEGIN("decode reassembles payload and flags");
    {
        can_parsed_t p;
        fill_std(&f, 0x18febb11, 8, d);
        f.flags = CAN_FRAME_EXTENDED;
        can_frame_build(&f, VALID_FRAME, 1, bits,
                        CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(can_frame_decode(bits, info.total_bits, &p) == VALID_FRAME);
        CHECK(p.ack == 1);
        CHECK(p.id == 0x18febb11);
        CHECK(p.is_extended == 1);
        CHECK(p.dlc == 8);
        CHECK(memcmp(p.data, d, 8) == 0);
        CHECK(p.crc == p.crc_calc);
    }
    TEST_END();

    TEST_BEGIN("RTR carries no data");
    {
        can_parsed_t p;
        fill_std(&f, 0x456, 8, d);
        f.flags = CAN_FRAME_RTR;
        can_frame_build(&f, VALID_FRAME, 0, bits,
                        CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(can_frame_decode(bits, info.total_bits, &p) == VALID_FRAME);
        CHECK(p.is_rtr == 1);
        CHECK(p.dlc == 8);
        CHECK(info.unstuffed_bits == 19 + 15); // header + CRC, no data
    }
    TEST_END();
}

/****************************************************************
 * 2.5 Invalid frames
 ****************************************************************/
static void test_invalid_frames(void) {
    static const uint8_t d[8] = {0xde, 0xad, 0xbe, 0xef,
                                  0x12, 0x34, 0x56, 0x78};
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_frame_t f;
    can_build_info_t info;
    can_parsed_t p;

    TEST_BEGIN("valid frame decodes cleanly");
    fill_std(&f, 0x123, 8, d);
    can_frame_build(&f, VALID_FRAME, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(can_frame_decode(bits, info.total_bits, &p) == VALID_FRAME);
    TEST_END();

    TEST_BEGIN("CRC error is reported");
    can_frame_build(&f, INVALID_CRC, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(can_frame_decode(bits, info.total_bits, &p) == INVALID_CRC);
    CHECK(p.crc != p.crc_calc);
    TEST_END();

    TEST_BEGIN("payload bit error is reported as CRC error");
    can_frame_build(&f, INVALID_BIT, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(can_frame_decode(bits, info.total_bits, &p) == INVALID_CRC);
    TEST_END();

    TEST_BEGIN("stuff violation is reported");
    fill_std(&f, 0x7ff, 8, d);
    can_frame_build(&f, INVALID_STUFF, 0, bits,
                    CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(max_run(bits, info.stuffed_bits) > 5);
    CHECK(can_frame_decode(bits, info.total_bits, &p) == INVALID_STUFF);
    TEST_END();
}

/****************************************************************
 * Build error handling
 ****************************************************************/
static void test_build_errors(void) {
    static const uint8_t d[8] = {0};
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_frame_t f;

    TEST_BEGIN("invalid inputs are rejected");
    fill_std(&f, 0x123, 9, d);
    CHECK(can_frame_build(&f, VALID_FRAME, 0, bits,
                          CAN2040_LITE_MAX_FRAME_BITS, NULL) == 0);
    fill_std(&f, 0x800, 0, NULL);
    CHECK(can_frame_build(&f, VALID_FRAME, 0, bits,
                          CAN2040_LITE_MAX_FRAME_BITS, NULL) == 0);
    fill_std(&f, 0x123, 8, d);
    CHECK(can_frame_build(&f, VALID_FRAME, 0, bits, 10, NULL) == 0);
    TEST_END();
}


/****************************************************************
 * API skeleton
 ****************************************************************/
static void test_api_skeleton(void) {
    struct can2040_lite cd;
    struct can2040_lite_stats st;
    struct can2040_lite_msg msg;

    TEST_BEGIN("API setup/start/transmit/stop");
    memset(&msg, 0, sizeof(msg));
    can2040_lite_setup(&cd, 0);
    CHECK(can2040_lite_check_transmit(&cd) == 0);

    can2040_lite_start(&cd, 120000000u, 500000u, 0, 1);
    CHECK(can2040_lite_check_transmit(&cd) == 1);

    msg.id = 0x123;
    msg.dlc = 8;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);

    msg.dlc = 9;
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);

    msg.dlc = 0;
    msg.id = 0x800; // out of range for a standard ID
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);

    msg.id = CAN2040_LITE_ID_EFF | 0x18febb11u;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);

    can2040_lite_get_statistics(&cd, &st);
    CHECK(st.tx_attempt == 2);

    can2040_lite_stop(&cd);
    CHECK(can2040_lite_check_transmit(&cd) == 0);
    TEST_END();
}

int main(void) {
    printf("========================================\n");
    printf("CAN2040-Lite PC-based protocol tests\n");
    printf("========================================\n");

    test_bit_helpers();
    test_stuffing();
    test_crc();
    test_frame_layout();
    test_invalid_frames();
    test_build_errors();
    test_api_skeleton();

    printf("\n========================================\n");
    printf("TOTAL: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");
    return g_failed ? 1 : 0;
}
