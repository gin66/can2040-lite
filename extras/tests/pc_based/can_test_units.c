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
#include "can_edge_sm.h"
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

/****************************************************************
 * Edge-timestamp ISR state machine
 *
 * Stuffing stops at the end of the CRC sequence.  A run of recessive
 * CRC bits therefore continues into the recessive delimiter with no
 * edge.  Four such bits plus the delimiter is five; a recessive ACK
 * makes six.  That is legal, and it must not be reported as a stuff
 * error.  Five recessive CRC bits are different: the encoder inserts
 * a dominant stuff bit before the delimiter, so that case does have
 * edges.  The machine has to consume that stuff bit and still arm ACK
 * from the 15th CRC bit, before either edge arrives.
 ****************************************************************/

#define EDGE_TICKS 32u
#define EDGE_T0    0x40000u

static uint32_t edge_at(uint32_t t0, uint32_t ticks, uint32_t bit) {
    return (t0 - bit * ticks) & CAN_EDGE_TIME_MASK;
}

// Walk the stuffed prefix the way the receiver does.  *raw_end is the
// wire index of the first bit after the 15th CRC bit (the trailing stuff
// bit when the sequence ends on a 5-run, otherwise the CRC delimiter).
// Returns 0 on a consistent prefix.
static int prefix_shape(const uint8_t *bits, uint32_t stuffed,
                        uint32_t unstuffed, uint32_t *raw_end,
                        int *trail_ones, int *stuff_after) {
    uint32_t pos = 0;
    uint32_t got = 0;
    uint32_t run = 0;
    uint32_t n;
    uint8_t last = 0;

    while (got < unstuffed && pos < stuffed) {
        uint8_t bit = bits_get(bits, pos++);
        if (run == 5) {
            if (bit == last)
                return -1;
            run = 1;
            last = bit;
            continue;
        }
        got++;
        if (run > 0 && bit == last)
            run++;
        else {
            run = 1;
            last = bit;
        }
    }
    if (got != unstuffed)
        return -1;

    *raw_end = pos;
    if (run == 5) {
        if (pos >= stuffed || bits_get(bits, pos) == last || pos + 1 != stuffed)
            return -1;
        *stuff_after = 1;
    } else {
        if (pos != stuffed)
            return -1;
        *stuff_after = 0;
    }

    n = 0;
    while (n < stuffed && bits_get(bits, stuffed - 1u - n) == 1)
        n++;
    *trail_ones = (int)n;
    return 0;
}

static int drive_span(can_edge_sm_t *sm, const uint8_t *bits, uint32_t from,
                      uint32_t until, uint32_t t0, uint32_t ticks) {
    uint8_t prev = (from == 0) ? 1 : bits_get(bits, from - 1u);
    uint32_t i;
    int edges = 0;
    int saw_frame = sm->in_frame;

    for (i = from; i < until; i++) {
        uint8_t bit = bits_get(bits, i);
        // Stop once the frame under test ends.  A later falling edge would
        // be the next SOF and would wipe this result.
        if (saw_frame && !sm->in_frame)
            break;
        if (bit != prev) {
            can_edge_sm_push(sm, can_edge_word(edge_at(t0, ticks, i), bit));
            edges++;
            saw_frame = 1;
            if (!sm->in_frame)
                break;
        }
        prev = bit;
    }
    if (sm->in_frame)
        can_edge_sm_time(sm, edge_at(t0, ticks, until));
    return edges;
}

static void check_parsed(const can_edge_sm_t *sm, const can_parsed_t *want) {
    CHECK(sm->frame_ready);
    CHECK(sm->validity == want->validity);
    CHECK(sm->parsed.validity == want->validity);
    if (want->validity != VALID_FRAME)
        return;
    CHECK(sm->crc_match);
    CHECK(sm->ack_armed);
    CHECK(sm->ack_seen);
    CHECK(sm->parsed.id == want->id);
    CHECK(sm->parsed.dlc == want->dlc);
    CHECK(sm->parsed.is_extended == want->is_extended);
    CHECK(sm->parsed.is_rtr == want->is_rtr);
    CHECK(sm->parsed.ack == want->ack);
    CHECK(sm->ack == want->ack);
    CHECK(sm->parsed.crc == want->crc);
    CHECK(sm->parsed.crc_calc == want->crc_calc);
    CHECK(sm->parsed.stuffed_bits == want->stuffed_bits);
    CHECK(sm->parsed.total_bits == want->total_bits);
    CHECK(memcmp(sm->parsed.data, want->data, want->dlc) == 0);
}

// Batch driver: edges plus one time update at the end.
static void drive_batch(can_edge_sm_t *sm, const uint8_t *bits, uint32_t nbits,
                        uint32_t t0, uint32_t ticks) {
    can_edge_sm_init(sm, ticks);
    drive_span(sm, bits, 0, nbits, t0, ticks);
}

// One time update per bit, so equal-bit runs never wait for an edge.
static void drive_bitclock(can_edge_sm_t *sm, const uint8_t *bits,
                           uint32_t nbits, uint32_t t0, uint32_t ticks) {
    uint8_t prev = 1;
    uint32_t i;

    can_edge_sm_init(sm, ticks);
    for (i = 0; i < nbits; i++) {
        uint8_t bit = bits_get(bits, i);
        if (bit != prev) {
            can_edge_sm_push(sm, can_edge_word(edge_at(t0, ticks, i), bit));
            if (!sm->in_frame)
                return;
        }
        prev = bit;
        if (sm->in_frame)
            can_edge_sm_time(sm, edge_at(t0, ticks, i + 1u));
        if (!sm->in_frame)
            return;
    }
}

static void check_agrees(const uint8_t *bits, uint32_t nbits,
                         uint32_t t0, uint32_t ticks) {
    can_edge_sm_t batch;
    can_edge_sm_t clock;
    can_parsed_t want;

    CHECK(can_frame_decode(bits, nbits, &want) == want.validity);
    drive_batch(&batch, bits, nbits, t0, ticks);
    check_parsed(&batch, &want);
    drive_bitclock(&clock, bits, nbits, t0, ticks);
    check_parsed(&clock, &want);
    if (want.validity == VALID_FRAME) {
        CHECK(batch.parsed.ack == clock.parsed.ack);
        CHECK(batch.ack_arm_time == clock.ack_arm_time);
    }
}

// Stop on the 15th CRC bit, check that ACK is armed before the slot,
// then finish.  A recessive ACK is consumed only by can_edge_sm_time().
static void check_ack_arm(const can_frame_t *frame, int ack) {
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    can_edge_sm_t sm;
    can_parsed_t want;
    uint32_t nbits;
    uint32_t raw_end = 0;
    uint32_t ack_index;
    int trail = 0;
    int stuff_after = 0;
    int edges;

    nbits = can_frame_build(frame, VALID_FRAME, ack, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits == info.total_bits);
    CHECK(prefix_shape(bits, info.stuffed_bits, info.unstuffed_bits,
                       &raw_end, &trail, &stuff_after) == 0);
    ack_index = info.stuffed_bits + 1u;

    can_edge_sm_init(&sm, EDGE_TICKS);
    edges = drive_span(&sm, bits, 0, raw_end, EDGE_T0, EDGE_TICKS);
    CHECK(edges > 0);
    CHECK(sm.in_frame);
    CHECK(!sm.frame_ready);
    CHECK(!sm.ack_seen);
    CHECK(sm.crc_match);
    CHECK(sm.ack_armed);
    CHECK(sm.ack_arm_time == edge_at(EDGE_T0, EDGE_TICKS, ack_index));

    // Stuff bit (if any) and CRC delimiter.  The ACK slot has not started.
    edges = drive_span(&sm, bits, raw_end, ack_index, EDGE_T0, EDGE_TICKS);
    CHECK(sm.ack_armed);
    CHECK(!sm.ack_seen);
    CHECK(!sm.frame_ready);
    if (!ack && !stuff_after && trail >= 1) {
        // Delimiter continues the recessive CRC tail: no edge at all.
        CHECK(edges == 0);
        CHECK(bits_get(bits, info.stuffed_bits - 1u) == 1);
        CHECK(bits_get(bits, info.stuffed_bits) == 1);
    }

    if (bits_get(bits, ack_index) != bits_get(bits, ack_index - 1u))
        can_edge_sm_push(&sm, can_edge_word(
            edge_at(EDGE_T0, EDGE_TICKS, ack_index),
            bits_get(bits, ack_index)));
    can_edge_sm_time(&sm, edge_at(EDGE_T0, EDGE_TICKS, ack_index + 1u));
    CHECK(sm.ack_seen);
    CHECK(sm.ack == (ack ? 1 : 0));
    CHECK(!sm.frame_ready);

    drive_span(&sm, bits, ack_index + 1u, nbits, EDGE_T0, EDGE_TICKS);
    CHECK(can_frame_decode(bits, nbits, &want) == VALID_FRAME);
    check_parsed(&sm, &want);
}

enum {
    PAT_DOM_EDGE = 0, // CRC ends dominant, no trailing stuff: edge at delimiter
    PAT_REC1,
    PAT_REC2,
    PAT_REC3,
    PAT_REC4,         // four recessive CRC bits + delimiter [+ recessive ACK] = 5 [or 6]
    PAT_REC5,         // five recessive CRC bits, then a dominant stuff bit
    PAT_DOM5,         // five dominant CRC bits, then a recessive stuff bit
    PAT_EXT_REC4,
    PAT_RTR_REC4,
    PAT_COUNT
};

static int pattern_of(int ext, int rtr, int stuff_after, int trail,
                      uint8_t last_bit) {
    if (ext || rtr) {
        if (!ext && rtr && !stuff_after && trail == 4)
            return PAT_RTR_REC4;
        if (ext && !rtr && !stuff_after && trail == 4)
            return PAT_EXT_REC4;
        return -1;
    }
    if (stuff_after && last_bit == 0 && trail == 0)
        return PAT_REC5;
    if (stuff_after && last_bit == 1)
        return PAT_DOM5;
    if (!stuff_after && last_bit == 0 && trail == 0)
        return PAT_DOM_EDGE;
    if (!stuff_after && trail >= 1 && trail <= 4)
        return PAT_REC1 + (trail - 1);
    return -1;
}

static void find_tail_patterns(can_frame_t *found) {
    uint32_t n;
    int got = 0;

    memset(found, 0, sizeof(*found) * PAT_COUNT);
    for (n = 0; n < 300000u && got < PAT_COUNT; n++) {
        can_frame_t frame;
        can_build_info_t info;
        uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
        uint32_t raw_end = 0;
        int trail = 0;
        int stuff_after = 0;
        int which;
        int ext = (n & 1u) != 0;
        int rtr = (n % 5u) == 0;
        uint32_t i;

        memset(&frame, 0, sizeof(frame));
        frame.dlc = (uint8_t)((n / 2u) % 9u);
        if (ext) {
            frame.flags = CAN_FRAME_EXTENDED;
            frame.id = (n * 0x45d9fu) & 0x1fffffffu;
        } else {
            frame.id = (n * 17u) & 0x7ffu;
        }
        if (rtr)
            frame.flags = (uint8_t)(frame.flags | CAN_FRAME_RTR);
        for (i = 0; i < 8; i++)
            frame.data[i] = (uint8_t)(n * (i + 3u) + (n >> (i + 1u)));

        if (can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info) == 0)
            continue;
        if (prefix_shape(bits, info.stuffed_bits, info.unstuffed_bits,
                         &raw_end, &trail, &stuff_after) != 0)
            continue;
        which = pattern_of(ext, rtr,
                           stuff_after, trail,
                           bits_get(bits, info.stuffed_bits - 1u));
        if (which < 0 || (found[which].flags & 0x80u) != 0)
            continue;
        // id 0 / dlc 0 is a legal frame, so a flag marks the slot filled.
        frame.flags = (uint8_t)(frame.flags | 0x80u);
        found[which] = frame;
        got++;
    }
}

static void test_edge_sm(void) {
    static const uint8_t payload[8] = {
        0xde, 0xad, 0xbe, 0xef, 0x12, 0x34, 0x56, 0x78
    };
    static const char *pat_name[PAT_COUNT] = {
        "dominant CRC end, edge at delimiter",
        "1 trailing recessive CRC bit",
        "2 trailing recessive CRC bits",
        "3 trailing recessive CRC bits",
        "4 trailing recessive CRC bits",
        "5 trailing recessive CRC bits (stuff bit)",
        "5 trailing dominant CRC bits (stuff bit)",
        "extended, 4 trailing recessive CRC bits",
        "RTR, 4 trailing recessive CRC bits"
    };
    can_frame_t found[PAT_COUNT];
    can_frame_t frame;
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    can_edge_sm_t sm;
    char name[96];
    uint32_t nbits;
    uint32_t i;
    int ack;
    int missing = 0;

    printf("\n-- Edge ISR state machine --\n");

    find_tail_patterns(found);
    for (i = 0; i < PAT_COUNT; i++) {
        if ((found[i].flags & 0x80u) == 0) {
            printf("    missing pattern: %s\n", pat_name[i]);
            missing = 1;
        }
        found[i].flags = (uint8_t)(found[i].flags & (uint8_t)~0x80u);
    }

    TEST_BEGIN("tail patterns exist");
    CHECK(!missing);
    TEST_END();

    for (i = 0; i <= 8; i++) {
        for (ack = 0; ack < 2; ack++) {
            snprintf(name, sizeof(name), "std dlc %u %s agrees with decode",
                     (unsigned)i, ack ? "ACK" : "NACK");
            TEST_BEGIN(name);
            fill_std(&frame, 0x123, (uint8_t)i, payload);
            nbits = can_frame_build(&frame, VALID_FRAME, ack, bits,
                                    CAN2040_LITE_MAX_FRAME_BITS, &info);
            CHECK(nbits != 0);
            check_agrees(bits, nbits, EDGE_T0, EDGE_TICKS);
            TEST_END();
        }
    }

    TEST_BEGIN("extended frame agrees with decode");
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x18febb11u;
    frame.dlc = 4;
    frame.flags = CAN_FRAME_EXTENDED;
    memcpy(frame.data, payload, 4);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    check_agrees(bits, nbits, EDGE_T0, EDGE_TICKS);
    TEST_END();

    TEST_BEGIN("RTR frame agrees with decode");
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x456;
    frame.dlc = 2;
    frame.flags = CAN_FRAME_RTR;
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    check_agrees(bits, nbits, EDGE_T0, EDGE_TICKS);
    TEST_END();

    for (i = 0; i < PAT_COUNT; i++) {
        for (ack = 0; ack < 2; ack++) {
            snprintf(name, sizeof(name), "%s, %s",
                     pat_name[i], ack ? "dominant ACK" : "recessive ACK");
            TEST_BEGIN(name);
            if (!missing)
                check_ack_arm(&found[i], ack);
            else
                CHECK(0);
            TEST_END();
        }
    }

    TEST_BEGIN("four recessive CRC bits plus delimiter and NACK are six ones");
    if (!missing) {
        uint32_t raw_end = 0;
        int trail = 0;
        int stuff_after = 0;
        nbits = can_frame_build(&found[PAT_REC4], VALID_FRAME, 0, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(prefix_shape(bits, info.stuffed_bits, info.unstuffed_bits,
                           &raw_end, &trail, &stuff_after) == 0);
        CHECK(!stuff_after);
        CHECK(trail == 4);
        CHECK(bits_get(bits, info.stuffed_bits) == 1);
        CHECK(bits_get(bits, info.stuffed_bits + 1u) == 1);
        // From the last dominant before this run through the ACK slot.
        {
            uint32_t run = 0;
            uint32_t p = info.stuffed_bits;
            while (p > 0 && bits_get(bits, p - 1u) == 1)
                p--;
            run = (info.stuffed_bits + 2u) - p; // CRC ones + delim + ACK
            CHECK(run == 6);
        }
        check_agrees(bits, nbits, EDGE_T0, EDGE_TICKS);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("five recessive CRC bits are followed by a dominant stuff bit");
    if (!missing) {
        uint32_t raw_end = 0;
        int trail = 0;
        int stuff_after = 0;
        uint32_t s;
        nbits = can_frame_build(&found[PAT_REC5], VALID_FRAME, 1, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(prefix_shape(bits, info.stuffed_bits, info.unstuffed_bits,
                           &raw_end, &trail, &stuff_after) == 0);
        CHECK(stuff_after);
        s = info.stuffed_bits;
        CHECK(bits_get(bits, s - 1u) == 0);
        CHECK(bits_get(bits, s - 2u) == 1);
        CHECK(bits_get(bits, s - 3u) == 1);
        CHECK(bits_get(bits, s - 4u) == 1);
        CHECK(bits_get(bits, s - 5u) == 1);
        CHECK(bits_get(bits, s - 6u) == 1);
        CHECK(bits_get(bits, s) == 1); // delimiter, opposite the stuff bit
        check_agrees(bits, nbits, EDGE_T0, EDGE_TICKS);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("CRC mismatch does not arm ACK");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, INVALID_CRC, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_CRC);
    CHECK(!sm.crc_match);
    CHECK(!sm.ack_armed);
    CHECK(!sm.ack_seen);
    TEST_END();

    TEST_BEGIN("stuff violation inside the stuffed region does not arm ACK");
    fill_std(&frame, 0x7ff, 8, payload);
    nbits = can_frame_build(&frame, INVALID_STUFF, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    CHECK(max_run(bits, info.stuffed_bits) > 5);
    drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_STUFF);
    CHECK(!sm.ack_armed);
    CHECK(!sm.crc_match);
    TEST_END();

    TEST_BEGIN("dominant CRC delimiter is a form error");
    fill_std(&frame, 0x123, 3, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    bits_put(bits, info.stuffed_bits, 0);
    {
        can_parsed_t want;
        CHECK(can_frame_decode(bits, nbits, &want) == INVALID_BIT);
        drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
        CHECK(sm.frame_ready);
        CHECK(sm.validity == INVALID_BIT);
    }
    TEST_END();

    TEST_BEGIN("dominant EOF is a form error after a CRC match");
    fill_std(&frame, 0x123, 3, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    bits_put(bits, info.stuffed_bits + 3u, 0);
    drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_BIT);
    CHECK(sm.crc_match);
    CHECK(sm.ack_armed);
    CHECK(sm.ack_seen);
    CHECK(sm.ack == 1);
    TEST_END();

    TEST_BEGIN("a fraction of a bit does not complete that bit");
    fill_std(&frame, 0x001, 0, NULL);
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    can_edge_sm_init(&sm, EDGE_TICKS);
    can_edge_sm_push(&sm, can_edge_word(EDGE_T0, 0));
    can_edge_sm_time(&sm, edge_at(EDGE_T0, EDGE_TICKS, 0) - (EDGE_TICKS / 2u));
    CHECK(sm.in_frame);
    CHECK(sm.raw_bits == 0);
    CHECK(!sm.frame_ready);
    can_edge_sm_time(&sm, edge_at(EDGE_T0, EDGE_TICKS, 1));
    CHECK(sm.raw_bits == 1);
    TEST_END();

    TEST_BEGIN("one late tick is carried and the frame still decodes");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    {
        uint32_t late = 0;
        uint8_t prev = 1;
        can_parsed_t want;
        can_edge_sm_init(&sm, EDGE_TICKS);
        for (i = 0; i < nbits; i++) {
            uint8_t bit = bits_get(bits, i);
            if (bit != prev) {
                uint32_t t = edge_at(EDGE_T0, EDGE_TICKS, i);
                if (late == 0 && i > 0) {
                    t = (t - 1u) & CAN_EDGE_TIME_MASK; // one tick later
                    late = i;
                }
                can_edge_sm_push(&sm, can_edge_word(t, bit));
            }
            prev = bit;
        }
        CHECK(late != 0);
        can_edge_sm_time(&sm, edge_at(EDGE_T0, EDGE_TICKS, nbits));
        CHECK(can_frame_decode(bits, nbits, &want) == VALID_FRAME);
        check_parsed(&sm, &want);
    }
    TEST_END();

    TEST_BEGIN("31-bit counter wrap during the frame");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    // 80 is only a few ticks; the frame crosses the 31-bit modulus.
    check_agrees(bits, nbits, 80u, EDGE_TICKS);
    TEST_END();

    TEST_BEGIN("back-to-back frames keep a separate result per SOF");
    {
        uint8_t both[CAN2040_LITE_MAX_FRAME_BYTES];
        uint8_t second[CAN2040_LITE_MAX_FRAME_BYTES];
        can_frame_t f2;
        can_build_info_t i2;
        can_parsed_t want;
        uint32_t n2;
        uint8_t saved_data[8];
        uint32_t saved_id;
        uint8_t saved_ack;

        fill_std(&frame, 0x111, 2, payload);
        fill_std(&f2, 0x222, 1, payload);
        bits_clear(both, CAN2040_LITE_MAX_FRAME_BITS);
        nbits = can_frame_build(&frame, VALID_FRAME, 0, both,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        n2 = can_frame_build(&f2, VALID_FRAME, 1, second,
                             CAN2040_LITE_MAX_FRAME_BITS, &i2);
        CHECK(nbits != 0 && n2 != 0);
        CHECK(info.total_bits + n2 <= CAN2040_LITE_MAX_FRAME_BITS);
        for (i = 0; i < n2; i++)
            bits_put(both, info.total_bits + i, bits_get(second, i));

        can_edge_sm_init(&sm, EDGE_TICKS);
        drive_span(&sm, both, 0, info.total_bits, EDGE_T0, EDGE_TICKS);
        CHECK(can_frame_decode(both, info.total_bits, &want) == VALID_FRAME);
        check_parsed(&sm, &want);
        saved_id = sm.parsed.id;
        saved_ack = sm.ack;
        memcpy(saved_data, sm.parsed.data, 2);

        drive_span(&sm, both, info.total_bits, info.total_bits + n2,
                   EDGE_T0, EDGE_TICKS);
        CHECK(can_frame_decode(second, n2, &want) == VALID_FRAME);
        check_parsed(&sm, &want);
        CHECK(sm.parsed.id == 0x222);
        CHECK(sm.ack == 1);
        CHECK(saved_id == 0x111);
        CHECK(saved_ack == 0);
        CHECK(saved_data[0] == payload[0]);
        CHECK(saved_data[1] == payload[1]);
    }
    TEST_END();

    TEST_BEGIN("crc match with a recessive slot is not an ACK");
    if (!missing) {
        nbits = can_frame_build(&found[PAT_REC4], VALID_FRAME, 0, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
        CHECK(sm.validity == VALID_FRAME);
        CHECK(sm.crc_match);
        CHECK(sm.ack_armed);
        CHECK(sm.ack_seen);
        CHECK(sm.ack == 0);

        nbits = can_frame_build(&found[PAT_REC4], VALID_FRAME, 1, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        drive_batch(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
        CHECK(sm.validity == VALID_FRAME);
        CHECK(sm.crc_match);
        CHECK(sm.ack == 1);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("idle recessive edges do not start a frame");
    can_edge_sm_init(&sm, EDGE_TICKS);
    can_edge_sm_time(&sm, EDGE_T0);
    can_edge_sm_push(&sm, can_edge_word(EDGE_T0 - 10u, 1));
    CHECK(!sm.in_frame);
    CHECK(!sm.frame_ready);
    CHECK(sm.raw_bits == 0);
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
    test_edge_sm();

    printf("\n========================================\n");
    printf("TOTAL: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");
    return g_failed ? 1 : 0;
}
