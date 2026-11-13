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
#include <stdbool.h>

#include "can2040_lite_proto.h"
#include "can2040_lite.h"
#include "can_edge_sm.h"
#include "can_frame_build.h"
#include "ack_cycle_model.h"
#include "can_rx_record_fly_micro4.h"

// The PIO images are built at runtime by the portable program builder.
#include "pico/can_pio_program.h"
#include "pico/can_pio_reloc.h"

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
    can2040_lite_set_hw(NULL);
    memset(&msg, 0, sizeof(msg));
    can2040_lite_setup(&cd, 0);
    CHECK(can2040_lite_check_transmit(&cd) == 0);

    can2040_lite_start(&cd, 120000000u, 500000u, 0, 1);
    CHECK(can2040_lite_check_transmit(&cd) == 1);

    msg.id = 0x123;
    msg.dlc = 8;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);

    msg.dlc = 9; // accepted: 8 data bytes, wire DLC nibble 9
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);

    msg.dlc = 16; // rejected
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);

    msg.dlc = 0;
    msg.id = 0x800; // out of range for a standard ID
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);

    msg.id = CAN2040_LITE_ID_EFF | 0x18febb11u;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);

    can2040_lite_get_statistics(&cd, &st);
    CHECK(st.tx_attempt == 3);

    can2040_lite_stop(&cd);
    CHECK(can2040_lite_check_transmit(&cd) == 0);
    TEST_END();
}

/****************************************************************
 * API transmit queue, packer, and reporting (fake ops table)
 ****************************************************************/

#define FAKE_MAX 16

static struct can2040_lite_event fake_ev[FAKE_MAX];
static int fake_ev_head;
static int fake_ev_tail;
static uint8_t fake_tx[FAKE_MAX][CAN2040_LITE_MAX_FRAME_BYTES];
static uint32_t fake_tx_nbits[FAKE_MAX];
static int fake_tx_count;
static int fake_start_count;
static int fake_stop_count;

static int fake_hw_start(struct can2040_lite *cd, uint32_t sys_clock,
                         uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx) {
    (void)cd; (void)sys_clock; (void)bitrate; (void)gpio_rx; (void)gpio_tx;
    fake_start_count++;
    return 0;
}

static void fake_hw_stop(struct can2040_lite *cd) {
    (void)cd;
    fake_stop_count++;
}

static int fake_hw_tx_frame(const uint8_t *bits, uint32_t nbits) {
    uint32_t i;
    if (fake_tx_count < FAKE_MAX) {
        bits_clear(fake_tx[fake_tx_count], CAN2040_LITE_MAX_FRAME_BITS);
        for (i = 0; i < nbits; i++)
            bits_put(fake_tx[fake_tx_count], i, bits_get(bits, i));
        fake_tx_nbits[fake_tx_count] = nbits;
    }
    fake_tx_count++;
    return 0;
}

static int fake_hw_poll(struct can2040_lite_event *ev) {
    if (fake_ev_head == fake_ev_tail)
        return 0;
    *ev = fake_ev[fake_ev_head];
    fake_ev_head = (fake_ev_head + 1) % FAKE_MAX;
    return 1;
}

static const struct can2040_lite_hw fake_hw = {
    fake_hw_start, fake_hw_stop, fake_hw_tx_frame, fake_hw_poll
};

static void fake_reset(void) {
    fake_ev_head = 0;
    fake_ev_tail = 0;
    fake_tx_count = 0;
    fake_start_count = 0;
    fake_stop_count = 0;
    can2040_lite_set_hw(&fake_hw);
}

static void fake_push_event(const struct can2040_lite_event *ev) {
    fake_ev[fake_ev_tail] = *ev;
    fake_ev_tail = (fake_ev_tail + 1) % FAKE_MAX;
}

static void fake_push_tx_done(int err) {
    struct can2040_lite_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.kind = 1;
    ev.tx_error = err;
    fake_push_event(&ev);
}

static void frame_from_msg(const struct can2040_lite_msg *m, can_frame_t *f) {
    memset(f, 0, sizeof(*f));
    f->id = m->id & 0x1fffffffu;
    f->dlc = (uint8_t)(m->dlc > 8u ? 8u : m->dlc);
    if (m->id & CAN2040_LITE_ID_EFF)
        f->flags |= CAN_FRAME_EXTENDED;
    if (m->id & CAN2040_LITE_ID_RTR)
        f->flags |= CAN_FRAME_RTR;
    memcpy(f->data, m->data, sizeof(f->data));
}

// Queue a parsed RX event built from a frame.  For INVALID_BIT the parser
// output is overridden because can_frame_build's bit flip presents as a CRC
// mismatch to can_frame_decode.
static void fake_push_rx_frame(const can_frame_t *f, frame_validity_t validity,
                               int ack) {
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    struct can2040_lite_event ev;
    uint32_t n;

    memset(&ev, 0, sizeof(ev));
    ev.kind = 2;
    n = can_frame_build(f, validity, ack, bits, CAN2040_LITE_MAX_FRAME_BITS,
                        &info);
    CHECK(n != 0);
    CHECK(can_frame_decode(bits, n, &ev.rx) != INVALID_STUFF ||
          validity == INVALID_STUFF);
    if (validity == INVALID_BIT)
        ev.rx.validity = INVALID_BIT;
    else if (validity == INVALID_STUFF)
        ev.rx.validity = INVALID_STUFF;
    fake_push_event(&ev);
}

static int cb_count;
static uint32_t cb_notify[FAKE_MAX];
static struct can2040_lite_msg cb_msg[FAKE_MAX];
static int cb_reenter;
static int cb_reenter_result;
static uint32_t cb_reenter_msg_id_after;

static void test_cb(struct can2040_lite *cd, uint32_t notify,
                    struct can2040_lite_msg *msg) {
    if (cb_count < FAKE_MAX) {
        cb_notify[cb_count] = notify;
        cb_msg[cb_count] = *msg;
    }
    cb_count++;

    if (cb_reenter && (notify & CAN2040_LITE_NOTIFY_TX)) {
        struct can2040_lite_msg m;
        cb_reenter = 0;
        memset(&m, 0, sizeof(m));
        m.id = 0x777;
        m.dlc = 0;
        cb_reenter_result = can2040_lite_transmit(cd, &m);
        cb_reenter_msg_id_after = msg->id;
    }
}

static void cb_reset(void) {
    cb_count = 0;
    cb_reenter = 0;
    cb_reenter_result = -1;
    cb_reenter_msg_id_after = 0;
    memset(cb_notify, 0, sizeof(cb_notify));
    memset(cb_msg, 0, sizeof(cb_msg));
}

static int packed_matches_oracle(const uint8_t *packed, uint32_t nbits,
                                 const can_frame_t *f, uint32_t *oracle_nbits) {
    uint8_t oracle[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    uint32_t n;
    uint32_t i;

    n = can_frame_build(f, VALID_FRAME, 0, oracle, CAN2040_LITE_MAX_FRAME_BITS,
                        &info);
    CHECK(n != 0);
    *oracle_nbits = info.stuffed_bits + 1u;
    if (nbits != info.stuffed_bits + 1u)
        return 0;
    for (i = 0; i < nbits; i++)
        if (bits_get(packed, i) != bits_get(oracle, i))
            return 0;
    return 1;
}

static void start_for_test(struct can2040_lite *cd, int32_t gpio_tx) {
    fake_reset();
    cb_reset();
    can2040_lite_setup(cd, 0);
    can2040_lite_callback_config(cd, test_cb);
    can2040_lite_start(cd, 120000000u, 500000u, 0, gpio_tx);
}

static void test_api_queue(void) {
    struct can2040_lite cd;
    struct can2040_lite_stats st;
    struct can2040_lite_msg msg;
    struct can2040_lite_msg msgs[5];
    can_frame_t frame;
    uint8_t un[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t ulen;
    uint32_t oracle_nbits;
    uint32_t i;
    int j;

    printf("\n-- API queue and reports --\n");

    TEST_BEGIN("five transmits queue, fifth rejected, order preserved");
    start_for_test(&cd, 1);
    memset(msgs, 0, sizeof(msgs));
    for (i = 0; i < 5; i++) {
        msgs[i].id = 0x100u + i;
        msgs[i].dlc = 1;
        msgs[i].data[0] = (uint8_t)i;
    }
    for (i = 0; i < 4; i++)
        CHECK(can2040_lite_transmit(&cd, &msgs[i]) == 0);
    CHECK(can2040_lite_transmit(&cd, &msgs[4]) == -1);
    CHECK(fake_tx_count == 1); // only the head was handed to the SM
    CHECK(can2040_lite_check_transmit(&cd) == 0);

    // Complete the first through a matching RX parse with ACK.
    frame_from_msg(&msgs[0], &frame);
    fake_push_rx_frame(&frame, VALID_FRAME, 1);
    can2040_lite_pio_irq_handler(&cd);
    CHECK(fake_tx_count == 2); // next message started
    CHECK(can2040_lite_check_transmit(&cd) == 1);
    CHECK(can2040_lite_transmit(&cd, &msgs[4]) == 0);

    // The fake saw 0x100 then 0x101, in call order.
    frame_from_msg(&msgs[0], &frame);
    CHECK(packed_matches_oracle(fake_tx[0], fake_tx_nbits[0], &frame,
                                &oracle_nbits));
    frame_from_msg(&msgs[1], &frame);
    CHECK(packed_matches_oracle(fake_tx[1], fake_tx_nbits[1], &frame,
                                &oracle_nbits));
    TEST_END();

    TEST_BEGIN("stop keeps the queue; setup clears it and the statistics");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x201;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    msg.id = 0x202;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    can2040_lite_stop(&cd);
    CHECK(can2040_lite_check_transmit(&cd) == 0);
    CHECK(fake_stop_count == 1);
    can2040_lite_start(&cd, 120000000u, 500000u, 0, 1);
    CHECK(can2040_lite_check_transmit(&cd) == 1); // both still queued
    CHECK(fake_start_count == 2);
    CHECK(fake_tx_count == 2); // the head is handed to the SM again

    fake_reset();
    can2040_lite_setup(&cd, 0);
    can2040_lite_start(&cd, 120000000u, 500000u, 0, 1);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(st.tx_attempt == 0);
    CHECK(st.tx_total == 0);
    CHECK(st.rx_total == 0);
    CHECK(fake_tx_count == 0); // empty queue starts nothing
    TEST_END();

    TEST_BEGIN("gpio_tx == -1 is receive-only");
    start_for_test(&cd, -1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x123;
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);
    CHECK(can2040_lite_check_transmit(&cd) == 0);
    TEST_END();

    TEST_BEGIN("DLC 15 queues with nibble 15 and eight data bytes; 16 rejected");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x321;
    msg.dlc = 15;
    for (i = 0; i < 8; i++)
        msg.data[i] = (uint8_t)(0xa0u + i);
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    CHECK(fake_tx_count == 1);
    ulen = bit_unstuff(fake_tx[0], fake_tx_nbits[0] - 1u, un,
                       CAN2040_LITE_MAX_FRAME_BITS);
    CHECK(ulen == 19u + 64u + 15u);
    CHECK(get_field(un, 15, 4) == 15);
    for (i = 0; i < 64; i++) {
        uint8_t want = (uint8_t)((msg.data[i / 8u] >> (7u - (i % 8u))) & 1u);
        CHECK(bits_get(un, 19u + i) == want);
    }
    msg.dlc = 16;
    CHECK(can2040_lite_transmit(&cd, &msg) == -1);
    TEST_END();

    TEST_BEGIN("packed bits match the frame builder (std/ext/RTR)");
    {
        struct can2040_lite_msg packed_msgs[3];
        memset(packed_msgs, 0, sizeof(packed_msgs));
        packed_msgs[0].id = 0x123;
        packed_msgs[0].dlc = 8;
        for (i = 0; i < 8; i++)
            packed_msgs[0].data[i] = (uint8_t)(i * 0x11u);
        packed_msgs[1].id = CAN2040_LITE_ID_EFF | 0x18febb11u;
        packed_msgs[1].dlc = 5;
        for (i = 0; i < 5; i++)
            packed_msgs[1].data[i] = (uint8_t)(0xf0u + i);
        packed_msgs[2].id = CAN2040_LITE_ID_RTR | 0x456;
        packed_msgs[2].dlc = 8;
        for (j = 0; j < 3; j++) {
            start_for_test(&cd, 1);
            CHECK(can2040_lite_transmit(&cd, &packed_msgs[j]) == 0);
            CHECK(fake_tx_count == 1);
            frame_from_msg(&packed_msgs[j], &frame);
            CHECK(packed_matches_oracle(fake_tx[0], fake_tx_nbits[0], &frame,
                                        &oracle_nbits));
        }
    }
    TEST_END();

    TEST_BEGIN("EFF is bit 31 and RTR is bit 30 in reports");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = CAN2040_LITE_ID_EFF | 0x18febb11u;
    msg.dlc = 1;
    msg.data[0] = 0x5a;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    frame_from_msg(&msg, &frame);
    fake_push_rx_frame(&frame, VALID_FRAME, 1);
    CHECK(can2040_lite_transmit(&cd, &msg) == 0); // second queues
    can2040_lite_pio_irq_handler(&cd);
    CHECK(cb_count == 1);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_TX);
    CHECK(cb_msg[0].id & CAN2040_LITE_ID_EFF);
    CHECK((cb_msg[0].id & CAN2040_LITE_ID_EFF) == (1u << 31));

    cb_reset();
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x456;
    frame.dlc = 0;
    frame.flags = CAN_FRAME_RTR;
    fake_push_rx_frame(&frame, VALID_FRAME, 0);
    can2040_lite_pio_irq_handler(&cd);
    CHECK(cb_count == 1);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_RX);
    CHECK((cb_msg[0].id & CAN2040_LITE_ID_RTR) == (1u << 30));
    TEST_END();

    TEST_BEGIN("valid RX calls back once and counts rx_total");
    start_for_test(&cd, 1);
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x123;
    frame.dlc = 2;
    frame.data[0] = 0xde;
    frame.data[1] = 0xad;
    fake_push_rx_frame(&frame, VALID_FRAME, 0);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 1);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_RX);
    CHECK(cb_msg[0].id == 0x123);
    CHECK(cb_msg[0].dlc == 2);
    CHECK(cb_msg[0].data[0] == 0xde && cb_msg[0].data[1] == 0xad);
    CHECK(st.rx_total == 1);
    TEST_END();

    TEST_BEGIN("CRC, stuff and form errors do not call back");
    start_for_test(&cd, 1);
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x123;
    frame.dlc = 8;
    for (i = 0; i < 8; i++)
        frame.data[i] = (uint8_t)i;
    fake_push_rx_frame(&frame, INVALID_CRC, 0);
    fake_push_rx_frame(&frame, INVALID_STUFF, 0);
    fake_push_rx_frame(&frame, INVALID_BIT, 0);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 0);
    CHECK(st.parse_error == 3);
    CHECK(st.crc_errors == 1);
    CHECK(st.stuff_errors == 1);
    CHECK(st.bit_errors == 1);
    CHECK(st.rx_total == 0);
    TEST_END();

    TEST_BEGIN("matching ACK completes TX; NACK does not call back");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x123;
    msg.dlc = 8;
    for (i = 0; i < 8; i++)
        msg.data[i] = (uint8_t)(0x30u + i);
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    frame_from_msg(&msg, &frame);
    fake_push_rx_frame(&frame, VALID_FRAME, 1);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 1);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_TX);
    CHECK(cb_msg[0].id == 0x123);
    CHECK(st.tx_total == 1);
    CHECK(st.tx_attempt == 1);

    start_for_test(&cd, 1);
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    fake_push_rx_frame(&frame, VALID_FRAME, 0);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 0);
    CHECK(st.tx_nack == 1);
    CHECK(st.parse_error == 1);
    CHECK(st.tx_attempt == 1 && st.tx_total == 0);
    TEST_END();

    TEST_BEGIN("tx_error completes the head without a callback");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x222;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    fake_push_tx_done(1);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 0);
    CHECK(st.tx_error == 1);
    CHECK(st.parse_error == 1);
    CHECK(st.tx_total == 0);
    TEST_END();

    TEST_BEGIN("two RX results before the callback report overflow");
    start_for_test(&cd, 1);
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x111;
    frame.dlc = 1;
    fake_push_rx_frame(&frame, VALID_FRAME, 0);
    frame.id = 0x222;
    fake_push_rx_frame(&frame, VALID_FRAME, 0);
    can2040_lite_pio_irq_handler(&cd);
    can2040_lite_get_statistics(&cd, &st);
    CHECK(cb_count == 2);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_ERROR);
    CHECK(cb_msg[0].id == 0);
    CHECK(cb_notify[1] == CAN2040_LITE_NOTIFY_RX);
    CHECK(cb_msg[1].id == 0x222);
    CHECK(st.rx_overflow == 1);
    CHECK(st.rx_total == 2);
    TEST_END();

    TEST_BEGIN("callback may transmit; its message pointer stays valid");
    start_for_test(&cd, 1);
    memset(&msg, 0, sizeof(msg));
    msg.id = 0x123;
    msg.dlc = 3;
    CHECK(can2040_lite_transmit(&cd, &msg) == 0);
    frame_from_msg(&msg, &frame);
    fake_push_rx_frame(&frame, VALID_FRAME, 1);
    cb_reenter = 1;
    can2040_lite_pio_irq_handler(&cd);
    CHECK(cb_reenter_result == 0);
    CHECK(cb_reenter_msg_id_after == 0x123);
    CHECK(cb_notify[0] == CAN2040_LITE_NOTIFY_TX);
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

// Measured Fly-Micro4 / ESP32 edge timing at 1 Mbit/s: the delta of a run is
// n*T plus a fixed offset that depends only on polarity, not on n.  Dominant
// runs are slightly long, recessive runs slightly short; the edge path adds
// the same fixed cost to every edge.  The decoder must classify each interval
// from its own delta, so these offsets never accumulate across a frame.
#define ASYM_DOM_EXTRA 1
#define ASYM_REC_LOSS  2
static int s_asym_bad_bit = -1;

static uint32_t asym_edge_time(uint32_t last_time, uint32_t last_bit,
                               uint32_t bit, uint32_t ticks, uint8_t level) {
    int32_t delta = (int32_t)((bit - last_bit) * ticks);
    if (level)
        delta += ASYM_DOM_EXTRA;
    else
        delta -= ASYM_REC_LOSS;
    if ((int)bit == s_asym_bad_bit)
        delta -= 8;  // push this interval outside the edge-timing budget
    return (last_time - (uint32_t)delta) & CAN_EDGE_TIME_MASK;
}

// Same as drive_span(), but every edge-to-edge delta carries the measured
// polarity offset.  Only whole frames (from == 0) are used.
static int drive_span_asym(can_edge_sm_t *sm, const uint8_t *bits,
                           uint32_t until, uint32_t t0, uint32_t ticks) {
    uint8_t prev = 1;
    uint32_t i;
    uint32_t last_bit = 0;
    uint32_t last_time = edge_at(t0, ticks, 0);
    int have_edge = 0;
    int edges = 0;
    int saw_frame = sm->in_frame;

    for (i = 0; i < until; i++) {
        uint8_t bit = bits_get(bits, i);
        if (saw_frame && !sm->in_frame)
            break;
        if (bit != prev) {
            uint32_t t = have_edge
                             ? asym_edge_time(last_time, last_bit, i, ticks, bit)
                             : edge_at(t0, ticks, i);
            can_edge_sm_push(sm, can_edge_word(t, bit));
            last_time = t;
            last_bit = i;
            have_edge = 1;
            edges++;
            saw_frame = 1;
            if (!sm->in_frame)
                break;
        }
        prev = bit;
    }
    if (sm->in_frame) {
        uint32_t now = have_edge
                           ? (last_time - (until - last_bit) * ticks) &
                                 CAN_EDGE_TIME_MASK
                           : edge_at(t0, ticks, until);
        can_edge_sm_time(sm, now);
    }
    return edges;
}

static void drive_batch_asym(can_edge_sm_t *sm, const uint8_t *bits,
                             uint32_t nbits, uint32_t t0, uint32_t ticks) {
    can_edge_sm_init(sm, ticks);
    s_asym_bad_bit = -1;
    drive_span_asym(sm, bits, nbits, t0, ticks);
}

static void check_agrees_asym(const uint8_t *bits, uint32_t nbits,
                              uint32_t t0, uint32_t ticks) {
    can_edge_sm_t batch;
    can_parsed_t want;

    CHECK(can_frame_decode(bits, nbits, &want) == want.validity);
    drive_batch_asym(&batch, bits, nbits, t0, ticks);
    check_parsed(&batch, &want);
    // The quiet-tail interleave is covered by the recorded hardware capture
    // test, which calls can_edge_sm_time() between FIFO words.
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

// Feed edges as can_rx_pair words: (t[1->0] << 16) | t[0->1], each the low
// 16 bits of the PIO's two-cycle counter.  Both edge polarities are sampled
// at the same phase of the count grid (see can_pio_rx_pair_program()), so
// every timestamp carries the same constant offset and it cancels in each
// fall/rise delta; the decoder uses bias 0.  Appends to an existing decoder,
// so a second frame can follow the first across an idle gap.
#define PAIR_SAMPLE_OFFSET 1u

static void feed_pairs(can_edge_pair_t *pair, can_edge_sm_t *sm,
                       const uint8_t *bits, uint32_t nbits,
                       uint32_t t0, uint32_t ticks) {
    uint8_t prev = 1;
    uint32_t hi = 0;
    uint32_t i;
    int have_hi = 0;
    int started = sm->in_frame;

    for (i = 0; i < nbits; i++) {
        uint8_t bit = bits_get(bits, i);
        if (started && !sm->in_frame)
            break;
        if (bit != prev) {
            uint32_t t =
                (uint16_t)(edge_at(t0, ticks, i) + PAIR_SAMPLE_OFFSET);
            if (!have_hi) {
                hi = t;
                have_hi = 1;
            } else {
                can_edge_pair_push(pair, (hi << 16) | t);
                have_hi = 0;
                started = 1;
                if (!sm->in_frame)
                    break;
            }
        }
        prev = bit;
    }
}

// One frame through the pair decoder, seeded so the SOF timestamp is t0.
static void drive_pairs(can_edge_sm_t *sm, const uint8_t *bits, uint32_t nbits,
                        uint32_t t0, uint32_t ticks) {
    can_edge_pair_t pair;

    can_edge_sm_init(sm, ticks);
    can_edge_pair_init(&pair, sm, t0, 0);
    feed_pairs(&pair, sm, bits, nbits, t0, ticks);
    if (sm->in_frame)
        can_edge_sm_time(sm, (pair.time - 16u * ticks) & CAN_EDGE_TIME_MASK);
}

// Drive a valid frame but replace the ACK slot with a dominant pulse at
// custom offsets from the slot start.  A negative start offset places the
// falling edge in the CRC delimiter.  `end_off` is measured from the slot
// start as well, so a 1-bit pulse is start+ticks .. start+ticks+ticks.
static void drive_ack_pulse(can_edge_sm_t *sm, const uint8_t *bits,
                            uint32_t nbits, uint32_t t0, uint32_t ticks,
                            uint32_t ack_index, int32_t start_off,
                            uint32_t end_off) {
    uint8_t prev = 1;
    uint32_t i;
    uint32_t slot_time = edge_at(t0, ticks, ack_index);
    uint32_t fall;

    can_edge_sm_init(sm, ticks);
    for (i = 0; i < ack_index; i++) {
        uint8_t bit = bits_get(bits, i);
        if (bit != prev)
            can_edge_sm_push(sm, can_edge_word(edge_at(t0, ticks, i), bit));
        prev = bit;
    }
    if (start_off >= 0)
        fall = (slot_time - (uint32_t)start_off) & CAN_EDGE_TIME_MASK;
    else
        fall = (slot_time + (uint32_t)(-start_off)) & CAN_EDGE_TIME_MASK;
    can_edge_sm_push(sm, can_edge_word(fall, 0));
    can_edge_sm_push(sm, can_edge_word(
        (slot_time - end_off) & CAN_EDGE_TIME_MASK, 1));
    can_edge_sm_time(sm, edge_at(t0, ticks, nbits));
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

    // Fly-Micro4 measured timing: each edge-to-edge delta is n*T plus a fixed
    // polarity offset.  This is not a bit-period error, so a decoder that
    // carries one phase accumulator drifts; one that classifies each edge
    // interval and re-anchors does not.
    // Asymmetric edge timing is the RX/CRC milestone.  A dominant ACK slot
    // changes the tail edge pattern; that belongs to the ACK step and is not
    // part of this recording (the receiver was RX-only and sent no ACK).
    for (i = 0; i <= 8; i++) {
        snprintf(name, sizeof(name),
                 "asymmetric edges, std dlc %u NACK", (unsigned)i);
        TEST_BEGIN(name);
        fill_std(&frame, 0x123, (uint8_t)i, payload);
        nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(nbits != 0);
        check_agrees_asym(bits, nbits, EDGE_T0, EDGE_TICKS);
        TEST_END();
    }

    TEST_BEGIN("asymmetric edges, extended frame NACK");
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x18febb11u;
    frame.dlc = 8;
    frame.flags = CAN_FRAME_EXTENDED;
    memcpy(frame.data, payload, 8);
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    check_agrees_asym(bits, nbits, EDGE_T0, EDGE_TICKS);
    TEST_END();

    TEST_BEGIN("asymmetric edges, RTR frame");
    memset(&frame, 0, sizeof(frame));
    frame.id = 0x456;
    frame.dlc = 2;
    frame.flags = CAN_FRAME_RTR;
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    check_agrees_asym(bits, nbits, EDGE_T0, EDGE_TICKS);
    TEST_END();

    TEST_BEGIN("run delta outside the edge-timing budget is a bit error");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    {
        // Find a one-bit recessive run and corrupt just that interval.
        uint32_t bad = CAN_BITS_ERROR;
        int level = 1;
        uint32_t run = 0;
        for (i = 0; i < info.stuffed_bits; i++) {
            uint8_t b = bits_get(bits, i);
            if (b == level) {
                run++;
            } else {
                if (level == 1 && run == 1) {
                    bad = i;  // this falling edge ends a 1-bit recessive run
                    break;
                }
                level = b;
                run = 1;
            }
        }
        CHECK(bad != CAN_BITS_ERROR);
        can_edge_sm_init(&sm, EDGE_TICKS);
        s_asym_bad_bit = (int)bad;
        drive_span_asym(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
        s_asym_bad_bit = -1;
        CHECK(sm.frame_ready);
        CHECK(sm.validity == INVALID_BIT);
    }
    TEST_END();

    // A run just over five bits must still classify as such with the measured
    // edge skew.  Rounding it down to five would silently drop the sixth bit,
    // so the violation would surface later as a CRC error, not INVALID_STUFF.
    TEST_BEGIN("six-bit run with measured edge skew is a stuff error");
    fill_std(&frame, 0x7ff, 8, payload);
    nbits = can_frame_build(&frame, INVALID_STUFF, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    CHECK(max_run(bits, info.stuffed_bits) > 5);
    can_edge_sm_init(&sm, EDGE_TICKS);
    s_asym_bad_bit = -1;
    drive_span_asym(&sm, bits, nbits, EDGE_T0, EDGE_TICKS);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_STUFF);
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

    // The nominal bit sample point is 7/8 T.  A dominant ACK pulse only
    // acknowledges the frame if it is present there; unlike a boundary
    // sample, this rejects a pulse that ends early or starts late.
    TEST_BEGIN("ACK pulse: correct inset timing is acknowledged");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_ack_pulse(&sm, bits, nbits, EDGE_T0, EDGE_TICKS,
                    info.stuffed_bits + 1u, (int32_t)(EDGE_TICKS / 4u),
                    EDGE_TICKS + EDGE_TICKS / 4u);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == VALID_FRAME);
    CHECK(sm.crc_match);
    CHECK(sm.ack_seen);
    CHECK(sm.ack == 1);
    TEST_END();

    TEST_BEGIN("ACK pulse ending before the sample point is not an ACK");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_ack_pulse(&sm, bits, nbits, EDGE_T0, EDGE_TICKS,
                    info.stuffed_bits + 1u, 0, EDGE_TICKS / 2u);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == VALID_FRAME);
    CHECK(sm.crc_match);
    CHECK(sm.ack_seen);
    CHECK(sm.ack == 0);
    TEST_END();

    TEST_BEGIN("ACK pulse starting after the sample point is not an ACK");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_ack_pulse(&sm, bits, nbits, EDGE_T0, EDGE_TICKS,
                    info.stuffed_bits + 1u,
                    (int32_t)((EDGE_TICKS * 15u) / 16u),
                    (EDGE_TICKS * 3u) / 2u);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == VALID_FRAME);
    CHECK(sm.crc_match);
    CHECK(sm.ack_seen);
    CHECK(sm.ack == 0);
    TEST_END();

    TEST_BEGIN("ACK pulse in the CRC delimiter is a form error");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_ack_pulse(&sm, bits, nbits, EDGE_T0, EDGE_TICKS,
                    info.stuffed_bits + 1u, -(int32_t)(EDGE_TICKS / 2u),
                    EDGE_TICKS / 2u);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_BIT);
    TEST_END();

    TEST_BEGIN("ACK pulse crossing the ACK delimiter sample is a form error");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_ack_pulse(&sm, bits, nbits, EDGE_T0, EDGE_TICKS,
                    info.stuffed_bits + 1u,
                    (int32_t)((EDGE_TICKS * 15u) / 16u),
                    (EDGE_TICKS * 15u) / 16u + EDGE_TICKS);
    CHECK(sm.frame_ready);
    CHECK(sm.validity == INVALID_BIT);
    TEST_END();
}

/****************************************************************
 * ACK schedule (can_ack_plan)
 ****************************************************************/

// Stuffer state and wire index at the first CRC bit, derived from the built
// frame.  A stuff bit that separates header/data from the CRC field is
// consumed first, matching what can_edge_sm has in its run/last when it
// accepts the first CRC bit.
static int crc_field_state(const uint8_t *bits, const can_build_info_t *info,
                           uint32_t *crc_wire, uint8_t *run, uint8_t *last) {
    uint32_t crc_start = info->unstuffed_bits - 15u;
    uint32_t pos = 0;
    uint32_t got = 0;
    uint32_t r = 0;
    uint8_t l = 1;

    while (pos < info->stuffed_bits) {
        uint8_t bit = bits_get(bits, pos);
        if (r == 5u) {
            r = 1;
            l = bit;
            pos++;
            continue;
        }
        if (got == crc_start)
            break;
        if (r > 0u && bit == l)
            r++;
        else {
            r = 1;
            l = bit;
        }
        got++;
        pos++;
    }
    if (got != crc_start)
        return -1;
    *crc_wire = pos;
    *run = (uint8_t)r;
    *last = l;
    return 0;
}

static void check_ack_plan_frame(const can_frame_t *frame) {
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    uint32_t crc_wire = 0;
    uint8_t run = 0;
    uint8_t last = 0;
    uint32_t slot;
    uint32_t terminal = 0;
    uint32_t from;
    uint32_t i;

    CHECK(can_frame_build(frame, VALID_FRAME, 0, bits,
                          CAN2040_LITE_MAX_FRAME_BITS, &info) != 0);
    CHECK(crc_field_state(bits, &info, &crc_wire, &run, &last) == 0);
    slot = info.stuffed_bits + 1u - crc_wire;

    // Expected terminal edge from the builder's wire image.
    for (i = 0; i < slot; i++) {
        uint8_t cur = bits_get(bits, crc_wire + i);
        uint8_t prev = (i == 0) ? bits_get(bits, crc_wire - 1u)
                                : bits_get(bits, crc_wire + i - 1u);
        if (cur != prev)
            terminal = i;
    }

    for (from = 0; from <= slot + 1u; from++) {
        can_ack_plan_t plan =
            can_ack_plan(info.crc, run, last, from);
        uint32_t edges = 0;
        uint8_t level;
        uint8_t sync;

        for (i = 1; i < slot; i++) {
            if (i > from &&
                bits_get(bits, crc_wire + i) !=
                    bits_get(bits, crc_wire + i - 1u))
                edges++;
        }
        if (from < slot)
            level = bits_get(bits, crc_wire + from);
        else
            level = 1;
        sync = (terminal == 0)
                   ? bits_get(bits, crc_wire - 1u)
                   : bits_get(bits, crc_wire + terminal - 1u);

        CHECK(plan.slot_bit == slot);
        CHECK(plan.gap_bits == slot - terminal);
        CHECK(plan.edges == edges);
        CHECK(plan.level == level);
        CHECK(plan.sync_level == sync);
        CHECK(plan.ok == (from < terminal ? 1u : 0u));
    }
}

static void test_ack_plan(void) {
    static const uint8_t payload[8] = {
        0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0
    };
    can_frame_t frame;
    uint8_t dlc;

    printf("\n-- ACK schedule --\n");

    TEST_BEGIN("std/ext DLC sweep matches the builder's CRC field");
    {
        int ok = 1;
        for (dlc = 0; dlc <= 8; dlc++) {
            fill_std(&frame, 0x123, dlc, payload);
            check_ack_plan_frame(&frame);
            ok = ok && (g_test_failed == 0);
            fill_std(&frame, 0x18febb11, dlc, payload);
            frame.flags = CAN_FRAME_EXTENDED;
            check_ack_plan_frame(&frame);
            ok = ok && (g_test_failed == 0);
        }
        CHECK(ok);
    }
    TEST_END();

    TEST_BEGIN("no stuff bits in the CRC field");
    {
        can_ack_plan_t plan = can_ack_plan(0x5555u, 1, 1, 0);
        CHECK(plan.slot_bit == 16u);
        CHECK(plan.gap_bits >= 1u && plan.gap_bits <= 5u);
        CHECK(plan.ok);
    }
    TEST_END();

    TEST_BEGIN("four stuff bits when run enters at 5");
    {
        can_ack_plan_t plan = can_ack_plan(0x0000u, 5, 0, 0);
        CHECK(plan.slot_bit == 20u);
        CHECK(plan.gap_bits == 2u); // trailing stuff bit, then delimiter
        CHECK(plan.ok);
    }
    TEST_END();

    TEST_BEGIN("builder emits CRC fields with 0 and 4 stuff bits");
    {
        uint32_t n;
        uint32_t i;
        int saw0 = 0;
        int saw4 = 0;
        for (n = 0; n < 300000u && (!saw0 || !saw4); n++) {
            can_frame_t f;
            uint8_t fb[CAN2040_LITE_MAX_FRAME_BYTES];
            can_build_info_t bi;
            uint32_t cw = 0;
            uint8_t r = 0, l = 0;
            uint32_t slot;
            uint32_t stuffs;

            memset(&f, 0, sizeof(f));
            f.dlc = (uint8_t)((n / 2u) % 9u);
            f.flags = (n & 1u) ? CAN_FRAME_EXTENDED : 0;
            f.id = f.flags ? ((n * 0x45d9fu) & 0x1fffffffu)
                           : ((n * 17u) & 0x7ffu);
            for (i = 0; i < 8; i++)
                f.data[i] = (uint8_t)(n * (i + 3u) + (n >> (i + 1u)));
            if (can_frame_build(&f, VALID_FRAME, 0, fb,
                                CAN2040_LITE_MAX_FRAME_BITS, &bi) == 0)
                continue;
            if (crc_field_state(fb, &bi, &cw, &r, &l) != 0)
                continue;
            slot = bi.stuffed_bits + 1u - cw;
            stuffs = slot - 16u;
            if (stuffs == 0 || stuffs == 4) {
                can_ack_plan_t plan = can_ack_plan(bi.crc, r, l, 0);
                CHECK(plan.slot_bit == slot);
                if (stuffs == 0)
                    saw0 = 1;
                else
                    saw4 = 1;
            }
        }
        CHECK(saw0);
        CHECK(saw4);
    }
    TEST_END();

    TEST_BEGIN("terminal edge on the delimiter boundary (gap 1)");
    {
        can_ack_plan_t plan = can_ack_plan(0x0000u, 4, 0, 0);
        CHECK(plan.gap_bits == 1u);
        CHECK(plan.sync_level == 0u);
        CHECK(plan.ok);
    }
    TEST_END();

    TEST_BEGIN("recessive tail gives gap 5");
    {
        can_ack_plan_t plan = can_ack_plan(0x000fu, 1, 0, 0);
        CHECK(plan.gap_bits == 5u);
        CHECK(plan.sync_level == 0u);
        CHECK(plan.ok);
    }
    TEST_END();

    TEST_BEGIN("from_bit at or past the terminal edge is not armed");
    {
        can_ack_plan_t plan = can_ack_plan(0x0000u, 4, 0, 0);
        uint32_t terminal = plan.slot_bit - plan.gap_bits;
        can_ack_plan_t past = can_ack_plan(0x0000u, 4, 0, terminal);
        can_ack_plan_t after = can_ack_plan(0x0000u, 4, 0, terminal + 1u);
        can_ack_plan_t slot = can_ack_plan(0x0000u, 4, 0, plan.slot_bit);
        CHECK(!past.ok);
        CHECK(!after.ok);
        CHECK(!slot.ok);
    }
    TEST_END();
}

/****************************************************************
 * ACK cycle model (mirrors can_pio_ack_program())
 ****************************************************************/

static uint32_t ack_gap_cycles(uint32_t gap_bits) {
    return gap_bits * ACK_MODEL_CYCLES_PER_BIT + ACK_MODEL_GAP_OFFSET;
}

static void test_ack_cycle_model(void) {
    ack_model_t m;

    printf("\n-- ACK cycle model --\n");

    TEST_BEGIN("follow: alternating edges, no pulse, X does not expire");
    {
        ack_edge_t edges[10];
        int i;
        for (i = 0; i < 10; i++) {
            edges[i].cycle = (i + 1) * 32;
            edges[i].level = (uint8_t)((i & 1) ? 1 : 0);
        }
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0xffffffffu, 0, 0, 0);
        ack_model_run(&m, 1, edges, 10, NULL, 0, 400);
        CHECK(m.pulse_start < 0);
        CHECK(m.x == 0xffffffffu - 10u);
    }
    TEST_END();

    TEST_BEGIN("armed Y=0: recessive through the slot, ends in release");
    {
        ack_edge_t edges[1] = {{64, 0}};
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 0, ack_gap_cycles(1), 0);
        ack_model_run(&m, 1, edges, 1, NULL, 0, 160);
        CHECK(m.pulse_start < 0);
        CHECK(m.pad == 1);
        CHECK(m.pc == ACK_MODEL_WRAP_TARGET);
    }
    TEST_END();

    TEST_BEGIN("armed Y=1, gap 1: dominant slot+0.25..slot+1.25");
    {
        ack_edge_t edges[1] = {{64, 0}};
        uint32_t start = 64u + 32u + 8u;
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 1, ack_gap_cycles(1), 0);
        ack_model_run(&m, 1, edges, 1, NULL, 0, 200);
        CHECK(m.pulse_start == (int)start);
        CHECK(m.pulse_end == (int)(start + 32u));
    }
    TEST_END();

    TEST_BEGIN("armed Y=1, gap 5: same window relative to the slot");
    {
        ack_edge_t edges[1] = {{64, 0}};
        uint32_t start = 64u + 5u * 32u + 8u;
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 1, ack_gap_cycles(5), 0);
        ack_model_run(&m, 1, edges, 1, NULL, 0, 300);
        CHECK(m.pulse_start == (int)start);
        CHECK(m.pulse_end == (int)(start + 32u));
    }
    TEST_END();

    TEST_BEGIN("Y set during the gap still pulses at slot+0.25");
    {
        ack_edge_t edges[1] = {{64, 0}};
        ack_exec_event_t execs[1];
        uint32_t start = 64u + 32u + 8u;
        execs[0].cycle = 96;
        execs[0].exec.kind = ACK_EXEC_SET_Y;
        execs[0].exec.value = 1;
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 0, ack_gap_cycles(1), 0);
        ack_model_run(&m, 1, edges, 1, execs, 1, 200);
        // The allow steals one gap cycle, so it can arrive one TX cycle
        // (1/32 bit) late; the inset has 0.3 bit of margin.
        CHECK(m.pulse_start >= (int)start);
        CHECK(m.pulse_start <= (int)start + 1);
        CHECK(m.pulse_end == m.pulse_start + 32);
    }
    TEST_END();

    TEST_BEGIN("late exec in release stays recessive");
    {
        ack_edge_t edges[1] = {{64, 0}};
        ack_exec_event_t execs[1];
        execs[0].cycle = 120;
        execs[0].exec.kind = ACK_EXEC_SET_Y;
        execs[0].exec.value = 1;
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 0, ack_gap_cycles(1), 0);
        ack_model_run(&m, 1, edges, 1, execs, 1, 160);
        CHECK(m.pulse_start < 0);
        CHECK(m.pad == 1);
    }
    TEST_END();

    TEST_BEGIN("exec during follow does not drop the next edge");
    {
        ack_edge_t edges[2] = {{200, 0}, {232, 1}};
        ack_exec_event_t execs[2];
        int cycle;
        execs[0].cycle = 100;
        execs[0].exec.kind = ACK_EXEC_SET_X;
        execs[0].exec.value = 1;
        execs[1].cycle = 101;
        execs[1].exec.kind = ACK_EXEC_PULL;
        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0xffffffffu, 0, 0, 44);
        for (cycle = 0; cycle <= 235; cycle++) {
            const ack_exec_t *exec = NULL;
            int i;
            uint8_t pin = ack_model_level(edges, 2, 1, cycle);
            for (i = 0; i < 2; i++)
                if (execs[i].cycle == cycle)
                    exec = &execs[i].exec;
            ack_model_step(&m, cycle, pin, exec);
            if (cycle == 202)
                CHECK(m.x == 0); // first edge counted
        }
        CHECK(m.pc == 8 || m.pc == 9); // second edge reached gap
        CHECK(m.x != 0xffffffffu);     // OSR gap was loaded by out x, 32
    }
    TEST_END();

    TEST_BEGIN("both entry polarities pulse at the same slot offset");
    {
        ack_edge_t high_edges[1] = {{64, 0}};
        ack_edge_t low_edges[1] = {{64, 1}};
        uint32_t start = 64u + 32u + 8u;

        ack_model_init(&m, ACK_MODEL_PC_HIGH, 0, 1, ack_gap_cycles(1), 0);
        ack_model_run(&m, 1, high_edges, 1, NULL, 0, 160);
        CHECK(m.pulse_start == (int)start);
        CHECK(m.pulse_end == (int)(start + 32u));

        ack_model_init(&m, ACK_MODEL_PC_LOW, 0, 1, ack_gap_cycles(1), 0);
        ack_model_run(&m, 0, low_edges, 1, NULL, 0, 160);
        CHECK(m.pulse_start == (int)start);
        CHECK(m.pulse_end == (int)(start + 32u));
    }
    TEST_END();
}

/****************************************************************
 * ACK arming model (steps 1-5 of the Pico backend)
 ****************************************************************/

typedef struct {
    uint32_t seq;
    uint32_t step;
    uint32_t arm_step;
    uint32_t allow_step;
    uint32_t arm_from;
    bool handled;
    bool armed;
    bool allowed;
    bool tx_active;
    can_ack_plan_t plan;
} ack_logic_t;

static void ack_logic_reset(ack_logic_t *l) {
    memset(l, 0, sizeof(*l));
    l->arm_step = UINT32_MAX;
    l->allow_step = UINT32_MAX;
}

// Mirrors the backend: compute the plan once at the CRC start, arm when the
// pad matches the plan, allow only after crc_match, and never arm for a frame
// this node transmitted.
static void ack_logic_step(ack_logic_t *l, can_edge_sm_t *sm, uint8_t pad) {
    l->step++;
    if (sm->frame_seq != l->seq) {
        l->seq = sm->frame_seq;
        l->handled = false;
        l->armed = false;
        l->allowed = false;
    }
    if (sm->in_frame && sm->crc_started && !l->handled) {
        uint32_t rel = (sm->last_edge_raw >= sm->crc_raw_start)
                           ? sm->last_edge_raw - sm->crc_raw_start
                           : 0u;
        l->handled = true;
        l->arm_from = rel;
        l->plan = can_ack_plan(sm->crc_calc, sm->crc_run_start,
                               sm->crc_last_start, rel);
        if (!l->tx_active && l->plan.ok && pad == l->plan.level) {
            l->armed = true;
            l->arm_step = l->step;
        }
    }
    if (sm->in_frame && sm->crc_match && l->armed && !l->allowed) {
        l->allowed = true;
        l->allow_step = l->step;
    }
    if (sm->validity == INVALID_CRC || sm->validity == INVALID_STUFF)
        l->armed = false;
}

// Feed one frame bit by bit, advancing a quiet bit clock like the RX path.
static void drive_frame_ack(can_edge_sm_t *sm, const uint8_t *bits,
                            uint32_t nbits, uint32_t t0, uint32_t ticks,
                            ack_logic_t *l, bool tx_active) {
    uint8_t prev = 1;
    uint32_t i;

    can_edge_sm_init(sm, ticks);
    ack_logic_reset(l);
    l->tx_active = tx_active;
    for (i = 0; i < nbits; i++) {
        uint8_t bit = bits_get(bits, i);
        if (bit != prev) {
            can_edge_sm_push(sm, can_edge_word(edge_at(t0, ticks, i), bit));
            ack_logic_step(l, sm, sm->level);
            if (!sm->in_frame)
                return;
        }
        prev = bit;
        if (sm->in_frame) {
            can_edge_sm_time(sm, edge_at(t0, ticks, i + 1u));
            ack_logic_step(l, sm, sm->level);
        }
        if (!sm->in_frame)
            return;
    }
}

static void ack_logic_on_frame(can_frame_t *frame, frame_validity_t validity,
                               ack_logic_t *l, bool tx_active) {
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    can_edge_sm_t sm;
    uint32_t nbits;

    nbits = can_frame_build(frame, validity, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    drive_frame_ack(&sm, bits, nbits, EDGE_T0, EDGE_TICKS, l, tx_active);
}

// A JMP's low 5 bits are the absolute target.  Adding the origin must not
// spill into the condition or delay fields.
static void check_relocated(const uint16_t *insns, uint32_t n,
                            uint32_t origin) {
    uint32_t i;

    for (i = 0; i < n; i++) {
        uint16_t in = insns[i];
        uint16_t got = can_pio_reloc_instr(in, origin);

        if ((in >> 13) == 0) {
            uint32_t addr = in & 31u;
            CHECK(addr + origin < 32u);
            CHECK((got & 31u) == addr + origin);
            CHECK((got & 0xffe0u) == (in & 0xffe0u));
        } else {
            CHECK(got == in);
        }
    }
}

static void test_ack_reloc(void) {
    const can_pio_program_t *ack = can_pio_ack_program();
    const can_pio_program_t *tx = can_pio_tx_program();
    const uint16_t *ack_insns = ack->code;
    const uint16_t *tx_insns = tx->code;
    uint32_t ack_n = ack->length;
    uint32_t tx_n = tx->length;
    const uint32_t origin = 11; // RX at 25, then TX, in pio_add_program
    uint32_t rel;

    printf("\n-- ACK image relocation --\n");

    TEST_BEGIN("ack and tx are 13 and 14 words");
    CHECK(ack_n == 13u);
    CHECK(tx_n == 14u);
    TEST_END();

    TEST_BEGIN("origin 0 leaves both images unchanged");
    check_relocated(ack_insns, ack_n, 0);
    check_relocated(tx_insns, tx_n, 0);
    TEST_END();

    TEST_BEGIN("origin 11 relocates JMP targets and keeps other words");
    check_relocated(ack_insns, ack_n, origin);
    check_relocated(tx_insns, tx_n, origin);
    // release (word 0) and the dominant drive (word 11) are the PC checks.
    CHECK((can_pio_reloc_instr(ack_insns[0], origin) >> 13) != 0);
    CHECK((can_pio_reloc_instr(ack_insns[12], origin) & 31u) == origin);
    CHECK((can_pio_reloc_instr(ack_insns[11], origin) >> 13) != 0);
    TEST_END();

    TEST_BEGIN("allow and release use absolute SMx_ADDR");
    CHECK(!can_ack_release_pc(0, origin));
    CHECK(can_ack_release_pc(origin + CAN_ACK_PC_RELEASE, origin));
    CHECK(!can_ack_allow_pc(origin + CAN_ACK_PC_RELEASE, origin));
    CHECK(!can_ack_allow_pc(origin - 1u, origin));
    for (rel = 1; rel < CAN_ACK_PC_SET_PINS; rel++)
        CHECK(can_ack_allow_pc(origin + rel, origin));
    for (rel = CAN_ACK_PC_SET_PINS; rel < ack_n; rel++)
        CHECK(!can_ack_allow_pc(origin + rel, origin));
    CHECK(CAN_ACK_ENTRY_HIGH == 1u);
    CHECK(CAN_ACK_ENTRY_LOW == 4u);
    TEST_END();
}

static void test_ack_backend_model(void) {
    can_frame_t found[PAT_COUNT];
    can_frame_t dom;
    can_frame_t rec4;
    can_build_info_t info;
    can_edge_sm_t sm;
    ack_logic_t l;
    bool have_dom;
    bool have_rec4;

    printf("\n-- ACK arming model --\n");
    find_tail_patterns(found);
    have_dom = (found[PAT_DOM_EDGE].flags & 0x80u) != 0;
    have_rec4 = (found[PAT_REC4].flags & 0x80u) != 0;
    dom = found[PAT_DOM_EDGE];
    rec4 = found[PAT_REC4];
    dom.flags = (uint8_t)(dom.flags & ~0x80u);
    rec4.flags = (uint8_t)(rec4.flags & ~0x80u);

    TEST_BEGIN("valid frame, dominant last CRC bit, gap 1: arm then allow");
    if (have_dom) {
        ack_logic_on_frame(&dom, VALID_FRAME, &l, false);
        CHECK(l.handled);
        CHECK(l.armed);
        CHECK(l.allowed);
        CHECK(l.plan.gap_bits == 1u);
        CHECK(l.arm_step < l.allow_step);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("valid frame, 5-bit recessive tail: gap preload");
    if (have_rec4) {
        ack_logic_on_frame(&rec4, VALID_FRAME, &l, false);
        CHECK(l.armed);
        CHECK(l.allowed);
        CHECK(l.plan.gap_bits == 5u);
        CHECK(l.plan.gap_bits * ACK_MODEL_CYCLES_PER_BIT +
                  ACK_MODEL_GAP_OFFSET == 162u);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("INVALID_CRC and INVALID_STUFF never set Y");
    {
        can_frame_t f;
        fill_std(&f, 0x123, 8, (const uint8_t *)"12345678");
        ack_logic_on_frame(&f, INVALID_CRC, &l, false);
        CHECK(!l.allowed);
        ack_logic_on_frame(&f, INVALID_STUFF, &l, false);
        CHECK(!l.allowed);
    }
    TEST_END();

    TEST_BEGIN("word that already contains the terminal edge does not arm");
    if (have_dom) {
        uint8_t frame_bits[CAN2040_LITE_MAX_FRAME_BYTES];
        uint32_t crc_wire = 0;
        uint8_t run = 0, last = 0;
        uint32_t slot;
        uint32_t nbits;

        nbits = can_frame_build(&dom, VALID_FRAME, 0, frame_bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        CHECK(nbits != 0);
        CHECK(crc_field_state(frame_bits, &info, &crc_wire, &run, &last) == 0);
        slot = info.stuffed_bits + 1u - crc_wire;
        // Drive through the delimiter bit; the terminal edge is inside the
        // word the model inspects.
        can_edge_sm_init(&sm, EDGE_TICKS);
        ack_logic_reset(&l);
        drive_span(&sm, frame_bits, 0, crc_wire + slot,
                   EDGE_T0, EDGE_TICKS);
        ack_logic_step(&l, &sm, sm.level);
        CHECK(sm.crc_started);
        CHECK(l.handled);
        CHECK(!l.plan.ok);
        CHECK(!l.armed);
    } else {
        CHECK(0);
    }
    TEST_END();

    TEST_BEGIN("a frame this node transmitted is never armed");
    if (have_dom) {
        ack_logic_on_frame(&dom, VALID_FRAME, &l, true);
        CHECK(l.handled);
        CHECK(!l.armed);
        CHECK(!l.allowed);
    } else {
        CHECK(0);
    }
    TEST_END();
}

static void test_edge_pair(void) {
    static const uint8_t payload[8] = {
        0xde, 0xad, 0xbe, 0xef, 0x12, 0x34, 0x56, 0x78
    };
    can_frame_t frame;
    can_frame_t f2;
    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t bits2[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    can_build_info_t info2;
    can_edge_sm_t sm;
    can_parsed_t want;
    uint32_t nbits;
    uint32_t n2;

    printf("\n-- Edge pair decoder --\n");

    TEST_BEGIN("16-bit deltas reconstruct timestamps across a wrap");
    {
        can_edge_sm_t s;
        can_edge_pair_t p;

        can_edge_sm_init(&s, EDGE_TICKS);
        can_edge_pair_init(&p, &s, 0x10010u, 0);
        // 1->0 at 0x10010 (t16 0x0010), 0->1 at 0x0fff0 (t16 0xfff0).
        can_edge_pair_push(&p, (0x0010u << 16) | 0xfff0u);
        CHECK(p.time == 0x0fff0u);
        CHECK(p.last == 0xfff0u);
        // Continue below the wrap; the delta is still exact.
        can_edge_pair_push(&p, (0xffe0u << 16) | 0xffd0u);
        CHECK(p.time == 0x0ffd0u);
        CHECK(p.last == 0xffd0u);
    }
    TEST_END();

    TEST_BEGIN("frame decodes from pair words across a 16-bit wrap");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    CHECK(can_frame_decode(bits, nbits, &want) == VALID_FRAME);
    // 0x10100 is chosen so the frame's timestamps roll past 0x10000.
    drive_pairs(&sm, bits, nbits, 0x10100u, EDGE_TICKS);
    check_parsed(&sm, &want);
    TEST_END();

    TEST_BEGIN("wrapped idle gap does not disturb the next frame");
    fill_std(&frame, 0x111, 2, payload);
    fill_std(&f2, 0x222, 1, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 0, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    n2 = can_frame_build(&f2, VALID_FRAME, 1, bits2,
                         CAN2040_LITE_MAX_FRAME_BITS, &info2);
    CHECK(n2 != 0);
    {
        can_edge_pair_t pair;
        uint32_t t0 = 0x80000u;
        uint32_t t1;

        can_edge_sm_init(&sm, EDGE_TICKS);
        can_edge_pair_init(&pair, &sm, t0, 0);
        feed_pairs(&pair, &sm, bits, nbits, t0, EDGE_TICKS);
        CHECK(sm.in_frame);
        can_edge_sm_time(&sm, (pair.time - 16u * EDGE_TICKS) &
                              CAN_EDGE_TIME_MASK);
        CHECK(sm.frame_ready);
        CHECK(sm.validity == VALID_FRAME);
        sm.frame_ready = 0;

        // More than 2^16 ticks later, so the 16-bit idle delta wraps.
        t1 = (t0 - (nbits + 3000u) * EDGE_TICKS) & CAN_EDGE_TIME_MASK;
        feed_pairs(&pair, &sm, bits2, n2, t1, EDGE_TICKS);
        CHECK(sm.in_frame);
        can_edge_sm_time(&sm, (pair.time - 16u * EDGE_TICKS) &
                              CAN_EDGE_TIME_MASK);
        CHECK(sm.frame_ready);
        CHECK(sm.validity == VALID_FRAME);
        CHECK(sm.parsed.id == 0x222);
        CHECK(sm.parsed.dlc == 1);
        CHECK(can_frame_decode(bits2, n2, &want) == VALID_FRAME);
        CHECK(memcmp(sm.parsed.data, want.data, 1) == 0);
    }
    TEST_END();

#if CAN_EDGE_PAIR_TIME_RECORDING
    TEST_BEGIN("capture log keeps every pair and replays the frame");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    CHECK(can_frame_decode(bits, nbits, &want) == VALID_FRAME);
    {
        can_edge_pair_t pair;
        can_edge_sm_t replay;
        uint32_t t0 = EDGE_T0;
        uint32_t count;
        uint32_t i;
        uint32_t ntrans = 0;
        uint32_t transitions[CAN2040_LITE_MAX_FRAME_BITS];
        uint8_t prev = 1;

        can_edge_sm_init(&sm, EDGE_TICKS);
        can_edge_pair_init(&pair, &sm, t0, 0);
        can_edge_pair_record_clear();
        feed_pairs(&pair, &sm, bits, nbits, t0, EDGE_TICKS);
        CHECK(sm.in_frame);
        can_edge_sm_time(&sm, (pair.time - 16u * EDGE_TICKS) &
                              CAN_EDGE_TIME_MASK);
        CHECK(sm.frame_ready);
        check_parsed(&sm, &want);

        count = can_edge_pair_record_count();
        CHECK(count > 0);
        CHECK(can_edge_pair_record_dropped() == 0);
        CHECK(can_edge_pair_record_get(count) == NULL);

        // Recorded times must equal the ideal edge positions the feeder used.
        for (i = 0; i < nbits; i++) {
            uint8_t bit = bits_get(bits, i);
            if (bit == prev)
                continue;
            prev = bit;
            transitions[ntrans++] = i;
        }
        CHECK(ntrans == 2u * count);
        for (i = 0; i < count; i++) {
            const can_edge_pair_record_t *r = can_edge_pair_record_get(i);
            uint32_t fall_bit = transitions[2u * i];
            uint32_t rise_bit = transitions[2u * i + 1u];

            CHECK(r->fall_time == edge_at(t0, EDGE_TICKS, fall_bit));
            CHECK(r->rise_time == edge_at(t0, EDGE_TICKS, rise_bit));
            CHECK((r->word >> 16) ==
                  (uint16_t)(edge_at(t0, EDGE_TICKS, fall_bit) +
                             PAIR_SAMPLE_OFFSET));
            CHECK((uint16_t)r->word ==
                  (uint16_t)(edge_at(t0, EDGE_TICKS, rise_bit) +
                             PAIR_SAMPLE_OFFSET));
        }

        // The same log must drive a fresh parser to the same result.
        can_edge_sm_init(&replay, EDGE_TICKS);
        for (i = 0; i < count; i++) {
            const can_edge_pair_record_t *r = can_edge_pair_record_get(i);
            can_edge_sm_push(&replay, can_edge_word(r->fall_time, 0));
            can_edge_sm_push(&replay, can_edge_word(r->rise_time, 1));
        }
        can_edge_sm_time(&replay,
                         (can_edge_pair_record_get(count - 1u)->rise_time -
                          16u * EDGE_TICKS) & CAN_EDGE_TIME_MASK);
        CHECK(replay.frame_ready);
        check_parsed(&replay, &want);
    }
    TEST_END();

    TEST_BEGIN("capture log locates the ACK pulse in its slot");
    fill_std(&frame, 0x123, 8, payload);
    nbits = can_frame_build(&frame, VALID_FRAME, 1, bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    CHECK(nbits != 0);
    {
        can_edge_pair_t pair;
        uint32_t t0 = EDGE_T0;
        uint32_t ack_index = info.stuffed_bits + 1u;
        uint32_t found = 0;
        uint32_t i;

        can_edge_sm_init(&sm, EDGE_TICKS);
        can_edge_pair_init(&pair, &sm, t0, 0);
        can_edge_pair_record_clear();
        feed_pairs(&pair, &sm, bits, nbits, t0, EDGE_TICKS);

        for (i = 0; i < can_edge_pair_record_count(); i++) {
            const can_edge_pair_record_t *r = can_edge_pair_record_get(i);
            if (r->fall_time != edge_at(t0, EDGE_TICKS, ack_index))
                continue;
            found++;
            CHECK(r->rise_time == edge_at(t0, EDGE_TICKS, ack_index + 1u));
            CHECK(bits_get(bits, ack_index) == 0);
        }
        CHECK(found == 1u);
    }
    TEST_END();
#endif // CAN_EDGE_PAIR_TIME_RECORDING
}

// Replay the raw Fly-Micro4 RX FIFO recording through the production edge
// decoder.  Between words, advance the quiet tail only up to one tick before
// the next recorded edge, matching the Pico service's behind-real-time
// estimate.  The recording starts at a SOF and holds three complete frames.
static void test_recorded_fly_micro4(void) {
    can_edge_sm_t sm;
    can_edge_pair_t pair;
    static const uint8_t expected[3][8] = {
        {0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25},
        {0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26},
        {0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27},
    };
    static const uint16_t expected_crc[3] = {0x7B32, 0x72B8, 0x30DA};
    const uint32_t *words = can_rx_record_fly_micro4_1m;
    const uint32_t n = sizeof(can_rx_record_fly_micro4_1m) /
                       sizeof(can_rx_record_fly_micro4_1m[0]);
    uint32_t got = 0;
    uint32_t i;

    printf("\n-- Recorded Fly-Micro4 RX edges --\n");
    TEST_BEGIN("recorded 1 Mbit RX edges decode three frames");
    can_edge_sm_init(&sm, CAN_RX_RECORD_FLY_MICRO4_TICKS_PER_BIT);
    can_edge_pair_init(&pair, &sm, 0, 0);
    for (i = 0; i < n; i++) {
        can_edge_pair_push(&pair, words[i]);
        if (i + 1u < n && sm.in_frame) {
            const uint32_t next_fall16 = (words[i + 1u] >> 16) & 0xffffu;
            const uint32_t d = (uint16_t)(pair.last - next_fall16);
            const uint32_t next_fall =
                (pair.time - d) & CAN_EDGE_TIME_MASK;
            can_edge_sm_time(&sm, (next_fall + 1u) & CAN_EDGE_TIME_MASK);
        }
        while (sm.frame_ready) {
            if (sm.validity == VALID_FRAME &&
                sm.parsed.id == 0x123u &&
                sm.parsed.dlc == 8u &&
                sm.parsed.crc == sm.parsed.crc_calc &&
                got < 3u) {
                CHECK(memcmp(sm.parsed.data, expected[got], 8) == 0);
                CHECK(sm.parsed.crc == expected_crc[got]);
                got++;
            }
            sm.frame_ready = 0;
        }
    }
    CHECK(got == 3u);
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
    test_api_queue();
    test_edge_sm();
    test_recorded_fly_micro4();
    test_ack_plan();
    test_ack_cycle_model();
    test_ack_reloc();
    test_ack_backend_model();
    test_edge_pair();

    printf("\n========================================\n");
    printf("TOTAL: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");
    return g_failed ? 1 : 0;
}
