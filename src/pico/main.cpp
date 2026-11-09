// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// main.cpp — On-target C++ harness.  Decodes a fixed set of frame vectors
// with the same portable core that the host tests use, and reports over USB
// serial.

#include <Arduino.h>
#include <string.h>

#include "can2040_lite.h"
#include "can2040_lite_pico.h"
#include "can2040_lite_proto.h"
#include "can_frame_build.h"

struct frame_vector {
    const char *name;
    const uint8_t *bits;
    uint32_t total_bits;
    frame_validity_t expected;
    uint32_t id;
    uint8_t dlc;
    uint8_t is_extended;
    uint8_t is_rtr;
    uint16_t crc;
};

// Generated with: can_gen <options> --c
static const uint8_t f_std[] = {
    0x12, 0x31, 0x05, 0x23, 0x45, 0x67, 0x89, 0xAB,
    0xCD, 0xEF, 0x05, 0x73, 0x8F, 0xFF, 0x80,
};
static const uint8_t f_ext[] = {
    0x63, 0xEF, 0x5D, 0x88, 0x88, 0xDE, 0xAD, 0xBE,
    0x77, 0x89, 0x1A, 0x2B, 0x3C, 0x4E, 0x1A, 0xFF,
    0xF8,
};
static const uint8_t f_rtr[] = {
    0x45, 0x68, 0x29, 0x56, 0xDF, 0xFF,
};
static const uint8_t f_bad_crc[] = {
    0x12, 0x31, 0x05, 0x23, 0x45, 0x67, 0x89, 0xAB,
    0xCD, 0xEF, 0x05, 0x73, 0x9F, 0xFF, 0x80,
};
static const uint8_t f_bad_stuff[] = {
    0x7F, 0xF4, 0x46, 0xF5, 0x6D, 0xF3, 0xBC, 0x48,
    0xD1, 0x59, 0xE3, 0x9A, 0xCF, 0xFF, 0xC0,
};
static const uint8_t f_bad_bit[] = {
    0x12, 0x31, 0x12, 0x46, 0x8A, 0xCF, 0x13, 0x57,
    0x9B, 0xDE, 0x0A, 0xE7, 0x1F, 0xFF,
};

static const frame_vector vectors[] = {
    {"std 0x123 dlc8",  f_std,       113, VALID_FRAME,   0x123,      8, 0, 0, 0x1738},
    {"ext 0x18FEBB11",  f_ext,       133, VALID_FRAME,   0x18febb11, 8, 1, 0, 0x4e1a},
    {"std RTR 0x456",   f_rtr,        48, VALID_FRAME,   0x456,      0, 0, 1, 0x4ab6},
    {"bad CRC",         f_bad_crc,   113, INVALID_CRC,   0,          0, 0, 0, 0},
    {"bad stuff",       f_bad_stuff, 114, INVALID_STUFF, 0,          0, 0, 0, 0},
    {"bad data bit",    f_bad_bit,   112, INVALID_CRC,   0,          0, 0, 0, 0},
};

static int passed;
static int failed;

static const char *validity_name(frame_validity_t v) {
    switch (v) {
    case VALID_FRAME:   return "VALID";
    case INVALID_CRC:   return "INVALID_CRC";
    case INVALID_STUFF: return "INVALID_STUFF";
    case INVALID_BIT:   return "INVALID_BIT";
    }
    return "?";
}

static void run_vector(const frame_vector *v) {
    can_parsed_t p;
    frame_validity_t got = can_frame_decode(v->bits, v->total_bits, &p);
    bool ok = (got == v->expected);

    if (ok && got == VALID_FRAME) {
        ok = (p.id == v->id) &&
             (p.dlc == v->dlc) &&
             (p.is_extended == v->is_extended) &&
             (p.is_rtr == v->is_rtr) &&
             (p.crc == v->crc) &&
             (p.crc_calc == v->crc);
    }

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  v->name, validity_name(v->expected), validity_name(got),
                  ok ? "PASS" : "FAIL");
    if (ok)
        passed++;
    else
        failed++;
}


static void run_api_smoke() {
    struct can2040_lite cd;
    struct can2040_lite_stats st;
    struct can2040_lite_msg msg;
    bool ok;

    memset(&msg, 0, sizeof(msg));
    can2040_lite_setup(&cd, 0);
    can2040_lite_start(&cd, 150000000u, 500000u, 0, 1);

    msg.id = 0x123;
    msg.dlc = 8;
    ok = can2040_lite_check_transmit(&cd) == 1;
    ok = ok && (can2040_lite_transmit(&cd, &msg) == 0);

    can2040_lite_get_statistics(&cd, &st);
    ok = ok && (st.tx_attempt == 1);
    can2040_lite_stop(&cd);
    ok = ok && (can2040_lite_check_transmit(&cd) == 0);

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  "API skeleton", "usable", "usable", ok ? "PASS" : "FAIL");
    if (ok)
        passed++;
    else
        failed++;
}


/****************************************************************
 * PIO loopback (TX and RX on the same GPIO)
 ****************************************************************/
#ifndef CAN2040_LITE_ENABLE_PIO_LOOPBACK
#define CAN2040_LITE_ENABLE_PIO_LOOPBACK 0
#endif

#if CAN2040_LITE_ENABLE_PIO_LOOPBACK
#define LOOP_GPIO 2
static bool loopback_done;
static uint8_t g_tx_bits[CAN2040_LITE_MAX_FRAME_BYTES];
static uint32_t g_tx_total;
static uint32_t g_last_tx;

static const uint8_t loop_data[8] = {0x12, 0x34, 0x56, 0x78,
                                     0x9a, 0xbc, 0xde, 0xf0};

static bool loopback_case(const char *name, int ack) {
    can_frame_t f;
    can_build_info_t info;
    can_parsed_t p;
    uint8_t tx_bits[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t rx_bits[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t total;
    uint32_t rx_n;
    frame_validity_t got;
    bool ok = true;

    memset(&f, 0, sizeof(f));
    f.id = 0x123;
    f.dlc = 8;
    memcpy(f.data, loop_data, 8);

    memset(tx_bits, 0, sizeof(tx_bits));
    total = can_frame_build(&f, VALID_FRAME, ack, tx_bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    ok = ok && (total != 0);

    if (ack) {
        memcpy(g_tx_bits, tx_bits, sizeof(g_tx_bits));
        g_tx_total = total;
        g_last_tx = millis();
    }

    can2040_lite_pico_rx_arm();
    can2040_lite_pico_tx_frame(tx_bits, total);
    ok = ok && can2040_lite_pico_rx_wait(100);

    memset(rx_bits, 0, sizeof(rx_bits));
    rx_n = can2040_lite_pico_rx_bits(rx_bits, CAN2040_LITE_MAX_FRAME_BITS);
    got = can_frame_decode(rx_bits, rx_n, &p);

    ok = ok && (got == VALID_FRAME);
    if (got == VALID_FRAME) {
        ok = ok && (p.id == f.id) && (p.dlc == 8) && (p.ack == ack);
        ok = ok && (memcmp(p.data, loop_data, 8) == 0);
    }

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  name, ack ? "VALID/ACK" : "VALID/NACK",
                  validity_name(got), ok ? "PASS" : "FAIL");
    Serial.flush();
    if (!ok) {
        Serial.printf("    rx bits: %u, decoded id=0x%X dlc=%u ack=%u\n",
                      (unsigned)rx_n, (unsigned)p.id, (unsigned)p.dlc,
                      (unsigned)p.ack);
        Serial.printf("    first tx bits:");
        for (uint32_t i = 0; i < 40; i++)
            Serial.printf("%u", bits_get(tx_bits, i));
        Serial.println();
        Serial.printf("    first rx bits:");
        for (uint32_t i = 0; i < 40; i++)
            Serial.printf("%u", bits_get(rx_bits, i));
        Serial.println();
    }
    if (ok)
        passed++;
    else
        failed++;
    return ok;
}

static void run_loopback() {
    if (can2040_lite_pico_rx_init(LOOP_GPIO, 500000) < 0 ||
        can2040_lite_pico_tx_init(LOOP_GPIO, 500000) < 0) {
        Serial.println("  PIO loopback       PIO init FAILED");
        failed += 2;
        return;
    }
    loopback_case("PIO loopback ACK", 1);
    loopback_case("PIO loopback NACK", 0);
}
#endif // CAN2040_LITE_ENABLE_PIO_LOOPBACK

static void run_suite() {
    passed = 0;
    failed = 0;

    Serial.println();
    Serial.println("========================================");
    Serial.println("CAN2040-Lite on-target protocol harness");
    Serial.println("========================================");
    Serial.printf("frame bits max: %u, bytes: %u\n",
                  (unsigned)CAN2040_LITE_MAX_FRAME_BITS,
                  (unsigned)CAN2040_LITE_MAX_FRAME_BYTES);

    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++)
        run_vector(&vectors[i]);
    run_api_smoke();

    Serial.println("========================================");
    Serial.printf("TOTAL: %d passed, %d failed\n", passed, failed);
    Serial.println(failed ? "RESULT: FAIL" : "RESULT: PASS");
}


void setup() {
    Serial.begin(115200);
    delay(1500);
    run_suite();
}

void loop() {
#if CAN2040_LITE_ENABLE_PIO_LOOPBACK
    if (!loopback_done) {
        loopback_done = true;
        delay(500);
        Serial.println();
        Serial.println("=== PIO loopback ===");
        Serial.flush();
        run_loopback();
    }
#endif
    // Send any character over USB serial to re-run the vector suite.
    if (Serial.available()) {
        while (Serial.available())
            (void)Serial.read();
        run_suite();
    }
#if CAN2040_LITE_ENABLE_PIO_LOOPBACK
    // Keep transmitting for Saleae/TX-timing captures.
    if (g_tx_total && (millis() - g_last_tx) > 200) {
        g_last_tx = millis();
        can2040_lite_pico_tx_frame(g_tx_bits, g_tx_total);
    }
#endif
    delay(20);
}
