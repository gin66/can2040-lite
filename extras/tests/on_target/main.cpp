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

#include <hardware/irq.h>
#include <hardware/pio.h>

#include "can2040.h"
#include "can2040_lite_pico.h"
#include "can2040_lite_proto.h"
#include "can_edge_sm.h"
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


#ifndef CAN2040_LITE_ENABLE_PIO_LOOPBACK
#define CAN2040_LITE_ENABLE_PIO_LOOPBACK 0
#endif

#ifndef CAN2040_LITE_BITRATE
#define CAN2040_LITE_BITRATE 500000u
#endif

// CAN-TX and CAN-RX share this pin in loopback tests.
#define LOOP_GPIO 2

static struct can2040 cbus;
static volatile uint32_t public_tx_notify;
static volatile uint32_t public_rx_notify;

static void can2040_app_cb(struct can2040 *cd, uint32_t notify,
                           struct can2040_msg *msg) {
    (void)cd;
    (void)msg;
    if (notify & CAN2040_NOTIFY_TX)
        public_tx_notify++;
    if (notify & CAN2040_NOTIFY_RX)
        public_rx_notify++;
}

static void can2040_pio_irq(void) {
    can2040_pio_irq_handler(&cbus);
}

// Application entry point, matching the can2040 documented startup sequence.
static void run_api_smoke() {
    struct can2040_stats st;
    struct can2040_msg msg;
    uint32_t start;
    bool ok;

    memset(&msg, 0, sizeof(msg));
    public_tx_notify = 0;
    public_rx_notify = 0;

    can2040_setup(&cbus, 0);
    can2040_callback_config(&cbus, can2040_app_cb);
    irq_set_exclusive_handler(PIO0_IRQ_0, can2040_pio_irq);
    irq_set_enabled(PIO0_IRQ_0, true);
    can2040_start(&cbus, 150000000u, CAN2040_LITE_BITRATE,
                  LOOP_GPIO, LOOP_GPIO);

    msg.id = 0x123;
    msg.dlc = 8;
    for (uint32_t i = 0; i < 8; i++)
        msg.data[i] = (uint8_t)(0x10u + i);

    ok = can2040_check_transmit(&cbus) == 1;
    ok = ok && (can2040_transmit(&cbus, &msg) == 0);

    // A single-pin loopback reads a recessive ACK slot, so NOTIFY_TX only
    // arrives with a second node.  Stop once the result is in either way.
    start = millis();
    while (millis() - start < 50u) {
        can2040_get_statistics(&cbus, &st);
        if (public_tx_notify || st.tx_nack || st.tx_error ||
            st.tx_attempt > st.tx_total)
            break;
        delay(1);
    }

    can2040_get_statistics(&cbus, &st);
    ok = ok && (st.tx_attempt == 1);
    can2040_stop(&cbus);
    ok = ok && (can2040_check_transmit(&cbus) == 0);

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  "Public API", "usable", "usable", ok ? "PASS" : "FAIL");
    if (ok)
        passed++;
    else
        failed++;
}


/****************************************************************
 * PIO loopback (TX and RX on the same GPIO)
 ****************************************************************/

#if CAN2040_LITE_ENABLE_PIO_LOOPBACK
static bool loopback_done;
static uint8_t g_tx_bits[CAN2040_LITE_MAX_FRAME_BYTES];
static uint32_t g_tx_total;
static uint32_t g_last_tx;

static const uint8_t loop_data[8] = {0x12, 0x34, 0x56, 0x78,
                                     0x9a, 0xbc, 0xde, 0xf0};

static bool tx_wait_done(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (!can2040_lite_pico_tx_done_pending() &&
           !can2040_lite_pico_tx_error_pending()) {
        if (millis() - start > timeout_ms)
            return false;
    }
    return can2040_lite_pico_tx_done_pending() != 0;
}

static bool loopback_case(const char *name, int ack) {
    can_frame_t f;
    can_build_info_t info;
    can_parsed_t p;
    uint8_t tx_bits[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t total;
    frame_validity_t got;
    bool ok = true;

    memset(&p, 0, sizeof(p));
    memset(&f, 0, sizeof(f));
    f.id = 0x123;
    f.dlc = 8;
    memcpy(f.data, loop_data, 8);

    memset(tx_bits, 0, sizeof(tx_bits));
    total = can_frame_build(&f, VALID_FRAME, ack, tx_bits,
                            CAN2040_LITE_MAX_FRAME_BITS, &info);
    ok = ok && (total != 0);

    // The SM drives through the CRC delimiter and parks recessive. On a
    // single pin the following ACK slot reads recessive.
    uint32_t tx_n = info.stuffed_bits + 1u;
    if (!ack) {
        // Keep the NACK frame for the periodic Saleae capture.
        memcpy(g_tx_bits, tx_bits, sizeof(g_tx_bits));
        g_tx_total = tx_n;
        g_last_tx = millis();
    }

    can2040_lite_pico_rx_arm();
    can2040_lite_pico_tx_frame(tx_bits, tx_n);
    ok = ok && can2040_lite_pico_rx_wait(100);
    ok = ok && can2040_lite_pico_tx_done_pending();

    // Use the edge-based parsed frame directly (includes one-bit ACK).
    const can_parsed_t *pp = can2040_lite_pico_rx_get_parsed();
    got = pp ? pp->validity : INVALID_BIT;
    if (pp)
        memcpy(&p, pp, sizeof(p));

    ok = ok && (got == VALID_FRAME);
    if (got == VALID_FRAME) {
        ok = ok && (p.id == f.id) && (p.dlc == 8) && (p.ack == 0);
        ok = ok && (memcmp(p.data, loop_data, 8) == 0);
    }

#if CAN_EDGE_PAIR_TIME_RECORDING
    // The RX FIFO log doubles as a logic analyzer: every edge pair of the
    // transmitted frame must be present, and fall/rise times stay usable.
    {
        uint32_t transitions = 0;
        uint32_t records = can_edge_pair_record_count();
        uint8_t prev = 1;

        for (uint32_t i = 0; i < tx_n; i++) {
            uint8_t bit = bits_get(tx_bits, i);
            if (bit != prev)
                transitions++;
            prev = bit;
        }
        ok = ok && (records == transitions / 2u);
        ok = ok && (can_edge_pair_record_dropped() == 0);
        Serial.printf("    capture log: %u pairs, %u expected\n",
                      (unsigned)records, (unsigned)(transitions / 2u));
    }
#endif

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  name, "VALID/NACK",
                  validity_name(got), ok ? "PASS" : "FAIL");
    Serial.flush();
    if (!ok) {
        Serial.printf("    decoded id=0x%X dlc=%u ack=%u validity=%s\n",
                      (unsigned)p.id, (unsigned)p.dlc, (unsigned)p.ack,
                      validity_name(p.validity));
        Serial.printf("    first tx bits:");
        for (uint32_t i = 0; i < 40; i++)
            Serial.printf("%u", bits_get(tx_bits, i));
        Serial.println();
    }
    if (ok)
        passed++;
    else
        failed++;
    return ok;
}


static void loopback_two_frames() {
    static const uint8_t data2[8] = {0xf0, 0xde, 0xbc, 0x9a,
                                     0x78, 0x56, 0x34, 0x12};
    can_frame_t f1;
    can_frame_t f2;
    can_build_info_t i1;
    can_build_info_t i2;
    can_parsed_t p1;
    can_parsed_t p2;
    uint8_t b1[CAN2040_LITE_MAX_FRAME_BYTES];
    uint8_t b2[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t n1;
    uint32_t n2;
    frame_validity_t g1;
    frame_validity_t g2;
    bool ok = true;

    memset(&p1, 0, sizeof(p1));
    memset(&p2, 0, sizeof(p2));
    memset(&f1, 0, sizeof(f1));
    f1.id = 0x123;
    f1.dlc = 8;
    memcpy(f1.data, loop_data, 8);
    memset(&f2, 0, sizeof(f2));
    f2.id = 0x456;
    f2.dlc = 8;
    memcpy(f2.data, data2, 8);

    n1 = can_frame_build(&f1, VALID_FRAME, 1, b1,
                         CAN2040_LITE_MAX_FRAME_BITS, &i1);
    n2 = can_frame_build(&f2, VALID_FRAME, 0, b2,
                         CAN2040_LITE_MAX_FRAME_BITS, &i2);
    ok = ok && n1 && n2;

    // First capture.
    can2040_lite_pico_rx_arm();
    can2040_lite_pico_tx_frame(b1, i1.stuffed_bits + 1u);
    ok = ok && can2040_lite_pico_rx_wait(200);
    const can_parsed_t *pp1 = can2040_lite_pico_rx_get_parsed();
    g1 = pp1 ? pp1->validity : INVALID_BIT;
    if (pp1)
        memcpy(&p1, pp1, sizeof(p1));
    ok = ok && (g1 == VALID_FRAME);
    ok = ok && (p1.id == 0x123) && (p1.ack == 0);
    ok = ok && (memcmp(p1.data, loop_data, 8) == 0);

    // Second capture after re-arming.
    can2040_lite_pico_rx_arm();
    can2040_lite_pico_tx_frame(b2, i2.stuffed_bits + 1u);
    ok = ok && can2040_lite_pico_rx_wait(200);
    const can_parsed_t *pp2 = can2040_lite_pico_rx_get_parsed();
    g2 = pp2 ? pp2->validity : INVALID_BIT;
    if (pp2)
        memcpy(&p2, pp2, sizeof(p2));
    ok = ok && (g2 == VALID_FRAME);
    ok = ok && (p2.id == 0x456) && (p2.ack == 0);
    ok = ok && (memcmp(p2.data, data2, 8) == 0);

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  "PIO two frames", "2 frames", validity_name(g2),
                  ok ? "PASS" : "FAIL");
    Serial.flush();
    if (!ok) {
        Serial.printf("    first: %s id=0x%X\n",
                      validity_name(g1), (unsigned)p1.id);
        Serial.printf("    second: %s id=0x%X\n",
                      validity_name(g2), (unsigned)p2.id);
    }
    if (ok)
        passed++;
    else
        failed++;
}

#define ABORT_RX_GPIO 3

static void expect_tx_abort(const char *name) {
    uint32_t start = millis();
    bool aborted;
    bool high;
    bool ok;

    while (!can2040_lite_pico_tx_error_pending() &&
           (millis() - start) < 20u)
        ;
    aborted = can2040_lite_pico_tx_error_pending() != 0;
    high = digitalRead(LOOP_GPIO) == HIGH;
    ok = aborted && high;

    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  name, "stopped", aborted ? "stopped" : "kept going",
                  ok ? "PASS" : "FAIL");
    Serial.flush();
    if (!ok)
        Serial.printf("    tx line %s, abort %u\n", high ? "high" : "low",
                      (unsigned)aborted);
    if (ok)
        passed++;
    else
        failed++;
}

// Readback is a separate pin, so it does not echo CAN-TX. A conflict is only
// a recessive bit against a dominant bus. Dominant bits are not compared.
// The pattern ends recessive so the park leaves CAN-TX released.
static void loopback_abort() {
    static const uint8_t recessive[8] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };
    // Bit 31 is the last driven bit and must be recessive.
    static const uint8_t dominant[4] = {0x00, 0x00, 0x00, 0x01};
    const uint32_t bit_us = 1000000u / CAN2040_LITE_BITRATE;
    bool echo_ok;

    can2040_lite_pico_tx_clear_status();
    can2040_lite_pico_tx_set_readback_gpio(ABORT_RX_GPIO);
    pinMode(ABORT_RX_GPIO, OUTPUT);
    digitalWrite(ABORT_RX_GPIO, HIGH);

    // All recessive, so a recessive readback matches until we force dominant.
    can2040_lite_pico_tx_frame(recessive, 64);
    delayMicroseconds(bit_us * 13u);
    digitalWrite(ABORT_RX_GPIO, LOW);
    delayMicroseconds(bit_us * 6u);
    digitalWrite(ABORT_RX_GPIO, HIGH);
    expect_tx_abort("PIO abort");

    can2040_lite_pico_tx_clear_status();
    digitalWrite(ABORT_RX_GPIO, HIGH);
    can2040_lite_pico_tx_frame(dominant, 32);
    echo_ok = tx_wait_done(20) && digitalRead(LOOP_GPIO) == HIGH &&
              !can2040_lite_pico_tx_error_pending();
    Serial.printf("  %-18s expected %-13s got %-13s %s\n",
                  "PIO dominant ok", "finished",
                  echo_ok ? "finished" : "aborted",
                  echo_ok ? "PASS" : "FAIL");
    Serial.flush();
    if (echo_ok)
        passed++;
    else
        failed++;

    can2040_lite_pico_tx_clear_status();
    can2040_lite_pico_tx_set_readback_gpio(LOOP_GPIO);
}

static void run_loopback() {
    if (can2040_lite_pico_rx_init(LOOP_GPIO, CAN2040_LITE_BITRATE) < 0 ||
        can2040_lite_pico_tx_init(LOOP_GPIO, LOOP_GPIO,
                                  CAN2040_LITE_BITRATE) < 0) {
        Serial.println("  PIO loopback       PIO init FAILED");
        failed += 2;
        return;
    }
    loopback_case("PIO loopback ACK", 1);
    loopback_case("PIO loopback NACK", 0);
    loopback_two_frames();
    loopback_abort();
}
#endif // CAN2040_LITE_ENABLE_PIO_LOOPBACK


#ifndef CAN2040_LITE_SALEAE_SELFTEST
#define CAN2040_LITE_SALEAE_SELFTEST 0
#endif

#if CAN2040_LITE_SALEAE_SELFTEST
// Mirrors FastAccelStepper's SR_00 wiring self-test: analyzer channel i is
// wired to GPIO 2+i and toggles with high time 50*(i+1) ms in a 1 s period.
static void saleae_selftest() {
    static const uint8_t pins[8] = {2, 3, 4, 5, 6, 7, 8, 9};
    uint32_t start;
    uint32_t i;

    for (i = 0; i < 8; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], LOW);
    }
    start = millis();
    while (millis() - start < 4000u) {
        uint32_t phase = (millis() - start) % 1000u;
        for (i = 0; i < 8; i++)
            digitalWrite(pins[i], (phase < 50u * (i + 1u)) ? HIGH : LOW);
        delay(1);
    }
    for (i = 0; i < 8; i++)
        digitalWrite(pins[i], LOW);
}
#endif

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
#if CAN2040_LITE_SALEAE_SELFTEST
    saleae_selftest();
#endif
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
    // Keep transmitting for Saleae/TX-timing captures (one message per tick;
    // the transmitter is one-shot and parks after each message).
    if (g_tx_total && (millis() - g_last_tx) > 200) {
        g_last_tx = millis();
        can2040_lite_pico_tx_frame(g_tx_bits, g_tx_total);
    }
#endif
    delay(20);
}
