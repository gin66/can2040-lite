// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite.c — RP2040/RP2350 driver core: public API, TX queue, packer.
//
// This file has no pico-sdk dependency so the host tests compile and exercise
// the same core as the target firmware.  Hardware operations are fixed calls
// into can2040_lite_backend.h; the target links the Pico backend and the host
// test binary links its fake backend.

#include <stddef.h>
#include <string.h>

#include "can2040_lite.h"
#include "can2040_lite_backend.h"

// One live instance.  A second start() on another object while the first is
// started leaves the hardware alone.
static struct can2040_lite *g_started;

void can2040_lite_setup(struct can2040_lite *cd, uint32_t pio_num) {
    if (!cd)
        return;
    memset(cd, 0, sizeof(*cd));
    cd->pio_num = pio_num;
    cd->gpio_rx = -1;
    cd->gpio_tx = -1;
    if (g_started == cd)
        g_started = NULL;
}

static void put_bits(uint8_t *buf, uint32_t *pos, uint32_t value,
                     uint32_t nbits) {
    uint32_t i;
    for (i = 0; i < nbits; i++)
        bits_put(buf, (*pos)++, (uint8_t)((value >> (nbits - 1u - i)) & 1u));
}

// Build the on-wire buffer can2040_lite_pico_tx_frame() accepts: the stuffed
// SOF..CRC sequence followed by one recessive CRC delimiter.  Returns the
// number of bits written, or 0 on error.
static uint32_t pack_frame(const struct can2040_lite_msg *msg, uint8_t *bits,
                           uint32_t max_bits) {
    uint8_t un[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t pos = 0;
    uint32_t crc_start;
    uint32_t data_bytes;
    uint32_t stuffed;
    uint32_t i;
    uint16_t crc;
    int ext = (msg->id & CAN2040_LITE_ID_EFF) != 0;
    int rtr = (msg->id & CAN2040_LITE_ID_RTR) != 0;
    uint32_t id = msg->id & ~(CAN2040_LITE_ID_EFF | CAN2040_LITE_ID_RTR);

    if (max_bits < 1u)
        return 0;
    bits_clear(un, sizeof(un) * 8u);

    // SOF
    bits_put(un, pos++, 0);

    if (ext) {
        put_bits(un, &pos, (id >> 18) & 0x7ffu, 11);   // base ID
        bits_put(un, pos++, 1);                        // SRR
        bits_put(un, pos++, 1);                        // IDE
        put_bits(un, &pos, id & 0x3ffffu, 18);         // extended ID
        bits_put(un, pos++, rtr ? 1 : 0);              // RTR
        bits_put(un, pos++, 0);                        // r1
        bits_put(un, pos++, 0);                        // r0
    } else {
        put_bits(un, &pos, id & 0x7ffu, 11);           // ID
        bits_put(un, pos++, rtr ? 1 : 0);              // RTR
        bits_put(un, pos++, 0);                        // IDE
        bits_put(un, pos++, 0);                        // r0
    }
    put_bits(un, &pos, msg->dlc & 0x0fu, 4);           // DLC nibble

    data_bytes = rtr ? 0 : (msg->dlc > 8u ? 8u : msg->dlc);
    for (i = 0; i < data_bytes * 8u; i++) {
        uint8_t bit = (uint8_t)((msg->data[i / 8u] >>
                                 (7u - (i % 8u))) & 1u);
        bits_put(un, pos++, bit);
    }

    crc_start = pos;
    crc = crc_can15_bits(0, un, crc_start);
    for (i = 0; i < 15u; i++)
        bits_put(un, crc_start + i, (uint8_t)((crc >> (14u - i)) & 1u));
    pos = crc_start + 15u;

    // Reserve the final recessive CRC delimiter.
    stuffed = bit_stuff(un, pos, bits, max_bits - 1u);
    if (stuffed == CAN_BITS_ERROR || stuffed == 0)
        return 0;
    bits_put(bits, stuffed++, 1);                      // CRC delimiter
    return stuffed;
}

static int msg_is_valid(const struct can2040_lite_msg *msg) {
    uint32_t id;

    if (!msg || msg->dlc > 15u)
        return 0;

    id = msg->id & ~(CAN2040_LITE_ID_EFF | CAN2040_LITE_ID_RTR);
    if (msg->id & CAN2040_LITE_ID_EFF)
        return id <= 0x1fffffffu;
    return id <= 0x7ffu;
}

void can2040_lite_start(struct can2040_lite *cd, uint32_t sys_clock,
                        uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx) {
    uint32_t save;

    if (!cd)
        return;

    if (g_started && g_started != cd && g_started->started)
        return;

    cd->sys_clock = sys_clock;
    cd->bitrate = bitrate;
    cd->gpio_rx = gpio_rx;
    cd->gpio_tx = gpio_tx;

    if (can2040_lite_backend_start(cd, sys_clock, bitrate, gpio_rx, gpio_tx) < 0)
        return;

    if (gpio_tx < 0) {
        // Receive-only: drop messages queued by a previous TX session.
        cd->tx_head = 0;
        cd->tx_count = 0;
        cd->tx_active = 0;
    }

    cd->started = 1;
    g_started = cd;

    // Hand a queue left over from a previous session to the SM.
    save = can2040_lite_critical_enter();
    if (!cd->tx_active && cd->tx_count > 0) {
        uint32_t head = cd->tx_head;

        if (can2040_lite_backend_tx_frame(cd->tx_bits[head],
                                          cd->tx_nbits[head]) == 0)
            cd->tx_active = 1;
    }
    can2040_lite_critical_exit(save);
}

int can2040_lite_transmit(struct can2040_lite *cd,
                          const struct can2040_lite_msg *msg) {
    uint32_t slot;
    uint32_t nbits;
    uint32_t save;

    if (!cd || !cd->started || cd->gpio_tx < 0 || !msg_is_valid(msg))
        return -1;

    save = can2040_lite_critical_enter();
    if (cd->tx_count >= CAN2040_LITE_TX_QUEUE_LEN) {
        can2040_lite_critical_exit(save);
        return -1;
    }

    slot = (cd->tx_head + cd->tx_count) % CAN2040_LITE_TX_QUEUE_LEN;
    nbits = pack_frame(msg, cd->tx_bits[slot], CAN2040_LITE_MAX_FRAME_BITS);
    if (nbits == 0) {
        can2040_lite_critical_exit(save);
        return -1;
    }
    cd->tx_msg[slot] = *msg;
    cd->tx_nbits[slot] = nbits;
    cd->tx_count++;
    cd->stats.tx_attempt++;
    // Hand the head to the SM if it is idle.
    if (!cd->tx_active) {
        slot = cd->tx_head;
        if (can2040_lite_backend_tx_frame(cd->tx_bits[slot],
                                          cd->tx_nbits[slot]) == 0)
            cd->tx_active = 1;
    }
    can2040_lite_critical_exit(save);
    return 0;
}

int can2040_lite_check_transmit(struct can2040_lite *cd) {
    if (!cd || !cd->started || cd->gpio_tx < 0)
        return 0;
    return cd->tx_count < CAN2040_LITE_TX_QUEUE_LEN;
}

// The test module is empty unless CAN2040_LITE_TEST is defined.
#include "can2040_lite_test.inc"

void can2040_lite_get_statistics(struct can2040_lite *cd,
                                 struct can2040_lite_stats *stats) {
    if (!cd || !stats)
        return;
    *stats = cd->stats;
}

void can2040_lite_stop(struct can2040_lite *cd) {
    uint32_t save;

    if (!cd)
        return;
    // Silence the callback before the hardware is released.  The head stays
    // queued.  Clearing tx_active lets the next start() hand it to the SM
    // again; a message that already left the pin can therefore be sent twice.
    save = can2040_lite_critical_enter();
    cd->started = 0;
    cd->tx_active = 0;
    if (g_started == cd)
        g_started = NULL;
    can2040_lite_critical_exit(save);
    can2040_lite_backend_stop();
}
