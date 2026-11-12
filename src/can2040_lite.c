// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite.c — Portable public API, transmit queue, and frame packer.
//
// This file has no pico-sdk dependency, so the host tests compile and exercise
// the same API that the target firmware uses.  Hardware operations go through
// the ops table installed by the target backend; with no table installed the
// driver still starts and queues, which is how the host tests run.

#include <stddef.h>
#include <string.h>

#include "can2040_lite.h"

// One live instance.  A second start() on another object while the first is
// started leaves the hardware alone.
static struct can2040_lite *g_started;
static const struct can2040_lite_hw *g_hw;

// Critical section around queue and statistics updates.  The target backend
// overrides these with save_and_disable_interrupts()/restore_interrupts().
// The host tests call the handler directly, so the empty section is enough.
__attribute__((weak)) uint32_t can2040_lite_critical_enter(void) {
    return 0;
}

__attribute__((weak)) void can2040_lite_critical_exit(uint32_t token) {
    (void)token;
}

// Weak hook: the Pico backend registers its ops table here.
__attribute__((weak)) void can2040_lite_hw_attach(void) {
}

void can2040_lite_set_hw(const struct can2040_lite_hw *hw) {
    g_hw = hw;
}

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

void can2040_lite_callback_config(struct can2040_lite *cd,
                                  can2040_lite_cb cb) {
    if (!cd)
        return;
    cd->cb = cb;
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

static void tx_advance_locked(struct can2040_lite *cd) {
    cd->tx_head = (cd->tx_head + 1u) % CAN2040_LITE_TX_QUEUE_LEN;
    cd->tx_count--;
    cd->tx_active = 0;
}

// Hand the queue head to the hardware when the state machine is idle.
static void tx_kick(struct can2040_lite *cd) {
    uint32_t slot;
    uint32_t save;

    if (!cd->started || cd->gpio_tx < 0 || !g_hw || !g_hw->tx_frame)
        return;

    save = can2040_lite_critical_enter();
    if (!cd->tx_active && cd->tx_count > 0) {
        slot = cd->tx_head;
        if (g_hw->tx_frame(cd->tx_bits[slot], cd->tx_nbits[slot]) == 0)
            cd->tx_active = 1;
    }
    can2040_lite_critical_exit(save);
}

void can2040_lite_start(struct can2040_lite *cd, uint32_t sys_clock,
                        uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx) {
    if (!cd)
        return;

    can2040_lite_hw_attach();

    if (g_started && g_started != cd && g_started->started)
        return;

    cd->sys_clock = sys_clock;
    cd->bitrate = bitrate;
    cd->gpio_rx = gpio_rx;
    cd->gpio_tx = gpio_tx;
    cd->started = 1;
    g_started = cd;

    if (g_hw && g_hw->start)
        g_hw->start(cd, sys_clock, bitrate, gpio_rx, gpio_tx);

    tx_kick(cd);
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
    can2040_lite_critical_exit(save);

    tx_kick(cd);
    return 0;
}

int can2040_lite_check_transmit(struct can2040_lite *cd) {
    if (!cd || !cd->started || cd->gpio_tx < 0)
        return 0;
    return cd->tx_count < CAN2040_LITE_TX_QUEUE_LEN;
}

static void msg_from_parsed(const can_parsed_t *p,
                            struct can2040_lite_msg *msg) {
    memset(msg, 0, sizeof(*msg));
    msg->id = p->id;
    if (p->is_extended)
        msg->id |= CAN2040_LITE_ID_EFF;
    if (p->is_rtr)
        msg->id |= CAN2040_LITE_ID_RTR;
    msg->dlc = p->dlc;
    memcpy(msg->data, p->data, sizeof(msg->data));
}

static int tx_matches(const struct can2040_lite *cd, const can_parsed_t *p) {
    uint32_t id;

    if (!cd->tx_active || cd->tx_count == 0)
        return 0;
    id = p->id;
    if (p->is_extended)
        id |= CAN2040_LITE_ID_EFF;
    if (p->is_rtr)
        id |= CAN2040_LITE_ID_RTR;
    return id == cd->tx_msg[cd->tx_head].id;
}

static void deliver(struct can2040_lite *cd, uint32_t notify,
                    const struct can2040_lite_msg *msg) {
    if (cd->cb)
        cd->cb(cd, notify, (struct can2040_lite_msg *)msg);
}

void can2040_lite_pio_irq_handler(struct can2040_lite *cd) {
    struct can2040_lite_event ev;
    struct can2040_lite_msg pending;
    int have_pending = 0;

    if (!cd || !cd->started || !g_hw || !g_hw->poll)
        return;

    while (g_hw->poll(&ev)) {
        if (ev.kind == 1) {
            // TX shift finished.  Only an error completes the head here; a
            // clean finish waits for the matching RX parse and its ACK bit.
            if (ev.tx_error) {
                uint32_t save = can2040_lite_critical_enter();
                int complete = cd->tx_active;
                if (complete) {
                    cd->tx_done_msg = cd->tx_msg[cd->tx_head];
                    tx_advance_locked(cd);
                    cd->stats.tx_error++;
                    cd->stats.parse_error++;
                }
                can2040_lite_critical_exit(save);
                if (complete)
                    tx_kick(cd);
            }
            continue;
        }

        if (ev.kind != 2)
            continue;

        if (ev.rx.validity == VALID_FRAME) {
            int done_tx = 0;
            int ack = 0;
            uint32_t save = can2040_lite_critical_enter();

            if (tx_matches(cd, &ev.rx)) {
                done_tx = 1;
                ack = ev.rx.ack;
                cd->tx_done_msg = cd->tx_msg[cd->tx_head];
                tx_advance_locked(cd);
                if (ack)
                    cd->stats.tx_total++;
                else {
                    cd->stats.tx_nack++;
                    cd->stats.parse_error++;
                }
            } else {
                cd->stats.rx_total++;
            }
            can2040_lite_critical_exit(save);

            if (done_tx) {
                if (ack)
                    deliver(cd, CAN2040_LITE_NOTIFY_TX, &cd->tx_done_msg);
                tx_kick(cd);
            } else {
                if (have_pending) {
                    // The previous result was overwritten before its callback
                    // could run.  Report the loss and keep the newer frame.
                    struct can2040_lite_msg zero;
                    memset(&zero, 0, sizeof(zero));
                    save = can2040_lite_critical_enter();
                    cd->stats.rx_overflow++;
                    can2040_lite_critical_exit(save);
                    deliver(cd, CAN2040_LITE_NOTIFY_ERROR, &zero);
                }
                msg_from_parsed(&ev.rx, &pending);
                have_pending = 1;
            }
            continue;
        }

        {
            uint32_t save = can2040_lite_critical_enter();
            cd->stats.parse_error++;
            if (ev.rx.validity == INVALID_CRC)
                cd->stats.crc_errors++;
            else if (ev.rx.validity == INVALID_STUFF)
                cd->stats.stuff_errors++;
            else
                cd->stats.bit_errors++;
            can2040_lite_critical_exit(save);
        }
    }

    if (have_pending)
        deliver(cd, CAN2040_LITE_NOTIFY_RX, &pending);
}

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
    if (g_hw && g_hw->stop)
        g_hw->stop(cd);
}
