// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite.c — Portable API skeleton.
//
// This file has no pico-sdk dependency, so the host tests compile and exercise
// the same API that the target firmware uses.  The PIO/DMA backend is attached
// by the target build (Phase 4/5); until then the API validates input and
// tracks statistics.

#include <stddef.h>
#include <string.h>

#include "can2040_lite.h"

void can2040_lite_setup(struct can2040_lite *cd, uint32_t pio_num) {
    if (!cd)
        return;
    memset(cd, 0, sizeof(*cd));
    cd->pio_num = pio_num;
    cd->gpio_rx = -1;
    cd->gpio_tx = -1;
}

void can2040_lite_callback_config(struct can2040_lite *cd,
                                  can2040_lite_cb cb) {
    if (!cd)
        return;
    cd->cb = cb;
}

void can2040_lite_start(struct can2040_lite *cd, uint32_t sys_clock,
                        uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx) {
    if (!cd)
        return;
    cd->sys_clock = sys_clock;
    cd->bitrate = bitrate;
    cd->gpio_rx = gpio_rx;
    cd->gpio_tx = gpio_tx;
    cd->started = 1;
    // Backend start hook: configure PIO0 SM0 (rx) and SM1 (tx), DMA, IRQs.
    // The rx + tx programs must fit in <= 29 instruction words.
}

static int msg_is_valid(const struct can2040_lite_msg *msg) {
    uint32_t id;

    if (!msg || msg->dlc > 8)
        return 0;

    id = msg->id & ~(CAN2040_LITE_ID_EFF | CAN2040_LITE_ID_RTR);
    if (msg->id & CAN2040_LITE_ID_EFF)
        return id <= 0x1fffffffu;
    return id <= 0x7ffu;
}

int can2040_lite_transmit(struct can2040_lite *cd,
                          const struct can2040_lite_msg *msg) {
    if (!cd || !cd->started || cd->gpio_tx < 0 || !msg_is_valid(msg))
        return -1;

    cd->stats.tx_attempt++;
    // Backend transmit hook: bit-stuff, queue to the TX SM, and report the
    // result through the callback (TX/TX_NACK/TX_ERROR).
    return 0;
}

int can2040_lite_check_transmit(struct can2040_lite *cd) {
    if (!cd || !cd->started || cd->gpio_tx < 0)
        return 0;
    // Backend hook: return non-zero when the TX queue has room.
    return 1;
}

void can2040_lite_pio_irq_handler(struct can2040_lite *cd) {
    if (!cd)
        return;
    // Backend hook: drain RX FIFO / DMA, detect frame end, decode, notify.
}

void can2040_lite_get_statistics(struct can2040_lite *cd,
                                 struct can2040_lite_stats *stats) {
    if (!cd || !stats)
        return;
    *stats = cd->stats;
}

void can2040_lite_stop(struct can2040_lite *cd) {
    if (!cd)
        return;
    // Backend hook: disable PIO SMs, DMA and IRQs, release the programs.
    cd->started = 0;
}
