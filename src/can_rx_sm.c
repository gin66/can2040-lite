// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_rx_sm.c — non-hot helpers and host-test wrappers.
//
// The hot edge/feed/state-machine path lives in can_rx_sm_inline.h and is
// compiled directly into the Pico ISR.

#include <string.h>

#include "can2040_lite_config.h"
#include "can_rx_sm_inline.h"

static void run_windows_init(can_rx_t *rx) {
    uint32_t t = rx->ticks_per_bit;
    uint8_t n;

    rx->run_hi[0] = 0;
    for (n = 1u; n <= 5u; n++) {
        uint32_t nominal = (uint32_t)n * t;
        uint32_t tol = nominal / 200u + t / 8u + 1u;
        uint32_t hi = nominal + tol;

        rx->run_hi[n] = (uint16_t)(hi > 0xffffu ? 0xffffu : hi);
        // Same lower edge the old per-edge expression used: 2*n*T - hi.
        rx->run_lo[n] = 2u * (uint32_t)n * t - rx->run_hi[n];
    }
}

void can_rx_edge(can_rx_t *rx, uint16_t time16, uint8_t level) {
    can_rx_edge_inline(rx, time16, level);
}

void can_rx_pair(can_rx_t *rx, uint32_t word) {
    can_rx_pair_inline(rx, word);
}

void can_rx_reset(can_rx_t *rx) {
    rx->i = CAN_BIT_WAIT_FRAME;
    rx->drop_next = 0;
    rx->rx_idx = 0;
    rx->tx_idx = 0;
    rx->write = &rx->frames[0];
}

void can_rx_init(can_rx_t *rx, uint32_t ticks_per_bit) {
    memset(rx, 0, sizeof(*rx));
#ifdef CAN_BIT_TICKS
    ticks_per_bit = CAN_BIT_TICKS;
#endif
    rx->ticks_per_bit = (uint16_t)ticks_per_bit;
    run_windows_init(rx);
    rx->ack_guard = (uint32_t)CAN_RX_ACK_GUARD_BITS * rx->ticks_per_bit;
    can_rx_reset(rx);
}
