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

uint8_t can_rx_crc_tail_bits(const can_rx_t *rx) {
    uint8_t s = rx->sm.state;
    uint8_t rem;
    uint16_t mask;

    if (s < ST_CRC_BASE || s > ST_CRC_LAST || rx->level != 1u)
        return 0;
    rem = (uint8_t)(ST_CRC_LAST - s + 1u);
    mask = (uint16_t)((1u << rem) - 1u);
    if ((rx->sm.crc_calc & mask) != mask)
        return 0;
    return rem;
}

can_rx_event_t can_rx_flush(can_rx_t *rx) {
    can_rx_event_t ev;
    uint8_t s = rx->sm.state;

    if (s == ST_WAIT_MESSAGE || s == ST_WAIT_ACK)
        return CAN_RX_NONE;

    // A 5-run in the CRC field is followed by a stuff bit, so all-same CRC
    // data always produces an edge.  The only edge-less ending is a final run
    // of 1..4 recessive CRC bits followed by the recessive delimiter and a
    // recessive ACK.  Anything else quiet for >5 bits is a stuff violation.
    if (s < ST_CRC_BASE || s > ST_CRC_LAST || rx->level != 1u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);

    ev = can_rx_feed_inline(rx, 1u, 5u);
    if (ev == CAN_RX_FRAME)
        return can_rx_publish(rx);
    if (ev == CAN_RX_NONE)
        ev = CAN_RX_STUFF_ERROR;
    return can_rx_fail_rx(rx, ev);
}

can_rx_event_t can_rx_feed(can_rx_t *rx, uint8_t level, uint8_t n) {
    return can_rx_feed_inline(rx, level, n);
}

can_rx_event_t can_rx_edge(can_rx_t *rx, uint16_t time16, uint8_t level) {
    return can_rx_edge_inline(rx, time16, level);
}

can_rx_event_t can_rx_pair(can_rx_t *rx, uint32_t word) {
    return can_rx_pair_inline(rx, word);
}

void can_rx_reset(can_rx_t *rx) {
    rx->sm.state = ST_WAIT_MESSAGE;
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
