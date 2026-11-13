// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_rx_sm_inline.h — hot RX parser, static inline __attribute__((always_inline)) for the ISR.
//
// This header is included by can_rx_sm.c (which provides testable wrappers)
// and by the Pico backend ISR.  The edge/feed/state-machine path is fully
// inlined; there is no per-edge or per-word subroutine call.

#ifndef CAN_RX_SM_INLINE_H
#define CAN_RX_SM_INLINE_H

#include <stdint.h>

#include "can_rx_sm.h"

enum {
    ST_WAIT_MESSAGE = 0,
    ST_SOF = 1,
    ST_ID_BASE = 2,
    ST_ID_LAST = 12,
    ST_RTR_SRR = 13,
    ST_IDE = 14,
    ST_STD_R0 = 15,
    ST_EXT_ID_BASE = 16,
    ST_EXT_ID_LAST = 33,
    ST_EXT_RTR = 34,
    ST_EXT_R1 = 35,
    ST_EXT_R0 = 36,
    ST_DLC_BASE = 37,
    ST_DLC_LAST = 40,
    ST_DATA_BASE = 41,
    ST_DATA_LAST = 104,
    ST_CRC_BASE = 105,
    ST_CRC_LAST = 119,
    ST_WAIT_ACK = 120
};

#define CAN_RX_ACK_GUARD_BITS 9u

static inline __attribute__((always_inline)) uint16_t can_rx_crc_add(uint16_t crc, uint8_t bit) {
    uint16_t mix = (uint16_t)(((crc >> 14) ^ bit) & 1u);

    crc = (uint16_t)((crc << 1) & 0x7fffu);
    if (mix)
        crc ^= 0x4599u;
    return crc;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_fail_sm(can_rx_sm_t *sm,
                                            can_rx_event_t why) {
    sm->state = ST_WAIT_MESSAGE;
    return why;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_sm_bit(can_rx_sm_t *sm,
                                           can_rx_frame_t *f, uint8_t bit) {
    uint8_t s = sm->state;

    if (s == ST_WAIT_MESSAGE || s == ST_WAIT_ACK)
        return CAN_RX_NONE;

    if (s < ST_CRC_BASE)
        sm->crc_calc = can_rx_crc_add(sm->crc_calc, bit);

    if (s >= ST_DATA_BASE && s <= ST_DATA_LAST) {
        uint8_t idx = (uint8_t)(s - ST_DATA_BASE);

        sm->shift = (uint8_t)((sm->shift << 1) | bit);
        if ((idx & 7u) == 7u)
            f->data[idx >> 3] = sm->shift;
        if ((uint8_t)(idx + 1u) == (uint8_t)(f->dlc << 3))
            sm->state = ST_CRC_BASE;
        else
            sm->state = (uint8_t)(s + 1u);
        return CAN_RX_NONE;
    }

    if (s >= ST_CRC_BASE && s <= ST_CRC_LAST) {
        uint8_t idx = (uint8_t)(s - ST_CRC_BASE);
        uint8_t expected = (uint8_t)((sm->crc_calc >> (14u - idx)) & 1u);

        if (bit != expected)
            return can_rx_fail_sm(sm, CAN_RX_CRC_ERROR);
        if (idx == 14u) {
            f->crc = sm->crc_calc;
            sm->state = ST_WAIT_ACK;
            return CAN_RX_FRAME;
        }
        sm->state = (uint8_t)(s + 1u);
        return CAN_RX_NONE;
    }

    if (s >= ST_ID_BASE && s <= ST_ID_LAST) {
        f->id = (f->id << 1) | bit;
        sm->state = (s == ST_ID_LAST) ? (uint8_t)ST_RTR_SRR : (uint8_t)(s + 1u);
        return CAN_RX_NONE;
    }

    if (s >= ST_EXT_ID_BASE && s <= ST_EXT_ID_LAST) {
        uint8_t idx = (uint8_t)(s - ST_EXT_ID_BASE);

        f->id |= (uint32_t)bit << (17u - idx);
        sm->state = (s == ST_EXT_ID_LAST) ? (uint8_t)ST_EXT_RTR : (uint8_t)(s + 1u);
        return CAN_RX_NONE;
    }

    if (s >= ST_DLC_BASE && s <= ST_DLC_LAST) {
        f->dlc = (uint8_t)((f->dlc << 1) | bit);
        if (s == ST_DLC_LAST) {
            f->dlc &= 0x0fu;
            if (f->dlc > 8u)
                f->dlc = 8u;
            sm->state = (f->dlc == 0u) ? ST_CRC_BASE : ST_DATA_BASE;
        } else {
            sm->state = (uint8_t)(s + 1u);
        }
        return CAN_RX_NONE;
    }

    switch (s) {
    case ST_SOF:
        if (bit)
            return can_rx_fail_sm(sm, CAN_RX_BIT_ERROR);
        sm->state = ST_ID_BASE;
        return CAN_RX_NONE;
    case ST_RTR_SRR:
        f->flags = (uint8_t)(bit << 1);
        sm->state = ST_IDE;
        return CAN_RX_NONE;
    case ST_IDE:
        if (bit) {
            if (!(f->flags & 2u))
                return can_rx_fail_sm(sm, CAN_RX_BIT_ERROR);
            f->flags = 1u;
            f->id = (f->id & 0x7ffu) << 18;
            sm->state = ST_EXT_ID_BASE;
        } else {
            if (f->flags & 2u)
                return can_rx_fail_sm(sm, CAN_RX_BIT_ERROR);
            f->flags = 0u;
            f->id &= 0x7ffu;
            sm->state = ST_STD_R0;
        }
        return CAN_RX_NONE;
    case ST_STD_R0:
        sm->state = ST_DLC_BASE;
        return CAN_RX_NONE;
    case ST_EXT_RTR:
        if (bit)
            return can_rx_fail_sm(sm, CAN_RX_BIT_ERROR);
        sm->state = ST_EXT_R1;
        return CAN_RX_NONE;
    case ST_EXT_R1:
        sm->state = ST_EXT_R0;
        return CAN_RX_NONE;
    case ST_EXT_R0:
        sm->state = ST_DLC_BASE;
        return CAN_RX_NONE;
    default:
        return can_rx_fail_sm(sm, CAN_RX_BIT_ERROR);
    }
}

static inline __attribute__((always_inline)) uint8_t can_rx_run_bits(const can_rx_t *rx, uint16_t elapsed) {
    uint8_t n = 1u;

    if (elapsed > rx->run_hi[1])
        n = 2u;
    if (elapsed > rx->run_hi[2])
        n = 3u;
    if (elapsed > rx->run_hi[3])
        n = 4u;
    if (elapsed > rx->run_hi[4])
        n = 5u;
    if (elapsed > rx->run_hi[5])
        return 6u;

    if ((uint32_t)elapsed <
        2u * (uint32_t)n * rx->ticks_per_bit - rx->run_hi[n])
        return 0u;
    return n;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_fail_rx(can_rx_t *rx,
                                            can_rx_event_t why) {
    rx->sm.state = ST_WAIT_MESSAGE;
    rx->drop_next = 0;
    if (why == CAN_RX_CRC_ERROR)
        rx->crc_errors++;
    else if (why == CAN_RX_STUFF_ERROR)
        rx->stuff_errors++;
    else
        rx->bit_errors++;
    return why;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_publish(can_rx_t *rx) {
    uint8_t next = (uint8_t)((rx->rx_idx + 1u) & CAN_RX_SLOT_MASK);

    if (next == rx->tx_idx) {
        rx->overflow++;
        return CAN_RX_OVERFLOW;
    }
    __sync_synchronize();
    rx->rx_idx = next;
    rx->write = &rx->frames[next];
    return CAN_RX_FRAME;
}

static inline __attribute__((always_inline)) void can_rx_start_frame(can_rx_t *rx, uint16_t time16) {
    rx->sm.state = ST_SOF;
    rx->sm.crc_calc = 0;
    rx->drop_next = 0;
    rx->write = &rx->frames[rx->rx_idx];
    rx->prev_time16 = time16;
    rx->level = 0;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_feed_inline(can_rx_t *rx, uint8_t level,
                                                uint8_t n) {
    can_rx_frame_t *f = rx->write;
    can_rx_event_t ev = CAN_RX_NONE;
    uint8_t bits = n;
    uint8_t limit = 5u;
    uint8_t i;

    if (n == 0u)
        return CAN_RX_BIT_ERROR;

    if (rx->drop_next) {
        bits--;
        limit = 4u;
        rx->drop_next = 0;
    }
    if (bits > limit)
        bits = limit;
    for (i = 0; i < bits; i++) {
        ev = can_rx_sm_bit(&rx->sm, f, level);
        if (ev != CAN_RX_NONE)
            break;
    }
    rx->drop_next = (uint8_t)(n == 5u);
    if (ev == CAN_RX_NONE && n > 5u)
        return CAN_RX_STUFF_ERROR;
    return ev;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_edge_inline(can_rx_t *rx,
                                                uint16_t time16,
                                                uint8_t level) {
    uint16_t prev = rx->prev_time16;
    uint16_t delta = (uint16_t)(prev - time16);
    uint8_t old_level = rx->level;
    uint8_t s = rx->sm.state;
    can_rx_event_t ev;
    uint8_t n;

    rx->prev_time16 = time16;
    rx->level = level;

    if (s == ST_WAIT_MESSAGE) {
        if (level == 0u)
            can_rx_start_frame(rx, time16);
        return CAN_RX_NONE;
    }

    if (s == ST_WAIT_ACK) {
        if (level == 0u &&
            (uint32_t)delta >=
                (uint32_t)CAN_RX_ACK_GUARD_BITS * rx->ticks_per_bit)
            can_rx_start_frame(rx, time16);
        return CAN_RX_NONE;
    }

    n = can_rx_run_bits(rx, delta);
    if (n == 0u)
        return can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);

    ev = can_rx_feed_inline(rx, old_level, n);
    if (ev == CAN_RX_FRAME) {
        ev = can_rx_publish(rx);
        if (level == 0u &&
            (uint32_t)delta >=
                (uint32_t)CAN_RX_ACK_GUARD_BITS * rx->ticks_per_bit)
            can_rx_start_frame(rx, time16);
        return ev;
    }
    if (ev != CAN_RX_NONE)
        return can_rx_fail_rx(rx, ev);
    return CAN_RX_NONE;
}

// One RX FIFO word contains fall then rise.  Both edges are inlined here.
static inline __attribute__((always_inline)) can_rx_event_t can_rx_pair_inline(can_rx_t *rx, uint32_t word) {
    can_rx_event_t e0 =
        can_rx_edge_inline(rx, (uint16_t)(word >> 16), 0u);
    can_rx_event_t e1 = can_rx_edge_inline(rx, (uint16_t)word, 1u);

    return (e1 != CAN_RX_NONE) ? e1 : e0;
}

#endif // CAN_RX_SM_INLINE_H
