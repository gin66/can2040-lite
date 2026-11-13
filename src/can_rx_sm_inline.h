// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_rx_sm_inline.h — RX parser.
//
// One FIFO word is two edges.  At 1 Mbit/s an alternating bitstream delivers
// that word every 2 us: 250 cycles at 125 MHz.  Each bulk field (ID, extended
// ID, DLC, data, CRC) has its own function.  A run that stays in the field
// does not call out.  SOF, ACK, and a run that crosses into the next field
// go through can_rx_other_edge.

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

// Shift first.  Bit 14 lands in bit 15, which is the test bit.  The uint16_t
// shift drops that bit on the next step, so the 15-bit mask is only needed
// when the CRC is stored.
static inline __attribute__((always_inline)) uint16_t can_rx_crc_bit(uint16_t crc, uint8_t bit) {
    crc = (uint16_t)(crc << 1);
    if ((bit == 0u) != ((crc & 0x8000u) == 0u))
        crc ^= 0x4599u;
    return crc;
}

// n identical bits, n >= 1.  Callers never pass 0.
static inline __attribute__((always_inline)) uint16_t can_rx_crc_run(uint16_t crc, uint8_t bit,
                                       uint8_t n) {
    do {
        crc = can_rx_crc_bit(crc, bit);
    } while (--n);
    return crc;
}

static inline __attribute__((always_inline)) can_rx_event_t can_rx_fail_sm(can_rx_sm_t *sm,
                                            can_rx_event_t why) {
    sm->state = ST_WAIT_MESSAGE;
    return why;
}

static inline __attribute__((always_inline)) uint8_t can_rx_take(uint8_t avail, uint8_t left) {
    return avail < left ? avail : left;
}

// Bulk fields.  Each returns how many bits of this run it consumed.
static __attribute__((noinline)) uint8_t can_rx_id_run(can_rx_sm_t *sm,
                                     can_rx_frame_t *f, uint8_t bit, uint8_t avail) {
    uint8_t left = (uint8_t)(ST_ID_LAST - sm->state + 1u);
    uint8_t n = can_rx_take(avail, left);
    uint32_t mask = bit ? ((1u << n) - 1u) : 0u;

    sm->crc_calc = can_rx_crc_run(sm->crc_calc, bit, n);
    f->id = (f->id << n) | mask;
    sm->state = (n == left) ? (uint8_t)ST_RTR_SRR : (uint8_t)(sm->state + n);
    return n;
}

static __attribute__((noinline)) uint8_t can_rx_ext_run(can_rx_sm_t *sm,
                                      can_rx_frame_t *f, uint8_t bit, uint8_t avail) {
    uint8_t idx = (uint8_t)(sm->state - ST_EXT_ID_BASE);
    uint8_t left = (uint8_t)(ST_EXT_ID_LAST - sm->state + 1u);
    uint8_t n = can_rx_take(avail, left);

    sm->crc_calc = can_rx_crc_run(sm->crc_calc, bit, n);
    if (bit) {
        // Wire order is MSB first: bit (17-idx) is the first bit of this run.
        uint8_t pos = (uint8_t)(18u - idx - n);

        f->id |= ((1u << n) - 1u) << pos;
    }
    sm->state = (n == left) ? (uint8_t)ST_EXT_RTR : (uint8_t)(sm->state + n);
    return n;
}

static __attribute__((noinline)) uint8_t can_rx_dlc_run(can_rx_sm_t *sm,
                                      can_rx_frame_t *f, uint8_t bit, uint8_t avail) {
    uint8_t left = (uint8_t)(ST_DLC_LAST - sm->state + 1u);
    uint8_t n = can_rx_take(avail, left);
    uint8_t mask = bit ? (uint8_t)((1u << n) - 1u) : 0u;

    sm->crc_calc = can_rx_crc_run(sm->crc_calc, bit, n);
    f->dlc = (uint8_t)((f->dlc << n) | mask);
    if (n == left) {
        f->dlc &= 0x0fu;
        if (f->dlc > 8u)
            f->dlc = 8u;
        sm->state = (f->dlc == 0u) ? (uint8_t)ST_CRC_BASE : (uint8_t)ST_DATA_BASE;
    } else {
        sm->state = (uint8_t)(sm->state + n);
    }
    return n;
}

static __attribute__((noinline)) uint8_t can_rx_data_run(can_rx_sm_t *sm,
                                       can_rx_frame_t *f, uint8_t bit, uint8_t avail) {
    uint8_t idx = (uint8_t)(sm->state - ST_DATA_BASE);
    uint8_t in_byte = (uint8_t)(idx & 7u);
    uint8_t room = (uint8_t)(8u - in_byte);
    uint8_t left = (uint8_t)((f->dlc << 3) - idx);
    uint8_t n = can_rx_take(avail, room);
    uint8_t mask;

    n = can_rx_take(n, left);
    mask = bit ? (uint8_t)((1u << n) - 1u) : 0u;
    sm->crc_calc = can_rx_crc_run(sm->crc_calc, bit, n);
    sm->shift = (uint8_t)((sm->shift << n) | mask);
    if ((uint8_t)(in_byte + n) == 8u)
        f->data[idx >> 3] = sm->shift;
    if (n == left)
        sm->state = ST_CRC_BASE;
    else
        sm->state = (uint8_t)(sm->state + n);
    return n;
}

// The next `n` CRC bits are one window of crc_calc.  Its MSB is the next wire bit.
static __attribute__((noinline)) can_rx_event_t can_rx_crc_run_field(can_rx_sm_t *sm,
                                                   can_rx_frame_t *f, uint8_t bit,
                                                   uint8_t avail, uint8_t *took) {
    uint8_t idx = (uint8_t)(sm->state - ST_CRC_BASE);
    uint8_t left = (uint8_t)(15u - idx);
    uint8_t n = can_rx_take(avail, left);
    uint16_t mask = (uint16_t)((1u << n) - 1u);
    uint16_t expect = (uint16_t)((sm->crc_calc >> (15u - idx - n)) & mask);

    *took = n;
    if (expect != (bit ? mask : 0u))
        return can_rx_fail_sm(sm, CAN_RX_CRC_ERROR);
    if (n == left) {
        f->crc = (uint16_t)(sm->crc_calc & 0x7fffu);
        sm->state = ST_WAIT_ACK;
        return CAN_RX_FRAME;
    }
    sm->state = (uint8_t)(sm->state + n);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_ctrl_bit(can_rx_sm_t *sm,
                                              can_rx_frame_t *f, uint8_t bit) {
    sm->crc_calc = can_rx_crc_bit(sm->crc_calc, bit);
    switch (sm->state) {
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

// Apply `bits` unstuffed bits of one level.  `bits` is already capped at 5.
static __attribute__((noinline)) can_rx_event_t can_rx_apply(can_rx_t *rx,
                                           can_rx_frame_t *f, uint8_t bit,
                                           uint8_t bits) {
    can_rx_sm_t *sm = &rx->sm;

    while (bits) {
        uint8_t s = sm->state;
        uint8_t took;
        can_rx_event_t ev;

        if (s == ST_WAIT_MESSAGE || s == ST_WAIT_ACK)
            return CAN_RX_NONE;
        if (s >= ST_DATA_BASE && s <= ST_DATA_LAST) {
            took = can_rx_data_run(sm, f, bit, bits);
        } else if (s >= ST_CRC_BASE && s <= ST_CRC_LAST) {
            ev = can_rx_crc_run_field(sm, f, bit, bits, &took);
            if (ev != CAN_RX_NONE)
                return ev;
        } else if (s >= ST_ID_BASE && s <= ST_ID_LAST) {
            took = can_rx_id_run(sm, f, bit, bits);
        } else if (s >= ST_EXT_ID_BASE && s <= ST_EXT_ID_LAST) {
            took = can_rx_ext_run(sm, f, bit, bits);
        } else if (s >= ST_DLC_BASE && s <= ST_DLC_LAST) {
            took = can_rx_dlc_run(sm, f, bit, bits);
        } else {
            ev = can_rx_ctrl_bit(sm, f, bit);
            if (ev != CAN_RX_NONE)
                return ev;
            took = 1u;
        }
        bits = (uint8_t)(bits - took);
    }
    return CAN_RX_NONE;
}

static inline __attribute__((always_inline)) uint8_t can_rx_run_bits(const can_rx_t *rx, uint16_t elapsed) {
    uint8_t n;

    if (elapsed <= rx->run_hi[1])
        n = 1u;
    else if (elapsed <= rx->run_hi[2])
        n = 2u;
    else if (elapsed <= rx->run_hi[3])
        n = 3u;
    else if (elapsed <= rx->run_hi[4])
        n = 4u;
    else if (elapsed <= rx->run_hi[5])
        n = 5u;
    else
        return 6u;
    if ((uint32_t)elapsed < rx->run_lo[n])
        return 0u;
    return n;
}

// Error path only.  Kept out of line so a fit edge does not pay its stores.
static __attribute__((noinline)) can_rx_event_t can_rx_fail_rx(can_rx_t *rx,
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

static __attribute__((noinline)) can_rx_event_t can_rx_feed_inline(can_rx_t *rx, uint8_t level,
                                                uint8_t n) {
    can_rx_frame_t *f = rx->write;
    can_rx_event_t ev = CAN_RX_NONE;
    uint8_t bits = n;
    uint8_t limit = 5u;

    if (n == 0u)
        return CAN_RX_BIT_ERROR;

    if (rx->drop_next) {
        bits--;
        limit = 4u;
        rx->drop_next = 0;
    }
    if (bits > limit)
        bits = limit;
    if (bits)
        ev = can_rx_apply(rx, f, level, bits);
    rx->drop_next = (uint8_t)(n == 5u);
    if (ev == CAN_RX_NONE && n > 5u)
        return CAN_RX_STUFF_ERROR;
    return ev;
}

static __attribute__((noinline)) can_rx_event_t can_rx_other_edge(can_rx_t *rx,
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
        if (level == 0u && (uint32_t)delta >= rx->ack_guard)
            can_rx_start_frame(rx, time16);
        return CAN_RX_NONE;
    }

    n = can_rx_run_bits(rx, delta);
    if (n == 0u)
        return can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);

    ev = can_rx_feed_inline(rx, old_level, n);
    if (ev == CAN_RX_FRAME) {
        ev = can_rx_publish(rx);
        if (level == 0u && (uint32_t)delta >= rx->ack_guard)
            can_rx_start_frame(rx, time16);
        return ev;
    }
    if (ev != CAN_RX_NONE)
        return can_rx_fail_rx(rx, ev);
    return CAN_RX_NONE;
}

// Publish a finished frame and, when this same dominant edge is already a
// new SOF, start the next one.  Only the CRC-complete edge reaches this.
static __attribute__((noinline)) can_rx_event_t can_rx_finish_frame(can_rx_t *rx,
                                                  uint16_t delta,
                                                  uint16_t time16) {
    can_rx_event_t ev = can_rx_publish(rx);

    if (rx->level == 0u && (uint32_t)delta >= rx->ack_guard)
        can_rx_start_frame(rx, time16);
    return ev;
}

// How many bits of this run enter the field.  n is the raw bin (1..6).
// A lone stuff bit (drop and n == 1) yields 0; the caller stores nothing else.
static inline __attribute__((always_inline)) uint8_t can_rx_cap(uint8_t n, uint8_t drop) {
    uint8_t bits = n;
    uint8_t limit = 5u;

    if (drop) {
        bits = (uint8_t)(n - 1u);
        limit = 4u;
    }
    if (bits > limit)
        bits = limit;
    return bits;
}

static __attribute__((noinline)) can_rx_event_t can_rx_data_edge(can_rx_t *rx,
                                               uint16_t time16,
                                               uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t n = can_rx_run_bits(rx, delta);
    uint8_t bit;
    uint8_t bits;
    uint8_t idx;
    uint8_t left;
    uint8_t inb;
    uint8_t room;
    uint8_t n1;
    uint8_t n2;
    uint8_t mask;
    can_rx_sm_t sm;
    can_rx_frame_t *f;

    if (n == 0u)
        return can_rx_other_edge(rx, time16, level);
    bit = (uint8_t)(rx->level & 1u);
    if (rx->drop_next && n == 1u) {
        rx->prev_time16 = time16;
        rx->level = level;
        rx->drop_next = 0;
        return CAN_RX_NONE;
    }
    bits = can_rx_cap(n, rx->drop_next);

    sm = rx->sm;
    f = rx->write;
    idx = (uint8_t)(sm.state - ST_DATA_BASE);
    left = (uint8_t)((f->dlc << 3) - idx);
    // Remainder spills into CRC.  Same machine as the cold edge, once per frame.
    if (bits > left)
        return can_rx_other_edge(rx, time16, level);

    inb = (uint8_t)(idx & 7u);
    room = (uint8_t)(8u - inb);
    n1 = bits <= room ? bits : room;
    n2 = (uint8_t)(bits - n1);
    mask = (uint8_t)((bit << n1) - bit);
    sm.crc_calc = can_rx_crc_run(sm.crc_calc, bit, bits);
    sm.shift = (uint8_t)((sm.shift << n1) | mask);
    if ((uint8_t)(inb + n1) == 8u)
        f->data[idx >> 3] = sm.shift;
    if (n2) {
        mask = (uint8_t)((bit << n2) - bit);
        sm.shift = (uint8_t)((sm.shift << n2) | mask);
    }
    sm.state = (bits == left) ? (uint8_t)ST_CRC_BASE : (uint8_t)(sm.state + bits);
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm = sm;
    rx->drop_next = (uint8_t)(n == 5u);
    if (n > 5u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_crc_edge(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t n = can_rx_run_bits(rx, delta);
    uint8_t bit;
    uint8_t bits;
    uint8_t idx;
    uint8_t left;
    uint8_t take;
    uint16_t mask;
    uint16_t expect;
    can_rx_sm_t sm;
    can_rx_frame_t *f;

    if (n == 0u)
        return can_rx_other_edge(rx, time16, level);
    bit = (uint8_t)(rx->level & 1u);
    if (rx->drop_next && n == 1u) {
        rx->prev_time16 = time16;
        rx->level = level;
        rx->drop_next = 0;
        return CAN_RX_NONE;
    }
    bits = can_rx_cap(n, rx->drop_next);

    sm = rx->sm;
    idx = (uint8_t)(sm.state - ST_CRC_BASE);
    left = (uint8_t)(15u - idx);
    take = bits < left ? bits : left;
    mask = (uint16_t)((1u << take) - 1u);
    expect = (uint16_t)((sm.crc_calc >> (15u - idx - take)) & mask);

    rx->prev_time16 = time16;
    rx->level = level;
    if (expect != (bit ? mask : 0u))
        return can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);

    if (take == left) {
        f = rx->write;
        f->crc = (uint16_t)(sm.crc_calc & 0x7fffu);
        sm.state = ST_WAIT_ACK;
        rx->sm = sm;
        rx->drop_next = (uint8_t)(n == 5u);
        return can_rx_finish_frame(rx, delta, time16);
    }
    sm.state = (uint8_t)(sm.state + take);
    rx->sm = sm;
    rx->drop_next = (uint8_t)(n == 5u);
    if (n > 5u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_id_edge(can_rx_t *rx,
                                             uint16_t time16,
                                             uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t n = can_rx_run_bits(rx, delta);
    uint8_t bit;
    uint8_t bits;
    uint8_t left;
    can_rx_sm_t sm;
    can_rx_frame_t *f;

    if (n == 0u)
        return can_rx_other_edge(rx, time16, level);
    bit = (uint8_t)(rx->level & 1u);
    if (rx->drop_next && n == 1u) {
        rx->prev_time16 = time16;
        rx->level = level;
        rx->drop_next = 0;
        return CAN_RX_NONE;
    }
    bits = can_rx_cap(n, rx->drop_next);

    sm = rx->sm;
    left = (uint8_t)(ST_ID_LAST - sm.state + 1u);
    if (bits > left)
        return can_rx_other_edge(rx, time16, level);

    f = rx->write;
    sm.crc_calc = can_rx_crc_run(sm.crc_calc, bit, bits);
    f->id = (f->id << bits) | (((uint32_t)bit << bits) - bit);
    sm.state = (bits == left) ? (uint8_t)ST_RTR_SRR : (uint8_t)(sm.state + bits);
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm = sm;
    rx->drop_next = (uint8_t)(n == 5u);
    if (n > 5u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_ext_edge(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t n = can_rx_run_bits(rx, delta);
    uint8_t bit;
    uint8_t bits;
    uint8_t idx;
    uint8_t left;
    can_rx_sm_t sm;
    can_rx_frame_t *f;

    if (n == 0u)
        return can_rx_other_edge(rx, time16, level);
    bit = (uint8_t)(rx->level & 1u);
    if (rx->drop_next && n == 1u) {
        rx->prev_time16 = time16;
        rx->level = level;
        rx->drop_next = 0;
        return CAN_RX_NONE;
    }
    bits = can_rx_cap(n, rx->drop_next);

    sm = rx->sm;
    idx = (uint8_t)(sm.state - ST_EXT_ID_BASE);
    left = (uint8_t)(ST_EXT_ID_LAST - sm.state + 1u);
    if (bits > left)
        return can_rx_other_edge(rx, time16, level);

    f = rx->write;
    sm.crc_calc = can_rx_crc_run(sm.crc_calc, bit, bits);
    if (bit) {
        uint8_t pos = (uint8_t)(18u - idx - bits);

        f->id |= ((1u << bits) - 1u) << pos;
    }
    sm.state = (bits == left) ? (uint8_t)ST_EXT_RTR : (uint8_t)(sm.state + bits);
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm = sm;
    rx->drop_next = (uint8_t)(n == 5u);
    if (n > 5u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_dlc_edge(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t n = can_rx_run_bits(rx, delta);
    uint8_t bit;
    uint8_t bits;
    uint8_t left;
    can_rx_sm_t sm;
    can_rx_frame_t *f;

    if (n == 0u)
        return can_rx_other_edge(rx, time16, level);
    bit = (uint8_t)(rx->level & 1u);
    if (rx->drop_next && n == 1u) {
        rx->prev_time16 = time16;
        rx->level = level;
        rx->drop_next = 0;
        return CAN_RX_NONE;
    }
    bits = can_rx_cap(n, rx->drop_next);

    sm = rx->sm;
    left = (uint8_t)(ST_DLC_LAST - sm.state + 1u);
    if (bits > left)
        return can_rx_other_edge(rx, time16, level);

    f = rx->write;
    sm.crc_calc = can_rx_crc_run(sm.crc_calc, bit, bits);
    f->dlc = (uint8_t)((f->dlc << bits) | (uint8_t)((bit << bits) - bit));
    if (bits == left) {
        f->dlc = (uint8_t)(f->dlc & 0x0fu);
        if (f->dlc > 8u)
            f->dlc = 8u;
        sm.state = (f->dlc == 0u) ? (uint8_t)ST_CRC_BASE : (uint8_t)ST_DATA_BASE;
    } else {
        sm.state = (uint8_t)(sm.state + bits);
    }
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm = sm;
    rx->drop_next = (uint8_t)(n == 5u);
    if (n > 5u)
        return can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
    return CAN_RX_NONE;
}

// The edge was only the stuff bit.  Three stores, no field update.
static __attribute__((noinline)) can_rx_event_t can_rx_drop_bit(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    rx->prev_time16 = time16;
    rx->level = level;
    rx->drop_next = 0;
    return CAN_RX_NONE;
}

// One data bit, already known to be in the 1-bit window and not a stuff bit.
// A single bit always fits in the current data byte and in the bytes still due.
static __attribute__((noinline)) can_rx_event_t can_rx_data_bit1(can_rx_t *rx,
                                               uint16_t time16,
                                               uint8_t level) {
    uint8_t bit = (uint8_t)(rx->level & 1u);
    uint8_t state = rx->sm.state;
    uint8_t shift = (uint8_t)((rx->sm.shift << 1) | bit);
    uint8_t idx = (uint8_t)(state - ST_DATA_BASE);
    can_rx_frame_t *f = rx->write;

    rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);
    rx->sm.shift = shift;
    rx->prev_time16 = time16;
    rx->level = level;
    if ((idx & 7u) == 7u)
        f->data[idx >> 3] = shift;
    if ((uint8_t)(idx + 1u) == (uint8_t)(f->dlc << 3))
        rx->sm.state = ST_CRC_BASE;
    else
        rx->sm.state = (uint8_t)(state + 1u);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_id_bit1(can_rx_t *rx,
                                             uint16_t time16,
                                             uint8_t level) {
    uint8_t bit = (uint8_t)(rx->level & 1u);
    uint8_t state = rx->sm.state;
    can_rx_frame_t *f = rx->write;

    f->id = (f->id << 1) | bit;
    rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm.state = (state == ST_ID_LAST) ? (uint8_t)ST_RTR_SRR
                                         : (uint8_t)(state + 1u);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_ext_bit1(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    uint8_t bit = (uint8_t)(rx->level & 1u);
    uint8_t state = rx->sm.state;
    uint8_t idx = (uint8_t)(state - ST_EXT_ID_BASE);
    can_rx_frame_t *f = rx->write;

    f->id |= (uint32_t)bit << (17u - idx);
    rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);
    rx->prev_time16 = time16;
    rx->level = level;
    rx->sm.state = (state == ST_EXT_ID_LAST) ? (uint8_t)ST_EXT_RTR
                                             : (uint8_t)(state + 1u);
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_dlc_bit1(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level) {
    uint8_t bit = (uint8_t)(rx->level & 1u);
    uint8_t state = rx->sm.state;
    can_rx_frame_t *f = rx->write;

    f->dlc = (uint8_t)((f->dlc << 1) | bit);
    rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);
    rx->prev_time16 = time16;
    rx->level = level;
    if (state == ST_DLC_LAST) {
        f->dlc &= 0x0fu;
        if (f->dlc > 8u)
            f->dlc = 8u;
        rx->sm.state = (f->dlc == 0u) ? (uint8_t)ST_CRC_BASE : (uint8_t)ST_DATA_BASE;
    } else {
        rx->sm.state = (uint8_t)(state + 1u);
    }
    return CAN_RX_NONE;
}

static __attribute__((noinline)) can_rx_event_t can_rx_crc_bit1(can_rx_t *rx,
                                              uint16_t time16,
                                              uint8_t level,
                                              uint16_t delta) {
    uint8_t bit = (uint8_t)(rx->level & 1u);
    uint8_t state = rx->sm.state;
    uint16_t crc = rx->sm.crc_calc;
    uint8_t idx = (uint8_t)(state - ST_CRC_BASE);
    uint8_t expect = (uint8_t)((crc >> (14u - idx)) & 1u);

    rx->prev_time16 = time16;
    rx->level = level;
    if (expect != bit)
        return can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
    if (state == ST_CRC_LAST) {
        rx->write->crc = (uint16_t)(crc & 0x7fffu);
        rx->sm.state = ST_WAIT_ACK;
        return can_rx_finish_frame(rx, delta, time16);
    }
    rx->sm.state = (uint8_t)(state + 1u);
    return CAN_RX_NONE;
}

// One edge.  Inlined into the FIFO drain: a range check and one call.
// A 1-bit run with no stuff bit is the word that arrives every 2 us.
static inline __attribute__((always_inline)) can_rx_event_t can_rx_edge_inline(can_rx_t *rx,
                                                             uint16_t time16,
                                                             uint8_t level) {
    uint8_t s = rx->sm.state;
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);
    uint8_t in1 = (uint8_t)(delta <= rx->run_hi[1] &&
                            (uint32_t)delta >= rx->run_lo[1]);
    uint8_t drop = rx->drop_next;

    if ((unsigned)(s - ST_DATA_BASE) <= (unsigned)(ST_DATA_LAST - ST_DATA_BASE)) {
        if (in1 && !drop)
            return can_rx_data_bit1(rx, time16, level);
        if (in1)
            return can_rx_drop_bit(rx, time16, level);
        return can_rx_data_edge(rx, time16, level);
    }
    if ((unsigned)(s - ST_CRC_BASE) <= (unsigned)(ST_CRC_LAST - ST_CRC_BASE)) {
        if (in1 && !drop)
            return can_rx_crc_bit1(rx, time16, level, delta);
        if (in1)
            return can_rx_drop_bit(rx, time16, level);
        return can_rx_crc_edge(rx, time16, level);
    }
    if ((unsigned)(s - ST_ID_BASE) <= (unsigned)(ST_ID_LAST - ST_ID_BASE)) {
        if (in1 && !drop)
            return can_rx_id_bit1(rx, time16, level);
        if (in1)
            return can_rx_drop_bit(rx, time16, level);
        return can_rx_id_edge(rx, time16, level);
    }
    if ((unsigned)(s - ST_EXT_ID_BASE) <= (unsigned)(ST_EXT_ID_LAST - ST_EXT_ID_BASE)) {
        if (in1 && !drop)
            return can_rx_ext_bit1(rx, time16, level);
        if (in1)
            return can_rx_drop_bit(rx, time16, level);
        return can_rx_ext_edge(rx, time16, level);
    }
    if ((unsigned)(s - ST_DLC_BASE) <= (unsigned)(ST_DLC_LAST - ST_DLC_BASE)) {
        if (in1 && !drop)
            return can_rx_dlc_bit1(rx, time16, level);
        if (in1)
            return can_rx_drop_bit(rx, time16, level);
        return can_rx_dlc_edge(rx, time16, level);
    }
    return can_rx_other_edge(rx, time16, level);
}

// One RX FIFO word is the 1->0 timestamp then the 0->1 timestamp.
static inline __attribute__((always_inline)) can_rx_event_t can_rx_pair_inline(can_rx_t *rx, uint32_t word) {
    can_rx_event_t e0 =
        can_rx_edge_inline(rx, (uint16_t)(word >> 16), 0u);
    can_rx_event_t e1 = can_rx_edge_inline(rx, (uint16_t)word, 1u);

    return (e1 != CAN_RX_NONE) ? e1 : e0;
}

#endif // CAN_RX_SM_INLINE_H
