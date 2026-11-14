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

#include "can_bit_layout.h"
#include "can_rx_sm.h"

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
// Error path only.  Kept out of line so a fit edge does not pay its stores.
static __attribute__((noinline)) void can_rx_fail_rx(can_rx_t *rx,
                                            can_rx_event_t why) {
    rx->i = CAN_BIT_WAIT_FRAME;
    rx->drop_next = 0;
    if (why == CAN_RX_CRC_ERROR)
        rx->crc_errors++;
    else if (why == CAN_RX_STUFF_ERROR)
        rx->stuff_errors++;
    else
        rx->bit_errors++;
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

// Advance one unstuffed bit through the shared 11/29-bit layout.  Index 0 is
// the first base-ID bit; IDE at 12 selects whether the extended-only bits are
// skipped; the unused data slots are skipped after the last used byte.
static inline __attribute__((always_inline)) void can_rx_advance(can_rx_t *rx,
                                                                uint8_t bit) {
    can_rx_frame_t *f = rx->write;
    uint8_t i = rx->i;
    uint8_t jump = 0;
    uint8_t shift;

    if (i != CAN_BIT_WAIT_FRAME)
        rx->shift = (uint8_t)((rx->shift << 1) | bit);

    if (i < CAN_BIT_CRC_FIRST)
        rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);

    // After eight CRC bits the shift holds the first eight expected bits.
    switch (i) {
    case CAN_BIT_ID_BASE_FIRST + 7u:
        f->id_part1 = rx->shift;
        break;
    case CAN_BIT_IDE:
        shift = rx->shift;
        f->id_part2 = shift;
        if ((shift & 1u) == 0u) {
            if (shift & 2u) {           // standard RTR
                can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
                return;
            }
            jump = CAN_IDE_STD_SKIP;
        }
        break;
    case CAN_BIT_RTR_EXT:
        if (rx->shift & 1u) {           // extended RTR
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
            return;
        }
        break;
    case CAN_BIT_ID_EXT_FIRST + 7u:
        f->id_part3 = rx->shift;
        break;
    case CAN_BIT_ID_EXT_LAST - 2u:
        f->id_part4 = rx->shift;
        break;
    case CAN_BIT_ID_EXT_LAST:
        f->id_part5 = rx->shift;
        break;
    case CAN_BIT_DLC_LAST:
        shift = rx->shift & 0x0fu;
        if (shift > CAN_DLC_MAX)
            shift = CAN_DLC_MAX;
        f->dlc = shift;
        jump = (8 - shift) << 3;
        break;
    case CAN_BIT_CRC_FIRST + 7u:
        if (rx->shift != (uint8_t)(rx->sm.crc_calc >> 7)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
        break;
    case CAN_BIT_CRC_LAST:
        if ((rx->shift & 0x7fu) != (rx->sm.crc_calc & 0x7fu)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
        break;
    case CAN_BIT_ACK_SLOT:
        f->ack = (uint8_t)(bit == 0u);
        if (bit == 0u)
            can_rx_publish(rx);
        else
            rx->ack_errors++;
        rx->drop_next = 0;
        rx->i = CAN_BIT_WAIT_FRAME;
        return;
    case CAN_BIT_WAIT_FRAME:
        // drop_next was armed by the run that just started, which includes
        // SOF; do not clear it here.
        rx->sm.crc_calc = 0;
        rx->write = &rx->frames[rx->rx_idx];
        rx->i = CAN_BIT_ID_BASE_FIRST;
        return;
    default:
        if (i >= CAN_BIT_DATA_FIRST && i <= CAN_BIT_DATA_LAST &&
            ((i - CAN_BIT_DATA_FIRST) & 7u) == 7u) {
            f->data[(i - CAN_BIT_DATA_FIRST) >> 3] = rx->shift;
        }
        break;
    }
    rx->i = (uint8_t)(i + 1u + jump);
}

// One run of `level`.  The first dominant bit after idle reaches the
// CAN_BIT_WAIT_FRAME case in can_rx_advance(), which consumes SOF.
static inline __attribute__((always_inline)) void can_rx_state(can_rx_t *rx,
                                                               uint8_t level,
                                                               uint16_t delta) {
    // A recessive run while idle is the inter-frame gap, not frame data.
    if (rx->i == CAN_BIT_WAIT_FRAME && level == 1u)
        return;

    int tail = (rx->i != CAN_BIT_WAIT_FRAME && rx->i > CAN_BIT_CRC_LAST);
    uint8_t n = 0;
    for (uint8_t i = 1u; i <= 5u; i++) {
        if (delta >= rx->run_lo[i] && delta <= rx->run_hi[i]) {
            n = i;
            break;
        }
    }
    if (n == 0u) {
        // No ACK: the tail has no edges, so the run reaching the next SOF is
        // longer than any legal stuffed run.  Drop the frame as a NACK and
        // resync at the SOF.
        if (level == 1u && rx->i != CAN_BIT_WAIT_FRAME &&
            rx->i >= CAN_BIT_CRC_TAIL_FIRST) {
            rx->ack_errors++;
            rx->drop_next = 0;
            rx->i = CAN_BIT_WAIT_FRAME;
            return;
        }
        if ((uint32_t)delta > rx->run_hi[5])
            can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
        else
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
        return;
    }

    // Stuffing stops at the CRC.  The one bit still owed is the trailing
    // stuff bit after a 5-run CRC; the delimiter/ACK/EOF are unstuffed.
    if (!tail) {
        uint8_t drop = rx->drop_next;

        rx->drop_next = (uint8_t)(n == 5u);
        if (drop)
            n--;
        // Stuffing ends at the CRC.  A run that reaches the delimiter must not
        // arm another stuff bit (it would consume the ACK).
        if (rx->i != CAN_BIT_WAIT_FRAME &&
            (uint32_t)rx->i + n > CAN_BIT_CRC_LAST + 1u)
            rx->drop_next = 0;
    } else if (rx->drop_next) {
        rx->drop_next = 0;
        n--;
    }

    while (n-- > 0) {
        can_rx_advance(rx, level);
        if (rx->i == CAN_BIT_WAIT_FRAME)
            return;
    }
}

// One edge.  Inlined into the FIFO drain: a delta and one state-machine call.
static inline __attribute__((always_inline)) void can_rx_edge_inline(can_rx_t *rx,
                                                             uint16_t time16,
                                                             uint8_t level) {
    uint16_t delta = (uint16_t)(rx->prev_time16 - time16);

    rx->prev_time16 = time16;
    can_rx_state(rx, level, delta);
}

// One RX FIFO word is the 1->0 timestamp then the 0->1 timestamp.
static inline __attribute__((always_inline)) void can_rx_pair_inline(can_rx_t *rx, uint32_t word) {
    can_rx_edge_inline(rx, (uint16_t)(word >> 16), 1u);
    can_rx_edge_inline(rx, (uint16_t)word, 0u);
}

#endif // CAN_RX_SM_INLINE_H
