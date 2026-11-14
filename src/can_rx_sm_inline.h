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

// The Pico backend overrides these for a per-bit logic-analyzer probe.  Host
// builds and targets that do not probe use the empty defaults.
#ifndef CAN_RX_BIT_PROBE_BEGIN
#define CAN_RX_BIT_PROBE_BEGIN() ((void)0)
#endif
#ifndef CAN_RX_BIT_PROBE_END
#define CAN_RX_BIT_PROBE_END() ((void)0)
#endif

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
    rx->pos = CAN_BIT_WAIT_FRAME;
    rx->crc_start = CAN_RX_CRC_NONE;
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

// One unstuffed wire bit.  `pos` is both the protocol state and the raw slot
// bit index; IDE/R1 jump it so DLC (36-39), data (40..) and CRC are aligned.
static inline __attribute__((always_inline)) void can_rx_advance(can_rx_t *rx,
                                                                uint8_t bit) {
    can_rx_frame_t *f = rx->write;
    uint8_t p = rx->pos;
    uint8_t shift = 0;

    if (p == CAN_BIT_WAIT_FRAME) {
        rx->pos = 0;
        rx->crc_start = CAN_RX_CRC_NONE;
        rx->write->flags = 0;
        rx->sm.crc_calc = 0;
        rx->write = &rx->frames[rx->rx_idx];
        return;
    }

    if (p < CAN_RX_RAW_BYTES * 8u) {
        uint8_t *dest = &f->raw[p >> 3];

        shift = (uint8_t)((*dest << 1) | bit);
        *dest = shift;
    }

    // CRC covers SOF..data.  crc_start is 0 until the DLC has been decoded.
    if (p < rx->crc_start)
        rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);

    if (p == CAN_BIT_IDE) {
        f->flags = bit;
        if (bit == 0u)
            p = (uint8_t)(p + CAN_IDE_STD_SKIP);
    } else if (p == CAN_BIT_R1) {
        p = (uint8_t)(p + (CAN_BIT_R0 - (CAN_BIT_R1 + 1u)));
    } else if (p == CAN_BIT_DLC_LAST) {
        uint8_t dlc = (uint8_t)(shift & 0x0fu);

        if (dlc > CAN_DLC_MAX)
            dlc = CAN_DLC_MAX;
        rx->crc_start = (uint8_t)(CAN_BIT_DATA_FIRST + 8u * dlc);
    } else if (p == rx->crc_start + 7u) {
        // The CRC start is byte-aligned; this is its first byte.
        if (f->raw[p >> 3] != (uint8_t)(rx->sm.crc_calc >> 7)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
    } else if (p == rx->crc_start + 14u) {
        // Last 7 CRC bits are right-aligned in the low bits.
        if ((f->raw[p >> 3] & 0x7fu) != (rx->sm.crc_calc & 0x7fu)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
    } else if (p == rx->crc_start + 15u) {
        // CRC delimiter must be recessive.
        if (bit == 0u) {
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
            return;
        }
        rx->drop_next = 0;
    } else if (p == rx->crc_start + 16u) {
        if (bit == 0u)
            can_rx_publish(rx);
        else
            rx->ack_errors++;
        rx->drop_next = 0;
        rx->crc_start = CAN_RX_CRC_NONE;
        rx->pos = CAN_BIT_WAIT_FRAME;
        return;
    }
    rx->pos = (uint8_t)(p + 1u);
}

// One run of `level`.  The first dominant bit after idle starts a frame.
static inline __attribute__((always_inline)) void can_rx_state(can_rx_t *rx,
                                                               uint8_t level,
                                                               uint16_t delta) {
    // A recessive run while idle is the inter-frame gap, not frame data.
    if (rx->pos == CAN_BIT_WAIT_FRAME && level == 1u)
        return;

    int tail = (rx->pos != CAN_BIT_WAIT_FRAME &&
                (uint32_t)rx->pos > (uint32_t)rx->crc_start + 14u);
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
        if (level == 1u && rx->pos != CAN_BIT_WAIT_FRAME &&
            (uint32_t)rx->pos >= (uint32_t)rx->crc_start + 11u) {
            rx->ack_errors++;
            rx->drop_next = 0;
            rx->crc_start = CAN_RX_CRC_NONE;
            rx->pos = CAN_BIT_WAIT_FRAME;
            return;
        }
        if ((uint32_t)delta > rx->run_hi[5])
            can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
        else
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
        return;
    }

    // Stuffing stops at the CRC.  The one bit still owed is the trailing
    // stuff bit after a 5-run CRC; the delimiter/ACK are unstuffed.
    if (!tail) {
        uint8_t drop = rx->drop_next;

        rx->drop_next = (uint8_t)(n == 5u);
        if (drop)
            n--;
    } else if (rx->drop_next) {
        rx->drop_next = 0;
        n--;
    }

    while (n-- > 0) {
        CAN_RX_BIT_PROBE_BEGIN();
        can_rx_advance(rx, level);
        CAN_RX_BIT_PROBE_END();
        if (rx->pos == CAN_BIT_WAIT_FRAME)
            break;
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
