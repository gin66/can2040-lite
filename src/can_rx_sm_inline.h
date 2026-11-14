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
    uint16_t fb;

    crc = (uint16_t)(crc << 1);
    fb = (uint16_t)((crc >> 15) ^ bit);
    return (uint16_t)(crc ^ (0x4599u & (uint16_t)(0u - fb)));
}

// n identical bits, n >= 1.  Callers never pass 0.
// Error path only.  Kept out of line so a fit edge does not pay its stores.
static __attribute__((noinline)) void can_rx_fail_rx(can_rx_t *rx,
                                            can_rx_event_t why) {
    rx->pos = CAN_BIT_WAIT_FRAME;
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
// bit index.  IDE/R1 jump it onto the shared control field (R0/DLC at 35..39)
// and DLC jumps it into the right-aligned data region; CRC is at the fixed
// CAN_BIT_CRC_FIRST field.
//
// The field layout lets the hot path skip most tests: every control bit is
// below CAN_BIT_DATA_FIRST, every data bit is below the fixed CRC field, and
// every CRC or tail bit is at or above CAN_BIT_CRC_FIRST.  A valid position
// never exceeds ACK_SLOT, which fits the raw slot, so the store needs no
// range check.
static inline __attribute__((always_inline)) void can_rx_advance(can_rx_t *rx,
                                                                uint8_t bit) {
    uint8_t p = rx->pos;
    can_rx_frame_t *f;
    uint8_t *dest;

    if (p == CAN_BIT_WAIT_FRAME) {
        rx->pos = 0;
        rx->write->flags = 0;
        rx->sm.crc_calc = 0;
        rx->write = &rx->frames[rx->rx_idx];
        return;
    }

    f = rx->write;
    dest = &f->raw[p >> 3];
    *dest = (uint8_t)((*dest << 1) | bit);

    // CRC covers SOF..data, which all live below the fixed CRC field.  DLC
    // jumps the store position so data ends at DATA_LAST and the CRC field is
    // always at CAN_BIT_CRC_FIRST.
    if (p < CAN_BIT_CRC_FIRST) {
        rx->sm.crc_calc = can_rx_crc_bit(rx->sm.crc_calc, bit);

        if (p < CAN_BIT_DATA_FIRST) {
            if (p == CAN_BIT_IDE) {
                f->flags = bit;
                if (bit == 0u)
                    p = (uint8_t)(p + CAN_IDE_STD_SKIP);
            } else if (p == CAN_BIT_R1) {
                p = (uint8_t)(p + (CAN_BIT_R0 - (CAN_BIT_R1 + 1u)));
            } else if (p == CAN_BIT_DLC_LAST) {
                uint8_t dlc = (uint8_t)(*dest & 0x0fu);

                if (dlc > CAN_DLC_MAX)
                    dlc = CAN_DLC_MAX;
                p = (uint8_t)(p + CAN_DATA_SKIP(dlc));
            }
        }
    } else if (p == CAN_BIT_CRC_FIRST + 7u) {
        // The CRC field is byte-aligned; this is its first byte.
        if (f->raw[p >> 3] != (uint8_t)(rx->sm.crc_calc >> 7)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
    } else if (p == CAN_BIT_CRC_FIRST + 14u) {
        // Last 7 CRC bits are right-aligned in the low bits.
        if ((f->raw[p >> 3] & 0x7fu) != (rx->sm.crc_calc & 0x7fu)) {
            can_rx_fail_rx(rx, CAN_RX_CRC_ERROR);
            return;
        }
    } else if (p == CAN_BIT_CRC_FIRST + 15u) {
        // CRC delimiter must be recessive.
        if (bit == 0u) {
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
            return;
        }
        rx->drop_next = 0;
    } else if (p == CAN_BIT_CRC_FIRST + 16u) {
        if (bit == 0u)
            can_rx_publish(rx);
        else
            rx->ack_errors++;
        rx->drop_next = 0;
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
                (uint32_t)rx->pos > (uint32_t)CAN_BIT_CRC_FIRST + 14u);
    uint8_t n = 0;

    // run_hi[] increases with n and the valid windows do not overlap, so the
    // first bound that contains delta selects the run.  A separate lower-bound
    // check then rejects the inter-window gaps and sub-one-bit glitches.
    if (delta <= rx->run_hi[1])
        n = 1;
    else if (delta <= rx->run_hi[2])
        n = 2;
    else if (delta <= rx->run_hi[3])
        n = 3;
    else if (delta <= rx->run_hi[4])
        n = 4;
    else if (delta <= rx->run_hi[5])
        n = 5;
    if (n != 0u && (uint32_t)delta < rx->run_lo[n])
        n = 0;

    if (n == 0u) {
        // No ACK: the tail has no edges, so the run reaching the next SOF is
        // longer than any legal stuffed run.  Drop the frame as a NACK and
        // resync at the SOF.
        if (level == 1u && rx->pos != CAN_BIT_WAIT_FRAME &&
            (uint32_t)rx->pos >= (uint32_t)CAN_BIT_CRC_FIRST + 11u) {
            rx->ack_errors++;
            rx->drop_next = 0;
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
