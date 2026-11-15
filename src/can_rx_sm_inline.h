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
// Optional probe around the once-per-frame CRC/stuffing prediction.  It runs
// at CRC entry, at least 11 bit times before the last-rise anchor, so its
// width is the margin check for the ACK pulse path.
#ifndef CAN_ACK_PREDICT_PROBE_BEGIN
#define CAN_ACK_PREDICT_PROBE_BEGIN() ((void)0)
#endif
#ifndef CAN_ACK_PREDICT_PROBE_END
#define CAN_ACK_PREDICT_PROBE_END() ((void)0)
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
#if CAN_ACK_GENERATION_SUPPORT
    rx->crc_latched = 0;
    rx->ack_active = 0;
    rx->ack_abort = 0;
    rx->ack_anchored = 0;
    rx->ack_pending = 0;
    rx->tail_on = 0;
    rx->tail_stuff = 0;
#endif
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

#if CAN_ACK_GENERATION_SUPPORT
// Compute the expected CRC wire from the now-complete CRC register and the
// stuffer state at the first CRC data bit.  This runs once per frame at CRC
// entry; the last-rise anchor is at least 11 bit times later, so the walk is
// off the pulse path.  `can_ack_wire` is in SRAM on the Pico.
static inline void can_rx_ack_predict(can_rx_t *rx) {
    rx->ack_wire_pos = 0;
    rx->ack_abort = 0;
    rx->ack_anchored = 0;
    rx->ack_active = 0;
    if (rx->crc_run < 1u || rx->crc_run > 4u)
        return;
    CAN_ACK_PREDICT_PROBE_BEGIN();
    can_ack_wire(rx->sm.crc_calc, rx->crc_run, rx->crc_last, &rx->ack_wire);
    CAN_ACK_PREDICT_PROBE_END();
    if (rx->ack_wire.slot_bit < 2u || rx->ack_wire.last_rise == 0u)
        return;
    rx->ack_active = 1;
}
#endif

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
#if CAN_ACK_GENERATION_SUPPORT
        // The first dominant bit is SOF; the run tracker restarts here.
        rx->crc_run = 1;
        rx->crc_last = (uint8_t)(bit & 1u);
        rx->crc_latched = 0;
        rx->ack_active = 0;
        rx->ack_abort = 0;
        rx->ack_anchored = 0;
        rx->ack_pending = 0;
        rx->tail_on = 0;
        rx->tail_stuff = 0;
#endif
        return;
    }

    f = rx->write;
    dest = &f->raw[p >> 3];
    *dest = (uint8_t)((*dest << 1) | bit);

#if CAN_ACK_GENERATION_SUPPORT
    // The expected wire is checked bit by bit, including the first CRC bit.
    // A mismatch stops the ACK; the normal CRC compare still decides whether
    // the frame is published.  Dropped stuff bits are counted by the run
    // handler in can_rx_state().
    if (rx->ack_active && !rx->ack_abort &&
        (uint32_t)p >= (uint32_t)CAN_BIT_CRC_FIRST &&
        (uint32_t)p < (uint32_t)CAN_BIT_CRC_FIRST + CAN_CRC_BITS) {
        if (rx->ack_wire.level[rx->ack_wire_pos] != bit)
            rx->ack_abort = 1;
        else
            rx->ack_wire_pos++;
    }
#endif

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
    }
#if CAN_ACK_GENERATION_SUPPORT
    else if (p == CAN_BIT_CRC_FIRST + 16u) {
        // The slot is sampled at 7/8 T.  A dominant slot arms publish; the
        // recessive ACK delimiter below is what actually commits the frame.
        if (bit == 0u)
            rx->ack_pending = 1;
        else
            rx->ack_errors++;
        rx->drop_next = 0;
        rx->pos = (uint8_t)(p + 1u);
        return;
    } else if (p == CAN_BIT_CRC_FIRST + 17u) {
        // ACK delimiter must be recessive.
        if (bit == 0u) {
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
            return;
        }
        if (rx->ack_pending) {
            can_rx_publish(rx);
            rx->ack_rx++;
        }
        rx->ack_pending = 0;
        rx->drop_next = 0;
        rx->tail_on = 0;
        rx->crc_latched = 0;
        rx->ack_active = 0;
        rx->pos = CAN_BIT_WAIT_FRAME;
        return;
    }
#else
    else if (p == CAN_BIT_CRC_FIRST + 16u) {
        if (bit == 0u)
            can_rx_publish(rx);
        else
            rx->ack_errors++;
        rx->drop_next = 0;
        rx->pos = CAN_BIT_WAIT_FRAME;
        return;
    }
#endif
    rx->pos = (uint8_t)(p + 1u);
}

#if CAN_ACK_GENERATION_SUPPORT
static inline __attribute__((always_inline)) uint8_t can_rx_is_tail(const can_rx_t *rx) {
    uint32_t base = (uint32_t)CAN_BIT_CRC_FIRST + 15u;

    return (uint8_t)((uint32_t)rx->pos == base ||
                     (uint32_t)rx->pos == base + 1u ||
                     (uint32_t)rx->pos == base + 2u);
}

// A dominant trailing stuff bit is the last 0->1 rise of the expected wire,
// but the tail walker owns it instead of the normal dominant-run path.
static inline uint8_t can_rx_ack_anchor_is_stuff(const can_rx_t *rx) {
    return (uint8_t)(rx->ack_active && !rx->ack_abort && !rx->ack_anchored &&
                     rx->ack_wire.has_trailing_stuff &&
                     rx->ack_wire.trailing_stuff_level == 0u &&
                     rx->ack_wire.r_tail == 1u);
}

// Tail walker for the CRC delimiter, ACK slot, and ACK delimiter.  Every
// edge is judged at the nominal 7/8-T sample point.  `level` is the level of
// the run that just ended (the pre-edge level); the sample stored is the
// post-edge level.  Publishing happens on a recessive ACK delimiter only.
static inline __attribute__((always_inline))
void can_rx_tail_edge(can_rx_t *rx, uint16_t time16, uint8_t level) {
    uint16_t ticks = rx->ticks_per_bit;
    uint16_t sample = (uint16_t)((ticks * 7u) / 8u);
    uint8_t post = (uint8_t)(level ^ 1u);

    while (rx->tail_on &&
           (rx->tail_stuff || can_rx_is_tail(rx)) &&
           (uint16_t)(rx->tail_mark - time16) >= ticks) {
        if (rx->tail_stuff) {
            // The rise that ends a dominant trailing stuff bit is the last
            // 0->1 of the expected wire.  Arm the pulse before the bit is
            // dropped; this is the R = 1 tight case.
            if (post == 1u && can_rx_ack_anchor_is_stuff(rx)) {
                rx->ack_anchored = 1;
                can_rx_ack_anchor(rx);
            }
            // Drop the trailing stuff bit; the delimiter opens next.
            rx->tail_stuff = 0;
            rx->tail_mark = (uint16_t)(rx->tail_mark - ticks);
            if (!(rx->tail_on && can_rx_is_tail(rx)))
                break;
            rx->tail_sample = level;
            continue;
        }
        CAN_RX_BIT_PROBE_BEGIN();
        can_rx_advance(rx, rx->tail_sample);
        CAN_RX_BIT_PROBE_END();
        rx->tail_mark = (uint16_t)(rx->tail_mark - ticks);
        if (!(rx->tail_on && can_rx_is_tail(rx)))
            break;
        // Seed the newly opened bit with the pre-edge level.  Step 2 below
        // may overwrite it when this edge is before the sample point.
        rx->tail_sample = level;
    }

    if (rx->tail_on && can_rx_is_tail(rx) &&
        (uint16_t)(rx->tail_mark - time16) <= sample)
        rx->tail_sample = post;

    // A 0->1 inside the open slot commits a recessive slot sample now.  The
    // ACK rise in the delimiter commits the open delimiter in this call.
    if (post == 1u && rx->tail_on) {
        if (rx->pos == (uint8_t)(CAN_BIT_CRC_FIRST + 16u) &&
            (uint16_t)(rx->tail_mark - time16) <= sample) {
            CAN_RX_BIT_PROBE_BEGIN();
            can_rx_advance(rx, rx->tail_sample);
            CAN_RX_BIT_PROBE_END();
            rx->tail_mark = (uint16_t)(rx->tail_mark - ticks);
            if (rx->tail_on &&
                rx->pos == (uint8_t)(CAN_BIT_CRC_FIRST + 17u)) {
                CAN_RX_BIT_PROBE_BEGIN();
                can_rx_advance(rx, rx->tail_sample);
                CAN_RX_BIT_PROBE_END();
            }
        } else if (rx->pos == (uint8_t)(CAN_BIT_CRC_FIRST + 17u)) {
            CAN_RX_BIT_PROBE_BEGIN();
            can_rx_advance(rx, rx->tail_sample);
            CAN_RX_BIT_PROBE_END();
        }
    }
}
#endif

// Emit one data bit.  Under the ACK flag this latches the stuffer state just
// before the first CRC data bit and tracks the live run for that latch.
// Returns the position before the call.
static inline __attribute__((always_inline)) uint8_t can_rx_emit_bit(can_rx_t *rx,
                                                                     uint8_t level) {
    uint8_t p = rx->pos;
#if CAN_ACK_GENERATION_SUPPORT
    uint8_t was_wait = (uint8_t)(p == CAN_BIT_WAIT_FRAME);
    uint8_t latch_now = (uint8_t)(!rx->crc_latched &&
                                  p == CAN_BIT_CRC_FIRST);

    if (latch_now) {
        rx->crc_latched = 1;
        can_rx_ack_predict(rx);
    }
#endif
    CAN_RX_BIT_PROBE_BEGIN();
    can_rx_advance(rx, level);
    CAN_RX_BIT_PROBE_END();
#if CAN_ACK_GENERATION_SUPPORT
    if (!was_wait && !latch_now) {
        if (level == rx->crc_last) {
            rx->crc_run++;
        } else {
            rx->crc_last = level;
            rx->crc_run = 1;
        }
    }
#endif
    return p;
}

// One run of `level`.  The first dominant bit after idle starts a frame.
static inline __attribute__((always_inline)) void can_rx_state(can_rx_t *rx,
                                                               uint8_t level,
                                                               uint16_t delta) {
    // A recessive run while idle is the inter-frame gap, not frame data.
    if (rx->pos == CAN_BIT_WAIT_FRAME && level == 1u)
        return;

#if CAN_ACK_GENERATION_SUPPORT
    if (rx->tail_on) {
        can_rx_tail_edge(rx, rx->prev_time16, level);
        return;
    }
#endif

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
#if CAN_ACK_GENERATION_SUPPORT
        // No ACK: the tail has no edges, so the run reaching the next SOF is
        // longer than any legal stuffed run.  Walk the recessive delimiter,
        // slot, and ACK delimiter so ack_errors comes from the slot arm.
        if (level == 1u && rx->pos != CAN_BIT_WAIT_FRAME &&
            (uint32_t)rx->pos >= (uint32_t)CAN_BIT_CRC_FIRST + 11u &&
            (uint32_t)delta > rx->ack_guard) {
            rx->pos = (uint8_t)(CAN_BIT_CRC_FIRST + 15u);
            while (rx->pos != CAN_BIT_WAIT_FRAME)
                can_rx_advance(rx, 1u);
            return;
        }
        // An unmatched delta that ends at or after the last stuffed bit is
        // the inset fall: emit the remaining CRC bits at this run's level,
        // hand the boundary to the walker, and do not call it a bit error.
        if (rx->pos != CAN_BIT_WAIT_FRAME &&
            (uint32_t)rx->pos <= (uint32_t)CAN_BIT_CRC_FIRST + 14u) {
            uint32_t remaining = ((uint32_t)CAN_BIT_CRC_FIRST + 15u) -
                                 (uint32_t)rx->pos;
            uint32_t lead = rx->drop_next ? 1u : 0u;
            uint32_t need = remaining + lead;
            uint32_t ticks = rx->ticks_per_bit;

            if (remaining <= 5u && (uint32_t)delta >= need * ticks) {
                uint32_t r;

                if (rx->drop_next) {
                    rx->drop_next = 0;
                    if (!rx->crc_latched) {
                        rx->crc_last = level;
                        rx->crc_run = 1;
                    }
                }
                for (r = 0; r < remaining; r++) {
                    (void)can_rx_emit_bit(rx, level);
                    if (rx->pos == CAN_BIT_WAIT_FRAME)
                        break;
                }
                if (rx->pos == CAN_BIT_WAIT_FRAME)
                    return;
                rx->tail_on = 1;
                rx->tail_stuff = 0;
                rx->tail_sample = level;
                rx->tail_mark = (uint16_t)(rx->prev_time16 +
                    (uint16_t)((uint32_t)delta - need * ticks));
                can_rx_tail_edge(rx, rx->prev_time16, level);
                return;
            }
        }
        if ((uint32_t)delta > rx->run_hi[5])
            can_rx_fail_rx(rx, CAN_RX_STUFF_ERROR);
        else
            can_rx_fail_rx(rx, CAN_RX_BIT_ERROR);
        return;
#else
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
#endif
    }

    // Stuffing stops at the CRC.  The one bit still owed is the trailing
    // stuff bit after a 5-run CRC; the delimiter/ACK are unstuffed.
    uint8_t drop = 0;
    if (!tail) {
        drop = rx->drop_next;
        rx->drop_next = (uint8_t)(n == 5u);
        if (drop)
            n--;
    } else if (rx->drop_next) {
        rx->drop_next = 0;
        n--;
        drop = 1;
    }

#if CAN_ACK_GENERATION_SUPPORT
    if (drop && !rx->crc_latched) {
        // The stuff bit continues this run at the same level.
        rx->crc_last = level;
        rx->crc_run = 1;
    }

    // A dropped stuff bit inside the CRC field is one more expected wire
    // bit.  The one before the first CRC data bit belongs to the data field
    // and is not part of the prediction.
    if (rx->ack_active && !rx->ack_abort && drop &&
        (uint32_t)rx->pos > (uint32_t)CAN_BIT_CRC_FIRST) {
        if (rx->ack_wire.level[rx->ack_wire_pos] != level)
            rx->ack_abort = 1;
        else
            rx->ack_wire_pos++;
    }

    // The last 0->1 rise of the expected CRC wire is the phase anchor.  This
    // run ends at that rise; call the hook before parsing the run so the
    // R = 1 case still has the full 1.25 bit budget.  The run's bits are
    // checked against the expected wire afterwards by the normal emit path.
    if (rx->ack_active && !rx->ack_abort && !rx->ack_anchored &&
        level == 0u &&
        (uint32_t)rx->pos > (uint32_t)CAN_BIT_CRC_FIRST &&
        (uint16_t)(rx->ack_wire_pos + n) == rx->ack_wire.last_rise) {
        uint8_t ok = 1;
        uint8_t k;

        for (k = 0; k < n; k++) {
            if (rx->ack_wire.level[rx->ack_wire_pos + k] != 0u) {
                ok = 0;
                break;
            }
        }
        if (ok) {
            rx->ack_anchored = 1;
            can_rx_ack_anchor(rx);
        } else {
            rx->ack_abort = 1;
        }
    }
#endif

    uint8_t count = n;
    for (uint8_t k = 0; k < count; k++) {
#if CAN_ACK_GENERATION_SUPPORT
        uint8_t p = can_rx_emit_bit(rx, level);
#else
        (void)can_rx_emit_bit(rx, level);
        uint8_t p = 0;
#endif

        if (rx->pos == CAN_BIT_WAIT_FRAME)
            break;
#if CAN_ACK_GENERATION_SUPPORT
        if ((uint32_t)p == (uint32_t)CAN_BIT_CRC_FIRST + 14u) {
            // Last CRC data bit.  Bits after it in this run are tail bits;
            // a 5-run that ends here still owes one trailing stuff bit.
            uint8_t after = (uint8_t)(count - 1u - k);
            uint8_t wire_len = (uint8_t)(count + (drop ? 1u : 0u));

            rx->tail_on = 1;
            rx->tail_sample = level;
            rx->tail_mark = (uint16_t)(rx->prev_time16 +
                (uint16_t)((uint32_t)after * rx->ticks_per_bit));
            rx->tail_stuff = (uint8_t)(after == 0u && wire_len == 5u);
            can_rx_tail_edge(rx, rx->prev_time16, level);
            return;
        }
#else
        (void)p;
#endif
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
