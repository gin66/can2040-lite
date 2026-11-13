// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_edge_sm.c — unstuff, CRC-15, and field parse of an edge-timestamp
// CAN stream.  See can_edge_sm.h for the FIFO word and the ACK arming rule.

#include <string.h>

#include "can_edge_sm.h"

// One uint8_t encodes both the protocol field and the bits still owed in it.
// The active range for a field is base+1 .. base+len, so a bit just does
// state--; reaching the base means the field is complete.  This replaces the
// old phase/flags/bits_left/bit_index variables.
enum {
    ST_IDLE = 0,
    ST_SOF = 1,
    ST_ID = 2,          // 3..13   (11 base-ID bits)
    ST_RTR = 14,
    ST_IDE = 15,
    ST_R0 = 16,
    ST_EXT_ID = 17,     // 18..35  (18 extended-ID bits)
    ST_EXT_RTR = 36,
    ST_R1 = 37,
    ST_R0E = 38,
    ST_DLC = 39,        // 40..43  (4 DLC bits)
    ST_DATA = 44,       // 45..108 (up to 64 data bits)
    ST_CRC = 109,       // 110..124 (15 CRC bits)
    ST_CRC_DONE = 125,  // waiting for the stuff bit after a 5-run CRC
    ST_DELIM = 126,
    ST_ACK = 127,
    ST_ACKDEL = 128,
    ST_EOF = 129,       // 130..136 (7 EOF bits)
    ST_IFS = 137,       // 138..140 (3 IFS bits)
    ST_ERROR = 141
};

static uint32_t elapsed_ticks(uint32_t earlier, uint32_t later) {
    return (earlier - later) & CAN_EDGE_TIME_MASK;
}

static void publish(can_edge_sm_t *sm) {
    can_parsed_t *p = &sm->parsed;

    memset(p, 0, sizeof(*p));
    p->id = sm->id;
    p->dlc = sm->dlc;
    p->is_extended = sm->is_ext;
    p->is_rtr = sm->is_rtr;
    p->ack = sm->ack_seen ? sm->ack : 0;
    memcpy(p->data, sm->data, sizeof(p->data));
    p->crc = sm->crc_recv;
    p->crc_calc = sm->crc_calc;
    p->validity = sm->validity;
    p->stuffed_bits = sm->stuffed_raw;
    p->total_bits = sm->raw_bits;
    sm->frame_ready = 1;
}

static void fail(can_edge_sm_t *sm, frame_validity_t why) {
    sm->validity = why;
    sm->in_frame = 0;
    sm->state = ST_ERROR;
    publish(sm);
}

static void finish_ok(can_edge_sm_t *sm) {
    sm->validity = VALID_FRAME;
    sm->in_frame = 0;
    sm->state = ST_IDLE;
    publish(sm);
}

static void crc_bit(can_edge_sm_t *sm, uint8_t bit) {
    uint16_t mix = (uint16_t)(((sm->crc_calc >> 14) ^ bit) & 1u);
    sm->crc_calc = (uint16_t)((sm->crc_calc << 1) & 0x7fff);
    if (mix)
        sm->crc_calc ^= 0x4599;
}

static void note_run(can_edge_sm_t *sm, uint8_t bit) {
    if (sm->run > 0 && bit == sm->last)
        sm->run++;
    else {
        sm->run = 1;
        sm->last = bit;
    }
}

static void enter_crc(can_edge_sm_t *sm) {
    sm->state = (uint8_t)(ST_CRC + 15u);
    sm->crc_recv = 0;
}

// Stuffing has ended.  raw_bits includes the last stuffed bit.  The tail
// starts at bit_end; tail_sample defaults to the level at that boundary and
// is refreshed by any edge that arrives before the sample point.
static void enter_delim(can_edge_sm_t *sm, uint32_t bit_end) {
    sm->state = ST_DELIM;
    sm->stuffed_raw = sm->raw_bits;
    sm->tail_mark = bit_end;
    sm->tail_sample = sm->level;
}

static void crc_done(can_edge_sm_t *sm, uint32_t bit_end) {
    sm->crc_match = (uint8_t)(sm->crc_recv == sm->crc_calc);
    if (!sm->crc_match) {
        fail(sm, INVALID_CRC);
        return;
    }

    // The ACK slot is the bit after the delimiter.  A 5-run still owes
    // its stuff bit before that delimiter.
#if CAN_ACK_GENERATION_SUPPORT
    {
        uint32_t ahead = (sm->run == 5u) ? 2u : 1u;

        sm->ack_armed = 1;
        sm->ack_arm_time =
            (bit_end - ahead * sm->ticks_per_bit) & CAN_EDGE_TIME_MASK;
    }
#endif
    if (sm->run == 5u)
        sm->state = ST_CRC_DONE;
    else
        enter_delim(sm, bit_end);
}

// One unstuffed bit.  The state value carries the field position:
//   ST_ID+11..ST_ID+1, ST_EXT_ID+18..+1, ST_DLC+4..+1,
//   ST_DATA+64..+1, ST_CRC+15..+1, ST_EOF+7..+1, ST_IFS+3..+1
// Each bit decrements the state; reaching the base completes the field.
static void parse_bit(can_edge_sm_t *sm, uint8_t bit, uint32_t bit_end) {
    uint8_t s = sm->state;

    if (s > ST_ID && s <= ST_ID + 11u) {
        sm->shift = (sm->shift << 1) | bit;
        crc_bit(sm, bit);
        if (s == ST_ID + 1u) {
            sm->id = sm->shift & 0x7ffu;
            sm->state = ST_RTR;
        } else {
            sm->state = (uint8_t)(s - 1u);
        }
        return;
    }
    if (s > ST_EXT_ID && s <= ST_EXT_ID + 18u) {
        sm->shift = (sm->shift << 1) | bit;
        crc_bit(sm, bit);
        if (s == ST_EXT_ID + 1u) {
            sm->id = ((sm->id & 0x7ffu) << 18) |
                     (sm->shift & 0x3ffffu);
            sm->state = ST_EXT_RTR;
        } else {
            sm->state = (uint8_t)(s - 1u);
        }
        return;
    }
    if (s > ST_DLC && s <= ST_DLC + 4u) {
        sm->shift = (sm->shift << 1) | bit;
        crc_bit(sm, bit);
        if (s == ST_DLC + 1u) {
            uint8_t dlc = (uint8_t)(sm->shift & 0x0fu);

            if (dlc > 8u)
                dlc = 8u;
            sm->dlc = dlc;
            if (sm->is_rtr || dlc == 0u)
                enter_crc(sm);
            else
                sm->state = (uint8_t)(ST_DATA + (uint32_t)dlc * 8u);
        } else {
            sm->state = (uint8_t)(s - 1u);
        }
        return;
    }
    if (s > ST_DATA && s <= ST_DATA + 64u) {
        uint32_t rem = (uint32_t)(s - ST_DATA);
        uint32_t idx = (uint32_t)sm->dlc * 8u - rem;

        if (bit) {
            uint8_t byte = (uint8_t)(idx >> 3);
            uint8_t pos = (uint8_t)(7u - (idx & 7u));

            if (byte < 8u)
                sm->data[byte] |= (uint8_t)(1u << pos);
        }
        crc_bit(sm, bit);
        if (rem == 1u)
            enter_crc(sm);
        else
            sm->state = (uint8_t)(s - 1u);
        return;
    }
    if (s > ST_CRC && s <= ST_CRC + 15u) {
        uint32_t rem = (uint32_t)(s - ST_CRC);

#if CAN_ACK_GENERATION_SUPPORT
        if (rem == 15u) {
            sm->crc_started = 1;
            sm->crc_raw_start = sm->raw_bits - 1u;
            sm->crc_run_start = sm->run;
            sm->crc_last_start = sm->last;
        }
#endif
        sm->crc_recv = (uint16_t)((sm->crc_recv << 1) | bit);
        if (rem == 1u)
            sm->state = ST_CRC;     // done marker; crc_done() runs after note
        else
            sm->state = (uint8_t)(s - 1u);
        return;
    }
    if (s > ST_EOF && s <= ST_EOF + 7u) {
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        if (s == ST_EOF + 1u)
            sm->state = (uint8_t)(ST_IFS + 3u);
        else
            sm->state = (uint8_t)(s - 1u);
        return;
    }
    if (s > ST_IFS && s <= ST_IFS + 3u) {
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        if (s == ST_IFS + 1u)
            finish_ok(sm);
        else
            sm->state = (uint8_t)(s - 1u);
        return;
    }

    switch (s) {
    case ST_SOF:
        crc_bit(sm, bit);
        sm->shift = 0;
        sm->state = (uint8_t)(ST_ID + 11u);
        break;
    case ST_RTR:
        sm->is_rtr = bit & 1u;      // standard RTR; overwritten if extended
        crc_bit(sm, bit);
        sm->state = ST_IDE;
        break;
    case ST_IDE:
        sm->is_ext = bit & 1u;
        crc_bit(sm, bit);
        if (bit) {
            sm->shift = 0;
            sm->state = (uint8_t)(ST_EXT_ID + 18u);
        } else {
            sm->state = ST_R0;
        }
        break;
    case ST_R0:
        crc_bit(sm, bit);
        sm->shift = 0;
        sm->state = (uint8_t)(ST_DLC + 4u);
        break;
    case ST_EXT_RTR:
        sm->is_rtr = bit & 1u;
        crc_bit(sm, bit);
        sm->state = ST_R1;
        break;
    case ST_R1:
        crc_bit(sm, bit);
        sm->state = ST_R0E;
        break;
    case ST_R0E:
        crc_bit(sm, bit);
        sm->shift = 0;
        sm->state = (uint8_t)(ST_DLC + 4u);
        break;
    case ST_DELIM:
        if (!sm->tail_sample) {
            fail(sm, INVALID_BIT);
            return;
        }
        sm->state = ST_ACK;
        sm->tail_mark = bit_end;
        sm->tail_sample = sm->level;
        break;
    case ST_ACK:
        sm->ack = (uint8_t)!sm->tail_sample;
        sm->ack_seen = 1;
        sm->state = ST_ACKDEL;
        sm->tail_mark = bit_end;
        sm->tail_sample = sm->level;
        break;
    case ST_ACKDEL:
        if (!sm->tail_sample) {
            fail(sm, INVALID_BIT);
            return;
        }
        sm->state = (uint8_t)(ST_EOF + 7u);
        break;
    default:
        fail(sm, INVALID_BIT);
        break;
    }
}

static void feed_raw(can_edge_sm_t *sm, uint8_t bit, uint32_t bit_end) {
    if (!sm->in_frame)
        return;
    if (sm->raw_bits >= CAN2040_LITE_MAX_FRAME_BITS) {
        fail(sm, INVALID_BIT);
        return;
    }
    sm->raw_bits++;

    // Stuffing is part of the frame state machine, not a separate filter.
    // `state` decides whether the stuffed region is still active; only then
    // can the next bit after a 5-run be a stuff bit.  Not every bit is
    // stuffed, and the tail (delimiter/ACK/EOF/IFS) is not stuffed at all.
    if (sm->state == ST_CRC_DONE) {
        // CRC ended on a 5-run: this first bit of the next run is the
        // trailing stuff bit, still inside the stuffed region.
        if (sm->run != 5u || bit == sm->last) {
            fail(sm, INVALID_STUFF);
            return;
        }
        sm->run = 1;
        sm->last = bit;
        enter_delim(sm, bit_end);
        return;
    }
    if (sm->state < ST_CRC_DONE && sm->run == 5u) {
        if (bit == sm->last) {
            fail(sm, INVALID_STUFF);
            return;
        }
        sm->run = 1;
        sm->last = bit;
        return;
    }

    // The delimiter, ACK, and ACK delimiter are judged at the 7/8-T sample
    // point, not at the bit boundary.
    if (sm->state == ST_DELIM || sm->state == ST_ACK ||
        sm->state == ST_ACKDEL)
        bit = sm->tail_sample;

    parse_bit(sm, bit, bit_end);
    note_run(sm, bit);
    if (sm->state == ST_CRC)
        crc_done(sm, bit_end);
}

// Classify one edge-to-edge interval as a run of n identical bits by its
// delta time only.  CAN stuffing guarantees at most five identical bits
// between edges; longer runs are accepted here so the stuff state machine can
// report the violation (INVALID_STUFF) rather than a generic form error.
//
// The acceptance budget for |elapsed - n*T| is the sum of three independent
// errors, in ticks:
//   - bit-clock mismatch: 0.5% of the interval.  ISO 11898-1 bounds the
//     oscillator tolerance by SJW / (20 * NBT); with SJW = 1 TQ and
//     NBT = 8 TQ that is 0.625%.  Crystal nodes are far better, so 0.5% is a
//     conservative cap.
//   - bus edge asymmetry: T/8.  Dominant is actively driven while recessive
//     rises through the termination and bus capacitance and the transceiver
//     comparator, so the two edge delays differ.  The Fly-Micro4 1 Mbit/s
//     recording measures +0.5 tick (dominant) and -2.6 tick (recessive) at
//     T = 66, about 4% of a bit; 12.5% covers slower or longer buses.
//   - capture quantization: 1 tick from the two-cycle PIO counter.
//
// Rounding to the nearest n keeps the windows disjoint; the budget only
// rejects intervals that are not a whole number of bits.  An interval longer
// than the maximum legal run is classified as a stuffing violation.
enum {
    CAN_EDGE_CLOCK_TOL_DIV = 200,   // 0.5% of the interval
    CAN_EDGE_EDGE_SKEW_DIV = 8      // 1/8 bit of slew/comparator asymmetry
};

// Precompute the accepted upper edge for each legal run length.  This is the
// only place that divides; the per-edge classifier just compares.  With the
// 1 Mbit/s Fly-Micro4 clock (133 MHz, 66 ticks/bit) these become 75, 141,
// 207, 273 and 339 ticks.
static void run_windows_init(can_edge_sm_t *sm) {
    uint32_t n;
    uint32_t t = sm->ticks_per_bit;

    sm->run_hi[0] = 0;
    if (t == 0) {
        for (n = 1; n <= CAN_EDGE_RUN_MAX_BITS; n++)
            sm->run_hi[n] = 0;
        return;
    }
    for (n = 1; n <= CAN_EDGE_RUN_MAX_BITS; n++) {
        uint32_t nominal = n * t;
        uint32_t tol = nominal / CAN_EDGE_CLOCK_TOL_DIV +
                       t / CAN_EDGE_EDGE_SKEW_DIV + 1u;

        sm->run_hi[n] = nominal + tol;
    }
}

// Classify a run of identical bits from its elapsed time.  Returns 1..5 for a
// legal stuffed-region run, CAN_EDGE_RUN_OVERFLOW for an interval longer than
// five bits, or -1 when the interval does not fit any legal run window.
//
// Selection is a fixed chain of comparisons on the precomputed windows, so it
// has no division and no loop.  Only intervals that land in the gap between
// two windows reach the lower-bound check.
static int run_bits(const can_edge_sm_t *sm, uint32_t elapsed) {
    uint32_t n = 1u;

    if (elapsed > sm->run_hi[1])
        n = 2u;
    if (elapsed > sm->run_hi[2])
        n = 3u;
    if (elapsed > sm->run_hi[3])
        n = 4u;
    if (elapsed > sm->run_hi[4])
        n = 5u;
    if (elapsed > sm->run_hi[5])
        return (int)CAN_EDGE_RUN_OVERFLOW;

    // run_hi[n] = n*T + tol, so the symmetric lower edge is 2*n*T - hi.
    if (elapsed < 2u * n * sm->ticks_per_bit - sm->run_hi[n])
        return -1;
    return (int)n;
}

// An edge is a bit-boundary sync point.  In the stuffed region the interval
// since the previous edge is a whole number of bits, classified by its own
// delta time.  The phase is re-anchored at every edge, so neither oscillator
// tolerance nor the dominant/recessive edge skew accumulates across the frame.
static void advance_edge(can_edge_sm_t *sm, uint32_t now) {
    uint32_t elapsed;
    int n;
    uint32_t k;

    if (!sm->in_frame || sm->ticks_per_bit == 0) {
        sm->mark = now;
        sm->edge_mark = now;
        sm->run_done = 0;
        sm->accum = 0;
        return;
    }

    elapsed = elapsed_ticks(sm->edge_mark, now);
    n = run_bits(sm, elapsed);
    if (n < 0 || n > (int)CAN_EDGE_RUN_MAX_BITS) {
        // A stuffed-region run is never longer than five bits.  An interval
        // beyond that is a stuff violation; one that fits no window is a
        // generic bit error.
        fail(sm, (n > (int)CAN_EDGE_RUN_MAX_BITS) ? INVALID_STUFF
                                                  : INVALID_BIT);
        sm->mark = now;
        sm->edge_mark = now;
        sm->run_done = 0;
        sm->accum = 0;
        return;
    }

    // The quiet-tail path may already have emitted the leading bits of this
    // run while waiting for the next edge.  Emit only the bits that are still
    // missing, with the last one ending exactly at this edge.
    {
        uint32_t done = sm->run_done;
        if (done > (uint32_t)n)
            done = (uint32_t)n;
        for (k = done; k < (uint32_t)n && sm->in_frame; k++) {
            const uint32_t end =
                (now + ((uint32_t)n - 1u - k) * sm->ticks_per_bit) &
                CAN_EDGE_TIME_MASK;
            feed_raw(sm, sm->level, end);
        }
    }

    sm->mark = now;
    sm->edge_mark = now;
    sm->run_done = 0;
    sm->accum = 0;
}

// Emit whole bits of the current level up to `now`.  The fraction stays in
// accum.  Used for the quiet tail (delimiter/ACK/EOF), where the nominal bit
// period free-runs for at most the 10-bit interframe gap.  The stuffed region
// is driven by advance_edge() so a phase error never accumulates.
static void advance_to(can_edge_sm_t *sm, uint32_t now) {
    if (!sm->in_frame || sm->ticks_per_bit == 0) {
        sm->mark = now;
        sm->accum = 0;
        return;
    }

    while (sm->in_frame) {
        uint32_t elapsed = elapsed_ticks(sm->mark, now);
        uint32_t step;
        uint32_t end;

        if (sm->accum + elapsed < sm->ticks_per_bit)
            break;
        step = sm->ticks_per_bit - sm->accum;
        end = (sm->mark - step) & CAN_EDGE_TIME_MASK;
        sm->mark = end;
        sm->accum = 0;
        feed_raw(sm, sm->level, end);
        sm->run_done++;
    }

    if (!sm->in_frame) {
        sm->mark = now;
        sm->accum = 0;
        return;
    }
    sm->accum += elapsed_ticks(sm->mark, now);
    sm->mark = now;
}

static void start_frame(can_edge_sm_t *sm, uint32_t time) {
    uint32_t ticks = sm->ticks_per_bit;
#if CAN_ACK_GENERATION_SUPPORT
    uint32_t seq = sm->frame_seq + 1u;
#endif
    uint8_t keep_ready = sm->frame_ready;
    can_parsed_t keep_parsed = sm->parsed;
    frame_validity_t keep_validity = sm->validity;
    uint32_t run_hi[CAN_EDGE_RUN_MAX_BITS + 1u];

    memcpy(run_hi, sm->run_hi, sizeof(run_hi));
    memset(sm, 0, sizeof(*sm));
#if CAN_ACK_GENERATION_SUPPORT
    sm->frame_seq = seq;
#endif
    sm->ticks_per_bit = ticks;
    memcpy(sm->run_hi, run_hi, sizeof(run_hi));
    sm->mark = time & CAN_EDGE_TIME_MASK;
    sm->edge_mark = sm->mark;
    sm->run_done = 0;
    sm->level = 0;
    sm->in_frame = 1;
    sm->state = ST_SOF;
    sm->validity = INVALID_BIT;

    // A frame that ended just before this SOF must remain readable until
    // the caller consumes it, even though the parser is already in the
    // next frame.
    if (keep_ready) {
        sm->parsed = keep_parsed;
        sm->frame_ready = 1;
        sm->validity = keep_validity;
    }
}

void can_edge_sm_init(can_edge_sm_t *sm, uint32_t ticks_per_bit) {
    memset(sm, 0, sizeof(*sm));
    sm->ticks_per_bit = ticks_per_bit;
    run_windows_init(sm);
    sm->level = 1;
    sm->validity = INVALID_BIT;
}

void can_edge_sm_push(can_edge_sm_t *sm, uint32_t word) {
    uint32_t time = (word >> 1) & CAN_EDGE_TIME_MASK;
    uint8_t new_level = (uint8_t)(word & 1u);
    int was_in_frame;

    if (sm->ticks_per_bit == 0)
        return;

    if (!sm->in_frame) {
        if (sm->level == 1 && new_level == 0)
            start_frame(sm, time);
        else {
            sm->level = new_level;
            sm->mark = time;
        }
        return;
    }

    was_in_frame = 1;
    if (sm->state <= ST_CRC_DONE)
        advance_edge(sm, time);
    else
        advance_to(sm, time);
    if (sm->in_frame) {
        // Tail bits are judged at the nominal sample point.  An edge that
        // arrives before it defines the level sampled in that bit; an edge
        // after it belongs to the next bit (advance_to() may already have
        // moved tail_mark to `time`).
        if (sm->state == ST_DELIM || sm->state == ST_ACK ||
            sm->state == ST_ACKDEL) {
            uint32_t off = elapsed_ticks(sm->tail_mark, time);
            uint32_t sample = (sm->ticks_per_bit * 7u) / 8u;

            if (off <= sample)
                sm->tail_sample = new_level;
        }
#if CAN_ACK_GENERATION_SUPPORT
        sm->last_edge_raw = sm->raw_bits;
#endif
        sm->level = new_level;
        return;
    }
    // The catch-up finished this frame.  A dominant edge after a good
    // frame is the next SOF.  An edge that merely revealed an error is not.
    if (was_in_frame && sm->validity == VALID_FRAME &&
        sm->level == 1 && new_level == 0)
        start_frame(sm, time);
    else
        sm->level = new_level;
}

void can_edge_sm_time(can_edge_sm_t *sm, uint32_t now) {
    if (sm->ticks_per_bit == 0)
        return;
    advance_to(sm, now & CAN_EDGE_TIME_MASK);
}

static void pair_push(can_edge_pair_t *pair, uint16_t time16, uint8_t level) {
    if (pair->have_last) {
        uint32_t delta = (uint16_t)(pair->last - time16) + pair->bias;
        pair->time = (pair->time - delta) & CAN_EDGE_TIME_MASK;
    }
    pair->last = time16;
    pair->have_last = 1;
    can_edge_sm_push(pair->sm, can_edge_word(pair->time, level));
}

void can_edge_pair_init(can_edge_pair_t *pair, can_edge_sm_t *sm,
                        uint32_t now, uint32_t bias) {
    pair->sm = sm;
    pair->time = now & CAN_EDGE_TIME_MASK;
    pair->last = 0;
    pair->bias = bias;
    pair->have_last = 0;
}

#if CAN_EDGE_PAIR_TIME_RECORDING
#if CAN_EDGE_PAIR_RECORD_CAPACITY < 1
#error "CAN_EDGE_PAIR_RECORD_CAPACITY must be at least 1"
#endif

static can_edge_pair_record_t pair_records[CAN_EDGE_PAIR_RECORD_CAPACITY];
static volatile uint32_t pair_record_count;
static volatile uint32_t pair_record_dropped;

void can_edge_pair_record_clear(void) {
    pair_record_count = 0;
    pair_record_dropped = 0;
}

uint32_t can_edge_pair_record_count(void) {
    return pair_record_count;
}

uint32_t can_edge_pair_record_dropped(void) {
    return pair_record_dropped;
}

const can_edge_pair_record_t *can_edge_pair_record_get(uint32_t index) {
    if (index >= pair_record_count)
        return NULL;
    return &pair_records[index];
}

static void pair_record(uint32_t word, uint32_t fall_time,
                        uint32_t rise_time) {
    uint32_t i = pair_record_count;

    if (i >= CAN_EDGE_PAIR_RECORD_CAPACITY) {
        pair_record_dropped++;
        return;
    }
    pair_records[i].word = word;
    pair_records[i].fall_time = fall_time & CAN_EDGE_TIME_MASK;
    pair_records[i].rise_time = rise_time & CAN_EDGE_TIME_MASK;
    __sync_synchronize();
    pair_record_count = i + 1u;
}
#endif // CAN_EDGE_PAIR_TIME_RECORDING

void can_edge_pair_push(can_edge_pair_t *pair, uint32_t word) {
#if CAN_EDGE_PAIR_TIME_RECORDING
    uint32_t fall_time;
#endif

    pair_push(pair, (uint16_t)(word >> 16), 0);
#if CAN_EDGE_PAIR_TIME_RECORDING
    fall_time = pair->time;
#endif
    pair_push(pair, (uint16_t)word, 1);
#if CAN_EDGE_PAIR_TIME_RECORDING
    pair_record(word, fall_time, pair->time);
#endif
}

#if CAN_ACK_GENERATION_SUPPORT
uint8_t can_edge_sm_past_ack_slot(const can_edge_sm_t *sm) {
    return (uint8_t)(!sm->in_frame || sm->ack_seen);
}

#define CAN_ACK_MAX_WIRE 21u   // 15 CRC + up to 4 stuff + delimiter

can_ack_plan_t can_ack_plan(uint16_t crc, uint8_t run, uint8_t last,
                            uint32_t from_bit) {
    uint8_t wire[CAN_ACK_MAX_WIRE];
    can_ack_plan_t plan;
    uint32_t n = 0;
    uint32_t terminal = 0;       // last change at or before the slot
    uint32_t i;
    uint8_t level = (uint8_t)(last & 1u);
    uint32_t r = run;

    for (i = 0; i < 15u; i++) {
        uint8_t bit = (uint8_t)((crc >> (14u - i)) & 1u);

        // Same rule as the receive stuffer: a 5-run inserts the opposite
        // level and the run restarts at the stuff bit.
        if (r == 5u) {
            uint8_t s = (uint8_t)(1u - level);
            wire[n++] = s;
            r = 1;
            level = s;
        }
        wire[n++] = bit;
        if (r > 0u && bit == level)
            r++;
        else {
            r = 1;
            level = bit;
        }
    }
    // A CRC sequence that ends on a 5-run still owes one stuff bit before
    // the delimiter.  Stuffing stops there.
    if (r == 5u) {
        uint8_t s = (uint8_t)(1u - level);
        wire[n++] = s;
        r = 1;
        level = s;
    }
    wire[n++] = 1u;              // CRC delimiter, unstuffed

    // Terminal edge: the last level change at or before the slot boundary.
    // Index 0 is the change from `last` into the first CRC bit.
    for (i = 1; i < n; i++) {
        if (wire[i] != wire[i - 1u])
            terminal = i;
    }

    plan.slot_bit = n;
    plan.gap_bits = n - terminal;
    plan.edges = 0;
    for (i = 1; i < n; i++) {
        if (i > from_bit && wire[i] != wire[i - 1u])
            plan.edges++;
    }
    plan.level = (from_bit < n) ? wire[from_bit]
                                : 1u; // ACK slot is recessive before we drive
    plan.sync_level = (terminal == 0u) ? (uint8_t)(last & 1u)
                                      : wire[terminal - 1u];
    plan.ok = (from_bit < terminal) ? 1u : 0u;

    return plan;
}
#endif // CAN_ACK_GENERATION_SUPPORT
