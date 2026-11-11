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

enum {
    PH_STUFF = 1,
    PH_DELIM,
    PH_ACK,
    PH_ACKDEL,
    PH_EOF,
    PH_IFS,
    PH_ERROR
};

static uint32_t elapsed_ticks(uint32_t earlier, uint32_t later) {
    return (earlier - later) & CAN_EDGE_TIME_MASK;
}

static uint8_t hdr_bit(uint64_t hdr, uint32_t nbits, uint32_t index) {
    return (uint8_t)((hdr >> (nbits - 1u - index)) & 1u);
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
    sm->phase = PH_ERROR;
    publish(sm);
}

static void finish_ok(can_edge_sm_t *sm) {
    sm->validity = VALID_FRAME;
    sm->in_frame = 0;
    sm->phase = 0;
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

static uint32_t field_bits(uint64_t hdr, uint32_t nbits,
                           uint32_t lo, uint32_t hi) {
    uint32_t v = 0;
    uint32_t i;
    for (i = lo; i <= hi; i++)
        v = (v << 1) | hdr_bit(hdr, nbits, i);
    return v;
}

static void finish_header(can_edge_sm_t *sm) {
    uint32_t n = sm->header_len;
    uint32_t dlc;

    sm->header_bits = n;
    sm->header_known = 1;
    sm->is_ext = hdr_bit(sm->hdr, n, 13);
    if (sm->is_ext) {
        sm->id = (field_bits(sm->hdr, n, 1, 11) << 18) |
                 field_bits(sm->hdr, n, 14, 31);
        sm->is_rtr = hdr_bit(sm->hdr, n, 32);
        dlc = field_bits(sm->hdr, n, 35, 38);
    } else {
        sm->id = field_bits(sm->hdr, n, 1, 11);
        sm->is_rtr = hdr_bit(sm->hdr, n, 12);
        dlc = field_bits(sm->hdr, n, 15, 18);
    }
    if (dlc > 8)
        dlc = 8;
    sm->dlc = (uint8_t)dlc;
    sm->crc_start = sm->header_bits + (sm->is_rtr ? 0u : (uint32_t)sm->dlc * 8u);
}

// Stuffing has ended.  raw_bits includes the last stuffed bit.
static void enter_delim(can_edge_sm_t *sm) {
    sm->phase = PH_DELIM;
    sm->stuffed_raw = sm->raw_bits;
}

static void accept_unstuffed(can_edge_sm_t *sm, uint8_t bit, uint32_t bit_end) {
    int covered = !sm->header_known || sm->unstuffed < sm->crc_start;
    int last_crc;

    if (covered)
        crc_bit(sm, bit);
    else
        sm->crc_recv = (uint16_t)((sm->crc_recv << 1) | bit);

    if (!sm->header_known) {
        sm->hdr = (sm->hdr << 1) | bit;
        if (sm->unstuffed == 13)
            sm->header_len = bit ? 39u : 19u;
        if (sm->header_len != 0 && sm->unstuffed + 1u == sm->header_len)
            finish_header(sm);
    } else if (!sm->is_rtr && sm->unstuffed >= sm->header_bits &&
               sm->unstuffed < sm->crc_start) {
        uint32_t di = sm->unstuffed - sm->header_bits;
        sm->data[di >> 3] = (uint8_t)((sm->data[di >> 3] << 1) | bit);
    }

    note_run(sm, bit);
    last_crc = sm->header_known &&
               (sm->unstuffed + 1u == sm->crc_start + 15u);
    sm->unstuffed++;

    if (!last_crc)
        return;

    sm->crc_match = (uint8_t)(sm->crc_recv == sm->crc_calc);
    if (!sm->crc_match) {
        fail(sm, INVALID_CRC);
        return;
    }

    // The ACK slot is the bit after the delimiter.  A 5-run still owes
    // its stuff bit before that delimiter.
    {
        uint32_t ahead = (sm->run == 5) ? 2u : 1u;
        sm->ack_armed = 1;
        sm->ack_arm_time =
            (bit_end - ahead * sm->ticks_per_bit) & CAN_EDGE_TIME_MASK;
    }
    if (sm->run == 5)
        sm->need_stuff = 1;
    else
        enter_delim(sm);
}

static void feed_raw(can_edge_sm_t *sm, uint8_t bit, uint32_t bit_end) {
    if (!sm->in_frame)
        return;
    if (sm->raw_bits >= CAN2040_LITE_MAX_FRAME_BITS) {
        fail(sm, INVALID_BIT);
        return;
    }
    sm->raw_bits++;

    switch (sm->phase) {
    case PH_STUFF:
        if (sm->need_stuff || sm->run == 5) {
            if (bit == sm->last) {
                fail(sm, INVALID_STUFF);
                return;
            }
            sm->run = 1;
            sm->last = bit;
            if (sm->need_stuff) {
                sm->need_stuff = 0;
                enter_delim(sm);
            }
            return;
        }
        accept_unstuffed(sm, bit, bit_end);
        return;

    case PH_DELIM:
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        sm->phase = PH_ACK;
        return;

    case PH_ACK:
        sm->ack = (uint8_t)!bit;
        sm->ack_seen = 1;
        sm->phase = PH_ACKDEL;
        return;

    case PH_ACKDEL:
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        sm->phase = PH_EOF;
        sm->tail_got = 0;
        return;

    case PH_EOF:
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        if (++sm->tail_got == 7) {
            sm->phase = PH_IFS;
            sm->tail_got = 0;
        }
        return;

    case PH_IFS:
        if (!bit) {
            fail(sm, INVALID_BIT);
            return;
        }
        if (++sm->tail_got == 3)
            finish_ok(sm);
        return;

    default:
        fail(sm, INVALID_BIT);
        return;
    }
}

// Emit whole bits of the current level up to `now`.  The fraction stays
// in accum.  A frame that ends on the way (IFS done, or an error) leaves
// the leftover interval unparsed.
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
    uint8_t keep_ready = sm->frame_ready;
    can_parsed_t keep_parsed = sm->parsed;
    frame_validity_t keep_validity = sm->validity;

    memset(sm, 0, sizeof(*sm));
    sm->ticks_per_bit = ticks;
    sm->mark = time & CAN_EDGE_TIME_MASK;
    sm->level = 0;
    sm->in_frame = 1;
    sm->phase = PH_STUFF;
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
    advance_to(sm, time);
    if (sm->in_frame) {
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
