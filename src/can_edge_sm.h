// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_edge_sm.h — ISR state machine for the edge-timestamp CAN receiver.
//
// Portable: no PIO dependency.  The live firmware still samples with rx.pio.
// This machine is what makes a one-bit ACK possible.  It is not wired to a
// pin, and it does not drive one.  A node must not ACK a frame it is
// transmitting; crc_match and the observed ACK slot are separate so the
// caller can tell those cases apart.
//
// Each FIFO word is one edge from rx_edge.pio, shift-left, autopush 32:
//   bits[31:1] = X[30:0]   modulo-2^31 down-counter at the edge
//   bit[0]     = pin level after the edge
// X counts down.  Elapsed ticks are (earlier - later) mod 2^31.
//
// Whole bit times are taken from that gap and the fraction is kept, including
// across an edge, so the bit count follows the counter.  Bits completed
// before the edge use the previous level.  ticks_per_bit is in the same
// units as the 31-bit field (PIO cycles / 2).  These tests use ideal spacing;
// the four-cycle edge path is two counts short and is the capture driver's
// constant to add, not part of this machine.
//
// Stuffing applies from SOF through the CRC sequence, including the stuff
// bit inserted when that sequence ends on a 5-run.  It stops there.  The
// CRC delimiter, ACK slot, and EOF are not stuffed, so a recessive CRC tail
// may continue through the delimiter with no edge and must not be a stuff
// error.  A quiet tail pushes no FIFO word; can_edge_sm_time() advances it.
//
// crc_match and ack_armed become true on the 15th CRC bit, before the ACK
// slot.  ack_arm_time is the down-counter value at which the ACK slot starts
// (after a pending stuff bit, if the CRC ended on a 5-run, and after the
// CRC delimiter).  ack is the slot once ack_seen is set: 1 = dominant.
//
// Results stay until the next SOF edge or can_edge_sm_init().  Read them
// when frame_ready becomes 1, before feeding the next falling edge.

#ifndef CAN_EDGE_SM_H
#define CAN_EDGE_SM_H

#include <stdint.h>

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

// Low 31 bits of the edge word.  X is wider; the capture keeps X[30:0].
#define CAN_EDGE_TIME_MASK 0x7fffffffu

static inline uint32_t can_edge_word(uint32_t time31, uint8_t level) {
    return ((time31 & CAN_EDGE_TIME_MASK) << 1) | (uint32_t)(level & 1u);
}

typedef struct {
    // Configuration.  Must be at least 1.
    uint32_t ticks_per_bit;

    // Running capture.  mark/accum/level are the bit clock.
    uint32_t mark;
    uint32_t accum;
    uint8_t level;
    uint8_t in_frame;
    uint8_t phase;
    uint8_t need_stuff;

    // Unstuff and field parse.
    uint32_t run;
    uint8_t last;
    uint64_t hdr;
    uint32_t header_len;
    uint32_t header_bits;
    uint32_t unstuffed;
    uint32_t crc_start;
    uint32_t raw_bits;
    uint32_t stuffed_raw;
    uint8_t header_known;
    uint8_t is_ext;
    uint8_t is_rtr;
    uint8_t dlc;
    uint32_t id;
    uint8_t data[8];
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t tail_got;

    // Outputs.  Meaningful when the corresponding flag is set.
    uint8_t crc_match;          // 15 CRC bits are in and match crc_calc
    uint8_t ack_armed;          // crc_match; ACK slot starts at ack_arm_time
    uint32_t ack_arm_time;
    uint8_t ack_seen;
    uint8_t ack;                // 1 = ACK slot was dominant
    uint8_t frame_ready;
    frame_validity_t validity;
    can_parsed_t parsed;
} can_edge_sm_t;

void can_edge_sm_init(can_edge_sm_t *sm, uint32_t ticks_per_bit);

// One FIFO word: (time << 1) | level.  A 1→0 edge while idle is SOF.
void can_edge_sm_push(can_edge_sm_t *sm, uint32_t word);

// Advance a quiet bus to this down-counter value.  No new edge.
void can_edge_sm_time(can_edge_sm_t *sm, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif // CAN_EDGE_SM_H
