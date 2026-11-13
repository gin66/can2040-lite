// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_edge_sm.h — ISR state machine for the edge-timestamp CAN receiver.
//
// Portable: no PIO dependency.  The live firmware feeds it from the pair
// capture program through can_edge_pair_push().  This machine is what makes
// a one-bit ACK possible.  It is not wired to a pin, and it does not drive
// one.  A node must not ACK a frame it is transmitting; crc_match and the
// observed ACK slot are separate so the caller can tell those cases apart.
//
// can_edge_sm_push() takes one edge:
//   bits[31:1] = modulo-2^31 down-counter value at the edge
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
// The CRC delimiter, ACK slot, and ACK delimiter are sampled at the nominal
// 7/8 bit point, not at the bit boundary.  A dominant pulse must therefore
// still be present at the sample point: an ACK that ends early or starts too
// late is reported as a NACK, and one that reaches into either delimiter
// sample is a form error.
//
// Results stay readable until the caller clears frame_ready or calls
// can_edge_sm_init().  A new SOF starts the next frame but does not discard
// a result that has not been consumed yet.

#ifndef CAN_EDGE_SM_H
#define CAN_EDGE_SM_H

#include <stdint.h>

#include "can2040_lite_config.h"
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
#if CAN_ACK_GENERATION_SUPPORT
    uint32_t frame_seq;         // incremented at every SOF
#endif
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
    // Tail bits are sampled at the nominal sample point, not at the bit
    // boundary, so a dominant ACK pulse that ends early or starts late is
    // not mistaken for a valid ACK.
    uint32_t tail_mark;         // down-counter time at the current tail bit start
    uint8_t tail_sample;        // level at the current tail bit's sample point
#if CAN_ACK_GENERATION_SUPPORT
    uint32_t last_edge_raw;     // raw wire index of the bit at the last edge
#endif
    uint32_t stuffed_raw;
#if CAN_ACK_GENERATION_SUPPORT
    uint32_t crc_raw_start;     // raw wire index of the first CRC bit
    uint8_t crc_run_start;      // stuffer state entering the CRC field
    uint8_t crc_last_start;
    uint8_t crc_started;
#endif
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
#if CAN_ACK_GENERATION_SUPPORT
    uint8_t ack_armed;          // crc_match; ACK slot starts at ack_arm_time
    uint32_t ack_arm_time;
#endif
    uint8_t ack_seen;
    uint8_t ack;                // 1 = ACK slot was dominant
    uint8_t frame_ready;
    frame_validity_t validity;
    can_parsed_t parsed;
} can_edge_sm_t;

void can_edge_sm_init(can_edge_sm_t *sm, uint32_t ticks_per_bit);

// One edge: (time << 1) | level.  A 1→0 edge while idle is SOF.
void can_edge_sm_push(can_edge_sm_t *sm, uint32_t word);

// Advance a quiet bus to this down-counter value.  No new edge.
void can_edge_sm_time(can_edge_sm_t *sm, uint32_t now);

// Reassembles pair-capture words into single-edge pushes.  Each FIFO word
// carries a 1→0 timestamp in bits[31:16] and the following 0→1 timestamp in
// bits[15:0], both the low 16 bits of the PIO's two-cycle down-counter.
//
// Consecutive timestamps are differenced modulo 2^16 and carried on a 31-bit
// virtual counter; the first timestamp after init is taken as `now`.  A gap
// inside a frame is only a few bit times, so it never wraps.  The idle gap
// before SOF may wrap, but can_edge_sm rebases at that SOF, so a shift of the
// virtual counter by a multiple of 2^16 does not affect the frame.
//
// `bias` is added to every inter-edge distance.  The capture program's edge
// path skips one counter decrement, so each gap measures one tick short;
// pass 1 for the pair capture.  A source with an ideal free-running clock
// passes 0.
typedef struct {
    can_edge_sm_t *sm;
    uint32_t time;      // 31-bit down-counter value at the last edge
    uint16_t last;      // last 16-bit PIO timestamp
    uint32_t bias;      // ticks the capture path loses per edge
    uint8_t have_last;
} can_edge_pair_t;

void can_edge_pair_init(can_edge_pair_t *pair, can_edge_sm_t *sm,
                        uint32_t now, uint32_t bias);
void can_edge_pair_push(can_edge_pair_t *pair, uint32_t word);

#if CAN_EDGE_PAIR_TIME_RECORDING
// Capture log for CAN_EDGE_PAIR_TIME_RECORDING.  Every FIFO word consumed by
// can_edge_pair_push() is appended until the log fills; later words are
// counted in the dropped total and not stored.  can2040_lite_pico_rx_arm()
// clears the log, so an armed frame starts at its first edge.
//
// `word` is the raw FIFO entry.  `fall_time`/`rise_time` are the reconstructed
// 31-bit down-counter values of its 1->0 and 0->1 edges with the pair bias
// already applied; they are in the same domain as can_edge_sm_push().  A
// consumer can therefore rebuild the exact edge timeline and check, for
// example, where a generated ACK pulse sits relative to the ACK slot.
typedef struct {
    uint32_t word;
    uint32_t fall_time;
    uint32_t rise_time;
} can_edge_pair_record_t;

void can_edge_pair_record_clear(void);
uint32_t can_edge_pair_record_count(void);
uint32_t can_edge_pair_record_dropped(void);
const can_edge_pair_record_t *can_edge_pair_record_get(uint32_t index);
#endif // CAN_EDGE_PAIR_TIME_RECORDING

#if CAN_ACK_GENERATION_SUPPORT
// ACK schedule for the expected CRC field.  Pure: no PIO types, no state.
//
// `crc` is the 15 expected CRC bits, MSB first on the wire.  `run`/`last` are
// the stuffer state at the start of the CRC field (`last` 1 = recessive).
// `from_bit` is a wire index measured from the first CRC bit; 0 asks for the
// plan at the CRC-field boundary, and n>0 means wire bit n has been seen.
//
// The plan walks the CRC bits with the same stuffing rule as can_edge_sm.  A
// 5-run inserts the opposite bit and the run restarts at 1; a 5-run at the end
// of the CRC sequence inserts one trailing stuff bit before the delimiter.
//
// `slot_bit` is the wire index of the ACK slot relative to the first CRC bit.
// `edges` counts level changes strictly after `from_bit` through the terminal
// edge (the last change at or before the slot boundary).  `gap_bits` is the
// distance from the terminal edge to the slot boundary.  `level` is the pad
// level at `from_bit`; `sync_level` is the level just before the terminal
// edge.  `ok` is 0 when `from_bit` is already at or past the terminal edge.
typedef struct {
    uint32_t edges;
    uint32_t gap_bits;
    uint32_t slot_bit;
    uint8_t level;
    uint8_t sync_level;
    uint8_t ok;
} can_ack_plan_t;

can_ack_plan_t can_ack_plan(uint16_t crc, uint8_t run, uint8_t last,
                            uint32_t from_bit);

// True once the parser has finished the ACK slot (or the frame ended).  The
// ACK helper may restart its follow loop only after this.
uint8_t can_edge_sm_past_ack_slot(const can_edge_sm_t *sm);
#endif // CAN_ACK_GENERATION_SUPPORT

#ifdef __cplusplus
}
#endif

#endif // CAN_EDGE_SM_H
