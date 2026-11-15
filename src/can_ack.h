// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_ack.h - expected stuffed CRC wire, computed once per frame.
//
// The CRC-15 over SOF..data is known when the parser reaches the first CRC
// data bit.  This module turns it into the exact wire levels the CRC field
// must have, from the first CRC data bit through the recessive CRC delimiter.
// It is pure: no PIO, no state, no pin access.  See docs/ack_generation.md.

#ifndef CAN_ACK_H
#define CAN_ACK_H

#include <stdint.h>

#include "can2040_lite_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CAN_ACK_GENERATION_SUPPORT

// 15 CRC data bits + up to 4 insertions + the trailing stuff bit + the
// recessive delimiter.
#define CAN_ACK_WIRE_MAX 21u

typedef struct {
    // Wire levels.  Time 0 is the first CRC data bit; `level[i]` is the
    // level of wire bit i.  `slot_bit` is the ACK slot boundary, one past
    // the recessive delimiter.
    uint8_t level[CAN_ACK_WIRE_MAX];
    uint8_t slot_bit;
    // Last 0->1 boundary strictly before the ACK slot.  This is the rising
    // edge that ends the last dominant wire bit of the CRC field, including
    // a trailing stuff bit.  It is 1..5 bit times before `slot_bit`.
    uint8_t last_rise;
    // Number of 0->1 boundaries in 1..last_rise.
    uint8_t rises;
    // `slot_bit - last_rise`, in bit times: 1..5.
    uint8_t r_tail;
    // A CRC that ends on a 5-run owes one trailing stuff bit.  Present is 1
    // and level is the wire level of that bit (0 = dominant, 1 = recessive).
    // A dominant trailing stuff bit is an anchor the normal run path cannot
    // see, because the tail walker owns it.
    uint8_t has_trailing_stuff;
    uint8_t trailing_stuff_level;
} can_ack_wire_t;

// Fill `wire` for the expected CRC field.  `run` and `last` are the stuffer
// state at the first CRC data bit: run is 1..4 and last is the level of the
// immediately preceding wire bit (1 = recessive).  A run of 5 inserts the
// opposite bit and restarts the run at 1; a CRC that ends on a 5-run still
// owes one trailing stuff bit before the delimiter.  The delimiter is
// recessive and unstuffed.
//
// `run` outside 1..4 returns `slot_bit == 0` (no prediction).
void can_ack_wire(uint16_t crc, uint8_t run, uint8_t last,
                  can_ack_wire_t *wire);

#endif // CAN_ACK_GENERATION_SUPPORT

#ifdef __cplusplus
}
#endif

#endif // CAN_ACK_H
