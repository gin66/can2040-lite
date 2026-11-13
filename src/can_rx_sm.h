// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_rx_sm.h — minimal RX bit/unstuff parser for the edge-pair capture.
//
// The parser is split into three layers:
//   1. run classification: one edge-to-edge delta -> n identical bits
//   2. unstuffer/feeder: drop a stuff bit after a 5-run, emit data bits
//   3. protocol state machine: one uint8_t state, shift and CRC
//
// The state machine writes decoded fields directly into a ring slot.  The
// ring is 32 slots and keeps one empty, so 31 frames fit.  The producer
// advances rx_idx, the consumer advances tx_idx.  Invalid frames never
// advance rx_idx.

#ifndef CAN_RX_SM_H
#define CAN_RX_SM_H

#include <stdint.h>

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_RX_SLOTS 32u
#define CAN_RX_SLOT_MASK (CAN_RX_SLOTS - 1u)

// Decoded frame.  Field order is chosen for alignment:
//   id 0..3, data 4..11, dlc 12, flags 13, crc 14..15.
// Byte order is not the wire order; the parser writes fields as it decodes
// them.  flags bit 0 is IDE (0 = standard, 1 = extended); RTR is unsupported.
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    uint8_t flags;
    uint16_t crc;
} can_rx_frame_t;

typedef char can_rx_frame_size_check[(sizeof(can_rx_frame_t) == 16u) ? 1 : -1];

typedef enum {
    CAN_RX_NONE = 0,
    CAN_RX_FRAME = 1,
    CAN_RX_STUFF_ERROR = 2,
    CAN_RX_BIT_ERROR = 3,
    CAN_RX_CRC_ERROR = 4,
    CAN_RX_OVERFLOW = 5
} can_rx_event_t;

// The only per-frame parser state.  `shift` assembles one data byte; `state`
// encodes both the field and the bit index within it; `crc_calc` is the
// CRC-15 register over SOF..data.
typedef struct {
    uint8_t shift;
    uint8_t state;
    uint16_t crc_calc;
} can_rx_sm_t;

typedef struct {
    // Configuration.  A run of n bits is accepted when
    // run_lo[n] <= delta <= run_hi[n].  Both bounds are fixed at init so the
    // edge path is only compares.
    uint16_t ticks_per_bit;
    uint16_t run_hi[6];
    uint32_t run_lo[6];
    uint32_t ack_guard;

    // Edge/run layer.
    uint16_t prev_time16;
    uint8_t level;
    uint8_t drop_next;

    // Parser.
    can_rx_sm_t sm;

    // Single-producer/single-consumer ring.
    can_rx_frame_t frames[CAN_RX_SLOTS];
    can_rx_frame_t *write;
    volatile uint8_t rx_idx;
    volatile uint8_t tx_idx;

    // Error counters, kept in the ISR domain.
    uint32_t crc_errors;
    uint32_t stuff_errors;
    uint32_t bit_errors;
    uint32_t overflow;
} can_rx_t;

void can_rx_init(can_rx_t *rx, uint32_t ticks_per_bit);
void can_rx_reset(can_rx_t *rx);

// One captured edge.  time16 is the low 16 bits of the PIO down-counter,
// level is the pin level after the edge.  The first falling edge starts a
// frame; the next edge supplies the first run (which includes SOF).
can_rx_event_t can_rx_edge(can_rx_t *rx, uint16_t time16, uint8_t level);

// Feed one unstuffed run directly.  Used by tests and by can_rx_edge().
can_rx_event_t can_rx_feed(can_rx_t *rx, uint8_t level, uint8_t n);

// Number of CRC bits still owed when the parser is in the CRC field at
// recessive level and all remaining expected CRC bits are recessive.  Returns
// 0 when no edge-less CRC-tail wait is needed.  This is the exact deadline
// (in bit times) for completing the last 1..4 bits of a NACKed frame.
uint8_t can_rx_crc_tail_bits(const can_rx_t *rx);

// Complete a frame whose recessive CRC tail runs into the recessive
// delimiter with no further edge.  Only valid while the parser is in the CRC
// field at recessive level; any other quiet state is a stuff error.  Call
// only when the FIFO is empty and the bus has been quiet for the deadline
// from can_rx_crc_tail_bits().
can_rx_event_t can_rx_flush(can_rx_t *rx);

static inline uint8_t can_rx_available(const can_rx_t *rx) {
    return (uint8_t)(rx->tx_idx != rx->rx_idx);
}

static inline can_rx_frame_t *can_rx_front(can_rx_t *rx) {
    return &rx->frames[rx->tx_idx];
}

static inline void can_rx_pop(can_rx_t *rx) {
    rx->tx_idx = (uint8_t)((rx->tx_idx + 1u) & CAN_RX_SLOT_MASK);
}

#ifdef __cplusplus
}
#endif

#endif // CAN_RX_SM_H
