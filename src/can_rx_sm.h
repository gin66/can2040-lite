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
#include "can_bit_layout.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_RX_SLOTS 32u
#define CAN_RX_SLOT_MASK (CAN_RX_SLOTS - 1u)

// Raw receive slot.  At IDE the parser jumps the store position so R0/DLC are
// at 35/36-39 for both frames.  DLC is in byte 4.  Data is right-aligned and
// always ends at CAN_BIT_DATA_LAST, so the CRC field is always stored at the
// fixed CAN_BIT_CRC_FIRST position (bytes 13-14); DLC only selects the data
// start.  `flags` bit 0 is IDE.
#define CAN_RX_RAW_BYTES 16u

typedef struct {
    uint8_t raw[CAN_RX_RAW_BYTES];
    uint8_t flags;
} can_rx_frame_t;

static inline uint8_t can_rx_raw_bit(const can_rx_frame_t *f, uint32_t pos) {
    return (uint8_t)((f->raw[pos >> 3] >> (7u - (pos & 7u))) & 1u);
}

static inline uint32_t can_rx_frame_id(const can_rx_frame_t *f) {
    uint32_t ide = (uint32_t)(f->flags & 1u);
    uint32_t base = ((uint32_t)f->raw[0] << 3) |
                    ((uint32_t)(f->raw[1] >> (ide ? 5u : 2u)) & 7u);
    uint32_t ext = 0;
    uint32_t i;

    if (!ide)
        return base;
    for (i = CAN_BIT_ID_EXT_FIRST; i <= CAN_BIT_ID_EXT_LAST; i++)
        ext = (ext << 1) | can_rx_raw_bit(f, i);
    return (base << 18) | ext;
}

static inline uint8_t can_rx_frame_flags(const can_rx_frame_t *f) {
    return (uint8_t)(f->flags & 1u);
}

static inline uint8_t can_rx_frame_dlc(const can_rx_frame_t *f) {
    return (uint8_t)(f->raw[CAN_BIT_DLC_FIRST >> 3] & 0x0fu);
}

// Right-aligned data: the used bytes end at CAN_BIT_DATA_LAST, so the first
// used byte is (CAN_BIT_CRC_FIRST >> 3) - dlc.
static inline void can_rx_frame_data(const can_rx_frame_t *f, uint8_t *out) {
    uint8_t dlc = can_rx_frame_dlc(f);
    uint32_t first;
    uint32_t b;

    if (dlc > CAN_DLC_MAX)
        dlc = CAN_DLC_MAX;
    first = (CAN_BIT_CRC_FIRST >> 3) - dlc;
    for (b = 0; b < 8u; b++)
        out[b] = 0;
    for (b = 0; b < dlc; b++)
        out[8u - dlc + b] = f->raw[first + b];
}

typedef char can_rx_frame_size_check[(sizeof(can_rx_frame_t) == 17u) ? 1 : -1];
// The parser stores every bit through ACK_SLOT without a range check.
typedef char can_rx_raw_fits_check[(CAN_BIT_ACK_SLOT < CAN_RX_RAW_BYTES * 8u) ?
                                    1 : -1];

typedef enum {
    CAN_RX_NONE = 0,
    CAN_RX_FRAME = 1,
    CAN_RX_STUFF_ERROR = 2,
    CAN_RX_BIT_ERROR = 3,
    CAN_RX_CRC_ERROR = 4,
    CAN_RX_OVERFLOW = 5
} can_rx_event_t;

// `crc_calc` is the CRC-15 register over SOF..data.
typedef struct {
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
    uint8_t drop_next;

    // Parser.
    uint8_t pos;            // unstuffed wire-bit position; 255 = idle
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
    uint32_t ack_errors;
    uint32_t overflow;
} can_rx_t;

void can_rx_init(can_rx_t *rx, uint32_t ticks_per_bit);
void can_rx_reset(can_rx_t *rx);

// One captured edge.  time16 is the low 16 bits of the PIO down-counter,
// level is the pin level after the edge.  The first falling edge starts a


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
