// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_frame_build.h — Host-side CAN 2.0B frame builder and defect injector.

#ifndef CAN_FRAME_BUILD_H
#define CAN_FRAME_BUILD_H

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

// Frame flags
#define CAN_FRAME_EXTENDED 0x01u
#define CAN_FRAME_RTR      0x02u

// An application-level CAN frame to build.
typedef struct {
    uint32_t id;          // 11-bit or 29-bit identifier
    uint8_t dlc;          // 0..8
    uint8_t flags;        // CAN_FRAME_EXTENDED | CAN_FRAME_RTR
    uint8_t data[8];      // payload (DLC bytes, MSB-first on the wire)
} can_frame_t;

// Metadata describing a built frame.
typedef struct {
    frame_validity_t validity;
    uint16_t crc;            // CRC value placed in the frame
    uint32_t unstuffed_bits; // SOF..CRC sequence, before stuffing
    uint32_t stuffed_bits;   // SOF..CRC sequence, after stuffing
    uint32_t total_bits;     // SOF..IFS
    uint8_t ack;             // 1 if the ACK slot was made dominant
} can_build_info_t;

// Build a complete on-wire frame into `bits`: the stuffed prefix
// (SOF..CRC sequence), then the unstuffed tail (CRC delimiter, ACK slot,
// ACK delimiter, EOF, IFS).
//
// `validity` selects the injected defect:
//   VALID_FRAME   - correct frame
//   INVALID_CRC   - one CRC bit flipped before stuffing
//   INVALID_BIT   - one payload/header bit flipped after the CRC was computed
//                   (so the CRC no longer matches)
//   INVALID_STUFF - one inserted stuff bit flipped, producing a 6-bit run
//
// `ack` makes the ACK slot dominant (1) or recessive (0).
// Returns total bits written (SOF..IFS), or 0 on error / insufficient space.
uint32_t can_frame_build(const can_frame_t *frame, frame_validity_t validity,
                         int ack, uint8_t *bits, uint32_t max_bits,
                         can_build_info_t *info);

#ifdef __cplusplus
}
#endif

#endif // CAN_FRAME_BUILD_H
