// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_bit_layout.h - unstuffed classic-CAN bit indices shared by the 11-bit
// and 29-bit frame layouts.
//
// Index 0 is the first base-ID bit; SOF is handled by the WAIT state, not
// indexed.  A standard frame is mapped onto the extended enumeration by
// skipping the extended-only region at IDE, so DLC, data, and CRC land on the
// same indices.  CAN_BIT_WAIT_FRAME is a special state value, not an index.

#ifndef CAN_BIT_LAYOUT_H
#define CAN_BIT_LAYOUT_H

// 29-bit frame, unstuffed, SOF excluded.
#define CAN_BIT_ID_BASE_FIRST   0u
#define CAN_BIT_ID_BASE_LAST    10u
#define CAN_BIT_SRR_RTR         11u
#define CAN_BIT_IDE             12u
#define CAN_BIT_ID_EXT_FIRST    13u
#define CAN_BIT_ID_EXT_LAST     30u
#define CAN_BIT_RTR_EXT         31u
#define CAN_BIT_R1              32u
// R0 and DLC start after a two-state gap so DLC ends on a byte boundary.
#define CAN_BIT_R0              35u
#define CAN_BIT_DLC_FIRST       36u
#define CAN_BIT_DLC_LAST        39u
#define CAN_BIT_DATA_FIRST      40u
#define CAN_BIT_DATA_LAST       103u
#define CAN_BIT_CRC_FIRST       104u
#define CAN_BIT_CRC_LAST        118u
// The edge-less no-ACK tail can start at most four CRC bits before the end:
// five trailing recessive CRC bits would have forced a dominant stuff bit.
#define CAN_BIT_CRC_TAIL_FIRST  (CAN_BIT_CRC_LAST - 3u)
#define CAN_BIT_CRC_DELIMITER   119u
#define CAN_BIT_ACK_SLOT        120u
#define CAN_BIT_ACK_DELIMITER   121u
#define CAN_BIT_EOF_START       122u
#define CAN_BIT_EOF_LAST        128u
#define CAN_BIT_COUNT           129u

// Special state: init value and the error/abort value for a bad bit time.
#define CAN_BIT_WAIT_FRAME      255u

// Field widths.
#define CAN_ID_BASE_BITS        11u
#define CAN_ID_EXT_BITS         18u
#define CAN_DLC_BITS            4u
#define CAN_DATA_SLOTS          64u
#define CAN_BITS_PER_BYTE       8u
#define CAN_CRC_BITS            15u
#define CAN_DLC_MAX             8u

// Standard frame: skip ext-ID + ext-RTR + r1 at IDE, landing on R0.
#define CAN_IDE_STD_SKIP        (CAN_BIT_R0 - (CAN_BIT_IDE + 1u))

// Standard-frame wire positions (shared layout minus the IDE skip).
#define CAN_BIT_STD_R0          (CAN_BIT_R0 - CAN_IDE_STD_SKIP)
#define CAN_BIT_STD_DLC_FIRST   (CAN_BIT_DLC_FIRST - CAN_IDE_STD_SKIP)
#define CAN_BIT_STD_DATA_FIRST  (CAN_BIT_DATA_FIRST - CAN_IDE_STD_SKIP)

// Last used data index for `dlc` bytes, and the jump from there to CRC.
#define CAN_DATA_LAST_IDX(dlc)  (CAN_BIT_DLC_LAST + CAN_BITS_PER_BYTE * (dlc))
#define CAN_DATA_SKIP(dlc)      (CAN_BITS_PER_BYTE * (CAN_DLC_MAX - (dlc)))
#define CAN_DATA_DELTA(dlc)     (CAN_DATA_SKIP(dlc) + 1u)

#endif // CAN_BIT_LAYOUT_H
