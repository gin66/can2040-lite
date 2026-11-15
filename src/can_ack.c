// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_ack.c - expected stuffed CRC wire.  Pure, but kept in SRAM on the Pico
// because the RX IRQ calls it at CRC entry.

#include "can_ack.h"
#include "can_bit_layout.h"

#if CAN_ACK_GENERATION_SUPPORT

#if defined(PICO_ON_DEVICE) || defined(ARDUINO_ARCH_RP2040)
#include "pico.h"
#define CAN_ACK_RAM(name) __not_in_flash_func(name)
#else
#define CAN_ACK_RAM(name) name
#endif

void CAN_ACK_RAM(can_ack_wire)(uint16_t crc, uint8_t run, uint8_t last,
                               can_ack_wire_t *wire) {
    uint8_t n = 0;
    uint8_t r = run;
    uint8_t level = (uint8_t)(last & 1u);
    uint8_t last_rise = 0;
    uint8_t rises = 0;
    uint8_t i;

    if (run < 1u || run > 4u) {
        wire->slot_bit = 0;
        wire->last_rise = 0;
        wire->rises = 0;
        wire->r_tail = 0;
        wire->has_trailing_stuff = 0;
        wire->trailing_stuff_level = 1;
        return;
    }
    wire->has_trailing_stuff = 0;
    wire->trailing_stuff_level = 1;

    for (i = 0; i < CAN_CRC_BITS; i++) {
        uint8_t bit = (uint8_t)((crc >> (14u - i)) & 1u);

        // Same rule as the receiver: a 5-run inserts the opposite level and
        // the run restarts at the stuff bit.
        if (r == 5u) {
            uint8_t s = (uint8_t)(1u - level);

            wire->level[n++] = s;
            r = 1;
            level = s;
        }
        wire->level[n++] = bit;
        if (bit == level) {
            r++;
        } else {
            r = 1;
            level = bit;
        }
    }

    // A CRC that ends on a 5-run still owes one stuff bit before the
    // delimiter.  Stuffing stops there.
    if (r == 5u) {
        uint8_t s = (uint8_t)(1u - level);

        wire->level[n++] = s;
        wire->has_trailing_stuff = 1;
        wire->trailing_stuff_level = s;
    }
    wire->level[n++] = 1u;              // CRC delimiter, unstuffed

    for (i = 1; i < n; i++) {
        if (wire->level[i - 1u] == 0u && wire->level[i] == 1u) {
            last_rise = i;
            rises++;
        }
    }

    wire->slot_bit = n;
    wire->last_rise = last_rise;
    wire->rises = rises;
    wire->r_tail = (uint8_t)(last_rise ? (n - last_rise) : 0u);
}

#endif // CAN_ACK_GENERATION_SUPPORT
