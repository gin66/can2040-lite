// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_proto.c — CAN 2.0B bit helpers, CRC-15, stuffing and decode.

#include <string.h>

#include "can2040_lite_proto.h"

/****************************************************************
 * CRC-15 (matches can2040's crc_byte recurrence)
 ****************************************************************/

uint16_t crc_can15_bits(uint16_t crc, const uint8_t *bits, uint32_t num_bits) {
    uint32_t i;
    for (i = 0; i < num_bits; i++) {
        uint16_t bit = bits_get(bits, i);
        uint16_t mix = (uint16_t)(((crc >> 14) ^ bit) & 1u);
        crc = (uint16_t)((crc << 1) & 0x7fff);
        if (mix)
            crc ^= 0x4599;
    }
    return crc;
}

/****************************************************************
 * Bit helpers (MSB-first)
 ****************************************************************/

uint8_t bits_get(const uint8_t *buf, uint32_t pos) {
    return (uint8_t)((buf[pos >> 3] >> (7 - (pos & 7))) & 1u);
}

void bits_put(uint8_t *buf, uint32_t pos, uint8_t bit) {
    uint8_t mask = (uint8_t)(1u << (7 - (pos & 7)));
    if (bit)
        buf[pos >> 3] |= mask;
    else
        buf[pos >> 3] &= (uint8_t)~mask;
}

void bits_clear(uint8_t *buf, uint32_t nbits) {
    memset(buf, 0, (nbits + 7u) / 8u);
}

/****************************************************************
 * Bit stuffing / unstuffing
 ****************************************************************/

uint32_t bit_stuff(const uint8_t *data, uint32_t data_bits,
                   uint8_t *stuffed, uint32_t max_stuffed_bits) {
    uint32_t si = 0;
    uint32_t run = 0;
    uint32_t i;
    uint8_t last = 0;

    for (i = 0; i < data_bits; i++) {
        uint8_t bit = bits_get(data, i);

        if (si >= max_stuffed_bits)
            return CAN_BITS_ERROR;
        bits_put(stuffed, si++, bit);

        if (run > 0 && bit == last) {
            run++;
            if (run == 5) {
                if (si >= max_stuffed_bits)
                    return CAN_BITS_ERROR;
                last = (uint8_t)!bit;
                bits_put(stuffed, si++, last);
                run = 1;
            }
        } else {
            run = 1;
            last = bit;
        }
    }
    return si;
}
