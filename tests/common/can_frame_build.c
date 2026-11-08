// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_frame_build.c — Host-side CAN 2.0B frame builder and defect injector.

#include <string.h>

#include "can_frame_build.h"

static void put_bits(uint8_t *buf, uint32_t *pos, uint32_t value,
                     uint32_t nbits) {
    uint32_t i;
    for (i = 0; i < nbits; i++)
        bits_put(buf, (*pos)++, (uint8_t)((value >> (nbits - 1 - i)) & 1u));
}

static int stuffing_ok(const uint8_t *bits, uint32_t nbits) {
    uint32_t run = 0;
    uint8_t last = 0;
    uint32_t i;
    for (i = 0; i < nbits; i++) {
        uint8_t bit = bits_get(bits, i);
        if (run > 0 && bit == last) {
            run++;
            if (run > 5)
                return 0;
        } else {
            run = 1;
            last = bit;
        }
    }
    return 1;
}

static int prefix_stuff_valid(const uint8_t *un, uint32_t nbits,
                              uint32_t max_bits) {
    uint8_t tmp[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t len;

    if (max_bits > CAN2040_LITE_MAX_FRAME_BITS)
        max_bits = CAN2040_LITE_MAX_FRAME_BITS;
    len = bit_stuff(un, nbits, tmp, max_bits);
    return len != CAN_BITS_ERROR && stuffing_ok(tmp, len);
}

static uint32_t build_prefix(const can_frame_t *frame,
                             frame_validity_t validity, uint8_t *bits,
                             uint32_t max_bits, uint32_t *unstuffed_bits,
                             uint16_t *crc_out) {
    uint8_t un[CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t pos = 0;
    uint32_t header_bits;
    uint32_t data_bits;
    uint32_t crc_start;
    uint32_t i;
    uint16_t crc;
    int ext = (frame->flags & CAN_FRAME_EXTENDED) != 0;
    int rtr = (frame->flags & CAN_FRAME_RTR) != 0;

    bits_clear(un, sizeof(un) * 8u);

    // SOF
    pos = 0;
    bits_put(un, pos++, 0);

    if (ext) {
        put_bits(un, &pos, (frame->id >> 18) & 0x7ff, 11); // base ID
        bits_put(un, pos++, 1);                            // SRR
        bits_put(un, pos++, 1);                            // IDE
        put_bits(un, &pos, frame->id & 0x3ffff, 18);       // extended ID
        bits_put(un, pos++, rtr ? 1 : 0);                  // RTR
        bits_put(un, pos++, 0);                            // r1
        bits_put(un, pos++, 0);                            // r0
    } else {
        put_bits(un, &pos, frame->id & 0x7ff, 11);        // ID
        bits_put(un, pos++, rtr ? 1 : 0);                  // RTR
        bits_put(un, pos++, 0);                            // IDE
        bits_put(un, pos++, 0);                            // r0
    }
    put_bits(un, &pos, frame->dlc & 0x0f, 4);              // DLC

    header_bits = pos;
    data_bits = rtr ? 0 : (uint32_t)frame->dlc * 8u;
    for (i = 0; i < data_bits; i++)
        bits_put(un, pos++, (uint8_t)((frame->data[i / 8] >>
                                       (7 - (i % 8))) & 1u));

    crc_start = pos;
    crc = crc_can15_bits(0, un, crc_start);

    if (validity == INVALID_CRC) {
        // Flip one CRC bit.  Pick a flip whose stuffed stream has no 6-run so
        // the receiver reports a CRC error, not a stuff error.
        uint16_t chosen = (uint16_t)(crc ^ 0x0080u);
        uint32_t candidate;
        for (candidate = 0; candidate < 15; candidate++) {
            uint16_t bad = (uint16_t)(crc ^ (uint16_t)(1u << candidate));
            for (i = 0; i < 15; i++)
                bits_put(un, crc_start + i,
                         (uint8_t)((bad >> (14 - i)) & 1u));
            if (prefix_stuff_valid(un, crc_start + 15, max_bits)) {
                chosen = bad;
                break;
            }
        }
        crc = chosen;
    }

    // Append the 15-bit CRC sequence (MSB first).
    for (i = 0; i < 15; i++)
        bits_put(un, crc_start + i, (uint8_t)((crc >> (14 - i)) & 1u));
    pos = crc_start + 15;

    if (validity == INVALID_BIT) {
        // Flip one payload/header bit after the CRC was computed.  Keep the
        // stuffed stream valid so the defect is reported as a CRC error.
        uint32_t first = data_bits > 0 ? header_bits : 1;
        uint32_t last = data_bits > 0 ? header_bits + data_bits : header_bits;
        uint32_t chosen = first;
        uint32_t candidate;
        int found = 0;
        for (candidate = first; candidate < last; candidate++) {
            bits_put(un, candidate, (uint8_t)!bits_get(un, candidate));
            if (prefix_stuff_valid(un, pos, max_bits)) {
                chosen = candidate;
                found = 1;
                break;
            }
            bits_put(un, candidate, (uint8_t)!bits_get(un, candidate));
        }
        if (!found)
            bits_put(un, chosen, (uint8_t)!bits_get(un, chosen));
    }

    // Recover the CRC actually carried (it differs for INVALID_CRC).
    crc = 0;
    for (i = 0; i < 15; i++)
        crc = (uint16_t)((crc << 1) | bits_get(un, crc_start + i));

    *unstuffed_bits = pos;
    *crc_out = crc;

    return (uint32_t)bit_stuff(un, pos, bits, max_bits);
}

uint32_t can_frame_build(const can_frame_t *frame, frame_validity_t validity,
                         int ack, uint8_t *bits, uint32_t max_bits,
                         can_build_info_t *info) {
    uint32_t stuffed_bits;
    uint32_t total;
    uint32_t unstuffed_bits;
    uint16_t crc;
    int ext;

    if (!frame || !bits || frame->dlc > 8 || max_bits < 14)
        return 0;

    ext = (frame->flags & CAN_FRAME_EXTENDED) != 0;
    if (ext ? (frame->id > 0x1fffffffu) : (frame->id > 0x7ffu))
        return 0;

    // Reserve room for the 13 unstuffed tail bits.
    stuffed_bits = build_prefix(frame, validity, bits, max_bits - 13u,
                                &unstuffed_bits, &crc);
    if (stuffed_bits == CAN_BITS_ERROR || stuffed_bits == 0)
        return 0;

    if (validity == INVALID_STUFF) {
        uint32_t run = 0;
        uint8_t last = 0;
        uint32_t i;
        int done = 0;
        for (i = 0; i < stuffed_bits && !done; i++) {
            uint8_t bit = bits_get(bits, i);
            if (run > 0 && bit == last) {
                run++;
                if (run == 5) {
                    if (i + 1 >= stuffed_bits)
                        return 0;
                    // The next bit is the inserted stuff bit; flip it so it
                    // joins the run and produces 6 identical bits.
                    bits_put(bits, i + 1, (uint8_t)!bits_get(bits, i + 1));
                    done = 1;
                }
            } else {
                run = 1;
                last = bit;
            }
        }
        if (!done)
            return 0;
    }

    total = stuffed_bits;
    bits_put(bits, total++, 1);           // CRC delimiter
    bits_put(bits, total++, ack ? 0 : 1); // ACK slot (dominant = ACK)
    bits_put(bits, total++, 1);           // ACK delimiter
    {
        uint32_t i;
        for (i = 0; i < 7; i++)
            bits_put(bits, total++, 1);   // EOF
        for (i = 0; i < 3; i++)
            bits_put(bits, total++, 1);   // IFS
    }

    if (info) {
        info->validity = validity;
        info->crc = crc;
        info->unstuffed_bits = unstuffed_bits;
        info->stuffed_bits = stuffed_bits;
        info->total_bits = total;
        info->ack = (uint8_t)(ack ? 1 : 0);
    }
    return total;
}
