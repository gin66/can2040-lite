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

// Generated for polynomial 0x4599 (reflected), MSB-first, zero initial value.
static const uint16_t crc_can15_table[256] = {
    0x0000,0x4599,0x4eab,0x0b32,0x58cf,0x1d56,0x1664,0x53fd,0x7407,0x319e,
    0x3aac,0x7f35,0x2cc8,0x6951,0x6263,0x27fa,0x2d97,0x680e,0x633c,0x26a5,
    0x7558,0x30c1,0x3bf3,0x7e6a,0x5990,0x1c09,0x173b,0x52a2,0x015f,0x44c6,
    0x4ff4,0x0a6d,0x5b2e,0x1eb7,0x1585,0x501c,0x03e1,0x4678,0x4d4a,0x08d3,
    0x2f29,0x6ab0,0x6182,0x241b,0x77e6,0x327f,0x394d,0x7cd4,0x76b9,0x3320,
    0x3812,0x7d8b,0x2e76,0x6bef,0x60dd,0x2544,0x02be,0x4727,0x4c15,0x098c,
    0x5a71,0x1fe8,0x14da,0x5143,0x73c5,0x365c,0x3d6e,0x78f7,0x2b0a,0x6e93,
    0x65a1,0x2038,0x07c2,0x425b,0x4969,0x0cf0,0x5f0d,0x1a94,0x11a6,0x543f,
    0x5e52,0x1bcb,0x10f9,0x5560,0x069d,0x4304,0x4836,0x0daf,0x2a55,0x6fcc,
    0x64fe,0x2167,0x729a,0x3703,0x3c31,0x79a8,0x28eb,0x6d72,0x6640,0x23d9,
    0x7024,0x35bd,0x3e8f,0x7b16,0x5cec,0x1975,0x1247,0x57de,0x0423,0x41ba,
    0x4a88,0x0f11,0x057c,0x40e5,0x4bd7,0x0e4e,0x5db3,0x182a,0x1318,0x5681,
    0x717b,0x34e2,0x3fd0,0x7a49,0x29b4,0x6c2d,0x671f,0x2286,0x2213,0x678a,
    0x6cb8,0x2921,0x7adc,0x3f45,0x3477,0x71ee,0x5614,0x138d,0x18bf,0x5d26,
    0x0edb,0x4b42,0x4070,0x05e9,0x0f84,0x4a1d,0x412f,0x04b6,0x574b,0x12d2,
    0x19e0,0x5c79,0x7b83,0x3e1a,0x3528,0x70b1,0x234c,0x66d5,0x6de7,0x287e,
    0x793d,0x3ca4,0x3796,0x720f,0x21f2,0x646b,0x6f59,0x2ac0,0x0d3a,0x48a3,
    0x4391,0x0608,0x55f5,0x106c,0x1b5e,0x5ec7,0x54aa,0x1133,0x1a01,0x5f98,
    0x0c65,0x49fc,0x42ce,0x0757,0x20ad,0x6534,0x6e06,0x2b9f,0x7862,0x3dfb,
    0x36c9,0x7350,0x51d6,0x144f,0x1f7d,0x5ae4,0x0919,0x4c80,0x47b2,0x022b,
    0x25d1,0x6048,0x6b7a,0x2ee3,0x7d1e,0x3887,0x33b5,0x762c,0x7c41,0x39d8,
    0x32ea,0x7773,0x248e,0x6117,0x6a25,0x2fbc,0x0846,0x4ddf,0x46ed,0x0374,
    0x5089,0x1510,0x1e22,0x5bbb,0x0af8,0x4f61,0x4453,0x01ca,0x5237,0x17ae,
    0x1c9c,0x5905,0x7eff,0x3b66,0x3054,0x75cd,0x2630,0x63a9,0x689b,0x2d02,
    0x276f,0x62f6,0x69c4,0x2c5d,0x7fa0,0x3a39,0x310b,0x7492,0x5368,0x16f1,
    0x1dc3,0x585a,0x0ba7,0x4e3e,0x450c,0x0095
};

uint16_t crc_can15_update(uint16_t crc, uint8_t data) {
    return (uint16_t)(((crc << 8) ^ crc_can15_table[((crc >> 7) ^ data) & 0xff])
                      & 0x7fff);
}

uint16_t crc_can15_calc(const uint8_t *data, uint32_t num_bytes) {
    uint16_t crc = 0;
    uint32_t i;
    for (i = 0; i < num_bytes; i++)
        crc = crc_can15_update(crc, data[i]);
    return crc;
}

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

// Streaming unstuffer.  Returns 0 on success, -1 when the stream is truncated
// or contains a stuff violation.  `error` distinguishes the two.
typedef struct {
    const uint8_t *buf;
    uint32_t len;
    uint32_t pos;
    uint32_t run;
    uint8_t last;
    int error;
} unstuffer_t;

static void unstuffer_init(unstuffer_t *u, const uint8_t *buf, uint32_t len) {
    u->buf = buf;
    u->len = len;
    u->pos = 0;
    u->run = 0;
    u->last = 0;
    u->error = 0;
}

static int unstuff_next(unstuffer_t *u, uint8_t *out) {
    while (u->pos < u->len) {
        uint8_t bit = bits_get(u->buf, u->pos++);

        if (u->run == 5) {
            if (bit == u->last) {
                u->error = 1;
                return -1;
            }
            // This is a stuff bit: discard it, but it starts the new run
            // (the transmitter counts it the same way).
            u->run = 1;
            u->last = bit;
            continue;
        }

        *out = bit;
        if (u->run > 0 && bit == u->last)
            u->run++;
        else {
            u->run = 1;
            u->last = bit;
        }
        return 0;
    }
    return -1;
}

// Unstuff exactly `count` bits from the current scanner position.
static frame_validity_t unstuff_n(unstuffer_t *u, uint8_t *out,
                                  uint32_t start, uint32_t count) {
    uint32_t i;
    uint8_t bit;
    for (i = 0; i < count; i++) {
        if (unstuff_next(u, &bit) != 0)
            return u->error ? INVALID_STUFF : INVALID_BIT;
        bits_put(out, start + i, bit);
    }
    return VALID_FRAME;
}

uint32_t bit_unstuff(const uint8_t *stuffed, uint32_t stuffed_bits,
                     uint8_t *unstuffed, uint32_t max_unstuffed_bits) {
    unstuffer_t u;
    uint32_t di = 0;
    uint8_t bit;

    unstuffer_init(&u, stuffed, stuffed_bits);
    while (di < max_unstuffed_bits) {
        if (unstuff_next(&u, &bit) != 0)
            break;
        bits_put(unstuffed, di++, bit);
    }

    if (u.error)
        return CAN_BITS_ERROR;
    if (di >= max_unstuffed_bits && u.pos < u.len)
        return CAN_BITS_ERROR;
    // A complete stuffed region never ends on a 5-run: the encoder must have
    // inserted the opposite stuff bit after it.
    if (u.run == 5)
        return CAN_BITS_ERROR;
    return di;
}

/****************************************************************
 * Frame decode
 ****************************************************************/

frame_validity_t can_frame_decode(const uint8_t *bits, uint32_t total_bits,
                                  can_parsed_t *out) {
    uint8_t un[CAN2040_LITE_MAX_FRAME_BYTES];
    unstuffer_t u;
    frame_validity_t res;
    uint32_t header_bits;
    uint32_t data_bits;
    uint32_t crc_start;
    uint32_t i;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint32_t p;
    int ext;
    int rtr;
    uint32_t id;
    uint8_t dlc;

    if (!out || !bits || total_bits == 0)
        return INVALID_BIT;

    memset(out, 0, sizeof(*out));
    bits_clear(un, sizeof(un) * 8u);
    unstuffer_init(&u, bits, total_bits);

    // Unstuff the first 19 bits: enough to see IDE at bit 13 for both formats.
    res = unstuff_n(&u, un, 0, 19);
    if (res != VALID_FRAME) {
        out->validity = res;
        return res;
    }
    ext = bits_get(un, 13) != 0;

    if (ext) {
        header_bits = 39;
        bits_clear(un, sizeof(un) * 8u);
        unstuffer_init(&u, bits, total_bits);
        res = unstuff_n(&u, un, 0, header_bits);
        if (res != VALID_FRAME) {
            out->validity = res;
            return res;
        }
        {
            uint32_t base = 0;
            uint32_t ext_id = 0;
            for (i = 1; i <= 11; i++)
                base = (base << 1) | bits_get(un, i);
            for (i = 14; i <= 31; i++)
                ext_id = (ext_id << 1) | bits_get(un, i);
            id = (base << 18) | ext_id;
            rtr = bits_get(un, 32);
            dlc = 0;
            for (i = 35; i <= 38; i++)
                dlc = (uint8_t)((dlc << 1) | bits_get(un, i));
        }
    } else {
        header_bits = 19;
        {
            uint32_t std_id = 0;
            for (i = 1; i <= 11; i++)
                std_id = (std_id << 1) | bits_get(un, i);
            id = std_id;
            rtr = bits_get(un, 12);
            dlc = 0;
            for (i = 15; i <= 18; i++)
                dlc = (uint8_t)((dlc << 1) | bits_get(un, i));
        }
    }

    if (dlc > 8)
        dlc = 8;
    data_bits = rtr ? 0 : (uint32_t)dlc * 8u;
    crc_start = header_bits + data_bits;

    res = unstuff_n(&u, un, header_bits, data_bits + 15);
    if (res != VALID_FRAME) {
        out->validity = res;
        out->id = id;
        out->dlc = dlc;
        out->is_extended = (uint8_t)ext;
        out->is_rtr = (uint8_t)rtr;
        return res;
    }

    if (!rtr) {
        uint32_t byte;
        for (byte = 0; byte < dlc; byte++) {
            uint8_t v = 0;
            uint32_t b;
            for (b = 0; b < 8; b++)
                v = (uint8_t)((v << 1) |
                              bits_get(un, header_bits + byte * 8u + b));
            out->data[byte] = v;
        }
    }

    // If the CRC sequence ended on a 5-run, consume the inserted stuff bit.
    if (u.run == 5) {
        uint8_t stuff_bit;
        if (u.pos >= u.len) {
            out->validity = INVALID_STUFF;
            return INVALID_STUFF;
        }
        stuff_bit = bits_get(bits, u.pos);
        if (stuff_bit == u.last) {
            out->validity = INVALID_STUFF;
            return INVALID_STUFF;
        }
        u.pos++;
    }

    crc_calc = crc_can15_bits(0, un, crc_start);
    crc_recv = 0;
    for (i = 0; i < 15; i++)
        crc_recv = (uint16_t)((crc_recv << 1) | bits_get(un, crc_start + i));

    out->id = id;
    out->dlc = dlc;
    out->is_extended = (uint8_t)ext;
    out->is_rtr = (uint8_t)rtr;
    out->crc = crc_recv;
    out->crc_calc = crc_calc;
    out->stuffed_bits = u.pos;

    if (crc_recv != crc_calc) {
        out->validity = INVALID_CRC;
        return INVALID_CRC;
    }

    // Unstuffed tail: CRC delimiter, ACK slot, ACK delimiter, EOF, IFS.
    p = u.pos;
    if (p >= total_bits) {
        out->validity = INVALID_BIT;
        return INVALID_BIT;
    }
    if (!bits_get(bits, p++)) {          // CRC delimiter must be recessive
        out->validity = INVALID_BIT;
        return INVALID_BIT;
    }
    out->ack = (uint8_t)(!bits_get(bits, p++)); // dominant = ACK
    if (!bits_get(bits, p)) {            // ACK delimiter must be recessive
        out->validity = INVALID_BIT;
        return INVALID_BIT;
    }
    p++;
    for (i = 0; i < 7 && p < total_bits; i++, p++) {
        if (!bits_get(bits, p)) {
            out->validity = INVALID_BIT;
            return INVALID_BIT;
        }
    }
    for (i = 0; i < 3 && p < total_bits; i++, p++) {
        if (!bits_get(bits, p)) {
            out->validity = INVALID_BIT;
            return INVALID_BIT;
        }
    }

    out->total_bits = p;
    out->validity = VALID_FRAME;
    return VALID_FRAME;
}
