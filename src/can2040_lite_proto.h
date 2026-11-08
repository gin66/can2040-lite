// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_proto.h — CAN 2.0B bit helpers, CRC-15, stuffing and decode.
//
// This is the protocol core shared by the host tests and the target firmware.
// It contains no PIO or pico-sdk dependencies.

#ifndef CAN2040_LITE_PROTO_H
#define CAN2040_LITE_PROTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Enough for a maximum-length classic CAN frame with worst-case stuffing
// (~164 bits); 256 leaves headroom for malformed test streams.
#define CAN2040_LITE_MAX_FRAME_BITS 256u
#define CAN2040_LITE_MAX_FRAME_BYTES ((CAN2040_LITE_MAX_FRAME_BITS + 7u) / 8u)

// Return value for bit-level helpers on violation / overflow.
#define CAN_BITS_ERROR UINT32_MAX

typedef enum {
    VALID_FRAME,
    INVALID_CRC,
    INVALID_STUFF,
    INVALID_BIT
} frame_validity_t;

// Result of decoding a raw stuffed frame.
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t is_extended;
    uint8_t is_rtr;
    uint8_t ack;            // 1 = ACK slot dominant, 0 = recessive
    uint8_t data[8];
    uint16_t crc;           // CRC carried by the frame
    uint16_t crc_calc;      // CRC computed over the decoded header/data
    frame_validity_t validity;
    uint32_t stuffed_bits;  // stuffed bits consumed (SOF..CRC + trailing stuff)
    uint32_t total_bits;    // stuffed prefix + unstuffed tail consumed
} can_parsed_t;

// Bit helpers (MSB-first packing: bit 0 is the MSB of byte 0).
uint8_t bits_get(const uint8_t *buf, uint32_t pos);
void bits_put(uint8_t *buf, uint32_t pos, uint8_t bit);
void bits_clear(uint8_t *buf, uint32_t nbits);

// CRC-15 (CAN polynomial x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^2 + 1,
// reflected form 0x4599), computed MSB-first.  crc_can15_update()/calc() are
// the byte-wise reference matching can2040's crc_byte() recurrence.
uint16_t crc_can15_update(uint16_t crc, uint8_t data);
uint16_t crc_can15_calc(const uint8_t *data, uint32_t num_bytes);
uint16_t crc_can15_bits(uint16_t crc, const uint8_t *bits, uint32_t num_bits);

// Bit stuffing / unstuffing.  Both operate on the *stuffed region only*
// (SOF .. end of CRC sequence); the tail (CRC delimiter, ACK, EOF, IFS) is
// never stuffed.
//
// bit_stuff() returns the number of bits written, or CAN_BITS_ERROR on
// overflow.  Output need not be zeroed.
//
// bit_unstuff() returns the number of unstuffed bits, or CAN_BITS_ERROR when
// the stream contains 6 consecutive identical bits (stuff violation) or is
// truncated.  Output need not be zeroed.
uint32_t bit_stuff(const uint8_t *data, uint32_t data_bits,
                   uint8_t *stuffed, uint32_t max_stuffed_bits);
uint32_t bit_unstuff(const uint8_t *stuffed, uint32_t stuffed_bits,
                     uint8_t *unstuffed, uint32_t max_unstuffed_bits);

// Decode a complete raw frame (stuffed prefix + unstuffed tail).
// Returns VALID_FRAME, INVALID_CRC, INVALID_STUFF or INVALID_BIT.
frame_validity_t can_frame_decode(const uint8_t *bits, uint32_t total_bits,
                                  can_parsed_t *out);

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_PROTO_H
