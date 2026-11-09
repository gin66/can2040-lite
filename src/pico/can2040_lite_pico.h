// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_pico.h — RP2040/RP2350 PIO backend (target only).

#ifndef CAN2040_LITE_PICO_H
#define CAN2040_LITE_PICO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Number of raw oversampled words captured per arm (one word per bit plus
// one marker word).  Enough for the longest classic CAN frame with margin.
#define CAN2040_LITE_PICO_RX_WORDS 400u

int can2040_lite_pico_rx_init(uint32_t gpio, uint32_t bitrate);
int can2040_lite_pico_tx_init(uint32_t gpio, uint32_t bitrate);

// Re-arm the RX SM at the SOF poll loop and start a DMA capture.
void can2040_lite_pico_rx_arm(void);

// Wait for the capture to finish.  Returns non-zero on success.
int can2040_lite_pico_rx_wait(uint32_t timeout_ms);

// Convert captured words to bits (majority per bit word).  Skips the marker
// word.  Returns the number of bits written.
uint32_t can2040_lite_pico_rx_bits(uint8_t *bits, uint32_t max_bits);

// Bit-stuff-independent transmit: `bits` is the complete on-wire frame
// (SOF..IFS, already stuffed), MSB-first.  Padding is recessive.
void can2040_lite_pico_tx_frame(const uint8_t *bits, uint32_t nbits);

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_PICO_H
