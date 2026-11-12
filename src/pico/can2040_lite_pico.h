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

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

struct can2040_lite;

int can2040_lite_pico_rx_init(uint32_t gpio, uint32_t bitrate);
// gpio is CAN-TX; gpio_rx is the readback (CAN-RX) pin.  They are the same
// pin in loopback tests and different with a real transceiver.
int can2040_lite_pico_tx_init(uint32_t gpio, uint32_t gpio_rx,
                              uint32_t bitrate);

// Re-arm the RX state machine.  The PIO SM polls for SOF; when a frame
// arrives the application's PIO IRQ handler calls
// can2040_lite_pio_irq_handler(), which drains this backend through poll().
void can2040_lite_pico_rx_arm(void);

// Wait for the next frame to be parsed (sets frame_ready).  Returns 1 on
// success, 0 on timeout.  Use can2040_lite_pico_rx_get_parsed() to read it.
int can2040_lite_pico_rx_wait(uint32_t timeout_ms);

// Return the parsed frame from the edge state machine, or NULL if no
// complete frame is available.  The caller must call this only after
// can2040_lite_pico_rx_wait() returns 1.  Subsequent frames overwrite the
// result.
const can_parsed_t *can2040_lite_pico_rx_get_parsed(void);

// Rebuild the on-wire bit stream from the last parsed frame: stuffed
// SOF..CRC followed by the unstuffed tail.  Returns the number of bits
// written.  Use can2040_lite_pico_rx_get_parsed() for the decoded frame.
uint32_t can2040_lite_pico_rx_bits(uint8_t *bits, uint32_t max_bits);

// One-shot message transmit.  `bits` is SOF through the recessive bit after
// the CRC (already stuffed), MSB-first.  The SM waits for 11 recessive bit
// times, then drives one bit per 32 cycles.  That last bit is driven and the
// SM parks without sampling or padding it; the pin stays at that level, which
// is the ACK slot, the end of frame and the interframe space.  ERR is only a
// recessive bit read as dominant.  A dominant bit is not a conflict.  DONE
// means the shift ended.  ACK vs NACK is on the RX capture, not on these
// IRQs.  Call only when the previous message is finished.  Returns 0 if the
// state machine accepted the message, -1 otherwise.
int can2040_lite_pico_tx_frame(const uint8_t *bits, uint32_t nbits);

// Change the readback pin used by the TX SM (e.g. to inject bus faults in a
// test).  Takes effect for the next message.
void can2040_lite_pico_tx_set_readback_gpio(uint32_t gpio);

// Message-complete and abort flags (latched until cleared or tx_frame()).
int can2040_lite_pico_tx_done_pending(void);
int can2040_lite_pico_tx_error_pending(void);
void can2040_lite_pico_tx_clear_status(void);

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_PICO_H
