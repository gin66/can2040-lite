// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite.h — Public CAN2040-Lite API.
//
// Two CAN PIO state machines (rx + tx) on one PIO block.  The rx + tx PIO
// programs together must stay within 29 instruction words, leaving 2 of the
// block's 32 words for the FreeRTOS cycle-counter SM.

#ifndef CAN2040_LITE_H
#define CAN2040_LITE_H

#include <stdint.h>

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

// Message ID flags (same layout as can2040).
#define CAN2040_LITE_ID_EFF 0x20000000u
#define CAN2040_LITE_ID_RTR 0x40000000u

// Callback notification types.
enum {
    CAN2040_LITE_NOTIFY_RX,       // Received frame (from bus)
    CAN2040_LITE_NOTIFY_TX,       // Transmitted frame (ACK confirmed)
    CAN2040_LITE_NOTIFY_TX_NACK,  // Transmitted frame, no ACK
    CAN2040_LITE_NOTIFY_TX_ERROR, // Transmitted frame, error detected
    CAN2040_LITE_NOTIFY_ERROR     // Internal error (buffer overflow)
};

struct can2040_lite;
struct can2040_lite_msg;

typedef void (*can2040_lite_cb)(struct can2040_lite *cd, int notify,
                                struct can2040_lite_msg *msg);

struct can2040_lite_msg {
    uint32_t id;          // 11-bit or 29-bit ID | CAN2040_LITE_ID_EFF/RTR
    uint32_t dlc;         // 0..8
    uint8_t data[8];
    uint32_t flags;       // reserved
    uint32_t crc_error;   // 1 if CRC failed (rx notifications)
};

struct can2040_lite_stats {
    uint32_t rx_total;      // Successfully received frames
    uint32_t tx_total;      // Successfully transmitted frames (ACK received)
    uint32_t tx_nack;       // Frames with no ACK (readback)
    uint32_t tx_error;      // Frames with an error detected (readback)
    uint32_t crc_errors;    // Received frames with CRC errors
    uint32_t stuff_errors;  // Frames with stuffing violations
    uint32_t rx_overflow;   // Frames dropped because no buffer was available
    uint32_t tx_attempt;    // Frames handed to the transmitter
};

struct can2040_lite {
    // Configuration
    int32_t gpio_rx;
    int32_t gpio_tx;
    uint32_t pio_num;
    uint32_t sys_clock;
    uint32_t bitrate;
    int started;
    can2040_lite_cb cb;

    // Statistics
    struct can2040_lite_stats stats;

    // Backend-private pointer (PIO/DMA state on target).
    void *backend;
};

// Initialise the driver state.  No hardware is touched until start().
void can2040_lite_setup(struct can2040_lite *cd, uint32_t pio_num);

// Register the frame/notification callback.
void can2040_lite_callback_config(struct can2040_lite *cd, can2040_lite_cb cb);

// Start the transceiver.  gpio_tx == -1 selects receive-only mode.
void can2040_lite_start(struct can2040_lite *cd, uint32_t sys_clock,
                        uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx);

// Queue a frame for transmission.  Returns 0 on success, -1 on error/queue
// full/not started.
int can2040_lite_transmit(struct can2040_lite *cd,
                          const struct can2040_lite_msg *msg);

// Returns non-zero when a transmit slot is available.
int can2040_lite_check_transmit(struct can2040_lite *cd);

// PIO IRQ handler (called from the target's PIO IRQ).
void can2040_lite_pio_irq_handler(struct can2040_lite *cd);

// Copy the current statistics.
void can2040_lite_get_statistics(struct can2040_lite *cd,
                                 struct can2040_lite_stats *stats);

// Stop the transceiver and release target resources.
void can2040_lite_stop(struct can2040_lite *cd);

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_H
