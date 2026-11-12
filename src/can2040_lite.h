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
//
// The public surface matches can2040 so an application can recompile against
// this header.  src/can2040.h aliases the can2040 spellings onto these.

#ifndef CAN2040_LITE_H
#define CAN2040_LITE_H

#include <stdint.h>

#include "can2040_lite_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

// Message ID flags (same layout as can2040).
#define CAN2040_LITE_ID_RTR (1u << 30)
#define CAN2040_LITE_ID_EFF (1u << 31)

// Callback notification bits (same values as can2040).
#define CAN2040_LITE_NOTIFY_RX    (1u << 20)
#define CAN2040_LITE_NOTIFY_TX    (1u << 21)
#define CAN2040_LITE_NOTIFY_ERROR (1u << 23)

// Transmit queue depth.  Messages are copied at transmit() time and leave in
// FIFO order with no id reordering.
#define CAN2040_LITE_TX_QUEUE_LEN 4

struct can2040_lite;
struct can2040_lite_msg;

typedef void (*can2040_lite_cb)(struct can2040_lite *cd, uint32_t notify,
                                struct can2040_lite_msg *msg);

struct can2040_lite_msg {
    uint32_t id;          // 11-bit or 29-bit ID | CAN2040_LITE_ID_EFF/RTR
    uint32_t dlc;         // 0..15; 8..15 carry 8 data bytes
    union {
        uint8_t data[8];
        uint32_t data32[2];
    };
};

// The first four fields match can2040 and are the ones a can2040 caller reads.
struct can2040_lite_stats {
    uint32_t rx_total;      // Successfully received frames
    uint32_t tx_total;      // Successfully transmitted frames (ACK received)
    uint32_t tx_attempt;    // Frames accepted by transmit()
    uint32_t parse_error;   // Received frames that are not VALID_FRAME, plus
                            // transmits that end in NACK, arbitration loss or
                            // a bit error
    uint32_t tx_nack;       // Frames with no ACK (readback)
    uint32_t tx_error;      // Frames with an error detected (readback)
    uint32_t crc_errors;    // Received frames with CRC errors
    uint32_t stuff_errors;  // Frames with stuffing violations
    uint32_t bit_errors;    // Received frames with form/bit errors
    uint32_t rx_overflow;   // Frames dropped because no buffer was available
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

    // Transmit queue.  tx_head/tx_count are protected by the critical section.
    struct can2040_lite_msg tx_msg[CAN2040_LITE_TX_QUEUE_LEN];
    uint8_t tx_bits[CAN2040_LITE_TX_QUEUE_LEN][CAN2040_LITE_MAX_FRAME_BYTES];
    uint32_t tx_nbits[CAN2040_LITE_TX_QUEUE_LEN];
    uint32_t tx_head;
    uint32_t tx_count;
    int tx_active;                 // head handed to the hardware
    struct can2040_lite_msg tx_done_msg;  // valid during a completion callback

    // Backend-private pointer (PIO/DMA state on target).
    void *backend;
};

// Hardware operations.  The host tests install a fake table; the Pico
// backend installs the real one.  With no table installed, start() still
// marks the driver started and transmit() still queues.
struct can2040_lite_event {
    int kind;                 // 0 = none, 1 = tx finished, 2 = rx frame
    int tx_error;             // tx finished: recessive bit read dominant
    can_parsed_t rx;
};

struct can2040_lite_hw {
    int (*start)(struct can2040_lite *cd, uint32_t sys_clock,
                 uint32_t bitrate, int32_t gpio_rx, int32_t gpio_tx);
    void (*stop)(struct can2040_lite *cd);
    int (*tx_frame)(const uint8_t *bits, uint32_t nbits);  // 0 = SM accepted
    int (*poll)(struct can2040_lite_event *ev);            // 1 = event stored
};

void can2040_lite_set_hw(const struct can2040_lite_hw *hw);

// Weak hook the Pico backend overrides to register its ops table before the
// first start().  The host link keeps the empty definition in can2040_lite.c.
void can2040_lite_hw_attach(void);

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

// PIO IRQ handler (called from the application's PIO IRQ).
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
