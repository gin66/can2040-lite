// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.

#ifndef CAN_PIO_PROGRAM_H
#define CAN_PIO_PROGRAM_H

#include <stdint.h>

#include "can2040_lite_config.h"

// Runtime-built PIO instruction images.  pioasm output is not used: the
// running backend assembles these at init so the ACK overlay can be omitted
// entirely when CAN_ACK_GENERATION_SUPPORT is 0.

#ifdef __cplusplus
extern "C" {
#endif

// One assembled image.  JMP targets are relative to origin 0; the TX/ACK
// overlay adds its slot origin when copied into PIO instruction memory.
typedef struct {
    uint16_t code[32];
    uint8_t length;
    uint8_t wrap_target;
    uint8_t wrap;
} can_pio_program_t;

#define CAN_PIO_RX_PAIR_LENGTH 7u
#define CAN_PIO_TX_LENGTH 14u
#define CAN_PIO_ACK_LENGTH 13u

// Edge-pair capture: two edges per FIFO word.
const can_pio_program_t *can_pio_rx_pair_program(void);
// One-shot 32-cycle-per-bit message shifter.
const can_pio_program_t *can_pio_tx_program(void);
#if CAN_ACK_GENERATION_SUPPORT
// ACK helper overlay of the TX instruction slot.
const can_pio_program_t *can_pio_ack_program(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // CAN_PIO_PROGRAM_H
