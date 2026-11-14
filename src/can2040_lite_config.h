// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.

#ifndef CAN2040_LITE_CONFIG_H
#define CAN2040_LITE_CONFIG_H

#ifndef CAN_ACK_GENERATION_SUPPORT
#define CAN_ACK_GENERATION_SUPPORT 1
#endif

// Opt-in capture log.  When set, the edge-pair decoder keeps every FIFO word
// it consumes, so a received frame can be inspected without a logic analyzer.
// fall_time/rise_time are reconstructed in can_edge_sm's 31-bit counter
// domain, which is enough to check edge placement including the ACK pulse.
// Off by default; the log stops when it reaches CAN_EDGE_PAIR_RECORD_CAPACITY.
#ifndef CAN_EDGE_PAIR_TIME_RECORDING
#define CAN_EDGE_PAIR_TIME_RECORDING 0
#endif

#ifndef CAN_EDGE_PAIR_RECORD_CAPACITY
#define CAN_EDGE_PAIR_RECORD_CAPACITY 256
#endif

// PIO interrupt flags used by the one-shot TX program for done and error.
// These are PIO IRQ *flag* numbers, not system IRQ numbers; the application
// still installs PIOx_IRQ_0.  RP2040 can route only flags 0..3 to the system
// IRQ, RP2350 has 0..7, so the defaults work on both.  The runtime program
// builder and the backend both read these, so they cannot drift.
#ifndef CAN_PIO_TX_DONE_IRQ
#define CAN_PIO_TX_DONE_IRQ 0u
#endif
#ifndef CAN_PIO_TX_ERR_IRQ
#define CAN_PIO_TX_ERR_IRQ 1u
#endif
#if CAN_PIO_TX_DONE_IRQ == CAN_PIO_TX_ERR_IRQ
#error "CAN_PIO_TX_DONE_IRQ and CAN_PIO_TX_ERR_IRQ must differ"
#endif
#if defined(PICO_PIO_VERSION) && PICO_PIO_VERSION == 0
#if CAN_PIO_TX_DONE_IRQ > 3u || CAN_PIO_TX_ERR_IRQ > 3u
#error "RP2040 can route only PIO IRQ flags 0..3"
#endif
#endif

#endif // CAN2040_LITE_CONFIG_H
