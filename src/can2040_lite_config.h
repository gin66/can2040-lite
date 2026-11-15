// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.

#ifndef CAN2040_LITE_CONFIG_H
#define CAN2040_LITE_CONFIG_H

// Production default: receive-only.  CAN2040-Lite's purpose is a small,
// deterministic RX path; the CPU-timed ACK anchor is experimental and is
// poor at high bus occupancy, where the anchor word can be queued behind
// earlier words and the fixed wire deadline is already gone.  The code stays
// behind this flag for bring-up and future work, but is compiled out by
// default.  Set it to 1 explicitly to build the ACK path.
#ifndef CAN_ACK_GENERATION_SUPPORT
#define CAN_ACK_GENERATION_SUPPORT 0
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

// ACK pulse placement constants.  The pulse is armed from the RX FIFO word
// that ends the last dominant wire bit of the stuffed CRC field.  The rise is
// one IRQ-entry latency before the handler runs, so that constant is added to
// the measured handler time:
//
//   target = (r_tail + 0.25) * cycles_per_bit
//   deadline = sys_tick_at_irq_entry + ENTRY_LATENCY - target
//
// ENTRY_LATENCY is the CPU cycles from the PIO push that wakes the RX IRQ to
// the first SysTick read in the handler, with the FIFO empty and RX IRQ at
// priority 0.  Measure it on the target; the default is a conservative
// starting value.  EXEC_BIAS covers the pio_sm_exec latency of `set pins`.
#ifndef CAN_ACK_ENTRY_LATENCY_CYCLES
#define CAN_ACK_ENTRY_LATENCY_CYCLES 48u
#endif

#ifndef CAN_ACK_EXEC_BIAS_CYCLES
#define CAN_ACK_EXEC_BIAS_CYCLES 8u
#endif

// Optional per-bit logic-analyzer probe.  The application selects a GPIO from
// its build flags (-DCAN_RX_BIT_PROBE_PIN=<n>); the Pico backend drives it.
// The RX parser pulses the pin around every unstuffed bit, so the measured
// pulse width is the parse time of one bit.  0 disables the probe.
#ifndef CAN_RX_BIT_PROBE_PIN
#define CAN_RX_BIT_PROBE_PIN 0
#endif

// Optional probe around the once-per-frame CRC/stuffing prediction.  The
// prediction runs at CRC entry, at least 11 bit times before the ACK anchor.
// The probe width is the margin check: it must stay far below that span.
#ifndef CAN_ACK_PREDICT_PROBE_PIN
#define CAN_ACK_PREDICT_PROBE_PIN 0
#endif

#endif // CAN2040_LITE_CONFIG_H
