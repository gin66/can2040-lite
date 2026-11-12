// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.

#include <string.h>

#include "can_pio_program.h"

// PIO instruction images are assembled here at runtime.  The composition
// depends on compile-time features (the ACK overlay is only built when
// CAN_ACK_GENERATION_SUPPORT is set), so this replaces pioasm output with
// pio_encode-style words.  All images use an origin of 0 for their JMP
// targets; the TX/ACK overlay is relocated when copied into its slot.

enum {
    PIO_OP_JMP = 0,
    PIO_OP_IN = 2,
    PIO_OP_OUT = 3,
    PIO_OP_MOV = 5,
    PIO_OP_IRQ = 6,
    PIO_OP_SET = 7
};

enum {
    PIO_COND_ALWAYS = 0,
    PIO_COND_NOT_X = 1,
    PIO_COND_XDEC = 2,
    PIO_COND_NOT_Y = 3,
    PIO_COND_YDEC = 4,
    PIO_COND_PIN = 6
};

enum {
    PIO_SRC_X = 1,
    PIO_DST_PINS = 0,
    PIO_DST_X = 1
};

// Instruction layout: opcode[15:13] | delay[12:8] | operands[7:0].
static uint16_t enc_jmp(unsigned cond, unsigned addr, unsigned delay) {
    return (uint16_t)((PIO_OP_JMP << 13) | ((delay & 0x1fu) << 8) |
                      ((cond & 7u) << 5) | (addr & 31u));
}

static uint16_t enc_in(unsigned src, unsigned count, unsigned delay) {
    return (uint16_t)((PIO_OP_IN << 13) | ((delay & 0x1fu) << 8) |
                      ((src & 7u) << 5) | (count & 31u));
}

static uint16_t enc_out(unsigned dst, unsigned count, unsigned delay) {
    return (uint16_t)((PIO_OP_OUT << 13) | ((delay & 0x1fu) << 8) |
                      ((dst & 7u) << 5) | (count & 31u));
}

static uint16_t enc_mov(unsigned dst, unsigned src) {
    return (uint16_t)((PIO_OP_MOV << 13) | ((dst & 7u) << 5) | (src & 7u));
}

static uint16_t enc_irq(unsigned index) {
    return (uint16_t)((PIO_OP_IRQ << 13) | (index & 31u));
}

static uint16_t enc_set(unsigned dst, unsigned data, unsigned delay) {
    return (uint16_t)((PIO_OP_SET << 13) | ((delay & 0x1fu) << 8) |
                      ((dst & 7u) << 5) | (data & 31u));
}

static void add(can_pio_program_t *p, uint16_t insn) {
    if (p->length < sizeof(p->code) / sizeof(p->code[0]))
        p->code[p->length++] = insn;
}

// Edge-pair RX capture.  X free-runs downward once every two cycles.  Each
// edge samples X[15:0] into the ISR; autopush at 32 bits emits one word:
//   word[31:16] = X[15:0] at the 1->0 edge
//   word[15:0]  = X[15:0] at the following 0->1 edge
// The edge path is two cycles with no `jmp x--`, so every captured edge
// leaves X one count high; the host decoder adds a fixed +1 tick per edge.
static void build_rx_pair(can_pio_program_t *p) {
    memset(p, 0, sizeof(*p));
    // X was 0: wrap to 0xFFFFFFFF and fall through into `high`.
    add(p, enc_jmp(PIO_COND_XDEC, 1, 0));
    p->wrap_target = p->length;
    // high: still recessive, keep counting.
    add(p, enc_jmp(PIO_COND_PIN, 0, 0));
    // 1->0 edge: upper half of the FIFO word, then fall into `low`.
    add(p, enc_in(PIO_SRC_X, 16, 0));
    // low: wait for the rise that completes the pair.
    add(p, enc_jmp(PIO_COND_PIN, 6, 0));
    // low_cnt: X was 0, fall through.  Its head is behind it, so the
    // 32-bit underflow needs the extra `jmp low` below.
    add(p, enc_jmp(PIO_COND_XDEC, 3, 0));
    add(p, enc_jmp(PIO_COND_ALWAYS, 3, 0));
    // low_cap: 0->1, lower half -> autopush.
    add(p, enc_in(PIO_SRC_X, 16, 0));
    p->wrap = (uint8_t)(p->length - 1u);
}

// One-shot TX shifter, 32 PIO cycles per bit.  Y is the bit count minus one,
// the OSR and joined FIFO hold the stuffed message, and autopull hides the
// word refill so there is no word-boundary jitter.  The bit after the CRC is
// only driven: the timing stops with DONE and leaves the pad recessive for
// ACK, EOF, and IFS.
static void build_tx(can_pio_program_t *p) {
    memset(p, 0, sizeof(*p));
    // idle_loop: sample the bus.  Dominant falls into `reload`.
    add(p, enc_jmp(PIO_COND_PIN, 2, 0));
    // reload: 23 samples; wrap sends the PC back to idle_loop.
    add(p, enc_set(PIO_DST_X, 22, 0));
    p->wrap = 1;
    // saw_rec: last sample, SOF is on the next edge with no delay.
    add(p, enc_jmp(PIO_COND_NOT_X, 4, 0));
    // 22 gaps of 1 + 1 + 14 = 16 cycles.
    add(p, enc_jmp(PIO_COND_XDEC, 0, 13));
    // bitloop: cycle 31 of the previous bit.
    add(p, enc_out(PIO_DST_X, 1, 0));
    // Cycle 0: drive the edge.
    add(p, enc_mov(PIO_DST_PINS, PIO_SRC_X));
    // Y == 0: the bit after the CRC.  Drive it and stop.
    add(p, enc_jmp(PIO_COND_YDEC, 9, 0));
    add(p, enc_irq(6));
    add(p, enc_jmp(PIO_COND_ALWAYS, 12, 0));
    // more: cycles 2..21.  X == 0 means dominant, so never sample.
    add(p, enc_jmp(PIO_COND_NOT_X, 13, 19));
    // Cycle 22 sample, then cycles 23..30.
    add(p, enc_jmp(PIO_COND_PIN, 4, 8));
    // Conflict falls through.  Not part of the bit timing.
    add(p, enc_irq(7));
    // park: stay here until software re-inits for the next message.
    add(p, enc_jmp(PIO_COND_ALWAYS, 12, 0));
    // dom: dominant bit, cycles 22..30, no sample.
    add(p, enc_jmp(PIO_COND_ALWAYS, 4, 8));
    p->wrap_target = 0;
}

#if CAN_ACK_GENERATION_SUPPORT
// ACK helper overlay of the TX instruction slot.  It follows bus edges and
// fires one inset dominant pulse on the ACK slot after the CPU allows it.
//   X    edges still to come, minus one; 0xFFFFFFFF while only following
//   OSR  gap cycles from the terminal edge to the fire check
//   Y    0 until the received CRC matches; 1 allows the pulse
// The plan supplies X = edges - 1 and the OSR gap.  The delay is [30], not
// [31]: the following jmp adds one cycle, so the dominant window is exactly
// 32 cycles (one bit), from slot+0.25 to slot+1.25.
static void build_ack(can_pio_program_t *p) {
    memset(p, 0, sizeof(*p));
    // release: recessive on every iteration.
    add(p, enc_set(PIO_DST_PINS, 1, 0));
    p->wrap_target = 0;
    p->wrap = 0;
    // high: entry when the pad is recessive.  Follow while X underflows.
    add(p, enc_jmp(PIO_COND_PIN, 1, 0));
    add(p, enc_jmp(PIO_COND_XDEC, 4, 0));
    add(p, enc_jmp(PIO_COND_ALWAYS, 8, 0));
    // low: entry when the pad is dominant.  Wait for the rising edge.
    add(p, enc_jmp(PIO_COND_PIN, 6, 0));
    add(p, enc_jmp(PIO_COND_ALWAYS, 4, 0));
    // rose: X expired on a rising edge.
    add(p, enc_jmp(PIO_COND_XDEC, 1, 0));
    add(p, enc_jmp(PIO_COND_ALWAYS, 8, 0));
    // gap: X = gap cycles; autopull is off.
    add(p, enc_out(PIO_DST_X, 32, 0));
    add(p, enc_jmp(PIO_COND_XDEC, 9, 0));
    // Not verified: leave the slot recessive.
    add(p, enc_jmp(PIO_COND_NOT_Y, 0, 0));
    // Dominant drive, 31 cycles plus the jmp below = one bit.
    add(p, enc_set(PIO_DST_PINS, 0, 30));
    add(p, enc_jmp(PIO_COND_ALWAYS, 0, 0));
}
#endif

static can_pio_program_t rx_pair_prog;
static can_pio_program_t tx_prog;
static int rx_pair_built;
static int tx_built;
#if CAN_ACK_GENERATION_SUPPORT
static can_pio_program_t ack_prog;
static int ack_built;
#endif

const can_pio_program_t *can_pio_rx_pair_program(void) {
    if (!rx_pair_built) {
        build_rx_pair(&rx_pair_prog);
        rx_pair_built = 1;
    }
    return &rx_pair_prog;
}

const can_pio_program_t *can_pio_tx_program(void) {
    if (!tx_built) {
        build_tx(&tx_prog);
        tx_built = 1;
    }
    return &tx_prog;
}

#if CAN_ACK_GENERATION_SUPPORT
const can_pio_program_t *can_pio_ack_program(void) {
    if (!ack_built) {
        build_ack(&ack_prog);
        ack_built = 1;
    }
    return &ack_prog;
}
#endif
