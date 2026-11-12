// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// ack_cycle_model.h — cycle model for the ACK PIO image.
//
// Header-only so it is part of the same `make test` entry.  The instruction
// table mirrors can_pio_ack_program(); the model runs one instruction per
// cycle, including jmp x-- pre-decrement semantics, delay slots, and
// pio_sm_exec as one stolen cycle that replaces the instruction the SM would
// have run.

#ifndef ACK_CYCLE_MODEL_H
#define ACK_CYCLE_MODEL_H

#include <stdint.h>
#include <stddef.h>

#ifndef ACK_MODEL_MAX_CYCLES
#define ACK_MODEL_MAX_CYCLES 1024
#endif

typedef enum {
    ACK_I_SET_PINS,
    ACK_I_JMP_PIN,
    ACK_I_JMP_XDEC,
    ACK_I_JMP,
    ACK_I_OUT_X,
    ACK_I_JMP_NOT_Y
} ack_insn_kind_t;

typedef struct {
    ack_insn_kind_t kind;
    int16_t target;
    uint8_t delay;
    uint8_t value;
} ack_insn_t;

// Mirrors can_pio_ack_program().  13 words, 32 cycles per bit.
static const ack_insn_t ack_program_model[] = {
    {ACK_I_SET_PINS,    0,  0,  1}, //  0 release: set pins, 1
    {ACK_I_JMP_PIN,     1,  0,  0}, //  1 high: jmp pin, high
    {ACK_I_JMP_XDEC,    4,  0,  0}, //  2 jmp x--, low
    {ACK_I_JMP,         8,  0,  0}, //  3 jmp gap
    {ACK_I_JMP_PIN,     6,  0,  0}, //  4 low: jmp pin, rose
    {ACK_I_JMP,         4,  0,  0}, //  5 jmp low
    {ACK_I_JMP_XDEC,    1,  0,  0}, //  6 rose: jmp x--, high
    {ACK_I_JMP,         8,  0,  0}, //  7 jmp gap
    {ACK_I_OUT_X,       0,  0, 32}, //  8 out x, 32
    {ACK_I_JMP_XDEC,    9,  0,  0}, //  9 gaploop: jmp x--, gaploop
    {ACK_I_JMP_NOT_Y,   0,  0,  0}, // 10 jmp !y, release
    {ACK_I_SET_PINS,    0, 30,  0}, // 11 set pins, 0 [30]
    {ACK_I_JMP,         0,  0,  0}, // 12 jmp release
};

#define ACK_MODEL_WRAP_TARGET 0
#define ACK_MODEL_WRAP        0
#define ACK_MODEL_PC_HIGH     1
#define ACK_MODEL_PC_LOW      4
#define ACK_MODEL_CYCLES_PER_BIT 32u
#define ACK_MODEL_GAP_OFFSET  2u  // gap cycles = 32 * gap_bits + 2

typedef enum {
    ACK_EXEC_NONE,
    ACK_EXEC_SET_Y,
    ACK_EXEC_SET_X,
    ACK_EXEC_PULL,
    ACK_EXEC_SET_PINS
} ack_exec_kind_t;

typedef struct {
    ack_exec_kind_t kind;
    uint32_t value;
} ack_exec_t;

typedef struct {
    int cycle;
    uint8_t level;
} ack_edge_t;

typedef struct {
    int cycle;
    ack_exec_t exec;
} ack_exec_event_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t osr;
    uint32_t fifo;
    int pc;
    int stall;
    uint8_t pad;
    int pulse_start;
    int pulse_end;
} ack_model_t;

static inline int ack_model_wrap(int pc) {
    return (pc == ACK_MODEL_WRAP) ? ACK_MODEL_WRAP_TARGET : pc;
}

static inline uint8_t ack_model_level(const ack_edge_t *edges, int nedges,
                                      uint8_t initial, int cycle) {
    uint8_t level = initial;
    int i;
    for (i = 0; i < nedges; i++) {
        if (edges[i].cycle <= cycle)
            level = edges[i].level;
    }
    return level;
}

static inline void ack_model_pad(ack_model_t *m, int cycle, uint8_t level) {
    if (m->pad == level)
        return;
    if (level == 0 && m->pulse_start < 0)
        m->pulse_start = cycle;
    else if (level == 1 && m->pulse_start >= 0 && m->pulse_end < 0)
        m->pulse_end = cycle;
    m->pad = level;
}

static inline void ack_model_apply_exec(ack_model_t *m, int cycle,
                                        const ack_exec_t *e) {
    switch (e->kind) {
    case ACK_EXEC_SET_Y:
        m->y = e->value;
        break;
    case ACK_EXEC_SET_X:
        m->x = e->value;
        break;
    case ACK_EXEC_PULL:
        m->osr = m->fifo;
        break;
    case ACK_EXEC_SET_PINS:
        ack_model_pad(m, cycle, (uint8_t)(e->value & 1u));
        break;
    default:
        break;
    }
}

// Run one PIO cycle.  `exec` is consumed instead of the program instruction
// when it is non-NULL, matching pio_sm_exec on an enabled SM.
static inline void ack_model_step(ack_model_t *m, int cycle, uint8_t pin,
                                  const ack_exec_t *exec) {
    const ack_insn_t *insn;
    int next;

    if (exec) {
        ack_model_apply_exec(m, cycle, exec);
        return; // stolen cycle; PC does not advance
    }
    if (m->stall > 0) {
        m->stall--;
        return;
    }

    insn = &ack_program_model[m->pc];
    next = m->pc + 1;
    switch (insn->kind) {
    case ACK_I_SET_PINS:
        ack_model_pad(m, cycle, insn->value);
        m->stall = insn->delay;
        break;
    case ACK_I_JMP_PIN:
        next = pin ? insn->target : m->pc + 1;
        break;
    case ACK_I_JMP_XDEC:
        if (m->x != 0) {
            m->x--;
            next = insn->target;
        } else {
            m->x = 0xffffffffu;
            next = m->pc + 1;
        }
        break;
    case ACK_I_JMP:
        next = insn->target;
        break;
    case ACK_I_OUT_X:
        m->x = m->osr;
        m->osr = 0;
        break;
    case ACK_I_JMP_NOT_Y:
        next = (m->y == 0) ? insn->target : m->pc + 1;
        break;
    default:
        next = m->pc + 1;
        break;
    }
    if (m->pc == ACK_MODEL_WRAP)
        next = ACK_MODEL_WRAP_TARGET;
    m->pc = next;
}

static inline void ack_model_init(ack_model_t *m, int start_pc,
                                  uint32_t x, uint32_t y, uint32_t osr,
                                  uint32_t fifo) {
    m->x = x;
    m->y = y;
    m->osr = osr;
    m->fifo = fifo;
    m->pc = start_pc;
    m->stall = 0;
    m->pad = 1;
    m->pulse_start = -1;
    m->pulse_end = -1;
}

static inline void ack_model_run(ack_model_t *m, uint8_t initial_pin,
                                 const ack_edge_t *edges, int nedges,
                                 const ack_exec_event_t *execs, int nexecs,
                                 int cycles) {
    int cycle;
    for (cycle = 0; cycle < cycles; cycle++) {
        const ack_exec_t *exec = NULL;
        uint8_t pin;
        int i;

        for (i = 0; i < nexecs; i++) {
            if (execs[i].cycle == cycle) {
                exec = &execs[i].exec;
                break;
            }
        }
        pin = ack_model_level(edges, nedges, initial_pin, cycle);
        ack_model_step(m, cycle, pin, exec);
    }
}

#endif // ACK_CYCLE_MODEL_H
