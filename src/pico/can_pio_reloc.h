// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// Helpers for the TX/ACK instruction-slot swap.  Host-testable: no SDK types.
//
// pioasm emits JMP targets as absolute instruction addresses with the program
// loaded at 0.  pio_add_program adds the load offset to a JMP word, which
// adjusts that 5-bit field.  A raw copy of the instruction array into a slot
// that is not at 0 jumps into whatever occupies the low addresses.

#ifndef CAN_PIO_RELOC_H
#define CAN_PIO_RELOC_H

#include <stdint.h>

// ack.pio, program-relative.  SMx_ADDR is the absolute PC, so compare
// addr - origin against these.
#define CAN_ACK_PC_RELEASE  0u
#define CAN_ACK_PC_SET_PINS 11u
#define CAN_ACK_ENTRY_HIGH  1u
#define CAN_ACK_ENTRY_LOW   4u

// Match pio_add_program_at_offset: a JMP (opcode 000) is `instr + origin`.
// Other opcodes are unchanged.  The sum stays inside the address field for
// any legal placement of these 13- and 14-word programs.
static inline uint16_t can_pio_reloc_instr(uint16_t instr, uint32_t origin) {
    if ((instr >> 13) == 0)
        return (uint16_t)(instr + origin);
    return instr;
}

// True while the helper is still in the follow or gap instructions.
// Release (relative 0) and the dominant drive (relative >= 11) are too late:
// an exec there either does nothing useful or replaces the pulse.
static inline int can_ack_allow_pc(uint32_t addr, uint32_t origin) {
    uint32_t rel;

    if (addr < origin)
        return 0;
    rel = addr - origin;
    if (rel == CAN_ACK_PC_RELEASE || rel >= CAN_ACK_PC_SET_PINS)
        return 0;
    return 1;
}

static inline int can_ack_release_pc(uint32_t addr, uint32_t origin) {
    return addr == origin + CAN_ACK_PC_RELEASE;
}

#endif // CAN_PIO_RELOC_H
