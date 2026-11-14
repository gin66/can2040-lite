// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_pico.cpp — RP2040/RP2350 PIO backend.
//
// Two CAN SMs on the selected PIO block (SM0-style rx + tx).  The rx + tx
// programs are 7 + 14 = 21 instructions, inside the 29-word budget that
// leaves 2 words for the FreeRTOS cycle counter SM.
//
// RX uses the runtime pair-capture program (edge pairs, 7 words) with a
// FIFO-not-empty IRQ.  Each word holds the 16-bit timestamps of a 1->0 edge
// and the following 0->1 edge; can_edge_pair_push() turns them into per-edge
// pushes for can_edge_sm.  The application installs the PIO IRQ handler and calls
// can2040_lite_pio_irq_handler(), which this backend defines as a direct
// can2040_lite_pico_rx_irq() call.  The backend poll() reports a finished
// TX and then the parsed frames for the main-loop message processor.

#include <Arduino.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>

#include "can2040_lite_pico.h"
#include "can2040_lite_pico_internal.h"
#include "can2040_lite_backend.h"
#include "can2040_lite.h"    // full struct can2040_lite definition
#include "can2040_lite_proto.h"
#include "can_edge_sm.h"
#include "can_rx_sm.h"
#include "can_rx_sm_inline.h"
#include "can_pio_program.h"
#include "can_pio_reloc.h"
// TX is exactly 32 PIO cycles per bit; autopull refills the OSR for free so
// there is no word-boundary jitter.  RX counts two-cycle ticks in X, so the
// divider and ticks_per_bit are chosen together to make that count exact.
static const float tx_clk_mult = 32.0f;

// TX is one-shot per CAN message.  The program raises one of the configured
// flags and parks; software must re-prepare the SM for the next message.
// The flag numbers live in can2040_lite_config.h so the runtime program
// builder and this backend cannot disagree.  RP2040 can route only flags
// 0..3 to the system IRQ; RP2350 has 0..7.
static inline pio_interrupt_source_t tx_irq_source(uint flag) {
    return (pio_interrupt_source_t)((uint)pis_interrupt0 + flag);
}
#if defined(PICO_PIO_VERSION) && PICO_PIO_VERSION == 0
static_assert(CAN_PIO_TX_DONE_IRQ <= 3u && CAN_PIO_TX_ERR_IRQ <= 3u,
              "RP2040 can route only PIO IRQ flags 0..3");
#endif

// Joined TX FIFO depth.  One entry is the bit count, and one recessive word
// is reserved so the OUT that empties the OSR can autopull instead of stalling.
// A worst-case classic message is 6 words, so count + message + pad fit.
static const uint CAN_TX_FIFO_WORDS = 8;

static_assert(CAN_PIO_RX_PAIR_LENGTH + CAN_PIO_TX_LENGTH <= 29,
              "rx_pair + tx PIO programs must fit in 29 words");
#if CAN_ACK_GENERATION_SUPPORT
static_assert(CAN_PIO_ACK_LENGTH <= CAN_PIO_TX_LENGTH,
              "ack PIO image must fit the TX instruction slot");
static_assert(CAN_PIO_RX_PAIR_LENGTH + CAN_PIO_ACK_LENGTH <= 29,
              "rx_pair + ack PIO programs must fit in 29 words");
#endif

// ACK image addresses are program-relative.  SMx_ADDR is absolute; add
// tx_offset before comparing.  See can_pio_reloc.h.

// The ACK image is clocked like TX.  The plan's gap is whole bit times; the
// model adds two cycles so the inset drive starts at slot + 0.25 bit:
//   set pins,0[30] starts at gap_bits*32 + 2 + 6 = slot + 8 cycles.
static const uint32_t ACK_CYCLES_PER_BIT = 32;
static const uint32_t ACK_GAP_OFFSET_CYCLES = 2;

static PIO backend_pio = pio0;   // selected by start() from cd->pio_num
static PIO rx_pio;
static PIO tx_pio;
static uint rx_sm;
static uint tx_sm;
static uint rx_offset;
static uint tx_offset;
static uint tx_gpio;
static uint tx_rx_gpio;
static pio_sm_config rx_cfg;
static pio_sm_config tx_cfg;
#if CAN_ACK_GENERATION_SUPPORT
static pio_sm_config ack_cfg;
#endif
static bool tx_initialized;

// ACK helper state.  The helper is resident on the TX instruction slot while
// this node is not transmitting.  It follows the RX pad and fires only after
// the receive parser allows it.
#if CAN_ACK_GENERATION_SUPPORT
static bool ack_image_loaded;     // TX slot currently holds the ACK image
static bool ack_helper_ok;        // helper is enabled and following the pad
#endif
static bool tx_active;
#if CAN_ACK_GENERATION_SUPPORT
static bool tx_done_seen;
#endif
static bool tx_finished_reported;

// Edge-fed RX parser and 8-slot frame ring.
static can_rx_t rx;

// Set when the driver core owns the RX path; NULL on the direct/test path.
static struct can2040_lite *backend_cd;

static uint32_t rx_rep_crc;
static uint32_t rx_rep_stuff;
static uint32_t rx_rep_bit;
static uint32_t rx_rep_overflow;

#if CAN_EDGE_PAIR_TIME_RECORDING
#define CAN_PICO_RAW_RECORD_WORDS 256u
#define CAN_PICO_RAW_RECORD_MASK (CAN_PICO_RAW_RECORD_WORDS - 1u)
static uint32_t rx_raw_record[CAN_PICO_RAW_RECORD_WORDS];
static volatile uint32_t rx_raw_record_head;
#endif

// Latest frame converted for the direct RX API.
static can_parsed_t latest_parsed;
static bool latest_parsed_valid;

// RX capture state.
static uint32_t rx_ticks_per_bit;
static uint32_t rx_clkdiv_256;   // PIO divider in 1/256 units
static uint rx_irq_num;
static bool rx_initialized;

// Events reported by poll() to the driver core.  pico_service_events()
// fills the ring from the RX FIFO, the quiet tail, and the TX done/error flags.
static struct can2040_lite_event ev_ring[16];
static uint ev_head;
static uint ev_tail;

extern "C" int can2040_lite_pico_tx_done_pending(void);
extern "C" int can2040_lite_pico_tx_error_pending(void);
#if CAN_ACK_GENERATION_SUPPORT
static void ack_start_follow(void);
static void ack_abort_for_tx(void);
#endif
static void pico_service_events(void);

// Both the ACK helper and the one-shot TX launcher run from the RX ISR (the
// driver core calls tx_frame() after a completed frame).  Never wait on
// the PIO without a bound: a stalled EXEC or a full FIFO must fail the
// operation, not the interrupt.
static bool pio_put_nb(uint32_t word) {
    if (pio_sm_is_tx_fifo_full(tx_pio, tx_sm))
        return false;
    pio_sm_put(tx_pio, tx_sm, word);
    return true;
}

static bool pio_exec_bounded(uint32_t instr) {
    uint spins = 0;

    pio_sm_exec(tx_pio, tx_sm, instr);
    while (pio_sm_is_exec_stalled(tx_pio, tx_sm)) {
        if (++spins > 8u)
            return false;
    }
    return true;
}

static float clkdiv_for(uint32_t bitrate, float mult) {
    return (float)clock_get_hz(clk_sys) / (mult * (float)bitrate);
}

static pio_program_t pio_program_of(const can_pio_program_t *p) {
    pio_program_t prog = {};

    prog.instructions = p->code;
    prog.length = p->length;
    prog.origin = -1;
    prog.pio_version = 0;
#if PICO_PIO_VERSION > 0
    prog.used_gpio_ranges = 0;
#endif
    return prog;
}

// Choose a divider and a whole number of two-cycle ticks per bit.  The PIO
// clock is 2 * ticks_per_bit * bitrate, close to the system clock, and the
// 1/256 divider keeps a bit an exact whole number of ticks.
static bool rx_clock_config(uint32_t bitrate, uint32_t *ticks,
                            uint32_t *div_256) {
    uint32_t hz = clock_get_hz(clk_sys);
    uint64_t n = (uint64_t)hz / (2u * (uint64_t)bitrate);
    uint64_t d;

    if (n == 0 || bitrate == 0)
        return false;
    d = (128ull * hz + (n * bitrate) / 2u) / (n * bitrate);
    if (d < 256u)
        d = 256u;
    if (d > 0xffffu * 256u + 255u)
        return false;
    *ticks = (uint32_t)n;
    *div_256 = (uint32_t)d;
    return true;
}

extern "C" int can2040_lite_pico_rx_init(uint32_t gpio, uint32_t bitrate) {
    const can_pio_program_t *prog = can_pio_rx_pair_program();
    pio_program_t pio_prog = pio_program_of(prog);
    int sm;
    int off;

    rx_pio = backend_pio;
    sm = pio_claim_unused_sm(rx_pio, false);
    if (sm < 0) {
        Serial.println("rx init: no free SM");
        return -1;
    }
    rx_sm = (uint)sm;
    off = pio_add_program(rx_pio, &pio_prog);
    if (off < 0) {
        Serial.println("rx init: no program space");
        return -1;
    }
    rx_offset = (uint)off;

    if (!rx_clock_config(bitrate, &rx_ticks_per_bit, &rx_clkdiv_256)) {
        Serial.println("rx init: bad bitrate");
        return -1;
    }
    // Edge-pair RX: shift-left with autopush at 32 bits.  One FIFO word is
    // (time[1->0] << 16) | time[0->1], each the low 16 bits of X at the edge.
    rx_cfg = pio_get_default_sm_config();
    sm_config_set_wrap(&rx_cfg, rx_offset + prog->wrap_target,
                       rx_offset + prog->wrap);
    sm_config_set_in_pins(&rx_cfg, gpio);
    sm_config_set_jmp_pin(&rx_cfg, gpio);
    sm_config_set_in_shift(&rx_cfg, false, true, 32); // shift left, autopush at 32
    sm_config_set_clkdiv_int_frac(&rx_cfg, (uint16_t)(rx_clkdiv_256 >> 8),
                                  (uint8_t)(rx_clkdiv_256 & 0xffu));
    sm_config_set_fifo_join(&rx_cfg, PIO_FIFO_JOIN_RX); // 8-deep joined RX
    can_rx_init(&rx, rx_ticks_per_bit);
    rx_rep_crc = rx_rep_stuff = rx_rep_bit = rx_rep_overflow = 0;

    pio_gpio_init(rx_pio, gpio);
    // Leave the pin direction alone: with loopback it is already an output.

    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);
    pio_sm_set_enabled(rx_pio, rx_sm, false);

    // Route this SM's RX-FIFO-not-empty source to PIO IRQ 0, the IRQ
    // can2040's sample startup enables.  The application owns and installs
    // the system handler; the backend only configures the source.
    {
        int irq = pio_get_irq_num(rx_pio, 0);
        if (irq < 0) {
            Serial.println("rx init: no PIO IRQ");
            return -1;
        }
        rx_irq_num = (uint)irq;
    }
    pio_set_irq0_source_enabled(
        rx_pio, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + rx_sm),
        true);
    irq_set_enabled(rx_irq_num, true);

    rx_initialized = true;
    return 0;
}

extern "C" int can2040_lite_pico_tx_init(uint32_t gpio, uint32_t gpio_rx,
                                         uint32_t bitrate) {
    const can_pio_program_t *prog = can_pio_tx_program();
    pio_program_t pio_prog = pio_program_of(prog);
    int sm;
    int off;

    tx_pio = backend_pio;
    sm = pio_claim_unused_sm(tx_pio, false);
    if (sm < 0) {
        Serial.println("tx init: no free SM");
        return -1;
    }
    tx_sm = (uint)sm;
    off = pio_add_program(tx_pio, &pio_prog);
    if (off < 0) {
        Serial.println("tx init: no program space");
        return -1;
    }
    tx_offset = (uint)off;
    tx_gpio = gpio;
    pio_interrupt_clear(tx_pio, CAN_PIO_TX_DONE_IRQ);
    pio_interrupt_clear(tx_pio, CAN_PIO_TX_ERR_IRQ);

    tx_cfg = pio_get_default_sm_config();
    sm_config_set_wrap(&tx_cfg, tx_offset + prog->wrap_target,
                       tx_offset + prog->wrap);
    sm_config_set_out_pins(&tx_cfg, gpio, 1);
    sm_config_set_set_pins(&tx_cfg, gpio, 1);
    sm_config_set_jmp_pin(&tx_cfg, gpio_rx);
    sm_config_set_out_shift(&tx_cfg, false, true, 32); // shift left, autopull
    sm_config_set_fifo_join(&tx_cfg, PIO_FIFO_JOIN_TX); // 8-deep TX FIFO
    sm_config_set_clkdiv(&tx_cfg, clkdiv_for(bitrate, tx_clk_mult));

    tx_rx_gpio = gpio_rx;
#if CAN_ACK_GENERATION_SUPPORT
    // The ACK image shares the TX slot.  It follows the RX pad and uses the
    // same SET/OUT and clock settings; only the pull is manual.
    {
        const can_pio_program_t *ack = can_pio_ack_program();

        ack_cfg = pio_get_default_sm_config();
        sm_config_set_wrap(&ack_cfg, tx_offset + ack->wrap_target,
                           tx_offset + ack->wrap);
    }
    sm_config_set_out_pins(&ack_cfg, gpio, 1);
    sm_config_set_set_pins(&ack_cfg, gpio, 1);
    sm_config_set_jmp_pin(&ack_cfg, gpio_rx);
    sm_config_set_out_shift(&ack_cfg, false, false, 32); // manual pull
    sm_config_set_fifo_join(&ack_cfg, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&ack_cfg, clkdiv_for(bitrate, tx_clk_mult));
#endif

    pio_gpio_init(tx_pio, gpio);
    if (gpio_rx != gpio)
        pio_gpio_init(tx_pio, gpio_rx);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, gpio, 1, true);

    pio_sm_init(tx_pio, tx_sm, tx_offset, &tx_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << gpio, 1u << gpio);
    pio_sm_set_enabled(tx_pio, tx_sm, false); // one-shot: started by tx_frame()

    // TX done/error flags are latched and polled from the main service.
    // Do not route them to the RX IRQ; the RX ISR must stay RX-only.
    pio_set_irq0_source_enabled(tx_pio, tx_irq_source(CAN_PIO_TX_DONE_IRQ), false);
    pio_set_irq0_source_enabled(tx_pio, tx_irq_source(CAN_PIO_TX_ERR_IRQ), false);

    tx_initialized = true;
    tx_active = false;
#if CAN_ACK_GENERATION_SUPPORT
    tx_done_seen = false;
    ack_image_loaded = false;
    ack_start_follow();                       // receive helper at idle
#endif
    return 0;
}

static struct can2040_lite_event *claim_event(void) {
    uint next = (ev_tail + 1u) % (uint)(sizeof(ev_ring) / sizeof(ev_ring[0]));

    if (next == ev_head)
        return NULL;
    return &ev_ring[ev_tail];
}

static void publish_event(void) {
    ev_tail = (ev_tail + 1u) % (uint)(sizeof(ev_ring) / sizeof(ev_ring[0]));
}

static int pop_event(struct can2040_lite_event *ev) {
    if (ev_head == ev_tail)
        return 0;
    *ev = ev_ring[ev_head];
    ev_head = (ev_head + 1u) % (uint)(sizeof(ev_ring) / sizeof(ev_ring[0]));
    return 1;
}

static void rx_convert(const can_rx_frame_t *f, can_parsed_t *p) {
    p->id = can_rx_frame_id(f);
    p->dlc = f->dlc;
    p->is_extended = (uint8_t)(can_rx_frame_flags(f) & 1u);
    p->is_rtr = 0;
    p->ack = f->ack;
    memcpy(p->data, f->data, sizeof(p->data));
    p->crc = 0;
    p->crc_calc = 0;
    p->validity = VALID_FRAME;
    p->stuffed_bits = 0;
    p->total_bits = 0;
}

static void rx_sync_stats(void) {
    struct can2040_lite_stats *st;
    uint32_t crc, stuff, bit, overflow;
    uint32_t save, d;

    if (!backend_cd)
        return;
    save = save_and_disable_interrupts();
    crc = rx.crc_errors;
    stuff = rx.stuff_errors;
    bit = rx.bit_errors;
    overflow = rx.overflow;
    restore_interrupts(save);

    st = &backend_cd->stats;
    d = crc - rx_rep_crc;
    if (d) { st->crc_errors += d; st->parse_error += d; rx_rep_crc = crc; }
    d = stuff - rx_rep_stuff;
    if (d) { st->stuff_errors += d; st->parse_error += d; rx_rep_stuff = stuff; }
    d = bit - rx_rep_bit;
    if (d) { st->bit_errors += d; st->parse_error += d; rx_rep_bit = bit; }
    d = overflow - rx_rep_overflow;
    if (d) { st->rx_overflow += d; rx_rep_overflow = overflow; }
}

// Driver-core path: convert ring slots to events.  The direct Pico API keeps
// the ring for itself.
static void rx_drain_events(void) {
    if (!backend_cd)
        return;
    while (can_rx_available(&rx)) {
        struct can2040_lite_event *slot = claim_event();

        if (!slot)
            return;
        slot->kind = 2;
        slot->tx_error = 0;
        rx_convert(can_rx_front(&rx), &slot->rx);
        publish_event();
        can_rx_pop(&rx);
    }
}

// Copy one image into the fixed TX instruction slot.  The caller has already
// disabled the SM.  JMP targets in the assembled arrays are relative to
// origin 0; relocate them the same way pio_add_program does.
#if CAN_ACK_GENERATION_SUPPORT
static void ack_copy_image(const uint16_t *insns, uint len) {
    for (uint i = 0; i < len; i++)
        tx_pio->instr_mem[tx_offset + i] =
            can_pio_reloc_instr(insns[i], tx_offset);
}

// Restart the helper in follow mode at the RX pad's current level.  Used at
// init, after rx re-arm, and once this node's own transmit has passed its ACK
// slot.  Leaves the SM halted and ack_helper_ok false on failure; ack_service()
// retries.
static void ack_start_follow(void) {
    uint32_t save;
    uint32_t pc;
    uint8_t level;
    bool ok;

    if (!tx_initialized)
        return;
    save = save_and_disable_interrupts();
    pio_sm_set_enabled(tx_pio, tx_sm, false);
    ack_helper_ok = false;
    if (!ack_image_loaded) {
        const can_pio_program_t *ack = can_pio_ack_program();

        ack_copy_image(ack->code, ack->length);
        ack_image_loaded = true;
    }

    level = gpio_get(tx_rx_gpio) ? 1u : 0u;
    pc = tx_offset + (level ? CAN_ACK_ENTRY_HIGH : CAN_ACK_ENTRY_LOW);
    pio_sm_init(tx_pio, tx_sm, pc, &ack_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << tx_gpio, 1u << tx_gpio);
    pio_sm_clear_fifos(tx_pio, tx_sm);

    // Follow with X = 0xFFFFFFFF and Y = 0.
    ok = pio_put_nb(0xffffffffu) &&
         pio_exec_bounded(pio_encode_pull(false, false)) &&
         pio_exec_bounded(pio_encode_out(pio_x, 32)) &&
         pio_exec_bounded(pio_encode_set(pio_y, 0));

    if (ok) {
        pio_sm_set_enabled(tx_pio, tx_sm, true);
        ack_helper_ok = true;
    }
    restore_interrupts(save);
}

// Abort the helper before loading can_tx.  The pad is forced recessive first,
// so an in-flight receive is NACKed rather than ACKed by a stale arm.
static void ack_abort_for_tx(void) {
    uint32_t save;

    if (!tx_initialized)
        return;
    save = save_and_disable_interrupts();
    if (ack_image_loaded) {
        const can_pio_program_t *tx = can_pio_tx_program();

        (void)pio_exec_bounded(pio_encode_set(pio_pins, 1));
        pio_sm_set_enabled(tx_pio, tx_sm, false);
        ack_copy_image(tx->code, tx->length);
        ack_image_loaded = false;
    }
    ack_helper_ok = false;
    restore_interrupts(save);
}

// Reload the helper after this node's own transmit, or restart the follow
// loop after a completed ACK slot.  Called from the ISR and the polling path.
static void ack_service(void) {
    if (!tx_initialized)
        return;

    if (tx_active) {
        if (!tx_done_seen &&
            (can2040_lite_pico_tx_done_pending() ||
             can2040_lite_pico_tx_error_pending()))
            tx_done_seen = true;
        if (tx_done_seen && can_edge_sm_past_ack_slot(&edge_sm)) {
            tx_active = false;
            ack_start_follow();
        }
        return;
    }

    // Retry a failed restart, and otherwise reload only after the helper has
    // driven its pulse and the parser is past the ACK slot.
    if (!ack_helper_ok ||
        (can_ack_release_pc(tx_pio->sm[tx_sm].addr, tx_offset) &&
         can_edge_sm_past_ack_slot(&edge_sm)))
        ack_start_follow();
}
#endif // CAN_ACK_GENERATION_SUPPORT

// Fast RX IRQ: FIFO drain + inline parser only.
extern "C" void can2040_lite_pico_rx_irq(void) {
    while (!pio_sm_is_rx_fifo_empty(rx_pio, rx_sm)) {
        const uint32_t word = pio_sm_get(rx_pio, rx_sm);

#if CAN_EDGE_PAIR_TIME_RECORDING
        rx_raw_record[rx_raw_record_head & CAN_PICO_RAW_RECORD_MASK] = word;
        rx_raw_record_head++;
#endif
        can_rx_pair_inline(&rx, word);
    }
}

// Main-loop service.  The fast RX IRQ owns the FIFO and parser;
// this function only handles TX flags, the quiet CRC tail, and ring->event
// conversion.
static void pico_service_events(void) {
    if (!rx_initialized)
        return;

    // TX flags first: the matching ACK/NACK is in the parsed own frame.
    if (tx_initialized && tx_active && !tx_finished_reported) {
        int err = can2040_lite_pico_tx_error_pending();
        int done = can2040_lite_pico_tx_done_pending();
        if (err || done) {
            struct can2040_lite_event *slot = claim_event();

            if (slot) {
                slot->kind = 1;
                slot->tx_error = err ? 1 : 0;
                publish_event();
            }
            can2040_lite_pico_tx_clear_status();
            tx_finished_reported = true;
#if CAN_ACK_GENERATION_SUPPORT
            tx_done_seen = true;
#else
            tx_active = false;
#endif
        }
    }

    if (rx.crc_errors != rx_rep_crc || rx.stuff_errors != rx_rep_stuff ||
        rx.bit_errors != rx_rep_bit || rx.overflow != rx_rep_overflow)
        rx_sync_stats();
    if (rx.rx_idx != rx.tx_idx)
        rx_drain_events();
}

static int pico_poll(struct can2040_lite_event *ev) {
    pico_service_events();
    return pop_event(ev);
}

#ifdef CAN2040_LITE_TEST
#include "can2040_lite_pico_test.inc"
#endif

extern "C" void can2040_lite_pico_rx_arm(void) {
    if (!rx_initialized)
        return;

    irq_set_enabled(rx_irq_num, false);
    can_rx_reset(&rx);
    latest_parsed_valid = false;
#ifdef CAN2040_LITE_TEST
    can2040_lite_pico_rx_record_clear();
#endif
    ev_head = 0;
    ev_tail = 0;

    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_restart(rx_pio, rx_sm);
    // pio_sm_init sets the PC to the program offset, `high_cnt`.  The caller
    // arms while the bus is recessive, so the first captured 1->0 is SOF.  If
    // an edge slips in during the arming sequence, that frame is lost to CRC;
    // the parser resyncs at the next SOF.  X needs no seeding.
    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);
    pio_sm_set_enabled(rx_pio, rx_sm, true);
    irq_set_enabled(rx_irq_num, true);

    // The helper follows only while this node is not transmitting.
#if CAN_ACK_GENERATION_SUPPORT
    if (!tx_active)
        ack_start_follow();
#endif
}


extern "C" int can2040_lite_pico_tx_done_pending(void) {
    return pio_interrupt_get(tx_pio, CAN_PIO_TX_DONE_IRQ) ? 1 : 0;
}

extern "C" int can2040_lite_pico_tx_error_pending(void) {
    return pio_interrupt_get(tx_pio, CAN_PIO_TX_ERR_IRQ) ? 1 : 0;
}

extern "C" void can2040_lite_pico_tx_clear_status(void) {
    pio_interrupt_clear(tx_pio, CAN_PIO_TX_DONE_IRQ);
    pio_interrupt_clear(tx_pio, CAN_PIO_TX_ERR_IRQ);
}

// Backend entry point for the public startup sequence.  The application
// installs and enables the PIO IRQ handler; the backend only configures the
// block, the SMs, and the interrupt sources.
static int pico_hw_start(struct can2040_lite *cd, uint32_t sys_clock,
                         uint32_t bitrate, int32_t gpio_rx,
                         int32_t gpio_tx) {
    (void)sys_clock;

    // stop() releases the SMs.  A second start() without a stop() must not
    // claim another pair.
    if (rx_initialized)
        return 0;

    backend_pio = (cd && cd->pio_num != 0) ? pio1 : pio0;
    if (gpio_rx < 0)
        return -1;

    if (can2040_lite_pico_rx_init((uint32_t)gpio_rx, bitrate) < 0)
        return -1;
    backend_cd = cd;
    if (gpio_tx >= 0) {
        if (can2040_lite_pico_tx_init((uint32_t)gpio_tx, (uint32_t)gpio_rx,
                                      bitrate) < 0)
            return -1;
    }
    can2040_lite_pico_rx_arm();
    return 0;
}

static void pico_hw_stop(void) {
    backend_cd = NULL;
    if (rx_initialized) {
        pio_program_t prog = pio_program_of(can_pio_rx_pair_program());

        pio_sm_set_enabled(rx_pio, rx_sm, false);
        pio_set_irq0_source_enabled(
            rx_pio,
            (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + rx_sm),
            false);
        pio_sm_unclaim(rx_pio, rx_sm);
        pio_remove_program(rx_pio, &prog, rx_offset);
        rx_initialized = false;
    }
    if (tx_initialized) {
        pio_program_t prog = pio_program_of(can_pio_tx_program());

        pio_sm_set_enabled(tx_pio, tx_sm, false);
        pio_set_irq0_source_enabled(tx_pio, tx_irq_source(CAN_PIO_TX_DONE_IRQ), false);
        pio_set_irq0_source_enabled(tx_pio, tx_irq_source(CAN_PIO_TX_ERR_IRQ), false);
        pio_sm_unclaim(tx_pio, tx_sm);
        pio_remove_program(tx_pio, &prog, tx_offset);
        tx_initialized = false;
#if CAN_ACK_GENERATION_SUPPORT
        ack_image_loaded = false;
#endif
        tx_active = false;
        tx_finished_reported = false;
    }
    ev_head = 0;
    ev_tail = 0;
}

extern "C" void can2040_lite_pio_irq_handler(struct can2040_lite *cd) {
    (void)cd;
    can2040_lite_pico_rx_irq();
}

// Fixed backend interface.  One CAN interface per MCU, so only start() needs
// the driver instance (for pio_num and the event/stats mapping).
extern "C" int can2040_lite_backend_start(struct can2040_lite *cd,
                                          uint32_t sys_clock,
                                          uint32_t bitrate, int32_t gpio_rx,
                                          int32_t gpio_tx) {
    return pico_hw_start(cd, sys_clock, bitrate, gpio_rx, gpio_tx);
}

extern "C" void can2040_lite_backend_stop(void) {
    pico_hw_stop();
}

extern "C" int can2040_lite_backend_tx_frame(const uint8_t *bits,
                                             uint32_t nbits) {
    return can2040_lite_pico_tx_frame(bits, nbits);
}

extern "C" int can2040_lite_backend_poll(struct can2040_lite_event *ev) {
    return pico_poll(ev);
}

extern "C" uint32_t can2040_lite_critical_enter(void) {
    return save_and_disable_interrupts();
}

extern "C" void can2040_lite_critical_exit(uint32_t token) {
    restore_interrupts(token);
}

// Prepare and start one message.  `nbits` covers SOF through the recessive
// bit after the CRC.  That bit is driven and the SM parks.  Y is the bit
// count, the OSR is preloaded with the first word, and the rest of the
// joined FIFO holds the remaining words.  Returns 0 if the SM accepted it.
extern "C" int can2040_lite_pico_tx_frame(const uint8_t *bits,
                                          uint32_t nbits) {
    const can_pio_program_t *tx = can_pio_tx_program();
    uint32_t words = (nbits + 31u) / 32u;

    // Count word plus the autopull pad occupy two FIFO entries.
    if (!bits || nbits == 0 || words + 2u > CAN_TX_FIFO_WORDS)
        return -1;

    // An in-flight receive is NACKed: force the pad recessive, swap in the
    // TX image, then run the normal transmit path.
#if CAN_ACK_GENERATION_SUPPORT
    ack_abort_for_tx();
#endif
    tx_active = true;
#if CAN_ACK_GENERATION_SUPPORT
    tx_done_seen = false;
#endif
    tx_finished_reported = false;

    // Leaving the SM disabled and the pad recessive keeps a failed launch
    // fail-safe; the queue head stays queued for the next tx_kick().
    auto launch_fail = [&]() {
        pio_sm_set_enabled(tx_pio, tx_sm, false);
        pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << tx_gpio, 1u << tx_gpio);
        tx_active = false;
        return -1;
    };

    // Halt the SM, clear the FIFOs/shift counters and point it at `reload`.
    // Restart leaves the OSR shift count at 32 (empty), so the pull below
    // is not the autopull no-op that a full OSR would be.  X is set by the
    // first instruction after enable; restart does not clear it.
    // `wrap` is the `set x, 22` that arms the idle count. Enabling the SM
    // runs it once, then the program wraps to the recessive poll.
    pio_sm_init(tx_pio, tx_sm, tx_offset + tx->wrap, &tx_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << tx_gpio, 1u << tx_gpio);

    // jmp y-- falls through on the bit where Y was already 0, so the count
    // loaded here is one less than the number of bits to shift.
    if (!pio_put_nb(nbits - 1u))
        return launch_fail();

    // Raw on-wire bits, MSB-first. Bits past nbits are never shifted.
    for (uint32_t w = 0; w < words; w++) {
        uint32_t word = 0;
        for (uint32_t i = 0; i < 32u; i++) {
            uint32_t pos = w * 32u + i;
            uint8_t bit = (pos < nbits) ? bits_get(bits, pos) : 1;
            word = (word << 1) | bit;
        }
        if (!pio_put_nb(word))
            return launch_fail();
    }
    // The OUT that empties the OSR autopulls. Without a spare word that
    // OUT stalls when nbits is a multiple of 32, and DONE never runs.
    if (!pio_put_nb(0xffffffffu))
        return launch_fail();

    // Move the count into Y and autopull the first message word, so the SOF
    // bit has no stall and every bit stays exactly 32 cycles.
    if (!pio_exec_bounded(pio_encode_pull(false, true)) ||
        !pio_exec_bounded(pio_encode_mov(pio_y, pio_osr)) ||
        !pio_exec_bounded(pio_encode_out(pio_null, 32)))
        return launch_fail();

    can2040_lite_pico_tx_clear_status();
    pio_sm_set_enabled(tx_pio, tx_sm, true);
    return 0;
}
