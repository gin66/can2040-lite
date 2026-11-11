// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_pico.cpp — RP2040/RP2350 PIO backend.
//
// Two CAN SMs on PIO0 (SM0 rx, SM1 tx).  The rx + tx programs are 7 + 14 =
// 21 instructions, inside the 29-word budget that leaves 2 words for the
// FreeRTOS cycle counter SM.
//
// RX uses rx_edge.pio (edge-timestamped, 7 words) with a FIFO-not-empty IRQ
// that feeds each word into can_edge_sm.  When a frame completes, the IRQ
// saves it and dispatches the callback.

#include <Arduino.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/sync.h>

#include "can2040_lite_pico.h"
#include "can2040_lite.h"    // full struct can2040_lite definition
#include "can2040_lite_proto.h"
#include "can_edge_sm.h"
#include "rx_edge.pio.h"
#include "tx.pio.h"

// TX is exactly 32 PIO cycles per bit; autopull refills the OSR for free so
// there is no word-boundary jitter.  RX counts two-cycle ticks in X, so the
// divider and ticks_per_bit are chosen together to make that count exact.
static const float tx_clk_mult = 32.0f;

// TX is one-shot per CAN message.  The program raises one of these IRQs and
// parks; software must re-prepare the SM for the next message.
static const uint CAN_TX_DONE_IRQ = 6;
static const uint CAN_TX_ERR_IRQ = 7;

// Joined TX FIFO depth.  One entry is the bit count, and one recessive word
// is reserved so the OUT that empties the OSR can autopull instead of stalling.
// A worst-case classic message is 6 words, so count + message + pad fit.
static const uint CAN_TX_FIFO_WORDS = 8;

static_assert(can_rx_edge_program.length + can_tx_program.length <= 29,
              "rx_edge + tx PIO programs must fit in 29 words");

static PIO rx_pio;
static PIO tx_pio;
static uint rx_sm;
static uint tx_sm;
static uint rx_offset;
static uint tx_offset;
static uint tx_gpio;
static pio_sm_config rx_cfg;
static pio_sm_config tx_cfg;

// Edge state machine — processes FIFO words from the RX SM.
static can_edge_sm_t edge_sm;

// Flag set by the IRQ handler when a frame is fully parsed.
static volatile bool frame_ready_flag = false;

// The latest parsed frame (valid when frame_ready_flag is true).
static can_parsed_t latest_parsed;

// RX capture state.  rx_bias converts the raw X count into the ideal
// two-cycle clock: each four-cycle edge path skips two decrements, so every
// edge is another two ticks early.
static uint rx_gpio;
static uint32_t rx_sys_hz;
static uint32_t rx_ticks_per_bit;
static uint32_t rx_clkdiv_256;   // PIO divider in 1/256 units
static uint32_t rx_bias;
static uint32_t rx_last_time;
static uint32_t rx_last_us;
static uint rx_irq_num;
static bool rx_initialized;

static void can2040_lite_pico_rx_isr(void);

static float clkdiv_for(uint32_t bitrate, float mult) {
    return (float)clock_get_hz(clk_sys) / (mult * (float)bitrate);
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
    int sm;
    int off;

    rx_pio = pio0;
    sm = pio_claim_unused_sm(rx_pio, false);
    if (sm < 0) {
        Serial.println("rx init: no free SM");
        return -1;
    }
    rx_sm = (uint)sm;
    off = pio_add_program(rx_pio, &can_rx_edge_program);
    if (off < 0) {
        Serial.println("rx init: no program space");
        return -1;
    }
    rx_offset = (uint)off;

    rx_gpio = gpio;
    rx_sys_hz = clock_get_hz(clk_sys);
    if (!rx_clock_config(bitrate, &rx_ticks_per_bit, &rx_clkdiv_256)) {
        Serial.println("rx init: bad bitrate");
        return -1;
    }
    rx_bias = 0;
    rx_last_time = 0;
    rx_last_us = time_us_32();

    // Edge RX: shift-left with autopush at 32 bits.  Each FIFO word is
    // (time << 1) | level, where time is the low 31 bits of the free-
    // running X counter at the moment of the edge.
    rx_cfg = can_rx_edge_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&rx_cfg, gpio);
    sm_config_set_jmp_pin(&rx_cfg, gpio);
    sm_config_set_in_shift(&rx_cfg, false, true, 32); // shift left, autopush at 32
    sm_config_set_clkdiv_int_frac(&rx_cfg, (uint16_t)(rx_clkdiv_256 >> 8),
                                  (uint8_t)(rx_clkdiv_256 & 0xffu));
    sm_config_set_fifo_join(&rx_cfg, PIO_FIFO_JOIN_RX); // 8-deep joined RX
    can_edge_sm_init(&edge_sm, rx_ticks_per_bit);

    pio_gpio_init(rx_pio, gpio);
    // Leave the pin direction alone: with loopback it is already an output.

    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);
    pio_sm_set_enabled(rx_pio, rx_sm, false);

    // Route this SM's RX-FIFO-not-empty source to PIO IRQ 1 and install the
    // system handler.  RX is level based, so there is no flag to clear.
    {
        int irq = pio_get_irq_num(rx_pio, 1);
        if (irq < 0) {
            Serial.println("rx init: no PIO IRQ");
            return -1;
        }
        rx_irq_num = (uint)irq;
    }
    irq_set_exclusive_handler(rx_irq_num, can2040_lite_pico_rx_isr);
    pio_set_irq1_source_enabled(
        rx_pio, (pio_interrupt_source_t)(pis_sm0_rx_fifo_not_empty + rx_sm),
        true);
    irq_set_enabled(rx_irq_num, true);

    // Route the portable IRQ entry to this backend.
    extern void can2040_lite_pico_attach_rx_irq(void (*)(struct can2040_lite *));
    can2040_lite_pico_attach_rx_irq(can2040_lite_pico_rx_irq_handler);
    rx_initialized = true;
    return 0;
}

extern "C" int can2040_lite_pico_tx_init(uint32_t gpio, uint32_t gpio_rx,
                                         uint32_t bitrate) {
    int sm;
    int off;

    tx_pio = pio0;
    sm = pio_claim_unused_sm(tx_pio, false);
    if (sm < 0) {
        Serial.println("tx init: no free SM");
        return -1;
    }
    tx_sm = (uint)sm;
    off = pio_add_program(tx_pio, &can_tx_program);
    if (off < 0) {
        Serial.println("tx init: no program space");
        return -1;
    }
    tx_offset = (uint)off;
    tx_gpio = gpio;
    pio_interrupt_clear(tx_pio, CAN_TX_DONE_IRQ);
    pio_interrupt_clear(tx_pio, CAN_TX_ERR_IRQ);

    tx_cfg = can_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&tx_cfg, gpio, 1);
    sm_config_set_set_pins(&tx_cfg, gpio, 1);
    sm_config_set_jmp_pin(&tx_cfg, gpio_rx);
    sm_config_set_out_shift(&tx_cfg, false, true, 32); // shift left, autopull
    sm_config_set_fifo_join(&tx_cfg, PIO_FIFO_JOIN_TX); // 8-deep TX FIFO
    sm_config_set_clkdiv(&tx_cfg, clkdiv_for(bitrate, tx_clk_mult));

    pio_gpio_init(tx_pio, gpio);
    if (gpio_rx != gpio)
        pio_gpio_init(tx_pio, gpio_rx);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, gpio, 1, true);

    pio_sm_init(tx_pio, tx_sm, tx_offset, &tx_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << gpio, 1u << gpio);
    pio_sm_set_enabled(tx_pio, tx_sm, false); // one-shot: started by tx_frame()
    return 0;
}

static struct can2040_lite *rx_driver;

// Move one finished result out of the state machine.  The state machine
// keeps it readable across the next SOF, but only one result is stored.
static bool rx_take(can_parsed_t *out) {
    if (!edge_sm.frame_ready)
        return false;
    *out = edge_sm.parsed;
    edge_sm.frame_ready = 0;
    return true;
}

static void rx_store(const can_parsed_t *p) {
    memcpy(&latest_parsed, p, sizeof(latest_parsed));
    __sync_synchronize();
    frame_ready_flag = true;
}

static void rx_dispatch(const can_parsed_t *p, struct can2040_lite *cd) {
    if (!cd || !cd->started)
        return;
    if (p->validity == VALID_FRAME)
        cd->stats.rx_total++;
    else if (p->validity == INVALID_CRC)
        cd->stats.crc_errors++;
    else if (p->validity == INVALID_STUFF)
        cd->stats.stuff_errors++;
    else
        cd->stats.bit_errors++;

    if (cd->cb) {
        struct can2040_lite_msg msg;
        memset(&msg, 0, sizeof(msg));
        msg.id = p->id;
        if (p->is_rtr)
            msg.id |= CAN2040_LITE_ID_RTR;
        if (p->is_extended)
            msg.id |= CAN2040_LITE_ID_EFF;
        msg.dlc = p->dlc;
        msg.crc_error = (p->validity != VALID_FRAME) ? 1u : 0u;
        memcpy(msg.data, p->data, sizeof(msg.data));
        cd->cb(cd, CAN2040_LITE_NOTIFY_RX, &msg);
    }
}

// Convert the system-time elapsed since the last captured edge into the
// PIO's two-cycle tick domain.  The single microsecond subtracted keeps the
// estimate behind real time, so it can never run past an edge that has not
// been consumed yet.
static uint32_t rx_estimate_now(void) {
    uint32_t elapsed_us = time_us_32() - rx_last_us;
    uint64_t cycles;
    uint64_t ticks;

    if (elapsed_us)
        elapsed_us--;
    cycles = (uint64_t)elapsed_us * rx_sys_hz / 1000000ull;
    ticks = cycles * 128ull / rx_clkdiv_256;
    return (rx_last_time - (uint32_t)ticks) & CAN_EDGE_TIME_MASK;
}

// Drain every captured edge, then let the quiet tail advance to now so a
// recessive CRC/ACK/EOF tail can finish without a further edge.
static void rx_service(struct can2040_lite *cd) {
    can_parsed_t p;

    if (!rx_initialized)
        return;

    while (!pio_sm_is_rx_fifo_empty(rx_pio, rx_sm)) {
        uint32_t word = pio_sm_get(rx_pio, rx_sm);

        rx_bias += 2u;
        rx_last_time = (((word >> 1) & CAN_EDGE_TIME_MASK) - rx_bias) &
                       CAN_EDGE_TIME_MASK;
        rx_last_us = time_us_32();
        can_edge_sm_push(&edge_sm, can_edge_word(rx_last_time, word & 1u));
        while (rx_take(&p)) {
            rx_store(&p);
            rx_dispatch(&p, cd);
        }
    }

    can_edge_sm_time(&edge_sm, rx_estimate_now());
    while (rx_take(&p)) {
        rx_store(&p);
        rx_dispatch(&p, cd);
    }
}

static void can2040_lite_pico_rx_isr(void) {
    can2040_lite_pio_irq_handler(rx_driver);
}

extern "C" void can2040_lite_pico_rx_arm(void) {
    if (!rx_initialized)
        return;

    irq_set_enabled(rx_irq_num, false);
    can_edge_sm_init(&edge_sm, rx_ticks_per_bit);
    rx_bias = 0;
    rx_last_time = 0;
    rx_last_us = time_us_32();
    frame_ready_flag = false;

    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_restart(rx_pio, rx_sm);
    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);

    // X = 0 is the sentinel the program converts to 0xFFFFFFFF on its first
    // `jmp x--` fall-through.  Entering the loop that matches the current pin
    // makes that fall-through part of the steady loop instead of a false edge.
    pio_sm_exec(rx_pio, rx_sm, pio_encode_set(pio_x, 0));
    if (gpio_get(rx_gpio) == 0)
        pio_sm_exec(rx_pio, rx_sm,
                    pio_encode_jmp(rx_offset + can_rx_edge_offset_low));
    else
        pio_sm_exec(rx_pio, rx_sm,
                    pio_encode_jmp(rx_offset + can_rx_edge_offset_high));
    pio_sm_set_enabled(rx_pio, rx_sm, true);
    irq_set_enabled(rx_irq_num, true);
}

extern "C" int can2040_lite_pico_rx_wait(uint32_t timeout_ms) {
    uint32_t start = millis();

    while (!frame_ready_flag) {
        if (rx_initialized) {
            can_parsed_t p;
            bool got;
            uint32_t save = save_and_disable_interrupts();

            // Only move the quiet tail when no captured edge is waiting;
            // otherwise the ISR will drain it before time advances again.
            if (pio_sm_is_rx_fifo_empty(rx_pio, rx_sm))
                can_edge_sm_time(&edge_sm, rx_estimate_now());
            got = rx_take(&p);
            if (got)
                rx_store(&p);
            restore_interrupts(save);
            if (got) {
                rx_dispatch(&p, rx_driver);
                break;
            }
        }
        if (millis() - start > timeout_ms)
            return 0;
        delay(1);
    }
    return 1;
}

extern "C" const can_parsed_t *can2040_lite_pico_rx_get_parsed(void) {
    if (!frame_ready_flag)
        return NULL;
    __sync_synchronize();
    return &latest_parsed;
}

// Rebuild the on-wire bit stream from the parsed fields.  Stuffing covers
// SOF through the CRC sequence, including a stuff bit when that sequence
// ends on a 5-run; the tail after it is not stuffed.
extern "C" uint32_t can2040_lite_pico_rx_bits(uint8_t *bits,
                                              uint32_t max_bits) {
    if (!frame_ready_flag || !bits)
        return 0;

    const can_parsed_t *p = &latest_parsed;
    uint32_t n = 0;
    uint8_t run = 0, last = 1;  // start recessive

    auto put = [&](uint8_t b) {
        if (n >= max_bits)
            return false;
        bits_put(bits, n++, b);
        return true;
    };
    auto stuff = [&]() {
        if (run < 5)
            return true;
        uint8_t s = (uint8_t)(1u - last);
        if (!put(s))
            return false;
        run = 1;
        last = s;
        return true;
    };
    auto emit = [&](uint32_t val, uint8_t cnt) {
        for (uint8_t i = 0; i < cnt; i++) {
            uint8_t b = (uint8_t)((val >> (cnt - 1u - i)) & 1u);
            if (!stuff() || !put(b))
                return false;
            run = (b == last) ? (uint8_t)(run + 1u) : 1u;
            last = b;
        }
        return true;
    };

    if (!p->is_extended) {
        // SOF, ID[10:0], RTR, IDE=0, r0, DLC[3:0]
        if (!emit(0u, 1))
            return n;
        if (!emit(p->id & 0x7ffu, 11))
            return n;
        if (!emit(p->is_rtr ? 1u : 0u, 1))
            return n;
        if (!emit(0u, 2))
            return n;
        if (!emit(p->dlc & 0x0fu, 4))
            return n;
    } else {
        // SOF, base ID[10:0], SRR=1, IDE=1, extended ID[17:0], RTR, r1, r0, DLC
        if (!emit(0u, 1))
            return n;
        if (!emit((p->id >> 18) & 0x7ffu, 11))
            return n;
        if (!emit(0x3u, 2))
            return n;
        if (!emit(p->id & 0x3ffffu, 18))
            return n;
        if (!emit(p->is_rtr ? 1u : 0u, 1))
            return n;
        if (!emit(0u, 2))
            return n;
        if (!emit(p->dlc & 0x0fu, 4))
            return n;
    }

    for (uint32_t di = 0; !p->is_rtr && di < (uint32_t)p->dlc * 8u; di++) {
        uint8_t b = (uint8_t)((p->data[di >> 3] >> (7u - (di & 7u))) & 1u);
        if (!stuff() || !put(b))
            return n;
        run = (b == last) ? (uint8_t)(run + 1u) : 1u;
        last = b;
    }

    for (uint32_t ci = 0; ci < 15u; ci++) {
        uint8_t b = (uint8_t)((p->crc >> (14u - ci)) & 1u);
        if (!stuff() || !put(b))
            return n;
        run = (b == last) ? (uint8_t)(run + 1u) : 1u;
        last = b;
    }
    if (!stuff())
        return n;

    if (!put(1u))
        return n;                       // CRC delimiter
    if (!put(p->ack ? 0u : 1u))
        return n;                       // ACK slot (dominant = 0)
    if (!put(1u))
        return n;                       // ACK delimiter
    for (int i = 0; i < 10; i++) {      // EOF (7) + IFS (3)
        if (!put(1u))
            return n;
    }
    return n;
}

extern "C" void can2040_lite_pico_tx_set_readback_gpio(uint32_t gpio) {
    pio_gpio_init(tx_pio, gpio);
    pio_sm_set_jmp_pin(tx_pio, tx_sm, gpio);
    sm_config_set_jmp_pin(&tx_cfg, gpio);
}

extern "C" int can2040_lite_pico_tx_done_pending(void) {
    return pio_interrupt_get(tx_pio, CAN_TX_DONE_IRQ) ? 1 : 0;
}

extern "C" int can2040_lite_pico_tx_error_pending(void) {
    return pio_interrupt_get(tx_pio, CAN_TX_ERR_IRQ) ? 1 : 0;
}

extern "C" void can2040_lite_pico_tx_clear_status(void) {
    pio_interrupt_clear(tx_pio, CAN_TX_DONE_IRQ);
    pio_interrupt_clear(tx_pio, CAN_TX_ERR_IRQ);
}

// Backend entry called by the portable IRQ hook.  Remember a driver pointer
// when one is supplied so later quiet-tail results can still be dispatched.
extern "C" void can2040_lite_pico_rx_irq_handler(
    struct can2040_lite *cd) {
    if (cd)
        rx_driver = cd;
    rx_service(cd ? cd : rx_driver);
}

// Prepare and start one message.  `nbits` covers SOF through the recessive
// bit after the CRC.  That bit is driven and the SM parks.  Y is the bit
// count, the OSR is preloaded with the first word, and the rest of the
// joined FIFO holds the remaining words.  The caller must not call this
// while a message is running.
extern "C" void can2040_lite_pico_tx_frame(const uint8_t *bits,
                                           uint32_t nbits) {
    uint32_t words = (nbits + 31u) / 32u;

    // Count word plus the autopull pad occupy two FIFO entries.
    if (!bits || nbits == 0 || words + 2u > CAN_TX_FIFO_WORDS)
        return;

    // Halt the SM, clear the FIFOs/shift counters and point it at `reload`.
    // Restart leaves the OSR shift count at 32 (empty), so the pull below
    // is not the autopull no-op that a full OSR would be.  X is set by the
    // first instruction after enable; restart does not clear it.
    // `can_tx_wrap` is the `set x, 22` that arms the idle count. Enabling the
    // SM runs it once, then the program wraps to the recessive poll.
    pio_sm_init(tx_pio, tx_sm, tx_offset + can_tx_wrap, &tx_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << tx_gpio, 1u << tx_gpio);

    // jmp y-- falls through on the bit where Y was already 0, so the count
    // loaded here is one less than the number of bits to shift.
    pio_sm_put_blocking(tx_pio, tx_sm, nbits - 1u);

    // Raw on-wire bits, MSB-first. Bits past nbits are never shifted.
    for (uint32_t w = 0; w < words; w++) {
        uint32_t word = 0;
        for (uint32_t i = 0; i < 32u; i++) {
            uint32_t pos = w * 32u + i;
            uint8_t bit = (pos < nbits) ? bits_get(bits, pos) : 1;
            word = (word << 1) | bit;
        }
        pio_sm_put_blocking(tx_pio, tx_sm, word);
    }
    // The OUT that empties the OSR autopulls. Without a spare word that
    // OUT stalls when nbits is a multiple of 32, and DONE never runs.
    pio_sm_put_blocking(tx_pio, tx_sm, 0xffffffffu);

    // Move the count into Y and autopull the first message word, so the SOF
    // bit has no stall and every bit stays exactly 32 cycles.
    pio_sm_exec_wait_blocking(tx_pio, tx_sm, pio_encode_pull(false, true));
    pio_sm_exec_wait_blocking(tx_pio, tx_sm, pio_encode_mov(pio_y, pio_osr));
    pio_sm_exec_wait_blocking(tx_pio, tx_sm, pio_encode_out(pio_null, 32));

    can2040_lite_pico_tx_clear_status();
    pio_sm_set_enabled(tx_pio, tx_sm, true);
}
