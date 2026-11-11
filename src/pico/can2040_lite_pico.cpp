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

#include <Arduino.h>

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>

#include "can2040_lite_pico.h"
#include "can2040_lite_proto.h"
#include "rx.pio.h"
#include "tx.pio.h"

// TX is exactly 32 PIO cycles per bit; autopull refills the OSR for free so
// there is no word-boundary jitter.  RX captures 32 samples per bit and its
// block is 66 cycles, so both clocks are integer multiples of the bitrate.
// Re-check the RX constant if the PIO branch timing differs on another part.
static const float tx_clk_mult = 32.0f;
static const float rx_clk_mult = 66.0f;

// TX is one-shot per CAN message.  The program raises one of these IRQs and
// parks; software must re-prepare the SM for the next message.
static const uint CAN_TX_DONE_IRQ = 6;
static const uint CAN_TX_ERR_IRQ = 7;

// Joined TX FIFO depth.  One entry is the bit count, and one recessive word
// is reserved so the OUT that empties the OSR can autopull instead of stalling.
// A worst-case classic message is 6 words, so count + message + pad fit.
static const uint CAN_TX_FIFO_WORDS = 8;

static_assert(can_rx_program.length + can_tx_program.length <= 29,
              "rx + tx PIO programs must fit in 29 words");

static PIO rx_pio;
static PIO tx_pio;
static uint rx_sm;
static uint tx_sm;
static uint rx_offset;
static uint tx_offset;
static uint tx_gpio;
static pio_sm_config rx_cfg;
static pio_sm_config tx_cfg;
static int rx_dma_chan = -1;

static uint32_t rx_words[CAN2040_LITE_PICO_RX_WORDS];

static float clkdiv_for(uint32_t bitrate, float mult) {
    return (float)clock_get_hz(clk_sys) / (mult * (float)bitrate);
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
    off = pio_add_program(rx_pio, &can_rx_program);
    if (off < 0) {
        Serial.println("rx init: no program space");
        return -1;
    }
    rx_offset = (uint)off;

    rx_cfg = can_rx_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&rx_cfg, gpio);
    sm_config_set_jmp_pin(&rx_cfg, gpio);
    sm_config_set_in_shift(&rx_cfg, false, false, 32); // shift left, no autopush
    sm_config_set_clkdiv(&rx_cfg, clkdiv_for(bitrate, rx_clk_mult));

    pio_gpio_init(rx_pio, gpio);
    // Leave the pin direction alone: with loopback it is already an output.

    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);
    pio_sm_set_enabled(rx_pio, rx_sm, false);
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

extern "C" void can2040_lite_pico_rx_arm(void) {
    pio_sm_set_enabled(rx_pio, rx_sm, false);
    pio_sm_clear_fifos(rx_pio, rx_sm);
    pio_sm_restart(rx_pio, rx_sm);
    pio_sm_init(rx_pio, rx_sm, rx_offset, &rx_cfg);

    if (rx_dma_chan < 0)
        rx_dma_chan = (int)dma_claim_unused_channel(true);

    dma_channel_config dc = dma_channel_get_default_config((uint)rx_dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(rx_pio, rx_sm, false));
    dma_channel_configure((uint)rx_dma_chan, &dc, rx_words,
                          &rx_pio->rxf[rx_sm],
                          CAN2040_LITE_PICO_RX_WORDS, true);

    pio_sm_set_enabled(rx_pio, rx_sm, true);
}

extern "C" int can2040_lite_pico_rx_wait(uint32_t timeout_ms) {
    uint32_t start = millis();
    while (dma_channel_is_busy((uint)rx_dma_chan)) {
        if (millis() - start > timeout_ms) {
            dma_channel_abort((uint)rx_dma_chan);
            pio_sm_set_enabled(rx_pio, rx_sm, false);
            return 0;
        }
        delay(1);
    }
    pio_sm_set_enabled(rx_pio, rx_sm, false);
    return 1;
}

extern "C" uint32_t can2040_lite_pico_rx_bits(uint8_t *bits,
                                              uint32_t max_bits) {
    uint32_t n = 0;

    // Word 0 is the poll marker; word 1 is the SOF bit.
    for (uint32_t w = 1; w < CAN2040_LITE_PICO_RX_WORDS && n < max_bits; w++) {
        uint32_t ones = (uint32_t)__builtin_popcount(rx_words[w]);
        bits_put(bits, n++, (ones > 16u) ? 1 : 0);
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
