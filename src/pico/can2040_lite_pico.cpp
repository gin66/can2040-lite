// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040_lite_pico.cpp — RP2040/RP2350 PIO backend.
//
// Two CAN SMs on PIO0 (SM0 rx, SM1 tx).  The rx + tx programs are 7 + 4 = 11
// instructions, well inside the 29-word budget that leaves 2 words for the
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

// Measured on RP2350: a taken PIO branch costs 1 cycle here, so
//   TX word = pull(1) + set(1) + 32*out(31) + 31*jmp(1) + jmp(1) = 1026
//   RX block = set(1) + 32*in(1) + 31*jmp(1) + jmp(1) + push(1) = 66
// Re-check these constants if the PIO branch timing differs on another part.
static const float tx_clk_mult = 32.0625f;
static const float rx_clk_mult = 66.0f;

static_assert(can_rx_program.length + can_tx_program.length <= 29,
              "rx + tx PIO programs must fit in 29 words");

static PIO rx_pio;
static PIO tx_pio;
static uint rx_sm;
static uint tx_sm;
static uint rx_offset;
static uint tx_offset;
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

extern "C" int can2040_lite_pico_tx_init(uint32_t gpio, uint32_t bitrate) {
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

    tx_cfg = can_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&tx_cfg, gpio, 1);
    sm_config_set_out_shift(&tx_cfg, false, false, 32); // shift left, no autopull
    sm_config_set_clkdiv(&tx_cfg, clkdiv_for(bitrate, tx_clk_mult));

    pio_gpio_init(tx_pio, gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
    pio_sm_set_consecutive_pindirs(tx_pio, tx_sm, gpio, 1, true);

    pio_sm_init(tx_pio, tx_sm, tx_offset, &tx_cfg);
    pio_sm_set_pins_with_mask(tx_pio, tx_sm, 1u << gpio, 1u << gpio);
    pio_sm_set_enabled(tx_pio, tx_sm, true);
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

extern "C" void can2040_lite_pico_tx_frame(const uint8_t *bits,
                                           uint32_t nbits) {
    uint32_t words = (nbits + 31u) / 32u;
    for (uint32_t w = 0; w < words; w++) {
        uint32_t word = 0;
        for (uint32_t i = 0; i < 32u; i++) {
            uint32_t pos = w * 32u + i;
            uint8_t bit = (pos < nbits) ? bits_get(bits, pos) : 1;
            word = (word << 1) | bit;
        }
        uint32_t spin;
        for (spin = 0; spin < 1000000u; spin++) {
            if (!pio_sm_is_tx_fifo_full(tx_pio, tx_sm)) {
                pio_sm_put(tx_pio, tx_sm, word);
                break;
            }
        }
        if (spin == 1000000u) {
            Serial.println("tx frame: FIFO stuck");
            return;
        }
    }
}
