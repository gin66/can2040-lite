// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
//
// can2040_lite_pico_test.inc — Stateful Pico test hooks.
//
// Included by can2040_lite_pico.cpp when CAN2040_LITE_TEST is defined so it
// can reach the backend's private state.

#ifdef CAN2040_LITE_TEST

// Non-blocking main-loop service: drain the FIFO and advance the parser ring.
extern "C" void can2040_lite_pico_rx_service(void) {
    uint32_t save;

    if (!rx_initialized)
        return;
    save = save_and_disable_interrupts();
    can2040_lite_pico_rx_irq();
    pico_service_events();
    restore_interrupts(save);
}

// Non-zero when a parsed frame is waiting in the parser ring.
extern "C" int can2040_lite_pico_rx_pending(void) {
    return rx_initialized && can_rx_available(&rx);
}

extern "C" const can_parsed_t *can2040_lite_pico_rx_get_parsed(void) {
    if (!can_rx_available(&rx))
        return NULL;
    __sync_synchronize();
    rx_convert(can_rx_front(&rx), &latest_parsed);
    can_rx_pop(&rx);
    latest_parsed_valid = true;
    return &latest_parsed;
}

extern "C" void can2040_lite_pico_rx_record_clear(void) {
#if CAN_EDGE_PAIR_TIME_RECORDING
    rx_raw_record_head = 0;
#endif
}

extern "C" uint32_t can2040_lite_pico_rx_record_count(void) {
#if CAN_EDGE_PAIR_TIME_RECORDING
    return rx_raw_record_head;
#else
    return 0;
#endif
}

extern "C" uint32_t can2040_lite_pico_rx_record_word(uint32_t index) {
#if CAN_EDGE_PAIR_TIME_RECORDING
    uint32_t head = rx_raw_record_head;
    uint32_t oldest = head > CAN_PICO_RAW_RECORD_WORDS
                          ? head - CAN_PICO_RAW_RECORD_WORDS
                          : 0u;

    if (index < oldest || index >= head)
        return 0;
    return rx_raw_record[index & CAN_PICO_RAW_RECORD_MASK];
#else
    (void)index;
    return 0;
#endif
}

extern "C" void can2040_lite_pico_tx_set_readback_gpio(uint32_t gpio) {
    tx_rx_gpio = gpio;
    pio_gpio_init(tx_pio, gpio);
    pio_sm_set_jmp_pin(tx_pio, tx_sm, gpio);
    sm_config_set_jmp_pin(&tx_cfg, gpio);
#if CAN_ACK_GENERATION_SUPPORT
    sm_config_set_jmp_pin(&ack_cfg, gpio);
#endif
}


#endif // CAN2040_LITE_TEST
