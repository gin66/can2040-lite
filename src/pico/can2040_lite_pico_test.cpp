// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
//
// can2040_lite_pico_test.cpp — Test-only blocking RX wait.

#ifdef CAN2040_LITE_TEST

#include <Arduino.h>

#include "can2040_lite_pico.h"
#include "can2040_lite_pico_internal.h"

extern "C" int can2040_lite_pico_rx_wait(uint32_t timeout_ms) {
    uint32_t start = millis();

    while (!can2040_lite_pico_rx_pending()) {
        can2040_lite_pico_rx_service();
        if (can2040_lite_pico_rx_pending())
            break;
        if (millis() - start > timeout_ms)
            return 0;
        delay(1);
    }
    return 1;
}

#endif // CAN2040_LITE_TEST
