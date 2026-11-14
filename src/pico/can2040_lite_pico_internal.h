// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
//
// can2040_lite_pico_internal.h — Private Pico backend hooks.

#ifndef CAN2040_LITE_PICO_INTERNAL_H
#define CAN2040_LITE_PICO_INTERNAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CAN2040_LITE_TEST
void can2040_lite_pico_rx_service(void);
int can2040_lite_pico_rx_pending(void);
#endif

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_PICO_INTERNAL_H
