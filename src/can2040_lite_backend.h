// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
//
// can2040_lite_backend.h — Fixed RP2040/RP2350 backend interface.
//
// The driver core calls these directly.  The Pico backend supplies them in
// production; the host test binary supplies a fake through its test module.
// There is no runtime ops-table indirection.

#ifndef CAN2040_LITE_BACKEND_H
#define CAN2040_LITE_BACKEND_H

#include "can2040_lite.h"

#ifdef __cplusplus
extern "C" {
#endif

int can2040_lite_backend_start(struct can2040_lite *cd, uint32_t sys_clock,
                               uint32_t bitrate, int32_t gpio_rx,
                               int32_t gpio_tx);
void can2040_lite_backend_stop(void);
int can2040_lite_backend_tx_frame(const uint8_t *bits, uint32_t nbits);
int can2040_lite_backend_poll(struct can2040_lite_event *ev);

uint32_t can2040_lite_critical_enter(void);
void can2040_lite_critical_exit(uint32_t token);

#ifdef __cplusplus
}
#endif

#endif // CAN2040_LITE_BACKEND_H
