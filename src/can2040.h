// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can2040.h — Source-compatible can2040 spellings for CAN2040-Lite.
//
// An application written against can2040 includes this header and recompiles
// against the CAN2040-Lite library.  This is source compatibility, not binary
// compatibility: struct can2040 (here can2040_lite) has a different layout.

#ifndef CAN2040_H
#define CAN2040_H

#include "can2040_lite.h"

#define can2040             can2040_lite
#define can2040_msg         can2040_lite_msg
#define can2040_stats       can2040_lite_stats
#define can2040_rx_cb       can2040_lite_cb
#define can2040_setup       can2040_lite_setup
#define can2040_callback_config can2040_lite_callback_config
#define can2040_start       can2040_lite_start
#define can2040_stop        can2040_lite_stop
#define can2040_get_statistics can2040_lite_get_statistics
#define can2040_pio_irq_handler can2040_lite_pio_irq_handler
#define can2040_check_transmit can2040_lite_check_transmit
#define can2040_transmit    can2040_lite_transmit
#define CAN2040_ID_RTR      CAN2040_LITE_ID_RTR
#define CAN2040_ID_EFF      CAN2040_LITE_ID_EFF
#define CAN2040_NOTIFY_RX   CAN2040_LITE_NOTIFY_RX
#define CAN2040_NOTIFY_TX   CAN2040_LITE_NOTIFY_TX
#define CAN2040_NOTIFY_ERROR CAN2040_LITE_NOTIFY_ERROR

#endif // CAN2040_H
