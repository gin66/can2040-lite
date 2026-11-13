/* SPDX-License-Identifier: GPL-3.0-only
 *
 * Recorded RP2040 RX FIFO words from the Fly-Micro4 at 1 Mbit/s.
 *
 * Source: ESP32-S3 TWAI in no-ack mode sending id 0x123, DLC 8, data byte 0
 * incrementing once per frame. The RP2040 was receive-only (no TX, no ACK).
 * Each word is (fall_timestamp16 << 16) | rise_timestamp16, exactly as the
 * pico backend drains it from the RX FIFO. The slice starts at the SOF of a
 * complete frame, contains three complete frames, and ends with the next SOF
 * so the third frame completes without a synthetic quiet-tail flush.
 *
 * Expected, in order:
 *   id 0x123 dlc 8 data 1e1f202122232425 crc 7B32
 *   id 0x123 dlc 8 data 1f20212223242526 crc 72B8
 *   id 0x123 dlc 8 data 2021222324252627 crc 30DA
 */
#ifndef CAN_RX_RECORD_FLY_MICRO4_H
#define CAN_RX_RECORD_FLY_MICRO4_H

#include <stdint.h>

#define CAN_RX_RECORD_FLY_MICRO4_TICKS_PER_BIT 66u

static const uint32_t can_rx_record_fly_micro4_1m[] = {
    0x4A8A49C4u, 0x49844900u, 0x48C147FAu, 0x477946B2u, 0x46734528u, 0x44E944A6u,
    0x43A14298u, 0x4151408Bu, 0x404B3F01u, 0x3EC13E3Du, 0x3DFD3CF5u, 0x3CB53C31u,
    0x3BF13B2Bu, 0x3AEB3A25u, 0x39E5391Fu, 0x389D3819u, 0x37DA3755u, 0x3716360Du,
    0x35CE3549u, 0x350A34C7u, 0x338032FBu, 0x327A31F5u, 0x317430EFu, 0x30B0306Du,
    0x48F84832u, 0x47F2476Eu, 0x472F4668u, 0x45E74520u, 0x44E14396u, 0x43574314u,
    0x41CD4106u, 0x40C73F7Du, 0x3F3D3EB9u, 0x3E793D71u, 0x3D313CADu, 0x3C6D3BA7u,
    0x3B673AA1u, 0x3A61399Bu, 0x39193895u, 0x385537D1u, 0x37913689u, 0x364A35C5u,
    0x35863543u, 0x3504347Fu, 0x344033BBu, 0x333A32F7u, 0x323431AFu, 0x3170312Du,
    0x30EE30ABu, 0x2FE82F21u, 0x476A46A4u, 0x466445E0u, 0x45A144DAu, 0x44594392u,
    0x43534208u, 0x4187403Cu, 0x3FFD3F78u, 0x3F393E31u, 0x3DF13D6Du, 0x3D2D3C67u,
    0x3C273B61u, 0x3B213A5Bu, 0x39D93955u, 0x39153891u, 0x38513749u, 0x37093685u,
    0x36453603u, 0x35C4353Fu, 0x3500347Bu, 0x33FA3333u, 0x32F4326Fu, 0x31AC3169u,
    0x30E82FDFu, 0x2F5E2F1Bu, 0x2E9A2E57u, 0x2E182DD5u, 0x45DD4517u,
};

#endif /* CAN_RX_RECORD_FLY_MICRO4_H */
