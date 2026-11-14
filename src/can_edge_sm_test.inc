// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
//
// can_edge_sm_test.inc — Test-only edge-pair recording ring.

#if defined(CAN2040_LITE_TEST) && CAN_EDGE_PAIR_TIME_RECORDING
#if CAN_EDGE_PAIR_RECORD_CAPACITY < 1
#error "CAN_EDGE_PAIR_RECORD_CAPACITY must be at least 1"
#endif

static can_edge_pair_record_t pair_records[CAN_EDGE_PAIR_RECORD_CAPACITY];
static volatile uint32_t pair_record_count;
static volatile uint32_t pair_record_dropped;

void can_edge_pair_record_clear(void) {
    pair_record_count = 0;
    pair_record_dropped = 0;
}

uint32_t can_edge_pair_record_count(void) {
    return pair_record_count;
}

uint32_t can_edge_pair_record_dropped(void) {
    return pair_record_dropped;
}

const can_edge_pair_record_t *can_edge_pair_record_get(uint32_t index) {
    if (index >= pair_record_count)
        return NULL;
    return &pair_records[index];
}

static void pair_record(uint32_t word, uint32_t fall_time,
                        uint32_t rise_time) {
    uint32_t i = pair_record_count;

    if (i >= CAN_EDGE_PAIR_RECORD_CAPACITY) {
        pair_record_dropped++;
        return;
    }
    pair_records[i].word = word;
    pair_records[i].fall_time = fall_time & CAN_EDGE_TIME_MASK;
    pair_records[i].rise_time = rise_time & CAN_EDGE_TIME_MASK;
    __sync_synchronize();
    pair_record_count = i + 1u;
}

#endif // CAN2040_LITE_TEST && CAN_EDGE_PAIR_TIME_RECORDING
