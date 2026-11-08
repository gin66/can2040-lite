// can_test.h — Header for CAN frame test generator and unit tests

#ifndef CAN_TEST_H
#define CAN_TEST_H

#include <stdint.h>

// Frame types
typedef enum {
    FRAME_STANDARD,
    FRAME_EXTENDED,
    FRAME_RTR_STANDARD,
    FRAME_RTR_EXTENDED
} frame_type_t;

// Frame validity
typedef enum {
    VALID_FRAME,
    INVALID_CRC,
    INVALID_STUFF,
    INVALID_BIT
} frame_validity_t;

// CRC functions
uint16_t crc_can15_update(uint16_t crc, uint8_t data);
uint16_t crc_can15_calc(const uint8_t *data, uint32_t num_bytes);
uint8_t crc_can5_update(uint8_t crc, uint8_t data);
uint8_t crc_can5_calc(const uint8_t *data, uint32_t num_bytes);

// Bit stuffing/unstuffing
uint32_t bit_stuff(const uint8_t *data, uint32_t data_bits, uint8_t *stuffed, uint32_t max_stuffed_bits);
uint32_t bit_unstuff(const uint8_t *stuffed, uint32_t stuffed_bits, uint8_t *unstuffed, uint32_t max_unstuffed_bits);

// Frame generation
uint32_t generate_can_frame(uint32_t id, int is_extended, int is_rtr,
                            const uint8_t *data, uint8_t dlc,
                            frame_validity_t validity,
                            uint8_t *stuffed, uint32_t max_bits);

#endif // CAN_TEST_H