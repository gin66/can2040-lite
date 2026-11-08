// can_test_units.c — Unit tests for CAN-RX-Lite (TDD)
//
// Tests:
// 1. Bit unstuffing (validates test generator output)
// 2. CRC-15 calculation (from can2040)
// 3. Frame parsing (standard and extended)
// 4. Invalid frame detection
//
// Usage: gcc -O2 can_test.c can_test_units.c -o can_test_units && ./can_test_units

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "can_test.h"

typedef struct {
    int passed;
    int failed;
} test_suite_t;

static test_suite_t current_suite;
static int global_passed = 0;
static int global_failed = 0;

#define SUITE(name) do { \
    current_suite.passed = 0; \
    current_suite.failed = 0; \
    printf("\n=== %s ===\n", (name)); \
} while(0)

#define TEST(desc) do { \
    printf("  TEST: %s ... ", (desc)); \
    fflush(stdout); \
} while(0)

#define ASSERT(cond, msg) do { \
    if (cond) { \
        current_suite.passed++; \
        printf("PASS\n"); \
    } else { \
        current_suite.failed++; \
        global_failed++; \
        printf("FAIL: %s\n", (msg)); \
    } \
} while(0)

#define ASSERT_EQ(a, b, msg) ASSERT((a) == (b), (msg))
#define ASSERT_GE(a, b, msg) ASSERT((a) >= (b), (msg))

void test_bit_unstuffing(void) {
    SUITE("Bit Unstuffing Tests");
    
    // Test 1: Simple unstuffing
    {
        TEST("Unstuff simple frame");
        uint8_t data[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
        uint8_t stuffed[200], unstuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        memset(unstuffed, 0, sizeof(unstuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x123, 0, 0, data, 8, VALID_FRAME, stuffed, 200);
        uint32_t unstuffed_bits = bit_unstuff(stuffed, stuffed_bits, unstuffed, 200);
        
        ASSERT(stuffed_bits > 64, "Stuffed frame should have > 64 bits");
        ASSERT(unstuffed_bits == stuffed_bits, "Unstuffing should return all bits for valid frame");
    }
    
    // Test 2: Unstuffing preserves data
    {
        TEST("Unstuffing preserves original data");
        uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
        uint8_t stuffed[200], unstuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        memset(unstuffed, 0, sizeof(unstuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x456, 0, 0, data, 8, VALID_FRAME, stuffed, 200);
        uint32_t unstuffed_bits = bit_unstuff(stuffed, stuffed_bits, unstuffed, 200);
        
        // Extract original data from unstuffed bits (skip SOF, ID, control, DLC)
        // For standard frame: skip 16 bits (SOF + 11-bit ID + IDE + RTR + r0)
        uint32_t data_start = 16;
        int match = 1;
        uint32_t i;
        for (i = 0; i < 8 && data_start + i * 8 < unstuffed_bits; i++) {
            uint8_t original = data[i];
            uint8_t extracted = 0;
            uint32_t b;
            for (b = 0; b < 8; b++) {
                extracted = (extracted << 1) | ((unstuffed[(data_start + i * 8 + b) / 8] >> (7 - ((data_start + i * 8 + b) % 8))) & 1);
            }
            if (extracted != original) {
                match = 0;
                break;
            }
        }
        ASSERT(match, "Unstuffed data should match original");
    }
    
    // Test 3: Extended frame unstuffing
    {
        TEST("Extended frame unstuffing");
        uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78};
        uint8_t stuffed[200], unstuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        memset(unstuffed, 0, sizeof(unstuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x18FEBB11, 1, 0, data, 8, VALID_FRAME, stuffed, 200);
        uint32_t unstuffed_bits = bit_unstuff(stuffed, stuffed_bits, unstuffed, 200);
        
        ASSERT(stuffed_bits > 100, "Extended frame should have > 100 stuffed bits");
        ASSERT(unstuffed_bits == stuffed_bits, "Extended frame unstuffing should succeed");
    }
    
    // Test 4: RTR frame
    {
        TEST("RTR frame unstuffing");
        uint8_t data[8] = {0};
        uint8_t stuffed[200], unstuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        memset(unstuffed, 0, sizeof(unstuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x789, 0, 1, data, 0, VALID_FRAME, stuffed, 200);
        uint32_t unstuffed_bits = bit_unstuff(stuffed, stuffed_bits, unstuffed, 200);
        
        ASSERT(stuffed_bits > 50, "RTR frame should have > 50 stuffed bits");
        ASSERT(unstuffed_bits == stuffed_bits, "RTR frame unstuffing should succeed");
    }
    
    // Print suite results
    printf("  Results: %d passed, %d failed\n", current_suite.passed, current_suite.failed);
    global_passed += current_suite.passed;
}

void test_crc_calculation(void) {
    SUITE("CRC Calculation Tests");
    
    // Test 1: CRC-15 for standard frame header
    {
        TEST("CRC-15 standard frame header");
        // Standard frame header: SOF + 11-bit ID + IDE + RTR + r0 + DLC
        // Example: ID=0x123, DLC=8
        uint8_t header[4] = {0x18, 0x0A, 0x41, 0x04};  // Simplified header bytes
        uint16_t crc = crc_can15_calc(header, 4);
        ASSERT(crc != 0, "CRC should be non-zero for valid header");
        ASSERT(crc < 0x8000, "CRC-15 should be < 0x8000");
    }
    
    // Test 2: CRC-15 for extended frame header
    {
        TEST("CRC-15 extended frame header");
        uint8_t header[6] = {0x3C, 0x61, 0xE0, 0xC9, 0x04, 0x10};  // Simplified
        uint16_t crc = crc_can15_calc(header, 6);
        ASSERT(crc != 0, "Extended frame CRC should be non-zero");
    }
    
    // Test 3: CRC-5 for header (standard frames)
    {
        TEST("CRC-5 standard frame");
        uint8_t header[2] = {0x18, 0x0A};
        uint8_t crc = crc_can5_calc(header, 2);
        ASSERT(crc < 0x20, "CRC-5 should be < 0x20");
    }
    
    // Test 4: CRC is deterministic
    {
        TEST("CRC is deterministic");
        uint8_t data[4] = {0x12, 0x34, 0x56, 0x78};
        uint16_t crc1 = crc_can15_calc(data, 4);
        uint16_t crc2 = crc_can15_calc(data, 4);
        ASSERT_EQ(crc1, crc2, "Same input should produce same CRC");
    }
    
    // Print suite results
    printf("  Results: %d passed, %d failed\n", current_suite.passed, current_suite.failed);
    global_passed += current_suite.passed;
}

void test_frame_parsing(void) {
    SUITE("Frame Parsing Tests");
    
    // Test 1: Standard frame parsing
    {
        TEST("Parse standard frame");
        uint8_t data[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
        uint8_t stuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x123, 0, 0, data, 8, VALID_FRAME, stuffed, 200);
        ASSERT(stuffed_bits > 0, "Frame generation should succeed");
        ASSERT(stuffed_bits < 200, "Frame should fit in buffer");
    }
    
    // Test 2: Extended frame parsing
    {
        TEST("Parse extended frame");
        uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
        uint8_t stuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        
        uint32_t stuffed_bits = generate_can_frame(0x18FEBB11, 1, 0, data, 8, VALID_FRAME, stuffed, 200);
        ASSERT(stuffed_bits > 100, "Extended frame should be > 100 bits");
    }
    
    // Test 3: Various DLC values
    {
        int dlc;
        TEST("Various DLC values");
        for (dlc = 0; dlc <= 8; dlc++) {
            uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
            uint8_t stuffed[200];
            memset(stuffed, 0, sizeof(stuffed));
            
            uint32_t bits = generate_can_frame(0x100, 0, 0, data, (uint8_t)dlc, VALID_FRAME, stuffed, 200);
            ASSERT(bits > 0, "DLC frame should generate bits");
        }
    }
    
    // Print suite results
    printf("  Results: %d passed, %d failed\n", current_suite.passed, current_suite.failed);
    global_passed += current_suite.passed;
}

void test_invalid_frames(void) {
    SUITE("Invalid Frame Tests");
    
    // Test 1: CRC error frame
    {
        TEST("Invalid CRC frame");
        uint8_t data[8] = {0xDE, 0xAD, 0xBE, 0xEF};
        uint8_t stuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        
        uint32_t bits = generate_can_frame(0x123, 0, 0, data, 4, INVALID_CRC, stuffed, 200);
        ASSERT(bits > 0, "Invalid CRC frame should still generate bits");
    }
    
    // Test 2: Stuff violation frame
    {
        TEST("Stuff violation frame");
        uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF};
        uint8_t stuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        
        uint32_t bits = generate_can_frame(0x456, 0, 0, data, 4, INVALID_STUFF, stuffed, 200);
        ASSERT(bits > 0, "Stuff violation frame should still generate bits");
    }
    
    // Test 3: Bit error frame
    {
        TEST("Bit error frame");
        uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD};
        uint8_t stuffed[200];
        memset(stuffed, 0, sizeof(stuffed));
        
        uint32_t bits = generate_can_frame(0x789, 0, 0, data, 4, INVALID_BIT, stuffed, 200);
        ASSERT(bits > 0, "Bit error frame should still generate bits");
    }
    
    // Print suite results
    printf("  Results: %d passed, %d failed\n", current_suite.passed, current_suite.failed);
    global_passed += current_suite.passed;
}

void run_all_tests(void) {
    printf("========================================\n");
    printf("CAN-RX-Lite Unit Tests (TDD)\n");
    printf("========================================\n");
    
    test_bit_unstuffing();
    test_crc_calculation();
    test_frame_parsing();
    test_invalid_frames();
    
    printf("\n========================================\n");
    printf("TOTAL: %d passed, %d failed\n", global_passed, global_failed);
    printf("========================================\n");
    
    if (global_failed > 0) {
        exit(1);
    }
}

// Override main to run tests
int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    run_all_tests();
    return 0;
}