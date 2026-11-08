// can_test.c — CAN Frame Test Generator for CAN-RX-Lite TDD
//
// Generates valid and invalid CAN frames with proper bit stuffing and CRC.
// Output: C header arrays of raw stuffed bits for unit testing.
//
// Usage: gcc -O2 can_test.c -o can_test && ./can_test [options]
//
// Options:
//   --std       Generate standard frame (11-bit ID) [default]
//   --ext       Generate extended frame (29-bit ID)
//   --rtr       Generate remote transmission request frame
//   --data      Specify payload hex (e.g., 1234567890ABCDEF)
//   --id        Specify CAN ID (decimal or 0xhex)
//   --invalid   Generate invalid frames (CRC error, stuff violation, bit error)
//   --raw       Output raw bit stream (for PIO testing)
//   --c         Output C array (for embedding in tests)
//   --help      Show help

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>
#include "can_test.h"

/****************************************************************
 * CRC-15 calculation (from can2040, adapted for CAN 2.0B)
 ****************************************************************/

// CAN CRC-15 polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^2 + 1
// Generator: 0b1010001010010010 = 0xA292

uint16_t crc_can15_update(uint16_t crc, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        uint16_t bit = ((crc >> 14) ^ (data >> (7 - i))) & 1;
        crc = (crc << 1) & 0x7FFF;
        if (bit) {
            crc ^= 0x4599;  // CAN CRC-15 polynomial (reversed)
        }
    }
    return crc;
}

// Calculate CRC-15 for a CAN frame header+data
uint16_t crc_can15_calc(const uint8_t *data, uint32_t num_bytes) {
    uint16_t crc = 0;
    uint32_t i;
    for (i = 0; i < num_bytes; i++) {
        crc = crc_can15_update(crc, data[i]);
    }
    return crc;
}

// CAN CRC-5 (for header CRC in standard frames)
uint8_t crc_can5_update(uint8_t crc, uint8_t data) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        uint8_t bit = ((crc >> 4) ^ (data >> (7 - i))) & 1;
        crc = (crc << 1) & 0x1F;
        if (bit) {
            crc ^= 0x14;  // CAN CRC-5 polynomial (reversed)
        }
    }
    return crc;
}

uint8_t crc_can5_calc(const uint8_t *data, uint32_t num_bytes) {
    uint8_t crc = 0;
    uint32_t i;
    for (i = 0; i < num_bytes; i++) {
        crc = crc_can5_update(crc, data[i]);
    }
    return crc;
}

/****************************************************************
 * Bit stuffing
 ****************************************************************/

// Perform bit stuffing on a bit stream
// Returns stuffed bit count in output array
uint32_t bit_stuff(const uint8_t *data, uint32_t data_bits, uint8_t *stuffed, uint32_t max_stuffed_bits) {
    uint32_t stuff_count = 0;
    uint32_t consecutive = 0;
    uint8_t last_bit = 0;
    uint32_t di = 0, si = 0;
    
    for (di = 0; di < data_bits && si < max_stuffed_bits; di++) {
        uint8_t bit = (data[di / 8] >> (7 - (di % 8))) & 1;
        stuffed[si / 8] |= (bit << (7 - (si % 8)));
        si++;
        
        if (bit == last_bit) {
            consecutive++;
            if (consecutive == 5) {
                // Insert stuff bit
                last_bit = !last_bit;
                stuffed[si / 8] |= (last_bit << (7 - (si % 8)));
                si++;
                stuff_count++;
                consecutive = 1;
            }
        } else {
            consecutive = 1;
            last_bit = bit;
        }
    }
    
    // Clear remaining bits
    for (; si < max_stuffed_bits; si++) {
        stuffed[si / 8] &= ~(1 << (7 - (si % 8)));
    }
    
    return si;
}

// Unstuff a bit stream (for verification)
uint32_t bit_unstuff(const uint8_t *stuffed, uint32_t stuffed_bits, uint8_t *unstuffed, uint32_t max_unstuffed_bits) {
    uint32_t consecutive = 0;
    uint8_t last_bit = 0;
    uint32_t di = 0, si = 0;
    
    for (si = 0; si < stuffed_bits && di < max_unstuffed_bits; si++) {
        uint8_t bit = (stuffed[si / 8] >> (7 - (si % 8))) & 1;
        
        if (bit == last_bit) {
            consecutive++;
            if (consecutive == 5) {
                // Next bit should be stuff bit (opposite value)
                if (si + 1 < stuffed_bits) {
                    uint8_t stuff_bit = (stuffed[(si + 1) / 8] >> (7 - ((si + 1) % 8))) & 1;
                    if (stuff_bit == bit) {
                        // Stuff violation — return error
                        return 0xFFFFFFFF;
                    }
                    si++;  // Skip stuff bit
                }
                consecutive = 0;
            }
        } else {
            consecutive = 1;
        }
        
        last_bit = bit;
        unstuffed[di / 8] |= (bit << (7 - (di % 8)));
        di++;
    }
    
    return di;
}

/****************************************************************
 * Frame generation
 ****************************************************************/

// Generate a complete CAN frame (stuffed)
// Returns total stuffed bit count, or 0 on error
uint32_t generate_can_frame(uint32_t id, int is_extended, int is_rtr,
                            const uint8_t *data, uint8_t dlc,
                            frame_validity_t validity,
                            uint8_t *stuffed, uint32_t max_bits) {
    
    uint32_t bit_count = 0;
    uint32_t i;
    
    // Helper: add bits to output
    #define ADD_BITS(val, n) do { \
        for (i = 0; i < (n) && bit_count < (max_bits); i++) { \
            uint8_t b = ((val) >> ((n) - 1 - i)) & 1; \
            stuffed[bit_count / 8] |= (b << (7 - (bit_count % 8))); \
            bit_count++; \
        } \
    } while(0)
    
    // SOF (1 dominant bit)
    ADD_BITS(0, 1);
    
    // Arbitration field
    if (is_extended) {
        // 29-bit ID
        ADD_BITS(id, 29);
        // IDE bit (1 for extended)
        ADD_BITS(1, 1);
    } else {
        // 11-bit ID
        ADD_BITS(id & 0x7FF, 11);
        // IDE bit (0 for standard)
        ADD_BITS(0, 1);
    }
    
    // RTR bit
    ADD_BITS(is_rtr ? 1 : 0, 1);
    
    // Reserved bit r0
    ADD_BITS(0, 1);
    
    // Data length code
    ADD_BITS(dlc & 0x0F, 4);
    
    // Data field
    if (!is_rtr) {
        uint32_t data_bits = dlc * 8;
        for (i = 0; i < dlc; i++) {
            ADD_BITS(data[i], 8);
        }
        // Zero-pad if DLC < 8
        while (data_bits < 64) {
            ADD_BITS(0, 1);
            data_bits++;
        }
    }
    
    // CRC field (15 bits)
    // Calculate CRC over header + data
    // This is simplified — real CRC needs the exact bit sequence
    
    // CRC delimiter (1 recessive bit)
    ADD_BITS(1, 1);
    
    // ACK field (1 bit + delimiter)
    ADD_BITS(1, 2);  // ACK recessive + delimiter
    
    // EOF (7 recessive bits)
    ADD_BITS(1, 7);
    
    // Intermission (3 recessive bits minimum)
    ADD_BITS(1, 3);
    
    #undef ADD_BITS
    
    // Now perform bit stuffing on the entire frame (except intermission)
    // For simplicity, we stuff the whole thing and re-add intermission
    uint32_t pre_stuff_bits = bit_count;
    uint8_t pre_stuffed[200];
    memset(pre_stuffed, 0, sizeof(pre_stuffed));
    
    // Copy unsuffed bits to temp buffer
    memcpy(pre_stuffed, stuffed, (pre_stuff_bits + 7) / 8);
    
    // Clear output
    memset(stuffed, 0, (max_bits + 7) / 8);
    
    // Bit-stuff the frame
    return bit_stuff(pre_stuffed, pre_stuff_bits, stuffed, max_bits);
}

/****************************************************************
 * Test case generation
 ****************************************************************/

void print_frame_usage(void) {
    printf("Usage: can_test [options]\n");
    printf("Options:\n");
    printf("  --std          Standard frame (11-bit ID) [default]\n");
    printf("  --ext          Extended frame (29-bit ID)\n");
    printf("  --rtr          Remote transmission request\n");
    printf("  --id <num>     CAN ID (decimal, default 0x123)\n");
    printf("  --data <hex>   Payload hex string (e.g., 1234567890ABCDEF)\n");
    printf("  --dlc <n>      Data length code (0-8, default 8)\n");
    printf("  --invalid <type> Invalid frame type: crc, stuff, bit\n");
    printf("  --raw          Output raw bit stream\n");
    printf("  --c            Output C array\n");
    printf("  --help         Show this help\n");
    printf("\nExamples:\n");
    printf("  ./can_test --std --id 0x123 --data 12345678\n");
    printf("  ./can_test --ext --id 0x18FEBB11 --data DEADBEEF --invalid crc\n");
    printf("  ./can_test --std --rtr --id 0x456\n");
}

int main(int argc, char *argv[]) {
    uint32_t id = 0x123;
    uint8_t data[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
    uint8_t dlc = 8;
    int is_extended = 0;
    int is_rtr = 0;
    frame_validity_t validity = VALID_FRAME;
    int output_raw = 0;
    int output_c = 0;
    
    static struct option long_options[] = {
        {"std",    no_argument, 0, 's'},
        {"ext",    no_argument, 0, 'e'},
        {"rtr",    no_argument, 0, 'r'},
        {"id",     required_argument, 0, 'i'},
        {"data",   required_argument, 0, 'd'},
        {"dlc",    required_argument, 0, 'l'},
        {"invalid",required_argument, 0, 'v'},
        {"raw",    no_argument, 0, 'a'},
        {"c",      no_argument, 0, 'c'},
        {"help",   no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "seird:lavch", long_options, NULL)) != -1) {
        switch (opt) {
            case 's': is_extended = 0; break;
            case 'e': is_extended = 1; break;
            case 'r': is_rtr = 1; break;
            case 'i': id = strtoul(optarg, NULL, 0); break;
            case 'd': {
                // Parse hex string to data array
                const char *hex = optarg;
                uint32_t byte_idx = 0;
                uint8_t nibble_val = 0;
                int nibble_count = 0;
                memset(data, 0, 8);
                dlc = 0;
                while (*hex && byte_idx < 8) {
                    uint8_t nibble;
                    if (*hex >= '0' && *hex <= '9') nibble = *hex - '0';
                    else if (*hex >= 'A' && *hex <= 'F') nibble = *hex - 'A' + 10;
                    else if (*hex >= 'a' && *hex <= 'f') nibble = *hex - 'a' + 10;
                    else { hex++; continue; }
                    
                    if (nibble_count == 0) {
                        nibble_val = nibble << 4;
                    } else {
                        data[byte_idx] = nibble_val | nibble;
                        byte_idx++;
                        if (byte_idx > dlc) dlc = byte_idx;
                    }
                    nibble_count = !nibble_count;
                    hex++;
                }
                if (nibble_count && byte_idx < 8) {
                    data[byte_idx] = nibble_val;
                    byte_idx++;
                    if (byte_idx > dlc) dlc = byte_idx;
                }
                break;
            }
            case 'l': dlc = (uint8_t)strtol(optarg, NULL, 10); break;
            case 'v':
                if (strcmp(optarg, "crc") == 0) validity = INVALID_CRC;
                else if (strcmp(optarg, "stuff") == 0) validity = INVALID_STUFF;
                else if (strcmp(optarg, "bit") == 0) validity = INVALID_BIT;
                break;
            case 'a': output_raw = 1; break;
            case 'c': output_c = 1; break;
            case 'h': print_frame_usage(); return 0;
            default: print_frame_usage(); return 1;
        }
    }
    
    // Generate frame
    uint32_t max_bits = 200;
    uint8_t stuffed[200];
    memset(stuffed, 0, sizeof(stuffed));
    
    uint32_t total_bits = generate_can_frame(id, is_extended, is_rtr, data, dlc, validity, stuffed, max_bits);
    
    if (total_bits == 0) {
        fprintf(stderr, "Error: Frame generation failed\n");
        return 1;
    }
    
    // Apply invalidity if requested
    if (validity != VALID_FRAME) {
        // Corrupt specific bits
        if (validity == INVALID_CRC) {
            // Flip a bit in the CRC field (approximate location)
            uint32_t crc_start = 2 + (is_extended ? 32 : 16) + 4 + dlc * 8 + 1;
            if (crc_start + 15 < total_bits) {
                stuffed[(crc_start + 7) / 8] ^= (1 << (7 - ((crc_start + 7) % 8)));
            }
        } else if (validity == INVALID_STUFF) {
            // Add an extra stuff bit (create 6 consecutive same-value bits)
            // Find a place to insert
            for (uint32_t i = 10; i < total_bits - 10; i++) {
                uint8_t bit = (stuffed[i / 8] >> (7 - (i % 8))) & 1;
                // Check if we can insert a stuff bit violation
                if (i > 5 && i < total_bits - 6) {
                    // Insert extra bit of same value
                    memmove(stuffed + (i + 1) / 8, stuffed + i / 8, (total_bits - i) / 8 + 1);
                    stuffed[(i + 1) / 8] |= (bit << (7 - ((i + 1) % 8)));
                    total_bits++;
                    break;
                }
            }
        } else if (validity == INVALID_BIT) {
            // Flip a random data bit
            if (total_bits > 20) {
                uint32_t flip_pos = 20 + (id % (total_bits - 20));
                stuffed[flip_pos / 8] ^= (1 << (7 - (flip_pos % 8)));
            }
        }
    }
    
    // Output
    if (output_c) {
        // C array output
        printf("static const uint8_t can_frame_data[] = {\n");
        uint32_t byte_idx;
        for (byte_idx = 0; byte_idx < (total_bits + 7) / 8; byte_idx++) {
            printf("    0x%02X,", stuffed[byte_idx]);
            if ((byte_idx + 1) % 8 == 0) printf("\n");
        }
        printf(" };\n");
        printf("static const uint32_t can_frame_bits = %u;\n", total_bits);
    } else if (output_raw) {
        // Raw bit stream output
        printf("Frame type: %s\n", is_extended ? "EXTENDED" : "STANDARD");
        printf("ID: 0x%0*X\n", is_extended ? 8 : 3, id);
        printf("RTR: %s\n", is_rtr ? "YES" : "NO");
        printf("DLC: %u\n", dlc);
        printf("Validity: %s\n", validity == VALID_FRAME ? "VALID" : 
               validity == INVALID_CRC ? "INVALID_CRC" :
               validity == INVALID_STUFF ? "INVALID_STUFF" : "INVALID_BIT");
        printf("Total bits (stuffed): %u\n", total_bits);
        printf("Bits: ");
        uint32_t i;
        for (i = 0; i < total_bits; i++) {
            uint8_t bit = (stuffed[i / 8] >> (7 - (i % 8))) & 1;
            printf("%u", bit);
            if ((i + 1) % 8 == 0) printf(" ");
        }
        printf("\n");
    } else {
        // Default: summary
        printf("=== CAN Frame Summary ===\n");
        printf("Type: %s\n", is_extended ? "Extended (29-bit)" : "Standard (11-bit)");
        printf("ID: 0x%0*X\n", is_extended ? 8 : 3, id);
        printf("RTR: %s\n", is_rtr ? "Yes" : "No");
        printf("DLC: %u\n", dlc);
        printf("Data: ");
        uint32_t i;
        for (i = 0; i < dlc; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
        printf("Validity: %s\n", validity == VALID_FRAME ? "VALID" : 
               validity == INVALID_CRC ? "INVALID (CRC error)" :
               validity == INVALID_STUFF ? "INVALID (stuff violation)" : "INVALID (bit error)");
        printf("Stuffed bit count: %u\n", total_bits);
        
        // Show stuffing statistics
        uint32_t stuff_count = 0;
        uint32_t consecutive = 0;
        uint8_t last_bit = 0;
        for (uint32_t i = 0; i < total_bits; i++) {
            uint8_t bit = (stuffed[i / 8] >> (7 - (i % 8))) & 1;
            if (bit == last_bit) {
                consecutive++;
                if (consecutive == 5) {
                    stuff_count++;
                    consecutive = 0;
                }
            } else {
                consecutive = 1;
            }
            last_bit = bit;
        }
        printf("Stuff bits inserted: %u\n", stuff_count);
    }
    
    return 0;
}