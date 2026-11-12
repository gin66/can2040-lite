// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// can_gen.c — Generate valid/invalid CAN 2.0B frames for PIO, Saleae and
// on-target vector tests.
//
// Build: make -C extras/tools/can_gen

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>

#include "can_frame_build.h"

static const char *validity_name(frame_validity_t v) {
    switch (v) {
    case VALID_FRAME:   return "VALID";
    case INVALID_CRC:   return "INVALID_CRC";
    case INVALID_STUFF: return "INVALID_STUFF";
    case INVALID_BIT:   return "INVALID_BIT";
    }
    return "?";
}

static void print_usage(void) {
    printf("Usage: can_gen [options]\n");
    printf("Options:\n");
    printf("  --std          Standard frame (11-bit ID) [default]\n");
    printf("  --ext          Extended frame (29-bit ID)\n");
    printf("  --rtr          Generate an RTR vector (the node does not answer an RTR)\n");
    printf("  --id <num>     CAN ID (decimal or 0xhex, default 0x123)\n");
    printf("  --data <hex>   Payload hex string (e.g., 1234567890ABCDEF)\n");
    printf("  --dlc <n>      Data length code (0-8, default from payload)\n");
    printf("  --invalid <t>  Invalid frame type: crc, stuff, bit\n");
    printf("  --ack          Make the ACK slot dominant (default: NACK)\n");
    printf("  --raw          Output the raw bit stream\n");
    printf("  --c            Output a C array\n");
    printf("  --help         Show this help\n");
    printf("\nExamples:\n");
    printf("  can_gen --std --id 0x123 --data 12345678\n");
    printf("  can_gen --ext --id 0x18FEBB11 --data DEADBEEF --invalid crc\n");
    printf("  can_gen --std --rtr --id 0x456  # negative/codec vector only\n");
    printf("  can_gen --std --id 0x123 --data 11223344 --ack\n");
}

static void print_raw_bits(const uint8_t *bits, uint32_t nbits) {
    uint32_t i;
    for (i = 0; i < nbits; i++) {
        printf("%u", bits_get(bits, i));
        if ((i + 1) % 8 == 0)
            printf(" ");
    }
    printf("\n");
}

int main(int argc, char *argv[]) {
    can_frame_t frame = {0};
    frame.id = 0x123;
    frame.dlc = 8;
    frame.flags = 0;
    {
        static const uint8_t def[8] = {0x12, 0x34, 0x56, 0x78,
                                       0x9a, 0xbc, 0xde, 0xf0};
        memcpy(frame.data, def, sizeof(def));
    }

    frame_validity_t validity = VALID_FRAME;
    int output_raw = 0;
    int output_c = 0;
    int ack = 0;
    int dlc_set = 0;

    static struct option long_options[] = {
        {"std",    no_argument,       0, 's'},
        {"ext",    no_argument,       0, 'e'},
        {"rtr",    no_argument,       0, 'r'},
        {"id",     required_argument, 0, 'i'},
        {"data",   required_argument, 0, 'd'},
        {"dlc",    required_argument, 0, 'l'},
        {"invalid",required_argument, 0, 'v'},
        {"ack",    no_argument,       0, 'k'},
        {"raw",    no_argument,       0, 'a'},
        {"c",      no_argument,       0, 'c'},
        {"help",   no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "seird:l:v:kach", long_options,
                              NULL)) != -1) {
        switch (opt) {
        case 's':
            frame.flags &= (uint8_t)~CAN_FRAME_EXTENDED;
            break;
        case 'e':
            frame.flags |= CAN_FRAME_EXTENDED;
            break;
        case 'r':
            frame.flags |= CAN_FRAME_RTR;
            break;
        case 'i':
            frame.id = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'd': {
            const char *hex = optarg;
            uint32_t byte_idx = 0;
            uint8_t nibble_val = 0;
            int nibble_count = 0;
            memset(frame.data, 0, sizeof(frame.data));
            frame.dlc = 0;
            while (*hex && byte_idx < 8) {
                uint8_t nibble;
                if (*hex >= '0' && *hex <= '9')
                    nibble = (uint8_t)(*hex - '0');
                else if (*hex >= 'A' && *hex <= 'F')
                    nibble = (uint8_t)(*hex - 'A' + 10);
                else if (*hex >= 'a' && *hex <= 'f')
                    nibble = (uint8_t)(*hex - 'a' + 10);
                else {
                    hex++;
                    continue;
                }
                if (nibble_count == 0) {
                    nibble_val = (uint8_t)(nibble << 4);
                } else {
                    frame.data[byte_idx++] = (uint8_t)(nibble_val | nibble);
                    if (byte_idx > frame.dlc)
                        frame.dlc = (uint8_t)byte_idx;
                }
                nibble_count = !nibble_count;
                hex++;
            }
            if (nibble_count && byte_idx < 8) {
                frame.data[byte_idx++] = nibble_val;
                if (byte_idx > frame.dlc)
                    frame.dlc = (uint8_t)byte_idx;
            }
            break;
        }
        case 'l':
            frame.dlc = (uint8_t)strtol(optarg, NULL, 10);
            dlc_set = 1;
            break;
        case 'v':
            if (strcmp(optarg, "crc") == 0)
                validity = INVALID_CRC;
            else if (strcmp(optarg, "stuff") == 0)
                validity = INVALID_STUFF;
            else if (strcmp(optarg, "bit") == 0)
                validity = INVALID_BIT;
            else {
                fprintf(stderr, "Unknown invalid type: %s\n", optarg);
                return 1;
            }
            break;
        case 'k':
            ack = 1;
            break;
        case 'a':
            output_raw = 1;
            break;
        case 'c':
            output_c = 1;
            break;
        case 'h':
            print_usage();
            return 0;
        default:
            print_usage();
            return 1;
        }
    }

    if (frame.dlc > 8) {
        fprintf(stderr, "Error: DLC must be 0-8\n");
        return 1;
    }
    if (!dlc_set && (frame.flags & CAN_FRAME_RTR))
        frame.dlc = 0;

    uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
    can_build_info_t info;
    memset(bits, 0, sizeof(bits));
    memset(&info, 0, sizeof(info));

    uint32_t total_bits = can_frame_build(&frame, validity, ack, bits,
                                          CAN2040_LITE_MAX_FRAME_BITS, &info);
    if (total_bits == 0) {
        fprintf(stderr, "Error: frame generation failed\n");
        return 1;
    }

    can_parsed_t parsed;
    frame_validity_t decoded = can_frame_decode(bits, total_bits, &parsed);

    if (output_c) {
        printf("static const uint8_t can_frame_bits[] = {\n");
        uint32_t byte_idx;
        for (byte_idx = 0; byte_idx < (total_bits + 7) / 8; byte_idx++) {
            printf("    0x%02X,", bits[byte_idx]);
            if ((byte_idx + 1) % 8 == 0)
                printf("\n");
        }
        printf("\n};\n");
        printf("static const uint32_t can_frame_total_bits = %u;\n",
               total_bits);
        printf("static const uint32_t can_frame_stuffed_bits = %u;\n",
               info.stuffed_bits);
        printf("static const uint16_t can_frame_crc = 0x%04X;\n", info.crc);
        return 0;
    }

    if (output_raw) {
        printf("Frame type: %s\n",
               (frame.flags & CAN_FRAME_EXTENDED) ? "EXTENDED" : "STANDARD");
        printf("ID: 0x%0*X\n",
               (frame.flags & CAN_FRAME_EXTENDED) ? 8 : 3, frame.id);
        printf("RTR: %s\n", (frame.flags & CAN_FRAME_RTR) ? "YES" : "NO");
        printf("DLC: %u\n", frame.dlc);
        printf("Validity: %s\n", validity_name(validity));
        printf("CRC: 0x%04X\n", info.crc);
        printf("Stuffed bits: %u\n", info.stuffed_bits);
        printf("Total bits: %u\n", total_bits);
        printf("Bits: ");
        print_raw_bits(bits, total_bits);
        printf("Decode: %s\n", validity_name(decoded));
        return 0;
    }

    printf("=== CAN Frame ===\n");
    printf("Type: %s%s\n",
           (frame.flags & CAN_FRAME_EXTENDED) ? "Extended (29-bit)"
                                              : "Standard (11-bit)",
           (frame.flags & CAN_FRAME_RTR) ? ", RTR" : "");
    printf("ID: 0x%0*X\n",
           (frame.flags & CAN_FRAME_EXTENDED) ? 8 : 3, frame.id);
    printf("DLC: %u\n", frame.dlc);
    if (!(frame.flags & CAN_FRAME_RTR)) {
        uint32_t i;
        printf("Data: ");
        for (i = 0; i < frame.dlc; i++)
            printf("%02X ", frame.data[i]);
        printf("\n");
    }
    printf("Validity: %s\n", validity_name(validity));
    printf("CRC: 0x%04X\n", info.crc);
    printf("Unstuffed prefix: %u bits\n", info.unstuffed_bits);
    printf("Stuffed prefix:   %u bits (%u stuff bits)\n",
           info.stuffed_bits, info.stuffed_bits - info.unstuffed_bits);
    printf("Total on wire:    %u bits\n", total_bits);
    printf("ACK slot: %s\n", ack ? "dominant (ACK)" : "recessive (NACK)");

    printf("--- Decode echo ---\n");
    printf("Result: %s\n", validity_name(decoded));
    if (decoded == VALID_FRAME) {
        uint32_t i;
        printf("ID: 0x%0*X  DLC: %u  %s%s\n",
               parsed.is_extended ? 8 : 3, parsed.id, parsed.dlc,
               parsed.is_extended ? "extended" : "standard",
               parsed.is_rtr ? " RTR" : "");
        printf("ACK: %s\n", parsed.ack ? "ACK" : "NACK");
        printf("CRC: 0x%04X (calculated 0x%04X)\n",
               parsed.crc, parsed.crc_calc);
        if (!parsed.is_rtr) {
            printf("Data: ");
            for (i = 0; i < parsed.dlc; i++)
                printf("%02X ", parsed.data[i]);
            printf("\n");
        }
    } else if (decoded == INVALID_CRC) {
        printf("CRC received 0x%04X, calculated 0x%04X\n",
               parsed.crc, parsed.crc_calc);
    }
    return 0;
}
