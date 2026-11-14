// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// rx_bit_bench.cpp - On-target per-bit RX parser benchmark.
//
// The first RX bench measures whole FIFO words and edges.  This one removes
// stuffing from standard 11-bit, DLC-8 frames and times can_rx_advance() once
// per unstuffed wire bit.  The result is a per-bit-position cost table for a
// standard CAN frame: SOF, ID, RTR, IDE, R0, DLC, data, CRC, CRC delimiter and
// ACK slot.
//
// No PIO, no backend, no CAN controller.  The timed call is the same inline
// state-machine step the production RX ISR uses, placed in SRAM.

#include <Arduino.h>

#include <hardware/clocks.h>
#include <hardware/structs/systick.h>
#include <hardware/sync.h>
#include <math.h>
#include <pico/platform.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can2040_lite_proto.h"
#include "can_frame_build.h"
#include "can_rx_sm.h"
#include "can_rx_sm_inline.h"

#define BENCH_FRAMES            100u
#define BENCH_ROUNDS            8u
#define BENCH_REPS              8u
#define BENCH_PREFIX_REPS       1u
#define BENCH_OVERHEAD_SAMPLES  4096u

// Standard 11-bit, DLC 8: SOF..CRC is 98 unstuffed bits; the CRC delimiter
// and ACK slot add two more, which is where can_rx_advance() stops.
#define STD_DLC                 8u
#define STD_PREFIX_BITS         98u
#define STD_TOTAL_BITS          100u

#define SYSTICK_MASK            0x00ffffffu

struct frame_meta {
    uint32_t id;
    uint32_t wire_bits;
    uint8_t data[8];
};

struct stats_t {
    uint32_t n;
    double min;
    double max;
    double mean;
    double sd;
    double p05;
    double p50;
    double p95;
    double p99;
};

struct hist_t {
    uint32_t width;
    uint32_t buckets;
    uint32_t max_count;
    uint32_t count[48];
};

static uint32_t g_ticks_per_bit;
static uint8_t g_bits[BENCH_FRAMES][STD_TOTAL_BITS];
static struct frame_meta g_frame[BENCH_FRAMES];
static uint32_t g_round[BENCH_ROUNDS][BENCH_FRAMES];
static uint32_t g_msg[BENCH_ROUNDS * BENCH_FRAMES];
static uint32_t g_prefix[BENCH_FRAMES][STD_TOTAL_BITS + 1u];
static uint16_t g_samples[BENCH_FRAMES][STD_TOTAL_BITS];
static uint16_t g_overhead[BENCH_OVERHEAD_SAMPLES];
static uint16_t g_all[BENCH_FRAMES * STD_TOTAL_BITS];

static uint32_t g_rng = 0x9e3779b9u;

static uint32_t rng_next(void) {
    uint32_t x = g_rng;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}

static inline __attribute__((always_inline)) uint32_t cvr_now(void) {
    return systick_hw->cvr;
}

static inline __attribute__((always_inline)) uint32_t cvr_delta(uint32_t start,
                                                                uint32_t end) {
    return (start - end) & SYSTICK_MASK;
}

static void format_f2(double v, char *out, size_t len) {
    uint32_t whole = (uint32_t)v;
    uint32_t frac = (uint32_t)((v - (double)whole) * 100.0 + 0.5);

    if (frac >= 100u) {
        whole++;
        frac = 0;
    }
    snprintf(out, len, "%lu.%02lu", (unsigned long)whole,
             (unsigned long)frac);
}

static int cmp_u16(const void *a, const void *b) {
    uint16_t x = *(const uint16_t *)a;
    uint16_t y = *(const uint16_t *)b;

    return (x > y) - (x < y);
}

static int cmp_u32(const void *a, const void *b) {
    uint32_t x = *(const uint32_t *)a;
    uint32_t y = *(const uint32_t *)b;

    return (x > y) - (x < y);
}

static double percentile_u16(const uint16_t *v, uint32_t n, double p) {
    uint32_t idx = (uint32_t)(p * (double)(n - 1u) + 0.5);

    if (idx >= n)
        idx = n - 1u;
    return (double)v[idx];
}

static double percentile_u32(const uint32_t *v, uint32_t n, double p) {
    uint32_t idx = (uint32_t)(p * (double)(n - 1u) + 0.5);

    if (idx >= n)
        idx = n - 1u;
    return (double)v[idx];
}

static void stats_u16(uint16_t *v, uint32_t n, struct stats_t *s) {
    double mean = 0.0;
    double m2 = 0.0;
    uint32_t i;

    qsort(v, n, sizeof(v[0]), cmp_u16);
    for (i = 0; i < n; i++) {
        double x = (double)v[i];
        double d = x - mean;

        mean += d / (double)(i + 1u);
        m2 += d * (x - mean);
    }
    s->n = n;
    s->min = (double)v[0];
    s->max = (double)v[n - 1u];
    s->mean = mean;
    s->sd = n > 1u ? sqrt(m2 / (double)n) : 0.0;
    s->p05 = percentile_u16(v, n, 0.05);
    s->p50 = percentile_u16(v, n, 0.50);
    s->p95 = percentile_u16(v, n, 0.95);
    s->p99 = percentile_u16(v, n, 0.99);
}

static void stats_u32(uint32_t *v, uint32_t n, struct stats_t *s) {
    double mean = 0.0;
    double m2 = 0.0;
    uint32_t i;

    qsort(v, n, sizeof(v[0]), cmp_u32);
    for (i = 0; i < n; i++) {
        double x = (double)v[i];
        double d = x - mean;

        mean += d / (double)(i + 1u);
        m2 += d * (x - mean);
    }
    s->n = n;
    s->min = (double)v[0];
    s->max = (double)v[n - 1u];
    s->mean = mean;
    s->sd = n > 1u ? sqrt(m2 / (double)n) : 0.0;
    s->p05 = percentile_u32(v, n, 0.05);
    s->p50 = percentile_u32(v, n, 0.50);
    s->p95 = percentile_u32(v, n, 0.95);
    s->p99 = percentile_u32(v, n, 0.99);
}

static void print_stats(const char *label, const struct stats_t *s) {
    char min[16], p05[16], p50[16], mean[16], p95[16], p99[16], max[16], sd[16];

    format_f2(s->min, min, sizeof(min));
    format_f2(s->p05, p05, sizeof(p05));
    format_f2(s->p50, p50, sizeof(p50));
    format_f2(s->mean, mean, sizeof(mean));
    format_f2(s->p95, p95, sizeof(p95));
    format_f2(s->p99, p99, sizeof(p99));
    format_f2(s->max, max, sizeof(max));
    format_f2(s->sd, sd, sizeof(sd));
    Serial.printf("  %-18s n=%5lu min=%8s p05=%8s p50=%8s mean=%8s "
                  "p95=%8s p99=%8s max=%8s sd=%7s\n",
                  label, (unsigned long)s->n, min, p05, p50, mean,
                  p95, p99, max, sd);
}

static void hist_build(const uint16_t *v, uint32_t n, struct hist_t *h) {
    uint32_t max = 0;
    uint32_t i;

    memset(h, 0, sizeof(*h));
    for (i = 0; i < n; i++) {
        if (v[i] > max)
            max = v[i];
    }
    h->width = max / 24u;
    if (h->width < 2u)
        h->width = 2u;
    h->width = (h->width + 1u) & ~1u;
    h->buckets = max / h->width + 1u;
    if (h->buckets > 48u)
        h->buckets = 48u;
    for (i = 0; i < n; i++) {
        uint32_t b = v[i] / h->width;

        if (b >= h->buckets)
            b = h->buckets - 1u;
        h->count[b]++;
    }
    for (i = 0; i < h->buckets; i++) {
        if (h->count[i] > h->max_count)
            h->max_count = h->count[i];
    }
}

static void hist_print(const char *label, const struct hist_t *h) {
    uint32_t scale = (h->max_count + 39u) / 40u;
    uint32_t b;
    uint32_t k;

    if (scale == 0)
        scale = 1;
    Serial.printf("  %s (bucket width %lu cycles)\n", label,
                  (unsigned long)h->width);
    for (b = 0; b < h->buckets; b++) {
        uint32_t lo = b * h->width;
        uint32_t hi = lo + h->width - 1u;

        Serial.printf("    %5lu..%5lu %6lu ", (unsigned long)lo,
                      (unsigned long)hi, (unsigned long)h->count[b]);
        for (k = 0; k < h->count[b] / scale; k++)
            Serial.print('#');
        Serial.println();
    }
}

static void fill_data(can_frame_t *f, uint32_t n) {
    uint32_t i;

    switch (n % 6u) {
    case 0:
        for (i = 0; i < 8u; i++)
            f->data[i] = 0xaau;
        break;
    case 1:
        for (i = 0; i < 8u; i++)
            f->data[i] = 0x55u;
        break;
    case 2:
        for (i = 0; i < 8u; i++)
            f->data[i] = 0xffu;
        break;
    case 3:
        for (i = 0; i < 8u; i++)
            f->data[i] = 0x00u;
        break;
    case 4:
        for (i = 0; i < 8u; i++)
            f->data[i] = (uint8_t)(0x5au ^ (n * 13u + i * 29u));
        break;
    default:
        for (i = 0; i < 8u; i++)
            f->data[i] = (uint8_t)rng_next();
        break;
    }
}

// Remove the inserted stuff bits from a valid stuffed prefix.  This is the
// same operation the receiver performs on the wire; the timed loop below
// therefore starts from exactly the bits can_rx_advance() sees.
static uint32_t unstuff_bits(const uint8_t *in, uint32_t nbits,
                             uint8_t *out, uint32_t max_bits) {
    uint32_t oi = 0;
    uint32_t run = 0;
    uint32_t i;
    uint8_t last = 0;

    for (i = 0; i < nbits && oi < max_bits; i++) {
        uint8_t bit = bits_get(in, i);

        if (run == 5u) {
            // The encoder inserted the opposite bit after the five-run, and
            // the transmitter counts that stuff bit as the start of the next
            // run.  Keep that run state so a following same-value data bit is
            // counted correctly.
            if (bit == last)
                return CAN_BITS_ERROR;
            run = 1;
            last = bit;
            continue;
        }
        bits_put(out, oi++, bit);
        if (run > 0u && bit == last)
            run++;
        else {
            last = bit;
            run = 1;
        }
    }
    if (run == 5u)
        return CAN_BITS_ERROR;
    return oi;
}

static void build_corpus(void) {
    uint32_t n;

    g_rng = 0x9e3779b9u;
    for (n = 0; n < BENCH_FRAMES; n++) {
        can_frame_t f;
        can_build_info_t info;
        uint8_t stuffed[CAN2040_LITE_MAX_FRAME_BYTES];
        uint8_t unstuffed[CAN2040_LITE_MAX_FRAME_BYTES];
        uint32_t total;
        uint32_t nu;
        uint32_t i;

        memset(&f, 0, sizeof(f));
        memset(&info, 0, sizeof(info));
        memset(stuffed, 0, sizeof(stuffed));
        memset(unstuffed, 0, sizeof(unstuffed));

        f.id = rng_next() & 0x7ffu;
        f.dlc = STD_DLC;
        f.flags = 0;
        fill_data(&f, n);

        total = can_frame_build(&f, VALID_FRAME, 1, stuffed,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        if (total == 0u) {
            Serial.printf("corpus: frame %lu build failed\n", (unsigned long)n);
            while (1)
                delay(1000);
        }
        nu = unstuff_bits(stuffed, info.stuffed_bits, unstuffed,
                          CAN2040_LITE_MAX_FRAME_BITS);
        if (nu != STD_PREFIX_BITS || nu != info.unstuffed_bits) {
            Serial.printf("corpus: frame %lu unstuff %lu/%lu/%lu\n",
                          (unsigned long)n, (unsigned long)nu,
                          (unsigned long)info.unstuffed_bits,
                          (unsigned long)STD_PREFIX_BITS);
            while (1)
                delay(1000);
        }
        for (i = 0; i < nu; i++)
            g_bits[n][i] = bits_get(unstuffed, i);
        g_bits[n][STD_PREFIX_BITS] = 1u;        // CRC delimiter
        g_bits[n][STD_PREFIX_BITS + 1u] = 0u;   // dominant ACK slot

        g_frame[n].id = f.id;
        g_frame[n].wire_bits = info.total_bits;
        memcpy(g_frame[n].data, f.data, sizeof(g_frame[n].data));
    }
}

static uint32_t feed_bits(can_rx_t *rx, const uint8_t *bits, uint32_t nbits) {
    uint32_t i;

    for (i = 0; i < nbits; i++)
        can_rx_advance(rx, bits[i]);
    return nbits;
}

static uint32_t __no_inline_not_in_flash_func(bench_advance_reps)(
        can_rx_t *rx, const uint8_t *bits, uint32_t nbits, uint32_t reps) {
    uint32_t t0;
    uint32_t t1;
    uint32_t r;
    uint32_t i;
    uint32_t save;

    can_rx_reset(rx);
    save = save_and_disable_interrupts();
    t0 = cvr_now();
    for (r = 0; r < reps; r++) {
        for (i = 0; i < nbits; i++)
            can_rx_advance(rx, bits[i]);
    }
    __asm__ volatile("" ::: "memory");
    t1 = cvr_now();
    restore_interrupts(save);
    return cvr_delta(t0, t1);
}

static uint32_t __no_inline_not_in_flash_func(bench_advance_calls)(
        can_rx_t *rx, const uint8_t *bits, uint32_t nbits, uint16_t *out) {
    uint32_t save;
    uint32_t i;

    save = save_and_disable_interrupts();
    for (i = 0; i < nbits; i++) {
        uint32_t t0 = cvr_now();
        uint32_t d;

        can_rx_advance(rx, bits[i]);
        __asm__ volatile("" ::: "memory");
        d = cvr_delta(t0, cvr_now());
        out[i] = (uint16_t)(d > 0xffffu ? 0xffffu : d);
    }
    restore_interrupts(save);
    return nbits;
}

static uint32_t __no_inline_not_in_flash_func(bench_overhead)(uint16_t *out,
                                                              uint32_t n) {
    uint32_t save;
    uint32_t i;

    save = save_and_disable_interrupts();
    for (i = 0; i < n; i++) {
        uint32_t t0 = cvr_now();
        uint32_t d;

        __asm__ volatile("" ::: "memory");
        d = cvr_delta(t0, cvr_now());
        out[i] = (uint16_t)(d > 0xffffu ? 0xffffu : d);
    }
    restore_interrupts(save);
    return n;
}

static uint32_t verify_corpus(void) {
    can_rx_t rx;
    uint32_t fails = 0;
    uint32_t n;

    can_rx_init(&rx, g_ticks_per_bit);
    for (n = 0; n < BENCH_FRAMES; n++) {
        struct frame_meta *m = &g_frame[n];
        uint32_t crc0;
        uint32_t stuff0;
        uint32_t bit0;
        uint32_t ov0;
        uint32_t ack0;
        bool ok;

        can_rx_reset(&rx);
        crc0 = rx.crc_errors;
        stuff0 = rx.stuff_errors;
        bit0 = rx.bit_errors;
        ov0 = rx.overflow;
        ack0 = rx.ack_errors;
        feed_bits(&rx, g_bits[n], STD_TOTAL_BITS);

        ok = can_rx_available(&rx) != 0u;
        if (ok) {
            const can_rx_frame_t *g = can_rx_front(&rx);
            uint8_t gd[8];

            can_rx_frame_data(g, gd);
            ok = can_rx_frame_id(g) == m->id &&
                 can_rx_frame_dlc(g) == STD_DLC &&
                 (can_rx_frame_flags(g) & 1u) == 0u &&
                 memcmp(gd, m->data, sizeof(m->data)) == 0;
            can_rx_pop(&rx);
        }
        ok = ok && rx.crc_errors == crc0 && rx.stuff_errors == stuff0 &&
             rx.bit_errors == bit0 && rx.overflow == ov0 &&
             rx.ack_errors == ack0 &&
             rx.pos == CAN_BIT_WAIT_FRAME;
        if (!ok) {
            Serial.printf("verify: frame %lu FAILED\n", (unsigned long)n);
            fails++;
        }
    }
    return fails;
}

static void run_timing(void) {
    can_rx_t rx;
    uint32_t round;
    uint32_t n;

    can_rx_init(&rx, g_ticks_per_bit);
    for (n = 0; n < BENCH_FRAMES; n++)
        (void)bench_advance_reps(&rx, g_bits[n], STD_TOTAL_BITS, 1u);

    for (round = 0; round < BENCH_ROUNDS; round++) {
        for (n = 0; n < BENCH_FRAMES; n++) {
            g_round[round][n] = bench_advance_reps(&rx, g_bits[n],
                                                   STD_TOTAL_BITS, BENCH_REPS);
            g_msg[round * BENCH_FRAMES + n] =
                (g_round[round][n] + BENCH_REPS / 2u) / BENCH_REPS;
        }
        Serial.print('.');
    }
    Serial.println();
}

static void run_call_samples(void) {
    can_rx_t rx;
    uint32_t n;

    can_rx_init(&rx, g_ticks_per_bit);
    for (n = 0; n < BENCH_FRAMES; n++) {
        can_rx_reset(&rx);
        (void)bench_advance_calls(&rx, g_bits[n], STD_TOTAL_BITS,
                                  g_samples[n]);
    }
    (void)bench_overhead(g_overhead, BENCH_OVERHEAD_SAMPLES);
}

// Marginal per-bit cost by differencing prefix timings.  This avoids placing
// a timer read inside the parser loop: T(p+1) - T(p) is one more call plus
// the loop iteration, and the outer loop, timer reads and reset all cancel.
static void run_prefix_timing(void) {
    can_rx_t rx;
    uint32_t f;
    uint32_t p;

    can_rx_init(&rx, g_ticks_per_bit);
    for (f = 0; f < BENCH_FRAMES; f++) {
        for (p = 0; p <= STD_TOTAL_BITS; p++) {
            g_prefix[f][p] = bench_advance_reps(&rx, g_bits[f], p,
                                                BENCH_PREFIX_REPS);
        }
    }
    for (f = 0; f < BENCH_FRAMES; f++) {
        for (p = 0; p < STD_TOTAL_BITS; p++) {
            uint32_t d = g_prefix[f][p + 1u] - g_prefix[f][p];

            g_samples[f][p] = (uint16_t)((d + BENCH_PREFIX_REPS / 2u) /
                                         BENCH_PREFIX_REPS);
        }
    }
}

static void bit_name(uint32_t idx, char *out, size_t len) {
    if (idx == 0u) {
        snprintf(out, len, "SOF");
    } else if (idx <= 11u) {
        snprintf(out, len, "ID%lu", (unsigned long)(11u - idx));
    } else if (idx == 12u) {
        snprintf(out, len, "RTR");
    } else if (idx == 13u) {
        snprintf(out, len, "IDE");
    } else if (idx == 14u) {
        snprintf(out, len, "R0");
    } else if (idx <= 18u) {
        snprintf(out, len, "DLC%lu", (unsigned long)(18u - idx));
    } else if (idx <= 82u) {
        snprintf(out, len, "D%lu", (unsigned long)(idx - 19u));
    } else if (idx == 90u) {
        snprintf(out, len, "CRC7 cmp");
    } else if (idx == 97u) {
        snprintf(out, len, "CRC0 cmp");
    } else if (idx <= 97u) {
        snprintf(out, len, "CRC%lu", (unsigned long)(97u - idx));
    } else if (idx == 98u) {
        snprintf(out, len, "CRC-DELIM");
    } else if (idx == 99u) {
        snprintf(out, len, "ACK-SLOT");
    } else {
        snprintf(out, len, "?");
    }
}

static const char *field_name(uint32_t idx) {
    if (idx == 0u)
        return "SOF";
    if (idx <= 11u)
        return "Base ID";
    if (idx <= 14u)
        return "Control";
    if (idx <= 18u)
        return "DLC";
    if (idx <= 82u)
        return "Data";
    if (idx <= 97u)
        return "CRC";
    return "Tail";
}

static void print_field_stats(void) {
    static const char *names[] = {
        "SOF", "Base ID", "Control", "DLC", "Data", "CRC", "Tail"
    };
    uint32_t f;
    uint32_t i;
    uint32_t n;

    Serial.println();
    Serial.println("-- Cost by field (prefix-differenced marginal cost) ----------");
    for (f = 0; f < 7u; f++) {
        n = 0;
        for (i = 0; i < BENCH_FRAMES * STD_TOTAL_BITS; i++) {
            uint32_t pos = i % STD_TOTAL_BITS;

            if (strcmp(field_name(pos), names[f]) == 0)
                g_all[n++] = g_samples[i / STD_TOTAL_BITS][pos];
        }
        {
            struct stats_t s;

            stats_u16(g_all, n, &s);
            print_stats(names[f], &s);
        }
    }
}

static void print_position_table(void) {
    uint32_t pos;
    uint32_t f;
    char name[16];

    Serial.println();
    Serial.println("-- Per-bit marginal cost, standard 11-bit DLC-8 frame --------");
    Serial.println("  (T(bit+1) - T(bit); one can_rx_advance call plus loop step)");
    Serial.println("  bit  name        n    min      p50     mean     p95      max      sd");
    for (pos = 0; pos < STD_TOTAL_BITS; pos++) {
        struct stats_t s;
        char min[16], p50[16], mean[16], p95[16], max[16], sd[16];

        for (f = 0; f < BENCH_FRAMES; f++)
            g_all[f] = g_samples[f][pos];
        stats_u16(g_all, BENCH_FRAMES, &s);
        bit_name(pos, name, sizeof(name));
        format_f2(s.min, min, sizeof(min));
        format_f2(s.p50, p50, sizeof(p50));
        format_f2(s.mean, mean, sizeof(mean));
        format_f2(s.p95, p95, sizeof(p95));
        format_f2(s.max, max, sizeof(max));
        format_f2(s.sd, sd, sizeof(sd));
        Serial.printf("  %3lu  %-10s %4lu %7s %8s %8s %8s %8s %8s\n",
                      (unsigned long)pos, name,
                      (unsigned long)s.n, min, p50, mean, p95, max, sd);
    }
}

static void run_bench(void) {
    uint32_t hz = clock_get_hz(clk_sys);
    uint32_t ticks = hz / 2000000u;
    uint32_t fails;
    uint64_t total = 0;
    uint64_t wire_bits = 0;
    uint32_t r;
    uint32_t n;
    uint32_t i;
    struct stats_t msg_stats;
    struct stats_t call_stats;
    struct stats_t corr_stats;
    struct stats_t overhead_stats;
    struct hist_t hist;
    double cycles_frame;
    double cycles_bit;
    double cycles_wire_bit;
    double load;
    char cf[16], cb[16], cw[16], load_s[16];

    if (ticks < 8u)
        ticks = 8u;
    if (ticks > 200u)
        ticks = 200u;
    g_ticks_per_bit = ticks;

    Serial.println();
    Serial.println("============================================================");
    Serial.println(" CAN2040-Lite RX per-bit parser benchmark (software only)");
    Serial.println("============================================================");
    Serial.printf("clock=%lu Hz  ticks_per_bit=%lu  ack_support=%u\n",
                  (unsigned long)hz, (unsigned long)ticks,
                  (unsigned)CAN_ACK_GENERATION_SUPPORT);
    Serial.printf("frames=%lu  bits/frame=%lu (standard 11-bit, DLC %lu)\n",
                  (unsigned long)BENCH_FRAMES, (unsigned long)STD_TOTAL_BITS,
                  (unsigned long)STD_DLC);
    Serial.println("bit sequence: SOF..CRC (98) + CRC delimiter + ACK slot");

    build_corpus();
    fails = verify_corpus();
    Serial.printf("verify: %lu failures\n", (unsigned long)fails);

    Serial.print("timing ");
    run_timing();

    for (r = 0; r < BENCH_ROUNDS; r++) {
        for (n = 0; n < BENCH_FRAMES; n++)
            total += g_round[r][n];
    }
    for (n = 0; n < BENCH_FRAMES; n++)
        wire_bits += (uint64_t)g_frame[n].wire_bits * BENCH_ROUNDS * BENCH_REPS;

    cycles_frame = (double)total /
                   ((double)BENCH_ROUNDS * BENCH_FRAMES * BENCH_REPS);
    cycles_bit = (double)total /
                 ((double)BENCH_ROUNDS * BENCH_FRAMES * BENCH_REPS *
                  STD_TOTAL_BITS);
    cycles_wire_bit = (double)total / (double)wire_bits;
    load = cycles_wire_bit / (hz / 1000000.0);

    run_call_samples();

    stats_u32(g_msg, BENCH_ROUNDS * BENCH_FRAMES, &msg_stats);
    {
        uint32_t k = 0;

        for (n = 0; n < BENCH_FRAMES; n++) {
            for (i = 0; i < STD_TOTAL_BITS; i++)
                g_all[k++] = g_samples[n][i];
        }
    }
    stats_u16(g_all, BENCH_FRAMES * STD_TOTAL_BITS, &call_stats);
    hist_build(g_all, BENCH_FRAMES * STD_TOTAL_BITS, &hist);
    stats_u16(g_overhead, BENCH_OVERHEAD_SAMPLES, &overhead_stats);
    {
        uint32_t med = (uint32_t)(overhead_stats.p50 + 0.5);

        for (n = 0; n < BENCH_FRAMES; n++) {
            for (i = 0; i < STD_TOTAL_BITS; i++)
                g_samples[n][i] = (uint16_t)(g_samples[n][i] > med ?
                                             g_samples[n][i] - med : 0u);
        }
    }
    {
        uint32_t k = 0;

        for (n = 0; n < BENCH_FRAMES; n++) {
            for (i = 0; i < STD_TOTAL_BITS; i++)
                g_all[k++] = g_samples[n][i];
        }
    }
    stats_u16(g_all, BENCH_FRAMES * STD_TOTAL_BITS, &corr_stats);

    // Replace the isolated-call samples with marginal prefix differences for
    // the per-field and per-bit tables.  The isolated-call stats above are
    // already captured.
    run_prefix_timing();

    format_f2(cycles_frame, cf, sizeof(cf));
    format_f2(cycles_bit, cb, sizeof(cb));
    format_f2(cycles_wire_bit, cw, sizeof(cw));
    format_f2(load * 100.0, load_s, sizeof(load_s));

    Serial.println();
    Serial.println("-- Corpus ----------------------------------------------------");
    {
        char wpf[16];

        format_f2((double)wire_bits /
                  ((double)BENCH_ROUNDS * BENCH_REPS * BENCH_FRAMES),
                  wpf, sizeof(wpf));
        Serial.printf("  frames=%lu  unstuffed bits/frame=%lu  "
                      "wire bits/frame=%s\n",
                      (unsigned long)BENCH_FRAMES,
                      (unsigned long)STD_TOTAL_BITS, wpf);
    }
    Serial.printf("  can_rx_advance calls/pass=%lu\n",
                  (unsigned long)(BENCH_FRAMES * STD_TOTAL_BITS));

    Serial.println();
    Serial.println("-- Cost summary (authoritative, whole-frame timing) ----------");
    Serial.printf("  cycles / frame            %s\n", cf);
    Serial.printf("  cycles / unstuffed bit    %s\n", cb);
    Serial.printf("  cycles / wire bit (approx) %s\n", cw);
    Serial.printf("  avg load @ 1 Mbit         %s %%\n", load_s);
    Serial.println("  (wire-bit load excludes edge/run classification)");

    Serial.println();
    Serial.println("-- Per-message distribution (cycles/frame) -------------------");
    print_stats("100-bit frame", &msg_stats);

    Serial.println();
    Serial.println("-- Per-call distribution (raw, includes timer reads) ---------");
    print_stats("can_rx_advance", &call_stats);
    print_stats("timer overhead", &overhead_stats);

    Serial.println();
    Serial.println("-- Per-call distribution (overhead-corrected) ----------------");
    print_stats("can_rx_advance", &corr_stats);

    Serial.println();
    Serial.println("-- Per-bit tables below use prefix differencing -------------");
    print_field_stats();
    print_position_table();

    Serial.println();
    Serial.println("-- Per-call histogram (raw cycles) ---------------------------");
    hist_print("can_rx_advance", &hist);

    Serial.println();
    Serial.printf("verify: %lu failures, corpus decode %s\n",
                  (unsigned long)fails, fails ? "FAIL" : "PASS");
    Serial.println("=== RX BIT BENCH DONE ===");
    Serial.flush();
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    run_bench();
}

void loop() {
    if (Serial.available()) {
        int c = Serial.read();

        while (Serial.available())
            (void)Serial.read();
        if (c == 'r' || c == 'R' || c == '\n' || c == '\r')
            run_bench();
    }
    delay(50);
}
