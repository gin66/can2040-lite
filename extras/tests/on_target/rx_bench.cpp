// SPDX-License-Identifier: GPL-3.0-only
//
// Copyright (C) 2026 Jochen Kiemes
// CAN2040-Lite is derived from can2040 (Copyright (C) 2021 Kevin O'Connor),
// licensed under GPL-3.0-only. See LICENSE for the full license text.
//
// rx_bench.cpp - On-target RX parser cycle benchmark.
//
// No PIO, no backend, no CAN controller: this build contains only the
// portable parser (can_rx_sm.c + can_rx_sm_inline.h), the protocol helpers,
// and the frame builder.  100 generated CAN 2.0B frames are turned into the
// exact PIO FIFO words the receiver would see, then parsed with
// can_rx_pair_inline() and with two calls to can_rx_edge_inline().
//
// Timing uses the Cortex-M0+ SysTick down-counter (processor clock).  The
// per-frame totals bracket several parses, so timer overhead is negligible;
// the per-call pass reports the raw and overhead-corrected distribution.
// All timed code is placed in SRAM, matching the production RX IRQ.

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
#define BENCH_MAX_WORDS         8192u
#define BENCH_MAX_EDGES         (2u * BENCH_MAX_WORDS)
#define BENCH_ROUNDS            8u
#define BENCH_REPS              8u
#define BENCH_OVERHEAD_SAMPLES  4096u
#define BENCH_T0                0x8000u
#define SYSTICK_MASK            0x00ffffffu

struct frame_meta {
    uint32_t id;
    uint32_t total_bits;
    uint32_t stuffed_bits;
    uint32_t unstuffed_bits;
    uint16_t words;
    uint8_t dlc;
    uint8_t ext;
    uint8_t rtr;
    uint8_t ack;
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
static uint32_t g_words[BENCH_MAX_WORDS];
static uint32_t g_word_count;
static uint32_t g_frame_off[BENCH_FRAMES + 1u];
static struct frame_meta g_frame[BENCH_FRAMES];

static uint32_t g_pair_round[BENCH_ROUNDS][BENCH_FRAMES];
static uint32_t g_edge_round[BENCH_ROUNDS][BENCH_FRAMES];

static uint32_t g_pair_msg[BENCH_ROUNDS * BENCH_FRAMES];
static uint32_t g_edge_msg[BENCH_ROUNDS * BENCH_FRAMES];

static uint16_t g_pair_samples[BENCH_MAX_WORDS];
static uint16_t g_edge_samples[BENCH_MAX_EDGES];
static uint16_t g_overhead_samples[BENCH_OVERHEAD_SAMPLES];

static uint32_t g_pair_sample_count;
static uint32_t g_edge_sample_count;

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
    Serial.printf("  %-24s n=%5lu min=%8s p05=%8s p50=%8s mean=%8s "
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
    h->width = max / 32u;
    if (h->width < 4u)
        h->width = 4u;
    h->width = (h->width + 3u) & ~3u;
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

static void build_corpus(uint32_t ticks) {
    uint32_t n;

    g_rng = 0x9e3779b9u;
    g_word_count = 0;

    for (n = 0; n < BENCH_FRAMES; n++) {
        can_frame_t f;
        can_build_info_t info;
        uint8_t bits[CAN2040_LITE_MAX_FRAME_BYTES];
        struct frame_meta *m = &g_frame[n];
        uint32_t total;
        uint32_t start = g_word_count;
        uint32_t i;
        uint8_t prev = 1;
        uint16_t fall = 0;
        uint8_t have_fall = 0;

        memset(&f, 0, sizeof(f));
        memset(&info, 0, sizeof(info));
        memset(bits, 0, sizeof(bits));

        f.dlc = (uint8_t)(n % 9u);
        if ((n % 5u) == 0u) {
            f.flags |= CAN_FRAME_EXTENDED;
            f.id = rng_next() & 0x1fffffffu;
        } else {
            f.id = rng_next() & 0x7ffu;
        }
        // The RX parser has no RTR data-suppression state, so RTR vectors use
        // DLC 0 (the supported RTR form in the host suite).
        if ((n % 11u) == 3u) {
            f.flags |= CAN_FRAME_RTR;
            f.dlc = 0;
        }
        fill_data(&f, n);

        m->id = f.id;
        m->dlc = f.dlc;
        m->ext = (uint8_t)((f.flags & CAN_FRAME_EXTENDED) ? 1u : 0u);
        m->rtr = (uint8_t)((f.flags & CAN_FRAME_RTR) ? 1u : 0u);
        m->ack = (uint8_t)((n % 17u) != 0u);
        memcpy(m->data, f.data, sizeof(m->data));

        total = can_frame_build(&f, VALID_FRAME, m->ack ? 1 : 0, bits,
                                CAN2040_LITE_MAX_FRAME_BITS, &info);
        if (total == 0u) {
            Serial.printf("corpus: frame %lu build failed\n", (unsigned long)n);
            while (1)
                delay(1000);
        }

        m->total_bits = info.total_bits;
        m->stuffed_bits = info.stuffed_bits;
        m->unstuffed_bits = info.unstuffed_bits;

        for (i = 0; i < total; i++) {
            uint8_t bit = bits_get(bits, i);
            uint16_t ts;

            if (bit == prev)
                continue;
            ts = (uint16_t)((uint16_t)BENCH_T0 - (uint16_t)(i * ticks));
            prev = bit;
            if (bit == 0u) {
                fall = ts;
                have_fall = 1;
            } else if (have_fall) {
                if (g_word_count >= BENCH_MAX_WORDS) {
                    Serial.println("corpus: word buffer overflow");
                    while (1)
                        delay(1000);
                }
                g_words[g_word_count++] = ((uint32_t)fall << 16) | ts;
                have_fall = 0;
            }
        }
        if (have_fall) {
            Serial.printf("corpus: frame %lu ends on a fall edge\n",
                          (unsigned long)n);
            while (1)
                delay(1000);
        }
        g_frame_off[n] = start;
        m->words = (uint16_t)(g_word_count - start);
    }
    g_frame_off[BENCH_FRAMES] = g_word_count;
}

static uint32_t feed_words(can_rx_t *rx, const uint32_t *words, uint32_t nw) {
    uint32_t i;

    for (i = 0; i < nw; i++)
        can_rx_pair_inline(rx, words[i]);
    return nw;
}

static uint32_t __no_inline_not_in_flash_func(bench_pair_reps)(
        can_rx_t *rx, const uint32_t *words, uint32_t nw, uint32_t reps) {
    uint32_t t0;
    uint32_t t1;
    uint32_t r;
    uint32_t i;
    uint32_t save;

    can_rx_reset(rx);
    save = save_and_disable_interrupts();
    t0 = cvr_now();
    for (r = 0; r < reps; r++) {
        for (i = 0; i < nw; i++)
            can_rx_pair_inline(rx, words[i]);
    }
    __asm__ volatile("" ::: "memory");
    t1 = cvr_now();
    restore_interrupts(save);
    return cvr_delta(t0, t1);
}

static uint32_t __no_inline_not_in_flash_func(bench_edge_reps)(
        can_rx_t *rx, const uint32_t *words, uint32_t nw, uint32_t reps) {
    uint32_t t0;
    uint32_t t1;
    uint32_t r;
    uint32_t i;
    uint32_t save;

    can_rx_reset(rx);
    save = save_and_disable_interrupts();
    t0 = cvr_now();
    for (r = 0; r < reps; r++) {
        for (i = 0; i < nw; i++) {
            uint32_t word = words[i];

            can_rx_edge_inline(rx, (uint16_t)(word >> 16), 1u);
            can_rx_edge_inline(rx, (uint16_t)word, 0u);
        }
    }
    __asm__ volatile("" ::: "memory");
    t1 = cvr_now();
    restore_interrupts(save);
    return cvr_delta(t0, t1);
}

static uint32_t __no_inline_not_in_flash_func(bench_pair_calls)(
        can_rx_t *rx, const uint32_t *words, uint32_t nw, uint16_t *out) {
    uint32_t save;
    uint32_t i;

    save = save_and_disable_interrupts();
    for (i = 0; i < nw; i++) {
        uint32_t t0 = cvr_now();
        uint32_t d;

        can_rx_pair_inline(rx, words[i]);
        __asm__ volatile("" ::: "memory");
        d = cvr_delta(t0, cvr_now());
        out[i] = (uint16_t)(d > 0xffffu ? 0xffffu : d);
    }
    restore_interrupts(save);
    return nw;
}

static uint32_t __no_inline_not_in_flash_func(bench_edge_calls)(
        can_rx_t *rx, const uint32_t *words, uint32_t nw, uint16_t *out) {
    uint32_t save;
    uint32_t i;

    save = save_and_disable_interrupts();
    for (i = 0; i < nw; i++) {
        uint32_t word = words[i];
        uint32_t t0;
        uint32_t d;

        t0 = cvr_now();
        can_rx_edge_inline(rx, (uint16_t)(word >> 16), 1u);
        __asm__ volatile("" ::: "memory");
        d = cvr_delta(t0, cvr_now());
        out[2u * i] = (uint16_t)(d > 0xffffu ? 0xffffu : d);

        t0 = cvr_now();
        can_rx_edge_inline(rx, (uint16_t)word, 0u);
        __asm__ volatile("" ::: "memory");
        d = cvr_delta(t0, cvr_now());
        out[2u * i + 1u] = (uint16_t)(d > 0xffffu ? 0xffffu : d);
    }
    restore_interrupts(save);
    return 2u * nw;
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
        bool ok = true;

        can_rx_reset(&rx);
        crc0 = rx.crc_errors;
        stuff0 = rx.stuff_errors;
        bit0 = rx.bit_errors;
        ov0 = rx.overflow;
        feed_words(&rx, &g_words[g_frame_off[n]], m->words);

        if (m->ack) {
            ok = can_rx_available(&rx) != 0u;
            if (ok) {
                const can_rx_frame_t *g = can_rx_front(&rx);
                uint8_t gd[8];

                can_rx_frame_data(g, gd);
                ok = can_rx_frame_id(g) == m->id &&
                     can_rx_frame_dlc(g) == m->dlc &&
                     (can_rx_frame_flags(g) & 1u) == m->ext;
                // RTR frames carry no data bits, so the raw slot stays zero.
                if (ok && !m->rtr)
                    ok = memcmp(&gd[8u - m->dlc], m->data, m->dlc) == 0;
                can_rx_pop(&rx);
            }
        } else {
            ok = !can_rx_available(&rx);
        }
        ok = ok && rx.crc_errors == crc0 && rx.stuff_errors == stuff0 &&
             rx.bit_errors == bit0 && rx.overflow == ov0;

        if (!ok) {
            Serial.printf("verify: frame %lu FAILED (ack=%u ext=%u dlc=%u "
                          "crc=%lu stuff=%lu bit=%lu ov=%lu)\n",
                          (unsigned long)n, (unsigned)m->ack,
                          (unsigned)m->ext, (unsigned)m->dlc,
                          (unsigned long)(rx.crc_errors - crc0),
                          (unsigned long)(rx.stuff_errors - stuff0),
                          (unsigned long)(rx.bit_errors - bit0),
                          (unsigned long)(rx.overflow - ov0));
            fails++;
        }
    }
    return fails;
}

static void run_timing(void) {
    can_rx_t rx;
    uint32_t round;
    uint32_t n;
    uint32_t i;

    can_rx_init(&rx, g_ticks_per_bit);

    for (n = 0; n < BENCH_FRAMES; n++)
        (void)bench_pair_reps(&rx, &g_words[g_frame_off[n]], g_frame[n].words, 1u);
    for (n = 0; n < BENCH_FRAMES; n++)
        (void)bench_edge_reps(&rx, &g_words[g_frame_off[n]], g_frame[n].words, 1u);

    for (round = 0; round < BENCH_ROUNDS; round++) {
        for (n = 0; n < BENCH_FRAMES; n++) {
            const uint32_t *words = &g_words[g_frame_off[n]];
            uint32_t nw = g_frame[n].words;

            g_pair_round[round][n] = bench_pair_reps(&rx, words, nw, BENCH_REPS);
            g_edge_round[round][n] = bench_edge_reps(&rx, words, nw, BENCH_REPS);

            g_pair_msg[round * BENCH_FRAMES + n] =
                (g_pair_round[round][n] + BENCH_REPS / 2u) / BENCH_REPS;
            g_edge_msg[round * BENCH_FRAMES + n] =
                (g_edge_round[round][n] + BENCH_REPS / 2u) / BENCH_REPS;
        }
        if ((round & 1u) != 0u)
            Serial.print('.');
    }
    Serial.println();

    i = 0;
    for (n = 0; n < BENCH_FRAMES; n++)
        i += g_frame[n].words;
    Serial.printf("timed %lu words per pass, %lu parse passes\n",
                  (unsigned long)i,
                  (unsigned long)(BENCH_ROUNDS * BENCH_REPS));
}

static void run_call_samples(void) {
    can_rx_t rx;
    uint32_t n;

    can_rx_init(&rx, g_ticks_per_bit);
    g_pair_sample_count = 0;
    g_edge_sample_count = 0;

    for (n = 0; n < BENCH_FRAMES; n++) {
        const uint32_t *words = &g_words[g_frame_off[n]];
        uint32_t nw = g_frame[n].words;

        can_rx_reset(&rx);
        g_pair_sample_count += bench_pair_calls(&rx, words, nw,
                                                &g_pair_samples[g_pair_sample_count]);

        can_rx_reset(&rx);
        g_edge_sample_count += bench_edge_calls(&rx, words, nw,
                                                &g_edge_samples[g_edge_sample_count]);
    }

    (void)bench_overhead(g_overhead_samples, BENCH_OVERHEAD_SAMPLES);
}

static void print_frame_table(void) {
    uint32_t n;

    Serial.println();
    Serial.println("Per-frame results (cycles/frame are mean over rounds)");
    Serial.println("  #  id         ext dlc rtr ack  bits stf unst  words"
                   "   pair/frm   edge/frm  pair/wrd  edge/edge  cyc/wirebit");
    for (n = 0; n < BENCH_FRAMES; n++) {
        struct frame_meta *m = &g_frame[n];
        double pair = 0.0;
        double edge = 0.0;
        double cpw;
        double cpe;
        double cpb;
        uint32_t r;

        for (r = 0; r < BENCH_ROUNDS; r++) {
            pair += (double)g_pair_round[r][n] / (double)BENCH_REPS;
            edge += (double)g_edge_round[r][n] / (double)BENCH_REPS;
        }
        pair /= (double)BENCH_ROUNDS;
        edge /= (double)BENCH_ROUNDS;
        cpw = m->words ? pair / (double)m->words : 0.0;
        cpe = m->words ? edge / (double)(2u * m->words) : 0.0;
        cpb = m->total_bits ? pair / (double)m->total_bits : 0.0;

        {
            char ps[16], es[16], pws[16], pes[16], pbs[16];

            format_f2(pair, ps, sizeof(ps));
            format_f2(edge, es, sizeof(es));
            format_f2(cpw, pws, sizeof(pws));
            format_f2(cpe, pes, sizeof(pes));
            format_f2(cpb, pbs, sizeof(pbs));
            Serial.printf("%3lu 0x%08lx  %u  %u   %u   %u  %4lu %3lu %4lu %5u"
                          " %10s %10s %9s %10s %11s\n",
                          (unsigned long)n, (unsigned long)m->id,
                          (unsigned)m->ext, (unsigned)m->dlc,
                          (unsigned)m->rtr, (unsigned)m->ack,
                          (unsigned long)m->total_bits,
                          (unsigned long)m->stuffed_bits,
                          (unsigned long)m->unstuffed_bits,
                          (unsigned)m->words, ps, es, pws, pes, pbs);
        }
    }
}

static void run_bench(void) {
    uint32_t hz = clock_get_hz(clk_sys);
    uint32_t ticks = hz / 2000000u;
    uint32_t fails;
    struct stats_t pair_msg_stats;
    struct stats_t edge_msg_stats;
    struct stats_t pair_call_stats;
    struct stats_t edge_call_stats;
    struct stats_t pair_corr_stats;
    struct stats_t edge_corr_stats;
    struct stats_t overhead_stats;
    struct hist_t pair_hist;
    struct hist_t edge_hist;
    uint64_t pair_total = 0;
    uint64_t edge_total = 0;
    uint64_t word_total = 0;
    uint64_t edge_count = 0;
    uint64_t wire_bits = 0;
    uint64_t stuffed_bits = 0;
    uint64_t unstuffed_bits = 0;
    uint32_t r;
    uint32_t n;
    uint32_t i;
    double pair_cpf;
    double edge_cpf;
    double pair_cpw;
    double edge_cpe;
    double pair_cpb;
    double edge_cpb;
    double load_avg;
    double load_worst = 0.0;
    uint32_t worst_frame = 0;

    if (ticks < 8u)
        ticks = 8u;
    if (ticks > 200u)
        ticks = 200u;
    g_ticks_per_bit = ticks;

    Serial.println();
    Serial.println("============================================================");
    Serial.println(" CAN2040-Lite RX parser cycle benchmark (software only)");
    Serial.println("============================================================");
    Serial.printf("clock=%lu Hz  ticks_per_bit=%lu  ack_support=%u\n",
                  (unsigned long)hz, (unsigned long)ticks,
                  (unsigned)CAN_ACK_GENERATION_SUPPORT);
    Serial.printf("frames=%lu  rounds=%lu  reps=%lu\n",
                  (unsigned long)BENCH_FRAMES,
                  (unsigned long)BENCH_ROUNDS, (unsigned long)BENCH_REPS);

    build_corpus(ticks);
    Serial.printf("corpus: %lu FIFO pair words from %lu frames\n",
                  (unsigned long)g_word_count, (unsigned long)BENCH_FRAMES);

    fails = verify_corpus();
    Serial.printf("verify: %lu failures\n", (unsigned long)fails);

    Serial.print("timing ");
    run_timing();

    for (r = 0; r < BENCH_ROUNDS; r++) {
        for (n = 0; n < BENCH_FRAMES; n++) {
            pair_total += g_pair_round[r][n];
            edge_total += g_edge_round[r][n];
        }
    }
    for (n = 0; n < BENCH_FRAMES; n++) {
        word_total += (uint64_t)g_frame[n].words * BENCH_ROUNDS * BENCH_REPS;
        edge_count += (uint64_t)g_frame[n].words * 2u * BENCH_ROUNDS * BENCH_REPS;
        wire_bits += (uint64_t)g_frame[n].total_bits * BENCH_ROUNDS * BENCH_REPS;
        stuffed_bits += (uint64_t)g_frame[n].stuffed_bits * BENCH_ROUNDS * BENCH_REPS;
        unstuffed_bits += (uint64_t)g_frame[n].unstuffed_bits * BENCH_ROUNDS * BENCH_REPS;
    }
    pair_cpf = (double)pair_total /
               ((double)BENCH_ROUNDS * BENCH_FRAMES * BENCH_REPS);
    edge_cpf = (double)edge_total /
               ((double)BENCH_ROUNDS * BENCH_FRAMES * BENCH_REPS);
    pair_cpw = (double)pair_total / (double)word_total;
    edge_cpe = (double)edge_total / (double)edge_count;
    pair_cpb = (double)pair_total / (double)wire_bits;
    edge_cpb = (double)edge_total / (double)wire_bits;
    load_avg = pair_cpb / (hz / 1000000.0);

    for (n = 0; n < BENCH_FRAMES; n++) {
        double cpf = 0.0;
        double cpb;
        uint32_t rr;

        for (rr = 0; rr < BENCH_ROUNDS; rr++)
            cpf += (double)g_pair_round[rr][n] / (double)BENCH_REPS;
        cpf /= (double)BENCH_ROUNDS;
        cpb = g_frame[n].total_bits ? cpf / (double)g_frame[n].total_bits : 0.0;
        if (cpb > load_worst) {
            load_worst = cpb;
            worst_frame = n;
        }
    }

    run_call_samples();

    stats_u32(g_pair_msg, BENCH_ROUNDS * BENCH_FRAMES, &pair_msg_stats);
    stats_u32(g_edge_msg, BENCH_ROUNDS * BENCH_FRAMES, &edge_msg_stats);
    stats_u16(g_pair_samples, g_pair_sample_count, &pair_call_stats);
    hist_build(g_pair_samples, g_pair_sample_count, &pair_hist);
    stats_u16(g_edge_samples, g_edge_sample_count, &edge_call_stats);
    hist_build(g_edge_samples, g_edge_sample_count, &edge_hist);
    stats_u16(g_overhead_samples, BENCH_OVERHEAD_SAMPLES, &overhead_stats);
    {
        uint32_t med = (uint32_t)(overhead_stats.p50 + 0.5);

        for (i = 0; i < g_pair_sample_count; i++)
            g_pair_samples[i] = (uint16_t)(g_pair_samples[i] > med ?
                                           g_pair_samples[i] - med : 0u);
        for (i = 0; i < g_edge_sample_count; i++)
            g_edge_samples[i] = (uint16_t)(g_edge_samples[i] > med ?
                                           g_edge_samples[i] - med : 0u);
    }
    stats_u16(g_pair_samples, g_pair_sample_count, &pair_corr_stats);
    stats_u16(g_edge_samples, g_edge_sample_count, &edge_corr_stats);

    Serial.println();
    Serial.println("-- Corpus ----------------------------------------------------");
    Serial.printf("  frames=%lu  pair words=%lu  edges=%lu\n",
                  (unsigned long)BENCH_FRAMES,
                  (unsigned long)g_word_count,
                  (unsigned long)(2u * g_word_count));
    Serial.printf("  wire bits=%lu  stuffed=%lu  unstuffed=%lu\n",
                  (unsigned long)(wire_bits / (BENCH_ROUNDS * BENCH_REPS)),
                  (unsigned long)(stuffed_bits / (BENCH_ROUNDS * BENCH_REPS)),
                  (unsigned long)(unstuffed_bits /
                                  (BENCH_ROUNDS * BENCH_REPS)));
    {
        char tpf[16];
        char wpf[16];

        format_f2((double)(2u * g_word_count) / (double)BENCH_FRAMES,
                  tpf, sizeof(tpf));
        format_f2((double)(wire_bits / (BENCH_ROUNDS * BENCH_REPS)) /
                  (double)BENCH_FRAMES, wpf, sizeof(wpf));
        Serial.printf("  transitions per frame=%s  wire bits per frame=%s\n",
                      tpf, wpf);
    }

    Serial.println();
    Serial.println("-- Cost summary (authoritative, whole-frame timing) ----------");
    {
        char a[16], b[16], c[16], d[16], e[16], f[16], g[16], h[16];
        char i[16], j[16], k[16];
        double pair_cstuffed = stuffed_bits ?
            (double)pair_total / (double)stuffed_bits : 0.0;
        double edge_cstuffed = stuffed_bits ?
            (double)edge_total / (double)stuffed_bits : 0.0;

        format_f2(pair_cpf, a, sizeof(a));
        format_f2(edge_cpf, b, sizeof(b));
        format_f2(pair_cpw, c, sizeof(c));
        format_f2(edge_cpe, d, sizeof(d));
        format_f2(pair_cpb, e, sizeof(e));
        format_f2(edge_cpb, f, sizeof(f));
        format_f2(load_avg * 100.0, g, sizeof(g));
        format_f2(load_worst / (hz / 1000000.0) * 100.0, h, sizeof(h));
        format_f2(pair_cstuffed, i, sizeof(i));
        format_f2(edge_cstuffed, j, sizeof(j));
        format_f2(load_worst, k, sizeof(k));
        Serial.printf("  %-27s %12s %12s\n", "metric", "pair_inline", "edge_inline");
        Serial.printf("  %-27s %12s %12s\n", "cycles / frame", a, b);
        Serial.printf("  %-27s %12s %12s\n", "cycles / word (pair)",
                      c, "-");
        Serial.printf("  %-27s %12s %12s\n", "cycles / edge", "-", d);
        Serial.printf("  %-27s %12s %12s\n", "cycles / wire bit", e, f);
        Serial.printf("  %-27s %12s %12s\n", "cycles / stuffed bit", i, j);
        Serial.printf("  %-27s %12s %12s\n", "avg CPU load @ 1 Mbit (%)", g, "-");
        Serial.printf("  %-27s %12s %12s\n", "worst-frame load @ 1 Mbit (%)",
                      h, "-");
        Serial.printf("  worst frame: #%lu (id=0x%08lX, %lu bits, %s cyc/bit)\n",
                      (unsigned long)worst_frame,
                      (unsigned long)g_frame[worst_frame].id,
                      (unsigned long)g_frame[worst_frame].total_bits,
                      k);
    }

    Serial.println();
    Serial.println("-- Per-message distribution (cycles/frame) -------------------");
    print_stats("pair_inline", &pair_msg_stats);
    print_stats("edge_inline", &edge_msg_stats);
    {
        char delta[16];
        char pct[16];

        format_f2(edge_cpf - pair_cpf, delta, sizeof(delta));
        format_f2(pair_cpf != 0.0 ?
                  (edge_cpf - pair_cpf) / pair_cpf * 100.0 : 0.0,
                  pct, sizeof(pct));
        Serial.printf("  pair-vs-edge delta: %s cycles/frame (%s%%)\n",
                      delta, pct);
    }

    Serial.println();
    Serial.println("-- Per-call distribution (raw, includes timer reads) ---------");
    print_stats("pair_inline", &pair_call_stats);
    print_stats("edge_inline", &edge_call_stats);
    print_stats("timer overhead", &overhead_stats);

    Serial.println();
    Serial.println("-- Per-call distribution (overhead-corrected) ----------------");
    print_stats("pair_inline", &pair_corr_stats);
    print_stats("edge_inline", &edge_corr_stats);

    Serial.println();
    Serial.println("-- Pair-call histogram (raw cycles) --------------------------");
    hist_print("pair_inline", &pair_hist);
    Serial.println("-- Edge-call histogram (raw cycles) --------------------------");
    hist_print("edge_inline", &edge_hist);

    print_frame_table();

    Serial.println();
    Serial.printf("verify: %lu failures, corpus decode %s\n",
                  (unsigned long)fails, fails ? "FAIL" : "PASS");
    Serial.println("=== RX BENCH DONE ===");
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
