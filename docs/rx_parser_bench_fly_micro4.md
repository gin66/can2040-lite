# RX parser cycle benchmark — Fly-Micro4 (RP2040)

Software-only measurement of the RX parser. No PIO, no PIO backend, no
transceiver and no CAN controller are involved: the harnesses build CAN
frames, turn them into the exact unstuffed bit or RX FIFO edge-pair stream
the receiver would see, and feed them to the portable parser.

This report describes the current `main` parser (pre-ACK-rework). The faster
ACK-rework measurements are preserved on `wip/ack-generation`; see §5.

| | |
|---|---|
| **Status** | Measured on target, corpora verified |
| **Date** | 2026-11-14 |
| **Hardware** | Mellow Fly-Micro4 V1.0, RP2040, 133 MHz |
| **Build** | Earle Philhower core, `-O2`, `CAN_ACK_GENERATION_SUPPORT=0` |
| **Counter** | Cortex-M0+ SysTick (`systick_hw->cvr`), 24-bit, processor clock |
| **Harness 1** | `extras/tests/on_target/rx_bench.cpp` — FIFO words / edges |
| **Harness 2** | `extras/tests/on_target/rx_bit_bench.cpp` — per unstuffed bit |
| **Result** | 0 decode failures in both runs |

The board settings match `../suspended-spheres/rp2040/platformio.ini`. The
Fly-Micro4 CAN pins (GPIO4/5) are not used. Timed functions are placed in SRAM
(`__no_inline_not_in_flash_func`), matching the production RX IRQ.

## 1. Repository layout

The repository is both a library and its own PlatformIO project:

- `library.json` + `src/` are the library. Host tests are built by the
  top-level `Makefile`.
- `platformio.ini` is only the on-target harness project. PlatformIO requires
  the project file at the project root, and `src_dir = .` makes the repository
  root the source root. Library consumers use `library.json`, not
  `platformio.ini`.
- `extras/tests/on_target/main.cpp` is the Arduino `setup()`/`loop()` entry
  point for the protocol/PIO-loopback harness. A PlatformIO project needs one
  entry point, so each benchmark has its own source and environment.

## 2. Measurement setup

Both harnesses use the same clock and build:

- 133 MHz, nominal `ticks_per_bit = 66` at 1 Mbit/s;
- `-O2`, `CAN_ACK_GENERATION_SUPPORT=0`;
- SysTick cycle counter, interrupts disabled around each timed section;
- 100 generated frames per run, deterministic corpus;
- every frame is verified before timing.

The first harness measures whole FIFO words (`can_rx_pair_inline()`) and
single edges (`can_rx_edge_inline()`). The second removes stuffing and times
`can_rx_advance()` once per unstuffed wire bit.

## 3. Benchmark 1 — FIFO words and edges

### 3.1 Corpus

100 frames: standard and extended IDs, DLC 0..8, RTR DLC 0, ACK/NACK, and
payload patterns that exercise different stuffing densities.

| | |
|---|---:|
| frames | 100 |
| FIFO pair words | 2012 |
| edges | 4024 |
| wire bits | 8356 |
| stuffed bits | 7056 |
| unstuffed bits | 6680 |
| transitions / frame | 40.24 |
| wire bits / frame | 83.56 |

Verification: **0 failures**.

### 3.2 Cost summary

Whole-frame timing, 8 rounds × 8 parses per frame:

| Metric | `pair_inline` | `edge_inline` |
|---|---:|---:|
| cycles / frame | 7724.36 | 7706.07 |
| cycles / word | 383.91 | — |
| cycles / edge | — | 191.50 |
| cycles / wire bit | 92.44 | 92.22 |
| cycles / stuffed bit | 109.47 | 109.21 |
| average CPU load @ 1 Mbit | **69.50%** | — |
| worst-frame load @ 1 Mbit | **83.22%** | — |

Worst frame: `#97`, id `0x6B0`, DLC 7, 104 wire bits, 39 pair words,
`110.68` cycles/wire bit.

On current `main`, `edge_inline` is about **18.3 cycles/frame faster** than
`pair_inline` (7706.07 vs 7724.36). The ACK-rework branch made the two paths
identical.

### 3.3 Per-message distribution

Cycles per frame over all frames and rounds (n = 800):

| | `pair_inline` | `edge_inline` |
|---|---:|---:|
| min | 3761 | 3752 |
| p05 | 3885 | 3876 |
| p50 | 7857 | 7838 |
| mean | 7724.61 | 7706.32 |
| p95 | 11423 | 11385 |
| p99 | 12666 | 12627 |
| max | 13213 | 13187 |
| standard deviation | 2633.94 | 2627.81 |

### 3.4 Per-call distribution

Raw values include the SysTick reads. The measured timer overhead is
**2 cycles**, so the corrected values remove it.

| | n | min | p05 | p50 | mean | p95 | p99 | max | sd |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `pair_inline` raw | 2012 | 102 | 195 | 353 | 388.97 | 649 | 810 | 912 | 150.76 |
| `pair_inline` corrected | 2012 | 100 | 193 | 351 | 386.97 | 647 | 808 | 910 | 150.76 |
| `edge_inline` raw | 4024 | 19 | 68 | 143 | 198.24 | 485 | 495 | 507 | 117.09 |
| `edge_inline` corrected | 4024 | 17 | 66 | 141 | 196.24 | 483 | 493 | 505 | 117.09 |

The per-call pair mean is slightly above `cycles / word` (386.97 vs 383.91)
because the per-call pass also performs the sample store and loop bookkeeping.

### 3.5 Selected frames

Slowest by cycles per wire bit:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 97 | `0x000006B0` | 0 | 7 | 104 | 39 | 11510.75 | 110.68 |
| 43 | `0x000004DD` | 0 | 7 | 103 | 38 | 11394.75 | 110.63 |
| 79 | `0x00000293` | 0 | 7 | 103 | 38 | 11375.75 | 110.44 |
| 61 | `0x00000678` | 0 | 7 | 104 | 38 | 11422.75 | 109.83 |
| 96 | `0x000000C4` | 0 | 6 | 96 | 35 | 10416.75 | 108.51 |

Slowest by cycles per frame:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 35 | `0x09D754C0` | 1 | 8 | 140 | 30 | 13212.75 | 94.38 |
| 60 | `0x1662FD90` | 1 | 6 | 117 | 40 | 12665.75 | 108.25 |
| 70 | `0x0B1EEE85` | 1 | 7 | 126 | 31 | 12535.75 | 99.49 |
| 97 | `0x000006B0` | 0 | 7 | 104 | 39 | 11510.75 | 110.68 |
| 7 | `0x0000065F` | 0 | 7 | 106 | 38 | 11483.75 | 108.34 |

Fastest by cycles per frame:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 9 | `0x000000DE` | 0 | 0 | 48 | 9 | 3760.75 | 78.35 |
| 72 | `0x000000A6` | 0 | 0 | 48 | 9 | 3771.75 | 78.58 |
| 69 | `0x00000110` | 0 | 0 | 48 | 9 | 3777.75 | 78.70 |
| 91 | `0x00000430` | 0 | 0 | 48 | 9 | 3792.75 | 79.02 |
| 54 | `0x0000021D` | 0 | 0 | 49 | 9 | 3803.75 | 77.63 |

The firmware prints all 100 frames.

## 4. Benchmark 2 — per-bit `can_rx_advance()`

### 4.1 Method

The harness generates 100 standard 11-bit, DLC-8 frames with varied IDs and
payloads. Each frame is built and stuffed, then unstuffed again. The timed
bit stream is:

- 98 unstuffed bits from SOF through the CRC sequence;
- CRC delimiter;
- ACK slot (dominant, so the frame is published).

That is exactly 100 calls to `can_rx_advance()` per frame. Per-bit cost is
measured by prefix differencing:

```text
cost(bit n) = T(n + 1) - T(n)
```

where `T(n)` is the time to parse the first `n` bits. This gives the marginal
cost of one call in the real parse loop, without placing a timer read inside
the loop. The whole-frame batch timing is the authoritative cross-check; its
average (84.64 cycles/bit) equals the weighted average of the per-bit table.
The isolated per-call distribution is reported separately.

### 4.2 Corpus and overall result

| | |
|---|---:|
| frames | 100 |
| unstuffed bits / frame | 100 |
| wire bits / frame | 117.05 |
| `can_rx_advance` calls / pass | 10000 |
| cycles / frame | 8464.16 |
| cycles / unstuffed bit | 84.64 |
| cycles / wire bit (approx) | 72.31 |
| parser load @ 1 Mbit | 54.37% |

Verification: **0 failures**.

The wire-bit figure is approximate for this harness: it divides the per-bit
state-machine cost by the stuffed wire-bit count and excludes edge/run
classification. Use benchmark 1 for the full RX load.

Per-call isolated distribution:

| | n | min | p05 | p50 | mean | p95 | p99 | max | sd |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| raw | 10000 | 40 | 65 | 83 | 80.60 | 85 | 85 | 100 | 8.23 |
| corrected | 10000 | 38 | 63 | 81 | 78.60 | 83 | 83 | 98 | 8.23 |
| timer overhead | 4096 | 2 | 2 | 2 | 2.00 | 2 | 2 | 2 | 0.00 |

The isolated call excludes the loop step, so it is a few cycles below the
marginal per-bit cost below.

### 4.3 Cost by field

Prefix-differenced marginal cost:

| Field | n | min | p50 | mean | p95 | max |
|---|---:|---:|---:|---:|---:|---:|
| SOF | 100 | 53 | 53 | 53.00 | 53 | 53 |
| Base ID | 1100 | 86 | 86 | 88.94 | 92 | 92 |
| Control | 300 | 76 | 86 | 85.41 | 92 | 92 |
| DLC | 400 | 82 | 88 | 88.16 | 92 | 92 |
| Data | 6400 | 86 | 86 | 88.98 | 92 | 92 |
| CRC | 1500 | 64 | 64 | 64.20 | 67 | 67 |
| Tail | 200 | 68 | 100 | 84.00 | 100 | 100 |

### 4.4 Per-bit table

Standard 11-bit, DLC-8 frame. Values are cycles; `range` is min–max over 100
frames. The first call (SOF) includes the inner-loop setup for the frame.

```text
bit  name        mean    range
  0  SOF         53.00   53
  1  ID10        88.40   86-92
  2  ID9         89.12   86-92
  3  ID8         88.82   86-92
  4  ID7         89.66   86-92
  5  ID6         88.94   86-92
  6  ID5         89.06   86-92
  7  ID4         88.64   86-92
  8  ID3         89.12   86-92
  9  ID2         88.82   86-92
 10  ID1         89.00   86-92
 11  ID0         88.76   86-92
 12  RTR         88.64   86-92
 13  IDE         78.52   76-82
 14  R0          89.06   86-92
 15  DLC3        88.82   86-92
 16  DLC2        89.36   86-92
 17  DLC1        89.06   86-92
 18  DLC0        85.42   82-88
 19  D0          89.06   86-92
 20  D1          89.30   86-92
 21  D2          88.88   86-92
 22  D3          88.94   86-92
 23  D4          89.00   86-92
 24  D5          89.36   86-92
 25  D6          89.18   86-92
 26  D7          88.64   86-92
 27  D8          89.24   86-92
 28  D9          88.52   86-92
 29  D10         89.36   86-92
 30  D11         89.12   86-92
 31  D12         88.88   86-92
 32  D13         88.76   86-92
 33  D14         89.30   86-92
 34  D15         89.18   86-92
 35  D16         88.64   86-92
 36  D17         88.82   86-92
 37  D18         88.64   86-92
 38  D19         89.24   86-92
 39  D20         89.06   86-92
 40  D21         89.00   86-92
 41  D22         89.18   86-92
 42  D23         89.06   86-92
 43  D24         89.24   86-92
 44  D25         88.70   86-92
 45  D26         88.46   86-92
 46  D27         88.94   86-92
 47  D28         89.30   86-92
 48  D29         88.88   86-92
 49  D30         88.58   86-92
 50  D31         88.88   86-92
 51  D32         89.12   86-92
 52  D33         88.94   86-92
 53  D34         89.06   86-92
 54  D35         89.06   86-92
 55  D36         89.66   86-92
 56  D37         89.24   86-92
 57  D38         89.24   86-92
 58  D39         88.64   86-92
 59  D40         89.06   86-92
 60  D41         88.64   86-92
 61  D42         89.78   86-92
 62  D43         88.70   86-92
 63  D44         89.30   86-92
 64  D45         88.70   86-92
 65  D46         88.88   86-92
 66  D47         89.00   86-92
 67  D48         89.12   86-92
 68  D49         89.18   86-92
 69  D50         89.06   86-92
 70  D51         88.40   86-92
 71  D52         88.64   86-92
 72  D53         88.58   86-92
 73  D54         88.64   86-92
 74  D55         88.94   86-92
 75  D56         89.00   86-92
 76  D57         88.76   86-92
 77  D58         89.18   86-92
 78  D59         88.52   86-92
 79  D60         88.70   86-92
 80  D61         89.30   86-92
 81  D62         88.76   86-92
 82  D63         89.30   86-92
 83  CRC14       64.00   64
 84  CRC13       64.00   64
 85  CRC12       64.00   64
 86  CRC11       64.00   64
 87  CRC10       64.00   64
 88  CRC9        64.00   64
 89  CRC8        64.00   64
 90  CRC7 cmp    64.00   64
 91  CRC6        64.00   64
 92  CRC5        64.00   64
 93  CRC4        64.00   64
 94  CRC3        64.00   64
 95  CRC2        64.00   64
 96  CRC1        64.00   64
 97  CRC0 cmp    67.00   67
 98  CRC-DELIM   68.00   68
 99  ACK-SLOT   100.00   100
```

### 4.5 Key observations

- Ordinary ID and data bits cost about **89 cycles** each; all ID and data
  positions stay within 86–92 cycles across the 100-frame corpus.
- IDE is cheaper (≈79 cycles) because the standard-frame parser jumps over the
  extended-ID region at that point.
- DLC0 is cheaper than the first three DLC bits (≈85 vs ≈89) and is where the
  parser computes `crc_start`.
- CRC data bits are the cheapest normal bits (≈64 cycles) because the CRC
  update is skipped once the CRC field has started.
- The CRC0 compare bit is 67 cycles; the CRC delimiter is 68.
- The ACK slot is the most expensive single bit at **100 cycles** because it
  publishes the frame.
- The per-bit table is the marginal cost in the real loop. The isolated call
  median is 83 raw / 81 corrected cycles; the difference is the loop step.

## 5. Interpretation

- Benchmark 1: the full FIFO/edge parser consumes **69.50% of one core on
  average** at 1 Mbit/s and **83.22% on the worst frame**. That is consistent
  with `docs/rx_bus_load.md`, where the full RX ISR measured about 70.6% of a
  core at roughly 90% bus load; the parser is the dominant part and IRQ/FIFO
  handling adds the rest.
- Benchmark 2: `can_rx_advance()` alone costs **84.64 cycles per unstuffed
  bit**, or 72.31 cycles per wire bit on the standard DLC-8 corpus (54.37% of
  the 133-cycle bit period). This excludes edge/run classification, so it is
  not the full ISR load.
- Per-frame cost scales with frame length and stuffing density. The slowest
  per-wire-bit frames are standard DLC-7 frames at 110.68 cycles/wire bit;
  the cheapest are short DLC-0 frames at about 78 cycles/wire bit.
- The ACK-rework branch measured **6573.69 cycles/frame**, **59.15% average
  load** and **71.60% worst-frame load** for benchmark 1. That is about 15%
  fewer cycles per frame than `main`; equivalently, `main` is about 17% slower.
  The rework also made the pair and edge totals identical. The `main` numbers
  above are the production parser before that rework.

## 6. Caveats

- Both benchmarks compile with `CAN_ACK_GENERATION_SUPPORT=0`. The Fly
  production image defaults to ACK generation enabled; the ACK tail/latch path
  is not included in these numbers.
- No PIO capture jitter, transceiver slew or IRQ entry is included. Timestamps
  are ideal nominal 66-tick spacings. The numbers isolate parser work.
- RTR vectors in benchmark 1 use DLC 0 because the current parser has no RTR
  data-suppression state.
- Benchmark 2 feeds the CRC delimiter and ACK slot explicitly. On a real NACK
  tail with no edges, the quiet-tail resynchronisation happens at the next SOF
  instead; the per-bit costs of those two tail bits are therefore the
  edge-present case.
- The per-bit table uses prefix differencing, not isolated timer reads. This
  includes the loop step and matches the whole-frame average. The isolated
  per-call distribution is reported separately for call-level shape.

## 7. Reproduce

```sh
cd can2040-lite

# FIFO word / edge benchmark
pio run -e fly_micro4_rx_bench -t upload
pio device monitor -e fly_micro4_rx_bench

# per-bit can_rx_advance benchmark
pio run -e fly_micro4_rx_bit_bench -t upload
pio device monitor -e fly_micro4_rx_bit_bench
```

Press Enter or `r` in either monitor to rerun. Host tests:

```sh
make test          # 177 passed, 0 failed
```

## Appendix — benchmark 1 histograms (`main`)

Pair-call raw cycles, bucket width 28:

```text
   84..  111     39
  168..  195     63
  196..  223     11
  224..  251     86
  252..  279    661
  280..  307     26
  308..  335     84
  336..  363    125
  364..  391    112
  392..  419     58
  420..  447     43
  448..  475    135
  476..  503     53
  504..  531     28
  532..  559    244
  560..  587     43
  588..  615     41
  616..  643     51
  644..  671     11
  672..  699     18
  700..  727     35
  728..  755     15
  756..  783      1
  784..  811     10
  812..  839      5
  840..  867      1
  868..  895      9
  896..  923      4
```

Edge-call raw cycles, bucket width 16:

```text
   16..   31    100
   64..   79    288
   96..  111     39
  128..  143   1589
  144..  159    551
  160..  175      4
  192..  207     93
  208..  223    201
  224..  239    294
  240..  255      7
  272..  287     82
  288..  303     32
  304..  319    143
  320..  335     48
  352..  367     58
  368..  383     30
  384..  399     52
  400..  415     42
  416..  431      6
  432..  447     39
  448..  463     14
  464..  479     70
  480..  495    206
  496..  511     36
```
