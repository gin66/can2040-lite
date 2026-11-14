# RX parser cycle benchmark — Fly-Micro4 (RP2040)

Software-only measurement of `can_rx_pair_inline()` and
`can_rx_edge_inline()`. No PIO, no PIO backend, no transceiver and no CAN
controller are involved: the test builds CAN frames, turns them into the exact
RX FIFO edge-pair words the receiver would see, and feeds them to the portable
parser.

| | |
|---|---|
| **Status** | Measured, corpus verified |
| **Date** | 2026-11-14 |
| **Hardware** | Mellow Fly-Micro4 V1.0, RP2040, 133 MHz |
| **Build** | `env:fly_micro4_rx_bench`, Earle Philhower core, `-O2`, `CAN_ACK_GENERATION_SUPPORT=0` |
| **Counter** | Cortex-M0+ SysTick (`systick_hw->cvr`), 24-bit, processor clock |
| **Harness** | `extras/tests/on_target/rx_bench.cpp` |
| **Result** | 0 decode failures; see §4 |

The Fly-Micro4 CAN pins (GPIO4/5) are not used. The board settings match
`../suspended-spheres/rp2040/platformio.ini` so the numbers describe the
production RP2040 image.

## 1. Repository layout

The repository is both a library and its own PlatformIO project:

- `library.json` + `src/` are the library. Host tests are built by the
  top-level `Makefile`.
- `platformio.ini` is only the on-target harness project. PlatformIO requires
  the project file at the project root, and `src_dir = .` makes the repository
  root the source root. Library consumers use `library.json`, not
  `platformio.ini`.
- `extras/tests/on_target/main.cpp` is the Arduino `setup()`/`loop()` entry
  point for the existing protocol/PIO-loopback harness. A PlatformIO project
  needs one entry point, so the benchmark has its own `rx_bench.cpp` and its
  own environment. The existing environment selects only `main.cpp` in
  `build_src_filter` to avoid two entry points.

## 2. Method

### 2.1 Corpus

`rx_bench.cpp` generates 100 deterministic frames:

- standard and extended identifiers, including low and high IDs;
- DLC 0..8;
- RTR vectors with DLC 0 (the form supported by the parser; the parser has no
  RTR data-suppression state);
- ACK and NACK frames;
- payloads that produce different stuffing patterns: `0xAA`, `0x55`, `0xFF`,
  `0x00`, a mixed pattern, and pseudo-random bytes.

Each frame is built with the same `can_frame_build()` used by the host tests,
loaded with a correct CRC and the normal 13-bit tail. Only transitions are
kept: each `word = (fall_timestamp16 << 16) | rise_timestamp16`, with nominal
`ticks_per_bit` derived from the clock (`133 MHz / (2 × 1 Mbit/s) = 66`).
This is the same word format the PIO RX FIFO produces.

The run is accepted only after every ACK frame decodes to the expected ID,
DLC, IDE flag and data (RTR is checked without data) and no CRC, stuff, bit or
ring-overflow counter changes. Final run: **0 failures**.

### 2.2 Timing

There are two passes:

1. **Whole-frame timing (authoritative).** For each frame, the parser is reset
   and the frame is parsed `BENCH_REPS = 8` times inside one SysTick bracket.
   The timer overhead is therefore divided by 8 parses. This is repeated
   `BENCH_ROUNDS = 8` times. Each timed function lives in SRAM
   (`__no_inline_not_in_flash_func`), matching the production RX IRQ.
2. **Per-call timing.** Each call to `can_rx_pair_inline()` (one FIFO word) and
   each call to `can_rx_edge_inline()` (one edge) is bracketed with a SysTick
   read. Interrupts are disabled around each frame's pass. The two-read timer
   overhead is measured with the same loop and reported separately. The
   overhead-corrected numbers subtract the measured median overhead.

All values are CPU cycles. At 133 MHz one CAN bit at 1 Mbit/s is 133 cycles.

## 3. Corpus size

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

## 4. Results

### 4.1 Cost summary

Whole-frame timing, 8 rounds × 8 parses per frame:

| Metric | `pair_inline` | `edge_inline` |
|---|---:|---:|
| cycles / frame | 6573.69 | 6573.69 |
| cycles / word | 326.72 | — |
| cycles / edge | — | 163.36 |
| cycles / wire bit | 78.67 | 78.67 |
| cycles / stuffed bit | 93.16 | 93.16 |
| average CPU load @ 1 Mbit | **59.15%** | — |
| worst-frame load @ 1 Mbit | **71.60%** | — |

Worst frame: `#43`, id `0x4DD`, DLC 7, 103 wire bits, 38 pair words,
`95.22` cycles/wire bit.

`pair_inline` and `edge_inline` produce identical per-frame totals (0.00
cycles/frame difference): a pair is two edge calls plus the two shifts.

### 4.2 Per-message distribution

Cycles per frame over all frames and rounds (n = 800):

| | pair / edge |
|---|---:|
| min | 3247 |
| p05 | 3375 |
| p50 | 6664 |
| mean | 6573.60 |
| p95 | 9818 |
| p99 | 10840 |
| max | 11051 |
| standard deviation | 2207.05 |

### 4.3 Per-call distribution

Raw values include the SysTick reads. The measured timer overhead is
**2 cycles** (min = p05 = p50 = p95 = max), so the corrected values are the
parser cost with the timer removed.

| | n | min | p05 | p50 | mean | p95 | p99 | max | sd |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `pair_inline` raw | 2012 | 94 | 179 | 303 | 332.88 | 564 | 651 | 744 | 120.81 |
| `pair_inline` corrected | 2012 | 92 | 177 | 301 | 330.88 | 562 | 649 | 742 | 120.81 |
| `edge_inline` raw | 4024 | 19 | 59 | 122 | 166.31 | 386 | 412 | 425 | 95.35 |
| `edge_inline` corrected | 4024 | 17 | 57 | 120 | 164.31 | 384 | 410 | 423 | 95.35 |

The per-call pair mean is slightly above `cycles / word` (330.88 vs 326.72)
because the per-call pass also performs the sample store and loop bookkeeping.

### 4.4 Pair-call histogram

Raw cycles, bucket width 24:

```text
   72..   95     39
  144..  167     35
  168..  191     39
  216..  239    504
  240..  263    265
  264..  287      4
  288..  311    256
  312..  335     63
  336..  359      5
  360..  383    180
  384..  407     50
  408..  431     33
  432..  455    157
  456..  479    144
  480..  503     65
  504..  527     53
  528..  551      9
  552..  575     36
  576..  599     32
  600..  623      9
  624..  647      9
  648..  671      6
  672..  695      3
  696..  719      7
  720..  743      8
  744..  767      1
```

The fast cluster at 216–239 cycles is the common stuffed-data path. The
360–383 cluster and the longer tail are field boundaries (DLC, CRC compare,
tail) and error-free multi-bit runs.

### 4.5 Edge-call histogram

Raw cycles, bucket width 16:

```text
   16..   31    100
   48..   63    145
   64..   79    143
   80..   95     39
  112..  127   1992
  128..  143    137
  144..  159     50
  176..  191    396
  192..  207    156
  208..  223     17
  224..  239      2
  240..  255    155
  256..  271    126
  272..  287     22
  288..  303      4
  304..  319     83
  320..  335     61
  336..  351     32
  352..  367      1
  368..  383    151
  384..  399     55
  400..  415    147
  416..  431     10
```

The 112–127 cycle bucket holds 1992 of 4024 edges: the ordinary
stuffing/CRC/data path. The 176–191 bucket is the next common field-boundary
path.

### 4.6 Selected frames

Slowest by cycles per wire bit:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 43 | `0x000004DD` | 0 | 7 | 103 | 38 | 9808.13 | 95.22 |
| 97 | `0x000006B0` | 0 | 7 | 104 | 39 | 9903.13 | 95.22 |
| 79 | `0x00000293` | 0 | 7 | 103 | 38 | 9788.13 | 95.03 |
| 61 | `0x00000678` | 0 | 7 | 104 | 38 | 9818.13 | 94.41 |
| 42 | `0x000006ED` | 0 | 6 | 96 | 35 | 8979.13 | 93.53 |

Slowest by cycles per frame:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 35 | `0x09D754C0` | 1 | 8 | 140 | 30 | 11051.13 | 78.94 |
| 60 | `0x1662FD90` | 1 | 6 | 117 | 40 | 10840.13 | 92.65 |
| 70 | `0x0B1EEE85` | 1 | 7 | 126 | 31 | 10560.13 | 83.81 |
| 97 | `0x000006B0` | 0 | 7 | 104 | 39 | 9903.13 | 95.22 |
| 7 | `0x0000065F` | 0 | 7 | 106 | 38 | 9897.13 | 93.37 |

Fastest by cycles per frame:

| # | id | ext | dlc | bits | words | cycles/frame | cyc/wirebit |
|---:|---|---:|---:|---:|---:|---:|---:|
| 69 | `0x00000110` | 0 | 0 | 48 | 9 | 3247.13 | 67.65 |
| 72 | `0x000000A6` | 0 | 0 | 48 | 9 | 3248.13 | 67.67 |
| 9 | `0x000000DE` | 0 | 0 | 48 | 9 | 3259.13 | 67.90 |
| 91 | `0x00000430` | 0 | 0 | 48 | 9 | 3264.13 | 68.00 |
| 54 | `0x0000021D` | 0 | 0 | 49 | 9 | 3279.13 | 66.92 |

The firmware prints the full 100-frame table, including per-frame
`pair/word`, `edge/edge` and `cycles/wirebit`.

## 5. Interpretation

- The parser consumes **78.67 cycles per wire bit on average**. At 1 Mbit/s
  and 133 MHz (133 cycles/bit) that is **59.15% of one core** for the parser
  alone; the worst frame reaches **71.60%**.
- This is consistent with the bus-load measurement in `docs/rx_bus_load.md`:
  the full RX ISR (IRQ entry + FIFO drain + parser) measured about 70.6% of a
  core at roughly 90% bus load. The parser is the dominant part; IRQ and PIO
  handling add the rest.
- `can_rx_pair_inline()` is exactly two `can_rx_edge_inline()` calls in
  behaviour and cost. One pair averages 326.72 cycles from whole-frame
  timing; one edge averages 163.36. Measured per-call means are a few cycles
  higher because they include the sample-store and loop bookkeeping.
- The distribution is bimodal: ordinary data/stuffing runs are cheap
  (112–127 cycles per edge), while field boundaries (DLC, CRC byte compare,
  CRC tail/delimiter) and long runs form the upper tail. Per-frame cost
  therefore depends on frame length and stuffing density, not only on DLC.

## 6. Caveats

- Build is `CAN_ACK_GENERATION_SUPPORT=0`: the ACK tail walker and the ACK
  latch are compiled out. The Fly production image defaults to ACK generation
  enabled. To measure that variant, rebuild the same environment with
  `-DCAN_ACK_GENERATION_SUPPORT=1`; the parser struct and tail path change.
- There is no PIO capture jitter, no transceiver slew and no IRQ entry in these
  numbers. The timestamps are ideal nominal 66-tick spacings. This isolates the
  parser; the real ISR is slightly more expensive.
- RTR vectors use DLC 0 because the current parser does not discard the data
  phase of a remote frame with a non-zero DLC.
- The whole-frame timing brackets 8 parses per sample, so the SysTick read
  overhead is below 0.05% of the reported per-frame values.
- Non-ACK frames include the quiet-tail resynchronisation that happens when the
  next SOF arrives; this is the real back-to-back NACK path.

## 7. Reproduce

```sh
cd can2040-lite
pio run -e fly_micro4_rx_bench -t upload
pio device monitor -e fly_micro4_rx_bench
# press Enter or 'r' to rerun
```

Host tests are unaffected:

```sh
make test          # 177 passed, 0 failed
```
