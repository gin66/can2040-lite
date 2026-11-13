# Fly-Micro4 / RP2040 CAN2040-Lite test findings

Status: **RX-only milestone complete. TX and ACK are not yet validated.**
The RP2040 receives and decodes real 1 Mbit/s CAN frames, but the recorded
capture contains no ACK because the node under test was deliberately
receive-only. The next step is to make the ACK path non-blocking and verify it.

Hardware/software used:

- Mellow Fly-Micro4 V1.0, RP2040 at 133 MHz, TJA1051/MCP2542 transceiver,
  CAN RX GPIO4, CAN TX GPIO5.
- RP2040 image: receive-only (`SS_CAN_RX_ONLY=1`) and
  `CAN_ACK_GENERATION_SUPPORT=0`, plus the raw RX FIFO capture API.
- Peer: Waveshare ESP32-S3, TWAI in **no-ack mode**, 1 Mbit/s, standard
  id `0x123`, DLC 8, data byte 0 incremented once per frame, about 1 ms
  period. No second node is needed and the ESP32 never waits for an ACK.
- Host tests: `make test` in `can2040-lite`.

## 1. Current status

- **RX only. No TX. No ACK.** The RP2040 does not answer ASK, does not send
  status, and does not drive the ACK slot. Every decoded recording frame has
  `ack = 0`.
- The raw capture path drains the PIO RX FIFO into a RAM buffer without
  running the protocol parser. This proved that the PIO/electrical capture is
  correct even when the production parser was still failing.
- The production parser decodes the recorded hardware data. Three
  consecutive frames decode with valid CRC:
  `1e..25`, `1f..26`, `20..27`, CRCs `7B32`, `72B8`, `30DA`.
- The edge classifier is now grounded rather than a placeholder: each
  edge-to-edge interval rounds to the nearest whole bit and is accepted
  within `n*T/200 + T/8 + 1` ticks (0.5% clock, dominant/recessive slew
  asymmetry, capture quantization). See section 4.
- Host test suite: `TOTAL: 137 passed, 0 failed`, including the recorded
  three-frame replay, the asymmetric-edge cases and a six-bit-run stuff
  check.
- ACK generation is **not** working yet. See section 6 for the multi-second
  interrupt finding. Decoder and tests are done; the ACK path is the next
  milestone.

## 2. IRQ number portability (why 6/7 changed)

The Pico backend used PIO IRQ flags 6 and 7 for TX done and TX error. That
compiles on RP2350, where `pio_interrupt_source_t` exposes
`pis_interrupt4..7` if `PICO_PIO_VERSION > 0`. On RP2040 only flags 0..3 can
be routed to the system IRQ; `pis_interrupt6` and `pis_interrupt7` do not
exist and the build fails.

The working change is:

- `CAN_TX_DONE_IRQ = 0`, `CAN_TX_ERR_IRQ = 1`.
- PIO image builder uses `enc_irq(0)` / `enc_irq(1)`.
- IRQ source routing uses `pis_interrupt0` / `pis_interrupt1`.
- `pio_interrupt_get/clear` use the same constants.

Recommendation for the next agent: put the flag numbers in
`can2040_lite_config.h` and derive the source enum from them, so the PIO
program and the CPU-side routing cannot drift:

```c
#define CAN_PIO_TX_DONE_IRQ 0u
#define CAN_PIO_TX_ERR_IRQ  1u
#if CAN_PIO_TX_DONE_IRQ > 3u || CAN_PIO_TX_ERR_IRQ > 3u
#  if PICO_PIO_VERSION == 0
#    error "RP2040 can only route PIO IRQ flags 0..3"
#  endif
#endif
```

These are PIO IRQ **flag** numbers, not system IRQ numbers. The application
still installs `PIOx_IRQ_0`; `can2040_lite_pico` only configures the sources.
Also verify that no other SM on the same PIO block uses flags 0/1.

Two integration points are also required on RP2040:

- `lib_archive = no` (or a strong backend anchor). With an archive, the weak
  `can2040_lite_hw_attach` definition in `can2040_lite.c.o` satisfies the
  call and the linker never extracts `can2040_lite_pico.cpp.o`, so the Pico
  backend is silently dropped.
- Exclude all CAN objects from flash: `*can2040_lite*.o`, `*can_edge_sm.c.o`,
  `*can_pio_program.c.o` in both `.text` and `.rodata` (see
  `rp2040/can2040_ram.py` and `rp2040/memmap_katapult.ld` in the
  suspended-spheres tree). Verify with `nm`: the handler, parser, ACK helper
  and raw capture routines must be at `0x2000_xxxx`.

## 3. Raw recording (for tests)

This is the **raw data**, not a decoded/post-processed view. Each word is
exactly what the Pico backend drains from the RX FIFO:

```
word = (fall_timestamp16 << 16) | rise_timestamp16
```

Both timestamps are the low 16 bits of the PIO two-cycle down-counter. The
nominal bit period is **66 ticks** at 1 Mbit/s on this 133 MHz board. The
slice below starts at the SOF of a complete frame, contains **three complete
CAN messages**, and ends with the next SOF so the third message completes
without a synthetic quiet-tail flush.

Full capture (527 words, includes partial frames before/after):
`extras/tests/common/can_rx_record_fly_micro4_1m.txt`

Test fixture (the 77 raw words below):
`extras/tests/common/can_rx_record_fly_micro4.h`

Raw words:

```c
static const uint32_t can_rx_record_fly_micro4_1m[] = {
    0x4A8A49C4u, 0x49844900u, 0x48C147FAu, 0x477946B2u,
    0x46734528u, 0x44E944A6u, 0x43A14298u, 0x4151408Bu,
    0x404B3F01u, 0x3EC13E3Du, 0x3DFD3CF5u, 0x3CB53C31u,
    0x3BF13B2Bu, 0x3AEB3A25u, 0x39E5391Fu, 0x389D3819u,
    0x37DA3755u, 0x3716360Du, 0x35CE3549u, 0x350A34C7u,
    0x338032FBu, 0x327A31F5u, 0x317430EFu, 0x30B0306Du,
    0x48F84832u, 0x47F2476Eu, 0x472F4668u, 0x45E74520u,
    0x44E14396u, 0x43574314u, 0x41CD4106u, 0x40C73F7Du,
    0x3F3D3EB9u, 0x3E793D71u, 0x3D313CADu, 0x3C6D3BA7u,
    0x3B673AA1u, 0x3A61399Bu, 0x39193895u, 0x385537D1u,
    0x37913689u, 0x364A35C5u, 0x35863543u, 0x3504347Fu,
    0x344033BBu, 0x333A32F7u, 0x323431AFu, 0x3170312Du,
    0x30EE30ABu, 0x2FE82F21u, 0x476A46A4u, 0x466445E0u,
    0x45A144DAu, 0x44594392u, 0x43534208u, 0x4187403Cu,
    0x3FFD3F78u, 0x3F393E31u, 0x3DF13D6Du, 0x3D2D3C67u,
    0x3C273B61u, 0x3B213A5Bu, 0x39D93955u, 0x39153891u,
    0x38513749u, 0x37093685u, 0x36453603u, 0x35C4353Fu,
    0x3500347Bu, 0x33FA3333u, 0x32F4326Fu, 0x31AC3169u,
    0x30E82FDFu, 0x2F5E2F1Bu, 0x2E9A2E57u, 0x2E182DD5u,
    0x45DD4517u,
};
```

Expected decoded metadata for those three messages (id `0x123`, DLC 8):

| frame | data | received CRC |
|---|---|---|
| 1 | `1e 1f 20 21 22 23 24 25` | `7B32` |
| 2 | `1f 20 21 22 23 24 25 26` | `72B8` |
| 3 | `20 21 22 23 24 25 26 27` | `30DA` |

The expected data/CRC table is metadata only; the fixture and the report
contain the raw edge words above.

## 4. Edge timing finding

Measured from the recording (ticks, nominal bit = 66):

| polarity | bits | mean | mean - n*66 |
|---|---:|---:|---:|
| dominant (fall->rise) | 1 | 66.55 | +0.55 |
| dominant | 2 | 132.47 | +0.47 |
| dominant | 3 | 198.51 | +0.51 |
| dominant | 4 | 264.47 | +0.47 |
| recessive (rise->fall) | 1 | 63.41 | -2.59 |
| recessive | 2 | 129.58 | -2.42 |
| recessive | 3 | 195.62 | -2.38 |
| recessive | 4 | 262.00 | -2.00 |

The offset is **constant per interval**, not proportional to the number of
bits. The slope is the correct 66 ticks/bit; only the intercept depends on
the edge polarity. This is not a bit-rate or oscillator-tolerance error. It
is consistent with a fixed polarity-dependent edge delay (recessive bus rise
is RC/pull-up limited, dominant fall is actively driven) plus the fixed PIO
edge path.

Consequences for the decoder:

- A single `bias` or one frame-wide phase accumulator cannot represent it.
  The old `advance_to` accumulator merged a 63-tick recessive bit into the
  next interval and drifted.
- The receiver must use **edge-to-edge delta time only**, round the run to the
  nearest whole `n` bits, and re-anchor the phase at every edge. Bit stuffing
  guarantees at most five identical bits, so the free-running interval is
  bounded; tolerance never accumulates.
- The acceptance budget for `|elapsed - n*T|` is the sum of three independent
  errors: 0.5% of the interval (ISO 11898-1 bounds oscillator tolerance by
  `SJW/(20*NBT)`, 0.625% for SJW=1, NBT=8 TQ); `T/8` for the dominant/
  recessive slew and comparator asymmetry (the recording measures 2.6 ticks
  at `T = 66`, about 4% of a bit); and one tick of two-cycle capture
  quantization.
- Runs longer than five are still classified so the stuff state machine
  reports `INVALID_STUFF` on the sixth bit rather than a generic form error.
  An interval longer than the maximum legal run is also classified as a
  stuff violation. Intervals that are not a whole number of bits within the
  budget report `INVALID_BIT`.
- The quiet tail (CRC delimiter, ACK, EOF, IFS) still free-runs on the
  nominal period from the last edge: at most the 10-bit interframe gap.

Implementation in the current tree (`src/can_edge_sm.c`):

- `run_bits()`: nearest-`n` classifier with the `n*T/200 + T/8 + 1` budget;
  an interval longer than ten bits is treated as a stuff violation.
- `advance_edge()`: per-edge delta classification and re-anchor.
- `edge_mark`: last bus edge time, independent of the quiet-tail `mark`.
- `run_done`: bits already emitted since that edge by the quiet path; the
  next edge emits only the missing bits. This is required because the Pico
  service calls `can_edge_sm_time()` between FIFO words.

## 5. Test strategy

Host tests are in `extras/tests/pc_based/can_test_units.c`.

Synthetic asymmetric tests:

- `drive_batch_asym()` models a fixed polarity offset:
  dominant `n*T + 1`, recessive `n*T - 2` at the test period.
- All standard DLC 0..8 NACK frames, extended and RTR frames.
- `run delta outside the edge-timing budget is a bit error` injects an
  interval beyond the budget and expects `INVALID_BIT`.
- `six-bit run with measured edge skew is a stuff error` keeps a run longer
  than five classified at its true length, so the parser reports
  `INVALID_STUFF` instead of dropping the sixth bit and surfacing a CRC error.
- `feed_pairs()`/`drive_pairs()` use the real capture model: every edge
  timestamp carries the same constant sampling offset and the decoder uses
  `bias = 0`, matching the recorded hardware.
- A dominant ACK changes the tail edge pattern; those asymmetric ACK cases
  are deliberately deferred to the ACK step. The RX milestone is NACK-only.

Recorded-hardware replay:

- `test_recorded_fly_micro4()` feeds the raw words above through the
  production pair decoder and edge state machine:
  `can_edge_pair_push()` then `can_edge_sm()` with `ticks_per_bit = 66` and
  `bias = 0`.
- Between words it calls `can_edge_sm_time()` only up to one tick before the
  next recorded edge, matching the Pico service's behind-real-time estimate.
- It asserts three frames with id `0x123`, DLC 8, the exact data bytes and
  the exact received CRC, and that `crc == crc_calc`.
- This test fails on the old accumulator decoder and passes on the current
  per-edge classifier.

On-target bring-up commands used (RX-only image):

- `can raw <ms>`: drain raw RX FIFO words to the serial port; no parser, no
  TX, no ACK.
- `can dbg`: PIO IRQ count, max/sum IRQ time in microseconds, and `FDEBUG`.
- `can stats`: portable driver statistics and application counters.
- `can arm`: core-1 re-arm of the RX PIO/parser (flush FIFO and state).

## 6. Super-long IRQ time with ACK generation

Status: **open.** The decoder and test work in this tree does not touch the
ACK path below; it remains the next milestone.

With ACK generation enabled, the RX ISR can block for **seconds**:

```
can dbg (ACK build, RX not held):
  irq=4 us_max=4415991 us_sum=4416214 fdebug=0x1
```

`us_max` is 4.4 s, and `FDEBUG = 0x1` is `RXSTALL` for SM0. The RX FIFO
(8 words joined) fills while the ISR is not draining it; edges are then lost.
With ACK support compiled out:

```
can dbg (RX-only build):
  irq=2 us_max=171 us_sum=174 fdebug=0x1
```

The 4.4 s is not parser cost. The ACK path calls blocking primitives from the
RX ISR:

- `ack_arm()`: `pio_sm_clear_fifos`, `pio_sm_put_blocking`,
  `pio_sm_exec_wait_blocking`.
- `ack_allow()`: `pio_sm_exec_wait_blocking`.
- `ack_start_follow()/ack_abort_for_tx()`: `pio_sm_put_blocking` and
  `pio_sm_exec_wait_blocking`.

If the TX/ACK SM is disabled, already parked, or the FIFO/OSR condition is
not ready, `pio_sm_exec_wait_blocking` waits for an instruction that may
never retire. The RX ISR then never returns; this is fatal at 1 Mbit/s.

Required next step:

- Remove every unbounded blocking wait from the RX ISR path. Use non-blocking
  `pio_sm_exec()` plus a bounded `pio_sm_is_exec_stalled()` poll, or defer
  ACK arming/allow to a bounded state machine outside the ISR.
- Keep the fail-safe: no ACK unless the CRC match is verified.
- Re-measure `us_max` at 1 Mbit/s with ACK enabled. The RX FIFO only tolerates
  about the time to receive 8 pairs; the ISR budget is a few microseconds,
  not milliseconds.

## 7. Open items

1. Replace the ACK blocking waits; verify ACK generation on the Fly-Micro4
   with the ESP32 (or a second RP2040) as a compliant peer.
2. Add asymmetric-timing ACK-tail tests after the ACK path is non-blocking.
3. Verify TX (the current recording is RX-only).
4. Re-run the recording with a node that ACKs so the ACK slot and EOF tail
   are real recorded edges, then measure ACK-slot margins.
5. Decide the final upstream form of the IRQ-number defines and the
   `lib_archive`/anchor requirement.
6. Restore the ESP32 show-controller image after the bench work; the current
   ESP32 runs `esp32cantx` (TWAI no-ack transmitter).

## 8. Reproduce

Host:

```sh
cd can2040-lite
make test
```

On-target (suspended-spheres tree):

```sh
cd rp2040 && pio run -e fly_micro4_rxraw
# flash the RP2040, leave the ESP32 running esp32cantx
# then over the RP2040 USB serial:
can arm
can raw 20
can dbg
```

The ESP32 bench transmitter was built from `esp32cantx/` in the
suspended-spheres tree with TWAI `TWAI_MODE_NO_ACK`.
