# RX ISR struct review

Running decision log for `can_edge_sm_t`. One field/group at a time.
Final decisions are reflected in `src/can_edge_sm.h`.

Starting size: 136 bytes.

## Decisions

| # | Field / group | Decision | Notes |
|---|---|---|---|
| 1 | `ticks_per_bit` | KEEP, `uint16_t` | Constant for one bitrate. If `CAN_BIT_TICKS` is defined at build time, use it directly and omit the struct field. Otherwise keep the 2-byte per-instance field so the host tests can use different tick values. |
| 2 | `run_hi[6]` | REPLACE | Runtime: precompute five upper bounds into one global `uint16_t can_edge_run_hi[5]` (10 B) at init, same approach as the bit ticks. Fixed bitrate: use five compile-time `EDGE_HI` constants and omit the array. Classifier is an unrolled compare chain; no loop and no per-edge math. |
| 3 | tail bit clock: `mark`, `accum`, `tail_mark`, `tail_sample` | CUT | No per-bit tail counting. Use `ST_WAIT_ACK` and `ST_WAIT_MESSAGE`. A dominant edge inside the ACK window is ACK; a dominant edge after the ACK deadline is the next message's SOF and the previous frame is NACK; timeout without a dominant edge is NACK. No ACK-delimiter/EOF/IFS validation. Keep one ACK deadline timestamp (reuse `ack_arm_time`). |
| 4 | `edge_mark` | CUT | No timestamp in the parser. Pair decoder keeps one 16-bit `prev_edge` and passes `(level, n)`. |
| 5 | `can_edge_pair_t` / `have_prev` | CUT | Drop the struct entirely. First edge after reset is ignored; SOF needs no delta, so no first-edge flag is needed. |
| 6 | `run_done` | CUT | No quiet-time bit emission path, so nothing to de-duplicate. |
| 7 | `shift` | SHRINK to `uint8_t` | Current byte shift register only. |
| 8 | `id` | REPLACE | Not a numeric accumulator. ID bits flush into `bytes[]`; state carries the byte slot; post-process in `main`. |
| 9 | `data[8]` | REPLACE | Same `bytes[]` buffer as the header and CRC. |
| 10 | unified buffer | `uint8_t bytes[15]` | Unstuffed header + data + CRC. On every 8th bit: `bytes[slot] = shift`, slot encoded in state. No shifts/masks in the hot path. |
| 11 | `crc_calc` | KEEP, `uint16_t` | CRC-15 over SOF..data, excluding the CRC field. Edge prediction uses the CRC sequence + `run`/`last`, not popcount. |
| 12 | `crc_recv` | CUT | Compare each received CRC bit against `(crc_calc >> (rem-1)) & 1`; maintain `crc_match`. Reconstruct `parsed.crc` from `bytes[15]` if reporting needs it. |
| 13 | `raw_bits` | CUT | No frame-wide wire counter. ACK plan only needs a CRC-relative position, max ~21 bits (`uint8_t`, not `uint16_t`). Drop `total_bits`/`stuffed_bits` metadata. |
| 14 | `stuffed_raw` | CUT | User: purpose unclear; not needed. |
| 15 | `in_frame` | CUT | Redundant with `state` (`ST_WAIT_MESSAGE` vs parsing). |
| 16 | `level` | CUT | Pair decoder passes `(level, n)` with each run. |
| 17 | `run` | CUT | Run length is passed from edge timing; after a 5-run drop the next run's first bit. |
| 18 | `last` | CUT | No run bookkeeping. |
| 19 | `is_ext` | CUT | IDE bit at bit 13 selects standard/extended state; output from `bytes[1]`. |
| 20 | `is_rtr` | CUT | RTR unsupported: RTR bit → invalid, wait for idle. |
| 21 | `dlc` | CUT | Computed at the 4th DLC bit; standard is 1 bit in `bytes[1]` + 3 in `shift`, extended is 4 in `shift`. |
| 22 | `crc_match` | CUT | Mismatch fails immediately; reaching CRC end implies match. |
| 23 | `ack_seen` | CUT | State encodes it. |
| 24 | `ack` | CUT | ACK/NACK encoded in completion state. |
| 25 | `frame_ready` | CUT | 3-slot (or more) round-robin ring with `rx_idx`/`tx_idx`. |
| 26 | `validity` | CUT | Invalid frames do not advance `rx_idx`; errors only in stats. |
| 27 | `frame_seq` | CUT | Not needed. |
| 28 | `ack_arm_time` | CUT for now | ACK window handled outside the SM. |
| 29-34 | `last_edge_raw`, `crc_raw_start`, `crc_run_start`, `crc_last_start`, `crc_started`, `ack_armed` | CUT | ACK plan inputs/state move out or are derived. |
| 35 | `parsed` | CUT | Ring slots hold `bytes[15]` + state; app post-processes. |

## Final survivor set (SM only)

```c
uint8_t  shift;         // 1
uint8_t  state;         // 1
uint16_t crc_calc;      // 2
```

4 bytes. New-frame init is only `state = ST_SOF` and `crc_calc = 0`.
`shift` is not cleared (8 shifts overwrite it).
ACK window is handled outside the SM.

## Round-robin buffer (provisional)

8 slots. Slot layout provisional, may change during state-machine redesign:

```c
typedef struct {
    uint32_t id;       // 0..3
    uint8_t  data[8];  // 4..11
    uint8_t  dlc;      // 12
    uint8_t  flags;    // 13
    uint16_t crc;      // 14..15
} can_rx_frame_t;      // 16 bytes, 8 slots = 128 bytes
```

`uint8_t rx_idx` (ISR producer) and `uint8_t tx_idx` (app consumer), single writer each.
Valid frame advances `rx_idx`; invalid frame overwrites the same slot.

## Service state

`prev_edge` (16-bit), stuff state (1 byte), `rx_idx`/`tx_idx` (2 bytes).
- edge/bit clock fields
- parser accumulators
- `data[8]`
- state/flags
- ACK-only fields
- `parsed`
