# Production function audit

Status: review draft.

Scope: functions defined in the firmware sources that are compiled for the
target, excluding the test modules (`src/can2040_lite_test.inc`,
`src/pico/can2040_lite_pico_test.cpp`, `src/can_edge_sm.c`) and anything
already inside `#ifdef CAN2040_LITE_TEST`.

Legend:

- **KEEP** — reachable from the public driver API, the fixed backend interface, or
  the RX ISR; required in production.
- **REVIEW** — no production caller: only tests, the generator tool, or a
  diagnostic harness call it. Candidate to move behind `CAN2040_LITE_TEST`,
  move to a test module, or delete.
- **DEAD** — no caller anywhere in the tree.

Scale after cleanup: 67 production functions, all justified. The earlier
pass removed 53 test-only/dead definitions from the production build.

---

## Resolved in this pass

Moved behind `CAN2040_LITE_TEST` or deleted: callback config, weak host
stubs, byte-wise CRC, unstuff/decode, `can_rx_edge/pair`, edge-pair recording,
dead ACK planning helpers, raw capture, Pico test hooks (`rx_service`,
`rx_pending`, `rx_get_parsed`, `rx_bits`, record ring, readback pin).

`can_edge_sm.c` is now `CAN2040_LITE_TEST`-only: the production RX path uses
`can_rx_sm_inline.h`. The ACK block in `can2040_lite_pico.cpp` still names the
removed `edge_sm` object, so `CAN_ACK_GENERATION_SUPPORT=1` needs a rewrite on
top of the new parser before it can be enabled.

---

## src/can2040_lite.c — driver core

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `can2040_lite_setup` (43) | Per-instance initialisation | zero the instance, store `pio_num`, clear a stale single-instance pointer; no hardware access | KEEP |
| `can2040_lite_start` (140) | Bring the driver up | require `start` (and `tx_frame` for TX mode), set parameters, call backend start, clear a stale RX-only queue, mark started, kick a leftover queue | KEEP |
| `can2040_lite_transmit` (186) | Queue one frame | validate ID/DLC, pack/stuff/CRC, enqueue, hand the head to the SM when idle, return 0/-1 | KEEP |
| `can2040_lite_check_transmit` (221) | Report TX queue space | non-zero when started, TX-capable, and the queue has a slot | KEEP |
| `can2040_lite_get_statistics` (230) | Snapshot counters | copy `cd->stats` to the caller | KEEP |
| `can2040_lite_stop` (237) | Release the driver | clear started/active, clear the single-instance pointer, call backend stop | KEEP |
| `put_bits` (61) | Append MSB-first bits | write `nbits` of `value` into the buffer | KEEP |
| `pack_frame` (71) | Build an on-wire frame | lay out ID/DLC/data, compute CRC-15, stuff, append the recessive delimiter; return bit count or 0 | KEEP |
| `msg_is_valid` (128) | Public input validation | reject NULL, DLC > 15, and IDs outside the selected format | KEEP |

## src/can2040_lite_proto.c — bit, CRC, stuffing, decode

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `crc_can15_bits` (60) | CRC over a bit buffer | fold MSB-first bits; used by packing and decode | KEEP |
| `bits_get` (76) | Read one MSB-first bit | return 0/1 | KEEP |
| `bits_put` (80) | Write one MSB-first bit | set or clear the bit in place | KEEP |
| `bits_clear` (88) | Zero a bit range | memset the covering bytes | KEEP |
| `bit_stuff` (96) | Insert stuff bits | copy bits, insert the opposite level after a 5-run, report overflow | KEEP |

## src/can_rx_sm.c — RX state-machine helpers

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `run_windows_init` (17) | Precompute RX run windows | fill `run_hi`/`run_lo` from ticks/bit | KEEP |
| `can_rx_reset` (41) | Reset the RX parser | wait for a frame, clear ring indices/write pointer | KEEP |
| `can_rx_init` (49) | Initialise the RX parser | zero state, set ticks/bit, windows, ACK guard, reset | KEEP |

## src/can_rx_sm_inline.h — hot RX parser

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `can_rx_crc_bit` (28) | One CRC-15 step | shift, conditionally XOR `0x4599` | KEEP |
| `can_rx_fail_rx` (37) | Error-path reset | clear parser state and count CRC/stuff/bit errors | KEEP |
| `can_rx_publish` (49) | Publish a frame | advance the ring, detect overflow, publish | KEEP |
| `can_rx_advance` (65) | One unstuffed bit | assemble ID/DLC/data/CRC, check CRC, handle ACK/tail | KEEP |
| `can_rx_quiet_tail` (159) | No-ACK tail | feed recessive bits until the next SOF or an error | KEEP |
| `can_rx_state` (173) | One level run | classify the delta, handle stuffing, dispatch bits | KEEP |
| `can_rx_edge_inline` (229) | One edge | compute the delta and update the parser | KEEP |
| `can_rx_pair_inline` (239) | One FIFO word | feed the fall and rise edges | KEEP |

## src/pico/can_pio_program.c — runtime PIO images

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `enc_jmp` (42) | Encode `jmp` | opcode + condition + address + delay | KEEP |
| `enc_in` (47) | Encode `in` | opcode + source + count + delay | KEEP |
| `enc_out` (52) | Encode `out` | opcode + destination + count + delay | KEEP |
| `enc_mov` (57) | Encode `mov` | opcode + destination + source | KEEP |
| `enc_irq` (61) | Encode `irq` | opcode + flag index | KEEP |
| `enc_set` (65) | Encode `set` | opcode + destination + data + delay | KEEP |
| `add` (70) | Append an instruction | bounds-check and append | KEEP |
| `build_rx_pair` (82) | Build RX edge-pair image | emit the capture program and wrap points | KEEP |
| `build_tx` (107) | Build one-shot TX image | emit the 32-cycle/bit shifter and DONE/ERR parks | KEEP |
| `build_ack` (148) | Build ACK helper image | emit edge-follow and one-bit dominant drive | KEEP |
| `can_pio_rx_pair_program` (184) | Return RX image (lazy build) | build once, return the image | KEEP |
| `can_pio_tx_program` (192) | Return TX image (lazy build) | build once, return the image | KEEP |
| `can_pio_ack_program` (201) | Return ACK image (lazy build) | build once, return the image | KEEP |

## src/pico/can2040_lite_pico.cpp — Pico backend

| Function | Mission | Expected behavior / duty | Verdict |
| --- | --- | --- | --- |
| `tx_irq_source` (49) | Map a PIO flag to an IRQ source | `pis_interrupt0 + flag` | KEEP |
| `pio_put_nb` (168) | Non-blocking TX FIFO put | fail if full, else enqueue | KEEP |
| `pio_exec_bounded` (175) | Bounded `pio_sm_exec` | spin at most 8 times, fail on stall | KEEP |
| `clkdiv_for` (186) | Derive a PIO divider | `clk_sys / (mult * bitrate)` | KEEP |
| `pio_program_of` (190) | Adapt the runtime image | fill `pio_program_t` fields | KEEP |
| `rx_clock_config` (206) | Choose RX timing | whole two-cycle ticks/bit and a 1/256 divider | KEEP |
| `can2040_lite_pico_rx_init` (224) | Configure RX SM | claim SM, load image, set pins/shift/divider/FIFO, enable source/IRQ | KEEP |
| `can2040_lite_pico_tx_init` (288) | Configure TX SM | claim SM, load image, set pins/shift/FIFO/divider, prepare ACK image | KEEP |
| `claim_event` (367) | Reserve an event slot | return NULL when the ring is full | KEEP |
| `publish_event` (375) | Publish an event slot | advance the tail | KEEP |
| `pop_event` (379) | Consume an event | copy the head and advance | KEEP |
| `rx_convert` (387) | Convert a parser frame | fill `can_parsed_t` from the ring frame | KEEP |
| `rx_sync_stats` (401) | Publish error counters | copy deltas into `cd->stats` under a critical section | KEEP |
| `rx_drain_events` (428) | Ring to event conversion | move available parser frames into the event ring | KEEP |
| `ack_copy_image` (448) | Relocate the ACK image | copy instructions with relocated JMP targets | KEEP |
| `ack_start_follow` (458) | Restart the ACK helper | load the image, enter at the pad level, clear arm state | KEEP |
| `ack_abort_for_tx` (502) | Leave ACK mode for TX | force recessive, restore the TX image, clear arm state | KEEP |
| `ack_service` (614) | ACK state service | finish own TX, retry/reload the helper | KEEP |
| `can2040_lite_pico_rx_irq` (640) | Fast RX ISR body | drain the FIFO, optionally raw-record, feed the parser only | KEEP |
| `pico_service_events` (666) | Deferred event service | report TX done/error, sync stats, drain frames | KEEP |
| `pico_poll` (703) | Backend poll hook | service events, then pop one event | KEEP |
| `can2040_lite_pico_rx_arm` (725) | Re-arm RX | clear parser/ring, restart SM, enable source/IRQ, restart ACK follow | KEEP |
| `can2040_lite_pico_tx_done_pending` (925) | TX-done flag | read the latched PIO flag | KEEP |
| `can2040_lite_pico_tx_error_pending` (929) | TX-error flag | read the latched PIO flag | KEEP |
| `can2040_lite_pico_tx_clear_status` (933) | Clear TX flags | clear DONE/ERR | KEEP |
| `pico_hw_start` (941) | Backend start | select PIO, init RX/TX, set backend_cd, arm RX | KEEP |
| `pico_hw_stop` (967) | Backend stop | disable/unclaim/remove SMs, clear state and event ring | KEEP |
| `can2040_lite_pio_irq_handler` (1001) | Strong IRQ entry | tail-call the RX ISR body | KEEP |
| `can2040_lite_pico_tx_frame` (1028) | Launch one frame | abort ACK mode, reset SM, load count+bits+pad, start | KEEP |
