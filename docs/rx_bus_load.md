# RX at 90% bus load

Measured on the Fly-Micro4 (RP2040, 133 MHz) after the service path stopped
collapsing several parsed frames onto one callback. The bus was at about
90% load, roughly 7.9k frames/s. The CAN pins showed about 81k edges in
200 ms, which matches that load.

## ISR

The application times `can2040_pio_irq_handler` with `timer_hw->timerawl`
(1 µs). One interrupt drains every FIFO word that is already queued, so
this is the whole drain, not one edge.

| | |
| --- | --- |
| Average | ≈ 5.1 µs |
| Max | 15 µs |
| PIO FDEBUG | `0x0` (no RXSTALL) |

The joined RX FIFO is 8 deep. At this edge rate a word arrives about every
5 µs, so a 15 µs drain still fits in the FIFO. The handler is keeping up
with the PIO. It is not inside the 2 µs per-word budget.

## Delivery

Over the measured window the parser and the application saw the same
frames, and the callback reported no errors:

| | |
| --- | --- |
| lite `rx_total` | 309,015 |
| app `rx` | 309,015 |
| app `err` | 0 |

Every frame that entered the parser ring was delivered. The earlier
single-slot hold in `can2040_lite_pio_service` is gone.

A shorter slice of the same run:

| counter | delta | approx rate |
| --- | ---: | --- |
| delivered | +52,220 | ~7.9k/s |
| ring overflow | +58 | ~9/s |
| `parse_error` | +451 | ~68/s (≈ 0.86%) |
| app queue drop | +7 | ~1/s |

`rx_overflow` counts a frame the parser dropped because its ring was full.
It does not count an application queue drop. `parse_error` is a CRC, stuff,
or form error and is not a lost valid frame.

## Parser ring

The parser ring is 32 slots. One slot stays empty so the producer and
consumer can tell full from empty, so 31 frames fit. At 7.9k frames/s that
is about 4 ms of frames between service calls. The event ring stays 16:
`can2040_lite_pio_service` empties it on the same call, and a full event
ring leaves the frame in the parser ring instead of dropping it.

This depth has not been measured again on the bus. The 5.1 µs average
leaves the main loop little time, so a stall longer than those 31 frames
still drops.

## Arrival time

A per-frame hardware timestamp is not recorded. The application can place
arrival within 1–2 ms, which is enough. The message struct is unchanged.

## No-ACK quiet-tail parser

This run validated the index-driven parser's quiet-tail path on an RX-only
image. The RP2040 was built from the live tree with the new
`can_rx_quiet_tail` logic and flashed with the `fly_micro4_record_noack`
environment (`CAN_ACK_GENERATION_SUPPORT=0`). Symbols were confirmed in RAM:

| symbol | address |
| --- | --- |
| `can_rx_quiet_tail` | `0x2000119c` |
| `can2040_lite_pico_rx_ack_errors` | `0x200029cc` |

Bench: ESP32-S3 TWAI in no-ack mode, standard id `0x123`, DLC 8, 1 Mbit/s,
about 90% load (≈8.1k frames/s). The ESP32 had dropped to
`TWAI_STATE_STOPPED` during the RP2040 reflash and needed one 1200-baud reset.

### ISR

Measured over a 10.2 s window:

| | |
| --- | --- |
| IRQ count | 1,452,334 |
| IRQ rate | 142,268 /s |
| Mean ISR | 4.96 µs (≈660 cycles @ 133 MHz) |
| Max ISR | 28 µs |
| ISR load | 70.6% of core 1 |
| PIO FDEBUG | `0x0` (no RXSTALL) |
| RX overflow | 0 |

The ISR stays within budget at this worst-case bus load, though it consumes
about 70% of one core.

### Functionality

Over a 5 s window the ESP32 reported `tx ok` delta 55,555:

| counter | result |
| --- | --- |
| `ack_errors` (app NACK) | +57,307, matching the TX rate within the sampling-window offset |
| `rx` | 0 (delivery is ACK-gated; this image generates no ACK) |
| `parse_err` | 1 total (boot-time partial frame) |
| `crc_errors` | 0 |
| `stuff_errors` | 0 |
| `bit_errors` | 1 total (boot-time partial frame) |

The quiet-tail path finishes every NACKed frame without stuffing, CRC, or
parse errors, and the missing-ACK counter tracks the transmitter 1:1. Frames
are not delivered until ACK generation is present, as intended.
