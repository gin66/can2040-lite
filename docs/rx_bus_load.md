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
