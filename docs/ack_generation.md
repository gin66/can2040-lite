# ACK generation

Status: **implemented and host-tested; on-target verification pending.** The
schedule, the `can_ack` PIO image (13 words), and the Pico backend are in
the tree. The analyzer cases and the go-window measurement need hardware.
The parser still observes the slot (`can_edge_sm.ack_seen` / `ack`) as
before.

Two firmware gaps are still open. Arming fires when the plan is `ok` and
the pad level matches; it does not recompute `from_bit` after the pad has
already moved, and it does not require the next edge to be one bit away.
Follow mode restarts only when the RX service runs, the program counter is
in `release`, and the parser is past the ACK slot. A recessive slot produces
no FIFO word, so the next frame's first edge finds the parser already inside
that frame and the helper stays in `release`. A stale X must not pulse.

The node will ACK a frame it received, and did not itself transmit, with an
edge-locked pulse on the TX state machine. The pulse is armed from the
expected CRC at the start of the CRC field and allowed only after that CRC
matches. A late or missing allow is a NACK.

## 1. What the protocol requires

CAN ACK field is two bits:

```
... C15 | D | A | Ad | E O F ...
              ^ ACK slot (1 bit, dominant = acknowledged)
          ^ CRC delimiter (must be recessive)
              ^ ACK delimiter (must be recessive)
```

Only three sample points matter (T = nominal bit time, SP ≈ 0.75–0.875):

| sample point | requirement |
|---|---|
| CRC delimiter | recessive |
| ACK slot | dominant |
| ACK delimiter | recessive |

The ACK is a one-bit dominant pulse. It has to cover the transmitter's ACK
sample point and be recessive at both delimiter sample points. A dominant
level at either delimiter sample is a form error.

The pulse is **inset**: dominant from **0.25 T to 1.25 T** of the ACK slot
(one bit wide, starting a quarter bit after the slot boundary). That covers
sample points from 0.5 T through 0.9 T and stays clear of a delimiter sample
at −0.25 T and at +1.75 T for about ±0.3 bit of placement error. A full bit
that starts on the slot boundary has only about 0.2 bit of margin on the
early side.

## 2. What the receiver knows, and when

CRC-15 covers the destuffed bits before the CRC field. At the **start of the
CRC field** the receiver knows:

- the expected 15 CRC bits (`crc_calc`),
- the run state entering the CRC field (`run`, `last`),
- the exact stuffing of the CRC field, including a trailing stuff bit,
- the wire index of the ACK slot: `N = 15 + stuff_bits + 1` delimiter bit.

A 15-bit CRC contributes **0–4 stuff bits**. Four is the trailing-stuff case
(the CRC sequence ends on a run of 5, so one more stuff bit is owed before
the delimiter). N is therefore **16–20** wire bits from the first CRC bit
through the delimiter. That prediction is the lever. It is valid only for
the expected bitstream; a frame that diverges is not ACKed.

The verdict (received CRC vs expected) is complete at the end of the 15th
CRC bit. After it there may be a trailing stuff bit, then the CRC delimiter,
then the ACK slot:

```
 CRC start                  verdict  slot start  sample point
    |                         |          |            |
 ...+-- C1 .. C15 [S] -- D -- A -- Ad --+------------+...
    |                         |< 1-2 bit >|<-- SP*T ->|
    |<-------- N = 16..20 wire bits ---->|
```

`rx_pair.pio` pushes a 1→0 timestamp only when the following 0→1 arrives.
When the last CRC bit or the trailing stuff bit is dominant, that pair is
pushed at the delimiter, one bit before the slot. The inset check sits at
slot+0.25, so the allow has **1.25 bit times** after that push. At 1 Mbit/s
that is 1.25 µs.

The helper takes its phase from a bus edge, so oscillator drift does not
cross the CRC field. The only open-loop interval is the gap from the last
edge before the slot to the pulse, and stuffing limits that gap to 5 bit
times. At the 0.15% tolerance used above 125 kbit/s the error over that gap
is a few hundredths of a bit. Edge detection itself is one TX cycle
(1/32 bit).

## 3. Disposition of the alternatives

| Option | Decision | Reason |
|---|---|---|
| A. Spin in the RX ISR | Drop | A spin across the CRC field is 16–20 µs at 1 Mbit/s and blocks FreeRTOS and FastAccelStepper. A tail-only spin still places the pin from software. RP2040 has no cycle counter. `time_us_32()` is 1 µs, one bit at 1 Mbit/s. |
| B. Hardware alarm | Drop | The Pico alarm pool is the microsecond timer. It is not phase-locked to the CAN bit, and the pin is owned by PIO. |
| C1. Countdown from the exec instant | Drop | The pulse tracks `pio_sm_exec` latency. That latency is interrupt jitter. |
| C2. Compare two free-running counters | Drop | RX and TX use different dividers. `rx_pair.pio` skips one tick of X on every edge. Reading RX X injects an instruction into the pair shifter. |
| D. CRC in PIO (can2040 match SM) | Parked | Cycle-exact, and the only option that removes the CPU from the last bit. It needs a bit-level receiver and about 10 words. Build it only if the go-window measurement in §7 fails. |
| E. Observe only | Shipped behavior, and the runtime fallback | Late arm, own transmit, CRC or stuff failure, and an aborted helper all leave the slot recessive. |

One driver, rated for 1 Mbit/s. A 500 kbit/s spin and a 1 Mbit/s helper would
be two ACK implementations.

PIO has no `jmp irq`. A blocking `wait irq` turns a late allow into a late
ACK. A spare GPIO for `jmp pin` spends a pin; the TX SM's jmp pin is already
the RX pad. The gate is Y, tested after the gap. Y is 0 until the CRC
matches.

## 4. Decision

Reuse the TX state machine. RX keeps `rx_pair.pio` (7 words). The TX
instruction slot stays 14 words and holds one of two images: `can_tx` while
this node transmits, `can_ack` while it receives. FreeRTOS keeps its 2-word
cycle counter. The live set is 7 + 14 + 2 = 23 words, inside the 29-word CAN
budget (21 CAN words plus the 2-word counter), with 6 words still free in
that budget and 9 in the 32-word block. No third SM. The images share the
TX slot's origin. JMP targets in the assembled arrays assume that origin
is 0; the copy adds the slot origin (`can_pio_reloc_instr`, the same add
`pio_add_program` does). `pio_add_program` fills from the top of the
32-word memory, so with RX loaded first the TX slot is at offset 11.
`SMx_ADDR` is absolute: release is `origin + 0`, and the dominant drive
starts at `origin + 11`. A second `pio_add_program` for the swap fragments
the 32-word block.

The TX SM is **enabled between frames**: it runs `can_ack` in follow mode.
`park` in `tx.pio` runs only after a message. The helper enables that SM
itself. Every arm clears Y before the SM is allowed to reach the check.

Clock is the existing TX divider: 32 cycles per bit. SET pins is the TX
pad (0 = dominant). The pin poll is the RX pad. Autopull is off.

```
; can_ack — overlay of the TX slot. Target shape, 13 words.
; The cycle model in extras/tests/pc_based/ack_cycle_model.h owns the encodings.
; The delay is [30], not [31]: the return jump below adds one cycle, so the
; dominant window is exactly 32 cycles (1 bit) wide.
;
;   high / low   entry, chosen from the pad level
;   X            edges still to come, minus 1, once armed;
;                0xFFFFFFFF while only following
;   OSR          gap cycles from the terminal edge to the fire check
;   Y            0 until CRC match; 1 allows the pulse
;
.wrap_target
release:
    set pins, 1              ; recessive, every iteration
.wrap                        ; wrap target = wrap = release
high:
    jmp pin, high
    jmp x--, low
    jmp gap                  ; X expired on a falling edge
low:
    jmp pin, rose
    jmp low
rose:
    jmp x--, high
    jmp gap                  ; X expired on a rising edge
gap:
    out x, 32                ; X = gap cycles; autopull off
gaploop:
    jmp x--, gaploop
    jmp !y, release          ; not verified
    set pins, 0 [30]         ; 31 cycles + the jmp below = 1 bit
    jmp release              ; dominant from slot+0.25 to slot+1.25
```

`jmp x--` with X = 0xFFFFFFFF keeps following. X is overwritten at CRC
start with the small remaining count; that write is the source of truth, so
the helper's earlier count is discarded. `set x, n` can hold it: from the
arm point fewer than 32 edges remain. The gap can be up to about 5 bit
times plus the inset, so it is preloaded through the OSR (`put`, then
`pull` / the `out` above), not through `set`.

While the helper runs:

1. At idle, or re-armed during EOF, start at `high` or `low` to match the
   pad, X = 0xFFFFFFFF, Y = 0. The release loop is not the idle state.
2. At CRC start, compute the edge list. Write X and the OSR gap only while
   the next counted edge is still at least one bit away, so the stolen exec
   cycles cannot land on it. If that edge is closer, spin at most two bit
   times for the RX pad to change, then write the count for the edges
   **after** the edge just seen.
3. If the last edge is already in the RX FIFO, or the pad has already passed
   it, leave Y at 0.
4. On CRC match, `pio_sm_exec` of `set y, 1`. On stuff or CRC failure, do
   nothing.
5. The terminal edge starts the gap. The check at the end of the gap pulses
   only when Y is set. A late `set y` lands in `release`, which drives the
   pad recessive on every iteration, so the bus cannot stick dominant.

The schedule is a pure function of `crc_calc`, `run`, and `last`. It lives
with `can_edge_sm` and is tested on the host. Only the pico backend touches
the SM.

A queued transmit aborts the helper first: `set pins, 1`, disable the SM,
copy the TX image back, then the normal `tx_frame` path. That in-flight
receive is a NACK. The helper is not loaded during this node's own
transmit, and it is restarted only for a later frame, so a loopback of our
own frame is not ACKed.

The CRC delimiter is not sampled before the pulse. In the worst case the
verdict arrives at the start of that bit; waiting for the bit to finish
would consume the slot. A dominant delimiter already makes the transmitter
send an error flag. This pulse can add one dominant bit to that flag. It
cannot ACK a frame whose CRC did not match.

## 5. Pin ownership

The TX SM owns the pad (`sm_config_set_out_pins` /
`sm_config_set_set_pins`). The RX SM only reads it.

- Do not change GPIO function or direction from the ISR. That glitches the
  mux on a live bus.
- `pio_sm_exec` of `set pins` uses the TX SM's SET mapping. It is how the
  CPU forces recessive on abort. It is not the ACK timing mechanism.
- `pio_sm_exec` runs the instruction in place of the SM's next instruction
  when the SM is enabled, and runs it immediately without advancing PC when
  the SM is disabled.

## 6. Fail-safe

Not verified means do not drive. A NACK makes the transmitter retry. A
false ACK hides a bad frame.

- Y defaults to 0 at every arm. The pulse instruction is reached only from
  `jmp !y` falling through.
- The predicted edge list is used only when the received bits still match
  it. A stuff error or CRC mismatch does not set Y.
- Abort, own transmit, and "sync edge already passed" leave the slot
  recessive.
- The release loop re-drives recessive, including when a late exec replaces
  the release instruction for one cycle.

## 7. Caveat that can still force option D

The allow has 1.25 bit times after the worst-case FIFO push, and the pass
line is 1.0 µs at 1 Mbit/s under the real FreeRTOS and stepper load. If USB
holds the core longer than that, raise the PIO IRQ above USB, or accept
NACK during USB transfers. Option D is in scope only when that priority
cannot be raised and those NACKs are unacceptable. Until the measurement
says so, D is not written.
