# CAN2040-Lite Design Document

**License:** GPL-3.0-only. CAN2040-Lite is derived from
[can2040](https://github.com/KevinOConnor/can2040), Copyright (C) 2021
Kevin O'Connor. See [`LICENSE`](../LICENSE) for the full license text.

## Restrictions

CAN2040-Lite is **not a full CAN node**; it is a classic CAN 2.0B data-frame
transceiver. It deliberately does not implement:

- ACK generation — it only observes the ACK slot via readback
- error-flag generation — it only observes 6+ dominant bits
- TEC/REC, Error Active/Passive, or bus-off
- automatic retransmission (the application retries)
- full arbitration participation (TX aborts on conflict)
- **RTR (remote frame) support**
- CAN FD
- programmable bit timing (no SJW/TSEG; fixed 32× oversampling)
- hardware acceptance filters (software filtering only)
- an on-chip transceiver (an external CAN transceiver is required)

A compliant node must be present on the bus to provide ACK and error
confinement.

## SM Reduction from can2040

### can2040 Original SM Allocation

can2040 uses 4 SMs, and all four programs **share a single 32-word PIO
instruction memory** — the complete program memory of one PIO block:

| SM | Name  | Purpose | Program words |
|----|-------|---------|---------------|
| 0  | sync  | Bit timing recovery, SOF/EOF detection, idle detection | 13 |
| 1  | rx    | Read CAN-RX, fill FIFO | 2 |
| 2  | match | CRC match detection, ACK signaling | 10 |
| 3  | tx    | Bit-stuffed transmission, arbitration | 7 |

**Total: 4 SMs sharing one 32-word program (13 + 2 + 10 + 7 = 32).**

### Full rp2040 SM Allocation (suspended-spheres project)

The rp2040 has **2 PIO blocks** (PIO0 and PIO1) with 4 SMs each, i.e. **8 SMs
total**. RP2350 has 3 blocks and 12 SMs.

#### PIO0 — CAN2040-Lite

| SM | Name  | Purpose | Program words | Notes |
|----|-------|---------|---------------|-------|
| 0  | rx    | **32× oversampled capture** — SOF poll, one word per bit, DMA | 7 | Software majority + edge re-sync |
| 1  | tx    | **32-bit word shifter** — MSB-first bit-stuffed stream | 4 | Idle wait/compare in software (TODO) |
| 2  | FreeRTOS | 2-instruction cycle counter | 2 | Reserved |
| 3  | —     | **UNUSED** | — | Free for custom PIO |

**PIO0 instruction memory: rx + tx ≤ 29 words, FreeRTOS 2 words → ≤ 31 of 32
used.** The 29-word CAN budget is a hard constraint, verified with `pioasm`.

#### PIO1 — FastAccelStepper

| SM | Name  | Purpose | Program words | Notes |
|----|-------|---------|---------------|-------|
| 0-3 | axes | One SM per stepper; all SMs on the block share the FAS program | up to 32 (whole block) | FastAccelStepper claims the block's program memory |

With a single axis, FAS uses 1 SM but still claims the block's program memory;
the other 3 SMs on PIO1 remain unclaimed.

**CAN2040-Lite: 2 SMs on PIO0, 11 PIO program words (hard limit 29)**  
**FreeRTOS counter: 1 SM on PIO0, 2 program words**  
**FastAccelStepper (4 axes): 4 SMs on PIO1, whole 32-word program**  
**Free in suspended-spheres: 1 SM on PIO0; 0 on PIO1**  
**With a single FAS axis: 4 of 8 SMs free across the chip**

### Design Rationale

#### Why 2 SMs on PIO0?

CAN2040-Lite uses 2 SMs on **PIO0**:
- **SM0 (rx)**: fixed-rate sampling with hard edge re-sync
- **SM1 (tx)**: bus idle wait + bit-stuffed transmit + per-bit readback

Receive and transmit cannot be folded into one SM: the TX program must run
continuously while shifting out bits, and the RX program must sample the bus at
the same time. Interleaving both directions in one SM would require the SM to
serve two independent bit clocks and would stall one direction while the other
runs.

#### Why edge-resynced sampling in RX (SM0)?

The key insight is that a separate sync SM can be avoided by folding
re-synchronization into the RX program:

- The RX SM samples 32 times per bit; its block is 66 PIO cycles (PIO clock = 66 × bitrate).
- On every observed bus edge it **hard re-syncs the bit phase**.
- There is no phase-locked loop or programmable time-quanta phase segment.
- Clock error cannot accumulate across the frame; it is bounded by the longest
  edge-free interval.

#### Why no match SM?

- CRC match detection triggers ACK injection in can2040
- CAN2040-Lite does NOT inject ACKs — it only observes them via readback
- CRC matching is deferred to software post-processing
- Saves the match SM entirely

#### Why TX path (SM1)?

- CAN2040-Lite transmits frames and verifies them via readback
- Bus idle wait before transmission (CAN protocol requirement)
- The shifted message is SOF through the recessive bit after the CRC. That
  bit is driven and not compared, and the SM parks with the pin left there.
  ACK, end of frame and interframe space are that recessive level
- The only conflict is recessive-sent/dominant-read (lost arbitration, or
  another node driving the bit). The pin is already recessive, so the SM
  parks without driving it again and without an error flag. A dominant bit
  is not compared: wired-AND hides every other transmitter
- Application handles retries (no automatic retransmission)

#### Why no arbitration in TX?

- A compliant CAN node on the bus handles bus arbitration
- CAN2040-Lite nodes simply wait for bus idle before transmitting
- If a collision occurs anyway, the per-bit compare aborts the loser, which is
  the same mechanism real controllers use

#### Why separate PIO blocks?

- CAN2040-Lite uses **PIO0** (SM0 + SM1)
- FastAccelStepper uses **PIO1**
- No resource conflicts between CAN and stepper control
- FreeRTOS cycle counter uses 1 SM and a 2-instruction program on PIO0

### PIO Instruction Budget

| Component | can2040 | CAN2040-Lite |
|-----------|---------|-------------|
| sync SM | 13 | 0 (integrated into rx) |
| rx SM | 2 | 7 (oversampled capture) |
| match SM | 10 | 0 (deferred to software) |
| tx SM | 7 | 14 (one-shot message TX) |
| **Total CAN program words** | **32** | **21 (hard limit 29)** |
| **SMs used (CAN)** | **4** | **2** |
| **SMs free (CAN only)** | **4** | **6** |
| **SMs free (CAN + FreeRTOS counter)** | — | **5** |
| **SMs free (CAN + FreeRTOS + 4-axis FAS)** | — | **1** |

can2040's four programs exactly fill one block's 32-word instruction memory.
CAN2040-Lite's two CAN programs must fit in ≤ 29 words so that the 2-word
FreeRTOS cycle-counter program can coexist on PIO0, leaving at most 1 spare
word.

`rx.pio` = 7 words, `tx.pio` = 14 words, total **21**.
TX runs at 32 PIO cycles per bit (PIO clock = 32 * bitrate) with zero
word-boundary jitter thanks to autopull.  RX captures 32 samples per bit in a
66-cycle block (PIO clock = 66 * bitrate).  A taken PIO branch costs 1 cycle on
this part; re-check these constants when porting to a different PIO version.

## Frame Processing Pipeline

```
CAN Bus (NRZ signal)
    |
    V
SM0 (Rx — Edge-Resynced Sampling):
    - Idle detection (11 recessive bits)
    - SOF detection (first dominant bit)
    - Fixed-rate sampling with hard re-sync on edges
    - Frame-end detection (EOF 7 recessive + IFS 3 recessive)
    - Readback monitoring (for transmitted frames)
    |
    V
DMA Transfer — Frame complete → copy to RAM buffer
    |
    V
Software Post-Processing:
    Step 1: Destuffing (remove stuff bits)
    Step 2: CRC verification
    Step 3: Frame parsing (ID, DLC, data)
    |
    V
Application Callback (valid frames only)
```

### Transmit Path

```
Application → Queue Frame → Bit-Stuff (software)
    |
    V
Bus Idle Wait (11 recessive bits)
    |
    V
Transmit on CAN-TX (bit-stuffed)
    |
    V
Readback on CAN-RX (per bit):
    - Sent recessive, read dominant → abort (lost arbitration / conflict)
    - 6+ consecutive dominant bits → error flag from another node
    |
    V
Report: DONE / ERROR (application retries or evaluates ACK via RX)
```

The signal path is **through the bus**: TX drives the transceiver, the
transceiver drives the bus, and RX reflects the resulting bus level (including
dominant bits from other nodes). It is not an internal transceiver loopback.

## Error Handling Strategy

| Error Type | can2040 | CAN2040-Lite |
|------------|---------|-------------|
| CRC error (RX) | Transmit error flag, retransmit | Discard frame, report to app |
| ACK missing (RX of own frame) | Retransmit | Report NACK to app |
| Error frame (TX readback) | Error flag | Report error to app |
| Stuff error | Error flag | Discard frame, report |
| Lost arbitration (TX) | Automatic arbitration | Abort TX on readback mismatch |
| Bus-off | Automatic state machine | N/A (no state machine) |

**Key principle:** CRC errors are reported to the application layer but the
frame is discarded. The application can use this for diagnostics (noise
monitoring, node health).

An error flag is **6 or more consecutive dominant bits**; the length is 6–12
bits depending on how many error-active nodes signal at the same time.

## Sampling Analysis

### Why edge-resynced fixed-rate sampling works

CAN oscillator tolerances are tighter than the often-quoted “±0.5%” at high
bit rates:

- **≤ 0.5%** permitted up to 125 kbit/s
- **≤ 0.15%** permitted above 125 kbit/s

A pure open-loop sampler would accumulate error over the ~50–160 bit times of a
frame. Edge re-sync removes that accumulation: at every bus edge the receiver
re-aligns its bit phase, so the worst-case sampling error is bounded by the
error accumulated since the last edge. Inside the stuffed region CAN guarantees
an edge at least every 5 bits, so drift stays well below half a bit even at the
0.15% tolerance. Outside the stuffed region (EOF/IFS) the bus is all recessive
and a small mis-sample is harmless.

### Sampling Strategy

- TX PIO clock = 32 × bitrate; RX PIO clock = 66 × bitrate
- Example: 500 kbit/s → TX 16 MHz, RX 33 MHz
- Example: 1 Mbit/s → TX 32 MHz, RX 66 MHz
- Sample at nominal bit center, re-aligning phase on each edge
- Idle = 11 recessive bits; SOF = first dominant bit

### Trade-offs

| Aspect | can2040 (precise) | CAN2040-Lite (edge-resynced) |
|--------|-------------------|------------------------------|
| Clock drift tolerance | Excellent (dedicated sync SM) | Good, bounded by edge interval |
| Implementation complexity | High (4 SMs, full program memory) | Lower (2 SMs, 21 words) |
| Frame loss in extreme drift | None | Possible only if edges are absent |
| Resource usage | 4 SMs + whole 32-word block | 2 SMs, 21 words |

## DMA Integration

### Buffer Size
- Maximum frame: ~164 bits (extended, DLC 8, worst-case stuffing)
- Storage: 5 × uint32_t (160 bits) plus a small margin
- DMA transfer on frame-complete signal

### DMA Configuration
- Source: PIO SM0 RX FIFO
- Destination: Application buffer (circular buffer)
- Trigger: Frame-complete IRQ (from SM0)
- No ARM core intervention during capture

### Throughput
At 1 Mbit/s the ceiling is roughly 6,000–8,000 frames/s depending on frame
length and stuffing. Raw bit capture is on the order of 100–170 kByte/s.

## API Design (Proposed)

```c
// Initialize CAN transceiver
void can2040_lite_setup(struct can2040_lite *cd, uint32_t pio_num);

// Configure callback for received/sent frames
void can2040_lite_callback_config(struct can2040_lite *cd, can2040_lite_cb rx_cb);

// Start the transceiver (no TX pin required for receive-only mode)
void can2040_lite_start(struct can2040_lite *cd, uint32_t sys_clock, uint32_t bitrate,
                       int32_t gpio_rx, int32_t gpio_tx);

// Queue a frame for transmission
int can2040_lite_transmit(struct can2040_lite *cd, struct can2040_lite_msg *msg);

// Check if transmit slot is available
int can2040_lite_check_transmit(struct can2040_lite *cd);

// IRQ handler (called from PIO IRQ)
void can2040_lite_pio_irq_handler(struct can2040_lite *cd);

// Get statistics
void can2040_lite_get_statistics(struct can2040_lite *cd, struct can2040_lite_stats *stats);

// Stop the transceiver
void can2040_lite_stop(struct can2040_lite *cd);
```

## Data Structures

```c
// Notification types for callback
enum {
    CAN2040_LITE_NOTIFY_RX,       // Received frame (from bus)
    CAN2040_LITE_NOTIFY_TX,       // Transmitted frame (confirmed by ACK)
    CAN2040_LITE_NOTIFY_TX_NACK,  // Transmitted frame, no ACK received
    CAN2040_LITE_NOTIFY_TX_ERROR, // Transmitted frame, error detected
    CAN2040_LITE_NOTIFY_ERROR,    // Internal error (buffer overflow)
};

struct can2040_lite_msg {
    uint32_t id;           // CAN ID (11-bit or 29-bit)
    uint32_t dlc;          // Data length code (0-8)
    uint8_t data[8];       // Payload data
    uint32_t flags;        // CAN2040_LITE_ID_EFF, CAN2040_LITE_ID_RTR
    uint32_t crc_error;    // 1 if CRC failed or the frame did not parse
};

struct can2040_lite_stats {
    uint32_t rx_total;      // Successfully received frames
    uint32_t tx_total;      // Successfully transmitted frames (ACK received)
    uint32_t tx_nack;       // Frames with no ACK (readback)
    uint32_t tx_error;      // Frames with error detected (readback)
    uint32_t crc_errors;    // Received frames with CRC errors
    uint32_t stuff_errors;  // Frames with stuffing violations
    uint32_t bit_errors;    // Received frames with form/bit errors
};
```

## Next Steps (TDD Roadmap)

1. ✅ White paper (this document)
2. ✅ Test generator (can_test.c) — create valid/invalid CAN frames
3. ⬜ Destuffing unit tests
4. ⬜ CRC verification unit tests
5. ⬜ Frame parsing unit tests
6. ⬜ Firmware skeleton and public API
7. ⬜ PIO rx state machine (edge-resynced sampling)
8. ⬜ PIO tx state machine (bus idle wait + bit-stuffed transmit)
9. ⬜ Readback path (ACK/NACK/error monitoring)
10. ⬜ DMA integration
11. ⬜ Full integration test on rp2040
