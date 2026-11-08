# CAN2040-Lite Design Document

## SM Reduction from can2040

### can2040 Original SM Allocation

| SM | Name  | Purpose | Instructions |
|----|-------|---------|--------------|
| 0  | sync  | Bit timing recovery, SOF/EOF detection, idle detection | 13 instructions |
| 1  | rx    | Read CAN-RX, fill FIFO | 3 instructions |
| 2  | match | CRC match detection, ACK signaling | 8 instructions |
| 3  | tx    | Bit-stuffed transmission, arbitration | 14 instructions |

**Total: 4 SMs, 38 PIO instructions**

### Full rp2040 SM Allocation (suspended-spheres project)

The rp2040 has **2 PIO blocks** (PIO0 and PIO1), each with 4 SMs.

#### PIO0 — CAN2040-Lite

| SM | Name  | Purpose | Instructions | Notes |
|----|-------|---------|--------------|-------|
| 0  | rx    | **Relaxed sampling** — SOF detection, periodic sampling, frame-end, readback monitoring | ~10 | Simple, no phase tracking |
| 1  | tx    | **Bus idle wait** — wait for 11+ recessive bits, then bit-stuffed transmit | ~8 | No arbitration |
| 2  | —     | **UNUSED** | — | Free for custom PIO |
| 3  | —     | **UNUSED** | — | Free for custom PIO |

#### PIO1 — FastAccelStepper

| SM | Name  | Purpose | Instructions | Notes |
|----|-------|---------|--------------|-------|
| 0  | os    | **2-word OS state machine** for stepper motor step generation | 2 instructions | FastAccelStepper |
| 1-3 | —    | **UNUSED** | — | Free for custom PIO |

**CAN2040-Lite: 2 SMs on PIO0, ~18 PIO instructions**  
**FastAccelStepper: 1 SM on PIO1, 2 instructions**  
**Free: 2 SMs on PIO0 + 3 SMs on PIO1 = 5 SMs total**

### Design Rationale

#### Why 2 SMs on PIO0?

CAN2040-Lite uses 2 SMs on **PIO0**:
- **SM0 (rx)**: Relaxed sampling — no phase tracking, just periodic sampling
- **SM1 (tx)**: Bus idle wait + bit-stuffed transmit

FastAccelStepper uses 1 SM on **PIO1** (separate PIO block, no conflict).

#### Why relaxed sampling in RX (SM0)?

The key insight is that **synchronization can be relaxed** for reception:

- CAN bus nodes are required to have clock accuracy within ±0.5%
- Sampling at the nominal rate introduces at most 2.5% timing error per bit
- No phase tracking or resynchronization needed
- Just sample at fixed intervals determined by PIO clock division

This eliminates the need for a separate sync SM entirely.

#### Why no match SM?

- CRC match detection triggers ACK injection in can2040
- CAN2040-Lite does NOT inject ACKs — it only observes them via readback
- CRC matching deferred to software post-processing
- Saves 8 instructions

#### Why TX path (SM1)?

- CAN2040-Lite transmits frames and verifies via readback
- Bus idle wait before transmission (CAN protocol requirement)
- Readback path monitors CAN-RX for ACK/ERR from transmitted frames
- Application handles retries (no automatic retransmission)

#### Why no arbitration in TX?

- A compliant CAN node on the bus handles arbitration
- CAN2040-Lite nodes simply wait for bus idle before transmitting
- No line arbitration participation

#### Why separate PIO blocks?

- CAN2040-Lite uses **PIO0** (SM0 + SM1)
- FastAccelStepper uses **PIO1** (SM0)
- No resource conflicts between CAN and stepper control
- FreeRTOS cycle counter uses timer, no PIO needed

### PIO Instruction Budget

| Component | can2040 | CAN2040-Lite |
|-----------|---------|-------------|
| SM0 (sync) | 13 | 0 (integrated into rx) |
| SM0 (rx) | 3 | 10 (relaxed sampling) |
| SM2 (match) | 8 | 0 (merged/deferred) |
| SM3 (tx) | 14 | 8 (simplified, no arbitration) |
| **Total** | **38** | **18** |
| **SMs used** | **4** | **1** |
| **SMs free** | **0** | **3** |

## Frame Processing Pipeline

```
CAN Bus (NRZ signal)
    |
    V
SM0 (Rx — Relaxed Sampling):
    - SOF detection (11 dominant bits)
    - Periodic sampling at bit centers (no phase tracking)
    - Frame-end detection (10+ recessive bits)
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
Bus Idle Wait (11+ recessive bits)
    |
    V
Transmit on CAN-TX (bit-stuffed)
    |
    V
Readback on CAN-RX:
    - Monitor for ACK slot (dominant bit)
    - Monitor for Error frames (6 dominant bits)
    |
    V
Report: SUCCESS / ERROR / NACK
```

## Error Handling Strategy

| Error Type | can2040 | CAN2040-Lite |
|------------|---------|-------------|
| CRC error (RX) | Transmit error flag, retransmit | Discard frame, report to app |
| ACK missing (TX readback) | Retransmit | Report NACK to app |
| Error frame (TX readback) | Error flag | Report error to app |
| Stuff error | Error flag | Discard frame, report |
| Bus-off | Automatic state machine | N/A (no state machine) |

**Key principle:** CRC errors are reported to the application layer but the frame is discarded. The application can use this for diagnostics (noise monitoring, node health).

## Relaxed Sampling Analysis

### Why relaxed sampling works

CAN bus specification requires nodes to have clock accuracy within ±0.5%:

- At 1 Mbit/s: actual bitrate is 995-1005 kbit/s
- Sampling at nominal rate: max 2.5% timing error per bit
- Over 100-160 bits: accumulated error is bounded
- Dominant bit (logic 0) is the strongest signal — easy to detect

### Sampling Strategy

- PIO clock = 32 × bitrate (same as can2040)
- Example: 500 kbit/s → PIO clock = 16 MHz
- Example: 1 Mbit/s → PIO clock = 32 MHz
- Sample at bit center (approximate)
- No phase-locked behavior
- SOF provides known starting point (dominant bit)

### Trade-offs

| Aspect | can2040 (precise) | CAN2040-Lite (relaxed) |
|--------|-------------------|------------------------|
| Clock drift tolerance | Excellent (resyncs every bit) | Good (±0.5% spec compliance) |
| Implementation complexity | High (2 SMs, phase tracking) | Low (1 SM, fixed sampling) |
| Frame loss in extreme drift | None | Possible (but within spec) |
| Resource usage | 4 SMs | 1 SM |

## DMA Integration

### Buffer Size
- Maximum frame: ~160 bits (including stuffing)
- Storage: 5 × uint32_t (160 bits)
- DMA transfer on frame-complete signal

### DMA Configuration
- Source: PIO SM0 RX FIFO
- Destination: Application buffer (circular buffer)
- Trigger: Frame-complete IRQ (from SM0)
- No ARM core intervention during capture

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
    uint32_t crc_error;    // 1 if CRC failed, 0 if valid
};

struct can2040_lite_stats {
    uint32_t rx_total;      // Successfully received frames
    uint32_t tx_total;      // Successfully transmitted frames (ACK received)
    uint32_t tx_nack;       // Frames with no ACK (readback)
    uint32_t tx_error;      // Frames with error detected (readback)
    uint32_t crc_errors;    // Received frames with CRC errors
    uint32_t stuff_errors;  // Frames with stuffing violations
};
```

## Next Steps (TDD Roadmap)

1. ✅ White paper (this document)
2. ✅ Test generator (can_test.c) — create valid/invalid CAN frames
3. ⬜ Destuffing unit tests
4. ⬜ CRC verification unit tests
5. ⬜ Frame parsing unit tests
6. ⬜ PIO rx state machine (relaxed sampling)
7. ⬜ TX path (bus idle wait + bit-stuffed transmit)
8. ⬜ Readback path (ACK/ERR monitoring)
9. ⬜ DMA integration
10. ⬜ Full integration test on rp2040