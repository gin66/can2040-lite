# White Paper

## CAN2040-Lite
A Minimal CAN Transceiver Architecture for RP2040 and RP2350 Using PIO

**Version:** Draft 0.3  
**Status:** Design Specification (TDD-driven)  
**Last Updated:** 2025-11-08  
**Project:** suspended-spheres (Kinetic Art Installation)  
**Target Hardware:** Raspberry Pi Pico (rp2040)

---

## Abstract

CAN2040-Lite is a lightweight CAN transceiver architecture for RP2040 and RP2350 microcontrollers that achieves full CAN frame transmission and reception using only **two PIO state machines (SMs) on one PIO block**.

The key insight is that synchronization logic can be integrated directly into the receive state machine, eliminating the need for a separate sync SM. Combined with a readback path that monitors transmitted bits on the CAN-RX line, the architecture provides full transmit/receive capability while using only 2 of 4 available SMs on **PIO0** — leaving **PIO1 entirely free** for FastAccelStepper.

Examples include:

- Kinetic art installations (suspended-spheres project)
- Broadcast control networks with readback verification
- Distributed actuator systems
- Sensor aggregation with status reporting
- Monitoring and diagnostics nodes

By integrating sync into the rx path and adding a readback transmit path, CAN2040-Lite achieves full CAN communication while reducing the state machine footprint from can2040's 4 SMs down to just 1.

---

## 1. Motivation

Existing software CAN implementations such as can2040 strive to behave as fully compliant CAN nodes using 4 PIO state machines. This requires implementation of numerous real-time protocol mechanisms.

The most challenging parts are not frame decoding itself, but:

- ACK slot handling
- Error flag insertion
- Bit monitoring during transmission
- Immediate protocol reactions
- State machine transitions between receiver and transmitter modes

These requirements introduce substantial implementation complexity.

### The Key Observation

Many embedded systems need full CAN communication (send + receive) but do not need the full complexity of a CAN controller:

- No retransmission needed (application handles retries)
- No fault confinement (another node handles it)
- No bus-off recovery (always connected to a compliant node)
- No automatic error state machine

Consider a network consisting of:

```
Master (compliant CAN node)
  |
  +--- CAN2040-Lite Node 1
  +--- CAN2040-Lite Node 2
  +--- CAN2040-Lite Node 3
  ...
  +--- CAN2040-Lite Node N
```

where:

- The master is a fully CAN-compliant node (handles ACK, error flags, arbitration)
- CAN2040-Lite nodes transmit frames and verify reception via readback
- CAN2040-Lite nodes receive frames from the master

In such systems, full CAN communication is needed, but full CAN compliance is not.

---

## 2. Design Philosophy

The central principles are:

1. **Receive first. Validate later.** — Capture the entire bitstream, then validate in software.
2. **Integrate sync into rx.** — The rx SM handles synchronization directly; no separate sync SM needed.
3. **Readback verification.** — Transmitted frames are verified by monitoring the CAN-RX line.

Conventional CAN controllers perform protocol analysis while the frame is still being received.

CAN2040-Lite instead records the entire bitstream and defers all validation until after frame completion.

The bus remains unaffected by the receiver (no error flags, no arbitration participation).

---

## 3. Scope

### Implemented

- ✅ SOF detection (integrated in rx SM)
- ✅ Bit timing recovery (integrated in rx SM)
- ✅ Hard synchronization (integrated in rx SM)
- ✅ Edge-based resynchronization (integrated in rx SM)
- ✅ Bit acquisition (integrated in rx SM)
- ✅ Frame-end detection (integrated in rx SM)
- ✅ Software destuffing
- ✅ Software CRC verification
- ✅ Software frame parsing
- ✅ Frame transmission (bit-stuffed)
- ✅ Readback path (transmitted bits monitored on CAN-RX)
- ✅ ACK evaluation (from readback)
- ✅ Error frame evaluation (from readback)

### Not Implemented

- ❌ ACK injection (transmitted by compliant master node)
- ❌ Error frame generation (handled by compliant master node)
- ❌ Arbitration participation (handled by compliant master node)
- ❌ Retransmission (application handles retries)
- ❌ Error counters (TEC/REC)
- ❌ Error Active / Error Passive states
- ❌ Bus Off state

---

## 4. Architecture

### Hardware Constraints (rp2040)

The rp2040 has exactly **4 PIO state machines (SMs)** across **2 PIO blocks** (PIO0 and PIO1).

The suspended-spheres project uses:
- **PIO0**: CAN2040-Lite — 2 SMs (SM0 rx + SM1 tx)
- **PIO1**: [FastAccelStepper](https://github.com/H2coding/FastAccelStepper) — 1 SM (the 2-word OS state machine for step generation)
- **FreeRTOS** cycle counter: uses timer/counter, no PIO SM needed

**CAN2040-Lite uses 2 SMs on PIO0 (SM0 + SM1), leaving 2 SMs free on PIO0**
**FastAccelStepper uses 1 SM on PIO1, leaving 3 SMs free on PIO1**

This is a clean separation: CAN and stepper control use separate PIO blocks, avoiding resource conflicts.

### SM Reduction Plan (from can2040's 4 SMs)

can2040 uses 4 SMs:

| SM | Name | Purpose | Instructions |
|----|------|---------|--------------|
| 0 | sync | Bit timing recovery, SOF/EOF detection, idle detection | 13 instructions |
| 1 | rx | Reads CAN-RX pin, fills FIFO | 3 instructions |
| 2 | match | CRC match detection → ACK injection signaling | 8 instructions |
| 3 | tx | Bit-stuffed transmission, line arbitration | 14 instructions |

**Total: 4 SMs, 38 PIO instructions**

### Full rp2040 SM Allocation (suspended-spheres project)

#### PIO0 — CAN2040-Lite

| SM | Used By | Purpose | Instructions |
|----|---------|---------|--------------|
| 0 | **CAN2040-Lite** | RX — relaxed sampling, SOF detection, frame-end, readback | ~10 |
| 1 | **CAN2040-Lite** | TX — bus idle wait, bit-stuffed transmit | ~8 |
| 2 | **UNUSED** | Free for custom PIO | — |
| 3 | **UNUSED** | Free for custom PIO | — |

#### PIO1 — FastAccelStepper

| SM | Used By | Purpose | Instructions |
|----|---------|---------|--------------|
| 0 | **FastAccelStepper** | 2-word OS state machine for step generation | 2 instructions |
| 1-3 | **UNUSED** | Free for custom PIO | — |

**CAN2040-Lite: 2 SMs on PIO0, ~18 PIO instructions**  
**FastAccelStepper: 1 SM on PIO1, 2 instructions**  
**Free: 2 SMs on PIO0 + 3 SMs on PIO1 = 5 SMs total**

### Key Insight: Relaxed Sampling in Rx

In can2040, a separate sync SM generates sample IRQs at bit center, which the rx SM reads. This requires two SMs working in concert, with precise phase tracking and resynchronization on every bit transition.

In CAN2040-Lite, the RX SM (SM0) uses **relaxed (approximate) sampling**:

- It reads the CAN-RX pin at fixed intervals determined by a simple clock division (PIO clock = 32 × bitrate)
- It does NOT track phase offsets between transmitter and receiver clocks
- It does NOT resynchronize on bit transitions
- It simply samples at approximate bit centers
- SOF detection (11 dominant bits) identifies frame start
- Frame end detection (10+ recessive bits) identifies frame end
- No phase-locked behavior — just periodic sampling

This eliminates all synchronization logic from the rx path. The rx SM becomes trivially simple.

**Why this works:** CAN bus nodes are required to have clock accuracy within ±0.5% (nominal bitrate tolerance). At 1 Mbit/s, this means the actual bitrate is between 995 kbit/s and 1005 kbit/s. Sampling at the nominal rate introduces at most 2.5% timing error per bit — well within the margin for reliable sampling. Over a full frame (~100-160 bits), the accumulated error is bounded and does not cause bit misreads because:

1. The sampling rate is fixed and known
2. SOF provides a known starting point (dominant bit)
3. Each bit is sampled independently (no phase accumulation)
4. The dominant bit (logic 0) is the strongest signal — easy to detect

**Trade-off:** Relaxed sampling may miss frames in extreme clock drift scenarios, but for normal CAN bus operation (nodes within spec), it works reliably. This is a worthwhile trade-off for eliminating the sync SM entirely.

### Transmit Path (SM1): Readback Verification

CAN2040-Lite is not passive-only. It transmits frames and verifies reception via a readback path.

The TX SM (SM1) handles transmission with bus idle wait:

1. **Bus Idle Wait:** Before transmitting, the TX path waits for bus idle (11+ consecutive recessive bits — intermission). This is required by the CAN protocol for proper line arbitration.

2. **Transmission:** The application queues a frame for transmission. Software handles bit-stuffing and transmits the frame on the CAN-TX pin.

3. **Readback:** Simultaneously, the RX SM (SM0) monitors the CAN-RX pin for the transmitted bits. Since the transceiver loops TX to RX internally, the transmitted frame appears on the RX line.

4. **ACK Evaluation:** The RX SM looks for the ACK slot (dominant bit) followed by the ACK delimiter (recessive). If an ACK is found, transmission is confirmed successful.

5. **Error Evaluation:** If an error frame (6 dominant bits) is detected on the RX line during transmission, the error is reported to the application.

6. **No Injection:** CAN2040-Lite does NOT inject ACK bits or error flags — it only observes them. A compliant CAN node on the bus handles these.

### PIO Capture Engine

PIO performs only real-time functions.

#### SM0 (Rx — Integrated Sync + Rx)

- SOF detection (11 dominant bits)
- Bit timing recovery (clock division: PIO clock = 32 × bitrate)
- Sampling trigger (sample IRQ at bit center)
- Frame-end detection (10+ recessive bits = end of frame)
- Reads CAN-RX pin, fills FIFO
- Disables sampling during idle (reduces CPU overhead)
- Monitors transmitted frames via readback path

No frame interpretation occurs inside PIO.  
No CRC calculations occur inside PIO.  
No stuffing logic occurs inside PIO.

PIO simply produces a timestamp-correct sequence of sampled bits.

---

## 5. Frame End Detection

A fixed frame length should not be assumed.

CAN frame size varies due to:

- Identifier length
- DLC
- Payload length
- Stuff bits

Instead, frame termination is detected by idle bus duration.

Example:

```
SOF → Data → CRC → ACK → EOF → Intermission
```

The receiver terminates recording when a sufficiently long recessive period is detected.

Conceptually:

```
if recessive_count > threshold:
    frame_complete = true
```

This removes any dependency on frame length calculations.

---

## 6. DMA Buffering

A complete classical CAN frame occupies approximately:

- 100-160 bits (including stuffing)
- 4-6 uint32 words of raw bit storage

Since RP2040 and RP2350 PIO FIFOs may be combined into larger FIFOs and DMA is available, buffering a complete frame is straightforward.

Even at:

- 1 Mbit/s
- 10,000 frames/s

Total data volume remains modest:

≈200 kByte/s

which is negligible for RP2040 and especially RP2350 systems.

---

## 7. Post-Processing

Once a frame is completely captured, software performs:

### Step 1: Destuffing

```
Raw Bits → Destuff → Clean Stream
```

Any stuffing violations may be detected at this point.

### Step 2: CRC Verification

CRC calculation occurs entirely in software.

Advantages:

- Simpler PIO program
- Easier maintenance
- Flexible implementation
- Negligible CPU load

For modern RP2040/RP2350 clock frequencies, CRC computation is effectively free.

### Step 3: Frame Parsing

Software extracts:

- Identifier
- RTR
- IDE
- DLC
- Payload
- CRC

and creates application-level messages.

---

## 8. Error Handling

### Conventional CAN

```
CRC Error → Transmit Error Flag
```

### CAN2040-Lite

```
CRC Error → Discard Frame (RX path)
Error Frame (readback) → Report to Application
ACK Missing (readback) → Report to Application
```

The receiver never interferes with ongoing bus communication.

A received frame is either:

- **Valid** → Deliver to application
- **Invalid** → Drop and report

A transmitted frame is either:

- **ACK received (readback)** → Success
- **Error detected (readback)** → Report error to application
- **No ACK (readback)** → Report NACK to application

Nothing more.

---

## 9. Benefits

### Simplicity

Large portions of CAN controller complexity disappear. Only 1 SM needed.

### Deterministic PIO Design

PIO performs only:

- Synchronization (integrated in rx)
- Sampling
- Buffering

This significantly reduces state-machine complexity.

### Scalability

Large numbers of CAN2040-Lite nodes become feasible.

Example:

```
1 Master (compliant CAN node)
120 CAN2040-Lite Actuators
```

Each actuator can transmit and receive, with the master handling arbitration and error confinement.

### Resource Efficiency

CAN2040-Lite uses 2 SMs on **PIO0**, leaving 2 SMs free on PIO0. FastAccelStepper uses 1 SM on **PIO1**, leaving 3 SMs free on PIO1.

**Total: 5 of 8 available SMs are free across both PIO blocks.**

### Robustness

No possibility exists for a software bug to:

- Corrupt arbitration
- Destroy frames
- Generate false error flags
- Force bus-off conditions

The receiver is fundamentally non-disruptive.

---

## 10. Limitations

The proposal is intentionally not fully CAN compliant.

Limitations include:

- No ACK injection (handled by compliant master node)
- No error frame generation (handled by compliant master node)
- No arbitration participation (handled by compliant master node)
- No fault confinement (handled by compliant master node)
- No automatic retransmission (application handles retries)
- No error state machine (TEC/REC, bus-off)

**At least one standards-compliant CAN controller must be present on the network.**

---

## 11. Application Domains

The concept is particularly attractive for:

- **Kinetic art installations** (suspended-spheres project)
  - Four stepper motors controlled by FastAccelStepper
  - CAN bus for command distribution
  - Full transmit/receive capability on each sphere
- Motion-control slave nodes
- LED installations
- Sensor distribution networks
- Read-only monitoring nodes
- Industrial telemetry receivers

Less suitable for:

- Automotive systems (requires full CAN compliance)
- Safety-critical systems
- Fully standalone CAN networks (no compliant node)

---

## 12. Conclusion

CAN2040-Lite proposes a fundamentally different view of CAN communication.

Instead of implementing a complete CAN controller with 4 SMs, it achieves full transmit/receive capability using only **1 SM** by:

1. Integrating synchronization directly into the receive state machine
2. Using a readback path to verify transmitted frames
3. Deferring all validation to software post-processing

The architecture treats CAN as:

> a self-clocking serial bitstream with delayed software validation

By eliminating the separate sync SM and integrating it into the rx path, and by using readback instead of ACK injection, CAN2040-Lite reduces the state machine footprint from 4 to 1 while maintaining full CAN communication capability.

For the rp2040, CAN2040-Lite uses 2 SMs on PIO0 while FastAccelStepper uses 1 SM on PIO1 — making CAN communication and stepper control simultaneously achievable on the same chip with 5 SMs still free across both PIO blocks.

For RP2040 and RP2350 devices, the combination of PIO, DMA and software post-processing appears sufficient to implement such a transceiver with very modest resource requirements, potentially making CAN communication achievable with only 25% of the complexity of existing full-controller solutions such as can2040.

### Development Approach: Test-Driven Development (TDD)

The implementation follows TDD principles:

1. Test generator creates raw CAN frames with proper bit stuffing
2. CRC calculation (from can2040) validates frame integrity
3. Unit tests verify destuffing, CRC verification, and frame parsing
4. Only then is the PIO capture engine implemented

This ensures correctness before hardware-specific code is written.

### Decision Log

- **2025-11-08:** Confirmed 2-SM allocation for CAN2040-Lite on PIO0 (SM0 rx + SM1 tx), FastAccelStepper uses PIO1
- **2025-11-08:** Match SM merged into Rx SM (no ACK injection needed)
- **2025-11-08:** TX path added (SM1, readback + ACK/ERR evaluation)
- **2025-11-08:** CRC errors reported to application (not silently dropped)
- **2025-11-08:** Another full CAN node on bus handles ACK/error confinement
- **2025-11-08:** Target project: suspended-spheres (kinetic art, 4 steppers)
- **2025-11-08:** Adopted TDD with test generator before PIO implementation
- **2025-11-08:** Renamed from CAN-RX-Lite to CAN2040-Lite (transmit capability)
- **2025-11-08:** Sync integrated into rx SM (2 SMs total: SM0 rx + SM1 tx)
- **2025-11-08:** RX uses relaxed sampling (no phase tracking)
- **2025-11-08:** TX includes bus idle wait before transmission
- **2025-11-08:** FastAccelStepper uses other PIO block (PIO1), not competing with CAN2040-Lite

### Project Name

**CAN2040-Lite** (Minimal CAN Transceiver for RP2040/RP2350)

Alternative names considered: PassiveCAN, ShadowCAN, ObserverCAN, CAN-Sniffer-Core, CAN-Monitor-RX.
