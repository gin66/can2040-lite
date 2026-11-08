# White Paper

## CAN2040-Lite
A Minimal CAN Transceiver Architecture for RP2040 and RP2350 Using PIO

**Version:** Draft 0.4  
**Status:** Design Specification (TDD-driven)  
**Last Updated:** 2026-11-08  
**Project:** suspended-spheres (Kinetic Art Installation)  
**Target Hardware:** Raspberry Pi Pico (rp2040)  
**License:** GPL-3.0-only (derived from can2040, Copyright (C) 2021 Kevin O'Connor)

---

## Abstract

CAN2040-Lite is a lightweight CAN transceiver architecture for RP2040 and RP2350 microcontrollers that achieves full CAN frame transmission and reception using only **two PIO state machines (SMs) on one PIO block**.

The key insight is that synchronization logic can be integrated directly into the receive state machine, eliminating the need for a separate sync SM. Combined with a readback path that compares transmitted bits against the bus, the architecture provides full transmit/receive capability while using only 2 of the 4 SMs in **PIO0**, with the two programs fitting in **29 of the block's 32 instruction words** so 2 words remain for the FreeRTOS cycle counter. **PIO1 is left entirely free** for FastAccelStepper.

Examples include:

- Kinetic art installations (suspended-spheres project)
- Broadcast control networks with readback verification
- Distributed actuator systems
- Sensor aggregation with status reporting
- Monitoring and diagnostics nodes

By integrating sync into the rx path and adding a readback transmit path,
CAN2040-Lite achieves full CAN communication with 2 SMs instead of can2040's 4,
and without consuming can2040's complete 32-word PIO program memory.

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

### Hardware Constraints

- **rp2040:** 2 PIO blocks × 4 SMs = **8 state machines total**.
- **RP2350:** 3 PIO blocks × 4 SMs = **12 state machines total**.

The suspended-spheres project uses:

- **PIO0**: CAN2040-Lite — 2 SMs (SM0 rx + SM1 tx) and the FreeRTOS
  cycle-counter SM — 1 SM free. The CAN programs must use ≤ 29 of the 32
  instruction words; the FreeRTOS counter uses 2.
- **PIO1**: [FastAccelStepper](https://github.com/gin66/FastAccelStepper) —
  one SM per stepper, and the FAS PIO program occupies (and the driver claims)
  the **entire instruction memory of the PIO block**. With 4 steppers this uses
  all 4 SMs on PIO1.
- **FreeRTOS** cycle counter: 1 SM with a 2-instruction program on PIO0
  (its 2 words are included in the PIO0 program budget)

This is a clean separation: CAN and stepper control use separate PIO blocks,
avoiding resource conflicts.

### SM Reduction Plan (from can2040's 4 SMs)

can2040 uses 4 SMs, and all four programs share **one 32-word PIO instruction
memory** (the complete program memory of a single PIO block):

| SM | Name | Purpose | Program words |
|----|------|---------|---------------|
| 0 | sync | Bit timing recovery, SOF/EOF detection, idle detection | 13 |
| 1 | rx | Reads CAN-RX pin, fills FIFO | 2 |
| 2 | match | CRC match detection → ACK injection signaling | 10 |
| 3 | tx | Bit-stuffed transmission, line arbitration | 7 |

**Total: 4 SMs sharing one 32-word program memory (13 + 2 + 10 + 7 = 32).**

### Full rp2040 SM Allocation (suspended-spheres project)

#### PIO0 — CAN2040-Lite

| SM | Used By | Purpose | Program words |
|----|---------|---------|---------------|
| 0 | **CAN2040-Lite** | RX — 32× oversampled capture, SOF poll, DMA; software slicing | 7 |
| 1 | **CAN2040-Lite** | TX — 32-bit word shifter, bit-stuffed stream | 4 |
| 2 | **FreeRTOS** | 2-instruction cycle counter | 2 |
| 3 | **UNUSED** | Free for custom PIO | — |

**PIO0 instruction memory: RX + TX ≤ 29 words, FreeRTOS 2 words → ≤ 31 of 32 used.**

#### PIO1 — FastAccelStepper (4 axes)

| SM | Used By | Purpose | Program words |
|----|---------|---------|---------------|
| 0-3 | **FastAccelStepper** | One SM per stepper axis; all share the block's program | up to 32 (whole block) |

With a single axis, FAS uses 1 SM on PIO1 but still claims the whole block's
program memory; 3 SMs on PIO1 then remain unclaimed.

**CAN2040-Lite: 2 SMs on PIO0, 11 PIO program words (hard limit 29)**  
**FreeRTOS counter: 1 SM on PIO0, 2 program words**  
**FastAccelStepper (suspended-spheres, 4 axes): 4 SMs on PIO1, whole 32-word program**  
**Free in suspended-spheres: 1 SM on PIO0; 0 SMs on PIO1**

### Key Insight: Edge-Resynchronized Sampling in Rx

In can2040, a separate sync SM generates sample IRQs at the bit center, which
the rx SM reads. This requires two SMs working in concert, with precise phase
tracking and resynchronization on every bit transition.

CAN2040-Lite folds that function into the RX SM (SM0) and uses a simpler
variant: **fixed-rate sampling with hard re-synchronization on bus edges**.

- The RX SM samples the CAN-RX pin 32 times per bit (one 32-sample word per
  bit) into the RX FIFO/DMA.
- It does **not** implement a full phase-locked loop or a programmable
  time-quanta phase segment inside PIO.
- The portable software takes the majority of each 32-sample word as the bit
  value and performs edge-resynchronized slicing, so clock drift cannot
  accumulate across the whole frame; any residual error is bounded by the time
  between re-sync edges (at most one bit time in the worst case).
- Bus idle is **11 consecutive recessive bits**; SOF is the **first dominant
  bit** after idle.
- Frame end is detected from the recessive EOF (7 bits) plus IFS (3 bits),
  i.e. a sustained recessive run rather than a fixed frame length.

This removes the separate sync SM and its inter-SM handshake while keeping the
sampling robust. It is *not* “sample at fixed intervals with no re-sync” — an
unstuffed CAN stream can contain long runs of identical bits, so without edge
re-sync even a small clock mismatch accumulates over a full frame.

**Why this works — the real tolerance numbers:** CAN does not permit ±0.5%
oscillator error at every bitrate. The permitted oscillator tolerance is
**≤ 0.5% only up to 125 kbit/s**, and tightens to **≤ 0.15% above
125 kbit/s**. Edge re-sync bounds the accumulation to the longest interval
between re-sync edges, which keeps even the tighter high-speed tolerance well
inside the sampling margin. Receivers must additionally tolerate the
transmitter's bit-rate error; because re-sync happens at every edge, the
worst-case sample position stays within a small fraction of a bit.

**Trade-off:** A frame with no edges for a long time (a long stuffed run) is
still sampled open-loop between edges, so the design relies on the CAN
requirement that stuffing inserts an edge at least every 5 bits inside the
stuffed region. Outside that region, EOF/IFS are all recessive and a small
mis-sample there is harmless.

### Transmit Path (SM1): Readback Verification

CAN2040-Lite is not passive-only. It transmits frames and verifies reception via a readback path.

The TX SM (SM1) handles transmission with bus idle wait:

1. **Bus Idle Wait:** Before transmitting, the TX path waits for bus idle (11+ consecutive recessive bits — intermission). This is required by the CAN protocol for proper line arbitration.

2. **Transmission:** The application queues a frame for transmission. Software handles bit-stuffing and transmits the frame on the CAN-TX pin.

3. **Readback:** Simultaneously, the RX SM (SM0) monitors the CAN-RX pin and
   compares it against the bit being transmitted, **bit by bit**. The signal
   path is through the bus, not “inside the transceiver”: the TX pin drives the
   transceiver, the transceiver drives the bus, and the same transceiver's RX
   pin reflects the resulting bus level (including other nodes' dominant bits).

4. **Lost-Arbitration Abort:** If the transmitter sends a recessive bit but
   reads back a dominant bit, it has lost arbitration (or another node is
   driving the bus). The TX SM aborts immediately without injecting an error
   flag, so the winning frame is not disturbed.

5. **ACK Evaluation:** The position of the ACK slot is known from the
   transmit stream itself (it follows the 15-bit CRC and CRC delimiter), so no
   separate frame parse is needed. A dominant bit in the ACK slot is an ACK; a
   recessive bit is a NACK. The following ACK delimiter (recessive) is checked
   for consistency.

6. **Error Evaluation:** An error flag is **6 or more consecutive dominant
   bits** (6–12 depending on how many error-active nodes are signalling). If
   such a run is detected on RX during the transmit window, the error is
   reported to the application.

7. **No Injection:** CAN2040-Lite does NOT inject ACK bits or error flags — it
   only observes them. A compliant CAN node on the bus handles these.

### PIO Capture Engine

PIO performs only real-time functions.

#### SM0 (Rx — Integrated Sync + Rx)

- Idle detection (11 consecutive recessive bits)
- SOF detection (first dominant bit after idle)
- Fixed-rate sampling (PIO clock = 32 × bitrate) with hard re-sync on edges
- Frame-end detection (EOF 7 recessive + IFS 3 recessive)
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

Instead, frame termination is detected by the recessive end-of-frame sequence.

Example:

```
SOF → Data → CRC → ACK → EOF (7 recessive) → IFS (3 recessive)
```

The receiver terminates recording when the EOF plus intermission recessive run
is seen. Bus idle itself is 11 recessive bits; SOF is the first dominant bit
that follows idle.

Conceptually:

```
if recessive_count > threshold:
    frame_complete = true
```

This removes any dependency on frame length calculations.

---

## 6. DMA Buffering

A complete classical CAN frame occupies approximately:

- 47 bits minimum (standard, DLC 0) up to ~164 bits maximum (extended,
  DLC 8, worst-case stuffing)
- **4–5 uint32 words** of raw bit storage for a maximum-length frame

Since RP2040 and RP2350 PIO FIFOs may be combined into larger FIFOs and DMA is available, buffering a complete frame is straightforward.

At 1 Mbit/s the theoretical ceiling is roughly **6,000–8,000 frames/s**
depending on frame length and stuffing; a maximum-length extended frame takes
about 164 bit times, so about 6,000 frames/s. Raw bit capture is on the order
of 100–170 kByte/s at that rate, which is negligible for RP2040 and especially
RP2350 systems.

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

Large portions of CAN controller complexity disappear. CAN2040-Lite uses 2 SMs
(rx + tx) on one PIO block.

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

CAN2040-Lite uses 2 SMs on **PIO0** and the FreeRTOS cycle counter uses a third
(2-word program), leaving 1 SM free on PIO0. In the suspended-spheres project
FastAccelStepper uses all 4 SMs (and the program memory) of **PIO1**.

**Suspended-spheres total: 1 of 8 SMs is free (on PIO0). With a single
FastAccelStepper axis, 4 of 8 SMs are free.**

### Robustness

The **receive path** is fundamentally non-disruptive: it only samples, so a
software bug in RX cannot corrupt arbitration, destroy frames, generate false
error flags, or force bus-off.

The **transmit path is different**: it actively drives the bus. A buggy or
misconfigured transmitter *can* disrupt the bus (for example by transmitting
outside a valid window or failing to stop after losing arbitration), so the TX
SM compares every transmitted bit against the RX pin and aborts on any
mismatch. It must still be treated as a potential bus participant, not as a
passive observer.

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

Instead of implementing a complete CAN controller with 4 SMs, it achieves
full transmit/receive capability using **2 SMs on one PIO block** by:

1. Integrating edge re-synchronization directly into the receive state machine
2. Using a per-bit readback path to verify transmitted frames and detect lost
   arbitration
3. Deferring all validation to software post-processing

The architecture treats CAN as:

> a self-clocking serial bitstream with delayed software validation

By eliminating the separate sync SM and integrating re-sync into the rx path,
and by using readback instead of ACK injection, CAN2040-Lite reduces the state
machine footprint from 4 SMs to 2 while maintaining full CAN communication
capability. It also avoids can2040's need to occupy a block's entire 32-word
instruction memory: the two CAN2040-Lite programs are much smaller, so other
programs can coexist on PIO0.

For the rp2040, CAN2040-Lite uses 2 SMs on PIO0. FastAccelStepper, when used
with four axes, uses all 4 SMs and the program memory of PIO1; that still
leaves 1 SM free on PIO0 after the FreeRTOS cycle counter. With a single
stepper axis, 4 of the chip's 8 SMs remain free.

For RP2040 and RP2350 devices, the combination of PIO, DMA and software
post-processing appears sufficient to implement such a transceiver with very
modest resource requirements — half the SMs of a full-controller solution such
as can2040, on one PIO block.

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
- **2025-11-08:** RX uses fixed-rate sampling (later corrected: with edge re-sync)
- **2025-11-08:** TX includes bus idle wait before transmission
- **2025-11-08:** FastAccelStepper uses other PIO block (PIO1), not competing with CAN2040-Lite
- **2026-11-08 (audit):** Corrected can2040 footprint: 4 SMs sharing one
  32-word program (13 + 2 + 10 + 7), not 38 instructions.
- **2026-11-08 (audit):** RX model corrected from “relaxed sampling, no re-sync”
  to fixed-rate sampling **with hard edge re-sync**; oscillator tolerance is
  ≤0.5% only up to 125 kbit/s and ≤0.15% above.
- **2026-11-08 (audit):** Corrected SOF/idle description: idle = 11 recessive
  bits, SOF = first dominant bit.
- **2026-11-08 (audit):** Corrected readback path: comparison is per bit
  through the bus, with lost-arbitration abort and ACK-slot position known from
  the transmit stream.
- **2026-11-08 (audit):** Corrected error-flag wording (6+ dominant bits,
  6–12 depending on error-active nodes) and DMA throughput numbers.
- **2026-11-08 (audit):** Clarified that RX is non-disruptive but TX drives the
  bus and can disrupt it if readback monitoring fails.
- **2026-11-08 (budget):** Hard PIO budget: rx + tx programs ≤ 29 instructions;
  2 of PIO0's 32 words are reserved for the FreeRTOS 2-instruction cycle
  counter SM.

### Project Name

**CAN2040-Lite** (Minimal CAN Transceiver for RP2040/RP2350)

Alternative names considered: PassiveCAN, ShadowCAN, ObserverCAN, CAN-Sniffer-Core, CAN-Monitor-RX.
