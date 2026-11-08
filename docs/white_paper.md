White Paper
CAN-RX-Lite
A Minimal Passive CAN Receiver Architecture for RP2040 and RP2350 Using PIO

Version: Draft 0.1
 Status: Concept Proposal

Abstract

This paper proposes a lightweight CAN reception architecture for RP2040 and RP2350 microcontrollers that intentionally avoids implementing a complete CAN controller.

The key observation is that many embedded systems require only reliable reception of CAN frames and do not need:

ACK generation
Error frame generation
Arbitration participation
Retransmission
Fault confinement
Bus-off recovery

Examples include:

Kinetic art installations
Broadcast control networks
Distributed actuator systems
Sensor aggregation
Monitoring and diagnostics nodes

By restricting functionality to passive reception, the complexity of a CAN implementation can be reduced substantially. The proposed architecture captures the raw CAN bitstream using PIO state machines and performs all protocol processing after complete frame reception.

1. Motivation

Existing software CAN implementations such as can2040 strive to behave as fully compliant CAN nodes. This requires implementation of numerous real-time protocol mechanisms.

The most challenging parts are not frame decoding itself, but:

ACK slot handling
Error flag insertion
Bit monitoring during transmission
Immediate protocol reactions
State machine transitions between receiver and transmitter modes

These requirements introduce substantial implementation complexity.

In many practical systems these features are unnecessary.

Consider a network consisting of:

Master
  |
  +--- Actuator 1
  +--- Actuator 2
  +--- Actuator 3
  ...
  +--- Actuator N


where:

the master transmits commands,
slaves rarely transmit,
at least one fully compliant CAN controller exists on the bus.

In such systems passive reception is often sufficient.

2. Design Philosophy

The central principle is:

Receive first. Validate later.

Conventional CAN controllers perform protocol analysis while the frame is still being received.

CAN-RX-Lite instead records the entire bitstream and defers all validation until after frame completion.

The bus remains completely unaffected by the receiver.

3. Scope

The receiver implements:

✅ SOF detection

✅ Bit timing recovery

✅ Hard synchronization

✅ Edge-based resynchronization

✅ Bit acquisition

✅ Frame-end detection

✅ Software destuffing

✅ Software CRC verification

✅ Software frame parsing

The receiver does not implement:

❌ ACK generation

❌ Error frame generation

❌ Arbitration

❌ Retransmission

❌ Error counters (TEC/REC)

❌ Error Active / Error Passive

❌ Bus Off state

❌ Transmission

4. Architecture
Layer 1: PIO Capture Engine

PIO performs only real-time functions.

Responsibilities:

SOF Detection
        |
        V
Bit Timing Recovery
        |
        V
Bit Sampling
        |
        V
Frame End Detection
        |
        V
DMA Buffer


No frame interpretation occurs inside PIO.

No CRC calculations occur inside PIO.

No stuffing logic occurs inside PIO.

PIO simply produces a timestamp-correct sequence of sampled bits.

5. Frame End Detection

A fixed frame length should not be assumed.

CAN frame size varies due to:

Identifier length
DLC
Payload length
Stuff bits

Instead, frame termination is detected by idle bus duration.

Example:

SOF
Data
CRC
ACK
EOF
Intermission


The receiver terminates recording when a sufficiently long recessive period is detected.

Conceptually:

if recessive_count > threshold:
    frame_complete = true


This removes any dependency on frame length calculations.

6. DMA Buffering

A complete classical CAN frame occupies approximately:

100-160 bits


including stuffing.

This corresponds roughly to:

4-6 uint32 words


of raw bit storage.

Since RP2040 and RP2350 PIO FIFOs may be combined into larger FIFOs and DMA is available, buffering a complete frame is straightforward.

Even at:

1 Mbit/s
10,000 frames/s


total data volume remains modest:

≈200 kByte/s


which is negligible for RP2040 and especially RP2350 systems.

7. Post-Processing

Once a frame is completely captured, software performs:

Step 1: Destuffing
Raw Bits
     |
     V
Destuff
     |
     V
Clean Stream


Any stuffing violations may be detected at this point.

Step 2: CRC Verification

CRC calculation occurs entirely in software.

Advantages:

Simpler PIO program
Easier maintenance
Flexible implementation
Negligible CPU load

For modern RP2040/RP2350 clock frequencies, CRC computation is effectively free.

Step 3: Frame Parsing

Software extracts:

Identifier
RTR
IDE
DLC
Payload
CRC

and creates application-level messages.

8. Error Handling

Conventional CAN:

CRC Error
    |
    V
Transmit Error Flag


CAN-RX-Lite:

CRC Error
    |
    V
Discard Frame


The receiver never interferes with ongoing bus communication.

A frame is either:

Valid   -> Deliver
Invalid -> Drop


Nothing more.

9. Benefits
Simplicity

Large portions of CAN controller complexity disappear.

Deterministic PIO Design

PIO performs only:

synchronization
sampling
buffering

This significantly reduces state-machine complexity.

Scalability

Large numbers of passive nodes become feasible.

Examples:

1 Master
120 Actuators


Each actuator acts as a frame consumer only.

Robustness

No possibility exists for a software bug to:

corrupt arbitration
destroy frames
generate false error flags
force bus-off conditions

The receiver is fundamentally passive.

10. Limitations

The proposal is intentionally not CAN compliant.

Limitations include:

No ACK generation
No participation in fault confinement
No transmission capability
Dependence on other nodes for ACK handling

At least one standards-compliant CAN controller must be present on the network.

11. Application Domains

The concept is particularly attractive for:

Kinetic art installations
Motion-control slave nodes
LED installations
Sensor distribution networks
Read-only monitoring nodes
Industrial telemetry receivers

Less suitable for:

Automotive systems
Safety-critical systems
Fully standards-compliant CAN networks
12. Conclusion

CAN-RX-Lite proposes a fundamentally different view of CAN reception.

Instead of implementing a complete CAN controller, it treats CAN as:

a self-clocking serial bitstream with delayed software validation.

By eliminating ACK generation, error signaling, arbitration handling and fault confinement, the architecture reduces implementation complexity dramatically while remaining useful for a large class of broadcast-oriented embedded applications.

For RP2040 and RP2350 devices, the combination of PIO, DMA and software post-processing appears sufficient to implement such a receiver with very modest resource requirements, potentially making CAN reception achievable with only a fraction of the complexity of existing full-controller solutions such as can2040.

Proposed Name: CAN-RX-Lite (Passive CAN Reception Architecture)
 Alternative Names: PassiveCAN, ShadowCAN, ObserverCAN, CAN-Sniffer-Core, CAN-Monitor-RX.
