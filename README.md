# CAN2040-Lite

A minimal CAN transceiver architecture for RP2040 and RP2350 microcontrollers using PIO.

## Overview

CAN2040-Lite is a classic CAN 2.0B data-frame transmit/receive implementation
that uses **two PIO state machines (SMs) on PIO0** — one for receive and one for
transmit. That is half of one PIO block, not a single SM: receive and transmit
cannot share one SM without stalling one direction while the other runs.

### Key Design Principles

- **Fixed-rate sampling with edge resynchronization** — The RX SM samples at a
  fixed rate but hard re-syncs its bit phase on bus edges. There is no separate
  sync SM; the re-sync logic lives in the RX program.
- **2 SM footprint on PIO0** — RX and TX are separate programs on the same PIO
  block. Their programs together are limited to **29 PIO instructions**, leaving
  2 of the block's 32 instruction words for the FreeRTOS cycle-counter SM.
- **Readback verification** — Transmitted bits are compared against the bus
  (CAN-RX) bit by bit so lost arbitration, ACK/NACK, and error frames can be
  detected.
- **TDD-driven development** — The test generator creates valid/invalid frames
  before the PIO implementation is written.

## Hardware Requirements

- Raspberry Pi Pico (rp2040) or Pico 2 (rp2350)
- Standard CAN transceiver (e.g., SN65HVD230, TJA1050)
- At least one other fully CAN-compliant node on the bus (for arbitration
  and error handling). ACK generation is implemented as the TX-slot helper
  in `docs/ack_generation.md`; its on-target verification is still pending.

## Restrictions

CAN2040-Lite is **not a full CAN node**. It is a lightweight data-frame
transceiver. ACK generation is an edge-locked pulse on the TX state machine
(`docs/ack_generation.md`); the host-side schedule and cycle model pass,
while the on-target check needs the RP2350 and a logic analyzer. It
deliberately omits the protocol mechanisms that require
real-time error participation:

- **No error-flag generation** — it only observes 6+ dominant bits and reports
  the error; it never injects an error flag.
- **No error counters / fault confinement** — no TEC/REC, Error Active /
  Error Passive, or bus-off state.
- **No automatic retransmission** — the application decides whether to retry.
- **No full arbitration participation** — TX aborts only when it sent
  recessive and the bus is dominant. The bit after the CRC is driven and
  not compared; ACK and end of frame are the recessive park. The node does
  not send an error flag.
- **No RTR reply** — an RTR is transmitted and a received RTR is delivered
  with the RTR flag set. The node does not answer a received RTR with a
  data frame.
- **No CAN FD** — classic CAN 2.0B only.
- **No programmable bit timing** — fixed 32× oversampling; no SJW or
  TSEG1/TSEG2 configuration.
- **No hardware acceptance filters** — filtering is done in software.
- **No on-chip transceiver** — an external CAN transceiver is required.

A CAN2040-Lite network must contain **at least one compliant CAN node** for
error confinement. CAN2040-Lite now drives ACKs on its own; that helper's
on-target verification is pending.

Current implementation notes (not protocol requirements): the target backend
services the RX FIFO from its not-empty IRQ, statistics capture RX/TX totals,
and frames are not timestamped. The ACK helper swaps the TX instruction image;
its schedule and cycle model are host-tested, and on-target verification is
pending.

## Project: suspended-spheres

This implementation is developed for the **suspended-spheres** kinetic art installation:

- 4 stepper motors controlled via FastAccelStepper
- CAN bus for command distribution
- Full transmit/receive capability on each sphere
- FastAccelStepper uses **one PIO SM per stepper** and loads its program into
  (and claims) the **entire instruction memory** of the PIO block it uses
- CAN2040-Lite uses 2 SMs on the other PIO block
- The FreeRTOS cycle-counter SM uses a 2-instruction program on PIO0, so the
  CAN rx+tx programs must fit in the remaining 29 words

### Resource Allocation (rp2040)

The rp2040 has **8 state machines total** (2 PIO blocks × 4 SMs).

| Component | PIO block | SMs Used | Program words | Notes |
|-----------|-----------|----------|---------------|-------|
| CAN2040-Lite (rx + tx) | PIO0 | 2 (SM0, SM1) | ≤ 29 combined | Hard budget |
| FreeRTOS cycle counter | PIO0 | 1 | 2 | Reserved |
| FastAccelStepper (4 axes) | PIO1 | 4 (all) | whole block | One SM per stepper |
| **Free** | PIO0 | **1** | **≤ 1** | Available for custom PIO code |

With a single FastAccelStepper axis the FAS footprint is 1 SM (plus the block's
program memory), leaving 4 of the 8 SMs free across the chip.

## Documentation

- **[White Paper](docs/white_paper.md)** — Complete design specification, architecture, and rationale
- **[Design Document](docs/design.md)** — SM reduction plan, API design, error handling strategy
- **[TDD Plan](docs/tdd_plan.md)** — Implementation plan and verification checkpoints

## Building and Running

The Makefiles wire the portable production code in `src/` into both the host
tests and the CLI tool, so the PC tests exercise the same protocol core that
runs on-target.

```bash
# Build everything and run the PC-based tests
make

# Or individually
make test      # extras/tests/pc_based/can_test_units
make tool      # extras/tools/can_gen/can_gen
make clean
```

Target build/flash (RP2350 Pico 2, PlatformIO + Arduino-Pico):

```bash
pio run                    # build
pio run -t upload          # flash (1200-baud touch -> BOOTSEL -> picotool)
pio device monitor         # 115200 baud USB serial
# Send any character to re-run the on-target vectors.
```

Direct host builds (equivalent to what the Makefiles do):

```bash
# PC-based host tests (production src/ + host-only frame builder)
cc -O2 -std=c11 -Wall -Wextra -Werror \
   -Isrc -Iextras/tests/common \
   -o extras/tests/pc_based/can_test_units \
   extras/tests/pc_based/can_test_units.c \
   src/*.c extras/tests/common/*.c

# Frame generator CLI
cc -O2 -std=c11 -Wall -Wextra -Werror \
   -Isrc -Iextras/tests/common \
   -o extras/tools/can_gen/can_gen \
   extras/tools/can_gen/can_gen.c \
   src/*.c extras/tests/common/*.c
```

Example frame generation:

```bash
./extras/tools/can_gen/can_gen --std --id 0x123 --data 123456789ABCDEF0
./extras/tools/can_gen/can_gen --ext --id 0x18FEBB11 --data DEADBEEF --invalid crc
./extras/tools/can_gen/can_gen --std --id 0x456 --data AABBCCDD --c
./extras/tools/can_gen/can_gen --std --id 0x789 --data 11223344 --raw
```

## Project Structure

```
can2040-lite/
├── README.md              # This file
├── Makefile               # Host build/test entry point
├── docs/
│   ├── white_paper.md     # Design specification
│   ├── design.md          # SM reduction plan, API design
│   ├── ack_generation.md  # ACK helper design and status
│   └── tdd_plan.md        # TDD implementation plan
├── platformio.ini         # RP2350 target build (Arduino-Pico core)
├── src/
│   ├── can2040_lite_proto.c/.h  # Shared protocol core (host + target)
│   ├── can_edge_sm.c/.h         # Edge-RX ISR state machine (host-tested)
│   ├── can2040_lite.c/.h        # Public API, transmit queue, frame packer
│   ├── can2040.h                # Source-compatible can2040 aliases
│   └── pico/
│       ├── can2040_lite_pico.cpp/.h  # PIO/DMA backend
│       ├── rx_pair.pio, tx.pio       # Live PIO programs (7 + 14 words)
│       ├── ack.pio                   # ACK helper overlay (13 words)
│       └── rx.pio                    # Legacy 32x oversampled RX (7 words)
└── extras/
    ├── tests/
    │   ├── common/
    │   │   └── can_frame_build.c/.h # Host-only frame builder
    │   ├── pc_based/
    │   │   ├── can_test_units.c     # PC-based unit tests
    │   │   └── Makefile
    │   └── on_target/
    │       └── main.cpp             # RP2350 vector/loopback harness
    └── tools/
        └── can_gen/
            ├── can_gen.c            # Raw/C frame-vector generator CLI
            └── Makefile
```

## Frame Generator (can_gen)

The test generator creates valid and invalid CAN frames with proper bit stuffing and CRC. It's the foundation of TDD for this project.

### Features

- Standard (11-bit) and extended (29-bit) frames
- RTR vectors (the node transmits an RTR and does not answer one)
- Configurable payload (0-8 bytes)
- Valid frames with correct CRC-15 and stuffing
- Invalid frames: CRC errors, stuff violations, bit errors
- C array output for embedding in unit tests
- Raw bit stream output for PIO verification

### Usage

```bash
./extras/tools/can_gen/can_gen [options]

Options:
  --std          Standard frame (11-bit ID) [default]
  --ext          Extended frame (29-bit ID)
  --rtr          Generate an RTR vector (the node does not answer an RTR)
  --id <num>     CAN ID (decimal, default 0x123)
  --data <hex>   Payload hex string (e.g., 1234567890ABCDEF)
  --dlc <n>      Data length code (0-8, default 8)
  --invalid <type> Invalid frame type: crc, stuff, bit
  --raw          Output raw bit stream
  --c            Output C array
  --help         Show this help
```

## Development Status

### Completed
- [x] White paper (Draft 0.4)
- [x] Design document (SM reduction plan, edge-resynced sampling)
- [x] Test generator with TDD framework
- [x] Host unit tests (stuffing, CRC-15, frame layout, decode)

- [x] Public API, transmit queue, and frame packer (`src/can2040.h`).
      Host-tested. Not run on hardware.
- [x] On-target RP2350 harness (vectors + API + loopback)
- [x] PIO RX and one-shot full-message TX: 7 + 14 = 21 instructions, inside
      the 29-word budget.  The 32x oversampler remains as `rx.pio`.
      The current TX program is not tested on hardware.
- [x] Edge-pair RX program (`rx_pair.pio`, 7 words, two edges per FIFO word)
      wired to the FIFO-not-empty IRQ, with a host-tested pair decoder feeding
      the ISR state machine.  Not tested on hardware.
- [x] TX/RX loopback with ACK and NACK readback
- [x] Saleae verification at 125k, 500k and 1M; back-to-back frames (18-bit gap)

### Planned
- [ ] ACK helper on-target verification and go-window measurement
      (`docs/ack_generation.md`)
- [ ] Error-frame injection and detection
- [ ] Real-bus integration with a compliant peer/transceiver
- [ ] Soak test and release

## License

CAN2040-Lite is licensed under the GNU General Public License v3.0 only
(`GPL-3.0-only`). See [`LICENSE`](LICENSE) for the full text.

The CAN protocol handling approach and CRC-15 reference implementation are
derived from [can2040](https://github.com/KevinOConnor/can2040),
Copyright (C) 2021 Kevin O'Connor, also licensed under GPL-3.0. All source
files carry an SPDX identifier and copyright notice.

## References

- [can2040](https://github.com/KevinOConnor/can2040) — Full CAN implementation for rp2040/rp2350
- [FastAccelStepper](https://github.com/gin66/FastAccelStepper) — Stepper motor control library for rp2040
- [RP2040 Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [CAN 2.0B Specification](https://www.can-cia.org/can/wp-content/uploads/2023/08/ds-006-002-001-02-v1-can-2-0-part-2-protocol.pdf)
