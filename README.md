# CAN2040-Lite

A minimal CAN transceiver architecture for RP2040 and RP2350 microcontrollers using PIO.

## Overview

CAN2040-Lite is a full transmit/receive CAN implementation that uses only **one PIO state machine (SM)** — achieving full CAN communication while using just 25% of the resources required by can2040.

### Key Design Principles

- **Relaxed sampling** — No phase tracking, no resynchronization. Just sample at fixed intervals.
- **1 SM footprint** — Integrates sync + rx + readback into a single state machine.
- **Readback verification** — Transmitted frames are verified by monitoring the CAN-RX line.
- **TDD-driven development** — Test generator creates valid/invalid frames before PIO implementation.

## Hardware Requirements

- Raspberry Pi Pico (rp2040) or Pico 2 (rp2350)
- Standard CAN transceiver (e.g., SN65HVD230, TJA1050)
- At least one other fully CAN-compliant node on the bus (for arbitration, ACK, error handling)

## Project: suspended-spheres

This implementation is developed for the **suspended-spheres** kinetic art installation:

- 4 stepper motors controlled via FastAccelStepper
- CAN bus for command distribution
- Full transmit/receive capability on each sphere
- FastAccelStepper requires 1 PIO SM (2-word OS state machine)
- CAN2040-Lite uses 1 SM, leaving 3 SMs free

### Resource Allocation (rp2040)

| Component | SMs Used | Notes |
|-----------|----------|-------|
| FastAccelStepper OS | 1 (SM3) | 2-word state machine for step generation |
| CAN2040-Lite (rx) | 1 (SM0) | Relaxed sampling, SOF, frame-end, readback |
| **Free** | **3 (SM1, SM2, SM3-unused)** | Available for custom PIO code |

## Documentation

- **[White Paper](docs/white_paper.md)** — Complete design specification, architecture, and rationale
- **[Design Document](docs/design.md)** — SM reduction plan, API design, error handling strategy

## Building the Test Generator

```bash
# Compile test generator
cd extras/tests
gcc -O2 can_test.c -o can_test

# Generate a standard CAN frame
./can_test --std --id 0x123 --data 1234567890ABCDEF

# Generate an extended frame with CRC error
./can_test --ext --id 0x18FEBB11 --data DEADBEEF --invalid crc

# Output as C array (for embedding in unit tests)
./can_test --std --id 0x456 --data AABBCCDD --c

# Raw bit stream output (for PIO verification)
./can_test --std --id 0x789 --data 11223344 --raw
```

## Project Structure

```
can2040-lite/
├── README.md              # This file
├── docs/
│   ├── white_paper.md     # Design specification
│   └── design.md          # SM reduction plan, API design
├── src/                   # Source code (Arduino library)
└── extras/
    └── tests/
        ├── can_test.c     # Frame test generator
        ├── can_test.h     # Test generator header
        └── can_test_units.c  # Unit test framework
```

## Test Generator (can_test.c)

The test generator creates valid and invalid CAN frames with proper bit stuffing and CRC. It's the foundation of TDD for this project.

### Features

- Standard (11-bit) and extended (29-bit) frames
- Remote Transmission Request (RTR) frames
- Configurable payload (0-8 bytes)
- Valid frames with correct CRC and stuffing
- Invalid frames: CRC errors, stuff violations, bit errors
- C array output for embedding in unit tests
- Raw bit stream output for PIO verification

### Usage

```bash
./can_test [options]

Options:
  --std          Standard frame (11-bit ID) [default]
  --ext          Extended frame (29-bit ID)
  --rtr          Remote transmission request
  --id <num>     CAN ID (decimal, default 0x123)
  --data <hex>   Payload hex string (e.g., 1234567890ABCDEF)
  --dlc <n>      Data length code (0-8, default 8)
  --invalid <type> Invalid frame type: crc, stuff, bit
  --raw          Output raw bit stream
  --c            Output C array
  --help         Show help
```

## Development Status

### Completed
- [x] White paper (Draft 0.3)
- [x] Design document (SM reduction plan, relaxed sampling)
- [x] Test generator with TDD framework

### In Progress
- [ ] Destuffing unit tests
- [ ] CRC verification unit tests
- [ ] Frame parsing unit tests

### Planned
- [ ] PIO rx state machine (relaxed sampling)
- [ ] TX path (bus idle wait + bit-stuffed transmit)
- [ ] Readback path (ACK/ERR monitoring)
- [ ] DMA integration
- [ ] Full integration test on rp2040

## License

This project is derived from can2040 (GNU GPLv3). See the original can2040 repository for license details.

## References

- [can2040](https://github.com/KevinOConnor/can2040) — Full CAN implementation for rp2040/rp2350
- [FastAccelStepper](https://github.com/H2coding/FastAccelStepper) — Stepper motor control library for rp2040
- [RP2040 Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [CAN 2.0B Specification](https://www.can-cia.org/can/wp-content/uploads/2023/08/ds-006-002-001-02-v1-can-2-0-part-2-protocol.pdf)