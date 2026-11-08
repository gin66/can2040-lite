# CAN2040-Lite — TDD Implementation Plan

**Status:** Draft 0.3 — Phases 0–3 implemented 2026-11-08 (RP2350 target harness passing)
**Scope:** Audit corrections, test-generator rewrite, host unit tests, PIO
validation, integration testing.
**License:** GPL-3.0-only (derived from can2040). See `LICENSE`.

Legend: `[ ]` todo · `[~]` in progress · `[x]` done · `[!]` blocked

**Hard design constraints:**
- CAN2040-Lite uses **exactly 2 CAN PIO state machines** (SM0 rx + SM1 tx, both
  on PIO0). No phase, test, or optimization may add a third CAN SM.
- The **rx + tx PIO programs together must be ≤ 29 instructions**. PIO0 has 32
  words of instruction memory; 2 words are reserved for the FreeRTOS
  cycle-counter SM (a 2-instruction program), leaving at most 1 spare word.
  This budget is verified with `pioasm` before C5/C6 can pass.

---

## Test Strategy

| Layer | What | Tools |
|-------|------|-------|
| Host unit tests | Frame generator, stuffing, CRC-15, decode pipeline (destuff → CRC → parse) | `cc`/`gcc`, plain C, `tests/pc_based/can_test_units` |
| On-target C++ tests | Public API (`can2040_lite_*`) over USB serial, driven from host | RP2350 Pico 2 (`/dev/cu.usbmodem4101`), PlatformIO + Arduino-Pico |
| PIO TX verification | Bit timing + content on the wire, TX only | Saleae Logic (fx2lafw, 8ch) + `sigrok-cli` |
| Loopback | TX and RX mapped to the same pin; verify readback, ACK/NACK evaluation | Saleae + Pico |
| Bus integration | Real CAN bus with at least one compliant node (ACK, error flags) | CAN transceiver, compliant peer |

Saleae usage follows the FastAccelStepper `salea_based` test approach
(capture with `sigrok-cli`, decode, compare against generated vectors).
Confirm availability first: `sigrok-cli --scan`, `sigrok-cli -L | grep -i can`.

---

## Commit Stopping Points

Commit **only** at the checkpoints below. Each checkpoint must be a coherent,
buildable state: no failing tests, no compiler warnings, no committed binaries
or generated artifacts. Do not commit mid-phase even if some steps are done.

| # | After | Included | Gate (must pass before commit) | Suggested message |
|---|-------|----------|--------------------------------|-------------------|
| C0 | 0.1 | `LICENSE`, SPDX/copyright headers, README license section | `grep -r` finds SPDX in every source file | `license: GPL-3.0-only with can2040 attribution` |
| C1 | 0.2–0.3 | `.gitignore`, README/white paper/design.md corrections | Docs contain no 1-SM/25%/“2 instructions” claims; numbers self-consistent | `docs: correct SM allocation, protocol facts; repo hygiene` |
| C2 | 1.1–1.9 | New generator API + implementation + CLI + split build | `make tool` clean with `-Werror`; `can_gen` builds | `tests: rewrite CAN frame generator with correct CRC-15 and stuffing` |
| C3 | 2.1–2.7 | Host unit tests, README build instructions | `make test` → 0 failed | `tests: host TDD suite green (stuffing, CRC, layout, decode)` |
| C4 | 3.1–3.4 | Firmware skeleton, ported decode pipeline, on-target harness | Host tests still green with shared sources; target build clean | `firmware: API skeleton and ported decode pipeline` |
| C5 | 4.1–4.5 | `rx.pio`, DMA capture, frame-end detection | Saleae RX captures match generator vectors | `pio: RX state machine with edge-resynced sampling and DMA capture` |
| C6 | 5.1–5.4 | `tx.pio`, conflict abort | Saleae TX content/timing matches vectors; `pioasm` confirms rx+tx ≤ 29 instructions | `pio: TX state machine with arbitration abort` |
| C7 | 6.1–6.4 | Loopback, ACK/NACK readback | Loopback and ACK/NACK cases pass | `pio: TX/RX loopback with ACK/NACK readback` |
| C8 | 7.1–7.5 | Bus integration tests, stats | All integration cases pass on hardware | `test: bus integration against compliant CAN node` |
| C9 | 8.1–8.5 | Hardening, docs, release | Soak + final docs consistency | `release: v0.1 hardened CAN2040-Lite` |

Rules:
- One logical change per checkpoint; if a checkpoint's gate fails, fix forward
  and commit at the next checkpoint (never amend a pushed commit).
- Phase 0.1 is a stop point on its own (C0) — licensing must not be bundled
  with behavioral changes.
- Tag C6 as `pio-tx-verified` and C8 as `bus-integration` if release tags are
  desired later.

---

## Phase 0 — Licensing, repo hygiene, documentation corrections

### 0.1 License
- [x] Add full GPL-3.0 text as `LICENSE`
- [x] Add SPDX + copyright headers to all source files
      (`GPL-3.0-only`; can2040-derived portions keep Kevin O'Connor attribution)
- [x] Update `README.md` License section (project is GPL-3.0-only; state which
      parts are can2040-derived)
- [x] Add license note to docs (`white_paper.md`, `design.md`)

> 🛑 **Commit checkpoint C0 — `license: GPL-3.0-only with can2040 attribution`**
> Gate: SPDX header present in every source file; LICENSE committed; no code
> changes bundled.


### 0.2 Repo hygiene
- [x] Keep built binaries out of git (`.gitignore`: `extras/tests/can_test`, `can_test_units`)
- [x] Add build output dir ignores (`build/`, `*.o`, `*.elf`, `*.uf2`, `.pio/`)

### 0.3 Documentation corrections (from audit)
README.md:
- [x] Fix resource story: CAN2040-Lite = **2 SMs on PIO0** (rx + tx), not 1 SM / 25%
- [x] Fix FastAccelStepper facts: **1 SM per stepper**, PIO program occupies the
      **entire 32-word instruction memory** of the block it claims (not
      "2-word OS state machine / 2 instructions")
- [x] Fix suspended-spheres allocation: 4 steppers → 4 SMs + full program memory
      on PIO1; CAN on PIO0; rp2040 has **8 SMs** total (2 blocks × 4)
- [x] Fix repo link: `gap/H2coding` → `https://github.com/gin66/FastAccelStepper`
- [x] Fix build instructions for unit tests (see Phase 1.7)
- [x] Update development status to match reality

docs/white_paper.md:
- [x] Abstract: 2 SMs on PIO0; state the real comparison (can2040: 4 SMs + all
      32 instruction words of one block)
- [x] Hardware constraints: rp2040 = 8 SMs; RP2350 = 3 blocks / 12 SMs
- [x] can2040 table: 4 SMs **sharing one 32-word program**; per-SM words are
      13 (sync) / 2 (rx) / 10 (match) / 7 (tx) = 32, not 38
- [x] Replace “relaxed sampling with no resync” with the correct model:
      fixed-rate sampling **with hard re-sync on bit edges** (still one SM);
      state the drift constraint and the real CAN oscillator tolerances
      (≤0.5% only up to 125 kbit/s; ≤0.15% above)
- [x] Fix SOF claim (“11 dominant bits”); correct is: bus idle = 11 recessive
      bits, then SOF = first dominant bit
- [x] Fix TX/readback mechanism (loop is through the bus, not “inside the
      transceiver”); specify per-bit RX comparison, lost-arbitration abort,
      ACK-slot position known from the transmit stream
- [x] Fix error-flag wording (error flag = 6+ dominant bits; 6–12 depending on
      number of error-active nodes)
- [x] Fix DMA section numbers (frame rate at 1 Mbit/s, 4–5 words per max frame)
- [x] Fix robustness claims (TX can disrupt the bus if it does not monitor RX)
- [x] Fix conclusion/resource percentages
- [x] Append decision-log entries for the audit corrections

docs/design.md:
- [x] Fix can2040 SM/program-word table (32 shared words)
- [x] Fix PIO allocation tables (2 SMs, PIO1 reserved by FastAccelStepper)
- [x] Fix instruction budget table (`SMs used 2`, free SMs 5 across the chip)
- [x] Rewrite relaxed-sampling analysis (edge re-sync + tolerance math)
- [x] Add TX conflict-abort and readback-correlation to pipeline diagram
- [x] Update error-handling table and next-steps list

> 🛑 **Commit checkpoint C1 — `docs: correct SM allocation, protocol facts; repo hygiene`**
> Gate: no 1-SM/25%/“2 instructions”/“4 SMs on rp2040” claims remain; all
> resource tables consistent with 8 SMs / 2 SMs on PIO0 / FAS whole block.


---

## Phase 1 — Host frame generator (`extras/tests`)

Reference facts (verified against can2040 source):

- CAN bit order, standard: `SOF(0) | ID[10:0] | RTR | IDE=0 | r0=0 | DLC[3:0]`
- CAN bit order, extended: `SOF | baseID[10:0] | SRR=1 | IDE=1 | extID[17:0] |
  RTR | r1=0 | r0=0 | DLC[3:0]`
- Data: `dlc` bytes, MSB first; remote frames carry no data
- CRC-15: poly `0x4599` (reflected), computed MSB-first over **unstuffed**
  SOF..last data bit; appended after data
- Stuffing applies to **SOF .. end of CRC sequence**; CRC delimiter, ACK, EOF,
  intermission are unstuffed
- Interframe tail: `CRC delimiter(1) | ACK slot | ACK delimiter(1) | EOF(7×1) |
  IFS(3×1)`

Steps:
- [x] 1.1 Redesign the API into the new layout:
      `src/can2040_lite_proto.h` (bits, `crc_can15_bits`, `bit_stuff`,
      `bit_unstuff`, `can_frame_decode`) and
      `tests/common/can_frame_build.h` (`can_frame_t`, `can_build_info_t`,
      `can_frame_build`)
- [x] 1.2 Implement `bits_get` / `bits_put` / `bits_clear` (MSB-first packing)
- [x] 1.3 Implement bit-level CRC-15; verified equal to can2040's
      `crc_byte` recurrence on byte-aligned streams
- [x] 1.4 Fix `bit_stuff`: return actual bit count; correct stuff-bit
      insertion; overflow detection
- [x] 1.5 Fix `bit_unstuff`: detect 6 identical bits as a stuff violation; does
      not depend on pre-zeroed output; documented semantics (stuffed region
      only); counts a discarded stuff bit as the start of the next run, matching
      the transmitter
- [x] 1.6 Implement `can_frame_build`: exact 2.0B layout, CRC, stuffed prefix,
      unstuffed tail, optional ACK bit; invalidity applied at the right stage
      (`INVALID_CRC` → corrupt CRC bits; `INVALID_BIT` → flip payload bit after
      CRC; `INVALID_STUFF` → flip a stuff bit to create a 6-run)
- [x] 1.7 Implement `can_frame_decode`: destuff, CRC verify, parse ID/DLC/data
      (including extended-ID reassembly)
- [x] 1.8 Rewrite CLI as `extras/tools/can_gen/can_gen.c`
      (summary / `--raw` / `--c`; stuffed length, total length, CRC, decode
      echo; DLC 0–8; optional `--ack`)
- [x] 1.9 Remove the duplicate-`main` problem by splitting the CLI from the
      library (`extras/tools/can_gen` vs `tests/pc_based`); builds documented
      in the root, `tests/pc_based`, and `extras/tools/can_gen` Makefiles

> 🛑 **Commit checkpoint C2 — `tests: rewrite CAN frame generator with correct CRC-15 and stuffing`**
> Gate: `make tool` builds `can_gen` with `-Wall -Wextra -Werror`; it emits
> frames with CRC + correct stuffing region; no unit-test changes needed.


---

## Phase 2 — Host unit tests (golden + property based)

- [x] 2.1 Bit helper tests (get/put roundtrip across byte boundaries)
- [x] 2.2 Stuffing tests: explicit patterns (runs of 0/1, alternating),
      roundtrip stuff→unstuff for std/ext/RTR frames, 6-run detection,
      overflow handling
- [x] 2.3 CRC tests: byte-wise reference (can2040 table) equality on
      byte-aligned streams; frozen golden CRC values for one std and one ext
      frame (std 0x123/DLC8 = 0x1738; ext 0x18FEBB11/DLC8 = 0x4E1A)
- [x] 2.4 Frame layout golden vectors: exact bit positions for SOF/ID/RTR/IDE/
      r0/r1/DLC; DLC sweep 0..8 (std/ext/RTR); length = stuffed + 13
      (CRC delimiter 1 + ACK 1 + ACK delimiter 1 + EOF 7 + IFS 3); no run >5
      in the stuffed region; tail is recessive
- [x] 2.5 Invalid-frame tests: decoder reports CRC mismatch / stuff error
- [x] 2.6 Makefiles added (root, `tests/pc_based`, `extras/tools/can_gen`);
      tests compile production `src/` and are green
- [x] 2.7 Update README build section with the verified commands

> 🛑 **Commit checkpoint C3 — `tests: host TDD suite green (stuffing, CRC, layout, decode)`**
> Gate: `make test` reports 26 passed / 0 failed with `-Wall -Wextra -Werror`,
> and covers std/ext/RTR, DLC 0–8, and all three invalid-frame classes.


---

## Phase 3 — Firmware skeleton

- [x] 3.1 Create `src/` layout and build system: portable core in
      `src/can2040_lite_proto.*`, public API in `src/can2040_lite.*`, target
      harness in `src/pico/main.cpp`, PlatformIO (`platformio.ini`) using the
      Arduino-Pico core on `rpipico2`
- [x] 3.2 Define public API (`can2040_lite_setup`, `_start`, `_transmit`,
      `_check_transmit`, `_get_statistics`, `_stop`, callback config); portable
      skeleton with validation and statistics (PIO backend hooks pending)
- [x] 3.3 Port decode pipeline from host code (destuff → CRC → parse); compile
      the same source in host tests and on target to avoid drift
      (`src/can2040_lite_proto.*` is linked into both)
- [x] 3.4 On-target C++ test harness: run frame vectors + API smoke test on
      RP2350, report over USB serial (`/dev/cu.usbmodem4101`); 7/7 passing

> 🛑 **Commit checkpoint C4 — `firmware: API skeleton and ported decode pipeline`**
> Gate: host tests green using the shared decode sources (26 passed); on-target
> build clean and 7/7 harness vectors pass on RP2350.


---

## Phase 4 — PIO RX state machine + capture

- [x] 4.1 Write `rx.pio`: 32× oversampled raw capture (one 32-bit word per
      bit), SOF poll via `jmp pin`, continuous capture after SOF.  Software
      takes the majority of each word and slices/de-stuffs/decodes.
      **7 PIO instructions** (`pioasm`); see `src/pico/rx.pio`.
- [~] 4.2 DMA from RX FIFO into a raw word buffer (implemented); frame-complete
      IRQ/driver integration still pending (harness currently waits on DMA)
- [~] 4.3 Loop host-generated vectors through a stimulus source; compare
      captured bitstreams (PIO loopback path added, hardware run pending)
- [ ] 4.4 Saleae/sigrok capture of RX pin under realistic bitrates (125k/500k/
      1M), verify sampling positions and frame boundaries
- [ ] 4.5 Robustness: idle suppression, overflow reporting, malformed frames

> 🛑 **Commit checkpoint C5 — `pio: RX state machine with edge-resynced sampling and DMA capture`**
> Gate: captured RX bitstreams equal generator vectors at 125k/500k/1M; frame
> boundaries correct; no missed idle suppression.


---

## Phase 5 — PIO TX state machine + Saleae verification

- [x] 5.1 Write `tx.pio`: 32-bit word shifter, MSB-first, one bit per 33.03125
      PIO cycles on average. **4 PIO instructions**. Bus-idle wait and
      per-bit RX compare are still TODO (software/backend). Combined rx+tx =
      **11 words**, leaving 21 of the 29-word CAN budget unused.
- [ ] 5.2 Abort on conflict: sent recessive but read dominant (lost
      arbitration), stuff/bit error cases; no error-flag injection
- [ ] 5.3 Saleae TX capture (TX only, FastAccelStepper approach); compare
      against generator vectors (content and bit timing)
- [ ] 5.4 Verify back-to-back frames and interframe spacing (≥3 bits)

> 🛑 **Commit checkpoint C6 — `pio: TX state machine with arbitration abort`**
> Gate: Saleae TX capture matches generated frame bit-for-bit; lost-arbitration
> abort verified; no bus disruption when aborted.


---

## Phase 6 — Loopback + ACK/NACK readback

- [ ] 6.1 Map TX and RX to the same pin; capture with Saleae
- [ ] 6.2 RX confirms own transmission (compare against transmitted stream)
- [ ] 6.3 ACK slot evaluation (dominant = ACKed; recessive = NACK); report
      NACK to application
- [ ] 6.4 Error-frame detection (6+ dominant bits) during TX window

> 🛑 **Commit checkpoint C7 — `pio: TX/RX loopback with ACK/NACK readback`**
> Gate: loopback self-reception verified; ACKed, NACKed, and error-frame cases
> each report the expected notification.


---

## Phase 7 — Bus integration (C++ tests over real CAN)

- [ ] 7.1 Two-node setup: CAN2040-Lite + compliant controller (master)
- [ ] 7.2 Master→Lite reception tests (std/ext, DLC sweep, RTR)
- [ ] 7.3 Lite→Master transmission tests (ACKed, NACKed, arbitration loss)
- [ ] 7.4 Error injection: CRC error, stuff error, missing ACK
- [ ] 7.5 Statistics API verification

> 🛑 **Commit checkpoint C8 — `test: bus integration against compliant CAN node`**
> Gate: RX, TX, ACK/NACK, arbitration loss, and error injection all pass on real
> hardware; statistics counters match observed traffic.


---

## Phase 8 — Hardening and release

- [ ] 8.1 Soak test (hours, mixed traffic) on 125k/500k/1M
- [ ] 8.2 Timing margin measurements (transceiver delays, sampling)
- [ ] 8.3 Power-on/off and hot-plug behavior
- [ ] 8.4 Final docs pass (white paper ↔ implementation consistency)
- [ ] 8.5 Tag release, publish GPL-3.0 license files

> 🛑 **Commit checkpoint C9 — `release: v0.1 hardened CAN2040-Lite`**
> Gate: soak test passed; docs consistent; `LICENSE` + SPDX complete; clean
> working tree (no binaries or generated files).


---

## Known audit findings to clear (references)

> The original `extras/tests/can_test.c` and `can_test_units.c` paths below
> have been replaced: protocol core → `src/can2040_lite_proto.*`, frame builder
> → `tests/common/can_frame_build.*`, tests → `tests/pc_based/`,
> CLI → `extras/tools/can_gen/`. The findings are retained as history.

- README:26–38 (FastAccelStepper “2 instructions”, wrong repo, SM arithmetic)
- white_paper.md:18,28,127,157–171,190–197,209,370,397,469 (SM counts, 32-word
  program, topology, sampling math, readback mechanism, percentages)
- design.md:9–14,29–38,90–98 (program words, allocation, budget table)
- can_test.c:84,110–116 (stuff_count unused, stuff returns max_bits)
- can_test.c:214–246 (no CRC emitted, wrong stuffing region)
- can_test.c:301–303 (DLC padding to 64 bits)
- can_test.c:360–389 (invalid-frame corruption applied to stuffed buffer)
- can_test.c:30–31,55–75 (wrong polynomial comment, CRC-5 not in CAN)
- can_test_units.c:9 (broken build command), 82 (wrong data offset),
  135–177 (vacuous CRC tests)
