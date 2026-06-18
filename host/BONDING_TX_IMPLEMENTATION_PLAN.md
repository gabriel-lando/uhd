# USRP Bonding TX Implementation Plan

## Multi-Device Synchronized Transmission for USB-Based USRPs

**Project Goal**: Extend the existing RX-only bonding mechanism with a reusable `bonded_transmitter` class that provides synchronized multi-device TX streaming, enabling full-duplex bonded operation and validated wideband data transfer across bonded channels.

**Prerequisite**: Phases 1–5 of the RX-only plan (`BONDING_IMPLEMENTATION_PLAN_FLEXIBLE.md`) are complete. The `bonded_receiver` class, frequency planning, and spectrum stitching are implemented and validated.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Architecture & Design Decisions](#architecture)
3. [Implementation Phases](#implementation-phases)
4. [API Design](#api-design)
5. [Testing & Validation](#testing-validation)
6. [Future Work](#future-work)

---

## 1. Executive Summary

### Objective

Add a generic, reusable `bonded_transmitter` C++ class to `lib/usrp/bonded/` that mirrors the `bonded_receiver` design: N devices, configurable via serials/rate/freq_plan/gain, supporting burst and continuous TX modes with synchronized timed transmission across all devices.

Then validate the complete TX+RX bonded system with a full-duplex demonstration that proves end-to-end data integrity over bonded wideband channels.

### Core Design Principles (same as RX plan)

- **Minimal upstream diff**: No additional upstream UHD files modified; all TX bonding logic in new files under `lib/usrp/bonded/`
- **No composite streamer**: Each device keeps its own `tx_streamer`; coordination happens at the `bonded_transmitter` layer
- **Reusable engine**: `bonded_transmitter` works identically from standalone tools, Python, or future GNURadio blocks
- **Configuration via device args**: Same serial/clock/time args as `bonded_receiver`
- **No over-engineering**: Simple parallel `send()` threads with timed metadata; no abstract TX strategy classes

### Key Deliverables

- ✅ `bonded_transmitter` class (burst + continuous TX modes)
- ✅ Shared device support (TX and RX streamers from the same `multi_usrp` instances)
- ✅ Simple packet codec with CRC-32 validation (for the demo)
- ✅ Full-duplex bonded example (`bonded_usrp_fullduplex.cpp`)
- ✅ Unit tests for TX alignment and packet codec
- ✅ Hardware validation with data integrity verification

### Hardware (same 4× B210 setup)

| Device   | Serial    | Full-Duplex Ports            |
| -------- | --------- | ---------------------------- |
| Device 0 | `30B56D6` | TX: TX/RX port, RX: RX2 port |
| Device 1 | `30DBC3C` | TX: TX/RX port, RX: RX2 port |
| Device 2 | `30DBC3D` | TX: TX/RX port, RX: RX2 port |
| Device 3 | `30EDB63` | TX: TX/RX port, RX: RX2 port |

**Full-duplex on B210**: Each device supports simultaneous TX (TX/RX port) and RX (RX2 port) in single-channel mode at up to ~56 MHz per direction.

**Bonded aggregate bandwidth**: 2 devices × 40 MHz with 15% overlap = ~68 MHz usable bandwidth per direction.

### Development Workflow

**Critical rule**: Same as RX plan. The AI assistant cannot compile or run code. All builds require `sudo` and must be executed manually by the user.

Workflow for every code change:

1. AI writes/modifies source files
2. AI asks user to compile, providing the exact build command
3. User compiles and reports back: success or error output
4. If compilation succeeds and runtime testing is needed, AI provides the exact command line
5. User runs the command and pastes the output back
6. AI analyzes the output and iterates if needed

**Never skip the user-in-the-loop step.**

---

## 2. Architecture & Design Decisions

### 2.1 TX Mirrors RX — Same Structural Pattern

The `bonded_transmitter` follows the exact same pattern as `bonded_receiver`:

| Aspect            | `bonded_receiver`                         | `bonded_transmitter`                     |
| ----------------- | ----------------------------------------- | ---------------------------------------- |
| Device management | N `multi_usrp` instances (one per serial) | Same (shared when full-duplex)           |
| Sync setup        | `setup_bonded_sync()` + PPS alignment     | Same                                     |
| Streamer creation | `get_rx_stream()` per device              | `get_tx_stream()` per device             |
| Parallel threads  | One recv thread per device                | One send thread per device               |
| Timing            | Timed stream commands                     | Timed `tx_metadata_t` (start_of_burst)   |
| Error tracking    | Overflow counting per device              | Underrun counting via `async_metadata_t` |
| Frequency plan    | Per-device center freq via `freq_plan`    | Same                                     |
| Continuous mode   | Ring buffers filled by recv threads       | TX queue consumed by send threads        |
| Burst mode        | `capture_burst(nsamps, delay)`            | `send_burst(data, delay)`                |

### 2.2 Shared Device Instances for Full-Duplex

When an application needs both TX and RX on the same physical devices (full-duplex), it must use **one `multi_usrp` per serial** shared between `bonded_transmitter` and `bonded_receiver`. This avoids conflicting clock/time source settings and double PPS alignment.

**Solution**: Both classes accept an optional `std::vector<multi_usrp::sptr>` of pre-opened devices. When provided, they skip `_open_devices()` and `_apply_sync()` / `_align_time()`, and only create their respective streamers.

```
┌────────────────────────────────────────────────────────────────┐
│  Application (e.g., bonded_usrp_fullduplex.cpp)                │
│                                                                │
│  1. Opens N multi_usrp instances                               │
│  2. Applies sync + PPS alignment (once)                        │
│  3. Passes shared devices to both TX and RX engines            │
└─────────────────────┬──────────────────────┬───────────────────┘
                      │                      │
        ┌─────────────▼──────────┐  ┌───────▼─────────────────┐
        │  bonded_transmitter    │  │  bonded_receiver         │
        │  (creates tx_streamers)│  │  (creates rx_streamers)  │
        │  (manages send threads)│  │  (manages recv threads)  │
        └─────────────┬──────────┘  └───────┬─────────────────┘
                      │                      │
              ┌───────▼──────────────────────▼────────┐
              │  Shared multi_usrp instances           │
              │  (one per physical device/serial)      │
              │  tx_streamer + rx_streamer coexist     │
              └───────────────────────────────────────┘
```

### 2.3 TX Timing Model

Unlike RX (where alignment is done after-the-fact by comparing timestamps), TX requires **pre-scheduled timing**:

- **Burst mode**: All devices receive IQ data with `tx_metadata_t.has_time_spec = true` and the same `time_spec`. The FPGA buffers samples and begins transmission at the specified time across all devices simultaneously.
- **Continuous mode**: Initial burst starts with timed metadata; subsequent `send()` calls are untimed (streaming mode). The FPGA maintains continuous flow as long as samples arrive before the TX buffer empties.

**Underrun handling**: Each device has an async metadata listener thread that polls `recv_async_msg()` to detect `ERROR_CODE_UNDERFLOW`. Underrun events are counted per-device and reported in `tx_result`.

### 2.4 Wideband TX with Frequency Plan

For bonded wideband TX (e.g., 80 MHz across 2 devices at 40 MHz each):

- The application generates wideband IQ data
- A **band splitter** (inverse of the spectrum stitcher) separates the wideband signal into per-device sub-bands with appropriate overlap/filtering
- Each device transmits its assigned sub-band at its center frequency
- The `bonded_transmitter` handles the per-device tuning and synchronized start

The band splitter is a DSP utility, kept separate from `bonded_transmitter` (same philosophy as spectrum stitcher being separate from `bonded_receiver`).

### 2.5 File Layout (additions to existing structure)

```
lib/usrp/bonded/
├── bonded_usrp.hpp              (existing — sync setup helper)
├── bonded_usrp.cpp              (existing)
├── bonded_receiver.hpp          (existing — RX engine)
├── bonded_receiver.cpp          (existing)
├── bonded_transmitter.hpp       (NEW — TX engine API)
├── bonded_transmitter.cpp       (NEW — TX engine implementation)
├── bonded_device_pool.hpp       (NEW — shared device open/sync helper)
├── bonded_device_pool.cpp       (NEW — implementation)
├── packet_codec.hpp             (NEW — frame encode/decode with CRC)
├── packet_codec.cpp             (NEW — implementation)
├── band_splitter.hpp            (NEW — wideband→per-device IQ split)
├── band_splitter.cpp            (NEW — implementation)
├── frequency_plan.hpp           (existing)
├── frequency_plan.cpp           (existing)
├── spectrum_stitcher.hpp        (existing)
├── spectrum_stitcher.cpp        (existing)
└── CMakeLists.txt               (updated — add new sources)

examples/
├── bonded_usrp_rx.cpp           (existing)
├── bonded_usrp_continuous.cpp   (existing)
├── bonded_usrp_wideband.cpp     (existing)
├── bonded_usrp_tx.cpp           (NEW — TX-only validation)
├── bonded_usrp_fullduplex.cpp   (NEW — full-duplex data link demo)
└── CMakeLists.txt               (updated — add new targets)

tests/
├── bonded_receiver_test.cpp     (existing)
├── bonded_transmitter_test.cpp  (NEW — TX alignment unit tests)
├── packet_codec_test.cpp        (NEW — encode/decode/CRC tests)
└── CMakeLists.txt               (updated — add new test targets)
```

---

## 3. Implementation Phases

### Phase 9: Bonded Transmitter Engine ⏳ PENDING

Create the `bonded_transmitter` class as a TX counterpart to `bonded_receiver`.

**Goals**:

- Provide a self-contained, reusable TX engine for N synchronized devices
- Support burst and continuous TX modes with timed metadata
- Track per-device underruns independently
- Support per-device frequency plans (for wideband TX)
- Accept pre-opened `multi_usrp` instances for full-duplex sharing

---

#### Phase 9.1 — Shared Device Pool Helper

Extract the common device-opening and sync logic used by both `bonded_receiver` and `bonded_transmitter` into a small helper class.

**What to implement**:

- `bonded_device_pool.hpp/.cpp` — utility class that:
  - Opens N `multi_usrp` instances (one per serial)
  - Applies clock/time source configuration
  - Waits for reference lock
  - Performs PPS time alignment
  - Returns the vector of ready-to-use `multi_usrp::sptr`

- Refactor `bonded_receiver` to optionally use this pool (backward-compatible: if no external devices passed, it creates its own pool internally as before)

**Why**: Avoids code duplication between TX and RX. The pool is a simple utility, not an abstract base class — it's called once at startup and then forgotten.

**Deliverables**:

- `lib/usrp/bonded/bonded_device_pool.hpp`
- `lib/usrp/bonded/bonded_device_pool.cpp`
- Updated `bonded_receiver` with alternate constructor accepting pre-opened devices
- CMakeLists.txt updated

**Acceptance criteria**:

- Existing `bonded_usrp_rx` and `bonded_usrp_continuous` produce identical results (regression)
- `bonded_receiver` works both with internal device creation (existing path) and with externally-provided devices

---

#### Phase 9.2 — `bonded_transmitter` Core Implementation

Implement the TX engine class.

**What to implement**:

- `bonded_transmitter.hpp` — public API (see Section 4 for full API)
- `bonded_transmitter.cpp` — implementation with:
  - `configure()`: create `tx_streamer` per device, set TX rate/freq/gain, start async metadata listener threads
  - `send_burst(data, delay)`: timed multi-device burst transmission with parallel send threads
  - `start_continuous()`: begin continuous TX mode with timed start
  - `push_samples(data)`: enqueue samples for continuous transmission (split across devices if freq_plan)
  - `stop()`: send end-of-burst on all devices, join threads, report final underrun counts
  - Async metadata listener: per-device thread polling `recv_async_msg()` for underrun/late/sequence errors

**Internal structure**:

- `_devices`: `vector<multi_usrp::sptr>` — shared or owned
- `_tx_streamers`: `vector<tx_streamer::sptr>` — one per device
- `_send_threads`: `vector<thread>` — one per device (continuous mode)
- `_tx_queues`: per-device thread-safe sample queues (continuous mode)
- `_async_threads`: per-device async message listener threads
- `_underrun_counts`: `vector<atomic<size_t>>` — per-device underrun counter

**Burst mode flow**:

1. Application provides `data[device_index] = vector<complex<float>>` (pre-split per device)
2. Compute `start_time = device[0].get_time_now() + delay`
3. For each device in parallel: `send()` with `tx_metadata_t{start_of_burst=true, has_time_spec=true, time_spec=start_time}`
4. After all data sent: `send()` with `end_of_burst=true` on each device
5. Return `tx_result` with per-device success/underrun status

**Continuous mode flow**:

1. `start_continuous()`: issue timed first `send()` with start_of_burst on all devices
2. Per-device send threads consume from their TX queue in a loop
3. `push_samples()`: splits wideband data (or accepts pre-split data) and enqueues per-device
4. `stop()`: sends EOB, joins threads, reports underrun statistics

**Deliverables**:

- `lib/usrp/bonded/bonded_transmitter.hpp`
- `lib/usrp/bonded/bonded_transmitter.cpp`
- CMakeLists.txt updated

**Acceptance criteria**:

- Compiles cleanly as part of libUHD
- Unit tests pass (Phase 9.4)
- Hardware validation with `bonded_usrp_tx` example (Phase 9.3)

---

#### Phase 9.3 — TX-Only Validation Example

Create a minimal example that uses `bonded_transmitter` to send a known waveform across N devices.

**What to implement**:

- `examples/bonded_usrp_tx.cpp`:
  - Parses CLI args: serials, rate, freq (or freq_plan), gain, duration, waveform type (CW tone / chirp / noise)
  - Creates `bonded_device_pool` → opens devices, syncs, aligns time
  - Creates `bonded_transmitter` with the pool's devices
  - Configures TX parameters
  - Sends burst or continuous waveform for specified duration
  - Reports: duration, samples sent per device, underrun count per device, PASS/FAIL (0 underruns = PASS)

**Example usage**:

```bash
# 2-device bonded TX, CW tone at 100 MHz, 40 MHz rate, 10 seconds
./bonded_usrp_tx \
  --args "sync_clock_source=external,sync_time_source=external,serial0=30B56D6,serial1=30DBC3C" \
  --rate 40e6 --freq 100e6 --gain 50 --duration 10 --waveform tone
# Expected: PASS  0 underruns on both devices
```

**Deliverables**:

- `examples/bonded_usrp_tx.cpp`
- CMakeLists.txt updated with build target

**Acceptance criteria**:

- 2-device TX at 40 MHz: 0 underruns over 10 s (PASS)
- 2-device TX at 40 MHz: 0 underruns over 60 s stress test (PASS)
- Waveform is receivable on a separate device (validated via existing `capture_sync.py` + `analyze_sync.py`)

---

#### Phase 9.4 — Unit Tests for `bonded_transmitter`

Offline tests (no hardware) validating TX scheduling and queue logic.

**What to implement**:

- `tests/bonded_transmitter_test.cpp`:
  - Test 1: Burst timing — verify all devices receive the same `time_spec` in metadata
  - Test 2: Continuous queue — push samples, verify per-device dequeue order and completeness
  - Test 3: Underrun tracking — simulate async underrun message, verify counter increments
  - Test 4: End-of-burst — verify EOB metadata is sent on `stop()`
  - Test 5: Pre-split data routing — verify device N gets only its assigned samples

**Deliverables**:

- `tests/bonded_transmitter_test.cpp`
- CMakeLists.txt updated

**Acceptance criteria**:

- `ctest -R bonded_transmitter --output-on-failure` passes (all 5 tests)

---

### Phase 10: Packet Codec Library ⏳ PENDING

Implement a minimal framed protocol for validated data transfer over bonded channels.

**Goals**:

- Provide encode/decode functions for framed QPSK packets with CRC-32
- Enable data integrity verification across bonded TX→RX links
- Keep it simple: no FEC, no adaptive modulation, no retransmission logic

---

#### Phase 10.1 — Frame Format and Codec Implementation

**Frame structure** (transmitted as IQ samples after QPSK modulation):

```
┌──────────────────────────────────────────────────────────────────┐
│ Preamble (64 symbols) │ Header (8 bytes) │ Payload (N bytes) │ CRC-32 (4 bytes) │
└──────────────────────────────────────────────────────────────────┘

Preamble: Zadoff-Chu sequence (root=1, length=64)
          — excellent auto-correlation, easy threshold detection
Header:   [payload_length: uint16] [sequence_number: uint16]
          [flags: uint8] [reserved: uint8] [reserved: uint16]
Payload:  arbitrary data (up to 65535 bytes per frame)
CRC-32:   IEEE 802.3 polynomial over header + payload bytes
```

**Modulation**: QPSK (Gray-coded, 2 bits/symbol)
**Pulse shaping**: Root-raised cosine filter (rolloff = 0.25, span = 6 symbols)

**What to implement**:

- `lib/usrp/bonded/packet_codec.hpp`:

  ```cpp
  namespace uhd { namespace usrp { namespace bonded {

  struct frame_header {
      uint16_t payload_length;
      uint16_t sequence_number;
      uint8_t flags;         // bit 0: ACK frame, bit 1: data frame
  };

  struct decoded_frame {
      frame_header header;
      std::vector<uint8_t> payload;
      bool crc_valid;
      double snr_estimate;   // from preamble correlation peak
      size_t sample_offset;  // position in input IQ stream
  };

  // Encode: bytes → IQ samples
  std::vector<std::complex<float>> encode_frame(
      const std::vector<uint8_t>& payload,
      uint16_t seq_num,
      uint8_t flags,
      double samples_per_symbol = 2.0
  );

  // Detect preamble in IQ stream, return sample offsets of detected frames
  std::vector<size_t> detect_preambles(
      const std::vector<std::complex<float>>& iq,
      double threshold = 0.7  // normalized correlation threshold
  );

  // Decode: IQ samples (starting at frame boundary) → decoded frame
  decoded_frame decode_frame(
      const std::vector<std::complex<float>>& iq,
      size_t offset,
      double samples_per_symbol = 2.0
  );

  // Utility: compute CRC-32 over byte buffer
  uint32_t compute_crc32(const uint8_t* data, size_t length);

  }}} // namespace
  ```

- `lib/usrp/bonded/packet_codec.cpp`:
  - Zadoff-Chu preamble generation and matched-filter correlation
  - QPSK modulator (Gray-coded: 00→+1+j, 01→-1+j, 11→-1-j, 10→+1-j, normalized)
  - QPSK demodulator (hard decision)
  - Root-raised cosine pulse shaping filter (apply on TX, matched filter on RX)
  - CRC-32 computation (IEEE 802.3 table-based)
  - Frame assembly and disassembly (bit packing/unpacking)

**Deliverables**:

- `lib/usrp/bonded/packet_codec.hpp`
- `lib/usrp/bonded/packet_codec.cpp`
- CMakeLists.txt updated

---

#### Phase 10.2 — Packet Codec Unit Tests

**What to implement**:

- `tests/packet_codec_test.cpp`:
  - Test 1: Encode → decode round-trip (no noise) — verify payload matches exactly, CRC valid
  - Test 2: Encode → add AWGN (SNR = 15 dB) → decode — verify CRC still passes
  - Test 3: Encode → corrupt 1 bit in payload → decode — verify CRC fails
  - Test 4: Preamble detection accuracy — multiple frames in stream, verify all offsets found
  - Test 5: Preamble false-positive rate — random noise input, verify no detections above threshold
  - Test 6: Sequence number and flags preserved through encode/decode

**Acceptance criteria**:

- `ctest -R packet_codec --output-on-failure` passes (all 6 tests)
- Round-trip at SNR ≥ 10 dB: 0% frame error rate over 1000 frames

---

### Phase 11: Band Splitter Utility ⏳ PENDING

Implement the inverse of the spectrum stitcher: split a wideband IQ signal into per-device sub-bands for bonded TX.

**Goals**:

- Given a wideband IQ stream and a frequency plan, produce per-device IQ streams at baseband
- Apply appropriate filtering and overlap windowing
- Keep as a standalone utility (not embedded in `bonded_transmitter`)

---

#### Phase 11.1 — Band Splitter Implementation

**What to implement**:

- `lib/usrp/bonded/band_splitter.hpp`:

  ```cpp
  namespace uhd { namespace usrp { namespace bonded {

  // Split wideband IQ into per-device sub-band IQ
  // Input: wideband_iq at aggregate sample rate
  // Output: per_device_iq[device_index] at per-device sample rate
  std::vector<std::vector<std::complex<float>>> split_to_bands(
      const std::vector<std::complex<float>>& wideband_iq,
      const adjacent_frequency_plan& plan,
      double aggregate_rate  // = sum of per-device rates minus overlaps
  );

  // Streaming variant: process chunk-by-chunk with filter state
  class band_splitter {
  public:
      explicit band_splitter(const adjacent_frequency_plan& plan, double aggregate_rate);
      std::vector<std::vector<std::complex<float>>> process(
          const std::vector<std::complex<float>>& chunk
      );
  };

  }}} // namespace
  ```

- `lib/usrp/bonded/band_splitter.cpp`:
  - Frequency-shift each sub-band to baseband (complex multiply by carrier)
  - Low-pass filter to per-device bandwidth
  - Decimate to per-device sample rate
  - Apply overlap window (complementary to stitcher's crossfade)

**Deliverables**:

- `lib/usrp/bonded/band_splitter.hpp`
- `lib/usrp/bonded/band_splitter.cpp`
- CMakeLists.txt updated

**Acceptance criteria**:

- Round-trip test: generate wideband → split → stitch → compare. Power error < 0.1 dB.
- No spectral leakage between adjacent bands beyond -40 dB

---

### Phase 12: Full-Duplex Bonded Example ⏳ PENDING

Create the showcase demonstration that proves bonded TX+RX works for real data transfer.

**Goals**:

- Demonstrate full-duplex bonded operation (simultaneous TX+RX on N devices)
- Validate end-to-end data integrity with CRC-32
- Measure throughput, frame error rate, and round-trip latency
- Produce a single binary that can run as either "node A" or "node B"

---

#### Phase 12.1 — Full-Duplex Example Implementation

**What to implement**:

- `examples/bonded_usrp_fullduplex.cpp`:

**Command-line interface**:

```bash
./bonded_usrp_fullduplex \
  --args "sync_clock_source=external,sync_time_source=external,serial0=X,serial1=Y" \
  --rate 40e6 --freq 900e6 --tx-gain 50 --rx-gain 40 \
  --role sender|receiver|loopback \
  --duration 60 --payload-size 1024 --report-interval 5
```

**Roles**:

| Role       | TX behavior                           | RX behavior                          |
| ---------- | ------------------------------------- | ------------------------------------ |
| `sender`   | Generate data frames, transmit bonded | Receive ACK frames, track round-trip |
| `receiver` | Transmit ACK frames for valid data    | Receive data frames, validate CRC    |
| `loopback` | Both sender + receiver (self-test)    | Both sender + receiver (self-test)   |

**Internal architecture** (single binary, single process):

```
┌─────────────────────────────────────────────────────────┐
│  bonded_usrp_fullduplex                                 │
│                                                         │
│  ┌─────────────────┐    ┌────────────────────────────┐  │
│  │  TX Pipeline     │    │  RX Pipeline                │  │
│  │                 │    │                            │  │
│  │  Data Gen       │    │  bonded_receiver           │  │
│  │  → encode_frame │    │  → get_aligned_samples     │  │
│  │  → push_samples │    │  → detect_preambles        │  │
│  │                 │    │  → decode_frame            │  │
│  │  bonded_        │    │  → validate CRC            │  │
│  │  transmitter    │    │  → generate ACK (if recv)  │  │
│  └─────────────────┘    └────────────────────────────┘  │
│                                                         │
│  Shared: bonded_device_pool (opens devices once)        │
└─────────────────────────────────────────────────────────┘
```

**Protocol flow (sender→receiver→sender)**:

1. Sender generates random payload (configurable size)
2. Sender encodes frame (preamble + header + payload + CRC)
3. Sender transmits via `bonded_transmitter` (bonded wideband TX)
4. Receiver captures via `bonded_receiver` (bonded wideband RX)
5. Receiver detects preamble, decodes frame, validates CRC
6. Receiver encodes ACK frame (seq_num of received frame + ACK flag)
7. Receiver transmits ACK via its `bonded_transmitter`
8. Sender receives ACK via its `bonded_receiver`
9. Sender confirms round-trip, logs latency

**Real-time statistics output** (every `--report-interval` seconds):

```
[  5.0s] TX: 487 frames | RX: 483 frames | CRC OK: 483 | CRC FAIL: 0 | FER: 0.00% | Throughput: 3.96 Mbps | RTT: 2.1 ms avg
[ 10.0s] TX: 974 frames | RX: 971 frames | CRC OK: 971 | CRC FAIL: 0 | FER: 0.00% | Throughput: 3.95 Mbps | RTT: 2.2 ms avg
```

**Final summary**:

```
=== Full-Duplex Bonded Validation Complete ===
Duration:        60.0 s
Frames sent:     5844
Frames received: 5841
CRC valid:       5841
CRC invalid:     0
Frame error rate: 0.00%
Payload throughput: 3.95 Mbps (aggregate bonded)
Round-trip latency: min=1.8 ms, mean=2.1 ms, max=4.3 ms, p99=3.1 ms
TX underruns:    dev0=0, dev1=0
RX overflows:    dev0=0, dev1=0
Result: PASS
```

**Deliverables**:

- `examples/bonded_usrp_fullduplex.cpp`
- CMakeLists.txt updated

---

#### Phase 12.2 — Loopback Self-Test Mode

The `--role loopback` mode runs both sender and receiver roles on the same devices (TX port → RX port on same device, or short antenna cable). This enables hardware validation without needing a second node.

**Use case**: Single-VM or single-host validation before deploying the two-VM experiment.

**Pass criteria**:

- 60 s at 40 MHz × 2 devices: 0% frame error rate, 0 underruns, 0 overflows
- Round-trip latency < 10 ms (dominated by processing, not RF)

---

#### Phase 12.3 — Two-Node Experiment (Validation Scenario)

This is NOT a code change — it's a validation procedure using the binary from Phase 12.1.

**Setup**:

- VM A: USB Controller 1 passed through → devices 30B56D6 + 30DBC3C
- VM B: USB Controller 2 passed through → devices 30DBC3D + 30EDB63
- Both VMs share physical 10 MHz + PPS reference (via USB-attached B210 SMA inputs)
- Antennas: VERT900 on all devices, short-range LOS

**Procedure**:

```bash
# On VM B (start receiver first):
./bonded_usrp_fullduplex \
  --args "sync_clock_source=external,sync_time_source=external,serial0=30DBC3D,serial1=30EDB63" \
  --rate 40e6 --freq 900e6 --tx-gain 50 --rx-gain 40 \
  --role receiver --duration 60

# On VM A (start sender after receiver is ready):
./bonded_usrp_fullduplex \
  --args "sync_clock_source=external,sync_time_source=external,serial0=30B56D6,serial1=30DBC3C" \
  --rate 40e6 --freq 900e6 --tx-gain 50 --rx-gain 40 \
  --role sender --duration 60
```

**Pass criteria**:

- Both nodes report 0% frame error rate
- Sender receives ACKs for ≥ 99% of sent frames
- No TX underruns or RX alignment failures on either node
- Aggregate throughput matches expected for QPSK at 40 MHz × 2 devices

**Deliverables**:

- Validated command lines added to Section 3.X of implementation plan
- Log files from both nodes
- Summary table with measured metrics

---

### Phase 13: TX Bonding Validation and Documentation ⏳ PENDING

Integrate TX validation results into the thesis documentation.

---

#### Phase 13.1 — TX Hardware Integration Tests

| #   | Test                        | Devices | Duration | Pass Criteria                    |
| --- | --------------------------- | ------- | -------- | -------------------------------- |
| 1   | TX burst, 2 devices         | 2       | 10 s     | 0 underruns                      |
| 2   | TX burst, 4 devices         | 4       | 10 s     | 0 underruns                      |
| 3   | TX continuous, 2 dev, 60 s  | 2       | 60 s     | 0 underruns                      |
| 4   | TX continuous, 4 dev, 60 s  | 4       | 60 s     | 0 underruns                      |
| 5   | Full-duplex loopback, 2 dev | 2       | 60 s     | 0% FER, 0 underruns, 0 overflows |
| 6   | Full-duplex two-node, 4 dev | 4       | 60 s     | 0% FER, ACK rate ≥ 99%           |
| 7   | Wideband TX stitch quality  | 2       | burst    | received spectrum seam < 1 dB    |

**Deliverables**: Log files, results summary table, validated commands in Section 3.X.

---

#### Phase 13.2 — Documentation and Thesis Integration

**Tasks**:

1. Update `BONDING_IMPLEMENTATION_PLAN_FLEXIBLE.md` with Phase 9–13 summary and cross-references
2. Add TX-related validated commands to Section 3.X
3. Write thesis section: "Bidirectional Bonded Communication Validation"
   - Full-duplex architecture
   - Packet codec design and rationale
   - Measured throughput, FER, and latency results
   - Comparison: bonded vs. single-device performance
4. Tag Git commit marking TX bonding phases complete

---

## 4. API Design

### 4.1 `bonded_transmitter` Public API

```cpp
namespace uhd { namespace usrp { namespace bonded {

class bonded_transmitter {
public:
    struct config {
        std::vector<std::string> serials;    // device serials (ignored if devices provided)
        std::string clock_source = "external";
        std::string time_source = "external";
        double rate;                         // TX sample rate per device (Hz)
        double freq;                         // TX center frequency (Hz)
        std::vector<double> freq_plan;       // per-device frequencies (overrides freq)
        double gain;                         // TX gain (dB)
        std::string subdev;                  // optional subdev spec (e.g., "A:A")
        bool strict = false;
        double lock_timeout = 5.0;
    };

    struct tx_result {
        std::vector<bool> success;           // per device
        std::vector<size_t> samples_sent;    // per device
        std::vector<size_t> underrun_count;  // per device
        std::string error_message;
    };

    // Standard constructor (opens devices internally)
    explicit bonded_transmitter(const config& cfg);

    // Shared-device constructor (for full-duplex — devices already opened and synced)
    bonded_transmitter(const config& cfg, std::vector<multi_usrp::sptr> devices);

    ~bonded_transmitter();

    // Setup: create TX streamers, configure TX hardware
    void configure();

    // --- Burst mode ---
    // Send pre-split data[device_index] = IQ samples for that device
    tx_result send_burst(
        const std::vector<std::vector<std::complex<float>>>& data,
        double delay_sec = 1.5
    );

    // --- Continuous mode ---
    void start_continuous(double delay_sec = 1.5);

    // Push per-device samples into TX queues
    void push_samples(const std::vector<std::vector<std::complex<float>>>& data);

    // Stop transmitting (sends EOB on all devices)
    tx_result stop();

    // --- Queries ---
    size_t num_devices() const;
    bool is_transmitting() const;
    bool is_configured() const;
    std::vector<size_t> get_underrun_counts() const;
};

}}} // namespace
```

### 4.2 `bonded_device_pool` API

```cpp
namespace uhd { namespace usrp { namespace bonded {

struct device_pool_config {
    std::vector<std::string> serials;
    std::string clock_source = "external";
    std::string time_source = "external";
    bool strict = false;
    double lock_timeout = 5.0;
};

class bonded_device_pool {
public:
    explicit bonded_device_pool(const device_pool_config& cfg);

    // Open all devices, apply sync, align time via PPS
    void initialize();

    // Get the shared device instances (for passing to TX/RX engines)
    std::vector<multi_usrp::sptr> get_devices() const;

    // Get a specific device
    multi_usrp::sptr get_device(size_t index) const;

    size_t num_devices() const;
};

}}} // namespace
```

### 4.3 Full-Duplex Usage Pattern

```cpp
// 1. Open devices once
bonded_device_pool::config pool_cfg;
pool_cfg.serials = {"30B56D6", "30DBC3C"};
pool_cfg.clock_source = "external";
pool_cfg.time_source = "external";

bonded_device_pool pool(pool_cfg);
pool.initialize();  // opens, syncs, PPS-aligns

// 2. Create TX engine (shared devices)
bonded_transmitter::config tx_cfg;
tx_cfg.rate = 40e6;
tx_cfg.freq = 900e6;
tx_cfg.gain = 50;

bonded_transmitter tx(tx_cfg, pool.get_devices());
tx.configure();  // creates tx_streamers only (no device open/sync)

// 3. Create RX engine (shared devices)
bonded_receiver::config rx_cfg;
rx_cfg.rate = 40e6;
rx_cfg.freq = 900e6;
rx_cfg.gain = 40;

bonded_receiver rx(rx_cfg, pool.get_devices());
rx.configure();  // creates rx_streamers only (no device open/sync)

// 4. Run full-duplex
tx.start_continuous();
rx.start_continuous();

while (running) {
    // TX: generate and push frames
    auto frame_iq = encode_frame(payload, seq++, FLAG_DATA);
    tx.push_samples(split_to_devices(frame_iq));

    // RX: receive and validate
    auto result = rx.get_aligned_samples(chunk_size);
    auto frames = detect_and_decode(result.data);
    for (auto& f : frames) {
        if (f.crc_valid) { /* success */ }
    }
}

auto tx_result = tx.stop();
rx.stop();
```

---

## 5. Testing & Validation

### Test Matrix

| Test                           | Phase | Hardware | Pass Criteria                      |
| ------------------------------ | ----- | -------- | ---------------------------------- |
| TX alignment unit test         | 9.4   | No       | All 5 cases pass                   |
| Packet codec unit test         | 10.2  | No       | All 6 cases pass                   |
| TX burst, 2 dev, 10 s          | 13.1  | Yes      | 0 underruns                        |
| TX burst, 4 dev, 10 s          | 13.1  | Yes      | 0 underruns                        |
| TX continuous, 2 dev, 60 s     | 13.1  | Yes      | 0 underruns                        |
| TX continuous, 4 dev, 60 s     | 13.1  | Yes      | 0 underruns                        |
| Full-duplex loopback, 2 dev    | 12.2  | Yes      | 0% FER, 0 underruns/overflows      |
| Full-duplex two-node, 4 dev    | 12.3  | Yes      | 0% FER, ≥99% ACK rate              |
| RX regression (bonded_usrp_rx) | 9.1   | Yes      | Identical to previous PASS results |

### Regression Safety

Every phase that modifies existing code (Phase 9.1 refactors `bonded_receiver`) must be followed by a full regression:

```bash
# After any library change:
cd /home/gabriel/uhd/host/build && sudo make -j$(nproc)
ctest -R bonded_receiver --output-on-failure
./examples/bonded_usrp_rx --args "sync_clock_source=external,sync_time_source=external,serial0=30B56D6,serial1=30DBC3C" --rate 10e6 --freq 100e6 --nsamps 100000
# Must still produce: PASS  spread=0.000 us
```

---

## 6. Future Work

These are explicitly out of scope for this plan but enabled by the architecture:

### 6.1 — Real Protocol at 160 MHz+ Bandwidth

- Use 4+ bonded radios (minimum 4 B210s) for aggregate ~140–160 MHz
- Implement a real communication protocol (e.g., OFDM-based)
- Communicate with a non-USRP device on the other side (e.g., commercial SDR, real base station)
- Requires: band_splitter at full rate, more sophisticated framing, possibly FEC

### 6.2 — OFDM Modulation Upgrade

- Replace QPSK packet codec with OFDM (FFT/IFFT-based)
- Cyclic prefix, pilot subcarriers, channel estimation
- Higher spectral efficiency (16-QAM, 64-QAM on good channels)
- More thesis-worthy but significantly more complex

### 6.3 — Adaptive Modulation and Coding

- Monitor SNR per sub-band, adapt modulation order dynamically
- Add FEC (convolutional, LDPC, or turbo codes)
- Implement ARQ (automatic repeat request) for reliability

### 6.4 — GNURadio TX Integration

- Extend `gr-bonded-usrp` OOT module with a `bonded_usrp_sink` block
- `gr::block::work()` → `bonded_transmitter::push_samples()`
- Enables GNURadio flowgraphs for bonded TX applications

### 6.5 — Multi-Host Distributed Bonding

- Bonded radios spread across multiple physical hosts
- Network-based time synchronization (PTP/White Rabbit) instead of shared PPS cable
- Requires coordination layer for TX scheduling across hosts

---

## Document History

| Version | Date       | Changes                                      |
| ------- | ---------- | -------------------------------------------- |
| 1.0     | 2026-05-26 | Initial TX bonding plan: Phases 9–13 defined |

**Status**: Plan complete. Implementation not started.
