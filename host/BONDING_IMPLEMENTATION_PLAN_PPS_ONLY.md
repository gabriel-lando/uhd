# USRP Bonding Implementation Plan - PPS-Only Synchronization

## Multi-Device Bandwidth Aggregation Using Only Pulse-Per-Second Signal

**Project Goal**: Develop a driver-level mechanism to bond multiple USRP B210 (or other USB-based) devices using ONLY external PPS synchronization, without requiring a shared 10 MHz reference clock.

**Hardware Assumed**: GPS module with 1PPS output, shared to all USRPs via their PPS input SMA port.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [PPS-Only Synchronization Rationale](#pps-only-rationale)
3. [Architecture Overview](#architecture-overview)
4. [UHD API Integration Points](#uhd-api-integration)
5. [Implementation Phases](#implementation-phases)
6. [PPS-Only Synchronization Strategy](#pps-only-synchronization)
7. [Frequency Drift Management](#frequency-drift-management)
8. [Testing & Validation](#testing-validation)
9. [Challenges & Mitigations](#challenges-mitigations)
10. [Comparison: PPS-Only vs. 10MHz+PPS](#comparison)

---

## 1. Executive Summary

### Objective

Create a bonding mechanism that requires ONLY a shared PPS signal (no 10 MHz reference), allowing each USRP to use its internal TCXO while maintaining time synchronization for bandwidth aggregation applications.

### Key Constraint

Each B210 runs its own 40 MHz TCXO (±2 ppm). All devices share a GPS-derived PPS signal connected to their PPS input SMA port. The PPS provides **time alignment only** — each device's sample clock still runs independently. This means:

- Hardware timestamps are aligned to within ~100 ns at each PPS edge
- Sample rates between devices differ by up to ±4 ppm (worst case)
- USB 3.0 transport adds non-deterministic latency (1-5 ms), but UHD hardware timestamps in `rx_metadata_t.time_spec` provide sub-sample timing — **USB jitter does NOT affect sample alignment**
- Phase drift accumulates continuously and must be tracked

### Key Differences from 10MHz+PPS Approach

- **Simpler Hardware**: Only PPS distribution needed (no 10 MHz splitter)
- **Lower Cost**: GPS module with PPS output ($20-50 vs. GPSDO $300-1000)
- **Independent Clocks**: Each device runs on internal oscillator
- **Drift Compensation**: Software must handle frequency drift between devices
- **Suitable For**: Applications tolerant to small frequency errors (~2-5 ppm), or able to use software drift compensation

### Key Features

- **PPS-Only Synchronization**: Time alignment using GPS PPS signal
- **Timestamp-Based Sample Alignment**: Use UHD hardware timestamps (`rx_metadata_t.time_spec`) for sub-sample alignment — immune to USB jitter
- **Drift Tracking**: Software-based frequency offset estimation and phase correction
- **Adaptive Combining**: Frequency-domain stitching with drift-aware guard bands
- **Simplified Setup**: GPS module + SMA cables only

### Target Use Cases

- **Wide-spectrum monitoring** (adjacent bands stitched together for >56 MHz total)
- **Non-coherent multi-band reception** (different frequency ranges per device — simplest case, no drift compensation needed between bands)
- **Budget-conscious deployments** without access to precision references
- **Mobile/portable setups** where GPS PPS is available but not 10 MHz
- **Research applications** studying drift compensation techniques

---

## 2. PPS-Only Synchronization Rationale

### 2.1 Why PPS-Only?

**Advantages:**

1. **Simplified Hardware Setup**
   - GPS module PPS output → SMA Y-splitter → B210 PPS inputs
   - No RF power splitter for 10 MHz required
   - Single coax distribution (PPS is a digital edge, not RF)
   - Reduces cable count and complexity

2. **Cost Reduction**
   - GPS receiver with PPS: $20-50 (vs. GPSDO: $300-1000)
   - No 10 MHz distribution amplifier needed
   - Fewer SMA cables and connectors
   - Total hardware cost reduction: ~$500-1000

3. **Flexibility**
   - PPS signal is digital (3.3V CMOS), tolerant of cable quality
   - Cable runs up to 10 meters have negligible impact (<50 ns)
   - GPS availability everywhere (field, mobile, indoor with antenna)

**Disadvantages:**

1. **Frequency Drift Between Devices**
   - B210 TCXO accuracy: ±2 ppm (±2 kHz at 1 GHz)
   - Two devices can differ by up to 4 ppm (±2 each, opposite directions)
   - At 56 MS/s sample rate, 4 ppm = 224 Hz sample rate difference
   - Over 1 second: 224 samples of slip (creates growing time misalignment)

2. **Phase Drift Accumulation**
   - No common frequency reference to discipline oscillators
   - At 2.4 GHz, 2 ppm offset = 4.8 kHz → 360° phase rotation every 208 µs
   - Phase noise of each TCXO is uncorrelated
   - Coherent combination of overlapping bands impossible without correction

3. **Periodic Recalibration**
   - Must re-estimate frequency offsets every 5-30 seconds
   - Drift rate changes with temperature (~0.1 ppm/°C for typical TCXO)
   - Unsuitable for applications requiring <0.1 ppm accuracy

### 2.2 Technical Feasibility

The B210's internal TCXO (40 MHz) has typical specifications:

- **Initial Accuracy**: ±2 ppm at 25°C
- **Temperature Stability**: ±2 ppm over 0-70°C
- **Aging**: ±1 ppm per year
- **Short-term Stability (Allan Deviation)**: ~1e-9 @ 1s (excellent over seconds)

For bandwidth aggregation, the key insight:

| Time Scale    | TCXO Behavior                          | Implication                                        |
| ------------- | -------------------------------------- | -------------------------------------------------- |
| < 1 second    | Very stable (Allan dev ~1e-9)          | Phase correction from last calibration is accurate |
| 1-30 seconds  | Drifts slowly (~0.01 ppm)              | Periodic recalibration sufficient                  |
| Minutes-hours | Temperature-driven drift (~0.1 ppm/°C) | Must track and correct                             |

**Bottom Line**: PPS-only is viable because TCXO short-term stability is excellent. The challenge is tracking the slowly-changing offset between devices, not fighting random jitter.

---

## 3. Architecture Overview

### 3.1 System Block Diagram

```
                    +------------------+
                    |   GPS Module     |
                    |  (1PPS Output)   |
                    +--------+---------+
                             |
                             | PPS (3.3V CMOS, 1 Hz)
                             |
                    +--------v---------+
                    |  SMA Y-Splitter  |
                    | (or buffer for   |
                    |  3+ devices)     |
                    +--+-------+-------+
                       |       |
            PPS IN     |       |     PPS IN
              +--------+       +--------+
              |                         |
       +------v-------+          +------v-------+
       |   USRP B210  |          |   USRP B210  |
       |  (Device 0)  |          |  (Device 1)  |
       |  TCXO: 40MHz |          |  TCXO: 40MHz |
       |  (REFERENCE)  |          |  (CORRECTED) |
       +------+-------+          +------+-------+
              |                         |
          USB 3.0                   USB 3.0
              |                         |
              +------------+------------+
                           |
                    +------v-------------+
                    |  Host PC           |
                    | +----------------+ |
                    | | Bonding Layer   | |
                    | | - Timestamp     | |
                    | |   alignment     | |
                    | | - Drift tracker | |
                    | | - Phase correct | |
                    | | - FFT stitch   | |
                    | +----------------+ |
                    +--------------------+
```

### 3.2 Key Architectural Differences

| Component               | 10MHz+PPS Approach                     | PPS-Only Approach                              |
| ----------------------- | -------------------------------------- | ---------------------------------------------- |
| **Clock Source**        | External 10 MHz reference              | Internal TCXO (independent per device)         |
| **Time Sync**           | PPS + locked oscillators               | PPS only (time aligned, freq independent)      |
| **Sample Rate**         | Identical (same master clock)          | Differs by up to ±4 ppm between devices        |
| **Frequency Coherence** | Hardware-locked (excellent)            | Software-tracked (good enough for stitching)   |
| **Drift Compensation**  | Not needed                             | Phase correction required, resampling optional |
| **Hardware Complexity** | High (2 reference signals)             | Low (1 signal: PPS)                            |
| **Setup Cost**          | $500-1500                              | $50-200                                        |
| **Best For**            | Coherent MIMO, adjacent-band stitching | Wideband monitoring, non-coherent multi-band   |

### 3.3 Software Components

New components for PPS-only operation:

1. **Timestamp Aligner**: Uses `rx_metadata_t.time_spec` hardware timestamps to align samples from different devices at sub-sample precision (immune to USB jitter)
2. **Drift Tracker**: Estimates frequency offset (ppm) between device oscillators using calibration bursts or PPS timestamp comparison
3. **Phase Corrector**: Applies per-sample phase rotation (`volk_32fc_s32fc_rotator_32fc`) to compensate accumulated drift
4. **Fractional Resampler** (optional): Corrects sample rate mismatch for long captures (>10 seconds between recalibrations)
5. **FFT Stitcher**: Combines frequency slices from different devices into a single wideband spectrum

### 3.4 What UHD Already Provides

Critical: UHD's existing infrastructure handles most of the hard problems:

| UHD Feature                                            | How We Use It                                                                                                                                                                                                              |
| ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `set_time_source("external")`                          | Configures B210 to timestamp PPS edges from our GPS                                                                                                                                                                        |
| `set_clock_source("internal")`                         | Keeps each device on its own TCXO                                                                                                                                                                                          |
| `set_time_unknown_pps(time_spec_t(0.0))`               | Synchronizes all device clocks at the next PPS edge — handles edge detection automatically                                                                                                                                 |
| `get_time_synchronized()`                              | Verifies all boards are within ~10 ms (coarse check)                                                                                                                                                                       |
| `rx_metadata_t.time_spec`                              | **Hardware timestamp of first sample** — this is the key to alignment. Each B210's FPGA stamps every packet with its local time. Since PPS aligned the time bases, these timestamps are directly comparable across devices |
| `stream_cmd_t` with `stream_now=false`                 | Start streaming at a specific future time on all devices simultaneously                                                                                                                                                    |
| `time_spec_t::to_ticks(sample_rate)`                   | Convert timestamps to sample indices for precise alignment                                                                                                                                                                 |
| Multi-device `multi_usrp::make("addr0=...,addr1=...")` | Already supports multiple devices as a single `multi_usrp` with multiple mboards                                                                                                                                           |

---

## 4. UHD API Integration Points

### 4.1 Using `multi_usrp` with Multiple B210s

UHD already supports multi-device configurations. Two B210s can be opened as a single `multi_usrp`:

```cpp
// UHD already supports this — two USB devices as one multi_usrp
uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(
    "serial=ABC123,serial=DEF456"
);

// This gives us:
// usrp->get_num_mboards() == 2
// usrp->get_rx_num_channels() == 4 (2 channels per B210)
// Board 0 = channels 0,1; Board 1 = channels 2,3
```

### 4.2 PPS-Based Time Synchronization (Already Working in UHD)

The synchronization procedure uses UHD's existing `set_time_unknown_pps()` which handles PPS edge detection automatically:

```cpp
// Step 1: Configure all boards for internal clock, external PPS
for (size_t mboard = 0; mboard < usrp->get_num_mboards(); ++mboard) {
    usrp->set_clock_source("internal", mboard);  // Internal TCXO
    usrp->set_time_source("external", mboard);    // GPS PPS on SMA
}

// Step 2: Synchronize time at next PPS edge
// set_time_unknown_pps() internally:
//   1. Polls get_time_last_pps() until it changes (PPS edge detected)
//   2. Calls set_time_next_pps(time, ALL_MBOARDS) immediately
//   3. Waits 1 second for the next PPS to latch the time
//   4. Verifies all boards within ~10ms
usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
std::this_thread::sleep_for(std::chrono::seconds(2));

// Step 3: Verify synchronization
for (size_t mboard = 0; mboard < usrp->get_num_mboards(); ++mboard) {
    uhd::time_spec_t t = usrp->get_time_last_pps(mboard);
    std::cout << "Board " << mboard << " PPS time: "
              << t.get_real_secs() << " s" << std::endl;
}
```

### 4.3 Timestamp-Based Sample Alignment

**This is the critical insight**: USB jitter does NOT matter for sample alignment because UHD's `rx_metadata_t` provides hardware timestamps:

```cpp
// Create separate streamers for each device's channels
uhd::stream_args_t stream_args("fc32");

// Streamer for device 0, channel 0
stream_args.channels = {0};
auto rx_stream_0 = usrp->get_rx_stream(stream_args);

// Streamer for device 1, channel 0
stream_args.channels = {2};  // Channel 2 = device 1, chan 0
auto rx_stream_1 = usrp->get_rx_stream(stream_args);

// Issue timed stream commands (start at same future time)
uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
cmd.stream_now = false;
cmd.time_spec = usrp->get_time_now() + uhd::time_spec_t(2.0);
rx_stream_0->issue_stream_cmd(cmd);
rx_stream_1->issue_stream_cmd(cmd);

// Receive — timestamps tell us EXACTLY which samples align
uhd::rx_metadata_t md0, md1;
size_t n0 = rx_stream_0->recv(&buf0[0], nsamps, md0, timeout);
size_t n1 = rx_stream_1->recv(&buf1[0], nsamps, md1, timeout);

// md0.time_spec and md1.time_spec are hardware timestamps
// They should match (within sub-sample) after PPS sync
// Convert to sample index for alignment:
int64_t tick0 = md0.time_spec.to_ticks(sample_rate);
int64_t tick1 = md1.time_spec.to_ticks(sample_rate);
int64_t offset = tick1 - tick0;  // Sample offset between devices
```

### 4.4 What We Must Build (Not in UHD)

| Component              | Why It's Needed                                                         |
| ---------------------- | ----------------------------------------------------------------------- |
| **Bonding layer**      | Wraps two `rx_streamer` instances into a single virtual wideband stream |
| **Drift estimator**    | Measures ppm offset between device TCXOs                                |
| **Phase corrector**    | Applies per-sample rotation to compensate drift                         |
| **FFT stitcher**       | Combines adjacent frequency slices into single output                   |
| **Calibration mode**   | Tunes both devices to same freq temporarily for drift measurement       |
| **Guard band manager** | Manages overlap/gap between frequency slices                            |

---

## 5. Implementation Phases

### Phase 1: Multi-Device Setup & PPS Sync (Weeks 1-3)

#### Tasks:

1. **Multi-device initialization with PPS**

   ```cpp
   // Create bonded USRP from two B210s sharing GPS PPS
   uhd::device_addr_t args;
   args["serial"] = "ABC123,DEF456";  // Both serials

   auto usrp = uhd::usrp::multi_usrp::make(args);

   // Configure PPS-only sync
   for (size_t mb = 0; mb < usrp->get_num_mboards(); ++mb) {
       usrp->set_clock_source("internal", mb);  // Each uses own TCXO
       usrp->set_time_source("external", mb);    // GPS PPS on SMA input
   }

   // Synchronize at PPS edge (UHD handles edge detection)
   usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
   std::this_thread::sleep_for(std::chrono::seconds(2));
   ```

2. **PPS presence verification**

   ```cpp
   bool verify_pps_present(uhd::usrp::multi_usrp::sptr usrp, size_t mboard) {
       // Record last PPS timestamp
       auto t0 = usrp->get_time_last_pps(mboard);

       // Wait up to 2 seconds for a new PPS edge
       auto start = std::chrono::steady_clock::now();
       while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
           auto t1 = usrp->get_time_last_pps(mboard);
           if (t1.get_full_secs() != t0.get_full_secs()) {
               return true;  // PPS edge detected
           }
       }
       return false;  // No PPS within 2 seconds
   }
   ```

3. **Time sync verification**

   ```cpp
   void verify_time_sync(uhd::usrp::multi_usrp::sptr usrp) {
       // Wait for next PPS to latch consistent timestamps
       std::this_thread::sleep_for(std::chrono::seconds(1));

       uhd::time_spec_t ref = usrp->get_time_last_pps(0);
       for (size_t mb = 1; mb < usrp->get_num_mboards(); ++mb) {
           uhd::time_spec_t t = usrp->get_time_last_pps(mb);
           double diff = std::abs(t.get_real_secs() - ref.get_real_secs());

           // PPS edge is captured by FPGA with sub-µs precision
           // But since each board's TCXO has different rate, the
           // fractional second will drift between PPS edges
           if (diff > 0.001) {  // > 1 ms = something is wrong
               throw uhd::runtime_error(
                   "PPS time sync failed on board " + std::to_string(mb));
           }
           std::cout << "Board " << mb << " time offset: "
                     << diff * 1e6 << " µs" << std::endl;
       }
   }
   ```

#### Deliverables:

- Multi-B210 initialization with GPS PPS
- PPS detection and verification
- Time synchronization using `set_time_unknown_pps()`
- Verified timestamps aligned across boards

---

### Phase 2: Timestamp-Aligned Dual Streaming (Weeks 4-6)

#### Tasks:

1. **Simultaneous reception from both devices**

   ```cpp
   class bonded_receiver {
   public:
       struct device_stream {
           uhd::rx_streamer::sptr streamer;
           std::vector<std::complex<float>> buffer;
           uhd::rx_metadata_t metadata;
           double center_freq;
       };

       bonded_receiver(uhd::usrp::multi_usrp::sptr usrp,
                       double sample_rate,
                       double freq0, double freq1) {
           _usrp = usrp;
           _sample_rate = sample_rate;

           // Configure frequencies (adjacent bands)
           usrp->set_rx_rate(sample_rate);
           usrp->set_rx_freq(freq0, 0);  // Device 0, ch 0
           usrp->set_rx_freq(freq1, 2);  // Device 1, ch 0

           // Create separate streamers (one per device)
           uhd::stream_args_t args("fc32");
           args.channels = {0};
           _streams[0].streamer = usrp->get_rx_stream(args);
           _streams[0].center_freq = freq0;

           args.channels = {2};
           _streams[1].streamer = usrp->get_rx_stream(args);
           _streams[1].center_freq = freq1;

           // Allocate buffers
           for (auto& s : _streams) {
               s.buffer.resize(BUFFER_SIZE);
           }
       }

       void start_streaming() {
           // Timed start: both devices begin at same time
           uhd::stream_cmd_t cmd(
               uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
           cmd.stream_now = false;
           cmd.time_spec = _usrp->get_time_now() + uhd::time_spec_t(1.0);

           _streams[0].streamer->issue_stream_cmd(cmd);
           _streams[1].streamer->issue_stream_cmd(cmd);
       }

       // Receive one buffer from each device
       // Returns sample offset between devices (from timestamps)
       int64_t recv_aligned(size_t nsamps) {
           size_t n0 = _streams[0].streamer->recv(
               &_streams[0].buffer[0], nsamps,
               _streams[0].metadata, 1.0);
           size_t n1 = _streams[1].streamer->recv(
               &_streams[1].buffer[0], nsamps,
               _streams[1].metadata, 1.0);

           // Check for errors
           if (_streams[0].metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE
            || _streams[1].metadata.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
               handle_stream_error();
               return -1;
           }

           // Compute sample alignment from hardware timestamps
           int64_t tick0 = _streams[0].metadata.time_spec.to_ticks(_sample_rate);
           int64_t tick1 = _streams[1].metadata.time_spec.to_ticks(_sample_rate);
           return tick1 - tick0;
       }

   private:
       uhd::usrp::multi_usrp::sptr _usrp;
       double _sample_rate;
       std::array<device_stream, 2> _streams;
       static constexpr size_t BUFFER_SIZE = 65536;
   };
   ```

2. **Timestamp alignment accounting for drift**

   After PPS sync, each device's TCXO ticks at a slightly different rate.
   The `time_spec` timestamps will slowly diverge:

   ```
   At PPS edge:     Device 0 time = 0.000000 s,  Device 1 time = 0.000000 s
   After 1 second:  Device 0 time = 1.000000 s,  Device 1 time = 1.000002 s  (2 ppm fast)
   After 10 seconds: Device 0 time = 10.00000 s, Device 1 time = 10.00002 s
   ```

   This is actually useful: the divergence directly measures the frequency offset!

   ```cpp
   // Track PPS-stamped times to measure drift
   double measure_drift_from_pps(uhd::usrp::multi_usrp::sptr usrp) {
       // Wait for next PPS and record both devices' timestamps
       auto pps0_a = usrp->get_time_last_pps(0);
       auto pps1_a = usrp->get_time_last_pps(1);

       // Wait for several PPS edges (more = better accuracy)
       std::this_thread::sleep_for(std::chrono::seconds(10));

       auto pps0_b = usrp->get_time_last_pps(0);
       auto pps1_b = usrp->get_time_last_pps(1);

       // Elapsed time on each device's clock
       double elapsed0 = pps0_b.get_real_secs() - pps0_a.get_real_secs();
       double elapsed1 = pps1_b.get_real_secs() - pps1_a.get_real_secs();

       // Drift in ppm (relative to device 0 as reference)
       // If elapsed1 > elapsed0, device 1's oscillator is faster
       double drift_ppm = ((elapsed1 - elapsed0) / elapsed0) * 1e6;

       // Accuracy: with 10 PPS edges, each with ~100 ns uncertainty,
       // error = 100 ns / 10 s = 0.01 ppm — actually very good!
       return drift_ppm;
   }
   ```

   **Key insight**: The PPS signal acts as a **calibration reference** — every second the GPS gives us a ground-truth timing edge. By comparing how each device's clock measures the interval between PPS edges, we get a precise drift estimate without any RF signals.

3. **Handle overflow (dropped samples) gracefully**
   ```cpp
   void handle_stream_error() {
       for (size_t i = 0; i < 2; ++i) {
           if (_streams[i].metadata.error_code
               == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
               // Overflow = device produced samples faster than host consumed
               // The timestamp in next packet tells us exactly how many
               // samples were lost. Log it and continue.
               std::cerr << "Device " << i << ": overflow detected at "
                         << _streams[i].metadata.time_spec.get_real_secs()
                         << " s" << std::endl;
               // Don't abort — timestamps will re-align automatically
           }
       }
   }
   ```

#### Deliverables:

- Simultaneous dual-device streaming
- Timestamp-based sample alignment
- PPS-based drift measurement (no RF signals needed!)
- Overflow handling

---

### Phase 3: Drift Estimation & Phase Correction (Weeks 7-10)

**This is the CRITICAL phase for PPS-only operation**

#### Tasks:

1. **Implement drift tracker with PPS-based method (primary)**

   ```cpp
   class pps_drift_tracker {
   public:
       struct drift_estimate {
           double frequency_offset_ppm;   // Dev1 vs Dev0 offset
           double phase_rate_rad_per_s;   // At RF center frequency
           uhd::time_spec_t last_update;
           int num_pps_samples;           // How many PPS edges used
       };

       // PRIMARY METHOD: PPS timestamp comparison
       // This works because the GPS PPS arrives at a KNOWN 1 Hz rate.
       // Each device timestamps the PPS edge with its own clock.
       // The difference in timestamps reveals the oscillator offset.
       drift_estimate update_from_pps(
           uhd::usrp::multi_usrp::sptr usrp,
           double rf_center_freq
       ) {
           auto pps0 = usrp->get_time_last_pps(0);
           auto pps1 = usrp->get_time_last_pps(1);

           // Store history for linear regression
           _pps_history.push_back({pps0, pps1});
           if (_pps_history.size() > MAX_HISTORY) {
               _pps_history.pop_front();
           }

           drift_estimate est;
           if (_pps_history.size() >= 2) {
               // Linear regression of PPS timestamps over time
               // Slope = relative clock rate (1.0 = identical, 1.000002 = +2ppm)
               est.frequency_offset_ppm = compute_pps_drift_regression();
               est.phase_rate_rad_per_s = 2.0 * M_PI * rf_center_freq
                                          * (est.frequency_offset_ppm / 1e6);
               est.last_update = uhd::time_spec_t::get_system_time();
               est.num_pps_samples = _pps_history.size();
           }

           return est;
       }

   private:
       struct pps_sample {
           uhd::time_spec_t dev0_time;
           uhd::time_spec_t dev1_time;
       };
       std::deque<pps_sample> _pps_history;
       static constexpr size_t MAX_HISTORY = 60;  // 1 minute of PPS data

       double compute_pps_drift_regression() {
           // Fit: dev1_frac_sec = slope * dev0_frac_sec + offset
           // slope - 1.0 = frequency offset in fractional units
           // Convert to ppm: (slope - 1.0) * 1e6
           //
           // With 10+ PPS samples and ~100 ns PPS jitter:
           // accuracy ≈ 100 ns / (N seconds) * 1e6 ppm
           //         ≈ 0.1/N ppm (e.g., 0.01 ppm after 10 seconds)

           size_t n = _pps_history.size();
           double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;

           for (size_t i = 0; i < n; ++i) {
               double x = _pps_history[i].dev0_time.get_real_secs()
                        - _pps_history[0].dev0_time.get_real_secs();
               double y = _pps_history[i].dev1_time.get_real_secs()
                        - _pps_history[0].dev1_time.get_real_secs();
               sum_x += x;
               sum_y += y;
               sum_xy += x * y;
               sum_xx += x * x;
           }

           double slope = (n * sum_xy - sum_x * sum_y)
                        / (n * sum_xx - sum_x * sum_x);
           double drift_ppm = (slope - 1.0) * 1e6;
           return drift_ppm;
       }
   };
   ```

   **Why PPS-based drift estimation is superior for this setup:**
   - No RF signals needed
   - Works during normal streaming (just read PPS timestamps)
   - Accuracy improves with time: ~0.01 ppm after 10 seconds
   - GPS gives us ground truth at 1 Hz — it's a free calibration signal
   - No need to tune devices to same frequency for calibration

   **Accuracy analysis:**

   ```
   PPS jitter from GPS module: ~100 ns (typical u-blox)
   B210 PPS timestamping resolution: 1/sample_rate ≈ 18 ns at 56 MS/s

   After N seconds of PPS data (linear regression):
   drift_accuracy ≈ sqrt(2) * PPS_jitter / N (seconds)

   N = 5 seconds:  accuracy ≈ 28 ns / 5 s = 0.006 ppm ✓
   N = 10 seconds: accuracy ≈ 28 ns / 10 s = 0.003 ppm ✓
   N = 30 seconds: accuracy ≈ 28 ns / 30 s = 0.001 ppm ✓

   This is MORE than enough for bandwidth aggregation!
   ```

2. **Calibration mode: cross-correlation verification (secondary)**

   For **initial verification only** — tune both devices to the **same** frequency temporarily to confirm PPS-based drift estimate:

   ```cpp
   double verify_drift_with_cross_correlation(
       uhd::usrp::multi_usrp::sptr usrp,
       double verification_freq,  // Tune both to this freq
       double sample_rate,
       size_t nsamps = 1048576  // ~19 ms at 56 MS/s
   ) {
       // Save current frequencies
       double saved_freq0 = usrp->get_rx_freq(0);
       double saved_freq1 = usrp->get_rx_freq(2);

       // Temporarily tune both to same frequency
       usrp->set_rx_freq(verification_freq, 0);
       usrp->set_rx_freq(verification_freq, 2);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));

       // Receive from both devices
       // (when tuned to same freq, they receive the same signal)
       // Cross-correlate to find time offset
       // Time offset change rate = frequency offset

       // ... receive and cross-correlate ...

       // Restore original frequencies
       usrp->set_rx_freq(saved_freq0, 0);
       usrp->set_rx_freq(saved_freq1, 2);

       return measured_drift_ppm;
   }
   ```

   **Important**: Cross-correlation only works when both devices receive the **same signal**. During normal bandwidth aggregation (different bands), this method cannot be used without retuning. Use the PPS method instead.

3. **Phase correction using Volk (efficient)**

   The core correction: compensate for the phase drift caused by the frequency offset between oscillators.

   ```cpp
   class phase_corrector {
   public:
       // Apply phase correction to compensate for oscillator drift
       // This corrects the effect of the frequency offset on the
       // received signal — the IF/baseband signal rotates because
       // the LO is off by drift_ppm from the reference device
       void correct(
           std::complex<float>* samples,
           size_t nsamps,
           double drift_ppm,
           double rf_center_freq,
           double sample_rate,
           double time_since_sync_seconds  // Time since last PPS sync
       ) {
           // Frequency offset at RF = drift_ppm * center_freq / 1e6
           // This manifests as a baseband frequency offset after mixing
           double freq_offset_hz = (drift_ppm / 1e6) * rf_center_freq;

           // Phase rate in radians per sample
           double phase_rate = 2.0 * M_PI * freq_offset_hz / sample_rate;

           // Initial phase = accumulated since last sync point
           double initial_phase = 2.0 * M_PI * freq_offset_hz
                                * time_since_sync_seconds;

           // Use Volk for efficient complex rotation
           // volk_32fc_s32fc_rotator_32fc does:
           //   output[i] = input[i] * phase * (phase_inc)^i
           lv_32fc_t phase_inc = lv_cmake(
               std::cos(-phase_rate), std::sin(-phase_rate));
           lv_32fc_t phase = lv_cmake(
               std::cos(-initial_phase), std::sin(-initial_phase));

           volk_32fc_s32fc_rotator_32fc(
               samples, samples, phase_inc, &phase, nsamps);
       }
   };
   ```

   **Why phase correction alone is sufficient (no resampling needed):**

   At 2 ppm drift and 56 MS/s sample rate:
   - Sample rate mismatch: 56e6 \* 2e-6 = 112 samples/second
   - Over a 1-second buffer: 112 samples slip out of 56 million = 0.0002%
   - Over a 100 ms buffer (typical FFT window): 11.2 samples slip

   For **frequency-domain stitching** (which this project does):
   - Each device produces an independent spectrum
   - The spectra are placed side-by-side in frequency
   - Sample rate mismatch means one spectrum is slightly stretched/compressed
   - At 2 ppm, the stretch is 2 parts per million — **negligible on a spectrum plot**
   - The phase offset IS significant and must be corrected for clean stitching

   **Resampling is only needed if** you need sample-level time alignment for
   coherent combination of overlapping bands. For adjacent-band stitching with
   guard bands, phase correction alone is sufficient.

4. **Optional: Fractional resampler for long captures**

   If captures run >30 seconds without recalibration, sample slip may become
   noticeable. Use a simple linear interpolator:

   ```cpp
   // Simple fractional delay for drift compensation
   // Only needed for very long captures or tight stitching
   void resample_linear(
       const std::complex<float>* input, size_t in_len,
       std::complex<float>* output, size_t out_len,
       double rate_ratio  // 1.0 + ppm/1e6
   ) {
       for (size_t i = 0; i < out_len; ++i) {
           double pos = i * rate_ratio;
           size_t idx = static_cast<size_t>(pos);
           double frac = pos - idx;

           if (idx + 1 < in_len) {
               output[i] = input[idx] * static_cast<float>(1.0 - frac)
                         + input[idx + 1] * static_cast<float>(frac);
           }
       }
   }
   ```

#### Deliverables:

- PPS-based drift estimation (primary, no RF needed)
- Cross-correlation verification mode (secondary)
- Volk-optimized phase correction
- Optional linear resampler
- Drift monitoring and logging

---

### Phase 4: Frequency-Domain Band Stitching (Weeks 11-14)

#### Tasks:

1. **FFT-based spectrum stitching**

   Each device receives an adjacent frequency band. We stitch them in the frequency domain:

   ```cpp
   class spectrum_stitcher {
   public:
       spectrum_stitcher(double sample_rate, size_t fft_size,
                         double guard_band_hz)
           : _sample_rate(sample_rate)
           , _fft_size(fft_size)
           , _guard_band_bins(
               static_cast<size_t>(guard_band_hz / (sample_rate / fft_size)))
       {
           // Allocate FFT plans (use FFTW or Volk)
           _window.resize(fft_size);
           create_blackman_harris_window(_window.data(), fft_size);
       }

       // Stitch two adjacent bands into one wider spectrum
       // Device 0: center_freq0, bandwidth = sample_rate
       // Device 1: center_freq1 = center_freq0 + sample_rate - guard_band
       void stitch(
           const std::complex<float>* buf0, size_t nsamps0,
           const std::complex<float>* buf1, size_t nsamps1,
           std::complex<float>* output_spectrum,
           size_t output_len
       ) {
           // Apply window functions
           apply_window(buf0, _windowed0.data(), _fft_size);
           apply_window(buf1, _windowed1.data(), _fft_size);

           // FFT each device's data
           fft_forward(_windowed0.data(), _spectrum0.data(), _fft_size);
           fft_forward(_windowed1.data(), _spectrum1.data(), _fft_size);

           // Place in output spectrum:
           //   output = [dev0_lower_bins | overlap_blend | dev1_upper_bins]
           size_t usable_bins = _fft_size - 2 * _guard_band_bins;

           // Device 0: take bins from guard_band to fft_size/2
           // (upper half of device 0's spectrum)
           size_t out_idx = 0;
           for (size_t i = _guard_band_bins; i < _fft_size - _guard_band_bins; ++i) {
               output_spectrum[out_idx++] = _spectrum0[i];
           }

           // Device 1: take bins from guard_band to fft_size/2
           for (size_t i = _guard_band_bins; i < _fft_size - _guard_band_bins; ++i) {
               output_spectrum[out_idx++] = _spectrum1[i];
           }
       }

   private:
       double _sample_rate;
       size_t _fft_size;
       size_t _guard_band_bins;
       std::vector<float> _window;
       std::vector<std::complex<float>> _windowed0, _windowed1;
       std::vector<std::complex<float>> _spectrum0, _spectrum1;
   };
   ```

2. **Guard band sizing**

   Guard bands serve two purposes:
   - **Filter rolloff**: B210 analog frontend rolls off at band edges
   - **Drift margin**: With PPS-only sync, the band edges shift slightly with drift

   ```cpp
   double calculate_guard_band_hz(double sample_rate, double drift_ppm) {
       // B210 analog filter rolloff: ~10% of bandwidth at edges
       double filter_guard = sample_rate * 0.10;

       // Drift margin: max frequency shift from oscillator offset
       // At 2 ppm, the LO is off by 2 ppm * center_freq
       // But the band edge only shifts by 2 ppm * sample_rate
       // (because sample rate determines bandwidth)
       double drift_guard = sample_rate * drift_ppm / 1e6;
       // At 56 MS/s, 5 ppm = 280 Hz — negligible!

       // Total guard band dominated by analog filter, NOT drift
       return filter_guard;  // ~5.6 MHz for 56 MS/s
       // Effective bandwidth per device: ~44.8 MHz
       // Two devices: ~89.6 MHz total
   }
   ```

   **Important realization**: The guard band is dominated by the B210's analog filter rolloff (~10%), not by oscillator drift. Even 5 ppm drift only shifts the band by 280 Hz at 56 MS/s — completely negligible compared to the MHz-scale filter rolloff.

3. **Overlap blending for adjacent bands**

   If devices are tuned with intentional frequency overlap, use weighted blending:

   ```cpp
   void blend_overlap_region(
       std::complex<float>* output,
       const std::complex<float>* spectrum0_upper,
       const std::complex<float>* spectrum1_lower,
       size_t overlap_bins
   ) {
       for (size_t i = 0; i < overlap_bins; ++i) {
           float w = static_cast<float>(i) / overlap_bins;  // 0 to 1
           // Raised-cosine blending
           float w0 = 0.5f * (1.0f + std::cos(M_PI * w));       // 1→0
           float w1 = 0.5f * (1.0f - std::cos(M_PI * w));       // 0→1
           output[i] = w0 * spectrum0_upper[i] + w1 * spectrum1_lower[i];
       }
   }
   ```

4. **Bonded RX streamer interface**

   ```cpp
   class bonded_rx_streamer {
   public:
       bonded_rx_streamer(
           uhd::usrp::multi_usrp::sptr usrp,
           double sample_rate,
           double center_freq,          // Center of combined band
           double total_bandwidth       // Desired total BW
       ) {
           _sample_rate = sample_rate;

           // Calculate per-device tuning
           // Device 0: center - BW/4
           // Device 1: center + BW/4
           double half_device_bw = sample_rate / 2.0;
           double freq0 = center_freq - half_device_bw + _guard_band / 2.0;
           double freq1 = center_freq + half_device_bw - _guard_band / 2.0;

           _receiver = std::make_unique<bonded_receiver>(
               usrp, sample_rate, freq0, freq1);
           _stitcher = std::make_unique<spectrum_stitcher>(
               sample_rate, FFT_SIZE, _guard_band);
           _drift_tracker = std::make_unique<pps_drift_tracker>();
           _phase_corrector = std::make_unique<phase_corrector>();
       }

       // Main receive function — returns stitched wideband data
       size_t recv(std::complex<float>* output, size_t nsamps,
                   uhd::rx_metadata_t& metadata, double timeout) {
           // 1. Receive from both devices
           int64_t offset = _receiver->recv_aligned(nsamps);

           // 2. Update drift estimate from PPS (non-blocking check)
           auto drift = _drift_tracker->get_latest_estimate();

           // 3. Apply phase correction to device 1's data
           double time_now = _receiver->get_stream_time();
           _phase_corrector->correct(
               _receiver->get_buffer(1), nsamps,
               drift.frequency_offset_ppm,
               _receiver->get_freq(1),
               _sample_rate, time_now);

           // 4. Stitch in frequency domain
           _stitcher->stitch(
               _receiver->get_buffer(0), nsamps,
               _receiver->get_buffer(1), nsamps,
               output, nsamps);

           return nsamps;
       }

   private:
       double _sample_rate;
       double _guard_band = 5.6e6;  // 10% of 56 MS/s
       static constexpr size_t FFT_SIZE = 4096;

       std::unique_ptr<bonded_receiver> _receiver;
       std::unique_ptr<spectrum_stitcher> _stitcher;
       std::unique_ptr<pps_drift_tracker> _drift_tracker;
       std::unique_ptr<phase_corrector> _phase_corrector;
   };
   ```

5. **Background PPS drift tracking thread**

   ```cpp
   // Runs in background, polls PPS timestamps every second
   void drift_tracking_thread(
       uhd::usrp::multi_usrp::sptr usrp,
       pps_drift_tracker* tracker,
       double rf_center_freq,
       std::atomic<bool>& running
   ) {
       while (running) {
           // Wait for next PPS edge
           auto last_pps = usrp->get_time_last_pps(0);
           while (usrp->get_time_last_pps(0).get_full_secs()
                  == last_pps.get_full_secs()) {
               std::this_thread::sleep_for(std::chrono::milliseconds(50));
               if (!running) return;
           }

           // Record PPS timestamps from both devices
           tracker->update_from_pps(usrp, rf_center_freq);
       }
   }
   ```

#### Deliverables:

- FFT-based spectrum stitching
- Guard band calculation (properly dominated by analog filter, not drift)
- Overlap blending
- Complete bonded RX streamer
- Background drift tracking

---

### Phase 5: Long-term Stability & Recalibration (Weeks 15-17)

#### Tasks:

1. **Continuous PPS monitoring and drift tracking**

   Since we have GPS PPS arriving every second, drift tracking is continuous and free:

   ```cpp
   class continuous_drift_monitor {
   public:
       void on_pps_tick(uhd::usrp::multi_usrp::sptr usrp) {
           auto pps0 = usrp->get_time_last_pps(0);
           auto pps1 = usrp->get_time_last_pps(1);

           _pps_log.push_back({pps0, pps1,
               std::chrono::steady_clock::now()});

           // Keep rolling window of last 60 samples
           if (_pps_log.size() > 60) _pps_log.pop_front();

           // Update drift estimate
           _current_drift_ppm = compute_drift_from_window();

           // Check for anomalies
           if (std::abs(_current_drift_ppm) > 10.0) {
               std::cerr << "WARNING: Drift exceeding 10 ppm ("
                         << _current_drift_ppm << " ppm). "
                         << "Check TCXO health." << std::endl;
           }

           // Log for post-processing
           if (_log_file.is_open()) {
               _log_file << pps0.get_real_secs() << ","
                         << pps1.get_real_secs() << ","
                         << _current_drift_ppm << std::endl;
           }
       }

       double get_current_drift_ppm() const { return _current_drift_ppm; }

   private:
       std::deque<pps_record> _pps_log;
       double _current_drift_ppm = 0.0;
       std::ofstream _log_file;
   };
   ```

2. **Periodic PPS re-sync (optional, for very long runs)**

   After many hours, accumulated numerical errors may warrant a time re-sync:

   ```cpp
   void periodic_resync(uhd::usrp::multi_usrp::sptr usrp,
                        double hours_between_resyncs = 4.0) {
       // Re-align device times at PPS edge
       // This does NOT interrupt streaming — it just resets the time counters
       // The drift tracker adapts immediately
       usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
       std::this_thread::sleep_for(std::chrono::seconds(2));

       // Reset drift tracker history (start fresh regression)
       _drift_tracker->reset_history();
   }
   ```

3. **Quality metrics**

   ```cpp
   struct bonding_quality_metrics {
       double drift_ppm;              // Current estimated drift
       double drift_accuracy_ppm;     // Estimated accuracy of drift estimate
       int pps_samples_used;          // Number of PPS edges in regression
       double time_since_sync_s;      // Seconds since last PPS time sync
       double phase_correction_rate;  // Rad/s being applied
       double guard_band_utilization; // How much of guard band is "used" by drift
       bool healthy;                  // Overall system health

       std::string to_string() const {
           std::stringstream ss;
           ss << "Drift: " << drift_ppm << " ppm"
              << " (±" << drift_accuracy_ppm << " ppm)"
              << " | PPS samples: " << pps_samples_used
              << " | Phase rate: " << phase_correction_rate << " rad/s"
              << " | " << (healthy ? "HEALTHY" : "DEGRADED");
           return ss.str();
       }
   };
   ```

#### Deliverables:

- Continuous PPS-based drift monitoring
- Anomaly detection
- Optional periodic re-sync
- Quality metrics and logging

---

### Phase 6: Testing & Validation (Weeks 18-22)

#### Hardware Test Setup:

```
GPS Module (u-blox NEO-M8N or similar)
    |
    PPS out (SMA)
    |
    Y-splitter
    |           |
    B210 #0     B210 #1
    PPS IN      PPS IN
    |           |
    USB 3.0     USB 3.0
    |           |
    Host PC (Linux, USB 3.0 ports on separate controllers recommended)
```

#### Test 1: PPS Synchronization Verification

```
Procedure:
1. Connect GPS PPS to both B210s
2. Run set_time_unknown_pps()
3. Read get_time_last_pps() from both boards every second for 60 seconds
4. Compare timestamps

Success Criteria:
- All PPS timestamps match to within 1 µs (typ. ~100 ns)
- No missed PPS edges
```

#### Test 2: Drift Estimation Accuracy

```
Procedure:
1. Sync both devices via PPS
2. Run PPS drift tracker for 60 seconds
3. Plot drift estimate over time
4. Verify convergence (should stabilize after ~10 PPS edges)

Success Criteria:
- Drift estimate converges to stable value (within 0.01 ppm jitter)
- Typical values: 0.5-3.0 ppm between two B210s
- Linear regression R² > 0.999
```

#### Test 3: Spectrum Stitching Quality

```
Procedure:
1. Connect FM antenna to both devices via splitter
2. Tune:
   - Device 0: 88.0 MHz center (receives 60-116 MHz)
   - Device 1: 140.0 MHz center (receives 112-168 MHz)
   - Overlap: 112-116 MHz (4 MHz overlap region)
3. Record 10 seconds of data from both devices
4. Stitch spectra and compare against single-device wideband capture

Success Criteria:
- No visible discontinuity at stitch boundary
- SNR in overlap region within 1 dB of single-device reference
- FM stations in stitched spectrum match frequency precisely
```

#### Test 4: Long-Duration Stability (8-hour Test)

```
Procedure:
1. Set up stitched wideband reception
2. Inject FM broadcast signal at band boundary (known frequency)
3. Run for 8 hours, log:
   - Drift estimate vs time
   - Phase correction applied
   - Signal quality at band boundary
   - Any overflows or sync errors

Success Criteria:
- Drift tracked continuously (no divergence)
- Signal quality at boundary: <2 dB degradation over 8 hours
- No unrecoverable errors
- Drift typically shows diurnal pattern (temperature-driven)
```

#### Test 5: Environmental Stress

```
Procedure:
1. Place one B210 in direct sunlight / near heat source
2. Monitor drift as temperature changes
3. Verify drift tracker follows the change

Expected Results:
- Drift increases by 0.5-2 ppm per 10°C temperature change
- Drift tracker follows within 2-3 PPS cycles (2-3 seconds)
- Phase correction adapts smoothly
```

#### Test 6: Non-Coherent Multi-Band (Easiest Case)

```
Procedure:
1. Tune Device 0 to 900 MHz (GSM band)
2. Tune Device 1 to 2.4 GHz (WiFi band)
3. Stream independently — no stitching needed, just time alignment

Success Criteria:
- Both streams run simultaneously without errors
- Timestamps from both devices are consistent
- No drift compensation needed (bands are independent)
- This is the simplest use case and should work immediately
```

#### Performance Metrics to Collect

| Metric                    | Target      | How to Measure                                       |
| ------------------------- | ----------- | ---------------------------------------------------- |
| PPS sync accuracy         | <1 µs       | Compare `get_time_last_pps()` across boards          |
| Drift estimation accuracy | <0.05 ppm   | Compare PPS method vs cross-correlation verification |
| Drift convergence time    | <10 seconds | Time from startup to stable estimate                 |
| Phase correction overhead | <5% CPU     | Profile `volk_32fc_s32fc_rotator_32fc` calls         |
| FFT stitching overhead    | <15% CPU    | Profile FFT + windowing + blending                   |
| Stitch boundary artifact  | <-40 dBc    | Measure with broadband noise source                  |
| Max continuous run time   | >8 hours    | Long-duration test                                   |
| Overflow rate             | <1 per hour | Count `ERROR_CODE_OVERFLOW` events                   |

#### Deliverables:

- Complete test suite with pass/fail criteria
- Performance benchmark results
- Long-term stability data
- Environmental stress test results

---

## 6. PPS-Only Synchronization Strategy

### 6.1 Hardware Setup

**Your Setup:**

```
GPS Module (1PPS output)
    |
    PPS out (SMA)
    |
    SMA Y-splitter (or buffer for 3+ devices)
    |                |
    B210 #0          B210 #1
    PPS IN (SMA)     PPS IN (SMA)
    |                |
    USB 3.0          USB 3.0
    |                |
    Host PC (Linux)
```

**GPS Module Requirements:**

- Must have PPS output (1 pulse per second)
- 3.3V CMOS compatible with B210 PPS input
- Typical PPS accuracy: 10-100 ns (any GPS module is adequate)
- Do NOT need timing-grade GPS — even $15 modules have <1 µs PPS jitter

**PPS Signal Distribution:**

- For 2 devices: Simple SMA Y-splitter ($5) is sufficient
- For 3+ devices: Use CMOS buffer IC (74HC14 or 74LVC1G17) to avoid signal degradation
- Cable length: <10 meters (keep jitter < 50 ns, which is negligible)

### 6.2 PPS Signal Quality

| Parameter         | Requirement | Typical GPS Module |
| ----------------- | ----------- | ------------------ |
| **Voltage Level** | 3.3V CMOS   | 3.3V CMOS          |
| **Rise Time**     | <100 ns     | 10-50 ns           |
| **Jitter**        | <1 µs       | 10-100 ns          |
| **Rate**          | 1 Hz        | 1 Hz               |
| **Pulse Width**   | >10 µs      | 100 ms typical     |

### 6.3 Synchronization Sequence (Detailed)

```cpp
void initialize_bonded_pps_only(
    const std::string& serial0,
    const std::string& serial1
) {
    // 1. Open both devices as one multi_usrp
    auto usrp = uhd::usrp::multi_usrp::make(
        "serial=" + serial0 + ",serial=" + serial1
    );
    std::cout << "Opened " << usrp->get_num_mboards()
              << " boards" << std::endl;

    // 2. Configure clock and time sources
    for (size_t mb = 0; mb < usrp->get_num_mboards(); ++mb) {
        usrp->set_clock_source("internal", mb);  // Each board uses own TCXO
        usrp->set_time_source("external", mb);    // PPS from GPS on SMA
    }

    // 3. Verify PPS is present on all boards
    for (size_t mb = 0; mb < usrp->get_num_mboards(); ++mb) {
        auto t0 = usrp->get_time_last_pps(mb);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        auto t1 = usrp->get_time_last_pps(mb);
        if (t1.get_full_secs() == t0.get_full_secs()) {
            throw uhd::runtime_error(
                "No PPS detected on board " + std::to_string(mb)
                + ". Check GPS module and SMA cable.");
        }
        std::cout << "Board " << mb << ": PPS OK" << std::endl;
    }

    // 4. Synchronize times at PPS edge
    // set_time_unknown_pps() handles edge detection internally:
    //   - Polls get_time_last_pps() until it transitions
    //   - Immediately calls set_time_next_pps(0.0, ALL_MBOARDS)
    //   - Waits for next PPS to latch the time
    usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 5. Verify alignment
    auto t0 = usrp->get_time_last_pps(0);
    auto t1 = usrp->get_time_last_pps(1);
    double diff = std::abs(t0.get_real_secs() - t1.get_real_secs());
    std::cout << "Time alignment: " << diff * 1e9 << " ns" << std::endl;

    if (diff > 0.001) {
        throw uhd::runtime_error("PPS time sync failed: "
            + std::to_string(diff * 1e6) + " µs difference");
    }

    std::cout << "PPS-only synchronization complete" << std::endl;
    std::cout << "NOTE: Devices use independent TCXOs." << std::endl;
    std::cout << "Drift tracking will compensate for oscillator offset."
              << std::endl;
}
```

### 6.4 Why USB Jitter Doesn't Matter

A common concern: "USB has milliseconds of jitter — how can we align samples?"

**Answer**: We don't use USB timing for alignment. We use **hardware timestamps**:

1. The B210 FPGA timestamps every packet with its local clock
2. After PPS sync, both devices' clocks start at time 0.000000
3. `rx_metadata_t.time_spec` tells us the exact time of the first sample in each packet
4. We compare timestamps to align samples — USB latency is irrelevant

```
Timeline:
                PPS edge       PPS edge       PPS edge
                   |              |              |
Device 0 clock:    0.000000      1.000000       2.000000
Device 1 clock:    0.000000      1.000002       2.000004  (2 ppm fast)
                   |              |              |
USB delivers       ~3ms later    ~5ms later     ~2ms later  (variable!)
packets to host

But metadata.time_spec = device clock time, NOT USB arrival time
So alignment is based on device timestamps, immune to USB jitter
```

---

## 7. Frequency Drift Management

### 7.1 Drift Estimation: PPS Timestamp Method (Primary)

This is the **single best method** for PPS-only bonding. It requires no RF signals, no external equipment, and runs continuously during normal operation.

**How it works:**

1. The GPS PPS pulse arrives at both B210s simultaneously (within ~100 ns)
2. Each B210's FPGA timestamps the PPS edge using its local TCXO-derived clock
3. Over time, the timestamps diverge because the TCXOs run at slightly different rates
4. A linear regression of the divergence gives the frequency offset in ppm

```cpp
// Complete PPS-based drift estimator
class pps_drift_estimator {
public:
    struct result {
        double drift_ppm;           // Frequency offset (dev1 relative to dev0)
        double uncertainty_ppm;     // 1-sigma uncertainty of estimate
        double phase_rate_rad_s;    // For phase correction at given RF freq
        size_t num_samples;         // PPS edges used in regression
        bool valid;                 // Enough data for reliable estimate
    };

    result update(uhd::usrp::multi_usrp::sptr usrp, double rf_freq) {
        // Read PPS timestamps from both boards
        double t0 = usrp->get_time_last_pps(0).get_real_secs();
        double t1 = usrp->get_time_last_pps(1).get_real_secs();

        if (_history.empty() || t0 != _history.back().t0) {
            // New PPS edge — store it
            _history.push_back({t0, t1});
            if (_history.size() > _max_history) {
                _history.pop_front();
            }
        }

        result r;
        r.num_samples = _history.size();
        r.valid = (r.num_samples >= 3);

        if (!r.valid) {
            r.drift_ppm = 0.0;
            r.uncertainty_ppm = 999.0;
            r.phase_rate_rad_s = 0.0;
            return r;
        }

        // Linear regression: t1 = slope * t0 + offset
        // drift_ppm = (slope - 1.0) * 1e6
        double n = r.num_samples;
        double sx = 0, sy = 0, sxy = 0, sxx = 0;
        double t0_base = _history.front().t0;

        for (auto& h : _history) {
            double x = h.t0 - t0_base;
            double y = h.t1 - _history.front().t1;
            sx += x; sy += y; sxy += x * y; sxx += x * x;
        }

        double denom = n * sxx - sx * sx;
        if (std::abs(denom) < 1e-20) {
            r.valid = false;
            return r;
        }

        double slope = (n * sxy - sx * sy) / denom;
        r.drift_ppm = (slope - 1.0) * 1e6;

        // Uncertainty estimate
        // PPS jitter ~100 ns, timestamping resolution ~18 ns at 56 MS/s
        // Combined uncertainty per point: ~102 ns
        // After N points over T seconds:
        //   slope_uncertainty ≈ sqrt(2) * 102e-9 / T
        //   ppm_uncertainty ≈ slope_uncertainty * 1e6
        double T = _history.back().t0 - _history.front().t0;
        if (T > 0) {
            r.uncertainty_ppm = (1.414 * 102e-9 / T) * 1e6;
        } else {
            r.uncertainty_ppm = 999.0;
        }

        // Phase rate for correction
        r.phase_rate_rad_s = 2.0 * M_PI * rf_freq * (r.drift_ppm / 1e6);

        return r;
    }

    void reset() { _history.clear(); }

private:
    struct pps_point { double t0, t1; };
    std::deque<pps_point> _history;
    size_t _max_history = 120;  // 2 minutes of PPS data
};
```

**Expected accuracy over time:**

| PPS Edges | Time Span  | Accuracy (ppm) | Phase Error at 2.4 GHz |
| --------- | ---------- | -------------- | ---------------------- |
| 3         | 3 seconds  | ~0.05          | ~43° per second        |
| 5         | 5 seconds  | ~0.03          | ~26° per second        |
| 10        | 10 seconds | ~0.015         | ~13° per second        |
| 30        | 30 seconds | ~0.005         | ~4° per second         |
| 60        | 60 seconds | ~0.002         | ~2° per second         |

After 10 seconds, drift accuracy is well under 0.1 ppm — more than sufficient for spectrum stitching.

### 7.2 Drift Estimation: Cross-Correlation Method (Verification Only)

**IMPORTANT**: This method requires both devices to receive the **same signal**. During normal bandwidth aggregation (adjacent bands), this is NOT possible without temporarily retuning. Use this only for initial verification.

```cpp
// ONE-TIME VERIFICATION: Tune both to same freq, cross-correlate
double verify_drift_xcorr(
    uhd::usrp::multi_usrp::sptr usrp,
    double verify_freq, double sample_rate
) {
    // Temporarily tune both devices to the same frequency
    double orig_freq0 = usrp->get_rx_freq(0);
    double orig_freq1 = usrp->get_rx_freq(2);

    usrp->set_rx_freq(verify_freq, 0);
    usrp->set_rx_freq(verify_freq, 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Capture two sequential bursts, measure phase change between them
    // ... (receive and cross-correlate) ...

    // Restore original frequencies
    usrp->set_rx_freq(orig_freq0, 0);
    usrp->set_rx_freq(orig_freq1, 2);

    return measured_drift_ppm;
}
```

### 7.3 Drift Compensation: Phase Correction

The primary correction mechanism: apply a per-sample phase rotation to undo the effect of the oscillator offset.

**Why this is the right approach:**

The oscillator offset of device 1 causes two effects:

1. **LO offset**: The local oscillator is off by `drift_ppm * center_freq` Hz
   → Manifests as a baseband frequency offset (phase rotation)
   → **Must correct** for clean stitching
2. **Sample rate offset**: The ADC clock is off by `drift_ppm * sample_rate` Hz
   → Manifests as slight time stretch (resampling needed to fix)
   → At 2 ppm and 56 MS/s: 112 samples/second drift
   → Over a 4096-sample FFT window (~73 µs): **0.008 samples** — negligible!
   → **Don't need to correct** for spectrum stitching

Therefore: **Phase correction alone is sufficient. No resampling needed.**

```cpp
// Efficient phase correction using Volk
void apply_phase_correction(
    std::complex<float>* samples,
    size_t nsamps,
    double drift_ppm,
    double rf_center_freq,
    double sample_rate,
    double& accumulated_phase  // Track between calls (in/out)
) {
    // Calculate correction frequency
    double freq_offset_hz = (drift_ppm / 1e6) * rf_center_freq;
    double phase_inc_per_sample = 2.0 * M_PI * freq_offset_hz / sample_rate;

    // Use Volk rotator for SIMD-accelerated correction
    lv_32fc_t phase_inc = lv_cmake(
        std::cos(-phase_inc_per_sample),
        std::sin(-phase_inc_per_sample)
    );
    lv_32fc_t phase = lv_cmake(
        std::cos(-accumulated_phase),
        std::sin(-accumulated_phase)
    );

    volk_32fc_s32fc_rotator_32fc(
        samples, samples, phase_inc, &phase, nsamps
    );

    // Update accumulated phase for next call
    accumulated_phase += phase_inc_per_sample * nsamps;
    // Keep in [-π, π] to avoid floating point issues
    accumulated_phase = std::fmod(accumulated_phase + M_PI, 2.0 * M_PI) - M_PI;
}
```

**Performance**: `volk_32fc_s32fc_rotator_32fc` uses SIMD (SSE/AVX) and processes ~500 MS/s on modern CPUs. Correcting 56 MS/s uses ~11% of one core.

### 7.4 When Is Resampling Actually Needed?

Only if you need **time-domain sample alignment** for coherent combination of overlapping bands. For adjacent-band stitching with guard bands, resampling is unnecessary.

| Use Case                    | Resampling Needed? | Why                                                          |
| --------------------------- | ------------------ | ------------------------------------------------------------ |
| Adjacent-band stitching     | **No**             | FFT windows are short; 0.008 samples/FFT drift is negligible |
| Non-coherent multi-band     | **No**             | Bands are independent                                        |
| Coherent overlapping bands  | **Yes**            | Need sample-level alignment for constructive addition        |
| Long captures without recal | **Maybe**          | After 100+ seconds, sample slip may become noticeable        |

If resampling IS needed, a simple linear interpolator suffices for 2 ppm correction:

```cpp
// Only use if sample-level alignment is required
void resample_compensate(
    const std::complex<float>* in, size_t in_len,
    std::complex<float>* out, size_t out_len,
    double ppm_correction
) {
    double ratio = 1.0 + (ppm_correction / 1e6);
    for (size_t i = 0; i < out_len; ++i) {
        double pos = i * ratio;
        size_t idx = static_cast<size_t>(pos);
        float frac = static_cast<float>(pos - idx);
        if (idx + 1 < in_len) {
            out[i] = in[idx] * (1.0f - frac) + in[idx + 1] * frac;
        }
    }
}
```

### 7.5 Drift Compensation Summary

```
For each buffer of samples from Device 1:

1. Get current drift estimate from PPS tracker
   (background thread updates every PPS edge = every second)

2. Apply phase correction (Volk rotator)
   - Compensates LO offset
   - ~11% CPU at 56 MS/s
   - This is the ONLY correction needed for spectrum stitching

3. FFT and stitch with Device 0's spectrum

No resampling. No pilot tones. No cross-correlation during streaming.
The GPS PPS signal does all the calibration work automatically.
```

---

## 8. Challenges & Mitigations (PPS-Only Specific)

### 8.1 Challenge: USB Bandwidth and Host Performance

**Problem**: Two B210s at 56 MS/s each = 112 MS/s total × 4 bytes/sample × 2 (I+Q already in complex) = ~448 MB/s USB throughput, plus FFT processing.

**Impact**:

- May exceed USB 3.0 bandwidth if both devices are on the same USB controller
- Processing overhead: FFT + phase correction + stitching = significant CPU load

**Mitigations**:

1. **Use separate USB controllers** for each B210 (check with `lsusb -t`)
2. **Reduce sample rate** if full 56 MS/s not needed (e.g., 20 MS/s per device = 40 MS/s combined still gives 36 MHz usable BW)
3. **Use `uhd::transport::zero_copy`** buffers to minimize copies
4. **Thread the processing pipeline**: receive thread → processing thread → output thread

### 8.2 Challenge: Oscillator Drift Rate Changes

**Problem**: TCXO drift isn't constant — it changes with temperature (~0.1 ppm/°C).

**Impact**:

- Phase correction based on stale drift estimate degrades over time
- Temperature transients (e.g., startup, sun exposure) cause rapid drift changes

**Mitigations**:

1. **PPS drift tracker updates every second** → drift estimate is always <1 second old
2. **Linear regression over 10-30 second window** → captures trend, not just instant value
3. **Allow 5-10 minute warm-up** at startup for TCXO to stabilize
4. **Keep devices at similar temperatures** (co-locate, use fan if needed)
5. **No temperature sensor needed** — PPS timestamps directly measure the effect of temperature on drift

### 8.3 Challenge: PPS Signal Loss

**Problem**: GPS module loses satellite lock → PPS stops → time bases diverge.

**Impact**:

- Drift tracker stops getting new PPS data
- Phase correction continues with last known drift estimate (OK for ~30 seconds)
- After minutes, accumulated error becomes significant

**Mitigations**:

1. **Detect PPS loss immediately** (monitor `get_time_last_pps()` changes)
2. **Hold last drift estimate** — TCXO drift changes slowly, so last estimate is valid for ~30-60 seconds
3. **Warn user** when PPS loss exceeds 30 seconds
4. **Auto-resync** when PPS recovers
5. **Log PPS status** for post-processing quality assessment

```cpp
bool check_pps_health(uhd::usrp::multi_usrp::sptr usrp) {
    static uhd::time_spec_t last_pps;
    auto current_pps = usrp->get_time_last_pps(0);

    if (current_pps.get_full_secs() == last_pps.get_full_secs()) {
        // PPS hasn't changed — might be missing
        return false;  // Caller should check how long it's been
    }
    last_pps = current_pps;
    return true;
}
```

### 8.4 Challenge: Not Suitable for Coherent Combination

**Problem**: PPS-only cannot achieve phase coherence between devices. Applications requiring constructive signal combination (beamforming, coherent MIMO) won't work.

**Impact**:

- Cannot combine overlapping bands coherently (signals don't add constructively)
- Phase noise is uncorrelated between devices

**Mitigations**:

1. **Design for non-coherent stitching** — place bands adjacent, not overlapping
2. **Use guard bands** at boundaries (dominated by analog filter rolloff anyway)
3. **Document clearly**: PPS-only is for bandwidth extension, not coherent gain
4. **Upgrade path**: If coherence is needed, switch to 10MHz+PPS

### 8.5 Challenge: Overflow Handling

**Problem**: If host can't consume data fast enough, USB buffers overflow. With two devices, overflows may occur at different times, complicating sample alignment.

**Impact**:

- Lost samples create gaps in one device's stream
- Timestamp alignment becomes offset by the gap

**Mitigations**:

1. **Hardware timestamps recover automatically** — after an overflow, the next packet's `time_spec` shows the correct time, so alignment resumes
2. **Detect overflows via `rx_metadata_t.error_code == ERROR_CODE_OVERFLOW`**
3. **Use larger USB buffers**: `args["recv_buff_size"] = "..." `
4. **Reduce sample rate** if overflows are frequent
5. **Use dedicated USB 3.0 ports** on separate controllers

---

## 9. Comparison: PPS-Only vs. 10MHz+PPS

### 9.1 Side-by-Side Comparison

| Aspect                      | PPS-Only                                   | 10MHz + PPS                     | Winner               |
| --------------------------- | ------------------------------------------ | ------------------------------- | -------------------- |
| **Hardware Cost**           | $50-100 (GPS + cables)                     | $500-1500 (GPSDO + splitter)    | PPS-Only             |
| **Setup Complexity**        | Low (1 SMA cable per device)               | Medium (2 cables + RF splitter) | PPS-Only             |
| **Time Sync Accuracy**      | ~100 ns (GPS PPS jitter)                   | ~10 ns (locked oscillators)     | 10MHz+PPS            |
| **Frequency Accuracy**      | 0.01 ppm (after PPS-based cal)             | <0.001 ppm (hardware-locked)    | 10MHz+PPS            |
| **Phase Coherence**         | Software-corrected (good for stitching)    | Hardware-locked (excellent)     | 10MHz+PPS            |
| **Sample Rate Match**       | Differs by ±4 ppm                          | Identical                       | 10MHz+PPS            |
| **Drift Compensation**      | Phase rotation only (low CPU)              | Not needed                      | 10MHz+PPS            |
| **Coherent Combination**    | Not possible                               | Excellent                       | 10MHz+PPS            |
| **Adjacent-Band Stitching** | Good (with phase correction)               | Excellent                       | 10MHz+PPS (marginal) |
| **Non-Coherent Multi-Band** | Excellent (trivial)                        | Excellent (overkill)            | Tie                  |
| **Software Complexity**     | Moderate (drift tracker + phase corrector) | Low                             | 10MHz+PPS            |
| **Portability**             | Excellent (GPS works everywhere)           | Limited (need lab equipment)    | PPS-Only             |

### 9.2 Decision Guide

```
USE PPS-ONLY WHEN:
  ✓ Budget constrained (<$100 for sync hardware)
  ✓ Adjacent-band stitching for wideband monitoring
  ✓ Non-coherent multi-band reception (easiest case)
  ✓ Portable/field deployment
  ✓ Application tolerates <0.05 ppm uncertainty (after PPS cal)
  ✓ No need for phase-coherent combination

USE 10MHz+PPS WHEN:
  ✓ Coherent MIMO / beamforming needed
  ✓ Phase-locked overlapping band combination
  ✓ Frequency accuracy critical (<0.001 ppm)
  ✓ Long-term phase stability required
  ✓ Lab environment with available reference equipment
```

### 9.3 Realistic Performance Expectations

```
Test: ~90 MHz Usable Bandwidth (2x B210 @ 56 MS/s, ~10% guard bands)

Metric                     | 10MHz+PPS  | PPS-Only   | Notes
---------------------------|------------|------------|------
Time sync (ns)             | ~10        | ~100       | Both sub-µs
Freq offset (ppm)          | <0.001     | <0.01*     | *After PPS cal
Stitch boundary artifact   | <-50 dBc   | <-40 dBc   | Phase correction quality
CPU overhead               | ~15%       | ~25%       | Phase corrector adds ~10%
Setup time                 | ~20 min    | ~5 min     | PPS-only much simpler
Setup cost                 | ~$800      | ~$50       | 16x cost difference
Usable BW per device       | ~45 MHz    | ~45 MHz    | Same (limited by analog filter)
Total usable BW            | ~90 MHz    | ~90 MHz    | Same!

Key insight: For adjacent-band stitching, the performance difference
is small. The stitch boundary artifact is the main differentiator,
and -40 dBc is acceptable for most monitoring applications.
```

---

## 10. Implementation Recommendations

### 10.1 Development Strategy

**Phase 1: Get Streaming Working (Weeks 1-6)**

- Open two B210s as multi_usrp
- Sync time via PPS using `set_time_unknown_pps()`
- Stream from both simultaneously with timed start
- Verify timestamp alignment
- This phase uses only existing UHD API — no new code needed

**Phase 2: Add Drift Tracking (Weeks 7-10)**

- Implement PPS-based drift estimator
- Background thread polls PPS timestamps every second
- Log drift estimates for analysis
- Verify against cross-correlation method (temporary retune)
- ~500 lines of new code

**Phase 3: Implement Phase Correction + Stitching (Weeks 11-14)**

- Phase corrector using Volk rotator
- FFT-based spectrum stitcher
- Guard band management
- Quality metrics
- ~1500 lines of new code

**Phase 4: Testing & Hardening (Weeks 15-22)**

- Systematic testing (see Phase 6 above)
- Long-duration stability tests
- Overflow recovery testing
- Performance benchmarking
- Documentation

### 10.2 Code Organization

```
lib/usrp/bonded/
├── bonded_receiver.cpp          # Dual-device streaming + timestamp alignment
├── bonded_receiver.hpp
├── pps_drift_estimator.cpp      # PPS-based drift tracking (primary method)
├── pps_drift_estimator.hpp
├── phase_corrector.cpp          # Volk-based phase rotation
├── phase_corrector.hpp
├── spectrum_stitcher.cpp        # FFT-based band stitching
├── spectrum_stitcher.hpp
├── bonded_rx_streamer.cpp       # High-level bonded RX interface
├── bonded_rx_streamer.hpp
└── CMakeLists.txt

examples/
├── bonded_wideband_rx.cpp       # Basic wideband reception example
├── bonded_spectrum_monitor.cpp  # Real-time spectrum display
└── bonded_drift_monitor.cpp     # Drift tracking diagnostic tool

tests/
├── pps_drift_test.cpp           # Unit test for drift estimator
├── phase_corrector_test.cpp     # Unit test for phase correction
└── bonded_integration_test.cpp  # Integration test with hardware
```

### 10.3 User API

```cpp
// Minimal user-facing API
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>

int main() {
    // 1. Open devices (standard UHD)
    auto usrp = uhd::usrp::multi_usrp::make("serial=ABC123,serial=DEF456");

    // 2. Configure PPS sync
    for (size_t mb = 0; mb < usrp->get_num_mboards(); ++mb) {
        usrp->set_clock_source("internal", mb);
        usrp->set_time_source("external", mb);
    }
    usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 3. Create bonded receiver
    double sample_rate = 56e6;
    double center_freq = 2.4e9;         // Center of combined band
    bonded_rx_streamer bonded(usrp, sample_rate, center_freq);

    // 4. Start streaming
    bonded.start();

    // 5. Receive stitched wideband data
    std::vector<std::complex<float>> buffer(65536);
    uhd::rx_metadata_t md;
    while (running) {
        size_t n = bonded.recv(buffer.data(), buffer.size(), md, 1.0);
        // buffer now contains ~90 MHz of stitched spectrum data
        process(buffer.data(), n);
    }

    bonded.stop();
    return 0;
}
```

### 10.4 Dependencies

- **UHD 4.0+** (base framework — already in this workspace)
- **Volk** (SIMD math — already a UHD dependency)
- **FFTW3** (FFT for spectrum stitching — widely available, `apt install libfftw3-dev`)
- **C++14** (GCC 7+, Clang 5+)
- No additional libraries needed

### 10.5 Key Design Decisions

| Decision                | Choice                                     | Rationale                                                               |
| ----------------------- | ------------------------------------------ | ----------------------------------------------------------------------- |
| Drift estimation method | PPS timestamps (primary)                   | No RF signals needed, works during normal operation, ~0.01 ppm accuracy |
| Drift correction method | Phase rotation only (no resampling)        | Sample rate offset is negligible for FFT-based stitching                |
| FFT library             | FFTW3                                      | Widely available, heavily optimized, supports in-place transforms       |
| Phase rotation          | Volk `rotator_32fc`                        | SIMD-accelerated, already a UHD dependency                              |
| Guard band sizing       | 10% of sample rate (analog filter limited) | Drift contribution is negligible (<300 Hz at 56 MS/s)                   |
| PPS sync method         | `set_time_unknown_pps()`                   | UHD handles edge detection automatically                                |
| Multi-device API        | Single `multi_usrp`                        | UHD already supports this, simpler than managing separate instances     |

---

## 11. Timeline and Milestones

| Week  | Milestone           | Deliverable                                          | Risk   |
| ----- | ------------------- | ---------------------------------------------------- | ------ |
| 1-2   | Hardware setup      | GPS PPS connected, both B210s responding             | Low    |
| 3-4   | PPS time sync       | `set_time_unknown_pps()` working, timestamps aligned | Low    |
| 5-6   | Dual streaming      | Both devices streaming simultaneously, timed start   | Low    |
| 7-8   | PPS drift estimator | Background drift tracking, accuracy verified         | Medium |
| 9-10  | Phase corrector     | Volk-based correction working, drift compensated     | Medium |
| 11-12 | FFT stitcher        | Spectra stitched, guard bands managed                | Medium |
| 13-14 | Bonded streamer     | Complete pipeline: recv → correct → stitch → output  | Medium |
| 15-16 | Integration testing | End-to-end tests passing                             | Medium |
| 17-18 | Long-duration tests | 8-hour stability verified                            | Low    |
| 19-20 | Performance tuning  | CPU profiling, buffer sizing optimization            | Low    |
| 21-22 | Documentation       | User guide, examples, API docs                       | Low    |

**Total: 22 weeks (~5.5 months)**

**Risk assessment**: Weeks 7-14 carry the most risk (novel code). Weeks 1-6 use only existing UHD API and should be straightforward. Weeks 15-22 are testing and polish.

---

## 12. Appendix: Quick Reference

### 12.1 Hardware Checklist

- [ ] 2x USRP B210
- [ ] GPS module with PPS output (any module with SMA PPS, e.g., u-blox NEO-M8N)
- [ ] GPS active antenna
- [ ] SMA Y-splitter (for PPS distribution to 2 devices)
- [ ] 2x SMA cables (GPS PPS → Y-splitter → B210 PPS inputs)
- [ ] 2x USB 3.0 cables
- [ ] Host PC with 2 USB 3.0 ports (ideally on separate controllers)
- [ ] Linux OS with UHD 4.0+ installed

### 12.2 Key UHD API Calls

```cpp
// Multi-device creation
auto usrp = multi_usrp::make("serial=...,serial=...");

// PPS-only sync configuration
usrp->set_clock_source("internal", mb);   // Own TCXO
usrp->set_time_source("external", mb);     // GPS PPS
usrp->set_time_unknown_pps(time_spec_t(0.0));  // Sync at PPS edge
usrp->get_time_last_pps(mb);              // Read PPS-stamped time
usrp->get_time_now(mb);                   // Read current device time

// Streaming
stream_cmd.stream_now = false;             // Timed start
stream_cmd.time_spec = future_time;        // All devices start together
rx_metadata_t.time_spec;                   // Hardware timestamp per packet
```

### 12.3 Performance Targets

| Parameter                 | Target           | Notes                                  |
| ------------------------- | ---------------- | -------------------------------------- |
| PPS sync accuracy         | <1 µs            | GPS module dependent (~100 ns typical) |
| Drift estimation accuracy | <0.02 ppm        | After 10 PPS edges                     |
| Phase correction CPU      | <15% of one core | Volk SIMD at 56 MS/s                   |
| FFT stitching CPU         | <20% of one core | FFTW3 at 56 MS/s, 4096-point FFT       |
| Total CPU                 | <50% of one core | Plus overheads                         |
| Stitch artifact           | <-40 dBc         | At band boundary                       |
| Usable bandwidth          | ~90 MHz          | 2× B210 at 56 MS/s, 10% guard bands    |
| Max overflow rate         | <1 per hour      | With proper USB controller setup       |

### 12.4 Troubleshooting

| Symptom                   | Cause                                     | Fix                                                    |
| ------------------------- | ----------------------------------------- | ------------------------------------------------------ |
| "No PPS detected"         | GPS module not locked, cable disconnected | Check GPS antenna has sky view, verify SMA connections |
| Large time offset (>1 ms) | PPS signal not reaching one device        | Check Y-splitter, try direct connection                |
| Drift estimate unstable   | Too few PPS samples                       | Wait 10+ seconds for convergence                       |
| Drift > 10 ppm            | Defective TCXO or wrong clock source      | Verify `set_clock_source("internal")` was called       |
| Overflow errors           | USB bandwidth exceeded                    | Use separate USB controllers, reduce sample rate       |
| Stitch artifacts          | Phase correction not applied              | Check drift estimator output, verify Volk install      |
| Poor spectrum edges       | Analog filter rolloff                     | Increase guard band to 12-15%                          |

---

**Document Version**: 2.0
**Last Updated**: March 10, 2026
**Status**: Ready for Implementation
**Key Changes from v1.0**:

- Corrected: USB jitter does NOT affect sample alignment (hardware timestamps)
- Corrected: Cross-correlation only works when devices receive same signal
- Corrected: Guard bands dominated by analog filter, not oscillator drift
- Corrected: Resampling unnecessary for adjacent-band stitching
- Added: PPS timestamp-based drift estimation (primary method, ~0.01 ppm)
- Added: UHD API integration details (multi_usrp, set_time_unknown_pps, rx_metadata_t)
- Added: Volk-optimized phase correction
- Removed: Pilot tone method (unnecessary with PPS-based drift)
- Removed: Temperature sensor dependency (B210 doesn't expose sensors)
- Removed: Complex adaptive resampler (phase correction alone is sufficient)
- Simplified: Overall architecture — fewer components, clearer data flow
