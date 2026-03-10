# USRP Bonding Implementation Plan - PPS-Only Synchronization
## Multi-Device Bandwidth Aggregation Using Only Pulse-Per-Second Signal

**Project Goal**: Develop a driver-level mechanism to bond multiple USRP B210 (or other USB-based) devices using ONLY external PPS synchronization, without requiring a shared 10 MHz reference clock.

---

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [PPS-Only Synchronization Rationale](#pps-only-rationale)
3. [Architecture Overview](#architecture-overview)
4. [Implementation Phases](#implementation-phases)
5. [PPS-Only Synchronization Strategy](#pps-only-synchronization)
6. [Frequency Drift Management](#frequency-drift-management)
7. [Testing & Validation](#testing-validation)
8. [Challenges & Mitigations](#challenges-mitigations)
9. [Comparison: PPS-Only vs. 10MHz+PPS](#comparison)

---

## 1. Executive Summary

### Objective
Create a bonding mechanism that requires ONLY a shared PPS signal (no 10 MHz reference), allowing each USRP to use its internal TCXO while maintaining time synchronization for bandwidth aggregation applications.

### Key Differences from 10MHz+PPS Approach
- **Simpler Hardware**: Only PPS distribution needed (no 10 MHz splitter)
- **Lower Cost**: Can use simple GPS receivers with PPS output
- **Independent Clocks**: Each device runs on internal oscillator
- **Drift Compensation**: Software must handle frequency drift between devices
- **Suitable For**: Applications tolerant to small frequency errors (~2-5 ppm)

### Key Features
- **PPS-Only Synchronization**: Time alignment using single pulse signal
- **Drift Tracking**: Software-based frequency offset estimation and correction
- **Adaptive Combining**: Frequency-domain alignment compensates for oscillator drift
- **Simplified Setup**: Minimal external hardware requirements
- **Trade-offs**: Slightly reduced coherence vs. 10MHz+PPS approach

### Target Use Cases
- **Wide-spectrum monitoring** where absolute frequency accuracy is less critical
- **Non-coherent multi-band reception** (different frequency ranges per device)
- **Budget-conscious deployments** without access to precision references
- **Mobile/portable setups** where GPS PPS is available but not 10 MHz
- **Research applications** studying drift compensation techniques

---

## 2. PPS-Only Synchronization Rationale

### 2.1 Why PPS-Only?

**Advantages:**
1. **Simplified Hardware Setup**
   - Only need PPS signal distribution (single coax)
   - No RF power splitter for 10 MHz required
   - Can use inexpensive GPS modules with PPS output
   - Reduces cable count and complexity

2. **Cost Reduction**
   - GPS receiver with PPS: $20-50 (vs. GPSDO: $300-1000)
   - No 10 MHz distribution amplifier needed
   - Fewer SMA cables and connectors
   - Total hardware cost reduction: ~$500-1000

3. **Flexibility**
   - Devices can operate at different locations (long cable runs)
   - PPS signal is digital (3.3V CMOS), less sensitive to cable quality
   - Easier to implement wireless PPS distribution if needed
   - Can use software-defined PPS on networked systems

**Disadvantages:**
1. **Frequency Drift Between Devices**
   - B210 TCXO accuracy: ±2 ppm (±2 kHz at 1 GHz)
   - Oscillators drift independently with temperature
   - Requires continuous frequency tracking and correction

2. **Phase Noise Accumulation**
   - No common frequency reference to discipline oscillators
   - Phase noise of each TCXO is uncorrelated
   - May impact certain applications (narrow-band, long integration)

3. **Periodic Recalibration**
   - Must re-estimate frequency offsets regularly
   - Cannot maintain coherence over very long periods
   - Unsuitable for applications requiring <0.1 ppm accuracy

### 2.2 Technical Feasibility

The B210's internal TCXO (40 MHz) has typical specifications:
- **Initial Accuracy**: ±2 ppm at 25°C
- **Temperature Stability**: ±2 ppm over 0-70°C
- **Aging**: ±1 ppm per year

For bandwidth aggregation:
- **Time Sync via PPS**: Achievable to <1 µs (sufficient for most apps)
- **Frequency Offset**: 2-4 ppm difference between devices is typical
- **Drift Rate**: ~0.1 ppm per minute due to temperature changes
- **Correction Needed**: Every 1-10 seconds depending on requirements

**Bottom Line**: PPS-only synchronization is viable for applications that can tolerate 2-5 ppm frequency uncertainty or can implement drift compensation.

---

## 3. Architecture Overview

### 3.1 System Block Diagram

```
                    +------------------+
                    |   GPS Receiver   |
                    |   (with PPS)     |
                    +--------+---------+
                             |
                             | PPS Signal
                             |
                    +--------v---------+
                    |  PPS Splitter/   |
                    |  Buffer (opt)    |
                    +--+-------+-------+
                       |       |
            PPS        |       |     PPS
              +--------+       +--------+
              |                         |
       +------v-------+          +------v-------+
       |   USRP B210  |          |   USRP B210  |
       |  (Device 0)  |          |  (Device 1)  |
       | Internal TCXO|          | Internal TCXO|
       +------+-------+          +------+-------+
              |                         |
          USB 3.0                   USB 3.0
              |                         |
              +------------+------------+
                           |
                    +------v-------+
                    |  Host PC     |
                    | Drift Tracker|
                    +--------------+
```

### 3.2 Key Architectural Differences

| Component | 10MHz+PPS Approach | PPS-Only Approach |
|-----------|-------------------|-------------------|
| **Clock Source** | External 10 MHz reference | Internal TCXO (independent per device) |
| **Time Sync** | PPS + locked oscillators | PPS only |
| **Frequency Coherence** | Hardware-locked (excellent) | Software-tracked (good) |
| **Drift Compensation** | Not needed | Required continuously |
| **Hardware Complexity** | High (2 reference signals) | Low (1 signal) |
| **Setup Cost** | $500-1500 | $50-200 |
| **Best For** | Coherent MIMO, precise freq | Wideband monitoring, non-coherent |

### 3.3 Software Components

New components for PPS-only operation:

1. **Drift Tracker**: Estimates frequency offset between devices
2. **Resampler**: Corrects for different effective sample rates
3. **Phase Rotator**: Compensates for accumulated phase drift
4. **Calibration Scheduler**: Triggers periodic re-calibration
5. **Temperature Monitor**: Tracks thermal drift if sensors available

---

## 4. Implementation Phases

### Phase 1: PPS-Only Discovery & Initialization (Weeks 1-2)

#### Tasks:
1. **Modify device discovery for PPS-only mode**
   ```cpp
   device_addr_t args;
   args["bonded"] = "true";
   args["sync_mode"] = "pps_only";  // NEW: Indicates PPS-only sync
   args["serial0"] = "ABC123";
   args["serial1"] = "DEF456";
   ```

2. **PPS verification without 10 MHz reference**
   - Configure devices to use internal TCXO
   - Verify PPS input is present on all devices
   - Check that devices can timestamp PPS edges

3. **Warning system for PPS-only limitations**
   - Inform user about frequency uncertainty
   - Recommend applications suitable for PPS-only
   - Provide option to upgrade to 10MHz+PPS if available

#### Deliverables:
- PPS-only device initialization code
- Detection of sync mode from device arguments
- User warnings about drift and limitations

---

### Phase 2: PPS-Based Time Synchronization (Weeks 3-4)

#### Tasks:
1. **Implement PPS-only synchronization**
   ```cpp
   class pps_only_sync_manager {
   public:
       void synchronize_time_only(
           const std::vector<multi_usrp::sptr>& devices
       ) {
           // Configure all devices for internal clock source
           for (auto& dev : devices) {
               dev->set_clock_source("internal");  // Use TCXO
               dev->set_time_source("external");    // Use PPS for time
           }
           
           // Wait for next PPS edge
           std::this_thread::sleep_for(std::chrono::milliseconds(1500));
           
           // Synchronize time at PPS edge
           const uhd::time_spec_t zero_time(0.0);
           for (auto& dev : devices) {
               dev->set_time_next_pps(zero_time);
           }
           
           // Wait for synchronization
           std::this_thread::sleep_for(std::chrono::milliseconds(2000));
           
           // Verify time alignment
           verify_time_sync_only(devices);
       }
       
   private:
       void verify_time_sync_only(
           const std::vector<multi_usrp::sptr>& devices
       );
   };
   ```

2. **Time synchronization verification**
   - Check that all devices report same time (within tolerance)
   - Acceptable time sync: <10 µs (PPS jitter + USB latency)
   - Log time differences for monitoring

3. **Handle PPS signal quality**
   - Detect missing PPS edges
   - Warn if PPS jitter is excessive
   - Implement timeout for PPS detection

#### Deliverables:
- PPS-only time synchronization implementation
- Time alignment verification tests
- PPS signal quality monitoring

---

### Phase 3: Frequency Drift Tracking (Weeks 5-8)

**This is the CRITICAL phase for PPS-only operation**

#### Tasks:
1. **Implement drift tracker**
   ```cpp
   class frequency_drift_tracker {
   public:
       struct drift_estimate {
           double frequency_offset_ppm;  // Estimated offset in ppm
           double phase_offset_radians;  // Accumulated phase
           uhd::time_spec_t last_update; // When estimate was updated
           double confidence;            // Estimation confidence 0-1
       };
       
       drift_estimate estimate_drift(
           size_t reference_device_idx,
           size_t target_device_idx,
           const std::vector<std::complex<float>*>& rx_buffers,
           size_t nsamps,
           double center_freq
       );
       
       void apply_drift_correction(
           std::complex<float>* buffer,
           size_t nsamps,
           const drift_estimate& drift,
           double elapsed_time
       );
       
   private:
       // Use pilot tone or known signal features
       double cross_correlate_for_drift(
           const std::complex<float>* ref_samples,
           const std::complex<float>* target_samples,
           size_t nsamps
       );
       
       // Track phase evolution over time
       double track_phase_drift(
           const std::complex<float>* samples,
           size_t nsamps,
           double last_phase
       );
   };
   ```

2. **Drift estimation methods**

   **Method A: Pilot Tone Injection**
   - Transmit known tone from external source
   - Measure frequency/phase difference between devices
   - Update: Every 5-10 seconds
   - Accuracy: ~0.1 ppm

   **Method B: Cross-Correlation**
   - When receiving same wideband signal on all devices
   - Use FFT to find frequency shift
   - Update: Continuously (every buffer)
   - Accuracy: ~0.5 ppm

   **Method C: Time-Stamp Based**
   - Compare device timestamps over time
   - Infer frequency drift from time drift rate
   - Update: Every second
   - Accuracy: ~1 ppm

   **Method D: GPS Timing (Enhanced)**
   - Use GPS receiver's time accuracy to discipline estimate
   - Compare device time against GPS time periodically
   - Update: Every 10 seconds
   - Accuracy: ~0.5 ppm

3. **Implement resampler for rate correction**
   ```cpp
   class adaptive_resampler {
   public:
       void resample_to_compensate_drift(
           const std::complex<float>* input,
           std::complex<float>* output,
           size_t input_length,
           double ppm_correction
       ) {
           // Calculate effective sample rate ratio
           double rate_ratio = 1.0 + (ppm_correction / 1e6);
           
           // Use polyphase filter bank for resampling
           // OR use FFT-based interpolation
           apply_fractional_delay(input, output, input_length, rate_ratio);
       }
       
   private:
       void apply_fractional_delay(
           const std::complex<float>* input,
           std::complex<float>* output,
           size_t length,
           double ratio
       );
       
       // Polyphase filter banks for efficient resampling
       std::vector<std::vector<float>> _filter_bank;
   };
   ```

4. **Phase rotation for accumulated drift**
   ```cpp
   void apply_phase_correction(
       std::complex<float>* samples,
       size_t nsamps,
       double phase_offset_radians,
       double phase_rate_rad_per_sample
   ) {
       for (size_t i = 0; i < nsamps; ++i) {
           double total_phase = phase_offset_radians + 
                               (i * phase_rate_rad_per_sample);
           std::complex<float> rotation(
               std::cos(total_phase),
               std::sin(total_phase)
           );
           samples[i] *= rotation;
       }
   }
   ```

#### Deliverables:
- Drift estimation algorithms (multiple methods)
- Real-time frequency tracking system
- Resampling and phase correction filters
- Drift monitoring and logging

---

### Phase 4: Adaptive Frequency Combining (Weeks 9-12)

#### Tasks:
1. **Modified RX streamer with drift compensation**
   ```cpp
   class pps_only_rx_streamer : public bonded_rx_streamer {
   public:
       size_t recv(
           const buffs_type& buffs,
           const size_t nsamps_per_buff,
           rx_metadata_t& metadata,
           const double timeout = 0.1,
           const bool one_packet = false
       ) override {
           // 1. Receive from all devices
           recv_from_all_devices();
           
           // 2. Estimate current drift (if needed)
           if (should_update_drift_estimate()) {
               update_drift_estimates();
           }
           
           // 3. Apply drift corrections before combining
           for (size_t dev = 0; dev < num_devices(); ++dev) {
               apply_drift_compensation(dev);
           }
           
           // 4. Combine frequency slices
           combine_frequency_slices_with_alignment(buffs, nsamps_per_buff);
           
           return nsamps_per_buff;
       }
       
   private:
       frequency_drift_tracker _drift_tracker;
       adaptive_resampler _resampler;
       std::vector<drift_estimate> _current_drifts;
       
       bool should_update_drift_estimate() {
           // Update every N samples or M seconds
           return (_samples_since_update > UPDATE_INTERVAL);
       }
       
       void apply_drift_compensation(size_t device_idx);
   };
   ```

2. **Frequency-domain alignment before combining**
   ```cpp
   void align_and_combine_with_drift(
       const std::vector<std::complex<float>*>& device_buffers,
       std::complex<float>* output_buffer,
       size_t nsamps,
       const std::vector<drift_estimate>& drifts
   ) {
       for (size_t dev = 0; dev < device_buffers.size(); ++dev) {
           // FFT to frequency domain
           fft_forward(device_buffers[dev], freq_buffer[dev], nsamps);
           
           // Apply frequency shift to compensate for drift
           // f_corrected = f * (1 + ppm/1e6)
           double freq_shift = drifts[dev].frequency_offset_ppm / 1e6;
           apply_frequency_shift(freq_buffer[dev], nsamps, freq_shift);
           
           // Apply phase correction
           apply_phase_rotation(freq_buffer[dev], nsamps, 
                               drifts[dev].phase_offset_radians);
           
           // Extract and place frequency slice
           extract_slice_to_output(freq_buffer[dev], combined_spectrum, dev);
       }
       
       // IFFT combined spectrum
       fft_inverse(combined_spectrum, output_buffer, nsamps);
   }
   ```

3. **Overlap region handling with drift**
   - In PPS-only mode, frequency slices may "drift" into each other
   - Implement adaptive guard bands based on current drift estimate
   - Use weighted combining in overlap regions

4. **Quality metrics and monitoring**
   ```cpp
   struct combining_quality_metrics {
       double estimated_snr_penalty_db;
       double max_frequency_error_hz;
       double phase_coherence_metric;
       bool drift_tracking_healthy;
       std::string warning_message;
   };
   ```

#### Deliverables:
- Drift-aware RX streamer implementation
- Frequency-domain drift compensation
- Combining quality monitoring
- Adaptive guard band management

---

### Phase 5: Calibration & Self-Correction (Weeks 13-15)

#### Tasks:
1. **Automatic drift calibration**
   ```cpp
   class drift_calibration_manager {
   public:
       void perform_calibration(
           bonded_usrp* usrp,
           calibration_method method
       ) {
           switch (method) {
               case PILOT_TONE:
                   calibrate_with_pilot_tone(usrp);
                   break;
               case KNOWN_SIGNAL:
                   calibrate_with_known_signal(usrp);
                   break;
               case CROSS_CORRELATION:
                   calibrate_with_cross_correlation(usrp);
                   break;
               case GPS_TIMESTAMP:
                   calibrate_with_gps_timing(usrp);
                   break;
           }
       }
       
   private:
       void calibrate_with_pilot_tone(bonded_usrp* usrp) {
           // User transmits known tone (e.g., 1 MHz CW)
           // All devices receive it
           // Measure frequency/phase differences
           // Store corrections
       }
       
       void calibrate_with_cross_correlation(bonded_usrp* usrp) {
           // Capture same wideband signal on all devices
           // Cross-correlate to find time/frequency offsets
           // Works with ambient RF environment
       }
       
       void calibrate_with_gps_timing(bonded_usrp* usrp) {
           // Use GPS receiver's precise timing
           // Compare device timestamps vs GPS time
           // Infer frequency offset from time drift rate
       }
   };
   ```

2. **Periodic recalibration scheduler**
   ```cpp
   class recalibration_scheduler {
   public:
       void start_background_calibration(
           bonded_usrp* usrp,
           double interval_seconds = 10.0
       ) {
           _calibration_thread = std::thread([this, usrp, interval_seconds]() {
               while (!_should_stop) {
                   std::this_thread::sleep_for(
                       std::chrono::milliseconds(
                           static_cast<long>(interval_seconds * 1000)
                       )
                   );
                   
                   // Perform quick calibration without interrupting streaming
                   perform_inline_calibration(usrp);
               }
           });
       }
       
   private:
       std::thread _calibration_thread;
       std::atomic<bool> _should_stop{false};
       
       void perform_inline_calibration(bonded_usrp* usrp);
   };
   ```

3. **Thermal drift modeling**
   ```cpp
   class thermal_drift_predictor {
   public:
       double predict_drift_rate(
           double current_temp_celsius,
           double time_since_last_cal_seconds
       ) {
           // Model: drift_ppm = k * (T - T_ref) + aging_term
           // Based on TCXO datasheet characteristics
           double temp_coefficient = 0.1;  // ppm per degree C
           double aging_rate = 0.0001;     // ppm per second
           
           double temp_drift = temp_coefficient * 
                              (current_temp_celsius - _reference_temp);
           double aging_drift = aging_rate * time_since_last_cal_seconds;
           
           return temp_drift + aging_drift;
       }
       
   private:
       double _reference_temp = 25.0;
   };
   ```

4. **Calibration utilities**
   ```bash
   # Command-line calibration tool
   uhd_bonded_calibrate_pps_only \
       --args="bonded=true,sync_mode=pps_only,serial0=ABC,serial1=DEF" \
       --method=pilot_tone \
       --tone_freq=2.4e9 \
       --output=pps_drift_cal.dat
   ```

#### Deliverables:
- Automated calibration routines
- Background recalibration system
- Thermal drift prediction
- Calibration tools and utilities

---

### Phase 6: Testing & Validation (Weeks 16-18)

#### PPS-Only Specific Tests:

1. **Drift Tracking Accuracy Test**
   ```
   Setup: 2x B210, PPS only
   Test: Place one device in temperature chamber
   Vary temperature: 20-40°C over 1 hour
   Measure: Drift tracking accuracy vs. true drift
   Success: Estimate within 0.5 ppm of actual drift
   ```

2. **Long-Duration Stability Test**
   ```
   Setup: 2x B210, PPS only
   Duration: 8 hours continuous operation
   Monitor: Frequency offset, combining quality
   Success: No degradation >3 dB over time
   ```

3. **Ambient Signal Test (No Pilot Tone)**
   ```
   Setup: 2x B210, PPS only
   Receive: WiFi, cellular, or broadcast signals
   Test: Cross-correlation based drift estimation
   Success: Automatic drift compensation works
   ```

4. **Comparison Against 10MHz+PPS**
   ```
   Setup: Same 2x B210, test both modes
   Measure: SNR, phase noise, spectrum quality
   Expected: PPS-only performs 1-3 dB worse
   Acceptable if: Meets application requirements
   ```

#### Deliverables:
- PPS-only specific test suite
- Drift compensation validation
- Performance comparison report
- Long-term stability data

---

## 5. PPS-Only Synchronization Strategy

### 5.1 Hardware Setup

**Minimum Hardware Requirements:**

```
Shopping List (PPS-Only):
- 2x USRP B210 ............................ $1300 each
- 1x GPS Module with PPS output ........... $30-50
  (e.g., u-blox NEO-M8N, Adafruit Ultimate GPS)
- 1x Active GPS antenna ................... $20-40
- 2x SMA cables for PPS ................... $10 each
- Optional: PPS signal buffer/splitter .... $30-50
- 2x USB 3.0 cables ....................... $10 each

Total: ~$2800 (vs. ~$3300-4000 with 10MHz+PPS)
Savings: $500-1200
```

**GPS Module Selection:**
- Must have PPS output (1 pulse per second)
- 3.3V CMOS or TTL compatible with B210
- Typical PPS accuracy: 10-100 ns (more than adequate)
- Examples:
  - u-blox NEO-M8N: $30, 10 ns accuracy
  - Adafruit Ultimate GPS: $40, 1 µs accuracy
  - GT-U7 GPS Module: $15, 1 µs accuracy

**PPS Signal Distribution:**
- For 2 devices: Direct connection or simple Y-cable
- For 3+ devices: Use CMOS buffer (74HC14 or similar)
- Cable length: <10 meters recommended (for low jitter)
- Use shielded cable if near RF sources

### 5.2 Connection Diagram

```
                  +----------------+
                  |  GPS Receiver  |
                  |  (NEO-M8N)     |
                  |                |
                  |  [PPS Output]  |
                  +-------+--------+
                          |
                          | Single SMA cable
                          |
              +-----------v-----------+
              |  Optional: PPS Buffer |
              |  (74HC14 hex inverter |
              |   configured as buffer|
              +-----------+-----------+
                          |
          +---------------+---------------+
          |                               |
    PPS +-v-+                       PPS +-v-+
        |   |                           |   |
   +----+---+----+                 +----+---+----+
   | B210 Dev 0  |                 | B210 Dev 1  |
   | (Internal   |                 | (Internal   |
   |  TCXO: 40MHz|                 |  TCXO: 40MHz|
   +------+------+                 +------+------+
          |                               |
      USB 3.0                         USB 3.0
          |                               |
          +---------------+---------------+
                          |
                   +------v-------+
                   |   Host PC    |
                   | Drift Tracker|
                   +--------------+
```

### 5.3 PPS Signal Quality Requirements

| Parameter | Requirement | Typical GPS Module |
|-----------|-------------|-------------------|
| **Voltage Level** | 3.3V CMOS or TTL | 3.3V CMOS |
| **Pulse Width** | 100 µs - 200 ms | 100 ms typical |
| **Rise Time** | <100 ns | 10-50 ns |
| **Jitter** | <1 µs | 10-100 ns |
| **Rate** | 1 Hz (1 PPS) | 1 Hz fixed |
| **Duty Cycle** | Not critical | 10% typical |

### 5.4 Synchronization Procedure (PPS-Only)

```cpp
void synchronize_pps_only(bonded_usrp* usrp) {
    std::cout << "=== PPS-ONLY SYNCHRONIZATION ===" << std::endl;
    std::cout << "NOTE: Devices will use internal oscillators" << std::endl;
    std::cout << "Expected frequency uncertainty: 2-5 ppm" << std::endl << std::endl;
    
    // Step 1: Configure for internal clocks
    for (size_t i = 0; i < usrp->get_num_bonded_devices(); ++i) {
        auto dev = usrp->get_bonded_device(i);
        
        std::cout << "Device " << i << ": Configuring for internal TCXO..." 
                  << std::endl;
        dev->set_clock_source("internal", 0);  // Use internal oscillator
        dev->set_time_source("external", 0);    // Use external PPS
    }
    
    // Step 2: Wait for PPS signal detection
    std::cout << "Waiting for PPS signal detection..." << std::endl;
    bool pps_detected = wait_for_pps_signal(usrp, 5.0);  // 5 second timeout
    
    if (!pps_detected) {
        throw uhd::runtime_error(
            "PPS signal not detected on one or more devices. "
            "Check GPS receiver and cable connections."
        );
    }
    std::cout << "PPS signal detected on all devices!" << std::endl;
    
    // Step 3: Synchronize time at next PPS edge
    std::cout << "Synchronizing device times at next PPS edge..." << std::endl;
    
    // Wait for a clean PPS edge (avoid being too close to edge)
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    
    const uhd::time_spec_t start_time(0.0);
    for (size_t i = 0; i < usrp->get_num_bonded_devices(); ++i) {
        usrp->get_bonded_device(i)->set_time_next_pps(start_time);
    }
    
    // Wait for PPS edge and synchronization to complete
    std::cout << "Waiting for PPS edge..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Step 4: Verify time synchronization
    std::cout << "Verifying time synchronization..." << std::endl;
    verify_pps_time_sync(usrp);
    
    // Step 5: Perform initial drift calibration
    std::cout << "Performing initial drift estimation..." << std::endl;
    perform_initial_drift_calibration(usrp);
    
    // Step 6: Start background drift tracking
    std::cout << "Starting continuous drift tracking..." << std::endl;
    usrp->start_drift_tracking(10.0);  // Re-calibrate every 10 seconds
    
    std::cout << "=== SYNCHRONIZATION COMPLETE ===" << std::endl;
    std::cout << "System ready for bonded operation" << std::endl;
    std::cout << "Drift compensation active" << std::endl << std::endl;
}

bool wait_for_pps_signal(bonded_usrp* usrp, double timeout_seconds) {
    auto start = std::chrono::steady_clock::now();
    
    for (size_t i = 0; i < usrp->get_num_bonded_devices(); ++i) {
        auto dev = usrp->get_bonded_device(i);
        uhd::time_spec_t initial_time = dev->get_time_now();
        
        // Wait for time to change (indicates PPS received)
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            uhd::time_spec_t current_time = dev->get_time_now();
            
            // Check if time has advanced (PPS received)
            if (current_time.get_full_secs() > initial_time.get_full_secs()) {
                std::cout << "Device " << i << ": PPS detected" << std::endl;
                break;
            }
            
            // Check timeout
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration<double>(elapsed).count() > timeout_seconds) {
                std::cerr << "Device " << i << ": PPS timeout!" << std::endl;
                return false;
            }
        }
    }
    
    return true;
}

void verify_pps_time_sync(bonded_usrp* usrp) {
    double max_time_diff = 0.0;
    uhd::time_spec_t ref_time;
    
    for (size_t i = 0; i < usrp->get_num_bonded_devices(); ++i) {
        auto current_time = usrp->get_bonded_device(i)->get_time_now();
        
        if (i == 0) {
            ref_time = current_time;
        } else {
            double diff = std::abs(
                current_time.get_real_secs() - ref_time.get_real_secs()
            );
            max_time_diff = std::max(max_time_diff, diff);
        }
        
        std::cout << "Device " << i << " time: " 
                  << current_time.get_real_secs() << " seconds" << std::endl;
    }
    
    // PPS-only sync is typically less precise than 10MHz+PPS
    // Allow up to 10 microseconds difference
    const double MAX_ALLOWED_DIFF = 10e-6;
    
    if (max_time_diff > MAX_ALLOWED_DIFF) {
        std::cerr << "WARNING: Time sync looser than expected: " 
                  << max_time_diff * 1e6 << " us" << std::endl;
        std::cerr << "This is acceptable for PPS-only mode" << std::endl;
    } else {
        std::cout << "Time sync excellent: " 
                  << max_time_diff * 1e9 << " ns" << std::endl;
    }
}
```

---

## 6. Frequency Drift Management

### 6.1 Drift Estimation Techniques

#### Technique 1: Pilot Tone Method (Highest Accuracy)

```cpp
double estimate_drift_pilot_tone(
    const std::complex<float>* ref_samples,
    const std::complex<float>* test_samples,
    size_t nsamps,
    double sample_rate,
    double expected_tone_freq
) {
    // 1. Find tone in reference device
    double ref_tone_freq = find_tone_frequency(
        ref_samples, nsamps, sample_rate, expected_tone_freq
    );
    
    // 2. Find tone in test device
    double test_tone_freq = find_tone_frequency(
        test_samples, nsamps, sample_rate, expected_tone_freq
    );
    
    // 3. Calculate drift in ppm
    // drift_ppm = (f_test - f_ref) / f_ref * 1e6
    double freq_diff = test_tone_freq - ref_tone_freq;
    double drift_ppm = (freq_diff / ref_tone_freq) * 1e6;
    
    return drift_ppm;
}

double find_tone_frequency(
    const std::complex<float>* samples,
    size_t nsamps,
    double sample_rate,
    double expected_freq
) {
    // Perform FFT
    std::vector<std::complex<float>> fft_out(nsamps);
    fft_forward(samples, fft_out.data(), nsamps);
    
    // Find peak near expected frequency
    size_t expected_bin = (expected_freq / sample_rate) * nsamps;
    size_t search_range = 100;  // bins
    
    size_t peak_bin = expected_bin;
    float max_mag = 0.0;
    
    for (size_t bin = expected_bin - search_range; 
         bin <= expected_bin + search_range; ++bin) {
        float mag = std::abs(fft_out[bin]);
        if (mag > max_mag) {
            max_mag = mag;
            peak_bin = bin;
        }
    }
    
    // Refine using parabolic interpolation
    double refined_bin = parabolic_interpolation(
        std::abs(fft_out[peak_bin - 1]),
        std::abs(fft_out[peak_bin]),
        std::abs(fft_out[peak_bin + 1]),
        peak_bin
    );
    
    // Convert bin to frequency
    double tone_freq = (refined_bin / nsamps) * sample_rate;
    return tone_freq;
}
```

**Usage**: User transmits CW tone, all devices receive it
**Accuracy**: ±0.01 ppm
**Update Rate**: Every calibration cycle (5-10 seconds)
**Pros**: Very accurate, works even with large initial drift
**Cons**: Requires external signal generator

#### Technique 2: Cross-Correlation Method (No Pilot Needed)

```cpp
double estimate_drift_cross_correlation(
    const std::complex<float>* ref_samples,
    const std::complex<float>* test_samples,
    size_t nsamps,
    double sample_rate
) {
    // 1. Compute FFTs
    std::vector<std::complex<float>> ref_fft(nsamps);
    std::vector<std::complex<float>> test_fft(nsamps);
    
    fft_forward(ref_samples, ref_fft.data(), nsamps);
    fft_forward(test_samples, test_fft.data(), nsamps);
    
    // 2. Cross-power spectrum
    std::vector<std::complex<float>> cross_spectrum(nsamps);
    for (size_t i = 0; i < nsamps; ++i) {
        cross_spectrum[i] = ref_fft[i] * std::conj(test_fft[i]);
        // Normalize
        float mag = std::abs(cross_spectrum[i]);
        if (mag > 1e-10) {
            cross_spectrum[i] /= mag;
        }
    }
    
    // 3. IFFT to get cross-correlation
    std::vector<std::complex<float>> xcorr(nsamps);
    fft_inverse(cross_spectrum.data(), xcorr.data(), nsamps);
    
    // 4. Find peak in cross-correlation
    size_t peak_idx = 0;
    float max_val = 0.0;
    for (size_t i = 0; i < nsamps; ++i) {
        float val = std::abs(xcorr[i]);
        if (val > max_val) {
            max_val = val;
            peak_idx = i;
        }
    }
    
    // 5. Convert lag to frequency offset
    // Frequency offset causes circular shift in frequency domain
    // which manifests as phase slope in cross-correlation
    
    // For more accurate method, use phase difference across frequency bins
    double phase_slope = compute_phase_slope(ref_fft, test_fft, nsamps);
    
    // Phase slope relates to frequency offset:
    // d(phase)/d(freq) = 2*pi * tau
    // where tau is the time offset
    // Frequency drift causes changing tau
    
    double drift_ppm = (phase_slope / (2.0 * M_PI)) * 1e6;
    
    return drift_ppm;
}

double compute_phase_slope(
    const std::vector<std::complex<float>>& fft1,
    const std::vector<std::complex<float>>& fft2,
    size_t nsamps
) {
    // Compute phase difference for each frequency bin
    std::vector<double> phase_diffs;
    
    for (size_t i = 1; i < nsamps/2; ++i) {  // Skip DC and negative freqs
        if (std::abs(fft1[i]) > 1e-5 && std::abs(fft2[i]) > 1e-5) {
            std::complex<float> ratio = fft2[i] / fft1[i];
            double phase = std::arg(ratio);
            phase_diffs.push_back(phase);
        }
    }
    
    // Fit linear trend to phase differences
    // Slope of this line indicates frequency offset
    return fit_linear_trend(phase_diffs);
}
```

**Usage**: Works with any wideband signal (WiFi, cellular, etc.)
**Accuracy**: ±0.1-0.5 ppm
**Update Rate**: Every buffer (continuous)
**Pros**: No external equipment needed
**Cons**: Requires sufficient signal energy, less accurate

#### Technique 3: GPS Timestamp Method

```cpp
double estimate_drift_gps_timestamp(
    multi_usrp::sptr device1,
    multi_usrp::sptr device2,
    double measurement_duration_seconds = 60.0
) {
    // 1. Record initial device times
    auto t1_start = device1->get_time_now();
    auto t2_start = device2->get_time_now();
    
    // 2. Wait for measurement duration
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            static_cast<long>(measurement_duration_seconds * 1000)
        )
    );
    
    // 3. Record final device times
    auto t1_end = device1->get_time_now();
    auto t2_end = device2->get_time_now();
    
    // 4. Calculate elapsed time for each device
    double elapsed1 = t1_end.get_real_secs() - t1_start.get_real_secs();
    double elapsed2 = t2_end.get_real_secs() - t2_start.get_real_secs();
    
    // 5. Drift is the difference in elapsed times
    // If device 2 clock runs fast, elapsed2 > elapsed1
    double time_diff = elapsed2 - elapsed1;
    double drift_ppm = (time_diff / measurement_duration_seconds) * 1e6;
    
    return drift_ppm;
}
```

**Usage**: No RF signals needed, pure timing measurement
**Accuracy**: ±0.5-1 ppm (depends on measurement duration)
**Update Rate**: Every 30-60 seconds
**Pros**: Simple, reliable, no RF required
**Cons**: Slow update rate, moderate accuracy

### 6.2 Drift Compensation Application

Once drift is estimated, apply corrections:

```cpp
void apply_comprehensive_drift_correction(
    std::complex<float>* samples,
    size_t nsamps,
    double sample_rate,
    const drift_estimate& drift,
    double time_since_cal_seconds
) {
    // 1. Calculate current frequency offset
    double freq_offset_hz = (drift.frequency_offset_ppm / 1e6) * 
                            sample_rate;  // Convert ppm to Hz
    
    // 2. Calculate accumulated phase
    double phase_accumulated = drift.phase_offset_radians + 
                              (2.0 * M_PI * freq_offset_hz * 
                               time_since_cal_seconds);
    
    // 3. Calculate phase rate (radians per sample)
    double phase_rate = 2.0 * M_PI * freq_offset_hz / sample_rate;
    
    // 4. Apply correction
    for (size_t i = 0; i < nsamps; ++i) {
        double phase = phase_accumulated + (i * phase_rate);
        
        // Complex exponential for phase rotation
        std::complex<float> correction(
            std::cos(-phase),  // Negative to correct
            std::sin(-phase)
        );
        
        samples[i] *= correction;
    }
}
```

**Optimization with Volk:**

```cpp
void apply_drift_correction_optimized(
    std::complex<float>* samples,
    size_t nsamps,
    double initial_phase,
    double phase_rate
) {
    // Generate phase ramp
    std::vector<float> phase_ramp(nsamps);
    for (size_t i = 0; i < nsamps; ++i) {
        phase_ramp[i] = initial_phase + (i * phase_rate);
    }
    
    // Use Volk for fast complex rotation
    // volk_32fc_s32fc_rotator_32fc(output, input, phase_inc, phase, nsamps)
    std::complex<float> phase_inc = std::exp(
        std::complex<float>(0, -phase_rate)
    );
    std::complex<float> phase_current = std::exp(
        std::complex<float>(0, -initial_phase)
    );
    
    volk_32fc_s32fc_rotator_32fc(
        samples,           // output (in-place)
        samples,           // input
        phase_inc,         // phase increment per sample
        &phase_current,    // initial phase
        nsamps
    );
}
```

### 6.3 Adaptive Update Strategy

```cpp
class adaptive_drift_manager {
public:
    void update_drift_estimate(
        const drift_estimate& new_estimate,
        double confidence
    ) {
        // Weighted average with previous estimate
        double alpha = calculate_smoothing_factor(confidence);
        
        _current_estimate.frequency_offset_ppm = 
            alpha * new_estimate.frequency_offset_ppm +
            (1.0 - alpha) * _current_estimate.frequency_offset_ppm;
        
        _current_estimate.phase_offset_radians = 
            normalize_phase(
                alpha * new_estimate.phase_offset_radians +
                (1.0 - alpha) * _current_estimate.phase_offset_radians
            );
        
        _current_estimate.last_update = uhd::time_spec_t::get_system_time();
        _current_estimate.confidence = confidence;
        
        // Adjust update rate based on drift stability
        adjust_update_interval();
    }
    
private:
    drift_estimate _current_estimate;
    std::vector<drift_estimate> _history;
    double _update_interval = 10.0;  // seconds
    
    double calculate_smoothing_factor(double confidence) {
        // Higher confidence = more weight to new estimate
        // Typical range: 0.1 (low confidence) to 0.9 (high confidence)
        return 0.5 + (confidence * 0.4);
    }
    
    void adjust_update_interval() {
        // If drift is stable, update less frequently
        if (_history.size() < 5) return;
        
        // Calculate variance of recent estimates
        double variance = calculate_variance(_history);
        
        if (variance < 0.1) {
            // Drift is stable, update less often
            _update_interval = std::min(30.0, _update_interval * 1.2);
        } else {
            // Drift is changing, update more often
            _update_interval = std::max(5.0, _update_interval * 0.8);
        }
    }
};
```

---

## 7. Testing & Validation

### 7.1 PPS-Only Specific Test Cases

#### Test 1: Drift Compensation Accuracy

```cpp
TEST(PPSOnly, DriftCompensationAccuracy) {
    // Setup
    auto usrp = create_pps_only_bonded_usrp();
    
    // Inject known drift (simulate in software)
    double injected_drift_ppm = 2.5;
    inject_artificial_drift(usrp, 1, injected_drift_ppm);
    
    // Estimate drift using pilot tone
    double estimated_drift = usrp->estimate_drift_pilot_tone(0, 1);
    
    // Verify accuracy
    EXPECT_NEAR(estimated_drift, injected_drift_ppm, 0.1);  // Within 0.1 ppm
}
```

#### Test 2: Temperature-Induced Drift

```
Manual Test Procedure:
1. Set up 2x B210 with PPS-only sync
2. Place one device in temperature chamber (or freezer)
3. Record temperature: Start at 25°C, cool to 5°C over 30 minutes
4. Monitor drift estimate every 10 seconds
5. Verify drift compensation maintains signal quality

Expected Results:
- Drift will increase with temperature change (2-4 ppm)
- Drift tracker should follow temperature-induced drift
- Signal quality degrades <2 dB despite temperature change
```

#### Test 3: Long-Term Stability

```python
# Python test script
import uhd
import time
import numpy as np

def test_long_term_pps_only_stability():
    # Create bonded USRP
    usrp = uhd.usrp.multi_usrp.make(
        "bonded=true,sync_mode=pps_only,serial0=...,serial1=..."
    )
    
    # Configure
    usrp.set_rx_rate(10e6)
    usrp.set_rx_freq(2.4e9)
    
    # Stream for 24 hours, measure quality every hour
    quality_log = []
    for hour in range(24):
        # Stream for 1 minute
        samples = receive_samples(usrp, nsamps=600000)  # 1 min @ 10 MS/s
        
        # Measure combining quality
        quality = measure_spectrum_quality(samples)
        quality_log.append(quality)
        
        print(f"Hour {hour}: Quality = {quality:.2f} dB")
        
        # Wait until next hour
        time.sleep(3600 - 60)  # 59 minutes
    
    # Verify quality doesn't degrade more than 3 dB over 24 hours
    max_degradation = max(quality_log) - min(quality_log)
    assert max_degradation < 3.0, f"Quality degraded {max_degradation:.2f} dB"
```

#### Test 4: Multi-Band Reception (Non-Coherent)

```
Test Scenario: Simultaneous Reception of Different Bands
Device 1: 900 MHz (GSM)
Device 2: 2.4 GHz (WiFi)

This is where PPS-only shines - no need for frequency coherence!

Expected: Works perfectly since bands are independent
```

### 7.2 Performance Expectations

| Metric | 10MHz+PPS | PPS-Only | Notes |
|--------|-----------|----------|-------|
| **Time Sync** | <0.1 µs | <10 µs | PPS jitter dominated |
| **Freq Accuracy** | <0.01 ppm | 2-5 ppm | TCXO uncertainty |
| **Phase Noise** | -130 dBc/Hz | -120 dBc/Hz | Uncorrelated oscillators |
| **SNR Penalty** | 0 dB | 1-3 dB | Drift compensation imperfect |
| **Coherence Time** | Hours | Seconds-Minutes | Requires recalibration |
| **Setup Cost** | $3500-4000 | $2800 | Significant savings |
| **Setup Time** | 20 minutes | 10 minutes | Simpler connections |

### 7.3 Recommended Test Equipment

For PPS-only validation:

1. **Spectrum Analyzer** (e.g., RTL-SDR, HackRF, or pro unit)
   - Verify combined spectrum quality
   - Measure artifacts at slice boundaries
   - Cost: $30-5000 depending on quality

2. **Signal Generator with CW Mode** (for pilot tone method)
   - Generate stable test tone
   - Examples: HackRF ($300), proper SigGen ($1000+)

3. **GPS Receiver with PPS Output**
   - Primary synchronization source
   - u-blox NEO-M8N recommended ($30-40)

4. **Oscilloscope** (optional, for PPS signal verification)
   - Verify PPS timing and jitter
   - Any scope with >10 MHz BW adequate

5. **Temperature Chamber** (optional, for drift testing)
   - Or use freezer/refrigerator
   - Verify thermal drift compensation

---

## 8. Challenges & Mitigations (PPS-Only Specific)

### 8.1 Challenge: Frequency Drift Management

**Problem**: Each device's TCXO drifts independently at 2-5 ppm, causing frequency and phase misalignment between devices.

**Impact**: 
- At 2.4 GHz, 2 ppm = 4.8 kHz frequency error
- Frequency slices "shift" relative to each other
- Phase accumulates over time, causing destructive interference

**Mitigations**:
1. **Continuous Drift Tracking** (PRIMARY)
   - Estimate drift every 5-10 seconds
   - Apply real-time corrections in frequency domain
   - Implementation: Phase-locked loop (PLL) tracking filter

2. **Pilot Tone Injection** (BEST ACCURACY)
   - User transmits known CW tone
   - All devices measure tone frequency
   - Calculate relative drift precisely (±0.01 ppm)

3. **Cross-Correlation** (NO EXTERNAL SIGNAL)
   - Use ambient RF signals
   - Cross-correlate between devices
   - Works with WiFi, cellular, broadcast signals

4. **Adaptive Guard Bands**
   - Increase guard band width when drift is high
   - Reduces usable bandwidth but maintains quality
   - Typical: 5-10% bandwidth sacrifice

5. **Thermal Management**
   - Keep devices at similar temperatures
   - Use fans or heat sinks
   - Monitor temperature if sensors available

### 8.2 Challenge: Phase Accumulation

**Problem**: Even with frequency correction, phase error accumulates over time due to imperfect drift estimation.

**Impact**:
- Phase coherence degrades over seconds to minutes
- Unsuitable for long-integration narrowband applications
- Affects applications requiring <10° phase accuracy

**Mitigations**:
1. **Periodic Phase Reset**
   - Re-synchronize phase every 10-30 seconds
   - Brief interruption (<1 second) in streaming
   - Acceptable for most monitoring applications

2. **Phase Tracking Loop**
   - Continuously measure and correct phase
   - Use reference signal (pilot tone or ambient)
   - Smoothly adjust without interruption

3. **Accept Limitations**
   - Document that PPS-only is not suitable for:
     - Phase-sensitive MIMO (use 10MHz+PPS instead)
     - Long-integration spectroscopy
     - Sub-degree phase accuracy requirements
   - Perfect for:
     - Wideband monitoring
     - Non-coherent multi-band
     - Energy detection

### 8.3 Challenge: Initial Acquisition Time

**Problem**: First drift estimate takes time to converge, delaying start of bonded operation.

**Impact**:
- Must wait 10-60 seconds before reliable bonding
- User experience not immediate

**Mitigations**:
1. **Fast Acquisition Mode**
   - Use pilot tone for initial cal (fastest, <5 seconds)
   - Switch to cross-correlation for ongoing tracking

2. **Stored Calibration**
   - Save drift estimates from previous sessions
   - Use as initial guess (speeds convergence)
   - Re-calibrate to account for temperature changes

3. **Progressive Quality**
   - Start operation with coarse drift estimate
   - Quality improves as estimate refines
   - User sees gradual improvement vs. sudden start

### 8.4 Challenge: Unsuitable for Some Applications

**Problem**: PPS-only cannot match 10MHz+PPS performance for coherent MIMO or high-precision applications.

**Impact**:
- Some users may be disappointed
- Research applications requiring phase-locked devices won't work

**Mitigations**:
1. **Clear Documentation**
   - Explicitly state PPS-only limitations
   - Provide decision tree: When to use PPS-only vs. 10MHz+PPS
   - Set realistic expectations

2. **Hybrid Mode Support**
   - Implement both PPS-only and 10MHz+PPS modes
   - Allow user to upgrade from PPS-only to 10MHz+PPS
   - Detection: If 10 MHz reference present, use it automatically

3. **Application-Specific Guidance**
   ```
   Recommended for PPS-Only:
   ✓ Wideband spectrum monitoring (>50 MHz)
   ✓ Energy detection
   ✓ Non-coherent multi-band reception
   ✓ General SDR applications
   ✓ Prototyping and development
   
   NOT Recommended for PPS-Only:
   ✗ Phase-synchronized MIMO beamforming
   ✗ Long-integration spectroscopy (<1 Hz RBW)
   ✗ Precision frequency measurement (<0.1 ppm)
   ✗ Phase-coherent radar
   ✗ Communications requiring phase sync (some)
   ```

---

## 9. Comparison: PPS-Only vs. 10MHz+PPS

### 9.1 Side-by-Side Comparison

| Aspect | PPS-Only | 10MHz + PPS | Winner |
|--------|----------|-------------|--------|
| **Hardware Cost** | $50-100 | $500-1500 | PPS-Only |
| **Setup Complexity** | Low (1 cable) | Medium (2 cables + splitter) | PPS-Only |
| **Time Sync Accuracy** | 1-10 µs | 0.01-0.1 µs | 10MHz+PPS |
| **Frequency Accuracy** | 2-5 ppm | <0.01 ppm | 10MHz+PPS |
| **Phase Coherence** | Poor (seconds) | Excellent (hours) | 10MHz+PPS |
| **SNR Performance** | -1 to -3 dB | 0 dB (reference) | 10MHz+PPS |
| **Drift Compensation** | Required (complex) | Not needed | 10MHz+PPS |
| **Suitable Apps** | Wideband, non-coherent | All applications | 10MHz+PPS |
| **Software Complexity** | High (tracking needed) | Low | 10MHz+PPS |
| **Portability** | Excellent (GPS anywhere) | Limited (need ref source) | PPS-Only |
| **Power Consumption** | Lower (no ref amp) | Higher | PPS-Only |

### 9.2 Decision Tree

```
Choose PPS-Only If:
  - Budget constrained (<$100 for sync hardware)
  - Application tolerates 2-5 ppm frequency uncertainty
  - Wideband monitoring (>50 MHz) is primary goal
  - Phase coherence not critical (>10° acceptable)
  - Portable/mobile deployment (GPS available)
  - Non-coherent multi-band reception
  
Choose 10MHz+PPS If:
  - Highest performance required
  - Phase-synchronized MIMO needed
  - Frequency accuracy critical (<0.1 ppm)
  - Long-term coherence required (>1 minute)
  - Communication systems with strict phase requirements
  - Research into coherent multi-device techniques
  
Choose Hybrid (Support Both):
  - Provide flexibility to users
  - Allow upgrade path from PPS-only to full performance
  - Research comparing both approaches (!!!)
```

### 9.3 Performance Comparison (Simulated)

Based on expected performance:

```
Test: 100 MHz Bandwidth Aggregation (2x B210)
Center Frequency: 2.4 GHz
Duration: 10 minutes

Metric                     | 10MHz+PPS  | PPS-Only   | Δ
---------------------------|------------|------------|-------
Time sync (µs)             | 0.05       | 5.0        | 100x worse
Freq sync (ppm)            | 0.005      | 2.5        | 500x worse
Combined spectrum SNR (dB) | -0.5       | -2.5       | 2 dB worse
Artifacts at boundaries    | -45 dBc    | -38 dBc    | 7 dB worse
Phase coherence duration   | >10 min    | ~30 sec    | 20x worse
CPU usage (%)              | 120        | 180        | 50% more
Setup cost ($)             | 1200       | 80         | 15x cheaper
Setup time (min)           | 20         | 10         | 2x faster

Conclusion: PPS-only trades performance for cost/simplicity
```

---

## 10. Implementation Recommendations

### 10.1 Phased Development Strategy

**Phase 1: Implement PPS-Only First** (Weeks 1-8)
- Simpler hardware setup
- Faster to test and validate
- Proves basic bonding architecture
- Identifies software challenges early

**Phase 2: Add Drift Compensation** (Weeks 9-15)
- Core innovation of PPS-only approach
- Publishable research contribution
- Validates frequency tracking algorithms

**Phase 3: Implement 10MHz+PPS** (Weeks 16-20)
- Add hardware-locked synchronization
- Compare performance against PPS-only
- Provide best of both worlds

**Phase 4: Hybrid Mode** (Weeks 21-22)
- Auto-detect available sync signals
- Seamlessly switch between modes
- User-friendly configuration

### 10.2 Code Organization

```
lib/usrp/bonded/
├── bonded_usrp.cpp              # Main virtual device
├── sync_manager.cpp             # Synchronization (both modes)
├── pps_only_sync.cpp            # PPS-only specific
├── clock_locked_sync.cpp        # 10MHz+PPS specific
├── drift_tracker.cpp            # Frequency drift estimation
├── drift_compensator.cpp        # Apply corrections
├── frequency_combiner.cpp       # FFT-based combining
└── calibration_manager.cpp      # Calibration routines

include/uhd/usrp/bonded/
├── bonded_usrp.hpp
├── sync_manager.hpp
├── drift_tracker.hpp
└── types.hpp                    # Common types/structs

examples/
├── bonded_rx_samples_pps_only.cpp
├── bonded_rx_samples_10mhz.cpp
├── drift_calibration.cpp
└── bonded_benchmark.cpp
```

### 10.3 User Interface Design

```cpp
// Simple API for users
device_addr_t args;

// Option 1: PPS-Only (auto-detected if no 10 MHz present)
args["bonded"] = "true";
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";
// Will use PPS-only if no 10 MHz detected

// Option 2: Explicit PPS-Only
args["bonded"] = "true";
args["sync_mode"] = "pps_only";
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";

// Option 3: Require 10MHz+PPS (fail if not available)
args["bonded"] = "true";
args["sync_mode"] = "clock_locked";  // Requires 10 MHz
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";

auto usrp = multi_usrp::make(args);
```

### 10.4 Documentation Strategy

Create three levels of documentation:

1. **Quick Start Guide** (PPS-Only)
   - "Get bonded USRP running in 10 minutes"
   - Minimum hardware: 2x B210 + GPS module ($50)
   - Step-by-step setup with photos

2. **User Manual**
   - Comparison of sync modes
   - When to use PPS-only vs. 10MHz+PPS
   - Troubleshooting guide
   - Performance expectations

3. **Research Paper**
   - Novel drift compensation algorithms
   - Performance comparison
   - Applications and limitations
   - Contribution to SDR community

---

## 11. Research Contributions

### 11.1 Novel Aspects of PPS-Only Bonding

1. **Drift Compensation Algorithms**
   - Real-time frequency tracking without common reference
   - Comparison of estimation methods (pilot, cross-corr, GPS)
   - Adaptive update strategies

2. **Cost-Performance Trade-offs**
   - Quantify performance degradation vs. cost savings
   - Establish acceptable use cases for PPS-only
   - Guidelines for system designers

3. **Practical Implementation**
   - First open-source implementation of PPS-only USRP bonding
   - Integration with UHD framework
   - Reproducible results for community

### 11.2 Potential Publications

1. **Conference Paper** (IEEE VTC, ICC, or GLOBECOM)
   - Title: "Cost-Effective Bandwidth Aggregation for USB-Based SDRs Using PPS-Only Synchronization"
   - Focus: System architecture, drift compensation, results

2. **Journal Article** (IEEE Trans. Wireless Comm. or similar)
   - Title: "Adaptive Frequency Tracking for Multi-Device Software-Defined Radio Systems"
   - Focus: Algorithms, theoretical analysis, comprehensive evaluation

3. **Master's Thesis**
   - Comprehensive treatment of both PPS-only and 10MHz+PPS
   - Implementation details, results, future work
   - Code release to community

### 11.3 Community Impact

**Open Source Contribution**:
- Submit code to UHD repository (Ettus/Analog Devices)
- Share on GitHub with examples
- Create tutorial videos

**Community Engagement**:
- Present at GNU Radio Conference (GRCon)
- Post on USRP Users mailing list
- Write blog posts about implementation

**Enable New Applications**:
- Wide-spectrum monitoring on budget
- Educational projects (universities, makerspaces)
- Citizen science (RF environment monitoring)
- Developing nations (low-cost SDR infrastructure)

---

## 12. Timeline and Milestones (PPS-Only Focus)

| Week | Milestone | Deliverable |
|------|-----------|-------------|
| 1-2 | PPS-Only Setup | GPS receiver, PPS distribution working |
| 3-4 | Time Sync | Devices time-aligned via PPS |
| 5-6 | Drift Estimation (Method 1) | Pilot tone method working |
| 7-8 | Drift Estimation (Method 2) | Cross-correlation method working |
| 9-10 | Drift Compensation | Real-time frequency correction |
| 11-12 | RX Streaming | Receive and combine with drift comp |
| 13-14 | Calibration System | Auto-calibration routines |
| 15-16 | TX Streaming | Transmit with drift compensation |
| 17 | Performance Testing | Benchmark against 10MHz+PPS |
| 18 | Long-term Stability | 24-hour test, thermal drift test |
| 19-20 | Documentation | User guide, API docs, examples |
| 21-22 | Paper Writing | Conference/journal submission |

**Total: 22 weeks (~5.5 months)**

---

## 13. Conclusion

### 13.1 Summary

PPS-only synchronization offers a **cost-effective alternative** to traditional 10MHz+PPS bonding for USRP bandwidth aggregation:

**Advantages**:
- **15x lower cost** for synchronization hardware ($50 vs. $750+)
- **Simpler setup** (single cable vs. dual)
- **Portable** (can use GPS anywhere)
- **Suitable for majority of SDR applications**

**Trade-offs**:
- Requires **software drift compensation** (complex)
- **2-3 dB SNR penalty** vs. hardware-locked
- **Not suitable for phase-critical applications**
- **Shorter coherence time** (seconds vs. hours)

### 13.2 When to Use PPS-Only

✅ **Recommended For:**
- Budget-conscious projects ($2800 vs. $3500+ total)
- Wideband spectrum monitoring (>50 MHz)
- Non-coherent multi-band reception
- Educational/research projects
- Prototype and development
- Applications tolerating 2-5 ppm frequency error

❌ **Not Recommended For:**
- Phase-synchronized MIMO
- Precision frequency measurement (<0.1 ppm)
- Long-integration narrowband applications
- Phase-coherent radar or communications

### 13.3 Research Value

This implementation provides:

1. **Novel Algorithms**: Drift tracking and compensation methods
2. **Practical System**: Working implementation in UHD framework
3. **Performance Data**: Quantified comparison vs. hardware-locked
4. **Community Benefit**: Open-source, lower barrier to entry
5. **Publication Potential**: Conference/journal papers

### 13.4 Next Steps

1. **Acquire Hardware**:
   - 2x USRP B210
   - GPS module with PPS (u-blox NEO-M8N recommended)
   - Basic test equipment

2. **Start Development**:
   - Follow phased implementation plan
   - Begin with PPS-only time synchronization
   - Add drift tracking incrementally

3. **Test and Validate**:
   - Systematic performance evaluation
   - Compare against 10MHz+PPS (if available)
   - Document results

4. **Publish and Share**:
   - Write thesis/papers
   - Release open-source code
   - Engage with SDR community

---

## 14. Appendix: Quick Reference

### 14.1 Hardware Shopping List (PPS-Only)

| Item | Quantity | Est. Cost | Supplier Examples |
|------|----------|-----------|-------------------|
| USRP B210 | 2 | $1300 each | Ettus/Analog Devices |
| GPS Module (NEO-M8N) | 1 | $35 | Amazon, eBay, Adafruit |
| GPS Active Antenna | 1 | $25 | Amazon, SparkFun |
| USB 3.0 Cables (1m) | 2 | $10 each | Amazon, Monoprice |
| SMA Cables (1m) | 2 | $8 each | Amazon, Pasternack |
| SMA Y-Splitter (optional) | 1 | $5 | Amazon |
| **Total** | | **~$2711** | |

### 14.2 Software Dependencies

- UHD 4.0+ (base framework)
- FFTW3 or Volk (FFT library)
- Boost (threads, system)
- C++14 compiler (GCC 7+, Clang 5+)
- Python 3.6+ (for utilities)
- Optional: GNU Radio 3.9+ (for testing)

### 14.3 Key Configuration Parameters

```cpp
// PPS-Only Configuration
sync_mode = "pps_only"
drift_update_interval = 10.0        // seconds
drift_estimation_method = "pilot"   // or "xcorr", "gps"
phase_tracking_enabled = true
guard_band_percentage = 7.5         // % of device bandwidth
recalibration_interval = 30.0       // seconds
```

### 14.4 Performance Targets (PPS-Only)

| Parameter | Target Value |
|-----------|--------------|
| Time Sync | <10 µs |
| Drift Estimation Accuracy | <0.5 ppm |
| Drift Compensation Latency | <100 ms |
| SNR Penalty | <3 dB |
| CPU Usage (2 devices) | <250% (2.5 cores) |
| Combining Artifacts | <-35 dBc |

---

**Document Version**: 1.0  
**Last Updated**: January 27, 2026  
**Author**: PPS-Only Implementation Plan  
**Status**: Ready for Implementation  
**Companion Document**: BONDING_IMPLEMENTATION_PLAN.md (10MHz+PPS approach)
