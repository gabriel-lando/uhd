# USRP Bonding Implementation Plan
## Multi-Device Bandwidth Aggregation for USB-Based USRPs

**Project Goal**: Develop a driver-level mechanism to bond multiple USRP B210 (or other USB-based) devices to create a unified virtual device with aggregated bandwidth capabilities.

---

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [Architecture Overview](#architecture-overview)
3. [Implementation Phases](#implementation-phases)
4. [Detailed Technical Design](#detailed-technical-design)
5. [Synchronization Strategy](#synchronization-strategy)
6. [Testing & Validation](#testing-validation)
7. [Challenges & Mitigations](#challenges-mitigations)
8. [References & Resources](#references-resources)

---

## 1. Executive Summary

### Objective
Create a transparent bonding layer in the UHD framework that allows multiple USB-based USRPs to be combined into a single virtual device, with each physical device handling a distinct frequency slice of the total desired bandwidth.

### Key Features
- **Virtual Device Interface**: Single `multi_usrp` object representing bonded devices
- **Frequency Division**: Automatic spectrum allocation across physical devices
- **Synchronization**: Time and frequency alignment using external 10 MHz + PPS
- **Transparent Operation**: User-facing API remains largely unchanged
- **Scalability**: Support for 2+ devices (initially optimize for 2-device case)

### Target Use Cases
- Wideband spectrum monitoring (>56 MHz instantaneous BW)
- Multi-band simultaneous operation
- Research applications requiring extended bandwidth
- Coherent multi-device MIMO extensions

---

## 2. Architecture Overview

### 2.1 Current UHD Architecture

```
User Application
       |
       v
multi_usrp API (lib/usrp/multi_usrp.cpp)
       |
       v
device abstraction (include/uhd/device.hpp)
       |
       v
Physical USRP (b200_impl, etc.)
       |
       v
USB Transport
```

### 2.2 Proposed Bonded Architecture

```
User Application
       |
       v
multi_usrp API (unchanged interface)
       |
       v
[NEW] bonded_usrp (virtual device layer)
       |
       +---> Physical USRP #1 (freq slice 1)
       |
       +---> Physical USRP #2 (freq slice 2)
       |
       +---> Physical USRP #N (freq slice N)
```

### 2.3 Key Components

1. **bonded_usrp class**: Virtual device aggregator
2. **frequency_slicer**: Manages spectrum allocation
3. **sync_manager**: Handles time/frequency synchronization
4. **stream_combiner**: Merges/splits RX/TX data streams
5. **discovery extensions**: Identifies bondable device sets

---

## 3. Implementation Phases

### Phase 1: Foundation & Discovery (Weeks 1-3)
**Goal**: Establish core infrastructure for device bonding

#### Tasks:
1. **Create bonded_usrp class skeleton**
   - Location: `lib/usrp/bonded_usrp.cpp` and `include/uhd/usrp/bonded_usrp.hpp`
   - Inherit from `multi_usrp` or create parallel interface
   - Implement device container to hold multiple `device::sptr` objects

2. **Extend device discovery mechanism**
   - Modify `lib/usrp/b200/b200_impl.cpp::find()` or create wrapper
   - Add bonding hints in `device_addr_t`:
     ```cpp
     device_addr_t args;
     args["bonded"] = "true";
     args["bond_id"] = "bond_group_1";
     args["serial0"] = "ABC123";
     args["serial1"] = "DEF456";
     ```

3. **Device addressing scheme**
   - Define bonding group identifiers
   - Serial number enumeration and ordering
   - Master/slave designation (for clock/time distribution)

#### Deliverables:
- `bonded_usrp.hpp/cpp` with constructor accepting multiple devices
- Modified discovery that can identify bondable device sets
- Documentation of bonding address format

---

### Phase 2: Synchronization Framework (Weeks 4-6)
**Goal**: Implement robust time and frequency synchronization

#### Tasks:
1. **Clock source management**
   ```cpp
   class sync_manager {
   public:
       void configure_external_refs(
           double ref_clock_freq = 10e6,
           const std::string& time_source = "external"
       );
       
       void synchronize_devices(
           const std::vector<device::sptr>& devices
       );
       
       bool verify_lock_status();
       
   private:
       void set_clock_source_all(const std::string& source);
       void set_time_source_all(const std::string& source);
       void align_time_at_pps();
   };
   ```

2. **Time alignment procedure**
   - Use existing `set_time_next_pps()` pattern
   - Implement verification mechanism
   - Add margin for USB timing uncertainties
   
   Reference implementation (from `examples/sync_to_gps.cpp`):
   ```cpp
   // Set time at next PPS
   usrp->set_time_source("external");
   std::this_thread::sleep_for(std::chrono::seconds(1));
   usrp->set_time_next_pps(uhd::time_spec_t(0.0));
   std::this_thread::sleep_for(std::chrono::seconds(2));
   ```

3. **Phase coherence**
   - Study B210 LO synchronization capabilities
   - Implement phase offset calibration
   - Store/apply correction factors

4. **Hardware requirements validation**
   - Check for external reference inputs on all devices
   - Verify 10 MHz reference lock on all boards
   - Ensure PPS signal arrives at all devices

#### Deliverables:
- `sync_manager` class implementation
- Synchronization test utilities
- Phase alignment calibration tools

---

### Phase 3: Frequency Domain Allocation (Weeks 7-9)
**Goal**: Implement intelligent spectrum slicing

#### Tasks:
1. **Frequency slicer implementation**
   ```cpp
   class frequency_slicer {
   public:
       struct freq_slice {
           double center_freq;
           double bandwidth;
           size_t device_index;
           double lo_offset;  // For image rejection
       };
       
       std::vector<freq_slice> compute_slices(
           double target_center_freq,
           double target_bandwidth,
           size_t num_devices,
           double device_max_bw
       );
       
   private:
       void optimize_lo_placement();
       void handle_guardbands();
   };
   ```

2. **Frequency planning algorithm**
   - For 2x B210s with 56 MHz each = 112 MHz total
   - Calculate optimal LO frequencies with minimal guard bands
   - Example for 100 MHz total BW centered at 2.4 GHz:
     ```
     Device 1: LO = 2.375 GHz, captures 2.347-2.403 GHz
     Device 2: LO = 2.425 GHz, captures 2.397-2.453 GHz
     Overlap region: 2.397-2.403 GHz (managed separately)
     ```

3. **Tune request translation**
   - Intercept `set_rx_freq()` and `set_tx_freq()` calls
   - Distribute to appropriate physical devices
   - Handle edge cases (frequency at slice boundary)

4. **Sample rate coordination**
   - Ensure all devices use same sample rate
   - Validate requested rates against hardware limits
   - Implement `set_rx_rate()` / `set_tx_rate()` forwarding

#### Deliverables:
- Frequency allocation algorithms
- LO optimization strategies
- Visualization tools for spectrum allocation

---

### Phase 4: Stream Management (Weeks 10-13)
**Goal**: Unified data path for RX/TX operations

#### Tasks:
1. **Stream combiner for RX**
   ```cpp
   class bonded_rx_streamer : public rx_streamer {
   public:
       size_t recv(
           const buffs_type& buffs,
           const size_t nsamps_per_buff,
           rx_metadata_t& metadata,
           const double timeout = 0.1,
           const bool one_packet = false
       ) override;
       
   private:
       std::vector<rx_streamer::sptr> _device_streamers;
       
       void recv_from_devices();
       void combine_frequency_slices();
       void detect_overflows();
   };
   ```

2. **RX data path**
   - Create parallel streamers for each device
   - Fetch samples from all devices simultaneously
   - Frequency-domain combining using FFT:
     ```
     Device 1 samples → FFT → Extract frequency slice 1
     Device 2 samples → FFT → Extract frequency slice 2
     Combine slices → IFFT → Output buffer
     ```

3. **TX data path**
   ```cpp
   class bonded_tx_streamer : public tx_streamer {
   public:
       size_t send(
           const buffs_type& buffs,
           const size_t nsamps_per_buff,
           const tx_metadata_t& metadata,
           const double timeout = 0.1
       ) override;
       
   private:
       void split_frequency_slices();
       void send_to_devices();
   };
   ```

4. **TX data splitting**
   - FFT input signal
   - Extract frequency bands for each device
   - IFFT for each device
   - Transmit on all devices with time alignment

5. **Buffer management**
   - Circular buffers for each device stream
   - Handle different USB latencies
   - Implement overflow/underflow recovery

6. **Timestamp coordination**
   - Ensure metadata timestamps are consistent
   - Handle time wrapping/overflow
   - Support timed commands across all devices

#### Deliverables:
- Bonded streamer implementations
- FFT-based frequency combining/splitting
- Performance benchmarks

---

### Phase 5: API Integration (Weeks 14-16)
**Goal**: Seamless integration with existing multi_usrp API

#### Tasks:
1. **Factory method modification**
   - Update `multi_usrp::make()` to detect bonding request
   ```cpp
   // In multi_usrp.cpp
   multi_usrp::sptr multi_usrp::make(const device_addr_t& dev_addr) {
       // Check for bonding flag
       if (dev_addr.has_key("bonded") && dev_addr["bonded"] == "true") {
           return bonded_usrp::make(dev_addr);
       }
       // Existing code...
   }
   ```

2. **API method forwarding**
   - Implement all pure virtual methods from `multi_usrp`
   - Forward get/set operations to appropriate device(s)
   - Examples:
     ```cpp
     void set_rx_gain(double gain, size_t chan) override {
         // Set gain on all devices
         for (auto& dev : _devices) {
             dev->set_rx_gain(gain, chan);
         }
     }
     
     double get_rx_freq(size_t chan) override {
         // Return virtual center frequency (not individual LOs)
         return _virtual_center_freq;
     }
     ```

3. **Property tree integration**
   - Create virtual property tree
   - Aggregate device properties
   - Handle mboard/channel mapping

4. **Error handling**
   - Consistent error reporting across devices
   - Graceful degradation if one device fails
   - Lock status monitoring

#### Deliverables:
- Complete bonded_usrp API implementation
- Compatibility layer for existing applications
- API documentation

---

### Phase 6: Calibration & Optimization (Weeks 17-19)
**Goal**: Achieve optimal performance and accuracy

#### Tasks:
1. **Amplitude calibration**
   - Measure gain differences between devices
   - Apply equalization factors
   - Frequency-dependent correction

2. **Phase calibration**
   - TX/RX phase offset measurement
   - Cable delay compensation
   - Store calibration data in EEPROM or config file

3. **Timing calibration**
   - Measure and compensate for USB latency differences
   - Group delay equalization
   - Sample time alignment verification

4. **Overlap region management**
   - Handle frequency overlap between adjacent slices
   - Implement weighted combining or selection
   - Minimize artifacts at boundaries

5. **Performance optimization**
   - CPU usage profiling
   - FFT library optimization (use FFTW, Volk, or GPU)
   - Multi-threading for parallel processing

6. **Calibration utilities**
   ```bash
   # Proposed calibration tool
   uhd_bonded_calibrate --args="bonded=true,serial0=ABC,serial1=DEF" \
                        --cal-type=phase \
                        --output=bond_cal.dat
   ```

#### Deliverables:
- Calibration procedures and tools
- Performance optimization
- Calibration data format specification

---

### Phase 7: Testing & Validation (Weeks 20-22)
**Goal**: Comprehensive verification

#### Tasks:
1. **Unit tests**
   - Frequency slicer logic
   - Synchronization mechanisms
   - Stream combining algorithms

2. **Integration tests**
   - End-to-end RX/TX with bonded devices
   - Multiple sample rates and bandwidths
   - Various center frequencies

3. **Performance benchmarks**
   - Throughput measurement
   - Latency analysis
   - CPU utilization
   - Spectrum quality (SFDR, phase noise)

4. **Hardware test scenarios**
   - 2-device bonding (primary case)
   - 3+ device bonding
   - Mixed device types (B200 + B210)
   - Various USB controllers and hubs

5. **Example applications**
   ```cpp
   // examples/bonded_rx_samples.cpp
   int main() {
       device_addr_t args;
       args["bonded"] = "true";
       args["serial0"] = "ABC123";
       args["serial1"] = "DEF456";
       
       auto usrp = multi_usrp::make(args);
       
       // Configure synchronization
       usrp->set_clock_source("external");
       usrp->set_time_source("external");
       
       // Request wide bandwidth
       usrp->set_rx_rate(61.44e6 * 2);  // 2x B210 max rate
       usrp->set_rx_freq(2.4e9);
       
       // Normal streaming operations...
   }
   ```

#### Deliverables:
- Comprehensive test suite
- Performance report
- Example applications demonstrating bonded operation

---

## 4. Detailed Technical Design

### 4.1 Core Classes

#### bonded_usrp Class
```cpp
// include/uhd/usrp/bonded_usrp.hpp
namespace uhd { namespace usrp {

class UHD_API bonded_usrp : public multi_usrp {
public:
    typedef std::shared_ptr<bonded_usrp> sptr;
    
    static sptr make(const device_addr_t& bonding_args);
    
    // Override all multi_usrp methods
    void set_rx_freq(const tune_request_t& tune_req, size_t chan) override;
    void set_rx_rate(double rate, size_t chan) override;
    rx_streamer::sptr get_rx_stream(const stream_args_t& args) override;
    // ... etc
    
    // Bonding-specific methods
    size_t get_num_bonded_devices() const;
    device::sptr get_bonded_device(size_t index) const;
    void enable_bonding_mode(bool enable);
    void calibrate_phase_offsets();
    
private:
    struct device_slice_info {
        device::sptr device;
        multi_usrp::sptr usrp_interface;
        double center_freq;
        double bandwidth;
        double phase_offset;
        double gain_correction;
    };
    
    std::vector<device_slice_info> _devices;
    sync_manager::sptr _sync_mgr;
    frequency_slicer::sptr _freq_slicer;
    
    void initialize_devices(const device_addr_t& args);
    void configure_synchronization();
    void compute_frequency_plan(double target_freq, double target_bw);
};

}} // namespace uhd::usrp
```

### 4.2 Synchronization Architecture

```cpp
// lib/usrp/bonded/sync_manager.cpp
class sync_manager {
public:
    struct sync_config {
        double ref_clock_freq;      // e.g., 10e6
        std::string clock_source;   // "external"
        std::string time_source;    // "external"
        bool verify_lock;           // true
        double pps_wait_time;       // 2.0 seconds
    };
    
    void synchronize(
        const std::vector<multi_usrp::sptr>& devices,
        const sync_config& config
    ) {
        // 1. Configure clock sources
        for (auto& dev : devices) {
            dev->set_clock_source(config.clock_source);
            dev->set_time_source(config.time_source);
        }
        
        // 2. Wait for locks
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        if (config.verify_lock) {
            verify_all_locks(devices);
        }
        
        // 3. Synchronize time at next PPS
        const time_spec_t zero_time(0.0);
        for (auto& dev : devices) {
            dev->set_time_next_pps(zero_time);
        }
        
        // 4. Wait for PPS edge
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                static_cast<long>(config.pps_wait_time * 1000)
            )
        );
        
        // 5. Verify time synchronization
        verify_time_sync(devices);
    }
    
private:
    void verify_all_locks(const std::vector<multi_usrp::sptr>& devices);
    void verify_time_sync(const std::vector<multi_usrp::sptr>& devices);
};
```

### 4.3 Frequency Domain Processing

```cpp
// lib/usrp/bonded/frequency_combiner.cpp
class frequency_combiner {
public:
    frequency_combiner(size_t fft_size, size_t num_devices)
        : _fft_size(fft_size)
        , _num_devices(num_devices)
    {
        // Initialize FFT plans
        _fft_forward = volk_fft_malloc_complex(fft_size);
        _fft_inverse = volk_fft_malloc_complex(fft_size);
        
        // Configure FFTW plans (or use volk_fft)
        _plan_forward = volk_fft_make_plan_forward(_fft_size);
        _plan_inverse = volk_fft_make_plan_inverse(_fft_size);
    }
    
    void combine_rx_samples(
        const std::vector<std::complex<float>*>& device_buffers,
        std::complex<float>* output_buffer,
        size_t nsamps,
        const std::vector<freq_slice>& slice_info
    ) {
        // For each FFT block
        for (size_t block = 0; block < nsamps / _fft_size; ++block) {
            // Clear output frequency buffer
            std::memset(_fft_output, 0, sizeof(std::complex<float>) * _fft_size);
            
            // Process each device
            for (size_t dev = 0; dev < _num_devices; ++dev) {
                // FFT device samples
                const size_t offset = block * _fft_size;
                volk_fft_execute_forward(
                    _plan_forward,
                    device_buffers[dev] + offset,
                    _fft_forward
                );
                
                // Extract and place frequency slice
                const auto& slice = slice_info[dev];
                copy_frequency_slice(
                    _fft_forward,
                    _fft_output,
                    slice.start_bin,
                    slice.end_bin
                );
            }
            
            // IFFT combined spectrum
            volk_fft_execute_inverse(
                _plan_inverse,
                _fft_output,
                output_buffer + block * _fft_size
            );
        }
    }
    
    void split_tx_samples(/* similar but reverse */) {
        // FFT input signal
        // Extract frequency slices for each device
        // IFFT each slice
    }
    
private:
    size_t _fft_size;
    size_t _num_devices;
    std::complex<float>* _fft_forward;
    std::complex<float>* _fft_inverse;
    std::complex<float>* _fft_output;
    void* _plan_forward;
    void* _plan_inverse;
    
    void copy_frequency_slice(
        const std::complex<float>* src,
        std::complex<float>* dst,
        size_t start_bin,
        size_t end_bin
    );
};
```

---

## 5. Synchronization Strategy

### 5.1 Hardware Requirements

**Essential:**
- External 10 MHz reference clock (shared by all devices)
- External PPS signal (shared by all devices)
- Clock distribution amplifier (if needed for signal integrity)
- Matched cable lengths (or calibrated delays)

**Recommended:**
- GPS-disciplined oscillator (GPSDO) for long-term stability
- Low-phase-noise 10 MHz reference (< -140 dBc/Hz @ 10 kHz)
- Active antenna for GPSDO (if using GPS)

### 5.2 Connection Topology

```
                    +------------------+
                    |  10 MHz Reference|
                    |   (e.g., GPSDO)  |
                    |  + PPS Output    |
                    +--------+---------+
                             |
                    +--------v---------+
                    | Clock Splitter/  |
                    | Distribution Amp |
                    +--+-------+-------+
                       |       |
          10MHz+PPS    |       |    10MHz+PPS
              +--------+       +--------+
              |                         |
       +------v-------+          +------v-------+
       |   USRP B210  |          |   USRP B210  |
       |  (Device 0)  |          |  (Device 1)  |
       +------+-------+          +------+-------+
              |                         |
          USB 3.0                   USB 3.0
              |                         |
              +------------+------------+
                           |
                    +------v-------+
                    |  Host PC     |
                    | (Linux/USB3) |
                    +--------------+
```

### 5.3 Synchronization Procedure

```cpp
// Detailed synchronization sequence
void perform_device_synchronization(bonded_usrp* bonded) {
    // Step 1: Configure external references
    for (size_t i = 0; i < bonded->get_num_bonded_devices(); ++i) {
        auto usrp = bonded->get_bonded_device(i);
        
        usrp->set_clock_source("external", 0);
        usrp->set_time_source("external", 0);
        
        // For B200/B210, may need to set reference frequency
        // (10 MHz is default, but can be changed)
        if (usrp->get_mboard_name() == "B200" || 
            usrp->get_mboard_name() == "B210") {
            // B210 doesn't have programmable ref freq,
            // but verify 10 MHz is connected
        }
    }
    
    // Step 2: Wait for clock lock
    std::cout << "Waiting for 10 MHz reference lock..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    // Step 3: Verify lock status
    for (size_t i = 0; i < bonded->get_num_bonded_devices(); ++i) {
        auto usrp = bonded->get_bonded_device(i);
        
        // Check if sensor exists (B210 may not have ref_locked sensor)
        auto sensors = usrp->get_mboard_sensor_names(0);
        if (std::find(sensors.begin(), sensors.end(), "ref_locked") 
            != sensors.end()) {
            
            auto ref_locked = usrp->get_mboard_sensor("ref_locked", 0);
            if (!ref_locked.to_bool()) {
                throw uhd::runtime_error(
                    "Device " + std::to_string(i) + " failed to lock to 10 MHz reference"
                );
            }
        }
    }
    
    // Step 4: Align time at next PPS
    std::cout << "Synchronizing time at next PPS..." << std::endl;
    const uhd::time_spec_t start_time(0.0);
    
    for (size_t i = 0; i < bonded->get_num_bonded_devices(); ++i) {
        bonded->get_bonded_device(i)->set_time_next_pps(start_time);
    }
    
    // Step 5: Wait for PPS edge (plus margin)
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Step 6: Verify time synchronization
    std::cout << "Verifying time sync..." << std::endl;
    double max_time_diff = 0.0;
    uhd::time_spec_t reference_time;
    
    for (size_t i = 0; i < bonded->get_num_bonded_devices(); ++i) {
        auto current_time = bonded->get_bonded_device(i)->get_time_now();
        
        if (i == 0) {
            reference_time = current_time;
        } else {
            double time_diff = std::abs(
                current_time.get_real_secs() - reference_time.get_real_secs()
            );
            max_time_diff = std::max(max_time_diff, time_diff);
        }
        
        std::cout << "Device " << i << " time: " 
                  << current_time.get_real_secs() << " seconds" << std::endl;
    }
    
    // Step 7: Check synchronization quality
    const double MAX_ALLOWED_TIME_DIFF = 10e-6;  // 10 microseconds
    if (max_time_diff > MAX_ALLOWED_TIME_DIFF) {
        std::cerr << "WARNING: Time difference between devices is " 
                  << max_time_diff * 1e6 << " us, which exceeds threshold of "
                  << MAX_ALLOWED_TIME_DIFF * 1e6 << " us" << std::endl;
    } else {
        std::cout << "Time synchronization successful! Max diff: " 
                  << max_time_diff * 1e9 << " ns" << std::endl;
    }
}
```

### 5.4 B210-Specific Considerations

**Clock Architecture:**
- B210 has internal 40 MHz VCTCXO
- Can lock to external 10 MHz reference
- No programmable reference frequency support
- AD9361 transceiver handles clock multiplication

**Reference Input:**
- J3 connector for external 10 MHz (+7 dBm nominal)
- J2 connector for external PPS (3.3V CMOS/TTL)
- Both are SMA connectors on rear panel

**Limitations:**
- No built-in GPSDO option
- USB 3.0 latency variations
- Shared USB bandwidth between devices

---

## 6. Testing & Validation

### 6.1 Unit Test Plan

```cpp
// tests/bonded_usrp_test.cpp

TEST(BondedUSRP, DeviceDiscovery) {
    // Test finding multiple devices
    device_addrs_t found = device::find("type=b200");
    EXPECT_GE(found.size(), 2);
}

TEST(BondedUSRP, SynchronizationBasic) {
    // Test clock and time source configuration
    device_addr_t args;
    args["bonded"] = "true";
    args["serial0"] = "ABC123";
    args["serial1"] = "DEF456";
    
    auto bonded = bonded_usrp::make(args);
    bonded->set_clock_source("external");
    bonded->set_time_source("external");
    
    // Verify all devices are synchronized
    // (implementation-specific checks)
}

TEST(BondedUSRP, FrequencyAllocation) {
    frequency_slicer slicer;
    
    auto slices = slicer.compute_slices(
        2.4e9,     // center freq
        100e6,     // total BW
        2,         // num devices
        56e6       // per-device BW
    );
    
    EXPECT_EQ(slices.size(), 2);
    EXPECT_NEAR(slices[0].center_freq, 2.375e9, 1e6);
    EXPECT_NEAR(slices[1].center_freq, 2.425e9, 1e6);
}

TEST(BondedUSRP, StreamCombining) {
    // Test RX stream combining
    // Generate known test signals on each device
    // Verify combined output matches expected spectrum
}
```

### 6.2 Integration Tests

**Test Scenario 1: Wide Spectrum Monitoring**
```
Configuration: 2x B210
Center Frequency: 2.4 GHz
Total Bandwidth: 100 MHz
Sample Rate: 61.44 MHz (per device)

Expected Result:
- Continuous spectrum from 2.35 to 2.45 GHz
- No gaps or artifacts at slice boundaries
- SNR within 3 dB of single-device operation
```

**Test Scenario 2: Multi-Band Operation**
```
Configuration: 2x B210
Device 1: 900 MHz band (cellular)
Device 2: 2.4 GHz band (WiFi)

Expected Result:
- Simultaneous reception from both bands
- Independent gain/antenna control per band
- Proper timestamp correlation
```

### 6.3 Performance Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Time Sync Accuracy | < 1 µs | Compare `get_time_now()` between devices |
| Phase Coherence | < 10° RMS | Transmit tone, measure phase difference |
| Throughput | > 100 MS/s | Sustained RX data rate to disk |
| CPU Utilization | < 200% (2 cores) | Linux `top` or `perf` |
| Latency | < 10 ms | Time from trigger to data output |
| Spectrum Continuity | < -40 dBc artifacts | Spectrum analyzer measurement |

### 6.4 Hardware Test Matrix

| Test Case | Devices | USB Config | Clock/PPS | Expected Result |
|-----------|---------|------------|-----------|-----------------|
| Basic 2-dev bonding | 2x B210 | Separate USB3 | External both | Pass |
| 3-device bonding | 3x B210 | Separate USB3 | External both | Pass |
| USB hub operation | 2x B210 | Through USB3 hub | External both | Degraded perf warning |
| Mixed devices | B200 + B210 | Separate USB3 | External both | Pass with caution |
| No external ref | 2x B210 | Separate USB3 | Internal both | Fail with error message |

---

## 7. Challenges & Mitigations

### 7.1 Technical Challenges

#### Challenge 1: USB Bandwidth Limitations
**Problem**: USB 3.0 theoretical limit is 5 Gbps, but practical throughput is ~3.2 Gbps. Two B210s at full rate (2 × 61.44 MS/s × 4 bytes × 2 channels) = 983 Mbps each = ~2 Gbps total. However, USB overhead and OS scheduling can cause issues.

**Mitigation**:
- Use separate USB 3.0 controllers for each device (not the same hub)
- Optimize USB buffer sizes in UHD transport layer
- Implement adaptive sample rate reduction if bandwidth insufficient
- Consider USB 3.1 Gen 2 (10 Gbps) if available

#### Challenge 2: Timing Jitter and Skew
**Problem**: USB introduces variable latency (typically 100-500 µs jitter), making sample-level timing coordination difficult.

**Mitigation**:
- Use hardware timestamps for all samples
- Implement jitter buffers to absorb timing variations
- Apply post-processing time alignment in frequency domain
- Use external triggers for critical timing events

#### Challenge 3: Frequency Overlap Artifacts
**Problem**: FFT-based combining can create artifacts at slice boundaries due to windowing effects and filter roll-off.

**Mitigation**:
- Design overlapping frequency slices with smooth transition zones
- Apply weighting functions in overlap regions
- Use guard bands (sacrifice 5-10% bandwidth for cleaner combining)
- Implement adaptive filtering based on signal conditions

#### Challenge 4: Phase Coherence
**Problem**: Even with external reference, devices may have phase offsets due to different cable lengths, device variations, or initialization timing.

**Mitigation**:
- Implement phase calibration routine (transmit known tone, measure phase)
- Store calibration data persistently
- Apply phase corrections in frequency domain during combining
- Periodic re-calibration during long acquisitions

#### Challenge 5: Different Device Temperatures
**Problem**: B210 devices may operate at different temperatures, causing frequency drift over time despite common reference.

**Mitigation**:
- Monitor temperature sensors if available
- Implement slow tracking loop for frequency correction
- Use short calibration bursts periodically
- Consider thermal management (fans, heat sinks)

### 7.2 Software Challenges

#### Challenge 6: Complexity of Multi-threaded Streaming
**Problem**: Coordinating parallel data streams, FFT processing, and buffer management across multiple threads is complex and error-prone.

**Mitigation**:
- Use proven threading libraries (Boost.Thread, C++11 std::thread)
- Implement clear separation of concerns (producer/consumer pattern)
- Extensive unit testing of threading logic
- Use thread-safe queue implementations

#### Challenge 7: Backward Compatibility
**Problem**: Changes to core UHD API could break existing applications.

**Mitigation**:
- Keep bonding features opt-in (via device address arguments)
- Maintain existing API behavior for non-bonded devices
- Provide compatibility layer for deprecated methods
- Comprehensive regression testing

#### Challenge 8: Error Recovery
**Problem**: If one device fails during operation, entire bonded system could crash.

**Mitigation**:
- Implement per-device health monitoring
- Graceful degradation (continue with remaining devices)
- Automatic reconnection for USB disconnects
- Clear error messages indicating which device failed

### 7.3 User Experience Challenges

#### Challenge 9: Configuration Complexity
**Problem**: Users need to understand synchronization, bonding groups, device ordering, etc.

**Mitigation**:
- Provide high-level helper utilities (e.g., `uhd_bonded_setup`)
- Auto-detect bondable device sets
- Smart defaults (auto-configure external refs if available)
- Clear documentation with examples

#### Challenge 10: Debugging Bonded Systems
**Problem**: Troubleshooting issues across multiple devices is more difficult than single-device debugging.

**Mitigation**:
- Enhanced logging with per-device tags
- Visualization tools showing sync status, frequency allocation, etc.
- Built-in diagnostic commands (e.g., "uhd_bonded_check")
- Community forum for bonding-specific issues

---

## 8. References & Resources

### 8.1 UHD Source Code Files (Key Files to Study)

| File | Purpose |
|------|---------|
| `lib/usrp/multi_usrp.cpp` | Core multi_usrp implementation, template for bonded_usrp |
| `lib/usrp/b200/b200_impl.cpp` | B210 device driver, understand USB streaming |
| `lib/usrp/b200/b200_io_impl.cpp` | B210 streaming implementation |
| `include/uhd/usrp/multi_usrp.hpp` | Multi_usrp API definition |
| `include/uhd/device.hpp` | Base device interface |
| `lib/rfnoc/rfnoc_graph.cpp` | RFNoC streaming (reference for advanced streaming) |
| `examples/sync_to_gps.cpp` | Synchronization example |
| `examples/tx_waveforms.cpp` | Multi-board synchronization example |
| `lib/transport/usb_zero_copy.cpp` | USB transport layer |

### 8.2 Documentation References

- **UHD Manual**: https://files.ettus.com/manual/
- **Synchronization Guide**: UHD Manual Section "Synchronization"
- **B200/B210 Manual**: Device-specific documentation
- **AD9361 Datasheet**: Understanding RF chip architecture
- **MIMO Cable Application Note**: Ettus Research AN-xxx

### 8.3 External References

1. **"Software-Defined Radio Receiver Design"** - James Tsui, Sections on multi-channel receivers
2. **"Understanding Digital Signal Processing"** - Richard Lyons, Chapters on FFT and filter banks
3. **IEEE Paper**: "Synchronized Wideband Spectrum Sensing Using Multiple USRPs"
4. **GNU Radio Multi-USRP**: Study gr-uhd implementation for multi-device coordination

### 8.4 Tools and Libraries

| Tool/Library | Purpose |
|--------------|---------|
| FFTW | High-performance FFT library |
| Volk | Vector-optimized operations (SIMD) |
| GNU Radio | Reference for multi-USRP streaming |
| Octave/MATLAB | Algorithm prototyping and simulation |
| Python + UHD | Rapid prototyping of bonding logic |
| Wireshark | USB traffic analysis for debugging |
| `uhd_find_devices` | Device discovery tool |
| `uhd_usrp_probe` | Device capability inspection |

### 8.5 Hardware Requirements Summary

**Minimum Setup (2-device bonding):**
- 2x USRP B210
- 2x USB 3.0 cables (short, high-quality)
- 1x 10 MHz reference source (e.g., Mini-Circuits GPSDO-IIM)
- 1x Clock distribution amplifier (if needed)
- 2x SMA cables for 10 MHz (matched lengths)
- 2x SMA cables for PPS (matched lengths)
- Host PC with 2+ separate USB 3.0 controllers
- Linux OS (Ubuntu 20.04+ recommended)

**Recommended Upgrades:**
- GPS-disciplined 10 MHz/PPS source (e.g., Jackson Labs Fury)
- RF-quality SMA cables (low loss, phase-stable)
- USB 3.1 Gen 2 controllers
- High-performance CPU (8+ cores) for FFT processing
- SSD for high-throughput data capture

---

## 9. Timeline and Milestones

| Milestone | Target Week | Deliverable |
|-----------|-------------|-------------|
| M1: Design Review | Week 2 | Architecture document approved |
| M2: Discovery Working | Week 3 | Can enumerate and initialize bonded devices |
| M3: Synchronization Demo | Week 6 | Devices locked and time-aligned |
| M4: Frequency Planning | Week 9 | LO frequencies properly allocated |
| M5: RX Streaming | Week 13 | Can receive and combine samples |
| M6: TX Streaming | Week 15 | Can split and transmit samples |
| M7: API Complete | Week 16 | Full multi_usrp compatibility |
| M8: Calibration Tools | Week 19 | Phase/amplitude calibration working |
| M9: Testing Complete | Week 22 | All test scenarios passing |
| M10: Documentation | Week 24 | User guide and examples published |
| M11: Research Paper | Week 26 | Submit findings to conference/journal |

---

## 10. Future Extensions

### Potential Enhancements Beyond Initial Implementation:

1. **Adaptive Frequency Allocation**
   - Dynamic reallocation based on spectral occupancy
   - Cognitive radio applications

2. **GPU Acceleration**
   - Offload FFT processing to GPU (CUDA/OpenCL)
   - Real-time processing for higher bandwidths

3. **Network-Based Bonding**
   - Bond USRPs across multiple machines
   - Distributed processing for massive arrays

4. **Automatic Calibration**
   - Self-calibration without external signal generator
   - Machine learning for optimal parameter selection

5. **Heterogeneous Bonding**
   - Mix different USRP models (B210 + N210)
   - Different sample rates per device

6. **MIMO Extensions**
   - Not just bandwidth bonding, but true MIMO
   - Beamforming and spatial multiplexing

7. **Real-time Signal Processing**
   - Integration with GNU Radio or other SDR frameworks
   - Zero-copy streaming to GPU

---

## 11. Risk Assessment

| Risk | Probability | Impact | Mitigation Priority |
|------|-------------|--------|---------------------|
| USB bandwidth insufficient | Medium | High | High - Test early |
| Synchronization drift | Medium | High | High - Continuous monitoring |
| FFT artifacts | Low | Medium | Medium - Design overlap zones |
| Device temperature effects | Low | Medium | Low - Environmental control |
| Software bugs in threading | High | High | High - Extensive testing |
| Hardware compatibility | Medium | Low | Medium - Document requirements |
| User adoption | Medium | Low | Low - Focus on documentation |

---

## 12. Success Criteria

The bonding implementation will be considered successful if:

1. **Functional Requirements**:
   - ✓ Two B210 devices can be bonded and operated as single device
   - ✓ Achieved instantaneous bandwidth > 100 MHz
   - ✓ Time synchronization < 1 µs between devices
   - ✓ Phase coherence < 10° RMS

2. **Performance Requirements**:
   - ✓ Sustained throughput > 200 MS/s (100 MS/s per device)
   - ✓ CPU utilization < 300% (3 cores) on modern processor
   - ✓ Spectrum artifacts < -40 dBc at slice boundaries

3. **Usability Requirements**:
   - ✓ Existing multi_usrp applications work with minimal changes
   - ✓ Clear error messages for configuration issues
   - ✓ Comprehensive documentation and examples

4. **Research Requirements**:
   - ✓ Publishable results demonstrating bonding effectiveness
   - ✓ Open-source contribution to UHD community
   - ✓ Validation with real-world communication signals

---

## Conclusion

This implementation plan provides a comprehensive roadmap for developing USRP bonding capability in the UHD framework. The phased approach allows for incremental development and testing, while the detailed technical design sections provide concrete guidance for implementation.

The key to success will be:
1. **Robust synchronization** using external references
2. **Efficient frequency-domain processing** for combining/splitting
3. **Careful thread and buffer management** for real-time performance
4. **Comprehensive testing** at each phase

By following this plan, you should be able to create a working prototype within 6 months, with refinement and optimization continuing thereafter. This research will contribute to the SDR community by enabling new applications requiring extended bandwidth on affordable USB-based USRP platforms.

---

**Document Version**: 1.0  
**Last Updated**: January 27, 2026  
**Author**: Implementation Plan Generator  
**Status**: Ready for Review and Implementation
