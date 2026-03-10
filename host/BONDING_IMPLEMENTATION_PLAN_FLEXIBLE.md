# USRP Bonding Implementation Plan - Flexible Synchronization
## Configurable Multi-Device Bandwidth Aggregation with Runtime Sync Mode Selection

**Project Goal**: Develop a flexible, configurable driver-level bonding mechanism for USB-based USRPs that supports multiple synchronization modes (10MHz+PPS, PPS-only, GPSDO, or internal) selectable at runtime by the user.

---

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [Flexible Architecture Design](#flexible-architecture)
3. [Configuration System](#configuration-system)
4. [Implementation Phases](#implementation-phases)
5. [Synchronization Modes](#synchronization-modes)
6. [Auto-Detection & Fallback](#auto-detection)
7. [API Design](#api-design)
8. [Testing & Validation](#testing-validation)
9. [User Guide](#user-guide)

---

## 1. Executive Summary

### Objective
Create a **unified bonding system** that supports multiple synchronization strategies, configurable by the user at device initialization time, with intelligent defaults and automatic fallback capabilities.

### Core Philosophy
**"One Implementation, Multiple Modes"**
- Single codebase supports all sync modes
- Configuration-driven behavior (not compile-time)
- Runtime mode selection via device arguments
- Automatic detection of available hardware
- Graceful degradation when optimal hardware unavailable

### Key Features
- ✅ **Multi-Mode Support**: 10MHz+PPS, PPS-only, GPSDO, internal, or mixed
- ✅ **Runtime Configuration**: User selects sync mode via device args
- ✅ **Auto-Detection**: Automatically detect available sync signals
- ✅ **Smart Defaults**: Works out-of-box with PPS + external clock
- ✅ **Graceful Fallback**: Degrades to best available mode
- ✅ **Per-Device Configuration**: Each device can have different settings
- ✅ **Validation & Warnings**: Alerts user to suboptimal configurations
- ✅ **Flexible API**: Easy to extend for future sync methods

### Configuration Examples

```cpp
// Example 1: Default configuration (PPS + external clock)
device_addr_t args;
args["bonded"] = "true";
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";
// Uses PPS-only by default with external clock source

// Example 2: Optimal configuration (10MHz + PPS)
args["bonded"] = "true";
args["sync_clock_source"] = "external";      // 10 MHz reference
args["sync_time_source"] = "external";       // PPS signal
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";

// Example 3: GPSDO mode
args["bonded"] = "true";
args["sync_clock_source"] = "gpsdo";
args["sync_time_source"] = "gpsdo";
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";

// Example 4: Per-device configuration
args["bonded"] = "true";
args["clock_source0"] = "external";   // Device 0: 10MHz ref
args["time_source0"] = "external";    // Device 0: PPS
args["clock_source1"] = "internal";   // Device 1: Internal TCXO
args["time_source1"] = "external";    // Device 1: PPS only
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";

// Example 5: Auto-detect mode (use best available)
args["bonded"] = "true";
args["sync_mode"] = "auto";  // Detect and use best available
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";
```

---

## 2. Flexible Architecture Design

### 2.1 Synchronization Abstraction Layer

The key to flexibility is **abstracting synchronization into a pluggable architecture**:

```
User Application
       |
       v
multi_usrp API (device args with sync config)
       |
       v
bonded_usrp (virtual device)
       |
       v
+------------------+
| Sync Manager     |  ← Configuration-driven
| (Mode Selector)  |
+------------------+
       |
       +--> Clock Source Strategy
       |         ├─ External 10MHz Strategy
       |         ├─ Internal TCXO Strategy
       |         ├─ GPSDO Strategy
       |         └─ Future: Network Time Strategy
       |
       +--> Time Source Strategy
       |         ├─ External PPS Strategy
       |         ├─ GPSDO PPS Strategy
       |         ├─ Internal Timer Strategy
       |         └─ Future: Software PPS Strategy
       |
       +--> Drift Compensation Strategy
                 ├─ None (hardware locked)
                 ├─ Pilot Tone Tracking
                 ├─ Cross-Correlation
                 └─ GPS Timestamp Method
```

### 2.2 Core Abstraction: Synchronization Strategy Pattern

```cpp
// include/uhd/usrp/bonded/sync_strategy.hpp

namespace uhd { namespace usrp { namespace bonded {

// Base class for clock synchronization strategies
class clock_sync_strategy {
public:
    virtual ~clock_sync_strategy() = default;
    
    virtual void configure_devices(
        const std::vector<multi_usrp::sptr>& devices
    ) = 0;
    
    virtual bool verify_lock() = 0;
    
    virtual double get_frequency_accuracy_ppm() const = 0;
    
    virtual bool requires_drift_compensation() const = 0;
    
    virtual std::string get_description() const = 0;
};

// Base class for time synchronization strategies
class time_sync_strategy {
public:
    virtual ~time_sync_strategy() = default;
    
    virtual void synchronize_time(
        const std::vector<multi_usrp::sptr>& devices
    ) = 0;
    
    virtual bool verify_synchronization() = 0;
    
    virtual double get_time_accuracy_seconds() const = 0;
    
    virtual std::string get_description() const = 0;
};

// Base class for drift compensation strategies
class drift_compensation_strategy {
public:
    virtual ~drift_compensation_strategy() = default;
    
    virtual void initialize(
        const std::vector<multi_usrp::sptr>& devices
    ) = 0;
    
    virtual drift_estimate estimate_drift(
        size_t reference_device,
        size_t target_device
    ) = 0;
    
    virtual void apply_compensation(
        std::complex<float>* samples,
        size_t nsamps,
        const drift_estimate& drift
    ) = 0;
    
    virtual bool is_enabled() const = 0;
};

}}} // namespace uhd::usrp::bonded
```

### 2.3 Concrete Strategy Implementations

```cpp
// Concrete strategies for different sync modes

// 1. External 10 MHz Clock Strategy
class external_10mhz_clock_strategy : public clock_sync_strategy {
public:
    void configure_devices(
        const std::vector<multi_usrp::sptr>& devices
    ) override {
        for (auto& dev : devices) {
            dev->set_clock_source("external");
        }
    }
    
    double get_frequency_accuracy_ppm() const override {
        return 0.01;  // Excellent accuracy
    }
    
    bool requires_drift_compensation() const override {
        return false;  // Hardware locked
    }
    
    std::string get_description() const override {
        return "External 10 MHz reference (hardware-locked)";
    }
};

// 2. Internal TCXO Clock Strategy
class internal_tcxo_clock_strategy : public clock_sync_strategy {
public:
    void configure_devices(
        const std::vector<multi_usrp::sptr>& devices
    ) override {
        for (auto& dev : devices) {
            dev->set_clock_source("internal");
        }
    }
    
    double get_frequency_accuracy_ppm() const override {
        return 2.5;  // Typical TCXO accuracy
    }
    
    bool requires_drift_compensation() const override {
        return true;  // Independent oscillators
    }
    
    std::string get_description() const override {
        return "Internal TCXO (requires drift compensation)";
    }
};

// 3. GPSDO Clock Strategy
class gpsdo_clock_strategy : public clock_sync_strategy {
public:
    void configure_devices(
        const std::vector<multi_usrp::sptr>& devices
    ) override {
        for (auto& dev : devices) {
            dev->set_clock_source("gpsdo");
        }
    }
    
    double get_frequency_accuracy_ppm() const override {
        return 0.001;  // GPS-disciplined accuracy
    }
    
    bool requires_drift_compensation() const override {
        return false;  // Each device disciplined independently
    }
    
    std::string get_description() const override {
        return "GPSDO (GPS-disciplined oscillator)";
    }
};

// Similar implementations for time_sync_strategy:
// - external_pps_time_strategy
// - gpsdo_pps_time_strategy
// - internal_time_strategy

// And for drift_compensation_strategy:
// - no_drift_compensation (for locked clocks)
// - pilot_tone_drift_compensation
// - cross_correlation_drift_compensation
// - gps_timestamp_drift_compensation
```

---

## 3. Configuration System

### 3.1 Configuration Structure

```cpp
// include/uhd/usrp/bonded/sync_config.hpp

namespace uhd { namespace usrp { namespace bonded {

struct sync_configuration {
    // Clock source configuration
    enum class clock_source_mode {
        AUTO_DETECT,    // Automatically select best available
        EXTERNAL_10MHZ, // Require external 10 MHz reference
        INTERNAL_TCXO,  // Use internal oscillators
        GPSDO,          // Use GPS-disciplined oscillators
        PER_DEVICE      // Each device configured individually
    };
    clock_source_mode clock_mode = clock_source_mode::AUTO_DETECT;
    
    // Time source configuration
    enum class time_source_mode {
        AUTO_DETECT,    // Automatically select best available
        EXTERNAL_PPS,   // External PPS signal
        GPSDO_PPS,      // PPS from GPSDO
        INTERNAL,       // Internal time (no external sync)
        PER_DEVICE      // Each device configured individually
    };
    time_source_mode time_mode = time_source_mode::EXTERNAL_PPS;
    
    // Drift compensation mode
    enum class drift_compensation_mode {
        AUTO,           // Enable if needed
        FORCE_ENABLE,   // Always enable
        FORCE_DISABLE,  // Never enable (even if needed)
        PILOT_TONE,     // Use pilot tone method
        CROSS_CORR,     // Use cross-correlation
        GPS_TIMESTAMP   // Use GPS timestamp method
    };
    drift_compensation_mode drift_mode = drift_compensation_mode::AUTO;
    
    // Per-device overrides (optional)
    std::map<size_t, std::string> per_device_clock_source;
    std::map<size_t, std::string> per_device_time_source;
    
    // Validation settings
    bool strict_validation = false;  // Fail if optimal config unavailable
    bool warnings_enabled = true;    // Show warnings for suboptimal config
    
    // Advanced settings
    double drift_update_interval_sec = 10.0;
    double calibration_interval_sec = 30.0;
    bool auto_fallback = true;  // Fall back to available modes
};

// Factory function to create configuration from device_addr_t
sync_configuration parse_sync_config(const device_addr_t& dev_addr);

// Validation function
struct validation_result {
    bool is_valid;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::string recommended_config;
};

validation_result validate_sync_config(
    const sync_configuration& config,
    const std::vector<multi_usrp::sptr>& devices
);

}}} // namespace uhd::usrp::bonded
```

### 3.2 Configuration Parser

```cpp
// lib/usrp/bonded/sync_config.cpp

sync_configuration parse_sync_config(const device_addr_t& dev_addr) {
    sync_configuration config;
    
    // Parse clock source mode
    if (dev_addr.has_key("sync_clock_source")) {
        std::string clock_src = dev_addr["sync_clock_source"];
        if (clock_src == "auto") {
            config.clock_mode = clock_source_mode::AUTO_DETECT;
        } else if (clock_src == "external") {
            config.clock_mode = clock_source_mode::EXTERNAL_10MHZ;
        } else if (clock_src == "internal") {
            config.clock_mode = clock_source_mode::INTERNAL_TCXO;
        } else if (clock_src == "gpsdo") {
            config.clock_mode = clock_source_mode::GPSDO;
        } else {
            throw uhd::value_error(
                "Invalid clock source: " + clock_src + 
                ". Valid options: auto, external, internal, gpsdo"
            );
        }
    }
    
    // Parse time source mode
    if (dev_addr.has_key("sync_time_source")) {
        std::string time_src = dev_addr["sync_time_source"];
        if (time_src == "auto") {
            config.time_mode = time_source_mode::AUTO_DETECT;
        } else if (time_src == "external") {
            config.time_mode = time_source_mode::EXTERNAL_PPS;
        } else if (time_src == "gpsdo") {
            config.time_mode = time_source_mode::GPSDO_PPS;
        } else if (time_src == "internal") {
            config.time_mode = time_source_mode::INTERNAL;
        } else {
            throw uhd::value_error(
                "Invalid time source: " + time_src + 
                ". Valid options: auto, external, gpsdo, internal"
            );
        }
    }
    
    // Parse per-device overrides
    for (const auto& key : dev_addr.keys()) {
        // Check for clock_source0, clock_source1, etc.
        if (key.find("clock_source") == 0 && key.length() > 12) {
            size_t device_idx = std::stoul(key.substr(12));
            config.per_device_clock_source[device_idx] = dev_addr[key];
            config.clock_mode = clock_source_mode::PER_DEVICE;
        }
        
        // Check for time_source0, time_source1, etc.
        if (key.find("time_source") == 0 && key.length() > 11) {
            size_t device_idx = std::stoul(key.substr(11));
            config.per_device_time_source[device_idx] = dev_addr[key];
            config.time_mode = time_source_mode::PER_DEVICE;
        }
    }
    
    // Parse drift compensation mode
    if (dev_addr.has_key("drift_compensation")) {
        std::string drift_mode = dev_addr["drift_compensation"];
        if (drift_mode == "auto") {
            config.drift_mode = drift_compensation_mode::AUTO;
        } else if (drift_mode == "enable" || drift_mode == "true") {
            config.drift_mode = drift_compensation_mode::FORCE_ENABLE;
        } else if (drift_mode == "disable" || drift_mode == "false") {
            config.drift_mode = drift_compensation_mode::FORCE_DISABLE;
        } else if (drift_mode == "pilot_tone") {
            config.drift_mode = drift_compensation_mode::PILOT_TONE;
        } else if (drift_mode == "cross_corr") {
            config.drift_mode = drift_compensation_mode::CROSS_CORR;
        } else if (drift_mode == "gps_timestamp") {
            config.drift_mode = drift_compensation_mode::GPS_TIMESTAMP;
        }
    }
    
    // Parse validation settings
    if (dev_addr.has_key("sync_strict")) {
        config.strict_validation = 
            (dev_addr["sync_strict"] == "true" || dev_addr["sync_strict"] == "1");
    }
    
    if (dev_addr.has_key("sync_warnings")) {
        config.warnings_enabled = 
            (dev_addr["sync_warnings"] == "true" || dev_addr["sync_warnings"] == "1");
    }
    
    // Parse advanced settings
    if (dev_addr.has_key("drift_update_interval")) {
        config.drift_update_interval_sec = std::stod(dev_addr["drift_update_interval"]);
    }
    
    if (dev_addr.has_key("calibration_interval")) {
        config.calibration_interval_sec = std::stod(dev_addr["calibration_interval"]);
    }
    
    return config;
}
```

### 3.3 Default Configuration

```cpp
// Default configuration when user doesn't specify
// Default: PPS + external clock (simplest, works for most cases)

sync_configuration get_default_config() {
    sync_configuration config;
    
    // Default to PPS synchronization (time only)
    config.clock_mode = clock_source_mode::INTERNAL_TCXO;
    config.time_mode = time_source_mode::EXTERNAL_PPS;
    
    // Auto-enable drift compensation when needed
    config.drift_mode = drift_compensation_mode::AUTO;
    
    // Enable warnings but don't fail on suboptimal config
    config.strict_validation = false;
    config.warnings_enabled = true;
    config.auto_fallback = true;
    
    return config;
}
```

---

## 4. Implementation Phases

### Phase 1: Core Abstraction Layer (Weeks 1-3)

#### Tasks:
1. **Define strategy interfaces**
   - Create base classes for clock/time/drift strategies
   - Design configuration structure
   - Implement parser for device arguments

2. **Implement strategy registry**
   ```cpp
   class strategy_registry {
   public:
       static void register_clock_strategy(
           const std::string& name,
           std::function<std::unique_ptr<clock_sync_strategy>()> factory
       );
       
       static std::unique_ptr<clock_sync_strategy> create_clock_strategy(
           const std::string& name
       );
       
       // Similar for time and drift strategies
   };
   ```

3. **Configuration validation system**
   - Check for incompatible combinations
   - Verify hardware capabilities
   - Generate helpful error messages

#### Deliverables:
- Strategy interface headers
- Configuration parser
- Validation framework

---

### Phase 2: Basic Strategies Implementation (Weeks 4-7)

#### Tasks:
1. **Implement clock strategies**
   - External 10 MHz strategy
   - Internal TCXO strategy
   - GPSDO strategy

2. **Implement time strategies**
   - External PPS strategy
   - GPSDO PPS strategy
   - Internal timer strategy

3. **Implement basic drift compensation**
   - No compensation (for locked clocks)
   - Simple GPS timestamp method

4. **Integration with bonded_usrp**
   ```cpp
   class bonded_usrp : public multi_usrp {
   public:
       static sptr make(const device_addr_t& dev_addr) {
           // Parse configuration
           auto config = parse_sync_config(dev_addr);
           
           // Validate
           auto validation = validate_sync_config(config, devices);
           if (!validation.is_valid && config.strict_validation) {
               throw uhd::runtime_error("Invalid sync configuration");
           }
           
           // Create appropriate strategies
           auto clock_strategy = create_clock_strategy(config);
           auto time_strategy = create_time_strategy(config);
           auto drift_strategy = create_drift_strategy(config);
           
           // Construct bonded_usrp
           return std::make_shared<bonded_usrp_impl>(
               devices, clock_strategy, time_strategy, drift_strategy
           );
       }
       
   private:
       std::unique_ptr<clock_sync_strategy> _clock_strategy;
       std::unique_ptr<time_sync_strategy> _time_strategy;
       std::unique_ptr<drift_compensation_strategy> _drift_strategy;
   };
   ```

#### Deliverables:
- All basic strategies implemented
- Integration with bonded_usrp
- Basic configuration examples working

---

### Phase 3: Advanced Drift Compensation (Weeks 8-11)

#### Tasks:
1. **Implement pilot tone drift strategy**
   - Tone detection and tracking
   - Frequency offset calculation
   - Real-time compensation

2. **Implement cross-correlation drift strategy**
   - FFT-based correlation
   - Phase slope calculation
   - Continuous tracking

3. **Adaptive strategy selection**
   ```cpp
   class adaptive_drift_strategy : public drift_compensation_strategy {
   public:
       void initialize(const std::vector<multi_usrp::sptr>& devices) override {
           // Try pilot tone first
           if (detect_pilot_tone()) {
               _active_strategy = std::make_unique<pilot_tone_drift_strategy>();
           }
           // Fall back to cross-correlation
           else if (has_wideband_signal()) {
               _active_strategy = std::make_unique<cross_corr_drift_strategy>();
           }
           // Fall back to GPS timestamp
           else {
               _active_strategy = std::make_unique<gps_timestamp_drift_strategy>();
           }
       }
       
   private:
       std::unique_ptr<drift_compensation_strategy> _active_strategy;
   };
   ```

4. **Performance optimization**
   - SIMD acceleration for compensation
   - Multi-threading for parallel processing
   - Buffer management optimization

#### Deliverables:
- Advanced drift compensation strategies
- Adaptive mode selection
- Performance benchmarks

---

### Phase 4: Auto-Detection & Fallback (Weeks 12-14)

#### Tasks:
1. **Hardware capability detection**
   ```cpp
   struct device_capabilities {
       bool has_external_ref_input;
       bool has_pps_input;
       bool has_gpsdo;
       bool ref_locked_sensor_available;
       std::vector<std::string> available_clock_sources;
       std::vector<std::string> available_time_sources;
   };
   
   device_capabilities detect_capabilities(multi_usrp::sptr device);
   ```

2. **Automatic mode selection**
   ```cpp
   sync_configuration auto_configure_sync(
       const std::vector<multi_usrp::sptr>& devices
   ) {
       sync_configuration config;
       
       // Detect capabilities of all devices
       std::vector<device_capabilities> caps;
       for (auto& dev : devices) {
           caps.push_back(detect_capabilities(dev));
       }
       
       // Check if all devices have external 10MHz capability
       bool all_have_ext_ref = std::all_of(caps.begin(), caps.end(),
           [](const auto& c) { return c.has_external_ref_input; });
       
       // Check if all devices have PPS capability
       bool all_have_pps = std::all_of(caps.begin(), caps.end(),
           [](const auto& c) { return c.has_pps_input; });
       
       // Select best available configuration
       if (all_have_ext_ref) {
           config.clock_mode = clock_source_mode::EXTERNAL_10MHZ;
           config.drift_mode = drift_compensation_mode::FORCE_DISABLE;
       } else {
           config.clock_mode = clock_source_mode::INTERNAL_TCXO;
           config.drift_mode = drift_compensation_mode::AUTO;
       }
       
       if (all_have_pps) {
           config.time_mode = time_source_mode::EXTERNAL_PPS;
       } else {
           config.time_mode = time_source_mode::INTERNAL;
       }
       
       return config;
   }
   ```

3. **Graceful fallback system**
   ```cpp
   void apply_sync_with_fallback(
       const sync_configuration& desired_config,
       const std::vector<multi_usrp::sptr>& devices
   ) {
       try {
           // Try desired configuration
           apply_sync_config(desired_config, devices);
       }
       catch (const uhd::runtime_error& e) {
           if (desired_config.auto_fallback) {
               UHD_LOG_WARNING("BONDED_SYNC")
                   << "Desired sync config failed: " << e.what()
                   << "\nAttempting fallback configuration...";
               
               // Determine fallback
               auto fallback_config = compute_fallback(desired_config, devices);
               apply_sync_config(fallback_config, devices);
               
               UHD_LOG_INFO("BONDED_SYNC")
                   << "Using fallback configuration: "
                   << describe_config(fallback_config);
           } else {
               throw;  // Re-throw if auto-fallback disabled
           }
       }
   }
   ```

4. **Signal quality monitoring**
   - Monitor reference lock status
   - Check PPS signal presence
   - Validate drift compensation quality

#### Deliverables:
- Auto-detection system
- Fallback mechanism
- Signal quality monitoring

---

### Phase 5: Testing & Validation (Weeks 15-18)

#### Testing Matrix:

| Test Case | Clock Source | Time Source | Drift Comp | Expected Result |
|-----------|--------------|-------------|------------|-----------------|
| Test 1 | External | External | None | Optimal performance |
| Test 2 | Internal | External | Auto | PPS-only mode |
| Test 3 | GPSDO | GPSDO | None | GPS-disciplined |
| Test 4 | Auto | Auto | Auto | Detect and configure |
| Test 5 | Per-device | Per-device | Auto | Mixed mode |
| Test 6 | External (fail) | External | Auto | Fallback to internal |

#### Test Implementation:

```cpp
TEST(FlexibleSync, AutoDetectMode) {
    device_addr_t args;
    args["bonded"] = "true";
    args["sync_clock_source"] = "auto";
    args["sync_time_source"] = "auto";
    args["serial0"] = "ABC123";
    args["serial1"] = "DEF456";
    
    auto usrp = bonded_usrp::make(args);
    
    // Should successfully detect and configure
    EXPECT_NO_THROW(usrp->get_time_now());
}

TEST(FlexibleSync, GracefulFallback) {
    device_addr_t args;
    args["bonded"] = "true";
    args["sync_clock_source"] = "external";  // Request external
    args["sync_time_source"] = "external";
    args["serial0"] = "ABC123";
    args["serial1"] = "DEF456";
    
    // Simulate no external reference connected
    // Should fall back to internal + PPS
    auto usrp = bonded_usrp::make(args);
    
    // Should still work (with warning)
    EXPECT_NO_THROW(usrp->get_time_now());
}

TEST(FlexibleSync, PerDeviceConfiguration) {
    device_addr_t args;
    args["bonded"] = "true";
    args["clock_source0"] = "external";
    args["clock_source1"] = "internal";
    args["time_source0"] = "external";
    args["time_source1"] = "external";
    args["serial0"] = "ABC123";
    args["serial1"] = "DEF456";
    
    auto usrp = bonded_usrp::make(args);
    
    // Mixed mode should work with drift compensation
    EXPECT_TRUE(usrp->is_drift_compensation_enabled());
}
```

#### Deliverables:
- Comprehensive test suite
- Configuration validation tests
- Fallback mechanism tests
- Performance comparison across modes

---

## 5. Synchronization Modes

### 5.1 Supported Mode Combinations

| Mode Name | Clock Source | Time Source | Drift Comp | Use Case |
|-----------|--------------|-------------|------------|----------|
| **Optimal** | External 10MHz | External PPS | None | Best performance, coherent MIMO |
| **PPS-Only** | Internal TCXO | External PPS | Required | Budget-friendly, wideband monitoring |
| **GPSDO** | GPSDO | GPSDO | None | Standalone with GPS, long-term stability |
| **Internal** | Internal TCXO | Internal | Required | Testing, non-time-critical |
| **Mixed** | Per-device | Per-device | As needed | Special use cases |
| **Auto** | Detected | Detected | As needed | User doesn't know/care |

### 5.2 Mode Capabilities Comparison

| Capability | Optimal | PPS-Only | GPSDO | Internal |
|------------|---------|----------|-------|----------|
| Time Sync Accuracy | <0.1 µs | <10 µs | <0.1 µs | Poor |
| Freq Accuracy | <0.01 ppm | 2-5 ppm | <0.001 ppm | 2-5 ppm |
| Phase Coherence | Excellent | Poor | Excellent | Poor |
| Setup Cost | High ($500+) | Low ($50) | Medium ($300) | None |
| Hardware Req | 10MHz+PPS dist | PPS only | GPSDO per dev | None |
| Software Complexity | Low | High | Low | High |
| Recommended For | Coherent apps | Wideband mon | GPS-based | Testing |

### 5.3 Mode Selection Algorithm

```cpp
sync_configuration select_optimal_mode(
    const std::vector<multi_usrp::sptr>& devices,
    const application_requirements& app_reqs
) {
    sync_configuration config;
    
    // Detect available hardware
    auto caps = detect_all_capabilities(devices);
    
    // Priority 1: If GPSDO available on all devices, use it
    if (all_have_gpsdo(caps)) {
        config.clock_mode = clock_source_mode::GPSDO;
        config.time_mode = time_source_mode::GPSDO_PPS;
        config.drift_mode = drift_compensation_mode::FORCE_DISABLE;
        return config;
    }
    
    // Priority 2: If external 10MHz + PPS available, use it
    if (all_have_external_ref(caps) && all_have_pps(caps)) {
        config.clock_mode = clock_source_mode::EXTERNAL_10MHZ;
        config.time_mode = time_source_mode::EXTERNAL_PPS;
        config.drift_mode = drift_compensation_mode::FORCE_DISABLE;
        return config;
    }
    
    // Priority 3: If PPS available, use PPS-only
    if (all_have_pps(caps)) {
        config.clock_mode = clock_source_mode::INTERNAL_TCXO;
        config.time_mode = time_source_mode::EXTERNAL_PPS;
        config.drift_mode = drift_compensation_mode::AUTO;
        return config;
    }
    
    // Priority 4: Fall back to internal (with warning)
    UHD_LOG_WARNING("BONDED_SYNC")
        << "No external synchronization available. "
        << "Using internal mode (time sync will be poor).";
    
    config.clock_mode = clock_source_mode::INTERNAL_TCXO;
    config.time_mode = time_source_mode::INTERNAL;
    config.drift_mode = drift_compensation_mode::FORCE_ENABLE;
    
    return config;
}
```

---

## 6. Auto-Detection & Fallback

### 6.1 Detection Procedure

```cpp
device_capabilities detect_capabilities(multi_usrp::sptr device) {
    device_capabilities caps;
    
    // 1. Check available clock sources
    try {
        caps.available_clock_sources = device->get_clock_sources(0);
        caps.has_external_ref_input = 
            std::find(caps.available_clock_sources.begin(),
                     caps.available_clock_sources.end(),
                     "external") != caps.available_clock_sources.end();
        caps.has_gpsdo = 
            std::find(caps.available_clock_sources.begin(),
                     caps.available_clock_sources.end(),
                     "gpsdo") != caps.available_clock_sources.end();
    } catch (...) {
        caps.available_clock_sources = {"internal"};
    }
    
    // 2. Check available time sources
    try {
        caps.available_time_sources = device->get_time_sources(0);
        caps.has_pps_input = 
            std::find(caps.available_time_sources.begin(),
                     caps.available_time_sources.end(),
                     "external") != caps.available_time_sources.end();
    } catch (...) {
        caps.available_time_sources = {"none"};
    }
    
    // 3. Check for ref_locked sensor
    try {
        auto sensors = device->get_mboard_sensor_names(0);
        caps.ref_locked_sensor_available = 
            std::find(sensors.begin(), sensors.end(), "ref_locked") 
            != sensors.end();
    } catch (...) {
        caps.ref_locked_sensor_available = false;
    }
    
    return caps;
}
```

### 6.2 Verification and Lock Detection

```cpp
bool verify_clock_lock(
    multi_usrp::sptr device,
    const std::string& clock_source,
    double timeout_seconds = 5.0
) {
    // If using internal clock, always "locked"
    if (clock_source == "internal") {
        return true;
    }
    
    // Check if device has ref_locked sensor
    auto caps = detect_capabilities(device);
    if (!caps.ref_locked_sensor_available) {
        UHD_LOG_WARNING("BONDED_SYNC")
            << "Device doesn't have ref_locked sensor. "
            << "Assuming lock is good (cannot verify).";
        return true;
    }
    
    // Wait for lock with timeout
    auto start_time = std::chrono::steady_clock::now();
    while (true) {
        try {
            auto sensor = device->get_mboard_sensor("ref_locked", 0);
            if (sensor.to_bool()) {
                return true;  // Locked!
            }
        } catch (...) {
            // Sensor read failed, might not be available
            return false;
        }
        
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration<double>(elapsed).count() > timeout_seconds) {
            return false;  // Timeout
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

bool verify_pps_signal(
    multi_usrp::sptr device,
    double timeout_seconds = 5.0
) {
    // Record initial time
    auto initial_time = device->get_time_now();
    auto start_time = std::chrono::steady_clock::now();
    
    // Wait for time to increment (indicating PPS received)
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        auto current_time = device->get_time_now();
        if (current_time.get_full_secs() > initial_time.get_full_secs()) {
            return true;  // PPS detected!
        }
        
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration<double>(elapsed).count() > timeout_seconds) {
            return false;  // No PPS detected
        }
    }
}
```

### 6.3 Fallback Strategy

```cpp
sync_configuration compute_fallback(
    const sync_configuration& failed_config,
    const std::vector<multi_usrp::sptr>& devices
) {
    sync_configuration fallback = failed_config;
    
    // If external clock failed, try internal
    if (failed_config.clock_mode == clock_source_mode::EXTERNAL_10MHZ) {
        UHD_LOG_WARNING("BONDED_SYNC")
            << "External 10 MHz reference not available. "
            << "Falling back to internal TCXO (will enable drift compensation).";
        
        fallback.clock_mode = clock_source_mode::INTERNAL_TCXO;
        fallback.drift_mode = drift_compensation_mode::AUTO;
    }
    
    // If external PPS failed, try internal time
    if (failed_config.time_mode == time_source_mode::EXTERNAL_PPS) {
        UHD_LOG_WARNING("BONDED_SYNC")
            << "External PPS signal not available. "
            << "Falling back to internal time (time sync will be poor).";
        
        fallback.time_mode = time_source_mode::INTERNAL;
    }
    
    // If GPSDO failed, try external then internal
    if (failed_config.clock_mode == clock_source_mode::GPSDO) {
        // First try external reference
        auto caps = detect_all_capabilities(devices);
        if (all_have_external_ref(caps)) {
            fallback.clock_mode = clock_source_mode::EXTERNAL_10MHZ;
        } else {
            fallback.clock_mode = clock_source_mode::INTERNAL_TCXO;
            fallback.drift_mode = drift_compensation_mode::AUTO;
        }
    }
    
    return fallback;
}
```

---

## 7. API Design

### 7.1 User-Facing API

```cpp
// Simple default usage (PPS + external clock)
auto usrp = multi_usrp::make("bonded=true,serial0=ABC,serial1=DEF");

// Explicit configuration
device_addr_t args;
args["bonded"] = "true";
args["sync_clock_source"] = "external";  // or "internal", "gpsdo", "auto"
args["sync_time_source"] = "external";   // or "gpsdo", "internal", "auto"
args["drift_compensation"] = "auto";     // or "enable", "disable", "pilot_tone"
args["serial0"] = "ABC123";
args["serial1"] = "DEF456";
auto usrp = multi_usrp::make(args);

// Query current configuration
std::string clock_mode = usrp->get_sync_clock_mode();
std::string time_mode = usrp->get_sync_time_mode();
bool drift_enabled = usrp->is_drift_compensation_enabled();

// Get synchronization status
auto sync_status = usrp->get_sync_status();
std::cout << "Clock locked: " << sync_status.clock_locked << std::endl;
std::cout << "Time synced: " << sync_status.time_synced << std::endl;
std::cout << "Drift estimate: " << sync_status.drift_ppm << " ppm" << std::endl;
```

### 7.2 Extended API for Advanced Users

```cpp
namespace uhd { namespace usrp {

class bonded_usrp : public multi_usrp {
public:
    // Synchronization configuration queries
    struct sync_status {
        bool clock_locked;
        bool time_synced;
        double max_time_difference_us;
        double drift_estimate_ppm;
        std::string clock_strategy;
        std::string time_strategy;
        std::string drift_strategy;
        std::vector<std::string> warnings;
    };
    
    virtual sync_status get_sync_status() const = 0;
    
    virtual std::string get_sync_clock_mode() const = 0;
    virtual std::string get_sync_time_mode() const = 0;
    virtual bool is_drift_compensation_enabled() const = 0;
    
    // Manual recalibration
    virtual void recalibrate_drift() = 0;
    virtual void force_time_resync() = 0;
    
    // Per-device configuration access
    virtual std::string get_device_clock_source(size_t device_idx) const = 0;
    virtual std::string get_device_time_source(size_t device_idx) const = 0;
    
    // Override synchronization strategies at runtime (advanced)
    virtual void set_clock_strategy(
        std::unique_ptr<clock_sync_strategy> strategy
    ) = 0;
    
    virtual void set_drift_compensation_method(
        const std::string& method  // "pilot_tone", "cross_corr", "gps_timestamp"
    ) = 0;
};

}} // namespace uhd::usrp
```

### 7.3 Configuration Validation API

```cpp
// Standalone validation function (doesn't require device creation)
validation_result validate_bonding_config(
    const device_addr_t& config
) {
    validation_result result;
    result.is_valid = true;
    
    // Parse configuration
    auto sync_config = parse_sync_config(config);
    
    // Check for logical inconsistencies
    if (sync_config.clock_mode == clock_source_mode::EXTERNAL_10MHZ &&
        sync_config.drift_mode == drift_compensation_mode::FORCE_ENABLE) {
        result.warnings.push_back(
            "Drift compensation is enabled but external clock is locked. "
            "This wastes CPU cycles. Consider using drift_compensation=auto."
        );
    }
    
    if (sync_config.clock_mode == clock_source_mode::INTERNAL_TCXO &&
        sync_config.drift_mode == drift_compensation_mode::FORCE_DISABLE) {
        result.errors.push_back(
            "Cannot disable drift compensation when using internal clocks. "
            "Devices will drift apart in frequency."
        );
        result.is_valid = false;
    }
    
    // Provide recommendations
    if (sync_config.clock_mode == clock_source_mode::INTERNAL_TCXO &&
        sync_config.time_mode == time_source_mode::EXTERNAL_PPS) {
        result.recommended_config = 
            "PPS-only mode detected. For best performance, consider adding "
            "external 10 MHz reference (set sync_clock_source=external).";
    }
    
    return result;
}

// Usage:
device_addr_t args;
args["bonded"] = "true";
args["sync_clock_source"] = "internal";
args["drift_compensation"] = "disable";  // ERROR!

auto validation = validate_bonding_config(args);
if (!validation.is_valid) {
    std::cerr << "Configuration errors:" << std::endl;
    for (const auto& error : validation.errors) {
        std::cerr << "  - " << error << std::endl;
    }
}
```

---

## 8. Testing & Validation

### 8.1 Configuration Test Matrix

```cpp
// Automated test suite for all configuration combinations

struct test_config {
    std::string clock_source;
    std::string time_source;
    std::string drift_comp;
    bool should_succeed;
    std::string expected_warning;
};

const std::vector<test_config> test_matrix = {
    // Optimal configurations
    {"external", "external", "auto", true, ""},
    {"external", "external", "disable", true, ""},
    {"gpsdo", "gpsdo", "auto", true, ""},
    
    // PPS-only configurations
    {"internal", "external", "auto", true, ""},
    {"internal", "external", "enable", true, ""},
    
    // Invalid configurations
    {"internal", "external", "disable", false, "Cannot disable drift comp"},
    {"external", "internal", "auto", true, "Suboptimal: no time sync"},
    
    // Auto-detect
    {"auto", "auto", "auto", true, ""},
    
    // Per-device configurations
    {"per_device", "per_device", "auto", true, ""},
};

TEST(FlexibleSync, ConfigurationMatrix) {
    for (const auto& test : test_matrix) {
        device_addr_t args;
        args["bonded"] = "true";
        args["sync_clock_source"] = test.clock_source;
        args["sync_time_source"] = test.time_source;
        args["drift_compensation"] = test.drift_comp;
        args["serial0"] = "ABC123";
        args["serial1"] = "DEF456";
        
        if (test.should_succeed) {
            EXPECT_NO_THROW({
                auto usrp = bonded_usrp::make(args);
            }) << "Config should succeed: " 
               << test.clock_source << "/" << test.time_source;
        } else {
            EXPECT_THROW({
                auto usrp = bonded_usrp::make(args);
            }, uhd::runtime_error) << "Config should fail: "
               << test.clock_source << "/" << test.time_source;
        }
    }
}
```

### 8.2 Runtime Mode Switching Test

```cpp
TEST(FlexibleSync, RuntimeModeSwitch) {
    // Start with default config
    auto usrp = multi_usrp::make("bonded=true,serial0=ABC,serial1=DEF");
    
    // Verify default mode (PPS-only)
    EXPECT_EQ(usrp->get_sync_clock_mode(), "internal");
    EXPECT_EQ(usrp->get_sync_time_mode(), "external");
    EXPECT_TRUE(usrp->is_drift_compensation_enabled());
    
    // Check if external reference available
    if (usrp->can_use_external_ref()) {
        // Upgrade to external reference mode
        usrp->set_sync_mode("external", "external");
        
        // Verify upgrade
        EXPECT_EQ(usrp->get_sync_clock_mode(), "external");
        EXPECT_FALSE(usrp->is_drift_compensation_enabled());
    }
}
```

### 8.3 Fallback Mechanism Test

```cpp
TEST(FlexibleSync, GracefulFallback) {
    device_addr_t args;
    args["bonded"] = "true";
    args["sync_clock_source"] = "external";  // Request external
    args["sync_time_source"] = "external";
    args["auto_fallback"] = "true";  // Enable fallback
    args["serial0"] = "ABC123";
    args["serial1"] = "DEF456";
    
    // Mock: Simulate external reference not available
    mock_no_external_reference();
    
    // Should fall back to internal + PPS
    auto usrp = bonded_usrp::make(args);
    
    // Verify fallback occurred
    EXPECT_EQ(usrp->get_sync_clock_mode(), "internal");
    EXPECT_EQ(usrp->get_sync_time_mode(), "external");
    
    // Should have warning in status
    auto status = usrp->get_sync_status();
    EXPECT_FALSE(status.warnings.empty());
    EXPECT_THAT(status.warnings[0], HasSubstr("Falling back"));
}
```

---

## 9. User Guide

### 9.1 Quick Start

**Scenario 1: "Just make it work" (Default)**

```cpp
// Simplest configuration - uses defaults
// Default: PPS sync with internal clock
device_addr_t args;
args["bonded"] = "true";
args["serial0"] = "31D508B";  // Your B210 serial numbers
args["serial1"] = "31D50C4";

auto usrp = multi_usrp::make(args);

// Requirements:
// - PPS signal connected to both devices
// - That's it!
```

**Scenario 2: "I have 10MHz + PPS" (Optimal)**

```cpp
// Best performance configuration
device_addr_t args;
args["bonded"] = "true";
args["sync_clock_source"] = "external";
args["sync_time_source"] = "external";
args["serial0"] = "31D508B";
args["serial1"] = "31D50C4";

auto usrp = multi_usrp::make(args);

// Requirements:
// - 10 MHz reference connected to both devices
// - PPS signal connected to both devices
```

**Scenario 3: "Let the software decide" (Auto)**

```cpp
// Automatic detection and configuration
device_addr_t args;
args["bonded"] = "true";
args["sync_mode"] = "auto";  // Detect best available
args["serial0"] = "31D508B";
args["serial1"] = "31D50C4";

auto usrp = multi_usrp::make(args);

// System will:
// 1. Check for GPSDO → use if available
// 2. Check for 10MHz+PPS → use if available
// 3. Check for PPS → use with drift compensation
// 4. Fall back to internal (with warning)
```

### 9.2 Configuration Decision Tree

```
START: I want to bond USRPs
    |
    ├─ Do you have 10 MHz reference + PPS?
    |   YES → Use sync_clock_source=external, sync_time_source=external
    |   |     ↓ Best performance (0.01 ppm, hardware locked)
    |   |
    |   NO → Do you have PPS signal?
    |       YES → Use sync_clock_source=internal, sync_time_source=external
    |       |     ↓ Good performance (2-5 ppm, drift compensation)
    |       |
    |       NO → Use sync_mode=internal
    |             ↓ Poor time sync (testing only)
    |
    ├─ Do you have GPSDO in devices?
    |   YES → Use sync_clock_source=gpsdo, sync_time_source=gpsdo
    |   |     ↓ Excellent standalone performance
    |
    ├─ Not sure what you have?
        → Use sync_mode=auto
          ↓ System detects and configures automatically
```

### 9.3 Common Configuration Patterns

```cpp
// Pattern 1: Budget setup (PPS-only, GPS module)
device_addr_t minimal;
minimal["bonded"] = "true";
minimal["serial0"] = "ABC";
minimal["serial1"] = "DEF";
// Defaults to PPS + internal clock
// Cost: ~$2800 total ($2600 USRPs + $50 GPS + $150 cables)

// Pattern 2: Professional setup (10MHz+PPS from GPSDO)
device_addr_t professional;
professional["bonded"] = "true";
professional["sync_clock_source"] = "external";
professional["sync_time_source"] = "external";
professional["serial0"] = "ABC";
professional["serial1"] = "DEF";
// Cost: ~$3800 total (+$1000 for GPSDO + distribution)

// Pattern 3: Standalone GPS (each device has GPSDO)
device_addr_t standalone;
standalone["bonded"] = "true";
standalone["sync_clock_source"] = "gpsdo";
standalone["sync_time_source"] = "gpsdo";
standalone["serial0"] = "ABC";
standalone["serial1"] = "DEF";
// Cost: ~$3800 total (+$1200 for 2x GPSDO modules)

// Pattern 4: Mixed mode (one device is reference)
device_addr_t mixed;
mixed["bonded"] = "true";
mixed["clock_source0"] = "external";   // Device 0 locked to 10MHz
mixed["time_source0"] = "external";    // Device 0 uses PPS
mixed["clock_source1"] = "internal";   // Device 1 uses TCXO
mixed["time_source1"] = "external";    // Device 1 uses PPS
mixed["serial0"] = "ABC";
mixed["serial1"] = "DEF";
// Device 0 acts as reference, Device 1 compensated
// Useful when only one external reference available
```

### 9.4 Troubleshooting Guide

**Problem: Configuration fails with "Invalid sync configuration"**

Solution:
```cpp
// Enable detailed error messages
args["sync_warnings"] = "true";

// Check validation before creating device
auto validation = validate_bonding_config(args);
if (!validation.is_valid) {
    for (const auto& error : validation.errors) {
        std::cerr << "Error: " << error << std::endl;
    }
}
```

**Problem: "External reference not locked"**

Solutions:
1. Check 10 MHz cable connections
2. Verify reference signal level (+7 dBm nominal for B210)
3. Check reference frequency (must be 10 MHz for B210)
4. Enable auto-fallback:
   ```cpp
   args["auto_fallback"] = "true";
   // Will fall back to internal clock if external fails
   ```

**Problem: "PPS signal not detected"**

Solutions:
1. Check PPS cable connections
2. Verify PPS signal voltage (3.3V CMOS/TTL for B210)
3. Check PPS pulse width (>100 µs)
4. Test PPS with single device first
5. Use internal time as fallback:
   ```cpp
   args["sync_time_source"] = "auto";
   // Will detect PPS or use internal
   ```

**Problem: Poor performance with drift compensation**

Solutions:
1. Upgrade to external 10 MHz reference (best solution)
2. Increase drift update interval:
   ```cpp
   args["drift_update_interval"] = "5.0";  // More frequent updates
   ```
3. Use pilot tone method:
   ```cpp
   args["drift_compensation"] = "pilot_tone";
   // Requires transmitting test tone
   ```
4. Check for temperature stability
5. Verify drift estimate quality:
   ```cpp
   auto status = usrp->get_sync_status();
   std::cout << "Drift: " << status.drift_estimate_ppm << " ppm" << std::endl;
   // Should be < 5 ppm for good performance
   ```

### 9.5 Performance Optimization Tips

1. **Use external 10 MHz reference when available**
   - Eliminates drift compensation overhead
   - Best phase noise and frequency accuracy
   - Worth the extra cost for critical applications

2. **Match cable lengths**
   - Keep PPS cable lengths equal (within 1 meter)
   - Reduces timing skew between devices
   - Important for sub-microsecond time sync

3. **Thermal management**
   - Keep devices at similar temperatures
   - Use fans if operating in warm environment
   - Reduces thermal drift in PPS-only mode

4. **Choose appropriate drift compensation method**
   - Pilot tone: Best accuracy, requires external signal
   - Cross-correlation: No external signal, moderate accuracy
   - GPS timestamp: Simplest, lowest accuracy

5. **Monitor sync status**
   ```cpp
   // Periodically check synchronization health
   auto status = usrp->get_sync_status();
   if (!status.clock_locked || !status.time_synced) {
       UHD_LOG_WARNING("APP") << "Sync degraded, recalibrating...";
       usrp->recalibrate_drift();
   }
   ```

---

## 10. Implementation Timeline

### Recommended Development Sequence:

| Weeks | Phase | Key Deliverables |
|-------|-------|------------------|
| 1-3 | Core Abstraction | Strategy interfaces, configuration parser |
| 4-7 | Basic Strategies | External, internal, GPSDO modes working |
| 8-11 | Drift Compensation | Advanced compensation algorithms |
| 12-14 | Auto-Detection | Hardware detection, fallback system |
| 15-18 | Testing | Comprehensive test suite, validation |
| 19-20 | Documentation | User guide, API docs, examples |
| 21-22 | Polish | Performance optimization, bug fixes |

**Total: 22 weeks (~5.5 months)**

---

## 11. Advantages of Flexible Approach

### 11.1 For Users

✅ **No Wrong Choices**: System guides users to optimal configuration
✅ **Upgrade Path**: Start with PPS-only, add 10MHz later without code changes
✅ **Works Out of Box**: Sensible defaults (PPS + internal)
✅ **Clear Feedback**: System explains what it's doing and why
✅ **Graceful Degradation**: Falls back to available hardware
✅ **Cost Flexibility**: Users choose investment level

### 11.2 For Developers

✅ **Single Codebase**: One implementation, multiple modes
✅ **Extensible**: Easy to add new sync strategies
✅ **Testable**: Each strategy can be unit tested
✅ **Maintainable**: Clear separation of concerns
✅ **Future-Proof**: Can add network sync, PTP, etc.

### 11.3 For Research

✅ **Comparative Studies**: Easy to compare sync modes
✅ **Algorithm Development**: New strategies plug in easily
✅ **Reproducibility**: Configuration fully specified in args
✅ **Publication**: Can show multiple approaches in paper

---

## 12. Conclusion

This flexible implementation plan provides:

1. **Maximum Flexibility**: Supports all sync modes in one codebase
2. **User-Friendly**: Smart defaults, auto-detection, helpful errors
3. **Production-Ready**: Robust fallback and validation
4. **Research-Friendly**: Easy to experiment with different modes
5. **Cost-Effective**: Users choose their budget/performance point
6. **Future-Proof**: Extensible architecture for new sync methods

### Success Criteria

✅ Works with default args (no sync knowledge required)
✅ Supports all documented sync modes
✅ Auto-detects available hardware
✅ Falls back gracefully when hardware unavailable
✅ Provides clear error messages and warnings
✅ Performance comparable to fixed-mode implementations
✅ Well-documented with examples for each mode

### Next Steps

1. Review and approve this flexible architecture
2. Begin Phase 1 implementation (abstraction layer)
3. Implement basic strategies (external, internal)
4. Add drift compensation
5. Implement auto-detection
6. Test and document

This approach gives you the **best of both worlds**: the simplicity of PPS-only for budget users and the performance of 10MHz+PPS for demanding applications, all in a single unified implementation that's configurable at runtime!

---

**Document Version**: 1.0  
**Last Updated**: January 27, 2026  
**Author**: Flexible Implementation Plan  
**Status**: Ready for Implementation  
**Related Documents**: 
- BONDING_IMPLEMENTATION_PLAN.md (10MHz+PPS reference)
- BONDING_IMPLEMENTATION_PLAN_PPS_ONLY.md (PPS-only reference)
