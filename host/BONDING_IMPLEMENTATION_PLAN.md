# USRP Bonding Implementation Plan

## Driver-Level Multi-Device Bandwidth Aggregation for USB-Based USRPs

**Author**: Gabriel Lando  
**Context**: Master's Degree Research — Wideband Spectrum Monitoring via USRP Bonding  
**Target Hardware**: Ettus B210 (USB 3.0) — Serials `30B56D6` and `30DBC3C`  
**Codebase**: UHD (USRP Hardware Driver) — extending `multi_usrp` API

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Background and Motivation](#2-background-and-motivation)
3. [Architecture Overview](#3-architecture-overview)
4. [Implementation Phases](#4-implementation-phases)
   - [Phase 1: Synchronization Validation Examples](#phase-1-synchronization-validation-examples)
   - [Phase 2: Python Analysis and Visualization](#phase-2-python-analysis-and-visualization)
   - [Phase 3: Core Bonding Library](#phase-3-core-bonding-library)
   - [Phase 4: Bonded RX Streaming](#phase-4-bonded-rx-streaming)
   - [Phase 5: Bonded TX Streaming](#phase-5-bonded-tx-streaming)
   - [Phase 6: GNU Radio Integration](#phase-6-gnu-radio-integration)
   - [Phase 7: Comprehensive Evaluation](#phase-7-comprehensive-evaluation)
5. [Synchronization Modes](#5-synchronization-modes)
6. [API Design](#6-api-design)
7. [Data Formats and File Conventions](#7-data-formats-and-file-conventions)
8. [Visualization and Academic Figures](#8-visualization-and-academic-figures)
9. [How to Run](#9-how-to-run)
10. [References](#10-references)

---

## 1. Executive Summary

### Objective

Extend the UHD `multi_usrp` driver to support **bonding of N USB-based USRP devices** (B200/B210), aggregating their bandwidth into a single logical wideband receiver or transmitter. The implementation supports configurable clock and time synchronization modes — `internal`, `external` (10 MHz + PPS), and `gpsdo` — selected at runtime via standard UHD device arguments.

### Research Goal

This work forms the core contribution of a master's degree thesis on wideband spectrum monitoring. The objective is to demonstrate that multiple low-cost USB SDRs can be coherently bonded to achieve instantaneous bandwidth beyond the capability of a single device, enabling applications such as:

- Wideband spectrum surveillance (e.g., ISM band monitoring across 80+ MHz)
- Multi-channel parallel reception for cognitive radio research
- Distributed sensing with time-aligned captures

### Design Principles

1. **Extend, don't fork**: Build on the existing `multi_usrp` API. All configuration uses standard UHD `device_addr_t` parameters.
2. **N-device support**: The architecture supports 2, 3, or more devices. No hardcoded assumptions about device count.
3. **Standard parameter interface**: Clock source, time source, gain, frequency, sample rate, and antenna are set exactly as they are for a single USRP — the bonding layer distributes them across devices.
4. **RX and TX support**: Both receive (wideband capture) and transmit (wideband playback) paths are implemented.
5. **Validation-first development**: Each phase begins with examples and tests that produce measurable, plottable data, ensuring every integration step is verifiable.
6. **Academic rigor**: All figures are publication-quality (LaTeX-compatible labels, proper axes, SI units). Each figure includes a descriptive caption suitable for a thesis chapter.

### Device Addressing

USB-based USRPs are addressed by serial number. To bond N devices:

```
# Two devices (the primary use case for this thesis)
--args="serial0=30B56D6,serial1=30DBC3C"

# Three devices
--args="serial0=30B56D6,serial1=30DBC3C,serial2=XXXXXXX"

# N devices
--args="serial0=S0,serial1=S1,...,serialN-1=SN-1"
```

Clock and time sources are configured identically to single-device usage:

```
# External 10 MHz reference + PPS
--args="serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external"

# GPSDO (requires GPSDO hardware on each device)
--args="serial0=30B56D6,serial1=30DBC3C,clock_source=gpsdo,time_source=gpsdo"

# Internal oscillators (free-running, requires drift compensation)
--args="serial0=30B56D6,serial1=30DBC3C,clock_source=internal,time_source=internal"
```

---

## 2. Background and Motivation

### 2.1 The Bandwidth Limitation of Single USB SDRs

The Ettus B210 provides up to 56 MHz of instantaneous bandwidth over USB 3.0. While sufficient for many applications, wideband spectrum monitoring scenarios (e.g., surveying the entire 902–928 MHz ISM band, or monitoring 100+ MHz of LTE spectrum) require instantaneous bandwidth exceeding a single device's capability.

### 2.2 Multi-Device Bonding as a Solution

By operating multiple B210 devices in parallel — each tuned to an adjacent frequency segment — and stitching their outputs in software, the effective bandwidth scales linearly with the number of devices. The critical challenge is **time and frequency synchronization**: all devices must sample coherently so that the stitched spectrum is continuous and artifact-free.

### 2.3 Synchronization Challenges for USB Devices

Unlike networked USRPs (X310, N320) that share a common 10 MHz/PPS backplane, USB devices are physically independent. Synchronization must be achieved through:

- **External PPS**: A shared pulse-per-second signal aligns sample timing across devices.
- **External 10 MHz**: A shared reference clock locks all local oscillators to the same frequency standard, eliminating relative frequency drift.
- **GPSDO**: Each device has a GPS-disciplined oscillator providing both time and frequency reference.
- **Internal (free-running)**: No external references; software-based drift estimation and compensation are required.

### 2.4 Relationship to the UHD multi_usrp API

The UHD `multi_usrp` class already supports multi-device operation via indexed device addresses (`addr0`, `addr1`, or `serial0`, `serial1`). It provides:

- `set_clock_source(source, mboard)` / `set_time_source(source, mboard)`
- `set_time_unknown_pps(time_spec)` — synchronizes time across all boards using PPS
- `set_time_next_pps(time_spec)` — sets time at the next PPS edge
- `get_rx_stream(stream_args)` / `get_tx_stream(stream_args)` — with multi-channel support
- `set_rx_rate()`, `set_rx_freq()`, `set_rx_gain()` — applied per-channel or globally

This project extends `multi_usrp` by adding a **bonding coordination layer** that:

1. Configures each device to a different center frequency (with overlap for stitching)
2. Manages synchronized streaming start/stop
3. Provides spectrum stitching and drift compensation utilities

---

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                User Application                      │
│  (C++ example / Python script / GNU Radio flowgraph) │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              multi_usrp::make(dev_addr)              │
│         (standard UHD device instantiation)          │
│  serial0=30B56D6, serial1=30DBC3C,                   │
│  clock_source=external, time_source=external         │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│            Bonding Coordination Layer                │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────┐  │
│  │ Sync Manager │ │ Freq Planner │ │ Drift Monitor│  │
│  │ (PPS/Clock)  │ │ (N segments) │ │ (ppm tracker)│  │
│  └──────┬──────┘ └──────┬───────┘ └──────┬───────┘  │
│         │               │               │           │
│  ┌──────▼───────────────▼───────────────▼────────┐  │
│  │          Bonded RX / TX Streamer               │  │
│  │  (multi-channel recv/send with alignment)      │  │
│  └──────┬────────────────────────────┬───────────┘  │
│         │                            │              │
│  ┌──────▼──────┐            ┌───────▼────────┐     │
│  │  Device 0   │            │   Device 1     │     │
│  │  (30B56D6)  │    ...     │   (30DBC3C)    │     │
│  │  B210 USB   │            │   B210 USB     │     │
│  └─────────────┘            └────────────────┘     │
└─────────────────────────────────────────────────────┘
```

### Key Components

| Component                   | Location                                       | Responsibility                                                        |
| --------------------------- | ---------------------------------------------- | --------------------------------------------------------------------- |
| **Sync Manager**            | `lib/usrp/bonded/sync_manager.{hpp,cpp}`       | Configure clock/time sources, verify lock, synchronize time registers |
| **Frequency Planner**       | `lib/usrp/bonded/freq_planner.{hpp,cpp}`       | Compute center frequencies for N devices with configurable overlap    |
| **Drift Monitor**           | `lib/usrp/bonded/drift_monitor.{hpp,cpp}`      | Estimate inter-device clock drift in ppm via PPS timestamp comparison |
| **Bonded RX Streamer**      | `lib/usrp/bonded/bonded_rx_streamer.{hpp,cpp}` | Coordinate multi-device recv(), align timestamps, stitch spectra      |
| **Bonded TX Streamer**      | `lib/usrp/bonded/bonded_tx_streamer.{hpp,cpp}` | Coordinate multi-device send(), distribute wideband TX across devices |
| **Spectrum Stitcher**       | `lib/usrp/bonded/spectrum_stitcher.{hpp,cpp}`  | FFT, overlap-add, phase correction for wideband spectrum assembly     |
| **Bonding Python Bindings** | `lib/usrp/bonded/bonded_python.cpp`            | Pybind11 wrappers for Python analysis scripts                         |

---

## 4. Implementation Phases

### Phase 1: Synchronization Validation Examples

**Goal**: Create standalone C++ programs that verify multi-device synchronization quality and save captured data to binary files for offline analysis.

**Rationale**: Before implementing the bonding library, we must empirically verify that our hardware setup (two B210s with shared PPS and/or 10 MHz reference) achieves adequate synchronization. This phase produces the raw data that Phase 2 will visualize.

#### 1.1 Sync Validation Capture Tool

**File**: `examples/sync_validation_capture.cpp`

A command-line tool that:

1. Instantiates a `multi_usrp` with N devices via serial numbers
2. Configures clock and time sources per user arguments
3. Synchronizes device time via `set_time_unknown_pps()` or `set_time_next_pps()`
4. Verifies reference lock on all devices (`ref_locked` sensor)
5. Captures simultaneous samples from all devices to separate binary files
6. Records a JSON metadata file with all capture parameters
7. Measures inter-device PPS timestamp deltas and clock drift

**Command-line interface**:

```bash
sync_validation_capture \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --duration=1.0 \
    --output-dir=./sync_capture \
    --format=sc16
```

**Output files**:

```
sync_capture/
├── metadata.json              # Capture parameters, device info, sync metrics
├── usrp_0_30B56D6.bin         # Raw IQ samples from device 0 (sc16 format)
├── usrp_1_30DBC3C.bin         # Raw IQ samples from device 1 (sc16 format)
└── ... (additional devices)
```

**Metadata JSON schema**:

```json
{
  "capture_tool": "sync_validation_capture",
  "capture_version": "1.0",
  "capture_timestamp_utc": "2026-03-10T12:00:00Z",
  "parameters": {
    "center_frequency_hz": 915000000.0,
    "sample_rate_hz": 10000000.0,
    "gain_db": 30.0,
    "bandwidth_hz": 10000000.0,
    "duration_s": 1.0,
    "sample_format": "sc16",
    "clock_source": "external",
    "time_source": "external",
    "bytes_per_sample": 4
  },
  "devices": [
    {
      "index": 0,
      "serial": "30B56D6",
      "mboard_name": "B210",
      "file": "usrp_0_30B56D6.bin",
      "actual_frequency_hz": 915000000.0,
      "actual_rate_hz": 10000000.0,
      "actual_gain_db": 30.0,
      "first_sample_timestamp_ticks": 0,
      "first_sample_timestamp_s": 0.0,
      "total_samples": 10000000,
      "overflow_count": 0,
      "stream_errors": []
    }
  ],
  "sync_results": {
    "pps_time_deltas_us": [],
    "timestamp_alignment_ticks": [],
    "drift_check_performed": true,
    "drift_measurement_duration_s": 5.0,
    "drift_ppm": [],
    "drift_reference_device": 0
  }
}
```

**Implementation details**:

```cpp
// Pseudocode for the synchronization and capture flow
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // 1. Parse arguments (boost::program_options)
    // 2. Create multi_usrp with N devices
    auto usrp = uhd::usrp::multi_usrp::make(args);
    const size_t num_mboards = usrp->get_num_mboards();

    // 3. Configure clock and time sources on ALL motherboards
    for (size_t m = 0; m < num_mboards; m++) {
        usrp->set_clock_source(clock_source, m);
        usrp->set_time_source(time_source, m);
    }

    // 4. Wait for reference lock
    for (size_t m = 0; m < num_mboards; m++) {
        // Poll ref_locked sensor with timeout
    }

    // 5. Synchronize time
    if (time_source == "external" || time_source == "gpsdo") {
        usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }

    // 6. Configure RX parameters on all channels
    for (size_t ch = 0; ch < usrp->get_rx_num_channels(); ch++) {
        usrp->set_rx_rate(rate, ch);
        usrp->set_rx_freq(freq, ch);
        usrp->set_rx_gain(gain, ch);
    }

    // 7. Create RX streamer with all channels
    uhd::stream_args_t stream_args("sc16", "sc16");
    std::vector<size_t> channels(usrp->get_rx_num_channels());
    std::iota(channels.begin(), channels.end(), 0);
    stream_args.channels = channels;
    auto rx_stream = usrp->get_rx_stream(stream_args);

    // 8. Start synchronized streaming at a future time
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    cmd.num_samps  = total_samples;
    cmd.stream_now = false;
    cmd.time_spec  = usrp->get_time_now() + uhd::time_spec_t(1.5);
    rx_stream->issue_stream_cmd(cmd);

    // 9. Receive samples into per-channel buffers and write to files
    // 10. Measure PPS deltas and drift
    // 11. Write metadata JSON
}
```

#### 1.2 Drift Monitoring Tool

**File**: `examples/drift_monitor.cpp`

A long-running tool that periodically measures the PPS timestamp difference between devices, logging drift over time. This provides data for drift-rate plots.

```bash
drift_monitor \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=internal \
    --time-source=external \
    --duration=3600 \
    --interval=1.0 \
    --output=drift_log.csv
```

**Output CSV format**:

```csv
elapsed_s,device_0_pps_ticks,device_1_pps_ticks,delta_ticks,delta_us,drift_ppm
1.0,10000000,10000003,3,0.300,0.300
2.0,20000000,20000007,7,0.700,0.350
...
```

#### 1.3 TX Synchronization Validation Tool

**File**: `examples/sync_validation_tx.cpp`

Transmits a known signal (e.g., a tone or chirp) simultaneously from all devices to verify TX timing alignment. A separate receiver (or loopback) captures the combined output.

```bash
sync_validation_tx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --waveform=tone \
    --duration=1.0
```

#### 1.4 Build Integration

Add to `examples/CMakeLists.txt`:

```cmake
# Bonding synchronization validation examples
list(APPEND example_sources
    sync_validation_capture.cpp
    drift_monitor.cpp
    sync_validation_tx.cpp
)
```

#### Phase 1 Deliverables

| Deliverable                             | Description                                          |
| --------------------------------------- | ---------------------------------------------------- |
| `examples/sync_validation_capture.cpp`  | Multi-device synchronized RX capture to binary files |
| `examples/drift_monitor.cpp`            | Long-running PPS drift measurement tool              |
| `examples/sync_validation_tx.cpp`       | Multi-device synchronized TX validation              |
| `docs/bonding/sync_validation_howto.md` | Step-by-step instructions for running each tool      |

#### Phase 1 Documentation: `docs/bonding/sync_validation_howto.md`

This document will include:

- Hardware setup diagram (PPS/10MHz distribution wiring)
- How to build the examples (`cmake` and `make` commands)
- CLI commands for each synchronization mode (external, gpsdo, internal)
- GNU Radio Companion instructions for equivalent tests using standard USRP Source/Sink blocks
- Expected output format and how to verify success

---

### Phase 2: Python Analysis and Visualization

**Goal**: Create a Python analysis toolkit that reads the binary capture data from Phase 1 and generates publication-quality figures for the thesis.

#### 2.1 Analysis Script

**File**: `examples/python/sync_analysis.py`

A comprehensive Python script that:

1. Reads the JSON metadata file
2. Loads binary IQ data from each device
3. Produces a suite of academic-quality figures

**Usage**:

```bash
python3 examples/python/sync_analysis.py \
    --input-dir=./sync_capture \
    --output-dir=./sync_capture/plots \
    --format=png \
    --dpi=300
```

#### 2.2 Figure Specifications

Each figure below is designed for direct inclusion in a master's thesis. Figures use LaTeX-compatible fonts (serif, 10–12 pt), proper axis labels with SI units, and a consistent color scheme.

---

**Figure 1: Time-Domain Overlay**

_Two-panel time-domain comparison of IQ samples from synchronized USRP devices._

This figure presents the raw in-phase (I) component of the captured signal from both devices, overlaid on the same time axis. The upper panel shows the full capture duration at a coarse scale, while the lower panel zooms into a short segment (e.g., 10 µs) to reveal sample-level alignment. When devices are properly synchronized via an external PPS and 10 MHz reference, the waveforms should overlap nearly identically. Any visible time offset between the traces indicates residual synchronization error, which can be quantified via cross-correlation (Figure 2).

This visualization serves as the first qualitative check of synchronization quality. Large offsets (visible at the coarse scale) indicate a failure in PPS synchronization, while small sub-sample offsets (visible only in the zoomed panel) indicate residual timing error that may require interpolation-based correction.

---

**Figure 2: Cross-Correlation Peak**

_Cross-correlation of the two captured signals to estimate the inter-device time offset._

The normalized cross-correlation function between the signals from Device 0 and Device 1 is computed and plotted as a function of lag in samples. The peak location indicates the relative time offset between the two devices; a peak at lag zero confirms sample-aligned synchronization. The peak's sharpness and height indicate signal quality and correlation confidence.

Cross-correlation is the standard method for estimating time-difference-of-arrival (TDOA) between two receivers and is widely used in the synchronization literature. The sub-sample accuracy can be improved by interpolating around the peak (e.g., parabolic interpolation). This measurement directly quantifies the synchronization precision achieved by the PPS alignment procedure and is a key metric for the thesis evaluation.

---

**Figure 3: Phase Difference Over Time**

_Instantaneous phase difference between the two received signals as a function of time._

The instantaneous phase of each complex IQ signal is computed via `atan2(Q, I)`, and the phase difference (Device 1 − Device 0) is plotted over the capture duration. For a shared frequency reference (external 10 MHz), the phase difference should remain approximately constant, indicating coherent LO operation. A linearly increasing phase difference indicates a frequency offset (drift) between the two local oscillators, with the slope proportional to the drift in Hz.

This figure is critical for evaluating the coherence of the two devices. In the external clock mode, a flat phase difference confirms that both LOs are locked to the same reference. In the internal clock mode, the slope of the phase difference directly yields the relative frequency error, which can be compared against the nominal TCXO specification (typically ±2 ppm for the B210).

---

**Figure 4: Power Spectral Density Overlay**

_Comparison of the power spectral density (PSD) estimates from both devices._

The PSD of each device's capture is estimated using Welch's method (e.g., 4096-point FFT, Hann window, 50% overlap) and plotted in dBFS on the same axes. The frequency axis is centered on the configured center frequency. This figure verifies that both devices observe the same spectral content with comparable signal-to-noise ratios, confirming correct frequency tuning and gain calibration.

Agreement between the two PSD traces validates that the RF front-ends of both devices are configured identically and that the captured signal is consistent across both receivers. Discrepancies may indicate gain mismatch, antenna coupling differences, or environmental asymmetries. For bonding applications, PSD agreement across the overlap region between adjacent frequency segments is essential for artifact-free spectrum stitching.

---

**Figure 5: Magnitude-Squared Coherence**

_Spectral coherence between the two received signals as a function of frequency._

The magnitude-squared coherence (MSC) is computed using Welch's method and plotted across the captured bandwidth. A coherence value of 1.0 indicates perfect linear relationship between the two signals at that frequency; values significantly below 1.0 indicate uncorrelated noise or synchronization-related phase noise.

Coherence is a frequency-domain measure of the degree to which two signals are related. High coherence across the band confirms that the two receivers are sampling the same signal with consistent phase relationships. This metric is particularly important for bonding applications where adjacent spectral segments must be stitched: coherence in the overlap region between segments must be high for the stitching algorithm to produce a seamless wideband spectrum.

---

**Figure 6: Synchronization Summary Dashboard**

_Multi-panel summary of key synchronization metrics._

A compact four-panel figure summarizing: (a) time offset in samples, (b) frequency offset in Hz, (c) PPS timestamp alignment verification, and (d) estimated drift rate in ppm. Each panel shows a single scalar value with its uncertainty, presented as a bar or gauge. This figure provides an at-a-glance assessment of synchronization quality.

This dashboard-style figure is intended for the results chapter of the thesis, providing a concise visual summary that complements the detailed figures above. The combination of time-domain, frequency-domain, and drift metrics gives a comprehensive characterization of the synchronization performance under the tested configuration.

---

**Figure 7: IQ Constellation Diagram**

_Constellation plot of the received IQ samples from each device._

The complex IQ samples from each device are plotted on the complex plane (I vs. Q). For a CW tone, the constellation should form a circle; for a modulated signal, the constellation shape reflects the modulation scheme. Comparing the constellations from both devices reveals differences in gain, phase offset, and noise characteristics.

Constellation diagrams are a standard tool in communications engineering for assessing signal quality. In the context of multi-device synchronization, comparing the constellations helps identify systematic phase or amplitude offsets that must be calibrated before combining the signals from multiple devices.

---

**Figure 8: Phase Difference Histogram**

_Statistical distribution of the instantaneous phase difference between devices._

A histogram of the instantaneous phase difference values (from Figure 3) with a fitted Gaussian overlay. The mean of the distribution corresponds to the static phase offset, and the standard deviation quantifies the phase noise / jitter between the two devices.

This statistical characterization is important for quantifying the repeatability and stability of the synchronization. A narrow distribution (small standard deviation) indicates stable, low-jitter synchronization suitable for coherent combining. The mean phase offset can be calibrated out in software. This figure provides the numerical basis for the synchronization precision claims in the thesis.

---

**Figure 9: Drift Rate Over Time** _(from drift_monitor data)_

_Clock drift between devices measured over an extended period._

Using the CSV data from the `drift_monitor` tool (Phase 1.2), this figure plots the measured drift in ppm as a function of elapsed time. For external 10 MHz reference, drift should be near zero. For internal oscillators, drift may be 1–5 ppm and may exhibit temperature-dependent variations.

Long-term drift characterization is essential for understanding the operational constraints of each synchronization mode. This figure demonstrates whether drift compensation is required (internal clock mode) or unnecessary (external reference mode), directly informing the design decisions for the bonding library.

---

**Figure 10: Synchronization Mode Comparison** _(composite figure)_

_Bar chart comparing key synchronization metrics across all tested modes._

A grouped bar chart comparing time offset (µs), frequency offset (Hz), phase jitter (degrees), and drift rate (ppm) across the three synchronization modes: external 10 MHz + PPS, PPS-only (internal clock), and internal-only. Error bars show the standard deviation across multiple captures.

This is the key comparative figure for the thesis evaluation chapter. It quantitatively demonstrates the tradeoff between synchronization quality and hardware complexity for each mode, supporting the thesis argument for the recommended configuration.

---

#### 2.3 Plotting Code Standards

All Python visualization code must follow these conventions:

```python
import matplotlib
matplotlib.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 13,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'figure.figsize': (7, 4.5),
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox_inches': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.2,
})
```

- All axis labels include SI units in parentheses, e.g., "Time (µs)", "Frequency (MHz)", "Power (dBFS)"
- Color scheme: use a colorblind-friendly palette (e.g., seaborn's `colorblind`)
- All figures saved in both PNG (300 dpi) and PDF (vector) formats
- Figure filenames follow the pattern `figNN_short_description.{png,pdf}`

#### Phase 2 Deliverables

| Deliverable                             | Description                                         |
| --------------------------------------- | --------------------------------------------------- |
| `examples/python/sync_analysis.py`      | Main analysis script producing all figures          |
| `examples/python/drift_analysis.py`     | Drift log analysis and plotting                     |
| `examples/python/bonding_plot_utils.py` | Shared plotting utilities and style configuration   |
| `docs/bonding/analysis_howto.md`        | Instructions for running analysis, expected outputs |

---

### Phase 3: Core Bonding Library

**Goal**: Implement the core bonding infrastructure as a library within UHD.

#### 3.1 Sync Manager

**Files**: `lib/usrp/bonded/sync_manager.hpp`, `lib/usrp/bonded/sync_manager.cpp`

Encapsulates the synchronization procedure:

```cpp
namespace uhd { namespace usrp { namespace bonded {

class sync_manager
{
public:
    struct config {
        std::string clock_source;  // "internal", "external", "gpsdo"
        std::string time_source;   // "internal", "external", "gpsdo"
        double lock_timeout_s = 5.0;
        double sync_settle_s  = 2.0;
    };

    struct status {
        bool all_locked;
        std::vector<bool> per_device_locked;
        std::vector<double> pps_deltas_us;
        double max_pps_delta_us;
    };

    sync_manager(multi_usrp::sptr usrp, const config& cfg);

    // Configure clock and time sources on all motherboards
    void configure_sources();

    // Wait for all reference oscillators to lock
    bool wait_for_lock();

    // Synchronize time registers across all devices
    void synchronize_time();

    // Verify synchronization by comparing PPS timestamps
    status verify_sync();

private:
    multi_usrp::sptr _usrp;
    config _cfg;
    size_t _num_mboards;
};

}}} // namespace uhd::usrp::bonded
```

#### 3.2 Frequency Planner

**Files**: `lib/usrp/bonded/freq_planner.hpp`, `lib/usrp/bonded/freq_planner.cpp`

Computes center frequencies for N devices to cover a target bandwidth with configurable overlap:

```cpp
namespace uhd { namespace usrp { namespace bonded {

class freq_planner
{
public:
    struct plan {
        double total_bandwidth_hz;
        double overlap_hz;
        std::vector<double> center_frequencies_hz;
        std::vector<double> per_device_rate_hz;
    };

    // Compute frequency plan for N devices
    static plan compute(
        double start_freq_hz,
        double total_bandwidth_hz,
        size_t num_devices,
        double per_device_rate_hz,
        double overlap_fraction = 0.1  // 10% overlap by default
    );
};

}}} // namespace uhd::usrp::bonded
```

#### 3.3 Drift Monitor

**Files**: `lib/usrp/bonded/drift_monitor.hpp`, `lib/usrp/bonded/drift_monitor.cpp`

Continuous background drift estimation:

```cpp
namespace uhd { namespace usrp { namespace bonded {

class drift_monitor
{
public:
    struct measurement {
        double elapsed_s;
        std::vector<double> drift_ppm;    // Per-device relative to device 0
        std::vector<int64_t> pps_ticks;
    };

    drift_monitor(multi_usrp::sptr usrp);

    // Take a single drift measurement
    measurement measure();

    // Get the latest drift estimate for a specific device
    double get_drift_ppm(size_t device_index) const;

private:
    multi_usrp::sptr _usrp;
    int64_t _initial_ticks_0;
    std::chrono::steady_clock::time_point _start_time;
};

}}} // namespace uhd::usrp::bonded
```

#### 3.4 Build Integration

**File**: `lib/usrp/bonded/CMakeLists.txt`

```cmake
target_sources(uhd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sync_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/freq_planner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drift_monitor.cpp
)
```

Added to `lib/CMakeLists.txt`:

```cmake
add_subdirectory(usrp/bonded)
```

#### Phase 3 Deliverables

| Deliverable                                 | Description                         |
| ------------------------------------------- | ----------------------------------- |
| `include/uhd/usrp/bonded/sync_manager.hpp`  | Sync manager public API             |
| `include/uhd/usrp/bonded/freq_planner.hpp`  | Frequency planner public API        |
| `include/uhd/usrp/bonded/drift_monitor.hpp` | Drift monitor public API            |
| `lib/usrp/bonded/sync_manager.cpp`          | Sync manager implementation         |
| `lib/usrp/bonded/freq_planner.cpp`          | Frequency planner implementation    |
| `lib/usrp/bonded/drift_monitor.cpp`         | Drift monitor implementation        |
| `lib/usrp/bonded/CMakeLists.txt`            | Build configuration                 |
| `tests/bonded_sync_test.cpp`                | Unit tests for sync manager         |
| `tests/bonded_freq_planner_test.cpp`        | Unit tests for frequency planner    |
| `docs/bonding/library_howto.md`             | Library API documentation and usage |

---

### Phase 4: Bonded RX Streaming

**Goal**: Implement a bonded RX streamer that coordinates multi-device reception for wideband capture and spectrum stitching.

#### 4.1 Bonded RX Streamer

**Files**: `lib/usrp/bonded/bonded_rx_streamer.hpp`, `lib/usrp/bonded/bonded_rx_streamer.cpp`

```cpp
namespace uhd { namespace usrp { namespace bonded {

class bonded_rx_streamer
{
public:
    struct config {
        double total_bandwidth_hz;
        double overlap_fraction = 0.1;
        std::string sample_format = "fc32";  // CPU format
        std::string otw_format   = "sc16";   // Over-the-wire format
        size_t spp = 0;                      // Samples per packet (0 = auto)
    };

    bonded_rx_streamer(
        multi_usrp::sptr usrp,
        const sync_manager& sync,
        const config& cfg
    );

    // Start synchronized reception across all devices
    void start(const uhd::time_spec_t& start_time);

    // Receive aligned samples from all devices
    // Returns per-device sample buffers and metadata
    size_t recv(
        std::vector<std::complex<float>*>& buffs,
        size_t nsamps_per_device,
        uhd::rx_metadata_t& md,
        double timeout = 1.0
    );

    // Stop reception
    void stop();

    // Get the frequency plan in use
    freq_planner::plan get_freq_plan() const;

private:
    multi_usrp::sptr _usrp;
    std::vector<uhd::rx_streamer::sptr> _streams;
    freq_planner::plan _plan;
};

}}} // namespace uhd::usrp::bonded
```

#### 4.2 Spectrum Stitcher

**Files**: `lib/usrp/bonded/spectrum_stitcher.hpp`, `lib/usrp/bonded/spectrum_stitcher.cpp`

Combines per-device spectra into a single wideband spectrum:

```cpp
namespace uhd { namespace usrp { namespace bonded {

class spectrum_stitcher
{
public:
    struct config {
        size_t fft_size = 4096;
        double overlap_hz;
        std::vector<double> center_freqs_hz;
        double sample_rate_hz;
    };

    spectrum_stitcher(const config& cfg);

    // Stitch per-device time-domain samples into a wideband spectrum
    std::vector<float> stitch(
        const std::vector<std::vector<std::complex<float>>>& per_device_samples
    );

    // Get the frequency axis for the stitched spectrum
    std::vector<double> get_frequency_axis() const;

private:
    config _cfg;
    std::vector<float> _window;
};

}}} // namespace uhd::usrp::bonded
```

#### 4.3 Wideband RX Example

**File**: `examples/bonded_wideband_rx.cpp`

A complete example demonstrating bonded wideband reception:

```bash
bonded_wideband_rx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --start-freq=900e6 \
    --total-bw=100e6 \
    --rate=56e6 \
    --gain=30 \
    --duration=5.0 \
    --output-dir=./wideband_capture
```

#### Phase 4 Deliverables

| Deliverable                                      | Description                                 |
| ------------------------------------------------ | ------------------------------------------- |
| `include/uhd/usrp/bonded/bonded_rx_streamer.hpp` | Bonded RX streamer public API               |
| `include/uhd/usrp/bonded/spectrum_stitcher.hpp`  | Spectrum stitcher public API                |
| `lib/usrp/bonded/bonded_rx_streamer.cpp`         | Bonded RX streamer implementation           |
| `lib/usrp/bonded/spectrum_stitcher.cpp`          | Spectrum stitcher implementation            |
| `examples/bonded_wideband_rx.cpp`                | Wideband RX example                         |
| `examples/python/wideband_rx_analysis.py`        | Wideband capture analysis and visualization |
| `tests/bonded_rx_streamer_test.cpp`              | Unit tests for bonded RX                    |
| `tests/spectrum_stitcher_test.cpp`               | Unit tests for spectrum stitcher            |
| `docs/bonding/wideband_rx_howto.md`              | Wideband RX usage guide (CLI + GRC)         |

---

### Phase 5: Bonded TX Streaming

**Goal**: Implement a bonded TX streamer for coordinated multi-device transmission.

#### 5.1 Bonded TX Streamer

**Files**: `lib/usrp/bonded/bonded_tx_streamer.hpp`, `lib/usrp/bonded/bonded_tx_streamer.cpp`

```cpp
namespace uhd { namespace usrp { namespace bonded {

class bonded_tx_streamer
{
public:
    struct config {
        double total_bandwidth_hz;
        double overlap_fraction = 0.1;
        std::string sample_format = "fc32";
        std::string otw_format   = "sc16";
    };

    bonded_tx_streamer(
        multi_usrp::sptr usrp,
        const sync_manager& sync,
        const config& cfg
    );

    // Send samples across all devices with timed transmission
    size_t send(
        const std::vector<const std::complex<float>*>& buffs,
        size_t nsamps_per_device,
        const uhd::tx_metadata_t& md,
        double timeout = 1.0
    );

    // Get the frequency plan in use
    freq_planner::plan get_freq_plan() const;

private:
    multi_usrp::sptr _usrp;
    std::vector<uhd::tx_streamer::sptr> _streams;
    freq_planner::plan _plan;
};

}}} // namespace uhd::usrp::bonded
```

#### 5.2 TX Validation Example

**File**: `examples/bonded_wideband_tx.cpp`

```bash
bonded_wideband_tx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --start-freq=900e6 \
    --total-bw=100e6 \
    --rate=56e6 \
    --gain=30 \
    --waveform=chirp \
    --duration=5.0
```

#### Phase 5 Deliverables

| Deliverable                                      | Description                         |
| ------------------------------------------------ | ----------------------------------- |
| `include/uhd/usrp/bonded/bonded_tx_streamer.hpp` | Bonded TX streamer public API       |
| `lib/usrp/bonded/bonded_tx_streamer.cpp`         | Bonded TX streamer implementation   |
| `examples/bonded_wideband_tx.cpp`                | Wideband TX example                 |
| `tests/bonded_tx_streamer_test.cpp`              | Unit tests for bonded TX            |
| `docs/bonding/wideband_tx_howto.md`              | Wideband TX usage guide (CLI + GRC) |

---

### Phase 6: GNU Radio Integration

**Goal**: Document how to use bonded USRPs with GNU Radio Companion (GRC) using only standard blocks.

#### 6.1 GNU Radio with Standard USRP Blocks

Since the bonding configuration is entirely parameter-driven via `device_addr_t`, users can leverage bonded devices in GNU Radio by simply setting the **Device Address** field in the standard **USRP Source** and **USRP Sink** blocks.

**USRP Source (RX) configuration in GRC**:

| Parameter          | Value                                                                        |
| ------------------ | ---------------------------------------------------------------------------- |
| Device Address     | `serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external` |
| Sync               | `pc` or `unknown_pps`                                                        |
| Num Channels       | `2` (one per device)                                                         |
| Samp Rate          | `10e6` (per channel)                                                         |
| Center Freq (Ch 0) | `910e6`                                                                      |
| Center Freq (Ch 1) | `920e6`                                                                      |
| Gain (Ch 0)        | `30`                                                                         |
| Gain (Ch 1)        | `30`                                                                         |

**USRP Sink (TX) configuration in GRC**:

| Parameter          | Value                                                                        |
| ------------------ | ---------------------------------------------------------------------------- |
| Device Address     | `serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external` |
| Sync               | `pc` or `unknown_pps`                                                        |
| Num Channels       | `2`                                                                          |
| Samp Rate          | `10e6`                                                                       |
| Center Freq (Ch 0) | `910e6`                                                                      |
| Center Freq (Ch 1) | `920e6`                                                                      |
| Gain (Ch 0)        | `30`                                                                         |
| Gain (Ch 1)        | `30`                                                                         |

#### 6.2 Example GRC Flowgraphs

**File**: `examples/gnuradio/bonded_rx_to_file.grc`

A GRC flowgraph that:

1. Uses a standard USRP Source block with bonded device addresses
2. Connects each channel to a File Sink (saving per-device binary data)
3. Optionally connects to a QT GUI Frequency Sink for real-time visualization

**File**: `examples/gnuradio/bonded_tx_from_file.grc`

A GRC flowgraph that:

1. Reads waveform data from files using File Source blocks
2. Feeds each channel to a standard USRP Sink block with bonded device addresses

**File**: `examples/gnuradio/bonded_spectrum_display.grc`

A GRC flowgraph for live wideband spectrum display:

1. USRP Source with bonded devices (each on a different center frequency)
2. Per-channel FFT blocks
3. Combined display using QT GUI tabs

#### 6.3 GNU Radio with Internal Sync (No External Reference)

For users without external PPS/10 MHz hardware:

| Parameter      | Value                             |
| -------------- | --------------------------------- |
| Device Address | `serial0=30B56D6,serial1=30DBC3C` |
| Sync           | `pc`                              |
| Clock Source   | (leave default / internal)        |
| Time Source    | (leave default / internal)        |

Note: With internal references, sample timestamps will drift over time. This is acceptable for non-coherent applications (e.g., independent spectrum monitoring on different bands).

#### Phase 6 Deliverables

| Deliverable                                     | Description                         |
| ----------------------------------------------- | ----------------------------------- |
| `examples/gnuradio/bonded_rx_to_file.grc`       | GRC flowgraph for bonded RX         |
| `examples/gnuradio/bonded_tx_from_file.grc`     | GRC flowgraph for bonded TX         |
| `examples/gnuradio/bonded_spectrum_display.grc` | GRC flowgraph for live spectrum     |
| `docs/bonding/gnuradio_howto.md`                | Comprehensive GRC integration guide |

---

### Phase 7: Comprehensive Evaluation

**Goal**: Produce a complete set of measurements and figures for the thesis evaluation chapter.

#### 7.1 Test Matrix

| Test ID | Devices | Clock Source | Time Source | Rate (MHz) | Duration (s) | Purpose                       |
| ------- | ------- | ------------ | ----------- | ---------- | ------------ | ----------------------------- |
| T1      | 2       | external     | external    | 10         | 1            | Baseline sync quality         |
| T2      | 2       | external     | external    | 25         | 1            | Higher rate sync              |
| T3      | 2       | external     | external    | 56         | 1            | Maximum rate sync             |
| T4      | 2       | internal     | external    | 10         | 1            | PPS-only sync                 |
| T5      | 2       | internal     | internal    | 10         | 1            | Free-running (control)        |
| T6      | 2       | gpsdo        | gpsdo       | 10         | 1            | GPSDO sync (if available)     |
| T7      | 2       | external     | external    | 10         | 3600         | Long-term drift, external ref |
| T8      | 2       | internal     | external    | 10         | 3600         | Long-term drift, PPS-only     |
| T9      | 2       | internal     | internal    | 10         | 3600         | Long-term drift, free-running |
| T10     | 2       | external     | external    | 56         | 5            | Wideband bonded RX            |

#### 7.2 Evaluation Metrics

For each test, the following metrics are computed and tabulated:

| Metric                  | Unit          | Method                                      |
| ----------------------- | ------------- | ------------------------------------------- |
| Time offset             | µs            | Cross-correlation peak location             |
| Time offset uncertainty | µs            | Cross-correlation peak width                |
| Frequency offset        | Hz            | Phase difference slope (linear fit)         |
| Phase jitter            | degrees (rms) | Std. dev. of instantaneous phase difference |
| Drift rate              | ppm           | PPS timestamp delta over time               |
| Coherence (mean)        | dimensionless | Mean MSC across band                        |
| Overflow count          | count         | Per-device overflow events                  |
| Capture throughput      | MB/s          | Total sustained write rate                  |

#### 7.3 Summary Table for Thesis

**File**: `examples/python/generate_evaluation_table.py`

Generates a LaTeX-formatted table of all metrics across all test configurations:

```latex
\begin{table}[htbp]
\centering
\caption{Synchronization performance across configuration modes}
\label{tab:sync_performance}
\begin{tabular}{lcccccc}
\toprule
Mode & Time Offset (µs) & Freq. Offset (Hz) & Phase Jitter (°) & Drift (ppm) & Coherence \\
\midrule
Ext. 10 MHz + PPS & $< 0.1$ & $< 0.01$ & $< 1.0$ & $\approx 0$ & $> 0.99$ \\
PPS only           & ...     & ...      & ...     & ...         & ...       \\
Internal           & ...     & ...      & ...     & ...         & ...       \\
\bottomrule
\end{tabular}
\end{table}
```

#### 7.4 Automated Test Runner

**File**: `examples/python/run_evaluation.py`

A Python script that orchestrates the full test matrix:

1. For each test configuration, invokes the C++ capture tool
2. Runs the analysis script to produce figures
3. Collects metrics into a combined results JSON
4. Generates the LaTeX summary table

```bash
python3 examples/python/run_evaluation.py \
    --output-dir=./evaluation_results \
    --build-dir=./build
```

#### Phase 7 Deliverables

| Deliverable                                    | Description                                 |
| ---------------------------------------------- | ------------------------------------------- |
| `examples/python/run_evaluation.py`            | Automated test runner                       |
| `examples/python/generate_evaluation_table.py` | LaTeX table generator                       |
| `evaluation_results/`                          | Complete set of captures, plots, and tables |
| `docs/bonding/evaluation_howto.md`             | Evaluation procedure documentation          |

---

## 5. Synchronization Modes

### 5.1 Supported Modes

| Mode             | Clock Source | Time Source | Ext. Hardware | Drift Comp.  | Use Case                            |
| ---------------- | ------------ | ----------- | ------------- | ------------ | ----------------------------------- |
| **External Ref** | `external`   | `external`  | 10 MHz + PPS  | Not required | Coherent bonding (recommended)      |
| **PPS-Only**     | `internal`   | `external`  | PPS only      | Required     | Budget-friendly wideband monitoring |
| **GPSDO**        | `gpsdo`      | `gpsdo`     | GPSDO module  | Not required | Distributed / outdoor deployments   |
| **Internal**     | `internal`   | `internal`  | None          | Required     | Testing / non-coherent applications |

### 5.2 Configuration via Device Arguments

All modes are configured through standard UHD `device_addr_t` parameters:

```cpp
// External reference (optimal)
uhd::device_addr_t args;
args["serial0"] = "30B56D6";
args["serial1"] = "30DBC3C";
args["clock_source"] = "external";
args["time_source"]  = "external";
auto usrp = uhd::usrp::multi_usrp::make(args);

// PPS-only
args["clock_source"] = "internal";
args["time_source"]  = "external";

// GPSDO
args["clock_source"] = "gpsdo";
args["time_source"]  = "gpsdo";

// Internal (free-running)
args["clock_source"] = "internal";
args["time_source"]  = "internal";
```

The bonding layer reads these parameters and configures each motherboard accordingly. There is no bonding-specific configuration; everything follows the standard UHD parameter conventions.

### 5.3 Performance Characteristics

| Parameter              | External Ref        | PPS-Only            | GPSDO        | Internal        |
| ---------------------- | ------------------- | ------------------- | ------------ | --------------- |
| Time sync accuracy     | < 100 ns            | < 100 ns            | < 100 ns     | > 1 µs          |
| Frequency accuracy     | < 0.01 ppm          | 2–5 ppm             | < 0.01 ppm   | 2–5 ppm         |
| Phase coherence        | Excellent           | Poor (drifts)       | Excellent    | Poor (drifts)   |
| Long-term stability    | Excellent           | Poor                | Excellent    | Poor            |
| Hardware cost          | ~$200 (clock dist.) | ~$30 (PPS splitter) | ~$300/device | $0              |
| Suitable for stitching | Yes                 | With compensation   | Yes          | Not recommended |

---

## 6. API Design

### 6.1 Design Philosophy

The bonding API is designed as a **utility layer on top of `multi_usrp`**, not a replacement. Users who need fine-grained control can still use `multi_usrp` directly. The bonding utilities provide convenience functions for common multi-device workflows.

### 6.2 Bonding Utilitiy Entry Points

```cpp
#include <uhd/usrp/bonded/sync_manager.hpp>
#include <uhd/usrp/bonded/freq_planner.hpp>
#include <uhd/usrp/bonded/drift_monitor.hpp>
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/usrp/bonded/bonded_tx_streamer.hpp>
#include <uhd/usrp/bonded/spectrum_stitcher.hpp>
```

### 6.3 Typical Usage Pattern

```cpp
// 1. Create multi_usrp with N devices
uhd::device_addr_t args;
args["serial0"]      = "30B56D6";
args["serial1"]      = "30DBC3C";
args["clock_source"] = "external";
args["time_source"]  = "external";
auto usrp = uhd::usrp::multi_usrp::make(args);

// 2. Synchronize devices
uhd::usrp::bonded::sync_manager sync(usrp, {
    .clock_source = "external",
    .time_source  = "external"
});
sync.configure_sources();
sync.wait_for_lock();
sync.synchronize_time();
auto status = sync.verify_sync();

// 3. Plan frequencies for wideband coverage
auto plan = uhd::usrp::bonded::freq_planner::compute(
    900e6,   // start frequency
    100e6,   // total bandwidth
    2,       // number of devices
    56e6,    // per-device sample rate
    0.1      // 10% overlap
);

// 4. Configure and start bonded RX
uhd::usrp::bonded::bonded_rx_streamer bonded_rx(usrp, sync, {
    .total_bandwidth_hz = 100e6,
    .overlap_fraction   = 0.1
});
bonded_rx.start(usrp->get_time_now() + uhd::time_spec_t(1.0));

// 5. Receive samples
std::vector<std::complex<float>*> buffs(2);
// ... allocate buffers ...
uhd::rx_metadata_t md;
bonded_rx.recv(buffs, 4096, md, 1.0);

// 6. Stitch spectrum
uhd::usrp::bonded::spectrum_stitcher stitcher({
    .fft_size         = 4096,
    .overlap_hz       = plan.overlap_hz,
    .center_freqs_hz  = plan.center_frequencies_hz,
    .sample_rate_hz   = plan.per_device_rate_hz[0]
});
auto wideband_spectrum = stitcher.stitch(per_device_samples);
```

---

## 7. Data Formats and File Conventions

### 7.1 Binary Sample Format

All binary capture files use the **sc16** (signed complex 16-bit integer) format by default:

- Each sample: 4 bytes (2 bytes I + 2 bytes Q)
- I component: `int16_t`, little-endian
- Q component: `int16_t`, little-endian
- Full-scale: ±32767

Reading in Python:

```python
import numpy as np

def load_sc16(filepath):
    raw = np.fromfile(filepath, dtype=np.int16)
    iq = raw[0::2] + 1j * raw[1::2]
    return iq.astype(np.complex64) / 32768.0  # Normalize to [-1, 1]
```

### 7.2 Metadata JSON

Every capture produces a companion `metadata.json` file containing:

- Capture tool name and version
- All configuration parameters (freq, rate, gain, clock/time source, etc.)
- Per-device information (serial, actual tuned values, overflow counts)
- Synchronization results (PPS deltas, drift measurements)
- Timestamps in UTC ISO 8601

### 7.3 File Naming Convention

```
<output_dir>/
├── metadata.json
├── usrp_0_<serial0>.bin
├── usrp_1_<serial1>.bin
├── ...
├── usrp_N-1_<serialN-1>.bin
└── plots/
    ├── fig01_time_domain.png
    ├── fig01_time_domain.pdf
    ├── fig02_cross_correlation.png
    ├── fig02_cross_correlation.pdf
    └── ...
```

---

## 8. Visualization and Academic Figures

### 8.1 Figure Requirements

All figures produced by this project must meet the following standards for inclusion in a master's thesis:

1. **Resolution**: 300 DPI minimum for raster formats; vector PDF preferred for line plots
2. **Font**: Serif family (Times New Roman or Computer Modern), consistent with LaTeX documents
3. **Font sizes**: 11–13 pt for body text, axis labels, and legends
4. **Grid**: Light gray gridlines (alpha 0.3) for readability
5. **Colors**: Colorblind-friendly palette; primary colors clearly distinguishable in grayscale
6. **Axis labels**: Full descriptive text with SI units in parentheses
7. **Legends**: Placed to avoid obscuring data; Device labels include serial numbers
8. **Figure size**: 7" × 4.5" default (single column); 3.3" × 3" for multi-panel sub-figures
9. **Captions**: Every figure includes 1–2 paragraphs explaining what is shown and why

### 8.2 Complete Figure List

| Figure  | Source Phase | Script                    | Description                                       |
| ------- | ------------ | ------------------------- | ------------------------------------------------- |
| Fig. 1  | Phase 2      | `sync_analysis.py`        | Time-domain overlay of synchronized captures      |
| Fig. 2  | Phase 2      | `sync_analysis.py`        | Cross-correlation peak for time offset estimation |
| Fig. 3  | Phase 2      | `sync_analysis.py`        | Phase difference over time                        |
| Fig. 4  | Phase 2      | `sync_analysis.py`        | PSD overlay from both devices                     |
| Fig. 5  | Phase 2      | `sync_analysis.py`        | Magnitude-squared coherence                       |
| Fig. 6  | Phase 2      | `sync_analysis.py`        | Synchronization summary dashboard                 |
| Fig. 7  | Phase 2      | `sync_analysis.py`        | IQ constellation diagram                          |
| Fig. 8  | Phase 2      | `sync_analysis.py`        | Phase difference histogram                        |
| Fig. 9  | Phase 2      | `drift_analysis.py`       | Drift rate over time                              |
| Fig. 10 | Phase 7      | `run_evaluation.py`       | Sync mode comparison bar chart                    |
| Fig. 11 | Phase 4      | `wideband_rx_analysis.py` | Stitched wideband spectrum                        |
| Fig. 12 | Phase 4      | `wideband_rx_analysis.py` | Overlap region detail and stitching quality       |
| Fig. 13 | Phase 7      | `run_evaluation.py`       | Throughput vs. number of devices                  |

---

## 9. How to Run

### 9.1 Prerequisites

```bash
# System dependencies
sudo apt-get install -y libuhd-dev uhd-host python3-numpy python3-matplotlib python3-scipy

# Build UHD from this repository
cd /home/gabriel/uhd/host
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 9.2 Phase 1: Synchronization Validation (CLI)

**External 10 MHz + PPS**:

```bash
cd /home/gabriel/uhd/host/build

# Capture synchronized data
./examples/sync_validation_capture \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --duration=1.0 \
    --output-dir=./sync_capture

# Monitor drift over 10 minutes
./examples/drift_monitor \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --duration=600 \
    --interval=1.0 \
    --output=./drift_external.csv
```

**PPS-only (internal clock)**:

```bash
./examples/sync_validation_capture \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=internal \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --duration=1.0 \
    --output-dir=./sync_capture_pps_only
```

**Internal only (no external references)**:

```bash
./examples/sync_validation_capture \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=internal \
    --time-source=internal \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --duration=1.0 \
    --output-dir=./sync_capture_internal
```

**TX synchronization validation**:

```bash
./examples/sync_validation_tx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --waveform=tone \
    --duration=1.0
```

### 9.3 Phase 1: Synchronization Validation (GNU Radio Companion)

To perform an equivalent synchronized capture in GRC:

1. Open GNU Radio Companion
2. Add a **USRP Source** block with:
   - Device Address: `serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external`
   - Sync: `Unknown PPS`
   - Num Channels: `2`
   - Sample Rate: `10e6`
   - Ch0 Center Freq: `915e6`
   - Ch1 Center Freq: `915e6`
   - Ch0 Gain: `30`
   - Ch1 Gain: `30`
3. Connect each output to a **File Sink** block:
   - Ch0 → `usrp_0_30B56D6.bin`
   - Ch1 → `usrp_1_30DBC3C.bin`
4. Optionally add **QT GUI Frequency Sink** blocks for real-time monitoring
5. Run the flowgraph

For TX validation:

1. Add a **Signal Source** block generating a CW tone
2. Connect to a **USRP Sink** block with:
   - Device Address: `serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external`
   - Sync: `Unknown PPS`
   - Num Channels: `2`
   - Sample Rate: `10e6`
3. Run the flowgraph

### 9.4 Phase 2: Analysis and Visualization

```bash
# Generate all synchronization analysis figures
python3 /home/gabriel/uhd/host/examples/python/sync_analysis.py \
    --input-dir=./sync_capture \
    --output-dir=./sync_capture/plots \
    --format=png \
    --dpi=300

# Generate drift analysis figures
python3 /home/gabriel/uhd/host/examples/python/drift_analysis.py \
    --input=./drift_external.csv \
    --output-dir=./drift_plots

# Compare all sync modes
python3 /home/gabriel/uhd/host/examples/python/sync_analysis.py \
    --input-dir=./sync_capture \
    --input-dir=./sync_capture_pps_only \
    --input-dir=./sync_capture_internal \
    --output-dir=./comparison_plots \
    --compare
```

### 9.5 Phase 4: Wideband Bonded RX (CLI)

```bash
# Wideband capture across 100 MHz
./examples/bonded_wideband_rx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --start-freq=900e6 \
    --total-bw=100e6 \
    --rate=56e6 \
    --gain=30 \
    --duration=5.0 \
    --output-dir=./wideband_capture

# Analyze wideband capture
python3 /home/gabriel/uhd/host/examples/python/wideband_rx_analysis.py \
    --input-dir=./wideband_capture \
    --output-dir=./wideband_capture/plots
```

### 9.6 Phase 4: Wideband Bonded RX (GNU Radio Companion)

1. Open GNU Radio Companion
2. Add a **USRP Source** block:
   - Device Address: `serial0=30B56D6,serial1=30DBC3C,clock_source=external,time_source=external`
   - Sync: `Unknown PPS`
   - Num Channels: `2`
   - Sample Rate: `56e6`
   - Ch0 Center Freq: `922e6` (900 + 56/2 - overlap/2)
   - Ch1 Center Freq: `972.4e6` (next segment with overlap)
   - Ch0 Gain: `30`
   - Ch1 Gain: `30`
3. Connect each channel output to processing blocks as needed
4. For spectrum stitching, use Python blocks or post-process with `wideband_rx_analysis.py`

### 9.7 Phase 5: Wideband Bonded TX (CLI)

```bash
./examples/bonded_wideband_tx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --start-freq=900e6 \
    --total-bw=100e6 \
    --rate=56e6 \
    --gain=30 \
    --waveform=chirp \
    --duration=5.0
```

### 9.8 Phase 7: Full Evaluation

```bash
# Run the complete evaluation test matrix
python3 /home/gabriel/uhd/host/examples/python/run_evaluation.py \
    --output-dir=./evaluation_results \
    --build-dir=/home/gabriel/uhd/host/build

# Generate LaTeX tables
python3 /home/gabriel/uhd/host/examples/python/generate_evaluation_table.py \
    --input-dir=./evaluation_results \
    --output=./evaluation_results/sync_performance_table.tex
```

---

## 10. References

1. Ettus Research, "USRP Hardware Driver and USRP Manual," https://files.ettus.com/manual/
2. Ettus Research, "Synchronization and MIMO Capability with USRP Devices," Application Note.
3. M. Lichtman et al., "Multi-USRP Synchronization for Wideband Spectrum Monitoring," IEEE MILCOM, 2016.
4. S. Merlin et al., "An Architecture for Multi-Band Spectrum Sensing Using Software-Defined Radios," IEEE DySPAN, 2015.
5. GNU Radio Project, "USRP Source/Sink Block Documentation," https://wiki.gnuradio.org/
6. E. Grayver, _Implementing Software Defined Radio_. Springer, 2013.
7. UHD Source Code, `include/uhd/usrp/multi_usrp.hpp` — Multi-USRP API reference.
8. UHD Source Code, `examples/rx_multi_samples.cpp` — Multi-device RX example.
