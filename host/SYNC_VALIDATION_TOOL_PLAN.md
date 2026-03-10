# USRP Synchronization Validation Tool — Implementation Plan

## Multi-Device Time & Phase Sync Validation with Binary Capture and Academic-Grade Visualization

**Project Goal**: Build a lightweight C++ example tool and a companion Python post-processing script that together validate time and phase synchronization across multiple USRP devices sharing an external PPS reference. The tool captures raw IQ data, performs real-time sync checks, and exports binary dumps. The Python script generates publication-quality plots suitable for academic research.

**Target Use Case**: Masters/PhD research requiring evidence that multiple USRPs are time-aligned and phase-coherent (or quantifying the deviation) after PPS synchronization.

---

## Table of Contents

1. [Overview & Motivation](#1-overview--motivation)
2. [Tool Architecture](#2-tool-architecture)
3. [C++ Capture Tool — `sync_validation_test`](#3-c-capture-tool)
4. [Binary Dump File Format](#4-binary-dump-file-format)
5. [Real-Time Sync Checks (C++ Side)](#5-real-time-sync-checks)
6. [Python Post-Processing Script — `plot_sync_validation.py`](#6-python-post-processing-script)
7. [Academic-Grade Plot Specifications](#7-academic-grade-plot-specifications)
8. [Implementation Phases](#8-implementation-phases)
9. [Build Integration](#9-build-integration)
10. [Usage Examples](#10-usage-examples)

---

## 1. Overview & Motivation

When using multiple USRPs synchronized via an external PPS signal (with or without a shared 10 MHz reference), researchers need an objective way to **validate** that:

- All devices begin streaming at the same hardware timestamp.
- Received samples are time-aligned across devices (sub-sample precision).
- Phase coherence is maintained (when using a shared 10 MHz clock) or its drift is quantified (when using independent TCXOs with PPS-only sync).
- No unexpected sample drops or timing discontinuities occurred.

This tool provides a **turnkey validation workflow**: run one command to capture, then one command to generate all plots.

### What This Tool Is NOT

- It is **not** a bonding/stitching engine — it does not combine signals.
- It is **not** a runtime daemon — it captures a finite snapshot and exits.
- It is **not** hardware-specific — it works with any UHD-compatible USRP that supports external PPS.

---

## 2. Tool Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    sync_validation_test (C++)                    │
│                                                                  │
│  ┌──────────┐  ┌──────────┐       ┌──────────┐                  │
│  │  USRP 0  │  │  USRP 1  │  ...  │  USRP N  │                 │
│  │ serial=A │  │ serial=B │       │ serial=Z │                  │
│  └────┬─────┘  └────┬─────┘       └────┬─────┘                  │
│       │              │                  │                         │
│       ▼              ▼                  ▼                         │
│  ┌─────────────────────────────────────────────┐                 │
│  │         PPS Sync & Timed Streaming          │                 │
│  │  - set_time_source("external")              │                 │
│  │  - set_time_unknown_pps(0.0)                │                 │
│  │  - timed stream_cmd (all start together)    │                 │
│  └──────────────────┬──────────────────────────┘                 │
│                     │                                            │
│       ┌─────────────┼─────────────┐                              │
│       ▼             ▼             ▼                              │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐                          │
│  │ RX Buf 0│  │ RX Buf 1│  │ RX Buf N│                          │
│  └────┬────┘  └────┬────┘  └────┬────┘                          │
│       │             │            │                                │
│       ▼             ▼            ▼                                │
│  ┌──────────────────────────────────────────┐                    │
│  │  Real-Time Checks (console output)       │                   │
│  │  - Timestamp alignment across devices    │                   │
│  │  - Overflow / error detection            │                   │
│  │  - PPS drift (get_time_last_pps delta)   │                   │
│  └──────────────────────────────────────────┘                    │
│       │             │            │                                │
│       ▼             ▼            ▼                                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐    ┌───────────────────┐ │
│  │ .bin  0 │  │ .bin  1 │  │ .bin  N │    │  metadata.json    │ │
│  └─────────┘  └─────────┘  └─────────┘    └───────────────────┘ │
└──────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌──────────────────────────────────────────────────────────────────┐
│              plot_sync_validation.py (Python)                    │
│                                                                  │
│  Reads: *.bin files + metadata.json                              │
│  Outputs: PDF/PNG figures (publication-ready)                    │
│                                                                  │
│  Plots generated:                                                │
│   1. Time-domain waveform overlay                                │
│   2. Cross-correlation (inter-device)                            │
│   3. Phase difference vs. time                                   │
│   4. Power spectral density overlay                              │
│   5. Timestamp alignment summary                                 │
│   6. Drift rate estimation (if PPS data available)               │
│   7. Coherence (magnitude-squared coherence)                     │
│   8. Constellation / IQ scatter                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. C++ Capture Tool — `sync_validation_test`

### 3.1 Command-Line Interface

The tool must be simple to invoke. Two serial specification modes are supported:

```bash
# Mode 1: Named serial arguments
sync_validation_test --serial0 30B56D6 --serial1 30DBC3C \
    --freq 915e6 --rate 10e6

# Mode 2: Comma-separated serial list (scales to N devices)
sync_validation_test --serials "30B56D6,30DBC3C" \
    --freq 915e6 --rate 10e6

# Full options
sync_validation_test \
    --serials "30B56D6,30DBC3C,30EAB12"   \
    --freq 915e6                           \
    --rate 10e6                            \
    --gain 30                              \
    --duration 1.0                         \
    --bw 10e6                              \
    --output-dir ./sync_capture            \
    --format sc16                          \
    --clock-source internal                \
    --subdev "A:A"                         \
    --channel 0                            \
    --skip-drift-check
```

### 3.2 Program Options Table

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `--serials` | string | *(required)* | Comma-separated list of USRP serial numbers |
| `--serial0`, `--serial1`, ... | string | — | Alternative: individual serial arguments |
| `--freq` | double | *(required)* | Center frequency in Hz (e.g., `915e6`) |
| `--rate` | double | `1e6` | Sample rate in samples/s |
| `--gain` | double | `0` | RX gain in dB |
| `--bw` | double | `0` (auto) | Analog bandwidth in Hz (0 = use rate) |
| `--duration` | double | `1.0` | Capture duration in seconds |
| `--output-dir` | string | `./sync_capture` | Directory for output files |
| `--format` | string | `sc16` | Sample format: `sc16` (16-bit int I/Q) or `fc32` (32-bit float I/Q) |
| `--clock-source` | string | `internal` | Clock source: `internal`, `external` |
| `--subdev` | string | — | RX subdevice spec (applied to all USRPs) |
| `--channel` | int | `0` | RX channel index per device |
| `--skip-drift-check` | flag | false | Skip the post-capture PPS drift measurement |
| `--antenna` | string | — | Antenna port (e.g., `RX2`, `TX/RX`) |
| `--spb` | size_t | `10000` | Samples per buffer for recv() |
| `--ref` | string | `external` | PPS/time reference source |

### 3.3 Program Flow (Pseudocode)

```
1.  Parse command-line options
2.  Build device address string: "serial=AAA,serial=BBB,..."
3.  Create multi_usrp from combined address
4.  For each mboard:
      a. Set clock_source (internal or external)
      b. Set time_source("external")  — PPS input
      c. Set RX rate, frequency, gain, bandwidth, antenna
5.  Verify PPS present on all mboards:
      - Poll get_time_last_pps() with timeout
      - Abort with clear error if PPS missing on any board
6.  Synchronize time at PPS edge:
      - usrp->set_time_unknown_pps(time_spec_t(0.0))
      - Sleep 2 seconds
7.  Verify time sync:
      - Read get_time_last_pps() on all mboards
      - Print per-board timestamps; error if delta > 1 ms
8.  Create output directory (if not exists)
9.  Create one rx_streamer with all device channels
      - stream_args.channels = {0, 1, ..., N-1}  (one ch per device)
10. Open one binary output file per device
11. Record start metadata (JSON):
      - Serials, freq, rate, gain, format, duration, timestamps, etc.
12. Issue timed stream command:
      - stream_now = false
      - time_spec = get_time_now() + 1.0 s (safe margin)
13. Receive loop:
      - recv() into per-channel buffers
      - Write each channel's buffer to its binary file
      - Track per-channel:
          * First packet timestamp (from rx_metadata_t.time_spec)
          * Total samples received
          * Overflow count
          * Any error codes
      - Print progress every 0.5 seconds
      - Respect --duration or Ctrl+C (SIGINT)
14. Stop streaming
15. Close binary files
16. Post-capture PPS drift check (unless --skip-drift-check):
      - Read get_time_last_pps() on all mboards
      - Sleep 5 seconds
      - Read again
      - Compute inter-device drift in ppm
17. Write final metadata.json:
      - All parameters
      - Per-device: serial, first_timestamp, total_samples, overflows
      - Drift measurement results (if performed)
      - File paths to binary dumps
18. Print summary to console
```

### 3.4 Multi-Device Initialization Details

All USRPs are opened as a single `multi_usrp` so that UHD handles coordinated tuning and timed commands:

```cpp
// Build the device address from serial list
uhd::device_addr_t dev_addr;
for (size_t i = 0; i < serials.size(); ++i) {
    dev_addr["serial" + (i == 0 ? "" : std::to_string(i))] = serials[i];
    // Produces: "serial=AAA,serial1=BBB,serial2=CCC"
    // UHD interprets this as a multi-device configuration
}

auto usrp = uhd::usrp::multi_usrp::make(dev_addr);
// usrp->get_num_mboards() == serials.size()
```

### 3.5 Streaming Strategy

A **single rx_streamer** with N channels (one per device) is used. This is the simplest approach and leverages UHD's internal alignment. The `recv()` call fills buffers for all channels simultaneously:

```cpp
uhd::stream_args_t stream_args(cpu_format, wire_format);
// One channel per device: channel 0 from dev0, channel 0 from dev1, etc.
for (size_t i = 0; i < num_devices; ++i) {
    stream_args.channels.push_back(i * channels_per_device + selected_channel);
}
auto rx_stream = usrp->get_rx_stream(stream_args);
```

Using a single streamer means `recv()` returns **time-aligned** samples across channels — UHD aligns based on timestamps internally. The metadata timestamp corresponds to all channels in the returned buffer.

### 3.6 Signal Handling

Register `SIGINT` handler for clean shutdown:

```cpp
static bool stop_signal_called = false;
void sig_int_handler(int) { stop_signal_called = true; }
std::signal(SIGINT, sig_int_handler);
```

---

## 4. Binary Dump File Format

### 4.1 File Naming Convention

```
<output-dir>/
├── usrp_0_<serial>.bin       # Raw IQ samples from device 0
├── usrp_1_<serial>.bin       # Raw IQ samples from device 1
├── ...
├── usrp_N_<serial>.bin       # Raw IQ samples from device N
└── metadata.json             # Capture metadata
```

### 4.2 Binary Format

Each `.bin` file contains raw interleaved I/Q samples with **no header**:

- **`sc16` format** (default): Signed 16-bit integers, interleaved `[I0, Q0, I1, Q1, ...]`. Each sample is 4 bytes (2 bytes I + 2 bytes Q). This is the native wire format for most USRPs and preserves full fidelity.
- **`fc32` format**: 32-bit IEEE 754 floats, interleaved `[I0, Q0, I1, Q1, ...]`. Each sample is 8 bytes. Easier to load in Python/NumPy but 2× file size.

File size = `num_samples × bytes_per_sample`:
- `sc16`: 1 second at 10 MS/s = 10M × 4 bytes = 40 MB
- `fc32`: 1 second at 10 MS/s = 10M × 8 bytes = 80 MB

### 4.3 Metadata JSON Schema

```json
{
    "capture_tool": "sync_validation_test",
    "capture_version": "1.0",
    "capture_timestamp_utc": "2026-03-10T14:30:00Z",
    "parameters": {
        "center_frequency_hz": 915000000.0,
        "sample_rate_hz": 10000000.0,
        "gain_db": 30.0,
        "bandwidth_hz": 10000000.0,
        "duration_s": 1.0,
        "sample_format": "sc16",
        "clock_source": "internal",
        "time_source": "external",
        "bytes_per_sample": 4
    },
    "devices": [
        {
            "index": 0,
            "serial": "30B56D6",
            "mboard_name": "B210",
            "file": "usrp_0_30B56D6.bin",
            "actual_frequency_hz": 914999998.0,
            "actual_rate_hz": 9999999.5,
            "actual_gain_db": 30.0,
            "first_sample_timestamp_ticks": 20000000,
            "first_sample_timestamp_s": 2.0,
            "total_samples": 10000000,
            "overflow_count": 0,
            "stream_errors": []
        },
        {
            "index": 1,
            "serial": "30DBC3C",
            "mboard_name": "B210",
            "file": "usrp_1_30DBC3C.bin",
            "actual_frequency_hz": 914999998.0,
            "actual_rate_hz": 9999999.5,
            "actual_gain_db": 30.0,
            "first_sample_timestamp_ticks": 20000000,
            "first_sample_timestamp_s": 2.0,
            "total_samples": 10000000,
            "overflow_count": 0,
            "stream_errors": []
        }
    ],
    "sync_results": {
        "pps_time_deltas_us": [0.0, 0.018],
        "timestamp_alignment_ticks": [0, 0],
        "drift_check_performed": true,
        "drift_measurement_duration_s": 5.0,
        "drift_ppm": [0.0, 1.23],
        "drift_reference_device": 0
    }
}
```

---

## 5. Real-Time Sync Checks (C++ Side)

The C++ tool performs the following checks during and after capture, printing results to stdout:

### 5.1 Pre-Capture Checks

| Check | Method | Pass Criteria |
|-------|--------|---------------|
| PPS detected | Poll `get_time_last_pps()` for edge change within 2 s | Edge detected on all boards |
| Time sync | Compare `get_time_last_pps()` across boards after `set_time_unknown_pps()` | All deltas < 1 ms |
| Tuning | Compare `get_rx_freq()` vs. requested | Within 1 Hz |
| Rate | Compare `get_rx_rate()` vs. requested | Log actual value |

### 5.2 During Capture

| Check | Method | Action |
|-------|--------|--------|
| First-packet timestamp match | Compare `rx_metadata_t.time_spec` from first recv | Log tick offset; warn if > 0 |
| Overflow detection | Check `rx_metadata_t.error_code == OVERFLOW` | Increment counter, log, continue |
| Timeout detection | Check `rx_metadata_t.error_code == TIMEOUT` | Log and stop |

### 5.3 Post-Capture Checks

| Check | Method | Output |
|-------|--------|--------|
| Sample count | Verify all devices captured same number of samples | Warn if mismatch |
| PPS drift | Measure `get_time_last_pps()` delta over 5 seconds | Report ppm per device |
| Cross-correlation peak (optional, for fc32 only) | Compute lag at max cross-correlation of first 1024 samples between device pairs | Report sample offset |

### 5.4 Console Output Format

```
=== USRP Sync Validation Test ===

[INFO] Opening 2 devices: serial=30B56D6, serial=30DBC3C
[INFO] Device 0: B210 (serial=30B56D6)
[INFO] Device 1: B210 (serial=30DBC3C)
[INFO] Setting frequency: 915.000 MHz
[INFO] Setting sample rate: 10.000 MS/s
[INFO] Setting gain: 30.0 dB

[SYNC] Checking PPS on all devices...
[SYNC] Device 0: PPS detected ✓
[SYNC] Device 1: PPS detected ✓
[SYNC] Synchronizing time at PPS edge...
[SYNC] Device 0: PPS time = 1.000000 s
[SYNC] Device 1: PPS time = 1.000000 s
[SYNC] Max time delta: 0.018 µs — PASS

[CAPTURE] Streaming 10.000 M samples (1.000 s) starting at t=2.000 s...
[CAPTURE] Progress: 25.0% | 2.50M samples | 0 overflows
[CAPTURE] Progress: 50.0% | 5.00M samples | 0 overflows
[CAPTURE] Progress: 75.0% | 7.50M samples | 0 overflows
[CAPTURE] Progress: 100.0% | 10.00M samples | 0 overflows
[CAPTURE] Done. All devices captured 10000000 samples.

[DRIFT] Measuring PPS drift over 5 seconds...
[DRIFT] Device 0 (ref): 0.000 ppm
[DRIFT] Device 1:       +1.23 ppm

[OUTPUT] Files written to ./sync_capture/
[OUTPUT]   usrp_0_30B56D6.bin  (40.00 MB)
[OUTPUT]   usrp_1_30DBC3C.bin  (40.00 MB)
[OUTPUT]   metadata.json

=== Summary ===
  Devices:      2
  Samples/dev:  10,000,000
  Duration:     1.000 s
  Overflows:    0 total
  Time sync:    PASS (delta < 1 µs)
  Drift:        +1.23 ppm (dev1 vs dev0)
  Status:       SUCCESS
```

---

## 6. Python Post-Processing Script — `plot_sync_validation.py`

### 6.1 Command-Line Interface

```bash
# Basic usage — reads metadata.json from the capture directory
python3 plot_sync_validation.py --input-dir ./sync_capture

# Full options
python3 plot_sync_validation.py \
    --input-dir ./sync_capture     \
    --output-dir ./sync_plots      \
    --format pdf                   \
    --dpi 300                      \
    --max-samples 100000           \
    --no-show
```

| Option | Default | Description |
|--------|---------|-------------|
| `--input-dir` | `./sync_capture` | Directory with `.bin` files and `metadata.json` |
| `--output-dir` | `./sync_plots` | Directory for output figures |
| `--format` | `pdf` | Output format: `pdf`, `png`, `svg` |
| `--dpi` | `300` | DPI for raster outputs |
| `--max-samples` | `100000` | Max samples to plot in time-domain (for readability) |
| `--no-show` | flag | Don't display interactive plots, just save |
| `--figsize` | `8,5` | Default figure size in inches (width,height) |

### 6.2 Script Flow

```
1.  Read metadata.json
2.  Load binary files:
      - Read sample format (sc16 or fc32) from metadata
      - np.fromfile() with appropriate dtype
      - For sc16: load as int16, reshape to (N,2), convert to complex64
      - For fc32: load as complex64 directly
3.  Validate loaded data:
      - Check sample counts match metadata
      - Print summary
4.  Generate plots (one function per figure):
      a. Time-domain waveform overlay
      b. Cross-correlation magnitude (all pairs)
      c. Instantaneous phase difference vs. time (all pairs)
      d. Power spectral density overlay
      e. Magnitude-squared coherence (all pairs)
      f. Timestamp & sync summary table (text figure)
      g. IQ constellation scatter
      h. Phase difference histogram
5.  Save all figures
6.  Print output paths
```

### 6.3 Dependencies

```
numpy >= 1.20
matplotlib >= 3.5
scipy >= 1.7
```

Standard scientific Python stack — no exotic dependencies.

---

## 7. Academic-Grade Plot Specifications

All plots follow these conventions for publication quality:

### 7.1 Global Style Settings

```python
import matplotlib as mpl
import matplotlib.pyplot as plt

# Use a clean, publication-ready style
plt.style.use('seaborn-v0_8-paper')  # or 'seaborn-paper' on older matplotlib

mpl.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'Computer Modern Roman'],
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.figsize': (8, 5),
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
    'lines.linewidth': 1.0,
    'lines.markersize': 4,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'legend.framealpha': 0.8,
    'legend.edgecolor': '0.8',
    'text.usetex': False,  # Set True if LaTeX is available
    'mathtext.fontset': 'cm',
})
```

### 7.2 Plot Descriptions

#### Plot 1: Time-Domain Waveform Overlay

- **Purpose**: Visual comparison of received signals across devices.
- **X-axis**: Time (µs), relative to capture start.
- **Y-axis**: Amplitude (normalized) or raw I/Q values.
- **Content**: Plot real part (I) of each device's signal, overlaid with distinct colors. Show only a short window (e.g., first 500 samples = 50 µs at 10 MS/s) for readability.
- **Annotations**: Device serial in legend. Vertical dashed line at t = 0.
- **Subplot variant**: Optionally a 2-row subplot (I on top, Q on bottom) for each pair.

#### Plot 2: Cross-Correlation Between Devices

- **Purpose**: Measure time delay between device signals.
- **X-axis**: Lag (samples), centered at 0.
- **Y-axis**: Normalized cross-correlation magnitude.
- **Content**: For each device pair (i, j), compute `correlate(sig_i, sig_j, mode='full')`, normalize by geometric mean of energies. Plot around lag = 0 (e.g., ±50 samples).
- **Annotations**: Mark peak location with vertical line and text label showing lag in samples and µs.
- **Subplot**: One subplot row per device pair (for > 2 devices).

#### Plot 3: Instantaneous Phase Difference vs. Time

- **Purpose**: Show phase drift between device pairs over the capture duration.
- **X-axis**: Time (ms or s).
- **Y-axis**: Phase difference (radians), unwrapped.
- **Content**: Compute `angle(sig_i * conj(sig_j))`, apply `np.unwrap()`. Plot the result. Fit a linear trend and overlay it — the slope gives the frequency offset.
- **Annotations**: Text box with fitted slope in Hz and equivalent ppm.
- **Inset**: Zoomed view of first 100 µs showing sub-sample alignment.

#### Plot 4: Power Spectral Density (PSD) Overlay

- **Purpose**: Compare frequency-domain characteristics across devices.
- **X-axis**: Frequency (MHz), relative to center.
- **Y-axis**: PSD (dB/Hz).
- **Content**: Welch's method (`scipy.signal.welch`) with appropriate NFFT (e.g., 4096). Overlay all devices.
- **Annotations**: Center frequency label, bandwidth markers.

#### Plot 5: Magnitude-Squared Coherence

- **Purpose**: Quantify frequency-domain correlation between devices.
- **X-axis**: Frequency (MHz), relative to center.
- **Y-axis**: Coherence (0 to 1).
- **Content**: `scipy.signal.coherence()` for each pair. High coherence (~1.0) indicates well-synchronized reception of the same signal.
- **Annotations**: Mean coherence value in text box.

#### Plot 6: Sync Summary Figure (Table)

- **Purpose**: Compact text-based summary of all sync metrics as a figure (for inclusion in papers).
- **Content**: Rendered table with:
  - Per-device: serial, actual freq, actual rate, sample count, overflows.
  - Pair-wise: time offset (ns), phase offset (deg), drift (ppm), cross-corr peak lag.
  - Overall: PASS/FAIL status.

#### Plot 7: IQ Constellation Scatter

- **Purpose**: Visualize signal quality and potential IQ imbalance.
- **X-axis**: In-phase (I).
- **Y-axis**: Quadrature (Q).
- **Content**: Scatter plot of first N samples from each device (separate subplots or overlaid with transparency).
- **Annotations**: Device serial label.

#### Plot 8: Phase Difference Histogram

- **Purpose**: Statistical distribution of instantaneous phase difference (before unwrapping).
- **X-axis**: Phase difference (degrees), range −180° to +180°.
- **Y-axis**: Probability density.
- **Content**: Histogram with 100 bins, optionally overlaid Gaussian fit.
- **Annotations**: Mean, standard deviation in text box.

### 7.3 Color Scheme

Use a color-blind-friendly palette. For up to 4 devices:

```python
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7']  # Blue, Red-orange, Green, Pink
DEVICE_LABELS = [f'USRP {i} ({serial})' for i, serial in enumerate(serials)]
```

### 7.4 Figure Numbering

All figures are saved with descriptive filenames:

```
sync_plots/
├── fig1_time_domain.pdf
├── fig2_cross_correlation.pdf
├── fig3_phase_difference.pdf
├── fig4_psd_overlay.pdf
├── fig5_coherence.pdf
├── fig6_sync_summary.pdf
├── fig7_iq_constellation.pdf
└── fig8_phase_histogram.pdf
```

---

## 8. Implementation Phases

### Phase 1: C++ Skeleton & Multi-Device Setup

**Tasks:**

1. Create `examples/sync_validation_test.cpp` with Boost.ProgramOptions CLI.
2. Implement serial parsing (comma-separated string → vector of strings).
3. Build `multi_usrp` with multi-serial device address.
4. Per-mboard configuration: clock source, time source, freq, rate, gain.
5. PPS detection loop with timeout and error message.
6. `set_time_unknown_pps()` synchronization.
7. Time sync verification (compare `get_time_last_pps()` across boards).

**Deliverables:**
- Compiles and links against UHD + Boost.
- Opens N USRPs, syncs them via PPS, prints sync status, exits.

### Phase 2: Streaming & Binary Capture

**Tasks:**

1. Create output directory (use `<filesystem>` or `boost::filesystem`).
2. Build single `rx_streamer` with N channels (one per device).
3. Allocate per-channel receive buffers.
4. Open per-device binary output files.
5. Timed stream command (all devices start at same future time).
6. Receive loop:
   - `recv()` into multi-channel buffer.
   - Write each channel to its file.
   - Track metadata: first timestamp, sample count, overflows.
   - Progress printing.
7. SIGINT handler for early termination.
8. Close files and print summary.

**Deliverables:**
- Captures samples from N USRPs simultaneously.
- Writes one `.bin` file per device.
- Reports overflows and errors.

### Phase 3: Metadata & Post-Capture Checks

**Tasks:**

1. Implement metadata JSON output (use string formatting — no JSON library dependency needed for simple flat output, or use nlohmann/json if already available in UHD build).
2. Post-capture PPS drift measurement (5-second observation).
3. First-packet timestamp comparison and logging.
4. Write `metadata.json` to output directory.

**Deliverables:**
- Complete `metadata.json` written alongside binary files.
- Drift measurement reported.

### Phase 4: Python Script — Data Loading & Basic Plots

**Tasks:**

1. Create `examples/python/plot_sync_validation.py`.
2. Implement `metadata.json` parser.
3. Implement binary file loader:
   - `sc16`: `np.fromfile(dtype=np.int16)` → reshape → complex conversion.
   - `fc32`: `np.fromfile(dtype=np.complex64)`.
4. Implement Plot 1 (time-domain overlay).
5. Implement Plot 4 (PSD overlay).
6. Implement Plot 7 (IQ constellation).

**Deliverables:**
- Script runs, loads data, generates 3 basic figures.

### Phase 5: Python Script — Sync Analysis Plots

**Tasks:**

1. Implement Plot 2 (cross-correlation) with peak finding.
2. Implement Plot 3 (phase difference vs. time) with linear fit.
3. Implement Plot 5 (coherence).
4. Implement Plot 8 (phase difference histogram).
5. Implement Plot 6 (sync summary table figure).

**Deliverables:**
- All 8 plots generated.
- Publication-quality output.

### Phase 6: Build Integration & Polish

**Tasks:**

1. Add `sync_validation_test.cpp` to `examples/CMakeLists.txt`.
2. Test build with CMake.
3. Test with actual hardware (2 USRPs + PPS).
4. Error handling polish: clear messages for common failures (no PPS, wrong serial, USB bandwidth).
5. README comments in source files.

**Deliverables:**
- Tool builds as part of UHD examples.
- End-to-end tested.

---

## 9. Build Integration

### 9.1 CMakeLists.txt Change

Add one line to `examples/CMakeLists.txt` in the `example_sources` list:

```cmake
set(example_sources
    benchmark_rate.cpp
    # ... existing entries ...
    sync_validation_test.cpp    # <-- ADD THIS
)
```

No additional dependencies beyond what UHD examples already link (`uhd`, `Boost::program_options`, `Boost::filesystem`).

### 9.2 Python Script Placement

Place the Python script at:

```
examples/python/plot_sync_validation.py
```

This follows the existing convention (`examples/python/` already exists).

---

## 10. Usage Examples

### 10.1 Basic Two-Device Validation

```bash
# Step 1: Capture 1 second of data at 915 MHz, 10 MS/s
./sync_validation_test \
    --serials "30B56D6,30DBC3C" \
    --freq 915e6 \
    --rate 10e6 \
    --gain 30 \
    --duration 1.0

# Step 2: Generate plots
python3 plot_sync_validation.py --input-dir ./sync_capture
```

### 10.2 Three-Device Validation with External 10 MHz

```bash
./sync_validation_test \
    --serials "30B56D6,30DBC3C,30EAB12" \
    --freq 2.4e9 \
    --rate 20e6 \
    --gain 40 \
    --duration 2.0 \
    --clock-source external \
    --output-dir ./captures/test_3dev

python3 plot_sync_validation.py \
    --input-dir ./captures/test_3dev \
    --output-dir ./captures/test_3dev/plots \
    --format png --dpi 600
```

### 10.3 Quick Sync Sanity Check (No Files Needed)

```bash
# Minimal capture — just check timestamps and drift
./sync_validation_test \
    --serials "30B56D6,30DBC3C" \
    --freq 915e6 \
    --rate 1e6 \
    --duration 0.1
```

### 10.4 High-Rate Capture for Drift Analysis

```bash
# Longer capture to see drift accumulate
./sync_validation_test \
    --serials "30B56D6,30DBC3C" \
    --freq 915e6 \
    --rate 10e6 \
    --duration 10.0 \
    --format fc32 \
    --output-dir ./drift_test
```

---

## Appendix A: File Structure After Implementation

```
examples/
├── sync_validation_test.cpp          # C++ capture tool (NEW)
├── CMakeLists.txt                    # Modified: add sync_validation_test
├── python/
│   └── plot_sync_validation.py       # Python plotting script (NEW)
└── ... (existing examples unchanged)
```

## Appendix B: Key UHD API Calls Used

| API Call | Purpose |
|----------|---------|
| `multi_usrp::make("serial=A,serial1=B")` | Open multiple USRPs as unified device |
| `set_clock_source("internal", mb)` | Use internal TCXO (PPS-only mode) |
| `set_clock_source("external", mb)` | Use external 10 MHz (full sync mode) |
| `set_time_source("external", mb)` | Accept PPS on SMA input |
| `set_time_unknown_pps(time_spec_t(0.0))` | Sync all device clocks at next PPS edge |
| `get_time_last_pps(mb)` | Read timestamp of last PPS edge (for drift measurement) |
| `get_rx_stream(stream_args)` | Create multi-channel streamer |
| `rx_stream->recv(buffs, nsamps, md)` | Receive aligned samples from all channels |
| `rx_metadata_t.time_spec` | Hardware timestamp of received samples |
| `stream_cmd_t` with `stream_now=false` | Schedule synchronized stream start |

## Appendix C: Validation Metrics & Interpretation

| Metric | Good (10 MHz + PPS) | Acceptable (PPS-Only) | Bad |
|--------|---------------------|----------------------|-----|
| Time delta at PPS | < 10 ns | < 100 ns | > 1 µs |
| Cross-corr peak lag | 0 samples | 0-1 samples | > 2 samples |
| Phase drift rate | < 0.01 Hz | < 5 kHz | > 10 kHz |
| Drift (ppm) | < 0.001 | < 3 | > 5 |
| Coherence (mean) | > 0.99 | > 0.8 | < 0.5 |
| Overflows | 0 | 0 | > 0 |
