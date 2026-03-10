# USRP Synchronization Validation Tools

Two tools for validating time and phase synchronization across multiple USRP
devices sharing an external PPS reference.

- `sync_validation_test` -- C++ capture tool (built with UHD examples)
- `python/plot_sync_validation.py` -- Python post-processing and plotting script

---

## sync_validation_test

Captures raw IQ samples from N USRPs simultaneously, synchronizing them via an
external PPS signal. Writes one binary file per device and a `metadata.json`
file describing the capture.

### Build

The tool is built automatically as part of the UHD examples:

    cd build
    make sync_validation_test

### Basic usage

    ./examples/sync_validation_test \
        --serials "30B56D6,30DBC3C" \
        --freq 915e6 \
        --rate 10e6 \
        --gain 30 \
        --duration 1.0

Output files are written to `./sync_capture/` by default.

### All options

    --serials       Comma-separated list of USRP serial numbers (required)
    --serial0/1/2/3 Alternative: one serial per flag instead of --serials
    --freq          Center frequency in Hz (default: 915e6)
    --rate          Sample rate in samples/s (default: 1e6)
    --gain          RX gain in dB (default: 0)
    --bw            Analog bandwidth in Hz; 0 = use sample rate (default: 0)
    --duration      Capture duration in seconds (default: 1.0)
    --output-dir    Directory for output files (default: ./sync_capture)
    --format        Sample format: sc16 or fc32 (default: sc16)
    --clock-source  Clock source: internal or external 10 MHz (default: internal)
    --ref           PPS/time reference source (default: external)
    --subdev        RX subdevice spec applied to all devices, e.g. "A:A"
    --channel       RX channel index per device (default: 0)
    --antenna       Antenna port, e.g. RX2 or TX/RX
    --spb           Samples per recv() buffer (default: 10000)
    --skip-drift-check  Skip the 5-second post-capture PPS drift measurement

### Output files

    sync_capture/
    |-- usrp_0_30B56D6.bin    raw IQ samples, device 0
    |-- usrp_1_30DBC3C.bin    raw IQ samples, device 1
    `-- metadata.json          capture parameters and sync results

Binary format:

- `sc16`: interleaved signed 16-bit integers [I0, Q0, I1, Q1, ...], 4 bytes/sample
- `fc32`: interleaved 32-bit floats [I0, Q0, I1, Q1, ...], 8 bytes/sample

### Example: two devices, external 10 MHz clock, 2-second capture

    ./examples/sync_validation_test \
        --serials "30B56D6,30DBC3C" \
        --freq 915e6 \
        --rate 10e6 \
        --gain 30 \
        --duration 2.0 \
        --clock-source external \
        --format fc32 \
        --output-dir ./my_capture

### Notes

- A PPS signal must be connected to all devices before running the tool.
  The tool aborts with a clear error if no PPS edge is detected within 2 seconds.
- `--clock-source internal` (default): each device uses its own TCXO; only the
  PPS edge is shared. Phase coherence is not guaranteed but timing alignment is.
- `--clock-source external`: all devices share a 10 MHz reference. Full phase
  coherence is possible.
- Press Ctrl+C to stop an in-progress capture cleanly.

---

## python/plot_sync_validation.py

Reads the binary files and `metadata.json` written by `sync_validation_test`
and generates up to 8 publication-quality figures.

### Dependencies

    pip install numpy matplotlib scipy

Requires numpy >= 1.20, matplotlib >= 3.5, scipy >= 1.7.

### Basic usage

    python3 examples/python/plot_sync_validation.py \
        --input-dir ./sync_capture

Figures are saved to `./sync_capture/plots/` by default.

### All options

    --input-dir     Directory with .bin files and metadata.json
                    (default: ./sync_capture)
    --output-dir    Directory for saved figures
                    (default: <input-dir>/plots)
    --format        Output format: pdf, png, or svg (default: pdf)
    --dpi           DPI for raster outputs (default: 300)
    --max-samples   Max samples loaded per device (default: 100000)
                    Increase to analyse more of a long capture, e.g. 1000000
    --no-show       Save figures without opening an interactive window
    --figsize       Figure size in inches, e.g. 10,6 (default: 8,5)

### Figures generated

**fig1_time_domain** -- Time-domain waveform overlay (I and Q)

  Plots the in-phase (I) and quadrature (Q) components of each device's signal
  over the first ~500 samples on shared time axes. If the devices are properly
  synchronized the waveforms should overlap almost exactly. Any visible
  horizontal shift between traces indicates a timing offset; a constant vertical
  offset indicates a phase offset.

**fig2_cross_correlation** -- Normalized cross-correlation magnitude per device pair

  Computes the cross-correlation between each pair of devices and plots its
  magnitude around lag zero. The peak of the curve shows the sample offset
  between the two streams. A peak at lag 0 means the streams are perfectly
  aligned. A peak at lag +/-1 means one device is ahead or behind by one
  sample. The annotation shows the lag in both samples and microseconds.

**fig3_phase_difference** -- Unwrapped instantaneous phase difference vs. time

  At every sample, computes the angle between the two complex signals and
  unwraps it to remove 2*pi jumps. A flat line near zero means the devices
  are phase-coherent. A linearly growing (or shrinking) line means there is a
  constant frequency offset between the two oscillators; the slope of the
  fitted trend line is reported in Hz and ppm. This chart is the primary
  indicator of whether a shared 10 MHz reference is improving coherence over
  PPS-only synchronization. The inset shows the first 100 us for sub-sample
  detail.

**fig4_psd_overlay** -- Power spectral density overlay (Welch method)

  Overlays the power spectral density of each device using Welch's method.
  The x-axis is frequency offset from the tuned center frequency. Identical
  PSD shapes confirm that both devices are receiving the same signal at the
  same operating point. Differences in noise floor or spectral shape may
  indicate gain mismatch or analog filter differences between units.

**fig5_coherence** -- Magnitude-squared coherence per device pair

  Measures how linearly correlated two signals are at each frequency, on a
  scale of 0 to 1. A value close to 1 across the band means the two devices
  are capturing the same signal coherently. Values near 0 mean the signals
  at that frequency are unrelated (e.g., dominated by independent noise). When
  using a shared 10 MHz reference, coherence should be close to 1 wherever
  signal is present. With PPS-only sync, coherence may degrade over long
  captures as phase drifts.

**fig6_sync_summary** -- Tabular summary of all sync metrics

  A single figure designed for inclusion in a report or paper. It shows per-
  device parameters (actual tuned frequency, sample rate, gain, sample count,
  overflow count) and pairwise metrics (PPS time delta, timestamp offset in
  ticks, frequency drift in ppm). Also lists the full capture parameters.
  Provides a compact pass/fail overview at a glance.

**fig7_iq_constellation** -- IQ scatter plot per device

  Plots the real (I) vs. imaginary (Q) component of each received sample as a
  scatter point. The shape of the constellation reflects the signal being
  received. For a single tone the points form a ring; for noise they fill a
  circle. Comparing the scatter across devices helps spot IQ imbalance,
  DC offset, or amplitude inconsistencies between units.

**fig8_phase_histogram** -- Phase-difference histogram with Gaussian fit

  Histograms the instantaneous phase difference between each device pair,
  wrapped to the range -180 to +180 degrees, and fits a Gaussian curve to it.
  A narrow, well-centred distribution means the devices are tightly phase-
  aligned. A wide distribution indicates phase noise or a large frequency
  offset. The mean (mu) quantifies the static phase offset between the two
  devices; the standard deviation (sigma) quantifies the instantaneous phase
  noise.

### Example: process a capture and save PNG figures

    python3 examples/python/plot_sync_validation.py \
        --input-dir ./sync_capture \
        --output-dir ./sync_capture/plots \
        --format png \
        --no-show

### Example: higher-resolution PDF with more samples analysed

    python3 examples/python/plot_sync_validation.py \
        --input-dir ./my_capture \
        --output-dir ./my_capture/plots \
        --format pdf \
        --dpi 300 \
        --max-samples 1000000

---

## Typical end-to-end workflow

Step 1: capture

    cd ~/uhd/host/build
    ./examples/sync_validation_test \
        --serials "30B56D6,30DBC3C" \
        --freq 915e6 \
        --rate 10e6 \
        --gain 30 \
        --duration 1.0

Step 2: generate plots

    python3 ../examples/python/plot_sync_validation.py \
        --input-dir ./sync_capture \
        --output-dir ./sync_capture/plots \
        --format png \
        --no-show

Step 3: inspect results

    ls ./sync_capture/plots/
