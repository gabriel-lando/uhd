# Bonding Analysis and Visualization How-To

This guide explains how to use the Python analysis toolkit to visualize synchronization quality and evaluate clock drift across bonded USRP devices. This corresponds to Phase 2 of the bonding implementation plan.

## Prerequisites

Before running the analysis scripts, ensure you have captured data using the tools from Phase 1.

- `sync_validation_capture` should have produced a directory with `metadata.json` and binary IQ data (`usrp_0_XYZ.bin`).
- `drift_monitor` should have produced a CSV file, e.g., `drift_log.csv`.

Additionally, you will need the following Python packages installed:

```bash
pip install numpy scipy matplotlib pandas seaborn
```

## Running the Synchronization Analysis

The main analysis script `sync_analysis.py` parses the capture data and generates Figures 1 through 8. These figures include time-domain overlays, cross-correlation estimates, power spectral densities, spectral coherence, and a summary dashboard.

To run the script:

```bash
python3 examples/python/sync_analysis.py \
    --input-dir ./sync_capture \
    --output-dir ./sync_capture/plots \
    --format png \
    --dpi 300
```

_Note: The script automatically generates both `.png` and `.pdf` graphics at 300 dpi for academic publications, using colorblind-friendly color palettes and standard label formatting._

### Expected Plots (Figures 1 to 8)

1. **fig01_time_domain_overlay**: Time-domain I/Q traces overlaid.
2. **fig02_cross_correlation**: A spike representing the time-delay gap between devices.
3. **fig03_phase_difference**: Tracks phase difference stability over time.
4. **fig04_psd_overlay**: Verifies power levels spectrum shape matching.
5. **fig05_coherence**: Validates signal similarity across frequency.
6. **fig06_sync_dashboard**: Compact multi-panel view of key synchronization metrics.
7. **fig07_constellation**: Displays IQ clouds matching typical receiver structures.
8. **fig08_phase_histogram**: Characterizes residual phase jitter mathematically via standard deviation.

## Running the Drift Analysis

The drift analysis script processes the timestamp log data accumulated by `drift_monitor`.

To run the script and generate Figure 9:

```bash
python3 examples/python/drift_analysis.py \
    --input-csv ./drift_log.csv \
    --output-dir ./drift_plots
```

### Expected Plot (Figure 9)

**fig09_drift_rate**: Long-term clock drift between devices measured over hours. Plots drift in parts-per-million (ppm), facilitating decisions over mandatory internal clock synchronization tracking loops against simple external 10MHz reference.
