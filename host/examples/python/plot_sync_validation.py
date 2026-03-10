#!/usr/bin/env python3
#
# Copyright 2026 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
"""Post-processing and visualization script for sync_validation_test captures.

Reads the binary IQ dump files and metadata.json produced by the
sync_validation_test C++ tool, then generates up to 8 publication-quality
figures suitable for academic research:

  fig1_time_domain.{pdf|png|svg}      – Time-domain waveform overlay
  fig2_cross_correlation.{pdf|png|svg} – Normalized cross-correlation
  fig3_phase_difference.{pdf|png|svg}  – Unwrapped phase difference vs. time
  fig4_psd_overlay.{pdf|png|svg}       – Power spectral density overlay
  fig5_coherence.{pdf|png|svg}         – Magnitude-squared coherence
  fig6_sync_summary.{pdf|png|svg}      – Tabular sync-metric summary
  fig7_iq_constellation.{pdf|png|svg}  – IQ scatter / constellation
  fig8_phase_histogram.{pdf|png|svg}   – Phase-difference histogram

Example usage:
  python3 plot_sync_validation.py --input-dir ./sync_capture
  python3 plot_sync_validation.py --input-dir ./sync_capture \\
      --output-dir ./sync_plots --format png --dpi 300 --no-show
"""

import argparse
import json
import os
import sys
import warnings

import numpy as np

# ---------------------------------------------------------------------------
# Optional imports – provide clear installation hints when missing
# ---------------------------------------------------------------------------
try:
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import matplotlib.ticker as mticker
    from matplotlib.gridspec import GridSpec
    from matplotlib.patches import FancyBboxPatch
except ImportError:
    sys.exit(
        "matplotlib is required.  Install with:\n"
        "  pip install matplotlib"
    )

try:
    from scipy import signal as sp_signal
    from scipy.optimize import curve_fit
    from scipy.stats import norm as sp_norm
except ImportError:
    sys.exit(
        "scipy is required.  Install with:\n"
        "  pip install scipy"
    )

# ---------------------------------------------------------------------------
# Publication-quality style
# ---------------------------------------------------------------------------
# Use a neutral, paper-friendly style – fall back gracefully on older
# matplotlib versions that do not have the v0_8 suffix.
for _style in ("seaborn-v0_8-paper", "seaborn-paper", "default"):
    try:
        plt.style.use(_style)
        break
    except OSError:
        pass

mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "DejaVu Serif", "Computer Modern Roman"],
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 12,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "figure.figsize": (8, 5),
        "figure.dpi": 150,
        "savefig.dpi": 300,
        "savefig.bbox": "tight",
        "savefig.pad_inches": 0.05,
        "lines.linewidth": 1.0,
        "lines.markersize": 4,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "grid.linestyle": "--",
        "legend.framealpha": 0.8,
        "legend.edgecolor": "0.8",
        "text.usetex": False,
        "mathtext.fontset": "cm",
    }
)

# Color-blind-friendly palette (up to 8 devices)
COLORS = [
    "#0072B2",  # blue
    "#D55E00",  # red-orange
    "#009E73",  # green
    "#CC79A7",  # pink
    "#E69F00",  # orange
    "#56B4E9",  # sky-blue
    "#F0E442",  # yellow
    "#000000",  # black
]

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--input-dir",
        default="./sync_capture",
        metavar="DIR",
        help="Directory containing .bin files and metadata.json "
             "(default: ./sync_capture)",
    )
    p.add_argument(
        "--output-dir",
        default=None,
        metavar="DIR",
        help="Directory for saved figures (default: <input-dir>/plots)",
    )
    p.add_argument(
        "--format",
        choices=["pdf", "png", "svg"],
        default="pdf",
        help="Output file format (default: pdf)",
    )
    p.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="DPI for raster outputs (default: 300)",
    )
    p.add_argument(
        "--max-samples",
        type=int,
        default=100_000,
        metavar="N",
        help="Max samples loaded per device for the analysis plots "
             "(default: 100000). Reduces memory for long captures.",
    )
    p.add_argument(
        "--no-show",
        action="store_true",
        help="Save figures without displaying them interactively.",
    )
    p.add_argument(
        "--figsize",
        default="8,5",
        metavar="W,H",
        help="Default figure size in inches, comma-separated (default: 8,5)",
    )
    args = p.parse_args()

    # Parse figsize
    try:
        w, h = (float(x) for x in args.figsize.split(","))
        args.figsize = (w, h)
    except ValueError:
        p.error("--figsize must be two comma-separated numbers, e.g. 8,5")

    if args.output_dir is None:
        args.output_dir = os.path.join(args.input_dir, "plots")

    return args


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_metadata(input_dir):
    """Load and return the metadata.json from the capture directory."""
    path = os.path.join(input_dir, "metadata.json")
    if not os.path.isfile(path):
        sys.exit(f"[ERROR] metadata.json not found in {input_dir}")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def load_samples(input_dir, metadata, max_samples):
    """Return a list of complex64 arrays, one per device.

    Handles both sc16 (int16 interleaved) and fc32 (complex64) formats.
    Caps at *max_samples* per device.
    """
    fmt = metadata["parameters"]["sample_format"]
    signals = []

    for dev in metadata["devices"]:
        fpath = os.path.join(input_dir, dev["file"])
        if not os.path.isfile(fpath):
            sys.exit(f"[ERROR] Binary file not found: {fpath}")

        if fmt == "sc16":
            raw = np.fromfile(fpath, dtype=np.int16)
            # Interleaved I/Q: [I0, Q0, I1, Q1, ...]
            n_avail = len(raw) // 2
            n = min(n_avail, max_samples)
            raw = raw[: n * 2].reshape(-1, 2)
            sig = raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)
            # Normalize to unit scale (int16 range: -32768 .. 32767)
            sig /= 32768.0
        elif fmt == "fc32":
            raw = np.fromfile(fpath, dtype=np.complex64)
            n = min(len(raw), max_samples)
            sig = raw[:n].astype(np.complex64)
        else:
            sys.exit(f"[ERROR] Unknown sample format in metadata: {fmt!r}")

        signals.append(sig)
        print(
            f"[INFO] Loaded device {dev['index']} ({dev['serial']}): "
            f"{len(sig):,} samples from {dev['file']}"
        )

    return signals


def device_labels(metadata):
    """Return a list of human-readable labels for each device."""
    return [
        f"USRP {d['index']} ({d['serial']})" for d in metadata["devices"]
    ]


def device_pairs(n):
    """Return all unique (i, j) pairs for n devices."""
    return [(i, j) for i in range(n) for j in range(i + 1, n)]


# ---------------------------------------------------------------------------
# Figure helpers
# ---------------------------------------------------------------------------

def _savefig(fig, output_dir, name, fmt, dpi, no_show):
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{name}.{fmt}")
    fig.savefig(path, dpi=dpi)
    print(f"[OUTPUT] Saved {path}")
    if not no_show:
        plt.show()
    plt.close(fig)


def _textbox(ax, text, loc="upper right", fontsize=8):
    """Add a styled text annotation box to an axes."""
    props = dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85,
                 edgecolor="0.7")
    # Map location strings to (x, y, ha, va) in axes coordinates
    locs = {
        "upper right": (0.97, 0.97, "right", "top"),
        "upper left":  (0.03, 0.97, "left",  "top"),
        "lower right": (0.97, 0.03, "right", "bottom"),
        "lower left":  (0.03, 0.03, "left",  "bottom"),
    }
    x, y, ha, va = locs.get(loc, locs["upper right"])
    ax.text(x, y, text, transform=ax.transAxes, fontsize=fontsize,
            verticalalignment=va, horizontalalignment=ha, bbox=props)


# ---------------------------------------------------------------------------
# Plot 1 – Time-domain waveform overlay
# ---------------------------------------------------------------------------

def plot_time_domain(signals, metadata, args):
    """Overlay the real (I) component of each device over a short window."""
    rate = metadata["parameters"]["sample_rate_hz"]
    freq = metadata["parameters"]["center_frequency_hz"]
    labels = device_labels(metadata)
    n_devs = len(signals)

    # Show at most 500 samples (50 µs at 10 MS/s) for readability
    n_show = min(500, min(len(s) for s in signals))
    t_us = np.arange(n_show) / rate * 1e6  # time axis in µs

    fig, axes = plt.subplots(2, 1, figsize=args.figsize, sharex=True)
    fig.suptitle(
        f"Time-Domain Waveform Overlay — f$_c$ = {freq/1e6:.3f} MHz",
        fontsize=12,
    )

    ax_i, ax_q = axes
    for idx, (sig, lbl) in enumerate(zip(signals, labels)):
        c = COLORS[idx % len(COLORS)]
        ax_i.plot(t_us, sig[:n_show].real, color=c, label=lbl, alpha=0.85)
        ax_q.plot(t_us, sig[:n_show].imag, color=c, label=lbl, alpha=0.85)

    ax_i.axvline(0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
    ax_q.axvline(0, color="k", linestyle="--", linewidth=0.8, alpha=0.5)
    ax_i.set_ylabel("I (normalized)")
    ax_q.set_ylabel("Q (normalized)")
    ax_q.set_xlabel("Time (µs)")
    ax_i.legend(loc="upper right", ncol=min(n_devs, 4))
    fig.tight_layout()

    _savefig(fig, args.output_dir, "fig1_time_domain", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 2 – Cross-correlation between device pairs
# ---------------------------------------------------------------------------

def plot_cross_correlation(signals, metadata, args):
    """Normalized cross-correlation magnitude per device pair."""
    rate = metadata["parameters"]["sample_rate_hz"]
    labels = device_labels(metadata)
    pairs = device_pairs(len(signals))

    if not pairs:
        print("[INFO] Only one device – skipping cross-correlation plot.")
        return

    n_pairs = len(pairs)
    fig_h = max(3.0, 2.5 * n_pairs)
    fig, axes = plt.subplots(n_pairs, 1, figsize=(args.figsize[0], fig_h),
                              squeeze=False)
    fig.suptitle("Normalized Cross-Correlation Magnitude", fontsize=12)

    lag_window = 100  # samples either side of centre

    for row, (i, j) in enumerate(pairs):
        ax = axes[row, 0]
        sig_i = signals[i]
        sig_j = signals[j]

        # Use first 10000 samples for speed; normalize energy
        n_cc = min(10_000, len(sig_i), len(sig_j))
        a = sig_i[:n_cc]
        b = sig_j[:n_cc]

        # Full cross-correlation
        cc = np.correlate(a, b, mode="full")
        cc_mag = np.abs(cc)
        energy = np.sqrt(np.sum(np.abs(a) ** 2) * np.sum(np.abs(b) ** 2))
        if energy > 0:
            cc_mag /= energy

        # Lags array: centre of 'full' correlation is at index len(a)-1
        centre = len(a) - 1
        lags = np.arange(-lag_window, lag_window + 1)
        idx_range = centre + lags
        # Clip in case the signal is very short
        mask = (idx_range >= 0) & (idx_range < len(cc_mag))
        lags_plot = lags[mask]
        cc_plot = cc_mag[idx_range[mask]]

        peak_idx = np.argmax(cc_plot)
        peak_lag = lags_plot[peak_idx]
        peak_val = cc_plot[peak_idx]
        peak_us = peak_lag / rate * 1e6

        c_i = COLORS[i % len(COLORS)]
        ax.plot(lags_plot, cc_plot, color=c_i, linewidth=1.0)
        ax.axvline(peak_lag, color="red", linestyle="--", linewidth=0.9, alpha=0.8)
        ax.set_ylabel("Norm. |Xcorr|")
        ax.set_title(
            f"{labels[i]}  vs.  {labels[j]}",
            fontsize=9,
        )
        _textbox(
            ax,
            f"Peak lag: {peak_lag:+d} samp  ({peak_us:+.3f} µs)\n"
            f"|Xcorr|_max = {peak_val:.4f}",
        )

    axes[-1, 0].set_xlabel("Lag (samples)")
    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig2_cross_correlation", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 3 – Instantaneous phase difference vs. time
# ---------------------------------------------------------------------------

def plot_phase_difference(signals, metadata, args):
    """Unwrapped phase difference between all device pairs."""
    rate = metadata["parameters"]["sample_rate_hz"]
    labels = device_labels(metadata)
    pairs = device_pairs(len(signals))

    if not pairs:
        print("[INFO] Only one device – skipping phase-difference plot.")
        return

    n_pairs = len(pairs)
    fig_h = max(3.0, 2.8 * n_pairs)
    fig, axes = plt.subplots(n_pairs, 1, figsize=(args.figsize[0], fig_h),
                              squeeze=False)
    fig.suptitle("Unwrapped Phase Difference vs. Time", fontsize=12)

    for row, (i, j) in enumerate(pairs):
        ax = axes[row, 0]
        n = min(len(signals[i]), len(signals[j]))
        t_ms = np.arange(n) / rate * 1e3  # time in ms

        sig_i = signals[i][:n]
        sig_j = signals[j][:n]

        # Instantaneous phase difference, unwrapped
        phase_diff = np.angle(sig_i * np.conj(sig_j))
        phase_unwrapped = np.unwrap(phase_diff)

        # Linear fit: slope gives frequency offset in rad/s → Hz
        coeffs = np.polyfit(t_ms * 1e-3, phase_unwrapped, 1)  # rad/s slope
        freq_offset_hz = coeffs[0] / (2 * np.pi)
        phase_trend = np.polyval(coeffs, t_ms * 1e-3)

        c_i = COLORS[i % len(COLORS)]
        ax.plot(t_ms, np.degrees(phase_unwrapped),
                color=c_i, linewidth=0.8, alpha=0.9, label="Meas.")
        ax.plot(t_ms, np.degrees(phase_trend),
                color="red", linestyle="--", linewidth=1.0, label="Linear fit")

        # Inset: first 100 µs
        if n > 100:
            axins = ax.inset_axes([0.55, 0.05, 0.42, 0.38])
            n_ins = min(int(100e-6 * rate), n)
            t_ins = np.arange(n_ins) / rate * 1e6
            axins.plot(t_ins, np.degrees(phase_unwrapped[:n_ins]),
                       color=c_i, linewidth=0.8)
            axins.set_xlabel("µs", fontsize=7)
            axins.set_ylabel("deg", fontsize=7)
            axins.tick_params(labelsize=7)
            axins.grid(True, alpha=0.3, linestyle="--")

        ax.set_ylabel("Phase diff (deg)")
        ax.set_title(f"{labels[i]}  –  {labels[j]}", fontsize=9)
        ax.legend(fontsize=8, loc="upper left")

        center_freq = metadata["parameters"]["center_frequency_hz"]
        freq_ppm = (freq_offset_hz / center_freq) * 1e6 if center_freq > 0 else 0.0
        _textbox(
            ax,
            f"Freq offset: {freq_offset_hz:+.2f} Hz\n"
            f"({freq_ppm:+.3f} ppm)",
        )

    axes[-1, 0].set_xlabel("Time (ms)")
    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig3_phase_difference", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 4 – Power Spectral Density overlay
# ---------------------------------------------------------------------------

def plot_psd(signals, metadata, args):
    """Welch PSD overlay for all devices."""
    rate = metadata["parameters"]["sample_rate_hz"]
    freq = metadata["parameters"]["center_frequency_hz"]
    labels = device_labels(metadata)

    nfft = 4096
    fig, ax = plt.subplots(figsize=args.figsize)
    ax.set_title(
        f"Power Spectral Density — f$_c$ = {freq/1e6:.3f} MHz",
        fontsize=12,
    )

    for idx, (sig, lbl) in enumerate(zip(signals, labels)):
        f_welch, pxx = sp_signal.welch(
            sig,
            fs=rate,
            nperseg=nfft,
            return_onesided=False,
            scaling="density",
        )
        # Shift to DC-centred
        f_welch = np.fft.fftshift(f_welch)
        pxx = np.fft.fftshift(pxx)
        pxx_db = 10 * np.log10(np.maximum(pxx, 1e-20))

        c = COLORS[idx % len(COLORS)]
        ax.plot(f_welch / 1e6, pxx_db, color=c, label=lbl, alpha=0.85, linewidth=0.9)

    ax.set_xlabel("Frequency offset (MHz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.axvline(0, color="k", linestyle=":", linewidth=0.7, alpha=0.5)

    # Bandwidth markers (±rate/2)
    bw_mhz = rate / 2e6
    ax.axvline(-bw_mhz, color="gray", linestyle="--", linewidth=0.7, alpha=0.5)
    ax.axvline( bw_mhz, color="gray", linestyle="--", linewidth=0.7, alpha=0.5,
                label=f"BW = {rate/1e6:.1f} MS/s")

    ax.legend(loc="upper right")
    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig4_psd_overlay", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 5 – Magnitude-squared coherence
# ---------------------------------------------------------------------------

def plot_coherence(signals, metadata, args):
    """Magnitude-squared coherence for all device pairs."""
    rate = metadata["parameters"]["sample_rate_hz"]
    labels = device_labels(metadata)
    pairs = device_pairs(len(signals))

    if not pairs:
        print("[INFO] Only one device – skipping coherence plot.")
        return

    n_pairs = len(pairs)
    fig_h = max(3.0, 2.5 * n_pairs)
    fig, axes = plt.subplots(n_pairs, 1, figsize=(args.figsize[0], fig_h),
                              squeeze=False)
    fig.suptitle("Magnitude-Squared Coherence", fontsize=12)

    for row, (i, j) in enumerate(pairs):
        ax = axes[row, 0]
        n = min(len(signals[i]), len(signals[j]))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # suppress onesided/complex warning
            f_coh, cxy = sp_signal.coherence(
                signals[i][:n],
                signals[j][:n],
                fs=rate,
                nperseg=1024,
            )
        # fftshift converts the [0, fs) layout to (-fs/2, fs/2] for display
        f_coh = np.fft.fftshift(f_coh)
        cxy = np.fft.fftshift(cxy)
        # Map the positive-only frequencies (one-sided case) to centred display
        if f_coh[-1] > rate / 2 * 1.01:
            # truly two-sided: fftshift already centred it correctly
            pass
        else:
            # one-sided: shift so DC is in the middle (mirror the spectrum)
            f_coh = f_coh - rate / 2

        mean_coh = float(np.mean(cxy))
        c_i = COLORS[i % len(COLORS)]
        ax.plot(f_coh / 1e6, cxy, color=c_i, linewidth=0.8)
        ax.set_ylim(0, 1.05)
        ax.set_ylabel("Coherence")
        ax.set_title(f"{labels[i]}  vs.  {labels[j]}", fontsize=9)
        _textbox(ax, f"Mean coherence = {mean_coh:.4f}")

    axes[-1, 0].set_xlabel("Frequency offset (MHz)")
    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig5_coherence", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 6 – Sync summary table figure
# ---------------------------------------------------------------------------

def plot_sync_summary(signals, metadata, args):
    """Render a tabular sync-metric summary as a figure."""
    params = metadata["parameters"]
    sync = metadata.get("sync_results", {})
    devs = metadata["devices"]
    n_devs = len(devs)
    pairs = device_pairs(n_devs)

    # Collect per-device rows
    dev_header = ["Serial", "Mboard", "Freq (MHz)", "Rate (MS/s)",
                  "Gain (dB)", "Samples", "Overflows"]
    dev_rows = []
    for d in devs:
        dev_rows.append([
            d["serial"],
            d.get("mboard_name", "—"),
            f"{d.get('actual_frequency_hz', 0)/1e6:.4f}",
            f"{d.get('actual_rate_hz', 0)/1e6:.4f}",
            f"{d.get('actual_gain_db', 0):.1f}",
            f"{d.get('total_samples', 0):,}",
            str(d.get("overflow_count", 0)),
        ])

    # Collect pairwise rows
    pair_header = ["Pair", "PPS delta (µs)", "Timestamp offset (ticks)",
                   "Drift (ppm)"]
    pps_deltas = sync.get("pps_time_deltas_us", [0.0] * n_devs)
    ts_align = sync.get("timestamp_alignment_ticks", [0] * n_devs)
    drift_ppm = sync.get("drift_ppm", [0.0] * n_devs)

    pair_rows = []
    for i, j in pairs:
        pair_rows.append([
            f"Dev{i} vs Dev{j}",
            f"{pps_deltas[j]:.4f}" if j < len(pps_deltas) else "—",
            str(ts_align[j]) if j < len(ts_align) else "—",
            f"{drift_ppm[j]:+.4f}" if j < len(drift_ppm) else "—",
        ])

    # Overall status row
    total_of = sum(d.get("overflow_count", 0) for d in devs)
    max_pps_delta = max((abs(x) for x in pps_deltas[1:]), default=0.0)
    status = "PASS" if total_of == 0 and max_pps_delta < 1.0 else "WARNING"

    # --- Build figure with a plain axes used as a canvas ------------------
    fig_h = 1.2 + 0.35 * (len(dev_rows) + len(pair_rows) + 6)
    fig = plt.figure(figsize=(args.figsize[0], max(5.0, fig_h)))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.axis("off")

    title_text = (
        "Sync Validation Summary\n"
        f"Capture: {metadata.get('capture_timestamp_utc','—')}  |  "
        f"Tool: {metadata.get('capture_tool','—')} v{metadata.get('capture_version','—')}\n"
        f"Format: {params.get('sample_format','—')}  |  "
        f"Duration: {params.get('duration_s', 0):.3f} s  |  "
        f"Status: {status}"
    )
    ax.text(0.5, 0.97, title_text, transform=ax.transAxes,
            ha="center", va="top", fontsize=10,
            fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#E8F4FD",
                      edgecolor="#2196F3", linewidth=1.2))

    # Helper to draw a table inside the axes
    def _draw_table(ax, header, rows, y_top, col_widths=None):
        n_cols = len(header)
        if col_widths is None:
            col_widths = [1.0 / n_cols] * n_cols
        row_h = 0.045
        x_starts = [sum(col_widths[:k]) for k in range(n_cols)]

        # Header row
        for ci, (hdr, xs, cw) in enumerate(zip(header, x_starts, col_widths)):
            ax.text(xs + cw / 2, y_top, hdr,
                    transform=ax.transAxes,
                    ha="center", va="top",
                    fontsize=8, fontweight="bold",
                    color="white",
                    bbox=dict(boxstyle="square,pad=0.1",
                              facecolor="#2196F3", linewidth=0))

        # Data rows
        for ri, row in enumerate(rows):
            bg = "#F5F5F5" if ri % 2 == 0 else "white"
            y = y_top - (ri + 1) * row_h
            for ci, (cell, xs, cw) in enumerate(zip(row, x_starts, col_widths)):
                ax.text(xs + cw / 2, y, cell,
                        transform=ax.transAxes,
                        ha="center", va="top",
                        fontsize=8,
                        bbox=dict(boxstyle="square,pad=0.1",
                                  facecolor=bg, linewidth=0))
        return y_top - (len(rows) + 1) * row_h

    y = 0.84
    ax.text(0.02, y, "Per-Device Metrics",
            transform=ax.transAxes, fontsize=9, fontweight="bold", va="top")
    y -= 0.02

    dev_col_w = [0.13, 0.12, 0.14, 0.14, 0.12, 0.20, 0.15]
    y = _draw_table(ax, dev_header, dev_rows, y, col_widths=dev_col_w)

    if pair_rows:
        y -= 0.04
        ax.text(0.02, y, "Pairwise Sync Metrics",
                transform=ax.transAxes, fontsize=9, fontweight="bold", va="top")
        y -= 0.02
        pair_col_w = [0.20, 0.25, 0.30, 0.25]
        y = _draw_table(ax, pair_header, pair_rows, y, col_widths=pair_col_w)

    # Capture parameters box
    y -= 0.05
    param_text = (
        f"  Center freq:  {params.get('center_frequency_hz',0)/1e6:.4f} MHz\n"
        f"  Sample rate:  {params.get('sample_rate_hz',0)/1e6:.4f} MS/s\n"
        f"  Bandwidth:    {params.get('bandwidth_hz',0)/1e6:.4f} MHz\n"
        f"  Gain:         {params.get('gain_db',0):.1f} dB\n"
        f"  Clock source: {params.get('clock_source','—')}\n"
        f"  Time source:  {params.get('time_source','—')}\n"
        f"  Drift check:  {'Yes' if sync.get('drift_check_performed') else 'No'}  "
        f"({sync.get('drift_measurement_duration_s',0):.0f} s observation)"
    )
    ax.text(0.02, y, "Capture Parameters", transform=ax.transAxes,
            fontsize=9, fontweight="bold", va="top")
    ax.text(0.02, y - 0.025, param_text, transform=ax.transAxes,
            fontsize=8, va="top", fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#F9F9F9",
                      edgecolor="0.7", linewidth=0.8))

    _savefig(fig, args.output_dir, "fig6_sync_summary", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 7 – IQ Constellation scatter
# ---------------------------------------------------------------------------

def plot_iq_constellation(signals, metadata, args):
    """IQ scatter plot per device, arranged in a grid."""
    n_devs = len(signals)
    labels = device_labels(metadata)

    # Layout: up to 2 plots per row
    n_cols = min(2, n_devs)
    n_rows = (n_devs + n_cols - 1) // n_cols
    fig_w = args.figsize[0]
    fig_h = max(3.5, 3.5 * n_rows)
    fig, axes = plt.subplots(n_rows, n_cols,
                             figsize=(fig_w, fig_h),
                             squeeze=False)
    fig.suptitle("IQ Constellation", fontsize=12)

    n_scatter = min(5_000, min(len(s) for s in signals))

    for idx, (sig, lbl) in enumerate(zip(signals, labels)):
        row, col = divmod(idx, n_cols)
        ax = axes[row][col]
        c = COLORS[idx % len(COLORS)]
        ax.scatter(sig[:n_scatter].real, sig[:n_scatter].imag,
                   s=1, alpha=0.3, color=c, rasterized=True)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("I")
        ax.set_ylabel("Q")
        ax.set_title(lbl, fontsize=9)

    # Hide unused subplots
    for idx in range(n_devs, n_rows * n_cols):
        row, col = divmod(idx, n_cols)
        axes[row][col].set_visible(False)

    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig7_iq_constellation", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Plot 8 – Phase-difference histogram
# ---------------------------------------------------------------------------

def plot_phase_histogram(signals, metadata, args):
    """Histogram of instantaneous phase differences (wrapped to ±180°)."""
    labels = device_labels(metadata)
    pairs = device_pairs(len(signals))

    if not pairs:
        print("[INFO] Only one device – skipping phase histogram.")
        return

    n_pairs = len(pairs)
    n_cols = min(2, n_pairs)
    n_rows = (n_pairs + n_cols - 1) // n_cols
    fig_h = max(3.5, 3.5 * n_rows)
    fig, axes = plt.subplots(n_rows, n_cols,
                             figsize=(args.figsize[0], fig_h),
                             squeeze=False)
    fig.suptitle("Phase-Difference Histogram", fontsize=12)

    for pidx, (i, j) in enumerate(pairs):
        row, col = divmod(pidx, n_cols)
        ax = axes[row][col]
        n = min(len(signals[i]), len(signals[j]))
        phase_diff_deg = np.degrees(
            np.angle(signals[i][:n] * np.conj(signals[j][:n]))
        )

        bins = np.linspace(-180, 180, 101)
        counts, bin_edges = np.histogram(phase_diff_deg, bins=bins, density=True)
        bin_centres = 0.5 * (bin_edges[:-1] + bin_edges[1:])
        c_i = COLORS[i % len(COLORS)]
        ax.bar(bin_centres, counts, width=bins[1] - bins[0],
               color=c_i, alpha=0.65, linewidth=0)

        # Optional Gaussian fit
        mu = float(np.mean(phase_diff_deg))
        sigma = float(np.std(phase_diff_deg))
        if sigma > 0:
            x_fit = np.linspace(-180, 180, 300)
            try:
                popt, _ = curve_fit(
                    lambda x, a, m, s: a * np.exp(-0.5 * ((x - m) / s) ** 2),
                    bin_centres,
                    counts,
                    p0=[counts.max(), mu, sigma],
                    maxfev=2000,
                )
                ax.plot(x_fit,
                        popt[0] * np.exp(-0.5 * ((x_fit - popt[1]) / popt[2]) ** 2),
                        "r--", linewidth=1.2, label="Gaussian fit")
            except RuntimeError:
                pass  # curve_fit did not converge – skip overlay

        ax.set_xlim(-180, 180)
        ax.set_xlabel("Phase difference (deg)")
        ax.set_ylabel("Probability density")
        ax.set_title(f"{labels[i]}  –  {labels[j]}", fontsize=9)
        if sigma > 0:
            ax.legend(fontsize=8)
        _textbox(
            ax,
            f"µ = {mu:.2f}°\nσ = {sigma:.2f}°",
            loc="upper left",
        )

    # Hide unused subplots
    for pidx in range(n_pairs, n_rows * n_cols):
        row, col = divmod(pidx, n_cols)
        axes[row][col].set_visible(False)

    fig.tight_layout()
    _savefig(fig, args.output_dir, "fig8_phase_histogram", args.format, args.dpi, args.no_show)


# ---------------------------------------------------------------------------
# Main entry-point
# ---------------------------------------------------------------------------

def main():
    args = parse_args()

    print("\n=== USRP Sync Validation – Post-Processing ===")
    print(f"[INFO] Input dir:  {args.input_dir}")
    print(f"[INFO] Output dir: {args.output_dir}")
    print(f"[INFO] Format:     {args.format}  (DPI={args.dpi})")

    # ------------------------------------------------------------------
    # 1. Load metadata
    # ------------------------------------------------------------------
    metadata = load_metadata(args.input_dir)
    n_devs = len(metadata["devices"])
    print(
        f"[INFO] Metadata loaded: {n_devs} device(s), "
        f"format={metadata['parameters']['sample_format']}, "
        f"rate={metadata['parameters']['sample_rate_hz']/1e6:.3f} MS/s, "
        f"duration={metadata['parameters']['duration_s']:.3f} s"
    )

    # ------------------------------------------------------------------
    # 2. Load binary files
    # ------------------------------------------------------------------
    signals = load_samples(args.input_dir, metadata, args.max_samples)

    # Validate sample counts
    for idx, (sig, dev) in enumerate(zip(signals, metadata["devices"])):
        expected = min(dev.get("total_samples", len(sig)), args.max_samples)
        if len(sig) != expected:
            warnings.warn(
                f"Device {idx}: loaded {len(sig)} samples, "
                f"expected {expected}"
            )

    if n_devs < 1:
        sys.exit("[ERROR] No devices found in metadata.")

    # ------------------------------------------------------------------
    # 3. Generate all figures
    # ------------------------------------------------------------------
    os.makedirs(args.output_dir, exist_ok=True)

    print("\n[INFO] Generating figures...")

    plot_time_domain(signals, metadata, args)
    plot_cross_correlation(signals, metadata, args)
    plot_phase_difference(signals, metadata, args)
    plot_psd(signals, metadata, args)
    plot_coherence(signals, metadata, args)
    plot_sync_summary(signals, metadata, args)
    plot_iq_constellation(signals, metadata, args)
    plot_phase_histogram(signals, metadata, args)

    print(f"\n=== Done. All figures saved to {args.output_dir}/ ===")


if __name__ == "__main__":
    main()
