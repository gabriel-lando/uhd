#!/usr/bin/env python3
import argparse
import json
import os
import warnings
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.stats import norm
from bonding_plot_utils import setup_academic_style, save_plot

def load_capture_data(data_dir):
    """Loads the metadata JSON and binary IQ data from the capture directory."""
    metadata_path = os.path.join(data_dir, "metadata.json")
    with open(metadata_path, 'r') as f:
        metadata = json.load(f)

    devices = metadata.get("devices", [])
    signals = []
    
    if len(devices) > 0:
        # Sort devices by index
        devices = sorted(devices, key=lambda d: d.get("index", 0))
        for d in devices:
            filename = d.get("file")
            filepath = os.path.join(data_dir, filename)
            raw_data = np.fromfile(filepath, dtype=np.int16)
            raw_data = raw_data.reshape(-1, 2)
            iq_data = (raw_data[:, 0] + 1j * raw_data[:, 1]) / 32768.0
            signals.append(iq_data)
    else:
        # Fallback to auto-detecting usrp_X.bin files
        idx = 0
        while True:
            filepath = os.path.join(data_dir, f"usrp_{idx}.bin")
            if not os.path.exists(filepath):
                break
            raw_data = np.fromfile(filepath, dtype=np.int16)
            raw_data = raw_data.reshape(-1, 2)
            iq_data = (raw_data[:, 0] + 1j * raw_data[:, 1]) / 32768.0
            signals.append(iq_data)
            idx += 1

    if len(signals) < 2:
        raise ValueError("Need at least 2 devices for analysis. Found files: " + str(len(signals)))

    return metadata, signals

def plot_time_domain(sig0, sig1, sample_rate, output_dir):
    # Figure 1: Time-Domain Overlay
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))
    
    time_axis = np.arange(len(sig0)) / sample_rate * 1e6 # microseconds

    # Coarse plot (first 1000 samples for visibility)
    N_coarse = min(1000, len(sig0))
    ax1.plot(time_axis[:N_coarse], np.real(sig0[:N_coarse]), label="Device 0 (I)")
    ax1.plot(time_axis[:N_coarse], np.real(sig1[:N_coarse]), label="Device 1 (I)", alpha=0.8)
    ax1.set_title("Time-Domain Overlay (Coarse)")
    ax1.set_xlabel("Time (µs)")
    ax1.set_ylabel("Amplitude")
    ax1.legend()

    # Fine plot (zoom into a smaller window, e.g., 50 samples)
    zoom_start = N_coarse // 2
    zoom_end = zoom_start + 50
    if zoom_end <= len(sig0):
        ax2.plot(time_axis[zoom_start:zoom_end], np.real(sig0[zoom_start:zoom_end]), marker='o', label="Device 0 (I)")
        ax2.plot(time_axis[zoom_start:zoom_end], np.real(sig1[zoom_start:zoom_end]), marker='x', label="Device 1 (I)", alpha=0.8)
    ax2.set_title("Time-Domain Overlay (Zoomed)")
    ax2.set_xlabel("Time (µs)")
    ax2.set_ylabel("Amplitude")
    ax2.legend()
    
    plt.tight_layout()
    save_plot(fig, "fig01_time_domain_overlay", output_dir)
    plt.close(fig)

def plot_cross_correlation(sig0, sig1, output_dir):
    # Figure 2: Cross-Correlation Peak
    N = 10000 # Use first 10,000 samples for quick correlation
    s0 = sig0[:N]
    s1 = sig1[:N]
    
    correlation = np.abs(signal.correlate(s1, s0, mode='full'))
    lags = signal.correlation_lags(len(s1), len(s0), mode='full')
    
    # Normalize
    correlation /= np.max(correlation)

    fig, ax = plt.subplots()
    ax.plot(lags, correlation)
    
    peak_lag = lags[np.argmax(correlation)]
    ax.axvline(peak_lag, color='red', linestyle='--', label=f'Peak at lag={peak_lag}')
    
    # Zoom in around the peak (+/- 100 samples)
    ax.set_xlim(peak_lag - 100, peak_lag + 100)
    
    ax.set_title("Cross-Correlation Peak")
    ax.set_xlabel("Lag (samples)")
    ax.set_ylabel("Normalized Magnitude")
    ax.legend()
    
    save_plot(fig, "fig02_cross_correlation", output_dir)
    plt.close(fig)
    return peak_lag

def plot_phase_difference(sig0, sig1, sample_rate, output_dir):
    # Figure 3: Phase Difference Over Time
    # Truncate to min length
    min_len = min(len(sig0), len(sig1))
    s0 = sig0[:min_len]
    s1 = sig1[:min_len]
    
    phase0 = np.arctan2(np.imag(s0), np.real(s0))
    phase1 = np.arctan2(np.imag(s1), np.real(s1))
    
    # Compute unwrapped phase difference to avoid discontinuities
    phase_diff = np.unwrap(phase1 - phase0)
    
    # Convert to degrees
    phase_diff_deg = np.degrees(phase_diff)
    
    time_axis = np.arange(min_len) / sample_rate * 1e6 # microseconds
    
    fig, ax = plt.subplots()
    
    # For large datasets, plot a decimated version to save memory
    decimation_factor = max(1, min_len // 10000)
    ax.plot(time_axis[::decimation_factor], phase_diff_deg[::decimation_factor])
    
    ax.set_title("Phase Difference Over Time")
    ax.set_xlabel("Time (µs)")
    ax.set_ylabel("Phase Difference (Degrees)")
    
    save_plot(fig, "fig03_phase_difference", output_dir)
    plt.close(fig)
    return phase_diff_deg

def plot_psd(sig0, sig1, sample_rate, center_freq, output_dir):
    # Figure 4: Power Spectral Density Overlay
    fig, ax = plt.subplots()
    
    f0, Pxx0 = signal.welch(sig0, fs=sample_rate, return_onesided=False, nperseg=4096)
    f1, Pxx1 = signal.welch(sig1, fs=sample_rate, return_onesided=False, nperseg=4096)
    
    # Shift to center frequency
    f0_shifted = np.fft.fftshift(f0) + center_freq
    f1_shifted = np.fft.fftshift(f1) + center_freq
    
    Pxx0_shifted = 10 * np.log10(np.fft.fftshift(Pxx0))
    Pxx1_shifted = 10 * np.log10(np.fft.fftshift(Pxx1))
    
    ax.plot(f0_shifted / 1e6, Pxx0_shifted, label="Device 0")
    ax.plot(f1_shifted / 1e6, Pxx1_shifted, label="Device 1", alpha=0.7)
    
    ax.set_title("Power Spectral Density Overlay")
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Power (dBFS/Hz)")
    ax.legend()
    
    save_plot(fig, "fig04_psd_overlay", output_dir)
    plt.close(fig)

def plot_coherence(sig0, sig1, sample_rate, center_freq, output_dir):
    # Figure 5: Magnitude-Squared Coherence
    min_len = min(len(sig0), len(sig1))
    s0 = sig0[:min_len]
    s1 = sig1[:min_len]
    
    fig, ax = plt.subplots()
    
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        f, Cxy = signal.coherence(s1, s0, fs=sample_rate, nperseg=4096)
    
    # This is a one-sided coherence or two-sided depending on input (complex defaults to 2-sided)
    # We shift for display
    f_shifted = np.fft.fftshift(f) + center_freq
    Cxy_shifted = np.fft.fftshift(Cxy)
    
    ax.plot(f_shifted / 1e6, Cxy_shifted)
    
    ax.set_title("Magnitude-Squared Coherence")
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("Coherence")
    ax.set_ylim(-0.05, 1.05)
    
    save_plot(fig, "fig05_coherence", output_dir)
    plt.close(fig)

def plot_summary_dashboard(offset_samples, phase_diff_deg, metadata, output_dir):
    # Figure 6: Synchronization Summary Dashboard
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 8))
    
    # a) Time offset in samples
    ax1.bar(["Offset"], [offset_samples], color='C0')
    ax1.set_title("Time Offset (samples)")
    ax1.set_ylabel("Samples")
    if offset_samples == 0:
        ax1.text(0, 0, '0', ha='center', va='bottom', fontsize=12)
    
    # b) Frequency offset in Hz (slope of phase difference)
    # Estimate drift from phase difference
    sample_rate_hz = float(metadata["parameters"].get("sample_rate_hz", 1e6))
    time_s = np.arange(len(phase_diff_deg)) / sample_rate_hz
    # Fit line: y = mx + c. Slope m is in deg/s. Hz is m / 360
    if len(phase_diff_deg) > 1:
        p = np.polyfit(time_s, phase_diff_deg, 1)
        freq_offset_hz = p[0] / 360.0
    else:
        freq_offset_hz = 0.0
        
    ax2.bar(["Offset"], [freq_offset_hz], color='C1')
    ax2.set_title("Frequency Offset (Hz)")
    ax2.set_ylabel("Hz")
    
    # c) PPS Timestamp Alignment Verification
    deltas = metadata.get("sync_results", {}).get("pps_time_deltas_us", [0.0])
    mean_delta = np.mean(deltas) if deltas else 0.0
    ax3.bar(["Mean Delta"], [mean_delta], color='C2')
    ax3.set_title("PPS Delta (µs)")
    ax3.set_ylabel("µs")
    
    # d) Estimated drift rate in ppm
    drifts = metadata.get("sync_results", {}).get("drift_ppm", [0.0])
    mean_drift = np.mean(drifts) if drifts else 0.0
    ax4.bar(["Drift"], [mean_drift], color='C3')
    ax4.set_title("Drift Rate (ppm)")
    ax4.set_ylabel("ppm")
    
    plt.tight_layout()
    save_plot(fig, "fig06_sync_dashboard", output_dir)
    plt.close(fig)

def plot_constellation(sig0, sig1, output_dir):
    # Figure 7: IQ Constellation Diagram
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
    
    # Plot a subset of points to avoid clutter
    N = min(10000, len(sig0))
    ax1.scatter(np.real(sig0[:N]), np.imag(sig0[:N]), alpha=0.1, s=1)
    ax1.set_title("Device 0 Constellation")
    ax1.set_xlabel("In-Phase (I)")
    ax1.set_ylabel("Quadrature (Q)")
    ax1.axis('equal')
    ax1.set_xlim(-1.1, 1.1)
    ax1.set_ylim(-1.1, 1.1)
    
    N = min(10000, len(sig1))
    ax2.scatter(np.real(sig1[:N]), np.imag(sig1[:N]), alpha=0.1, s=1)
    ax2.set_title("Device 1 Constellation")
    ax2.set_xlabel("In-Phase (I)")
    ax2.set_ylabel("Quadrature (Q)")
    ax2.axis('equal')
    ax2.set_xlim(-1.1, 1.1)
    ax2.set_ylim(-1.1, 1.1)
    
    plt.tight_layout()
    save_plot(fig, "fig07_constellation", output_dir)
    plt.close(fig)

def plot_phase_histogram(phase_diff_deg, output_dir):
    # Figure 8: Phase Difference Histogram
    fig, ax = plt.subplots()
    
    # Use a decimated version to speed up plotting
    decimation_factor = max(1, len(phase_diff_deg) // 100000)
    data = phase_diff_deg[::decimation_factor]
    
    # Shift phase difference to mean 0 for clean plotting (handling wrapping)
    # Standardize to [-180, 180]
    data_wrapped = (data + 180) % 360 - 180
    
    mu, std = norm.fit(data_wrapped)
    
    ax.hist(data_wrapped, bins=100, density=True, alpha=0.6, color='g')
    
    xmin, xmax = ax.get_xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    ax.plot(x, p, 'k', linewidth=2, label=rf'Fit: $\mu$={mu:.2f}°, $\sigma$={std:.2f}°')
    
    ax.set_title("Phase Difference Histogram")
    ax.set_xlabel("Phase Difference (Degrees)")
    ax.set_ylabel("Density")
    ax.legend()
    
    save_plot(fig, "fig08_phase_histogram", output_dir)
    plt.close(fig)

def main():
    parser = argparse.ArgumentParser(description="Synchronized USRP Data Analysis")
    parser.add_argument("--input-dir", required=True, help="Directory containing metadata.json and .bin files")
    parser.add_argument("--output-dir", required=True, help="Directory to save output plots")
    parser.add_argument("--format", default="png", help="Ignored, script output png/pdf automatically")
    parser.add_argument("--dpi", type=int, default=300, help="Ignored, script uses 300 dpi")
    
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    setup_academic_style()
    
    print(f"Loading data from {args.input_dir}...")
    metadata, signals = load_capture_data(args.input_dir)
    
    sig0 = signals[0]
    sig1 = signals[1]
    
    params = metadata.get("parameters", {})
    sample_rate = float(params.get("sample_rate_hz", 1e6))
    center_freq = float(params.get("center_frequency_hz", 915e6))
    
    print("Generating Figure 1: Time-Domain Overlay...")
    plot_time_domain(sig0, sig1, sample_rate, args.output_dir)
    
    print("Generating Figure 2: Cross-Correlation Peak...")
    offset_samples = plot_cross_correlation(sig0, sig1, args.output_dir)
    
    print("Generating Figure 3: Phase Difference Over Time...")
    phase_diff_deg = plot_phase_difference(sig0, sig1, sample_rate, args.output_dir)
    
    print("Generating Figure 4: Power Spectral Density...")
    plot_psd(sig0, sig1, sample_rate, center_freq, args.output_dir)
    
    print("Generating Figure 5: Magnitude-Squared Coherence...")
    plot_coherence(sig0, sig1, sample_rate, center_freq, args.output_dir)
    
    print("Generating Figure 6: Synchronization Summary Dashboard...")
    plot_summary_dashboard(offset_samples, phase_diff_deg, metadata, args.output_dir)
    
    print("Generating Figure 7: IQ Constellation Diagram...")
    plot_constellation(sig0, sig1, args.output_dir)
    
    print("Generating Figure 8: Phase Difference Histogram...")
    plot_phase_histogram(phase_diff_deg, args.output_dir)
    
    print(f"Analysis complete. Plots saved to {args.output_dir}")

if __name__ == "__main__":
    main()
