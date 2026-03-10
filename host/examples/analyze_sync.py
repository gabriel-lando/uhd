#!/usr/bin/env python3
"""
analyze_sync.py

Python script to analyze synchronization quality between two USRP devices
from captured sample files.

Usage:
    python3 analyze_sync.py device0_samples.dat device1_samples.dat [--rate 10e6]

This script:
- Loads samples from both devices
- Computes cross-correlation to find time offset
- Calculates frequency offset (drift)
- Visualizes spectrograms for alignment comparison
- Computes phase coherence over time
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq, fftshift
import argparse

def load_samples(filename):
    """Load complex float32 samples from binary file"""
    samples = np.fromfile(filename, dtype=np.complex64)
    print(f"Loaded {len(samples)} samples from {filename}")
    return samples

def compute_cross_correlation(sig1, sig2, max_lag=1000):
    """
    Compute cross-correlation between two signals.
    Returns: (lags, correlation)
    """
    # Use only first portion for speed
    n = min(len(sig1), len(sig2), 100000)
    sig1 = sig1[:n]
    sig2 = sig2[:n]
    
    # Normalize signals
    sig1 = sig1 - np.mean(sig1)
    sig2 = sig2 - np.mean(sig2)
    sig1 = sig1 / np.std(sig1)
    sig2 = sig2 / np.std(sig2)
    
    # Compute correlation around zero lag
    correlation = np.correlate(sig1, sig2, mode='same')
    correlation = np.abs(correlation) / len(sig1)
    
    # Extract region around peak
    center = len(correlation) // 2
    start = max(0, center - max_lag)
    end = min(len(correlation), center + max_lag)
    
    lags = np.arange(start - center, end - center)
    correlation = correlation[start:end]
    
    return lags, correlation

def estimate_time_offset(sig1, sig2, sample_rate):
    """
    Estimate time offset between two signals using cross-correlation.
    Returns: offset in seconds
    """
    lags, corr = compute_cross_correlation(sig1, sig2, max_lag=1000)
    
    # Find peak
    peak_idx = np.argmax(corr)
    peak_lag = lags[peak_idx]
    
    time_offset = peak_lag / sample_rate
    correlation_peak = corr[peak_idx]
    
    return time_offset, peak_lag, correlation_peak

def estimate_frequency_offset(sig1, sig2, sample_rate):
    """
    Estimate frequency offset between two signals using phase slope method.
    Returns: frequency offset in Hz
    """
    # Use middle portion of signal
    n = min(len(sig1), len(sig2))
    start = n // 4
    end = 3 * n // 4
    sig1 = sig1[start:end]
    sig2 = sig2[start:end]
    
    # Compute instantaneous phase difference
    phase_diff = np.angle(sig1 * np.conj(sig2))
    
    # Unwrap phase
    phase_unwrapped = np.unwrap(phase_diff)
    
    # Fit line to phase vs time
    time = np.arange(len(phase_unwrapped)) / sample_rate
    coeffs = np.polyfit(time, phase_unwrapped, 1)
    
    # Slope is 2*pi*freq_offset
    freq_offset = coeffs[0] / (2 * np.pi)
    
    return freq_offset, phase_unwrapped

def compute_spectrogram(samples, sample_rate, nperseg=1024):
    """Compute spectrogram of signal"""
    f, t, Sxx = signal.spectrogram(samples, fs=sample_rate, nperseg=nperseg,
                                     noverlap=nperseg//2, mode='magnitude')
    return f, t, Sxx

def plot_alignment_analysis(dev0, dev1, sample_rate):
    """Create comprehensive alignment analysis plots"""
    
    fig = plt.figure(figsize=(16, 12))
    
    # 1. Time domain signals
    ax1 = plt.subplot(4, 2, 1)
    time_axis = np.arange(min(10000, len(dev0))) / sample_rate * 1e6  # microseconds
    ax1.plot(time_axis, np.real(dev0[:len(time_axis)]), label='Device 0', alpha=0.7)
    ax1.plot(time_axis, np.real(dev1[:len(time_axis)]), label='Device 1', alpha=0.7)
    ax1.set_xlabel('Time (μs)')
    ax1.set_ylabel('Amplitude')
    ax1.set_title('Time Domain Signals (first 10k samples)')
    ax1.legend()
    ax1.grid(True)
    
    # 2. Cross-correlation
    ax2 = plt.subplot(4, 2, 2)
    time_offset, peak_lag, corr_peak = estimate_time_offset(dev0, dev1, sample_rate)
    lags, corr = compute_cross_correlation(dev0, dev1, max_lag=1000)
    ax2.plot(lags / sample_rate * 1e6, corr)
    ax2.axvline(peak_lag / sample_rate * 1e6, color='r', linestyle='--', 
                label=f'Peak at {time_offset*1e6:.3f} μs')
    ax2.set_xlabel('Time Lag (μs)')
    ax2.set_ylabel('Correlation')
    ax2.set_title(f'Cross-Correlation (peak: {corr_peak:.4f})')
    ax2.legend()
    ax2.grid(True)
    
    # 3. Spectrogram device 0
    ax3 = plt.subplot(4, 2, 3)
    f0, t0, Sxx0 = compute_spectrogram(dev0, sample_rate)
    im3 = ax3.pcolormesh(t0, fftshift(f0)/1e6, fftshift(10*np.log10(Sxx0 + 1e-10), axes=0), 
                          shading='gouraud', cmap='viridis')
    ax3.set_ylabel('Frequency (MHz)')
    ax3.set_xlabel('Time (s)')
    ax3.set_title('Spectrogram - Device 0')
    plt.colorbar(im3, ax=ax3, label='Power (dB)')
    
    # 4. Spectrogram device 1
    ax4 = plt.subplot(4, 2, 4)
    f1, t1, Sxx1 = compute_spectrogram(dev1, sample_rate)
    im4 = ax4.pcolormesh(t1, fftshift(f1)/1e6, fftshift(10*np.log10(Sxx1 + 1e-10), axes=0), 
                          shading='gouraud', cmap='viridis')
    ax4.set_ylabel('Frequency (MHz)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('Spectrogram - Device 1')
    plt.colorbar(im4, ax=ax4, label='Power (dB)')
    
    # 5. Power spectrum comparison
    ax5 = plt.subplot(4, 2, 5)
    n_fft = 8192
    freq0 = fftshift(fftfreq(n_fft, 1/sample_rate)) / 1e6
    psd0 = fftshift(10*np.log10(np.abs(fft(dev0[:n_fft], n_fft))**2 + 1e-10))
    psd1 = fftshift(10*np.log10(np.abs(fft(dev1[:n_fft], n_fft))**2 + 1e-10))
    ax5.plot(freq0, psd0, label='Device 0', alpha=0.7)
    ax5.plot(freq0, psd1, label='Device 1', alpha=0.7)
    ax5.set_xlabel('Frequency (MHz)')
    ax5.set_ylabel('PSD (dB)')
    ax5.set_title('Power Spectral Density')
    ax5.legend()
    ax5.grid(True)
    
    # 6. Frequency offset estimation
    ax6 = plt.subplot(4, 2, 6)
    freq_offset, phase_unwrapped = estimate_frequency_offset(dev0, dev1, sample_rate)
    time_phase = np.arange(len(phase_unwrapped)) / sample_rate
    ax6.plot(time_phase, phase_unwrapped, label='Measured', alpha=0.7)
    # Plot fitted line
    fitted_phase = np.polyval([2*np.pi*freq_offset, 0], time_phase)
    ax6.plot(time_phase, fitted_phase, 'r--', label=f'Fit: {freq_offset:.3f} Hz')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Phase Difference (rad)')
    ax6.set_title(f'Phase Difference (freq offset: {freq_offset:.3f} Hz, {freq_offset/sample_rate*1e6:.3f} ppm)')
    ax6.legend()
    ax6.grid(True)
    
    # 7. Instantaneous power
    ax7 = plt.subplot(4, 2, 7)
    window_size = 1000
    power0 = np.convolve(np.abs(dev0)**2, np.ones(window_size)/window_size, mode='valid')
    power1 = np.convolve(np.abs(dev1)**2, np.ones(window_size)/window_size, mode='valid')
    time_power = np.arange(len(power0)) / sample_rate
    ax7.plot(time_power, 10*np.log10(power0 + 1e-10), label='Device 0')
    ax7.plot(time_power, 10*np.log10(power1 + 1e-10), label='Device 1')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('Power (dB)')
    ax7.set_title('Instantaneous Power')
    ax7.legend()
    ax7.grid(True)
    
    # 8. Correlation vs time (coherence)
    ax8 = plt.subplot(4, 2, 8)
    chunk_size = 10000
    n_chunks = min(len(dev0), len(dev1)) // chunk_size
    coherence = []
    time_coherence = []
    
    for i in range(n_chunks):
        start = i * chunk_size
        end = start + chunk_size
        chunk0 = dev0[start:end]
        chunk1 = dev1[start:end]
        
        # Normalize and correlate
        chunk0 = (chunk0 - np.mean(chunk0)) / (np.std(chunk0) + 1e-10)
        chunk1 = (chunk1 - np.mean(chunk1)) / (np.std(chunk1) + 1e-10)
        
        corr_value = np.abs(np.sum(chunk0 * np.conj(chunk1))) / len(chunk0)
        coherence.append(corr_value)
        time_coherence.append(start / sample_rate)
    
    ax8.plot(time_coherence, coherence)
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Coherence')
    ax8.set_title('Correlation Coefficient Over Time')
    ax8.axhline(y=0.9, color='g', linestyle='--', alpha=0.5, label='Good (>0.9)')
    ax8.axhline(y=0.7, color='y', linestyle='--', alpha=0.5, label='Fair (>0.7)')
    ax8.legend()
    ax8.grid(True)
    
    plt.tight_layout()
    return fig

def print_summary(dev0, dev1, sample_rate):
    """Print summary statistics"""
    print("\n" + "="*60)
    print("SYNCHRONIZATION QUALITY SUMMARY")
    print("="*60)
    
    # Time offset
    time_offset, peak_lag, corr_peak = estimate_time_offset(dev0, dev1, sample_rate)
    print(f"\nTime Alignment:")
    print(f"  Time offset: {time_offset*1e6:.3f} μs ({peak_lag} samples)")
    print(f"  Correlation peak: {corr_peak:.4f}")
    
    if abs(time_offset) < 1e-6:
        print(f"  ✓ EXCELLENT (< 1 μs)")
    elif abs(time_offset) < 10e-6:
        print(f"  ✓ GOOD (< 10 μs)")
    elif abs(time_offset) < 100e-6:
        print(f"  ⚠ FAIR (< 100 μs)")
    else:
        print(f"  ✗ POOR (> 100 μs)")
    
    # Frequency offset
    freq_offset, _ = estimate_frequency_offset(dev0, dev1, sample_rate)
    freq_offset_ppm = abs(freq_offset) / sample_rate * 1e6
    print(f"\nFrequency Alignment:")
    print(f"  Frequency offset: {freq_offset:.3f} Hz")
    print(f"  Relative offset: {freq_offset_ppm:.3f} ppm")
    
    if freq_offset_ppm < 0.01:
        print(f"  ✓ EXCELLENT (< 0.01 ppm) - Hardware locked")
    elif freq_offset_ppm < 1.0:
        print(f"  ✓ GOOD (< 1 ppm)")
    elif freq_offset_ppm < 5.0:
        print(f"  ⚠ FAIR (< 5 ppm) - Typical for independent TCXOs")
    else:
        print(f"  ✗ POOR (> 5 ppm)")
    
    # Power levels
    power0 = np.mean(np.abs(dev0)**2)
    power1 = np.mean(np.abs(dev1)**2)
    power_diff_db = 10*np.log10(power0 / power1)
    
    print(f"\nPower Levels:")
    print(f"  Device 0: {10*np.log10(power0):.2f} dB")
    print(f"  Device 1: {10*np.log10(power1):.2f} dB")
    print(f"  Difference: {power_diff_db:.2f} dB")
    
    # Overall coherence
    n = min(len(dev0), len(dev1), 100000)
    dev0_norm = (dev0[:n] - np.mean(dev0[:n])) / np.std(dev0[:n])
    dev1_norm = (dev1[:n] - np.mean(dev1[:n])) / np.std(dev1[:n])
    overall_coherence = np.abs(np.sum(dev0_norm * np.conj(dev1_norm))) / n
    
    print(f"\nOverall Coherence:")
    print(f"  Correlation coefficient: {overall_coherence:.4f}")
    
    if overall_coherence > 0.9:
        print(f"  ✓ EXCELLENT correlation")
    elif overall_coherence > 0.7:
        print(f"  ✓ GOOD correlation")
    elif overall_coherence > 0.5:
        print(f"  ⚠ FAIR correlation")
    else:
        print(f"  ✗ POOR correlation")
    
    # Recommendations
    print(f"\nRecommendations:")
    if freq_offset_ppm > 1.0 and freq_offset_ppm < 5.0:
        print("  • Consider adding external 10 MHz reference for better frequency lock")
    if abs(time_offset) > 10e-6:
        print("  • Check PPS cable connections and signal quality")
    if corr_peak < 0.7:
        print("  • Verify both devices are tuned to same frequency")
        print("  • Check antenna connections and RF environment")
    if overall_coherence > 0.9 and freq_offset_ppm < 0.01:
        print("  ✓ Excellent synchronization! Devices are well aligned.")
    
    print("="*60 + "\n")

def main():
    parser = argparse.ArgumentParser(
        description='Analyze synchronization quality between two USRP devices'
    )
    parser.add_argument('file0', help='Sample file from device 0')
    parser.add_argument('file1', help='Sample file from device 1')
    parser.add_argument('--rate', type=float, default=10e6, 
                       help='Sample rate in Hz (default: 10e6)')
    parser.add_argument('--no-plot', action='store_true',
                       help='Skip plotting (summary only)')
    
    args = parser.parse_args()
    
    # Load samples
    print(f"Loading samples...")
    dev0 = load_samples(args.file0)
    dev1 = load_samples(args.file1)
    
    # Check lengths
    if len(dev0) == 0 or len(dev1) == 0:
        print("ERROR: One or both files are empty!")
        return 1
    
    # Truncate to same length
    n = min(len(dev0), len(dev1))
    dev0 = dev0[:n]
    dev1 = dev1[:n]
    
    # Print summary
    print_summary(dev0, dev1, args.rate)
    
    # Create plots
    if not args.no_plot:
        print("Generating plots...")
        fig = plot_alignment_analysis(dev0, dev1, args.rate)
        plt.savefig('sync_analysis.png', dpi=150, bbox_inches='tight')
        print("Plot saved to: sync_analysis.png")
        plt.show()
    
    return 0

if __name__ == '__main__':
    exit(main())
