#!/usr/bin/env python3
"""
Analyze synchronization between two IQ captures produced by capture_sync.py.

Metrics computed:
  - Time offset    — windowed cross-correlation with sub-sample interpolation
  - Frequency offset — least-squares fit to the unwrapped inter-device phase
  - Phase coherence  — stddev of phase residuals after removing the linear trend

When --tone-offset is supplied, segmented-FFT analysis is used instead of
per-sample phase estimation, which is required when per-sample SNR is negative.
"""
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate, windows, firwin, lfilter


def parse_args():
    p = argparse.ArgumentParser(
        description="Analyze sync between two IQ captures.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--files', nargs=2, required=True,
                   help='IQ .npy files from capture_sync.py')
    p.add_argument('--rate', type=float, default=1e6,
                   help='Sample rate in Hz; overridden by metadata if available')
    p.add_argument('--save-plot', metavar='FILE',
                   help='Save plot to FILE instead of displaying interactively')
    p.add_argument('--tone-offset', type=float, default=None, metavar='HZ',
                   help='IF offset (Hz) of the TX tone used during capture '
                        '(e.g. 100e3). Enables segmented-FFT phase analysis and '
                        'resolves the cross-correlation period ambiguity.')
    return p.parse_args()


def load_meta(npy_path):
    """Read key: value metadata from the companion _meta.txt file, if present."""
    base = npy_path[:-4] + '_meta.txt'
    meta = {}
    if os.path.exists(base):
        with open(base) as f:
            for line in f:
                if ': ' in line:
                    k, v = line.split(': ', 1)
                    meta[k.strip()] = v.strip()
    return meta


def filter_tone(sig, rate, tone_offset, bw=1e3):
    """Frequency-shift the tone at *tone_offset* Hz to DC and apply a 1 kHz LPF."""
    n = len(sig)
    t = np.arange(n) / rate
    sig_bb = sig * np.exp(-1j * 2 * np.pi * tone_offset * t)
    ntaps = 2047
    h = firwin(ntaps, (bw / 2) / (rate / 2))
    sig_filt = lfilter(h, 1.0, sig_bb)
    delay = (ntaps - 1) // 2
    return sig_filt[delay:]


def estimate_tone_snr(sig, rate, tone_offset):
    """Estimate tone SNR (dB) by comparing the DFT bin at *tone_offset*
    to the median noise floor measured 10–20 kHz away from the tone.
    Returns (snr_db, tone_power, noise_floor).
    """
    n = len(sig)
    win = windows.hann(n)
    S = np.fft.fft(sig * win)  # complex IQ: use fft, not rfft
    psd = np.abs(S) ** 2 / n
    freqs = np.fft.fftfreq(n, d=1.0 / rate)
    tone_bin = int(round(tone_offset / rate * n)) % n
    tone_power = psd[tone_bin]
    guard = int(10e3 / rate * n)
    noise_bins = np.concatenate([
        psd[max(0, tone_bin - 2 * guard): max(0, tone_bin - guard)],
        psd[tone_bin + guard: tone_bin + 2 * guard],
    ])
    noise_floor = float(np.median(noise_bins)) if len(noise_bins) > 0 else 1.0
    snr_db = 10 * np.log10(tone_power / noise_floor) if noise_floor > 0 else 0.0
    return float(snr_db), float(tone_power), float(noise_floor)


def resolve_tone_lag(raw_lag, rate, tone_offset):
    """Fold *raw_lag* into (-T/2, T/2] where T = rate/tone_offset.

    A CW tone produces correlation peaks at every multiple of its period.
    Folding to the nearest alias recovers the true sub-period time offset.
    """
    period = rate / tone_offset
    wrapped = raw_lag % period
    if wrapped > period / 2:
        wrapped -= period
    return wrapped


def cross_correlation(x, y):
    """Hann-windowed cross-correlation with parabolic sub-sample peak interpolation."""
    n = min(len(x), len(y))
    win = windows.hann(n)
    xc = correlate(x[:n] * win, y[:n] * win, mode='full')
    lag = np.arange(-(n - 1), n)
    peak_idx = int(np.argmax(np.abs(xc)))
    if 1 <= peak_idx < len(xc) - 1:
        y0, y1, y2 = np.abs(xc[peak_idx - 1:peak_idx + 2])
        denom = y0 - 2 * y1 + y2
        sub = (y0 - y2) / (2 * denom) if denom != 0 else 0.0
    else:
        sub = 0.0
    fractional_offset = lag[peak_idx] + sub
    return fractional_offset, xc, lag


def estimate_freq_offset(x, y, rate):
    """Estimate frequency offset via LS fit to the unwrapped per-sample phase diff.

    Returns (freq_offset, phase_diff_wrapped, phase_unwrapped, phase_residuals).
    """
    n = min(len(x), len(y))
    phase_diff = np.angle(x[:n] * np.conj(y[:n]))
    phase_unwrapped = np.unwrap(phase_diff)
    t = np.arange(n) / rate
    coeffs = np.polyfit(t, phase_unwrapped, 1)
    slope = coeffs[0]
    freq_offset = slope / (2 * np.pi)
    phase_residuals = phase_unwrapped - np.polyval(coeffs, t)
    return freq_offset, phase_diff, phase_unwrapped, phase_residuals


def estimate_freq_offset_tone(x, y, rate, tone_offset, seg_dur=0.05):
    """Estimate frequency offset via segmented-FFT cross-spectrum.

    Divides both signals into *seg_dur*-second segments, computes each
    segment's FFT, and extracts the cross-phase at the tone bin. The
    coherent FFT integration overcomes the negative per-sample SNR that
    makes direct per-sample unwrapping unreliable.

    Returns (freq_offset, seg_phases_wrapped, seg_phases_unwrapped,
             phase_residuals, seg_times).
    """
    n = min(len(x), len(y))
    seg_len = int(seg_dur * rate)
    n_segs = n // seg_len
    if n_segs < 4:
        n_segs = max(4, n // int(0.01 * rate))
        seg_len = n // n_segs

    win = windows.hann(seg_len)
    tone_bin = int(round(tone_offset / rate * seg_len)) % seg_len

    seg_phases = np.zeros(n_segs)
    seg_times  = np.zeros(n_segs)
    for i in range(n_segs):
        sl = slice(i * seg_len, (i + 1) * seg_len)
        Sx = np.fft.fft(x[sl] * win)
        Sy = np.fft.fft(y[sl] * win)
        seg_phases[i] = np.angle(Sx[tone_bin] * np.conj(Sy[tone_bin]))
        seg_times[i]  = (i + 0.5) * seg_dur

    seg_phases_unwrapped = np.unwrap(seg_phases)
    coeffs = np.polyfit(seg_times, seg_phases_unwrapped, 1)
    freq_offset     = coeffs[0] / (2 * np.pi)
    phase_residuals = seg_phases_unwrapped - np.polyval(coeffs, seg_times)
    return freq_offset, seg_phases, seg_phases_unwrapped, phase_residuals, seg_times


def plot_results(xc, lag, offset, phase_diff, phase_unwrapped, phase_residuals,
                 freq_offset, rate, seg_times=None, save_path=None):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))

    axes[0].plot(lag, np.abs(xc))
    axes[0].set_title(
        f'Cross-correlation  |  peak lag = {offset:.3f} samples  '
        f'({offset / rate * 1e6:.3f} µs)')
    axes[0].set_xlabel('Lag (samples)')
    axes[0].set_ylabel('Correlation magnitude')
    axes[0].grid(True)

    if seg_times is not None:
        t_axis = seg_times * 1e3  # ms
        xlabel = 'Segment centre (ms)'
        plot_kw = {'marker': 'o'}
    else:
        t_axis = np.arange(len(phase_diff)) / rate * 1e3
        xlabel = 'Time (ms)'
        plot_kw = {}

    axes[1].plot(t_axis, phase_diff, alpha=0.8, label='wrapped phase diff', **plot_kw)
    axes[1].set_title(
        f'Phase difference per segment  |  '
        f'freq offset = {freq_offset:.3f} Hz  |  '
        f'residual stddev = {np.std(phase_residuals):.3f} rad')
    axes[1].set_xlabel(xlabel)
    axes[1].set_ylabel('Phase diff (rad)')
    axes[1].legend()
    axes[1].grid(True)

    slope_line = np.polyval(
        [freq_offset * 2 * np.pi, phase_unwrapped[0]],
        t_axis / 1e3)
    axes[2].plot(t_axis, phase_unwrapped, color='tab:orange',
                 label='unwrapped', **plot_kw)
    axes[2].plot(t_axis, slope_line, 'k--', linewidth=1, label='LS fit')
    axes[2].set_title('Unwrapped phase difference + least-squares fit')
    axes[2].set_xlabel(xlabel)
    axes[2].set_ylabel('Phase (rad)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"Plot saved to {save_path}")
    else:
        plt.show()


def main():
    args = parse_args()
    x = np.load(args.files[0])
    y = np.load(args.files[1])
    meta0 = load_meta(args.files[0])
    meta1 = load_meta(args.files[1])

    # Use sample rate from metadata if both files agree (and CLI was not overridden)
    if meta0.get('rate') and meta1.get('rate'):
        r0, r1 = float(meta0['rate']), float(meta1['rate'])
        if r0 == r1:
            args.rate = r0

    offset, xc, lag = cross_correlation(x, y)
    # Resolve the period ambiguity when a CW tone was used.
    if args.tone_offset is not None:
        offset = resolve_tone_lag(offset, args.rate, args.tone_offset)

    snr_a, snr_b = None, None
    seg_times = None
    if args.tone_offset is not None:
        snr_a, _, _ = estimate_tone_snr(x, args.rate, args.tone_offset)
        snr_b, _, _ = estimate_tone_snr(y, args.rate, args.tone_offset)
        (freq_offset, phase_diff, phase_unwrapped,
         phase_residuals, seg_times) = estimate_freq_offset_tone(
            x, y, args.rate, args.tone_offset)
    else:
        freq_offset, phase_diff, phase_unwrapped, phase_residuals = \
            estimate_freq_offset(x, y, args.rate)
    phase_coherence_std = float(np.std(phase_residuals))
    low_snr = snr_a is not None and (snr_a < 10 or snr_b < 10)

    # --- Summary report ---
    print("=" * 54)
    print("  Synchronization Analysis Report")
    print("=" * 54)
    if meta0:
        print(f"  Device A     : {meta0.get('serial', args.files[0])}")
        print(f"  Device B     : {meta1.get('serial', args.files[1])}")
        print(f"  Clock source : {meta0.get('clock_source', 'unknown')}")
        print(f"  Time source  : {meta0.get('time_source', 'unknown')}")
    print(f"  Sample rate  : {args.rate / 1e6:.3f} MHz")
    if snr_a is not None:
        print(f"  Tone SNR (A) : {snr_a:+.1f} dB")
        print(f"  Tone SNR (B) : {snr_b:+.1f} dB")
        if low_snr:
            print(f"  WARNING: SNR < 10 dB — freq/phase metrics unreliable.")
            print(f"           Move devices closer, increase TX gain, or add attenuation.")
    print("-" * 54)
    time_offset_us = offset / args.rate * 1e6
    print(f"  Time offset     : {offset:+.3f} samples  ({time_offset_us:+.3f} µs)")
    print(f"  Freq offset     : {freq_offset:+.3f} Hz")
    print(f"  Phase coherence : {phase_coherence_std:.3f} rad (stddev)")
    print("-" * 54)

    verdict_time  = "GOOD" if abs(time_offset_us) < 10 else "POOR"
    verdict_freq  = "N/A (low SNR)" if low_snr else ("GOOD" if abs(freq_offset) < 10 else "POOR")
    verdict_phase = "N/A (low SNR)" if low_snr else ("GOOD" if phase_coherence_std < 0.5 else "POOR")
    print(f"  Time sync       : {verdict_time}  (threshold <10 µs)")
    print(f"  Freq sync       : {verdict_freq}  (threshold <10 Hz)")
    print(f"  Phase coherence : {verdict_phase}  (threshold <0.5 rad)")

    clock_src = meta0.get('clock_source', 'unknown')
    time_src  = meta0.get('time_source', 'unknown')
    if verdict_time == "POOR" or verdict_freq == "POOR":
        if clock_src == 'internal' and time_src == 'internal':
            print("\n  HINT: Both devices used internal clocks with no external sync.")
            print("  Add a PPS signal and capture with --time-source external to improve")
            print("  time alignment. Add a shared 10 MHz reference and use")
            print("  --clock-source external to eliminate frequency offset.")
        elif clock_src == 'internal' and time_src in ('external', 'gpsdo'):
            print("\n  HINT: PPS-only mode — time alignment should be good but a frequency")
            print("  offset is expected because each device runs its own TCXO.")
            print("  Add a shared 10 MHz reference and use --clock-source external")
            print("  to reduce frequency offset.")
    print("=" * 54)

    plot_results(xc, lag, offset, phase_diff, phase_unwrapped, phase_residuals,
                 freq_offset, args.rate, seg_times=seg_times,
                 save_path=args.save_plot)


if __name__ == '__main__':
    main()
