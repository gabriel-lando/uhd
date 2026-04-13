#!/usr/bin/env python3
"""
Analyze synchronization between IQ captures produced by capture_sync.py.

Usage:
  analyze_sync.py --files <file0.npy> <file1.npy> [file2.npy ...] [options]

Device 0 (first file) is the reference. Metrics are computed pairwise for
each (device 0, device i) combination — odd numbers of devices are fine.
One plot is generated per pair; when saving with --save-plot the pair index
is appended before the extension (e.g. report_0.png, report_1.png).

Metrics computed per pair:
  - Time offset      — Hann-windowed cross-correlation with sub-sample interpolation
  - Frequency offset — least-squares fit to the unwrapped inter-device phase
  - Phase coherence  — stddev of phase residuals after removing the linear trend

When --tone-offset is supplied, segmented-FFT analysis is used (50 ms segments)
instead of per-sample phase estimation; this is required when the per-sample SNR
is negative, as is typical for wideband RX captures without a strong CW tone.
"""
import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate, windows, firwin, lfilter


def parse_args():
    p = argparse.ArgumentParser(
        description="Analyze sync between IQ captures (device 0 is the reference).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--files', nargs='+', required=True,
                   help='IQ .npy files from capture_sync.py (minimum 2; '
                        'first file is the reference device)')
    p.add_argument('--rate', type=float, default=1e6,
                   help='Sample rate in Hz; overridden by metadata if available')
    p.add_argument('--save-plot', metavar='FILE',
                   help='Save plot to FILE instead of displaying interactively. '
                        'With multiple pairs the pair index is appended before the extension.')
    p.add_argument('--tone-offset', type=float, default=None, metavar='HZ',
                   help='IF offset (Hz) of the TX tone used during capture '
                        '(e.g. 100e3). Enables segmented-FFT phase analysis and '
                        'resolves the cross-correlation period ambiguity.')
    args = p.parse_args()
    if len(args.files) < 2:
        p.error('--files requires at least 2 .npy files')
    return args


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


def _save_path_for_pair(save_path, pair_idx, n_pairs):
    """Insert pair index into save_path when there are multiple pairs."""
    if save_path is None or n_pairs == 1:
        return save_path
    root, ext = os.path.splitext(save_path)
    return f"{root}_{pair_idx}{ext}"


def plot_results(xc, lag, offset, phase_diff, phase_unwrapped, phase_residuals,
                 freq_offset, rate, label_ref, label_dev,
                 seg_times=None, save_path=None):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))

    axes[0].plot(lag, np.abs(xc))
    axes[0].set_title(
        f'[{label_ref} → {label_dev}]  Cross-correlation  |  '
        f'peak lag = {offset:.3f} samples  ({offset / rate * 1e6:.3f} µs)')
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


def _analyze_pair(x, y, rate, tone_offset):
    """Run all analysis on one (reference, device) pair.

    Returns a dict with keys: offset, xc, lag, freq_offset, phase_diff,
    phase_unwrapped, phase_residuals, seg_times, snr_ref, snr_dev, low_snr.
    """
    offset, xc, lag = cross_correlation(x, y)
    if tone_offset is not None:
        offset = resolve_tone_lag(offset, rate, tone_offset)

    snr_ref, snr_dev = None, None
    seg_times = None
    if tone_offset is not None:
        snr_ref, _, _ = estimate_tone_snr(x, rate, tone_offset)
        snr_dev, _, _ = estimate_tone_snr(y, rate, tone_offset)
        (freq_offset, phase_diff, phase_unwrapped,
         phase_residuals, seg_times) = estimate_freq_offset_tone(
            x, y, rate, tone_offset)
    else:
        freq_offset, phase_diff, phase_unwrapped, phase_residuals = \
            estimate_freq_offset(x, y, rate)

    return dict(
        offset=offset, xc=xc, lag=lag,
        freq_offset=freq_offset,
        phase_diff=phase_diff,
        phase_unwrapped=phase_unwrapped,
        phase_residuals=phase_residuals,
        seg_times=seg_times,
        snr_ref=snr_ref, snr_dev=snr_dev,
        low_snr=(snr_ref is not None and (snr_ref < 10 or snr_dev < 10)),
    )


def main():
    args = parse_args()
    files = args.files
    n_pairs = len(files) - 1

    # Load all captures and metadata
    sigs  = [np.load(f) for f in files]
    metas = [load_meta(f) for f in files]

    # Resolve sample rate from metadata (use first file's value if consistent)
    for m in metas:
        if m.get('rate'):
            args.rate = float(m['rate'])
            break

    ref_serial  = metas[0].get('serial', files[0])
    clock_src   = metas[0].get('clock_source', 'unknown')
    time_src    = metas[0].get('time_source',  'unknown')

    # Header
    print("=" * 62)
    print("  Synchronization Analysis Report")
    print("=" * 62)
    print(f"  Reference    : {ref_serial}")
    print(f"  Clock source : {clock_src}")
    print(f"  Time source  : {time_src}")
    print(f"  Sample rate  : {args.rate / 1e6:.3f} MHz")
    print(f"  Pairs        : {n_pairs} (each vs. reference)")

    pair_results = []
    for i in range(1, len(files)):
        dev_serial = metas[i].get('serial', files[i])
        r = _analyze_pair(sigs[0], sigs[i], args.rate, args.tone_offset)
        r['ref_serial'] = ref_serial
        r['dev_serial'] = dev_serial
        pair_results.append(r)

        phase_coherence_std = float(np.std(r['phase_residuals']))
        time_offset_us = r['offset'] / args.rate * 1e6

        print("-" * 62)
        print(f"  Pair         : {ref_serial} → {dev_serial}")
        if r['snr_ref'] is not None:
            print(f"  Tone SNR (ref) : {r['snr_ref']:+.1f} dB")
            print(f"  Tone SNR (dev) : {r['snr_dev']:+.1f} dB")
            if r['low_snr']:
                print("  WARNING: SNR < 10 dB — freq/phase metrics unreliable.")
        print(f"  Time offset     : {r['offset']:+.3f} samples  ({time_offset_us:+.3f} µs)")
        print(f"  Freq offset     : {r['freq_offset']:+.3f} Hz")
        print(f"  Phase coherence : {phase_coherence_std:.3f} rad (stddev)")

        verdict_time  = "GOOD" if abs(time_offset_us) < 10 else "POOR"
        verdict_freq  = "N/A (low SNR)" if r['low_snr'] else ("GOOD" if abs(r['freq_offset']) < 10 else "POOR")
        verdict_phase = "N/A (low SNR)" if r['low_snr'] else ("GOOD" if phase_coherence_std < 0.5 else "POOR")
        print(f"  Time sync       : {verdict_time}  (threshold <10 µs)")
        print(f"  Freq sync       : {verdict_freq}  (threshold <10 Hz)")
        print(f"  Phase coherence : {verdict_phase}  (threshold <0.5 rad)")

        if verdict_time == "POOR" or verdict_freq == "POOR":
            if clock_src == 'internal' and time_src == 'internal':
                print("\n  HINT: No external sync. Use --time-source external + shared PPS for")
                print("  time alignment; add --clock-source external to eliminate freq offset.")
            elif clock_src == 'internal' and time_src in ('external', 'gpsdo'):
                print("\n  HINT: PPS-only — freq offset expected (independent TCXOs).")
                print("  Add --clock-source external with a shared 10 MHz reference.")

    print("=" * 62)

    # Plots — one figure per pair
    for i, r in enumerate(pair_results):
        phase_coherence_std = float(np.std(r['phase_residuals']))
        sp = _save_path_for_pair(args.save_plot, i, n_pairs)
        plot_results(
            r['xc'], r['lag'], r['offset'],
            r['phase_diff'], r['phase_unwrapped'], r['phase_residuals'],
            r['freq_offset'], args.rate,
            label_ref=r['ref_serial'], label_dev=r['dev_serial'],
            seg_times=r['seg_times'], save_path=sp,
        )


if __name__ == '__main__':
    main()
