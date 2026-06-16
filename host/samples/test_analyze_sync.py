#!/usr/bin/env python3
#
# Copyright 2026 Gabriel Lando
# SPDX-License-Identifier: GPL-3.0-or-later
#

import os

import numpy as np
import pytest

import analyze_sync


def _make_random_iq(n, seed=1234):
    rng = np.random.default_rng(seed)
    x = rng.standard_normal(n) + 1j * rng.standard_normal(n)
    return x.astype(np.complex64)


def _shift_with_zeros(x, shift):
    """Return y where positive shift means y is delayed vs x by `shift` samples."""
    if shift == 0:
        return x.copy()
    if shift > 0:
        return np.concatenate([
            np.zeros(shift, dtype=x.dtype),
            x[:-shift],
        ])
    shift = -shift
    return np.concatenate([
        x[shift:],
        np.zeros(shift, dtype=x.dtype),
    ])


def test_cross_correlation_known_time_offset():
    rate = 10e6
    x = _make_random_iq(32768, seed=1)
    y = _shift_with_zeros(x, shift=5)  # y delayed by 5 samples

    offset, _, _ = analyze_sync.cross_correlation(x, y)
    time_offset_us = offset / rate * 1e6

    # Sign convention in analyze_sync: delayed y -> negative lag.
    assert abs(offset + 5.0) < 0.2
    assert abs(time_offset_us + 0.5) < 0.05


def test_estimate_freq_and_phase_offset_known_ground_truth():
    rate = 1e6
    n = 200000
    t = np.arange(n) / rate

    f0 = 80e3
    df = 120.0
    phi = 0.7

    x = np.exp(1j * 2 * np.pi * f0 * t).astype(np.complex64)
    y = np.exp(1j * (2 * np.pi * (f0 + df) * t + phi)).astype(np.complex64)

    freq_offset, phase_diff, _, phase_residuals = analyze_sync.estimate_freq_offset(x, y, rate)

    # estimate_freq_offset computes angle(x * conj(y)), so the expected sign is -df.
    assert abs(freq_offset + df) < 1.0

    # For equal-length pure tones, wrapped phase starts near -phi.
    wrapped_phase0 = np.angle(np.exp(1j * phase_diff[0]))
    wrapped_expect = np.angle(np.exp(-1j * phi))
    assert abs(wrapped_phase0 - wrapped_expect) < 0.05

    # Pure tone model should be highly coherent after removing linear trend.
    assert float(np.std(phase_residuals)) < 0.05


def test_segmented_fft_tone_estimator_with_noise():
    rate = 1e6
    n = 300000
    t = np.arange(n) / rate

    tone = 100e3
    # Keep |df| below 1/(2*seg_dur)=10 Hz (seg_dur=50 ms in estimator),
    # otherwise the segmented phase progression aliases.
    df = 4.0
    phi = 0.3

    rng = np.random.default_rng(42)
    noise_scale = 1.5

    x = np.exp(1j * 2 * np.pi * tone * t) + noise_scale * (
        rng.standard_normal(n) + 1j * rng.standard_normal(n)
    )
    y = np.exp(1j * (2 * np.pi * (tone + df) * t + phi)) + noise_scale * (
        rng.standard_normal(n) + 1j * rng.standard_normal(n)
    )

    out = analyze_sync._analyze_pair(
        x.astype(np.complex64),
        y.astype(np.complex64),
        rate,
        tone_offset=tone,
    )

    # Sign convention matches estimate_freq_offset(): x * conj(y) => -df.
    assert abs(out["freq_offset"] + df) < 1.0, \
        f"Expected {-df} Hz, got {out['freq_offset']:.3f} Hz"
    assert out["seg_times"] is not None
    assert np.isfinite(out["offset"])


@pytest.mark.skipif(
    "UHD_SYNC_REGRESSION_FILES" not in os.environ,
    reason="Set UHD_SYNC_REGRESSION_FILES='ref.npy:dev.npy' to run real-capture regression.",
)
def test_regression_real_capture_dataset():
    files = os.environ["UHD_SYNC_REGRESSION_FILES"].split(":")
    assert len(files) == 2, "UHD_SYNC_REGRESSION_FILES must be 'ref.npy:dev.npy'"

    ref_path, dev_path = files
    assert os.path.exists(ref_path), f"Missing regression file: {ref_path}"
    assert os.path.exists(dev_path), f"Missing regression file: {dev_path}"

    rate = float(os.environ.get("UHD_SYNC_REGRESSION_RATE", "1000000"))
    tone_env = os.environ.get("UHD_SYNC_REGRESSION_TONE")
    tone_offset = float(tone_env) if tone_env else None

    x = np.load(ref_path)
    y = np.load(dev_path)

    out = analyze_sync._analyze_pair(x, y, rate, tone_offset=tone_offset)

    # Broad invariants for regression stability.
    assert np.isfinite(out["offset"])
    assert np.isfinite(out["freq_offset"])
    assert np.isfinite(np.std(out["phase_residuals"]))
    assert abs(out["offset"] / rate * 1e6) < 20.0
