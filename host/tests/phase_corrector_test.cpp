// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for phase_corrector.
// Validates that a known phase ramp is applied correctly and that the phase
// accumulator carries across consecutive calls.

#include <uhd/usrp/bonded/phase_corrector.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <complex>
#include <vector>

using namespace uhd::usrp::bonded;

static constexpr float TOLERANCE = 1e-4f;

// Generate a unit-magnitude DC signal (all samples = 1+0j)
static std::vector<std::complex<float>> dc_signal(size_t n)
{
    return std::vector<std::complex<float>>(n, {1.0f, 0.0f});
}

// Phase of a complex sample in radians
static float phase_of(std::complex<float> c)
{
    return std::atan2(c.imag(), c.real());
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_zero_drift_no_rotation)
{
    phase_corrector pc;
    auto sig = dc_signal(1024);

    // With 0 ppm drift, signal should be unchanged
    pc.correct(sig.data(), sig.size(), 0.0, 2.4e9, 20e6);

    for (size_t i = 0; i < sig.size(); ++i) {
        BOOST_CHECK_SMALL(sig[i].imag(), TOLERANCE);
        BOOST_CHECK_CLOSE(sig[i].real(), 1.0f, 0.01f);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_phase_ramp_direction)
{
    // 2 ppm drift at 2.4 GHz → freq_offset = 2e-6 * 2.4e9 = 4800 Hz
    // phase_inc_per_sample = -2π * 4800 / 20e6 = -1.508e-3 rad / sample
    // After 1000 samples, total rotation ≈ -1.508 rad
    phase_corrector pc;
    const double drift_ppm     = 2.0;
    const double rf_freq       = 2.4e9;
    const double sample_rate   = 20e6;
    const size_t nsamps        = 1000;

    auto sig = dc_signal(nsamps);
    pc.correct(sig.data(), nsamps, drift_ppm, rf_freq, sample_rate);

    // First sample gets no rotation (phase starts at 0)
    BOOST_CHECK_SMALL(phase_of(sig[0]), TOLERANCE);

    // Each subsequent sample should have a slightly more negative phase
    // (we're correcting positive drift → rotate backward)
    const double expected_inc =
        -2.0 * M_PI * (drift_ppm / 1e6) * rf_freq / sample_rate;
    for (size_t i = 1; i < nsamps; ++i) {
        float expected_phase = static_cast<float>(expected_inc * i);
        // Wrap to [-π, π]
        while (expected_phase >  static_cast<float>(M_PI)) expected_phase -= static_cast<float>(2.0 * M_PI);
        while (expected_phase < -static_cast<float>(M_PI)) expected_phase += static_cast<float>(2.0 * M_PI);
        BOOST_CHECK_CLOSE(phase_of(sig[i]), expected_phase, 0.1f);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_phase_continuity_across_calls)
{
    // Split a 2048-sample block into two 1024-sample calls.
    // The phase should be continuous across the boundary.
    const double drift_ppm   = 3.0;
    const double rf_freq     = 900e6;
    const double sample_rate = 10e6;
    const size_t half        = 1024;

    // Reference: correct all at once
    phase_corrector pc_ref;
    auto sig_full = dc_signal(2 * half);
    pc_ref.correct(sig_full.data(), 2 * half, drift_ppm, rf_freq, sample_rate);

    // Split into two calls (same corrector instance = same accumulator)
    phase_corrector pc_split;
    auto sig_split = dc_signal(2 * half);
    pc_split.correct(sig_split.data(),        half, drift_ppm, rf_freq, sample_rate);
    pc_split.correct(sig_split.data() + half, half, drift_ppm, rf_freq, sample_rate);

    // Results should be within 0.1% (Volk fp accumulation is not associative)
    for (size_t i = 0; i < 2 * half; ++i) {
        BOOST_CHECK_CLOSE(sig_full[i].real(), sig_split[i].real(), 0.1f);
        BOOST_CHECK_CLOSE(sig_full[i].imag(), sig_split[i].imag(), 0.1f);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_reset_phase_restarts_accumulator)
{
    phase_corrector pc;
    const double drift_ppm   = 2.0;
    const double rf_freq     = 2.4e9;
    const double sample_rate = 20e6;

    // Apply correction to first 100 samples
    auto sig1 = dc_signal(100);
    pc.correct(sig1.data(), 100, drift_ppm, rf_freq, sample_rate);

    // Reset and apply to another 100 samples from scratch
    pc.reset_phase();
    auto sig2 = dc_signal(100);
    pc.correct(sig2.data(), 100, drift_ppm, rf_freq, sample_rate);

    // After reset the phase trajectory should be identical to sig1
    for (size_t i = 0; i < 100; ++i) {
        BOOST_CHECK_CLOSE(sig1[i].real(), sig2[i].real(), 0.01f);
        BOOST_CHECK_CLOSE(sig1[i].imag(), sig2[i].imag(), 0.01f);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_magnitude_preserved)
{
    // Phase correction must not change sample magnitudes
    phase_corrector pc;
    const size_t N = 4096;

    // Use varying-magnitude input
    std::vector<std::complex<float>> sig(N);
    for (size_t i = 0; i < N; ++i) {
        float mag   = static_cast<float>(i % 10) + 1.0f;
        float theta = static_cast<float>(i) * 0.1f;
        sig[i] = {mag * std::cos(theta), mag * std::sin(theta)};
    }

    // Save magnitudes
    std::vector<float> mag_before(N);
    for (size_t i = 0; i < N; ++i) {
        mag_before[i] = std::abs(sig[i]);
    }

    pc.correct(sig.data(), N, 2.5, 1.5e9, 56e6);

    for (size_t i = 0; i < N; ++i) {
        BOOST_CHECK_CLOSE(std::abs(sig[i]), mag_before[i], 0.01f);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_empty_buffer_no_crash)
{
    phase_corrector pc;
    // Should not crash or modify state
    pc.correct(nullptr, 0, 2.0, 2.4e9, 20e6);
}
