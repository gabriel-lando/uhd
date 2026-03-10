// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#include <uhd/usrp/bonded/spectrum_stitcher.hpp>
#include <uhd/exception.hpp>
#include <fftw3.h>
#include <cmath>
#include <stdexcept>

namespace uhd { namespace usrp { namespace bonded {

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

spectrum_stitcher::spectrum_stitcher(double sample_rate,
    size_t fft_size,
    double guard_band_hz,
    size_t num_devices)
    : _sample_rate(sample_rate), _fft_size(fft_size), _num_devices(num_devices)
{
    if (fft_size == 0 || (fft_size & (fft_size - 1)) != 0) {
        throw uhd::value_error("spectrum_stitcher: fft_size must be a power of two");
    }
    if (num_devices < 2) {
        throw uhd::value_error("spectrum_stitcher: num_devices must be >= 2");
    }

    _guard_band_hz = (guard_band_hz < 0.0)
                         ? (sample_rate * 0.10) // default: 10% per side
                         : guard_band_hz;

    const double bin_width_hz = sample_rate / static_cast<double>(fft_size);
    _guard_band_bins           = static_cast<size_t>(_guard_band_hz / bin_width_hz);

    // Usable bins per half = (fft_size/2) - 2*guard  (guard at DC and Nyquist sides)
    // Per device total = 2 halves
    const size_t bins_per_half = fft_size / 2 - 2 * _guard_band_bins;
    if (bins_per_half == 0) {
        throw uhd::value_error(
            "spectrum_stitcher: guard band too large for fft_size");
    }
    _usable_bins_per_device = bins_per_half * 2;
    _output_bins            = num_devices * _usable_bins_per_device;

    // Allocate per-device working buffers and FFTW plans
    _windowed.resize(num_devices);
    _spectrum.resize(num_devices);
    _plans.resize(num_devices, nullptr);

    for (size_t d = 0; d < num_devices; ++d) {
        _windowed[d].assign(fft_size * 2, 0.0f);
        _spectrum[d].assign(fft_size * 2, 0.0f);

        _plans[d] = reinterpret_cast<fftw_plan_s*>(
            fftwf_plan_dft_1d(static_cast<int>(fft_size),
                reinterpret_cast<fftwf_complex*>(_windowed[d].data()),
                reinterpret_cast<fftwf_complex*>(_spectrum[d].data()),
                FFTW_FORWARD,
                FFTW_ESTIMATE));

        if (!_plans[d]) {
            throw uhd::runtime_error(
                "spectrum_stitcher: failed to create FFTW plan for device "
                + std::to_string(d));
        }
    }

    _window.resize(fft_size);
    _create_blackman_harris_window();
}

spectrum_stitcher::~spectrum_stitcher()
{
    for (auto& p : _plans) {
        if (p) {
            fftwf_destroy_plan(reinterpret_cast<fftwf_plan>(p));
            p = nullptr;
        }
    }
}

// ---------------------------------------------------------------------------
// stitch()
// ---------------------------------------------------------------------------

void spectrum_stitcher::stitch(
    const std::vector<const std::complex<float>*>& inputs,
    size_t nsamps,
    std::complex<float>* output)
{
    if (inputs.size() < _num_devices) {
        throw uhd::value_error(
            "spectrum_stitcher::stitch: not enough input buffers");
    }
    if (nsamps < _fft_size) {
        throw uhd::value_error(
            "spectrum_stitcher::stitch: nsamps must be >= fft_size");
    }

    const size_t N  = _fft_size;
    const size_t gb = _guard_band_bins;

    // Window + FFT for each device
    for (size_t d = 0; d < _num_devices; ++d) {
        _apply_window(inputs[d], _windowed[d].data());
        fftwf_execute(reinterpret_cast<fftwf_plan>(_plans[d]));
    }

    // Copy usable bins into the output spectrum.
    //
    // FFT output layout (natural order):
    //   bin 0       = DC
    //   bin 1..N/2-1 = positive frequencies
    //   bin N/2..N-1 = negative frequencies
    //
    // For each device d, copy:
    //   positive usable bins: [gb .. N/2 - gb)   → physical [fc_d + gb_hz .. fc_d + R/2 - gb_hz]
    //   negative usable bins: [N/2+gb .. N - gb)  → physical [fc_d - R/2 + gb_hz .. fc_d - gb_hz]
    //
    // Devices are ordered from lowest to highest center frequency (device 0
    // is the most negative sub-band), so iterating d=0..N-1 in order produces
    // a spectrum that covers the full combined band from low to high.

    size_t out_idx = 0;
    for (size_t d = 0; d < _num_devices; ++d) {
        const float* sp = _spectrum[d].data();

        // Positive-frequency usable bins
        for (size_t i = gb; i < N / 2 - gb; ++i) {
            output[out_idx].real(sp[i * 2]);
            output[out_idx].imag(sp[i * 2 + 1]);
            ++out_idx;
        }
        // Negative-frequency usable bins
        for (size_t i = N / 2 + gb; i < N - gb; ++i) {
            output[out_idx].real(sp[i * 2]);
            output[out_idx].imag(sp[i * 2 + 1]);
            ++out_idx;
        }
    }
    // out_idx == _output_bins here
}

// ---------------------------------------------------------------------------
// Window
// ---------------------------------------------------------------------------

void spectrum_stitcher::_create_blackman_harris_window()
{
    // Four-term Blackman-Harris window
    const size_t N              = _fft_size;
    const double twopi_over_N   = 2.0 * M_PI / static_cast<double>(N - 1);
    for (size_t i = 0; i < N; ++i) {
        double x   = static_cast<double>(i);
        _window[i] = static_cast<float>(
            0.35875
            - 0.48829 * std::cos(1.0 * twopi_over_N * x)
            + 0.14128 * std::cos(2.0 * twopi_over_N * x)
            - 0.01168 * std::cos(3.0 * twopi_over_N * x));
    }
}

void spectrum_stitcher::_apply_window(
    const std::complex<float>* in, float* out) const
{
    for (size_t i = 0; i < _fft_size; ++i) {
        float w        = _window[i];
        out[i * 2]     = in[i].real() * w;
        out[i * 2 + 1] = in[i].imag() * w;
    }
}

}}} // namespace uhd::usrp::bonded

