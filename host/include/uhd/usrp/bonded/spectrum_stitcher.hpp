// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <uhd/config.hpp>
#include <complex>
#include <cstddef>
#include <memory>
#include <vector>

// Forward-declare the FFTW plan type to avoid pulling in fftw3.h in the header.
struct fftw_plan_s;

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Combines N adjacent frequency-domain slices (one per bonded USRP device)
 * into a single stitched wideband spectrum.
 *
 * Each buffer of time-domain samples is windowed (Blackman-Harris),
 * forward-FFT'd, and placed into the output spectrum buffer.  Guard-band
 * bins at each band edge are discarded to account for analog filter
 * roll-off (~10% of bandwidth per edge by default).
 *
 * Device 0 provides the lowest sub-band; device N-1 provides the highest.
 * Within each device the positive-frequency bins are placed first, followed
 * by the negative-frequency bins, giving a contiguous output spectrum
 * arranged from (center - N*rate/2) to (center + N*rate/2).
 */
class UHD_API spectrum_stitcher
{
public:
    /*!
     * \param sample_rate      Per-device sample rate (S/s).
     * \param fft_size         FFT size per device (must be a power of two).
     * \param guard_band_hz    One-sided guard band in Hz.  Bins within this
     *                         range of each edge are dropped.  Pass a negative
     *                         value to use the default (10% of sample_rate).
     * \param num_devices      Number of devices / sub-bands to stitch.
     */
    spectrum_stitcher(double sample_rate,
        size_t fft_size,
        double guard_band_hz = -1.0,
        size_t num_devices   = 2);

    ~spectrum_stitcher();

    // Non-copyable; owns FFTW plans.
    spectrum_stitcher(const spectrum_stitcher&)            = delete;
    spectrum_stitcher& operator=(const spectrum_stitcher&) = delete;

    /*!
     * Stitch N time-domain buffers into one output spectrum.
     *
     * \param inputs   Per-device time-domain sample buffers, phase-corrected
     *                 (inputs[0] is the reference, already correct; inputs[i>0]
     *                 have had phase correction applied by phase_corrector).
     *                 Each buffer must contain at least fft_size() samples.
     * \param nsamps   Number of samples to consume from each buffer.
     * \param output   Output spectrum.  Must hold at least output_bins()
     *                 complex float values.
     */
    void stitch(const std::vector<const std::complex<float>*>& inputs,
        size_t nsamps,
        std::complex<float>* output);

    //! Total number of complex output bins after stitching.
    size_t output_bins() const { return _output_bins; }

    size_t fft_size() const { return _fft_size; }
    size_t num_devices() const { return _num_devices; }
    double guard_band_hz() const { return _guard_band_hz; }

    //! Usable bins contributed by each device (= 2*(fft_size/2 - 2*guard_bins)).
    size_t usable_bins_per_device() const { return _usable_bins_per_device; }

private:
    double _sample_rate;
    size_t _fft_size;
    size_t _num_devices;
    double _guard_band_hz;
    size_t _guard_band_bins;          //!< Guard bins per band edge (each half)
    size_t _usable_bins_per_device;   //!< Usable bins per device = N - 4*_guard_band_bins
    size_t _output_bins;              //!< = num_devices * usable_bins_per_device

    std::vector<float> _window;

    // Per-device working buffers (float interleaved for fftwf)
    std::vector<std::vector<float>> _windowed; // [dev][bin*2]
    std::vector<std::vector<float>> _spectrum; // [dev][bin*2]

    // Per-device FFTW plans (single-precision)
    std::vector<fftw_plan_s*> _plans;

    void _create_blackman_harris_window();
    void _apply_window(const std::complex<float>* in, float* out) const;
};

}}} // namespace uhd::usrp::bonded
