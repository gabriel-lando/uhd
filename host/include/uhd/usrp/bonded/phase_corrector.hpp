// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <uhd/config.hpp>
#include <complex>
#include <cstddef>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Applies a per-sample complex phase rotation to a buffer of fc32 samples,
 * compensating for the baseband frequency offset that results from the
 * oscillator drift between two USRP boards.
 *
 * Uses volk_32fc_s32fc_x2_rotator2_32fc for SIMD-accelerated operation.
 * The internal phase accumulator is carried between successive calls so
 * that phase continuity is maintained across buffer boundaries.
 *
 * Implementation note:
 *   freq_offset_hz = (drift_ppm / 1e6) * rf_center_freq
 *   phase_inc_per_sample = -2π * freq_offset_hz / sample_rate
 *   (negative sign = we rotate backwards to undo the positive drift)
 */
class UHD_API phase_corrector
{
public:
    phase_corrector();

    /*!
     * Apply correction in-place.
     *
     * \param samples         Buffer of complex float samples (modified in place).
     * \param nsamps          Number of samples in the buffer.
     * \param drift_ppm       Estimated frequency offset of the device whose
     *                        samples are being corrected, relative to the
     *                        reference device.
     * \param rf_center_freq  RF center frequency of the device (Hz).
     * \param sample_rate     Sample rate of the device (S/s).
     */
    void correct(std::complex<float>* samples,
        size_t nsamps,
        double drift_ppm,
        double rf_center_freq,
        double sample_rate);

    /*!
     * Reset the internal phase accumulator to zero (e.g. after a PPS re-sync
     * or a gap in the stream that resets alignment).
     */
    void reset_phase();

private:
    // Stored as separate floats matching lv_32fc_t layout (real, imag)
    float _phase_real = 1.0f;
    float _phase_imag = 0.0f;
};

}}} // namespace uhd::usrp::bonded
