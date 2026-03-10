// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#include <uhd/usrp/bonded/phase_corrector.hpp>
#include <volk/volk.h>
#include <cmath>

namespace uhd { namespace usrp { namespace bonded {

phase_corrector::phase_corrector()
{
    reset_phase();
}

void phase_corrector::reset_phase()
{
    _phase_real = 1.0f;
    _phase_imag = 0.0f;
}

void phase_corrector::correct(std::complex<float>* samples,
    size_t nsamps,
    double drift_ppm,
    double rf_center_freq,
    double sample_rate)
{
    if (nsamps == 0) {
        return;
    }

    // Frequency offset at RF = drift_ppm * center_freq / 1e6
    // This appears in baseband as a phase rotation; we apply the
    // conjugate rotation to cancel it.
    const double freq_offset_hz       = (drift_ppm / 1e6) * rf_center_freq;
    const double phase_inc_per_sample = -2.0 * M_PI * freq_offset_hz / sample_rate;

    // Phase-increment phasor as a contiguous complex<float> so the
    // reinterpret_cast to lv_32fc_t* reads valid {real, imag} bytes.
    const std::complex<float> phase_inc_phasor(
        static_cast<float>(std::cos(phase_inc_per_sample)),
        static_cast<float>(std::sin(phase_inc_per_sample)));

    // volk_32fc_s32fc_x2_rotator2_32fc signature:
    //   (out, in, phase_inc*, phase*, num_points)
    // Operates in-place (out == in is valid).
    volk_32fc_s32fc_x2_rotator2_32fc(
        reinterpret_cast<lv_32fc_t*>(samples),
        reinterpret_cast<const lv_32fc_t*>(samples),
        reinterpret_cast<const lv_32fc_t*>(&phase_inc_phasor),
        reinterpret_cast<lv_32fc_t*>(&_phase_real),
        static_cast<unsigned int>(nsamps));
    // _phase_real/_phase_imag are updated in-place by the Volk kernel,
    // so the next call automatically continues from where this one left off.
}

}}} // namespace uhd::usrp::bonded
