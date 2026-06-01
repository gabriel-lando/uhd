//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "frequency_plan.hpp"

#include <stdexcept>

namespace uhd { namespace usrp { namespace bonded {

adjacent_frequency_plan make_adjacent_frequency_plan(size_t num_devices,
    double center_freq_hz,
    double per_device_bw_hz,
    double overlap_fraction)
{
    if (num_devices == 0) {
        throw std::invalid_argument("num_devices must be > 0");
    }
    if (per_device_bw_hz <= 0.0) {
        throw std::invalid_argument("per_device_bw_hz must be > 0");
    }
    if (overlap_fraction < 0.0 || overlap_fraction >= 1.0) {
        throw std::invalid_argument("overlap_fraction must be in [0, 1)");
    }

    adjacent_frequency_plan plan;
    plan.per_device_bw_hz = per_device_bw_hz;
    plan.overlap_fraction = overlap_fraction;
    plan.step_hz          = per_device_bw_hz * (1.0 - overlap_fraction);
    plan.centers_hz.resize(num_devices);

    if (num_devices == 1) {
        plan.centers_hz[0] = center_freq_hz;
        return plan;
    }

    const double start = center_freq_hz
                         - 0.5 * static_cast<double>(num_devices - 1) * plan.step_hz;
    for (size_t i = 0; i < num_devices; i++) {
        plan.centers_hz[i] = start + static_cast<double>(i) * plan.step_hz;
    }

    return plan;
}

}}} // namespace uhd::usrp::bonded
