//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <cstddef>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

struct adjacent_frequency_plan
{
    std::vector<double> centers_hz;
    double step_hz            = 0.0;
    double per_device_bw_hz   = 0.0;
    double overlap_fraction   = 0.0;
};

/*!
 * Build an adjacent-band frequency plan centered around center_freq_hz.
 *
 * For N devices with per-device bandwidth BW and overlap fraction O:
 *   step = BW * (1 - O)
 * Bands are centered symmetrically around center_freq_hz.
 */
UHD_API adjacent_frequency_plan make_adjacent_frequency_plan(size_t num_devices,
    double center_freq_hz,
    double per_device_bw_hz,
    double overlap_fraction);

}}} // namespace uhd::usrp::bonded
