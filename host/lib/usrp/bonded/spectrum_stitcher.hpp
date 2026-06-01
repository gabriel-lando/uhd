//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <cstddef>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Stitch ordered per-device power spectra using overlap-and-crossfade.
 *
 * spectra[d][k] is the k-th bin for device d, with devices ordered
 * from lower to higher center frequency.
 */
UHD_API std::vector<float> stitch_power_spectra(
    const std::vector<std::vector<float>>& spectra,
    size_t overlap_bins);

/*!
 * Fractional-overlap convenience wrapper.
 * overlap_fraction is clamped to [0, 0.95].
 */
UHD_API std::vector<float> stitch_power_spectra_fraction(
    const std::vector<std::vector<float>>& spectra, double overlap_fraction);

}}} // namespace uhd::usrp::bonded
