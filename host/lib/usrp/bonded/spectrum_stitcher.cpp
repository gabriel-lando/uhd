//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "spectrum_stitcher.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace uhd { namespace usrp { namespace bonded {

static constexpr double kPi = 3.14159265358979323846;

std::vector<float> stitch_power_spectra(const std::vector<std::vector<float>>& spectra,
    size_t overlap_bins)
{
    if (spectra.empty()) {
        return {};
    }

    const size_t num_dev = spectra.size();
    const size_t bins    = spectra.front().size();
    if (bins == 0) {
        return {};
    }
    for (size_t d = 1; d < num_dev; d++) {
        if (spectra[d].size() != bins) {
            throw std::invalid_argument("all spectra must have the same bin count");
        }
    }
    if (overlap_bins >= bins) {
        throw std::invalid_argument("overlap_bins must be smaller than per-device bins");
    }

    const size_t stitched_bins = num_dev * bins - (num_dev - 1) * overlap_bins;
    std::vector<float> out;
    out.reserve(stitched_bins);

    // First band copied in full.
    out.insert(out.end(), spectra[0].begin(), spectra[0].end());

    for (size_t d = 1; d < num_dev; d++) {
        // Replace previous band's tail with crossfaded overlap.
        if (overlap_bins > 0) {
            const size_t tail_start = out.size() - overlap_bins;
            for (size_t k = 0; k < overlap_bins; k++) {
                const float prev = out[tail_start + k];
                const float next = spectra[d][k];
                const double x   = (static_cast<double>(k) + 0.5)
                                 / static_cast<double>(overlap_bins);
                const float w_prev = static_cast<float>(std::cos(0.5 * kPi * x));
                const float w_next = static_cast<float>(std::sin(0.5 * kPi * x));
                out[tail_start + k] = prev * w_prev * w_prev + next * w_next * w_next;
            }
        }

        // Append non-overlapping tail from the new band.
        out.insert(out.end(),
            spectra[d].begin() + static_cast<std::ptrdiff_t>(overlap_bins),
            spectra[d].end());
    }

    return out;
}

std::vector<float> stitch_power_spectra_fraction(
    const std::vector<std::vector<float>>& spectra, double overlap_fraction)
{
    if (spectra.empty() || spectra.front().empty()) {
        return {};
    }
    const double f = std::clamp(overlap_fraction, 0.0, 0.95);
    const size_t bins = spectra.front().size();
    const size_t overlap_bins = static_cast<size_t>(std::llround(f * bins));
    return stitch_power_spectra(spectra, overlap_bins);
}

}}} // namespace uhd::usrp::bonded
