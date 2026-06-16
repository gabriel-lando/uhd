//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <complex>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Configuration for band_splitter.
 *
 * Mirrors overlap_reconstructor_config but inverted: the combined wideband
 * stream is the input and per-radio streams are the outputs.
 *
 * Standard 20 MHz / two-radio setup:
 *   input_rate       = 20e6
 *   per_radio_rate   = 10e6
 *   radio1_offset_hz = -5e6   (lower / Radio A)
 *   radio2_offset_hz = +5e6   (upper / Radio B)
 *
 * The split is performed by NCO-translating each half-band slice to DC and
 * polyphase-decimating.  The decimation AA filter has its −6 dB point at
 * input_rate/4 (= exactly the center seam), which is the unused DC subcarrier
 * in 802.11 and other direct-conversion OFDM systems.
 */
struct band_splitter_config
{
    double input_rate       = 20e6; //!< Wideband input rate (Hz)
    double per_radio_rate   = 10e6; //!< Per-radio output rate (Hz)
    double radio1_offset_hz = -5e6; //!< radio1 center − fc (Hz), lower slice
    double radio2_offset_hz = +5e6; //!< radio2 center − fc (Hz), upper slice
};

class UHD_API band_splitter
{
public:
    explicit band_splitter(const band_splitter_config& cfg);

    /*!
     * One-shot split of a finite wideband capture (offline / unit-test).
     * Returns {slice1, slice2} at per_radio_rate.
     */
    std::pair<std::vector<std::complex<float>>, std::vector<std::complex<float>>>
    split(const std::vector<std::complex<float>>& wideband);

    /*!
     * Streaming split (real-time / GNU Radio).
     * Consumes n samples from the wideband input (at input_rate) and appends
     * the resulting per-radio samples (at per_radio_rate) to out1 and out2.
     * Call reset() to start a new stream.
     */
    void process(const std::complex<float>* in,
        size_t n,
        std::vector<std::complex<float>>& out1,
        std::vector<std::complex<float>>& out2);

    //! Clear all streaming state (starts a fresh filter history).
    void reset();

    const band_splitter_config& config() const { return _cfg; }

private:
    struct impl;

    band_splitter_config _cfg;
    std::shared_ptr<impl> _impl;
};

}}} // namespace uhd::usrp::bonded
