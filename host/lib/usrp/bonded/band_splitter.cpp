//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Time-domain wideband band splitter — the TX counterpart of
// overlap_reconstructor.  A single combined wideband baseband stream is
// NCO-translated per radio and polyphase-decimated so each output is a
// per-radio stream ready to drive a B210 TX chain.
//
// DSP per radio i:
//   1. NCO translate by -radio_i_offset_hz (brings that radio's half-band
//      to DC in the baseband frame).
//   2. Polyphase decimate by input_rate / per_radio_rate (rational in general;
//      for the standard 2× case 20 MHz → 10 MHz: interp=1, decim=2).
//      The decimation AA filter's −6 dB point is at input_rate/4, exactly
//      the center seam between the two slices (the unused DC subcarrier in
//      802.11 and other direct-conversion OFDM systems — pure hard-split).
//

#include "band_splitter.hpp"
#include "bonded_resample.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>

#ifdef UHD_HAVE_VOLK
#    include <volk/volk.h>
#endif

namespace uhd { namespace usrp { namespace bonded {

namespace {
using namespace detail; // cf, kPi, poly_resampler, …
} // namespace

struct band_splitter::impl
{
    band_splitter_config cfg;
    size_t interp, decim;

    // NCO state: unit-phasor (phase accumulator) and per-sample increment.
    cf inc1, inc2;
    cf rot1{1.0f, 0.0f};
    cf rot2{1.0f, 0.0f};

    poly_resampler rs1, rs2;

    // Reusable scratch (avoids per-call heap churn at small GR chunk sizes).
    std::vector<cf> tmp1_, tmp2_;

    explicit impl(const band_splitter_config& c) : cfg(c)
    {
        const size_t g =
            std::gcd(static_cast<size_t>(std::llround(c.input_rate)),
                static_cast<size_t>(std::llround(c.per_radio_rate)));
        interp = static_cast<size_t>(std::llround(c.per_radio_rate)) / g;
        decim  = static_cast<size_t>(std::llround(c.input_rate)) / g;
        rs1.init(interp, decim);
        rs2.init(interp, decim);

        // Negate offset: shift the radio's slice UP to DC.
        const double w1 =
            -2.0 * kPi * c.radio1_offset_hz / c.input_rate;
        const double w2 =
            -2.0 * kPi * c.radio2_offset_hz / c.input_rate;
        inc1 = cf(static_cast<float>(std::cos(w1)),
            static_cast<float>(std::sin(w1)));
        inc2 = cf(static_cast<float>(std::cos(w2)),
            static_cast<float>(std::sin(w2)));
    }

    void rotate(std::vector<cf>& x, cf& phase, cf inc)
    {
#ifdef UHD_HAVE_VOLK
        volk_32fc_s32fc_x2_rotator_32fc(
            reinterpret_cast<lv_32fc_t*>(x.data()),
            reinterpret_cast<const lv_32fc_t*>(x.data()),
            *reinterpret_cast<const lv_32fc_t*>(&inc),
            reinterpret_cast<lv_32fc_t*>(&phase),
            static_cast<unsigned>(x.size()));
#else
        for (auto& v : x) {
            v *= phase;
            phase *= inc;
        }
        const float mag = std::abs(phase);
        if (mag > 0.f) {
            phase /= mag;
        }
#endif
    }

    void process(const cf* in,
        size_t n,
        std::vector<cf>& out1,
        std::vector<cf>& out2)
    {
        // Copy wideband into two scratch buffers for independent NCO rotation.
        tmp1_.assign(in, in + n);
        tmp2_.assign(in, in + n);

        // 1) NCO: translate each copy so its radio's slice is at DC.
        rotate(tmp1_, rot1, inc1);
        rotate(tmp2_, rot2, inc2);

        // 2) Polyphase decimate; the built-in AA filter is the split filter.
        rs1.feed(tmp1_.data(), n, out1);
        rs2.feed(tmp2_.data(), n, out2);
    }
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

band_splitter::band_splitter(const band_splitter_config& cfg) : _cfg(cfg)
{
    if (cfg.input_rate <= 0 || cfg.per_radio_rate <= 0) {
        throw std::invalid_argument("band_splitter: sample rates must be > 0");
    }
    if (cfg.per_radio_rate > cfg.input_rate) {
        throw std::invalid_argument(
            "band_splitter: per_radio_rate must be <= input_rate");
    }
}

std::pair<std::vector<std::complex<float>>, std::vector<std::complex<float>>>
band_splitter::split(const std::vector<std::complex<float>>& wideband)
{
    reset();
    std::vector<std::complex<float>> out1, out2;
    _impl->process(wideband.data(), wideband.size(), out1, out2);
    return {std::move(out1), std::move(out2)};
}

void band_splitter::process(const std::complex<float>* in,
    size_t n,
    std::vector<std::complex<float>>& out1,
    std::vector<std::complex<float>>& out2)
{
    if (!_impl) {
        _impl = std::make_shared<impl>(_cfg);
    }
    _impl->process(in, n, out1, out2);
}

void band_splitter::reset()
{
    _impl = std::make_shared<impl>(_cfg);
}

}}} // namespace uhd::usrp::bonded
