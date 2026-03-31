// Licensed under the same terms as UHD

#include <uhd/usrp/bonded/spectrum_stitcher.hpp>
#include <cmath>
#include <algorithm>

namespace uhd { namespace usrp { namespace bonded {

spectrum_stitcher::spectrum_stitcher(const config& cfg) : _cfg(cfg)
{
    // create Hann window for fft_size
    _window.resize(_cfg.fft_size);
    for (size_t n = 0; n < _cfg.fft_size; ++n) {
        _window[n] = 0.5f * (1.0f - std::cos(2.0 * M_PI * n / (_cfg.fft_size - 1)));
    }
}

std::vector<float> spectrum_stitcher::stitch(
    const std::vector<std::vector<std::complex<float>>>& per_device_samples)
{
    // Very small reference implementation: compute per-device PSD via simple periodogram
    size_t devices = per_device_samples.size();
    if (devices == 0) return {};
    size_t N = _cfg.fft_size;
    std::vector<float> stitched(N * devices, 0.0f);
    for (size_t d = 0; d < devices; ++d) {
        const auto& samples = per_device_samples[d];
        // accumulate magnitude-squared of first N samples
        for (size_t k = 0; k < N && k < samples.size(); ++k) {
            float win = _window[k];
            auto s = samples[k] * win;
            stitched[d*N + k] = std::norm(s);
        }
    }
    return stitched;
}

std::vector<double> spectrum_stitcher::get_frequency_axis() const
{
    size_t devices = _cfg.center_freqs_hz.size();
    std::vector<double> axis(_cfg.fft_size * (devices ? devices : 1), 0.0);
    const double df = _cfg.sample_rate_hz / static_cast<double>(_cfg.fft_size);
    for (size_t d = 0; d < (devices?devices:1); ++d) {
        double center = (devices? _cfg.center_freqs_hz[d] : 0.0);
        for (size_t k = 0; k < _cfg.fft_size; ++k) {
            axis[d*_cfg.fft_size + k] = center - _cfg.sample_rate_hz/2.0 + df * k;
        }
    }
    return axis;
}

}}} // namespace uhd::usrp::bonded
