// Licensed under the same terms as UHD
#pragma once

#include <uhd/config.hpp>
#include <complex>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

class UHD_API spectrum_stitcher
{
public:
    struct config {
        size_t fft_size = 4096;
        double overlap_hz = 0.0;
        std::vector<double> center_freqs_hz;
        double sample_rate_hz = 0.0;
    };

    explicit spectrum_stitcher(const config& cfg);

    // Stitch per-device time-domain samples into a wideband spectrum
    std::vector<float> stitch(
        const std::vector<std::vector<std::complex<float>>>& per_device_samples
    );

    // Get the frequency axis for the stitched spectrum
    std::vector<double> get_frequency_axis() const;

private:
    config _cfg;
    std::vector<float> _window;
};

}}} // namespace uhd::usrp::bonded
