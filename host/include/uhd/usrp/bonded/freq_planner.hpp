// Licensed under the same terms as UHD
#pragma once

#include <uhd/config.hpp>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

class UHD_API freq_planner
{
public:
    struct plan {
        double total_bandwidth_hz = 0.0;
        double overlap_hz = 0.0;
        std::vector<double> center_frequencies_hz;
        std::vector<double> per_device_rate_hz;
    };

    // Compute frequency plan for N devices
    static plan compute(
        double start_freq_hz,
        double total_bandwidth_hz,
        size_t num_devices,
        double per_device_rate_hz,
        double overlap_fraction = 0.1  // 10% overlap by default
    );
};

}}} // namespace uhd::usrp::bonded
