// Licensed under the same terms as UHD
#pragma once

#include <uhd/config.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <chrono>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

class UHD_API drift_monitor
{
public:
    struct measurement {
        double elapsed_s = 0.0;
        std::vector<double> drift_ppm;    // Per-device relative to device 0
        std::vector<long long> pps_ticks;
    };

    drift_monitor(uhd::usrp::multi_usrp::sptr usrp);

    // Take a single drift measurement
    measurement measure();

    // Get the latest drift estimate for a specific device
    double get_drift_ppm(size_t device_index) const;

private:
    uhd::usrp::multi_usrp::sptr _usrp;
    std::chrono::steady_clock::time_point _start_time;
    std::vector<double> _last_pps_secs;
    std::vector<double> _last_drift_ppm;
};

}}} // namespace uhd::usrp::bonded
