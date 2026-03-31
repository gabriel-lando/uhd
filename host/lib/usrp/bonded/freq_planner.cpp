// Licensed under the same terms as UHD

#include <uhd/usrp/bonded/freq_planner.hpp>

namespace uhd { namespace usrp { namespace bonded {

freq_planner::plan freq_planner::compute(
    double start_freq_hz,
    double total_bandwidth_hz,
    size_t num_devices,
    double per_device_rate_hz,
    double overlap_fraction)
{
    plan p;
    if (num_devices == 0) return p;
    p.total_bandwidth_hz = total_bandwidth_hz;
    p.overlap_hz = per_device_rate_hz * overlap_fraction;
    p.per_device_rate_hz.assign(num_devices, per_device_rate_hz);

    // Simple tiling of the total bandwidth into num_devices segments
    double segment = total_bandwidth_hz / static_cast<double>(num_devices);
    p.center_frequencies_hz.resize(num_devices);
    for (size_t i = 0; i < num_devices; ++i) {
        double seg_start = start_freq_hz + segment * static_cast<double>(i);
        p.center_frequencies_hz[i] = seg_start + segment / 2.0;
    }

    return p;
}

}}} // namespace
