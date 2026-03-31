// Licensed under the same terms as UHD
#pragma once

#include <uhd/config.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/bonded/sync_manager.hpp>
#include <uhd/usrp/bonded/freq_planner.hpp>
#include <complex>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

class UHD_API bonded_rx_streamer
{
public:
    struct config {
        double total_bandwidth_hz = 0.0;
        double overlap_fraction = 0.1;
        std::string sample_format = "fc32";  // CPU format
        std::string otw_format   = "sc16";   // Over-the-wire format
        size_t spp = 0;                      // Samples per packet (0 = auto)
    };

    bonded_rx_streamer(
        uhd::usrp::multi_usrp::sptr usrp,
        const sync_manager& sync,
        const config& cfg
    );

    // Start synchronized reception across all devices
    void start(const uhd::time_spec_t& start_time);

    // Receive aligned samples from all devices
    // Returns number of samples received per device
    size_t recv(
        std::vector<std::vector<std::complex<float>>>& buffs,
        size_t nsamps_per_device,
        uhd::rx_metadata_t& md,
        double timeout = 1.0
    );

    // Stop reception
    void stop();

    // Get the frequency plan in use
    freq_planner::plan get_freq_plan() const { return _plan; }

private:
    uhd::usrp::multi_usrp::sptr _usrp;
    std::vector<uhd::rx_streamer::sptr> _streams;
    freq_planner::plan _plan;
    config _cfg;
    const sync_manager& _sync;
};

}}} // namespace uhd::usrp::bonded
