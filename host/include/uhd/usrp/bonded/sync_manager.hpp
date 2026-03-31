// Licensed under the same terms as UHD
#pragma once

#include <uhd/config.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/time_spec.hpp>
#include <string>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

class UHD_API sync_manager
{
public:
    struct config {
        std::string clock_source;  // "internal", "external", "gpsdo"
        std::string time_source;   // "internal", "external", "gpsdo"
        double lock_timeout_s = 5.0;
        double sync_settle_s  = 2.0;
    };

    struct status {
        bool all_locked = false;
        std::vector<bool> per_device_locked;
        std::vector<double> pps_deltas_us;
        double max_pps_delta_us = 0.0;
    };

    sync_manager(uhd::usrp::multi_usrp::sptr usrp, const config& cfg);

    // Configure clock and time sources on all motherboards
    void configure_sources();

    // Wait for all reference oscillators to lock
    bool wait_for_lock();

    // Synchronize time registers across all devices
    void synchronize_time();

    // Verify synchronization by comparing PPS timestamps
    status verify_sync();

private:
    uhd::usrp::multi_usrp::sptr _usrp;
    config _cfg;
    size_t _num_mboards;
};

}}} // namespace uhd::usrp::bonded
