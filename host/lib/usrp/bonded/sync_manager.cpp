// Licensed under the same terms as UHD

#include <uhd/usrp/bonded/sync_manager.hpp>
#include <uhd/types/sensors.hpp>
#include <chrono>
#include <thread>
#include <algorithm>

namespace uhd { namespace usrp { namespace bonded {

sync_manager::sync_manager(uhd::usrp::multi_usrp::sptr usrp, const config& cfg)
    : _usrp(usrp), _cfg(cfg)
{
    _num_mboards = (_usrp) ? _usrp->get_num_mboards() : 0;
}

void sync_manager::configure_sources()
{
    if (!_usrp) return;
    for (size_t m = 0; m < _num_mboards; ++m) {
        // Prefer to set both clock and time together where available
        try {
            _usrp->set_sync_source(_cfg.clock_source, _cfg.time_source, m);
        } catch (...) {
            // Fallback to separate calls
            try { _usrp->set_clock_source(_cfg.clock_source, m); } catch (...) {}
            try { _usrp->set_time_source(_cfg.time_source, m); } catch (...) {}
        }
    }
}

bool sync_manager::wait_for_lock()
{
    if (!_usrp) return false;
    using clock = std::chrono::steady_clock;
    auto start = clock::now();
    while (std::chrono::duration<double>(clock::now() - start).count() < _cfg.lock_timeout_s) {
        bool all_locked = true;
        for (size_t m = 0; m < _num_mboards; ++m) {
            try {
                uhd::sensor_value_t s = _usrp->get_mboard_sensor("ref_locked", m);
                if (!s.to_bool()) { all_locked = false; break; }
            } catch (...) {
                all_locked = false; break;
            }
        }
        if (all_locked) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

void sync_manager::synchronize_time()
{
    if (!_usrp) return;
    // If using external/GPSDO time source prefer unknown_pps sync which
    // catches a PPS edge and sets time synchronously across boards.
    const std::string ts = _cfg.time_source;
    if (ts == "external" || ts == "gpsdo") {
        _usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    } else {
        _usrp->set_time_now(uhd::time_spec_t(0.0));
    }
    // Allow time for settling
    std::this_thread::sleep_for(std::chrono::duration<double>(_cfg.sync_settle_s));
}

sync_manager::status sync_manager::verify_sync()
{
    status st;
    if (!_usrp) return st;
    st.per_device_locked.resize(_num_mboards, false);
    st.pps_deltas_us.resize(_num_mboards, 0.0);

    // Read last PPS timestamp for each motherboard
    std::vector<double> pps_secs(_num_mboards, 0.0);
    for (size_t m = 0; m < _num_mboards; ++m) {
        try {
            uhd::time_spec_t t = _usrp->get_time_last_pps(m);
            pps_secs[m] = t.get_real_secs();
        } catch (...) {
            pps_secs[m] = 0.0;
        }
        try {
            uhd::sensor_value_t s = _usrp->get_mboard_sensor("ref_locked", m);
            st.per_device_locked[m] = s.to_bool();
        } catch (...) {
            st.per_device_locked[m] = false;
        }
    }

    if (_num_mboards == 0) return st;
    double ref = pps_secs[0];
    double max_delta = 0.0;
    for (size_t m = 0; m < _num_mboards; ++m) {
        double delta = (pps_secs[m] - ref) * 1e6; // microseconds
        st.pps_deltas_us[m] = delta;
        if (std::abs(delta) > max_delta) max_delta = std::abs(delta);
    }
    st.max_pps_delta_us = max_delta;
    st.all_locked = std::all_of(st.per_device_locked.begin(), st.per_device_locked.end(), [](bool v){ return v; });
    return st;
}

}}} // namespace uhd::usrp::bonded
