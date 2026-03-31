// Licensed under the same terms as UHD

#include <uhd/usrp/bonded/drift_monitor.hpp>
#include <uhd/types/time_spec.hpp>
#include <chrono>

namespace uhd { namespace usrp { namespace bonded {

drift_monitor::drift_monitor(uhd::usrp::multi_usrp::sptr usrp)
    : _usrp(usrp), _start_time(std::chrono::steady_clock::now())
{
    if (_usrp) {
        size_t n = _usrp->get_num_mboards();
        _last_pps_secs.assign(n, 0.0);
        _last_drift_ppm.assign(n, 0.0);
        // initialize
        measure();
    }
}

drift_monitor::measurement drift_monitor::measure()
{
    measurement m;
    if (!_usrp) return m;
    auto now = std::chrono::steady_clock::now();
    m.elapsed_s = std::chrono::duration<double>(now - _start_time).count();

    size_t n = _usrp->get_num_mboards();
    m.drift_ppm.resize(n, 0.0);
    m.pps_ticks.resize(n, 0);

    std::vector<double> pps_secs(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        try {
            uhd::time_spec_t t = _usrp->get_time_last_pps(i);
            pps_secs[i] = t.get_real_secs();
            m.pps_ticks[i] = t.to_ticks(1.0); // tick_rate=1 -> seconds as ticks
        } catch (...) {
            pps_secs[i] = 0.0;
            m.pps_ticks[i] = 0;
        }
    }

    if (n == 0) return m;
    double ref = pps_secs[0];
    for (size_t i = 0; i < n; ++i) {
        double delta = pps_secs[i] - ref;
        double drift_ppm = 0.0;
        if (m.elapsed_s > 0.0) drift_ppm = (delta / m.elapsed_s) * 1e6;
        m.drift_ppm[i] = drift_ppm;
        _last_pps_secs[i] = pps_secs[i];
        _last_drift_ppm[i] = drift_ppm;
    }

    return m;
}

double drift_monitor::get_drift_ppm(size_t device_index) const
{
    if (device_index >= _last_drift_ppm.size()) return 0.0;
    return _last_drift_ppm[device_index];
}

}}} // namespace
