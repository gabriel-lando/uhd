// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#include <uhd/usrp/bonded/pps_drift_estimator.hpp>
#include <uhd/utils/log.hpp>
#include <cmath>
#include <stdexcept>

namespace uhd { namespace usrp { namespace bonded {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

pps_drift_estimator::pps_drift_estimator(size_t num_devices, size_t max_history)
    : _num_devices(num_devices), _max_history(max_history)
{
    if (num_devices < 2) {
        throw std::invalid_argument(
            "pps_drift_estimator: num_devices must be >= 2");
    }

    // Pre-fill result vector; device 0 is always the reference.
    _latest.resize(num_devices);
    for (auto& r : _latest) {
        r.drift_ppm       = 0.0;
        r.uncertainty_ppm = 999.0;
        r.phase_rate_rad_s = 0.0;
        r.num_samples     = 0;
        r.valid           = false;
    }
    _latest[0].uncertainty_ppm = 0.0; // reference is exact by definition
}

// ---------------------------------------------------------------------------
// update() — poll hardware
// ---------------------------------------------------------------------------

std::vector<pps_drift_estimator::result> pps_drift_estimator::update(
    const std::vector<uhd::usrp::multi_usrp::sptr>& devices, double rf_freq)
{
    if (devices.size() < _num_devices) {
        UHD_LOG_WARNING("BONDED",
            "pps_drift_estimator::update: fewer devices than expected ("
                << devices.size() << " < " << _num_devices << ")");
        return _latest;
    }

    pps_point pt;
    pt.timestamps.resize(_num_devices);
    for (size_t i = 0; i < _num_devices; ++i) {
        pt.timestamps[i] = devices[i]->get_time_last_pps(0).get_real_secs();
    }

    // Only record if device 0's PPS timestamp has advanced.
    if (_history.empty()
        || pt.timestamps[0] != _history.back().timestamps[0]) {
        _history.push_back(std::move(pt));
        if (_history.size() > _max_history) {
            _history.pop_front();
        }
        _latest = _recompute(rf_freq);

        UHD_LOG_DEBUG("BONDED",
            "PPS edges: " << _latest[0].num_samples
                          << " | drift (dev1): " << _latest[1].drift_ppm
                          << " ppm");
    }

    return _latest;
}

// ---------------------------------------------------------------------------
// add_observation() — N-device (unit tests / offline use)
// ---------------------------------------------------------------------------

std::vector<pps_drift_estimator::result> pps_drift_estimator::add_observation(
    const std::vector<double>& times, double rf_freq)
{
    if (times.size() < _num_devices) {
        throw std::invalid_argument(
            "pps_drift_estimator::add_observation: times.size() < num_devices");
    }

    if (_history.empty()
        || times[0] != _history.back().timestamps[0]) {
        pps_point pt;
        pt.timestamps.assign(times.begin(), times.begin() + _num_devices);
        _history.push_back(std::move(pt));
        if (_history.size() > _max_history) {
            _history.pop_front();
        }
    }

    _latest = _recompute(rf_freq);
    return _latest;
}

// ---------------------------------------------------------------------------
// add_observation() — 2-device convenience overload (used by unit tests)
// ---------------------------------------------------------------------------

pps_drift_estimator::result pps_drift_estimator::add_observation(
    double t0, double t1, double rf_freq)
{
    auto all = add_observation(std::vector<double>{t0, t1}, rf_freq);
    return all[1]; // return device 1's result (only non-reference device)
}

// ---------------------------------------------------------------------------
// latest() — single device
// ---------------------------------------------------------------------------

pps_drift_estimator::result pps_drift_estimator::latest(size_t dev_idx) const
{
    if (dev_idx >= _latest.size()) {
        throw std::out_of_range("pps_drift_estimator::latest: dev_idx out of range");
    }
    return _latest[dev_idx];
}

// ---------------------------------------------------------------------------
// reset()
// ---------------------------------------------------------------------------

void pps_drift_estimator::reset()
{
    _history.clear();
    for (size_t i = 0; i < _num_devices; ++i) {
        _latest[i]               = {};
        _latest[i].valid         = false;
        _latest[i].uncertainty_ppm = (i == 0) ? 0.0 : 999.0;
    }
}

// ---------------------------------------------------------------------------
// _recompute() — run per-device linear regression
// ---------------------------------------------------------------------------

std::vector<pps_drift_estimator::result> pps_drift_estimator::_recompute(
    double rf_freq)
{
    const size_t n_pts = _history.size();
    std::vector<result> out(_num_devices);

    // Device 0 is always the reference.
    out[0].drift_ppm        = 0.0;
    out[0].uncertainty_ppm  = 0.0;
    out[0].phase_rate_rad_s = 0.0;
    out[0].num_samples      = n_pts;
    out[0].valid            = (n_pts >= 3);

    const double unc = _compute_uncertainty_ppm();

    for (size_t i = 1; i < _num_devices; ++i) {
        out[i].num_samples      = n_pts;
        out[i].valid            = (n_pts >= 3);

        if (!out[i].valid) {
            out[i].drift_ppm        = 0.0;
            out[i].uncertainty_ppm  = 999.0;
            out[i].phase_rate_rad_s = 0.0;
        } else {
            out[i].drift_ppm        = _compute_drift_ppm(i);
            out[i].uncertainty_ppm  = unc;
            out[i].phase_rate_rad_s =
                2.0 * M_PI * rf_freq * (out[i].drift_ppm / 1e6);
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// _compute_drift_ppm() — OLS slope for device dev_idx vs device 0
// ---------------------------------------------------------------------------

double pps_drift_estimator::_compute_drift_ppm(size_t dev_idx) const
{
    // X = device 0 timestamps (reference clock), Y = (t_i - t_0) differences.
    // OLS slope of Y on X gives the fractional frequency offset; * 1e6 = ppm.

    const double n     = static_cast<double>(_history.size());
    const double t0_base = _history.front().timestamps[0];
    const double d_base  = _history.front().timestamps[dev_idx]
                           - _history.front().timestamps[0];

    double sx = 0.0, sy = 0.0, sxy = 0.0, sxx = 0.0;

    for (const auto& p : _history) {
        double x = p.timestamps[0] - t0_base;
        double y = (p.timestamps[dev_idx] - p.timestamps[0]) - d_base;
        sx  += x;
        sy  += y;
        sxy += x * y;
        sxx += x * x;
    }

    double denom = n * sxx - sx * sx;
    if (std::abs(denom) < 1e-30) {
        return 0.0;
    }

    double slope = (n * sxy - sx * sy) / denom;
    return slope * 1e6; // slope is already the fractional frequency offset
}

// ---------------------------------------------------------------------------
// _compute_uncertainty_ppm() — based on time span and PPS jitter
// ---------------------------------------------------------------------------

double pps_drift_estimator::_compute_uncertainty_ppm() const
{
    if (_history.size() < 2) {
        return 999.0;
    }

    double T = _history.back().timestamps[0] - _history.front().timestamps[0];
    if (T <= 0.0) {
        return 999.0;
    }

    // PPS jitter ~100 ns (GPS), B210 FPGA timestamping ~18 ns → ~102 ns combined.
    // Slope uncertainty = sqrt(2) * noise / T; * 1e6 → ppm.
    constexpr double pps_noise_s = 102e-9;
    return (1.41421356 * pps_noise_s / T) * 1e6;
}

}}} // namespace uhd::usrp::bonded
