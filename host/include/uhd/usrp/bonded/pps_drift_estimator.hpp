// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <uhd/config.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <cstddef>
#include <deque>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Estimates the frequency offset (in ppm) between each device and the reference
 * device (device 0) by comparing GPS PPS hardware timestamps over time.
 *
 * All devices in a bonded group share the same physical PPS pulse.  Each device
 * timestamps the arriving PPS edge with its own independent TCXO-derived clock.
 * Because two oscillators run at slightly different rates the timestamp
 * difference slowly diverges.  Linear regression over many PPS edges yields the
 * relative clock offset per device with ~0.01 ppm accuracy after 10 seconds.
 *
 * This method requires NO RF signals — it runs in a background thread during
 * normal wideband streaming.
 *
 * Supports 2..N devices.  Device 0 is always taken as the reference
 * (drift_ppm[0] == 0.0 by definition).
 */
class UHD_API pps_drift_estimator
{
public:
    struct result {
        double drift_ppm        = 0.0;  //!< Frequency offset relative to device 0
        double uncertainty_ppm  = 0.0;  //!< 1-sigma uncertainty of the estimate
        double phase_rate_rad_s = 0.0;  //!< Phase rotation rate at rf_freq (rad/s)
        size_t num_samples      = 0;    //!< Number of PPS edges used in regression
        bool valid              = false;//!< true when enough data has been collected
    };

    /*!\param num_devices  Number of devices in the bonded group (≥ 2).
     * \param max_history  Maximum PPS edges to retain in the regression window
     *                     (default 120 = 2 minutes).
     */
    explicit pps_drift_estimator(size_t num_devices = 2, size_t max_history = 120);

    /*!\brief Poll hardware PPS timestamps from all devices and update estimates.
     *
     * Call this once per second from a background thread.
     *
     * \param devices  All N devices in the bonded group.
     * \param rf_freq  RF center frequency used to compute phase_rate_rad_s (Hz).
     * \returns        Updated per-device results (result[0].drift_ppm == 0.0).
     */
    std::vector<result> update(
        const std::vector<uhd::usrp::multi_usrp::sptr>& devices, double rf_freq);

    //! Latest per-device results without polling hardware.
    std::vector<result> latest_all() const { return _latest; }

    //! Latest result for a single device (default: device 1, for 2-device use).
    result latest(size_t dev_idx = 1) const;

    /*!\brief Inject a synthetic N-device PPS observation (for unit tests / no hardware).
     *
     * \param times   PPS timestamp for each device (seconds).
     *                times[0] is device 0 (reference).
     * \param rf_freq RF frequency used for phase_rate_rad_s (Hz).
     * \returns Updated per-device results.
     */
    std::vector<result> add_observation(
        const std::vector<double>& times, double rf_freq = 0.0);

    /*!\brief Convenience overload for 2-device unit tests.
     *
     * Equivalent to add_observation({t0, t1}, rf_freq) and returns result[1].
     */
    result add_observation(double t0, double t1, double rf_freq = 0.0);

    //! Clear all accumulated PPS history (e.g. after a time re-sync).
    void reset();

private:
    struct pps_point {
        std::vector<double> timestamps; //!< timestamps[i] = device i get_time_last_pps
    };

    size_t _num_devices;
    size_t _max_history;
    std::deque<pps_point> _history;
    std::vector<result> _latest;

    std::vector<result> _recompute(double rf_freq);
    double _compute_drift_ppm(size_t dev_idx) const;
    double _compute_uncertainty_ppm() const;
};

}}} // namespace uhd::usrp::bonded
