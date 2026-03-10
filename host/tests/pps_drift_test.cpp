// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for pps_drift_estimator.
// These do not require hardware — synthetic PPS timestamps are injected
// via add_observation().

#include <uhd/usrp/bonded/pps_drift_estimator.hpp>
#include <boost/test/unit_test.hpp>
#include <cmath>

using namespace uhd::usrp::bonded;

// ---------------------------------------------------------------------------
// Helper: populate the estimator with N observations simulating a constant
// drift of `drift_ppm` ppm (board 1 relative to board 0).
// ---------------------------------------------------------------------------
static void feed_observations(pps_drift_estimator& est,
    double drift_ppm,
    size_t n,
    double start_t0 = 0.0,
    double rf_freq  = 2.4e9)
{
    const double rate = 1.0 + drift_ppm / 1e6;
    for (size_t i = 0; i < n; ++i) {
        double t0 = start_t0 + static_cast<double>(i);
        double t1 = t0 * rate; // board 1 runs at (1 + drift_ppm/1e6) speed
        est.add_observation(t0, t1, rf_freq);
    }
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_not_valid_before_3_observations)
{
    pps_drift_estimator est;

    auto r = est.add_observation(0.0, 0.0);
    BOOST_CHECK(!r.valid);

    r = est.add_observation(1.0, 1.000002); // 2 ppm
    BOOST_CHECK(!r.valid);

    r = est.add_observation(2.0, 2.000004);
    BOOST_CHECK(r.valid);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_zero_drift)
{
    pps_drift_estimator est;
    feed_observations(est, 0.0, 20);

    auto r = est.latest();
    BOOST_CHECK(r.valid);
    BOOST_CHECK_SMALL(r.drift_ppm, 1e-6); // should be essentially 0
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_positive_drift_2ppm)
{
    pps_drift_estimator est;
    feed_observations(est, 2.0, 30);

    auto r = est.latest();
    BOOST_CHECK(r.valid);
    // Linear regression should recover exactly 2 ppm for perfect data
    BOOST_CHECK_CLOSE(r.drift_ppm, 2.0, 0.01); // within 0.01%
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_negative_drift)
{
    pps_drift_estimator est;
    feed_observations(est, -1.5, 30);

    auto r = est.latest();
    BOOST_CHECK(r.valid);
    BOOST_CHECK_CLOSE(r.drift_ppm, -1.5, 0.01);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_uncertainty_decreases_with_more_observations)
{
    pps_drift_estimator est5;
    feed_observations(est5, 2.0, 5);

    pps_drift_estimator est30;
    feed_observations(est30, 2.0, 30);

    // Uncertainty should decrease as time span grows
    BOOST_CHECK_LT(est30.latest().uncertainty_ppm, est5.latest().uncertainty_ppm);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_phase_rate_at_rf_freq)
{
    pps_drift_estimator est;
    const double rf_freq   = 2.4e9;
    const double drift_ppm = 2.0;
    feed_observations(est, drift_ppm, 20, 0.0, rf_freq);

    auto r = est.latest();
    BOOST_CHECK(r.valid);

    // Expected: phase_rate = 2π * rf_freq * drift_ppm / 1e6
    double expected = 2.0 * M_PI * rf_freq * (drift_ppm / 1e6);
    BOOST_CHECK_CLOSE(r.phase_rate_rad_s, expected, 0.01);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_reset_clears_history)
{
    pps_drift_estimator est;
    feed_observations(est, 3.0, 20);
    BOOST_CHECK(est.latest().valid);

    est.reset();
    BOOST_CHECK(!est.latest().valid);
    BOOST_CHECK_EQUAL(est.latest().num_samples, 0u);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_max_history_window)
{
    // Default max_history = 120; filling 150 observations should not crash
    // and should only keep the last 120.
    pps_drift_estimator est(2, 120);
    feed_observations(est, 2.0, 150);

    auto r = est.latest();
    BOOST_CHECK(r.valid);
    BOOST_CHECK_LE(r.num_samples, 120u);
    BOOST_CHECK_CLOSE(r.drift_ppm, 2.0, 0.01);
}

// ---------------------------------------------------------------------------
BOOST_AUTO_TEST_CASE(test_duplicate_t0_not_added)
{
    pps_drift_estimator est;
    est.add_observation(1.0, 1.000002);
    est.add_observation(1.0, 1.000003); // same t0, should be ignored
    est.add_observation(2.0, 2.000004);

    // Only 2 unique observations → not yet valid
    auto r = est.add_observation(3.0, 3.000006);
    BOOST_CHECK(r.valid);
    BOOST_CHECK_EQUAL(r.num_samples, 3u);
}
