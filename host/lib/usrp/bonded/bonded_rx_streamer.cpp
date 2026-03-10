// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

namespace uhd { namespace usrp { namespace bonded {

// ---------------------------------------------------------------------------
// bonding_quality_metrics
// ---------------------------------------------------------------------------

std::string bonding_quality_metrics::to_string() const
{
    std::ostringstream ss;
    ss << "PPS samples: " << pps_samples_used
       << " | PPS: " << (pps_healthy ? "OK" : "MISSING")
       << " | " << (valid ? "VALID" : "CALIBRATING");
    for (size_t i = 0; i < drift_ppm.size(); ++i) {
        ss << " | Dev" << (i + 1) << " drift: " << drift_ppm[i]
           << " ppm (±" << uncertainty_ppm[i] << " ppm)"
           << " phase: " << phase_rate_rad_s[i] << " rad/s";
    }
    return ss.str();
}

// ---------------------------------------------------------------------------
// bonded_rx_streamer
// ---------------------------------------------------------------------------

bonded_rx_streamer::bonded_rx_streamer(
    std::vector<uhd::usrp::multi_usrp::sptr> devices,
    double sample_rate,
    double center_freq,
    size_t fft_size)
    : _devices(std::move(devices))
    , _sample_rate(sample_rate)
    , _center_freq(center_freq)
{
    const size_t N = _devices.size();
    if (N < 2) {
        throw uhd::runtime_error(
            "bonded_rx_streamer requires at least 2 devices");
    }

    // Guard band: 10% of sample rate per side
    const double guard_band = sample_rate * 0.10;

    // Tune each device uniformly around center_freq:
    //   freq_i = center_freq + (i - (N-1)/2.0) * sample_rate
    // e.g. N=2: freq_0 = center - R/2, freq_1 = center + R/2
    std::vector<double> freqs(N);
    for (size_t i = 0; i < N; ++i) {
        freqs[i] = center_freq + (static_cast<double>(i) - (N - 1) / 2.0) * sample_rate;
    }

    _receiver = std::make_unique<bonded_receiver>(_devices, sample_rate, freqs);
    _stitcher = std::make_unique<spectrum_stitcher>(
        sample_rate, fft_size, guard_band / 2.0, N);
    _drift_estimator = std::make_unique<pps_drift_estimator>(N);

    // Create N-1 phase correctors: device 0 is reference, no correction needed
    _phase_correctors.resize(N - 1);
    for (size_t i = 0; i < N - 1; ++i) {
        _phase_correctors[i] = std::make_unique<phase_corrector>();
    }

    // Initialize per-device quality vectors
    _quality.drift_ppm.assign(N - 1, 0.0);
    _quality.uncertainty_ppm.assign(N - 1, 0.0);
    _quality.phase_rate_rad_s.assign(N - 1, 0.0);
}

bonded_rx_streamer::~bonded_rx_streamer()
{
    stop();
}

void bonded_rx_streamer::start()
{
    _running = true;
    _receiver->start_streaming();
    _drift_thread = std::thread([this]() { _drift_tracking_loop(); });
}

void bonded_rx_streamer::stop()
{
    _running = false;
    if (_receiver) {
        _receiver->stop_streaming();
    }
    if (_drift_thread.joinable()) {
        _drift_thread.join();
    }
}

size_t bonded_rx_streamer::recv(std::complex<float>* output,
    size_t noutput,
    uhd::rx_metadata_t& metadata,
    double timeout)
{
    const size_t fft_sz = _stitcher->fft_size();
    const size_t N      = _devices.size();

    if (noutput < _stitcher->output_bins()) {
        UHD_LOG_WARNING("BONDED",
            "recv(): output buffer too small ("
                << noutput << " < " << _stitcher->output_bins() << ")");
        return 0;
    }

    if (!_receiver->recv_aligned(fft_sz, timeout)) {
        return 0;
    }

    metadata = _receiver->get_stream(0).metadata;

    // Build input pointer vector and apply phase correction to devices 1..N-1
    const auto latest = _drift_estimator->latest_all();
    std::vector<const std::complex<float>*> inputs(N);
    inputs[0] = _receiver->get_buffer(0);

    for (size_t i = 1; i < N; ++i) {
        if (i - 1 < latest.size()) {
            const auto& est = latest[i - 1];
            if (est.valid) {
                _phase_correctors[i - 1]->correct(
                    _receiver->get_buffer(i),
                    fft_sz,
                    est.drift_ppm,
                    _receiver->get_freq(i),
                    _sample_rate);
            }
        }
        inputs[i] = _receiver->get_buffer(i);
    }

    _stitcher->stitch(inputs, fft_sz, output);
    return _stitcher->output_bins();
}

size_t bonded_rx_streamer::output_size() const
{
    return _stitcher->output_bins();
}

bonding_quality_metrics bonded_rx_streamer::quality() const
{
    return _quality;
}

// ---------------------------------------------------------------------------
// Background drift tracking
// ---------------------------------------------------------------------------

void bonded_rx_streamer::_drift_tracking_loop()
{
    UHD_LOG_DEBUG("BONDED", "PPS drift tracking thread started");

    double last_pps_seen = 0.0;

    while (_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!_running) {
            break;
        }

        const double current_pps =
            _devices[0]->get_time_last_pps(0).get_real_secs();

        bool pps_healthy = true;
        if (last_pps_seen > 0.0) {
            const double wall_elapsed =
                _devices[0]->get_time_now(0).get_real_secs() - current_pps;
            if (wall_elapsed > 2.0) {
                pps_healthy = false;
                UHD_LOG_WARNING("BONDED",
                    "No PPS detected for " << wall_elapsed
                                           << " s. Check GPS module.");
            }
        }

        if (current_pps != last_pps_seen) {
            last_pps_seen = current_pps;

            const auto all_results =
                _drift_estimator->update(_devices, _center_freq);

            const size_t nr = all_results.size();
            if (nr == 0) {
                _quality.pps_healthy = pps_healthy;
                continue;
            }

            _quality.pps_samples_used =
                static_cast<int>(all_results[0].num_samples);
            _quality.pps_healthy = pps_healthy;
            _quality.valid       = all_results[0].valid;

            for (size_t i = 0; i < nr && i < _quality.drift_ppm.size(); ++i) {
                _quality.drift_ppm[i]       = all_results[i].drift_ppm;
                _quality.uncertainty_ppm[i] = all_results[i].uncertainty_ppm;
                _quality.phase_rate_rad_s[i]= all_results[i].phase_rate_rad_s;

                if (std::abs(all_results[i].drift_ppm) > 10.0) {
                    UHD_LOG_WARNING("BONDED",
                        "Device " << (i + 1) << " oscillator drift "
                        << all_results[i].drift_ppm
                        << " ppm > 10 ppm. Check TCXO / clock source.");
                }
            }
        } else {
            _quality.pps_healthy = pps_healthy;
        }
    }

    UHD_LOG_DEBUG("BONDED", "PPS drift tracking thread stopped");
}

}}} // namespace uhd::usrp::bonded

