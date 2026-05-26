//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "bonded_receiver.hpp"

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace uhd { namespace usrp { namespace bonded {

// Threshold for inter-device first-packet timestamp spread (seconds).
static constexpr double ALIGN_THRESHOLD_SECS = 1e-3; // 1 ms

bonded_receiver::bonded_receiver(const config& cfg) : _config(cfg) {}

bonded_receiver::~bonded_receiver()
{
    if (_streaming) {
        stop();
    }
}

// =========================================================================
// configure() — full setup pipeline
// =========================================================================
void bonded_receiver::configure()
{
    if (_configured) {
        throw uhd::runtime_error("bonded_receiver::configure() called twice");
    }
    _open_devices();
    _apply_sync();
    _configure_hardware();
    _create_streamers();
    _align_time();
    _configured = true;
}

// =========================================================================
// Burst mode
// =========================================================================
bonded_receiver::rx_result bonded_receiver::capture_burst(
    size_t nsamps, double delay_sec)
{
    if (!_configured) {
        throw uhd::runtime_error(
            "bonded_receiver::capture_burst() called before configure()");
    }
    if (_streaming) {
        throw uhd::runtime_error(
            "bonded_receiver::capture_burst() called while continuous streaming");
    }

    const size_t num_dev = _devices.size();
    rx_result result;
    result.data.resize(num_dev);
    result.first_timestamps.resize(num_dev, -1.0);
    result.success.resize(num_dev, false);

    // Schedule a timed burst on all devices
    const uhd::time_spec_t start_time =
        _devices[0]->get_time_now() + uhd::time_spec_t(delay_sec);

    std::cout << boost::format(
                     "[bonded_receiver] Scheduling burst: %zu samps at t=%.6f s\n")
                     % nsamps % start_time.get_real_secs();

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = nsamps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = start_time;
    for (auto& s : _streamers) {
        s->issue_stream_cmd(stream_cmd);
    }

    // Drain each device in parallel threads
    std::mutex print_mu;
    auto drain_fn = [&](size_t d) {
        auto& streamer      = _streamers[d];
        const size_t nch    = _channels_per_device;
        const size_t buf_sz = streamer->get_max_num_samps();

        // Allocate output storage
        result.data[d].resize(nch);
        for (size_t c = 0; c < nch; c++) {
            result.data[d][c].resize(nsamps);
        }

        // Temporary recv buffer
        std::vector<std::vector<std::complex<float>>> bufs(
            nch, std::vector<std::complex<float>>(buf_sz));
        std::vector<std::complex<float>*> ptrs(nch);
        for (size_t c = 0; c < nch; c++) {
            ptrs[c] = bufs[c].data();
        }

        double timeout = delay_sec + 0.5;
        bool first     = true;
        size_t total   = 0;
        uhd::rx_metadata_t md;

        while (total < nsamps) {
            const size_t to_recv = std::min(buf_sz, nsamps - total);
            const size_t n = streamer->recv(ptrs, to_recv, md, timeout);
            timeout        = 0.5;

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::lock_guard<std::mutex> lock(print_mu);
                std::cerr << boost::format(
                                 "[bonded_receiver] Device %u: timeout\n")
                                 % d;
                return;
            }
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::lock_guard<std::mutex> lock(print_mu);
                std::cerr << boost::format(
                                 "[bonded_receiver] Device %u: error %s\n")
                                 % d % md.strerror();
                return;
            }
            if (first) {
                result.first_timestamps[d] = md.time_spec.get_real_secs();
                std::lock_guard<std::mutex> lock(print_mu);
                std::cout << boost::format(
                                 "[bonded_receiver] Device %u: first packet "
                                 "%zu samps @ t=%.9f s\n")
                                 % d % n % result.first_timestamps[d];
                first = false;
            }
            // Copy to output
            for (size_t c = 0; c < nch; c++) {
                std::copy(bufs[c].begin(),
                    bufs[c].begin() + n,
                    result.data[d][c].begin() + total);
            }
            total += n;
        }
        result.success[d] = true;
    };

    std::vector<std::thread> threads;
    threads.reserve(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        threads.emplace_back(drain_fn, d);
    }
    for (auto& t : threads) {
        t.join();
    }

    // Compute alignment
    std::vector<double> valid_ts;
    for (size_t d = 0; d < num_dev; d++) {
        if (result.success[d] && result.first_timestamps[d] >= 0.0) {
            valid_ts.push_back(result.first_timestamps[d]);
        }
    }

    if (valid_ts.size() == num_dev && num_dev > 1) {
        const double ts_min = *std::min_element(valid_ts.begin(), valid_ts.end());
        const double ts_max = *std::max_element(valid_ts.begin(), valid_ts.end());
        result.inter_device_spread_us = (ts_max - ts_min) * 1e6;
        result.aligned = (ts_max - ts_min) <= ALIGN_THRESHOLD_SECS;
    } else if (valid_ts.size() == num_dev && num_dev == 1) {
        result.aligned = true;
    } else {
        result.aligned       = false;
        result.error_message = "One or more devices did not receive data";
    }

    return result;
}

// =========================================================================
// Continuous mode
// =========================================================================
void bonded_receiver::start_continuous()
{
    if (!_configured) {
        throw uhd::runtime_error(
            "bonded_receiver::start_continuous() called before configure()");
    }
    if (_streaming) {
        throw uhd::runtime_error("bonded_receiver: already streaming");
    }

    const size_t num_dev = _devices.size();
    _stop_flag = false;
    _ring_buffers.resize(num_dev);
    _ring_mutexes = std::vector<std::mutex>(num_dev);

    // Issue a common timed continuous stream command to all devices.
    // Using stream_now=true may fail for multi-channel streamers on some
    // devices (e.g., B210), and a timed start keeps devices aligned.
    const uhd::time_spec_t start_time =
        _devices[0]->get_time_now() + uhd::time_spec_t(0.1);
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = start_time;
    for (auto& s : _streamers) {
        s->issue_stream_cmd(stream_cmd);
    }

    // Launch recv threads
    _recv_threads.reserve(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        _recv_threads.emplace_back(&bonded_receiver::_recv_loop, this, d);
    }
    _streaming = true;
}

bonded_receiver::rx_result bonded_receiver::get_aligned_samples(
    size_t nsamps, double timeout_sec)
{
    if (!_streaming) {
        throw uhd::runtime_error(
            "bonded_receiver::get_aligned_samples() called while not streaming");
    }

    const size_t num_dev = _devices.size();
    rx_result result;
    result.data.resize(num_dev);
    result.first_timestamps.resize(num_dev, -1.0);
    result.success.resize(num_dev, false);
    bool degraded = false;

    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::duration<double>(timeout_sec);

    // Wait until all ring buffers have enough data
    while (std::chrono::steady_clock::now() < deadline) {
        bool all_ready = true;
        for (size_t d = 0; d < num_dev; d++) {
            std::lock_guard<std::mutex> lock(_ring_mutexes[d]);
            size_t available = 0;
            for (const auto& c : _ring_buffers[d]) {
                available += c.num_samples;
            }
            if (available < nsamps) {
                all_ready = false;
                break;
            }
        }
        if (all_ready) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Find the alignment point: latest first-chunk timestamp across all devices
    double align_ts = -1.0;
    for (size_t d = 0; d < num_dev; d++) {
        std::lock_guard<std::mutex> lock(_ring_mutexes[d]);
        if (_ring_buffers[d].empty()) {
            result.error_message = "Device " + std::to_string(d) + " has no data";
            return result;
        }
        double first_ts = _ring_buffers[d].front().timestamp;
        if (first_ts > align_ts) {
            align_ts = first_ts;
        }
    }

    // Extract nsamps from each device starting at or after align_ts
    for (size_t d = 0; d < num_dev; d++) {
        std::lock_guard<std::mutex> lock(_ring_mutexes[d]);
        auto& ring = _ring_buffers[d];

        // Discard chunks older than alignment point
        while (!ring.empty()) {
            auto& front = ring.front();
            double chunk_end_ts =
                front.timestamp
                + static_cast<double>(front.num_samples) / _config.rate;
            if (chunk_end_ts <= align_ts) {
                ring.pop_front();
            } else {
                break;
            }
        }

        if (ring.empty()) {
            const size_t nch = _channels_per_device;
            result.data[d].resize(nch);
            for (size_t c = 0; c < nch; c++) {
                result.data[d][c].assign(nsamps, std::complex<float>(0.0f, 0.0f));
            }
            result.first_timestamps[d] = align_ts;
            result.success[d]          = false;
            degraded                   = true;
            continue;
        }

        // Collect nsamps
        const size_t nch = _channels_per_device;
        result.data[d].resize(nch);
        for (size_t c = 0; c < nch; c++) {
            result.data[d][c].reserve(nsamps);
        }

        size_t collected = 0;
        bool first       = true;
        while (collected < nsamps && !ring.empty()) {
            auto& front      = ring.front();
            size_t skip      = 0;

            // For the first chunk, skip samples before align_ts
            if (first) {
                double offset_secs = align_ts - front.timestamp;
                if (offset_secs > 0) {
                    skip = static_cast<size_t>(offset_secs * _config.rate);
                    if (skip >= front.num_samples) {
                        ring.pop_front();
                        continue;
                    }
                }
                result.first_timestamps[d] = align_ts;
                first = false;
            }

            size_t avail = front.num_samples - skip;
            size_t to_copy = std::min(avail, nsamps - collected);

            for (size_t c = 0; c < nch; c++) {
                result.data[d][c].insert(result.data[d][c].end(),
                    front.data[c].begin() + skip,
                    front.data[c].begin() + skip + to_copy);
            }
            collected += to_copy;

            if (skip + to_copy >= front.num_samples) {
                ring.pop_front();
            } else {
                // Partial consumption: trim the front chunk
                for (size_t c = 0; c < nch; c++) {
                    front.data[c].erase(
                        front.data[c].begin(), front.data[c].begin() + skip + to_copy);
                }
                front.timestamp +=
                    static_cast<double>(skip + to_copy) / _config.rate;
                front.num_samples -= (skip + to_copy);
            }
        }

        result.success[d] = (collected >= nsamps);
        if (!result.success[d]) {
            for (size_t c = 0; c < nch; c++) {
                result.data[d][c].resize(nsamps, std::complex<float>(0.0f, 0.0f));
            }
            if (result.first_timestamps[d] < 0.0) {
                result.first_timestamps[d] = align_ts;
            }
            degraded = true;
        }
    }

    // Compute alignment spread
    std::vector<double> valid_ts;
    for (size_t d = 0; d < num_dev; d++) {
        if (result.first_timestamps[d] >= 0.0) {
            valid_ts.push_back(result.first_timestamps[d]);
        }
    }
    if (valid_ts.size() == num_dev) {
        const double ts_min = *std::min_element(valid_ts.begin(), valid_ts.end());
        const double ts_max = *std::max_element(valid_ts.begin(), valid_ts.end());
        result.inter_device_spread_us = (ts_max - ts_min) * 1e6;
        result.aligned = true; // aligned by construction (same align_ts)
    }

    if (degraded) {
        result.error_message =
            "One or more devices overflowed or ran short; missing samples were zero-filled";
    }

    return result;
}

void bonded_receiver::stop()
{
    if (!_streaming) return;

    _stop_flag = true;

    // Stop streaming on all devices
    uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    for (auto& s : _streamers) {
        s->issue_stream_cmd(stop_cmd);
    }

    // Join all threads
    for (auto& t : _recv_threads) {
        if (t.joinable()) {
            t.join();
        }
    }
    _recv_threads.clear();
    _ring_buffers.clear();
    _streaming = false;
}

// =========================================================================
// Private helpers
// =========================================================================

void bonded_receiver::_open_devices()
{
    std::cout << boost::format(
                     "[bonded_receiver] Opening %zu device(s)...\n")
                     % _config.serials.size();

    for (size_t i = 0; i < _config.serials.size(); i++) {
        uhd::device_addr_t dev_args;
        dev_args["serial"] = _config.serials[i];
        std::cout << boost::format(
                         "[bonded_receiver] Opening serial=%s ...\n")
                         % _config.serials[i];
        _devices.push_back(uhd::usrp::multi_usrp::make(dev_args));
    }
}

void bonded_receiver::_apply_sync()
{
    std::cout << "[bonded_receiver] Applying sync config to all devices...\n";
    const double lock_timeout = _config.lock_timeout;
    const bool strict         = _config.strict;

    for (size_t d = 0; d < _devices.size(); d++) {
        auto& usrp = _devices[d];
        const size_t n = usrp->get_num_mboards();
        for (size_t m = 0; m < n; m++) {
            usrp->set_clock_source(_config.clock_source, m);
            usrp->set_time_source(_config.time_source, m);
        }

        // Wait for reference lock on external/gpsdo
        if (_config.clock_source == "external" || _config.clock_source == "gpsdo") {
            const auto deadline = std::chrono::steady_clock::now()
                                  + std::chrono::duration<double>(lock_timeout);
            bool locked = false;
            while (std::chrono::steady_clock::now() < deadline) {
                bool all_locked = true;
                for (size_t m = 0; m < n; m++) {
                    try {
                        if (!usrp->get_mboard_sensor("ref_locked", m).to_bool()) {
                            all_locked = false;
                            break;
                        }
                    } catch (const uhd::lookup_error&) {
                        // sensor absent — assume locked
                    }
                }
                if (all_locked) {
                    locked = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!locked) {
                const std::string msg =
                    "Device " + std::to_string(d) + " (serial="
                    + _config.serials[d] + "): ref not locked within "
                    + std::to_string(static_cast<int>(lock_timeout)) + " s.";
                if (strict) {
                    throw uhd::runtime_error(msg);
                }
                std::cerr << "[bonded_receiver] WARNING: " << msg << "\n";
            } else {
                std::cout << boost::format(
                                 "[bonded_receiver] Device %u: reference locked.\n")
                                 % d;
            }
        }
    }
}

void bonded_receiver::_configure_hardware()
{
    for (size_t d = 0; d < _devices.size(); d++) {
        auto& usrp = _devices[d];
        if (!_config.subdev.empty()) {
            usrp->set_rx_subdev_spec(_config.subdev);
        }
        usrp->set_rx_rate(_config.rate);
        const size_t nch = usrp->get_rx_num_channels();
        for (size_t ch = 0; ch < nch; ch++) {
            usrp->set_rx_freq(uhd::tune_request_t(_config.freq), ch);
            usrp->set_rx_gain(_config.gain, ch);
        }
        std::cout << boost::format(
                         "[bonded_receiver] Device %u: %.3f Msps, %.3f MHz, %u ch\n")
                         % d % (usrp->get_rx_rate() / 1e6)
                         % (usrp->get_rx_freq(0) / 1e6) % nch;

        if (d == 0) {
            _channels_per_device = nch;
        }
    }
}

void bonded_receiver::_create_streamers()
{
    for (size_t d = 0; d < _devices.size(); d++) {
        const size_t nch = _channels_per_device;
        std::vector<size_t> ch_list(nch);
        for (size_t c = 0; c < nch; c++) {
            ch_list[c] = c;
        }
        uhd::stream_args_t sa("fc32");
        sa.channels  = ch_list;
        _streamers.push_back(_devices[d]->get_rx_stream(sa));
    }
}

void bonded_receiver::_align_time()
{
    if (_config.time_source == "external" || _config.time_source == "gpsdo") {
        // B210 quirk: MCR changes reset time_source to "none".
        // Re-apply after all MCR changes from set_rx_rate/get_rx_stream.
        for (auto& dev : _devices) {
            dev->set_time_source(_config.time_source);
        }

        // Wait 500ms for FPGA PPS detection circuit to stabilise
        std::cout << "[bonded_receiver] Waiting for FPGA to stabilise after MCR change...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Detect PPS edge on device 0
        std::cout << "[bonded_receiver] Waiting for PPS edge on device 0...\n";
        const auto pps_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
        uhd::time_spec_t last_pps = _devices[0]->get_time_last_pps();
        while (_devices[0]->get_time_last_pps() == last_pps) {
            if (std::chrono::steady_clock::now() > pps_deadline) {
                throw uhd::runtime_error(
                    "No PPS detected on device 0 within 2.5 s.\n"
                    "Check PPS cable connection to all devices.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // PPS just fired — arm ALL devices before the next edge (~999 ms away)
        std::cout << "[bonded_receiver] PPS detected — arming all devices...\n";
        for (auto& dev : _devices) {
            dev->set_time_next_pps(uhd::time_spec_t(0.0));
        }

        // Wait for the next PPS edge to latch time 0 on every device
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[bonded_receiver] PPS alignment done on all devices.\n";
    } else {
        // Internal time: set_time_now(0) on all devices simultaneously
        std::cout << "[bonded_receiver] Internal time: set_time_now(0) on all devices.\n";
        for (auto& dev : _devices) {
            dev->set_time_now(
                uhd::time_spec_t(0.0), uhd::usrp::multi_usrp::ALL_MBOARDS);
        }
    }
}

void bonded_receiver::_recv_loop(size_t d)
{
    uhd::set_thread_priority_safe();

    auto& streamer      = _streamers[d];
    const size_t nch    = _channels_per_device;
    const size_t buf_sz = streamer->get_max_num_samps();

    std::vector<std::vector<std::complex<float>>> bufs(
        nch, std::vector<std::complex<float>>(buf_sz));
    std::vector<std::complex<float>*> ptrs(nch);
    for (size_t c = 0; c < nch; c++) {
        ptrs[c] = bufs[c].data();
    }

    uhd::rx_metadata_t md;
    while (!_stop_flag) {
        const size_t n = streamer->recv(ptrs, buf_sz, md, 0.1);
        if (n == 0) continue;

        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            std::cerr << boost::format(
                             "[bonded_receiver] Device %u: overflow (O)\n")
                             % d;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE
            && md.error_code != uhd::rx_metadata_t::ERROR_CODE_OVERFLOW
            && md.error_code != uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cerr << boost::format(
                             "[bonded_receiver] Device %u: error %s\n")
                             % d % md.strerror();
            continue;
        }

        // Build chunk and push to ring buffer
        chunk c;
        c.timestamp   = md.time_spec.get_real_secs();
        c.num_samples = n;
        c.data.resize(nch);
        for (size_t ch = 0; ch < nch; ch++) {
            c.data[ch].assign(bufs[ch].begin(), bufs[ch].begin() + n);
        }

        {
            std::lock_guard<std::mutex> lock(_ring_mutexes[d]);
            _ring_buffers[d].push_back(std::move(c));
            // Limit ring buffer size (keep ~2 seconds of data max)
            const size_t max_chunks =
                static_cast<size_t>(2.0 * _config.rate / buf_sz) + 1;
            while (_ring_buffers[d].size() > max_chunks) {
                _ring_buffers[d].pop_front();
            }
        }
    }
}

}}} // namespace uhd::usrp::bonded
