//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "bonded_transmitter.hpp"

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace uhd { namespace usrp { namespace bonded {

bonded_transmitter::bonded_transmitter(const config& cfg) : _config(cfg)
{
    if (!cfg.sample_delay_trim.empty()
        && cfg.sample_delay_trim.size() != cfg.serials.size()) {
        throw std::invalid_argument(
            "bonded_transmitter: sample_delay_trim must be empty or match "
            "the number of serials");
    }
}

bonded_transmitter::~bonded_transmitter()
{
    if (_streaming) {
        stop();
    }
}

// =========================================================================
// configure() — full setup pipeline
// =========================================================================
void bonded_transmitter::configure()
{
    if (_configured) {
        throw uhd::runtime_error("bonded_transmitter::configure() called twice");
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
bonded_transmitter::tx_result bonded_transmitter::send_burst(
    const std::vector<std::vector<std::complex<float>>>& samples,
    double delay_sec)
{
    if (!_configured) {
        throw uhd::runtime_error(
            "bonded_transmitter::send_burst() called before configure()");
    }
    if (_streaming) {
        throw uhd::runtime_error(
            "bonded_transmitter::send_burst() called while continuous streaming");
    }
    if (samples.size() != _devices.size()) {
        throw std::invalid_argument(
            "bonded_transmitter::send_burst(): samples must have one entry "
            "per device");
    }

    const size_t num_dev = _devices.size();
    tx_result result;
    result.success.assign(num_dev, false);
    result.underrun_counts.assign(num_dev, 0);

    const uhd::time_spec_t start_time =
        _devices[0]->get_time_now() + uhd::time_spec_t(delay_sec);

    std::cout << boost::format(
                     "[bonded_transmitter] Scheduling burst at t=%.6f s\n")
                     % start_time.get_real_secs();

    // Send from each device in parallel threads.
    std::mutex print_mu;
    auto send_fn = [&](size_t d) {
        auto& streamer       = _streamers[d];
        const auto& dev_samp = samples[d];

        // Apply integer sample-delay trim: prepend zeros if trim[d] > 0.
        std::vector<std::complex<float>> padded;
        const std::complex<float>* send_ptr = dev_samp.data();
        size_t send_n                       = dev_samp.size();
        if (!_config.sample_delay_trim.empty() && _config.sample_delay_trim[d] > 0) {
            const size_t pad = static_cast<size_t>(_config.sample_delay_trim[d]);
            padded.resize(pad + dev_samp.size(), std::complex<float>(0.0f, 0.0f));
            std::copy(dev_samp.begin(), dev_samp.end(), padded.begin() + pad);
            send_ptr = padded.data();
            send_n   = padded.size();
        }

        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst   = false;
        md.has_time_spec  = true;
        md.time_spec      = start_time;

        const size_t buf_sz = streamer->get_max_num_samps();
        size_t total        = 0;
        bool ok             = true;

        while (total < send_n) {
            const bool last = (total + buf_sz >= send_n);
            if (last) {
                md.end_of_burst = true;
            }
            const size_t chunk = std::min(buf_sz, send_n - total);
            const std::vector<const std::complex<float>*> ptrs = {send_ptr + total};
            const size_t n = streamer->send(ptrs, chunk, md, 1.0);
            if (n == 0) {
                std::lock_guard<std::mutex> lock(print_mu);
                std::cerr << boost::format(
                                 "[bonded_transmitter] Device %u: send() returned 0\n")
                                 % d;
                ok = false;
                break;
            }
            md.start_of_burst = false;
            md.has_time_spec  = false;
            total += n;
        }
        result.success[d] = ok;
    };

    std::vector<std::thread> threads;
    threads.reserve(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        threads.emplace_back(send_fn, d);
    }
    for (auto& t : threads) {
        t.join();
    }

    for (size_t d = 0; d < num_dev; d++) {
        result.underrun_counts[d] = _underrun_counts[d].load();
    }
    return result;
}

// =========================================================================
// Continuous mode
// =========================================================================
void bonded_transmitter::start_continuous()
{
    if (!_configured) {
        throw uhd::runtime_error(
            "bonded_transmitter::start_continuous() called before configure()");
    }
    if (_streaming) {
        throw uhd::runtime_error("bonded_transmitter: already streaming");
    }

    const size_t num_dev = _devices.size();
    _stop_flag.store(false);
    _stream_start_set = false; // reset so the first chunk triggers start-time computation
    _tx_queues.resize(num_dev);
    _tx_mutexes = std::vector<std::mutex>(num_dev);
    _tx_cvs     = std::vector<std::condition_variable>(num_dev);
    _underrun_counts = std::vector<std::atomic<size_t>>(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        _underrun_counts[d].store(0);
    }

    // Pre-queue the per-device zero-padding from sample_delay_trim.
    if (!_config.sample_delay_trim.empty()) {
        for (size_t d = 0; d < num_dev; d++) {
            const int trim = _config.sample_delay_trim[d];
            if (trim > 0) {
                tx_chunk pad;
                pad.data.assign(static_cast<size_t>(trim),
                    std::complex<float>(0.0f, 0.0f));
                std::lock_guard<std::mutex> lk(_tx_mutexes[d]);
                _tx_queues[d].push_back(std::move(pad));
            }
        }
    }

    // Launch send threads.  Each thread blocks until the first data chunk
    // arrives from the GR scheduler, then computes a shared TX start time
    // (50 ms in the future) so both radios start at exactly the same sample.
    _send_threads.reserve(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        _send_threads.emplace_back([this, d]() {
            _send_loop_with_start(d);
        });
    }

    _streaming = true;
}

void bonded_transmitter::queue_samples(
    const std::vector<const std::complex<float>*>& ptrs, size_t n)
{
    if (!_streaming) {
        throw uhd::runtime_error(
            "bonded_transmitter::queue_samples() called while not streaming");
    }
    if (ptrs.size() != _devices.size()) {
        throw std::invalid_argument(
            "bonded_transmitter::queue_samples(): ptrs must have one entry per "
            "device");
    }
    for (size_t d = 0; d < ptrs.size(); d++) {
        tx_chunk c;
        c.data.assign(ptrs[d], ptrs[d] + n);
        std::lock_guard<std::mutex> lk(_tx_mutexes[d]);
        _tx_queues[d].push_back(std::move(c));
        _tx_cvs[d].notify_one();
    }
}

void bonded_transmitter::stop()
{
    if (!_streaming) return;

    _stop_flag.store(true);

    // Wake all send threads so they can exit.
    for (size_t d = 0; d < _tx_cvs.size(); d++) {
        _tx_cvs[d].notify_all();
    }

    for (auto& t : _send_threads) {
        if (t.joinable()) t.join();
    }
    _send_threads.clear();

    for (auto& t : _async_threads) {
        if (t.joinable()) t.join();
    }
    _async_threads.clear();
    _tx_queues.clear();
    _streaming = false;
}

std::vector<size_t> bonded_transmitter::get_underrun_counts() const
{
    std::vector<size_t> counts(_underrun_counts.size(), 0);
    for (size_t d = 0; d < _underrun_counts.size(); d++) {
        counts[d] = _underrun_counts[d].load();
    }
    return counts;
}

// =========================================================================
// Private helpers
// =========================================================================

void bonded_transmitter::_open_devices()
{
    std::cout << boost::format(
                     "[bonded_transmitter] Opening %zu device(s)...\n")
                     % _config.serials.size();

    for (const auto& serial : _config.serials) {
        uhd::device_addr_t dev_args;
        dev_args["serial"] = serial;
        std::cout << boost::format(
                         "[bonded_transmitter] Opening serial=%s ...\n")
                         % serial;
        _devices.push_back(uhd::usrp::multi_usrp::make(dev_args));
    }

    // Init underrun counters here (size is now known).
    _underrun_counts = std::vector<std::atomic<size_t>>(_devices.size());
    for (auto& c : _underrun_counts) c.store(0);
}

void bonded_transmitter::_apply_sync()
{
    std::cout << "[bonded_transmitter] Applying sync config to all devices...\n";

    for (size_t d = 0; d < _devices.size(); d++) {
        auto& usrp       = _devices[d];
        const size_t nmb = usrp->get_num_mboards();
        for (size_t m = 0; m < nmb; m++) {
            usrp->set_clock_source(_config.clock_source, m);
            usrp->set_time_source(_config.time_source, m);
        }

        if (_config.clock_source == "external"
            || _config.clock_source == "gpsdo") {
            const auto deadline = std::chrono::steady_clock::now()
                                  + std::chrono::duration<double>(
                                      _config.lock_timeout);
            bool locked = false;
            while (std::chrono::steady_clock::now() < deadline) {
                bool all_locked = true;
                for (size_t m = 0; m < nmb; m++) {
                    try {
                        if (!usrp->get_mboard_sensor("ref_locked", m)
                                 .to_bool()) {
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
                    + std::to_string(static_cast<int>(_config.lock_timeout))
                    + " s.";
                if (_config.strict) {
                    throw uhd::runtime_error(msg);
                }
                std::cerr << "[bonded_transmitter] WARNING: " << msg << "\n";
            } else {
                std::cout << boost::format(
                                 "[bonded_transmitter] Device %u: reference "
                                 "locked.\n")
                                 % d;
            }
        }
    }
}

void bonded_transmitter::_configure_hardware()
{
    const bool use_freq_plan = !_config.freq_plan.empty();
    if (use_freq_plan && _config.freq_plan.size() != _devices.size()) {
        throw uhd::runtime_error(
            "bonded_transmitter: freq_plan size must match number of devices");
    }

    for (size_t d = 0; d < _devices.size(); d++) {
        auto& usrp = _devices[d];
        if (!_config.subdev.empty()) {
            usrp->set_tx_subdev_spec(_config.subdev);
        }
        usrp->set_tx_rate(_config.rate);
        const double dev_freq =
            use_freq_plan ? _config.freq_plan[d] : _config.freq;
        const size_t nch = usrp->get_tx_num_channels();
        for (size_t ch = 0; ch < nch; ch++) {
            usrp->set_tx_freq(uhd::tune_request_t(dev_freq), ch);
            usrp->set_normalized_tx_gain(_config.gain, ch);
        }
        std::cout << boost::format(
                         "[bonded_transmitter] Device %u: %.3f Msps, "
                         "%.3f MHz, %u ch%s\n")
                         % d % (usrp->get_tx_rate() / 1e6)
                         % (usrp->get_tx_freq(0) / 1e6) % nch
                         % (use_freq_plan ? " (freq_plan)" : "");
    }
    // Bonded TX uses exactly one TX chain per device (one half-band per B210).
    // Force this regardless of how many channels the device advertises.
    _channels_per_device = 1;
}

void bonded_transmitter::_create_streamers()
{
    for (size_t d = 0; d < _devices.size(); d++) {
        std::vector<size_t> ch_list(_channels_per_device);
        for (size_t c = 0; c < _channels_per_device; c++) ch_list[c] = c;

        uhd::stream_args_t sa("fc32");
        if (!_config.stream_args.empty()) {
            sa.args = uhd::device_addr_t(_config.stream_args);
        }
        sa.channels = ch_list;
        _streamers.push_back(_devices[d]->get_tx_stream(sa));
    }

    // Start async-message listener threads (one per device).
    _async_threads.reserve(_devices.size());
    for (size_t d = 0; d < _devices.size(); d++) {
        _async_threads.emplace_back(
            &bonded_transmitter::_async_msg_loop, this, d);
    }
}

void bonded_transmitter::_align_time()
{
    if (_config.time_source == "external"
        || _config.time_source == "gpsdo") {
        // Re-apply after MCR changes (same B210 quirk as on the RX side).
        for (auto& dev : _devices) {
            dev->set_time_source(_config.time_source);
        }

        std::cout << "[bonded_transmitter] Waiting for FPGA to stabilise...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[bonded_transmitter] Waiting for PPS edge on device 0...\n";
        const auto pps_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
        uhd::time_spec_t last_pps = _devices[0]->get_time_last_pps();
        while (_devices[0]->get_time_last_pps() == last_pps) {
            if (std::chrono::steady_clock::now() > pps_deadline) {
                throw uhd::runtime_error(
                    "No PPS detected on device 0 within 2.5 s.\n"
                    "Check PPS cable to all devices.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "[bonded_transmitter] PPS detected — arming all devices...\n";
        for (auto& dev : _devices) {
            dev->set_time_next_pps(uhd::time_spec_t(0.0));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[bonded_transmitter] PPS alignment done.\n";
    } else {
        std::cout << "[bonded_transmitter] Internal time: set_time_now(0).\n";
        for (auto& dev : _devices) {
            dev->set_time_now(uhd::time_spec_t(0.0),
                uhd::usrp::multi_usrp::ALL_MBOARDS);
        }
    }
}

void bonded_transmitter::_send_loop_with_start(size_t d)
{
    uhd::set_thread_priority_safe();

    auto& streamer  = _streamers[d];
    const size_t bs = streamer->get_max_num_samps();

    // Silence buffer: sent when the queue is momentarily empty to keep the
    // hardware stream alive between 802.11 bursts (avoids underruns).
    std::vector<std::complex<float>> silence(bs, {0.0f, 0.0f});

    // ------------------------------------------------------------------
    // Phase 1: block until the first data chunk arrives from the GR
    // scheduler.  The phy pipeline has large internal buffers (up to ~100 ms
    // worth of samples at 20 MHz) that must fill before work() is first
    // called; we must not start the hardware stream before that happens.
    // ------------------------------------------------------------------
    {
        std::unique_lock<std::mutex> lk(_tx_mutexes[d]);
        _tx_cvs[d].wait(lk, [&]() {
            return !_tx_queues[d].empty() || _stop_flag.load();
        });
    }
    if (_stop_flag.load()) return;

    // ------------------------------------------------------------------
    // Phase 2: compute a shared TX start time (50 ms from now), using
    // device 0 as the time reference so both send threads schedule the
    // same absolute start sample on the synchronized time base.
    // ------------------------------------------------------------------
    uhd::time_spec_t start_time;
    {
        std::lock_guard<std::mutex> lk(_stream_start_mutex);
        if (!_stream_start_set) {
            _stream_start_time =
                _devices[0]->get_time_now() + uhd::time_spec_t(0.05);
            _stream_start_set = true;
        }
        start_time = _stream_start_time;
    }

    // ------------------------------------------------------------------
    // Phase 3: continuous send loop.
    //
    // Two-level strategy to avoid both underruns and mid-frame silence:
    //
    //   INTRA-FRAME: consecutive GR work() calls for the same 802.11 frame
    //   are spaced ~50-100 µs apart (phy pipeline runs at CPU speed).
    //   We wait up to 1 ms for the next chunk.  If it arrives in time we
    //   send real data with no silence injected.
    //
    //   INTER-FRAME: the 300 ms strobe gap far exceeds the 1 ms threshold,
    //   so the wait times out.  We then send one FIFO-depth burst of silence
    //   (~26 ms on B210) that keeps the hardware fed, then wait again.
    //   Each silence burst refills the FIFO to capacity, so regardless of
    //   how many inter-frame cycles elapse the hardware never starves.
    //
    // Because wait_for() checks the predicate before sleeping, a notify_one
    // that arrives while we're inside the silence burst is not lost: the
    // queue will be non-empty when we reach wait_for() next iteration and
    // it returns immediately without waiting.
    // ------------------------------------------------------------------

    // Silence burst: sized to one B210 TX FIFO depth (262144 samples =
    // 26.2 ms @ 10 MHz).  Using max(bs*128, 262144) generalises across
    // USRP models.  streamer->send() blocks at hardware rate once the FIFO
    // is full, so the burst naturally takes ~26 ms of real wall time and
    // keeps the FIFO continuously full throughout the inter-frame gap.
    const size_t silence_burst = std::max(bs * 128, size_t(262144));

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec      = start_time;

    while (!_stop_flag.load()) {
        std::vector<std::complex<float>> chunk_data;
        {
            std::unique_lock<std::mutex> lk(_tx_mutexes[d]);
            // Wait up to 1 ms for real data.
            // • Intra-frame: next GR work() chunk arrives in < 200 µs → wakes fast.
            // • Inter-frame: 300 ms gap → times out → silence burst follows.
            _tx_cvs[d].wait_for(lk, std::chrono::milliseconds(1), [&]() {
                return !_tx_queues[d].empty() || _stop_flag.load();
            });
            if (!_tx_queues[d].empty()) {
                chunk_data = std::move(_tx_queues[d].front().data);
                _tx_queues[d].pop_front();
            }
        }

        if (_stop_flag.load() && chunk_data.empty()) break;

        if (chunk_data.empty()) {
            // Inter-frame gap: send one FIFO-depth of silence.
            // send() blocks at hardware rate once FIFO is full, so this
            // burst takes ~26 ms and refills the FIFO to capacity.
            for (size_t sent = 0; sent < silence_burst && !_stop_flag.load();) {
                const size_t n = std::min(bs, silence_burst - sent);
                const std::vector<const std::complex<float>*> ptrs = {
                    silence.data()};
                streamer->send(ptrs, n, md, 0.1);
                md.start_of_burst = false;
                md.has_time_spec  = false;
                sent += n;
            }
        } else {
            // Real data: send the entire chunk without injecting any silence.
            size_t off = 0;
            while (off < chunk_data.size() && !_stop_flag.load()) {
                const size_t n = std::min(bs, chunk_data.size() - off);
                const std::vector<const std::complex<float>*> ptrs = {
                    chunk_data.data() + off};
                streamer->send(ptrs, n, md, 0.1);
                md.start_of_burst = false;
                md.has_time_spec  = false;
                off += n;
            }
        }
    }

    // Send end-of-burst marker with a real (silent) sample.
    uhd::tx_metadata_t eob;
    eob.end_of_burst = true;
    const std::complex<float> silence_sample(0.0f, 0.0f);
    const std::vector<const std::complex<float>*> eob_ptr = {&silence_sample};
    streamer->send(eob_ptr, 1, eob, 0.1);
}

void bonded_transmitter::_async_msg_loop(size_t d)
{
    while (!_stop_flag.load()) {
        uhd::async_metadata_t amd;
        if (_streamers[d]->recv_async_msg(amd, 0.05)) {
            if (amd.event_code
                == uhd::async_metadata_t::EVENT_CODE_UNDERFLOW) {
                _underrun_counts[d].fetch_add(1);
                std::cerr << boost::format(
                                 "[bonded_transmitter] Device %u: underrun "
                                 "(U)\n")
                                 % d;
            }
        }
    }
}

}}} // namespace uhd::usrp::bonded
