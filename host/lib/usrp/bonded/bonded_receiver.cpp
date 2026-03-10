// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#include <uhd/usrp/bonded/bonded_receiver.hpp>
#include <uhd/exception.hpp>
#include <uhd/utils/log.hpp>
#include <chrono>
#include <thread>

namespace uhd { namespace usrp { namespace bonded {

bonded_receiver::bonded_receiver(std::vector<uhd::usrp::multi_usrp::sptr> devices,
    double sample_rate,
    const std::vector<double>& freqs)
    : _devices(std::move(devices)), _sample_rate(sample_rate)
{
    const size_t N = _devices.size();
    if (N < 2) {
        throw uhd::runtime_error(
            "bonded_receiver requires at least 2 devices");
    }
    if (freqs.size() != N) {
        throw uhd::value_error(
            "bonded_receiver: freqs.size() must match devices.size()");
    }

    _streams.resize(N);
    uhd::stream_args_t args("fc32", "sc16");
    args.channels = {0}; // one channel per independent multi_usrp

    for (size_t i = 0; i < N; ++i) {
        _devices[i]->set_rx_rate(sample_rate, 0);
        _devices[i]->set_rx_freq(uhd::tune_request_t(freqs[i]), 0);

        _streams[i].streamer    = _devices[i]->get_rx_stream(args);
        _streams[i].center_freq = _devices[i]->get_rx_freq(0);
        _streams[i].buffer.resize(BUFFER_SIZE);
    }
}

void bonded_receiver::start_streaming()
{
    if (_streaming) {
        return;
    }

    // Use device 0 as the timing reference.  All devices were synchronised
    // to device 0 by make_bonded_usb(), so a common time_spec works.
    const uhd::time_spec_t start_time =
        _devices[0]->get_time_now(0) + uhd::time_spec_t(0.5);

    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
    cmd.stream_now = false;
    cmd.time_spec  = start_time;

    for (auto& s : _streams) {
        s.streamer->issue_stream_cmd(cmd);
    }

    _streaming = true;
}

void bonded_receiver::stop_streaming()
{
    if (!_streaming) {
        return;
    }

    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    cmd.stream_now = true;

    for (auto& s : _streams) {
        s.streamer->issue_stream_cmd(cmd);
    }

    _streaming = false;
}

bool bonded_receiver::recv_aligned(size_t nsamps, double timeout)
{
    const size_t actual_nsamps = std::min(nsamps, BUFFER_SIZE);
    bool ok = true;

    for (size_t i = 0; i < _streams.size(); ++i) {
        size_t n = _streams[i].streamer->recv(
            _streams[i].buffer.data(),
            actual_nsamps,
            _streams[i].metadata,
            timeout);

        if (_streams[i].metadata.error_code
            != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            _handle_overflow(i);
            ok = false;
        } else if (n == 0) {
            ok = false;
        }
    }

    return ok;
}

double bonded_receiver::get_stream_time_seconds() const
{
    return _streams[0].metadata.time_spec.get_real_secs();
}

void bonded_receiver::_handle_overflow(size_t idx)
{
    const auto& md = _streams[idx].metadata;
    if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
        UHD_LOG_WARNING(
            "BONDED",
            "Device " << idx << ": overflow at "
                      << md.time_spec.get_real_secs() << " s. "
                      << "Timestamp alignment recovers automatically.");
    } else {
        UHD_LOG_WARNING(
            "BONDED",
            "Device " << idx << ": error code "
                      << md.error_code << " at "
                      << md.time_spec.get_real_secs() << " s.");
    }
}

}}} // namespace uhd::usrp::bonded
