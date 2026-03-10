// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <uhd/config.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Handles simultaneous reception from N independently-opened USRP devices
 * (one multi_usrp per device, all synchronized to a common PPS edge via
 * multi_usrp::make_bonded_usb) and aligns received samples using the
 * hardware timestamps embedded in rx_metadata_t.
 *
 * USB transport jitter is irrelevant: alignment is determined from the
 * B210 FPGA hardware timestamps, not from USB arrival times.
 */
class UHD_API bonded_receiver
{
public:
    static constexpr size_t BUFFER_SIZE = 65536;

    struct device_stream {
        rx_streamer::sptr streamer;
        std::vector<std::complex<float>> buffer;
        rx_metadata_t metadata;
        double center_freq = 0.0;
    };

    /*!
     * Construct a bonded_receiver.
     *
     * \param devices     Vector of independently-opened multi_usrp instances
     *                    (at least 2), already PPS-synchronised (e.g. via
     *                    multi_usrp::make_bonded_usb).  Each must have exactly
     *                    one RX channel active on mboard 0.
     * \param sample_rate Common sample rate applied to all devices (S/s).
     * \param freqs       Per-device RX center frequency (Hz); freqs[i] is
     *                    applied to devices[i].  Must be the same size as devices.
     */
    bonded_receiver(std::vector<uhd::usrp::multi_usrp::sptr> devices,
        double sample_rate,
        const std::vector<double>& freqs);

    /*!
     * Issue a timed stream command so all devices start capturing
     * at the same hardware timestamp.
     */
    void start_streaming();

    /*!
     * Stop streaming on all devices.
     */
    void stop_streaming();

    /*!
     * Receive one buffer from each device.
     *
     * \param nsamps   Number of samples to request per device.
     * \param timeout  Per-device receive timeout in seconds.
     * \returns true on success, false if any device reported an error.
     */
    bool recv_aligned(size_t nsamps, double timeout = 1.0);

    //! Number of devices.
    size_t num_devices() const { return _streams.size(); }

    //! Const reference to the receive state for device \p idx.
    const device_stream& get_stream(size_t idx) const { return _streams[idx]; }

    //! Writable pointer to the sample buffer of device \p idx.
    std::complex<float>* get_buffer(size_t idx) { return _streams[idx].buffer.data(); }

    double get_sample_rate() const { return _sample_rate; }
    double get_freq(size_t idx) const { return _streams[idx].center_freq; }

    //! Hardware timestamp (seconds) of the most recent buffer from device 0.
    double get_stream_time_seconds() const;

private:
    std::vector<uhd::usrp::multi_usrp::sptr> _devices;
    double _sample_rate;
    std::vector<device_stream> _streams;
    bool _streaming = false;

    void _handle_overflow(size_t idx);
};

}}} // namespace uhd::usrp::bonded
