//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/config.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <atomic>
#include <complex>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Synchronized wideband transmitter using two bonded B210s.
 *
 * Mirrors bonded_receiver: opens two devices on a shared external 10 MHz
 * reference + PPS, configures TX hardware to fc ± T/4 center frequencies,
 * aligns their clocks to a common t=0 via the PPS edge, and provides both
 * burst (timed) and continuous (queued) transmission.
 *
 * The band_splitter engine produces the per-radio streams; this class only
 * manages the device/sync/TX-streamer lifecycle.
 *
 * Per-device integer sample-delay trim: positive trim T pads device d's
 * output with T zero samples at the start, delaying its effective start by
 * T / per_radio_rate seconds relative to the common time base.  This corrects
 * any residual inter-device skew that survives the shared 10 MHz + PPS sync.
 */
class UHD_API bonded_transmitter
{
public:
    //! Configuration for the bonded transmitter.
    struct config {
        std::vector<std::string> serials; //!< Ordered list of device serials
        std::string clock_source = "external";
        std::string time_source  = "external";
        double rate = 10e6;              //!< Per-radio TX rate (Hz)
        double freq = 100e6;             //!< Uniform center frequency (Hz)
        std::vector<double> freq_plan;   //!< Optional per-device center freqs (Hz)
        double gain = 0.0;               //!< TX gain (dB)
        std::string subdev;              //!< Optional subdev spec
        std::string stream_args;         //!< Optional TX stream args
        bool strict        = false;      //!< Throw on lock failure vs. warn
        double lock_timeout = 5.0;       //!< Seconds to wait for ref_locked

        //! Per-device integer sample-delay trim (samples at `rate`).
        //! Positive value T: prepend T zero-samples to that device's stream.
        //! Must be either empty (no trim) or the same length as serials.
        std::vector<int> sample_delay_trim;
    };

    //! Result of a transmit operation.
    struct tx_result {
        std::vector<bool> success;         //!< Per-device transmit success
        std::vector<size_t> underrun_counts; //!< Per-device underrun count
        std::string error_message;         //!< Non-empty on failure
    };

    explicit bonded_transmitter(const config& cfg);
    ~bonded_transmitter();

    // Non-copyable
    bonded_transmitter(const bonded_transmitter&) = delete;
    bonded_transmitter& operator=(const bonded_transmitter&) = delete;

    /*!
     * Full setup: open devices, sync clocks, configure HW, create TX streamers,
     * align time via PPS.
     * \throws uhd::runtime_error on failure (or warns if !strict)
     */
    void configure();

    // --- Burst mode ---

    /*!
     * Send a one-shot timed burst on all devices simultaneously.
     *
     * \param samples   Per-device complex samples (samples[device_index])
     * \param delay_sec Start time = now() + delay_sec
     * \return tx_result with per-device success + underrun counts
     */
    tx_result send_burst(
        const std::vector<std::vector<std::complex<float>>>& samples,
        double delay_sec = 1.5);

    // --- Continuous mode ---

    /*!
     * Start background send threads and arm a continuous timed TX stream.
     * Call queue_samples() to push data; call stop() when done.
     */
    void start_continuous();

    /*!
     * Push n samples per device into the send queues.
     * Thread-safe; called from the GNU Radio work thread.
     *
     * \param ptrs  ptrs[d] points to n complex<float> samples for device d
     * \param n     Number of samples per device
     */
    void queue_samples(const std::vector<const std::complex<float>*>& ptrs,
        size_t n);

    /*!
     * Stop continuous streaming: flush queues, join send threads.
     */
    void stop();

    // --- Queries ---
    size_t num_devices() const { return _config.serials.size(); }
    bool is_streaming() const { return _streaming; }
    bool is_configured() const { return _configured; }
    std::vector<size_t> get_underrun_counts() const;

private:
    // Item in the per-device send queue.
    struct tx_chunk {
        std::vector<std::complex<float>> data;
    };

    config _config;
    bool _configured = false;
    bool _streaming  = false;
    size_t _channels_per_device = 1; // TX bonding uses 1 TX channel per device

    std::vector<uhd::usrp::multi_usrp::sptr> _devices;
    std::vector<uhd::tx_streamer::sptr> _streamers;

    // Continuous mode: per-device queues + send threads.
    std::vector<std::deque<tx_chunk>> _tx_queues;
    std::vector<std::mutex> _tx_mutexes;
    std::vector<std::condition_variable> _tx_cvs;
    std::vector<std::thread> _send_threads;
    std::atomic<bool> _stop_flag{false};

    // Async-message threads count underruns from the TX streamer.
    std::vector<std::thread> _async_threads;
    std::vector<std::atomic<size_t>> _underrun_counts;

    // Internal helpers.
    void _open_devices();
    void _apply_sync();
    void _configure_hardware();
    void _create_streamers();
    void _align_time();
    void _send_loop_with_start(size_t device_index);
    void _async_msg_loop(size_t device_index);

    // Shared TX start time: computed by the first send thread to receive data
    // (both threads use the same value so they start simultaneously).
    std::mutex _stream_start_mutex;
    bool _stream_start_set = false;
    uhd::time_spec_t _stream_start_time;
};

}}} // namespace uhd::usrp::bonded
