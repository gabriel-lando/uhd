// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <uhd/config.hpp>
#include <uhd/stream.hpp>
#include <uhd/usrp/bonded/bonded_receiver.hpp>
#include <uhd/usrp/bonded/phase_corrector.hpp>
#include <uhd/usrp/bonded/pps_drift_estimator.hpp>
#include <uhd/usrp/bonded/spectrum_stitcher.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Quality metrics reported by quality().
 *
 * For N bonded devices, drift/uncertainty/phase_rate are per non-reference
 * device (indices 0..N-2 correspond to physical devices 1..N-1 relative to
 * the reference device 0).
 */
struct UHD_API bonding_quality_metrics {
    std::vector<double> drift_ppm;       //!< Oscillator drift per non-ref device (ppm)
    std::vector<double> uncertainty_ppm; //!< 1-sigma drift uncertainty per device
    std::vector<double> phase_rate_rad_s;//!< Phase correction rate per device (rad/s)
    int pps_samples_used = 0;            //!< PPS edges used in regression
    bool pps_healthy     = true;         //!< false if PPS not seen for > 2 s
    bool valid           = false;        //!< true once drift estimate is ready

    std::string to_string() const;
};

/*!
 * High-level bonded RX streamer for N USB USRPs.
 *
 * Wraps N UHD rx_streamers (one per device) into a single recv() call that
 * returns a stitched wideband spectrum N times wider than a single device.
 *
 * Typical usage:
 * \code
 *   auto devices = uhd::usrp::multi_usrp::make_bonded_usb({"SN_A", "SN_B"});
 *
 *   uhd::usrp::bonded::bonded_rx_streamer bonded(devices, 20e6, 2.4e9);
 *   bonded.start();
 *
 *   std::vector<std::complex<float>> buf(bonded.output_size());
 *   uhd::rx_metadata_t md;
 *   while (running) {
 *       size_t n = bonded.recv(buf.data(), buf.size(), md, 1.0);
 *   }
 *   bonded.stop();
 * \endcode
 *
 * Thread safety: recv() is NOT thread-safe.  Call it from a single thread.
 * The background PPS drift-tracking thread is managed internally.
 */
class UHD_API bonded_rx_streamer
{
public:
    static constexpr size_t DEFAULT_FFT_SIZE = 4096;

    /*!
     * Construct and configure the N-device bonded streamer.
     *
     * \param devices      Vector of multi_usrp handles (one per physical USRP),
     *                     already synchronized by make_bonded_usb().
     * \param sample_rate  Per-device sample rate (S/s).
     * \param center_freq  Center frequency of the COMBINED band (Hz).
     *                     Devices are uniformly spaced around this center.
     * \param fft_size     FFT size used for stitching (default: 4096).
     */
    bonded_rx_streamer(std::vector<uhd::usrp::multi_usrp::sptr> devices,
        double sample_rate,
        double center_freq,
        size_t fft_size = DEFAULT_FFT_SIZE);

    ~bonded_rx_streamer();

    // Non-copyable
    bonded_rx_streamer(const bonded_rx_streamer&)            = delete;
    bonded_rx_streamer& operator=(const bonded_rx_streamer&) = delete;

    //! Start streaming on all devices.
    void start();

    //! Stop streaming and join the background drift-tracking thread.
    void stop();

    /*!
     * Receive the next stitched wideband spectrum.
     *
     * \param output   Output buffer (must hold at least output_size() values).
     * \param noutput  Size of the output buffer.
     * \param metadata Metadata from device 0's most recent packet.
     * \param timeout  Per-device receive timeout (seconds).
     * \returns        Number of output frequency bins written, or 0 on error.
     */
    size_t recv(std::complex<float>* output,
        size_t noutput,
        uhd::rx_metadata_t& metadata,
        double timeout = 1.0);

    //! Number of output frequency bins (N_devices × usable_bins_per_device).
    size_t output_size() const;

    //! Latest bonding quality metrics.
    bonding_quality_metrics quality() const;

private:
    std::vector<uhd::usrp::multi_usrp::sptr> _devices;
    double _sample_rate;
    double _center_freq;

    std::unique_ptr<bonded_receiver> _receiver;
    std::unique_ptr<spectrum_stitcher> _stitcher;
    std::unique_ptr<pps_drift_estimator> _drift_estimator;
    // Phase correctors for devices 1..N-1 (device 0 is the reference).
    std::vector<std::unique_ptr<phase_corrector>> _phase_correctors;

    // Background PPS drift tracking
    std::thread _drift_thread;
    std::atomic<bool> _running{false};
    void _drift_tracking_loop();

    // Cached quality metrics (updated by background thread)
    mutable bonding_quality_metrics _quality;
    double _last_pps_time = 0.0;
};

}}} // namespace uhd::usrp::bonded
