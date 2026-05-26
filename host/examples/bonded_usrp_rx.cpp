//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_usrp_rx — validation example for the bonded multi-device sync path
// (lib/usrp/bonded/).
//
// USB B2xx devices (B210) cannot be combined into a single multi_usrp instance;
// each physical unit must be opened as a separate multi_usrp::sptr.  This
// example:
//   1. Parses serial0=, serial1=, ... keys from --args.
//   2. Opens one multi_usrp per serial, applies bonded clock/time source config.
//   3. Configures rate / freq / gain on all devices.
//   4. Aligns time via PPS (or set_time_now) on ALL devices after HW setup.
//   5. Issues a common timed burst to every device simultaneously.
//   6. Drains each device stream in a parallel thread.
//   7. Compares first-packet timestamps across devices — PASS if spread < 1 ms.
//
// Typical invocation (two B210s, shared 10 MHz + PPS):
//   bonded_usrp_rx
//     --args "bonded=true,sync_clock_source=external,sync_time_source=external,serial0=30B56D6,serial1=30DBC3C"
//     --rate 10e6 --freq 100e6 --nsamps 10000
//
// Minimal invocation (PPS-only, internal clock):
//   bonded_usrp_rx
//     --args "bonded=true,serial0=30B56D6,serial1=30DBC3C"
//     --rate 10e6 --freq 100e6
//

#include <uhd/types/device_addr.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace po = boost::program_options;

// Threshold for inter-device first-packet timestamp spread (seconds).
static constexpr double PASS_THRESHOLD_SECS = 1e-3; // 1 ms

// Burst timing tolerance for single-device check (µs).
static constexpr double BURST_TIMING_THRESHOLD_US = 10.0;

// ---------------------------------------------------------------------------
// Parse all serialN= keys from a device_addr in order (serial0, serial1, ...)
// ---------------------------------------------------------------------------
static std::vector<std::string> parse_serial_list(const uhd::device_addr_t& addr)
{
    std::vector<std::string> serials;
    if (!addr.has_key("serial0")) {
        if (addr.has_key("serial")) {
            serials.push_back(addr["serial"]);
        }
        return serials;
    }
    for (size_t i = 0;; i++) {
        const std::string key = "serial" + std::to_string(i);
        if (!addr.has_key(key)) {
            break;
        }
        serials.push_back(addr[key]);
    }
    return serials;
}

// ---------------------------------------------------------------------------
// Open one multi_usrp per serial and apply bonded sync configuration.
// Each device's clock_source and time_source are set; ref_locked is polled
// when clock_source is external or gpsdo.
// ---------------------------------------------------------------------------
static void apply_sync_to_device(
    uhd::usrp::multi_usrp::sptr usrp, const uhd::device_addr_t& args)
{
    const std::string clock_src =
        args.has_key("sync_clock_source") ? args["sync_clock_source"] : "internal";
    const std::string time_src =
        args.has_key("sync_time_source") ? args["sync_time_source"] : "external";
    const bool strict =
        args.has_key("sync_strict")
        && (args["sync_strict"] == "true" || args["sync_strict"] == "1");
    const double lock_timeout = args.has_key("sync_lock_timeout")
                                    ? std::stod(args["sync_lock_timeout"])
                                    : 5.0;

    const size_t n = usrp->get_num_mboards();
    for (size_t m = 0; m < n; m++) {
        usrp->set_clock_source(clock_src, m);
        usrp->set_time_source(time_src, m);
    }

    if (clock_src == "external" || clock_src == "gpsdo") {
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
                "Bonded sync: ref not locked within "
                + std::to_string(static_cast<int>(lock_timeout)) + " s.";
            if (strict) {
                throw uhd::runtime_error(msg);
            }
            std::cerr << "[BONDED_SYNC] WARNING: " << msg << "\n";
        } else {
            std::cout << "[BONDED_SYNC] Reference locked.\n";
        }
    }
}

static std::vector<uhd::usrp::multi_usrp::sptr> open_bonded_devices(
    const std::vector<std::string>& serials, const uhd::device_addr_t& base_args)
{
    std::vector<uhd::usrp::multi_usrp::sptr> devices;
    for (size_t i = 0; i < serials.size(); i++) {
        uhd::device_addr_t dev_args = base_args;
        dev_args["serial"]          = serials[i];
        // Remove indexed serial keys so device::make uses only the plain serial=
        for (size_t j = 0; j < serials.size(); j++) {
            dev_args.pop("serial" + std::to_string(j));
        }
        // Disable the bonded intercept in make(): we apply sync below after
        // ALL devices are open and hardware is configured.
        dev_args["bonded"] = "false";
        std::cout << boost::format("[bonded_usrp_rx] Opening serial=%s ...\n")
                         % serials[i];
        devices.push_back(uhd::usrp::multi_usrp::make(dev_args));
    }

    // Apply clock/time source config + ref-lock check on every device.
    std::cout << "[bonded_usrp_rx] Applying sync config to all devices...\n";
    for (size_t d = 0; d < devices.size(); d++) {
        std::cout << boost::format("  Device %u (serial=%s)...\n") % d % serials[d];
        apply_sync_to_device(devices[d], base_args);
    }
    return devices;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    uhd::set_thread_priority_safe();

    // -------------------------------------------------------------------------
    // Command-line options
    // -------------------------------------------------------------------------
    std::string args, subdev;
    double rate, freq, gain, delay;
    size_t nsamps;

    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message.")
        ("args,a",
            po::value<std::string>(&args)->default_value(
                "bonded=true,serial0=30B56D6,serial1=30DBC3C"),
            "UHD device address arguments.\n"
            "Must include bonded=true plus serialN= keys for each device.\n"
            "Optional sync keys:\n"
            "  sync_clock_source  — internal (default), external, gpsdo\n"
            "  sync_time_source   — external (default), internal, gpsdo\n"
            "  sync_strict        — true: throw on lock/PPS failure\n"
            "Example (10 MHz + PPS):\n"
            "  --args \"bonded=true,sync_clock_source=external,"
            "sync_time_source=external,serial0=ABC,serial1=DEF\"")
        ("rate,r",
            po::value<double>(&rate)->default_value(10e6),
            "RX sample rate (Sps).")
        ("freq,f",
            po::value<double>(&freq)->default_value(100e6),
            "RX centre frequency (Hz).")
        ("gain,g",
            po::value<double>(&gain)->default_value(40),
            "RX gain (dB).")
        ("nsamps,n",
            po::value<size_t>(&nsamps)->default_value(10000),
            "Number of samples to capture per channel.")
        ("delay,d",
            po::value<double>(&delay)->default_value(1.5),
            "Seconds in the future at which the timed burst starts.")
        ("subdev",
            po::value<std::string>(&subdev),
            "RX subdevice spec, applied uniformly to all boards.")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout << "bonded_usrp_rx — bonded multi-device receive validation\n\n"
                  << desc << std::endl;
        return EXIT_SUCCESS;
    }
    po::notify(vm);

    const uhd::device_addr_t dev_addr(args);
    const std::string time_src =
        dev_addr.has_key("sync_time_source") ? dev_addr["sync_time_source"] : "external";

    // -------------------------------------------------------------------------
    // Parse serial list and open one multi_usrp per device
    // -------------------------------------------------------------------------
    const std::vector<std::string> serials = parse_serial_list(dev_addr);
    if (serials.empty()) {
        std::cerr << "[bonded_usrp_rx] ERROR: no serialN= keys found in --args.\n";
        return EXIT_FAILURE;
    }
    std::cout << boost::format(
                     "\n[bonded_usrp_rx] Found %zu device serial(s): ") % serials.size();
    for (const auto& s : serials) {
        std::cout << s << " ";
    }
    std::cout << "\n";

    std::vector<uhd::usrp::multi_usrp::sptr> devices =
        open_bonded_devices(serials, dev_addr);
    const size_t num_dev = devices.size();

    // -------------------------------------------------------------------------
    // Hardware setup: rate / freq / gain on every device
    // -------------------------------------------------------------------------
    for (size_t d = 0; d < num_dev; d++) {
        auto& usrp = devices[d];
        if (vm.count("subdev")) {
            usrp->set_rx_subdev_spec(subdev);
        }
        usrp->set_rx_rate(rate);
        const size_t nch = usrp->get_rx_num_channels();
        for (size_t ch = 0; ch < nch; ch++) {
            usrp->set_rx_freq(uhd::tune_request_t(freq), ch);
            usrp->set_rx_gain(gain, ch);
        }
        std::cout << boost::format(
                         "[bonded_usrp_rx] Device %u: %.3f Msps, %.3f MHz, %u ch\n")
                         % d % (usrp->get_rx_rate() / 1e6)
                         % (usrp->get_rx_freq(0) / 1e6) % nch;
    }

    // -------------------------------------------------------------------------
    // Build per-device streamers (all MCR changes happen here)
    // -------------------------------------------------------------------------
    std::vector<uhd::rx_streamer::sptr> streamers(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        const size_t nch = devices[d]->get_rx_num_channels();
        std::vector<size_t> ch_list(nch);
        for (size_t c = 0; c < nch; c++) {
            ch_list[c] = c;
        }
        uhd::stream_args_t sa("fc32");
        sa.channels  = ch_list;
        streamers[d] = devices[d]->get_rx_stream(sa);
    }

    // -------------------------------------------------------------------------
    // Time alignment — AFTER all hardware setup and streamer creation.
    //
    // Multi-device PPS strategy:
    //   set_time_unknown_pps() is WRONG for N>1 devices — it consumes one PPS
    //   edge detecting that PPS is present, then arms the device for the NEXT
    //   edge and sleeps 1 s.  A sequential loop would leave device 1 with no
    //   edge to detect within its 1.1 s window.
    //
    //   Correct approach:
    //   1. Poll get_time_last_pps() on device 0 until it ticks (PPS confirmed).
    //   2. Immediately call set_time_next_pps(0) on ALL devices — we now have
    //      ~999 ms before the next edge to arm every device.
    //   3. Sleep 1 s so the shared PPS edge latches time 0 on all devices.
    // -------------------------------------------------------------------------
    if (time_src == "external" || time_src == "gpsdo") {
        // The B200 resets time_source to "none" on every master clock rate
        // change. Re-apply it now, after all MCR changes from set_rx_rate()
        // and get_rx_stream() are complete, so PPS detection is actually active.
        for (size_t d = 0; d < num_dev; d++) {
            devices[d]->set_time_source(time_src);
        }

        // Give the FPGA 500 ms to stabilise the PPS detection circuit after
        // the MCR change and the time-source re-arm above.
        std::cout << "[bonded_usrp_rx] Waiting for FPGA to stabilise after MCR change...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "[bonded_usrp_rx] Waiting for PPS edge on device 0...\n";
        const auto pps_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
        uhd::time_spec_t last_pps = devices[0]->get_time_last_pps();
        while (devices[0]->get_time_last_pps() == last_pps) {
            if (std::chrono::steady_clock::now() > pps_deadline) {
                throw uhd::runtime_error(
                    "No PPS detected on device 0 within 2.5 s.\n"
                    "Check PPS cable connection to both devices.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // PPS just fired — arm ALL devices before the next edge (~999 ms away).
        std::cout << "[bonded_usrp_rx] PPS detected — arming all devices...\n";
        for (size_t d = 0; d < num_dev; d++) {
            devices[d]->set_time_next_pps(uhd::time_spec_t(0.0));
        }
        // Wait for the next PPS edge to latch time 0 on every device.
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[bonded_usrp_rx] PPS alignment done on all devices.\n";
    } else {
        std::cout << "[bonded_usrp_rx] Internal time: set_time_now(0) on all devices.\n";
        for (auto& usrp : devices) {
            usrp->set_time_now(
                uhd::time_spec_t(0.0), uhd::usrp::multi_usrp::ALL_MBOARDS);
        }
    }

    // -------------------------------------------------------------------------
    // Issue a common timed stream command to every device
    // -------------------------------------------------------------------------
    const uhd::time_spec_t start_time =
        devices[0]->get_time_now() + uhd::time_spec_t(delay);
    std::cout << boost::format(
                     "[bonded_usrp_rx] Scheduling burst: %zu samps at t=%.6f s\n")
                     % nsamps % start_time.get_real_secs();

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps  = nsamps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec  = start_time;
    for (auto& s : streamers) {
        s->issue_stream_cmd(stream_cmd);
    }

    // -------------------------------------------------------------------------
    // Drain each device stream in parallel threads; record first-packet time
    // -------------------------------------------------------------------------
    std::vector<double> first_ts(num_dev, -1.0);
    std::vector<size_t> rx_counts(num_dev, 0);
    std::vector<bool> rx_ok(num_dev, false);
    std::mutex print_mu;

    auto drain_fn = [&](size_t d) {
        auto& streamer      = streamers[d];
        const size_t nch    = devices[d]->get_rx_num_channels();
        const size_t buf_sz = streamer->get_max_num_samps();
        std::vector<std::vector<std::complex<float>>> bufs(
            nch, std::vector<std::complex<float>>(buf_sz));
        std::vector<std::complex<float>*> ptrs(nch);
        for (size_t c = 0; c < nch; c++) {
            ptrs[c] = bufs[c].data();
        }

        double timeout = delay + 0.5;
        bool first     = true;
        size_t total   = 0;
        uhd::rx_metadata_t md;

        while (total < nsamps) {
            const size_t n = streamer->recv(ptrs, buf_sz, md, timeout);
            timeout        = 0.5;
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::lock_guard<std::mutex> lock(print_mu);
                std::cerr << boost::format(
                                 "[bonded_usrp_rx] Device %u: timeout\n") % d;
                return;
            }
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::lock_guard<std::mutex> lock(print_mu);
                std::cerr << boost::format("[bonded_usrp_rx] Device %u: error %s\n")
                                 % d % md.strerror();
                return;
            }
            if (first) {
                first_ts[d] = md.time_spec.get_real_secs();
                std::lock_guard<std::mutex> lock(print_mu);
                std::cout << boost::format(
                                 "[bonded_usrp_rx] Device %u: first packet "
                                 "%zu samps @ t=%.9f s\n")
                                 % d % n % first_ts[d];
                first = false;
            }
            total += n;
        }
        rx_counts[d] = total;
        rx_ok[d]     = true;
    };

    std::vector<std::thread> threads;
    threads.reserve(num_dev);
    for (size_t d = 0; d < num_dev; d++) {
        threads.emplace_back(drain_fn, d);
    }
    for (auto& t : threads) {
        t.join();
    }

    for (size_t d = 0; d < num_dev; d++) {
        std::cout << boost::format(
                         "[bonded_usrp_rx] Device %u: received %zu / %zu samps\n")
                         % d % rx_counts[d] % nsamps;
    }

    // -------------------------------------------------------------------------
    // Validation: compare first-packet timestamps across devices
    // -------------------------------------------------------------------------
    std::cout << "\n[bonded_usrp_rx] --- Synchronisation validation ---\n";

    std::vector<double> valid_ts;
    for (size_t d = 0; d < num_dev; d++) {
        if (rx_ok[d] && first_ts[d] >= 0.0) {
            valid_ts.push_back(first_ts[d]);
            std::cout << boost::format("  Device %u first-packet ts : %.9f s\n")
                             % d % first_ts[d];
        }
    }

    bool sync_ok = false;
    if (valid_ts.size() < num_dev) {
        std::cout << "  FAIL — one or more devices did not receive data.\n";
    } else if (num_dev == 1) {
        const double err_us =
            std::abs(first_ts[0] - start_time.get_real_secs()) * 1e6;
        const bool timing_ok = err_us <= BURST_TIMING_THRESHOLD_US;
        std::cout << boost::format(
                         "  Burst timing error  : %.3f µs  [%s]  (threshold %.0f µs)\n")
                         % err_us % (timing_ok ? "OK" : "FAIL")
                         % BURST_TIMING_THRESHOLD_US;
        sync_ok = timing_ok;
    } else {
        const double ts_min  = *std::min_element(valid_ts.begin(), valid_ts.end());
        const double ts_max  = *std::max_element(valid_ts.begin(), valid_ts.end());
        const double spread  = ts_max - ts_min;
        const bool spread_ok = spread <= PASS_THRESHOLD_SECS;
        std::cout << boost::format(
                         "  Inter-device spread : %.3f µs  [%s]  (threshold %.0f µs)\n")
                         % (spread * 1e6) % (spread_ok ? "OK" : "FAIL")
                         % (PASS_THRESHOLD_SECS * 1e6);
        if (!spread_ok) {
            std::cout
                << "  Tip: ensure PPS and (optionally) 10 MHz are connected to all\n"
                   "       devices and use\n"
                   "       sync_clock_source=external,sync_time_source=external.\n";
        }
        sync_ok = spread_ok;
    }

    std::cout << "\n  Result: " << (sync_ok ? "PASS" : "FAIL") << "\n";
    return sync_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
