//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_usrp_rx — validation example for bonded multi-device synchronized RX.
//
// Uses bonded_receiver from lib/usrp/bonded/ to demonstrate burst capture
// across N USB B210 devices with shared reference synchronization.
//
// Typical invocation (two B210s, shared 10 MHz + PPS):
//   bonded_usrp_rx
//     --args "serial0=30B56D6,serial1=30DBC3C,sync_clock_source=external,sync_time_source=external"
//     --rate 10e6 --freq 100e6 --nsamps 10000
//

#include "../lib/usrp/bonded/bonded_receiver.hpp"
#include "../lib/usrp/bonded/frequency_plan.hpp"

#include <uhd/types/device_addr.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace po = boost::program_options;

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

static std::vector<double> parse_freq_plan(const uhd::device_addr_t& addr)
{
    std::vector<double> freqs;

    // Prefer explicit indexed keys: freq0, freq1, ...
    if (addr.has_key("freq0")) {
        for (size_t i = 0;; i++) {
            const std::string key = "freq" + std::to_string(i);
            if (!addr.has_key(key)) {
                break;
            }
            freqs.push_back(std::stod(addr[key]));
        }
        return freqs;
    }

    // Alternative: freq_plan=F0,F1,F2,...
    if (addr.has_key("freq_plan")) {
        std::stringstream ss(addr["freq_plan"]);
        std::string token;
        while (std::getline(ss, token, ',')) {
            if (!token.empty()) {
                freqs.push_back(std::stod(token));
            }
        }
    }

    return freqs;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // -------------------------------------------------------------------------
    // Command-line options
    // -------------------------------------------------------------------------
    std::string args, subdev;
    double rate, freq, gain, delay, plan_center, plan_overlap;
    size_t nsamps;

    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message.")
        ("args,a",
            po::value<std::string>(&args)->default_value(
                "serial0=30B56D6,serial1=30DBC3C"),
            "UHD device address arguments.\n"
            "Must include serialN= keys for each device.\n"
            "Optional sync keys:\n"
            "  sync_clock_source  — internal, external (default), gpsdo\n"
            "  sync_time_source   — external (default), internal, gpsdo\n"
            "Optional frequency-plan keys (Phase 5):\n"
            "  freq0,freq1,...    — per-device center frequencies in Hz\n"
            "  freq_plan=F0,F1... — comma-separated per-device frequencies\n"
            "Example (10 MHz + PPS):\n"
            "  --args \"sync_clock_source=external,"
            "sync_time_source=external,serial0=ABC,serial1=DEF\"")
        ("rate,r",
            po::value<double>(&rate)->default_value(10e6),
            "RX sample rate (Sps).")
        ("freq,f",
            po::value<double>(&freq)->default_value(100e6),
            "RX centre frequency (Hz).")
        ("plan-center-freq", po::value<double>(&plan_center),
            "Optional aggregate center frequency for auto frequency-plan generation (Hz).")
        ("plan-overlap", po::value<double>(&plan_overlap)->default_value(0.10),
            "Overlap fraction for auto frequency-plan generation [0,1).")
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

    // -------------------------------------------------------------------------
    // Parse serial list
    // -------------------------------------------------------------------------
    const std::vector<std::string> serials = parse_serial_list(dev_addr);
    std::vector<double> freq_plan = parse_freq_plan(dev_addr);
    if (serials.empty()) {
        std::cerr << "[bonded_usrp_rx] ERROR: no serialN= keys found in --args.\n";
        return EXIT_FAILURE;
    }
    if (!freq_plan.empty() && freq_plan.size() != serials.size()) {
        std::cerr << "[bonded_usrp_rx] ERROR: freq plan size ("
                  << freq_plan.size() << ") does not match serial count ("
                  << serials.size() << ").\n";
        return EXIT_FAILURE;
    }
    if (freq_plan.empty() && vm.count("plan-center-freq")) {
        auto plan = uhd::usrp::bonded::make_adjacent_frequency_plan(
            serials.size(), plan_center, rate, plan_overlap);
        freq_plan = plan.centers_hz;
        std::cout << boost::format(
                         "[bonded_usrp_rx] Auto freq plan: center=%.3f MHz, step=%.3f MHz, overlap=%.1f%%\n")
                         % (plan_center / 1e6) % (plan.step_hz / 1e6)
                         % (plan_overlap * 100.0);
        for (size_t i = 0; i < freq_plan.size(); i++) {
            std::cout << boost::format("  f%u = %.3f MHz\n") % i % (freq_plan[i] / 1e6);
        }
    }
    std::cout << boost::format(
                     "\n[bonded_usrp_rx] Found %zu device serial(s): ") % serials.size();
    for (const auto& s : serials) {
        std::cout << s << " ";
    }
    std::cout << "\n";

    // -------------------------------------------------------------------------
    // Configure bonded_receiver
    // -------------------------------------------------------------------------
    uhd::usrp::bonded::bonded_receiver::config cfg;
    cfg.serials      = serials;
    cfg.clock_source = dev_addr.cast<std::string>("sync_clock_source", "external");
    cfg.time_source  = dev_addr.cast<std::string>("sync_time_source", "external");
    cfg.rate         = rate;
    cfg.freq         = freq;
    cfg.freq_plan    = freq_plan;
    cfg.gain         = gain;
    cfg.strict       = dev_addr.has_key("sync_strict");
    cfg.lock_timeout = std::stod(
        dev_addr.cast<std::string>("sync_lock_timeout", "5.0"));
    if (vm.count("subdev")) {
        cfg.subdev = subdev;
    }

    uhd::usrp::bonded::bonded_receiver rx(cfg);
    rx.configure();

    // -------------------------------------------------------------------------
    // Burst capture
    // -------------------------------------------------------------------------
    auto result = rx.capture_burst(nsamps, delay);

    // -------------------------------------------------------------------------
    // Validation output (same format as original for comparison)
    // -------------------------------------------------------------------------
    const size_t num_dev = rx.num_devices();

    for (size_t d = 0; d < num_dev; d++) {
        const size_t got = result.success[d] ? result.data[d][0].size() : 0;
        std::cout << boost::format(
                         "[bonded_usrp_rx] Device %u: received %zu / %zu samps\n")
                         % d % got % nsamps;
    }

    std::cout << "\n[bonded_usrp_rx] --- Synchronisation validation ---\n";

    for (size_t d = 0; d < num_dev; d++) {
        if (result.success[d]) {
            std::cout << boost::format("  Device %u first-packet ts : %.9f s\n")
                             % d % result.first_timestamps[d];
        }
    }

    if (!result.error_message.empty()) {
        std::cout << "  FAIL — " << result.error_message << "\n";
    } else if (num_dev == 1) {
        std::cout << "  Single-device burst OK.\n";
    } else {
        const bool spread_ok = result.aligned;
        std::cout << boost::format(
                         "  Inter-device spread : %.3f µs  [%s]  (threshold 1000 µs)\n")
                         % result.inter_device_spread_us
                         % (spread_ok ? "OK" : "FAIL");
    }

    std::cout << "\n  Result: " << (result.aligned ? "PASS" : "FAIL") << "\n";
    return result.aligned ? EXIT_SUCCESS : EXIT_FAILURE;
}
