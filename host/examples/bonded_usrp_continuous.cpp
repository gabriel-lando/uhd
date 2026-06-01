//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_usrp_continuous -- continuous bonded multi-device synchronized RX.
//
// Demonstrates bonded_receiver continuous mode:
//   configure() -> start_continuous() -> get_aligned_samples() -> stop()
//

#include "../lib/usrp/bonded/bonded_receiver.hpp"
#include "../lib/usrp/bonded/frequency_plan.hpp"

#include <uhd/types/device_addr.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace po = boost::program_options;

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
    std::string args, subdev;
    double rate, freq, gain, duration, timeout, plan_center, plan_overlap;
    size_t chunk_size, progress_interval;

    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message.")
        ("args,a",
            po::value<std::string>(&args)->default_value(
                "serial0=30B56D6,serial1=30DBC3C,sync_clock_source=external,sync_time_source=external"),
            "UHD device address arguments with serialN and sync keys.\n"
            "Optional frequency-plan keys (Phase 5): freq0,freq1,... or freq_plan=F0,F1,... (Hz).")
        ("rate,r", po::value<double>(&rate)->default_value(10e6),
            "RX sample rate (Sps).")
        ("freq,f", po::value<double>(&freq)->default_value(100e6),
            "RX center frequency (Hz).")
        ("plan-center-freq", po::value<double>(&plan_center),
            "Optional aggregate center frequency for auto frequency-plan generation (Hz).")
        ("plan-overlap", po::value<double>(&plan_overlap)->default_value(0.10),
            "Overlap fraction for auto frequency-plan generation [0,1).")
        ("gain,g", po::value<double>(&gain)->default_value(40.0),
            "RX gain (dB).")
        ("duration", po::value<double>(&duration)->default_value(10.0),
            "Continuous capture duration (seconds).")
        ("chunk-size", po::value<size_t>(&chunk_size)->default_value(65536),
            "Samples per get_aligned_samples() call.")
        ("progress-interval", po::value<size_t>(&progress_interval)->default_value(100),
            "Print routine aligned progress every N iterations (0 disables routine progress).")
        ("timeout", po::value<double>(&timeout)->default_value(1.0),
            "Timeout per get_aligned_samples() call (seconds).")
        ("subdev", po::value<std::string>(&subdev),
            "RX subdevice spec, applied uniformly to all boards.")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout << "bonded_usrp_continuous -- bonded multi-device continuous RX\n\n"
                  << desc << std::endl;
        return EXIT_SUCCESS;
    }
    po::notify(vm);

    const uhd::device_addr_t dev_addr(args);
    const std::vector<std::string> serials = parse_serial_list(dev_addr);
    std::vector<double> freq_plan = parse_freq_plan(dev_addr);
    if (serials.empty()) {
        std::cerr << "[bonded_usrp_continuous] ERROR: no serialN= keys found in --args.\n";
        return EXIT_FAILURE;
    }
    if (!freq_plan.empty() && freq_plan.size() != serials.size()) {
        std::cerr << "[bonded_usrp_continuous] ERROR: freq plan size ("
                  << freq_plan.size() << ") does not match serial count ("
                  << serials.size() << ").\n";
        return EXIT_FAILURE;
    }

    if (freq_plan.empty() && vm.count("plan-center-freq")) {
        auto plan = uhd::usrp::bonded::make_adjacent_frequency_plan(
            serials.size(), plan_center, rate, plan_overlap);
        freq_plan = plan.centers_hz;
        std::cout << boost::format(
                         "[bonded_usrp_continuous] Auto freq plan: center=%.3f MHz, step=%.3f MHz, overlap=%.1f%%\n")
                         % (plan_center / 1e6) % (plan.step_hz / 1e6)
                         % (plan_overlap * 100.0);
        for (size_t i = 0; i < freq_plan.size(); i++) {
            std::cout << boost::format("  f%u = %.3f MHz\n") % i % (freq_plan[i] / 1e6);
        }
    }

    std::cout << boost::format(
                     "\n[bonded_usrp_continuous] Found %zu device serial(s): ")
                     % serials.size();
    for (const auto& s : serials) {
        std::cout << s << " ";
    }
    std::cout << "\n";

    uhd::usrp::bonded::bonded_receiver::config cfg;
    cfg.serials      = serials;
    cfg.clock_source = dev_addr.cast<std::string>("sync_clock_source", "external");
    cfg.time_source  = dev_addr.cast<std::string>("sync_time_source", "external");
    cfg.rate         = rate;
    cfg.freq         = freq;
    cfg.freq_plan    = freq_plan;
    cfg.gain         = gain;
    cfg.strict       = dev_addr.has_key("sync_strict");
    cfg.lock_timeout = std::stod(dev_addr.cast<std::string>("sync_lock_timeout", "5.0"));
    if (vm.count("subdev")) {
        cfg.subdev = subdev;
    }

    uhd::usrp::bonded::bonded_receiver rx(cfg);
    rx.configure();
    rx.start_continuous();

    const auto end_time =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);

    size_t iter = 0;
    size_t fail_count = 0;
    size_t degraded_count = 0;
    while (std::chrono::steady_clock::now() < end_time) {
        auto result = rx.get_aligned_samples(chunk_size, timeout);
        if (!result.aligned) {
            fail_count++;
            std::cerr << boost::format(
                             "[bonded_usrp_continuous] Iter %zu: alignment FAIL: %s\n")
                             % iter
                             % (result.error_message.empty() ? "unknown error"
                                                             : result.error_message);
        } else if (!result.error_message.empty()) {
            degraded_count++;
            std::cout << boost::format(
                             "[bonded_usrp_continuous] Iter %zu: aligned, spread=%.3f us (degraded: %s)\n")
                             % iter % result.inter_device_spread_us % result.error_message;
        } else if (progress_interval != 0 && (iter % progress_interval) == 0) {
            std::cout << boost::format(
                             "[bonded_usrp_continuous] Iter %zu: aligned, spread=%.3f us\n")
                             % iter % result.inter_device_spread_us;
        }
        iter++;
    }

    rx.stop();
    const auto overflow_counts = rx.get_overflow_counts();
    const auto zero_fill_counts = rx.get_zero_fill_counts();

    std::cout << "\n[bonded_usrp_continuous] Summary\n"
              << "  Iterations: " << iter << "\n"
              << "  Degraded iterations: " << degraded_count << "\n"
              << "  Alignment failures: " << fail_count << "\n"
              << "  Result: " << (fail_count == 0 ? "PASS" : "FAIL") << "\n";

    for (size_t d = 0; d < overflow_counts.size(); d++) {
        std::cout << boost::format(
                         "  Device %u overflow events: %zu, zero-fill events: %zu\n")
                         % d % overflow_counts[d] % zero_fill_counts[d];
    }

    return fail_count == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
