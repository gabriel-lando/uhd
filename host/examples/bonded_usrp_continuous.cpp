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

#include <uhd/types/device_addr.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
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

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args, subdev;
    double rate, freq, gain, duration, timeout;
    size_t chunk_size;

    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message.")
        ("args,a",
            po::value<std::string>(&args)->default_value(
                "serial0=30B56D6,serial1=30DBC3C,sync_clock_source=external,sync_time_source=external"),
            "UHD device address arguments with serialN and sync keys.")
        ("rate,r", po::value<double>(&rate)->default_value(10e6),
            "RX sample rate (Sps).")
        ("freq,f", po::value<double>(&freq)->default_value(100e6),
            "RX center frequency (Hz).")
        ("gain,g", po::value<double>(&gain)->default_value(40.0),
            "RX gain (dB).")
        ("duration", po::value<double>(&duration)->default_value(10.0),
            "Continuous capture duration (seconds).")
        ("chunk-size", po::value<size_t>(&chunk_size)->default_value(65536),
            "Samples per get_aligned_samples() call.")
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
    if (serials.empty()) {
        std::cerr << "[bonded_usrp_continuous] ERROR: no serialN= keys found in --args.\n";
        return EXIT_FAILURE;
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
        } else {
            std::cout << boost::format(
                             "[bonded_usrp_continuous] Iter %zu: aligned, spread=%.3f us\n")
                             % iter % result.inter_device_spread_us;
        }
        iter++;
    }

    rx.stop();

    std::cout << "\n[bonded_usrp_continuous] Summary\n"
              << "  Iterations: " << iter << "\n"
              << "  Degraded iterations: " << degraded_count << "\n"
              << "  Alignment failures: " << fail_count << "\n"
              << "  Result: " << (fail_count == 0 ? "PASS" : "FAIL") << "\n";

    return fail_count == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
