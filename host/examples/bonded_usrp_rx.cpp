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

#include <uhd/types/device_addr.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <cstdlib>
#include <iostream>
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

int UHD_SAFE_MAIN(int argc, char* argv[])
{
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
                "serial0=30B56D6,serial1=30DBC3C"),
            "UHD device address arguments.\n"
            "Must include serialN= keys for each device.\n"
            "Optional sync keys:\n"
            "  sync_clock_source  — internal, external (default), gpsdo\n"
            "  sync_time_source   — external (default), internal, gpsdo\n"
            "Example (10 MHz + PPS):\n"
            "  --args \"sync_clock_source=external,"
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

    // -------------------------------------------------------------------------
    // Parse serial list
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

    // -------------------------------------------------------------------------
    // Configure bonded_receiver
    // -------------------------------------------------------------------------
    uhd::usrp::bonded::bonded_receiver::config cfg;
    cfg.serials      = serials;
    cfg.clock_source = dev_addr.cast<std::string>("sync_clock_source", "external");
    cfg.time_source  = dev_addr.cast<std::string>("sync_time_source", "external");
    cfg.rate         = rate;
    cfg.freq         = freq;
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
