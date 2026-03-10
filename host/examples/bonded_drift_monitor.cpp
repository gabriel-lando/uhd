// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_drift_monitor -- PPS-based oscillator drift monitor for N bonded USRPs.
//
// Continuously displays estimated frequency offset (ppm) between all devices
// relative to device 0, using GPS PPS timestamps.  No RF signal required.
//
// Usage:
//   bonded_drift_monitor --serials A,B
//   bonded_drift_monitor --serial0 A --serial1 B --serial2 C
//

#include <uhd/exception.hpp>
#include <uhd/usrp/bonded/pps_drift_estimator.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace po = boost::program_options;

static volatile bool stop_signal = false;
static void sig_int_handler(int) { stop_signal = true; }

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help",                                "show this help message")
        ("serials",  po::value<std::string>()->default_value(""), "Comma-separated serial numbers (e.g. 'A,B,C')")
        ("serial0",  po::value<std::string>()->default_value(""), "Serial number of device 0")
        ("serial1",  po::value<std::string>()->default_value(""), "Serial number of device 1")
        ("serial2",  po::value<std::string>()->default_value(""), "Serial number of device 2 (optional)")
        ("serial3",  po::value<std::string>()->default_value(""), "Serial number of device 3 (optional)")
        ("rf-freq",  po::value<double>()->default_value(2.4e9),   "RF frequency for phase-rate calculation (Hz)")
        ("duration", po::value<double>()->default_value(0.0),     "How long to monitor (0 = run until Ctrl+C)")
        ("csv",      po::value<std::string>()->default_value(""), "Optional CSV output file path")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    // Build serials vector
    std::vector<std::string> serials;
    const std::string serials_str = vm["serials"].as<std::string>();
    if (!serials_str.empty()) {
        boost::split(serials, serials_str, boost::is_any_of(","));
        for (auto& s : serials) boost::trim(s);
        serials.erase(std::remove_if(serials.begin(), serials.end(),
            [](const std::string& s) { return s.empty(); }), serials.end());
    } else {
        for (const auto* key : {"serial0", "serial1", "serial2", "serial3"}) {
            const std::string s = vm[key].as<std::string>();
            if (!s.empty()) serials.push_back(s);
        }
    }
    if (serials.size() < 2) {
        std::cerr << "ERROR: Provide at least 2 serial numbers via --serials or "
                     "--serial0/--serial1.\n";
        return EXIT_FAILURE;
    }

    const double rf_freq   = vm["rf-freq"].as<double>();
    const double duration  = vm["duration"].as<double>();
    const std::string csv_path = vm["csv"].as<std::string>();

    // Open and synchronize devices
    std::cout << "Opening and synchronizing " << serials.size()
              << " USRPs..." << std::endl;
    auto devices = uhd::usrp::multi_usrp::make_bonded_usb(serials);
    std::cout << "PPS sync complete." << std::endl;

    std::signal(SIGINT, &sig_int_handler);

    // Open CSV output if requested
    std::ofstream csv_file;
    if (!csv_path.empty()) {
        csv_file.open(csv_path);
        if (!csv_file.is_open()) {
            std::cerr << "ERROR: Cannot open CSV file: " << csv_path << std::endl;
            return EXIT_FAILURE;
        }
        // Write CSV header
        csv_file << "wall_time_s,pps_t0";
        for (size_t i = 1; i < devices.size(); ++i) {
            csv_file << ",pps_t" << i
                     << ",drift" << i << "_ppm"
                     << ",uncert" << i << "_ppm"
                     << ",phase_rate" << i << "_rad_s";
        }
        csv_file << ",pps_samples\n";
    }

    uhd::usrp::bonded::pps_drift_estimator estimator(devices.size());

    const size_t N = devices.size();
    std::cout << "\nMonitoring PPS drift across " << N
              << " devices. Press Ctrl+C to stop.\n" << std::endl;

    // Print header row
    std::cout << std::setw(10) << "Time(s)";
    for (size_t i = 1; i < N; ++i) {
        std::cout << std::setw(14) << ("Dev" + std::to_string(i) + " drift(ppm)")
                  << std::setw(13) << ("uncert(ppm)");
    }
    std::cout << std::setw(8) << "PPS#" << std::endl;
    std::cout << std::string(10 + (N - 1) * 27 + 8, '-') << std::endl;

    const auto program_start = std::chrono::steady_clock::now();
    double last_t0 = 0.0;

    while (!stop_signal) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        const double t0 = devices[0]->get_time_last_pps(0).get_real_secs();
        if (t0 == last_t0) continue;
        last_t0 = t0;

        const auto all_results = estimator.update(devices, rf_freq);
        if (all_results.empty()) continue;

        const double wall_secs =
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - program_start)
                .count();

        if (all_results[0].valid) {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(10) << wall_secs;
            for (const auto& r : all_results) {
                std::cout << std::setprecision(4)
                          << std::setw(14) << r.drift_ppm
                          << std::setprecision(5)
                          << std::setw(13) << r.uncertainty_ppm;
            }
            std::cout << std::setw(8) << all_results[0].num_samples
                      << std::endl;
        } else {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(10) << wall_secs
                      << "  (calibrating: " << all_results[0].num_samples
                      << " PPS edges so far)" << std::endl;
        }

        if (csv_file.is_open() && all_results[0].valid) {
            csv_file << std::fixed << std::setprecision(9)
                     << wall_secs << "," << t0;
            for (size_t i = 0; i < all_results.size(); ++i) {
                const double ti = devices[i + 1]->get_time_last_pps(0).get_real_secs();
                csv_file << "," << ti
                         << "," << all_results[i].drift_ppm
                         << "," << all_results[i].uncertainty_ppm
                         << "," << all_results[i].phase_rate_rad_s;
            }
            csv_file << "," << all_results[0].num_samples << "\n";
            csv_file.flush();
        }

        if (duration > 0.0 && wall_secs >= duration) break;
    }

    if (csv_file.is_open()) {
        csv_file.close();
        std::cout << "\nCSV written to " << csv_path << std::endl;
    }

    const auto final_all = estimator.latest_all();
    std::cout << "\nFinal drift estimates:" << std::endl;
    for (size_t i = 0; i < final_all.size(); ++i) {
        const auto& r = final_all[i];
        std::cout << "  Dev" << (i + 1) << ": " << r.drift_ppm << " ppm"
                  << " (±" << r.uncertainty_ppm << " ppm)"
                  << " from " << r.num_samples << " PPS edges" << std::endl;
    }

    return EXIT_SUCCESS;
}

