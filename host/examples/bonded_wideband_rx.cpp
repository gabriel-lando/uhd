// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_wideband_rx -- Wideband RX example using N PPS-only bonded USRPs.
//
// Hardware:  N x USRP B210 with GPS PPS shared to all PPS SMA inputs.
//
// Usage (2 devices):
//   bonded_wideband_rx --serials A,B --freq 2400e6 --rate 20e6
// Usage (3 devices):
//   bonded_wideband_rx --serials A,B,C --freq 2400e6 --rate 20e6
// Usage (individual options):
//   bonded_wideband_rx --serial0 A --serial1 B --freq 2400e6 --rate 20e6
//

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
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
        ("freq",     po::value<double>()->default_value(2.4e9),   "Center frequency of the combined band (Hz)")
        ("rate",     po::value<double>()->default_value(20e6),    "Per-device sample rate (S/s)")
        ("gain",     po::value<double>()->default_value(40.0),    "RX gain (dB)")
        ("duration", po::value<double>()->default_value(10.0),    "Capture duration (seconds)")
        ("outfile",  po::value<std::string>()->default_value(""), "Output file for stitched spectrum (binary fc32)")
        ("fft-size", po::value<size_t>()->default_value(4096),    "FFT size for spectrum stitching")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    // Build serials vector from --serials or individual --serialN options
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

    const double center_freq  = vm["freq"].as<double>();
    const double sample_rate  = vm["rate"].as<double>();
    const double gain         = vm["gain"].as<double>();
    const double duration     = vm["duration"].as<double>();
    const std::string outfile = vm["outfile"].as<std::string>();
    const size_t fft_size     = vm["fft-size"].as<size_t>();

    // Open and synchronize devices via make_bonded_usb()
    std::cout << "Opening and synchronizing " << serials.size()
              << " USRPs..." << std::endl;
    auto devices = uhd::usrp::multi_usrp::make_bonded_usb(serials);
    std::cout << "PPS sync complete." << std::endl;

    // Set gain on all devices
    for (auto& dev : devices) {
        dev->set_rx_gain(gain, 0);
    }

    // Create bonded streamer (handles N-device tuning and stitching)
    uhd::usrp::bonded::bonded_rx_streamer bonded(
        devices, sample_rate, center_freq, fft_size);
    const size_t output_bins = bonded.output_size();

    std::cout << "\nBonded RX configuration:" << std::endl;
    std::cout << "  Devices:         " << devices.size() << std::endl;
    std::cout << "  Combined center: " << center_freq / 1e6 << " MHz" << std::endl;
    std::cout << "  Per-device rate: " << sample_rate / 1e6 << " MS/s" << std::endl;
    std::cout << "  Combined BW:     "
              << sample_rate * static_cast<double>(devices.size()) / 1e6
              << " MHz" << std::endl;
    std::cout << "  Output bins:     " << output_bins << std::endl;
    std::cout << "\nCapturing for " << duration << " seconds...\n"
              << "Press Ctrl+C to stop early." << std::endl;

    std::signal(SIGINT, &sig_int_handler);
    bonded.start();

    std::ofstream ofs;
    if (!outfile.empty()) {
        ofs.open(outfile, std::ios::binary);
        if (!ofs.is_open()) {
            std::cerr << "ERROR: Cannot open output file: " << outfile << std::endl;
            bonded.stop();
            return EXIT_FAILURE;
        }
    }

    std::vector<std::complex<float>> buffer(output_bins);
    uhd::rx_metadata_t md;
    const auto start_time = std::chrono::steady_clock::now();
    size_t total_spectra  = 0;
    size_t error_count    = 0;

    while (!stop_signal) {
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - start_time)
                           .count();
        if (elapsed >= duration) break;

        size_t n = bonded.recv(buffer.data(), buffer.size(), md, 1.0);
        if (n == 0) {
            ++error_count;
            continue;
        }
        ++total_spectra;

        if (ofs.is_open()) {
            ofs.write(reinterpret_cast<const char*>(buffer.data()),
                static_cast<std::streamsize>(n * sizeof(std::complex<float>)));
        }

        if (total_spectra % 1000 == 0) {
            std::cout << bonded.quality().to_string() << std::endl;
        }
    }

    bonded.stop();
    if (ofs.is_open()) ofs.close();

    std::cout << "\nCapture complete." << std::endl;
    std::cout << "  Total spectra:  " << total_spectra << std::endl;
    std::cout << "  Receive errors: " << error_count << std::endl;
    std::cout << "  Final quality:  " << bonded.quality().to_string() << std::endl;

    return EXIT_SUCCESS;
}

