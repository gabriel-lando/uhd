// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_spectrum_monitor -- Real-time wideband spectrum monitor for N bonded USRPs.
//
// Usage:
//   bonded_spectrum_monitor --serials A,B --freq 2400e6 --rate 20e6 --bins 80
//   bonded_spectrum_monitor --serial0 A --serial1 B --serial2 C --freq 2400e6
//

#include <uhd/exception.hpp>
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <algorithm>
#include <chrono>
#include <complex>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

namespace po = boost::program_options;

static volatile bool stop_signal = false;
static void sig_int_handler(int) { stop_signal = true; }

// ASCII-art power bar
static std::string power_bar(float power_db, float min_db = -100.0f, float max_db = 0.0f)
{
    const int width = 40;
    int filled = static_cast<int>(
        (std::clamp(power_db, min_db, max_db) - min_db) / (max_db - min_db) * width);
    return std::string(filled, '#') + std::string(width - filled, ' ');
}

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
        ("freq",     po::value<double>()->default_value(2.4e9),  "Center frequency (Hz)")
        ("rate",     po::value<double>()->default_value(20e6),   "Per-device sample rate (S/s)")
        ("gain",     po::value<double>()->default_value(40.0),   "RX gain (dB)")
        ("bins",     po::value<size_t>()->default_value(80),     "Number of display bins (averaged)")
        ("fft-size", po::value<size_t>()->default_value(4096),   "FFT size")
        ("avg",      po::value<size_t>()->default_value(10),     "Spectra to average before display")
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

    const double center_freq   = vm["freq"].as<double>();
    const double sample_rate   = vm["rate"].as<double>();
    const double gain          = vm["gain"].as<double>();
    const size_t display_bins  = vm["bins"].as<size_t>();
    const size_t fft_size      = vm["fft-size"].as<size_t>();
    const size_t avg_count     = vm["avg"].as<size_t>();

    // Open and synchronize devices
    std::cout << "Opening and synchronizing " << serials.size()
              << " USRPs..." << std::endl;
    auto devices = uhd::usrp::multi_usrp::make_bonded_usb(serials);
    std::cout << "PPS sync complete." << std::endl;

    for (auto& dev : devices) {
        dev->set_rx_gain(gain, 0);
    }

    std::signal(SIGINT, &sig_int_handler);

    uhd::usrp::bonded::bonded_rx_streamer bonded(
        devices, sample_rate, center_freq, fft_size);
    const size_t nbins = bonded.output_size();

    std::vector<std::complex<float>> spectrum(nbins);
    std::vector<float> power_avg(nbins, 0.0f);
    uhd::rx_metadata_t md;

    bonded.start();

    const double combined_bw = sample_rate * static_cast<double>(devices.size());
    std::cout << "\nWideband Spectrum Monitor — "
              << (center_freq / 1e6) << " MHz center, "
              << (combined_bw / 1e6) << " MHz combined bandwidth ("
              << devices.size() << " devices)\n"
              << "Press Ctrl+C to quit.\n"
              << std::endl;

    size_t avg_idx = 0;
    while (!stop_signal) {
        size_t n = bonded.recv(spectrum.data(), spectrum.size(), md, 1.0);
        if (n == 0) continue;

        // Accumulate power (magnitude squared)
        for (size_t i = 0; i < n; ++i) {
            float mag2 = spectrum[i].real() * spectrum[i].real()
                       + spectrum[i].imag() * spectrum[i].imag();
            power_avg[i] += mag2;
        }
        ++avg_idx;

        if (avg_idx < avg_count) continue;

        // Average and convert to dB
        std::vector<float> power_db(n);
        for (size_t i = 0; i < n; ++i) {
            float avg   = power_avg[i] / static_cast<float>(avg_count);
            power_db[i] = (avg > 0.0f) ? (10.0f * std::log10(avg)) : -200.0f;
        }

        const size_t stride = std::max<size_t>(1, n / display_bins);
        std::fill(power_avg.begin(), power_avg.end(), 0.0f);
        avg_idx = 0;

        // Move cursor to top of display area
        std::cout << "\033[" << (display_bins + 3) << "A";

        const double bw_per_bin = combined_bw / static_cast<double>(n);
        const double start_freq = center_freq - combined_bw / 2.0;

        std::cout << std::string(70, '-') << "\n";
        std::cout << "Center: " << std::fixed << std::setprecision(3)
                  << (center_freq / 1e6) << " MHz  |  "
                  << bonded.quality().to_string() << "\n";
        std::cout << std::string(70, '-') << "\n";

        for (size_t b = 0; b < display_bins; ++b) {
            size_t bin_start = b * stride;
            size_t bin_end   = std::min(bin_start + stride, n);
            if (bin_start >= n) break;
            float avg_db = 0.0f;
            for (size_t i = bin_start; i < bin_end; ++i) {
                avg_db += power_db[i];
            }
            avg_db /= static_cast<float>(bin_end - bin_start);

            double freq_mhz = (start_freq + (b + 0.5) * stride * bw_per_bin) / 1e6;
            std::cout << std::fixed << std::setw(8) << std::setprecision(1)
                      << freq_mhz << " MHz |"
                      << power_bar(avg_db) << "| "
                      << std::setprecision(1) << avg_db << " dB\n";
        }
    }

    bonded.stop();
    std::cout << "\nStopped." << std::endl;
    return EXIT_SUCCESS;
}
