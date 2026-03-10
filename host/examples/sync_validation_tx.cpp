//
// Copyright 2026 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <complex>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

namespace po = boost::program_options;

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args, clock_source, time_source, waveform;
    double freq, rate, gain, duration;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Help message")
        ("args", po::value<std::string>(&args)->default_value("serial0=30B56D6,serial1=30DBC3C"), "USRP device arguments")
        ("clock-source", po::value<std::string>(&clock_source)->default_value("external"), "Clock source")
        ("time-source", po::value<std::string>(&time_source)->default_value("external"), "Time source")
        ("freq", po::value<double>(&freq)->default_value(915e6), "Center frequency in Hz")
        ("rate", po::value<double>(&rate)->default_value(10e6), "Sample rate in Hz")
        ("gain", po::value<double>(&gain)->default_value(30.0), "Gain in dB")
        ("waveform", po::value<std::string>(&waveform)->default_value("tone"), "Waveform (tone)")
        ("duration", po::value<double>(&duration)->default_value(1.0), "Duration of capture in seconds");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << "Sync Validation TX\n" << desc << std::endl;
        return ~0;
    }
    po::notify(vm);

    std::cout << "Creating multi_usrp with args: " << args << std::endl;
    auto usrp = uhd::usrp::multi_usrp::make(args);
    const size_t num_mboards = usrp->get_num_mboards();

    for (size_t m = 0; m < num_mboards; m++) {
        usrp->set_clock_source(clock_source, m);
        usrp->set_time_source(time_source, m);
    }

    if (time_source == "external" || time_source == "gpsdo") {
        usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }

    size_t num_channels = usrp->get_tx_num_channels();
    for (size_t ch = 0; ch < num_channels; ch++) {
        usrp->set_tx_rate(rate, ch);
        usrp->set_tx_freq(freq, ch);
        usrp->set_tx_gain(gain, ch);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    uhd::stream_args_t stream_args("fc32", "sc16");
    std::vector<size_t> channels(num_channels);
    std::iota(channels.begin(), channels.end(), 0);
    stream_args.channels = channels;
    auto tx_stream = usrp->get_tx_stream(stream_args);

    size_t total_samples = static_cast<size_t>(rate * duration);
    std::vector<std::vector<std::complex<float>>> buffs(num_channels, std::vector<std::complex<float>>(tx_stream->get_max_num_samps()));
    
    // Generate tone
    for (size_t i = 0; i < buffs[0].size(); i++) {
        float sample = 0.5f * std::sin(2.0f * M_PI * i * 0.05f); // 5% of nyquist frequency
        for (size_t ch = 0; ch < num_channels; ch++) {
            buffs[ch][i] = std::complex<float>(sample, sample);
        }
    }

    std::vector<const std::complex<float>*> buff_ptrs(num_channels);
    for (size_t ch = 0; ch < num_channels; ch++) {
        buff_ptrs[ch] = buffs[ch].data();
    }

    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = usrp->get_time_now() + uhd::time_spec_t(1.0);

    size_t num_tx_samps = 0;
    while (num_tx_samps < total_samples) {
        size_t samps_to_send = std::min(total_samples - num_tx_samps, buffs[0].size());
        if (num_tx_samps + samps_to_send >= total_samples) md.end_of_burst = true;
        
        size_t t = tx_stream->send(buff_ptrs, samps_to_send, md);
        num_tx_samps += t;
        md.start_of_burst = false;
        md.has_time_spec = false;
    }

    std::cout << "Transmitted " << num_tx_samps << " samples." << std::endl;
    return EXIT_SUCCESS;
}
