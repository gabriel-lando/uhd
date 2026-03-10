//
// Copyright 2026 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <chrono>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <thread>
#include <vector>

namespace po = boost::program_options;
namespace pt = boost::property_tree;

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args, clock_source, time_source, output_dir, format;
    double freq, rate, gain, duration;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Help message")
        ("args", po::value<std::string>(&args)->default_value("serial0=30B56D6,serial1=30DBC3C"), "USRP device arguments")
        ("clock-source", po::value<std::string>(&clock_source)->default_value("external"), "Clock source (internal, external, gpsdo)")
        ("time-source", po::value<std::string>(&time_source)->default_value("external"), "Time source (internal, external, gpsdo)")
        ("freq", po::value<double>(&freq)->default_value(915e6), "Center frequency in Hz")
        ("rate", po::value<double>(&rate)->default_value(10e6), "Sample rate in Hz")
        ("gain", po::value<double>(&gain)->default_value(30.0), "Gain in dB")
        ("duration", po::value<double>(&duration)->default_value(1.0), "Duration of capture in seconds")
        ("output-dir", po::value<std::string>(&output_dir)->default_value("./sync_capture"), "Output directory")
        ("format", po::value<std::string>(&format)->default_value("sc16"), "Sample format (e.g., sc16)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << "Sync Validation Capture\n" << desc << std::endl;
        return ~0;
    }
    po::notify(vm);

    std::cout << "Creating multi_usrp with args: " << args << std::endl;
    auto usrp = uhd::usrp::multi_usrp::make(args);
    const size_t num_mboards = usrp->get_num_mboards();

    std::cout << "Configuring " << num_mboards << " motherboards..." << std::endl;
    for (size_t m = 0; m < num_mboards; m++) {
        usrp->set_clock_source(clock_source, m);
        usrp->set_time_source(time_source, m);
    }

    if (clock_source == "external" || clock_source == "gpsdo") {
        std::cout << "Waiting for reference lock..." << std::endl;
        for (size_t m = 0; m < num_mboards; m++) {
            bool locked = false;
            for (int i = 0; i < 30; i++) {
                if (usrp->get_mboard_sensor("ref_locked", m).to_bool()) {
                    locked = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!locked) {
                std::cerr << "Warning: Reference lock timeout on mboard " << m << std::endl;
            }
        }
    }

    size_t num_channels = usrp->get_rx_num_channels();
    std::cout << "Configuring " << num_channels << " RX channels..." << std::endl;
    for (size_t ch = 0; ch < num_channels; ch++) {
        usrp->set_rx_rate(rate, ch);
        usrp->set_rx_freq(freq, ch);
        usrp->set_rx_gain(gain, ch);
    }

    std::cout << "Synchronizing time... (" << time_source << ")" << std::endl;
    if (time_source == "external" || time_source == "gpsdo") {
        usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1)); // Settling time

    uhd::stream_args_t stream_args(format, format);
    std::vector<size_t> channels(num_channels);
    std::iota(channels.begin(), channels.end(), 0);
    stream_args.channels = channels;
    auto rx_stream = usrp->get_rx_stream(stream_args);

    size_t total_samples = static_cast<size_t>(rate * duration);
    size_t spb = rx_stream->get_max_num_samps();
    
    std::cout << "Starting capture of " << total_samples << " samples..." << std::endl;
    uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    cmd.num_samps  = total_samples;
    cmd.stream_now = false;
    
    // Calculate a start time in the near future (e.g. aligned to next full second)
    uhd::time_spec_t current_time = usrp->get_time_now();
    uhd::time_spec_t start_time = current_time + uhd::time_spec_t(1.0);
    
    // For rigorous PPS synchronization, snap to the next integer boundary
    if (time_source == "external" || time_source == "gpsdo") {
        start_time = usrp->get_time_last_pps() + uhd::time_spec_t(2.0);
        if (start_time < current_time + uhd::time_spec_t(0.1)) {
            start_time += uhd::time_spec_t(1.0); // Ensure it's comfortably in the future
        }
    }
    
    cmd.time_spec = start_time;
    rx_stream->issue_stream_cmd(cmd);

    // Provide buffers
    std::vector<std::vector<std::complex<int16_t>>> buffs(num_channels, std::vector<std::complex<int16_t>>(total_samples));
    std::vector<std::complex<int16_t>*> buff_ptrs(num_channels);
    for (size_t ch = 0; ch < num_channels; ch++) {
        buff_ptrs[ch] = buffs[ch].data();
    }

    uhd::rx_metadata_t md;
    size_t num_rx_samps = 0;
    while (num_rx_samps < total_samples) {
        size_t samps_to_recv = std::min(total_samples - num_rx_samps, spb);
        std::vector<std::complex<int16_t>*> recv_ptrs(num_channels);
        for (size_t ch = 0; ch < num_channels; ch++) {
            recv_ptrs[ch] = buffs[ch].data() + num_rx_samps;
        }

        size_t r = rx_stream->recv(recv_ptrs, samps_to_recv, md, 3.0);
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
            std::cerr << "RX Error: " << md.strerror() << std::endl;
        }
        if (r == 0) break;
        num_rx_samps += r;
    }

    std::cout << "Received " << num_rx_samps << " samples." << std::endl;

    // Output directory
    std::string cmd_mkdir = "mkdir -p " + output_dir;
    int unused = system(cmd_mkdir.c_str());
    (void)unused;

    // Save to files
    for (size_t ch = 0; ch < num_channels; ch++) {
        std::string filename = output_dir + "/usrp_" + std::to_string(ch) + ".bin";
        std::ofstream outfile(filename, std::ios::binary);
        if (outfile.is_open()) {
            outfile.write(reinterpret_cast<const char*>(buffs[ch].data()), num_rx_samps * sizeof(std::complex<int16_t>));
            outfile.close();
            std::cout << "Saved " << filename << std::endl;
        }
    }

    // Write metadata JSON
    pt::ptree pt;
    pt.put("capture_tool", "sync_validation_capture");
    pt.put("parameters.center_frequency_hz", freq);
    pt.put("parameters.sample_rate_hz", rate);
    pt.put("parameters.gain_db", gain);
    pt.put("parameters.duration_s", duration);
    pt.put("parameters.sample_format", format);
    pt.put("parameters.clock_source", clock_source);
    pt.put("parameters.time_source", time_source);
    
    std::string meta_filename = output_dir + "/metadata.json";
    pt::write_json(meta_filename, pt);
    std::cout << "Saved metadata to " << meta_filename << std::endl;

    return EXIT_SUCCESS;
}
