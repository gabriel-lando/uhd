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
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

namespace po = boost::program_options;

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args, clock_source, time_source, output_log;
    double duration, interval;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Help message")
        ("args", po::value<std::string>(&args)->default_value("serial0=30B56D6,serial1=30DBC3C"), "USRP device arguments")
        ("clock-source", po::value<std::string>(&clock_source)->default_value("internal"), "Clock source")
        ("time-source", po::value<std::string>(&time_source)->default_value("external"), "Time source")
        ("duration", po::value<double>(&duration)->default_value(3600), "Duration to run in seconds")
        ("interval", po::value<double>(&interval)->default_value(1.0), "Measurement interval in seconds")
        ("output", po::value<std::string>(&output_log)->default_value("drift_log.csv"), "Output CSV file");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << "Drift Monitor\n" << desc << std::endl;
        return ~0;
    }
    po::notify(vm);

    std::cout << "Creating multi_usrp with args: " << args << std::endl;
    auto usrp = uhd::usrp::multi_usrp::make(args);
    const size_t num_mboards = usrp->get_num_mboards();

    if (num_mboards < 2) {
        std::cerr << "Drift monitor requires at least 2 motherboards." << std::endl;
        return ~0;
    }

    for (size_t m = 0; m < num_mboards; m++) {
        usrp->set_clock_source(clock_source, m);
        usrp->set_time_source(time_source, m);
    }

    std::cout << "Synchronizing..." << std::endl;
    if (time_source == "external" || time_source == "gpsdo") {
        usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } else {
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }

    std::ofstream csv(output_log);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output file: " << output_log << std::endl;
        return ~0;
    }

    csv << "elapsed_s,device_0_pps_ticks,device_1_pps_ticks,delta_ticks,delta_us,drift_ppm\n";

    std::cout << "Monitoring drift for " << duration << " seconds..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed > duration) break;

        uhd::time_spec_t t0 = usrp->get_time_last_pps(0);
        uhd::time_spec_t t1 = usrp->get_time_last_pps(1);

        long long ticks0 = t0.to_ticks(1e6); // Assuming 1 MHz tick rate for simplicity of logic
        long long ticks1 = t1.to_ticks(1e6);
        long long delta = ticks1 - ticks0;
        double delta_us = static_cast<double>(delta);
        double drift_ppm = elapsed > 0 ? (delta_us / elapsed) : 0.0;

        csv << elapsed << "," << ticks0 << "," << ticks1 << "," << delta << "," << delta_us << "," << drift_ppm << "\n";
        
        std::cout << boost::format("Elapsed: %.1f s | Delta: %lld ticks | Drift: %.3f ppm") % elapsed % delta % drift_ppm << std::endl;

        std::this_thread::sleep_for(std::chrono::duration<double>(interval));
    }

    csv.close();
    std::cout << "Done! Results saved to " << output_log << std::endl;
    return EXIT_SUCCESS;
}
