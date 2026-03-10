//
// Copyright 2026 Ettus Research
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

/**
 * sync_validation_test.cpp
 * 
 * Simple application to validate time/frequency synchronization between two USRP B210 devices.
 * 
 * This program:
 * 1. Configures two USRPs with the same center frequency and sample rate
 * 2. Synchronizes their time using PPS (and optionally 10 MHz reference)
 * 3. Starts streaming at a specified future time (simultaneous start)
 * 4. Captures samples from both devices
 * 5. Saves samples to files for offline analysis
 * 6. Performs basic alignment checks (time offset, phase correlation)
 * 
 * Usage examples:
 *   # PPS-only sync (internal clocks)
 *   ./sync_validation_test --serial0 30B56D6 --serial1 30DBC3C --freq 915e6
 * 
 *   # 10MHz + PPS sync (optimal)
 *   ./sync_validation_test --serial0 30B56D6 --serial1 30DBC3C --freq 915e6 --ref external
 * 
 *   # With GPSDO
 *   ./sync_validation_test --serial0 30B56D6 --serial1 30DBC3C --freq 915e6 --ref gpsdo
 */

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
#include <vector>

namespace po = boost::program_options;

// Global flag for clean exit
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}

// Structure to hold samples from one device
struct device_samples {
    std::vector<std::complex<float>> samples;
    uhd::time_spec_t start_time;
    uhd::time_spec_t end_time;
    double actual_rate;
};

/**
 * Synchronize time across multiple USRPs using PPS signal
 */
void sync_usrp_time(const std::vector<uhd::usrp::multi_usrp::sptr>& usrps)
{
    std::cout << "\n=== Synchronizing USRP Time ===" << std::endl;
    
    // Wait for PPS to ensure we're not at edge
    std::cout << "Waiting for PPS..." << std::endl;
    auto last_pps_time = usrps[0]->get_time_last_pps();
    while (last_pps_time == usrps[0]->get_time_last_pps()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Set next PPS to time 0.0
    std::cout << "Setting time on next PPS..." << std::endl;
    for (auto& usrp : usrps) {
        usrp->set_time_next_pps(uhd::time_spec_t(0.0));
    }
    
    // Wait for next PPS to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Verify synchronization
    std::cout << "\nTime synchronization check:" << std::endl;
    double max_time_diff = 0.0;
    auto reference_time = usrps[0]->get_time_now();
    
    for (size_t i = 0; i < usrps.size(); i++) {
        auto device_time = usrps[i]->get_time_now();
        double time_diff = std::abs(device_time.get_real_secs() - reference_time.get_real_secs());
        max_time_diff = std::max(max_time_diff, time_diff);
        
        std::cout << boost::format("  Device %d: %.9f seconds") % i % device_time.get_real_secs()
                  << std::endl;
    }
    
    std::cout << boost::format("  Max time difference: %.6f us") % (max_time_diff * 1e6)
              << std::endl;
    
    if (max_time_diff > 1e-3) {  // 1 ms threshold
        std::cerr << "\nWARNING: Time difference exceeds 1 ms!" << std::endl;
        std::cerr << "Check PPS connections and make sure both devices receive the signal." << std::endl;
    } else {
        std::cout << "  ✓ Time synchronization GOOD" << std::endl;
    }
}

/**
 * Check reference lock status
 */
bool check_ref_lock(uhd::usrp::multi_usrp::sptr usrp, const std::string& clock_source)
{
    if (clock_source == "internal") {
        return true;  // Internal clock always "locked"
    }
    
    // Check if device has ref_locked sensor
    auto sensor_names = usrp->get_mboard_sensor_names();
    if (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") == sensor_names.end()) {
        std::cout << "  Note: Device doesn't have ref_locked sensor, assuming OK" << std::endl;
        return true;
    }
    
    // Wait for lock with timeout
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 5.0) {
        try {
            auto sensor = usrp->get_mboard_sensor("ref_locked");
            if (sensor.to_bool()) {
                return true;
            }
        } catch (...) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return false;
}

/**
 * Receive samples from a single USRP
 */
device_samples receive_samples(uhd::usrp::multi_usrp::sptr usrp,
    size_t num_samples,
    const uhd::time_spec_t& start_time,
    size_t device_id)
{
    device_samples result;
    result.samples.resize(num_samples);
    result.start_time = start_time;
    result.actual_rate = usrp->get_rx_rate();
    
    // Create stream
    uhd::stream_args_t stream_args("fc32", "sc16");
    stream_args.channels = {0};  // Use first channel
    auto rx_stream = usrp->get_rx_stream(stream_args);
    
    // Setup streaming command
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = num_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = start_time;
    
    rx_stream->issue_stream_cmd(stream_cmd);
    
    // Receive samples
    uhd::rx_metadata_t md;
    size_t num_rx_samps = 0;
    bool overflow_message = false;
    
    std::cout << boost::format("Device %d: Waiting for samples at t=%.3f...") % device_id
                  % start_time.get_real_secs() << std::endl;
    
    // Receive in chunks (like rx_samples_to_file does)
    const size_t samps_per_buff = rx_stream->get_max_num_samps();
    
    while (num_rx_samps < num_samples && !stop_signal_called) {
        size_t num_to_recv = std::min(samps_per_buff, num_samples - num_rx_samps);
        size_t num_received = rx_stream->recv(
            &result.samples[num_rx_samps], num_to_recv, md, 3.0, false);
        
        // Handle errors
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
            std::cerr << boost::format("Device %d: Timeout waiting for samples") % device_id
                      << std::endl;
            break;
        }
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            if (!overflow_message) {
                overflow_message = true;
                std::cerr << boost::format("Device %d: Overflow detected (D)") % device_id 
                          << std::endl;
            }
            // Continue receiving
            continue;
        }
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE 
            && md.error_code != uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
            std::cerr << boost::format("Device %d: Receiver error: %s") % device_id
                          % md.strerror() << std::endl;
            break;
        }
        
        num_rx_samps += num_received;
        
        // Print first sample timestamp
        if (num_rx_samps == num_received && md.has_time_spec) {
            std::cout << boost::format("Device %d: First sample at t=%.9f") % device_id
                          % md.time_spec.get_real_secs() << std::endl;
        }
    }
    
    result.samples.resize(num_rx_samps);
    result.end_time = usrp->get_time_now();
    
    std::cout << boost::format("Device %d: Received %d samples") % device_id % num_rx_samps
              << std::endl;
    
    return result;
}

/**
 * Save samples to file
 */
void save_samples(const std::string& filename, const std::vector<std::complex<float>>& samples)
{
    std::ofstream outfile(filename, std::ios::binary);
    if (!outfile.is_open()) {
        std::cerr << "ERROR: Cannot open file: " << filename << std::endl;
        return;
    }
    
    outfile.write(reinterpret_cast<const char*>(samples.data()),
        samples.size() * sizeof(std::complex<float>));
    
    outfile.close();
    std::cout << "Saved " << samples.size() << " samples to " << filename << std::endl;
}

/**
 * Perform basic alignment analysis
 */
void analyze_alignment(const device_samples& dev0, const device_samples& dev1)
{
    std::cout << "\n=== Alignment Analysis ===" << std::endl;
    
    // Time alignment
    double time_offset = dev1.start_time.get_real_secs() - dev0.start_time.get_real_secs();
    std::cout << boost::format("Time offset: %.9f seconds (%.3f us)") % time_offset
                  % (time_offset * 1e6) << std::endl;
    
    if (std::abs(time_offset) < 1e-6) {  // 1 microsecond
        std::cout << "  ✓ Time alignment EXCELLENT (< 1 us)" << std::endl;
    } else if (std::abs(time_offset) < 1e-5) {  // 10 microseconds
        std::cout << "  ✓ Time alignment GOOD (< 10 us)" << std::endl;
    } else if (std::abs(time_offset) < 1e-4) {  // 100 microseconds
        std::cout << "  ⚠ Time alignment FAIR (< 100 us)" << std::endl;
    } else {
        std::cout << "  ✗ Time alignment POOR (> 100 us)" << std::endl;
    }
    
    // Cross-correlation for time delay estimation
    if (dev0.samples.size() > 100 && dev1.samples.size() > 100) {
        std::cout << "\nCross-correlation analysis:" << std::endl;
        
        // Use first 1000 samples for quick correlation
        size_t corr_len = std::min(size_t(1000), std::min(dev0.samples.size(), dev1.samples.size()));
        
        // Simple cross-correlation at zero lag
        std::complex<double> correlation(0.0, 0.0);
        double power0 = 0.0, power1 = 0.0;
        
        for (size_t i = 0; i < corr_len; i++) {
            correlation += std::complex<double>(dev0.samples[i]) * 
                          std::conj(std::complex<double>(dev1.samples[i]));
            power0 += std::norm(dev0.samples[i]);
            power1 += std::norm(dev1.samples[i]);
        }
        
        double normalized_corr = std::abs(correlation) / std::sqrt(power0 * power1);
        double phase_diff_deg = std::arg(correlation) * 180.0 / M_PI;
        
        std::cout << boost::format("  Correlation coefficient: %.4f") % normalized_corr << std::endl;
        std::cout << boost::format("  Phase difference: %.2f degrees") % phase_diff_deg << std::endl;
        
        if (normalized_corr > 0.9) {
            std::cout << "  ✓ Strong correlation - devices are well synchronized" << std::endl;
        } else if (normalized_corr > 0.7) {
            std::cout << "  ⚠ Moderate correlation - check frequency accuracy" << std::endl;
        } else {
            std::cout << "  ✗ Weak correlation - sync may have issues" << std::endl;
        }
    }
    
    // Power level comparison
    double power0 = 0.0, power1 = 0.0;
    for (size_t i = 0; i < std::min(dev0.samples.size(), dev1.samples.size()); i++) {
        power0 += std::norm(dev0.samples[i]);
        power1 += std::norm(dev1.samples[i]);
    }
    power0 /= dev0.samples.size();
    power1 /= dev1.samples.size();
    
    double power_diff_db = 10.0 * std::log10(power0 / power1);
    
    std::cout << "\nPower levels:" << std::endl;
    std::cout << boost::format("  Device 0: %.2f dB") % (10.0 * std::log10(power0)) << std::endl;
    std::cout << boost::format("  Device 1: %.2f dB") % (10.0 * std::log10(power1)) << std::endl;
    std::cout << boost::format("  Difference: %.2f dB") % power_diff_db << std::endl;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // Set thread priority
    uhd::set_thread_priority_safe();
    
    // Program options
    std::string serial0, serial1, ref, pps, subdev;
    double freq, rate, gain, duration;
    size_t num_samples;
    
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("serial0", po::value<std::string>(&serial0)->required(), "Serial number of first USRP")
        ("serial1", po::value<std::string>(&serial1)->required(), "Serial number of second USRP")
        ("freq", po::value<double>(&freq)->default_value(915e6), "RF center frequency in Hz")
        ("rate", po::value<double>(&rate)->default_value(10e6), "Sample rate in Hz")
        ("gain", po::value<double>(&gain)->default_value(40), "RX gain in dB")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), 
            "Clock reference (internal, external, gpsdo)")
        ("pps", po::value<std::string>(&pps)->default_value("external"), 
            "PPS source (none, external, gpsdo)")
        ("duration", po::value<double>(&duration)->default_value(1.0), 
            "Duration to capture in seconds")
        ("nsamps", po::value<size_t>(&num_samples)->default_value(0), 
            "Number of samples to capture (overrides duration)")
        ("subdev", po::value<std::string>(&subdev)->default_value("A:A"), 
            "Subdevice specification")
    ;
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    
    if (vm.count("help")) {
        std::cout << "Synchronization Validation Test" << std::endl << std::endl;
        std::cout << "This tool validates time/frequency sync between two B210 devices." << std::endl;
        std::cout << desc << std::endl;
        std::cout << "\nExamples:" << std::endl;
        std::cout << "  PPS-only (internal clocks):" << std::endl;
        std::cout << "    ./sync_validation_test --serial0 ABC --serial1 DEF --freq 915e6" << std::endl;
        std::cout << "\n  10MHz + PPS (optimal):" << std::endl;
        std::cout << "    ./sync_validation_test --serial0 ABC --serial1 DEF --freq 915e6 --ref external" << std::endl;
        std::cout << "\n  With GPSDO:" << std::endl;
        std::cout << "    ./sync_validation_test --serial0 ABC --serial1 DEF --freq 915e6 --ref gpsdo --pps gpsdo" << std::endl;
        return EXIT_SUCCESS;
    }
    
    po::notify(vm);
    
    // Calculate number of samples if not specified
    if (num_samples == 0) {
        num_samples = size_t(duration * rate);
    }
    
    // Register signal handler
    std::signal(SIGINT, &sig_int_handler);
    std::cout << "Press Ctrl+C to stop" << std::endl;
    
    std::cout << "\n=== USRP Synchronization Validation Test ===" << std::endl;
    std::cout << boost::format("Configuration:") << std::endl;
    std::cout << boost::format("  Device 0: %s") % serial0 << std::endl;
    std::cout << boost::format("  Device 1: %s") % serial1 << std::endl;
    std::cout << boost::format("  Frequency: %.3f MHz") % (freq / 1e6) << std::endl;
    std::cout << boost::format("  Sample rate: %.3f MHz") % (rate / 1e6) << std::endl;
    std::cout << boost::format("  RX gain: %.1f dB") % gain << std::endl;
    std::cout << boost::format("  Clock reference: %s") % ref << std::endl;
    std::cout << boost::format("  PPS source: %s") % pps << std::endl;
    std::cout << boost::format("  Samples to capture: %d (%.3f sec)") % num_samples 
                  % (num_samples / rate) << std::endl;
    
    // Create device addresses
    uhd::device_addr_t dev0_addr;
    dev0_addr["serial"] = serial0;
    
    uhd::device_addr_t dev1_addr;
    dev1_addr["serial"] = serial1;
    
    // Create USRPs
    std::cout << "\n=== Creating USRP Devices ===" << std::endl;
    auto usrp0 = uhd::usrp::multi_usrp::make(dev0_addr);
    auto usrp1 = uhd::usrp::multi_usrp::make(dev1_addr);
    
    std::cout << "Device 0: " << usrp0->get_pp_string() << std::endl;
    std::cout << "Device 1: " << usrp1->get_pp_string() << std::endl;
    
    std::vector<uhd::usrp::multi_usrp::sptr> usrps = {usrp0, usrp1};
    
    // Configure clock/time sources
    std::cout << "\n=== Configuring Clock/Time Sources ===" << std::endl;
    for (size_t i = 0; i < usrps.size(); i++) {
        std::cout << boost::format("Device %d:") % i << std::endl;
        
        // Set clock source
        usrps[i]->set_clock_source(ref);
        std::cout << boost::format("  Clock source: %s") % usrps[i]->get_clock_source(0) << std::endl;
        
        // Set time source
        if (pps != "none") {
            usrps[i]->set_time_source(pps);
            std::cout << boost::format("  Time source: %s") % usrps[i]->get_time_source(0) << std::endl;
        }
        
        // Check reference lock
        if (!check_ref_lock(usrps[i], ref)) {
            std::cerr << boost::format("ERROR: Device %d reference not locked!") % i << std::endl;
            std::cerr << "Check " << ref << " reference connection." << std::endl;
            return EXIT_FAILURE;
        } else {
            std::cout << "  ✓ Reference locked" << std::endl;
        }
    }
    
    // Synchronize time
    if (pps != "none") {
        sync_usrp_time(usrps);
    } else {
        std::cout << "\nWARNING: No PPS synchronization - time alignment will be poor!" << std::endl;
        // Set arbitrary time
        for (auto& usrp : usrps) {
            usrp->set_time_now(uhd::time_spec_t(0.0));
        }
    }
    
    // Configure RX parameters
    std::cout << "\n=== Configuring RX Parameters ===" << std::endl;
    for (size_t i = 0; i < usrps.size(); i++) {
        usrps[i]->set_rx_subdev_spec(subdev);
        usrps[i]->set_rx_rate(rate);
        usrps[i]->set_rx_freq(uhd::tune_request_t(freq));
        usrps[i]->set_rx_gain(gain);
        usrps[i]->set_rx_antenna("RX2");
        
        std::cout << boost::format("Device %d:") % i << std::endl;
        std::cout << boost::format("  Actual RX rate: %.3f MHz") 
                      % (usrps[i]->get_rx_rate() / 1e6) << std::endl;
        std::cout << boost::format("  Actual RX freq: %.3f MHz") 
                      % (usrps[i]->get_rx_freq() / 1e6) << std::endl;
        std::cout << boost::format("  Actual RX gain: %.1f dB") 
                      % usrps[i]->get_rx_gain() << std::endl;
    }
    
    // Allow time for settling
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Calculate start time (1 second in the future)
    auto current_time = usrp0->get_time_now();
    auto start_time = uhd::time_spec_t(current_time.get_real_secs() + 1.0);
    
    std::cout << "\n=== Starting Synchronized Capture ===" << std::endl;
    std::cout << boost::format("Current time: %.3f") % current_time.get_real_secs() << std::endl;
    std::cout << boost::format("Start time: %.3f") % start_time.get_real_secs() << std::endl;
    std::cout << "Starting streaming on both devices..." << std::endl;
    
    // Use promises/futures to get results from threads
    std::promise<device_samples> promise0, promise1;
    auto future0 = promise0.get_future();
    auto future1 = promise1.get_future();
    
    std::thread thread0([&]() {
        try {
            promise0.set_value(receive_samples(usrp0, num_samples, start_time, 0));
        } catch (...) {
            promise0.set_exception(std::current_exception());
        }
    });
    
    std::thread thread1([&]() {
        try {
            promise1.set_value(receive_samples(usrp1, num_samples, start_time, 1));
        } catch (...) {
            promise1.set_exception(std::current_exception());
        }
    });
    
    // Wait for completion
    thread0.join();
    thread1.join();
    
    // Get results
    device_samples dev0_samples = future0.get();
    device_samples dev1_samples = future1.get();
    
    if (stop_signal_called) {
        std::cout << "\nInterrupted by user" << std::endl;
        return EXIT_SUCCESS;
    }
    
    // Save samples to files
    std::cout << "\n=== Saving Samples ===" << std::endl;
    save_samples("device0_samples.dat", dev0_samples.samples);
    save_samples("device1_samples.dat", dev1_samples.samples);
    
    // Analyze alignment
    analyze_alignment(dev0_samples, dev1_samples);
    
    // Summary
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "✓ Capture complete" << std::endl;
    std::cout << "✓ Samples saved to device0_samples.dat and device1_samples.dat" << std::endl;
    std::cout << "\nFor detailed analysis, you can:" << std::endl;
    std::cout << "  1. Load samples in MATLAB/Python" << std::endl;
    std::cout << "  2. Compute spectrograms to visualize alignment" << std::endl;
    std::cout << "  3. Calculate detailed cross-correlation" << std::endl;
    std::cout << "  4. Measure frequency drift over time" << std::endl;
    
    std::cout << "\nPython analysis example:" << std::endl;
    std::cout << "  import numpy as np" << std::endl;
    std::cout << "  dev0 = np.fromfile('device0_samples.dat', dtype=np.complex64)" << std::endl;
    std::cout << "  dev1 = np.fromfile('device1_samples.dat', dtype=np.complex64)" << std::endl;
    std::cout << "  corr = np.correlate(dev0, dev1, mode='full')" << std::endl;
    std::cout << "  # Plot correlation, spectrograms, etc." << std::endl;
    
    return EXIT_SUCCESS;
}
