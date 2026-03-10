//
// Copyright 2026 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

//
// sync_validation_test.cpp
//
// Validates time and phase synchronization across multiple USRP devices sharing
// an external PPS reference (and optionally a shared 10 MHz clock).  Each device
// is opened with its own multi_usrp handle and rx_streamer; all streamers start
// at the same absolute timed hardware timestamp to provide synchronized capture.
//
// Output:
//   <output-dir>/usrp_0_<serial>.bin   – raw IQ samples (sc16 or fc32)
//   <output-dir>/usrp_N_<serial>.bin
//   <output-dir>/metadata.json         – capture parameters and sync results
//
// Build: listed in examples/CMakeLists.txt (same deps as other examples).
//
// Usage:
//   sync_validation_test --serials "30B56D6,30DBC3C" --freq 915e6 --rate 10e6
//

#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <complex>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace po = boost::program_options;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
static volatile bool stop_signal_called = false;

void sig_int_handler(int)
{
    stop_signal_called = true;
}

// ---------------------------------------------------------------------------
// Per-device capture state
// ---------------------------------------------------------------------------
struct DeviceState
{
    size_t index{0};
    std::string serial;
    uhd::usrp::multi_usrp::sptr usrp;
    uhd::rx_streamer::sptr streamer;

    // metadata filled during/after capture
    std::string mboard_name;
    double actual_freq_hz{0.0};
    double actual_rate_hz{0.0};
    double actual_gain_db{0.0};

    uhd::time_spec_t first_timestamp{0.0};
    bool first_timestamp_set{false};
    size_t total_samples{0};
    size_t overflow_count{0};
    std::vector<std::string> stream_errors;

    // drift measurement (filled post-capture)
    double pps_before_s{0.0};
    double pps_after_s{0.0};
    double drift_ppm{0.0};
    bool drift_measured{false};
};

// ---------------------------------------------------------------------------
// Helper: current UTC time as ISO-8601 string
// ---------------------------------------------------------------------------
static std::string utc_now_str()
{
    auto now       = std::chrono::system_clock::now();
    std::time_t t  = std::chrono::system_clock::to_time_t(now);
    std::tm* utctm = std::gmtime(&t);
    std::ostringstream oss;
    oss << std::put_time(utctm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

// ---------------------------------------------------------------------------
// Helper: JSON-escape a string (minimal – only escapes backslash and quote)
// ---------------------------------------------------------------------------
static std::string json_str(const std::string& s)
{
    std::string out;
    out.reserve(s.size() + 2);
    out += '"';
    for (char c : s) {
        if (c == '"')
            out += "\\\"";
        else if (c == '\\')
            out += "\\\\";
        else
            out += c;
    }
    out += '"';
    return out;
}

// ---------------------------------------------------------------------------
// Capture thread – runs independently for each device
// ---------------------------------------------------------------------------
static void capture_thread(DeviceState& dev,
    const std::string& output_dir,
    const std::string& format,
    size_t spb,
    size_t total_samps,
    std::mutex& print_mutex)
{
    const size_t bytes_per_samp = (format == "sc16") ? 4 : 8;

    // Open output binary file
    const std::string fname =
        output_dir + "/usrp_" + std::to_string(dev.index) + "_" + dev.serial + ".bin";
    std::ofstream outfile(fname, std::ios::binary);
    if (!outfile.is_open()) {
        std::lock_guard<std::mutex> lock(print_mutex);
        std::cerr << "[ERROR] Cannot open output file: " << fname << std::endl;
        return;
    }

    // Allocate receive buffer
    const double recv_timeout = 3.0; // generous first-packet timeout
    uhd::rx_metadata_t md;

    if (format == "sc16") {
        std::vector<std::complex<short>> buf(spb);
        std::vector<void*> buffs = {buf.data()};

        size_t samps_received = 0;
        bool first_packet     = true;

        while (!stop_signal_called && samps_received < total_samps) {
            const size_t samps_to_recv = std::min(spb, total_samps - samps_received);
            size_t n = dev.streamer->recv(buffs, samps_to_recv, md,
                first_packet ? recv_timeout : 0.1, false);

            if (first_packet && n > 0) {
                dev.first_timestamp     = md.time_spec;
                dev.first_timestamp_set = true;
                first_packet            = false;
            }

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                dev.overflow_count++;
                if (dev.overflow_count <= 5) {
                    std::lock_guard<std::mutex> lock(print_mutex);
                    std::cerr << boost::format("[WARN] Device %d (%s): overflow #%u\n")
                                     % dev.index % dev.serial % dev.overflow_count;
                }
            } else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::lock_guard<std::mutex> lock(print_mutex);
                std::cerr << boost::format(
                                 "[ERROR] Device %d (%s): timeout, stopping\n")
                                 % dev.index % dev.serial;
                dev.stream_errors.push_back("TIMEOUT");
                break;
            } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::lock_guard<std::mutex> lock(print_mutex);
                std::cerr << boost::format(
                                 "[WARN] Device %d (%s): error code 0x%x\n")
                                 % dev.index % dev.serial % md.error_code;
                dev.stream_errors.push_back("ERR_" + std::to_string(md.error_code));
            }

            if (n > 0) {
                outfile.write(
                    reinterpret_cast<const char*>(buf.data()), n * bytes_per_samp);
                samps_received += n;
                dev.total_samples = samps_received;
            }
        }
    } else {
        // fc32
        std::vector<std::complex<float>> buf(spb);
        std::vector<void*> buffs = {buf.data()};

        size_t samps_received = 0;
        bool first_packet     = true;

        while (!stop_signal_called && samps_received < total_samps) {
            const size_t samps_to_recv = std::min(spb, total_samps - samps_received);
            size_t n = dev.streamer->recv(buffs, samps_to_recv, md,
                first_packet ? recv_timeout : 0.1, false);

            if (first_packet && n > 0) {
                dev.first_timestamp     = md.time_spec;
                dev.first_timestamp_set = true;
                first_packet            = false;
            }

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW) {
                dev.overflow_count++;
                if (dev.overflow_count <= 5) {
                    std::lock_guard<std::mutex> lock(print_mutex);
                    std::cerr << boost::format("[WARN] Device %d (%s): overflow #%u\n")
                                     % dev.index % dev.serial % dev.overflow_count;
                }
            } else if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::lock_guard<std::mutex> lock(print_mutex);
                std::cerr << boost::format(
                                 "[ERROR] Device %d (%s): timeout, stopping\n")
                                 % dev.index % dev.serial;
                dev.stream_errors.push_back("TIMEOUT");
                break;
            } else if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                std::lock_guard<std::mutex> lock(print_mutex);
                std::cerr << boost::format(
                                 "[WARN] Device %d (%s): error code 0x%x\n")
                                 % dev.index % dev.serial % md.error_code;
                dev.stream_errors.push_back("ERR_" + std::to_string(md.error_code));
            }

            if (n > 0) {
                outfile.write(
                    reinterpret_cast<const char*>(buf.data()), n * bytes_per_samp);
                samps_received += n;
                dev.total_samples = samps_received;
            }
        }
    }

    outfile.close();
}

// ---------------------------------------------------------------------------
// Progress reporter thread
// ---------------------------------------------------------------------------
static void progress_thread(const std::vector<DeviceState>& devs,
    size_t total_samps,
    std::mutex& print_mutex)
{
    const double report_interval_s = 0.5;
    while (!stop_signal_called) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(report_interval_s));

        size_t min_samps = total_samps;
        size_t total_overflows = 0;
        for (const auto& d : devs) {
            min_samps = std::min(min_samps, d.total_samples);
            total_overflows += d.overflow_count;
        }

        double pct = 100.0 * static_cast<double>(min_samps)
                     / static_cast<double>(total_samps);

        std::lock_guard<std::mutex> lock(print_mutex);
        std::cout << boost::format(
                         "[CAPTURE] Progress: %5.1f%% | %.2fM samples | "
                         "%zu overflows\n")
                         % pct % (min_samps / 1e6) % total_overflows;
        std::cout.flush();

        if (min_samps >= total_samps)
            break;
    }
}

// ---------------------------------------------------------------------------
// Write metadata.json
// ---------------------------------------------------------------------------
static void write_metadata(const std::string& output_dir,
    const std::string& capture_ts,
    double freq,
    double rate,
    double gain,
    double bw,
    double duration,
    const std::string& format,
    const std::string& clock_source,
    const std::vector<DeviceState>& devs,
    const std::vector<double>& pps_deltas_us,
    bool drift_check_performed,
    double drift_duration_s)
{
    const size_t bytes_per_samp = (format == "sc16") ? 4 : 8;
    const std::string path      = output_dir + "/metadata.json";
    std::ofstream f(path);

    f << "{\n";
    f << "    \"capture_tool\": \"sync_validation_test\",\n";
    f << "    \"capture_version\": \"1.0\",\n";
    f << "    \"capture_timestamp_utc\": " << json_str(capture_ts) << ",\n";
    f << "    \"parameters\": {\n";
    f << boost::format("        \"center_frequency_hz\": %.1f,\n") % freq;
    f << boost::format("        \"sample_rate_hz\": %.1f,\n") % rate;
    f << boost::format("        \"gain_db\": %.1f,\n") % gain;
    f << boost::format("        \"bandwidth_hz\": %.1f,\n") % bw;
    f << boost::format("        \"duration_s\": %.6f,\n") % duration;
    f << "        \"sample_format\": " << json_str(format) << ",\n";
    f << "        \"clock_source\": " << json_str(clock_source) << ",\n";
    f << "        \"time_source\": \"external\",\n";
    f << "        \"bytes_per_sample\": " << bytes_per_samp << "\n";
    f << "    },\n";

    f << "    \"devices\": [\n";
    for (size_t i = 0; i < devs.size(); ++i) {
        const auto& d = devs[i];
        const std::string bin_name =
            "usrp_" + std::to_string(d.index) + "_" + d.serial + ".bin";
        f << "        {\n";
        f << "            \"index\": " << d.index << ",\n";
        f << "            \"serial\": " << json_str(d.serial) << ",\n";
        f << "            \"mboard_name\": " << json_str(d.mboard_name) << ",\n";
        f << "            \"file\": " << json_str(bin_name) << ",\n";
        f << boost::format("            \"actual_frequency_hz\": %.6f,\n")
                 % d.actual_freq_hz;
        f << boost::format("            \"actual_rate_hz\": %.6f,\n") % d.actual_rate_hz;
        f << boost::format("            \"actual_gain_db\": %.6f,\n") % d.actual_gain_db;

        if (d.first_timestamp_set) {
            const int64_t ticks = static_cast<int64_t>(
                d.first_timestamp.get_real_secs() * d.actual_rate_hz);
            f << "            \"first_sample_timestamp_ticks\": " << ticks << ",\n";
            f << boost::format("            \"first_sample_timestamp_s\": %.9f,\n")
                     % d.first_timestamp.get_real_secs();
        } else {
            f << "            \"first_sample_timestamp_ticks\": null,\n";
            f << "            \"first_sample_timestamp_s\": null,\n";
        }

        f << "            \"total_samples\": " << d.total_samples << ",\n";
        f << "            \"overflow_count\": " << d.overflow_count << ",\n";
        f << "            \"stream_errors\": [";
        for (size_t e = 0; e < d.stream_errors.size(); ++e) {
            if (e)
                f << ", ";
            f << json_str(d.stream_errors[e]);
        }
        f << "]\n";
        f << "        }" << (i + 1 < devs.size() ? "," : "") << "\n";
    }
    f << "    ],\n";

    // Sync results
    f << "    \"sync_results\": {\n";
    f << "        \"pps_time_deltas_us\": [";
    for (size_t i = 0; i < pps_deltas_us.size(); ++i) {
        if (i)
            f << ", ";
        f << boost::format("%.4f") % pps_deltas_us[i];
    }
    f << "],\n";

    // Timestamp alignment ticks (delta from device 0's first timestamp)
    f << "        \"timestamp_alignment_ticks\": [";
    for (size_t i = 0; i < devs.size(); ++i) {
        if (i)
            f << ", ";
        if (devs[0].first_timestamp_set && devs[i].first_timestamp_set) {
            double delta_s = devs[i].first_timestamp.get_real_secs()
                             - devs[0].first_timestamp.get_real_secs();
            int64_t delta_ticks =
                static_cast<int64_t>(delta_s * devs[i].actual_rate_hz);
            f << delta_ticks;
        } else {
            f << "null";
        }
    }
    f << "],\n";

    f << "        \"drift_check_performed\": "
      << (drift_check_performed ? "true" : "false") << ",\n";
    f << boost::format("        \"drift_measurement_duration_s\": %.1f,\n")
             % drift_duration_s;

    f << "        \"drift_ppm\": [";
    for (size_t i = 0; i < devs.size(); ++i) {
        if (i)
            f << ", ";
        f << boost::format("%.4f") % devs[i].drift_ppm;
    }
    f << "],\n";
    f << "        \"drift_reference_device\": 0\n";
    f << "    }\n";
    f << "}\n";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::signal(SIGINT, sig_int_handler);

    // ------------------------------------------------------------------
    // Command-line options
    // ------------------------------------------------------------------
    std::string serials_str, serial0, serial1, serial2, serial3;
    std::string output_dir, format, clock_source, subdev, antenna, ref;
    double freq, rate, gain, bw, duration;
    size_t channel, spb;

    // clang-format off
    po::options_description desc("sync_validation_test options");
    desc.add_options()
        ("help,h",
            "Show this help message and exit.")
        ("serials",
            po::value<std::string>(&serials_str)->default_value(""),
            "Comma-separated list of USRP serial numbers (e.g. \"30B56D6,30DBC3C\").")
        ("serial0",
            po::value<std::string>(&serial0)->default_value(""),
            "Serial number of device 0 (alternative to --serials).")
        ("serial1",
            po::value<std::string>(&serial1)->default_value(""),
            "Serial number of device 1.")
        ("serial2",
            po::value<std::string>(&serial2)->default_value(""),
            "Serial number of device 2.")
        ("serial3",
            po::value<std::string>(&serial3)->default_value(""),
            "Serial number of device 3.")
        ("freq",
            po::value<double>(&freq)->default_value(915e6),
            "Center frequency in Hz (e.g. 915e6).")
        ("rate",
            po::value<double>(&rate)->default_value(1e6),
            "Sample rate in samples/s.")
        ("gain",
            po::value<double>(&gain)->default_value(0.0),
            "RX gain in dB.")
        ("bw",
            po::value<double>(&bw)->default_value(0.0),
            "Analog bandwidth in Hz (0 = use rate).")
        ("duration",
            po::value<double>(&duration)->default_value(1.0),
            "Capture duration in seconds.")
        ("output-dir",
            po::value<std::string>(&output_dir)->default_value("./sync_capture"),
            "Directory for output files.")
        ("format",
            po::value<std::string>(&format)->default_value("sc16"),
            "Sample format: sc16 (16-bit int) or fc32 (32-bit float).")
        ("clock-source",
            po::value<std::string>(&clock_source)->default_value("internal"),
            "Clock source: internal or external (10 MHz).")
        ("ref",
            po::value<std::string>(&ref)->default_value("external"),
            "PPS/time reference source (default: external).")
        ("subdev",
            po::value<std::string>(&subdev)->default_value(""),
            "RX subdevice spec applied to all USRPs (e.g. \"A:A\").")
        ("channel",
            po::value<size_t>(&channel)->default_value(0),
            "RX channel index per device.")
        ("antenna",
            po::value<std::string>(&antenna)->default_value(""),
            "Antenna port (e.g. RX2, TX/RX).")
        ("spb",
            po::value<size_t>(&spb)->default_value(10000),
            "Samples per buffer for recv().")
        ("skip-drift-check",
            "Skip post-capture PPS drift measurement.")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << "=== USRP Sync Validation Test ===" << std::endl << std::endl;
        std::cout << desc << std::endl;
        std::cout << "\nExample:" << std::endl;
        std::cout << "  sync_validation_test --serials \"30B56D6,30DBC3C\""
                     " --freq 915e6 --rate 10e6 --gain 30"
                  << std::endl;
        return EXIT_SUCCESS;
    }

    po::notify(vm);

    const bool skip_drift = vm.count("skip-drift-check") > 0;

    // Validate format
    if (format != "sc16" && format != "fc32") {
        std::cerr << "[ERROR] --format must be sc16 or fc32" << std::endl;
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // Build serial list (from --serials or --serial0/1/2/3)
    // ------------------------------------------------------------------
    std::vector<std::string> serials;

    if (!serials_str.empty()) {
        boost::split(serials, serials_str, boost::is_any_of(","));
        for (auto& s : serials)
            boost::trim(s);
    } else {
        for (const auto& s : {serial0, serial1, serial2, serial3}) {
            if (!s.empty())
                serials.push_back(s);
        }
    }

    if (serials.empty()) {
        std::cerr << "[ERROR] No serial numbers provided. "
                     "Use --serials or --serial0/--serial1."
                  << std::endl;
        return EXIT_FAILURE;
    }
    if (serials.size() > 8) {
        std::cerr << "[ERROR] Tool supports up to 8 devices." << std::endl;
        return EXIT_FAILURE;
    }

    const size_t num_devs = serials.size();

    // ------------------------------------------------------------------
    // Banner
    // ------------------------------------------------------------------
    std::cout << "\n=== USRP Sync Validation Test ===\n" << std::endl;
    std::cout << boost::format("[INFO] Opening %u device(s)") % num_devs;
    for (size_t i = 0; i < num_devs; ++i)
        std::cout << ((i == 0) ? ": " : ", ") << "serial=" << serials[i];
    std::cout << std::endl;

    // ------------------------------------------------------------------
    // Create output directory
    // ------------------------------------------------------------------
    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
        std::cerr << "[ERROR] Cannot create output directory: " << output_dir
                  << " — " << ec.message() << std::endl;
        return EXIT_FAILURE;
    }

    // ------------------------------------------------------------------
    // Open devices and configure RF
    // ------------------------------------------------------------------
    std::vector<DeviceState> devs(num_devs);
    const std::string capture_ts = utc_now_str();

    for (size_t i = 0; i < num_devs; ++i) {
        devs[i].index  = i;
        devs[i].serial = serials[i];

        const std::string addr = "serial=" + serials[i];
        try {
            devs[i].usrp = uhd::usrp::multi_usrp::make(addr);
        } catch (const std::exception& e) {
            std::cerr << boost::format("[ERROR] Cannot open device %u (serial=%s): %s\n")
                             % i % serials[i] % e.what();
            return EXIT_FAILURE;
        }

        devs[i].mboard_name = devs[i].usrp->get_mboard_name(0);
        std::cout << boost::format("[INFO] Device %u: %s (serial=%s)") % i
                         % devs[i].mboard_name % serials[i]
                  << std::endl;

        // Apply subdevice spec before anything else
        if (!subdev.empty())
            devs[i].usrp->set_rx_subdev_spec(subdev, 0);

        // Clock and time sources
        devs[i].usrp->set_clock_source(clock_source, 0);
        devs[i].usrp->set_time_source(ref, 0);

        // Sample rate
        devs[i].usrp->set_rx_rate(rate, channel);
        devs[i].actual_rate_hz = devs[i].usrp->get_rx_rate(channel);

        // Frequency
        uhd::tune_request_t tune_req(freq);
        devs[i].usrp->set_rx_freq(tune_req, channel);
        devs[i].actual_freq_hz = devs[i].usrp->get_rx_freq(channel);

        // Gain
        devs[i].usrp->set_rx_gain(gain, channel);
        devs[i].actual_gain_db = devs[i].usrp->get_rx_gain(channel);

        // Bandwidth
        if (bw > 0.0)
            devs[i].usrp->set_rx_bandwidth(bw, channel);

        // Antenna
        if (!antenna.empty())
            devs[i].usrp->set_rx_antenna(antenna, channel);
    }

    std::cout << boost::format("[INFO] Setting frequency: %.3f MHz") % (freq / 1e6)
              << std::endl;
    std::cout << boost::format("[INFO] Setting sample rate: %.3f MS/s") % (rate / 1e6)
              << std::endl;
    std::cout << boost::format("[INFO] Setting gain: %.1f dB") % gain << std::endl;

    // ------------------------------------------------------------------
    // PPS detection
    // ------------------------------------------------------------------
    std::cout << "\n[SYNC] Checking PPS on all devices..." << std::endl;

    for (size_t i = 0; i < num_devs; ++i) {
        uhd::time_spec_t t0 = devs[i].usrp->get_time_last_pps(0);
        auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        bool detected = false;

        while (std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(100ms);
            uhd::time_spec_t t1 = devs[i].usrp->get_time_last_pps(0);
            if (t1 != t0) {
                detected = true;
                break;
            }
        }

        if (!detected) {
            std::cerr << boost::format(
                             "[ERROR] Device %u (%s): no PPS detected within 2 "
                             "seconds.\n"
                             "        Check PPS cable and signal source.\n")
                             % i % serials[i];
            return EXIT_FAILURE;
        }
        std::cout << boost::format("[SYNC] Device %u: PPS detected") % i << std::endl;
    }

    // ------------------------------------------------------------------
    // Synchronize time at PPS edge (all devices simultaneously)
    // ------------------------------------------------------------------
    std::cout << "[SYNC] Synchronizing time at PPS edge..." << std::endl;

    // Strategy: wait for a PPS edge to fire on device 0, then immediately
    // call set_time_next_pps(0.0) on ALL devices before the next edge fires.
    // Because all devices share the same PPS signal, they will all latch 0
    // at the same physical edge — giving sub-microsecond time alignment.
    //
    // NOTE: do NOT use set_time_unknown_pps() sequentially — it blocks ~2 s
    // per call (waits for one full PPS edge internally), causing each device
    // to latch on a different edge and ending up ~1 s apart.
    {
        uhd::time_spec_t last_pps = devs[0].usrp->get_time_last_pps(0);
        auto edge_deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (devs[0].usrp->get_time_last_pps(0) == last_pps) {
            if (std::chrono::steady_clock::now() > edge_deadline) {
                std::cerr << "[ERROR] PPS edge not seen within 2 s on device 0 "
                             "during sync.\n";
                return EXIT_FAILURE;
            }
            std::this_thread::sleep_for(10ms);
        }
    }
    // A PPS edge just fired on device 0.  We now have ~1 second to schedule
    // the time latch on every device before the NEXT edge arrives.
    for (size_t i = 0; i < num_devs; ++i) {
        devs[i].usrp->set_time_next_pps(uhd::time_spec_t(0.0));
    }

    // Wait for the next PPS edge to fire and latch the scheduled time.
    // 1.1 s is sufficient; the extra 0.1 s provides margin for slow hosts.
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    // Verify time alignment
    std::vector<double> pps_times(num_devs);
    for (size_t i = 0; i < num_devs; ++i) {
        pps_times[i] = devs[i].usrp->get_time_last_pps(0).get_real_secs();
        std::cout << boost::format("[SYNC] Device %u: PPS time = %.6f s")
                         % i % pps_times[i]
                  << std::endl;
    }

    std::vector<double> pps_deltas_us(num_devs, 0.0);
    double max_delta_us = 0.0;
    for (size_t i = 1; i < num_devs; ++i) {
        pps_deltas_us[i] = (pps_times[i] - pps_times[0]) * 1e6;
        max_delta_us     = std::max(max_delta_us, std::abs(pps_deltas_us[i]));
    }

    const double sync_threshold_ms = 1.0;
    if (max_delta_us > sync_threshold_ms * 1000.0) {
        std::cerr << boost::format(
                         "[ERROR] Time delta %.3f µs exceeds %.0f ms threshold — "
                         "sync failed.\n")
                         % max_delta_us % sync_threshold_ms;
        return EXIT_FAILURE;
    }

    std::cout << boost::format("[SYNC] Max time delta: %.3f µs — %s") % max_delta_us
                     % (max_delta_us < 1.0 ? "PASS" : "WARNING (> 1 µs)")
              << std::endl;

    // ------------------------------------------------------------------
    // Set up RX streamers (one per device, single channel each)
    // ------------------------------------------------------------------
    const std::string cpu_format  = format;
    const std::string wire_format = "sc16"; // always sc16 on wire

    for (size_t i = 0; i < num_devs; ++i) {
        uhd::stream_args_t stream_args(cpu_format, wire_format);
        stream_args.channels = {channel};
        devs[i].streamer     = devs[i].usrp->get_rx_stream(stream_args);
    }

    // ------------------------------------------------------------------
    // Compute stream start time and total samples
    // ------------------------------------------------------------------
    // Use device 0 as the timing reference for the stream_now command.
    // All devices have synchronized hardware clocks (same PPS-latched time),
    // so a common absolute time_spec works across all of them.
    const double stream_delay_s     = 1.5; // time in advance to schedule start
    const uhd::time_spec_t now      = devs[0].usrp->get_time_now(0);
    const uhd::time_spec_t start_ts = now + uhd::time_spec_t(stream_delay_s);

    const size_t total_samps = static_cast<size_t>(duration * rate);

    std::cout << boost::format(
                     "\n[CAPTURE] Streaming %.3f M samples (%.3f s) "
                     "starting at t=%.3f s...")
                     % (total_samps / 1e6) % duration % start_ts.get_real_secs()
              << std::endl;

    // Issue timed stream command to all devices
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps   = total_samps;
    stream_cmd.stream_now  = false;
    stream_cmd.time_spec   = start_ts;

    for (size_t i = 0; i < num_devs; ++i) {
        devs[i].streamer->issue_stream_cmd(stream_cmd);
    }

    // ------------------------------------------------------------------
    // Capture threads + progress reporting
    // ------------------------------------------------------------------
    std::mutex print_mutex;
    std::vector<std::thread> capture_threads;

    for (size_t i = 0; i < num_devs; ++i) {
        capture_threads.emplace_back(
            capture_thread,
            std::ref(devs[i]),
            std::ref(output_dir),
            std::ref(format),
            spb,
            total_samps,
            std::ref(print_mutex));
    }

    std::thread prog_thread(
        progress_thread, std::cref(devs), total_samps, std::ref(print_mutex));

    // Join capture threads
    for (auto& t : capture_threads)
        t.join();

    stop_signal_called = true; // signal progress thread to exit
    prog_thread.join();

    // Stop streamers
    uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    for (size_t i = 0; i < num_devs; ++i) {
        try {
            devs[i].streamer->issue_stream_cmd(stop_cmd);
        } catch (...) {
        }
    }

    std::cout << boost::format(
                     "\n[CAPTURE] Done. All devices targeted %zu samples.\n")
                     % total_samps;

    // ------------------------------------------------------------------
    // First-packet timestamp comparison
    // ------------------------------------------------------------------
    bool ts_match = true;
    if (num_devs > 1) {
        for (size_t i = 1; i < num_devs; ++i) {
            if (devs[0].first_timestamp_set && devs[i].first_timestamp_set) {
                double delta_s = devs[i].first_timestamp.get_real_secs()
                                 - devs[0].first_timestamp.get_real_secs();
                double delta_samps = std::abs(delta_s * rate);
                if (delta_samps > 0.5) {
                    ts_match = false;
                    std::cout << boost::format(
                                     "[WARN] Device %u first-packet timestamp offset: "
                                     "%.3f µs (%.1f samples)\n")
                                     % i % (delta_s * 1e6) % delta_samps;
                }
            }
        }
        if (ts_match)
            std::cout << "[SYNC] First-packet timestamps match across devices." << std::endl;
    }

    // ------------------------------------------------------------------
    // Post-capture PPS drift measurement
    // ------------------------------------------------------------------
    const double drift_obs_s = 5.0;

    if (!skip_drift) {
        std::cout << boost::format(
                         "\n[DRIFT] Measuring PPS drift over %.0f seconds...")
                         % drift_obs_s
                  << std::endl;

        // First reading
        std::vector<double> pps_before(num_devs), pps_after(num_devs);
        for (size_t i = 0; i < num_devs; ++i)
            pps_before[i] = devs[i].usrp->get_time_last_pps(0).get_real_secs();

        std::this_thread::sleep_for(std::chrono::duration<double>(drift_obs_s));

        // Second reading
        for (size_t i = 0; i < num_devs; ++i)
            pps_after[i] = devs[i].usrp->get_time_last_pps(0).get_real_secs();

        // Device 0 is the reference; compute drift of device i relative to device 0
        const double ref_delta = (pps_after[0] - pps_before[0]);
        devs[0].drift_ppm      = 0.0;
        devs[0].drift_measured = true;
        std::cout << boost::format("[DRIFT] Device 0 (ref, serial=%s): 0.000 ppm")
                         % serials[0]
                  << std::endl;

        for (size_t i = 1; i < num_devs; ++i) {
            const double dev_delta = (pps_after[i] - pps_before[i]);
            // ppm = (delta_i - delta_0) / drift_obs * 1e6
            devs[i].drift_ppm     = (dev_delta - ref_delta) / drift_obs_s * 1e6;
            devs[i].drift_measured = true;
            std::cout << boost::format("[DRIFT] Device %u (serial=%s): %+.3f ppm")
                             % i % serials[i] % devs[i].drift_ppm
                      << std::endl;
        }
    }

    // ------------------------------------------------------------------
    // Verify sample counts
    // ------------------------------------------------------------------
    bool count_ok = true;
    for (size_t i = 1; i < num_devs; ++i) {
        if (devs[0].total_samples != devs[i].total_samples) {
            count_ok = false;
            std::cout << boost::format(
                             "[WARN] Sample count mismatch: device 0 = %zu, "
                             "device %u = %zu\n")
                             % devs[0].total_samples % i % devs[i].total_samples;
        }
    }

    // ------------------------------------------------------------------
    // Write output summary
    // ------------------------------------------------------------------
    std::cout << boost::format("\n[OUTPUT] Files written to %s/\n") % output_dir;
    const size_t bytes_per_samp = (format == "sc16") ? 4 : 8;
    for (size_t i = 0; i < num_devs; ++i) {
        const std::string fname =
            "usrp_" + std::to_string(i) + "_" + serials[i] + ".bin";
        const double mb =
            static_cast<double>(devs[i].total_samples * bytes_per_samp) / (1024.0 * 1024.0);
        std::cout << boost::format("[OUTPUT]   %-35s (%.2f MB)\n") % fname % mb;
    }
    std::cout << "[OUTPUT]   metadata.json" << std::endl;

    // ------------------------------------------------------------------
    // Write metadata JSON
    // ------------------------------------------------------------------
    write_metadata(output_dir,
        capture_ts,
        freq,
        rate,
        gain,
        (bw > 0.0) ? bw : rate,
        duration,
        format,
        clock_source,
        devs,
        pps_deltas_us,
        !skip_drift,
        skip_drift ? 0.0 : drift_obs_s);

    // ------------------------------------------------------------------
    // Final summary
    // ------------------------------------------------------------------
    size_t total_overflows = 0;
    for (const auto& d : devs)
        total_overflows += d.overflow_count;

    double max_drift_ppm = 0.0;
    size_t max_drift_dev = 0;
    for (size_t i = 1; i < num_devs; ++i) {
        if (std::abs(devs[i].drift_ppm) > std::abs(max_drift_ppm)) {
            max_drift_ppm = devs[i].drift_ppm;
            max_drift_dev = i;
        }
    }

    const bool overall_pass = (total_overflows == 0) && ts_match && count_ok;

    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << boost::format("  Devices:      %u\n") % num_devs;
    std::cout << boost::format("  Samples/dev:  %zu\n") % devs[0].total_samples;
    std::cout << boost::format("  Duration:     %.3f s\n") % duration;
    std::cout << boost::format("  Overflows:    %zu total\n") % total_overflows;
    std::cout << boost::format("  Time sync:    %s (delta: %.3f µs)\n")
                     % (max_delta_us < 1.0 ? "PASS" : "WARN") % max_delta_us;
    if (!skip_drift && num_devs > 1) {
        std::cout << boost::format("  Drift:        %+.3f ppm (dev%u vs dev0)\n")
                         % max_drift_ppm % max_drift_dev;
    }
    std::cout << boost::format("  Status:       %s\n")
                     % (overall_pass ? "SUCCESS" : "WARNING — see messages above");

    return EXIT_SUCCESS;
}
