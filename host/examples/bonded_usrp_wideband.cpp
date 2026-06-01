//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//
// bonded_usrp_wideband — Phase 5 wideband spectrum capture and stitching.
//
// Captures a simultaneous IQ burst from N B210 devices tuned to adjacent bands,
// computes a per-device power spectrum via windowed FFT, stitches all bands
// into a single wideband spectrum using overlap/crossfade, and reports
// seam-quality metrics (power discontinuity at each band boundary).
//
// Typical invocation (4 B210s, shared 10 MHz + PPS, 30 MHz aggregate at 100 MHz):
//   bonded_usrp_wideband
//     --args "sync_clock_source=external,sync_time_source=external,serial0=30B56D6,serial1=30DBC3C,serial2=30DBC3D,serial3=30EDB63"
//     --rate 10e6 --plan-center-freq 100e6 --plan-overlap 0.10 --fft-size 1024 --save spectra.csv
//

#include "../lib/usrp/bonded/bonded_receiver.hpp"
#include "../lib/usrp/bonded/frequency_plan.hpp"
#include "../lib/usrp/bonded/spectrum_stitcher.hpp"

#include <uhd/types/device_addr.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace po = boost::program_options;

// ---------------------------------------------------------------------------
// Helpers
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
        if (!addr.has_key(key)) break;
        serials.push_back(addr[key]);
    }
    return serials;
}

static std::vector<double> parse_freq_plan(const uhd::device_addr_t& addr)
{
    std::vector<double> freqs;
    if (addr.has_key("freq0")) {
        for (size_t i = 0;; i++) {
            const std::string key = "freq" + std::to_string(i);
            if (!addr.has_key(key)) break;
            freqs.push_back(std::stod(addr[key]));
        }
        return freqs;
    }
    if (addr.has_key("freq_plan")) {
        std::stringstream ss(addr["freq_plan"]);
        std::string token;
        while (std::getline(ss, token, ',')) {
            if (!token.empty()) freqs.push_back(std::stod(token));
        }
    }
    return freqs;
}

// Build a Hann window of length N.
static std::vector<float> hann_window(size_t N)
{
    static constexpr double kTwoPi = 6.28318530717958647692;
    std::vector<float> w(N);
    for (size_t i = 0; i < N; i++) {
        w[i] = static_cast<float>(
            0.5 * (1.0 - std::cos(kTwoPi * i / static_cast<double>(N - 1))));
    }
    return w;
}

// Compute power spectrum (dBFS) for one channel's IQ buffer by averaging
// nfft-point Hann-windowed FFTs over non-overlapping segments.
// Returns nfft bins ordered [DC, +BW/2) i.e. centred-frequency after fftshift.
static std::vector<float> power_spectrum_dbfs(
    const std::vector<std::complex<float>>& iq, size_t nfft)
{
    static constexpr double kTwoPi = 6.28318530717958647692;

    const size_t nsamps  = iq.size();
    const size_t nsegs   = nsamps / nfft;

    if (nsegs == 0 || nfft < 2) {
        return std::vector<float>(nfft, -120.0f);
    }

    const auto win = hann_window(nfft);
    // coherent power correction for Hann window
    double win_power = 0.0;
    for (auto w : win) win_power += static_cast<double>(w) * w;

    std::vector<double> accum(nfft, 0.0);

    for (size_t seg = 0; seg < nsegs; seg++) {
        // Apply window then DFT (direct implementation, O(N²) but fine for N≤8192)
        const size_t base = seg * nfft;
        for (size_t k = 0; k < nfft; k++) {
            double re = 0.0, im = 0.0;
            for (size_t n = 0; n < nfft; n++) {
                const double phase = kTwoPi * k * n / static_cast<double>(nfft);
                const auto s = iq[base + n];
                const double ws = win[n];
                re += ws * static_cast<double>(s.real()) * std::cos(phase)
                    + ws * static_cast<double>(s.imag()) * std::sin(phase);
                im += ws * static_cast<double>(s.imag()) * std::cos(phase)
                    - ws * static_cast<double>(s.real()) * std::sin(phase);
            }
            accum[k] += (re * re + im * im);
        }
    }

    // Average, normalise, convert to dBFS
    std::vector<float> psd(nfft);
    for (size_t k = 0; k < nfft; k++) {
        double pwr = accum[k] / (static_cast<double>(nsegs) * win_power);
        psd[k]     = static_cast<float>(10.0 * std::log10(pwr + 1e-30));
    }

    // fftshift: move negative-freq half to the left
    std::rotate(psd.begin(), psd.begin() + nfft / 2, psd.end());

    return psd;
}

// Measure seam discontinuity: compare the last margin_bins of band d against
// the first margin_bins of band d+1 in the stitched spectrum at position p.
static double seam_discontinuity_db(const std::vector<float>& stitched,
    size_t seam_pos,
    size_t margin_bins)
{
    if (seam_pos < margin_bins || seam_pos + margin_bins > stitched.size()) {
        return 0.0;
    }
    double left = 0.0, right = 0.0;
    for (size_t k = 0; k < margin_bins; k++) {
        left  += stitched[seam_pos - margin_bins + k];
        right += stitched[seam_pos + k];
    }
    left /= margin_bins;
    right /= margin_bins;
    return std::abs(left - right);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    std::string args, save_path;
    double rate, freq, gain, delay, plan_center, plan_overlap;
    size_t nsamps, nfft;
    bool calibrate;

    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help,h", "Show this help message.")
        ("args,a",
            po::value<std::string>(&args)->default_value(
                "serial0=30B56D6,serial1=30DBC3C,serial2=30DBC3D,serial3=30EDB63,"
                "sync_clock_source=external,sync_time_source=external"),
            "UHD device address arguments.\n"
            "Must include serialN= keys for each device.\n"
            "Optional frequency-plan keys: freq0,freq1,... or freq_plan=F0,F1,...\n")
        ("rate,r", po::value<double>(&rate)->default_value(10e6),
            "Per-device RX sample rate (Sps).")
        ("freq,f", po::value<double>(&freq)->default_value(100e6),
            "Uniform center frequency if no freq plan is used (Hz).")
        ("plan-center-freq", po::value<double>(&plan_center),
            "Aggregate center frequency for auto adjacent-band frequency plan (Hz).")
        ("plan-overlap", po::value<double>(&plan_overlap)->default_value(0.10),
            "Overlap fraction for auto frequency plan [0,1).")
        ("gain,g", po::value<double>(&gain)->default_value(40.0),
            "RX gain (dB).")
        ("nsamps,n", po::value<size_t>(&nsamps)->default_value(131072),
            "Samples per channel per device for spectrum averaging.")
        ("fft-size", po::value<size_t>(&nfft)->default_value(1024),
            "FFT size for power spectrum computation (power of 2 recommended).")
        ("delay,d", po::value<double>(&delay)->default_value(1.5),
            "Burst start delay (seconds).")
        ("save,s", po::value<std::string>(&save_path),
            "Optional: save stitched spectrum and per-device spectra to CSV file.")
        ("calibrate,c", po::bool_switch(&calibrate)->default_value(false),
            "Normalise each device's mean noise floor to the global mean before stitching.\n"
            "Eliminates inter-device gain offsets so seam discontinuity reflects\n"
            "only DSP artefacts, not RF gain variation.")
    ;
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
        std::cout
            << "bonded_usrp_wideband — Phase 5 wideband spectrum capture and stitching\n\n"
            << desc << std::endl;
        return EXIT_SUCCESS;
    }
    po::notify(vm);

    const uhd::device_addr_t dev_addr(args);
    const std::vector<std::string> serials = parse_serial_list(dev_addr);
    std::vector<double> freq_plan          = parse_freq_plan(dev_addr);

    if (serials.empty()) {
        std::cerr << "[bonded_usrp_wideband] ERROR: no serialN= keys found in --args.\n";
        return EXIT_FAILURE;
    }
    if (!freq_plan.empty() && freq_plan.size() != serials.size()) {
        std::cerr << "[bonded_usrp_wideband] ERROR: freq plan size ("
                  << freq_plan.size() << ") does not match serial count ("
                  << serials.size() << ").\n";
        return EXIT_FAILURE;
    }
    if (freq_plan.empty() && vm.count("plan-center-freq")) {
        auto plan = uhd::usrp::bonded::make_adjacent_frequency_plan(
            serials.size(), plan_center, rate, plan_overlap);
        freq_plan = plan.centers_hz;
        std::cout << boost::format(
                         "[bonded_usrp_wideband] Auto freq plan: center=%.3f MHz, "
                         "step=%.3f MHz, overlap=%.1f%%\n")
                         % (plan_center / 1e6) % (plan.step_hz / 1e6)
                         % (plan_overlap * 100.0);
        for (size_t i = 0; i < freq_plan.size(); i++) {
            std::cout << boost::format("  f%u = %.3f MHz\n") % i % (freq_plan[i] / 1e6);
        }
    }

    if (nsamps < nfft) {
        std::cerr << "[bonded_usrp_wideband] ERROR: nsamps must be >= fft-size.\n";
        return EXIT_FAILURE;
    }

    // -----------------------------------------------------------------------
    // Configure and capture
    // -----------------------------------------------------------------------
    std::cout << boost::format(
                     "\n[bonded_usrp_wideband] Found %zu device serial(s): ")
                     % serials.size();
    for (const auto& s : serials) std::cout << s << " ";
    std::cout << "\n";

    uhd::usrp::bonded::bonded_receiver::config cfg;
    cfg.serials      = serials;
    cfg.clock_source = dev_addr.cast<std::string>("sync_clock_source", "external");
    cfg.time_source  = dev_addr.cast<std::string>("sync_time_source", "external");
    cfg.rate         = rate;
    cfg.freq         = freq;
    cfg.freq_plan    = freq_plan;
    cfg.gain         = gain;
    cfg.strict       = dev_addr.has_key("sync_strict");
    cfg.lock_timeout = std::stod(dev_addr.cast<std::string>("sync_lock_timeout", "5.0"));

    uhd::usrp::bonded::bonded_receiver rx(cfg);
    rx.configure();
    auto result = rx.capture_burst(nsamps, delay);

    if (!result.aligned) {
        std::cerr << "[bonded_usrp_wideband] FAIL: " << result.error_message << "\n";
        return EXIT_FAILURE;
    }

    const size_t num_dev = serials.size();
    std::cout << boost::format(
                     "\n[bonded_usrp_wideband] Burst captured: spread=%.3f us  [%s]\n")
                     % result.inter_device_spread_us
                     % (result.aligned ? "aligned" : "MISALIGNED");

    // -----------------------------------------------------------------------
    // Compute per-device power spectra (channel 0 of each device)
    // -----------------------------------------------------------------------
    std::cout << "\n[bonded_usrp_wideband] Computing power spectra ...\n";

    const size_t overlap_bins =
        freq_plan.empty()
            ? 0
            : static_cast<size_t>(std::llround(plan_overlap * static_cast<double>(nfft)));

    std::vector<std::vector<float>> spectra(num_dev);
    std::vector<float> dev_mean(num_dev, 0.0f);
    for (size_t d = 0; d < num_dev; d++) {
        spectra[d]  = power_spectrum_dbfs(result.data[d][0], nfft);
        double sum  = 0.0;
        for (float v : spectra[d]) sum += v;
        dev_mean[d] = static_cast<float>(sum / static_cast<double>(spectra[d].size()));
        const double fc = freq_plan.empty() ? freq : freq_plan[d];
        const float pmin = *std::min_element(spectra[d].begin(), spectra[d].end());
        const float pmax = *std::max_element(spectra[d].begin(), spectra[d].end());
        std::cout << boost::format(
                         "  Device %u  fc=%.3f MHz  min=%.1f dBFS  max=%.1f dBFS  mean=%.1f dBFS\n")
                         % d % (fc / 1e6) % pmin % pmax % dev_mean[d];
    }

    // -----------------------------------------------------------------------
    // Optional per-device gain calibration
    // -----------------------------------------------------------------------
    if (calibrate) {
        float global_mean = 0.0f;
        for (float m : dev_mean) global_mean += m;
        global_mean /= static_cast<float>(num_dev);
        std::cout << boost::format(
                         "\n[bonded_usrp_wideband] Calibrating: global mean=%.2f dBFS\n")
                         % global_mean;
        for (size_t d = 0; d < num_dev; d++) {
            const float offset = global_mean - dev_mean[d];
            for (float& v : spectra[d]) v += offset;
            std::cout << boost::format(
                             "  Device %u  offset=%.2f dB (%.2f → %.2f dBFS mean)\n")
                             % d % offset % dev_mean[d] % (dev_mean[d] + offset);
        }
    }

    // -----------------------------------------------------------------------
    // Stitch bands
    // -----------------------------------------------------------------------
    std::cout << boost::format(
                     "\n[bonded_usrp_wideband] Stitching %zu bands, overlap=%zu bins\n")
                     % num_dev % overlap_bins;

    const std::vector<float> stitched =
        uhd::usrp::bonded::stitch_power_spectra(spectra, overlap_bins);

    // -----------------------------------------------------------------------
    // Seam quality metrics
    // -----------------------------------------------------------------------
    const size_t margin    = std::max(size_t{4}, overlap_bins / 4);
    const size_t step_bins = nfft - overlap_bins;
    bool seam_pass         = true;

    std::cout << "\n[bonded_usrp_wideband] Seam quality:\n";
    for (size_t d = 0; d + 1 < num_dev; d++) {
        const size_t seam_pos = (d + 1) * step_bins;
        const double disc     = seam_discontinuity_db(stitched, seam_pos, margin);
        const bool ok         = disc < 3.0;  // 3 dB threshold; 1 dB ideal
        if (!ok) seam_pass = false;
        std::cout << boost::format(
                         "  Seam %u→%u  pos=%zu  discontinuity=%.2f dB  [%s]\n")
                         % d % (d + 1) % seam_pos % disc % (ok ? "OK" : "FAIL");
    }

    const double stitched_bw_mhz =
        freq_plan.empty()
            ? rate / 1e6
            : (freq_plan.back() - freq_plan.front() + rate) / 1e6;
    std::cout << boost::format(
                     "\n[bonded_usrp_wideband] Stitched spectrum: %zu bins spanning "
                     "approx %.1f MHz\n")
                     % stitched.size() % stitched_bw_mhz;

    // -----------------------------------------------------------------------
    // Optional CSV save
    // -----------------------------------------------------------------------
    if (vm.count("save")) {
        std::ofstream f(save_path);
        if (!f.is_open()) {
            std::cerr << "[bonded_usrp_wideband] WARNING: cannot open " << save_path
                      << " for writing.\n";
        } else {
            // Header
            f << "bin,stitched_dbfs";
            for (size_t d = 0; d < num_dev; d++) {
                f << ",dev" << d << "_dbfs";
            }
            f << "\n";

            const size_t nrows = stitched.size();
            for (size_t k = 0; k < nrows; k++) {
                f << k << "," << stitched[k];
                // per-device spectra are shorter; pad with empty for out-of-range
                for (size_t d = 0; d < num_dev; d++) {
                    if (k < spectra[d].size()) {
                        f << "," << spectra[d][k];
                    } else {
                        f << ",";
                    }
                }
                f << "\n";
            }
            std::cout << boost::format(
                             "[bonded_usrp_wideband] Saved spectrum to %s\n")
                             % save_path;
        }
    }

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    const bool overall_pass = result.aligned && seam_pass;
    std::cout << "\n[bonded_usrp_wideband] Result: " << (overall_pass ? "PASS" : "FAIL")
              << "\n";

    return overall_pass ? EXIT_SUCCESS : EXIT_FAILURE;
}
