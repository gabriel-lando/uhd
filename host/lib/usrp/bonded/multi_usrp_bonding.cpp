// Copyright 2026 Gabriel
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Implementation of multi_usrp::make_bonded_usb().
// Opens N USB USRP devices independently and synchronises their hardware
// clocks to a common PPS edge — the same strategy used by sync_validation_test.

#include <uhd/exception.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/log.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace uhd { namespace usrp {

std::vector<multi_usrp::sptr> multi_usrp::make_bonded_usb(
    const std::vector<std::string>& serials,
    const std::string& clock_source,
    const std::string& time_source)
{
    if (serials.size() < 2) {
        throw uhd::runtime_error(
            "make_bonded_usb: at least 2 serial numbers are required");
    }

    // -----------------------------------------------------------------------
    // Open every device independently.  USB devices cannot be aggregated in a
    // single make(addr0=...,addr1=...) call the way Ethernet devices can.
    // -----------------------------------------------------------------------
    std::vector<sptr> devices;
    devices.reserve(serials.size());

    for (size_t i = 0; i < serials.size(); ++i) {
        UHD_LOG_DEBUG("BONDED",
            "Opening device " << i << " (serial=" << serials[i] << ")");
        try {
            devices.push_back(make(device_addr_t("serial=" + serials[i])));
        } catch (const std::exception& e) {
            throw uhd::runtime_error(
                std::string("make_bonded_usb: failed to open device ")
                + std::to_string(i) + " (serial=" + serials[i]
                + "): " + e.what());
        }

        // Configure sync sources right after opening so the hardware settles.
        devices.back()->set_clock_source(clock_source, 0);
        devices.back()->set_time_source(time_source, 0);
    }

    // -----------------------------------------------------------------------
    // Wait for a PPS edge on device 0 (the reference board).
    // We poll get_time_last_pps(0) until it advances.
    // This costs at most one PPS interval (~1 second).
    // -----------------------------------------------------------------------
    UHD_LOG_DEBUG("BONDED", "Waiting for PPS edge on reference device...");

    const auto pps_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    auto last_pps = devices[0]->get_time_last_pps(0);

    while (devices[0]->get_time_last_pps(0) == last_pps) {
        if (std::chrono::steady_clock::now() > pps_deadline) {
            throw uhd::runtime_error(
                "make_bonded_usb: no PPS detected on device 0 within 3 seconds. "
                "Check GPS module and SMA cable to the PPS input.");
        }
        std::this_thread::sleep_for(10ms);
    }

    // -----------------------------------------------------------------------
    // A PPS edge just fired on device 0.  We now have ~1 second to call
    // set_time_next_pps(0.0) on every device before the NEXT edge arrives.
    // All devices share the same physical PPS signal, so they will all latch
    // time = 0 on the same next edge, giving sub-microsecond alignment.
    //
    // IMPORTANT: do NOT call set_time_unknown_pps() sequentially — it blocks
    // ~2 s per device (waits internally for a whole PPS interval), causing
    // each device to latch on a different edge and end up exactly 1 s apart.
    // -----------------------------------------------------------------------
    UHD_LOG_DEBUG("BONDED",
        "PPS edge detected — scheduling time latch on "
            << devices.size() << " device(s)");

    for (auto& dev : devices) {
        dev->set_time_next_pps(time_spec_t(0.0));
    }

    // Wait for the next PPS edge to fire and latch the scheduled time.
    // 1.1 s is sufficient; the extra 0.1 s provides margin for slow USB hosts.
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    // -----------------------------------------------------------------------
    // Sanity-check alignment: all devices should report the same last-PPS time.
    // -----------------------------------------------------------------------
    const double t0 = devices[0]->get_time_last_pps(0).get_real_secs();
    for (size_t i = 1; i < devices.size(); ++i) {
        const double ti      = devices[i]->get_time_last_pps(0).get_real_secs();
        const double delta_us = std::abs(ti - t0) * 1e6;
        if (delta_us > 1000.0) {
            UHD_LOG_WARNING("BONDED",
                "Device " << i << " time delta = " << delta_us
                          << " µs — PPS sync may have failed. "
                          "Ensure all devices share exactly one PPS source.");
        } else {
            UHD_LOG_DEBUG("BONDED",
                "Device " << i << ": time delta = " << delta_us << " µs — OK");
        }
    }

    return devices;
}

}} // namespace uhd::usrp
