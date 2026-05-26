//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "bonded_usrp.hpp"
#include <uhd/exception.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/utils/log.hpp>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

namespace uhd { namespace usrp { namespace bonded {

namespace {

// Poll ref_locked sensor on every mboard until all are locked or timeout expires.
// Returns true if all mboards reported locked; false if timeout was reached first.
// Mboards that do not have a ref_locked sensor are treated as always-locked.
static bool wait_for_ref_locked(
    uhd::usrp::multi_usrp::sptr usrp, double timeout_seconds)
{
    const size_t n = usrp->get_num_mboards();
    const auto deadline = std::chrono::steady_clock::now()
                          + std::chrono::duration<double>(timeout_seconds);

    while (std::chrono::steady_clock::now() < deadline) {
        bool all_locked = true;
        for (size_t m = 0; m < n; m++) {
            try {
                const auto sensor = usrp->get_mboard_sensor("ref_locked", m);
                if (!sensor.to_bool()) {
                    all_locked = false;
                    break;
                }
            } catch (const uhd::lookup_error&) {
                // Sensor not present on this mboard — assume locked.
            }
        }
        if (all_locked) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

} // anonymous namespace

void setup_bonded_sync(
    uhd::usrp::multi_usrp::sptr usrp, const uhd::device_addr_t& args)
{
    const size_t n = usrp->get_num_mboards();

    // --- Parse user arguments (all have sensible defaults) ---
    const std::string clock_src =
        args.has_key("sync_clock_source") ? args["sync_clock_source"] : "internal";
    const std::string time_src =
        args.has_key("sync_time_source") ? args["sync_time_source"] : "external";
    const bool strict = args.has_key("sync_strict")
                        && (args["sync_strict"] == "true" || args["sync_strict"] == "1");
    const double lock_timeout =
        args.has_key("sync_lock_timeout") ? std::stod(args["sync_lock_timeout"]) : 5.0;

    UHD_LOGGER_INFO("BONDED_SYNC")
        << "Bonded sync setup: " << n << " mboard(s), "
        << "clock_source=" << clock_src << ", time_source=" << time_src;

    // --- Step 1: apply clock and time sources to every mboard ---
    for (size_t m = 0; m < n; m++) {
        usrp->set_clock_source(clock_src, m);
        usrp->set_time_source(time_src, m);
    }

    // --- Step 2: wait for external reference lock if needed ---
    if (clock_src == "external" || clock_src == "gpsdo") {
        UHD_LOGGER_INFO("BONDED_SYNC")
            << "Waiting up to " << lock_timeout << " s for reference lock...";

        if (!wait_for_ref_locked(usrp, lock_timeout)) {
            const std::string msg =
                "Bonded sync: external reference not locked within "
                + std::to_string(static_cast<int>(lock_timeout))
                + " seconds. Check 10 MHz cable connections.";
            if (strict) {
                throw uhd::runtime_error(msg);
            }
            UHD_LOGGER_WARNING("BONDED_SYNC")
                << msg << " Continuing without confirmed lock.";
        } else {
            UHD_LOGGER_INFO("BONDED_SYNC") << "Reference locked on all mboards.";
        }
    }

    // Time alignment is intentionally NOT performed here.
    // Calling set_rx_rate() and get_rx_stream() after make() can trigger master
    // clock rate changes on B2xx devices, which reset the on-board time counter
    // and invalidate any earlier PPS alignment.
    // Call set_time_unknown_pps() (for external/gpsdo) or set_time_now()
    // (for internal) AFTER completing all hardware configuration.
    UHD_LOGGER_INFO("BONDED_SYNC")
        << "Clock/time sources configured. Time alignment must be performed "
           "after hardware setup (rate, freq, gain, streamers).";
}

void setup_multi_device_sync(
    const std::vector<uhd::usrp::multi_usrp::sptr>& devices,
    const uhd::device_addr_t& args)
{
    UHD_LOGGER_INFO("BONDED_SYNC")
        << "Multi-device sync setup for " << devices.size() << " device(s).";

    for (size_t i = 0; i < devices.size(); i++) {
        UHD_LOGGER_INFO("BONDED_SYNC") << "  Configuring device " << i << "...";
        setup_bonded_sync(devices[i], args);
    }
}

}}} // namespace uhd::usrp::bonded
