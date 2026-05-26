//
// Copyright 2026 Gabriel Lando
// SPDX-License-Identifier: GPL-3.0-or-later
//

#pragma once

#include <uhd/types/device_addr.hpp>
#include <uhd/usrp/multi_usrp.hpp>

namespace uhd { namespace usrp { namespace bonded {

/*!
 * Apply clock and time synchronization to a multi_usrp that was created with
 * bonded=true in the device arguments.
 *
 * Reads the following optional keys from \p args:
 *   sync_clock_source  — "internal" (default), "external", or "gpsdo"
 *   sync_time_source   — "external" (default), "internal", or "gpsdo"
 *   sync_strict        — "true" / "1": throw on lock/PPS failure instead of warning
 *   sync_lock_timeout  — seconds to wait for ref_locked (default: 5.0)
 *
 * Steps performed:
 *   1. set_clock_source + set_time_source on every mboard
 *   2. Wait for ref_locked when clock_source is external or gpsdo
 *
 * NOTE: Time alignment (set_time_unknown_pps / set_time_now) must be called by the
 * application AFTER all hardware setup (rate, freq, gain, streamers) is complete,
 * because B2xx master clock rate changes during setup reset the on-board counter.
 *
 * \param usrp   multi_usrp already created with the correct number of mboards
 * \param args   original device address arguments passed to multi_usrp::make()
 * \throws uhd::runtime_error if sync_strict=true and lock cannot be confirmed
 */
void setup_bonded_sync(
    uhd::usrp::multi_usrp::sptr usrp, const uhd::device_addr_t& args);

/*!
 * Convenience overload for multi-device bonding: applies clock/time source
 * configuration and waits for reference lock on every device in \p devices.
 *
 * This is the correct path for USB-based devices (e.g. B210) where each
 * physical unit must be opened as a separate multi_usrp instance.
 *
 * Each element of \p devices is configured identically using \p args.
 * The same sync_clock_source / sync_time_source / sync_strict /
 * sync_lock_timeout keys are honoured as in the single-device overload.
 *
 * Time alignment across devices must still be performed by the caller after
 * hardware setup, using set_time_unknown_pps() on each device individually
 * (the PPS edge is shared, so all boards snap to the same edge).
 *
 * \param devices  vector of per-device multi_usrp::sptr (one per physical B210)
 * \param args     original device address arguments
 */
void setup_multi_device_sync(
    const std::vector<uhd::usrp::multi_usrp::sptr>& devices,
    const uhd::device_addr_t& args);

}}} // namespace uhd::usrp::bonded
