//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/exception.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/stream.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/log.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

static constexpr const char* BONDED_TYPE_KEY   = "type";
static constexpr const char* BONDED_TYPE_VALUE = "bonded_b200";

static size_t require_size_t(const uhd::device_addr_t& args,
    const std::string& key,
    const size_t min_value,
    const size_t max_value)
{
    if (!args.has_key(key)) {
        throw uhd::value_error("Missing required key: " + key);
    }
    const auto value = args.cast<size_t>(key, 0);
    if (value < min_value || value > max_value) {
        throw uhd::value_error("Invalid value for " + key + ": " + args[key]);
    }
    return value;
}

static std::string get_or_default(
    const uhd::device_addr_t& args, const std::string& key, const std::string& def)
{
    return args.has_key(key) ? args[key] : def;
}

static double get_or_default_double(
    const uhd::device_addr_t& args, const std::string& key, const double def)
{
    return args.has_key(key) ? args.cast<double>(key, def) : def;
}

static uhd::device_addr_t require_device_addr_string(
    const uhd::device_addr_t& args, const std::string& key)
{
    if (!args.has_key(key)) {
        throw uhd::value_error("Missing required key: " + key);
    }
    return uhd::device_addr_t(args[key]);
}

static std::vector<std::string> split_ws_list(const std::string& in)
{
    std::istringstream iss(in);
    std::vector<std::string> out;
    std::string tok;
    while (iss >> tok) {
        out.push_back(tok);
    }
    return out;
}

static uhd::device_addr_t make_member_args_from_indexed_keys(
    const uhd::device_addr_t& args, const size_t index)
{
    const std::string idx = std::to_string(index);
    const std::string serial_key = "bonded_serial" + idx;
    const std::string type_key   = "bonded_type" + idx;

    if (!args.has_key(serial_key)) {
        throw uhd::value_error("Missing required key: " + serial_key
                               + " (use bonded_serials or bonded_serialN)");
    }

    uhd::device_addr_t dev_args;
    dev_args["serial"] = args[serial_key];
    dev_args["type"]   = args.has_key(type_key) ? args[type_key] : "b200";
    return dev_args;
}

static std::vector<size_t> default_channels_all(const size_t n)
{
    std::vector<size_t> ch;
    ch.reserve(n);
    for (size_t i = 0; i < n; i++) {
        ch.push_back(i);
    }
    return ch;
}

static std::vector<double> compute_subband_centers(
    const double center_freq, const double total_bw, const double guard_bw, const size_t n)
{
    if (n == 0) {
        return {};
    }
    if (!(total_bw > 0.0)) {
        throw uhd::value_error("bonded_total_bw must be > 0");
    }
    if (guard_bw < 0.0) {
        throw uhd::value_error("bonded_guard_bw must be >= 0");
    }
    const double b_dev = (total_bw + (static_cast<double>(n) - 1.0) * guard_bw)
                         / static_cast<double>(n);
    const double step = b_dev - guard_bw;
    const double mid  = (static_cast<double>(n) - 1.0) / 2.0;

    std::vector<double> freqs;
    freqs.reserve(n);
    for (size_t i = 0; i < n; i++) {
        freqs.push_back(center_freq + (static_cast<double>(i) - mid) * step);
    }
    return freqs;
}

class bonded_rx_streamer final : public uhd::rx_streamer
{
public:
    bonded_rx_streamer(std::vector<uhd::rx_streamer::sptr> streams,
        const std::vector<size_t>& virt_channels)
        : _streams(std::move(streams))
        , _virt_channels(virt_channels)
    {
        if (_streams.size() != _virt_channels.size()) {
            throw uhd::value_error("bonded_rx_streamer: streams/channels mismatch");
        }
        if (_streams.empty()) {
            throw uhd::value_error("bonded_rx_streamer: no streams");
        }
    }

    size_t get_num_channels(void) const override
    {
        return _streams.size();
    }

    size_t get_max_num_samps(void) const override
    {
        size_t max_samps = _streams.front()->get_max_num_samps();
        for (const auto& s : _streams) {
            max_samps = std::min(max_samps, s->get_max_num_samps());
        }
        return max_samps;
    }

    size_t recv(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        uhd::rx_metadata_t& metadata,
        const double timeout,
        const bool one_packet) override
    {
        if (buffs.size() != get_num_channels()) {
            metadata.error_code = uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT;
            return 0;
        }

        // Receive one channel per underlying USRP streamer.
        // We reconcile metadata: timestamps must match if present.
        uhd::rx_metadata_t md0;
        size_t min_nsamps = std::numeric_limits<size_t>::max();

        for (size_t i = 0; i < _streams.size(); i++) {
            uhd::rx_metadata_t mdi;
            uhd::rx_streamer::buffs_type one_buff(&buffs[i], 1);
            const size_t n = _streams[i]->recv(one_buff, nsamps_per_buff, mdi, timeout, one_packet);
            min_nsamps      = std::min(min_nsamps, n);

            if (i == 0) {
                md0 = mdi;
            } else {
                const bool ts_mismatch = (md0.has_time_spec && mdi.has_time_spec
                                             && (md0.time_spec != mdi.time_spec))
                                         || (md0.has_time_spec != mdi.has_time_spec);
                if (ts_mismatch) {
                    metadata = md0;
                    metadata.error_code = uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT;
                    return 0;
                }
                if (mdi.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE
                    && md0.error_code == uhd::rx_metadata_t::ERROR_CODE_NONE) {
                    md0 = mdi;
                }
            }
        }

        metadata = md0;
        if (min_nsamps == std::numeric_limits<size_t>::max()) {
            return 0;
        }
        return min_nsamps;
    }

    void issue_stream_cmd(const uhd::stream_cmd_t& stream_cmd) override
    {
        for (auto& s : _streams) {
            s->issue_stream_cmd(stream_cmd);
        }
    }

    void post_input_action(
        const std::shared_ptr<uhd::rfnoc::action_info>& action, const size_t port) override
    {
        for (auto& s : _streams) {
            s->post_input_action(action, port);
        }
    }

private:
    std::vector<uhd::rx_streamer::sptr> _streams;
    std::vector<size_t> _virt_channels;
};

class bonded_tx_streamer final : public uhd::tx_streamer
{
public:
    bonded_tx_streamer(std::vector<uhd::tx_streamer::sptr> streams,
        const std::vector<size_t>& virt_channels)
        : _streams(std::move(streams))
        , _virt_channels(virt_channels)
    {
        if (_streams.size() != _virt_channels.size()) {
            throw uhd::value_error("bonded_tx_streamer: streams/channels mismatch");
        }
        if (_streams.empty()) {
            throw uhd::value_error("bonded_tx_streamer: no streams");
        }
    }

    size_t get_num_channels(void) const override
    {
        return _streams.size();
    }

    size_t get_max_num_samps(void) const override
    {
        size_t max_samps = _streams.front()->get_max_num_samps();
        for (const auto& s : _streams) {
            max_samps = std::min(max_samps, s->get_max_num_samps());
        }
        return max_samps;
    }

    size_t send(const buffs_type& buffs,
        const size_t nsamps_per_buff,
        const uhd::tx_metadata_t& metadata,
        const double timeout) override
    {
        if (buffs.size() != get_num_channels()) {
            return 0;
        }
        size_t min_nsamps = std::numeric_limits<size_t>::max();
        for (size_t i = 0; i < _streams.size(); i++) {
            uhd::tx_streamer::buffs_type one_buff(&buffs[i], 1);
            const size_t n = _streams[i]->send(one_buff, nsamps_per_buff, metadata, timeout);
            min_nsamps      = std::min(min_nsamps, n);
        }
        if (min_nsamps == std::numeric_limits<size_t>::max()) {
            return 0;
        }
        return min_nsamps;
    }

    bool recv_async_msg(uhd::async_metadata_t& async_metadata, double timeout) override
    {
        // Poll streamers in order and return first available message.
        for (auto& s : _streams) {
            if (s->recv_async_msg(async_metadata, timeout)) {
                return true;
            }
        }
        return false;
    }

    void post_output_action(
        const std::shared_ptr<uhd::rfnoc::action_info>& action, const size_t port) override
    {
        for (auto& s : _streams) {
            s->post_output_action(action, port);
        }
    }

private:
    std::vector<uhd::tx_streamer::sptr> _streams;
    std::vector<size_t> _virt_channels;
};

class bonded_multi_usrp final : public uhd::usrp::multi_usrp
{
public:
    explicit bonded_multi_usrp(const uhd::device_addr_t& bonded_args)
    {
        _bonded_args = bonded_args;
        _n           = require_size_t(_bonded_args, "bonded_num", 1, 32);

        _clock_source = get_or_default(_bonded_args, "bonded_clock_source", "external");
        _time_source  = get_or_default(_bonded_args, "bonded_time_source", "external");

        _aggregate_center_freq = get_or_default_double(_bonded_args, "bonded_center_freq", 0.0);
        _aggregate_total_bw    = get_or_default_double(_bonded_args, "bonded_total_bw", 0.0);
        _guard_bw              = get_or_default_double(_bonded_args, "bonded_guard_bw", 0.0);

        // Create underlying USRPs
        // NOTE: UHD's string-form args parser does not allow nested args strings
        // like bonded_args0=type=b200,serial=XXXX (it would contain '=' and ',').
        // Therefore, we support two string-friendly encodings:
        // - bonded_serials="SER0 SER1 ..." (space-separated)
        // - bonded_serial0=SER0,bonded_serial1=SER1,... (and optional bonded_typeN)
        _usrps.reserve(_n);
        if (_bonded_args.has_key("bonded_serials")) {
            const auto serials = split_ws_list(_bonded_args["bonded_serials"]);
            if (serials.size() != _n) {
                throw uhd::value_error("bonded_serials must contain exactly bonded_num serials");
            }
            for (size_t i = 0; i < _n; i++) {
                uhd::device_addr_t dev_args;
                dev_args["type"]   = "b200";
                dev_args["serial"] = serials[i];
                _usrps.push_back(uhd::usrp::multi_usrp::make(dev_args));
            }
        } else if (_bonded_args.has_key("bonded_serial0")) {
            for (size_t i = 0; i < _n; i++) {
                uhd::device_addr_t dev_args = make_member_args_from_indexed_keys(_bonded_args, i);
                if (dev_args.has_key(BONDED_TYPE_KEY)
                    && dev_args[BONDED_TYPE_KEY] == BONDED_TYPE_VALUE) {
                    throw uhd::value_error("bonded_type" + std::to_string(i)
                                           + " must not be bonded_b200 (would recurse)");
                }
                _usrps.push_back(uhd::usrp::multi_usrp::make(dev_args));
            }
        } else {
            // Backward-compatible path for programmatic construction where bonded_argsN
            // values can be set without going through the string parser.
            for (size_t i = 0; i < _n; i++) {
                const std::string key = "bonded_args" + std::to_string(i);
                uhd::device_addr_t dev_args;
                try {
                    dev_args = require_device_addr_string(_bonded_args, key);
                } catch (const uhd::value_error&) {
                    throw uhd::value_error(
                        "Missing bonded member specification. Use bonded_serials=\"SER0 SER1\" "
                        "or bonded_serial0=SER0,bonded_serial1=SER1,... (bonded_argsN is only "
                        "usable when constructing device_addr_t programmatically).");
                }
                if (dev_args.has_key(BONDED_TYPE_KEY)
                    && dev_args[BONDED_TYPE_KEY] == BONDED_TYPE_VALUE) {
                    throw uhd::value_error("bonded_args" + std::to_string(i)
                                           + " must not be type=bonded_b200 (would recurse)");
                }
                _usrps.push_back(uhd::usrp::multi_usrp::make(dev_args));
            }
        }

        // Compute per-device centers if configured
        if (_aggregate_total_bw > 0.0) {
            _subband_centers = compute_subband_centers(
                _aggregate_center_freq, _aggregate_total_bw, _guard_bw, _n);
        } else {
            _subband_centers.assign(_n, _aggregate_center_freq);
        }

        // Initialize sync
        _apply_sync_sources();
        _align_time_next_pps();

        // Apply initial tune if configured
        if (_aggregate_center_freq != 0.0) {
            for (size_t i = 0; i < _n; i++) {
                uhd::tune_request_t tr(_subband_centers[i]);
                _usrps[i]->set_rx_freq(tr, 0);
                _usrps[i]->set_tx_freq(tr, 0);
            }
        }
    }

    ~bonded_multi_usrp(void) override = default;

    // Core plumbing
    uhd::device::sptr get_device(void) override
    {
        return _usrps.front()->get_device();
    }

    uhd::property_tree::sptr get_tree(void) const override
    {
        return _usrps.front()->get_tree();
    }

    uhd::rx_streamer::sptr get_rx_stream(const uhd::stream_args_t& args) override
    {
        uhd::stream_args_t mapped = args;
        if (mapped.channels.empty()) {
            mapped.channels = default_channels_all(_n);
        }

        std::vector<uhd::rx_streamer::sptr> streams;
        std::vector<size_t> virt_channels;
        streams.reserve(mapped.channels.size());
        virt_channels.reserve(mapped.channels.size());

        for (const size_t virt_chan : mapped.channels) {
            if (virt_chan >= _n) {
                throw uhd::index_error("Requested RX channel out of range: "
                                       + std::to_string(virt_chan));
            }
            uhd::stream_args_t one = mapped;
            one.channels           = {0};
            streams.push_back(_usrps[virt_chan]->get_rx_stream(one));
            virt_channels.push_back(virt_chan);
        }

        return std::make_shared<bonded_rx_streamer>(std::move(streams), virt_channels);
    }

    uhd::tx_streamer::sptr get_tx_stream(const uhd::stream_args_t& args) override
    {
        uhd::stream_args_t mapped = args;
        if (mapped.channels.empty()) {
            mapped.channels = default_channels_all(_n);
        }

        std::vector<uhd::tx_streamer::sptr> streams;
        std::vector<size_t> virt_channels;
        streams.reserve(mapped.channels.size());
        virt_channels.reserve(mapped.channels.size());

        for (const size_t virt_chan : mapped.channels) {
            if (virt_chan >= _n) {
                throw uhd::index_error("Requested TX channel out of range: "
                                       + std::to_string(virt_chan));
            }
            uhd::stream_args_t one = mapped;
            one.channels           = {0};
            streams.push_back(_usrps[virt_chan]->get_tx_stream(one));
            virt_channels.push_back(virt_chan);
        }

        return std::make_shared<bonded_tx_streamer>(std::move(streams), virt_channels);
    }

    // Identification
    uhd::dict<std::string, std::string> get_usrp_rx_info(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_usrp_rx_info(0);
    }

    uhd::dict<std::string, std::string> get_usrp_tx_info(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_usrp_tx_info(0);
    }

    // Mboard methods
    void set_master_clock_rate(double rate, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->set_master_clock_rate(rate, 0);
        });
    }

    double get_master_clock_rate(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_master_clock_rate(0);
    }

    uhd::meta_range_t get_master_clock_rate_range(const size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_master_clock_rate_range(0);
    }

    std::string get_pp_string(void) override
    {
        return "bonded_b200: " + std::to_string(_n) + " devices";
    }

    std::string get_mboard_name(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_mboard_name(0);
    }

    uhd::time_spec_t get_time_now(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_time_now(0);
    }

    uhd::time_spec_t get_time_last_pps(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_time_last_pps(0);
    }

    void set_time_now(const uhd::time_spec_t& time_spec, size_t mboard = ALL_MBOARDS) override
    {
        // For multi-device bonding, forwarding set_time_now() sequentially creates skew between
        // devices (host call latency), which then shows up as mismatched RX timestamps.
        //
        // Best-effort behavior: If the caller sets all mboards, schedule the update on the next
        // PPS edge for all devices and wait briefly for PPS to tick. If PPS does not tick,
        // fall back to sequential set_time_now().
        if (mboard != ALL_MBOARDS || _n <= 1) {
            _for_each_mboard(
                mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_time_now(time_spec, 0); });
            return;
        }

        const bool time_source_supports_pps = (_time_source != "internal") && (_time_source != "none");
        if (time_source_supports_pps) {
            std::vector<uhd::time_spec_t> last_pps_before;
            last_pps_before.reserve(_n);
            for (auto& u : _usrps) {
                last_pps_before.push_back(u->get_time_last_pps(0));
            }

            _align_time_next_pps(time_spec);

            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (std::chrono::steady_clock::now() < deadline) {
                bool all_advanced = true;
                for (size_t i = 0; i < _n; i++) {
                    if (_usrps.at(i)->get_time_last_pps(0) == last_pps_before.at(i)) {
                        all_advanced = false;
                        break;
                    }
                }
                if (all_advanced) {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            UHD_LOG_WARNING("BONDED_B200",
                "Timed out waiting for PPS while aligning time; falling back to set_time_now()"
            );
        }

        _for_each_mboard(ALL_MBOARDS,
            [&](uhd::usrp::multi_usrp::sptr& u) { u->set_time_now(time_spec, 0); });
    }

    void set_time_next_pps(
        const uhd::time_spec_t& time_spec, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->set_time_next_pps(time_spec, 0);
        });
    }

    void set_time_unknown_pps(const uhd::time_spec_t& time_spec) override
    {
        for (auto& u : _usrps) {
            u->set_time_unknown_pps(time_spec);
        }
    }

    bool get_time_synchronized(void) override
    {
        // All devices must report synchronized
        for (auto& u : _usrps) {
            if (!u->get_time_synchronized()) {
                return false;
            }
        }
        return true;
    }

    void set_command_time(
        const uhd::time_spec_t& time_spec, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->set_command_time(time_spec, 0);
        });
    }

    void clear_command_time(size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->clear_command_time(0);
        });
    }

    void issue_stream_cmd(const uhd::stream_cmd_t& stream_cmd, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->issue_stream_cmd(stream_cmd, 0);
        });
    }

    void set_time_source(const std::string& source, size_t mboard = ALL_MBOARDS) override
    {
        _time_source = source;
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_time_source(source, 0); });
    }

    std::string get_time_source(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_time_source(0);
    }

    std::vector<std::string> get_time_sources(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_time_sources(0);
    }

    void set_clock_source(const std::string& source, size_t mboard = ALL_MBOARDS) override
    {
        _clock_source = source;
        _for_each_mboard(
            mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_clock_source(source, 0); });
    }

    std::string get_clock_source(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_clock_source(0);
    }

    std::vector<std::string> get_clock_sources(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_clock_sources(0);
    }

    void set_sync_source(const std::string& clock_source,
        const std::string& time_source,
        size_t mboard = ALL_MBOARDS) override
    {
        _clock_source = clock_source;
        _time_source  = time_source;
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->set_sync_source(clock_source, time_source, 0);
        });
    }

    void set_sync_source(const uhd::device_addr_t& sync_source, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_sync_source(sync_source, 0); });
    }

    uhd::device_addr_t get_sync_source(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_sync_source(0);
    }

    std::vector<uhd::device_addr_t> get_sync_sources(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_sync_sources(0);
    }

    void set_clock_source_out(const bool enb, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_clock_source_out(enb, 0); });
    }

    void set_time_source_out(const bool enb, size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_time_source_out(enb, 0); });
    }

    size_t get_num_mboards(void) override
    {
        return _n;
    }

    uhd::sensor_value_t get_mboard_sensor(const std::string& name, size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_mboard_sensor(name, 0);
    }

    std::vector<std::string> get_mboard_sensor_names(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_mboard_sensor_names(0);
    }

    void set_user_register(const uint8_t addr,
        const uint32_t data,
        size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard, [&](uhd::usrp::multi_usrp::sptr& u) {
            u->set_user_register(addr, data, 0);
        });
    }

    // Channel count
    size_t get_rx_num_channels(void) override
    {
        return _n;
    }

    size_t get_tx_num_channels(void) override
    {
        return _n;
    }

    // Rates
    void set_rx_rate(double rate, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_rate(rate, 0); });
    }

    void set_tx_rate(double rate, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_tx_rate(rate, 0); });
    }

    double get_rx_rate(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_rate(0);
    }

    double get_tx_rate(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_rate(0);
    }

    uhd::meta_range_t get_rx_rates(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_rates(0);
    }

    uhd::meta_range_t get_tx_rates(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_rates(0);
    }

    // Frequency
    uhd::tune_result_t set_rx_freq(const uhd::tune_request_t& tune_request, size_t chan = 0) override
    {
        if (chan == ALL_CHANS) {
            // Interpret target as new aggregate center, recompute if configured
            if (_aggregate_total_bw > 0.0) {
                _aggregate_center_freq = tune_request.target_freq;
                _subband_centers = compute_subband_centers(
                    _aggregate_center_freq, _aggregate_total_bw, _guard_bw, _n);
                uhd::tune_result_t last;
                for (size_t i = 0; i < _n; i++) {
                    uhd::tune_request_t tr(_subband_centers[i]);
                    last = _usrps[i]->set_rx_freq(tr, 0);
                }
                return last;
            }
        }
        return _usrps.at(chan)->set_rx_freq(tune_request, 0);
    }

    uhd::tune_result_t set_tx_freq(const uhd::tune_request_t& tune_request, size_t chan = 0) override
    {
        if (chan == ALL_CHANS) {
            if (_aggregate_total_bw > 0.0) {
                _aggregate_center_freq = tune_request.target_freq;
                _subband_centers = compute_subband_centers(
                    _aggregate_center_freq, _aggregate_total_bw, _guard_bw, _n);
                uhd::tune_result_t last;
                for (size_t i = 0; i < _n; i++) {
                    uhd::tune_request_t tr(_subband_centers[i]);
                    last = _usrps[i]->set_tx_freq(tr, 0);
                }
                return last;
            }
        }
        return _usrps.at(chan)->set_tx_freq(tune_request, 0);
    }

    double get_rx_freq(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_freq(0);
    }

    double get_tx_freq(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_freq(0);
    }

    // --- Required multi_usrp APIs (forwarded) ---

    uhd::wb_iface::sptr get_user_settings_iface(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_user_settings_iface(0);
    }

    uhd::rfnoc::radio_control& get_radio_control(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_radio_control(0);
    }

    uhd::extension::extension::sptr get_extension(
        const uhd::direction_t trx, const size_t chan) override
    {
        return _usrps.at(chan)->get_extension(trx, 0);
    }

    void set_rx_spp(const size_t spp, const size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_spp(spp, 0); });
    }

    uhd::freq_range_t get_rx_freq_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_freq_range(0);
    }

    uhd::freq_range_t get_fe_rx_freq_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_fe_rx_freq_range(0);
    }

    std::vector<std::string> get_rx_lo_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_names(0);
    }

    void set_rx_lo_source(
        const std::string& src, const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_lo_source(src, name, 0);
    }

    const std::string get_rx_lo_source(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_source(name, 0);
    }

    std::vector<std::string> get_rx_lo_sources(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_sources(name, 0);
    }

    void set_rx_lo_export_enabled(
        bool enabled, const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_lo_export_enabled(enabled, name, 0);
    }

    bool get_rx_lo_export_enabled(
        const std::string& name = ALL_LOS, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_export_enabled(name, 0);
    }

    double set_rx_lo_freq(double freq, const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->set_rx_lo_freq(freq, name, 0);
    }

    double get_rx_lo_freq(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_freq(name, 0);
    }

    uhd::freq_range_t get_rx_lo_freq_range(
        const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_lo_freq_range(name, 0);
    }

    std::vector<std::string> get_tx_lo_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_names(0);
    }

    void set_tx_lo_source(const std::string& src,
        const std::string& name = ALL_LOS,
        const size_t chan       = 0) override
    {
        _usrps.at(chan)->set_tx_lo_source(src, name, 0);
    }

    const std::string get_tx_lo_source(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_source(name, 0);
    }

    std::vector<std::string> get_tx_lo_sources(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_sources(name, 0);
    }

    void set_tx_lo_export_enabled(
        const bool enabled, const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_lo_export_enabled(enabled, name, 0);
    }

    bool get_tx_lo_export_enabled(
        const std::string& name = ALL_LOS, const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_export_enabled(name, 0);
    }

    double set_tx_lo_freq(
        const double freq, const std::string& name, const size_t chan = 0) override
    {
        return _usrps.at(chan)->set_tx_lo_freq(freq, name, 0);
    }

    double get_tx_lo_freq(const std::string& name, const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_freq(name, 0);
    }

    uhd::freq_range_t get_tx_lo_freq_range(
        const std::string& name, const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_lo_freq_range(name, 0);
    }

    void set_rx_gain(double gain, const std::string& name, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_gain(gain, name, 0);
    }

    std::vector<std::string> get_rx_gain_profile_names(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_gain_profile_names(0);
    }

    void set_rx_gain_profile(const std::string& profile, const size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_gain_profile(profile, 0);
    }

    std::string get_rx_gain_profile(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_gain_profile(0);
    }

    void set_normalized_rx_gain(double gain, size_t chan = 0) override
    {
        _usrps.at(chan)->set_normalized_rx_gain(gain, 0);
    }

    void set_rx_agc(bool enable, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_agc(enable, 0);
    }

    double get_rx_gain(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_gain(name, 0);
    }

    double get_normalized_rx_gain(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_normalized_rx_gain(0);
    }

    uhd::gain_range_t get_rx_gain_range(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_gain_range(name, 0);
    }

    std::vector<std::string> get_rx_gain_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_gain_names(0);
    }

    void set_rx_antenna(const std::string& ant, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_antenna(ant, 0);
    }

    void set_tx_antenna(const std::string& ant, size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_antenna(ant, 0);
    }

    std::string get_rx_antenna(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_antenna(0);
    }

    std::string get_tx_antenna(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_antenna(0);
    }

    std::vector<std::string> get_rx_antennas(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_antennas(0);
    }

    uhd::meta_range_t get_rx_bandwidth_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_bandwidth_range(0);
    }

    uhd::sensor_value_t get_rx_sensor(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_sensor(name, 0);
    }

    std::vector<std::string> get_rx_sensor_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_sensor_names(0);
    }

    void set_rx_dc_offset(const bool enb, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan, [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_dc_offset(enb, 0); });
    }

    void set_rx_dc_offset(const std::complex<double>& offset, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan,
            [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_dc_offset(offset, 0); });
    }

    uhd::meta_range_t get_rx_dc_offset_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_dc_offset_range(0);
    }

    void set_rx_iq_balance(const bool enb, size_t chan) override
    {
        _usrps.at(chan)->set_rx_iq_balance(enb, 0);
    }

    void set_rx_iq_balance(
        const std::complex<double>& correction, size_t chan = ALL_CHANS) override
    {
        _for_each_chan(chan,
            [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_iq_balance(correction, 0); });
    }

    bool has_rx_power_reference(const size_t chan = 0) override
    {
        return _usrps.at(chan)->has_rx_power_reference(0);
    }

    void set_rx_power_reference(const double power_dbm, const size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_power_reference(power_dbm, 0);
    }

    double get_rx_power_reference(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_power_reference(0);
    }

    uhd::meta_range_t get_rx_power_range(const size_t chan) override
    {
        return _usrps.at(chan)->get_rx_power_range(0);
    }

    uhd::freq_range_t get_tx_freq_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_freq_range(0);
    }

    uhd::freq_range_t get_fe_tx_freq_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_fe_tx_freq_range(0);
    }

    void set_tx_gain(double gain, const std::string& name, size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_gain(gain, name, 0);
    }

    std::vector<std::string> get_tx_gain_profile_names(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_gain_profile_names(0);
    }

    void set_tx_gain_profile(const std::string& profile, const size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_gain_profile(profile, 0);
    }

    std::string get_tx_gain_profile(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_gain_profile(0);
    }

    void set_normalized_tx_gain(double gain, size_t chan = 0) override
    {
        _usrps.at(chan)->set_normalized_tx_gain(gain, 0);
    }

    double get_tx_gain(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_gain(name, 0);
    }

    double get_normalized_tx_gain(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_normalized_tx_gain(0);
    }

    uhd::gain_range_t get_tx_gain_range(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_gain_range(name, 0);
    }

    std::vector<std::string> get_tx_gain_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_gain_names(0);
    }

    bool has_tx_power_reference(const size_t chan = 0) override
    {
        return _usrps.at(chan)->has_tx_power_reference(0);
    }

    void set_tx_power_reference(const double power_dbm, const size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_power_reference(power_dbm, 0);
    }

    double get_tx_power_reference(const size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_power_reference(0);
    }

    uhd::meta_range_t get_tx_power_range(const size_t chan) override
    {
        return _usrps.at(chan)->get_tx_power_range(0);
    }

    std::vector<std::string> get_tx_antennas(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_antennas(0);
    }

    uhd::meta_range_t get_tx_bandwidth_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_bandwidth_range(0);
    }

    uhd::sensor_value_t get_tx_sensor(const std::string& name, size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_sensor(name, 0);
    }

    std::vector<std::string> get_tx_sensor_names(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_sensor_names(0);
    }

    void set_tx_dc_offset(const std::complex<double>& offset, size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_dc_offset(offset, 0);
    }

    uhd::meta_range_t get_tx_dc_offset_range(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_dc_offset_range(0);
    }

    void set_tx_iq_balance(const std::complex<double>& correction, size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_iq_balance(correction, 0);
    }

    std::vector<std::string> get_gpio_banks(const size_t mboard) override
    {
        return _usrps.at(mboard)->get_gpio_banks(0);
    }

    void set_gpio_attr(const std::string& bank,
        const std::string& attr,
        const uint32_t value,
        const uint32_t mask,
        const size_t mboard) override
    {
        _usrps.at(mboard)->set_gpio_attr(bank, attr, value, mask, 0);
    }

    uint32_t get_gpio_attr(
        const std::string& bank, const std::string& attr, const size_t mboard) override
    {
        return _usrps.at(mboard)->get_gpio_attr(bank, attr, 0);
    }

    std::vector<std::string> get_gpio_src_banks(const size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_gpio_src_banks(0);
    }

    std::vector<std::string> get_gpio_srcs(
        const std::string& bank, const size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_gpio_srcs(bank, 0);
    }

    std::vector<std::string> get_gpio_src(
        const std::string& bank, const size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_gpio_src(bank, 0);
    }

    void set_gpio_src(const std::string& bank,
        const std::vector<std::string>& srcs,
        const size_t mboard = 0) override
    {
        _usrps.at(mboard)->set_gpio_src(bank, srcs, 0);
    }

    std::vector<std::string> get_rx_filter_names(const size_t chan) override
    {
        return _usrps.at(chan)->get_rx_filter_names(0);
    }

    uhd::filter_info_base::sptr get_rx_filter(
        const std::string& name, const size_t chan) override
    {
        return _usrps.at(chan)->get_rx_filter(name, 0);
    }

    void set_rx_filter(const std::string& name,
        uhd::filter_info_base::sptr filter,
        const size_t chan) override
    {
        _usrps.at(chan)->set_rx_filter(name, filter, 0);
    }

    std::vector<std::string> get_tx_filter_names(const size_t chan) override
    {
        return _usrps.at(chan)->get_tx_filter_names(0);
    }

    uhd::filter_info_base::sptr get_tx_filter(
        const std::string& name, const size_t chan) override
    {
        return _usrps.at(chan)->get_tx_filter(name, 0);
    }

    void set_tx_filter(const std::string& name,
        uhd::filter_info_base::sptr filter,
        const size_t chan) override
    {
        _usrps.at(chan)->set_tx_filter(name, filter, 0);
    }

    uhd::rfnoc::mb_controller& get_mb_controller(const size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_mb_controller(0);
    }

    void set_rx_bandwidth(double bandwidth, size_t chan = 0) override
    {
        _usrps.at(chan)->set_rx_bandwidth(bandwidth, 0);
    }

    void set_tx_bandwidth(double bandwidth, size_t chan = 0) override
    {
        _usrps.at(chan)->set_tx_bandwidth(bandwidth, 0);
    }

    double get_rx_bandwidth(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_bandwidth(0);
    }

    double get_tx_bandwidth(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_bandwidth(0);
    }

    // --- Everything else: forward or throw to keep MVP small ---

    uhd::usrp::dboard_iface::sptr get_rx_dboard_iface(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_dboard_iface(0);
    }

    uhd::usrp::dboard_iface::sptr get_tx_dboard_iface(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_dboard_iface(0);
    }

    // Many advanced APIs are not required for initial bonding research.
    // We implement them as best-effort passthrough or throw not_implemented.

    // Subdev specs
    void set_rx_subdev_spec(const uhd::usrp::subdev_spec_t& spec,
        size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard,
            [&](uhd::usrp::multi_usrp::sptr& u) { u->set_rx_subdev_spec(spec, 0); });
    }

    uhd::usrp::subdev_spec_t get_rx_subdev_spec(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_rx_subdev_spec(0);
    }

    void set_tx_subdev_spec(const uhd::usrp::subdev_spec_t& spec,
        size_t mboard = ALL_MBOARDS) override
    {
        _for_each_mboard(mboard,
            [&](uhd::usrp::multi_usrp::sptr& u) { u->set_tx_subdev_spec(spec, 0); });
    }

    uhd::usrp::subdev_spec_t get_tx_subdev_spec(size_t mboard = 0) override
    {
        return _usrps.at(mboard)->get_tx_subdev_spec(0);
    }

    std::string get_rx_subdev_name(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_rx_subdev_name(0);
    }

    std::string get_tx_subdev_name(size_t chan = 0) override
    {
        return _usrps.at(chan)->get_tx_subdev_name(0);
    }

    // Placeholder implementations for the rest (compiler will tell us if more are needed).

private:
    void _apply_sync_sources()
    {
        UHD_LOG_INFO("BONDED_B200",
            "Setting sync sources clock='" + _clock_source + "' time='" + _time_source + "'");
        for (auto& u : _usrps) {
            u->set_clock_source(_clock_source, 0);
            u->set_time_source(_time_source, 0);
        }
    }

    void _align_time_next_pps(const uhd::time_spec_t& time_spec = uhd::time_spec_t(0.0))
    {
        // Set all device times at next PPS to align.
        for (auto& u : _usrps) {
            u->set_time_next_pps(time_spec, 0);
        }
    }

    template <typename F>
    void _for_each_mboard(size_t mboard, F&& f)
    {
        if (mboard == ALL_MBOARDS) {
            for (auto& u : _usrps) {
                f(u);
            }
            return;
        }
        if (mboard >= _n) {
            throw uhd::index_error("mboard out of range");
        }
        f(_usrps.at(mboard));
    }

    template <typename F>
    void _for_each_chan(size_t chan, F&& f)
    {
        if (chan == ALL_CHANS) {
            for (auto& u : _usrps) {
                f(u);
            }
            return;
        }
        if (chan >= _n) {
            throw uhd::index_error("channel out of range");
        }
        f(_usrps.at(chan));
    }

    uhd::device_addr_t _bonded_args;
    size_t _n = 0;

    std::string _clock_source;
    std::string _time_source;

    double _aggregate_center_freq = 0.0;
    double _aggregate_total_bw    = 0.0;
    double _guard_bw              = 0.0;

    std::vector<double> _subband_centers;

    std::vector<uhd::usrp::multi_usrp::sptr> _usrps;
};

} // namespace

namespace uhd { namespace usrp {

// Factory used by multi_usrp.cpp when args specify type=bonded_b200
uhd::usrp::multi_usrp::sptr make_bonded_b200_multi_usrp(const uhd::device_addr_t& dev_addr)
{
    return std::make_shared<bonded_multi_usrp>(dev_addr);
}

}} // namespace uhd::usrp
