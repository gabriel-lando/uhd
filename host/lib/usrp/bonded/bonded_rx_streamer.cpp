// Licensed under the same terms as UHD

#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/stream.hpp>
#include <stdexcept>
#include <algorithm>

namespace uhd { namespace usrp { namespace bonded {

bonded_rx_streamer::bonded_rx_streamer(
    uhd::usrp::multi_usrp::sptr usrp,
    const sync_manager& sync,
    const config& cfg)
    : _usrp(usrp), _cfg(cfg), _sync(sync)
{
    if (!_usrp) throw std::invalid_argument("usrp pointer is null");
    size_t n = _usrp->get_num_mboards();
    // create one rx_streamer per board using otw format
    _streams.clear();
    for (size_t m = 0; m < n; ++m) {
        uhd::stream_args_t sargs(_cfg.otw_format, _cfg.sample_format);
        sargs.channels = {static_cast<size_t>(m)};
        _streams.push_back(_usrp->get_rx_stream(sargs));
    }
}

void bonded_rx_streamer::start(const uhd::time_spec_t& start_time)
{
    // Issue a start command for each stream at the given time
    for (auto& s : _streams) {
        uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        cmd.stream_now = false;
        cmd.time_spec = start_time;
        s->issue_stream_cmd(cmd);
    }
}

size_t bonded_rx_streamer::recv(
    std::vector<std::vector<std::complex<float>>>& buffs,
    size_t nsamps_per_device,
    uhd::rx_metadata_t& md,
    double timeout)
{
    if (_streams.empty()) return 0;
    size_t n = _streams.size();
    buffs.resize(n);
    size_t got = 0;

    for (size_t i = 0; i < n; ++i) {
        buffs[i].assign(nsamps_per_device, std::complex<float>(0,0));
        // receive into vector's data pointer
        size_t recvd = _streams[i]->recv(&buffs[i][0], nsamps_per_device, md, timeout);
        if (i == 0) got = recvd;
        // If other devices returned different sample counts, take min
        got = std::min(got, recvd);
    }
    return got;
}

void bonded_rx_streamer::stop()
{
    for (auto& s : _streams) {
        if (s) {
            uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
            cmd.stream_now = true;
            s->issue_stream_cmd(cmd);
        }
    }
}

}}} // namespace uhd::usrp::bonded
