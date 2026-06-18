#include "band_splitter_impl.h"

#include "band_splitter.hpp" // bonded::band_splitter
#include <gnuradio/io_signature.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gr {
namespace bonded_usrp {

band_splitter::sptr band_splitter::make(double rate_in,
    double rate_out,
    double radio1_offset,
    double radio2_offset)
{
    return gnuradio::make_block_sptr<band_splitter_impl>(
        rate_in, rate_out, radio1_offset, radio2_offset);
}

band_splitter_impl::band_splitter_impl(double rate_in,
    double rate_out,
    double radio1_offset,
    double radio2_offset)
    : gr::block("band_splitter",
          gr::io_signature::make(1, 1, sizeof(gr_complex)),
          gr::io_signature::make(2, 2, sizeof(gr_complex))),
      _rate_in(rate_in),
      _rate_out(rate_out)
{
    bonded::band_splitter_config cfg;
    cfg.input_rate       = rate_in;
    cfg.per_radio_rate   = rate_out;
    cfg.radio1_offset_hz = radio1_offset;
    cfg.radio2_offset_hz = radio2_offset;
    _splitter = std::make_unique<bonded::band_splitter>(cfg);

    set_relative_rate(rate_out / rate_in);
}

band_splitter_impl::~band_splitter_impl() = default;

bool band_splitter_impl::start()
{
    _splitter->reset();
    _leftover0.clear();
    _leftover1.clear();
    _lead = 0;
    return true;
}

void band_splitter_impl::forecast(int noutput_items,
    gr_vector_int& ninput_items_required)
{
    // Inverse of the decimation ratio: need ~rate_in/rate_out inputs per
    // output sample.  The leftover queue absorbs the variable per-chunk yield.
    int need = static_cast<int>(std::ceil(noutput_items * _rate_in / _rate_out));
    need = std::max(1, need);
    for (auto& r : ninput_items_required) {
        r = need;
    }
}

int band_splitter_impl::general_work(int noutput_items,
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    const gr_complex* in = static_cast<const gr_complex*>(input_items[0]);
    gr_complex* out0     = static_cast<gr_complex*>(output_items[0]);
    gr_complex* out1     = static_cast<gr_complex*>(output_items[1]);

    int produced = 0;

    // 1) Drain leftover samples from the previous call.
    const size_t avail = _leftover0.size() - _lead;
    if (avail > 0) {
        const size_t take =
            std::min(avail, static_cast<size_t>(noutput_items));
        std::memcpy(out0, _leftover0.data() + _lead,
            take * sizeof(gr_complex));
        std::memcpy(out1, _leftover1.data() + _lead,
            take * sizeof(gr_complex));
        produced += static_cast<int>(take);
        _lead += take;
        if (_lead == _leftover0.size()) {
            _leftover0.clear();
            _leftover1.clear();
            _lead = 0;
        }
    }

    // 2) Process new input when output space remains and leftover is drained.
    int consumed = 0;
    if (produced < noutput_items && _leftover0.empty()) {
        const int navail = ninput_items[0];
        if (navail > 0) {
            _tmp0.clear();
            _tmp1.clear();
            _splitter->process(in, static_cast<size_t>(navail), _tmp0, _tmp1);
            consumed = navail;

            const size_t room =
                static_cast<size_t>(noutput_items - produced);
            const size_t take = std::min(room, _tmp0.size());
            if (take > 0) {
                std::memcpy(out0 + produced, _tmp0.data(),
                    take * sizeof(gr_complex));
                std::memcpy(out1 + produced, _tmp1.data(),
                    take * sizeof(gr_complex));
                produced += static_cast<int>(take);
            }
            if (take < _tmp0.size()) {
                _leftover0.assign(
                    _tmp0.begin() + static_cast<std::ptrdiff_t>(take),
                    _tmp0.end());
                _leftover1.assign(
                    _tmp1.begin() + static_cast<std::ptrdiff_t>(take),
                    _tmp1.end());
                _lead = 0;
            }
        }
    }

    consume_each(consumed);
    return produced;
}

} // namespace bonded_usrp
} // namespace gr
