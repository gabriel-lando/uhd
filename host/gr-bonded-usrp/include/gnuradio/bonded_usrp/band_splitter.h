#pragma once

#include <gnuradio/bonded_usrp/api.h>
#include <gnuradio/block.h>
#include <memory>

namespace gr {
namespace bonded_usrp {

/*!
 * \brief Wideband band-splitter block (TX side).
 *
 * Takes one wideband complex baseband stream (at rate_in) and produces two
 * per-radio streams (each at rate_out) by NCO-translating each half-band
 * slice to DC and polyphase-decimating.
 *
 * This is the exact TX-side inverse of overlap_reconstructor:
 *   in:  wideband at rate_in (e.g. 20 MHz, centered at fc)
 *   out0: lower half at rate_out (e.g. 10 MHz, centered at fc + radio1_offset)
 *   out1: upper half at rate_out (e.g. 10 MHz, centered at fc + radio2_offset)
 *
 * The decimation AA filter's −6 dB point lands at the center seam, which is
 * the unused DC subcarrier in 802.11 and other direct-conversion OFDM systems.
 */
class BONDED_USRP_API band_splitter : virtual public gr::block
{
public:
    using sptr = std::shared_ptr<band_splitter>;

    static sptr make(double rate_in,
                     double rate_out,
                     double radio1_offset,
                     double radio2_offset);
};

} // namespace bonded_usrp
} // namespace gr
