#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
#
# Hardware-free smoke test for the bonded_usrp.band_splitter GNU Radio block.
#
# Confirms that the pybind11 bindings, forecast(), and general_work()
# integrate correctly with the GR scheduler:
#   - the block accepts a wideband complex stream and produces two outputs
#   - both output lengths match the expected decimated rate
#   - all output samples are finite
#
# Numerical correctness of the split DSP is validated by the C++ ctest
# band_splitter_test.  For the full split→recombine→802.11-decode loop see
# samples/wifi_bonded_tx.py (M3, requires hardware).

import sys
import numpy as np
from gnuradio import gr, blocks
from gnuradio import bonded_usrp


def main():
    rate_in  = 20e6
    rate_out = 10e6
    n        = 60000  # wideband input samples

    rng = np.random.default_rng(0)
    wb  = (rng.standard_normal(n) + 1j * rng.standard_normal(n)).astype(
        np.complex64)

    tb     = gr.top_block()
    src    = blocks.vector_source_c(wb.tolist(), False)
    spltr  = bonded_usrp.band_splitter(rate_in, rate_out, -5e6, 5e6)
    snk0   = blocks.vector_sink_c()
    snk1   = blocks.vector_sink_c()

    tb.connect((src,   0), (spltr, 0))
    tb.connect((spltr, 0), (snk0,  0))
    tb.connect((spltr, 1), (snk1,  0))
    tb.run()

    out0 = np.array(snk0.data())
    out1 = np.array(snk1.data())

    expected = n * rate_out / rate_in  # = n/2
    print(f"in={n}  out0={len(out0)}  out1={len(out1)}  expected~={expected:.0f}")
    print(f"ratio0={len(out0)/n:.4f}  ratio1={len(out1)/n:.4f}")

    ok = (len(out0) > 0 and len(out1) > 0
          and abs(len(out0) - len(out1)) <= 8
          and len(out0) > 0.80 * expected  # GR terminates finite sources ~10% early for downsampling blocks
          and len(out0) <= expected + 8192
          and np.all(np.isfinite(out0.view(np.float32)))
          and np.all(np.isfinite(out1.view(np.float32))))

    if not ok:
        print("FAIL: unexpected output lengths or non-finite samples")
        return 1

    print("PASS: band_splitter block runs in a flowgraph")
    return 0


if __name__ == '__main__':
    sys.exit(main())
