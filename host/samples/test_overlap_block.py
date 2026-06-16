#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
#
# Hardware-free smoke test for the bonded_usrp.overlap_reconstructor GNU Radio
# block: confirms the pybind11 bindings, forecast(), and general_work() integrate
# with the GR scheduler and produce output at the expected 4/3 rate.
#
# (Numerical reconstruction correctness is covered by the C++ unit test
#  overlap_reconstructor_test under ctest.)

import sys
import numpy as np
from gnuradio import gr, blocks
from gnuradio import bonded_usrp


def main():
    rate_in, rate_out = 15e6, 20e6
    n = 60000  # per-radio input samples
    rng = np.random.default_rng(0)
    a = (rng.standard_normal(n) + 1j * rng.standard_normal(n)).astype(np.complex64)
    b = (rng.standard_normal(n) + 1j * rng.standard_normal(n)).astype(np.complex64)

    tb = gr.top_block()
    src_a = blocks.vector_source_c(a.tolist(), False)
    src_b = blocks.vector_source_c(b.tolist(), False)
    recon = bonded_usrp.overlap_reconstructor(
        rate_in, rate_out, 20e6, 5e6, -5e6, 5e6, 4096)
    snk = blocks.vector_sink_c()

    tb.connect((src_a, 0), (recon, 0))
    tb.connect((src_b, 0), (recon, 1))
    tb.connect((recon, 0), (snk, 0))
    tb.run()

    out = np.array(snk.data())
    expected = n * rate_out / rate_in
    print(f"in={n}  out={len(out)}  expected~={expected:.0f}  "
          f"ratio={len(out)/n:.4f}")
    ok = (len(out) > 0.9 * expected and len(out) <= expected + 8192
          and np.all(np.isfinite(out.view(np.float32))))
    if not ok:
        print("FAIL: unexpected output length or non-finite samples")
        return 1
    print("PASS: overlap_reconstructor block runs in a flowgraph")
    return 0


if __name__ == '__main__':
    sys.exit(main())
