#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
#
# M1 calibration measurement: two bonded B210s each transmit a CW tone at a
# DISTINCT known frequency, simultaneously, on a shared 10 MHz + PPS reference.
#
# A monitor receiver (3rd B210) captures both tones at once; because its single
# LO phase is COMMON to both tones, it cancels in the difference, so the captured
# data lets us recover the true inter-radio relative phase phi(B-A) over time:
#   flat  -> static offset (one-shot correction works)
#   ramp  -> relative CFO (constant frequency correction)
#   walk  -> phase noise (needs a fast tracking loop)
#
# This is NOT production DSP — it is a measurement harness for the TX phase-
# calibration work (see TX_BONDING_PLAN). Production correction will be in C++.
#
# Usage:
#   ./bonded_cw.py --serial-a 30B56D6 --serial-b 30DBC3C --freq 2.484e9
#   # then capture on the monitor:
#   ./wifi_trx.py --mode rx --samp-rate 20e6 --rx-serial 30DBC3D \
#       --rx-gain 0.4 --iq-capture /tmp/mon_iq.fc32

import sys, signal, argparse, time
from gnuradio import analog, gr, uhd


def make_sink(serial, freq, lo_offset, rate, gain, clock_src, time_src):
    snk = uhd.usrp_sink(
        ",".join(('', f"serial={serial}")),
        uhd.stream_args(cpu_format="fc32", args='', channels=[0]),
        '')
    snk.set_clock_source(clock_src, 0)
    snk.set_time_source(time_src, 0)
    snk.set_subdev_spec('A:B', 0)
    snk.set_samp_rate(rate)
    snk.set_center_freq(
        uhd.tune_request(freq, rf_freq=freq - lo_offset,
                         rf_freq_policy=uhd.tune_request.POLICY_MANUAL), 0)
    snk.set_normalized_gain(gain, 0)
    return snk


class BondedCW(gr.top_block):
    def __init__(self, a):
        gr.top_block.__init__(self, "Bonded CW", catch_exceptions=True)
        per_radio = a.samp_rate / 2.0
        freq_a = a.freq - a.samp_rate / 4.0   # 2479 MHz default
        freq_b = a.freq + a.samp_rate / 4.0   # 2489 MHz default
        print(f"[cw] Radio A {a.serial_a} @ {freq_a/1e6:.3f} MHz + tone {a.tone_a/1e6:+.2f} MHz")
        print(f"[cw] Radio B {a.serial_b} @ {freq_b/1e6:.3f} MHz + tone {a.tone_b/1e6:+.2f} MHz")
        print(f"[cw] monitor should see tones at "
              f"{(freq_a+a.tone_a-a.freq)/1e6:+.2f} and "
              f"{(freq_b+a.tone_b-a.freq)/1e6:+.2f} MHz (rel to fc)")

        self.snk_a = make_sink(a.serial_a, freq_a, a.lo_offset, per_radio,
                               a.tx_gain, a.clock_source, a.time_source)
        self.snk_b = make_sink(a.serial_b, freq_b, a.lo_offset, per_radio,
                               a.tx_gain, a.clock_source, a.time_source)
        # Align both to a common t=0 on the shared time base.
        self.snk_a.set_time_unknown_pps(uhd.time_spec(0))
        self.snk_b.set_time_unknown_pps(uhd.time_spec(0))

        self.src_a = analog.sig_source_c(per_radio, analog.GR_COS_WAVE,
                                         a.tone_a, a.ampl, 0, 0)
        self.src_b = analog.sig_source_c(per_radio, analog.GR_COS_WAVE,
                                         a.tone_b, a.ampl, 0, 0)
        self.connect((self.src_a, 0), (self.snk_a, 0))
        self.connect((self.src_b, 0), (self.snk_b, 0))


def main():
    p = argparse.ArgumentParser(description='Two-radio CW tone TX (calibration measurement)')
    p.add_argument('--freq', type=float, default=2.484e9)
    p.add_argument('--samp-rate', type=float, default=20e6)
    p.add_argument('--lo-offset', type=float, default=0.0)
    p.add_argument('--tx-gain', type=float, default=0.8)
    p.add_argument('--ampl', type=float, default=0.3)
    p.add_argument('--tone-a', type=float, default=2e6, help='Radio A baseband tone (Hz)')
    p.add_argument('--tone-b', type=float, default=-2e6, help='Radio B baseband tone (Hz)')
    p.add_argument('--serial-a', type=str, default='30B56D6')
    p.add_argument('--serial-b', type=str, default='30DBC3C')
    p.add_argument('--clock-source', type=str, default='external')
    p.add_argument('--time-source', type=str, default='external')
    a = p.parse_args()

    tb = BondedCW(a)

    def shutdown(sig=None, frame=None):
        print("\n[cw] stopping"); tb.stop(); tb.wait(); sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    tb.start()
    print("[cw] transmitting tones — Ctrl+C to stop")
    tb.wait()


if __name__ == '__main__':
    main()
