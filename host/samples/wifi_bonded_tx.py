#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0
#
# Bonded WiFi TX — hard-split wideband transmission across two synchronized B210s.
#
# Thesis design (hard split at the center seam)
# ---------------------------------------------
# A single 802.11a/g wideband baseband at `samp_rate` (e.g. 20 MHz) is split by
# band_splitter into two per-radio streams at samp_rate/2 (e.g. 10 MHz):
#
#   Radio A  serial_a  tuned to fc - samp_rate/4  lower half-band
#   Radio B  serial_b  tuned to fc + samp_rate/4  upper half-band
#   seam: the center (unused DC) subcarrier of the 802.11 frame
#
# The receiver (any compliant 802.11 device) sees a per-subcarrier channel that
# is flat on each half; its LTF equalizer corrects the independent gain/phase of
# each half.  The seam lands on the unused DC subcarrier — no information is
# lost.
#
# ALL TX DSP lives in C++:
#   - bonded_usrp.band_splitter     wideband → two per-radio streams (NCO +
#                                   polyphase decimation, −6 dB split at DC)
#   - bonded_usrp.bonded_sink       synchronized 2-radio TX engine
#                                   (shared 10 MHz + PPS, timed start,
#                                   per-device sample-delay trim)
#
# This Python file is only a thin flowgraph; no DSP is done here.
#
# Usage
# -----
#   Terminal 1 — bonded TX (this script):
#       ./wifi_bonded_tx.py --serial-a 30B56D6 --serial-b 30DBC3C --tx-gain 0.5
#   Terminal 2 — standard single-radio RX (no bonded code):
#       ./wifi_trx.py --mode rx --samp-rate 20e6 --rx-serial 30EDB63 \
#           --pcap bonded.pcap
#   Compare:
#       tcpdump -r bonded.pcap | wc -l
#       # vs: ./wifi_trx.py --mode tx --samp-rate 20e6 --tx-serial 30DBC3D
#             ./wifi_trx.py --mode rx  --samp-rate 20e6 --rx-serial 30EDB63 \
#                 --pcap baseline.pcap
#
# Requires: gnuradio, gr-ieee802-11, gr-foo, uhd, gr-bonded-usrp
#           external 10 MHz + PPS shared between both bonded TX radios

import os
import sys
import signal
import argparse
import time

sys.path.append(os.environ.get('GRC_HIER_PATH', os.path.expanduser('~/.grc_gnuradio')))

import pmt
from gnuradio import blocks, gr, network
import ieee802_11
import foo
from gnuradio import bonded_usrp

from wifi_phy_hier import wifi_phy_hier


ENCODING_NAMES = [
    'BPSK 1/2', 'BPSK 3/4', 'QPSK 1/2', 'QPSK 3/4',
    '16QAM 1/2', '16QAM 3/4', '64QAM 2/3', '64QAM 3/4',
]


class WifiBondedTx(gr.top_block):
    """wifi_phy_hier → band_splitter → bonded_sink (all DSP in C++).

    Chain:
      [strobe] → mac ──msg──▶ wifi_phy_hier ──stream──▶ multiply ──▶ pad
                null_src ──────────────────────────────────────────────▲
      pad ──▶ band_splitter ──▶ bonded_sink (radio A / radio B)
    """

    def __init__(self, args):
        gr.top_block.__init__(self, "WiFi Bonded TX", catch_exceptions=True)

        fc           = args.freq
        samp_rate    = args.samp_rate          # combined/wideband rate (e.g. 20 MHz)
        per_radio    = samp_rate / 2.0         # per-B210 rate after splitting
        half_bw      = samp_rate / 4.0         # ± offset from fc to each radio center
        freq_a       = fc - half_bw            # lower radio center
        freq_b       = fc + half_bw            # upper radio center

        print(f"[bonded-tx] fc={fc/1e6:.3f} MHz  combined={samp_rate/1e6:.0f} MHz  "
              f"per-radio={per_radio/1e6:.0f} MHz")
        print(f"[bonded-tx] Radio A ({args.serial_a}) @ {freq_a/1e6:.3f} MHz")
        print(f"[bonded-tx] Radio B ({args.serial_b}) @ {freq_b/1e6:.3f} MHz")
        print(f"[bonded-tx] delay trim: A={args.delay_trim_a} B={args.delay_trim_b} samples")
        print(f"[bonded-tx] Requires external 10 MHz + PPS on both TX radios")

        # ── 802.11 PHY (operates at the combined wideband rate) ───────────────
        self.phy = wifi_phy_hier(
            bandwidth=samp_rate,
            chan_est=ieee802_11.LS,
            encoding=ieee802_11.Encoding(args.encoding),
            frequency=fc,
            sensitivity=0.56,
        )

        self.mac = ieee802_11.mac(
            args.src_mac, args.dst_mac, args.bss_mac,
        )

        self.strobe = blocks.message_strobe(
            pmt.intern("x" * args.pdu_length), args.interval,
        )

        # wifi_phy_hier needs a null stream input (baseband sense path, unused TX)
        self.null_src = blocks.vector_source_c((0,), True, 1, [])

        # Scale TX amplitude and ensure minimum output buffer for bursty wifi
        self.multiply = blocks.multiply_const_cc(args.tx_amplitude)
        self.multiply.set_min_output_buffer(100000)

        # Zero-pad between packets so the receiver sees clean burst boundaries.
        # packet_pad2(debug, delay, delay_sec, pad_front_samples, pad_tail_samples)
        self.pad = foo.packet_pad2(False, False, 0.01, 100, 1000)
        self.pad.set_min_output_buffer(96000)

        # Optional TCP PDU input
        if args.tcp_port:
            self.socket_pdu = network.socket_pdu(
                'TCP_SERVER', '', str(args.tcp_port), 10000, False)
            self.msg_connect((self.socket_pdu, 'pdus'), (self.mac, 'app in'))

        # ── Band splitter: 1×20 MHz → 2×10 MHz (C++ DSP) ─────────────────────
        self.splitter = bonded_usrp.band_splitter(
            samp_rate, per_radio, -half_bw, +half_bw)

        # ── Synchronized 2-radio TX sink (C++ bonded_transmitter) ─────────────
        freq_plan_csv = (args.freq_plan_csv
                         if args.freq_plan_csv
                         else f"{freq_a},{freq_b}")
        # Per-radio TX gain to balance the two half-bands at the receiver
        # (two independent B210s/antennas rarely radiate equal power).  Falls
        # back to the uniform --tx-gain when a per-radio value is unset.
        gain_a = args.tx_gain_a if args.tx_gain_a is not None else args.tx_gain
        gain_b = args.tx_gain_b if args.tx_gain_b is not None else args.tx_gain
        gain_plan_csv = (f"{gain_a},{gain_b}"
                         if (args.tx_gain_a is not None
                             or args.tx_gain_b is not None)
                         else "")
        if gain_plan_csv:
            print(f"[bonded-tx] per-radio TX gain: A={gain_a} B={gain_b}")

        # Inter-radio phase-coherence correction (applied to radio B).
        # freeze the relative CFO with --freq-offset-b and pin the relative
        # phase with --phase-offset-b (see bonded_cw.py / cw_analyze.py).
        freq_offset_csv  = f"{args.freq_offset_a},{args.freq_offset_b}"
        phase_offset_csv = f"{args.phase_offset_a},{args.phase_offset_b}"
        if args.freq_offset_b or args.phase_offset_b or args.freq_offset_a or args.phase_offset_a:
            print(f"[bonded-tx] phase corr: A(f={args.freq_offset_a}Hz,ph={args.phase_offset_a}) "
                  f"B(f={args.freq_offset_b}Hz,ph={args.phase_offset_b})")

        self.bonded_snk = bonded_usrp.bonded_sink(
            f"{args.serial_a},{args.serial_b}",
            per_radio,
            fc,
            args.tx_gain,
            args.clock_source,
            args.time_source,
            args.strict_lock,
            args.lock_timeout,
            "A:B",          # subdev: VERT2450 antenna port on B210
            freq_plan_csv,
            args.stream_args,
            args.delay_trim_a,
            args.delay_trim_b,
            gain_plan_csv,
            freq_offset_csv,
            phase_offset_csv,
        )

        # ── Connections ───────────────────────────────────────────────────────
        # PHY message path
        self.msg_connect((self.strobe, 'strobe'), (self.mac, 'app in'))
        self.msg_connect((self.mac, 'phy out'), (self.phy, 'mac_in'))

        # PHY stream path → split → transmit
        self.connect((self.null_src, 0),   (self.phy,      0))
        self.connect((self.phy,      0),   (self.multiply, 0))
        self.connect((self.multiply, 0),   (self.pad,      0))
        self.connect((self.pad,      0),   (self.splitter, 0))
        self.connect((self.splitter, 0),   (self.bonded_snk, 0))  # radio A
        self.connect((self.splitter, 1),   (self.bonded_snk, 1))  # radio B


def parse_mac_addr(s):
    parts = s.split(':')
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            f"MAC address must be xx:xx:xx:xx:xx:xx, got: {s}")
    return [int(p, 16) for p in parts]


def main():
    p = argparse.ArgumentParser(
        description='Bonded WiFi TX — hard-split wideband across two B210s '
                    '(all DSP in C++).',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # ── Radio parameters ──────────────────────────────────────────────────────
    p.add_argument('--freq', type=float, default=2.484e9,
                   help='Channel center fc in Hz (must match the RX). '
                        'Default 2.484e9 = 802.11 channel 14, kept clear of '
                        'the polluted 1/6/11 production WiFi channels.')
    p.add_argument('--samp-rate', type=float, default=20e6,
                   help='Combined wideband rate in Hz; per-radio = samp-rate/2')
    p.add_argument('--tx-gain', type=float, default=0.5,
                   help='Uniform TX gain, normalized [0.0, 1.0]; 0.5 ≈ 44 dB '
                        'on B210.  Used for any radio without a per-radio override.')
    p.add_argument('--tx-gain-a', type=float, default=None,
                   help='Per-radio normalized TX gain for Radio A (lower half); '
                        'overrides --tx-gain.  Use to balance the two half-bands '
                        'at the RX (e.g. boost the weaker radio).')
    p.add_argument('--tx-gain-b', type=float, default=None,
                   help='Per-radio normalized TX gain for Radio B (upper half); '
                        'overrides --tx-gain.')

    # ── Inter-radio phase-coherence correction ────────────────────────────────
    p.add_argument('--freq-offset-a', type=float, default=0.0,
                   help='Digital freq correction (Hz) on Radio A (rarely needed).')
    p.add_argument('--freq-offset-b', type=float, default=0.0,
                   help='Digital freq correction (Hz) on Radio B to freeze the '
                        'inter-radio relative CFO (set to -measured_CFO).')
    p.add_argument('--phase-offset-a', type=float, default=0.0,
                   help='Static phase offset (rad) on Radio A.')
    p.add_argument('--phase-offset-b', type=float, default=0.0,
                   help='Static phase offset (rad) on Radio B to pin the '
                        'inter-radio relative phase away from the sync dead zone.')
    p.add_argument('--tx-amplitude', type=float, default=0.6,
                   help='Baseband amplitude scale applied before the splitter')
    p.add_argument('--serial-a', type=str, default='30B56D6',
                   help='B210 serial for Radio A (lower half-band)')
    p.add_argument('--serial-b', type=str, default='30DBC3C',
                   help='B210 serial for Radio B (upper half-band)')

    # ── Sync / hardware ───────────────────────────────────────────────────────
    p.add_argument('--clock-source', type=str, default='external',
                   help='UHD clock source (external, internal, gpsdo)')
    p.add_argument('--time-source', type=str, default='external',
                   help='UHD time source (external, internal, gpsdo)')
    p.add_argument('--strict-lock', action='store_true',
                   help='Throw on reference lock failure instead of warning')
    p.add_argument('--lock-timeout', type=float, default=5.0,
                   help='Seconds to wait for 10 MHz reference lock')
    p.add_argument('--stream-args', type=str, default='',
                   help='Extra UHD TX stream args')
    p.add_argument('--freq-plan-csv', type=str, default='',
                   help='Override per-device center freqs (Hz, comma-separated).'
                        '  Default: fc ± samp-rate/4')

    # ── Per-device delay trim ─────────────────────────────────────────────────
    p.add_argument('--delay-trim-a', type=int, default=0,
                   help='Integer sample-delay trim for Radio A (positive = '
                        'delay A by N samples at per-radio rate).  Use to '
                        'correct residual inter-device skew after PPS alignment.')
    p.add_argument('--delay-trim-b', type=int, default=0,
                   help='Integer sample-delay trim for Radio B.')

    # ── 802.11 MAC/PHY parameters ─────────────────────────────────────────────
    p.add_argument('--encoding', type=int, default=0, choices=range(8),
                   help='MCS index: '
                        + ', '.join(f'{i}={n}'
                                    for i, n in enumerate(ENCODING_NAMES)))
    p.add_argument('--pdu-length', type=int, default=500,
                   help='TX payload size in bytes')
    p.add_argument('--interval', type=int, default=300,
                   help='TX strobe period in ms')
    p.add_argument('--tcp-port', type=int, default=None,
                   help='TCP server port for external PDU input')
    p.add_argument('--src-mac', type=parse_mac_addr,
                   default='23:23:23:23:23:23',
                   help='TX source MAC address')
    p.add_argument('--dst-mac', type=parse_mac_addr,
                   default='42:42:42:42:42:42',
                   help='TX destination MAC address')
    p.add_argument('--bss-mac', type=parse_mac_addr,
                   default='ff:ff:ff:ff:ff:ff',
                   help='TX BSS MAC address')

    # ── GR scheduler ─────────────────────────────────────────────────────────
    p.add_argument('--max-noutput-items', type=int, default=8192,
                   help='GNU Radio scheduler max noutput_items')

    args = p.parse_args()

    per_radio = args.samp_rate / 2.0
    print(f"[bonded-tx] WiFi bonded TX  fc={args.freq/1e6:.3f} MHz  "
          f"combined={args.samp_rate/1e6:.0f} MHz  per-radio={per_radio/1e6:.0f} MHz  "
          f"encoding={ENCODING_NAMES[args.encoding]}  interval={args.interval} ms")

    tb = WifiBondedTx(args)

    def shutdown(sig=None, frame=None):
        print("\n[bonded-tx] shutting down ...")
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    tb.start(args.max_noutput_items)
    print("[bonded-tx] running — press Ctrl+C to stop")
    tb.wait()


if __name__ == '__main__':
    main()
