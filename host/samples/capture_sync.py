#!/usr/bin/env python3
"""
Capture time-aligned IQ samples from two USRP B210s.

Timed capture (synchronized start on a shared PPS edge) is enabled
automatically when --time-source is external or gpsdo.
Both RX streams are armed with the same absolute timestamp and drained
in parallel threads to avoid USB scheduling skew.

Optionally, one device can transmit a CW tone during capture (--tx-serial)
to provide a coherent reference signal for phase/frequency analysis.
"""
import argparse
import numpy as np
import uhd
import time
import sys
import threading


def parse_args():
    p = argparse.ArgumentParser(
        description="Capture synchronized samples from two B210s.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--serials', nargs=2, required=True,
                   help='Serial numbers of the two B210s')
    p.add_argument('--rate', type=float, default=1e6,
                   help='Sample rate in Hz (default: 1e6)')
    p.add_argument('--freq', type=float, default=915e6,
                   help='Center frequency in Hz (default: 915e6)')
    p.add_argument('--gain', type=float, default=30,
                   help='RX gain in dB (default: 30)')
    p.add_argument('--nsamps', type=int, default=1_000_000,
                   help='Samples to capture per device (default: 1000000)')
    p.add_argument('--clock-source', default='internal',
                   choices=('internal', 'external', 'gpsdo'),
                   help='Clock reference for both devices (default: internal)')
    p.add_argument('--time-source', default='internal',
                   choices=('internal', 'external', 'gpsdo'),
                   help='Time/PPS source for both devices (default: internal)')
    p.add_argument('--timed', action='store_true',
                   help='Force timed start even with internal time source')
    p.add_argument('--ref-lock-timeout', type=float, default=10.0,
                   help='Timeout (s) waiting for external clock lock (default: 10)')
    p.add_argument('--pps-timeout', type=float, default=12.0,
                   help='Timeout (s) waiting for PPS detection (default: 12)')
    p.add_argument('--tx-serial', default=None,
                   help='Transmit a CW tone from this device during capture '
                        '(must be one of --serials)')
    p.add_argument('--tx-gain', type=float, default=60.0,
                   help='TX gain in dB (default: 60)')
    p.add_argument('--tx-offset', type=float, default=100e3,
                   help='Tone IF offset from center in Hz (default: 100e3)')
    return p.parse_args()


def setup_usrp(serial, rate, freq, gain, clock_source, time_source):
    usrp = uhd.usrp.MultiUSRP(f"serial={serial}")
    usrp.set_rx_rate(rate)
    usrp.set_rx_freq(freq)
    usrp.set_rx_gain(gain)
    print(f"[{serial}] RX antenna: {usrp.get_rx_antenna()}")
    try:
        usrp.set_clock_source(clock_source)
        print(f"[{serial}] Set clock_source = {clock_source}")
    except Exception as e:
        print(f"[{serial}] WARNING: Could not set clock_source to '{clock_source}': {e}")
    try:
        usrp.set_time_source(time_source)
        print(f"[{serial}] Set time_source  = {time_source}")
    except Exception as e:
        print(f"[{serial}] WARNING: Could not set time_source to '{time_source}': {e}")
    return usrp


def wait_for_ref_lock(usrp, serial, timeout):
    """Poll ref_locked sensor until it asserts or timeout expires."""
    sensor_names = usrp.get_mboard_sensor_names(0)
    if 'ref_locked' not in sensor_names:
        print(f"[{serial}] No ref_locked sensor available — assuming locked.")
        return True
    print(f"[{serial}] Waiting for external clock lock (timeout={timeout:.0f}s)...")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            if usrp.get_mboard_sensor('ref_locked', 0).to_bool():
                print(f"[{serial}] ref_locked = OK")
                return True
        except Exception:
            pass
        time.sleep(0.1)
    print(f"[{serial}] ERROR: External clock NOT locked after {timeout:.0f}s. "
          "Check 10 MHz cable and signal level (~+7 dBm).")
    return False


def wait_for_pps(usrp, serial, timeout):
    """Wait until get_time_last_pps() advances, indicating a PPS edge."""
    print(f"[{serial}] Waiting for PPS (timeout={timeout:.0f}s)...")
    t0 = usrp.get_time_last_pps()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.5)
        t1 = usrp.get_time_last_pps()
        if t1 != t0:
            print(f"[{serial}] PPS detected.")
            return True
    print(f"[{serial}] ERROR: PPS NOT detected after {timeout:.0f}s. "
          "Check PPS cable and signal (3.3 V CMOS, >100 µs pulse width).")
    return False


def synchronize_time(usrps, serials, timeout=3.0):
    """Arm set_time_next_pps(0) on all devices, then verify the latch.

    Polls get_time_last_pps() to detect the PPS edge rather than sleeping
    a fixed interval, so the post-edge readback is only milliseconds late.
    """
    print("Arming set_time_next_pps(0) on all devices...")
    t0_pps = usrps[0].get_time_last_pps().get_real_secs()
    for usrp in usrps:
        usrp.set_time_next_pps(uhd.types.TimeSpec(0.0))

    # Wait for the PPS edge to latch the new time.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        time.sleep(0.05)
        if usrps[0].get_time_last_pps().get_real_secs() != t0_pps:
            break
    else:
        print("ERROR: PPS edge not detected after arming set_time_next_pps. "
              "Check PPS cabling.")
        return False

    time.sleep(0.05)
    ok = True
    for usrp, serial in zip(usrps, serials):
        t = usrp.get_time_now().get_real_secs()
        print(f"[{serial}] Device time after PPS sync: {t:.3f} s")
        if t < 0 or t > 0.5:
            print(f"[{serial}] ERROR: Time sync failed (expected ~0 s, got {t:.3f} s). "
                  "Check PPS cabling.")
            ok = False
    return ok


def start_tx_tone(usrp, serial, freq, rate, tx_gain, tone_offset, stop_event):
    """Stream a CW tone at (freq + tone_offset) Hz in a background thread.

    The non-zero offset avoids LO self-mixing leakage at DC.
    Call stop_event.set() to stop; returns the started thread.
    """
    usrp.set_tx_rate(rate)
    usrp.set_tx_freq(freq)
    usrp.set_tx_gain(tx_gain)
    print(f"[{serial}] TX antenna: {usrp.get_tx_antenna()}")
    print(f"[{serial}] TX tone: freq={freq/1e6:.3f} MHz "
          f"(+{tone_offset/1e3:.1f} kHz offset), "
          f"rate={rate/1e6:.3f} MHz, gain={tx_gain} dB")

    st_args = uhd.usrp.StreamArgs('fc32', 'fc32')
    tx_stream = usrp.get_tx_stream(st_args)
    max_samps = tx_stream.get_max_num_samps()
    t = np.arange(max_samps) / rate
    tone_buf = np.exp(1j * 2 * np.pi * tone_offset * t).astype(np.complex64)

    md = uhd.types.TXMetadata()
    md.start_of_burst = True
    md.end_of_burst = False
    md.has_time_spec = False

    def _tx_loop():
        nonlocal md
        first = True
        while not stop_event.is_set():
            if first:
                md.start_of_burst = True
                first = False
            else:
                md.start_of_burst = False
            tx_stream.send(tone_buf, md)
        eob = uhd.types.TXMetadata()
        eob.end_of_burst = True
        tx_stream.send(np.zeros(1, dtype=np.complex64), eob)

    t = threading.Thread(target=_tx_loop, daemon=True)
    t.start()
    return t


def arm_capture(usrp, serial, nsamps, start_time):
    """Issue the RX stream command without blocking on recv."""
    st_args = uhd.usrp.StreamArgs('fc32', 'fc32')
    rx_stream = usrp.get_rx_stream(st_args)
    stream_cmd = uhd.types.StreamCMD(uhd.types.StreamMode.num_done)
    stream_cmd.num_samps = nsamps
    if start_time is not None:
        print(f"[{serial}] Scheduling timed capture at "
              f"{start_time.get_real_secs():.2f} s")
        stream_cmd.stream_now = False
        stream_cmd.time_spec = start_time
    else:
        print(f"[{serial}] Starting immediate capture")
        stream_cmd.stream_now = True
    usrp.issue_stream_cmd(stream_cmd)
    return rx_stream


def recv_capture(rx_stream, serial, nsamps):
    """Drain the RX stream into a buffer, looping until nsamps are received."""
    md = uhd.types.RXMetadata()
    samps = np.zeros(nsamps, dtype=np.complex64)
    total = 0
    while total < nsamps:
        chunk = rx_stream.recv(samps[total:], md, timeout=30.0)
        if md.error_code == uhd.types.RXMetadataErrorCode.overflow:
            # Overflow: UHD sends a 0-sample notification; streaming continues.
            total += chunk
            continue
        if md.error_code != uhd.types.RXMetadataErrorCode.none:
            print(f"[{serial}] WARNING: recv error {md.error_code}; aborting.")
            break
        if chunk == 0:
            print(f"[{serial}] WARNING: recv() returned 0 samples; aborting.")
            break
        total += chunk
    print(f"[{serial}] Received {total}/{nsamps} samples, "
          f"error_code={md.error_code}")
    return samps, md


def save_data(serial, samps, md, rate, freq, gain, clock_source, time_source):
    fname = f"capture_{serial}.npy"
    np.save(fname, samps)
    meta_lines = {
        'serial': serial,
        'rate': rate,
        'freq': freq,
        'gain': gain,
        'clock_source': clock_source,
        'time_source': time_source,
        'timestamp': time.time(),
        'metadata': str(md),
    }
    with open(f"capture_{serial}_meta.txt", 'w') as f:
        for k, v in meta_lines.items():
            f.write(f"{k}: {v}\n")
    print(f"Saved {fname} and capture_{serial}_meta.txt")


def main():
    args = parse_args()
    serial0, serial1 = args.serials
    uses_pps = args.time_source in ('external', 'gpsdo')
    use_timed = uses_pps or args.timed

    print(f"Clock source : {args.clock_source}")
    print(f"Time source  : {args.time_source}")
    print(f"Timed capture: {use_timed}")
    print()

    if args.tx_serial is not None and args.tx_serial not in args.serials:
        print(f"ERROR: --tx-serial {args.tx_serial!r} is not in --serials {args.serials}.")
        sys.exit(1)

    usrp0 = setup_usrp(serial0, args.rate, args.freq, args.gain,
                        args.clock_source, args.time_source)
    usrp1 = setup_usrp(serial1, args.rate, args.freq, args.gain,
                        args.clock_source, args.time_source)

    if args.clock_source in ('external', 'gpsdo'):
        ok0 = wait_for_ref_lock(usrp0, serial0, args.ref_lock_timeout)
        ok1 = wait_for_ref_lock(usrp1, serial1, args.ref_lock_timeout)
        if not (ok0 and ok1):
            print("External clock lock failed on one or both devices. Aborting.")
            sys.exit(1)

    if uses_pps:
        ok0 = wait_for_pps(usrp0, serial0, args.pps_timeout)
        ok1 = wait_for_pps(usrp1, serial1, args.pps_timeout)
        if not (ok0 and ok1):
            print("PPS detection failed on one or both devices. Aborting.")
            sys.exit(1)
        if not synchronize_time([usrp0, usrp1], [serial0, serial1]):
            print("Time synchronization failed. Aborting.")
            sys.exit(1)

    # Shared start timestamp derived from the common PPS timeline.
    if use_timed:
        start_time = usrp0.get_time_last_pps() + uhd.types.TimeSpec(5.0)
    else:
        start_time = None

    # Start TX tone before arming RX so the signal is already present at capture time.
    tx_stop = None
    tx_thread = None
    if args.tx_serial is not None:
        tx_usrp = usrp0 if args.tx_serial == serial0 else usrp1
        tx_stop = threading.Event()
        tx_thread = start_tx_tone(tx_usrp, args.tx_serial,
                                  args.freq, args.rate, args.tx_gain,
                                  args.tx_offset, tx_stop)
        time.sleep(0.1)  # brief settling time before arming RX

    # Arm both devices with the same timestamp, then drain in parallel.
    stream0 = arm_capture(usrp0, serial0, args.nsamps, start_time)
    stream1 = arm_capture(usrp1, serial1, args.nsamps, start_time)

    results = [None, None]

    def _recv(idx, stream, serial):
        results[idx] = recv_capture(stream, serial, args.nsamps)

    t0 = threading.Thread(target=_recv, args=(0, stream0, serial0), daemon=True)
    t1 = threading.Thread(target=_recv, args=(1, stream1, serial1), daemon=True)
    t0.start(); t1.start()
    t0.join(); t1.join()

    samps0, md0 = results[0]
    samps1, md1 = results[1]

    # Stop the TX tone now that both captures are done
    if tx_stop is not None:
        tx_stop.set()
        tx_thread.join(timeout=2.0)

    save_data(serial0, samps0, md0, args.rate, args.freq, args.gain,
              args.clock_source, args.time_source)
    save_data(serial1, samps1, md1, args.rate, args.freq, args.gain,
              args.clock_source, args.time_source)

    print("\nCapture complete.")


if __name__ == '__main__':
    main()
