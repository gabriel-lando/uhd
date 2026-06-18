#!/usr/bin/env python3
# Analyze a monitor capture from bonded_cw.py: recover the inter-radio relative
# phase phi(B-A) over time and classify it (static / CFO / phase-noise).
#
#   ./cw_analyze.py /tmp/mon_iq.fc32 [--tone-a-mon -3e6 --tone-b-mon +3e6 --fs 20e6]
#
# tone-*-mon = where each radio's tone lands in the MONITOR baseband (rel to fc).
# With bonded_cw.py defaults (tone_a=+2M, tone_b=-2M, fc midband) these are
# -3 MHz (radio A) and +3 MHz (radio B).
import sys, argparse
import numpy as np


def downconvert_tone(x, f, fs, dec):
    n = np.arange(len(x))
    bb = x * np.exp(-1j * 2 * np.pi * f * n / fs)        # tone -> DC
    m = (len(bb) // dec) * dec
    return bb[:m].reshape(-1, dec).mean(1)                # block-average (LPF+decimate)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('file')
    p.add_argument('--tone-a-mon', type=float, default=-3e6)
    p.add_argument('--tone-b-mon', type=float, default=+3e6)
    p.add_argument('--fs', type=float, default=20e6)
    p.add_argument('--dec', type=int, default=2000)       # -> 10 kHz phase-sample rate
    a = p.parse_args()

    x = np.fromfile(a.file, dtype=np.complex64)
    print(f"loaded {len(x)} samp = {len(x)/a.fs:.2f}s")
    A = downconvert_tone(x, a.tone_a_mon, a.fs, a.dec)
    B = downconvert_tone(x, a.tone_b_mon, a.fs, a.dec)
    fsd = a.fs / a.dec
    # gate on tone presence (both tones strong)
    g = (np.abs(A) > 0.3 * np.median(np.abs(A)[np.abs(A) > 0])) & \
        (np.abs(B) > 0.3 * np.median(np.abs(B)[np.abs(B) > 0]))
    if g.sum() < 50:
        print("Not enough tone energy — were both radios transmitting during capture?")
        print(f"  |A| med={np.median(np.abs(A)):.3g}  |B| med={np.median(np.abs(B)):.3g}")
        return
    rel = np.unwrap(np.angle(B[g] * np.conj(A[g])))       # phi(B-A) over time
    t = np.arange(len(rel)) / fsd
    slope, intercept = np.polyfit(t, rel, 1)              # rad/s
    cfo = slope / (2 * np.pi)                             # Hz
    resid = rel - (slope * t + intercept)
    print(f"\ninter-radio relative phase phi(B-A):")
    print(f"  mean = {np.degrees(intercept):+.0f} deg   span = {np.degrees(rel.max()-rel.min()):.0f} deg")
    print(f"  linear drift (relative CFO) = {cfo:+.1f} Hz")
    print(f"  residual after de-trend     = {np.degrees(resid.std()):.1f} deg rms")
    dur = len(rel) / fsd
    # phase rotation over one 802.11 frame (~0.7 ms) from the CFO alone
    print(f"  -> over a 700us frame, CFO gives {360*abs(cfo)*0.7e-3:.0f} deg of relative rotation")
    # verdict
    cfo_deg_per_frame = 360 * abs(cfo) * 0.7e-3
    if np.degrees(resid.std()) > 40:
        v = "PHASE NOISE dominant -> needs a fast tracking loop (M3)"
    elif cfo_deg_per_frame > 30:
        v = "RELATIVE CFO dominant -> a constant frequency correction may suffice"
    else:
        v = "mostly STATIC -> a one-shot phase correction may suffice (M2)"
    print(f"\nVERDICT: {v}")
    # coarse trajectory
    k = max(1, len(rel) // 20)
    print("trajectory (deg):", " ".join(f"{np.degrees(v):+.0f}" for v in rel[::k][:20]))


if __name__ == '__main__':
    main()
