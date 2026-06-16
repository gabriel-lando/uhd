# Offline Full-20 MHz Bonded Decode — Reproduction & Explanation

A guide to reproduce the end-to-end demonstration that **two low-cost USB SDRs
(Ettus B210) can be bonded into one 20 MHz wideband receiver using an
intentional frequency overlap**, and that the reconstructed baseband decodes
real 802.11a/g packets.

This document is written so it can be used directly as the basis for a
pre-thesis presentation: it explains _what_ is built, _how_ the reconstruction
algorithm works, _why_ this algorithm was chosen over the alternatives, and the
exact steps to reproduce the result.

---

## 1. The problem and the contribution

A single B210 cannot reliably stream a full 20 MHz 802.11 channel over USB in our
setup. The idea is to **bond two radios**: each one tunes to a different part of
the channel, and we recombine their captures in software into one contiguous
20 MHz baseband that a standard 802.11 receiver can decode.

The naïve way — a **hard split**, where radio A takes the lower half and radio B
the upper half, meeting back-to-back — fails, because:

- The two radios have **independent local oscillators**. Even with a shared
  reference clock, each LO powers up with an unknown, fixed **phase offset**.
  At the seam, the two halves are added with the wrong relative phase, which
  destroys the OFDM subcarriers there.
- There is **no shared information** at the seam to estimate and correct that
  phase, and any small filter/gain mismatch creates a discontinuity.

**The contribution of this work is the overlap strategy:** each radio captures a
_wider_ slice so the two captures **share a 5 MHz region**. That shared region is
used to (a) measure and correct the relative gain/phase between the radios and
(b) **crossfade** the two spectra smoothly instead of butt-joining them. The
out-of-band corners that fall outside the target channel are discarded.

> **One-line pitch for the panel:** _"Instead of splitting the channel and gluing
> two halves at a seam, we let the two radios overlap, use the shared band to
> phase-align them, and crossfade — turning the hardest point (the join) into the
> most reliable one."_

---

## 2. The frequency plan (worked numbers)

Target: a 20 MHz channel centred at `fc` (default `fc = 2.412 GHz`).

```
                 fc-10   fc-5     fc        fc+5   fc+10
 target channel:   |------|--------|--------|------|        (20 MHz, fc ± 10)

 Radio A @ fc-5 :  [============= 15 MHz =============]      covers fc-12.5 .. fc+2.5
 Radio B @ fc+5 :           [============= 15 MHz =============]   fc-2.5 .. fc+12.5

 overlap (5 MHz):                 [========]                 fc-2.5 .. fc+2.5  (shared)
 discard corner A:  [==]                                     fc-12.5 .. fc-10  (out of band)
 discard corner B:                                  [==]     fc+10 .. fc+12.5  (out of band)
```

### The general plan (overlap is configurable)

Let `T` be the target channel bandwidth and `O` the overlap. Each radio is
responsible for one half of the channel (`T/2`) and additionally captures the
*whole* overlap, so:

```
  per-radio capture   = T/2 + O
  radio centres       = fc ± T/4                 (the two halves)
  overlap each radio  = O/2 past fc into the shared band
  corner discarded    = O/2 beyond each channel edge   (per radio)
  total RF spanned    = T + O   (= 2 corners of O/2 + the channel T)
```

| Quantity | Formula | `T=20, O=5` (default) | `T=20, O=1` |
|---|---|---|---|
| Target / output rate | `T` | 20 MHz | 20 MHz |
| Overlap | `O` | 5 MHz | 1 MHz |
| Per-radio capture | `T/2 + O` | 15 MHz | 11 MHz |
| Radio A / B centre | `fc ∓ T/4` | fc∓5 MHz | fc∓5 MHz |
| Corner discarded / radio | `O/2` | 2.5 MHz | 0.5 MHz |

A single `--samp-rate T` (plus optional `--overlap O`) sets the whole plan; the
radios go to `fc ± (per_radio − O)/2 = fc ± T/4`.

### What the overlap knob actually trades (important for the defense)

Reducing the overlap is appealing — each radio then captures *less* (11 MHz
instead of 15 MHz), which lowers the **per-radio USB/ADC bandwidth** (the very
bottleneck that motivated bonding). That is a real win **for the radios**.

But it is **not** a CPU win for the reconstruction — in fact the opposite:

| Overlap `O` (at `T=20`) | Per-radio capture | Reconstruction throughput |
|---|---|---|
| 5 MHz | 15 MHz | 0.29× real-time |
| 2 MHz | 12 MHz | 0.24× |
| 1 MHz | 11 MHz | 0.19× |

The crossfade transition band *equals* the overlap, so a **narrow overlap means
a sharper transition, which requires longer FIR filters** (and a less convenient
resampling ratio). The reconstruction stays spectrally correct at every setting
(verified band-flat to ±0.4% at `O = 5/2/1 MHz`), but it costs *more* CPU, not
less. The knob that genuinely reduces reconstruction CPU is the **target rate
`T`** (halving `T` roughly halves the load), not the overlap.

> **For the panel:** *"Overlap trades two independent budgets: a smaller overlap
> eases the per-radio USB link but demands sharper reconstruction filters. So
> overlap is tuned for the **radios'** capture limit and alignment robustness;
> the reconstruction's CPU is set by the target bandwidth."*

---

## 3. System architecture

Two layers. **All signal processing is in C++**; Python is only a thin test /
flowgraph interface.

```
   Radio A (B210 @ fc-5, 15 Msps) ┐
                                   │  shared 10 MHz ref + PPS
   Radio B (B210 @ fc+5, 15 Msps) ┘
            │
            ▼  (C++)  uhd::usrp::bonded::bonded_receiver
   synchronized acquisition: open both, lock to external 10 MHz,
   detect PPS edge, latch a common time=0 (timestamp-aligned to <1 µs)
            │   two time-aligned 15 Msps streams (ch0 of each radio)
            ▼  (C++)  uhd::usrp::bonded::overlap_reconstructor
   overlap-aware reconstruction  ->  one contiguous 20 Msps baseband at fc
            │
            ▼  gr-ieee802-11 (standard 802.11a/g receiver)
   Schmidl–Cox sync → OFDM FFT → channel equalizer → Viterbi → MAC parse
            │
            ▼
   decoded frames (counted / written to pcap)
```

C++ engines live in `lib/usrp/bonded/`:

- `bonded_receiver` — synchronized multi-radio acquisition.
- `overlap_reconstructor` — the reconstruction algorithm (Section 4).

They are wrapped as GNU Radio blocks in `gr-bonded-usrp/` (`bonded_source`,
`overlap_reconstructor`) and driven by `samples/wifi_bonded_rx.py`.

---

## 4. The reconstruction algorithm

Inputs: two time-aligned complex baseband streams at 15 Msps (one per radio).
Output: one complex baseband at 20 Msps, centred at `fc`, ready to decode.

The reconstruction is done **in the complex (IQ) domain** so that **phase is
preserved** — this is essential, because OFDM carries information in the phase of
each subcarrier. (An earlier power-spectrum "stitcher" was discarded precisely
because it throws phase away and therefore cannot be decoded.)

### Step 1 — Resample each radio 15 → 20 Msps

A polyphase rational resampler with a windowed-sinc anti-alias filter brings each
per-radio slice onto the common target grid (for the default plan that is
15 → 20 Msps, i.e. interpolate by 4 / decimate by 3; other overlap settings give
other ratios, e.g. 11 → 20). After this, each radio's content sits on the same
sample grid as the target channel.

### Step 2 — Translate each slice to the target baseband

Each radio is digitally frequency-shifted by a numerically controlled oscillator
(complex rotator) so its content lands at the correct absolute frequency in the
`fc`-centred baseband: radio A by −5 MHz, radio B by +5 MHz. Now both streams are
expressed in the _same_ coordinate system (the 20 MHz target channel).

### Step 3 — Align radio B to radio A using the shared overlap (the key step)

In the 5 MHz overlap both radios observe the **same RF signal**, so any
difference between them is the instrumentation mismatch we must remove:

- a constant **gain** ratio (different RX gains / cable losses), and
- a constant **phase** offset (independent LO start phases).

We band-limit both translated streams to the overlap band and accumulate three
running statistics over that band:

```
  A = radio-A overlap samples,  B = radio-B overlap samples
  S_aa = Σ |A|²        S_bb = Σ |B|²        S_xy = Σ A · conj(B)

  gain  = sqrt(S_aa / S_bb)          (a power ratio — robust)
  phase = arg(S_xy)                  (the coherent cross-term)
  α     = gain · e^{j·phase}
```

`α` is then applied to radio B (`B ← α·B`) so the two radios match in the shared
band. Two design points worth highlighting to the panel:

- **Why a power ratio for gain, not `|S_xy|/S_bb`?** On a real, _bursty_ WiFi
  signal the overlap band is just noise most of the time. A correlation-based
  gain would be dragged toward zero by that noise (the noise inflates `S_bb`
  while `S_xy` stays ~0). The power ratio `sqrt(S_aa/S_bb)` is immune to this.
  _(This was an actual bug that produced "0 packets decoded" until fixed.)_
- **The alignment is static** (the radios share a 10 MHz reference, so there is
  no frequency drift between them). We therefore estimate it during a short
  warm-up and then **freeze** it — both correct and cheap.

### Step 4 — Combine with complementary crossfade filters (and discard corners)

The two aligned streams are passed through a complementary pair of **complex FIR
filters** and summed:

```
  out = H_low(A) + H_high(α·B)
```

- `H_low` passes everything below the overlap, then ramps **down** across the
  overlap with a raised-cosine (`cos²`) weight, and is zero above it.
- `H_high` is the mirror: zero below the overlap, ramps **up** (`sin²`), passes
  everything above.
- Because `cos² + sin² = 1`, the two weights **sum to unity** across the overlap,
  so the shared band is reconstructed with flat amplitude and (after Step 3)
  coherent phase — there is no seam.
- Both filters are **zero outside the 20 MHz target band**, which is exactly how
  the out-of-band corners (Section 2) are discarded.

### Step 5 — Decode

The contiguous 20 Msps baseband is fed to the standard `gr-ieee802-11` receiver:
Schmidl–Cox short/long-preamble synchronization, 64-point OFDM FFT, channel
equalization, Viterbi decoding, and MAC parsing/CRC.

### Validation of the reconstruction (independent of hardware)

A unit test (`tests/overlap_reconstructor_test.cpp`, run with
`ctest -R overlap_reconstructor`) takes a known wideband signal, splits it into
two overlapping slices in software (injecting a known gain/phase offset on
radio B), reconstructs it, and asserts:

- the alignment **recovers the injected gain and phase exactly**,
- the reconstructed magnitude response is **flat to ±4%** across the whole band
  (lower-only, overlap, and upper-only regions), with negligible spurious energy,
- a tone sitting exactly in the seam is reconstructed at **−65 dB error**.

This proves the algorithm is correct before any radio is involved.

---

## 5. Why this algorithm (design rationale)

| Decision                                | Why                                                                                             | Alternative rejected                                                                                       |
| --------------------------------------- | ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| **Overlap, not hard split**             | Gives a shared band to phase-align the radios and to crossfade; removes the seam discontinuity. | Hard split at DC → seam destroys OFDM; the original code did this and decoded 0 packets.                   |
| **Complex (IQ) reconstruction**         | OFDM information is in subcarrier phase; must preserve phase.                                   | Power-spectrum stitching (magnitude only) — unusable for decode.                                           |
| **Raised-cosine crossfade (cos²/sin²)** | Weights sum to 1 → flat, coherent reconstruction across the join.                               | Linear or abrupt blend → amplitude ripple / discontinuity.                                                 |
| **Gain via power ratio**                | Robust to the overlap band being mostly noise on bursty traffic.                                | Correlation-magnitude gain → collapses to 0 under noise.                                                   |
| **Estimate-then-freeze alignment**      | The offset is static (shared clock); cheap and stable.                                          | Per-sample re-estimation → costly and noisier.                                                             |
| **Time-domain VOLK FIR pipeline**       | Streams efficiently on any block size; all logic stays in C++.                                  | FFT overlap-save also works and is mathematically equivalent; chosen variant favours streaming simplicity. |

---

## 6. Why validate _offline_ (record → reconstruct → decode)

The reconstruction is validated and correct; the remaining variable is **runtime
throughput**. On a single host CPU the full 20 MHz reconstruction does not run
faster than real time, so a _live_ flowgraph would drop samples and confound the
experiment with overflow artifacts.

**Offline replay removes that confound:** we record the two synchronized 15 MHz
slices to disk once (a lightweight operation — no DSP), then reconstruct and
decode from the files at whatever speed the CPU allows. A file source **never
drops samples**, so the decode result reflects the _method's_ correctness, not
the host's instantaneous speed. This is the clean, reproducible way to
demonstrate **packet recovery at the full 20 MHz**, which is the thesis claim.

> **For the panel:** _"We separate the scientific claim (the overlap method
> recovers packets) from an engineering constraint (single-host real-time
> throughput). Offline capture-then-process validates the claim without the
> confound; real-time is characterized separately as future work."_

---

## 7. Prerequisites

- 4× Ettus B210. Roles used here:
  - **TX** (reference transmitter): serial `30DBC3D`
  - **Bonded RX pair**: `30B56D6` (Radio A, lower slice) + `30DBC3C` (Radio B, upper slice)
  - (a 4th radio `30EDB63` is available for the single-radio baseline comparison)
- The **bonded RX pair must share an external 10 MHz reference and a PPS signal**
  (clock distribution). The TX does not need to share it.
- Software installed: UHD (with the `lib/usrp/bonded` engines), the
  `gr-bonded-usrp` GNU Radio module, plus `gr-ieee802-11` and `gr-foo`.

If UHD or the module need (re)building:

```bash
# UHD library (from the build directory)
cd ~/uhd/host/build && make uhd -j$(nproc) && sudo make install && sudo ldconfig

# GNU Radio bonded module (must be rebuilt whenever the C++ headers change)
cd ~/uhd/host/gr-bonded-usrp/build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig
```

---

## 8. Step-by-step reproduction

All commands run from `~/uhd/host/samples`.

### Step 0 — Prove the reconstruction with no hardware

```bash
cd ~/uhd/host/build
ctest -R overlap_reconstructor --output-on-failure
```

Expect `100% tests passed`. This confirms the alignment recovery, band-flat
reconstruction, and seam quality described in Section 4.

### Step 1 — Transmit a 20 MHz 802.11 reference (Terminal 1)

```bash
cd ~/uhd/host/samples
./wifi_trx.py --mode tx --samp-rate 20e6 --tx-serial 30DBC3D
# optional: --encoding 0 (BPSK 1/2)  --pdu-length 500  --interval 300
```

### Step 2 — (baseline) Single-radio 20 MHz reference decode (Terminal 2)

Confirms the transmitted signal is decodable and gives a reference packet rate.

```bash
cd ~/uhd/host/samples
./wifi_trx.py --mode rx --samp-rate 20e6 --rx-serial 30EDB63 --pcap baseline.pcap
```

### Step 3 — Record the two synchronized overlapping slices (Terminal 3)

Lightweight capture (no reconstruction): writes two raw complex-float files.

```bash
cd ~/uhd/host/samples
timeout 8 ./wifi_bonded_rx.py --mode record \
    --freq 2.412e9 --samp-rate 20e6 \
    --serial-a 30B56D6 --serial-b 30DBC3C \
    --rx-gain-db 40 --rec-prefix cap
# -> writes cap_a.fc32 (Radio A slice) and cap_b.fc32 (Radio B slice)
#
# Optional: a narrower overlap records less per radio (less USB), e.g.
#   ... --samp-rate 20e6 --overlap 1e6 ...   # 11 MHz/radio instead of 15
# The replay step MUST use the same --samp-rate and --overlap.
```

Make sure the TX (Step 1) is running during this window. ~8 s at 2×15 Msps is a
few GB — ensure disk space; shorten `timeout` if needed.

### Step 4 — Reconstruct + decode offline (no radios needed)

```bash
cd ~/uhd/host/samples
./wifi_bonded_rx.py --mode replay --freq 2.412e9 --samp-rate 20e6 \
    --rec-prefix cap --pcap bonded.pcap
# if you recorded with a custom --overlap, pass the SAME value here too
```

Console prints each decoded frame (`decode_mac`/`parse_mac`); frames are also
written to `bonded.pcap`.

### Step 5 — Compare against the baseline

```bash
echo "single-radio baseline: $(tcpdump -r baseline.pcap 2>/dev/null | wc -l) frames"
echo "bonded reconstruction: $(tcpdump -r bonded.pcap   2>/dev/null | wc -l) frames"
```

**Success criterion:** the bonded reconstruction decodes a comparable number of
frames to the single-radio baseline (vs. **zero** for the old hard-split code).

---

## 9. What to show in the presentation

1. **The frequency plan** (Section 2 diagram) — the core idea in one picture.
2. **The unit-test numbers** (Step 0): alignment recovers injected gain/phase
   exactly; reconstruction flat to ±4%; seam at −65 dB. _Correctness proven in
   isolation._
3. **The pcap comparison** (Step 5): bonded ≈ baseline frame count. Open both in
   Wireshark to show real, CRC-valid 802.11 frames recovered from the bonded
   capture.
4. (Optional) A spectrum plot of `cap_a.fc32`, `cap_b.fc32`, and the
   reconstructed output, showing the two overlapping slices and the seamless
   20 MHz result.
5. **The honest runtime note** (Section 10): the method is validated at full
   20 MHz offline; live real-time is CPU-bound and characterized separately.

---

## 10. Tuning and troubleshooting

- **Bonded decodes nothing but baseline works:** almost always RX level. Sweep
  `--rx-gain-db 30/40/50` during recording, and try `--chan-est 0..3` on replay.
- **Verify synchronization:** the recorder prints "reference locked" and
  "PPS alignment done" for both devices. If not, check the 10 MHz/PPS cabling.
- **Disk throughput during record:** 2×15 Msps ≈ 240 MB/s. Use an SSD/NVMe and a
  short `timeout`; if you see overflow during record, reduce the duration.
- **Lower-rate live demo (optional):** `--samp-rate 5e6` runs the same pipeline
  near real time for a live demonstration of a narrower channel.

---

## 11. Limitations and future work

- **Real-time throughput** at 20 MHz is bounded by single-host CPU. The
  reconstruction was optimized extensively (FFTW, VOLK, multi-core, time-domain
  FIR) and reaches ~real-time only at ~5 MHz; full 20 MHz live would require a
  fused single-filter design, a faster CPU, or a GPU. Offline replay sidesteps
  this for the scientific claim.
- **Alignment model** corrects a static gain + phase (sufficient for shared-clock
  radios). A fractional-delay term could be added if radios with independent
  clocks are used.
- **Two radios / 2× bandwidth** here; the frequency-plan utility generalizes the
  overlap layout to more radios for wider aggregate bandwidth (future work).
