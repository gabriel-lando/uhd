# TX Bonding Plan — split one wideband signal across two synchronized radios

> Companion to the RX work in `CLAUDE.md` / `samples/OFFLINE_20MHZ_DECODE.md`.
> Supersedes `BONDING_TX_IMPLEMENTATION_PLAN.md` (that doc's Zadoff-Chu/QPSK
> codec is **obsolete** — we reuse gr-ieee802-11, same as RX).

## 1. Goal

Thesis Part 2: bond two low-cost B210s into one wideband **transmitter** — each
radio emits half of a wideband signal, and a **single, black-box** receiver (out
of our control: a real 802.11 device, or anything else) decodes the recombined
signal off the air. This is the mirror image of the proven RX bonding
(`overlap_reconstructor` + `bonded_receiver`).

There is **no salvageable TX code** — a grep/find confirmed the only "TX" artifact
is the obsolete design doc above. This is a clean-slate build mirroring the RX
architecture.

## 2. Design decisions (settled with the user) and why

### 2.1 Modulation-agnostic, pure-IQ splitter

The splitter is pure IQ DSP and knows nothing about 802.11. Whatever produces the
wideband baseband (today `wifi_phy_hier`; tomorrow LTE / 5G-NR / DVB) sits
upstream and is irrelevant. (The RX `overlap_reconstructor` is already agnostic in
exactly this way — it just stitches raw IQ.)

### 2.2 Hard split at the combined center — no overlap/crossfade

Radio A carries the lower half, Radio B the upper half; the seam lands on the
**center subcarrier that OFDM systems leave unused** (802.11 nulls DC; LTE
reserves DC — a general property of direct-conversion OFDM, not an 802.11 hack).

**Why hard split and not overlap:** Over the air the receiver sees an effective
channel that is _flat on the lower half_ (Radio A's gain/phase/delay baked in) and
_flat on the upper half_ (Radio B's), with a **step at the center**. Any receiver
that does per-subcarrier channel equalization with reference symbols — i.e. all
OFDM systems (802.11, LTE, 5G-NR, WiMAX, DVB-T) — estimates that channel per
subcarrier and inverts it. A clean step between two flat plateaus is "just a
channel." The **only** fragile subcarrier is the one exactly on the seam, and that
is the unused center subcarrier.

An **overlap + crossfade** (as on RX) would instead make the overlapped
subcarriers a _sum of both radios' LO phases_: `w_A·g_A·e^{jφA} + w_B·g_B·e^{jφB}`.
With uncalibrated, random per-power-up LO phases these **partially cancel** — at
the crossfade center the magnitude `∝ |cos(Δφ/2)|`, which can be a deep null a
black-box receiver cannot fix. So overlap would _require_ a calibration loop, and
it spreads the risk across a whole band instead of the single unused center
subcarrier. Hard split is the lowest-risk, RX-agnostic, standards-compliant path.

### 2.3 Do nothing on the RX side

RX is a standard receiver we don't touch — so calibrating against it is off the
table. This is what makes 2.2 decisive: we need a waveform any compliant receiver
decodes with its _normal_ equalizer.

### 2.4 TX sync only; rely on the RX equalizer for phase/gain

TX does shared 10 MHz + PPS time sync (already proven on RX, 0.000 µs spread) plus
a **fine per-device sample-delay trim** so inter-radio skew stays within the
cyclic prefix (~0.8 µs / 16 samples @ 20 MHz). Shared 10 MHz ⇒ no relative CFO.
The RX's LTF/reference-symbol equalizer absorbs the static per-radio gain/phase/
delay. No TX calibration.

### 2.5 Rates

`T = 20 MHz` combined ⇒ `per_radio = T/2 = 10 MHz` (the user's "20 MHz in two
10 MHz radios"). Side benefit: TX DSP is far lighter than the RX reconstructor
(just 2 NCOs + 2 decimators — no estimation, no crossfade FIRs), so **real-time at
20 MHz is plausible on TX**, unlike RX.

## 3. The DSP — `band_splitter` = exact inverse of `overlap_reconstructor`

Input: one wideband complex baseband at rate `T`, centered at 0. Per radio `i`
with center offset `off_i` (Radio A `-T/4`, Radio B `+T/4`):

1. **NCO translate** the combined stream by `-off_i` to bring that radio's half to
   baseband 0 (reuse the `volk_32fc_s32fc_x2_rotator_32fc` pattern).
2. **Rational resample** `T → per_radio` (decimate-by-2 for `per_radio = T/2`).
   The decimator's built-in anti-alias lowpass has its −6 dB point at `T/4` =
   **exactly the seam**, so the decimation filter _is_ the split filter. The two
   half-band slices tile the band with the boundary on the center subcarrier — no
   separate crossfade FIRs.

Reuse the `poly_resampler` + NCO helpers that already live in
`overlap_reconstructor.cpp`. **Recommended:** extract them into a small internal
header `lib/usrp/bonded/bonded_resample.hpp` shared by both engines; the existing
`overlap_reconstructor` ctest guards against regression. **Fallback** if the
refactor looks risky: duplicate the ~80-line resampler into `band_splitter.cpp`
and leave RX untouched.

Config struct mirrors `overlap_reconstructor_config`, inverted: `input_rate (T)`,
`per_radio_rate`, `radio1_offset_hz`, `radio2_offset_hz`. API:
`std::pair<vec,vec> split(const vec& wideband)` (one-shot) and
`process(const cf* in, size_t n, vec& out1, vec& out2)` (streaming, for GR).

## 4. Milestones

### M1 — band_splitter DSP + software loopback (NO hardware) ← first

- `lib/usrp/bonded/band_splitter.{hpp,cpp}` — the engine above.
- `lib/usrp/bonded/bonded_resample.hpp` — shared resampler/NCO (extracted).
- `tests/band_splitter_test.cpp` — ctest mirroring
  `tests/overlap_reconstructor_test.cpp`: synthesize a wideband multitone, split,
  recombine inline (upsample×2 + NCO shift back + sum — disjoint halves add
  cleanly), assert tones recovered flat (±~1%), low inter-tone spur, center-seam
  tone handled. Pure C++, no GR, no hardware.
- GR block `band_splitter` (1 complex in → 2 complex out) in `gr-bonded-usrp/`:
  `include/.../band_splitter.h`, `lib/band_splitter_impl.{h,cc}`,
  `python/.../bindings/band_splitter_python.cc` (+ register in
  `python_bindings.cc`), `grc/bonded_usrp_band_splitter.block.yml`. Mirror
  `overlap_reconstructor_impl` reversed (decimating, two output ports; implement
  `forecast`/`general_work`, propagate stream tags with positions scaled by the
  decimation).
- `samples/test_band_splitter.py` — smoke test mirroring `test_overlap_block.py`:
  `wifi_phy_hier` (802.11 TX) → `band_splitter` → recombine via the existing
  `overlap_reconstructor` (reused as the in-software "air") → gr-ieee802-11 RX →
  assert frames decode. (Reconstructor FIR length is clamped [17,257], so a
  near-zero overlap stays well-conditioned; the seam imperfection lands on the
  unused center subcarrier.)
- Build wiring: add `band_splitter.cpp` to `lib/usrp/bonded/CMakeLists.txt`;
  extend the per-file `-O3 -ffast-math -march=native` + FFTW/VOLK/OpenMP scoping
  in `lib/CMakeLists.txt` to cover `band_splitter.cpp`; add the test to
  `tests/CMakeLists.txt`; add GR sources to `gr-bonded-usrp/lib/CMakeLists.txt`
  and the bindings `CMakeLists.txt`.

### M2 — bonded_transmitter (synchronized 2-radio TX engine)

- `lib/usrp/bonded/bonded_transmitter.{hpp,cpp}` — mirror of `bonded_receiver`:
  reuse `setup_bonded_sync()` (`bonded_usrp.hpp`) and `frequency_plan` for the
  `fc ∓ T/4` centers; open devices, external 10 MHz + PPS, latch common
  `time = 0` (same `_align_time` PPS-edge logic). Create TX streamers
  (`get_tx_stream`). Burst mode (timed `tx_metadata_t.has_time_spec`, common
  start) and continuous mode (per-device send threads + queues). Async-msg
  listener threads for underrun counts. **Per-device integer/fractional
  sample-delay trim** to null residual inter-radio skew.
- GR block `bonded_sink` (2 complex in → device) wrapping it, mirroring
  `bonded_source` (header/impl/binding/GRC/CMake).
- `tests/bonded_transmitter_test.cpp` — no-hardware logic tests (freq-plan
  geometry, queue/burst-timing bookkeeping, underrun accounting), mirroring
  `tests/bonded_receiver_test.cpp`.

### M3 — On-air end-to-end + thesis result

- `samples/wifi_bonded_tx.py` — thin flowgraph, modes like the RX driver:
  `wifi_phy_hier` (or null/file source) → `band_splitter` → `bonded_sink` on the
  two synced B210s. Burst gating via `packet_pad` before the splitter; ensure
  `packet_len`/burst tags survive the decimation so bursts stay timed.
- Decode on the black-box RX by reusing the existing single-radio
  `samples/wifi_trx.py --mode rx` (no bonded code on RX).
- Validate decoded-frame count vs the single-radio 20 MHz baseline. Tune per-radio
  TX gains (match A/B power) and the sample-delay trim until seam/timing is clean.
  Capture spectra of slice A, slice B, and the recombined air signal for the
  presentation.
- Hardware roles (from `CLAUDE.md`): bonded **TX pair** = `30B56D6` (lower) +
  `30DBC3C` (upper), shared 10 MHz + PPS; RX-under-test = `30EDB63` (or any
  standard receiver). `fc = 2.412 GHz`.

## 5. Build / install (gotchas from CLAUDE.md)

```bash
# after editing lib/usrp/bonded/*
cd ~/uhd/host/build && make uhd band_splitter_test -j16
ctest -R band_splitter --output-on-failure
sudo cp -P lib/libuhd.so* /usr/local/lib/ && sudo ldconfig   # fast lib-only install
# after a C++ HEADER change, rebuild+reinstall the GR module (else free(): invalid pointer):
cd ~/uhd/host/gr-bonded-usrp/build && cmake .. && make -j16 && sudo make install && sudo ldconfig
```

Install libuhd **before** rebuilding the GR module (it links the installed lib).
If `build/` has root-owned files: `sudo chown -R gabriel:gabriel build`.

## 6. Verification

- **M1 (no hardware):**
  - `ctest -R band_splitter --output-on-failure` → green (flatness / spur / seam).
  - `cd ~/uhd/host/samples && python3 test_band_splitter.py` → frames decode
    through split → recombine → gr-ieee802-11.
  - Regression: `ctest -R overlap_reconstructor` still green (shared-resampler
    refactor didn't break RX).
- **M2 (no hardware):** `ctest -R bonded_transmitter --output-on-failure`.
- **M3 (hardware):** run `wifi_bonded_tx.py` on the two synced B210s + decode on
  the RX radio; compare `tcpdump -r` frame counts to the single-radio baseline;
  confirm slice-A / slice-B / recombined spectra look right.

## 7. Out of scope / deferred

- Overlap + crossfade variant and any monitor-radio calibration (only needed if a
  future modulation can't tolerate the hard seam; the agnostic design leaves room
  to add an `overlap > 0` path later).
- The Zadoff-Chu/QPSK codec from `BONDING_TX_IMPLEMENTATION_PLAN.md` (obsolete).
- N > 2 radios.

---

## 8. Implementation record (what was built, algorithms, rationale)

This section documents every file created or modified during implementation,
the concrete algorithms used, and the reasoning behind each non-obvious choice.
All DSP is in C++; Python is a thin GR flowgraph wrapper only.

---

### 8.1 `lib/usrp/bonded/bonded_resample.hpp` (new)

**Purpose:** Shared DSP primitives used by both `overlap_reconstructor.cpp`
(RX) and `band_splitter.cpp` (TX). Extracted from the original
`overlap_reconstructor.cpp` so neither engine duplicates code.

**Contents:**

- `detail::cf` — `std::complex<float>` typedef.
- `detail::kPi` — `double` π constant.
- `detail::hamming(n)` — Hamming window of length `n`. Used as the FIR
  prototype envelope: `w[k] = 0.54 − 0.46·cos(2πk/(n−1))`. Chosen over
  rectangular because its first sidelobe is −43 dB vs −13 dB; this is
  sufficient for the ~50 dB alias suppression needed to keep out-of-band
  inter-radio leakage below the noise floor.
- `detail::design_lowpass_real(cutoff_norm, n_taps)` — Windowed-sinc lowpass:
  `h[k] = sinc(2·cutoff·(k − M)) · hamming[k]` where `M = (n_taps−1)/2`.
  The `−6 dB` point is `cutoff_norm` (normalized 0..0.5). For the band
  splitter the cutoff is set to `0.25` (= `per_radio_rate / (2·input_rate)`)
  so the filter passes the lower/upper quarter-band exactly up to the center
  seam. Tap count is clamped to `[17, 257]` and forced odd for linear phase.
- `detail::dot_cc(a, b, n)` / `detail::dot_cr(a, b, n)` — Dot products with
  VOLK acceleration (`volk_32fc_32f_dot_prod_32fc` / `volk_32fc_x2_dot_prod_32fc`
  when `UHD_HAVE_VOLK` is defined, plain loop otherwise).
- `detail::poly_resampler` — Streaming polyphase rational resampler:
  - Initialized with integer `interp` and `decim` computed from
    `gcd(input_rate, output_rate)`: `interp = output/gcd`, `decim = input/gcd`.
    For the standard 2× case (20 MHz → 10 MHz): `interp=1`, `decim=2`.
  - Prototype lowpass designed at `0.5/max(interp,decim)` (Nyquist of the
    higher rate). Polyphase decomposition: `h_p[k] = h[p + k·interp]` for
    `p = 0…interp−1`.
  - `feed(in, n, out)` maintains a history buffer of length `n_taps` across
    calls so filtering is continuous across GR work() chunk boundaries — no
    edge artifacts between calls.
  - Output length per call: `floor((n·interp + phase_acc) / decim)`, where
    `phase_acc` tracks the fractional input position.

**Why a shared header:** Both engines use identical NCO + polyphase logic.
A single source of truth means a bug fix or performance improvement applies
to both TX and RX simultaneously, and the existing
`overlap_reconstructor_test` acts as a regression gate.

---

### 8.2 `lib/usrp/bonded/band_splitter.{hpp,cpp}` (new)

**Purpose:** TX DSP engine — splits one wideband complex baseband at rate `T`
into two per-radio streams at `T/2`, each centered at 0 (ready to drive a
B210 TX chain tuned to `fc ± T/4`).

**Algorithm per radio `i` (offset `off_i`):**

1. **Copy** the wideband input into a per-radio scratch buffer.
2. **NCO rotation** by `−off_i`: multiply each sample by `e^{−j2π·off_i/T·t}`.
   For Radio A (`off_i = −T/4`): rotation phasor increment
   `Δφ_A = e^{+j·π/2}` shifts the lower half-band up to DC.
   For Radio B (`off_i = +T/4`): `Δφ_B = e^{−j·π/2}` shifts the upper
   half-band down to DC.
   Implemented with `volk_32fc_s32fc_x2_rotator_32fc` (VOLK) or a plain
   multiply-accumulate loop (scalar fallback). The phase accumulator is kept
   unit-magnitude by dividing by `|phase|` every call (prevents drift).
3. **Polyphase decimate** the rotated buffer through `poly_resampler` with
   `interp=1`, `decim=2`. The AA filter's `−6 dB` point is at `T/4`,
   which is **exactly the center seam** between the two half-bands. The
   decimation filter _is_ the split filter — no separate crossfade FIR is
   needed. Alias suppression is >40 dB (Hamming window), well below the
   noise floor of any practical 802.11 link.

**Why not overlap/crossfade on TX:** Adding overlap would make the
overlapping subcarriers a weighted sum of both radios' LO phases:
`w_A·g_A·e^{jφ_A} + w_B·g_B·e^{jφ_B}`. With uncalibrated LO phases
(random at each power-up, ~uniform on [0, 2π]) the crossfade center can be
a deep null `∝ |cos(Δφ/2)|` that a black-box receiver cannot equalize.
Hard split confines all risk to the one center subcarrier, which is the
unused DC null in every direct-conversion OFDM standard (802.11, LTE, 5G-NR,
DVB-T). The RX equalizer inverts the independent per-half gain/phase as a
static channel; no TX-side calibration is needed.

**Config struct** (`band_splitter_config`):

```cpp
double input_rate;       // T (combined rate, e.g. 20e6)
double per_radio_rate;   // T/2 (per-radio rate, e.g. 10e6)
double radio1_offset_hz; // offset of radio A slice center from fc (e.g. −5e6)
double radio2_offset_hz; // offset of radio B slice center from fc (e.g. +5e6)
```

**API:** `split(const vec&)` one-shot; `process(in, n, out1, out2)` streaming
(used by the GR block's `general_work`).

---

### 8.3 `tests/band_splitter_test.cpp` (new)

Four ctests (no hardware, no GR):

| Test                         | What it checks                                                                                                                            |
| ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `flatness_and_recombination` | Split a 16-tone multitone, upsample×2 + NCO-shift back + sum, check each tone recovered within ±1% amplitude and inter-tone spur < −30 dB |
| `half_assignment`            | A tone in the lower quarter-band appears only in `out1`; a tone in the upper quarter-band appears only in `out2`                          |
| `seam_attenuation`           | A tone exactly at the center seam (`off=0`) is attenuated ≥20 dB in both outputs (the AA filter suppresses it)                            |
| `streaming_identity`         | Feed the same 65536-sample block in 512-sample chunks; verify output matches the one-shot `split()` result sample-for-sample              |

All four pass (ctest #49 green).

---

### 8.4 GR block `band_splitter` in `gr-bonded-usrp/` (new)

Files: `include/gnuradio/bonded_usrp/band_splitter.h`,
`lib/band_splitter_impl.{h,cc}`,
`python/bonded_usrp/bindings/band_splitter_python.cc`,
`grc/bonded_usrp_band_splitter.block.yml`.

**GR scheduling interface:**

- `1 complex in → 2 complex out` (`general_work`, not `sync_block`, because
  input and output rates differ by the decimation factor).
- `forecast(noutput_items, ninput_required)`:
  `ninput_required[0] = ceil(noutput_items · input_rate / per_radio_rate)`.
  This tells the GR scheduler to queue enough input before calling `work`.
- `general_work`: calls `impl::process(in, n_in, out1, out2)`, handles the
  leftover-input pattern (`consume(0, n_consumed)` where `n_consumed` is
  derived from the number of output samples actually produced and the
  interp/decim ratio), and propagates stream tags with sample positions
  scaled by `per_radio_rate / input_rate`.
- `set_min_output_buffer` is not called — the GR scheduler sizes buffers based
  on `forecast` naturally.

---

### 8.5 `lib/usrp/bonded/bonded_transmitter.{hpp,cpp}` (new)

**Purpose:** Synchronized 2-radio TX engine. Mirrors `bonded_receiver` in
structure: opens devices, applies external 10 MHz + PPS sync, aligns clocks,
creates TX streamers, and provides burst and continuous transmission.

**Hardware sync (reused from `bonded_receiver`):**

1. Open each device independently (`uhd::usrp::multi_usrp::make`).
2. Set `clock_source = "external"`, `time_source = "external"` on both.
3. Wait for `ref_locked` sensor (timeout configurable, default 5 s).
4. Wait for the PPS edge on device 0 (`get_time_last_pps()` polling).
5. On PPS: call `set_time_next_pps(0.0)` on all devices simultaneously.
   Both radios now share a common `t = 0` with < 1 µs alignment (verified
   in the RX work, 0.000 µs spread measured).

**Channel count fix:** `usrp->get_tx_num_channels()` returns 2 on a B210
(channels A:A and A:B). The bonded TX uses exactly one TX chain per device
(one half-band per B210), so `_channels_per_device` is pinned to `1` at the
end of `_configure_hardware()` regardless of what the device reports. Without
this fix `_create_streamers()` opened a 2-channel streamer and the send loop
passed only 1 buffer pointer → UHD read `ptrs[1]` off the end of the vector
→ segfault.

**TX gain:** `set_normalized_tx_gain(gain, ch)` is used (not `set_tx_gain`).
The normalized API maps `[0.0, 1.0]` to the full hardware range (~89 dB on
B210). Using the raw dB API with a value like `0.5` would set 0.5 dB
(near-zero power) while the flowgraph intends 50% of full scale.

**Subdev:** `"A:B"` is passed explicitly so the TX streamer uses the VERT2450
antenna port, matching the `wifi_trx.py` reference and the physical antenna
wiring.

**Burst mode (`send_burst`):** Schedules a timed burst `now + delay_sec`
using `tx_metadata_t.has_time_spec = true`. Per-device integer sample-delay
trim (`sample_delay_trim[d]`) prepends `trim` zero samples before the burst
data, shifting that device's effective start by `trim / rate` seconds. This
corrects residual inter-device skew after PPS alignment within the cyclic
prefix budget (~16 samples at 20 MHz → 0.8 µs ≤ CP duration).

**Continuous mode (`start_continuous` / `queue_samples` / `stop`):**

Three-phase send loop per device (background thread):

_Phase 1 — pipeline fill wait:_
The GR wifi PHY sets `set_min_output_buffer` to up to 2 077 440 samples
(≈ 100 ms at 20 MHz) on internal blocks. The GR scheduler must fill all
internal buffers before `bonded_sink::work()` is ever called. The send
thread blocks on the queue condition variable until the first chunk arrives
from the GR scheduler. Starting the hardware TX stream before that would
schedule the timed burst in the past → immediate underruns from sample 0.

_Phase 2 — dynamic shared start time:_
Once the first chunk arrives, device 0's current time is read and
`start_time = t_now + 50 ms` is computed. A `std::mutex` + `bool` flag
ensure both send threads (one per radio) read the same `start_time` computed
by whichever thread wakes first. Both radios begin their timed burst at the
exact same sample, preserving inter-radio alignment.

_Phase 3 — two-level continuous loop:_

```
wait_for(1 ms) for queue data
  ├─ data arrived → send entire chunk, no silence
  └─ timeout (queue empty for > 1 ms) → send one FIFO-depth silence burst (~26 ms)
```

_Intra-frame silence prevention:_ GR delivers an 802.11 frame in consecutive
`work()` calls spaced ~50–200 µs apart (phy pipeline runs at CPU speed, not
at 10 MHz wire speed). The 1 ms wait threshold is >> inter-chunk spacing,
so the loop never times out mid-frame. Every chunk of a frame is sent as
real data with no silence inserted.

_Inter-frame gap handling:_ The 300 ms strobe interval far exceeds 1 ms, so
the wait times out. One FIFO-depth silence burst (262 144 samples = 26.2 ms
at 10 MHz) is sent. `streamer->send()` blocks at hardware rate once the
FPGA TX FIFO is full, so the silence burst takes ~26 ms of real time and
keeps the FIFO continuously full. ~12 such cycles cover the full 300 ms gap
with no underruns.

_Why not spin-loop with non-blocking dequeue:_ A non-blocking dequeue
(previous iteration) dequeues one GR chunk and sends it in ~70 µs, then
immediately checks again. If the next GR work() call hasn't arrived yet
(~100 µs later), the loop injects one `bs`-sample silence buffer mid-frame.
This corrupts the OFDM FFT window alignment for the affected symbols — the
first 2–3 frames decode because the GR pipeline is warm, then every frame
is broken. The 1 ms wait resolves the race without introducing any
under-run risk because the FIFO depth is >> the wait time.

---

### 8.6 GR block `bonded_sink` in `gr-bonded-usrp/` (new)

Files: `include/gnuradio/bonded_usrp/bonded_sink.h`,
`lib/bonded_sink_impl.{h,cc}`,
`python/bonded_usrp/bindings/bonded_sink_python.cc`,
`grc/bonded_usrp_bonded_sink.block.yml`.

`sync_block`, `2 complex in → 0 out`. `start()` calls
`_tx->configure(); _tx->start_continuous()`. `stop()` calls `_tx->stop()`.
`work()` calls `_tx->queue_samples(ptrs, noutput_items)` — purely a
pass-through; all DSP and scheduling is in the C++ engine.

---

### 8.7 `tests/bonded_transmitter_test.cpp` (new)

Nine no-hardware logic tests (ctest #50):

| Test                                       | What it checks                                                       |
| ------------------------------------------ | -------------------------------------------------------------------- |
| `default_config`                           | Default-constructed config has sane defaults                         |
| `delay_trim_size_mismatch`                 | `sample_delay_trim.size() != serials.size()` throws                  |
| `valid_trim_accepted`                      | Matching sizes accepted without throw                                |
| `freq_plan_geometry`                       | `freq_plan[0] = fc − per_radio/2`, `freq_plan[1] = fc + per_radio/2` |
| `freq_plan_mismatch`                       | `freq_plan.size() != serials.size()` throws                          |
| `delay_trim_timing_math`                   | 8-sample trim @ 10 MHz = 800 ns ≤ CP (800 ns)                        |
| `underrun_counters_init_zero`              | Both per-device underrun counters start at 0                         |
| `send_burst_before_configure_throws`       | `send_burst()` before `configure()` throws `uhd::runtime_error`      |
| `start_continuous_before_configure_throws` | `start_continuous()` before `configure()` throws                     |

All nine pass (ctest #50 green).

---

### 8.8 `samples/wifi_bonded_tx.py` (new)

Thin GR flowgraph; no DSP in Python.

**Chain:**

```
msg_strobe(300 ms)
    → ieee802_11.mac
    → wifi_phy_hier(bandwidth=20 MHz, fc=2.412 GHz, encoding=BPSK½)
    → multiply_const_cc(0.6)           # baseband amplitude scale
    → foo.packet_pad2(False, False, 0.01, 100, 1000)   # burst boundaries
    → bonded_usrp.band_splitter(20→10 MHz)
    → bonded_usrp.bonded_sink(2× B210, subdev=A:B)
```

`wifi_phy_hier` operates at the combined 20 MHz rate. The band_splitter
decimates to 10 MHz per radio, which is the rate the B210s are tuned to.

**Key parameter choices:**

- `foo.packet_pad2(False, False, 0.01, 100, 1000)` — 100 sample front-pad,
  1000 sample tail-pad, no delay mode. Ensures clean burst boundaries so
  the receiver's Schmidl-Cox correlator sees unambiguous packet edges.
  Argument types must match the pybind signature exactly: `(bool, bool,
float, int, int)` — passing floats for the int arguments causes
  `TypeError` at runtime.
- `multiply_const_cc(0.6)` — scales baseband amplitude before splitting.
  Combined with `--tx-gain` (normalized, `set_normalized_tx_gain`) this
  gives independent control of the digital headroom and the RF power.
- `null_src → phy` stream input (unused RX/sense port of `wifi_phy_hier`).

---

### 8.9 Bugs encountered and fixed during hardware bring-up

#### Bug 1 — `foo.packet_pad2` argument types

**Symptom:** `TypeError` at flowgraph start.
**Cause:** `packet_pad2` pybind signature is `(bool, bool, float, int, int)`;
passing `0.001` (float) for the int pad-count arguments raises a type error.
**Fix:** `foo.packet_pad2(False, False, 0.01, 100, 1000)`.

#### Bug 2 — Segfault on B210 with 2-channel TX

**Symptom:** Hard crash on `start_continuous()`.
**Cause:** `usrp->get_tx_num_channels()` returns 2 on a B210 (both A:A and
A:B subdevices are reported). `_create_streamers()` created a 2-channel TX
streamer. The send loop passed a 1-element pointer vector; UHD's framing
code read `ptrs[1]` (the second channel buffer pointer) off the end of the
vector → stack corruption → segfault.
**Fix:** Pin `_channels_per_device = 1` at end of `_configure_hardware()`.
Also fixed EOB send: `nullptr` + 0 samples → `&silence_sample` + 1 sample
(UHD dereferences the pointer even for `end_of_burst` sends).

#### Bug 3 — Immediate underruns from pipeline fill time

**Symptom:** Flood of `[bonded_transmitter] Device 0/1: underrun (U)` from
the first second, no packets decoded.
**Cause:** `start_continuous()` computed `start_time = now + 100 ms` and
launched the hardware TX stream immediately. The GR PHY pipeline has
internal minimum output buffers of up to 2 077 440 samples (set by
`fft_v_fftw` and `ofdm_cyclic_prefixer`) which the scheduler must fill before
the first `work()` call. This takes > 100 ms → by the time GR first called
`bonded_sink::work()` the hardware was already past the scheduled start time
→ all samples late → continuous underrun.
**Fix:** Phase 1 blocking wait (§8.5) + dynamic shared start time.

#### Bug 4 — TX gain 0.5 dB vs 0.5 normalized (~44 dB)

**Symptom:** Occasionally decoded 2–3 packets (ambient noise decodes) but
no sustained reception regardless of RX gain setting.
**Cause:** `set_tx_gain(0.5)` sets 0.5 dB of gain (near-zero RF power).
The reference `wifi_trx.py` uses `set_normalized_gain(0.5)` which maps to
~44 dB on a B210 (~32 000× more power).
**Fix:** Changed to `set_normalized_tx_gain(gain, ch)` in
`bonded_transmitter.cpp`.

#### Bug 5 — Wrong antenna port (A:A vs A:B)

**Symptom:** Very weak signal even at normalized gain 1.0; suspected
saturation but no `O` overflows at RX.
**Cause:** Default subdev `""` maps to `A:A` on a B210. The VERT2450
2.4 GHz antennas are physically connected to the `A:B` SMA connectors.
`A:A` carries the VERT900 900 MHz antenna — ~10–15 dB lower efficiency at
2.4 GHz and physically a different port. Both `wifi_trx.py` TX and RX
scripts explicitly use `set_subdev_spec('A:B', 0)`.
**Fix:** Hardcoded `"A:B"` as the subdev string in `wifi_bonded_tx.py`
(passed to `bonded_sink` → `bonded_transmitter::config.subdev`).

#### Bug 6 — Mid-frame silence injection (non-blocking spin loop)

**Symptom:** First 2–3 packets decoded after TX start, then zero decodes
indefinitely until TX was restarted.
**Cause:** The spin-loop (non-blocking dequeue) sent one GR chunk (~700
samples, ~70 µs at 10 MHz) then immediately checked the queue again. The
next GR `work()` call for the same 802.11 frame arrived ~100 µs later.
During that window the loop injected `bs` (~2040) silence samples
mid-frame. This corrupted the OFDM FFT window for those symbols. The
Schmidl-Cox correlator state machine on the RX fired once for the clean
initial burst, then got stuck in a broken state and never re-acquired.
**Fix:** Two-level wait strategy (§8.5 Phase 3): `wait_for(1 ms)` before
declaring an inter-frame gap, then a full FIFO-depth silence burst (not a
single `bs` slice) for the inter-frame gap. Intra-frame chunk-to-chunk
spacing is < 200 µs << 1 ms, so the wait never times out mid-frame.
