# CLAUDE.md — Bonded Wideband RX (project context to resume)

> **Status: PAUSED.** This file captures everything needed to resume the RX
> bonding work later. Read this first, then `samples/OFFLINE_20MHZ_DECODE.md`
> for the full algorithm explanation.

## 1. Goal (thesis Part 1)

Bond two low-cost USB SDRs (Ettus B210) into one **20 MHz wideband receiver**
using an **intentional frequency overlap** (NOT a hard split), and prove the
reconstructed baseband decodes real **802.11a/g** packets. Part 2 (TX bonding)
is out of scope for now.

Each radio captures a *wider* slice that shares a configurable overlap with the
other; the overlap is used to phase/gain-align the two independent radios and to
crossfade them into a contiguous channel; out-of-band corners are discarded.

## 2. Hard requirement from the user

**All signal-processing/DSP logic lives in C++.** Python is ONLY a thin
test/flowgraph interface — never put important DSP in Python. The user does not
trust the pre-existing (archived) code; rewriting from scratch is fine.

## 3. Architecture

```
Radio A (B210 @ fc-T/4) ┐ shared 10 MHz + PPS
Radio B (B210 @ fc+T/4) ┘
   │ (C++) uhd::usrp::bonded::bonded_receiver  — synchronized acquisition
   │        (lock 10 MHz, detect PPS, latch common time=0; <1 µs aligned)
   ▼ (C++) uhd::usrp::bonded::overlap_reconstructor — the reconstruction DSP
   │        one contiguous combined-rate baseband at fc
   ▼ gr-ieee802-11 receiver — Schmidl-Cox → OFDM FFT → equalize → Viterbi → MAC
   ▼ decoded frames (counted / pcap)
```

C++ engines: `lib/usrp/bonded/`. GNU Radio wrappers: `gr-bonded-usrp/`. Driver
script: `samples/wifi_bonded_rx.py`.

## 4. Key files

**New this session (untracked — not yet committed):**
- `lib/usrp/bonded/overlap_reconstructor.{hpp,cpp}` — THE reconstruction engine
  (time-domain VOLK pipeline). Self-contained.
- `tests/overlap_reconstructor_test.cpp` — ctest (no hardware): alignment
  recovery, band-flatness, seam, streaming.
- `gr-bonded-usrp/` — OOT module: blocks `bonded_source` (acquisition) and
  `overlap_reconstructor` (2 complex in → 1 complex out), pybind11 bindings, GRC
  yml, CMake.
- `samples/wifi_bonded_rx.py` — thin flowgraph, modes `live|record|replay`.
- `samples/test_overlap_block.py` — hardware-free GR smoke test.
- `samples/OFFLINE_20MHZ_DECODE.md` — full algorithm + reproduction + panel notes.
- `samples/archive/` — superseded experiments (old hard-split pipeline, phase6_3
  runs, perf automation, captures). Do NOT resurrect these.

**Modified by me:** `lib/usrp/bonded/CMakeLists.txt` (added overlap_reconstructor
source + FFTW/VOLK/OpenMP find+link + per-file -O3/-ffast-math/-march=native),
`tests/CMakeLists.txt` (added test + include dir for `lib/usrp/bonded`).

**Pre-existing / reused (the `M` on bonded_receiver.* predates this session):**
- `bonded_receiver.{hpp,cpp}` — synchronized acquisition. Reused as-is; it is
  config-driven (any rate, per-device `freq_plan`). PPS/MCR sync is proven
  (measured 0.000 µs spread); do NOT destabilize it.
- `frequency_plan.{hpp,cpp}` — adjacent-band plan helper (supports N radios).
- `spectrum_stitcher.{hpp,cpp}` — power-spectrum stitch; **NOT used for decode**
  (discards phase). Display only.

Older design docs (still in repo root): `BONDING_IMPLEMENTATION_PLAN_FLEXIBLE.md`,
`BONDING_TX_IMPLEMENTATION_PLAN.md`.

## 5. The reconstruction algorithm (time-domain, VOLK)

Per radio: (1) polyphase rational resample per_radio→T; (2) NCO rotator to
translate the slice to the fc-centred baseband; then (3) estimate the static
inter-radio **gain+phase** over the shared overlap and apply it to radio B; (4)
combine with complementary complex crossfade FIRs `out = Hlow(A) + Hhigh(α·B)`
(weights cos²+sin²=1; filters are 0 outside the channel → corners discarded);
(5) feed gr-ieee802-11. Full math/rationale in `samples/OFFLINE_20MHZ_DECODE.md`.

Critical correctness points (don't regress):
- **Complex (IQ) reconstruction** — phase must be preserved (OFDM). Power-spectrum
  stitching is unusable.
- **Gain estimate = power ratio** `sqrt(Σ|A|²/Σ|B|²)`, NOT `|Σ A·conj(B)|/Σ|B|²`.
  The latter collapses to 0 on bursty WiFi (overlap is mostly noise) → "0 packets
  decoded" (this was a real bug).
- **Alignment is static** (shared 10 MHz clock): estimate during a 200k-sample
  warmup, then **freeze** and skip the overlap-analysis filters.

## 6. Configurable overlap (general plan)

`--samp-rate T` + optional `--overlap O`:
```
per-radio capture = T/2 + O      radio centres = fc ± T/4
corner discarded  = O/2 per radio   (default O = 0.25·T → 15 MHz/radio @ T=20)
```
Crossfade FIR length auto-scales ~`8·T/O` (clamped 17..257) in
`overlap_reconstructor::impl`. **Counter-intuitive tradeoff (verified):** a
SMALLER overlap lowers per-radio USB bandwidth but RAISES reconstruction CPU
(sharper transition → longer filters). Reconstruction stays band-flat (±0.4%)
at O=5/2/1 MHz. The CPU knob that matters is T, not O.

## 7. Build / install workflow (IMPORTANT gotchas)

UHD build dir: `~/uhd/host/build` (CMAKE_BUILD_TYPE=Release, configured).
GR module build dir: `~/uhd/host/gr-bonded-usrp/build`.

After editing `lib/usrp/bonded/*`:
```bash
cd ~/uhd/host/build && make uhd overlap_reconstructor_test -j16
ctest -R overlap_reconstructor --output-on-failure        # validate
sudo cp -P lib/libuhd.so* /usr/local/lib/ && sudo ldconfig # FAST install (lib only)
# (sudo make install also works but rebuilds all tests — slow)
```

**The gr-bonded-usrp module links the INSTALLED libuhd**, so install libuhd
*before* rebuilding the module. And **rebuild + reinstall the GR module whenever
a C++ HEADER changed** (class layout changes → otherwise it crashes with
`free(): invalid pointer`):
```bash
cd ~/uhd/host/gr-bonded-usrp/build && cmake .. && make -j16 \
  && sudo make install && sudo ldconfig
```

Gotchas:
- If cmake errors with "Permission denied" in build/: prior `sudo make install`
  left root-owned files → `sudo chown -R gabriel:gabriel build`.
- Optional deps auto-detected and linked into libuhd for the reconstructor:
  **FFTW3f, VOLK, OpenMP** (see the blocks added in `lib/CMakeLists.txt`). They
  exist on this machine. The reconstructor source is built with
  `-O3 -ffast-math -funroll-loops -march=native`.
- `gr-ieee802-11`, `gr-foo`, `gnuradio` 3.10 are installed; `bonded_usrp` Python
  module imports from `/usr/local/lib/python3.12/dist-packages/gnuradio/`.

## 8. Current status

- **Reconstruction algorithm: DONE & validated** (ctest green — alignment
  recovers injected gain/phase exactly, ±0.4% band-flat, −65 dB seam, streaming
  matches). Works through the GR block + flowgraph (smoke test passes).
- **Offline record→replay: implemented, lossless** — the validated path to
  demonstrate decode. **NOT yet run on real B210s** → that on-air run is the main
  pending validation.
- **Live real-time: NOT achievable at 20 MHz** (~0.3×); marginal at 5 MHz.
  See §9. This is documented honestly as a runtime constraint; the scientific
  claim is validated offline at full 20 MHz.

## 9. Real-time exploration (so we don't repeat it)

Throughput at 20 MHz combined, pure DSP (this 16-core CPU):
`double radix-2 0.03× → float+poly+NCO 0.07× → FFTW 0.14× → +OpenMP/batching 0.29×`;
`time-domain VOLK + frozen align + scratch reuse ≈ 0.30×`. Scales ~inversely with
rate (≈0.58× @10 MHz, ≈1.05× @5 MHz pure DSP; ≈0.64× @5 MHz in a synthetic GR
flowgraph). **OpenMP barely helps** — the parallelizable part is only ~8%; the
serial resampler/FIRs + per-output VOLK call overhead + GR harness dominate.
Next real-time avenues (not done): **fused single complex-polyphase filter per
radio** (one VOLK dot/output instead of ~4 calls — the most promising), a lower
target rate, or GPU.

## 10. Hardware setup

4× B210. Roles used: **TX** `30DBC3D`; **bonded RX pair** `30B56D6` (Radio A,
lower) + `30DBC3C` (Radio B, upper) — these two MUST share an external 10 MHz
ref + PPS; **single-radio baseline** `30EDB63`. Default `fc = 2.412 GHz`.

## 11. Quick validation commands (full detail in OFFLINE_20MHZ_DECODE.md §8)

```bash
# 0. no hardware
cd ~/uhd/host/build && ctest -R overlap_reconstructor --output-on-failure
cd ~/uhd/host/samples && python3 test_overlap_block.py
# 1. TX (term 1)
./wifi_trx.py --mode tx --samp-rate 20e6 --tx-serial 30DBC3D
# 2. baseline single-radio (term 2)
./wifi_trx.py --mode rx --samp-rate 20e6 --rx-serial 30EDB63 --pcap baseline.pcap
# 3. record bonded slices (term 3; TX must be running)
timeout 8 ./wifi_bonded_rx.py --mode record --serial-a 30B56D6 --serial-b 30DBC3C --rec-prefix cap --rx-gain-db 40
# 4. reconstruct + decode offline (no radios)
./wifi_bonded_rx.py --mode replay --rec-prefix cap --pcap bonded.pcap
tcpdump -r bonded.pcap | wc -l   # vs baseline.pcap
```

## 12. TODO when resuming (in priority order)

1. **Run the offline record→replay on the real B210s** and compare decoded frame
   count to the single-radio 20 MHz baseline. This is the headline thesis result.
   Tune `--rx-gain-db` (30/40/50) and `--chan-est 0..3` if needed.
2. Capture a spectrum plot of `cap_a.fc32`, `cap_b.fc32`, and the reconstructed
   output for the presentation (shows the two slices + seamless result).
3. (Optional) Live real-time: implement the **fused polyphase filter** (§9) or
   demo at a lower rate (`--samp-rate 5e6`).
4. (Cleanup) `--fft-size` on `wifi_bonded_rx.py` and the GR block is now
   **vestigial** (time-domain path ignores it) — remove it.
5. (Optional) Extend overlap reconstruction to N>2 radios (frequency_plan
   already generalizes the layout).
6. Decide whether/when to `git commit` (currently all new bonded work is
   untracked on branch `gl/add-sync-test`; nothing committed this session).

## 13. Lessons / do-not-redo

- Do NOT reintroduce the hard-split or the power-spectrum stitcher.
- Keep the power-ratio gain estimator.
- Keep all DSP in C++; Python stays a thin interface.
- Don't trust `samples/archive/`.
- Memory notes: `[[bonding-overlap-design]]`, `[[bonding-dsp-in-cpp]]`.
