# Phase 2: Synchronization Capture & Analysis

This directory contains scripts to capture and analyze synchronization between two or more USRP B210 devices.

## Prerequisites

- Python 3.7+
- UHD Python bindings (`uhd`)
- numpy, scipy, matplotlib
- Two or more B210s connected via USB

Install dependencies using a virtual environment (required on Ubuntu 24.04+ / Debian 12+ due to PEP 668).
The `uhd` Python bindings are installed system-wide by the UHD build; grant the venv access to them with `--system-site-packages`:

```bash
# One-time: install the venv package if not already present
sudo apt install python3-venv

python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install numpy scipy matplotlib
```

Activate the venv before running any script:

```bash
source .venv/bin/activate
```

---

## 1. Capture Synchronized Samples

```
capture_sync.py --serials <S0> <S1> [S2 ...] [options]
```

Supports **2 to 8 devices**. All devices are initialized, synchronized, and captured simultaneously.

### Key options

| Option               | Default      | Description                                                          |
| -------------------- | ------------ | -------------------------------------------------------------------- |
| `--serials`          | _(required)_ | Serial numbers of B210s to use (2–8 devices)                         |
| `--clock-source`     | `internal`   | `internal` \| `external` \| `gpsdo`                                  |
| `--time-source`      | `internal`   | `internal` \| `external` \| `gpsdo`                                  |
| `--rate`             | `1e6`        | Sample rate (Hz)                                                     |
| `--freq`             | `915e6`      | Center frequency (Hz)                                                |
| `--gain`             | `30`         | RX gain (dB)                                                         |
| `--nsamps`           | `10000000`   | Samples to capture per device                                        |
| `--timed`            | off          | Force timed start even with internal time source                     |
| `--ref-lock-timeout` | `10`         | Seconds to wait for external clock lock                              |
| `--pps-timeout`      | `12`         | Seconds to wait for PPS detection                                    |
| `--tx-serial`        | _(off)_      | Serial of the device that transmits a CW tone during capture         |
| `--tx-gain`          | `60`         | TX gain in dB (only used when `--tx-serial` is set)                  |
| `--tx-offset`        | `100e3`      | Tone IF offset from center frequency in Hz (only with `--tx-serial`) |

Timed capture (synchronized start on a PPS edge) is enabled automatically when `--time-source` is `external` or `gpsdo`. Use `--timed` to force it with internal time.

All devices are armed with the **same absolute start timestamp** before any starts receiving, then all RX streams are drained in parallel threads. This guarantees the captures begin at the same point in time regardless of USB scheduling latency.

### Built-in CW tone transmitter

Pass `--tx-serial <S>` (where `<S>` is one of `--serials`) to make that device transmit a continuous-wave tone during capture. This gives `analyze_sync.py` a coherent signal to measure against instead of noise.

- The tone is placed at `--tx-offset` Hz above the center frequency (default **+100 kHz**). A non-zero offset avoids LO self-mixing leakage that appears at DC on every SDR.
- On a B210, TX uses the **TX/RX** SMA port and RX uses the **RX2** SMA port. Both should have antennas attached.
- The tone starts before arming the RX streams and stops after all captures complete.

> **USB bandwidth note:** each B210 at 1 MHz needs ~8 MB/s. With 3+ devices on the same USB controller you may see occasional overflows, which are handled gracefully. Using ports connected to separate USB controllers (separate PCIe root ports) is recommended for 4+ devices.

### Sync mode examples

**Internal only** (no external hardware, baseline comparison):

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

**PPS-only** (shared PPS signal, each device uses its own TCXO):

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C \
  --time-source external \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

**Optimal** (shared 10 MHz reference + shared PPS):

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C \
  --clock-source external --time-source external \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

**Three devices — optimal sync + TX tone:**

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C THIRD_SERIAL \
  --tx-serial 30B56D6 \
  --clock-source external --time-source external \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

**Optimal + built-in TX tone** (recommended for phase coherence measurement):

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C \
  --tx-serial 30B56D6 \
  --clock-source external --time-source external \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

**GPSDO** (a GPSDO module is installed in the internal header of each device):

```bash
python3 capture_sync.py \
  --serials 30B56D6 30DBC3C \
  --clock-source gpsdo --time-source gpsdo \
  --rate 1e6 --nsamps 10000000 --freq 915e6 --gain 30
```

The script saves one `.npy` file and one `_meta.txt` file per device, recording the serial, sample rate, frequency, clock source, and time source used.

---

## 2. Analyze Synchronization

```
analyze_sync.py --files <file0.npy> <file1.npy> [file2.npy ...] [options]
```

Device 0 (the first file) is the **reference**. Metrics are computed for every pair `(0, i)`. An odd number of devices is fully supported — you get N-1 independent pairwise results.

### Key options

| Option          | Default      | Description                                                                                                                          |
| --------------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `--files`       | _(required)_ | IQ `.npy` files from `capture_sync.py` (first file is the reference)                                                                 |
| `--rate`        | `1e6`        | Sample rate in Hz (auto-read from metadata if available)                                                                             |
| `--tone-offset` | _(off)_      | IF offset (Hz) of the CW TX tone used during capture (e.g. `100e3`)                                                                  |
| `--save-plot`   | _(show)_     | Save plot(s) to this file. With multiple pairs the pair index is appended before the extension (e.g. `report_0.png`, `report_1.png`) |

### When to use `--tone-offset`

Always pass `--tone-offset` when the capture was made with `--tx-serial`. It enables three improvements:

1. **Cross-correlation period disambiguation** — a CW tone produces cross-correlation peaks at every multiple of its period ($T = \text{rate} / f_\text{offset}$). For a 100 kHz tone at 1 MHz rate that is every 10 samples. The flag folds the raw peak lag into the interval $(-T/2,\, T/2]$, recovering the true sub-period offset.

2. **Segmented FFT phase analysis** — per-sample phase estimation fails when the per-sample SNR is negative (a 23 dB FFT SNR over 1M samples is only −37 dB per sample). The script instead divides the signal into 50 ms segments and reads the FFT bin at the tone frequency in each segment. The FFT coherently integrates all samples in the segment, so the per-segment SNR is high even when the per-sample SNR is low. Phase is then unwrapped across the much smaller set of segment phases.

3. **Accurate SNR reporting** — the report prints the tone SNR (integrated over the full capture) for each device, making it easy to see whether the result is measurement-noise limited or genuinely poor.

### Example

```bash
# Two devices, no TX tone:
python3 analyze_sync.py \
  --files capture_30B56D6.npy capture_30DBC3C.npy

# Two devices, with TX tone at +100 kHz (recommended):
python3 analyze_sync.py \
  --files capture_30B56D6.npy capture_30DBC3C.npy \
  --tone-offset 100e3 --save-plot report.png

# Three devices (device 0 is reference; two pairwise reports generated):
python3 analyze_sync.py \
  --files capture_30B56D6.npy capture_30DBC3C.npy capture_THIRD.npy \
  --tone-offset 100e3

# Save plots to files (produces report_0.png, report_1.png for 3 devices):
python3 analyze_sync.py \
  --files capture_30B56D6.npy capture_30DBC3C.npy capture_THIRD.npy \
  --tone-offset 100e3 --save-plot report.png
```

The script reads clock/time source information from the companion `_meta.txt` files automatically and includes them in the printed report.

### Output

Printed summary (two-device example):

```
======================================================
  Synchronization Analysis Report
======================================================
  Reference    : 30B56D6
  Pair 1       : 30DBC3C
  Clock source : external
  Time source  : external
  Sample rate  : 1.000 MHz
------------------------------------------------------
  [ Pair 0: 30B56D6 → 30DBC3C ]
  Tone SNR (ref)  : +47.3 dB
  Tone SNR (dev)  : +57.2 dB
  Time offset     : +0.088 samples  (+0.088 µs)
  Freq offset     : +0.000 Hz
  Phase coherence : 0.311 rad (stddev)
  Time sync       : GOOD  (threshold <10 µs)
  Freq sync       : GOOD  (threshold <10 Hz)
  Phase coherence : GOOD  (threshold <0.5 rad)
======================================================
```

With three devices a second `[ Pair 1: 30B56D6 → THIRD_SERIAL ]` block follows, then the final separator.

Three plots:

1. **Cross-correlation magnitude** — time offset between captures
2. **Phase difference per segment** — wrapped per-segment phase, shows coherence
3. **Unwrapped phase difference + LS fit** — linear trend gives frequency offset; residuals give phase coherence

**Tone SNR lines** appear only when `--tone-offset` is given. If either device reports SNR < 10 dB, the freq and phase verdicts are marked `N/A (low SNR)` and a hint is printed.

If results are POOR the script prints a hint explaining which hardware is missing and which flag to add.

### Interpreting the plots

#### Plot 1 — Cross-correlation magnitude

This plot answers: **did the two captures start at the same moment?**

The x-axis is lag in samples. You are looking for a **single sharp spike near lag = 0**, sitting clearly above the noise pedestal (the broad hill shape caused by the Hann window applied before correlation).

| What you see                                    | Meaning                                                                      |
| ----------------------------------------------- | ---------------------------------------------------------------------------- |
| Sharp spike at ~0                               | Captures are time-aligned — time sync is working                             |
| Spike far from 0 (e.g. ±thousands of samples)   | Captures started at different times — check PPS cabling or timed-start logic |
| Multiple peaks of equal height spaced regularly | CW tone ambiguity — add `--tone-offset` to resolve                           |
| No clear spike, only flat noise                 | No coherent signal received — noise-only captures always look like this      |

The title reports the sub-sample interpolated lag and its equivalent in microseconds.

#### Plot 2 — Wrapped phase difference per segment

This plot answers: **is the phase relationship between the two devices stable over time?**

Each dot is the inter-device phase difference measured from one 50 ms segment's FFT (with `--tone-offset`) or from each sample (without). The phase is wrapped to the interval (−π, +π].

A constant phase offset between two synchronized devices is **normal and expected** — the RF path from TX to each RX has a different physical length, so the two received signals naturally have different absolute phases. What matters is that this offset stays **stable**.

| What you see                                  | Meaning                                                                                                     |
| --------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| Dots clustered tightly at a constant value    | Good phase coherence — shared reference is working                                                          |
| Dots near ±π jumping between +3 and −3 rad    | Not instability — this is a single stable value straddling the ±π branch cut; unwrap in Plot 3 will confirm |
| Dots slowly drifting across the full ±π range | Frequency offset between devices — expect this with `--clock-source internal`                               |
| Dots scattered randomly across ±π             | No coherent signal, or insufficient SNR (check Tone SNR lines in the report)                                |

The title shows the residual stddev (phase coherence) after removing the linear frequency trend.

#### Plot 3 — Unwrapped phase difference + least-squares fit

This plot answers: **is there a frequency offset between the two devices, and how large is the phase wander around it?**

`np.unwrap()` removes the ±π discontinuities from Plot 2, revealing the continuous phase evolution. The dashed line is the least-squares linear fit:

- **Slope** → frequency offset (Hz). A flat line means both LOs are locked to the same reference.
- **Scatter around the line** → phase coherence. The tighter the orange dots cluster around the dashed line, the more phase-coherent the two devices are.

| What you see                    | Meaning                                                                                        |
| ------------------------------- | ---------------------------------------------------------------------------------------------- |
| Flat dashed line, tight scatter | Shared 10 MHz reference working; low phase wander                                              |
| Sloped dashed line              | Frequency offset — expected with `--clock-source internal` (2–5 ppm at 915 MHz ≈ 1800–4600 Hz) |
| Flat line but large scatter     | RF path instability (multipath, air movement); or low SNR                                      |
| Irregular/noisy orange trace    | Per-sample SNR too low — use `--tone-offset` and capture more samples                          |

The phase coherence shown in the printed report is the **standard deviation of the residuals** (orange dots minus the dashed line), not the raw wrapped phase. This correctly separates a constant frequency offset from true phase instability.

---

## Sync mode comparison

| Mode          | `--clock-source` | `--time-source` | Time offset | Freq offset | Phase coherence |
| ------------- | ---------------- | --------------- | ----------- | ----------- | --------------- |
| Internal only | `internal`       | `internal`      | > 100 µs    | 2–5 ppm     | Not meaningful  |
| PPS-only      | `internal`       | `external`      | < 10 µs     | 2–5 ppm     | Not meaningful  |
| Optimal       | `external`       | `external`      | < 1 µs      | < 0.01 ppm  | < 0.5 rad       |
| GPSDO         | `gpsdo`          | `gpsdo`         | < 1 µs      | < 0.001 ppm | < 0.5 rad       |

Phase coherence is only meaningful when a coherent RF signal is present at both receivers (use `--tx-serial`). Noise-only captures will always show poor coherence regardless of hardware sync quality.

---

See script docstrings for full option details.
