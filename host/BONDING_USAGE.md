# PPS-Only Bonded Multi-USRP — User Guide

Two USRP B210s sharing a GPS PPS pulse are combined into a single wideband
receiver. Each device keeps its own independent TCXO (no 10 MHz cable
required); the PPS timestamps are used to measure and continuously correct
for the oscillator offset between the two boards.

## Contents

1. [Hardware setup](#1-hardware-setup)
2. [Building](#2-building)
3. [CLI tools](#3-cli-tools)
   - [bonded_wideband_rx](#31-bonded_wideband_rx)
   - [bonded_drift_monitor](#32-bonded_drift_monitor)
   - [bonded_spectrum_monitor](#33-bonded_spectrum_monitor)
4. [C++ API quick-start](#4-c-api-quick-start)
5. [GNU Radio integration](#5-gnu-radio-integration)
6. [Tuning and band layout](#6-tuning-and-band-layout)
7. [Quality metrics reference](#7-quality-metrics-reference)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Hardware setup

```
GPS antenna
    |
 GPS module  (e.g. u-blox NEO-8M, trimble, any 3.3 V CMOS PPS)
    |
  1-PPS output  ──┬── SMA cable ──► B210 #0  "PPS" SMA
                  └── SMA cable ──► B210 #1  "PPS" SMA
```

- Each B210 uses its **internal TCXO** for sampling. Do **not** connect
  10 MHz between the boards — that would over-constrain the system and
  defeat the drift estimation.
- The PPS signal is only used to align hardware timestamps. RF signals
  on the two RX chains are completely independent.
- Both USBs plug directly into the host. A powered USB 3 hub works too.

---

## 2. Building

The bonded layer is compiled into `libuhd` automatically during a normal
CMake build, provided `fftw3f` and `volk` are available.

```bash
# Debian / Ubuntu prerequisites
sudo apt install libvolk-dev libfftw3-dev

cd /path/to/uhd/host
mkdir -p build && cd build
cmake ..
make -j$(nproc) uhd bonded_wideband_rx bonded_drift_monitor bonded_spectrum_monitor
```

CMake will print confirmation lines:

```
-- Building bonded PPS examples (fftw3f + volk found)
-- Building bonded PPS unit tests (volk found)
```

Unit tests:

```bash
make pps_drift_test phase_corrector_test
./tests/pps_drift_test
./tests/phase_corrector_test
```

---

## 3. CLI tools

All tools accept `--serial0` / `--serial1` to select specific devices.
Omitting both causes UHD to auto-detect all connected USRPs (first two
found are used).

Find serial numbers:

```bash
uhd_find_devices
# output example:
# [000] B200
#     serial: 30B56D6
# [001] B200
#     serial: 30DBC3C
```

### 3.1 `bonded_wideband_rx`

Captures wideband baseband data and optionally writes a binary `fc32` file.

| Option       | Default  | Description                                                |
| ------------ | -------- | ---------------------------------------------------------- |
| `--serial0`  | _(auto)_ | Serial number of device 0 (lower sideband)                 |
| `--serial1`  | _(auto)_ | Serial number of device 1 (upper sideband)                 |
| `--freq`     | `2400e6` | Center of the **combined** band (Hz)                       |
| `--rate`     | `20e6`   | Per-device sample rate (S/s); combined bandwidth ≈ 2× this |
| `--gain`     | `40`     | RX gain applied to all channels (dB)                       |
| `--duration` | `10.0`   | Capture duration (seconds); 0 = run until Ctrl+C           |
| `--fft-size` | `4096`   | FFT size for stitching                                     |
| `--outfile`  | _(none)_ | Binary output file path (raw `fc32` spectrum bins)         |

**Examples:**

```bash
# 10-second capture, 40 MHz combined BW centred on 2.4 GHz
./examples/bonded_wideband_rx \
    --serial0=30B56D6 --serial1=30DBC3C \
    --freq=2400e6 --rate=20e6 --gain=40 --duration=10

# Save to file for offline processing
./examples/bonded_wideband_rx \
    --serial0=30B56D6 --serial1=30DBC3C \
    --freq=2400e6 --rate=20e6 --duration=60 \
    --outfile=capture.fc32

# 60 MHz combined BW at 915 MHz
./examples/bonded_wideband_rx \
    --freq=915e6 --rate=30e6 --gain=50
```

**Output file format:** Each `recv()` call appends `output_size()` complex
float samples (interleaved I/Q, little-endian `float32`) representing the
stitched FFT spectrum. Load in Python:

```python
import numpy as np
data = np.fromfile("capture.fc32", dtype=np.complex64)
# data.shape = (n_spectra * output_bins,)
# reshape:
n_bins = 4096 - 2 * 128  # default fft_size minus 2 guard-band regions
spectra = data.reshape(-1, n_bins)
```

Quality metrics are printed to stdout every 10 seconds:

```
[quality] drift=+0.47 ppm  unc=0.03 ppm  PPS_samples=47  phase_rate=+3.54 rad/s
```

---

### 3.2 `bonded_drift_monitor`

Diagnostic tool. **No RF required** — only needs PPS. Useful to verify
GPS lock and characterise board-to-board frequency offset before committing
to a capture.

| Option       | Default  | Description                                                |
| ------------ | -------- | ---------------------------------------------------------- |
| `--serial0`  | _(auto)_ | Serial number of device 0                                  |
| `--serial1`  | _(auto)_ | Serial number of device 1                                  |
| `--rf-freq`  | `2.4e9`  | RF frequency used to compute the displayed phase rate (Hz) |
| `--duration` | `0`      | How long to run (0 = until Ctrl+C)                         |
| `--csv`      | _(none)_ | Optional CSV output file                                   |

**Examples:**

```bash
# Watch drift converge live (stop with Ctrl+C)
./examples/bonded_drift_monitor \
    --serial0=30B56D6 --serial1=30DBC3C

# Record 10 minutes of drift history to a CSV
./examples/bonded_drift_monitor \
    --serial0=30B56D6 --serial1=30DBC3C \
    --rf-freq=2400e6 --duration=600 \
    --csv=drift_log.csv
```

Example output:

```
t=  5 s  drift=+0.47 ppm  unc=0.42 ppm  n= 5  (estimating...)
t= 10 s  drift=+0.47 ppm  unc=0.21 ppm  n=10  OK
t= 30 s  drift=+0.47 ppm  unc=0.07 ppm  n=30  OK
```

The CSV file has columns: `elapsed_s, drift_ppm, uncertainty_ppm, num_samples, phase_rate_rad_s`.

---

### 3.3 `bonded_spectrum_monitor`

Real-time ASCII spectrum display. Updates the terminal in-place using
ANSI escape sequences.

| Option       | Default  | Description                                              |
| ------------ | -------- | -------------------------------------------------------- |
| `--serial0`  | _(auto)_ | Serial number of device 0                                |
| `--serial1`  | _(auto)_ | Serial number of device 1                                |
| `--freq`     | `2400e6` | Center of the combined band (Hz)                         |
| `--rate`     | `20e6`   | Per-device sample rate (S/s)                             |
| `--gain`     | `40`     | RX gain (dB)                                             |
| `--fft-size` | `4096`   | FFT size                                                 |
| `--bins`     | `80`     | Number of display columns (spectrum is averaged to this) |
| `--avg`      | `10`     | Spectra to average before each display refresh           |

**Example:**

```bash
./examples/bonded_spectrum_monitor \
    --serial0=30B56D6 --serial1=30DBC3C \
    --freq=2450e6 --rate=20e6 --gain=50 --bins=80
```

Sample display:

```
Bonded wideband spectrum  centre=2450.0 MHz  span=40.0 MHz
──────────────────────────────────────────────────────────
  -80 dB [ 2430.0 MHz ] ########
  -65 dB [ 2432.5 MHz ] #####################
  -72 dB [ 2447.5 MHz ] ###############
  drift=+0.47 ppm  unc=0.03 ppm  PPS n=58  rate=3.54 rad/s
```

---

## 4. C++ API quick-start

```cpp
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/usrp/multi_usrp.hpp>

// 1. Open devices
auto usrp = uhd::usrp::multi_usrp::make("serial0=30B56D6,serial1=30DBC3C");

// 2. PPS-only sync — internal clock on each board, external PPS for time
for (size_t mb = 0; mb < usrp->get_num_mboards(); ++mb) {
    usrp->set_clock_source("internal", mb);
    usrp->set_time_source("external", mb);
}
usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
std::this_thread::sleep_for(std::chrono::seconds(2)); // wait for PPS to latch

// 3. Set gain
for (size_t ch = 0; ch < usrp->get_rx_num_channels(); ++ch)
    usrp->set_rx_gain(40.0, ch);

// 4. Create bonded streamer
//    Arguments: usrp, per_device_rate, combined_center_freq [, fft_size]
uhd::usrp::bonded::bonded_rx_streamer bonded(usrp, 20e6, 2400e6);
bonded.start();

// 5. Receive loop
std::vector<std::complex<float>> buf(bonded.output_size());
uhd::rx_metadata_t md;

while (running) {
    size_t n = bonded.recv(buf.data(), buf.size(), md, /*timeout=*/1.0);
    if (n == 0) continue;  // overflow or timeout — safe to retry

    // buf[0..n-1] = stitched wideband FFT spectrum (complex float)
    auto q = bonded.quality();
    // q.drift_ppm, q.uncertainty_ppm, q.pps_samples_used, q.valid, ...
}

bonded.stop();
```

The `output_size()` method returns the number of usable FFT bins across
both devices combined (approximately `fft_size - 2*guard_bins`, see
[§6](#6-tuning-and-band-layout)).

---

## 5. GNU Radio integration

The bonded layer does not ship a dedicated GNU Radio block, but can be
driven from a Python block or from out-of-tree C++.

### 5.1 Python block (gr-uhd companion)

Create a GNU Radio Python block that wraps `bonded_rx_streamer` via the
UHD Python bindings. Since `bonded_rx_streamer` is a standard UHD API
class (exported with `UHD_API`), the UHD Python bindings module can access
it directly.

```
GNU Radio Companion layout
──────────────────────────
[ Bonded RX Source (Python) ] ──fc32──► [ Vector to Stream ]
                                                    │
                                          ┌─────────┴──────────┐
                                   [ FFT Display ]     [ File Sink ]
```

**Python block source — `bonded_rx_source.py`:**

```python
import numpy as np
import uhd
from gnuradio import gr

class bonded_rx_source(gr.sync_block):
    """
    GNU Radio source block wrapping uhd::usrp::bonded::bonded_rx_streamer.

    Output port 0: vector of complex64, length = output_bins
    """

    def __init__(self, serial0, serial1, center_freq, sample_rate,
                 gain=40.0, fft_size=4096):
        self._output_bins = fft_size  # approximate; updated after streamer init

        # Open USRP
        addr = ""
        if serial0 and serial1:
            addr = f"serial0={serial0},serial1={serial1}"
        self._usrp = uhd.usrp.MultiUSRP(addr)

        if self._usrp.get_num_mboards() < 2:
            raise RuntimeError("Need ≥ 2 mboards for bonded operation")

        # PPS-only sync
        for mb in range(self._usrp.get_num_mboards()):
            self._usrp.set_clock_source("internal", mb)
            self._usrp.set_time_source("external", mb)
        self._usrp.set_time_unknown_pps(uhd.types.TimeSpec(0.0))

        import time
        time.sleep(2)  # wait for PPS edge to latch

        for ch in range(self._usrp.get_rx_num_channels()):
            self._usrp.set_rx_gain(gain, ch)

        # Create bonded streamer
        self._streamer = uhd.usrp.bonded.BondedRxStreamer(
            self._usrp, sample_rate, center_freq, fft_size)
        self._output_bins = self._streamer.output_size()

        gr.sync_block.__init__(
            self,
            name="Bonded RX Source",
            in_sig=None,
            out_sig=[(np.complex64, self._output_bins)])

        self._streamer.start()

    def work(self, input_items, output_items):
        md = uhd.types.RXMetadata()
        n = self._streamer.recv(output_items[0][0], self._output_bins, md, 1.0)
        if n == 0:
            return 0  # overflow / timeout — GR will call work() again
        return 1  # produced 1 output vector

    def stop(self):
        self._streamer.stop()
        return True
```

> **Note:** The Python binding name `uhd.usrp.bonded.BondedRxStreamer` is
> illustrative. UHD's Python binding generator wraps C++ class names with
> CamelCase. If your UHD build does not auto-generate these bindings you
> can use `ctypes` / `cffi` against `libuhd.so` or write a thin pybind11
> wrapper. See §5.2 for the pybind11 approach.

**Use in GRC (.grc):**

```yaml
# Add this to your .grc flow graph as an "Import" block:
import sys
sys.path.insert(0, "/path/to/bonded_rx_source.py")
from bonded_rx_source import bonded_rx_source
```

Then add a `Python Block` in GRC referring to the class above.

---

### 5.2 Minimal pybind11 wrapper (standalone)

If you prefer to compile a thin Python module instead of relying on UHD's
auto-generated bindings, add this file to your project:

```cpp
// bonded_py.cpp
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace uhd::usrp::bonded;

PYBIND11_MODULE(bonded_py, m) {
    py::class_<bonding_quality_metrics>(m, "BondingQualityMetrics")
        .def_readonly("drift_ppm",        &bonding_quality_metrics::drift_ppm)
        .def_readonly("uncertainty_ppm",  &bonding_quality_metrics::uncertainty_ppm)
        .def_readonly("pps_samples_used", &bonding_quality_metrics::pps_samples_used)
        .def_readonly("phase_rate_rad_s", &bonding_quality_metrics::phase_rate_rad_s)
        .def_readonly("pps_healthy",      &bonding_quality_metrics::pps_healthy)
        .def_readonly("valid",            &bonding_quality_metrics::valid);

    py::class_<bonded_rx_streamer>(m, "BondedRxStreamer")
        .def(py::init<uhd::usrp::multi_usrp::sptr, double, double, size_t>(),
             py::arg("usrp"), py::arg("sample_rate"), py::arg("center_freq"),
             py::arg("fft_size") = bonded_rx_streamer::DEFAULT_FFT_SIZE)
        .def("start",       &bonded_rx_streamer::start)
        .def("stop",        &bonded_rx_streamer::stop)
        .def("output_size", &bonded_rx_streamer::output_size)
        .def("quality",     &bonded_rx_streamer::quality)
        .def("recv",
            [](bonded_rx_streamer& self,
               py::array_t<std::complex<float>> arr,
               double timeout) {
                py::buffer_info buf = arr.request();
                uhd::rx_metadata_t md;
                size_t n = self.recv(
                    static_cast<std::complex<float>*>(buf.ptr),
                    static_cast<size_t>(buf.size),
                    md, timeout);
                return py::make_tuple(n, (int)md.error_code);
            },
            py::arg("output"), py::arg("timeout") = 1.0);
}
```

Build with:

```bash
c++ -O2 -shared -fPIC $(python3 -m pybind11 --includes) \
    bonded_py.cpp -o bonded_py$(python3-config --extension-suffix) \
    -luhd $(pkg-config --libs fftw3f volk)
```

Then in a GNU Radio Python block:

```python
import numpy as np
import uhd
import bonded_py
from gnuradio import gr

class bonded_rx_source(gr.sync_block):
    def __init__(self, serial0, serial1, center_freq, sample_rate,
                 gain=40.0, fft_size=4096):
        usrp = uhd.usrp.MultiUSRP(f"serial0={serial0},serial1={serial1}")
        for mb in range(usrp.get_num_mboards()):
            usrp.set_clock_source("internal", mb)
            usrp.set_time_source("external", mb)
        usrp.set_time_unknown_pps(uhd.types.TimeSpec(0.0))
        import time; time.sleep(2)
        for ch in range(usrp.get_rx_num_channels()):
            usrp.set_rx_gain(gain, ch)

        self._streamer = bonded_py.BondedRxStreamer(
            usrp, sample_rate, center_freq, fft_size)
        n = self._streamer.output_size()

        gr.sync_block.__init__(self, "Bonded RX", in_sig=None,
                               out_sig=[(np.complex64, n)])
        self._n = n
        self._streamer.start()

    def work(self, input_items, output_items):
        buf = np.frombuffer(output_items[0][0], dtype=np.complex64)
        count, _ = self._streamer.recv(buf, 1.0)
        return 1 if count > 0 else 0

    def stop(self):
        self._streamer.stop()
        return True
```

### 5.3 Complete GRC flow graph example

```
┌──────────────────────────────────────────────────────────────┐
│                   GNU Radio Companion                        │
│                                                              │
│  [Bonded RX Source]──vec/fc32──►[Vector to Stream]           │
│                                        │                     │
│                                   ─────┴──────               │
│                                  │            │              │
│                             [FFT Sink]   [File Sink]         │
│                          (Frequency     (raw fc32)           │
│                           display)                           │
└──────────────────────────────────────────────────────────────┘
```

Key GRC block parameters:

- **Bonded RX Source** output vector length: `output_size()` (set in Python `__init__`)
- **Vector to Stream**: item size = `complex`, vec len = same as above
- **FFT Sink**: size = same, `fft_size` should match the block's output size
- **File Sink**: item size = `complex`, unchunked binary

---

## 6. Tuning and band layout

`bonded_rx_streamer` tunes the two devices symmetrically around
`center_freq`. With per-device rate `R` and guard band `G`:

```
Device 0 center  =  center_freq  -  R/2
Device 1 center  =  center_freq  +  R/2

Combined span    =  2 × R  (minus 2 guard edges)
Usable bins      =  fft_size  -  2 × guard_bins
```

The default guard band is 128 bins (≈ 3% of the band per side), which
avoids the DC spur and the sharp filter roll-off at the band edges. You
can verify the actual layout at runtime:

```cpp
uhd::usrp::bonded::spectrum_stitcher st(fft_size);
std::cout << "Usable output bins: " << st.output_bins() << "\n";
std::cout << "Guard band (per side, Hz): " << st.guard_band_hz(sample_rate) << "\n";
```

Rule of thumb: for 40 MHz combined span use `--rate=20e6 --fft-size=4096`.

---

## 7. Quality metrics reference

`bonded_rx_streamer::quality()` returns a `bonding_quality_metrics` struct:

| Field              | Units | Meaning                                                   |
| ------------------ | ----- | --------------------------------------------------------- |
| `drift_ppm`        | ppm   | Frequency offset of board 1 relative to board 0           |
| `uncertainty_ppm`  | ppm   | 1-sigma uncertainty of the estimate                       |
| `pps_samples_used` | count | PPS edges in the regression window (max 120 = 2 min)      |
| `phase_rate_rad_s` | rad/s | Phase rotation rate at the configured RF center frequency |
| `pps_healthy`      | bool  | `false` if no new PPS edge has been seen for > 2 seconds  |
| `valid`            | bool  | `true` when ≥ 3 PPS edges have been collected             |

Typical values after ~30 seconds of operation with B210 hardware:

```
drift      ≈  ±0.5 ppm   (TCXO typical)
uncertainty ≈  0.05 ppm  (30-edge regression)
phase_rate ≈  ±3–6 rad/s at 2.4 GHz
```

Phase correction is applied automatically inside `recv()` once `valid` is
`true`. There is no additional configuration required.

---

## 8. Troubleshooting

### "No PPS detected on board N"

- Check GPS module has acquired ≥ 4 satellites and the PPS LED is blinking.
- Verify the SMA cable is connected to the **PPS** SMA input, not the **REF**
  input.
- Measure the PPS line with a multimeter: it should briefly pulse to ~3.3 V
  once per second.

### "Need at least 2 mboards"

UHD found only one device. Either:

- The second USB cable is not connected.
- One serial number is wrong — recheck with `uhd_find_devices`.
- USB power budget is too low — use a powered hub.

### Drift estimate takes too long to converge

Convergence requires ≥ 3 PPS edges (3 seconds) for the estimate to be valid
and ≥ 30 edges (30 seconds) for uncertainty below 0.1 ppm. If the estimate
never converges:

- Check `pps_healthy` — if `false`, the PPS is dropped before reaching the
  second board.
- Run `bonded_drift_monitor` to observe the raw PPS timing.

### Large time alignment on start-up (`> 1 ms` warning)

`set_time_unknown_pps()` schedules the time-latch on the **next** PPS edge.
If both boards receive the edge within the same USB polling window the
offset is < 1 µs. An offset > 1 ms means one board latched a different
PPS edge — increase the sleep after `set_time_unknown_pps()` to 2 seconds
or add an explicit edge-detection loop.

### Output spectrum has a visible seam in the middle

The guard band hides the DC spur and filter roll-off but the power levels
can still differ at the junction if the gains are not matched. Use
`usrp->set_rx_gain(gain, ch)` on **all** channels, not just channel 0.

### GNU Radio block produces zero-length outputs

`work()` returns `0` when `bonded_rx_streamer::recv()` returns 0 (on
overflow or timeout). GR will call `work()` again immediately. If
overflow is frequent, increase the UHD receive buffer:

```python
usrp.set_recv_frame_size(8192)  # bytes, before creating the streamer
```
