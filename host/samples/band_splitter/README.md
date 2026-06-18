# band_splitter — test/dev GNU Radio block (sibling test repo)

`band_splitter` is a **test-only** GNU Radio block: it splits one wideband
capture into two overlapping radio slices, so the bonded `overlap_reconstructor`
can be exercised offline from a single capture (no second radio). It was moved
**out of the lean `gr-bonded-usrp` product module** and belongs in the sibling
test-tools repo.

## Files

| File | Role |
|---|---|
| `band_splitter.{hpp,cpp}` | C++ DSP engine (namespace `bonded`) |
| `bonded_resample.hpp` | vendored shared resampler header (from the module) |
| `bonded_api.h` | vendored `BONDED_API` macro |
| `overlap_reconstructor.hpp` | vendored — `band_splitter_test.cpp` uses `rational_resample()` |
| `band_splitter_impl.{cc,h}` | GR block wrapper |
| `band_splitter.h` | GR block public header (`gnuradio/bonded_usrp/band_splitter.h`) |
| `band_splitter_python.cc` | pybind11 binding |
| `bonded_usrp_band_splitter.block.yml` | GRC block definition |
| `band_splitter_test.cpp` | Boost.Test unit test |
| `test_band_splitter.py` | Python smoke test |

## Build dependency

`rational_resample()` is **defined in the installed `gnuradio-bonded_usrp`
module** (`overlap_reconstructor.cpp`). When wiring this into the sibling repo,
either link the installed module or compile a copy of `overlap_reconstructor.cpp`
into the test target. `band_splitter.cpp` itself only needs `band_splitter.hpp`
+ `bonded_resample.hpp` (both vendored here).

All symbols are in `namespace bonded` to match the decoupled module.
