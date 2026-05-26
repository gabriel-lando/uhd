# test_dboard_coercion Usage Guide

Tests USRP daughterboard frequency/gain coercion and optionally verifies external clock reference lock.

## Binary Location

```
build/examples/test_dboard_coercion
```

## Options

| Option         | Description                                              | Default        |
| -------------- | -------------------------------------------------------- | -------------- |
| `--args`       | UHD device address (IP, serial, etc.)                    | _(any device)_ |
| `--chan`       | Channel index                                            | `0`            |
| `--tx`         | Test TX frequency and gain coercion                      | _(off)_        |
| `--rx`         | Test RX frequency and gain coercion                      | _(off)_        |
| `--ref`        | Clock reference: `internal`, `external`, `mimo`, `gpsdo` | _(unchanged)_  |
| `--freq-step`  | Frequency step between tune points (Hz)                  | `100e6`        |
| `--gain-step`  | Gain step between gain points (dB)                       | `1.0`          |
| `--no-tx-gain` | Skip TX gain coercion                                    | _(off)_        |
| `--no-rx-gain` | Skip RX gain coercion                                    | _(off)_        |
| `--verbose`    | Print every tune/gain check (not just summary)           | _(off)_        |

> **Note:** At least one of `--tx` or `--rx` must be specified.

---

## Usage Examples

### Test external clock reference lock only (no full sweep)

```bash
# Device 30B56D6
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref external \
  --rx --no-rx-gain \
  --freq-step 1e9

# Device 30DBC3C
./build/examples/test_dboard_coercion \
  --args "serial=30DBC3C" \
  --ref external \
  --rx --no-rx-gain \
  --freq-step 1e9
```

If the external 10 MHz reference is not locked, the program throws and exits immediately after the lock check.

---

### Full RX + TX coercion test with external clock

```bash
# Device 30B56D6
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref external \
  --rx --tx

# Device 30DBC3C
./build/examples/test_dboard_coercion \
  --args "serial=30DBC3C" \
  --ref external \
  --rx --tx
```

---

### Full test with internal clock (no external reference required)

```bash
# Device 30B56D6
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref internal \
  --rx --tx

# Device 30DBC3C
./build/examples/test_dboard_coercion \
  --args "serial=30DBC3C" \
  --ref internal \
  --rx --tx
```

---

### Verbose output for debugging

```bash
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref external \
  --rx --tx \
  --verbose
```

---

## What the Program Does

1. Opens the USRP device matching `--args`
2. Sets TX and RX sample rate to 1 MS/s
3. If `--ref` is provided, calls `set_clock_source()` and waits 1 second
4. Checks `ref_locked` sensor (for `external`) or `mimo_locked` (for `mimo`) — **throws on failure**
5. Sweeps the daughterboard frequency range in steps of `--freq-step`, verifying the tuned frequency is within 0.01% of the requested frequency
6. At each frequency, checks `lo_locked` sensor if available
7. For each frequency, sweeps gain in steps of `--gain-step` and verifies accuracy within 0.01%
8. Prints a pass/fail summary per direction (TX/RX)

## Exit Codes

| Code     | Meaning                                                                                    |
| -------- | ------------------------------------------------------------------------------------------ |
| `0`      | Success                                                                                    |
| Non-zero | Device not found, ref not locked, unsupported daughterboard, or missing `--tx`/`--rx` flag |
