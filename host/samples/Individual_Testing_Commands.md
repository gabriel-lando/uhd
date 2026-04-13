# Phase 1 — Per-device PPS / External Clock Evaluation

Purpose: verify each USRP B210 is receiving PPS and/or external 10 MHz reference using existing UHD tools (no UHD source changes required).

Devices used in examples:

- Serial: 30B56D6
- Serial: 30DBC3C

## Prerequisites

- Linux host with UHD built; binaries are under `build/examples/`.
- Devices connected via USB to the host.
- If using external 10 MHz/PPS hardware, ensure cabling is connected to the B210 REF IN and PPS IN inputs before running any test.

---

## Step 1 — Enumerate devices

```bash
uhd_find_devices -v
```

Confirm both serials (`30B56D6` and `30DBC3C`) are listed. If a device is missing, check USB connections and rerun.

---

## Step 2 — Probe each device

```bash
uhd_usrp_probe --args="serial=30B56D6"
uhd_usrp_probe --args="serial=30DBC3C"
```

Filter for the relevant fields:

```bash
uhd_usrp_probe --args="serial=30B56D6" | grep -iE "clock|time|pps|ref|lock"
uhd_usrp_probe --args="serial=30DBC3C" | grep -iE "clock|time|pps|ref|lock"
```

What to look for:

- `clock_source` / `clock_sources` — should list `external` if the 10 MHz input is wired.
- `time_source` / `time_sources` — should list `external` or `pps`.
- Any `ref_locked` or `lo_locked` sensor lines.

---

## Step 3 — Test PPS input with `test_pps_input`

`test_pps_input` attempts to detect a rising PPS edge and latch the time. It exits with an error if no PPS is observed within the timeout.

**Binary location:** `build/examples/test_pps_input`

### Test with default time source (internal)

```bash
./build/examples/test_pps_input --args="serial=30B56D6"
./build/examples/test_pps_input --args="serial=30DBC3C"
```

### Test with external PPS source

```bash
./build/examples/test_pps_input --args="serial=30B56D6" --source="external"
./build/examples/test_pps_input --args="serial=30DBC3C" --source="external"
```

### Test with GPSDO PPS source

```bash
./build/examples/test_pps_input --args="serial=30B56D6" --source="gpsdo"
./build/examples/test_pps_input --args="serial=30DBC3C" --source="gpsdo"
```

**Options:**

| Option     | Description                                                   | Default   |
| ---------- | ------------------------------------------------------------- | --------- |
| `--args`   | UHD device address (serial, IP, etc.)                         | _(any)_   |
| `--source` | Time source: `external`, `gpsdo`, or blank for device default | _(blank)_ |

**Pass criterion:** program prints `Success!` and exits with code `0`.  
**Fail criterion:** program throws an exception or times out — PPS signal is absent or miscabled.

---

## Step 4 — Test external 10 MHz reference with `test_dboard_coercion`

`test_dboard_coercion` sets the clock source, waits for the `ref_locked` sensor to assert, and throws immediately if lock is not achieved. This is the fastest way to confirm a working 10 MHz input.

**Binary location:** `build/examples/test_dboard_coercion`

### Quick lock-only check (no full frequency sweep)

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

If the external 10 MHz reference is not locked the program throws and exits immediately after the lock check — no sweep is performed.

### Full RX + TX coercion sweep with external clock

```bash
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref external \
  --rx --tx

./build/examples/test_dboard_coercion \
  --args "serial=30DBC3C" \
  --ref external \
  --rx --tx
```

### Full test with internal clock (no external reference required)

```bash
./build/examples/test_dboard_coercion \
  --args "serial=30B56D6" \
  --ref internal \
  --rx --tx

./build/examples/test_dboard_coercion \
  --args "serial=30DBC3C" \
  --ref internal \
  --rx --tx
```

**Key options:**

| Option         | Description                                              | Default        |
| -------------- | -------------------------------------------------------- | -------------- |
| `--args`       | UHD device address                                       | _(any device)_ |
| `--ref`        | Clock reference: `internal`, `external`, `mimo`, `gpsdo` | _(unchanged)_  |
| `--rx`         | Test RX direction                                        | _(off)_        |
| `--tx`         | Test TX direction                                        | _(off)_        |
| `--no-rx-gain` | Skip RX gain sweep (use for a lock-only check)           | _(off)_        |
| `--freq-step`  | Frequency step between tune points (Hz)                  | `100e6`        |
| `--verbose`    | Print every tune/gain result                             | _(off)_        |

**Pass criterion:** program prints a pass summary and exits with code `0`.  
**Fail criterion:** throws on `ref_locked` check → 10 MHz reference absent or miscabled.

---

## Pass / Fail Checklist

| Check                                  | Device 30B56D6 | Device 30DBC3C |
| -------------------------------------- | -------------- | -------------- |
| Enumerated by `uhd_find_devices`       | ☐              | ☐              |
| `uhd_usrp_probe` completes             | ☐              | ☐              |
| `test_pps_input` — PASS                | ☐              | ☐              |
| `test_dboard_coercion` ref lock — PASS | ☐              | ☐              |

---

## Troubleshooting

- **Device not found:** check USB cable and run `dmesg | tail -20` to inspect enumeration errors.
- **`test_pps_input` fails:** verify PPS cable is connected to the B210 PPS IN SMA; confirm signal is 3.3 V CMOS/TTL with pulse width >100 µs.
- **`test_dboard_coercion` fails on ref lock:** verify 10 MHz cable is connected to the B210 REF IN SMA; confirm signal level is approximately +7 dBm at 10 MHz.
- **Both tests fail on one device:** try a different USB port or USB cable, then re-run `uhd_find_devices`.

---

## Next Steps

- If both devices pass all checks, proceed to **Phase 2**: capture simultaneous IQ snapshots with `samples/capture_sync.py` and quantify time/frequency offset with `samples/analyze_sync.py`.
- If either device fails, resolve the cabling or hardware issue before proceeding.

---

Documented by: Phase 1 checklist for B210 serials 30B56D6 and 30DBC3C
