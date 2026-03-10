# Synchronization Validation How-To

This document demonstrates how to run the synchronization validation examples introduced in Phase 1 of the Bonding Implementation.

## Hardware Setup

To validate bonding synchronization, you need:

- At least two Ettus B210 (or compatible) devices.
- A shared 10 MHz reference signal.
- A shared PPS (Pulse Per Second) signal.
  The reference signals should be distributed to the `10 MHz In` and `PPS In` connectors of all coordinated USRP devices.

## Building the Examples

The synchronization examples are built alongside the regular UHD examples.
Ensure you have created a build directory and run CMake:

```bash
mkdir -p build && cd build
cmake ../
make -j4
```

## Running Sync Validation Capture

This tool captures simultaneously from multiple devices to separate binary files, enabling offline analysis.

**Example execution (External Clock and Time):**

```bash
./examples/sync_validation_capture \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --duration=1.0 \
    --output-dir=./sync_capture \
    --format=sc16
```

This produces `./sync_capture/usrp_0.bin`, `./sync_capture/usrp_1.bin`, and a `metadata.json` file.

## Running Drift Monitor

This tool tracks local oscillator clock drift between two devices over time.

**Example execution (Internal Clock, External Time):**

```bash
./examples/drift_monitor \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=internal \
    --time-source=external \
    --duration=3600 \
    --interval=1.0 \
    --output=drift_log.csv
```

## Running Sync Validation TX

This program transmits an aligned tone simultaneously across multiple specified motherboards.

**Example execution:**

```bash
./examples/sync_validation_tx \
    --args="serial0=30B56D6,serial1=30DBC3C" \
    --clock-source=external \
    --time-source=external \
    --freq=915e6 \
    --rate=10e6 \
    --gain=30 \
    --waveform=tone \
    --duration=1.0
```

## Further Steps

You can use GNU Radio or custom Python scripts to load the capture bins and execute cross-correlation validation to visualize multi-device alignment fidelity natively.
