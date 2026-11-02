# Real-Time Gait Analysis Wearable

A real-time wearable gait-analysis platform built around the Arduino Nano 33 BLE Rev2. The firmware reads motion data from the Bosch BMI270 IMU, optionally uses the BMM150 magnetometer, performs gait event detection and stride analysis in real time, logs raw and processed data to SD, and can stream BLE data or edge-AI results.

## Overview

This project implements a Phase 2 production firmware for a bilateral ankle-wearable system designed to monitor gait and detect abnormal patterns such as tibial shock, asymmetry, and freeze-index-based gait instability.

The codebase includes:

- Sensor acquisition from BMI270
- Gait DSP and event detection
- Stride parameter estimation
- SD card logging for raw IMU, event, and stride data
- BLE data-collection/streaming modes
- Haptic feedback triggers
- Freeze index computation for gait stability analysis

## Hardware

- Board: Arduino Nano 33 BLE Rev2 (nRF52840, Cortex-M4F)
- IMU: Bosch BMI270
- Magnetometer: Bosch BMM150
- Actuator: DRV2605L haptic driver
- Storage: MicroSD card (SPI)
- Wireless: Bluetooth LE 5.0

## System Architecture

The firmware runs as an RTOS-based multi-thread system:

- T1 — Sensor acquisition thread
- T2 — Haptic thread
- T3 — DSP thread
- T4 — Freeze-index thread
- T5 — BLE thread
- T6 — Logger thread

This separation keeps the time-critical sensor pipeline responsive while offloading logging and background analysis.

## Key Features

### Gait Analysis

- 400 Hz IMU sampling
- 100 Hz gait DSP pipeline
- Event detection for:
  - heel strike
  - toe off
  - mid stance
  - mid swing
- Stride parameter estimation:
  - stride time
  - swing time
  - stance time
  - stride length
  - tibial shock
  - asymmetry index
  - freeze index

### Data Logging

The firmware records:

- raw IMU binary samples to SD
- gait event markers
- stride CSV summaries
- optional edge-AI inference labels

### BLE Modes

The project supports multiple BLE behaviors:

- data collection mode
- streaming mode
- edge-AI mode

The device role is configurable for left/right use (`'L'` or `'R'`).

### Calibration Notes

The code includes several calibration-sensitive parameters that should be tuned against real gait data:

- tibial shock threshold
- asymmetry threshold
- stride-length scaling
- haptic trigger thresholds
- direction conventions for left/right mounting

## Repository Files

- `arc_sense_main.ino` — main firmware entry point and RTOS orchestration
- `gait_dsp.cpp` / `gait_dsp.h` — gait event detection and stride calculations
- `freeze_index.cpp` / `freeze_index.h` — freeze-index analysis
- `bmi270_driver.cpp` / `bmi270_driver.h` — BMI270 IMU driver
- `bmm150_driver.cpp` / `bmm150_driver.h` — BMM150 magnetometer driver
- `haptic_driver.cpp` / `haptic_driver.h` — DRV2605L haptic output
- `ble_streaming.cpp` / `ble_streaming.h` — BLE service and communication logic
- `sd_logger.cpp` / `sd_logger.h` — SD logging and write queue management
- `ring_buffer.h` — lock-free ring buffer implementation

## Build and Use

### Arduino Setup

1. Install the Arduino Mbed OS Nano core.
2. Open the project in Arduino IDE or VS Code with Arduino support.
3. Select the board:
   - `Arduino Nano 33 BLE`
4. Compile and flash `arc_sense_main.ino`.

### Board Configuration

In `arc_sense_main.ino`, configure:

```cpp
static constexpr char DEVICE_ROLE = 'L';
static constexpr bool DEVICE_IS_MASTER = true;
static constexpr BleMode BLE_OPERATING_MODE = BLE_MODE_DATA_COLLECTION;
```

These values control:

- device naming
- SD file naming
- BLE advertising identity
- operating mode for live streaming or data collection

### Wiring Notes

The firmware documents the SD card wiring and UART debug output:

- SD CS pin: D10
- MOSI: D11
- MISO: D12
- SCK: D13
- Debug serial: D1/D0 via `Serial1`

## Intended Use

This system is intended as a wearable gait-monitoring platform for research, prototyping, and real-time movement analysis. It is particularly suited for ankle-mounted monitoring in clinical or experimental settings where live gait event detection and data logging are required.

## Notes

This repo is structured as a research firmware project and reflects an iterative development approach. Some features are intentionally placeholders or require calibration and validation on the bench before deployment in practice.

## License

This project does not currently declare an explicit license file. If you plan to redistribute or extend the design, confirm the intended licensing terms before publication or commercial use.

