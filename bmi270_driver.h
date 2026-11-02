#pragma once
/*
 * BMI270 driver interface for the onboard Nano 33 BLE Rev2 sensor.
 * Bosch API access stays behind this interface so the acquisition thread
 * does not depend on sensor-driver details.
 */

#include "gait_dsp.h"   // for ImuSample
#include <cstdint>

// Initializes the board IMU and Bosch BMI270 API. Returns false on any
// configuration or bus error; role is 'L' or 'R' for axis mapping.
bool bmi270_init(char role);

// Configures FIFO for accel+gyro streaming and the polling watermark.
bool bmi270_configure_fifo(uint32_t watermarkSamples);

// Drains up to maxSamples from the FIFO into `out`, returns actual count
// read. Called only from SensorAcquisitionThread (owns the I2C bus for
// this sensor).
uint32_t bmi270_read_fifo_burst(ImuSample* out, uint32_t maxSamples);
