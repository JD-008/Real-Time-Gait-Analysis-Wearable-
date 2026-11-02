#pragma once
/*
 * bmm150_driver.h — BMM150 magnetometer driver interface.
 *
 * The BMM150 shares the I2C bus with the BMI270 and is accessible via
 * the Arduino_BMI270_BMM150 library's IMU singleton. bmm150_init() must
 * be called AFTER bmi270_init() (which already calls IMU.begin()).
 *
 * Phase 2: basic read implemented. Hard-iron calibration and heading/yaw
 * are stubbed — require a figure-eight calibration session on the real
 * hardware before they can return meaningful values.
 *
 * Phase 3: tilt-compensated heading and yaw-drift correction for the
 * stride-length trajectory reconstruction in gait_dsp.cpp.
 */

// Initialise the magnetometer. Call after bmi270_init(). Returns false if
// the combined IMU library does not report a usable magnetometer sample rate.
bool bmm150_init();

// Read hard-iron-corrected magnetic field in µT on all 3 axes.
// Returns true if a finite new sample was available, false otherwise.
bool bmm150_read(float* mx, float* my, float* mz);

// Compute heading in degrees [0, 360) from the latest mag reading.
// Returns 0.0 until hard-iron calibration coefficients are set in
// bmm150_driver.cpp. TODO: implement tilt compensation (Phase 3).
float bmm150_heading_deg();
