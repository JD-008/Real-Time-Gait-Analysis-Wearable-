/*
 * bmm150_driver.cpp — BMM150 magnetometer driver (Phase 2 implementation).
 *
 * The BMM150 shares the I2C bus with the BMI270 and is included in the
 * Arduino_BMI270_BMM150 library bundle. It provides 3-axis magnetic field
 * data at up to 30 Hz (low-power preset used here).
 *
 * Phase 2 scope for magnetometer:
 *   - Init the mag at a fixed 10 Hz data rate (enough for heading)
 *   - bmm150_read() returns calibrated magnetic field in µT on all 3 axes
 *   - Heading computation (atan2 of X/Y after hard-iron correction) is
 *     stubbed — calibration coefficients depend on device placement and
 *     the magnetic environment, so they can only be populated after a
 *     figure-eight calibration on the actual hardware.
 *   - Yaw-drift correction for stride length is a Phase 3 item (requires
 *     sensor fusion with the gyro orientation quaternion in gait_dsp.cpp).
 *
 * NOTE: The Arduino_BMI270_BMM150 library exposes magnetometer data via
 * IMU.readMagneticField() on the same IMU object used by bmi270_driver.cpp.
 * Both drivers share the same singleton — bmm150_init() therefore does NOT
 * call IMU.begin() again (bmi270_init() already did that).
 */

#include "bmm150_driver.h"
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

static bool s_initialized = false;

// Hard-iron calibration offsets in µT — set after figure-eight calibration.
// Until calibrated, these are zero (raw field values are logged as-is).
// TODO: populate from a calibration procedure and store in EEPROM or flash.
static float s_hardIronX = 0.0f;
static float s_hardIronY = 0.0f;
static float s_hardIronZ = 0.0f;

static float s_latestMx = 0.0f;
static float s_latestMy = 0.0f;
static float s_latestMz = 0.0f;

bool bmm150_init() {
    // IMU.begin() was already called by bmi270_init() and initialises the
    // magnetometer as part of the same device (BMM150 is auxiliary to BMI270
    // in the library's combined IMU object). We just verify the mag is
    // returning data by checking the sample rate.
    float magRate = IMU.magneticFieldSampleRate();
    if (magRate <= 0.0f) {
        Serial.println("[BMM150] WARNING: magneticFieldSampleRate() returned 0 "
                       "— mag may not be available on this hardware revision");
        s_initialized = false;
        return false;
    }

    s_initialized = true;
    Serial.print("[BMM150] OK — mag sample rate: ");
    Serial.print(magRate, 1);
    Serial.println(" Hz (raw field; hard-iron calibration not set)");
    return true;
}

bool bmm150_read(float* mx, float* my, float* mz) {
    if (!s_initialized || mx == nullptr || my == nullptr || mz == nullptr) return false;
    if (!IMU.magneticFieldAvailable()) return false;

    float x, y, z;
    IMU.readMagneticField(x, y, z);
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return false;

    // Apply hard-iron correction (subtract measured magnetic bias)
    *mx = x - s_hardIronX;
    *my = y - s_hardIronY;
    *mz = z - s_hardIronZ;

    s_latestMx = *mx;
    s_latestMy = *my;
    s_latestMz = *mz;
    return true;
}

float bmm150_heading_deg() {
    if (!s_initialized) return 0.0f;
    // Compass heading in degrees [0, 360) using horizontal flux components
    float heading = atan2f(-s_latestMy, s_latestMx) * 180.0f / (float)M_PI;
    if (heading < 0.0f) heading += 360.0f;
    return heading;
}
