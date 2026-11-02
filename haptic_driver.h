#pragma once
/*
 * SmartElex DRV2605L haptic motor driver interface — I2C. Wraps the
 * Adafruit_DRV2605 library (or direct register access if lower latency
 * is needed once measured — flagged as a possible optimization if the
 * library's call overhead shows up in the latency budget).
 */

enum HapticPattern {
    HAPTIC_PATTERN_ABNORMAL_GAIT = 0,   // TODO: choose/calibrate DRV2605L
                                        // effect ID(s) for the demo
};

bool drv2605_init();
bool drv2605_is_initialized();

// Fires the given haptic pattern. Called only from HapticThread — keep
// this call path as short as possible (no logging, no extra I2C reads).
void drv2605_fire_pattern(HapticPattern pattern);
