/*
 * haptic_driver.cpp — DRV2605L haptic motor driver implementation.
 *
 * Uses the Adafruit DRV2605 library over I2C (address 0x5A).
 * The DRV2605L on the SmartElex board drives an ERM or LRA motor —
 * select the correct library below (1 = ERM, 6 = LRA).
 *
 * Called only from HapticThread (osPriorityHigh), so no mutex needed.
 */

#include "haptic_driver.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>

static Adafruit_DRV2605 drv;
static bool s_initialized = false;
static constexpr bool HAPTIC_MOTOR_IS_LRA = false; // Set true for a linear resonant actuator.

// DRV2605L effect IDs (from the datasheet / Adafruit library):
//   1  = Strong Click - 100%
//   6  = Strong Buzz - 100%
//  14  = Strong Buzz - 60%
//  47  = Buzz 1 - 100%
//  52  = Pulsing Strong 1 - 100%
// Pick one that gives clear tactile feedback through the ankle mount.
static constexpr uint8_t EFFECT_ABNORMAL_GAIT = 52;  // Pulsing Strong

bool drv2605_init() {
    s_initialized = false;
    // The external DRV2605L is connected to the board's header I2C bus.
    if (!drv.begin(&Wire)) {
        Serial.println("[DRV2605L] WARNING: not detected on Wire (address 0x5A); haptics disabled");
        return false;
    }

    if (HAPTIC_MOTOR_IS_LRA) {
        drv.useLRA();
        drv.selectLibrary(6);
    } else {
        drv.useERM();
        drv.selectLibrary(1);
    }
    drv.setMode(DRV2605_MODE_INTTRIG);       // Internal trigger mode:
                                             // load waveform, then go()

    s_initialized = true;
    Serial.println("[DRV2605L] OK — initialized, internal trigger mode");
    return true;
}

void drv2605_fire_pattern(HapticPattern pattern) {
    if (!s_initialized) return;

    if (pattern != HAPTIC_PATTERN_ABNORMAL_GAIT) return;

    drv.setWaveform(0, EFFECT_ABNORMAL_GAIT);  // slot 0: the effect
    drv.setWaveform(1, 0);                     // slot 1: end marker
    drv.go();                                  // fire!
}

bool drv2605_is_initialized() {
    return s_initialized;
}
