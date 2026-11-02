/*
 * bmi270_driver.cpp — BMI270 6-axis IMU driver implementation.
 *
 * Uses the Arduino board library to enable the onboard sensor rail and the
 * Bosch BMI270 API for configuration/FIFO. FIFO reads are polled from the
 * RTOS thread because the onboard INT1 route is not Arduino D2.
 */

#include "bmi270_driver.h"
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <utilities/BMI270-Sensor-API/bmi270.h>
#include <Wire.h>
// Use the Bosch API already vendored and compiled by Arduino_BMI270_BMM150.

using namespace std::chrono_literals;

static struct bmi2_dev s_bmi2_dev;
static uint8_t s_dev_addr = 0x68;
static TwoWire* s_wire = &Wire1;
static char s_role = 'L';
static uint16_t s_fifoWatermarkBytes = 0;
static bool s_haveSensorTime = false;
static uint32_t s_lastSensorTime = 0;
static uint64_t s_sensorTimeEpoch = 0;
static uint32_t s_fifoPollCalls = 0;
static uint32_t s_fifoReadErrors = 0;
static uint32_t s_fifoExtractErrors = 0;
static uint32_t s_fifoExtractWarnings = 0;
static uint32_t s_fifoSamplesReturned = 0;
static uint16_t s_lastFifoLength = 0;
static int8_t s_lastFifoError = BMI2_OK;
static constexpr uint16_t BMI270_MAX_I2C_FIFO_READ = 247; // 19 complete 13-byte frames

// Conversion factors for 16g and 2000dps ranges (raw 16-bit to float)
static constexpr float BMI270_ACCEL_SCALE_16G = 16.0f / 32768.0f;
static constexpr float BMI270_GYRO_SCALE_2000DPS = 2000.0f / 32768.0f;


static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (reg_data == nullptr || len == 0 || len > 250) return BMI2_E_COM_FAIL;
    s_wire->beginTransmission(s_dev_addr);
    s_wire->write(reg_addr);
    if (s_wire->endTransmission(false) != 0) return BMI2_E_COM_FAIL;
    if (s_wire->requestFrom((uint8_t)s_dev_addr, (uint8_t)len) != len)
        return BMI2_E_COM_FAIL;
    for (uint32_t i = 0; i < len; i++) {
        if (!s_wire->available()) return BMI2_E_COM_FAIL;
        reg_data[i] = (uint8_t)s_wire->read();
    }
    return BMI2_OK;
}

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    s_wire->beginTransmission(s_dev_addr);
    s_wire->write(reg_addr);
    for (uint32_t i = 0; i < len; i++) {
        s_wire->write(reg_data[i]);
    }
    if (s_wire->endTransmission() != 0) return BMI2_E_COM_FAIL;
    return BMI2_OK;
}

static void delay_us(uint32_t period, void *intf_ptr) {
    delayMicroseconds(period);
}

bool bmi270_init(char role) {
    s_role = role;
    s_haveSensorTime = false;
    s_lastSensorTime = 0;
    s_sensorTimeEpoch = 0;
    if (!IMU.begin()) {
        Serial.println("[BMI270] ERROR: IMU.begin() failed — check I2C wiring");
        return false;
    }

    // The onboard IMU is on Wire1. Falling back to Wire would probe the
    // external header bus and can mask a board/bus configuration error.
    s_wire = &Wire1;
    s_wire->beginTransmission(s_dev_addr);
    if (s_wire->endTransmission() != 0) {
        s_dev_addr = 0x69;
        s_wire->beginTransmission(s_dev_addr);
        if (s_wire->endTransmission() != 0) {
            Serial.println("[BMI270] ERROR: sensor not found on Wire1 at 0x68/0x69");
            return false;
        }
    }

    s_bmi2_dev.chip_id = BMI270_CHIP_ID;
    s_bmi2_dev.intf = BMI2_I2C_INTF;
    s_bmi2_dev.read = i2c_read;
    s_bmi2_dev.write = i2c_write;
    s_bmi2_dev.delay_us = delay_us;
    s_bmi2_dev.read_write_len = 30; // matches Arduino_BMI270_BMM150's Wire limit
    s_bmi2_dev.intf_ptr = nullptr;
    s_bmi2_dev.dummy_byte = 0;
    s_bmi2_dev.resolution = 16;

    // The Arduino wrapper's IMU.begin() does not initialize this Bosch API
    // device context. The Bosch initializer loads the BMI270 configuration
    // image and initializes the internal API state.
    int8_t rslt = bmi270_init(&s_bmi2_dev);
    if (rslt != BMI2_OK) {
        Serial.print("[BMI270] ERROR: Bosch init failed, code=");
        Serial.println(rslt);
        return false;
    }

    struct bmi2_sens_config config[2];
    config[0].type = BMI2_ACCEL;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_16G;

    config[1].type = BMI2_GYRO;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi270_set_sensor_config(config, 2, &s_bmi2_dev);
    if (rslt != BMI2_OK) {
        Serial.print("[BMI270] ERROR: sensor config failed, code=");
        Serial.println(rslt);
        return false;
    }

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi270_sensor_enable(sens_list, 2, &s_bmi2_dev);
    if (rslt != BMI2_OK) {
        Serial.print("[BMI270] ERROR: sensor enable failed, code=");
        Serial.println(rslt);
        return false;
    }

    // Enable hardware gyro offset compensation
    rslt = bmi2_set_gyro_offset_comp(BMI2_ENABLE, &s_bmi2_dev);
    if (rslt != BMI2_OK) {
        Serial.print("[BMI270] ERROR: gyro offset compensation failed, code=");
        Serial.println(rslt);
        return false;
    }

    Serial.println("[BMI270] OK — Bosch API initialized, 400Hz, +-16g");
    return true;
}

static uint8_t s_fifo_data[1024];

bool bmi270_configure_fifo(uint32_t watermarkSamples) {
    if (watermarkSamples == 0 || watermarkSamples > 19) return false;
    int8_t rslt;
    // Disable advance power save mode before configuring FIFO (required by Bosch API)
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &s_bmi2_dev);
    if (rslt != BMI2_OK) goto config_error;

    // Reset FIFO configuration
    rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &s_bmi2_dev);
    if (rslt != BMI2_OK) goto config_error;

    // Configure FIFO with Accel + Gyro + Header
    rslt = bmi2_set_fifo_config(BMI2_FIFO_GYR_EN | BMI2_FIFO_ACC_EN | BMI2_FIFO_HEADER_EN,
                                BMI2_ENABLE, &s_bmi2_dev);
    if (rslt != BMI2_OK) goto config_error;
    
    // 1 frame = 1 byte header + 6 accel + 6 gyro
    s_fifoWatermarkBytes = (uint16_t)(watermarkSamples * 13);
    rslt = bmi2_set_fifo_wm(s_fifoWatermarkBytes, &s_bmi2_dev);
    if (rslt != BMI2_OK) goto config_error;
    Serial.print("[BMI270] FIFO ready; polling watermark ");
    Serial.print(watermarkSamples);
    Serial.println(" samples");
    return true;

config_error:
    Serial.print("[BMI270] ERROR: FIFO configuration failed, code=");
    Serial.println(rslt);
    return false;
}

uint32_t bmi270_read_fifo_burst(ImuSample* out, uint32_t maxSamples) {
    if (out == nullptr || maxSamples == 0) return 0;
    s_fifoPollCalls++;
    if ((s_fifoPollCalls % 1000) == 0) {
        Serial.print("[BMI270] FIFO diag bytes="); Serial.print(s_lastFifoLength);
        Serial.print(" samples="); Serial.print(s_fifoSamplesReturned);
        Serial.print(" readErrors="); Serial.print(s_fifoReadErrors);
        Serial.print(" extractErrors="); Serial.print(s_fifoExtractErrors);
        Serial.print(" extractWarnings="); Serial.print(s_fifoExtractWarnings);
        Serial.print(" lastStatus="); Serial.println(s_lastFifoError);
    }

    uint16_t fifo_length = 0;
    int8_t rslt = bmi2_get_fifo_length(&fifo_length, &s_bmi2_dev);
    if (rslt != BMI2_OK) {
        s_fifoReadErrors++;
        s_lastFifoError = rslt;
        return 0;
    }
    s_lastFifoLength = fifo_length;
    
    if (fifo_length < s_fifoWatermarkBytes) return 0;
    // Wire transactions are limited to 250 bytes by the Arduino board
    // library. Drain in bounded chunks; the FIFO retains any bytes not read.
    if (fifo_length > BMI270_MAX_I2C_FIFO_READ) fifo_length = BMI270_MAX_I2C_FIFO_READ;
    
    struct bmi2_fifo_frame fifo_frame = {};
    fifo_frame.data = s_fifo_data;
    fifo_frame.length = fifo_length;
    
    rslt = bmi2_read_fifo_data(&fifo_frame, &s_bmi2_dev);
    if (rslt != BMI2_OK) {
        s_fifoReadErrors++;
        s_lastFifoError = rslt;
        return 0;
    }
    
    struct bmi2_sens_axes_data acc_data[50];
    struct bmi2_sens_axes_data gyr_data[50];
    uint16_t acc_count = 50;
    uint16_t gyr_count = 50;
    
    int8_t acc_rslt = bmi2_extract_accel(acc_data, &acc_count, &fifo_frame, &s_bmi2_dev);
    int8_t gyr_rslt = bmi2_extract_gyro(gyr_data, &gyr_count, &fifo_frame, &s_bmi2_dev);
    // The Bosch FIFO parser uses positive return values for warnings such as
    // BMI2_W_PARTIAL_READ (valid frames were extracted, with more remaining).
    // Only negative statuses indicate a failed parse; rejecting warnings here
    // silently discarded every usable batch on this I2C FIFO path.
    if (acc_rslt < BMI2_OK || gyr_rslt < BMI2_OK) {
        s_fifoExtractErrors++;
        s_lastFifoError = (acc_rslt != BMI2_OK) ? acc_rslt : gyr_rslt;
        return 0;
    }
    if (acc_rslt > BMI2_OK || gyr_rslt > BMI2_OK) s_fifoExtractWarnings++;
    
    uint32_t sensor_time = fifo_frame.sensor_time & 0x00FFFFFF;
    if (sensor_time == 0) {
        // A sensor-time FIFO control frame is not guaranteed in each drained
        // watermark. Fall back to the live 24-bit sensor-time register.
        uint8_t time_bytes[3] = {0, 0, 0};
        if (bmi2_get_regs(BMI2_SENSORTIME_ADDR, time_bytes, 3, &s_bmi2_dev) != BMI2_OK)
            { s_fifoReadErrors++; s_lastFifoError = BMI2_E_COM_FAIL; return 0; }
        sensor_time = (uint32_t)time_bytes[0] |
                      ((uint32_t)time_bytes[1] << 8) |
                      ((uint32_t)time_bytes[2] << 16);
    }
    if (s_haveSensorTime && sensor_time < s_lastSensorTime &&
        (s_lastSensorTime - sensor_time) > 0x00800000UL) {
        s_sensorTimeEpoch += 0x01000000ULL;
    }
    s_lastSensorTime = sensor_time;
    s_haveSensorTime = true;
    const uint64_t newestUnwrapped = s_sensorTimeEpoch + sensor_time;

    uint32_t count = (acc_count < gyr_count) ? acc_count : gyr_count;
    if (count > maxSamples) count = maxSamples;
    
    for (uint32_t i = 0; i < count; i++) {
        float ax = acc_data[i].x;
        float ay = acc_data[i].y;
        float az = acc_data[i].z;
        float gx = gyr_data[i].x;
        float gy = gyr_data[i].y;
        float gz = gyr_data[i].z;

        // Apply body-frame rotation matrix (placeholder matrix mapping raw to: X=AP, Y=ML, Z=Vertical)
        // For the left ankle, mirror the lateral axis
        float mapped_ax = ax; 
        float mapped_ay = (s_role == 'L') ? -ay : ay; 
        float mapped_az = az;
        
        float mapped_gx = gx; 
        float mapped_gy = (s_role == 'L') ? -gy : gy; 
        float mapped_gz = gz;

        out[i].accelX = mapped_ax * BMI270_ACCEL_SCALE_16G;
        out[i].accelY = mapped_ay * BMI270_ACCEL_SCALE_16G;
        out[i].accelZ = mapped_az * BMI270_ACCEL_SCALE_16G;
        out[i].gyroX  = mapped_gx * BMI270_GYRO_SCALE_2000DPS;
        out[i].gyroY  = mapped_gy * BMI270_GYRO_SCALE_2000DPS;
        out[i].gyroZ  = mapped_gz * BMI270_GYRO_SCALE_2000DPS;
        
        // Sensor time ticks are exactly 39.0625 µs (ratio 625 / 16).
        // At 400Hz, samples are ~2500us apart, which is ~64 ticks.
        // Mask with 0x00FFFFFF to handle 24-bit counter rollover.
        uint64_t tick_offset = (uint64_t)(count - 1 - i) * 64ULL;
        uint64_t sample_time = (newestUnwrapped >= tick_offset) ?
            (newestUnwrapped - tick_offset) : 0;
        out[i].sensor_time = (uint32_t)(sample_time & 0x00FFFFFFULL);
        out[i].timestamp_us = (uint32_t)((sample_time * 625ULL) >> 4);
    }
    
    s_fifoSamplesReturned += count;
    s_lastFifoError = (acc_rslt > BMI2_OK) ? acc_rslt :
                      ((gyr_rslt > BMI2_OK) ? gyr_rslt : BMI2_OK);
    return count;
}
 