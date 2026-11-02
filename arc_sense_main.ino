/*
 * ============================================================================
 *  ARC SENSE — Phase 2 Firmware
 *  Target: Arduino Nano 33 BLE Rev2 (Nordic nRF52840, Cortex-M4F @ 64 MHz)
 *  Core:   Arduino Mbed OS Nano core (arduino:mbed_nano)
 *
 *  PHASE 2 CHANGES vs. PHASE 1
 *  ----------------------------
 *  1. SD Logger integrated — raw IMU samples use the ARC1 binary format;
 *     gait events and stride parameters use CSV files.
 *  2. BLE DATA-COLLECTION MODE — BLE advertises role and accepts marker
 *     commands; stride notifications are disabled. Edge AI is provisioned.
 *  3. BILATERAL SUPPORT — DEVICE_ROLE compile-time switch ('L' / 'R').
 *     Filename prefix, BLE device name, and asymmetry-index documentation
 *     all reflect the role. Same code runs on both legs.
 *  4. BILATERAL CLOCK SYNC — not implemented. Each board currently records
 *     its own local sensor clock; BLE only advertises to an external central.
 *  5. FREEZE INDEX — CMSIS-DSP FFT/PSD body implemented (stub removed).
 *     Freeze Index now returned in latestGaitParams.freezeIndex.
 *     FoG haptic trigger deferred to Phase 3 (threshold TBD from data).
 *  6. BMM150 magnetometer — basic read implemented; heading stubbed.
 *  7. STRIDE LENGTH — Rampp-style trajectory estimate; any placement-specific
 *     scale requires calibration against a reference walkway.
 *  8. EDGE AI PROVISION — EdgeAIResult struct flows through DSPThread →
 *     sd_logger_log_stride() → SD card (-1 label = no inference yet).
 *     To activate Phase 3 AI: populate edgeAIResult in DSPThread and
 *     flip BLE to BLE_MODE_EDGE_AI.
 *
 *  BILATERAL NOTES
 *  ---------------
 *  The SAME source can tag left or right files with DEVICE_ROLE below.
 *  DEVICE_IS_MASTER is a BLE label only; it does not synchronize two boards.
 *
 *  Axis conventions: if the sensor is mounted in mirror-image positions
 *  on the two ankles, TOE_OFF_CROSSING_DIRECTION and GYRO_SAGITTAL_AXIS
 *  may need to be flipped for the right leg. These constants live in
 *  gait_dsp.cpp §1 — verify on bench recording from BOTH legs before
 *  deploying bilaterally.
 *
 *  ASYMMETRY INDEX
 *  ---------------
 *  Each device computes a consecutive-stride asymmetry proxy (stride N
 *  vs stride N-1 on the SAME leg). True bilateral asymmetry (left vs
 *  right) requires the post-processing dashboard to align the two devices'
 *  stride CSVs by session_us timestamp and apply the Symmetry Index formula.
 *  The firmware-side consecutive-stride number is still useful for detecting
 *  gross left/right imbalance within a single limb over time.
 *
 *  SD CARD WIRING (Nano 33 BLE)
 *  ----------------------------
 *  MOSI = D11, MISO = D12, SCK = D13, CS = D10 (default, change SD_CS_PIN)
 *
 *  THREAD ARCHITECTURE (unchanged from Phase 1)
 *  ---------------------------------------------
 *   T1   SensorAcquisitionThread  Realtime  — polls the 400 Hz IMU FIFO
 *   T2   HapticThread             High      — threshold → DRV2605L
 *   T3   DSPThread                Normal    — event pipeline + SD log enqueue
 *   T4   FreezeIndexThread        Low       — sliding-window FFT/PSD
 *   T5   BLEStreamThread          BelowNorm — BLE poll + marker commands
 *   T6   LoggerThread             Low       — SD queue drain (new in Phase 2)
 * ============================================================================
 */

#include <mbed.h>
#include <rtos.h>
#include <mbed_stats.h>

// ---------------------------------------------------------------------------
// Module headers
// ---------------------------------------------------------------------------
#include "bmi270_driver.h"
#include "bmm150_driver.h"
#include "gait_dsp.h"
#include "haptic_driver.h"
#include "freeze_index.h"
#include "ring_buffer.h"
#include "ble_streaming.h"
#include "sd_logger.h"

// ---------------------------------------------------------------------------
// DEBUG OUTPUT ROUTING
// ---------------------------------------------------------------------------
// USB port on this unit is unavailable. Route ALL Serial.print() calls to
// the hardware UART on pins D1 (TX) and D0 (RX) so a USB-TTL adapter can
// be used for Serial Monitor.
//
// Wiring (connect USB-TTL adapter to the ARC SENSE board):
//   ARC SENSE D1 (TX)  →  USB-TTL RX
//   ARC SENSE D0 (RX)  →  USB-TTL TX   (optional, only if you send commands)
//   ARC SENSE GND      →  USB-TTL GND
//
// Open Serial Monitor at 115200 baud on the port your USB-TTL adapter appears as.
//
// To revert to USB CDC (when USB is repaired): comment out the #define below.
#define Serial Serial1

using namespace rtos;
using namespace mbed;
using namespace std::chrono_literals;

// Thread execution heartbeat counters for concurrency & health monitoring
static volatile uint32_t t1_sensor_ticks = 0;
static volatile uint32_t t2_haptic_ticks = 0;
static volatile uint32_t t3_dsp_ticks    = 0;
static volatile uint32_t t4_freeze_ticks = 0;
static volatile uint32_t t5_ble_ticks    = 0;
static volatile uint32_t t6_logger_ticks = 0;

// ===========================================================================
// *** CONFIGURE THESE TWO CONSTANTS BEFORE FLASHING ***
// ===========================================================================

// Device role: 'L' for left ankle, 'R' for right ankle.
// Affects SD filename prefix (I_L_NNNN.BIN / I_R_NNNN.BIN and CSV names),
// BLE device name ("ArcSense-L" / "ArcSense-R"), and Serial log prefix.
static constexpr char DEVICE_ROLE = 'L';

// BLE metadata flag only. It does not synchronize clocks between devices.
static constexpr bool DEVICE_IS_MASTER = true;

// ===========================================================================
// IMU / DSP configuration constants
// ===========================================================================
static constexpr uint32_t IMU_ACQUISITION_RATE_HZ    = 400;   // Raw hardware rate
static constexpr uint32_t DSP_SAMPLE_RATE_HZ         = 100;   // Decimated rate
static constexpr uint32_t FIFO_WATERMARK_SAMPLES     = 18;    // 234 bytes; stays within Wire's 250-byte transaction limit
static constexpr uint32_t RING_BUFFER_DEPTH          = 1024;  // ~2.5 s @ 400 Hz
static constexpr uint32_t FREEZE_WINDOW_SAMPLES      = 256;
static constexpr uint32_t FREEZE_WINDOW_OVERLAP_PCT  = 50;


// Haptic thresholds — placeholder values, calibrate from bench data (Weeks 6–7)
static constexpr float TIBIAL_SHOCK_THRESHOLD_G      = 4.0f;
static constexpr float ASYMMETRY_INDEX_THRESHOLD     = 0.15f; // 15 %

// BLE operating mode for this phase
static constexpr BleMode BLE_OPERATING_MODE = BLE_MODE_DATA_COLLECTION;
// To switch to full streaming for debug:  BLE_MODE_STREAMING
// To activate Edge AI push (Phase 3):    BLE_MODE_EDGE_AI

// ---------------------------------------------------------------------------
// Inter-thread synchronization
// ---------------------------------------------------------------------------
static EventFlags hapticTriggerFlags;
static constexpr uint32_t FLAG_ABNORMAL_GAIT = (1UL << 0);
static rtos::Mutex serialMutex;

// Ring buffers
static SpscRingBuffer<ImuSample, RING_BUFFER_DEPTH> imuRingBuffer;
static SpscRingBuffer<float,     RING_BUFFER_DEPTH> verticalAccelRingBuffer;

// Latest computed gait params — written by DSPThread, read by others.
// Single-writer per field, atomic-sized, no mutex needed for the read side.
static StrideParameters latestGaitParams;

// Current Edge AI result — written by DSPThread (Phase 3), read by BLE/SD.
// During data collection stays at default {label=-1, confidence=0}.
static EdgeAIResult latestEdgeAI;

// ---------------------------------------------------------------------------
// ===========================================================================
// T1 — Sensor Acquisition Thread
// ===========================================================================
void SensorAcquisitionThread() {
    // INT1 on this board is routed to an internal MCU pad, not D2. Polling
    // the hardware FIFO avoids a permanently blocked acquisition thread.
    if (!bmi270_configure_fifo(FIFO_WATERMARK_SAMPLES)) {
        Serial.println("[SensorAcq] ERROR: FIFO setup failed; acquisition stopped");
        return;
    }

    ImuSample burst[50];
    uint32_t bmmPollCounter = 0;

    for (;;) {
        t1_sensor_ticks++;
        uint32_t count = bmi270_read_fifo_burst(burst, 50);
        if (count == 0) {
            ThisThread::sleep_for(2ms);
            continue;
        }
        for (uint32_t i = 0; i < count; i++) {
            imuRingBuffer.push(burst[i]);
        }

        // Opportunistic BMM150 read (~1 Hz, every 20 FIFO watermark cycles = ~1000ms).
        // Executed exclusively within SensorAcquisitionThread so no other thread
        // contends for the Wire1 I2C bus.
        if (++bmmPollCounter >= 20) {
            bmmPollCounter = 0;
            float mx, my, mz;
            if (bmm150_read(&mx, &my, &mz)) {
                // TODO Phase 3: pass (mx, my, mz) into gait_dsp for heading-based
                // de-drift correction and yaw-invariant stride length computation.
                (void)mx; (void)my; (void)mz;
            }
        }

        // Cooperative yield: ensures SensorAcquisitionThread never starves lower-priority
        // threads or the Mbed OS USB CDC communication stack.
        ThisThread::sleep_for(1ms);
    }
}

// ===========================================================================
// T2 — Haptic Thread
// ===========================================================================
void HapticThread() {
    if (!drv2605_init()) {
        return; // Haptics hardware not detected on Wire — thread exits cleanly
    }

    for (;;) {
        hapticTriggerFlags.wait_any(FLAG_ABNORMAL_GAIT);
        t2_haptic_ticks++;
        drv2605_fire_pattern(HAPTIC_PATTERN_ABNORMAL_GAIT);
    }
}

// ===========================================================================
// T3 — DSP Thread
// Primary pipeline: event detection → temporal params → stride length →
//   tibial shock → asymmetry → SD log enqueue → haptic trigger.
// Also forwards vertical accel to Freeze Index ring buffer.
// ===========================================================================
void DSPThread() {
    gait_dsp_init(DSP_SAMPLE_RATE_HZ);

    ImuSample sample;
    ImuSample accum;
    uint32_t sampleCount = 0;
    uint32_t decimateCounter = 0;

    for (;;) {
        if (!imuRingBuffer.pop(sample)) {
            ThisThread::sleep_for(1ms);
            continue;
        }

        t3_dsp_ticks++;
        sampleCount++;

        // ---- SD: log every raw IMU sample (400 Hz) ----
        sd_logger_log_sample(sample);

        // ---- Debug serial: print raw values periodically ----
        if (sampleCount % 2000 == 0) { // 2000 samples = 5s at 400Hz
            Serial.print("[IMU-"); Serial.print(DEVICE_ROLE); Serial.print("]");
            Serial.print(" gX="); Serial.print(sample.gyroX, 1);
            Serial.print(" gY="); Serial.print(sample.gyroY, 1);
            Serial.print(" gZ="); Serial.print(sample.gyroZ, 1);
            Serial.print(" | aX="); Serial.print(sample.accelX, 3);
            Serial.print(" aY="); Serial.print(sample.accelY, 3);
            Serial.print(" aZ="); Serial.println(sample.accelZ, 3);
        }
        
        // Decimate to 100 Hz for gait DSP and Freeze Index
        if (decimateCounter == 0) {
            accum = sample;
        } else {
            accum.accelX += sample.accelX;
            accum.accelY += sample.accelY;
            accum.accelZ += sample.accelZ;
            accum.gyroX  += sample.gyroX;
            accum.gyroY  += sample.gyroY;
            accum.gyroZ  += sample.gyroZ;
        }
        decimateCounter++;
        
        if (decimateCounter == 4) {
            decimateCounter = 0;
            accum.accelX *= 0.25f;
            accum.accelY *= 0.25f;
            accum.accelZ *= 0.25f;
            accum.gyroX  *= 0.25f;
            accum.gyroY  *= 0.25f;
            accum.gyroZ  *= 0.25f;
            accum.timestamp_us = sample.timestamp_us;

            if ((sampleCount / 4) % 1000 == 0) {
                Serial.print("[DSP] Rate: "); Serial.print(gait_dsp_measured_rate_hz(), 1); Serial.print(" Hz, Rejects: ");
                Serial.println(gait_dsp_rejected_sample_count());
            }

            // ---- Gait event detection (100 Hz) ----
            GaitEvent event = gait_dsp_process_sample(accum);

            // ---- Log gait events to SD ----
            if (event.type == EVENT_MID_SWING) {
                sd_logger_log_event(SD_EVENT_MID_SWING, event.sampleIndex, event.timestamp_us);
                serialMutex.lock();
                Serial.print("[EVENT-"); Serial.print(DEVICE_ROLE); Serial.print("] MID_SWING idx=");
                Serial.println(event.sampleIndex);
                serialMutex.unlock();
            } else if (event.type == EVENT_TOE_OFF) {
                sd_logger_log_event(SD_EVENT_TOE_OFF, event.sampleIndex, event.timestamp_us);
            } else if (event.type == EVENT_MID_STANCE) {
                sd_logger_log_event(SD_EVENT_MID_STANCE, event.sampleIndex, event.timestamp_us);
            }

            // ---- Heel strike: compute features, log stride ----
            if (event.type == EVENT_HEEL_STRIKE) {
                sd_logger_log_event(SD_EVENT_HEEL_STRIKE, event.sampleIndex, event.timestamp_us);
                if (gait_dsp_take_new_stride()) {
                    // Tibial shock
                    float shock_g = gait_dsp_get_latest_stride().tibialShockG;
                    latestGaitParams.tibialShockG = shock_g;

                    // Copy temporal + spatial parameters from DSP core
                    const StrideParameters& stride = gait_dsp_get_latest_stride();
                    latestGaitParams.strideTimeS   = stride.strideTimeS;
                    latestGaitParams.swingTimeS    = stride.swingTimeS;
                    latestGaitParams.stanceTimeS   = stride.stanceTimeS;
                    latestGaitParams.strideLengthM = stride.strideLengthM;
                    latestGaitParams.strideCount++; // increment stride counter

                    // Haptic: tibial shock threshold with cooldown (prevents firing on consecutive steps)
                    static uint32_t lastHapticStride = 0;
                    bool hapticTriggered = false;
                    if (shock_g >= TIBIAL_SHOCK_THRESHOLD_G && (latestGaitParams.strideCount - lastHapticStride >= 3)) {
                        lastHapticStride = latestGaitParams.strideCount;
                        hapticTriggerFlags.set(FLAG_ABNORMAL_GAIT);
                        hapticTriggered = true;
                    }

                    // Thread-safe Serial output
                    serialMutex.lock();
                    Serial.print("[EVENT-"); Serial.print(DEVICE_ROLE); Serial.print("] HEEL_STRIKE idx=");
                    Serial.println(event.sampleIndex);
                    Serial.print("  shock=");  Serial.print(shock_g, 2);         Serial.print("g");
                    Serial.print("  stride="); Serial.print(stride.strideTimeS, 3); Serial.print("s");
                    Serial.print("  swing=");  Serial.print(stride.swingTimeS, 3);  Serial.print("s");
                    Serial.print("  stance="); Serial.print(stride.stanceTimeS, 3); Serial.print("s");
                    Serial.print("  len=");
                    if (stride.strideLengthM > 0.001f) {
                        Serial.print(stride.strideLengthM, 3);
                        Serial.print("m (corrected)");
                    } else {
                        Serial.print("(pending — needs 2+ mid-stance events)");
                    }
                    Serial.println();

                    if (hapticTriggered) {
                        Serial.println("  >> HAPTIC TRIGGER: tibial shock (cooldown active)");
                    }
                    serialMutex.unlock();

                    // Consecutive-stride asymmetry proxy
                    if (stride.strideTimeS > 0.0f) {
                        static float prevStrideTimeS = 0.0f;
                        if (prevStrideTimeS > 0.0f) {
                            float asymmetry = gait_dsp_compute_asymmetry_index(
                                stride.strideTimeS, prevStrideTimeS);
                            latestGaitParams.asymmetryIndex = asymmetry;

                            if (asymmetry >= ASYMMETRY_INDEX_THRESHOLD) {
                                hapticTriggerFlags.set(FLAG_ABNORMAL_GAIT);
                                serialMutex.lock();
                                Serial.print("  >> ASYMMETRY ALERT (consecutive-stride): ");
                                Serial.print(asymmetry * 100.0f, 1);
                                Serial.println("%");
                                serialMutex.unlock();
                            }
                        }
                        prevStrideTimeS = stride.strideTimeS;
                    }

                    // ---- SD: log stride parameters ----
                    sd_logger_log_stride(latestGaitParams, event.sampleIndex,
                                         event.timestamp_us, latestEdgeAI);
                }
            }

            // Feed vertical-axis accel (Y on our ankle mount) to Freeze Index pipeline (100 Hz)
            verticalAccelRingBuffer.push(accum.accelY);
        } // End of decimated 100Hz block
    }
}

// ===========================================================================
// T4 — Freeze Index Thread
// ===========================================================================
void FreezeIndexThread() {
    freeze_index_init(FREEZE_WINDOW_SAMPLES, DSP_SAMPLE_RATE_HZ);

    static float window[FREEZE_WINDOW_SAMPLES];
    uint32_t hopSize = FREEZE_WINDOW_SAMPLES * (100 - FREEZE_WINDOW_OVERLAP_PCT) / 100;

    for (;;) {
        t4_freeze_ticks++;
        if (verticalAccelRingBuffer.available() >= FREEZE_WINDOW_SAMPLES) {
            verticalAccelRingBuffer.peek_window(window, FREEZE_WINDOW_SAMPLES);

            float fi = freeze_index_compute(window, FREEZE_WINDOW_SAMPLES);
            latestGaitParams.freezeIndex = fi;

            // TODO Phase 3: wire FoG threshold to haptic flag.
            // if (freeze_index_is_fog(fi)) {
            //     hapticTriggerFlags.set(FLAG_ABNORMAL_GAIT);
            // }

            verticalAccelRingBuffer.advance(hopSize);
            ThisThread::sleep_for(5ms);
        } else {
            ThisThread::sleep_for(20ms);
        }
    }
}

// ===========================================================================
// T5 — BLE Stream Thread
// Data-collection mode: BLE services and marker commands remain available;
// gait parameters are written to SD rather than notified for every stride.
// Stride parameters are not pushed as notifications.
// To enable streaming for debug: change BLE_OPERATING_MODE above.
// ===========================================================================
void BLEStreamThread() {
    uint32_t lastPushedStrideCount = 0;

    for (;;) {
        t5_ble_ticks++;
        ble_streaming_poll();

        // Streaming mode: push per-stride updates
        if (BLE_OPERATING_MODE == BLE_MODE_STREAMING &&
            latestGaitParams.strideCount != lastPushedStrideCount) {
            ble_streaming_update(latestGaitParams);
            lastPushedStrideCount = latestGaitParams.strideCount;
        }

        // Edge AI mode: push inference results
        if (BLE_OPERATING_MODE == BLE_MODE_EDGE_AI) {
            ble_streaming_update_edgeai(latestEdgeAI);
        }
        
        static uint32_t lastDrops = 0;
        uint32_t currentDrops = imuRingBuffer.get_drops();
        if (currentDrops != lastDrops) {
            Serial.print("[WARNING] IMU RingBuffer Drops: ");
            Serial.println(currentDrops);
            lastDrops = currentDrops;
        }

        ThisThread::sleep_for(50ms);
    }
}

// ===========================================================================
// T6 — Logger Thread (NEW in Phase 2)
// Drains the SD logger queues every 20 ms. Runs at the same priority as
// FreezeIndexThread (osPriorityLow) — both are background tasks that must
// never delay haptic or core DSP.
//
// SD initialization is completed in setup() before workers start. This
// thread is the only runtime SD writer.
// ===========================================================================
void LoggerThread() {
    uint32_t lastReportMs = millis();
    uint32_t prevSampleDrops = 0, prevEventDrops = 0, prevStrideDrops = 0;
    for (;;) {
        t6_logger_ticks++;
        sd_logger_poll();
        if ((uint32_t)(millis() - lastReportMs) >= 5000) {
            uint32_t sampleDrops, eventDrops, strideDrops;
            uint32_t sampleRows, eventRows, strideRows;
            sd_logger_get_drop_counts(&sampleDrops, &eventDrops, &strideDrops);
            sd_logger_get_written_counts(&sampleRows, &eventRows, &strideRows);
            if (sampleRows > 0 || eventRows > 0 || strideRows > 0) {
                Serial.print("[SDLogger] written samples/events/strides=");
                Serial.print(sampleRows); Serial.print('/');
                Serial.print(eventRows); Serial.print('/');
                Serial.println(strideRows);
            }
            if (sampleDrops != prevSampleDrops || eventDrops != prevEventDrops ||
                strideDrops != prevStrideDrops) {
                Serial.print("[SDLogger] queue drops samples/events/strides=");
                Serial.print(sampleDrops); Serial.print('/');
                Serial.print(eventDrops); Serial.print('/');
                Serial.println(strideDrops);
                prevSampleDrops = sampleDrops;
                prevEventDrops = eventDrops;
                prevStrideDrops = strideDrops;
            }
            lastReportMs = millis();
        }
        ThisThread::sleep_for(20ms);
    }
}

// ===========================================================================
// Thread objects — declared globally so they persist after setup() returns
// Priorities calibrated for zero starvation and deterministic scheduling
// ===========================================================================
Thread sensorThread(osPriorityHigh,        3072, nullptr, "SensorAcq");
Thread hapticThread(osPriorityHigh,        2048, nullptr, "Haptic");
Thread dspThread   (osPriorityNormal,      4096, nullptr, "GaitDSP");
Thread freezeThread(osPriorityLow,         4096, nullptr, "FreezeIdx");
Thread bleThread   (osPriorityBelowNormal, 3072, nullptr, "BLE");
Thread loggerThread(osPriorityLow,         4096, nullptr, "Logger");

// ===========================================================================
// setup()
// ===========================================================================
void setup() {
    // NOTE: Serial is remapped to Serial1 (hardware UART on D1/D0).
    // Serial1 is always ready; no USB enumeration wait needed.
    Serial.begin(115200);
    delay(10); // Brief settle for UART TX line

    Serial.println("\n============================================================");
    Serial.print(" ARC SENSE — Full Integrated Production Firmware (Role: ");
    Serial.print(DEVICE_ROLE);
    Serial.print(", Master: ");
    Serial.print(DEVICE_IS_MASTER ? "YES" : "NO");
    Serial.println(")");
    Serial.println(" Board: Arduino Nano 33 BLE Rev2 (nRF52840 Cortex-M4F)");
    Serial.println("============================================================");

    // 1. Initialise BMI270 (Internal Wire1, 400 Hz hardware FIFO)
    Serial.print("Initializing Bosch BMI270 API at 400 Hz... ");
    if (!bmi270_init(DEVICE_ROLE)) {
        delay(250);
        if (!bmi270_init(DEVICE_ROLE)) {
            Serial.println("\n[ FAIL ] BMI270 initialization failed; acquisition stopped.");
            return;
        }
    }
    Serial.println("[ PASS ]");

    // 2. Initialise BMM150 (Internal Wire1, 10 Hz)
    Serial.print("Initializing Bosch BMM150 Magnetometer... ");
    if (!bmm150_init()) {
        Serial.println("[ WARN ] BMM150 unavailable; continuing without magnetometer.");
    } else {
        Serial.println("[ PASS ]");
    }

    // 3. Mount MicroSD card (SPI CS=D10)
    Serial.print("Initializing MicroSD Storage on CS Pin D10... ");
    if (!sd_logger_init(SD_CS_PIN, DEVICE_ROLE, 0)) {
        Serial.println("[ WARN ] SD logger unavailable; proceeding with live streaming & DSP.");
    } else {
        Serial.println("[ PASS ] Card mounted successfully.");
    }

    // 4. Initialise BLE GATT services & start advertising
    // NOTE: On Arduino Nano 33 BLE (Mbed OS), BLE.begin() MUST be called in setup()
    // on the main thread before custom RTOS threads start. Calling it inside a child
    // thread causes the Cordio stack event-loop init to time out.
    Serial.print("Initializing BLE 5.0 Wireless stack... ");
    bool bleOk = ble_streaming_init(BLE_OPERATING_MODE, DEVICE_ROLE, DEVICE_IS_MASTER);
    if (!bleOk) {
        Serial.println("[ WARN ] BLE initialization failed; proceeding without BLE.");
    } else {
        Serial.println("[ PASS ] BLE initialized and advertising.");
    }

    // 5. Start RTOS worker threads in reverse priority order (T6 to T1)
    // This guarantees downstream queues and background workers are active before sensor pumping.
    Serial.println("Starting RTOS worker threads (T6 to T1)...");
    osStatus s6 = loggerThread.start(LoggerThread);
    osStatus s5 = bleOk ? bleThread.start(BLEStreamThread) : osOK;
    osStatus s4 = freezeThread.start(FreezeIndexThread);
    osStatus s3 = dspThread.start(DSPThread);
    osStatus s2 = hapticThread.start(HapticThread);
    osStatus s1 = sensorThread.start(SensorAcquisitionThread);

    if (s1 != osOK || s2 != osOK || s3 != osOK || s4 != osOK || s5 != osOK || s6 != osOK) {
        Serial.println("[ FAIL ] One or more RTOS worker threads failed to start!");
        Serial.print("  Statuses: T6="); Serial.print((int)s6);
        Serial.print(" T5="); Serial.print((int)s5);
        Serial.print(" T4="); Serial.print((int)s4);
        Serial.print(" T3="); Serial.print((int)s3);
        Serial.print(" T2="); Serial.print((int)s2);
        Serial.print(" T1="); Serial.println((int)s1);
    } else {
        Serial.println("[ PASS ] All 6 RTOS threads active and running cooperatively!\n");
    }
}

// ===========================================================================
// loop() — Periodically audits system health & thread heartbeats
// ===========================================================================
void loop() {
    static uint32_t lastHealthReportMs = 0;
    if ((uint32_t)(millis() - lastHealthReportMs) >= 5000) {
        lastHealthReportMs = millis();
        mbed_stats_heap_t heapStats;
        mbed_stats_heap_get(&heapStats);

        serialMutex.lock();
        Serial.println("\n------------------------------------------------------------");
        Serial.print(" ARC SENSE [Role: "); Serial.print(DEVICE_ROLE);
        Serial.println("] — RTOS HEALTH & CONCURRENCY AUDIT");
        Serial.println("------------------------------------------------------------");
        Serial.print("  Heap Used:    "); Serial.print(heapStats.current_size);
        Serial.print(" B | Max Peak: "); Serial.print(heapStats.max_size);
        Serial.print(" B | Avail: "); Serial.print(heapStats.reserved_size);
        Serial.println(" B");

        Serial.println("  Thread Heartbeat Counters (Active Cycles):");
        Serial.print("    T1 Sensor (400Hz High):     "); Serial.println(t1_sensor_ticks);
        Serial.print("    T2 Haptic (Event Driven):   "); Serial.println(t2_haptic_ticks);
        Serial.print("    T3 DSP    (100Hz Normal):   "); Serial.println(t3_dsp_ticks);
        Serial.print("    T4 Freeze (50Hz Low):       "); Serial.println(t4_freeze_ticks);
        Serial.print("    T5 BLE    (20Hz BelowNorm): "); Serial.println(t5_ble_ticks);
        Serial.print("    T6 Logger (50Hz Low):       "); Serial.println(t6_logger_ticks);

        Serial.print("  IMU Ring Buffer Drops: ");
        Serial.println(imuRingBuffer.get_drops());
        Serial.println("------------------------------------------------------------\n");
        serialMutex.unlock();
    }
    ThisThread::sleep_for(1000ms);
}
