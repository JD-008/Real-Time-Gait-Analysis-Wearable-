#pragma once
/*
 * sd_logger.h — Asynchronous SD-card logging for ARC SENSE.
 *
 * DESIGN
 * ------
 * The SD library's File::print() calls are NOT safe to call from the
 * high-priority sensor or DSP threads — they can block for multiple
 * milliseconds on an SPI bus transaction, which would jitter the
 * sensor acquisition timing and degrade the gait event timestamps.
 *
 * Solution: caller threads enqueue entries into lock-free SPSC ring
 * buffers (same ring_buffer.h used elsewhere). A dedicated low-priority
 * LoggerThread drains the queues and writes to SD. This keeps all
 * SD I/O strictly off the sensor and DSP critical paths.
 *
 * THREE FILES per session (session index auto-incremented so files are never
 * silently overwritten):
 *
 *   I_<ROLE>_NNNN.BIN — ARC1 header + 20-byte raw IMU records
 *   E_<ROLE>_NNNN.CSV — gait events (MSw, TO, HS, MS)
 *   S_<ROLE>_NNNN.CSV — per-stride computed parameters
 *
 * where <ROLE> is "L" or "R" depending on DEVICE_ROLE (set in main).
 *
 * THREAD SAFETY
 * -------------
 * sd_logger_log_sample()  — call from DSPThread (same thread that pops
 *                           from imuRingBuffer, so guaranteed single-producer)
 * sd_logger_log_event()   — call from DSPThread on event detection
 * sd_logger_log_stride()  — call from DSPThread on heel-strike resolution
 * sd_logger_poll()        — call from LoggerThread ONLY (~20ms interval)
 * sd_logger_close()       — call from setup() shutdown path or never (power-off)
 *
 * INTEGRATION CHECKLIST
 * ---------------------
 * 1. Wire SD card to SPI: MOSI=11, MISO=12, SCK=13, CS=SD_CS_PIN (define below)
 * 2. Add "SD" to Arduino library dependencies
 * 3. Call sd_logger_init(SD_CS_PIN, role_char, offset) in setup() before
 *    starting worker threads
 * 4. In DSPThread, call sd_logger_log_sample() on every IMU sample
 * 5. In DSPThread, call sd_logger_log_event() whenever a gait event is detected
 * 6. In DSPThread, call sd_logger_log_stride() on every heel-strike resolution
 * 7. LoggerThread loop: call sd_logger_poll() every ~20ms, sleep_for(20ms)
 *
 * EDGE AI PROVISION
 * -----------------
 * The stride CSV includes a reserved `edgeai_label` column (written as -1
 * during data collection). When Edge AI is activated in Phase 3, DSPThread
 * will call sd_logger_log_edgeai() to update this column with the inference
 * result before the stride row is flushed.
 *
 * SD CARD WIRING (Nano 33 BLE / nRF52840)
 * ----------------------------------------
 * Nano 33 BLE uses its primary SPI bus on the standard Arduino SPI pins:
 *   MOSI = D11, MISO = D12, SCK = D13
 * SD CS pin is user-defined — default below assumes D10 (change as needed).
 */
 
#include "gait_dsp.h"   // ImuSample, StrideParameters, GaitEventType
#include <SdFat.h>
 
// ---------------------------------------------------------------------------
// Configuration — tune in main before calling sd_logger_init()
// ---------------------------------------------------------------------------
static constexpr uint8_t SD_CS_PIN = 10;   // SPI CS for SD card module
 
// ---------------------------------------------------------------------------
// Event type mirror — matches GaitEventType but expressed as an SD log enum
// so sd_logger.cpp doesn't need to know GaitEventType internals.
// ---------------------------------------------------------------------------
enum SdLogEventType : uint8_t {
    SD_EVENT_MID_SWING   = 0,
    SD_EVENT_TOE_OFF     = 1,
    SD_EVENT_HEEL_STRIKE = 2,
    SD_EVENT_MID_STANCE  = 3,
};
 
// ---------------------------------------------------------------------------
// Edge AI result (Phase 3 provision — populated as "not yet run" by default)
// ---------------------------------------------------------------------------
struct EdgeAIResult {
    int8_t  label      = -1;    // -1 = no inference yet; 0..N = class index
    float   confidence = 0.0f;  // 0..1
};
 
// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------
 
// Initialise SD card and open session files.
//   chipSelectPin: SPI CS pin connected to SD module
//   roleChar:      'L' for left device, 'R' for right device
//   syncOffsetUs: optional externally measured clock offset; current firmware
//                 does not automatically synchronize the two boards
// Returns true on success. All subsequent calls are no-ops if init fails.
bool sd_logger_init(uint8_t chipSelectPin, char roleChar, int32_t syncOffsetUs = 0);
 
// Enqueue a raw IMU sample for writing. Non-blocking; drops newest on overrun.
// Call from DSPThread on every popped IMU sample.
void sd_logger_log_sample(const ImuSample& sample);
 
// Enqueue a gait event. Call from DSPThread when event detected.
void sd_logger_log_event(SdLogEventType type, uint32_t sampleIndex,
                         uint32_t timestamp_us);
 
// Enqueue a completed stride's parameters. Call from DSPThread at HS.
// edgeAI: pass default EdgeAIResult{} during data-collection; fill in Phase 3.
void sd_logger_log_stride(const StrideParameters& params,
                          uint32_t hsSampleIndex, uint32_t timestamp_us,
                          const EdgeAIResult& edgeAI = EdgeAIResult{});
 
// Drain queues and write to SD. Call from LoggerThread every ~20ms.
// Caps work per call to MAX_DRAIN_PER_POLL entries to keep loop bounded.
void sd_logger_poll();
 
// Snapshot cumulative queue overflows for collection-quality diagnostics.
void sd_logger_get_drop_counts(uint32_t* samples, uint32_t* events,
                               uint32_t* strides);
void sd_logger_get_written_counts(uint32_t* samples, uint32_t* events,
                                  uint32_t* strides);
 
// Flush and close all files. Call before power-off if possible.
void sd_logger_close();
 
// Marker setters (called from BLE command channel)
void sd_logger_set_sync_marker(uint16_t marker);
void sd_logger_set_label_marker(uint16_t label);