#pragma once
/*
 * ble_streaming.h — BLE GATT service for ARC SENSE.
 *
 * PHASE 2 OPERATING MODES
 * -----------------------
 * BLE_MODE_DATA_COLLECTION (default — current phase):
 *   Device advertises as "ArcSense-L" or "ArcSense-R" and accepts a
 *   central connection. Device-info, local timestamp, and marker
 *   characteristics are exposed. All stride data goes to SD card. The
 *   timestamp is not exchanged between boards. This is "relax mode" — the BLE stack is NOT
 *   loaded with per-stride notifications.
 *
 * BLE_MODE_EDGE_AI (Phase 3, provision only — not yet activated):
 *   After on-device inference, push EdgeAI results over BLE to a
 *   central dashboard. Stride data still goes to SD; BLE only carries
 *   inference labels + confidence, not raw parameters.
 *
 * BLE_MODE_STREAMING (legacy / debug):
 *   Full stride parameter notifications at ~1 Hz. Use only for bench
 *   validation — adds BLE overhead to the main loop.
 *
 * SYNC MECHANISM
 * --------------
 * The selected master exposes its local session-start micros() value as a
 * GATT characteristic. This firmware does not connect the two boards or
 * synchronize their clocks; each device currently records local timestamps.
 *
 * Thread safety: ble_streaming_init() must be called from setup() on the main
 * thread before starting RTOS threads (required by Mbed OS Cordio stack).
 * ble_streaming_poll() and characteristic updates are called from BLEStreamThread.
 */

#include "gait_dsp.h"    // StrideParameters
#include "sd_logger.h"   // EdgeAIResult

// ---------------------------------------------------------------------------
// BLE operating mode
// ---------------------------------------------------------------------------
enum BleMode : uint8_t {
    BLE_MODE_DATA_COLLECTION = 0,   // relax — SD logging, minimal BLE
    BLE_MODE_EDGE_AI         = 1,   // push AI inference results over BLE
    BLE_MODE_STREAMING       = 2,   // full stride parameter streaming (debug)
};

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Initialise ArduinoBLE, create GATT services, begin advertising.
//   mode:     operating mode (see above)
//   roleChar: 'L' or 'R' — sets device name "ArcSense-L" / "ArcSense-R"
//   isMaster: if true, exposes this board's local start time to a central
// Returns true on success.
bool ble_streaming_init(BleMode mode, char roleChar, bool isMaster);

// Must be called regularly (~10 Hz) from BLEStreamThread.
void ble_streaming_poll();

// Push stride parameters (BLE_MODE_STREAMING only — no-op in other modes).
void ble_streaming_update(const StrideParameters& params);

// Push Edge AI result (BLE_MODE_EDGE_AI only — no-op in other modes).
void ble_streaming_update_edgeai(const EdgeAIResult& result);

// Returns true if a BLE central is currently connected.
bool ble_streaming_is_connected();

