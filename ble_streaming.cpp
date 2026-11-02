/*
 * ble_streaming.cpp — BLE GATT service for ARC SENSE Phase 2.
 *
 * Custom 128-bit UUID base: 19B10000-E8F2-537E-4F6C-D104768A1214
 *
 * UUID map:
 *   0x0000  Gait parameter service (streaming mode)
 *   0x0001  Stride time
 *   0x0002  Swing time
 *   0x0003  Stance time
 *   0x0004  Stride length
 *   0x0005  Tibial shock
 *   0x0006  Asymmetry index
 *   0x0007  Freeze index
 *
 *   0x0008  Device info service
 *   0x0009  Device role ('L'/'R' as uint8)
 *   0x000A  BLE mode (uint8)
 *
 *   0x000B  Sync service
 *   0x000C  Session start timestamp (uint32, microseconds, written by master)
 *
 *   0x000D  Edge AI service  (Phase 3 provision — characteristics exist,
 *   0x000E  AI label (int8)   not yet wired to inference output)
 *   0x000F  AI confidence (float)
 */

#include "ble_streaming.h"
#include <ArduinoBLE.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// UUIDs
// ---------------------------------------------------------------------------
// Gait streaming
static const char* SVC_GAIT_UUID       = "19B10000-E8F2-537E-4F6C-D104768A1214";
static const char* STRIDE_TIME_UUID    = "19B10001-E8F2-537E-4F6C-D104768A1214";
static const char* SWING_TIME_UUID     = "19B10002-E8F2-537E-4F6C-D104768A1214";
static const char* STANCE_TIME_UUID    = "19B10003-E8F2-537E-4F6C-D104768A1214";
static const char* STRIDE_LEN_UUID     = "19B10004-E8F2-537E-4F6C-D104768A1214";
static const char* TIBIAL_SHOCK_UUID   = "19B10005-E8F2-537E-4F6C-D104768A1214";
static const char* ASYMMETRY_UUID      = "19B10006-E8F2-537E-4F6C-D104768A1214";
static const char* FREEZE_IDX_UUID     = "19B10007-E8F2-537E-4F6C-D104768A1214";
static const char* STRIDE_COUNT_UUID   = "19B10013-E8F2-537E-4F6C-D104768A1214";

// Device info
static const char* SVC_INFO_UUID       = "19B10008-E8F2-537E-4F6C-D104768A1214";
static const char* ROLE_UUID           = "19B10009-E8F2-537E-4F6C-D104768A1214";
static const char* BLE_MODE_UUID       = "19B1000A-E8F2-537E-4F6C-D104768A1214";

// Sync
static const char* SVC_SYNC_UUID       = "19B1000B-E8F2-537E-4F6C-D104768A1214";
static const char* SYNC_TS_UUID        = "19B1000C-E8F2-537E-4F6C-D104768A1214";

// Command channel (markers)
static const char* SVC_CMD_UUID        = "19B10010-E8F2-537E-4F6C-D104768A1214";
static const char* SYNC_MARKER_UUID    = "19B10011-E8F2-537E-4F6C-D104768A1214";
static const char* LABEL_MARKER_UUID   = "19B10012-E8F2-537E-4F6C-D104768A1214";

// Edge AI (provision)
static const char* SVC_EDGEAI_UUID     = "19B1000D-E8F2-537E-4F6C-D104768A1214";
static const char* EDGEAI_LABEL_UUID   = "19B1000E-E8F2-537E-4F6C-D104768A1214";
static const char* EDGEAI_CONF_UUID    = "19B1000F-E8F2-537E-4F6C-D104768A1214";

// ---------------------------------------------------------------------------
// GATT services
// ---------------------------------------------------------------------------
static BLEService gaitService(SVC_GAIT_UUID);
static BLEService infoService(SVC_INFO_UUID);
static BLEService syncService(SVC_SYNC_UUID);
static BLEService cmdService(SVC_CMD_UUID);
static BLEService edgeAIService(SVC_EDGEAI_UUID);

// --- Gait streaming characteristics ---
static BLEFloatCharacteristic strideTimeChar  (STRIDE_TIME_UUID, BLERead | BLENotify);
static BLEFloatCharacteristic swingTimeChar   (SWING_TIME_UUID,  BLERead | BLENotify);
static BLEFloatCharacteristic stanceTimeChar  (STANCE_TIME_UUID, BLERead | BLENotify);
static BLEFloatCharacteristic strideLenChar   (STRIDE_LEN_UUID,  BLERead | BLENotify);
static BLEFloatCharacteristic tibialShockChar (TIBIAL_SHOCK_UUID,BLERead | BLENotify);
static BLEFloatCharacteristic asymmetryChar   (ASYMMETRY_UUID,   BLERead | BLENotify);
static BLEFloatCharacteristic freezeIdxChar   (FREEZE_IDX_UUID,  BLERead | BLENotify);
static BLEUnsignedIntCharacteristic strideCountChar(STRIDE_COUNT_UUID, BLERead | BLENotify);

// --- Device info characteristics ---
static BLEByteCharacteristic  deviceRoleChar  (ROLE_UUID,        BLERead);
static BLEByteCharacteristic  bleModeChar     (BLE_MODE_UUID,    BLERead);

// --- Sync characteristics ---
// Exposes this board's local session-start micros() value. The current
// firmware does not exchange a timestamp between the two boards, so this is
// not a bilateral clock synchronization mechanism.
static BLEUnsignedIntCharacteristic syncTsChar(SYNC_TS_UUID, BLERead | BLEWrite | BLENotify);

// --- Command channel characteristics ---
static BLEUnsignedShortCharacteristic syncMarkerChar(SYNC_MARKER_UUID, BLERead | BLEWrite);
static BLEUnsignedShortCharacteristic labelMarkerChar(LABEL_MARKER_UUID, BLERead | BLEWrite);

// --- Edge AI characteristics (provision) ---
static BLEByteCharacteristic  aiLabelChar     (EDGEAI_LABEL_UUID, BLERead | BLENotify);
static BLEFloatCharacteristic aiConfChar      (EDGEAI_CONF_UUID,  BLERead | BLENotify);

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static BleMode  s_mode          = BLE_MODE_DATA_COLLECTION;
static char     s_role          = 'X';
static bool     s_isMaster      = false;
static bool     s_centralConnected = false;
static uint32_t s_sessionStartUs  = 0;   // master's session start (micros)

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
bool ble_streaming_init(BleMode mode, char roleChar, bool isMaster) {
    s_mode     = mode;
    s_role     = roleChar;
    s_isMaster = isMaster;

    if (!BLE.begin()) {
        Serial.println("[BLE] ERROR: BLE.begin() failed");
        return false;
    }

    // Device name encodes role so both devices are distinguishable by the
    // central / mobile app without needing to read a characteristic first.
    char devName[16];
    snprintf(devName, sizeof(devName), "ArcSense-%c", s_role);
    BLE.setLocalName(devName);
    BLE.setDeviceName(devName);

    // --- Set up Device Info service (always present) ---
    infoService.addCharacteristic(deviceRoleChar);
    infoService.addCharacteristic(bleModeChar);
    BLE.addService(infoService);
    deviceRoleChar.writeValue((uint8_t)s_role);
    bleModeChar.writeValue((uint8_t)s_mode);

    // --- Set up Sync service (always present) ---
    syncService.addCharacteristic(syncTsChar);
    BLE.addService(syncService);
    if (s_isMaster) {
        s_sessionStartUs = (uint32_t)micros();
        syncTsChar.writeValue(s_sessionStartUs);
        Serial.print("[BLE] Master session start: "); Serial.println(s_sessionStartUs);
    } else {
        syncTsChar.writeValue((uint32_t)0);
    }

    // --- Set up Command channel service (always present) ---
    cmdService.addCharacteristic(syncMarkerChar);
    cmdService.addCharacteristic(labelMarkerChar);
    BLE.addService(cmdService);
    syncMarkerChar.writeValue((uint16_t)0);
    labelMarkerChar.writeValue((uint16_t)0);

    // --- Set up Edge AI service (provision — always present but silent) ---
    edgeAIService.addCharacteristic(aiLabelChar);
    edgeAIService.addCharacteristic(aiConfChar);
    BLE.addService(edgeAIService);
    aiLabelChar.writeValue((uint8_t)255);  // 255 = no inference
    aiConfChar.writeValue(0.0f);

    // --- Set up Gait streaming service ---
    // In DATA_COLLECTION mode these characteristics exist but are never
    // notified — they just hold the last stride's values, readable on
    // demand. This keeps the service table consistent across modes so a
    // central can connect to either mode without needing a different UUID list.
    gaitService.addCharacteristic(strideTimeChar);
    gaitService.addCharacteristic(swingTimeChar);
    gaitService.addCharacteristic(stanceTimeChar);
    gaitService.addCharacteristic(strideLenChar);
    gaitService.addCharacteristic(tibialShockChar);
    gaitService.addCharacteristic(asymmetryChar);
    gaitService.addCharacteristic(freezeIdxChar);
    gaitService.addCharacteristic(strideCountChar);
    BLE.addService(gaitService);

    // Zero-init gait characteristics
    strideTimeChar.writeValue(0.0f);
    swingTimeChar.writeValue(0.0f);
    stanceTimeChar.writeValue(0.0f);
    strideLenChar.writeValue(0.0f);
    tibialShockChar.writeValue(0.0f);
    asymmetryChar.writeValue(0.0f);
    freezeIdxChar.writeValue(0.0f);
    strideCountChar.writeValue((uint32_t)0);

    // Advertise the gait service UUID so centrals can filter by it.
    BLE.setAdvertisedService(gaitService);
    BLE.advertise();

    Serial.print("[BLE] OK — advertising as \"");
    Serial.print(devName);
    Serial.print("\" in mode ");
    Serial.println((int)s_mode);

    return true;
}

// ---------------------------------------------------------------------------
// Poll
// ---------------------------------------------------------------------------
void ble_streaming_poll() {
    BLE.poll();

    BLEDevice central = BLE.central();
    if (central) {
        if (!s_centralConnected) {
            s_centralConnected = true;
            Serial.print("[BLE] Central connected: ");
            Serial.println(central.address());

            // Keep the master's local timestamp characteristic current for
            // a central app. The second board does not read it automatically.
            if (s_isMaster) {
                syncTsChar.writeValue(s_sessionStartUs);
            }
        }
        if (!central.connected()) {
            s_centralConnected = false;
            Serial.println("[BLE] Central disconnected — re-advertising");
            BLE.advertise();
        } else {
            // Check for incoming markers
            if (syncMarkerChar.written()) {
                sd_logger_set_sync_marker(syncMarkerChar.value());
                Serial.print("[BLE] Received sync marker: ");
                Serial.println(syncMarkerChar.value());
            }
            if (labelMarkerChar.written()) {
                sd_logger_set_label_marker(labelMarkerChar.value());
                Serial.print("[BLE] Received label marker: ");
                Serial.println(labelMarkerChar.value());
            }
        }
    } else if (s_centralConnected) {
        s_centralConnected = false;
        Serial.println("[BLE] Central disconnected");
        BLE.advertise();
    }
}

// ---------------------------------------------------------------------------
// Gait parameter update — BLE_MODE_STREAMING only
// ---------------------------------------------------------------------------
void ble_streaming_update(const StrideParameters& params) {
    if (s_mode != BLE_MODE_STREAMING) return;

    strideTimeChar.writeValue(params.strideTimeS);
    swingTimeChar.writeValue(params.swingTimeS);
    stanceTimeChar.writeValue(params.stanceTimeS);
    strideLenChar.writeValue(params.strideLengthM);
    tibialShockChar.writeValue(params.tibialShockG);
    asymmetryChar.writeValue(params.asymmetryIndex);
    freezeIdxChar.writeValue(params.freezeIndex);
    strideCountChar.writeValue(params.strideCount);
}

// ---------------------------------------------------------------------------
// Edge AI result update — BLE_MODE_EDGE_AI only (provision for Phase 3)
// ---------------------------------------------------------------------------
void ble_streaming_update_edgeai(const EdgeAIResult& result) {
    if (s_mode != BLE_MODE_EDGE_AI) return;
    aiLabelChar.writeValue((uint8_t)(result.label < 0 ? 255 : result.label));
    aiConfChar.writeValue(result.confidence);
}

// ---------------------------------------------------------------------------
// Connection status
// ---------------------------------------------------------------------------
bool ble_streaming_is_connected() {
    return s_centralConnected;
}

// ---------------------------------------------------------------------------
