/*
 * sd_logger.cpp — Asynchronous SD-card logger for ARC SENSE Phase 2.
 *
 * See sd_logger.h for design rationale, threading contract, and
 * integration checklist.
 *
 * Changes vs. original draft:
 *   - roleChar parameter in init() → L/R prefix in filenames
 *   - optional syncOffsetUs applied only when a real offset is supplied;
 *     the current firmware does not synchronize the two device clocks
 *   - EdgeAIResult column added to stride CSV (Phase 3 provision)
 *   - Serial log messages use [SDLogger-L] / [SDLogger-R] for clarity
 */
 
#include "sd_logger.h"
#include "ring_buffer.h"
#include <Arduino.h>
#include <SPI.h>
 
#pragma pack(push, 1)
struct ImuBinaryHeader {
    char magic[4]; // 'A','R','C','1'
    uint32_t session_id;
    uint32_t start_time_us;
    char role;
    uint8_t reserved[19];
}; // 32 bytes
 
struct ImuBinaryRecord {
    uint32_t sensor_time;
    int16_t accel[3];
    int16_t gyro[3];
    uint16_t sync_marker;
    uint16_t label_marker;
}; // 20 bytes
#pragma pack(pop)
static_assert(sizeof(ImuBinaryHeader) == 32, "ARC1 header layout changed");
static_assert(sizeof(ImuBinaryRecord) == 20, "ARC1 sample record layout changed");
 
struct EventLogEntry {
    uint32_t timestamp_us;
    uint32_t sampleIndex;
    SdLogEventType type;
};
 
struct StrideLogEntry {
    uint32_t timestamp_us;
    uint32_t sampleIndex;
    StrideParameters params;
    EdgeAIResult     edgeAI;
};
 
// Queue depths: 1024 samples of 20-byte binary records = 20 KB buffer (2.56s headroom at 400 Hz)
static SpscRingBuffer<ImuBinaryRecord, 1024> s_sampleQueue;
static SpscRingBuffer<EventLogEntry,     64> s_eventQueue;
static SpscRingBuffer<StrideLogEntry,    64> s_strideQueue;
 
static SdFat s_sd;
static File32 s_imuFile;
static File32 s_eventFile;
static File32 s_strideFile;
static bool s_initialized   = false;
static bool s_filesOpen     = false;
static char s_role          = 'X';
static int32_t s_syncOffset = 0;
 
static uint8_t s_imuBlockBuffer[2048];
static uint32_t s_imuBlockOffset = 0;
 
static volatile uint16_t s_syncMarker = 0;
static volatile uint16_t s_labelMarker = 0;
static uint16_t s_sampleSeqCounter = 0;
static uint32_t s_eventsWritten = 0;
static uint32_t s_stridesWritten = 0;
static uint32_t s_rawBytesCommitted = 0;
 
void sd_logger_set_sync_marker(uint16_t marker) { s_syncMarker = marker; }
void sd_logger_set_label_marker(uint16_t label) { s_labelMarker = label; }
 
static inline uint32_t apply_sync(uint32_t ts_us) {
    int64_t corrected = (int64_t)ts_us - (int64_t)s_syncOffset;
    return (corrected < 0) ? 0 : (uint32_t)corrected;
}
 
static bool find_next_session_index(char role, uint32_t* outIndex) {
    for (uint32_t i = 0; i < 10000; i++) {
        char imuName[32], eventName[32], strideName[32];
        snprintf(imuName, sizeof(imuName), "I_%c_%04lu.BIN", role, (unsigned long)i);
        snprintf(eventName, sizeof(eventName), "E_%c_%04lu.CSV", role, (unsigned long)i);
        snprintf(strideName, sizeof(strideName), "S_%c_%04lu.CSV", role, (unsigned long)i);
        if (!s_sd.exists(imuName) && !s_sd.exists(eventName) && !s_sd.exists(strideName)) {
            *outIndex = i;
            return true;
        }
    }
    return false;
}
 
static const char* event_type_name(SdLogEventType t) {
    switch (t) {
        case SD_EVENT_MID_SWING:   return "MID_SWING";
        case SD_EVENT_TOE_OFF:     return "TOE_OFF";
        case SD_EVENT_HEEL_STRIKE: return "HEEL_STRIKE";
        case SD_EVENT_MID_STANCE:  return "MID_STANCE";
        default:                   return "UNKNOWN";
    }
}
 
static constexpr uint32_t MAX_DRAIN_PER_POLL = 256;
 
bool sd_logger_init(uint8_t chipSelectPin, char roleChar, int32_t syncOffsetUs) {
    s_role       = roleChar;
    s_syncOffset = syncOffsetUs;
    s_eventsWritten = s_stridesWritten = 0;
    s_rawBytesCommitted = 0;
    s_sampleSeqCounter = 0;
 
    SPI.begin();
    delay(50);
 
    Serial.print("[SDLogger-"); Serial.print(s_role);
    Serial.print("] Trying SdFat.begin() on CS pin "); Serial.println(chipSelectPin);
 
    bool mounted = s_sd.begin(chipSelectPin);
    if (!mounted) {
        delay(200);
        mounted = s_sd.begin(chipSelectPin);
    }
    if (!mounted) {
        Serial.print("[SDLogger-"); Serial.print(s_role);
        Serial.println("] ERROR: SdFat.begin() failed.");
        s_initialized = false;
        return false;
    }
 
    uint32_t sessionIdx;
    if (!find_next_session_index(s_role, &sessionIdx)) {
        Serial.println("[SDLogger] ERROR: no unused session index remains");
        s_initialized = false;
        return false;
    }
 
    char imuName[32], eventName[32], strideName[32];
    snprintf(imuName,    sizeof(imuName),    "I_%c_%04lu.BIN", s_role, (unsigned long)sessionIdx);
    snprintf(eventName,  sizeof(eventName),  "E_%c_%04lu.CSV", s_role, (unsigned long)sessionIdx);
    snprintf(strideName, sizeof(strideName), "S_%c_%04lu.CSV", s_role, (unsigned long)sessionIdx);
 
    s_imuFile    = s_sd.open(imuName,    O_WRITE | O_CREAT);
    s_eventFile  = s_sd.open(eventName,  O_WRITE | O_CREAT);
    s_strideFile = s_sd.open(strideName, O_WRITE | O_CREAT);
 
    if (!s_imuFile || !s_eventFile || !s_strideFile) {
        Serial.println("[SDLogger] ERROR: could not create all session files");
        if (s_imuFile) s_imuFile.close();
        if (s_eventFile) s_eventFile.close();
        if (s_strideFile) s_strideFile.close();
        s_initialized = false;
        return false;
    }
    s_filesOpen = true;
 
    // Pre-allocate 10 MB for the IMU file to prevent write stalls
    if (!s_imuFile.preAllocate(10 * 1024 * 1024)) {
        Serial.println("[SDLogger] WARNING: preAllocate failed, continuing anyway");
    }
    if (!s_imuFile.seekSet(0)) {
        Serial.println("[SDLogger] ERROR: could not seek raw file to its header");
        s_imuFile.close(); s_eventFile.close(); s_strideFile.close();
        s_filesOpen = false;
        s_initialized = false;
        return false;
    }
 
    ImuBinaryHeader header = {
        {'A', 'R', 'C', '1'},
        sessionIdx,
        apply_sync(micros()),
        s_role,
        {0}
    };
    memcpy(s_imuBlockBuffer, &header, sizeof(ImuBinaryHeader));
    s_imuBlockOffset = sizeof(ImuBinaryHeader);
 
    s_eventFile.println("session_us,timestamp_us,sampleIndex,eventType");
    s_strideFile.println("session_us,timestamp_us,sampleIndex,strideTimeS,swingTimeS,stanceTimeS,strideLengthM,tibialShockG,asymmetryIndex,freezeIndex,edgeai_label,edgeai_confidence");
 
    s_imuFile.sync();
    s_eventFile.sync();
    s_strideFile.sync();
 
    Serial.print("[SDLogger-"); Serial.print(s_role);
    Serial.println("] OK");
 
    s_initialized = true;
    return true;
}
 
void sd_logger_log_sample(const ImuSample& sample) {
    if (!s_initialized) return;
    ImuBinaryRecord rec;
    rec.sensor_time = sample.sensor_time;
    rec.accel[0] = (int16_t)(sample.accelX * (32768.0f / 16.0f));
    rec.accel[1] = (int16_t)(sample.accelY * (32768.0f / 16.0f));
    rec.accel[2] = (int16_t)(sample.accelZ * (32768.0f / 16.0f));
    rec.gyro[0]  = (int16_t)(sample.gyroX * (32768.0f / 2000.0f));
    rec.gyro[1]  = (int16_t)(sample.gyroY * (32768.0f / 2000.0f));
    rec.gyro[2]  = (int16_t)(sample.gyroZ * (32768.0f / 2000.0f));
    rec.sync_marker = s_syncMarker;
    rec.label_marker = (s_labelMarker != 0) ? s_labelMarker : s_sampleSeqCounter++;
    if (s_syncMarker != 0) s_syncMarker = 0; // Clear after logging once
 
    (void)s_sampleQueue.push(rec);
}
 
void sd_logger_log_event(SdLogEventType type, uint32_t sampleIndex, uint32_t timestamp_us) {
    if (!s_initialized) return;
    EventLogEntry e = {timestamp_us, sampleIndex, type};
    (void)s_eventQueue.push(e);
}
 
void sd_logger_log_stride(const StrideParameters& params, uint32_t hsSampleIndex, uint32_t timestamp_us, const EdgeAIResult& edgeAI) {
    if (!s_initialized) return;
    StrideLogEntry e = {timestamp_us, hsSampleIndex, params, edgeAI};
    (void)s_strideQueue.push(e);
}
 
void sd_logger_poll() {
    if (!s_initialized) return;
    static uint32_t s_pollCount = 0;
    s_pollCount++;
 
    ImuBinaryRecord rec;
    uint32_t drained = 0;
    while (drained < MAX_DRAIN_PER_POLL && s_sampleQueue.pop(rec)) {
        memcpy(&s_imuBlockBuffer[s_imuBlockOffset], &rec, sizeof(ImuBinaryRecord));
        s_imuBlockOffset += sizeof(ImuBinaryRecord);
        if (s_imuBlockOffset >= 512) {
            uint32_t fullSectors = (s_imuBlockOffset / 512) * 512;
            if (s_imuFile.write(s_imuBlockBuffer, fullSectors) != fullSectors) {
                Serial.println("[SDLogger] ERROR: short write to raw IMU file");
                s_initialized = false;
                return;
            }
            s_rawBytesCommitted += fullSectors;
            s_imuBlockOffset -= fullSectors;
            if (s_imuBlockOffset > 0) {
                memmove(s_imuBlockBuffer, s_imuBlockBuffer + fullSectors, s_imuBlockOffset);
            }
        }
        drained++;
    }

    EventLogEntry ee;
    drained = 0;
    while (drained < MAX_DRAIN_PER_POLL && s_eventQueue.pop(ee)) {
        uint32_t sessTs = apply_sync(ee.timestamp_us);
        s_eventFile.print(sessTs); s_eventFile.print(',');
        s_eventFile.print(ee.timestamp_us); s_eventFile.print(',');
        s_eventFile.print(ee.sampleIndex); s_eventFile.print(',');
        s_eventFile.println(event_type_name(ee.type));
        s_eventsWritten++;
        drained++;
    }
 
    StrideLogEntry sle;
    drained = 0;
    while (drained < MAX_DRAIN_PER_POLL && s_strideQueue.pop(sle)) {
        uint32_t sessTs = apply_sync(sle.timestamp_us);
        s_strideFile.print(sessTs); s_strideFile.print(',');
        s_strideFile.print(sle.timestamp_us); s_strideFile.print(',');
        s_strideFile.print(sle.sampleIndex); s_strideFile.print(',');
        s_strideFile.print(sle.params.strideTimeS, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.swingTimeS, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.stanceTimeS, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.strideLengthM, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.tibialShockG, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.asymmetryIndex, 4); s_strideFile.print(',');
        s_strideFile.print(sle.params.freezeIndex, 4); s_strideFile.print(',');
        s_strideFile.print(sle.edgeAI.label); s_strideFile.print(',');
        s_strideFile.println(sle.edgeAI.confidence, 4);
        s_stridesWritten++;
        drained++;
    }
 
    if (s_pollCount % 100 == 0) {
        s_imuFile.sync();
        s_eventFile.sync();
        s_strideFile.sync();
    }
}
 
void sd_logger_get_drop_counts(uint32_t* samples, uint32_t* events,
                               uint32_t* strides) {
    if (samples) *samples = s_sampleQueue.get_drops();
    if (events) *events = s_eventQueue.get_drops();
    if (strides) *strides = s_strideQueue.get_drops();
}
 
void sd_logger_get_written_counts(uint32_t* samples, uint32_t* events,
                                  uint32_t* strides) {
    if (samples) {
        uint32_t sampleBytes = (s_rawBytesCommitted > sizeof(ImuBinaryHeader))
            ? s_rawBytesCommitted - sizeof(ImuBinaryHeader) : 0;
        *samples = sampleBytes / sizeof(ImuBinaryRecord);
    }
    if (events) *events = s_eventsWritten;
    if (strides) *strides = s_stridesWritten;
}
 
void sd_logger_close() {
    if (!s_filesOpen) return;
    
    // Flush remaining IMU block
    if (s_imuBlockOffset > 0) {
        uint32_t wrote = s_imuFile.write(s_imuBlockBuffer, s_imuBlockOffset);
        if (wrote == s_imuBlockOffset) s_rawBytesCommitted += wrote;
        else Serial.println("[SDLogger] ERROR: failed to flush final raw IMU tail");
        s_imuBlockOffset = 0;
    }
    
    // Truncate the file to the actual length written since we pre-allocated it
    s_imuFile.truncate(s_imuFile.position());
 
    s_imuFile.sync();    s_imuFile.close();
    s_eventFile.sync();  s_eventFile.close();
    s_strideFile.sync(); s_strideFile.close();
    s_filesOpen = false;
    s_initialized = false;
    Serial.print("[SDLogger-"); Serial.print(s_role);
    Serial.println("] closed");
}