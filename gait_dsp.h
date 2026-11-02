#pragma once
/*
 * Deterministic gait DSP — shared types + declarations.
 *
 * Event/feature definitions implemented here follow Rampp et al. (2015,
 * IEEE TBME) directly, with two explicit, documented departures:
 *
 *  1. ORIGINAL PAPER RAN OFFLINE, POST-SEGMENTATION. Rampp et al. first
 *     ran Barth's DTW stride segmentation (batch, with manual correction),
 *     THEN computed events within each already-known stride boundary.
 *     We run continuously in real time with no pre-segmentation step —
 *     justified for Phase 1 because our demo is a controlled, supervised
 *     walk (start/stop on command), and Haji Ghassemi et al. (2018) showed
 *     simple event/peak-based methods match segmentation-heavy methods
 *     when the walking sequence is homogeneous. This does NOT hold for
 *     unconstrained/free-living use — see FREEZE_INDEX / Phase 2 notes
 *     elsewhere for where an HMM-based segmenter would need to be added.
 *
 *  2. TRAJECTORY RECONSTRUCTION USES RAMPP'S QUATERNION + PIECEWISE-LINEAR
 *     DE-DRIFT METHOD, NOT A KALMAN FILTER. This was a deliberate choice,
 *     not a simplification of convenience: Bailey & Harle (2014) found
 *     their equivalent lightweight method (orientation reset + linear
 *     de-drift) matched or beat a full ZUPT-aided EKF for position-type
 *     metrics. Rampp's method costs no matrix inversion or covariance
 *     propagation — just quaternion multiplication and two linear
 *     interpolations per stride — so it is both the historically-
 *     validated AND the cheaper choice here. There is no EKF in this file.
 *
 * Sign/axis conventions (TO zero-crossing direction, which physical axis
 * is the sagittal gyro axis on OUR mount) are left as named calibration
 * constants in gait_dsp.cpp — these depend on sensor mounting orientation and MUST be
 * verified against our own BMI270 bench recordings, not assumed from the
 * paper's Shimmer2R mounting.
 */
 
#include <cstdint>
 
// ---------------------------------------------------------------------
// One accel+gyro sample as read out of the BMI270 FIFO, timestamped.
// ---------------------------------------------------------------------
struct ImuSample {
    uint32_t timestamp_us;
    uint32_t sensor_time;           // BMI270 FIFO ticks (~39us)
    float accelX, accelY, accelZ;   // g
    float gyroX,  gyroY,  gyroZ;    // deg/s
};
 
// Minimal quaternion (w, x, y, z) for orientation tracking between
// mid-stance events (Rampp Eq. 4-11).
struct Quaternion {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};
 
enum GaitEventType {
    EVENT_NONE = 0,
    EVENT_MID_SWING,     // MSw: sagittal-plane gyro absolute maximum
    EVENT_TOE_OFF,       // TO:  sagittal gyro zero-crossing (sign is mount-dependent)
    EVENT_HEEL_STRIKE,   // HS:  gyro-coarse + accel-refine (see .cpp)
    EVENT_MID_STANCE     // MS:  lowest-energy window across gyro axes
};
 
// One detected event, timestamped in IMU sample units (not wall time)
// so downstream stride-time math is exact regardless of scheduling jitter.
struct GaitEvent {
    GaitEventType type = EVENT_NONE;
    uint32_t sampleIndex = 0;     // index into the continuous sample counter
    uint32_t timestamp_us = 0;
};
 
// Per-stride computed parameters. Spatial fields are only valid once a
// full MS-to-MS trajectory segment has been processed (see gait_dsp.cpp);
// temporal fields are valid as soon as two consecutive HS (or TO) events
// exist.
struct StrideParameters {
    // --- Temporal (Rampp Eqs. 1-3) ---
    float strideTimeS  = 0.0f;   // tHS(i+1) - tHS(i)
    float swingTimeS   = 0.0f;   // tHS(i)   - tTO(i)
    float stanceTimeS  = 0.0f;   // tTO(i+1) - tHS(i)
 
    // --- Spatial (Rampp Sec. II-D3) ---
    float strideLengthM = 0.0f;  // 2D norm of horizontal displacement
 
    // --- Deterministic DSP additions (not from Rampp) ---
    float tibialShockG   = 0.0f; // peak resultant accel at/near HS
    float asymmetryIndex = 0.0f; // see note in gait_dsp.cpp: generic
                                  // symmetry-index formula, not sourced
                                  // from any of the four studied papers
    float freezeIndex    = 0.0f; // written by the Freeze Index thread
    uint32_t strideCount = 0;    // continuously incrementing counter
};
 
// --- Lifecycle -----------------------------------------------------------
 
// sampleRateHz MUST match the actual configured BMI270 ODR from main.ino
// (IMU_SAMPLE_RATE_HZ) — passed explicitly rather than duplicated as a
// separate constant here, so the two can never silently drift apart.
void gait_dsp_init(uint32_t sampleRateHz);
 
// --- Per-sample processing ------------------------------------------------
// Feeds one new IMU sample into the event-detection pipeline. Returns the
// event detected AT THIS SAMPLE, if any (most samples return EVENT_NONE —
// events are sparse). Internally this also feeds the trajectory
// reconstruction accumulator (see gait_dsp.cpp) between MS events.
GaitEvent gait_dsp_process_sample(const ImuSample& sample);
 
// --- Feature calculations, called by DSPThread on qualifying events ------
 
// Peak resultant acceleration in a short window around a detected HS.
// Matches Rampp's use of the accelerometer for HS refinement, but reports
// magnitude rather than just timing — this is our own addition (Objective
// 2), not part of the cited papers.
float gait_dsp_compute_tibial_shock(const ImuSample& hsWindowPeak);
 
// Generic symmetry-index formula: |paramLeft - paramRight| / mean(L, R).
// NOTE: none of the four studied papers define an asymmetry index — this
// is the standard generic symmetry-index construct from the wider gait
// literature, included here for completeness. Requires bilateral data;
// see note in gait_dsp.cpp on the single-vs-bilateral open item.
float gait_dsp_compute_asymmetry_index(float paramLeft, float paramRight);
 
// --- Optional secondary cross-check (Mo & Chow 2018, M-method) -----------
// Shank vertical-acceleration IC/TO detection. NOT wired into the primary
// haptic trigger — this exists purely as an independent cross-check signal
// for the validation section of the paper (agreement between an
// accelerometer-based and gyroscope-based feature, both from the same
// shank-mounted IMU).
struct AccelCrossCheckEvent {
    bool icDetected = false;
    bool toDetected = false;
    uint32_t icSampleIndex = 0;
    uint32_t toSampleIndex = 0;
};
AccelCrossCheckEvent gait_dsp_accel_cross_check(const ImuSample& sample);
 
// --- Access to the latest completed stride's parameters -------------------
const StrideParameters& gait_dsp_get_latest_stride();
 
// True once per newly validated stride (then clears). Use this to decide
// when to print / run asymmetry / push BLE, instead of comparing values.
bool gait_dsp_take_new_stride();
 
// Diagnostics
uint32_t gait_dsp_rejected_sample_count();
float    gait_dsp_measured_rate_hz();   // compare against IMU_SAMPLE_RATE_HZ
 