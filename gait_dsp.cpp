/*
 * gait_dsp.cpp — deterministic gait DSP, implemented from:
 *   Rampp et al. 2015 (IEEE TBME)      — event detection, temporal
 *                                          parameters, trajectory/stride
 *                                          length
 *   Bailey & Harle 2014 (Procedia Eng) — corroborates lightweight
 *                                          de-drift over EKF for position
 *   Haji Ghassemi et al. 2018 (Sensors)— justifies skipping segmentation
 *                                          for a controlled/supervised walk
 *   Mo & Chow 2018 (Gait & Posture)    — optional accel cross-check (M-method)
 *
 * All numeric CONSTANTS below are named and grouped so they can be
 * recalibrated from our own BMI270 bench recordings without touching
 * algorithm logic. Do not treat these starting values as final —
 * see the comment on each group for what it's based on vs. what needs
 * verification on our hardware.
 */
 
#include "gait_dsp.h"
#include <cmath>
#include <cstring>
#include <Arduino.h>
 
// ===========================================================================
// SECTION 1 — Calibration constants
// ===========================================================================
 
// --- Axis / sign conventions -----------------------------------------------
// Rampp's Shimmer2R used its Z gyro for medio-lateral (sagittal-plane rotation),
// x = posterior-anterior, y = superior-inferior, mounted laterally below
// the ankle. OUR axis mapping depends entirely on how the BMI270 is
// physically oriented on the ankle mount — this is NOT assumed to be a
// direct match and MUST be confirmed from a labeled bench recording
// before trusting sign-dependent logic (TO zero-crossing direction below).
// Until confirmed, these are our best-guess placeholders.
static constexpr int   GYRO_SAGITTAL_AXIS       = 2;      // Z = Medio-lateral rotation (sagittal plane)
static constexpr float GYRO_SAGITTAL_SIGN       = -1.0f;  // Invert Gyro Z: physical forward swing becomes positive (+200 dps)
static constexpr int   ACCEL_AP_AXIS            = 0;      // X = Anterior-Posterior
static constexpr int   ACCEL_VERTICAL_AXIS      = 1;      // Y = Superior-Inferior (gravity is on Y in this mount)
 
// Direction of the gyro sagittal-axis zero-crossing at toe-off.
// With inverted Gyro Z (positive during forward swing), the plantarflexion-to-
// dorsiflexion transition is a negative-to-positive crossing (+1).
static constexpr int TOE_OFF_CROSSING_DIRECTION = +1;
 
// --- Mid-swing (MSw) detection: adaptive threshold, not fixed deg/s -------
// Rationale (see prior discussion): absolute deg/s thresholds from the
// literature were tuned for different hardware/subjects/mounting and do
// not transfer. We gate on a percentage of a running peak estimate instead.
static constexpr float MSW_ADAPTIVE_MARGIN_PCT   = 0.60f; // candidate peak
                                                           // must exceed this
                                                           // fraction of the
                                                           // running peak
                                                           // estimate — TUNE
static constexpr float MSW_PEAK_EMA_ALPHA        = 0.2f;  // smoothing factor
                                                           // for the running
                                                           // peak estimate
 
// --- Physiologically plausible stride-time bounds -------------------------
// Reused across Rampp/Barth-derived work (also appears as the same 600ms/
// 2500ms pair in Haji Ghassemi et al.'s peak-detection implementation) as
// a STRUCTURAL bound, not a magnitude threshold — much more likely to
// transfer across hardware than a raw deg/s cutoff, but still worth
// sanity-checking once we have our own recordings.
static constexpr float MIN_STRIDE_TIME_S = 0.6f;
static constexpr float MAX_STRIDE_TIME_S = 2.5f;
 
// --- Heel-strike (HS) refinement window ------------------------------------
// Rampp: search accel minimum in [-50ms, +20ms] around the gyro-coarse
// estimate. These offsets are given directly in the paper (not hardware-
// specific in the same way peak magnitudes are), so we carry them as-is.
static constexpr float HS_REFINE_WINDOW_BEFORE_S = 0.050f;
static constexpr float HS_REFINE_WINDOW_AFTER_S  = 0.150f;
 
// --- Mid-stance (MS) window ------------------------------------------------
// Rampp: 140ms window, 70ms overlap, lowest combined gyro energy.
static constexpr float MS_WINDOW_S   = 0.140f;
static constexpr float MS_OVERLAP_S  = 0.070f;
 
// --- Trajectory de-drift proportions ---------------------------------------
// Rampp: first 4% / last 2% of stride samples define the two drift-model
// plateaus (Eq. 16-18). These are proportions of stride length, not
// hardware-dependent, so carried as-is.
static constexpr float DRIFT_PLATEAU_START_FRAC = 0.04f;
static constexpr float DRIFT_PLATEAU_END_FRAC   = 0.02f;
 
// --- HS timing guards (tune from walking logs) -----------------------------
// HS search starts this long after MSw. Walking log showed HS 0.15-0.25 s
// after MSw, so 0.08 s leaves margin. If HS stops resolving or shifts late
// after this patch, lower it (0.05 s).
static constexpr float HS_MIN_DELAY_AFTER_MSW_S = 0.10f;
 
// --- Tibial shock window around the refined HS ------------------------------
static constexpr float SHOCK_WINDOW_BEFORE_S = 0.050f;
static constexpr float SHOCK_WINDOW_AFTER_S  = 0.100f;
 
// --- Startup: first stride interval after power-on is unreliable ------------
static constexpr uint32_t STARTUP_HS_TO_DISCARD = 0;
 
// --- Sample plausibility (see sample_is_plausible) --------------------------
static constexpr float GYRO_PLAUSIBLE_MAX_DPS  = 1000.0f; // ankle swing < ~700
static constexpr float ACCEL_PLAUSIBLE_MAX_G   = 16.0f;
 
// 0 = stride/swing/stance from sample INDEX / s_sampleRateHz (what worked in
// the Phase-1 walk). 1 = from timestamp_us. Check gait_dsp_measured_rate_hz()
// first: if it is far from IMU_SAMPLE_RATE_HZ, switch to 1.
#define USE_TIMESTAMP_TIMEBASE 1
 
// --- Buffer sizing ----------------------------------------------------------
// Must hold at least one full MAX_STRIDE_TIME_S plus the delay between
// MS and MSw. Sized generously (5s worth) to tolerate slow/shuffling gait.
static constexpr uint32_t MAX_STRIDE_BUFFER_SAMPLES = 512; // ~5s @ 100Hz
 
// ===========================================================================
// SECTION 2 — Module state
// ===========================================================================
 
static uint32_t s_sampleRateHz = 200;
static uint32_t s_sampleCounter = 0;
 
// Rolling sagittal-gyro history for backward search (TO before a confirmed MSw,
// HS window before/after). Sized to comfortably hold one stride.
static float    s_gyroSagittalHist[MAX_STRIDE_BUFFER_SAMPLES];
static float    s_accelApHist[MAX_STRIDE_BUFFER_SAMPLES];
static float    s_accelVerticalHist[MAX_STRIDE_BUFFER_SAMPLES];
static ImuSample s_sampleHist[MAX_STRIDE_BUFFER_SAMPLES];
static ImuSample s_trajScratch[MAX_STRIDE_BUFFER_SAMPLES];
static uint32_t s_histWriteIdx = 0;
static uint32_t s_histCount = 0; // caps at MAX_STRIDE_BUFFER_SAMPLES
 
// Mid-swing peak-tracking state (causal single-pass peak detection)
static float    s_runningPeakEma = 100.0f; // deg/s, seeded conservatively
static bool     s_risingEdgeActive = false;
static float    s_candidatePeakVal = 0.0f;
static uint32_t s_candidatePeakIdx = 0;
static uint32_t s_lastMSwIdx = 0;
static bool     s_haveLastMSw = false;
 
// Event timestamps for temporal parameter calculation
static uint32_t s_lastToIdx = 0, s_lastHsIdx = 0;
static bool     s_haveLastTo = false, s_haveLastHs = false;
static uint32_t s_prevHsIdx = 0;
static bool     s_havePrevHs = false;
static uint32_t s_lastMsIdx = 0;
static bool     s_haveLastMs = false;
 
static GaitEvent s_eventQueue[8];
static uint32_t s_eventQueueHead = 0;
static uint32_t s_eventQueueTail = 0;
 
static void queue_event(GaitEventType type, uint32_t idx, uint32_t ts) {
    if (s_eventQueueTail - s_eventQueueHead < 8) {
        s_eventQueue[s_eventQueueTail % 8] = {type, idx, ts};
        s_eventQueueTail++;
    }
}
 
// Trajectory reconstruction state (accumulated between MS events)
static Quaternion s_orientation;
static uint32_t   s_trajSegmentStartIdx = 0;
static bool       s_trajSegmentActive = false;
 
static StrideParameters s_latestStride;
 
static uint32_t s_hsCount = 0;
static bool     s_newStrideFlag = false;
static uint32_t s_rejectedSamples = 0;
static ImuSample s_lastValidSample;
static bool     s_haveLastValid = false;
static uint32_t s_lastHsTs = 0, s_prevHsTs = 0, s_lastToTs = 0;
static uint32_t s_tsFirst = 0, s_tsLatest = 0;
static bool     s_haveTsFirst = false;
 
// Per-MSw-cycle flag: has an HS already been resolved for the current MSw?
// Reset to false each time a new MSw is confirmed (Section 9), set to true
// once find_heel_strike_after() succeeds. Kept as a simple bool rather
// than richer per-MSw state — known simplification, revisit once running
// against real bench data.
static bool s_hsResolvedFlag = false;
static inline bool s_haveLastHs_ResolvedForThisMSw() { return s_hsResolvedFlag; }
 
// --- Mid-stance (MS) detection state ----------------------------------------
// After each HS, we compute gyro energy in sliding windows during stance.
// The window with minimum energy is the MS candidate. It's confirmed when
// the next MSw fires (which signals the end of stance phase).
static bool     s_msSearchActive = false;       // actively searching for MS
static float    s_msMinEnergy = 1e9f;            // lowest energy seen so far
static uint32_t s_msBestIdx = 0;                 // sample index of best MS
static float    s_msEnergyWindow[32] = {};
static uint32_t s_msEnergyWrite = 0;
static uint32_t s_msEnergyCount = 0;
static uint32_t s_msSamplesSinceEval = 0;
 
// ===========================================================================
// SECTION 3 — Small math helpers
// ===========================================================================
 
static Quaternion quat_multiply(const Quaternion& a, const Quaternion& b) {
    Quaternion r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}
 
static Quaternion quat_normalize(const Quaternion& q) {
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm < 1e-9f) return Quaternion{1,0,0,0};
    Quaternion r;
    r.w = q.w/norm; r.x = q.x/norm; r.y = q.y/norm; r.z = q.z/norm;
    return r;
}
 
 
 
// ===========================================================================
// SECTION 4 — Lifecycle
// ===========================================================================
 
void gait_dsp_init(uint32_t sampleRateHz) {
    s_sampleRateHz = sampleRateHz;
    s_sampleCounter = 0;
    s_histWriteIdx = 0;
    s_histCount = 0;
    s_risingEdgeActive = false;
    s_haveLastMSw = false;
    s_haveLastTo = false;
    s_haveLastHs = false;
    s_havePrevHs = false;
    s_haveLastMs = false;
    s_trajSegmentActive = false;
    s_orientation = Quaternion{1, 0, 0, 0};
    s_latestStride = StrideParameters{};
    // MS detection reset
    s_msSearchActive = false;
    s_msMinEnergy = 1e9f;
    s_msBestIdx = 0;
    memset(s_msEnergyWindow, 0, sizeof(s_msEnergyWindow));
    s_msEnergyWrite = 0;
    s_msEnergyCount = 0;
    s_msSamplesSinceEval = 0;
    
    s_hsCount = 0;  s_newStrideFlag = false;  s_rejectedSamples = 0;
    s_haveLastValid = false;  s_haveTsFirst = false;
    s_lastHsTs = s_prevHsTs = s_lastToTs = 0;
    
    s_eventQueueHead = 0;
    s_eventQueueTail = 0;
}
 
static inline uint32_t seconds_to_samples(float seconds) {
    return static_cast<uint32_t>(seconds * static_cast<float>(s_sampleRateHz));
}
 
// Timestamp of an absolute sample index, if still in the rolling history.
static bool ts_at_index(uint32_t absIdx, uint32_t* ts) {
    if (absIdx > s_sampleCounter) return false;
    uint32_t age = s_sampleCounter - absIdx;
    if (age >= s_histCount) return false;
    uint32_t ring = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - age - 1)
                    % MAX_STRIDE_BUFFER_SAMPLES;
    *ts = s_sampleHist[ring].timestamp_us;
    return true;
}
 
static float elapsed_s(uint32_t idxA, uint32_t tsA, uint32_t idxB, uint32_t tsB) {
#if USE_TIMESTAMP_TIMEBASE
    (void)idxA; (void)idxB;
    return static_cast<float>(static_cast<uint32_t>(tsB - tsA)) * 1e-6f; // wrap-safe
#else
    (void)tsA; (void)tsB;
    return static_cast<float>(idxB - idxA) / static_cast<float>(s_sampleRateHz);
#endif
}
 
// Access sagittal gyro at absolute sample index from rolling history
static inline float hist_sagittal_gyro(uint32_t absIdx) {
    if (absIdx > s_sampleCounter) return 0.0f;
    uint32_t age = s_sampleCounter - absIdx;
    if (age >= s_histCount) return 0.0f;
    uint32_t ring = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - age - 1) % MAX_STRIDE_BUFFER_SAMPLES;
    return s_gyroSagittalHist[ring];
}

// Access resultant acceleration magnitude at absolute sample index from rolling history
static inline float hist_resultant_accel(uint32_t absIdx) {
    if (absIdx > s_sampleCounter) return 0.0f;
    uint32_t age = s_sampleCounter - absIdx;
    if (age >= s_histCount) return 0.0f;
    uint32_t ring = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - age - 1) % MAX_STRIDE_BUFFER_SAMPLES;
    const ImuSample& s = s_sampleHist[ring];
    return sqrtf(s.accelX * s.accelX + s.accelY * s.accelY + s.accelZ * s.accelZ);
}

// Peak resultant acceleration (g) in [hs - 50 ms, hs + 100 ms].
static float peak_resultant_around(uint32_t hsIdx) {
    uint32_t b = seconds_to_samples(SHOCK_WINDOW_BEFORE_S);
    uint32_t a = seconds_to_samples(SHOCK_WINDOW_AFTER_S);
    uint32_t start = (hsIdx > b) ? (hsIdx - b) : 0;
    uint32_t end   = hsIdx + a;
    if (end > s_sampleCounter) end = s_sampleCounter;
    float peak = 0.0f;
    for (uint32_t i = start; i <= end; i++) {
        float m = hist_resultant_accel(i);
        if (m > peak) peak = m;
    }
    return peak;
}
 
// Reject invalid or out-of-range samples. Do not reject exact zero values on
// selected axes: those are valid measurements for some sensor orientations.
static bool sample_is_plausible(const ImuSample& s) {
    const float a[3] = {s.accelX, s.accelY, s.accelZ};
    const float g[3] = {s.gyroX,  s.gyroY,  s.gyroZ};
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(a[i]) || !std::isfinite(g[i])) return false;
        if (fabsf(a[i]) > ACCEL_PLAUSIBLE_MAX_G)           return false;
        if (fabsf(g[i]) >= GYRO_PLAUSIBLE_MAX_DPS)         return false;
    }
    return true;
}
 
// ===========================================================================
// SECTION 5 — Mid-swing (MSw) detection
// Causal single-pass peak detection: track a candidate peak once the
// signal rises above an adaptive fraction of the running peak estimate;
// confirm the peak once the signal has fallen back down by a hysteresis
// margin (i.e., we detect MSw one small step AFTER it actually occurred —
// unavoidable for a true local-maximum test on a causal stream).
// ===========================================================================
 
static bool detect_mid_swing(float gyroSagittal, uint32_t idx, uint32_t* outIdx) {
    float threshold = s_runningPeakEma * MSW_ADAPTIVE_MARGIN_PCT;
 
    if (!s_risingEdgeActive) {
        if (gyroSagittal > threshold) {
            s_risingEdgeActive = true;
            s_candidatePeakVal = gyroSagittal;
            s_candidatePeakIdx = idx;
        }
        return false;
    }
 
    // Still rising — update candidate
    if (gyroSagittal >= s_candidatePeakVal) {
        s_candidatePeakVal = gyroSagittal;
        s_candidatePeakIdx = idx;
        return false;
    }
 
    // Falling — confirm peak once it has dropped back below threshold
    // (guards against noise creating a premature confirm on a shoulder)
    if (gyroSagittal < threshold * 0.7f) {
        s_risingEdgeActive = false;
 
        // Enforce minimum stride-time guard against the previous MSw
        if (s_haveLastMSw) {
            float dt = static_cast<float>(s_candidatePeakIdx - s_lastMSwIdx)
                       / static_cast<float>(s_sampleRateHz);
            if (dt < MIN_STRIDE_TIME_S) {
                return false; // too soon — likely noise, reject silently
            }
        }
 
        // Update the running peak estimate (EMA) with the confirmed peak
        s_runningPeakEma = (1.0f - MSW_PEAK_EMA_ALPHA) * s_runningPeakEma
                          + MSW_PEAK_EMA_ALPHA * s_candidatePeakVal;
 
        *outIdx = s_candidatePeakIdx;
        return true;
    }
 
    return false;
}
 
// ===========================================================================
// SECTION 6 — Toe-off (TO): gyro sagittal-axis zero-crossing
// Rampp: TO is the zero-crossing where the ankle moves from plantarflexion
// to dorsal extension. We search backward from a confirmed MSw for the
// most recent qualifying crossing, bounded by the previous HS (a TO cannot
// occur before the HS that started the preceding stance phase).
// ===========================================================================
 
static bool find_toe_off_before(uint32_t msWIdx, uint32_t searchLowerBound,
                                 uint32_t* outIdx) {
    if (s_histCount < 2) return false;
 
    // Walk backward through history from msWIdx toward searchLowerBound
    uint32_t startOffset = (s_sampleCounter > msWIdx) ? (s_sampleCounter - msWIdx) : 0;
    uint32_t endOffset    = (s_sampleCounter > searchLowerBound) ? (s_sampleCounter - searchLowerBound) : s_histCount;
    endOffset = (endOffset < s_histCount) ? endOffset : (s_histCount - 1);
 
    for (uint32_t off = startOffset; off + 1 < endOffset; off++) {
        // Map "samples back from now" to history buffer indices
        uint32_t idxNewer = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 1) % MAX_STRIDE_BUFFER_SAMPLES;
        uint32_t idxOlder  = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 2) % MAX_STRIDE_BUFFER_SAMPLES;
 
        float newer = s_gyroSagittalHist[idxNewer];
        float older = s_gyroSagittalHist[idxOlder];
 
        bool crossesUp   = (older < 0.0f && newer >= 0.0f);
        bool crossesDown = (older > 0.0f && newer <= 0.0f);
 
        if ((TOE_OFF_CROSSING_DIRECTION > 0 && crossesUp) ||
            (TOE_OFF_CROSSING_DIRECTION < 0 && crossesDown)) {
            *outIdx = s_sampleCounter - off - 1;
            return true; // most-recent qualifying crossing found first
        }
    }
    return false;
}
 
// ===========================================================================
// SECTION 7 — Heel-strike (HS): gyro-coarse, accel-refine
// Rampp: within [absolute-max .. swing-to-swing midpoint], find the
// minimum between steepest-negative-slope and steepest-positive-slope
// points in gyro; refine with an accel-AP minimum in [-50ms,+20ms].
// ===========================================================================
 
static bool find_heel_strike_after(uint32_t msWIdx, uint32_t* outIdx) {
    if (s_histCount < 4) return false;
 
    // Guard 1: don't look until HS_MIN_DELAY_AFTER_MSW_S has passed
    uint32_t minDelay     = seconds_to_samples(HS_MIN_DELAY_AFTER_MSW_S);
    uint32_t forwardLimit = seconds_to_samples(MAX_STRIDE_TIME_S * 0.5f);
    uint32_t searchStart  = msWIdx + minDelay;
    if (s_sampleCounter <= searchStart + 2) return false;
    uint32_t searchEnd = msWIdx + forwardLimit;
    if (searchEnd > s_sampleCounter) searchEnd = s_sampleCounter;
 
    // 1. Search for coarse HS: negative-going zero-crossing of sagittal gyro
    uint32_t coarseHsIdx = 0;
    bool foundCoarse = false;
    for (uint32_t i = searchStart; i <= searchEnd; i++) {
        float older = hist_sagittal_gyro(i - 1);
        float newer = hist_sagittal_gyro(i);
        if (older > 0.0f && newer <= 0.0f) {
            coarseHsIdx = i;
            foundCoarse = true;
            break;
        }
    }
 
    // Fallback: if no zero-crossing after 350 ms, look for local minimum of sagittal gyro
    if (!foundCoarse && (searchEnd - msWIdx >= seconds_to_samples(0.35f))) {
        float minVal = 1e9f;
        uint32_t minIdx = searchStart;
        for (uint32_t i = searchStart; i <= searchEnd; i++) {
            float val = hist_sagittal_gyro(i);
            if (val < minVal) {
                minVal = val;
                minIdx = i;
            }
        }
        if (minIdx + 3 <= searchEnd) { // confirmed local trough
            coarseHsIdx = minIdx;
            foundCoarse = true;
        }
    }
 
    if (!foundCoarse) return false;
 
    // Guard 2: wait until full forward refinement window has arrived in history
    uint32_t afterSamples = seconds_to_samples(HS_REFINE_WINDOW_AFTER_S);
    if (s_sampleCounter < coarseHsIdx + afterSamples) return false;
 
    uint32_t beforeSamples = seconds_to_samples(HS_REFINE_WINDOW_BEFORE_S);
    uint32_t refineStart   = (coarseHsIdx > beforeSamples) ? (coarseHsIdx - beforeSamples) : (msWIdx + 1);
    if (refineStart <= msWIdx) refineStart = msWIdx + 1;
    uint32_t refineEnd     = coarseHsIdx + afterSamples;
    if (refineEnd > s_sampleCounter) refineEnd = s_sampleCounter;
 
    // Refine to physical ground impact: maximum resultant acceleration peak
    float maxAccel = 0.0f;
    uint32_t refinedIdx = coarseHsIdx;
    for (uint32_t i = refineStart; i <= refineEnd; i++) {
        float resA = hist_resultant_accel(i);
        if (resA > maxAccel) {
            maxAccel = resA;
            refinedIdx = i;
        }
    }
 
    // Guard 3: wait until full shock window after refined HS has been received
    uint32_t shockAfter = seconds_to_samples(SHOCK_WINDOW_AFTER_S);
    if (refinedIdx + shockAfter > s_sampleCounter) return false;
 
    *outIdx = refinedIdx;
    return true;
}
 
// ===========================================================================
// SECTION 8 — Trajectory reconstruction (Rampp Sec. II-D3)
// Quaternion orientation integration + gravity removal + piecewise-linear
// de-drift + double trapezoidal integration, anchored at MS-to-MS segments.
// Simplified to a straight-line-segment implementation matched to the
// paper's equations; NOT an EKF (see header note).
// ===========================================================================
 
// Called once per completed MS-to-MS segment. samples[] must span exactly
// one segment (index 0 = this MS, index count-1 = next MS).
static float reconstruct_stride_length(const ImuSample* samples, uint32_t count) {
    if (count < 4) return 0.0f;
 
    // Map physical sensor axes to Rampp's computational frame.
    // Rampp assumes: Z = Vertical, X = AP, Y = ML.
    // By re-mapping here, we preserve the math of Eq 4-25 exactly without
    // needing to rewrite the quaternion transformations for our sensor.
    auto get_axis = [](const ImuSample& s, int axisId, bool isGyro) -> float {
        if (isGyro) return (axisId == 0) ? s.gyroX : (axisId == 1) ? s.gyroY : s.gyroZ;
        return (axisId == 0) ? s.accelX : (axisId == 1) ? s.accelY : s.accelZ;
    };
 
    #define R_X(s) get_axis(s, ACCEL_AP_AXIS, false)
    #define R_Y(s) get_axis(s, GYRO_SAGITTAL_AXIS, false)
    #define R_Z(s) get_axis(s, ACCEL_VERTICAL_AXIS, false)
    
    #define R_GX(s) get_axis(s, ACCEL_AP_AXIS, true)
    #define R_GY(s) get_axis(s, GYRO_SAGITTAL_AXIS, true)
    #define R_GZ(s) get_axis(s, ACCEL_VERTICAL_AXIS, true)
 
    // Orientation at segment start: accelerometer-as-tilt-sensor (Eq. 4-8)
    float ax = R_X(samples[0]), ay = R_Y(samples[0]), az = R_Z(samples[0]);
    float phi = atan2f(-ay, -az);                       // Eq. 7
    float psi = atan2f(-ax, sqrtf(ay*ay + az*az));       // Eq. 8
    Quaternion qx{cosf(phi/2), sinf(phi/2), 0, 0};
    Quaternion qz{cosf(psi/2), 0, 0, sinf(psi/2)};
    Quaternion q = quat_multiply(qx, qz);                 // Eq. 4
 
    // Integrate orientation across the segment via gyro (Eq. 9-11),
    // removing gravity at each step (Eq. 12-15) to build a gravity-
    // corrected acceleration trace for later de-drift + integration.
    static float aE[MAX_STRIDE_BUFFER_SAMPLES][3]; // gravity-corrected accel
    float dt = 1.0f / static_cast<float>(s_sampleRateHz);
 
    for (uint32_t j = 0; j < count; j++) {

        float gxr = R_GX(samples[j]) * (M_PI / 180.0f);
        float gyr = R_GY(samples[j]) * (M_PI / 180.0f);
        float gzr = R_GZ(samples[j]) * (M_PI / 180.0f);
 
        Quaternion rate{0, gxr, gyr, gzr};
        Quaternion qDot = quat_multiply(q, rate);
        Quaternion qNext;
        qNext.w = q.w + 0.5f * dt * qDot.w;
        qNext.x = q.x + 0.5f * dt * qDot.x;
        qNext.y = q.y + 0.5f * dt * qDot.y;
        qNext.z = q.z + 0.5f * dt * qDot.z;
        q = quat_normalize(qNext);
 
        Quaternion a_s{0, R_X(samples[j]), R_Y(samples[j]), R_Z(samples[j])};
        Quaternion qConj{q.w, -q.x, -q.y, -q.z};
        Quaternion a_e = quat_multiply(quat_multiply(q, a_s), qConj);
 
        aE[j][0] = a_e.x * 9.81f;
        aE[j][1] = a_e.y * 9.81f;
        aE[j][2] = (a_e.z + 1.0f) * 9.81f;
    }
 
    // Piecewise-linear drift model (Eq. 16-18): plateau = mean of first
    // DRIFT_PLATEAU_START_FRAC and last DRIFT_PLATEAU_END_FRAC samples.
    uint32_t kn = static_cast<uint32_t>(DRIFT_PLATEAU_START_FRAC * count);
    uint32_t ln = static_cast<uint32_t>(DRIFT_PLATEAU_END_FRAC * count);
    if (kn < 1) kn = 1;
    if (ln < 1) ln = 1;
 
    float y0[3] = {0,0,0}, y1[3] = {0,0,0};
    for (uint32_t j = 0; j < kn; j++)
        for (int a = 0; a < 3; a++) y0[a] += aE[j][a];
    for (int a = 0; a < 3; a++) y0[a] /= static_cast<float>(kn);
 
    for (uint32_t j = count - ln; j < count; j++)
        for (int a = 0; a < 3; a++) y1[a] += aE[j][a];
    for (int a = 0; a < 3; a++) y1[a] /= static_cast<float>(ln);
 
    // De-drift, then trapezoidal-integrate to velocity (Eq. 19-20)
    static float vel[MAX_STRIDE_BUFFER_SAMPLES][3];
    vel[0][0] = vel[0][1] = vel[0][2] = 0.0f;
    float previousDrift[3] = {y0[0], y0[1], y0[2]};
    for (uint32_t j = 1; j < count; j++) {

        for (int a = 0; a < 3; a++) {
            const uint32_t plateauEnd = count - ln;
            float drift;
            if (j < kn) {
                drift = y0[a];
            } else if (j >= plateauEnd) {
                drift = y1[a];
            } else {
                const float fraction = static_cast<float>(j - kn) /
                    static_cast<float>(plateauEnd - kn);
                drift = y0[a] + (y1[a] - y0[a]) * fraction;
            }
            const float deDrifted = aE[j][a] - drift;
            const float previousDeDrifted = aE[j - 1][a] - previousDrift[a];
            vel[j][a] = vel[j-1][a] +
                0.5f * (deDrifted + previousDeDrifted) * dt;
            previousDrift[a] = drift;
        }
    }
 
    // Zero-velocity-assumption drift correction on velocity itself
    // (Eq. 21-23): straight line from origin to mean of the last 5 samples.
    float v1[3] = {0,0,0};
    uint32_t tailN = (count >= 5) ? 5 : count;
    for (uint32_t j = count - tailN; j < count; j++)
        for (int a = 0; a < 3; a++) v1[a] += vel[j][a];
    for (int a = 0; a < 3; a++) v1[a] /= static_cast<float>(tailN);
 
    static float pos[MAX_STRIDE_BUFFER_SAMPLES][3];
    pos[0][0] = pos[0][1] = pos[0][2] = 0.0f;
    for (uint32_t j = 1; j < count; j++) {

        for (int a = 0; a < 3; a++) {
            float frac = static_cast<float>(j) / static_cast<float>(count);
            float vCorrected = vel[j][a] - v1[a]*frac;
            float vPrevCorrected = vel[j-1][a] - v1[a]*(static_cast<float>(j-1)/count);
            pos[j][a] = pos[j-1][a] + 0.5f*(vCorrected + vPrevCorrected)*dt;
        }
    }
 
    // Stride length = 2D norm of horizontal displacement (Eq. 25)
    // After axis remap: pos[0] = AP (forward), pos[1] = ML (lateral), pos[2] = Vertical
    float px = pos[count-1][0];  // anterior-posterior displacement
    float py = pos[count-1][1];  // medio-lateral displacement
    float rawLength = sqrtf(px*px + py*py);
 
    // Do not apply a generic shank lever-arm multiplier: the study reports
    // the trajectory displacement directly and does not establish such a
    // correction. Calibrate a placement-specific model against ground truth.
    return rawLength;
}
 
// ===========================================================================
// SECTION 9 — Public per-sample entry point
// ===========================================================================
 
// Mid-stance sliding-window energy computation. Called on every sample
// during the stance phase (after HS, before next MSw). Tracks the window
// with minimum combined gyro energy — that window center is MS.
static void ms_accumulate_sample(const ImuSample& sample, uint32_t idx) {
    float energy = sample.gyroX * sample.gyroX
                 + sample.gyroY * sample.gyroY
                 + sample.gyroZ * sample.gyroZ;
    uint32_t windowSamples = seconds_to_samples(MS_WINDOW_S);
    if (windowSamples < 2) windowSamples = 2;
    if (windowSamples > 32) windowSamples = 32;
    uint32_t hopSamples = seconds_to_samples(MS_OVERLAP_S);
    if (hopSamples < 1) hopSamples = 1;
 
    s_msEnergyWindow[s_msEnergyWrite] = energy;
    s_msEnergyWrite = (s_msEnergyWrite + 1) % windowSamples;
    if (s_msEnergyCount < windowSamples) s_msEnergyCount++;
    s_msSamplesSinceEval++;
 
    if (s_msEnergyCount == windowSamples && s_msSamplesSinceEval >= hopSamples) {
        float totalEnergy = 0.0f;
        for (uint32_t i = 0; i < windowSamples; i++)
            totalEnergy += s_msEnergyWindow[i];
        const float avgEnergy = totalEnergy / static_cast<float>(windowSamples);
        if (avgEnergy < s_msMinEnergy) {
            s_msMinEnergy = avgEnergy;
            // MS = center of the window
            s_msBestIdx = idx - windowSamples / 2;
        }
        s_msSamplesSinceEval = 0;
    }
}
 
// Confirm MS and trigger stride length if we have a previous MS
static void confirm_mid_stance(uint32_t msIdx) {
    if (s_haveLastMs) {
        uint32_t count = msIdx - s_lastMsIdx + 1;
        uint32_t ageOldest = s_sampleCounter - s_lastMsIdx;
        uint32_t maxPlausibleSamples = seconds_to_samples(MAX_STRIDE_TIME_S);
 
        // Ensure the old MS is still in our rolling history and within bounds
        if (count >= 4 && count <= maxPlausibleSamples + 1 && count <= MAX_STRIDE_BUFFER_SAMPLES && ageOldest < s_histCount) {
            // Extract MS(i) to MS(i+1) sequence into linear buffer
            for (uint32_t i = 0; i < count; i++) {
                uint32_t absIdx = s_lastMsIdx + i;
                uint32_t age = s_sampleCounter - absIdx;
                uint32_t ringIdx = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - age - 1) % MAX_STRIDE_BUFFER_SAMPLES;
                s_trajScratch[i] = s_sampleHist[ringIdx];
            }
            s_latestStride.strideLengthM = reconstruct_stride_length(s_trajScratch, count);
        }
    }
 
    s_lastMsIdx = msIdx;
    s_haveLastMs = true;
    
    uint32_t msTs = 0;
    if (!ts_at_index(msIdx, &msTs)) msTs = s_lastHsTs; // fallback
    queue_event(EVENT_MID_STANCE, msIdx, msTs);
}
 
static void process_sample_internal(const ImuSample& sample) {
    uint32_t idx = s_sampleCounter;
 
    // Push into rolling history
    s_sampleHist[s_histWriteIdx] = sample; // Store full sample for MS-to-MS extraction

    s_gyroSagittalHist[s_histWriteIdx]  = ((GYRO_SAGITTAL_AXIS == 0) ? sample.gyroX
                                          : (GYRO_SAGITTAL_AXIS == 1) ? sample.gyroY
                                          : sample.gyroZ) * GYRO_SAGITTAL_SIGN;
    s_accelApHist[s_histWriteIdx]       = (ACCEL_AP_AXIS == 0) ? sample.accelX
                                          : (ACCEL_AP_AXIS == 1) ? sample.accelY
                                          : sample.accelZ;
    s_accelVerticalHist[s_histWriteIdx] = (ACCEL_VERTICAL_AXIS == 0) ? sample.accelX
                                          : (ACCEL_VERTICAL_AXIS == 1) ? sample.accelY
                                          : sample.accelZ;
    s_histWriteIdx = (s_histWriteIdx + 1) % MAX_STRIDE_BUFFER_SAMPLES;
    if (s_histCount < MAX_STRIDE_BUFFER_SAMPLES) s_histCount++;
 
    // Feed mid-stance sliding window if searching
    if (s_msSearchActive) {
        ms_accumulate_sample(sample, idx);
    }
 
    float gyroSagittal = s_gyroSagittalHist[(s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - 1) % MAX_STRIDE_BUFFER_SAMPLES];
 
    uint32_t msWIdx;
    if (detect_mid_swing(gyroSagittal, idx, &msWIdx)) {
        uint32_t eventTs = sample.timestamp_us;
        (void)ts_at_index(msWIdx, &eventTs);
        queue_event(EVENT_MID_SWING, msWIdx, eventTs);
 
        // MSw signals end of stance phase — confirm the MS candidate
        // found during the preceding stance phase.
        // Guard: s_msMinEnergy initializes to 1e9f; any completed window
        // evaluation sets it far below that. Use < 1e9f so that even a
        // partial window (short/fast-pace stride) still confirms MS.
        if (s_msSearchActive && s_msMinEnergy < 1e9f) {
            confirm_mid_stance(s_msBestIdx);
        } else if (s_msSearchActive && s_msEnergyCount >= 2) {
            // Fallback: window partially filled but evaluation threshold
            // not reached — use the best candidate seen so far.
            confirm_mid_stance(s_msBestIdx);
        }
        s_msSearchActive = false;  // stop searching until next HS
 
        // From this confirmed MSw, look for TO before it and (on the NEXT
        // call chain, once enough future samples exist) HS after it.
        uint32_t toIdx;
        uint32_t searchLowerBound = s_haveLastHs ? s_lastHsIdx : 0;
        s_haveLastTo = false;                       // never reuse a stale TO
        if (find_toe_off_before(msWIdx, searchLowerBound, &toIdx)) {
            s_lastToIdx = toIdx;
            s_haveLastTo = true;
            if (!ts_at_index(toIdx, &s_lastToTs)) s_lastToTs = sample.timestamp_us;
            queue_event(EVENT_TOE_OFF, toIdx, s_lastToTs);
        }
 
        s_lastMSwIdx = msWIdx;
        s_haveLastMSw = true;
        s_hsResolvedFlag = false; // new cycle — allow HS search to run again
        s_sampleCounter++;
        return;
    }
 
    // Attempt HS resolution once enough forward samples exist past the
    // last confirmed MSw (bounded by MAX_STRIDE_TIME_S/2, see function).
    if (s_haveLastMSw && !s_haveLastHs_ResolvedForThisMSw()) {
        uint32_t hsIdx;
        if (find_heel_strike_after(s_lastMSwIdx, &hsIdx)) {
            s_prevHsIdx = s_lastHsIdx;  s_prevHsTs = s_lastHsTs;
            s_havePrevHs = s_haveLastHs;
            s_lastHsIdx = hsIdx;
            if (!ts_at_index(hsIdx, &s_lastHsTs)) s_lastHsTs = sample.timestamp_us;
            s_haveLastHs = true;
            s_hsResolvedFlag = true;
            s_hsCount++;
 
            uint32_t eventTs = sample.timestamp_us;
            (void)ts_at_index(hsIdx, &eventTs);
            queue_event(EVENT_HEEL_STRIKE, hsIdx, eventTs);
 
            // Start MS search during the upcoming stance phase
            s_msSearchActive = true;
            s_msMinEnergy = 1e9f;
            s_msBestIdx = hsIdx;
            s_msEnergyWrite = 0;
            s_msEnergyCount = 0;
            s_msSamplesSinceEval = 0;
 
            // Tibial shock: peak resultant around the refined HS
            s_latestStride.tibialShockG = peak_resultant_around(hsIdx);
 
            // Temporal parameters (skip startup strides if configured)
            if (s_havePrevHs && s_hsCount > STARTUP_HS_TO_DISCARD) {
                float dtStride = elapsed_s(s_prevHsIdx, s_prevHsTs, s_lastHsIdx, s_lastHsTs);
                if (dtStride >= MIN_STRIDE_TIME_S && dtStride <= MAX_STRIDE_TIME_S) {
                    // Preserve strideLengthM — set by confirm_mid_stance() at
                    // the previous MSw. Do NOT zero it here; carry it forward
                    // so the caller always sees the most recent trajectory value.
                    float preservedLength = s_latestStride.strideLengthM;
                    s_latestStride.strideTimeS = dtStride;
                    s_latestStride.swingTimeS  = 0.0f;
                    s_latestStride.stanceTimeS = 0.0f;
                    s_latestStride.strideLengthM = preservedLength; // explicit keep
                    // TO must lie between the previous HS and this HS
                    bool toOk = s_haveLastTo && s_lastToIdx > s_prevHsIdx
                                             && s_lastToIdx < s_lastHsIdx;
                    if (toOk) {
                        float sw = elapsed_s(s_lastToIdx, s_lastToTs, s_lastHsIdx, s_lastHsTs);
                        if (sw > 0.0f && sw < dtStride) {
                            s_latestStride.swingTimeS  = sw;
                            s_latestStride.stanceTimeS = dtStride - sw;
                        }
                    }
                    s_newStrideFlag = true;   // fresh, validated stride
                }
            }
 
            s_sampleCounter++;
            return;
        }
    }
 
    s_sampleCounter++;
}
 
GaitEvent gait_dsp_process_sample(const ImuSample& raw) {
    ImuSample clean = raw;
    if (!sample_is_plausible(raw)) {
        s_rejectedSamples++;
        if (!s_haveLastValid) return GaitEvent{};   // nothing to substitute yet
        clean = s_lastValidSample;                  // hold last good value so
        clean.timestamp_us = raw.timestamp_us;      // sample indexing stays continuous
    } else {
        s_lastValidSample = raw;
        s_haveLastValid = true;
    }
    if (!s_haveTsFirst) { s_tsFirst = clean.timestamp_us; s_haveTsFirst = true; }
    s_tsLatest = clean.timestamp_us;
    process_sample_internal(clean);
    
    if (s_eventQueueHead < s_eventQueueTail) {
        return s_eventQueue[s_eventQueueHead++ % 8];
    }
    return GaitEvent{};
}
 
bool gait_dsp_take_new_stride() {
    bool f = s_newStrideFlag;
    s_newStrideFlag = false;
    return f;
}
uint32_t gait_dsp_rejected_sample_count() { return s_rejectedSamples; }
float gait_dsp_measured_rate_hz() {
    if (!s_haveTsFirst || s_sampleCounter < 2 || s_tsLatest == s_tsFirst) return 0.0f;
    return static_cast<float>(s_sampleCounter - 1) * 1e6f
           / static_cast<float>(static_cast<uint32_t>(s_tsLatest - s_tsFirst));
}
 
 
// ===========================================================================
// SECTION 10 — Feature calculations
// ===========================================================================
 
float gait_dsp_compute_tibial_shock(const ImuSample& hsWindowPeak) {
    float ax = hsWindowPeak.accelX, ay = hsWindowPeak.accelY, az = hsWindowPeak.accelZ;
    return sqrtf(ax*ax + ay*ay + az*az);
}
 
float gait_dsp_compute_asymmetry_index(float paramLeft, float paramRight) {
    // Standard Symmetry Index (Robinson et al. 1987, widely adopted):
    //   SI = |L - R| / (0.5 * (L + R)) × 100%
    // Returned as a fraction (0..1), not a percentage.
    //
    // BILATERAL USE: paramLeft and paramRight are stride times (or any
    // bilateral parameter) from LEFT and RIGHT legs respectively.
    // When called from a SINGLE device (consecutive-stride proxy), both
    // parameters are from the same leg on consecutive strides — this is
    // documented as an approximation, not a true bilateral measure.
    //
    // TRUE BILATERAL ASYMMETRY requires post-processing:
    //   - Both devices log stride CSVs with aligned session timestamps
    //   - Dashboard/cloud aligns strides by session_us and computes SI
    //     from matching L/R stride rows
    //   - This firmware correctly computes per-stride consecutive-stride
    //     proxy for single-device feedback purposes only
    float mean = 0.5f * (paramLeft + paramRight);
    if (fabsf(mean) < 1e-6f) return 0.0f;
    return fabsf(paramLeft - paramRight) / mean;
}
 
AccelCrossCheckEvent gait_dsp_accel_cross_check(const ImuSample& sample) {
    // Mo & Chow 2018 M-method (shank-mounted): IC = minimum before the
    // positive peak of vertical shank acceleration; TO = minimum within
    // the region of interest after. This is a placeholder single-sample
    // stub — a real implementation needs the same kind of rolling-window
    // peak/minimum search as Sections 5-7, not shown here to avoid
    // duplicating that machinery for what is explicitly a secondary,
    // non-critical-path validation signal (see header note).
    AccelCrossCheckEvent result;
    (void)sample; // TODO: implement rolling min/max search on
                  // s_accelVerticalHist once the primary pipeline above
                  // is validated against bench data — this is intentionally
                  // deferred, not forgotten.
    return result;
}
 
const StrideParameters& gait_dsp_get_latest_stride() {
    return s_latestStride;
}