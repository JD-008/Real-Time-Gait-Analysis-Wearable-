# gait_dsp patch — apply to the "previous" (working) gait_dsp.cpp

Fixes: (1) tibial shock = peak in a window around HS, (2) HS timing guards +
startup discard + stale-TO/stale-stride fixes, (3) invalid-sample rejection.
Everything else in your file stays as is. Each block says WHERE it goes.

---------------------------------------------------------------------------
## A. SECTION 1 — add these constants (after MAX_STRIDE_BUFFER_SAMPLES)
---------------------------------------------------------------------------
```cpp
// --- HS timing guards (tune from walking logs) -----------------------------
// HS search starts this long after MSw. Walking log showed HS 0.15-0.25 s
// after MSw, so 0.08 s leaves margin. If HS stops resolving or shifts late
// after this patch, lower it (0.05 s).
static constexpr float HS_MIN_DELAY_AFTER_MSW_S = 0.08f;

// --- Tibial shock window around the refined HS ------------------------------
static constexpr float SHOCK_WINDOW_BEFORE_S = 0.050f;
static constexpr float SHOCK_WINDOW_AFTER_S  = 0.100f;

// --- Startup: first stride interval after power-on is unreliable ------------
static constexpr uint32_t STARTUP_HS_TO_DISCARD = 1;

// --- Sample plausibility (see sample_is_plausible) --------------------------
static constexpr float GYRO_PLAUSIBLE_MAX_DPS  = 1000.0f; // ankle swing < ~700
static constexpr float ACCEL_PLAUSIBLE_MAX_G   = 16.0f;

// 0 = stride/swing/stance from sample INDEX / s_sampleRateHz (what worked in
// the Phase-1 walk). 1 = from timestamp_us. Check gait_dsp_measured_rate_hz()
// first: if it is far from IMU_SAMPLE_RATE_HZ, switch to 1.
#define USE_TIMESTAMP_TIMEBASE 0
```

---------------------------------------------------------------------------
## B. SECTION 2 — add module state (after s_latestStride)
---------------------------------------------------------------------------
```cpp
static uint32_t s_hsCount = 0;
static bool     s_newStrideFlag = false;
static uint32_t s_rejectedSamples = 0;
static ImuSample s_lastValidSample;
static bool     s_haveLastValid = false;
static uint32_t s_lastHsTs = 0, s_prevHsTs = 0, s_lastToTs = 0;
static uint32_t s_tsFirst = 0, s_tsLatest = 0;
static bool     s_haveTsFirst = false;
```
In `gait_dsp_init()` add:
```cpp
    s_hsCount = 0;  s_newStrideFlag = false;  s_rejectedSamples = 0;
    s_haveLastValid = false;  s_haveTsFirst = false;
    s_lastHsTs = s_prevHsTs = s_lastToTs = 0;
```

---------------------------------------------------------------------------
## C. New helpers — put after seconds_to_samples() (before SECTION 5)
---------------------------------------------------------------------------
```cpp
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

// Peak resultant acceleration (g) in [hs - 50 ms, hs + 100 ms].
static float peak_resultant_around(uint32_t hsIdx) {
    uint32_t b = seconds_to_samples(SHOCK_WINDOW_BEFORE_S);
    uint32_t a = seconds_to_samples(SHOCK_WINDOW_AFTER_S);
    uint32_t start = (hsIdx > b) ? (hsIdx - b) : 0;
    uint32_t end   = hsIdx + a;
    if (end > s_sampleCounter) end = s_sampleCounter;
    float peak = 0.0f;
    for (uint32_t i = start; i <= end; i++) {
        uint32_t age = s_sampleCounter - i;
        if (age >= s_histCount) continue;
        uint32_t ring = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - age - 1)
                        % MAX_STRIDE_BUFFER_SAMPLES;
        const ImuSample& s = s_sampleHist[ring];
        float m = sqrtf(s.accelX*s.accelX + s.accelY*s.accelY + s.accelZ*s.accelZ);
        if (m > peak) peak = m;
    }
    return peak;
}

// Rejects the glitch rows seen on the SD log: gyro -500.06/-1933.1, accel
// X=Y=0 exactly, Z=-3.02. Range checks alone cannot catch accel (-3.02 g is
// inside the +-4 g range), so the X==Y==0 signature is checked too.
// Ideally ALSO fix at the source: check return codes in bmi270_driver.cpp.
static bool sample_is_plausible(const ImuSample& s) {
    const float a[3] = {s.accelX, s.accelY, s.accelZ};
    const float g[3] = {s.gyroX,  s.gyroY,  s.gyroZ};
    for (int i = 0; i < 3; i++) {
        if (!std::isfinite(a[i]) || !std::isfinite(g[i])) return false;
        if (fabsf(a[i]) > ACCEL_PLAUSIBLE_MAX_G)           return false;
        if (fabsf(g[i]) >= GYRO_PLAUSIBLE_MAX_DPS)         return false;
    }
    if (s.accelX == 0.0f && s.accelY == 0.0f) return false;
    return true;
}
```

---------------------------------------------------------------------------
## D. SECTION 7 — REPLACE find_heel_strike_after() entirely
---------------------------------------------------------------------------
```cpp
static bool find_heel_strike_after(uint32_t msWIdx, uint32_t* outIdx) {
    if (s_histCount < 4) return false;

    // Guard 1: don't even look until HS_MIN_DELAY_AFTER_MSW_S has passed
    uint32_t minDelay    = seconds_to_samples(HS_MIN_DELAY_AFTER_MSW_S);
    uint32_t forwardLimit = seconds_to_samples(MAX_STRIDE_TIME_S * 0.5f);
    uint32_t searchStart = msWIdx + minDelay;
    if (s_sampleCounter <= searchStart + 2) return false;
    uint32_t searchEnd = msWIdx + forwardLimit;
    if (searchEnd > s_sampleCounter) searchEnd = s_sampleCounter;

    float steepestNeg = 0.0f; uint32_t steepestNegIdx = searchStart;
    float steepestPos = 0.0f; uint32_t steepestPosIdx = searchStart;
    bool foundNeg = false;

    for (uint32_t i = searchStart + 1; i < searchEnd; i++) {
        uint32_t off = s_sampleCounter - i;
        if (off + 1 >= s_histCount) continue;
        uint32_t idxCur  = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 1) % MAX_STRIDE_BUFFER_SAMPLES;
        uint32_t idxPrev = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 2) % MAX_STRIDE_BUFFER_SAMPLES;
        float slope = s_gyroSagittalHist[idxCur] - s_gyroSagittalHist[idxPrev];

        if (!foundNeg) {
            if (slope < steepestNeg) { steepestNeg = slope; steepestNegIdx = i; foundNeg = true; }
        } else {
            if (slope > steepestPos) { steepestPos = slope; steepestPosIdx = i; }
            if (i > steepestNegIdx && slope > 0.0f && steepestPosIdx == i) break;
        }
    }
    if (!foundNeg || steepestPosIdx <= steepestNegIdx) return false;

    uint32_t coarseHsIdx = steepestNegIdx;
    float minVal = 1e9f;
    for (uint32_t i = steepestNegIdx; i <= steepestPosIdx; i++) {
        uint32_t off = s_sampleCounter - i;
        if (off >= s_histCount) continue;
        uint32_t idx = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 1) % MAX_STRIDE_BUFFER_SAMPLES;
        if (s_gyroSagittalHist[idx] < minVal) { minVal = s_gyroSagittalHist[idx]; coarseHsIdx = i; }
    }

    uint32_t beforeSamples = seconds_to_samples(HS_REFINE_WINDOW_BEFORE_S);
    uint32_t afterSamples  = seconds_to_samples(HS_REFINE_WINDOW_AFTER_S);
    uint32_t refineStart = (coarseHsIdx > beforeSamples) ? (coarseHsIdx - beforeSamples) : 0;
    if (refineStart <= msWIdx) refineStart = msWIdx + 1;   // Guard 2: HS must be AFTER MSw
    uint32_t refineEnd   = coarseHsIdx + afterSamples;
    if (refineEnd > s_sampleCounter) refineEnd = s_sampleCounter;

    float minAccel = 1e9f; uint32_t refinedIdx = coarseHsIdx;
    for (uint32_t i = refineStart; i <= refineEnd; i++) {
        uint32_t off = s_sampleCounter - i;
        if (off >= s_histCount) continue;
        uint32_t idx = (s_histWriteIdx + MAX_STRIDE_BUFFER_SAMPLES - off - 1) % MAX_STRIDE_BUFFER_SAMPLES;
        if (s_accelApHist[idx] < minAccel) { minAccel = s_accelApHist[idx]; refinedIdx = i; }
    }

    // Guard 3: wait until the full shock window after HS has been received
    uint32_t shockAfter = seconds_to_samples(SHOCK_WINDOW_AFTER_S);
    if (refinedIdx + shockAfter > s_sampleCounter) return false;

    *outIdx = refinedIdx;
    return true;
}
```

---------------------------------------------------------------------------
## E. SECTION 9 — rename the public function, add a wrapper
---------------------------------------------------------------------------
Change the existing line
`GaitEvent gait_dsp_process_sample(const ImuSample& sample) {`
to
`static GaitEvent process_sample_internal(const ImuSample& sample) {`
and add this AFTER that function (before SECTION 10):
```cpp
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
    return process_sample_internal(clean);
}

bool gait_dsp_take_new_stride() {
    bool f = s_newStrideFlag;
    s_newStrideFlag = false;
    return f;
}
uint32_t gait_dsp_rejected_sample_count() { return s_rejectedSamples; }
float gait_dsp_measured_rate_hz() {
    if (!s_haveTsFirst || s_tsLatest == s_tsFirst) return 0.0f;
    return static_cast<float>(s_sampleCounter) * 1e6f
           / static_cast<float>(static_cast<uint32_t>(s_tsLatest - s_tsFirst));
}
```

---------------------------------------------------------------------------
## F. Inside process_sample_internal — MSw block: fix stale TO
---------------------------------------------------------------------------
Replace
```cpp
        uint32_t toIdx;
        uint32_t searchLowerBound = s_haveLastHs ? s_lastHsIdx : 0;
        if (find_toe_off_before(msWIdx, searchLowerBound, &toIdx)) {
            s_lastToIdx = toIdx;
            s_haveLastTo = true;
        }
```
with
```cpp
        uint32_t toIdx;
        uint32_t searchLowerBound = s_haveLastHs ? s_lastHsIdx : 0;
        s_haveLastTo = false;                       // never reuse a stale TO
        if (find_toe_off_before(msWIdx, searchLowerBound, &toIdx)) {
            s_lastToIdx = toIdx;
            s_haveLastTo = true;
            if (!ts_at_index(toIdx, &s_lastToTs)) s_lastToTs = sample.timestamp_us;
        }
```

---------------------------------------------------------------------------
## G. Inside process_sample_internal — REPLACE the HS-resolution block
(the whole `if (find_heel_strike_after(...)) { ... }` body)
---------------------------------------------------------------------------
```cpp
        if (find_heel_strike_after(s_lastMSwIdx, &hsIdx)) {
            s_prevHsIdx = s_lastHsIdx;  s_prevHsTs = s_lastHsTs;
            s_havePrevHs = s_haveLastHs;
            s_lastHsIdx = hsIdx;
            if (!ts_at_index(hsIdx, &s_lastHsTs)) s_lastHsTs = sample.timestamp_us;
            s_haveLastHs = true;
            s_hsResolvedFlag = true;
            s_hsCount++;

            event.type = EVENT_HEEL_STRIKE;
            event.sampleIndex = hsIdx;
            event.timestamp_us = sample.timestamp_us;

            // Start MS search during the upcoming stance phase
            s_msSearchActive = true;
            s_msMinEnergy = 1e9f;
            s_msBestIdx = hsIdx;
            s_msWindowAccum = 0;
            s_msWindowEnergy = 0.0f;
            s_msWindowStartIdx = idx;

            // Tibial shock: peak resultant around the refined HS
            s_latestStride.tibialShockG = peak_resultant_around(hsIdx);

            // Temporal parameters (skip the first, unreliable stride interval)
            if (s_havePrevHs && s_hsCount > STARTUP_HS_TO_DISCARD + 1) {
                float dtStride = elapsed_s(s_prevHsIdx, s_prevHsTs, s_lastHsIdx, s_lastHsTs);
                if (dtStride >= MIN_STRIDE_TIME_S && dtStride <= MAX_STRIDE_TIME_S) {
                    s_latestStride.strideTimeS = dtStride;
                    s_latestStride.swingTimeS  = 0.0f;
                    s_latestStride.stanceTimeS = 0.0f;
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
            return event;
        }
```

---------------------------------------------------------------------------
## H. gait_dsp.h — add declarations
---------------------------------------------------------------------------
```cpp
// True once per newly validated stride (then clears). Use this to decide
// when to print / run asymmetry / push BLE, instead of comparing values.
bool gait_dsp_take_new_stride();

// Diagnostics
uint32_t gait_dsp_rejected_sample_count();
float    gait_dsp_measured_rate_hz();   // compare against IMU_SAMPLE_RATE_HZ
```

---------------------------------------------------------------------------
## I. arc_sense_main.ino (DSPThread) — what to change
---------------------------------------------------------------------------
1. On EVENT_HEEL_STRIKE, do NOT compute shock from the current sample. Use
   `gait_dsp_get_latest_stride().tibialShockG`.
2. Only print the stride line, run the asymmetry check, set the haptic flag and
   update BLE/SD when `gait_dsp_take_new_stride()` returns true.
3. Print `gait_dsp_measured_rate_hz()` and `gait_dsp_rejected_sample_count()`
   every ~10 s while bench-testing.
4. In the SD logger: log the HS event/stride after the guard, and keep the
   logger init before the threads start.

---------------------------------------------------------------------------
## What to check after the walk
---------------------------------------------------------------------------
- First HS never precedes first MSw; first stride row is skipped.
- Shock stays >= ~1 g and varies smoothly (no 0.46/0.59 values).
- HS - MSw gaps still ~0.15-0.25 s. If they shift or HS stops resolving,
  lower HS_MIN_DELAY_AFTER_MSW_S.
- gait_dsp_measured_rate_hz() near 100. If it is ~83, set USE_TIMESTAMP_TIMEBASE 1.
- rejected-sample count stays near 0 when nothing is wrong. If it climbs, fix
  the root cause in the BMI270 driver / bus wiring.