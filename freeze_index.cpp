/*
 * freeze_index.cpp — Freeze Index (Freezing-of-Gait) computation.
 *
 * Algorithm: Bächlin et al. (2010, IEEE TNSRE)
 *   "Wearable Assistant for Parkinson's Disease Patients With
 *    Freezing of Gait"
 *
 * Implementation uses CMSIS-DSP arm_rfft_fast_f32 which is included
 * in the Mbed OS board core for the nRF52840 (Nano 33 BLE).
 * No extra library needed — just uncomment the arm_math.h include.
 *
 * Band definitions (Bächlin §III-A):
 *   Locomotor band: 0.5–3 Hz  (normal stride frequency range)
 *   Freeze   band:  3–8 Hz    (high-frequency tremor / shuffling)
 *   Freeze Index = freeze_power / locomotor_power
 *   FI > ~2.0 is the published threshold for a FoG episode
 *   (Bächlin used a threshold tuned per-subject; 2.0 is the population
 *    median — keep this as a tunable constant, not a magic number).
 *
 * PHASE 2 STATUS: CMSIS-DSP body implemented. FI threshold not yet
 * connected to the haptic trigger (see FreezeIndexThread in main.ino —
 * the threshold evaluation and flag-set is deferred until we have
 * validated FI values on bench data with a simulated FoG episode).
 */

#include "freeze_index.h"
#include <Arduino.h>
#include <cstring>
#include <cmath>

// CMSIS-DSP — available in the Mbed OS Nano core, no extra library needed.
// If the compiler reports arm_math.h not found, add to board core include
// path: <core_path>/cores/arduino/mbed/targets/TARGET_NORDIC/TARGET_NRF5x/
// TARGET_SDK_15_0/libraries/util/sdk_common.h and ensure arm_math.h is on
// the search path via the board package.
#include <arm_math.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static constexpr float FREEZE_THRESHOLD = 2.0f;  // FI > 2.0 → likely FoG
                                                   // (Bächlin population median)
                                                   // TODO: tune per-subject

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static uint32_t               s_windowSamples = 256;
static uint32_t               s_sampleRateHz  = 100;
static arm_rfft_fast_instance_f32 s_fftInstance;
static float                  s_hannWindow[256];   // pre-computed Hann coefficients
static bool                   s_initialized   = false;

// Scratch buffers (static to avoid stack allocation inside the loop)
static float s_windowedBuf[256];
static float s_fftOut[256];      // complex output: [re0, im0, re1, im1, ...]
static float s_psd[129];         // bins 0..N/2 inclusive for a 256-point RFFT

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
void freeze_index_init(uint32_t windowSamples, uint32_t sampleRateHz) {
    s_windowSamples = windowSamples;
    s_sampleRateHz  = sampleRateHz;

    if (windowSamples > 256) {
        Serial.println("[FreezeIndex] WARNING: windowSamples > 256, capping at 256");
        s_windowSamples = 256;
    }

    // Initialise CMSIS-DSP RFFT instance for the given window size.
    // arm_rfft_fast_init_f32 supports: 32, 64, 128, 256, 512, 1024, 2048, 4096
    arm_status status = arm_rfft_fast_init_f32(&s_fftInstance, s_windowSamples);
    if (status != ARM_MATH_SUCCESS) {
        Serial.print("[FreezeIndex] ERROR: arm_rfft_fast_init_f32 failed, status=");
        Serial.println((int)status);
        s_initialized = false;
        return;
    }

    // Pre-compute Hann window: w[n] = 0.5 * (1 - cos(2π*n / (N-1)))
    for (uint32_t i = 0; i < s_windowSamples; i++) {
        s_hannWindow[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (s_windowSamples - 1)));
    }

    s_initialized = true;
    Serial.print("[FreezeIndex] OK — window="); Serial.print(s_windowSamples);
    Serial.print(" samples @ ");               Serial.print(s_sampleRateHz);
    Serial.println(" Hz");
}

// ---------------------------------------------------------------------------
// Compute — returns the Freeze Index for one window
// ---------------------------------------------------------------------------
float freeze_index_compute(const float* window, uint32_t windowSamples) {
    if (!s_initialized) return 0.0f;

    uint32_t N = (windowSamples < s_windowSamples) ? windowSamples : s_windowSamples;

    // 1) Apply Hann window element-wise to reduce spectral leakage
    float mean = 0.0f;
    for (uint32_t i = 0; i < N; i++) mean += window[i];
    if (N > 0) mean /= static_cast<float>(N);
    for (uint32_t i = 0; i < N; i++) {
        s_windowedBuf[i] = (window[i] - mean) * s_hannWindow[i];
    }
    // Zero-pad if windowSamples < s_windowSamples (shouldn't happen in practice)
    for (uint32_t i = N; i < s_windowSamples; i++) {
        s_windowedBuf[i] = 0.0f;
    }

    // 2) Real FFT via CMSIS-DSP
    // arm_rfft_fast_f32 output: [re(0), re(N/2), re(1), im(1), re(2), im(2), ...]
    arm_rfft_fast_f32(&s_fftInstance, s_windowedBuf, s_fftOut, 0 /* forward */);

    // 3) Compute PSD (magnitude^2) for each bin.
    // Bin k → frequency f_k = k * Fs / N
    // Bin 0 (DC) and bin N/2 (Nyquist) are real-only in the packed format.
    s_psd[0] = s_fftOut[0] * s_fftOut[0];                     // DC
    uint32_t halfN = s_windowSamples / 2;
    for (uint32_t k = 1; k < halfN; k++) {
        float re = s_fftOut[2*k];
        float im = s_fftOut[2*k + 1];
        s_psd[k] = re*re + im*im;
    }
    s_psd[halfN] = s_fftOut[1] * s_fftOut[1];                 // Nyquist

    // 4) Sum PSD in bands.
    // Bin index for frequency f: k = round(f * N / Fs)
    // Locomotor band: 0.5–3 Hz
    // Freeze   band:  3–8 Hz
    float binWidth = (float)s_sampleRateHz / (float)s_windowSamples;
    uint32_t locoLo = (uint32_t)ceilf(0.5f / binWidth);      // first bin >= 0.5 Hz
    uint32_t locoHi = (uint32_t)(3.0f / binWidth);            // 3.0 Hz
    uint32_t frzLo  = locoHi + 1;                              // first bin above 3 Hz
    uint32_t frzHi  = (uint32_t)(8.0f / binWidth);            // 8.0 Hz

    // Guard against exceeding the available bins
    if (locoLo < 1)       locoLo = 1;       // skip DC
    if (locoHi > halfN)   locoHi = halfN;
    if (frzLo  > halfN)   frzLo  = halfN;
    if (frzHi  > halfN)   frzHi  = halfN;

    float locomotorPower = 0.0f;
    for (uint32_t k = locoLo; k <= locoHi; k++) locomotorPower += s_psd[k];

    float freezePower = 0.0f;
    for (uint32_t k = frzLo;  k <= frzHi;  k++) freezePower    += s_psd[k];

    // 5) Freeze Index = freeze_power / locomotor_power (Bächlin Eq. 1)
    if (locomotorPower < 1e-9f) return 0.0f;   // guard div-by-zero (silence/noise floor)
    return freezePower / locomotorPower;
}

// ---------------------------------------------------------------------------
// Threshold query — lets callers avoid duplicating the threshold constant
// ---------------------------------------------------------------------------
bool freeze_index_is_fog(float fi) {
    return fi > FREEZE_THRESHOLD;
}
