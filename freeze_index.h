#pragma once
/*
 * Freeze Index (Freezing-of-Gait) computation — Bächlin et al. method.
 * Band-power ratio between ~3-8Hz ("freeze band") and ~0.5-3Hz
 * ("locomotor band") on vertical-axis acceleration, via sliding-window
 * FFT/PSD.
 *
 * Implementation should use CMSIS-DSP (arm_math.h), e.g. arm_rfft_fast_f32
 * on a Hann-windowed frame, then bin-sum magnitude^2 over each band —
 * this runs on the Cortex-M4F's hardware FPU and is well within budget
 * at the window sizes used here (256 samples @ 200Hz = 1.28s window).
 *
 * Scope note: per project doc, this is validated only on simulated
 * freeze-like episodes (shuffling, start-hesitation) in healthy
 * volunteers for Phase 1 — not clinically validated on a Parkinson's
 * population. Framed as algorithmic feasibility only.
 */

#include <cstdint>

void freeze_index_init(uint32_t windowSamples, uint32_t sampleRateHz);

// Computes the Freeze Index for one window of vertical-axis accel samples.
// Returns the band-power ratio (freeze band / locomotor band).
// A value > ~2.0 indicates a likely Freezing-of-Gait episode (Bächlin 2010).
float freeze_index_compute(const float* window, uint32_t windowSamples);

// Convenience threshold check. Returns true if fi exceeds the published
// FoG detection threshold (2.0 — tunable in freeze_index.cpp).
// NOTE: not yet wired to the haptic trigger; Phase 3 TODO.
bool freeze_index_is_fog(float fi);
