/*
 * ============================================================
 * features.h — DSP Feature Extraction
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 *
 * Computes a 16-element feature vector from a SensorWindow:
 *   [0]  vib_rms_motor      (m/s²)
 *   [1]  vib_mean_motor     (m/s²)
 *   [2]  vib_std_motor      (m/s²)
 *   [3]  vib_kurtosis_motor (dimensionless)
 *   [4]  vib_skewness_motor (dimensionless)
 *   [5]  vib_crest_motor    (peak / RMS)
 *   [6]  vib_rms_base       (m/s²)
 *   [7]  vib_std_base       (m/s²)
 *   [8]  fft_peak_freq_hz   (Hz)
 *   [9]  fft_peak_mag       (a.u.)
 *   [10] cur_rms_mA         (mA)
 *   [11] cur_mean_mA        (mA)
 *   [12] cur_std_mA         (mA)
 *   [13] temp_motor_C       (°C, or 0 if NaN)
 *   [14] rpm_motor          (RPM)
 *   [15] rpm_delta          (motor − base RPM, 0 if only one sensor)
 * ============================================================
 */

#pragma once
#ifndef FEATURES_H
#define FEATURES_H

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensors.h"

// ==================== FEATURE VECTOR ====================
struct FeatureVector {
    float data[FEATURE_COUNT];
    bool  valid;
};

// ==================== SIMPLE STATISTICS ====================
static void compute_stats(const float* buf, uint16_t n,
    float& mean, float& rms, float& std_dev,
    float& kurtosis, float& skewness, float& crest_factor)
{
    if (n == 0) {
        mean = rms = std_dev = kurtosis = skewness = crest_factor = 0.0f;
        return;
    }

    // Mean and RMS
    double sum = 0.0, sum_sq = 0.0;
    float  peak = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum    += buf[i];
        sum_sq += (double)buf[i] * buf[i];
        if (fabsf(buf[i]) > peak) peak = fabsf(buf[i]);
    }
    mean = (float)(sum / n);
    rms  = sqrtf((float)(sum_sq / n));
    std_dev = sqrtf(fabsf((float)(sum_sq / n) - mean * mean));

    // Higher moments
    double sum3 = 0.0, sum4 = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        double d = buf[i] - mean;
        double d2 = d * d;
        sum3 += d2 * d;
        sum4 += d2 * d2;
    }
    float mu3 = (float)(sum3 / n);
    float mu4 = (float)(sum4 / n);
    float var  = std_dev * std_dev;

    skewness     = (var > 1e-8f) ? mu3 / (std_dev * var) : 0.0f;
    kurtosis     = (var > 1e-8f) ? mu4 / (var * var) - 3.0f : 0.0f;
    crest_factor = (rms > 1e-6f) ? peak / rms : 1.0f;
}

// ==================== MAGNITUDE BUFFER ====================
// Compute vector magnitude array from XYZ channels
static void magnitude_buf(
    const float* ax, const float* ay, const float* az,
    uint16_t n, float* out)
{
    for (uint16_t i = 0; i < n; i++) {
        out[i] = sqrtf(ax[i]*ax[i] + ay[i]*ay[i] + az[i]*az[i]);
    }
}

// ==================== FFT (Goertzel-based peak finder) ====================
// Full FFT is expensive (≈2–3 ms for N=512 on S3 with ESP32-S3 FPU).
// We use a Goertzel bank at octave-spaced frequencies to find peak bin
// cheaply without allocating large buffers.
// Tested frequency range: 5 Hz to (SAMPLE_RATE_HZ/2)

static constexpr int _FFT_BINS = 24;
static const float _fft_freqs[_FFT_BINS] = {
    5.0f,   7.8f,  12.2f,  19.1f,  29.8f,  46.6f,
    50.0f,  72.8f, 100.0f, 113.8f, 150.0f, 177.8f,
    200.0f, 277.8f,300.0f, 433.6f, 400.0f, 500.0f,
    0.0f    // sentinel — ignored
};

static void goertzel_peak(
    const float* sig, uint16_t N, float fs,
    float& peak_freq, float& peak_mag)
{
    peak_freq = 0.0f;
    peak_mag  = 0.0f;

    for (int b = 0; b < _FFT_BINS - 1; b++) {
        float f = _fft_freqs[b];
        if (f <= 0.0f || f >= fs / 2.0f) continue;

        float k  = f * N / fs;
        float w  = 2.0f * M_PI * k / N;
        float coeff = 2.0f * cosf(w);

        double q0 = 0.0, q1 = 0.0, q2 = 0.0;
        for (uint16_t i = 0; i < N; i++) {
            q0 = coeff * q1 - q2 + sig[i];
            q2 = q1; q1 = q0;
        }
        float mag = sqrtf((float)(q1*q1 + q2*q2 - q1*q2*coeff));
        if (mag > peak_mag) {
            peak_mag  = mag;
            peak_freq = f;
        }
    }
    // Normalize by N
    peak_mag /= (float)N;
}

// ==================== EXTRACT FEATURES ====================
FeatureVector features_extract(const SensorWindow& win) {
    FeatureVector fv = {};
    fv.valid = false;

    if (!sensors_window_valid(win)) return fv;

    const uint16_t N = win.vib_samples;

    // --- Magnitude buffer (motor IMU) ---
    static float mag_m[WINDOW_SIZE];
    magnitude_buf(win.ax_motor, win.ay_motor, win.az_motor, N, mag_m);

    float rms_m, mean_m, std_m, kurt_m, skew_m, crest_m;
    compute_stats(mag_m, N, mean_m, rms_m, std_m, kurt_m, skew_m, crest_m);

    // --- Magnitude buffer (base IMU) ---
    static float mag_b[WINDOW_SIZE];
    magnitude_buf(win.ax_base, win.ay_base, win.az_base, N, mag_b);

    float rms_b, mean_b, std_b, kurt_b, skew_b, crest_b;
    compute_stats(mag_b, N, mean_b, rms_b, std_b, kurt_b, skew_b, crest_b);
    (void)kurt_b; (void)skew_b; (void)crest_b; // future use

    // --- FFT peak ---
    float fft_freq = 0.0f, fft_mag = 0.0f;
    // Use Z-axis of motor IMU for spectral analysis (dominant fault axis)
    goertzel_peak(win.az_motor, N, (float)SAMPLE_RATE_HZ, fft_freq, fft_mag);

    // --- Current stats ---
    float cur_rms = 0.0f, cur_mean = 0.0f, cur_std = 0.0f, dummy1, dummy2, dummy3;
    if (win.cur_samples > 0) {
        compute_stats(win.current_mA, win.cur_samples,
            cur_mean, cur_rms, cur_std, dummy1, dummy2, dummy3);
    }

    // --- Temperature (guard NaN) ---
    float temp_m = isnan(win.temp_motor_C)   ? 0.0f : win.temp_motor_C;
    float temp_b = isnan(win.temp_bearing_C) ? 0.0f : win.temp_bearing_C;
    (void)temp_b;  // stored in future feature expansion

    // --- Pack vector ---
    fv.data[ 0] = rms_m;
    fv.data[ 1] = mean_m;
    fv.data[ 2] = std_m;
    fv.data[ 3] = kurt_m;
    fv.data[ 4] = skew_m;
    fv.data[ 5] = crest_m;
    fv.data[ 6] = rms_b;
    fv.data[ 7] = std_b;
    fv.data[ 8] = fft_freq;
    fv.data[ 9] = fft_mag;
    fv.data[10] = cur_rms;
    fv.data[11] = cur_mean;
    fv.data[12] = cur_std;
    fv.data[13] = temp_m;
    fv.data[14] = win.rpm_motor;
    fv.data[15] = win.rpm_motor - win.rpm_base;

    fv.valid = true;
    return fv;
}

#endif // FEATURES_H
