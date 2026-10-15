/*
 * ============================================================
 * logger.h — Logging / CSV streaming
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 * ============================================================
 */
#pragma once
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "config.h"

// Macro wrappers that compile away at lower log levels
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  #define LOG_DEBUG(msg) Serial.println("[DBG] " + String(msg))
#else
  #define LOG_DEBUG(msg) do {} while(0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
  #define LOG_INFO(msg)  Serial.println("[INF] " + String(msg))
#else
  #define LOG_INFO(msg)  do {} while(0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
  #define LOG_WARN(msg)  Serial.println("[WRN] " + String(msg))
#else
  #define LOG_WARN(msg)  do {} while(0)
#endif

#define LOG_ERROR(msg) Serial.println("[ERR] " + String(msg))

// ==================== CSV DATA COLLECTION ====================
// Format compatible with pipeline/scripts/collect_data.py
// Header printed once in setup(); rows printed each inference cycle.
static bool _csv_header_printed = false;

inline void log_csv_header() {
    if (!LOG_CSV_ENABLED) return;
    if (_csv_header_printed) return;
    Serial.println(
        "ts_ms,vib_rms_m,vib_mean_m,vib_std_m,vib_rms_b,vib_mean_b,vib_std_b,"
        "vib_kurtosis_m,vib_skew_m,vib_crest_m,"
        "fft_peak_freq_hz,fft_peak_mag,"
        "cur_rms_mA,cur_mean_mA,cur_std_mA,"
        "temp_motor_C,temp_bearing_C,"
        "rpm_motor,rpm_base,"
        "as_state,ml_label,ml_conf,fault"
    );
    _csv_header_printed = true;
}

inline void log_csv_row(
    uint32_t ts,
    float vib_rms_m, float vib_mean_m, float vib_std_m,
    float vib_rms_b, float vib_mean_b, float vib_std_b,
    float kurt_m, float skew_m, float crest_m,
    float fft_freq, float fft_mag,
    float cur_rms, float cur_mean, float cur_std,
    float temp_motor, float temp_bearing,
    float rpm_motor, float rpm_base,
    int as_state, int ml_label, float ml_conf,
    const char* fault)
{
    if (!LOG_CSV_ENABLED) return;
    char buf[256];
    snprintf(buf, sizeof(buf),
        "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f,%.2f,%.4f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%d,%d,%.4f,%s",
        (unsigned long)ts,
        vib_rms_m, vib_mean_m, vib_std_m,
        vib_rms_b, vib_mean_b, vib_std_b,
        kurt_m, skew_m, crest_m,
        fft_freq, fft_mag,
        cur_rms, cur_mean, cur_std,
        temp_motor, temp_bearing,
        rpm_motor, rpm_base,
        as_state, ml_label, ml_conf, fault
    );
    Serial.println(buf);
}

#endif // LOGGER_H
