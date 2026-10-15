/*
 * ============================================================
 * adaptive_sensing.h — Event-Triggered Adaptive Sensing
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 *
 * States:
 *   AS_NORMAL  — low-rate sampling when system is healthy
 *   AS_WATCH   — medium-rate when any signal is elevated
 *   AS_ALERT   — high-rate during active fault detection
 *
 * Transitions use configurable thresholds + hysteresis to
 * prevent chattering. This satisfies requirement #7.
 * ============================================================
 */

#pragma once
#ifndef ADAPTIVE_SENSING_H
#define ADAPTIVE_SENSING_H

#include <Arduino.h>
#include "config.h"
#include "features.h"
#include "logger.h"

// ==================== STATES ====================
enum AdaptiveState : uint8_t {
    AS_NORMAL = 0,
    AS_WATCH  = 1,
    AS_ALERT  = 2
};

static const char* AS_NAMES[] = { "NORMAL", "WATCH", "ALERT" };

// ==================== CONTROLLER ====================
class AdaptiveSensingController {
public:
    AdaptiveSensingController()
        : _state(AS_NORMAL)
        , _calm_count(0)
        , _last_sample_ms(0)
    {}

    // Call once per extracted feature vector to potentially change state.
    // Returns the updated state.
    AdaptiveState update(const FeatureVector& fv) {
        AdaptiveState desired = _state;

        // --- Evaluate signals against thresholds ---
        float vib_std  = fv.data[2];    // vib_std_motor
        float cur_rms  = fv.data[10];   // cur_rms_mA / 1000 = A approximation
        float temp_m   = fv.data[13];   // temp_motor_C

        bool alert_condition =
            (vib_std  > AS_VIB_ALERT_THRESHOLD)     ||
            (cur_rms  > AS_CURRENT_ALERT * 1000.0f) ||  // mA → compare
            (temp_m   > AS_TEMP_ALERT && temp_m > 0.1f);

        bool watch_condition =
            (vib_std  > AS_VIB_WATCH_THRESHOLD)     ||
            (cur_rms  > AS_CURRENT_WATCH * 1000.0f) ||
            (temp_m   > AS_TEMP_WATCH && temp_m > 0.1f);

        if (alert_condition) {
            desired = AS_ALERT;
            _calm_count = 0;
        } else if (watch_condition) {
            desired = AS_WATCH;
            _calm_count = 0;
        } else {
            // Increment calm counter; only step down after enough consecutive calm readings
            _calm_count++;
        }

        // Apply hysteresis (downward transition only after AS_HYSTERESIS_COUNT calm windows)
        if (desired < _state) {
            if (_calm_count >= AS_HYSTERESIS_COUNT) {
                AdaptiveState prev = _state;
                _state = desired;
                LOG_INFO("AdaptiveSensing: " + String(AS_NAMES[prev]) +
                         " → " + String(AS_NAMES[_state]));
                _calm_count = 0;
            }
        } else if (desired > _state) {
            AdaptiveState prev = _state;
            _state = desired;
            LOG_INFO("AdaptiveSensing: " + String(AS_NAMES[prev]) +
                     " → " + String(AS_NAMES[_state]));
            _calm_count = 0;
        }

        return _state;
    }

    // Returns true when it's time to take the next window
    bool ready_for_next_window() {
        uint32_t now = millis();
        uint32_t interval = _interval_for_state(_state);
        if ((now - _last_sample_ms) >= interval) {
            _last_sample_ms = now;
            return true;
        }
        return false;
    }

    AdaptiveState state() const { return _state; }
    const char*   state_name() const { return AS_NAMES[_state]; }

    uint32_t interval_ms() const { return _interval_for_state(_state); }

private:
    AdaptiveState _state;
    uint8_t       _calm_count;
    uint32_t      _last_sample_ms;

    static uint32_t _interval_for_state(AdaptiveState s) {
        switch (s) {
            case AS_ALERT:  return AS_INTERVAL_ALERT_MS;
            case AS_WATCH:  return AS_INTERVAL_WATCH_MS;
            default:        return AS_INTERVAL_NORMAL_MS;
        }
    }
};

#endif // ADAPTIVE_SENSING_H
