/*
 * ============================================================
 * health_model.h — Motor Health Score & RUL Estimation
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 * ============================================================
 */

#pragma once
#ifndef HEALTH_MODEL_H
#define HEALTH_MODEL_H

#include <Arduino.h>
#include "config.h"
#include "features.h"

class MotorHealthModel {
public:
    MotorHealthModel() : _filtered_health(100.0f) {}

    // Compute a 0–100 health score from the feature vector
    // and the confirmed fault label.
    int compute(const FeatureVector& fv, int fault_label) {
        if (!fv.valid) return (int)_filtered_health;

        float vib_stress  = _stress_curve(fv.data[2], AS_VIB_WATCH_THRESHOLD,  AS_VIB_ALERT_THRESHOLD,  40.0f);
        float cur_stress  = _stress_curve(fv.data[10] / 1000.0f, AS_CURRENT_WATCH, AS_CURRENT_ALERT, 30.0f);
        float temp_stress = _stress_curve(fv.data[13], AS_TEMP_WATCH, AS_TEMP_ALERT, 20.0f);

        // Fault penalty
        float fault_penalty = 0.0f;
        switch (fault_label) {
            case LABEL_FRICTION:     fault_penalty = 5.0f;  break;
            case LABEL_IMBALANCE:    fault_penalty = 10.0f; break;
            case LABEL_MISALIGNMENT: fault_penalty = 15.0f; break;
            case LABEL_OVERLOAD:     fault_penalty = 20.0f; break;
            default: break;
        }

        float raw = 100.0f - (vib_stress + cur_stress + temp_stress + fault_penalty);
        raw = constrain(raw, 0.0f, 100.0f);

        // Low-pass filter to avoid sudden jumps
        _filtered_health = 0.3f * raw + 0.7f * _filtered_health;
        _filtered_health = constrain(_filtered_health, 0.0f, 100.0f);

        return (int)roundf(_filtered_health);
    }

    // Remaining Useful Life estimate (heuristic %)
    int estimate_rul(int health_pct) {
        return constrain((int)(health_pct * 1.2f), 0, 100);
    }

    const char* maintenance_advice(int rul) {
        if (rul > 80)  return "Healthy";
        if (rul > 50)  return "Service Soon";
        if (rul > 20)  return "Urgent Maintenance";
        return "STOP — Immediate Maintenance";
    }

private:
    float _filtered_health;

    // Smooth stress curve: 0 below low_thresh, rises linearly to max_stress at high_thresh
    static float _stress_curve(float val, float low_thresh, float high_thresh, float max_stress) {
        if (val <= low_thresh) return 0.0f;
        float fraction = (val - low_thresh) / (high_thresh - low_thresh);
        return constrain(fraction, 0.0f, 1.0f) * max_stress;
    }
};

#endif // HEALTH_MODEL_H
