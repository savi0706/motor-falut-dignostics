/*
 * ============================================================
 * main.cpp — Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 * Target: ESP32-S3-DevKitC-1
 *
 * Architecture (all modules in include/):
 *   config.h         — pin map, thresholds, parameters
 *   logger.h         — debug/info/warn/error + CSV streaming
 *   sensors.h        — 2×MPU6050, INA219, 2×DS18B20, 2×Hall
 *   features.h       — DSP: RMS, std, kurtosis, FFT peak
 *   adaptive_sensing.h — event-triggered state machine
 *   ml_inference.h   — TFLite Micro engine + confirmation
 *   health_model.h   — health score + RUL
 *   alerts.h         — LED + buzzer alert manager
 *   display_oled.h   — OLED 128×64
 *   telemetry.h      — Wi-Fi JSON dashboard
 *
 * Detected classes: Healthy | Imbalance | Misalignment | Overload | Friction
 *
 * Safety: hardware protection trips are ML-independent (see sensors.h)
 * Credentials: loaded from secrets.h (not committed, see template)
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "config.h"
#include "logger.h"
#include "sensors.h"
#include "features.h"
#include "adaptive_sensing.h"
#include "ml_inference.h"
#include "health_model.h"
#include "alerts.h"
#include "display_oled.h"
#include "telemetry.h"

// ==================== GLOBAL OBJECTS ====================
Adafruit_SSD1306        oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
AdaptiveSensingController as_ctrl;
MLEngine                ml_engine;
FaultClassifier         fault_classifier;
MotorHealthModel        health_model;
AlertManager            alert_mgr;

// ==================== STATE VARIABLES ====================
static InferenceResult  _last_result     = {};
static int              _confirmed_label = LABEL_HEALTHY;
static int              _health_pct      = 100;
static bool             _safety_tripped  = false;
static uint32_t         _telemetry_timer = 0;

// ==================== MOTOR DRIVER CONTROL ====================
static void motor_driver_init() {
    pinMode(MOTOR_AIN1, OUTPUT);
    pinMode(MOTOR_AIN2, OUTPUT);
    pinMode(MOTOR_PWMA, OUTPUT);
    pinMode(MOTOR_STBY, OUTPUT);

    // Enable TB6612FNG
    digitalWrite(MOTOR_STBY, HIGH);
    // Set direction forward
    digitalWrite(MOTOR_AIN1, HIGH);
    digitalWrite(MOTOR_AIN2, LOW);
    // PWM via analogWrite (LEDC auto-configured on ESP32)
    analogWrite(MOTOR_PWMA, 0);   // Start at 0 — user sets speed
}

static void motor_emergency_stop() {
    digitalWrite(MOTOR_STBY, LOW);  // Disable TB6612FNG
    analogWrite(MOTOR_PWMA, 0);
    LOG_ERROR("MOTOR EMERGENCY STOP — hardware safety trip");
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(500);

    LOG_INFO("=== Motor Fault Diagnosis TinyML v1.0 ===");
    LOG_INFO("Build date: " __DATE__ " " __TIME__);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);   // 400 kHz

    // OLED (non-fatal if absent)
    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        LOG_WARN("OLED init failed — continuing without display");
    } else {
        oled_startup();
    }

    // Alerts
    alert_mgr.begin();

    // Motor driver
    motor_driver_init();

    // Sensors (MPU6050 motor IMU is mandatory)
    bool sensors_ok = sensors_init();
    if (!sensors_ok) {
        LOG_ERROR("Critical sensor init failed — system may degrade");
        alert_mgr.set_level(ALERT_WARNING);
    }

    // ML engine
    if (!ml_engine.init()) {
        LOG_WARN("ML engine init failed — running in feature-only mode");
    }

    // Wi-Fi telemetry (graceful if no secrets.h)
    telemetry_init();

    // CSV header for data collection
    log_csv_header();

    LOG_INFO("Setup complete. Starting acquisition loop.");
}

// ==================== MAIN LOOP ====================
void loop() {
    // ===== ADAPTIVE SENSING: gating =====
    if (!as_ctrl.ready_for_next_window()) {
        alert_mgr.update();
        telemetry_handle_client();
        yield();
        return;
    }

    // ===== ACQUIRE SENSOR WINDOW =====
    SensorWindow win = sensors_acquire_window();

    // ===== HARDWARE SAFETY TRIP (ML-INDEPENDENT) =====
    if (sensors_safety_trip(win)) {
        if (!_safety_tripped) {
            _safety_tripped = true;
            motor_emergency_stop();
            oled_safety_trip("Over-I or Over-T");
            LOG_ERROR("Hardware safety trip triggered");
        }
        alert_mgr.set_level(ALERT_SAFETY);
        alert_mgr.update();
        telemetry_handle_client();
        return;
    }
    _safety_tripped = false;

    // ===== FEATURE EXTRACTION =====
    FeatureVector fv = features_extract(win);
    if (!fv.valid) {
        LOG_WARN("Invalid feature vector — skipping inference");
        return;
    }

    // ===== ADAPTIVE SENSING UPDATE =====
    AdaptiveState as_state = as_ctrl.update(fv);

    // ===== ML INFERENCE =====
    if (ml_engine.is_ready()) {
        _last_result = ml_engine.run(fv);
    }
    _confirmed_label = fault_classifier.update(_last_result);

    // ===== MOTOR HEALTH =====
    _health_pct = health_model.compute(fv, _confirmed_label);
    int rul = health_model.estimate_rul(_health_pct);

    // ===== DETERMINE ALERT LEVEL =====
    {
        AlertLevel lvl = ALERT_NONE;
        if (_confirmed_label != LABEL_HEALTHY)
            lvl = ALERT_FAULT;
        else if (_health_pct < HEALTH_CRIT_LEVEL)
            lvl = ALERT_WARNING;
        else if (_health_pct < HEALTH_WARN_LEVEL)
            lvl = ALERT_WARNING;
        else if (as_state == AS_WATCH)
            lvl = ALERT_WATCH;
        alert_mgr.set_level(lvl);
    }
    alert_mgr.update();

    // ===== OLED DISPLAY =====
    const char* fault_name = fault_classifier.confirmed_name();
    if (win.rpm_motor < RPM_MIN_VALID) {
        oled_stopped();
    } else if (_confirmed_label != LABEL_HEALTHY) {
        oled_fault_alert(fault_name);
    } else {
        oled_display_main(
            fv.data[0], fv.data[10],       // vib_rms, cur_rms_mA
            win.rpm_motor, win.temp_motor_C,
            _health_pct, fault_name,
            as_ctrl.state_name(),
            _last_result.confidence);
    }

    // ===== TELEMETRY =====
    if (millis() - _telemetry_timer >= TELEMETRY_INTERVAL_MS) {
        _telemetry_timer = millis();
        TelemetryPayload payload = {
            .ts_ms        = win.timestamp_ms,
            .vib_rms      = fv.data[0],
            .cur_rms_mA   = fv.data[10],
            .rpm          = win.rpm_motor,
            .temp_motor   = win.temp_motor_C,
            .temp_bearing = win.temp_bearing_C,
            .health_pct   = _health_pct,
            .fault        = fault_name,
            .confidence   = _last_result.confidence,
            .as_state     = (int)as_state,
            .fft_peak_freq = fv.data[8],
            .fft_peak_mag  = fv.data[9],
            .safety_trip   = _safety_tripped
        };
        telemetry_push(payload);
    }
    telemetry_handle_client();

    // ===== CSV LOGGING (data collection) =====
    log_csv_row(
        win.timestamp_ms,
        fv.data[0], fv.data[1], fv.data[2],   // vib motor: rms, mean, std
        fv.data[6], 0.0f,       fv.data[7],   // vib base:  rms, mean(N/A), std
        fv.data[3], fv.data[4], fv.data[5],   // kurtosis, skewness, crest
        fv.data[8], fv.data[9],               // fft_freq, fft_mag
        fv.data[10], fv.data[11], fv.data[12],// cur rms, mean, std
        win.temp_motor_C, win.temp_bearing_C,
        win.rpm_motor,   win.rpm_base,
        (int)as_state,
        _confirmed_label, _last_result.confidence,
        fault_name
    );

    // Maintenance advisory log
    LOG_DEBUG("Health: " + String(_health_pct) + "% RUL: " + String(rul) +
              " " + String(health_model.maintenance_advice(rul)));
}
