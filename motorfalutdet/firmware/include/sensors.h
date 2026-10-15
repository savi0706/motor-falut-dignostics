/*
 * ============================================================
 * sensors.h — Multi-Sensor Acquisition Module
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 *
 * Sensors managed:
 *   2× MPU6050  (I2C, addr 0x68 / 0x69)
 *   1× INA219   (I2C, addr 0x40)
 *   2× DS18B20  (1-Wire, GPIO ONE_WIRE_BUS)
 *   2× Hall     (interrupt-driven, GPIO HALL1_PIN / HALL2_PIN)
 * ============================================================
 */

#pragma once
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <INA219_WE.h>
#include "config.h"
#include "logger.h"

// ==================== SENSOR OBJECTS ====================
static Adafruit_MPU6050 mpu_motor;   // 0x68 — on motor body
static Adafruit_MPU6050 mpu_base;    // 0x69 — on base/bearing

static OneWire            oneWire(ONE_WIRE_BUS);
static DallasTemperature  tempSensors(&oneWire);
static DeviceAddress       tempAddr[2];   // ROM addresses for 2× DS18B20

static INA219_WE ina219(INA219_ADDR);

// ==================== RPM STATE (interrupt-safe) ====================
static volatile uint32_t _hall1_count = 0;
static volatile uint32_t _hall2_count = 0;
static volatile uint32_t _hall1_last_us = 0;
static volatile uint32_t _hall2_last_us = 0;

static float _rpm1_filtered = 0.0f;
static float _rpm2_filtered = 0.0f;

// ==================== SENSOR STATUS FLAGS ====================
static bool mpu_motor_ok  = false;
static bool mpu_base_ok   = false;
static bool ina219_ok     = false;
static bool temp_count    = 0;   // number of DS18B20 found

// ==================== ISR ====================
void IRAM_ATTR hall1_isr() {
    uint32_t now = micros();
    if (now - _hall1_last_us > 5000) {   // 5 ms debounce
        _hall1_count++;
        _hall1_last_us = now;
    }
}

void IRAM_ATTR hall2_isr() {
    uint32_t now = micros();
    if (now - _hall2_last_us > 5000) {
        _hall2_count++;
        _hall2_last_us = now;
    }
}

// ==================== SENSOR RAW DATA STRUCT ====================
struct SensorWindow {
    // Vibration window buffers (WINDOW_SIZE samples each)
    float ax_motor[WINDOW_SIZE];   // m/s²
    float ay_motor[WINDOW_SIZE];
    float az_motor[WINDOW_SIZE];
    float ax_base[WINDOW_SIZE];
    float ay_base[WINDOW_SIZE];
    float az_base[WINDOW_SIZE];
    uint16_t vib_samples;          // actual samples collected

    // Current
    float current_mA[CURRENT_SAMPLES];
    uint16_t cur_samples;

    // Temperature (°C) — snapshot at end of window
    float temp_motor_C;
    float temp_bearing_C;

    // RPM — computed at end of window
    float rpm_motor;
    float rpm_base;

    // Window acquisition timestamp
    uint32_t timestamp_ms;
    bool valid;                    // false if critical sensor failed
};

// ==================== INITIALISE SENSORS ====================
bool sensors_init() {
    bool all_ok = true;

    // --- MPU6050 #0 (motor, 0x68) ---
    Wire.beginTransmission(MPU6050_MOTOR_ADDR);
    mpu_motor_ok = mpu_motor.begin(MPU6050_MOTOR_ADDR, &Wire);
    if (mpu_motor_ok) {
        mpu_motor.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu_motor.setFilterBandwidth(MPU6050_BAND_94_HZ);
        mpu_motor.setGyroRange(MPU6050_RANGE_500_DEG);
        LOG_INFO("MPU6050 motor OK @ 0x68");
    } else {
        LOG_ERROR("MPU6050 motor FAIL @ 0x68");
        all_ok = false;
    }

    // --- MPU6050 #1 (base, 0x69) ---
    mpu_base_ok = mpu_base.begin(MPU6050_BASE_ADDR, &Wire);
    if (mpu_base_ok) {
        mpu_base.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu_base.setFilterBandwidth(MPU6050_BAND_94_HZ);
        mpu_base.setGyroRange(MPU6050_RANGE_500_DEG);
        LOG_INFO("MPU6050 base  OK @ 0x69");
    } else {
        LOG_WARN("MPU6050 base  FAIL @ 0x69 — single-IMU fallback");
        // Non-fatal: fall back to motor IMU only
    }

    // --- INA219 ---
    ina219_ok = ina219.init();
    if (ina219_ok) {
        ina219.setADCMode(INA219_SAMPLE_MODE_128);   // 128 samples for averaging
        ina219.setMeasureMode(INA219_CONTINUOUS);
        LOG_INFO("INA219 OK @ 0x40");
    } else {
        LOG_ERROR("INA219 FAIL @ 0x40");
        all_ok = false;
    }

    // --- DS18B20 ---
    tempSensors.begin();
    int found = tempSensors.getDeviceCount();
    temp_count = (uint8_t)min(found, 2);
    for (int i = 0; i < temp_count; i++) {
        tempSensors.getAddress(tempAddr[i], i);
        tempSensors.setResolution(tempAddr[i], 11);   // 11-bit ≈ 375 ms
    }
    if (temp_count == 0) {
        LOG_WARN("DS18B20: no sensors found — temperature unavailable");
    } else {
        LOG_INFO("DS18B20: found " + String(temp_count) + " sensor(s)");
    }

    // --- Hall sensors ---
    pinMode(HALL1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL1_PIN), hall1_isr, RISING);
    pinMode(HALL2_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL2_PIN), hall2_isr, RISING);
    LOG_INFO("Hall sensors initialised");

    return all_ok;
}

// ==================== RPM MEASUREMENT ====================
// Call once per window; pass elapsed milliseconds since last call.
void rpm_update(uint32_t elapsed_ms) {
    // Atomically read and reset counters
    noInterrupts();
    uint32_t c1 = _hall1_count; _hall1_count = 0;
    uint32_t c2 = _hall2_count; _hall2_count = 0;
    interrupts();

    if (elapsed_ms == 0) return;

    float raw1 = (c1 * 60000.0f) / (elapsed_ms * RPM_PULSES_PER_REV);
    float raw2 = (c2 * 60000.0f) / (elapsed_ms * RPM_PULSES_PER_REV);

    // Low-pass filter α=0.3
    _rpm1_filtered = 0.7f * _rpm1_filtered + 0.3f * raw1;
    _rpm2_filtered = 0.7f * _rpm2_filtered + 0.3f * raw2;
}

float rpm_motor_get() { return _rpm1_filtered; }
float rpm_base_get()  { return _rpm2_filtered; }

// ==================== ACQUIRE ONE WINDOW ====================
SensorWindow sensors_acquire_window() {
    SensorWindow win = {};
    win.timestamp_ms = millis();
    win.valid = true;

    // ----- Vibration: interleave both IMUs -----
    uint16_t n = 0;
    uint32_t t_start = micros();
    uint32_t dt_us = 1000000UL / SAMPLE_RATE_HZ;

    for (uint16_t i = 0; i < WINDOW_SIZE; i++) {
        uint32_t t_target = t_start + (uint32_t)i * dt_us;

        // Wait for next sample time (busy-wait, needed for timing accuracy)
        while (micros() < t_target) { /* spin */ }

        sensors_event_t a_m, g_m, tmp;
        if (mpu_motor_ok && mpu_motor.getEvent(&a_m, &g_m, &tmp)) {
            win.ax_motor[i] = a_m.acceleration.x;
            win.ay_motor[i] = a_m.acceleration.y;
            win.az_motor[i] = a_m.acceleration.z;
        } else {
            // Sensor failure mid-window: mark invalid and break
            win.ax_motor[i] = win.ay_motor[i] = win.az_motor[i] = 0.0f;
            if (i > 10) { win.valid = false; }
        }

        if (mpu_base_ok) {
            sensors_event_t a_b, g_b, tmp2;
            mpu_base.getEvent(&a_b, &g_b, &tmp2);
            win.ax_base[i] = a_b.acceleration.x;
            win.ay_base[i] = a_b.acceleration.y;
            win.az_base[i] = a_b.acceleration.z;
        } else {
            // Degrade gracefully: copy motor data
            win.ax_base[i] = win.ax_motor[i];
            win.ay_base[i] = win.ay_motor[i];
            win.az_base[i] = win.az_motor[i];
        }
        n++;
    }
    win.vib_samples = n;

    // ----- Current: INA219 burst -----
    uint16_t cn = 0;
    if (ina219_ok) {
        for (uint16_t i = 0; i < CURRENT_SAMPLES; i++) {
            win.current_mA[cn++] = ina219.getCurrent_mA();
            delayMicroseconds(500);   // ~2 kHz
        }
    }
    win.cur_samples = cn;

    // ----- Temperature -----
    tempSensors.requestTemperatures();
    delay(200);   // 11-bit conversion time
    win.temp_motor_C   = (temp_count > 0) ? tempSensors.getTempC(tempAddr[0]) : NAN;
    win.temp_bearing_C = (temp_count > 1) ? tempSensors.getTempC(tempAddr[1]) : NAN;

    // ----- RPM -----
    uint32_t elapsed = millis() - win.timestamp_ms;
    rpm_update(elapsed > 0 ? elapsed : 1);
    win.rpm_motor = rpm_motor_get();
    win.rpm_base  = rpm_base_get();

    return win;
}

// ==================== VALIDATE WINDOW ====================
bool sensors_window_valid(const SensorWindow& w) {
    // Motor IMU is mandatory
    if (!mpu_motor_ok) return false;
    if (w.vib_samples < WINDOW_SIZE / 2) return false;
    // Temperature bounds
    if (!isnan(w.temp_motor_C) && (w.temp_motor_C < -10.0f || w.temp_motor_C > 150.0f))
        return false;
    return w.valid;
}

// ==================== SAFETY TRIP (HARDWARE-INDEPENDENT) ====================
// Returns true if a hardware trip condition is detected.
// This function MUST NOT be gated by ML inference results.
bool sensors_safety_trip(const SensorWindow& w) {
    // Overcurrent
    if (ina219_ok) {
        float cur_A = ina219.getCurrent_mA() / 1000.0f;
        if (cur_A > CURRENT_OVERCURRENT_A) return true;
    }
    // Over-temperature
    if (!isnan(w.temp_motor_C)   && w.temp_motor_C   > TEMP_MOTOR_SHUTDOWN_C)   return true;
    if (!isnan(w.temp_bearing_C) && w.temp_bearing_C > TEMP_BEARING_SHUTDOWN_C) return true;
    return false;
}

#endif // SENSORS_H
