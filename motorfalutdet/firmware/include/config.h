/*
 * ============================================================
 * config.h — System Configuration
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 * Target: ESP32-S3
 * ============================================================
 *
 * I2C Address Allocation (verified conflict-free):
 *   MPU6050 #0 (motor)   → 0x68 (AD0=GND)
 *   MPU6050 #1 (base)    → 0x69 (AD0=3V3)
 *   INA219               → 0x40 (A0=A1=GND)
 *   OLED SSD1306         → 0x3C
 *
 * 1-Wire Bus: DS18B20 #0 and #1 share one GPIO (ROM-addressed)
 *
 * TB6612FNG motor driver: STBY=HIGH to enable, PWM via LEDC
 *
 * Voltage / current protections are HARDWARE-independent of ML.
 */

#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// ==================== I2C PINS ====================
#define SDA_PIN         8    // ESP32-S3 native USB board default
#define SCL_PIN         9

// ==================== SENSOR PINS ====================
// Hall effect sensors (RPM) — using PCNT-capable GPIOs
#define HALL1_PIN       5    // Motor shaft hall sensor
#define HALL2_PIN       6    // Base/bearing hall sensor (optional)

// DS18B20 1-Wire (both sensors on one bus)
#define ONE_WIRE_BUS    7

// INA219 uses I2C — no extra pin needed

// ==================== OUTPUT PINS ====================
#define LED_NORMAL      15
#define LED_WARNING     16
#define LED_FAULT       17
#define BUZZER_PIN      18

// ==================== MOTOR DRIVER (TB6612FNG) ====================
#define MOTOR_AIN1      35
#define MOTOR_AIN2      36
#define MOTOR_PWMA      37
#define MOTOR_STBY      38

// ==================== DISPLAY ====================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_ADDR       0x3C

// ==================== SENSOR I2C ADDRESSES ====================
#define MPU6050_MOTOR_ADDR  0x68   // AD0 = GND
#define MPU6050_BASE_ADDR   0x69   // AD0 = VCC
#define INA219_ADDR         0x40   // A0=A1=GND

// ==================== SAMPLING ====================
#define SAMPLE_RATE_HZ          1000    // Target acquisition rate (Hz) per MPU6050
#define WINDOW_SIZE             512     // Samples per analysis window
#define WINDOW_OVERLAP          128     // Overlap for sliding window
#define FFT_SIZE                512     // Must equal WINDOW_SIZE
#define CURRENT_SAMPLES         256     // INA219 samples per window

// ==================== ADAPTIVE SENSING THRESHOLDS ====================
// Transitions: NORMAL → WATCH → ALERT
#define AS_VIB_WATCH_THRESHOLD  0.35f   // m/s² std-dev  → enter WATCH mode
#define AS_VIB_ALERT_THRESHOLD  0.80f   // m/s² std-dev  → enter ALERT mode
#define AS_CURRENT_WATCH        0.30f   // A RMS         → enter WATCH
#define AS_CURRENT_ALERT        0.60f   // A RMS         → enter ALERT
#define AS_TEMP_WATCH           55.0f   // °C            → enter WATCH
#define AS_TEMP_ALERT           70.0f   // °C            → enter ALERT

// Hysteresis: return to lower state only after consecutive calm readings
#define AS_HYSTERESIS_COUNT     5

// Sampling interval per adaptive state (ms between windows)
#define AS_INTERVAL_NORMAL_MS   2000
#define AS_INTERVAL_WATCH_MS    500
#define AS_INTERVAL_ALERT_MS    100

// ==================== FAULT CONFIRMATION ====================
// Require N consecutive detections before declaring a fault
#define FAULT_CONFIRM_COUNT     3

// ==================== MOTOR HEALTH THRESHOLDS ====================
#define HEALTH_WARN_LEVEL       75
#define HEALTH_CRIT_LEVEL       30

// ==================== RPM LIMITS ====================
#define RPM_MIN_VALID           50
#define RPM_MAX_VALID           5000
#define RPM_PULSES_PER_REV      1     // Adjust for your encoder/magnet count

// ==================== INA219 CONFIG ====================
// TB6612FNG max motor current: 1.2A continuous, 3.2A peak
// Use 0.1 Ω shunt for 3.2A full-scale → INA219 in 32V_2A mode is fine
#define INA219_SHUNT_OHMS       0.1f
#define CURRENT_OVERCURRENT_A   1.5f  // Hardware-independent trip point

// ==================== TEMPERATURE SAFETY ====================
// Independent hardware safety — NOT gated by ML
#define TEMP_MOTOR_SHUTDOWN_C   80.0f
#define TEMP_BEARING_SHUTDOWN_C 75.0f

// ==================== WIFI TELEMETRY ====================
// Credentials loaded from secrets.h (not committed)
// If secrets.h is absent, WiFi is disabled gracefully
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #define WIFI_SSID    ""
  #define WIFI_PASS    ""
  #define WIFI_ENABLED 0
#endif

#ifndef WIFI_ENABLED
  #define WIFI_ENABLED 1
#endif

#define TELEMETRY_PORT      80
#define TELEMETRY_PATH      "/"
#define TELEMETRY_INTERVAL_MS  1000

// ==================== LOGGING ====================
#define LOG_LEVEL_DEBUG     3
#define LOG_LEVEL_INFO      2
#define LOG_LEVEL_WARN      1
#define LOG_LEVEL_ERROR     0

#ifndef LOG_LEVEL
  #define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_CSV_ENABLED     1   // Stream CSV over Serial for data collection

// ==================== ML INFERENCE ====================
// Feature vector length must match trained model input
#define FEATURE_COUNT       16   // See features.h for vector layout

// Tensor arena sizing (bytes) — tune after measuring with MicroInterpreter
#define TFLITE_TENSOR_ARENA_SIZE  (48 * 1024)

// Classification labels (must match model output index order)
#define NUM_CLASSES         5
#define LABEL_HEALTHY       0
#define LABEL_IMBALANCE     1
#define LABEL_MISALIGNMENT  2
#define LABEL_OVERLOAD      3
#define LABEL_FRICTION      4

// Minimum confidence to accept a prediction (0..1)
#define CONFIDENCE_THRESHOLD  0.60f

#endif // CONFIG_H
