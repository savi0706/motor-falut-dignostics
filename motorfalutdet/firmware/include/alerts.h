/*
 * ============================================================
 * alerts.h — LED / Buzzer Alert Module
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 *
 * Alert hierarchy (highest priority first):
 *   1. SAFETY TRIP  — blink all LEDs fast, buzzer continuous
 *   2. FAULT        — red LED + intermittent buzzer
 *   3. WARNING      — yellow LED
 *   4. WATCH        — yellow LED slow blink
 *   5. NORMAL       — green LED
 * ============================================================
 */

#pragma once
#ifndef ALERTS_H
#define ALERTS_H

#include <Arduino.h>
#include "config.h"

enum AlertLevel : uint8_t {
    ALERT_NONE    = 0,
    ALERT_WATCH   = 1,
    ALERT_WARNING = 2,
    ALERT_FAULT   = 3,
    ALERT_SAFETY  = 4   // hardware trip — independent of ML
};

class AlertManager {
public:
    AlertManager() : _level(ALERT_NONE), _blink_state(false), _last_blink_ms(0) {}

    void begin() {
        pinMode(LED_NORMAL,  OUTPUT);
        pinMode(LED_WARNING, OUTPUT);
        pinMode(LED_FAULT,   OUTPUT);
        pinMode(BUZZER_PIN,  OUTPUT);
        all_off();
    }

    void set_level(AlertLevel lvl) { _level = lvl; }

    // Call from loop() — handles non-blocking blink patterns
    void update() {
        uint32_t now = millis();
        switch (_level) {
            case ALERT_SAFETY:
                // Fast blink all LEDs + continuous buzzer
                if (now - _last_blink_ms >= 150) {
                    _blink_state = !_blink_state;
                    _last_blink_ms = now;
                    digitalWrite(LED_NORMAL,  _blink_state ? HIGH : LOW);
                    digitalWrite(LED_WARNING, _blink_state ? HIGH : LOW);
                    digitalWrite(LED_FAULT,   _blink_state ? HIGH : LOW);
                }
                digitalWrite(BUZZER_PIN, HIGH);
                break;

            case ALERT_FAULT:
                // Red LED solid + intermittent buzzer
                digitalWrite(LED_NORMAL,  LOW);
                digitalWrite(LED_WARNING, LOW);
                digitalWrite(LED_FAULT,   HIGH);
                if (now - _last_blink_ms >= 800) {
                    _blink_state = !_blink_state;
                    _last_blink_ms = now;
                }
                digitalWrite(BUZZER_PIN, _blink_state ? HIGH : LOW);
                break;

            case ALERT_WARNING:
                digitalWrite(LED_NORMAL,  LOW);
                digitalWrite(LED_WARNING, HIGH);
                digitalWrite(LED_FAULT,   LOW);
                digitalWrite(BUZZER_PIN,  LOW);
                break;

            case ALERT_WATCH:
                // Yellow slow blink
                if (now - _last_blink_ms >= 1000) {
                    _blink_state = !_blink_state;
                    _last_blink_ms = now;
                }
                digitalWrite(LED_NORMAL,  LOW);
                digitalWrite(LED_WARNING, _blink_state ? HIGH : LOW);
                digitalWrite(LED_FAULT,   LOW);
                digitalWrite(BUZZER_PIN,  LOW);
                break;

            default:  // ALERT_NONE / NORMAL
                digitalWrite(LED_NORMAL,  HIGH);
                digitalWrite(LED_WARNING, LOW);
                digitalWrite(LED_FAULT,   LOW);
                digitalWrite(BUZZER_PIN,  LOW);
                break;
        }
    }

    void all_off() {
        digitalWrite(LED_NORMAL,  LOW);
        digitalWrite(LED_WARNING, LOW);
        digitalWrite(LED_FAULT,   LOW);
        digitalWrite(BUZZER_PIN,  LOW);
    }

private:
    AlertLevel _level;
    bool       _blink_state;
    uint32_t   _last_blink_ms;
};

#endif // ALERTS_H
