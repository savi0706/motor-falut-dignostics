/*
 * ============================================================
 * display_oled.h — OLED Display Module
 * Adaptive Multi-Sensor TinyML Motor Fault Diagnosis
 * ============================================================
 */

#pragma once
#ifndef DISPLAY_OLED_H
#define DISPLAY_OLED_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

extern Adafruit_SSD1306 oled;

// ==================== STARTUP SCREEN ====================
inline void oled_startup() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(14, 4);  oled.print("Motor Fault Diag");
    oled.setCursor(26, 16); oled.print("TinyML v1.0");
    oled.setCursor(28, 28); oled.print("ESP32-S3");
    oled.setCursor(22, 42); oled.print("Initialising...");
    oled.display();
}

// ==================== HEALTH BAR ====================
inline void oled_health_bar(int health, int x, int y, int w, int h) {
    int fill = map(constrain(health, 0, 100), 0, 100, 0, w);
    oled.drawRect(x, y, w, h, SSD1306_WHITE);
    oled.fillRect(x, y, fill, h, SSD1306_WHITE);
}

// ==================== MAIN DATA DISPLAY ====================
inline void oled_display_main(
    float vib_rms, float cur_rms_mA,
    float rpm, float temp_c,
    int health_pct,
    const char* fault,
    const char* as_state,
    float confidence)
{
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);

    // Line 0: header
    oled.setCursor(0, 0);
    oled.print("Motor Diag [");
    oled.print(as_state[0]);   // first char of AS state
    oled.print("]");

    // Line 1: vibration + RPM
    oled.setCursor(0, 10);
    oled.print("VIB:");  oled.print(vib_rms, 2);
    oled.setCursor(72, 10);
    oled.print("RPM:"); oled.print((int)rpm);

    // Line 2: current + temp
    oled.setCursor(0, 20);
    oled.print("CUR:");  oled.print(cur_rms_mA / 1000.0f, 2); oled.print("A");
    oled.setCursor(72, 20);
    oled.print("T:");    oled.print(temp_c, 1);

    // Line 3: health bar
    oled.setCursor(0, 30);
    oled.print("HLT:");
    oled_health_bar(health_pct, 28, 31, 60, 6);
    oled.setCursor(92, 30);
    oled.print(health_pct); oled.print("%");

    // Line 4: fault class + confidence
    oled.setCursor(0, 42);
    oled.print("FLT:");
    oled.print(fault);

    oled.setCursor(0, 54);
    oled.print("CONF:"); oled.print(confidence * 100.0f, 0); oled.print("%");

    oled.display();
}

// ==================== FAULT ALERT SCREEN ====================
inline void oled_fault_alert(const char* fault_name) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(4, 8);  oled.print("!! FAULT");
    oled.setTextSize(1);
    oled.setCursor(10, 36); oled.print(fault_name);
    oled.display();
}

// ==================== STOPPED SCREEN ====================
inline void oled_stopped() {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(2);
    oled.setCursor(8, 24); oled.print("STOPPED");
    oled.display();
}

// ==================== SAFETY TRIP SCREEN ====================
inline void oled_safety_trip(const char* reason) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);  oled.print("!! HW SAFETY TRIP");
    oled.setCursor(0, 14); oled.print(reason);
    oled.display();
}

#endif // DISPLAY_OLED_H
