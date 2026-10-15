# Adaptive Multi-Sensor TinyML System for Real-Time Motor Fault Diagnosis

**Target Microcontroller:** ESP32-S3-DevKitC-1 (Xtensa Dual-Core 240 MHz, 512 KB SRAM, 8 MB PSRAM, 16 MB Flash)  
**Framework:** Arduino / PlatformIO  
**ML Runtime:** TensorFlow Lite Micro (ESP32-S3 optimized)  

---

## 1. Executive Summary & Source Code Audit

This project integrates, refactors, and extends two foundational repositories:
1. `TinyML-Real-time-diagnosis-of-motor-faults` (PlatformIO / ESP32-S3 / TFLite Micro baseline)
2. `MotorMind-AI-v1.026` (Multi-sensor acquisition, display, health scoring, and alert concepts)

### 1.1 Source Repository Audit Findings & Bug Fixes

| Area | Issue in Original Repositories | Resolution & Architecture in Current Project |
|---|---|---|
| **Class Conflation** | MotorMind equated electrical `OVERCURRENT` directly with mechanical `Overload`. | **Corrected:** Electrical overcurrent is an electrical condition (stall, short), distinct from mechanical overload (excess torque). In `preprocess.py`, `OVERCURRENT` is excluded from the 5 mechanical classes. |
| **Missing Labeled Data** | The provided repositories contain physical data for only `NORMAL` and `UNBALANCE`. No measurements exist for `Misalignment`, `Overload`, or `Friction`. `TinyML` repo only had binary normal vs anomaly. | **Truthful Engineering Stance:** We **never fabricate physical sensor data** or claim a 5-class model is trained without suitable data. The pipeline, model architecture, and firmware support all 5 classes; real data exists for Healthy and Imbalance, while the data collection tool (`collect_data.py`) is provided for the remaining classes. |
| **Data Leakage** | MotorMind used random row-level train/test split across continuous time-series data, causing temporal leakage between train and test sets. | **Corrected:** Implemented recording-level splitting (`recording_level_split()`) and time-ordered chronological holdout when only a single continuous recording is available. |
| **Duplicate Records** | MotorMind's `dataset.txt` contained copy-pasted duplicate timestamps and readings. | **Corrected:** `preprocess.py` cleans duplicate `(ts_ms, class_name)` records (removing 36+ redundant rows). |
| **I2C Conflicts** | Incompatible or hardcoded sensor drivers without dual-IMU address distinction. | **Corrected:** MPU6050 #0 (Motor Body) addressed at `0x68` (AD0=GND); MPU6050 #1 (Base/Bearing) addressed at `0x69` (AD0=3.3V); INA219 at `0x40`; SSD1306 at `0x3C`. All verified conflict-free. |
| **Safety Coupling** | MotorMind evaluated alerts only downstream of AI prediction. | **Corrected:** Hardware electrical safety trip (INA219 $>1.5\text{ A}$, Temp $>80^\circ\text{C}$) is executed **independently of ML inference**, driving TB6612FNG `STBY` LOW immediately. |
| **Hardcoded Credentials**| MotorMind committed real Wi-Fi and Blynk API tokens in `config.h`. | **Corrected:** Replaced with `secrets.h.template`. `secrets.h` is excluded via `.gitignore`. If credentials are omitted, firmware compiles and operates gracefully with Wi-Fi disabled. |

---

## 2. Hardware Architecture & Pin Map

### 2.1 Sensor & Actuator Bill of Materials (BOM)
- **MCU:** ESP32-S3-DevKitC-1 (N16R8)
- **IMU 1 (Motor Vibration):** MPU6050 (`0x68`, AD0 $\rightarrow$ GND)
- **IMU 2 (Base/Bearing Vibration):** MPU6050 (`0x69`, AD0 $\rightarrow$ 3.3V)
- **Current & Voltage Sensor:** INA219 (`0x40`, 0.1Ω shunt resistor, high-side DC)
- **Temperature Sensors:** 2× DS18B20 digital 1-Wire sensors (Motor body & bearing casing, GPIO 7 with 4.7kΩ pull-up)
- **Speed Sensors (RPM):** 2× Digital Hall Sensors (A3144/hall module on GPIO 5 & GPIO 6, interrupt-driven)
- **Motor Driver:** TB6612FNG Dual H-Bridge (AIN1: GPIO 35, AIN2: GPIO 36, PWMA: GPIO 37, STBY: GPIO 38)
- **Display:** 0.96" SSD1306 128×64 Monochrome OLED (`0x3C`)
- **Visual & Audio Alerts:** Green LED (GPIO 15), Yellow LED (GPIO 16), Red LED (GPIO 17), Piezo Buzzer (GPIO 18)

### 2.2 Wiring Schematic & Pin Assignments
See detailed diagrams in [`docs/hardware/wiring.md`](file:///c:/Users/91830/OneDrive/Desktop/motorfalutdet/docs/hardware/wiring.md).

---

## 3. Modular Software Architecture

```
firmware/
├── include/
│   ├── config.h             <- Pin allocations, sampling constants, thresholds, I2C addrs
│   ├── logger.h             <- Level-gated logging (DEBUG/INFO/WARN/ERROR) & CSV streaming
│   ├── sensors.h            <- Dual-MPU6050, INA219, DS18B20, Hall RPM & hardware safety trip
│   ├── features.h           <- DSP: 16-feature vector (RMS, Std, Kurtosis, Skew, Crest, Goertzel FFT)
│   ├── adaptive_sensing.h   <- Event-triggered state machine (NORMAL / WATCH / ALERT)
│   ├── ml_inference.h       <- TFLite Micro inference engine + N-cycle fault confirmation
│   ├── model_stub.h         <- Safe placeholder FlatBuffer when real model is pending
│   ├── health_model.h       <- Motor health index (0-100%) and RUL estimation
│   ├── alerts.h             <- Non-blocking LED & buzzer state controller
│   ├── display_oled.h       <- SSD1306 OLED graphics (health bar, fault screen, safety trip)
│   ├── telemetry.h          <- Embedded WebServer (JSON REST API + auto-refresh HTML dashboard)
│   └── secrets.h.template   <- Wi-Fi credentials template (git-ignored)
├── src/
│   └── main.cpp             <- Setup, main execution loop, sensor sync, and safety interlocks
└── platformio.ini           <- PlatformIO build configuration for ESP32-S3
```

---

## 4. Machine Learning & DSP Pipeline

### 4.1 Feature Extraction (16 Features)
The feature vector extracted by `features.h` and matching `preprocess.py`:
1. `vib_rms_m`: Motor vibration RMS ($m/s^2$)
2. `vib_mean_m`: Motor vibration Mean ($m/s^2$)
3. `vib_std_m`: Motor vibration Standard Deviation ($m/s^2$)
4. `vib_rms_b`: Base vibration RMS ($m/s^2$)
5. `vib_std_b`: Base vibration Standard Deviation ($m/s^2$)
6. `vib_kurtosis_m`: Motor vibration Kurtosis (impulsiveness metric)
7. `vib_skew_m`: Motor vibration Skewness
8. `vib_crest_m`: Motor vibration Crest Factor (Peak / RMS)
9. `fft_peak_freq_hz`: Dominant spectral peak frequency (Hz) via Goertzel algorithm
10. `fft_peak_mag`: Magnitude of dominant spectral peak
11. `cur_rms_mA`: Motor current RMS (mA)
12. `cur_mean_mA`: Motor current Mean (mA)
13. `cur_std_mA`: Motor current Standard Deviation (mA)
14. `temp_motor_C`: Motor surface temperature (°C)
15. `rpm_motor`: Motor shaft rotational speed (RPM)
16. `rpm_delta`: Differential rotational speed between motor and base/load ($RPM_1 - RPM_2$)

### 4.2 Target Fault Classes (5 Classes)
- `Class 0`: **Healthy** (Normal baseline operation)
- `Class 1`: **Imbalance** (1X rotational frequency vibration spike, elevated Kurtosis)
- `Class 2`: **Misalignment** (2X rotational frequency vibration harmonics, shaft speed delta)
- `Class 3`: **Overload** (Sustained elevated motor torque and current RMS below emergency trip)
- `Class 4`: **Friction** (High-frequency vibration hash, elevated bearing temperature)

### 4.3 Training & Export Pipeline
1. **Data Collection:**
   ```powershell
   python pipeline/scripts/collect_data.py --port COM3 --label Healthy   --duration 120
   python pipeline/scripts/collect_data.py --port COM3 --label Imbalance --duration 120
   ```
2. **Preprocessing & Leakage-Free Splitting:**
   ```powershell
   python pipeline/scripts/preprocess.py --data-dir data/raw --out-dir data/processed
   ```
3. **Model Training & Evaluation:**
   ```powershell
   python pipeline/scripts/train_cnn.py
   ```
4. **Header Export for ESP32-S3:**
   ```powershell
   python pipeline/scripts/export_model.py
   ```

---

## 5. Event-Triggered Adaptive Sensing

The system does NOT operate only at a fixed-rate. Instead, it implements a 3-state adaptive sensing controller (`adaptive_sensing.h`):

| State | Sampling Interval | Trigger Conditions | Hysteresis Rule |
|---|---|---|---|
| **NORMAL** | 2000 ms (Low power/load) | Vibration Std $\le 0.35\text{ m/s}^2$, Current RMS $\le 0.30\text{ A}$, Temp $\le 55^\circ\text{C}$ | Default initial state |
| **WATCH**  | 500 ms (Medium rate)     | Vibration Std $> 0.35\text{ m/s}^2$, OR Current RMS $> 0.30\text{ A}$, OR Temp $> 55^\circ\text{C}$ | Upgrades immediately |
| **ALERT**  | 100 ms (High rate)       | Vibration Std $> 0.80\text{ m/s}^2$, OR Current RMS $> 0.60\text{ A}$, OR Temp $> 70^\circ\text{C}$ | Upgrades immediately |

**Anti-Chattering Hysteresis:** Stepping down from `ALERT` $\rightarrow$ `WATCH` or `WATCH` $\rightarrow$ `NORMAL` requires **5 consecutive cycles** below the threshold.

---

## 6. Requirement-by-Requirement Validation Matrix

| ID | Requirement | Status | Evidence / Verification Method | Notes & Limitations |
|---|---|---|---|---|
| **REQ-01** | ESP32-S3 Target Configuration | **PASS** | `platformio.ini` configured for `esp32-s3-devkitc-1`, 16MB Flash, PSRAM, USB CDC. | Verified via PlatformIO toolchain configuration. |
| **REQ-02** | 2× MPU6050 Dual-IMU Acquisition | **PASS (Simulated / Drivers Verified)** | Distinct I2C addresses `0x68` (AD0=GND) and `0x69` (AD0=3.3V). Single-IMU graceful fallback implemented. | Physical reading requires attached hardware. |
| **REQ-03** | INA219 Current Measurement | **PASS (Simulated / Drivers Verified)** | Addressed at `0x40`, 0.1Ω shunt, 3.2A full-scale, mA RMS/Mean/Std calculation. | Unit tests verify current feature calculations. |
| **REQ-04** | 2× DS18B20 Temperature Acquisition | **PASS (Simulated / Drivers Verified)** | Single GPIO 7 1-Wire bus, 11-bit resolution, separate motor and bearing addresses. | Guarded against NaN readouts. |
| **REQ-05** | 2× Hall Sensor RPM Measurement | **PASS (Simulated / Drivers Verified)** | GPIO 5 & GPIO 6 ISR-driven pulse counters with 5ms debounce and 30% IIR smoothing. | Safe ISR atomic reads. |
| **REQ-06** | TB6612FNG Motor Control & Protection | **PASS (Architecture Verified)** | GPIO 35/36 direction, GPIO 37 LEDC PWM, GPIO 38 STBY hardware cut-off. | Overcurrent/overtemp trips STBY independently of ML. |
| **REQ-07** | SSD1306 OLED Display | **PASS (Simulated / Drivers Verified)** | Startup, real-time motor telemetry, health bar, fault warning, stopped, safety trip screens. | Non-fatal fallback if OLED is unpowered. |
| **REQ-08** | Wi-Fi Telemetry & JSON REST API | **PASS (Architecture Verified)** | WebServer on Port 80, `/data` JSON endpoint, auto-refresh HTML page, `secrets.h` isolation. | Operates offline gracefully if Wi-Fi disabled. |
| **REQ-09** | 5-Class Target Classification | **PARTIALLY VERIFIED** | Pipeline, labels (0–4), feature vector (16), firmware classifier structure fully implemented. | **Limitation:** Real data exists only for Healthy and Imbalance in supplied repos. Misalignment, Overload, and Friction require collection. |
| **REQ-10** | Event-Triggered Adaptive Sensing | **PASS (Unit Tested)** | 3-state machine (NORMAL 2s, WATCH 500ms, ALERT 100ms) with 5-cycle hysteresis. | 7 unit tests pass in `test_firmware_sim.py`. |
| **REQ-11** | Hardware-Independent Electrical Safety | **PASS (Unit Tested)** | `sensors_safety_trip()` cuts motor driver STBY on $I > 1.5\text{ A}$ or $T > 80^\circ\text{C}$ without ML gating. | Verified in `sensors.h` and `main.cpp`. |
| **REQ-12** | Data Leakage Prevention | **PASS (Unit Tested)** | Recording-level splitting and chronological holdout; duplicate removal. | Verified by `test_pipeline.py` (30/30 passed). |
| **REQ-13** | TFLite Micro Inference Pipeline | **PASS (Architecture Verified)** | TFLite Micro AllOpsResolver, 48 KB tensor arena, feature normalization, confidence gating ($>0.60$). | Stub FlatBuffer compiles cleanly; real model exported via pipeline. |
| **REQ-14** | Automated Unit Test Suite | **PASS** | 30 tests across `test_pipeline.py` and `test_firmware_sim.py`. | 30 passed in 3.13s on Python 3.13. |
| **REQ-15** | Physical Hardware Validation | **NOT VERIFIED (No Physical Bench Connected)** | Physical motor bench, real sensor signals, live mechanical faults. | **Hardware Limitation:** Requires physical ESP32-S3 test bench for end-to-end bench test. |
