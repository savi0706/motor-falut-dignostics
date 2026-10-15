# Hardware Architecture, Wiring & Electrical Integration Guide

**Project:** Adaptive Multi-Sensor TinyML System for Real-Time Motor Fault Diagnosis  
**Target:** ESP32-S3 (ESP32-S3-DevKitC-1, 16MB Flash, 8MB PSRAM)  
**Author:** TinyML & Embedded Systems Engineering  

---

## 1. System Block Diagram & Power Architecture

```
                 +-----------------------------------------------+
                 |              +5V / +12V DC Motor Power        |
                 |               (External Isolated Supply)       |
                 +----------------------+------------------------+
                                        |
                                        v
                                 [VM: Motor V+]
                                        |
                    +-------------------+-------------------+
                    |         TB6612FNG Motor Driver        |
                    | VM: 4.5V-15V  | Continuous: 1.2A/ch  |
                    | VCC: 3.3V     | Peak: 3.2A pulse     |
                    +-----+-------+----------+--------------+
                          |       |          |
                     [Motor +] [Motor -] [STBY: GPIO 38]
                          |       |
                          v       v
                   +---------------------+
                   |   DC Motor (Test)   |
                   +----------+----------+
                              |
       +----------------------+----------------------+
       |                      |                      |
[MPU6050 #0 (0x68)]    [DS18B20 #0]          [Hall Sensor #1]
  Motor Body Vib.      Motor Temp.             Shaft Speed (RPM)
       |                      |                      |
       |                      |                      |
+------+----------------------+----------------------+----------------------+
|                                ESP32-S3                                   |
|   - Xtensa Dual-Core 240MHz, 512KB SRAM, 8MB PSRAM, 16MB Flash           |
|   - Shared I2C Bus @ 400kHz (GPIO 8 SDA, GPIO 9 SCL)                     |
|   - Dedicated 1-Wire Bus (GPIO 7)                                        |
|   - Interrupts for RPM (GPIO 5, GPIO 6)                                  |
|   - Hardware Emergency Cut-Off (STBY LOW, PWM 0)                         |
+------+----------------------+----------------------+----------------------+
       |                      |                      |
[MPU6050 #1 (0x69)]    [DS18B20 #1]          [Hall Sensor #2]
  Base/Bearing Vib.    Bearing Temp.          Base/Load Speed
       |                      |                      |
[INA219 (0x40)]        [SSD1306 (0x3C)]     [LEDs & Buzzer]
  Motor Current         128x64 OLED          Status & Safety
```

---

## 2. Pinout & GPIO Allocation Table

All pin assignments have been audited for ESP32-S3 strapping pin safety, native USB compatibility, and internal peripheral conflicts:

| Subsystem | Component / Signal | ESP32-S3 GPIO | Direction / Type | Notes |
|---|---|---|---|---|
| **I2C Bus** | I2C SDA | **GPIO 8** | Open-Drain Bidirectional | 4.7kΩ pull-up to 3.3V |
| **I2C Bus** | I2C SCL | **GPIO 9** | Open-Drain Bidirectional | 4.7kΩ pull-up to 3.3V |
| **1-Wire Bus** | DS18B20 #0 & #1 | **GPIO 7** | Bidirectional (1-Wire) | 4.7kΩ pull-up to 3.3V |
| **RPM Sensors**| Hall Sensor 1 (Motor) | **GPIO 5** | Input with Pull-up | Hardware Interrupt (RISING) |
| **RPM Sensors**| Hall Sensor 2 (Base)  | **GPIO 6** | Input with Pull-up | Hardware Interrupt (RISING) |
| **Alerts** | Green LED (Normal) | **GPIO 15** | Output (Active HIGH) | 330Ω current limiting resistor |
| **Alerts** | Yellow LED (Warning)| **GPIO 16** | Output (Active HIGH) | 330Ω current limiting resistor |
| **Alerts** | Red LED (Fault)     | **GPIO 17** | Output (Active HIGH) | 330Ω current limiting resistor |
| **Alerts** | Piezo Buzzer        | **GPIO 18** | Output (Active HIGH) | Driven via NPN transistor / module |
| **Motor Driver**| TB6612FNG AIN1    | **GPIO 35** | Output | Direction control |
| **Motor Driver**| TB6612FNG AIN2    | **GPIO 36** | Output | Direction control |
| **Motor Driver**| TB6612FNG PWMA    | **GPIO 37** | Output (PWM / LEDC) | 20kHz PWM frequency |
| **Motor Driver**| TB6612FNG STBY    | **GPIO 38** | Output (Active HIGH) | **Hardware Trip: Driven LOW on fault** |
| **Native USB**  | USB D- / D+       | **GPIO 19, 20**| Internal USB CDC | Serial monitor & telemetry logging |

---

## 3. I2C Bus Address Map & Conflict Verification

The project integrates 4 I2C slave devices on a single shared 400 kHz bus:

| Device | Model | Default Address | Selected Address | Configuration / Address Pin | Status |
|---|---|---|---|---|---|
| Vibration Sensor 1 | MPU6050 (Motor Body) | 0x68 | **0x68** | `AD0` pin tied to **GND** | Confirmed No Conflict |
| Vibration Sensor 2 | MPU6050 (Base/Bearing)| 0x68 | **0x69** | `AD0` pin tied to **3.3V** | Confirmed No Conflict |
| Current / Voltage  | INA219 (Motor Shunt) | 0x40 | **0x40** | `A0` = GND, `A1` = GND | Confirmed No Conflict |
| Display            | SSD1306 (128x64 OLED)| 0x3C | **0x3C** | Factory default address | Confirmed No Conflict |

**I2C Verification Result:** `0x3C`, `0x40`, `0x68`, and `0x69` are completely distinct. No I2C address conflict exists.

---

## 4. Electrical Specifications & Limits

### 4.1 TB6612FNG Motor Driver
- **Motor Supply Voltage ($V_M$):** 4.5 V to 13.5 V DC (external battery or regulated DC power pack). Never power motor from ESP32 3.3V rail!
- **Logic Voltage ($V_{CC}$):** 2.7 V to 5.5 V (Connect to ESP32-S3 3.3V output pin).
- **Current Limits:**
  - Continuous Output: **1.2 A per channel**.
  - Peak Pulse Output: **3.2 A** ($t < 10\text{ ms}$).
  - Hardware Overcurrent Trip Setting: Configured at **1.5 A continuous** in `sensors_safety_trip()`.

### 4.2 INA219 High-Side Current Monitor
- **Operating Voltage:** 3.0 V to 5.5 V.
- **Shunt Resistor:** 0.1 Ω (100 mΩ), 1% tolerance, 1 W rating.
- **PGA Setting:** $\pm 320\text{ mV}$ range $\rightarrow$ Max measurable current: $I_{\text{max}} = \frac{0.320\text{ V}}{0.1\ \Omega} = 3.2\text{ A}$.
- **Resolution:** $0.8\text{ mA}$ per LSB.

### 4.3 DS18B20 1-Wire Digital Thermometer
- **Temperature Range:** $-55^\circ\text{C}$ to $+125^\circ\text{C}$.
- **Accuracy:** $\pm 0.5^\circ\text{C}$ from $-10^\circ\text{C}$ to $+85^\circ\text{C}$.
- **Resolution:** Configured to 11-bit ($0.125^\circ\text{C}$ step, $375\text{ ms}$ max conversion time).
- **Safety Trip Limits:**
  - Motor Body Trip: **$80.0^\circ\text{C}$**.
  - Bearing Housing Trip: **$75.0^\circ\text{C}$**.

---

## 5. Wiring Instructions & Schematic Reference

### Step 1: Power & Common Ground
1. Connect external motor power supply (+6V to +12V) directly to TB6612FNG **VM** and INA219 **V+ / VIN+**.
2. Connect external power supply Ground to TB6612FNG **GND** and to ESP32-S3 **GND** (common ground plane).
3. Connect ESP32-S3 **3.3V** pin to TB6612FNG **VCC**, MPU6050 #0 VCC, MPU6050 #1 VCC, INA219 VCC, DS18B20 #0 VDD, DS18B20 #1 VDD, and SSD1306 VCC.

### Step 2: I2C Bus Wiring
1. Connect ESP32-S3 **GPIO 8** to SDA pins of MPU6050 #0, MPU6050 #1, INA219, and SSD1306.
2. Connect ESP32-S3 **GPIO 9** to SCL pins of MPU6050 #0, MPU6050 #1, INA219, and SSD1306.
3. On MPU6050 #0: connect **AD0 to GND**.
4. On MPU6050 #1: connect **AD0 to 3.3V**.
5. Add two 4.7kΩ pull-up resistors between SDA $\leftrightarrow$ 3.3V and SCL $\leftrightarrow$ 3.3V if the breakout boards lack built-in pullups.

### Step 3: Current Measurement (INA219)
1. Route the positive motor power rail from external supply into INA219 **VIN+**.
2. Connect INA219 **VIN-** to TB6612FNG **VM**.
3. This places the 0.1Ω shunt on the high side of the motor driver, sensing total motor current.

### Step 4: 1-Wire Temperature Sensors
1. Connect DS18B20 #0 DQ pin and DS18B20 #1 DQ pin together to **GPIO 7**.
2. Connect a 4.7kΩ pull-up resistor between GPIO 7 and 3.3V.
3. Affix DS18B20 #0 to the motor stator casing with thermal adhesive/tape.
4. Affix DS18B20 #1 directly adjacent to the front shaft bearing.

### Step 5: Hall RPM Sensors
1. Position Hall Sensor #1 facing a neodymium magnet glued to the motor shaft. Connect output to **GPIO 5**.
2. Position Hall Sensor #2 facing a magnet on the load/base shaft. Connect output to **GPIO 6**.
3. Connect sensor VCC to 3.3V and GND to common Ground.

### Step 6: Motor Driver Control & Emergency Stop
1. Connect **GPIO 35** $\rightarrow$ TB6612FNG **AIN1**.
2. Connect **GPIO 36** $\rightarrow$ TB6612FNG **AIN2**.
3. Connect **GPIO 37** $\rightarrow$ TB6612FNG **PWMA**.
4. Connect **GPIO 38** $\rightarrow$ TB6612FNG **STBY**.
5. Connect Motor terminals to TB6612FNG **AO1** and **AO2**.

---

## 6. Safety Interlocks & Independent Hardware Protection

The firmware architecture guarantees that electrical and thermal protection routines run **completely independently of ML predictions**:
- **Overcurrent Condition:** If INA219 reads $> 1.5\text{ A}$ continuous, `sensors_safety_trip()` returns `true` immediately.
- **Overtemperature Condition:** If motor temperature $> 80^\circ\text{C}$ or bearing temperature $> 75^\circ\text{C}$, `sensors_safety_trip()` returns `true`.
- **Action Taken Upon Trip:**
  1. `digitalWrite(MOTOR_STBY, LOW)` — TB6612FNG outputs instantly enter high-impedance mode.
  2. `analogWrite(MOTOR_PWMA, 0)` — PWM duty cycle clamped to zero.
  3. All LEDs flash synchronously at 3.3 Hz and piezo buzzer sounds continuous tone.
  4. OLED displays `!! HW SAFETY TRIP`.
  5. Telemetry packet broadcasts `safety_trip: true`.
