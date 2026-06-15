# High-Precision 3-Axis Inclinometer Using ESP32

This repository contains the firmware and documentation for a high-precision 3-axis digital inclinometer built around the ESP32 microcontroller. The project was developed as part of the Electronics Design Workshop (EDW) for the 4th semester at Netaji Subhas University of Technology (NSUT), Delhi.

---

## Overview

The system fuses real-time data from an MPU6050 inertial measurement unit and a QMC5883P magnetometer using the Madgwick AHRS sensor fusion algorithm to compute roll, pitch, and yaw. Orientation data is visualized on a 2.8-inch ST7735 TFT LCD using sprite-based double buffering to maintain a flicker-free 20 fps frame rate. All sensor sampling operates at 100 Hz in a fully non-blocking main loop. Additionally, an Edge Impulse machine learning model runs on-device for real-time gesture recognition from the IMU data.

---

## Key Features

* **Advanced Sensor Fusion:** Utilizes the Madgwick AHRS filter combined with tilt-compensated magnetometer calculations to ensure stable orientation data.
* **Flicker-Free UI:** Leverages the TFT_eSPI sprite framework to render graphics off-screen and push updates via a single DMA transfer.
* **Multiple Display Modes:** Offers a four-panel instrument configuration and three distinct graphical bubble-level modes.
* **Relative Zero Calibration:** Enables the user to redefine any orientation as the zero reference axis dynamically.
* **Configurable Threshold Alerts:** Features a software state machine driven passive buzzer providing audible alerts when user-defined tilt thresholds (5 to 90 degrees) are breached.
* **Persistent Storage:** Utilizes the ESP32's Non-Volatile Storage (NVS) flash to save user configurations across power cycles.
* **On-Device Machine Learning:** Incorporates an Edge Impulse deployment for standalone gesture classification.

---

## Hardware Architecture

### Bill of Materials

| S.No. | Component | Quantity |
| :--- | :--- | :--- |
| 1 | ESP32 DevKit (ESP32-WROOM-32, 38 pins) | 1 |
| 2 | 2.8" ST7735 TFT LCD (128x160) | 1 |
| 3 | MPU6050 IMU Module (Accelerometer + Gyroscope) | 1 |
| 4 | QMC5883P Magnetometer Module | 1 |
| 5 | Passive Buzzer (12 mm, ~16-32 Ohm) | 1 |
| 6 | Tactile Push Buttons (6x6 mm, Normally Open) | 5 |
| 7 | Li-ion Battery (18650, 3.7 V) with holder | 1 |
| 8 | TP4056 Charger Module (with DW01A protection) | 1 |
| 9 | MT3608 Mini DC-DC Boost Module | 1 |
| 10 | SPST Toggle Switch | 1 |
| 11 | Resistors (2.2 kOhm for I2C pull-ups) | 2 |
| 12 | Decoupling Capacitors (100 nF, 10 uF, 22 uF) | Assorted |
| 13 | ZeroBoard / Perfboard | 1 |

### Pin Configuration

| ESP32 GPIO | Connected To | Protocol | Purpose |
| :--- | :--- | :--- | :--- |
| **GPIO 21** | MPU6050 SDA, QMC5883P SDA | I2C (400 kHz) | Shared I2C data line |
| **GPIO 22** | MPU6050 SCL, QMC5883P SCL | I2C (400 kHz) | Shared I2C clock line |
| **GPIO 23** | TFT MOSI (SDA) | SPI (20 MHz) | Display data |
| **GPIO 18** | TFT SCLK (SCL) | SPI (20 MHz) | Display clock |
| **GPIO 5** | TFT CS | Digital Output | Display chip select |
| **GPIO 2** | TFT DC | Digital Output | Display data/command |
| **GPIO 4** | TFT RST | Digital Output | Display reset |
| **GPIO 15** | Zero Button | Input (Internal Pullup) | Zero calibration |
| **GPIO 19** | Mode Button | Input (Internal Pullup) | Display mode cycling |
| **GPIO 26** | Left Button | Input (Internal Pullup) | Settings decrease |
| **GPIO 27** | Middle Button | Input (Internal Pullup) | Settings enter/exit |
| **GPIO 25** | Right Button | Input (Internal Pullup) | Settings increase |
| **GPIO 32** | Passive Buzzer | PWM (LEDC Channel)| 2 kHz beep output |
| **3.3 V** | Sensors, TFT VCC | Power | Main power rail |
| **GND** | All GND pins | Power | Common ground |

---

## Mathematical Modeling

### Tilt-Compensated Magnetometer Heading

To determine reliable yaw (heading) when the device is tilted, the raw magnetometer vector coordinates are rotated back into the horizontal plane using the roll ($\theta_R$) and pitch ($\theta_P$) values obtained from the Madgwick sensor fusion filter:

$$m_{X,comp} = m_{X}\cos\theta_{P} + m_{Y}\sin\theta_{R}\sin\theta_{P} + m_{Z}\cos\theta_{R}\sin\theta_{P}$$

$$m_{Y,comp} = m_{Y}\cos\theta_{R} - m_{Z}\sin\theta_{R}$$

$$\psi = \arctan2(-m_{Y,comp}, m_{X,comp}) + \delta$$

Where $\delta$ represents the localized magnetic declination factor (**0.5°**).

---

## User Guide

### Display Modes

* **Mode 0 (Normal View):** Displays roll, pitch, and tilt-compensated magnetic heading values across four rounded panels, alongside real-time inference data from Edge Impulse.
* **Mode 1 (Z-Axis Bubble Level):** Renders a 2D spirit level simulation where an anti-aliased bubble tracks orientation across concentric reference rings and changes colour parameters upon threshold breach.
* **Mode 2 (X-Axis Bubble Level):** Provides a specialized 1D graphical bubble configuration constrained horizontally to precisely calibrate single-axis pitch variations.
* **Mode 3 (Y-Axis Bubble Level):** Provides a specialized 1D graphical bubble configuration constrained horizontally to precisely calibrate single-axis roll variations.

### Input Interactivity

> **Note on Hold-to-Repeat Functionality:** The Left and Right adjustment buttons support continuous scaling when held down for over 1 second, shifting values at a 5 Hz repetition rate.

* **Zero Button (GPIO 15):** Captures the current physical orientation and defines it as the temporary reference baseline. Subsequent angle calculations are processed relatively from this posture.
* **Mode Button (GPIO 19):** Cycles through the 4 individual display layout variants sequentially when the settings menu is closed.
* **Middle Button (GPIO 27):** Accesses or exits the interactive Settings Menu overlay to adjust threshold limits and triggers an immediate update to NVS flash memory upon termination.

---

## Firmware Architecture

The software implementation is structured to maintain a fully non-blocking architecture, omitting any standard blocking delay functions in the main execution framework. Workload scheduling is managed via asynchronous timestamp evaluation intervals:

| Operational Task | Operational Frequency | Description |
| :--- | :--- | :--- |
| **Sensor Polling + Fusion** | 100 Hz (10 ms intervals) | Samples IMU/Magnetometer registers and advances Madgwick math iterations. |
| **Edge Impulse Data Ingestion**| 60 Hz (16 ms intervals) | Forwards raw 6-axis data streams directly into the internal ML classification buffer. |
| **UI Rendering and Refresh** | 20 Hz (50 ms intervals) | Compiles off-screen sprite buffers and initiates one-shot transfers to the TFT. |
| **Button Debounce Evaluation** | Continuous Polling | Monitors input states over a 50 ms stabilization window to resolve switch noise. |
| **Audio State Sequencing** | Continuous Polling | Drives the active timing intervals for double-beep warnings and confirmation tones. |

---

## System Diagnostics & Troubleshooting

* **SMD TP4056 Overheating Fault:** Hand-soldering fine-pitch SMD components led to an underlying short between power lines. *Solution:* This layout was upgraded to a dedicated 6-pin pre-assembled module hosting an integrated DW01A protection IC to ensure structural reliability.
* **ESP32 I2C Communication Failures:** The default Wire implementation caused bus runtime crashes when using traditional repeated-start parameters (`endTransmission(false)`). *Solution:* Resolving this required developing a custom QMC5883P driver enforcing absolute STOP conditions (`endTransmission(true)`) before executing read calls.
* **Graphic Tearing and UI Refresh Stutter:** Direct screen drawing produced severe flickering. *Solution:* Transitioning to a sprite-based double-buffering scheme isolated approximately 40 KB of system RAM to enable high-speed graphics.

---

## Project Credits

Developed by the following engineering team members at NSUT Delhi (Session 2025-2026):

* **Aryaman Raj Jaiswal** (2024UEC2658)
* **Ansh Aggarwal** (2024UEC2672)
* **Priyanshu Singh** (2024UEC2687)
