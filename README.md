![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=cplusplus&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-ESP32-E7352C?style=flat)
![Protocol](https://img.shields.io/badge/Protocol-BLE_HID-0082FC?style=flat)
![License](https://img.shields.io/badge/License-MIT-22C55E?style=flat)
![Status](https://img.shields.io/badge/Status-In_Development-F59E0B?style=flat)

[Leia em Português 🇧🇷](README.pt.md)

# Glouse: Motion-Controlled BLE Mouse

Glouse is a wearable glove that replaces a conventional mouse using hand orientation and finger touch. An **ESP32** reads motion data from an **MPU6050** sensor to move the cursor based on hand tilt (pitch/roll), while the ESP32's built-in **capacitive touch pins** detect finger taps for click actions. The device pairs with any computer or smartphone over **Bluetooth HID** — no USB dongle, no drivers.

This project was developed as an undergraduate final project (TCC) at **Universidade de Vila Velha (UVV)**, Brazil.

---

## Motivation

Traditional mice require a flat surface and constant lateral hand movement, which limits usability in compact environments, during travel, or in workflows that demand frequent keyboard-to-mouse transitions. Glouse explores an alternative interaction model: controlling the cursor through natural hand inclination and fingertip touch, untethered from any surface.

---

## Table of Contents

1. [Features](#features)
2. [Components](#components)
3. [How It Works](#how-it-works)
4. [Prototypes](#prototypes)
5. [Pin Configuration](#pin-configuration)
6. [Getting Started](#getting-started)
7. [Current Status](#current-status)
8. [License](#license)

---

## Features

- **Motion Control** — Cursor moves based on hand pitch and roll via MPU6050
- **Touch-Sensitive Buttons** — Click actions through ESP32 native capacitive pins (no mechanical buttons)
- **BLE HID Connectivity** — Pairs natively with computers and smartphones as a standard Bluetooth mouse
- **Wireless & Portable** — Battery-powered, no cables or USB receivers required
- **Multitasking** — FreeRTOS handles sensor reading and BLE transmission concurrently
- **Adjustable Sensitivity** — Pointer and scroll sensitivity configurable in code

---

## Components

### Hardware

| Component | Role |
|---|---|
| ESP32 | Main microcontroller — processing, BLE, capacitive touch |
| MPU6050 | 6-axis IMU (accelerometer + gyroscope) for hand orientation |
| LiPo Battery | Portable power supply |
| Charge + Step-up Module | Battery management (v2) |
| OLED Display | Status display — battery level, mode (v2, in progress) |

### Software Environment

- PlatformIO in Visual Studio Code
- Arduino framework for ESP32

### Libraries

- [I2Cdev](https://github.com/jrowberg/i2cdevlib) — I2C communication with the MPU6050
- [MPU6050_6Axis_MotionApps20](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) — DMP-based orientation processing
- [Wire](https://www.arduino.cc/en/Reference/Wire) — I2C Arduino library
- [BleMouse](https://github.com/T-vK/ESP32-BLE-Mouse) — BLE HID mouse emulation for ESP32

---

## How It Works

1. **Motion Detection**
   - The MPU6050 continuously reads accelerometer and gyroscope data
   - Pitch and roll angles are extracted and mapped to X/Y cursor displacement
   - Sensor fusion and filtering reduce noise and prevent erratic movement

2. **Touch Controls**
   - ESP32 capacitive pins act as electrodes on each finger position
   - Finger contact is detected as a capacitance change — no physical buttons needed
   - Grounding the GND pin to the user's hand stabilizes readings significantly

3. **BLE Mouse Transmission**
   - The ESP32 presents itself to the host as a standard Bluetooth HID mouse
   - Movement deltas and click events are sent as HID reports at low latency
   - Compatible with Windows, macOS, Linux, and Android out of the box

4. **FreeRTOS Multitasking**
   - Sensor reading and BLE transmission run as separate tasks, ensuring smooth cursor movement even under Bluetooth scheduling delays

---

## Prototypes

### Prototype 1 — Proof of Concept

The first version validated the full sensor-to-cursor pipeline using a wired assembly:
- ESP32 + MPU6050 + 18650 battery + charge module + step-up converter
- Components fixed to a regular glove with tape and loose wiring
- **Result:** Cursor movement, click detection, and BLE pairing all functional — but heavy and ergonomically limited due to the 18650 battery

### Prototype 2 — Refined Build

Built on the confirmed functionality of v1, focused on ergonomics and portability:
- Smaller, lighter LiPo battery
- Integrated charger and step-up on a single compact board
- Cleaner component layout redistributed across the glove
- Reduced exposed wiring
- OLED display mount prepared for status feedback (battery, mode)

---

## Pin Configuration

| ESP32 Pin | Connection | Component | Function |
|---|---|---|---|
| 21 | SDA | MPU6050 | I2C Data |
| 22 | SCL | MPU6050 | I2C Clock |
| 3.3V | VCC | MPU6050 | Power |
| GND | GND | MPU6050 | Ground |
| GND | Skin contact | User's hand | Touch reference ground* |
| 15 | Finger 1.1 | Glove | Left Click |
| 13 | Finger 1.2 | Glove | Back |
| 12 | Finger 1.3 | Glove | Forward |
| 14 | Finger 2.1 | Glove | Right Click |
| 27 | Finger 2.2 | Glove | Middle Click |
| 33 | Finger 3.1 | Glove | Scroll (special) |
| 32 | Finger 3.2 | Glove | Config (special) |

**\*** Connecting GND to the user's skin provides a stable capacitance reference, significantly improving touch detection accuracy.

---

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) installed in Visual Studio Code
- Libraries listed above (or resolved automatically via `platformio.ini`)

### Setup

1. Clone the repository
2. Open the project folder in VS Code with PlatformIO
3. Connect the ESP32 via USB
4. Build and upload via PlatformIO

### Usage

1. Power on the glove
2. Pair with your computer or smartphone via Bluetooth (device name: **"Glouse"**)
3. Use hand tilt to move the cursor and finger taps to click

---

## Current Status

| Feature | Status |
|---|---|
| BLE HID connection | ✅ Working |
| Cursor movement (pitch/roll) | ✅ Working |
| Click detection (capacitive) | ✅ Working |
| Scroll gesture | ✅ Working |
| OLED status display | 🔄 In progress |
| Wiring diagram | 📋 Pending |
| Sensitivity calibration UI (on display) | 📋 Planned |

> This project is in active development. Some features and wiring details may change.

---

## License

This project is open-source under the [MIT License](LICENSE).
