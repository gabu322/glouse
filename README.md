# Glouse: Motion-Controlled BLE Mouse

[Leia em Português 🇧🇷](./README.pt.md)

Glouse is a motion-controlled BLE (Bluetooth Low Energy) mouse powered by an ESP32 microcontroller and an MPU6050 motion sensor. It allows you to control the cursor and perform mouse actions using hand gestures and touch-sensitive buttons integrated into a glove.

## Table of Contents

1. [Features](#features)
2. [Components Used](#components-used)
   - [Hardware](#hardware)
   - [Software Environment](#software-environment)
   - [Libraries](#libraries)
3. [How It Works](#how-it-works)
4. [Pin Configuration](#pin-configuration)
5. [Getting Started](#getting-started)
   - [Prerequisites](#prerequisites)
   - [Setup](#setup)
   - [Usage](#usage)
6. [Notes](#notes)
7. [License](#license)

## Features

- **Motion Control**: Use the MPU6050 to move the mouse pointer based on pitch and roll.
- **Touch-Sensitive Buttons**: Perform mouse clicks and other actions using touch-sensitive inputs.
- **BLE Connectivity**: Connect the ESP32 to your computer or device as a Bluetooth mouse.
- **Customizable Sensitivity**: Adjust pointer and scroll sensitivity.
- **FreeRTOS Integration**: Leverages the multitasking capabilities of the ESP32.

## Components Used

### Hardware

- ESP32 microcontroller
- MPU6050 6-axis motion sensor (accelerometer and gyroscope)
- Touch-sensitive inputs (ESP32 touch pins)

### Software Environment

- PlatformIO in Visual Studio Code
- Arduino framework

### Libraries

- [I2Cdev](https://github.com/jrowberg/i2cdevlib): For I2C communication with the MPU6050.
- [MPU6050\_6Axis\_MotionApps20](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050): Simplifies interaction with the MPU6050.
- [Wire](https://www.arduino.cc/en/Reference/Wire): I2C library for Arduino.
- [BleMouse](https://github.com/T-vK/ESP32-BLE-Mouse): Enables the ESP32 to act as a BLE mouse.

## How It Works

1. **Motion Detection**:

   - The MPU6050 captures motion data (yaw, pitch, and roll).
   - The data is processed to control the mouse pointer's movement.

2. **Touch Controls**:

   - Touch-sensitive pins on the ESP32 detect user interactions.
   - Each pin corresponds to a specific mouse action (e.g., left click, right click, scroll).

3. **BLE Mouse Functionality**:

   - The ESP32 acts as a BLE mouse and communicates with a host device.

## Pin Configuration

| ESP32 Pin | Connection | Local   | Function                                 |
| --------- | ---------- | ------- | ---------------------------------------- |
| 21        | SDA        | MPU6050 | MPU Data reader                          |
| 22        | SCL        | MPU6050 | MPU Clock                                |
| 3.3V      | 3.3V       | MPU6050 | MPU supply                               |
| GND       | GND        | MPU6050 | MPU ground                               |
| GND       | Your hand  |         | Ground to your hand for stable readings* |
| 15        | Finger 1.1 | Glove   | Left Click                               |
| 13        | Finger 1.2 | Glove   | Back                                     |
| 12        | Finger 1.3 | Glove   | Forward                                  |
| 14        | Finger 2.1 | Glove   | Right Click                              |
| 27        | Finger 2.2 | Glove   | Middle Click                             |
| 33        | Finger 3.1 | Glove   | Scroll (special)                         |
| 32        | Finger 3.2 | Glove   | Config (special)                         |

**\*** The GND pin connected to your hand improves the sensitivity and stability of touch readings by providing a consistent reference ground.

## Getting Started

### Prerequisites

- Install [PlatformIO](https://platformio.org/) in Visual Studio Code.
- Ensure you have the necessary libraries installed (listed above or find them in platformio.ini).

### Setup

1. Clone the repository.
2. Open the project in VS Code with PlatformIO.
3. Connect the ESP32 to your computer via USB.
4. Build and upload the code to the ESP32.

### Usage

- Pair the ESP32 with your computer or device via Bluetooth (default device name: "Glouse").
- Wear the glove and use touch-sensitive inputs and motion to control the mouse.

## Notes

- This project is currently in development, and some features or wiring details may change in the future.
- The wiring diagram will be added in a later update.

## License

This project is open-source and available under the MIT License.

