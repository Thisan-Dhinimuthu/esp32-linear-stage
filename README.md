# ESP32 Linear Stage Control System

This project is an ESP32-based closed-loop linear actuator control system. It uses a stepper motor with a TB6600 driver, encoder feedback, limit switches, keypad input, OLED display, and Wi-Fi control for accurate linear position control.

## Features

- ESP32 microcontroller-based control
- Stepper motor control using TB6600 driver
- Closed-loop position feedback using rotary encoder
- Left and right limit switch protection
- Homing function for position reference
- 4x4 keypad input for position commands
- OLED display for system status
- Wi-Fi web control interface
- PlatformIO project structure

## Hardware Used

- ESP32 development board
- NEMA 17 stepper motor
- TB6600 stepper motor driver
- Rotary encoder
- 2 limit switches
- 4x4 matrix keypad
- SSD1306 OLED display
- External power supply

## Pin Configuration

| Component | ESP32 Pin |
|---|---|
| Step pin | GPIO 21 |
| Direction pin | GPIO 19 |
| Enable pin | GPIO 18 |
| Left limit switch | GPIO 34 |
| Right limit switch | GPIO 35 |
| Encoder A | GPIO 32 |
| Encoder B | GPIO 33 |

## Software

This project is developed using:

- VS Code
- PlatformIO
- Arduino framework
- AccelStepper library
- Keypad library
- Adafruit SSD1306 OLED library

## Project Aim

The aim of this project is to design and develop a low-cost closed-loop linear motion control system that can move to a required position accurately using encoder feedback and limit switch safety.

## Future Improvements

- Add PID position control
- Add mobile app control
- Improve mechanical accuracy
- Add emergency stop button
- Store calibration data in EEPROM
