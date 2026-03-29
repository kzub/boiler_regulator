# AI Agent Context & Developer Guide: Boiler Valve Regulator

This document serves as the comprehensive context and reference guide for any AI agent working on this project in the future. Read this file first to understand the hardware, software architecture, core logic, and critical constraints before making any modifications to the codebase.

## 1. Project Overview
This project is a custom firmware for a **Remote Heater Valve Controller** (specifically designed for ESBE VRG131 valves). It uses an Arduino to control two stepper motors via a CNC shield. The device features an I2C LCD, a rotary encoder with a push button for local UI, 1-Wire temperature sensors, and an Ethernet interface for remote monitoring and control.

## 2. Hardware Specifications
* **Microcontroller:** Arduino Leonardo ETH R3 (ATmega32U4)
* **Shield:** CNC Shield V3 (customized - jumpers removed for A-axis isolation)
* **Motors:** 2x NEMA 17 Stepper Motors (A4988 / DRV8825 drivers)
* **UI:** 16x2 I2C LCD (PCF8574T), Rotary Encoder (KY-040), 1x Push Button
* **Sensors:** 3x DS18B20 OneWire temperature sensors
* **Endstops:** 2x Physical limit switches (**SHARED** by both motors for MIN and MAX bounds)

### Pinout Configuration
* **A0:** One-Wire Temp Sensors
* **A1:** Menu Button (Debounced in software)
* **A2 / A3:** Rotary Encoder A / B
* **11:** Shared MIN Endstop (Both motors trigger this at position 0)
* **9:** Shared MAX Endstop (Both motors trigger this at max position)
* **4 / 7:** Motor 1 (First Floor) - Step / Dir
* **12 / 13:** Motor 2 (Second Floor) - Step / Dir
* **8:** Global Motor Enable (Active LOW)

## 3. Software Architecture
* **Environment:** PlatformIO (`leonardoeth` board, `arduino` framework)
* **Key Libraries:** `AccelStepper`, `DallasTemperature`, `OneWire`, `LiquidCrystal_I2C`, `Ethernet`, `EEPROM`
* **Local UI State Machine:**
  * `SCREEN_DASHBOARD`: Displays temperatures (F1, F2, Boiler, Room).
  * `SCREEN_CONTROL_M1`: Manual control of Motor 1 using the encoder.
  * `SCREEN_CONTROL_M2`: Manual control of Motor 2 using the encoder.
* **Network API (HTTP Port 80):**
  * `GET /status`: Returns JSON with temperatures and motor positions.
  * `GET /set?motor=[1|2]&pos=[+X|-X|min|max]`: Move a specific motor relatively or to a bound.
  * `GET /setip?ip=XXX.XXX.XXX.XXX`: Sets static IP, saves to EEPROM, and requires reboot.

## 4. Core Logic & Special Behaviors
* **Shared Endstops (Collision Prevention):** Because both motors share the same physical MIN and MAX endstops, a motor triggering an endstop will block the other motor if not cleared. 
  * **Logic:** When an endstop is hit, the firmware forces the motor to slowly back off (`motor.runSpeed()`) until the switch is physically released (`digitalRead() == HIGH`), followed by an extra `BACKOFF_STEPS` clearance move.
* **Motor Inversion:** Motor 1's physical mounting is mirrored compared to Motor 2. Motor 1 direction pins are inverted in software (`motor1.setPinsInverted(true, false, false);`).
* **Power Management:** To prevent overheating, stepper drivers are dynamically enabled/disabled. The enable pin (8) goes LOW (active) only when `distanceToGo() != 0`.
* **Encoder Tuning:** One encoder click corresponds to `500` motor steps for fast manual tuning.

## 5. CRITICAL CONSTRAINTS: Memory (Flash & RAM)
**DO NOT IGNORE THIS SECTION.**
The ATmega32U4 has severe memory limitations (28KB Flash, 2.5KB RAM). As of the latest build, **Flash is at 99.4% capacity (~170 bytes free)**.

When modifying the code, you **MUST** adhere to the following rules:
1. **No `String` class:** Never use Arduino `String` objects. Use standard C strings (`char array`), pointer arithmetic, and manual parsing (e.g., as seen in `parseAndSaveIP`).
2. **Use `F()` Macro:** All static strings in `print()` statements must be wrapped in `F("...")` to keep them out of RAM.
3. **No Heavy Libraries:** Do not import JSON parsing libraries (like `ArduinoJson`). Construct JSON responses manually via `client.print()`.
4. **Code Reuse:** If you add logic, see if you can refactor existing logic to save space.
5. **Compile Check:** Always run `pio run` to verify the size after making changes. If the build fails due to size, you must aggressively optimize your additions.

## 6. Recent Modifications (March 2026)
* Fixed shared endstop collision by changing `triggerBackoff` to wait for the specific pin to physically release before marking the backoff as complete.
* Added `motor1.setPinsInverted(true, false, false)` to align Floor 1 valve rotation with logical increment/decrement.
* Increased `ENCODER_TURN_STEP` to `500` to speed up the manual UI tuning.
* Increased the auto-revert to dashboard timer (`menuResetSeconds`) to 5 minutes.

## 7. How to build
```bash
cd controller
pio run
```