# Controller Firmware Guide

This guide applies to `controller/`. Read the repository-level `../AGENTS.md`
first. Firmware changes affect physical equipment and must be compiled within
the ATmega32U4 limits before handoff.

## Hardware

- Microcontroller: Arduino Leonardo ETH R3, ATmega32U4.
- Shield: customized CNC Shield V3; A-axis clone jumpers are removed.
- Motors: two NEMA 17 steppers using A4988/DRV8825 drivers.
- Valves: ESBE VRG131, with Motor 1 mechanically mirrored relative to Motor 2.
- Local UI: 16×2 I2C LCD, KY-040 rotary encoder, and push button.
- Temperatures: four DS18B20 readings on one OneWire bus: first floor, second
  floor, boiler water, and boiler room.
- Endstops: one shared MIN input and one shared MAX input for both motors.
- Driver enable: one global active-low enable pin.

## Pinout

| Pin | Function |
| --- | --- |
| A0 | OneWire temperature bus |
| A1 | Menu button, software debounced |
| A2 / A3 | Encoder A / B |
| 11 | Shared MIN endstop |
| 9 | Shared MAX endstop |
| 4 / 7 | Motor 1 STEP / DIR |
| 12 / 13 | Motor 2 STEP / DIR |
| 8 | Global motor enable, active LOW |

Pin 4 may conflict with an Ethernet shield SD-card chip-select line. The current
hardware arrangement works without using that SD interface.

## Firmware structure

The implementation is intentionally concentrated in `src/main.cpp` because
flash space is nearly exhausted.

- `setup()` initializes pins, LCD, sensors, motors, EEPROM IP, and Ethernet.
- `loop()` services sensors, local input, HTTP, motor stepping, and LCD updates.
- `manageMotors()` serializes physical motor execution and handles shared
  endstops.
- `triggerBackoff()` releases the active switch and adds clearance before
  normalizing the counter.
- `handleNetwork()` implements a minimal HTTP parser and JSON output.
- `parseAndSaveIP()` updates the four EEPROM bytes used for the static address.

Only one motor is stepped at a time. Drivers are enabled while either motor has
distance remaining and disabled when both are idle. Temperature conversion is
skipped while motors are enabled.

## Local UI

The button cycles dashboard → Motor 1 → Motor 2 → dashboard. The encoder moves
the selected motor by `ENCODER_TURN_STEP` (`500`) per click. Control screens
return to the dashboard after five minutes.

## Controller HTTP API

The server listens on port 80.

- `GET /status` → JSON containing `fl1`, `fl2`, `boiler`, `room`, `m1`, `m2`.
- `GET /set?motor=1|2&pos=min|max|<signed steps>` → accepts one movement.
- `GET /setip?ip=A.B.C.D` → writes EEPROM; a reboot is required.

The parser is deliberately minimal and does not URL-decode query values. Send a
positive relative count as unsigned digits; `%2B` is not a valid substitute. A
successful response means only that a command was accepted.

## Endstop behavior

Both motors share the physical endstop signals. When a matching switch and
movement direction are detected, the firmware:

1. runs slowly away until the switch is released;
2. moves another `BACKOFF_STEPS` (`100`) for clearance;
3. normalizes the raw counter near `100` or `9900`.

Do not simplify this into an immediate counter reset: leaving a shared switch
pressed prevents safe operation of the other motor.

Motor 1 is mirrored and currently calls
`motor1.setPinsInverted(true, false, false)`. This creates the deployed
compatibility issue described below.

## Critical memory constraints

ATmega32U4 capacity is 28 KB flash and 2.5 KB RAM. The latest verified build
uses approximately 28,502/28,672 bytes of flash (99.4%) and 1,008/2,560 bytes of
RAM.

Mandatory rules:

1. Never introduce Arduino `String`; use fixed buffers and pointer parsing.
2. Wrap every static string passed to `print()`/`println()` in `F("...")`.
3. Do not add JSON or other heavy libraries; emit JSON manually.
4. Prefer refactoring/reuse over parallel implementations.
5. Run `pio run` after every firmware change and report flash/RAM usage.
6. Do not flash hardware without explicit authorization. Compilation alone is
   not permission to upload or reboot the controller.

## TODO: correct Motor 1 endstop direction

The deployed controller cannot currently be flashed. Motor 1's DIR output is
inverted, but `manageMotors()` still associates negative logical motion with
`PIN_COMMON_MIN` and positive logical motion with `PIN_COMMON_MAX`. On installed
hardware the two normalisation constants therefore end up swapped: a `min` cycle
finishes near `9500`, a `max` cycle near `500`, and samples during travel can be
outside that interval.

The webapp tolerates either mapping, because it anchors on whichever constant the
firmware last wrote and counts real steps from there. When a firmware update
becomes possible:

1. verify which physical shared switch is reached for each Motor 1 direction;
2. update only Motor 1's endstop direction/pin and backoff mapping;
3. run `pio run` and remain within flash/RAM limits;
4. upload only with explicit approval and physically verify MIN, MAX, release,
   and shared-switch clearance;
5. recalibrate from the dashboard and verify 0%, 25%, 50%, 75%, and 100% for
   both motors.

Note for the webapp side: the counter constants stay unrelated to the physical
travel even after this fix, so the measured-travel scale must stay.

## Build

```bash
cd controller
pio run
```
