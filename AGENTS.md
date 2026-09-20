# Boiler Regulator: Repository Guide

Read this file first for repository-wide context, then read the nearest nested
`AGENTS.md` before changing a component:

- `controller/AGENTS.md` — hardware, firmware, pinout, endstops, memory limits,
  and the deferred Motor 1 firmware TODO.
- `webapp/AGENTS.md` — React/Fastify architecture, HTTP API, calibration model,
  persistence, diagnostics, deployment, and tests.

## Project purpose

This repository controls two ESBE VRG131 heating valves and exposes their state
through a mobile-first LAN dashboard. An Arduino Leonardo ETH drives both
stepper motors and reads temperature sensors. A Node.js web application is the
only browser-facing component and translates percentages into the controller's
small HTTP protocol.

## Repository layout

- `controller/` — PlatformIO Arduino firmware for the physical controller.
- `webapp/` — React client plus Fastify backend. The backend connects directly
  to the controller, normally at `http://192.168.88.20`.
- `deploy/` — host-specific deployment assets (`deploy/bob/` for the Synology
  NAS; `deploy/bob/deploy.sh` syncs, rebuilds, restarts and verifies it).
- `docs/` — design choices and implementation plans.
- `3D_models/` — printable enclosure/mechanical assets.

## System architecture

```text
Browser
  │ HTTP :8080
  ▼
React UI + Fastify backend
  │ controller HTTP :80
  ▼
Arduino Leonardo ETH → CNC shield → two stepper motors/shared endstops
```

The browser never connects to the Arduino directly. The backend owns command
serialization, calibration, persisted position knowledge, retries, and
diagnostic logs. The controller owns real-time stepping, physical endstop
handling, local LCD/encoder input, temperature acquisition, and motor power.

## Cross-component invariants

1. Both motors share MIN and MAX endstop inputs. Never run the motors in
   parallel; all backend motion must remain behind one global command lock.
2. A controller command response acknowledges receipt, not completed movement.
   Completion must be inferred from stable `/status` samples.
3. Do not retry movement commands automatically. Duplicate relative movement is
   unsafe. Read-only status calls may be retried during transient timeouts.
4. Treat motor movement and firmware upload as physical operations. Unit tests,
   builds, and read-only checks do not authorize movement or flashing.
5. Preserve calibration data across webapp restarts. An interrupted calibration
   must remain invalid rather than silently trusting an old scale.
6. The counter values the firmware writes at the end-stops are constants, not
   measured positions. Never derive a percentage scale from the distance
   between them. Read both component guides before changing direction,
   endpoint, or percentage logic.
7. Keep the controller protocol small and backward compatible. Coordinate any
   protocol change across `controller/` and `webapp/`.

## Current deployed compatibility state

At every end-stop the firmware backs off and overwrites the counter with a
constant: `BACKOFF_STEPS` at one end and `MAX_POSITION_LIMIT - BACKOFF_STEPS`
at the other (`500` and `9500` on the deployed build). Those `9000` steps are a
firmware convention; the valves really travel about `6000` steps. The webapp
therefore measures travel during calibration and converts percentages with the
measured step count.

Motor 1 is physically mirrored and uses an inverted direction pin. Its deployed
firmware still has the non-inverted endstop-direction mapping, so it writes the
two constants the other way round (`min` near `9500`, `max` near `500`) while
its counter still grows towards MAX; intermediate values can be outside both
constants. The backend needs no dedicated switch for this: it anchors on
whichever constant the firmware last wrote and counts steps from there. After
the firmware TODO in `controller/AGENTS.md` is flashed, recalibrate and
physically verify both regulators.

## Verification entry points

Firmware:

```bash
cd controller
pio run
```

Web application:

```bash
cd webapp
npm test
npm run typecheck
npm run lint
npm run build
```

Run the smallest relevant checks while iterating, then all component checks
before handoff. Never use a real movement command as an automated test.
