# Web Application Guide

This guide applies to `webapp/`. Read `../AGENTS.md` first for system-wide
invariants. This component must work with the currently deployed controller
without requiring a firmware update.

## Purpose and runtime

The webapp is a mobile-first LAN dashboard and the safety/translation layer
between browsers and the Arduino controller.

- Runtime: Node.js 22+.
- Backend: Fastify 5, TypeScript, native `fetch`.
- Frontend: React 19, TypeScript, Vite.
- Default bind address: `0.0.0.0:8080`.
- Controller origin: `CONTROLLER_BASE_URL`, normally `http://192.168.88.20`.
- Persistence: JSON calibration snapshot plus an `.in-progress` marker.
- Deployment: local process, `compose.yaml` (named volume), or the Synology
  NAS project in `../deploy/bob/` (bind-mounted `data/`, host port 8088).

The browser calls only `/api` on this application. Never add direct
browser-to-Arduino requests.

## Source map

```text
src/shared/contracts.ts            shared API and calibration types
src/server/server.ts               Fastify assembly/static client serving
src/server/config.ts               validated environment configuration
src/server/routes.ts               browser-facing HTTP routes/errors
src/server/build-info.ts           build stamp reported by /api/version
src/server/arduino-client.ts       controller HTTP protocol client
src/server/calibration-manager.ts  calibration, motion, percentages, recovery
src/server/calibration-store.ts    atomic state and interruption marker
src/server/command-queue.ts        global physical-operation lock
src/server/controller-journal.ts   controller/backend diagnostic journal
src/client/api.ts                  API client and browser-side journal
src/client/hooks/                  polling/status/log composition
src/client/components/             controls, copied log UI, error boundary
scripts/build-info.mjs             writes dist/build-info.json during a build
tests/unit/                        backend and persistence tests
tests/controller-simulator.ts      firmware-accurate controller double
tests/mock-arduino-server.ts       development controller substitute
```

Keep hardware semantics in the server. The client displays state, confirms
physical actions, and reports failures; it must not calculate raw movement.

## Browser-facing API

- `GET /health/live`, `GET /health/ready`, `GET /api/version`
- `GET /api/status`, `GET /api/calibration`
- `POST /api/calibration` with no JSON body required
- `GET /api/controller-log`
- `POST /api/regulators/:id/limit` with `{ "limit": "min" | "max" }`
- `POST /api/regulators/:id/target` with `{ "percent": 0..100 }`

The request helper adds `Content-Type: application/json` only when a body exists.
Do not reintroduce an empty JSON body/header for calibration; Fastify rejects it
with `FST_ERR_CTP_EMPTY_JSON_BODY`.

Controller failures map to structured errors: unreachable 503, timeout 504,
invalid controller response/calibration failure 502, invalid input 400, and
busy/conflict states 409.

## Motion safety model

`CommandQueue` is a single global lock because both motors share endstops. Keep
all movement and calibration behind it. Never add parallel motor execution.

A command response acknowledges acceptance, not completed movement. Completion
is inferred from stable `/status` samples. Never automatically retry movement
commands: a lost response does not prove the Arduino rejected the command.
Calibration may retry read-only status calls through transient failures.

During calibration, `publicStatus()` returns the latest sample owned by the
calibration job. It must not start a competing Arduino request. The Arduino HTTP
server is effectively single-request; concurrent polling caused intermittent
503 errors.

## Calibration and persistence

`CalibrationManager` owns the state machine. Preserve these invariants:

1. Capture the trusted opening percentage before movement.
2. Calibrate and restore Motor 1 before beginning Motor 2.
3. Save `<calibration file>.in-progress` before starting.
4. Remove that marker only after both scales/restored positions are saved.
5. After a restart with the marker present, require recalibration and restore
   from persisted trusted percentages, not failed endpoint positions.
6. A counter that no longer belongs to the scale invalidates it.
7. Manual encoder changes observed while ready update `lastObservedRaw` and
   `lastKnownPercent` atomically.

Snapshots use temporary-file-plus-rename writes. Preserve that atomic pattern.
Do not casually edit or delete production calibration data; it represents
physical valve state.

## Measured-travel scale (schema 4)

The controller never reports a physical position. At an end-stop it backs off and
*writes a constant* into the counter: `BACKOFF_STEPS` at one end,
`MAX_POSITION_LIMIT - BACKOFF_STEPS` at the other (`500`/`9500` on the deployed
build). The `9000` steps between those constants are a firmware convention; the
valves travel roughly `6000` steps. A scale built from `maxRaw - minRaw` opens a
valve about 1.5× too far — a 50% request landed near 75%. Never reintroduce it.

Each regulator therefore stores measured steps plus a moving reference point:

- `minRaw` / `maxRaw` — the constants observed after a MIN and a MAX homing;
- `travelSteps` — measured steps between both homed positions;
- `anchorRaw` / `anchorPercent` — the last trusted reference point.

`percent = anchorPercent + (raw - anchorRaw) × 100 / travelSteps`, and every move
is the inverse. Calibration per regulator is exactly **two homing runs**: MIN
establishes one constant, MAX measures the travel and leaves the anchor at 100%,
and the opening is restored with one relative move. Do not add a third homing run
to “return to a known MIN”; the anchor already carries that information.

Travel is measured from the peak excursion observed during the MAX run, and the
MIN run measures in the same way where the valve stood before calibration — that
is what the opening is restored to when no trusted percentage is persisted, so a
first calibration never leaves both valves wide open:

1. only counter values the motor has already left count as travel — the value it
   settles on is the end-stop constant, which for Motor 2 is *above* the furthest
   position actually reached;
2. half of the last sampled step is added, because the switch closes between two
   `/status` reads and is never sampled at the moment it trips (calibration polls
   at `100 ms` to keep that step small);
3. the firmware clearance is subtracted: it is the smaller of the two constants,
   since the firmware writes `BACKOFF_STEPS` at one end and
   `MAX_POSITION_LIMIT - BACKOFF_STEPS` at the other.

Homing re-anchors: a counter that is exactly `minRaw` or `maxRaw` means the
firmware just normalised it, so the anchor moves to 0% or 100%. This covers
manual MIN/MAX buttons, a target move that ran into a switch, and mirrored
Motor 1 — whose `min` writes the high constant while its counter still grows
towards MAX, so valid values sit outside both constants. Raw zero can be a valid
position on that scale; do not restore the assumption that every zero is a
controller reset. A counter that fits no branch invalidates the scale instead.

Schema 3 and older snapshots are rejected on load: they never measured travel, so
they cannot be repaired without moving the valve. The dashboard asks for a new
calibration, and the previous openings stay untouched until it runs.

After `../controller/AGENTS.md`'s firmware TODO is flashed, Motor 1 simply writes
the constants the other way round; recalibrate and verify both directions. The
measured-travel scale stays.

## Build identity

`npm run build` writes `dist/build-info.json` (version, build time, commit) and
`GET /api/version` serves it; the dashboard prints it in a footer. The NAS image
is built from a copy of `webapp/` without `.git`, so `deploy/bob/deploy.sh`
stamps `app/.build-commit` and the build falls back to it. A container reporting
a build time older than the last deploy was started without rebuilding; that is
what `deploy.sh` checks before it reports success.

## Diagnostics

The dashboard merges three timestamped sources:

- `browser` — browser → backend attempts, retained in tab `sessionStorage`;
- `controller` — backend → Arduino requests and responses;
- `backend` — calibration phases, boundaries, travel, targets/deltas, retries,
  mismatches, and failures.

The UI retains the most recent journal attempts, highlights failures, and has a
copy button with Clipboard API plus insecure-HTTP fallback. Do not log
credentials, headers, or secrets. The server journal is process-local;
calibration state is durable.

Both journals are bounded so continuous polling cannot grow memory or tab storage
without limit: the backend keeps `defaultJournalLimit` (500) newest-first entries
in `controller-journal.ts`, and the browser keeps `browserLogLimit` (200) in
`client/api.ts`, shrinking the persisted copy when `sessionStorage` rejects a
write. Both limits are covered by unit tests; keep that coverage when changing
them.

## Frontend behavior

- Status polls every five seconds and faster during reported motion.
- Two consecutive browser/backend failures mark the dashboard offline.
- Controls require confirmation before physical commands.
- Quick targets are 25%, 50%, and 75%; incremental step buttons are absent.
- Calibration, offline state, or active motion disables conflicting controls.
- The Russian UI follows the system light/dark theme.
- `ErrorBoundary` wraps the app: a render error shows a readable fatal screen
  with a manual reload instead of a blank page, and never retries a command.
- A footer prints the running build (version, commit, build time) from
  `/api/version`; it stays hidden when that request fails.

Keep controls touch-friendly and preserve accessible labels/live regions.

## Configuration

See `.env.example`. Important variables are `CONTROLLER_BASE_URL`,
`CONTROLLER_TIMEOUT_MS`, `HOST`, `PORT`, `STATUS_POLL_INTERVAL_MS`,
`MOTION_POLL_INTERVAL_MS`, `MOTION_TIMEOUT_MS`, `AUTO_CALIBRATE`, and
`CALIBRATION_DATA_PATH`. Calibration itself samples at `100 ms` (or
`MOTION_POLL_INTERVAL_MS` when that is smaller), because the gap between two
samples limits how precisely the travel can be measured and that measurement
scales every later move.

Production data must use persistent storage. `compose.yaml` stores
`/data/calibration.json` in the `calibration-data` volume; the NAS project
bind-mounts `/volume1/docker/dacha-temp/data` instead and runs the
container as `1026:100` so that directory stays writable. The image creates
`/data` owned by its `boiler` user, so a fresh named volume is writable too.

The NAS project sets `AUTO_CALIBRATE=false`: creating or restarting a container
must never drive the valves unattended. `../deploy/bob/deploy.sh` refuses to
restart the container while that flag is anything else. See
`../deploy/bob/README.md`.

## Development and verification

```bash
npm ci
cp .env.example .env
npm run dev
```

Required checks before handoff:

```bash
npm test
npm run typecheck
npm run lint
npm run build
```

Use `tests/controller-simulator.ts` and fake timers in tests. It reproduces the
end-stop constants, the mirrored Motor 1 mapping and the poll granularity, and it
also backs `npm run mock:arduino`. Whenever calibration changes, assert the
*physical* opening it produces, not only the counter arithmetic.

Production-style local run:

```bash
CONTROLLER_BASE_URL=http://192.168.88.20 \
AUTO_CALIBRATE=false \
HOST=0.0.0.0 PORT=8080 \
CALIBRATION_DATA_PATH="$PWD/data/calibration.json" \
npm run start
```

Build before `npm run start`. Restarting the app is not permission to calibrate
or move hardware; use read-only checks unless physical motion is requested.

## Coding rules

- TypeScript is strict; preserve client/server/shared boundaries.
- Server ESM relative imports use `.js` suffixes.
- Validate all external input and controller responses.
- Keep state-changing routes narrow and errors structured.
- Preserve unrelated changes and runtime calibration files.
- Prefer focused unit tests before touching real equipment.
