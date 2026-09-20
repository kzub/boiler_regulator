import { mkdtemp } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { describe, expect, it, vi } from 'vitest';
import { ArduinoClient } from '../../src/server/arduino-client.js';
import { CalibrationManager } from '../../src/server/calibration-manager.js';
import { CommandQueue } from '../../src/server/command-queue.js';
import { loadConfig } from '../../src/server/config.js';
import { CalibrationStore } from '../../src/server/calibration-store.js';
import { ControllerSimulator } from '../controller-simulator.js';
import type { CalibrationSnapshot } from '../../src/shared/contracts.js';

/** An ideal scale for the default simulator: 5000 real steps between both homed ends. */
function seed(percent = 50): CalibrationSnapshot {
  const offset = Math.round(percent * 5000 / 100);
  return {
    schemaVersion: 4,
    regulators: {
      // Deployed Motor 1 is mirrored: MIN normalises high, MAX low, counter grows towards MAX.
      '1': {
        minRaw: 9500, maxRaw: 500, travelSteps: 5000, anchorRaw: 9500, anchorPercent: 0,
        lastObservedRaw: 9500 + offset, lastKnownPercent: percent, lastDesiredPercent: null, calibratedAt: '2026-09-13T00:00:00.000Z',
      },
      '2': {
        minRaw: 500, maxRaw: 9500, travelSteps: 5000, anchorRaw: 500, anchorPercent: 0,
        lastObservedRaw: 500 + offset, lastKnownPercent: percent, lastDesiredPercent: null, calibratedAt: '2026-09-13T00:00:00.000Z',
      },
    },
  };
}

async function subject(options: { snapshot?: CalibrationSnapshot | null; at?: number } = {}) {
  const snapshot = options.snapshot === undefined ? seed(options.at ?? 50) : options.snapshot;
  const controller = new ControllerSimulator();
  if (options.at !== undefined || snapshot) {
    controller.placeAt(1, options.at ?? 50);
    controller.placeAt(2, options.at ?? 50);
  }
  const file = join(await mkdtemp(join(tmpdir(), 'boiler-manager-')), 'calibration.json');
  const store = new CalibrationStore(file);
  if (snapshot) await store.save(snapshot);
  const config = loadConfig({
    CONTROLLER_BASE_URL: 'http://mock.local', AUTO_CALIBRATE: 'false', CALIBRATION_DATA_PATH: file,
    MOTION_TIMEOUT_MS: '10000', MOTION_POLL_INTERVAL_MS: '100',
  });
  let requestCount = 0;
  const fetcher: typeof fetch = async (input, init) => { requestCount += 1; return controller.fetcher(input, init); };
  const manager = new CalibrationManager(new ArduinoClient(config, fetcher), store, new CommandQueue(), config, { info() {}, warn() {}, error() {} });
  return { manager, controller, store, requestCount: () => requestCount };
}

async function runCalibration(manager: CalibrationManager): Promise<void> {
  await manager.startCalibration();
  for (let index = 0; index < 400 && manager.calibrationStatus.state === 'running'; index += 1) {
    await vi.advanceTimersByTimeAsync(500);
  }
}

/** Polls like the dashboard does until the backend stops reporting motion. */
async function settle(manager: CalibrationManager): Promise<void> {
  for (let index = 0; index < 80 && manager.isBusy; index += 1) {
    await manager.publicStatus();
    await vi.advanceTimersByTimeAsync(500);
  }
}

describe('CalibrationManager', () => {
  it('accepts a persisted scale while current positions stay inside it', async () => {
    const { manager } = await subject();
    const status = await manager.publicStatus();
    expect(status.calibration.state).toBe('ready');
    expect(status.regulators.floor1.percent).toBe(50);
    expect(status.regulators.floor2.percent).toBe(50);
    expect(status.temperatures.boilerWater).toBe(24.5);
  });

  it('invalidates both regulator percentages on a controller reset', async () => {
    const { manager, controller } = await subject();
    await manager.publicStatus();
    controller.reboot();
    const status = await manager.publicStatus();
    expect(status.calibration.state).toBe('uncalibrated');
    expect(status.regulators.floor1.percent).toBeNull();
    expect(status.regulators.floor2.percent).toBeNull();
  });

  it('requires a new calibration for a snapshot that never measured travel', async () => {
    const { manager } = await subject({ snapshot: null });
    const status = await manager.publicStatus();
    expect(status.calibration.state).toBe('uncalibrated');
    expect(status.calibration.error).toBe('Требуется калибровка');
  });

  it('does not trust the old scale after an interrupted calibration', async () => {
    const { manager, store } = await subject();
    await store.markCalibrationStarted('2026-09-20T07:33:46.905Z');
    const status = await manager.publicStatus();
    expect(status.calibration.state).toBe('uncalibrated');
    expect(status.calibration.error).toBe('Предыдущая калибровка была прервана');
    expect(status.regulators.floor1.percent).toBeNull();
  });

  it.each([1, 2] as const)('opens regulator %i to the requested share of the real travel', async (motor) => {
    const { manager, controller } = await subject({ at: 0 });
    const response = await manager.moveToPercent(motor, 50);
    expect(response.moved).toBe(true);
    // The distance between the two end-stop constants is 9000 steps while the valve
    // only travels 5000, so a scale built from those constants would open it to ~74 %.
    expect(controller.physicalPercent(motor)).toBe(50);
    expect(controller.commands).toEqual([`${motor}:2500`]);
  });

  it('moves the mirrored Motor 1 counter outside its end-stop constants without losing the scale', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller } = await subject({ at: 0 });
      await manager.moveToPercent(1, 80);
      await settle(manager);
      expect(controller.counter(1)).toBe(13500); // above both 9500 and 500
      const status = await manager.publicStatus();
      expect(status.calibration.state).toBe('ready');
      expect(status.regulators.floor1.percent).toBe(80);
    } finally { vi.useRealTimers(); }
  });

  it('does not invalidate the scale while a manual limit command is in motion', async () => {
    const { manager, controller } = await subject({ at: 10 });
    await manager.moveToLimit(1, 'min');
    let left = false;
    for (let index = 0; index < 10 && !left; index += 1) {
      const status = await manager.publicStatus();
      // A raw counter temporarily leaves its scale on the way to an end-stop.
      expect(status.calibration.state).toBe('ready');
      expect(status.regulators.floor1.state).toBe('moving');
      left = controller.counter(1) < 9500;
    }
    expect(left).toBe(true);
  });

  it('re-anchors on a completed manual limit run', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller, store } = await subject({ at: 40 });
      await manager.publicStatus();
      await manager.moveToLimit(2, 'max');
      await settle(manager);

      expect(controller.physicalPercent(2)).toBe(100);
      expect((await manager.publicStatus()).regulators.floor2.percent).toBe(100);
      expect((await store.load())?.regulators['2']).toMatchObject({ anchorRaw: 9500, anchorPercent: 100 });

      await manager.moveToPercent(2, 25);
      expect(controller.physicalPercent(2)).toBe(25);
    } finally { vi.useRealTimers(); }
  });

  it('does not poll Arduino in the background after a manual command', async () => {
    vi.useFakeTimers();
    try {
      const { manager, requestCount } = await subject();
      await manager.moveToLimit(1, 'min');
      const acceptedCount = requestCount();
      await vi.advanceTimersByTimeAsync(2000);
      expect(requestCount()).toBe(acceptedCount);
    } finally { vi.useRealTimers(); }
  });

  it('persists a position changed with the controller encoder', async () => {
    const { manager, controller, store } = await subject({ at: 20 });
    await manager.publicStatus();
    controller.moveLocally(2, 500);
    await manager.publicStatus();
    const saved = await store.load();
    expect(saved?.regulators['2'].lastObservedRaw).toBe(2000);
    expect(saved?.regulators['2'].lastKnownPercent).toBe(30);
  });

  it('measures real travel with two homing runs per regulator', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller, store } = await subject({ snapshot: null, at: 60 });
      await manager.publicStatus();
      await runCalibration(manager);

      expect(manager.calibrationStatus.state).toBe('ready');
      const homingRuns = controller.commands.filter((command) => command.endsWith('min') || command.endsWith('max'));
      expect(homingRuns).toEqual(['1:min', '1:max', '2:min', '2:max']);

      const saved = await store.load();
      // 5000 real steps measured instead of the 9000 between the end-stop constants.
      expect(saved?.regulators['1']).toMatchObject({ minRaw: 9500, maxRaw: 500, travelSteps: 5000 });
      expect(saved?.regulators['2']).toMatchObject({ minRaw: 500, maxRaw: 9500, travelSteps: 5000 });
      expect(controller.usableTravel).toBe(5000);
    } finally { vi.useRealTimers(); }
  });

  it('restores the opening measured by the MIN run when nothing is remembered', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller, store } = await subject({ snapshot: null, at: 40 });
      await manager.publicStatus();
      await runCalibration(manager);

      expect(manager.calibrationStatus.state).toBe('ready');
      // A first-ever calibration must not leave both valves wide open.
      for (const motor of [1, 2] as const) expect(controller.physicalPercent(motor)).toBe(40);
      expect((await store.load())?.regulators['2'].lastKnownPercent).toBe(40);
    } finally { vi.useRealTimers(); }
  });

  it('restores the remembered opening after counters reset and calibration completes', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller, store } = await subject({ at: 30 });
      await manager.publicStatus();
      controller.reboot();
      expect((await manager.publicStatus()).calibration.state).toBe('uncalibrated');

      await runCalibration(manager);

      expect(manager.calibrationStatus.state).toBe('ready');
      for (const motor of [1, 2] as const) expect(controller.physicalPercent(motor)).toBe(30);
      const saved = await store.load();
      expect(saved?.regulators['1'].lastKnownPercent).toBe(30);
      expect(saved?.regulators['2'].lastKnownPercent).toBe(30);
      expect(await store.calibrationWasInterrupted()).toBe(false);
    } finally { vi.useRealTimers(); }
  });

  it('captures fresh counters immediately before a manual calibration and restores them', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller } = await subject({ at: 50 });
      await manager.publicStatus();
      controller.placeAt(1, 75);
      controller.placeAt(2, 20);

      await runCalibration(manager);

      expect(manager.calibrationStatus.state).toBe('ready');
      expect(controller.physicalPercent(1)).toBe(75);
      expect(controller.physicalPercent(2)).toBe(20);
    } finally { vi.useRealTimers(); }
  });

  it('keeps percentages accurate after a fresh calibration', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller } = await subject({ snapshot: null, at: 0 });
      await manager.publicStatus();
      await runCalibration(manager);

      for (const motor of [1, 2] as const) {
        await manager.moveToPercent(motor, 50);
        expect(controller.physicalPercent(motor)).toBe(50);
        await settle(manager);
      }
    } finally { vi.useRealTimers(); }
  });

  it('tolerates two consecutive 503 responses during calibration', async () => {
    vi.useFakeTimers();
    try {
      const { manager, controller } = await subject();
      await manager.publicStatus();
      controller.statusFailures = 2;
      await runCalibration(manager);
      expect(manager.calibrationStatus.state).toBe('ready');
      expect(controller.statusFailures).toBe(0);
    } finally { vi.useRealTimers(); }
  });

  it('serves cached status while calibration owns the controller connection', async () => {
    vi.useFakeTimers();
    try {
      const { manager, requestCount } = await subject();
      await manager.publicStatus();
      await manager.startCalibration();
      await vi.advanceTimersByTimeAsync(0);
      const calibrationRequestCount = requestCount();

      const status = await manager.publicStatus();

      expect(status.calibration.state).toBe('running');
      expect(requestCount()).toBe(calibrationRequestCount);
    } finally { vi.useRealTimers(); }
  });
});
