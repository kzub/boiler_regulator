import { mkdtemp, readFile, writeFile } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { CalibrationStore } from '../../src/server/calibration-store.js';
import type { CalibrationSnapshot } from '../../src/shared/contracts.js';

const snapshot: CalibrationSnapshot = {
  schemaVersion: 4,
  regulators: {
    '1': {
      minRaw: 9500, maxRaw: 500, travelSteps: 5000, anchorRaw: 9500, anchorPercent: 0,
      lastObservedRaw: 12000, lastKnownPercent: 50, lastDesiredPercent: 50, calibratedAt: '2026-09-13T00:00:00.000Z',
    },
    '2': {
      minRaw: 500, maxRaw: 9500, travelSteps: 5000, anchorRaw: 9500, anchorPercent: 100,
      lastObservedRaw: 7500, lastKnownPercent: 60, lastDesiredPercent: null, calibratedAt: '2026-09-13T00:00:00.000Z',
    },
  },
};

describe('CalibrationStore', () => {
  it('writes a versioned snapshot and restores it', async () => {
    const folder = await mkdtemp(join(tmpdir(), 'boiler-test-'));
    const file = join(folder, 'state', 'calibration.json');
    const store = new CalibrationStore(file);
    await store.save(snapshot);
    expect(JSON.parse(await readFile(file, 'utf8'))).toEqual(snapshot);
    expect(await store.load()).toEqual(snapshot);
  });

  it('does not trust malformed persisted data', async () => {
    const folder = await mkdtemp(join(tmpdir(), 'boiler-test-'));
    const store = new CalibrationStore(join(folder, 'calibration.json'));
    expect(await store.load()).toBeNull();
  });

  it('persists an interrupted calibration marker until completion', async () => {
    const folder = await mkdtemp(join(tmpdir(), 'boiler-test-'));
    const store = new CalibrationStore(join(folder, 'calibration.json'));
    expect(await store.calibrationWasInterrupted()).toBe(false);
    await store.markCalibrationStarted('2026-09-20T07:33:46.905Z');
    expect(await store.calibrationWasInterrupted()).toBe(true);
    await store.markCalibrationCompleted();
    expect(await store.calibrationWasInterrupted()).toBe(false);
  });

  it('rejects an older snapshot whose scale was never measured', async () => {
    const folder = await mkdtemp(join(tmpdir(), 'boiler-test-'));
    const file = join(folder, 'calibration.json');
    // Schema 3 derived percentages from the two end-stop constants, which are not
    // the valve travel. Such a scale cannot be repaired without moving the valve.
    const legacy = {
      schemaVersion: 3,
      regulators: {
        '1': { minRaw: 9500, maxRaw: 500, counterMode: 'wrapped', travelSteps: 6003, lastObservedRaw: 12441, lastKnownPercent: 49, lastDesiredPercent: 49, calibratedAt: '2026-09-20T07:59:18.901Z' },
        '2': { minRaw: 500, maxRaw: 9500, lastObservedRaw: 500, lastKnownPercent: 0, lastDesiredPercent: 0, calibratedAt: '2026-09-20T07:59:18.901Z' },
      },
    };
    await writeFile(file, JSON.stringify(legacy));
    expect(await new CalibrationStore(file).load()).toBeNull();
  });

  it('rejects a snapshot without a measured travel', async () => {
    const folder = await mkdtemp(join(tmpdir(), 'boiler-test-'));
    const file = join(folder, 'calibration.json');
    const incomplete: Record<string, unknown> = { ...snapshot.regulators['1'] };
    delete incomplete.travelSteps;
    await writeFile(file, JSON.stringify({ ...snapshot, regulators: { ...snapshot.regulators, '1': incomplete } }));
    expect(await new CalibrationStore(file).load()).toBeNull();
  });
});
