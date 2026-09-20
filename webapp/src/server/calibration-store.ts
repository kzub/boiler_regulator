import { mkdir, readFile, rename, unlink, writeFile } from 'node:fs/promises';
import { dirname } from 'node:path';
import type { CalibrationSnapshot } from '../shared/contracts.js';

function finiteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

function validPercent(value: unknown): value is number {
  return typeof value === 'number' && Number.isInteger(value) && value >= 0 && value <= 100;
}

function isRegulator(value: unknown): boolean {
  if (!value || typeof value !== 'object') return false;
  const item = value as Record<string, unknown>;
  return ['minRaw', 'maxRaw', 'anchorRaw', 'lastObservedRaw'].every((key) => finiteNumber(item[key]))
    && Number.isInteger(item.travelSteps) && (item.travelSteps as number) > 0
    && validPercent(item.anchorPercent) && validPercent(item.lastKnownPercent)
    && (item.lastDesiredPercent === null || validPercent(item.lastDesiredPercent))
    && typeof item.calibratedAt === 'string';
}

/**
 * Only schema 4 is trusted. Earlier snapshots derived percentages from the two
 * end-stop constants the firmware writes into the counter, which are unrelated to
 * the physical travel. Such a scale cannot be repaired without moving the valve,
 * so an old file is rejected and a new calibration is required.
 */
export function isSnapshot(value: unknown): value is CalibrationSnapshot {
  if (!value || typeof value !== 'object') return false;
  const snapshot = value as Record<string, unknown>;
  if (snapshot.schemaVersion !== 4 || !snapshot.regulators || typeof snapshot.regulators !== 'object') return false;
  const regulators = snapshot.regulators as Record<string, unknown>;
  return (['1', '2'] as const).every((key) => isRegulator(regulators[key]));
}

export class CalibrationStore {
  constructor(private readonly filePath: string) {}

  private get markerPath(): string { return `${this.filePath}.in-progress`; }

  async load(): Promise<CalibrationSnapshot | null> {
    try {
      const parsed: unknown = JSON.parse(await readFile(this.filePath, 'utf8'));
      return isSnapshot(parsed) ? parsed : null;
    } catch { return null; }
  }

  async save(snapshot: CalibrationSnapshot): Promise<void> {
    await mkdir(dirname(this.filePath), { recursive: true });
    const temporary = `${this.filePath}.${process.pid}.tmp`;
    await writeFile(temporary, JSON.stringify(snapshot), { encoding: 'utf8', mode: 0o600 });
    await rename(temporary, this.filePath);
  }

  async calibrationWasInterrupted(): Promise<boolean> {
    try { await readFile(this.markerPath, 'utf8'); return true; }
    catch { return false; }
  }

  async markCalibrationStarted(startedAt: string): Promise<void> {
    await mkdir(dirname(this.filePath), { recursive: true });
    await writeFile(this.markerPath, JSON.stringify({ startedAt }), { encoding: 'utf8', mode: 0o600 });
  }

  async markCalibrationCompleted(): Promise<void> {
    try { await unlink(this.markerPath); }
    catch (error) {
      if (!(error instanceof Error && 'code' in error && error.code === 'ENOENT')) throw error;
    }
  }
}
