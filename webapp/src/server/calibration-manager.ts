import type {
  ArduinoStatus, CalibrationRegulator, CalibrationSnapshot, CalibrationStatus, Limit,
  PublicRegulator, RegulatorId, StatusResponse,
} from '../shared/contracts.js';
import type { Config } from './config.js';
import { ArduinoClient } from './arduino-client.js';
import { CalibrationStore } from './calibration-store.js';
import { CommandQueue } from './command-queue.js';
import { ControllerError } from './errors.js';
import { ControllerJournal } from './controller-journal.js';

const MIN_TRAVEL_STEPS = 100;
const STABLE_MS = 1500;
const MIN_OBSERVATION_MS = 2000;
const STABLE_SAMPLES = 5;
const TRANSIENT_ERROR_RETRIES = 2;
/**
 * Calibration owns the controller connection, so it can sample more finely than
 * dashboard polling. The gap between two samples is what limits how precisely the
 * travel can be measured, and that measurement scales every later move.
 */
const CALIBRATION_POLL_INTERVAL_MS = 100;
/**
 * How far outside 0..100 % a counter may sit before its scale is treated as lost.
 * It has to absorb the travel measurement error, and stay well below the distance
 * between an end-stop constant and zero, so that a restarted controller (counter
 * back at 0) is still recognised as out of scale.
 */
const PERCENT_TOLERANCE = 5;

interface ActiveMotion {
  motor: RegulatorId;
  targetPercent: number | null;
  expectedRaw: number | null;
  startedAt: number;
  lastRaw: number;
  stableSince: number;
  stableSamples: number;
}

interface PhaseResult {
  finalRaw: number;
  /** Peak travel observed before the counter settled on its end-stop constant. */
  excursion: number;
  /** Distance covered between the last two travel samples. */
  sampleStep: number;
}

const pause = (milliseconds: number) => new Promise<void>((resolve) => setTimeout(resolve, milliseconds));
const idKey = (id: RegulatorId) => String(id) as '1' | '2';
const clampPercent = (value: number) => Math.max(0, Math.min(100, Math.round(value)));
const motorName = (motor: RegulatorId) => (motor === 1 ? 'первый' : 'второй');

export class CalibrationManager {
  private snapshot: CalibrationSnapshot | null = null;
  private initialized = false;
  private calibration: CalibrationStatus = { state: 'waiting_for_controller', phase: null, progressPercent: 0, startedAt: null, error: null };
  private activeMotion: ActiveMotion | null = null;
  private lastStatus: ArduinoStatus | null = null;
  private interruptedCalibration = false;

  constructor(
    private readonly client: ArduinoClient,
    private readonly store: CalibrationStore,
    private readonly queue: CommandQueue,
    private readonly config: Config,
    private readonly log: Pick<Console, 'info' | 'warn' | 'error'> = console,
    private readonly journal?: ControllerJournal,
  ) {}

  async initialize(): Promise<void> {
    if (this.initialized) return;
    this.initialized = true;
    this.snapshot = await this.store.load();
    this.interruptedCalibration = await this.store.calibrationWasInterrupted();
    try {
      const status = await this.client.status();
      this.lastStatus = status;
      if (this.interruptedCalibration) {
        this.invalidate('Предыдущая калибровка была прервана', false);
      } else if (this.snapshot && this.positionsFit(status)) {
        this.calibration = { state: 'ready', phase: null, progressPercent: 100, startedAt: null, error: null };
        await this.rememberStatus(status);
      } else {
        this.invalidate('Требуется калибровка', this.config.autoCalibrate);
      }
    } catch {
      this.calibration = { state: 'waiting_for_controller', phase: null, progressPercent: 0, startedAt: null, error: null };
    }
  }

  get calibrationStatus(): CalibrationStatus { return this.calibration; }
  get isBusy(): boolean { return this.queue.busy || this.activeMotion !== null; }

  async publicStatus(): Promise<StatusResponse> {
    await this.initialize();
    // Calibration already polls the single-request Arduino HTTP server. Reuse its
    // latest sample so dashboard polling cannot collide with the calibration job.
    if (this.calibration.state === 'running' && this.lastStatus) return this.toPublicStatus(this.lastStatus);
    const status = await this.client.status();
    this.lastStatus = status;
    await this.observe(status);
    return this.toPublicStatus(status);
  }

  async calibrationStatusResponse(): Promise<CalibrationStatus> { await this.initialize(); return this.calibration; }

  async startCalibration(manual = true): Promise<CalibrationStatus> {
    await this.initialize();
    if (this.calibration.state === 'running') throw new Error('CALIBRATION_IN_PROGRESS');
    if (this.isBusy) throw new Error('CONTROLLER_BUSY');
    const trustCurrentPosition = !this.interruptedCalibration;
    const startedAt = new Date().toISOString();
    await this.store.markCalibrationStarted(startedAt);
    this.interruptedCalibration = true;
    this.calibration = {
      state: 'running', phase: 'motor1_min', progressPercent: 0,
      startedAt, error: null,
    };
    this.runCalibration(trustCurrentPosition).catch((error: unknown) => {
      const message = error instanceof Error ? error.message : 'Calibration failed';
      this.calibration = { ...this.calibration, state: 'failed', error: message, phase: null };
      this.log.error({ event: 'calibration_failed', message });
      this.journal?.event('calibration_failed', { message, lastStatus: this.lastStatus }, false);
    });
    if (manual) {
      this.log.info({ event: 'calibration_started', source: 'manual' });
      this.journal?.event('calibration_started', { source: 'manual' });
    }
    return this.calibration;
  }

  async moveToLimit(motor: RegulatorId, limit: Limit): Promise<void> {
    await this.initialize();
    if (this.calibration.state === 'running') throw new Error('CALIBRATION_IN_PROGRESS');
    if (this.isBusy) throw new Error('CONTROLLER_BUSY');
    await this.queue.run(async () => this.client.limit(motor, limit));
    this.beginMotion(motor, null, null);
    this.log.info({ event: 'limit_command', motor, limit, result: 'accepted' });
  }

  async moveToPercent(motor: RegulatorId, percent: number): Promise<{ moved: boolean; position: number }> {
    await this.initialize();
    if (this.calibration.state === 'running') throw new Error('CALIBRATION_IN_PROGRESS');
    if (this.calibration.state !== 'ready' || !this.snapshot) throw new Error('CALIBRATION_REQUIRED');
    if (this.isBusy) throw new Error('CONTROLLER_BUSY');
    const fresh = await this.client.status();
    this.lastStatus = fresh;
    if (!this.positionsFit(fresh)) {
      this.invalidate('Позиция вышла за пределы шкалы', this.config.autoCalibrate);
      throw new Error('CALIBRATION_REQUIRED');
    }
    const current = motor === 1 ? fresh.m1 : fresh.m2;
    const scale = this.snapshot.regulators[idKey(motor)];
    // Adopt first: a counter resting on an end-stop constant moves the anchor.
    if (this.remember(scale, current)) await this.store.save(this.snapshot);
    const targetRaw = this.rawAt(scale, percent);
    const delta = targetRaw - current;
    if (delta === 0) {
      scale.lastDesiredPercent = percent;
      await this.store.save(this.snapshot);
      return { moved: false, position: current };
    }
    await this.queue.run(async () => this.client.relative(motor, delta));
    scale.lastDesiredPercent = percent;
    await this.store.save(this.snapshot);
    this.beginMotion(motor, percent, targetRaw);
    this.log.info({ event: 'target_command', motor, percent, delta, result: 'accepted' });
    this.journal?.event('target_command', {
      motor, percent, current, currentPercent: Math.round(this.percentAt(scale, current) * 10) / 10,
      targetRaw, delta, travelSteps: scale.travelSteps,
    });
    return { moved: true, position: current };
  }

  private invalidate(reason: string, autoStart: boolean): void {
    this.activeMotion = null;
    this.calibration = { state: 'uncalibrated', phase: null, progressPercent: 0, startedAt: null, error: reason };
    if (autoStart) void this.startCalibration(false).catch(() => undefined);
  }

  // --- Scale arithmetic -----------------------------------------------------
  // The firmware writes a fixed constant into the counter at each end-stop
  // (BACKOFF_STEPS at one end, MAX_POSITION_LIMIT - BACKOFF_STEPS at the other).
  // The distance between those constants is a firmware convention, not the valve
  // travel, so every percentage is derived from measured steps counted from the
  // last trusted anchor instead.

  private percentAt(scale: CalibrationRegulator, raw: number): number {
    return scale.anchorPercent + (raw - scale.anchorRaw) * 100 / scale.travelSteps;
  }

  private rawAt(scale: CalibrationRegulator, percent: number): number {
    return Math.round(scale.anchorRaw + (percent - scale.anchorPercent) * scale.travelSteps / 100);
  }

  /** Homing normalises the counter to a known constant, which re-establishes the anchor. */
  private endpointPercent(scale: CalibrationRegulator, raw: number): number | null {
    if (raw === scale.minRaw) return 0;
    if (raw === scale.maxRaw) return 100;
    return null;
  }

  private accepts(scale: CalibrationRegulator, raw: number): boolean {
    if (this.endpointPercent(scale, raw) !== null) return true;
    const percent = this.percentAt(scale, raw);
    return percent >= -PERCENT_TOLERANCE && percent <= 100 + PERCENT_TOLERANCE;
  }

  /** Stores the observed position, re-anchoring on an end-stop constant. Returns true when anything changed. */
  private remember(scale: CalibrationRegulator, raw: number): boolean {
    const endpoint = this.endpointPercent(scale, raw);
    let changed = false;
    if (endpoint !== null && (scale.anchorRaw !== raw || scale.anchorPercent !== endpoint)) {
      scale.anchorRaw = raw;
      scale.anchorPercent = endpoint;
      changed = true;
    }
    const percent = clampPercent(this.percentAt(scale, raw));
    if (scale.lastObservedRaw !== raw || scale.lastKnownPercent !== percent) {
      scale.lastObservedRaw = raw;
      scale.lastKnownPercent = percent;
      changed = true;
    }
    return changed;
  }

  private positionsFit(status: ArduinoStatus): boolean {
    if (!this.snapshot) return false;
    return ([1, 2] as const).every((motor) => {
      const value = motor === 1 ? status.m1 : status.m2;
      return this.accepts(this.snapshot!.regulators[idKey(motor)], value);
    });
  }

  private async observe(status: ArduinoStatus): Promise<void> {
    // A limit command intentionally drives raw position beyond the calibrated range
    // until its physical switch is found. It is not a reset or a reason to recalibrate.
    if (this.activeMotion) {
      await this.updateMotion(status);
      return;
    }
    if (this.calibration.state === 'ready' && !this.positionsFit(status)) {
      this.invalidate('Контроллер перезапущен или позиция вне шкалы', this.config.autoCalibrate);
      return;
    }
    if (this.snapshot && this.calibration.state === 'ready') {
      await this.rememberStatus(status);
    }
  }

  private beginMotion(motor: RegulatorId, targetPercent: number | null, expectedRaw: number | null): void {
    const raw = this.lastStatus ? (motor === 1 ? this.lastStatus.m1 : this.lastStatus.m2) : 0;
    this.activeMotion = { motor, targetPercent, expectedRaw, startedAt: Date.now(), lastRaw: raw, stableSince: Date.now(), stableSamples: 0 };
    // Completion is observed through browser-originated /api/status polling.
    // No active browser means no background status traffic to the Arduino.
  }

  private async updateMotion(status: ArduinoStatus): Promise<void> {
    const motion = this.activeMotion;
    if (!motion) return;
    const raw = motion.motor === 1 ? status.m1 : status.m2;
    const now = Date.now();
    if (raw !== motion.lastRaw) {
      motion.lastRaw = raw;
      motion.stableSince = now;
      motion.stableSamples = 0;
    } else motion.stableSamples += 1;
    if (now - motion.startedAt > this.config.motionTimeoutMs) {
      this.activeMotion = null;
      this.log.warn({ event: 'motion_timeout', motor: motion.motor });
      return;
    }
    if (now - motion.startedAt < MIN_OBSERVATION_MS || now - motion.stableSince < STABLE_MS || motion.stableSamples < STABLE_SAMPLES) return;
    if (motion.expectedRaw !== null && Math.abs(raw - motion.expectedRaw) > this.tolerance(motion.motor)) {
      this.log.warn({ event: 'motion_target_mismatch', motor: motion.motor, expected: motion.expectedRaw, actual: raw });
      this.journal?.event('motion_target_mismatch', { motor: motion.motor, expected: motion.expectedRaw, actual: raw }, false);
    }
    if (this.snapshot) {
      // A move that ended on an end-stop constant re-anchors here. One that ended
      // outside the scale is caught by the next observation, which invalidates it.
      const scale = this.snapshot.regulators[idKey(motion.motor)];
      if (this.accepts(scale, raw) && this.remember(scale, raw)) await this.store.save(this.snapshot);
    }
    this.activeMotion = null;
  }

  private async rememberStatus(status: ArduinoStatus): Promise<void> {
    if (!this.snapshot) return;
    let changed = false;
    for (const motor of [1, 2] as const) {
      const raw = motor === 1 ? status.m1 : status.m2;
      if (this.remember(this.snapshot.regulators[idKey(motor)], raw)) changed = true;
    }
    if (changed) await this.store.save(this.snapshot);
  }

  private rememberedPercent(snapshot: CalibrationSnapshot | null, status: ArduinoStatus, motor: RegulatorId): number | null {
    if (!snapshot) return null;
    const scale = snapshot.regulators[idKey(motor)];
    const currentRaw = motor === 1 ? status.m1 : status.m2;
    if (this.accepts(scale, currentRaw)) return clampPercent(this.percentAt(scale, currentRaw));
    return scale.lastKnownPercent;
  }

  private tolerance(motor: RegulatorId): number {
    const travel = this.snapshot?.regulators[idKey(motor)].travelSteps;
    return travel ? Math.max(1, Math.round(travel * 0.01)) : 1;
  }

  private async calibrationStatusRead(): Promise<ArduinoStatus> {
    let consecutiveErrors = 0;
    while (true) {
      try { return await this.client.status(); }
      catch (error) {
        if (!(error instanceof ControllerError) || consecutiveErrors >= TRANSIENT_ERROR_RETRIES) throw error;
        consecutiveErrors += 1;
        this.log.warn({ event: 'calibration_status_retry', consecutiveErrors });
        this.journal?.event('calibration_status_retry', { consecutiveErrors, code: error.code }, false);
        await pause(this.config.motionPollIntervalMs);
      }
    }
  }

  private async runCalibration(trustCurrentPosition: boolean): Promise<void> {
    await this.queue.run(async () => {
      const before = await this.calibrationStatusRead();
      this.lastStatus = before;
      const previousSnapshot = this.snapshot;
      const startingRaw = { '1': before.m1, '2': before.m2 };
      const restorePercent: Record<'1' | '2', number | null> = {
        '1': trustCurrentPosition ? this.rememberedPercent(previousSnapshot, before, 1) : previousSnapshot?.regulators['1'].lastKnownPercent ?? null,
        '2': trustCurrentPosition ? this.rememberedPercent(previousSnapshot, before, 2) : previousSnapshot?.regulators['2'].lastKnownPercent ?? null,
      };
      this.journal?.event('calibration_initial_state', {
        startingRaw, restorePercent,
        previousScale: previousSnapshot ? {
          '1': { minRaw: previousSnapshot.regulators['1'].minRaw, maxRaw: previousSnapshot.regulators['1'].maxRaw, travelSteps: previousSnapshot.regulators['1'].travelSteps },
          '2': { minRaw: previousSnapshot.regulators['2'].minRaw, maxRaw: previousSnapshot.regulators['2'].maxRaw, travelSteps: previousSnapshot.regulators['2'].travelSteps },
        } : null,
      });
      if (previousSnapshot && trustCurrentPosition) {
        let changed = false;
        for (const motor of [1, 2] as const) {
          const scale = previousSnapshot.regulators[idKey(motor)];
          const raw = startingRaw[idKey(motor)];
          // An out-of-scale counter is the reset signal. Keep the last valid
          // persisted opening for that motor instead of overwriting it.
          if (this.accepts(scale, raw) && this.remember(scale, raw)) changed = true;
        }
        if (changed) await this.store.save(previousSnapshot);
      }
      this.activeMotion = null;
      const now = new Date().toISOString();
      const calibrated = {} as CalibrationSnapshot['regulators'];
      for (const motor of [1, 2] as const) {
        const key = idKey(motor);
        const baseProgress = motor === 1 ? 0 : 50;
        this.calibration = { ...this.calibration, phase: motor === 1 ? 'motor1_min' : 'motor2_min', progressPercent: baseProgress };
        const min = await this.calibratePhase(motor, 'min');
        this.journal?.event('calibration_boundary', { motor, limit: 'min', finalRaw: min.finalRaw, excursion: min.excursion });

        this.calibration = { ...this.calibration, phase: motor === 1 ? 'motor1_max' : 'motor2_max', progressPercent: baseProgress + 20 };
        const max = await this.calibratePhase(motor, 'max');
        this.journal?.event('calibration_boundary', { motor, limit: 'max', finalRaw: max.finalRaw, excursion: max.excursion });

        const scale = this.buildScale(motor, min, max, now);
        calibrated[key] = scale;

        // Without a trusted persisted opening, the MIN run itself says where the
        // valve stood: it measured the distance from there down to the switch.
        const measuredStart = clampPercent((this.measuredSteps(min) - this.backoffSteps(min, max)) * 100 / scale.travelSteps);
        const targetPercent = restorePercent[key] ?? measuredStart;
        this.journal?.event('calibration_restore_target', { motor, targetPercent, measuredStart, remembered: restorePercent[key] });
        this.calibration = {
          ...this.calibration,
          phase: motor === 1 ? 'motor1_restore' : 'motor2_restore',
          progressPercent: baseProgress + 35,
        };
        const finalRaw = await this.restorePosition(motor, targetPercent, scale);
        this.remember(scale, finalRaw);
        scale.lastDesiredPercent = targetPercent;
      }
      const nextSnapshot: CalibrationSnapshot = { schemaVersion: 4, regulators: calibrated };
      this.snapshot = nextSnapshot;
      this.journal?.event('calibration_scales', {
        motor1: nextSnapshot.regulators['1'], motor2: nextSnapshot.regulators['2'],
      });
      await this.store.save(nextSnapshot);
      await this.store.markCalibrationCompleted();
      this.interruptedCalibration = false;
      this.calibration = { state: 'ready', phase: null, progressPercent: 100, startedAt: null, error: null };
      this.log.info({ event: 'calibration_completed', restoredPercent: restorePercent });
      this.journal?.event('calibration_completed', {
        restoredPercent: {
          '1': nextSnapshot.regulators['1'].lastKnownPercent,
          '2': nextSnapshot.regulators['2'].lastKnownPercent,
        },
        finalRaw: {
          '1': nextSnapshot.regulators['1'].lastObservedRaw,
          '2': nextSnapshot.regulators['2'].lastObservedRaw,
        },
      });
    });
  }

  /**
   * MIN and MAX homing give the two counter constants; the MAX run additionally
   * measures how many steps the valve really travels between them. The anchor is
   * left at MAX, so the remaining restore move is one relative command and no
   * third homing run is needed.
   */
  private buildScale(motor: RegulatorId, min: PhaseResult, max: PhaseResult, calibratedAt: string): CalibrationRegulator {
    if (min.finalRaw === max.finalRaw) throw new Error(`Не удалось подтвердить границы: ${motorName(motor)} регулятор`);
    const backoffSteps = this.backoffSteps(min, max);
    const travelSteps = Math.round(this.measuredSteps(max) - backoffSteps);
    this.journal?.event('calibration_travel_measured', {
      motor, minRaw: min.finalRaw, maxRaw: max.finalRaw, excursion: max.excursion,
      sampleStep: max.sampleStep, backoffSteps, travelSteps,
    });
    if (travelSteps < MIN_TRAVEL_STEPS) throw new Error(`Не удалось измерить ход: ${motorName(motor)} регулятор`);
    return {
      minRaw: min.finalRaw, maxRaw: max.finalRaw, travelSteps,
      anchorRaw: max.finalRaw, anchorPercent: 100,
      lastObservedRaw: max.finalRaw, lastKnownPercent: 100, lastDesiredPercent: null, calibratedAt,
    };
  }

  /**
   * Clearance the firmware adds after releasing a switch. It writes BACKOFF_STEPS at
   * one end-stop and MAX_POSITION_LIMIT - BACKOFF_STEPS at the other, so the smaller
   * of the two observed constants is that clearance. It is not usable travel.
   */
  private backoffSteps(min: PhaseResult, max: PhaseResult): number {
    return Math.max(0, Math.min(min.finalRaw, max.finalRaw));
  }

  /**
   * Steps a homing run really covered. The switch closes between two status samples
   * and the firmware overwrites the counter before the next one, so the last step is
   * never observed to its end; half of the sampling granularity is the expected
   * unobserved remainder.
   */
  private measuredSteps(phase: PhaseResult): number {
    return phase.excursion + Math.min(phase.sampleStep, phase.excursion) / 2;
  }

  private async calibratePhase(motor: RegulatorId, limit: Limit): Promise<PhaseResult> {
    const before = await this.calibrationStatusRead();
    const startingRaw = motor === 1 ? before.m1 : before.m2;
    this.journal?.event('calibration_phase_started', { motor, limit, startingRaw });
    await this.client.limit(motor, limit); // A control command is intentionally sent once only.
    const result = await this.waitForStop(motor, startingRaw, `Тайм-аут калибровки: ${motorName(motor)} регулятор, ${limit.toUpperCase()}`, limit === 'min' ? -1 : 1);
    this.journal?.event('calibration_phase_completed', { motor, limit, startingRaw, ...result });
    return result;
  }

  private async restorePosition(motor: RegulatorId, percent: number, scale: CalibrationRegulator): Promise<number> {
    const before = await this.calibrationStatusRead();
    const startingRaw = motor === 1 ? before.m1 : before.m2;
    const targetRaw = this.rawAt(scale, percent);
    const delta = targetRaw - startingRaw;
    const tolerance = Math.max(1, Math.round(scale.travelSteps * 0.02));
    this.journal?.event('calibration_restore_started', {
      motor, percent, startingRaw, targetRaw, delta, travelSteps: scale.travelSteps, tolerance,
    });
    if (delta === 0) return startingRaw;
    await this.client.relative(motor, delta);
    const finalRaw = (await this.waitForStop(motor, startingRaw, `Тайм-аут возврата: ${motorName(motor)} регулятор`, Math.sign(delta) as -1 | 0 | 1)).finalRaw;
    if (Math.abs(finalRaw - targetRaw) > tolerance) {
      this.journal?.event('calibration_restore_mismatch', { motor, percent, startingRaw, targetRaw, delta, finalRaw, tolerance }, false);
      throw new Error(`Не удалось вернуть ${motorName(motor)} регулятор в прежнее положение`);
    }
    this.journal?.event('calibration_restore_completed', { motor, percent, startingRaw, targetRaw, delta, finalRaw });
    return finalRaw;
  }

  private async waitForStop(motor: RegulatorId, startingRaw: number, timeoutMessage: string, direction: -1 | 0 | 1): Promise<PhaseResult> {
    const directed = (raw: number) => (direction > 0 ? raw - startingRaw : direction < 0 ? startingRaw - raw : Math.abs(raw - startingRaw));
    let lastRaw = startingRaw; let stableSince = Date.now(); let stableSamples = 0;
    let excursion = 0; let sampleStep = 0; let pendingStep = 0;
    const startedAt = Date.now();
    const interval = Math.min(this.config.motionPollIntervalMs, CALIBRATION_POLL_INTERVAL_MS);
    while (Date.now() - startedAt <= this.config.motionTimeoutMs) {
      await pause(interval);
      let current: ArduinoStatus;
      try { current = await this.client.status(); }
      catch (error) {
        // The firmware can briefly stop serving HTTP while clearing a shared end-stop.
        // Keep watching until the phase-wide timeout rather than treating one missed poll as a failed homing move.
        if (error instanceof ControllerError) {
          this.journal?.event('calibration_poll_error', { motor, code: error.code, lastRaw }, false);
          continue;
        }
        throw error;
      }
      this.lastStatus = current;
      const raw = motor === 1 ? current.m1 : current.m2;
      if (raw !== lastRaw) {
        // Only a value the motor has already left counts as travel. The value it
        // finally rests on is the constant the firmware writes at the end-stop, and
        // that constant may be far beyond the position actually reached.
        excursion = Math.max(excursion, directed(lastRaw));
        sampleStep = pendingStep;
        pendingStep = Math.abs(raw - lastRaw);
        lastRaw = raw; stableSince = Date.now(); stableSamples = 0;
      } else stableSamples += 1;
      const elapsed = Date.now() - startedAt;
      if (elapsed >= MIN_OBSERVATION_MS && Date.now() - stableSince >= STABLE_MS && stableSamples >= STABLE_SAMPLES) return { finalRaw: raw, excursion, sampleStep };
    }
    throw new Error(timeoutMessage);
  }

  private toPublicStatus(status: ArduinoStatus): StatusResponse {
    const warnings: string[] = [];
    const temperatures = {
      floor1: cleanTemperature(status.fl1, 'Первый этаж', warnings),
      floor2: cleanTemperature(status.fl2, 'Второй этаж', warnings),
      boilerWater: cleanTemperature(status.boiler, 'Температура воды котла', warnings),
      boilerRoom: cleanTemperature(status.room, 'Бойлерная комната', warnings),
    };
    return {
      temperatures,
      regulators: {
        floor1: this.regulator(1, status.m1),
        floor2: this.regulator(2, status.m2),
      },
      controller: { online: true }, calibration: this.calibration, warnings, receivedAt: new Date().toISOString(),
    };
  }

  private regulator(motor: RegulatorId, raw: number): PublicRegulator {
    const active = this.activeMotion?.motor === motor ? this.activeMotion : null;
    const scale = this.snapshot?.regulators[idKey(motor)];
    if (!scale || this.calibration.state !== 'ready') {
      return { position: raw, percent: null, targetPercent: active?.targetPercent ?? null, state: 'uncalibrated' };
    }
    return {
      position: raw, percent: clampPercent(this.percentAt(scale, raw)),
      targetPercent: active?.targetPercent ?? null, state: active ? 'moving' : 'ready',
    };
  }
}

function cleanTemperature(value: number, name: string, warnings: string[]): number | null {
  if (!Number.isFinite(value) || value === -127 || value === 85 || value < -55 || value > 125) {
    warnings.push(`Датчик «${name}» не передал достоверное значение`);
    return null;
  }
  return value;
}
