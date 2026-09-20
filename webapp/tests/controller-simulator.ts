/**
 * Behavioural double for the deployed controller firmware.
 *
 * The important property it reproduces: the firmware does not count the valve
 * travel. It runs into a shared end-stop, backs off `backoffSteps`, and then
 * *writes a constant* into the counter — `backoffSteps` at one end and
 * `positionLimit - backoffSteps` at the other. The distance between those two
 * constants (9000 by default) has nothing to do with the real travel (5000 by
 * default), and deployed Motor 1 is mirrored, so it writes them the other way
 * round while its counter still grows towards MAX.
 */
import type { ArduinoStatus, RegulatorId } from '../src/shared/contracts.js';

export interface SimulatorOptions {
  /** Physical steps between the two end-stop switches. */
  travelSteps?: number;
  /** Clearance the firmware adds after releasing a switch. */
  backoffSteps?: number;
  /** Firmware `MAX_POSITION_LIMIT`. */
  positionLimit?: number;
  /** How far a motor gets between two `/status` reads. */
  stepsPerPoll?: number;
  /** Deployed Motor 1 normalises MIN to the high constant and MAX to the low one. */
  mirroredMotor1?: boolean;
}

interface MotorState {
  /** True physical position in steps, measured from the MIN switch. */
  position: number;
  counter: number;
  queue: number[];
  minConstant: number;
  maxConstant: number;
}

export class ControllerSimulator {
  /** Every accepted `/set` command as `motor:pos`. */
  readonly commands: string[] = [];
  statusFailures = 0;
  temperatures = { fl1: 21.5, fl2: 22, boiler: 24.5, room: 61 };
  private readonly travel: number;
  private readonly backoff: number;
  private readonly stepsPerPoll: number;
  private readonly motors: Record<RegulatorId, MotorState>;

  constructor(options: SimulatorOptions = {}) {
    this.travel = options.travelSteps ?? 6000;
    this.backoff = options.backoffSteps ?? 500;
    this.stepsPerPoll = options.stepsPerPoll ?? 200;
    const low = this.backoff;
    const high = (options.positionLimit ?? 10000) - this.backoff;
    const mirrored = options.mirroredMotor1 ?? true;
    this.motors = {
      1: this.homedAtMin(mirrored ? high : low, mirrored ? low : high),
      2: this.homedAtMin(low, high),
    };
  }

  /** Travel reachable between the two homed positions: what 0..100 % must map to. */
  get usableTravel(): number { return this.travel - 2 * this.backoff; }

  /** True valve opening, independent of whatever the counter currently says. */
  physicalPercent(motor: RegulatorId): number {
    return (this.motors[motor].position - this.backoff) * 100 / this.usableTravel;
  }

  counter(motor: RegulatorId): number { return this.motors[motor].counter; }

  /** Places a valve at an opening as if it had travelled there from its MIN end-stop. */
  placeAt(motor: RegulatorId, percent: number): void {
    const state = this.motors[motor];
    const offset = Math.round(percent * this.usableTravel / 100);
    state.position = this.backoff + offset;
    state.counter = state.minConstant + offset;
    state.queue.length = 0;
  }

  /** A controller restart: AccelStepper starts counting from zero again. */
  reboot(): void {
    for (const motor of [1, 2] as const) {
      this.motors[motor].counter = 0;
      this.motors[motor].queue.length = 0;
    }
  }

  /** A turn of the local encoder: the counter jumps in one observed step. */
  moveLocally(motor: RegulatorId, steps: number): void {
    const state = this.motors[motor];
    this.drive(state, steps);
    state.counter = state.queue.at(-1) ?? state.counter;
    state.queue.length = 0;
  }

  readonly fetcher: typeof fetch = async (input) => {
    const url = new URL(String(input));
    if (url.pathname === '/status') {
      if (this.statusFailures > 0) {
        this.statusFailures -= 1;
        return new Response('', { status: 503 });
      }
      return new Response(JSON.stringify(this.sample()));
    }
    if (url.pathname !== '/set') return new Response('{"error":"bad_req"}');
    const motor: RegulatorId = url.searchParams.get('motor') === '1' ? 1 : 2;
    const pos = url.searchParams.get('pos') ?? '';
    this.commands.push(`${motor}:${pos}`);
    const state = this.motors[motor];
    // A homing command is a safety run: it always reaches its switch.
    if (pos === 'min') this.drive(state, -(this.travel + this.backoff));
    else if (pos === 'max') this.drive(state, this.travel + this.backoff);
    else this.drive(state, Number(pos));
    return new Response('{"status":"ok"}');
  };

  /** One `/status` read reveals the next point of the path the motor is walking. */
  private sample(): ArduinoStatus {
    for (const motor of [1, 2] as const) {
      const state = this.motors[motor];
      if (state.queue.length) state.counter = state.queue.shift()!;
    }
    return { ...this.temperatures, m1: this.motors[1].counter, m2: this.motors[2].counter };
  }

  private drive(state: MotorState, delta: number): void {
    const sign = Math.sign(delta);
    if (!sign) return;
    let counter = state.queue.at(-1) ?? state.counter;
    let remaining = Math.abs(delta);
    while (remaining > 0) {
      const step = Math.min(this.stepsPerPoll, remaining);
      const room = sign > 0 ? this.travel - state.position : state.position;
      if (room <= step) {
        // The switch closed. The firmware releases it, adds clearance and
        // overwrites the counter with its constant for that end.
        state.position = sign > 0 ? this.travel - this.backoff : this.backoff;
        state.queue.push(sign > 0 ? state.maxConstant : state.minConstant);
        return;
      }
      state.position += sign * step;
      counter += sign * step;
      remaining -= step;
      state.queue.push(counter);
    }
  }

  private homedAtMin(minConstant: number, maxConstant: number): MotorState {
    return { position: this.backoff, counter: minConstant, queue: [], minConstant, maxConstant };
  }
}
