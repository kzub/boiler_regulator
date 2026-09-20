export type RegulatorId = 1 | 2;
export type Limit = 'min' | 'max';
export type CalibrationState = 'waiting_for_controller' | 'uncalibrated' | 'running' | 'ready' | 'failed';
export type RegulatorState = 'ready' | 'moving' | 'uncalibrated' | 'error';

export interface Temperatures {
  floor1: number | null;
  floor2: number | null;
  boilerWater: number | null;
  boilerRoom: number | null;
}

export interface PublicRegulator {
  position: number | null;
  percent: number | null;
  targetPercent: number | null;
  state: RegulatorState;
}

export interface StatusResponse {
  temperatures: Temperatures;
  regulators: { floor1: PublicRegulator; floor2: PublicRegulator };
  controller: { online: boolean };
  calibration: CalibrationStatus;
  warnings: string[];
  receivedAt: string;
}

export interface CalibrationStatus {
  state: CalibrationState;
  phase: CalibrationPhase | null;
  progressPercent: number;
  startedAt: string | null;
  error: string | null;
}

export type CalibrationPhase =
  | 'motor1_min' | 'motor1_max' | 'motor2_min' | 'motor2_max'
  | 'motor1_restore' | 'motor2_restore';

export interface BuildInfo {
  /** `package.json` version of the running build. */
  version: string;
  /** When the build was produced, or null when the server runs from sources. */
  builtAt: string | null;
  commit: string | null;
}

export interface ApiError {
  error: { code: string; message: string };
}

export interface CalibrationRegulator {
  /** Counter value the firmware writes when a MIN homing finishes. */
  minRaw: number;
  /** Counter value the firmware writes when a MAX homing finishes. */
  maxRaw: number;
  /** Measured motor steps between the homed MIN and MAX positions. */
  travelSteps: number;
  /** Counter value of the last trusted reference point. */
  anchorRaw: number;
  /** Opening percentage that belonged to `anchorRaw`. */
  anchorPercent: number;
  lastObservedRaw: number;
  lastKnownPercent: number;
  lastDesiredPercent: number | null;
  calibratedAt: string;
}

export interface CalibrationSnapshot {
  schemaVersion: 4;
  regulators: Record<'1' | '2', CalibrationRegulator>;
}

export interface ArduinoStatus {
  fl1: number;
  fl2: number;
  boiler: number;
  room: number;
  m1: number;
  m2: number;
}
