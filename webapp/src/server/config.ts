import path from 'node:path';

export interface Config {
  controllerBaseUrl: URL;
  controllerTimeoutMs: number;
  port: number;
  host: string;
  statusPollIntervalMs: number;
  motionPollIntervalMs: number;
  motionTimeoutMs: number;
  autoCalibrate: boolean;
  calibrationDataPath: string;
}

function requiredUrl(value: string | undefined): URL {
  if (!value) throw new Error('CONTROLLER_BASE_URL is required');
  let url: URL;
  try { url = new URL(value); } catch { throw new Error('CONTROLLER_BASE_URL must be a valid URL'); }
  if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash) {
    throw new Error('CONTROLLER_BASE_URL allows only a plain http(s) origin');
  }
  return url;
}

function positiveInt(name: string, value: string | undefined, fallback: number): number {
  const parsed = Number(value ?? fallback);
  if (!Number.isInteger(parsed) || parsed <= 0) throw new Error(`${name} must be a positive integer`);
  return parsed;
}

export function loadConfig(env = process.env): Config {
  const calibrationDataPath = env.CALIBRATION_DATA_PATH || path.resolve('data/calibration.json');
  return {
    controllerBaseUrl: requiredUrl(env.CONTROLLER_BASE_URL),
    controllerTimeoutMs: positiveInt('CONTROLLER_TIMEOUT_MS', env.CONTROLLER_TIMEOUT_MS, 20000),
    port: positiveInt('PORT', env.PORT, 8080),
    host: env.HOST || '0.0.0.0',
    statusPollIntervalMs: positiveInt('STATUS_POLL_INTERVAL_MS', env.STATUS_POLL_INTERVAL_MS, 5000),
    motionPollIntervalMs: positiveInt('MOTION_POLL_INTERVAL_MS', env.MOTION_POLL_INTERVAL_MS, 300),
    motionTimeoutMs: positiveInt('MOTION_TIMEOUT_MS', env.MOTION_TIMEOUT_MS, 30000),
    autoCalibrate: (env.AUTO_CALIBRATE ?? 'true').toLowerCase() !== 'false',
    calibrationDataPath,
  };
}
