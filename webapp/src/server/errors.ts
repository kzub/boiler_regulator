export class ControllerError extends Error {
  constructor(public readonly code: 'CONTROLLER_UNREACHABLE' | 'CONTROLLER_TIMEOUT' | 'INVALID_CONTROLLER_RESPONSE', message: string) {
    super(message);
  }
}

export function errorStatus(code: string): number {
  if (code === 'CONTROLLER_UNREACHABLE') return 503;
  if (code === 'CONTROLLER_TIMEOUT') return 504;
  if (code === 'INVALID_CONTROLLER_RESPONSE' || code === 'CALIBRATION_FAILED') return 502;
  if (code === 'INVALID_REQUEST') return 400;
  return 409;
}
