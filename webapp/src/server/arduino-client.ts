import type { ArduinoStatus, Limit, RegulatorId } from '../shared/contracts.js';
import type { Config } from './config.js';
import { ControllerError } from './errors.js';
import { ControllerJournal } from './controller-journal.js';

const statusFields = ['fl1', 'fl2', 'boiler', 'room', 'm1', 'm2'] as const;

function parseStatus(value: unknown): ArduinoStatus {
  if (!value || typeof value !== 'object') throw new ControllerError('INVALID_CONTROLLER_RESPONSE', 'Controller returned invalid JSON');
  const record = value as Record<string, unknown>;
  for (const field of statusFields) {
    if (typeof record[field] !== 'number' || !Number.isFinite(record[field])) {
      throw new ControllerError('INVALID_CONTROLLER_RESPONSE', `Controller is missing numeric ${field}`);
    }
  }
  return record as unknown as ArduinoStatus;
}

export class ArduinoClient {
  constructor(private readonly config: Config, private readonly fetcher: typeof fetch = fetch, private readonly journal?: ControllerJournal) {}

  private url(path: string): URL { return new URL(path, this.config.controllerBaseUrl); }

  private async request(path: string): Promise<Response> {
    try {
      return await this.fetcher(this.url(path), { signal: AbortSignal.timeout(this.config.controllerTimeoutMs) });
    } catch (error) {
      if (error instanceof DOMException && error.name === 'TimeoutError') {
        this.journal?.record(`GET ${path}`, 'ERROR: timeout');
        throw new ControllerError('CONTROLLER_TIMEOUT', 'Controller did not respond in time');
      }
      this.journal?.record(`GET ${path}`, 'ERROR: unreachable');
      throw new ControllerError('CONTROLLER_UNREACHABLE', 'Controller is unavailable');
    }
  }

  async status(): Promise<ArduinoStatus> {
    const response = await this.request('/status');
    if (!response.ok) {
      this.journal?.record('GET /status', `HTTP ${response.status}`);
      throw new ControllerError('CONTROLLER_UNREACHABLE', 'Controller is unavailable');
    }
    try {
      const parsed = parseStatus(await response.json());
      this.journal?.record('GET /status', JSON.stringify(parsed));
      return parsed;
    }
    catch (error) {
      if (error instanceof ControllerError) throw error;
      this.journal?.record('GET /status', 'ERROR: invalid JSON');
      throw new ControllerError('INVALID_CONTROLLER_RESPONSE', 'Controller returned invalid JSON');
    }
  }

  async limit(motor: RegulatorId, limit: Limit): Promise<void> {
    await this.command(`/set?motor=${motor}&pos=${limit}`);
  }

  async relative(motor: RegulatorId, delta: number): Promise<void> {
    if (!Number.isInteger(delta) || delta === 0) return;
    // The controller uses a minimal parser and does not URL-decode "%2B".
    // Unsigned digits are already interpreted as a positive relative move.
    await this.command(`/set?motor=${motor}&pos=${delta}`);
  }

  private async command(path: string): Promise<void> {
    const response = await this.request(path);
    if (!response.ok) {
      this.journal?.record(path, `HTTP ${response.status}`);
      throw new ControllerError('CONTROLLER_UNREACHABLE', 'Controller is unavailable');
    }
    let body: unknown;
    try { body = await response.json(); } catch {
      this.journal?.record(path, 'ERROR: invalid JSON');
      throw new ControllerError('INVALID_CONTROLLER_RESPONSE', 'Controller returned invalid command response');
    }
    if (!body || typeof body !== 'object' || (body as Record<string, unknown>).status !== 'ok') {
      this.journal?.record(path, `ERROR: ${JSON.stringify(body)}`);
      throw new ControllerError('INVALID_CONTROLLER_RESPONSE', 'Controller did not accept the command');
    }
    this.journal?.record(path, JSON.stringify(body));
  }
}
