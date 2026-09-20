import type { ApiError, BuildInfo, CalibrationStatus, Limit, StatusResponse } from '../shared/contracts';

export interface ControllerLogEntry {
  at: string;
  request: string;
  response: string;
  source: 'browser' | 'controller' | 'backend';
  ok: boolean;
}

const browserLogStorageKey = 'boiler-regulator-browser-log';
const browserLogListeners = new Set<() => void>();

/** Newest-first retention limit for the tab-local journal, so polling cannot grow memory or sessionStorage without bound. */
export const browserLogLimit = 200;

function trim(entries: ControllerLogEntry[]): ControllerLogEntry[] {
  return entries.length > browserLogLimit ? entries.slice(0, browserLogLimit) : entries;
}

function loadBrowserLog(): ControllerLogEntry[] {
  if (typeof window === 'undefined') return [];
  try {
    const value: unknown = JSON.parse(window.sessionStorage.getItem(browserLogStorageKey) || '[]');
    return Array.isArray(value) ? trim(value as ControllerLogEntry[]) : [];
  } catch { return []; }
}

let browserLog = loadBrowserLog();

/** Persists the journal, shrinking it when the tab storage quota rejects the write. */
function persistBrowserLog(entries: ControllerLogEntry[]): void {
  for (let size = entries.length; size > 0; size = Math.floor(size / 2)) {
    try {
      window.sessionStorage.setItem(browserLogStorageKey, JSON.stringify(entries.slice(0, size)));
      return;
    } catch { /* Try a smaller slice, then give up and keep the in-memory journal. */ }
  }
  try { window.sessionStorage.removeItem(browserLogStorageKey); } catch { /* Keep the in-memory journal. */ }
}

function recordBrowserRequest(request: string, response: string, ok: boolean): void {
  const entry: ControllerLogEntry = { at: new Date().toISOString(), request, response, source: 'browser', ok };
  browserLog = trim([entry, ...browserLog]);
  persistBrowserLog(browserLog);
  browserLogListeners.forEach((listener) => listener());
}

export function browserLogEntries(): ControllerLogEntry[] { return [...browserLog]; }
export function subscribeBrowserLog(listener: () => void): () => void {
  browserLogListeners.add(listener);
  return () => browserLogListeners.delete(listener);
}

export class ApiRequestError extends Error {
  constructor(public readonly code: string, message: string) { super(message); }
}

async function request<T>(url: string, init?: RequestInit): Promise<T> {
  let response: Response;
  const headers = new Headers(init?.headers);
  const method = init?.method || 'GET';
  const requestDescription = `${method} ${url}${typeof init?.body === 'string' ? ` ${init.body}` : ''}`;
  const shouldLog = url !== '/api/controller-log';
  if (init?.body !== undefined && !headers.has('Content-Type')) headers.set('Content-Type', 'application/json');
  try { response = await fetch(url, { ...init, headers }); }
  catch {
    if (shouldLog) recordBrowserRequest(requestDescription, 'NETWORK ERROR: приложение недоступно', false);
    throw new ApiRequestError('NETWORK_ERROR', 'Не удалось связаться с приложением');
  }
  const body: unknown = await response.json().catch(() => null);
  if (!response.ok) {
    const apiError = body as ApiError | null;
    const code = apiError?.error?.code || 'REQUEST_FAILED';
    const message = apiError?.error?.message || 'Не удалось выполнить запрос';
    if (shouldLog) recordBrowserRequest(requestDescription, `HTTP ${response.status}: ${code} — ${message}`, false);
    throw new ApiRequestError(code, message);
  }
  if (shouldLog) recordBrowserRequest(requestDescription, `HTTP ${response.status}`, true);
  return body as T;
}

export const api = {
  status: () => request<StatusResponse>('/api/status'),
  version: () => request<BuildInfo>('/api/version'),
  calibration: () => request<CalibrationStatus>('/api/calibration'),
  startCalibration: () => request<CalibrationStatus>('/api/calibration', { method: 'POST' }),
  controllerLog: () => request<{ entries: ControllerLogEntry[] }>('/api/controller-log'),
  limit: (id: 1 | 2, limit: Limit) => request(`/api/regulators/${id}/limit`, { method: 'POST', body: JSON.stringify({ limit }) }),
  target: (id: 1 | 2, percent: number) => request(`/api/regulators/${id}/target`, { method: 'POST', body: JSON.stringify({ percent }) }),
};
