import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

interface FakeStorage {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

const storageKey = 'boiler-regulator-browser-log';

/** Minimal sessionStorage substitute; `maxLength` simulates the browser quota rejecting a write. */
function fakeStorage(seed: string | null = null, maxLength = Number.POSITIVE_INFINITY): FakeStorage & { map: Map<string, string> } {
  const map = new Map<string, string>();
  if (seed !== null) map.set(storageKey, seed);
  return {
    map,
    getItem: (key) => map.get(key) ?? null,
    setItem: (key, value) => {
      if (value.length > maxLength) throw new Error('QuotaExceededError');
      map.set(key, value);
    },
    removeItem: (key) => { map.delete(key); },
  };
}

function stubOkFetch(): void {
  vi.stubGlobal('fetch', vi.fn(async () => new Response('{}', { status: 200, headers: { 'Content-Type': 'application/json' } })));
}

async function loadApi(storage: FakeStorage) {
  vi.stubGlobal('window', { sessionStorage: storage });
  vi.resetModules();
  return import('../../src/client/api.js');
}

function storedEntries(storage: { map: Map<string, string> }): unknown[] {
  return JSON.parse(storage.map.get(storageKey) || '[]') as unknown[];
}

beforeEach(() => { stubOkFetch(); });
afterEach(() => { vi.unstubAllGlobals(); });

describe('browser journal retention', () => {
  it('keeps only the newest entries in memory and in sessionStorage', async () => {
    const storage = fakeStorage();
    const { api, browserLogEntries, browserLogLimit } = await loadApi(storage);

    const total = browserLogLimit + 50;
    for (let index = 0; index < total; index += 1) await api.target(1, index % 101);

    const entries = browserLogEntries();
    expect(entries).toHaveLength(browserLogLimit);
    expect(entries[0].request).toContain(`{"percent":${(total - 1) % 101}}`);
    expect(storedEntries(storage)).toHaveLength(browserLogLimit);
  });

  it('trims an oversized journal persisted by an earlier session', async () => {
    const oversized = Array.from({ length: 900 }, (_unused, index) => ({
      at: new Date(index).toISOString(), request: `GET /api/status#${index}`, response: 'HTTP 200', source: 'browser', ok: true,
    }));
    const storage = fakeStorage(JSON.stringify(oversized));
    const { browserLogEntries, browserLogLimit } = await loadApi(storage);

    const entries = browserLogEntries();
    expect(entries).toHaveLength(browserLogLimit);
    expect(entries[0].request).toBe('GET /api/status#0');
  });

  it('shrinks the persisted journal when the storage quota rejects the write', async () => {
    const storage = fakeStorage(null, 2000);
    const { api, browserLogEntries } = await loadApi(storage);

    for (let index = 0; index < 60; index += 1) await api.target(2, index % 101);

    const stored = storedEntries(storage);
    expect(stored.length).toBeGreaterThan(0);
    expect(stored.length).toBeLessThan(browserLogEntries().length);
    expect(browserLogEntries()).toHaveLength(60);
  });

  it('keeps the in-memory journal when sessionStorage rejects every write', async () => {
    const storage = fakeStorage(null, 0);
    const { api, browserLogEntries } = await loadApi(storage);

    await api.limit(1, 'min');

    expect(browserLogEntries()).toHaveLength(1);
    expect(storage.map.has(storageKey)).toBe(false);
  });
});
