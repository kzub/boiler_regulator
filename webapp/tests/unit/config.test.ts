import { describe, expect, it } from 'vitest';
import { loadConfig } from '../../src/server/config.js';

describe('configuration', () => {
  it('accepts a plain controller http origin', () => {
    const config = loadConfig({ CONTROLLER_BASE_URL: 'http://192.168.88.20' });
    expect(config.controllerBaseUrl.href).toBe('http://192.168.88.20/');
    expect(config.motionTimeoutMs).toBe(30000);
  });

  it.each(['ftp://controller', 'http://user:password@controller', 'http://controller/?x=1'])('rejects unsafe controller URLs', (url) => {
    expect(() => loadConfig({ CONTROLLER_BASE_URL: url })).toThrow('CONTROLLER_BASE_URL');
  });
});
