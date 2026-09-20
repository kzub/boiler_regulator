import { describe, expect, it, vi } from 'vitest';
import { ArduinoClient } from '../../src/server/arduino-client.js';
import { loadConfig } from '../../src/server/config.js';
import { ControllerError } from '../../src/server/errors.js';

const config = loadConfig({ CONTROLLER_BASE_URL: 'http://controller.local', CONTROLLER_TIMEOUT_MS: '50' });
const valid = { fl1: 20, fl2: 21, boiler: 22, room: 23, m1: 100, m2: 9900 };

describe('ArduinoClient', () => {
  it('strictly validates status fields', async () => {
    const fetcher = vi.fn(async () => new Response(JSON.stringify({ ...valid, m2: '9900' })));
    await expect(new ArduinoClient(config, fetcher).status()).rejects.toMatchObject<Partial<ControllerError>>({ code: 'INVALID_CONTROLLER_RESPONSE' });
  });

  it('maps a target to precisely one relative command', async () => {
    const fetcher = vi.fn(async () => new Response('{"status":"ok"}'));
    await new ArduinoClient(config, fetcher).relative(1, 55);
    expect(fetcher).toHaveBeenCalledTimes(1);
    expect(String(fetcher.mock.calls[0][0])).toContain('/set?motor=1&pos=55');
  });

  it('does not accept a controller command error body', async () => {
    const fetcher = vi.fn(async () => new Response('{"status":"error"}'));
    await expect(new ArduinoClient(config, fetcher).limit(2, 'min')).rejects.toMatchObject<Partial<ControllerError>>({ code: 'INVALID_CONTROLLER_RESPONSE' });
  });
});
