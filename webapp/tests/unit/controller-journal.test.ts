import { describe, expect, it } from 'vitest';
import { ControllerJournal, defaultJournalLimit } from '../../src/server/controller-journal.js';

describe('controller journal', () => {
  it('keeps every controller attempt within the retention limit and marks failures', () => {
    const journal = new ControllerJournal();
    for (let index = 0; index < 35; index += 1) journal.record('GET /status', `{"attempt":${index}}`);
    journal.record('GET /status', 'HTTP 503');
    journal.record('GET /status', 'ERROR: timeout');

    const entries = journal.recent();
    expect(entries).toHaveLength(37);
    expect(entries[0]).toMatchObject({ source: 'controller', ok: false, response: 'ERROR: timeout' });
    expect(entries[1]).toMatchObject({ source: 'controller', ok: false, response: 'HTTP 503' });
    expect(entries[2]).toMatchObject({ source: 'controller', ok: true, response: '{"attempt":34}' });
  });

  it('drops the oldest entries once the retention limit is reached', () => {
    const journal = new ControllerJournal(5);
    for (let index = 0; index < 12; index += 1) journal.record('GET /status', `{"attempt":${index}}`);

    const entries = journal.recent();
    expect(entries).toHaveLength(5);
    expect(entries[0].response).toBe('{"attempt":11}');
    expect(entries[4].response).toBe('{"attempt":7}');
  });

  it('bounds the default journal so continuous polling cannot grow memory without limit', () => {
    const journal = new ControllerJournal();
    for (let index = 0; index < defaultJournalLimit + 250; index += 1) journal.record('GET /status', `{"attempt":${index}}`);
    journal.event('calibration_started', { motor: 1 });

    const entries = journal.recent();
    expect(entries).toHaveLength(defaultJournalLimit);
    expect(entries[0]).toMatchObject({ source: 'backend', request: 'calibration_started' });
  });

  it('rejects a non-positive retention limit', () => {
    expect(() => new ControllerJournal(0)).toThrow(/positive integer/);
    expect(() => new ControllerJournal(2.5)).toThrow(/positive integer/);
  });

  it('records backend calibration events in the same journal', () => {
    const journal = new ControllerJournal();
    journal.event('calibration_restore_started', { motor: 1, delta: 8730 });
    expect(journal.recent()[0]).toMatchObject({
      source: 'backend', ok: true, request: 'calibration_restore_started', response: '{"motor":1,"delta":8730}',
    });
  });
});
