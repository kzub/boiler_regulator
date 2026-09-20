export interface ControllerJournalEntry {
  at: string;
  request: string;
  response: string;
  source: 'controller' | 'backend';
  ok: boolean;
}

/** Newest-first retention limit. Polling runs for the whole process lifetime, so the journal must stay bounded. */
export const defaultJournalLimit = 500;

/** Process-lifetime diagnostic journal. It deliberately stores no headers, URL origin, or secrets. */
export class ControllerJournal {
  private readonly entries: ControllerJournalEntry[] = [];
  private readonly limit: number;

  constructor(limit: number = defaultJournalLimit) {
    if (!Number.isInteger(limit) || limit <= 0) throw new Error('journal limit must be a positive integer');
    this.limit = limit;
  }

  record(request: string, response: string): void {
    const ok = !response.startsWith('ERROR:') && !/^HTTP [45]/.test(response);
    this.push({ at: new Date().toISOString(), request, response, source: 'controller', ok });
  }

  event(name: string, details: Record<string, unknown>, ok = true): void {
    this.push({
      at: new Date().toISOString(), request: name, response: JSON.stringify(details), source: 'backend', ok,
    });
  }

  recent(): ControllerJournalEntry[] { return [...this.entries]; }

  /** Keeps the newest entries first and drops the oldest tail beyond the limit. */
  private push(entry: ControllerJournalEntry): void {
    this.entries.unshift(entry);
    if (this.entries.length > this.limit) this.entries.length = this.limit;
  }
}
