/** A deliberately tiny global lock: shared end-stops make parallel motion unsafe. */
export class CommandQueue {
  private active = false;

  get busy(): boolean { return this.active; }

  async run<T>(operation: () => Promise<T>): Promise<T> {
    if (this.active) throw new Error('CONTROLLER_BUSY');
    this.active = true;
    try { return await operation(); }
    finally { this.active = false; }
  }
}
