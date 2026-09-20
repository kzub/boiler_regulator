/** Minimal controllable Arduino HTTP double for local development and integration tests. */
import Fastify from 'fastify';
import type { ArduinoStatus, RegulatorId } from '../src/shared/contracts.js';
import { ControllerSimulator, type SimulatorOptions } from './controller-simulator.js';

export interface MockArduino {
  status: ArduinoStatus;
  online: boolean;
  malformedResponse: boolean;
  reset(): void;
  localMove(motor: RegulatorId, steps: number): void;
  physicalPercent(motor: RegulatorId): number;
}

export function createMockArduino(options: SimulatorOptions = {}): { app: ReturnType<typeof Fastify>; controller: MockArduino } {
  // The simulator reproduces what makes this controller awkward: it normalises the
  // counter to a fixed constant at each end-stop, and Motor 1 writes them mirrored.
  const simulator = new ControllerSimulator(options);
  const controller: MockArduino = {
    get status(): ArduinoStatus { return { ...simulator.temperatures, m1: simulator.counter(1), m2: simulator.counter(2) }; },
    online: true, malformedResponse: false,
    reset() { simulator.reboot(); },
    localMove(motor, steps) { simulator.moveLocally(motor, steps); },
    physicalPercent(motor) { return simulator.physicalPercent(motor); },
  };
  const app = Fastify({ logger: false });
  app.addHook('onRequest', async (_request, reply) => { if (!controller.online) return reply.code(503).send(); });
  app.get('/status', async (_request, reply) => {
    if (controller.malformedResponse) return reply.send({ m1: 'broken' });
    return reply.send(await (await simulator.fetcher('http://mock/status')).json());
  });
  app.get('/set', async (request, reply) => {
    const query = request.query as { motor?: string; pos?: string };
    if ((query.motor !== '1' && query.motor !== '2') || !query.pos) return reply.send({ status: 'error' });
    if (!['min', 'max'].includes(query.pos) && !Number.isInteger(Number(query.pos))) return reply.send({ status: 'error' });
    const url = `http://mock/set?motor=${query.motor}&pos=${encodeURIComponent(query.pos)}`;
    return reply.send(await (await simulator.fetcher(url)).json());
  });
  return { app, controller };
}

if (process.argv[1]?.endsWith('mock-arduino-server.ts')) {
  const { app } = createMockArduino();
  const port = Number(process.env.MOCK_ARDUINO_PORT || 8090);
  app.listen({ host: '127.0.0.1', port }).then(() => console.log(`Mock Arduino: http://127.0.0.1:${port}`));
}
