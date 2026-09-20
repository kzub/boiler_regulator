import path from 'node:path';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import Fastify from 'fastify';
import fastifyStatic from '@fastify/static';
import { ArduinoClient } from './arduino-client.js';
import { CalibrationManager } from './calibration-manager.js';
import { CommandQueue } from './command-queue.js';
import { loadConfig, type Config } from './config.js';
import { CalibrationStore } from './calibration-store.js';
import { ControllerJournal } from './controller-journal.js';
import { registerRoutes } from './routes.js';

export function createServer(config: Config) {
  const app = Fastify({ logger: { level: process.env.LOG_LEVEL || 'info' } });
  const journal = new ControllerJournal();
  const manager = new CalibrationManager(
    new ArduinoClient(config, fetch, journal), new CalibrationStore(config.calibrationDataPath), new CommandQueue(), config, app.log, journal,
  );
  registerRoutes(app, manager, journal);
  const currentDir = path.dirname(fileURLToPath(import.meta.url));
  const staticRoot = path.resolve(currentDir, '../client');
  if (existsSync(staticRoot)) app.register(fastifyStatic, { root: staticRoot, wildcard: false });
  app.setNotFoundHandler((request, reply) => {
    if (request.url.startsWith('/api/')) return reply.code(404).send({ error: { code: 'NOT_FOUND', message: 'Маршрут не найден' } });
    return existsSync(staticRoot) ? reply.sendFile('index.html') : reply.code(404).send({ error: { code: 'NOT_FOUND', message: 'Маршрут не найден' } });
  });
  app.addHook('onClose', async () => app.log.info('Boiler regulator server stopped'));
  return app;
}

async function main(): Promise<void> {
  const config = loadConfig();
  const app = createServer(config);
  const close = async () => { await app.close(); process.exit(0); };
  process.once('SIGTERM', close); process.once('SIGINT', close);
  await app.listen({ port: config.port, host: config.host });
}

if (process.argv[1] === fileURLToPath(import.meta.url)) {
  main().catch((error: unknown) => { console.error(error); process.exit(1); });
}
