import type { FastifyInstance, FastifyReply } from 'fastify';
import type { Limit, RegulatorId } from '../shared/contracts.js';
import { ControllerError, errorStatus } from './errors.js';
import { CalibrationManager } from './calibration-manager.js';
import { ControllerJournal } from './controller-journal.js';
import { loadBuildInfo } from './build-info.js';

function idFrom(value: unknown): RegulatorId | null {
  return value === '1' ? 1 : value === '2' ? 2 : null;
}

function fail(reply: FastifyReply, code: string, message: string) {
  return reply.code(errorStatus(code)).send({ error: { code, message } });
}

function bodyRecord(value: unknown): Record<string, unknown> | null {
  return value && typeof value === 'object' && !Array.isArray(value) ? value as Record<string, unknown> : null;
}

export function registerRoutes(app: FastifyInstance, manager: CalibrationManager, journal: ControllerJournal): void {
  app.get('/health/live', async () => ({ status: 'ok' }));
  app.get('/health/ready', async () => ({ status: 'ok' }));

  // Which build is actually running. The panel is updated by rebuilding its
  // container, so this is the quickest way to tell a stale image apart.
  const buildInfo = loadBuildInfo();
  app.get('/api/version', async (_request, reply) => reply.header('Cache-Control', 'no-store').send(buildInfo));

  app.get('/api/status', async (_request, reply) => {
    try { return reply.header('Cache-Control', 'no-store').send(await manager.publicStatus()); }
    catch (error) { return handleError(reply, error); }
  });

  app.get('/api/calibration', async (_request, reply) => {
    try { return reply.header('Cache-Control', 'no-store').send(await manager.calibrationStatusResponse()); }
    catch (error) { return handleError(reply, error); }
  });

  app.get('/api/controller-log', async (_request, reply) => reply.header('Cache-Control', 'no-store').send({ entries: journal.recent() }));

  app.post('/api/calibration', async (_request, reply) => {
    try { return reply.code(202).send(await manager.startCalibration()); }
    catch (error) { return handleError(reply, error); }
  });

  app.post('/api/regulators/:id/limit', async (request, reply) => {
    const motor = idFrom((request.params as { id?: string }).id);
    const body = bodyRecord(request.body);
    const limit = body?.limit;
    if (!motor || (limit !== 'min' && limit !== 'max')) return fail(reply, 'INVALID_REQUEST', 'Допустимы только регуляторы 1/2 и предел min/max');
    try {
      await manager.moveToLimit(motor, limit as Limit);
      return reply.code(202).send({ accepted: true, regulator: motor, target: limit, acceptedAt: new Date().toISOString() });
    } catch (error) { return handleError(reply, error); }
  });

  app.post('/api/regulators/:id/target', async (request, reply) => {
    const motor = idFrom((request.params as { id?: string }).id);
    const body = bodyRecord(request.body);
    const percent = body?.percent;
    if (!motor || typeof percent !== 'number' || !Number.isInteger(percent) || percent < 0 || percent > 100) {
      return fail(reply, 'INVALID_REQUEST', 'percent должен быть целым числом от 0 до 100');
    }
    try {
      const result = await manager.moveToPercent(motor, percent);
      return reply.code(result.moved ? 202 : 200).send({ accepted: true, regulator: motor, targetPercent: percent, moved: result.moved, acceptedAt: new Date().toISOString() });
    } catch (error) { return handleError(reply, error); }
  });
}

function handleError(reply: FastifyReply, error: unknown) {
  if (error instanceof ControllerError) return fail(reply, error.code, 'Контроллер недоступен или ответил неожиданно');
  if (error instanceof Error && ['CONTROLLER_BUSY', 'CALIBRATION_REQUIRED', 'CALIBRATION_IN_PROGRESS'].includes(error.message)) {
    const message = {
      CONTROLLER_BUSY: 'Дождитесь завершения текущего движения',
      CALIBRATION_REQUIRED: 'Сначала выполните калибровку',
      CALIBRATION_IN_PROGRESS: 'Идёт автоматическая калибровка',
    }[error.message] as string;
    return fail(reply, error.message, message);
  }
  appError(reply, error);
  return fail(reply, 'CALIBRATION_FAILED', 'Не удалось выполнить операцию с регулятором');
}

function appError(reply: FastifyReply, error: unknown): void {
  reply.server.log.error({ error }, 'Unhandled API error');
}
