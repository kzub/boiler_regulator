import { useState } from 'react';
import type { Limit } from '../shared/contracts';
import { api, ApiRequestError } from './api';
import { AppVersion } from './components/AppVersion';
import { CommandConfirm } from './components/CommandConfirm';
import { ConnectionStatus } from './components/ConnectionStatus';
import { ControllerLog } from './components/ControllerLog';
import { RegulatorControl } from './components/RegulatorControl';
import { TemperatureMetric } from './components/TemperatureMetric';
import { useStatus } from './hooks/use-status';
import { useControllerLog } from './hooks/use-controller-log';

type Pending = { kind: 'target'; id: 1 | 2; percent: number } | { kind: 'limit'; id: 1 | 2; limit: Limit } | { kind: 'calibration' };

const phaseText = {
  motor1_min: 'Первый этаж → MIN', motor1_max: 'Первый этаж → MAX',
  motor2_min: 'Второй этаж → MIN', motor2_max: 'Второй этаж → MAX',
  motor1_restore: 'Первый этаж → прежнее положение',
  motor2_restore: 'Второй этаж → прежнее положение',
};

function floorName(id: 1 | 2): string { return id === 1 ? 'Первый этаж' : 'Второй этаж'; }

function initialNotice(code: string): string {
  return ({ CONTROLLER_BUSY: 'Дождитесь завершения текущего движения.', CALIBRATION_REQUIRED: 'Сначала выполните калибровку.', CALIBRATION_IN_PROGRESS: 'Сейчас идёт калибровка.' } as Record<string, string>)[code] || 'Не удалось подтвердить приём команды. Проверьте состояние контроллера.';
}

export default function App() {
  const { data, loading, offline, lastError, refresh } = useStatus();
  const controllerLog = useControllerLog();
  const [pending, setPending] = useState<Pending | null>(null);
  const [sending, setSending] = useState(false);
  const [notice, setNotice] = useState<string | null>(null);
  const calibration = data?.calibration;
  const motionInProgress = data?.regulators.floor1.state === 'moving' || data?.regulators.floor2.state === 'moving';
  const globalDisabled = offline || !data || calibration?.state === 'running' || motionInProgress;

  const execute = async () => {
    if (!pending || sending) return;
    setSending(true);
    try {
      if (pending.kind === 'target') await api.target(pending.id, pending.percent);
      else if (pending.kind === 'limit') await api.limit(pending.id, pending.limit);
      else await api.startCalibration();
      setNotice(pending.kind === 'calibration' ? 'Калибровка запущена. Оба регулятора будут двигаться автоматически.' : 'Команда принята. Дождитесь обновления позиции.');
      setPending(null); await refresh();
    } catch (error) { setNotice(initialNotice(error instanceof ApiRequestError ? error.code : 'UNKNOWN')); setPending(null); }
    finally { setSending(false); }
  };

  const dialog = pending && {
    title: pending.kind === 'calibration' ? 'Запустить калибровку?' : pending.kind === 'target'
      ? `${floorName(pending.id)} → ${pending.percent}%?` : `${floorName(pending.id)} → ${pending.limit === 'min' ? '0%' : '100%'}?`,
    text: pending.kind === 'calibration' ? 'Регуляторы по очереди дойдут до обоих физических пределов, затем вернутся к сохранённому проценту открытия. В это время управление недоступно.'
      : pending.kind === 'limit' ? 'Регулятор дойдёт до физического концевика. Контроллер подтверждает только приём команды, а не достижение положения.'
      : 'Команда начнёт движение. Контроллер подтверждает только её приём, а не достижение положения.',
  };

  return <main className="app-shell">
    <header><div><p className="eyebrow">BOILER REGULATOR</p><h1>Котельная</h1></div><ConnectionStatus offline={offline} receivedAt={data?.receivedAt} error={lastError} /></header>
    {notice && <div className="notice" aria-live="polite">{notice}<button aria-label="Закрыть уведомление" onClick={() => setNotice(null)}>×</button></div>}
    {loading && !data ? <section className="loading" aria-live="polite">Получаем состояние контроллера…</section> : <>
      <section className="temperatures" aria-labelledby="temperatures-heading"><div className="section-title"><h2 id="temperatures-heading">Температуры</h2><span>сейчас</span></div><div className="temperature-grid">
        <TemperatureMetric label="Первый этаж" value={data?.temperatures.floor1 ?? null} />
        <TemperatureMetric label="Второй этаж" value={data?.temperatures.floor2 ?? null} />
        <TemperatureMetric label="Температура воды котла" value={data?.temperatures.boilerWater ?? null} />
        <TemperatureMetric label="Бойлерная комната" value={data?.temperatures.boilerRoom ?? null} />
      </div></section>
      {calibration && calibration.state !== 'ready' && <section className={`calibration ${calibration.state}`} aria-live="polite"><div><p className="eyebrow">КАЛИБРОВКА</p><h2>{calibration.state === 'running' ? 'Регуляторы калибруются' : calibration.state === 'failed' ? 'Калибровка не завершена' : 'Нужна калибровка'}</h2><p>{calibration.state === 'running' && calibration.phase ? phaseText[calibration.phase] : calibration.error || 'Проценты пока недоступны.'}</p></div>{calibration.state === 'running' ? <div className="progress"><span>{calibration.progressPercent}%</span><i style={{ width: `${calibration.progressPercent}%` }} /></div> : <button onClick={() => setPending({ kind: 'calibration' })} disabled={offline}>Перекалибровать</button>}</section>}
      {data?.warnings.length ? <div className="warnings" role="status">{data.warnings.join(' · ')}</div> : null}
      <section className="regulator-list" aria-label="Управление регуляторами">
        <RegulatorControl id={1} title="Первый этаж" data={data?.regulators.floor1 ?? { position: null, percent: null, targetPercent: null, state: 'uncalibrated' }} disabled={globalDisabled || data?.regulators.floor1.state === 'moving'} onTarget={(percent) => setPending({ kind: 'target', id: 1, percent })} onLimit={(limit) => setPending({ kind: 'limit', id: 1, limit })} />
        <RegulatorControl id={2} title="Второй этаж" data={data?.regulators.floor2 ?? { position: null, percent: null, targetPercent: null, state: 'uncalibrated' }} disabled={globalDisabled || data?.regulators.floor2.state === 'moving'} onTarget={(percent) => setPending({ kind: 'target', id: 2, percent })} onLimit={(limit) => setPending({ kind: 'limit', id: 2, limit })} />
      </section>
    </>}
    <ControllerLog entries={controllerLog} />
    <AppVersion />
    {dialog && <CommandConfirm {...dialog} busy={sending} onConfirm={() => void execute()} onClose={() => setPending(null)} />}
  </main>;
}
