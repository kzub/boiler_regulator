import { useEffect, useState } from 'react';
import type { PublicRegulator } from '../../shared/contracts';

interface Props {
  id: 1 | 2; title: string; data: PublicRegulator; disabled: boolean;
  onTarget: (percent: number) => void; onLimit: (limit: 'min' | 'max') => void;
}

const quickTargets = [0, 25, 50, 75, 100];

/** 0% and 100% are the physical endstops, so they stay available without a calibrated scale. */
function isLimitPercent(percent: number): boolean { return percent === 0 || percent === 100; }

export function RegulatorControl({ id, title, data, disabled, onTarget, onLimit }: Props) {
  const [draft, setDraft] = useState(data.percent ?? 50);
  useEffect(() => { if (data.percent !== null && data.state !== 'moving') setDraft(data.percent); }, [data.percent, data.state]);
  const unavailable = disabled || data.percent === null;

  /** 0% drives to MIN and 100% to MAX; everything between is a calculated percentage target. */
  const apply = (percent: number) => {
    if (percent === 0) onLimit('min');
    else if (percent === 100) onLimit('max');
    else onTarget(percent);
  };

  return <section className="regulator-card" aria-labelledby={`regulator-${id}`}>
    <div className="card-heading"><div><p className="eyebrow">Регулятор {id}</p><h2 id={`regulator-${id}`}>{title}</h2></div><span className={`state ${data.state}`}>{data.state === 'moving' ? 'движение' : data.percent === null ? 'нет шкалы' : 'готов'}</span></div>
    {data.percent === null ? <div className="uncalibrated"><strong>Шкала не откалибрована</strong><span>Промежуточные проценты станут доступны после полной калибровки. 0% и 100% работают всегда.</span></div> : <>
      <div className="percent-row"><div><span>Текущее</span><strong>{data.percent}%</strong></div>{data.targetPercent !== null && <div><span>Цель</span><strong>{data.targetPercent}%</strong></div>}</div>
      <p className="position">Позиция: {data.position ?? '—'} шагов</p>
    </>}
    <div className="quick-targets" aria-label="Быстрый выбор положения">
      {quickTargets.map((percent) => <button
        key={percent}
        className="secondary"
        disabled={isLimitPercent(percent) ? disabled : unavailable || data.percent === percent}
        onClick={() => apply(percent)}
      >{percent}%</button>)}
    </div>
    <p className="limit-hint">0% и 100% доходят до физического концевика.</p>
    {data.percent !== null && <>
      <label className="slider-label" htmlFor={`target-${id}`}>Подготовить цель: <b>{draft}%</b></label>
      <input id={`target-${id}`} type="range" min="0" max="100" step="1" value={draft} disabled={unavailable} onChange={(event) => setDraft(Number(event.target.value))} />
      <button className="primary wide" disabled={unavailable || draft === data.percent} onClick={() => apply(draft)}>Установить {draft}%</button>
    </>}
  </section>;
}
