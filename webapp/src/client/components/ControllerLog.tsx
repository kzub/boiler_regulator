import { useState } from 'react';
import type { ControllerLogEntry } from '../api';

interface Props { entries: ControllerLogEntry[]; }

function sourceLabel(source: ControllerLogEntry['source']): string {
  if (source === 'browser') return 'браузер → приложение';
  if (source === 'controller') return 'приложение → контроллер';
  return 'калибровка backend';
}

function logText(entries: ControllerLogEntry[]): string {
  return entries.map((entry) => [
    new Date(entry.at).toLocaleString('ru-RU'), sourceLabel(entry.source),
    `→ ${entry.request}`, `← ${entry.response}`,
  ].join('\n')).join('\n\n');
}

async function copyText(value: string): Promise<void> {
  if (navigator.clipboard && window.isSecureContext) return navigator.clipboard.writeText(value);
  const textarea = document.createElement('textarea');
  textarea.value = value; textarea.style.position = 'fixed'; textarea.style.opacity = '0';
  document.body.appendChild(textarea); textarea.select();
  const copied = document.execCommand('copy'); textarea.remove();
  if (!copied) throw new Error('copy failed');
}

export function ControllerLog({ entries }: Props) {
  const [copyState, setCopyState] = useState<'idle' | 'copied' | 'failed'>('idle');
  const copy = async () => {
    try { await copyText(logText(entries)); setCopyState('copied'); }
    catch { setCopyState('failed'); }
    window.setTimeout(() => setCopyState('idle'), 2000);
  };
  return <section className="controller-log" aria-labelledby="controller-log-heading">
    <div className="section-title"><div><h2 id="controller-log-heading">Журнал запросов</h2><span>последние попытки</span></div><button className="copy-log" type="button" onClick={() => void copy()} disabled={entries.length === 0}>{copyState === 'copied' ? 'Скопировано' : copyState === 'failed' ? 'Ошибка копирования' : 'Копировать журнал'}</button></div>
    <div className="log-window" role="log" aria-live="polite">
      {entries.length === 0 ? <p>Ожидание первого запроса…</p> : entries.map((entry, index) => <div className={`log-entry${entry.ok ? '' : ' failed'}`} key={`${entry.source}-${entry.at}-${index}`}>
        <time>{new Date(entry.at).toLocaleTimeString('ru-RU')}</time>
        <span className="log-source">{sourceLabel(entry.source)}</span>
        <code>→ {entry.request}</code><code>← {entry.response}</code>
      </div>)}
    </div>
  </section>;
}
