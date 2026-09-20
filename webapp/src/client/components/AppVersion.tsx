import { useEffect, useState } from 'react';
import type { BuildInfo } from '../../shared/contracts';
import { api } from '../api';

const builtFormat = new Intl.DateTimeFormat('ru-RU', {
  day: '2-digit', month: '2-digit', year: 'numeric', hour: '2-digit', minute: '2-digit',
});

/** Says which build is running: the panel is updated by rebuilding its container. */
export function AppVersion() {
  const [info, setInfo] = useState<BuildInfo | null>(null);
  useEffect(() => {
    let active = true;
    api.version().then((value) => { if (active) setInfo(value); }).catch(() => undefined);
    return () => { active = false; };
  }, []);
  if (!info) return null;
  const parts = [`версия ${info.version}`];
  if (info.commit) parts.push(info.commit);
  parts.push(info.builtAt ? `сборка ${builtFormat.format(new Date(info.builtAt))}` : 'запуск из исходников');
  return <footer className="app-version" title={info.builtAt ?? undefined}>{parts.join(' · ')}</footer>;
}
