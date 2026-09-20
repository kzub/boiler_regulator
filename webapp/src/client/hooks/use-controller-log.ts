import { useEffect, useState } from 'react';
import { api, browserLogEntries, subscribeBrowserLog, type ControllerLogEntry } from '../api';

export function useControllerLog(): ControllerLogEntry[] {
  const [controllerEntries, setControllerEntries] = useState<ControllerLogEntry[]>([]);
  const [browserEntries, setBrowserEntries] = useState(browserLogEntries);
  useEffect(() => {
    let alive = true;
    const unsubscribe = subscribeBrowserLog(() => setBrowserEntries(browserLogEntries()));
    const load = () => api.controllerLog().then((data) => { if (alive) setControllerEntries(data.entries); }).catch(() => undefined);
    load();
    const timer = window.setInterval(load, 5000);
    return () => { alive = false; unsubscribe(); window.clearInterval(timer); };
  }, []);
  return [...browserEntries, ...controllerEntries].sort((left, right) => right.at.localeCompare(left.at));
}
