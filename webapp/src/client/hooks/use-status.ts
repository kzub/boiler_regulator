import { useCallback, useEffect, useRef, useState } from 'react';
import type { StatusResponse } from '../../shared/contracts';
import { api, ApiRequestError } from '../api';

export interface StatusModel {
  data: StatusResponse | null;
  loading: boolean;
  offline: boolean;
  lastError: string | null;
  refresh: () => Promise<void>;
}

export function useStatus(): StatusModel {
  const [data, setData] = useState<StatusResponse | null>(null);
  const [loading, setLoading] = useState(true);
  const [offline, setOffline] = useState(false);
  const [lastError, setLastError] = useState<string | null>(null);
  const inFlight = useRef(false);
  const failures = useRef(0);

  const refresh = useCallback(async () => {
    if (inFlight.current) return;
    inFlight.current = true;
    try {
      const next = await api.status();
      failures.current = 0; setData(next); setOffline(false); setLastError(null);
    } catch (error) {
      failures.current += 1;
      setLastError(error instanceof ApiRequestError ? error.message : 'Не удалось обновить данные');
      if (failures.current >= 2) setOffline(true);
    } finally { setLoading(false); inFlight.current = false; }
  }, []);

  useEffect(() => {
    void refresh();
    let timer: number | undefined;
    const schedule = () => {
      window.clearInterval(timer);
      const moving = data?.regulators.floor1.state === 'moving' || data?.regulators.floor2.state === 'moving';
      timer = window.setInterval(() => { if (!document.hidden) void refresh(); }, moving ? 300 : 5000);
    };
    const visibility = () => { if (!document.hidden) void refresh(); schedule(); };
    schedule(); document.addEventListener('visibilitychange', visibility);
    return () => { window.clearInterval(timer); document.removeEventListener('visibilitychange', visibility); };
  }, [data?.regulators.floor1.state, data?.regulators.floor2.state, refresh]);

  return { data, loading, offline, lastError, refresh };
}
