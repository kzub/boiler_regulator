interface Props { offline: boolean; receivedAt?: string; error?: string | null; }

export function ConnectionStatus({ offline, receivedAt, error }: Props) {
  if (offline) return <div className="connection offline" role="status">Нет связи. Показаны последние достоверные данные{error ? `: ${error}` : ''}.</div>;
  if (!receivedAt) return <div className="connection" role="status">Подключаемся к контроллеру…</div>;
  const seconds = Math.max(0, Math.round((Date.now() - new Date(receivedAt).getTime()) / 1000));
  return <div className="connection" role="status">На связи · обновлено {seconds === 0 ? 'только что' : `${seconds} с назад`}</div>;
}
