interface Props { label: string; value: number | null; }

export function TemperatureMetric({ label, value }: Props) {
  return <article className="temperature-card">
    <span>{label}</span>
    <strong>{value === null ? '—' : `${value.toFixed(1)}°C`}</strong>
    {value === null && <small>нет данных</small>}
  </article>;
}
