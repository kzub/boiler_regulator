import { useEffect, useRef } from 'react';

interface Props { title: string; text: string; busy: boolean; onConfirm: () => void; onClose: () => void; }

export function CommandConfirm({ title, text, busy, onConfirm, onClose }: Props) {
  const confirm = useRef<HTMLButtonElement>(null);
  useEffect(() => { confirm.current?.focus(); }, []);
  return <div className="modal-backdrop" role="presentation" onMouseDown={(event) => { if (event.target === event.currentTarget && !busy) onClose(); }}>
    <section className="dialog" role="dialog" aria-modal="true" aria-labelledby="confirm-title">
      <h2 id="confirm-title">{title}</h2><p>{text}</p>
      <div className="dialog-actions"><button className="secondary" disabled={busy} onClick={onClose}>Отмена</button><button ref={confirm} disabled={busy} onClick={onConfirm}>{busy ? 'Отправляем…' : 'Подтвердить'}</button></div>
    </section>
  </div>;
}
