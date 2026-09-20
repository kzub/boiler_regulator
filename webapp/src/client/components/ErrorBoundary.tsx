import { Component, type ErrorInfo, type ReactNode } from 'react';

interface Props { children: ReactNode; }
interface State { message: string | null; }

/**
 * Catches render errors so a client bug shows a readable screen instead of a blank page.
 * It never retries a physical command: recovery is an explicit page reload by the operator.
 */
export class ErrorBoundary extends Component<Props, State> {
  state: State = { message: null };

  static getDerivedStateFromError(error: unknown): State {
    return { message: error instanceof Error && error.message ? error.message : 'Неизвестная ошибка интерфейса' };
  }

  componentDidCatch(error: Error, info: ErrorInfo): void {
    console.error('UI error boundary', error, info.componentStack);
  }

  render(): ReactNode {
    if (this.state.message === null) return this.props.children;
    return <main className="app-shell">
      <section className="fatal" role="alert">
        <p className="eyebrow">ОШИБКА ИНТЕРФЕЙСА</p>
        <h1>Приложение остановилось</h1>
        <p>Регуляторы продолжают работать: страница не управляет ими напрямую, и ни одна команда не была повторена автоматически.</p>
        <code>{this.state.message}</code>
        <button type="button" onClick={() => window.location.reload()}>Перезагрузить страницу</button>
      </section>
    </main>;
  }
}
