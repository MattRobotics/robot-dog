/**
 * Same-origin transport to the MATDOG host bridge (matdog_bridge.py).
 *
 * Reads: Server-Sent Events from /api/bridge/events with automatic fallback to
 * polling GET /api/motion/status. Without a bridge (Vite proxy straight to the
 * Controller) only polling is used, at the Controller-friendly 1 Hz default.
 *
 * The single write is postStop(): POST /api/bridge/stop, which makes the bridge
 * write the two de-escalation commands. There is no other command path.
 */
import type { TransportError } from './liveStatus';

export type TransportKind = 'sse' | 'poll' | 'idle';

export interface TransportEvents {
  onPayload(value: unknown, via: TransportKind, bytes: number): void;
  onError(kind: TransportError, detail: string): void;
  onTransport(kind: TransportKind): void;
}

export interface TransportOptions {
  readonly useSse: boolean;
  readonly pollHz: number;
  readonly timeoutMs: number;
}

const SSE_SILENCE_MS = 3500;
const SSE_RETRY_MS = 10000;

async function fetchJson(url: string, timeoutMs: number): Promise<{ value: unknown; bytes: number }> {
  const abort = new AbortController();
  const timer = window.setTimeout(() => abort.abort(), timeoutMs);
  try {
    const response = await fetch(url, { cache: 'no-store', signal: abort.signal });
    if (!response.ok) throw Object.assign(new Error(`HTTP ${response.status}`), { httpStatus: response.status });
    const body = await response.text();
    return { value: JSON.parse(body), bytes: body.length };
  } finally {
    window.clearTimeout(timer);
  }
}

function classifyError(error: unknown): TransportError {
  if (error instanceof DOMException && error.name === 'AbortError') return 'HTTP_TIMEOUT';
  if (typeof error === 'object' && error !== null && 'httpStatus' in error) return 'HTTP_ERROR';
  return 'BRIDGE_UNREACHABLE';
}

/** Returns the bridge health object when this page is served by (or proxied to) the bridge. */
export async function detectBridge(timeoutMs = 1500): Promise<Record<string, unknown> | null> {
  try {
    const { value } = await fetchJson('/api/bridge/health', timeoutMs);
    const health = value as Record<string, unknown> | null;
    return health && health.name === 'matdog_bridge' ? health : null;
  } catch {
    return null;
  }
}

export interface StopResult { readonly ok: boolean; readonly status: number; readonly body: unknown; readonly error: string }

/** De-escalation only. Single tap, no confirmation; the bridge writes STOP lines and nothing else. */
export async function postStop(timeoutMs = 4000): Promise<StopResult> {
  const abort = new AbortController();
  const timer = window.setTimeout(() => abort.abort(), timeoutMs);
  try {
    const response = await fetch('/api/bridge/stop', { method: 'POST', cache: 'no-store', signal: abort.signal });
    const body: unknown = await response.json().catch(() => null);
    return { ok: response.ok, status: response.status, body, error: response.ok ? '' : `HTTP ${response.status}` };
  } catch (error) {
    return { ok: false, status: 0, body: null, error: classifyError(error) };
  } finally {
    window.clearTimeout(timer);
  }
}

export class LiveTransport {
  private source: EventSource | null = null;
  private pollTimer = 0;
  private retryTimer = 0;
  private watchdogTimer = 0;
  private lastSseMs = 0;
  private running = false;
  private polling = false;
  private busy = false;
  kind: TransportKind = 'idle';
  payloads = 0;
  bytes = 0;

  constructor(private readonly events: TransportEvents, private readonly options: TransportOptions) {}

  start(): void {
    if (this.running) return;
    this.running = true;
    if (this.options.useSse && typeof EventSource !== 'undefined') this.startSse();
    else this.startPolling();
  }

  stop(): void {
    this.running = false;
    this.closeSse();
    this.polling = false;
    window.clearTimeout(this.pollTimer);
    window.clearTimeout(this.retryTimer);
    this.setKind('idle');
  }

  private setKind(kind: TransportKind): void {
    if (kind !== this.kind) { this.kind = kind; this.events.onTransport(kind); }
  }

  private deliver(value: unknown, via: TransportKind, bytes: number): void {
    this.payloads += 1; this.bytes += bytes;
    this.events.onPayload(value, via, bytes);
  }

  private startSse(): void {
    this.closeSse();
    const source = new EventSource('/api/bridge/events');
    this.source = source;
    this.lastSseMs = performance.now();
    source.onmessage = (event: MessageEvent<string>) => {
      this.lastSseMs = performance.now();
      let value: unknown;
      try { value = JSON.parse(event.data); } catch { return; }
      if (this.polling) { this.polling = false; window.clearTimeout(this.pollTimer); }
      this.setKind('sse');
      this.deliver(value, 'sse', event.data.length);
    };
    source.onerror = () => this.sseFailed('event stream error');
    this.watchdogTimer = window.setInterval(() => {
      if (performance.now() - this.lastSseMs > SSE_SILENCE_MS) this.sseFailed('event stream silent');
    }, 1000);
  }

  private closeSse(): void {
    window.clearInterval(this.watchdogTimer);
    if (this.source) { this.source.onmessage = null; this.source.onerror = null; this.source.close(); this.source = null; }
  }

  private sseFailed(detail: string): void {
    if (!this.running) return;
    this.closeSse();
    this.events.onError('BRIDGE_UNREACHABLE', `${detail}; falling back to polling`);
    this.startPolling();
    window.clearTimeout(this.retryTimer);
    this.retryTimer = window.setTimeout(() => { if (this.running) this.startSse(); }, SSE_RETRY_MS);
  }

  private startPolling(): void {
    if (this.polling) return;
    this.polling = true;
    this.setKind('poll');
    void this.pollOnce();
  }

  private async pollOnce(): Promise<void> {
    window.clearTimeout(this.pollTimer);
    if (!this.running || !this.polling) return;
    const started = performance.now();
    if (!this.busy) {
      this.busy = true;
      try {
        const { value, bytes } = await fetchJson('/api/motion/status', this.options.timeoutMs);
        if (this.running && this.polling) this.deliver(value, 'poll', bytes);
      } catch (error) {
        if (this.running) this.events.onError(classifyError(error), error instanceof Error ? error.message : String(error));
      } finally {
        this.busy = false;
      }
    }
    if (!this.running || !this.polling) return;
    const period = 1000 / Math.min(20, Math.max(0.2, this.options.pollHz));
    this.pollTimer = window.setTimeout(() => void this.pollOnce(), Math.max(0, period - (performance.now() - started)));
  }
}
