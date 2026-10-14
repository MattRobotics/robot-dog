/**
 * Live-feed classification: one unambiguous source badge plus separate
 * diagnostic codes. Pure functions, no DOM and no network, so the mapping is
 * unit-tested. Fake or synthetic data can never be classified as LIVE.
 */
import type { MotionTelemetryStream } from './telemetry';

export type FeedMode = 'off' | 'synthetic' | 'live' | 'trace';
export type SourceBadge = 'LIVE' | 'FAKE_DEVICE' | 'OFFLINE_SYNTHETIC' | 'TRACE' | 'STALE' | 'OFF';
export type TransportError = 'NONE' | 'BRIDGE_UNREACHABLE' | 'HTTP_TIMEOUT' | 'HTTP_ERROR';
export type DiagnosticCode =
  | 'RESTORE_REQUIRED' | 'MOTION_NOT_PERMITTED' | 'TELEMETRY_STALE' | 'BRIDGE_UNREACHABLE'
  | 'SERIAL_DISCONNECTED' | 'SERIAL_OWNER_BUSY' | 'ASSETS_MISSING' | 'HTTP_TIMEOUT' | 'LEGACY_POLL'
  | 'NO_CONTROLLER_DATA' | 'JOINT_OUTSIDE_URDF_LIMIT';

export const FAKE_DEVICE_SOURCE = 'FAKE_DEVICE_NOT_HARDWARE';

export const DIAGNOSTIC_TEXT: Readonly<Record<DiagnosticCode, string>> = {
  RESTORE_REQUIRED: 'Calibration RESTORE not done: measured joint angles are unavailable (raw ticks only).',
  MOTION_NOT_PERMITTED: 'Controller readiness/permit does not allow motion.',
  TELEMETRY_STALE: 'Last Controller sample is older than 1.5 s (or flagged stale).',
  BRIDGE_UNREACHABLE: 'The host bridge did not answer (network error or HTTP error).',
  SERIAL_DISCONNECTED: 'The bridge has no open serial port (USB unplugged or Controller absent).',
  SERIAL_OWNER_BUSY: 'Another process owns the serial port; close other monitors.',
  ASSETS_MISSING: 'Canonical URDF or one of the 17 collision STLs failed to load.',
  HTTP_TIMEOUT: 'A status request timed out.',
  LEGACY_POLL: 'Installed firmware has no @TEL stream: bridge polls read-only commands (~1.5 s per joint).',
  NO_CONTROLLER_DATA: 'The bridge is up but has no Controller sample yet (or the Controller rebooted).',
  JOINT_OUTSIDE_URDF_LIMIT: 'A measured joint is outside the canonical URDF limit; the grey model is hidden, see the table.',
};

export interface BridgeInfo {
  readonly source: string;
  readonly serialState: string;
  readonly serialDetail: string;
  readonly ageMs: number | null;
  readonly bootNonce: number | null;
  readonly rebootsSeen: number;
  readonly linesPerS: number | null;
  readonly telHz: number | null;
  readonly fakeDevice: boolean;
  readonly phase: string;
  readonly consoleActive: boolean;
  readonly legacyCycleMs: number | null;
}

function record(value: unknown): Record<string, unknown> | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value) ? value as Record<string, unknown> : null;
}
function num(value: unknown): number | null { return typeof value === 'number' && Number.isFinite(value) ? value : null; }
function text(value: unknown, fallback = ''): string { return typeof value === 'string' ? value.slice(0, 200) : fallback; }

export function parseBridgeInfo(value: unknown): BridgeInfo | null {
  const bridge = record(value);
  if (!bridge) return null;
  return {
    source: text(bridge.source, 'NONE'), serialState: text(bridge.serial_state, 'UNKNOWN'),
    serialDetail: text(bridge.serial_detail), ageMs: num(bridge.age_ms), bootNonce: num(bridge.boot_nonce),
    rebootsSeen: num(bridge.reboots_seen) ?? 0, linesPerS: num(bridge.lines_per_s), telHz: num(bridge.tel_hz),
    fakeDevice: bridge.fake_device === true, phase: text(bridge.phase), consoleActive: bridge.console_active === true,
    legacyCycleMs: num(bridge.legacy_cycle_ms),
  };
}

export interface LiveSnapshot {
  readonly mode: FeedMode;
  readonly hasFrame: boolean;
  readonly stale: boolean;
  readonly bridge: BridgeInfo | null;
  readonly telemetrySource: string | null;
  readonly transportError: TransportError;
  readonly assetsMissing: boolean;
  /** `available:false` from the bridge: reachable, but no Controller sample. */
  readonly available: boolean | null;
  readonly restore: boolean | null;
  readonly permit: boolean | null;
  readonly reason: string;
  readonly actualMissing: boolean;
  readonly anyUnmapped: boolean;
  readonly commissioningRestore: string | null;
  readonly jointLimitViolation: boolean;
}

export const EMPTY_SNAPSHOT: LiveSnapshot = {
  mode: 'off', hasFrame: false, stale: true, bridge: null, telemetrySource: null, transportError: 'NONE',
  assetsMissing: false, available: null, restore: null, permit: null, reason: '', actualMissing: false,
  anyUnmapped: false, commissioningRestore: null, jointLimitViolation: false,
};

export interface SourceClassification { readonly badge: SourceBadge; readonly label: string }

/** Exactly one badge. Fake/synthetic data is never LIVE, whatever the transport says. */
export function classifySource(s: LiveSnapshot): SourceClassification {
  if (s.mode === 'synthetic') return { badge: 'OFFLINE_SYNTHETIC', label: 'OFFLINE_SYNTHETIC' };
  if (s.mode === 'trace') return { badge: 'TRACE', label: 'TRACE (offline replay)' };
  if (s.mode === 'off') return s.hasFrame ? { badge: 'STALE', label: 'STALE (feed paused)' } : { badge: 'OFF', label: 'OFF' };
  const via = s.bridge ? (s.bridge.source === 'NONE' ? 'BRIDGE' : s.bridge.source) : 'CONTROLLER_HTTP';
  const source = s.telemetrySource ?? '';
  if (s.bridge?.fakeDevice || /FAKE/i.test(source)) {
    return { badge: 'FAKE_DEVICE', label: `FAKE_DEVICE · ${via}${s.stale ? ' · STALE' : ''}` };
  }
  if (/SYNTHETIC|SIMULAT/i.test(source)) return { badge: 'OFFLINE_SYNTHETIC', label: `OFFLINE_SYNTHETIC · ${via}` };
  if (!s.hasFrame || s.stale) return { badge: 'STALE', label: s.hasFrame ? `STALE · ${via}` : `STALE · no sample · ${via}` };
  return { badge: 'LIVE', label: `LIVE · ${via}` };
}

/** Separate, simultaneous reasons instead of one STALE/BLOCKED badge. Order is display order. */
export function diagnosticCodes(s: LiveSnapshot): DiagnosticCode[] {
  const codes: DiagnosticCode[] = [];
  if (s.assetsMissing) codes.push('ASSETS_MISSING');
  if (s.mode === 'live') {
    if (s.transportError === 'BRIDGE_UNREACHABLE' || s.transportError === 'HTTP_ERROR') codes.push('BRIDGE_UNREACHABLE');
    if (s.transportError === 'HTTP_TIMEOUT') codes.push('HTTP_TIMEOUT');
    const serial = s.bridge?.serialState ?? '';
    if (serial === 'SERIAL_OWNER_BUSY') codes.push('SERIAL_OWNER_BUSY');
    else if (serial === 'SERIAL_DISCONNECTED' || serial === 'SERIAL_ERROR') codes.push('SERIAL_DISCONNECTED');
    if (s.bridge?.source === 'LEGACY_POLL') codes.push('LEGACY_POLL');
    if (s.available === false) codes.push('NO_CONTROLLER_DATA');
    if (s.hasFrame && s.stale) codes.push('TELEMETRY_STALE');
  }
  if (s.hasFrame) {
    const commissioning = s.commissioningRestore ?? '';
    if (s.restore === false || s.reason === 'RESTORE_REQUIRED' || (s.actualMissing && s.anyUnmapped) ||
      commissioning === 'NOT_RUN' || commissioning.startsWith('REJECT')) codes.push('RESTORE_REQUIRED');
    if (s.permit === false || /NOT_PERMITTED|PERMIT_REQUIRED|PERMIT_DENIED/.test(s.reason)) codes.push('MOTION_NOT_PERMITTED');
    if (s.jointLimitViolation) codes.push('JOINT_OUTSIDE_URDF_LIMIT');
  }
  return codes;
}

/** Controller boot identity seen by the bridge; a change means a reboot. */
export function bootIdentity(value: unknown): string {
  const status = record(value), bridge = record(status?.bridge);
  const nonce = num(status?.boot_nonce) ?? num(bridge?.boot_nonce);
  return `${nonce ?? '-'}:${num(bridge?.reboots_seen) ?? 0}`;
}

/**
 * Accepts a live status into the freshness stream. A Controller reboot makes
 * uptime go backwards; the strict stream would then reject every later sample,
 * so a changed boot identity resets the stream first (the old sample is not
 * reused, the new one starts fresh).
 */
export class LiveFeedState {
  identity: string | null = null;
  resets = 0;
  accept(stream: MotionTelemetryStream, value: unknown, receivedAtMs: number): boolean {
    const identity = bootIdentity(value);
    if (this.identity !== null && identity !== this.identity) { stream.reset(); this.resets += 1; }
    this.identity = identity;
    return stream.accept(value, receivedAtMs);
  }
  reset(): void { this.identity = null; }
}
