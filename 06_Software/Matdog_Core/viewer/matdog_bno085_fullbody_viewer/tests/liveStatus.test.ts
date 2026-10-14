import { describe, expect, it } from 'vitest';
import { classifySource, diagnosticCodes, EMPTY_SNAPSHOT, LiveFeedState, parseBridgeInfo, type LiveSnapshot } from '../src/motion/liveStatus';
import { MotionTelemetryStream } from '../src/motion/telemetry';
import { parseDashboardModel } from '../src/dashboard/dashboardModel';

const bridge = (overrides: Record<string, unknown> = {}) =>
  parseBridgeInfo({ source: 'TEL', serial_state: 'SERIAL_OPEN', age_ms: 50, boot_nonce: 7, reboots_seen: 0, ...overrides });
const live = (overrides: Partial<LiveSnapshot> = {}): LiveSnapshot =>
  ({ ...EMPTY_SNAPSHOT, mode: 'live', hasFrame: true, stale: false, bridge: bridge(), telemetrySource: 'HARDWARE_SERVO_FEEDBACK', restore: true, permit: true, ...overrides });

// Shape of the new firmware @TEL line (status object + tel extras), values illustrative.
function telStatus(nowMs: number, sampledMs: number, bootNonce = 7, restored = true): Record<string, unknown> {
  return {
    version: 1, now_ms: nowMs, executor_state: 'OFF', stand_state: 'IDLE', owner: 'none', fault: false,
    reason: restored ? 'OPERATOR_GO_REQUIRED' : 'RESTORE_REQUIRED', stale: false, tracking_error_rad: 0,
    telemetry: { sampled_at_ms: sampledMs, actual_rad: restored ? Array(12).fill(0.1) : null, target_rad: null,
      quaternion: [1, 0, 0, 0], base_height_m: null, source: 'HARDWARE_SERVO_FEEDBACK', feet: null },
    readiness: { restore: restored, calibration_accepted: restored, limits_approved: false, permit: false, support_observable: false, hardware_approved: false },
    commissioning: { mode: 'DIAGNOSTIC', limits: 'NONE', restore: restored ? 'RESTORED' : 'NOT_RUN', pose: restored ? 'PRONE_COMPATIBLE' : 'NOT_EVALUATED',
      pose_reason: 'OK', pose_joint: 10, pose_max_dev_rad: 0.1335, entry: 'NONE', session: 'OFF' },
    tel: 1, boot_nonce: bootNonce, tel_seq: 1,
    servos: Array.from({ length: 12 }, (_, slot) => ({ slot, id: 11 + slot, raw: 2000, mapped: restored, energized: 0, speed: 0, goal: 0, temp: 31, volt_dv: 115, status: 0, age_ms: 40 })),
  };
}

describe('live source classification', () => {
  it('never labels fake or synthetic data LIVE', () => {
    expect(classifySource(live({ bridge: bridge({ fake_device: true }) })).badge).toBe('FAKE_DEVICE');
    expect(classifySource(live({ telemetrySource: 'FAKE_DEVICE_NOT_HARDWARE' })).badge).toBe('FAKE_DEVICE');
    expect(classifySource(live({ telemetrySource: 'SYNTHETIC_NOT_HARDWARE_QUALIFIED' })).badge).toBe('OFFLINE_SYNTHETIC');
    expect(classifySource({ ...EMPTY_SNAPSHOT, mode: 'synthetic' }).badge).toBe('OFFLINE_SYNTHETIC');
    expect(classifySource({ ...EMPTY_SNAPSHOT, mode: 'trace' }).badge).toBe('TRACE');
  });
  it('is LIVE only with a fresh hardware sample', () => {
    expect(classifySource(live()).badge).toBe('LIVE');
    expect(classifySource(live({ stale: true })).badge).toBe('STALE');
    expect(classifySource(live({ hasFrame: false })).badge).toBe('STALE');
    expect(classifySource(live({ bridge: bridge({ source: 'LEGACY_POLL' }) })).label).toContain('LEGACY_POLL');
  });
});

describe('separate diagnostic codes', () => {
  it('distinguishes RESTORE_REQUIRED, MOTION_NOT_PERMITTED and staleness', () => {
    expect(diagnosticCodes(live({ restore: false }))).toEqual(['RESTORE_REQUIRED']);
    expect(diagnosticCodes(live({ permit: false }))).toEqual(['MOTION_NOT_PERMITTED']);
    expect(diagnosticCodes(live({ stale: true }))).toEqual(['TELEMETRY_STALE']);
    expect(diagnosticCodes(live({ commissioningRestore: 'NOT_RUN' }))).toContain('RESTORE_REQUIRED');
    expect(diagnosticCodes(live())).toEqual([]);
  });
  it('reports transport and serial faults by name', () => {
    expect(diagnosticCodes(live({ transportError: 'HTTP_TIMEOUT' }))).toContain('HTTP_TIMEOUT');
    expect(diagnosticCodes(live({ transportError: 'BRIDGE_UNREACHABLE' }))).toContain('BRIDGE_UNREACHABLE');
    expect(diagnosticCodes(live({ bridge: bridge({ serial_state: 'SERIAL_OWNER_BUSY' }) }))).toContain('SERIAL_OWNER_BUSY');
    expect(diagnosticCodes(live({ bridge: bridge({ serial_state: 'SERIAL_DISCONNECTED' }) }))).toContain('SERIAL_DISCONNECTED');
    expect(diagnosticCodes(live({ bridge: bridge({ source: 'LEGACY_POLL' }) }))).toContain('LEGACY_POLL');
    expect(diagnosticCodes(live({ available: false }))).toContain('NO_CONTROLLER_DATA');
    expect(diagnosticCodes({ ...EMPTY_SNAPSHOT, assetsMissing: true })).toEqual(['ASSETS_MISSING']);
  });
});

describe('firmware @TEL status', () => {
  it('parses joints, read-only servo feedback and the commissioning view', () => {
    const model = parseDashboardModel(telStatus(1000, 990));
    expect(model.servos).toHaveLength(12);
    expect(model.servos[0]?.energized).toBe(false);
    expect(model.commissioning?.mode).toBe('DIAGNOSTIC');
    const stream = new MotionTelemetryStream();
    expect(stream.accept(telStatus(1000, 990), 0)).toBe(true);
    expect(stream.frame?.actual).not.toBeNull();
  });
  it('keeps missing actual angles null before RESTORE (never zeros)', () => {
    const stream = new MotionTelemetryStream();
    expect(stream.accept(telStatus(1000, 990, 7, false), 0)).toBe(true);
    expect(stream.frame?.actual).toBeNull();
  });
  it('restarts freshness after a Controller reboot (uptime goes backwards)', () => {
    const stream = new MotionTelemetryStream(), feed = new LiveFeedState();
    expect(feed.accept(stream, { ...telStatus(90000, 89990), bridge: { boot_nonce: 7 } }, 0)).toBe(true);
    expect(stream.accept(telStatus(2000, 1990), 10)).toBe(false); // strict stream alone would stall
    expect(feed.accept(stream, { ...telStatus(2000, 1990, 9), bridge: { boot_nonce: 9 } }, 20)).toBe(true);
    expect(feed.resets).toBe(1);
  });
});
