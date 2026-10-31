/** Version 1 Controller motion telemetry, strictly read-only. */
import type { Sh2Quaternion } from '../frame/orientation';

// Controller array order, explicitly distinct from alphabetical sorting.
export const JOINT_NAMES = ['lf', 'rf', 'rh', 'lh'].flatMap((leg) =>
  [`${leg}_hip_joint`, `${leg}_upper_leg_joint`, `${leg}_lower_leg_joint`]);
export const LEGS = ['LF', 'RF', 'RH', 'LH'] as const;
export type Leg = typeof LEGS[number];
export type Point3 = readonly [number, number, number];
export interface FootTelemetry {
  readonly leg: Leg;
  readonly position: Point3 | null;
  readonly contact: 'support' | 'swing' | 'unknown';
  readonly phase: number | null;
  readonly trajectory: readonly Point3[];
}
export interface MotionFrame {
  readonly controllerNowMs: number;
  readonly sampledAtMs: number;
  readonly executorState: string;
  readonly standState: string;
  readonly owner: string;
  readonly fault: string;
  readonly reason: string;
  readonly quaternion: Sh2Quaternion | null;
  readonly actual: Readonly<Record<string, number>> | null;
  readonly target: Readonly<Record<string, number>> | null;
  /** Defined relative to the measured/declared support plane; never derived from IMU. */
  readonly baseHeightM: number | null;
  readonly basePositionM: Point3 | null;
  readonly feet: readonly FootTelemetry[];
  readonly trackingErrorRad: number | null;
  /**
   * Frame-level staleness of MEASURED channels. A legacy Controller `stale`
   * that only reflects an absent (null) joint channel is reported through
   * `jointsStale`, never as a stale transport/IMU (14 Oct counterexample).
   */
  readonly stale: boolean;
  /** Joint feedback unusable for motion (legacy Controller `stale`). */
  readonly jointsStale: boolean;
  /** Per-joint measured radians; null = not measured (never 0). */
  readonly actualByJoint: Readonly<Record<string, number | null>> | null;
  /** Raw ST3215 ticks in Controller order, provenance only (never converted here). */
  readonly rawTicks: readonly number[] | null;
  readonly channels: MotionChannels;
  /** Issue #48 RESTORE state as reported by the Controller (null on older firmware). */
  readonly restore: { readonly decision: string; readonly reason: string; readonly generation: number | null } | null;
}
/** Independent freshness axes; null = not reported by this firmware. */
export interface MotionChannels {
  readonly imuValid: boolean | null;
  readonly imuAgeMs: number | null;
  readonly rawCount: number | null;
  readonly mappedCount: number | null;
}
function object(value: unknown): Record<string, unknown> | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value)
    ? value as Record<string, unknown> : null;
}
function finite(value: unknown): value is number { return typeof value === 'number' && Number.isFinite(value); }
function nonnegative(value: unknown): value is number { return finite(value) && value >= 0; }
function label(value: unknown, fallback: string): string {
  return typeof value === 'string' ? value.slice(0, 160) : fallback;
}
function point(value: unknown): Point3 | null {
  return Array.isArray(value) && value.length === 3 && value.every(finite)
    ? [value[0] as number, value[1] as number, value[2] as number] : null;
}
function joints(value: unknown, names: unknown): Record<string, number> | null {
  if (value == null) return null;
  const order = names == null ? JOINT_NAMES : names;
  if (!Array.isArray(order) || order.length !== 12 || new Set(order).size !== 12 ||
    !order.every((name) => typeof name === 'string' && JOINT_NAMES.includes(name))) return null;
  if (!Array.isArray(value) || value.length !== 12 || !value.every(finite)) return null;
  return Object.fromEntries(order.map((name, index) => [name as string, value[index] as number]));
}
function jointsByName(value: unknown, names: unknown): Record<string, number | null> | null {
  if (!Array.isArray(value) || value.length !== 12) return null;
  const order = names == null ? JOINT_NAMES : names;
  if (!Array.isArray(order) || order.length !== 12 || new Set(order).size !== 12 ||
    !order.every((name) => typeof name === 'string' && JOINT_NAMES.includes(name))) return null;
  if (!value.every((v) => v === null || finite(v))) return null;
  return Object.fromEntries(order.map((name, index) => [name as string, value[index] as number | null]));
}
function channelsOf(status: Record<string, unknown>, telemetry: Record<string, unknown>,
  actualByJoint: Record<string, number | null> | null, rawTicks: readonly number[] | null, quaternion: unknown): MotionChannels {
  const c = object(status.channels), imu = object(c?.imu), raw = object(c?.raw), mapped = object(c?.mapped);
  const imuServo = object(status.imu);
  return {
    imuValid: typeof imu?.valid === 'boolean' ? imu.valid : typeof imuServo?.valid === 'boolean' ? imuServo.valid : quaternion != null ? true : null,
    imuAgeMs: nonnegative(imu?.age_ms) ? imu.age_ms : nonnegative(imuServo?.stamp_ms) && nonnegative(status.now_ms) && status.now_ms >= imuServo.stamp_ms
      ? status.now_ms - imuServo.stamp_ms : null,
    rawCount: nonnegative(raw?.count) ? raw.count : rawTicks ? rawTicks.length : null,
    mappedCount: nonnegative(mapped?.count) ? mapped.count
      : actualByJoint ? Object.values(actualByJoint).filter((v) => v !== null).length
        : Array.isArray(telemetry.actual_rad) ? 12 : telemetry.actual_rad === null ? 0 : null,
  };
}
export function parseMotionFrame(value: unknown): MotionFrame | null {
  const status = object(value), telemetry = object(status?.telemetry);
  if (!status || status.version !== 1 || !telemetry || !nonnegative(status.now_ms) ||
    !nonnegative(telemetry.sampled_at_ms) || telemetry.sampled_at_ms > status.now_ms + 5) return null;
  let quaternion: Sh2Quaternion | null = null;
  const q = telemetry.quaternion;
  if (Array.isArray(q) && q.length === 4 && q.every(finite) && Math.hypot(...q as number[]) > 1e-9) {
    const norm = Math.hypot(...q as number[]);
    quaternion = { w: q[0]! / norm, x: q[1]! / norm, y: q[2]! / norm, z: q[3]! / norm };
  }
  const basePosition = point(telemetry.base_position_m);
  const baseHeight = nonnegative(telemetry.base_height_m) ? telemetry.base_height_m : null;
  if (telemetry.base_position_m != null && (!basePosition || basePosition[2] < 0 ||
    baseHeight !== null && Math.abs(basePosition[2] - baseHeight) > 1e-9)) return null;
  const feet = LEGS.map((leg): FootTelemetry => {
    const source = Array.isArray(telemetry.feet)
      ? object(telemetry.feet.find((entry) => object(entry)?.leg === leg)) : null;
    const contact = source?.contact === 'support' || source?.contact === 'swing' ? source.contact : 'unknown';
    return { leg, position: point(source?.position_m), contact,
      phase: finite(source?.phase) && source.phase >= 0 && source.phase <= 1 ? source.phase : null,
      trajectory: Array.isArray(source?.trajectory_m)
        ? source.trajectory_m.slice(-64).map(point).filter((p): p is Point3 => p !== null) : [] };
  });
  const actual = joints(telemetry.actual_rad, telemetry.joint_names);
  const actualByJoint = actual ?? jointsByName(telemetry.actual_rad_by_joint, telemetry.joint_names);
  const rawTicks = Array.isArray(telemetry.raw_ticks) && telemetry.raw_ticks.length === 12 &&
    telemetry.raw_ticks.every((t) => Number.isInteger(t) && (t as number) >= 0 && (t as number) < 4096)
    ? telemetry.raw_ticks as number[] : null;
  const legacyStale = status.stale === true || telemetry.stale === true;
  const anyMeasured = actualByJoint !== null && Object.values(actualByJoint).some((v) => v !== null);
  return {
    controllerNowMs: status.now_ms, sampledAtMs: telemetry.sampled_at_ms,
    executorState: label(status.executor_state, 'UNKNOWN'), standState: label(status.stand_state, 'UNKNOWN'),
    owner: label(status.owner, 'NONE'), fault: status.fault === true ? 'FAULT' : label(status.fault, 'NONE'), reason: label(status.reason, ''),
    quaternion, actual, actualByJoint, rawTicks,
    channels: channelsOf(status, telemetry, actualByJoint, rawTicks, telemetry.quaternion),
    restore: object(status.restore) ? { decision: label(object(status.restore)!.decision, 'UNKNOWN'), reason: label(object(status.restore)!.reason, ''),
      generation: nonnegative(object(status.restore)!.generation) ? object(status.restore)!.generation as number : null } : null,
    jointsStale: legacyStale || actual === null,
    target: joints(telemetry.target_rad, telemetry.joint_names),
    baseHeightM: baseHeight ?? basePosition?.[2] ?? null,
    basePositionM: basePosition,
    feet, trackingErrorRad: nonnegative(telemetry.tracking_error_rad) ? telemetry.tracking_error_rad : nonnegative(status.tracking_error_rad) ? status.tracking_error_rad : null,
    // A stale flag can only age data that exists: with no measured joint the
    // flag describes the missing joint channel, not the transport or the IMU.
    // Firmware that declares stale_scope=MOTION_JOINT_FEEDBACK reports channel
    // ages separately; its `stale` never ages the frame as a whole.
    // A host bridge reports its own serial/sample freshness as transport_stale.
    stale: status.transport_stale === true || (status.stale_scope === 'MOTION_JOINT_FEEDBACK' ? false : legacyStale && anyMeasured),
  };
}
/** Freshness combines Controller sample age and local transport age. No wall clock conversion. */
export class MotionTelemetryStream {
  frame: MotionFrame | null = null;
  receivedAtMs = 0;
  readonly staleAfterMs = 1500;
  accept(value: unknown, receivedAtMs: number): boolean {
    const frame = parseMotionFrame(value);
    if (!frame || !nonnegative(receivedAtMs) ||
      (this.frame && (frame.sampledAtMs < this.frame.sampledAtMs || frame.controllerNowMs < this.frame.controllerNowMs))) return false;
    // A cached/stalled response cannot renew sample freshness merely by arriving again.
    const clockAdvanced = !this.frame || frame.controllerNowMs > this.frame.controllerNowMs;
    this.frame = frame; if (clockAdvanced) this.receivedAtMs = receivedAtMs; return true;
  }
  reset(): void { this.frame = null; this.receivedAtMs = 0; }
  ageMs(nowMs: number): number | null {
    if (!this.frame) return null;
    return Math.max(0, this.frame.controllerNowMs - this.frame.sampledAtMs) + Math.max(0, nowMs - this.receivedAtMs);
  }
  isStale(nowMs: number): boolean {
    const age = this.ageMs(nowMs);
    return age === null || age > this.staleAfterMs || this.frame?.stale === true;
  }
}
