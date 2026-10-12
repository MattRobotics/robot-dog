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
  readonly stale: boolean;
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
  return {
    controllerNowMs: status.now_ms, sampledAtMs: telemetry.sampled_at_ms,
    executorState: label(status.executor_state, 'UNKNOWN'), standState: label(status.stand_state, 'UNKNOWN'),
    owner: label(status.owner, 'NONE'), fault: status.fault === true ? 'FAULT' : label(status.fault, 'NONE'), reason: label(status.reason, ''),
    quaternion, actual: joints(telemetry.actual_rad, telemetry.joint_names),
    target: joints(telemetry.target_rad, telemetry.joint_names),
    baseHeightM: baseHeight ?? basePosition?.[2] ?? null,
    basePositionM: basePosition,
    feet, trackingErrorRad: nonnegative(telemetry.tracking_error_rad) ? telemetry.tracking_error_rad : nonnegative(status.tracking_error_rad) ? status.tracking_error_rad : null,
    stale: status.stale === true || telemetry.stale === true,
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
