/**
 * Read-only parser for the optional dashboard fields of a Controller / host
 * bridge status object (readiness, commissioning, per-servo feedback, IMU,
 * bridge health). Every key is optional and unknown values are kept as text.
 *
 * This module only reads telemetry that the Controller already reported (the
 * per-servo enable flag included). It has no network, serial, DOM or command
 * path; tests/safetyBoundary.test.ts enforces that.
 */
import { JOINT_NAMES, type MotionFrame } from '../motion/telemetry';
import { parseBridgeInfo, type BridgeInfo } from '../motion/liveStatus';

const RAD_TO_DEG = 180 / Math.PI;

export interface ServoFeedback {
  readonly slot: number;
  readonly busId: number | null;
  readonly raw: number | null;
  readonly mapped: boolean | null;
  readonly energized: boolean | null;
  readonly tempC: number | null;
  readonly voltageV: number | null;
  readonly status: number | null;
  readonly ageMs: number | null;
  /** Legacy poll only: per-joint measured angle when the full channel is incomplete. */
  readonly actualRad: number | null;
}

export interface Commissioning {
  readonly mode: string | null;
  readonly pose: string | null;
  readonly poseMaxDevRad: number | null;
  readonly poseJoint: number | null;
  readonly entry: string | null;
  readonly limits: string | null;
  readonly approvedJoints: number | null;
  readonly commissioningJoints: number | null;
  readonly primeClampMask: number | null;
  readonly reason: string | null;
  readonly restore: string | null;
  readonly segment: number | null;
  readonly phase: string | null;
  readonly session: string | null;
}

export interface Readiness {
  readonly restore: boolean | null;
  readonly calibrationAccepted: boolean | null;
  readonly limitsApproved: boolean | null;
  readonly permit: boolean | null;
  readonly supportObservable: boolean | null;
  readonly hardwareApproved: boolean | null;
}

export interface DashboardModel {
  readonly available: boolean;
  readonly source: string | null;
  readonly nowMs: number | null;
  readonly reason: string;
  readonly executorState: string | null;
  readonly standState: string | null;
  readonly trackingErrorRad: number | null;
  readonly readiness: Readiness | null;
  readonly commissioning: Commissioning | null;
  readonly localSession: { readonly state: string | null; readonly reason: string | null } | null;
  readonly validation: { readonly scope: string | null; readonly segment: number | null; readonly capable: boolean | null } | null;
  readonly servos: readonly (ServoFeedback | null)[];
  readonly jointAtMs: readonly (number | null)[] | null;
  readonly rawTicks: readonly (number | null)[] | null;
  readonly imu: { readonly valid: boolean | null; readonly stampMs: number | null } | null;
  readonly power: { readonly state: string | null; readonly socPct: number | null; readonly valid: boolean | null } | null;
  readonly legacy: { readonly jointsSeen: number | null; readonly cycleMs: number | null } | null;
  readonly bridge: BridgeInfo | null;
}

function record(value: unknown): Record<string, unknown> | null {
  return typeof value === 'object' && value !== null && !Array.isArray(value) ? value as Record<string, unknown> : null;
}
function num(value: unknown): number | null { return typeof value === 'number' && Number.isFinite(value) ? value : null; }
function bool(value: unknown): boolean | null { return typeof value === 'boolean' ? value : null; }
function str(value: unknown): string | null { return typeof value === 'string' ? value.slice(0, 120) : null; }
function numArray(value: unknown): (number | null)[] | null {
  return Array.isArray(value) && value.length === 12 ? value.map(num) : null;
}

function servo(value: unknown, slot: number): ServoFeedback | null {
  const entry = record(value);
  if (!entry) return null;
  const enable = num(entry.energized);
  const volt = num(entry.volt_dv);
  return {
    slot: num(entry.slot) ?? slot, busId: num(entry.id), raw: num(entry.raw), mapped: bool(entry.mapped),
    energized: enable === null ? bool(entry.energized) : enable !== 0, tempC: num(entry.temp),
    voltageV: volt === null ? null : volt / 10, status: num(entry.status), ageMs: num(entry.age_ms),
    actualRad: num(entry.actual_rad),
  };
}

export function parseDashboardModel(value: unknown): DashboardModel {
  const status = record(value) ?? {};
  const telemetry = record(status.telemetry);
  const readiness = record(status.readiness);
  const commissioning = record(status.commissioning);
  const session = record(status.local_session);
  const validation = record(status.validation);
  const imu = record(status.imu);
  const power = record(status.power);
  const legacy = record(status.legacy);
  const servos = Array.isArray(status.servos) ? status.servos : [];
  return {
    available: status.available !== false && telemetry !== null,
    source: str(telemetry?.source), nowMs: num(status.now_ms), reason: str(status.reason) ?? '',
    executorState: str(status.executor_state), standState: str(status.stand_state),
    trackingErrorRad: num(status.tracking_error_rad) ?? num(telemetry?.tracking_error_rad),
    readiness: readiness && {
      restore: bool(readiness.restore), calibrationAccepted: bool(readiness.calibration_accepted),
      limitsApproved: bool(readiness.limits_approved), permit: bool(readiness.permit),
      supportObservable: bool(readiness.support_observable), hardwareApproved: bool(readiness.hardware_approved),
    },
    commissioning: commissioning && {
      mode: str(commissioning.mode), pose: str(commissioning.pose), poseMaxDevRad: num(commissioning.pose_max_dev_rad),
      poseJoint: num(commissioning.pose_joint), entry: str(commissioning.entry), limits: str(commissioning.limits),
      approvedJoints: num(commissioning.approved_joints), commissioningJoints: num(commissioning.commissioning_joints),
      primeClampMask: num(commissioning.prime_clamp_mask), reason: typeof commissioning.reason === 'string' ? commissioning.reason.slice(0, 120) : null,
      restore: str(commissioning.restore), segment: num(commissioning.segment), phase: str(commissioning.phase),
      session: str(commissioning.session),
    },
    localSession: session && { state: str(session.state), reason: str(session.reason) },
    validation: validation && { scope: str(validation.scope), segment: num(validation.segment), capable: bool(validation.capable) },
    servos: JOINT_NAMES.map((_, slot) => servo(servos.find((entry) => record(entry)?.slot === slot) ?? servos[slot], slot)),
    jointAtMs: numArray(telemetry?.joint_at_ms), rawTicks: numArray(telemetry?.raw_ticks),
    imu: imu && { valid: bool(imu.valid), stampMs: num(imu.stamp_ms) },
    power: power && { state: str(power.state), socPct: num(power.soc_pct), valid: bool(power.valid) },
    legacy: legacy && { jointsSeen: num(legacy.joints_seen), cycleMs: num(legacy.cycle_ms) },
    bridge: parseBridgeInfo(status.bridge),
  };
}

export const JOINT_COLUMNS = ['joint', 'id', 'actual °', 'target °', 'err °', 'raw', 'energized', '°C', 'V', 'age ms'] as const;

export interface JointLimit { readonly lower: number | null; readonly upper: number | null }
export interface JointRow {
  readonly name: string;
  readonly cells: readonly string[];
  readonly unmapped: boolean;
  readonly old: boolean;
  readonly outsideLimit: boolean;
}

const LABEL = (name: string): string => {
  const [leg, part] = name.split('_');
  return `${(leg ?? '').toUpperCase()} ${part === 'upper' ? 'upper' : part === 'lower' ? 'lower' : 'hip'}`;
};
const deg = (rad: number | null): string => rad === null ? '—' : (rad * RAD_TO_DEG).toFixed(1);
const show = (value: number | null, digits = 0): string => value === null ? '—' : value.toFixed(digits);

/**
 * One row per Controller slot (LF, RF, RH, LH x hip/upper/lower). Measured and
 * target channels stay separate; a missing value is shown as an em dash, never 0.
 */
export function jointRows(model: DashboardModel | null, frame: MotionFrame | null,
  limits: Readonly<Record<string, JointLimit>> = {}, staleAfterMs = 1500): JointRow[] {
  return JOINT_NAMES.map((name, slot) => {
    const feedback = model?.servos[slot] ?? null;
    const actual = frame?.actual?.[name] ?? (feedback?.mapped === false ? null : feedback?.actualRad ?? null);
    const target = frame?.target?.[name] ?? null;
    const raw = feedback?.raw ?? model?.rawTicks?.[slot] ?? null;
    const stamp = model?.jointAtMs?.[slot] ?? null;
    const age = feedback?.ageMs ?? (stamp !== null && model?.nowMs != null ? Math.max(0, model.nowMs - stamp) : null);
    const limit = limits[name];
    const outsideLimit = actual !== null && limit !== undefined &&
      ((limit.lower !== null && actual < limit.lower - 1e-9) || (limit.upper !== null && actual > limit.upper + 1e-9));
    return {
      name,
      cells: [LABEL(name), show(feedback?.busId ?? null), deg(actual), deg(target),
        actual !== null && target !== null ? ((actual - target) * RAD_TO_DEG).toFixed(2) : '—', show(raw),
        feedback?.energized == null ? '—' : feedback.energized ? 'ON' : 'off', show(feedback?.tempC ?? null),
        show(feedback?.voltageV ?? null, 1), show(age)],
      unmapped: feedback?.mapped === false,
      old: age !== null && age > staleAfterMs,
      outsideLimit,
    };
  });
}
