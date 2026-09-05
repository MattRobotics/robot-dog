/**
 * Synthetic orientation source: lets the +ROLL / +PITCH / +YAW conventions be
 * verified before any hardware is connected.
 *
 * It produces the same `Sh2Quaternion` values the parser would produce from a
 * real RV line, so it exercises the identical downstream path. It never opens
 * a serial port and never emits anything towards hardware.
 *
 * Activation: `?sim=1`, or any of `?roll=`, `?pitch=`, `?yaw=` (degrees).
 */

import { DEG_TO_RAD, rollPitchYawToSh2, type Sh2Quaternion } from '../frame/orientation';

export interface SyntheticAngles {
  rollDeg: number;
  pitchDeg: number;
  yawDeg: number;
}

export interface SyntheticConfig {
  readonly enabled: boolean;
  readonly angles: SyntheticAngles;
}

export const SYNTHETIC_PRESETS: readonly { label: string; angles: SyntheticAngles; expectation: string }[] = [
  { label: 'zero', angles: { rollDeg: 0, pitchDeg: 0, yawDeg: 0 }, expectation: 'level, nose along +X' },
  { label: '+roll 25°', angles: { rollDeg: 25, pitchDeg: 0, yawDeg: 0 }, expectation: 'LEFT side rises' },
  { label: '+pitch 25°', angles: { rollDeg: 0, pitchDeg: 25, yawDeg: 0 }, expectation: 'NOSE goes down' },
  { label: '+yaw 30°', angles: { rollDeg: 0, pitchDeg: 0, yawDeg: 30 }, expectation: 'NOSE turns left (seen from above)' },
];

function readAngle(params: URLSearchParams, key: string): number | null {
  const raw = params.get(key);
  if (raw === null) return null;
  const value = Number(raw);
  return Number.isFinite(value) ? value : null;
}

export function readSyntheticConfig(search: string): SyntheticConfig {
  const params = new URLSearchParams(search);
  const rollDeg = readAngle(params, 'roll');
  const pitchDeg = readAngle(params, 'pitch');
  const yawDeg = readAngle(params, 'yaw');
  const hasAngle = rollDeg !== null || pitchDeg !== null || yawDeg !== null;
  const enabled = params.get('sim') === '1' || params.get('sim') === 'true' || hasAngle;
  return {
    enabled,
    angles: { rollDeg: rollDeg ?? 0, pitchDeg: pitchDeg ?? 0, yawDeg: yawDeg ?? 0 },
  };
}

export function anglesToSh2(angles: SyntheticAngles): Sh2Quaternion {
  return rollPitchYawToSh2(
    angles.rollDeg * DEG_TO_RAD,
    angles.pitchDeg * DEG_TO_RAD,
    angles.yawDeg * DEG_TO_RAD,
  );
}

/**
 * Renders synthetic angles as the exact RV line the firmware would print, so
 * the serial parser itself can be exercised without hardware.
 */
export function anglesToRvLine(angles: SyntheticAngles, count = 0): string {
  const q = anglesToSh2(angles);
  return (
    `RV w=${q.w.toFixed(6)} x=${q.x.toFixed(6)} y=${q.y.toFixed(6)} z=${q.z.toFixed(6)} ` +
    `accuracy_rad=0.000000 status=3 count=${count}`
  );
}
