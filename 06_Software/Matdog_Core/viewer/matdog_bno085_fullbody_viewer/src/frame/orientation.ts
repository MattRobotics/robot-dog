/**
 * MATDOG orientation / coordinate-frame mapping.
 *
 * ---------------------------------------------------------------------------
 * FRAME CHAIN (frozen, see README "Coordinate-frame mapping")
 * ---------------------------------------------------------------------------
 *
 *   SH2_ROTATION_VECTOR quaternion   ->   MATDOG base_link   ->   Three.js root
 *
 * 1. MATDOG base_link convention (canonical URDF REV00):
 *        +X = forward, +Y = left, +Z = up   (right-handed)
 *
 * 2. BNO085 / SH2 axes, as frozen by hardware Phase D validation:
 *        SH2 +X -> base_link +X
 *        SH2 +Y -> base_link +Y
 *        SH2 +Z -> base_link +Z
 *    Identical axes, identical signs. Therefore NO axis permutation and NO
 *    sign flip is applied anywhere in this file. The SH2 quaternion already
 *    IS the base_link orientation quaternion.
 *
 * 3. The Three.js world of this viewer is deliberately configured Z-up with
 *    the same handedness and the same axis roles as base_link
 *    (camera.up = +Z, ground plane = XY). Therefore the mapping
 *    base_link -> Three.js root object is also the identity, and the only
 *    conversion performed is the storage-order difference:
 *
 *        SH2 / URDF order : (w, x, y, z)
 *        THREE.Quaternion : (x, y, z, w)
 *
 * Consequences, matching the frozen hardware definitions:
 *        +ROLL  = rotation about +X : left side of the robot rises
 *        +PITCH = rotation about +Y : nose/front goes down
 *        +YAW   = rotation about +Z : nose turns left seen from above
 *
 * q and -q describe the same rotation. Rendering is therefore insensitive to
 * the sign, but the raw stream is still hemisphere-aligned (see
 * `alignSignWithPrevious`) so that logged/displayed values do not flip
 * arbitrarily and any future interpolation stays on the short arc.
 */

import { Quaternion } from 'three';

/** A raw SH2 rotation-vector quaternion in the sensor's (w, x, y, z) order. */
export interface Sh2Quaternion {
  readonly w: number;
  readonly x: number;
  readonly y: number;
  readonly z: number;
}

export const SH2_IDENTITY: Sh2Quaternion = { w: 1, x: 0, y: 0, z: 0 };

export function isFiniteQuaternion(q: Sh2Quaternion): boolean {
  return (
    Number.isFinite(q.w) && Number.isFinite(q.x) && Number.isFinite(q.y) && Number.isFinite(q.z)
  );
}

export function quaternionNorm(q: Sh2Quaternion): number {
  return Math.hypot(q.w, q.x, q.y, q.z);
}

/** Normalises to unit length; returns identity for a degenerate input. */
export function normalizeSh2(q: Sh2Quaternion): Sh2Quaternion {
  const norm = quaternionNorm(q);
  if (!Number.isFinite(norm) || norm < 1e-9) return SH2_IDENTITY;
  return { w: q.w / norm, x: q.x / norm, y: q.y / norm, z: q.z / norm };
}

/**
 * SH2 (w, x, y, z) -> THREE.Quaternion (x, y, z, w).
 * This is the ONLY conversion between the sensor and the rendered root; it is
 * a storage-order change, not an axis change.
 */
export function sh2ToThree(q: Sh2Quaternion, target = new Quaternion()): Quaternion {
  return target.set(q.x, q.y, q.z, q.w);
}

/** THREE.Quaternion -> SH2 (w, x, y, z) order. */
export function threeToSh2(q: Quaternion): Sh2Quaternion {
  return { w: q.w, x: q.x, y: q.y, z: q.z };
}

/**
 * Returns `q` or `-q`, whichever lies in the same hemisphere as `previous`.
 * Both represent the identical rotation; this only keeps the numeric stream
 * continuous so the readout does not jump and interpolation stays short-arc.
 */
export function alignSignWithPrevious(
  previous: Sh2Quaternion | null,
  q: Sh2Quaternion,
): Sh2Quaternion {
  if (previous === null) return q;
  const dot = previous.w * q.w + previous.x * q.x + previous.y * q.y + previous.z * q.z;
  if (dot >= 0) return q;
  return { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
}

export function conjugate(q: Sh2Quaternion): Sh2Quaternion {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

/** Hamilton product, (w, x, y, z) order. */
export function multiply(a: Sh2Quaternion, b: Sh2Quaternion): Sh2Quaternion {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

/**
 * Viewer-only recentering.
 *
 *     q_viewer = conjugate(q_reference) * q_raw
 *
 * `q_reference` is the raw quaternion captured when the operator pressed
 * Recenter. The result is the robot's orientation expressed relative to the
 * pose it held at that instant, so:
 *   - at the moment of recentering the robot renders exactly at identity
 *     (nose along viewer +X, left side along +Y, up along +Z);
 *   - a subsequent rotation of the physical robot about its own body axis
 *     appears as the same rotation about the corresponding viewer axis,
 *     which is what the +ROLL / +PITCH / +YAW acceptance checks require.
 *
 * Nothing is sent to the sensor and no calibration state is affected: this is
 * pure host-side quaternion algebra.
 */
export function applyReference(
  raw: Sh2Quaternion,
  reference: Sh2Quaternion | null,
): Sh2Quaternion {
  if (reference === null) return raw;
  return multiply(conjugate(reference), raw);
}

/**
 * Builds an SH2-order quaternion from MATDOG roll/pitch/yaw in radians using
 * the URDF fixed-axis convention R = Rz(yaw) * Ry(pitch) * Rx(roll).
 * Used by the synthetic orientation source and by the frame unit tests.
 */
export function rollPitchYawToSh2(roll: number, pitch: number, yaw: number): Sh2Quaternion {
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
  };
}

/** Inverse of `rollPitchYawToSh2`, for readouts. Angles in radians. */
export function sh2ToRollPitchYaw(q: Sh2Quaternion): { roll: number; pitch: number; yaw: number } {
  const { w, x, y, z } = normalizeSh2(q);
  const sinRoll = 2 * (w * x + y * z);
  const cosRoll = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinRoll, cosRoll);

  const sinPitch = 2 * (w * y - z * x);
  const pitch = Math.abs(sinPitch) >= 1 ? Math.sign(sinPitch) * (Math.PI / 2) : Math.asin(sinPitch);

  const sinYaw = 2 * (w * z + x * y);
  const cosYaw = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinYaw, cosYaw);

  return { roll, pitch, yaw };
}

export const RAD_TO_DEG = 180 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180;
