/**
 * Coordinate-frame and recenter validation.
 *
 * These tests encode the FROZEN MATDOG hardware conventions:
 *   base_link: +X forward, +Y left, +Z up
 *   +ROLL  (about +X) -> left side of the robot rises
 *   +PITCH (about +Y) -> nose goes down
 *   +YAW   (about +Z) -> nose turns left seen from above
 * and the frozen BNO085 axis alignment SH2 +X/+Y/+Z -> base_link +X/+Y/+Z
 * with identical signs, i.e. no permutation and no sign flip anywhere.
 */

import { Quaternion, Vector3 } from 'three';
import { describe, expect, it } from 'vitest';

import {
  alignSignWithPrevious,
  applyReference,
  conjugate,
  DEG_TO_RAD,
  multiply,
  normalizeSh2,
  rollPitchYawToSh2,
  sh2ToRollPitchYaw,
  sh2ToThree,
  threeToSh2,
  type Sh2Quaternion,
} from '../src/frame/orientation';
import { OrientationController } from '../src/frame/orientationController';
import { anglesToRvLine, anglesToSh2 } from '../src/dev/syntheticSource';
import { parseSensorLine, type RvRecord } from '../src/serial/lineParser';

const NOSE = new Vector3(1, 0, 0); // base_link +X, forward
const LEFT = new Vector3(0, 1, 0); // base_link +Y, left
const UP = new Vector3(0, 0, 1); // base_link +Z, up

/** Rotates a body-frame direction into the viewer world with an SH2 quaternion. */
function rotate(q: Sh2Quaternion, direction: Vector3): Vector3 {
  return direction.clone().applyQuaternion(sh2ToThree(q));
}

describe('SH2 -> base_link -> three.js mapping', () => {
  it('is a storage-order change only: (w,x,y,z) -> (x,y,z,w)', () => {
    const q: Sh2Quaternion = { w: 0.642151, x: 0.005371, y: 0.016052, z: -0.766357 };
    const three = sh2ToThree(q);
    expect(three.x).toBe(q.x);
    expect(three.y).toBe(q.y);
    expect(three.z).toBe(q.z);
    expect(three.w).toBe(q.w);
    expect(threeToSh2(three)).toEqual(q);
  });

  it('leaves the identity quaternion with the robot nose along viewer +X', () => {
    const identity = rollPitchYawToSh2(0, 0, 0);
    expect(rotate(identity, NOSE).x).toBeCloseTo(1, 12);
    expect(rotate(identity, LEFT).y).toBeCloseTo(1, 12);
    expect(rotate(identity, UP).z).toBeCloseTo(1, 12);
  });
});

describe('frozen MATDOG rotation conventions', () => {
  it('+ROLL raises the LEFT side of the robot', () => {
    const q = rollPitchYawToSh2(30 * DEG_TO_RAD, 0, 0);
    const left = rotate(q, LEFT);
    expect(left.z).toBeGreaterThan(0.4); // left side went up
    expect(rotate(q, NOSE).x).toBeCloseTo(1, 9); // nose unaffected
    // Symmetry: -roll must lower the left side by the same amount.
    expect(rotate(rollPitchYawToSh2(-30 * DEG_TO_RAD, 0, 0), LEFT).z).toBeCloseTo(-left.z, 12);
  });

  it('+PITCH takes the NOSE down', () => {
    const q = rollPitchYawToSh2(0, 25 * DEG_TO_RAD, 0);
    const nose = rotate(q, NOSE);
    expect(nose.z).toBeLessThan(-0.35); // nose below the horizon
    expect(nose.x).toBeGreaterThan(0); // still pointing forward
    expect(rotate(q, LEFT).y).toBeCloseTo(1, 9); // left unaffected
  });

  it('+YAW turns the NOSE to the left seen from above', () => {
    const q = rollPitchYawToSh2(0, 0, 40 * DEG_TO_RAD);
    const nose = rotate(q, NOSE);
    expect(nose.y).toBeGreaterThan(0.5); // nose swung towards +Y (left)
    expect(nose.z).toBeCloseTo(0, 9); // stayed level
    expect(rotate(q, UP).z).toBeCloseTo(1, 9);
  });

  it('round-trips roll/pitch/yaw through the quaternion', () => {
    const cases = [
      [10, -20, 35],
      [-45, 15, -120],
      [0, 0, 179],
      [5, 5, 5],
    ] as const;
    for (const [rollDeg, pitchDeg, yawDeg] of cases) {
      const q = rollPitchYawToSh2(rollDeg * DEG_TO_RAD, pitchDeg * DEG_TO_RAD, yawDeg * DEG_TO_RAD);
      const rpy = sh2ToRollPitchYaw(q);
      expect(rpy.roll / DEG_TO_RAD).toBeCloseTo(rollDeg, 6);
      expect(rpy.pitch / DEG_TO_RAD).toBeCloseTo(pitchDeg, 6);
      expect(rpy.yaw / DEG_TO_RAD).toBeCloseTo(yawDeg, 6);
    }
  });
});

describe('q and -q equivalence', () => {
  it('renders identically', () => {
    const q = rollPitchYawToSh2(0.3, -0.6, 1.4);
    const negated: Sh2Quaternion = { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
    for (const direction of [NOSE, LEFT, UP]) {
      const a = rotate(q, direction);
      const b = rotate(negated, direction);
      expect(a.distanceTo(b)).toBeLessThan(1e-12);
    }
  });

  it('is hemisphere-aligned so the numeric stream stays continuous', () => {
    const q = rollPitchYawToSh2(0.3, -0.6, 1.4);
    const negated: Sh2Quaternion = { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
    expect(alignSignWithPrevious(q, negated)).toEqual(q);
    expect(alignSignWithPrevious(q, q)).toEqual(q);
    expect(alignSignWithPrevious(null, negated)).toEqual(negated);
  });

  it('does not make the controller jump when the sensor flips sign', () => {
    const controller = new OrientationController();
    const q = rollPitchYawToSh2(0.2, 0.1, 2.9);
    controller.update(q);
    const before = controller.viewerThreeQuaternion().clone();

    controller.update({ w: -q.w, x: -q.x, y: -q.y, z: -q.z });
    const after = controller.viewerThreeQuaternion().clone();

    // Same rotation, and also the same numeric hemisphere.
    expect(Math.abs(before.dot(after))).toBeCloseTo(1, 12);
    expect(before.dot(after)).toBeGreaterThan(0);
  });
});

describe('recenter (viewer-only reference)', () => {
  it('renders exactly identity at the instant of recentering', () => {
    const raw = rollPitchYawToSh2(0.21, -0.34, 2.1);
    const recentred = normalizeSh2(applyReference(raw, raw));
    expect(recentred.w).toBeCloseTo(1, 12);
    expect(recentred.x).toBeCloseTo(0, 12);
    expect(recentred.y).toBeCloseTo(0, 12);
    expect(recentred.z).toBeCloseTo(0, 12);
  });

  it('shows a body rotation about the robot axis as the same viewer rotation', () => {
    // Robot parked level but yawed 90 degrees, then pitched nose-down 20 deg
    // about its OWN +Y axis. After recentering the viewer must show a pure
    // +pitch, not a roll.
    const reference = rollPitchYawToSh2(0, 0, 90 * DEG_TO_RAD);
    const bodyPitch = rollPitchYawToSh2(0, 20 * DEG_TO_RAD, 0);
    const raw = multiply(reference, bodyPitch);

    const viewer = normalizeSh2(applyReference(raw, reference));
    const rpy = sh2ToRollPitchYaw(viewer);

    expect(rpy.roll / DEG_TO_RAD).toBeCloseTo(0, 9);
    expect(rpy.pitch / DEG_TO_RAD).toBeCloseTo(20, 9);
    expect(rpy.yaw / DEG_TO_RAD).toBeCloseTo(0, 9);
    expect(rotate(viewer, NOSE).z).toBeLessThan(0); // nose still goes down
  });

  it('is exactly conjugate(reference) * raw and uses no Euler subtraction', () => {
    const raw = rollPitchYawToSh2(0.4, 0.2, -1.1);
    const reference = rollPitchYawToSh2(-0.15, 0.9, 0.6);
    expect(applyReference(raw, reference)).toEqual(multiply(conjugate(reference), raw));
    expect(applyReference(raw, null)).toEqual(raw);
  });

  it('through the controller: recenter zeroes, clearReference restores raw', () => {
    const controller = new OrientationController();
    const raw = rollPitchYawToSh2(0.1, 0.25, -2.4);
    controller.update(raw);

    controller.recenter();
    const zeroed = controller.viewerThreeQuaternion().clone();
    expect(Math.abs(zeroed.w)).toBeCloseTo(1, 12);
    expect(controller.snapshot().isRecentered).toBe(true);
    // The raw quaternion is preserved untouched next to the viewer one.
    expect(controller.snapshot().raw).toEqual(normalizeSh2(raw));

    controller.clearReference();
    expect(controller.snapshot().isRecentered).toBe(false);
    expect(controller.viewerThreeQuaternion().dot(sh2ToThree(normalizeSh2(raw)))).toBeCloseTo(1, 12);
  });

  it('keeps applying the reference to subsequent samples', () => {
    const controller = new OrientationController();
    const reference = rollPitchYawToSh2(0, 0, 120 * DEG_TO_RAD);
    controller.update(reference);
    controller.recenter();

    controller.update(multiply(reference, rollPitchYawToSh2(30 * DEG_TO_RAD, 0, 0)));
    const rpy = sh2ToRollPitchYaw(controller.snapshot().viewer);
    expect(rpy.roll / DEG_TO_RAD).toBeCloseTo(30, 9);
    expect(rpy.pitch / DEG_TO_RAD).toBeCloseTo(0, 9);
    expect(rpy.yaw / DEG_TO_RAD).toBeCloseTo(0, 9);
  });
});

describe('OrientationController robustness', () => {
  it('rejects non-finite and degenerate samples without moving the robot', () => {
    const controller = new OrientationController();
    const good = rollPitchYawToSh2(0.1, 0.2, 0.3);
    expect(controller.update(good)).toBe(true);
    const reference = controller.viewerThreeQuaternion().clone();

    for (const bad of [
      { w: Number.NaN, x: 0, y: 0, z: 0 },
      { w: Number.POSITIVE_INFINITY, x: 0, y: 0, z: 0 },
      { w: 0, x: 0, y: 0, z: 0 },
    ]) {
      expect(controller.update(bad)).toBe(false);
    }

    expect(controller.viewerThreeQuaternion().equals(reference)).toBe(true);
    expect(controller.snapshot().sampleCount).toBe(1);
  });
});

describe('synthetic orientation source', () => {
  it('emits RV lines the production parser turns back into the same quaternion', () => {
    for (const angles of [
      { rollDeg: 25, pitchDeg: 0, yawDeg: 0 },
      { rollDeg: 0, pitchDeg: 25, yawDeg: 0 },
      { rollDeg: 0, pitchDeg: 0, yawDeg: 30 },
      { rollDeg: -12, pitchDeg: 7, yawDeg: -140 },
    ]) {
      const expected = anglesToSh2(angles);
      const record = parseSensorLine(anglesToRvLine(angles, 7)) as RvRecord;
      expect(record.kind).toBe('rv');
      expect(record.count).toBe(7);

      const parsed = new Quaternion(record.x, record.y, record.z, record.w);
      expect(Math.abs(parsed.dot(sh2ToThree(expected)))).toBeCloseTo(1, 6);
    }
  });
});
