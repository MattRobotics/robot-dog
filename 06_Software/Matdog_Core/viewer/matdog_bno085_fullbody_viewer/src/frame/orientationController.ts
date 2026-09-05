/**
 * Holds the three clearly separated orientation concepts and applies the
 * result to the robot root.
 *
 *   rawQuaternion       : exactly what SH2_ROTATION_VECTOR reported
 *                         (hemisphere-aligned only; never modified otherwise)
 *   referenceQuaternion : the raw value captured by Recenter, or null
 *   viewerQuaternion    : conjugate(reference) * raw  -- what is rendered
 *
 * Recenter is host-side only. Nothing here writes to the sensor, the DCD or
 * any bus.
 */

import { Quaternion } from 'three';
import {
  alignSignWithPrevious,
  applyReference,
  isFiniteQuaternion,
  normalizeSh2,
  sh2ToThree,
  SH2_IDENTITY,
  type Sh2Quaternion,
} from './orientation';

export interface OrientationSnapshot {
  readonly raw: Sh2Quaternion;
  readonly viewer: Sh2Quaternion;
  readonly reference: Sh2Quaternion | null;
  readonly isRecentered: boolean;
  readonly sampleCount: number;
}

export class OrientationController {
  private raw: Sh2Quaternion = SH2_IDENTITY;
  private reference: Sh2Quaternion | null = null;
  private viewer: Sh2Quaternion = SH2_IDENTITY;
  private samples = 0;
  private readonly threeQuaternion = new Quaternion();

  /**
   * Accepts a raw SH2 quaternion. Returns false (and changes nothing) when
   * the sample is not usable, so a corrupt line can never move the robot.
   */
  update(candidate: Sh2Quaternion): boolean {
    if (!isFiniteQuaternion(candidate)) return false;
    const normalized = normalizeSh2(candidate);
    if (normalized === SH2_IDENTITY && candidate.w !== 1) return false;

    this.raw = alignSignWithPrevious(this.samples === 0 ? null : this.raw, normalized);
    this.viewer = normalizeSh2(applyReference(this.raw, this.reference));
    this.samples += 1;
    return true;
  }

  /** Defines the current raw orientation as the viewer reference. */
  recenter(): void {
    this.reference = this.raw;
    this.viewer = normalizeSh2(applyReference(this.raw, this.reference));
  }

  /** Drops the reference; the raw sensor orientation is shown again. */
  clearReference(): void {
    this.reference = null;
    this.viewer = this.raw;
  }

  /** Quaternion to hand to the robot root, in THREE (x, y, z, w) order. */
  viewerThreeQuaternion(): Quaternion {
    return sh2ToThree(this.viewer, this.threeQuaternion);
  }

  snapshot(): OrientationSnapshot {
    return {
      raw: this.raw,
      viewer: this.viewer,
      reference: this.reference,
      isRecentered: this.reference !== null,
      sampleCount: this.samples,
    };
  }
}
