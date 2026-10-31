import { JOINT_NAMES } from './telemetry';
import { rollPitchYawToSh2 } from '../frame/orientation';

/**
 * ILLUSTRATIVE display fixture only. Never hardware, trajectory or STAND
 * validation, and never shown as a measured posture: it publishes TARGETS and
 * leaves the actual joint channel null (the twin draws it UNMEASURED).
 *
 * Endpoints are copied from the firmware references, in canonical URDF
 * convention (all four legs share axis +Y; LOWER positive = foot down):
 *   PRONE  = PairedProneData.h first knot   (front U/L 1.46153/-1.60570, rear 1.57080/-1.60570, height 0.0114 m)
 *   STAND  = StandReferenceData.h kStand     (front U/L 0.40987/0.34958,  rear 0.89833/0.00078,  height 0.150 m)
 * The 14 Oct demo used upper 0.5 / lower -0.8 for every leg: the LOWER sign
 * was inverted, which put the feet forward and up (anatomically wrong).
 * Between the endpoints this fixture interpolates linearly in joint space for
 * display; that is NOT the certified paired route (rear deploy, front deploy,
 * load transfer), whose geometry lives in the firmware and its certificates.
 */
export const PRONE_REFERENCE = { front: { upper: 1.4615259421016524, lower: -1.6057029118349999 },
  rear: { upper: 1.5707963267948966, lower: -1.6057029118349999 }, heightM: 0.011389821228567475 } as const;
export const STAND_REFERENCE = { front: { upper: 0.40987242488000036, lower: 0.3495791223438952 },
  rear: { upper: 0.8983341577111159, lower: 0.0007813874861022763 }, heightM: 0.15 } as const;

export function referencePose(alpha: number): number[] {
  const a = Math.min(1, Math.max(0, alpha));
  return JOINT_NAMES.map((name) => {
    if (name.includes('_hip_')) return 0;
    const end = name.startsWith('lf') || name.startsWith('rf') ? 'front' : 'rear';
    const p = PRONE_REFERENCE[end], s = STAND_REFERENCE[end];
    return name.includes('_upper_') ? p.upper + (s.upper - p.upper) * a : p.lower + (s.lower - p.lower) * a;
  });
}

export function syntheticMotionStatus(elapsedMs: number): unknown {
  const now = Math.max(0, elapsedMs), t = (now / 1000) % 20;
  let state: string, alpha: number;
  if (t < 2) { state = t < 1 ? 'OBSERVE' : 'CLASSIFY'; alpha = 0; }
  else if (t < 8) { state = 'RISE_ILLUSTRATIVE'; alpha = (t - 2) / 6; }
  else if (t < 12) { state = 'STAND_REFERENCE'; alpha = 1; }
  else if (t < 18) { state = 'LIE_DOWN_ILLUSTRATIVE'; alpha = 1 - (t - 12) / 6; }
  else { state = 'PRONE_REFERENCE'; alpha = 0; }
  const smooth = alpha * alpha * (3 - 2 * alpha);
  const height = PRONE_REFERENCE.heightM + (STAND_REFERENCE.heightM - PRONE_REFERENCE.heightM) * smooth;
  const q = rollPitchYawToSh2(0, 0, .05 * Math.sin(t / 3));
  return { version: 1, now_ms: now, executor_state: state, stand_state: state,
    owner: 'SYNTHETIC_DISPLAY_FIXTURE', fault: 'NONE', stale: false,
    reason: 'ILLUSTRATIVE feed from firmware reference endpoints; no hardware, no trajectory qualification, actual joints UNMEASURED',
    telemetry: { sampled_at_ms: now, joint_names: JOINT_NAMES, quaternion: [q.w, q.x, q.y, q.z], source: 'OFFLINE_SYNTHETIC',
      actual_rad: null, target_rad: referencePose(smooth), base_height_m: height,
      feet: ['LF', 'RF', 'RH', 'LH'].map((leg) => ({ leg, contact: 'unknown' })) } };
}
