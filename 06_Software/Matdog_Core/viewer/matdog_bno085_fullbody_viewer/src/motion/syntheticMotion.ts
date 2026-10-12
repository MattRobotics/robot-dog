import { JOINT_NAMES, LEGS } from './telemetry';
import { rollPitchYawToSh2 } from '../frame/orientation';

/** Display fixture only. Physical/path qualification is provided by Motion Core offline tests. */
export function syntheticMotionStatus(elapsedMs: number): unknown {
  const now = Math.max(0, elapsedMs), t = (now / 1000) % 24;
  let state: string, height: number, flex: number;
  if (t < 2) { state = t < 1 ? 'OBSERVE' : 'CLASSIFY'; height = .04; flex = 1; }
  else if (t < 5) { state = 'ACQUIRE_SUPPORT'; height = .04; flex = 1 - (t - 2) / 3 * .25; }
  else if (t < 10) { state = 'RISE'; const a = (t - 5) / 5; height = .04 + a * .11; flex = .75 * (1 - a); }
  else if (t < 11) { state = 'VERIFY_STAND'; height = .15; flex = 0; }
  else if (t < 12) { state = 'STAND'; height = .15; flex = 0; }
  else if (t < 17) { state = 'WALK'; height = .15; flex = 0; }
  else if (t < 19) { state = 'STOP'; height = .15; flex = 0; }
  else if (t < 20) { state = 'OBSERVE_INTERMEDIATE'; height = .105; flex = .35; }
  else if (t < 22) { state = 'RISE'; const a = (t - 20) / 2; height = .105 + a * .045; flex = .35 * (1 - a); }
  else { state = 'STAND'; height = .15; flex = 0; }
  const phase = (t % 1), walk = state === 'WALK';
  const target = JOINT_NAMES.map((name, index) => {
    const cycle = walk ? Math.sin(phase * Math.PI * 2 + Math.floor(index / 3) * Math.PI / 2) * .05 : 0;
    return name.includes('_hip_') ? .02 * Math.sin(t) : name.includes('_upper_') ? .5 + flex + cycle : -.8 - flex * .5 - cycle;
  });
  const actual = target.map((q, i) => q + Math.sin(t * 3 + i) * .004);
  const q = rollPitchYawToSh2(.015 * Math.sin(t), .012 * Math.cos(t), .08 * Math.sin(t / 4));
  return { version: 1, now_ms: now, executor_state: state, stand_state: state,
    owner: 'SYNTHETIC_DISPLAY_FIXTURE', fault: 'NONE', reason: 'Illustrative feed; no hardware or trajectory qualification', stale: false,
    telemetry: { sampled_at_ms: now, joint_names: JOINT_NAMES, quaternion: [q.w, q.x, q.y, q.z],
      actual_rad: actual, target_rad: target, base_height_m: height, tracking_error_rad: .004,
      feet: LEGS.map((leg, i) => { const p = (phase + i * .25) % 1, swing = walk && p > .75;
        const x = i < 2 ? .12 : -.12, y = leg.startsWith('L') ? .1 : -.1;
        return { leg, contact: t < 5 ? 'unknown' : swing ? 'swing' : 'support', phase: walk ? p : null,
          position_m: [x + (walk ? .015 * Math.sin(p * 2 * Math.PI) : 0), y, swing ? .015 * Math.sin((p - .75) * 4 * Math.PI) : 0],
          trajectory_m: walk ? Array.from({ length: 24 }, (_, n) => [x + .015 * Math.sin(n / 23 * 2 * Math.PI), y, n / 23 > .75 ? .015 * Math.sin((n / 23 - .75) * 4 * Math.PI) : 0]) : [] }; }) } };
}
