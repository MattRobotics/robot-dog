import { describe, expect, it } from 'vitest';
import { BufferGeometry, Quaternion, Vector3 } from 'three';
import { JOINT_NAMES, MotionTelemetryStream, parseMotionFrame } from '../src/motion/telemetry';
import { MotionTwinController } from '../src/motion/twinController';
import { referencePose, syntheticMotionStatus } from '../src/motion/syntheticMotion';
import { buildRobot } from '../src/urdf/buildRobot';
import { parseUrdf } from '../src/urdf/parseUrdf';
import { nodeXmlParser, readCanonicalUrdfText, TEST_URDF_URL } from './support/nodeEnvironment';

function sample(now = 1000): Record<string, unknown> {
  return { version: 1, now_ms: now, executor_state: 'STAND', stand_state: 'STAND', owner: 'local', fault: 'NONE',
    telemetry: { sampled_at_ms: now - 10, joint_names: JOINT_NAMES, actual_rad: JOINT_NAMES.map(() => .1),
      target_rad: JOINT_NAMES.map(() => .2), quaternion: [Math.cos(.15), 0, 0, Math.sin(.15)], base_height_m: .15,
      feet: [{ leg: 'LF', position_m: [.1, .1, 0], contact: 'support', phase: .3,
        trajectory_m: [[.1, .1, 0], [.11, .1, .01]] }] } };
}

describe('Motion telemetry contract', () => {
  it('uses Controller order LF RF RH LH and accepts null unavailable channels without making up contact or height', () => {
    expect(JOINT_NAMES[6]).toBe('rh_hip_joint');
    const frame = parseMotionFrame({ version: 1, now_ms: 100, telemetry: { sampled_at_ms: 100, actual_rad: null, target_rad: null, quaternion: null } });
    expect(frame?.actual).toBeNull(); expect(frame?.baseHeightM).toBeNull();
    const fault = parseMotionFrame({ version: 1, now_ms: 100, fault: true, tracking_error_rad: .03, telemetry: { sampled_at_ms: 100 } });
    expect(fault?.fault).toBe('FAULT'); expect(fault?.trackingErrorRad).toBe(.03);
    expect(frame?.feet.every((f) => f.contact === 'unknown' && f.position === null)).toBe(true);
  });
  it('rejects protocol drift/future stamps and incomplete/non-finite joint channels', () => {
    expect(parseMotionFrame({ ...sample(), version: 2 })).toBeNull();
    const future = sample(); (future.telemetry as Record<string, unknown>).sampled_at_ms = 9000;
    expect(parseMotionFrame(future)).toBeNull();
    const partial = sample(); (partial.telemetry as Record<string, unknown>).actual_rad = [1, 2];
    expect(parseMotionFrame(partial)?.actual).toBeNull();
    const invalid = sample(); (invalid.telemetry as Record<string, unknown>).target_rad = JOINT_NAMES.map(() => Infinity);
    expect(parseMotionFrame(invalid)?.target).toBeNull();
  });
  it('preserves declared base XYZ and rejects conflicting height or malformed XYZ', () => {
    const input = sample(); const data = input.telemetry as Record<string, unknown>;
    data.base_position_m = [.03, -.02, .15];
    expect(parseMotionFrame(input)?.basePositionM).toEqual([.03, -.02, .15]);
    data.base_position_m = [.03, -.02, .14]; expect(parseMotionFrame(input)).toBeNull();
    data.base_position_m = [0, Infinity, .15]; expect(parseMotionFrame(input)).toBeNull();
    delete data.base_height_m; data.base_position_m = [0, 0, .15];
    expect(parseMotionFrame(input)?.baseHeightM).toBe(.15);
  });
  it('respects explicit joint names, disallows duplicate names and bounds trajectory memory', () => {
    const input = sample(); const data = input.telemetry as Record<string, unknown>;
    data.joint_names = [...JOINT_NAMES].reverse(); data.actual_rad = JOINT_NAMES.map((_, i) => i);
    expect(parseMotionFrame(input)?.actual?.[JOINT_NAMES[0]!]).toBe(11);
    data.joint_names = JOINT_NAMES.map(() => JOINT_NAMES[0]);
    expect(parseMotionFrame(input)?.actual).toBeNull();
    data.feet = [{ leg: 'LF', trajectory_m: Array.from({ length: 5000 }, () => [0, 0, 0]) }];
    expect(parseMotionFrame(input)?.feet[0]?.trajectory.length).toBe(64);
  });
  it('declares stale for local transport age, Controller sample age and explicit stale; rejects reordered frames', () => {
    const stream = new MotionTelemetryStream(); expect(stream.isStale(0)).toBe(true);
    expect(stream.accept(sample(), 500)).toBe(true); expect(stream.isStale(600)).toBe(false);
    expect(stream.ageMs(600)).toBe(110); expect(stream.isStale(2000)).toBe(true);
    expect(stream.accept(sample(), 2100)).toBe(true);
    expect(stream.isStale(2100)).toBe(true); // repeated cached status does not renew freshness
    expect(stream.accept(sample(900), 700)).toBe(false);
    const old = sample(5000); (old.telemetry as Record<string, unknown>).sampled_at_ms = 2000;
    expect(stream.accept(old, 800)).toBe(true); expect(stream.isStale(800)).toBe(true);
    stream.reset(); expect(stream.accept({ ...sample(), stale: true }, 900)).toBe(true);
    expect(stream.isStale(900)).toBe(true);
  });
});

describe('Motion twin composition', () => {
  it('composes quaternion, all twelve actual joints, distinct targets, support height and foot trails simultaneously', async () => {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
    const loadMeshGeometry = async (): Promise<BufferGeometry> => new BufferGeometry().setFromPoints([new Vector3(), new Vector3(.01, 0, 0), new Vector3(0, .01, 0)]);
    const options = { model, urdfUrl: TEST_URDF_URL, loadMeshGeometry };
    const actual = await buildRobot(options), target = await buildRobot(options), twin = new MotionTwinController(actual, target);
    twin.apply(parseMotionFrame(sample())!);
    expect(actual.jointSnapshot()).toEqual(Object.fromEntries(JOINT_NAMES.map((name) => [name, .1])));
    expect(target.jointSnapshot()).toEqual(Object.fromEntries(JOINT_NAMES.map((name) => [name, .2])));
    expect(actual.rootObject.quaternion.angleTo(new Quaternion(0, 0, Math.sin(.15), Math.cos(.15)))).toBeCloseTo(0);
    expect(target.rootObject.quaternion.angleTo(actual.rootObject.quaternion)).toBeCloseTo(0);
    expect(actual.rootObject.position.z).toBe(.15); expect(target.rootObject.position.z).toBe(.15);
    const translated = sample(1050); (translated.telemetry as Record<string, unknown>).base_position_m = [.03, -.02, .15];
    twin.apply(parseMotionFrame(translated)!);
    expect(actual.rootObject.position.toArray()).toEqual([.03, -.02, .15]);
    expect(target.rootObject.position.toArray()).toEqual([.03, -.02, .15]);
    twin.apply(parseMotionFrame(sample(1060))!);
    expect(actual.rootObject.position.toArray()).toEqual([0, 0, .15]);
    expect(twin.markers.get('LF')?.position.toArray()).toEqual([.1, .1, 0]);
    expect(twin.markers.get('RF')?.visible).toBe(false); expect(twin.trails.get('LF')?.geometry.getAttribute('position').count).toBe(2);
    twin.setStale(true); expect(target.rootObject.visible).toBe(false); expect(twin.overlay.visible).toBe(false);
    twin.apply(parseMotionFrame(sample(1100))!); twin.setStale(false); expect(target.rootObject.visible).toBe(true);
    const bad = sample(1200); (bad.telemetry as Record<string, unknown>).actual_rad = JOINT_NAMES.map(() => 99);
    twin.apply(parseMotionFrame(bad)!); expect(twin.diagnostic).toContain('implausible');
    // Never hidden; implausible channels are held at neutral and drawn red.
    expect(actual.rootObject.visible).toBe(true); expect(actual.jointValue(JOINT_NAMES[0]!)).toBe(0);
    expect(twin.linkStates.get('lf_lower_leg_link')).toBe('IMPLAUSIBLE');
  });
  it('synthetic fixture is ILLUSTRATIVE: firmware reference endpoints, targets only, actual UNMEASURED, no WALK', () => {
    const phases = [0, 1500, 5000, 10000, 15000, 19000].map((t) => parseMotionFrame(syntheticMotionStatus(t))!);
    expect(phases.map((p) => p.executorState)).toEqual(['OBSERVE', 'CLASSIFY', 'RISE_ILLUSTRATIVE', 'STAND_REFERENCE', 'LIE_DOWN_ILLUSTRATIVE', 'PRONE_REFERENCE']);
    expect(phases.every((p) => p.actual === null && p.actualByJoint === null && p.reason.includes('no hardware'))).toBe(true);
    expect(phases.some((p) => /WALK/.test(p.executorState))).toBe(false);
    expect(phases[3]?.target).toEqual(Object.fromEntries(JOINT_NAMES.map((n, i) => [n, referencePose(1)[i]])));
    expect(phases[3]?.baseHeightM).toBe(.15);
    expect(phases[0]?.feet.every((foot) => foot.contact === 'unknown')).toBe(true);
  });
});

describe('offline trace adapter', () => {
  it('replays actual Motion Core JSONL with source stamps and source-declared contacts preserved', async () => {
    const { readFile } = await import('node:fs/promises');
    const { resolve } = await import('node:path');
    const { parseOfflineTrace } = await import('../src/motion/offlineTrace');
    const source = resolve(process.cwd(), '../../../../09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_synthetic.jsonl');
    const text = await readFile(source, 'utf8');
    const original = text.trim().split('\n').map((line) => JSON.parse(line));
    const records = parseOfflineTrace(text);
    expect(records.length).toBe(original.length);
    records.forEach((record, index) => {
      const raw = original[index], parsed = parseMotionFrame(record)!;
      expect(parsed).not.toBeNull(); expect(parsed.sampledAtMs).toBe(raw.telemetry.stamp_ms);
      expect(record.telemetry).toMatchObject({ actual_rad: raw.telemetry.actual_rad, target_rad: raw.telemetry.target_rad, quaternion: raw.telemetry.quaternion });
      expect(parsed.reason).toContain('SYNTHETIC_NOT_HARDWARE_QUALIFIED');
      expect(parsed.feet[0]?.contact).toBe(raw.telemetry.feet.LF.state === 'STANCE' ? 'support' : 'swing');
    });
    expect(new Set(records.map((r) => r.stand_state))).toContain('WALK');
    expect(parseMotionFrame(original[0])).toBeNull(); // live contract has no offline alias coercion
  });
  it('accepts JSON containers and rejects malformed, reordered and non-finite joint traces', async () => {
    const { parseOfflineTrace } = await import('../src/motion/offlineTrace');
    expect(parseOfflineTrace(JSON.stringify({ frames: [sample()] })).length).toBe(1);
    expect(() => parseOfflineTrace('[]')).toThrow();
    expect(() => parseOfflineTrace(JSON.stringify([sample(2000), sample(1000)]))).toThrow('reordered');
    const invalid = sample(); (invalid.telemetry as Record<string, unknown>).actual_rad = [1, 2];
    expect(() => parseOfflineTrace(JSON.stringify([invalid]))).toThrow('joint channel');
    expect(() => parseOfflineTrace('{bad}')).toThrow();
  });
});

describe('14 Oct hardware counterexample and honest twin display', () => {
  // First real @TEL of 2026-10-14T17:14:55Z (sanitized subset, firmware 83c3410).
  const tel = { version: 1, now_ms: 892599, stale: true, reason: 'RESTORE_REQUIRED', executor_state: 'OFF', stand_state: 'IDLE', owner: 'none', fault: false,
    telemetry: { sampled_at_ms: 892247, actual_rad: null, raw_ticks: [2018, 3002, 3146, 1999, 1196, 944, 2063, 1126, 994, 2049, 3023, 3131], target_rad: null,
      quaternion: [0.12493896, 0.00524902, 0.00561523, -0.99212646], base_height_m: null, source: 'HARDWARE_SERVO_FEEDBACK', feet: null },
    imu: { valid: true, stamp_ms: 892591 } };
  async function rig() {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
    const loadMeshGeometry = async (): Promise<BufferGeometry> => new BufferGeometry().setFromPoints([new Vector3(), new Vector3(.01, 0, 0), new Vector3(0, .01, 0)]);
    const options = { model, urdfUrl: TEST_URDF_URL, loadMeshGeometry };
    const actual = await buildRobot(options), target = await buildRobot(options);
    return { actual, target, twin: new MotionTwinController(actual, target) };
  }
  it('legacy stale with a null joint channel does not stale the frame; IMU and raw stay independent', () => {
    const frame = parseMotionFrame(tel)!;
    expect(frame.stale).toBe(false); expect(frame.jointsStale).toBe(true);
    expect(frame.actual).toBeNull(); expect(frame.quaternion).not.toBeNull();
    expect(frame.rawTicks?.[0]).toBe(2018);
    expect(frame.channels).toEqual({ imuValid: true, imuAgeMs: 8, rawCount: 12, mappedCount: 0 });
    const stream = new MotionTelemetryStream(); expect(stream.accept(tel, 0)).toBe(true);
    expect(stream.isStale(100)).toBe(false);          // 352 ms sample + 100 ms transport < 1500 ms
    expect(stream.isStale(1200)).toBe(true);          // transport age still ages it
  });
  it('a measured-but-stale legacy frame stays stale; new firmware scopes stale to joints only', () => {
    const measured = structuredClone(tel) as Record<string, any>; measured.telemetry.actual_rad = JOINT_NAMES.map(() => .1);
    expect(parseMotionFrame(measured)!.stale).toBe(true);
    const scoped = structuredClone(tel) as Record<string, any>; scoped.stale_scope = 'MOTION_JOINT_FEEDBACK';
    scoped.telemetry.actual_rad_by_joint = JOINT_NAMES.map((_, i) => i === 4 ? null : .1);
    scoped.channels = { imu: { valid: true, age_ms: 8 }, raw: { count: 12, max_age_ms: 352 }, mapped: { count: 11, mask: 4079 } };
    const f = parseMotionFrame(scoped)!;
    expect(f.stale).toBe(false); expect(f.jointsStale).toBe(true); expect(f.actual).toBeNull();
    expect(f.actualByJoint?.[JOINT_NAMES[4]!]).toBeNull(); expect(f.actualByJoint?.[JOINT_NAMES[5]!]).toBe(.1);
    expect(f.channels.mappedCount).toBe(11);
    expect(parseMotionFrame({ ...scoped, transport_stale: true })!.stale).toBe(true);   // bridge link down
    expect(parseMotionFrame({ ...tel, transport_stale: false })!.stale).toBe(false);   // legacy joint stale only
  });
  it('renders the 17-link model UNMEASURED at URDF neutral with IMU orientation, never hidden or posed as measured', async () => {
    const { actual, twin } = await rig();
    twin.apply(parseMotionFrame(tel)!);
    expect(actual.rootObject.visible).toBe(true);
    expect(twin.actualState).toBe('UNMEASURED');
    expect(Object.values(actual.jointSnapshot()).every((v) => v === 0)).toBe(true);
    expect(twin.orientationMeasured).toBe(true);
    expect([...twin.linkStates.values()].filter((s) => s === 'UNMEASURED').length).toBe(16); // all but base_link
    expect(twin.linkStates.get('base_link')).toBe('MEASURED');
    expect(twin.linkStates.size).toBe(17);
    expect(twin.diagnostic).toContain('UNMEASURED');
    const noImu = structuredClone(tel) as Record<string, any>; noImu.telemetry.quaternion = null; noImu.now_ms += 10;
    twin.apply(parseMotionFrame(noImu)!);
    expect(twin.linkStates.get('base_link')).toBe('UNMEASURED');
  });
  it('partial mapping: measured legs drawn measured, the unmapped joint subtree grey; out-of-range drawn red', async () => {
    const { actual, twin } = await rig();
    const scoped = structuredClone(tel) as Record<string, any>; scoped.stale_scope = 'MOTION_JOINT_FEEDBACK';
    const values = referencePose(0); values[4] = null as unknown as number;          // RF upper unmapped
    values[2] = -1.6057029118349999 - .01;                                            // LF lower 10 mrad beyond the URDF limit
    scoped.telemetry.actual_rad_by_joint = values;
    twin.apply(parseMotionFrame(scoped)!);
    expect(twin.actualState).toBe('PARTIAL');
    expect(twin.linkStates.get('rf_upper_leg_link')).toBe('UNMEASURED');
    expect(twin.linkStates.get('rf_foot_link')).toBe('UNMEASURED');
    expect(twin.linkStates.get('rf_hip_link')).toBe('MEASURED');
    expect(twin.linkStates.get('lf_lower_leg_link')).toBe('OUT_OF_RANGE');
    expect(twin.linkStates.get('lh_upper_leg_link')).toBe('MEASURED');
    expect(twin.diagnostic).toContain('lf_lower_leg_joint=-1.6157');
    expect(actual.jointValue('rh_upper_leg_joint')).toBeCloseTo(1.5707963267948966);
  });
  it('URDF kinematics: the STAND reference stands and the old demo pose did not', async () => {
    const { actual } = await rig();
    const feetZ = (pose: number[]): number[] => {
      actual.setRootOrientation(new Quaternion()); actual.rootObject.position.set(0, 0, 0);
      actual.setJointAngles(Object.fromEntries(JOINT_NAMES.map((n, i) => [n, pose[i]!])));
      actual.rootObject.updateMatrixWorld(true);
      return ['lf', 'rf', 'rh', 'lh'].map((leg) => actual.linkObject(`${leg}_foot_link`)!.getWorldPosition(new Vector3()).z);
    };
    const stand = feetZ(referencePose(1));
    expect(stand.every((z) => z < -.13 && z > -.16)).toBe(true);                     // ~150 mm under base_link
    expect(Math.max(...stand) - Math.min(...stand)).toBeLessThan(.01);               // four-foot support plane
    const prone = feetZ(referencePose(0));
    expect(prone.every((z) => z > -.03)).toBe(true);                                 // folded at body level
    const oldDemo = feetZ(JOINT_NAMES.map((n) => n.includes('_hip_') ? 0 : n.includes('_upper_') ? .5 : -.8));
    expect(oldDemo.every((z) => z > -.08)).toBe(true);                               // 14 Oct demo: feet ~50 mm, not 150 mm
  });
});
