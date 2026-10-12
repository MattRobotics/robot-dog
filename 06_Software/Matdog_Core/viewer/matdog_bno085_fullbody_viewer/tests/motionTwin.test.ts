import { describe, expect, it } from 'vitest';
import { BufferGeometry, Quaternion, Vector3 } from 'three';
import { JOINT_NAMES, MotionTelemetryStream, parseMotionFrame } from '../src/motion/telemetry';
import { MotionTwinController } from '../src/motion/twinController';
import { syntheticMotionStatus } from '../src/motion/syntheticMotion';
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
    twin.apply(parseMotionFrame(bad)!); expect(twin.diagnostic).toContain('outside canonical');
    expect(actual.rootObject.visible).toBe(false); expect(actual.jointValue(JOINT_NAMES[0]!)).toBe(.1);
  });
  it('synthetic fixture exposes prone acquisition rise stand walk stop and an intermediate restart', () => {
    const phases = [0, 1500, 3000, 7000, 10500, 11500, 14000, 18000, 19500, 21000, 23000]
      .map((t) => parseMotionFrame(syntheticMotionStatus(t))!);
    expect(phases.map((p) => p.executorState)).toEqual(['OBSERVE', 'CLASSIFY', 'ACQUIRE_SUPPORT', 'RISE', 'VERIFY_STAND', 'STAND', 'WALK', 'STOP', 'OBSERVE_INTERMEDIATE', 'RISE', 'STAND']);
    expect(phases[4]?.baseHeightM).toBe(.15); expect(phases[8]?.baseHeightM).toBe(.105);
    expect(phases.every((p) => Object.keys(p.actual!).length === 12 && p.reason.includes('no hardware'))).toBe(true);
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
