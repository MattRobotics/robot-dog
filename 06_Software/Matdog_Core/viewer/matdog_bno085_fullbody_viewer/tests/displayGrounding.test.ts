import { beforeAll, describe, expect, it } from 'vitest';
import { Vector3 } from 'three';
import { rollPitchYawToSh2, sh2ToThree } from '../src/frame/orientation';
import { JOINT_NAMES, parseMotionFrame } from '../src/motion/telemetry';
import { MotionTwinController } from '../src/motion/twinController';
import { referencePose } from '../src/motion/syntheticMotion';
import { describeTwin } from '../src/mobile/mobileModel';
import { buildRobot } from '../src/urdf/buildRobot';
import { parseUrdf } from '../src/urdf/parseUrdf';
import { createFileStlLoader, nodeXmlParser, readCanonicalUrdfText, TEST_URDF_URL, TEST_URDF_URL_PREFIX } from './support/nodeEnvironment';

// Geometry/display fixtures only: interpolating these endpoints does not
// qualify a Motion trajectory. The real REV01-FIX graph and foot STLs are used.
function status(pose = referencePose(1), validHeight = false): Record<string, any> {
  const q = rollPitchYawToSh2(.012, -.009, -.8);
  return { version: 1, now_ms: 1000, stale: false, state: 'STANDING', phase: 'HOLD',
    height_valid: validHeight, height_source: 'ENCODER_IMU_FLAT_GROUND_ESTIMATE',
    base_link_height_mm: 147.8, requested_height_mm: 150, imu_valid: true, imu_age_ms: 10,
    telemetry: { sampled_at_ms: 990, actual_rad: pose, quaternion: [q.w, q.x, q.y, q.z] } };
}

describe('SIMPLE height estimate admission', () => {
  it('uses the valid base_link estimate, not the command/nominal height, and preserves raw input', () => {
    const s = status(undefined, true), raw = structuredClone(s);
    expect(parseMotionFrame(s)?.baseHeightM).toBeCloseTo(.1478, 12);
    expect(parseMotionFrame(s)?.baseHeightSource).toBe('controller-estimate');
    expect(s).toEqual(raw);
    s.height_valid = false;
    expect(parseMotionFrame(s)?.baseHeightM).toBeNull(); // PRONE/WALK publish an invalid number too
    delete s.base_link_height_mm;
    expect(parseMotionFrame(s)?.baseHeightM).toBeNull();
  });
  it('rejects invalid, stale, partial or old-IMU estimates without changing the joint channel', () => {
    for (const change of [{ stale: true }, { imu_valid: false }, { imu_age_ms: 201 },
      { base_link_height_mm: NaN }, { height_source: 'UNKNOWN' }, { snapshot_age_ms: 300 }]) {
      const s = Object.assign(status(undefined, true), change);
      expect(parseMotionFrame(s)?.baseHeightM).toBeNull();
      expect(parseMotionFrame(s)?.actual).not.toBeNull();
    }
    const s = status(undefined, true); s.telemetry.actual_rad[3] = null;
    expect(parseMotionFrame(s)?.baseHeightM).toBeNull();
  });
});

describe('mobile world/base/floor display transform', () => {
  let twin: MotionTwinController;
  beforeAll(async () => {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
    const options = { model, urdfUrl: TEST_URDF_URL, loadMeshGeometry: createFileStlLoader(TEST_URDF_URL_PREFIX) };
    twin = new MotionTwinController(await buildRobot(options), await buildRobot(options));
    twin.groundFromFeet = true;
  });
  function feetZ(): number[] {
    twin.actual.rootObject.updateMatrixWorld(true);
    return ['lf', 'rf', 'rh', 'lh'].map((leg) => {
      const mesh = twin.actual.collisionMesh(`${leg}_foot_link`)!;
      const p = mesh.geometry.getAttribute('position'), v = new Vector3();
      let z = Infinity;
      for (let i = 0; i < p.count; i++) z = Math.min(z, v.fromBufferAttribute(p, i).applyMatrix4(mesh.matrixWorld).z);
      return z;
    });
  }
  it('raises the whole body PRONE→STAND→PRONE against static Z=0 while preserving articulation and IMU', () => {
    const heights: number[] = [];
    for (const progress of [0, .25, .5, .75, 1, .5, 0]) {
      const s = status(referencePose(progress));
      const frame = parseMotionFrame(s)!; twin.apply(frame);
      heights.push(twin.actual.rootObject.position.z);
      expect(Math.min(...feetZ())).toBeCloseTo(0, 8);
      expect(twin.actual.jointSnapshot()).toEqual(Object.fromEntries(JOINT_NAMES.map((name, i) => [name, s.telemetry.actual_rad[i]])));
      expect(twin.actual.rootObject.quaternion.angleTo(sh2ToThree(frame.quaternion!))).toBeCloseTo(0);
      expect(twin.grounding.source).toBe('visual-envelope');
    }
    expect(heights[0]).toBeLessThan(.04); expect(heights[4]).toBeGreaterThan(.12);
    expect(heights[6]).toBeCloseTo(heights[0]!, 10);
  });
  it('keeps a lifted leg above the floor and applies only one global correction to the controller estimate', () => {
    const pose = referencePose(1); pose[1]! += .4;
    const s = status(pose, true); s.telemetry.quaternion = [1, 0, 0, 0]; twin.apply(parseMotionFrame(s)!);
    const z = feetZ();
    expect(Math.min(...z)).toBeGreaterThanOrEqual(-1e-9);
    expect(z[0]! - Math.max(...z.slice(1))).toBeGreaterThan(.005);
    expect(twin.actual.rootObject.position.z).toBeCloseTo(.1478 + twin.grounding.adjustmentM);
    expect(twin.grounding.source).toBe('controller-estimate');
    expect(twin.grounding.baseHeightM).toBeCloseTo(.1478, 12);
    expect(describeTwin(parseMotionFrame(s), 10, false, 'MEASURED', '', 0, twin.grounding).rows.some(([, value]) => value.includes('non è una misura assoluta'))).toBe(true);
  });
  it('does not ground neutral/unmeasured geometry or claim stale placement as current', () => {
    const s = status(); s.telemetry.actual_rad = null;
    twin.apply(parseMotionFrame(s)!);
    expect(twin.grounding.source).toBe('unavailable');
    const text = describeTwin(parseMotionFrame(status()), 2000, true, 'MEASURED', '', 0, twin.grounding);
    expect(text.rows.find(([key]) => key === 'Riferimento pavimento')?.[1]).toContain('Non disponibile');
  });
});
