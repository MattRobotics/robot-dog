/**
 * Builds the full articulated robot from the CANONICAL URDF and the real
 * canonical collision STLs, then checks the scene-graph guarantees the IMU
 * phase and the future ST3215 phase both depend on.
 */

import { Quaternion, Vector3 } from 'three';
import { beforeAll, describe, expect, it } from 'vitest';

import { buildRobot, resolveMeshUrl, rpyToQuaternion, type MatdogRobot } from '../src/urdf/buildRobot';
import { collisionLinks, parseUrdf, type UrdfModel } from '../src/urdf/parseUrdf';
import { rollPitchYawToSh2, sh2ToThree, DEG_TO_RAD } from '../src/frame/orientation';
import { styleForLink } from '../src/viewer/linkStyle';
import {
  createFileStlLoader,
  nodeXmlParser,
  readCanonicalUrdfText,
  TEST_URDF_URL,
  TEST_URDF_URL_PREFIX,
} from './support/nodeEnvironment';

const EXPECTED_COLLISION_MESH_COUNT = 17;

let model: UrdfModel;
let robot: MatdogRobot;
let loadedUrls: string[];

beforeAll(async () => {
  model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
  loadedUrls = [];
  const fileLoader = createFileStlLoader(TEST_URDF_URL_PREFIX);
  robot = await buildRobot({
    model,
    urdfUrl: TEST_URDF_URL,
    loadMeshGeometry: async (url) => {
      loadedUrls.push(url);
      return fileLoader(url);
    },
    styleForLink,
  });
}, 60_000);

describe('collision geometry coverage', () => {
  it('loads exactly the collision meshes the canonical URDF declares', () => {
    expect(robot.collisionMeshCount).toBe(EXPECTED_COLLISION_MESH_COUNT);
    expect(robot.collisionMeshCount).toBe(collisionLinks(model).length);
    expect(loadedUrls).toHaveLength(EXPECTED_COLLISION_MESH_COUNT);
    expect([...robot.collisionLinkNames].sort()).toEqual(
      collisionLinks(model)
        .map((link) => link.name)
        .sort(),
    );
  });

  it('uses the collision mesh set, never the heavy visual meshes', () => {
    for (const url of loadedUrls) {
      expect(url).toContain('/meshes/collision/');
    }
  });

  it('fails loudly instead of silently skipping a mesh', async () => {
    await expect(
      buildRobot({
        model,
        urdfUrl: TEST_URDF_URL,
        loadMeshGeometry: async (url) => {
          if (url.includes('rh_lower_leg_link')) throw new Error('simulated fetch failure');
          return createFileStlLoader(TEST_URDF_URL_PREFIX)(url);
        },
      }),
    ).rejects.toThrow(/rh_lower_leg_link.*simulated fetch failure/s);
  }, 60_000);

  it('resolves mesh URLs relative to the URDF', () => {
    expect(resolveMeshUrl('/canonical/m/robot.urdf', 'meshes/collision/base_link.stl')).toBe(
      '/canonical/m/meshes/collision/base_link.stl',
    );
    expect(resolveMeshUrl('/canonical/m/robot.urdf', 'package://matdog/meshes/a.stl')).toBe(
      '/canonical/m/meshes/a.stl',
    );
  });
});

describe('articulated scene graph', () => {
  it('keeps every link a separate object under a single root', () => {
    expect(robot.linkNames).toHaveLength(model.links.length);
    expect(robot.jointNames).toHaveLength(model.joints.length);
    for (const link of model.links) {
      expect(robot.linkObject(link.name), `${link.name} object missing`).toBeDefined();
    }
    expect(robot.linkObject('base_link')!.parent).toBe(robot.rootObject);
  });

  it('reproduces the URDF hierarchy: base -> hip -> upper -> lower -> foot', () => {
    const ancestry = (linkName: string): string[] => {
      const names: string[] = [];
      let current = robot.linkObject(linkName)!.parent;
      while (current) {
        if (typeof current.userData.urdfLink === 'string') names.push(current.userData.urdfLink);
        current = current.parent;
      }
      return names;
    };
    expect(ancestry('lf_foot_link')).toEqual([
      'lf_lower_leg_link',
      'lf_upper_leg_link',
      'lf_hip_link',
      'base_link',
    ]);
  });

  it('places the links where the canonical URDF says, in metres', () => {
    robot.resetJoints();
    robot.rootObject.updateMatrixWorld(true);

    const worldPosition = (linkName: string): Vector3 =>
      robot.linkObject(linkName)!.getWorldPosition(new Vector3());

    // lf_hip_joint origin 0.1125 0.0475 0.0465 from base_link.
    expect(worldPosition('lf_hip_link').toArray()).toEqual([0.1125, 0.0475, 0.0465]);
    // + lf_upper_leg_joint 0 0.048 0, then lf_lower_leg_joint 0 0 -0.09.
    expect(worldPosition('lf_lower_leg_link').x).toBeCloseTo(0.1125, 9);
    expect(worldPosition('lf_lower_leg_link').y).toBeCloseTo(0.0955, 9);
    expect(worldPosition('lf_lower_leg_link').z).toBeCloseTo(-0.0435, 9);

    // Left legs at +Y, right legs at -Y; front legs at +X, hind at -X.
    expect(worldPosition('rf_hip_link').y).toBeLessThan(0);
    expect(worldPosition('lh_hip_link').x).toBeLessThan(0);
  });

  it('scales the millimetre STLs into metres', () => {
    const mesh = robot
      .linkObject('base_link')!
      .getObjectByName('base_link__collision') as unknown as { scale: Vector3 };
    expect(mesh.scale.toArray()).toEqual([0.001, 0.001, 0.001]);
  });

  it('has a body-sized bounding box, not a millimetre-sized one', async () => {
    const { Box3 } = await import('three');
    robot.resetJoints();
    robot.rootObject.quaternion.identity();
    robot.rootObject.updateMatrixWorld(true);
    const box = new Box3().setFromObject(robot.rootObject);
    const size = box.getSize(new Vector3());
    // MATDOG REV00 is roughly 0.35 m long, 0.2 m wide, 0.2 m tall.
    expect(size.x).toBeGreaterThan(0.2);
    expect(size.x).toBeLessThan(0.7);
    expect(size.y).toBeGreaterThan(0.1);
    expect(size.y).toBeLessThan(0.5);
    expect(size.z).toBeGreaterThan(0.05);
    expect(size.z).toBeLessThan(0.5);
  });
});

describe('joint API (future ST3215 integration point)', () => {
  it('exposes all twelve revolute joints by URDF name', () => {
    const movable = model.joints.filter((joint) => joint.type !== 'fixed').map((j) => j.name);
    expect(movable).toHaveLength(12);
    for (const name of movable) expect(robot.jointNames).toContain(name);
  });

  it('moves the child chain and leaves other legs alone', () => {
    robot.resetJoints();
    robot.rootObject.updateMatrixWorld(true);
    const before = robot.linkObject('lf_foot_link')!.getWorldPosition(new Vector3());
    const otherBefore = robot.linkObject('rh_foot_link')!.getWorldPosition(new Vector3());

    robot.setJointAngle('lf_upper_leg_joint', 0.5);
    robot.rootObject.updateMatrixWorld(true);

    const after = robot.linkObject('lf_foot_link')!.getWorldPosition(new Vector3());
    const otherAfter = robot.linkObject('rh_foot_link')!.getWorldPosition(new Vector3());

    expect(after.distanceTo(before)).toBeGreaterThan(0.02);
    expect(otherAfter.distanceTo(otherBefore)).toBeLessThan(1e-12);
    expect(robot.jointValue('lf_upper_leg_joint')).toBeCloseTo(0.5, 12);

    robot.resetJoints();
    robot.rootObject.updateMatrixWorld(true);
    expect(robot.linkObject('lf_foot_link')!.getWorldPosition(new Vector3()).distanceTo(before)).toBeLessThan(1e-12);
  });

  it('clamps to the URDF limits and refuses fixed joints and unknown names', () => {
    expect(robot.setJointAngle('lf_hip_joint', 10)).toBeCloseTo(0.785398163397, 9);
    expect(robot.setJointAngle('lf_hip_joint', -10)).toBeCloseTo(-0.785398163397, 9);
    expect(() => robot.setJointAngle('lf_foot_joint', 0.1)).toThrow(/fixed/);
    expect(() => robot.setJointAngle('no_such_joint', 0.1)).toThrow(/unknown joint/);
    expect(() => robot.setJointAngle('lf_hip_joint', Number.NaN)).toThrow(/non-finite/);
    robot.resetJoints();
  });

  it('is idempotent: setting the same angle repeatedly does not drift', () => {
    robot.resetJoints();
    for (let index = 0; index < 50; index += 1) robot.setJointAngle('rf_lower_leg_joint', -0.4);
    robot.rootObject.updateMatrixWorld(true);
    const repeated = robot.linkObject('rf_foot_link')!.getWorldPosition(new Vector3());

    robot.resetJoints();
    robot.setJointAngle('rf_lower_leg_joint', -0.4);
    robot.rootObject.updateMatrixWorld(true);
    expect(robot.linkObject('rf_foot_link')!.getWorldPosition(new Vector3()).distanceTo(repeated)).toBeLessThan(1e-12);
    robot.resetJoints();
  });
});

describe('root orientation applies to the whole body only', () => {
  it('rotates every link rigidly without mutating any joint value', () => {
    robot.resetJoints();
    robot.setJointAngle('lh_upper_leg_joint', 0.35);
    robot.setJointAngle('rf_lower_leg_joint', -0.5);
    robot.rootObject.updateMatrixWorld(true);

    const snapshotBefore = robot.jointSnapshot();
    const localBefore = new Map(
      robot.linkNames.map((name) => [name, robot.linkObject(name)!.position.clone()]),
    );
    const worldBefore = new Map(
      robot.linkNames.map((name) => [name, robot.linkObject(name)!.getWorldPosition(new Vector3())]),
    );

    const q = sh2ToThree(rollPitchYawToSh2(20 * DEG_TO_RAD, -15 * DEG_TO_RAD, 40 * DEG_TO_RAD));
    robot.setRootOrientation(q);
    robot.rootObject.updateMatrixWorld(true);

    expect(robot.jointSnapshot()).toEqual(snapshotBefore);

    for (const name of robot.linkNames) {
      // Local transforms are untouched: the rotation lives on the root only.
      expect(robot.linkObject(name)!.position.equals(localBefore.get(name)!)).toBe(true);
      // World positions are exactly the rigid rotation of the previous ones.
      const expected = worldBefore.get(name)!.clone().applyQuaternion(q);
      expect(robot.linkObject(name)!.getWorldPosition(new Vector3()).distanceTo(expected)).toBeLessThan(1e-9);
    }

    robot.setRootOrientation(new Quaternion());
    robot.resetJoints();
  });

  it('preserves inter-link distances under any root orientation', () => {
    robot.resetJoints();
    robot.setRootOrientation(new Quaternion());
    robot.rootObject.updateMatrixWorld(true);
    const reference = robot
      .linkObject('lf_foot_link')!
      .getWorldPosition(new Vector3())
      .distanceTo(robot.linkObject('rh_foot_link')!.getWorldPosition(new Vector3()));

    for (const [r, p, y] of [
      [30, 0, 0],
      [0, 30, 0],
      [0, 0, 150],
      [-40, 25, -95],
    ]) {
      robot.setRootOrientation(
        sh2ToThree(rollPitchYawToSh2(r! * DEG_TO_RAD, p! * DEG_TO_RAD, y! * DEG_TO_RAD)),
      );
      robot.rootObject.updateMatrixWorld(true);
      const distance = robot
        .linkObject('lf_foot_link')!
        .getWorldPosition(new Vector3())
        .distanceTo(robot.linkObject('rh_foot_link')!.getWorldPosition(new Vector3()));
      expect(distance).toBeCloseTo(reference, 9);
    }
    robot.setRootOrientation(new Quaternion());
  });

  it('renders the frozen conventions on the real geometry', () => {
    robot.resetJoints();

    // +ROLL: the left-front foot must rise relative to the right-front foot.
    robot.setRootOrientation(sh2ToThree(rollPitchYawToSh2(25 * DEG_TO_RAD, 0, 0)));
    robot.rootObject.updateMatrixWorld(true);
    const lfZ = robot.linkObject('lf_foot_link')!.getWorldPosition(new Vector3()).z;
    const rfZ = robot.linkObject('rf_foot_link')!.getWorldPosition(new Vector3()).z;
    expect(lfZ).toBeGreaterThan(rfZ);

    // +PITCH: the front of the robot must drop below the hind.
    robot.setRootOrientation(sh2ToThree(rollPitchYawToSh2(0, 25 * DEG_TO_RAD, 0)));
    robot.rootObject.updateMatrixWorld(true);
    const frontZ = robot.linkObject('lf_hip_link')!.getWorldPosition(new Vector3()).z;
    const hindZ = robot.linkObject('lh_hip_link')!.getWorldPosition(new Vector3()).z;
    expect(frontZ).toBeLessThan(hindZ);

    // +YAW: the front of the robot must swing towards +Y (left).
    robot.setRootOrientation(sh2ToThree(rollPitchYawToSh2(0, 0, 30 * DEG_TO_RAD)));
    robot.rootObject.updateMatrixWorld(true);
    expect(robot.linkObject('lf_hip_link')!.getWorldPosition(new Vector3()).y).toBeGreaterThan(
      robot.linkObject('lh_hip_link')!.getWorldPosition(new Vector3()).y,
    );

    robot.setRootOrientation(new Quaternion());
  });
});

describe('URDF rpy convention', () => {
  it('matches the URDF fixed-axis order Rz(yaw) * Ry(pitch) * Rx(roll)', () => {
    const expected = new Quaternion()
      .setFromAxisAngle(new Vector3(0, 0, 1), 1.1)
      .multiply(new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), -0.7))
      .multiply(new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), 0.3));
    // Compared through the dot product: Quaternion.angleTo() goes through
    // acos(), which loses precision for near-identical rotations.
    expect(Math.abs(rpyToQuaternion([0.3, -0.7, 1.1]).dot(expected))).toBeCloseTo(1, 14);
  });
});
