/**
 * Builds an articulated Three.js scene graph from a parsed URDF model using
 * the <collision> meshes only.
 *
 * Structure (one Object3D per URDF frame, nothing is flattened):
 *
 *   rootObject                 <- the whole robot; base_link orientation from the IMU
 *     linkObject(base_link)
 *       collisionObject        <- URDF <collision><origin>, mesh scale, STL mesh
 *       jointObject(lf_hip_joint)      <- URDF <joint><origin>, static
 *         articulationObject           <- ONLY this rotates with the joint value
 *           linkObject(lf_hip_link)
 *             ...
 *
 * Keeping the joint origin and the articulation in separate objects means a
 * joint value can be written repeatedly without accumulating error, and the
 * IMU orientation written to `rootObject` can never disturb the joint pose.
 */

import { BufferGeometry, Euler, Mesh, MeshStandardMaterial, Object3D, Quaternion, Vector3 } from 'three';
import type { UrdfJoint, UrdfModel, Vec3 } from './parseUrdf';

/** Loads one collision STL and returns its geometry, in millimetres. */
export type MeshGeometryLoader = (url: string) => Promise<BufferGeometry>;

export interface LinkStyle {
  readonly color: number;
  readonly metalness: number;
  readonly roughness: number;
}

export interface BuildRobotOptions {
  readonly model: UrdfModel;
  /** Absolute or app-relative URL of the URDF; mesh filenames resolve against it. */
  readonly urdfUrl: string;
  readonly loadMeshGeometry: MeshGeometryLoader;
  readonly styleForLink?: (linkName: string) => LinkStyle;
}

export interface JointHandle {
  readonly joint: UrdfJoint;
  /** Object carrying the static URDF joint origin. */
  readonly originObject: Object3D;
  /** Object carrying the actuated rotation/translation. */
  readonly articulationObject: Object3D;
  value: number;
}

export class RobotBuildError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'RobotBuildError';
  }
}

const DEFAULT_STYLE: LinkStyle = { color: 0x9aa4ad, metalness: 0.15, roughness: 0.65 };

/** URDF fixed-axis rpy -> quaternion. Verified equal to Rz(yaw)*Ry(pitch)*Rx(roll). */
export function rpyToQuaternion(rpy: Vec3, target = new Quaternion()): Quaternion {
  return target.setFromEuler(new Euler(rpy[0], rpy[1], rpy[2], 'ZYX'));
}

function applyOrigin(object: Object3D, xyz: Vec3, rpy: Vec3): void {
  object.position.set(xyz[0], xyz[1], xyz[2]);
  rpyToQuaternion(rpy, object.quaternion);
}

/** Resolves a URDF mesh filename against the URDF's own URL. */
export function resolveMeshUrl(urdfUrl: string, filename: string): string {
  const packageMatch = /^package:\/\/[^/]+\/(.*)$/.exec(filename);
  const relative = packageMatch ? (packageMatch[1] as string) : filename;
  if (/^(https?:|data:|blob:|file:)/.test(relative) || relative.startsWith('/')) return relative;
  const base = urdfUrl.slice(0, urdfUrl.lastIndexOf('/') + 1);
  return `${base}${relative}`;
}

/**
 * The assembled robot. `setJointAngle` is the integration point for future
 * ST3215 joint-state telemetry; nothing in this class talks to any bus.
 */
export class MatdogRobot {
  readonly rootObject: Object3D;
  readonly model: UrdfModel;
  private readonly links = new Map<string, Object3D>();
  private readonly joints = new Map<string, JointHandle>();
  private readonly collisionMeshes = new Map<string, Mesh>();

  constructor(model: UrdfModel, rootObject: Object3D) {
    this.model = model;
    this.rootObject = rootObject;
  }

  /** @internal */
  registerLink(name: string, object: Object3D): void {
    this.links.set(name, object);
  }

  /** @internal */
  registerJoint(handle: JointHandle): void {
    this.joints.set(handle.joint.name, handle);
  }

  /** @internal */
  registerCollisionMesh(linkName: string, mesh: Mesh): void {
    this.collisionMeshes.set(linkName, mesh);
  }

  linkObject(name: string): Object3D | undefined {
    return this.links.get(name);
  }

  get linkNames(): readonly string[] {
    return [...this.links.keys()];
  }

  get jointNames(): readonly string[] {
    return [...this.joints.keys()];
  }

  get collisionMeshCount(): number {
    return this.collisionMeshes.size;
  }

  get collisionLinkNames(): readonly string[] {
    return [...this.collisionMeshes.keys()];
  }

  jointValue(jointName: string): number {
    const handle = this.joints.get(jointName);
    if (!handle) throw new RobotBuildError(`unknown joint: ${jointName}`);
    return handle.value;
  }

  /**
   * Sets one joint, in radians, clamped to the URDF limits.
   * Fixed joints are rejected. Returns the value actually applied.
   *
   * FUTURE ST3215 INTEGRATION POINT: a joint-state consumer converts servo
   * counts to radians (MATDOG_JOINT_CALIBRATION.yaml semantics) and calls
   * this method. This viewer never writes to a servo bus.
   */
  setJointAngle(jointName: string, radians: number): number {
    const handle = this.joints.get(jointName);
    if (!handle) throw new RobotBuildError(`unknown joint: ${jointName}`);
    if (handle.joint.type === 'fixed') {
      throw new RobotBuildError(`joint ${jointName} is fixed and cannot be actuated`);
    }
    if (!Number.isFinite(radians)) {
      throw new RobotBuildError(`joint ${jointName}: non-finite value ${radians}`);
    }

    let value = radians;
    const { lower, upper } = handle.joint;
    if (handle.joint.type !== 'continuous') {
      if (lower !== null) value = Math.max(value, lower);
      if (upper !== null) value = Math.min(value, upper);
    }

    const axis = new Vector3(...handle.joint.axis);
    if (axis.lengthSq() === 0) {
      throw new RobotBuildError(`joint ${jointName}: zero-length axis`);
    }
    axis.normalize();

    if (handle.joint.type === 'prismatic') {
      handle.articulationObject.position.copy(axis).multiplyScalar(value);
    } else {
      handle.articulationObject.quaternion.setFromAxisAngle(axis, value);
    }
    handle.value = value;
    return value;
  }

  /** Sets several joints at once; unknown names throw. */
  setJointAngles(values: Readonly<Record<string, number>>): void {
    for (const [name, radians] of Object.entries(values)) this.setJointAngle(name, radians);
  }

  /** Returns every actuatable joint to 0 rad. */
  resetJoints(): void {
    for (const handle of this.joints.values()) {
      if (handle.joint.type !== 'fixed') this.setJointAngle(handle.joint.name, 0);
    }
  }

  /** Snapshot of all actuatable joint values, for tests and diagnostics. */
  jointSnapshot(): Record<string, number> {
    const snapshot: Record<string, number> = {};
    for (const handle of this.joints.values()) {
      if (handle.joint.type !== 'fixed') snapshot[handle.joint.name] = handle.value;
    }
    return snapshot;
  }

  /** Applies the base_link orientation. Joint values are untouched. */
  setRootOrientation(quaternion: Quaternion): void {
    this.rootObject.quaternion.copy(quaternion);
  }
}

/**
 * Builds the robot. Every <collision> mesh in the model must load: a failure
 * rejects the whole build rather than silently rendering a partial robot.
 */
export async function buildRobot(options: BuildRobotOptions): Promise<MatdogRobot> {
  const { model, urdfUrl, loadMeshGeometry } = options;
  const styleForLink = options.styleForLink ?? (() => DEFAULT_STYLE);

  const rootObject = new Object3D();
  rootObject.name = `${model.name}__root`;

  const robot = new MatdogRobot(model, rootObject);

  const linkObjects = new Map<string, Object3D>();
  for (const link of model.links) {
    const object = new Object3D();
    object.name = link.name;
    object.userData.urdfLink = link.name;
    linkObjects.set(link.name, object);
    robot.registerLink(link.name, object);
  }

  // Wire the tree: parent link -> joint origin -> articulation -> child link.
  for (const joint of model.joints) {
    const parentObject = linkObjects.get(joint.parent);
    const childObject = linkObjects.get(joint.child);
    if (!parentObject || !childObject) {
      throw new RobotBuildError(`${joint.name}: parent or child link object missing`);
    }

    const originObject = new Object3D();
    originObject.name = `${joint.name}__origin`;
    originObject.userData.urdfJoint = joint.name;
    applyOrigin(originObject, joint.origin.xyz, joint.origin.rpy);

    const articulationObject = new Object3D();
    articulationObject.name = `${joint.name}__articulation`;
    articulationObject.userData.urdfJoint = joint.name;

    originObject.add(articulationObject);
    articulationObject.add(childObject);
    parentObject.add(originObject);

    robot.registerJoint({ joint, originObject, articulationObject, value: 0 });
  }

  const rootLinkObject = linkObjects.get(model.rootLink);
  if (!rootLinkObject) throw new RobotBuildError(`root link object missing: ${model.rootLink}`);
  rootObject.add(rootLinkObject);

  // Collision geometry, loaded in parallel. Any rejection fails the build.
  const collisionLinks = model.links.filter((link) => link.collision !== null);
  const loaded = await Promise.all(
    collisionLinks.map(async (link) => {
      const collision = link.collision!;
      const url = resolveMeshUrl(urdfUrl, collision.mesh.filename);
      try {
        const geometry = await loadMeshGeometry(url);
        return { link, collision, geometry };
      } catch (error) {
        throw new RobotBuildError(
          `${link.name}: failed to load collision mesh ${url}: ` +
            `${error instanceof Error ? error.message : String(error)}`,
        );
      }
    }),
  );

  for (const { link, collision, geometry } of loaded) {
    if (!geometry.getAttribute('position')) {
      throw new RobotBuildError(`${link.name}: collision mesh has no vertex positions`);
    }
    geometry.computeVertexNormals();

    const style = styleForLink(link.name);
    const material = new MeshStandardMaterial({
      color: style.color,
      metalness: style.metalness,
      roughness: style.roughness,
    });
    material.name = `${link.name}__collision_material`;

    const mesh = new Mesh(geometry, material);
    mesh.name = `${link.name}__collision`;
    mesh.userData.urdfLink = link.name;
    mesh.castShadow = false;
    mesh.receiveShadow = false;

    // <collision><origin> first, then the mesh unit scale (STL mm -> m).
    const collisionObject = new Object3D();
    collisionObject.name = `${link.name}__collision_origin`;
    applyOrigin(collisionObject, collision.origin.xyz, collision.origin.rpy);
    mesh.scale.set(collision.mesh.scale[0], collision.mesh.scale[1], collision.mesh.scale[2]);
    collisionObject.add(mesh);

    const linkObject = linkObjects.get(link.name);
    if (!linkObject) throw new RobotBuildError(`link object missing: ${link.name}`);
    linkObject.add(collisionObject);
    robot.registerCollisionMesh(link.name, mesh);
  }

  if (robot.collisionMeshCount !== collisionLinks.length) {
    throw new RobotBuildError(
      `collision mesh count mismatch: URDF declares ${collisionLinks.length}, ` +
        `scene contains ${robot.collisionMeshCount}`,
    );
  }

  rootObject.updateMatrixWorld(true);
  return robot;
}
