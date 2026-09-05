/**
 * Minimal URDF parser for the MATDOG canonical model.
 *
 * Pure and dependency-free: takes URDF XML text, returns a plain description
 * of the link/joint tree plus the <collision> mesh references. It contains no
 * Three.js and no DOM-renderer code so it can be unit tested under Node.
 *
 * Scope is deliberately limited to what the canonical REV00 URDF uses:
 *   - <link> with an optional single <collision><geometry><mesh>
 *   - <joint type="revolute"|"fixed"|"continuous"|"prismatic"> with
 *     <origin xyz rpy>, <axis xyz> and <limit lower upper>
 * Unknown elements (<inertial>, <visual>, <material>, the MATDOG
 * <hardware> block) are ignored.
 */

export type Vec3 = readonly [number, number, number];

export interface UrdfOrigin {
  readonly xyz: Vec3;
  /** URDF fixed-axis roll-pitch-yaw, radians. */
  readonly rpy: Vec3;
}

export interface UrdfMesh {
  /** Verbatim filename attribute from the URDF. */
  readonly filename: string;
  readonly scale: Vec3;
}

export interface UrdfCollision {
  readonly origin: UrdfOrigin;
  readonly mesh: UrdfMesh;
}

export interface UrdfLink {
  readonly name: string;
  readonly collision: UrdfCollision | null;
}

export type UrdfJointType =
  | 'revolute'
  | 'continuous'
  | 'prismatic'
  | 'fixed'
  | 'floating'
  | 'planar';

export interface UrdfJoint {
  readonly name: string;
  readonly type: UrdfJointType;
  readonly parent: string;
  readonly child: string;
  readonly origin: UrdfOrigin;
  readonly axis: Vec3;
  readonly lower: number | null;
  readonly upper: number | null;
}

export interface UrdfModel {
  readonly name: string;
  readonly links: readonly UrdfLink[];
  readonly joints: readonly UrdfJoint[];
  /** The single link that is never a joint child. */
  readonly rootLink: string;
}

export class UrdfParseError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'UrdfParseError';
  }
}

const IDENTITY_ORIGIN: UrdfOrigin = { xyz: [0, 0, 0], rpy: [0, 0, 0] };
const DEFAULT_AXIS: Vec3 = [1, 0, 0];
const UNIT_SCALE: Vec3 = [1, 1, 1];

const JOINT_TYPES: ReadonlySet<string> = new Set<UrdfJointType>([
  'revolute',
  'continuous',
  'prismatic',
  'fixed',
  'floating',
  'planar',
]);

function parseVec3(raw: string | null, fallback: Vec3, field: string): Vec3 {
  if (raw === null) return fallback;
  const tokens = raw.trim().split(/\s+/);
  if (tokens.length !== 3) {
    throw new UrdfParseError(`${field}: expected 3 numbers, got ${JSON.stringify(raw)}`);
  }
  const values = tokens.map((token) => Number(token));
  if (values.some((value) => !Number.isFinite(value))) {
    throw new UrdfParseError(`${field}: non-finite value in ${JSON.stringify(raw)}`);
  }
  return [values[0] as number, values[1] as number, values[2] as number];
}

function parseNumber(raw: string | null, field: string): number | null {
  if (raw === null) return null;
  const value = Number(raw);
  if (!Number.isFinite(value)) {
    throw new UrdfParseError(`${field}: non-finite value ${JSON.stringify(raw)}`);
  }
  return value;
}

function parseOrigin(parent: Element, field: string): UrdfOrigin {
  const node = firstChildElement(parent, 'origin');
  if (!node) return IDENTITY_ORIGIN;
  return {
    xyz: parseVec3(node.getAttribute('xyz'), [0, 0, 0], `${field}.origin.xyz`),
    rpy: parseVec3(node.getAttribute('rpy'), [0, 0, 0], `${field}.origin.rpy`),
  };
}

function childElements(parent: Element, tagName: string): Element[] {
  const result: Element[] = [];
  for (const child of Array.from(parent.children)) {
    if (child.tagName.toLowerCase() === tagName) result.push(child);
  }
  return result;
}

function firstChildElement(parent: Element, tagName: string): Element | null {
  return childElements(parent, tagName)[0] ?? null;
}

function parseCollision(linkNode: Element, linkName: string): UrdfCollision | null {
  const collisions = childElements(linkNode, 'collision');
  if (collisions.length === 0) return null;
  if (collisions.length > 1) {
    throw new UrdfParseError(
      `${linkName}: this viewer supports exactly one <collision> per link, found ${collisions.length}`,
    );
  }
  const collision = collisions[0] as Element;
  const geometry = firstChildElement(collision, 'geometry');
  if (!geometry) {
    throw new UrdfParseError(`${linkName}: <collision> without <geometry>`);
  }
  const mesh = firstChildElement(geometry, 'mesh');
  if (!mesh) {
    throw new UrdfParseError(
      `${linkName}: <collision> geometry is not a <mesh>; primitive collision shapes are not supported`,
    );
  }
  const filename = mesh.getAttribute('filename');
  if (!filename) {
    throw new UrdfParseError(`${linkName}: <collision> mesh has no filename`);
  }
  const scale = parseVec3(mesh.getAttribute('scale'), UNIT_SCALE, `${linkName}.collision.mesh.scale`);
  if (scale.some((value) => value === 0)) {
    throw new UrdfParseError(`${linkName}: <collision> mesh scale contains zero: ${scale.join(' ')}`);
  }
  return {
    origin: parseOrigin(collision, `${linkName}.collision`),
    mesh: { filename, scale },
  };
}

function parseJoint(jointNode: Element): UrdfJoint {
  const name = jointNode.getAttribute('name');
  if (!name) throw new UrdfParseError('<joint> without a name attribute');

  const rawType = (jointNode.getAttribute('type') ?? '').toLowerCase();
  if (!JOINT_TYPES.has(rawType)) {
    throw new UrdfParseError(`${name}: unsupported joint type ${JSON.stringify(rawType)}`);
  }

  const parent = firstChildElement(jointNode, 'parent')?.getAttribute('link');
  const child = firstChildElement(jointNode, 'child')?.getAttribute('link');
  if (!parent) throw new UrdfParseError(`${name}: <parent link=...> missing`);
  if (!child) throw new UrdfParseError(`${name}: <child link=...> missing`);

  const axisNode = firstChildElement(jointNode, 'axis');
  const axis = parseVec3(axisNode?.getAttribute('xyz') ?? null, DEFAULT_AXIS, `${name}.axis`);

  const limitNode = firstChildElement(jointNode, 'limit');
  const lower = parseNumber(limitNode?.getAttribute('lower') ?? null, `${name}.limit.lower`);
  const upper = parseNumber(limitNode?.getAttribute('upper') ?? null, `${name}.limit.upper`);

  return {
    name,
    type: rawType as UrdfJointType,
    parent,
    child,
    origin: parseOrigin(jointNode, name),
    axis,
    lower,
    upper,
  };
}

/**
 * DOM parsing indirection so the same code runs in the browser (DOMParser)
 * and under Node tests (a small injected parser).
 */
export type XmlDocumentParser = (xml: string) => Document;

export const browserXmlParser: XmlDocumentParser = (xml) =>
  new DOMParser().parseFromString(xml, 'application/xml');

export function parseUrdf(xml: string, parseXml: XmlDocumentParser = browserXmlParser): UrdfModel {
  const document = parseXml(xml);

  const parserError = document.getElementsByTagName('parsererror')[0];
  if (parserError) {
    throw new UrdfParseError(`URDF XML is malformed: ${parserError.textContent ?? 'unknown error'}`);
  }

  const robot = document.documentElement;
  if (!robot || robot.tagName.toLowerCase() !== 'robot') {
    throw new UrdfParseError('URDF root element is not <robot>');
  }

  const links: UrdfLink[] = [];
  const linkNames = new Set<string>();
  for (const linkNode of childElements(robot, 'link')) {
    const name = linkNode.getAttribute('name');
    if (!name) throw new UrdfParseError('<link> without a name attribute');
    if (linkNames.has(name)) throw new UrdfParseError(`duplicate link name: ${name}`);
    linkNames.add(name);
    links.push({ name, collision: parseCollision(linkNode, name) });
  }
  if (links.length === 0) throw new UrdfParseError('URDF contains no <link>');

  const joints: UrdfJoint[] = [];
  const jointNames = new Set<string>();
  const childLinks = new Set<string>();
  for (const jointNode of childElements(robot, 'joint')) {
    const joint = parseJoint(jointNode);
    if (jointNames.has(joint.name)) throw new UrdfParseError(`duplicate joint name: ${joint.name}`);
    jointNames.add(joint.name);
    if (!linkNames.has(joint.parent)) {
      throw new UrdfParseError(`${joint.name}: unknown parent link ${joint.parent}`);
    }
    if (!linkNames.has(joint.child)) {
      throw new UrdfParseError(`${joint.name}: unknown child link ${joint.child}`);
    }
    if (childLinks.has(joint.child)) {
      throw new UrdfParseError(`link ${joint.child} is the child of more than one joint`);
    }
    childLinks.add(joint.child);
    joints.push(joint);
  }

  const roots = links.filter((link) => !childLinks.has(link.name));
  if (roots.length !== 1) {
    throw new UrdfParseError(
      `expected exactly one root link, found ${roots.length}: ${roots.map((l) => l.name).join(', ')}`,
    );
  }

  return {
    name: robot.getAttribute('name') ?? 'robot',
    links,
    joints,
    rootLink: (roots[0] as UrdfLink).name,
  };
}

/** Links that carry a collision mesh, in URDF document order. */
export function collisionLinks(model: UrdfModel): readonly UrdfLink[] {
  return model.links.filter((link) => link.collision !== null);
}

/** Names of the joints this viewer can actuate (revolute/continuous/prismatic). */
export function movableJointNames(model: UrdfModel): readonly string[] {
  return model.joints.filter((joint) => joint.type !== 'fixed').map((joint) => joint.name);
}
