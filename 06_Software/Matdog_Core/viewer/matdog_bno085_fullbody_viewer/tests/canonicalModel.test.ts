/**
 * Validates the canonical source of truth itself and the staging that feeds
 * the viewer. These tests read the repository's canonical URDF directly —
 * they never read a viewer-local copy of the geometry.
 */

import { existsSync, readFileSync, statSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { describe, expect, it } from 'vitest';

import {
  collisionMeshReferences,
  findRepoRoot,
  canonicalUrdfPath,
  sha256,
  syncCanonicalAssets,
} from '../scripts/sync_canonical_assets.mjs';
import {
  CANONICAL_URDF_PATH,
  CANONICAL_URDF_DIR,
  nodeXmlParser,
  readCanonicalUrdfText,
  VIEWER_DIR,
} from './support/nodeEnvironment';
import { collisionLinks, movableJointNames, parseUrdf } from '../src/urdf/parseUrdf';

/**
 * The MATDOG REV00 baseline: 1 base link + 4 legs x 4 links.
 * Also pinned in 06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py.
 */
const EXPECTED_LINK_COUNT = 17;
const EXPECTED_JOINT_COUNT = 16;
const EXPECTED_REVOLUTE_JOINT_COUNT = 12;
const EXPECTED_FIXED_JOINT_COUNT = 4;
const EXPECTED_COLLISION_MESH_COUNT = 17;

describe('canonical URDF discovery', () => {
  it('finds the canonical URDF from the viewer directory', () => {
    expect(existsSync(CANONICAL_URDF_PATH)).toBe(true);
    expect(canonicalUrdfPath(findRepoRoot(VIEWER_DIR))).toBe(CANONICAL_URDF_PATH);
  });

  it('is the URDF revision pinned by the MATDOG kinematics module', () => {
    // Pin from 06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py.
    const pinnedSource = readFileSync(
      join(findRepoRoot(VIEWER_DIR), '06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py'),
      'utf8',
    );
    const pinned = /CANONICAL_URDF_SHA256 = \(\s*"([0-9a-f]{64})"/.exec(pinnedSource);
    expect(pinned, 'CANONICAL_URDF_SHA256 pin not found').not.toBeNull();
    expect(sha256(readFileSync(CANONICAL_URDF_PATH))).toBe(pinned![1]);
  });
});

describe('canonical URDF topology', () => {
  it('parses to the expected MATDOG REV00 topology', async () => {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);

    expect(model.name).toBe('matt_robodog_rev00');
    expect(model.rootLink).toBe('base_link');
    expect(model.links).toHaveLength(EXPECTED_LINK_COUNT);
    expect(model.joints).toHaveLength(EXPECTED_JOINT_COUNT);

    const revolute = model.joints.filter((joint) => joint.type === 'revolute');
    const fixed = model.joints.filter((joint) => joint.type === 'fixed');
    expect(revolute).toHaveLength(EXPECTED_REVOLUTE_JOINT_COUNT);
    expect(fixed).toHaveLength(EXPECTED_FIXED_JOINT_COUNT);
    expect(movableJointNames(model)).toHaveLength(EXPECTED_REVOLUTE_JOINT_COUNT);
  });

  it('declares one collision mesh per link, all in the collision directory', async () => {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
    const withCollision = collisionLinks(model);

    expect(withCollision).toHaveLength(EXPECTED_COLLISION_MESH_COUNT);
    expect(withCollision).toHaveLength(model.links.length);

    for (const link of withCollision) {
      const collision = link.collision!;
      expect(collision.mesh.filename, `${link.name} must use the collision mesh set`).toContain(
        'meshes/collision/',
      );
      // STLs are authored in millimetres; the URDF stays SI via this scale.
      expect(collision.mesh.scale).toEqual([0.001, 0.001, 0.001]);
    }
  });

  it('every referenced collision mesh resolves to a real, non-empty binary STL', async () => {
    const model = parseUrdf(await readCanonicalUrdfText(), nodeXmlParser);
    const resolved: string[] = [];

    for (const link of collisionLinks(model)) {
      const meshPath = resolve(CANONICAL_URDF_DIR, link.collision!.mesh.filename);
      expect(existsSync(meshPath), `${link.name}: missing ${meshPath}`).toBe(true);
      const stats = statSync(meshPath);
      expect(stats.size, `${link.name}: empty mesh`).toBeGreaterThan(84);
      const head = readFileSync(meshPath).subarray(0, 40).toString('utf8');
      expect(head.startsWith('version https://git-lfs'), `${link.name}: unfetched LFS pointer`).toBe(
        false,
      );
      resolved.push(link.name);
    }

    expect(resolved).toHaveLength(EXPECTED_COLLISION_MESH_COUNT);
  });

  it('the independent regex scanner agrees with the TypeScript parser', async () => {
    // Two independent implementations must see the same set of collision
    // meshes; if one silently skipped a mesh this comparison fails.
    const urdfText = await readCanonicalUrdfText();
    const model = parseUrdf(urdfText, nodeXmlParser);

    const fromScanner = collisionMeshReferences(urdfText)
      .map((reference) => `${reference.linkName}:${reference.filename}`)
      .sort();
    const fromParser = collisionLinks(model)
      .map((link) => `${link.name}:${link.collision!.mesh.filename}`)
      .sort();

    expect(fromScanner).toEqual(fromParser);
    expect(fromScanner).toHaveLength(EXPECTED_COLLISION_MESH_COUNT);
  });
});

describe('canonical asset staging', () => {
  it('stages the URDF verbatim plus exactly the collision meshes', () => {
    const provenance = syncCanonicalAssets({ quiet: true });
    const stagedRoot = join(VIEWER_DIR, 'public', 'canonical', 'matt_robodog_rev00');

    expect(provenance.collision_mesh_count).toBe(EXPECTED_COLLISION_MESH_COUNT);
    expect(provenance.canonical_urdf.sha256).toBe(sha256(readFileSync(CANONICAL_URDF_PATH)));

    const stagedUrdf = join(stagedRoot, provenance.canonical_urdf.staged_as);
    expect(readFileSync(stagedUrdf)).toEqual(readFileSync(CANONICAL_URDF_PATH));

    for (const mesh of provenance.collision_meshes) {
      const staged = join(stagedRoot, mesh.staged_relative_path);
      expect(existsSync(staged), `${mesh.link}: not staged`).toBe(true);
      expect(sha256(readFileSync(staged)), `${mesh.link}: staged copy differs`).toBe(mesh.sha256);
      expect(
        readFileSync(join(CANONICAL_URDF_DIR, mesh.staged_relative_path)).length,
        `${mesh.link}: source/staged size mismatch`,
      ).toBe(mesh.bytes);
    }
  });

  it('leaves the canonical model untouched', () => {
    // Staging only reads: verify the canonical tree still matches SHA256SUMS.txt
    // entries for the collision meshes it lists.
    const sumsPath = join(CANONICAL_URDF_DIR, 'SHA256SUMS.txt');
    if (!existsSync(sumsPath)) return;

    const lines = readFileSync(sumsPath, 'utf8')
      .split('\n')
      .map((line) => line.trim())
      .filter((line) => line.includes('meshes/collision/'));

    expect(lines.length, 'SHA256SUMS.txt lists no collision meshes').toBeGreaterThan(0);

    for (const line of lines) {
      const match = /^([0-9a-f]{64})\s+\*?(.+)$/.exec(line);
      if (!match) continue;
      const meshPath = resolve(CANONICAL_URDF_DIR, match[2]!);
      if (!existsSync(meshPath)) continue;
      expect(sha256(readFileSync(meshPath)), `${match[2]} was modified`).toBe(match[1]);
    }
  });
});

describe('viewer isolation', () => {
  it('keeps every viewer source file inside the viewer directory', () => {
    expect(dirname(VIEWER_DIR).endsWith(join('Matdog_Core', 'viewer'))).toBe(true);
  });
});
