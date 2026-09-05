#!/usr/bin/env node
/**
 * MATDOG BNO085 full-body viewer — canonical asset staging.
 *
 * READ-ONLY with respect to the canonical model. This script:
 *   1. locates the canonical URDF in the repository;
 *   2. parses it and extracts every <collision> mesh reference;
 *   3. verifies that every referenced collision STL exists;
 *   4. copies the URDF verbatim plus ONLY the collision STLs into
 *      public/canonical/<model>/ so Vite can serve them;
 *   5. writes CANONICAL_PROVENANCE.json (sha256 of every staged file,
 *      counts, source paths) so the staged copy's origin is obvious.
 *
 * The canonical URDF and the canonical STL files are never written to.
 * The staged copy is gitignored and fully regenerated on every run, so
 * there is exactly ONE maintained geometry definition: the canonical URDF.
 */

import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { dirname, join, posix, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const VIEWER_DIR = resolve(dirname(fileURLToPath(import.meta.url)), '..');

export const CANONICAL_URDF_REPO_RELATIVE_PATH =
  '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf';

export const STAGED_MODEL_DIR_NAME = 'matt_robodog_rev00';

/** Walks up from `startDir` until the canonical URDF is found. */
export function findRepoRoot(startDir = VIEWER_DIR) {
  let current = resolve(startDir);
  for (;;) {
    if (existsSync(join(current, CANONICAL_URDF_REPO_RELATIVE_PATH))) return current;
    const parent = dirname(current);
    if (parent === current) {
      throw new Error(
        `Canonical URDF not found. Walked up from ${startDir} looking for ` +
          `${CANONICAL_URDF_REPO_RELATIVE_PATH}`,
      );
    }
    current = parent;
  }
}

export function canonicalUrdfPath(repoRoot = findRepoRoot()) {
  return join(repoRoot, CANONICAL_URDF_REPO_RELATIVE_PATH);
}

/**
 * Minimal, dependency-free scan of the canonical URDF for collision mesh
 * references. Intentionally independent from the runtime TypeScript parser:
 * if the two ever disagree, the tests fail instead of silently skipping a mesh.
 */
export function collisionMeshReferences(urdfText) {
  const references = [];
  const linkPattern = /<link\b[^>]*\bname\s*=\s*"([^"]+)"[\s\S]*?<\/link>/g;
  for (const linkMatch of urdfText.matchAll(linkPattern)) {
    const linkName = linkMatch[1];
    const collisionPattern = /<collision\b[\s\S]*?<\/collision>/g;
    for (const collisionMatch of linkMatch[0].matchAll(collisionPattern)) {
      const meshMatch = /<mesh\b[^>]*\bfilename\s*=\s*"([^"]+)"[^>]*>/.exec(collisionMatch[0]);
      if (!meshMatch) {
        throw new Error(`${linkName}: <collision> without an STL <mesh filename=...>`);
      }
      references.push({ linkName, filename: meshMatch[1] });
    }
  }
  return references;
}

export function sha256(buffer) {
  return createHash('sha256').update(buffer).digest('hex');
}

function stripUriScheme(filename) {
  // Tolerate package:// even though the canonical REV00 URDF uses plain
  // relative paths.
  const packageMatch = /^package:\/\/[^/]+\/(.*)$/.exec(filename);
  if (packageMatch) return packageMatch[1];
  if (filename.startsWith('file://')) return filename.slice('file://'.length);
  return filename;
}

export function syncCanonicalAssets({ quiet = false } = {}) {
  const repoRoot = findRepoRoot();
  const urdfPath = canonicalUrdfPath(repoRoot);
  const urdfDir = dirname(urdfPath);
  const urdfBuffer = readFileSync(urdfPath);
  const references = collisionMeshReferences(urdfBuffer.toString('utf8'));

  if (references.length === 0) {
    throw new Error(`No <collision> meshes found in ${urdfPath}`);
  }

  const stagedRoot = join(VIEWER_DIR, 'public', 'canonical', STAGED_MODEL_DIR_NAME);
  rmSync(stagedRoot, { recursive: true, force: true });
  mkdirSync(stagedRoot, { recursive: true });

  const stagedUrdfName = 'matt_robodog_rev00.urdf';
  writeFileSync(join(stagedRoot, stagedUrdfName), urdfBuffer);

  const meshes = [];
  const seen = new Set();
  for (const { linkName, filename } of references) {
    const relativeMeshPath = stripUriScheme(filename);
    const sourcePath = resolve(urdfDir, relativeMeshPath);
    if (!existsSync(sourcePath)) {
      throw new Error(`${linkName}: collision mesh does not exist: ${sourcePath}`);
    }
    const meshBuffer = readFileSync(sourcePath);
    if (meshBuffer.length === 0) {
      throw new Error(`${linkName}: collision mesh is empty: ${sourcePath}`);
    }
    // Git-LFS pointer files are text stubs; refuse to stage them silently.
    if (meshBuffer.subarray(0, 40).toString('utf8').startsWith('version https://git-lfs')) {
      throw new Error(
        `${linkName}: collision mesh is an unfetched git-lfs pointer: ${sourcePath}. ` +
          'Run "git lfs pull" in the repository first.',
      );
    }

    const destinationPath = join(stagedRoot, relativeMeshPath);
    mkdirSync(dirname(destinationPath), { recursive: true });
    writeFileSync(destinationPath, meshBuffer);

    if (!seen.has(relativeMeshPath)) {
      seen.add(relativeMeshPath);
      meshes.push({
        link: linkName,
        urdf_filename: filename,
        staged_relative_path: relativeMeshPath.split(/[\\/]/).join(posix.sep),
        source_repo_relative_path: relative(repoRoot, sourcePath).split(/[\\/]/).join(posix.sep),
        bytes: meshBuffer.length,
        sha256: sha256(meshBuffer),
      });
    }
  }

  const provenance = {
    generated_by: '06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer/scripts/sync_canonical_assets.mjs',
    note:
      'Generated artefact. Do not edit by hand and do not commit. ' +
      'The single source of truth is the canonical URDF listed below.',
    canonical_urdf: {
      repo_relative_path: CANONICAL_URDF_REPO_RELATIVE_PATH,
      bytes: urdfBuffer.length,
      sha256: sha256(urdfBuffer),
      staged_as: stagedUrdfName,
    },
    collision_mesh_count: meshes.length,
    collision_meshes: meshes.sort((a, b) => a.link.localeCompare(b.link)),
  };

  writeFileSync(
    join(stagedRoot, 'CANONICAL_PROVENANCE.json'),
    `${JSON.stringify(provenance, null, 2)}\n`,
  );

  if (!quiet) {
    console.log(`[sync] canonical URDF : ${relative(repoRoot, urdfPath)}`);
    console.log(`[sync] urdf sha256    : ${provenance.canonical_urdf.sha256}`);
    console.log(`[sync] collision meshes staged: ${meshes.length}`);
    console.log(`[sync] staged into    : ${relative(VIEWER_DIR, stagedRoot)}`);
  }

  return provenance;
}

const invokedDirectly =
  process.argv[1] && resolve(process.argv[1]) === resolve(fileURLToPath(import.meta.url));

if (invokedDirectly) {
  try {
    syncCanonicalAssets();
  } catch (error) {
    console.error(`[sync] FAILED: ${error instanceof Error ? error.message : String(error)}`);
    process.exit(1);
  }
}
