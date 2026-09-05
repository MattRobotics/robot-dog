/**
 * Node-side helpers shared by the test suite.
 *
 * The viewer itself never uses these: in the browser the URDF is parsed with
 * the platform DOMParser and meshes are fetched over HTTP. Under Node the
 * tests read the CANONICAL files straight from the repository, which is what
 * makes "every collision mesh in the canonical URDF is really loaded" a
 * meaningful assertion.
 */

import { readFile } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { DOMParser } from '@xmldom/xmldom';
import type { BufferGeometry } from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import type { MeshGeometryLoader } from '../../src/urdf/buildRobot';
import type { XmlDocumentParser } from '../../src/urdf/parseUrdf';

export const VIEWER_DIR = resolve(dirname(fileURLToPath(import.meta.url)), '../..');

export const CANONICAL_URDF_REPO_RELATIVE_PATH =
  '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf';

export function findRepoRoot(startDir: string = VIEWER_DIR): string {
  let current = resolve(startDir);
  for (;;) {
    if (existsSync(join(current, CANONICAL_URDF_REPO_RELATIVE_PATH))) return current;
    const parent = dirname(current);
    if (parent === current) {
      throw new Error(`canonical URDF not found above ${startDir}`);
    }
    current = parent;
  }
}

export const REPO_ROOT = findRepoRoot();
export const CANONICAL_URDF_PATH = join(REPO_ROOT, CANONICAL_URDF_REPO_RELATIVE_PATH);
export const CANONICAL_URDF_DIR = dirname(CANONICAL_URDF_PATH);

export const nodeXmlParser: XmlDocumentParser = (xml) =>
  new DOMParser().parseFromString(xml, 'text/xml') as unknown as Document;

export async function readCanonicalUrdfText(): Promise<string> {
  return readFile(CANONICAL_URDF_PATH, 'utf8');
}

/**
 * Mesh loader backed by the real canonical STL files. URLs are of the form
 * `<urdfUrl dir>/meshes/collision/<link>.stl`, so the prefix used by the
 * tests is simply stripped back to a repository path.
 */
export function createFileStlLoader(urlPrefix: string): MeshGeometryLoader {
  const loader = new STLLoader();
  return async (url: string): Promise<BufferGeometry> => {
    if (!url.startsWith(urlPrefix)) throw new Error(`unexpected mesh url: ${url}`);
    const relative = url.slice(urlPrefix.length);
    const absolute = join(CANONICAL_URDF_DIR, relative);
    const buffer = await readFile(absolute);
    const arrayBuffer = buffer.buffer.slice(
      buffer.byteOffset,
      buffer.byteOffset + buffer.byteLength,
    ) as ArrayBuffer;
    return loader.parse(arrayBuffer);
  };
}

export const TEST_URDF_URL_PREFIX = 'https://test.invalid/canonical/matt_robodog_rev00/';
export const TEST_URDF_URL = `${TEST_URDF_URL_PREFIX}matt_robodog_rev00.urdf`;
