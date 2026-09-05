/**
 * Browser collision-STL loader.
 *
 * Fetches the staged canonical STL and parses it with three's STLLoader.
 * A non-OK response or an unparsable body rejects, which makes the robot
 * build fail loudly instead of rendering a silently incomplete model.
 */

import type { BufferGeometry } from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import type { MeshGeometryLoader } from '../urdf/buildRobot';

export function createFetchStlLoader(): MeshGeometryLoader {
  const loader = new STLLoader();
  return async (url: string): Promise<BufferGeometry> => {
    const response = await fetch(url);
    if (!response.ok) {
      throw new Error(`HTTP ${response.status} ${response.statusText}`);
    }
    const buffer = await response.arrayBuffer();
    if (buffer.byteLength === 0) throw new Error('empty STL response');
    return loader.parse(buffer);
  };
}
