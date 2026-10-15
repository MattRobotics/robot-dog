/**
 * MTW1 mesh pack (scripts/build_mobile_twin_assets.py): decimated, quantized
 * DISPLAY geometry for the phone twin served by the robot AP. Never a
 * clearance model. Positions are millimetres, exactly like the STL loader.
 */
import { BufferAttribute, BufferGeometry } from 'three';
import type { UrdfModel } from './parseUrdf';
import { resolveMeshUrl, type MeshGeometryLoader } from './buildRobot';

export class MtwFormatError extends Error {
  constructor(message: string) { super(message); this.name = 'MtwFormatError'; }
}

export function parseMtw(buffer: ArrayBuffer): Map<string, BufferGeometry> {
  const view = new DataView(buffer), bytes = new Uint8Array(buffer);
  let at = 0;
  const need = (n: number): void => { if (at + n > buffer.byteLength) throw new MtwFormatError('truncated MTW1 pack'); };
  need(6);
  if (String.fromCharCode(bytes[0]!, bytes[1]!, bytes[2]!, bytes[3]!) !== 'MTW1') throw new MtwFormatError('not an MTW1 pack');
  at = 4;
  const count = view.getUint16(at, true); at += 2;
  if (count === 0 || count > 64) throw new MtwFormatError(`implausible mesh count ${count}`);
  const out = new Map<string, BufferGeometry>();
  for (let m = 0; m < count; m++) {
    need(1); const len = view.getUint8(at); at += 1;
    need(len); const name = new TextDecoder().decode(bytes.subarray(at, at + len)); at += len;
    if (!/^[a-z0-9_]+$/.test(name) || out.has(name)) throw new MtwFormatError(`bad or duplicate link name '${name}'`);
    need(32);
    const min = [view.getFloat32(at, true), view.getFloat32(at + 4, true), view.getFloat32(at + 8, true)];
    const scale = [view.getFloat32(at + 12, true), view.getFloat32(at + 16, true), view.getFloat32(at + 20, true)];
    const vertices = view.getUint32(at + 24, true), indices = view.getUint32(at + 28, true); at += 32;
    if (!min.concat(scale).every(Number.isFinite) || vertices === 0 || indices === 0 || indices % 3 !== 0 || vertices > 4_000_000)
      throw new MtwFormatError(`bad header for ${name}`);
    need(vertices * 6);
    const position = new Float32Array(vertices * 3);
    for (let i = 0; i < vertices * 3; i++) position[i] = min[i % 3]! + view.getUint16(at + i * 2, true) * scale[i % 3]!;
    at += vertices * 6;
    const wide = vertices > 65535, size = wide ? 4 : 2;
    need(indices * size);
    const index = wide ? new Uint32Array(indices) : new Uint16Array(indices);
    for (let i = 0; i < indices; i++) {
      const v = wide ? view.getUint32(at + i * 4, true) : view.getUint16(at + i * 2, true);
      if (v >= vertices) throw new MtwFormatError(`index out of range in ${name}`);
      index[i] = v;
    }
    at += indices * size;
    const geometry = new BufferGeometry();
    geometry.setAttribute('position', new BufferAttribute(position, 3));
    geometry.setIndex(new BufferAttribute(index, 1));
    geometry.computeVertexNormals();
    out.set(name, geometry);
  }
  if (at !== buffer.byteLength) throw new MtwFormatError('trailing bytes after MTW1 pack');
  return out;
}

/** Adapts an MTW1 pack to buildRobot's per-URL loader: one geometry per collision link. */
export function mtwMeshLoader(model: UrdfModel, urdfUrl: string, pack: Map<string, BufferGeometry>): MeshGeometryLoader {
  const byUrl = new Map<string, string>();
  for (const link of model.links) if (link.collision) byUrl.set(resolveMeshUrl(urdfUrl, link.collision.mesh.filename), link.name);
  return async (url: string) => {
    const link = byUrl.get(url);
    const geometry = link ? pack.get(link) : undefined;
    if (!geometry) throw new MtwFormatError(`no MTW1 mesh for ${url}`);
    return geometry.clone();
  };
}
