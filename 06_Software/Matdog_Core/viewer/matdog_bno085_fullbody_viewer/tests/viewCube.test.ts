import { describe, expect, it } from 'vitest';
import { Matrix4, Object3D, PerspectiveCamera, Vector3 } from 'three';
import { CUBE_FACES, POLE_EPSILON, createViewCube, cubeViewport, faceViewOffset, insideCube, type CubeFace } from '../src/viewer/viewCube';

Object3D.DEFAULT_UP.set(0, 0, 1);  // the viewer world is Z-up (scene.ts)

/** Screen right/up of a camera at `eye` looking at the origin with world up +Z. */
function screenAxes(eye: Vector3): { right: Vector3; up: Vector3 } {
  const m = new Matrix4().lookAt(eye, new Vector3(), new Vector3(0, 0, 1));
  return { right: new Vector3().setFromMatrixColumn(m, 0), up: new Vector3().setFromMatrixColumn(m, 1) };
}
const close = (a: Vector3, b: [number, number, number], tol = 1e-2): void => {
  expect(a.x).toBeCloseTo(b[0], -Math.log10(tol)); expect(a.y).toBeCloseTo(b[1], -Math.log10(tol)); expect(a.z).toBeCloseTo(b[2], -Math.log10(tol));
};

describe('view cube faces (camera only)', () => {
  it('places the camera on each face axis at the same distance', () => {
    for (const { face, normal } of CUBE_FACES) {
      const o = new Vector3(...faceViewOffset(face, 0.9));
      expect(o.length()).toBeCloseTo(0.9, 12);
      expect(o.clone().normalize().angleTo(new Vector3(...normal))).toBeLessThanOrEqual(POLE_EPSILON + 1e-12);
    }
    expect(faceViewOffset('front', NaN)).toEqual([1, 0, 0]);
  });
  it('keeps MATDOG +X as the front; CAD top/bottom: front down from above, up from below', () => {
    const expected: Record<CubeFace, { right: [number, number, number]; up: [number, number, number] }> = {
      front: { right: [0, 1, 0], up: [0, 0, 1] },     // the robot faces the viewer: its left is on the right
      back: { right: [0, -1, 0], up: [0, 0, 1] },
      left: { right: [-1, 0, 0], up: [0, 0, 1] },     // seen from +Y: the front is on the left
      right: { right: [1, 0, 0], up: [0, 0, 1] },
      top: { right: [0, 1, 0], up: [-1, 0, 0] },      // tilted up from the front view: same left/right
      bottom: { right: [0, 1, 0], up: [1, 0, 0] },    // tilted down from the front view
    };
    for (const face of Object.keys(expected) as CubeFace[]) {
      const axes = screenAxes(new Vector3(...faceViewOffset(face, 1)));
      close(axes.right, expected[face].right); close(axes.up, expected[face].up);
    }
  });
  it('prints every label upright and readable from its own view', () => {
    for (const { face, normal, up } of CUBE_FACES) {
      const axes = screenAxes(new Vector3(...faceViewOffset(face, 1)));
      close(axes.up, up);
      close(axes.right, new Vector3().crossVectors(new Vector3(...up), new Vector3(...normal)).toArray() as [number, number, number]);
    }
  });
});

describe('view cube picking and viewport', () => {
  it('returns the face the camera looks at, for all six views', () => {
    const cube = createViewCube({ x: 0xff5b5b, y: 0x5bff8f, z: 0x6ba6ff }), main = new PerspectiveCamera(45, 1, 0.01, 100);
    const w = 400, h = 300, v = cubeViewport(w, h), cx = v.x + v.size / 2, cy = h - v.y - v.size / 2;
    for (const { face } of CUBE_FACES) {
      main.position.set(...faceViewOffset(face, 0.8)); main.lookAt(0, 0, 0); main.updateMatrixWorld();
      cube.orient(main);
      expect(cube.pick(cx, cy, w, h)).toBe(face);
      expect(cube.pick(v.x + 2, h - v.y - v.size + 2, w, h)).toBeNull();  // corner: off the cube
    }
    expect(cube.pick(10, 10, w, h)).toBeNull();
    cube.dispose();
  });
  it('picks the top face above the front face from the default elevated view', () => {
    const cube = createViewCube({ x: 1, y: 2, z: 3 }), main = new PerspectiveCamera(45, 1, 0.01, 100);
    main.position.set(Math.cos(Math.PI / 7), 0, Math.sin(Math.PI / 7)); main.lookAt(0, 0, 0); main.updateMatrixWorld();
    cube.orient(main);
    const w = 400, h = 300, v = cubeViewport(w, h), cx = v.x + v.size / 2, top = h - v.y - v.size;
    expect(cube.pick(cx, top + v.size / 2, w, h)).toBe('front');
    expect(cube.pick(cx, top + v.size / 2 - 0.2 * v.size * 0.5, w, h)).toBe('top');
    cube.dispose();
  });
  it('sits in the top-right corner, 80-130 px, and only there', () => {
    for (const [w, h] of [[390, 280], [1280, 460], [200, 200]] as const) {
      const v = cubeViewport(w, h);
      expect(v.size).toBeGreaterThanOrEqual(80); expect(v.size).toBeLessThanOrEqual(130);
      expect(v.x + v.size).toBe(w - 8); expect(v.y + v.size).toBe(h - 8);
      expect(insideCube(w - 8 - v.size / 2, 8 + v.size / 2, w, h)).toBe(true);
      expect(insideCube(w / 2, h / 2, w, h)).toBe(false);
      expect(insideCube(w - 4, 4, w, h)).toBe(false);
    }
  });
});
