/**
 * CAD view cube with an XYZ triad (dev.17). Camera only: a face click turns the
 * existing camera to that axis-aligned view about the same target and distance;
 * a drag stays OrbitControls' rotation. It never touches the robot model, the
 * IMU quaternion, the URDF or any request. It is drawn in a corner of the
 * existing renderer (scissor) with its own small scene and an orthographic
 * camera that copies the main camera's orientation.
 *
 * World axes (scene.ts, Z-up): +X MATDOG front, +Y left, +Z up. After "Reset
 * vista" the robot's +X is the world +X, so FRONTE is the robot's front view.
 */
import {
  ArrowHelper,
  BoxGeometry,
  CanvasTexture,
  EdgesGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  Mesh,
  MeshBasicMaterial,
  OrthographicCamera,
  PlaneGeometry,
  Raycaster,
  SRGBColorSpace,
  Scene,
  Sprite,
  SpriteMaterial,
  Vector2,
  Vector3,
  type Camera,
  type Material,
  type WebGLRenderer,
} from 'three';

export type CubeFace = 'front' | 'back' | 'left' | 'right' | 'top' | 'bottom';
type Vec3 = [number, number, number];
export interface FaceSpec { readonly face: CubeFace; readonly label: string; readonly normal: Vec3; readonly up: Vec3 }

/**
 * Outward normal and text-up direction of each face (text reads from outside).
 * CAD convention: the top view has the front at the bottom of the screen and
 * the bottom view has it at the top, so left/right stay those of the front view
 * and ALTO reads upright from the default front-high view.
 */
export const CUBE_FACES: readonly FaceSpec[] = [
  { face: 'front', label: 'FRONTE', normal: [1, 0, 0], up: [0, 0, 1] },
  { face: 'back', label: 'RETRO', normal: [-1, 0, 0], up: [0, 0, 1] },
  { face: 'left', label: 'SINISTRA', normal: [0, 1, 0], up: [0, 0, 1] },
  { face: 'right', label: 'DESTRA', normal: [0, -1, 0], up: [0, 0, 1] },
  { face: 'top', label: 'ALTO', normal: [0, 0, 1], up: [-1, 0, 0] },
  { face: 'bottom', label: 'BASSO', normal: [0, 0, -1], up: [1, 0, 0] },
];

/**
 * Tilt off the Z axis for the top and bottom views: lookAt and OrbitControls are
 * singular exactly on the up axis. Toward +X: the front is screen-down from
 * above and screen-up from below (the label directions above).
 */
export const POLE_EPSILON = 1e-3;

/** Camera offset from the target for a face view at `distance` (side views level). */
export function faceViewOffset(face: CubeFace, distance: number): Vec3 {
  const d = Number.isFinite(distance) && distance > 0 ? distance : 1;
  const [nx, ny, nz] = CUBE_FACES.find((f) => f.face === face)?.normal ?? [1, 0, 0];
  if (nz !== 0) return [d * Math.sin(POLE_EPSILON), 0, nz * d * Math.cos(POLE_EPSILON)];
  return [nx * d, ny * d, 0];
}

/** Square viewport of the cube in the top-right corner: CSS px, WebGL origin bottom-left. */
export function cubeViewport(width: number, height: number): { x: number; y: number; size: number } {
  const margin = 8;
  const size = Math.round(Math.min(130, Math.max(80, 0.3 * Math.min(width, height))));
  return { x: width - size - margin, y: height - size - margin, size };
}

/** Whether a canvas point (CSS px, origin top-left) lies on the cube's viewport. */
export function insideCube(px: number, py: number, width: number, height: number): boolean {
  const v = cubeViewport(width, height), top = height - v.y - v.size;
  return px >= v.x && px <= v.x + v.size && py >= top && py <= top + v.size;
}

export interface ViewCube {
  /** Aligns the cube camera with the main camera (also done by render). */
  orient(main: Camera): void;
  /** Draws the cube over the frame already rendered, in its corner. */
  render(renderer: WebGLRenderer, main: Camera, width: number, height: number): void;
  /** Face under a canvas point (CSS px, origin top-left) for the last orientation, or null. */
  pick(px: number, py: number, width: number, height: number): CubeFace | null;
  dispose(): void;
}

const HALF_VIEW = 1.2;

/** Text texture where a 2D canvas exists (browser); a plain face elsewhere (unit tests). */
function labelMaterial(text: string, color: string, maxSize: number, background: string | null): Material {
  const canvas = typeof document === 'undefined' ? null : document.createElement('canvas');
  const g = canvas?.getContext('2d');
  if (!canvas || !g) return new MeshBasicMaterial({ color: 0x1d2a35 });
  canvas.width = canvas.height = 128;
  if (background) {
    g.fillStyle = background; g.fillRect(0, 0, 128, 128);
    g.strokeStyle = '#5c7a8c'; g.lineWidth = 6; g.strokeRect(3, 3, 122, 122);
  }
  let size = maxSize;  // the largest font that fits the face
  do { g.font = `bold ${size}px system-ui, sans-serif`; } while (g.measureText(text).width > 112 && --size > 10);
  g.fillStyle = color; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 64, 66);
  const map = new CanvasTexture(canvas);
  map.colorSpace = SRGBColorSpace;
  return background ? new MeshBasicMaterial({ map }) : new SpriteMaterial({ map, depthTest: false });
}

export function createViewCube(colors: { x: number; y: number; z: number }): ViewCube {
  const scene = new Scene();
  const camera = new OrthographicCamera(-HALF_VIEW, HALF_VIEW, HALF_VIEW, -HALF_VIEW, 0.1, 10);
  const faces: Mesh[] = [];
  const cube = new Group();
  for (const spec of CUBE_FACES) {
    const n = new Vector3(...spec.normal), u = new Vector3(...spec.up), r = new Vector3().crossVectors(u, n);
    const mesh = new Mesh(new PlaneGeometry(1, 1), labelMaterial(spec.label, '#e6edf5', 40, '#1d2a35'));
    mesh.matrixAutoUpdate = false;
    mesh.matrix.makeBasis(r, u, n).setPosition(n.multiplyScalar(0.5));
    mesh.userData.face = spec.face;
    faces.push(mesh);
    cube.add(mesh);
  }
  const edges = new LineSegments(new EdgesGeometry(new BoxGeometry(1, 1, 1)), new LineBasicMaterial({ color: 0x9fb2c3 }));
  cube.add(edges);
  // XYZ triad from the back-right-bottom corner, in the scene's axis colours.
  const corner = new Vector3(-0.5, -0.5, -0.5);
  const triad = new Group();
  for (const [axis, dir, color] of [['X', new Vector3(1, 0, 0), colors.x], ['Y', new Vector3(0, 1, 0), colors.y], ['Z', new Vector3(0, 0, 1), colors.z]] as const) {
    triad.add(new ArrowHelper(dir, corner, 1.25, color, 0.2, 0.11));
    const material = labelMaterial(axis, `#${color.toString(16).padStart(6, '0')}`, 76, null);
    if (material instanceof SpriteMaterial) {
      const sprite = new Sprite(material);
      sprite.position.copy(corner).addScaledVector(dir, 1.42);
      sprite.scale.setScalar(0.3);
      triad.add(sprite);
    }
  }
  scene.add(cube, triad);
  const raycaster = new Raycaster(), pointer = new Vector2();

  const orient = (main: Camera): void => {
    // Same orientation as the main camera, looking at the cube from its side.
    camera.quaternion.copy(main.quaternion);
    camera.position.set(0, 0, 3).applyQuaternion(main.quaternion);
    camera.updateMatrixWorld();
    scene.updateMatrixWorld();  // face matrices are set once (matrixAutoUpdate off)
  };

  return {
    orient,
    render(renderer, main, width, height): void {
      if (width <= 0 || height <= 0) return;
      orient(main);
      const v = cubeViewport(width, height), autoClear = renderer.autoClear;
      renderer.autoClear = false;
      renderer.clearDepth();
      renderer.setScissorTest(true);
      renderer.setScissor(v.x, v.y, v.size, v.size);
      renderer.setViewport(v.x, v.y, v.size, v.size);
      renderer.render(scene, camera);
      renderer.setScissorTest(false);
      renderer.setViewport(0, 0, width, height);
      renderer.autoClear = autoClear;
    },
    pick(px, py, width, height): CubeFace | null {
      if (!insideCube(px, py, width, height)) return null;
      const v = cubeViewport(width, height), top = height - v.y - v.size;
      pointer.set(((px - v.x) / v.size) * 2 - 1, -((py - top) / v.size) * 2 + 1);
      raycaster.setFromCamera(pointer, camera);
      const hit = raycaster.intersectObjects(faces, false)[0];
      return hit ? (hit.object.userData.face as CubeFace) : null;
    },
    dispose(): void {
      scene.traverse((o) => {
        const m = o as Mesh;
        m.geometry?.dispose();
        const materials = Array.isArray(m.material) ? m.material : m.material ? [m.material] : [];
        for (const material of materials) { (material as MeshBasicMaterial).map?.dispose(); material.dispose(); }
      });
    },
  };
}
