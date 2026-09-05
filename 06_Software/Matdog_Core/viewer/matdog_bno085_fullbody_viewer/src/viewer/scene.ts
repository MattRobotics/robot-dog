/**
 * Three.js scene for the MATDOG full-body viewer.
 *
 * The world is configured Z-UP so that world axes coincide with MATDOG
 * base_link axes:  +X forward, +Y left, +Z up.  With that choice the IMU
 * quaternion needs no conversion beyond (w,x,y,z) -> (x,y,z,w).
 */

import {
  AmbientLight,
  ArrowHelper,
  AxesHelper,
  Color,
  DirectionalLight,
  GridHelper,
  HemisphereLight,
  Object3D,
  PerspectiveCamera,
  Scene,
  Vector3,
  WebGLRenderer,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export interface ViewerScene {
  readonly scene: Scene;
  readonly camera: PerspectiveCamera;
  readonly renderer: WebGLRenderer;
  readonly controls: OrbitControls;
  /** Parent for the robot; keeps world helpers separate from robot content. */
  readonly robotAnchor: Object3D;
  start(): void;
  dispose(): void;
  resetCamera(): void;
  /** Renders a single frame, re-syncing the viewport to the container first. */
  renderFrame(): void;
}

const BACKGROUND_COLOR = 0x111418;

/**
 * Conventional axis colours: X red, Y green, Z blue. Used by the world axes,
 * by the forward/left arrows and by the panel legend, so the three always
 * agree. These stay colour-coded even though the robot itself is neutral.
 */
export const AXIS_COLORS = {
  x: 0xff5b5b,
  y: 0x5bff8f,
  z: 0x6ba6ff,
} as const;

export const AXIS_LEGEND: readonly { label: string; color: number }[] = [
  { label: '+X forward', color: AXIS_COLORS.x },
  { label: '+Y left', color: AXIS_COLORS.y },
  { label: '+Z up', color: AXIS_COLORS.z },
];

export function createViewerScene(container: HTMLElement): ViewerScene {
  // World is Z-up: this must be set before any camera/controls maths.
  Object3D.DEFAULT_UP.set(0, 0, 1);

  const scene = new Scene();
  scene.background = new Color(BACKGROUND_COLOR);

  const camera = new PerspectiveCamera(45, 1, 0.01, 100);
  camera.up.set(0, 0, 1);

  const renderer = new WebGLRenderer({ antialias: true });
  // Display size is owned by CSS (#canvas-host canvas { width/height: 100% }).
  // Every setSize call therefore passes updateStyle = false: writing an inline
  // pixel size here would override the stylesheet and freeze the canvas at its
  // first measured size.
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0, 0.05);

  scene.add(new AmbientLight(0xffffff, 0.55));
  scene.add(new HemisphereLight(0xbfd4e6, 0x20242a, 0.8));

  const keyLight = new DirectionalLight(0xffffff, 1.6);
  keyLight.position.set(0.6, 0.9, 1.2);
  scene.add(keyLight);

  const fillLight = new DirectionalLight(0xffffff, 0.5);
  fillLight.position.set(-0.8, -0.6, 0.4);
  scene.add(fillLight);

  // Grid: GridHelper is authored in the XZ plane, rotate it into MATDOG XY.
  const grid = new GridHelper(1.6, 32, 0x3b4653, 0x232a32);
  grid.rotateX(Math.PI / 2);
  scene.add(grid);

  // World axes at the origin: X red = forward, Y green = left, Z blue = up.
  const axes = new AxesHelper(0.22);
  axes.setColors(AXIS_COLORS.x, AXIS_COLORS.y, AXIS_COLORS.z);
  scene.add(axes);

  // Explicit forward / left cues so front-back and left-right are unmistakable.
  const forwardArrow = new ArrowHelper(
    new Vector3(1, 0, 0),
    new Vector3(0, 0, 0),
    0.34,
    AXIS_COLORS.x,
    0.06,
    0.035,
  );
  const leftArrow = new ArrowHelper(
    new Vector3(0, 1, 0),
    new Vector3(0, 0, 0),
    0.28,
    AXIS_COLORS.y,
    0.05,
    0.03,
  );
  scene.add(forwardArrow, leftArrow);

  const robotAnchor = new Object3D();
  robotAnchor.name = 'robot_anchor';
  scene.add(robotAnchor);

  const resetCamera = (): void => {
    // Behind-left-above: the nose (+X) points away to the right of frame.
    camera.position.set(-0.42, -0.46, 0.34);
    controls.target.set(0, 0, 0.05);
    controls.update();
  };
  resetCamera();

  /**
   * Single authority for viewport size. Tracks the real container box, so
   * maximising or resizing the window updates the drawing buffer and the
   * camera projection immediately, with no reload and no distortion.
   */
  let lastWidth = 0;
  let lastHeight = 0;
  let lastPixelRatio = 0;

  const resize = (): void => {
    const width = container.clientWidth;
    const height = container.clientHeight;
    // The container can legitimately measure 0 before first layout or while
    // hidden; skip until it has a real box rather than baking in a bad size.
    if (width === 0 || height === 0) return;

    const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
    if (width === lastWidth && height === lastHeight && pixelRatio === lastPixelRatio) return;
    lastWidth = width;
    lastHeight = height;
    lastPixelRatio = pixelRatio;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setPixelRatio(pixelRatio);
    renderer.setSize(width, height, false);
  };

  // ResizeObserver on the actual viewer container catches every cause of a
  // size change (window resize, panel reflow, layout changes), not just
  // window events. The window listener additionally covers moving the window
  // to a display with a different devicePixelRatio.
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(container);
  window.addEventListener('resize', resize);
  resize();

  let animationHandle = 0;
  let running = false;

  /**
   * Renders one frame, re-checking the container box first. `resize()` is a
   * no-op unless the box actually changed, so this costs two clean layout
   * reads per frame and guarantees the viewport tracks the container even if
   * a ResizeObserver or window resize notification is missed or arrives
   * before reflow has settled.
   */
  const renderFrame = (): void => {
    resize();
    controls.update();
    renderer.render(scene, camera);
  };

  const tick = (): void => {
    animationHandle = requestAnimationFrame(tick);
    renderFrame();
  };

  return {
    scene,
    camera,
    renderer,
    controls,
    robotAnchor,
    resetCamera,
    renderFrame,
    start(): void {
      if (running) return;
      running = true;
      tick();
    },
    dispose(): void {
      running = false;
      cancelAnimationFrame(animationHandle);
      resizeObserver.disconnect();
      window.removeEventListener('resize', resize);
      controls.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    },
  };
}
