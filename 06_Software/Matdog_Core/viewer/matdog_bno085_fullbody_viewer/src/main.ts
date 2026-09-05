/**
 * MATDOG BNO085 full-body viewer — application wiring.
 *
 * Data flow:
 *
 *   Web Serial (read-only)  ─┐
 *                            ├─> parseSensorLine -> RV record
 *   synthetic RV lines      ─┘        │
 *                                     v
 *                      OrientationController (raw / reference / viewer)
 *                                     │
 *                                     v
 *                  MatdogRobot.setRootOrientation(base_link)
 *                                     │
 *                          articulated URDF collision scene graph
 *
 * Joint values are a separate channel (`robot.setJointAngle`) that the IMU
 * path never touches; ST3215 telemetry is intentionally not wired up.
 */

import './style.css';

import { Vector3 } from 'three';

import { OrientationController } from './frame/orientationController';
import { RAD_TO_DEG, sh2ToRollPitchYaw, type Sh2Quaternion } from './frame/orientation';
import { parseSensorLine, type SensorRecord } from './serial/lineParser';
import { Bno085SerialReader, isWebSerialAvailable, type SerialConnectionState } from './serial/webSerial';
import {
  anglesToRvLine,
  readSyntheticConfig,
  SYNTHETIC_PRESETS,
  type SyntheticAngles,
} from './dev/syntheticSource';
import { buildRobot, type MatdogRobot } from './urdf/buildRobot';
import { collisionLinks, movableJointNames, parseUrdf, type UrdfModel } from './urdf/parseUrdf';
import { AXIS_LEGEND, createViewerScene, type ViewerScene } from './viewer/scene';
import { createFetchStlLoader } from './viewer/stlLoader';
import { styleForLink } from './viewer/linkStyle';

const CANONICAL_URDF_URL = `${import.meta.env.BASE_URL}canonical/matt_robodog_rev00/matt_robodog_rev00.urdf`;
const PROVENANCE_URL = `${import.meta.env.BASE_URL}canonical/matt_robodog_rev00/CANONICAL_PROVENANCE.json`;

function element<T extends HTMLElement>(id: string): T {
  const found = document.getElementById(id);
  if (!found) throw new Error(`missing DOM element: #${id}`);
  return found as T;
}

function formatQuaternion(q: Sh2Quaternion): string {
  return [q.w, q.x, q.y, q.z].map((value) => value.toFixed(6).padStart(9)).join(' ');
}

interface SensorStatus {
  rvAccuracyRad: number | null;
  rvStatus: number | null;
  rvCount: number | null;
  rvLastMs: number | null;
  rvRateHz: number | null;
  magMagnitude: number | null;
  magStatus: number | null;
  runtimeResets: number | null;
}

const status: SensorStatus = {
  rvAccuracyRad: null,
  rvStatus: null,
  rvCount: null,
  rvLastMs: null,
  rvRateHz: null,
  magMagnitude: null,
  magStatus: null,
  runtimeResets: null,
};

const orientation = new OrientationController();
let robot: MatdogRobot | null = null;
let viewerScene: ViewerScene | null = null;

/* ------------------------------------------------------------------ UI --- */

const overlay = element('overlay');
const serialStateBadge = element('serial-state');
const serialDetail = element('serial-detail');
const btnConnect = element<HTMLButtonElement>('btn-connect');
const btnDisconnect = element<HTMLButtonElement>('btn-disconnect');
const btnRecenter = element<HTMLButtonElement>('btn-recenter');
const btnClearReference = element<HTMLButtonElement>('btn-clear-reference');

function setOverlay(message: string | null, isError = false): void {
  overlay.classList.toggle('hidden', message === null);
  overlay.classList.toggle('error', isError);
  if (message !== null) overlay.textContent = message;
}

function renderSerialState(state: SerialConnectionState, detail?: string): void {
  serialStateBadge.textContent = state;
  serialStateBadge.className = `badge state-${state.toLowerCase()}`;
  serialDetail.textContent = detail ?? '';
  btnConnect.disabled = state === 'CONNECTING' || state === 'CONNECTED';
  btnDisconnect.disabled = state === 'DISCONNECTED' || state === 'ERROR';
}

function renderOrientation(): void {
  const snapshot = orientation.snapshot();
  element('q-viewer').textContent = formatQuaternion(snapshot.viewer);
  element('q-raw').textContent = formatQuaternion(snapshot.raw);
  const { roll, pitch, yaw } = sh2ToRollPitchYaw(snapshot.viewer);
  element('rpy-viewer').textContent =
    `${(roll * RAD_TO_DEG).toFixed(1)}° / ${(pitch * RAD_TO_DEG).toFixed(1)}° / ${(yaw * RAD_TO_DEG).toFixed(1)}°`;
  element('recenter-state').textContent = snapshot.isRecentered
    ? 'yes (viewer reference frame)'
    : 'no (raw sensor frame)';
}

function renderStatus(): void {
  element('rv-accuracy').textContent =
    status.rvAccuracyRad === null
      ? '—'
      : `${status.rvAccuracyRad.toFixed(6)} rad (${(status.rvAccuracyRad * RAD_TO_DEG).toFixed(2)}°)`;
  element('rv-status').textContent = status.rvStatus === null ? '—' : String(status.rvStatus);

  const ageMs = status.rvLastMs === null ? null : Math.round(performance.now() - status.rvLastMs);
  element('rv-flow').textContent =
    status.rvCount === null
      ? '—'
      : `${status.rvCount} / ${status.rvRateHz === null ? '—' : `${status.rvRateHz.toFixed(1)} Hz`} / ${ageMs} ms`;

  element('mag-readout').textContent =
    status.magMagnitude === null
      ? '—'
      : `${status.magMagnitude.toFixed(3)} µT / status ${status.magStatus ?? '—'}`;

  element('runtime-resets').textContent =
    status.runtimeResets === null ? '—' : String(status.runtimeResets);
}

/* -------------------------------------------------------------- records --- */

let lastRvTimestamp: number | null = null;

function applyRawQuaternion(q: Sh2Quaternion): boolean {
  const accepted = orientation.update(q);
  if (accepted && robot) robot.setRootOrientation(orientation.viewerThreeQuaternion());
  renderOrientation();
  return accepted;
}

function handleRecord(record: SensorRecord): void {
  switch (record.kind) {
    case 'rv': {
      const now = performance.now();
      if (lastRvTimestamp !== null) {
        const deltaMs = now - lastRvTimestamp;
        if (deltaMs > 0) {
          const instantaneous = 1000 / deltaMs;
          status.rvRateHz =
            status.rvRateHz === null ? instantaneous : status.rvRateHz * 0.7 + instantaneous * 0.3;
        }
      }
      lastRvTimestamp = now;
      status.rvLastMs = now;
      status.rvAccuracyRad = record.accuracyRad;
      status.rvStatus = record.status;
      status.rvCount = record.count;
      applyRawQuaternion({ w: record.w, x: record.x, y: record.y, z: record.z });
      break;
    }
    case 'mag':
      status.magMagnitude = record.magnitude;
      status.magStatus = record.status;
      break;
    case 'counts':
      if (record.runtimeResets !== null) status.runtimeResets = record.runtimeResets;
      break;
    case 'save_gate':
      if (record.runtimeResets !== null) status.runtimeResets = record.runtimeResets;
      break;
    case 'reset':
      if (record.runtimeResets !== null) status.runtimeResets = record.runtimeResets;
      break;
    case 'gyr':
      break;
  }
  renderStatus();
}

/** Feeds raw serial text through the exact production parser path. */
function feedSerialText(text: string): number {
  let applied = 0;
  for (const line of text.split(/[\r\n]+/)) {
    const record = parseSensorLine(line);
    if (record === null) continue;
    handleRecord(record);
    applied += 1;
  }
  return applied;
}

/* --------------------------------------------------------------- serial --- */

const serial = new Bno085SerialReader({
  onState: (state, detail) => {
    renderSerialState(state, detail);
    if (state === 'DISCONNECTED') {
      lastRvTimestamp = null;
      status.rvRateHz = null;
    }
  },
  onRecord: handleRecord,
});

btnConnect.addEventListener('click', () => void serial.connect());
btnDisconnect.addEventListener('click', () => void serial.disconnect());
btnRecenter.addEventListener('click', () => {
  orientation.recenter();
  if (robot) robot.setRootOrientation(orientation.viewerThreeQuaternion());
  renderOrientation();
});
btnClearReference.addEventListener('click', () => {
  orientation.clearReference();
  if (robot) robot.setRootOrientation(orientation.viewerThreeQuaternion());
  renderOrientation();
});

/* ------------------------------------------------------------ synthetic --- */

const synthetic = readSyntheticConfig(window.location.search);
const syntheticAngles: SyntheticAngles = { ...synthetic.angles };
let syntheticCount = 0;

function emitSyntheticAngles(): void {
  syntheticCount += 1;
  // Deliberately routed through the serial line parser so the synthetic path
  // and the hardware path are byte-identical downstream.
  feedSerialText(anglesToRvLine(syntheticAngles, syntheticCount));
}

function setSyntheticRpyDeg(rollDeg: number, pitchDeg: number, yawDeg: number): void {
  syntheticAngles.rollDeg = rollDeg;
  syntheticAngles.pitchDeg = pitchDeg;
  syntheticAngles.yawDeg = yawDeg;
  if (synthetic.enabled) {
    (element<HTMLInputElement>('sim-roll')).value = String(rollDeg);
    (element<HTMLInputElement>('sim-pitch')).value = String(pitchDeg);
    (element<HTMLInputElement>('sim-yaw')).value = String(yawDeg);
    element('sim-roll-out').textContent = `${rollDeg}°`;
    element('sim-pitch-out').textContent = `${pitchDeg}°`;
    element('sim-yaw-out').textContent = `${yawDeg}°`;
  }
  emitSyntheticAngles();
}

function setUpSyntheticPanel(): void {
  const block = element('sim-block');
  block.hidden = false;

  const bind = (inputId: string, outputId: string, key: keyof SyntheticAngles): void => {
    const input = element<HTMLInputElement>(inputId);
    const output = element(outputId);
    input.value = String(syntheticAngles[key]);
    output.textContent = `${syntheticAngles[key]}°`;
    input.addEventListener('input', () => {
      syntheticAngles[key] = Number(input.value);
      output.textContent = `${input.value}°`;
      emitSyntheticAngles();
    });
  };

  bind('sim-roll', 'sim-roll-out', 'rollDeg');
  bind('sim-pitch', 'sim-pitch-out', 'pitchDeg');
  bind('sim-yaw', 'sim-yaw-out', 'yawDeg');

  const presets = element('sim-presets');
  const expectation = element('sim-expectation');
  for (const preset of SYNTHETIC_PRESETS) {
    const button = document.createElement('button');
    button.type = 'button';
    button.textContent = preset.label;
    button.addEventListener('click', () => {
      setSyntheticRpyDeg(preset.angles.rollDeg, preset.angles.pitchDeg, preset.angles.yawDeg);
      expectation.textContent = `expected: ${preset.expectation}`;
    });
    presets.append(button);
  }
}

/* ----------------------------------------------------------------- boot --- */

function renderModelInfo(model: UrdfModel, sourceLabel: string): void {
  element('model-source').textContent = sourceLabel;
  element('model-counts').textContent =
    `${model.links.length} links / ${model.joints.length} joints ` +
    `(${movableJointNames(model).length} movable) / ${collisionLinks(model).length} collision meshes`;

  const legend = element('model-legend');
  legend.replaceChildren();
  for (const entry of AXIS_LEGEND) {
    const span = document.createElement('span');
    const swatch = document.createElement('i');
    swatch.className = 'swatch';
    swatch.style.background = `#${entry.color.toString(16).padStart(6, '0')}`;
    span.append(swatch, document.createTextNode(entry.label));
    legend.append(span);
  }
}

async function boot(): Promise<void> {
  renderSerialState('DISCONNECTED');
  renderOrientation();
  renderStatus();

  if (!isWebSerialAvailable()) {
    serialDetail.textContent =
      'Web Serial not available in this browser. Use Chrome or Edge over http://localhost.';
  }

  const viewer = createViewerScene(element('canvas-host'));
  viewerScene = viewer;
  viewer.start();

  try {
    const response = await fetch(CANONICAL_URDF_URL);
    if (!response.ok) {
      throw new Error(
        `cannot fetch staged canonical URDF (HTTP ${response.status}). ` +
          'Run "npm run sync:assets".',
      );
    }
    const model = parseUrdf(await response.text());

    robot = await buildRobot({
      model,
      urdfUrl: CANONICAL_URDF_URL,
      loadMeshGeometry: createFetchStlLoader(),
      styleForLink,
    });
    robot.resetJoints();
    viewer.robotAnchor.add(robot.rootObject);
    robot.setRootOrientation(orientation.viewerThreeQuaternion());

    let sourceLabel = '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf';
    try {
      const provenance = await (await fetch(PROVENANCE_URL)).json();
      sourceLabel = `${provenance.canonical_urdf.repo_relative_path}\nsha256 ${provenance.canonical_urdf.sha256}`;
    } catch {
      /* provenance is informational only */
    }
    renderModelInfo(model, sourceLabel);

    setOverlay(
      `Loaded ${robot.collisionMeshCount} collision meshes · ${robot.jointNames.length} joints · ` +
        'neutral URDF pose. Connect the BNO085 to drive base_link.',
    );
    window.setTimeout(() => setOverlay(null), 6000);
  } catch (error) {
    setOverlay(
      `Model load failed: ${error instanceof Error ? error.message : String(error)}`,
      true,
    );
    throw error;
  }

  if (synthetic.enabled) {
    setUpSyntheticPanel();
    emitSyntheticAngles();
  }
}

/* ------------------------------------------------- dev/test entry point --- */

declare global {
  interface Window {
    matdogViewer: {
      readonly ready: Promise<void>;
      readonly robot: () => MatdogRobot | null;
      readonly orientation: () => OrientationController;
      setSyntheticRpyDeg: (rollDeg: number, pitchDeg: number, yawDeg: number) => void;
      forceRender: () => { triangles: number; calls: number; frame: number; aspect: number };
      renderInfo: () => { triangles: number; calls: number; frame: number; aspect: number };
      linkWorldPosition: (linkName: string) => { x: number; y: number; z: number } | null;
      applyRawQuaternion: (q: Sh2Quaternion) => boolean;
      feedSerialText: (text: string) => number;
      recenter: () => void;
      clearReference: () => void;
      snapshot: () => Record<string, unknown>;
    };
  }
}

const ready = boot();

window.matdogViewer = {
  ready,
  robot: () => robot,
  orientation: () => orientation,
  setSyntheticRpyDeg,
  applyRawQuaternion,
  feedSerialText,
  recenter: () => btnRecenter.click(),
  clearReference: () => btnClearReference.click(),
  /**
   * Renders one frame synchronously and reports what WebGL drew. Used by the
   * browser smoke test, where requestAnimationFrame is throttled because the
   * pane is not compositing.
   */
  forceRender: () => {
    viewerScene?.renderFrame();
    const info = viewerScene?.renderer.info.render;
    return {
      triangles: info?.triangles ?? 0,
      calls: info?.calls ?? 0,
      frame: info?.frame ?? 0,
      aspect: viewerScene?.camera.aspect ?? 0,
    };
  },
  /** WebGL diagnostics, used by the browser smoke test. */
  renderInfo: () => {
    const info = viewerScene?.renderer.info.render;
    return {
      triangles: info?.triangles ?? 0,
      calls: info?.calls ?? 0,
      frame: info?.frame ?? 0,
      aspect: viewerScene?.camera.aspect ?? 0,
    };
  },
  /** World-space position of a link, for frame verification from the console. */
  linkWorldPosition: (linkName: string) => {
    const object = robot?.linkObject(linkName);
    if (!object) return null;
    robot!.rootObject.updateMatrixWorld(true);
    const position = object.getWorldPosition(new Vector3());
    return { x: position.x, y: position.y, z: position.z };
  },
  snapshot: () => ({
    serialState: serial.connectionState,
    orientation: orientation.snapshot(),
    status: { ...status },
    collisionMeshCount: robot?.collisionMeshCount ?? 0,
    linkCount: robot?.linkNames.length ?? 0,
    jointCount: robot?.jointNames.length ?? 0,
    joints: robot?.jointSnapshot() ?? {},
  }),
};

ready.catch((error) => console.error('[matdog-viewer] boot failed', error));
