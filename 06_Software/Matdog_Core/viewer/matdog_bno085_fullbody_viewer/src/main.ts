/**
 * MATDOG host Digital Twin: read-only Controller motion telemetry and BNO085
 * orientation compose on the canonical articulated collision scene graph.
 * Actual/target joint radians are independent channels; physical contact and
 * support-plane height are never inferred from the IMU or encoder geometry.
 */

import './style.css';

import { Vector3, type BufferGeometry } from 'three';
import { MotionTelemetryStream, JOINT_NAMES } from './motion/telemetry';
import { parseOfflineTrace } from './motion/offlineTrace';
import { MotionTwinController } from './motion/twinController';
import { syntheticMotionStatus } from './motion/syntheticMotion';
import {
  classifySource,
  diagnosticCodes,
  LiveFeedState,
  type DiagnosticCode,
  type LiveSnapshot,
  type SourceClassification,
  type TransportError,
} from './motion/liveStatus';
import { detectBridge, LiveTransport, postStop, type StopResult, type TransportEvents, type TransportKind } from './motion/bridgeClient';
import { jointRows, parseDashboardModel, type DashboardModel, type JointLimit } from './dashboard/dashboardModel';
import { renderDashboard, renderLiveBar, renderStopResult, setUpJointTable } from './dashboard/dashboardView';

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
let twin: MotionTwinController | null = null;
const motionStream = new MotionTelemetryStream();
let motionMode: 'off' | 'synthetic' | 'live' | 'trace' = 'off';
let motionStartMs = 0;
let motionTimer = 0;
let traceRecords: unknown[] = [];
let traceIndex = 0;
let traceOriginMs = 0;

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
  if (accepted && robot) {
    robot.setRootOrientation(orientation.viewerThreeQuaternion());
    twin?.target.setRootOrientation(orientation.viewerThreeQuaternion());
  }
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
    if (state === 'CONNECTED') stopMotionFeed();
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
  twin?.target.setRootOrientation(orientation.viewerThreeQuaternion());
  renderOrientation();
});
btnClearReference.addEventListener('click', () => {
  orientation.clearReference();
  if (robot) robot.setRootOrientation(orientation.viewerThreeQuaternion());
  twin?.target.setRootOrientation(orientation.viewerThreeQuaternion());
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

/* ---------------------------------------------------------- motion twin --- */
const urlParams = new URLSearchParams(window.location.search);
const pollHzParam = Number(urlParams.get('poll_hz'));
const liveFeed = new LiveFeedState();
let liveTransport: LiveTransport | null = null;
let transportKind: TransportKind = 'idle';
let transportError: TransportError = 'NONE';
let bridgeHealth: Record<string, unknown> | null = null;
let lastLiveAvailable: boolean | null = null;
let dashboardModel: DashboardModel | null = null;
let assetsMissing = false;
let jointLimits: Record<string, JointLimit> = {};
let lastSource: SourceClassification = { badge: 'OFF', label: 'OFF' };
let lastCodes: DiagnosticCode[] = [];
let lastStop: StopResult | null = null;
const btnStop = element<HTMLButtonElement>('btn-stop');
const dashboardBlock = element('dashboard-block');

function liveSnapshot(nowMs: number): LiveSnapshot {
  const frame = motionStream.frame;
  const rows = jointRows(dashboardModel, frame, jointLimits);
  return {
    mode: motionMode, hasFrame: frame !== null, stale: motionMode === 'off' || motionStream.isStale(nowMs),
    bridge: dashboardModel?.bridge ?? null, telemetrySource: dashboardModel?.source ?? null,
    transportError, assetsMissing, available: motionMode === 'live' ? lastLiveAvailable : null,
    restore: dashboardModel?.readiness?.restore ?? null, permit: dashboardModel?.readiness?.permit ?? null,
    reason: frame?.reason ?? '', actualMissing: frame !== null && frame.actual === null,
    anyUnmapped: dashboardModel?.servos.some((servo) => servo?.mapped === false) ?? false,
    commissioningRestore: dashboardModel?.commissioning?.restore ?? null,
    jointLimitViolation: rows.some((row) => row.outsideLimit),
  };
}
function renderLive(nowMs: number): void {
  const snapshot = liveSnapshot(nowMs);
  lastSource = classifySource(snapshot);
  lastCodes = diagnosticCodes(snapshot);
  renderLiveBar(lastSource, lastCodes);
  if (dashboardBlock.hidden) return;
  renderDashboard({ model: dashboardModel, frame: motionStream.frame, limits: jointLimits,
    transport: motionMode === 'live' ? transportKind : motionMode, sampleAgeMs: motionStream.ageMs(nowMs), codes: lastCodes });
}
function renderMotion(): void {
  const now = performance.now();
  const frame = motionStream.frame, age = motionStream.ageMs(now);
  const stale = motionMode === 'off' || motionStream.isStale(now);
  element('twin-fresh').textContent = frame ? stale ? 'STALE' : 'FRESH' : 'NO DATA';
  element('twin-fresh').className = 'badge ' + (stale ? 'state-error' : 'state-connected');
  twin?.setStale(stale);
  renderLive(now);
  if (!frame) return;
  if (frame.quaternion === null) {
    element('q-raw').textContent = 'UNKNOWN (no current Controller quaternion)';
    element('q-viewer').textContent = 'UNKNOWN'; element('rpy-viewer').textContent = 'UNKNOWN';
  }
  element('twin-executor').textContent = `${frame.executorState} / ${frame.standState} / ${frame.owner}`;
  element('twin-time').textContent = `${frame.controllerNowMs.toFixed(0)} / ${frame.sampledAtMs.toFixed(0)} ms / ${age?.toFixed(0)} ms`;
  element('twin-height').textContent = frame.baseHeightM === null ? 'UNKNOWN (support plane unavailable)' : `${(frame.baseHeightM * 1000).toFixed(1)} mm above reported support plane`;
  const error = frame.trackingErrorRad === null ? 'UNKNOWN' : `${frame.trackingErrorRad.toFixed(4)} rad`;
  element('twin-fault').textContent = `${frame.fault} / tracking ${error} · ${frame.reason}`;
  element('twin-feet').textContent = frame.feet.map((f) => `${f.leg} ${f.contact} phase ${f.phase?.toFixed(2) ?? '—'}`).join(' · ');
  // Per joint: measured angle (null = not measured, never 0) / target / raw tick provenance.
  element('twin-joints').textContent = JOINT_NAMES.map((name, i) => `${name.padEnd(22)} ${(frame.actualByJoint ?? frame.actual)?.[name]?.toFixed(4) ?? 'UNMEASURED'} / ${frame.target?.[name]?.toFixed(4) ?? 'NONE'} · raw ${frame.rawTicks?.[i] ?? '—'}`).join('\n');
  element('twin-diagnostic').textContent = twin?.diagnostic || '';
}
function applyMotionStatus(value: unknown, live = false): boolean {
  const now = performance.now();
  const accepted = live ? liveFeed.accept(motionStream, value, now) : motionStream.accept(value, now);
  if (!accepted) {
    element('twin-diagnostic').textContent = 'Rejected telemetry: malformed, unsupported version, or timestamp regression.';
    return false;
  }
  if (!live) dashboardModel = parseDashboardModel(value);
  const frame = motionStream.frame!;
  // Interpolation is visual only (live feed); numbers always show the received sample.
  twin?.apply(frame, live ? now : undefined);
  if (frame.quaternion) {
    orientation.update(frame.quaternion);
    // Recenter belongs to the technical viewer only; it also rotates the target ghost.
    robot?.setRootOrientation(orientation.viewerThreeQuaternion());
    twin?.target.setRootOrientation(orientation.viewerThreeQuaternion());
    renderOrientation();
  }
  renderMotion(); return true;
}
/** One bridge/Controller status object (SSE event or poll response). */
function handleLivePayload(value: unknown): void {
  if (motionMode !== 'live') return;
  transportError = 'NONE';
  dashboardModel = parseDashboardModel(value);
  const status = value as Record<string, unknown> | null;
  lastLiveAvailable = status?.available !== false;
  if (!lastLiveAvailable) { renderMotion(); return; }
  applyMotionStatus(value, true);
}
const transportEvents: TransportEvents = {
  onPayload: (value) => handleLivePayload(value),
  onError: (kind, detail) => {
    transportError = kind;
    element('twin-diagnostic').textContent = `Live feed: ${kind} (${detail})`;
    renderMotion();
  },
  onTransport: (kind) => { transportKind = kind; renderMotion(); },
};
function stopMotionFeed(): void {
  motionMode = 'off'; window.clearTimeout(motionTimer);
  liveTransport?.stop(); liveTransport = null; transportKind = 'idle';
  btnRecenter.disabled = false; btnClearReference.disabled = false;
  btnStop.hidden = true;
  renderMotion();
  element('twin-source').textContent = 'Feed paused; samples will become STALE. No command was sent.';
}
async function sampleMotion(): Promise<void> {
  window.clearTimeout(motionTimer);
  if (document.hidden || motionMode === 'off' || motionMode === 'live') return;
  if (motionMode === 'synthetic') applyMotionStatus(syntheticMotionStatus(performance.now() - motionStartMs));
  if (motionMode === 'trace') {
    const until = traceOriginMs + performance.now() - motionStartMs;
    while (traceIndex < traceRecords.length) {
      const entry = traceRecords[traceIndex] as { now_ms?: number };
      if (typeof entry.now_ms !== 'number' || entry.now_ms > until) break;
      applyMotionStatus(entry); traceIndex += 1;
    }
    if (traceIndex === traceRecords.length) { stopMotionFeed(); element('twin-source').textContent = 'Offline trace completed. Final sample is retained with its timestamp.'; return; }
  }
  renderMotion();
  if ((motionMode as string) !== 'off' && !document.hidden) motionTimer = window.setTimeout(() => void sampleMotion(), 100);
}
function startLiveTransport(): void {
  liveTransport?.stop();
  const viaBridge = bridgeHealth !== null;
  const pollHz = Number.isFinite(pollHzParam) && pollHzParam > 0 ? pollHzParam : viaBridge ? 10 : 1;
  liveTransport = new LiveTransport(transportEvents, {
    useSse: viaBridge && urlParams.get('transport') !== 'poll', pollHz, timeoutMs: 800 });
  liveTransport.start();
}
function startMotionFeed(mode: 'synthetic' | 'live' | 'trace'): void {
  stopMotionFeed(); motionStream.reset(); liveFeed.reset(); motionMode = mode; motionStartMs = performance.now();
  dashboardModel = null; lastLiveAvailable = null; transportError = 'NONE';
  // Synthetic/trace data share a support-plane world frame with feet/trajectories,
  // so recenter is locked there. The live feed reports no feet/base position:
  // the visual-only recenter stays available (nothing is sent to the sensor).
  orientation.clearReference();
  btnRecenter.disabled = mode !== 'live'; btnClearReference.disabled = mode !== 'live';
  if (twin) twin.interpolationMs = mode === 'live' ? 100 : 0;
  dashboardBlock.hidden = false;
  btnStop.hidden = mode !== 'live';
  element('twin-source').textContent = mode === 'synthetic'
    ? 'SYNTHETIC DISPLAY FIXTURE: prone → acquire → rise 150 mm → WALK → STOP → intermediate restart. Illustrative values; physical and trajectory qualification are separate.'
    : mode === 'trace' ? 'OFFLINE TRACE REPLAY: source timestamps and joint targets are preserved. Synthetic evidence is not hardware qualification.'
      : bridgeHealth
        ? 'Host bridge feed (matdog_bridge.py): SSE stream, 10 Hz polling fallback. The bridge owns the serial port; this page only reads, plus the STOP de-escalation request.'
        : 'Controller HTTP read-only feed (1 Hz). No automatic serial access or hardware commands.';
  if (mode === 'live') { startLiveTransport(); renderMotion(); return; }
  void sampleMotion();
}
element('btn-motion-synthetic').addEventListener('click', () => startMotionFeed('synthetic'));
element('btn-motion-live').addEventListener('click', () => {
  void detectBridge().then((health) => { bridgeHealth = health; startMotionFeed('live'); });
});
element('btn-motion-pause').addEventListener('click', stopMotionFeed);
btnStop.addEventListener('click', () => {
  // De-escalation: single tap, no confirmation dialog.
  btnStop.disabled = true; btnStop.textContent = 'STOPPING…';
  void postStop().then((result) => {
    lastStop = result;
    renderStopResult(result);
    btnStop.classList.add('sent'); btnStop.textContent = result.ok ? 'STOP SENT' : 'STOP FAILED';
    window.setTimeout(() => { btnStop.disabled = false; btnStop.textContent = 'STOP'; btnStop.classList.remove('sent'); }, 1500);
  });
});
element('btn-panel-toggle').addEventListener('click', () => {
  const collapsed = document.body.classList.toggle('panel-collapsed');
  element('btn-panel-toggle').setAttribute('aria-expanded', String(!collapsed));
});
function loadMotionTrace(text: string): void {
  const records = parseOfflineTrace(text);
  traceOriginMs = records[0]!.now_ms as number;
  traceRecords = records; traceIndex = 0; startMotionFeed('trace');
}
element<HTMLInputElement>('motion-trace-file').addEventListener('change', async (event) => {
  const file = (event.target as HTMLInputElement).files?.[0]; if (!file) return;
  try {
    if (file.size > 10 * 1024 * 1024) throw new Error('trace exceeds 10 MiB');
    loadMotionTrace(await file.text());
  } catch (error) { element('twin-diagnostic').textContent = `Trace rejected: ${error instanceof Error ? error.message : String(error)}`; }
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { window.clearTimeout(motionTimer); liveTransport?.stop(); }
  else if (motionMode === 'live') liveTransport?.start();
  else void sampleMotion();
});
window.setInterval(() => { if (!document.hidden) { renderMotion(); renderStatus(); } }, 500);
setUpJointTable();

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
    // URDF model limits only (display flag); hard limits live on the Controller.
    jointLimits = Object.fromEntries(model.joints.map((joint) => [joint.name, { lower: joint.lower, upper: joint.upper }]));

    const fetchGeometry = createFetchStlLoader();
    const meshCache = new Map<string, Promise<BufferGeometry>>();
    const loadMeshGeometry = (url: string): Promise<BufferGeometry> => {
      let geometry = meshCache.get(url);
      if (!geometry) { geometry = fetchGeometry(url); meshCache.set(url, geometry); }
      return geometry;
    };
    robot = await buildRobot({ model, urdfUrl: CANONICAL_URDF_URL, loadMeshGeometry, styleForLink });
    const target = await buildRobot({ model, urdfUrl: CANONICAL_URDF_URL, loadMeshGeometry, styleForLink });
    twin = new MotionTwinController(robot, target);
    // Visual-only joint easing for the live feed; numbers stay the received sample.
    viewer.setBeforeRender((nowMs) => twin?.animate(nowMs));
    viewer.robotAnchor.add(target.rootObject);
    viewer.scene.add(twin.overlay);
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

    // The canonical model has exactly 17 collision meshes; fewer is a deployment fault.
    assetsMissing = robot.collisionMeshCount !== 17;
    setOverlay(
      `Loaded ${robot.collisionMeshCount} collision meshes · ${robot.jointNames.length} joints · ` +
        'neutral URDF pose. Connect the BNO085 to drive base_link.',
    );
    window.setTimeout(() => setOverlay(null), 6000);
  } catch (error) {
    assetsMissing = true; renderMotion();
    setOverlay(
      `Model load failed (ASSETS_MISSING): ${error instanceof Error ? error.message : String(error)}`,
      true,
    );
    throw error;
  }

  if (synthetic.enabled) {
    setUpSyntheticPanel();
    emitSyntheticAngles();
  }
  const motion = new URLSearchParams(window.location.search).get('motion');
  if (motion === 'synthetic') startMotionFeed('synthetic');
  if (motion === 'live' || motion === null) {
    // Served by the host bridge: LIVE is the default. Without a bridge the page
    // stays OFF unless ?motion=live explicitly asks for the 1 Hz Controller feed.
    bridgeHealth = await detectBridge();
    if (bridgeHealth !== null || motion === 'live') startMotionFeed('live');
  }
  if (motion === 'trace') {
    try {
      // Fixed same-origin staged test output; no arbitrary URL or hardware route.
      const response = await fetch('./motion/adaptive_motion_synthetic.jsonl', { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}; stage the offline trace first`);
      loadMotionTrace(await response.text());
    } catch (error) { element('twin-diagnostic').textContent = `Trace unavailable: ${error instanceof Error ? error.message : String(error)}`; }
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
      applyMotionStatus: (status: unknown) => boolean;
      startMotionSynthetic: () => void;
      stopMotionFeed: () => void;
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
  applyMotionStatus,
  startMotionSynthetic: () => startMotionFeed('synthetic'),
  stopMotionFeed,
  feedSerialText,
  recenter: () => btnRecenter.click(),
  clearReference: () => btnClearReference.click(),
  /**
   * Renders one frame synchronously and reports what WebGL drew. Used by the
   * browser smoke test, where requestAnimationFrame is throttled because the
   * pane is not compositing.
   */
  forceRender: () => {
    twin?.animate(performance.now());
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
    targets: twin?.target.jointSnapshot() ?? {},
    targetVisible: twin?.target.rootObject.visible ?? false,
    motion: motionStream.frame,
    motionStale: motionMode === 'off' || motionStream.isStale(performance.now()),
    source: lastSource, codes: lastCodes, stop: lastStop, transport: transportKind,
  }),
};

ready.catch((error) => console.error('[matdog-viewer] boot failed', error));
