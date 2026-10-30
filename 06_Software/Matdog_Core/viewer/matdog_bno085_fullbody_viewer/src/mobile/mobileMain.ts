/**
 * MATDOG phone twin, served by the robot itself on its AP (no Asus, no USB,
 * no internet). Strictly read-only: it GETs /api/motion/status and static
 * assets; it has no command, STOP or serial path (STOP lives in the portal).
 * WebGL runs on the phone; the ESP32 only serves bytes.
 */
import { createViewerScene } from '../viewer/scene';
import { parseUrdf } from '../urdf/parseUrdf';
import { buildRobot } from '../urdf/buildRobot';
import { styleForLink } from '../viewer/linkStyle';
import { mtwMeshLoader, parseMtw } from '../urdf/mtwLoader';
import { MotionTwinController } from '../motion/twinController';
import { JOINT_NAMES, MotionTelemetryStream } from '../motion/telemetry';
import { describeTwin, type TwinFacts } from './mobileModel';

const URDF_URL = 'matt_robodog_rev00.urdf', MESH_URL = 'matdog_collision.mtw', STATUS_URL = '/api/motion/status';
const POLL_MS = 500, TIMEOUT_MS = 2000;
const el = <T extends HTMLElement = HTMLElement>(id: string): T => document.getElementById(id) as T;
const stream = new MotionTelemetryStream();
let twin: MotionTwinController | null = null, lastError = '', modelError = '', failures = 0;
const parentFeed = new URLSearchParams(location.search).get('feed') === 'parent' && window.parent !== window;

function chip(id: string, text: string, tone: string): void { const c = el(id); c.textContent = text; c.className = `chip ${tone}`; }
function render(): void {
  const now = performance.now(), frame = stream.frame;
  const facts: TwinFacts = describeTwin(frame, stream.ageMs(now), stream.isStale(now), twin?.actualState ?? null, modelError || lastError, failures);
  for (const c of facts.chips) chip(c.id, c.text, c.tone);
  el('banner').textContent = facts.banner;
  const dl = el('facts'); dl.replaceChildren();
  for (const [k, v] of facts.rows) { const dt = document.createElement('dt'), dd = document.createElement('dd'); dt.textContent = k; dd.textContent = v; dl.append(dt, dd); }
  el('joints').textContent = facts.jointLines.join('\n');
  if (twin && stream.isStale(now)) twin.setStale(true);
}
function accept(body: unknown): void {
  if (stream.accept(body, performance.now()) && stream.frame) twin?.apply(stream.frame, performance.now());
  lastError = ''; failures = 0;
}
async function poll(): Promise<void> {
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), TIMEOUT_MS);
  try {
    // credentials: 'omit' — a read-only viewer must never act as the operator
    // heartbeat that keeps a Motion session alive (it has no STOP).
    const response = await fetch(STATUS_URL, { cache: 'no-store', credentials: 'omit', signal: abort.signal });
    const text = await response.text();
    let body: unknown = null; try { body = JSON.parse(text); } catch { /* reported below */ }
    if (!response.ok) throw new Error(`HTTP_${response.status} ${(body as { error?: string } | null)?.error ?? text.slice(0, 60)}`);
    if (body === null) throw new Error('HTTP_INVALID_JSON');
    accept(body);
  } catch (error) {
    failures += 1; lastError = abort.signal.aborted ? 'HTTP_TIMEOUT' : error instanceof Error ? error.message : String(error);
  } finally { clearTimeout(timer); render(); setTimeout(() => void poll(), POLL_MS); }
}
const MODEL_RETRY_MS = 5000;
async function loadModel(viewer: ReturnType<typeof createViewerScene>): Promise<void> {
  try {
    // The robot defers large downloads while Motion is active (503 + Retry-After).
    const [urdfText, pack] = await Promise.all([
      fetch(URDF_URL, { credentials: 'omit' }).then((r) => { if (!r.ok) throw new Error(`URDF HTTP ${r.status}`); return r.text(); }),
      fetch(MESH_URL, { credentials: 'omit' }).then((r) => { if (!r.ok) throw new Error(`MESH HTTP ${r.status}`); return r.arrayBuffer(); }),
    ]);
    const model = parseUrdf(urdfText);
    const loadMeshGeometry = mtwMeshLoader(model, URDF_URL, parseMtw(pack));
    const actual = await buildRobot({ model, urdfUrl: URDF_URL, loadMeshGeometry, styleForLink });
    const target = await buildRobot({ model, urdfUrl: URDF_URL, loadMeshGeometry, styleForLink });
    twin = new MotionTwinController(actual, target);
    twin.interpolationMs = 0; // display the received ACTUAL pose, no extra 500 ms lag
    viewer.setBeforeRender((nowMs) => twin?.animate(nowMs));
    viewer.robotAnchor.add(target.rootObject); viewer.robotAnchor.add(actual.rootObject);
    actual.resetJoints();
    viewer.start();
    modelError='';
    if(stream.frame){twin.apply(stream.frame,performance.now());twin.setStale(stream.isStale(performance.now()));}
    if(parentFeed)window.parent.postMessage({type:'MATDOG_TWIN_READY'},location.origin);
    if (actual.collisionMeshCount !== 17) lastError = `ASSETS_MISSING ${actual.collisionMeshCount}/17`;
  } catch (error) {
    modelError = `MODEL_LOAD_FAILED ${error instanceof Error ? error.message : String(error)} (retry ${MODEL_RETRY_MS / 1000} s)`;
    setTimeout(() => void loadModel(viewer), MODEL_RETRY_MS);
  }
  render();
}
async function boot(): Promise<void> {
  const viewer = createViewerScene(el('viewport'));
  void loadModel(viewer);
  // Embedded in the Controller dashboard (?feed=parent): reuse the status the
  // dashboard already polls (same-origin postMessage), so one page = one poll.
  if (parentFeed) {
    window.addEventListener('message', (e) => {
      if (e.origin !== location.origin || e.source !== window.parent) return;
      accept(e.data); render();
    });
  } else void poll();
  setInterval(render, 1000);
}
void JOINT_NAMES;
void boot();
