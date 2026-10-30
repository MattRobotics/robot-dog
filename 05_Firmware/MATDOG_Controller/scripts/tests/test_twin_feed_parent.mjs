// Phone Twin feed modes, executed on the viewer's real src/mobile/mobileMain.ts
// (node type stripping) with real telemetry/mobileModel and only the WebGL
// scene, URDF/mesh builders and 3D controller stubbed. Embedded in the
// dashboard (?feed=parent inside a frame) the Twin renders the status the
// dashboard posts and never requests /api/motion/status; standalone (or the
// same URL opened top-level) it keeps its own 500 ms poll. Run via
// run_simple_dashboard_tests.sh with the Controller's real status fixtures.
import { register, createRequire } from 'node:module';
import { pathToFileURL, fileURLToPath } from 'node:url';
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert';
const require = createRequire(import.meta.url);
const { JSDOM } = require('jsdom');
const fx = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
const viewer = path.join(path.dirname(fileURLToPath(import.meta.url)), '../../../../06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer');
const STUBS = {
  '../viewer/scene': 'export function createViewerScene(){return{setBeforeRender(){},robotAnchor:{add(){}},start(){}}}',
  '../urdf/parseUrdf': 'export function parseUrdf(){return{}}',
  '../urdf/buildRobot': 'export async function buildRobot(){return{rootObject:{},resetJoints(){},collisionMeshCount:17}}',
  '../viewer/linkStyle': 'export const styleForLink=()=>({})',
  '../urdf/mtwLoader': 'export const mtwMeshLoader=()=>()=>null;export const parseMtw=()=>({})',
  '../motion/twinController': 'export class MotionTwinController{constructor(){this.applied=[];this.actualState="MEASURED";this.interpolationMs=0;globalThis.__twins.push(this)}apply(f){this.applied.push(f)}setStale(){}animate(){}}',
};
register('data:text/javascript,' + encodeURIComponent(`const STUBS=${JSON.stringify(STUBS)};
export async function resolve(spec, ctx, next) {
  const from = ctx.parentURL || '';
  if (from.includes('/src/mobile/mobileMain.ts') && spec in STUBS) return { url: 'data:text/javascript,' + encodeURIComponent(STUBS[spec]), shortCircuit: true };
  if (spec.startsWith('.') && from.includes('/matdog_bno085_fullbody_viewer/src/') && !spec.endsWith('.ts')) return next(spec + '.ts', ctx);
  return next(spec, ctx);
}`));
const settle = async () => { for (let i = 0; i < 50; i++) await new Promise(r => setImmediate(r)); };
const html = fs.readFileSync(path.join(viewer, 'mobile.html'), 'utf8');
let instance = 0;
async function boot(query, framed) {
  const dom = new JSDOM(html, { url: 'http://192.168.4.1/twin/' + query });
  const fetches = [], timers = [], listeners = [], parent = { dashboard: true, postMessage(){} };
  const win = { addEventListener: (type, fn) => { if (type === 'message') listeners.push(fn); } };
  win.parent = framed ? parent : win;
  globalThis.__twins = [];
  Object.assign(globalThis, {
    window: win, document: dom.window.document, location: dom.window.location,
    setTimeout: (fn, ms) => { timers.push({ fn, ms }); return timers.length; }, clearTimeout: () => {}, setInterval: () => 0,
    fetch: async (url, init = {}) => {
      fetches.push({ url, init });
      if (url === '/api/motion/status') return { ok: true, status: 200, text: async () => fx.status_walking };
      return { ok: true, status: 200, text: async () => '<robot name="matdog"/>', arrayBuffer: async () => new ArrayBuffer(4) };
    },
  });
  await import(pathToFileURL(path.join(viewer, 'src/mobile/mobileMain.ts')).href + '?instance=' + (++instance));
  await settle();
  const twin = globalThis.__twins[0];
  assert(twin, 'model built');
  return { dom, fetches, timers, listeners, parent, twin, statusReads: () => fetches.filter(f => f.url === '/api/motion/status').length };
}
let checks = 0;
const check = (c, m) => { checks++; assert(c, m); };

// Embedded: the dashboard's single poll feeds the Twin; the Twin never polls.
{
  const t = await boot('?feed=parent', true);
  check(t.statusReads() === 0, 'embedded Twin issues no status request');
  check(t.fetches.every(f => f.url === 'matt_robodog_rev00.urdf' || f.url === 'matdog_collision.mtw'), 'embedded Twin loads only its model');
  check(t.timers.every(f => f.ms !== 500), 'no poll timer armed');
  check(t.listeners.length === 1, 'one message listener');
  const status = JSON.parse(fx.status_walking);
  const post = (data, origin = 'http://192.168.4.1', source = t.parent) => t.listeners[0]({ data, origin, source });
  post(status, 'http://192.168.4.2'); post(status, 'http://192.168.4.1', {}); post({ unrelated: true });
  check(t.twin.applied.length === 0, 'foreign origin, foreign source or a non-status message is ignored');
  post(status);
  check(t.twin.applied.length === 1, 'the dashboard status is rendered');
  const frame = t.twin.applied[0];
  const { JOINT_NAMES } = await import(pathToFileURL(path.join(viewer, 'src/motion/telemetry.ts')).href);
  JOINT_NAMES.forEach((joint, i) => check(frame.actual[joint] === status.telemetry.actual_rad[i], 'same ACTUAL angle ' + joint));
  check(JSON.stringify(frame.quaternion) !== 'null', 'BNO085 quaternion carried');
  check(t.dom.window.document.getElementById('banner').textContent !== 'In attesa della prima telemetria…', 'Twin view updated');
  const older = JSON.parse(fx.status_walking); older.now_ms -= 350; older.telemetry.sampled_at_ms -= 350; post(older);
  check(t.twin.applied.length === 1, 'an older snapshot never replaces a newer one');
  const newer = JSON.parse(fx.status_walking); newer.now_ms += 350; newer.telemetry.sampled_at_ms += 350; post(newer);
  check(t.twin.applied.length === 2, 'each newer dashboard snapshot is applied');
  // Badges: the bare status has no channels (installed rc.4: 'IMU ? ms', 'RAW N/D');
  // the dashboard binds imu_valid/imu_age_ms/joints[].valid to them.
  const chip = id => t.dom.window.document.getElementById(id).textContent;
  check(chip('chip-imu') === 'IMU ? ms' && chip('chip-raw') === 'RAW N/D', 'bare status: IMU age and RAW count unknown');
  const bound = JSON.parse(fx.status_walking); bound.now_ms += 700; bound.telemetry.sampled_at_ms += 700;
  bound.channels = { imu: { valid: bound.imu_valid, age_ms: bound.imu_age_ms }, raw: { count: bound.joints.filter(j => j.valid).length } }; post(bound);
  check(chip('chip-imu') === `IMU ${bound.imu_age_ms} ms` && chip('chip-raw') === 'RAW 12/12', 'bound status: IMU age and RAW 12/12 shown');
  const invalid = JSON.parse(JSON.stringify(bound)); invalid.now_ms += 350; invalid.telemetry.sampled_at_ms += 350; invalid.telemetry.quaternion = null; invalid.channels.imu.valid = false; post(invalid);
  check(chip('chip-imu') === 'IMU NON VALIDA', 'Controller imu_valid=false is shown as invalid');
  for (const timer of t.timers) timer.fn();
  check(t.statusReads() === 0, 'still no status request after every timer fired');
  console.log(`embedded: 0 status requests, ${t.twin.applied.length} frames from the dashboard poll`);
}
// Same URL opened top-level (not in a frame): falls back to its own poll.
{
  const t = await boot('?feed=parent', false);
  check(t.listeners.length === 0 && t.statusReads() === 1, 'top-level ?feed=parent polls by itself');
}
// Standalone: own poll every 500 ms, without credentials (never the operator session).
{
  const t = await boot('', false);
  check(t.listeners.length === 0, 'standalone has no message listener');
  check(t.statusReads() === 1 && t.fetches.find(f => f.url === '/api/motion/status').init.credentials === 'omit', 'standalone polls without credentials');
  check(t.twin.applied.length === 1, 'standalone renders its own status');
  const next = t.timers.filter(f => f.ms === 500);
  check(next.length === 1, 'next poll armed at 500 ms');
  next[0].fn(); await settle();
  check(t.statusReads() === 2, 'standalone keeps polling');
  console.log('standalone: own poll every 500 ms');
}
console.log(`TWIN_FEED_PARENT: ${checks} checks PASS`);
