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
  '../viewer/scene': 'export function createViewerScene(){const scene={resetCalls:[],resetCamera(yaw=0){this.resetCalls.push(yaw)},setBeforeRender(){},robotAnchor:{add(){},rotation:{z:0}},start(){}};globalThis.__viewers.push(scene);return scene}',
  '../urdf/parseUrdf': 'export function parseUrdf(){return{}}',
  '../urdf/buildRobot': 'export async function buildRobot(){return{rootObject:{quaternion:{w:1,x:0,y:0,z:0}},resetJoints(){},collisionMeshCount:17}}',
  '../viewer/linkStyle': 'export const styleForLink=()=>({})',
  '../urdf/mtwLoader': 'export const mtwMeshLoader=()=>()=>null;export const parseMtw=()=>({})',
  '../motion/twinController': 'export class MotionTwinController{constructor(actual){this.actual=actual;this.applied=[];this.actualState="MEASURED";this.interpolationMs=0;globalThis.__twins.push(this)}apply(f){this.applied.push(f);if(f.quaternion&&f.channels.imuValid!==false&&f.channels.imuAgeMs<=200)this.actual.rootObject.quaternion=f.quaternion}setStale(){}animate(){}}',
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
  const fetches = [], timers = [], listeners = [], parentPosts = [], observers = [];
  const parent = { dashboard: true, postMessage(data, origin){parentPosts.push({data,origin})} };
  let naturalHeight = 420;
  dom.window.document.getElementById('app').getBoundingClientRect = () => ({ height: naturalHeight });
  const win = { addEventListener: (type, fn) => { if (type === 'message') listeners.push(fn); } };
  win.parent = framed ? parent : win;
  globalThis.__twins = []; globalThis.__viewers = [];
  Object.assign(globalThis, {
    window: win, document: dom.window.document, location: dom.window.location,
    setTimeout: (fn, ms) => { timers.push({ fn, ms }); return timers.length; }, clearTimeout: () => {}, setInterval: () => 0,
    ResizeObserver: class { constructor(callback){observers.push(callback)} observe(){} },
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
  return { dom, fetches, timers, listeners, parent, parentPosts, twin, scene: globalThis.__viewers[0], statusReads: () => fetches.filter(f => f.url === '/api/motion/status').length,
    resize(height){naturalHeight=height;observers.forEach(callback=>callback())} };
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
  check(t.dom.window.document.querySelectorAll('header .chip').length === 1 && !t.dom.window.document.querySelector('header').textContent.includes('MATDOG Twin'), 'compact Twin header has one badge and no repeated title');
  check(t.dom.window.document.getElementById('dashboard-link').hidden && !t.dom.window.document.getElementById('panel').open, 'embedded STOP link is redundant; technical diagnostics stay expandable');
  check(!t.dom.window.document.getElementById('viewport').contains(t.dom.window.document.getElementById('banner')) && !t.dom.window.document.getElementById('viewport').contains(t.dom.window.document.getElementById('panel')), 'all diagnostics are outside the WebGL surface');
  const sizes = () => t.parentPosts.filter(p=>p.data.type==='MATDOG_TWIN_HEIGHT');
  check(sizes().length===1 && sizes()[0].data.height===420 && sizes()[0].origin==='http://192.168.4.1', 'natural height sent to the same-origin parent');
  const beforeSizeRequests=t.fetches.length;
  t.resize(900);t.resize(900);
  check(sizes().length===2 && sizes().at(-1).data.height===900 && t.fetches.length===beforeSizeRequests, 'details growth reports a new height once without requests');
  const reset = t.dom.window.document.getElementById('reset-view');
  check(reset && reset.textContent === 'Reset vista', 'read-only view reset is exposed');
  const beforeFirstReset = t.fetches.length;
  reset.click();
  check(t.scene.resetCalls.length === 1 && t.scene.resetCalls[0] === 0 && t.scene.robotAnchor.rotation.z === 0, 'reset before telemetry: canonical +X camera, no yaw reference');
  check(t.fetches.length === beforeFirstReset && t.twin.applied.length === 0, 'reset before telemetry neither fetches nor invents a posture');
  const status = JSON.parse(fx.status_walking);
  // Tilted, nonzero heading: Reset cancels only the yaw seen at the click with a
  // graphical reference on the robot anchor (ACTUAL and TARGET together); the
  // camera takes the canonical +X view; quaternion, roll and pitch untouched.
  status.telemetry.quaternion = [.94, .10, -.12, .29];
  const post = (data, origin = 'http://192.168.4.1', source = t.parent) => t.listeners[0]({ data, origin, source });
  post(status, 'http://192.168.4.2'); post(status, 'http://192.168.4.1', {}); post({ unrelated: true });
  check(t.twin.applied.length === 0, 'foreign origin, foreign source or a non-status message is ignored');
  post(status);
  check(t.twin.applied.length === 1, 'the dashboard status is rendered');
  const frame = t.twin.applied[0];
  const { JOINT_NAMES } = await import(pathToFileURL(path.join(viewer, 'src/motion/telemetry.ts')).href);
  JOINT_NAMES.forEach((joint, i) => check(frame.actual[joint] === status.telemetry.actual_rad[i], 'same ACTUAL angle ' + joint));
  check(JSON.stringify(frame.quaternion) !== 'null', 'BNO085 quaternion carried');
  const q=frame.quaternion, heading=Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  check(Math.abs(q.z)>.1 && t.scene.resetCalls.length === 2 && t.scene.resetCalls[1] === 0 && Math.abs(t.scene.robotAnchor.rotation.z+heading)<1e-12, 'first measured frame shows the robot front: anchor yaw -heading, canonical +X camera');
  const receivedQuaternion = JSON.stringify(status.telemetry.quaternion), frameQuaternion = JSON.stringify(frame.quaternion);
  const beforeReset = t.fetches.length, beforeFrames = t.twin.applied.length, beforeCameraResets = t.scene.resetCalls.length;
  reset.click();
  check(t.scene.resetCalls.length === beforeCameraResets + 1 && t.scene.resetCalls.at(-1) === 0 && Math.abs(t.scene.robotAnchor.rotation.z+heading)<1e-12, 'Reset vista: anchor yaw reference -heading and canonical +X camera');
  check(t.fetches.length === beforeReset && t.fetches.every(f => !f.init.method || f.init.method === 'GET'), 'Reset vista issues no extra telemetry GET or command POST');
  check(t.twin.applied.length === beforeFrames && JSON.stringify(frame.quaternion) === frameQuaternion && JSON.stringify(status.telemetry.quaternion) === receivedQuaternion, 'Reset vista preserves the received IMU quaternion and robot posture');
  check(t.dom.window.document.getElementById('banner').textContent !== 'In attesa della prima telemetria…', 'Twin view updated');
  const older = JSON.parse(fx.status_walking); older.now_ms -= 350; older.telemetry.sampled_at_ms -= 350; post(older);
  check(t.twin.applied.length === 1, 'an older snapshot never replaces a newer one');
  const newer = JSON.parse(fx.status_walking); newer.now_ms += 350; newer.telemetry.sampled_at_ms += 350; post(newer);
  check(t.twin.applied.length === 2, 'each newer dashboard snapshot is applied');
  // SIMPLE already reports IMU validity/age at top level. The dashboard
  // additionally binds the raw joint count into optional channels.*.
  const chip = id => t.dom.window.document.getElementById(id).textContent;
  const diagnostics = () => t.dom.window.document.getElementById('facts').textContent;
  check(chip('twin-status') === 'LIVE' && diagnostics().includes(`valida · ${newer.imu_age_ms} ms`) && diagnostics().includes('RAWN/D'), 'single LIVE badge; source IMU age and unknown RAW remain in diagnostics');
  const bound = JSON.parse(fx.status_walking); bound.now_ms += 700; bound.telemetry.sampled_at_ms += 700;
  bound.telemetry.quaternion=status.telemetry.quaternion;
  bound.channels = { imu: { valid: bound.imu_valid, age_ms: bound.imu_age_ms }, raw: { count: bound.joints.filter(j => j.valid).length } }; post(bound);
  check(chip('twin-status') === 'LIVE' && diagnostics().includes(`valida · ${bound.imu_age_ms} ms`) && diagnostics().includes('RAW12/12'), 'bound status: IMU age and RAW 12/12 kept in expandable diagnostics');
  const displayedQuaternion=JSON.stringify(t.twin.actual.rootObject.quaternion);
  const invalid = JSON.parse(JSON.stringify(bound)); invalid.now_ms += 350; invalid.telemetry.sampled_at_ms += 350; invalid.telemetry.quaternion = null; invalid.channels.imu.valid = false; post(invalid);
  check(chip('twin-status') !== 'LIVE' && diagnostics().includes('NON VALIDA'), 'Controller imu_valid=false never shows LIVE');
  reset.click();
  check(JSON.stringify(t.twin.actual.rootObject.quaternion)===displayedQuaternion && t.scene.resetCalls.at(-1)===0&&Math.abs(t.scene.robotAnchor.rotation.z+heading)<1e-12, 'Reset follows retained displayed orientation after invalid IMU, not the rejected stream sample');
  const demo = JSON.parse(JSON.stringify(bound)); demo.now_ms += 1000; demo.telemetry.sampled_at_ms += 1000; demo.telemetry.source = 'OFFLINE_UI_FIXTURE'; post(demo);
  check(chip('twin-status') === 'OFFLINE', 'an offline fixture with full measured channels is explicitly OFFLINE');
  // Yaw 0/+90/-90/180 deg with roll/pitch: each Reset cancels exactly the yaw at the click.
  let stamp = 2000;
  for (const yawDeg of [0, 90, -90, 180]) {
    const y = yawDeg * Math.PI / 180, r = .2, pt = -.15, cy = Math.cos(y/2), sy = Math.sin(y/2), cp = Math.cos(pt/2), sp = Math.sin(pt/2), cr = Math.cos(r/2), sr = Math.sin(r/2);
    const s = JSON.parse(fx.status_walking); stamp += 700; s.now_ms += stamp; s.telemetry.sampled_at_ms += stamp;
    s.telemetry.quaternion = [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy];
    post(s); const g = t.twin.applied.at(-1).quaternion, h = Math.atan2(2*(g.w*g.z+g.x*g.y),1-2*(g.y*g.y+g.z*g.z));
    reset.click();
    check(t.scene.resetCalls.at(-1) === 0 && Math.abs(Math.atan2(Math.sin(t.scene.robotAnchor.rotation.z+h),Math.cos(t.scene.robotAnchor.rotation.z+h)))<1e-9 && Math.abs(Math.abs(h)-Math.abs(y))<1e-6, 'Reset vista at yaw ' + yawDeg + ' deg');
  }
  post({ unrelated: true });
  check(chip('twin-status') === 'ERRORE' || chip('twin-status') === 'OFFLINE · ERRORE', 'rejected telemetry never leaves a falsely positive badge');
  for (const timer of t.timers) timer.fn();
  check(t.statusReads() === 0, 'still no status request after every timer fired');
  console.log(`embedded: 0 status requests, ${t.twin.applied.length} frames from the dashboard poll`);
}
// Same URL opened top-level (not in a frame): falls back to its own poll.
{
  const t = await boot('?feed=parent', false);
  check(t.listeners.length === 0 && t.statusReads() === 1, 'top-level ?feed=parent polls by itself');
  check(!t.dom.window.document.getElementById('dashboard-link').hidden, 'standalone Twin keeps access to dashboard STOP');
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
