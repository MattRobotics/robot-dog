// Executable DOM check of the SHIPPED SIMPLE dashboard (PortalPage.h +
// MotionDashboard.h). Controller response bodies are the real MotionService
// output written by `test_simple_dashboard_contract fixtures`; refusals use the
// real sendJsonError format string from HttpTransport.cpp. Every command body
// the page passes to fetch is written verbatim to ENVELOPES for the
// `envelopes` phase. No device, no network. Run via run_simple_dashboard_tests.sh.
const { JSDOM } = require('jsdom');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const [fixturesPath, envelopesPath] = process.argv.slice(2);
assert(fixturesPath && envelopesPath, 'usage: test_simple_dashboard_dom.cjs FIXTURES.json ENVELOPES.tsv');
const src = path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const http = fs.readFileSync(path.join(src, 'network/HttpTransport.cpp'), 'utf8');
const errorFormat = http.split('esp_err_t HttpTransport::sendJsonError')[1].match(/snprintf\(body,sizeof\(body\),"((?:[^"\\]|\\.)*)"/)[1].replace(/\\"/g, '"');
const jsonError = (code, ms = 0) => errorFormat.replace('%s', code).replace('%lu', String(ms));
assert.deepStrictEqual(JSON.parse(jsonError('X', 3)), { version: 1, ok: false, error: 'X', elapsed_ms: 3 });
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
const CSRF = 'c'.repeat(64);
const settle = async () => { for (let i = 0; i < 30; i++) await new Promise(r => setImmediate(r)); };

async function boot() {
  let clock = 1000;
  const timers = [], intervals = [], calls = [], errors = [];
  // inflight: requests issued and not yet answered (each needs its own
  // HTTP/1.1 connection); holdUrl keeps one read pending until release().
  const device = { status: fx.status_ready, replies: [], inflight: 0, reads: 0, holdUrl: null, release: null, sessionOk: true };
  const dom = new JSDOM(html, { url: 'http://192.168.4.1/', runScripts: 'dangerously', pretendToBeVisual: true,
    beforeParse(w) {
      w.localStorage.setItem('md_login', '1');  // the page resumes its session at start
      Object.defineProperty(w.performance, 'now', { value: () => clock });
      w.Date.now = () => clock;  // session timers use the same fixture clock
      w.setTimeout = (fn, ms) => { timers.push({ fn, ms, active: true }); return timers.length; };
      w.clearTimeout = id => { if (timers[id - 1]) timers[id - 1].active = false; };
      w.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
      w.addEventListener('error', e => errors.push(e.message));
      w.fetch = async (url, init = {}) => {
        const read = !init.method || init.method === 'GET';
        calls.push({ url, method: init.method || 'GET', headers: init.headers || {}, body: init.body, inflight: ++device.inflight, reads: read ? ++device.reads : device.reads, timersAt: timers.length });
        try { return await serve(url, init); } finally { device.inflight--; if (read) device.reads--; }
      };
      const serve = async (url, init) => {
        const reply = (text, status = 200) => ({ ok: status >= 200 && status < 300, status, text: async () => text, json: async () => JSON.parse(text) });
        if (url === device.holdUrl) await new Promise((resolve, reject) => {
          device.release = () => { device.holdUrl = null; resolve(); };
          init.signal.addEventListener('abort', () => { device.holdUrl = null; reject(new w.DOMException('aborted', 'AbortError')); }, { once: true });
        });
        if (url === '/wifi/session') return device.sessionOk ? reply(CSRF) : reply('AUTH_FAILED', 403);
        if (url === '/status') return reply('mode=MAINTENANCE\nauthority=NONE\npower_state=RUN\nsystem_health=READY\n');
        if (url === '/wifi/status') return reply(JSON.stringify({ profiles: [], scan: [], active_profile: 0, sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
        if (url === '/api/motion/status') return reply(device.status);
        if (url === '/api/motion/command') {
          const next = device.replies.shift();
          assert(next, 'unexpected command ' + init.body);
          if (next === 'hang') return new Promise((_, reject) => init.signal.addEventListener('abort', () => reject(new w.DOMException('aborted', 'AbortError')), { once: true }));
          if (next === 'network') throw new TypeError('Failed to fetch');
          let responseBody=next.body;try{const a=JSON.parse(responseBody),c=JSON.parse(init.body);if(a.ack===true&&!next.keepIdentity){a.id=c.id;a.sequence=c.sequence;responseBody=JSON.stringify(a)}}catch{}
          return reply(responseBody, next.status);
        }
        return reply('{}', 404);
      };
    } });
  await settle();
  const doc = dom.window.document, q = s => doc.querySelector(s);
  const poll = async (advance = 350) => { clock += advance; intervals.filter(i => i.ms === 350).forEach(i => i.fn()); await settle(); };
  const fire = async ms => { const t = timers.find(t => t.active && t.ms === ms); assert(t, 'active timer ' + ms); t.active = false; clock += ms; t.fn(); await settle(); };
  return { dom, device, q, calls, errors, poll, fire, timers, intervals, clock: () => clock };
}

(async () => {
  const p = await boot();
  const { q, device } = p;
  const envelopes = [];
  const commands = () => p.calls.filter(c => c.url === '/api/motion/command');
  const last = () => q('#simple-last').textContent;
  const kind = () => q('#simple-last').dataset.kind;
  const values = () => q('#simple-values').textContent;
  await p.poll();
  assert.deepStrictEqual(p.errors, [], 'shipped scripts run without DOM errors');
  assert.strictEqual(q('#simple-state').textContent, 'READY');
  assert.strictEqual(q('#simple-state').dataset.tone, 'ok', 'Controller READY with admission true is green');
  assert(!q('#simple-stand').disabled && !q('#simple-walk').disabled, 'STAND/WALK enabled by Controller readiness with a session');
  assert(values().includes('Motion ammessa dal Controller' + 'sì'));
  // Motion is the first, open page; STOP stays in the header on every page.
  assert.strictEqual(p.dom.window.document.querySelector('nav button').textContent, 'Motion');
  assert(!q('#motion-page').hidden && q('#dashboard').hidden && q('#motion-page').contains(q('#simple-motion')));
  q('[data-page="dashboard"]').click(); assert(q('#motion-page').hidden && !q('#dashboard').hidden && q('header #simple-stop'));
  p.dom.window.document.querySelector('nav button').click(); assert(!q('#motion-page').hidden);
  const twinPanel = q('#simple-twin');
  assert(twinPanel && !twinPanel.hidden && !twinPanel.closest('details'), 'Twin visible without opening a disclosure');
  assert(twinPanel.compareDocumentPosition(q('#simple-controls')) & p.dom.window.Node.DOCUMENT_POSITION_FOLLOWING, 'Twin precedes Motion controls in the cockpit');
  assert.strictEqual(q('#simple-twin h3 a'), null, 'no redundant Comandi link');
  assert.strictEqual(q('#simple-controls').className, 'command-panel');
  const joystick = q('#simple-joystick');
  assert(joystick && q('#joy-pad') && joystick.textContent.includes('NON VALIDATO SU HARDWARE') && joystick.textContent.includes('Rotazione'), 'joystick panel shipped, marked not hardware-validated');
  assert(q('#joy-left').disabled && q('#joy-right').disabled && /LATERAL_YAW_NOT_QUALIFIED/.test(q('#joy-left').title + q('#joy-right').title), 'rotation visible, disabled with the Controller refusal');
  const diagnostics = q('#simple-diagnostics');
  assert(diagnostics && !diagnostics.open && diagnostics.contains(q('#simple-values')) && diagnostics.contains(q('#simple-joints')) && diagnostics.contains(q('#simple-detail')), 'technical details remain in one closed diagnostic panel');
  assert(!q('#simple-state').closest('details') && !q('#simple-reason').closest('details') && !q('#simple-last').closest('details') && !q('#simple-stop').closest('details'), 'state, command result and STOP remain visible outside diagnostics');
  assert(q('#simple-stand').nextElementSibling === q('#simple-prone') && q('#simple-walk').nextElementSibling === q('#simple-walk-stop'), 'paired Motion controls preserved');
  assert.strictEqual(p.dom.window.getComputedStyle(q('header')).position, 'sticky', 'header STOP remains reachable while scrolling');
  assert.strictEqual(q('#simple-prone').textContent, 'PRONE');
  assert(!/sperimentale/i.test(q('#simple-motion').textContent + q('#simple-prone').title), 'PRONE copy reflects dev.7 validation');
  assert(/coppia ON/.test(q('#simple-prone').title) && /Appoggio non verificato/.test(q('#simple-prone').title), 'PRONE remains explicit about support and torque');
  const ready = JSON.parse(fx.status_ready);
  // Twelve joints in degrees from the real status rad values; measured pose and session rows.
  const jrows = [...q('#simple-joints').tBodies[0].rows];
  assert.strictEqual(jrows.length, 12);
  for (const [i, j] of ready.joints.entries()) {
    assert.strictEqual(jrows[i].cells[0].textContent, j.joint);
    assert(j.valid, 'idle fixture has fresh feedback');
    assert.strictEqual(jrows[i].cells[2].textContent, (ready.telemetry.actual_rad[j.slot] * 180 / Math.PI).toFixed(1) + '°', 'angle from the telemetry shared with the Twin');
  }
  assert(values().includes('Posa misurata (encoder)PRONE (ripiegata)') && ready.pose_region === 'PRONE_ENTRY', values());
  assert(values().includes('Sessione amministratorevalida'));
  assert(values().includes('Quota nominale richiesta (comando)') && values().includes('Quota nominale stimata (encoder + IMU)'));
  assert(values().includes('Quota base_link stimata (encoder + IMU)') && !values().includes('Ventre sul terreno'), 'height presentation identifies an estimate');
  assert(/nessuno inviato/.test(last()));
  let received = p.clock(), sequence = 0;

  // One command: send through the real UI control, check the envelope, record it.
  async function send(name, action, reply, type, params, ttl) {
    device.replies.push(reply);
    const before = commands().length;
    action();
    await settle();
    assert.strictEqual(commands().length, before + 1, name + ': one request');
    const call = commands().at(-1);
    assert.strictEqual(call.method, 'POST');
    assert.strictEqual(call.headers['Content-Type'], 'application/json');
    assert.strictEqual(call.headers['X-Csrf-Token'], CSRF);
    assert(call.body.length <= 1024, name + ': bounded body');
    const b = JSON.parse(call.body);
    assert.deepStrictEqual(Object.keys(b), ['version', 'id', 'sequence', 'issued_at_ms', 'ttl_ms', 'owner', 'type', 'params']);
    assert.strictEqual(b.version, 1); assert.strictEqual(b.owner, 'dashboard'); assert.strictEqual(b.type, type);
    assert.deepStrictEqual(b.params, params); assert.strictEqual(b.ttl_ms, ttl);
    assert(/^[A-Za-z0-9_.-]{1,48}$/.test(b.id)); assert.strictEqual(b.sequence, ++sequence);
    assert.strictEqual(b.issued_at_ms, Math.floor(JSON.parse(device.status).now_ms + p.clock() - received), name + ': Controller uptime estimate');
    if (name) envelopes.push(name + '\t' + call.body);
    return b;
  }
  const refresh = async status => { device.status = status; await p.poll(); received = p.clock(); };
  const ok = body => ({ status: 200, body });

  // Deduplicate repeated readiness, while real refusals remain explicit.
  {
    const t = JSON.parse(fx.status_ready); t.ready = false;
    t.reason = t.readiness_reason = 'SERVO_FEEDBACK_STALE';
    await refresh(JSON.stringify(t));
    assert.strictEqual(q('#simple-reason').textContent, 'Feedback dei 12 servo non aggiornati.');
    t.reason = 'IMU_STALE';
    await refresh(JSON.stringify(t));
    assert.strictEqual(q('#simple-reason').textContent, 'Campione IMU scaduto: torque OFF. · Feedback dei 12 servo non aggiornati.', 'distinct real reasons are preserved');
    for (const [reason, readiness] of [['OFFLINE_UI_FIXTURE', 'OFFLINE_UI_FIXTURE'], ['OFFLINE_UI_FIXTURE', 'OFFLINE_DISPLAY_ONLY']]) {
      t.reason = reason; t.readiness_reason = readiness;
      await refresh(JSON.stringify(t));
      assert.strictEqual(q('#simple-reason').textContent, 'Modalità offline', 'one readable offline indication before the Twin is ready');
      assert(!q('#simple-reason').hidden);
    }
    await refresh(fx.status_ready);
  }

  // Torque from the control read only: 1 ON, 0 OFF; -1, any other value, a
  // control read older than 3 s or an invalid feedback are UNKNOWN.
  {
    const t = JSON.parse(fx.status_ready);
    const set = (i, valid, torque, controlAge, age) => Object.assign(t.joints[i], { valid, torque, control_age_ms: controlAge, age_ms: age });
    set(0, true, 1, 100, 20); set(1, true, 0, 100, 20); set(2, true, -1, 0, 20); set(3, true, 1, 3001, 20);
    set(4, false, 1, 100, 20); set(5, true, 2, 100, 20); set(6, true, 0, 3000, 5000);
    await refresh(JSON.stringify(t));
    const cells = [...q('#simple-joints').tBodies[0].rows].map(r => [r.cells[5].textContent, r.cells[6].textContent, r.cells[7].textContent]);
    assert.deepStrictEqual(cells.slice(0, 7), [['ON', '100 ms', '20 ms'], ['OFF', '100 ms', '20 ms'], ['UNKNOWN', '0 ms', '20 ms'], ['UNKNOWN', '3001 ms', '20 ms'],
      ['UNKNOWN', '100 ms', '20 ms'], ['UNKNOWN', '100 ms', '20 ms'], ['OFF', '3000 ms', '5000 ms']]);
    assert(ready.joints.every(j => j.torque === -1 || j.torque === 0), 'real idle fixture: torque never read as ON');
    await refresh(fx.status_ready);
    assert([...q('#simple-joints').tBodies[0].rows].every(r => r.cells[5].textContent !== 'ON'), 'idle robot never shown with torque ON');
  }
  // Digital Twin: the EXISTING viewer parser (06_Software viewer, telemetry.ts)
  // accepts the real Controller status; same ACTUAL angles and BNO085 quaternion.
  {
    const viewer = path.join(__dirname, '../../../../06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer/src/motion/telemetry.ts');
    const { parseMotionFrame, JOINT_NAMES } = await import(require('url').pathToFileURL(viewer).href);
    for (const name of ['status_ready', 'status_walking', 'status_stopping', 'status_stopped']) {
      const status = JSON.parse(fx[name]), frame = parseMotionFrame(status);
      assert(frame && frame.quaternion && !frame.stale && frame.sampledAtMs <= frame.controllerNowMs, name + ': viewer accepts the Controller frame');
      if (status.telemetry.actual_rad) {
        assert(frame.actual && !frame.jointsStale, name);
        JOINT_NAMES.forEach((joint, i) => assert.strictEqual(frame.actual[joint], status.telemetry.actual_rad[i], name + ' ' + joint));
      } else {
        // SAFE_OFF verification pauses polling: the Twin shows the last angles as stale.
        assert(name === 'status_stopped' && frame.actual === null && frame.jointsStale && JOINT_NAMES.every((joint, i) => frame.actualByJoint[joint] === status.telemetry.actual_rad_by_joint[i]), name);
      }
    }
    const partial = JSON.parse(fx.status_ready); partial.telemetry.actual_rad = null; partial.telemetry.actual_rad_by_joint = Array(12).fill(0.1); partial.telemetry.actual_rad_by_joint[4] = null;
    const pf = parseMotionFrame(partial);
    assert(pf && pf.actual === null && pf.jointsStale && pf.actualByJoint[JOINT_NAMES[4]] === null, 'a joint never read stays unknown in the Twin');
    // The visible Twin is preloaded only at SAFE_OFF and renders this page's
    // own status (postMessage, same origin): one status read per poll
    // (test_twin_feed_parent.mjs: the Twin never polls).
    const frameEl = q('#simple-twin-frame');
    assert.strictEqual(frameEl.getAttribute('src'), '/twin/?feed=parent', 'Twin preloaded at SAFE_OFF');
    const posted = [], reads = () => p.calls.filter(c => c.url === '/api/motion/status').length;
    const frameWindow = { postMessage: (data, origin) => posted.push({ data, origin }) };
    Object.defineProperty(frameEl, 'contentWindow', { configurable: true, get: () => frameWindow });
    Object.defineProperty(frameEl, 'offsetHeight', { configurable: true, value: 502 });
    Object.defineProperty(frameEl, 'clientHeight', { configurable: true, value: 500 });
    const heightMessage = (height, origin = 'http://192.168.4.1', source = frameWindow) => p.dom.window.dispatchEvent(new p.dom.window.MessageEvent('message', { origin, source, data: { type: 'MATDOG_TWIN_HEIGHT', height } }));
    const initialHeight = frameEl.style.height, beforeHeight = p.calls.length;
    heightMessage(900, 'http://192.168.4.2'); heightMessage(900, 'http://192.168.4.1', {});
    for (const bad of [0, -1, NaN, Infinity, 10000, '900']) heightMessage(bad);
    assert.strictEqual(frameEl.style.height, initialHeight, 'only same-origin exact iframe finite bounded heights accepted');
    heightMessage(555.2); assert.strictEqual(frameEl.style.height, '558px', 'natural collapsed content height includes iframe borders');
    heightMessage(960); assert.strictEqual(frameEl.style.height, '962px', 'expanded diagnostics grow below the render surface without clipping the border-box');
    assert.strictEqual(p.calls.length, beforeHeight, 'iframe sizing issues no request');
    await refresh(fx.status_ready);
    assert.strictEqual(posted.length, 1, 'visible preloaded Twin receives same telemetry');posted.length=0;
    assert.strictEqual(frameEl.getAttribute('src'), '/twin/?feed=parent', 'Twin served by the same host, fed by this page');
    let before = reads();
    await refresh(fx.status_walking);
    assert.strictEqual(reads(), before + 1, 'Twin open: still one status read per poll');
    assert(posted.length === 1 && posted[0].origin === 'http://192.168.4.1', 'status posted to the same origin only');
    const polled = JSON.parse(fx.status_walking), { channels, ...rest } = posted[0].data;
    assert.deepStrictEqual(rest, polled, 'the Twin gets exactly the polled status');
    assert.deepStrictEqual(JSON.parse(JSON.stringify(channels)), { imu: { valid: polled.imu_valid, age_ms: polled.imu_age_ms }, raw: { count: 12 } }, 'IMU age and RAW count bound to the viewer channels');
    assert.strictEqual(frameEl.getAttribute('src'), '/twin/?feed=parent', 'Twin preserves its model between states');
    before = reads();
    await refresh(fx.status_ready);
    assert(reads() === before + 1 && posted.length === 2, 'idle again: one read and existing feed');
    // A first load interrupted by WALK may get a 503 for the module: retry the
    // iframe once after SAFE_OFF, without a timer/download loop during WALK.
    await refresh(fx.status_walking);
    const srcDescriptor = Object.getOwnPropertyDescriptor(p.dom.window.HTMLIFrameElement.prototype, 'src');
    let navigations = 0;
    Object.defineProperty(frameEl, 'src', { configurable: true, get(){ return srcDescriptor.get.call(this); }, set(v){ navigations++; srcDescriptor.set.call(this,v); } });
    const opened=navigations;
    await refresh(fx.status_walking); await refresh(fx.status_walking);
    assert.strictEqual(navigations,opened,'no repeated downloads while WALK is active');
    await refresh(fx.status_ready);
    assert.strictEqual(navigations,opened+1,'automatic iframe retry when motion is idle');
    await refresh(fx.status_ready);
    assert.strictEqual(navigations,opened+1,'no repeated reload after idle recovery');
    // The loaded Twin owns the sole demo badge; the command panel still
    // surfaces every real fault/refusal, even in an offline fixture.
    p.dom.window.dispatchEvent(new p.dom.window.MessageEvent('message', { origin: 'http://192.168.4.1', source: frameWindow, data: { type: 'MATDOG_TWIN_READY' } }));
    const demo = JSON.parse(fx.status_ready); demo.ready = false;
    demo.reason = 'OFFLINE_UI_FIXTURE'; demo.readiness_reason = 'OFFLINE_DISPLAY_ONLY';
    await refresh(JSON.stringify(demo));
    assert(q('#simple-reason').hidden && q('#simple-reason').textContent === '', 'loaded Twin demo badge is not duplicated');
    demo.fault = true;
    await refresh(JSON.stringify(demo));
    assert.strictEqual(q('#simple-state').textContent, 'FAULT');
    assert.strictEqual(q('#simple-state').dataset.tone, 'bad', 'fault flag takes priority over raw READY state');
    assert(!q('#simple-reason').hidden && q('#simple-reason').textContent === 'Guasto segnalato dal Controller.', 'fault flag is never hidden by a demo reason');
    demo.reason = 'IMU_STALE';
    await refresh(JSON.stringify(demo));
    assert(!q('#simple-reason').hidden && q('#simple-reason').textContent === 'Campione IMU scaduto: torque OFF.', 'a real fault stays visible beside the Twin demo badge');
    await refresh(fx.status_ready);
    delete frameEl.src;
    delete frameEl.contentWindow;
    // READY with admission false (stale servo feedback) is not shown as ready.
    const notAdmitted = JSON.parse(fx.status_ready); notAdmitted.ready = false; notAdmitted.readiness_reason = 'SERVO_FEEDBACK_STALE';
    await refresh(JSON.stringify(notAdmitted));
    assert.strictEqual(q('#simple-state').textContent, 'NON PRONTO');
    assert.strictEqual(q('#simple-state').dataset.tone, 'warn', 'admission false is never rendered green, even with state READY');
    assert(values().includes('Motion ammessa dal Controllerno · Feedback dei 12 servo non aggiornati.') && q('#simple-stand').disabled && !q('#simple-stop').disabled, values());
    notAdmitted.state = 'FAULT'; notAdmitted.fault = false;
    await refresh(JSON.stringify(notAdmitted));
    assert.strictEqual(q('#simple-state').dataset.tone, 'bad', 'Controller FAULT state is red independently of fault flag');
    notAdmitted.stale = true;
    await refresh(JSON.stringify(notAdmitted));
    assert.strictEqual(q('#simple-state').textContent, 'FAULT · DATI SCADUTI', 'stale data does not conceal an observed fault');
    assert.strictEqual(q('#simple-state').dataset.tone, 'bad');
    await refresh(fx.status_ready);
  }
  // Controller STOP facts, independent of labels and of an admission ACK.
  {
    const s = JSON.parse(fx.status_ready);
    s.state='MOVING';s.phase='STOP';s.reason='STOP_IN_PROGRESS';s.torque_active=true;s.safe_off_verified=false;s.safe_off_mask=0;
    s.stop={active:true,in_progress:true,motion_verified:false,support_verified:false,stand_resume_allowed:false,stand_resume_reason:'STOP_IN_PROGRESS'};
    await refresh(JSON.stringify(s));
    assert.strictEqual(q('#simple-state').textContent,'STOP IN CORSO');
    assert(q('#simple-stand').disabled&&q('#simple-walk').disabled&&q('#simple-walk-stop').disabled&&!q('#simple-stop').disabled);
    assert(values().includes('Compensazione IMUcongelata durante STOP')&&q('#simple-reason').textContent.includes('Supporto non verificato'));
    s.state='STOPPED';s.reason='STOPPED_SUPPORT_UNVERIFIED';s.stop.in_progress=false;s.stop.motion_verified=true;
    s.stop.stand_resume_reason='STOP_RECOVERY_POSTURE_UNQUALIFIED';
    await refresh(JSON.stringify(s));
    assert.strictEqual(q('#simple-state').textContent,'STOP COMPLETATO · COPPIA ON');assert(q('#simple-stand').disabled);
    assert(q('#simple-reason').textContent.includes('STOP_RECOVERY_POSTURE_UNQUALIFIED'), 'actual Controller recovery refusal is visible');
    s.stop.stand_resume_reason='STOP_RECOVERY_PATH_NOT_CLEAR';await refresh(JSON.stringify(s));
    assert(q('#simple-reason').textContent.includes('STOP_RECOVERY_PATH_NOT_CLEAR') && q('#simple-reason').textContent.includes('Supporto non verificato') && q('#simple-stand').disabled, 'geometric recovery refusal does not qualify support or bypass Controller admission');
    s.stop.stand_resume_reason='FUTURE_CONTROLLER_REFUSAL';await refresh(JSON.stringify(s));
    assert(q('#simple-reason').textContent.includes('FUTURE_CONTROLLER_REFUSAL') && q('#simple-stand').disabled, 'unknown refusal stays visible and never grants admission');
    s.stop.stand_resume_allowed=true;s.stop.stand_resume_reason='STAND_RECOVERY_AVAILABLE';await refresh(JSON.stringify(s));assert(!q('#simple-stand').disabled&&q('#simple-walk').disabled);
    assert(q('#simple-reason').textContent.includes('Recupero STAND ammesso dal Controller:') && q('#simple-reason').textContent.includes('piccolo avanzamento'), 'admission comes from Controller boolean');
    const recovering=JSON.parse(JSON.stringify(s));recovering.state='MOVING';recovering.phase='WALK_STANDARD';recovering.reason='STAND_FROM_STOPPED_WALK';recovering.stop.active=false;
    await refresh(JSON.stringify(recovering));assert(q('#simple-reason').textContent.includes('Recupero STAND dal WALK arrestato') && q('#simple-reason').textContent.includes('supporto non verificato') && q('#simple-stand').disabled && q('#simple-walk').disabled, 'explicit stopped-WALK recovery is displayed from Controller state while motion commands stay gated');
    s.stop.active=false;s.stop.stand_resume_allowed=false;s.stop.motion_verified=false;s.torque_active=false;s.safe_off_verified=true;s.safe_off_mask=8191;s.phase='IDLE';s.reason='SAFE_OFF_REQUESTED';
    await refresh(JSON.stringify(s));assert.strictEqual(q('#simple-state').textContent,'SAFE_OFF VERIFICATO · COPPIA OFF');assert(q('#simple-stand').disabled);
    s.fault=true;s.state='FAULT';s.reason='SERVO_COMMUNICATION_LOST';s.stale=false;await refresh(JSON.stringify(s));
    assert(q('#simple-state').textContent.startsWith('FAULT') && q('#simple-state').dataset.tone==='bad' && q('#simple-reason').textContent.includes('Comunicazione servo persa'), 'fresh feedback after a fault keeps FAULT red and preserves the original reason');
    const faultStopAck=JSON.parse(fx.ack_stop);faultStopAck.reason='STOP_REQUESTED_FAULT_LATCHED';
    await send('',()=>q('#simple-stop').click(),ok(JSON.stringify(faultStopAck)),'MOTION.STOP',{},5000);
    assert(last().includes('STOP_REQUESTED_FAULT_LATCHED') && last().includes('fault già presente'), 'STOP ACK explains that the prior fault remains latched');
    const faultStop=p.dom.window.matdogMotionTrace.commands.at(-1);
    s.active_command={id:faultStop.id,sequence:faultStop.sequence,outcome:'ACCEPTED',reason:'SAFE_OFF_IN_PROGRESS_FAULT_LATCHED'};
    await refresh(JSON.stringify(s));assert(last().includes('Verifica coppia OFF in corso') && q('#simple-state').dataset.tone==='bad', 'SAFE_OFF progress never masks the original FAULT');
    s.active_command={id:faultStop.id,sequence:faultStop.sequence,outcome:'COMPLETED',reason:'SAFE_OFF_VERIFIED_FAULT_LATCHED'};
    await refresh(JSON.stringify(s));
    assert(last().includes('COMPLETED') && last().includes('SAFE_OFF_VERIFIED_FAULT_LATCHED') && last().includes('il fault originale resta attivo'), 'successful SAFE_OFF completion is distinct from clearing a prior fault');
    assert(q('#simple-state').textContent==='FAULT' && q('#simple-state').dataset.tone==='bad' && q('#simple-reason').textContent.includes('Comunicazione servo persa') && q('#simple-stand').disabled, 'successful STOP in prior FAULT preserves red fault, original cause and movement refusal');
    assert.strictEqual(p.dom.window.document.querySelectorAll('#simple-stop').length,1);
    await refresh(fx.status_ready);
  }
  // 1-2. STAND 50 / 150 mm.
  q('#simple-height').value = '50';
  await send('stand50', () => q('#simple-stand').click(), ok(fx.ack_stand_accepted), 'POSTURE.STAND', { target_height_m: .05 }, 5000);
  assert(/STAND 50 mm · .* · HTTP 200 · ACCEPTED · COMMAND_ACCEPTED/.test(last()), last());
  assert(/movimento da confermare nello stato Motion/.test(last()) && kind() === 'acked');
  // Matching identity resolves even a delayed/lost ACK; unrelated statuses cannot.
  const traceCommand=p.dom.window.matdogMotionTrace.commands.at(-1);
  const completed=JSON.parse(fx.status_ready);completed.active_command={id:traceCommand.id,sequence:traceCommand.sequence,outcome:'COMPLETED',reason:'STAND_REACHED'};
  await refresh(JSON.stringify(completed));assert(/completamento confermato/.test(last())&&!/movimento da confermare/.test(last()),last());
  // 10. The ACK survives the telemetry render, also when the state changes.
  const ack = last();
  await refresh(fx.status_ready); assert.strictEqual(last(), ack, 'ACK kept after a poll');
  assert.strictEqual(q('#simple-reason').textContent, 'Pronto al comando.', 'telemetry reason rendered separately');
  await refresh(fx.status_moving); assert.strictEqual(last(), ack); assert.strictEqual(q('#simple-state').textContent, 'MOVING');
  assert(q('#simple-stand').disabled, 'no second STAND while MOVING');
  await refresh(fx.status_ready);
  q('#simple-height').value = '150';
  await send('stand150', () => q('#simple-stand').click(), ok(fx.ack_stand_accepted), 'POSTURE.STAND', { target_height_m: .15 }, 5000);
  assert(/STAND 150 mm/.test(last()));

  // 3. WALK STANDARD presets 30/45/60 mm per 6.5 s cycle.
  for (const [i, mm] of [[0, 30], [1, 45], [2, 60]]) {
    await refresh(fx.status_ready);
    q('#simple-stride').value = String(i);
    await send('walk' + mm, () => q('#simple-walk').click(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: [.030, .045, .060][i] / 6.5, vy: 0, yaw_rate: 0 }, 1000);
    assert(new RegExp('WALK STANDARD ' + mm + ' mm · .* · HTTP 200 · ACCEPTED').test(last()), last());
  }
  // 4. WALK STOP: zero velocity, available while WALKING.
  await refresh(fx.status_walking);
  assert.strictEqual(q('#simple-state').textContent, 'WALKING');
  assert(!q('#simple-walk-stop').disabled);
  await send('walk_stop', () => q('#simple-walk-stop').click(), ok(fx.ack_walk_stop), 'BASE_VELOCITY', { vx: 0, vy: 0, yaw_rate: 0 }, 1000);
  // 4b. JOYSTICK (dev.15): fore/aft intent with ttl 1 s, refreshed at 240 ms,
  // neutral on release, dead zone silent; disabled over WALK STANDARD and STOP.
  await refresh(fx.status_ready);
  {
    const pad = q('#joy-pad'), joyBox = q('#simple-joystick'), vmax = .045 / 6.5;
    const pointer = (type, y) => pad.dispatchEvent(new p.dom.window.MouseEvent(type, { bubbles: true, cancelable: true, clientY: y }));
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'false', 'joystick operable from READY with a session');
    const before = commands().length;
    pointer('pointerdown', 90); pointer('pointerup', 90); await settle();
    assert.strictEqual(commands().length, before, 'dead-zone press and release issue no request');
    await send('joystick_forward', () => pointer('pointerdown', 32), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: vmax, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    assert(/JOYSTICK avanti 6\.9 mm\/s \(100 %\)/.test(last()), last());
    const tick = p.intervals.filter(i => i.ms === 240).at(-1);
    assert(tick, 'refresh period inside the Controller 1 s deadline');
    await send('', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: vmax, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    pointer('pointermove', 119);  // half way back: proportional beyond the dead zone
    await send('joystick_backward', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: Math.sign(-.5) * Math.min(1, (.5 - .12) / (1 - .12)) * vmax, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    await send('joystick_release', () => pointer('pointerup', 119), ok(fx.ack_walk_stop), 'BASE_VELOCITY', { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    assert.strictEqual(q('#joy-knob').getAttribute('cy'), '90', 'knob back to neutral');
    const released = commands().length; tick.fn(); await settle();
    assert.strictEqual(commands().length, released, 'no refresh after release');
    await refresh(fx.status_walking);
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'true', 'no joystick over WALK STANDARD');
    pointer('pointerdown', 32); pointer('pointerup', 32); await settle();
    assert.strictEqual(commands().length, released, 'disabled joystick issues no request');
    const steered = JSON.parse(fx.status_walking); steered.walk_velocity = { vx_m_s: vmax, joystick: true }; steered.reason = 'WALK_JOYSTICK';
    await refresh(JSON.stringify(steered));
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'false', 'Controller joystick walk keeps the joystick operable');
    assert(/Controller: WALKING · Joystick: passo continuo in corso\. · legge del passo 6\.9 mm\/s/.test(q('#joy-state').textContent), q('#joy-state').textContent);
    await send('', () => pointer('pointerdown', 32), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: vmax, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    // Admission lost while pressed (STOP in progress): one neutral command, then disabled.
    device.replies.push(ok(fx.ack_walk_stop)); const pressed = commands().length;
    await refresh(fx.status_stopping); ++sequence;
    assert.strictEqual(commands().length, pressed + 1, 'one neutral command when admission is lost');
    assert.deepStrictEqual(JSON.parse(commands().at(-1).body).params, { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' });
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'true', 'joystick disabled while STOP is active');
    await refresh(fx.status_walking);
  }
  // 5. STOP button; 9. admission is not presented as verified arrest.
  await send('stop_button', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert(/STOP · .* · HTTP 200 · ACCEPTED · STOP_REQUESTED/.test(last()) && /arresto da verificare/.test(last()), last());
  await refresh(fx.status_stopping);
  assert(!values().includes('13/13 verificato') && /\d+\/13 verificati/.test(values()), values());
  await refresh(fx.status_stopped_on);
  assert.strictEqual(q('#simple-state').textContent,'STOP COMPLETATO · COPPIA ON');
  assert(!q('#simple-stand').disabled&&JSON.parse(fx.status_stopped_on).stop.stand_resume_allowed,
    'explicit STAND follows Controller admission after a verified synchronous HEIGHT/WALK stop');
  assert(values().includes('Compensazione IMUcongelata durante STOP'));
  const refusedStop=JSON.parse(fx.status_stopped_on);
  refusedStop.stop.stand_resume_allowed=false;refusedStop.stop.stand_resume_reason='STOP_RECOVERY_PATH_NOT_CLEAR';
  await refresh(JSON.stringify(refusedStop));
  assert(q('#simple-stand').disabled&&q('#simple-reason').textContent.includes('STOP_RECOVERY_PATH_NOT_CLEAR'),
    'UI preserves the Controller refusal and exposes its reason');
  await refresh(fx.status_stopped);
  assert(values().includes('SAFE_OFF13/13 verificato'), values());
  assert(/STOP_REQUESTED/.test(last()), 'STOP ACK kept until the next command');
  // 6. Escape sends the same STOP intent, also while another command is in flight.
  await refresh(fx.status_ready);
  q('#simple-height').value = '150';
  await send('', () => q('#simple-stand').click(), 'hang', 'POSTURE.STAND', { target_height_m: .15 }, 5000);
  assert(/in attesa della risposta/.test(last()) && kind() === 'pending');
  await send('stop_escape', () => p.dom.window.dispatchEvent(new p.dom.window.KeyboardEvent('keydown', { key: 'Escape' })), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert(/STOP \(Esc\) · .* · STOP_REQUESTED/.test(last()), last());
  // A held Escape autorepeats keydown: no further STOP; a new press still sends one.
  const beforeRepeat = commands().length;
  for (let i = 0; i < 5; i++) p.dom.window.dispatchEvent(new p.dom.window.KeyboardEvent('keydown', { key: 'Escape', repeat: true }));
  await settle();
  assert.strictEqual(commands().length, beforeRepeat, 'autorepeat sends no STOP');
  await send('', () => p.dom.window.dispatchEvent(new p.dom.window.KeyboardEvent('keydown', { key: 'Escape' })), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  // Browser abort at 1.2 s: the Controller budget is 3 s, so the outcome is unknown.
  await p.fire(5000);
  assert(/STOP \(Esc\).*STOP_REQUESTED/.test(last()), 'a late older timeout cannot overwrite STOP: '+last());

  // 8. Refusals show HTTP code, backend error and reason; 9. REJECTED is not success.
  // Explicit pre-execution refusals vs unknown outcomes (HttpTransport semantics):
  // only an error the transport sends before a command can run is a refusal.
  const R = 'RIFIUTATO PRIMA DELL’ESECUZIONE', U = 'ESITO SCONOSCIUTO';
  const outcomes = [
    [400, jsonError('INVALID_JSON_SCHEMA'), 'bad', /HTTP 400 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore INVALID_JSON_SCHEMA/],
    [403, jsonError('DASHBOARD_OWNER_REQUIRED'), 'bad', /HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore DASHBOARD_OWNER_REQUIRED/],
    [403, jsonError('GATE_CLOSING'), 'bad', /HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore GATE_CLOSING/],
    [403, jsonError('SUPERSEDED_BY_STOP'), 'bad', /HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore SUPERSEDED_BY_STOP/],
    [503, jsonError('MAILBOX_BUSY', 3000), 'bad', /HTTP 503 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore MAILBOX_BUSY/],
    [400, '{"version":1,"ok":false,"error":"BODY_TIMEOUT_OR_TRUNCATED","reason":"FIXTURE_REASON"}', 'bad', /RIFIUTATO PRIMA DELL’ESECUZIONE · errore BODY_TIMEOUT_OR_TRUNCATED · ragione FIXTURE_REASON/],
    [503, jsonError('CONTROLLER_LOOP_LATE', 3001), 'warn', /HTTP 503 · errore CONTROLLER_LOOP_LATE .* · ESITO SCONOSCIUTO: il Controller può averlo eseguito/],
    [500, jsonError('ACK_UNAVAILABLE'), 'warn', /HTTP 500 · errore ACK_UNAVAILABLE · ESITO SCONOSCIUTO/],
    [500, 'Internal Server Error', 'warn', /HTTP 500 · errore RISPOSTA_NON_DECODIFICABILE · ESITO SCONOSCIUTO/],
    [400, '<html>400 Bad Request</html>', 'warn', /HTTP 400 · errore RISPOSTA_NON_DECODIFICABILE · ESITO SCONOSCIUTO/],
    [200, '{"version":1}', 'warn', /HTTP 200 · errore RISPOSTA_SENZA_ACK · ESITO SCONOSCIUTO/],
  ];
  for (const [status, body, expectedKind, expected] of outcomes) {
    await refresh(fx.status_ready);
    await send('', () => q('#simple-stand').click(), { status, body }, 'POSTURE.STAND', { target_height_m: .15 }, 5000);  // exactly one request: never replayed
    assert(expected.test(last()) && kind() === expectedKind, last());
    assert(!/NON ESEGUITO|ACCEPTED/.test(last()) && (expectedKind === 'bad') === last().includes(R) && (expectedKind === 'warn') === last().includes(U), last());
    assert.strictEqual(q('#message').className, expectedKind); assert(expected.test(q('#message').textContent));
    const shown = last(); await refresh(fx.status_ready); assert.strictEqual(last(), shown, 'outcome kept after a poll');
  }
  await send('', () => q('#simple-stop').click(), { status: 503, body: jsonError('STOP_LANE_LATE', 600) }, 'MOTION.STOP', {}, 5000);
  assert(/STOP · .* · HTTP 503 · errore STOP_LANE_LATE \(.*\) · ESITO SCONOSCIUTO: .*per torque OFF globale usare @SERVO SAFE_OFF 51 via USB o il KEY OFF\./.test(last()) && kind() === 'bad', last());
  await refresh(fx.status_ready);
  await send('', () => q('#simple-stand').click(), ok(fx.ack_rejected), 'POSTURE.STAND', { target_height_m: .15 }, 5000);
  assert(/HTTP 200 · REJECTED dal Controller, non eseguito · IMU_UNAVAILABLE/.test(last()) && kind() === 'warn', last());
  assert(!/ACCEPTED|movimento da confermare/.test(last()), 'a REJECTED ACK is never shown as accepted motion');
  assert(/REJECTED/.test(q('#message').textContent), 'the refusal is in the banner');
  await refresh(fx.status_ready);
  q('#simple-height').value = '73';
  await send('', () => q('#simple-stand').click(), ok(fx.ack_stand_accepted), 'POSTURE.STAND', { target_height_m: .073 }, 5000);
  assert(q('#message').hidden && !/REJECTED/.test(q('#message').textContent), 'a later accepted command clears the earlier refusal: ' + q('#message').textContent);
  assert(p.dom.window.matdogMotionTrace.commands.some(c => c.ack && c.ack.outcome === 'REJECTED'), 'the refusal stays in the diagnostics');
  q('#simple-height').value = '150';
  await refresh(fx.status_ready);
  assert(values().includes('Release perse · ultimo movimento') && values().includes('Release perse · totale dal boot'));
  await refresh(fx.status_ready);
  await send('', () => q('#simple-stand').click(), 'network', 'POSTURE.STAND', { target_height_m: .15 }, 5000);
  assert(/errore di rete: Failed to fetch · ESITO SCONOSCIUTO/.test(last()), last());

  await refresh(fx.status_ready);
  await send('',()=>q('#simple-stand').click(),'hang','POSTURE.STAND',{target_height_m:.15},5000);
  await p.fire(5000);assert(/ESITO SCONOSCIUTO/.test(last()),last());
  const lost=p.dom.window.matdogMotionTrace.commands.at(-1), recovered=JSON.parse(fx.status_ready);
  recovered.active_command={id:lost.id,sequence:lost.sequence,outcome:'COMPLETED',reason:'STAND_REACHED'};
  await refresh(JSON.stringify(recovered));assert(/completamento confermato/.test(last())&&!/SCONOSCIUTO/.test(last()),last());
  assert(lost.error==='AbortError'&&lost.observed.outcome==='COMPLETED','trace keeps timeout AND later execution proof');
  await refresh(fx.status_ready);
  await send('',()=>q('#simple-stand').click(),{status:200,body:fx.ack_stand_accepted,keepIdentity:true},'POSTURE.STAND',{target_height_m:.15},5000);
  assert(/ACK_ID_MISMATCH.*ESITO SCONOSCIUTO/.test(last()),last());
  // B. One read lane for the whole page (PortalPage mdNet; listener: 2 sockets,
  // LRU purge). Request-level model only: no socket or latency is simulated.
  await refresh(fx.status_ready);
  const motionReads = () => p.calls.filter(c => c.url === '/api/motion/status');
  device.holdUrl = '/status';
  await p.fire(4000);  // the portal refresh (POLL_MS) issues /status in the lane
  const held = p.calls.at(-1);
  assert.strictEqual(held.url, '/status');
  const readsBefore = motionReads().length;
  await p.poll(0);  // a Motion poll tick while /status is in flight
  assert.deepStrictEqual(motionReads().slice(readsBefore).map(c => c.reads), [], 'the Motion read waits in the lane, never beside a portal read');
  await send('', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert.strictEqual(commands().at(-1).inflight, 2, 'STOP sent at once, beside at most one read');
  const releasedAt = p.timers.length;
  device.release(); await settle();
  const next = p.calls.slice(p.calls.indexOf(held) + 1).filter(c => c.method === 'GET').map(c => c.url);
  assert.deepStrictEqual(next.slice(0, 2), ['/api/motion/status', '/wifi/status'], 'the heartbeat read waits for at most one diagnostic read');
  const beat = motionReads().at(-1);
  assert.strictEqual(beat.headers['X-Csrf-Token'], CSRF, 'the poll still carries the deadman proof');
  assert(beat.timersAt - 1 >= releasedAt && p.timers[beat.timersAt - 1].ms === 800, 'the 800 ms read budget starts when the read is sent');
  assert.strictEqual(beat.reads, 1, 'one read in flight');
  // A diagnostic read that times out releases the lane to the Motion poll.
  device.holdUrl = '/status';
  await p.fire(4000);
  assert.strictEqual(p.calls.at(-1).url, '/status');
  const readsQueued = motionReads().length;
  await p.poll(0);
  assert.strictEqual(motionReads().length, readsQueued);
  await p.fire(800);  // get('/status') abort
  assert.strictEqual(motionReads().length, readsQueued + 1, 'Motion poll proceeds after an aborted diagnostic read');
  await refresh(fx.status_ready);

  // Admin session: kept valid while the page is used, never a Motion condition.
  const sessionCalls = () => p.calls.filter(c => c.url === '/wifi/session').length;
  await refresh(fx.status_ready);
  let renewals = sessionCalls();
  for (let t = 0; t < 40; t++) await p.poll(10000);  // 400 s of polling
  assert(sessionCalls() >= renewals + 3, 'session cookie refreshed about every 120 s');
  p.dom.window.eval('link()');  // the page's own temporary-session expiry check
  assert(!!p.dom.window.eval('csrf') && q('#login').hidden, 'session still valid after 400 s of use');
  received = p.clock();
  await send('', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert(/STOP_REQUESTED/.test(last()), 'STOP available after 400 s');
  // A refused STOP (expired session) is renewed and resent once.
  await refresh(fx.status_ready);
  renewals = sessionCalls();
  device.replies.push({ status: 403, body: jsonError('AUTH_FAILED') }, ok(fx.ack_stop));
  q('#simple-stop').click(); await settle(); await settle();
  const stops = commands().slice(-2).map(c => JSON.parse(c.body));
  assert(stops.length === 2 && stops.every(b => b.type === 'MOTION.STOP') && stops[1].sequence === stops[0].sequence + 1);
  sequence += 2;
  assert(sessionCalls() === renewals + 1 && /STOP · .* · HTTP 200 · ACCEPTED · STOP_REQUESTED/.test(last()), last());
  // Renewed but still refused: shown as a refusal, never a third attempt.
  device.replies.push({ status: 403, body: jsonError('AUTH_FAILED') }, { status: 403, body: jsonError('AUTH_FAILED') });
  q('#simple-stop').click(); await settle(); await settle(); sequence += 2;
  assert(/STOP · .* · HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · errore AUTH_FAILED \(Sessione/.test(last()) && kind() === 'bad', last());
  assert.strictEqual(device.replies.length, 0);
  // A session that cannot be renewed is shown, not hidden behind a valid-looking UI.
  device.sessionOk = false;
  device.replies.push({ status: 403, body: jsonError('AUTH_FAILED') });
  q('#simple-stop').click(); await settle(); await settle(); sequence += 1;
  assert(/AUTH_FAILED: sessione non rinnovabile, accedere di nuovo\. Usare @STOP via USB o il KEY OFF\./.test(last()) && !q('#login').hidden, last());
  assert.strictEqual(p.dom.window.eval('csrf'), '');
  // A lapsed session reported by the poll is resumed.
  device.sessionOk = true;
  const lapsed = JSON.parse(fx.status_ready); lapsed.admin_session = false;
  p.dom.window.eval("csrf='" + CSRF + "'");
  renewals = sessionCalls();
  await refresh(JSON.stringify(lapsed));
  assert.strictEqual(sessionCalls(), renewals + 1, 'lapsed session resumed from the poll');
  // Hiding the page never stops WALK: no presence condition in the browser either.
  await refresh(fx.status_walking);
  const beforeHidden = commands().length;
  Object.defineProperty(p.dom.window.document, 'hidden', { configurable: true, value: true });
  p.dom.window.document.dispatchEvent(new p.dom.window.Event('visibilitychange')); await settle();
  Object.defineProperty(p.dom.window.document, 'hidden', { configurable: true, value: false });
  assert.strictEqual(commands().length, beforeHidden, 'no WALK STOP when the page is hidden');
  await refresh(fx.status_ready);

  // PRONE is visible, explicit and requires an existing STAND HOLD; it never masquerades as STOP.
  // dev.5: offered only from STAND (the Controller still refuses it elsewhere: fixture ack_prone).
  assert(q('#simple-prone').disabled, 'PRONE disabled outside STANDING');
  const standing=JSON.parse(fx.status_ready);standing.state='STANDING';standing.phase='HOLD';await refresh(JSON.stringify(standing));
  assert(!q('#simple-prone').disabled, 'PRONE offered from STANDING');
  await send('prone', () => q('#simple-prone').click(), ok(fx.ack_prone), 'POSTURE.LIE_DOWN', {}, 5000);
  assert(/REJECTED.*PRONE_REQUIRES_STAND_HOLD/.test(last())); // Controller refusal still rendered
  await refresh(fx.status_prone_unverified);
  assert(/Appoggio non verificato/.test(q('#simple-reason').textContent));
  // Held PRONE terminal (state PRONE, torque ON): STAND/WALK offered without STOP; nothing to WALK STOP.
  assert(!q('#simple-stand').disabled && !q('#simple-walk').disabled, 'STAND after PRONE without STOP');
  assert(q('#simple-prone').disabled && q('#simple-walk-stop').disabled);
  assert(/goal fissi; appoggio non verificato/.test(q('#simple-values').textContent));
  assert(!q('#simple-stop').disabled);
  // A new HTTP response with old controller feedback is not fresh.
  const stale=JSON.parse(fx.status_ready);stale.stale=true;
  await refresh(JSON.stringify(stale));
  assert.strictEqual(q('#simple-state').textContent,'FEEDBACK NON FRESCO');
  assert(q('#simple-stand').disabled && q('#simple-prone').disabled && !q('#simple-stop').disabled);
  const delayed=JSON.parse(fx.status_ready);delayed.snapshot_age_ms=2000;
  await refresh(JSON.stringify(delayed));
  assert.strictEqual(q('#simple-state').textContent,'COLLEGAMENTO PERSO');
  // dev.5 (trace 30/10): source-fresh data stay fresh between polls. A render one
  // poll period (350 ms) after the response is not FEEDBACK NON FRESCO; only the
  // link bound (snapshot age >= 1500 ms) degrades it, to COLLEGAMENTO PERSO.
  await refresh(fx.status_ready);device.holdUrl='/api/motion/status';
  await p.poll(350);
  assert.strictEqual(q('#simple-state').textContent,'READY','fresh source data between polls');
  assert(/Età snapshot Motion\d+ ms/.test(values()) && parseInt(values().split('Età snapshot Motion')[1])>=350, 'effective page age stays visible');
  await p.poll(1200);
  assert.strictEqual(q('#simple-state').textContent,'COLLEGAMENTO PERSO','link bound unchanged');
  const tr=p.dom.window.matdogMotionTrace;assert(tr.poll_skipped>=1,'skipped poll recorded');
  device.release();await settle();
  // While the next GET hangs, age advances and stale controls disable without another response.
  await refresh(fx.status_walking);device.holdUrl='/api/motion/status';
  await p.poll(2000);
  assert.strictEqual(q('#simple-state').textContent,'COLLEGAMENTO PERSO');
  assert(!q('#simple-walk-stop').disabled && !q('#simple-stop').disabled, 'deceleration and emergency STOP remain reachable when telemetry is late');
  assert(values().includes('NON AGGIORNATO'));
  const agedRow=q('#simple-joints').tBodies[0].rows[0];
  assert(parseInt(agedRow.cells[7].textContent)>=2000);
  await send('', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  device.release();await settle();received=p.clock();
  // D. Controller readiness, not System READY, decides the controls.
  await refresh(fx.status_not_ready);
  assert(values().includes('Motion ammessa dal Controllerno · IMU non disponibile'), values());
  assert(q('#simple-stand').disabled && q('#simple-walk').disabled);
  // No session: nothing is sent and the page says so.
  p.dom.window.eval("csrf=''");
  const sent = commands().length;
  q('#simple-stop').click(); await settle();
  assert.strictEqual(commands().length, sent);
  assert(/STOP · .* · NON INVIATO: serve l’accesso amministratore\. Usare anche @STOP via USB o il KEY OFF\./.test(last()) && !q('#login').hidden, last());

  assert.deepStrictEqual(p.errors, []);
  assert(p.calls.filter(c => c.method === 'GET').every(c => c.reads === 1), 'never two page reads in flight during the whole session');
  // A session renewal of an older command cannot resend it after priority STOP.
  {
    const r=await boot();r.device.holdUrl='/wifi/session';
    r.device.replies.push({status:403,body:jsonError('AUTH_FAILED')});
    r.q('#simple-stand').click();await settle();
    assert(r.device.release,'old command waiting for session renewal');
    r.device.replies.push(ok(fx.ack_stop));r.q('#simple-stop').click();await settle();
    r.device.release();await settle();
    const sent=r.calls.filter(c=>c.url==='/api/motion/command').map(c=>JSON.parse(c.body).type);
    assert.deepStrictEqual(sent,['POSTURE.STAND','MOTION.STOP'],'no stale authenticated retry after STOP');
    assert(/STOP_REQUESTED/.test(r.q('#simple-last').textContent));
    assert.deepStrictEqual(r.errors,[]);r.dom.window.close();
  }
  fs.writeFileSync(envelopesPath, envelopes.join('\n') + '\n');
  console.log('SIMPLE_DASHBOARD_DOM = PASS (' + envelopes.length + ' envelopes captured from the shipped page; real MotionService fixtures; ' + (sequence) + ' commands)');
  p.dom.window.close();
})().catch(e => { console.error(e); process.exit(1); });
