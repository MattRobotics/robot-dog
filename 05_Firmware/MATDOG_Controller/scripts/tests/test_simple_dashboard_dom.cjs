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

async function boot(initialStatus = fx.status_ready) {
  let clock = 1000;
  const timers = [], intervals = [], calls = [], errors = [];
  // inflight: requests issued and not yet answered (each needs its own
  // HTTP/1.1 connection); holdUrl keeps every matching request pending until
  // release() (all) or releaseFirst() (the oldest); an abort clears holdUrl.
  const device = { status: initialStatus, replies: [], inflight: 0, reads: 0, holdUrl: null, release: null, releaseFirst: null, held: [], sessionOk: true };
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
          const entry = { resolve };
          device.held.push(entry);
          device.release = () => { device.holdUrl = null; device.held.splice(0).forEach(h => h.resolve()); };
          device.releaseFirst = () => { const h = device.held.shift(); assert(h, 'a held request'); h.resolve(); };
          init.signal?.addEventListener('abort', () => { device.holdUrl = null; const i = device.held.indexOf(entry); if (i >= 0) device.held.splice(i, 1); reject(new w.DOMException('aborted', 'AbortError')); }, { once: true });
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
  // dev.17 cockpit: one top section; the compact joystick has no badge and no
  // prose; the guide texts live in the closed diagnostics.
  assert.strictEqual(q('#simple-motion h2').textContent, 'MATDOG — Motion Cockpit');
  assert(joystick && q('#joy-pad') && joystick.textContent.includes('Rotazione') && !joystick.querySelector('.joystick-badge') &&
    !/VERIFICATO SU HARDWARE|mm\/s|Trascinare/.test(joystick.textContent.replace(q('#joy-state').textContent, '')), 'compact joystick: no badge, no prose');
  assert(/Joystick \(verificato su hardware in dev\.15\): trascinare/.test(q('#simple-diagnostics').textContent) &&
    /🐢 30, 45 o 🐇 60 mm è il passo per ciclo/.test(q('#simple-diagnostics').textContent), 'guide texts kept in the diagnostics');
  {
    // STAND height: 50-150 mm in 5 mm steps (slider + stepper); stride: 🐢 30 / 45 / 🐇 60 mm per cycle.
    // Choosing a value sends nothing: only STAND or WALK sends, and the Controller may refuse.
    const h = q('#simple-height'), before = p.calls.filter(c => c.url === '/api/motion/command').length;
    assert(h.type === 'range' && h.min === '50' && h.max === '150' && h.step === '5' && h.value === '150' && q('#simple-height-value').textContent === '150 mm');
    q('#simple-height-down').click(); assert(h.value === '145' && q('#simple-height-value').textContent === '145 mm');
    for (let k = 0; k < 30; k++) q('#simple-height-down').click();
    assert(h.value === '50' && q('#simple-height-value').textContent === '50 mm', 'stepper clamps at 50 mm');
    for (let k = 0; k < 30; k++) q('#simple-height-up').click();
    assert(h.value === '150', 'stepper clamps at 150 mm');
    h.value = '75'; h.dispatchEvent(new p.dom.window.Event('input', { bubbles: true })); h.dispatchEvent(new p.dom.window.Event('change', { bubbles: true }));
    assert.strictEqual(q('#simple-height-value').textContent, '75 mm');
    const radios = [...p.dom.window.document.querySelectorAll('input[name="simple-stride"]')];
    assert.deepStrictEqual(radios.map(r => r.value + ':' + r.nextElementSibling.textContent), ['0:🐢 30 mm', '1:45 mm', '2:🐇 60 mm']);
    assert(radios[1].checked && /passo per ciclo/.test(q('#simple-stride').closest('fieldset').textContent) && /non velocità/.test(q('#simple-stride').getAttribute('aria-label')));
    radios[2].click(); radios[0].click(); radios[1].click(); await settle();
    assert.strictEqual(p.calls.filter(c => c.url === '/api/motion/command').length, before, 'no command on a value change');
    h.value = '150'; h.dispatchEvent(new p.dom.window.Event('input', { bubbles: true }));
  }
  assert(/rotazione antioraria/.test(q('#joy-left').title) && /rotazione oraria/.test(q('#joy-right').title), 'rotation buttons shipped');
  const diagnostics = q('#simple-diagnostics');
  assert(diagnostics && !diagnostics.open && diagnostics.contains(q('#simple-values')) && diagnostics.contains(q('#simple-joints')) && diagnostics.contains(q('#simple-detail')), 'technical details remain in one closed diagnostic panel');
  assert(!q('#simple-state').closest('details') && !q('#simple-reason').closest('details') && !q('#simple-last').closest('details') && !q('#simple-stop').closest('details'), 'state, command result and STOP remain visible outside diagnostics');
  assert(q('#simple-stand').nextElementSibling === q('#simple-prone') && q('#simple-walk').nextElementSibling === q('#simple-walk-stop'), 'paired Motion controls preserved');
  assert.strictEqual(p.dom.window.getComputedStyle(q('header')).position, 'sticky', 'header STOP remains reachable while scrolling');
  assert.strictEqual(q('#simple-prone').textContent, 'PRONE');
  assert(!/sperimentale/i.test(q('#simple-motion').textContent + q('#simple-prone').title), 'PRONE copy reflects dev.7 validation');
  assert(/coppia viene rilasciata/.test(q('#simple-prone').title) && /Appoggio non verificato/.test(q('#simple-prone').title), 'PRONE remains explicit about support and torque');
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
    // dev.17: the Controller orders joystick commands by page session + sequence: "simple-<session>-<sequence>".
    assert(/^simple-[0-9a-z]+-\d+$/.test(b.id) && b.id.endsWith('-' + b.sequence), name + ': id carries the page session and its sequence');
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
    assert(!q('#simple-reason').hidden && q('#simple-reason').textContent === '', 'loaded Twin demo badge is not duplicated (the fixed slot stays, empty)');
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
    s.stop.stand_resume_allowed=true;s.stop.stand_resume_reason='STAND_RECOVERY_AVAILABLE';await refresh(JSON.stringify(s));
    assert(!q('#simple-stand').disabled&&!q('#simple-walk').disabled&&q('#simple-prone').disabled&&q('#simple-walk-stop').disabled, 'dev.17: STAND and WALK follow the same Controller admission after STOP; PRONE and WALK STOP stay off');
    assert(q('#simple-reason').textContent.includes('Ripresa ammessa dal Controller (STAND o WALK):') && q('#simple-reason').textContent.includes('piccolo avanzamento'), 'admission comes from Controller boolean');
    const recovering=JSON.parse(JSON.stringify(s));recovering.state='MOVING';recovering.phase='WALK_STANDARD';recovering.reason='STAND_FROM_STOPPED_WALK';recovering.stop.active=false;
    await refresh(JSON.stringify(recovering));assert(q('#simple-reason').textContent.includes('Ripresa dal WALK arrestato') && q('#simple-reason').textContent.includes('supporto non verificato') && q('#simple-stand').disabled && q('#simple-walk').disabled, 'explicit stopped-WALK recovery is displayed from Controller state while motion commands stay gated');
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
    q('input[name="simple-stride"][value="' + i + '"]').click();
    await send('walk' + mm, () => q('#simple-walk').click(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: [.030, .045, .060][i] / 6.5, vy: 0, yaw_rate: 0 }, 1000);
    assert(new RegExp('WALK STANDARD ' + mm + ' mm · .* · HTTP 200 · ACCEPTED').test(last()), last());
  }
  // 4. WALK STOP: zero velocity, available while WALKING.
  await refresh(fx.status_walking);
  assert.strictEqual(q('#simple-state').textContent, 'WALKING');
  assert(!q('#simple-walk-stop').disabled);
  await send('walk_stop', () => q('#simple-walk-stop').click(), ok(fx.ack_walk_stop), 'BASE_VELOCITY', { vx: 0, vy: 0, yaw_rate: 0 }, 1000);
  // 4b. JOYSTICK (dev.15): vx (pad up), vy (pad left), yaw_rate (held buttons),
  // ttl 1 s, refreshed at 240 ms, scaled into the Controller ellipsoid, neutral
  // on release, dead zone silent; disabled over WALK STANDARD and STOP.
  await refresh(fx.status_ready);
  {
    const pad = q('#joy-pad'), joyBox = q('#simple-joystick'), L = [.045 / 6.5, .020 / 6.5, .20 / 6.5];
    // Pointer ids as a browser gives them: one per finger (pad 1, rotation 2).
    const pointer = (el, type, x, y, id = el === pad ? 1 : 2) => { const ev = new p.dom.window.MouseEvent(type, { bubbles: true, cancelable: true, clientX: x, clientY: y }); Object.defineProperty(ev, 'pointerId', { value: id }); el.dispatchEvent(ev); };
    const shaped = (f, l, w) => { const r = Math.hypot(f, l), k = r < .12 ? 0 : Math.min(1, (r - .12) / (1 - .12)) / r; let n = [f * k, l * k, w]; const m = Math.hypot(n[0], n[1], n[2]); if (m > 1) n = n.map(v => v / m); return n.map((v, i) => v * L[i]); };
    const body = v => ({ vx: v[0], vy: v[1], yaw_rate: v[2], gait: 'JOYSTICK' });
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'false', 'joystick operable from READY with a session');
    assert(!q('#joy-left').disabled && !q('#joy-right').disabled, 'rotation buttons enabled with the joystick');
    const before = commands().length;
    pointer(pad, 'pointerdown', 90, 90); pointer(pad, 'pointerup', 90, 90); await settle();
    assert.strictEqual(commands().length, before, 'dead-zone press and release issue no request');
    await send('joystick_forward', () => pointer(pad, 'pointerdown', 90, 32), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    assert(/JOYSTICK avanti 6\.9 mm\/s/.test(last()), last());
    const tick = p.intervals.filter(i => i.ms === 240).at(-1);
    assert(tick, 'refresh period inside the Controller 1 s deadline');
    await send('', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    pointer(pad, 'pointermove', 90, 119);  // half way back: proportional beyond the dead zone
    await send('joystick_backward', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(-.5, 0, 0)), 1000);
    pointer(pad, 'pointermove', 61, 90);   // half way left
    await send('joystick_lateral', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(0, .5, 0)), 1000);
    // Pad full forward-right with the clockwise button held: scaled into the ellipsoid.
    pointer(pad, 'pointermove', 148, 32);
    await send('', () => pointer(q('#joy-right'), 'pointerdown', 0, 0), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1 / Math.SQRT2, -1 / Math.SQRT2, -1)), 1000);
    // Button released, pad still held: the pad intent alone is sent at once, no neutral.
    await send('', () => pointer(q('#joy-right'), 'pointerup', 0, 0), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1 / Math.SQRT2, -1 / Math.SQRT2, 0)), 1000);
    await send('joystick_release', () => pointer(pad, 'pointerup', 148, 32), ok(fx.ack_walk_stop), 'BASE_VELOCITY', { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    assert.strictEqual(q('#joy-knob').getAttribute('cy'), '90', 'knob back to neutral');
    // Rotation alone from a held button, then neutral on its release.
    await send('joystick_turn', () => pointer(q('#joy-left'), 'pointerdown', 0, 0), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(0, 0, 1)), 1000);
    assert(/antiorario 1\.76°\/s/.test(last()), last());
    await send('', () => pointer(q('#joy-left'), 'pointerup', 0, 0), ok(fx.ack_walk_stop), 'BASE_VELOCITY', { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, 1000);
    const released = commands().length; p.intervals.filter(i => i.ms === 240).forEach(i => i.fn()); await settle();
    assert.strictEqual(commands().length, released, 'no refresh after release');
    await refresh(fx.status_walking);
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'true', 'no joystick over WALK STANDARD');
    assert(q('#joy-left').disabled && q('#joy-right').disabled, 'rotation disabled with the joystick');
    pointer(pad, 'pointerdown', 90, 32); pointer(pad, 'pointerup', 90, 32); await settle();
    assert.strictEqual(commands().length, released, 'disabled joystick issues no request');
    const steered = JSON.parse(fx.status_walking); steered.walk_velocity = { vx_m_s: L[0], vy_m_s: 0, yaw_rate_rad_s: 0, joystick: true }; steered.reason = 'WALK_JOYSTICK';
    await refresh(JSON.stringify(steered));
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'false', 'Controller joystick walk keeps the joystick operable');
    assert(/Controller: WALKING · Joystick: passo continuo in corso\. · legge del passo avanti 6\.9 mm\/s/.test(q('#joy-state').textContent), q('#joy-state').textContent);
    await send('', () => pointer(pad, 'pointerdown', 90, 32), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    // Admission lost while pressed (STOP in progress): one neutral command, then disabled.
    device.replies.push(ok(fx.ack_walk_stop)); const pressed = commands().length;
    await refresh(fx.status_stopping); ++sequence;
    assert.strictEqual(commands().length, pressed + 1, 'one neutral command when admission is lost');
    assert.deepStrictEqual(JSON.parse(commands().at(-1).body).params, { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' });
    assert.strictEqual(joyBox.getAttribute('aria-disabled'), 'true', 'joystick disabled while STOP is active');
    assert(values().includes('Correzione IMU (richiesta → applicata, mrad)') && values().includes('Provenienza giunti'), 'IMU correction and joint provenance in diagnostics');
    await refresh(fx.status_walking);
  }
  // 4c. dev.17 input robustness: the pad centre is anchored at pointerdown, so a
  // reflow cannot change the request; a refusal banner never ends a hold and
  // never enters the page flow; the intent ends on a release anywhere, on
  // pointercancel, focus loss and a hidden page; a network failure leaves no
  // stuck sender; the Twin's resize waits for the release.
  await refresh(fx.status_ready);
  {
    const pad = q('#joy-pad'), L = [.045 / 6.5, .020 / 6.5, .20 / 6.5], doc = p.dom.window.document;
    const pointer = (el, type, x, y, id) => { const ev = new p.dom.window.MouseEvent(type, { bubbles: true, cancelable: true, clientX: x, clientY: y }); Object.defineProperty(ev, 'pointerId', { value: id }); el.dispatchEvent(ev); };
    const shaped = (f, l, w) => { const r = Math.hypot(f, l), k = r < .12 ? 0 : Math.min(1, (r - .12) / (1 - .12)) / r; let n = [f * k, l * k, w]; const m = Math.hypot(n[0], n[1], n[2]); if (m > 1) n = n.map(v => v / m); return n.map((v, i) => v * L[i]); };
    const body = v => ({ vx: v[0], vy: v[1], yaw_rate: v[2], gait: 'JOYSTICK' }), neutral = { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' };
    let rect = { left: 0, top: 0, width: 180, height: 180 };
    pad.getBoundingClientRect = () => ({ ...rect, x: rect.left, y: rect.top, right: rect.left + rect.width, bottom: rect.top + rect.height });
    const frameEl = q('#simple-twin-frame'), twinHeight = h => p.dom.window.dispatchEvent(new p.dom.window.MessageEvent('message', { origin: 'http://192.168.4.1', source: frameEl.contentWindow, data: { type: 'MATDOG_TWIN_HEIGHT', height: h } }));
    await send('', () => pointer(pad, 'pointerdown', 90, 32, 7), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    const tick = p.intervals.filter(i => i.ms === 240).at(-1);
    // The page reflows 60 px while the finger stays still: same request.
    rect = { left: 0, top: 60, width: 180, height: 180 };
    pointer(pad, 'pointermove', 90, 32, 7);
    await send('', () => tick.fn(), { status: 503, body: jsonError('CONTROLLER_LOOP_LATE', 101) }, 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    assert(!q('#message').hidden && /CONTROLLER_LOOP_LATE/.test(q('#message').textContent), 'refusal banner shown while the pad is held');
    assert.strictEqual(q('#message').parentElement, q('header'), 'the banner is an overlay in the header, outside the page flow');
    assert.strictEqual(q('#simple-last').title, q('#simple-last').textContent, 'full result in the fixed slot title');
    assert(/CONTROLLER_LOOP_LATE/.test(q('#simple-log li').textContent) && q('#simple-log li').dataset.kind === 'warn', 'refusal kept in the diagnostics log');
    await send('', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    // Another finger lifted elsewhere does not end this hold.
    pointer(doc.body, 'pointerup', 10, 10, 8); await settle();
    await send('', () => tick.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    const heightBefore = frameEl.style.height;
    twinHeight(700);
    assert.strictEqual(frameEl.style.height, heightBefore, 'Twin resize deferred while the joystick is held');
    // The finger lifted outside the pad, its capture missing: released all the same.
    await send('', () => pointer(doc.body, 'pointerup', 300, 400, 7), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    assert.strictEqual(frameEl.style.height, '702px', 'Twin resize applied on release');
    rect = { left: 0, top: 0, width: 180, height: 180 };
    // pointercancel (a system gesture) releases.
    await send('', () => pointer(pad, 'pointerdown', 90, 32, 9), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    await send('', () => pointer(pad, 'pointercancel', 90, 32, 9), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    // Focus loss releases.
    await send('', () => pointer(pad, 'pointerdown', 90, 32, 10), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    await send('', () => p.dom.window.dispatchEvent(new p.dom.window.Event('blur')), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    // A hidden page releases.
    await send('', () => pointer(pad, 'pointerdown', 90, 32, 11), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    Object.defineProperty(doc, 'hidden', { configurable: true, value: true });
    await send('', () => doc.dispatchEvent(new p.dom.window.Event('visibilitychange')), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    Object.defineProperty(doc, 'hidden', { configurable: true, value: false });
    doc.dispatchEvent(new p.dom.window.Event('visibilitychange')); await settle();  // visible again: the portal refresh restarts
    // A rotation button left without capture releases.
    await send('', () => pointer(q('#joy-left'), 'pointerdown', 0, 0, 12), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(0, 0, 1)), 1000);
    await send('', () => pointer(q('#joy-left'), 'pointerleave', 0, 0, 12), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    // Connection interrupted on a refresh: no stuck sender, the next tick sends
    // again (the Controller's own 1 s deadline covers the gap), release is neutral.
    await send('', () => pointer(pad, 'pointerdown', 90, 32, 13), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    const tick2 = p.intervals.filter(i => i.ms === 240).at(-1);
    await send('', () => tick2.fn(), 'network', 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    assert(/errore di rete/.test(last()), last());
    await send('', () => tick2.fn(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', body(shaped(1, 0, 0)), 1000);
    await send('', () => pointer(pad, 'pointerup', 90, 32, 13), ok(fx.ack_walk_stop), 'BASE_VELOCITY', neutral, 1000);
    const idle = commands().length; p.intervals.filter(i => i.ms === 240).forEach(i => i.fn()); await settle();
    assert.strictEqual(commands().length, idle, 'no refresh after any release');
    delete pad.getBoundingClientRect;
    q('#message').click();
  }
  // 5. STOP button; 9. admission is not presented as verified arrest.
  await send('stop_button', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  { const r = p.dom.window.matdogMotionTrace.commands.at(-1);  // dev.17: STOP input -> request measurable on the phone
    // jsdom stamps events with the real epoch clock, a browser with performance.now()'s origin:
    // the clock agreement is checked in Chrome (test_stop_twin_load_browser.cjs), presence here.
    assert(r.type === 'MOTION.STOP' && Number.isFinite(r.input_ms), 'STOP trace record carries the input event time: ' + JSON.stringify(r)); }
  assert(/STOP · .* · HTTP 200 · ACCEPTED · STOP_REQUESTED/.test(last()) && /arresto da verificare/.test(last()), last());
  await refresh(fx.status_stopping);
  assert(!values().includes('13/13 verificato') && /\d+\/13 verificati/.test(values()), values());
  await refresh(fx.status_stopped_on);
  assert.strictEqual(q('#simple-state').textContent,'STOP COMPLETATO · COPPIA ON');
  assert(!q('#simple-stand').disabled&&!q('#simple-walk').disabled&&JSON.parse(fx.status_stopped_on).stop.stand_resume_allowed,
    'explicit STAND or WALK follows Controller admission after a verified synchronous HEIGHT/WALK stop');
  assert(values().includes('Compensazione IMUcongelata durante STOP'));
  const refusedStop=JSON.parse(fx.status_stopped_on);
  refusedStop.stop.stand_resume_allowed=false;refusedStop.stop.stand_resume_reason='STOP_RECOVERY_PATH_NOT_CLEAR';
  await refresh(JSON.stringify(refusedStop));
  assert(q('#simple-stand').disabled&&q('#simple-walk').disabled&&q('#simple-reason').textContent.includes('STOP_RECOVERY_PATH_NOT_CLEAR'),
    'UI preserves the Controller refusal and exposes its reason');
  await refresh(fx.status_stopped);
  assert(values().includes('SAFE_OFF13/13 verificato'), values());
  assert(/STOP_REQUESTED/.test(last()), 'STOP ACK kept until the next command');
  // dev.17: WALK after STOP is the ordinary WALK STANDARD preset; the Controller admits it.
  await refresh(fx.status_stopped_on);
  { const i = Number(q('input[name="simple-stride"]:checked').value);
    await send('', () => q('#simple-walk').click(), ok(fx.ack_walk_accepted), 'BASE_VELOCITY', { vx: [.030, .045, .060][i] / 6.5, vy: 0, yaw_rate: 0 }, 1000);
    assert(new RegExp('WALK STANDARD ' + [30, 45, 60][i] + ' mm · .* · HTTP 200 · ACCEPTED').test(last()), last()); }
  await refresh(fx.status_stopped);
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
  q('#simple-height').value = '75';
  await send('', () => q('#simple-stand').click(), ok(fx.ack_stand_accepted), 'POSTURE.STAND', { target_height_m: .075 }, 5000);
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
  // B. Read lanes (dev.17). The portal's diagnostic lane (mdNet) and the Motion
  // poll's own lane: never more than two reads in flight for the whole page, a
  // read is aborted only after 3.5 s (lwIP retransmission timeout), a stalled
  // read is overlapped once after 700 ms, and a late answer never overwrites a
  // newer one. Request-level model only: no socket or latency is simulated.
  await refresh(fx.status_ready);
  const motionReads = () => p.calls.filter(c => c.url === '/api/motion/status');
  const trace = p.dom.window.matdogMotionTrace;
  device.holdUrl = '/status';
  await p.fire(4000);  // the portal refresh (POLL_MS) issues /status in its lane
  const held = p.calls.at(-1);
  assert.strictEqual(held.url, '/status');
  assert(p.timers[held.timersAt - 1].ms === 3500 && p.timers[held.timersAt - 1].active, 'the portal read budget is 3.5 s, armed when the read is sent');
  let readsBefore = motionReads().length;
  await p.poll(); received = p.clock();  // a Motion poll tick while /status is in flight
  assert.deepStrictEqual(motionReads().slice(readsBefore).map(c => c.reads), [2], 'the Motion read does not wait behind a portal read');
  await send('', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert.strictEqual(commands().at(-1).inflight, 2, 'STOP sent at once, beside the held read');
  device.release(); await settle();
  assert.deepStrictEqual(p.calls.slice(p.calls.indexOf(held) + 1).filter(c => c.method === 'GET' && c.url !== '/api/motion/status').map(c => c.url), ['/wifi/status'], 'portal reads stay sequential in their lane');
  const beat = motionReads().at(-1);
  assert.strictEqual(beat.headers['X-Csrf-Token'], CSRF, 'the poll still carries the deadman proof');
  assert(p.timers[beat.timersAt - 1].ms === 3500, 'the Motion read budget is 3.5 s, armed when the read is sent');
  // A stalled Motion read (one TCP segment lost on the Controller): the next
  // ticks inside 700 ms are skipped, then ONE overlapping read keeps telemetry
  // live; with two in flight every tick is skipped; the late answer is dropped.
  await refresh(fx.status_ready);
  device.holdUrl = '/api/motion/status';
  readsBefore = motionReads().length;
  let skipped = trace.poll_skipped;
  await p.poll();  // read 1, held
  await p.poll();  // 350 ms: skipped
  assert.strictEqual(motionReads().length, readsBefore + 1);
  assert.strictEqual(trace.poll_skipped, skipped + 1, 'a tick inside READ_OVERLAP_MS is skipped and counted');
  device.holdUrl = null;
  device.status = fx.status_walking;
  await p.poll();  // 700 ms: read 2 overlaps read 1 and is answered
  assert.strictEqual(motionReads().length, readsBefore + 2);
  assert.strictEqual(motionReads().at(-1).reads, 2, 'two reads in flight at most');
  assert.strictEqual(q('#simple-state').textContent, 'WALKING', 'telemetry continues beside the stalled read');
  const samples = trace.samples.length, dropped = trace.late_dropped;
  device.status = fx.status_ready;
  device.releaseFirst(); await settle();  // read 1 answers after read 2
  assert.strictEqual(trace.late_dropped, dropped + 1, 'late answer counted');
  assert.strictEqual(trace.samples.length, samples, 'late answer not recorded as a sample');
  assert.strictEqual(q('#simple-state').textContent, 'WALKING', 'a late answer never overwrites a newer one');
  // Two reads held: no third read, STOP still has its connection at once.
  await refresh(fx.status_ready);
  device.holdUrl = '/api/motion/status';
  readsBefore = motionReads().length; skipped = trace.poll_skipped;
  await p.poll(); await p.poll(700); await p.poll(); await p.poll(700);
  assert.strictEqual(motionReads().length, readsBefore + 2, 'never a third read while two are in flight');
  assert.strictEqual(trace.poll_skipped, skipped + 2);
  await send('', () => q('#simple-stop').click(), ok(fx.ack_stop), 'MOTION.STOP', {}, 5000);
  assert.strictEqual(commands().at(-1).inflight, 3, 'STOP sent at once beside the two held reads');
  // Nor does the portal lane add a third read: its diagnostic refresh is
  // deferred, not sent, and a deferral is not a lost link.
  const portalBefore = p.calls.filter(c => c.url === '/status').length;
  await p.fire(4000);
  assert.strictEqual(p.calls.filter(c => c.url === '/status').length, portalBefore, 'no third read: the diagnostic refresh is deferred');
  assert(p.timers.some(t => t.active && t.ms === 4000), 'diagnostic refresh rescheduled at its normal cadence');
  assert(!/Ultimo tentativo senza risposta|Nessuna risposta/.test(q('#linkText').textContent), q('#linkText').textContent);
  // Only the 3.5 s budget ends a stalled read: a TIMEOUT, then the lane reopens.
  const errors = trace.poll_errors.length;
  await p.fire(3500); await p.fire(3500);
  assert(trace.poll_errors.slice(errors).some(e => e.error === 'TIMEOUT_3500MS'), 'a read is aborted only at 3.5 s');
  device.release(); await settle();
  await refresh(fx.status_ready);
  assert.strictEqual(q('#simple-state').textContent, 'READY', 'reads resume after the abort');

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
  await refresh(fx.status_prone_rest);
  assert(/PRONE a riposo: coppia OFF/.test(q('#simple-reason').textContent) && /Appoggio passivo non misurato/.test(q('#simple-reason').textContent));
  // PRONE rest (READY, torque OFF verified): STAND/WALK offered (they re-acquire the pose); nothing to WALK STOP.
  assert(!q('#simple-stand').disabled && !q('#simple-walk').disabled, 'STAND after PRONE rest');
  assert(q('#simple-prone').disabled && q('#simple-walk-stop').disabled);
  assert(/Compensazione IMUnon attiva/.test(q('#simple-values').textContent));
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
  assert(p.calls.filter(c => c.method === 'GET').every(c => c.reads <= 2), 'never more than two page reads in flight, both lanes counted');
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
  // dev.17: the Twin after a page reload with torque ON (dev.16 hardware: it
  // stayed unavailable until SAFE_OFF). Started at the first poll in HOLD (the
  // Controller serves its large files as 304 from the browser cache, otherwise
  // defers them), never during WALK/MOVING or a held joystick, never reloaded
  // while torque is ON, retried once at torque OFF, kept once ready.
  {
    const spy = r => { let n = 0; const el = r.q('#simple-twin-frame'), d = Object.getOwnPropertyDescriptor(r.dom.window.HTMLIFrameElement.prototype, 'src');
      Object.defineProperty(el, 'src', { configurable: true, get() { return d.get.call(this); }, set(v) { n++; d.set.call(this, v); } }); return () => n; };
    assert(JSON.parse(fx.status_standing).torque_active === true && JSON.parse(fx.status_standing).state === 'STANDING', 'real HOLD fixture with torque ON');
    const hold = await boot(fx.status_standing);
    assert.strictEqual(hold.q('#simple-twin-frame').getAttribute('src'), '/twin/?feed=parent', 'a reload in HOLD with torque ON starts the Twin');
    assert(/coppia ON: solo dalla cache del browser/.test(hold.q('#simple-twin-state').title), hold.q('#simple-twin-state').title);
    const navs = spy(hold);
    for (let i = 0; i < 4; i++) await hold.poll();
    assert.strictEqual(navs(), 0, 'no reload loop while torque is ON');
    hold.device.status = fx.status_ready; await hold.poll();
    assert.strictEqual(navs(), 1, 'not ready: one retry at torque OFF');
    await hold.poll(); await hold.poll();
    assert.strictEqual(navs(), 1, 'no repeated reload');
    const walking = await boot(fx.status_walking);
    assert(!walking.q('#simple-twin-frame').getAttribute('src'), 'no Twin load during WALK');
    assert(/a robot fermo/.test(walking.q('#simple-twin-state').title), walking.q('#simple-twin-state').title);
    walking.device.status = fx.status_standing; await walking.poll();
    assert.strictEqual(walking.q('#simple-twin-frame').getAttribute('src'), '/twin/?feed=parent', 'started at the next HOLD, torque ON');
    const frame = walking.q('#simple-twin-frame'), navs2 = spy(walking), win = { postMessage() {} };
    Object.defineProperty(frame, 'contentWindow', { configurable: true, get: () => win });
    walking.dom.window.dispatchEvent(new walking.dom.window.MessageEvent('message', { origin: 'http://192.168.4.1', source: win, data: { type: 'MATDOG_TWIN_READY' } }));
    walking.device.status = fx.status_ready; await walking.poll(); await walking.poll();
    assert.strictEqual(navs2(), 0, 'a ready Twin is kept, never reloaded at torque OFF');
    const steered = JSON.parse(fx.status_walking); steered.walk_velocity = { vx_m_s: .045 / 6.5, vy_m_s: 0, yaw_rate_rad_s: 0, joystick: true }; steered.reason = 'WALK_JOYSTICK';
    const js = await boot(JSON.stringify(steered)), pad = js.q('#joy-pad');
    const press = (type, id) => { const ev = new js.dom.window.MouseEvent(type, { bubbles: true, cancelable: true, clientX: 90, clientY: 32 }); Object.defineProperty(ev, 'pointerId', { value: id }); pad.dispatchEvent(ev); };
    assert(!js.q('#simple-twin-frame').getAttribute('src'), 'no Twin load during a joystick walk');
    js.device.replies.push({ status: 200, body: fx.ack_walk_accepted }); press('pointerdown', 1); await settle();
    js.device.status = fx.status_standing; await js.poll();
    assert(!js.q('#simple-twin-frame').getAttribute('src'), 'no Twin load while the joystick is held');
    js.device.replies.push({ status: 200, body: fx.ack_walk_stop }); press('pointerup', 1); await settle(); await js.poll();
    assert.strictEqual(js.q('#simple-twin-frame').getAttribute('src'), '/twin/?feed=parent', 'started after the release, in HOLD');
    assert.deepStrictEqual(js.calls.filter(c => c.url === '/api/motion/command').map(c => JSON.parse(c.body).params.vx > 0 ? 'move' : 'neutral'), ['move', 'neutral']);
    assert.deepStrictEqual([hold, walking, js].flatMap(r => r.errors), []);
    for (const r of [hold, walking, js]) r.dom.window.close();
  }
  fs.writeFileSync(envelopesPath, envelopes.join('\n') + '\n');
  console.log('SIMPLE_DASHBOARD_DOM = PASS (' + envelopes.length + ' envelopes captured from the shipped page; real MotionService fixtures; ' + (sequence) + ' commands)');
  p.dom.window.close();
})().catch(e => { console.error(e); process.exit(1); });
