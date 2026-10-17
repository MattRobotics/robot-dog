// Executable browser DOM regression against the SHIPPED embedded page.
// NODE_PATH may point to an isolated jsdom install. All requests are fixtures.
const { JSDOM } = require('jsdom');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const src = path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const parts = [...portal.matchAll(/R"MATDOG\(([\s\S]*?)\)MATDOG"/g)].map(m => m[1]);
const html = parts[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const contract = fs.readFileSync(path.join(src, 'motion/MotionContract.cpp'), 'utf8');
const actions = [...contract.matchAll(/A\("([^"]+)","([^"]+)","([^"]+)","([^"]+)","([^"]+)",(true|false),/g)].map(m => ({
  id: m[1], category: m[2], label: m[3], parameters: m[4], capability: m[5],
  implemented: m[6] === 'true', implementation: m[6] === 'true' ? 'software_tested' : 'planned',
  prerequisites: 'Controller measured evidence and explicit GO', available: false, reason: 'OPERATOR_GO_REQUIRED',
}));
assert.strictEqual(actions.length, 48, 'test uses all 48 actual Controller catalog IDs');
const settle = async () => { for (let i = 0; i < 20; i++) await new Promise(r => setImmediate(r)); };
async function boot(enabled = false, options = {}) {
  let clock = 1000;
  const device = { down: false, holdCommand: null, commands: [], restores: [], inflight: 0, maxReadInflight: 0, failures: {}, rtt: options.rtt || {}, caps: { version: 1, hardware_approved: enabled, teleoperation: enabled,
    limits: { vx_m_s: .03, vy_m_s: .02, yaw_rate_rad_s: .15 }, actions: actions.map(a => ({ ...a, label: a.id === 'action.wave' ? '<img src=x onerror=alert(1)>' : a.label, available: enabled && a.implemented || a.id.startsWith('motion.') })) },
    status: { version: 1, now_ms: 1000, last_sequence: 10, executor_state: 'OFF', stand_state: 'OBSERVE', stale: false, fault: false, owner: 'none', readiness: { restore: true, permit: enabled } } };
  const intervals = [], timers = [], errors = [], calls = [];
  const dom = new JSDOM(html, { url: options.url || 'http://192.168.4.1/', runScripts: 'dangerously', pretendToBeVisual: true,
    beforeParse(w) {
      Object.defineProperty(w.performance, 'now', { value: () => clock });
      w.setTimeout = (fn, ms) => { timers.push({ fn, ms, active: true }); return timers.length; }; w.clearTimeout = id => { if (timers[id - 1]) timers[id - 1].active = false; };
      w.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
      w.addEventListener('error', e => errors.push(e.message));
      w.fetch = async (url, init = {}) => {
        calls.push({ url, method: init.method || 'GET', csrf: (init.headers || {})['X-Csrf-Token'] });
        const read = !init.method || init.method === 'GET';
        if (read) { device.inflight++; device.maxReadInflight = Math.max(device.maxReadInflight, device.inflight); }
        try { return await serve(url, init); } finally { if (read) device.inflight--; }
      };
      const serve = async (url, init) => {
        if (url === '/api/motion/command') device.commands.push(JSON.parse(init.body));
        if (device.down) throw Error('fixture connection lost');
        clock += device.rtt[url] || 0;
        const failure = device.failures[url]?.shift();
        const aborted = () => new Promise((resolve, reject) => init.signal.addEventListener('abort', () => reject(new w.DOMException('fixture abort', 'AbortError')), { once: true }));
        if (failure === 'network') throw Error('fixture connection lost');
        if (failure === 'timeout') return aborted();
        const reply = (text, status = 200) => ({ ok: status === 200, status, text: async () => text, json: async () => JSON.parse(text) });
        if (url === '/status') return reply('mode=MAINTENANCE\nauthority=NONE\nimu_rv_count=10\n');
        if (url === '/wifi/status') return reply(JSON.stringify({ profiles: [], scan: [], active_profile: 0, sta_test: { seq: 0 }, policy: {} }));
        if (failure === 'body_timeout') return { ok: true, status: 200, json: aborted, text: aborted };
        if (failure === 'invalid_json') return reply('invalid JSON');
        if (failure === 'http_error') return reply('{"reason":"SERVICE_UNAVAILABLE"}', 503);
        if (failure === 'text403') return reply('<html>403 Forbidden ADMIN_AP_REQUIRED</html>', 403);
        if (failure === 'json503') return reply('{"version":1,"ok":false,"error":"CONTROLLER_LOOP_LATE","elapsed_ms":1201}', 503);
        if (url === '/api/calibration/restore') {
          device.restores.push({ body: JSON.parse(init.body), csrf: init.headers['X-Csrf-Token'], type: init.headers['Content-Type'] });
          return reply(JSON.stringify({ version: 1, decision: 'ALLOW_OPERATOR', reason: 'OK', restored: true, mapped_joints: 12, limits_approved: false, motion_authorized: false }));
        }
        if (url === '/api/motion/status') return reply(JSON.stringify(device.status));
        if (url === '/api/motion/capabilities') return reply(JSON.stringify(device.caps));
        if (url === '/api/motion/command') {
          const command = JSON.parse(init.body);
          if (device.holdCommand) await device.holdCommand;
          return reply(JSON.stringify({ version: 1, id: command.id, sequence: command.sequence, outcome: command.type === 'MOTION.STOP' ? 'COMPLETED' : 'ACCEPTED', reason: 'OK' }));
        }
        return reply('{}', 404);
      };
    } });
  await settle();
  const q = s => dom.window.document.querySelector(s);
  const pulse = async (advance = 0) => { clock += advance; intervals.forEach(i => i.fn()); await settle(); };
  const motionPoll = async () => { const poll = timers.find(t => t.active && (t.ms >= 50 && t.ms <= 1000) && t.fn.name === 'refreshMotion'); assert(poll); poll.active = false; poll.fn(); await settle(); };
  const fireTimer = async delay => { const t = timers.find(t => t.active && t.ms === delay); assert(t, 'active timer ' + delay); t.active = false; clock += delay; t.fn(); await settle(); };
  return { dom, device, q, calls, errors, pulse, motionPoll, fireTimer, timers, close: () => dom.window.close() };
}
(async () => {
  let p = await boot();
  assert.deepStrictEqual(p.errors, [], 'shipped scripts execute without DOM errors');
  assert.strictEqual(p.dom.window.document.querySelectorAll('nav button').length, 7);
  for (const id of ['dashboard', 'wifi', 'ap', 'performance', 'ota', 'diagnostics', 'motion-control']) {
    p.q('[data-page="' + id + '"]').click(); assert.strictEqual(p.q('#' + id).hidden, false);
  }
  await settle();
  assert.strictEqual(p.dom.window.document.querySelectorAll('[data-action]').length, 48);
  assert.strictEqual(p.dom.window.document.querySelector('img'), null, 'catalog values never create DOM markup');
  for (const button of p.dom.window.document.querySelectorAll('[data-action]')) {
    assert.strictEqual(button.disabled, !['motion.stop', 'motion.status'].includes(button.dataset.action), button.dataset.action);
  }
  assert(p.q('#motion-stand').disabled && p.q('#motion-lie').disabled && p.q('#motion-body').disabled);
  assert.strictEqual(p.q('#motion-joystick').getAttribute('aria-disabled'), 'true');
  // A synthetic DOM event cannot execute an unqualified action or velocity.
  p.q('[data-action="action.wave"]').onclick();
  p.q('[data-velocity="1,0"]').onpointerdown({ pointerId: 1 });
  p.q('#motion-yaw').value = '.1'; p.q('#motion-yaw').oninput(); await settle();
  assert.strictEqual(p.device.commands.length, 0, 'no physical semantic command while Controller unavailable');
  p.q('#motion-stop-button').click(); await settle();
  assert.strictEqual(p.device.commands[0].type, 'MOTION.STOP');
  p.close();

  p = await boot(true);
  assert(!p.q('#motion-stand').disabled);
  assert(p.q('[data-action="gait.trot"]').disabled, 'unvalidated gait remains disabled with live permit');
  p.q('#motion-stand').click(); await settle();
  const stand = p.device.commands[0];
  assert.strictEqual(stand.type, 'POSTURE.STAND'); assert.deepStrictEqual(stand.params, { target_height_m: .15 });
  assert.strictEqual(stand.version, 1); assert.strictEqual(stand.owner, 'dashboard'); assert(stand.sequence > 10);
  assert.strictEqual(stand.issued_at_ms, 1000); assert(/^[a-zA-Z0-9_.-]{1,48}$/.test(stand.id));
  const arrow = p.q('[data-velocity="1,1"]'); arrow.setPointerCapture = () => {};
  arrow.onpointerdown({ pointerId: 1 }); await settle();
  const v = p.device.commands.at(-1); assert.strictEqual(v.type, 'BASE_VELOCITY'); assert.strictEqual(v.ttl_ms, 350);
  assert(v.params.vx > 0 && v.params.vy > 0 && v.params.vx <= .03 && v.params.vy <= .02);
  assert.deepStrictEqual(Object.keys(v.params).sort(), ['vx', 'vy', 'yaw_rate']);
  arrow.onpointerup(); await settle(); assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP');
  arrow.onpointerdown({ pointerId: 2 }); await settle();
  p.dom.window.dispatchEvent(new p.dom.window.Event('blur')); await settle();
  assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP', 'focus loss stops');
  arrow.onpointerdown({ pointerId: 3 }); await settle();
  await p.pulse(1600); assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP', 'Controller status TTL stops');
  assert(p.q('#motion-stand').disabled, 'stale status disables posture');
  p.close();

  p = await boot(true);
  // Relying only on fetch completion loses a release. The UI queues STOP behind an in-flight command.
  let release; p.device.holdCommand = new Promise(resolve => { release = resolve; });
  const b = p.q('[data-velocity="1,0"]'); b.setPointerCapture = () => {};
  b.onpointerdown({ pointerId: 9 }); await settle(); b.onpointerup(); await settle();
  // Issue #52: STOP has its own lane - it is on the wire BEFORE the held command returns.
  assert.deepStrictEqual(p.device.commands.map(c => c.type), ['BASE_VELOCITY', 'MOTION.STOP'], 'STOP not queued behind the in-flight command');
  release(); p.device.holdCommand = null; await settle();
  assert.deepStrictEqual(p.device.commands.map(c => c.type), ['BASE_VELOCITY', 'MOTION.STOP']);
  p.dom.window.dispatchEvent(new p.dom.window.Event('offline')); await settle();
  assert(p.q('#motion-stand').disabled, 'connection loss disables capability');
  p.close();
  // Diagnostics separate telemetry delivery, policy, faults and capability age.
  p = await boot(true);
  p.q('#motion-control').hidden = false;
  p.device.status.now_ms += 100;
  p.device.status.readiness = { restore: false, permit: false };
  await p.motionPoll();
  let badge = p.q('#motion-fresh').textContent;
  assert(badge.startsWith('FRESH'));
  assert(badge.includes('RESTORE_REQUIRED') && badge.includes('MOTION_NOT_PERMITTED'));
  assert(!badge.includes('TELEMETRY_STALE'), 'policy refusal does not invent stale telemetry');
  p.device.status.now_ms += 100;
  p.device.status.readiness = { restore: true, permit: true };
  p.device.status.fault = true;
  await p.motionPoll();
  badge = p.q('#motion-fresh').textContent;
  assert(badge.includes('MOTION_FAULT') && !badge.includes('TELEMETRY_STALE'));
  assert(p.q('#motion-stand').disabled, 'fresh fault still prevents action');
  p.close();

  p = await boot(true);
  p.q('#motion-control').hidden = false;
  p.device.status.now_ms += 100;
  p.device.status.reason = 'CHANGED';
  p.device.failures['/api/motion/capabilities'] = ['network', 'network'];
  await p.motionPoll();
  await p.fireTimer(250); // The one bounded GET retry.
  await p.pulse();
  badge = p.q('#motion-fresh').textContent;
  assert(badge.includes('CAPABILITIES_STALE') && badge.includes('NETWORK_ERROR'));
  assert(!badge.includes('TELEMETRY_STALE'), 'capability failure does not invalidate a fresh sample');
  assert(p.q('#motion-stand').disabled);
  assert.strictEqual(p.calls.filter(c => c.url === '/api/motion/capabilities').length, 3, 'one initial GET and exactly two recovery attempts');
  p.close();

  p = await boot(true);
  p.q('#motion-control').hidden = false;
  p.device.failures['/api/motion/status'] = ['timeout', 'body_timeout'];
  await p.motionPoll();
  await p.fireTimer(2000);
  await p.fireTimer(250);
  await p.fireTimer(2000);
  await p.pulse();
  badge = p.q('#motion-fresh').textContent;
  assert(badge.includes('HTTP_TIMEOUT') && badge.includes('TELEMETRY_STALE'));
  assert(p.q('#motion-state').textContent.includes('scaduti 2'));
  assert.strictEqual(p.calls.filter(c => c.url === '/api/motion/status').length, 3, 'read timeout retries once, including stalled JSON body');
  p.q('#motion-stop-button').click(); await settle(); await p.pulse();
  assert(p.q('#motion-fresh').textContent.includes('HTTP_TIMEOUT'), 'a successful STOP cannot erase the failed status diagnosis');
  p.device.status.now_ms += 100;
  await p.motionPoll();
  assert(!p.q('#motion-fresh').textContent.includes('HTTP_TIMEOUT'), 'fresh status clears its own timeout');
  p.close();

  p = await boot(true);
  p.device.failures['/api/motion/command'] = ['timeout'];
  p.q('#motion-stand').click(); await settle();
  await p.fireTimer(2000);
  assert.strictEqual(p.device.commands.length, 1, 'unconfirmed POST is never replayed');
  assert(p.q('#motion-stand').disabled, 'unconfirmed command revokes cached capability');
  assert(p.q('#motion-result').textContent.includes('HTTP_TIMEOUT'));
  assert(!p.timers.some(t => t.active && t.ms === 250), 'no command retry is scheduled');
  p.close();

  for (const [failure, code] of [['invalid_json', 'HTTP_INVALID_JSON'], ['http_error', 'HTTP_503']]) {
    p = await boot(true);
    p.device.failures['/api/motion/status'] = [failure];
    await p.motionPoll();
    assert(p.q('#motion-fresh').textContent.includes(code));
    assert.strictEqual(p.calls.filter(c => c.url === '/api/motion/status').length, 2, 'non-transient response does not retry');
    p.close();
  }

  p = await boot(true);
  p.device.status.now_ms += 100;
  p.device.status.commissioning = { mode: 'FIXTURE_COMMISSIONING', limits: 'COMMISSIONING', commissioning_joints: 12, approved_joints: 0, entry: 'ENTRY_DISENGAGE', phase: 'SEGMENT_GRANTED', prime_clamp_mask: 4095 };
  p.device.status.telemetry = { source: 'HARDWARE_SERVO_FEEDBACK', actual_rad: null, target_rad: null };
  await p.motionPoll();
  const stateText = p.q('#motion-state').textContent;
  assert(stateText.includes('12/12 · COMMISSIONING') && stateText.includes('PRIME clamp mask4095'));
  assert(stateText.includes('ENTRY_DISENGAGE / SEGMENT_GRANTED'));
  assert(stateText.includes('12 limitiUNAPPROVED'), 'RAM commissioning count never claims formally approved joints');
  assert(stateText.includes('Encoder actualUNKNOWN'), 'missing actual joints are never rendered as twelve zeroes');
  p.close();

  p = await boot(true);
  p.q('#motion-control').hidden = false;
  const rebootArrow = p.q('[data-velocity="1,0"]'); rebootArrow.setPointerCapture = () => {};
  rebootArrow.onpointerdown({ pointerId: 20 }); await settle();
  p.device.status.now_ms = 1; // A new boot cannot retain a held velocity.
  await p.motionPoll();
  assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP');
  assert.strictEqual(p.device.commands.at(-1).issued_at_ms, 1, 'STOP uses the new Controller clock');
  const countAfterReboot = p.device.commands.length;
  await p.pulse(120);
  assert.strictEqual(p.device.commands.length, countAfterReboot, 'held input is never resumed after reboot');
  p.close();

  p = await boot(true);
  p.q('#motion-control').hidden = false;
  p.device.status.now_ms += 100;
  p.device.status.readiness.permit = false;
  p.device.failures['/api/motion/capabilities'] = ['timeout', 'timeout'];
  await p.motionPoll();
  assert(p.q('#motion-stand').disabled, 'changed Controller gate invalidates old capabilities before refresh completes');
  await p.fireTimer(2000); await p.fireTimer(250); await p.fireTimer(2000);
  p.close();

  p = await boot(true, { rtt: { '/api/motion/status': 80, '/api/motion/capabilities': 40 } });
  assert(p.q('#motion-state').textContent.includes('RTT 40 ms, max 80 ms, media 60 ms'), 'RTT is observed, not a preset');
  assert(p.q('#motion-state').textContent.includes('timeout 2000 ms'), 'default read timeout exceeds the 1200 ms server read budget');
  p.close();
  p = await boot(false, { url: 'http://192.168.4.1/?motion_timeout_ms=1000' });
  assert(p.q('#motion-state').textContent.includes('timeout 1000 ms'), 'bounded browser override preserved');
  p.close();
  p = await boot(false, { url: 'http://192.168.4.1/?motion_timeout_ms=99999' });
  assert(p.q('#motion-state').textContent.includes('timeout 2000 ms'), 'out-of-bounds timeout rejected');
  p.close();

  p = await boot();
  const input = p.q('#motion-twin-url'), link = p.q('#motion-twin');
  assert.strictEqual(link.getAttribute('aria-disabled'), 'true');
  for (const value of ['http://localhost:8765/', 'http://localhost.:8765/', 'http://127.1:8765/', 'http://[::1]:8765/', 'http://[::ffff:127.0.0.1]:8765/', 'http://0.0.0.0:8765/', 'javascript:alert(1)', 'http://192.168.1.40:8765/', 'http://192.168.1.149:9999/', 'http://user:secret@192.168.1.40:8765/']) {
    input.value = value; input.dispatchEvent(new p.dom.window.Event('change'));
    assert.strictEqual(link.getAttribute('aria-disabled'), 'true', value);
    assert.strictEqual(p.dom.window.localStorage.getItem('matdog.twin.url'), null);
  }
  input.value = 'http://192.168.1.149:8765/'; input.dispatchEvent(new p.dom.window.Event('change'));
  assert.strictEqual(link.href, 'http://192.168.1.149:8765/');
  assert.strictEqual(link.getAttribute('aria-disabled'), 'false');
  assert.strictEqual(p.dom.window.localStorage.getItem('matdog.twin.url'), link.href);
  assert(!link.textContent.includes('LIVE'), 'configured link alone is not evidence of a hardware LIVE feed');
  p.close();

  p=await boot(true);
  p.device.status.readiness.permit=false;p.device.status.now_ms+=100;await p.motionPoll();
  assert(p.q('#motion-stand').disabled,'current permit=false wins over available=true capability');
  assert(p.device.caps.actions.find(a=>a.id==='posture.stand').available,'fixture retains cached available=true');
  p.close();

  // --- 14 Oct regressions -----------------------------------------------
  for (const [failure, code, text] of [['text403', 'HTTP_403', '403 Forbidden ADMIN_AP_REQUIRED'], ['json503', 'HTTP_503', 'CONTROLLER_LOOP_LATE']]) {
    p = await boot(true);
    p.device.failures['/api/motion/status'] = [failure];
    await p.motionPoll();
    const badge14 = p.q('#motion-fresh').textContent;
    assert(badge14.includes(code) && !badge14.includes('HTTP_INVALID_JSON'), failure + ': real status, not a JSON-parse diagnosis');
    assert(p.q('#motion-result').textContent.includes(text), failure + ': server text/code is shown');
    p.close();
  }
  p = await boot(true);
  p.q('#motion-control').hidden = false;
  p.device.failures['/api/motion/status'] = ['timeout'];
  await p.motionPoll();                              // read in flight, stalled
  const portalPoll = p.timers.find(t => t.active && t.ms === 4000 && t.fn.name === 'refresh');
  assert(portalPoll, 'portal poll timer');
  const before = p.calls.length; portalPoll.active = false; portalPoll.fn(); await settle();
  assert.strictEqual(p.calls.length, before, 'portal read waits for the in-flight Motion read (single-flight lane)');
  p.q('#motion-stop-button').click(); await settle();
  assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP', 'STOP is sent while a read is stalled: it never queues behind reads');
  await p.fireTimer(2000);                           // read aborts -> lane frees
  await settle();
  assert(p.calls.slice(before).some(c => c.url === '/status'), 'queued portal read runs after the stalled read');
  assert(p.device.maxReadInflight <= 1, 'never two concurrent reads from one page');
  p.close();

  p = await boot(true);
  p.device.status.now_ms += 100;
  p.device.status.readiness.restore = false;
  p.device.status.restore = { decision: 'PENDING_OPERATOR_REVIEW', reason: 'INSTALLATION_ATTESTATION_ABSENT', trigger: 'BOOT', generation: 1, fingerprint: '9d5323b7d59bb080' };
  p.device.status.channels = { imu: { valid: true, age_ms: 8 }, raw: { count: 12, max_age_ms: 352 }, mapped: { count: 0, mask: 0 } };
  await p.motionPoll();
  const restoreText = p.q('#motion-restore').textContent;
  assert(restoreText.includes('PENDING_OPERATOR_REVIEW') && restoreText.includes('INSTALLATION_ATTESTATION_ABSENT') && restoreText.includes('9d5323b7d59bb080'));
  assert(restoreText.includes('Giunti mappati0/12') && restoreText.includes('IMUvalida · 8 ms') && restoreText.includes('Encoder raw12/12'));
  const button = p.q('#motion-restore-button'), confirm = p.q('#motion-restore-confirm');
  assert(button.disabled, 'no admin session: disabled');
  p.dom.window.eval("csrf='c'.repeat(64)");
  confirm.checked = false; confirm.onchange(); assert(button.disabled, 'no explicit confirmation: disabled');
  confirm.checked = true; confirm.onchange(); assert(!button.disabled, 'admin + per-attempt confirmation: enabled');
  button.onclick(); await settle();
  assert.strictEqual(p.device.restores.length, 1);
  assert.deepStrictEqual(p.device.restores[0].body, { generation: 1, confirm: 'CONFIRM_CURRENT_INSTALLATION' });
  assert.strictEqual(p.device.restores[0].csrf, 'c'.repeat(64));
  assert.strictEqual(p.device.restores[0].type, 'application/json');
  assert(!confirm.checked, 'confirmation is consumed by one attempt');
  assert(p.q('#motion-result').textContent.includes('"motion_authorized": false'));
  assert.strictEqual(p.device.commands.length, 0, 'RESTORE sends no Motion command');
  p.device.status.readiness.restore = true; p.device.status.now_ms += 100;
  await p.motionPoll();
  confirm.checked = true; confirm.onchange();
  assert(button.disabled && button.textContent.includes('già attivo'), 'idempotent UI once restored');
  p.close();

  // --- Operational READY (directive 2026-10-14) -----------------------------
  p = await boot(true);
  p.device.status.now_ms += 100;
  p.device.status.operational = { system_health: 'READY', calibration: 'RESTORED', joint_feedback: 'AVAILABLE', motion_authority: 'GRANTED', phase: 'READY', reason: 'OK', mode: 'RUN', entry_reason: '', operator_present: true };
  await p.motionPoll();
  const opText = p.q('#motion-state').textContent;
  assert(opText.includes('SistemaREADY') && opText.includes('CalibrazioneRESTORED') && opText.includes('Feedback giuntiAVAILABLE') && opText.includes('Motion AuthorityGRANTED'));
  assert(opText.includes('Operatore (heartbeat)presente'));
  assert(!p.q('#motion-stand').disabled, 'READY + Controller-available STAND: no extra consent step in the UI');
  assert(p.q('[data-action="posture.lie_down"]').disabled, 'unimplemented action stays disabled');
  assert(p.q('[data-action="posture.lie_down"]').closest('.motion-action').textContent.includes('planned'), 'unimplemented action is labelled as such');
  assert(p.q('header #motion-stop-button'),'STOP visible on every tab');
  assert(p.q('#dashboard-twin-card')&&p.q('#dashboard-twin-frame'), 'Twin integrated in initial Dashboard');
  assert.strictEqual(p.q('nav').children[1].dataset.page,'motion-control','Motion is second tab');
  assert(p.q('#motion-restore-card').hidden,'automatic RESTORE hides repeated confirmation');
  p.q('#motion-stand').click(); await settle();
  assert.strictEqual(p.device.commands.at(-1).type, 'POSTURE.STAND');
  // Deadman proof (2026-10-15): status reads carry the admin CSRF token, so the
  // controller can tell the Dashboard (which holds STOP) from a read-only Twin.
  p.dom.window.eval("csrf='d'.repeat(64);sessionAt=Date.now()"); p.device.status.now_ms += 100;
  await p.motionPoll();
  const read = p.calls.filter(c => c.url === '/api/motion/status').at(-1);
  assert.strictEqual(read.csrf, 'd'.repeat(64), 'status poll presents the CSRF heartbeat proof');
  assert(p.q('#motion-stand').closest('.card').textContent.includes('Solo con MATDOG sospeso sulla fixture'), 'fixture-only SAFE_OFF warning is visible');
  p.close();
  // --- Continuous WALK (2026-10-15, Issue #52): START only from verified STAND, STOP never queued ---
  p = await boot(true);
  p.device.caps.walk = { period_s: 7, lift_m: 0.015, strides_m: [0.02, 0.03, 0.04], fast: { period_s: 3, lift_m: 0.010, stride_m: 0.06, experimental: true } };
  p.device.status.now_ms += 100;
  p.device.status.operational = { system_health: 'READY', calibration: 'RESTORED', joint_feedback: 'AVAILABLE', motion_authority: 'GRANTED', phase: 'READY', reason: 'OK', mode: 'RUN', entry_reason: '', operator_present: true, walk: 'WALK_BOUND' };
  await p.motionPoll();
  assert(p.q('#motion-walk-start').disabled, 'no WALK before a verified STAND');
  p.device.status.stand_state = 'STAND'; p.device.status.reason = 'STAND_VERIFIED'; p.device.status.now_ms += 100;
  await p.motionPoll();
  assert(!p.q('#motion-walk-start').disabled, 'verified STAND + bound walk: WALK START available');
  const walkBefore = p.device.commands.length;
  p.q('#motion-walk-start').click(); await settle();
  assert.strictEqual(p.device.commands.length, walkBefore + 1, 'exactly one command per click');
  const start = p.device.commands.at(-1);
  assert.strictEqual(start.type, 'BASE_VELOCITY'); assert.strictEqual(start.params.vx, 0.03 / 7, 'preset stride / period, exact');
  assert.strictEqual(start.params.vy, 0); assert.strictEqual(start.params.yaw_rate, 0);
  assert.strictEqual(p.q('#motion-walk-stride').value, '1', 'default stays the 30 mm STANDARD preset, never the experimental FAST');
  assert(p.q('#motion-walk-stride option[value="3"]').textContent.includes('SPERIMENTALE'), 'FAST is labelled experimental');
  p.device.status.operational.walk = 'WALK'; p.device.status.now_ms += 100; await p.motionPoll();
  p.q('#motion-walk-stop').click(); await settle();
  const wstop = p.device.commands.at(-1);
  assert(wstop.type === 'BASE_VELOCITY' && wstop.params.vx === 0 && wstop.params.vy === 0 && wstop.params.yaw_rate === 0, 'WALK STOP = coordinated zero velocity');
  // Red STOP while a command is held by a slow Controller: sent immediately on its own lane.
  let releaseWalk; p.device.holdCommand = new Promise(resolve => { releaseWalk = resolve; });
  p.q('#motion-walk-stop').click(); await settle();
  const n0 = p.device.commands.length;
  p.q('#motion-stop-button').click(); await settle();
  assert.strictEqual(p.device.commands.length, n0 + 1, 'emergency STOP is not queued behind a pending command');
  assert.strictEqual(p.device.commands.at(-1).type, 'MOTION.STOP');
  releaseWalk(); p.device.holdCommand = null; await settle();
  // Explicit FAST selection: exact 0.06 m / 3 s, never rounded.
  p.device.status.operational.walk = 'WALK_BOUND'; p.device.status.stand_state = 'STAND'; p.device.status.now_ms += 100; await p.motionPoll();
  p.q('#motion-walk-stride').value = '3'; p.q('#motion-walk-start').click(); await settle();
  assert.strictEqual(p.device.commands.at(-1).params.vx, 0.06 / 3, 'FAST preset speed exact');
  p.device.status.operational.walk = 'WALK_RANGE_OUTSIDE_LIMITS'; p.device.status.stand_state = 'STAND'; p.device.status.now_ms += 100; await p.motionPoll();
  assert(p.q('#motion-walk-start').disabled, 'unbound walk keeps WALK START disabled');
  p.close();
  assert(!/getWriter|\.writable|\/servo\/|grantAuthority|TORQUE_ON|writeRegister/.test(motion));
  console.log('MOTION_DASHBOARD_DOM = PASS (48 actions, 6 preserved tabs, semantic schema, disabled gates, TTL, release, focus loss, in-flight STOP, split diagnoses, measured RTT, bounded read retry, no POST replay, LAN Twin URL, HTTP status-before-JSON, single-flight reads, STOP lane, RESTORE gate, operational READY; FIXTURE_ONLY; WALK START/STOP; FAST experimental opt-in; STOP not queued behind a pending command)');
})().catch(error => { console.error(error); process.exitCode = 1; });
