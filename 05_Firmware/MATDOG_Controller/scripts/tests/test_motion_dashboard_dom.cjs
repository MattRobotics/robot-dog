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
async function boot(enabled = false) {
  let clock = 1000;
  const device = { down: false, holdCommand: null, commands: [], caps: { version: 1, hardware_approved: enabled, teleoperation: enabled,
    limits: { vx_m_s: .03, vy_m_s: .02, yaw_rate_rad_s: .15 }, actions: actions.map(a => ({ ...a, label: a.id === 'action.wave' ? '<img src=x onerror=alert(1)>' : a.label, available: enabled && a.implemented || a.id.startsWith('motion.') })) },
    status: { version: 1, now_ms: 1000, last_sequence: 10, executor_state: 'OFF', stand_state: 'OBSERVE', stale: false, fault: false, owner: 'none' } };
  const intervals = [], timers = [], errors = [], calls = [];
  const dom = new JSDOM(html, { url: 'http://192.168.4.1/', runScripts: 'dangerously', pretendToBeVisual: true,
    beforeParse(w) {
      Object.defineProperty(w.performance, 'now', { value: () => clock });
      w.setTimeout = (fn, ms) => { timers.push({ fn, ms }); return timers.length; }; w.clearTimeout = () => {};
      w.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
      w.addEventListener('error', e => errors.push(e.message));
      w.fetch = async (url, init = {}) => {
        calls.push({ url, method: init.method || 'GET' });
        if (device.down) throw Error('fixture connection lost');
        const reply = (text, status = 200) => ({ ok: status === 200, status, text: async () => text, json: async () => JSON.parse(text) });
        if (url === '/status') return reply('mode=MAINTENANCE\nauthority=NONE\nimu_rv_count=10\n');
        if (url === '/wifi/status') return reply(JSON.stringify({ profiles: [], scan: [], active_profile: 0, sta_test: { seq: 0 }, policy: {} }));
        if (url === '/api/motion/status') return reply(JSON.stringify(device.status));
        if (url === '/api/motion/capabilities') return reply(JSON.stringify(device.caps));
        if (url === '/api/motion/command') {
          const command = JSON.parse(init.body); device.commands.push(command);
          if (device.holdCommand) await device.holdCommand;
          return reply(JSON.stringify({ version: 1, id: command.id, sequence: command.sequence, outcome: command.type === 'MOTION.STOP' ? 'COMPLETED' : 'ACCEPTED', reason: 'OK' }));
        }
        return reply('{}', 404);
      };
    } });
  await settle();
  const q = s => dom.window.document.querySelector(s);
  const pulse = async (advance = 0) => { clock += advance; intervals.forEach(i => i.fn()); await settle(); };
  const motionPoll = async () => { const poll = timers.filter(t => t.ms === 1000 || t.ms === 4000).find(t => t.fn.name === 'refreshMotion'); assert(poll); poll.fn(); await settle(); };
  return { dom, device, q, calls, errors, pulse, motionPoll, close: () => dom.window.close() };
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
  b.onpointerdown({ pointerId: 9 }); await settle(); b.onpointerup();
  release(); p.device.holdCommand = null; await settle();
  assert.deepStrictEqual(p.device.commands.map(c => c.type), ['BASE_VELOCITY', 'MOTION.STOP']);
  p.dom.window.dispatchEvent(new p.dom.window.Event('offline')); await settle();
  assert(p.q('#motion-stand').disabled, 'connection loss disables capability');
  p.close();
  assert(!/getWriter|\.writable|\/servo\/|grantAuthority|TORQUE_ON|writeRegister/.test(motion));
  console.log('MOTION_DASHBOARD_DOM = PASS (48 actions, 6 preserved tabs, semantic schema, disabled gates, TTL, release, focus loss, in-flight STOP)');
})().catch(error => { console.error(error); process.exitCode = 1; });
