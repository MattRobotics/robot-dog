// Executable DOM check of the SHIPPED dashboard's WALK selector and joystick
// (2026-11-10). Controller bodies are the real MotionService fixtures
// (test_simple_dashboard_contract fixtures, capabilities included); a small
// Controller model answers like MotionService/SimpleMotion: a selection takes
// effect only when ACCEPTED (refused while STOP is latched), a joystick demand
// is ACCEPTED only inside the ellipsoid of the Controller's selected mode
// (vNext: lateral, yaw and backward of that mode's capabilities joystick
// limits), otherwise REJECTED. vNext audit: a backward walk keeps its own law's
// limits after a selection (ACK WALK_MODE_NEXT_WALK, status walk_mode.joystick);
// the page keeps the held stick on them. Same harness and dependency
// (jsdom) as test_simple_dashboard_dom.cjs. No device, no network.
const { JSDOM } = require('jsdom');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const [fixturesPath] = process.argv.slice(2);
assert(fixturesPath, 'usage: test_joystick_mode_dom.cjs FIXTURES.json');
const src = path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
const caps = JSON.parse(fx.capabilities);
const top = Object.fromEntries(caps.walk_modes.map(m => [m.name, m.top_speed_mm_s / 1000]));
const lim = Object.fromEntries(caps.walk_modes.map(m => [m.name, m.joystick]));  // vNext: per-mode joystick limits
assert(['SLOW', 'STANDARD', 'FAST'].every(m => lim[m] && lim[m].vy_max_m_s > 0 && lim[m].yaw_rate_max_rad_s > 0 && lim[m].vx_back_max_m_s >= 0), 'every mode publishes its joystick limits');
assert(['SLOW', 'STANDARD', 'FAST'].every(m => lim[m].vx_back_max_m_s > 0), 'backward on every law (vNext audit: STANDARD too)');
assert(lim.SLOW.vy_max_m_s === caps.joystick.vy_max_m_s && lim.SLOW.yaw_rate_max_rad_s === caps.joystick.yaw_rate_max_rad_s && lim.SLOW.vx_back_max_m_s === caps.joystick.vx_back_max_m_s, 'SLOW limits equal the dev.6 joystick object');
const CSRF = 'c'.repeat(64);
const settle = async () => { for (let i = 0; i < 30; i++) await new Promise(r => setImmediate(r)); };
const withAck = (body, outcome, reason) => { const a = JSON.parse(body); a.outcome = outcome; a.reason = reason; return JSON.stringify(a); };

async function boot() {
  let clock = 1000;
  const timers = [], intervals = [], calls = [];
  // The Controller model. holdNext keeps the next command pending until release().
  // law: the walk's mode; a backward walk keeps it (joystickMode) until the walk ends.
  const ctl = { selected: 'SLOW', law: 'SLOW', backward: false, walking: false, stop: false, rejected: 0, lastJoystick: 0, maxGap: 0, holdNext: false, held: [] };
  const held = () => ctl.walking && ctl.backward;
  const joyMode = () => held() ? ctl.law : ctl.selected;
  const statusBody = () => { const s = JSON.parse(ctl.stop ? fx.status_stopped_on : ctl.walking ? fx.status_joystick_walking : fx.status_ready);
    s.walk_mode = Object.assign({}, s.walk_mode, { selected: ctl.selected, target: held() ? ctl.law : ctl.selected, active: held() ? ctl.law : ctl.selected });
    if (joyMode() !== ctl.selected) s.walk_mode.joystick = joyMode(); else delete s.walk_mode.joystick; return JSON.stringify(s); };
  const answer = body => {
    const c = JSON.parse(body), p = c.params;
    if (c.type === 'MOTION.STOP') { ctl.stop = true; ctl.walking = false; return fx.ack_stop; }
    if (c.type === 'GAIT.SELECT') { if (ctl.stop) return fx.ack_gait_select_latched; ctl.selected = p.gait; return held() ? withAck(fx.ack_gait_select, 'ACCEPTED', 'WALK_MODE_NEXT_WALK') : fx.ack_gait_select; }
    if (c.type === 'BASE_VELOCITY' && p.gait === 'JOYSTICK') {
      if (ctl.stop) return withAck(fx.ack_walk_accepted, 'REJECTED', 'STOP_LATCHED');  // SimpleMotion::drive while stopping
      if (!p.vx && !p.vy && !p.yaw_rate) { ctl.walking = false; ctl.backward = false; return fx.ack_walk_stop; }
      const j = lim[joyMode()], r = [top[joyMode()], j.vy_max_m_s, j.yaw_rate_max_rad_s];
      const norm = (p.vx / r[0]) ** 2 + (p.vy / r[1]) ** 2 + (p.yaw_rate / r[2]) ** 2;
      if (norm > 1 + 1e-9 || -p.vx > j.vx_back_max_m_s + 1e-12) { ++ctl.rejected; return withAck(fx.ack_walk_accepted, 'REJECTED', 'JOYSTICK_VELOCITY_RANGE'); }
      if (ctl.walking && ctl.lastJoystick) ctl.maxGap = Math.max(ctl.maxGap, clock - ctl.lastJoystick);
      if (!ctl.walking) { ctl.law = ctl.selected; ctl.backward = p.vx < 0; }
      ctl.walking = true; ctl.lastJoystick = clock; return fx.ack_walk_accepted;
    }
    return fx.ack_rejected;
  };
  const dom = new JSDOM(html, { url: 'http://192.168.4.1/', runScripts: 'dangerously', pretendToBeVisual: true,
    beforeParse(w) {
      w.localStorage.setItem('md_login', '1');
      Object.defineProperty(w.performance, 'now', { value: () => clock });
      w.Date.now = () => clock;
      w.setTimeout = (fn, ms) => { timers.push({ fn, ms, active: true }); return timers.length; };
      w.clearTimeout = id => { if (timers[id - 1]) timers[id - 1].active = false; };
      w.setInterval = (fn, ms) => { intervals.push({ fn, ms }); return intervals.length; };
      w.fetch = async (url, init = {}) => {
        calls.push({ url, body: init.body, at: clock });
        const reply = (text, status = 200) => ({ ok: status >= 200 && status < 300, status, text: async () => text, json: async () => JSON.parse(text) });
        if (url === '/wifi/session') return reply(CSRF);
        if (url === '/status') return reply('mode=RUN\nauthority=MOTION\npower_state=RUN\nsystem_health=READY\n');
        if (url === '/wifi/status') return reply(JSON.stringify({ profiles: [], scan: [], active_profile: 0, sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
        if (url === '/api/motion/capabilities') return reply(fx.capabilities);
        if (url === '/api/motion/status') return reply(statusBody());
        if (url === '/api/motion/command') {
          if (ctl.holdNext) { ctl.holdNext = false; await new Promise(r => ctl.held.push(r)); }
          const a = JSON.parse(answer(init.body)), c = JSON.parse(init.body); a.id = c.id; a.sequence = c.sequence;
          return reply(JSON.stringify(a));
        }
        return reply('{}', 404);
      };
    } });
  await settle();
  const q = s => dom.window.document.querySelector(s);
  const poll = async () => { clock += 350; intervals.filter(i => i.ms === 350).forEach(i => i.fn()); await settle(); };
  const tick = async () => { clock += 240; intervals.filter(i => i.ms === 240).at(-1)?.fn(); await settle(); };
  const fire20 = async (n = 1) => { for (let k = 0; k < n; k++) { for (const t of timers.filter(t => t.active && t.ms === 20)) { t.active = false; clock += 20; t.fn(); } await settle(); } };
  const release = async () => { ctl.held.splice(0).forEach(r => r()); await settle(); };
  return { dom, ctl, q, calls, poll, tick, fire20, release };
}

(async () => {
  const p = await boot();
  const { q, ctl } = p;
  const pad = q('#joy-pad');
  const pointer = (type, f, l) => { const ev = new p.dom.window.MouseEvent(type, { bubbles: true, cancelable: true, clientX: 90 - 58 * l, clientY: 90 - 58 * f }); Object.defineProperty(ev, 'pointerId', { value: 1 }); pad.dispatchEvent(ev); };
  const cmds = () => p.calls.filter(c => c.url === '/api/motion/command').map(c => Object.assign(JSON.parse(c.body), { at: c.at }));
  const last = () => cmds().at(-1);
  const radio = () => q('input[name="simple-walk-mode"]:checked').value;
  const click = mode => q(`input[name="simple-walk-mode"][value="${mode}"]`).click();
  // Stick deflection f (dead zone .12 rescaled) -> fraction of the full scale, as the page computes it.
  const fraction = f => { const r = Math.abs(f), k = r < .12 ? 0 : Math.min(1, (r - .12) / (1 - .12)) / r; return f * k; };
  await p.poll();

  // 1. Every mode: payload, 0 / 50 / 100 %, supported components, centre.
  for (const mode of ['SLOW', 'STANDARD', 'FAST']) {
    if (mode !== 'SLOW') { click(mode); await settle(); await p.poll(); }
    assert.strictEqual(ctl.selected, mode); assert.strictEqual(radio(), mode);
    const n0 = cmds().length;
    pointer('pointerdown', .05, 0); await settle(); await p.tick();
    assert.strictEqual(cmds().length, n0, mode + ': 0 % (dead zone) sends nothing');
    pointer('pointermove', 1, 0); await p.tick();
    assert.deepStrictEqual([last().type, last().params], ['BASE_VELOCITY', { vx: top[mode], vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }], mode + ': 100 % = the mode\'s top speed');
    await p.poll();
    pointer('pointermove', .5, 0); await p.tick();
    assert(Math.abs(last().params.vx - fraction(.5) * top[mode]) < 1e-12, mode + ': 50 % of the stick = ' + fraction(.5).toFixed(3) + ' of full scale');
    pointer('pointermove', 0, .6); await p.tick();
    assert(last().params.vy > 0 && last().params.vx === 0 && Math.abs(last().params.vy - fraction(.6) * lim[mode].vy_max_m_s) < 1e-12, mode + ': lateral sent, proportional to the mode\'s own limit');
    pointer('pointermove', 0, 1); await p.tick();
    assert(last().params.vy === lim[mode].vy_max_m_s, mode + ': full lateral = the mode\'s lateral limit');
    pointer('pointermove', 1, 0); await p.tick(); await p.poll();
    pointer('pointermove', -.6, 0); await p.tick();
    if (lim[mode].vx_back_max_m_s > 0) assert(last().params.vx < 0 && Math.abs(last().params.vx + fraction(.6) * lim[mode].vx_back_max_m_s) < 1e-12, mode + ': backward sent, proportional to the mode\'s back limit');
    else assert.deepStrictEqual(last().params, { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, mode + ': no backward on this law, neutral (WALK STOP)');
    pointer('pointermove', .7071, -.7071); await p.tick();
    assert(last().params.vx > 0 && last().params.vy < 0, mode + ': diagonal forward-right sent');
    pointer('pointermove', 1, 0); await p.tick(); await p.poll();
    pointer('pointermove', 0, 0); await p.tick();
    assert.deepStrictEqual(last().params, { vx: 0, vy: 0, yaw_rate: 0, gait: 'JOYSTICK' }, mode + ': back to centre is neutral (WALK STOP)');
    pointer('pointerup', 0, 0); await settle(); await p.poll();
  }
  assert.strictEqual(ctl.rejected, 0, 'every demand inside the Controller\'s range');

  // 2. Stick held at 100 % in SLOW, a refresh in flight, selector to FAST: the
  //    selection waits for it, then goes; no refresh while it waits; the next
  //    refresh at once, on FAST full scale; WALK never interrupted.
  click('SLOW'); await settle(); await p.poll();
  pointer('pointerdown', 1, 0); await settle(); await p.tick(); await p.poll();
  ctl.holdNext = true; await p.tick();
  const mark = cmds().length; click('FAST'); await settle();
  assert.strictEqual(cmds().length, mark, 'the selection waits for the refresh in flight (nothing else sent)');
  await p.release(); await p.fire20(2);
  const change = cmds().slice(mark).map(c => c.type + ':' + (c.params.gait === 'JOYSTICK' ? c.params.vx : c.params.gait));
  assert.deepStrictEqual(change, ['GAIT.SELECT:FAST', 'BASE_VELOCITY:' + top.FAST], 'held stick: GAIT.SELECT FAST, then at once the FAST full scale (' + change.join(', ') + ')');
  const changeGap = ctl.maxGap;  // a request stuck past 1 s (section 3) ends the walk at the Controller's deadline, as before
  assert(ctl.selected === 'FAST' && ctl.walking && changeGap < 1000, 'WALK kept: accepted refreshes ' + changeGap + ' ms apart (< 1 s deadline)');
  await p.poll(); assert.strictEqual(radio(), 'FAST');

  // 3. A request still in flight after 1 s: an explicit refusal, no silent drop;
  //    the selector returns to the Controller's mode, refreshes keep its scale.
  ctl.holdNext = true; await p.tick();
  const mark3 = cmds().length; click('STANDARD'); await settle();
  await p.fire20(51);
  assert(/Modalità WALK STANDARD NON inviata/.test(q('#message').textContent) && !q('#message').hidden, 'timeout: explicit message (' + q('#message').textContent + ')');
  assert(cmds().slice(mark3).every(c => c.type !== 'GAIT.SELECT'), 'timeout: no GAIT.SELECT sent');
  await p.release(); await p.poll();
  assert(radio() === 'FAST' && ctl.selected === 'FAST', 'timeout: the selector back on the Controller\'s FAST');
  await p.tick(); assert(last().type === 'BASE_VELOCITY' && last().params.vx === top.FAST, 'timeout: refreshes on the accepted FAST scale');

  // 4. Priority: with a selection waiting, the release's neutral and STOP go at once.
  ctl.holdNext = true; await p.tick(); const mark4 = cmds().length;
  click('SLOW'); await settle();
  pointer('pointerup', 0, 0); await settle();
  assert.deepStrictEqual(cmds().slice(mark4).map(c => c.type + ':' + c.params.vx), ['BASE_VELOCITY:0'], 'release: neutral at once, ahead of the waiting selection');
  await p.release(); await p.fire20(2); await p.poll();
  assert(ctl.selected === 'SLOW' && radio() === 'SLOW', 'the selection still reaches the Controller after the release');
  pointer('pointerdown', 1, 0); await settle(); await p.tick(); await p.poll();
  ctl.holdNext = true; await p.tick(); const mark5 = cmds().length;
  click('FAST'); await settle();
  q('#simple-stop').click(); await settle();
  assert.deepStrictEqual(cmds().slice(mark5).map(c => c.type), ['MOTION.STOP'], 'STOP at once, ahead of the waiting selection');
  await p.release(); await p.fire20(2); await p.poll();
  assert(ctl.selected === 'SLOW' && radio() === 'SLOW', 'after STOP the Controller refuses the selection; the selector stays on its SLOW');

  // 5. Repeated selections with the stick held: sent in order, one at a time,
  //    no refresh in between; dashboard, joystick scale and Controller agree.
  ctl.stop = false; pointer('pointerup', 0, 0); await settle(); await p.poll();
  pointer('pointerdown', 1, 0); await settle(); await p.tick(); await p.poll();
  ctl.holdNext = true; await p.tick(); const mark6 = cmds().length;
  for (const m of ['FAST', 'STANDARD', 'FAST', 'SLOW']) { click(m); await settle(); }
  assert.strictEqual(cmds().length, mark6, 'repeated: nothing sent while the refresh is in flight');
  await p.release(); await p.fire20(4);
  const seq = cmds().slice(mark6).map(c => c.type + ':' + (c.params.gait === 'JOYSTICK' ? c.params.vx : c.params.gait));
  assert.deepStrictEqual(seq, ['GAIT.SELECT:FAST', 'GAIT.SELECT:STANDARD', 'GAIT.SELECT:FAST', 'GAIT.SELECT:SLOW', 'BASE_VELOCITY:' + top.SLOW], 'repeated: in order, no refresh in between (' + seq.join(', ') + ')');
  await p.poll();
  assert(ctl.selected === 'SLOW' && radio() === 'SLOW', 'repeated: Controller and selector agree on the last one');
  assert.strictEqual(ctl.rejected, 0, 'no joystick demand ever beyond the Controller\'s accepted mode');
  pointer('pointerup', 0, 0); await settle(); await p.poll();

  // 6. vNext audit: FAST backward held, STANDARD selected: the Controller defers it
  //    (WALK_MODE_NEXT_WALK) and keeps FAST's limits (status walk_mode.joystick);
  //    the held stick stays on FAST's backward limit, never neutral, before and
  //    after the next status; after the release the next walk is STANDARD's.
  click('FAST'); await settle(); await p.poll();
  pointer('pointerdown', -1, 0); await settle(); await p.tick(); await p.poll();
  assert(ctl.walking && ctl.backward && ctl.law === 'FAST' && last().params.vx === -lim.FAST.vx_back_max_m_s, 'FAST backward at its limit');
  const mark7 = cmds().length; click('STANDARD'); await settle(); await p.fire20(2); await p.tick();
  assert(ctl.selected === 'STANDARD' && cmds().slice(mark7).some(c => c.type === 'GAIT.SELECT'), 'selection accepted (deferred)');
  assert(last().type === 'BASE_VELOCITY' && last().params.vx === -lim.FAST.vx_back_max_m_s, 'deferred: the held stick keeps FAST\'s backward limit before the status (' + last().params.vx + ')');
  await p.poll(); await p.tick();
  assert(last().params.vx === -lim.FAST.vx_back_max_m_s && ctl.walking && ctl.law === 'FAST', 'deferred: still FAST\'s limit after the status, the walk goes on');
  assert(/limiti FAST/.test(q('#joy-state').textContent), 'the page names the limits it applies (' + q('#joy-state').textContent + ')');
  pointer('pointerup', 0, 0); await settle(); await p.poll();
  pointer('pointerdown', -1, 0); await settle(); await p.tick();
  assert(last().params.vx === -lim.STANDARD.vx_back_max_m_s && ctl.law === 'STANDARD', 'the next walk takes STANDARD and its backward limit');
  assert.strictEqual(ctl.rejected, 0, 'no joystick demand beyond the limits the Controller applies');
  pointer('pointerup', 0, 0); await settle();
  console.log(`JOYSTICK_MODE_DOM = PASS (full scale SLOW/STANDARD/FAST ${top.SLOW * 1e3}/${top.STANDARD * 1e3}/${top.FAST * 1e3} mm/s, half stick ${(fraction(.5) * 100).toFixed(1)} %, lateral on every mode's own limits, backward where the law has it; held-stick change in order with refreshes at most ${changeGap} ms apart (deadline 1 s); timeout explicit; release and STOP first; repeated selections consistent; shipped page, real fixtures, no device)`);
})().catch(e => { console.error(e); process.exit(1); });
