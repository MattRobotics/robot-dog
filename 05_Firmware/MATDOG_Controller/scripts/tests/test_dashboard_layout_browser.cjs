// Real-browser layout check of the SHIPPED dashboard (PortalPage.h +
// MotionDashboard.h) in Chrome. While the joystick pad is held, a command
// refusal banner, the long Motion-budget link text, a longer reason text and a
// Twin height report must neither move the pad nor change the request (the
// finger jitters by 0.3 px after each, as a real one does), and STOP must stay
// on screen and uncovered. Desktop (mouse) and phone (touch).
// Controller bodies are the real MotionService fixtures; /twin/ is a stub page
// (this checks the parent page only). No device; 127.0.0.1 only.
// Usage: NODE_PATH=<dir with playwright-core> node test_dashboard_layout_browser.cjs FIXTURES.json [SRC_DIR] [SHOTS_DIR]
// Chrome: MATDOG_CHROME, default /usr/bin/google-chrome.
'use strict';
const { chromium } = require('playwright-core');
const http = require('http'), fs = require('fs'), path = require('path'), assert = require('assert');
const [fixturesPath, srcArg, shots] = process.argv.slice(2);
assert(fixturesPath, 'usage: test_dashboard_layout_browser.cjs FIXTURES.json [SRC_DIR] [SHOTS_DIR]');
const src = srcArg || path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
const CSRF = 'c'.repeat(64);
const jsonError = (code, ms) => JSON.stringify({ version: 1, ok: false, error: code, elapsed_ms: ms });
const TWIN = '<!doctype html><meta charset="utf-8"><body style="margin:0;background:#0c131a;color:#9fb2c3"><div id="app" style="height:420px">twin stub</div>' +
  '<script>parent.postMessage({type:"MATDOG_TWIN_HEIGHT",height:420},location.origin);parent.postMessage({type:"MATDOG_TWIN_READY"},location.origin)</script>';

const device = { status: fx.status_ready, budget: false, replies: [], commands: [] };
const server = http.createServer((req, res) => {
  let body = '';
  req.on('data', c => { body += c; });
  req.on('end', () => {
    const url = req.url.split('?')[0];
    const send = (code, type, text) => { res.writeHead(code, { 'Content-Type': type, 'Cache-Control': 'no-store' }); res.end(text); };
    if (url === '/') return send(200, 'text/html; charset=utf-8', html);
    if (url === '/twin/') return send(200, 'text/html; charset=utf-8', TWIN);
    if (url === '/status') return device.budget ? send(503, 'application/json', jsonError('MOTION_BUDGET_UNAVAILABLE', 101))
      : send(200, 'text/plain', 'mode=MAINTENANCE\nauthority=NONE\npower_state=RUN\nsystem_health=READY\n');
    if (url === '/wifi/status') return send(200, 'application/json', JSON.stringify({ profiles: [], scan: [], active_profile: 0, sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
    if (url === '/wifi/session') return send(200, 'text/plain', CSRF);
    if (url === '/api/motion/status') return send(200, 'application/json', device.status);
    if (url === '/api/motion/command') {
      const c = JSON.parse(body); device.commands.push(c);
      const r = device.replies.shift() || { status: 200, body: c.params.vx || c.params.vy || c.params.yaw_rate ? fx.ack_walk_accepted : fx.ack_walk_stop };
      let text = r.body;
      try { const a = JSON.parse(text); if (a.ack === true) { a.id = c.id; a.sequence = c.sequence; text = JSON.stringify(a); } } catch (e) { /* refusal body */ }
      return send(r.status, 'application/json', text);
    }
    return send(404, 'application/json', '{}');
  });
});

async function scenario(browser, base, name, viewport, touch) {
  Object.assign(device, { status: fx.status_ready, budget: false, replies: [], commands: [] });
  const context = await browser.newContext({ viewport, deviceScaleFactor: 1, hasTouch: touch, isMobile: touch });
  await context.addInitScript(() => {
    try { localStorage.setItem('md_login', '1'); } catch (e) { /* no storage */ }
    // Registered before the page's own listeners: the instant the finger lifts.
    for (const t of ['pointerup', 'pointercancel']) addEventListener(t, () => { window.__liftedAt = performance.now(); }, true);
  });
  const page = await context.newPage(), errors = [];
  page.on('pageerror', e => errors.push(e.message));
  const result = { name, viewport: viewport.width + 'x' + viewport.height, input: touch ? 'touch' : 'mouse', shifts: {}, errors };
  try {
    await page.goto(base + '/');
    await page.waitForFunction(() => document.querySelector('#simple-joystick')?.getAttribute('aria-disabled') === 'false', null, { timeout: 10000 });
    await page.waitForFunction(() => !!document.querySelector('#simple-twin-frame')?.getAttribute('src'), null, { timeout: 10000 });
    await page.evaluate(() => document.getElementById('joy-pad').scrollIntoView({ block: 'center' }));
    await page.waitForTimeout(400);
    const rect = sel => page.evaluate(s => { const r = document.querySelector(s).getBoundingClientRect(); return { x: r.left, y: r.top, w: r.width, h: r.height }; }, sel);
    const r0 = await rect('#joy-pad');
    const fx0 = r0.x + r0.w / 2, fy0 = r0.y + r0.h * 0.3;  // finger: forward, beyond the dead zone
    const cdp = touch ? await context.newCDPSession(page) : null;
    if (touch) await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: fx0, y: fy0, id: 1 }] });
    else { await page.mouse.move(fx0, fy0); await page.mouse.down(); }
    const joystickCommands = () => device.commands.filter(c => c.type === 'BASE_VELOCITY' && c.params.gait === 'JOYSTICK');
    await page.waitForFunction(n => window.matdogMotionTrace.commands.length > n, 0, { timeout: 5000 });
    const finger = async (x, y) => { if (touch) await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x, y, id: 1 }] }); else await page.mouse.move(x, y); };
    const shift = async label => { const r = await rect('#joy-pad'); result.shifts[label] = Math.max(Math.abs(r.x - r0.x), Math.abs(r.y - r0.y));
      await finger(fx0, fy0 + 0.3); await finger(fx0, fy0); await page.waitForTimeout(350); };
    // 1. A refused refresh: banner + longer last-command text.
    device.replies.push({ status: 503, body: jsonError('CONTROLLER_LOOP_LATE', 101) });
    await page.waitForFunction(() => !document.getElementById('message').hidden, null, { timeout: 5000 });
    if (shots) await page.screenshot({ path: path.join(shots, name + '-banner.png') });
    result.stop_with_banner = await page.evaluate(() => { const b = document.getElementById('simple-stop').getBoundingClientRect(), hit = document.elementFromPoint(b.left + b.width / 2, b.top + b.height / 2); return hit && hit.id; });
    await shift('refusal_banner');
    // 2. The general diagnostic paused for Motion budget: the longest link text.
    device.budget = true;
    await page.waitForFunction(() => /MOTION_BUDGET_UNAVAILABLE/.test(document.getElementById('linkText').textContent), null, { timeout: 9000 });
    await shift('budget_link_text');
    // 3. A longer Controller reason, joystick still admitted.
    const reasoned = JSON.parse(fx.status_ready); reasoned.reason = 'PRONE_REST'; device.status = JSON.stringify(reasoned);
    await page.waitForFunction(() => /PRONE a riposo/.test(document.getElementById('simple-reason').textContent), null, { timeout: 5000 });
    await shift('reason_text');
    // 4. The Twin reports a taller page.
    const twin = page.frames().find(f => f.url().includes('/twin/'));
    assert(twin, 'Twin stub frame loaded');
    const frameBefore = (await rect('#simple-twin-frame')).h;
    await twin.evaluate(() => { document.getElementById('app').style.height = '900px'; parent.postMessage({ type: 'MATDOG_TWIN_HEIGHT', height: 900 }, location.origin); });
    await page.waitForTimeout(300);
    await shift('twin_height');
    result.twin_height_while_held = (await rect('#simple-twin-frame')).h;
    // STOP on screen and on top of everything, banner shown.
    result.stop = await page.evaluate(() => { const b = document.getElementById('simple-stop').getBoundingClientRect(), hit = document.elementFromPoint(b.left + b.width / 2, b.top + b.height / 2);
      return { top: Math.round(b.top), bottom: Math.round(b.bottom), viewport: innerHeight, hit: hit && hit.id }; });
    if (shots) await page.screenshot({ path: path.join(shots, name + '-held.png') });
    const held = joystickCommands().filter(c => c.params.vx || c.params.vy);
    result.requests_while_held = held.length;
    result.request_max_change_pct = held.length ? +(100 * Math.max(...held.map(c => Math.hypot(c.params.vx - held[0].params.vx, c.params.vy - held[0].params.vy))) / Math.hypot(held[0].params.vx, held[0].params.vy)).toFixed(2) : null;
    result.request_constant = held.length > 3 && result.request_max_change_pct < 2;
    // Release: one neutral command; the deferred Twin height is applied.
    const before = device.commands.length;
    if (touch) await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    else await page.mouse.up();
    await page.waitForFunction(() => { const c = window.matdogMotionTrace.commands.at(-1); return c && /rilascio|neutro/.test(c.label); }, null, { timeout: 5000 });
    await page.waitForTimeout(600);
    const after = device.commands.slice(before);
    // Every command the page sent from the instant the finger lifted is neutral
    // (a refresh sent just before may still reach the server after it).
    result.after_release = await page.evaluate(() => window.matdogMotionTrace.commands.filter(c => c.sent_ms >= window.__liftedAt).map(c => c.label));
    result.server_after_release = after.map(c => (c.params.vx || c.params.vy || c.params.yaw_rate) ? 'move' : 'neutral');
    result.neutral_on_release = result.after_release.length >= 1 && result.after_release.every(l => /rilascio|neutro/.test(l)) && result.server_after_release.at(-1) === 'neutral';
    result.twin_height_after_release = (await rect('#simple-twin-frame')).h;
    result.twin_deferred = result.twin_height_while_held === frameBefore && result.twin_height_after_release > frameBefore;
    if (shots) await page.screenshot({ path: path.join(shots, name + '-released.png') });
  } finally { await context.close(); }
  return result;
}

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const base = 'http://127.0.0.1:' + server.address().port;
  const browser = await chromium.launch({ executablePath: process.env.MATDOG_CHROME || '/usr/bin/google-chrome', headless: true });
  const chrome = 'Chrome ' + browser.version(), results = [];
  try {
    results.push(await scenario(browser, base, 'desktop', { width: 1280, height: 900 }, false));
    results.push(await scenario(browser, base, 'phone', { width: 390, height: 844 }, true));
  } finally { await browser.close(); server.close(); }
  let failures = 0;
  for (const r of results) {
    const worst = Math.max(...Object.values(r.shifts));
    const ok = worst < 0.5 && r.request_constant && r.neutral_on_release && r.stop.hit === 'simple-stop' && r.stop_with_banner === 'simple-stop' && r.stop.top >= 0 && r.stop.bottom <= r.stop.viewport && r.errors.length === 0;
    if (!ok) failures++;
    console.log('LAYOUT ' + r.name + ' ' + r.viewport + ' ' + r.input + ' pad_shift_px=' + JSON.stringify(r.shifts) + ' request_constant=' + r.request_constant +
      ' requests_while_held=' + r.requests_while_held + ' request_max_change_pct=' + r.request_max_change_pct + ' neutral_on_release=' + r.neutral_on_release + ' after_release=' + JSON.stringify(r.after_release) + ' server_after_release=' + JSON.stringify(r.server_after_release) + ' stop=' + JSON.stringify(r.stop) +
      ' twin_deferred=' + r.twin_deferred + ' errors=' + r.errors.length + ' -> ' + (ok ? 'PASS' : 'FAIL'));
  }
  console.log('DASHBOARD_LAYOUT_BROWSER ' + (failures ? 'FAIL' : 'PASS') + ' (' + chrome + '; shipped page; fixtures; no device)');
  process.exit(failures ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
