// STOP while the Digital Twin loads (dev.17 pre-flash), in Chrome with the
// SHIPPED page and the Twin bytes of TwinAssets.h served with handleTwin's
// semantics (gzip, ETag, 304; >8 KB bodies deferred with 503 while torque is
// ON). The robot holds still with torque ON (real status_standing fixture,
// live timestamps), so a page reload starts the Twin. STOP is clicked and
// Escape pressed while the Twin script compiles (warm cache: 304s) or while it
// is refused (cold cache), at phone-class CPU throttling, and again with the
// Twin idle. Measured: input -> STOP request at the server (same clock), and
// in the page input event -> STOP request sent. Also checked: every STOP is
// sent once per input, no joystick command, no request outside the page's
// own reads, Twin assets and STOP. Offline, no device: the robot-side STOP
// lane is not exercised here.
// Usage: NODE_PATH=<dir with playwright-core> node test_stop_twin_load_browser.cjs FIXTURES.json
'use strict';
const { chromium } = require('playwright-core');
const http = require('http'), fs = require('fs'), path = require('path'), assert = require('assert');
const [fixturesPath] = process.argv.slice(2);
assert(fixturesPath, 'usage: test_stop_twin_load_browser.cjs FIXTURES.json');
const src = path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const h = fs.readFileSync(path.join(src, 'network/TwinAssets.h'), 'utf8');
const arrays = {}; for (const m of h.matchAll(/const uint8_t (kAsset\d+)\[(\d+)\] = \{([0-9,]+)\};/g)) arrays[m[1]] = Buffer.from(m[3].split(',').map(Number));
const assets = {}; for (const m of h.matchAll(/\{"(\/twin\/[^"]*)", "([^"]+)", "\\"([0-9a-f]{32})\\"", (kAsset\d+), sizeof\(kAsset\d+\), (\d+)\}/g)) assets[m[1]] = { type: m[2], etag: '"' + m[3] + '"', gz: arrays[m[4]] };
assert.strictEqual(Object.keys(assets).length, 4, 'four Twin assets');
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
assert(JSON.parse(fx.status_standing).torque_active === true, 'HOLD fixture with torque ON');
const t0 = Date.now(), ns = () => Number(process.hrtime.bigint()) / 1e6;
const device = { torqueOn: false, log: [] };
const server = http.createServer((req, res) => {
  let body = ''; req.on('data', c => { body += c; }); req.on('end', () => {
    const url = req.url.split('?')[0], at = ns();
    let type = null; if (url === '/api/motion/command') try { type = JSON.parse(body).type; } catch (e) { type = 'INVALID'; }
    device.log.push({ url, at, type, params: type ? (JSON.parse(body).params || {}) : null });
    const send = (code, headers, data) => { res.writeHead(code, headers); res.end(data); };
    if (url === '/') return send(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' }, html);
    if (url === '/status') return send(200, { 'Content-Type': 'text/plain' }, 'mode=MAINTENANCE\nsystem_health=READY\n');
    if (url === '/wifi/status') return send(200, { 'Content-Type': 'application/json' }, JSON.stringify({ profiles: [], scan: [], sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
    if (url === '/wifi/session') return send(200, { 'Content-Type': 'text/plain' }, 'c'.repeat(64));
    if (url === '/api/motion/status') {
      const s = JSON.parse(device.torqueOn ? fx.status_standing : fx.status_ready), t = Date.now() - t0;
      s.now_ms += t; s.telemetry.sampled_at_ms += t;
      return send(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }, JSON.stringify(s));
    }
    if (url === '/api/motion/command') {
      let a = JSON.parse(fx.ack_stop); try { const c = JSON.parse(body); a.id = c.id; a.sequence = c.sequence; } catch (e) { /* refused below */ }
      return send(200, { 'Content-Type': 'application/json' }, JSON.stringify(a));
    }
    const a = assets[url];
    if (a) {
      if (req.headers['if-none-match'] === a.etag) return send(304, { ETag: a.etag });
      if (a.gz.length > 8192 && device.torqueOn) return send(503, { 'Content-Type': 'application/json', 'Retry-After': '5' }, '{"version":1,"ok":false,"error":"TWIN_DEFERRED_MOTION_ACTIVE","elapsed_ms":0}');
      return send(200, { 'Content-Type': a.type, 'Content-Encoding': 'gzip', ETag: a.etag, 'Cache-Control': 'no-cache' }, a.gz);
    }
    return send(404, { 'Content-Type': 'application/json' }, '{}');
  });
});
const median = v => { const s = [...v].sort((a, b) => a - b); return s.length ? s[Math.floor(s.length / 2)] : null; };

async function scenario(browser, base, { warm, throttle, input }) {
  const context = await browser.newContext({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 1 });
  await context.addInitScript(() => {
    try { localStorage.setItem('md_login', '1'); } catch (e) { /* no storage */ }
    window.__inputs = [];  // registered before the page's listeners
    for (const t of ['pointerdown', 'keydown']) addEventListener(t, e => window.__inputs.push(e.timeStamp), true);
  });
  const page = await context.newPage(), cdp = await context.newCDPSession(page), errors = [];
  page.on('pageerror', e => errors.push(e.message));
  try {
    if (warm) {  // the Twin was loaded earlier at torque OFF: its files are in the browser cache
      device.torqueOn = false;
      await page.goto(base + '/');
      await page.waitForFunction(() => document.getElementById('simple-twin-state')?.textContent === '', null, { timeout: 30000 });
    }
    device.torqueOn = true; device.log.length = 0;
    await cdp.send('Emulation.setCPUThrottlingRate', { rate: throttle });
    await page.goto(base + '/');  // reload in HOLD with torque ON
    const box = await page.locator('#simple-stop').boundingBox();
    const press = async () => { const at = ns(); if (input === 'click') await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2); else await page.keyboard.press('Escape'); return at; };
    // Wait for the Twin script request, then press 4 times, 120 ms apart, during its load.
    const deadline = ns() + 30000;
    while (!device.log.some(r => r.url === '/twin/assets/twin.js') && ns() < deadline) await new Promise(r => setTimeout(r, 5));
    const scriptAt = device.log.find(r => r.url === '/twin/assets/twin.js')?.at;
    const loading = [];
    for (let k = 0; k < 4; k++) { loading.push(await press()); await new Promise(r => setTimeout(r, 120)); }
    await page.waitForTimeout(3000);
    const ready = await page.evaluate(() => document.getElementById('simple-twin-state').textContent === '');
    await page.waitForTimeout(1500);
    const idle = [];
    for (let k = 0; k < 2; k++) { idle.push(await press()); await new Promise(r => setTimeout(r, 300)); }
    await page.waitForTimeout(1500);
    const stops = device.log.filter(r => r.type === 'MOTION.STOP').map(r => r.at);
    const joystick = device.log.filter(r => r.type === 'BASE_VELOCITY').length;
    const allowed = new Set(['/', '/status', '/wifi/status', '/wifi/session', '/api/motion/status', '/api/motion/command', ...Object.keys(assets)]);
    const foreign = [...new Set(device.log.map(r => r.url).filter(u => !allowed.has(u)))];
    const twinRequests = device.log.filter(r => r.url.startsWith('/twin')).length;
    const page_ = await page.evaluate(() => ({ inputs: window.__inputs, sent: window.matdogMotionTrace.commands.filter(c => c.type === 'MOTION.STOP').map(c => c.sent_ms),
      traced: window.matdogMotionTrace.commands.filter(c => c.type === 'MOTION.STOP').map(c => c.input_ms) }));
    const pairs = (from, to) => from.map((t, i) => to[i] - t);
    const all = [...loading, ...idle];
    const serverLat = stops.length === all.length ? pairs(all, stops) : null;
    const pageLat = page_.sent.length === page_.inputs.length ? pairs(page_.inputs, page_.sent) : null;
    // The page's own trace (input_ms, used on the phone) must agree with the event times seen here.
    const traceAgrees = page_.traced.length === page_.inputs.length && page_.traced.every((t, i) => Number.isFinite(t) && Math.abs(t - page_.inputs[i]) < 1);
    const r = { warm, throttle, input, trace_input_ms_agrees: traceAgrees, twin_ready_after_reload: ready, script_requested_ms_after_reload: scriptAt ? Math.round(scriptAt - (device.log[0]?.at ?? scriptAt)) : null,
      stops_sent: stops.length, inputs: all.length, joystick_commands: joystick, foreign_requests: foreign, twin_requests: twinRequests, errors: errors.length,
      server_ms_loading: serverLat && serverLat.slice(0, 4).map(Math.round), server_ms_idle: serverLat && serverLat.slice(4).map(Math.round),
      page_ms_loading: pageLat && pageLat.slice(0, 4).map(Math.round), page_ms_idle: pageLat && pageLat.slice(4).map(Math.round) };
    return r;
  } finally { await cdp.send('Emulation.setCPUThrottlingRate', { rate: 1 }).catch(() => {}); await context.close(); }
}

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const base = 'http://127.0.0.1:' + server.address().port;
  const browser = await chromium.launch({ executablePath: process.env.MATDOG_CHROME || '/usr/bin/google-chrome', headless: true,
    args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const chrome = 'Chrome ' + browser.version(), results = [];
  try {
    for (const throttle of [1, 4, 6]) for (const input of ['click', 'escape']) results.push(await scenario(browser, base, { warm: true, throttle, input }));
    for (const input of ['click', 'escape']) results.push(await scenario(browser, base, { warm: false, throttle: 6, input }));
  } finally { await browser.close(); server.close(); }
  let failures = 0, worst = 0;
  for (const r of results) {
    // Every input sends exactly one STOP (none lost, none doubled), no joystick
    // command, no request outside the page's reads, Twin files and STOP.
    const ok = r.stops_sent === r.inputs && r.joystick_commands === 0 && r.foreign_requests.length === 0 && r.errors === 0 && r.server_ms_loading && r.page_ms_loading && r.trace_input_ms_agrees;
    if (!ok) failures++;
    if (r.server_ms_loading) worst = Math.max(worst, ...r.server_ms_loading);
    console.log('STOP_TWIN_LOAD ' + JSON.stringify(r) + ' -> ' + (ok ? 'PASS' : 'FAIL'));
  }
  console.log('STOP_DURING_TWIN_LOAD_BROWSER ' + (failures ? 'FAIL' : 'PASS') + ' (' + chrome + '; worst input->STOP request at the server during a Twin load: ' + Math.round(worst) +
    ' ms; measured, not a bound; offline, software WebGL, no device)');
  process.exit(failures ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
