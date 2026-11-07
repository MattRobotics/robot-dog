// Measures the browser-side cost of loading the real Digital Twin (TwinAssets.h
// bytes, server semantics of handleTwin: gzip, ETag, no-cache, 304, Motion
// deferral of >8 KB bodies) inside the shipped dashboard, in Chrome with CPU
// throttling. Reports long tasks on the shared main thread and the lateness of
// a 50 ms probe timer (what a STOP click or a joystick refresh would wait).
// Used for the dev.17 candidate report (Twin after a reload with torque ON).
// FIXTURES.json: `test_simple_dashboard_contract fixtures OUT.json` (real
// MotionService bodies); SRC_DIR: 05_Firmware/MATDOG_Controller/src of the
// build under test. Chrome at /usr/bin/google-chrome, software WebGL.
// usage: NODE_PATH=<dir with playwright-core> node measure_twin_load.cjs FIXTURES.json SRC_DIR
'use strict';
const { chromium } = require('playwright-core');
const http = require('http'), fs = require('fs'), path = require('path'), zlib = require('zlib');
const [fixturesPath, src] = process.argv.slice(2);
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const h = fs.readFileSync(path.join(src, 'network/TwinAssets.h'), 'utf8');
const arrays = {}; for (const m of h.matchAll(/const uint8_t (kAsset\d+)\[(\d+)\] = \{([0-9,]+)\};/g)) arrays[m[1]] = Buffer.from(m[3].split(',').map(Number));
const assets = {}; for (const m of h.matchAll(/\{"(\/twin\/[^"]*)", "([^"]+)", "\\"([0-9a-f]{32})\\"", (kAsset\d+), sizeof\(kAsset\d+\), (\d+)\}/g)) assets[m[1]] = { type: m[2], etag: '"' + m[3] + '"', gz: arrays[m[4]] };
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
const standing = JSON.parse(fx.status_walking); standing.state = 'STANDING'; standing.phase = 'HOLD'; standing.reason = 'STAND_HOLD'; standing.walk_velocity = {};
const device = { status: fx.status_ready, motion: false, log: [] };
const server = http.createServer((req, res) => {
  let body = ''; req.on('data', c => { body += c; }); req.on('end', () => {
    const url = req.url.split('?')[0];
    const send = (code, headers, data) => { device.log.push({ url, code, bytes: data ? data.length : 0, motion: device.motion }); res.writeHead(code, headers); res.end(data); };
    if (url === '/') return send(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' }, html);
    if (url === '/status') return send(200, { 'Content-Type': 'text/plain' }, 'mode=MAINTENANCE\nsystem_health=READY\n');
    if (url === '/wifi/status') return send(200, { 'Content-Type': 'application/json' }, JSON.stringify({ profiles: [], scan: [], sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
    if (url === '/wifi/session') return send(200, { 'Content-Type': 'text/plain' }, 'c'.repeat(64));
    if (url === '/api/motion/status') return send(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }, device.status);
    if (url === '/api/motion/command') return send(200, { 'Content-Type': 'application/json' }, fx.ack_walk_accepted);
    const a = assets[url];
    if (a) {
      if (req.headers['if-none-match'] === a.etag) return send(304, { ETag: a.etag }, null);
      if (a.gz.length > 8192 && device.motion) return send(503, { 'Content-Type': 'application/json', 'Retry-After': '5', 'Cache-Control': 'no-store' }, '{"version":1,"ok":false,"error":"TWIN_DEFERRED_MOTION_ACTIVE","elapsed_ms":0}');
      return send(200, { 'Content-Type': a.type, 'Content-Encoding': 'gzip', ETag: a.etag, 'Cache-Control': 'no-cache', 'Accept-Ranges': 'none' }, a.gz);
    }
    send(404, { 'Content-Type': 'application/json' }, '{}');
  });
});

async function run(browser, base, label, { throttle, warm, motionStatus }) {
  const context = await browser.newContext({ viewport: { width: 390, height: 844 }, isMobile: true, hasTouch: true });
  await context.addInitScript(() => {
    try { localStorage.setItem('md_login', '1'); } catch (e) {}
    window.__long = []; window.__late = [];
    try { new PerformanceObserver(l => { for (const e of l.getEntries()) window.__long.push({ t: e.startTime, d: e.duration, a: e.attribution?.[0]?.containerSrc || e.attribution?.[0]?.name }); }).observe({ type: 'longtask', buffered: true }); } catch (e) {}
    let last = performance.now(); setInterval(() => { const n = performance.now(); window.__late.push([n, n - last - 50]); last = n; }, 50);
  });
  const page = await context.newPage(), cdp = await context.newCDPSession(page);
  if (warm) { device.status = fx.status_ready; device.motion = false; await page.goto(base + '/'); await page.waitForFunction(() => /Digital Twin/.test(document.body.textContent) && window.__twinReady === true || false, null, { timeout: 2000 }).catch(() => {}); await page.waitForTimeout(6000); }
  device.status = motionStatus ? JSON.stringify(motionStatus) : fx.status_ready; device.motion = !!motionStatus; device.log.length = 0;
  await cdp.send('Emulation.setCPUThrottlingRate', { rate: throttle });
  const t0 = Date.now();
  await page.goto(base + '/');
  let ready = false;
  for (let i = 0; i < 60 && !ready; i++) { await page.waitForTimeout(250); ready = await page.evaluate(() => document.getElementById('simple-twin-state').textContent === '').catch(() => false); }
  await page.waitForTimeout(3000);
  const out = await page.evaluate(() => ({ long: window.__long, late: window.__late.map(x => x[1]), notice: document.getElementById('simple-twin-state').textContent, frame: document.getElementById('simple-twin-frame').getAttribute('src') }));
  await cdp.send('Emulation.setCPUThrottlingRate', { rate: 1 });
  await context.close();
  const longMax = Math.max(0, ...out.long.map(x => x.d)), longSum = out.long.reduce((s, x) => s + x.d, 0), lateMax = Math.max(0, ...out.late);
  const twinReq = device.log.filter(x => x.url.startsWith('/twin'));
  console.log(JSON.stringify({ label, throttle, warm, motion: !!motionStatus, twin_ready: out.notice === '', frame: out.frame, notice: out.notice,
    longtasks: out.long.length, longtask_max_ms: Math.round(longMax), longtask_total_ms: Math.round(longSum), probe_late_max_ms: Math.round(lateMax),
    twin_requests: twinReq.map(x => x.url.replace('/twin', '') + ':' + x.code + ':' + x.bytes), elapsed_ms: Date.now() - t0 }));
}

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const base = 'http://127.0.0.1:' + server.address().port;
  const browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: true, args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  try {
    for (const throttle of [1, 4, 6]) {
      await run(browser, base, 'refresh torque ON HOLD, warm cache (304)', { throttle, warm: true, motionStatus: standing });
      await run(browser, base, 'refresh torque ON HOLD, cold cache (server defers)', { throttle, warm: false, motionStatus: standing });
      await run(browser, base, 'refresh torque OFF, cold cache', { throttle, warm: false, motionStatus: null });
      await run(browser, base, 'refresh torque OFF, warm cache (304)', { throttle, warm: true, motionStatus: null });
    }
  } finally { await browser.close(); server.close(); }
})().catch(e => { console.error(e); process.exit(1); });
