// Real-browser check of the Digital Twin view cube (dev.17) with the SHIPPED
// bytes: the dashboard page (PortalPage.h + MotionDashboard.h) and the Twin
// assets of TwinAssets.h, served with handleTwin's gzip/ETag semantics, in
// Chrome (software WebGL). Desktop mouse and phone touch: the cube is drawn,
// a click/tap on a face snaps the camera to that face's view, a drag is still
// the orbit rotation, "Reset vista" restores the initial front view, and the
// cube issues no HTTP request of its own. Camera only: Motion fixtures, no device.
// Usage: NODE_PATH=<dir with playwright-core> node test_twin_viewcube_browser.cjs FIXTURES.json [SHOTS_DIR]
'use strict';
const { chromium } = require('playwright-core');
const http = require('http'), fs = require('fs'), path = require('path'), assert = require('assert');
const [fixturesPath, shots] = process.argv.slice(2);
assert(fixturesPath, 'usage: test_twin_viewcube_browser.cjs FIXTURES.json [SHOTS_DIR]');
const src = path.join(__dirname, '../../src');
const portal = fs.readFileSync(path.join(src, 'network/PortalPage.h'), 'utf8');
const motion = fs.readFileSync(path.join(src, 'network/MotionDashboard.h'), 'utf8');
const html = portal.split('R"MATDOG(')[1].split(')MATDOG"')[0] + motion.split('R"MOTION(')[1].split(')MOTION"')[0] + '</html>';
const h = fs.readFileSync(path.join(src, 'network/TwinAssets.h'), 'utf8');
const arrays = {}; for (const m of h.matchAll(/const uint8_t (kAsset\d+)\[(\d+)\] = \{([0-9,]+)\};/g)) arrays[m[1]] = Buffer.from(m[3].split(',').map(Number));
const assets = {}; for (const m of h.matchAll(/\{"(\/twin\/[^"]*)", "([^"]+)", "\\"([0-9a-f]{32})\\"", (kAsset\d+), sizeof\(kAsset\d+\), (\d+)\}/g)) assets[m[1]] = { type: m[2], etag: '"' + m[3] + '"', gz: arrays[m[4]] };
assert.strictEqual(Object.keys(assets).length, 4, 'four Twin assets');
const fx = JSON.parse(fs.readFileSync(fixturesPath, 'utf8'));
const log = [], started = Date.now();
const server = http.createServer((req, res) => {
  req.resume(); req.on('end', () => {
    const url = req.url.split('?')[0]; log.push(url);
    const send = (code, headers, data) => { res.writeHead(code, headers); res.end(data); };
    if (url === '/') return send(200, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' }, html);
    if (url === '/status') return send(200, { 'Content-Type': 'text/plain' }, 'mode=MAINTENANCE\nsystem_health=READY\n');
    if (url === '/wifi/status') return send(200, { 'Content-Type': 'application/json' }, JSON.stringify({ profiles: [], scan: [], sta_test: { seq: 0 }, policy: {}, network_phase: 'ADMIN_AP', admin_generation: 1 }));
    if (url === '/wifi/session') return send(200, { 'Content-Type': 'text/plain' }, 'c'.repeat(64));
    if (url === '/api/motion/status') {  // a live Controller: each read is a new sample
      const s = JSON.parse(fx.status_ready), t = Date.now() - started;
      s.now_ms += t; s.telemetry.sampled_at_ms += t;
      return send(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }, JSON.stringify(s));
    }
    const a = assets[url];
    if (a) {
      if (req.headers['if-none-match'] === a.etag) return send(304, { ETag: a.etag });
      return send(200, { 'Content-Type': a.type, 'Content-Encoding': 'gzip', ETag: a.etag, 'Cache-Control': 'no-cache', 'Accept-Ranges': 'none' }, a.gz);
    }
    return send(404, { 'Content-Type': 'application/json' }, '{}');
  });
});
// Same corner as viewCube.ts cubeViewport (CSS px, from the canvas top-right).
const cubeCentre = (w, hgt) => { const size = Math.round(Math.min(130, Math.max(80, 0.3 * Math.min(w, hgt)))); return { x: w - 8 - size / 2, y: 8 + size / 2, size }; };
// Share of light pixels (cube edges, labels, triad) in a PNG, decoded in the page.
const lightShare = (page, png) => page.evaluate(b64 => new Promise(resolve => { const i = new Image(); i.onload = () => {
  const cv = document.createElement('canvas'); cv.width = i.width; cv.height = i.height; const g = cv.getContext('2d'); g.drawImage(i, 0, 0);
  const d = g.getImageData(0, 0, i.width, i.height).data; let n = 0; for (let k = 0; k < d.length; k += 4) if (d[k] + d[k + 1] + d[k + 2] > 360) n++; resolve(n / (d.length / 4)); };
  i.src = 'data:image/png;base64,' + b64; }), png.toString('base64'));

async function scenario(browser, base, name, viewport, touch) {
  const context = await browser.newContext({ viewport, deviceScaleFactor: 1, hasTouch: touch, isMobile: touch });
  await context.addInitScript(() => { try { localStorage.setItem('md_login', '1'); } catch (e) { /* no storage */ } });
  const page = await context.newPage(), errors = [];
  page.on('pageerror', e => errors.push(e.message));
  const out = { name, errors };
  try {
    await page.goto(base + '/');
    await page.waitForFunction(() => document.getElementById('simple-twin-state')?.textContent === '', null, { timeout: 30000 });
    const frame = page.frames().find(f => f.url().includes('/twin/'));
    await frame.waitForFunction(() => document.querySelector('#viewport canvas'), null, { timeout: 10000 });
    await page.evaluate(() => document.getElementById('simple-twin').scrollIntoView({ block: 'start' }));
    await page.waitForTimeout(800);
    const canvas = await frame.locator('#viewport canvas').boundingBox();
    const view = () => frame.evaluate(() => document.getElementById('viewport').dataset.view);
    const c = cubeCentre(canvas.width, canvas.height), at = { x: canvas.x + c.x, y: canvas.y + c.y };
    const cube = async label => { if (shots) await page.screenshot({ path: path.join(shots, name + '-' + label + '.png'), clip: { x: canvas.x, y: canvas.y, width: canvas.width, height: canvas.height } }); };
    const shot = () => page.screenshot({ clip: { x: canvas.x, y: canvas.y, width: canvas.width, height: canvas.height } });
    const tap = async (x, y) => { if (touch) await page.touchscreen.tap(x, y); else await page.mouse.click(x, y); await page.waitForTimeout(400); };
    const cdp = touch ? await context.newCDPSession(page) : null;
    const drag = async (x0, y0, dx, dy) => {
      if (touch) {
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: x0, y: y0, id: 1 }] });
        for (let k = 1; k <= 10; k++) await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: x0 + dx * k / 10, y: y0 + dy * k / 10, id: 1 }] });
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
      } else { await page.mouse.move(x0, y0); await page.mouse.down(); await page.mouse.move(x0 + dx, y0 + dy, { steps: 10 }); await page.mouse.up(); }
      await page.waitForTimeout(900);  // OrbitControls damping settles
    };
    out.initial = await view();
    await cube('initial');
    const initialShot = await shot();
    // The cube is drawn: its corner is not the plain scene background.
    const corner = await page.screenshot({ clip: { x: at.x - c.size / 2, y: at.y - c.size / 2, width: c.size, height: c.size } });
    out.cube_light_share = +(await lightShare(page, corner)).toFixed(3);
    out.cube_drawn = out.cube_light_share > 0.02;
    const before = log.length;
    await tap(at.x, at.y); out.after_front_click = await view(); await cube('front');
    await drag(canvas.x + canvas.width * 0.4, canvas.y + canvas.height * 0.3, 0, canvas.height * 0.45);
    await tap(at.x, at.y); out.after_drag_down = await view(); await cube('top');
    const reset = () => frame.evaluate(() => document.getElementById('reset-view').click());  // the button's own handler, no page scroll
    await reset(); await page.waitForTimeout(600); out.after_reset = await view();
    await drag(canvas.x + canvas.width * 0.4, canvas.y + canvas.height * 0.5, canvas.height * 0.25, 0);
    await tap(at.x, at.y); out.after_quarter_turn = await view(); await cube('side');
    await reset(); await page.waitForTimeout(900);
    const resetShot = await shot();
    if (shots) await page.screenshot({ path: path.join(shots, name + '-reset.png'), clip: { x: canvas.x, y: canvas.y, width: canvas.width, height: canvas.height } });
    out.reset_matches_initial = Buffer.compare(resetShot, initialShot) === 0 || (await page.evaluate(([a, b]) => new Promise(resolve => {
      const load = s => new Promise(r => { const i = new Image(); i.onload = () => r(i); i.src = 'data:image/png;base64,' + s; });
      Promise.all([load(a), load(b)]).then(([x, y]) => { const cv = document.createElement('canvas'); cv.width = x.width; cv.height = x.height; const g = cv.getContext('2d');
        g.drawImage(x, 0, 0); const da = g.getImageData(0, 0, cv.width, cv.height).data; g.drawImage(y, 0, 0); const db = g.getImageData(0, 0, cv.width, cv.height).data;
        let diff = 0; for (let i = 0; i < da.length; i += 4) if (Math.abs(da[i] - db[i]) + Math.abs(da[i + 1] - db[i + 1]) + Math.abs(da[i + 2] - db[i + 2]) > 24) diff++;
        resolve(diff / (da.length / 4) < 0.005); });
    }), [resetShot.toString('base64'), initialShot.toString('base64')]));
    out.requests = [...new Set(log.slice(before))];
    out.frame_errors = await frame.evaluate(() => document.getElementById('banner')?.textContent || '');
  } finally { await context.close(); }
  return out;
}

(async () => {
  await new Promise(r => server.listen(0, '127.0.0.1', r));
  const base = 'http://127.0.0.1:' + server.address().port;
  const browser = await chromium.launch({ executablePath: process.env.MATDOG_CHROME || '/usr/bin/google-chrome', headless: true,
    args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
  const chrome = 'Chrome ' + browser.version(), results = [];
  try {
    results.push(await scenario(browser, base, 'desktop', { width: 1280, height: 900 }, false));
    results.push(await scenario(browser, base, 'phone', { width: 390, height: 844 }, true));
  } finally { await browser.close(); server.close(); }
  let failures = 0;
  for (const r of results) {
    const ok = r.initial === 'reset' && r.cube_drawn && r.after_front_click === 'front' && r.after_drag_down === 'top' && r.after_reset === 'reset' &&
      ['left', 'right'].includes(r.after_quarter_turn) && r.reset_matches_initial && r.requests.every(u => ['/api/motion/status', '/status', '/wifi/status', '/wifi/session'].includes(u)) && r.errors.length === 0;
    if (!ok) failures++;
    console.log('VIEWCUBE ' + r.name + ' ' + JSON.stringify({ ...r, errors: r.errors.length }) + ' -> ' + (ok ? 'PASS' : 'FAIL'));
  }
  console.log('TWIN_VIEWCUBE_BROWSER ' + (failures ? 'FAIL' : 'PASS') + ' (' + chrome + ', software WebGL; shipped page and TwinAssets.h; camera only; no device)');
  process.exit(failures ? 1 : 0);
})().catch(e => { console.error(e); process.exit(1); });
