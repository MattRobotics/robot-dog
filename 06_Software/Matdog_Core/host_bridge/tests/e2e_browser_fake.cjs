#!/usr/bin/env node
// Browser E2E of the unified Dashboard + Digital Twin against the bridge in
// --fake-device mode (pty simulator, labelled FAKE_DEVICE_NOT_HARDWARE).
// It proves composition only: real hardware/Android LIVE is a separate test.
// Usage: NODE_PATH=<dir with playwright-core> node e2e_browser_fake.cjs <viewer-dist> <out-dir>
'use strict';
const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');
const http = require('http');
const { chromium } = require('playwright-core');

const [dist, outDir] = process.argv.slice(2);
if (!dist || !outDir) { console.error('usage: e2e_browser_fake.cjs <viewer-dist> <out-dir>'); process.exit(2); }
fs.mkdirSync(outDir, { recursive: true });
const port = 8799, base = `http://127.0.0.1:${port}`;
const bridgePath = path.join(__dirname, '..', 'matdog_bridge.py');
const logDir = fs.mkdtempSync(path.join(require('os').tmpdir(), 'matdog-e2e-'));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
function get(url) {
  return new Promise((resolve, reject) => {
    http.get(url, (res) => { let body = ''; res.on('data', (c) => { body += c; }); res.on('end', () => resolve({ status: res.statusCode, body })); }).on('error', reject);
  });
}
function sse(url, ms) {
  return new Promise((resolve) => {
    let events = 0, bytes = 0; const sizes = [];
    const req = http.get(url, (res) => {
      let pending = '';
      res.on('data', (chunk) => {
        bytes += chunk.length; pending += chunk.toString();
        let i; while ((i = pending.indexOf('\n\n')) >= 0) { const block = pending.slice(0, i); pending = pending.slice(i + 2); if (block.startsWith('data:') || block.includes('\ndata:')) { events += 1; sizes.push(block.length); } }
      });
    });
    setTimeout(() => { req.destroy(); resolve({ events, bytes, seconds: ms / 1000, events_per_s: events / (ms / 1000), mean_event_bytes: sizes.length ? Math.round(sizes.reduce((a, b) => a + b, 0) / sizes.length) : 0 }); }, ms);
  });
}
(async () => {
  const bridge = spawn('python3', [bridgePath, '--fake-device', '--http', `127.0.0.1:${port}`, '--console', '127.0.0.1:8798', '--viewer-dist', dist, '--log-dir', logDir, '--lock-dir', logDir], { stdio: ['ignore', 'pipe', 'pipe'] });
  let bridgeOut = ''; bridge.stdout.on('data', (d) => { bridgeOut += d; }); bridge.stderr.on('data', (d) => { bridgeOut += d; });
  const result = { schema: 'MATDOG_DASHBOARD_E2E_FAKE_DEVICE_V1', hardware: false, checks: {} };
  let browser;
  try {
    for (let i = 0; i < 50; i += 1) { try { const h = await get(`${base}/api/bridge/health`); if (h.status === 200) break; } catch { /* starting */ } await sleep(200); }
    await sleep(2500);
    result.health = JSON.parse((await get(`${base}/api/bridge/health`)).body);
    const status = await get(`${base}/api/motion/status`); result.status_http = status.status;
    const parsed = JSON.parse(status.body); result.status_source = parsed?.telemetry?.source; result.status_bytes = status.body.length;
    result.sse = await sse(`${base}/api/bridge/events`, 5000);
    browser = await chromium.launch({ executablePath: process.env.MATDOG_TEST_CHROME || '/usr/bin/google-chrome', headless: true,
      args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', '--no-sandbox'] });
    for (const [label, viewport] of [['desktop', { width: 1440, height: 900 }], ['mobile_390x844', { width: 390, height: 844 }]]) {
      const page = await browser.newPage({ viewport, deviceScaleFactor: label === 'desktop' ? 1 : 2, isMobile: label !== 'desktop', hasTouch: label !== 'desktop' });
      const consoleErrors = [], missing = []; page.on('console', (m) => { if (m.type() === 'error') consoleErrors.push(m.text()); });
      page.on('response', (r) => { if (r.status() >= 400) missing.push(`${r.status()} ${new URL(r.url()).pathname}`); });
      await page.goto(`${base}/`, { waitUntil: 'load' });
      await page.waitForFunction(() => window.matdogViewer && window.matdogViewer.robot() !== null, null, { timeout: 60000 });
      await sleep(3000);
      // Headless rAF is throttled: render explicitly before each observation.
      const first = await page.evaluate(() => { window.matdogViewer.forceRender(); return window.matdogViewer.snapshot(); });
      await sleep(1500);
      const second = await page.evaluate(() => { window.matdogViewer.forceRender(); return window.matdogViewer.snapshot(); });
      const moved = Object.keys(first.joints).some((k) => Math.abs((first.joints[k] ?? 0) - (second.joints[k] ?? 0)) > 1e-6);
      const sampleMoved = JSON.stringify(first.motion?.actual) !== JSON.stringify(second.motion?.actual);
      await page.evaluate(() => window.matdogViewer.forceRender());
      await page.screenshot({ path: path.join(outDir, `dashboard_twin_${label}_FAKE_DEVICE.png`), fullPage: false });
      const badge = await page.evaluate(() => (document.querySelector('#live-badge, .live-badge, [data-live-badge]')?.textContent ?? '').trim());
      let stop = null;
      if (label === 'desktop') {
        await page.click('#btn-stop');
        await sleep(800);
        stop = await page.evaluate(() => window.matdogViewer.snapshot().stop);
      }
      result.checks[label] = { collision_meshes: second.collisionMeshCount, joints: second.jointCount, source: second.source, codes: second.codes,
        transport: second.transport, joints_animated: moved, telemetry_actual_changed: sampleMoved, badge_text: badge, stop, console_errors: consoleErrors.slice(0, 5), http_errors: missing.slice(0, 5) };
      await page.close();
    }
    const d = result.checks.desktop, m = result.checks.mobile_390x844;
    result.pass = [d, m].every((c) => c.collision_meshes === 17 && c.joints_animated && c.source?.badge === 'FAKE_DEVICE') &&
      d.stop?.ok === true && result.status_source === 'FAKE_DEVICE_NOT_HARDWARE' && result.sse.events_per_s >= 5;
  } catch (error) {
    result.error = String(error && error.stack || error); result.pass = false;
  } finally {
    if (browser) await browser.close();
    bridge.kill('SIGTERM'); await sleep(500);
    result.bridge_log_tail = bridgeOut.split('\n').slice(-8);
    fs.writeFileSync(path.join(outDir, 'dashboard_e2e_fake_device.json'), JSON.stringify(result, null, 2) + '\n');
    console.log(JSON.stringify({ pass: result.pass, sse: result.sse, desktop: result.checks.desktop, mobile: result.checks.mobile_390x844, error: result.error }, null, 1));
    process.exit(result.pass ? 0 : 1);
  }
})();
