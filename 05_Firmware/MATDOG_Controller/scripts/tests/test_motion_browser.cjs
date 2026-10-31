#!/usr/bin/env node
/** Fresh, owned headless browser; localhost offline servers only. No hardware. */
'use strict';
const assert = require('node:assert/strict');
const path = require('node:path');
const fs = require('node:fs');
const { createHash } = require('node:crypto');
const { chromium } = require('playwright-core');
const output = path.resolve(__dirname, '../../../../09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/browser');
(async () => {
  fs.mkdirSync(output, { recursive: true });
  const browser = await chromium.launch({ executablePath: process.env.MATDOG_TEST_CHROME || '/usr/bin/google-chrome', headless: true,
    args: ['--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader'] });
  try {
    const context = await browser.newContext({ viewport: { width: 1440, height: 1000 } });
    const page = await context.newPage(), errors = [], writes = [];
    page.on('pageerror', (error) => errors.push(error.message));
    page.on('request', (request) => { if (request.method() !== 'GET') writes.push(request.url()); });
    await page.goto('http://127.0.0.1:5185', { waitUntil: 'networkidle' });
    await page.locator('nav button[data-page="motion-control"]').click();
    await page.waitForFunction(() => document.querySelectorAll('[data-action]').length === 48);
    assert.equal(await page.locator('nav button[data-page]').count(), 7);
    assert.equal(await page.locator('[data-action]:disabled').count(), 46);
    assert.equal(await page.locator('#motion-joystick').getAttribute('aria-disabled'), 'true');
    for (const id of ['motion-stand','motion-lie','motion-gait','motion-speed','motion-yaw','motion-body']) assert.equal(await page.locator('#'+id).isDisabled(), true);
    assert.equal(await page.locator('#motion-stop-button').isDisabled(), false);
    assert.equal(await page.locator('.motion-action').count(), 48);
    await page.screenshot({ path: path.join(output, 'dashboard_motion_desktop.png'), fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.screenshot({ path: path.join(output, 'dashboard_motion_mobile.png'), fullPage: true });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true, 'no mobile horizontal overflow');
    assert.equal(writes.length, 0, 'displaying controls never issues writes');
    assert.deepEqual(errors, []);
    await page.setViewportSize({ width: 1440, height: 1000 });
    await page.goto('http://127.0.0.1:5183/?motion=trace', { waitUntil: 'networkidle' });
    await page.evaluate(() => window.matdogViewer.ready);
    await page.waitForFunction(() => window.matdogViewer.snapshot().motion?.standState === 'ACQUIRE_SUPPORT', { timeout: 15000 });
    const trace = await page.evaluate(() => ({ snapshot: window.matdogViewer.snapshot(), render: window.matdogViewer.forceRender() }));
    assert.equal(trace.snapshot.collisionMeshCount, 17); assert.equal(trace.snapshot.jointCount, 16);
    assert.equal(trace.snapshot.motionStale, false); assert.equal(Object.keys(trace.snapshot.joints).length, 12);
    assert.equal(trace.snapshot.motion.owner, 'offline_trace');
    assert.match(trace.snapshot.motion.reason, /SYNTHETIC_NOT_HARDWARE_QUALIFIED/);
    assert.match(trace.snapshot.motion.reason, /EXTERNAL_FIXTURE_SUPPORT/);
    assert.match(await page.locator('#twin-fault').textContent(), /EXTERNAL_FIXTURE_SUPPORT/);
    assert.ok(trace.render.triangles > 0); assert.ok(trace.render.calls > 0);
    await page.screenshot({ path: path.join(output, 'twin_trace_acquire.png'), fullPage: true });
    await page.locator('#btn-motion-pause').click();
    assert.equal(await page.locator('#twin-fresh').textContent(), 'STALE');
    assert.equal((await page.evaluate(() => window.matdogViewer.snapshot())).targetVisible, false);
    await page.locator('#btn-motion-synthetic').click();
    await page.waitForFunction(() => window.matdogViewer.snapshot().motion?.standState === 'WALK', { timeout: 20000 });
    const walk = await page.evaluate(() => window.matdogViewer.snapshot());
    assert.equal(walk.motionStale, false); assert.equal(walk.targetVisible, true);
    await page.screenshot({ path: path.join(output, 'twin_synthetic_walk.png'), fullPage: true });
    assert.deepEqual(errors, []); assert.equal(writes.length, 0, 'host twin emits no commands');
    const traceBytes = fs.readFileSync(path.join(output, '..', 'adaptive_motion_synthetic.jsonl'));
    const traceSource = traceBytes.toString('utf8').trim().split('\n').map((line) => JSON.parse(line));
    fs.writeFileSync(path.join(output, 'browser_checks.json'), JSON.stringify({ trace_sha256: createHash('sha256').update(traceBytes).digest('hex'), trace_samples: traceSource.length, external_fixture_frames: traceSource.filter((frame) => frame.telemetry.external_support_confirmed === true).length, result: 'PASS', browser: await browser.version(), environment: 'Fresh owned headless Chromium; SwiftShader software rendering; no hardware.', action_count: 48, disabled_actions: 46, preserved_tabs: 6, display_requests_write_count: writes.length, screenshots: ['dashboard_motion_desktop.png','dashboard_motion_mobile.png','twin_trace_acquire.png','twin_synthetic_walk.png'], trace }, null, 2)+'\n');
    console.log('MOTION_BROWSER = PASS (48 cards, six tabs preserved, disabled controls, mobile layout, WebGL canonical meshes + trace/synthetic feed, stale overlay, no writes)');
    await context.close();
  } finally { await browser.close(); }
})().catch((error) => { console.error(error); process.exit(1); });
