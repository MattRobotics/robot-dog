// 503 MOTION_BUDGET_UNAVAILABLE on /status is a paused telemetry, not a lost link.
// Fixtures and page boot reused verbatim from test_portal_dom.cjs; no device.
// Optional DOM test: NODE_PATH points to an isolated jsdom installation.
// All fetch calls are fixtures; this test never contacts a device or server.
// It executes the shipped page script unmodified, extracted from PortalPage.h.
const {JSDOM} = require('jsdom');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const network = path.join(__dirname, '../../src/network');
const source = fs.readFileSync(path.join(network, 'PortalPage.h'), 'utf8');
// The six existing portal features are isolated here; the appended Motion fragment
// is executed by test_motion_dashboard_dom.cjs with Controller capability fixtures.
const literals = [...source.matchAll(/R"MATDOG\(([\s\S]*?)\)MATDOG"/g)].map(m => m[1]);
const html = literals[0] + "</html>";
const security = fs.readFileSync(path.join(network, 'PortalSecurity.cpp'), 'utf8');

const xss = '<img src=x onerror=alert(1)>';
const profile = {ssid: xss, enabled: true, dhcp: true,
  ip: '0.0.0.0', mask: '0.0.0.0', gateway: '0.0.0.0', dns: '0.0.0.0'};
const noTest = {seq: 0, outcome: 'NONE', ssid: '', profile: 0, associated: false, reason: 0,
  attempts: 0, ip: '0.0.0.0', rssi: 0, channel: 0, elapsed_ms: 0, rolled_back: false};
const wifiBase = {network_phase: 'ADMIN_AP', admin_generation: 3, build_id: '837f5237de88',
  ota_ingest: 0, tls: false, connected: false, ssid: xss, bssid: '', ip: '0.0.0.0', rssi: 0,
  channel: 1, ap_active: true, ap_ssid: 'MATDOG-227594', ap_ip: '192.168.4.1', ap_clients: 1,
  sleep_effective: false, source: 'NVS', active_profile: 1, disconnect_reason: 201,
  config_phase: 'IDLE', config_busy: false, config_error: 0, roam_enabled: false,
  roam_threshold: -80, roam_hysteresis: 8, scan_interval_ms: 60000, roam_dwell_ms: 120000,
  bandwidth_mhz: 20, ap_timeout_ms: 900000, ap_always: false,
  profiles: [profile, {...profile, enabled: false}], ap_reload_pending: false,
  sta_enabled: true, sta_credentials: true, sta_state: 'IDLE', sta_fault: 'NONE',
  admin_provisioned: true, sta_test: noTest,
  policy: {run: false, role: 'MAINTENANCE_AP', recovery_level: 0, retry_in_ms: 0}, scan: []};
const statusBase = {build_id: '837f5237de88', system_health: 'READY', power_state: 'RUN',
  mode: 'MAINTENANCE', authority: 'NONE', imu_availability: 'PASS', bms_availability: 'PASS',
  servo_availability: 'PASS', led_availability: 'PASS', imu_stream: 'ON', imu_rv_count: '1000',
  bms_comm: 'OK', bms_age_ms: '400', bms_valid: 'YES', bms_pack_v: '11.3',
  bms_soc_percent: '38.0', wifi_state: 'INACTIVE', wifi_connected: 'NO', wifi_fault: 'NONE',
  led_presentation: 'READY', actuator_policy_epoch: '0', actuator_geometry_bound: 'YES',
  actuator_readiness_torque_enable: 'BLOCKED', actuator_readiness_position_command: 'BLOCKED',
  calibration_session_state: 'NO_SESSION', calibration_hardware_motion_authorized: 'NO',
  calibration_restore_required: 'NO', ota_state: 'IDLE', ota_running_build_id: '837f5237de88',
  ota_ingest_compiled: 'NO', ota_rollback_possible: 'NO', uptime_ms: '125000',
  authority_inhibited: 'NO', authority_inhibit_reason: 'NONE', imu_detected: 'ONLINE',
  bms_detected: 'ONLINE', servo_detected: 'ONLINE', led_detected: 'UNKNOWN',
  imu_runtime_resets: '0', bms_sample_age_ms: '400', bms_fresh: 'YES',
  bms_fresh_limit_ms: '5000', bms_current_a: '-0.4', bms_state: 'DISCHARGING',
  bms_charge_mos: 'ON', bms_discharge_mos: 'ON', bms_alarms: '0000 0000 0000 0000',
  bms_cells: '3', bms_cell_min_mv: '3779', bms_cell_max_mv: '3781', bms_cell_delta_mv: '2',
  bms_cell_mv: '3780,3781,3779', servo_canonical: '17', servo_expected_now: '13',
  servo_census: 'NOT_RUN', calibration_holds_authority: 'NO', calibration_stale: 'YES',
  calibration_available: 'YES', calibration_persist_verdict: 'VALID_ACKNOWLEDGED',
  calibration_ack_generation: '1', calibration_record_intact: 'YES', calibration_nvs: 'READY',
  led_soc_valid: 'YES', led_soc_segments: '3', led_charging: 'NO', led_charging_fault: 'NO',
  led_battery_warning: 'NO', led_battery_critical: 'NO', led_diagnostic: 'NONE',
  led_soc_percent: '38.0', network_phase: 'ADMIN_AP', wifi_enabled: 'YES', ap_active: 'YES',
  ap_clients: '1', heap_free: '210000'};
const text = o => Object.entries(o).map(([k, v]) => k + '=' + v).join('\n') + '\n';
const settle = async () => { for (let i = 0; i < 40; ++i) await new Promise(r => setImmediate(r)); };

// One page instance against a scripted device. `device` is mutable.
async function boot(options = {}) {
  const device = {status: {...statusBase, ...(options.status || {})},
    wifi: JSON.parse(JSON.stringify({...wifiBase, ...(options.wifi || {})})),
    down: false, session: options.session || false, csrf: 'c'.repeat(64),
    token: 'a'.repeat(64), calls: [], confirm: true, expireOnce: false};
  const errors = [], timers = [], intervals = [];
  const dom = new JSDOM(html, {url: 'http://192.168.4.1/', runScripts: 'dangerously', pretendToBeVisual: true,
    beforeParse(w) {
      for (const [k, v] of Object.entries(options.storage || {})) w.localStorage.setItem(k, v);
      w.confirm = () => device.confirm;
      w.setTimeout = (fn, ms) => { timers.push({fn, ms}); return timers.length; };
      w.clearTimeout = () => {};
      w.setInterval = (fn, ms) => { intervals.push({fn, ms}); return intervals.length; };
      w.addEventListener('error', e => errors.push(e.message));
      w.fetch = async (url, init = {}) => {
        const method = init.method || 'GET', body = init.body == null ? '' : String(init.body);
        device.calls.push({url, method, body, csrf: (init.headers || {})['X-Csrf-Token']});
        if (device.down) throw new Error('network down');
        if (method === 'GET' && url === '/status' && device.budget)
          return {ok: false, status: 503, text: async () => '{"version":1,"ok":false,"error":"MOTION_BUDGET_UNAVAILABLE","elapsed_ms":0}'};
        if (method === 'GET' && url === '/status' && device.late)
          return {ok: false, status: 503, text: async () => '{"version":1,"ok":false,"error":"CONTROLLER_LOOP_LATE","elapsed_ms":800}'};
        const reply = (status, t) => ({ok: status === 200, status, text: async () => t,
          json: async () => JSON.parse(t)});
        if (method === 'GET') {
          if (url === '/status') return reply(200, text(device.status));
          if (url === '/wifi/status') return reply(200, JSON.stringify(device.wifi));
          return reply(404, 'not found');
        }
        if (url === '/wifi/login') {
          if (!body.startsWith('token=' + device.token)) return reply(403, 'AUTH_FAILED');
          device.session = true; device.remembered = body.endsWith('&remember=1');
          return reply(200, device.csrf);
        }
        if (url === '/wifi/session')
          return device.session || device.remembered ? (device.session = true, reply(200, device.csrf))
                                                     : reply(403, 'AUTH_FAILED');
        if (url === '/wifi/logout') { device.session = device.remembered = false; return reply(200, 'LOGGED_OUT'); }
        if (device.expireOnce) { device.expireOnce = false; return reply(403, 'AUTH_FAILED'); }
        if (!device.session || init.headers['X-Csrf-Token'] !== device.csrf) return reply(403, 'AUTH_FAILED');
        if (url === '/wifi/config') return reply(200, 'ADMIN_CLOSING_STA_TEST_RELOGIN_REQUIRED');
        return reply(200, 'QUEUED');
      };
    }});
  await settle();
  const d = dom.window.document;
  // The page's only other timer is the 6 s abort guard of a request.
  // poll() runs the scheduled refresh and returns the delay of the next one.
  const poll = async () => { const t = timers.filter(x => x.ms !== 800).pop(); timers.length = 0; assert(t, 'a poll is scheduled'); t.fn(); await settle(); return (timers.filter(x => x.ms !== 800).pop() || {}).ms; };
  const tick = () => intervals.forEach(i => i.fn());
  return {dom, d, device, errors, timers, intervals, poll, tick, q: s => d.querySelector(s),
    close: () => dom.window.close()};
}
const row = (page, list, label) => {
  const dt = [...page.d.querySelectorAll(list + ' dt')].find(e => e.textContent === label);
  return dt ? dt.nextElementSibling : null;
};

(async () => {
  let checks = 0;
  const ok = (cond, name) => { ++checks; assert(cond, name); };
  const page = await boot();
  const dot = () => page.q('#linkDot').className, txt = () => page.q('#linkText').textContent;
  const cards = () => [...page.d.querySelectorAll('.card')].map(c => c.textContent).join('|');
  ok(dot() === 'dot ok', 'initial poll ok');
  const before = cards();
  // 503 MOTION_BUDGET_UNAVAILABLE: telemetry paused, not a lost link.
  page.device.budget = true;
  for (let i = 0; i < 3; ++i) {
    const next = await page.poll();
    ok(next === 4000, 'budget refusal keeps the normal poll period (no lost-link backoff)');
    ok(dot() === 'dot warn', 'budget refusal shown as warning, not as lost link');
    ok(txt().includes('MOTION_BUDGET_UNAVAILABLE') && txt().includes('non aggiornati'), 'explicit budget reason and stale data');
    ok(page.d.body.classList.contains('stale'), 'shown values marked stale');
    ok(page.q('#staleNote').textContent.includes('movimento'), 'stale note explains the Motion pause');
    ok(cards() === before, 'no new state rendered from a refusal');
  }
  // Real network loss is still recognised.
  page.device.budget = false; page.device.down = true;
  await page.poll(); await page.poll();
  ok(dot() === 'dot bad' && txt().startsWith('Nessuna risposta'), 'network loss still reported as lost link');
  ok(page.q('#staleNote').textContent.includes('non risponde'), 'stale note for a real loss');
  // Recovery.
  page.device.down = false; await page.poll();
  ok(dot() === 'dot ok' && !page.d.body.classList.contains('stale'), 'recovery clears warning and stale');
  // Any other 503 remains a failed attempt.
  page.device.late = true; await page.poll();
  ok(dot() === 'dot warn' && txt().startsWith('Ultimo tentativo senza risposta'), 'other 503 counted as a failed attempt');
  ok(page.errors.length === 0, 'no page script errors: ' + page.errors.join(';'));
  page.close();
  console.log('PORTAL_MOTION_BUDGET_DOM PASS checks=' + checks + ' (shipped PortalPage.h script, jsdom fixtures, no device)');
})().catch(e => { console.error(e); process.exit(1); });
