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
  const eq = (a, b, name) => { ++checks; assert.strictEqual(a, b, name); };

  // ---- structure, obsolete text, no external assets --------------------------
  let p = await boot();
  eq(p.errors.length, 0, 'no script errors: ' + p.errors.join('; '));
  eq(p.d.querySelectorAll('nav button').length, 6, 'six sections');
  eq(p.d.querySelectorAll('#profiles fieldset').length, 2, 'two STA profiles');
  eq(p.q('[name=p0_ssid]').value, xss, 'SSID copied into the field as text');
  eq(p.q('[name=p0_password]').value, '', 'passwords are write only');
  eq(p.q('[name=ap_name]').value, 'MATDOG-227594', 'AP name');
  eq(p.q('#performanceForm [name=test_profile]').value, '1', 'active profile preselected');
  for (const button of p.d.querySelectorAll('nav button')) {
    button.click();
    eq(p.d.getElementById(button.dataset.page).hidden, false, 'page shown');
    eq([...p.d.querySelectorAll('section')].filter(s => s.id !== 'login' && !s.hidden).length, 1, 'one page at a time');
  }
  ok(!p.d.querySelector('script[src],link[href],img,iframe'), 'no external assets, no injected element');
  ok(!/https?:\/\/(?!192\.168\.4\.1)/.test(html), 'no external URL in the page');
  ok(!html.includes('offline candidate') && !html.includes('prove hardware sono ancora'), 'obsolete static text removed');
  ok(!/HARDWARE VALIDATED|VALIDATO/i.test(html), 'no static validation claim');
  ok(!p.q('#details').textContent.includes('password'), 'no password in diagnostics');
  ok(!/servo.*(move|muovi)|torque_on|\/servo\//i.test(html.split('<script>')[1]), 'no motion control in the page');

  // ---- system, mode, safety -----------------------------------------------------
  eq(p.q('#build').textContent.startsWith('firmware 837f5237de88'), true, 'build id from the device');
  eq(p.q('#mode').textContent, 'MAINTENANCE', 'mode shown');
  ok(p.q('#modeBar').classList.contains('maint'), 'MAINTENANCE visually distinct');
  eq(p.q('#health').textContent, 'READY', 'system health mirrors /status');
  eq(p.q('#power').textContent, 'RUN', 'power state mirrors /status');
  eq(p.q('#authority').textContent, 'NONE', 'authority mirrors /status');
  ok(p.q('#motion').textContent.startsWith('MOVIMENTO NON AUTORIZZATO'), 'motion lock evident');
  ok(!p.q('#motion').classList.contains('live'), 'locked style');
  eq(p.d.querySelectorAll('#mods .chip.ok').length, 4, 'four module chips PASS');
  eq(row(p, '#servoList', 'Torque enable').className, 'lock', 'blocked gate is not green');
  eq(row(p, '#servoList', 'Movimento').textContent, 'INIBITO', 'movement inhibited');
  eq(row(p, '#servoList', 'Popolazione attesa').textContent, '13 servo (17 ID canonici)', 'expected population');
  eq(row(p, '#servoList', 'Popolazione rilevata').className, 'na', 'no census: population not claimed');
  eq(p.q('#calBadge').textContent, 'DISPONIBILE', 'calibration availability');
  eq(row(p, '#calList', 'Generazione acknowledged').textContent, '1', 'acknowledged generation');
  eq(row(p, '#calList', 'Movimento hardware (calibrazione)').textContent, 'NON AUTORIZZATO', 'availability is not a motion permit');
  eq(row(p, '#ledList', 'SoC mostrato dai LED').textContent, '38 % · 3 segmenti', 'LED SoC from the LED snapshot');

  // ---- battery: fresh / thresholds / stale / offline -------------------------------
  eq(p.q('#soc').textContent, '38 %', 'SoC equals the BMS value');
  eq(p.q('#socBar i').style.width, '38%', 'bar equals the BMS value');
  ok(p.q('#socBar').classList.contains('ok'), 'normal charge colour');
  eq(p.q('#packV').textContent, '11.3 V · 3S', 'pack voltage');
  eq(row(p, '#bmsMore', 'Tensioni celle').textContent, '3780 · 3781 · 3779 mV', 'cell voltages');
  p.device.status.bms_soc_percent = '25.0'; await p.poll();
  ok(p.q('#socBar').classList.contains('warn') && p.q('#bmsBadge').textContent === 'BASSA', 'low charge colour');
  eq(p.q('#socBar i').style.width, '25%', 'bar follows the update');
  p.device.status.bms_soc_percent = '10.0'; await p.poll();
  ok(p.q('#socBar').classList.contains('bad') && p.q('#bmsBadge').textContent === 'CRITICA', 'critical charge colour');
  Object.assign(p.device.status,{bms_soc_percent:'33.2',bms_pack_v:'9.7',bms_cell_min_mv:'3190',led_battery_critical:'YES'});await p.poll();
  eq(p.q('#soc').textContent,'33 %','BMS estimate retained, not rewritten as physical zero');
  ok(p.q('#socBar').hidden,'critical: no SOC bar suggesting usable autonomy');
  ok(p.q('#energyState').textContent.includes('energia operativa esaurita'),'qualitative unavailable energy');
  p.device.status.bms_soc_percent='NaN';await p.poll();
  eq(p.q('#bmsBadge').textContent,'CRITICA','voltage alarm remains critical with invalid SOC');
  eq(p.q('#soc').textContent,'N/D','fresh invalid SOC is unknown, not false stale or zero');
  p.device.status.led_battery_critical='NO';
  Object.assign(p.device.status, {bms_soc_percent: '38.0', bms_fresh: 'NO', bms_sample_age_ms: '9000', bms_comm: 'TIMEOUT'}); await p.poll();
  eq(p.q('#soc').textContent, 'STALE', 'stale sample is not shown as current');
  eq(p.q('#socBar i').style.width, '0px', 'stale: empty bar');
  ok(!p.q('#socBar').classList.contains('ok'), 'stale: no green bar');
  eq(row(p, '#bmsList', 'Comunicazione').className, 'bad', 'comm failure marked');
  ok(row(p, '#bmsList', 'Ultimo valore (non attuale)'), 'old value labelled as not current');
  Object.assign(p.device.status, {bms_valid: 'NO', bms_fresh: 'NO', bms_soc_percent: '0.0', bms_pack_v: '0.0'}); await p.poll();
  eq(p.q('#soc').textContent, 'N/D', 'no sample: N/D');
  eq(p.q('#bmsBadge').textContent, 'OFFLINE', 'BMS offline badge');
  p.close();

  // ---- error states are not shown as healthy ---------------------------------------
  p = await boot({status: {system_health: 'FAULT', mode: 'RUN', imu_availability: 'FAULT', imu_detected: 'NO_RESPONSE'}});
  ok(p.q('#modeBar').classList.contains('bad'), 'fault overrides the mode colour');
  eq(p.q('#imuBadge').className, 'chip bad', 'IMU fault badge');
  p.close();
  p = await boot({status: {mode: 'RUN'}});
  ok(p.q('#modeBar').classList.contains('run') && !p.q('#modeBar').classList.contains('maint'), 'RUN distinct from MAINTENANCE');
  p.close();

  // ---- IMU update indicator needs two samples ----------------------------------------
  p = await boot();
  eq(row(p, '#imuList', 'Aggiornamento dati').className, 'na', 'first sample: rate not claimed');
  p.close();

  // ---- network: STA idle in ADMIN_AP is not a fault ----------------------------------
  p = await boot();
  eq(p.q('#apBadge').textContent, 'ATTIVO', 'AP active');
  eq(row(p, '#apList', 'Client collegati').textContent, '1', 'AP clients');
  eq(p.q('#staBadge').textContent, 'IN PAUSA', 'STA paused, not failed');
  ok(!p.q('#staBadge').classList.contains('bad'), 'STA not flagged as fault in ADMIN_AP');
  eq(row(p, '#staList', 'Ultima disconnessione STA').className, 'na', 'an old disconnect reason is informative, not a fault, in ADMIN_AP');
  eq(row(p, '#staList', 'Ultima disconnessione STA').textContent, 'codice 201 · rete non trovata', 'disconnect reason readable');
  eq(row(p, '#staList', 'Rete del router').textContent, xss, 'router SSID as text');
  ok(p.q('#staNote').textContent.includes('non indicano un guasto'), 'explicit note');
  // ---- radio role policy: MAINTENANCE keeps the AP, the saved router waits for RUN ----
  eq(row(p, '#apList', 'Policy radio').textContent, 'MAINTENANCE · AP amministrativo, sempre attivo', 'policy in force shown');
  ok(p.q('#staNote').textContent.includes('Il router verrà usato in RUN'), 'saved router is not presented as a failed connection');
  eq(p.q('#lockNote').hidden, true, 'configuration open in MAINTENANCE');
  ok(!p.q('#apOn') && !p.q('#apOff') && !p.q('[name=boot_policy]'), 'no control contradicts the mode policy');
  ok(!row(p, '#apList', 'Nuovo tentativo sul router'), 'no retry announced in MAINTENANCE');
  eq(p.q('#legacyNote').hidden, true, 'no legacy note without the legacy flag');
  eq(p.q('#apForm [name=ap_always]').type, 'hidden', 'the old AP-always preference is no longer a choice');
  p.close();
  // a record inherited from older firmware still carries ap_always: shown as ignored, kept as it is
  p = await boot({storage: {md_remember: '1'}, session: true, wifi: {ap_always: true}});
  eq(p.q('#legacyNote').hidden, false, 'legacy flag disclosed');
  ok(p.q('#legacyNote').textContent.includes('Non ha più effetto'), 'and stated as having no effect');
  p.q('#apForm').dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  eq(new URLSearchParams(p.device.calls.filter(c => c.url === '/wifi/config').pop().body).get('ap_always'), '1', 'the stored value is sent back unchanged, not cleared');
  p.close();
  p = await boot({status: {mode: 'RUN'}, wifi: {policy: {run: true, role: 'RUN_RECOVERY_AP', recovery_level: 1, retry_in_ms: 42000}}});
  eq(row(p, '#apList', 'Policy radio').textContent, 'RUN · AP di recupero: router non raggiungibile', 'recovery AP explained');
  eq(row(p, '#apList', 'Policy radio').className, 'warn', 'recovery is not shown as nominal');
  eq(row(p, '#apList', 'Nuovo tentativo sul router').textContent, 'tra 42 s', 'next router attempt shown');
  eq(p.q('#lockNote').hidden, false, 'configuration locked outside MAINTENANCE is stated');
  p.close();
  p = await boot({status: {mode: 'RUN'}, wifi: {sta_credentials: false, policy: {run: true, role: 'RUN_NO_PROFILE_AP', recovery_level: 0, retry_in_ms: 0}}});
  eq(row(p, '#apList', 'Policy radio').textContent, 'RUN · AP di recupero: nessun router configurato', 'no-profile recovery explained');
  p.close();
  p = await boot({wifi: {sta_credentials: false}});
  eq(p.q('#staBadge').textContent, 'NON CONFIGURATO', 'no router configured');
  p.close();

  // ---- polling: sequential, bounded, visible failure ------------------------------------
  p = await boot();
  const gets = p.device.calls.filter(c => c.method === 'GET').map(c => c.url);
  eq(gets.join(','), '/status,/wifi/status', 'one sequential pair per refresh');
  eq(p.timers.filter(x => x.ms !== 800).pop().ms, 4000, 'poll interval');
  eq(await p.poll(), 4000, 'steady poll interval');
  eq(p.device.calls.filter(c => c.method === 'GET').length, 4, 'exactly two requests per poll');
  p.device.down = true;
  eq(await p.poll(), 8000, 'first failure backs off');
  ok(!p.d.body.classList.contains('stale'), 'one failure is tolerated');
  eq(await p.poll(), 12000, 'second failure backs off further');
  ok(p.d.body.classList.contains('stale'), 'dashboard marked as not current');
  ok(p.q('#linkText').textContent.startsWith('Nessuna risposta'), 'communication error stated');
  ok(p.q('#linkDot').classList.contains('bad'), 'link indicator red');
  eq(await p.poll(), 15000, 'back-off is capped');
  p.device.down = false; await p.poll();
  ok(!p.d.body.classList.contains('stale') && p.q('#linkText').textContent.startsWith('Aggiornato alle'), 'recovers with a timestamp');
  p.close();

  // ---- administrator: read only by default, token login, remember ---------------------------
  p = await boot();
  eq(p.q('#adminState').textContent, 'Sola lettura', 'no session by default');
  eq(p.device.calls.filter(c => c.method === 'POST').length, 0, 'no credential request without a hint');
  p.q('#loginOpen').click(); await settle();
  eq(p.q('#login').hidden, false, 'login form shown on request');
  ok(p.q('#loginForm [name=remember]').checked, 'remember is the default');
  const form = p.q('#loginForm');
  form.elements.token.value = 'short'; form.dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  eq(p.device.calls.filter(c => c.url === '/wifi/login').length, 0, 'malformed token never sent');
  form.elements.token.value = 'b'.repeat(64); form.dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  eq(p.q('#adminState').textContent, 'Sola lettura', 'wrong token refused');
  eq(form.elements.token.value, '', 'token field cleared after an attempt');
  form.elements.token.value = p.device.token; form.dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  const login = p.device.calls.filter(c => c.url === '/wifi/login').pop();
  eq(login.body, 'token=' + p.device.token + '&remember=1', 'exact login body');
  eq(p.q('#adminState').textContent, 'Amministratore connesso', 'administrator shown');
  eq(p.q('#login').hidden, true, 'login form hidden once connected');
  eq(p.q('#logout').hidden, false, 'logout offered');
  eq(p.dom.window.localStorage.getItem('md_remember'), '1', 'only a hint is stored');
  ok(![...Array(p.dom.window.localStorage.length).keys()].some(i => p.dom.window.localStorage.getItem(p.dom.window.localStorage.key(i)).includes(p.device.token)), 'token never stored by the page');
  ok(!p.dom.window.document.cookie, 'page script sets no cookie');
  // an expired session is reopened without the token, then the write is retried
  const write = async page => { page.q('#performanceForm').dispatchEvent(new page.dom.window.Event('submit', {cancelable: true})); await settle(); };
  p.device.expireOnce = true; await write(p);
  const tail = p.device.calls.slice(-3).map(c => c.url).join(',');
  eq(tail, '/wifi/config,/wifi/session,/wifi/config', 'expired session: resume, then retry');
  eq(p.q('#adminState').textContent, 'Amministratore connesso', 'still connected');
  // explicit logout
  p.q('#logout').click(); await settle();
  eq(p.q('#adminState').textContent, 'Sola lettura', 'logged out');
  eq(p.dom.window.localStorage.getItem('md_remember'), null, 'hint removed');
  const before = p.device.calls.length; await write(p);
  eq(p.device.calls.slice(before).filter(c => c.url === '/wifi/config').length, 0, 'no write after logout');
  eq(p.q('#login').hidden, false, 'login requested for a write');
  p.close();

  // ---- remembered device: recognised at page load, and after a new AP generation ---------------
  p = await boot({storage: {md_remember: '1'}, session: true});
  eq(p.device.calls[0].url, '/wifi/session', 'session reopened before the first refresh');
  eq(p.q('#adminState').textContent, 'Amministratore connesso', 'recognised without a token');
  eq(p.q('#login').hidden, true, 'no login form for a recognised device');
  p.device.wifi.admin_generation = 4; await p.poll(); await settle();
  eq(p.device.calls.filter(c => c.url === '/wifi/session').length, 2, 'new generation: session reopened once');
  eq(p.q('#adminState').textContent, 'Amministratore connesso', 'still administrator after the AP returned');
  p.close();
  p = await boot({storage: {md_remember: '1'}});
  eq(p.q('#adminState').textContent, 'Sola lettura', 'revoked device falls back to read only');
  p.close();

  // ---- temporary session expires in the UI when the controller's does ---------------------------
  const ttl = Number(/issued_ < (\d+)/.exec(security)[1]);
  eq(Number(/SESSION_MS=(\d+)/.exec(html)[1]), ttl, 'page session window equals the controller TTL');
  eq(Number(/POLL_MS=(\d+)/.exec(html)[1]), 4000, 'documented poll interval');

  // ---- STA test: every outcome has a distinct, readable message ---------------------------------
  const cases = {CONNECTED: 'Connessione riuscita', AUTH_REJECTED: 'Credenziali rifiutate',
    AP_NOT_FOUND: 'Router non trovato', NO_IP: 'nessun indirizzo IP', TIMEOUT: 'Tempo scaduto',
    ABORTED: 'Prova interrotta', STORAGE_ERROR: 'Errore di memoria'};
  const seen = new Set();
  for (const [outcome, phrase] of Object.entries(cases)) {
    const report = {...noTest, seq: 7, outcome, ssid: 'casa', attempts: 2, elapsed_ms: 30000,
      reason: 15, rolled_back: outcome !== 'CONNECTED' && outcome !== 'STORAGE_ERROR',
      ip: outcome === 'CONNECTED' ? '192.168.1.53' : '0.0.0.0', rssi: -61, channel: 6};
    p = await boot({wifi: {sta_test: report}});
    const box = p.q('#testResult');
    eq(box.hidden, false, outcome + ' result visible');
    ok(box.textContent.includes(phrase), outcome + ' message');
    eq(box.textContent.includes('Rollback eseguito'), report.rolled_back, outcome + ' rollback statement');
    eq(box.classList.contains('ok'), outcome === 'CONNECTED', outcome + ' only success is green');
    if (outcome === 'CONNECTED') ok(box.textContent.includes('192.168.1.53') && box.textContent.includes('COMMITTED'), 'success shows the address');
    seen.add(box.textContent.split('\n')[1]);
    p.close();
  }
  eq(seen.size, Object.keys(cases).length, 'outcomes are distinguishable');

  // ---- STA test flow: accepted request, AP closed, result on return ---------------------------------
  p = await boot({storage: {md_remember: '1'}, session: true, wifi: {profiles: [{...profile, ssid: 'casa'}, {...profile, enabled: false, ssid: ''}], active_profile: 0}});
  const wform = p.q('#wifiForm');
  p.q('[name=p0_password]').value = 'router-secret';
  p.device.confirm = false; wform.dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  eq(p.device.calls.filter(c => c.url === '/wifi/config').length, 0, 'declined confirmation sends nothing');
  p.device.confirm = true; wform.dispatchEvent(new p.dom.window.Event('submit', {cancelable: true})); await settle();
  const config = p.device.calls.filter(c => c.url === '/wifi/config').pop();
  ok(config.body.includes('p0_ssid=casa') && config.body.includes('p0_password=router-secret') && config.body.includes('test_profile=0'), 'config form posted');
  ok(!config.body.includes('ap_always') && !config.body.includes('ap_name'), 'router form leaves the AP settings alone');
  eq(config.csrf, p.device.csrf, 'CSRF header on the write');
  eq(p.q('[name=p0_password]').value, '', 'password cleared after sending');
  ok(!p.d.body.textContent.includes('router-secret'), 'password never rendered');
  ok(!JSON.stringify(p.dom.window.localStorage).includes('router-secret'), 'password never stored');
  p.device.down = true; await p.poll(); await p.poll(); p.tick();
  ok(p.q('#linkText').textContent.startsWith('Prova del router in corso'), 'expected AP closure explained, not an error');
  p.device.down = false; p.device.wifi.admin_generation = 4;
  p.device.wifi.sta_test = {...noTest, seq: 9, outcome: 'AUTH_REJECTED', ssid: 'casa', reason: 15, attempts: 4, elapsed_ms: 30000, rolled_back: true};
  await p.poll(); await settle();
  ok(p.q('#message').textContent.includes('Credenziali rifiutate') && p.q('#message').textContent.includes('Rollback eseguito'), 'result announced when the AP returns');
  eq(p.q('#wifi').hidden, false, 'Wi-Fi section opened on the result');
  eq(p.q('#adminState').textContent, 'Amministratore connesso', 'no new login after the test');
  p.close();
  // a request the worker refused is reported with its reason
  p = await boot({storage: {md_remember: '1', md_watch: JSON.stringify({seq: 0, t: Date.now() - 5000, ap: 'MATDOG-227594'})}, session: true, wifi: {config_error: 6}});
  ok(p.q('#message').textContent.includes('nuova password'), 'refused request explained');
  p.close();

  console.log('PORTAL_DOM = PASS (' + checks + ' checks: dashboard values, stale/offline handling, ' +
    'mode and safety display, session resume/logout, STA test outcomes, XSS escaping, no external assets)');
})().catch(error => { console.error(error); process.exitCode = 1; });
