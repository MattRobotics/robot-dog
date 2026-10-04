// Optional DOM smoke test: NODE_PATH points to an isolated jsdom installation.
// All fetch calls are fixtures; this test never contacts a device or server.
const {JSDOM} = require('jsdom');
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const source = fs.readFileSync(path.join(__dirname, '../../src/network/PortalPage.h'), 'utf8');
const html = source.split('R"MATDOG(')[1].split(')MATDOG";')[0];
const profile = {ssid:'<img src=x onerror=alert(1)>', enabled:true, dhcp:true,
  ip:'0.0.0.0',mask:'0.0.0.0',gateway:'0.0.0.0',dns:'0.0.0.0'};
const state = {profiles:[profile,{...profile,enabled:false}],active_profile:1,
  connected:true,ssid:profile.ssid,bssid:'01:02:03:04:05:06',ip:'192.168.1.53',
  rssi:-60,channel:6,ap_active:true,ap_ssid:'MATDOG-test',sleep_effective:false,
  config_phase:'IDLE',config_busy:false,config_error:0,build_id:'offline-preview',
  ota_ingest:0,tls:false,scan:[],roam_enabled:false,roam_threshold:-80,
  roam_hysteresis:8,scan_interval_ms:60000,roam_dwell_ms:120000,
  bandwidth_mhz:20,ap_timeout_ms:900000,ap_always:false};
const errors = [];
const dom = new JSDOM(html,{url:'http://192.168.4.1/',runScripts:'dangerously',
  beforeParse(w){w.fetch=async()=>({ok:true,json:async()=>state});w.setTimeout=()=>0;
    w.addEventListener('error',e=>errors.push(e.message));}});
setImmediate(()=>{
  try {
    const d=dom.window.document;
    assert.deepEqual(errors,[]);
    assert.equal(d.querySelectorAll('nav button').length,6);
    assert.equal(d.querySelectorAll('#profiles fieldset').length,2);
    assert.equal(d.querySelector('[name=p0_ssid]').value,profile.ssid);
    assert.equal(d.querySelector('[name=p0_password]').value,'');
    assert.equal(d.querySelector('[name=ap_name]').value,'MATDOG-test');
    assert.equal(d.querySelector('#performanceForm [name=test_profile]').value,'1');
    for(const button of d.querySelectorAll('nav button')) {
      button.click();assert.equal(d.getElementById(button.dataset.page).hidden,false);
      assert.equal([...d.querySelectorAll('section')].filter(s=>s.id!=='login'&&!s.hidden).length,1);
    }
    assert(!d.querySelector('script[src],link[href],img'));
    assert(!d.querySelector('#details').textContent.includes('password'));
    console.log('PORTAL_DOM = PASS (six tabs, dual profiles, secondary selection, write-only passwords, XSS text escaping, no external assets)');
  } catch(error) {console.error(error);process.exitCode=1;}
  finally {dom.window.close();}
});
