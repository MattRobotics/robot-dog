#ifndef MATDOG_NETWORK_PORTAL_PAGE_H
#define MATDOG_NETWORK_PORTAL_PAGE_H
#include "MotionDashboard.h"
// The AP-local portal: one self-contained document, no external resource.
//
// Data model. The Dashboard renders two existing read-only endpoints and
// GET /status (controller telemetry, key=value) and
// GET /wifi/status (network state, JSON). MotionDashboard adds the Controller
// semantic /api/motion service. Every value shown is a cached
// controller value; the page never causes a BMS, IMU or servo-bus transaction.
//
// Refresh. POLL_MS = 4000: the two requests are issued one after the other on
// a single keep-alive connection (the listener allows two sockets), the DALY
// poll cadence is 2 s, and polling stops while the tab is hidden. Failures
// back off to 15 s. A 1 s local timer only repaints ages; it sends nothing.
//
// Honesty rules. Green is used only for a state the controller itself reports
// as verified (PASS, ONLINE, OK, a fresh sample). A blocked actuator gate is
// shown as locked, not as healthy. A BMS sample that is invalid or older than
// the controller's freshness limit is shown as N/D or STALE, never as a
// current number. When the device stops answering, the whole Dashboard is
// marked as not current.
//
// Battery colours are presentation only: BAT_LOW / BAT_CRIT below, or the
// controller's own battery_warning / battery_critical facts when it sets them.
namespace matdog { namespace network {
static constexpr char kPortalPage[]=R"MATDOG(<!doctype html><html lang="it"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>MATDOG · controllo locale</title><style>
:root{color-scheme:dark;font:16px/1.4 system-ui,sans-serif;--bg:#0f1720;--card:#18232f;--line:#33475a;--text:#e6edf5;--muted:#9fb2c3;--accent:#5cd6cf;--ok:#3fbf86;--warn:#e3a53a;--bad:#ea5a52;--na:#7d8b9a;--lock:#8fa3ff}
*{box-sizing:border-box}body{margin:0 auto;max-width:1040px;padding:12px;background:var(--bg);color:var(--text)}
h1{font-size:22px;margin:0;color:var(--accent);letter-spacing:.04em}h2{font-size:18px;margin:0 0 10px}h3{font-size:15px;margin:0 0 10px;display:flex;justify-content:space-between;align-items:center;gap:8px;color:var(--muted);text-transform:uppercase;letter-spacing:.05em}
.top{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}.brand{display:flex;align-items:baseline;gap:10px;flex-wrap:wrap}.who{display:flex;align-items:center;gap:8px}
.mono{font-family:ui-monospace,monospace;font-size:13px}.muted{color:var(--muted)}small{color:var(--muted);display:block;font-size:12px;text-transform:uppercase;letter-spacing:.05em}
button{padding:11px 16px;background:#20384a;color:#fff;border:1px solid #456075;border-radius:8px;font:inherit;cursor:pointer}button:hover{background:#2a5663}button:disabled{opacity:.5;cursor:default}button.small{padding:6px 12px;font-size:14px}
nav{display:flex;gap:8px;margin:12px 0;overflow-x:auto;padding-bottom:4px}nav button{white-space:nowrap;flex:0 0 auto}nav button.on{background:var(--accent);color:#06222a;border-color:var(--accent);font-weight:600}
section{margin:12px 0}section.page{background:var(--card);padding:16px;border-radius:12px}section#dashboard{background:none;padding:0}
.link{display:flex;align-items:center;gap:8px;margin-top:10px;font-size:14px;color:var(--muted)}.dot{width:10px;height:10px;border-radius:50%;background:var(--na);flex:0 0 auto}.dot.ok{background:var(--ok)}.dot.warn{background:var(--warn)}.dot.bad{background:var(--bad)}
#message{position:sticky;top:6px;z-index:2;margin-top:10px;padding:12px;border-radius:8px;background:#234e56;border-left:5px solid var(--accent);white-space:pre-line}#message.bad{background:#4a2326;border-color:var(--bad)}#message.warn{background:#4a3a1c;border-color:var(--warn)}#message.ok{background:#1d4233;border-color:var(--ok)}
.chip{display:inline-block;padding:2px 10px;border-radius:999px;font-size:13px;font-weight:600;border:1px solid var(--na);color:var(--na);text-transform:none;letter-spacing:0;white-space:nowrap}.chip.ok{border-color:var(--ok);color:var(--ok)}.chip.warn{border-color:var(--warn);color:var(--warn)}.chip.bad{border-color:var(--bad);color:#fff;background:var(--bad)}.chip.lock{border-color:var(--lock);color:var(--lock)}.chip.info{border-color:var(--accent);color:var(--accent)}
.mode{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;padding:14px 16px;border-radius:12px;border:2px solid var(--na);background:var(--card)}.mode b{font-size:30px;line-height:1.1;letter-spacing:.03em}.mode.maint{border-color:var(--warn)}.mode.maint b{color:var(--warn)}.mode.run{border-color:var(--accent)}.mode.run b{color:var(--accent)}.mode.bad{border-color:var(--bad);background:#3a1d20}.mode.bad b{color:var(--bad)}
.motion{padding:8px 12px;border-radius:8px;font-weight:700;border:2px solid var(--lock);color:var(--lock);text-align:center}.motion.live{border-color:var(--bad);color:#fff;background:var(--bad)}.motion span{display:block;font-weight:400;font-size:12px}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px;margin-top:12px}.card{background:var(--card);border-radius:12px;padding:14px;min-width:0}
.kpis{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:10px}.kpis b{font-size:19px;overflow-wrap:anywhere}.big{display:flex;align-items:baseline;gap:12px;flex-wrap:wrap}.big b{font-size:46px;line-height:1}.big span{font-size:20px;color:var(--muted)}
.bar{height:18px;border-radius:9px;background:#0c131a;border:1px solid var(--line);overflow:hidden;margin:10px 0}.bar i{display:block;height:100%;width:0;background:var(--na);transition:width .4s}.bar.ok i{background:var(--ok)}.bar.warn i{background:var(--warn)}.bar.bad i{background:var(--bad)}
dl{display:grid;grid-template-columns:minmax(36%,1fr) auto;gap:4px 12px;margin:0;align-items:baseline}dt{color:var(--muted);font-size:14px}dd{margin:0;text-align:right;overflow-wrap:anywhere;min-width:0}dd.ok{color:var(--ok)}dd.warn{color:var(--warn)}dd.bad{color:var(--bad)}dd.na{color:var(--na)}dd.lock{color:var(--lock)}
.mods{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:8px}.note{font-size:13px;color:var(--muted);margin:8px 0 0}.sub{margin-top:12px;padding-top:10px;border-top:1px solid var(--line)}.sub.first{margin:0;padding:0;border:0}.sub>b{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:6px}
details{margin-top:10px}summary{cursor:pointer;color:var(--accent);font-size:14px}details dl{margin-top:8px}
.stale .card,.stale .mode{opacity:.5}#staleNote{display:none;margin-top:12px;padding:10px;border-radius:8px;background:#4a2326;border-left:5px solid var(--bad)}.stale #staleNote{display:block}
label{display:block;margin:12px 0;font-size:14px;color:var(--muted)}input,select{display:block;width:100%;margin-top:4px;padding:11px;background:var(--bg);color:#fff;border:1px solid #456075;border-radius:6px;font:inherit}input[type=checkbox]{display:inline;width:auto;margin:0 6px 0 0}label.check{color:var(--text)}
fieldset{border:1px solid #456075;margin:14px 0;border-radius:8px;padding:6px 12px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:0 18px}
pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px;margin:8px 0 0}a{color:var(--accent)}.result{white-space:pre-line;margin:12px 0;padding:12px;border-radius:8px;border-left:5px solid var(--na);background:#111c26}.result.ok{border-color:var(--ok)}.result.bad{border-color:var(--bad)}.result.warn{border-color:var(--warn)}
[hidden]{display:none!important}</style>
<header class="top"><div class="brand"><h1>MATDOG</h1><span id="build" class="mono muted">firmware …</span></div><div class="who"><span id="adminState" class="chip">Sola lettura</span><button id="loginOpen" class="small">Accedi</button><button id="logout" class="small" hidden>Esci</button></div></header>
<div class="link" role="status"><span id="linkDot" class="dot"></span><span id="linkText">Connessione al dispositivo…</span></div>
<div id="message" role="status" hidden></div>
<nav><button data-page="dashboard" class="on">Dashboard</button><button data-page="wifi">Wi-Fi</button><button data-page="ap">Access Point</button><button data-page="performance">Performance</button><button data-page="ota">OTA</button><button data-page="diagnostics">Diagnostics</button></nav>
<section id="login" class="page" hidden><h2>Accesso amministratore</h2><p class="muted">Serve solo per modificare la configurazione. Il token è quello univoco impostato via USB (64 cifre esadecimali). L'HTTP locale è protetto dal collegamento WPA2: chi conosce la password dell'AP può leggere lo stato, ma senza token non può modificare nulla.</p><form id="loginForm"><label>Token amministratore<input type="password" name="token" minlength="64" maxlength="64" autocomplete="off" autocapitalize="off" spellcheck="false" required></label><label class="check"><input type="checkbox" name="remember" checked>Ricorda questo dispositivo</label><p class="note">Con «Ricorda» questo browser viene riconosciuto senza reinserire il token, anche dopo il riavvio del robot, finché il token amministratore non viene cambiato. Senza, l'accesso dura 5 minuti.</p><button>Accedi</button></form></section>
<section id="dashboard"><div id="modeBar" class="mode"><div><small>Modalità operativa</small><b id="mode">—</b></div><div id="motion" class="motion">MOVIMENTO NON AUTORIZZATO<span id="motionWhy"></span></div></div>
<div id="staleNote">Dati non aggiornati: il dispositivo non risponde. I valori sotto sono gli ultimi ricevuti, non lo stato attuale.</div>
<div class="cards">
<div class="card"><h3>Stato generale</h3><div class="kpis"><div><small>Sistema</small><b id="health">—</b></div><div><small>Alimentazione</small><b id="power">—</b></div><div><small>Authority</small><b id="authority">—</b></div></div><div class="mods" id="mods"></div><dl id="sysList"></dl><p class="note" id="safety"></p></div>
<div class="card"><h3>Batteria · BMS Daly<span id="bmsBadge" class="chip">N/D</span></h3><div class="big"><b id="soc">N/D</b><span id="packV"></span></div><div id="socBar" class="bar" role="meter" aria-label="Stato di carica" aria-valuemin="0" aria-valuemax="100"><i></i></div><dl id="bmsList"></dl><details><summary>Celle, MOS e allarmi</summary><dl id="bmsMore"></dl></details></div>
<div class="card"><h3>IMU BNO085<span id="imuBadge" class="chip">N/D</span></h3><dl id="imuList"></dl></div>
<div class="card"><h3>Servo e attuatori<span id="servoBadge" class="chip">N/D</span></h3><dl id="servoList"></dl></div>
<div class="card"><h3>Calibrazione<span id="calBadge" class="chip">N/D</span></h3><dl id="calList"></dl><p class="note">La calibrazione disponibile è un'evidenza salvata, non un'autorizzazione al movimento.</p></div>
<div class="card"><h3>LED<span id="ledBadge" class="chip">N/D</span></h3><dl id="ledList"></dl></div>
<div class="card"><h3>Rete</h3><div class="sub first"><b>Access Point<span id="apBadge" class="chip">N/D</span></b><dl id="apList"></dl></div><div class="sub"><b>Client STA (router)<span id="staBadge" class="chip">N/D</span></b><dl id="staList"></dl><p class="note" id="staNote"></p></div></div>
</div><details><summary>Diagnostica avanzata · /status completo</summary><pre id="rawStatus"></pre></details></section>
<section id="wifi" class="page" hidden><h2>Wi-Fi · router di casa</h2><p class="muted">In MAINTENANCE il MATDOG resta sul proprio access point: il router salvato qui viene usato da solo soltanto in RUN. «Prova e salva» è una verifica temporanea: la radio è una sola, quindi l'AP si chiude (fino a circa 40 secondi), il telefono viene scollegato e al termine l'AP torna da solo, anche se la prova fallisce. La configurazione salvata cambia solo se il router accetta le credenziali e assegna un indirizzo. Solo reti a 2,4 GHz con WPA2.</p><p class="note" id="lockNote" hidden>Il controller non è in MAINTENANCE: la configurazione di rete non è modificabile adesso.</p><div id="testResult" class="result" hidden></div><button id="scan" disabled>Scan indisponibile in ADMIN_AP</button><pre id="scanResults" hidden></pre><form id="wifiForm"><div class="grid" id="profiles"></div><label>Profilo da provare<select name="test_profile"><option value="0">Primary</option><option value="1">Secondary</option></select></label><button>Prova e salva</button></form></section>
<section id="ap" class="page" hidden><h2>Access Point</h2><p class="muted">La modalità del controller decide il ruolo della radio. In MAINTENANCE l'AP e questo portale sono sempre attivi, senza timeout. In RUN il MATDOG passa al router configurato e apre l'AP solo come recupero: nessun profilo, oppure router che non risponde o rifiuta le credenziali. Dall'AP di recupero riprova il router dopo 60 secondi, raddoppiando l'attesa a ogni fallimento fino al limite qui sotto, e mai mentre qualcuno è collegato. Quando è sul router il portale non è raggiungibile: si torna qui passando a MAINTENANCE. Ogni modifica qui sotto passa dalla prova di connessione al router della sezione Wi-Fi.</p><form id="apForm"><label>Nome AP (vuoto: MAC dispositivo)<input name="ap_name" maxlength="32"></label><label>Nuova password WPA2 (vuoto: conserva)<input name="ap_password" type="password" maxlength="63" autocomplete="new-password"></label><label>Attesa massima sull'AP di recupero prima di riprovare il router (millisecondi)<input name="ap_timeout_ms" type="number" min="60000" max="3600000" value="900000"></label><input name="ap_always" type="hidden" value="0"><p class="note" id="legacyNote" hidden>In memoria è presente la vecchia preferenza «AP sempre disponibile». Non ha più effetto: in RUN il router configurato viene comunque usato. Viene conservata così com'è.</p><input name="test_profile" type="hidden" value="0"><button>Prova e salva</button></form></section>
<section id="performance" class="page" hidden><h2>Performance</h2><p class="muted">Roaming disattivato di default. Scansioni sospese in calibrazione, RUN e OTA. Con la policy attuale la STA è attiva stabilmente solo in RUN, dove le scansioni sono sospese: roaming e scan non hanno quindi effetto pratico. Il modem sleep resta OFF perché HWCDC non prova l'apertura di una sessione USB.</p><form id="performanceForm"><label>Roaming<select name="roam_enabled"><option value="0">Disattivo</option><option value="1">Attivo</option></select></label><label>Soglia RSSI<input name="roam_threshold" type="number" min="-95" max="-50" value="-80"></label><label>Isteresi dB<input name="roam_hysteresis" type="number" min="3" max="30" value="8"></label><label>Intervallo scansioni ms<input name="scan_interval_ms" type="number" min="30000" max="3600000" value="60000"></label><label>Dwell ms<input name="roam_dwell_ms" type="number" min="60000" max="3600000" value="120000"></label><label>Larghezza banda STA<select name="bandwidth_mhz"><option value="20">HT20</option><option value="40">HT40 · da validare sul router</option></select></label><input name="test_profile" type="hidden" value="0"><button>Prova e salva</button></form></section>
<section id="ota" class="page" hidden><h2>OTA</h2><p class="muted">L'upload usa esclusivamente TLS con identità verificata e HMAC, dopo i gate di sicurezza. Il riavvio remoto resta bloccato; nessun aggiornamento è attivato da questa pagina.</p><dl id="otaList"></dl><pre id="firmware"></pre></section>
<section id="diagnostics" class="page" hidden><h2>Diagnostics</h2><p class="muted">Risposta completa di /wifi/status.</p><pre id="details"></pre></section>
<script>
const POLL_MS=4000,STALE_MS=12000,BAT_LOW=30,BAT_CRIT=15,SESSION_MS=300000;
const PAGES=['dashboard','wifi','ap','performance','ota','diagnostics'];
const $=s=>document.querySelector(s);
const E=(t,c,x)=>{const e=document.createElement(t);if(c)e.className=c;if(x!=null)e.textContent=x;return e};
let csrf='',sessionAt=0,resumeAt=0,editing=false,st=null,wf=null,lastOk=0,fails=0,timer=0,busy=false,watch=null,rv=null,generation=0;
const store={get(k){try{return localStorage.getItem(k)}catch(e){return null}},set(k,v){try{v==null?localStorage.removeItem(k):localStorage.setItem(k,v)}catch(e){}}};
const message=$('#message');
function show(text,kind){message.textContent=text;message.className=kind||'';message.hidden=!text}
message.onclick=()=>show('');
function page(name){PAGES.forEach(p=>$('#'+p).hidden=p!==name);document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.page===name))}
document.querySelectorAll('[data-page]').forEach(b=>b.onclick=()=>page(b.dataset.page));
function dur(ms){const s=Math.max(0,Math.round(ms/1000));if(s<90)return s+' s';const m=Math.floor(s/60);if(m<90)return m+' min '+(s%60)+' s';const h=Math.floor(m/60);return h<48?h+' h '+(m%60)+' min':Math.floor(h/24)+' g '+(h%24)+' h'}
function chip(id,text,kind){const e=$(id);e.textContent=text;e.className='chip '+(kind||'')}
function rows(id,list){const d=$(id);d.textContent='';for(const r of list){if(!r)continue;d.append(E('dt','',r[0]),E('dd',r[2]||'',r[1]==null||r[1]===''?'N/D':String(r[1])))}}
const CLASS={PASS:'ok',DEGRADED:'warn',FAULT:'bad'};
const yn=v=>v==='YES'?'sì':v==='NO'?'no':'N/D';
// ---- Dashboard -------------------------------------------------------------
function renderStatus(){
 const s=st;if(!s)return;
 $('#build').textContent='firmware '+(s.build_id||'N/D')+(s.uptime_ms?' · acceso da '+dur(+s.uptime_ms):'');
 $('#lockNote').hidden=s.mode==='MAINTENANCE';
 const mode=s.mode||'UNKNOWN',faulty=s.system_health==='FAULT'||(s.power_state&&s.power_state!=='RUN');
 $('#mode').textContent=mode;$('#modeBar').className='mode '+(faulty?'bad':mode==='MAINTENANCE'?'maint':mode==='RUN'?'run':'');
 // Movement is reported as possible only when every gate says so.
 const why=[];
 if(s.calibration_hardware_motion_authorized!=='YES')why.push('motion hardware NO');
 if(s.authority==='NONE')why.push('authority NONE');
 if(s.authority_inhibited==='YES')why.push('inibizione '+s.authority_inhibit_reason);
 if(s.actuator_readiness_torque_enable!=='READY')why.push('torque '+(s.actuator_readiness_torque_enable||'N/D'));
 if(s.actuator_readiness_position_command!=='READY')why.push('posizione '+(s.actuator_readiness_position_command||'N/D'));
 const m=$('#motion');m.className='motion'+(why.length?'':' live');
 m.firstChild.textContent=why.length?'MOVIMENTO NON AUTORIZZATO':'MOVIMENTO ABILITATO';$('#motionWhy').textContent=why.join(' · ');
 const h=$('#health');h.textContent=s.system_health||'N/D';h.style.color=s.system_health==='READY'?'var(--ok)':s.system_health==='FAULT'?'var(--bad)':'var(--warn)';
 const p=$('#power');p.textContent=s.power_state||'N/D';p.style.color=s.power_state==='RUN'?'':'var(--warn)';
 $('#authority').textContent=s.authority||'N/D';
 const mods=$('#mods');mods.textContent='';
 for(const [k,label] of [['imu','IMU'],['bms','BMS'],['servo','SERVO'],['led','LED']]){const v=s[k+'_availability']||'UNKNOWN';mods.append(E('span','chip '+(CLASS[v]||''),label+' '+v))}
 rows('#sysList',[['Calibrazione in corso',yn(s.calibration_holds_authority)],['Inibizione attuatori',s.authority_inhibited==='YES'?'ATTIVA · '+s.authority_inhibit_reason:'non attiva'],['Stato OTA',s.ota_state],['Memoria libera',s.heap_free?Math.round(s.heap_free/1024)+' kB (minimo '+Math.round(s.heap_min_free/1024)+' kB)':null],['Stack controller libero (minimo)',s.loop_stack_min_free?s.loop_stack_min_free+' B':null]]);
 $('#safety').textContent=why.length?'Stato di sicurezza: attuatori bloccati (fail-closed). I comandi Motion sono subordinati alle capability del Controller.':'Stato di sicurezza: gli attuatori possono essere comandati dal controller.';
 // Battery: a value is shown as current only when the controller calls it fresh.
 const valid=s.bms_valid==='YES',fresh=s.bms_fresh==='YES',soc=parseFloat(s.bms_soc_percent),bar=$('#socBar');
 const crit=s.led_battery_critical==='YES'||soc<=BAT_CRIT,low=s.led_battery_warning==='YES'||soc<=BAT_LOW;
 if(valid&&fresh&&isFinite(soc)){
  $('#soc').textContent=soc.toFixed(0)+' %';$('#packV').textContent=s.bms_pack_v+' V · 3S';
  bar.className='bar '+(crit?'bad':low?'warn':'ok');bar.firstChild.style.width=Math.max(0,Math.min(100,soc))+'%';bar.setAttribute('aria-valuenow',soc.toFixed(0));
  chip('#bmsBadge',crit?'CRITICA':low?'BASSA':s.bms_state==='CHARGING'?'IN CARICA':'OK',crit?'bad':low?'warn':'ok');
 }else{
  $('#soc').textContent=valid?'STALE':'N/D';$('#packV').textContent='';bar.className='bar';bar.firstChild.style.width='0';bar.removeAttribute('aria-valuenow');
  chip('#bmsBadge',valid?'DATO VECCHIO':s.bms_comm==='NEVER_POLLED'?'IN ATTESA':'OFFLINE',valid?'warn':'bad');
 }
 rows('#bmsList',[['Comunicazione',s.bms_comm,s.bms_comm==='OK'?'ok':'bad'],['Età ultimo dato valido',valid?dur(+s.bms_sample_age_ms):'nessun dato',fresh?'':'warn'],
  valid&&!fresh&&isFinite(soc)?['Ultimo valore (non attuale)',soc.toFixed(0)+' % · '+s.bms_pack_v+' V','na']:null,
  valid?['Stato BMS',s.bms_state]:null,valid?['Corrente',s.bms_current_a+' A']:null]);
 const alarms=s.bms_alarms||'',clear=/^[0 ]+$/.test(alarms);
 rows('#bmsMore',valid?[['Celle',s.bms_cells],['Tensioni celle',(s.bms_cell_mv||'').split(',').join(' · ')+' mV'],['Min / max / delta',s.bms_cell_min_mv+' / '+s.bms_cell_max_mv+' / '+s.bms_cell_delta_mv+' mV'],['MOS carica',s.bms_charge_mos],['MOS scarica',s.bms_discharge_mos],['Parole di allarme (grezze)',clear?'nessuna':alarms,clear?'':'warn'],['Soglie colore barra','bassa ≤ '+BAT_LOW+' % · critica ≤ '+BAT_CRIT+' %','na']]:[['Dettagli','non disponibili senza un campione valido','na']]);
 // IMU: the rotation-vector counter must advance between two polls.
 const count=+s.imu_rv_count,now=Date.now();let rate=null;
 if(rv&&isFinite(count)&&now>rv.t&&count>=rv.c)rate=(count-rv.c)*1000/(now-rv.t);
 if(isFinite(count))rv={c:count,t:now};
 chip('#imuBadge',s.imu_availability||'N/D',CLASS[s.imu_availability]);
 rows('#imuList',[['Rilevamento',s.imu_detected,s.imu_detected==='ONLINE'?'ok':s.imu_detected==='NO_RESPONSE'?'bad':'na'],['Stream',s.imu_stream],['Campioni rotation-vector',isFinite(count)?count.toLocaleString('it-IT'):null],
  ['Aggiornamento dati',rate==null?'in misura…':rate>0?'≈ '+rate.toFixed(0)+' campioni/s':'FERMO',rate==null?'na':rate>0?'ok':'bad'],['Reset a runtime',s.imu_runtime_resets,+s.imu_runtime_resets?'warn':'']]);
 // Servo: a blocked gate is the safe state, shown as locked.
 const gate=v=>v==='BLOCKED'?'lock':v==='READY'?'warn':'na',census=s.servo_census||'N/D';
 chip('#servoBadge',s.servo_availability||'N/D',CLASS[s.servo_availability]);
 rows('#servoList',[['Bus servo',s.servo_detected,s.servo_detected==='ONLINE'?'ok':s.servo_detected==='NO_RESPONSE'?'bad':'na'],['Popolazione attesa',s.servo_expected_now?s.servo_expected_now+' servo ('+s.servo_canonical+' ID canonici)':null],
  ['Popolazione rilevata',census==='NOT_RUN'?'non verificata dal boot':s.servo_census_present+' presenti · '+s.servo_census_missing+' mancanti · '+census,census==='PASS'?'ok':census==='NOT_RUN'?'na':'bad'],
  ['Autorità di controllo',s.authority],['Torque enable',s.actuator_readiness_torque_enable,gate(s.actuator_readiness_torque_enable)],['Position command',s.actuator_readiness_position_command,gate(s.actuator_readiness_position_command)],
  ['Movimento',why.length?'INIBITO':'ABILITATO',why.length?'lock':'bad']]);
 const avail=s.calibration_available;
 chip('#calBadge',avail==='YES'?'DISPONIBILE':avail==='NO'?'NON DISPONIBILE':'N/D',avail==='YES'?'ok':avail==='NO'?'warn':'');
 rows('#calList',[['Record persistente',s.calibration_persist_verdict,s.calibration_persist_verdict==='VALID_ACKNOWLEDGED'?'ok':'warn'],['Generazione acknowledged',s.calibration_ack_generation],['Record integro',yn(s.calibration_record_intact),s.calibration_record_intact==='YES'?'ok':'warn'],
  ['Sessione',s.calibration_session_state],['Restore richiesto',yn(s.calibration_restore_required),s.calibration_restore_required==='YES'?'warn':''],['Movimento hardware (calibrazione)',s.calibration_hardware_motion_authorized==='YES'?'AUTORIZZATO':'NON AUTORIZZATO',s.calibration_hardware_motion_authorized==='YES'?'bad':'lock']]);
 chip('#ledBadge',s.led_availability||'N/D',CLASS[s.led_availability]);
 rows('#ledList',[['Presentazione',s.led_presentation],['SoC mostrato dai LED',s.led_soc_valid==='YES'?(+s.led_soc_percent).toFixed(0)+' % · '+s.led_soc_segments+' segmenti':'N/D',s.led_soc_valid==='YES'?'':'na'],['In carica',yn(s.led_charging)],['Fault di carica',yn(s.led_charging_fault),s.led_charging_fault==='YES'?'bad':''],['Diagnostica anello',s.led_diagnostic]]);
 $('#rawStatus').textContent=Object.entries(s).map(([k,v])=>k+'='+v).join('\n');
 rows('#otaList',[['Build in esecuzione',s.ota_running_build_id],['Stato',s.ota_state],['Ingest compilato',yn(s.ota_ingest_compiled)],['Rollback possibile',yn(s.ota_rollback_possible)]]);
}
// ---- Network ---------------------------------------------------------------
const ROLE={MAINTENANCE_AP:['','MAINTENANCE · AP amministrativo, sempre attivo'],RUN_STA:['','RUN · router'],RUN_NO_PROFILE_AP:['warn','RUN · AP di recupero: nessun router configurato'],RUN_RECOVERY_AP:['warn','RUN · AP di recupero: router non raggiungibile']};
const TEST={CONNECTED:['ok','Connessione riuscita: il router ha accettato le credenziali e ha assegnato un indirizzo IP.'],AUTH_REJECTED:['bad','Credenziali rifiutate dal router: password errata, oppure handshake WPA2 non completato.'],AP_NOT_FOUND:['bad','Router non trovato: nessuna rete con questo nome e sicurezza compatibile a portata (solo 2,4 GHz).'],NO_IP:['bad','Associazione riuscita, ma nessun indirizzo IP: DHCP del router non ha risposto, o IP statico non valido.'],TIMEOUT:['bad','Tempo scaduto: la connessione non si è completata entro 30 secondi.'],ABORTED:['warn','Prova interrotta prima del termine (contesto critico o fault radio).'],STORAGE_ERROR:['bad','Errore di memoria: la configurazione non è stata scritta in NVS.'],RUNNING:['warn','Prova in corso…']};
const CONFIG_ERR={4:'indirizzo IP statico rifiutato dal driver',5:'richiesta non ammessa (amministrazione chiusa o controller occupato)',6:'cambiando SSID serve anche la nuova password',8:'configurazione non valida, oppure il profilo da provare non è attivo',9:'scrittura NVS fallita',10:'lettura NVS fallita',11:'fault radio',12:'profilo AP non coerente'};
const REASON={2:'autenticazione scaduta',3:'uscita volontaria',8:'uscita volontaria',15:'handshake WPA2 non completato',200:'segnale del router perso',201:'rete non trovata',202:'autenticazione rifiutata',203:'associazione rifiutata',204:'handshake WPA2 non completato',205:'connessione fallita',210:'rete non trovata con sicurezza compatibile',211:'rete non trovata con sicurezza compatibile',212:'segnale troppo debole'};
const reason=c=>'codice '+c+(REASON[c]?' · '+REASON[c]:'');
function testText(t){const d=TEST[t.outcome];if(!d)return null;let x='Ultima prova · rete «'+t.ssid+'»\n'+d[1];
 if(t.outcome==='CONNECTED')x+='\nIP '+t.ip+' · RSSI '+t.rssi+' dBm · canale '+t.channel+' · '+dur(t.elapsed_ms)+' · tentativi '+t.attempts+'\nConfigurazione salvata (COMMITTED).';
 else if(t.outcome!=='RUNNING')x+='\nDriver: '+reason(t.reason)+' · tentativi '+t.attempts+' · '+dur(t.elapsed_ms)+(t.rolled_back?'\nRollback eseguito: la configurazione precedente è intatta.':'');
 return [d[0],x]}
function renderWifi(){
 const s=wf;if(!s)return;
 chip('#apBadge',s.ap_active?'ATTIVO':'NON ATTIVO',s.ap_active?'ok':'warn');
 const pol=s.policy||{},why=ROLE[pol.role]||['',pol.role||'N/D'];
 rows('#apList',[['SSID',s.ap_ssid],['Indirizzo',s.ap_ip],['Client collegati',s.ap_clients],['Amministrazione',s.network_phase],['Policy radio',why[1],why[0]],
  pol.role==='RUN_RECOVERY_AP'?['Nuovo tentativo sul router',pol.retry_in_ms?'tra '+dur(pol.retry_in_ms):s.ap_clients?'quando nessuno è collegato':'imminente','warn']:null]);
 // One shared radio: while this page is reachable the STA is intentionally off.
 const admin=s.network_phase==='ADMIN_AP',profile=s.profiles[s.active_profile]||{},configured=s.sta_credentials;
 if(s.connected)chip('#staBadge','CONNESSO','ok');else if(admin)chip('#staBadge',configured?'IN PAUSA':'NON CONFIGURATO','');else chip('#staBadge','NON CONNESSO',configured?'bad':'');
 rows('#staList',[['Abilitato',s.sta_enabled&&configured?'sì':'no'],['Rete del router',configured?profile.ssid:'nessuna',configured?'':'na'],['Connesso',s.connected?'sì':admin?'no · radio in uso dall\'AP':'no',s.connected?'ok':'na'],
  ['IP assegnato',s.connected?s.ip:'—','na'],['RSSI',s.connected?s.rssi+' dBm':'—','na'],s.disconnect_reason?['Ultima disconnessione STA',reason(s.disconnect_reason),admin?'na':'warn']:null,
  s.sta_test.seq?['Ultima prova router',s.sta_test.outcome+(s.sta_test.outcome==='CONNECTED'?' · '+s.sta_test.ip:''),(TEST[s.sta_test.outcome]||[''])[0]]:null]);
 $('#legacyNote').hidden=!s.ap_always;
 $('#staNote').textContent=admin?(pol.run?'Il router viene riprovato dall\'AP di recupero secondo la policy; qui sopra lo stato.':'In MAINTENANCE il client STA è sospeso di proposito, anche con un router salvato: «non connesso» e 0.0.0.0 non indicano un guasto. Il router verrà usato in RUN.'):'';
 const r=$('#testResult'),t=s.sta_test.seq?testText(s.sta_test):null;r.hidden=!t;if(t){r.className='result '+t[0];r.textContent=t[1]}
 $('#details').textContent=JSON.stringify(s,null,2);$('#scanResults').textContent=JSON.stringify(s.scan,null,2);$('#scanResults').hidden=!s.scan.length;
 $('#firmware').textContent='Build '+s.build_id+' · ingest '+s.ota_ingest+' · TLS '+s.tls;
 for(const f of ['apForm','performanceForm'])if(!editing)document.getElementById(f).elements.test_profile.value=s.active_profile;
 if(!editing){s.profiles.forEach((p,i)=>{for(const key of ['ssid','enabled','dhcp','ip','mask','gateway','dns'])$('[name=p'+i+'_'+key+']').value=typeof p[key]==='boolean'?(p[key]?'1':'0'):p[key]});$('[name=ap_name]').value=s.ap_ssid;$('#wifiForm').elements.test_profile.value=s.active_profile;
  for(const key of ['roam_enabled','roam_threshold','roam_hysteresis','scan_interval_ms','roam_dwell_ms','bandwidth_mhz','ap_timeout_ms','ap_always']){const el=$('[name='+key+']');if(el)el.value=typeof s[key]==='boolean'?(s[key]?'1':'0'):s[key]}syncStatic()}
 // A new AP generation ends the old session; a remembered device reopens it.
 if(generation&&s.admin_generation!==generation&&csrf){csrf='';if(store.get('md_remember'))resume();else authUi()}
 generation=s.admin_generation;
 if(watch){
  const done=text=>{watch=null;store.set('md_watch',null);return text};
  if(s.sta_test.seq>watch.seq&&s.sta_test.outcome!=='RUNNING'){const d=testText(s.sta_test)||['warn','Prova terminata: '+s.sta_test.outcome];done();show(d[1],d[0]);page('wifi')}
  else if(s.sta_test.seq<watch.seq)show(done('Il dispositivo è stato riavviato durante la prova: esito non disponibile. La configurazione salvata è quella precedente, salvo conferma nella sezione Wi-Fi.'),'warn');
  else if(!s.config_busy&&s.sta_test.seq===watch.seq&&Date.now()-watch.t>4000)show(done('Richiesta non eseguita: '+(CONFIG_ERR[s.config_error]||'errore '+s.config_error)+'. Nessuna modifica salvata.'),'bad');
 }else if(s.ap_reload_pending)show('Transizione AP in corso: il portale si chiuderà e riaprirà.','warn');
 // A remembered device that lost its session is reopened quietly, at most
 // once every 30 s; a write also retries on demand.
 if(!csrf&&store.get('md_remember')&&Date.now()-resumeAt>30000)resume();
}
function link(){
 const age=lastOk?Date.now()-lastOk:0,dot=$('#linkDot'),text=$('#linkText');
 if(watch&&(fails||age>2*POLL_MS)){dot.className='dot warn';text.textContent='Prova del router in corso da '+dur(Date.now()-watch.t)+': AP chiuso (previsto). Se il telefono non si ricollega da solo, riconnettilo a «'+watch.ap+'».'}
 else if(!lastOk){dot.className='dot '+(fails>1?'bad':'');text.textContent=fails>1?'Dispositivo non raggiungibile. Collega il telefono al Wi-Fi MATDOG e apri http://192.168.4.1':'Connessione al dispositivo…'}
 else if(fails>=2||age>STALE_MS){dot.className='dot bad';text.textContent='Nessuna risposta da '+dur(age)+' · ultimo aggiornamento '+new Date(lastOk).toLocaleTimeString('it-IT')}
 else if(fails){dot.className='dot warn';text.textContent='Ultimo tentativo senza risposta · dati delle '+new Date(lastOk).toLocaleTimeString('it-IT')+' ('+dur(age)+' fa)'}
 else{dot.className='dot ok';text.textContent='Aggiornato alle '+new Date(lastOk).toLocaleTimeString('it-IT')+' · '+dur(age)+' fa · ogni '+POLL_MS/1000+' s'}
 document.body.classList.toggle('stale',!!lastOk&&(fails>=2||age>STALE_MS));
 // A temporary session ends on the controller after SESSION_MS.
 if(csrf&&!store.get('md_remember')&&Date.now()-sessionAt>SESSION_MS){csrf='';authUi();show('Sessione temporanea scaduta. Accedi di nuovo per modificare.','warn')}
}
async function get(path){const c=new AbortController(),t=setTimeout(()=>c.abort(),6000);try{const r=await fetch(path,{cache:'no-store',signal:c.signal});if(!r.ok)throw Error(path);return await r.text()}finally{clearTimeout(t)}}
async function refresh(){
 if(busy)return;busy=true;clearTimeout(timer);let ok=false;
 // Sequential on purpose: one keep-alive connection, never two at once.
 try{const a=await get('/status'),next={};for(const l of a.split('\n')){const i=l.indexOf('=');if(i>0)next[l.slice(0,i)]=l.slice(i+1)}const b=JSON.parse(await get('/wifi/status'));st=next;wf=b;ok=true}catch(e){}
 if(ok){fails=0;lastOk=Date.now();try{renderStatus();renderWifi()}catch(e){show('Errore di visualizzazione: '+e.message,'bad')}}else fails++;
 link();busy=false;
 if(!document.hidden)timer=setTimeout(refresh,ok?POLL_MS:Math.min(15000,POLL_MS*(1+fails)));
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)clearTimeout(timer);else refresh()});
// ---- Administrator session ---------------------------------------------------
function authUi(){const on=!!csrf;chip('#adminState',on?'Amministratore connesso':'Sola lettura',on?'ok':'');$('#logout').hidden=!on;$('#loginOpen').hidden=on;if(on)$('#login').hidden=true}
function needLogin(){$('#login').hidden=false;$('#login').scrollIntoView&&$('#login').scrollIntoView();const t=$('#loginForm').elements.token;t.focus&&t.focus()}
async function send(path,body){const r=await fetch(path,{method:'POST',cache:'no-store',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Csrf-Token':csrf},body});const t=await r.text();if(!r.ok){const e=Error(t);e.status=r.status;throw e}return t}
// Reopens the session without the token: the live session cookie, or the
// remembered-device cookie, is sent by the browser only to /wifi/session.
async function resume(){resumeAt=Date.now();try{csrf='';csrf=await send(location.protocol==='https:'?'/api/motion/session':'/wifi/session','');sessionAt=Date.now();authUi();return true}catch(e){csrf='';authUi();return false}}
async function adminPost(path,body){
 if(!csrf&&!(await resume())){needLogin();throw Error('Serve l\'accesso amministratore per modificare la configurazione.')}
 try{return await send(path,body)}catch(e){if(e.message!=='AUTH_FAILED')throw e;if(await resume())return send(path,body);needLogin();throw Error('Sessione scaduta: accedi di nuovo.')}
}
$('#loginOpen').onclick=async()=>{if(store.get('md_remember')&&await resume())show('Amministratore connesso: dispositivo riconosciuto.','ok');else needLogin()};
$('#loginForm').onsubmit=async e=>{e.preventDefault();const f=e.target,token=f.elements.token.value.trim(),remember=f.elements.remember.checked;
 if(!/^[0-9a-fA-F]{64}$/.test(token)){show('Il token deve avere esattamente 64 cifre esadecimali.','bad');return}
 try{csrf=await send(location.protocol==='https:'?'/api/motion/login':'/wifi/login','token='+token+(remember&&location.protocol!=='https:'?'&remember=1':''));sessionAt=Date.now();store.set('md_remember',remember?'1':null);store.set('md_login',String(sessionAt));authUi();show(remember?'Amministratore connesso. Questo dispositivo verrà riconosciuto senza reinserire il token.':'Amministratore connesso per 5 minuti (dispositivo non ricordato).','ok')}
 catch(x){csrf='';authUi();show(x.message==='AUTH_FAILED'?'Token rifiutato. Dopo un errore attendi qualche secondo prima di riprovare (60 s dopo cinque errori).':'Accesso non riuscito: '+x.message,'bad')}finally{f.reset()}};
$('#logout').onclick=async()=>{try{await send(location.protocol==='https:'?'/api/motion/logout':'/wifi/logout','')}catch(e){}csrf='';store.set('md_remember',null);store.set('md_login',null);authUi();show('Disconnesso. Questo dispositivo non è più ricordato.','ok')};
// ---- Forms -----------------------------------------------------------------
for(let i=0;i<2;i++){const f=E('fieldset');f.append(E('legend','',['Primary','Secondary'][i]));
 const add=(key,label,type,extra)=>{const l=E('label','',label);let c;if(type==='select'){c=E('select');for(const [v,t] of extra)c.append(Object.assign(E('option','',t),{value:v}))}else{c=E('input');c.type=type;Object.assign(c,extra||{})}c.name='p'+i+'_'+key;l.append(c);l.dataset.key=key;f.append(l);return c};
 add('ssid','Nome rete (SSID)','text',{maxLength:32,autocapitalize:'off',spellcheck:false});add('password','Password (mai mostrata; vuota = conserva)','password',{maxLength:63,autocomplete:'new-password'});
 add('enabled','Profilo','select',[['0','Non attivo'],['1','Attivo']]);add('dhcp','Indirizzo','select',[['1','Automatico (DHCP)'],['0','Statico']]).onchange=syncStatic;
 for(const [k,l] of [['ip','IP statico'],['mask','Subnet mask'],['gateway','Gateway'],['dns','DNS']])add(k,l,'text',{value:'0.0.0.0',inputMode:'decimal'});
 $('#profiles').append(f)}
function syncStatic(){for(let i=0;i<2;i++){const fixed=$('[name=p'+i+'_dhcp]').value==='0';for(const k of ['ip','mask','gateway','dns'])$('[name=p'+i+'_'+k+']').parentNode.hidden=!fixed}}
syncStatic();
const CLOSE='Il MATDOG chiuderà l\'access point per provare il router (fino a circa 40 secondi) e questo telefono verrà scollegato. L\'AP torna da solo al termine, anche se la prova fallisce. Procedere?';
for(const id of ['wifiForm','apForm','performanceForm'])document.getElementById(id).onsubmit=async e=>{e.preventDefault();const f=e.target;
 const body=new URLSearchParams(new FormData(f));
 if(id==='wifiForm'){const i=f.elements.test_profile.value;if(!$('[name=p'+i+'_ssid]').value){show('Inserisci il nome della rete da provare.','bad');return}$('[name=p'+i+'_enabled]').value='1';body.set('p'+i+'_enabled','1')}
 if(!confirm(CLOSE))return;
 try{const seq=wf?wf.sta_test.seq:0,ap=wf?wf.ap_ssid:'MATDOG';await adminPost('/wifi/config',body);
  watch={seq,t:Date.now(),ap};store.set('md_watch',JSON.stringify(watch));editing=false;f.querySelectorAll('input[type=password]').forEach(p=>p.value='');
  show('Richiesta accettata. L\'AP si chiude tra un secondo per la prova: resta su questa pagina, l\'esito compare quando l\'AP ritorna.','warn')}
 catch(x){show(x.message==='REFUSED_BUSY_OR_RECOVERY'?'Richiesta rifiutata: la configurazione di rete si modifica solo in MAINTENANCE, a controller non occupato.':x.message,'bad')}};
$('#scan').onclick=async()=>{try{show(await adminPost('/wifi/scan',''))}catch(x){show(x.message,'bad')}};
document.querySelectorAll('form').forEach(f=>{if(f.id!=='loginForm')f.addEventListener('input',()=>editing=true)});
// ---- Start -----------------------------------------------------------------
try{const w=JSON.parse(store.get('md_watch'));if(w&&Date.now()-w.t<600000)watch=w;else store.set('md_watch',null)}catch(e){}
authUi();setInterval(link,1000);
(async()=>{if(store.get('md_remember')||Date.now()-(+store.get('md_login')||0)<SESSION_MS)await resume();refresh()})();
</script>)MATDOG" MATDOG_MOTION_DASHBOARD "</html>";
} }
#endif
