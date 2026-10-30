#pragma once
// Motion page and header STOP added to the existing portal. Calibration,
// persistence, network and diagnostics pages remain owned by PortalPage.
#define MATDOG_MOTION_DASHBOARD R"MOTION(<style>
#simple-motion .controls{display:flex;gap:12px;align-items:end;flex-wrap:wrap}
#simple-motion button{min-height:48px}#simple-stop{background:#a52835;border-color:#ff7888;font-weight:800;min-width:130px}
#simple-state{font-size:22px}#simple-state[data-state=FAULT]{color:var(--bad)}
#simple-state[data-state=STANDING],#simple-state[data-state=READY]{color:var(--ok)}
#simple-state[data-state=MOVING],#simple-state[data-state=WALKING]{color:var(--warn)}
#simple-motion .note{max-width:850px}#simple-detail{max-height:320px;overflow:auto}
#simple-last{font-weight:600}#simple-last[data-kind=bad]{color:var(--bad)}#simple-last[data-kind=warn]{color:var(--warn)}#simple-last[data-kind=pending]{color:var(--muted)}
#simple-joints{width:100%;border-collapse:collapse;font-size:14px;margin-top:10px}#simple-joints th,#simple-joints td{padding:4px 6px;border-bottom:1px solid var(--line);text-align:right}#simple-joints th:first-child,#simple-joints td:first-child{text-align:left}
#simple-joints tr.bad td{color:var(--bad)}
#simple-twin-frame{width:100%;height:380px;border:1px solid var(--line);border-radius:8px;background:#0c131a;margin-top:8px}
</style><script>
(()=>{'use strict';const by=id=>document.getElementById(id);
const card=document.createElement('div');card.className='card';card.id='simple-motion';
card.innerHTML=`<h2>MATDOG Controller · Motion <span id="simple-state" data-state="STOPPED">COLLEGAMENTO…</span></h2>
<p id="simple-reason" role="status">Lettura dello stato reale dal Controller.</p>
<p id="simple-last" role="status" data-kind="none">Ultimo comando: nessuno inviato da questa pagina.</p>
<div class="controls"><label>Quota STAND (mm)<input id="simple-height" type="number" min="50" max="150" step="5" value="150"></label>
<button id="simple-stand" disabled>STAND</button><button id="simple-prone" disabled title="Discesa sperimentale: termina con coppia ON. Appoggio non verificato; nessun SAFE_OFF automatico.">PRONE · sperimentale</button><label>WALK STANDARD<select id="simple-stride"><option value="0">30 mm / ciclo</option><option value="1" selected>45 mm / ciclo</option><option value="2">60 mm / ciclo</option></select></label>
<button id="simple-walk" disabled>WALK</button><button id="simple-walk-stop" disabled>WALK STOP</button></div>
<dl id="simple-values"></dl><p class="note">Un solo comando basta: STAND acquisisce la posa reale, abilita i servo e raggiunge la quota (50–150 mm, workspace STAND a contatti fissi); WALK, se serve, porta prima a STAND 150 mm e poi usa il ciclo STANDARD di 6,5 s. STAND e WALK proseguono anche se questa pagina viene chiusa: terminano solo con WALK STOP, STOP o un guasto fisico. WALK STOP conclude il passo e mantiene STAND. STOP toglie coppia ai 13 servo (pulsante, Esc, USB @STOP). Il KEY OFF del BMS sul dorso è un'interruzione hardware indipendente.</p>
<p class="note">PRONE è riconosciuto ma rifiutato finché l’appoggio finale non è qualificato; non avvia una discesa parziale. La quota stimata deriva da encoder e IMU con ipotesi di piano di appoggio: non misura la distanza dal pavimento. La posa misurata è la regione geometrica certificata in cui si trovano gli encoder; non prova l'appoggio a terra.</p>
<table id="simple-joints"><thead><tr><th>Giunto</th><th>ID</th><th>Angolo</th><th>RAW</th><th>Temp.</th><th>Coppia</th><th>Età coppia</th><th>Età feedback</th></tr></thead><tbody></tbody></table>
<details id="simple-twin"><summary>Digital Twin 3D</summary><p class="note">Viewer MATDOG servito da questo Controller (stesso indirizzo): mostra lo stato già letto da questa pagina (12 angoli ACTUAL e quaternion BNO085), nessuna richiesta né lettura servo aggiuntiva. Il modello si scarica solo a Motion fermo: già in cache si apre anche durante il movimento, altrimenti resta «Caricamento modello…» finché il pannello non viene riaperto a Motion fermo.</p><iframe id="simple-twin-frame" title="MATDOG Digital Twin"></iframe></details>
<details><summary>Dati Motion grezzi e manutenzione</summary><pre id="simple-detail"></pre><p>Calibrazione persistente: scheda Calibrazione della Dashboard. Diagnostica servo, calibrazione e persistenza: USB <code>@MODE MAINTENANCE</code> e comandi Calibration &amp; Maintenance esistenti (con Motion fermo).</p></details>`;
// Motion is the first page; the header STOP is visible from every page.
const motionPage=document.createElement('section');motionPage.id='motion-page';motionPage.append(card);by('dashboard').before(motionPage);
const tab=document.createElement('button');tab.dataset.page='motion-page';tab.textContent='Motion';tab.onclick=()=>page('motion-page');document.querySelector('nav').prepend(tab);
PAGES.push('motion-page');page('motion-page');
// The Twin loads only while open (closing unloads it) and has no poll of its
// own here: feed=parent makes it render the status this page already polls.
let twinRetryWhenIdle=false;
const twinBusy=s=>!s||s.torque_active||s.state==='MOVING'||!s.safe_off_verified;
by('simple-twin').addEventListener('toggle',e=>{twinRetryWhenIdle=e.target.open&&twinBusy(status);by('simple-twin-frame').src=e.target.open?'/twin/?feed=parent':'about:blank'});
const stop=document.createElement('button');stop.id='simple-stop';stop.textContent='STOP';document.querySelector('header .who').append(stop);
let status=null,received=0,sequence=0,polling=false,sending=false,transitAge=0,cookieAt=Date.now(),lapseAt=0;const session=Date.now().toString(36);
const reasons={PRONE_DESCENT_NOT_VALIDATED:"Discesa PRONE sperimentale in corso; collaudo fisico non eseguito.",PRONE_CONTACT_UNVERIFIED:"Discesa terminata, coppia ON. Appoggio non verificato: SAFE_OFF automatico escluso.",PRONE_REQUIRES_STAND_HOLD:"PRONE richiede uno STAND fermo con coppia ON.",PRONE_PATH_NOT_CLEAR:"Posa fuori dal passaggio PRONE ammesso.",PRONE_MEASURED_OUTSIDE_BOX:"Feedback fuori dalla box del tratto PRONE: arresto per fault.",PRONE_LANDING_NOT_QUALIFIED:'PRONE non disponibile: trasferimento del peso e appoggio ventrale non qualificati. HOLD resta attivo; STOP resta indipendente.',READY:'Pronto al comando.',STAND_REACHED:'Posa STAND raggiunta e mantenuta.',ACQUIRING_REAL_POSE:'Acquisizione della posa iniziale reale.',WALK_STANDARD:'WALK STANDARD in corso.',WALK_STOPPING:'Conclusione coordinata del passo.',STOP_REQUESTED:'STOP richiesto: torque OFF.',IMU_STALE:'Campione IMU scaduto: torque OFF.',HEIGHT_OUTSIDE_MATDOG_WORKSPACE:'Quota fuori dal workspace MATDOG o dai limiti reali.',SERVO_PREFLIGHT_REQUIRED:'Verifica iniziale dei profili servo in corso.',CALIBRATION_RESTORE_REQUIRED:'Calibrazione NVS non ancora ripristinata.',BMS_OR_POWER_UNAVAILABLE:'Alimentazione o BMS non disponibile.',SAFE_OFF_IN_PROGRESS:'Disabilitazione torque e verifica sui 13 servo in corso.',SERVO_COMMUNICATION_LOST:'Comunicazione servo persa: torque OFF.',SERVO_CONTROL_READ_LOST:'Lettura dei comandi servo persa: torque OFF.',SERVO_TORQUE_OR_GOAL_LOST:'Un servo non mantiene il comando richiesto: torque OFF.',SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT:'Allarme servo o limite elettrico: torque OFF.',MEASURED_MECHANICAL_LIMIT:'Un giunto ha raggiunto il limite meccanico ammesso.',NATIVE_TARGET_NOT_REACHED:'Un servo non ha raggiunto la posa entro il tempo disponibile.',INITIAL_POSE_OUTSIDE_SAFE_ENTRY:'Posa iniziale fuori dalla regione di ingresso geometricamente verificata.',REFERENCE_CHANGED_BEFORE_ENABLE:'La posa iniziale è cambiata prima dell’abilitazione.',STAND_CLEARANCE_UNAVAILABLE:'Il prossimo tratto STAND non ha spazio geometrico sufficiente.',STAND_PLAN_UNAVAILABLE:'Riferimento STAND non pianificabile entro i limiti: torque OFF.',WALK_SETTLING:'Assestamento finale del passo.',IMU_UNAVAILABLE:'IMU non disponibile o campione non valido.',GEN1_LIMITS_OR_FINGERPRINT:'Calibrazione gen1 o limiti non corrispondenti.',SYNC_WRITE_FAILED:'Trasmissione coordinata ai servo non completata.',TORQUE_ENABLE_UNCERTAIN:'Abilitazione servo non confermata: torque OFF.',MOTION_DEADLINE_LOST:'Aggiornamento Motion interrotto: torque OFF.',SERVO_FEEDBACK_STALE:'Feedback dei 12 servo non aggiornati.',FAULT_STOP_REQUIRED:'Premere STOP dopo aver risolto la causa del guasto.',
COMMAND_ACCEPTED:'Accettato dal Controller: l’esecuzione è confermata solo dallo stato Motion.',SAFE_OFF_REQUESTED:'Torque OFF richiesto: la verifica 13/13 è solo nella riga SAFE_OFF.',INVALID_JSON_SCHEMA:'Comando non conforme al contratto Motion.',DASHBOARD_OWNER_REQUIRED:'Owner del comando diverso da dashboard.',AUTH_FAILED:'Sessione o token CSRF non validi: accedere di nuovo.',SUPERSEDED_BY_STOP:'Annullato da uno STOP successivo.',STOP_LANE_LATE:'STOP non confermato entro 600 ms: usare anche @STOP via USB.',CONTROLLER_LOOP_LATE:'Il Controller non ha servito la richiesta in tempo.',MAILBOX_BUSY:'Controller occupato da un’altra richiesta.',EXPIRED:'Comando scaduto prima dell’esecuzione.',FUTURE_TIMESTAMP:'Timestamp del comando oltre l’uptime del Controller.',MOTION_BUSY:'Motion già in movimento.',MAINTENANCE_BUSY:'Operazione di manutenzione in corso.',SYSTEM_CONFIGURATION_BUSY:'Configurazione di rete o OTA in corso.',CALIBRATION_OR_LIMITS_NOT_READY:'Calibrazione o limiti non ancora pronti.',ACTUATOR_OWNER_BUSY:'Attuatori in uso da un’altra funzione.',IMU_ACQUISITION_TASK_REQUIRED:'Acquisizione IMU non attiva.'};
const poses={PRONE_ENTRY:'PRONE (ripiegata)',STAND_HEIGHT:'STAND (regione quote)',WALK:'WALK (regione passo)',STAND_LIFT:'transizione STAND (sollevamento)',STAND_DEPLOY:'transizione STAND (apertura)',OUTSIDE_CERTIFIED:'fuori dalle regioni certificate',UNKNOWN:'non disponibile (feedback non fresco)'};
function render(){if(!status)return;const s=status,age=Math.max(0,performance.now()-received)+transitAge;
const snapshotAge=age+(Number.isFinite(s.snapshot_age_ms)?s.snapshot_age_ms:0);
const linked=snapshotAge<1500,fresh=linked&&s.stale===false;
const aged=ms=>Number.isFinite(ms)?Math.ceil(ms+age):null;
window.mdMotionHealth={fresh,age:Math.ceil(snapshotAge)};
// READY is the executor's idle state; Motion admission is s.ready.
const shown=!linked?'COLLEGAMENTO PERSO':!fresh?'FEEDBACK NON FRESCO':s.state==='READY'&&!s.ready?'NON PRONTO':s.state;
by('simple-state').textContent=shown;
const bar=by('motion');if(bar&&bar.firstChild)bar.firstChild.nodeValue=shown;if(by('motionWhy'))by('motionWhy').textContent=s.reason;by('simple-state').dataset.state=s.state;
by('simple-reason').textContent=(reasons[s.reason]||s.reason)+(s.ready?'':' · '+(reasons[s.readiness_reason]||s.readiness_reason));
const available=fresh&&s.ready&&!!csrf&&!sending&&!['MOVING','WALKING','FAULT'].includes(s.state);
by('simple-stand').disabled=!available;by('simple-walk').disabled=!available;by('simple-prone').disabled=!available;
by('simple-walk-stop').disabled=!fresh||!csrf||!['MOVING','WALKING'].includes(s.state);
const dl=by('simple-values');dl.replaceChildren();let off=0;for(let m=s.safe_off_mask>>>0;m;m&=m-1)off++;
const active=s.active_command||{},cal=typeof st==='object'&&st?st:{};
for(const [k,v] of [['Età snapshot Motion',Math.ceil(snapshotAge)+' ms · '+(fresh?'ricevuto, feedback valido alla sorgente':'NON AGGIORNATO')],['Motion ammessa dal Controller',s.ready?'sì':'no · '+(reasons[s.readiness_reason]||s.readiness_reason)],['Esito operativo (telemetria Controller)',active.reason==='NOT_INITIALIZED'?'nessuno':active.outcome+' · '+active.reason],['Posa misurata (encoder)',poses[s.pose_region]||s.pose_region||'N/D'],['Fase',s.phase],['Quota richiesta (comando)',s.requested_height_mm+' mm'],['Quota stimata (encoder + IMU)',s.height_valid?Number(s.height_estimate_mm).toFixed(1)+' mm':'non disponibile'],['SAFE_OFF',!linked?'ultimo snapshot scaduto · verifica attuale non disponibile':s.safe_off_verified?'13/13 verificato':(s.torque_active?'torque attivo':'verifica in corso')+' · '+off+'/13 verificati'],['Sessione amministratore',s.admin_session?'valida':'non valida: accedere per comandare (un Motion accettato prosegue)'],['Calibrazione',cal.calibration_persist_verdict?cal.calibration_persist_verdict+' · gen '+cal.calibration_ack_generation:'vedi scheda Calibrazione'],['IMU',s.imu_valid&&aged(s.imu_age_ms)<=200?'valida · '+aged(s.imu_age_ms)+' ms':'non disponibile'],['Compensazione IMU',!s.torque_active?'non attiva':s.phase==='PRONE_CONTACT_UNVERIFIED'?'goal fissi; appoggio non verificato':s.imu_correction_limited?'limitata dal workspace':'attiva nei limiti'],['Cicli WALK',Number(s.walk_cycles).toFixed(2)]]){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;dl.append(dt,dd)}
// Torque comes from the control read (each joint every 1.44 s, control_age_ms),
// not from the fast feedback (age_ms): 0 OFF, 1 ON, anything else UNKNOWN.
const torque=j=>!linked||!j.valid||!(aged(j.control_age_ms)<=3000)?'UNKNOWN':j.torque===0?'OFF':j.torque===1?'ON':'UNKNOWN';
// ACTUAL angles: the telemetry block shared with the Digital Twin.
const t=s.telemetry||{},ang=t.actual_rad||t.actual_rad_by_joint||[];
const angle=i=>typeof ang[i]==='number'?(ang[i]*180/Math.PI).toFixed(1)+'°':'N/D';
const rows=[];for(const j of s.joints||[]){const tr=document.createElement('tr');if(!j.valid||!fresh||aged(j.age_ms)>250)tr.className='bad';
 for(const v of [j.joint,j.id,angle(j.slot),j.raw,j.temperature+' °C',torque(j),aged(j.control_age_ms)+' ms',aged(j.age_ms)+' ms']){const td=document.createElement('td');td.textContent=String(v);tr.append(td)}rows.push(tr)}
by('simple-joints').tBodies[0].replaceChildren(...rows);
by('simple-detail').textContent=JSON.stringify({period_us:s.period_us,missed_releases:s.missed_releases,max_gap_us:s.max_gap_us,joints:s.joints},null,2);
}
// The last command result is kept apart from the telemetry render: an HTTP
// answer or an ACK is never proof of motion, and a poll never hides it.
async function command(type,params={},priority=false,label=type,retried=false){
const result=(kind,text)=>{const e=by('simple-last');e.dataset.kind=kind;e.textContent='Ultimo comando · '+label+' · '+new Date().toLocaleTimeString('it-IT')+' · '+text;if(kind==='bad'||kind==='warn')show('Comando '+label+': '+text,kind)};
if(!csrf){result('bad','NON INVIATO: serve l’accesso amministratore'+(type==='MOTION.STOP'?'. Usare anche @STOP via USB o il KEY OFF.':'.'));needLogin();return}
if(sending&&!priority)return;
if(!priority)sending=true;result('pending','inviato, in attesa della risposta…');render();
const uptime=status?Math.floor(status.now_ms+performance.now()-received):0;
const body={version:1,id:'simple-'+session+'-'+(++sequence),sequence,issued_at_ms:uptime,ttl_ms:type==='BASE_VELOCITY'?1000:5000,owner:'dashboard',type,params};
// Longer than the Controller's own budgets (STOP lane 0.6 s, command 3 s,
// plus one read ahead in the HTTP task): a late answer is not a refusal.
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),5000);
const why=c=>c+(reasons[c]?' ('+reasons[c]+')':'');let renew=false;
// HttpTransport refuses these strictly before a command can run: every 400,
// every 403 except ACK_UNAVAILABLE (sent after execution), and the dispatch
// refusals that never queue it. Anything else without an ACK is unknown:
// CONTROLLER_LOOP_LATE may still run, STOP_LANE_LATE stays in the STOP lane.
const refused=(status,code)=>!!code&&(status===400||(status===403&&code!=='ACK_UNAVAILABLE')||(status===503&&(code==='MAILBOX_BUSY'||code==='ADMIN_GATE_CLOSED')));
const unknown=text=>result(type==='MOTION.STOP'?'bad':'warn',text+' · ESITO SCONOSCIUTO: il Controller può averlo eseguito, verificare lo stato Motion'+(type==='MOTION.STOP'?'; se SAFE_OFF non è confermato usare @STOP via USB o il KEY OFF.':'.'));
try{const r=await fetch('/api/motion/command',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json','X-Csrf-Token':csrf},body:JSON.stringify(body),signal:abort.signal});
 const text=await r.text();let a=null;try{a=JSON.parse(text)}catch(e){}
 const http='HTTP '+r.status;
 // Refused at authentication: nothing reached the Controller; renew once.
 if(r.status===403&&a&&a.error==='AUTH_FAILED'&&!retried)renew=true;
 else if(!r.ok||!a||a.ack!==true){const code=a&&a.error,detail=code?why(code)+(a.reason?' · ragione '+a.reason:''):a?'RISPOSTA_SENZA_ACK':'RISPOSTA_NON_DECODIFICABILE';
  if(refused(r.status,code))result('bad',http+' · RIFIUTATO PRIMA DELL’ESECUZIONE · errore '+detail);else unknown(http+' · errore '+detail)}
 else if(a.outcome==='REJECTED')result('warn',http+' · REJECTED dal Controller, non eseguito · '+why(a.reason));
 else result(a.outcome==='ABORTED'?'warn':'acked',http+' · '+a.outcome+' · '+why(a.reason)+(type==='MOTION.STOP'?' · SAFE_OFF non ancora verificato':' · movimento da confermare nello stato Motion'));}
catch(e){unknown(e.name==='AbortError'?'nessuna risposta entro 5 s':'errore di rete: '+e.message)}
finally{clearTimeout(timer);if(!priority)sending=false;render()}
if(!renew)return;
result('pending','sessione non valida: rinnovo e nuovo invio…');
if(await resume())return command(type,params,priority,label,true);
result('bad','HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · AUTH_FAILED: sessione non rinnovabile, accedere di nuovo'+(type==='MOTION.STOP'?'. Usare @STOP via USB o il KEY OFF.':'.'));needLogin();
}
// The status poll shares the page's single read lane (PortalPage mdNet): it
// waits for at most one 800 ms diagnostic read, and commands/STOP, outside
// the lane, always keep the second socket. It is telemetry and admin-session
// upkeep only: no Motion command depends on it.
async function poll(){if(polling)return;polling=true;
try{await window.mdNet(async()=>{const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),800);
 try{const started=performance.now();const r=await fetch('/api/motion/status',{cache:'no-store',headers:csrf?{'X-Csrf-Token':csrf}:{},signal:abort.signal});if(!r.ok)throw Error(r.status);status=await r.json();received=performance.now();transitAge=Math.max(0,received-started);render();
  if(by('simple-twin').open){
   if(twinBusy(status))twinRetryWhenIdle=true;
   if(twinRetryWhenIdle&&!twinBusy(status)){twinRetryWhenIdle=false;by('simple-twin-frame').src='/twin/?feed=parent'}
   // The viewer reads its optional freshness channels (IMU age, RAW count) under
   // channels.*; this status carries the same Controller facts as imu_valid,
   // imu_age_ms and joints[].valid. Bound here, nothing new is measured.
   by('simple-twin-frame').contentWindow?.postMessage({...status,channels:{imu:{valid:status.imu_valid,age_ms:status.imu_age_ms},raw:{count:(status.joints||[]).filter(j=>j.valid).length}}},location.origin)}}
 finally{clearTimeout(timer)}})}
catch(e){render()}
finally{polling=false}
// Admin session upkeep: an authenticated poll renews it on the Controller;
// the cookie is refreshed through /wifi/session; a lapsed one is resumed.
if(!status||!csrf)return;const now=Date.now();
if(status.admin_session===true){sessionAt=now;if(now-cookieAt>120000){cookieAt=now;resume()}}
else if(status.admin_session===false&&now-lapseAt>5000){lapseAt=now;cookieAt=now;resume()}}
// Canonical Controller contract (MotionContract.cpp): POSTURE.STAND,
// BASE_VELOCITY, MOTION.STOP, owner 'dashboard'. The parser is case-sensitive.
by('simple-stand').onclick=()=>{const mm=Number(by('simple-height').value);command('POSTURE.STAND',{target_height_m:mm/1000},false,'STAND '+mm+' mm')};
by('simple-prone').onclick=()=>command('POSTURE.LIE_DOWN',{},false,'PRONE');
by('simple-walk').onclick=()=>{const i=Number(by('simple-stride').value);command('BASE_VELOCITY',{vx:[.030,.045,.060][i]/6.5,vy:0,yaw_rate:0},false,'WALK STANDARD '+[30,45,60][i]+' mm')};
by('simple-walk-stop').onclick=()=>command('BASE_VELOCITY',{vx:0,vy:0,yaw_rate:0},true,'WALK STOP');
stop.onclick=()=>command('MOTION.STOP',{},true,'STOP');
// Each Escape press is a STOP; the key autorepeat of a held key is not.
window.addEventListener('keydown',e=>{if(e.key==='Escape'&&!e.repeat)command('MOTION.STOP',{},true,'STOP (Esc)')});
Promise.resolve(authReady).finally(()=>{poll();setInterval(()=>{render();poll()},350)});
})();
</script>)MOTION"
