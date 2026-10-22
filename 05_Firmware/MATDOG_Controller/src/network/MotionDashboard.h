#pragma once
// Operator controls added to the existing portal. Calibration, persistence,
// network and diagnostics pages remain owned by PortalPage.
#define MATDOG_MOTION_DASHBOARD R"MOTION(<style>
#simple-motion .controls{display:flex;gap:12px;align-items:end;flex-wrap:wrap}
#simple-motion button{min-height:48px}#simple-stop{background:#a52835;border-color:#ff7888;font-weight:800;min-width:130px}
#simple-state{font-size:22px}#simple-state[data-state=FAULT]{color:var(--bad)}
#simple-state[data-state=STANDING],#simple-state[data-state=READY]{color:var(--ok)}
#simple-state[data-state=MOVING],#simple-state[data-state=WALKING]{color:var(--warn)}
#simple-motion .note{max-width:850px}#simple-detail{max-height:320px;overflow:auto}
</style><script>
(()=>{'use strict';const by=id=>document.getElementById(id);
const card=document.createElement('div');card.className='card';card.id='simple-motion';
card.innerHTML=`<h2>MATDOG SIMPLE <span id="simple-state" data-state="STOPPED">COLLEGAMENTO…</span></h2>
<p id="simple-reason" role="status">Lettura dello stato reale dal Controller.</p>
<div class="controls"><label>Quota nominale (mm)<input id="simple-height" type="number" min="0" max="150" step="5" value="150"></label>
<button id="simple-stand" disabled>STAND</button><label>WALK STANDARD<select id="simple-stride"><option value="0">20 mm / ciclo</option><option value="1" selected>30 mm / ciclo</option><option value="2">40 mm / ciclo</option></select></label>
<button id="simple-walk" disabled>WALK</button><button id="simple-walk-stop" disabled>WALK STOP</button></div>
<dl id="simple-values"></dl><p class="note">STAND abilita automaticamente i servo. Il workspace STAND a contatti fissi consente 60–150 mm: una quota non raggiungibile viene rifiutata. WALK raggiunge prima 150 mm, poi usa il ciclo STANDARD di 7 s. WALK STOP conclude il passo e mantiene STAND. STOP toglie coppia; la perdita del collegamento operatore per 3 s avvia SAFE_OFF.</p>
<p class="note">La quota è una stima da encoder e IMU con ipotesi di piano di appoggio. Sulla fixture non misura la distanza dal pavimento.</p>
<details><summary>Feedback servo e manutenzione</summary><pre id="simple-detail"></pre><p>Calibrazione, generazioni NVS e certificati: scheda Diagnostics e comandi Calibration &amp; Maintenance esistenti via USB.</p></details>`;
by('dashboard').prepend(card);
const stop=document.createElement('button');stop.id='simple-stop';stop.textContent='STOP';document.querySelector('header .who').append(stop);
let status=null,received=0,sequence=0,polling=false,sending=false;const session=Date.now().toString(36);
const reasons={READY:'Pronto al comando.',STAND_REACHED:'Posa STAND raggiunta e mantenuta.',ACQUIRING_REAL_POSE:'Acquisizione della posa iniziale reale.',WALK_STANDARD:'WALK STANDARD in corso.',WALK_STOPPING:'Conclusione coordinata del passo.',STOP_REQUESTED:'STOP richiesto: torque OFF.',OPERATOR_LINK_LOST:'Collegamento operatore perso: torque OFF.',IMU_STALE:'Campione IMU scaduto: torque OFF.',HEIGHT_OUTSIDE_MATDOG_WORKSPACE:'Quota fuori dal workspace MATDOG o dai limiti reali.',SERVO_PREFLIGHT_REQUIRED:'Verifica iniziale dei profili servo in corso.',CALIBRATION_RESTORE_REQUIRED:'Calibrazione NVS non ancora ripristinata.',BMS_OR_POWER_UNAVAILABLE:'Alimentazione o BMS non disponibile.',SAFE_OFF_IN_PROGRESS:'Disabilitazione torque e verifica sui 13 servo in corso.',SERVO_COMMUNICATION_LOST:'Comunicazione servo persa: torque OFF.',SERVO_CONTROL_READ_LOST:'Lettura dei comandi servo persa: torque OFF.',SERVO_TORQUE_OR_GOAL_LOST:'Un servo non mantiene il comando richiesto: torque OFF.',SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT:'Allarme servo o limite elettrico: torque OFF.',MEASURED_MECHANICAL_LIMIT:'Un giunto ha raggiunto il limite meccanico ammesso.',NATIVE_TARGET_NOT_REACHED:'Un servo non ha raggiunto la posa entro il tempo disponibile.',INITIAL_POSE_OUTSIDE_SAFE_ENTRY:'Posa iniziale fuori dalla regione di ingresso geometricamente verificata.',REFERENCE_CHANGED_BEFORE_ENABLE:'La posa iniziale è cambiata prima dell’abilitazione.',STAND_CLEARANCE_UNAVAILABLE:'Il prossimo tratto STAND non ha spazio geometrico sufficiente.',IMU_UNAVAILABLE:'IMU non disponibile o campione non valido.',GEN1_LIMITS_OR_FINGERPRINT:'Calibrazione gen1 o limiti non corrispondenti.',SYNC_WRITE_FAILED:'Trasmissione coordinata ai servo non completata.',TORQUE_ENABLE_UNCERTAIN:'Abilitazione servo non confermata: torque OFF.',MOTION_DEADLINE_LOST:'Aggiornamento Motion interrotto: torque OFF.',FAULT_STOP_REQUIRED:'Premere STOP dopo aver risolto la causa del guasto.'};
function render(){if(!status)return;const s=status,age=performance.now()-received,fresh=age<1500;
by('simple-state').textContent=fresh?s.state:'COLLEGAMENTO PERSO';
const bar=by('motion');if(bar&&bar.firstChild)bar.firstChild.nodeValue=fresh?s.state:'COLLEGAMENTO PERSO';if(by('motionWhy'))by('motionWhy').textContent=s.reason;by('simple-state').dataset.state=s.state;
by('simple-reason').textContent=(reasons[s.reason]||s.reason)+(s.ready?'':' · '+(reasons[s.readiness_reason]||s.readiness_reason));
const available=fresh&&s.ready&&!!csrf&&!sending&&!['MOVING','WALKING','FAULT'].includes(s.state);
by('simple-stand').disabled=!available;by('simple-walk').disabled=!available;
by('simple-walk-stop').disabled=!fresh||!csrf||!['MOVING','WALKING'].includes(s.state);
const dl=by('simple-values');dl.replaceChildren();
for(const [k,v] of [['Quota richiesta',s.requested_height_mm+' mm'],['Quota stimata',s.height_valid?Number(s.height_estimate_mm).toFixed(1)+' mm':'non disponibile'],['Fase',s.phase],['SAFE_OFF',s.safe_off_verified?'13/13 verificato':s.torque_active?'torque attivo':'verifica in corso'],['Collegamento operatore',s.heartbeat?'attivo':'accesso richiesto'],['IMU',s.imu_valid?'valida · '+s.imu_age_ms+' ms':'non disponibile'],['Compensazione IMU',!s.torque_active?'non attiva':s.imu_correction_limited?'limitata dal workspace':'attiva nei limiti'],['Cicli WALK',Number(s.walk_cycles).toFixed(2)]]){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;dl.append(dt,dd)}
by('simple-detail').textContent=JSON.stringify({period_us:s.period_us,missed_releases:s.missed_releases,max_gap_us:s.max_gap_us,joints:s.joints},null,2);
}
async function command(type,params={},priority=false){
if(!csrf){needLogin();return}if(sending&&!priority)return;
if(!priority)sending=true;render();
const uptime=status?Math.floor(status.now_ms+performance.now()-received):0;
const body={version:1,id:'simple-'+session+'-'+(++sequence),sequence,issued_at_ms:uptime,ttl_ms:type==='BASE_VELOCITY'?1000:5000,owner:'DASHBOARD',type,params};
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1200);
try{const r=await fetch('/api/motion/command',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json','X-Csrf-Token':csrf},body:JSON.stringify(body),signal:abort.signal});const a=await r.json();if(!r.ok)throw Error(a.message||a.reason||r.status);by('simple-reason').textContent=(reasons[a.reason]||a.reason)+' · '+a.outcome;if(a.outcome==='REJECTED')show(reasons[a.reason]||a.reason,'warn');}
catch(e){show('Comando '+type+': '+e.message,'bad')}
finally{clearTimeout(timer);if(!priority)sending=false;render()}
}
async function poll(){if(polling)return;polling=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),800);
try{const r=await fetch('/api/motion/status',{cache:'no-store',headers:csrf?{'X-Csrf-Token':csrf}:{},signal:abort.signal});if(!r.ok)throw Error(r.status);status=await r.json();received=performance.now();render()}
catch(e){if(status&&performance.now()-received>1500){by('simple-state').textContent='COLLEGAMENTO PERSO';by('simple-stand').disabled=true;by('simple-walk').disabled=true}}
finally{clearTimeout(timer);polling=false}}
by('simple-stand').onclick=()=>command('STAND',{target_height_m:Number(by('simple-height').value)/1000});
by('simple-walk').onclick=()=>command('BASE_VELOCITY',{vx:[.020,.030,.040][Number(by('simple-stride').value)]/7,vy:0,yaw_rate:0});
by('simple-walk-stop').onclick=()=>command('BASE_VELOCITY',{vx:0,vy:0,yaw_rate:0},true);
stop.onclick=()=>command('STOP',{},true);
window.addEventListener('keydown',e=>{if(e.key==='Escape')command('STOP',{},true)});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&status?.state==='WALKING')command('BASE_VELOCITY',{vx:0,vy:0,yaw_rate:0},true)});
Promise.resolve(authReady).finally(()=>{poll();setInterval(poll,350)});
})();
</script>)MOTION"
