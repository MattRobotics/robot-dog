#pragma once
// Motion page and header STOP added to the existing portal. Calibration,
// persistence, network and diagnostics pages remain owned by PortalPage.
#define MATDOG_MOTION_DASHBOARD R"MOTION(<style>
header.top{position:sticky;top:0;z-index:10;padding:8px 0;background:var(--bg)}
header.top .who{flex-wrap:wrap;max-width:100%}
#simple-motion .cockpit{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:18px;align-items:start;margin-bottom:18px}
#simple-motion .controls{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;align-items:end}
#simple-motion .controls label{grid-column:1/-1;margin:0}
#simple-controls{scroll-margin-top:130px}
/* The command column grows with its contents, including future controls. */
#simple-motion .command-panel{min-width:0}
#simple-twin{min-width:0;text-align:center}
#simple-twin h3{margin:0;flex-wrap:wrap;text-align:left}
#simple-twin-state{font-size:13px;color:var(--muted);margin:6px 0;text-align:left;height:1.5em;line-height:1.5em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#simple-motion button{min-height:48px}#simple-stop{min-height:48px;background:#a52835;border-color:#ff7888;font-weight:800;min-width:130px}
/* Fixed slots (dev.17): the texts above the joystick change with every poll and
   command, their height never does (ellipsis or two lines; full text in title and
   in the diagnostics log), so nothing moves the pad or a button under a finger. */
#simple-state{display:block;font-size:22px;color:var(--muted);height:1.4em;line-height:1.4em;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#simple-reason,#simple-last{display:-webkit-box;-webkit-box-orient:vertical;-webkit-line-clamp:2;overflow:hidden;height:2.8em;line-height:1.4em;margin:6px 0}
#simple-log{margin:8px 0;padding-left:20px;font-size:13px}#simple-log li[data-kind=bad]{color:var(--bad)}#simple-log li[data-kind=warn]{color:var(--warn)}#simple-state[data-tone=bad]{color:var(--bad)}
#simple-state[data-tone=ok]{color:var(--ok)}#simple-state[data-tone=warn]{color:var(--warn)}
#simple-motion .note{max-width:850px}#simple-detail{max-height:320px;overflow:auto}
#simple-last{font-weight:600}#simple-last[data-kind=bad]{color:var(--bad)}#simple-last[data-kind=warn]{color:var(--warn)}#simple-last[data-kind=pending]{color:var(--muted)}
#simple-joints{width:100%;border-collapse:collapse;font-size:14px;margin-top:10px}#simple-joints th,#simple-joints td{padding:4px 6px;border-bottom:1px solid var(--line);text-align:right}#simple-joints th:first-child,#simple-joints td:first-child{text-align:left}
#simple-joints tr.bad td{color:var(--bad)}
#simple-twin-frame{display:block;width:100%;height:clamp(340px,56vh,560px);border:1px solid var(--line);border-radius:8px;background:#0c131a;margin:8px auto 0}
#simple-joystick{margin:14px 0;padding:12px;border:1px solid var(--line);border-radius:10px;background:#101a23;color:var(--muted)}
#simple-joystick .joystick-title{display:flex;align-items:center;justify-content:space-between;gap:8px}
#simple-joystick h3{margin:0;font-size:15px;color:var(--text)}
#simple-joystick .joystick-badge{padding:2px 7px;border:1px solid #55717e;border-radius:999px;color:#b8d3df;font-size:11px;white-space:nowrap}
#simple-joystick p{margin:5px 0 8px;font-size:12px}
#simple-joystick .joystick-pad{display:flex;align-items:center;justify-content:center;gap:12px}
#simple-joystick svg{display:block;width:166px;height:166px;max-width:70%;flex-shrink:0}
#simple-joystick .joystick-yaw{display:flex;flex-direction:column;align-items:center;gap:4px;font-size:11px}
#simple-joystick .joystick-yaw button{display:grid;place-items:center;width:42px;min-height:42px;height:42px;padding:0;border:1px solid #405360;border-radius:50%;background:#182630;color:#b8d3df;font-size:29px;line-height:1}
#simple-joystick #joy-pad,#simple-joystick .joystick-yaw button{touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}#simple-joystick #joy-pad{cursor:grab}#simple-joystick[data-active=true] #joy-pad{cursor:grabbing}
#simple-joystick[aria-disabled=true] #joy-pad{opacity:.55;cursor:not-allowed}#simple-joystick .joystick-yaw button:disabled{opacity:.45}#joy-state{color:var(--text)}
@media(max-width:940px){#simple-motion .cockpit{grid-template-columns:minmax(0,1fr)}#simple-motion .controls{grid-template-columns:repeat(4,minmax(0,1fr))}#simple-motion .controls label{grid-column:span 2}}
@media(max-width:600px){#simple-motion .controls{grid-template-columns:repeat(2,minmax(0,1fr))}#simple-motion .controls label{grid-column:1/-1}#simple-twin-frame{height:clamp(280px,44vh,400px)}#simple-joints{display:block;overflow-x:auto;font-size:13px}#simple-stop{min-width:90px}}
</style><script>
(()=>{'use strict';const by=id=>document.getElementById(id);
const card=document.createElement('div');card.className='card';card.id='simple-motion';
card.innerHTML=`<h2>MATDOG Controller · Motion <span id="simple-state" data-state="STOPPED" data-tone="muted">COLLEGAMENTO…</span></h2>
<div class="cockpit"><div id="simple-twin" aria-labelledby="simple-twin-title"><h3 id="simple-twin-title">Digital Twin 3D</h3><iframe id="simple-twin-frame" title="MATDOG Digital Twin"></iframe><p id="simple-twin-state" role="status">Modello 3D non ancora caricato.</p></div><div class="command-panel" id="simple-controls" tabindex="-1" aria-label="Comandi Motion">

<p id="simple-reason" role="status">Lettura dello stato reale dal Controller.</p>
<p id="simple-last" role="status" data-kind="none">Ultimo comando: nessuno inviato da questa pagina.</p>
<div class="controls"><label>Quota STAND (mm)<input id="simple-height" type="number" min="50" max="150" step="1" value="150"></label>
<button id="simple-stand" disabled>STAND</button><button id="simple-prone" disabled title="Discesa PRONE: al termine verificato la coppia viene rilasciata (SAFE_OFF ordinario, PRONE_REST). Appoggio non verificato.">PRONE</button><label>WALK STANDARD<select id="simple-stride"><option value="0">30 mm / ciclo</option><option value="1" selected>45 mm / ciclo</option><option value="2">60 mm / ciclo</option></select></label>
<button id="simple-walk" disabled>WALK</button><button id="simple-walk-stop" disabled>WALK STOP</button></div>
<section id="simple-joystick" aria-label="Joystick avanti/indietro, laterale e rotazione" aria-disabled="true" data-active="false">
<div class="joystick-title"><h3>Joystick</h3><span class="joystick-badge">VERIFICATO SU HARDWARE (dev.15)</span></div>
<p>Trascinare: avanti/indietro fino a 6,9 mm/s e laterale fino a 3,1 mm/s, proporzionali; tenere premuto ↶/↷ per ruotare fino a 1,8°/s. Combinazioni scalate nel limite del Controller. Al rilascio WALK STOP; senza comandi per 1 s il Controller esegue WALK STOP da solo. Cambio avanti/indietro: il passo si conclude, poi si riparte.</p>
<div class="joystick-pad"><svg id="joy-pad" viewBox="0 0 180 180" xmlns="http://www.w3.org/2000/svg" aria-label="Velocità avanti/indietro e laterale">
<circle cx="90" cy="90" r="58" fill="#15242f" stroke="#3c5667" stroke-width="2"/>
<circle cx="90" cy="90" r="7" fill="none" stroke="#294454" stroke-dasharray="3 3"/>
<path d="M90 32V148" stroke="#3c6a80" stroke-width="3"/><path d="M32 90H148" stroke="#3c6a80" stroke-width="3"/>
<circle id="joy-knob" cx="90" cy="90" r="22" fill="#243e4e" stroke="#6b9cb2" stroke-width="2"/>
<path d="M90 26l-7 10h14zM90 154l-7-10h14z" fill="#acc5d0"/><path d="M26 90l10-7v14zM154 90l-10-7v14z" fill="#acc5d0"/>
<g fill="#a8bdc9" font-family="system-ui,sans-serif" font-size="9" text-anchor="middle"><text x="90" y="17">Avanti</text><text x="90" y="173">Indietro</text><text x="16" y="112">Sinistra</text><text x="164" y="112">Destra</text></g>
</svg><div class="joystick-yaw"><button id="joy-left" type="button" disabled title="Tenere premuto: rotazione antioraria">↶</button><small>Rotazione</small><button id="joy-right" type="button" disabled title="Tenere premuto: rotazione oraria">↷</button></div></div>
<p id="joy-state" role="status">Richiesto: neutro.</p></section>
</div></div>
<p class="note">STOP richiede l’arresto al Controller: può mantenere la coppia ON. SAFE_OFF verifica coppia OFF sui 13 servo. WALK STOP mantiene STAND.</p>
<details id="simple-diagnostics"><summary>Stato e diagnostica Motion</summary><h4>Ultimi esiti dei comandi (dal più recente)</h4><ol id="simple-log"></ol><dl id="simple-values"></dl>
<p class="note">ACTUAL e assetto BNO085 dallo stesso stato Motion, senza letture servo aggiuntive. Modello caricato a robot fermo (anche con coppia ON, dai file nella cache del browser) e mantenuto durante Motion; a cache fredda con coppia ON attende torque OFF.</p><p class="note">Un solo comando basta: STAND acquisisce la posa reale, abilita i servo e raggiunge la quota (50–150 mm, workspace STAND a contatti fissi); WALK, se serve, porta prima alla propria quota nominale indicata dal Controller e poi usa il ciclo STANDARD di 6,5 s. STAND e WALK proseguono anche se questa pagina viene chiusa: terminano solo con WALK STOP, STOP o un guasto fisico. WALK STOP risponde con ACK e poi conclude il passo: normalmente circa 7–9 s, più assestamento; durante l’avvio può richiedere più tempo. Mantiene STAND e coppia ON. STOP (pulsante, Esc, USB @STOP) richiede l’arresto contestuale; completamento, coppia e supporto sono indicati dal Controller. Per torque OFF globale: USB @SERVO SAFE_OFF 51, con verifica 13/13. SAFE_OFF non interrompe l'alimentazione. Il KEY OFF del BMS sul dorso è un'interruzione hardware indipendente.</p>
<p class="note">PRONE è disponibile da STAND/HOLD: esegue la discesa e, raggiunta la posa finale, rilascia la coppia con il SAFE_OFF ordinario (PRONE_REST, verifica OFF sui 13 servo). L'appoggio passivo non è misurato. STAND riparte dalla posa reale assestata: acquisizione, verifica della regione certificata, riabilitazione. La quota stimata deriva da encoder e IMU con ipotesi di piano di appoggio: non misura la distanza dal pavimento. La posa misurata è la regione geometrica certificata in cui si trovano gli encoder; non prova l'appoggio a terra.</p>
<table id="simple-joints"><thead><tr><th>Giunto</th><th>ID</th><th>Angolo</th><th>RAW letto (FAST)</th><th>Temp.</th><th>Coppia</th><th>Età CONTROL (coppia/goal)</th><th>Età FAST (posizione)</th></tr></thead><tbody></tbody></table>

<button id="simple-trace">Scarica diagnostica Motion di questa pagina</button><pre id="simple-detail"></pre><p>Calibrazione persistente: scheda Calibrazione della Dashboard. Diagnostica servo, calibrazione e persistenza: USB <code>@MODE MAINTENANCE</code> e comandi Calibration &amp; Maintenance esistenti (con Motion fermo).</p></details>`;
// Motion is the first page; the header STOP is visible from every page.
const motionPage=document.createElement('section');motionPage.id='motion-page';motionPage.append(card);by('dashboard').before(motionPage);
const tab=document.createElement('button');tab.dataset.page='motion-page';tab.textContent='Motion';tab.onclick=()=>page('motion-page');document.querySelector('nav').prepend(tab);
PAGES.push('motion-page');page('motion-page');
// The Twin renders the status this page already polls (feed=parent: no read of
// its own, no servo read) and is kept during Motion. dev.17: it is also started
// with torque ON, so a page reload in STAND/HOLD keeps it. The Controller serves
// its large files only at torque OFF or as 304 from the browser cache
// (handleTwin); the load costs this page's main thread 0.2-0.4 s once (Chrome,
// phone-class CPU throttling), so it never starts while the robot moves or
// walks or the joystick is held. A load that could not complete (cold cache with
// torque ON, or a transfer cut when Motion engaged) is retried once at torque OFF.
let twinStarted=false,twinReady=false,twinRetryWhenIdle=false;
const twinIdle=s=>!!s&&!s.torque_active&&s.state!=='MOVING'&&!!s.safe_off_verified;  // large files served
const twinHold=s=>!s||s.state==='MOVING'||s.state==='WALKING'||joyActive;                // no load now
function prepareTwin(){
 if(twinReady)return;
 if(twinStarted&&!twinIdle(status))twinRetryWhenIdle=true;
 if(twinHold(status))return;
 if(!twinStarted||(twinRetryWhenIdle&&twinIdle(status))){twinStarted=true;twinRetryWhenIdle=!twinIdle(status);by('simple-twin-frame').src='/twin/?feed=parent'}
}
// The Twin's own height is applied at once, except while the joystick is held
// (dev.17): then it waits for the release, so the page never moves under it.
let twinHeight=0;
function twinResize(){if(!twinHeight||joyActive)return;const frame=by('simple-twin-frame');frame.style.height=Math.ceil(twinHeight)+frame.offsetHeight-frame.clientHeight+'px';twinHeight=0}
window.addEventListener('message',e=>{
 if(e.origin!==location.origin||e.source!==by('simple-twin-frame').contentWindow)return;
 if(e.data?.type==='MATDOG_TWIN_HEIGHT'&&Number.isFinite(e.data.height)&&e.data.height>0&&e.data.height<10000){twinHeight=e.data.height;twinResize()}
 if(e.data?.type==='MATDOG_TWIN_READY'){twinReady=true;twinRetryWhenIdle=false;render()}
});
const stop=document.createElement('button');stop.id='simple-stop';stop.textContent='STOP';document.querySelector('header .who').append(stop);
const READ_ABORT_MS=3500,READ_OVERLAP_MS=700,readStarts=[];let readSequence=0,appliedSequence=0;
let status=null,received=0,sequence=0,sending=false,transitAge=0,cookieAt=Date.now(),lapseAt=0;const session=Date.now().toString(36);
let latestCommand=null,commandNotice='';
const trace=window.matdogMotionTrace={version:1,started_utc:new Date().toISOString(),commands:[],samples:[],poll_errors:[],poll_skipped:0,late_dropped:0};
const keep=(list,item,max)=>{list.push(item);if(list.length>max)list.shift()};
by('simple-trace').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify(trace,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download='matdog-motion-trace.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
const reasons={PRONE_DESCENT_NOT_VALIDATED:"Discesa PRONE in corso; al termine la coppia viene rilasciata (PRONE_REST).",PRONE_REST:"PRONE a riposo: coppia OFF (verificata sui 13 servo al termine). Appoggio passivo non misurato. STAND riacquisisce la posa reale e riabilita la coppia.",STAND_PATH_NOT_CLEAR:"Posa reale fuori dalla regione di ingresso certificata: rialzata rifiutata.",NOT_WALKING:"Nessun passo in corso da fermare.",PRONE_REQUIRES_STAND_HOLD:"PRONE richiede uno STAND fermo con coppia ON.",PRONE_PATH_NOT_CLEAR:"Posa fuori dal passaggio PRONE ammesso.",PRONE_MEASURED_OUTSIDE_BOX:"Feedback fuori dalla box del tratto PRONE: arresto per fault.",PRONE_NOT_COMPLETED_STOPPING:"PRONE non completato entro il tempo: arresto controllato con coppia ON, nessun rilascio a quota intermedia.",PRONE_OUTSIDE_SEGMENT_SUPPORT_HELD:"PRONE interrotto: posa sotto carico fuori dal tratto ma dentro una regione certificata; corpo sostenuto, coppia ON, protezioni attive. Usare STAND (se ammesso) oppure SAFE_OFF con il robot sostenuto.",PRONE_NOT_COMPLETED_SUPPORT_HELD:"PRONE non completato: corpo sostenuto, coppia ON e protezioni attive. Usare STAND (se la posa è certificata) oppure SAFE_OFF con il robot sostenuto.",PRONE_LANDING_NOT_QUALIFIED:'PRONE non disponibile: trasferimento del peso e appoggio ventrale non qualificati. HOLD resta attivo; STOP resta indipendente.',READY:'Pronto al comando.',STAND_REACHED:'Posa STAND raggiunta e mantenuta.',HEIGHT_CHANGE:'Cambio di quota in corso.',HEIGHT_PATH_NOT_CLEAR:'Cambio di quota non ammesso dalla posa reale attuale: nessun movimento.',ACQUIRING_REAL_POSE:'Acquisizione della posa iniziale reale.',WALK_STANDARD:'WALK STANDARD in corso.',WALK_STOPPING:'Conclusione coordinata del passo.',STOP_REQUESTED:'STOP richiesto: attendere il risultato del Controller.',STOP_REQUESTED_FAULT_LATCHED:'STOP richiesto con fault già presente: attendere la verifica coppia OFF sui 13 servo.',SAFE_OFF_VERIFIED_FAULT_LATCHED:'SAFE_OFF verificato sui 13 servo; il fault originale resta attivo.',SAFE_OFF_IN_PROGRESS_FAULT_LATCHED:'Verifica coppia OFF in corso; il fault originale resta attivo.',STAND_FROM_STOPPED_WALK:'Recupero STAND dal WALK arrestato in corso; coppia mantenuta e supporto non verificato.',STOP_HOLD_TRACKING_UNSAFE:'Posa STOP fuori dai limiti di mantenimento: coppia OFF.',STOP_RECOVERY_PATH_NOT_CLEAR:'Recupero STAND rifiutato: percorso del gait fuori dai limiti ammessi.',STOP_IN_PROGRESS:'Arresto controllato in corso, coppia mantenuta.',STOPPED_SUPPORT_UNVERIFIED:'Arresto verificato, coppia ON.',SAFE_OFF_VERIFIED:'SAFE_OFF verificato: coppia OFF sui 13 servo.',SAFE_OFF_STAND_REFUSED:'STAND rifiutato dopo SAFE_OFF: nessuna riabilitazione implicita.',STOP_RECOVERY_SWING_UNVERIFIED:'Recupero STAND rifiutato: swing non qualificato.',STOP_RECOVERY_WALK_UNVERIFIED:'Recupero STAND da WALK non qualificato.',STOP_RECOVERY_POSTURE_UNQUALIFIED:'Recupero STAND da posa intermedia non qualificato.',STOP_RECOVERY_FEEDBACK_UNAVAILABLE:'Feedback non disponibile per il recupero STAND.',STOP_RECOVERY_TORQUE_UNVERIFIED:'Coppia non verificata per il recupero STAND.',STOP_IMU_REFERENCE_NOT_DIFFERENTIABLE:'Raccordo della correzione IMU non qualificato: fallback SAFE_OFF.',IMU_STALE:'Campione IMU scaduto: torque OFF.',HEIGHT_OUTSIDE_MATDOG_WORKSPACE:'Quota fuori dal workspace MATDOG o dai limiti reali.',SERVO_PREFLIGHT_REQUIRED:'Verifica iniziale dei profili servo in corso.',CALIBRATION_RESTORE_REQUIRED:'Calibrazione NVS non ancora ripristinata.',BMS_OR_POWER_UNAVAILABLE:'Alimentazione o BMS non disponibile.',SAFE_OFF_IN_PROGRESS:'Disabilitazione torque e verifica sui 13 servo in corso.',SERVO_COMMUNICATION_LOST:'Comunicazione servo persa: torque OFF.',SERVO_CONTROL_READ_LOST:'Lettura dei comandi servo persa: torque OFF.',SERVO_TORQUE_OR_GOAL_LOST:'Un servo non mantiene il comando richiesto: torque OFF.',SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT:'Allarme servo o limite elettrico: torque OFF.',MEASURED_MECHANICAL_LIMIT:'Un giunto ha raggiunto il limite meccanico ammesso.',NATIVE_TARGET_NOT_REACHED:'Un servo non ha raggiunto la posa entro il tempo disponibile.',INITIAL_POSE_OUTSIDE_SAFE_ENTRY:'Posa iniziale fuori dalla regione di ingresso geometricamente verificata.',REFERENCE_CHANGED_BEFORE_ENABLE:'La posa iniziale è cambiata prima dell’abilitazione.',STAND_CLEARANCE_UNAVAILABLE:'Il prossimo tratto STAND non ha spazio geometrico sufficiente.',STAND_PLAN_UNAVAILABLE:'Riferimento STAND non pianificabile entro i limiti: torque OFF.',WALK_SETTLING:'Assestamento finale del passo.',STAND_FROM_STOPPED_PRONE:'Ripresa STAND dalla posa mantenuta dallo STOP (PRONE o salita), coppia mantenuta.',WALK_JOYSTICK:'Joystick: passo continuo in corso.',JOYSTICK_TIMEOUT:'Joystick: nessun comando entro 1 s, il Controller conclude il passo (WALK STOP).',JOYSTICK_REVERSING:'Joystick: cambio di verso, il passo si conclude prima di ripartire.',JOYSTICK_VELOCITY_RANGE:'Velocità oltre il limite del Controller.',WALK_STANDARD_ACTIVE:'WALK STANDARD in corso: WALK STOP prima del joystick.',WALK_STOPPING:'Conclusione del passo in corso.',GAIT_NOT_IN_SIMPLE_V1:'Andatura non disponibile.',IMU_UNAVAILABLE:'IMU non disponibile o campione non valido.',GEN1_LIMITS_OR_FINGERPRINT:'Calibrazione gen1 o limiti non corrispondenti.',SYNC_WRITE_FAILED:'Trasmissione coordinata ai servo non completata.',TORQUE_ENABLE_UNCERTAIN:'Abilitazione servo non confermata: torque OFF.',MOTION_DEADLINE_LOST:'Aggiornamento Motion interrotto: torque OFF.',SERVO_FEEDBACK_STALE:'Feedback dei 12 servo non aggiornati.',FAULT_STOP_REQUIRED:'Fault persistente: STOP non riabilita il robot.',
COMMAND_ACCEPTED:'Accettato dal Controller: l’esecuzione è confermata solo dallo stato Motion.',SAFE_OFF_REQUESTED:'Torque OFF richiesto: la verifica 13/13 è solo nella riga SAFE_OFF.',INVALID_JSON_SCHEMA:'Comando non conforme al contratto Motion.',DASHBOARD_OWNER_REQUIRED:'Owner del comando diverso da dashboard.',AUTH_FAILED:'Sessione o token CSRF non validi: accedere di nuovo.',SUPERSEDED_BY_STOP:'Annullato da uno STOP successivo.',STOP_LANE_LATE:'STOP non confermato entro 600 ms: usare anche @STOP via USB.',CONTROLLER_LOOP_LATE:'Il Controller non ha servito la richiesta in tempo.',MAILBOX_BUSY:'Controller occupato da un’altra richiesta.',EXPIRED:'Comando scaduto prima dell’esecuzione.',FUTURE_TIMESTAMP:'Timestamp del comando oltre l’uptime del Controller.',MOTION_BUSY:'Motion già in movimento.',MAINTENANCE_BUSY:'Operazione di manutenzione in corso.',SYSTEM_CONFIGURATION_BUSY:'Configurazione di rete o OTA in corso.',CALIBRATION_OR_LIMITS_NOT_READY:'Calibrazione o limiti non ancora pronti.',ACTUATOR_OWNER_BUSY:'Attuatori in uso da un’altra funzione.',IMU_ACQUISITION_TASK_REQUIRED:'Acquisizione IMU non attiva.'};
const poses={PRONE_ENTRY:'PRONE (ripiegata)',STAND_HEIGHT:'STAND (regione quote)',WALK:'WALK (regione passo)',STAND_LIFT:'STAND bassa o sollevamento',STAND_DEPLOY:'transizione STAND (apertura)',OUTSIDE_CERTIFIED:'fuori dalle regioni certificate',UNKNOWN:'non disponibile (feedback non fresco)'};
// #simple-last is a fixed two-line slot (dev.17): the full text stays in its
// title and every distinct HTTP result or terminal outcome in the diagnostics log.
function lastShow(kind,text,logged=kind!=='pending'){const e=by('simple-last');e.dataset.kind=kind;e.textContent=e.title=text;
 const log=by('simple-log');if(!logged||log.firstChild?.textContent===text)return;
 const li=document.createElement('li');li.dataset.kind=kind;li.textContent=text;log.prepend(li);while(log.children.length>20)log.lastChild.remove()}
function render(){if(!status)return;const s=status,age=Math.max(0,performance.now()-received)+transitAge;
const snapshotAge=age+(Number.isFinite(s.snapshot_age_ms)?s.snapshot_age_ms:0);
// Source freshness: the 250 ms servo-feedback limit applies to j.age_ms as the
// Controller measured it at capture (it also sets s.stale). The page's own
// delay since then is bounded separately by `linked` (<1500 ms): adding it to
// the 250 ms limit with a 350 ms poll made data look stale ~90% of the time.
const linked=snapshotAge<1500,fresh=linked&&s.stale===false&&(s.joints||[]).length===12&&s.joints.every(j=>j.valid&&Number.isFinite(j.age_ms)&&j.age_ms<=250);
const twinNotice=twinReady?(fresh?'':'Dati Motion scaduti: posa non attuale.'):!twinStarted?'Modello 3D: caricamento a robot fermo (non durante movimento, passo o joystick)':
 twinIdle(s)?'Preparazione modello 3D…':'Caricamento modello 3D… (coppia ON: solo dalla cache del browser, altrimenti a torque OFF)';
by('simple-twin-state').textContent=by('simple-twin-state').title=twinNotice;
const ac=s.active_command||{},c=latestCommand;
const commandReason=ac.reason+(reasons[ac.reason]?' ('+reasons[ac.reason]+')':'');
if(linked&&c&&ac.id===c.id&&ac.sequence===c.sequence){
 c.observed=ac;c.observed_ms=performance.now();
 lastShow(ac.outcome==='ABORTED'?'warn':'acked','Ultimo comando · '+c.label+' · '+ac.outcome+' · '+commandReason+(ac.outcome==='COMPLETED'?' · completamento confermato dal Controller':ac.outcome==='ABORTED'?' · esecuzione interrotta':' · esecuzione osservata: '+s.phase),ac.outcome==='COMPLETED'||ac.outcome==='ABORTED');
}
const aged=ms=>Number.isFinite(ms)?Math.ceil(ms+age):null;
window.mdMotionHealth={fresh,age:Math.ceil(snapshotAge)};
// READY is the executor's idle state; Motion admission is s.ready.
const controllerFault=s.fault===true||s.state==='FAULT';
const stopInfo=s.stop||{},offVerified=s.safe_off_verified===true&&s.safe_off_mask===8191&&s.torque_active===false;
const stoppedOn=stopInfo.active===true&&stopInfo.motion_verified===true&&stopInfo.in_progress===false&&s.state==='STOPPED'&&s.torque_active===true;
const offTerminal=offVerified&&s.state==='STOPPED';
const shown=controllerFault?('FAULT'+(!fresh?' · DATI SCADUTI':'')):!linked?'COLLEGAMENTO PERSO':!fresh?'FEEDBACK NON FRESCO':offTerminal?'SAFE_OFF VERIFICATO · COPPIA OFF':stopInfo.in_progress===true?'STOP IN CORSO':stoppedOn?'STOP COMPLETATO · COPPIA ON':s.state==='READY'&&!s.ready?'NON PRONTO':s.state;
by('simple-state').textContent=by('simple-state').title=shown;
// Admission and fault facts own the colour, independently of the displayed wording.
by('simple-state').dataset.tone=controllerFault?'bad':!fresh?'muted':!s.ready?'warn':stopInfo.active?'warn':offTerminal?'ok':['READY','STANDING'].includes(s.state)?'ok':['MOVING','WALKING'].includes(s.state)?'warn':'muted';
const bar=by('motion');if(bar&&bar.firstChild)bar.firstChild.nodeValue=shown;if(by('motionWhy'))by('motionWhy').textContent=s.reason;by('simple-state').dataset.state=s.state;
const demoReason=code=>code==='OFFLINE_UI_FIXTURE'||code==='OFFLINE_DISPLAY_ONLY';
const reasonCodes=[s.reason,...(s.ready?[]:[s.readiness_reason])].filter(Boolean);
const reasonMessages=[...new Set(reasonCodes.filter(code=>!demoReason(code)).map(code=>reasons[code]||code))];
if(s.fault&&!reasonMessages.length)reasonMessages.push('Guasto segnalato dal Controller.');
// Once loaded, the Twin owns the single demo badge. Never suppress real faults.
if(!twinReady&&reasonCodes.some(demoReason))reasonMessages.unshift('Modalità offline');
if(stopInfo.active===true&&stopInfo.support_verified!==true)reasonMessages.push('Supporto non verificato');
if(stopInfo.active===true){
 const resumeReason=stopInfo.stand_resume_reason;
 reasonMessages.push(stopInfo.stand_resume_allowed===true?'Recupero STAND ammesso dal Controller: da WALK completa il gait e può comportare un piccolo avanzamento. Contatto fisico non verificato.':'STAND non ammesso dal Controller: '+(resumeReason?(reasons[resumeReason]?reasons[resumeReason]+' ['+resumeReason+']':resumeReason):'motivo non disponibile'));
}
by('simple-reason').textContent=by('simple-reason').title=reasonMessages.join(' · ');
const available=fresh&&s.ready&&!controllerFault&&!!csrf&&!sending&&!['MOVING','WALKING','FAULT'].includes(s.state);
by('simple-stand').disabled=!available||(stopInfo.active?stopInfo.stand_resume_allowed!==true:!['READY','STANDING'].includes(s.state));by('simple-walk').disabled=!available||stopInfo.active===true||!['READY','STANDING'].includes(s.state);by('simple-prone').disabled=!available||stopInfo.active===true||s.state!=='STANDING';
by('simple-walk-stop').disabled=!csrf||stopInfo.active===true||!['MOVING','WALKING'].includes(s.state);
// Joystick: from a held pose, or while the Controller reports its joystick walk.
const wv=s.walk_velocity||{};joyAllowed=fresh&&s.ready&&!controllerFault&&!!csrf&&stopInfo.active!==true&&(['READY','STANDING'].includes(s.state)||wv.joystick===true);
if(!joyAllowed&&joyActive)joyRelease();by('simple-joystick').setAttribute('aria-disabled',String(!joyAllowed));by('joy-left').disabled=by('joy-right').disabled=!joyAllowed;
const law=Number.isFinite(wv.vx_m_s)?joyText([wv.vx_m_s,wv.vy_m_s||0,wv.yaw_rate_rad_s||0]):'N/D';
by('joy-state').textContent='Richiesto: '+joyText(joyVelocity())+' · Controller: '+s.state+' · '+(reasons[s.reason]||s.reason)+(wv.joystick===true?' · legge del passo '+law:'');
const dl=by('simple-values');dl.replaceChildren();let off=0;for(let m=s.safe_off_mask>>>0;m;m&=m-1)off++;
const ct=s.command_timing||{},active=s.active_command||{},cal=typeof st==='object'&&st?st:{};
const ic=s.imu_correction||{},tm=s.timing||{},th=s.thermal||{};
const rec=r=>Array.isArray(r)?'lettura '+r[0]+' · nominale '+r[1]+' · compensazione '+r[2]+' · conversione '+r[3]+' · frame '+r[4]+' µs; release precedente '+r.slice(6,12).join('/')+' µs; solve '+r[12]+', IK '+r[13]+', Newton '+r[14]+', FK '+r[15]:'nessuna';
const extra=[['Correzione IMU (richiesta → applicata, mrad)',ic.reason?ic.reason+' · roll '+ic.wanted_mrad[0]+' → '+ic.applied_mrad[0]+' · pitch '+ic.wanted_mrad[1]+' → '+ic.applied_mrad[1]:'N/D'],
 ['Temperature servo','picchi isolati scartati '+(th.spikes??'N/D')+' · finestre calde aperte 0x'+((th.hot_pending_mask??0).toString(16))+' · conferme dirette '+(th.checks??'N/D')+(Array.isArray(th.last)?th.last.map(e=>' · ID '+e[0]+' bulk '+e[1]+' °C diretta '+(e[2]<0?'non valida':e[2]+' °C')+' ('+e[3]+', '+e[4]+' ms fa)').join(''):'')],
 ['Release più pesante · partizioni',rec(tm.max_work)],['Prima release persa · partizioni',rec(tm.first_miss)],
 ['Release oltre 20 ms / periodi persi','comando '+(ct.overruns??'N/D')+' / '+(ct.missed??'N/D')+' · dall\'avvio '+(s.overrun_releases??'N/D')+' / '+(s.missed_releases??'N/D')+(Array.isArray(tm.max_work_meta)?' · release più pesante: ritardo '+tm.max_work_meta[0]+' µs, fuori Motion '+tm.max_work_meta[1]+' µs, servizi '+tm.max_work_meta[2]+' µs':'')],
 ['Provenienza giunti','RAW letto e posizione: FAST (~120 ms); coppia e goal: CONTROL (~1,4 s); RAW comandato: writer corrente. Campioni non simultanei.']];
for(const [k,v] of [...extra,['Età snapshot Motion',Math.ceil(snapshotAge)+' ms · '+(fresh?'ricevuto, feedback valido alla sorgente':'NON AGGIORNATO')],['Motion ammessa dal Controller',s.ready?'sì':'no · '+(reasons[s.readiness_reason]||s.readiness_reason)],['Esito operativo (telemetria Controller)',active.reason==='NOT_INITIALIZED'?'nessuno':active.outcome+' · '+active.reason],['Posa misurata (encoder)',poses[s.pose_region]||s.pose_region||'N/D'],['Fase',s.phase],['Quota nominale richiesta (comando)',s.requested_height_mm+' mm'],['Quota nominale stimata (encoder + IMU)',s.height_valid?Number(s.height_estimate_mm).toFixed(1)+' mm':'non disponibile'],['Quota base_link stimata (encoder + IMU)',s.height_valid?Number(s.base_link_height_mm).toFixed(1)+' mm':'non disponibile'],['SAFE_OFF',!linked?'ultimo snapshot scaduto · verifica attuale non disponibile':s.safe_off_verified?'13/13 verificato':(s.torque_active?'torque attivo':'verifica in corso')+' · '+off+'/13 verificati'],['Sessione amministratore',s.admin_session?'valida':'non valida: accedere per comandare (un Motion accettato prosegue)'],['Calibrazione',cal.calibration_persist_verdict?cal.calibration_persist_verdict+' · gen '+cal.calibration_ack_generation:'vedi scheda Calibrazione'],['IMU',s.imu_valid&&aged(s.imu_age_ms)<=200?'valida · '+aged(s.imu_age_ms)+' ms':'non disponibile'],['Compensazione IMU',!s.torque_active?'non attiva':stopInfo.active===true?'congelata durante STOP':s.imu_correction_limited?'limitata dal workspace':'attiva nei limiti'],['Cicli WALK',Number(s.walk_cycles).toFixed(2)],['Release perse · ultimo movimento',(ct.missed??'N/D')+(ct.first_miss_phase?' · prima in '+ct.first_miss_phase+', ritardo '+Math.round(ct.first_miss_lateness_us/1000)+' ms':'')],['Release perse · totale dal boot',s.missed_releases],['Release più pesante · ultimo movimento',ct.max_work_us?(ct.max_work_us/1000).toFixed(1)+' ms in '+ct.max_work_phase+' · compensazione max '+(ct.max_compensate_us/1000).toFixed(1)+' ms':'N/D']]){const dt=document.createElement('dt'),dd=document.createElement('dd');dt.textContent=k;dd.textContent=v;dl.append(dt,dd)}
// Torque comes from the control read (each joint every 1.44 s, control_age_ms),
// not from the fast feedback (age_ms): 0 OFF, 1 ON, anything else UNKNOWN.
const torque=j=>!linked||!j.valid||!(aged(j.control_age_ms)<=3000)?'UNKNOWN':j.torque===0?'OFF':j.torque===1?'ON':'UNKNOWN';
// ACTUAL angles: the telemetry block shared with the Digital Twin.
const t=s.telemetry||{},ang=t.actual_rad||t.actual_rad_by_joint||[];
const angle=i=>typeof ang[i]==='number'?(ang[i]*180/Math.PI).toFixed(1)+'°':'N/D';
const rows=[];for(const j of s.joints||[]){const tr=document.createElement('tr');if(!j.valid||!fresh||j.age_ms>250)tr.className='bad';
 for(const v of [j.joint,j.id,angle(j.slot),j.raw,j.temperature+' °C',torque(j),aged(j.control_age_ms)+' ms',aged(j.age_ms)+' ms']){const td=document.createElement('td');td.textContent=String(v);tr.append(td)}rows.push(tr)}
by('simple-joints').tBodies[0].replaceChildren(...rows);
by('simple-detail').textContent=JSON.stringify({period_us:s.period_us,missed_releases:s.missed_releases,max_gap_us:s.max_gap_us,joints:s.joints},null,2);
}
// The last command result is kept apart from the telemetry render: an HTTP
// answer or an ACK is never proof of motion, and a poll never hides it.
async function command(type,params={},priority=false,label=type,retried=false){
let record=null;
const result=(kind,text)=>{if(record&&latestCommand!==record)return;lastShow(kind,'Ultimo comando · '+label+' · '+new Date().toLocaleTimeString('it-IT')+' · '+text);
 // The banner shows only the latest command: an earlier refusal stays in the Motion diagnostics, not here.
 if(kind==='bad'||kind==='warn'){commandNotice='Comando '+label+': '+text;show(commandNotice,kind)}
 else if(commandNotice){if(message.textContent===commandNotice)show('');commandNotice=''}};
if(!csrf){result('bad','NON INVIATO: serve l’accesso amministratore'+(type==='MOTION.STOP'?'. Usare anche @STOP via USB o il KEY OFF.':'.'));needLogin();return}
if(sending&&!priority)return;
if(!priority)sending=true;result('pending','inviato, in attesa della risposta…');render();
const uptime=status?Math.floor(status.now_ms+performance.now()-received):0;
const body={version:1,id:'simple-'+session+'-'+(++sequence),sequence,issued_at_ms:uptime,ttl_ms:type==='BASE_VELOCITY'?1000:5000,owner:'dashboard',type,params};
// Longer than the Controller's own budgets (STOP lane 0.6 s, command 3 s,
// plus one read ahead in the HTTP task): a late answer is not a refusal.
record={id:body.id,sequence:body.sequence,label,type,sent_ms:performance.now(),issued_at_ms:uptime};latestCommand=record;keep(trace.commands,record,48);
const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),5000);
const why=c=>c+(reasons[c]?' ('+reasons[c]+')':'');let renew=false;
// HttpTransport refuses these strictly before a command can run: every 400,
// every 403 except ACK_UNAVAILABLE (sent after execution), and the dispatch
// refusals that never queue it. Anything else without an ACK is unknown:
// CONTROLLER_LOOP_LATE may still run, STOP_LANE_LATE stays in the STOP lane.
const refused=(status,code)=>!!code&&(status===400||(status===403&&code!=='ACK_UNAVAILABLE')||(status===503&&(code==='MAILBOX_BUSY'||code==='ADMIN_GATE_CLOSED')));
const unknown=text=>result(type==='MOTION.STOP'?'bad':'warn',text+' · ESITO SCONOSCIUTO: il Controller può averlo eseguito, verificare lo stato Motion'+(type==='MOTION.STOP'?'; per torque OFF globale usare @SERVO SAFE_OFF 51 via USB o il KEY OFF.':'.'));
try{const r=await fetch('/api/motion/command',{method:'POST',cache:'no-store',headers:{'Content-Type':'application/json','X-Csrf-Token':csrf},body:JSON.stringify(body),signal:abort.signal});
 record.headers_ms=performance.now();record.server_timing=r.headers?.get('Server-Timing')||null;record.http=r.status;
 const text=await r.text();record.body_ms=performance.now();let a=null;try{a=JSON.parse(text)}catch(e){}
 record.ack=a;const http='HTTP '+r.status;
 // Refused at authentication: nothing reached the Controller; renew once.
 if(r.status===403&&a&&a.error==='AUTH_FAILED'&&!retried)renew=true;
 else if(!r.ok||!a||a.ack!==true){const code=a&&a.error,detail=code?why(code)+(a.reason?' · ragione '+a.reason:''):a?'RISPOSTA_SENZA_ACK':'RISPOSTA_NON_DECODIFICABILE';
  if(refused(r.status,code))result('bad',http+' · RIFIUTATO PRIMA DELL’ESECUZIONE · errore '+detail);else unknown(http+' · errore '+detail)}
 else if(a.id!==body.id||a.sequence!==body.sequence)unknown(http+' · ACK_ID_MISMATCH');
 else if(a.outcome==='REJECTED')result('warn',http+' · REJECTED dal Controller, non eseguito · '+why(a.reason));
 else result(a.outcome==='ABORTED'?'warn':'acked',http+' · '+a.outcome+' · '+why(a.reason)+(type==='MOTION.STOP'?(a.reason==='SAFE_OFF_REQUESTED'?' · SAFE_OFF non ancora verificato':' · arresto da verificare nello stato Motion'):' · movimento da confermare nello stato Motion'));}
catch(e){record.error=e.name;record.failed_ms=performance.now();unknown(e.name==='AbortError'?'nessuna risposta entro 5 s':'errore di rete: '+e.message)}
finally{clearTimeout(timer);if(!priority)sending=false;render()}
if(!renew||latestCommand!==record)return;
result('pending','sessione non valida: rinnovo e nuovo invio…');
const renewed=await resume();if(latestCommand!==record)return;
if(renewed)return command(type,params,priority,label,true);
result('bad','HTTP 403 · RIFIUTATO PRIMA DELL’ESECUZIONE · AUTH_FAILED: sessione non rinnovabile, accedere di nuovo'+(type==='MOTION.STOP'?'. Usare @STOP via USB o il KEY OFF.':'.'));needLogin();
}
// The status poll is telemetry and admin-session upkeep only: no Motion
// command depends on it, and commands/STOP use their own requests. Listener:
// 7 sockets (HttpTransport), enough for both read lanes, a command and STOP.
async function poll(){
 // Own read lane (dev.17), outside the portal's diagnostic lane: at most two
 // reads in flight for the WHOLE page (window.mdReads, both lanes), a second
 // Motion read only when the one in flight is older than READ_OVERLAP_MS (a TCP
 // segment lost on the Controller costs a retransmission timeout, not an abort
 // and a new connection). With one command, WALK STOP and STOP that is 5 of the
 // 6 connections a browser opens to one host: STOP never waits for a free one.
 // The newest issued read wins; an older late answer is counted and dropped.
 const t0=performance.now();
 if((window.mdReads||0)>=2||(readStarts.length&&t0-readStarts[0]<READ_OVERLAP_MS)){trace.poll_skipped++;return}
 const seq=++readSequence;readStarts.push(t0);window.mdReads=(window.mdReads||0)+1;
 const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),READ_ABORT_MS);
 try{const r=await fetch('/api/motion/status',{cache:'no-store',headers:csrf?{'X-Csrf-Token':csrf}:{},signal:abort.signal});if(!r.ok)throw Error(r.status);const body=await r.json();
  if(seq<appliedSequence)trace.late_dropped++;
  else{appliedSequence=seq;status=body;received=performance.now();transitAge=Math.max(0,received-t0);keep(trace.samples,{received_ms:received,roundtrip_ms:transitAge,status},240);prepareTwin();render();
  if(twinStarted){
   // The viewer reads its optional freshness channels (IMU age, RAW count) under
   // channels.*; this status carries the same Controller facts as imu_valid,
   // imu_age_ms and joints[].valid. Bound here, nothing new is measured.
   by('simple-twin-frame').contentWindow?.postMessage({...status,channels:{imu:{valid:status.imu_valid,age_ms:status.imu_age_ms},raw:{count:(status.joints||[]).filter(j=>j.valid).length}}},location.origin)}}}
 catch(e){keep(trace.poll_errors,{at_ms:performance.now(),since_queued_ms:performance.now()-t0,error:e.name==='AbortError'?'TIMEOUT_'+READ_ABORT_MS+'MS':String(e.message||e.name)},240);render()}
 finally{clearTimeout(timer);readStarts.splice(readStarts.indexOf(t0),1);window.mdReads--}
// Admin session upkeep: an authenticated poll renews it on the Controller;
// the cookie is refreshed through /wifi/session; a lapsed one is resumed.
if(!status||!csrf)return;const now=Date.now();
if(status.admin_session===true){sessionAt=now;if(now-cookieAt>120000){cookieAt=now;resume()}}
else if(status.admin_session===false&&now-lapseAt>5000){lapseAt=now;cookieAt=now;resume()}}
// Joystick: forward/left on the pad, yaw on the held buttons, each proportional
// with a dead zone and scaled into the Controller's ellipsoid. The Controller
// owns rate limits, gait, IK and timing; the page only refreshes the intent well
// inside the Controller's 1 s deadline and sends neutral (WALK STOP) on release.
// JOY_LIMIT = kJoystickStrideM, kJoystickLateralM, kJoystickYawRad / kWalkPeriodS.
const JOY_LIMIT=[.045/6.5,.020/6.5,.20/6.5],JOY_DEAD=.12,JOY_MS=240,pad=by('joy-pad');
let joy=[0,0],joyYaw=0,joyPad=false,joyActive=false,joyAllowed=false,joyTimer=0,joyPointer=null,joySent=false,joyOrigin=null;
function joyVelocity(){const r=Math.hypot(joy[0],joy[1]),k=r<JOY_DEAD?0:Math.min(1,(r-JOY_DEAD)/(1-JOY_DEAD))/r;
 let n=[joy[0]*k,joy[1]*k,joyYaw];const m=Math.hypot(n[0],n[1],n[2]);if(m>1)n=n.map(v=>v/m);return n.map((v,i)=>v*JOY_LIMIT[i])}
const joyText=v=>{const t=[];if(v[0])t.push((v[0]>0?'avanti ':'indietro ')+(Math.abs(v[0])*1000).toFixed(1)+' mm/s');
 if(v[1])t.push((v[1]>0?'sinistra ':'destra ')+(Math.abs(v[1])*1000).toFixed(1)+' mm/s');
 if(v[2])t.push((v[2]>0?'antiorario ':'orario ')+(Math.abs(v[2])*180/Math.PI).toFixed(2)+'°/s');return t.length?t.join(', '):'neutro'};
function joyDraw(){by('joy-knob').setAttribute('cx',String(90-58*joy[1]));by('joy-knob').setAttribute('cy',String(90-58*joy[0]));by('simple-joystick').dataset.active=String(joyActive)}
// The pad centre is taken once, at pointerdown (dev.17): a reflow while the pad
// is held can move the pad on screen, never the requested velocity.
function joyAnchor(){const r=pad.getBoundingClientRect(),w=r.width||180,h=r.height||180;joyOrigin={x:r.left+w/2,y:r.top+h/2,w,h}}
function joyMove(e){const o=joyOrigin;if(!o)return;  // knob travel: 58 of the 180 viewBox units
 let f=(o.y-e.clientY)*180/(58*o.h),l=(o.x-e.clientX)*180/(58*o.w);const m=Math.hypot(f,l);if(m>1){f/=m;l/=m}joy=[f,l];joyDraw()}
function joyTick(){if(!joyActive)return;const v=joyVelocity();
 // Back to neutral: WALK STOP once, like a release. The STOP lane is untouched.
 if(!v[0]&&!v[1]&&!v[2]){if(joySent){joySent=false;command('BASE_VELOCITY',{vx:0,vy:0,yaw_rate:0,gait:'JOYSTICK'},true,'JOYSTICK neutro')}return}
 if(sending)return;joySent=true;command('BASE_VELOCITY',{vx:v[0],vy:v[1],yaw_rate:v[2],gait:'JOYSTICK'},false,'JOYSTICK '+joyText(v))}
function joyStart(){if(joyActive)return;joyActive=true;joyTick();joyTimer=setInterval(joyTick,JOY_MS)}
function joyRelease(){if(!joyActive)return;joyActive=false;joyPad=false;joyPointer=null;joyOrigin=null;joyYaw=0;clearInterval(joyTimer);joy=[0,0];joyDraw();
 if(joySent){joySent=false;command('BASE_VELOCITY',{vx:0,vy:0,yaw_rate:0,gait:'JOYSTICK'},true,'JOYSTICK rilascio')}twinResize()}
// Release ends only when neither the pad nor a rotation button is held.
const joyEnd=()=>{if(!joyPad&&!joyYaw)joyRelease();else joyTick()};
pad.addEventListener('pointerdown',e=>{if(!joyAllowed||joyPad)return;e.preventDefault();joyPointer=e.pointerId;try{pad.setPointerCapture?.(e.pointerId)}catch(x){}
 joyPad=true;joyAnchor();joyMove(e);if(joyActive)joyTick();else joyStart()});
pad.addEventListener('pointermove',e=>{if(joyPad&&e.pointerId===joyPointer)joyMove(e)});
// Pad release: on the pad (captured pointer) and, should the capture be missing,
// anywhere in the window, so a finger lifted off the pad always ends the intent.
const padUp=e=>{if(joyPad&&e.pointerId===joyPointer){joyPad=false;joyPointer=null;joyOrigin=null;joy=[0,0];joyDraw();joyEnd()}};
for(const t of ['pointerup','pointercancel','lostpointercapture'])pad.addEventListener(t,padUp);
for(const t of ['pointerup','pointercancel'])window.addEventListener(t,padUp,true);
for(const el of [pad,by('joy-left'),by('joy-right')])el.addEventListener('contextmenu',e=>e.preventDefault());  // long press is a hold
for(const [id,sign] of [['joy-left',1],['joy-right',-1]]){const b=by(id);
 b.addEventListener('pointerdown',e=>{if(!joyAllowed)return;e.preventDefault();try{b.setPointerCapture?.(e.pointerId)}catch(x){}joyYaw=sign;if(joyActive)joyTick();else joyStart()});
 for(const t of ['pointerup','pointercancel','pointerleave','lostpointercapture'])b.addEventListener(t,()=>{if(joyYaw===sign){joyYaw=0;joyEnd()}})}
window.addEventListener('blur',joyRelease);document.addEventListener('visibilitychange',()=>{if(document.hidden)joyRelease()});
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
