# dev.10: RCA del collaudo del 03/11/2026

## Provenienza e perimetro

Sorgente installato: `ea17b7f1b083e08887e89530dc9bef912b6a7158`,
MATDOG `1.0.1-dev.10`, ROBOT_POWERED, OTA ingest 0. APP qualificata:
1.655.616 byte, SHA256
`3c11d87b58e8252d5cb4f794605a793a3fb60e9c1f1f5aa033fb7d6b990046b8`.
Il worktree `dev10-simple-audit` e questa APP sono preservati. Le patch qui
descritte sono su `codex/dev10-hw-rca-20261103`, derivato da quel commit.
La successiva identità offline è `1.0.1-dev.11`; l'APP di revisione viene
compilata esternamente al worktree, con manifest come fonte della sua identità.
Nessun flash/OTA/reset, comando al robot, modifica NVS/EEPROM, push o merge.
La patch separata a `TiltedContactIk.cpp` non è inclusa.

**VERIFIED HARDWARE — osservazione operatore**, registrata in
[#62](https://github.com/MattRobotics/robot-dog/issues/62#issuecomment-6612213273)
e [#52](https://github.com/MattRobotics/robot-dog/issues/52#issuecomment-6612214174):
a terra dalla PRONE corretta funzionano STAND a diverse quote, WALK STANDARD,
PRONE, STOP con coppia mantenuta da STAND e WALK; STAND esplicito dopo STOP da
STAND funziona. L'assenza di collasso osservata non misura frenata né appoggi.
RESTORE gen1, Census 13/13 incluso ID51 e flash APP-only sono confermati anche
dal log locale `build-output/dev10-simple-audit/flash-4T0hcqj6/boot-usb.log`.
Backup e rollback dev.7 restano preservati.

## A. Recupero dopo STOP da WALK

**Causa dimostrata del rifiuto nella dev.10 installata.**
In `ea17b7f1`, `SimpleMotion::standResumeRefusal()` rifiuta incondizionatamente l'origine
WALK: `STOP_RECOVERY_SWING_UNVERIFIED` oppure `STOP_RECOVERY_WALK_UNVERIFIED`.
`MotionService::command(STAND)` propaga quel rifiuto; la dashboard usa
`stop.stand_resume_allowed`, quindi il pulsante disabilitato segue il Controller.
Il difetto UI era l'assenza del motivo specifico nella presentazione del terminale.

Ricostruzione del contratto esatto dev.10:

- Il frame WALK accettato dal writer conserva fase e presenza nominale di swing
  in `rememberReference()`. Non sono misure di contatto.
- `beginStop()` revoca il vecchio intento e costruisce il riferimento dai RAW
  effettivamente accettati, inclusi arrotondamento e IMU. La velocità iniziale
  è una secante dei due ultimi riferimenti, non velocità motore misurata.
- `StopPlan` termina per giunto a `q + T*v/2 + T*T*a/12`, nei limiti esistenti.
  Non completa lo swing né conserva una legge cartesiana dei piedi stance.
- `generateStop()` verifica feedback successivo al terminale, coppia/goal/Drive,
  velocità, errore, carico e dwell. Solo allora: STOPPED,
  `STOPPED_SUPPORT_UNVERIFIED`, coppia ON, `support_verified=false`.
- `standResumeRefusal()` richiede STOP completato, energia/authority/IMU e
  feedback freschi, coppia ON verificata, nessun FAULT/OFF; ammette solo HOLD
  o PRONE_WAIT con i rispettivi percorsi esistenti. Da HOLD, STAND riparte
  esplicitamente senza ACQUIRE/REFERENCE/ENABLE.

La fase swing/stance effettiva e i dodici RAW finali dello STOP segnalato non
sono nei log disponibili: non sono ricostruibili numericamente dall'esito UI.
Il bit nominale `stopHadSwing_`, anche falso, non prova quattro appoggi terminali.
Il precedente percorso di acquisizione WALK→150 mm prova il box anticollisione,
non il supporto; non è stato usato per aggirare il rifiuto.

Il primo commit `f5c486e` espone il rifiuto come `stop.stand_resume_reason`.
La ripresa notturna aggiunge un **recupero limitato, VERIFIED OFFLINE**:

- Tutte le guardie STOP esistenti restano necessarie. Lo swing nominale noto
  resta rifiutato con `STOP_RECOVERY_SWING_UNVERIFIED`.
- Posa misurata, RAW mantenuti decodificati e ultimo nominale devono essere
  nel box HEIGHT esistente; posa e riferimento fermato devono anche essere
  entro l'allowance STAND esistente di 12 tick dal nominale STAND150.
- STAND esplicito conserva i RAW fino al prossimo release. Poi ricontrolla
  entrambi gli estremi, riusa ViaPlan/HEIGHT verso STAND150 e infine la quota
  richiesta. Primo riferimento RAW identico; vecchia correzione IMU non
  applicata due volte, nuova compensazione riavviata gradualmente.
- Nessun ACQUIRE/REFERENCE/ENABLE, nessuna ripresa del gait, nessun OFF.
  STOP prima/durante il primo TX e SAFE_OFF conservano la precedenza.
- Il rifiuto per le altre pose senza swing è ora
  `STOP_RECOVERY_REQUIRES_STAND_POSE`. Il Controller non misura gli appoggi:
  `support_verified` rimane `false`, anche per il subset ammesso.

I test runtime coprono avvio e recentraggio finale del normale WALK STOP,
tre passi, quote 50/100/150, IMU positiva/negativa e variabile, RAW iniziale,
zero enable/OFF, rifiuti nelle otto finestre regolari stance/swing e race di
STOP/OFF/posa cambiata prima del primo release. Non qualificano il supporto.

La sonda separata `build-output/dev10-hw-rca-20261103/walk-recovery-probe.txt`
descrive endpoint analitici del polinomio STOP, non l'intera API e non hardware:
griglia 20 ms, IMU level, tre passi. Nei cicli 1–3 nessuno dei 196 campioni nominali
quattro-stance per passo entra in HEIGHT. Esistono endpoint compatibili nei
primi frame d'avvio. I PASS runtime sono quelli del test distinto sopra.

**Aperto:** recupero dal WALK regolare fuori HEIGHT e dallo swing. Il subset
non risolve genericamente il caso hardware segnalato. Occorre qualificare un raccordo dal
riferimento arrestato che ricomponga l'eventuale swing mantenendo i tre piedi
stance e solo dopo ricentri il corpo. Le primitive di traiettoria non sono da
sole una prova di supporto. Servono posa/IMU/RAW/fase dello STOP reale e prova
del percorso nelle condizioni fisiche dichiarate; nessuna ripresa automatica,
riabilitazione implicita o cancellazione FAULT è stata introdotta.

### Perché non estendere semplicemente il recupero a tutto kWalk

Verifica mirata della ripresa finale del 04/11: il limite HEIGHT è un limite
del raccordo attuale, non una prova che ogni posa WALK collida. Il precedente
percorso di acquisizione `InitialRoute::WALK` usa ViaPlan verso STAND150 in
kWalk, ma non conserva gli anchor del gait e non qualifica gli appoggi.

Controesempio software con le unità di produzione: passo 60 mm/ciclo,
ciclo 1,04, quattro stance nominali, STOP verificato. Il raccordo diretto
RAW fermati→STAND150 dura 1,040597 s e resta in kWalk. A metà raccordo il
margine dal box è 0,051442 rad, ma la distanza fra riferimenti FK LF–RH
si riduce di 18,375 mm, e di 36,208 mm al terminale. Anche i punti geometrici
più bassi confermano tale variazione. Quindi il semplice piano nei giunti
non mantiene l'impronta iniziale; il solo successo anticollisione non è
una prova di continuità degli appoggi.

Non si deduce una caduta: attrito, deformazioni, contatti portanti e COM reale
non sono misurati. Nemmeno un ricentraggio cartesiano con quattro anchor fissi
può raggiungere quel canonico150, perché le due impronte hanno distanze diverse.
Servono uno STAND/HEIGHT coerente con l'impronta sfalsata oppure un
riposizionamento esplicito dei piedi con supporto rimanente qualificato.
`gaitHold(cycle)` ricrea anchor canonici, non lo STOP arbitrario: non è un
raccordo pronto da richiamare. `worldContactInverseKinematics` resta una
primitiva riutilizzabile, non una misura di contatto.

Il minimo fatto fisico mancante è quali piedi siano effettivamente portanti
e su quale piano al terminale STOP (oppure sostegno esterno del corpo).
La telemetria attuale non misura questo fatto; il bit stance e gli encoder
non lo sostituiscono. Il successivo percorso deve conservare gli appoggi
qualificati prima di riposizionare un piede. Nessuna nuova macchina a stati,
conferma amministrativa, deduzione di contatto o ripresa del gait è introdotta.
Artefatti della verifica: `build-output/dev10-hw-rca-20261103/`:
`contact-recovery-probe.txt` e `contact-recovery-probe-core.txt`. Sono dati
offline del modello, non una traccia del robot né una prova di impossibilità fisica.

### STOP ricevuto quando il Controller è già in FAULT

La ricognizione notturna ha corretto un errore nel riepilogo: nessuno dei tre
primi commit modificava questo esito. Il vecchio `MotionService::update()`
concludeva il nuovo STOP come ABORTED usando subito il precedente motivo
PRONE; inoltre un precedente STOP terminale poteva impedirne la nuova ricevuta.

La patch distingue il contesto della richiesta: STOP ricevuto dopo FAULT
risponde `STOP_REQUESTED_FAULT_LATCHED`, attende l'OFF verificato e conclude
`SAFE_OFF_VERIFIED_FAULT_LATCHED`. Stato FAULT e motivo originale restano
separati e latched. Un fault nuovo durante un arresto inizialmente sano
continua invece a farlo ABORTED col motivo reale. I duplicati del terminale
torqueON non riavviano identità, dwell o deadline. Test con OFF ancora non
verificato, OFF già completato, STOP successivo e fault durante STOP: PASS.

## B. PRONE sulla fixture e feedback stale

### Passaggio prematuro: difetto software riprodotto e corretto

Prima divergenza nel replay mirato: `arrived()` accettava la posa entro i
12 tick esistenti, poi `nextStand()` chiamava `beginProne()` che richiedeva
la posa già nel box successivo. `settledUnderLoad()` richiedeva invece già
quel box. Un arrivo ordinario poteva quindi produrre FAULT anticipato.

Caso **VERIFIED OFFLINE**, tracking scriptato, non misura del robot:

| Grandezza | Valore |
|---|---|
| Fase corrente → successiva | PRONE_HEIGHT → PRONE_LIFT |
| Giunto | slot 7, RH_UPPER |
| Posa misurata scriptata | 1,972699293221 rad |
| Riferimento terminale | 1,976507520163 rad |
| Errore | −2,482578 tick, entro i 12 tick esistenti |
| Box corrente kHeight | [0,873333849305; 2,049151704260] rad |
| Box successivo kLift | [1,973197145824; 2,067102697901] rad |
| Prima esclusione | 0,000497852603 rad sotto kLift |
| Baseline | PRONE_PATH_NOT_CLEAR a 11.900.000 µs |
| Deadline ancora valida | 26.825.348 µs |

Patch: l'arrivo PRONE attende l'appartenenza al box successivo mantenendo
riferimento, controlli del box corrente e deadline. La stessa mappa dei box
serve arrivo ordinario e loaded-settle. Il controllo finale in `beginProne()`
resta attivo. Il test dimostra sia prosecuzione quando il tracking recupera,
sia FAULT `NATIVE_TARGET_NOT_REACHED` alla deadline originale se non recupera.
Nessun box, target, limite, tolleranza, Drive o compensazione IMU modificato.

**Limite della diagnosi hardware:** manca il campione del PRONE interrotto.
Non è dimostrato che il caso sopra sia la causa della prova fixture, né che
la fixture sia ammessa in tutte le pose. Non si attribuisce l'evento al carico.
Per identificare la prima divergenza reale servono l'export immediato della
pagina e l'esistente `@MOTION DIAG`: fase del primo exit, RAW comandati e
misurati, età/speed/load, Drive, tempi, assetto IMU vicino all'evento. Le
diagnostiche leggono cache; nessuna nuova acquisizione o movimento è stato
eseguito in questa RCA. Non è disponibile un confronto tracking fixture/terra.

### Feedback congelato dopo FAULT: causa indipendente dimostrata

`Controller::stopPreempt()` consultava `SimpleMotion::busPreemptRequested()`;
il latch FAULT rendeva il predicato sempre vero anche dopo SAFE_OFF 13/13.
Le letture fast/control bounded venivano quindi cancellate e `fb_` non veniva
più pubblicato. Il Controller continuava le release e rispondeva alla rete.
Non era una prova di perdita fisica della comunicazione servo.

Patch: lo scope READ esistente permette queste sole letture anche con FAULT
latched. STOP nuovo, SAFE_OFF e OFF incompleto continuano a preemptarle;
fuori dallo scope il fence FAULT resta attivo. Nessun frame/enable, reset
del fault o movimento di recupero. Test con il predicato di preemption reale:
prima della patch feedback stale; dopo, letture fresche per tutta la popolazione
fast/control, coppia OFF, maschera 8191, FAULT e motivo invariati. STOP/OFF
sopraggiunti nel READ mantengono la priorità.

Riscontro reale disponibile: `Downloads/matdog-motion-trace (4).json`, SHA256
`6d6435d071dc5744fb96bae0df8a84173247eba5282cc7f1c54adc86302d3bf5`.
Contiene un precedente STAND150 rifiutato durante ACQUIRE con
`INITIAL_POSE_OUTSIDE_SAFE_ENTRY` (69.434→69.663 ms), non la prova PRONE.
I 240 campioni successivi mostrano RAW fermi e age 12,8→97,9 s, IMU fresca,
release crescenti. Il fault precede lo stale: non invertire causa ed effetto.

## C. Scatti LF_UPPER

**Causa non determinata; nessuna patch di tuning giustificata.** Non è stata
trovata una cattura della fase WALK descritta. Il piede sollevato esclude
il solo trascinamento come spiegazione dell'osservazione, non distingue
comando discontinuo da risposta servo/meccanica.

Verifica del sorgente installato:

- Stessa legge gait/IK per LF/RF/RH/LH, offset 0/0,5/0,75/0,25;
  nessun ramo speciale LF_UPPER. Le geometrie anteriori/posteriori differiscono.
- Mapping preservato: LF_UPPER ID12, RF_UPPER ID22, RH_UPPER ID32,
  LH_UPPER ID42; direzioni UPPER +/−/−/+, trasformazioni gen1 senza wrap.
- Un frame SyncWrite per i 12 giunti, Drive comune 1000/50, periodo nominale
  20 ms. La compensazione alterna solve e riuso offset su tutti i giunti;
  non è provato che il ritmo 40 ms del solve causi lo scatto LF.
- Il parametro operatore è 60 mm/ciclo (ciclo 6,5 s), non 60 mm/s.
  I test di continuità nominale coprono quattro zampe e tutti i tre passi;
  non provano la regolarità del TX fisico con IMU o la risposta dell'attuatore.

Misura discriminante minima: una finestra dello scatto con timestamp dei RAW
finali realmente trasmessi (ID12 e confronto ID22), encoder/speed/età, fase e
IMU, associata al video. La pagina a circa 350 ms non risolve discontinuità
20/40 ms; la cattura passiva UART evita di modificare il firmware. Se il TX
è regolare, prima verifica fisica a robot sostenuto e coppia OFF: scorrimento,
gioco, horn/linkage/cavi LF rispetto RF, poi profilo nativo e alimentazione
in sola lettura. Nessun cambio PID/Drive/IK/gait/EEPROM/calibrazione proposto
senza questa distinzione. Nessuna prova hardware autorizzata è stata eseguita.

Ultimo controllo degli export disponibili: cinque file `matdog-motion-trace*`
in Downloads. Soltanto `(3)` contiene 80 campioni WALK, del 31/10, non associati
allo scatto dev.10; `(4)` del 03/11 contiene solo il FAULT precedente descritto
sopra. Nessuno contiene i frame TX del momento segnalato. Stato causa: UNKNOWN.

## D. Dashboard e Twin conclusi

Commit UI distinto `209f3fc`, completato prima dell'interruzione finale e
verificato come realmente presente, senza duplicarlo:

- Rimosso Comandi ↓. Joystick XY e yaw statici IN SVILUPPO nella dashboard
  operativa, dopo i comandi attivi; nessun handler o endpoint Motion.
- Reset camera al riferimento assoluto +X, senza alterare quaternion BNO085,
  modello o Motion.
- Diagnostica fuori dal canvas, sotto il viewer. Altezza iframe adattata sul
  canale parent già esistente, origine/source verificate, nessun nuovo polling.
  Superficie WebGL e buffer invariati aprendo/chiudendo la diagnostica, inclusi
  bordi iframe; nessuno scroll interno o clipping desktop/mobile.
- Motivo esatto dell'ammissione/rifiuto STAND, STOP con coppia ON, supporto
  non verificato, FAULT con causa originale, stale e OFF13 restano distinti.
- Il parser Twin preferisce state/phase attuali, con fallback legacy: corretto
  UNKNOWN/UNKNOWN quando il Controller aveva già pubblicato lo stato reale.

Pipeline asset esistente eseguita: header finale 236.920 byte gzip. URDF e
mesh invariati. Artefatti browser e test in
`build-output/dev10-hw-rca-20261103/ui/`; le fixture sono esplicitamente offline.

## Verifiche e decisione

- `run_stop_support_tests.sh` ASan/UBSan: PASS, inclusi rifiuti WALK stance/swing,
  HOLD50/100/150, recupero esplicito HOLD/PRONE, duplicati, deadline,
  preemption, fault e SAFE_OFF13; nuovo caso feedback dopo FAULT prima RED,
  poi PASS.
- `run_simple_motion_tests.sh` ASan/UBSan: PASS; nuovo handover PRONE prima
  RED, poi attesa/catch-up/timeout PASS; regressioni STAND, PRONE, gait,
  IMU, loaded hold e rifiuti prima dell'abilitazione PASS.
- `test_motion_http_transport.py`: 97 PASS dopo l'estensione status.
- `test_stop_adapter.py`: PASS writer/preemption/cancellation.
- Ripresa: `motion-resume-final.log` e `simple-resume-final.log` PASS con i
  nuovi casi di recupero compatibile, RAW C0/IMU e STOP ricevuto in FAULT.
- UI finale: fixture 27, DOM 37 comandi, contratto 161, parent feed 47,
  budget 28 PASS; viewer 27, asset 42, typecheck e build web PASS.
- Browser desktop 1440×1000 e mobile 390×844: canvas/buffer identici con
  diagnostica aperta, zero POST da joystick/reset/diagnostica, FAULT originale
  ancora rosso dopo STOP completato. Nessuna prova hardware implicita.

Le funzioni hardware elencate all'inizio sono preservate nel codice e coperte
dalle regressioni pertinenti. Le patch non sono state qualificate sul robot.
**NO-GO a un nuovo flash APP-only in questa consegna:** recupero WALK non
qualificato, causa LF aperta e associazione tra handover riprodotto e PRONE
hardware ancora da verificare. L'artefatto offline di revisione non viene
presentato come soluzione completa del recupero WALK. Il suo manifest e i
risultati della compilazione/verifica restano esterni al worktree in
`build-output/dev11-hw-rca-20261104/artifacts/ROBOT_POWERED-ota0/`.

Commit Motion conservati: `f5c486e`, `2c2b1b7`, `09eb734`; continuazione
`ac91f6e`. UI separata `209f3fc`. Il successivo commit documenta queste prove
e assegna l'identità offline dev.11; il manifest identifica il relativo SHA
completo. Nessun cambiamento a partizioni, schema NVS, limiti, geometria,
mapping, solver separato o procedura APP-only.
