# MATDOG — remediation Motion unificata, verifica offline 19 ottobre 2026

Mandato: `MATDOG_MOTION_ARCH_REMEDIATION_HANDOFF_20261019.tar.gz`. Letti integralmente i quattro documenti principali e i dodici sorgenti prima dell'implementazione; checksum del pacchetto verificati, dodici sorgenti identici al worktree iniziale. Base `b5be64c4e7657b3b8f02deb312c6a2f0dc385812`, worktree autorizzato `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-tolerance-v1-20261018`, branch `fix/tolerance-operational-v1-20261018`, inizialmente CLEAN. Nessun altro worktree modificato, nessun agente parallelo, nessun accesso hardware, flash, reset o comando Motion.

## Diff architetturale e responsabilità

- `src/execution/MotionCoordinator.cpp:219`: STAND temporizzato riusa **lo stesso** `MotionExecutor::beginStream/stream/monitor/endStream` e `PeriodicScheduler` di WALK. Un frame coordinato per release; la seconda chiamata del Controller dopo assessment completa la medesima release. Nessun nuovo task, motore, framework o macchina a stati parallela. Il percorso finito resta per i comandi finiti preesistenti.
- `src/motion/AdaptiveStand.cpp:142`: ENTRY conserva il generatore quintico a due stadi; le cinque fasi conservano `phaseTimedSample` sulla polilinea qualificata. La progressione usa il delta del clock monotono. Su release tardiva il delta del parametro è limitato da `maxJointStepRad/maxVelocityRadS` (21,0526 ms con i valori attuali): il tempo eccedente rallenta il medesimo percorso, senza salti o burst. `checkSegment` e l'Executor continuano a verificare limiti, passo, velocità e accelerazione.
- `AdaptiveStand.cpp:227`: il microframe consuma soltanto la sequenza **TX**; l'endpoint strutturale consuma la sequenza completata dopo **HOLD misurato**. Conservati ENTRY_DISENGAGE, ENTRY_ALIGN, PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER, RISE e VERIFY_STAND. Nel replay sono sette HOLD, non uno per microframe. Il planner non riapre il vecchio gate temporale finito dopo il settle: la verifica post-TX/arrivo/quiete resta atomica nel solo HOLD. Le assunzioni di supporto fixture restano dichiarazioni del modello, senza inferenza di contatto o portanza.
- `src/execution/MotionExecutor.cpp:191`: PRE_TX esegue il monitor sui soli riferimenti già inviati; il candidato è validato prima di `frameTx`. `TX_COMPLETE_UNCONFIRMED` resta trasmissione UART, non arrivo. I riferimenti dello storico entrano al timestamp di completamento TX. Il seed della posa già tenuta/primata è esplicitamente distinto da un nuovo TX; se uno storico necessario è stato perso, STAND rifiuta il confronto anziché usare un riferimento futuro.
- `MotionExecutor.cpp:242`: tracking per giunto contro il riferimento attivo al timestamp reale del suo encoder, con **lag aggiuntivo STAND zero**, senza importare i 150 ms di WALK. Tre campioni nuovi per divergenza persistente; rileggere la medesima snapshot non incrementa il contatore. Il controllo geometrico è separato e immediato: l'intero vettore misurato deve stare nel box ±20 mrad di **un unico endpoint storico comune**, attivo all'istante del campione più vecchio. Un prodotto di dodici riferimenti storici differenti non diventa un certificato geometrico. Restano anche i limiti raw/URDF e il box dei limiti live dei target.
- `MotionExecutor.cpp:279`: HOLD richiede tutti i dodici campioni validi strettamente successivi all'ultimo TX, errore entro la tolleranza d'arrivo approvata, velocità misurata finita e zero per STAND, timeout originale 4 s. Un campione in recupero non prova né osservazione collettiva né quiete. Un nuovo campione fuori dalla banda d'arrivo revoca un precedente settle prima di `endStream`.
- `src/core/Controller.cpp:1315`: cambiano soltanto i due selettori della cache di controllo lenta. I 100 ms/1500 ms riservati a WALK restano per WALK; STAND conserva i precedenti 20 ms/età configurata del blocco di controllo, anche quando usa streaming. Producer, batch fast di tre letture, STOP fra le letture, burst torque-OFF prima della diagnostica, verifica SAFE_OFF, ownership e PRIME/ENERGIZE/ARM sono invariati. Verificata uguaglianza byte per byte del Controller con la base dopo aver invertito queste due sole sostituzioni.

Eliminate dal percorso STAND le duplicazioni `submit → WRITING → TRACKING → TIMED_TRACKED → nuovo submit`, il confronto pre-TX contro il target futuro, il controllo intermedio sul target corrente con un campione vecchio e la pseudo-completion senza osservazione post-TX. `submit` non accetta più un microframe temporizzato non finale: la vecchia semantica non può essere riattivata incidentalmente. Rimossi il relativo ramo di cadenza finita e l'ACK `TIMED_TRACKED`.

Nessuna modifica a NVS, EEPROM, offset, PID, ID, geometria, calibrazione, mesh, certificatori, profili velocità/accelerazione/tolleranze, dashboard, BMS o partizioni. Nessun limite aumentato. WALK mantiene il suo stream, region box, lag e rate di profilo.

## Copertura geometrica e limiti della dimostrazione

`20 + 4 + 0,768990394 + 15 = 39,768990394 mrad < 40 mrad`; residuo 0,231009606 mrad. Il box del vettore misurato è ancorato a un unico punto della traiettoria certificata; non si attribuisce alla finestra temporale alcun ampliamento del tubo. Il passo completo resta riservato anche per un frame che attraversa un vertice. I limiti raw e i bound folded caricati restano indipendenti e vincolanti. Il monitor arresta subito un'uscita dal box geometrico anche prima dei tre campioni di mancato tracking.

`test_tolerance_geometry_contract.py` passa sui certificati esistenti ENTRY bounded, 251 segmenti route, WALK e quota. Verificati SHA dei rispettivi URDF/mesh/profili/contratto; nessuna nuova certificazione o traiettoria sostitutiva. Ricevuta `geometry-contract.json`.

Questo dimostra contenimento del vettore di **campioni** ammesso dal software e validità geometrica dei target nel modello. Non ricostruisce una posa fisica simultanea fra polling diversi, né garantisce la posizione attuale da un encoder passato. Restano da qualificare sul robot l'incertezza generale 15 mrad, quella locale folded 8 mrad, gioco, flessione, carico, supporto, cablaggi/cover e ritardo reale. Nessuna dichiarazione di accuratezza realtime fisica.

## Test mirati e risultati

Artefatti esterni: `/home/matteo-manicardi/MATDOG/verification-artifacts/MOTION_UNIFIED_20261019/`.

| Prova | Risultato offline |
|---|---|
| `scripts/tests/run_motion_unified_tests.py` (C++ produzione, ASan/UBSan, warning come errori) | PASS, `UNIFIED_MOTION failures=0` |
| PRE_TX con actual a 19 mrad dal riferimento inviato e 22 mrad dal candidato futuro | TX ammesso; nessun falso raggiungimento richiesto |
| Finale con campioni pre-TX / undicesimo fresco e dodicesimo vecchio / recovering / velocità non nulla | Arrivo rifiutato; accettato soltanto con tutti e dodici post-TX e quieti |
| Divergenza persistente interna al box geometrico | STOP al terzo campione nuovo; snapshot ripetute non contano |
| STOP prima TX e fra ogni coppia di letture del batch; geometria, raw, collisione, clock, torque, power, feedback perso/recupero persistente, starvation, frame rifiutato/parziale, passo/rate/accelerazione, timeout HOLD | Arresto e zero nuovi frame dopo latch; SAFE_OFF non si conferma con prova negativa |
| Quattro STAND completi con fisica e encoder quantizzati, feedback asincrono | PASS; 7 checkpoint, nessuna barriera d'arrivo nei microframe; un frame/release, tutti i giunti post-TX al settle |
| WALK STANDARD, un ciclo oltre ramp, zero velocity e stop coordinato | PASS nominale e loop 6 ms |
| WALK da READY → STAND automatico → WALK; PRIME/ENERGIZE/ARM automatici | PASS, una sola richiesta armer; nel modello rinnovata la TTL anche durante STAND come nel Controller |
| Perdita heartbeat durante WALK | SAFE_OFF richiesto, `LOCAL_PRESENCE_DEADMAN_EXPIRED`, zero frame dopo STOP |
| `run_servo_packet_tests.sh` | PASS 14 check: unico SyncWritePosEx 104 byte / 12 giunti identico alla libreria SCServo 1.0.2 installata; torque-OFF sync 32 byte |
| `run_prime_diagnostic_tests.py --bus-only` | PASS, trasporto originale in stub, 1794 check già esistenti; nessun accesso al dispositivo |
| Contratto geometrico e `git diff --check` | PASS |

Replay negativo della base: i dodici file originali del pacchetto sono stati reinseriti in una copia esterna dei soli input di compilazione host; nessun worktree aggiuntivo. Stesso modello asincrono da 84 ms e stop-short di cinque tick: `TRACKING_LAG_LIMIT`, 69 frame, 1,554 s. Il candidato completa quel modello. Questo riproduce un ramo software fallace; non identifica quale ramo abbia fermato i cinque tentativi hardware (2,5–2,7 s), né pretende di riprodurne tutta la meccanica.

La nuova suite è la regressione del contratto streaming. Le vecchie assertion che richiedevano `TIMED_TRACKED`/completion intermedia finita sono superate intenzionalmente; non vengono presentate come test del nuovo contratto. Non eseguite campagne generiche o prove hardware.

## Durata e cadenza stimate

Modello: 12 servo, polling round-robin con timestamp individuali, quantizzazione reale attraverso resolver gen1 del test, tau 30 ms, stop-short 5 tick, tetti del drive STAND 300/20 invariato, consegna UART modellata a 1,1 ms. Il backend riporta 1,04 ms per il frame di 104 byte. Tempo da fine ARM al finale verificato; PRIME/ARM e attesa operatore vanno aggiunti. Il percorso e i parametri operativi restano 0,19 rad/s, 2 rad/s², passo 4 mrad e periodo nominale 20 ms.

| Modello | STAND s | Frame | HOLD | TX regolari p50/p99/MAX ms | MAX con checkpoint ms | Età feedback MAX ms | Ciclo encoder MAX ms | Tracking peak mrad |
|---|---:|---:|---:|---|---:|---:|---:|---:|
| Loop 4 ms | 17,900 | 857 | 7 | 20 / 20 / 20 | 160 | 12,840 | 16 | 16,256 |
| Loop 4 ms + jitter 0..1,2 ms | 18,426384 | 883 | 7 | 19,064 / 23,904 / 24,580 | 164,587 | 16,310 | 20,539 | 16,225 |
| Loop 21 ms, tre letture/batch | 18,228 | 818 | 7 | 21 / 21 / 21 | 210 | 65 | 84 | 13,194 |
| Loop 6 ms | 18,816 | 901 | 7 | 18 / 24 / 24 | 174 | 18,840 | 24 | 15,767 |

Il ciclo encoder di 84 ms è il periodo tra letture dello stesso giunto; non viene confuso con l'età massima della snapshot, 65 ms in quel replay. I massimi globali TX includono soltanto le attese fisiche dei sette checkpoint: non sono jitter di ogni microframe. Nel caso con jitter i quantili globali TX sono 19,069 / 24,339 / 164,587 ms. Passo peggiore 3,996 mrad, entro 4; margine tracking peggiore 3,744 mrad, senza spendere i 15 mrad di allowance CAD. Nessuna stima di velocità WALK fisica viene ricavata dal replay.

Overhead di calcolo misurato **sul computer host con sanitizer**, per tick del driver completo (fisica, producer simulato, planner/executor/service): p50/p99/MAX nominali 389,9/2127,2/6002,5 µs; jitter 399,7/2227,5/6408,1 µs; ciclo encoder 84 ms 2102,9/2379,3/6306,8 µs; loop 6 ms 416,3/2325,1/4416,9 µs. Non è il WCET dell'ESP32, non entra come tempo sintetico nel clock del modello e non qualifica timing UART, stack o scheduling hardware. Gli overhead simulati di release, TX, polling e settle sono già inclusi nelle durate della tabella.

Confronto con le simulazioni storiche del pacchetto: nominale 17,900 contro 17,360 s (+0,540 s, circa +3,1%); jitter 18,426 contro 19,905 s (−1,479 s, circa −7,4%). Le ipotesi di replay/checkpoint non sono identiche, quindi questi scarti non sono benchmark hardware né uno speedup garantito. La route esplorativa 9,700 s e il caso 12,660 s con percorso rivisto restano non integrati e non qualificati. Non sono promessi 5 s o <1 s. Le durate risultanti sono compatibili con il bound orientativo storico 13,60 s e con ulteriori tempi di accelerazione, polling e settle.

## Candidato, provenienza build e rollback

Un solo commit contiene questa remediation e la suite mirata. La build deve essere eseguita dal suo HEAD CLEAN con `MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh`, output esterno `MOTION_UNIFIED_20261019/build/`. SHA commit, SHA256 APP, manifest, verifica del layout e stato Git finale vengono conservati nelle ricevute esterne `final-receipt.json`, `build.log` e `manifest-verification.json`; il manifest non viene alterato a mano né incluso nel sorgente. La ricevuta finale attesta il risultato effettivo della compilazione successiva al commit, non questo testo preparatorio.

Rollback già disponibile, nuovamente hashato in sola lettura: `b5be64c4e7657b3b8f02deb312c6a2f0dc385812`, APP SHA256 `8afd56037a415ff72d0c56a320f9e1b3671d2dab03b9a9ad5a46b7c0d1be87e7`, file `MOTION_POLLING_20261019_111015Z/build/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`. Dettagli in `rollback.json`. Nessun nuovo dump, modifica o verifica live della scheda.

Restano non validati fisicamente STAND, WALK STANDARD, arrivo a 150 mm, tracking in carico, supporto, termica/tensione durante moto e latenza STOP entro i timeout sincroni reali. Nessuna validazione hardware dichiarata. Flash e singola prova fixture richiedono successiva autorizzazione dell'operatore; questa esecuzione non li avvia.
