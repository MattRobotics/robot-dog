# MATDOG — Motion Hardware Readiness, 13 ottobre 2026

Consegna software per Draft PR #47, continuazione di `4097ec7b49ae1670d488d15e172b09fd97e84026` sul branch `matdog/motion-night-20261012`, esclusivamente nel worktree `/home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog`. Il controllo iniziale ha trovato branch/HEAD corretti, worktree pulito, origin `https://github.com/MattRobotics/robot-dog.git` e baseline presente. PR collegata alla task. Nessun nuovo worktree, reset, clean, rebase o merge.

**A — FIRST SUPERVISED MICRO-MOTION ON FIXTURE: NO-GO fisico.** Implementazione del lifecycle e del micro-movimento presente, ma mancano record/installazione verificati oggi, 12 limiti approvati per la posa effettiva, qualificazione temporale/prime, fixture e taglio fisico dell'alimentazione verificati, envelope elettrico/termico e autorizzazione separata. Questi gate sono indipendenti dal WALK.

**B — AUTONOMOUS PRONE STAND + WALK: NO-GO fisico.** Oltre ai gate comuni, la posa iniziale LOWER è incompatibile con la proposta con margine; membership della route a risoluzione encoder e misure di quota/supporto/carico non sono qualificate. WALK ha quattro punti con derivate non convergenti, swept continuo della prima falcata ancora assente e nessuna prova dinamica. Nessuno di questi problemi è stato trasformato in un PASS hardware.

Nessun dispositivo è stato aperto: nessun flash, OTA, torque ON, movimento, scrittura EEPROM/NVS o attivazione di authority. Firmware installato identificato nella baseline come `b08322dedc5564c080f691dc98047511ef2b953f`; non è questa build candidata. SHA e commit della nuova immagine saranno riportati in `BUILD_RECEIPT.json`, con il suo manifest effettivo.

## Modifiche runtime permanenti

`MotionService → MotionCoordinator → AdaptiveStand → MotionExecutor → ActuatorRuntime → ServoBusActuatorBackend` contiene ora la route pitch-aware congelata. `PairedProneData.h` è generato dal JSON esistente, con 252 knot/251 segmenti e gli estremi dei convex hull dei 17 STL canonici. Questi estremi sono sufficienti per la distanza dal piano: non sostituiscono i triangoli dei certificatori di collisione.

SHA della route originale: `f86ad336ffdaa1bf287f54d8a1a233494bf935156439610fd35ebc25bed411f2`. I 242 campioni originali, le coppie speculari, la priorità posteriore, gli HIP neutrali dopo PRE-ALIGN e i raccordi C4/150 mm sono invariati. Il clock quintico comune conserva posizione, velocità e accelerazione nulle ai raccordi. Il Controller contiene il binding locale tipizzato `ReviewedMotionRuntime`, vincolato a generation, policy epoch, 12 limiti già ammessi e digest della route; non fabbrica l'artifact di approvazione. In boot nessun profilo operativo è approvato.

Stati: OBSERVE → CLASSIFY → PRE_ALIGN → REAR_DEPLOY → FRONT_DEPLOY → LOAD_TRANSFER → RISE → VERIFY_STAND → STAND. Fino al trasferimento è richiesto sostegno ventrale; poi la maschera piedi richiede testimoni esterni. Le richieste di contatto del planner non sono misure.

Un nuovo ingresso autonomo deve corrispondere a un knot certificato, inclusi body pose e 12 joint. Le posizioni intermedie recuperabili sono i knot riconosciuti della route; pose sconosciute, fuori route o non finite sono rifiutate. Le tolleranze di membership (1e-7 rad, 1e-8 m e 1e-7 sui coefficienti di rotazione) sono intenzionalmente strette: **non sono compatibili automaticamente con un encoder da 4096 tick/giro**. Non è stata introdotta una tolleranza hardware permissiva: occorre certificare un tubo di ingresso intorno alla posa effettivamente misurata. Il q0 non fornisce quota né appoggio.

Replay indipendente: 1.757 valutazioni del profilo, confronto dei 12 target e body X/Z/pitch a 2e-12 rad/m. Composizione completa con backend simulato: 3.142 frame eseguiti e 37.704 scritture GoalPosition attraverso mapping individuale/Safe Actuator Layer. Le approvazioni e gli strumenti del test sono dichiarati sintetici; nessun risultato attribuito al robot.

`MotionSession` implementa OFF → PRIMING → ENERGIZING → ARMING → ARMED, prime del PRESENT con torque OFF, torque tramite Safe Actuator Layer, feedback successivo a tutte le scritture e arm dell'Executor. Scadenza/revoca di permit, cambio epoch/generation/owner, fault, drift, feedback stale o perdita fixture provocano SAFE_OFF_REQUIRED. Il Controller ritenta SAFE_OFF dei 12 joint finché tutti risultano OFF. Incertezza di scrittura rimane un fault. STOP autonomo coordinato e arresto immediato per fault mantengono i percorsi precedenti; STOP cancella qualsiasi riavvio di velocità accodato.

Scope locali distinti: FIXTURE_MICRO, AUTONOMOUS_STAND, AUTONOMOUS_WALK. Il primo usa un envelope esecutivo e un percorso locale certificato, senza dipendere dal planner STAND o dalla dinamica WALK. Il micro-test ha un solo joint in escursione, gli altri 11 mantengono target osservati; common quintic, durata finita ≤30 s, ≤600 intervalli, excursion/rate/acceleration e mapping verificati. Nessun default concede escursione. Il gate immagine `kOperationalMotionAuthorized=false` è fisso, senza override di build. Le API locali C++ non hanno binding HTTP né un comando operatore di grant nella candidata: l'adapter locale autorizzato e la review dell'immagine abilitata sono lavoro esplicito successivo ai gate, non un flag nascosto da attivare oggi.

## Geometria e clearance

`adjacent_axis.json`: 12/12 PASS_CONTINUOUS, intero range angolare della route, tutti i triangoli canonici, nessun joint-core escluso e nessuna intersezione intenzionale assunta. Il nuovo metodo cancella il moto comune nel frame della cerniera e usa estremi trigonometrici esatti delle proiezioni dei triangoli, assi SAT e suddivisione limitata. Roundoff budget invariato 1 nm. Il problema del vecchio bound relativo è stato risolto cambiando metodo, senza allentare soglie o cambiare pose.

`assembly_nesting.json`: tutte le 17 triangolazioni chiuse; ray parity con tre direzioni su ogni componente connessa delle 12 coppie adiacenti, nessun annidamento iniziale. In combinazione con la separazione continua, componenti inizialmente esterne non attraversano le altre lungo la route. Non qualifica montaggio reale, giochi, cavi, tolleranze di fabbricazione o solidi fisici non inclusi negli STL.

`nonadjacent_3mm.json`: 251/251 segmenti, 120 coppie/segmento, lower bound continuo minimo **3,0064599155 mm**, riferimento 3 mm PASS geometrico. Il bound precedente ≥118,303 µm rimane una prova conservativa meno forte, non una misura della clearance reale. La query del certificatore ora include soglia richiesta + movimento relativo + budget di ricerca; la soglia fisica di 3 mm non cambia.

`operational_clearance.json` distingue lower bound continuo, witness di superficie (upper bound 7,2685943122 mm per il minimo globale) e distanza minima esatta ancora non determinata. La clearance geometrica reale è quindi nell'intervallo [3,0064599155; 7,2685943122] mm per le coppie non adiacenti; non si dichiara che coincida con nessuno dei due estremi. Clearance sotto carico e sull'assemblaggio attuale BLOCKED misure fisiche.

Contatto previsto: prossimità STL/piano entro i budget congelati. Compenetrazione: superfici che attraversano altri solidi o piano, certificata separatamente. Supporto geometrico: contatti nominali/poligono assumendo piedi portanti. Carico fisico: forze/reazioni/slittamento/capacità attuatori, non dimostrati. Encoder, IMU e corrente/tensione non costituiscono un sensore di appoggio.

## Calibrazione e limiti

La Full Calibration 24/24 del 1 ottobre è conservata; non è stata ripetuta. Il gen-1 è stato salvato dopo la nuova calibrazione del **6 ottobre**, non è il semplice record del 1 ottobre. Fonte di q0/fine contacts: `MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md`, con SAVE slot A e ACK generation 1. Reset verificato storicamente, power-cycle completo non dimostrato. Nessun backup NVS sensibile copiato o pubblicato.

Il runtime usa esclusivamente trasformazioni dal record corrente restituito dalla RESTORE e ammesso contro geometria e identità fisiche. I numeri storici nel report sono confronti per review, non offset compilati nel Motion runtime. ID bus da solo non prova l'identità del servo.

`joint_limits_review.json` e `JOINT_LIMITS.md`: 12 proposte, **0/12 approvate**, con fonti, contatti, range per fase, margine URDF, margine verso proposta e verifica richiesta. La proposta interseca l'URDF con i contatti gen-1 usando inset 16 tick + spread fine osservato. È un margine candidato aritmetico, non una qualificazione statistica di backlash o carico.

I LOWER iniziali −92° hanno margine URDF zero. Verso la proposta le carenze sono LF 1,472656°, RF 2,615234°, RH 1,956055°, LH 2,087891°. La route geometrica congelata rimane valida come candidato; l'eseguibilità sull'installazione è bloccata. Un difetto di envelope è ora provato, ma correggere la posa iniziale richiederebbe un nuovo raccordo certificato verso una posa misurata: non viene rigenerata alla cieca l'intera traiettoria.

LF UPPER: contatto MAX del 1 ottobre 122,96° contro URDF 122,5° (spare 0,46°); il confronto gen-1 del 6 ottobre dà 121,464844°, 1,035156° interno al bordo URDF. La proposta limita MAX a 119,882813°. La route usa MAX 86,821178° e conserva 33,061635° dalla proposta: questo non approva l'intero range URDF né elimina l'obbligo di controllo dell'installazione/zero e del margine caricato.

Posizione candidata calcolabile offline. Rate/accelerazione/tracking caricati, torque/output ceiling, envelope elettrico/termico, stop latency e bound temporali del bus **non qualificabili dai contatti disponibili**. I valori .25 rad/s e 2 rad/s² del compiler sono timing software candidati, non limiti hardware approvati.

## Producer, ownership e Dashboard

`OperationalFeedback` conserva 12 slot bounded, timestamp individuale e lettura raw separata da actual semantico. Ogni fallimento sostituisce lo slot precedente; snapshot stale/futuro, status errore, torque invalido, temperatura fuori dominio e raw fuori range sono rifiutati. Mapping SI richiede le trasformazioni accettate.

Controller legge al massimo un ID per tick, quando discovery/calibrazione non possiedono il bus. Usa BNO085 quaternion/timestamp e BMS reale/allarmi/freshness. Quota e sostegno provengono solo dal producer locale di osservazioni esterne. Default UNKNOWN: nessuna misura sintetica o altezza IMU inventata. Il profilo qualificato deve specificare freshness IMU e limiti elettrici/termici; i default zero approvano nulla.

L'attuale lettura ControlFeedback usa due timeout da 100 ms. Il ciclo completo di 12 ID può dunque violare una deadline operativa: **WCET/round-trip/dropout/STOP fisici sono gate OPEN**, il fatto che il polling sia bounded non li rende qualificati. Occorre misurare e, se necessario, sostituire la strategia con un reader a timeout/deadline qualificati. A non richiede un 50 Hz WALK, ma richiede comunque age/abort/holding efficaci per il proprio envelope.

Dashboard conserva 48 azioni, Digital Twin e joystick. JSON espone actual encoder, target comandato, raw tick e timestamp separati; RESTORE/12 limiti/permit/fault visibili. Server rifiuta le azioni non ammesse. STOP attraversa il servizio sicuro; autenticazione/CSRF restano obbligatorie e il taglio fisico resta indipendente dalla rete.

RUN_STA: HTTPS sul socket STA dell'IP effettivo, phase CLIENT_STA/RUN_STA, connessione/generation e driver verificati; Host e Origin esatti, cookie Secure/HttpOnly/SameSiteStrict, CSRF e binding al peer. Credenziali remembered-device limitate all'AP. I login Motion non concedono authority. Configurazione TLS locale valida resta necessaria; nessuna chiave/certificato sensibile aggiunta alla PR. Provisioning/radio/certificato browser sul robot non sono stati collaudati.

Browser IAB su `127.0.0.1:5185`: pagina reale embedded, catalogo 48 azioni, RESTORE REQUIRED, limiti UNAPPROVED, permit REVOKED, encoder UNKNOWN, target NONE, STAND/joystick disabilitati e STOP con `OFFLINE_FIXTURE_ACK_ONLY`. Il backend browser è dichiaratamente offline; il suo ACK non è un test di arresto fisico. Test HTTP usano handler, parser, PortalSession e MotionService di produzione con socket/driver/mailbox doubles.

## WALK e task successivi specifici

G4 e FeedbackWalk conservati. L'adapter bounded a cinque iterazioni usa gli estremi STL canonici nel planner permanente; i target delle 214 pose della demo corrispondono entro 3,22e-15 rad. Le derivate della mappa completa includono la correzione STL, body/feet e yaw. Test con stencil h/h/2 e tolleranze 1e-7 rad/s, 1e-5 rad/s²: 210 pose convergono, quattro punti (21; 42,5; 53,5; 54,5 s) cambiano witness di contatto e vengono rifiutati. Non viene aumentata la tolleranza per ottenere PASS. `stl_gait_runtime.json` registra BLOCKED_DERIVATIVE_WITNESS_SWITCH; il percorso position-only è un oracle, non il planner eseguibile.

Parametri demo: WALK/G4, height .150 m, lift .005 m, duty .8, advance .005 m/ciclo, sway .040 m, shift iniziale/finale 8 s e clock quintico su un ciclo di 90 s; vy=yaw=0. Il vx medio 5mm/90s non è il parametro periodico di un G4 a velocità costante. Nessuna autorizzazione a riprodurre la demo sul robot. Continuità STAND→GAIT_START e ritorno all-foot sono implementati; nuovi vx/vy/yaw vengono accodati, arresto coordinato ai vecchi anchor, rebase solo a STAND tracciato e nuova rampa. STOP/TTL/fault cancellano la coda. La suite testa il cambio combinato e la cancellazione, senza riscrivere lo scheduler.

Prima falcata: baseline sampled geometry PASS, **continuous swept WALK ancora BLOCKED**. La certificazione della route prona non copre il G4 tra i campioni o i nuovi comandi di velocità. Non è utile certificare come eseguibile un clock che il runtime rifiuta per derivate: prima occorre isolare gli switch STL, scegliere un riferimento di contatto fisicamente corretto e differenziabile oppure arrestare la progressione in quei raccordi; poi qualificare derivate e swept continuo della funzione runtime effettiva per stance/swing, entry/stop/restart e per il box vx/vy/yaw richiesto. Segue supporto caricato/dinamica/first stride con operatori presenti. Gate autonomo distinto dal micro-test A.

## Consegne e prossima azione personale

- `FIRST_MICRO_MOTION.md`: gate minimi, sessione read-only possibile ora, successiva procedura di movimento subordinata a review e autorizzazione separata.
- `AUTONOMOUS_STAND_WALK.md`: gate aggiuntivi e procedure staged.
- `JOINT_LIMITS.md` + JSON: matrice 12 limiti e margini per tutte le fasi.
- `TEST_RESULTS.json`: PASS/FAIL/BLOCKED separati, test ed evidenze.
- `BUILD_RECEIPT.json`: binario candidato locale, SHA256, manifest, source commit esatto, layout 16MiB 2×5MiB, authorization 0.
- `HANDOFF.md`: branch/commit finale, PR e percorsi.

La prossima azione personale è una **sessione locale read-only con il robot sostenuto e alimentazione servo fisicamente interrompibile**, documentando firmware effettivo, integrità/generation del record, identità dei 12 servo, encoder/torque/GoalPosition e temperature/tensioni, quaternion/timestamp, BMS e posa/linkage reali. Non richiede rifare Full Calibration. Non usare RESTORE nuova sul vecchio firmware come se fosse già disponibile; identificare prima l'immagine installata e i suoi comandi. Poi review del piccolo workspace osservato e dei gate A, prima di chiedere la separata autorizzazione per una nuova immagine e per il singolo micro-test.
