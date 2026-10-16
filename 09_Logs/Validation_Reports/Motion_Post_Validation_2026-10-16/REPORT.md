# MATDOG — audit offline post validazione Motion del 16 ottobre 2026

## Esito e perimetro

Root cause riprodotta: **il clock di inizio loop viene usato per ammettere STAND dopo una lettura servo completata nello stesso loop**. Il giunto appena letto appare futuro rispetto a quel clock; `MotionSession::gate()` lo rifiuta correttamente rispetto agli input ricevuti, ma l'input temporale del Controller è sbagliato. Non serve allargare la finestra di feedback. Una seconda causa indipendente fa scadere la presenza quando Motion Control non è la scheda selezionata: polling ogni 4 s contro deadman di 3 s.

Questa revisione prepara un candidato; **PRONE→STAND non è ancora validato sull'hardware**. Non sono stati aperti dispositivi seriali, eseguiti flash, comandi ai servo, scritture NVS/EEPROM, push o merge. Build e test sono locali. Il worktree Motion originale e il checkout iniziale con le sue modifiche sono preservati. Worktree di lavoro: `/home/matteo-manicardi/.codex/worktrees/motion-post-validation-20261016/robot-dog`, branch `fix/motion-post-validation-20261016`.

Baseline: `2c2c13bf52c125194d692f666606c0bd3600adfc`; applicazione installata dichiarata dall'operatore: SHA-256 `92472cd00f6ef9745361fcb1452d63c2ddda520b7eb445a4ceea98e485ac0cd5`. La ricevuta già esistente `~/MATDOG/verification-artifacts/FLASH_ISSUE52_20261016/APP_FLASH_RECEIPT_3.json` conferma quell'associazione e il readback del flash precedente; nessun nuovo readback è stato eseguito.

## 1. Sequenza temporale dei comandi 4 e 5

Fonte: `HW_MOTION_VALIDATION_20261016/session_20261016T180951Z.log`, SHA-256 `61dcda9bc9f1d01bfa9c989c6fcd6e05342e437ac1a0999c1c41115eda720da1`. Analizzati 8474 record `@TEL`: executor sempre OFF; età RAW massima pubblicata 85 ms. Questo dimostra la freschezza ai campioni USB, non in ogni istante intermedio.

Il logger usa esplicitamente `datetime.now(datetime.timezone.utc)`: i prefissi USB sono UTC. Il 16 ottobre Europe/Rome è UTC+2. Gli orari seguenti includono entrambe le convenzioni. I timestamp del Controller sono invece uptime monotono; non sono UTC né ora civile. Nessun clock è stato modificato.

| Evento | Riga | UTC | Europe/Rome | Controller ms | Esito osservato |
|---|---:|---|---|---:|---|
| Prima di comando 4 | 7575 | 18:25:44.774620 | 20:25:44.774620 | 1160083 | ultimo comando 3, permit/support/hardware true, OFF; motivo storico DEADMAN |
| Prima osservazione comando 4 elaborato | 7576 | 18:25:45.282773 | 20:25:45.282773 | 1160591 | last_sequence=4; sessione OFF, reason STALE_FEEDBACK |
| Prima di comando 5 | 7658 | 18:26:26.571301 | 20:26:26.571301 | 1201879 | ultimo comando 4, permit/support/hardware true, OFF; storico STALE_FEEDBACK |
| Prima osservazione comando 5 elaborato | 7659 | 18:26:27.075360 | 20:26:27.075360 | 1202383 | last_sequence=5; sessione OFF, reason STALE_FEEDBACK |
| Presenza successivamente scaduta | 7676 | 18:26:35.624022 | 20:26:35.624022 | 1210932 | permit=false; OPERATOR_GO_REQUIRED; storico sovrascritto DEADMAN |

`ack=true` indica una risposta semantica ricevuta: `outcome=REJECTED` resta un rifiuto. Non certifica esecuzione né movimento. Lo stato OFF e la mancanza di una sessione ammessa sono coerenti con nessuna sequenza PRIME/ENERGIZE avviata.

Ordine giunti: LF HIP/UPPER/LOWER, RF HIP/UPPER/LOWER, RH HIP/UPPER/LOWER, LH HIP/UPPER/LOWER. ID: `13,12,11,23,22,21,33,32,31,43,42,41`.

| Giunto/ID | 4 prima stamp / age ms | 4 dopo stamp / age ms | 5 prima stamp / age ms | 5 dopo stamp / age ms |
|---|---|---|---|---|
| LF HIP 13 | 1160032 / 51 | 1160591 / 0 | 1201850 / 29 | 1202345 / 38 |
| LF UPPER 12 | 1160037 / 46 | 1160528 / 63 | 1201860 / 19 | 1202350 / 33 |
| LF LOWER 11 | 1160042 / 41 | 1160533 / 58 | 1201865 / 14 | 1202359 / 24 |
| RF HIP 23 | 1160047 / 36 | 1160538 / 53 | 1201870 / 9 | 1202364 / 19 |
| RF UPPER 22 | 1160052 / 31 | 1160543 / 48 | 1201879 / 0 | 1202369 / 14 |
| RF LOWER 21 | 1160057 / 26 | 1160548 / 43 | 1201811 / 68 | 1202378 / 5 |
| RH HIP 33 | 1160063 / 20 | 1160553 / 38 | 1201816 / 63 | 1202383 / 0 |
| RH UPPER 32 | 1160068 / 15 | 1160558 / 33 | 1201821 / 58 | 1202315 / 68 |
| RH LOWER 31 | 1160073 / 10 | 1160563 / 28 | 1201826 / 53 | 1202320 / 63 |
| LH HIP 43 | 1160078 / 5 | 1160571 / 20 | 1201832 / 47 | 1202325 / 58 |
| LH UPPER 42 | 1160083 / 0 | 1160576 / 15 | 1201840 / 39 | 1202330 / 53 |
| LH LOWER 41 | 1160022 / 61 | 1160581 / 10 | 1201845 / 34 | 1202340 / 43 |

**Limite probatorio:** i log non contengono il clock passato al gate, la copia per-giunto esattamente all'ammissione, né un dump degli epoch per quei due comandi. Non è possibile identificare onestamente l'ID rifiutato o l'esatta età di gate dai campioni successivi a 2 Hz. La tabella riporta i campioni reali ai bordi, non un'interpolazione. Il difetto è dimostrato dall'ordine di chiamata del firmware installato e riprodotto sulle dodici fasi round-robin. L'assenza di stale reale nell'intervallo fra i campioni non è dimostrabile retroattivamente.

Lo script riproducibile `05_Firmware/MATDOG_Controller/scripts/audit_motion_validation.py` produce `audit_evidence.json` e `commands_4_5_feedback.csv` nella directory esterna degli artefatti.

## 2. Percorso degli snapshot, clock ed epoch

Sequenza della baseline:

1. `loop()` chiama `Controller::update(millis())`: clock T0.
2. `updateOperationalMotion(T0)` legge un servo; il campione riceve `millis()` T1 alla fine del blocco, e `sampled_at_us` dal clock monotono esteso.
3. Dentro questa funzione `now_ms=millis()` corregge solo la **copia locale** dell'argomento. `OperationalFeedback::snapshot(T1, age)` produce `operational_feedback_` valido.
4. La posa misurata viene copiata in `MotionCoordinator::observation_`; la telemetria viene copiata in `MotionService::telemetry_`. Questi snapshot hanno scopi diversi; non sono due produttori di acquisizione. `OperationalFeedback` mantiene anche la propria copia mappata, da cui viene costruito lo snapshot filtrato. Non sono timestamp sintetici né timbrati dal heartbeat.
5. Il chiamante conserva T0 e invoca `http_transport_.update(T0)` alla fine del loop. Il servizio inoltra T0 a `MotionService::command`, `MotionCoordinator::accept`, `armOperationalSession`, `MotionSession::request`.
6. Con T1>T0, `int32_t(T0 - fb.sampled_at_ms[i]) < 0`: rifiuto STALE_FEEDBACK prima di PRIME. L'envelope operativo resta quello esistente (default feedback 400 ms); il confronto negativo è una protezione reale da mantenere.
7. `MotionService::statusJson(T0)` esclude lo stesso giunto dal conteggio RAW perché futuro. USB usa invece il clock locale aggiornato e può mostrare 12/12. Quindi RAW 11/12 **non dimostra** uno stallo HTTP di 400 ms: esiste una spiegazione deterministica nel codice, indipendente dal carico HTTP.

La patch passa `millis()` al servizio HTTP dopo le acquisizioni e ricampiona il produttore con il tempo corrente all'ingresso di `armOperationalSession`, dopo `preArmCheck`. Quest'ultimo può aver impiegato tempo per la geometria: un feedback davvero invecchiato durante il calcolo resta rifiutato. Non vengono inventati campioni, usati target al posto degli encoder o tolti controlli futuri/stale.

`policy_epoch`, generazione della calibration/runtime review, generazione del permit ed authority lease restano distinti. L'ammissione verifica il runtime contro calibrazione/epoch; la sessione cattura epoch e lease, poi rifiuta `SESSION_CHANGED` se cambiano. Nessun epoch è stato riavviato per aggirare il rifiuto. Le copie esistenti sono aggiornate nel Controller cooperativo: nessun nuovo produttore concorrente viene introdotto. Il candidato aggiunge `MOTION_ADMISSION now_ms=… epoch=… generation=… lease=… result=… joints=slot:stamp:age:read_ok,…` dopo la decisione. Questa è la traccia da acquisire al prossimo tentativo; non è una ricostruzione dei log vecchi.

Distinzioni operative:

- **Falso stale:** età negativa generata dal clock T0 rispetto a un campione T1 correttamente acquisito nello stesso loop.
- **Stale reale:** read fallita, valore non finito, timestamp futuro rispetto al vero tempo di valutazione, oppure età oltre l'envelope. Rimane rifiuto/SAFE_OFF.
- **Motivo storico:** `local_session.reason` con stato OFF non rappresenta il permit attuale né l'esito dell'ultimo poll. Prima il deadman sovrascriveva quel campo anche senza sessione attiva. Ora l'expiry revoca il permit ma chiama `revoke(DEADMAN)` solo per una sessione non OFF; la UI etichetta esplicitamente lo storico.

## 3. Deadman, sessione e Dashboard

Difetto confermato: il timer era `panel.hidden ? 4000 : 1000`; cambiare scheda nella stessa pagina poteva quindi superare il deadman operativo di 3000 ms anche con pagina in primo piano. Il link `/twin/` navigava inoltre fuori dalla Dashboard. Entrambi sono indipendenti dal falso stale.

Correzioni:

- Poll Motion ogni secondo su tutte le schede visibili, con periodo calcolato dall'inizio del poll. Pagina nascosta/chiusa, rete persa o autenticazione scaduta non generano presenza permanente: senza poll autenticato il Controller scade dopo 3 s. Nessun timer server autonomo rinnova il permit.
- Resume della sessione cookie prima del primo poll, anche con hint di login precedente; rinnovo della sessione live in pagina visibile prima dei 5 minuti. Cookie/CSRF e origine AP restano richiesti. Le preferenze browser non concedono autorità.
- RESTORE automatico riuscito nasconde la conferma manuale dell'installazione. La checkbox rimane solo nel recupero di un RESTORE assente/rifiutato; non è un'autorizzazione hardware memorizzata a tempo indefinito.
- STAND richiede anche `snapshot.readiness.permit===true`, oltre a stato fresco e capability disponibile. Una capability vecchia con `available=true` non vince sul permit corrente revocato. WALK START segue la stessa regola.
- STOP viene mantenuto nell'intestazione, visibile su tutte le schede, e conserva la corsia indipendente dalle letture/comandi ordinari. Motion Control è la seconda scheda.
- Risposta REJECTED mostrata come rifiuto con il motivo reale; HTTP error/status e timeout restano distinti dagli errori JSON. Il POST non viene ripetuto automaticamente.
- Letture diagnostiche ordinarie limitate a 800 ms nella lane comune: budget deadman 3000 meno periodo 1000 e budget di dispatch Motion-read 1200. Non è un aumento di timeout Motion; su rete lenta si preferisce una diagnostica incompleta a bloccare il heartbeat dietro un GET diagnostico da 6 s.

`HTTP_TIMEOUT` e `ADMIN_AP_REQUIRED`: i log USB consegnati non conservano richieste/risposte HTTP del primo tentativo, perciò non identificano il socket o il pacchetto che ha fallito. `ADMIN_AP_REQUIRED` è un diniego del percorso di rete/amministrazione, non la prova di encoder stale; `HTTP_TIMEOUT` indica risposta non conclusa entro il timer browser, non dimostra perdita di comando. La regressione esercita 403 testuale, 503 JSON, rete persa, timeout anche sul body, retry GET limitato e nessun replay POST. Nessun bypass dell'AP è stato aggiunto.

## 4. HTTP, USB, memoria e stack

Il Controller coopera nello stesso thread per poll servo, planner/executor, formattazione JSON e mailbox HTTP. L'HTTPD gestisce le richieste nel proprio singolo task; i dati sono ottenuti dalla mailbox. Una trasmissione grande sul task HTTP può ritardare i successivi heartbeat; un JSON grande sul Controller può allungare il loop. STOP ha una mailbox dedicata ma non preempie codice già in esecuzione: il limite temporale va misurato.

Lo scheduler Motion mantiene le scadenze assolute esistenti e non recupera frame arretrati a raffica. Il round-robin conserva i timestamp di fine lettura per giunto; i dodici encoder non sono simultanei. Durante streaming vengono usati fast feedback e controllo lento già presenti. Non sono cambiati rate, profili, tracking, limite di corrente/coppia o calibrazione.

USB `emitTelemetry` già ammette solo record interi se `Serial.availableForWrite()` basta, con TX non bloccante. Il candidato conserva questo comportamento; un record può essere perso, osservabile dal salto `tel_seq`. La formattazione a 10 Hz ha comunque un costo CPU. Nel log il passaggio 10→0 Hz non è necessario per mantenere FRESH; a 2 Hz il comando arriva ma incontra lo stesso errore di clock. Non attribuire quel rifiuto alla saturazione USB.

| UTC | Heap libero B | Minimo storico B |
|---|---:|---:|
| 18:10:01 | 50740 | 48300 |
| 18:20:37 | 47396 | 21160 |
| 18:22:37, USB disattivato | 47396 | 21160 |
| 18:33:45 | 48800 | 21160 |

Il minimo è monotono per definizione e non risale liberando memoria. Il recupero del libero esclude una perdita permanente di circa 27 KB in questa cattura; non prova assenza di ogni leak in sessioni più lunghe. Il buffer temporaneo capability è 20480 B, oltre al buffer persistente del Controller; socket e rete richiedono ulteriore memoria: compatibile col picco misurato, senza attribuzione esatta delle allocazioni. Nessuna riduzione arbitraria di stack/buffer è stata applicata.

Loop stack esistente: 16 KiB. HTTP stack AP: 24 KiB; TLS: 28 KiB. Non sommare i frame dei due task come se fossero uno stack. La traccia nuova espone heap interno libero, minimo storico, largest contiguous block e minimo stack libero del loop sia in HTTP Motion status sia in USB `runtime`. Non è un nuovo dato misurato del robot; servirà alla prossima cattura. L'analisi ELF del candidato e le identità delle build sono in `BUILD_AND_TEST_RESULTS.md`.

## 5. Twin e separazione delle mesh

Twin integrato nella prima Dashboard tramite iframe del **bridge host già presente**. L'URL host viene configurato una volta e conservato nel browser; senza host la pagina spiega il dato mancante e non inventa un feed LIVE. Il telefono deve raggiungere il computer host (es. stesso AP/rete), e il bridge deve essere l'unico proprietario USB se usato per la telemetria. Non avviare contemporaneamente bridge e logger seriale.

Il listener HTTP del Controller rifiuta gli asset Twin oltre 8192 B anche quando idle (`TWIN_HOST_REQUIRED`); durante Motion mantiene il motivo `TWIN_DEFERRED_MOTION_ACTIVE`. Non può più iniziare a trasmettere `twin.js` da 142623 B gzip o mesh grandi in competizione con le richieste di controllo. Le vecchie URL firmware 3D non sono il percorso supportato del candidato. I byte degli asset firmware e il certificato geometrico non vengono rigenerati.

Nel viewer host `<visual>` e `<collision>` sono letti separatamente, con i loro origin/scale e oggetti distinti. Si visualizzano le mesh visuali; un controllo mostra il layer collisione viola. La certificazione continua a usare le collision originali. Staging copia l'URDF verbatim e verifica i file; nessuna modifica a `03_CAD`, SHA256SUMS, collision pack certificato o profili Motion. Il viewer compatto/mobile continua a usare collision geometry e non diventa una nuova fonte di certificazione.

L'assenza di trasferimenti 3D dall'HTTPD evita la contesa applicativa identificata. Il traffico radio del PC/telefono, le allocazioni Wi-Fi e il costo del render sul telefono restano da stressare fisicamente: precaricare le mesh prima di STAND e registrare loop/RTT/deadman. Non è dimostrata una garanzia hard real-time della radio da test offline.

## 6. Energia

Evidenza DALY comunicata dall'operatore: minima 3190 mV (cella 3), massima 3290 mV, pacco 9,7 V, SOC 33,2%; protezioni hardware 3200 mV/cella e 9,60 V/pacco. La cella minima ha attraversato il limite anche se il pacco era ancora sopra la soglia e il SOC suggeriva un terzo residuo. I campioni BMS del log Motion qui analizzato sono invece attorno a 3771–3785 mV: non sono una cattura dell'evento di sottotensione.

Avvisi candidati, **solo presentazione**, indipendenti dal SOC:

| Livello | Ingresso (una delle due) | Rilascio (entrambe) |
|---|---|---|
| Warning | cella minima ≤3500 mV oppure pacco ≤10,50 V | cella ≥3600 mV e pacco ≥10,80 V |
| Critical | cella minima ≤3350 mV oppure pacco ≤10,05 V | cella ≥3450 mV e pacco ≥10,35 V, poi rivaluta warning |

Sono margini conservativi sopra le protezioni note, non nuove soglie DALY né una stima di autonomia sotto carico. Restano anche gli avvisi SOC preesistenti. La tensione è valutata solo con campione valido/fresco e tensioni plausibili; gli avvisi non vengono presentati come scarica durante la carica. SOC invalido non sopprime un avviso di tensione valido. I latch sopravvivono a dati stale ma non sono mostrati come allarmi attuali senza campione fresco. Nessuna scrittura al BMS o comando di spegnimento viene aggiunto. Le soglie advisory vanno validate con caduta sotto carico e sbilanciamento reale; non si abbassano i 3,20 V/9,60 V hardware né il gate tensione servo già esistente.

## 7. Procedura per il prossimo test hardware (non eseguita)

1. Il flash è un'attività successiva e richiede l'autorizzazione specifica di quella sessione. Verificare profilo ROBOT_POWERED, SHA del binario, commit sorgente e manifest nel dossier build. Nessun flash automatico da questo report.
2. Robot sospeso sulla fixture, zampe e cavi liberi, volume sicuro, caricatore scollegato, alimentazione e KEY controllabili dall'operatore. Non usare il test a terra. Preparare STOP visibile e taglio fisico dell'alimentazione.
3. Batteria caricata con celle sopra gli avvisi e senza allarmi; leggere minima cella, pacco, MOS e tensioni servo. Non usare il solo SOC per decidere. Non muovere se ci sono warning energetici o cali anomali; documentare prima la condizione.
4. Un unico proprietario USB. Aprire Dashboard AP autenticata, configurare il bridge host e precaricare il Twin. Verificare RESTORE automatico, 12 RAW freschi, posa PRONE riconosciuta, autorità GRANTED/READY, nessuna coppia. Prima di armare, verificare refresh e passaggio Dashboard↔Motion↔altre schede per almeno 30 s: presenza continua, nessun picco di gap/timeout. Nascondere la pagina a coppia OFF: presenza deve decadere, STAND disabilitarsi e non riattivare una sessione da sola al ritorno.
5. Prima prova a 2 Hz USB; raccogliere contemporaneamente timestamp UTC e uptime, età per giunto, heap libero/minimo/largest, minimo stack, RTT/gap e traccia di ammissione. `MOTION_ADMISSION` deve riportare il vero clock, tutti i read_ok e ages non negative entro l'envelope. Una riga incompleta non è evidenza: TX diagnostica è best effort.
6. Un solo STAND esplicito. Attesi PASS d'ammissione → PRIMING → ENERGIZING → ARMING → ARMED → entry → salita → STAND_VERIFIED. Il PASS di ammissione non autorizza ignorare fault successivi. Nessun retry cieco dopo un rifiuto: salvare motivo, ID/age e stato.
7. Interrompere subito con STOP/taglio fisico se tracking non converge, il robot urta, una cella cede, compare fault/timeout o si perde presenza. Confermare SAFE_OFF e leggere coppia OFF di tutti i 12 giunti; non assumere che ack STOP da solo provi il completamento.
8. Dopo una prova riuscita, STOP e controlli memoria/stack; solo in una prova successiva confrontare 0/2/10 Hz USB e stress del viewer. Una perdita heartbeat durante sessione attiva deve condurre a SAFE_OFF: provarla su fixture con operatore pronto, mai come primo movimento e mai a terra. Niente WALK/FAST finché PRONE→STAND e STOP non sono misurati.

Rischi residui: dinamica fisica non validata, durata/WCET di geometria e letture reali, margine heap frammentato, rete radio, risposta del servo al PRIME, sbilanciamento/sag batteria. Il candidato corregge difetti riproducibili di ammissione e UI, non qualifica questi aspetti tramite simulazione.
