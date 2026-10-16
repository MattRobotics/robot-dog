# Candidato offline — build, regressioni e stack

Sorgente firmware e viewer: commit locale **`652c2e2b526694839d11351a6ed6fd947e6f7588`**. Entrambe le build sono state avviate con sorgente CLEAN e `GIT_DIRTY=0`, prima della scrittura di questi report. Le eventuali revisioni successive di sola documentazione non cambiano l'identità dei binari. Nessun upload.

Artefatti completi: `~/MATDOG/verification-artifacts/MOTION_POST_VALIDATION_20261016/`.

## Build

| Profilo | Binario B | Slot usato | SHA-256 applicazione |
|---|---:|---:|---|
| USB_ONLY | 3876848 | 73,9% | `072f850bb161539f1b1e0efe3e9519b6b78c5896b55c32f48c5f3cc4e2f09096` |
| ROBOT_POWERED | 3885168 | 74,1% | `ac972a5ef802988af6a515903a5afd5589d3c2d0e04400f99f69670d8ff499d7` |

Entrambe PASS. Layout immutato `MATDOG_16M_2x5M_NVS_V1`, slot app 5242880 B, SHA tabella partizioni `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`. OTA ingest=0, default sorgente USB_ONLY preservato. `MOTION_AUTHORIZED=0` e `MOTION_AUTHORITY=RUNTIME_GOVERNED` nei manifest: non è stato introdotto un flag di autorizzazione permanente.

Percorsi relativi alla directory artefatti:

- `build-usb/artifacts/USB_ONLY-ota0/MATDOG_Controller.ino.bin` e relativo `matdog_build_manifest.txt`.
- `build-powered/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`, ELF/map e relativo `matdog_build_manifest.txt`.
- Log `build-usb.log`, `build-powered.log`; versioni/board/FQBN/UTC nei manifest.

ROBOT_POWERED: il compilatore riporta globali 182028 B, spazio staticamente restante 145652 B. Questo **non è** l'heap libero runtime: task, Wi-Fi, buffer e allocazioni riducono il valore dopo il boot.

Warning di build preesistenti: lambda `word` inutilizzata in ServoBus, parametro ACC in SCServo, e accesso `Mem[-23]` nel ramo cache di `SMS_STS::ReadMode`. Quest'ultimo metodo è presente nella libreria/vtable linkata, ma non è chiamato dal codice ServoBus del Controller (che usa letture validate). Il warning non è stato soppresso né la libreria installata modificata; rimane un rischio di dipendenza da esaminare prima di usare quel metodo. Nessun nuovo warning nei file modificati.

## Regressioni concluse

| Verifica | Risultato | Evidenza |
|---|---|---|
| Clock/deadman, hook Controller estratti | PASS; limite 3000/3001 ms, futuro, wrap uint32, OFF storico, sessione attiva→SAFE_OFF, auth≠presenza | `clock.final.log` |
| Ammissione e STAND operativo | PASS, **768673 check**, zero failure; producer reale, sessione/executor/planner reali con backend servo sintetico | `stand.final.log` |
| Canali feedback / RAW | PASS, 29 check; falso 11/12 da timestamp futuro, ritorno 12/12 al clock corretto, stale reale escluso, mapping separato | `channels.final.log` |
| HTTP/security/mailbox | PASS, 78 check; handler produzione estratto con driver/socket double, asset grandi rifiutati idle/active, STOP e non replay | `http.final.log` |
| Dashboard Motion DOM | PASS; 48 capability, permit corrente prevalente, ordine schede, Twin iniziale, RESTORE nascosto, STOP globale, timeout/403/503 | `dom.final.log` |
| Portale DOM | PASS, 167 check; login/resume/logout, stale, XSS, sola lettura, configurazione invariata | `portal.final.log` |
| Policy LED/energia | PASS, 113709 check; 3190 mV/9,7 V con SOC 33,2%, isteresi, SOC NaN, stale, charging | `led.final.log` |
| Suite operativa completa, eseguita dal gate generale | PASS, **8813802 check**, zero failure; include survey STANDARD/FAST e controlli tracking con modelli sintetici | `static-audit.log` |
| Gate e mutazioni LED dopo aggiornamento del produttore | PASS, **113/113** casi; 69 statici, 42 policy compilate e 2 baseline | `led-static.final.log`, `led-mutation.final.log` |
| Viewer host | PASS, **85 test**; typecheck, staging/hash canonici, separate visual/collision scene objects, build Vite | `viewer.final.log` |

Verifica visiva aggiuntiva con browser reale su `motion_dashboard_demo.py --port 5187`, solo fixture locale: Dashboard iniziale, Twin host, Motion seconda scheda e STOP globale visibili. Nessuna connessione al robot. Tab e server di prova chiusi.

Il test STAND è stato eseguito con `MATDOG_STAND_ONLY=1`; include anche `operationalRefusals` e `heartbeatCoupling`, senza eseguire la costosa survey completa dei range WALK. Le prove STAND arrivano a STAND_VERIFIED per le pose simulate, preservano limiti/keep-out e tracking; il modello con feedback più lento impiega di più. I tempi 21–59 s sono risultati dei modelli, non una promessa al robot.

I log `operational.log` e `operational.final.log` appartengono a tentativi ridondanti della suite WALK completa interrotti intenzionalmente per evitare lavoro duplicato. Non sono test conclusi PASS e non sono usati come evidenza. Il gate generale ha inoltre concluso la suite operativa completa: OPERATIONAL_MOTION PASS, 8813802 check. Il negative control FAST con drive STANDARD viene intenzionalmente fermato da TRACKING_LAG_PERSISTENT: la riga STREAM_WALK FAILED per quel caso è attesa, non un errore nascosto. Questa simulazione non qualifica lo stack ESP32 né il carico fisico. Il comando generale iniziale ha concluso con tre finding tutti relativi al nuovo produttore advisory batteria (due rifiuti della vecchia regola e il baseline test LED conseguente). Il gate è stato aggiornato in modo stretto per riconoscere soltanto il nuovo blocco advisory: freschezza, dati cache, soglie, isteresi e maschera di carica sono fissati; gli altri produttori/alias restano vietati. Il controllo LED rieseguito è PASS e i metatest sono PASS 113/113; gli altri controlli già conclusi non vengono ripetuti. `static-audit-resolution.json` documenta la verifica composta e prova che l'unica funzione di audit cambiata è `check_led_status_boundaries`, con sorgenti compilati invariati rispetto alle build. Il log del primo FAIL è conservato: non è presentato come una nuova esecuzione integrale PASS.

## Analisi stack del candidato ROBOT_POWERED

Metodo: `xtensa-esp32s3-elf-objdump -d -C` sull'ELF effettivamente prodotto; frame dall'istruzione Xtensa `entry a1,N`, in byte. Script e dati: `analyze_stack.py`, `stack_analysis.json`, `powered.disassembly.txt` negli artefatti. I percorsi automatici considerano solo chiamate dirette a simboli esatti; sono un supporto all'ispezione, **non un limite superiore**. Jump table possono essere disassemblate come istruzioni e i call virtuali/indiretti richiedono verifica manuale. Non usare una somma automatica di ogni ramo o di task diversi come misura reale.

| Funzione | Frame B |
|---|---:|
| Controller::update | 128 |
| Controller::updateOperationalMotion | 1392 |
| HttpTransport::update | 48 |
| HttpTransport::serviceRequest (task Controller) | 4576 |
| MotionService::command | 400 |
| MotionCoordinator::accept | 336 |
| preArmCheck, corpo separato dal compilatore | 1824 |
| planMeasuredEntry | 576 |
| Controller::armOperationalSession | 480 |
| MotionSession::request | 240 |
| MotionCoordinator::startStand | 1552 |
| AdaptiveStand::request / update | 1648 / 2288 |
| HttpTransport::handleMotion (task HTTP) | 6976 |

Percorso pre-arm: corpo `preArmCheck` + pairedProneSample/FK fino al leaf = 4144 B di frame noti. Aggiungendo loop 32, update 128, HTTP-update 48, serviceRequest 4576, MotionService command 400, coordinator accept 336 e wrapper preArm 32 si ottengono circa **9696 B**, prima dei wrapper runtime/indiretti e delle librerie. Non si aggiunge anche la catena arm/request: il pre-arm è già tornato quando inizia quella catena. Con stack loop 16384 B questa stima lascia margine, ma **solo il minimo libero runtime durante primo STAND/entry può validarlo**. Lo stack HTTP 24576 B appartiene a un altro task: handleMotion e parser hanno una catena diretta MATDOG circa 7808 B più HTTPD/librerie; non si sommano ai 9696 B del Controller.

Rischio aggiuntivo scoperto nell'analisi, già nella baseline: il percorso generico WALK `FeedbackWalk::generate → solveStlContactGait` usa frame elevati (`solveStlContactGait` 6272 B, `adapted` 1664 B). La somma strutturale della catena generica dal Controller può superare 16 KiB (circa 17632 B nei frame noti). Il percorso certificato può scegliere rami diversi, ma questo audit **non qualifica lo stack di tutte le modalità WALK**. Anche il gestore persistente USB ha un frame ampio, fuori dal test Motion qui autorizzato. Non si aumenta lo stack consumando arbitrariamente il poco heap disponibile. **Non estendere questa candidatura a WALK/FAST o operazioni persistenti concorrenti** senza un audit dedicato dei rami effettivi e high-water misurato; profili e controlli non vengono modificati in questa patch STAND.

Le nuove misure USB `runtime.loop_stack_min_free` e HTTP `transport.loop_stack_min_free` sono il minimo libero storico del task Controller, insieme a heap libero/minimo/largest. Gli header ESP-IDF della toolchain ESP32 riportano high-water in byte. Il firmware installato non ne ha registrato una serie USB, perciò non è corretto dichiarare un margine fisico già misurato. Registrare il dato dopo ammissione, PRIME, entry, STAND e STOP; se il margine cade in modo anomalo, fermare la prova e analizzare prima di ripetere.

## Riproduzione offline

Da `05_Firmware/MATDOG_Controller`, senza collegare dispositivi:

```sh
python3 scripts/tests/test_post_validation_clock.py
MATDOG_STAND_ONLY=1 python3 scripts/tests/run_operational_motion_tests.py
bash scripts/tests/run_telemetry_channels_tests.sh
python3 scripts/tests/test_motion_http_transport.py
node scripts/tests/test_motion_dashboard_dom.cjs
node scripts/tests/test_portal_dom.cjs
python3 scripts/static_audit.py
```

I test DOM richiedono `jsdom` (usato 26.1.0, installazione temporanea fuori dal repository tramite `NODE_PATH`). Il gate generale esegue anche la suite operativa completa, che può impiegare parecchi minuti. Nel viewer host: `npm ci` dal lockfile e `npm run verify`. Gli oggetti LFS canonici sono stati materializzati nel solo worktree isolato con `git lfs checkout`, usando gli oggetti locali; nessun `pull` o download di geometria.

Build senza upload, ciascuna con propria directory esterna nuova:

```sh
MATDOG_BUILD_ROOT=/percorso/assoluto/esterno/build-usb MATDOG_PROFILE=USB_ONLY scripts/build.sh --jobs 3
MATDOG_BUILD_ROOT=/percorso/assoluto/esterno/build-powered MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh --jobs 3
```

La directory esterna non deve intersecare alcun checkout Git. Usare i manifest per identificare ogni rebuild: la policy `UTC_NOW` rende normali SHA diversi fra compilazioni dello stesso sorgente. Non usare l'eventuale binario USB_ONLY come candidato di movimento e non avviare `upload.sh`, `flash_app_only.sh` o runner hardware durante queste regressioni.

Commit audit/test aggiuntivo: `b0b7e51`. Nessun file compilato firmware o viewer modificato rispetto a `652c2e2`; i binari sopra mantengono la loro identità verificata.
