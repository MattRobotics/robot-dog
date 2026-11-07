# MATDOG Controller 1.0.1-dev.17 — candidata di stabilizzazione operativa (07/11/2026)

> **Esito (07/11/2026, aggiunto dopo il collaudo; il testo sotto resta quello della candidata):** questa candidata `17db0ef` è stata flashata APP-only, avviata e collaudata sul robot, poi rilasciata senza ricompilazione come prima Stable GitHub `v1.0.1`. Evidenze e limiti: [Dev17_HW_2026-11-07/REPORT.md](../Dev17_HW_2026-11-07/REPORT.md) · [manifest firmware/release](../Dev17_HW_2026-11-07/RELEASE_MANIFEST.md).

> **Aggiornamento pre-flash (07/11/2026 notte):** candidata consolidata **`17db0ef`**, stessa versione `1.0.1-dev.17` (mai installata): comandi joystick ammessi solo in ordine di pagina (§2.6), STOP durante il caricamento del Twin misurato (§2.7), pagine di stato verificate solo-RAM (§2.8), `input_ms` dello STOP nella traccia. La build di `7503bfc` (APP `f49b531a…`) è **SUPERATA per il flash**: conservata con `NOT_FOR_FLASH.txt`, rifiutata dallo script di flash.

> **IMPLEMENTATA, NON VALIDATA SU HARDWARE.** In questo mandato nessun flash, riavvio o movimento; robot spento con KEY OFF del BMS. **dev.16** (`e0ddbb1`, APP `73952eec…`) resta il firmware installato e il riferimento validato su hardware il 07/11 ([dossier](../Dev16_HW_2026-11-07/REPORT.md)). Calibrazione gen-1, mapping dei 13 servo, geometria REV01-FIX/URDF, FK/IK, box di collisione, limiti, partizioni, NVS/EEPROM, STOP/SAFE_OFF/deadman/watchdog: invariati. Collaudo proposto: [FLASH_AND_TEST.md](FLASH_AND_TEST.md).

## 1. Identità

| Elemento | Valore |
|---|---|
| Base | sorgente dev.16 installato `e0ddbb1` (ref locale `refs/preserve/20261107/dev16-installed-e0ddbb1`) |
| Commit (branch `matdog/dev17-stabilization`) | `2205ee7` HTTP/trasporto → `2b00597` input/UI → `3288c48` Twin refresh → `d817b12` STOP→WALK → `1b822fc` cockpit e ViewCube → `7503bfc` versione e changelog → pre-flash: `86ac045` ordine joystick → `b1c9f85` test pagine di stato e STOP-Twin → `5e59cac` `input_ms` STOP → **`17db0efe23242ae8932b5549546a9758d9aa4645`** changelog |
| Versione / build | `1.0.1-dev.17` / `17db0efe2324` (stringhe verificate nel binario) |
| APP | `MATDOG_Controller.ino.bin`, **1.707.808 byte** (+13.760 rispetto a dev.16), SHA256 **`14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83`** |
| Manifest | `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `HARDWARE_PROFILE=ROBOT_POWERED`, `OTA_INGEST_ENABLED=0`, layout `MATDOG_16M_2x5M_NVS_V1`, slot APP 32,6 %, `PRIVATE_TLS_CONFIGURATION_SHA256=ABSENT` |
| Partizioni / bootloader / sdkconfig | tabella `8f756ecb…`, bootloader e `sdkconfig` **identici** a dev.16 (non vengono scritti) |
| Piano di scrittura | solo `app0` 0x010000–0x1b0f20 (erase fino a 0x1b1000); NVS MATDOG 0xfe0000–0xff0000 intatta (`matdog_layout.py check-write`) |
| Stack (frame ELF dev.16 → dev.17) | rispetto a dev.16, nel codice MATDOG: `SimpleMotion::startWalk` 1.616 → 1.728 B (+112 B, task loop; minimo libero misurato in dev.16 ≥ 4.208 B), `walkGait` fuori linea 32 B, nuovo `MotionService::admitJoystickOrder` 64 B (chiamato da `MotionService::command`, 192 B, sul task loop), `update::parseHex` (funzione esistente, percorsi portale/HTTP) ora fuori linea 80 B; massimo invariato 9.648 B (handler HTTPD) |
| Binario | solo locale ASUS: `~/MATDOG/build-output/dev17-stabilization-20261107/build-17db0ef/artifacts/ROBOT_POWERED-ota0/` (APP, ELF, map, manifest; sola lettura). **Superate, mai da flashare:** `build-7503bfc` (APP `f49b531a…`, accetta comandi joystick fuori ordine), `build-667c2a5` (layout intermedio), `o2-eval-7503bfc` (misura `-O2`), ognuna con `NOT_FOR_FLASH.txt` |

## 2. Cause e correzioni

### 2.1 HTTP — `/status` 503 e risposte lente (P0)

Catena analizzata: richiesta → ammissione slack → dispatch → generazione risposta → invio TCP → completamento → browser.

| Punto | Evidenza | Classificazione | Correzione |
|---|---|---|---|
| **Ammissione** (prima divergenza lato Controller) | `HttpTransport::update` rispondeva **subito 503 `MOTION_BUDGET_UNAVAILABLE`** a `/status` e `/wifi/status` quando il giro di loop cadeva negli ultimi 5 ms prima della release Motion (`motionSlack(now,5000)`), **anche senza overrun**. HW dev.16: `/status` in serie con "ripetuti HTTP 503 immediati" e banner intermittente. Riprodotto nel test di contesa (`test_http_fence_contention.py`, caso 13) | DIMOSTRATO (codice + test) | Le due pagine di stato entrano tra le letture esenti da slack come `/api/motion/status`: formattano solo fatti in RAM (nessun bus, NVS o radio). Tempo di servizio sul loop esportato (`status_service_last_us`/`_max_us`). I **comandi** richiedono ancora 5 ms di slack |
| **Dispatch / generazione** | Tracce 23–32: `late` 0 e `busy` 0 in tutte le finestre; `dispatch_max_ms` ≤ 125 ms (joystick, traccia 27); lettura Motion servita sul loop in 312–674 µs (un picco 8.183 µs, traccia 32) | VERIFICATO HW dev.16 | nessuna |
| **TCP / browser** (prima divergenza lato pagina) | Il browser abortiva ogni lettura a **800 ms**; lwIP del Controller recupera un segmento perso solo per **RTO** (iniziale 3 s, timer 250 ms). Finestre con coppia ON: **217 `TIMEOUT_800MS` su 2.377 letture**, fino a 44 per finestra (1 su ~7 nel HOLD prolungato, traccia 26); IDLE coppia OFF (traccia 24): 0 su 117. Letture riuscite: mediana 38–78 ms, p95 109–217 ms, massimo 344 ms. `poll_skipped` 21–1.359 per finestra (corsia unica condivisa) | DIMOSTRATO (dati + configurazione lwIP) | Lettura abortita solo a **3,5 s**. Polling Motion su **corsia propria**: seconda lettura solo se quella in volo ha più di 700 ms; **al massimo 2 letture in volo per tutta la pagina** (il refresh diagnostico del portale viene rinviato invece di aprire una terza); vince la risposta più recente, quelle tardive sono contate (`late_dropped`) e scartate. Con un comando, WALK STOP e STOP restano ≤ 5 delle 6 connessioni per host: STOP non attende mai una connessione libera |
| **Perdita di segmenti solo con coppia ON** | Correlazione forte ma non assoluta (traccia 23: 183 campioni con coppia ON e 0 errori). Nessun codice firmware cambia la radio con Motion | **UNKNOWN** | Misura mancante: cattura pacchetti lato client (tcpdump/Wireshark) in HOLD vs IDLE, con RSSI/canale e corrente BMS |
| **200 lente/incomplete** su `/status` | 1.409/1.899 byte al timeout di 2 s del client terminale | coerente con perdita + RTO; non attribuito oltre | idem |

### 2.2 Joystick e messaggi (P0)

| | dev.16 | dev.17 |
|---|---|---|
| Banner `#message` | nel flusso sopra la card Motion | overlay sotto l'header sticky: non sposta nulla, non copre STOP; tocco = chiusura |
| Testi dinamici sopra il joystick (link, stato, motivo, ultimo comando, avviso Twin) | lunghezza variabile, `hidden` alternato | slot fissi (1–2 righe, ellissi), testo completo nel `title` e in un **log diagnostico** degli ultimi 20 esiti (rifiuti consultabili) |
| Centro del pad | riletto a ogni `pointermove` dalla posizione corrente | **ancorato al `pointerdown`**: un reflow sposta il pad, mai la velocità richiesta |
| Rilascio | `pointerup`/`pointercancel`/`lostpointercapture` sul pad, `pointerleave` sui pulsanti di rotazione, `blur`, pagina nascosta | in più `pointerup`/`pointercancel` ovunque nella finestra (cattura mancante); long press senza menu contestuale; altezza Twin applicata solo al rilascio |
| Chrome 156, pagina spedita, fixture reali, pad tenuto premuto | spostamento pad **89–202 px** (banner, testo budget, motivo, altezza Twin); con 0,3 px di tremolio del dito la richiesta cambia del **75,8 %** | **0 px** in tutti e 4 i disturbi, desktop mouse 1280×900 e telefono touch 390×844; richiesta ≤ 1,12 % (solo tremolio); dopo il sollevamento solo `JOYSTICK rilascio`; STOP visibile e non coperto |

Deadman invariato: refresh a 240 ms, scadenza Controller 1 s, WALK STOP automatico. L'ordine dei comandi joystick, prima solo un rischio ipotizzato, è stato **dimostrato e corretto** prima del flash (§2.6).

### 2.3 Twin dopo refresh con coppia ON (P0)

Motivazione verificata di `twinBusy`: copia lato pagina di una regola già del server. `handleTwin` rinvia solo i file grandi (`twin.js` e mesh, > 8 KB gzip) mentre Motion è ingaggiato e risponde sempre **304** a un ETag valido; pagina (1.536 B) e URDF (2.598 B) sono sempre serviti. Costo reale misurato in Chrome con i byte di `TwinAssets.h` e la semantica di `handleTwin` (viewport telefono, throttling CPU 1×/4×/6×):

| Scenario | Richieste Twin | Long task max (main thread condiviso con STOP) |
|---|---|---|
| dev.16, refresh in HOLD con coppia ON | nessuna (Twin assente fino a torque OFF) | — |
| dev.17, refresh in HOLD, cache calda | 4 × **304** (0 byte di corpo) | **213 / 290 / 397 ms** una sola volta |
| dev.17, refresh in HOLD, cache fredda | pagina 200 + `twin.js` **503** (77 B), poi nulla | 115–166 ms (solo pagina) |
| dev.16, caricamento a torque OFF (cache fredda) | 4 × 200 (235 KB gzip) | 200 / 372 / 505 ms |

Soluzione: il Twin parte **a robot fermo anche con coppia ON**; mai durante MOVING/WALKING o con il joystick premuto (nessun blocco del main thread sotto un robot in moto o un dito); un caricamento incompleto (cache fredda con coppia ON, o trasferimento interrotto all'ingaggio di Motion) viene ritentato **una volta** a torque OFF. Stessa telemetria (`postMessage` del poll esistente, nessuna lettura o accesso servo aggiuntivo); LIVE resta la regola di freschezza del viewer; Reset vista, IMU, FK/IK invariati.

### 2.4 STOP → WALK (P1)

| | Prima (dev.16) | Dopo (dev.17) |
|---|---|---|
| WALK con STOP completato e coppia ON | rifiutato `STOP_LATCHED` (MotionService e `walk()`); servivano STAND (es. 150 mm), attesa HOLD, poi WALK che riscende a ~120 mm | un **WALK STANDARD** esatto viene ammesso **come STAND dopo STOP** (`standResumeRefusal`: coppia 13/13, feedback fresco, IMU, potenza/autorità, percorso cinematico dall'origine dello STOP) e segue la **rotta di ripresa esistente** di `stand()` fino alla quota WALK, poi avvia WALK STANDARD |
| Origine WALK | — | completa il proprio WALK STOP con la legge del passo (stesso passo e inviluppo d'inclinazione), HOLD a 120 mm, poi il passo richiesto (`requestedStride_` applicato da `startWalk`) |
| Origine HOLD/HEIGHT | — | HEIGHT dal RAW tenuto alla quota WALK, poi WALK |
| Origine discesa PRONE | — | rotta certificata della regione (come STAND) |
| Rifiuti | — | STOP in corso `STOP_IN_PROGRESS`; dopo SAFE_OFF `SAFE_OFF_STAND_REFUSED`; fault `FAULT_STOP_REQUIRED`; ammissione negata = stesso motivo di STAND; WALK STOP, joystick e PRONE restano `STOP_LATCHED` |

Nessuno stato, task o macchina a stati nuovi; nessun ACQUIRE/ENABLE né SAFE_OFF; niente riparte senza il nuovo comando WALK; `beginStop()` continua a revocare ogni intento di passo. La Dashboard abilita WALK con lo stesso booleano del Controller usato per STAND (`stop.stand_resume_allowed`).

### 2.5 Motion Cockpit e ViewCube (P2)

- **Cockpit** (richiesta dell'operatore nella sessione dev.16): Twin in alto a tutta larghezza; sotto un unico pannello con Postura (quota STAND 50–150 mm a **passi di 5 mm**, slider + stepper ±5 mm, STAND, PRONE), WALK STANDARD (**🐢 30 / 45 / 🐇 60 mm per ciclo**: passo, non velocità; i tre preset `kStrideM` esistono e sono testati) con WALK e WALK STOP, joystick compatto. Badge "VERIFICATO SU HARDWARE (dev.15)" e testi didattici spostati nella diagnostica chiusa. Cambiare un valore non invia nulla; rifiuti solo dal Controller. STOP nell'header sticky su ogni pagina. Desktop 1280×900 e 1024×768 a tre colonne, telefono 390×844 a una colonna.
- **ViewCube CAD** (`viewer/src/viewer/viewCube.ts`): cubo con etichette italiane e triade XYZ nei colori degli assi, disegnato nell'angolo dello stesso renderer (scissor, camera ortografica che copia l'orientamento). Click/tap su una faccia → vista lungo quell'asse con stesso target e distanza (FRONTE = +X MATDOG; ALTO/BASSO in convenzione CAD, fronte in basso/in alto, sinistra/destra come la vista frontale); trascinamento = rotazione OrbitControls; Reset vista invariato. **Solo camera**: modello, quaternione IMU, URDF, telemetria e richieste invariati; solo three.js core.
- Bundle `twin.js` ricostruito in modo riproducibile (node 22.22.2; il sorgente invariato era stato prima ricompilato **byte-identico** al `twin.js` spedito `084a197d…`): 582.190 B raw, SHA256 `f26d84f3…`, 148.390 B gzip (il messaggio del commit `1b822fc` riporta 582.162 B raw, misura di un bundle intermedio; SHA e gzip là sono corretti); pagina, mesh e URDF **byte-identici**; totale 240.875 B gzip (budget < 300.000).

### 2.6 Pre-flash — comandi joystick fuori ordine (P0)

**Difetto dimostrato** (offline, percorso di produzione: corpo JSON come quello della pagina → parser → gate HTTP con sessione/CSRF → `MotionService` → `SimpleMotion`), sul candidato precedente `7503bfc`: robot in HOLD alla quota WALK, il movimento seq 10 consegnato **dopo** il rilascio seq 11 ma entro il suo TTL di 1 s viene **ACCETTATO**; il robot lascia la HOLD dopo 20 ms e **cammina** fino al deadman (`JOYSTICK_TIMEOUT`). Accettati anche: il movimento seq 12 riconsegnato dopo il rilascio seq 13, un movimento più vecchio seq 9, una richiesta tardiva della pagina precedente dopo un reload; un refresh vecchio estendeva il deadman da 1,0 a 1,62 s. `MotionService` controllava solo il TTL e il replay esatto dell'ultimo id; HTTP può riordinare (una connessione per richiesta, ritrasmissione TCP).

**Correzione minima** (`MotionService::admitJoystickOrder`, solo RAM, solo owner `dashboard` e comandi `JOYSTICK`, dopo TTL, STOP, latch STOP, owner e replay esatto, prima di `drive()`): l'id della pagina è `simple-<sessione pagina>-<sequence>`. Nella stessa sessione un movimento o un neutro è ammesso solo sopra la sequence più recente ammessa: duplicati, superati e ritardatari → `JOYSTICK_SUPERSEDED`, quindi non raggiungono `drive()` né prolungano il deadman. Una sessione diversa (reload) è ammessa solo se il comando non è stato emesso prima del più recente (`issued_at_ms`, lo stesso tempo del Controller usato dal TTL): la nuova pagina riparte dalla propria sequence 1, una richiesta tardiva della pagina precedente è rifiutata. Id senza la propria sequence → `JOYSTICK_SEQUENCE_REQUIRED`. Invariati: STOP (gestito prima, con qualsiasi sequence e timestamp), WALK STANDARD, WALK STOP, STAND, PRONE, HostLink (owner `local`), TTL, deadman, limiti. Nessun nuovo stato macchina, task o persistenza.

Test (PASS su `17db0ef`, FAIL sugli stessi punti con `7503bfc`): movimento 10 → neutro 11 → movimento 10 tardivo rifiutato e robot fermo in HOLD; movimento 12 → neutro 13 → 12 riconsegnato rifiutato; ritrasmissione esatta dell'ultimo comando servita dal replay; movimento più vecchio rifiutato; nuova pressione dopo il rilascio: cammina (joystick non bloccato); reload a sequence 1 accettato e richiesta tardiva della pagina precedente rifiutata; TTL scaduto; WALK STOP ritardato che ferma comunque; STOP con sequence vecchia e timestamp scaduto accettato; joystick rifiutato a STOP attivo; formato dell'id; deadman a 1,0 s dall'ultimo refresh accettato mentre un refresh più vecchio è rifiutato. DOM: ogni id della pagina contiene sessione e sequence.

Limite: tra pagine diverse l'ordine usa `issued_at_ms`, stima del clock del Controller fatta dalla pagina (errore < 1 s, limitato dal TTL): un comando di una nuova pagina emesso entro quell'errore dall'ultimo della pagina precedente può essere rifiutato una volta (direzione sicura); il refresh successivo (240 ms) è ammesso.

### 2.7 Pre-flash — STOP durante il caricamento del Twin

Chrome 156, pagina spedita, byte del Twin di `TwinAssets.h` con la semantica di `handleTwin`, robot in HOLD con coppia ON (fixture reale), reload; STOP premuto (click) ed Escape 4 volte durante il caricamento e 2 volte a Twin inattivo; due esecuzioni registrate (più due precedenti coerenti).

| Condizione | Input → richiesta STOP al server: mediana / massimo |
|---|---|
| Cache calda, CPU 1× | 8 / 142 ms |
| Cache calda, CPU 4× | 18 / 371 ms |
| Cache calda, CPU 6× | 29 / 509 ms |
| Cache fredda (il Controller rinvia `twin.js`), CPU 6× | 15 / 36 ms |
| Twin inattivo, CPU 1×–6× | ≤ 28 ms |

Ogni input invia **esattamente uno STOP** (nessuno perso o duplicato), nessun comando joystick involontario, nessuna richiesta oltre letture della pagina, file del Twin e STOP (quindi nessun accesso nuovo al bus servo). Il ritardo è tutto nella pagina (input → invio): il Twin same-origin condivide il main thread e l'input che cade nel task lungo di compilazione attende. Nessuna modifica semplice lo elimina senza worker o isolamento dell'iframe, esclusi dal mandato; il recupero del Twin in HOLD con coppia ON resta. **Non è uno STOP istantaneo:** ogni STOP registra ora `input_ms` nella traccia, e il collaudo misura il ritardo sul telefono ([FLASH_AND_TEST.md](FLASH_AND_TEST.md) B3).

### 2.8 Pre-flash — `/status` e `/wifi/status` senza slack

Verifica del codice: sul loop i due handler chiamano 22 accessor di `ControllerService`, tutti letture di stato già in RAM o classificatori puri (BMS, IMU, LED, servo, calibrazione, OTA, Wi-Fi in cache, snapshot Motion, `readiness()` su flag `constexpr`), più contatori heap e formattazione: nessuna lettura del bus servo, nessuna API NVS, nessuna chiamata al driver Wi-Fi, nessuna attesa o lock. Nuova guardia ancorata al sorgente (`test_slack_exempt_session.py`, `STATUS_PAGES_RAM_ONLY`) che fallisce se compare una di queste chiamate o un accessor non rivisto. STOP: corsia dedicata invariata e prioritaria (contesa HTTP 723 controlli); mailbox a slot unico invariata; connessioni: ≤ 2 letture per pagina, server a 7 socket. Tempo di servizio esportato (`transport.status_service_last_us`/`_max_us`) per il collaudo; la lettura Motion equivalente costava 312–674 µs su dev.16.

## 3. Regressioni eseguite (offline)

| Suite | Esito |
|---|---|
| `test_http_fence_contention.py` (723 controlli: stato senza slack servito, comandi in attesa, STOP), `test_slack_exempt_session.py`, `test_ap_only_transport.py` (134), `test_motion_http_transport.py` (97) | PASS |
| `run_simple_dashboard_tests.sh`: fixture reali 37, DOM 67 comandi (corsie di lettura, sovrapposizione, risposta tardiva, tetto 2 letture, STOP accanto a due letture trattenute, abort 3,5 s; reflow durante pressione, banner, rilascio fuori pad, `pointercancel`, `blur`, pagina nascosta, rotazione `pointerleave`, errore di rete; Twin dopo reload; cockpit; WALK dopo STOP), Twin feed 52, budget portale 28, contratto 269 (incluso STOP→WALK via HTTP con la busta WALK reale) | PASS |
| `test_dashboard_layout_browser.cjs` (Chrome 156, desktop mouse + telefono touch) e `test_twin_viewcube_browser.cjs` (Twin reale da `TwinAssets.h`: cubo disegnato, click/tap faccia, trascinamento, Reset vista identico alla vista iniziale, nessuna richiesta nuova) | PASS; la prima **FALLISCE sulla pagina dev.16** (difetto riprodotto) |
| `run_simple_motion_tests.sh` (incl. `stopThenWalk`: STOP in WALK45 in tre fasi del passo → WALK60, STOP in HOLD, STOP in HEIGHT, rifiuti; suite JOYSTICK) | PASS |
| Pre-flash: fixture/contratto con ordine joystick (fixture 71 controlli, contratto 269), `test_slack_exempt_session.py` con `STATUS_PAGES_RAM_ONLY`, `test_stop_twin_load_browser.cjs` (8 scenari, `input_ms` della traccia coerente 8/8), ripetute layout e ViewCube | PASS (su `17db0ef`) |
| `run_stop_support_tests.sh`, `run_prone_tests.sh` (replay hardware + replay dev.15 4/4 `PRONE_REST`) | PASS |
| viewer `vitest` (105, di cui 6 ViewCube), `tsc --noEmit`, `test_twin_assets.py` (42) | PASS |
| `static_audit.py` completo, `test_dev_identity_build.py` | PASS (§3.1) |
| `test_portal_dom.cjs`, `test_motion_dashboard_dom.cjs`, `test_motion_transport_mode.py` (legacy, fuori dalle suite correnti) | falliscono **alla stessa asserzione sulla baseline dev.16**: preesistenti, non riscritti |

Limiti: nessuna prova su ESP32-S3, radio, servo o browser del telefono reale; il layout è verificato in Chrome headless (WebGL software) con viewport emulati; i test DOM modellano le richieste, non i socket.

### 3.1 Gate offline completo

`static_audit.py` completo sul commit consolidato `17db0ef` (dipendenze offline pinned su `PYTHONPATH`): **`STATIC_AUDIT = PASS`**, 363 sorgenti; `HOST_TESTS = PASS`, `MOTION_CONVERGENCE_SOURCE = PASS`, `MOTION_HTTP_TRANSPORT = PASS` (97 controlli); suite di esecuzione Motion `DEFERRED` come nella selezione dev.16 (286 artefatti); le righe `FAIL` nel log sono i mutanti attesi di `USB_FRAMING_MUTATIONS = PASS` (blocco identico all'esecuzione su `7503bfc`). `test_dev_identity_build.py`: 11/11 OK su `17db0ef`. Log locale: `~/MATDOG/build-output/dev17-stabilization-20261107/static_audit-17db0ef.log` (quello di `7503bfc`, anch'esso PASS, è conservato accanto).

## 4. Timing

- Nessuna modifica al calcolo Motion (gait, IK, compensazione, T1) in dev.17. Effetti offline attesi sul loop: le pagine di stato ora servite costano formattazione in RAM (misurata su HW dalla nuova coppia `status_service_*`; la lettura Motion equivalente costava 312–674 µs); i comandi restano dietro lo slack di 5 ms.
- Variabilità HOLD (dev.15: ~7,4→33,1 ms per lo stesso lavoro): **UNKNOWN**, nessuna causa assegnata.
- **`-O2` selettivo — valutato offline, NON incluso** (non autorizzato automaticamente, nessuna misura ESP32-S3 possibile qui): build A/B di `7503bfc` con `#pragma GCC optimize("O2")` su 12 unità Motion (`SimpleMotion.cpp` e `motion/` IK, FK, compensazione, gait), DIRTY, **mai da flashare** (`o2-eval-7503bfc/NOT_FOR_FLASH.txt`). APP 1.707.120 → **1.722.144 B (+15.024, +0,9 %)**; codice dei simboli Motion 101.091 → **119.648 B (+18 %)**; 97 funzioni Motion con frame diverso, per lo più +16…+32 B (`generateWalk` 1.760→1.792, `SimpleMotion::update` 1.312→1.328), massimo invariato 9.648 B. Confronto numerico su Xtensa e costo per release: **non misurabili offline** (nessun simulatore/target); il codice Motion più grande aumenta la pressione sulla I-cache da 16 KB condivisa, già sospettata nella variabilità HOLD. Nessun beneficio dichiarabile: serve un A/B supervisionato su ESP32-S3 (stessi scenari HOLD/WALK45/joystick, confronto `max_work`, overrun, persi, `loop_stack_min_free`), solo con autorizzazione esplicita.

## 5. Difetti residui che bloccano la stable

1. dev.17 non è validata su hardware: tutto quanto sopra è IMPLEMENTED NOT VALIDATED HARDWARE fino al collaudo di [FLASH_AND_TEST.md](FLASH_AND_TEST.md).
2. Perdita di segmenti TCP con coppia ON: causa UNKNOWN (misura: cattura pacchetti HOLD vs IDLE). dev.17 ne riduce gli effetti, non la causa.
3. Timing di locomozione: overrun residui in WALK e joystick (dev.16: WALK45 +127 overrun, joystick yaw +54 persi/+504 overrun per finestra); T1 non garantisce 20 ms ovunque; variabilità HOLD UNKNOWN.
4. STOP dalla Dashboard durante la compilazione del Twin dopo un reload in HOLD: ritardo misurato in Chrome fino a ~0,5 s (CPU 6×) per l'input che cade nel task lungo; da misurare sul telefono (B3, `input_ms`). Ordine joystick tra pagine diverse basato su `issued_at_ms` (§2.6, limite in direzione sicura).
5. Rotazione grafica del Twin durante l'assestamento IMU in STAND (osservata in dev.16): origine non dimostrata, non toccata.
6. Gait lenta (obiettivo fino a 5× non qualificato), picchi termici isolati ID21/31, LED intermittenti, RESTORE manuale (#48): invariati.
