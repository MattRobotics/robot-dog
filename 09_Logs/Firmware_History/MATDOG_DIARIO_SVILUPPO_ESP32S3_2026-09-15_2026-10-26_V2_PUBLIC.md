# MATDOG — Diario storico dello sviluppo firmware ESP32-S3
## 15 settembre – 26 ottobre 2026 · V2 · edizione pubblica

> **Edizione pubblicabile derivata dalla V2 originale (27 ottobre 2026).** Sono stati anonimizzati esclusivamente i riferimenti ai percorsi del computer locale, sostituiti con identificativi `SRC-####` e descrizioni logiche delle evidenze. Non sono stati ricalcolati SHA, modificati esiti dei test, o estese le qualifiche hardware. L'originale V2 resta la fonte congelata, SHA256: `787308caa0a8fdb921dfbc130a7d38f28f60a06874e600dc261779357c69a5e6`. Le fonti `SRC-####` rinviano all'indice §7, non a file necessariamente presenti su GitHub. Identità della APP verificata tramite SHA256 nei §2–3; un commit sorgente, una build e un flash sono eventi distinti.

**Edizione:** 26 ottobre 2026. **Perimetro temporale:** eventi disponibili fino alla candidata RC5 costruita alle `2026-10-26T08:19:48Z`; non è un’attestazione dello stato live del robot. Le ore dei log senza offset restano nel loro dominio originale: non sono convertite implicitamente in UTC. Il 25 ottobre termina l’ora legale a Roma; per questo le 21:19:44 locali del 25/10 corrispondono alle 20:19:44Z. Data build, flash, test e redazione sono campi distinti.

Ricostruzione documentale in sola lettura su un archivio locale di progetto. Nessuna compilazione, suite firmware, accesso al robot, interrogazione seriale/HTTP, flash o operazione Git è stata eseguita. Il documento originale fu prodotto senza modificare i worktree. V1 conservata intatta. Gli audit già conclusi sono riusati; le lacune ancora aperte sono dichiarate, senza promettere completezza di un archivio che ha perso alcune catture.

**Conclusione verificabile:** ultima APP con flash e prove fisiche reperiti `54ade4fd…` (RC5); ultima candidata `fe33fab6…`, 1.615.824 byte, qualificata soltanto offline e non flashata nei report consultati. STAND fisicamente osservato sulla prima, ma completamento misurato, disponibilità HTTP e STOP end-to-end durante blackout non qualificati. Nessuna prova reperita di WALK continuo riuscito.

### Come leggere l’evidenza

- **HW-LOG:** flash/readback o comportamento fisico sostenuto da log locali; vale solo per quella APP e configurazione.
- **HW-REPORT:** resoconto contemporaneo o osservazione esplicita dell’operatore; non equivale a cattura RAW oggi disponibile.
- **OFFLINE:** build, calcoli, replay, test host o revisione; tempi virtuali/x86 non sono tempi ESP32-S3.
- **IMPLEMENTED:** implementazione presente, esito pertinente ancora non qualificato.
- **HYPOTHESIS:** spiegazione non discriminata dai dati fisici.
- **UNKNOWN:** dato non reperito o misura mai acquisita. “Flash non reperito” non significa dimostrazione che non sia mai avvenuto.

**Riuso:** KEEP = risultato/componente conservabile nel proprio dominio; ADAPT = concetto utile da adattare; SUPERSEDED = implementazione sostituita; DIAGNOSTIC = restrizione/strumentazione temporanea. Un guasto pre-attuazione non è classificato come difetto cinematico. PASS HTTP, ACCEPTED, ARMED, TX-complete e SAFE_OFF verificato sono eventi diversi.

## 1. Correzioni sostanziali rispetto alla V1

1. Inserite le installazioni G3 del 17/09, G3.1 del 18/09 e DALY del 19/09; la prima integrazione ROBOT_POWERED precede il 26/09. Recuperato il problema CDC che riduceva IMU a 0,68 Hz a porta chiusa.
2. Recuperate freeze NEXTGEN del 25/09, integrazione `c45858c` del 26/09 e prima calibrazione 24/24 del **1° ottobre**, RAM-only. DEV3 del 6/10 non è la prima calibrazione completa: aggiunge il risultato persistito gen1.
3. Inserite migrazione/layout M0 e prove fallite/bloccate, WPA2 del 10/10, patch intermedie del 17–19/10 (`a34d528`, `b5be64c4`), V5 effettivamente provata `9eeef0d`, SIMPLE RC1/RC2 e diagnostica timing del 25/10.
4. `991288…` è **RC4-r4**, non la RC4 originale; la prima RC4 archiviata è `ac31bf0d…`. `62fc214…` risulta realmente flashata il 24/10.
5. Jacobiano `dbc64f…`: dopo due preflight interrotti, esiste **flash e verify indipendente riuscito** il 25/10; non resta una semplice candidata offline. I fault successivi non dimostrano un difetto del Jacobiano.
6. RC5 intermedia `35c659…`, poi `54ade4…`; Network Recovery `bac0c9…`, poi finale `fe33fab…`. La finale contiene anche cancellazione/TTL/clock HTTP. La RCA rete non dimostra la causa completa del blackout.
7. Il log d84 registra `tx_sequence=218` **cumulativo**: non è dimostrato che tutti i 218 frame appartengano a un solo tentativo. Il nesso clock→accelerazione fisica resta ipotesi, benché il difetto software sia riprodotto.
8. Frequenza: SIMPLE V1 nasce descritta a 25 Hz; le RC4/RC5 recenti usano release di **20 ms, 50 Hz nominali**. Non si trasferisce il 25 Hz a tutta la famiglia né si presenta il nominale come cadenza fisicamente garantita.

## 2. Cronologia verificata

Per ogni scheda: identità/sorgente, problema e modifica, prove/esito, limiti e decisione. Il registro §3 integra SHA completi, build UTC, CLEAN/DIRTY, dimensioni e manifest; fa parte delle schede. Se una scheda rinvia a un report senza inventario dirty completo, tale lacuna resta UNKNOWN e non viene colmata dal checkout odierno.

### 15/09 — Controller V0.1.0, prima baseline USB_ONLY

**Identità / installazione.** Firmware `5b371da5482f9b0bd2df1c37ed361250ea54ae8f`; APP `6e6d92f898dbe95000b53dbb252c7eb5d3deaa9a4b161e2b1934436a76b29364`, 384.400 B. Flash APP-only documentato nella sessione 2.3; il merge/tag non identifica da solo il binario.

**Architettura, problema e modifica.** Integrazione Controller, BNO085, classificazione profili e modalità MAINTENANCE/RUN. Correzione timeout SAFE_OFF distinto dal diagnostico e parser OTA fail-closed per flag UNKNOWN/slot ambigui.

**Prove ed esito effettivo.** HW-REPORT: boot USB_ONLY, IMU live, runtime_resets=0, soak 60 s; digest APP e confronto bootloader/tabella/otadata PASS. SAFE_OFF su bus non alimentato risponde UNVERIFIED_NO_RESPONSE: i ~205–210 ms non sono una latenza di spegnimento servo acceso.

**Funzionante / criticità residue.** Acquisizione IMU e classificazione periferiche assenti funzionano. DALY/servo/LED powered non ancora qualificati insieme. L’upload standard precedente citato dalla V1 non è la scrittura APP-only di questa scheda; data e immagine di quell’upload restano UNKNOWN.

**Decisione e riuso.** KEEP profili, provenance e IMU; upgrade G2/G3 per hardware alimentato.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001).

### 16/09 — G2, preparazione ROBOT_POWERED e hardening provenienza

**Identità / installazione.** G2 e revisioni pre-G3: identità APP specifica e diff completi UNKNOWN nella presente ricostruzione; nessun flash in queste fasi.

**Architettura, problema e modifica.** Provenienza lega profilo/FQBN alla APP; READY powered deve dipendere da una prova servo reale, non da periferica mai interrogata.

**Prove ed esito effettivo.** OFFLINE: build e suite G2 PASS nei record datati. Hardware G3 ancora TO_TEST.

**Funzionante / criticità residue.** Contratti software utilizzabili; nessuna nuova alimentazione o movimento dimostrato.

**Decisione e riuso.** KEEP hardening; G3 il 17/09.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001).

### 17/09 — G3, prima installazione ROBOT_POWERED senza moto

**Identità / installazione.** Build `6065d86c2da4`, CLEAN; APP `0454345808a6aac46ec79fc711f48904c3a80008738b485918060801e4a99442`, 387.776 B; commit completo UNKNOWN nel record letto. Flash APP-only/digest PASS.

**Architettura, problema e modifica.** Controller unico con batteria, DALY, servo rail e LED reali; readiness da census esplicito.

**Prove ed esito effettivo.** HW-REPORT: DALY ONLINE, LED chase e ritorno OFF, census 13 attesi, SAFE_OFF/readback 13/13, soak 60 s. Nessun GoalPosition/Torque ON. Secondo census inizialmente mancante.

**Funzionante / criticità residue.** Periferiche powered funzionanti; validità IMU limitata alla porta CDC aperta. KEY non discrimina MOS nelle due posizioni. Prova di ripetibilità census chiusa il 18/09.

**Decisione e riuso.** KEEP evidenze bus/periferiche; SUPERSEDED build da G3.1 per dipendenza CDC.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001).

### 18/09 — G3.1, loop indipendente dalla porta USB

**Identità / installazione.** `e2fc60531b28351472a2bfa5105a2170147610bb`, CLEAN; APP `e2b474b5c07e98649fbaf31d3d08970d022ffbdb5f1212d6b606dc28cac065d2`, 387.808 B, flash APP-only/digest PASS.

**Architettura, problema e modifica.** Prima divergenza A/B stesso boot: RV 0,68 Hz dopo chiusura porta, ~50 Hz aperta. HWCDC conservava connected e attendeva TX con ring pieno. Patch buffer 3072 B e TX timeout 0 prima di Serial.begin.

**Prove ed esito effettivo.** HW-REPORT: porta chiusa 62,121 s, RV 50,10 Hz; riaperta 50,13 Hz, reset=0. Secondo census coerente e G3 formalmente chiuso.

**Funzionante / criticità residue.** Indipendenza CDC verificata per questa configurazione; formattazione/CPU non diventano costo nullo. Nessuna traiettoria Motion provata.

**Decisione e riuso.** KEEP timeout TX zero e metodo A/B; upgrade DALY KEY.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001).

### 19–24/09 — DALY KEY, lettura/configurazione e correzione elettrica

**Identità / installazione.** 19/09 read-only: `a57fcddd29ac9bc7574520c86af57e183f43ae89`, CLEAN, APP `7c0d5d35524177045ffc17ec890b131860b9ccf15c3d574ea9c500a4ebc4c6ba`, 389.888 B. Scrittura: sorgente `6322563` (completo UNKNOWN), APP `e9283ced5801d87d5fe44f95411ead2de6d6f6dad2101c88b210c31cd0e645b6`; entrambe installazioni APP-only riportate.

**Architettura, problema e modifica.** FC03 conferma KEY disabilitata 0x0055; una sola FC06 imposta 0x005A DISCHARGE, ACK e readback. Rail ancora attiva a KEY OFF: ritorno buck su B− bypassava MOS via masse; corretto VIN− su P−.

**Prove ed esito effettivo.** HW-REPORT: scrittura 19/09 17:36:36.975, telemetria riprende, nessun reset. 24/09 closeout alimentazione/USB servizio senza VBUS; KEY OFF senza caricatore porta rail a 0 V. Raw del 19/09 persi in scratch, dichiarato il 20/09; sopravvivono trascrizioni committate.

**Funzionante / criticità residue.** KEEP logica KEY/readback e cablaggio corretto; persistenza del registro dopo vero power cycle BMS UNKNOWN. Caricatore B+/P− può alimentare bus con KEY OFF: nessuna equivalenza con isolamento completo.

**Decisione e riuso.** KEEP risultati elettrici nel dominio misurato; non ripetere scrittura per ricreare log perduti. Upgrade integrazione NEXTGEN.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001); [2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md · SRC-0002](#src-0002).

### 25/09 — NEXTGEN freeze V1/V2/V3, fallimenti e revisioni offline

**Identità / installazione.** V1 `c8906378df04` (completo UNKNOWN), APP `94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0`, 982.432 B. V2 `e3dd4e36e327c69d2a115c31a9831743f55f5359`; V3 precede `c45858c`. Identità complete delle APP V2/V3 pre-flash UNKNOWN qui.

**Architettura, problema e modifica.** I2–I6 integra LED, authority, calibration engine e HostLink; iniziale I7/I8 differita. V2 realizza HttpTransport/OtaSession/HMAC, corregge correlazione risposte mailbox, leak semafori start/stop e challenge DoS. Freeze e recovery vengono rivisti prima del primo flash.

**Prove ed esito effettivo.** OFFLINE: V2 17 suite/5804 check, audit 100 file PASS; nessuna prova powered attribuibile ai freeze. Gli stati “deferred” di V1 restano tali fino alla successiva implementazione.

**Funzionante / criticità residue.** Contratti e gate riutilizzabili; nomi freeze/final non provano installazione. SHA può cambiare con il solo BUILD_ID pur con logica firmware identica.

**Decisione e riuso.** SUPERSEDED i freeze; KEEP decisioni mailbox/lifecycle/provenienza. Installazione distinta del 26/09.

**Evidenze:** [2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md · SRC-0003](#src-0003); [2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md · SRC-0004](#src-0004); [2026-09-25_RECOVERY_BACKUP_AND_CANDIDATE_V3.md · SRC-0005](#src-0005).

### 26/09 — NEXTGEN V3 realmente installata

**Identità / installazione.** `c45858c532e97a5a104fb8330ad9cd909b75663e`; APP `13a1e9504e950d3d4bbc14ed78032019c8bf8bacb75804ca57555069787d5b71`, 1.027.472 B, ROBOT_POWERED/OTA ingest 1. Flash e digest PASS.

**Architettura, problema e modifica.** Prima installazione I7/I8; build diversa da ad08db6 autorizzata inizialmente per commit documentale incorporato nel BUILD_ID. Pre-write rifiuto NO_EXPECTED_HASH per nome manifest backup, nessuna scrittura prima della correzione del parametro.

**Prove ed esito effettivo.** HW-REPORT: IMU/DALY/LED, servo preflight 12/12 OFF, ciclo server START/STOP/START su USB. STA associa/DHCP 192.168.1.136, RSSI −89…−93 dBm; ARP FAILED, curl non raggiunge /status. OTA challenge/auth non esercitati.

**Funzionante / criticità residue.** Periferiche e lifecycle funzionano; associazione Wi-Fi non prova traffico IP. WIFI_TICK max 45.612 µs storico; steady last 3–19 µs, non costo periodico 45 ms dimostrato. Classificazione RF debole del report, causalità completa non misurata.

**Decisione e riuso.** KEEP integrazione; SUPERSEDED APP da LED V2. Motion rimane BLOCKED.

**Evidenze:** [2026-09-26_HARDWARE_VALIDATION_CANDIDATE_V3.md · SRC-0006](#src-0006).

### 26/09 — LED V2 focused

**Identità / installazione.** `88062e1a1217f288ebcc161c6213dbb543ea6f8a`; APP `772a046e1b2d61888dba0ab7ddc759be13bbd4a33d1722b6be5285c313aac56b`, 1.029.232 B; APP-only PASS.

**Architettura, problema e modifica.** SOC renderer/diagnostica; warning/critical riservati senza inventare policy batteria.

**Prove ed esito effettivo.** HW-REPORT: BOOTING breathing, preflight 12/12, READY, SOC 75,5/75,2% → nove verdi; test 0→12→0 e ritorno automatico al SOC reale.

**Funzionante / criticità residue.** Funzionano mapping ring GPIO47 e BMS→LED. Charging live e accuratezza cromatica/SOC non qualificate; SOC100 non significa carica completa.

**Decisione e riuso.** KEEP componente LED e prove specifiche; upgrade Q0/calibrazione.

**Evidenze:** [2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md · SRC-0007](#src-0007).

### 27/09 — CR2-C Q0 e CR3 preparazione

**Identità / installazione.** `315d4ade6ff0de59f6f3032f9864accb1680c669`; APP `753936ac1dc12d4af11f6ffaed51a8aaeda5b76f260e35e188a3d065f9f52b59`, 1.036.560 B, flash APP-only PASS.

**Architettura, problema e modifica.** Acquisizione q0 attuale senza torque, successiva preparazione backend/envelope/persistenza prove Q0. Corretto prima dell’esecuzione l’assunto servo rail separata: robot interamente alimentato e sostenuto.

**Prove ed esito effettivo.** HW-REPORT: SAFE_OFF 13/13, backup 16 MiB, Q0 CAPTURE 9/16 con posa manuale URDF; 12 candidati con spread 0. OFFLINE CR3 prepara esecuzione, non una calibrazione 24 contatti.

**Funzionante / criticità residue.** Funzionano read-only e mapping; Q0 candidato non è record gen1 completo. Calibrazione e Motion non autorizzati da questa prova.

**Decisione e riuso.** KEEP metodo identità/q0; ADAPT backend CR3. Upgrade quattro gambe/24 contatti.

**Evidenze:** [2026-09-27_CR2C_Q0_HARDWARE_SESSION.md · SRC-0008](#src-0008); [2026-09-28_CR3_M5_PRODUCTION_COMPOSITION.md · SRC-0009](#src-0009); [2026-09-28_CR3_M6_ENVELOPES_AND_SESSION_CLOSEOUT.md · SRC-0010](#src-0010).

### 29–30/09 — calibrazione quattro gambe, sequenze e tentativi

**Identità / installazione.** 29/09 `14881cdcf711a0213a2993390420f9f19c313056` riportata dalla V1: APP/digest e log nominativo di flash UNKNOWN in questa verifica, quindi non promossa a HW-LOG. Fonti primarie di generalizzazione offline e sequenza 24 contatti recuperate.

**Architettura, problema e modifica.** Rimozione assunzioni LF-only; LF→RF→RH→LH, pose prerequisite/parking per evitare ostacoli a q=0. Il semplice nome “Full Calibration” non assicura 24 contatti reali.

**Prove ed esito effettivo.** OFFLINE: generalizzazione/traceability V25 e geometria. Tentativi falliti 30/09–01/10 conservati accanto al successo del 01/10; identità e successione completa di ciascun tentativo restano UNKNOWN.

**Funzionante / criticità residue.** KEEP sequenza/parking e tracciabilità; non usare prove read-only o envelope non accettato come calibrazione completa.

**Decisione e riuso.** ADAPT implementazione quattro gambe; successo 24/24 con dfcecb6 il 01/10.

**Evidenze:** [2026-09-29_CALIBRATION_FULL_4LEG.md · SRC-0011](#src-0011); [2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md · SRC-0012](#src-0012); [README.md · SRC-0013](#src-0013).

### 01/10 — prima TRUE Full Calibration 24/24, ancora RAM-only

**Identità / installazione.** `dfcecb670d0565d2db1a8152b6cd7ad230bdb87d`, CLEAN; APP `5fe625cd061b203f4067acec7e6a28ab2582449c74b611f4f66607da11c503da`, 1.100.432 B. Flash 18:02; run 18:08:41–18:17:12.

**Architettura, problema e modifica.** Calibrazione nativa V25, goal discreti RAW e monitor senza IK/FK continui. Q0 promosso misurato per installazione attuale, non 2048 né zero affine.

**Prove ed esito effettivo.** HW-LOG tramite report con hash del log integrale: quattro gambe 6/6, nessun retry per gamba, witness 24/24, ripetibilità ≤4 tick. SAFE_OFF 13/13 e ritorno entro 6 tick. Export d4e1f4df…; log b1d6f60e… referenziati dal README.

**Funzionante / criticità residue.** Contatti e ritorno funzionano; RAM persa al power cycle, NVS/restore non implementati. LOWER MAX sistematicamente 3,0–5,7° corto vs URDF, RF margine scout6 tick; q0 shift non spiega MIN concorde e MAX corto. 23 spike termici confutati da letture dirette. Envelopes non accettati, nessuna qualifica STAND.

**Decisione e riuso.** KEEP misure angolari, sequenza, identità e lezioni meccaniche; SUPERSEDED persistenza RAM-only da gen1 DEV3. Non importare tick di installazioni precedenti.

**Evidenze:** [README.md · SRC-0013](#src-0013).

### 02–03/10 — migrazione M0/M0.1/M0.2/M0.3/M0.4 e layout NVS

**Identità / installazione.** Checkpoint `174aa04`/`2757604400db34e3107abacd994ca6fb9d0bb515`; candidato CLEAN `be0c12979e5b4b8ddc9dd21772d0d106e4358f4a`: powered APP `7cc1cbbc024e58630ed93fd0df8b7491659c0c20d6e1a4916333230eb1a82be0`, 1.123.472 B; USB `f0f3df4e83708f04d4e7acb44ab35794028521a0abfe50fa95b219694c498c3e`.

**Architettura, problema e modifica.** Passaggio layout legacy app0 3 MiB/FFAT a MATDOG_16M_2x5M_NVS_V1: slot 5 MiB e matdog_nvs a0xfe0000. Writer pinning/profili/readback, recupero ROM e confronto backup. M0.2 blocca migrazione: recupero fisico non pronto e tool USB_ONLY rifiuta powered PROFILE_MISMATCH. M0.4 qualifica software ROBOT_POWERED senza ricompilare.

**Prove ed esito effettivo.** OFFLINE: backup A/B e regioni, test writer/fault injection PASS; i rifiuti pre-porta non sono guasti del firmware vivo. M0.3 USB_ONLY resta riferimento legacy, non isolamento elettrico. Stato finale di ogni singolo evento W della migrazione UNKNOWN qui; layout V1 successivo accertato dai flash di ottobre.

**Funzionante / criticità residue.** KEEP backup, profilo esplicito e partizioni; USB_ONLY non spegne rail. Non dedurre avvenuta migrazione da piano PASS. Nessun moto.

**Decisione e riuso.** SUPERSEDED i blocchi come fotografia della sessione, KEEP controlli tecnici; chiusure M0.4/P1 e firmware persistenza seguono.

**Evidenze:** [2026-10-02_M0_2_HARDWARE_ALIGNMENT.md · SRC-0014](#src-0014); [REPORT.md · SRC-0015](#src-0015); [MATDOG_M0_3B_RECOVERY_USB_ONLY_LEGACY_REPORT.md · SRC-0016](#src-0016); [2026-10-03_M0_4_P1_CLOSED.md · SRC-0017](#src-0017).

### 03–05/10 — 0.1.0 persistenza, PHY e convergenza DEV1

**Identità / installazione.** V1 riporta `7b258b36c257…` e PHY `a06314e8f364…`; identità completa/digest flash del singolo checkpoint UNKNOWN nel materiale letto. DEV1 viene congelata e verificata nel package nominato.

**Architettura, problema e modifica.** Persistenza record/ACK e layout; PHY one-shot dà miglioramento RSSI iniziale senza stabilità al reboot. Post-abort termico e convergenza selettiva Motion/non-Motion. Wi-Fi/OTA Shelly è ramo offline separato, non automaticamente firmware vivo.

**Prove ed esito effettivo.** HW-REPORT nella V1: flash/readback e STA iniziale, poi RSSI −88/−84 e ping fallito; questi numeri restano attribuiti alla V1, non riconfermati con raw qui. Package DEV1 qualifica byte/provenienza offline; preflight hardware falliti prima di scrivere non sono installazioni.

**Funzionante / criticità residue.** KEEP NVS/provenienza; stabilità Wi-Fi non chiusa. Motion ancora non autorizzato; RESTORE non va confuso con LOAD/SAVE/ACK.

**Decisione e riuso.** ADAPT DEV1; upgrade boot census DEV2 e termica DEV3.

**Evidenze:** [FROZEN_CANDIDATE_VERIFICATION.md · SRC-0018](#src-0018); [CONVERGENCE_RECEIPT.md · SRC-0019](#src-0019); [VALIDATION.md · SRC-0001](#src-0001).

### 06/10 — DEV2 boot self-test e fallimento termico LF

**Identità / installazione.** `d0ede365dbbdbf3940a31929e8e2e132faf9d49f`, dev.2; APP installata prefisso235e67be, completo UNKNOWN qui. Boot/census hardware qualificati nel delta audit.

**Architettura, problema e modifica.** Automazione boot census; termica con latch su ripetuti spike bulk refutati. LF UPPER MAX legge95/78/77°C, diretti32–34°C; il terzo episodio pubblica71 e ferma OVER_TEMPERATURE.

**Prove ed esito effettivo.** HW-REPORT: FAIL_BLOCKING calibrazione, no retry/nessuna altra gamba; SAFE_OFF, export e permit revocato. Non è surriscaldamento dimostrato: il latch su artefatti confutati è la divergenza verificabile.

**Funzionante / criticità residue.** Boot self-test riutilizzabile. Dato bulk supera checksum ma non è temperatura affidabile durante moto; non eliminare protezione reale. Posa dopo stop non viene recuperata magicamente dal nuovo firmware.

**Decisione e riuso.** KEEP census; SUPERSEDED latch termico con DEV3.

**Evidenze:** [MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md · SRC-0020](#src-0020); [FAILURE_STOP_RECEIPT.md · SRC-0021](#src-0021).

### 06–07/10 — DEV3 termica diretta, 24/24 persistita gen1 e cold boot

**Identità / installazione.** `b3fd945bdaf37d192d97b605ac0f59b67f1dba45`, CLEAN; powered APP `5e2cbd17771712cc4e2ae10863edb33b59119754debdd5283551a1bb56f2c4be`, 1.178.272 B; build `2026-10-06T20:22:48Z`, flash20:59Z. USB distinta non flashata.

**Architettura, problema e modifica.** Spike bulk avvia conferma; verdetto da tre diretti, maggioranza/telemetry fault, limite 70°C invariato. Rimossi latch da bulk refutati; runner recovery solo same-boot e condizioni nominate. Dopo stop DEV2 riposizionamento q0 eseguito a mano, non recovery software dopo reboot.

**Prove ed esito effettivo.** HW-LOG / HW-REPORT: 24/24 circa 8,5 min, SAFE_OFF dopo gambe; 38 bulk refutati e115 conferme dirette. SAVE/ACK/reset, gen1 fingerprint be8d5902bb4e37da; record12 joint. 07/10 cold boot/persistenza. Raw NVS hash6d03b9dd804f470b26f0516b4093e733112b7bf58eb27989162798e6a00d3edd.

**Funzionante / criticità residue.** KEEP record/contatti e politica calibratore. Acceptance pacchetto BLOCKED per portal403; Motion_AUTHORIZED0, RESTORE NOT_IMPLEMENTED in questa fase. La futura singola conferma termica Motion non eredita automaticamente la qualifica a maggioranza.

**Decisione e riuso.** KEEP gen1 e safety; ADAPT termica. Upgrade AP/security, senza rifare contatti come rituale Motion.

**Evidenze:** [MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md · SRC-0022](#src-0022); [MATDOG_V0_2_DEV3_FINAL_HW_VALIDATION_REPORT_2026-10-07.md · SRC-0023](#src-0023).

### 08–10/10 — integrazioni selettive, AP403 e WPA2

**Identità / installazione.** PR1/2/3 sono integrazioni offline: non la APP DEV3 fisica. AP-only `dfcd2d00ee60091e5c3a3dd12adbbf76d43e0424`, APP `ad2c67137cd107bcced00fbb996f2953271ccff001ed42a501f9558625094ead` dalla V1, non flash confermato qui. Patch flash10/10 `837f5237de88ee0ce63003a0d46afea8369b8a9d` su `41753ac3f5ba9a14e5da8c786e8b124e30c17011`; SHA APP in manifest collegato.

**Architettura, problema e modifica.** AP403 viene trattato separatamente da RF/PHY. Error11: softAP riscrive config e genera AP_STOP/AP_START non armati. Password privata diversa da NVS/driver; commit di una richiesta non provava che la precedente AP KEY fosse applicata. Correzione ACK per sequenza e eventi attesi.

**Prove ed esito effettivo.** OFFLINE host/audit PASS; HW-REPORT flash837f523 PASS, due boot zero fault, stored=driver=file MATCH anche dopo reboot; associazione reale WPA2 inizialmente NON DIMOSTRATA. PR3 container build48/64 B diversa dal frozen DEV3: non stessa immagine.

**Funzionante / criticità residue.** KEEP gate/ACK/diagnosi; AP corretto nel driver non equivale a telefono collegato. MAIN/source integration non trasferisce l’acceptance hardware.

**Decisione e riuso.** SUPERSEDED rami intermedi; upgrade STA_STOP11/10.

**Evidenze:** [VALIDATION.md · SRC-0001](#src-0001); [AP_ROOT_CAUSE_FINAL.md · SRC-0024](#src-0024); [RAPPORTO.md · SRC-0025](#src-0025); [BINARY_MANIFEST.md · SRC-0026](#src-0026).

### 11–13/10 — chiusura non-Motion, PRONE e preparazione commissioning

**Identità / installazione.** Wi-Fi `b08322dedc5564c080f691dc98047511ef2b953f`, APP `ccbce2ff6ffee9a5e132bad8646f379fb31fdd305c0d577df992e85e2ebf4b43`; flash APP-only documentato dal closeout. PR47/altre build Motion non equivalgono alla installazione.

**Architettura, problema e modifica.** STA_STOP inatteso dopo bandwidth configura nuovo set atteso; chiusura AP/STA mantenendo gen1. 12/10 paired PRONE e contatto reale, route e FK/IK; 13/10 readiness runtime.

**Prove ed esito effettivo.** HW-REPORT AP/STA e NVS conservata. OFFLINE paired geometry e commissioning; Digital Twin telefono end-to-end e STAND/WALK completi non qualificati.

**Funzionante / criticità residue.** KEEP stack periferiche e geometria. RESTORE ancora non implementato nei checkpoint non-Motion; diventa operativo nei successivi commissioning.

**Decisione e riuso.** KEEP base non-Motion; ADAPT PRONE/route al Motion, upgrade83c3410.

**Evidenze:** [2026-10-11_HW_NON_MOTION_CLOSEOUT.md · SRC-0027](#src-0027); [MATDOG_HW_CLOSEOUT_INPUT.md · SRC-0028](#src-0028); [REPORT.md · SRC-0029](#src-0029); [REPORT.md · SRC-0030](#src-0030).

### 14–16/10 — runtime commissioning, realtime, stack fix e primo rifiuto

**Identità / installazione.** 14/10 `83c3410c90c151abfcd1747d3251dc75cd453d44`, APP `ec0a5aeae25ede2b5066f675adcca5ad1778f8ca988c872e3c2bde5e240cd91e`, 3.675.520 B riportati nella V1; flash identity raw non nuovamente estratta. 15/10 b064ef5/5358499, 16/10 stackfix9bbfe01: manifest e hash distinti nel §3. Manifest: sorgente `9bbfe01eab31fe423179e2f3ee505919bd761013` / `DIRTY`, build UTC `2026-10-15T21:07:02Z`, APP `754d798d189c2da339d7f982dc657e4ae5de96b82f6a5e506583825c3b8eab1f`, 3999408 B.

**Architettura, problema e modifica.** Commissioning PRONE, record restore e pipeline supervisor. Audit hardware14/10 separa start-pose/tracking; rework realtime e stack, branch IMU G5-B/inerziale offline non vanno fusi nel risultato installato.

**Prove ed esito effettivo.** OFFLINE build profili USB/powered, con DIRTY e UTC talvolta SOURCE_DATE_EPOCH: timestamp non è ora del test. Hardware16/10 STAND rifiutato STALE_FEEDBACK prima dell’attuazione secondo report post-validation; nessun difetto IK/giunti dimostrato.

**Funzionante / criticità residue.** KEEP runtime restore e diagnosi; readiness/heartbeat e temporizzazione ancora critici. APP delle candidate15/16 non automaticamente installata; stato flash di ciascuna UNKNOWN salvo receipt specifica.

**Decisione e riuso.** ADAPT separazione servizi/STOP; DIAGNOSTIC gate commissioning. Upgrade supervised17/10.

**Evidenze:** [FLASH_GO_NO_GO.md · SRC-0031](#src-0031); [FINAL_REPORT_OPERATIONAL_ARCHITECTURE.md · SRC-0032](#src-0032); [REPORT.md · SRC-0033](#src-0033).

### 17/10 — supervised8e1673b e9512816, PRIME/Q0 diagnostici

**Identità / installazione.** Prima APP `8e1673b371400cacff9f24e344009bc20ae32b63`, SHA `af13420cd45858b1af89781b430fa49af97e114ddb168873cf5c66b3e0037711`, 3.887.408 B: write/verify PASS e inizialmente lasciata ROM, boot trattato a parte. Successiva9512816 installata in receipt distinto; SHA completo nel relativo manifest se presente.

**Architettura, problema e modifica.** PRIME diagnostico senza energizzare, Q0/readback e correzioni write backend. Stop-and-wait di≤6 mrad ad ogni microframe contro residui4–5 tick produce ASSESSMENT_STALE: non prova IK sbagliata.

**Prove ed esito effettivo.** HW-REPORT: PRIME COMPLETE_NO_ENERGIZE12/12 e SAFE_OFF; log di primo STAND riporta mancata convergenza/gate. Altra prova ID13 PRIME_BACKEND_NOT_APPLIED/BACKEND_REJECTED. Letture goal/speed/ACC zero dopo reboot non ricostruiscono registri al fault precedente.

**Funzionante / criticità residue.** KEEP bus/readback/SAFE_OFF. PRIME-only dimostra preparazione, non STAND né torque ON. Mancano readback concomitanti completi al fault ID13.

**Decisione e riuso.** DIAGNOSTIC le build PRIME; SUPERSEDED tolleranza microframe. Upgrade tolleranza/polling18–19/10.

**Evidenze:** [EXECUTION_REPORT.md · SRC-0034](#src-0034); [FIRST_BOOT_REPORT.md · SRC-0035](#src-0035); [FLASH_REPORT.md · SRC-0036](#src-0036); [ANALISI_PRIMO_STAND.md · SRC-0037](#src-0037); [CANDIDATE.md · SRC-0038](#src-0038).

### 18–19/10 — tolleranza,16ca51eb,a34d528 e b5be64c4

**Identità / installazione.** 18/10 patch `8b9ee70a60bfa777ed50484c5e9546ceffd72f3b` inizialmente offline. 19/10 flash16ca51eb3037754782260718b27d6a1dc06ebe2f, APP e4f71b8ec2cb410d9bfed91a9fe16fafaf8dd36ad77cb4921087a2fd545e5635. Poi a34d528eea43b670f919132d60e4f24f37a45898/b5199a178f54f3667a165fd023f75463db22eb8ebdc75742a213d02b6b5fbf15 e b5be64c4e7657b3b8f02deb312c6a2f0dc385812/8afd56037a415ff72d0c56a320f9e1b3671d2dab03b9a9ad5a46b7c0d1be87e7: tre APP distinte.

**Architettura, problema e modifica.** Correzione tolleranza/feedback e polling; progredire da ARMED verso streaming senza attendere arrivo di ogni microframe.

**Prove ed esito effettivo.** HW-LOG: flash/verify e restore gen1/census 13/13. a34 diagnostica: quattro ARMED→STALE in101–111 ms, tx2–3; b5be log: quattro TRACKING_LAG_LIMIT dopo 68–74 frame e~2,5–2,7 s. Le sessioni flash dichiarano MOTION NOT TESTED; le prove sono log successivi distinti. Non attribuire tutti i fault a16ca.

**Funzionante / criticità residue.** Funzionano boot, NVS/restore e parziale trasmissione. Nessuno STAND150 completo; numero TX non prova quota o applicazione ai12 servo.

**Decisione e riuso.** KEEP bus/restore; SUPERSEDED executor/polling da unified d84. Causa meccanica dei singoli arresti non discriminata.

**Evidenze:** [REPORT_FINALE.txt · SRC-0039](#src-0039); [REPORT_FINALE.txt · SRC-0040](#src-0040); [diagnostic-extract.txt · SRC-0041](#src-0041); [REPORT_FINALE.txt · SRC-0042](#src-0042); [stand-20261019T175718-0gwr3j.log · SRC-0043](#src-0043).

### 19/10 — unified eac9fc00→d84b65df, movimento parziale e ACCELERATION_LIMIT

**Identità / installazione.** Candidato eac9fc00e3744b711880b46a0fa55f72e5fd563d, APP1c49818545340314c675f335a5689e41c36146a0f11a075976d2ffde5248e21e, superato offline. Installata d84b65df755cd00afddc2d7278e48e6d1e722fb7, CLEAN; APP449e3e9260039920bff243f2c0c2e428df1da1d57da4848da5f78db4bd6e3082,3.916.432 B.

**Architettura, problema e modifica.** Pipeline Coordinator→PRIME/ENERGIZE/ARM→Entry/AdaptiveStand→retimer→Executor→ServoBus. Correttivi residui e backend nel preflash, non stessa APP della build unified iniziale.

**Prove ed esito effettivo.** HW-LOG: POST_FLASH e STAND_WALK; IMU_STALE, poi ACCELERATION_LIMIT now_ms30666/tx_sequence218, SAFE_OFF. Piccoli movimenti dichiarati dall’operatore.218 è cumulativo fra sessioni: non un conteggio garantito di un solo STAND. START_OUTSIDE_LIMITS/scope/profile/feedback sono distinti rifiuti nei resoconti.

**Funzionante / criticità residue.** Funzionano enable e movimento parziale; target/release/dt/giunto causale al fault mancanti. Il confronto accelerazione è software; non dimostra scarsa accelerazione del servo o cinematica errata. Nessun completamento STAND/WALK.

**Decisione e riuso.** KEEP log e protezioni; SUPERSEDED algoritmo clock da V2, pipeline poi SIMPLE.

**Evidenze:** [matdog_build_manifest.txt · SRC-0044](#src-0044); [STAND_WALK_HARDWARE_20261019.log · SRC-0045](#src-0045); [POST_FLASH_D84B65DF_20261019.log · SRC-0046](#src-0046); [REPORT.md · SRC-0047](#src-0047).

### 20/10 — Motion V2, clock accelerazione

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile. Commit/APP del passaggio riportati sotto.

**Architettura, problema e modifica.** 5d320795d69a6e76492deb81bf212a95498cfddb: eliminato miscuglio dt release/dt TX-complete nel calcolo accelerazione.

**Prove ed esito effettivo.** OFFLINE stesso stimolo passa da3,176 a1,750 rad/s²; vero3,5 continua rifiutato. Build corretta; obiettivo≤5 s FAIL sul planner completo.

**Funzionante / criticità residue.** Nesso esclusivo con primo fault hardware HYPOTHESIS: operandi reali mancanti.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [REPORT.md · SRC-0047](#src-0047).

### 20/10 — Motion V3, fine TX reale

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile. Commit/APP del passaggio riportati sotto.

**Architettura, problema e modifica.** b7bc4336d5c25845305bcc39be90e07615fcd3fb: campione prima della fine UART non avvia quiete checkpoint.

**Prove ed esito effettivo.** OFFLINE arrivo post-TX e STOP; prestazioni≤5 s NON raggiunte. Un IOTimeOut inter-byte non limita una transazione intera.

**Funzionante / criticità residue.** Nessuna qualifica dinamica o tempo ESP32; TX-complete resta diverso da esecuzione servo.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [REPORT.md · SRC-0048](#src-0048).

### 20/10 — Motion V4, quote e retimer

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile. Commit/APP del passaggio riportati sotto.

**Architettura, problema e modifica.** 9d7eadbbf296538d2ee2883354bac81e2a84109a: durata quota non più minimo 12 s universale; admission STOP STA e diagnostica sottoguardia HTTP.

**Prove ed esito effettivo.** OFFLINE150→149 da12,152 a1,852 s; STAND completo~18,489 s. Nessuna installazione documentata in quel report.

**Funzionante / criticità residue.** Target≤5 s mancato; stabilità fisica, WCET e recovery universale UNKNOWN.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [REPORT.md · SRC-0049](#src-0049).

### 20–21/10 — Motion V5 e fix P0

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile.  Manifest: sorgente `ac37c15819d9f4031acba548d7ee6e96d05c6215` / `CLEAN`, build UTC `2026-10-20T22:32:20Z`, APP `5ad7ac082bd5dfd09fc8438d1da1737822329224c3a23d6775adc54620147bcc`, 3936608 B.

**Architettura, problema e modifica.** WP1 UART, WP2 IMU, WP3 MotionLaw/quote. Successivo43b018f7a1247a49442644d7fb4f868c5714ede2 corregge stale UART e lane STOP nel percorso locale; APP8b7a45c58aa1d95ca6ea8d67c8a409c226dc41baba174cbd5d3d739fef2d4875.

**Prove ed esito effettivo.** OFFLINE V5 STAND~17,737 s vs18,489 V4. Trial4d109c… è9d7eadbb+DIRTY, non ac37 CLEAN. P0 compilato, flash del solo P0 UNKNOWN qui.

**Funzionante / criticità residue.** Nessuna prestazione fisica attribuita al replay; V5 non è una singola immagine.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [REPORT.md · SRC-0050](#src-0050).

### 21/10 — V5 IMU fix realmente provata

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile.  Manifest: sorgente `9eeef0ddd74005522ee6766e6839df295aa26c49` / `CLEAN`, build UTC `2026-10-21T06:47:49Z`, APP `26be67499cf886985cd798e835dca7b2949882be1f0a945aa6bd6cabb42f4723`, 3937680 B.

**Architettura, problema e modifica.** 9eeef0d corregge percorso IMU; differente dal candidatoac37.

**Prove ed esito effettivo.** HW-LOG / HW-REPORT: sette arresti:2 JOINT_LIMIT senza frame Motion, poi4 ACCELERATION_LIMIT e1 OUTSIDE_PATH_BOX durante REAR; piccoli movimenti osservati. Boot/artefatto/dichiarazione coerenti, ma log STAND senza nuovo digest della flash.

**Funzionante / criticità residue.** Operandi accelerazione/RAW completi mancanti; JOINT_LIMIT iniziale non è fallimento della traiettoria attuata.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [RCA_STAND_20261021_PHASE1.md · SRC-0051](#src-0051).

### 21/10 — V6 pre-ARM e530e6f4

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile.  Manifest: sorgente `2f7b81822d708444db910bfe0f179dcc41be1b2e` / `CLEAN`, build UTC `2026-10-21T15:57:52Z`, APP `4cec9a11ecfd7dafc1784c311810505de9635c6b32cccfbd37cda0052b1e5b7a`, 3944496 B.

**Architettura, problema e modifica.** 2f7b818: gate pre-ARM chiama12 volte velocityForLateness numerica; blocca13,6 s.530e6f4172235bb86de08fe852e6b5ddef9a9b99 corregge lavoro sincrono costoso; APPcb69ed4b6305ef28b56edd753b7452c934286e0a3637b024e6d3f940a6eba73d.

**Prove ed esito effettivo.** HW-LOG: quattro rifiuti STALE prima dell’attuazione su 2f7b818.530e build OFFLINE; installazione separata non dimostrata qui.

**Funzionante / criticità residue.** Durante i quattro rifiuti IK/retimer/target non entrano in esecuzione. Il blocco di calcolo, non una cinematica fallita, è la prima divergenza.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [RCA_OPERATIONAL_FAILURE.md · SRC-0052](#src-0052).

### 21–22/10 — V6 ad0a091, sei abort REAR

**Identità / installazione.** Identità completa nel manifest §3 ove disponibile.  Manifest: sorgente `ad0a0910124cb849056f872c83594916902dfe24` / `CLEAN`, build UTC `2026-10-21T19:36:18Z`, APP `7ada67c82992c16326e1cc039c5a70ffb98761d350b51bdab0ebe469981144a2`, 3944800 B.

**Architettura, problema e modifica.** Fix readiness consente PRIME/ARM e REAR_DEPLOY. PATH_BOX verifica regione temporale su feedback reale.

**Prove ed esito effettivo.** HW-LOG flash21/10 e6 STAND150: tutti OUTSIDE_PATH_BOX, joint10 LH_UPPER/M42,20,05–21,71 mrad. SAFE_OFF latch→verified~31–32 ms nelle prove. Mai RISE né STAND150.

**Funzionante / criticità residue.** ~3,7–4 mrad confronto nuovo goal; residuo15,6–17,7 rispetto a goal trasmessi solo se consegnati. ACC20/carico/attrito/unità/TX perso HYPOTHESIS. Modelloτ30 ms falsificato; predicato non dimostrato matematicamente errato.

**Decisione e riuso.** SUPERSEDED implementazione executor/retimer da SIMPLE. KEEP prove negative, STOP e principi fisici; ADAPT geometria e limiti senza reintrodurre automaticamente PATH_BOX temporale.

**Evidenze:** [RCA_PATHBOX_PHASE1_OPUS55_20261022.md · SRC-0053](#src-0053).

### 22/10 — SIMPLE RC1 e RC2, semplificazione e fallimento geometrico offline

**Identità / installazione.** RC1 e RC2 hanno manifest CLEAN distinti nel §3; RC1 `46123741fcfc4fc393f48752f3fe156fb3e94369`, RC2 `8d0b47da979e809b9f5460a6827534e81e4532bc`. Nessun flash reperito di queste due APP. Manifest: sorgente `46123741fcfc4fc393f48752f3fe156fb3e94369` / `CLEAN`, build UTC `2026-10-22T11:05:35Z`, APP `018fcf294819e1fb2b8ca5fe3cbbe8b2836c7e05056b9906822d3e4131269f5d`, 1603360 B.

**Architettura, problema e modifica.** Comando semantico→SimpleMotion→gait/IK→ServoBus SYNC_WRITE; rimosso vecchio executor multilivello e PRIME/ARM come rituali utente. Prima descrizione25 Hz; le fasi successive evolvono a20 ms. RC2 usa corde tra nodi di fase.

**Prove ed esito effettivo.** OFFLINE build RC1 iniziale e finale conservate; RC2 ottimizza, ma corde deviano69/283 mrad e piedi15 mm; LIFT 107→251 certificate NOT_PASS per quattro coppie anteriori alla soglia 3 mm. Nessuna collisione hardware osservata.

**Funzionante / criticità residue.** KEEP FK/IK/resolver e owner UART; SUPERSEDED scorciatoie RC2. Il successo host non autorizza trasferire geometria di un percorso diverso.

**Decisione e riuso.** ADAPT SIMPLE; upgrade RC3 via-point e separazione DEPLOY/LIFT/HEIGHT.

**Evidenze:** [SIMPLE_MOTION_V1.md · SRC-0054](#src-0054); [SIMPLE_OPT_20261022 · SRC-0055](#src-0055); [SIMPLE_TRAJ_V2_20261022 · SRC-0056](#src-0056).

### 22/10 — SIMPLE RC3, flash e restore fisico

**Identità / installazione.** Flash22/10, non23/10: il23 è richiamo documentale. Commit e APP nel manifest. APP-only/readback PASS; sostituisce ad0a091. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `CLEAN`, build UTC `2026-10-22T19:49:11Z`, APP `e244f9b8e1d3d2be10defc348912b947cf7676b56142753e1a82b61d8d00d2ea`, 1598400 B.

**Architettura, problema e modifica.** Route con via-point, DEPLOY 0–107/LIFT 107–160 e HEIGHT; regioni certificate nel dominio. STAND semantico, WALK, IMU e STOP.

**Prove ed esito effettivo.** HW-LOG: due boot restore12/12, census 13/13, IMU/BMS disponibili, NVS gen1 invariata, torque inizialeOFF/SAFE_OFF. Nessun STAND/WALK nella sessione flash. OFFLINE replay PRONE→150~12,80 s e WALK0,0046–0,0092 m/s.

**Funzionante / criticità residue.** Funzionano installazione e restore; dashboard STAND/STOP400, BASE_VELOCITY403; obiettivi≤5 s e≥0,06 m/s non raggiunti dai replay. Certificati non provano dinamica/terreno.

**Decisione e riuso.** KEEP geometria/restore e semplicità; SUPERSEDED contratto dashboard da P1/P1.1/RC4.

**Evidenze:** [REPORT_FLASH_BOOT.md · SRC-0057](#src-0057); [SIMPLE_MOTION_V1.md · SRC-0054](#src-0054).

### 23/10 — P1 Dashboard e P1.1 STOP lane

**Identità / installazione.** Due APP DIRTY su HEAD RC3, entrambe rc.3: P1 `1e30228076d6ae251120adbb504bb95805fd7fe57af17037c7ea5cd9694f79d8`; P1.1 nel manifest. Nessun flash reperito nei report. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-23T07:38:29Z`, APP `ad287f708bac5771521e6bfbb4d706e636c8641bdf5aa71191af335002ce120a`, 1601696 B.

**Architettura, problema e modifica.** P1 allinea dashboard/POSTURE.STAND/MOTION.STOP; P1.1 rende STOP ripetuto idempotente e preserva progresso SAFE_OFF 13/13, slot prioritario rispetto al mailbox.

**Prove ed esito effettivo.** OFFLINE riproduzione400/400/403 e regressioni PASS. La lane non garantisce arrivo via socket quando il solo HTTPD è già occupato.

**Funzionante / criticità residue.** KEEP contratto semantico e latch/progresso. Latenza STOP hardware delle APP UNKNOWN.

**Decisione e riuso.** SUPERSEDED candidate da RC4, KEEP correzioni.

**Evidenze:** [SIMPLE_RC3_P1_DASHBOARD_OFFLINE_20261023 · SRC-0058](#src-0058); [SIMPLE_RC3_P1_1_STOP_LANE_OFFLINE_20261023 · SRC-0059](#src-0059).

### 23–24/10 — RC4 prima/r2/r3/r4 e “final” interrotta

**Identità / installazione.** APP primaac31bf0d…, r22a1c391…, r3fb747265…, r499128852…: SHA completi e date UTC nel §3; tutte HEAD f838+DIRTY. r2:23 modificati+3 untracked/26 hash. Directory final23/10 non è una build finale riuscita. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-23T17:38:35Z`, APP `2a1c39162c09086a61c0edf71f952dab528a094ee36baa857599b165fa48bc84`, 1606576 B.

**Architettura, problema e modifica.** RUN al boot senza movimento spontaneo, restore automatico, nessun deadman browser, STAND persistente/WALK fino STOP o fault; consolidamento UI/HTTP/Twin. Cambi specifici r3/r4 da attribuire ai rispettivi artefatti, non al nome generico rc.4.

**Prove ed esito effettivo.** OFFLINE suite mirate; audit finale FAIL25 finding, con aspettative executor rimosso e harness realmente non eseguibili. Log build final23/10 interrotto conservato: non attribuirgli APP di r2.

**Funzionante / criticità residue.** Implementazione operativa più semplice, ma nessun nuovo esito fisico per queste quattro APP nei checkpoint. Funzionalità desiderate non equivalgono a STAND/WALK riusciti.

**Decisione e riuso.** SUPERSEDED build da final-codex; KEEP direzione architetturale. Audit storico resta FAIL.

**Evidenze:** [MATDOG_CONTROLLER_RC4_OFFLINE_20261023 · SRC-0060](#src-0060); [MATDOG_CONTROLLER_RC4_R3_OFFLINE_20261024 · SRC-0061](#src-0061); [MATDOG_CONTROLLER_RC4_R4_OFFLINE_20261024 · SRC-0062](#src-0062); [matdog-controller-1.0.0-rc.4-final-20261023.build.log · SRC-0063](#src-0063).

### 24/10 — RC4 final-codex realmente installata

**Identità / installazione.** APP 62fc214… nel manifest, flash24/10 10:37Z sessione nominata; non più soltanto offline. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-24T06:47:09Z`, APP `62fc214a1546c3d4cd21b5cd0bbfe8a33868ac6a6282ee474d457394a5d75384`, 1610176 B.

**Architettura, problema e modifica.** Chiusura correttezza dashboard/HTTP e review flags prima della prova Motion.

**Prove ed esito effettivo.** HW-LOG: una scrittura APP0/digest indipendente; regioni protette identiche, boot90 s, RUN/READY/IDLE, gen1 automatic restore12/12, census 13/13, SAFE_OFF 13/13, IMU/DALY ONLINE. Nessun Motion impartito nella sessione.

**Funzionante / criticità residue.** Funzionano boot e periferiche. max_gap27718 µs/missed0 in campioni idle non qualifica carico Motion; Twin end-to-end UNKNOWN. Banner STALE prima del restore non è perdita gen1.

**Decisione e riuso.** KEEP boot/restore e prove; upgrade RX recovery dopo staleness.

**Evidenze:** [REPORT.md · SRC-0064](#src-0064); [REPORT_FLASH_BOOT.md · SRC-0065](#src-0065).

### 24/10 — RX Recovery e soak

**Identità / installazione.** b391458… distinta da 62fc; flash e osservazione separati. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-24T12:32:36Z`, APP `b39145868c66fc68b559ff5c6b6803979d481b822c1db789669ff4c7e00efa36`, 1610560 B.

**Architettura, problema e modifica.** RX_PENDING persistente non recuperava; ripristinata avanzabilità READ senza allentare parser e senza gate Motion aggiuntivi.

**Prove ed esito effettivo.** HW-REPORT: soak650 s/64 campioni READY, nessuno STALE, torque OFF; OFFLINE regressioni bus. Prova successiva STAND genera guasto pre-enable, non successo implicito del soak.

**Funzionante / criticità residue.** KEEP recovery e parser; ACK/TX e cache fresche non provano registri control né traiettoria.

**Decisione e riuso.** KEEP fix, SUPERSEDED APP da preenable/control-pending.

**Evidenze:** [REPORT_RX_RECOVERY.md · SRC-0066](#src-0066); [REPORT_FLASH_OBSERVATION.md · SRC-0067](#src-0067).

### 24/10 — Pre-enable4e312c7a, STOP socket e Twin

**Identità / installazione.** 4e312c7a82f4ac864c057deedb76d3b4482babb6ef82321ce50b215e40be48f7; installazione confermata dalle fonti successive sulla APP corrente e sessione APP-only. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-24T14:05:45Z`, APP `4e312c7a82f4ac864c057deedb76d3b4482babb6ef82321ce50b215e40be48f7`, 1611072 B.

**Architettura, problema e modifica.** Gestione letture/frame non applicati prima enable; STOP failed-fetch distinto da risposta di rifiuto.7 socket riduce possibile LRU close; dashboard postMessage allinea età IMU e RAW per Twin.

**Prove ed esito effettivo.** OFFLINE regressioni e negativi PASS; HW-REPORT cinque REFERENCE_CHANGED e1 INITIAL_SERVO_READ_FAILED, RAW stabili/nessun movimento. STOP richiesto resta non applicato nello stato FAULT, ma SAFE_OFF 13/13 già effettuato dal fault.

**Funzionante / criticità residue.** Funziona de-escalation. Trasporto STOP causa UNKNOWN senza cattura; LRU è meccanismo plausibile, non causa misurata. Viewer standalone conserva badge?ms/N-D; bundle non riproducibile dai sorgenti attuali.

**Decisione e riuso.** KEEP distinzione UX/telemetria/safety; upgrade RCA control-pending senza chiamare REFERENCE un difetto IK.

**Evidenze:** [REPORT.md · SRC-0068](#src-0068); [MATDOG_PREENABLE_APP_ONLY_20261024T165255Z · SRC-0069](#src-0069); [REPORT.md · SRC-0070](#src-0070).

### 24–25/10 — Control Pending e Final Pre-enable

**Identità / installazione.** 7fd0d9ee… candidata intermedia OFFLINE; cdcefdf5… finale flashata25/10, non stessa APP. Manifest distinti nel §3. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-24T23:41:18Z`, APP `cdcefdf5c8f0bb30687796158f205b893ebf27f1aef8f4f805a3ddc88e35712e`, 1611776 B.

**Architettura, problema e modifica.** Pending per ID/width ma last TX per ID: fast width 15 rinvia indefinitamente control width 10 su ID21/33. Patch timestamp per ID+width. Finale distingue RX_PENDING senza TX da vera mancata risposta; tabella compattata da33 larghezze a4, risparmio29464 B.

**Prove ed esito effettivo.** OFFLINE vecchia regola46 assertion FAIL→17738 check PASS; vecchia preparazione23 scenari FAIL→integrata PASS. HW-REPORT finale flash/boot PASS, recupero ID21/33~1440 ms; poi quattro MOTION_REFERENCE_FAULT su ID13: torque 1,raw=ref 1989,delta 0,profile 300/8.

**Funzionante / criticità residue.** KEEP controllo pending; causa dei cinque REFERENCE della4e312 resta UNKNOWN. La lettura valida torque 1 sucdcef è distinta dal controllo RX. Risposta remota stessaID/width oltre 100 ms resta ambigua nel protocollo.

**Decisione e riuso.** KEEP fix, SUPERSEDED preparazione da Torque Fix.

**Evidenze:** [REPORT.md · SRC-0070](#src-0070); [REPORT.md · SRC-0071](#src-0071); [REPORT_FLASH_BOOT.md · SRC-0072](#src-0072); [hardware-original.log · SRC-0073](#src-0073).

### 25/10 — Torque Fix, effetto GoalPosition e aborti successivi

**Identità / installazione.** 942171458f55515e66177f56b0b6cbcaca658cd4ac2abb0ae5ee24d06b7de3d8, flash/boot nel report; non confondere con cdcef. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-25T07:01:38Z`, APP `942171458f55515e66177f56b0b6cbcaca658cd4ac2abb0ae5ee24d06b7de3d8`, 1611904 B.

**Architettura, problema e modifica.** Prima divergenza REFERENCE→primo ENABLE prima di ogni EnableTorque1 esplicito. QC C018 storico mostra WritePosEx che arma torque; estensione al SYNC_WRITE del 25/10 è inferenza riprodotta, non cattura TX/RX reale. Patch OFF dopo reference e OFF solo suffisso dopo retry, readback primaON.

**Prove ed esito effettivo.** OFFLINE regressioni bus+motion; HW-REPORT APP installata, successivi tentativi log090701:31 tentativi classificati dalla RCA,26 STALE, 3 REFERENCE_CHANGED, 1 CONTROL_READ_LOST e 1 BMS_OR_POWER_UNAVAILABLE (vedere fonte per cronologia). Nessun STAND riuscito qualificato.

**Funzionante / criticità residue.** KEEP ordine OFF/readback e reference RAW immutata. Non provati guastoID13, IK errata o falsa calibrazione; causalità completa torque residua dopo broadcast UNKNOWN.

**Decisione e riuso.** KEEP fix; upgrade RCA carico→correttiva/timing→Jacobiano→RC5.

**Evidenze:** [REPORT.md · SRC-0074](#src-0074); [REPORT_FLASH_BOOT.md · SRC-0075](#src-0075); [REPORT.md · SRC-0076](#src-0076).

### 25/10 — RCA carico, correttiva520c1e87 e diagnostica timing

**Identità / installazione.** RCA iniziale nessuna APP nuova, firmware942171 invariato. Corretta520c1e87… offline1611936 B. Diagnostica finale installata d15d4711598b638992dc4472d79fe723c1770d28adbd832450f005e3b52988b5,1614656 B; prima timing-only conservata e superata, SHA nel suo receipt da recuperare. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-25T16:26:54Z`, APP `520c1e87c8cb2349e2a9868c12c4973a36de57b9ab37d693da6b6abf0b999eb6`, 1611936 B.

**Architettura, problema e modifica.** Carico gen/compensazione/IK/FK e provenanceTag osservato host, causa primaria durata target UNKNOWN. Correttiva cache tag albind/clear, freshness al clock reale, DALY RX sempre servita e solo avvioTX sotto slack. Timing-only separata congela primo sforamento RAM e stampa dopo SAFE_OFF.

**Prove ed esito effettivo.** OFFLINE: 5200 target/4978 frame RAW byte-identici; regressioni PASS. Vecchio modello40/47/81 ms non produce STALE: aspettativa iniziale fallita conservata;45/47/81 produce dato263,43 ms e STALE. Pausa16 ms dimostra falso FUTURE solo condizionale. HW-LOG flashd15 e boot READY, nessun Motion nella sessione; logstand50_184914 è solo acquisizione passiva.

**Funzionante / criticità residue.** KEEP correzioni dimostrate, non causa hardware retroattiva. Profilo x86 non misura ESP32; campione FAULT fresco dopo polling non ricostruisce gate causale. Nessuna prova che logger provochi il carico.

**Decisione e riuso.** DIAGNOSTIC d15, non firmware operativo finale; SUPERSEDED candidata520 da ulteriori ottimizzazioni. Servono tempi target già indicati, nessuna nuova prova eseguita per il diario.

**Evidenze:** [REPORT.md · SRC-0076](#src-0076); [REPORT.md · SRC-0077](#src-0077); [RESUME.md · SRC-0078](#src-0078); [REPORT.md · SRC-0079](#src-0079).

### 25/10 sera — RC4 Jacobiano analitico installato dopo due preflight

**Identità / installazione.** dbc64f119121d72558573443b8cf6198ac9eb29f29d5a2f75503ab8b085e4313. Directory213809 contiene preflight;214602 OTA verifica preliminare;214919 contiene vera scrittura/verify. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-25T20:19:44Z`, APP `dbc64f119121d72558573443b8cf6198ac9eb29f29d5a2f75503ab8b085e4313`, 1613616 B.

**Architettura, problema e modifica.** Jacobiano analitico sostituisce differenze finite mantenendo percorso/target; prosegue ottimizzazione del costo numerico.

**Prove ed esito effettivo.** OFFLINE A/B87 righe byte-identiche secondo V1/replay conservati, senza target/RAW misurati al robot; suite gait envelope fallita per sorgenti assenti, non PASS. HW-LOG verify0x189f30=1613616 byte digest matched. ZIP220111 analizzato dal night handoff:176 snapshot,13 ingressiFAULT,10BMS/2reference/1stale,miss 0→111,max gap51398 µs.

**Funzionante / criticità residue.** Funziona installazione, non STAND completo. Primo preflight bloccato non annulla il successivo flash riuscito. Nessun nesso dimostrato “Jacobiano→BMS fault”; contributi elettrici/timing non separati.

**Decisione e riuso.** KEEP analitico nei limiti equivalenza; SUPERSEDED APP da RC5 night recovery.

**Evidenze:** [MATDOG_ANALYTIC_JAC_20261025 · SRC-0080](#src-0080); [FLASH_ANALYTIC_JAC_20261025_213809 · SRC-0081](#src-0081); [ota-verification.log · SRC-0082](#src-0082); [flash.log · SRC-0083](#src-0083); [verify.log · SRC-0084](#src-0084); [HANDOFF.md · SRC-0085](#src-0085).

### 25 notte–26/10 — prima RC5 Night Recovery

**Identità / installazione.** 35c659cb206643f8fbc0d734162b7ce27f8125ff8c2becce08eef84927db5a99; build25/10 22:15Z, consegna26/10; non flashata nel suo report. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-25T22:15:07Z`, APP `35c659cb206643f8fbc0d734162b7ce27f8125ff8c2becce08eef84927db5a99`, 1615744 B.

**Architettura, problema e modifica.** DALY scheduler asincrono indipendente dallo slack, senza flush; policy separa allarmi/MOS/stale/power. HTTP/TLS core0, RAM snapshot IMU, altezza10Hz in slack, LED rinviato duranteengaged; HOLD riusa nominale, no doppia generazione confini. RX_PENDING classificato correttamente anche secondo blocco differito. Reference guard/retry OFF limitato2 con readback.

**Prove ed esito effettivo.** OFFLINE DALY reale con doubles,17795 bus check,13 pose registrate→STAND50 modello, STAND150/WALK3 stride/STOP/SAFE_OFF; zero miss nel carico virtuale6ms. Audit completo storico FAIL30, pure-source19→17; non verde.

**Funzionante / criticità residue.** KEEP riduzione interferenze e fault specifici. Soglia BMS3 s invariata,5 s classifica persistenza senza grazia movimento. RAW13 pose replay non è dinamica fisica. Causa dei10BMS hardware non provata perché predicati causali mancanti.

**Decisione e riuso.** KEEP correzioni; SUPERSEDED APP da reference_guard 54ade4.

**Evidenze:** [HANDOFF.md · SRC-0085](#src-0085); [FINAL_RECEIPT.json · SRC-0086](#src-0086).

### 26/10 — RC5 reference_guard, flash e due STAND con blackout HTTP

**Identità / installazione.** 54ade4fdf9c620194d15ed7990d2856ef1a971232988fecf6addca14bc18046f,1.615.728 B; flash 07:16 sessione con verify/readback e boot. È l’ultima APP con installazione reperita. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-26T05:50:05Z`, APP `54ade4fdf9c620194d15ed7990d2856ef1a971232988fecf6addca14bc18046f`, 1615728 B.

**Architettura, problema e modifica.** Micro-patch diagnostica reference_guard: cambia latch/campione del motivo, non legge di moto. Preserva analitico, DALY async, retry OFF e SAFE_OFF.206 copie sorgenti compilatore/inventari attestati dalla review.

**Prove ed esito effettivo.** OFFLINE51 check diagnostica, power-chain reale con doubles, dashboard e regressioni; audit17 invariati classificati14 obsoleti/3falsi positivi, FAIL storico conservato. HW-LOG: STAND50: ultimo campione07:33:39.802 READY/IDLE,miss 0; primo50307:33:42.022, body mancante, poi timeout. STAND150 ultimo07:37:23.455 MOVING/ENABLE,torque attiva,fault false,miss 0,BMS age 1689 ms / IMU valida; timeout07:37:27.463. Operatore osserva movimento e pose compatibili; recupero dopo KEY OFF/ON.

**Funzionante / criticità residue.** Funziona movimento fisico parziale/visivo; nessun campione riuscito post-comando nel primo logger. Quota reale, HOLD completato, tracking12 joint, STOP durante blackout UNKNOWN.503 prova risposta radio/TCP/httpd iniziale, non spegnimento AP; riavvio fra prove manuale, non reset spontaneo dimostrato. Servo può finire ultimo goal localmente.

**Decisione e riuso.** KEEP evidenze fisiche; IMPLEMENTED ma non qualificati controllo continuo/STAND/WALK. Upgrade rete bac0→fe33; non attribuire blackout a cinematica.

**Evidenze:** [FLASH_AND_BOOT_REPORT.md · SRC-0087](#src-0087); [FINAL_REVIEW.md · SRC-0088](#src-0088); [hardware-evidence-summary.json · SRC-0089](#src-0089); [MATDOG_ANALYTIC_STAND50_20261026_073322_1237175 · SRC-0090](#src-0090); [MATDOG_ANALYTIC_STAND50_20261026_073709_1238452 · SRC-0091](#src-0091).

### 26/10 — Network Recovery bac0c9de, intermedia non flashata

**Identità / installazione.** bac0c9dedbefa3945da166aec6bb6b249e677665263b9264f4af21b4b9eeaee8,1.615.712 B; versione sempre rc.5; nessuna installazione nel report. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-26T07:10:31Z`, APP `bac0c9dedbefa3945da166aec6bb6b249e677665263b9264f4af21b4b9eeaee8`, 1615712 B.

**Architettura, problema e modifica.** Due difetti software riprodotti: snapshot Wi-Fi sospeso perengaged ancheHOLD; richiesta scaduta mai servita trattiene mailbox senza slack e può partire tardi. Patch copia RAM continua e cleanup prima gate5 ms.

**Prove ed esito effettivo.** OFFLINE snapshot1035 check, mailbox/fence79 TSan/ASAN, regressioni e buildPASS; HTTP legacy harness FAIL per ancora assente, audit25 invariato. Controprove successive trovano ancora race: reader scade fra primo awaitingResponse e ammissione/chiamata Motion; commands1 accepted1.

**Funzionante / criticità residue.** KEEP snapshot/cleanup; SUPERSEDED questa APP per cancellazione incompleta. Nessuna fairness garantita: se visita sempre a+16ms nel periodo 20 ms manca margine5 ms. Freeze dello snapshot non spegne direttamente AP, worker continua: causa completa blackout non dimostrata.

**Decisione e riuso.** Upgrade immediato fe33fab, Motion invariato. Non contare test iniziali PASS come qualifica della race non ancora coperta.

**Evidenze:** [REPORT.md · SRC-0092](#src-0092); [NETWORK_ONLY.patch · SRC-0093](#src-0093); [RESULT.md · SRC-0094](#src-0094).

### 26/10 — candidata finale RC5 fe33fab, qualificata offline

**Identità / installazione.** fe33fab65672642b39f611911c5996d949ba2877727c89407e5daad438b2e2e3. Manifest SHA 0edcc3be243cf09b98913ff7e0523fb3c05c18ab6da106d2ec951b30c8f6ee6b; HEAD f838+DIRTY, non un nuovo commit. Ultima candidata reperita; NON FLASHATA secondo receipt. Manifest: sorgente `f838f677a1826c4eaabc5889473ef016b1ebad98` / `DIRTY`, build UTC `2026-10-26T08:19:48Z`, APP `fe33fab65672642b39f611911c5996d949ba2877727c89407e5daad438b2e2e3`, 1615824 B.

**Architettura, problema e modifica.** Solo HttpTransport rispetto a bac0: TTL Motion ordinario non STOP limitato al budget dispatch, ricontrollo abbandono dopo ammissione e clock millis aggiornato alla validazione. Preserva snapshot Wi-Fi e pulizia. Nessuna modifica Motion/IK/gait/servo/BMS/IMU/calibrazione o policySTOP.

**Prove ed esito effettivo.** OFFLINE stessi62 controlli: rc.5 precedente13 errori, bac0 3 errori, finale0 sottoASAN/UBSAN. Concorrenza82 check TSAN+82ASAN; snapshot 1035 PASS.341 release simulate/0miss e330 rinvii perMOVING / HOLD non sono tempi reali. Comandi scaduti non accettati, readerWAITING oltre budget→EXPIRED.201 artefatti protetti invariati,209 file compilati identici a bac0 e solo HttpTransport diverso. Logger ispezionato staticamente, mai avviato.

**Funzionante / criticità residue.** Funzionano casi host riprodotti. HTTP resta vincolato allo slack;503CONTROLLER_LOOP_LATE corretto può persistere. STOP lane separata dopo pubblicazione, ma singoloHTTPD può bloccare ingresso socket; ACK non è torque OFF13/13. WCET/radio/heap/stackHTTPD dopo guasto eSTOP end-to-end UNKNOWN. Audit globale storicoFAIL non promossoPASS.

**Decisione e riuso.** KEEP come candidata offline più recente, IMPLEMENTED hardware non validato. Nessun esito delle prove54ade4 trasferibile automaticamente. Futuro test è fuori da questo diario/read-only.

**Evidenze:** [RESULT.md · SRC-0094](#src-0094); [RECEIPT.json · SRC-0095](#src-0095); [CANCELLATION_ONLY.patch · SRC-0096](#src-0096); [source-final.json · SRC-0097](#src-0097).

## 3. Registro delle identità binarie e provenienza

Estrazione diretta dai manifest in percorsi di artefatti nominati. Non è una scansione di tutte le cache e non certifica che ogni build storica sia sopravvissuta. Le righe USB_ONLY e ROBOT_POWERED sono candidate diverse. Per i DIRTY il commit è soltanto la base; il diff/inventario va recuperato dal dossier associato, mai ricostruito dal worktree corrente.

**Verifica documentale eseguita ora:** dove il BIN è presente accanto al manifest, SHA256 e dimensione ricalcolati in sola lettura. `MATCH` prova integrità del file archiviato, non installazione. Un `UNKNOWN` significa BIN non accanto alla copia manifest. Per stato flash/esito/riuso valgono le schede §2; le varianti solo-build non esplicitamente installate restano OFFLINE/flash UNKNOWN.

| Build / profilo / manifest | Build UTC dichiarata | Sorgente / stato | APP SHA256 | Byte dichiarati / letti | Confronto file |
|---|---|---|---|---|---|
| [matdog_build_manifest.txt · SRC-0098](#src-0098) (manifest SRC-0098) | 2026-10-25T20:19:44Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `dbc64f119121d72558573443b8cf6198ac9eb29f29d5a2f75503ab8b085e4313` | 1613616 / 1613616 | MATCH |
| [matdog_build_manifest.txt · SRC-0099](#src-0099) (manifest SRC-0099) | 2026-10-23T12:24:04Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `ac31bf0dc1fb47b3bb2c2bf3baf2421a9ff6aeb62d167d0dfa7331ebdfbd9dd8` | 1605456 / 1605456 | MATCH |
| [matdog_build_manifest.txt · SRC-0100](#src-0100) (manifest SRC-0100) | 2026-10-23T17:38:35Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `2a1c39162c09086a61c0edf71f952dab528a094ee36baa857599b165fa48bc84` | 1606576 / 1606576 | MATCH |
| [matdog_build_manifest.txt · SRC-0101](#src-0101) (manifest SRC-0101) | 2026-10-23T22:40:03Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `fb74726578c4a164a1030c58c5ff849a98f59dcd5f811d6cdb2ac6a1884aae8d` | 1608592 / 1608592 | MATCH |
| [matdog_build_manifest.txt · SRC-0102](#src-0102) (manifest SRC-0102) | 2026-10-24T06:25:19Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `99128852b3b466a0f20d7ff53f5cb9b8db54af7a6feb329eedcabd5135bfd021` | 1609680 / 1609680 | MATCH |
| [matdog_build_manifest.txt · SRC-0103](#src-0103) (manifest SRC-0103) | 2026-10-24T18:06:51Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `7fd0d9eebf8a0388008f908b409e771886420c6d04ec5000e16ad3aca3f3d34d` | 1613104 / 1613104 | MATCH |
| [matdog_build_manifest.txt · SRC-0104](#src-0104) (manifest SRC-0104) | 2026-10-24T06:47:09Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `62fc214a1546c3d4cd21b5cd0bbfe8a33868ac6a6282ee474d457394a5d75384` | 1610176 / 1610176 | MATCH |
| [matdog_build_manifest.txt · SRC-0105](#src-0105) (manifest SRC-0105) | 2026-10-25T16:26:54Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `520c1e87c8cb2349e2a9868c12c4973a36de57b9ab37d693da6b6abf0b999eb6` | 1611936 / 1611936 | MATCH |
| [matdog_build_manifest.txt · SRC-0106](#src-0106) (manifest SRC-0106) | 2026-10-24T23:41:18Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `cdcefdf5c8f0bb30687796158f205b893ebf27f1aef8f4f805a3ddc88e35712e` | 1611776 / 1611776 | MATCH |
| [matdog_build_manifest.txt · SRC-0107](#src-0107) (manifest SRC-0107) | 2026-10-24T14:05:45Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `4e312c7a82f4ac864c057deedb76d3b4482babb6ef82321ce50b215e40be48f7` | 1611072 / 1611072 | MATCH |
| [matdog_build_manifest.txt · SRC-0108](#src-0108) (manifest SRC-0108) | 2026-10-24T12:32:36Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `b39145868c66fc68b559ff5c6b6803979d481b822c1db789669ff4c7e00efa36` | 1610560 / 1610560 | MATCH |
| [matdog_build_manifest.txt · SRC-0109](#src-0109) (manifest SRC-0109) | 2026-10-25T07:01:38Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `942171458f55515e66177f56b0b6cbcaca658cd4ac2abb0ae5ee24d06b7de3d8` | 1611904 / 1611904 | MATCH |
| [matdog_build_manifest.txt · SRC-0110](#src-0110) (manifest SRC-0110) | 2026-10-25T22:15:07Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `35c659cb206643f8fbc0d734162b7ce27f8125ff8c2becce08eef84927db5a99` | 1615744 / 1615744 | MATCH |
| [matdog_build_manifest.txt · SRC-0111](#src-0111) (manifest SRC-0111) | 2026-10-26T05:50:05Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `54ade4fdf9c620194d15ed7990d2856ef1a971232988fecf6addca14bc18046f` | 1615728 / 1615728 | MATCH |
| [matdog_build_manifest.txt · SRC-0112](#src-0112) (manifest SRC-0112) | 2026-10-26T07:10:31Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `bac0c9dedbefa3945da166aec6bb6b249e677665263b9264f4af21b4b9eeaee8` | 1615712 / 1615712 | MATCH |
| [matdog_build_manifest.txt · SRC-0113](#src-0113) (manifest SRC-0113) | 2026-10-26T08:19:48Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `fe33fab65672642b39f611911c5996d949ba2877727c89407e5daad438b2e2e3` | 1615824 / 1615824 | MATCH |
| [matdog_build_manifest.txt · SRC-0114](#src-0114) (manifest SRC-0114) | 2026-10-15T10:11:13Z | `b064ef5625718df78be9a7ccee479b598e2944b9` / DIRTY | `b3e7aef3d897ad6940e3ea8ca4da9497d85dd769b9f30b334718f22542feb8ab` | 3949600 / 3949600 | MATCH |
| [matdog_build_manifest.txt · SRC-0115](#src-0115) (manifest SRC-0115) | 2026-10-15T16:01:32Z | `b064ef5625718df78be9a7ccee479b598e2944b9` / DIRTY | `36c975afd213f8e95b370dd31521cf246dfac0139f27f386a407e5b0aabba179` | 3837216 / 3837216 | MATCH |
| [matdog_build_manifest.txt · SRC-0116](#src-0116) (manifest SRC-0116) | 2026-10-15T10:11:13Z | `b064ef5625718df78be9a7ccee479b598e2944b9` / DIRTY | `4802c2bce4e6afe6134c44b4e6dfb6fc9467ba81157708df8010e54b4ee76c7e` | 3956096 / 3956096 | MATCH |
| [matdog_build_manifest.txt · SRC-0117](#src-0117) (manifest SRC-0117) | 2026-10-15T10:11:13Z | `b064ef5625718df78be9a7ccee479b598e2944b9` / DIRTY | `b4de06c90e96e3170431b052fad4b749ae6df34b38491c8b7c9b4c0274c6c8cf` | 3843728 / 3843728 | MATCH |
| [matdog_build_manifest.txt · SRC-0118](#src-0118) (manifest SRC-0118) | 2026-10-15T20:37:48Z | `5358499acb016a3d876aad3aa44b903584b66c98` / DIRTY | `1d1b31fc35dee1d1413c9a584e7b43a647cf2e6561cfc15ed3bb13b573255cd0` | 3956080 / 3956080 | MATCH |
| [matdog_build_manifest.txt · SRC-0119](#src-0119) (manifest SRC-0119) | 2026-10-15T20:37:48Z | `5358499acb016a3d876aad3aa44b903584b66c98` / DIRTY | `5cac4e062c4f23e5ab1c57da5919f838e59dff8a0330b298c6c3f07e731e15cf` | 3843728 / 3843728 | MATCH |
| [matdog_build_manifest.txt · SRC-0120](#src-0120) (manifest SRC-0120) | 2026-10-22T13:14:09Z | `8d0b47da979e809b9f5460a6827534e81e4532bc` / CLEAN | `1688695e7132580f3984e9aef84817d4889a25a7249a0ccd8b9f4bc5e591bfc7` | 1594816 / 1594816 | MATCH |
| [matdog_build_manifest.txt · SRC-0121](#src-0121) (manifest SRC-0121) | 2026-10-22T19:49:11Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / CLEAN | `e244f9b8e1d3d2be10defc348912b947cf7676b56142753e1a82b61d8d00d2ea` | 1598400 / 1598400 | MATCH |
| [matdog_build_manifest.txt · SRC-0122](#src-0122) (manifest SRC-0122) | 2026-10-22T11:05:35Z | `46123741fcfc4fc393f48752f3fe156fb3e94369` / CLEAN | `018fcf294819e1fb2b8ca5fe3cbbe8b2836c7e05056b9906822d3e4131269f5d` | 1603360 / 1603360 | MATCH |
| [matdog_build_manifest.txt · SRC-0123](#src-0123) (manifest SRC-0123) | 2026-10-23T07:38:29Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `ad287f708bac5771521e6bfbb4d706e636c8641bdf5aa71191af335002ce120a` | 1601696 / 1601696 | MATCH |
| [matdog_build_manifest.txt · SRC-0124](#src-0124) (manifest SRC-0124) | 2026-10-23T07:04:58Z | `f838f677a1826c4eaabc5889473ef016b1ebad98` / DIRTY | `1e30228076d6ae251120adbb504bb95805fd7fe57af17037c7ea5cd9694f79d8` | 1601328 / 1601328 | MATCH |
| [matdog_build_manifest.txt · SRC-0125](#src-0125) (manifest SRC-0125) | 2026-10-15T21:07:02Z | `9bbfe01eab31fe423179e2f3ee505919bd761013` / DIRTY | `754d798d189c2da339d7f982dc657e4ae5de96b82f6a5e506583825c3b8eab1f` | 3999408 / 3999408 | MATCH |
| [matdog_build_manifest.txt · SRC-0126](#src-0126) (manifest SRC-0126) | 2026-10-20T22:32:20Z | `ac37c15819d9f4031acba548d7ee6e96d05c6215` / CLEAN | `5ad7ac082bd5dfd09fc8438d1da1737822329224c3a23d6775adc54620147bcc` | 3936608 / 3936608 | MATCH |
| [matdog_build_manifest.txt · SRC-0127](#src-0127) (manifest SRC-0127) | 2026-10-21T06:47:49Z | `9eeef0ddd74005522ee6766e6839df295aa26c49` / CLEAN | `26be67499cf886985cd798e835dca7b2949882be1f0a945aa6bd6cabb42f4723` | 3937680 / 3937680 | MATCH |
| [matdog_build_manifest.txt · SRC-0128](#src-0128) (manifest SRC-0128) | 2026-10-21T05:50:37Z | `43b018f7a1247a49442644d7fb4f868c5714ede2` / CLEAN | `8b7a45c58aa1d95ca6ea8d67c8a409c226dc41baba174cbd5d3d739fef2d4875` | 3937584 / 3937584 | MATCH |
| [matdog_build_manifest.txt · SRC-0129](#src-0129) (manifest SRC-0129) | 2026-10-20T17:12:08Z | `9d7eadbbf296538d2ee2883354bac81e2a84109a` / DIRTY | `4d109c697e8a40f3f3e14533ce4ae205026c6609a6fc58d3d95295ae3ed3526a` | 3927552 / 3927552 | MATCH |
| [matdog_build_manifest.txt · SRC-0130](#src-0130) (manifest SRC-0130) | 2026-10-21T15:57:52Z | `2f7b81822d708444db910bfe0f179dcc41be1b2e` / CLEAN | `4cec9a11ecfd7dafc1784c311810505de9635c6b32cccfbd37cda0052b1e5b7a` | 3944496 / 3944496 | MATCH |
| [matdog_build_manifest.txt · SRC-0131](#src-0131) (manifest SRC-0131) | 2026-10-21T18:32:55Z | `530e6f4172235bb86de08fe852e6b5ddef9a9b99` / CLEAN | `cb69ed4b6305ef28b56edd753b7452c934286e0a3637b024e6d3f940a6eba73d` | 3944688 / 3944688 | MATCH |
| [matdog_build_manifest.txt · SRC-0132](#src-0132) (manifest SRC-0132) | 2026-10-21T19:36:18Z | `ad0a0910124cb849056f872c83594916902dfe24` / CLEAN | `7ada67c82992c16326e1cc039c5a70ffb98761d350b51bdab0ebe469981144a2` | 3944800 / 3944800 | MATCH |
| [candidate-build-manifest.txt · SRC-0133](#src-0133) (manifest SRC-0133) | 2026-10-18T20:56:21Z | `16ca51eb3037754782260718b27d6a1dc06ebe2f` / CLEAN | `e4f71b8ec2cb410d9bfed91a9fe16fafaf8dd36ad77cb4921087a2fd545e5635` | 3912944 / UNKNOWN | UNKNOWN |
| [candidate-build-manifest.txt · SRC-0134](#src-0134) (manifest SRC-0134) | 2026-10-19T05:53:44Z | `a34d528eea43b670f919132d60e4f24f37a45898` / CLEAN | `b5199a178f54f3667a165fd023f75463db22eb8ebdc75742a213d02b6b5fbf15` | 3913424 / UNKNOWN | UNKNOWN |
| [candidate-build-manifest.txt · SRC-0135](#src-0135) (manifest SRC-0135) | 2026-10-19T11:14:27Z | `b5be64c4e7657b3b8f02deb312c6a2f0dc385812` / CLEAN | `8afd56037a415ff72d0c56a320f9e1b3671d2dab03b9a9ad5a46b7c0d1be87e7` | 3913360 / UNKNOWN | UNKNOWN |
| [matdog_build_manifest.txt · SRC-0136](#src-0136) (manifest SRC-0136) | 2026-10-19T17:41:02Z | `eac9fc00e3744b711880b46a0fa55f72e5fd563d` / CLEAN | `1c49818545340314c675f335a5689e41c36146a0f11a075976d2ffde5248e21e` | 3915296 / 3915296 | MATCH |
| [matdog_build_manifest.txt · SRC-0044](#src-0044) (manifest SRC-0044) | 2026-10-19T18:58:24Z | `d84b65df755cd00afddc2d7278e48e6d1e722fb7` / CLEAN | `449e3e9260039920bff243f2c0c2e428df1da1d57da4848da5f78db4bd6e3082` | 3916432 / 3916432 | MATCH |
| [matdog_build_manifest.txt · SRC-0137](#src-0137) (manifest SRC-0137) | 2026-10-20T14:04:40Z | `5d320795d69a6e76492deb81bf212a95498cfddb` / CLEAN | `d909dc10cc864bd96ed5c65a9595e222177de60ec84918bfd4f5ee14bba8e752` | 3920800 / 3920800 | MATCH |
| [matdog_build_manifest.txt · SRC-0138](#src-0138) (manifest SRC-0138) | 2026-10-20T14:54:17Z | `b7bc4336d5c25845305bcc39be90e07615fcd3fb` / CLEAN | `d8bfb5f55efeece060641f8d818e7d75405cfb94136db5f4b8de68edda2dbaf5` | 3920880 / 3920880 | MATCH |
| [matdog_build_manifest.txt · SRC-0139](#src-0139) (manifest SRC-0139) | 2026-10-20T15:53:43Z | `9d7eadbbf296538d2ee2883354bac81e2a84109a` / CLEAN | `261e173d5864377e1bcab9c00b66c56fd669a58783780044c704cec9c94b474b` | 3922320 / 3922320 | MATCH |

### Provenienza DIRTY da conservare con i binari

Per RC4-r2 il dossier conserva `source_sha256_at_build.txt` e diff; per RC4 successive i receipt/inventari dei dossier preenable/control/torque identificano gli input. RC5 Night: `TASK_ONLY.patch`, checkpoint e208 input; reference_guard: `baseline/preexisting.patch`, `REVIEW_ONLY.patch`,206 sorgenti compilatore; Network: `dirty-before.patch`, `NETWORK_ONLY.patch`, `REVIEW.patch`; finale: `CANCELLATION_ONLY.patch`, `source-final.json` e210 input. Sono fotografie differenti su HEAD f838f677. Il suffisso “dirty” da solo non permette riproduzione: servono inventario e contenuti salvati insieme.

Le directory timing-only non adottano tutte lo schema artifacts/PROFILO: APP d15 è attestata dal report hardware e dal suo receipt specifico, non omessa perché fuori dalla tabella. Prima timing build e build “final”23/10 interrotta restano artefatti diagnostici/superati; non si attribuisce loro il digest della build successiva.

## 4. Catena delle criticità e delle decisioni

| Prima divergenza verificabile | Causa dimostrata / ipotesi | Patch successiva | Confine del risultato |
|---|---|---|---|
|18/09 IMU 0,68 Hz con CDC chiusa|HWCDC connected latched eTX bloccante nel core installato|e2fc605 timeoutTX0|50,10 Hz a porta chiusa; non qualifica tutto Motion|
|19/09 MOS OFF ma rail ON|Bypass B−/P− tramite buck/masse, resoconto elettrico|Ricablaggio VIN− → P−, closeout24/09|KEY OFF rail 0 senza caricatore; caricatore diverso|
|26/09 DHCP ma ARPFAILED|Link RF classificato debole; causa completa non misurata|DiagnosiPHY/Wi-Fi successive|CONNECTED non prova HTTP|
|01/10 calibrazione 24/24 persa al reboot|RAM-only, persistenza assente|RecordNVS eDEV3 gen1|Restore operativo verificato successivamente, non al01/10|
|06/10 termica LF dopo bulk refutati|Latch su spike confutati|DEV3 direct-only maggioranza|70°C invariati; Motion singolo diretto è implementazione diversa|
|10/10 error 11/config password mismatch|Eventi softAP stop/start non armati e ACK ambiguo|837f523, poi b083 STA_STOP|Driver/key MATCH non prova associazione telefono|
|16/10 STALE prima attuazione|Gate freshness/readiness, nesso completoUNKNOWN|Realtime/commissioning successivi|Non difetto cinematico|
|17/10 ASSESSMENT_STALE|Soglia6 mrad/microframe incompatibile con residui osservati|Tolleranza/polling18–19|Non giustifica eliminare limiti fisici|
|19/10 a34 STALE 100 ms, b5 TRACKING_LAG|Polling/tracking da distinguere, campioni incompleti|Unifiedd84|Nessun salto a STAND completo|
|19/10 d84ACCELERATION_LIMIT|Clock misto riprodotto; causa unica runHYPOTHESIS|V2 clock → V3 TX → V4 quote|218cumulativo; mancano operandi causali|
|21/10 V6 STALE13,6 s pre-ARM|Calcolo numerico sincrono12 chiamate|530e6f4/ad0a091|Nessuna esecuzioneIK/retimer nel guasto|
|21/10 ad0a091 PATH_BOX REAR|Errore osservato20–22 mrad, protezione applicata|Decisione SIMPLE 22/10|ACC20 / carico / attrito / TX non discriminati|
|22/10 RC2 margini geometrici falliti|Corde/via insufficienti nel calcolo offline|RC3via e regioni separate|NOT_PASS non è collisione avvenuta|
|23/10 STAND/STOP400|Contratto dashboard non allineato|P1 eSTOP idempotente P1.1/RC4|Offline, non latenza fisica|
|24/10 pendingcontrol persistente|Timer per ID contro pending per ID/width|7fd0, poi cdcef|Recupero specifico; non spiega da soloREFERENCE|
|25/10 ID13torque 1/delta 0 prima enable|QC side effect profilatura; applicazione alSYNC del run inferita|942171OFFreference/suffisso|Non catturaTX/RX del 25/10|
|25/10 loadSTALE/BMS/miss|Ritardo/gate temporale e starvation riprodotti condizionalmente|520cache/freshness/RX; d15misura; analitico; RC5async|TempiESP e predicato di ogni faultUNKNOWN|
|26/10 HTTP 503/timeout durante STAND|Freeze dello snapshot e mailbox scaduta dimostrati software; blackout completoUNKNOWN|bac0poi fe33TTL/cancel/clock|Nessuna validazione fisica rete patchata|

## 5. Architetture, riuso e lezioni tecniche

| Implementazione / risultato | Stato di riuso al 26/10 | Lezione e limite |
|---|---|---|
|Controller unico ESP32-S3 eownerUART|KEEP|Intenzioni semantiche da dashboard; niente autorità attuativa al browser|
|G3 / LED / DALY / BNO085|KEEP nel dominio di prova|Non riscrivere periferiche per correggere il Motion; radio e IMU sucore0 possono ancora contendere risorse|
|NVS gen1, unità e mapping|KEEP|Fingerprint be8d5902bb4e37da, record schema 1 / marker schema 2 non sono nuove generazioni; calibrazione RAM01/10 diversa dalla persistita06/10|
|Resolver joint↔RAW|KEEP|raw = q0+encoder_direction×ticks(q), arrotondamenti/limiti del resolver; niente wrap, q0 = 2048 o motorDirection = encoder_direction assunto|
|Mapping|KEEP|LF 13/12/11, RF 23/22/21, RH 33/32/31, LH 43/42/41 per HIP/UPPER/LOWER; ID51 incluso in SAFE_OFF, escluso dai 12 joint Motion|
|Calibratore nativo 24 contatti|KEEP misure; ADAPT motore diagnostico|Goal discreti e feedback ≠ IK/FK continui50 Hz; suo successo non prova prestazioni Motion|
|Termica a maggioranza DEV3|KEEP calibratore; ADAPT Motion|Checksum valido non basta a rendere temperatura bulk fisica; conferma diretta e guasto reale restano necessari|
|V4/V5/V6 executor, retimer ePATH_BOX temporale|SUPERSEDED|Clock/TX/readiness/STOP producono lezioni riutilizzabili; non reintrodurre pipeline complessa come vincolo universale SIMPLE|
|GeometriaV5/FK/IK/regioni|KEEP/ADAPT|URDF pin e certificati legati a mesh, route e margini; IMU non misura quota o forza al piede; compensa modello, non dimostra equilibrio|
|SIMPLE RC1/RC2|SUPERSEDED|Semplicità utile; scorciatoie geometricheRC2 fallisconooffline e sono corrette in RC3|
|RC3 restore/via|KEEP; APP superata|Restore12/12 fisico, percorsoMotion non qualificato nella sessione flash|
|RC4 busRX eREFERENCE|KEEP fix; APP superate|TX-complete non è applicazione servo; GoalPosition può armare torque; readbackOFF/ON distinto dalla cache|
|RC5async/analitico/reference_guard|KEEP con qualifica pertinente|Ottimizzazioni non trasformano replay ideali in tracking delrobot; torque cache datata distinta daSAFE_OFF postSTOP|
|RC5 rete bac0|SUPERSEDED|TestPASS iniziali non coprivano race reader; conservare negativo ebuild per la decisione|
|RC5 finale fe33|CANDIDATA / OFFLINE|Nessuna fairness senza slack, nessuna latenzaSTOP radio garantita; misura hardware pendente|
|STOP/SAFE_OFF|KEEP protezioni; misureHW non trasferibili|Latch prioritario→nessun nuovo target→broadcastOFF→readback 13/13. ACKHTTP non provaOFF; collo51 compreso.31–32ms V6 non è garanziaRC5|
|PRIME-only, compileSTAND-only, heartbeatbrowser|DIAGNOSTIC/SUPERSEDED|Utili come strumenti storici; rimossi come rituali normali. Sessioneauth controlla accesso, non un deadman universale del moto|
|Timing-onlyd15|DIAGNOSTIC|Record RAM inclusivo, stampa dopo la safety; non integrare numeri annidati sommando due volte né usare come firmwareoperativo|

**Precedente C4-C fuori periodo:** l’addendum23/10 conserva un successo fisico dichiarato alcommit58b97392dc13efa30e166fe3802efa677dcba011; artefatto51campioni100→150mm è offline_only ecommand_eligiblefalse. Mancano start/end12 joint e quota misurata. Non negare la dichiarazione, non promuoverla a qualifica RC5; C5tick storici e la sua collisione RF sono altra implementazione superata.

## 6. Stato finale e limiti operativi storici

Ultima installazione documentata54ade4; fe33fab candidata ancora offline. Gen1/restore e comunicazioni periferiche hanno prove fisiche precedenti ebootRC5; disponibilità sotto movimento, riuso delle regioni per risposta dinamica reale, quota0–150mm e WALK STANDARD restano da qualificare. Non reperita camminata continua riuscita. FAST non qualificato.

Non si prescrive qui un flash o una nuova campagna. I runbook delle fonti contengono autorizzazioni e condizioni della loro sessione storica: non sono istruzioni eseguite per produrre questo diario. Le lacune documentali non sono nuovi gate firmware e non giustificano ripetere scritture/calibrazioni storiche.

## 7. Indice tracciabile delle fonti e controllo documentale

I riferimenti delle schede e dei manifest puntano agli ID `SRC-####` indicizzati qui sotto. Le ubicazioni fisiche locali non sono pubblicate e gli ID non promettono che gli artefatti siano presenti nel repository GitHub. Gli hash sotto congelano solo le fonti direttamente collegate, non tutto il contenuto delle directory. I report sono fonti primarie di sessione quando contengono risultati/esiti, ma dove manca la catturaRAW il grado è HW-REPORT. Addendum23/10 usato come indice, V1 come lista di ipotesi/lacune; nessuna conversazione non disponibile viene inventata.

- [1-MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V1.md · SRC-0140](#src-0140)
- [MATDOG_LOCAL_DOCUMENTATION_CLEANUP_REPORT.md · SRC-0141](#src-0141)
- [MATDOG_MOTION_CORE_REUSE_ADDENDUM_2026-10-23.md · SRC-0142](#src-0142)

| Identificativo fonte / nome logico (senza percorso locale) | SHA256 ricalcolato nella V2 originale |
|---|---|
| <a id="src-0098"></a> **SRC-0098** · Build matdog-analytic-jac-20261025_211944 · matdog_build_manifest.txt | `e1ed9cb694b02f35c2ec8f9a00249697059adba4533ec60edc2b51f1e6c2c12f` |
| <a id="src-0099"></a> **SRC-0099** · Build matdog-controller-1.0.0-rc.4-20261023 · matdog_build_manifest.txt | `bf6587c58d4169477bd349f5e3f785f7002122c47fac8bab9b1213839a20b377` |
| <a id="src-0063"></a> **SRC-0063** · Build matdog-controller-1.0.0-rc.4-final-20261023.build.log | `03ee17cae3b5795c7da96d4d73761ddbc4d92cef563f2005fe2cb059a4509c8e` |
| <a id="src-0100"></a> **SRC-0100** · Build matdog-controller-1.0.0-rc.4-r2-20261023 · matdog_build_manifest.txt | `c3f26781b5dbe4ef441598c68a5006db46e45ee3734491db151d456327a78e2f` |
| <a id="src-0101"></a> **SRC-0101** · Build matdog-controller-1.0.0-rc.4-r3-20261024 · matdog_build_manifest.txt | `d277f5b3e32f78cefd214528b370a6004c4f34442b864111b1df7281b9a342b7` |
| <a id="src-0102"></a> **SRC-0102** · Build matdog-controller-1.0.0-rc.4-r4-20261024 · matdog_build_manifest.txt | `844fea421bc963fa8fcfb55fd97de7de62937091f94f8f5735883451494ca506` |
| <a id="src-0103"></a> **SRC-0103** · Build matdog-controller-rc4-control-pending-20261024 · matdog_build_manifest.txt | `3eadce7905b62c816aa19a5175daac1a03c1d3010505df16bc3868a20f7e8205` |
| <a id="src-0104"></a> **SRC-0104** · Build matdog-controller-rc4-final-codex-20261024 · matdog_build_manifest.txt | `70a5712cfa61a853f3fbdc2f8fe5d3d837cfb5e8975a0e161bf42bea8c1e9328` |
| <a id="src-0105"></a> **SRC-0105** · Build matdog-controller-rc4-motion-corrective-20261025 · matdog_build_manifest.txt | `cdd9ddaa1e080a4abcc5c13d267378872cdb0b4b8ec16ee4e36a27ff0f8a2e0f` |
| <a id="src-0106"></a> **SRC-0106** · Build matdog-controller-rc4-preenable-final-20261025 · matdog_build_manifest.txt | `861a9d295ed41f325e55ed469ab0b41f4f3b889100f1fc5d38a4c5620387caf7` |
| <a id="src-0107"></a> **SRC-0107** · Build matdog-controller-rc4-preenable-fix-20261024 · matdog_build_manifest.txt | `2542190fe75eac8148f7e156e7445e874a56dc2666bf8470174ddc633c6e3927` |
| <a id="src-0108"></a> **SRC-0108** · Build matdog-controller-rc4-rx-recovery-codex-20261024T122219Z · matdog_build_manifest.txt | `0b7f5d8682356e8e3a0c56a2456640d55d97eaebf193a613749766e8c3431a95` |
| <a id="src-0109"></a> **SRC-0109** · Build matdog-controller-rc4-stand-torque-20261025 · matdog_build_manifest.txt | `1d925415f12f83195ac42e11c423ab7a9594b12ad25dedaf5c73525e5c628052` |
| <a id="src-0110"></a> **SRC-0110** · Build matdog-night-recovery-20261026 · matdog_build_manifest.txt | `1543362aa2f54038c8e2cfb56cd72b86e5a1a16b71b7b8bf361a0d3cb51027f2` |
| <a id="src-0111"></a> **SRC-0111** · Build matdog-rc5-final-review-20261026 · matdog_build_manifest.txt | `89713efd77a8123a22c83107802bc4c56a21b227fe25b98040742de3589797a0` |
| <a id="src-0112"></a> **SRC-0112** · Build matdog-rc5-network-recovery-20261026 · matdog_build_manifest.txt | `8c78e2508f3d31a35f8ad029e71e4cf73422a961ff128e945daf23c4df2f06d9` |
| <a id="src-0113"></a> **SRC-0113** · Build matdog-rc5-pre-hardware-20261026 · matdog_build_manifest.txt | `0edcc3be243cf09b98913ff7e0523fb3c05c18ab6da106d2ec951b30c8f6ee6b` |
| <a id="src-0114"></a> **SRC-0114** · Build realtime-20261015-powered · matdog_build_manifest.txt | `2bc0115d9fad04a8095f5cb9c6a1ef6f57306a8f8089cce0e3d60517ead32cf5` |
| <a id="src-0115"></a> **SRC-0115** · Build realtime-20261015-usb · matdog_build_manifest.txt | `2c266ac9db6678582ea3903d1653b17d28fcc0fc21a46c992296fed3e99998b7` |
| <a id="src-0116"></a> **SRC-0116** · Build realtime-20261015b-powered · matdog_build_manifest.txt | `7246b757a8b0bf360d2697889ffbae15fc27ae11e05a4ac81e6822ff7525fa3e` |
| <a id="src-0117"></a> **SRC-0117** · Build realtime-20261015b-usb · matdog_build_manifest.txt | `f73ead1429a6feb19a4c49e8971aefd5ff81c4cbc44d02e75b5e01de7260668b` |
| <a id="src-0118"></a> **SRC-0118** · Build realtime-20261015c-powered · matdog_build_manifest.txt | `63f45f568d1718e4c77e897722b1bdd87867bfcfaad4b0d9a57ef4fafcb170cc` |
| <a id="src-0119"></a> **SRC-0119** · Build realtime-20261015c-usb · matdog_build_manifest.txt | `a180074a5cdebf304d838c064068dd0faad9d3ce2f7cc7046f9b03a2fe51cba1` |
| <a id="src-0120"></a> **SRC-0120** · Build simple-motion-opt-v1 · matdog_build_manifest.txt | `3278a7822888fab3b4054874d05d826ff6ebe1c4d09dff5a1577cf88244aaf5a` |
| <a id="src-0121"></a> **SRC-0121** · Build simple-motion-traj-v2 · matdog_build_manifest.txt | `9665887110df49889ddf3b4cac5d3ccc10a984496989090fcc08a719115cd477` |
| <a id="src-0122"></a> **SRC-0122** · Build simple-motion-v1 · matdog_build_manifest.txt | `15e5f8e3a9c4a4a3b033120d3be78485b7b996632e04f76e45be1954304d1c8a` |
| <a id="src-0123"></a> **SRC-0123** · Build simple-rc3-p1-1-stop-lane-20261023 · matdog_build_manifest.txt | `b02d899dd9c001773e598e9dae0a9e17ce27a4056c9dae1555ce4e4afd913999` |
| <a id="src-0124"></a> **SRC-0124** · Build simple-rc3-p1-dashboard-20261023 · matdog_build_manifest.txt | `0c67c0d0dc07de64fe8ade7758d7fcdcdaa6f19b33a1bded275f35b08c19ffdc` |
| <a id="src-0125"></a> **SRC-0125** · Build stackfix-20261016 · matdog_build_manifest.txt | `caeb99c302b37c399f309d795ff33dea69d18d3ae735c02808883b8283ebb52a` |
| <a id="src-0126"></a> **SRC-0126** · Build v5-candidate-20261021 · matdog_build_manifest.txt | `aac2abfc8c7addcb23ee5e81cd846b1cf8661a9ed9e89cfa53b31d75ac7b1fc1` |
| <a id="src-0127"></a> **SRC-0127** · Build v5-imu-fix-20261021 · matdog_build_manifest.txt | `c7e62b17981d9936504a243d07b163887e91c0ae8b8b979ec44fdf376ec74eef` |
| <a id="src-0128"></a> **SRC-0128** · Build v5-p0-fixes-20261021 · matdog_build_manifest.txt | `0f24276760a7d412be3e21b6be59eede532ad39cbeb4a448ae6c9bc3399fc7f0` |
| <a id="src-0129"></a> **SRC-0129** · Build v5-trial-20261020 · matdog_build_manifest.txt | `68069e712491e21181ef425bbcb5aa7f240cb8588174cc67f1345974167df4cd` |
| <a id="src-0130"></a> **SRC-0130** · Build v6-20261021 · matdog_build_manifest.txt | `1df3a1123c94d2a578e82b62fb43f1dbba3efcf8f6491df9403e700155d8baad` |
| <a id="src-0131"></a> **SRC-0131** · Build v6-rca-530e6f4-20261021 · matdog_build_manifest.txt | `1432b84a14b00294fa30e8d6253a465f882b88d0037cde2c76e3a0ee96598980` |
| <a id="src-0132"></a> **SRC-0132** · Build v6-rca-ad0a091-20261021 · matdog_build_manifest.txt | `2c1ced29d993a1ce608b616c3a459b9c08a82df39f48996e964f3c44d7061104` |
| <a id="src-0141"></a> **SRC-0141** · Handoff MATDOG_LOCAL_DOCUMENTATION_REVIEW_20261023 · MATDOG_LOCAL_DOCUMENTATION_CLEANUP_REPORT.md | `f5fc3fd9a5d6239a1dfa2c6d9f425e0e363f5507ed4b04a458c20901a33c8e13` |
| <a id="src-0142"></a> **SRC-0142** · Handoff MATDOG_LOCAL_DOCUMENTATION_REVIEW_20261023 · MATDOG_MOTION_CORE_REUSE_ADDENDUM_2026-10-23.md | `d7bf1af178510f40343f9efd9ac4b46c3ff21e78f101ce72246ad3694357218b` |
| <a id="src-0024"></a> **SRC-0024** · Dossier AP403_OFFLINE_20261009 · AP_ROOT_CAUSE_FINAL.md | `441fd2ec9858c7fd83743b69194f4bc4042187a0609d115ff8923f2689d439b6` |
| <a id="src-0039"></a> **SRC-0039** · Dossier APP_ONLY_16ca51eb_20261019T041613Z · REPORT_FINALE.txt | `becc85d3b2da523c7969bef5d4de0674ec772a8efc91ac88ee377c99d25f2f93` |
| <a id="src-0133"></a> **SRC-0133** · Dossier APP_ONLY_16ca51eb_20261019T041613Z · candidate-build-manifest.txt | `9294b7cee1ee508ecae90444f210cb9e4787a08e3027d10027596f49886c6c65` |
| <a id="src-0040"></a> **SRC-0040** · Dossier APP_ONLY_a34d528_20261019T102758Z · REPORT_FINALE.txt | `2e8bad2c2bd5e267f779440a5e60ef0635df8aabc5999a7ed2e9d4ef2e469c30` |
| <a id="src-0134"></a> **SRC-0134** · Dossier APP_ONLY_a34d528_20261019T102758Z · candidate-build-manifest.txt | `ed231fd83cf05d02deb8ee1983795b0954afe0f73a62eff89fe4b2a17258039f` |
| <a id="src-0042"></a> **SRC-0042** · Dossier APP_ONLY_b5be64c4_20261019T153441Z · REPORT_FINALE.txt | `323a317f5a4576c9121892961cf12fb58eee2a413eae620e7758530d005e0cd6` |
| <a id="src-0135"></a> **SRC-0135** · Dossier APP_ONLY_b5be64c4_20261019T153441Z · candidate-build-manifest.txt | `ca366fa33e4613aa8a475fc166893eed78624b07777e6187567e9c059a9603cd` |
| <a id="src-0082"></a> **SRC-0082** · Dossier FLASH_ANALYTIC_JAC_FINAL_20261025_214602 · ota-verification.log | `4725f00c42a0d874547b740416a0afb381e444c946bbc149712aeba35f207c2f` |
| <a id="src-0083"></a> **SRC-0083** · Dossier FLASH_ANALYTIC_JAC_FINAL_20261025_214919 · flash.log | `e42eb6d4d4d15c680f6fbfe0487829cf83668aacb00e76b7ec0dbf868cbc9925` |
| <a id="src-0084"></a> **SRC-0084** · Dossier FLASH_ANALYTIC_JAC_FINAL_20261025_214919 · verify.log | `391fe57cdd4329e63aa524e0fc12cb334905bed5209d1a9a2a8fdc398d0e1fc6` |
| <a id="src-0087"></a> **SRC-0087** · Dossier FLASH_RC5_FINAL_20261026_071644 · FLASH_AND_BOOT_REPORT.md | `88f9ad204d95b031025beb9e97dd4715ea23b8dfde5bc61245022bc9a0ec5ddc` |
| <a id="src-0064"></a> **SRC-0064** · Dossier MATDOG_CONTROLLER_RC4_FINAL_CODEX_20261024 · REPORT.md | `4156c669be59b8e476923c0d01f2fb07976d70dbfd3239800fe0c1c7d3695b16` |
| <a id="src-0070"></a> **SRC-0070** · Dossier MATDOG_CONTROL_PENDING_RCA_20261024 · REPORT.md | `51b1cdbb61b44a3965e875c2cad932f2ddae8ec9f32c18082755b34b1fce6415` |
| <a id="src-0032"></a> **SRC-0032** · Dossier MATDOG_HW_FAIL_AUDIT_20261014 · FINAL_REPORT_OPERATIONAL_ARCHITECTURE.md | `e27f481f749580b5a5cdaaddd757e7c2d35ee7ffee5cb89fb413f974b9efeefb` |
| <a id="src-0016"></a> **SRC-0016** · Dossier MATDOG_M0_3B_USB_ONLY_LEGACY_2757604_20261003 · MATDOG_M0_3B_RECOVERY_USB_ONLY_LEGACY_REPORT.md | `9ce071b7515623f2b894686ac35a8eb49a698c322d9aa83d2e547f47fe75db32` |
| <a id="src-0015"></a> **SRC-0015** · Dossier MATDOG_M0_4_ROBOT_POWERED_2757604_20261003 · REPORT.md | `811d89c8a0ad4b2d9000ef34725c0c124d9cf294b7519d36876e6a65cf70ae50` |
| <a id="src-0077"></a> **SRC-0077** · Dossier MATDOG_MOTION_CORRECTIVE_RCA_20261025 · REPORT.md | `39f5375a6b219f586000274201d20a4fb96110511b1275e7ae979602a02d95c7` |
| <a id="src-0078"></a> **SRC-0078** · Dossier MATDOG_MOTION_CORRECTIVE_RCA_20261025 · RESUME.md | `f3edda666da37e71a82a9472853c1f71f2ba0a0259afa14169290c2ea69bd97d` |
| <a id="src-0076"></a> **SRC-0076** · Dossier MATDOG_MOTION_LOAD_RCA_20261025 · REPORT.md | `71fd475b31df254d7a337550800367d26f6f77a2b75531ae25be2a40df61bd41` |
| <a id="src-0086"></a> **SRC-0086** · Dossier MATDOG_NIGHT_RECOVERY_20261026 · FINAL_RECEIPT.json | `8b262a4bc47119ab95762fbfc284fa26ef39745dd7ff62939bcc9ed7ab5a7a23` |
| <a id="src-0085"></a> **SRC-0085** · Dossier MATDOG_NIGHT_RECOVERY_20261026 · HANDOFF.md | `e9e787c1c16b2c1cad203b144a3b601fc48cf80aad86bfd49887762646c63854` |
| <a id="src-0071"></a> **SRC-0071** · Dossier MATDOG_PREENABLE_FINAL_20261025 · REPORT.md | `c5cd0c2bbcc1633d3b8ad95a254991360828374fce4516297e6d574ad7e6c37d` |
| <a id="src-0072"></a> **SRC-0072** · Dossier MATDOG_PREENABLE_FINAL_FLASH_20261025T061733Z · REPORT_FLASH_BOOT.md | `2a20a77947a01849d5c291e7e38fb9a5178b7d822f19db1fd2592ab818ad2cf4` |
| <a id="src-0065"></a> **SRC-0065** · Dossier MATDOG_RC4_APP_ONLY_20261024T103731Z · REPORT_FLASH_BOOT.md | `1f66d0509ccadaeb73e340c690c8a93a0f01256390f8f1aeff5f7dc0cffbeb41` |
| <a id="src-0067"></a> **SRC-0067** · Dossier MATDOG_RC4_RX_RECOVERY_FLASH_20261024T125454Z · REPORT_FLASH_OBSERVATION.md | `a63ea4a56ef5a173cd7b46db0db59d746e7e2660685a69f867778c441ec041d0` |
| <a id="src-0088"></a> **SRC-0088** · Dossier MATDOG_RC5_FINAL_REVIEW_20261026 · FINAL_REVIEW.md | `819d52050df3d208b6cf620e93ea37e893e2486afe555d3cd0cd068c50c3cfb9` |
| <a id="src-0093"></a> **SRC-0093** · Dossier MATDOG_RC5_NETWORK_RECOVERY_20261026 · NETWORK_ONLY.patch | `b73903ffbe34e6f2cc66c3f5a78c43e137faca60034e461330f04b4db020aad8` |
| <a id="src-0092"></a> **SRC-0092** · Dossier MATDOG_RC5_NETWORK_RECOVERY_20261026 · REPORT.md | `20398456383659bd9f5fc19cc8cda8c9d3bb1e882982a7b4b98eacb6ac24d6b2` |
| <a id="src-0089"></a> **SRC-0089** · Dossier MATDOG_RC5_NETWORK_RECOVERY_20261026 · hardware-evidence-summary.json | `95776bc6a153a275c1d447c2048dd0444b5f67f5e6611c80196696557cadf140` |
| <a id="src-0096"></a> **SRC-0096** · Dossier MATDOG_RC5_PRE_HARDWARE_20261026 · CANCELLATION_ONLY.patch | `adbfd4b049d185a19ff900d178afaeb55f64d8ff1f55b1e1a2e7d6cc24da6ccf` |
| <a id="src-0095"></a> **SRC-0095** · Dossier MATDOG_RC5_PRE_HARDWARE_20261026 · RECEIPT.json | `2e17059d7bf7cfd20240a46d49b866b8e9c678a7804091652f1124b6d9b065c4` |
| <a id="src-0094"></a> **SRC-0094** · Dossier MATDOG_RC5_PRE_HARDWARE_20261026 · RESULT.md | `b367dd6355cab599399b89bb07981bf1ed287e43b0154d2673457bbb15146de9` |
| <a id="src-0097"></a> **SRC-0097** · Dossier MATDOG_RC5_PRE_HARDWARE_20261026 · source-final.json | `6e8b4c50a640681cabe01888ded4df156fe00a9d3258d160a8125eb2148c1048` |
| <a id="src-0068"></a> **SRC-0068** · Dossier MATDOG_STAND_PREENABLE_RCA_20261024 · REPORT.md | `c25f7251a1d56ebb8260137cd88c4b48e4f2b117787f407077457647f0b8c7c2` |
| <a id="src-0074"></a> **SRC-0074** · Dossier MATDOG_STAND_TORQUE_FIX_20261025 · REPORT.md | `79df785b1e1bb479f918172b91e3b6fdb128c426a2ac7a229ef3de496e247c65` |
| <a id="src-0073"></a> **SRC-0073** · Dossier MATDOG_STAND_TORQUE_FIX_20261025 · hardware-original.log | `c21359f0eba9214505a7bd54c444eebe9701ad3e3c39a20efd47892a69265311` |
| <a id="src-0075"></a> **SRC-0075** · Dossier MATDOG_STAND_TORQUE_FLASH_20261025T072853Z · REPORT_FLASH_BOOT.md | `2727174f3cc4bdce4bf621d6a775d0e72e535c29790bfa296344ca24761a711d` |
| <a id="src-0079"></a> **SRC-0079** · Dossier MATDOG_TIMING_FLASH_20261025T174600Z · REPORT.md | `db10c3636622e516dfaa2022442e12913955ed3b3ca0b7af7598f37c132a46e5` |
| <a id="src-0019"></a> **SRC-0019** · Dossier MATDOG_V0_2_DEV1_CONVERGENCE_20261005 · CONVERGENCE_RECEIPT.md | `e24c19aaa1b38ec8ec697113e8c548ce76619e8032a9c8d11701e7ec93c30dc2` |
| <a id="src-0018"></a> **SRC-0018** · Dossier MATDOG_V0_2_DEV1_HW_VALIDATION_PREP_20261006 · FROZEN_CANDIDATE_VERIFICATION.md | `a59242588463de5a89d4186fd062b65ee6afee7c6f601cedc40753bd8f5ec07a` |
| <a id="src-0021"></a> **SRC-0021** · Dossier MATDOG_V0_2_DEV2_BOOT_SELFTEST_20261006 · FAILURE_STOP_RECEIPT.md | `46a04239a59fde217585c9a698bb965b882a5cfb0c0d3e9b3afba2556e9b07ad` |
| <a id="src-0023"></a> **SRC-0023** · Dossier MATDOG_V0_2_DEV3_FINAL_HW_VALIDATION_20261007 · MATDOG_V0_2_DEV3_FINAL_HW_VALIDATION_REPORT_2026-10-07.md | `93c1e6eea075de61dce800d9a7719d3675a439cb12fafb39ec542048de9545e0` |
| <a id="src-0026"></a> **SRC-0026** · Dossier MATDOG_WPA2_PATCH_FLASH_20261010 · BINARY_MANIFEST.md | `fe8f91f0bf963cb18bdc26073a0c6559cf847c0bc10cc0a9b2d0d0fa8ad04884` |
| <a id="src-0025"></a> **SRC-0025** · Dossier MATDOG_WPA2_PATCH_FLASH_20261010 · RAPPORTO.md | `b1e7478c078a5854704b1ff3a5d691ac39737d61d50fd379da2a1b868b054279` |
| <a id="src-0051"></a> **SRC-0051** · Dossier MOTION_CORE_V5_20261020 · RCA_STAND_20261021_PHASE1.md | `8fabe0af6933700ff2b26bb5845521974fb0fd95570d2602553f53f1fac02c8b` |
| <a id="src-0050"></a> **SRC-0050** · Dossier MOTION_CORE_V5_20261020 · REPORT.md | `b58bad07d60dc42b2dff2f8b8d51a43f80d9dab3fbb24c11d4167ea3877c1dfb` |
| <a id="src-0052"></a> **SRC-0052** · Dossier MOTION_CORE_V6_20261021 · RCA_OPERATIONAL_FAILURE.md | `c211d395078c415c7e0727421bde4f3aa15553f3cdb071ca3af71551f7530425` |
| <a id="src-0053"></a> **SRC-0053** · Dossier MOTION_CORE_V6_20261021 · RCA_PATHBOX_PHASE1_OPUS55_20261022.md | `93c5724a7eaab42966ab2a85539fd7f49493320b5fa0fb011c87c2f93260d480` |
| <a id="src-0033"></a> **SRC-0033** · Dossier MOTION_POST_VALIDATION_20261016 · REPORT.md | `9d1471a8831021b0225e0f7958ecc6290f5aec93ed9b3e1e94afe6d4bcd03563` |
| <a id="src-0047"></a> **SRC-0047** · Dossier MOTION_REMEDIATION_V2_20261020 · REPORT.md | `e36f32251c3b6b565b788efeae3cd69947857e7ed402a87f4ccdaa838e4cb2a1` |
| <a id="src-0137"></a> **SRC-0137** · Dossier MOTION_REMEDIATION_V2_20261020 · matdog_build_manifest.txt | `e64e063ca0213810c31fb4f586b38d4a43dbfe5efd13afca9443f46cae1015ce` |
| <a id="src-0049"></a> **SRC-0049** · Dossier MOTION_RUNTIME_V4_20261020 · REPORT.md | `3f83fe123649aa11f9ad59a32b744df4d1961e71466001939b1e936e0dc9edec` |
| <a id="src-0139"></a> **SRC-0139** · Dossier MOTION_RUNTIME_V4_20261020 · matdog_build_manifest.txt | `441503e5f6b243543072c45fbeb85063c41066bb68e116c7d46bd3e9bc919356` |
| <a id="src-0048"></a> **SRC-0048** · Dossier MOTION_SYSTEM_V3_20261020 · REPORT.md | `8a53621fbfd68fcde1a85ded8b5cb47b6911df44c7f7249293f15eb2ce62944a` |
| <a id="src-0138"></a> **SRC-0138** · Dossier MOTION_SYSTEM_V3_20261020 · matdog_build_manifest.txt | `6588a7baa6f82a59ccc4f89d1aebab4dbf2e3027778ad30e2b4e4470195dee03` |
| <a id="src-0046"></a> **SRC-0046** · Dossier MOTION_UNIFIED_20261019 · POST_FLASH_D84B65DF_20261019.log | `d71cc5613aa7d30ddcaacf50bee296ec04237496a45e52bbe13bf0852e8077ee` |
| <a id="src-0045"></a> **SRC-0045** · Dossier MOTION_UNIFIED_20261019 · STAND_WALK_HARDWARE_20261019.log | `1936a5391f05c99c4aa71929630f0e5c9acc9df2133056092e366de358000edb` |
| <a id="src-0136"></a> **SRC-0136** · Dossier MOTION_UNIFIED_20261019 · matdog_build_manifest.txt | `e10ade4c69b6ba28c0ae3c1acf95b7a66a96296b8085d6f3d00659a46349d8cd` |
| <a id="src-0044"></a> **SRC-0044** · Dossier MOTION_UNIFIED_20261019 · matdog_build_manifest.txt | `bc927b95b7666b39a4d97ee3a461abd0a06a87bbe2fa6ae5d7939facaa56ae4a` |
| <a id="src-0029"></a> **SRC-0029** · Dossier PAIRED_PRONE_20261012 · REPORT.md | `ca01f8a01aeafb9b4e5eb57a41bea11924c86769f5167b50df41da475d23cf63` |
| <a id="src-0030"></a> **SRC-0030** · Dossier PRONE_GEOMETRY_RH_20261012 · REPORT.md | `3d10ad4a39cff5c049bba1153cee2822705a715f4511c81dccfd12fd3595f4ae` |
| <a id="src-0066"></a> **SRC-0066** · Dossier RC4_RX_RECOVERY_IMPLEMENTATION_20261024T122219Z · REPORT_RX_RECOVERY.md | `ddfdf62d818625313e5d4c90ec3270a2645cc914a754dc34330c43ce3adadcbf` |
| <a id="src-0031"></a> **SRC-0031** · Dossier RUNTIME_COMMISSIONING_20261014 · FLASH_GO_NO_GO.md | `0cd9d0c34c325c3aeed70680e1c88d32c8ca14d5085bbe1c2403dbc7db80c01f` |
| <a id="src-0038"></a> **SRC-0038** · Dossier SERVO_PRIME_READBACK_20261017 · CANDIDATE.md | `9c97819929960152b6507ba5f72205d4f886e30c84c0c4d3eb294c39c64903de` |
| <a id="src-0057"></a> **SRC-0057** · Dossier SIMPLE_RC3_APP_ONLY_20261022T202441Z · REPORT_FLASH_BOOT.md | `5b7ee5c38d55c6e21518f3bf31a65f86ffc985cef73037059fe4ee8cbf984d85` |
| <a id="src-0041"></a> **SRC-0041** · Dossier STAND_A34D528_20261019 · diagnostic-extract.txt | `6d5e60149a0c5f43b8ddd56097e97374a8b66da021173f1e641844da25107efa` |
| <a id="src-0043"></a> **SRC-0043** · Dossier STAND_B5BE64C4_20261019 · stand-20261019T175718-0gwr3j.log | `52a19ca7abdf0ce84e389725992d69f9d2ba0d8ab1574f5583b91c897c173275` |
| <a id="src-0037"></a> **SRC-0037** · Dossier STAND_OPERATIONAL_20261017 · ANALISI_PRIMO_STAND.md | `c2de8c13480f416f709bdf75a2e4240e0586c25499e50fe93600fdd15fa7c59c` |
| <a id="src-0034"></a> **SRC-0034** · Dossier SUPERVISED_APP_FLASH_20261017T055655Z · EXECUTION_REPORT.md | `ae9dbc4f21adaab79c03da41eb1aeff415fcbcef9c73b66fb06f2c0d808134fc` |
| <a id="src-0035"></a> **SRC-0035** · Dossier SUPERVISED_APP_FLASH_20261017T055655Z · FIRST_BOOT_REPORT.md | `62aac30cc3cd1b5da78a72351b0349018929ec2d5c5e658f186a912140bc5a44` |
| <a id="src-0036"></a> **SRC-0036** · Dossier SUPERVISED_APP_FLASH_9512816_20261017T102834Z · FLASH_REPORT.md | `904df210ff5b7b6d14b0cee17de946701a7db5d27c9d692c38bc07303ca5133d` |
| <a id="src-0028"></a> **SRC-0028** · Dossier closeout-20261011 · MATDOG_HW_CLOSEOUT_INPUT.md | `c35b6cea733b1fd6bb6027dd2be213af56b58946cb4a2b58adac145069748e2b` |
| <a id="src-0054"></a> **SRC-0054** · Firmware Controller · SIMPLE_MOTION_V1.md | `a85a749903412d1cf4a405f9f9d5f5e7bafde7317e146efd25f60256acf54f22` |
| <a id="src-0001"></a> **SRC-0001** · Firmware Controller · VALIDATION.md | `85f5ee2fa3635e8a7747f960d5f89a5ad70c19d1bea4aaa3f6f1c52b43e62da7` |
| <a id="src-0002"></a> **SRC-0002** · Registro sviluppo · 2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md | `04386e5db7cc3cf1ef828a1b93e2f88d766b1cf35cfacf10aa6248adfccda0f5` |
| <a id="src-0003"></a> **SRC-0003** · Registro sviluppo · 2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md | `97d213caff35fd8d03ab3185f137f3afdfcbeb89c99da2597252ab7cbcf9fe54` |
| <a id="src-0004"></a> **SRC-0004** · Registro sviluppo · 2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md | `0325049023e871320177cf6eee60b811cbff1bd2d526a27fd9ba20c5c70a5540` |
| <a id="src-0005"></a> **SRC-0005** · Registro sviluppo · 2026-09-25_RECOVERY_BACKUP_AND_CANDIDATE_V3.md | `977e8fd87adacfcb198528246baa84beae256f101543ef60dcf3a2d28b63899c` |
| <a id="src-0006"></a> **SRC-0006** · Registro sviluppo · 2026-09-26_HARDWARE_VALIDATION_CANDIDATE_V3.md | `57ec1f3f57d8c6b710f4d5a065d9253129cdfdcebfb2149d6d32a19406abcc69` |
| <a id="src-0007"></a> **SRC-0007** · Registro sviluppo · 2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md | `e803574675f6e068f386bcfd3a01218185ce6cfff367844dab79763b5105f769` |
| <a id="src-0008"></a> **SRC-0008** · Registro sviluppo · 2026-09-27_CR2C_Q0_HARDWARE_SESSION.md | `7ff3ecdb4e4048dfa92cd37cfb9f9911c0023e6733fcf346c65b5911895fec69` |
| <a id="src-0009"></a> **SRC-0009** · Registro sviluppo · 2026-09-28_CR3_M5_PRODUCTION_COMPOSITION.md | `1222b30842c3e93343a646fdb86fc234f28c13ce4aee3e3e57926531ee8e427e` |
| <a id="src-0010"></a> **SRC-0010** · Registro sviluppo · 2026-09-28_CR3_M6_ENVELOPES_AND_SESSION_CLOSEOUT.md | `9c5d84c415836224d8dd985a9e7e2152474c24bd9aa3dd5e0b2b8ee46bc88364` |
| <a id="src-0011"></a> **SRC-0011** · Registro sviluppo · 2026-09-29_CALIBRATION_FULL_4LEG.md | `ce0baec5ee3db1c81919d0dd051d47a02ebd1c5dba0fa3b93f5f66324840dd14` |
| <a id="src-0012"></a> **SRC-0012** · Registro sviluppo · 2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md | `6c06b0a354477b3bd909dc3e24a27b4dff63605e0356ddcc16a09a503a20f606` |
| <a id="src-0014"></a> **SRC-0014** · Registro sviluppo · 2026-10-02_M0_2_HARDWARE_ALIGNMENT.md | `bb63fdaabcdfb6715f3f1c4ca5c6fb664bececfb3e24ab72b6d630860cc8f813` |
| <a id="src-0017"></a> **SRC-0017** · Registro sviluppo · 2026-10-03_M0_4_P1_CLOSED.md | `5cea157f3fe7d965c5031d4c85c98c6a639e9282854d9a9e25ca2f8de6a8829a` |
| <a id="src-0027"></a> **SRC-0027** · Registro sviluppo · 2026-10-11_HW_NON_MOTION_CLOSEOUT.md | `fca5c648a77cc40ccb64dfc247d3276c601745f072e72d7a5f3c4f6958ca8c85` |
| <a id="src-0013"></a> **SRC-0013** · Validazione Full_Calibration_24_Contact_Hardware_2026-10-01 · README.md | `50e05a50ec759073935b253f80e6918a895cdeabda9fe77c6ab4b13a31c2d090` |
| <a id="src-0020"></a> **SRC-0020** · Validazione · MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md | `538887b2ffb117a9e72ff685aca86c45ae60d2f6d180430cdbb13439d997a787` |
| <a id="src-0022"></a> **SRC-0022** · Validazione · MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md | `cf3b45f3a68d159dd35fd32751c724c9ec8dabbdcfde1f895889d9b6618469cd` |
| <a id="src-0140"></a> **SRC-0140** · Allegato documentale / 1-MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V1.md | `191728a8231e6ed9d44c93c428841b6ebb7e3bc50a7dbbee54c230c4afc9abcc` |

**Fonti citate senza SHA di sorgente nel registro originale** (nessun hash inventato):

| Identificativo fonte / nome logico | SHA256 fonte |
|---|---|
| <a id="src-0055"></a> **SRC-0055** · Dossier SIMPLE_OPT_20261022 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0056"></a> **SRC-0056** · Dossier SIMPLE_TRAJ_V2_20261022 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0058"></a> **SRC-0058** · Dossier SIMPLE_RC3_P1_DASHBOARD_OFFLINE_20261023 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0059"></a> **SRC-0059** · Dossier SIMPLE_RC3_P1_1_STOP_LANE_OFFLINE_20261023 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0060"></a> **SRC-0060** · Dossier MATDOG_CONTROLLER_RC4_OFFLINE_20261023 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0061"></a> **SRC-0061** · Dossier MATDOG_CONTROLLER_RC4_R3_OFFLINE_20261024 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0062"></a> **SRC-0062** · Dossier MATDOG_CONTROLLER_RC4_R4_OFFLINE_20261024 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0069"></a> **SRC-0069** · Dossier MATDOG_PREENABLE_APP_ONLY_20261024T165255Z | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0080"></a> **SRC-0080** · Dossier MATDOG_ANALYTIC_JAC_20261025 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0081"></a> **SRC-0081** · Dossier FLASH_ANALYTIC_JAC_20261025_213809 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0090"></a> **SRC-0090** · Dossier MATDOG_ANALYTIC_STAND50_20261026_073322_1237175 | UNKNOWN — hash non riportato nella V2 |
| <a id="src-0091"></a> **SRC-0091** · Dossier MATDOG_ANALYTIC_STAND50_20261026_073709_1238452 | UNKNOWN — hash non riportato nella V2 |

**Chiusura documentale (audit V2 originale):** verificate all'epoca l’esistenza di tutti i 156 collegamenti locali e la corrispondenza di tutti i BIN disponibili accanto ai 43 manifest del registro. Nessun mismatch rilevato. Questo controllo attesta i file archiviati; non sostituisce flash/readback o prove dinamiche. La copia iniziale completa recuperata dopo l’interruzione è preservata nell’archivio storico identificato come `MATDOG_DIARIO_STORICO_20261026_V2`.

## 8. Lacune non ancora risolte

| ID | Lacuna / campo UNKNOWN | Evidenza mancante | Effetto sulla lettura storica |
|---|---|---|---|
|L01|Upload standard precedente 15/09 e candidateG2|Manifest/uploadlog distinto dalla APP 5b371da|Non identificabile come installazione APP-only15/09|
|L02|Raw sessioniKEY 19/09|Catture scratch perse, perdita dichiarata 20/09 inVALIDATION|Restano trascrizioni; SHA raw storici non riverificabili, non ricreare con nuova scrittura|
|L03|Persistenza0x0120 dopo vero powercycleBMS|Readback postciclo documentato|ImmediatoACK/readback non prova persistenza|
|L04|Tutti i tentativi 30/09–01/10 e flash 29/09|Manifest+log nominativo di ogni tentativo, APP 14881cd completa|Successo01/10 certo nel report; successione integrale tentativi non chiusa|
|L05|M0 esecuzione completa e versioni0.1/PHY03/10|Receipt di tutteW/boot, APP 7b258/a063 esatta e relatividiff|PianiPASS non provano migrazione; numeriWi-FiV1 non riconfermati raw|
|L06|DEV1/DEV2 SHA completi e installazioni intermedie|Frozenmanifest/receipt di ciascuna APP non estratti nel registro|DEV3non eredita automaticamente tutta laqualifica dei rami|
|L07|Firmware14/16 e alcunePRIME17–18: installazione perAPP|Flash/readback+manifest+dirtysnapshot della singola build|Checkout/compilazione e report generico non identificano sempre binari installati|
|L08|Operandi realiACCELERATION_LIMIT d84/V5|Target consecutivi, release,TX,dt,giunto al fault|Clock bug dimostrato offline, causa esclusiva hardwareHYPOTHESIS|
|L09|Ritardo M42 V6|TracciaTX/RX e goal applicati, dinamica/carico/ACC reali|Non si sceglie attrito,servo guasto oPATH_BOXbug come certezza|
|L10|Microdelta completo r3/r4 e prima timing build|Receipt/diff eSHA iniziale timing, non solonome|Manifest distingueAPP; comportamento specifico non ricostruito per ogni microrevisione|
|L11|SOURCE_STATEdirty delle versioni antiche senza snapshot|Diff+untracked congelati al build|Commit/troncamento non basta a riproduzione; nessun diff odierno usato alposto dello storico|
|L12|Causa precisa dei5REFERENCE su 4e312 e torque residua su 942|Campione causale e catturaTX/RX/OFF applicato alservo|RX_PENDING eQC side effect non dimostrano causa di ogni episodio|
|L13|Causa target dei 26 STALE / 1 BMS / 1 CONTROL_READ_LOST e tempi CPU|Prima release lenta con durate sezioni e predicati al fault; rigaMOTION_TIMING_FIRST deltest effettivo|Profilo host e pattern sintetici non ricostruiscono il run|
|L14|Body 503 e seriale contemporaneo26/10|Corpo 503, log USB continuo,resetreason,stato AP/TCP dopo perdita|Non distinguibiliCONTROLLER_LOOP_LATE/MAILBOX_BUSY né reset/brownout/starvation radio|
|L15|CompletamentoSTAND50/150 eSTOP durante blackout su 54ade4|End state/HOLD, quota misurata,12 feedback eSTOP → OFF 13/13 sincronizzati|Movimento visivo conservato senza dichiarare successo completo|
|L16|Rete/WCET/STOP end-to-end su fe33|Prova fisica della APP esatta, radio/socket/Controller/SAFE_OFF correlati|Finale soltanto OFFLINE, nessun esito HW trasferito|
|L17|WALK continuo e FAST|Log+osservazioni fisiche attribuiti a APP econfigurazione|Nessuna qualifica fisica reperita; replay non colma|
|L18|Successo C4-C antecedente al periodo|Start/end 12 joint, timestamp, quota econfigurazione|Precedente dichiarato resta storico, non baselineRC5|
|L19|Integrità totale archivio/worktree mentre altro lavoro è attivo|Snapshot globale immutabile non richiesto né eseguito|Hash solo fonti/BIN collegati; nessuna promessa di immutabilità globale durante sessioni Claude|

**Integrità V1:** SHA256 prima e dopo la generazione V2: `191728a8231e6ed9d44c93c428841b6ebb7e3bc50a7dbbee54c230c4afc9abcc`; identico. Nessun originale riscritto. Nessun test firmware o hardware effettuato nella verifica documentale.
