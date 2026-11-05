# MATDOG dev.14 — PARTIAL HW VALIDATION, 05/11/2026

**Flash APP-only e boot: VERIFIED HARDWARE. Motion: PARTIAL HW VALIDATION; non stable.** Robot dichiarato spento dopo la sessione. Questo closeout non autorizza altre operazioni hardware e non integra firmware in main.

## Identità e provenienza

| Identità | Valore |
|---|---|
| Firmware realmente installato | 1.0.1-dev.14 |
| Commit sorgente | 4ff2aadf4ed907d39f2b3ad4f9e6af54db5decee |
| Build ID | 4ff2aadf4ed9 |
| APP SHA256 | 71028f95a8aee4b4ff7237f9420eec6aeeb0efa0aee5c00c00ca5a7bb947bd22 |
| APP | 1.669.584 byte, ROBOT_POWERED, OTA ingest 0, source CLEAN |
| Sorgente consultabile | [branch dev.14](https://github.com/MattRobotics/robot-dog/tree/validation/matdog-dev14-hw-20261105) |
| main alla preparazione del closeout | 07f4a2178354d28de71dfa1dad5462f12cbd5437; non identifica il firmware installato |

Il branch sorgente ha HEAD esattamente al commit sopra: nessuna modifica o ricompilazione per la pubblicazione. La build non è ricostruibile bit per bit dal solo SHA Git. La dev.7 pubblicata resta una pre-release storica; nessuna promozione stable/Latest. Dev.15 è sviluppo parallelo, preservato: questo report non ne attesta build o validazione hardware.

## Flash e boot — VERIFIED HARDWARE

Prima sessione sospesa prima della scrittura per riavvii correlati all'apertura del reader CDC. RCA della procedura: transizioni DTR/RTS plausibili, causa specifica non catturata. Reader escluso dalla ripresa e da Motion; nessuna nuova RCA o riproduzione per il closeout.

Ripresa: stato HTTP READY, SAFE_OFF 8191 e coppia OFF. Esptool 5.3.1 ROM --no-stub; un ingresso usb-reset, accessi successivi no-reset/no-reset, singola scrittura APP e verify-flash indipendente. Layout reale MATDOG_16M_2x5M_NVS_V1; tabella e otadata verificati, APP0 selezionata a 0x10000, slot 0x500000. Scrittura 0x10000..0x1a79d0, erase di settore fino a 0x1a8000. Hash della scrittura verificato e verifica indipendente **digest matched**, APPLICATION_ONLY_FLASH=PASS. Riavvio watchdog qualificato; controllo successivo esclusivamente HTTP.

Nessuna scrittura di bootloader, tabella, otadata, NVS, EEPROM, eFuse o altro slot. Nessun dump aggiuntivo. L'ASUS ha richiesto riconnessione manuale all'AP salvato dopo il boot; non è stato eseguito un altro reset.

HTTP: versione/build corrispondenti, READY/RUN, Census PASS 13/13, IMU e BMS disponibili, circa11,5V nel campione, allarmi BMS assenti, SAFE_OFF8191 iniziale. NVS READY, VALID_ACKNOWLEDGED, gen-1 integra, restore_required=NO. Fingerprint e conteggio RESTORE12/12 non sono esportati direttamente dall'HTTP di questa sessione: non vengono inventati. RESTORE automatico12/12 ha evidenze hardware nelle sessioni precedenti; boot e Motion operativi dev.14 sono confermati. RESTORE manuale/manutenzione restano aperti in #48.

## Sequenza e osservazioni fisiche

| Prova | Evidenza e limite |
|---|---|
| STAND150/HOLD | Completato, HOLD oltre30s. L'operatore ha chiarito che il robot partiva già a150 sulla fixture: non prova una salita da terra. |
| STOP terminale | STOPPED_SUPPORT_UNVERIFIED con coppiaON; oltre40s senza falso STOP_NOT_SETTLED; richieste STOP ripetute e STAND esplicito funzionanti. Perturbazione LH_LOWER non eseguita. |
| Trasferimento | Progressione sospesa durante posa a terra con anticaduta; nessun SAFE_OFF automatico. |
| PRONE → STAND150 a terra | Comandi completati, discesa e ritorno regolari secondo operatore. PRONE_CONTACT_UNVERIFIED non certifica sostegno. |
| STAND120/HOLD | Completato, oltre30s; appoggio stabile riferito. Ultimo campione controllato: differenza RAW max15tick, load max128/1023. |
| WALK30 iniziale | Appoggi e swing regolari secondo operatore; interrotto dall'agente per release perse persistenti, con STOP contestuale al posto del WALK_STOP previsto. |
| WALK60 successivo dell'operatore | Eseguito realmente; camminata regolare, vibrazione LF non osservata. Evidenze successive non sono parte della finestra del primo report. |
| WALK45/secondo STOP durante recovery | Non attestati come prove completate dalle evidenze qui riassunte. |

L'assenza di vibrazione LF nel WALK60 non dimostra la risoluzione dell'intermittenza storica. Il primo WALK30 ha raggiunto circa3,58cicli prima dello STOP, oltre i due pianificati; la raccolta HTTP a1Hz e i passaggi agente/tool non costituiscono una protezione in tempo reale.

## Timing e recuperi — VERIFIED HARDWARE

| Percorso | max_work_us | max_compensate_us | Missed |
|---|---:|---:|---:|
| WALK30 prima dello STOP | 34776 | 18810 | 51;52 includendo arresto |
| WALK60 | 33393 | 20062 | 59 |
| Recovery WALK60 → STAND150 | 36135 | 20536 | 22 aggiuntivi |

Periodo nominale20000us. I massimi delle sezioni sono indipendenti: non vanno sottratti per dedurre costi della medesima release. Prima divergenza WALK30: missed1 a uptime539177ms/ciclo0,187536, poi incrementi persistenti. Lavoro superiore al periodo è confermato; attribuzione dominante fra calcolo/preemption/I/O è UNKNOWN. Nessuna nuova architettura o soglia è giustificata dal solo massimo.

Stack loop minimo nel WALK30 osservato4384B, dopo STOP3068B; traccia successiva2496B già nel primo snapshot. High-water cumulativo: non attribuirlo automaticamente al WALK60. Gap massimo osservato36500us. Nessun campione sotto2048B o gap>=60ms nella finestra pubblicata; questo non qualifica il timing.

STOP WALK30 → STAND120:8,044s (930562→938606ms). STOP WALK60 → STAND150:11,443s (1006582→1018025ms). Per il primo recupero sono conservate le ricevute, non gli snapshot di fase. Per60: gait di chiusura circa7,5–7,8s, poi WALK_SETTLE e HEIGHT. I22miss si accumulano prima di WALK_SETTLE. Il codice completa un ciclo WALK_STOP e recentramento: nessun replay accidentale dimostrato. Spostamento nominale da modello circa42,31mm, **non odometria hardware**.

## STOP da PRONE e coppia

Nella traccia successiva: ultimo campione pre-STOP PRONE_DEPLOY/PRONE_ENTRY; STOP accettato1030219ms e completato1032241ms. motion_verified=true, support_verified=false, stand_resume_allowed=false, reason STOP_RECOVERY_POSTURE_UNQUALIFIED. La whitelist SimpleMotion.cpp:445–446 esclude le origini PRONE mobili prima dei successivi controlli geometrici: recupero mancante, non collisione dimostrata. Non rimuovere la guardia senza qualificare il raccordo dai RAW mantenuti.

PRONE terminale mantiene coppiaON e non verifica supporto. L'operatore desidera riposo tramite l'unico STOP contestuale; nessun torque-OFF automatico è qualificato se il sostegno resta non verificato. La traccia7 attesta PRONE con coppiaON, ma non la risposta a un successivo STOP da PRONE. Nessun comando PARK introdotto.

## IMU, telemetria, temperatura e UI

- Traccia6: age_ms max136, control_age_ms max1568, stale globale false. FAST e CONTROL hanno epoche diverse; goal letto non equivale a commanded_raw corrente. Differenze non provano goal persi.
- imu_correction_limited=true62/62 snapshot WALK_STANDARD:40 WALK60 e22 recovery. Il flag aggrega fit non disponibile/proiezione geometrica e non prova guasto IMU. Dominio richiesto/applicato non è esportato: causa di ciascun true UNKNOWN.
- Nuove osservazioni operatore, già documentate in [#52](https://github.com/MattRobotics/robot-dog/issues/52#issuecomment-6666202865): pitch quasi assente a STAND150; a STAND100 pitch e roll attivi ma pitch troppo lento, escursione roll insufficiente. Sono osservazioni hardware qualitative, non latenza o dominio misurati. Le spiegazioni geometriche e di rate restano HYPOTHESIS fino alla prima divergenza strumentata.
- Traccia7 citata nella stessa issue: HOLD100 imu_valid133/133, limited16/133 efalse117/133; PRONE terminale29snapshot a coppiaON. Il tentativo iniziale STAND100 riceve503 MOTION_BUDGET_UNAVAILABLE, poi un altro comando termina: non prova che il primo fosse accettato. Missed cumulativi23 costanti nella finestra non sono23 nuove perdite.
- RH_HIP94°C isolati nel primo log,128°C nella traccia6, campioni adiacenti26/28°C. La rilettura diretta oltre70°C è già nel codice; causa sensore/risposta/corruzione UNKNOWN. Nessun surriscaldamento fisico dimostrato e nessuna soppressione delle protezioni termiche.
- Digital Twin segue la posa secondo operatore; Reset vista non ripristina il riferimento frontale+X richiesto. Difetto UI aperto (#62); nessun reset fisico IMU proposto o eseguito.
- LED: nessuna anomalia riferita nell'ultima sessione; difetto intermittente storico #62 **non risolto** per assenza di evento.

## Classificazione e conclusione

VERIFIED HARDWARE: flash/boot esatti, risposte e telemetria registrate, comportamenti visibili riferiti dall'operatore entro i relativi limiti. VERIFIED OFFLINE: verifiche candidate riportate nei ricevuti locali precedenti, non rieseguite per questo closeout. IMPLEMENTED NOT VALIDATED: capability e recuperi non attraversati nelle prove pertinenti. HYPOTHESIS: meccanismi non discriminati dai dati. UNKNOWN: causa dominante dei picchi, sorgente dei valori termici e supporto fisico non misurato.

**NO-GO alla dichiarazione stable e alla qualifica del joystick.** Nessuna promozione di dev.14 o dev.15. #52 timing/IMU/STOP/recovery aperta; #62 Twin/LED e coordinamento aperta; #48 RESTORE manuale/manutenzione parziale; #46 stable futura aperta. Una successiva candidata unica è un obiettivo, non un risultato di questa pubblicazione.

## Evidenze e privacy

[Estratto flash sanitizzato](FLASH_EVIDENCE.txt) e [provenienza delle evidenze](EVIDENCE.md). Tracce complete, snapshot, report originali e build restano su ASUS. Nessun backup/dump NVS/EEPROM, credenziale, configurazione privata o binario è aggiunto alla PR. Gli archivi storici non sono riscritti.
