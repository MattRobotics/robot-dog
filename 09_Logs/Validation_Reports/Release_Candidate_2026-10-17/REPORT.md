# MATDOG — Recupero e candidato APP-only, 17/10/2026

**Decisione: GO per una decisione esplicita di flash APP-only del candidato sotto identificato. Nessun flash eseguito e nessun hardware PASS.** La qualificazione statica è COMPOSTA, con il primo FAIL conservato e la risoluzione esatta verificata dal gate; non è un secondo audit integrale PASS.

Il recupero iniziale è stato read-only: branch fix/motion-post-validation-20261016, HEAD c54eab0, nessun cambiamento staged, 15 file modificati e 6 untracked. Snapshot dei 21 file, patch e hash in RECOVERY_RECEIPT.json/recovered-files. Preservati 652c2e2, b0b7e51, b060385, 3360954 e c54eab0; worktree Motion originale 2c2c13b pulito. Nessun ripristino/reset/rebase, nessun dispositivo o processo seriale aperto/interrotto.

Il commit host c54eab0 era già presente, con 15 test PASS. La sessione interrotta aveva preparato CSP, diagnostica bounded, Battery/LED, timestamp e barriera STAND-only; non aveva avviato nuove build/audit completi. I primi log LED FAIL sono conservati insieme ai successivi PASS, senza sostituire il significato delle prove. Le modifiche recuperate sono state revisionate, corrette nei casi SOC non numerico e formatter non supportato, e consolidate nel sorgente pulito 8e1673b371400cacff9f24e344009bc20ae32b63. Anche la nuova regola CSP dell'audit è stata corretta: il comment stripper storico trattava // negli URL come commento; la regola ora legge soltanto i literal della policy reale, senza indebolire gli altri controlli.


## Identità congelata e commit

| Ruolo | Identità |
|---|---|
| Sorgente compilato pulito | `8e1673b371400cacff9f24e344009bc20ae32b63` |
| Host recuperato | `c54eab02017f318591aa8ae6104f662fc11d3701` |
| Tooling/qualificazione successiva | `66cb04e21e85084fd9b725d458d29b169604eb0e` (solo host/test, nessun sorgente compilato cambiato) |
| ROBOT_POWERED SHA-256 | `af13420cd45858b1af89781b430fa49af97e114ddb168873cf5c66b3e0037711` |
| USB_ONLY SHA-256 | `205533ee418f3f0e7c308d741c504456090ade8f90da082b1bae2b5d62de7f59` |
| Layout/tabella | `MATDOG_16M_2x5M_NVS_V1` / `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |

File ROBOT_POWERED verificato:

`/home/matteo-manicardi/MATDOG/verification-artifacts/RECOVERY_FINAL_20261017/build-powered/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`

ELF ricostruito con esptool locale byte-identico al BIN; digest ELF incorporato corretto, manifest e tabella verificati. Entrambe le build: CLEAN, ROBOT_POWERED/USB_ONLY corretti, OTA ingest 0, boot motion 0, runtime-governed, supervised-validation-capable 0 senza falsificazioni. Versione semantica resta 0.2.0-dev.5: identificare il candidato con GIT_SHA/build ID e digest, non con quella sola versione.

## Stato recuperato prima di intervenire

| Requisito | Stato prima dell'interruzione | Modifiche recuperate | Test esistenti | Lavoro completato ora |
|---|---|---|---|---|
| Host APP-only | c54eab0 committato | Gate V3, inventari, evidenze | 15 PASS | Qualificazione composta stretta, 20 test, workflow su nuovo pacchetto e tamper test |
| MOTION_ADMISSION | Non committato | Buffer 768 B, backpressure, niente Print formatter | Overflow/short write PASS | Revisione, nuove build ed ELF/stack |
| Twin CSP | Non committato | Allowlist e validazione UI | Browser reale/header produzione PASS | Riutilizzo provato byte per byte, fixture AP-only corretta |
| Battery/LED | Non committato | Debounce distinto, critical immediato, blink 2 Hz, energia separata | 113871 check e 117 mutazioni PASS dopo i primi FAIL conservati | Caso SOC NaN UI, build e regressioni finali |
| Timestamp | Non committato | UTC/Rome/DST UI e log bridge | Formatter/DST e bridge PASS | Fallback formatter e test di entrambe le transizioni DST |
| Build/congelamento | Non iniziato | Nessuna nuova build trovata al recupero | Vecchio ac972… soltanto | Nuovo SHA congelato, designazione e piano APP-only |

## Matrice finale e prove

| Requisito | Stato | Evidenza / limite |
|---|---|---|
| Recupero/preservazione | PASS | RECOVERY_RECEIPT.json, patch e 21 file recuperati, commit precedenti conservati |
| Provenienza Git e gate | PASS | CANDIDATE.json committato; sorgente e qualificazione distinti, inventari e hash indipendenti |
| Audit integrale originario | FAIL risolto | Una invocazione, un finding della fixture CSP; log originale static-audit.log invariato |
| Qualificazione statica finale | PASS COMPOSTA | Include esatto nella fixture; AP-only 134 check, session clock 45, source convergence e identity build 11 test completati; host-remainder.log |
| Gate/composizione negativa | PASS | 20 test: dirty, source/tool drift, app/ELF/manifest/evidenze, profilo, flag capability, errore non risolvibile, coda test mancante e risoluzione alterata |
| Runner APP-only reale con double file-only | PASS offline | 6 scenari: PASS, MAC, TABLE, OTA, VERIFY, BACKUP; zero scritture prima dei gate, una sola coppia APP in caso positivo, readback simulato indipendente |
| Alterazione artefatti nuovi | PASS | 11 rifiuti su copie reali BIN/ELF/SDK/tabella/manifest/profilo/evidenze/sorgente/tool/ignored-source/dirty; positivo con commit di qualificazione distinto. Originali non modificati |
| Diagnostica Motion | PASS | Massimi campi, età signed, overflow, buffer, backpressure e short write/no retry; niente Print::vprintf sul percorso |
| STAND / feedback / deadman | PASS offline | 768673 check finali; clock 3000/3001 ms, futuro/wrap/OFF storico; HTTP produzione 78 |
| UI/CSP/auth/POST | PASS offline | DOM Motion, portale 172 check; CSP restrittiva e frame validati con browser reale/header, host assente e URL non valido; niente replay POST |
| Batteria/LED | PASS offline | 113871 check, 117/117 mutazioni; 3190 mV/9,7 V/SOC 33,2 critico, nessuna barra verde; carica e KEY OFF preservati |
| UTC/Rome/DST | PASS nelle nuove viste/log | Formatter con UTC/offset/CEST/CET, entrambi i salti DST; log host e fallback; nessun wall-clock safety |
| Build / ELF / manifest / partizioni | PASS | USB_ONLY e ROBOT_POWERED puliti, ELF→BIN identico, schema/layout invariati |
| Stack/memoria | PASS analisi; OPEN runtime | 11456 B noti su loop 16384; margine aritmetico 4928 prima del runtime; globali +8 B. Nessun aumento stack |
| Boot OFF / RESTORE | PASS per sorgente e host test; OPEN fisico | Auto-RESTORE/binding/authority non avviano torque; STAND esplicito e gate live necessari |
| Rollback 2c2c13b | PASS statico | File/hash/manifest/layout/range verificati, runner originale normale senza designazione nuova; NVS preservata |
| PRONE→STAND fixture / timing reale / heap | OPEN | Non eseguiti; nessun hardware PASS |
| WALK/FAST e altre suite Motion fuori selezione | ESCLUSI / OPEN | Barriera STAND-only attiva; survey precedente riutilizzata solo come evidenza algoritmica, 286 suite/artifacts generali restano DEFERRED dichiarati |

Tutte le prove della nuova campagna sono in `~/MATDOG/verification-artifacts/RECOVERY_FINAL_20261017/`; quelle recuperate restano in FINAL_REMEDIATION_20261016, con ricevute di riutilizzo. La designazione conserva il risultato originale FAIL/exit 1 e annida la risoluzione PASS/exit 0: non maschera né rinomina il primo esito.

## Correzioni

- Gate V3: designazione committata in checkout pulito; source_commit e qualification_commit separati; inventari Git indipendenti dei sorgenti compilati e del tooling; SHA app/ELF/manifest/tabella/SDK; digest ELF incorporato nell'app; evidenze di questa campagna con hash, source e exit status. Nessuna ricevuta del 13/10 è usata per qualificare il candidato nuovo e nessuna capability viene falsificata. Il runner conserva tutti i gate MAC, backup, layout, OTA/anti-rollback, partizione, range, write singolo e verify-flash.
- MOTION_ADMISSION: buffer locale 768 B, integer/string snprintf bounded, nessun Print::vprintf o allocazione esplicita; tutti i 12 slot/stamp/età signed/read-ok e epoch/generation/lease mantenuti. Capacità insufficiente o overflow scartano il log; short write non viene ritentato. La decisione safety non dipende dal successo della diagnostica.
- Twin: frame-src consente soltanto http://192.168.4.2:8765 e http://192.168.1.149:8765, le stesse origini validate dalla UI. connect-src resta self; auth/CSRF e le altre direttive non sono allargate. PC AP deve avere il .2 autorizzato oppure il telefono deve raggiungere il PC sull'origine LAN autorizzata: nessuna rete/configurazione è stata modificata automaticamente. Il bridge serve asset e possiede la USB; il viewer non rinnova il deadman. Un host assente non causa fallback di download sul robot.
- Energia: warning 3500 mV/10,50 V, critical 3350 mV/10,05 V; ingresso critical immediato sul nuovo campione. Warning e recupero richiedono due campioni DALY distinti consecutivi; repaint, timestamp regressivo e campioni stale non contano. Uscita critical 3450 mV/10,35 V e warning 3600 mV/10,80 V, entrambe le condizioni. Un dato stale conserva il latch dell'allarme, mentre la UI dichiara disponibilità non verificata. Carica fresca preserva il breathing verde, fault DALY e KEY OFF solo informativo hanno la semantica precedente. Il critical di scarica lampeggia rosso 2 Hz, duty 50%, entro il ceiling esistente.
- La UI distingue SOC BMS stimato/non calibrato da disponibilità energetica. A 3190 mV/9,7 V/SOC 33,2 mostra BATTERY_CRITICAL e energia operativa esaurita; la barra SOC è nascosta. Il valore BMS resta una stima, non è ricalcolato come SOC fisico zero. Anche un SOC NaN non nasconde il critical di tensione.
- Orari: UTC ISO8601 e Europe/Rome con offset e CEST/CET nella UI diagnostica, incluso DST; log bridge con UTC/Rome/fonte host e monotonic separato. Fallback solo UTC dichiarato se il browser non supporta il formatter. Uptime, nonce USB e clock safety invariati; nessuna scrittura orologio DALY, nessun RTC presunto. Il nonce HTTP uniforme e l'ancoraggio civile comune a tutti i vecchi log non sono introdotti: restano limiti diagnostici espliciti.
- kStandOnlyCandidate=true impedisce binding operativo WALK/FAST e avvio walk da CLI. Algoritmi, profili, tracking, limiti, calibrazione, STOP e timeout non sono alterati.

## Memoria e stack

ROBOT_POWERED: 3887408 B, 74,1% dello slot 5242880. Globali 182036 B, +8 B rispetto al candidato precedente; spazio staticamente restante 145644 B non equivale all'heap runtime. ELF reale: pre-arm 9744 B noti, avvio STAND 9120, update STAND 11456, arm/diagnostica 7424; frame arm 1248. Stack loop 16384 e HTTP 24576 invariati. Differenza di 16 B nel percorso STAND rispetto alla review precedente; margine aritmetico prima di runtime/libc 4928 B, non watermark misurato né limite superiore totale. Callx con literal e dispatch virtuali riesaminati. Il grosso percorso WALK resta escluso.

Il minimo heap storico circa 21 KB non dimostrava una perdita permanente: il libero recuperava circa 48 KB. La nuova build non fornisce misure fisiche; raccogliere libero/minimo/largest e watermark/gap durante la prima fixture. Le latenze reali HTTP/USB, la percezione LED e l'anticipo energetico sotto carico restano da misurare.

## Qualificazione e rischio residuo

Le prove browser reali del 16/10 con header di produzione, host FAKE_DEVICE presente, host assente e URL respinto sono riutilizzate: policy e codice iframe sono byte-identici, con ricevuta BROWSER_EVIDENCE_REUSE.json. Le parti pagina cambiate il 17/10 sono coperte dai nuovi DOM test. Il bridge era simulato su PTY, mai il robot.

Le 8813802 verifiche operative precedenti non sono state ripetute: root algoritmi Motion/execution/actuator/calibration/servo/power e modelli invariati, prova hash/diff in MOTION_EVIDENCE_REUSE.json. È stato invocato un solo audit completo: ha eseguito i controlli statici e la campagna host fino alla fixture AP-only, che mancava del nuovo include CSP. Il log originale resta FAIL/exit 1 con un solo finding. La correzione host/test 66cb04e aggiunge esclusivamente l'header reale nella fixture, senza modificare il firmware o regole dell'audit. Sono stati poi eseguiti AP-only, session clock e tutti i comandi rimasti a valle del runner, con esito PASS. Il gate accetta questa qualificazione COMPOSTA soltanto per quell'errore esatto e quella correzione esatta, con log/hash/provenienza ed evidenze complete; 20 test positivi/negativi ne coprono il rifiuto su errori diversi, prove alterate o code del runner mancanti. L'audit usa con MATDOG_STAND_ONLY=1 soltanto per la suite operativa: STAND, rifiuti e heartbeat nuovamente verificati; WALK/FAST non qualificati. Non chiamare questo un rerun della survey WALK completa.

OPEN fisici: boot osservato sul dispositivo, heap/stack residuo e timing STOP/deadman sotto carico, autonomia/anticipo soglie/LED, connettività AP PC/telefono e identità dell'host alle origini autorizzate; PRONE→STAND fisico sulla fixture. HTTPD resta single-task con budget finiti: priorità STOP al Controller dopo ricezione non garantisce preemption del socket/server. Taglio fisico e fixture indispensabili nella campagna successiva. OPEN diagnostici: HTTP nonce non uniforme e vecchio logger con data nel nome file, senza riscrivere lo storico DALY. Questi limiti non costituiscono hardware PASS.

## Handoff

[Procedura APP-only e rollback](APP_ONLY_FLASH_AND_ROLLBACK.md). Il GO riguarda la preparazione offline di questa immagine per una futura decisione di flash. Non autorizza accesso hardware, servo, NVS/EEPROM/BMS, push/merge o qualificazione WALK/FAST. La successiva PRONE→STAND resta una campagna sulla fixture, con STOP e taglio fisico.
