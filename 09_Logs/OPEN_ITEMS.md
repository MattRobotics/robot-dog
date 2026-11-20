# Stato operativo corrente — 20/11/2026: installata `1.0.3-dev.2` (TROT58, `3152e88`); Stable `v1.0.2`

**Installata (build di sviluppo, non una release):** `1.0.3-dev.2`, build `3152e880c3cc`, commit `3152e880c3cc2453ba1513a652b730670e0d0dd5`, APP SHA256 `3c8774a9f9d0facfc0cb65af67ad4745c10bd9b2a2a9e906cbb8b0d9802f8c68`.
- **Flash:** APP-only il 19/11 sera; NVS gen-1 e calibrazione `be8d5902bb4e37da` invariate.
- **Evidenze:** [TROT58_HW_2026-11-19](Validation_Reports/TROT58_HW_2026-11-19/REPORT.md).
- **Stable pubblicata (Latest):** [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2) (`1.0.2-dev.7`, `1314bc4`, APP `dd16bd62…`), invariata. È il rollback del firmware installato.
- **Stable precedente:** `v1.0.1`, immutabile.

## Chiusi (VERIFIED HARDWARE sulla fixture, 19/11/2026 sera — `3152e88`)

| Area | Stato |
|---|---|
| Qualifica di `1.0.3-dev.2` | **KEEP, criteri C1–C10 PASS**, fissati prima della prova. Flash e boot, identità, `capabilities`, salute e memoria (`heap_min_free` 39 772 B) |
| TROT58 | Integrato come quarta modalità WALK, solo avanti: 39/49/58 mm ogni 0,9 s, alzata 8,5 mm. Blocco T 37/37: legge esatta (54,44 / 64,44 mm/s), indipendenza fra le andature (`WALK_MODE_NEXT_WALK`), STOP in trotto 2,2 / 2,4 s con ripresa STAND e WALK |
| Regressione crawl | Catena A/B della `v1.0.2` 8/8 (140 passi). SLOW, STANDARD e FAST invariati |
| Motion Timing sul candidato | Massimo 11 629 µs su 16 000, 0 overrun, 0 release operative perse. Una release IDLE saltata a riposo, riportata a parte |
| STOP | USB 16/16 (14 in movimento, verificati in 2,0–3,0 s, e 2 a riposo); HTTP dalla dashboard 1/1 con ripresa STAND |
| Joystick dalla dashboard (pressione tenuta) | Avanti in TROT58 a fondo corsa, tenuto dall'operatore, in D1 |

## Chiusi (VERIFIED HARDWARE sulla fixture, 19/11/2026 mattina — Stable `v1.0.2`, `1314bc4`)

| Area | Stato |
|---|---|
| Motion Timing P0 | **CLOSED — VERIFIED HARDWARE nel perimetro delle prove del 19/11.** Massimo 12 729 µs su 16 000 µs (margine 3 271 µs); 0 overrun; 0 release perse in catena 8/8, D1, D2, blocco D, S9. Congelato: si riapre solo con nuove evidenze di regressione ([ADR-008](Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md)). Non è un WCET né una qualifica a terra |
| STOP | USB 17/17: 14 in catena, 3 nel completamento, verificati in 2,2–3,2 s. HTTP dalla dashboard 2/2, con ripresa STAND |
| Joystick indietro, laterale e in rotazione | STANDARD nel blocco D (27/27) e SLOW sterzato in S9 (15/15), sulla fixture. Velocità eseguite solo sull'asse richiesto; nessun `WALK_IK_OR_WORKSPACE` |
| Dashboard D1/D2 | STAND, HEIGHT, WALK STANDARD, WALK STOP, STOP HTTP e ripresa: PASS con 1 e 2 client HTTP |
| Margine di heap interno con 2 client | Misurato sulla build operativa: `heap_min_free` minimo 43 368 B, soglia 24 576. Il 16/11 la diagnostica aveva 1 096 B di margine |
| Prima ACQUIRE dopo il reset del flash | 5,0 ms, 0 release perse (B, S, W PASS) |
| Blocco D | Eseguito su `1314bc4` (27/27). Il blocco C, la finestra di 30 s in avanti, non è stato ripetuto: il cammino in avanti è coperto da E, F e H |

## Sospesi

| Area | Stato |
|---|---|
| Silenzi DALY all'accensione | **RCA congelata il 19/11.** A/B dev.5 contro `30c894a` INCONCLUSIVE; B2 del 19/11 CLEAN 358/358; dopo il flash di `1314bc4`, 692 letture valide senza errori. Causa **UNKNOWN**; nessuna modifica BMS né soluzione dimostrata. Si osserva con la telemetria esistente e si riapre su una nuova anomalia documentata. Precauzione invariata: QON + 6 min e 150 poll validi prima di Motion ([B2](Validation_Reports/BMS_DALY_B2_2026-11-19/README.md)) |

## Da qualificare

| Area | Stato | Issue |
|---|---|---|
| Locomozione a terra G0–G5 | Mai qualificata. STANDARD, FAST e TROT58 hanno `hardware_qualified=false`; `1.0.3-dev.2` e `v1.0.2` sono qualificate solo sulla fixture. La prova esplorativa a terra del 19/11 sera, dalla dashboard, non è una qualifica: il trace è conservato in locale e va analizzato a parte | #52 |
| STOP a terra a fine swing anteriore; camminate lunghe a terra; frenata sotto carico | Non provati | #52 |
| Condizione `random_3` di `kSteer` (beccheggio ≈ +55 mrad, passo 40 mm) | Certificata solo offline; S9 è rimasto entro ±31 mrad | #52 |
| Ground Contact sensorless, Fase 1 | Diagnostica passiva `GROUND_CONTACT.OBSERVE` (scheda Calibration & Maintenance) presente nel firmware `v1.0.2`: VERIFIED OFFLINE, mai provata sul robot. Probing e trasferimento del carico non disponibili (`THREE_SUPPORT_TRANSFER_UNAVAILABLE`); **nessuna misura assoluta del terreno** (`ground_height_available=false`). [Rapporto](Validation_Reports/Ground_Contact_Phase1_2026-11-13/REPORT.md) | #52 |
| Ciclo a freddo S7 su `1314bc4` (ACQUIRE B dopo freddo) | Non eseguito il 19/11; ACQUIRE è invariato rispetto a `30c894a` | — |
| Temperature spurie | Picchi isolati di un campione (fino a 65–69 °C) su giunti diversi, in `1314bc4` come in `30c894a`; nessuna protezione scattata | #52 |
| Joystick dalla dashboard (pressione tenuta) | Sulla fixture provato solo in avanti (TROT58, D1 del 19/11 sera); indietro, laterale e rotazione sono coperti via USB | #62 |
| HTTP con coppia ON, STOP su telefono durante il caricamento del Twin, UX del Twin, LED | Come al 07/11 | #62 |
| RESTORE manuale e manutenzione | Come al 07/11 | #48 |
| Gemello MuJoCo: carichi a terra | Come al 10/11 | — |

## Prossima milestone: Ground Locomotion & Gait Performance

**Non è sviluppata oggi:** qui si registrano solo il punto di partenza e i limiti da misurare.

**Punto di partenza (`1.0.3-dev.2`, solo fixture; velocità nominali, non misurate a terra):**

| Modo | Velocità massima | Alzata del piede |
|---|---|---|
| SLOW | 20 mm/s | 8 mm |
| STANDARD | 30 mm/s | 8 mm |
| FAST | 29,6 mm/s | 6 mm |
| TROT58 | 64,4 mm/s, solo avanti | 8,5 mm |

- **Joystick:** in avanti, indietro, laterale e in rotazione nei crawl; TROT58 solo in avanti.
- **Margine P0:** 3,3 ms, con READ e FRAME come costi principali. Il massimo resta 12 729 µs della `v1.0.2`; su `3152e88` il massimo nelle prove eseguite è 11 629 µs.

**Da sviluppare o qualificare** (NOT VALIDATED, criteri da fissare prima delle prove):
- **locomozione reale a terra:** prova G0–G5 con batteria ≥ 11,0 V e anticaduta, STOP a terra, stabilità e margine di supporto, per i crawl e per TROT58. Prima: COM reale e offset di montaggio dell'IMU. Da analizzare a parte la prova esplorativa del 19/11 sera;
- **velocità e fluidità del gait:** velocità reale rispetto a quella comandata, per modo; oscillazioni del corpo e continuità dei frame;
- **prestazioni WALK FAST:** 40 mm/s non raggiunti. Lo studio offline del 19/11 (non pubblicato) dice due cose:
  - il crawl non li raggiunge entro i limiti di STOP, drive e cattura;
  - la camminata a coppie diagonali sfasate supera legge e plant, ma non i criteri dinamici registrati nel gemello.
  
  Non è integrata: FAST resta 29,6 mm/s. Valutazione precedente del 16/11: [LOCO_EVAL](Validation_Reports/Unified_Operational_2026-11-14/P0_Peaks_2026-11-16/LOCO_EVAL.md);
- **TROT multidirezionale:** TROT58 va solo in avanti. Indietro, laterale e rotazione richiedono inviluppo IMU e box sterzati propri;
- **locomozione omnidirezionale Go2-like:** non iniziata;
- **joystick proporzionali:** oggi il joystick è proporzionale entro la legge della modalità scelta. L'estensione al controllo omnidirezionale è da definire;
- **Kinect v2:** misura esterna per le prove a terra. Non ancora configurata: scopo, montaggio, calibrazione e sincronizzazione da definire;
- **MuJoCo:** gemello V1 offline; discrepanza dei carichi a terra (rapporto M5) da calibrare con misure nuove;
- **valutazione RL:** non iniziata;
- **alzata effettiva delle zampe e distanza dal terreno:** nel gemello il piede in swing tocca;
- **tracking e frenata sotto carico,** cedimento delle anche;
- **sway e traiettorie;**
- **ripresa della diagnostica Ground Contact sensorless,** senza dichiarare disponibile una misura assoluta del terreno.

**Riferimento:** issue [#52](https://github.com/MattRobotics/robot-dog/issues/52).

# Stato al 18/11/2026 (storico)

**Installata:** `30c894a` (`1.0.2-dev.7`, APP `03c863cc…`), VERIFIED HARDWARE su fixture (catena 140/140).
- **P0:** ancora aperto (20 761 µs, 2 overrun).
- **Candidati offline:** P1, FP32 e `kSteer`.
- **DALY:** A/B INCONCLUSIVE.
- **Riferimenti:** [handoff del 18/11](Development_Log/2026-11-18_MATDOG_SESSION_CLOSE_HANDOFF.md) · [qualifica di `30c894a`](Validation_Reports/Motion_30c894a_HW_2026-11-18/README.md) · [campagna P0 del 17/11](Validation_Reports/MATDOG_1_0_2_P0_2026-11-17/REPORT.md).

# Stato al 16/11/2026 sera (storico, branch `integ/matdog-unified-operational-20261114`)

**Installata:** `1.0.2-dev.8` **diagnostica** `6be46906d069` (`a86c680` + F3 + IRAM G2s), APP SHA256 `be678d83037451d7937783e70d443a83612cbc992005a9f0cac9561ad0a13b68`.
- **VERIFIED HARDWARE il 16/11 su fixture:** mirata 12/12, A/B 18/18, FAST E–G 20/20, H 20/20, nessun fault. Copre STAND 80–150, WALK STANDARD/SLOW/FAST 29,6 mm/s, STOP (a riposo, in camminata, nel FAST, dentro la RECOVERY) e RECOVERY.
- **Calibrazioni:** NVS gen-1 e restore 12/12 invariati.
- **Rollback:** `a86c680`.
- **Sorgente del branch:** integra la stessa legge Motion senza strumentazione (build `33134ae28260`, non installata).
- **Riferimenti:** [rapporto](Validation_Reports/Unified_Operational_2026-11-14/P0_F3_G2s_2026-11-16/REPORT.md) · [handoff](Validation_Reports/Unified_Operational_2026-11-14/HANDOFF.md) · [ADR-007](Architecture_Decisions/ADR-007_Motion_Timing_Integration_and_Heap_Margin.md).

## Residui al 16/11/2026

| Area | Stato |
|---|---|
| Motion Timing P0 | **Aperto.** Su `6be4690` restano 2 overrun e release fino a 20,8 ms: STAND/HEIGHT/PRONE attenuate a 2 soluzioni; picchi WALK e RECOVERY 19,5–20 ms. Criterio di chiusura: 0 perse operative, 0 overrun, massimo ≤ 16 ms |
| Margine di heap interno | 1 096 B sopra 24 KB con 2 client HTTP sulla build diagnostica. Un terzo client (dashboard o watcher) lo annulla. Build operativa: +8 168 B di heap statico (ELF), margine non misurato |
| DALY all'accensione | Silenzi con RX idle-high, fino a 182 s, uno anche dal primo poll; causa **UNKNOWN**. Manca la misura RS485 in solo ascolto su A/B durante un silenzio. Precauzione: QON + 6 min e 150 poll validi prima di Motion |
| Blocchi C e D su `6be4690` | Non ripetuti (frame invariati fuori dal FAST, VERIFIED OFFLINE); ultima esecuzione su `a86c680` il 16/11 pomeriggio |
| Prove a terra delle build dev.8 | Non eseguite |
| Build operativa integrata `33134ae` | Solo VERIFIED OFFLINE; mai installata |

# Stato al 10/11/2026 (storico): `1.0.2-dev.6` ACC 220 installata (build di sviluppo)

**Installata:** `1.0.2-dev.6`, sorgente `450d2c7` (in `main` dalla PR #67), build `450d2c793dcb`, APP SHA256 `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256`.
- **VERIFIED HARDWARE il 10/11:** fixture Q1–Q8 qualificata; 14/14 STOP verificati (8 fixture, 6 terra); 0 fault Motion; registro 85 = 220 su 12/12.
- **A terra:** verifica positiva ma fuori protocollo G0–G5.
- **Riferimenti:** [report HW](Validation_Reports/Dev6_ACC220_HW_2026-11-10/REPORT.md) · [evidenze](Validation_Reports/Dev6_ACC220_HW_2026-11-10/EVIDENCE_MANIFEST.md).
- **Rollback:** `1fbee7b`.
- **Stable storica:** `v1.0.1` (`1.0.1-dev.17`), immutabile.

## Residui dopo la validazione del 10/11 (lavoro del prossimo ciclo)

| Area | Stato | Issue |
|---|---|---|
| Qualifica a terra G0–G5 | Non eseguita: la prova del 10/11 era fuori protocollo (batteria 10,6–10,2 V, anticaduta non confermata per iscritto). Requisiti: batteria ≥ 11,0 V, anticaduta, progressione condizionata a un margine delle anche ≥ 4 tick | #52 |
| STOP a terra a fine swing anteriore; mantenimenti lunghi dopo STOP; camminate lunghe a terra | Mai provocati o provati (a terra STANDARD 11 s, FAST 34 s) | #52 |
| Frenata ACC 180/220 sotto carico | Provata solo indirettamente (STOP verificati), non misurata | #52 |
| Joystick laterale, in rotazione e all'indietro in STANDARD e FAST | Non implementato, solo SLOW: handoff della sessione firmware (`HANDOFF_JOYSTICK_MULTIDIREZIONALE.md` sull'ASUS, vedi il [manifest](Validation_Reports/Dev6_ACC220_HW_2026-11-10/EVIDENCE_MANIFEST.md)) | #52 |
| Cambio di profilo con la leva tenuta | Verificato solo offline (DOM 21/21); serve una prova da telefono o tablet | #62 |
| SAFE_OFF dalla dashboard | Il pulsante non esiste; alternative: PRONE, USB `@SERVO SAFE_OFF 51`, KEY OFF | #62 |
| Avviso batteria a coppia ON | Con Motion attivo la presentazione LED è sospesa: l'avviso compare solo a coppia OFF | #62 |
| D1: release perse e overrun | 131 perse e 2 657 overrun, 0 fault; concentrate nelle riprese dopo STOP e nella sterzata | #52 |
| D2: HTTP 503 `MOTION_BUDGET_UNAVAILABLE` | 1 caso in sterzata (aggiornamento joystick non eseguito; STOP e velocità nulla esenti) | #62 |
| D3: temperature spurie | Letture isolate fino a 150 °C solo a coppia ON, tutte filtrate dal firmware; causa da verificare | #52 |
| `EVIDENCE.sha256` della sessione del 10/11 | Assente; la cartella ha uno `SHA256SUMS` completo e verificato. Chiusura a cura del responsabile firmware | — |
| Gemello MuJoCo: carichi a terra | Gemello a 1,9 volte il duty HW in mediana e 3/3 fault agli STOP a terra, dove l'HW li verifica. Per calibrarlo servono forze ai piedi e letture dense delle anche ([rapporto M5](../06_Software/Matdog_MuJoCo/REPORT_M5.md)) | — |

## Stato precedente — 07/11/2026: prima Stable `v1.0.1` (firmware 1.0.1-dev.17)

**Allora installata, VERIFIED HARDWARE nel perimetro qualificato e rilasciata:** `1.0.1-dev.17`, build `17db0efe2324`, sorgente `17db0ef`, APP SHA256 `14e055a0bfa2adcab8f87df29ece0a0cf01a6e9e13b99aee1c94bda59ba84a83`, promossa senza ricompilazione come GitHub Release `v1.0.1`. Rollback: dev.16 `e0ddbb1791d0`. [Report HW](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md) · [manifest release](Validation_Reports/Dev17_HW_2026-11-07/RELEASE_MANIFEST.md).

## Checklist della prima Stable (chiusa con `v1.0.1`)

- [x] APP installata identificata (commit, build ID, SHA256, profilo, slot) e flash APP-only verificato con digest.
- [x] Boot, NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13, SAFE_OFF 8191, audit IDLE.
- [x] STAND100/125, WALK STANDARD 45/60, WALK STOP, STOP e WALK diretto dopo STOP, joystick e rilascio, PRONE con SAFE_OFF, Twin dopo refresh con coppia ON, ViewCube.
- [x] `/status` senza 503 sotto slack Motion, letture oltre il recupero TCP, joystick senza reflow e in ordine (dev.17).
- [x] Sorgente su `main`, tag e Release con APP originale, manifest e checksum; dev.16 conservata come rollback.

## Lavoro residuo registrato il 07/11/2026 (non prerequisiti della `v1.0.1`; in parte affrontato dalle build `1.0.2`)

| Area | Stato | Issue |
|---|---|---|
| Timing Motion | overrun/perse in WALK e joystick (finestra `(36)`: +64 perse, +491 overrun, lavoro max 47,2 ms); variabilità HOLD UNKNOWN | #52 |
| Stack del loop | minimo osservato 2.060 B durante joystick/WALK (dev.16 ≥ 4.208 B): da misurare e ridurre prima di aggiungere lavoro al loop | #52 |
| Velocità e qualità del gait | gait lento (obiettivo fino a 5× non qualificato), sway laterale del joystick, WALK_SETTLE ~8,9 s | #52 |
| HTTP con coppia ON | rari stalli e risposte lunghe (max 1,68 s); causa delle perdite TCP UNKNOWN, serve cattura pacchetti HOLD vs IDLE | #62 |
| STOP durante il caricamento Twin | misurato solo in Chrome (fino a ~0,5 s); misura end-to-end su telefono reale a cache fredda mancante | #62 |
| UX Twin | overlay leggermente trasparente dei comandi dentro il Twin; zampe posteriori sospese sul pavimento grafico in PRONE | #62 |
| LED | verde stabile osservato in dev.17, nessuna qualificazione del sottosistema | #62 |
| RESTORE e manutenzione | RESTORE manuale da dashboard, rifiuti per giunto, multigenerazione non qualificati | #48 |
| Non ricollaudati in dev.17 | rotazioni IMU pitch/roll (dev.15: roll applicato ~1,9° contro ~4,8° richiesti, limite `WORKSPACE`), quote STAND diverse da 100/125, WALK FAST, funzioni avanzate | #52 |
| Termica servo | picchi termici isolati ID21/ID31 (dev.15/dev.16) da discriminare tra dato spurio e fault reale, senza ridurre le protezioni | #52 |
| HostLink / Jetson | controllo Motion via USB (HostLink) non qualificato su hardware; Jetson non ancora integrata ([rapporto HostLink del 06/11](Validation_Reports/HostLink_2026-11-06/REPORT.md)) | — |

## Archivi storici — per provenienza, non per attività corrente

Le sezioni originali della dev.15 e dev.16 sono preservate nei loro dossier, anziché ripetere qui checklist di una vecchia build che riportavano `PR #64 DRAFT`, `dev.17 non flashata` o `PRONE non qualificato`:

- [dev.16 hardware](Validation_Reports/Dev16_HW_2026-11-07/REPORT.md) · [candidata pre-flash](Validation_Reports/Dev16_Candidate_2026-11-07/REPORT.md).
- [dev.15 hardware](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [RCA timing](Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md).
- [dev.17 candidata offline](Validation_Reports/Dev17_Candidate_2026-11-07/REPORT.md) · [dev.17 realmente validata e promossa a Stable](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md).

Per i dettagli dello stato del repository, del tag e dell'APP installata usa [README corrente](../README.md) e [Release v1.0.1](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1). Il tag è immutabile: i successivi commit di sola documentazione su `main` non alterano il binario originale pubblicato.

Archivio cronologico: [dev.14 05/11](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [dev.7 31/10](Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md) · [snapshot documentale dev.14](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md) · versione completa precedente di questo file, con gli snapshot dev.15/dev.16, al commit della release: [`OPEN_ITEMS.md` @ `35f5ca4`](https://github.com/MattRobotics/robot-dog/blob/35f5ca43e0118cdfc501c14456efb615abb9a8be/09_Logs/OPEN_ITEMS.md).

Prossimi interventi tecnici: issue [#52](https://github.com/MattRobotics/robot-dog/issues/52), [#62](https://github.com/MattRobotics/robot-dog/issues/62) e [#48](https://github.com/MattRobotics/robot-dog/issues/48), tutte da trattare come **lavoro del ciclo successivo**.
