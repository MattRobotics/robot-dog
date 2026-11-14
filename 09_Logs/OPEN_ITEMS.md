# Stato operativo corrente — 10/11/2026: `1.0.2-dev.6` ACC 220 installata (build di sviluppo)

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
| HostLink / Jetson | controllo Motion via USB (HostLink) non qualificato su hardware; Jetson non ancora integrata | — |

## Archivi storici — per provenienza, non per attività corrente

Le sezioni originali della dev.15 e dev.16 sono preservate nei loro dossier, anziché ripetere qui checklist di una vecchia build che riportavano `PR #64 DRAFT`, `dev.17 non flashata` o `PRONE non qualificato`:

- [dev.16 hardware](Validation_Reports/Dev16_HW_2026-11-07/REPORT.md) · [candidata pre-flash](Validation_Reports/Dev16_Candidate_2026-11-07/REPORT.md).
- [dev.15 hardware](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [RCA timing](Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md).
- [dev.17 candidata offline](Validation_Reports/Dev17_Candidate_2026-11-07/REPORT.md) · [dev.17 realmente validata e promossa a Stable](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md).

Per i dettagli dello stato del repository, del tag e dell'APP installata usa [README corrente](../README.md) e [Release v1.0.1](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.1). Il tag è immutabile: i successivi commit di sola documentazione su `main` non alterano il binario originale pubblicato.

Archivio cronologico: [dev.14 05/11](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [dev.7 31/10](Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md) · [snapshot documentale dev.14](Historical/Dev14_Documentation_Closeout_2026-11-05/README.md) · versione completa precedente di questo file, con gli snapshot dev.15/dev.16, al commit della release: [`OPEN_ITEMS.md` @ `35f5ca4`](https://github.com/MattRobotics/robot-dog/blob/35f5ca43e0118cdfc501c14456efb615abb9a8be/09_Logs/OPEN_ITEMS.md).

Prossimi interventi tecnici: issue [#52](https://github.com/MattRobotics/robot-dog/issues/52), [#62](https://github.com/MattRobotics/robot-dog/issues/62) e [#48](https://github.com/MattRobotics/robot-dog/issues/48), tutte da trattare come **lavoro del ciclo successivo**.
