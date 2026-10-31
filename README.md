# MATDOG — quadrupede robotico personalizzato

MATDOG è la piattaforma quadrupede di Matt Robotics: un robot a quattro zampe progettato da zero, con l'obiettivo di arrivare a una **locomozione continua, precisa e sicura** (STAND, WALK, stabilizzazione IMU) comandata a livello semantico da una dashboard e, in futuro, da un computer di alto livello.

> **Stato in una riga:** il robot ha una base hardware validata (alimentazione, calibrazione a 24 contatti, diagnostica), ma **il movimento (STAND/WALK) non è validato su hardware** (esistono solo prove fisiche parziali, vedi sotto). Nessuna build di questo repository è autorizzata al flash sul robot attuale.

## Com'è fatto

| Parte | Descrizione |
|---|---|
| Controller | **ESP32-S3**: unica autorità su Motion, bus servo e sicurezza runtime (STOP, SAFE_OFF, limiti, watchdog) |
| Attuatori | 12 servo **ST3215** per le 4 zampe (HIP/UPPER/LOWER) + collo (ID 51); 17 posizioni allocate nel progetto, 13 installate |
| Sensori | IMU **BNO085**, BMS **DALY** (tensione, corrente, SOC, allarmi), feedback di posizione/carico/temperatura dei servo |
| Interfaccia | **Dashboard web** servita dall'ESP32-S3 (rete AP del robot) con comandi semantici (nella linea RC5: STAND, WALK, STOP, RESTORE) e Digital Twin in sola lettura |
| Motion | Il browser invia intenti; l'ESP32 li traduce in traiettorie, cinematica inversa e comandi sul bus. Nessun percorso diretto browser → servo |
| Alto livello (futuro) | Jetson Orin Nano Super, ROS 2 / MoveIt 2 |

## Dove trovare le cose

| Cartella | Contenuto |
|---|---|
| [`05_Firmware/`](05_Firmware/README.md) | Firmware del Controller ESP32-S3 ([README Controller](05_Firmware/MATDOG_Controller/README.md)) e strumenti di banco congelati |
| [`06_Software/`](06_Software) | Calibrazione, geometria, cinematica, bridge host, viewer/Digital Twin |
| [`03_CAD/`](03_CAD/README.md) | CAD, URDF operativo REV01-FIX, 17 STL canoniche, workbook di riferimento |
| [`04_Electronics/`](04_Electronics/README.md) | Alimentazione, cablaggio, mappatura servo ([stati di alimentazione e ricarica](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md)) |
| [`08_Tests/`](08_Tests) | Procedure di validazione ripetibili (le suite host del firmware stanno in `05_Firmware`) |
| [`01_Docs/`](01_Docs/02_Architecture/ARCHITECTURE.md) | Architettura e [roadmap](01_Docs/02_Architecture/ROADMAP.md) |
| [`09_Logs/`](09_Logs/README.md) | Evidenze, decisioni, report di validazione, [indice storico](09_Logs/HISTORY_INDEX.md), [problemi aperti](09_Logs/OPEN_ITEMS.md) |
| [`02_BOM/`](02_BOM), [`07_Media/`](07_Media), [`10_Backups/`](10_Backups) | Componenti, media, ricevute dei backup cifrati |

## Stato al 2026-10-31

Quattro livelli distinti: non vanno confusi.

**1. Firmware realmente installato sul robot (ultima verifica hardware: 2026-10-30).**
`1.0.1-dev.5`, APP SHA-256 `c8b27d672eca145ad8291639fa6762d2279854fe57e93631c29d7506f2cd5bfb`, compilata dal sorgente locale `df43feb`, flashata il 30/10 con readback identico, calibrazione NVS gen-1 invariata. Non è stata riverificata dopo il 30/10. L'identità del dispositivo va comunque riletta (SHA-256 della APP) prima di qualsiasi azione. Rollback qualificato: rc.11 r2 `fc0a7141…`. **Nessuna build di `main` è il firmware installato.**

**2. Sorgente dev5 locale.**
Branch `dev5/prone-stand-20261030` @ `df43feb`: esiste **solo in locale** (nessun branch `dev5` sul remoto), non è unito a `main`. Contiene STAND dal terminale PRONE e le tabelle Motion certificate su **geometria REV00**. Log e tracce hardware di dev5 sono evidenze locali non pubblicate (checkpoint in `verification-artifacts/`, fuori dal repository).

**3. Geometria e documentazione integrate in `main` (PR [#57](https://github.com/MattRobotics/robot-dog/pull/57), merge commit `887103e501a4a02be89e204b77a5301af6d15905`, 2026-10-31).** Verificate solo offline, senza validazione hardware:

- URDF operativo **REV01-FIX** (`03_CAD/URDF/matt_robodog_rev00/`; nomi e percorsi `matt_robodog_rev00` mantenuti per compatibilità, la revisione si riconosce dagli SHA-256 in `SHA256SUMS.txt`): nuovo `base_link` e quattro quote **HIP Z +2,2 mm** (LF/RF 0,0487 m, RH/LH 0,0287 m); cinematica, assi e limiti invariati.
- **17 STL canoniche** in `meshes/collision/`, condivise da `<visual>` e `<collision>`; non esistono più STL estetiche separate né un secondo archivio geometrico.
- **Digital Twin**: le mesh di rendering derivano dalle 17 STL canoniche (staging dal viewer e decimazione a griglia 3 mm con gli strumenti già esistenti); nessuna seconda sorgente geometrica. Il Twin del firmware installato incorpora ancora la mesh decimata REV00.
- **Masse, CoM e inerzie Solid Edge** (30/10) nei 17 blocchi `<inertial>`: configurazione nominale unica con Jetson Orin, 2,928 kg (assieme CAD 2,937 kg, residuo di 9 g documentato e non allocato). Nessuna validazione dinamica su hardware.
- **Workbook canonico a 7 fogli** `03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx` (non aperto in Excel; controllo strutturale OpenXML eseguito).
- **Riferimento progettuale ST3215 a 40 RPM** (quasi a vuoto a 11,1 V, stima preliminare derivata dai 45 RPM a 12 V; ≈ 27,8 RPM a 9,2 kg·cm): è un riferimento di progetto **non qualificato**. I **limiti attuali restano 29 RPM** nell'URDF e invariati; il Motion di dev5 usa limiti propri più bassi.
- **Base storica di calibrazione REV00 preservata** in `03_CAD/URDF/historical_calibration_basis/rev00/`, verificata da `matdog_calibration_basis.py --check` (eseguito da `static_audit`). **NVS gen-1 e tag storico `3713f4ddc43b204e` invariati**; calibrazioni, certificati hardware e STOP non toccati.
- Passaggio di consegne per la sessione Motion: [`ASTRA_HANDOFF.md`](09_Logs/Validation_Reports/REV01_FIX_Alignment_2026-10-31/ASTRA_HANDOFF.md).

**4. Funzioni Motion ancora da qualificare.**
STAND a 150 mm, ritorno a PRONE e WALK STANDARD non sono validati: le certificazioni Motion (box di collisione, tabelle PRONE, regioni di camminata, certificato CAD del recupero all'avvio) sono ancora riferite a REV00 e vanno rieseguite con gli strumenti FCL ufficiali su REV01-FIX (sulla sonda locale `stand(0.15)` viene rifiutato, in modo fail-closed). Restano da chiarire la temporizzazione di WALK (missed releases) e la decisione sui limiti di velocità 29/40 RPM. Un merge in `main` non è una validazione hardware.

## Linee di sviluppo: cosa è dove

- **`main` (linea ufficiale):** contiene la baseline del Controller `0.2.0-dev.3` (persistenza della calibrazione V1, layout flash `MATDOG_16M_2x5M_NVS_V1`, Wi‑Fi/OTA, LED, diagnostica). Il codice di movimento presente è libreria non collegata. **Una build di `main` non è il firmware installato né è stata accettata su hardware.**
- **Checkpoint RC5 (storico, conservato):** il branch `matdog/rc5-source-checkpoint-20261026` (commit `c0e82234a202238521235dd240e6071011909115`) è una fotografia sorgente della candidata RC5 `1.0.0-rc.5` (STAND/WALK semantici, dashboard di movimento, restore automatico), **verificata solo offline**. La candidata (APP `fe33fab6…`) non è mai stata flashata ed è stata superata dal lavoro RCA successivo ([#52](https://github.com/MattRobotics/robot-dog/issues/52)); la PR #54 è chiusa senza merge. Il branch resta intatto per consultazione e provenienza.
- **RCA corrente:** sviluppo ancora **non integrato** in `main` e non pubblicato (al 31/10 il sorgente più recente è `dev5` locale, vedi "Stato al 2026-10-31"). La futura integrazione firmware avrà una PR nuova, basata sul sorgente corretto e tracciabile dopo la RCA. Nessuna funzione di movimento (STAND, WALK, STOP) è considerata risolta o validata su hardware.
- **Esperimenti offline archiviati:** la pipeline IMU G5‑B (ex PR #50) e il modello inerziale REV01 (ex PR #51) non sono in `main`; i loro commit sono conservati dai tag `archive/g5b-imu-offline-20261027` e `archive/rev01-inertial-20261027` (riferimenti storici, **non** release né validazioni hardware). Le vecchie PR #47 e #49 sono chiuse: la loro storia è contenuta nel checkpoint RC5.

## Cosa è validato su hardware e cosa no

| Validato su hardware (con ambito e data) | Non validato |
|---|---|
| Funzionamento `ROBOT_POWERED` senza movimento, loop indipendente da USB (2026‑09‑18) | **STAND completo con quota misurata, WALK continuo, FAST** |
| Spegnimento con KEY, porta USB di servizio GPIO19/20 (2026‑09‑24) | Rete/dashboard disponibili durante il movimento; STOP end‑to‑end durante i blackout HTTP |
| Calibrazione a 24 contatti, 4 zampe (2026‑10‑01); SAVE/LOAD su un candidato dev.3 | Limiti operativi / envelope approvati |
| Anello LED RMT‑DMA (2026‑10‑10, ambito limitato) | Restore manuale da dashboard e criteri di rifiuto per giunto ([#48](https://github.com/MattRobotics/robot-dog/issues/48)) |
| Boot con restore automatico 12/12 e censimento 13/13 su build installate del 22–24/10 (senza movimento) | Prestazioni in tempo reale su ESP32 (tempi, jitter) |
| Prove fisiche **parziali** di STAND il 26/10 e, su build locali non pubblicate, il 28–30/10: STAND50 con carico (28/10), STAND100 e STAND150 (29–30/10), PRONE→STAND senza STOP da USB e dashboard con 0 missed release (30/10, dev.5). Ambito limitato, evidenze locali | Release firmware stabile ([#46](https://github.com/MattRobotics/robot-dog/issues/46)) |

La sessione supervisionata del 27/10 non ha avuto esito positivo (vedi [#52](https://github.com/MattRobotics/robot-dog/issues/52)); nessuna causa è stata stabilita.

**Firmware installato:** ultima verifica hardware il 30/10/2026, `1.0.1-dev.5` APP `c8b27d67…` (dettagli in "Stato al 2026-10-31"); non riverificato dopo quella data. Lo storico (diario V2) arriva al 26/10 con la APP `54ade4fd…`, solo provenienza storica. Prima di qualsiasi azione va verificata l'identità del dispositivo reale (SHA256 della APP, non il branch o `main`). Una sequenza dashboard non pianificata (STAND150, WALK 45 mm, STAND100, PRONE da 100 mm) su dev.5 ha prodotto 41 missed release: WALK non è qualificato.

## Storia e diario firmware (15 settembre – 26 ottobre 2026)

[`09_Logs/Firmware_History/`](09_Logs/Firmware_History/README.md): diario [Markdown](09_Logs/Firmware_History/MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V2_PUBLIC.md) e [Excel](09_Logs/Firmware_History/MATDOG_Diario_Sviluppo_ESP32S3_V2_2026-09-15_2026-10-26_PUBLIC.xlsx): cronologia, 43 manifest con SHA256 delle APP, criticità, decisioni e lacune; distingue prove hardware (HW‑LOG/HW‑REPORT) da risultati offline e UNKNOWN. Il vecchio README di stato (fino al 26/10) è conservato in [`09_Logs/Historical/README_until_2026-10-26.md`](09_Logs/Historical/README_until_2026-10-26.md): è evidenza storica, non stato corrente.

## Problemi aperti

- [#52](https://github.com/MattRobotics/robot-dog/issues/52) — architettura real‑time e prestazioni di WALK (STOP, servo bus, tracking).
- [#48](https://github.com/MattRobotics/robot-dog/issues/48) — RESTORE della calibrazione gen‑1 automatico e manuale da dashboard.
- [#46](https://github.com/MattRobotics/robot-dog/issues/46) — pubblicazione di una release firmware stabile dopo la validazione hardware.
- Qualificazione Motion su REV01-FIX (STAND 150 mm, ritorno a PRONE, WALK STANDARD, limiti ST3215): vedi [`ASTRA_HANDOFF.md`](09_Logs/Validation_Reports/REV01_FIX_Alignment_2026-10-31/ASTRA_HANDOFF.md) e [`OPEN_ITEMS.md`](09_Logs/OPEN_ITEMS.md).

## Sicurezza

- **Non flashare `main` sul robot attuale** e non considerare nessuna PR come autorizzazione al flash o al movimento. Vedi [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).
- `MOTION_AUTHORIZED=0` nelle baseline documentate; un risultato offline non è una prova fisica.
- Con un caricatore collegato il bus `B+/P−` resta alimentato anche con KEY spento: l'isolamento affidabile è il sezionatore fusibile.
- Mai modificare EEPROM/ID/`PositionOffset` dei servo, NVS o calibrazioni senza una procedura autorizzata; `PositionOffset = 0` è la baseline.
- Gli stati usati nei documenti (VALIDATED, IMPLEMENTED, HISTORICAL…) hanno il significato definito nello storico: HISTORICAL non è mai autorizzazione attuale.

Per agenti AI: vedi [`AGENTS.md`](AGENTS.md). Costruito e documentato da Matt Robotics.
