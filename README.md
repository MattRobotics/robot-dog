# MATDOG — quadrupede robotico personalizzato

MATDOG è la piattaforma quadrupede di Matt Robotics: un robot a quattro zampe progettato da zero, con l'obiettivo di arrivare a una **locomozione continua, precisa e sicura** (STAND, WALK, stabilizzazione IMU) comandata a livello semantico da una dashboard e, in futuro, da un computer di alto livello.

> **Stato in una riga:** il robot ha una base hardware validata (alimentazione, calibrazione a 24 contatti, diagnostica), ma **il movimento (STAND/WALK) non è validato su hardware**. Nessuna build di questo repository è autorizzata al flash sul robot attuale.

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
| [`03_CAD/`](03_CAD) | CAD, URDF, mesh |
| [`04_Electronics/`](04_Electronics/README.md) | Alimentazione, cablaggio, mappatura servo ([stati di alimentazione e ricarica](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md)) |
| [`08_Tests/`](08_Tests) | Procedure di validazione ripetibili (le suite host del firmware stanno in `05_Firmware`) |
| [`01_Docs/`](01_Docs/02_Architecture/ARCHITECTURE.md) | Architettura e [roadmap](01_Docs/02_Architecture/ROADMAP.md) |
| [`09_Logs/`](09_Logs/README.md) | Evidenze, decisioni, report di validazione, [indice storico](09_Logs/HISTORY_INDEX.md), [problemi aperti](09_Logs/OPEN_ITEMS.md) |
| [`02_BOM/`](02_BOM), [`07_Media/`](07_Media), [`10_Backups/`](10_Backups) | Componenti, media, ricevute dei backup cifrati |

## Linee di sviluppo: cosa è dove

- **`main` (questa linea):** contiene la baseline del Controller `0.2.0-dev.3` (persistenza della calibrazione V1, layout flash `MATDOG_16M_2x5M_NVS_V1`, Wi‑Fi/OTA, LED, diagnostica). Il codice di movimento presente è libreria non collegata. **Una build di `main` non è il firmware installato né è stata accettata su hardware.**
- **Linea SIMPLE / RC5 (ancora in sviluppo):** firmware `1.0.0-rc.5` con STAND/WALK semantici, dashboard di movimento e restore automatico. È proposta per `main` nella PR draft [#54](https://github.com/MattRobotics/robot-dog/pull/54) (branch `matdog/rc5-source-checkpoint-20261026`, commit `c0e82234`), **non ancora unita**. È una fotografia sorgente **verificata solo offline**; la candidata RC5 (APP `fe33fab6…`) non è mai stata flashata ed è oggetto di RCA ([#52](https://github.com/MattRobotics/robot-dog/issues/52)). Le modifiche RCA più recenti non sono nel repository.
- **Linee indipendenti, non unite:** [#50](https://github.com/MattRobotics/robot-dog/pull/50) (pipeline IMU G5‑B offline) e [#51](https://github.com/MattRobotics/robot-dog/pull/51) (modello inerziale REV01), solo analisi offline. Le vecchie PR #47 e #49 sono chiuse: la loro storia è contenuta in #54.

## Cosa è validato su hardware e cosa no

| Validato su hardware (con ambito e data) | Non validato |
|---|---|
| Funzionamento `ROBOT_POWERED` senza movimento, loop indipendente da USB (2026‑09‑18) | **STAND completo con quota misurata, WALK continuo, FAST** |
| Spegnimento con KEY, porta USB di servizio GPIO19/20 (2026‑09‑24) | Rete/dashboard disponibili durante il movimento; STOP end‑to‑end durante i blackout HTTP |
| Calibrazione a 24 contatti, 4 zampe (2026‑10‑01); SAVE/LOAD su un candidato dev.3 | Limiti operativi / envelope approvati |
| Anello LED RMT‑DMA (2026‑10‑10, ambito limitato) | Restore manuale da dashboard e criteri di rifiuto per giunto ([#48](https://github.com/MattRobotics/robot-dog/issues/48)) |
| Boot con restore automatico 12/12 e censimento 13/13 su build installate del 22–24/10 (senza movimento) | Prestazioni in tempo reale su ESP32 (tempi, jitter) |
| Prove fisiche **parziali** di STAND il 26/10 | Release firmware stabile ([#46](https://github.com/MattRobotics/robot-dog/issues/46)) |

La sessione supervisionata del 27/10 non ha avuto esito positivo (vedi [#52](https://github.com/MattRobotics/robot-dog/issues/52)); nessuna causa è stata stabilita.

**Firmware installato oggi:** *non dichiarato qui.* Lo storico (diario V2) riporta come ultimo flash documentato fino al 26/10 la APP `54ade4fd…`, ma è solo provenienza storica: prima di qualsiasi azione va verificata l'identità del dispositivo reale (SHA256 della APP, non il branch o `main`).

## Storia e diario firmware (15 settembre – 26 ottobre 2026)

[`09_Logs/Firmware_History/`](09_Logs/Firmware_History/README.md): diario [Markdown](09_Logs/Firmware_History/MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V2_PUBLIC.md) e [Excel](09_Logs/Firmware_History/MATDOG_Diario_Sviluppo_ESP32S3_V2_2026-09-15_2026-10-26_PUBLIC.xlsx): cronologia, 43 manifest con SHA256 delle APP, criticità, decisioni e lacune; distingue prove hardware (HW‑LOG/HW‑REPORT) da risultati offline e UNKNOWN. Il vecchio README di stato (fino al 26/10) è conservato in [`09_Logs/Historical/README_until_2026-10-26.md`](09_Logs/Historical/README_until_2026-10-26.md): è evidenza storica, non stato corrente.

## Problemi aperti

- [#52](https://github.com/MattRobotics/robot-dog/issues/52) — architettura real‑time e prestazioni di WALK (STOP, servo bus, tracking).
- [#48](https://github.com/MattRobotics/robot-dog/issues/48) — RESTORE della calibrazione gen‑1 automatico e manuale da dashboard.
- [#46](https://github.com/MattRobotics/robot-dog/issues/46) — pubblicazione di una release firmware stabile dopo la validazione hardware.

## Sicurezza

- **Non flashare `main` sul robot attuale** e non considerare nessuna PR come autorizzazione al flash o al movimento. Vedi [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).
- `MOTION_AUTHORIZED=0` nelle baseline documentate; un risultato offline non è una prova fisica.
- Con un caricatore collegato il bus `B+/P−` resta alimentato anche con KEY spento: l'isolamento affidabile è il sezionatore fusibile.
- Mai modificare EEPROM/ID/`PositionOffset` dei servo, NVS o calibrazioni senza una procedura autorizzata; `PositionOffset = 0` è la baseline.
- Gli stati usati nei documenti (VALIDATED, IMPLEMENTED, HISTORICAL…) hanno il significato definito nello storico: HISTORICAL non è mai autorizzazione attuale.

Per agenti AI: vedi [`AGENTS.md`](AGENTS.md). Costruito e documentato da Matt Robotics.
