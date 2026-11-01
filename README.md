# MATDOG — quadrupede robotico personalizzato

**[Ultimo firmware MATDOG pubblicato — 1.0.1-dev.7 | Baseline validata su hardware | Pre-release](https://github.com/MattRobotics/robot-dog/releases/tag/matdog-controller-v1.0.1-dev.7)**

MATDOG è la piattaforma quadrupede di Matt Robotics: un robot a quattro zampe progettato da zero, con l'obiettivo di arrivare a una **locomozione continua, precisa e sicura** (STAND, WALK, stabilizzazione IMU) comandata a livello semantico da una dashboard e, in futuro, da un computer di alto livello.

> **Stato in una riga:** STAND, PRONE, WALK STANDARD, STOP e Digital Twin sono **collaudati su hardware** con il firmware `1.0.1-dev.7` (sessione supervisionata del 31/10/2026). È una baseline verificata e una pre-release: non è ancora una release stabile, e l'analisi delle prestazioni continua.

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

Cinque identità distinte: non vanno confuse.

| Identità | Valore |
|---|---|
| **Firmware installato e collaudato** | `1.0.1-dev.7`, flashato APP-only il 31/10/2026 con readback identico |
| **APP validata** | SHA-256 `4c7e2c8a6e41fbd218cb09834c6543e3f039c349089cf02a7fc8bceded5bcc3c`, 1.641.808 B, asset della [Release](https://github.com/MattRobotics/robot-dog/releases/tag/matdog-controller-v1.0.1-dev.7) |
| **Commit sorgente installato** | `4634de8007bd086ce07f8874119f47569f76ea1c`, tag `matdog-controller-v1.0.1-dev.7` |
| **`main`** | Sorgenti integrati dalla PR [#60](https://github.com/MattRobotics/robot-dog/pull/60). `src/` è identico al commit installato, ma il commit è diverso e una build di `main` non è il binario validato |
| **Calibrazione NVS gen-1** | Invariata, SHA-256 `6d03b9dd804f470b26f0516b4093e733112b7bf58eb27989162798e6a00d3edd`. Il dump resta solo locale |

Geometria: URDF **REV01-FIX** `ce63034e…`, 17 STL canoniche, unica geometria fisica. La base storica REV00 di calibrazione resta in `03_CAD/URDF/historical_calibration_basis/rev00/`. Quota nominale definita una sola volta: 46,5 mm sotto gli assi HIP anteriori.

Report del collaudo: [`Motion_dev7_HW_2026-10-31`](09_Logs/Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md).

## Linee di sviluppo: cosa è dove

- **`main`:** linea ufficiale, contiene lo stack Motion SIMPLE della dev.7.
- **Rollback locali:** dev.6 `98c4a82` (APP `5760eeee…`) e dev.5 `df43feb` (APP `c8b27d67…`). I checkpoint, con NVS di riferimento e APP, sono su disco locale e non nel repository.
- **Storici:** il checkpoint RC5 è conservato dal tag `archive/rc5-source-checkpoint-20261026`; anche gli altri tag `archive/*` restano conservati.

## Cosa è validato su hardware e cosa no

| Validato su hardware | Non validato o aperto |
|---|---|
| Sessione dev.7 del 31/10/2026: STAND diretto tra quote 50–150 mm in salita e discesa | Restore manuale da dashboard e criteri di rifiuto per giunto ([#48](https://github.com/MattRobotics/robot-dog/issues/48)) |
| STAND da PRONE, PRONE da 50 e 150 mm, PRONE → STAND senza STOP | Picchi di lavoro per release fino a 36,9 ms, causa non spiegata ([#52](https://github.com/MattRobotics/robot-dog/issues/52)) |
| WALK STANDARD 45 mm via USB e con dashboard + Twin, 60 mm via dashboard, WALK STOP | Stabilità WALK: margine di 3,7 mm solo da modello, CoM reale ignoto |
| STOP e SAFE_OFF 13/13; restore automatico gen-1 al boot | Scorrimento reversibile degli anteriori in STAND (55 mm), cedimento anteriore di 2–3 mm in WALK |
| Dashboard e Digital Twin REV01-FIX, sincronizzato a vista | `PRONE_CONTACT_UNVERIFIED`: nessun SAFE_OFF automatico; luce ventre 10,5 mm misurata |
| **0 release perse e 0 fault** in tre cammini (dev.6: 8 solo USB, 58 nell'intera sessione) | WALK FAST e aumento di velocità; correzione PRONE su terreno inclinato misurato |
| Calibrazione a 24 contatti (2026-10-01), spegnimento con KEY, LED RMT-DMA, funzionamento `ROBOT_POWERED` | Limiti ST3215 oltre quelli operativi |

Tre cammini sono un risultato netto, non ancora una statistica. `static_audit` resta FAIL con 25 rilievi obsoleti rispetto all'architettura SIMPLE, e `run_host_tests.sh` completa compila sorgenti rimossi: vedi [`OPEN_ITEMS.md`](09_Logs/OPEN_ITEMS.md).

## Storia e diario firmware (15 settembre – 26 ottobre 2026)

[`09_Logs/Firmware_History/`](09_Logs/Firmware_History/README.md): diario [Markdown](09_Logs/Firmware_History/MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V2_PUBLIC.md) e [Excel](09_Logs/Firmware_History/MATDOG_Diario_Sviluppo_ESP32S3_V2_2026-09-15_2026-10-26_PUBLIC.xlsx): cronologia, 43 manifest con SHA256 delle APP, criticità, decisioni e lacune; distingue prove hardware (HW‑LOG/HW‑REPORT) da risultati offline e UNKNOWN. Il vecchio README di stato (fino al 26/10) è conservato in [`09_Logs/Historical/README_until_2026-10-26.md`](09_Logs/Historical/README_until_2026-10-26.md): è evidenza storica, non stato corrente.

## Problemi aperti

- [#52](https://github.com/MattRobotics/robot-dog/issues/52) — tempo reale e prestazioni di WALK: picchi Motion, poi efficienza, stabilità e velocità.
- [#48](https://github.com/MattRobotics/robot-dog/issues/48) — RESTORE manuale da dashboard e criteri di rifiuto.
- [#46](https://github.com/MattRobotics/robot-dog/issues/46) — release firmware stabile: la dev.7 è una pre-release.
- Elenco completo: [`OPEN_ITEMS.md`](09_Logs/OPEN_ITEMS.md).

## Sicurezza

- **Non flashare build di `main` sul robot.** Il firmware validato è l'APP della Release, installata solo con la procedura APP-only e con un operatore presente. Nessuna PR o Release autorizza da sola un flash o un movimento. Vedi [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).
- `MOTION_AUTHORIZED=0` indica soltanto che non esistono permessi a compilazione e che al boot non parte alcun movimento: l'ammissione reale è `ready` a runtime. Un risultato offline non è una prova fisica.
- Con un caricatore collegato il bus `B+/P−` resta alimentato anche con KEY spento: l'isolamento affidabile è il sezionatore fusibile.
- Mai modificare EEPROM/ID/`PositionOffset` dei servo, NVS o calibrazioni senza una procedura autorizzata; `PositionOffset = 0` è la baseline.
- Gli stati usati nei documenti (VALIDATED, IMPLEMENTED, HISTORICAL…) hanno il significato definito nello storico: HISTORICAL non è mai autorizzazione attuale.

Per agenti AI: vedi [`AGENTS.md`](AGENTS.md). Costruito e documentato da Matt Robotics.
