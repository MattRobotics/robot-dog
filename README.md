# MATDOG — quadrupede robotico di Matt Robotics

MATDOG è un robot a quattro zampe progettato da zero. Il Controller ESP32-S3 trasforma i comandi della dashboard in traiettorie e comandi coordinati ai servo; raccoglie feedback e gestisce STOP e protezioni. Il Digital Twin mostra la posa misurata. Il joystick direzionale continuo è ora operativo nella dev.15; i prossimi obiettivi sono PRONE affidabile, timing real-time, gait più rapido e integrazione futura della Jetson, mantenendo l'autorità degli attuatori sull'ESP32-S3.

**Al 06/11/2026 il robot ha installata la dev.15, build `7997ad4d4497`**, flash APP-only verificato sulla partizione attiva e **validazione hardware PARZIALE**. STAND 50–150 mm, WALK STANDARD, joystick 3DOF proporzionale, Digital Twin (incluso Reset vista frontale +X) e risposta IMU sono stati osservati sul robot reale. **PRONE è intermittente**: due completamenti fino a `PRONE_REST` con torque OFF 13/13, ma due abort documentati ~53 mm con SAFE_OFF e un ulteriore fallimento riferito dall'operatore. Il timing Motion supera ripetutamente 20 ms, e la locomozione è lenta. **Non è una release stable.**

[Report hardware dev.15](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [Manifest evidenze e SHA256](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/EVIDENCE_MANIFEST.md) · [Audit video](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/VIDEO_AUDIT.md) · [Piano prestazioni 2×–5×](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/PERFORMANCE_PLAN.md) · [RCA timing 20 ms](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md) · [Problemi aperti](09_Logs/OPEN_ITEMS.md) · [Report storico dev.14](09_Logs/Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) · [Candidata dev.16 (non flashata)](09_Logs/Validation_Reports/Dev16_Candidate_2026-11-07/REPORT.md)

## Hardware e architettura

| Parte | Configurazione effettiva |
|---|---|
| Controller | ESP32-S3, unica autorità su Motion, bus servo e sicurezza runtime |
| Attuatori | 12 ST3215 delle zampe più collo ID51: 13 installati, 17 posizioni canoniche allocate |
| Sensori | IMU BNO085, BMS DALY, feedback di posizione, carico, tensione e temperatura dei servo |
| Interfaccia | Dashboard web dell'ESP32; comandi semantici STAND, PRONE, WALK STANDARD e STOP; Twin in sola lettura |
| Motion | Stack SIMPLE: gait, cinematica inversa, compensazione IMU, frame coordinati e feedback asincrono |
| Geometria | URDF operativo REV01-FIX; mapping e calibrazione persistente gen-1 preservati |
| Alto livello | Jetson Orin Nano Super previsto per integrazioni successive; nessuna autorità diretta browser → servo |

## Stato operativo dev.15

| Funzione | Risultato verificato e limiti |
|---|---|
| STAND | STAND100 e STAND150 completati; STAND50 completato nel seguito, con risalita automatica PRONE→STAND100. |
| WALK STANDARD | WALK45 e WALK60 eseguiti sul robot reale, WALK STOP operativo. La cadenza è bassa e le deadline a 20 ms sono ripetutamente perse. |
| Joystick 3DOF | Avanti/indietro, strafe, yaw nei due sensi e composizioni proporzionali effettivamente provati. Rilascio del comando corretto; assestamento verso STAND fino a ~8,4 s, troppo lento. |
| PRONE terminale | **INTERMITTENTE / NON QUALIFICATO**: due `PRONE_REST` con torque OFF13/13; due timeout verificati a ~52 mm con caduta al rilascio; terzo caso riferito senza JSON. Necessarie RCA fase e gestione sicura del supporto. |
| Stabilizzazione IMU | Pitch/roll attivi e più reattivi secondo operatore; ampiezza roll limitata da `WORKSPACE`. Correzione applicata ≠ livellamento assoluto del corpo. |
| STOP | Accessibile e verificato: STOP in STAND/WALK può lasciare coppia ON se supporto non verificato; dopo fault PRONE conferma SAFE_OFF 13/13 ma il fault resta latched. |
| Digital Twin | Visualizzato all'inizio della scheda Motion, segue la posa reale; Reset vista **+X verificato hardware**. L'operatore riferisce appoggio grafico corretto durante STAND; restano da qualificare completo PRONE/WALK swing, rendering mobile e stati stale/offline della issue #62. |
| NVS / BMS | gen-1 `VALID_ACKNOWLEDGED`, census 13/13, READY; BMS fresco in dev.15. Restore manuale da dashboard non qualificato. |
| LED / termica | Sfarfallamento policromatico intermittente aperto; picchi termici isolati da indagare, non prova di surriscaldamento continuo. |
| HostLink / Jetson | Bridge/USB non qualificati Motion hardware; Jetson prevista, **non installata**. |

Le evidenze sono specifiche di questa sessione; l'hardware non è ancora qualificato per operatività stabile o locomozione veloce. I 15 JSON Motion grezzi sono pubblicati come archivio con SHA256 ([raw/](09_Logs/Validation_Reports/Dev15_HW_2026-11-06/raw/)); il video domestico resta locale, identificato nel manifest.

## Riferimenti da non confondere

| Linea | Significato |
|---|---|
| **main** | Sorgente remoto integrato, basato sulla linea dev.7 più documentazione. Non contiene dev.15; main non è l'APP installata. La [PR #64](https://github.com/MattRobotics/robot-dog/pull/64) (draft, non mergiata) integra documentazione **e** sorgente dev.15. |
| **dev.14** | Firmware installato e validato parzialmente **il 05/11**, ora storico; sorgente sul [checkpoint](https://github.com/MattRobotics/robot-dog/tree/checkpoint/matdog-dev14-hw-20261105). |
| **dev.15** | **Firmware installato e parzialmente validato hardware il 06/11**. Build `7997ad4d4497`, APP SHA256 `b81d3a4ffaa27a7690b196e77e7174c0cc7851563fcfd64cd3bddb40e62f9e13`. Il sorgente esatto `7997ad4` è pubblicato sul branch `docs/dev15-hardware-validation-20261106` (PR #64) tramite il merge `e5b0644`: tree firmware identico a `7997ad4`, **non su main**. Una ricompilazione non è dichiarata byte-identica all'APP installata. |
| **dev.16** | **Candidata IMPLEMENTED NOT VALIDATED HARDWARE, non flashata** (07/11): commit `e0ddbb1`, APP `73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`, nella PR #64. Corregge la consegna PRONE a quota intermedia, aggiunge diagnostica timing (T0) e riuso IMU a release alterne (T1). Il firmware installato resta dev.15. |
| **dev.7** | [Pre-release storica pubblicata](https://github.com/MattRobotics/robot-dog/releases/tag/matdog-controller-v1.0.1-dev.7), con [report del31/10](09_Logs/Validation_Reports/Motion_dev7_HW_2026-10-31/REPORT.md). Non è più il firmware installato e non viene promossa a stable. |

Identità storica dev.14 (05/11): commit `4ff2aadf4ed907d39f2b3ad4f9e6af54db5decee`, build `4ff2aadf4ed9`, profilo ROBOT_POWERED, OTA ingest0. APP SHA256: `71028f95a8aee4b4ff7237f9420eec6aeeb0efa0aee5c00c00ca5a7bb947bd22`. Il tag conserva il sorgente; non è una nuova GitHub Release e non garantisce una ricompilazione bit-identica.

## Priorità aperte

- **[#52 — Motion](https://github.com/MattRobotics/robot-dog/issues/52):** PRONE intermittente con rilascio pericoloso della coppia a ~53 mm, fault recovery, costo `COMPENSATE`/deadline 20 ms e `MOTION_BUDGET_UNAVAILABLE` via HTTP, cadenza e sway WALK, risposta ROLL limitata e velocità utile da aumentare (target evolutivo fino a 5×, **non validato**).
- **[#62 — Dashboard, Twin e LED](https://github.com/MattRobotics/robot-dog/issues/62):** Reset vista frontale +X **risolto e verificato dall'operatore sulla dev.15**; LED intermittente e altri criteri UI restano aperti. Non chiudere l'intera issue.
- **[#48 — RESTORE e manutenzione](https://github.com/MattRobotics/robot-dog/issues/48):** automatico gen-1 verificato; percorso manuale, diagnosi di rifiuto e modalità manutenzione ancora da qualificare.
- **[#46 — Stable futura](https://github.com/MattRobotics/robot-dog/issues/46):** richiede ancora integrazione sorgente/binario corrispondente, RCA e qualifica Motion prima della release.

## Orientarsi nel repository

- [Architettura](01_Docs/02_Architecture/ARCHITECTURE.md), [firmware](05_Firmware/README.md), [software e Twin](06_Software), [CAD](03_CAD/README.md), [elettronica e alimentazione](04_Electronics/README.md).
- [Report e log](09_Logs/README.md), [cronologia](09_Logs/HISTORY_INDEX.md), [problemi aperti](09_Logs/OPEN_ITEMS.md).
- [Storia firmware fino al26/10](09_Logs/Firmware_History/README.md) e [snapshot documentali precedenti](09_Logs/Historical/Dev14_Documentation_Closeout_2026-11-05/README.md). Documenti originali conservati; le loro condizioni sono riferite alle rispettive date.

## Operazioni e sicurezza

Non usare il solo nome di un branch per scegliere un firmware da installare: servono artefatto, manifest, SHA e procedura APP-only supervisionata. La pubblicazione di sorgenti o documentazione non autorizza flash o movimenti. Nessun movimento al boot; l'ammissione Motion è governata a runtime dall'ESP32.

Con caricatore collegato il bus può restare alimentato anche con KEY OFF. Preservare STOP, limiti, NVS e mapping; nessuna scrittura EEPROM/offset servo fuori da procedure autorizzate. [Avvertenze flash e relativo contesto storico](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) · [stati di alimentazione](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).

Per agenti AI: [AGENTS.md](AGENTS.md).
