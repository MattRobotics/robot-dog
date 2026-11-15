# MATDOG — branch operativo unico, 14/11/2026

**Branch `integ/matdog-unified-operational-20261114`.** È l'unica linea di sviluppo operativo, accanto a `main`. Il consolidamento è solo Git: nessuna modifica funzionale al firmware, nessun flash, nessuna simulazione.

## Contenuto e provenienza

| Filone | HEAD originario | Come entra |
|---|---|---|
| Battery P1 + Motion vNext, candidato `1.0.2-dev.7` | `c3b67d0` (`integ/motion-vnext-battery-p1-dev7-20261111`), release `b561062` | antenato di `38e599a` |
| Ground Contact Sensorless Fase 1 (diagnostica passiva, dashboard, test) | `38e599a` (`codex/ground-contact-phase1-20261113`), codice `5472b65` | base del branch |
| Ricerca MuJoCo: M6, FAST 40, RCA 01, vNext, identificazione di C | `f13d6b9` (`sim/mujoco-dev7-correlation-20261112`) | merge ordinario `e2e3d75` |

- Il ramo MuJoCo porta con sé `main` `6838e4d` (PR #68 gemello V1, PR #69 documenti): ora `main` è antenato di questo branch.
- I tre HEAD originari sono antenati del branch: nessun commit perso, verificato con `git merge-base --is-ancestor` e `git rev-list`.
- Fuori da `06_Software/Matdog_MuJoCo` il merge cambia solo documenti Markdown. I sorgenti firmware sono identici a `38e599a`; l'albero MuJoCo è identico a `f13d6b9`.
- Nel merge ci sono stati due conflitti, entrambi documentali (`CHANGELOG.md` del Controller e `09_Logs/Validation_Reports/README.md`). Sono stati risolti conservando tutte le voci.
- V7, V7L60, CRAWL C2 e TROT T3 sono in `06_Software/Matdog_MuJoCo/gait_study/vnext/patches/` come patch di ricerca, con la base `F40T_JCB66.patch` e `SHA256SUMS`. **Non sono applicate al firmware ESP32-S3.**

## Identità: non confonderle

La stringa `1.0.2-dev.7` non identifica da sola un firmware: la portano sia l'immagine `b561062` sia la build Ground Contact `5472b65`.

| Identità | Valore | Stato |
|---|---|---|
| Firmware realmente installato (15/11 pomeriggio, letto sul dispositivo) | **`e321e8668139`**, `1.0.2-dev.8` **diagnostica** con IRAM G1, APP SHA256 `b88c448d8b95f44fb851d727bcb50d1438b6bc8b61918b3f7d7fcebb4b8847b2`, slot app0. Sorgente `e321e86` = `8530d3a` + IRAM G1 (`277f3c1`, `b570dd9`, `e321e86`): **non** è un commit di questo branch (branch locale `cand/diag-8530d3a-iram-20261115`, bundle `robot-dog-offline-dev-20261115-candidates.bundle`). Prima: `8530d3a11ac9` ([installazione](DIAG_INSTALL_8530d3a.md), [validazione 15/11 mattina](HW_VALIDATION_8530d3a_20261115.md)); rollback pronto, non eseguito ([validazione IRAM e RCA DALY](HW_VALIDATION_e321e86_20261115.md)) | VERIFIED HARDWARE su fixture (15/11 pomeriggio): flash APP-only PASS, suite A–G 65/65 in un'unica esecuzione; Timing −8/−11 % per release, perse 28 → 10, overrun 648 → 323, picco 36,1 ms: **Timing P0 non chiuso** |
| Sorgente firmware di questo branch | `539b646`: Ground Contact Fase 1 sopra dev.7 `c3b67d0`, più correzione FAST (`05039ca`), Patch A (`4d4b797`), riarmo STOP (`7878be0`) e binding delle conversioni servo (`539b646`). I commit documentali successivi non cambiano il firmware | IMPLEMENTED NOT HARDWARE MOTION VALIDATED |
| IRAM G1 (15/11) | **Integrato** sul branch operativo con il merge `f3b0d48` di `cand/timing-iram-20261115` (`5eff171`, `e7b4266`, `f349af9`): build CLEAN `f3b0d48cd56d`, APP `a6baacf5…`, **non flashata**, `static_audit` PASS. La diagnostica installata `e321e86` (= `8530d3a` + la stessa patch, identica byte per byte) resta un branch locale separato (`cand/diag-8530d3a-iram-20261115`); bundle `backups/git/robot-dog-offline-dev-20261115-candidates.bundle` | VERIFIED HARDWARE tramite `e321e86` (fixture 15/11): Timing −8/−11 % per release, heap −4,6 KB, nessuna regressione ([validazione HW](HW_VALIDATION_e321e86_20261115.md), [rapporto offline](Offline_Dev_2026-11-15/REPORT.md)). Timing P0 non chiuso |
| Candidato compilato offline (storico) | `5472b65`, build `5472b65dfa28`, APP SHA256 `d8582e7d57d37e0b8afb9b1fb0f59e105d8a8de788aaaabe8302cf431f6c7a68`, manifest `7c764540…f8d8` | VERIFIED OFFLINE, **mai flashato**, superato |
| Ricerca MuJoCo | Patch V7/V7L60/C2/T3 su `b561062` + `F40T_JCB66` | VERIFIED OFFLINE nel gemello, nessuna prova hardware |
| `main` | `6838e4d`, protetto, invariato | Antenato |

- `README.md`, `AGENTS.md` e l'intestazione del `CHANGELOG.md` del Controller vengono da `main` e descrivono lo stato del 10/11.
- Le voci più recenti sono nella cronologia del `CHANGELOG.md` e in `09_Logs/Validation_Reports/README.md`.
- Vanno riallineati in un passaggio documentale prima di qualsiasi PR verso `main`, dopo aver letto il firmware dal dispositivo.

## Obiettivi vincolanti per lo sviluppo successivo

Requisiti della build stabile finale. Questo consolidamento non li implementa.

- **Battery P1:** DALY/BMS correttamente inizializzato anche dopo un'accensione a freddo.
- **Timing Motion:** entro il criterio operativo, senza regressioni.
- **STAND:** da 0 a 150 mm secondo il workspace realmente qualificato.
- **WALK STANDARD:** multidirezionale e fluida.
- **WALK FAST:** obiettivo 40 mm/s, postura più eretta, maggiore clearance effettiva dei piedi.
- **STOP e modi:** STOP affidabile; cambi di modo e ripresa funzionanti.
- **Ground Contact Calibration completa:**
  - probing su tutte e quattro le zampe (LF, RF, LH, RH);
  - 3–5 cicli release/touchdown per zampa;
  - valutazione della ripetibilità.

Vincoli sul probing e sul perimetro della build:

- La Fase 1 sensorless passiva non costituisce il completamento del probing.
- Nessuna esclusione permanente delle zampe posteriori.
- Lo studio offline dà margini del COM negativi di circa 5–6 mm sui tre appoggi a STAND 100–120 mm quando si esclude una posteriore (`Ground_Contact_Phase1_2026-11-13/REPORT.md`).
- Per qualificare il probing bisogna dimostrare un trasferimento del carico o un sostegno sicuro prima del sollevamento, conservando STOP e limiti fisici. Nessun movimento non qualificato.
- CRAWL e TROT restano obiettivi successivi e non bloccano la build stabile.
- La Jetson è una linea indipendente, per la cognizione. L'ESP32-S3 conserva l'intera autorità Motion real-time.

## Evidenze e recupero

- **Bundle Git:** `~/MATDOG/backups/git/robot-dog-unified-operational-20261114.bundle`.
  - Contiene questo branch e i tre nomi storici, che puntano a `c3b67d0`, `38e599a` e `f13d6b9`.
  - È incrementale: richiede `main` `6838e4d`.
- **Bundle precedenti:** `robot-dog-integ-dev7-20261111-c3b67d0.bundle` e `robot-dog-sim-mujoco-dev7-correlation-20261112-dddaeed.bundle`.
- **Artefatti in `~/MATDOG/verification-artifacts/`:**
  - `MATDOG_MUJOCO_VNEXT_LOCO_20261113/` (punto d'ingresso `HANDOFF_VNEXT_F2.md`);
  - `MATDOG_MUJOCO_FAST40_20261112/`;
  - `MATDOG_MUJOCO_RCA01_MU_20261113/`;
  - `MATDOG_GROUND_CONTACT_PHASE1_20261113/`.
- **Sessione HW dell'11/11:** `~/MATDOG/backups/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz`.
- **Backup leggero vNext:** `~/MATDOG/backups/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113_checkpoint_light.tar.gz`.
- **Worktree locale:** `~/MATDOG/worktrees/matdog-unified-operational-20261114`.

## Limiti

- Nessun flash e nessuna lettura del dispositivo durante il consolidamento.
- Nessuna PR verso `main`.
- Nessuna ulteriore modifica funzionale senza un nuovo mandato.
