# MATDOG — branch operativo unico, 14/11/2026

**Branch `integ/matdog-unified-operational-20261114`.** È l'unica linea di sviluppo operativo, accanto a `main`. Il consolidamento è solo Git: nessuna modifica funzionale al firmware, nessun flash, nessuna simulazione.

> **Aggiornamento 16/11/2026 sera:** il firmware installato è `6be46906d069` (diagnostica F3 + IRAM G2s, validata su fixture). La legge Motion qualificata di quella build è ora **integrata** in questo branch, senza strumentazione diagnostica.
> - Sintesi, identità, verifiche e recupero: [rapporto P0 F3 G2s](P0_F3_G2s_2026-11-16/REPORT.md).
> - Il primo paragrafo qui sotto descrive il consolidamento del 14/11 (solo Git); quello del 16/11 ha cambiato il firmware sorgente, per cherry-pick tracciati.

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
| **Firmware realmente installato (16/11 sera, letto sul dispositivo alle 19:45Z)** | **`6be46906d069`**, `1.0.2-dev.8` **diagnostica** = `a86c680` + F3 + IRAM G2s, APP SHA256 `be678d83037451d7937783e70d443a83612cbc992005a9f0cac9561ad0a13b68` (1 784 368 B), slot app0, flash APP-only con verify alle 19:29Z. Sorgente `6be4690`: **non** è un commit di questo branch (`cand/p0-f3-g2s-20261116`, bundle `robot-dog-g2s-20261116.bundle`); ricostruibile da `f3b0d48` con `P0_F3_G2s_2026-11-16/patches/installed_6be4690/`. NVS gen-1 e restore 12/12 invariati. Rollback esatto `a86c680` | VERIFIED HARDWARE su fixture: prova mirata 12/12, A/B 18/18, FAST E–G 20/20 (29,6 mm/s, STOP nel FAST 2,2 s), H 20/20; nessun fault; rispetto ad `a86c680` nella stessa sessione −6…−10 % per release, perse 5 → 0, overrun 6 → 0. **Timing P0 non chiuso** (2 overrun, 20,8 ms). Heap minimo 25 672 B, **margine 1 096 B** con 2 client HTTP ([rapporto](P0_F3_G2s_2026-11-16/REPORT.md)) |
| **Sorgente firmware di questo branch (16/11 sera)** | La legge Motion di `6be4690` **senza** strumentazione diagnostica: T2R1M, E1/E1b, P3a/P3b, P5, F3, IRAM G2 + G2s, cherry-pick da `f3b0d48` (§4 del [rapporto](P0_F3_G2s_2026-11-16/REPORT.md)). Build di verifica CLEAN `33134ae28260`, APP `0e07a4ac…b7b6`, **mai installata**; stringa di versione `1.0.2-dev.7` invariata | VERIFIED OFFLINE (test host, `static_audit`, gate secante STOP, suite Motion); comportamento VERIFIED HARDWARE tramite `6be4690` |
| Installato prima (16/11 pomeriggio e sera) | `a86c6800b543` (`82c1af9` + P5): A0–F2 56/56, FAULT F3 nel FAST 32 mm/s, causa la secante STOP preesistente ([P5](P0_F3_G2s_2026-11-16/sources/HW_a86c680_P5_20261116.md)); `cb9ed85` (F3 + G2): MEMORY_STOP 9 748 B con 3 client HTTP e rollback ([rapporto](P0_F3_G2s_2026-11-16/sources/HW_cb9ed85_MEMSTOP_20261116.md)). Ancora prima, **`2638204fb4ef`**, `1.0.2-dev.8` **diagnostica** = `82c1af9` + P4, APP SHA256 `79018a28eb7876b7fbc768b8558d9d8bb31788b31f9d961739633054be39c6f4`, slot app0. Sorgente `2638204`: **non** è un commit di questo branch (`cand/diag-p0-p4-20261116`). Prima: `82c1af95e0a5` ([validazione](HW_VALIDATION_82c1af9_20261116.md)), rollback esatto verificato in ROM, non eseguito — VERIFIED HARDWARE su fixture (16/11 sera): flash APP-only PASS, A–G 65/65 + H 19/19, 0 perse; overrun 47 (sessione a +5–7 % per istruzione su entrambe le build, non P4); P4 senza beneficio sulle release in overrun ([validazione](HW_VALIDATION_2638204_20261116.md)): **Timing P0 non chiuso**. Silenzio DALY di 132 s all'avvio a freddo, prima del flash ([analisi](P0_Closure_2026-11-16/DALY_EPISODE_20261116.md)) | VERIFIED HARDWARE (storico, vedi i rapporti collegati) |
| Candidato P0 (16/11, poi installato e superato da `6be4690`) | **`a86c680`** (`cand/diag-p0-p5-20261116`) = `82c1af9` + **P5** (`StopPlan::build` si ferma al primo punto di controllo fuori box; ricerca `87338f8` su `research/p0-stopplan-20261116`); **P4 esclusa**. Build CLEAN `a86c6800b543`, APP `f7bc8b62…e55b`, **non flashata**; rollback esatto `2638204` | VERIFIED OFFLINE: equivalenza bit a bit su 25 M casi e 111 750 chiamate registrate; frame identici; WALK_SETTLE rifiutata 2,6 M → 0,69 M istruzioni (QEMU); `static_audit` e 4 suite Motion PASS ([rapporto](P0_Closure_2026-11-16/REPORT.md), [piano](P0_Closure_2026-11-16/NEXT_SESSION_PLAN_a86c680.md)) |
| Picchi residui dopo T2R1M (16/11, offline) | Branch locale `research/p0-peaks-20261116` su `0fb22b0`: E1 `8346b48` (seme dell'IK di compensazione), P3a `157c95f` (`ViaPlan::plan` esatto), E1b `1b17674` (seme nella ripresa), P3b `719912a` (release del piano). Diagnostica `82c1af9` (= `46b3175` + le quattro patch + D3) **flashata e validata il 16/11 pomeriggio** (riga sopra). **P4** `e57dd89` (una sola soluzione nelle release attenuate) e candidato `2638204` (= `82c1af9` + P4): **flashato e validato il 16/11 sera** (riga sopra); P4 esclusa dalla candidata successiva. Bundle `backups/git/robot-dog-p0-peaks-20261116.bundle` | E1/E1b/P3a/P3b: VERIFIED HARDWARE tramite `82c1af9` (16/11 pomeriggio; QEMU WALK max 1 430 → 1 172 k, RECOVERY 2 156 → 1 562 k, OTHER 2 045 → 1 376 k; stato IK identico su 216 196 soluzioni). P4: VERIFIED OFFLINE (release OTHER a 2 soluzioni host 5 376 → 904, QEMU 3 917 → 697; stesse sequenze; `static_audit` e quattro suite Motion PASS su `e57dd89` e `2638204`), non provata su chip ([rapporto](P0_Peaks_2026-11-16/REPORT.md), [locomozione](P0_Peaks_2026-11-16/LOCO_EVAL.md), [misure a terra](P0_Peaks_2026-11-16/GROUND_MEASUREMENT_PLAN.md)) |
| Sorgente firmware di questo branch fino al 16/11 pomeriggio | `539b646`: Ground Contact Fase 1 sopra dev.7 `c3b67d0`, più correzione FAST (`05039ca`), Patch A (`4d4b797`), riarmo STOP (`7878be0`) e binding delle conversioni servo (`539b646`). I commit documentali successivi non cambiano il firmware | IMPLEMENTED NOT HARDWARE MOTION VALIDATED |
| IRAM G1 (15/11) | **Integrato** sul branch operativo con il merge `f3b0d48` di `cand/timing-iram-20261115` (`5eff171`, `e7b4266`, `f349af9`): build CLEAN `f3b0d48cd56d`, APP `a6baacf5…`, **non flashata**, `static_audit` PASS. La diagnostica installata `e321e86` (= `8530d3a` + la stessa patch, identica byte per byte) resta un branch locale separato (`cand/diag-8530d3a-iram-20261115`); bundle `backups/git/robot-dog-offline-dev-20261115-candidates.bundle` | VERIFIED HARDWARE tramite `e321e86` (fixture 15/11): Timing −8/−11 % per release, heap −4,6 KB, nessuna regressione ([validazione HW](HW_VALIDATION_e321e86_20261115.md), [rapporto offline](Offline_Dev_2026-11-15/REPORT.md)). Timing P0 non chiuso |
| Motion Timing P0 strutturale (15/11 sera) | Candidato di ricerca **T2R1M** `0fb22b0` (branch locale `research/p0-t2r1m-20261115`, 4 istruzioni in `SimpleMotion.cpp`, nessuno stato): la ripresa dopo STOP usa il bracket T2; la release leggera T1 vale anche mentre la correzione scorre. Alternative misurate: T2R `bb5520b`, T2R1 `4757fa0`, T1M `a62171c`. Diagnostica `46b3175` (T2R1M + strumentazione Timing dev.8 + D2 `4e3a740`), **flashata il 16/11**. Bundle `backups/git/robot-dog-p0-structural-20261115.bundle` | VERIFIED HARDWARE tramite `46b3175` (16/11): RECOVERY −40/−45 % di istruzioni come in QEMU; WALK invariato in tempo; errore IMU sterzato +16 % ([rapporto offline](P0_Structural_2026-11-15/REPORT.md), [validazione](HW_VALIDATION_46b3175_20261116.md)). Timing P0 non chiuso |
| Candidato compilato offline (storico) | `5472b65`, build `5472b65dfa28`, APP SHA256 `d8582e7d57d37e0b8afb9b1fb0f59e105d8a8de788aaaabe8302cf431f6c7a68`, manifest `7c764540…f8d8` | VERIFIED OFFLINE, **mai flashato**, superato |
| Ricerca MuJoCo | Patch V7/V7L60/C2/T3 su `b561062` + `F40T_JCB66` | VERIFIED OFFLINE nel gemello, nessuna prova hardware |
| `main` | `6838e4d`, protetto, invariato | Antenato |

- Dal 16/11 sera `README.md`, `AGENTS.md`, l'intestazione del `CHANGELOG.md` del Controller e `09_Logs/OPEN_ITEMS.md` portano in testa lo stato del 16/11 su questo branch. Il resto di quei file conserva lo stato del 10/11 di `main`, come storico.
- Le voci più recenti sono nella cronologia del `CHANGELOG.md` e in `09_Logs/Validation_Reports/README.md`.
- Decisione sull'integrazione delle ottimizzazioni Timing e sul margine di heap: [ADR-007](../../Architecture_Decisions/ADR-007_Motion_Timing_Integration_and_Heap_Margin.md).
- **Prima di qualsiasi PR verso `main`:** leggere il firmware dal dispositivo.

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
- **16/11 (P5, F3, G2/G2s, consolidamento):** archivi, bundle e SHA256 nel §6 del [rapporto](P0_F3_G2s_2026-11-16/REPORT.md).
  - Bundle `robot-dog-g2s-20261116.bundle`: `6be4690` e `b5e6691`, richiede `a86c680`.
  - Bundle `robot-dog-f3fix-iram2-20261116.bundle`: `cb9ed85`, `6146ef1` e `dfc1156`.
  - Patch del firmware installato: `P0_F3_G2s_2026-11-16/patches/`.

## Limiti

- Nessun flash e nessuna lettura del dispositivo durante il consolidamento.
- Nessuna PR verso `main`.
- Nessuna ulteriore modifica funzionale senza un nuovo mandato.
