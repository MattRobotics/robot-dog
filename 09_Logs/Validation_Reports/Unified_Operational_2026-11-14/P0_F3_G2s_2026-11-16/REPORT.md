# P5, F3, IRAM G2/G2s e consolidamento: 16/11/2026, pomeriggio e sera

Chiude la giornata del 16/11 dopo `HW_VALIDATION_2638204_20261116.md` e `P0_Closure_2026-11-16/REPORT.md`. Le copie integrali dei rapporti di sessione sono in `sources/`; qui ci sono sintesi, identità e recupero.

## Identità: non confonderle

| Identità | Valore | Stato |
|---|---|---|
| **Firmware installato sul robot** (letto dal dispositivo il 16/11 alle 19:45Z) | **`6be46906d069`**, `1.0.2-dev.8` **diagnostica** = `a86c680` + F3 + IRAM G2s. APP SHA256 `be678d83037451d7937783e70d443a83612cbc992005a9f0cac9561ad0a13b68`, 1 784 368 B, slot app0, ROBOT_POWERED, OTA ingest 0. Flash APP-only con verify alle 19:29Z | **VERIFIED HARDWARE** su fixture (§3). NVS gen-1 `LOAD_COUNT=4` `VALID_ACKNOWLEDGED`, restore 12/12 `be8d5902bb4e37da`, calibrazioni **non toccate** |
| Sorgente esatto del firmware installato | Commit `6be4690` (branch locale `cand/p0-f3-g2s-20261116`). **Non è un commit di questo branch**: è ricostruibile bit per bit da `f3b0d48` con `patches/installed_6be4690/` (19 patch, albero `8b4ec14e…` verificato) e dal bundle `robot-dog-g2s-20261116.bundle` | — |
| **Sorgente integrato in questo branch** (16/11 sera) | La legge Motion di `6be4690` **senza** la strumentazione diagnostica: 12 cherry-pick da `f3b0d48` (§4). Build di verifica CLEAN `33134ae28260`, APP `0e07a4ac0b5764076670b8310b07f44eee15d957ec33703f18f9e30ea1b9b7b6` (1 775 280 B), **non installata**; stringa di versione invariata `1.0.2-dev.7` | VERIFIED OFFLINE: test host, `static_audit`, gate secante STOP, suite Motion (§4). Il comportamento Motion è VERIFIED HARDWARE tramite `6be4690`, che ha la stessa legge |
| Rollback esatto | `a86c680` (APP `f7bc8b62…e55b`), script `c77428e1…`, verificato in ROM il 16/11 sera (digest uguale prima del flash) | pronto, non eseguito |
| `main` | `6838e4d`, invariato | — |

## 1. Pomeriggio: P5 su `a86c680` (`sources/HW_a86c680_P5_20261116.md`)

- **Flash e prove:** `a86c680` (= `82c1af9` + P5) flashato alle 14:31Z. Suite A0–F2 56/56 PASS, poi **FAULT in F3** `WALK_IK_OR_WORKSPACE`, con FAST preset 2 a 32 mm/s.
- **Esito:** P5 non responsabile, dimostrato a parità di ingressi.
- **STOP:** B4 2,2 s, D14 2,6 s, D18 2,2 s.
- **Timing:** 33 overrun, massimo 25,5 ms. Nessun rollback.
- **RCA F3** (`sources/RCA_F3_20261116.md`): il FAST preset 2 portava la secante di riferimento a 31 tick per release, contro il limite della prova STOP di 31,36 alla velocità di cammino. Il margine era preesistente: le build operative hanno la stessa legge.
- **DALY** (`sources/RCA_DALY_20261116.md`): silenzi tutti entro ≈ 335 s dal QON. Precauzione di procedura: QON + 6 min e 150 poll consecutivi validi prima di Motion.

## 2. Sviluppo offline: F3 e IRAM G2/G2s

- **F3 (`6146ef1` su `a86c680`):** falcata del FAST preset 2 da 80 a 74 mm, quindi **velocità da 32 a 29,6 mm/s**. Il picco di secante scende a 29 tick, margine +1,36 sul limite di 31,36.
  - Nuovo gate `scripts/offline_motion/run_stop_secant_gate.sh`: ogni modo, preset, direzione del joystick e transizione deve restare almeno 1 tick sotto il limite.
  - STANDARD preset 2 resta il caso più stretto (+0,36). Le etichette della dashboard dicono «30 mm/s» (29,6).
- **IRAM G2 (`3fd8758`/`dfc1156`):** 19 funzioni Motion spostate in IRAM. Costo +10 300 B di IRAM, cioè −10 240 B di heap interno (ELF). Replay QEMU: miss medi −34/−34/−36/−71/−39 %.
- **G2s (`b5e6691`):** G2 senza `contactInverseKinematicsGroundNormal` e `validGaitParameters`. 17 funzioni, costo **−6 144 B** di heap, miss −26/−27/−28/−45/−28 %.
  - Privi dell'attributo, i sorgenti sono identici ad `a86c680`.
- **Rapporti:** `sources/OFFLINE_F3FIX_IRAM_G2_20261116.md`, `sources/RCA_MEMSTOP_G2s_20261116.md`; insiemi IRAM in `results/iram_sets_eval3.txt`.

## 3. Hardware della sera

### `cb9ed85` (F3 + G2): MEMORY_STOP e rollback (`sources/HW_cb9ed85_MEMSTOP_20261116.md`)

- **Prima del flash:**
  - silenzio DALY dal primo poll per 182 s (RX idle-high), rientro spontaneo;
  - riferimento `a86c680` 28/28 PASS.
- **Flash e arresto:** dopo il flash di `cb9ed85`, `heap_min_free` è sceso a **11 144 → 9 748 B**, sotto la soglia di 24 KB, all'avvio della prima prova e prima di ogni comando Motion. La guardia ha mandato lo STOP: FAULT `STOP_REFERENCE_UNAVAILABLE`, coppie OFF.
- **Rollback** ad `a86c680`, verificato.
- **RCA:**
  - un watcher HTTP rimasto attivo (PID di `setsid`), quindi 3 client contemporanei;
  - transitorio di lwIP/Wi-Fi in heap interno: 1 client ≈ 21 KB, 2 client 26,8–28,5 KB (una volta 36,2), 3 client 50,4 KB (`results/heap_survey_all.txt`, `results/cb9ed85_min_drops.txt`);
  - causa formalmente **UNKNOWN**, ipotesi principale il terzo client;
  - nessuna regressione funzionale.

### `6be4690` (F3 + G2s): qualificato (`sources/HW_6be4690_G2s_20261116.md`)

- **Avvio:** QON alle 19:14:12Z. DALY 303/303 poll validi senza silenzi; dopo il flash 177/177.
- **Disciplina della sessione:**
  - al massimo **2 client HTTP**, imposti dal registro dei processi;
  - controllo **M0** della memoria prima di ogni Motion.
- **Riferimento `a86c680`** nella stessa sessione: prova mirata 12/12, A/B 18/18.

| Prova su `6be4690` | Esito | Note |
|---|---|---|
| Mirata D8–D11 (STAND 120, sterzata ±yaw STANDARD, rilasci, PRONE) | 12/12 | 0 perse, 0 overrun (riferimento: 5 e 3) |
| A/B (STOP a riposo, STAND 120/150/80/120/110, PRONE, yaw 22 s, **STOP B4 2,6 s**, RECOVERY B6) | 18/18 | 0 perse, 0 overrun |
| E–G (camminata STANDARD con cambi STANDARD→SLOW→STANDARD, WALK STOP; **FAST 29,6 mm/s** 15 s + 10 s, **STOP nel FAST 2,2 s**, ripresa, WALK STOP, PRONE) | 20/20 | 0 fault, 0 `STOP_PLAN` in 53 campioni HTTP del FAST; 1 overrun (HEIGHT a 2 soluzioni, 20,8 ms) |
| H (STOP in camminata ×2, **STOP dentro la RECOVERY a 1,0 s e 0,3 s**, riprese a STAND_REACHED) | 20/20 | 1 overrun (PRONE_HEIGHT a 2 soluzioni, 20,8 ms) |

**Beneficio di G2s nella stessa sessione, sugli stessi passi** (`results/6be4690_compare_*.txt`):

| Classe | `a86c680` → `6be4690` |
|---|---|
| WALK | 10,67–10,81 → 9,99 ms/release (−0,68…−0,82 ms, −6,4…−7,6 %) |
| RECOVERY | 10,57 → 9,96 (−0,61 ms, −5,8 %) |
| OTHER, STAND/HEIGHT | 8,67 → 7,84 (−0,83 ms, −9,6 %) |

- Le istruzioni sono invariate; `istall_mem` scende da 0,40 a 0,35 e, per OTHER, da 0,32 a 0,23.
- Sui 29 passi comuni: perse 5 → 0, overrun 6 → 0. Massimi: WALK 21,2 → 19,4 ms, OTHER 22,1 → 19,4 ms, RECOVERY 18,3 → 19,1 ms.
- IMU sterzata: massimo 5,02 → 3,04 mrad, media invariata.

**Memoria** (`results/6be4690_memory_candidate_runs.txt`):
- costo statico misurato −6 132 B;
- con 2 client il minimo è sceso da 41 156 a 26 540 B nei primi 2 s della prima prova (avvio di poller e watcher);
- minimo di sessione **25 672 B, margine 1 096 B** sopra la soglia di 24 KB; nessuna allocazione fallita.
- Un terzo client lo annullerebbe. La build operativa integrata ha 8 168 B di heap statico in più della diagnostica (ELF), ma non è misurata.

## 4. Consolidamento sul branch operativo

**Integrati** (cherry-pick `-x` in ordine, origine nel messaggio):

| Commit qui | Origine | Contenuto | Verifica |
|---|---|---|---|
| `4d02b80` `b72e4fd` `23577ad` | `bb5520b` `4757fa0` `0fb22b0` | T2R, T2R1, **T2R1M**: ripresa dopo STOP sul bracket T2, release leggera T1 | VERIFIED HARDWARE (`46b3175`, `82c1af9`, `a86c680`, `6be4690`) |
| `b4ce6d5` `8cad07f` `eb104cc` `b90208c` | `ae9b6d1` `db6d621` `447082c` `e6ee46b` | E1, P3a, E1b, P3b: seme dell'IK di compensazione, `ViaPlan` esatto, release del piano | VERIFIED HARDWARE (`82c1af9` e successive) |
| `69f974d` | `b104f00` | **P5**: la prova STOP rifiutata si ferma al primo controllo fuori box | VERIFIED OFFLINE (equivalenza su 25 M casi); in HW da `a86c680` e `6be4690` |
| `f886cf2` | `a0451de` (F3 `6146ef1`) | **F3**: FAST preset 2 da 74 mm (29,6 mm/s), gate secante, testi della dashboard | VERIFIED HARDWARE (`6be4690`, E–G) |
| `5e841b7` `702ce0b` `33134ae` | `e3fe5a0` `cb9ed85` `6be4690` | **IRAM G2 e G2s** (rete: 17 funzioni) e ri-pin dei sorgenti | VERIFIED HARDWARE (`6be4690`) |

- **Escluse dal cherry-pick:** le righe di composizione diagnostica in `BuildConfig.h` (di F3 e G2s). Il `BuildConfig` operativo conserva la sua storia.
- **Non integrati, conservati come candidati o diagnostica:**
  - strumentazione Timing dev.8 (`eb5d4c1`, PMU, `@MOTION TIMING` esteso);
  - D2 DALY UART2 (`867dd78`, `46b3175` per i test);
  - D3 (`82c1af9`);
  - note `BuildConfig` (`5672323`, `a86c680`, `c3bb3e0`);
  - **P4** (`785bcec`, stessa patch di `e57dd89`; candidato `2638204`): nessun beneficio misurato, `patches/excluded_P4/`;
  - **G2 completo** (`cb9ed85`): superato da G2s per il margine di memoria.
  - Tutto è in `patches/installed_6be4690/`, con SHA256.
- **Albero rispetto a `6be4690`:** differisce soltanto per quella strumentazione, la storia in `BuildConfig` e le voci recenti del `CHANGELOG` (`git diff 6be4690 33134ae`). La legge Motion è la stessa.
- **Controlli offline su `33134ae`** (`~/MATDOG/verification-artifacts/MATDOG_1_0_2_CONSOLIDATION_20261116/checks/`):
  - `run_host_tests.sh` PASS (le righe FAIL sono le mutazioni volute);
  - `static_audit` PASS, `MOTION_CONVERGENCE_SOURCE` PASS;
  - `run_stop_secant_gate.sh` PASS (FAST 29, margine +1,36; STANDARD +0,36);
  - suite Motion PASS: `simple_motion` 138 s, `stop_support` 46 s, `prone` 135 s (dev.15 replay 4/4 PRONE_REST), `night_recovery` 264 s (`results/motion_suites_33134ae.txt`);
  - build di verifica ROBOT_POWERED CLEAN: `.iram0.text` 92 743 B, `_heap_start` 0x3fcbd960.

## 5. Problemi aperti

- **Motion Timing P0 aperto.**
  - Su `6be4690` restano 2 overrun e release oltre 16 ms: STAND/HEIGHT/PRONE attenuate a 2 soluzioni (20,8 ms, COMPENSATE 10,5–12 ms); picchi WALK e RECOVERY 19,5–20 ms.
  - G2s abbassa le medie del 6–10 %, non i picchi sotto 16 ms.
- **Memoria:** con 2 client il margine è 1 096 B sulla build diagnostica. Va rispettato il limite di 2 client nelle sessioni e misurato il margine della build operativa prima di un flash.
- **DALY:** silenzi all'accensione (RX idle-high, fino a 182 s; uno anche dal primo poll). Causa **UNKNOWN**.
  - Manca la misura decisiva: ricevitore RS485 in solo ascolto su A/B durante un silenzio.
  - Il LED dell'XY-017 lampeggia anche a controller spento, quindi non è discriminante.
- **Non validato:**
  - qualunque prova a terra delle build dev.8;
  - blocchi C e D della suite su `6be4690` (eseguiti su `a86c680` il pomeriggio e su build precedenti; F3 e G2s non cambiano quei frame, VERIFIED OFFLINE);
  - la build operativa integrata `33134ae` sul robot.

## 6. Recupero

- **Rimettere sul robot il firmware installato:** script `flash-qualified-app0-1.0.2-dev.8-6be4690.sh` (SHA256 `6f369ae1…`, variabile `MATDOG_P0F3G2S_FLASH_AUTHORIZED`), immagine in `~/MATDOG/build-output/g2s-20261116/p0-f3-g2s-6be4690/`.
- **Ricostruire il sorgente:** `git checkout f3b0d48 && git am patches/installed_6be4690/*.patch` (albero `8b4ec14e…` = `6be4690`).
- **Rollback:** `rollback-flash-qualified-app0-1.0.2-dev.8-a86c680.sh` (SHA256 `c77428e1…`, variabile `MATDOG_DIAGP5_FLASH_AUTHORIZED`).
- **Strumenti di sessione:** `../timing_tools/` (registro dei processi, `run_acquisition.py`, M0, guardia, prove offline in `sim_memstop/`).

**Archivi fuori dal repository** (ASUS, `~/MATDOG/backups/`), SHA256:

| Archivio | SHA256 |
|---|---|
| `verification-artifacts/MATDOG_1_0_2_DIAG_A86C680_HW_20261116.tar.gz` | `1815c7fc1175b684fc9ad9a87f6609a5ded95f903c344511fd48d8c95c577594` |
| `verification-artifacts/MATDOG_1_0_2_F3FIX_IRAM2_20261116_light.tar.gz` | `9cec99de8d2776b50deb7b74cded3736617393ea6e68a1a3047b798bc70989b7` |
| `verification-artifacts/MATDOG_1_0_2_P0F3G2_HW_20261116.tar.gz` | `2a0a333c70fe18075ad10b50640badcbede73a7b9c2739c32c5dfb068338b61f` |
| `verification-artifacts/MATDOG_1_0_2_G2_MEMSTOP_RCA_20261116.tar.gz` | `18bd388e8e31d601122f6bb594743d718e52f06281f043e9df5e026f2ea7def3` |
| `verification-artifacts/MATDOG_1_0_2_G2S_HW_20261116.tar.gz` | `65ef306276c855025980d9044fa0403c005c2a6f805b608eb5cdfc1fa6302e68` |
| `verification-artifacts/MATDOG_1_0_2_P0_CLOSURE_20261116_light.tar.gz` | `e422d91e43f3648e53d3ef39a1c4642cf99ba2a1a74a8573b217ce86f1c4f945` |
| `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116_light.tar.gz` | `55a6327f2ba7e9174954ec99f68d1ceed4f45835585849526504e5321bbf61bc` |
| `verification-artifacts/MATDOG_1_0_2_DIAG_2638204_HW_20261116.tar.gz` | `7f050feb172f41c407633725d58e242e8c3d73209980d0aa3f033b8e6fe62b8e` |
| `verification-artifacts/MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116.tar.gz` | `b8fec9782d87b0bc4e6190b2f51db59f27544c73910f8936192cc1f9e9a70ee5` |
| `verification-artifacts/MATDOG_1_0_2_DIAG_46B3175_HW_20261116.tar.gz` | `f108bee3a8e7adb2e98b3dd315449bcb81d11836ec7c1771cfed048d5e9a1d16` |
| `git/robot-dog-g2s-20261116.bundle` (`6be4690`, `b5e6691`) | `36041e8807226bcc6c8a99e3f49226c91465e5f681846a1dc7eac5e6a35fb113` |
| `git/robot-dog-f3fix-iram2-20261116.bundle` (`cb9ed85`, `6146ef1`, `dfc1156`) | `212a44a7ba689a2fd1bcc0f24601125e0370343972393eec7c14156b123a615e` |
| `git/robot-dog-p0-closure-20261116.bundle` | `e3874c080f8841bb98e44cb7eab64c5288029b4e5c621849dc11a6d43d8ba80e` |
| `git/robot-dog-p0-peaks-20261116.bundle` | `96dc02615a43b024e755708b4c5ca2f0f51e37853c3c6c733666c51e23e5bc2e` |
| `git/robot-dog-p0-structural-20261115.bundle` | `ccc33c0e7376975dd628cb3087ca2f48d998df7bb08091d95c841001002ed0f4` |
| `git/robot-dog-offline-dev-20261115-candidates.bundle` | `c1e87724f5af30860d7d77adf90bc440a5c7023d23b66fec3eb8252ec613684c` |

- Le cartelle di sessione complete restano sigillate in `~/MATDOG/verification-artifacts/`, ognuna con il proprio `SHA256SUMS.txt`.
- Nel repository non entrano log seriali, dump di trace, immagini APP né credenziali.
- I manifest delle build e gli SHA delle APP sono in `manifests/`.
