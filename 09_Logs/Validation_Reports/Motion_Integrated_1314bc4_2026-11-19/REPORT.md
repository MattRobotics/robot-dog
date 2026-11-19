# MATDOG — candidato Motion integrato `1314bc4`: build CLEAN pronta per il collaudo (19/11/2026)

Lavoro solo offline:
- **Non toccati:** robot, porte seriali, dashboard; controllo DALY a freddo di Fable non interrotto.
- **Non eseguiti:** flash, merge in `main`.
- **Nessuna nuova analisi né ottimizzazione:** questa è l'integrazione e la verifica di quanto validato il 18/11.

## 1. Commit del candidato integrato

**`1314bc411c589c4216e95629f4665d9d01f05781`**, branch `cand/ksteer-slow-recert-20261118` (pubblicato).

| Commit | Contenuto |
|---|---|
| `30c894a0d4303d497d750cccb5fd12f347c61d7d` | Baseline operativa: ACQUIRE v2. Contiene `85bad0e` (PlanSplit V2, fix STOP→STAND, writer HTTP, validDefinition) e, prima, T1/T2, IRAM G2s (25 funzioni) e F3 |
| `eb1ba28d9ad32a794ec6afc1e2adff80c01c02fe` | P1 minima: ricalcolo dell'alzata del raccordo 10 → 50 µm, solo `Gait.cpp` |
| `05e1214b4010f65794d8e038366d5a3e36251bb8` | Nucleo FP32 |
| `1314bc411c589c4216e95629f4665d9d01f05781` | `kSteer` SLOW ricertificato: un limite più la prova |

**Non incorporati:** W1, W2/B, P1 completa, varianti della regola del 2 % e della tolleranza di Newton, tuning HTTP.

**Invariati:** geometria, calibrazioni, NVS, limiti, STOP, feedback, conversione RAW e protezioni operative.

## 2. APP CLEAN, SHA256 e manifest

| Voce | Valore |
|---|---|
| APP | `dd16bd6215d836f8dae6389df7b13b8fcdcff20bdec06ca9948a34ae9b0dd82a`, 1 777 952 B (33,9 % di app0) |
| Manifest | `SOURCE_COMMIT=1314bc41…`, `BUILD_ID=1314bc411c58`, `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, ROBOT_POWERED, OTA ingest 0, `FW_VERSION=1.0.2-dev.7`, `BUILD_UTC=2026-11-19T05:27:14Z` |
| Partizioni | Tabella `8f756ecb…`, uguale a `30c894a` e `05e1214` |
| Bootloader e sdkconfig | Identici a `05e1214` |
| Build | `~/MATDOG/build-output/motion-integrated-20261119/1314bc4/artifacts/ROBOT_POWERED-ota0/` (bin, ELF, map, manifest, sdkconfig) |
| Backup | `~/MATDOG/backups/esp32/app_images/1314bc4/` (SHA256SUMS verificato) |

**Identità distinta da `05e1214`** (APP `75ac737d…`):
- il binario contiene il nuovo limite di RF UPPER di `kSteer` (1,004873916 rad) **1 volta** e il vecchio (0,963456434) **0 volte**;
- in `05e1214` è il contrario;
- differiscono 134 byte: costante, identità, data di build.

## 3. Differenze

| Contro | Differenza |
|---|---|
| `05e1214` | Firmware: solo `src/simple/SteerCollisionData.h` (massimo di RF UPPER 0,963456 → 1,004874 rad, `kSteerProof`). Il resto sono le evidenze in `09_Logs`. Codice macchina Motion identico: 8 714 istruzioni, 587 FPU, stesse chiamate. Frame QEMU identici nei 5 scenari |
| `30c894a` | 3 commit, 19 file: P1, FP32 e `kSteer` (`checks/diffstat_vs_30c894a.txt`) |

## 4. Verifiche offline

| Verifica | Esito | Nota |
|---|---|---|
| Identità e sorgenti | CLEAN@`1314bc4` prima e dopo la build | |
| `static_audit` | **PASS** | Stesso commit, eseguito il 18/11 |
| Gate di convergenza | **PASS** | Rieseguito oggi |
| `run_simple_motion_tests` | PASS | Oggi |
| `run_stop_support_tests` | PASS | Oggi |
| `run_prone_tests` | PASS | Oggi |
| Gate del secante STOP | PASS | Stesso commit, 18/11 |
| Matrice di perturbazioni | PASS, 0 fault | Stesso commit, 18/11 |
| Inviluppo del joystick SLOW/STANDARD/FAST | **PASS** in tutti e tre | `random_3` +33,9 mrad; 18/11 |
| Certificato `kSteer` (FCL) | 120/120 + 12/12, limite inferiore 3,085 mm | `kSteerProof` = SHA256 del certificato `06c38f8d…` |
| Boot in QEMU del firmware reale | Nessun panic in 60 s | Messaggi identici a `30c894a`; QEMU non emula la PSRAM octal |
| Costruttori globali | 0 istruzioni FPU | Nessun rischio di `Coprocessor exception` all'avvio |
| Banco QEMU Motion (5 scenari) | Nessun panic | Digest dei frame identici a `05e1214` |

**Non ripetuto:** la campagna FP32 dei 38 153 frame, perché i sorgenti numerici sono identici a `05e1214`.

## 5. Memoria, P0 (QEMU) e regressioni

**Memoria (ELF):**
- `.iram0.text` 89 323 B (`30c894a`: 92 743);
- `.dram0.bss` 117 808 B;
- `_heap_start` `0x3fcbca40`: **~3,9 KB di heap interno in più** rispetto a `30c894a`.

**Stack (QEMU):** minimo libero del task Motion 8 388 B (`30c894a`: 7 476).

**P0 in QEMU** (CCOUNT deterministico, **non** tempo hardware; il limite di 16 ms non è dichiarato raggiunto):

| Misura | `30c894a` | `05e1214` | `1314bc4` |
|---|---:|---:|---:|
| Release > 280 k (B3/D8/H/E/S) | 163/230/140/116/224 | 0 | 0 |
| Release più pesante | 403–529 k | 248–269 k | 248–270 k |
| Doppia compensazione, senza piano | 354 231 | 91 067 | 91 060 (−74 %) |
| NOMINAL + COMPENSATE | 439 865 | 110 857 | 110 822 (−75 %) |
| WALK sterzato con > 4 IK | 439 865 | 110 857 | 110 822 (−75 %) |

Contributi già misurati il 18/11:
- P1 riduce la frequenza delle doppie IK sterzate;
- FP32 riduce il lavoro delle tre classi del 74–75 %;
- `kSteer` corregge `random_3` in SLOW e non cambia il costo.

**Regressioni:** nessuna.

## 6. Pacchetto APP-only e rollback

**Script:** `preflash/flash-qualified-app0-1.0.2-dev.7-1314bc4.sh`, autorizzazione `MATDOG_INTEG_1314BC4_FLASH_AUTHORIZED`.
- Deriva da quello qualificato di `30c894a`: solo 26 righe di pin cambiate.
- Scrive solo app0 `0x010000..0x1c2120`, cancellazione fino a `0x1c3000`.
- NVS `0xfe0000..0xff0000`, bootloader e tabella delle partizioni non toccati.
- Gate offline PASS; rifiuta senza autorizzazione.

**Rollback a `30c894a`:**
- script `812a2e83…`, già usato sul robot il 18/11;
- gate offline PASS;
- backup `app_images/30c894a` verificato.

**Procedura HW:** `hw_session/PROCEDURE.md`, con gli strumenti del 18/11 adattati solo nelle pin.
- **Prerequisito:** esito del controllo DALY a freddo di Fable.
- **Misure:** tempo massimo e distribuzione per classe; overrun e release perse; STOP USB e HTTP; START/STAND, HEIGHT e PRONE; WALK STANDARD e sterzato; qualità della compensazione e continuità dei frame.
- Riferimento: la catena `1148` di `30c894a`.

## 7. Blocker reali

**Nessun blocker offline.**

Vincoli, che non sono difetti del candidato:
- il P0 richiede la misura hardware;
- lo STOP HTTP richiede l'accesso amministratore dell'operatore alla dashboard;
- lo SLOW sterzato è qualificato solo offline, salvo il passo facoltativo S9;
- il firmware BMS è quello di `30c894a`: l'installazione dipende dall'esito del controllo DALY a freddo in corso.

## 8. Giudizio

**PRONTO PER COLLAUDO HARDWARE: SÌ.**

**Autorizzazione al flash: NON concessa.** Va data nella sessione attiva, per questo firmware, dopo la valutazione dell'esito DALY. Quella del flash A/B BMS non vale.
