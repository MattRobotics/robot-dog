> Copia del piano. I percorsi relativi (`preflash/`, `tools/`, `d3_selftest/`) si riferiscono a `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/candidate_82c1af9/`; le immagini sono in `build-output/p0-peaks-20261116/diag-82c1af9/`.

# Prossima sessione hardware — verifica sul chip di E1/E1b/P3a/P3b e D3 (candidato `82c1af9`)

Piano preparato il 16/11/2026 a robot spento. **Non autorizzato:** richiede un nuovo mandato, l'operatore presente alla fixture e lo STOP accessibile. I criteri sono fissati qui prima della prova e non si rilassano a posteriori.

## 1. Che cosa si verifica e che cosa si preserva

**Da verificare sul chip:**
- E1 / E1b / P3a / P3b: picchi di calcolo WALK sterzato, inizio della ripresa, ALIGN → DEPLOY e DEPLOY → LIFT;
- D3: se le perse a riposo seguono la dump diagnostica `@MOTION TIMING`.

**Da preservare:**
- STOP e SAFE_OFF;
- calibrazioni gen-1 e NVS (`LOAD_COUNT=4`, `VALID_ACKNOWLEDGED`, nessuna scrittura);
- mappatura servo, limiti, box;
- la build installata `46b3175`, che è il rollback esatto (verificata in ROM con `verify-flash` prima di scrivere).

**Esclusi:** FAST 40, movimenti a terra, modifiche a limiti o criteri, più di un flash.

## 2. Identità (VERIFIED OFFLINE, 16/11)

| Voce | Valore |
|---|---|
| Installato (atteso) | `46b31757eb81`, APP `57508746…f80b2`, app0 |
| Candidato | `82c1af9` (`cand/diag-p0-peaks-20261116`) = `46b3175` + E1 + P3a + E1b + P3b + D3 |
| APP | `93c61562fdcf1d79206565a9dc440debd2bc8dbd7e2fa91176d7aefd2f966ace`, 1 783 744 B, CLEAN, ROBOT_POWERED, OTA 0; tabella, bootloader e sdkconfig identici a `46b3175` |
| Verifiche | `static_audit` PASS; `simple_motion`, `stop_support`, `prone`, `night_recovery` PASS; sezioni: IRAM identica, DRAM +32 B |
| Equivalenza del Motion | patch identiche a `719912a`: 64 scenari host con le stesse sequenze; IK shadow con stato identico su 216 196 soluzioni |
| Script | `preflash/flash-qualified-app0-1.0.2-dev.8-82c1af9.sh`, autorizzazione `MATDOG_DIAG82C_FLASH_AUTHORIZED`; dry gates PASS, 4 controlli negativi REFUSE |
| Rollback | `preflash/rollback-flash-qualified-app0-1.0.2-dev.8-46b3175.sh` (autorizzazione `MATDOG_DIAG46B_FLASH_AUTHORIZED`), dry gates PASS |
| Strumenti | `tools/` (`TOOLS.sha256`): quelli del 16/11 con `EXPECT_BUILD='82c1af95e0a5'` e la nuova autorizzazione; `rom_checks.sh` verifica `46b3175` come installato; nuovo `analyze_d3.py` (provato su log sintetico, `d3_selftest/`) |

## 3. Sequenza

1. **Accensione e gate BMS.** `cold_start.sh` registra dal QON, con D1+D2. Gate: ≥ 150 poll consecutivi OK. Un silenzio in corso dopo 20 min = FAULT persistente: niente flash.
2. **Verifica in sola lettura di `46b3175`:** `EXPECT_BUILD=46b31757eb81 python3 -I preflight.py …` → 8/8; firma USB, NVS `LOAD_COUNT=4` gen-1.
3. **Registratori chiusi.** Poi `rom_entry.sh` e `rom_checks.sh`: layout app0, digest di `46b3175` uguale.
4. **Flash unico** con lo script pinnato (`verify-flash`). Poi `boot_and_record.sh`.
5. **Verifica passiva:**
   - build `82c1af95e0a5` = `ota_running_build_id`;
   - NVS `LOAD_COUNT=4` gen-1 invariata; restore 12/12 `be8d5902bb4e37da`;
   - census 13/13, IMU PASS, SAFE_OFF 8191, coppie OFF;
   - D2 presente;
   - **D3 presente:** `@MOTION TIMING` stampa `record=LAST_IDLE_SKIP` e `dump_us=`;
   - gate BMS 150 OK; preflight 8/8.
6. **Suite A–G** (`fixture_suite.py`), poi **blocco H** (`fixture_block_h.py`), come il 16/11, se A–G è 65/65 senza fault.
7. **Chiusura:** PRONE_REST, SAFE_OFF, coppie OFF, stream BMS OFF.

## 4. Criteri (fissati prima)

| Voce | Criterio | Come |
|---|---|---|
| Regressione funzionale | 65/65 + 19/19, nessun fault; ogni passo finisce come il 16/11 | `results.json`, `compare_steps.py` contro `suite_20261116_0716` |
| STOP | verifica dello STOP entro i tempi del 15–16/11 (2,2–2,6 s); ripresa a STAND_REACHED | `stop_compare` come il 16/11 |
| Qualità IMU | sterzato \|voluto − applicato\|: media e massimo non peggiori del 16/11 (1,39 / 13,3 mrad a 1 Hz). Offline E1 non cambia la statistica (0,926 → 0,922) | `imu_quality` come il 16/11 |
| Timing E1/E1b/P3a/P3b | misurati e riportati per classe e blocco, contro il 16/11: ms per release, istruzioni, picchi dal boot (`heaviest`), overrun, perse. **Nessuna soglia nuova.** Atteso offline: picco WALK sterzato più basso, RECOVERY all'inizio della ripresa più basso, OTHER DEPLOY/LIFT più basso | `analyze_diag.py` invariato |
| D3 | Ipotesi «le perse all'attivazione da PRONE_REST sono la dump diagnostica» **confermata** solo se: ogni persa a riposo è ATTRIBUITA alla finestra di una dump; nessuna persa cade nel conteggio di un'attivazione (`command_timing.missed` = 0 per le STAND da PRONE_REST, salvo l'attesa ACC della prima). Altrimenti **smentita**, con la causa che D3 indica (`MOTION_ACC_CEILING us=`, `next_release_in_us=`) | `analyze_d3.py` |
| **P0** | chiuso solo con **0 perse operative, 0 overrun, massimo ≤ 16 ms** (dev.8 §8.3, invariato). Atteso: **non chiuso** (§5) | — |
| Memoria | heap −32 B atteso; nessuna soglia nuova; confronto del minimo con il 16/11 | `/status` |

**Arresto:** FAULT, perdita BMS, stato inatteso o timeout fermano la suite. Un solo `@STOP` se il robot è attivo e non in fault. Nessuna ripresa dopo un FAULT di sicurezza o una perdita BMS.

**Rollback** solo se un criterio di sicurezza fallisce (fault ripetuto, STOP non verificato), con lo script pinnato di `46b3175`.

## 5. Problemi residui di P0 che la sessione non chiude

1. Lo stallo di istruzioni (cache) resta dominante: meno istruzioni non vuol dire 1:1 meno tempo. Il 16/11 il WALK ha −6/−12 % di istruzioni con tempo invariato.
2. Inizio della ripresa: soluzione esatta + compensazione, fino a 1 562 k istruzioni in QEMU.
3. DEPLOY → LIFT: 1 376 k istruzioni in QEMU.
4. Le release sterzate con soluzione + compensazione piena viste sul chip (27,0 ms) non sono riprodotte a quell'ampiezza negli scenari QEMU. La sessione misura se E1 le abbassa.
5. Il tempo fuori da N+C (~3,6 ms per release) consuma da solo un quarto del budget.
