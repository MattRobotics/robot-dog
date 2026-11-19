> Copia del piano. I percorsi relativi (`preflash/`, `tools/`, `image/`) si riferiscono a `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/candidate_2638204/`; le immagini sono in `build-output/p0-peaks-20261116/diag-2638204/`.

# Prossima sessione hardware (eventuale) — candidato `2638204` = `82c1af9` + P4

Preparato il 16/11/2026 a robot spento. **Non autorizzato:** richiede un nuovo mandato. Stessa procedura e stessi criteri della sessione `82c1af9` (`MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116/README.md` e `PLAN_FROM_CANDIDATE.md` §4); qui solo le differenze.

## Identità (VERIFIED OFFLINE)

| Voce | Valore |
|---|---|
| Installato (atteso) | `82c1af95e0a5`, APP `93c61562…6ace`, validato sulla fixture il 16/11 (A–G 65/65, H 19/19) |
| Candidato | `2638204` (`cand/diag-p0-p4-20261116`) = `82c1af9` + P4 `785bcec` (cherry-pick identico a `e57dd89`) + commento di composizione |
| APP | `79018a28eb7876b7fbc768b8558d9d8bb31788b31f9d961739633054be39c6f4`, 1 783 680 B, CLEAN, ROBOT_POWERED, OTA 0; tabella, bootloader e sdkconfig identici; IRAM e DRAM identiche a `82c1af9`, `.flash.text` −60 B |
| Script | `preflash/flash-qualified-app0-1.0.2-dev.8-2638204.sh`, autorizzazione `MATDOG_DIAG263_FLASH_AUTHORIZED`; dry gates PASS; 4 controlli negativi REFUSE; rifiuto senza autorizzazione |
| Rollback | `82c1af9` (`rollback-…-82c1af9.sh` = script che l'ha installato), dry gates PASS; secondo livello `46b3175` |
| Strumenti | `tools/` (`TOOLS.sha256`): quelli del 16/11 pomeriggio con `EXPECT_BUILD='2638204fb4ef'`; `rom_checks.sh` verifica `82c1af9` come installato |
| Verifiche offline | `static_audit` e quattro suite Motion: `audit/static_audit_diag_2638204.txt`, `audit/motion_suites_2638204.txt`. P4 sul branch di ricerca: host 64 scenari con le stesse sequenze, QEMU 46/46 frame = host, release OTHER a 2 soluzioni 3 917 → 697 |

## Che cosa misura in più

- **P4:**
  - effetto sulle release attenuate di STAND, PRONE e HEIGHT, che il 16/11 pomeriggio davano 18 overrun su 31 (20–22 ms, 2 soluzioni);
  - atteso: `max_work_solves` 2 → 1 nella maggior parte di quelle release e massimo per comando sotto i 20–22 ms del 16/11. Nessuna soglia nuova.
- **WALK_SETTLE dopo un rilascio sterzato** (D5, D9, D11; fino a 26,8 ms con 4 IK):
  - nessuna patch, perché la causa non è dimostrata;
  - la rotazione dei gruppi PMU nel record `MOTION_PMU_HEAVIEST` può dare, in un'altra sessione, il conteggio delle istruzioni (gruppo 0) di quel tipo di release;
  - serve a separare istruzioni in più da stalli.
- **D3:** come il 16/11, durata delle dump e eventuali perse a riposo.

## Criteri

Quelli del §4 del piano `82c1af9`, invariati, con il confronto contro `MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116/suite_20261116_0924` e `h_20261116_0932`.

- Il criterio IMU (media e massimo sterzati non peggiori) resta riferito a `46b3175`: **1,39 / 13,30 mrad a 1 Hz**. Il 16/11 pomeriggio `82c1af9` non lo ha soddisfatto sul massimo (13,87); il riferimento non si sposta su quel valore.
- P0 chiuso solo con 0 perse operative, 0 overrun, massimo ≤ 16 ms.
