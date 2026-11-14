# Installazione diagnostica `8530d3a11ac9` — 14/11/2026 sera

**DIAGNOSTIC INSTALLED — PASSIVE BOOT VERIFIED — MOTION VALIDATION PENDING.**

Un solo flash APP-only autorizzato dall'operatore, seguito solo da verifiche passive. Nessun movimento e nessun comando Motion. Nessuna scrittura su NVS, bootloader, tabella partizioni, otadata, BMS o calibrazioni.

## Identità e provenienza

| Voce | Valore |
|---|---|
| Installato prima | `0526cf6b9d79` (diagnostica del 14/11 pomeriggio), confermato via USB e con `verify-flash` in ROM |
| Installato ora | `8530d3a11ac9`, `1.0.2-dev.8` diagnostica, APP 1 781 424 B, SHA256 `75fdd3acb3849929b98c5637b8ed8de84de53ad1c6fd93bced2e08b61e0c4d4b`, slot app0 `0x010000` |
| Sorgente del binario | Commit `8530d3a11ac9f25b4d39fc6811d6aa99564a54a5` = `539b646` di questo branch + strumentazione Timing dev.8u di `0526cf6` (contatori `be9198d`). **Non pubblicato e da non fondere**: bundle `~/MATDOG/backups/git/robot-dog-diag-8530d3a.bundle` |
| Firmware del branch | `539b646` (Patch A, correzione FAST, riarmo STOP, binding delle conversioni): IMPLEMENTED NOT HARDWARE MOTION VALIDATED. Gli HEAD documentali successivi non cambiano il firmware |
| Rollback | `0526cf6` (APP `495ffd50…742fd5`) o dev.7 `b561062` (APP `28744982…53d4`), script pinnati nelle evidenze del 14/11; backup integrale 16 MiB `5cbba0b9…` |

## Esiti (VERIFIED HARDWARE PASSIVE)

- **Preflight (in ROM, sola lettura):** ESP32-S3 rev v0.2, MAC `14:c1:9f:22:75:94`; tabella `8f756ecb…`, app0 attiva; APP installata uguale a `0526cf6`.
- **Stato prima del flash:** READY, SAFE_OFF 8191, coppie OFF, DALY `comm=OK`, NVS gen-1 `VALID_ACKNOWLEDGED`.
- **Flash:** scrittura `0x010000..0x1c2eb0` in 41 s, hash verificato, `verify-flash` con digest uguale al candidato.
- **Boot** (watchdog-reset):
  - identità: build `8530d3a11ac9`, `GIT_DIRTY=NO`;
  - NVS gen-1 invariata; calibrazione ripristinata 12/12 con coppie OFF;
  - census 13/13; IMU PASS;
  - DALY `comm=OK` con campi D1 presenti;
  - READY senza fault, SAFE_OFF 8191, `torque_active=false`;
  - diagnostica Timing inizializzata (`diag_preemption=1`, elenco task);
  - AP del robot attivo; nessun panic, watchdog o brownout.
- **8 min passivi:**
  - 227/227 poll DALY `comm=OK` (`rx_bytes=129`, `stale_bytes=0`, `fail_streak=0`, primo byte 10–24 ms);
  - uptime monotono, LED solo BOOTING → READY;
  - SAFE_OFF e coppie OFF costanti, 0 release perse.

## Limiti e prossimo passo

- 8 minuti non dimostrano la fine dell'intermittenza DALY (causa UNKNOWN).
- Timing P0 non risolto finché non ci sono misure in movimento.
- Validazione Motion da autorizzare: riarmo STOP → STAND sulla fixture; sequenza del 14/11 (STAND 110, yaw STANDARD, STOP, STAND 120) con `@MOTION TIMING`; FAST sterzato solo dopo.

Evidenze: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_P0CLOSE_DIAG_INSTALL_20261114/INSTALL_RESULT.md` (+ `SHA256SUMS.txt`). Offline: `MATDOG_1_0_2_MOTION_P0_CLOSURE_20261114/REPORT.md`.
