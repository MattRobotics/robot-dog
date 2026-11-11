# dev.7 HW 11/11/2026 — indice delle evidenze

Report: [HW_REPORT.md](HW_REPORT.md). I log grezzi **non** sono nel repository: restano nell'archivio locale, verificato con gli SHA256 qui sotto.

## A. Archivio grezzo

| Voce | Valore |
|---|---|
| Directory originale | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z/` (51 MB, 138 file + `INVENTORY_SHA256.txt`) |
| Archivio | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz`, 12 101 662 B |
| SHA256 archivio | **`43e2d972537d8df610c527c91473cccf3257c993e5cb087da87f08c3f23f3c69`** (anche in `….tar.gz.sha256`) |
| Seconda copia | `~/MATDOG/backups/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz` (+ `.sha256`): `sha256sum -c` OK, `cmp` identico. **Stesso disco fisico** (`/dev/sda2`): protegge da cancellazioni accidentali, non da un guasto del disco. Il disco esterno `/mnt/verbatim-hd` non era montato |
| Verifica | archivio estratto in una directory temporanea: `diff -r` identico all'originale; tutti i 138 hash di `INVENTORY_SHA256.txt` verificati sui file estratti |
| Manifest | `INVENTORY_SHA256.txt` (dimensione, SHA256 e percorso di ogni file); `MANIFEST_SHA256.txt` (51 file, scritto alla chiusura dei registratori alle 19:56Z, ancora valido); `firmware_identity/SHA256SUMS.txt` |

Ricontrollo: `cd ~/MATDOG/verification-artifacts && sha256sum -c MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz.sha256`.

## B. Log e file dell'archivio

| File | Contenuto | Intervallo (UTC) | SHA256 |
|---|---|---|---|
| `usb/serial.log` | seriale USB completo (46 500 righe; `usb_session.py`, unico proprietario della porta) | 18:52:12–19:55:33 | `10cb2315…3b51` |
| `usb/http.jsonl` | 4 551 risposte: `/api/motion/status` 1 Hz, `/status` 5 s (`http_watch.py`) | 18:52:27–19:55:55 | `5b5b96f8…a956` |
| `usb/events.jsonl` | 61 eventi di comando del registratore (solo diagnostica e `QUIT`, poi `session_end`) | 18:52:12–19:55:54 | `f6513508…b046` |
| `usb/watch_events.jsonl` | 58 `@MOTION DIAG` accodati automaticamente su WORKSPACE persistente | 18:56:26–19:29:26 | nell'inventario |
| `usb/ctl.d/*.sent` | comandi consegnati al registratore | — | nell'inventario |
| `usb_session.console`, `http_watch.console` | console dei registratori (ultime righe: `USB SESSION END (Motion untouched)`; PRONE_REST 19:49:36) | — | nell'inventario |
| `flash/` | script APP-only con pin, ingresso ROM, layout, `flash.log` (verify-flash), boot | 18:50:58–18:52:12 | nell'inventario |
| `pre/`, `h0/` | preflight sulla dev.6 installata e H0 sulla dev.7 | 18:49–18:56 | nell'inventario |
| `*_summary.txt`, `m1_flex.txt`, `ground_timeline.txt`, `ground_directions.txt` | analisi per fase (`session_summary.py`, script inline) | per fase | nell'inventario; copie in `extracts/` |
| `firmware_identity/` | APP, manifest, ELF e partizioni della build installata; APP, manifest e script di rollback `450d2c7` | — | `SHA256SUMS.txt` |
| `offline_repro_tools/` | sonde host per il cambio di modo e la loro uscita rifatta l'11/11 sera | — | nell'inventario |
| `SESSION.md`, `SESSION_CLOSURE.md`, `TIME_RANGES.md`, `HW_REPORT.md` | note di sessione, chiusura, intervalli, bozza di report (superata da questo report, stessi esiti) | — | nell'inventario |

Intervalli per fase: [extracts/TIME_RANGES.md](extracts/TIME_RANGES.md).

## C. Estratti nel repository (`extracts/`, 180 KB, controllati: nessuna credenziale, IP o MAC)

Riepiloghi per fase (`f1…f6`, `fixture_full_summary`, `ground_summary`, `ground2_summary`, `h0_summary`), `m1_flex.txt`, `ground_directions.txt`, `ground_timeline.txt`, `pre_offline_gates.txt`, `flash_verification_lines.txt`, `FIRMWARE_IDENTITY.md`, `SESSION_CLOSURE.md`, `TIME_RANGES.md`, `offline_modechange/` (sonda, script di build, uscite su `450d2c7` e `b561062`).

## D. Dalle affermazioni alle prove

| Affermazione (HW_REPORT) | Prova originale | Estratto |
|---|---|---|
| APP installata = `28744982…` | `flash/flash.log` (righe «Hash of data verified», «digest matched», `APPLICATION_ONLY_FLASH = PASS`); banner in `usb/serial.log` | `flash_verification_lines.txt`, `h0_summary.txt` |
| Tetto ACC 220 verificato | `usb/serial.log` 18:56:13 e 19:31:14 `MOTION_ACC_CEILING verified=220` | `f1_summary_a.txt`, `ground_summary.txt` |
| 7 STOP fixture verificati | `usb/http.jsonl` (`stop.motion_verified`, `torque_active`) | `fixture_full_summary.txt` §STOP |
| 7 STOP a terra verificati | `usb/http.jsonl` 19:35:30, 19:42:19, 19:42:50, 19:43:28, 19:44:49, 19:47:24, 19:49:06 | `ground_summary.txt`, `ground2_summary.txt` §STOP; `ground_timeline.txt` |
| Cambio da FAST pendente sulla fixture | `usb/http.jsonl` 19:17:39–19:19:06 (`walk_mode` selected/active/target, `imu_correction`) | `f4a_summary.txt`; `offline_modechange/` |
| FAST→SLOW a terra in 5 s | `usb/http.jsonl` 19:47:15–19:47:20 | `ground2_summary.txt` §CAMBI |
| M1 cedimento ≤ 4 tick | `usb/http.jsonl` HOLD 19:31:26–19:39:03 (`joints[].raw`, `goal`) | `m1_flex.txt` |
| Direzioni e velocità a terra | `usb/http.jsonl` (`walk_velocity`, `walk_mode.active`) | `ground_directions.txt` |
| Margine HIP ≥ 5,3 tick | `usb/http.jsonl` (`telemetry.actual_rad`) | `ground*_summary.txt` §ANCHE |
| Release perse 43 in 30 s; ripartizione per stato | `usb/http.jsonl` (`missed_releases`, `timing.max_work`) | `ground2_summary.txt` §TEMPI; numeri per stato e sezione calcolati da `usb/http.jsonl` l'11/11 sera (HW_REPORT §6 A1) |
| Letture di temperatura isolate | `usb/http.jsonl` (`joints[].temperature`, `thermal.last`) | `ground*_summary.txt` §TEMPERATURE |
| Batteria | `usb/http.jsonl` `/status` (`bms_*`, `led_*`) | `fixture_full_summary.txt`, `ground*_summary.txt` |
| Chiusura e KEY OFF | `usb/http.jsonl` 19:49:36 (PRONE_REST, mask 8191); `usb/serial.log` 19:55:33 brownout; `usb/events.jsonl` `session_end` | `SESSION_CLOSURE.md` |

## E. Riferimenti fuori dal repository

- Piano e scheda preflight: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_INTEGRATED_20261111/PREFLIGHT_CARD.md`, `REPORT.md`, `evidence/`.
- Build: `~/MATDOG/build-output/integrated-dev7-20261111/build-b561062/artifacts/ROBOT_POWERED-ota0/`.
- Rollback: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/flash/flash-qualified-app0-1.0.2-dev.6-450d2c7.sh`; backup dell'intera flash `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin`, SHA256 `5cbba0b9…fd32`.
