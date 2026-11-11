# Handoff per la prossima sessione (scritto l'11/11/2026 sera)

## Stato del dispositivo

- Robot **spento con la chiave** (19:55:33Z). Prima dello spegnimento: PRONE_REST, SAFE_OFF verificato, mask 8191, coppie OFF.
- Ultima batteria letta: 11,9 V, cella min 3971 mV, SOC 92 %, nessun allarme.
- Configurazione **senza Jetson Orin**.
- Nessun processo di acquisizione attivo: registratore USB chiuso con `QUIT`, watcher HTTP fermato.

## Firmware installato (senza ambiguità)

| Voce | Valore |
|---|---|
| Versione / BUILD_ID | `1.0.2-dev.7` / `b5610620d0ce` |
| Commit | `b5610620d0cef40cdc34bd62bf6256857bc9cfde` |
| APP SHA256 | `28744982e293d5a56a0246c7ccacd083d1a0040f70ece29da4ac72fbfe6053d4` (app0 `0x010000`) |
| Binario e manifest | `~/MATDOG/build-output/integrated-dev7-20261111/build-b561062/artifacts/ROBOT_POWERED-ota0/` + copia nell'archivio (`firmware_identity/`) |
| Script APP-only | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z/flash/flash-qualified-app0-1.0.2-dev.7-b561062.sh` (`MATDOG_DEV7_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`) |

I commit di sola documentazione successivi a `b561062` **non** sono il firmware installato.

**Rollback:** `450d2c7` (`1.0.2-dev.6`, APP `82122e06…4256`) con `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/flash/flash-qualified-app0-1.0.2-dev.6-450d2c7.sh` (`MATDOG_DEV6_ACC220_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`):
1. liberare la seriale;
2. ingresso in ROM;
3. eseguire lo script.

Backup dell'intera flash solo per emergenza: `~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin` (SHA256 `5cbba0b9…fd32`). Mai il `merged.bin` da 16 MB: sovrascrive l'NVS.

## Repository

- Worktree `~/MATDOG/worktrees/matdog-integrated-dev7-20261111`, branch `integ/motion-vnext-battery-p1-dev7-20261111`. Contiene il sorgente della build più il commit di documentazione con questa directory.
- Remoto `origin` = `MattRobotics/robot-dog`, branch omonimo (vedi il commit e il push nella chiusura della sessione).
- `main` non toccato. Nessuna PR, nessun tag, nessuna release.
- Branch di origine rimasti invariati:
  - `feat/motion-multidirectional-vnext-20261111` @ `a7f5c2c`;
  - `fix/battery-p1-20261111` @ `f5b9d32`;
  - `proposal/acc85-runtime` @ `450d2c7`.

## Evidenze

- Archivio `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV7_HW_20261111T1849Z.tar.gz`, SHA256 `43e2d972537d8df610c527c91473cccf3257c993e5cb087da87f08c3f23f3c69`.
- Copia in `~/MATDOG/backups/verification-artifacts/`, sullo stesso disco: da copiare sul disco esterno quando è collegato.
- Dettagli: [EVIDENCE_INDEX.md](EVIDENCE_INDEX.md).

## Decisioni già prese

1. dev.7 è VALIDATED HARDWARE nel perimetro provato, NOT STABLE: [HW_REPORT.md](HW_REPORT.md).
2. Il cedimento statico delle anche misurato (≤ 4 tick) riduce il rischio residuo dello STOP in avanti. Gli STOP in avanti a terra sono passati nei tre modi.
3. Priorità di sviluppo: P0 timing → P1 cambio da FAST → P2 STANDARD 25 mm/s → P3 spostamento laterale → P4 temperature ([OPEN_ITEMS.md](OPEN_ITEMS.md)).
4. Il disturbo IMU delle 19:34:34–36Z è chiuso: riposizionamento manuale per il cavo USB.
5. Nessun gate procedurale nuovo nel firmware. Nessuna modifica a calibrazioni, NVS, limiti o EEPROM servo.
6. La dev.8 non è iniziata.

## Obiettivo prioritario: P0 timing (offline)

**Primo passo consigliato:** ricostruire offline la release più lunga a terra (ripresa STAND_FROM_STOPPED_WALK dopo lo STOP in yaw STANDARD, 19:44:49–19:45:01Z; e la camminata sterzata STANDARD delle 19:48:03Z) e misurare dove va il tempo:
- sezioni nominal e compensate;
- conteggio di solve, IK, Newton e FK, confrontati col tempo a muro.

Da lì si trova la prima divergenza e si scrive una patch minima.

Dati: `usb/http.jsonl` (`timing.max_work`, `max_work_meta`, `command_timing`, `work_hist`) e `extracts/ground2_summary.txt`.

## Cosa non ripetere

- Fixture F0–F3, F6, M1 e gli STOP a terra già passati, salvo che una patch tocchi quel percorso.
- Indagine sul disturbo IMU delle 19:34.
- Ipotesi sul cambio da FAST con il rig a inclinazione statica: già fatta, output identico su `450d2c7` e `b561062`, non riproduce il caso FAST→STANDARD.
- Flash di prova della dev.7: è già installata.

## Criteri di successo della prossima sessione

- P0: causa misurata (non presunta) e patch minima con regressioni offline PASS (`test_simple_motion`, `servo_plant`, `static_audit`, dashboard). Su hardware, nella stessa sequenza a terra: ≤ 16 release perse in 30 s, nessuna release oltre 46,7 ms, STOP e ripresa invariati.
- Ogni nuova build: commit CLEAN, nuovo BUILD_ID, script APP-only con pin e rollback a `b561062` pronto.
