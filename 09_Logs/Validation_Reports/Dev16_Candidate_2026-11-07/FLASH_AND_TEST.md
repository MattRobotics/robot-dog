# dev.16 — flash APP-only e collaudo supervisionato (PREPARATO, NON ESEGUITO)

Nulla di quanto segue è stato eseguito. Ogni passo hardware richiede l'**autorizzazione esplicita dell'operatore nella sessione attiva** e il **supporto fisico anticaduta** già posizionato. Candidata: [REPORT.md](REPORT.md).

## A. Flash APP-only (stessa procedura qualificata di dev.14 e dev.15)

Script preparato, solo in locale (contiene percorsi host):
`~/MATDOG/verification-artifacts/MATDOG_DEV16_FLASH_PREP_20261107/flash-qualified-app0-dev16.sh`.

Si rifiuta di partire senza `MATDOG_DEV16_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`. Prima di scrivere ricontrolla:
- SHA, dimensione e manifest della APP (`CLEAN`, ROBOT_POWERED, OTA0, tabella `8f756ecb…`);
- SHA del backup completo;
- piano di scrittura `matdog_layout.py check-write`.

Esegue **una** sola `write-flash` di APP0 a `0x010000` e poi un `verify-flash` indipendente. Non scrive bootloader, tabella, otadata, NVS, EEPROM, eFuse né l'altro slot.

| # | Passo | Criterio per proseguire |
|---|---|---|
| A0 | Robot sostenuto, batteria carica, solo USB-C dell'ESP32 verso l'ASUS. HTTP `/status`: `build_id=7997ad4d4497`, READY, BMS fresco (< 2 s), SAFE_OFF 8191, coppia 0 ×12 | tutti veri |
| A1 | Asset di rollback: `sha256sum -c rollback-assets.sha256` (APP dev.15 `b81d3a4f…`, backup `5cbba0b9…`, candidata `73952eec…`) | 3 × OK |
| A2 | Ingresso in ROM (unico reset): `esptool --chip esp32s3 --port $PORT --baud 115200 --no-stub --connect-attempts 1 --before usb-reset --after no-reset read-mac` | MAC `14:c1:9f:22:75:94` |
| A3 | Layout e otadata in sola lettura, con lo stesso strumento delle sessioni precedenti | `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…`, slot attivo `app0`, rollback bootloader abilitato |
| A4 | `MATDOG_DEV16_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED ./flash-qualified-app0-dev16.sh` | `Hash of data verified`, `digest matched`, `APPLICATION_ONLY_FLASH=PASS` |
| A5 | Avvio: `esptool … --no-stub --connect-attempts 1 --before no-reset --after watchdog-reset read-mac`; da qui **solo HTTP** | — |
| A6 | `/status`: `firmware_version=1.0.1-dev.16`, `build_id=e0ddbb1791d0`, READY/RUN, census 13/13, IMU/BMS/SERVO PASS, NVS gen-1 `VALID_ACKNOWLEDGED`, nessun restore, SAFE_OFF 8191 | tutti veri, altrimenti rollback |

**Rollback (APP-only, stessa procedura):** si ripete A2–A5 con `APPLICATION_BINARY` = APP dev.15 (`…/hostlink-build-7997ad4/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`, SHA `b81d3a4f…`, 1.691.328 B, manifest relativo, `SOURCE_COMMIT=7997ad4…`). Il backup completo del 10/09/2026 contiene **firmware e NVS di quella data**: si usa solo per un ripristino catastrofico, mai come rollback ordinario (sovrascriverebbe la calibrazione gen-1).

## B. Collaudo supervisionato (dopo A6)

Registrare per ogni prova la traccia Dashboard Motion e il campionatore HTTP a 1 Hz. Leggere `command_timing.overruns`, `missed`, `max_work_meta`, `overrun_releases`, `transport.loop_stack_min_free`, `transport.late`/`busy`.

| # | Prova | Atteso dev.16 | Interrompere (STOP) se |
|---|---|---|---|
| B1 | IDLE 60 s | 0 overrun, 0 persi, `loop_stack_min_free` ≥ 2 KB | fault, stack < 1,5 KB |
| B2 | STAND100 da PRONE_ENTRY, HOLD 60 s con Dashboard attiva; poi da HOLD **STAND150** e **WALK STANDARD 45** | completati; **nessun 503** `MOTION_BUDGET_UNAVAILABLE` (in dev.15 erano 2 su 2) | movimenti inattesi |
| B3 | PRONE da STAND100, poi da STAND150, poi da STAND ~120 dopo un WALK (i casi falliti in dev.15), con il supporto anticaduta sotto il corpo | `PRONE_REST`, coppia OFF 13/13; consegna a ~52 mm senza attesa | caduta o contatto non previsto. Se compare `PRONE_NOT_COMPLETED_SUPPORT_HELD` il robot è **in coppia**: non premere SAFE_OFF senza sostenerlo; provare STAND (se ammesso), registrare il JSON |
| B4 | WALK45 per 3 cicli, WALK STOP; poi WALK60 | finestra da ~84 s con **molto meno di +50** periodi persi (atteso ~0); overrun ridotti | `WALK_IK_OR_WORKSPACE`, oscillazioni anomale |
| B5 | Joystick avanti/indietro, strafe, yaw: 60 s ciascuno | funzionamento come dev.15; periodi persi e overrun minori di dev.15 (+235/+383/+429 per finestra), **non** zero | deadman o rilascio non immediati |
| B6 | STOP in HOLD e in WALK; ripresa STAND | ACK STOP ≤ 80 ms, `STOPPED_SUPPORT_UNVERIFIED`, ripresa ammessa | STOP non applicato |
| B7 | Tavola inclinata ferma in HOLD (come dev.15) | correzione e `WORKSPACE` come dev.15, nessun fault | fault |

**Criteri di accettazione per la PR #64** (decisione dell'operatore): B1–B4 e B6 senza fault né cadute; PRONE riuscito in tutti i casi di B3, oppure sostenuto con motivo esplicito e mai caduto; nessun 503 da HOLD; WALK45 senza periodi persi o con riduzione documentata. B5 serve a quantificare il residuo del joystick e a decidere l'A/B `-O2` (REPORT §4).
