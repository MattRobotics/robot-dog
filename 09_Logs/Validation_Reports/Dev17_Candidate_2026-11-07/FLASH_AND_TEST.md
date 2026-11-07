# dev.17 — flash APP-only e collaudo minimo (PREPARATO, NON ESEGUITO)

Nulla di quanto segue è stato eseguito. Ogni passo hardware richiede l'**autorizzazione esplicita dell'operatore nella sessione attiva** e il **supporto fisico anticaduta** già posizionato (non dedotto dalla telemetria). Candidata: [REPORT.md](REPORT.md). Stessa procedura qualificata di dev.14, dev.15 e dev.16.

## A. Flash APP-only

Script preparato, solo in locale (contiene percorsi host):
`~/MATDOG/verification-artifacts/MATDOG_DEV17_FLASH_PREP_20261107/flash-qualified-app0-dev17.sh`.
Si rifiuta di partire senza `MATDOG_DEV17_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED` (verificato: `REFUSE`). Prima di scrivere ricontrolla SHA, dimensione e manifest della APP (`CLEAN`, ROBOT_POWERED, OTA ingest 0, tabella `8f756ecb…`), SHA del backup completo e piano `matdog_layout.py check-write` (solo `app0` 0x010000–0x1b0c70). Esegue **una** `write-flash` di APP0 e poi un `verify-flash` indipendente. Non scrive bootloader, tabella, otadata, NVS, EEPROM, eFuse né l'altro slot.

| # | Passo | Criterio per proseguire |
|---|---|---|
| A0 | Robot sostenuto, batteria carica, solo USB-C dell'ESP32 verso l'ASUS. HTTP `/status`: `firmware_version=1.0.1-dev.16`, `build_id=e0ddbb1791d0`, READY, BMS fresco, SAFE_OFF 8191, coppia 0 ×12 | tutti veri |
| A1 | `sha256sum -c rollback-assets.sha256`: APP dev.16 `73952eec…` (rollback), backup `5cbba0b9…`, candidata `f49b531a…` | 3 × OK |
| A2 | Ingresso in ROM (unico reset): `esptool --chip esp32s3 --port $PORT --baud 115200 --no-stub --connect-attempts 1 --before usb-reset --after no-reset read-mac` | MAC `14:c1:9f:22:75:94` |
| A3 | Layout e otadata in sola lettura, stesso strumento delle sessioni precedenti | `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…`, slot attivo `app0`, rollback bootloader abilitato |
| A4 | `MATDOG_DEV17_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED ./flash-qualified-app0-dev17.sh` | `Hash of data verified`, `digest matched`, `APPLICATION_ONLY_FLASH=PASS` |
| A5 | Avvio: `esptool … --no-stub --connect-attempts 1 --before no-reset --after watchdog-reset read-mac`; da qui **solo HTTP** | — |
| A6 | `/status`: `firmware_version=1.0.1-dev.17`, `build_id=7503bfc5e595`, READY/RUN, census 13/13, IMU/BMS/SERVO PASS, NVS gen-1 `VALID_ACKNOWLEDGED`, nessun restore, SAFE_OFF 8191 | tutti veri, altrimenti rollback |

**Rollback (APP-only, stessa procedura):** ripetere A2–A5 con `APPLICATION_BINARY` = APP dev.16 (`~/MATDOG/build-output/dev16-stabilization-20261107/build-e0ddbb1/artifacts/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`, SHA `73952eec…`, 1.694.048 B, manifest `SOURCE_COMMIT=e0ddbb1…`, sola lettura) usando lo script già qualificato `MATDOG_DEV16_FLASH_PREP_20261107/flash-qualified-app0-dev16.sh`. Il backup completo del 10/09/2026 contiene **firmware e NVS di quella data**: solo per ripristino catastrofico, mai come rollback ordinario (sovrascriverebbe la calibrazione gen-1).

## B. Collaudo minimo (dopo A6)

Registrare per ogni prova la traccia Dashboard Motion e, in parallelo, `/status` e `/api/motion/status` a 1 Hz da terminale. Leggere `transport.status_service_max_us`, `late`/`busy`, `command_timing.overruns`/`missed`, `loop_stack_min_free`, `trace.poll_errors`, `trace.late_dropped`. **Non ripetere** prove PRONE/STAND già positive in dev.16 senza causa.

| # | Prova | Accettazione dev.17 | Interrompere (STOP) se |
|---|---|---|---|
| B1 | IDLE 60 s con Dashboard aperta; `curl /status` 30 volte | 0 overrun/persi; **0 × 503** su `/status`; `status_service_max_us` registrato; `loop_stack_min_free` ≥ 2 KB | fault, stack < 1,5 KB |
| B2 | STAND100, HOLD 120 s con Dashboard aperta; `curl /status` 30 volte durante HOLD | **0 × 503** `MOTION_BUDGET_UNAVAILABLE`; nessun banner di budget; `poll_errors` solo come TIMEOUT a 3,5 s (non a 800 ms); telemetria regolare (età snapshot < 1,5 s salvo perdite isolate) | movimenti inattesi |
| B3 | Ricarica della pagina in HOLD con coppia ON | Twin ricompare entro pochi secondi senza torque OFF (cache calda); con cache svuotata: messaggio "coppia ON: solo dalla cache…", ricaricato una volta a torque OFF; STOP sempre utilizzabile | Twin blocca la pagina > 1 s |
| B4 | Joystick tenuto premuto 30 s mentre arrivano messaggi (forzare un rifiuto, p.es. comando durante MOVING) | il pad **non si sposta**; velocità costante a dito fermo; rilascio → `JOYSTICK rilascio` e arresto del passo; `blur`/cambio app → arresto | deadman o rilascio non immediati |
| B5 | STOP durante WALK45, poi **WALK** (senza STAND) | ammesso se `stand_resume_allowed`; completa il passo interrotto, quota WALK 120 mm, WALK STANDARD con il passo scelto; **nessun passaggio a 150 mm**; senza nuovo WALK nulla riparte | `WALK_IK_OR_WORKSPACE`, oscillazioni, contatto non previsto |
| B6 | STOP in HOLD150, poi WALK; STOP in HOLD, WALK STOP e joystick | WALK: HEIGHT 150→120 poi WALK; WALK STOP e joystick rifiutati `STOP_LATCHED`; STAND resta ammesso | idem |
| B7 | Cockpit e ViewCube su telefono e PC | Twin in alto a tutta larghezza; quota 50–150 a passi di 5 mm senza invio al cambio; 🐢30/45/🐇60; click/tap facce del cubo, trascinamento, Reset vista come dev.16 | — |
| B8 | WALK45 3 cicli, WALK STOP; WALK60 | come dev.16 (VERIFIED HARDWARE); overrun confrontati con dev.16 per finestra, nessun peggioramento atteso (calcolo Motion invariato) | come dev.16 |

**Accettazione per la PR #64** (decisione dell'operatore): B1–B6 senza fault né cadute; 0 × 503 su `/status` in IDLE e HOLD; pad immobile sotto messaggi; Twin presente dopo refresh in HOLD; STOP→WALK diretto ammesso o rifiutato con motivo preciso. B8 documenta il residuo timing (non risolto da dev.17). In caso di regressione: rollback APP-only a dev.16.
