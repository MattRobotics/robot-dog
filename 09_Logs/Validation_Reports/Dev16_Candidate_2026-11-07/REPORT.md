# MATDOG Controller 1.0.1-dev.16 — candidata di stabilizzazione (07/11/2026)

**Stato: IMPLEMENTED NOT VALIDATED HARDWARE.** Candidata compilata e qualificata offline. **Non flashata**: sul robot resta installata la dev.15 (`7997ad4d4497`, APP `b81d3a4f…`). Non è una release stable. Procedura di flash e collaudo: [FLASH_AND_TEST.md](FLASH_AND_TEST.md).

## 1. Identità

| Elemento | Valore |
|---|---|
| Base | sorgente dev.15 installato `7997ad4` (ref locale `refs/preserve/20261107/dev15-installed-7997ad4`) |
| Commit | `115032e` (PRONE) → **`e0ddbb1791d0726d757fb17e10c4ba2d67daf103`** (T0/T1, identità) su `matdog/dev16-stabilization` |
| Versione / build | `1.0.1-dev.16` / `e0ddbb1791d0` (stringhe verificate nel binario) |
| APP | `MATDOG_Controller.ino.bin`, **1.694.048 byte**, SHA256 **`73952eec1ddb29976941b7afd16149b15f790a2e8c46cb2067d1f72a8424138d`** |
| Manifest | `SOURCE_STATE=CLEAN`, `GIT_DIRTY=0`, `HARDWARE_PROFILE=ROBOT_POWERED`, `OTA_INGEST_ENABLED=0`, layout `MATDOG_16M_2x5M_NVS_V1`, slot APP 32,3 % |
| Partizioni / bootloader / sdkconfig | tabella `8f756ecb…` **identica** a dev.15; bootloader e `sdkconfig` identici a dev.15 (non vengono scritti) |
| Binario | solo locale ASUS: `~/MATDOG/build-output/dev16-stabilization-20261107/build-e0ddbb1/artifacts/ROBOT_POWERED-ota0/` (con ELF, map, manifest) |
| Configurazione privata | nessun header `*.local.h` (come dev.15); `PRIVATE_TLS_CONFIGURATION_SHA256=ABSENT` |

## 2. Modifiche funzionali (diff minimo)

| Area | Modifica | File |
|---|---|---|
| **PRONE — causa** | Consegna `PRONE_HEIGHT → PRONE_LIFT` a **52,0 mm** (`kProneHandoverM`) invece di 53 mm | `SimpleMotion.h`, `SimpleMotion.cpp` (3 occorrenze del letterale) |
| **PRONE — nessuna caduta** | Deadline di un segmento PRONE scaduta, oppure posa sotto carico fuori dalla box del tratto ma `pointSafe` in `kHeight`/`kLift` → **STOP contestuale esistente** (`beginStop`) con coppia, motivi `PRONE_NOT_COMPLETED_SUPPORT_HELD` / `PRONE_OUTSIDE_SEGMENT_SUPPORT_HELD`. Ogni guardia di quello STOP ricade su fault + SAFE_OFF | `SimpleMotion.cpp` (`proneNotCompleted`), testi Dashboard |
| **T0** | Chiavi `timing.max_work`/`first_miss` corrette; `command_timing.overruns`, `overrun_releases`, `max_work_meta`/`first_miss_meta` = [ritardo, tempo dalla fine della release precedente, lavoro esterno] µs; riga Dashboard | `MotionTiming.h`, `MotionService.*`, `MotionDashboard.h` |
| **T1** | A correzione IMU assestata, release WALK/HOLD alterne riusano l'ultima correzione accettata come offset articolare sul nuovo nominale; controlli di sicurezza a ogni release; solve completo forzato da slew, fase, gait, IMU, unwind o recupero STOP, con passo ≤ 1 periodo dopo una release leggera | `SimpleMotion.*` |
| UI / identità | Badge joystick "VERIFICATO SU HARDWARE (dev.15)"; `1.0.1-dev.16`; unità bridge preparata attende dev.16 | `MotionDashboard.h`, `BuildConfig.h`, `host_bridge/` |

**Invariati:** mapping servo, URDF REV01-FIX, FK/IK, calibrazione gen-1, box di collisione, limiti meccanici, STOP/SAFE_OFF, deadman, guardie elettriche e termiche, gait (periodo, stride, sway), Digital Twin. Nessuna sorgente della libreria `motion/` modificata (il pin `motion_final_release_sources.json` resta valido).

## 3. PRONE — RCA e prova

**Predicato bloccante (dati hardware dev.15, target ricostruito con la mappa dei segni dell'encoder):** a fine `PRONE_HEIGHT` sia `arrived()` (`SimpleMotion.cpp:589-597` in dev.15) sia `settledUnderLoad()` (`:626-660`) richiedono la posa **misurata** dentro `kLift`. A 53 mm il target nominale dei RH/LH_UPPER sta solo **2,2 tick** dentro `kLift` (rig host = ricostruzione hardware: 2,2 / 4,5 / 13,2 tick per RH_U / LF_L / LF_U).

| Caso | Residuo critico | Posa misurata rispetto a `kLift` | Esito dev.15 |
|---|---|---|---|
| trace(15) da ~120 mm | LH_UPPER 3 tick | **−0,32 tick** | timeout 18,88 s, SAFE_OFF a 52,2 mm |
| trace(18) da 150 mm | LF_UPPER 14 tick (sotto carico) | **−0,79 tick** | timeout 20,96 s, SAFE_OFF a 52,4 mm |
| trace(22) da 100 mm | LH_UPPER 2 tick | +0,68 tick | `PRONE_REST` |
| trace(22) da 50 mm | (consegna diretta in `kLift`) | — | `PRONE_REST` |

Residui, velocità, coppia, freschezza e carico erano tutti entro i limiti. A bloccare era solo la box di consegna, con una differenza di **meno di un tick** tra successo e fallimento.

**Scelta di 52,0 mm:** è il massimo del margine bilaterale contro il peggior cedimento registrato per ogni giunto, su 96 casi (69 + 23 residui HOLD dev.14, 4 consegne PRONE dev.15). Risultato: ≥ 4,7 tick in `kHeight` e ≥ 3,5 tick in `kLift`. A 50,5 mm il LF_LOWER esce da `kHeight` (−6,1); a 53 mm i giunti escono da `kLift` (−0,8).

**Replay offline (VERIFIED OFFLINE):**
- `test_prone_dev15_replay` sul sorgente **dev.15 non modificato**: FAULT `NATIVE_TARGET_NOT_REACHED` dopo 18,50 s e 21,00 s (robot: 18,88 e 20,96 s). Anche il caso STAND100 riuscito sul robot fallisce nel replay: il margine è sotto il tick.
- Stesso test su dev.16: **4/4 `PRONE_REST`**, margine minimo di consegna in `kLift` 3,2–16,1 tick.
- Consegna forzatamente bloccata (40 tick di cedimento RH_U): `PRONE_NOT_COMPLETED_SUPPORT_HELD`, coppia ON, nessun OFF. Ripresa STAND ammessa e completata, SAFE_OFF su richiesta verificato 13/13, perdita di alimentazione durante il sostegno → FAULT + SAFE_OFF.
- `test_prone_hardware_replay`, 69 residui hardware dev.14: **69/69** a terminale, tutti i margini meccanici e di box > 4 tick. Sul sorgente dev.15 falliva identico per un errore di mappatura fase→box del test, corretto.

## 4. Timing — T0 e T1

**T0:** il JSON ora distingue release oltre 20 ms (`overruns`), periodi persi (`missed`) e tempo fuori release. Il contratto di stato occupa 4 489 B su un buffer di 5 248 B; lo snapshot resta ≤ 1 536 B (vincolo statico).

**T1 (rig host, contatori di lavoro identici al robot):**

| Scenario | Lavoro libm medio per release dev.15 → dev.16 | Tempo host medio | Release leggere | Scostamento dal target IK esatto |
|---|---|---|---|---|
| HOLD100 inclinato | trig 106 → 56 (−47 %) | 7,9 → 5,6 µs (−29 %) | 47 % | 0 tick |
| WALK45 inclinato | 242 → 188 (−22 %) | 23,3 → 18,1 µs (−22 %) | 45 % | ≤ 1,0 tick |
| WALK60 inclinato | 242 → 185 (−23 %) | 22,2 → 16,9 µs (−24 %) | 49 % | ≤ 0,2 tick |
| JOYSTICK inclinato | 334 → 267 (−20 %) | 25,3 → 19,8 µs (−22 %) | 43 % | ≤ 1,0 tick |

- Prova WALK su tavola (35 casi): nessun fault; errore d'assetto massimo 2,17 mrad (dev.15: 1,97), roll 2,08 (1,67); differenza mediana 0,06 mrad. HOLD su tavola e contratto dello slew invariati.
- Durante lo sviluppo due varianti di T1 sono state **scartate**:
  - passo doppio dopo una release leggera: fault `WALK_IK_OR_WORKSPACE` a 3° di tavola, blocco in HOLD a 150 mm;
  - estrapolazione dell'offset: scostamento ~1,9 tick, non nullo in HOLD.

**Previsione hardware (HYPOTHESIS, da misurare con T0):** coppia WALK45 completa + leggera ≈ 24 + 14 ms ≈ 38 ms < 40 → periodi persi verso 0, con margine scarso. In HOLD le release leggere da ~4–5 ms dovrebbero aprire la finestra di slack di 5 ms, con 503 per STAND/WALK da HOLD attesi a zero. **Joystick:** coppia ≈ 37 + 21 ms > 40 → resterà oltre budget, con meno periodi persi di dev.15.

**Ottimizzazione successiva identificata (non inclusa in dev.16):** compilazione `-O2` delle sole unità Motion calde (`SimpleMotion.cpp`, `motion/*.cpp`) invece di `-Os` globale. È numericamente neutra: stesse operazioni IEEE emulate in ROM, nessuna contrazione FMA in double. Il guadagno sull'ESP32-S3 **non è stimabile offline**, quindi va misurato con T0 in un A/B hardware. Rischio da controllare: crescita dei frame di stack per inlining. Basterebbe un guadagno ≥ 15 % per riportare la coppia del joystick sotto i 40 ms. `float` nei cicli IK e pianificazione a 25 Hz con interpolazione restano passi successivi, non giustificati prima di quella misura.

## 5. Regressioni eseguite (offline)

| Suite | Esito |
|---|---|
| `run_simple_motion_tests.sh` (sequenze, PRONE, STOP, WALK/joystick, tavola HOLD/WALK, slew) | PASS |
| `run_stop_support_tests.sh` | PASS |
| `run_prone_tests.sh` (hardware replay 69 + dev.15 replay) | PASS |
| `run_simple_dashboard_tests.sh` (contratto 240 controlli, DOM, Twin feed, budget portal) | PASS |
| `test_motion_http_transport.py`, `test_http_fence_contention.py` (723 controlli), `test_slack_exempt_session.py` | PASS |
| `test_static_audit_*` (network, safe actuator, startup), `test_dev_identity_build.py` | PASS |
| `static_audit.py` completo (363 sorgenti, convergenza Motion) | PASS |
| Frame di stack ELF dev.15 → dev.16 | `compensateLevel` 464→592 B, `SimpleMotion::update` 1296→1312, handler HTTPD +16 B; massimo invariato 9 648 B |

**Non eseguiti / dichiarati:** `test_prone_descent`, `test_prone_admission`, `test_prone_height_residuals`, `test_prone_temperature` precedono la dev.15 e falliscono identici sul sorgente installato `7997ad4`. Non sono stati riscritti.

**Calibrazione/NVS:** nessun file di calibrazione, persistenza, NVS o EEPROM modificato (diff limitato a `simple/`, `core/MotionService*`, `network/MotionDashboard.h`, `config/BuildConfig.h`). Nessuna scrittura hardware eseguita.

## 6. Rischi residui

1. **Margine PRONE 3,5–4,7 tick** nel caso peggiore combinato: non dimostrato contro cedimenti maggiori non ancora registrati. Il ripiego è il sostegno con coppia, non lo SAFE_OFF.
2. **Sostegno automatico**: dopo `PRONE_NOT_COMPLETED_SUPPORT_HELD` il robot resta in coppia finché l'operatore non sceglie STAND o SAFE_OFF; restano attive le guardie termiche, elettriche, di tracking e di box. Il supporto fisico non è misurato.
3. **Joystick** ancora oltre 20 ms per release (previsione).
4. **T1** è un'approssimazione (≤ 1 tick sul target): errore d'assetto WALK modellato fino a +0,64 mrad caso per caso.
5. **Stack del loop**: +144 B sul percorso WALK peggiore; minimo libero osservato in dev.15 2 240 B. Va ricontrollato `loop_stack_min_free` in collaudo.
6. Tutti i risultati sono host/offline: **nessuno è VERIFIED HARDWARE**.
