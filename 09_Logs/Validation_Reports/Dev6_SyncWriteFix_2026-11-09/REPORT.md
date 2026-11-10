# MATDOG 1.0.2-dev.6 — Fix della traccia 44 (SYNC_WRITE_FAILED) e nuova build

**Data:** 09/11/2026. **Branch:** `feat/gait-redesign-20261108`, pubblicato su `feat/motion-perf-next-20261107`. **PR:** #67, OPEN e DRAFT.

**Installato sul robot:** `1.0.2-dev.6` `70abcff54cd3`, APP `67d55e1a…`, conservata. **Rollback:** `1.0.2-dev.4` `4358f13`.

**RCA d'origine:** `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_INCIDENT_SYNCWRITE_20261109T175118Z/`.

**Operazioni non eseguite:** flash, accesso all'hardware, scritture NVS/EEPROM, calibrazioni, modifiche dei limiti, merge.

**Classi di evidenza:**
- **VH** = verificato su hardware;
- **VO** = verificato offline;
- **INH** = implementato, non validato su hardware.

## Verdetto

**READY FOR SUPERVISED FLASH** della build `dd76fad6358e`. È una correzione software verificata offline e il percorso STAND/SLOW/STOP non ha regressioni.

STANDARD e FAST restano **INH**. Che il gate ora le ammetta non qualifica fisicamente ACC 180/220 né le velocità 1800/1950 (§7).

## 1. Causa verificata (VH, traccia 44)

- **Sul robot:** `WALK_STANDARD` va in fault alla prima release (cycles 0,000) con `SYNC_WRITE_FAILED`. Il drive commandato era ancora {480,13} e nessuna release era stata persa.
- **Percorso:** `SimpleMotion::send()` → `Controller::SimpleServoPort::frame()` → `ServoBus::setOperationalDrive()`.
- **Causa:** il gate ammetteva solo {480,13} e {1000,50}, quindi rifiutava {1800,180}. `frame()` restituiva false **prima** di `syncWriteGoalPositions`: nessun byte trasmesso e nessuna preemption.

È un difetto d'integrazione deterministico: la tabella delle modalità WALK e il gate del ServoBus non erano allineati, e nessun test li confrontava.

## 2. Diff effettivo (`4ed0d6a`, solo `src/servo/ServoBus.h`)

```diff
-    if(!((speed==480&&acc==13)||(speed==1000&&acc==50)))return false;  // STAND rc.10, WALK
+    if(!((speed==480&&acc==13)||(speed==1000&&acc==50)||(speed==1800&&acc==180)||(speed==1950&&acc==220)))return false;
```

Più quattro righe di commento ([`fix_servobus.diff`](evidence/fix_servobus.diff)).

**Invariati:** STOP, SAFE_OFF, torque enable, conversione RAW, cinematica, prove geometriche, limiti, NVS e calibrazioni.

**Coppie ammesse:** solo le quattro coppie esatte. Ogni altra coppia è rifiutata prima della trasmissione.

**Due elenchi, nessuna dipendenza nuova.** Il ServoBus (livello servo) non include SimpleMotion e la tabella `kWalkModes` resta l'unica fonte delle leggi. La coerenza è garantita da due test (§3):
- il ServoBus reale deve ammettere ogni drive della tabella;
- ogni frame della suite SimpleMotion deve usare un drive della tabella.

## 3. Regressioni sul ServoBus reale

[`test_preenable_bus.cpp`](../../../05_Firmware/MATDOG_Controller/scripts/tests/test_preenable_bus.cpp), funzione `walkModeDrives()`.

**Configurazione del banco:**
- SimpleMotion di produzione;
- **`Controller::SimpleServoPort::frame` di produzione**, estratto testualmente dal runner come fa `test_stop_adapter.py`;
- ServoBus reale con UART simulata a registri;
- predicato di preemption di produzione;
- inseguimento ideale dei registri, quindi nessuna fisica del servo.

| Verifica (punto del mandato) | `70abcff` (installata) | `dd76fad` |
|---|---|---|
| Gate ammette STAND {480,13} / SLOW {1000,50} (2) | PASS / PASS | PASS / PASS |
| Gate ammette STANDARD {1800,180} / FAST {1950,220} (3) | **FAIL / FAIL** | PASS / PASS |
| SYNC_WRITE decodificato: 104 B, 12 id, ACC, GoalSpeed, 12 target, checksum (4) | FAIL per STANDARD e FAST | PASS per tutte e 4 |
| 11 coppie non previste rifiutate, 0 byte su UART (5) | PASS | PASS |
| STAND 150: ogni frame {480,13} (2) | PASS | PASS |
| WALK SLOW 33 s, 6,50 cicli, ogni frame {1000,50}; WALK STOP → STANDING | PASS | PASS |
| Selezione cambiata due volte durante SLOW, applicata al WALK successivo | PASS | PASS |
| **WALK STANDARD dopo WALK STOP** (sequenza della traccia 44) | **FAULT SYNC_WRITE_FAILED a 0,00 cicli, 1 frame rifiutato** | WALKING 6,50 cicli, ogni frame {1800,180} |
| STOP durante STANDARD: frame d'arresto {1800,180}, STOP verificato, coppia ON (6) | non eseguibile | PASS |
| Ripresa dopo STOP in STANDARD; WALK STOP → STANDING | non eseguibile | PASS |
| WALK FAST 7,80 cicli, ogni frame {1950,220}; WALK STOP | non eseguibile | PASS |
| Ritorno a SLOW: HEIGHT {480,13}, WALK {1000,50} | non eseguibile | PASS |
| SAFE_OFF da WALK: 13/13 coppia OFF verificata (6) | PASS | PASS |
| Totale | **7 FAIL** | **0 FAIL** |

Evidenze: [`real_servobus_harness_on_70abcff.txt`](evidence/real_servobus_harness_on_70abcff.txt) (prova negativa, stesso test sul sorgente installato) e [`real_servobus_harness_after_fix.txt`](evidence/real_servobus_harness_after_fix.txt).

Sul sorgente `70abcff` il banco riproduce la traccia 44 punto per punto: SLOW 6,50 cicli, WALK STOP, STANDARD in fault a 0,00 cicli.

**Lacune del banco corrette.** Nessuna indebolisce un controllo; erano tutte presenti anche su HEAD.
- **`decodeRaw` gen-1 mancante:** è il RAW_CONVERSION «artificiale». La classe base restituiva false, quindi ogni correzione IMU in WALK veniva rifiutata. Ora la decodifica è la stessa di `Controller::SimpleServoPort::decodeRaw`.
- **Hook di preemption del bus:** era `stopping()`, che preemptava anche i frame autorizzati dello STOP, quindi nessun arresto con coppia attiva era mai stato provato. Ora è il predicato di produzione `busPreemptRequested()`.
- **Caso `PERSISTENT`:** si aspettava un nuovo STAND dopo fault e STOP. Dal 02/11 (`c2d95a8`) il fault resta latched (`FAULT_STOP_REQUIRED`). Falliva anche su HEAD e non era nel gate host.

**Test del banco preesistenti, invariati e PASS:** DECISIVE, PRIME (OFF perso, TX incerta, STOP durante il frame), recuperi bus, servo assente, PROTECTION ×6 con SAFE_OFF 13/13, STOP in tre fasi senza riavvio automatico.

## 4. Altre verifiche (VO)

| Verifica | Esito |
|---|---|
| Suite SimpleMotion (ASan/UBSan) con il nuovo vincolo «ogni frame usa un drive della tabella»: STAND, WALK SLOW/STANDARD/FAST, 8 STOP con ripresa per modalità, ripartenza dopo WALK STOP, `WALK_IMU_SETTLING`, joystick, PRONE, H1-RCA | PASS ([`simple_motion_summary.txt`](evidence/simple_motion_summary.txt)) |
| Suite night-recovery: DALY, SimpleMotion, deadline del ServoBus, banco preenable, night recovery, pacchetti servo, IMU | PASS ([`night_recovery_suite_summary.txt`](evidence/night_recovery_suite_summary.txt)) |
| Adapter STOP: `SimpleServoPort::frame` di produzione con doppi; ammissione STOP, preemption, barriera HTTP | PASS ([`stop_adapter.txt`](evidence/stop_adapter.txt)) |
| Test host, contratto HTTP (97 controlli) | PASS |
| Dashboard: fixture 84, DOM 92 comandi, contratto 6 112, browser con il selettore a 7 larghezze | PASS |
| Audit statico (370 file) | PASS |
| Compilazione ESP32-S3 CLEAN `ROBOT_POWERED` | PASS (§8) |

**Altri filtri sul percorso operativo, controllati:**
- `packet::buildGoalFrame` accetta qualsiasi speed fino a 0x7fff e qualsiasi ACC.
- `syncWriteGoalPositions` richiede solo una coppia validata e il bus libero.
- `stopFrameAuthorized` confronta il drive con il drive di arresto (quello commandato).
- Le riletture di WALK e STOP confrontano goal speed e ACC per uguaglianza. La decodifica è un `le16` semplice, quindi 1800 e 1950 vengono letti correttamente: nessuna interpretazione del bit 10 come segno.
- `setValidationDrive` non è nel percorso operativo.
- `kCalibrationDrive` {160,8} non viene mai inviato in un frame.

**Nessun altro filtro rifiuta i profili scelti.**

## 5. Dashboard

Il commit `5938fd9` (selettore WALK leggibile nella colonna stretta) è incluso nella build. Nessun'altra modifica alla UI.

## 6. Motion budget

Non c'entra con il fault: il rifiuto è un controllo di valori. Resta il dato hardware della traccia 44 da affrontare a parte: picco di 23,7 ms in HEIGHT, compensazione IMU 17,5 ms, 73 overrun, 0 release perse.

## 7. Rischio residuo

**Ancora non misurato sul robot:**
- ACC 180/220 e velocità 1800/1950. Il gate ora le lascia passare ma nessun servo le ha mai eseguite: al banco QC, ACC sopra 50 non è mai stato provato;
- inseguimento, corrente, temperatura e frenata sotto carico (la prova di arresto assume 8 400 step/s²);
- il comportamento del firmware ST3215 con questi valori di registro (memorizzazione e rilettura).

**Conseguenza se la rilettura differisse:** una differenza tra valore scritto e riletto produrrebbe `SERVO_TORQUE_OR_GOAL_LOST`. È un esito fail-closed, ma va osservato sulla fixture.

**Controlli fisici necessari:**
- FAST ha alzata 6 mm: la luce reale del piede va verificata;
- il modello del servo non accoppia corpo, contatto e IMU.

## 8. Identità della candidata e flash preparato (non eseguito)

| Campo | Valore |
|---|---|
| Versione | `1.0.2-dev.6` (stessa stringa, nuova build) |
| Commit | `dd76fad6358e3b753807c3d8cf453a5adc819f87` |
| Build ID | `dd76fad6358e`, CLEAN `ROBOT_POWERED`, OTA ingest 0 |
| APP | 1 738 848 B, SHA256 `5fac0a11c367a4590c33d90c6228d26f8c24e74b0d56827658f738d60613778d` |
| Manifest | [`build_manifest_dd76fad.txt`](evidence/build_manifest_dd76fad.txt) |
| Script APP-only | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6_T44FIX_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.6-dd76fad.sh` (SHA256 `3de79167…`) |
| Autorizzazione | `MATDOG_DEV6_T44FIX_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`. Senza variabile rifiuta, anche con le autorizzazioni precedenti impostate (verificato) |
| Scrittura | una sola, app0 0x010000; NVS, otadata, partizioni ed EEPROM non toccati |
| Rollback | `rollback-assets.sha256` 9/9 OK: dev.4 `4358f13`, dev.6 installata `70abcff`, build storiche, backup completo |

## 9. Qualifica hardware minima dopo il flash

Da eseguire solo su tua autorizzazione, con il robot sulla fixture.

1. **H0:** boot, identità `dd76fad6358e`, IDLE ≥ 60 s.
2. **SLOW di regressione:** WALK SLOW e WALK STOP. Atteso identico alla traccia 44: 6,5 cicli, drive {1000,50}.
3. **STANDARD sulla fixture, zampe libere:**
   - verificare con `@MOTION STATUS` che la rilettura `walk_mode.servo_read` dia 1800/1800/180/180 e che non scatti `SERVO_TORQUE_OR_GOAL_LOST`;
   - poi STOP in più fasi;
   - registrare inseguimento, corrente e temperatura.
4. **FAST sulla fixture,** come al punto 3, con attenzione alla luce del piede.
5. **A terra,** solo dopo i punti 1–4.
