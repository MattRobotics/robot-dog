# MATDOG — handoff 18/11/2026 (chiusura sessione)

## 1. Hardware e immagine installata

**Installato: `30c894a`**
- APP `03c863cc433622f672c11cae45f46694c4aa6f78ba0136cc1b7bc818a2e4b340`, 1 779 312 B.
- Ripristinato da Fable alle 22:38 (ora italiana) con lo script qualificato `812a2e83…`: `Verification successful (digest matched)`. È stata scritta solo app0.

**Avvio dopo il rollback:**
- `1.0.2-dev.7` `30c894a0d430`;
- NVS gen 1 `VALID_ACKNOWLEDGED`;
- calibrazione `be8d5902bb4e37da` 12/12, coppie OFF;
- census 13/13;
- READY/IDLE, SAFE_OFF 8191;
- DALY 118/118 validi.

Fonte: `verification-artifacts/MATDOG_BMS_DALY_AB_FABLE_20261118/REPORT_AB.md`.

**Il candidato Motion `05e1214` non è installato.**

## 2. A/B BMS (DALY)

**Esito: INCONCLUSIVE.** Il criterio era 3 avvii a freddo; ne sono stati fatti 2, per decisione dell'operatore.

| Firmware | Avvii a freddo | Silenzi DALY |
|---|---|---|
| dev.5 `df43feb` | 2 | 0 |
| `30c894a` | 1 | 1 (~92 s) |

**Il BMS non è risolto.** L'indicazione a favore di una regressione firmware non è dimostrata.

Passi proposti da Fable:
1. Registrare dall'accensione il prossimo avvio a freddo di `30c894a`, in sola lettura.
2. Se mostra il silenzio: bisect dev.5…dev.14.
3. Se è pulito: ripetere A/B alternati.

## 3. Branch, commit, build

| Branch (GitHub) | SHA | Stato |
|---|---|---|
| `integ/matdog-unified-operational-20261114` | locale `30c894a0d4303d497d750cccb5fd12f347c61d7d`; remoto `06b6db0` (non spostato) | Baseline installata |
| `cand/p1-edge-lift-halftick-20261118` | `eb1ba28d9ad32a794ec6afc1e2adff80c01c02fe` | P1 minima |
| `cand/motion-fp32-20261118` | `05e1214b4010f65794d8e038366d5a3e36251bb8` | P1 + FP32: build CLEAN APP `75ac737d7ee65bcb9bc1c2cb43722b742b64fb8a4af78492a54eb2b2057257c5` (1 777 952 B), **non flashata** |
| `cand/ksteer-slow-recert-20261118` | `1314bc411c589c4216e95629f4665d9d01f05781` | Sorgente integrato per la prossima build: P1 + FP32 + `kSteer`. **Non compilato: la APP `05e1214` NON contiene `kSteer`** |
| `docs/motion-fp32-session-close-20261118` | (questo commit) | Rapporti, manifest, hash degli archivi, handoff |

- **Pacchetto APP-only di `05e1214`:** `verification-artifacts/MATDOG_1_0_2_MOTION_FP32_20261118/preflash/` (gate offline PASS).
- **Rollback a `30c894a`:** script `812a2e83…`, backup `backups/esp32/app_images/30c894a`.
- **Archivi locali e hash:** `09_Logs/Validation_Reports/Motion_FP32_2026-11-18/LOCAL_ARCHIVES.sha256.txt`.

## 4. Ottimizzazioni

| Categoria | Voci |
|---|---|
| Già integrate (`30c894a`) | ACQUIRE v2, T1/T2, PlanSplit V2, fix STOP→STAND, writer HTTP, validDefinition, IRAM G2s, F3 |
| Candidate | P1 `eb1ba28` e FP32 `05e1214`, compilate; `kSteer` SLOW `1314bc4`, solo sorgente |
| Escluse | P1 completa (`TiltedContactIk`), W1, varianti della regola del 2 %, cambio della tolleranza di Newton |
| Non incorporate | W2, B, tuning HTTP |

## 5. Prove superate

**Hardware (18/11, su `30c894a`):**
- identità, NVS e calibrazione;
- catena 140/140;
- STOP USB 14/14;
- correzione ACQUIRE.

**Offline, `05e1214`:**
- `static_audit` PASS;
- FP32 contro double: 60 frame su 38 153 diversi di 1 tick, nessuno di più; livellamento IMU, motivi e contatti identici;
- secanti STOP identiche; perturbazioni 0 fault;
- boot in QEMU OK;
- QEMU, classi P0: −74 / −75 / −75 %, nessuna release sopra 280 k.

**Offline, `1314bc4`:**
- certificato FCL 120/120 + 12/12, limite inferiore 3,085 mm;
- inviluppo joystick SLOW/STANDARD/FAST PASS;
- `static_audit` PASS.

## 6. Ancora aperto

- **P0 Motion timing: OPEN.** Il CCOUNT di QEMU non è tempo hardware. Su `30c894a`: 20,8 ms e 2 overrun.
- **STOP HTTP e dashboard D1/D2:** NOT TESTED.
- **Silenzio DALY:** aperto (A/B inconclusivo).
- **SLOW sterzato:** non qualificabile senza `1314bc4` compilato e collaudato (`random_3` preesistente).
- **Restano in double e pesanti:** le release che costruiscono un piano di segmento (~250–270 k in QEMU) e `compensationSafe` (~60 k).

## 7. Primo passo di domani

1. Valutare con l'operatore l'esito DALY e decidere la registrazione B al prossimo avvio a freddo.
2. Se `1314bc4` è approvato:
   - build CLEAN di `1314bc4`, con `static_audit`, manifest, boot in QEMU e pacchetto APP-only;
   - poi **una sola** qualifica hardware Motion mirata (piano in `Motion_FP32_2026-11-18/REPORT.md` §7), con rollback a `30c894a`.
