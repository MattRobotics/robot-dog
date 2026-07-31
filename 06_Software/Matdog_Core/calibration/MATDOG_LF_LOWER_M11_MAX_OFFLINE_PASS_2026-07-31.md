# MATDOG — LF_LOWER_M11_MAX offline PASS

**Data:** 2026-07-31  
**Stato:** PASS offline; hardware non avviato  
**Profilo successivo:** `LF_LOWER_M11_MAX`

## Sorgente e pacchetto verificati

```text
NormaCore head:
fff8e8989ca945bb56982ab5f626e3b45ba8b2dd

robot-dog preparation head usato localmente:
6946079423fa593135d1cec6e145656e4a5eab87

worktree:
~/MATDOG/worktrees/norma-core-lf-lower-m11-max-v30
```

Il worktree è stato riutilizzato in detached HEAD sul commit NormaCore previsto.

## Gate completati

```text
LF_LOWER profile-number test: PASS
isolated HIP hardware block test: PASS
cargo test --package st3215: 87 passed, 0 failed
ST3215 doc-tests: PASS
station-viewer npm build: PASS
Station release build: PASS
```

Il controllo npm ha segnalato vulnerabilità nelle dipendenze installate, ma la build è terminata correttamente. Non è stato eseguito `npm audit fix`, perché una modifica automatica del lockfile non appartiene a questo checkpoint hardware e richiederebbe una revisione separata.

## Parametri hardware riconfermati

```text
profile                  = LF_LOWER_M11_MAX
probe_motor_id           = 11
probe_sign               = -1
home_tick                = 2048
baseline_target_tick     = 1984
urdf_limit_tick          = 1621
guard_tick               = 1557
contact_corridor         = 1557..1685
prerequisite_m42_tick    = 2389
prerequisite_m13_tick    = 2048
prerequisite_m12_tick    = 3072
goal_position_unsigned   = true
ram_only                 = true
hip_profiles_blocked     = true
```

## Artefatti locali

```text
offline marker:
~/MATDOG/_archive/verification-artifacts/
MATDOG_LF_LOWER_M11_MAX_OFFLINE_V29_20260731T182641Z/
OFFLINE_LF_LOWER_M11_MAX_V29_PASS.env

runner preparation:
~/MATDOG/_archive/verification-artifacts/
MATDOG_LF_LOWER_M11_MAX_RUNNER_PREP_V29_20260731T183321Z

executor summary:
~/MATDOG/_archive/verification-artifacts/
MATDOG_LF_LOWER_M11_MAX_OFFLINE_EXECUTOR_V31_20260731T182640Z/SUMMARY.env

bootstrap artifact:
~/MATDOG/_archive/verification-artifacts/
MATDOG_LF_LOWER_M11_MAX_OFFLINE_BOOTSTRAP_V31_20260731T182640Z
```

## Provenienza finale

```text
result=PASS
offline_tests=PASS
viewer_build=PASS
station_release_build=PASS
runner_state=PREPARED_NOT_EXECUTED
hardware_started=false
serial_opened=false
```

## Gate prima dell'hardware

La successiva fase può soltanto:

1. verificare in read-only marker, checksum, binario e configurazione;
2. verificare che Station sia spenta, che le porte siano libere e che la seriale non abbia proprietari;
3. richiedere conferme fisiche esplicite dell'operatore;
4. avviare Station con una sola armatura `LF_LOWER_M11_MAX`;
5. lasciare l'avvio della calibrazione esclusivamente al pulsante `Auto Calibrate` della UI MATDOG;
6. verificare `Done 14/14`, due contatti, global torque OFF e seriale libera dopo lo stop.

Restano vietati EEPROM, Position Offset, LOCK, Reset, ResetCalibration, RegWrite, Action, Save e Freeze. Nessun merge in `main` è richiesto o autorizzato da questo checkpoint.
