# MATDOG — passaggio di consegna dopo il pilot nativo M12 MIN

**Data:** 2026-07-25
**Stato:** foundation nativa mergiata e primo pilot hardware completato
**Lingua operativa:** italiano

## Sorgenti di verità definitive

```text
MATDOG:
  repository: MattRobotics/robot-dog
  clone: ~/MATDOG/github/robot-dog
  branch canonico: main
  commit evidenze pilot: ee3aa9c99c24531921b27bc7fde56414c02f9e7e

integrazione ST3215:
  repository: MattRobotics/norma-core
  clone: ~/norma-core
  branch canonico: main
  PR #2: merged
  merge method: squash
  merge commit: b05419178921d1948ee3a1cd2a3a6b57c20a67d3
  head revision validata: 0afb995b41fa158e1bb3fa8f8e45165786364178
  workflow finale: #15 PASS
```

Il branch `matdog/native-calibrator-foundation` è concluso e deve essere
rimosso localmente e remotamente dopo il riallineamento a `main`.

## Risultato hardware congelato

```text
LF_UPPER / M12 / MIN
first contact:  1443 tick
second contact: 1443 tick
spread:         0 tick
baseline:       median 1, MAD 0
final state:    Done 11/11
final cleanup:  verified global torque OFF
```

Il limite URDF MIN era `1451` tick. Il contatto meccanico ripetibile è stato
misurato a `1443` tick. È un dato di contatto fisico e non va applicato come
limite safe agli altri giunti.

Checkpoint tecnico:

```text
06_Software/Matdog_Core/calibration/
MATDOG_NATIVE_M12_MIN_PILOT_CHECKPOINT_2026-07-25.md
```

Evidenza grezza:

```text
09_Logs/Calibration/M12_native_pilot_2026-07-25/
```

SHA-256 canonici:

```text
log:  e77b70c4fc7418d85122bbc6bd7ce67c37894e9dc616a9063c6e0a9078aaa009
meta: 17145f7d3b119a32e5a78057224d1f0f3a23381d2c9223bd97a5d39662a1ba61
```

## Architettura da conservare

```text
Station
→ driver ST3215 Rust
→ auto_calibrate/matdog.rs
→ scritture RAM allowlisted
→ command-result barrier + fresh readback
→ telemetria InferenceState
→ stall detection position/velocity/persistence
→ backoff + repeatability
→ global torque-OFF
```

Vincoli permanenti:

- Station è l'unico proprietario della seriale;
- `GOAL_POSITION` resta unsigned `0…4095`;
- nessun signed-wrap;
- calibrazione meccanica MATDOG RAM-only;
- nessun reset, Offset, EEPROM, RegWrite/Action o Freeze;
- runtime gate attivo durante l'arming MATDOG;
- set servo esatto:
  `11,12,13,21,22,23,31,32,33,41,42,43`;
- MATDOG non è simmetrico fronte/retro: gli hip anteriori sono 20 mm più alti
  dei posteriori;
- non copiare un solo profilo di zampa sulle quattro catene senza applicare le
  trasformazioni URDF specifiche.

## Stato completato

- repository e branch temporanei storici ripuliti;
- foundation Rust nativa mergiata in `norma-core/main`;
- topology-aware sparse-ID discovery;
- arming esplicito `LF_UPPER_M12_MIN`;
- runtime gate RAM-only;
- comando automatico `ResetCalibration` della UI rifiutato prima della seriale;
- workflow #15: contratto, rustfmt, 63 test e build Station PASS;
- pilot M12 MIN reale;
- doppio contatto ripetibile a `1443` tick;
- ritorno home e torque-OFF globale;
- log, metadati, hash e checkpoint salvati su `robot-dog/main`.

## Prossima attività

Generalizzare il calibratore ai 24 contatti:

```text
LF → RF → RH → LH
per ogni zampa:
  UPPER MIN/MAX
  HIP MIN/MAX con prerequisite UPPER +50°
  LOWER MIN/MAX con prerequisite UPPER +90°, HIP 0°
  ritorno HOME
```

La nuova chat deve:

1. verificare che entrambi i clone locali siano puliti e su `main`;
2. leggere questo handoff e il checkpoint hardware prima di modificare codice;
3. progettare profili data-driven per i 12 giunti;
4. mantenere un solo giunto attivo alla volta;
5. implementare MIN e MAX con direzione, guard e prerequisite specifici;
6. registrare tick coarse/fine, spread, corrente diagnostica e readback;
7. proporre un safe margin separato dal contatto meccanico;
8. salvare limiti misurati e safe nel progetto MATDOG, non negli Offset EEPROM;
9. completare i 24 contatti;
10. rigenerare `HOME q=0 → LOW_STAND → NOMINAL_STAND`;
11. validare FK, IK, collisioni e traiettorie offline;
12. eseguire il nuovo stand fisico supervisionato;
13. soltanto dopo riprendere body-height control e gait engine.

## Primo controllo della prossima chat

```bash
cd ~/MATDOG/github/robot-dog
git status --short --branch
git fetch --prune origin
git pull --ff-only origin main

cd ~/norma-core
git status --short --branch
git fetch --prune matt
git switch main
git pull --ff-only matt main
```

Non riutilizzare gli startup legati agli SHA intermedi e non premere `Save`
nella UI per questo pilot RAM-only.