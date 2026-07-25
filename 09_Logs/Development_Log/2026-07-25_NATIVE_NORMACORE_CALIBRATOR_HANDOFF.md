# MATDOG — passaggio di consegna dopo il pilot nativo M12 MIN

**Data:** 2026-07-25
**Stato:** primo pilot hardware nativo completato con successo
**Lingua operativa:** italiano

## Sorgenti di verità

```text
MATDOG:
  repository: MattRobotics/robot-dog
  clone: ~/MATDOG/github/robot-dog
  branch canonico: main

integrazione ST3215:
  repository: MattRobotics/norma-core
  clone: ~/norma-core
  PR: #2
  branch di lavoro: matdog/native-calibrator-foundation
  head dopo pulizia finale: 0afb995b41fa158e1bb3fa8f8e45165786364178
```

## Risultato congelato

```text
LF_UPPER / M12 / MIN
first contact:  1443 tick
second contact: 1443 tick
spread:         0 tick
baseline:       median 1, MAD 0
final state:    Done 11/11
final cleanup:  verified global torque OFF
```

Checkpoint tecnico:

```text
06_Software/Matdog_Core/calibration/MATDOG_NATIVE_M12_MIN_PILOT_CHECKPOINT_2026-07-25.md
```

Evidenza grezza:

```text
09_Logs/Calibration/M12_native_pilot_2026-07-25/
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
- MATDOG non è simmetrico fronte/retro:
  gli hip anteriori sono 20 mm più alti dei posteriori;
- non copiare un singolo profilo di zampa su tutte le catene senza
  trasformazioni URDF specifiche.

## Attività completate

- repository e branch temporanei ripuliti;
- foundation Rust nativa;
- topology-aware sparse-ID discovery;
- arming esplicito `LF_UPPER_M12_MIN`;
- runtime gate RAM-only;
- test offline e build Station;
- pilot M12 MIN reale;
- doppio contatto ripetibile a 1443 tick;
- ritorno home e torque-OFF globale;
- evidenze e checkpoint salvati.

## Prossima attività necessaria

Generalizzare il calibratore da un solo contatto ai 24 contatti:

```text
LF → RF → RH → LH
per ogni zampa:
  UPPER MIN/MAX
  HIP MIN/MAX con prerequisite UPPER +50°
  LOWER MIN/MAX con prerequisite UPPER +90°, HIP 0°
  ritorno HOME
```

La nuova chat deve:

1. verificare lo stato remoto reale dei due repository;
2. verificare la chiusura/merge della PR #2;
3. progettare configurazioni data-driven per i 12 giunti;
4. mantenere un solo giunto attivo alla volta;
5. implementare MIN e MAX con guard specifici;
6. registrare tick coarse/fine, spread, corrente, readback e safe margin;
7. salvare limiti misurati e safe nel progetto MATDOG, non negli Offset EEPROM;
8. completare i 24 contatti;
9. rigenerare `HOME q=0 → LOW_STAND → NOMINAL_STAND`;
10. validare FK, IK, collisioni e traiettorie offline;
11. eseguire stand fisico supervisionato;
12. soltanto dopo riprendere body-height control e gait engine.

## Primo controllo della prossima chat

```bash
cd ~/MATDOG/github/robot-dog
git status --short --branch
git fetch --prune origin

cd ~/norma-core
git status --short --branch
git fetch --prune matt
```

Non riutilizzare gli startup legati agli SHA intermedi e non premere `Save`
nella UI per questo pilot RAM-only.
