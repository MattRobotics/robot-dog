# MATDOG — checkpoint nativo LF_LOWER M11 MIN V28R

**Data:** 2026-07-31  
**Esito complessivo:** PASS  
**Bus:** `5B14114953`  
**Profilo:** `LF_LOWER_M11_MIN`  
**Runner:** `V28R`  
**Stato documento:** checkpoint hardware; non autorizza il profilo successivo.

## Contatto fisico congelato

```text
coarse:          3094 tick
fine:            3092 tick
spread:             2 tick
baseline median:    0
baseline MAD:       0
```

Il risultato è una misura di contatto meccanico per questo profilo e questa
configurazione prerequisite. Non è un limite operativo safe e non viene
esteso ad altre configurazioni o ad altri giunti.

## Prerequisite e probing

```text
M42 -> 2389
M13 -> 2048
M12 -> 3072  (LF_UPPER orizzontale)
M11            unico probing joint
```

Il profilo ha mantenuto l'ordine meccanico `UPPER -> LOWER -> HIP`; i profili
HIP isolati restano bloccati.

## Recovery e chiusura

```text
M11: 3092 -> 2048, distance=1044, timeout=31100 ms
M12: 3067 -> 2048, distance=1019, timeout=30475 ms
M13: 2054 -> 2048
M42: 2385 -> 2048
status: Done 14/14
final verified global torque OFF: true
serial free after Station stop: true
```

## Catena software realmente validata localmente

```text
9cd2fa00e15c4393c89d4141765dd60329072732  ordered UPPER -> LOWER -> HIP
7174149d7a047c2c5a631f51af190a6855a967c6  MATDOG Auto Calibrate-only UI
f03cc2fedd4888e1d571e3ff0831ebcdd4b9d059  bounded startup prerequisite home settle
5c52a93c2f889c556fb2f66cfce70f8843354f58  preserve probe-home handoff during restore
```

At publication time `5c52a93...` was a local commit object and was not yet
present on GitHub. This checkpoint records the evidence; it does not claim
that the remote source branch has already been aligned.

```text
Station: NormaCore.Dev station 0.1.0-beta.9 (5c52a93)
Station SHA256: 027aef78d05ac20534fbfcea4554bf36a1ff4a4959b23277c7465b9d51c86fa5
ST3215 tests: 87/87 PASS
Rust warnings: 0
V27R original marker SHA256: d6cb7d950c797837d19a0077308b17198d3c56657f6ae1fa3cc2db2475fd87c9
```

## Evidenza online inclusa

La directory di evidenza contiene:

- `SUMMARY.txt`: riepilogo del run V28R;
- `METADATA.env`: metadati verificati dal terminale;
- `station_excerpt.log`: estratti determinanti del log Station;
- `MARKER_PROVENANCE.env`: identità e SHA-256 del marker originale locale;
- `RUNNER_PROVENANCE.md`: identità e percorso del runner originale locale;
- `SHA256SUMS`: hash delle copie pubblicate in questo checkpoint.

Gli artefatti originali completi restano nell'archivio locale indicato nei
metadati. Gli estratti online sono deliberatamente etichettati come ricostruiti
dal terminale verificato, non come copie byte-identiche dei file locali.

## Contratto permanente

- Station è l'unico proprietario della seriale;
- nessun `pyserial` parallelo;
- `GOAL_POSITION` resta unsigned `0..4095`;
- sole scritture RAM: `TorqueEnable`, `Acc`, `GoalPosition`, `GoalSpeed`, `TorqueLimit`;
- vietati EEPROM, Position Offset, LOCK, Reset/ResetCalibration, RegWrite/Action, Save/Freeze;
- una sola armatura esplicita per avvio Station;
- Reset e Save disabilitati nella UI MATDOG;
- piano tavolo `Z=-0.180 m`, robot sostenuto e zampe libere;
- nessun merge automatico in `main`.

## Gate successivo

`LF_LOWER_M11_MAX` resta bloccato finché:

1. la catena sorgente fino a `5c52a93` non è materializzata su branch draft;
2. CI indipendente non passa su sorgente, viewer e Station;
3. i numeri MAX non sono riconfermati dal sorgente remoto allineato;
4. offline gate e runner separato non sono generati da quella stessa revisione.
