# MATDOG — checkpoint nativo LF_UPPER M12 MIN + MAX

**Data:** 2026-07-28
**Esito complessivo:** PASS
**Bus:** `5B14114953`
**Giunto:** `LF_UPPER / M12`
**Contratto:** restart-safe, distance-aware, RAM-only

## Contatti fisici congelati

```text
MIN
  coarse:          1443 tick
  fine:            1443 tick
  spread:          0 tick
  limite URDF:     1451 tick

MAX
  coarse:          3443 tick
  fine:            3442 tick
  spread:          1 tick
  limite URDF:     3442 tick
  guard:           3506 tick
  baseline:        median 0, MAD 0
```

Il contatto MAX coincide con il limite URDF entro 1 tick. Il valore è un
contatto meccanico misurato, non ancora un limite operativo safe. MIN e MAX
restano dati separati dai futuri margini di esercizio.

## Ritorno e cleanup MAX

```text
ritorno finale M12: 3442 -> 2048
distanza:            1394 tick
budget calcolato:     39850 ms
tempo reale:          circa 17 s
stato finale:         Done 14/14
cleanup:              global torque OFF verificato
seriale dopo arresto: libera
```

Il precedente timeout fisso di 12 secondi era insufficiente per i ritorni
lunghi. La correzione calcola il budget dalla distanza mantenendo invariati
`GoalSpeed=80`, `TorqueLimit=400`, geometria, rilevatore di contatto e
allowlist RAM.

## Restart-safe entry verificato

```text
M42: 2385 -> prerequisite 2389
M13: 2054 -> prerequisite 2048
M11: 2047 -> prerequisite 2048
M12: 2367 -> home 2048
```

Sono ammessi soltanto residui appartenenti al profilo armato e dentro i
corridoi verificati. Residui di altri profili e pose fuori corridoio restano
hard-block prima del moto.

## Software ed evidenze

```text
norma-core main:       32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
PR #4 draft head:      8103392cc16b01d02653d5bb889ed616b3e31be7
workflow V13 #57:      PASS
local tested commit:   6ef728bc629a57efdabc43aa29ffc356c156b532
Station SHA-256:       d09d6d5a71a90ffd439c1dd83388e73ba48c6c5c24ba6109d1e6c6b0ab8a5278
ST3215 tests:          77 passed, 0 failed
```

Evidenze:

```text
09_Logs/Calibration/M12_native_min_max_2026-07-28/
```

## Contratto permanente

- Station unico proprietario della seriale;
- `GOAL_POSITION` unsigned `0..4095`;
- nessun signed-wrap;
- sole RAM: `TorqueEnable`, `Acc`, `GoalPosition`, `GoalSpeed`,
  `TorqueLimit`;
- nessun Reset, Position Offset, EEPROM, LOCK, RegWrite/Action, Save o Freeze;
- un solo profilo esplicitamente armato per avvio;
- global torque OFF verificato su successo e fallimento;
- nessuna esecuzione automatica dei contatti rimanenti.

## Prossimo profilo autorizzabile dopo questo checkpoint

```text
profilo:             LF_HIP_M13_MIN
probe:               M13
movimento tick:      crescente
home:                2048
baseline target:     2112
limite URDF MIN:     2560
guard:               2624
coarse / fine:       32 / 8 tick
backoff:             96 tick
allowed motors:      11, 12, 13, 42
prerequisite M42:    2389  (LH_UPPER +30°)
prerequisite M12:    2617  (LF_UPPER +50°)
prerequisite M11:    2048  (LF_LOWER 0°)
```

Il prossimo profilo deve essere prima validato offline sullo stesso contratto
restart-safe e distance-aware, poi eseguito una sola volta sotto supervisione
e congelato prima di passare a `LF_HIP_M13_MAX`.
