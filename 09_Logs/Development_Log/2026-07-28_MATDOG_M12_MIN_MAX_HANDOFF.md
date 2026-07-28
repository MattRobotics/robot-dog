# MATDOG — handoff dopo chiusura LF_UPPER M12 MIN/MAX

**Data:** 2026-07-28
**Stato:** M12 MIN e MAX fisicamente validati e congelati

## Stato canonico atteso dopo il push

```text
robot-dog/main: nuovo commit prodotto da questo checkpoint
norma-core/main: 32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
PR #4: draft, aperta, non mergiata
PR #4 head: 8103392cc16b01d02653d5bb889ed616b3e31be7
local V13 tested commit: 6ef728bc629a57efdabc43aa29ffc356c156b532
```

## Risultati M12

```text
LF_UPPER_M12_MIN: 1443 / 1443, spread 0
LF_UPPER_M12_MAX: 3443 / 3442, spread 1
return home:       PASS
prerequisites:     restored
final torque OFF:  PASS
serial free:       PASS
```

## Prossima fase

Preparare un solo profilo:

```text
LF_HIP_M13_MIN
```

Numeri revisionati:

```text
M13 home=2048, baseline=2112, URDF_MIN=2560, guard=2624
M42 prerequisite=2389
M12 prerequisite=2617
M11 prerequisite=2048
```

Prima del moto:

1. verificare remoti e clone locali;
2. usare il binario restart-safe distance-aware già testato o ricostruirlo
   deterministicamente dallo stesso sorgente;
3. eseguire test mirati sul profilo M13 MIN e suite completa;
4. avviare Station con un solo arm value;
5. robot completamente sostenuto, quattro zampe libere;
6. premere soltanto Auto Calibrate, mai Save o Reset;
7. congelare coarse, fine, spread, baseline, ritorno e cleanup prima di M13 MAX.

Non automatizzare la sequenza dei 21 contatti ancora mancanti.
