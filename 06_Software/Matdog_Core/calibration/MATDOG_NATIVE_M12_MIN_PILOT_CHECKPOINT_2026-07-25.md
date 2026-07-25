# MATDOG — checkpoint pilot hardware nativo M12 MIN

**Data:** 2026-07-25
**Esito:** PASS
**Robot:** MATDOG
**Bus ST3215:** `5B14114953`
**Adapter:** `/dev/ttyACM0` / seriale stabile `5B14114953`
**Giunto pilot:** `LF_UPPER / M12 / limite MIN`
**NormaCore head usato dal pilot:** `286d6d1050c3f4abfad2139cb1325c934bb94c41`

## Risultato misurato

```text
contatto coarse: 1443 tick
contatto fine:   1443 tick
spread:          0 tick
baseline median: 1 raw
baseline MAD:    0 raw
ritorno home:    2048 comandato; circa 2039 osservato nella UI, entro 10 tick
stato finale:    Calibration Complete, 11 / 11
cleanup:         torque-OFF globale verificato
```

Il limite URDF MIN di M12 era `1451` tick. Il contatto fisico ripetibile è
stato misurato a `1443` tick, cioè 8 tick oltre il limite modellato nella
direzione MIN. Questo dato è un contatto meccanico misurato, non ancora un
limite operativo safe definitivo.

## Sequenza realmente eseguita

1. set esatto dei 12 ID MATDOG verificato;
2. torque-OFF globale verificato;
3. priming M12 sulla posizione presente;
4. sole scritture RAM: `TorqueEnable`, `Acc = 4`, `GoalPosition`,
   `GoalSpeed = 80`, `TorqueLimit = 400`;
5. ritorno a home `2048`;
6. baseline in movimento verso `1984`;
7. approccio coarse a passi di 32 tick;
8. contatto coarse `target=1431, present=1443`;
9. backoff di 96 tick e verifica recupero;
10. approccio fine a passi di 8 tick;
11. contatto fine `target=1431, present=1443`;
12. ripetibilità PASS con spread 0;
13. ritorno a home;
14. torque-OFF globale verificato.

## Sicurezza verificata

- `GoalPosition` unsigned standard;
- nessun reset servo;
- nessuna scrittura o unlock EEPROM;
- nessuna modifica Position Offset;
- nessun RegWrite/Action;
- nessun Freeze Calibration;
- Station unico proprietario della seriale;
- il comando automatico `ResetCalibration` generato dalla UI è stato
  rifiutato dal runtime gate prima della seriale;
- sovracorrente, status/errori, perdita torque, deriva torque-limit e deriva
  goal restano hard-abort;
- il rilevamento di stallo usa posizione, velocità, persistenza e distanza dal
  target; la corrente resta diagnostica e hard-abort, ma non è requisito per
  il contatto.

## Evidenze

```text
09_Logs/Calibration/M12_native_pilot_2026-07-25/matdog_m12_pilot_286d6d1_20260725-165017.log
09_Logs/Calibration/M12_native_pilot_2026-07-25/matdog_m12_pilot_286d6d1_20260725-165017.meta
09_Logs/Calibration/M12_native_pilot_2026-07-25/SHA256SUMS.txt
```

SHA-256 log:

```text
e77b70c4fc7418d85122bbc6bd7ce67c37894e9dc616a9063c6e0a9078aaa009
```

SHA-256 metadati:

```text
17145f7d3b119a32e5a78057224d1f0f3a23381d2c9223bd97a5d39662a1ba61
```

## Decisione

La foundation nativa RAM-only per il primo pilot M12 MIN è validata
fisicamente. Non estendere i `1443` tick direttamente a tutti i giunti e non
scriverli in EEPROM. Il prossimo sviluppo deve generalizzare la procedura ai
24 contatti dei 12 giunti, con profili espliciti per zampa e prerequisite
geometriche MATDOG.
