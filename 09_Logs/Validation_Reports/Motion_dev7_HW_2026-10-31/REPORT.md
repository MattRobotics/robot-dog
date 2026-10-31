# Collaudo hardware dev.7 (31/10/2026)

**Stato:** confermata dall'operatore come baseline hardware il 31/10/2026.

`1.0.1-dev.7` @ `4634de8007bd086ce07f8874119f47569f76ea1c`, APP `4c7e2c8a6e41fbd218cb09834c6543e3f039c349089cf02a7fc8bceded5bcc3c`. Installata e in esecuzione. Nessun merge, push, modifica firmware o calibrazione. Rollback dev.6 `5760eeee…` non usato, pronto in `flash/rollback/`.

La colonna Fonte indica le cartelle della sessione: log, NVS e checkpoint restano solo locali.

**Esito:** nessuna regressione osservata rispetto a dev.6. Cambi di quota diretti confermati, 0 release perse in tutta la sessione, dashboard e Twin regolari.

| Voce | Esito | Fonte |
|---|---|---|
| Flash APP-only, rilettura, 9 regioni, head | PASS | `flash/` |
| NVS prima, dopo flash, a fine sessione | `6d03b9dd…` invariata | `flash/`, `post/` |
| Boot, readiness, servo, calibrazione, STOP da fermo | PASS | `01-postflash/` |
| STAND 50, 70, 100, 110, 150 | PASS, 0 fault, 0 perse. Cambi diretti da posa ceduta `STAND_LIFT` | `02-session/` |
| 150 → 100 → 50 → 150 | PASS | `02-session/` |
| PRONE da 150, rialzata senza STOP, 50 → 150 | PASS | `02-session/` |
| WALK45 USB | PASS, regolare per l'operatore | `02-session/` |
| Quote via dashboard (120, 90, 150), WALK 60, PRONE, STOP | PASS | `03-dashboard/trace.json` |
| WALK45 da dashboard con Twin aperto | PASS, 0 stale su 80 campioni | `03-dashboard/trace2.json` |
| PRONE e STOP finali | PASS, SAFE_OFF 8191 | |

## Release perse

| Prova | dev.6 | dev.7 |
|---|---|---|
| WALK45 USB, stessi 21 s e stesso polling | 8 | **0** |
| WALK con telefono o dashboard | 22 | **0** (WALK 60 senza Twin, WALK45 con Twin) |
| Totale dal boot, sessione completa | 58 | **0** |

Sono tre cammini: risultato netto, non ancora statistica.

**Lavoro per release:** resta alto. Massimi 25,1 ms (USB), 36,9 ms (dashboard e Twin), 30,0 ms in PRONE_LIFT, con compensazione fino a 21,7 ms. La release pesante recupera nella successiva, quindi non si accumula un periodo. Il picco della compensazione rimane non spiegato; prima di aumentare la velocità del gait va chiarito.

## Compensazione IMU a 40 ms
Osservazione dell'operatore: camminata molto buona e sincronizzata. Piccolo abbassamento su tre zampe, anteriore 2–3 mm, poi ristabilizzato. Nessun confronto video con dev.6, quindi non si può dire se sia cambiato. Campioni a 2 Hz:

| | dev.6 USB | dev.7 USB | dev.7 dashboard |
|---|---|---|---|
| Rollio | −1,76…+0,86° | −1,93…+0,96° | −1,65…+1,21° |
| Beccheggio | −0,32…+1,65° | +0,08…+2,14° | −0,07…+2,17° |
| Scarto max tracking | 75 tick | 92 tick | 80 tick |

Beccheggio circa 0,5° più alto a 2 Hz: non è una prova di peggioramento, ma da tenere d'occhio.

## Misure fisiche
| Misura | Valore | Riferimento |
|---|---|---|
| Ventre a STAND 150 | 146,5 mm | Stima `base_link` 146,7 |
| Luce sotto il ventre da prono | 10,5 mm | Modello 12,1–12,8 |
| Scorrimento da prono a 150 | Anteriori +55 mm, posteriori 0 | Modello 50–53 e 0: **confermato** |
| Ritorno a prono | 4 zampe esattamente sui segni iniziali | Reversibile in questa prova |
| Percorso WALK | ~65 mm in 13 s prima di WALK STOP | Non confrontato col teorico |

## Previsioni non confermate
- La discesa a quota bassa non è arrivata alla situazione prevista dal banco: in discesa i residui a 50 mm sono ~5 tick contro 25 in salita, posa in `kHeight`. La regola di riposo in `kLift` non è stata esercitata; il fault previsto su dev.6 non è dimostrato sul robot.
- Quota a 50/100 mm dipende dal verso: 47,9/97,8 mm salendo, 49,4/99,5 scendendo.

## Non chiuso
- Causa dei picchi di compensazione.
- Correzione PRONE su terreno inclinato: non provata sotto inclinazione misurata.
- Stabilità WALK: margine modello 3,7 mm, CoM reale non misurato.
- Lieve testo arancione `MOTION_BUDGET_UNAVAILABLE` a coppia ON: comportamento previsto, formulazione migliorabile. 5 letture di stato scadute a 800 ms nella prima sequenza dashboard, nessuna nell'ultima.
- Picchi spuri di temperatura: 20, tutti filtrati. Temperatura reale massima 34 °C.
