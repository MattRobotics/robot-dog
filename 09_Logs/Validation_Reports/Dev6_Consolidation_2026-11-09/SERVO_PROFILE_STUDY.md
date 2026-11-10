# ST3215 — studio del profilo Speed/Acceleration (consolidamento dev.6, 09/11/2026)

Classe: **VERIFIED OFFLINE** per la legge e la prova di STOP; **UNKNOWN** per corrente, coppia e inseguimento reale del servo. Nessuna scrittura EEPROM; nessun accesso hardware.

## Semantica dei registri (libreria SCServo installata, `SMS_STS.h`)

| Registro | Indirizzo | Area | Unità | Uso nel firmware |
|---|---:|---|---|---|
| Acceleration | 41 | RAM | 100 step/s² (1 step = 2π/4096 rad) | `Drive.acc`, scritto a ogni `SYNC_WRITE` |
| Goal Position | 42–43 | RAM | step | riferimento a 20 ms |
| Goal Time | 44–45 | RAM | ms | non usato (0) |
| Goal Speed | 46–47 | RAM | step/s | `Drive.speed` |
| Torque Limit | 48–49 | RAM | ‰ | invariato |
| Lock | 55 | RAM | — | non toccato: nessuna scrittura EEPROM (indirizzi < 40) |

`WritePosEx`/`SyncWritePosEx` scrivono 7 byte a partire da 41 (ACC, posizione, tempo, velocità): il profilo vive in RAM e accompagna ogni riferimento. Il servo genera internamente un profilo trapezoidale verso ogni nuovo goal con quei limiti; il firmware usa gli stessi due numeri nella prova di arresto (`StopPlan`: `vmax = speed·2π/4096`, `amax = acc·100·2π/4096`).

Baseline WALK `{1000, 50}` = **1,534 rad/s**, **7,67 rad/s²**. STAND `{480, 13}`, calibrazione `{160, 8}`: fuori studio.

## Matrice valutata offline

Strumento: `walk_tilt_envelope_check.sh --drive-speed S --drive-acc A`, con la prova di STOP sui riferimenti esatti, sui punti medi T2 e su 8 fasi di griglia, agli inviluppi certificati (`kWalkTilt75`, nuovo `kWalkTilt100`). Picco della legge (indipendente dal profilo): **1,275 rad/s** (LF_UPPER, 100 mm), **5,76 rad/s²** (RH_LOWER).

| Profilo | Velocità / accelerazione | 75 mm | 100 mm | Note |
|---|---|---|---|---|
| **{1000, 50} baseline** | 1,534 / 7,67 | PASS | PASS | margine legge: velocità 17 %, accelerazione 25 % |
| {1000, 30} | 1,534 / 4,60 | FAIL (8 067 STOP) | FAIL (133 566 STOP) | accelerazione sotto il picco della legge |
| {800, 50} | 1,227 / 7,67 | PASS | **FAIL** | velocità sotto il picco della legge a 100 mm |
| {1000, 80} | 1,534 / 12,27 | PASS | PASS | arresto più corto |
| {1000, 100} | 1,534 / 15,34 | PASS | PASS | arresto più corto |
| {1500, 50} | 2,301 / 7,67 | PASS | PASS | velocità non vincolante |
| {1500, 100} | 2,301 / 15,34 | PASS | PASS | — |

### Cosa comprerebbe un'accelerazione maggiore

Raggio vincolante di WALK100 (beccheggio in giù, prova di STOP con T2), limite fattibile per bisezione:

| ACC | limite fattibile | inviluppo ×0,85 |
|---:|---:|---:|
| 50 | 15,0 mrad | **12,7 mrad** (installato in questa dev.6) |
| 80 | 30,6 mrad | 26,0 mrad |
| 100 | 33,1 mrad | 28,2 mrad |

L'hardware chiede −15…−18 mrad (dev.4 a terra, WALK100) e −24…−29 mrad (dev.17 statico): con ACC 80 la correzione di livello non verrebbe tagliata in WALK100.

## Scelta

**Resta `{1000, 50}`.** Motivazione quantitativa:

1. È l'unico profilo con evidenza hardware: dev.4 WALK100 a terra, H1 su fixture, STAND150 caricato.
2. La legge ne usa l'83 % della velocità e il 75 % dell'accelerazione, e tutte le prove offline passano.
3. La sola ragione per cambiare è l'autorità di livellamento a 100 mm (12,7 → 26 mrad con ACC 80). Comporta però un arresto fisicamente più brusco: coppia di picco più alta sulle gambe caricate, dove la RCA di carico ha mostrato cedimento P-only. Mancano dati di corrente e frenata, e andrebbero ri-derivati tutti gli inviluppi.

Questo non è un PASS hardware: si sceglie di non introdurre un cambiamento fisico senza dati.

**Candidato per un A/B hardware futuro:** `{1000, 80}`. Prima occorrono:
- corrente e temperatura in WALK100 e STOP;
- l'errore di inseguimento;
- gli inviluppi `kWalkTilt75/100` e `kSteerTilt` ri-derivati con lo stesso strumento.

Nessun selettore di profilo in dashboard né persistenza (vincolo del mandato).
