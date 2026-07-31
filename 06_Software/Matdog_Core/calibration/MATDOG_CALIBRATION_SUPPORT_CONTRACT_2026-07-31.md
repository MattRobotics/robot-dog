# MATDOG — Contratto del supporto durante la calibrazione

**Data:** 2026-07-31  
**Stato:** vincolante per il redesign UPPER → LOWER → HIP  
**Frame:** `base_link`

## 1. Dato geometrico acquisito

Durante la calibrazione MATDOG è sostenuto con il riferimento inferiore del `base_link` a:

```text
Z = 0.000 m
```

Il piano del tavolo è a:

```text
Z = -0.180 m
```

La distanza verticale nominale tra `base_link` e tavolo è quindi 180 mm.

## 2. Fixture esterna

La forma e l'ingombro della fixture non vengono modellati.

L'operatore dichiara che la fixture:

- è stata progettata specificamente per MATDOG;
- è stata verificata manualmente e sperimentalmente;
- contiene gli scarichi necessari per le zampe;
- non può interferire con i giunti quando viene rispettata la sequenza ordinata;
- non richiede ulteriori quote o volumi keep-out software.

Questa è un'assunzione esplicita del test hardware, non una conclusione ricavata dall'URDF.

## 3. Sequenza che rende valida l'esclusione fixture

L'esclusione della fixture è valida soltanto con questo ordine:

```text
UPPER MIN/MAX
→ UPPER orizzontale
→ LOWER MIN/MAX
→ posa coordinata UPPER + LOWER compatta
→ HIP MIN/MAX
```

Un profilo HIP isolato, una lower lasciata estesa o una upper non portata nella posa prevista invalidano l'assunzione e devono essere bloccati dal software.

## 4. Controlli che restano obbligatori

Anche senza modellare la fixture, l'audit deve verificare:

- limiti e guard software;
- collisioni MATDOG–MATDOG sulle collision mesh reali;
- collisioni con `base_link`;
- collisioni tra zampe;
- continuità delle transizioni;
- distanza dal piano del tavolo a `Z=-0.180 m`;
- dipendenza UPPER → LOWER → HIP;
- recovery in ordine inverso;
- torque OFF globale finale.

## 5. Contratto macchina

Il runner hardware deve verificare l'hash del file:

```text
MATDOG_CALIBRATION_SUPPORT_CONTRACT.yaml
```

Il contratto non autorizza movimenti diversi da quelli coperti dall'audit e non sostituisce i gate di self-collision, guard, telemetria e recovery.
