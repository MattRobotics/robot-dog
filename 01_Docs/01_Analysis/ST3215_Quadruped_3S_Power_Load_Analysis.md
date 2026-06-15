# ST3215_Quadruped_3S_Power_Load_Analysis

Questo file Excel documenta l’analisi elettrica e quasi-statica dei servomotori **Feetech ST-3215-C018** utilizzati nel progetto **robot-dog**.

Il workbook è stato creato per verificare se la configurazione attuale di alimentazione e la coppia disponibile dei servo sono coerenti con il carico previsto del quadrupede, in particolare nel caso del **giunto più sollecitato** durante la fase di appoggio e di sollevamento.

---

## Scopo del file

Il file **`ST3215_Quadruped_3S_Power_Load_Analysis.xlsx`** è stato costruito per:

- raccogliere in un unico posto i dati principali del servo **Feetech ST-3215-C018**;
- analizzare l’alimentazione di **12 servo** su **serial bus TTL**;
- modellare una distribuzione di potenza con **4 rami da 3 motori** (`3+3+3+3`);
- calcolare la **caduta di tensione** sul motore più lontano del ramo;
- stimare la **coppia disponibile** e la **corrente assorbita** in funzione della tensione batteria;
- verificare il comportamento del robot in condizioni di carico su **4 zampe** e su **2 zampe**;
- stimare l’**autonomia batteria** con una batteria da **9000 mAh**.

---

## Dati di partenza usati nel file

### Servo analizzato

- **Modello:** Feetech `ST-3215-C018`
- **Tipo:** smart serial bus servo TTL con encoder magnetico
- **Tensione nominale di riferimento:** `12 V`
- **Coppia nominale @12V:** `10 kg·cm`
- **Coppia di stallo @12V:** `30 kg·cm`
- **Corrente nominale @12V:** `0,9 A`
- **Corrente di stallo @12V:** `2,7 A`
- **Kt:** `11 kg·cm/A`
- **Lunghezza cavo servo:** `15 cm`

### Configurazione robot

- **Numero servo:** `12`
- **Architettura:** quadrupede `12 DOF`
- **Peso robot completo:** `3,3 kg`
- **Braccio critico massimo:** `110 mm`
- **Assegnazione ID servo:** organizzata per zampa e giunto

### Alimentazione

- **Batteria:** `LiPo 3S`
- **Tensione minima considerata:** `9,0 V`
- **Tensione nominale:** `11,1 V`
- **Tensione massima:** `12,6 V`
- **Capacità batteria:** `9000 mAh`
- **Fattore capacità utile:** `0,8`

### Cablaggio di potenza

- **Distribuzione alimentazione:** `4 rami da 3 servo`
- **Sezione conduttore:** `0,25 mm² (24 AWG)`
- **Materiale:** rame
- **Resistività usata:** `0,0175 Ω·mm²/m`

---

## Cosa contiene il workbook

Il file Excel è suddiviso in **3 fogli** principali.

### 1. `Data_Motor`

Foglio di riepilogo del motore e delle ipotesi di progetto.

Contiene:
- le specifiche elettriche ufficiali del servo a `12 V`;
- le specifiche meccaniche principali;
- i dati del cablaggio (`15 cm`, connettore, sezione usata);
- i dati di progetto relativi a batteria, peso robot e braccio critico.

### 2. `Power_Supply_Calc`

Foglio dedicato all’analisi dell’alimentazione.

Contiene:
- i parametri di ingresso del sistema;
- il calcolo della resistenza equivalente del ramo;
- il calcolo della **caduta di tensione** sul **3° servo del ramo**;
- la stima della tensione effettiva disponibile sul motore per tre stati batteria:
  - **minima carica**;
  - **nominale 3S**;
  - **massima carica**;
- la stima della **corrente nominale** e della **corrente massima** per motore;
- la stima della **coppia nominale** e della **coppia massima erogabile**;
- la corrente totale di sistema per `12 motori`;
- la potenza totale del sistema;
- la stima di **autonomia batteria**.

### 3. `Robot_Load_Analysis`

Foglio dedicato alla verifica del giunto più sollecitato del quadrupede.

Contiene:
- il calcolo del carico sul giunto critico con **braccio massimo di 110 mm**;
- la verifica del caso di supporto del peso su:
  - **4 zampe**;
  - **2 zampe**;
- la coppia richiesta al giunto;
- la corrente richiesta al servo;
- la caduta di tensione sul ramo nel caso conservativo;
- la tensione effettiva al motore;
- la coppia massima disponibile in quel punto di lavoro;
- il margine tra coppia disponibile e coppia richiesta.

---

## Formule principali implementate

### Resistenza di un segmento di cavo (andata + ritorno)

```text
R_seg = 2 · ρ · L / A
```

Dove:
- `ρ` = resistività del rame
- `L` = lunghezza del tratto di cavo
- `A` = sezione del conduttore

### Caduta di tensione sul 3° motore del ramo

Caso generale:

```text
ΔV_3 = R_seg · (I1 + 2·I2 + 3·I3)
```

Caso semplificato con tre motori allo stesso carico:

```text
ΔV_3 = 12 · ρ · L · I / A
```

### Tensione effettiva disponibile sul motore

```text
V_mot = V_batt - ΔV
```

### Coppia massima disponibile stimata

```text
T_disponibile = T_12V · V_mot / 12
```

### Corrente richiesta dal giunto critico

```text
I_richiesta = T_richiesta / Kt
```

### Autonomia batteria

```text
Autonomia [min] = ((Capacità_mAh / 1000) · Fattore_capacità_utile / Corrente_totale_A) · 60
```

---

## Assunzioni del modello

Il file usa alcune assunzioni semplificative ma utili per il dimensionamento preliminare:

- il bus dati è comune a tutti i servo, mentre l’alimentazione è separata in **4 rami da 3 servo**;
- la caduta di tensione viene valutata sui soli conduttori di potenza `+V` e `GND`;
- la stima della coppia disponibile fuori da `12 V` viene ottenuta scalando in funzione della tensione effettiva al motore;
- l’analisi del robot è **statica / quasi-statica**;
- il caso su **2 zampe** è usato come scenario più gravoso;
- per l’autonomia viene usato un **fattore di capacità utile = 0,8**.

---

## Come leggere i risultati

### In `Power_Supply_Calc`

Serve per capire:
- quanta tensione arriva realmente al motore più lontano del ramo;
- quanta corrente assorbe il sistema nelle diverse condizioni di batteria;
- quanta potenza elettrica totale è richiesta;
- quale autonomia utile ci si può aspettare con la batteria attuale.

### In `Robot_Load_Analysis`

Serve per capire:
- se il servo riesce a sviluppare abbastanza coppia nel caso più gravoso;
- quanto margine hai tra coppia richiesta e coppia disponibile;
- come cambia la disponibilità di coppia al variare della tensione batteria.

---

## Interpretazione corretta del file

Questo file **non è un simulatore dinamico completo del robot**, ma uno strumento di verifica tecnica per:

- dimensionare il sistema di alimentazione;
- controllare la compatibilità tra massa robot e servo scelti;
- identificare i casi di carico più critici;
- avere una stima realistica della tensione ai motori e dell’autonomia.

È quindi da intendere come documento di:
- **analisi preliminare**;
- **verifica ingegneristica**;
- **supporto alla progettazione meccanica ed elettrica**.

---

## File collegati utili

- `01_Docs/03_Datasheets/` → datasheet dei servo e dei componenti elettrici;
- `01_Docs/02_Architecture/` → documenti di architettura del quadrupede;
- `01_Docs/04_Notes/` → note progettuali, osservazioni e appunti di sviluppo.

---

## Nota finale

Se in futuro aggiornerai cinematica, massa robot, batteria, cablaggio o disposizione dei rami, questo file Excel dovrà essere aggiornato di conseguenza, perché i risultati dipendono direttamente da questi parametri.
