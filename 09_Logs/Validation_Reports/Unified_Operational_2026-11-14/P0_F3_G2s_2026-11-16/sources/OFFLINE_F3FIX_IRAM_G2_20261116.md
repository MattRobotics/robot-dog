# Correzione F3 e ottimizzazione IRAM P0 — rapporto offline (16/11/2026 sera)

Mandato dell'operatore «Correzione F3 e chiusura Motion Timing P0». MATDOG spento, installato `a86c680`. Nessun flash, nessuna prova hardware. Due candidati separati, poi uno integrato. Nessuna modifica a STOP/StopPlan, accelerazioni, limiti, calibrazioni, DALY.

## Sintesi

1. **F3 corretto offline, al costo di 32 → 29,6 mm/s nel FAST.**
   - **Picco:** il FAST preset 2 passa da 31 a **29 tick RAW per release**. Il limite della prova STOP è 31,36 tick: margine 2,36 tick (1,36 con il tick di quantizzazione).
   - **Velocità:** 32 mm/s non è fattibile con il margine chiesto (§1.2). Il FAST passa a **74 mm ogni 2,5 s = 29,6 mm/s**, una sola costante; profilo di swing, duty, quota, alzata e drive invariati.
   - **Gate offline:** il nuovo gate della secante fallisce su `a86c680` e passa sul candidato. La batteria di regressione dà uscite identiche fuori dal FAST e PASS nel FAST.
2. **IRAM G2 misurata offline.**
   - **Funzioni:** 19 in IRAM, oltre alle 6 di G1.
   - **Miss I-cache medi, traccia QEMU sul layout reale del candidato:** WALK −31 %, compensazione −31 %, WALK STOP −33 %, STAND/HEIGHT a due soluzioni −70 %, RECOVERY −37 % (§2.5).
   - **Memoria (immagine di produzione):** IRAM +10 300 B, heap interno −10 240 B.
   - **Identità:** frame e digest identici su host e su Xtensa (QEMU).
   - **Stima sul chip** (calibrata su G1): −4…−8 % sul cammino, −8…−17 % sulle release STAND/HEIGHT a due soluzioni. Va misurata con il PMU.
3. **Sopra 16 ms restano:** WALK STOP (≈ 23 ms stimati), cammini sterzati (≈ 19–20), RECOVERY (≈ 18–19), parte delle release STAND/HEIGHT a due soluzioni (≈ 17–19). **P0 non si chiude con G2**; servono altre leve (§4).
4. **Candidati:**
   - `6146ef1` (F3), `3fd8758` (IRAM G2);
   - `cb9ed85` (F3 + G2 + ri-pin dei test, su `a86c680`), con pacchetto di flash e rollback pronti, non eseguiti.
5. **Prossima prova hardware:** una sola sessione sulla fixture, un solo flash di `cb9ed85` dopo un riferimento su `a86c680` nella stessa sessione, poi il blocco FAST. Precauzione DALY QON + 6 min (`hw_next/NEXT_SESSION_PLAN.md`).

## 1. Correzione F3

### 1.1 Meccanismo

- Con `a = 0`, StopPlan accetta al massimo **2,405 rad/s** al drive del cammino (residuo 0,35 rad, frenata 84, griglia 20 ms), cioè **31,36 tick per release**.
- Nel FAST preset 2 la secante di picco è di 31 tick: RF_UPPER a metà volo (progressione 0,47), LF_UPPER al ciclo 0,91. 32 tick rifiutano il passo di correzione e il nuovo tentativo, quindi FAULT `WALK_IK_OR_WORKSPACE`.
- Il rig riproduce il rifiuto alla fase del FAULT sul robot (ciclo 2,408).

### 1.2 Perché non basta il profilo di swing

- **Scansione:** 24 coppie (rampa iniziale, rampa finale) nel rig, sorgente di produzione, fixture e terreno, ritardo 0/1, rumore IMU (`f3fix/out/scan/`).
  - Spostare la velocità del piede **prima** nel volo abbassa il picco degli UPPER anteriori (fine volo), ma alza il LOWER posteriore (inizio volo), RH/LH_LOWER a 31.
  - Spostarla dopo fa il contrario. Le rampe finali corte producono rifiuti e FAULT (fino a 147).
- **Miglior profilo a 80 mm:** 30 tick (0,28 / 0,50). Non raggiunge il margine di 2 tick.
- **Il picco scala con la velocità:** a duty fisso la velocità media del volo è v·d/(1−d), indipendente dal periodo. Ridurre il duty toglierebbe supporto statico.

### 1.3 Compromesso quantificato (scansione completa, tutti i gruppi)

| Falcata FAST preset 2 | Velocità | Secante max | Rifiuti/FAULT | Margine box min |
|---|---|---|---|---|
| 80 mm (attuale) | 32,0 mm/s | 31 | 3 / 0 | 3,8 tick |
| 76 mm + rampe 0,28/0,50 | 30,4 mm/s | 29 | 0 / 0 | 3,8 (scansione ridotta; cambia il profilo di tutto il FAST) |
| 75 mm | 30,0 mm/s | 30 | 0 / 0 | 3,8 |
| **74 mm (scelta)** | **29,6 mm/s** | **29** | **0 / 0** | **3,8** |
| 73 mm | 29,2 mm/s | 29 | 0 / 0 | 4,3 |

- **Gruppi:** preset 0/1/2, joystick sterzato, joystick frontale a fondo scala (fixture e terreno), preset 1→2 in cammino, STANDARD→FAST in cammino, E→F3 fixture e terreno.
- **Scelta:** 74 mm è la modifica minima che raggiunge −2 tick. Cambia solo `strideM[2]` del FAST, cioè il preset 2 e il fondo scala del joystick frontale FAST (definito dalla stessa costante).
- **Conseguenza:** il FAST e lo STANDARD preset 2 hanno ora quasi la stessa velocità (29,6 e 30 mm/s). Il FAST resta diverso per ciclo (2,5 s), quota (105 mm) e alzata (6 mm). Decisione di progetto per l'operatore.

### 1.4 Gate offline nuovo (`scripts/offline_motion/run_stop_secant_gate.sh`)

**Cosa verifica:**
- ogni frame inviato di ogni modo, preset, direzione joystick e transizione, più la sequenza E→F3;
- condizioni: piano, inclinazione da fixture (due quadranti), pendenza con livellamento, ritardo di una release, release in ritardo;
- la secante come la prende `rateFromHistory`, più 1 tick di quantizzazione, deve stare dentro la frontiera di `StopPlan::build` al drive di quel frame. La frontiera si trova chiamando StopPlan stesso, senza riscriverne la formula.

| Build | FAST preset 2 / joystick FAST / E→F3 | STANDARD preset 2 e joystick | SLOW | Esito |
|---|---|---|---|---|
| `a86c680` | 31 tick, margine −0,64 | 30, +0,36 | ≤ 17 (frontiera 20) | **FAIL** |
| `6146ef1` | **29, +1,36** | 30, +0,36 | ≤ 17 | **PASS** |

### 1.5 Qualifica (`f3fix/regressions/`, base `a86c680` contro `6146ef1`)

- **Identici byte per byte:**
  - test unitari della messa a terra;
  - digest dei frame;
  - matrici STOP finestra SLOW, STANDARD, FAST e FAST all'indietro;
  - handover FAST nelle 4 direzioni;
  - controllo di supporto della camminata;
  - inviluppi d'inclinazione di ogni modo e falcata tranne FAST 2;
  - inviluppi joystick SLOW e STANDARD.
- **Diversi come atteso, tutti PASS:**
  - inviluppo FAST falcata 2: 74 mm, 0 errori nominale/IK/box/STOP;
  - servo plant: 0 FAULT, STOP tutti verificati, tutti i modi;
  - test unitari: solo i valori FAST (74 mm, joystick 29,6 mm/s, durate dei cambi modo).
- **Preesistenti su `a86c680`, identici sul candidato (non da F3):**
  - matrice STOP finestra SLOW avanti: 160/168, `STOP_MEASURED_OUTSIDE_BOX`, decisione di progetto documentata, fallita anche il 14/11;
  - inviluppo joystick SLOW: `random_3` con margine d'inclinazione −3,0 mrad, 6 frame. Il 14/11 il ramo operativo passava: va verificato sulla linea diagnostica.
- **Inviluppo joystick FAST con CAD** (supporto statico reale, gambe e raccordo): PASS su entrambi. Supporto minimo 0,69 → **0,95 mm**, gioco non-piede invariato 2,36 mm.
- **Audit statico:** `STATIC_AUDIT = PASS` su entrambi (il candidato ha un file in più, il gate).

### 1.6 Rischi residui

- **STANDARD preset 2 e joystick frontale STANDARD:** 30 tick, margine reale 1,36 tick. Passano il gate con il tick di quantizzazione. Nel rig nessun rifiuto. Fuori mandato: è un candidato al prossimo margine.
- **Correzione dell'RCA F3 del 16/11:** il joystick frontale a fondo scala nel FAST usa la stessa costante del preset 2. Allora era a 31 tick, non ≤ 25 come scritto (la scansione usava la falcata sterzata). Il rischio era lo stesso del preset 2; la correzione lo copre.

## 2. IRAM G2

### 2.1 Metodo (quello di G1)

- **Traccia:** rig QEMU con il sorgente di `a86c680`, 5 596 release in cinque percorsi marcati (`iram2/rig/iram_scenarios.h`), log di esecuzione della flash in streaming verso il modello di I-cache 16 KB 8 vie (`iram2/tools/icache_sim`, generalizzato a N percorsi).
- **Selezione:** classifica per miss evitati per byte (`rank2.py`); effetto esatto di ogni insieme ripetuto sul dump (`icache_replay`, `eval_sets.py`).
- **Base:** `a86c680` con G1 già in IRAM. Una release tocca ancora 24–36 KB di codice in flash.

### 2.2 Base: miss medi/massimi per release (modello 16 KB)

| Percorso | Release | Istruzioni in flash | Miss medi | Miss max | KB toccati |
|---|---|---|---|---|---|
| WALK nominale | 545 | 98 891 | 1 203 | 1 405 | 34,5 |
| WALK + compensazione IMU | 1 935 | 107 961 | 1 334 | 1 630 | 36,4 |
| WALK STOP / SETTLE | 933 | 100 849 | 1 200 | 1 963 | 34,0 |
| STAND/HEIGHT a due soluzioni | 1 611 | 83 176 | 762 | 1 304 | 24,2 |
| RECOVERY | 547 | 93 892 | 1 079 | 1 786 | 32,0 |

### 2.3 Curva beneficio/byte (insiemi avidi, effetto esatto)

| IRAM aggiunta | WALK | Compensazione | STOP | STAND/HEIGHT | RECOVERY |
|---|---|---|---|---|---|
| 4 KB | −27 % | −28 % | −29 % | −34 % | −28 % |
| 8 KB | −34 % | −35 % | −36 % | −48 % | −36 % |
| 12 KB | −42 % | −42 % | −45 % | −73 % | −47 % |
| 16 KB | −53 % | −51 % | −56 % | −87 % | −57 % |
| 24 KB | −72 % | −69 % | −74 % | −96 % | −77 % |

Ogni KB costa un KB di heap interno. Sul chip `heap_min_free` è stato 41,6–42,0 KB nelle tre suite del 16/11, il blocco libero massimo 31,7 KB.

### 2.4 Candidato G2 (`3fd8758`)

- **19 funzioni:**
  - libreria Motion: contatto con normale al suolo, contatto inclinato nel mondo, `validBodyPose`, `pointToBase`, `bodyPoseFromRollPitch`, `invert3`, `gaitOffset`, `legPhase`, `validGaitParameters`, `tiltFromQuaternion`, conversione dello stato del contatto;
  - `SimpleMotion`: `compensationTarget`, `readTimed`, `frameTimed`, `pointSafe`, `send`, `rememberReference`, `rateFromHistory`, `measured`.
- **Escluse `StopPlan::build` e `jointSlot`:** sono inline nell'header. Il pool di letterali di una funzione COMDAT resta in flash e il link fallisce (`l32r: literal placed after use`). `StopPlan::build` da sola varrebbe altri −7 punti sui cammini: richiede di definirla fuori linea, con gli script dei test da aggiornare.
- **Immagine di produzione** (ROBOT_POWERED, CLEAN): `.iram0.text` 87 215 → 97 515 B, `.flash.text` −9 756 B, `_heap_start` +10 240 B, APP 1 784 288 B.
- **Effetto simulato** (insieme esatto sul dump della base): WALK 1 203 → 795 (−34 %), compensazione 1 334 → 877 (−34 %), STOP 1 200 → 765 (−36 %), STAND/HEIGHT 762 → 223 (−71 %), RECOVERY 1 079 → 656 (−39 %). Massimi: −28, −31, −37, −38, −20 %.
- **Misura sul layout reale del candidato (traccia QEMU di `3fd8758`):** §2.5.
- **Identità:**
  - sorgente: diff limitato agli attributi;
  - digest host uguali; digest QEMU: §2.5;
  - batteria host: invariata per costruzione, l'attributo è vuoto fuori da ESP.

### 2.5 Misura sulla traccia del candidato

Traccia QEMU di `3fd8758` (`iram2/out/g2/`), stessi 5 596 release, modello 16 KB:

| Percorso | Miss medi base → G2 | Δ | Miss max base → G2 | Δ | Codice flash per release |
|---|---|---|---|---|---|
| WALK nominale | 1 203 → 829 | −31 % | 1 405 → 1 085 | −23 % | 34,5 → 26,9 KB |
| WALK + compensazione | 1 334 → 917 | −31 % | 1 630 → 1 180 | −28 % | 36,4 → 28,9 KB |
| WALK STOP / SETTLE | 1 200 → 804 | −33 % | 1 963 → 1 303 | −34 % | 34,0 → 26,6 KB |
| STAND/HEIGHT a due soluzioni | 762 → 225 | −70 % | 1 304 → 858 | −34 % | 24,2 → 16,5 KB |
| RECOVERY | 1 079 → 682 | −37 % | 1 786 → 1 439 | −19 % | 32,0 → 24,7 KB |

- **Identità:** digest QEMU identici alla base (`f78342099a26cb2b`, `7ea9f32a2be1c03f`), stessi stati e istogrammi.
- Istruzioni (CCOUNT sotto icount) +0,02 %: le chiamate fra IRAM e flash usano la forma lunga. Pila minima del task Motion 7 352 → 7 368 B.
- IRAM del rig +10 304 B (65 751 → 76 055).

### 2.6 Stima sul chip e rischi

- **Calibrazione G1** (15/11): −36…−43 % di miss simulati hanno dato −8…−11 % di ms per release sul chip (fattore ≈ 0,24). Il metodo assoluto (≈ 1 µs per miss simulato) dà circa la metà.
- **Stima G2** (dai miss misurati sul layout reale):
  - WALK e compensazione −4…−7 %;
  - STOP −4…−8 %;
  - STAND/HEIGHT a due soluzioni −8…−17 %;
  - RECOVERY −4…−9 %.
- **Rischi:**
  - heap −10 KB: atteso `heap_min_free` ≈ 31 KB, blocco massimo da verificare;
  - il modello ignora il traffico del core 0 e la PSRAM, per cui il beneficio reale può essere maggiore;
  - nessun cambio funzionale.

## 3. DALY

- RCA UNKNOWN, nessuna modifica.
- **Precauzione per il prossimo collaudo:** QON + 6 minuti e 150 poll consecutivi validi. Non entra nel firmware.
- **Osservazione passiva dell'XY-017:** `daly/PASSIVE_OBSERVATION_XY017.md`.

## 4. Percorsi sopra 16 ms (stima per `cb9ed85`)

| Classe (suite `a86c680`, 16/11) | Oggi | Stima con G2 |
|---|---|---|
| WALK STOP con nominale pesante | 25,5 ms | ≈ 23–24 |
| Attenuate STAND/HEIGHT a due soluzioni | 20,3–23,0 | ≈ 17–21 |
| Cammini sterzati | 20,2–21,9 | ≈ 18,6–21 |
| RECOVERY | 20,1–21,5 | ≈ 18–20,6 |
| Fascia 15–20 ms dei cammini | 4–7 % delle release | ridotta, non eliminata |

**Leve successive, quantificate, non implementate:**
- `StopPlan::build` fuori linea in IRAM: −7 punti di miss sui cammini;
- IRAM più ampia (16–24 KB: −51…−77 %), limitata dall'heap;
- I-cache da 32 KB: richiede di ricompilare il core. Con la base attuale il modello dà WALK −42 %, STAND −91 %, RECOVERY −48 %, cumulabile con G2, al costo di 16 KB di SRAM.

Nessuna di queste da sola porta WALK STOP sotto 16 ms: il resto è calcolo (istruzioni), non cache.

## 5. Candidati e commit (locali, non pubblicati)

| Branch | Commit | Contenuto | Immagine |
|---|---|---|---|
| `cand/f3-fast74-20261116` | `6146ef1` | F3 + gate della secante + testi dashboard | APP `afe0747c…`, 1 783 744 B |
| `cand/iram-g2-20261116` | `3fd8758` + `dfc1156` (ri-pin dei 7 sorgenti nel gate) | IRAM G2 | APP 1 784 288 B, build `3fd87581a5cf` |
| `cand/p0-f3-g2-20261116` | **`cb9ed85`** (`a0451de` + `e3fe5a0` + `c3bb3e0` nota + ri-pin) | integrazione dopo la qualifica separata; audit statico PASS | APP `254eca05…`, 1 784 288 B |

## 6. Prossima prova hardware

`hw_next/NEXT_SESSION_PLAN.md`:
- un QON con la precauzione DALY;
- riferimento `a86c680` nella stessa sessione (prova mirata D8–D11, blocchi A e B);
- un flash di `cb9ed85`, stessi passi;
- blocco FAST (E → F1–F9) e blocco H;
- chiusura PRONE_REST.

Pacchetto in `hw_next/preflash` (dry gates PASS, rollback esatto `a86c680`).
