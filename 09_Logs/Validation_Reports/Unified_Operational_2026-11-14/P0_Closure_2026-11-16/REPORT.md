# MATDOG — chiusura Motion Timing P0 (16/11/2026): WALK_SETTLE, P4 e budget

Mandato dell'operatore del 16/11 «Chiusura Motion Timing P0» e mandato prioritario successivo («chiusura definitiva»). Questo rapporto raccoglie l'analisi offline; la sessione hardware è in `MATDOG_1_0_2_DIAG_2638204_HW_20261116/`. Robot spento, nessun flash dopo quella sessione.

## Decisione

1. **Peggioramento di P4** (29 → 47 overrun): verificato che **non è P4 né il layout**.
   - Oggi entrambe le build costano il **5–7 % in più per istruzione** rispetto al 16/11, a parità di istruzioni, clock, interrupt e preemption; cresce solo lo stallo istruzioni.
   - A parità di giorno e sequenza `2638204` non è più lenta di `82c1af9` (WALK 11,18 contro 11,30 ms per release, 733 contro 732 k istruzioni).
   - P4 però **non dà beneficio reale**: le release attenuate in overrun hanno ancora 2 soluzioni (15/15 e 24/25). **Esclusa.** (§A2)
2. **Picco WALK_SETTLE (24–27 ms):**
   - **verificato offline:** prova STOP rifiutata dopo la ricerca completa al bordo del workspace, con la correzione IMU «scappata» sulla fixture;
   - **sul chip resta un'ipotesi forte:** coincidono la firma (1 soluzione, 4 IK, nessun frame), gli stalli nella norma, l'eccesso di istruzioni stimato (2,4 M contro 1,5–2,6 M riprodotti) e il gruppo 5 (interrupt esclusi). Manca l'osservazione diretta del rifiuto. (§A, §B)
3. **Patch minima P5** (`StopPlan.h`, 9 righe aggiunte e 2 tolte): con `a = 0` la ricerca si ferma alla prima durata con un punto di controllo fuori dal box, che resta fuori a ogni durata più lunga (dimostrato, anche in virgola mobile). **Equivalenza:**
   - 25 M casi casuali e di confine bit a bit sull'host e 150 000 sul toolchain ESP32-S3 in QEMU: 0 differenze;
   - 111 750 chiamate reali registrate nel rig: 0 differenze;
   - frame identici in 162 103 release;
   - static audit PASS; `simple_motion`, `stop_support`, `prone`, `night_recovery` PASS. (§B)
4. **Costo** (QEMU, regime del chip): le release WALK_SETTLE rifiutate passano da 1 504 / 2 604 k a 655 / 688 k istruzioni (mediana / max); la release peggiore della corsa da 2 703 a 1 391 k. Sul chip è atteso ≈ 11–13 ms invece di 27 ms (stima). (§B.4)
5. **Regressioni:** nessuna. Le 8 suite non pertinenti falliscono identiche su `82c1af9`, per sorgenti legacy assenti e ambiente Node. (§F)
6. **Firmware:**
   - **installato:** `2638204fb4ef` (P4);
   - **candidata:** `a86c680` = `82c1af9` + P5, APP `f7bc8b62…e55b`, CLEAN, non flashata;
   - **rollback esatto:** `2638204`.
7. **Unica validazione hardware necessaria:** prova mirata D8–D11 su `2638204` e su `a86c680` nella stessa sessione, poi A–G e H. Serve a misurare se la firma WALK_SETTLE sparisce sul chip e a separare il fattore di sessione. (`candidate_a86c680/NEXT_SESSION_PLAN.md`)
8. **P0 non si chiude con P5:**
   - il **4–7 % delle release** sta fra 15 e 20 ms, per il ~90 % nei cammini (sterzati soprattutto);
   - restano gli overrun delle attenuate (2 soluzioni non riducibili in modo esatto), di WALK STOP e delle riprese;
   - **nessuna semplificazione esatta trovata.** L'unica leva identificata del −20 % necessario è portare in IRAM il percorso caldo del cammino (stallo istruzioni 42–46 % dei cicli): richiede un mandato. (§C)

## A. Prima divergenza di WALK_SETTLE (offline, prima della sessione)

Massimo del 16/11 pomeriggio: **26,82 ms**, release 29394, WALK_SETTLE dopo il rilascio dello yaw orario (D11), compensazione 20,96 ms con 1 soluzione, 4 IK, 4 Newton, 11 FK.

### A.1 Che cosa distinguono i gruppi PMU esistenti

La rotazione di `MotionTiming.h` arma un gruppo per release (`release % 7`). Il record `MOTION_PMU_HEAVIEST` conserva, per classe, la release più pesante dal boot con il suo gruppo e con CCOUNT e i due contatori per sezione.

| Gruppo | Contatori | Che cosa separa nella release critica |
|---|---|---|
| 0 | istruzioni ritirate / cicli | istruzioni misurate e IPC: calcolo contro stalli |
| 1 | cicli di stallo lato istruzioni / lato dati | quota di stallo |
| 2 | stalli istruzioni su memoria / mul-div iterativi | cache istruzioni |
| 3 | stalli dati su memoria / bolle di pipeline | cache dati e bolle |
| 4 | interrupt / eccezioni di finestra | carico di interrupt |
| 5–6 | cicli, istruzioni, stalli sopra il livello 0 | ISR e sezioni critiche |

Il gruppo della release critica non si sceglie: ogni occorrenza ne dà uno solo. `other_us` misura il tempo degli altri task del core nella release.

**Misurato sul chip il 16/11** (D11, gruppo 1), nella compensazione:

| Misura | Release critica | Tutte le release OTHER (stesso gruppo) |
|---|---|---|
| Cicli | 5 030 966 (240,1 MHz) | — |
| Stallo istruzioni | 31,2 % | 33,3 % |
| Stallo dati | 3,8 % | 3,7 % |
| Altri task | 0 µs | — |
| Frame trasmesso | no (`frame_us=0`: RAW uguale al precedente) | — |

- **Stalli nella norma.** Dalla chiusura «cicli = istruzioni + stalli + bolle» (decomposizione della classe: 47,6 + 33,3 + 3,7 + 18,0 = 102,6 %), con il rapporto bolle/istruzioni della classe (0,378), le istruzioni della compensazione sono **2,37 M** (banda 2,09–2,75 M con quel rapporto ×1,5…×0,5).
- **Confronto con il rig:** QEMU con lo stesso lavoro IK dà 0,62–0,69 M.
- **Prima divergenza:** un eccesso di **istruzioni** (≥ 1,3 M) dentro COMPENSATE, non di stalli né di preemption. È una stima: il conteggio diretto richiede il gruppo 0 su quella release.

### A.2 Riproduzione mirata dei passi D9/D11

`MATDOG_1_0_2_DIAG_2638204_HW_20261116/fixture_settle.py`:
- passi D8–D11 della suite qualificata (yaw antiorario 12 s, rilascio, yaw orario 12 s, rilascio), tra uno STAND 120 da PRONE_REST e un PRONE, con una dump `@MOTION TIMING` prima e dopo;
- stesse funzioni, comandi, controlli e regole di arresto della suite pinnata;
- nessun comando e nessun log nuovi; l'unico registratore HTTP resta `http_watch.py` a 1 Hz.

### A.3 Costo dei percorsi della compensazione (QEMU, stesso codice)

**Percorsi di una release WALK_SETTLE** (`compensate` → `compensateLevel`, fase `WALK_HOLD`):
- non può usare la release leggera T1;
- fit del supporto (3 FK);
- `compensationTarget` (4 IK nel modello a spigolo);
- `compensationSafe`:
  - conversioni RAW;
  - controlli della posa caricata;
  - stima della velocità dalla storia RAW;
  - prova STOP `StopPlan::build`.
- In caso di rifiuto il riferimento resta quello precedente: stesso RAW, nessun frame trasmesso.

**Unico ciclo a lunghezza variabile con contatori IK costanti:** la ricerca di `StopPlan::build`.
- Con accelerazione iniziale nulla (ogni chiamante), il vincolo di accelerazione migliora con la durata. Quello di box peggiora, perché i punti di controllo si allontanano da `q` lungo `v`.
- Quindi la ricerca riesce alla prima o seconda durata utile, oppure fallisce provandole tutte fino a 60 passi.

**Costi isolati** (`offline/stopplan_qemu/`, `result.txt`; istruzioni = CCOUNT × 3,088):

| Caso | Istruzioni |
|---|---|
| Prova riuscita (2 iterazioni, 12 giunti) | 95 k |
| Prova fallita, primo giunto fuori box = indice 0 | 0,41 M |
| idem, indice 2 | 1,20 M |
| idem, indice 5 | 2,40 M |
| idem, indice 8 | 3,60 M |
| idem, indice 11 | 4,79 M |
| idem, indice 11, altri giunti fermi | 1,71 M |

**Riproduzione del regime del chip** (`offline/settle_edge/`, albero `p4` = `e57dd89`, percorso WALK_HOLD identico a `82c1af9`/`2638204`):
- Il rig originale chiude WALK_SETTLE in 6 release: i servo simulati riportano velocità 0 e l'arrivo (tolleranza 12 tick, velocità ≤ 50) scatta dopo 100 ms.
- Sul chip i servo che inseguono la correzione riportano velocità. Sulla fixture la correzione «scappa»: le zampe non caricate seguono il riferimento, quindi il fit del supporto vede la correzione applicata più l'inclinazione del corpo (voluto ≈ applicato + 8–12 mrad nei campioni HTTP). Il settle quindi dura finché la correzione si ferma al bordo del workspace.
- **Modifica di misura** (solo strumento): un servo che si è mosso dall'ultima pubblicazione riporta una velocità sopra 50; HOLD finale di 8 s.
- **Host:** WALK_SETTLE di 48–72 release, correzione fino a 62–75 mrad. Alla fine 3–8 release consecutive sono rifiutate per **`STOP_PLAN`** (e `LOAD_BOX`). In HOLD il bordo è **`POINT_SAFE`**, come nei campioni HTTP del chip.
- **QEMU** (7 366 frame su 7 368 identici all'host; le 2 differenze sono l'ultima release tracciata di due scenari, IMU_STALE solo in QEMU):

| Release | n | COMPENSATE (mediana / max) | Prova STOP (mediana / max) |
|---|---|---|---|
| WALK_SETTLE accettata | 208 | 680 k / 725 k | 87 k / 127 k |
| WALK_SETTLE rifiutata `STOP_PLAN` | 22 | **1 502 k / 2 604 k** | 900 k / 2 016 k |
| WALK_SETTLE rifiutata `LOAD_BOX` | 10 | 590 k / 597 k | 0 |
| HOLD accettata | 931 | 83 k / 692 k | 40 k / 125 k |
| HOLD rifiutata `POINT_SAFE` | 772 | 293 k / 700 k | 0 |

- Le release rifiutate per `STOP_PLAN` hanno **lo stesso lavoro IK della release critica** (1/4/4/11) e 1,5–2,6 M istruzioni nella compensazione. La stima del chip è 2,37 M.

### A.4–A.5 Stato della causa e dati mancanti

- **Ipotesi H1:** al bordo del workspace, in WALK_SETTLE, la prova STOP fallisce dopo la ricerca completa. Ogni release rifiutata la ripete finché l'arrivo chiude la fase.
- **Evidenze:**
  - è riprodotta offline;
  - è compatibile con tutte le misure del chip: stalli nella norma, eccesso di istruzioni stimato, lavoro IK uguale, nessun frame, overrun multipli per rilascio (2–3), HOLD successivo a `POINT_SAFE`;
  - **non è ancora osservata sul chip.**
- **Dati mancanti con la strumentazione presente:**
  1. il conteggio diretto delle istruzioni della release critica: gruppo 0, deciso dalla rotazione;
  2. il codice di rifiuto delle release pesanti: lo stato HTTP a 1 Hz lo campiona di rado, e la finestra dei rifiuti è di 60–160 ms per settle;
  3. il numero di iterazioni della ricerca: nessun contatore.
- Nessuna patch prima della sessione. WALK_SETTLE non è modificato.

## C.7 (preparazione) Affidabilità del criterio IMU a 1 Hz

Metrica di `session_compare.py`: `hypot(voluto − applicato)` in roll e pitch, campioni HTTP a 1 Hz nei cammini sterzati. Riferimento invariato: `46b3175` (media 1,39, massimo 13,30 mrad).

- **Sul chip** (16/11, entrambe le sessioni) i campioni più alti cadono nel transitorio di slew all'avvio dei cammini sterzati (motivo `SLEW` o primo campione `ENVELOPE`). Lì la correzione recupera a 40 mrad/s, quindi il valore campionato dipende dalla fase del campione: 13,30 contro 13,87 mrad corrispondono a ~14 ms di differenza di fase.
- **Nessun dato a risoluzione maggiore sul chip:** voluto e applicato per release non sono registrati e aggiungerli vorrebbe dire strumentazione nuova. L'IMU a 2 Hz nel log USB dà l'assetto del corpo, che sulla fixture è fermo: non misura l'errore di correzione.
- **Rig** (`offline/imu_sampling/`, serie per release dell'albero P4, compensazione WALK identica a `82c1af9`; 22 cammini sterzati, 52 135 release):
  - per release: media 1,02, p90 1,34, massimo 32,97 mrad;
  - errore massimo nei primi 0,5 s di ogni cammino: mediana 8,4, massimo 21,5 mrad;
  - campionatore a 1 Hz con fase casuale per cammino (2 000 estrazioni): **media 1,00 [5–95 %: 0,80–1,36]**, **massimo 19,4 [5–95 %: 14,7–29,0]** mrad.
- **Conclusione:**
  - la media a 1 Hz è affidabile entro circa ±30 %;
  - il massimo a 1 Hz varia di un fattore ~2 per la sola fase di campionamento, quindi non distingue differenze di pochi mrad tra due sessioni;
  - il criterio resta quello fissato e si applica com'è; questa incertezza si riporta accanto, non serve a farlo passare.
- P4 non modifica la compensazione WALK: nel rig le statistiche IMU di WALK e RECOVERY sono identiche tra `719912a` e P4 (n, media, p99, massimo e motivi; `imu_track_p4_vs_719912a.txt`). Tra `82c1af9` e `2638204` la statistica sterzata è quindi attesa uguale.

## A2. Perché `2638204` (P4) ha più overrun di `82c1af9` (offline, dati del chip del 16/11)

**Comparabilità** dei due test:
- stessa suite A–G e blocco H pinnati, stessa fixture;
- il 16/11 la suite `82c1af9` è partita subito dopo il boot del flash; oggi la suite `2638204` è partita dopo la prova mirata t263 sullo stesso boot;
- l'attesa ACC alla prima attivazione è caduta in A2 il 16/11 (6 perse) e in t263 oggi (registro già a 220 dopo t82: 3,9 ms, 0 perse);
- la tabella per blocco ora esclude la finestra boot → A0 (`session_compare.py` allineato, autotest sul 16/11 identico byte per byte).

**Overrun per singola release** (`tools/overrun_timeline.py`: incrementi del contatore dal boot nello stato HTTP a 1 Hz; esatti quando l'overrun ha alzato il massimo del comando):

| Sessione | Totale | Attenuate | Sterzati | WALK STOP | WALK_SETTLE | Ripresa | Frontale |
|---|---|---|---|---|---|---|---|
| `82c1af9` 16/11, A–G | 29 | 16 | 4 | 2 | 6 | 0 | 1 |
| `2638204` oggi, A–G | 47 | 22 | 11 | 6 | 3 | 3 | 2 |
| `82c1af9` H / `2638204` H | 2 / 4 | 2 / 4 | — | — | — | — | — |
| `82c1af9` oggi, prova mirata | 8 | 6 | 0 | 2 | 0 | 0 | 0 |
| `2638204` oggi, prova mirata | 14 | 9 | 2 | 0 | 3 | 0 | 0 |

**Tempo per istruzione** (N+C, istruzioni dal gruppo PMU 0; `ns_per_instruction.txt`):

| Classe | `82c1af9` 16/11 | `82c1af9` oggi (t82b) | `2638204` oggi |
|---|---|---|---|
| WALK (codice identico) | 9,73–9,81 ns | 10,45 ns | 10,17–10,34 ns |
| OTHER | 8,45–8,90 ns | 9,30 ns* | 9,14–9,52 ns |
| RECOVERY | 9,70–10,25 ns | — | 10,15–10,33 ns |

\* include la tenuta della STOP dopo l'interruzione dello strumento.

- **Fattore di sessione (dimostrato):** oggi entrambe le build costano il 5–7 % in più per istruzione rispetto al 16/11.
  - Stesse istruzioni per release WALK nei blocchi abbinati (±1,4 %); stesso clock (240,1 MHz).
  - Stessi interrupt ed eccezioni di finestra (~600 per release, ~1 % dei cicli); altri task 0.
  - Cresce lo stallo di memoria istruzioni (0,40–0,41 → 0,42–0,46).
  - L'origine non è identificata; non è la build.
- **Fattore di build (stesso giorno, stessa sequenza, 1 656 release WALK ciascuna, 732 contro 733 k istruzioni):** `2638204` 11,18 ms per release contro 11,30 ms di `82c1af9`. **Nessuna penalità di layout dimostrata.**
- **Effetto algoritmico di P4:** istruzioni per release OTHER −1,5…−11 % per blocco. Le release attenuate che fanno overrun però hanno ancora 2 soluzioni: 15/15 su `82c1af9`, 24/25 su `2638204`. La compensazione della release massima dei comandi attenuati è invariata (paired per passo: 14,2–15,8 contro 13,6–15,7 ms).
- **Rig con il regime del chip** (servo che riportano la velocità; `offline/settle_edge/`): P4 porta le release attenuate a 2 soluzioni da 386 a 112 su 1 862. Le restanti sono quelle in cui la scala voluta supera la proiezione, cioè le più pesanti: proprio quelle che sul chip fanno il massimo di ogni comando.
- **Stato e traiettoria:** i picchi dipendono dallo stato della fixture e cambiano da una sessione all'altra in entrambi i versi (D5 20,6 → 27,0; D9 24,0 → 21,7; D11 26,8 → 22,1 ms). Sulla fixture le zampe scariche fanno «scappare» la correzione IMU fino al bordo del workspace, e quali release lo incontrano varia.
- **Conclusione:** il passaggio 29 → 47 è spiegato dal fattore di sessione (+5–7 % per istruzione, con una coda di release vicina ai 20 ms) e dalla variabilità dei picchi, non da P4 né dal layout. **P4 però non dà un beneficio reale su P0**, perché non tocca le release che fanno overrun: **esclusa dalla prossima candidata.**

## B. WALK_SETTLE: la prova STOP rifiutata e la patch P5

### B.1 Comportamento di `StopPlan::build`

`StopPlan::build(q, v, a, lo, hi, vmax, amax)`:
- Prova le durate `t = 0,02·s`, con `s` da `first` a 60.
- Per ogni giunto controlla i punti di controllo del quartico (`q`, `p1 = q + t·v/4 + …`, `end = q + t·v/2 + t²·a/12`): nel box, finiti, con residuo `|p − q| ≤ 0,35 rad`. Poi la velocità e l'accelerazione nei punti estremi.
- Accetta la prima durata valida. Altrimenti rifiuta dopo la 60ª.

Con `a = 0` (tutti i chiamanti: `beginStop`, `compensationSafe`, `loadedArrestInside`):
- La velocità `v·(1 − 3u² + 2u³)` **non dipende da `t`**.
- L'accelerazione di picco `1,5·|v|/t` **diminuisce con `t`**: la ricerca parte già da `first = ⌊1,5·max|v|/amax/0,02⌋`.
- I punti `p1 = q + t·v/4` ed `end = q + t·v/2` sono affini in `t` e valgono `q` a `t = 0`.
  - `q` è nel box, controllato in ingresso, e l'insieme dei `t` con il punto nel box è un intervallo che contiene 0.
  - Il residuo `t·|v|/k` cresce con `t`.
  - **In virgola mobile** ogni passo (`steps·0,02`, `t·v`, divisione per 2 o 4 esatta, `q + x`, `+ (t·t·0)/12 = +0`) è un arrotondamento monotono: i valori calcolati sono monotoni in `t`.
- **Quindi:** un punto fuori box, fuori residuo o non finito a una durata resta fuori a tutte le durate successive. La ricerca può riuscire solo alla prima o seconda durata utile; altrimenti fallisce provandole tutte.

**Costi misurati** (QEMU, `offline/stopplan_qemu/`):
- prova accettata: ~95 k istruzioni;
- prova rifiutata: da 0,41 M a 4,79 M, secondo quanti giunti si valutano prima di quello che esce dal box.

**Esiti STOP_PLAN** (rig con il regime del chip, `offline/settle_edge/`):
- rifiuti solo in WALK_SETTLE, quando la correzione IMU arriva al bordo del workspace: 3–8 release consecutive per settle;
- in HOLD il bordo è `POINT_SAFE` (rifiuto prima della prova, economico), come nei campioni HTTP del chip.

### B.2 Patch P5 (`87338f8` su `research/p0-stopplan-20261116`; nella candidata `b104f00`)

```cpp
const double t=steps*.02;bool valid=true,outside=false;
...  // controllo dei punti: in caso di fallimento {valid=false;outside=true;}
if(valid){duration=t;return true;}
if(rest&&outside)return false;   // nessuna durata successiva può essere valida
```

- 9 righe aggiunte e 2 tolte in `src/simple/StopPlan.h`.
- `outside` lo impostano solo i controlli dei punti, monotoni. Il vincolo di accelerazione, l'unico che migliora con `t`, non lo imposta mai.
- Con `a ≠ 0` il comportamento è identico al vecchio.
- Dopo un rifiuto `end[]` è memoria di lavoro: nessun chiamante la legge. `test_stop_support.cpp` legge `end` solo dopo un `build()` accettato, dove resta identico.

### B.3 Equivalenza (vecchia `719912a`/`82c1af9` contro P5)

| Prova | Casi | Esito |
|---|---|---|
| Casuali bit a bit (esito, durata, q, v, a; con piano accettato anche `end[]` e `sample()` in 6 istanti), `-O2` | 20 000 000: 14,0 M `a = 0` casuali, 3,0 M costruiti esattamente sul confine o a ±1 ulp alla durata d'uscita, 2,0 M `a ≠ 0`, 1,0 M input non validi; 6,3 M accettati, 13,7 M rifiutati | **0 differenze** |
| Idem, `-O0` (codegen diverso) | 5 000 000 | **0 differenze** |
| Idem sul toolchain ESP32-S3 (double in virgola mobile software, compilatore e flag del firmware) sotto QEMU | 150 000 (47 368 accettati, 102 632 rifiutati) | **0 differenze** |
| Casi registrati: ogni chiamata reale del codice Motion esegue anche la vecchia ricerca e confronta, rig 64 scenari | 154 735 release, 106 964 chiamate (5 rifiutate) | **0 differenze**; frame, stati e digest identici a `p3b` in tutte le release |
| Idem, regime del chip (`settle_edge`) | 7 368 release, 4 786 chiamate (20 rifiutate) | **0 differenze**; frame identici; iterazioni di ricerca 8 418 → 7 338 |

- STOP, limiti, condizioni di accettazione, piani validi e comportamento dei fault sono identici per costruzione e per prova: stesso esito e stessa durata in ogni chiamata.
- La verifica STOP non è indebolita: rifiuta esattamente gli stessi casi, solo prima.

### B.4 Costo delle release peggiori prima e dopo P5

**Riproduzione del regime del chip** (QEMU, `offline/settle_edge/settle_edge_qemu_p3b_vs_p5.txt`, stesso codice di `82c1af9` senza e con P5):

| Release | Senza P5 (mediana / max) | Con P5 (mediana / max) |
|---|---|---|
| WALK_SETTLE rifiutata `STOP_PLAN` (22 release), COMPENSATE | 1 504 k / **2 604 k** | 655 k / **688 k** |
| idem, prova STOP (massimo) | 2 016 k | 98 k |
| WALK_SETTLE accettata (208), HOLD (1 701) | invariate | invariate |
| Release peggiore dell'intera corsa | 2 703 k | **1 391 k** (−49 %) |

- Frame e codici di rifiuto sono identici fra le due build (7 366/7 366) e fra QEMU e host.
- **Stima sul chip:** la release critica del 16/11 ha 5,03 M cicli di compensazione per ≈ 2,4 M istruzioni stimate. Con P5 la compensazione rifiutata scende a ≈ 0,66 M istruzioni, cioè ≈ 6 ms al rapporto cicli/istruzioni misurato. La release torna a ≈ 11–13 ms, sotto il picco attuale di 26,8–27,0 ms.
- È una stima; la misura si fa sulla fixture.

**Struttura della patch** (ELF contro `82c1af9`):
- cambia solo la dimensione di `StopPlan::build` (1 949 → 1 959 B); il resto del codice flash si sposta di 8 B;
- IRAM e DRAM sono identiche.
- P4 aveva spostato il codice di −60/+12 B senza penalità misurabile nello stesso giorno.

## C. Percorsi ancora sopra budget

**Distribuzione delle release ≥ 15 ms per tipo di comando** (istogramma del firmware per comando, `above15_by_command_type.txt`):

| Tipo di comando | Quota ≥ 15 ms (`82c1af9` / `2638204`) | Release 15–20 ms | ≥ 20 ms |
|---|---|---|---|
| Cammino sterzato, con rilascio e settle | 72,9 % / 74,9 % | 2 013 / 3 905 | 51 / 102 |
| Cammino frontale, preset, WALK STOP | 16,5 % / 15,8 % | 459 / 831 | 9 / 16 |
| STOP | 4,3 % / 4,4 % | 120 / 230 | 3 / 4 |
| PRONE e STAND/HEIGHT attenuati | 4,4 % / 2,6 % | 107 / 117 | 18 / 23 |
| Ripresa dopo STOP | 1,8 % / 1,8 % | 51 / 92 | 0 / 6 |

Sull'intera suite: **4,15 % / 7,41 % delle release fra 15 e 20 ms** (2 707 / 5 094), 0,12 % / 0,22 % sopra 20 ms.

**Ordinati per contributo al superamento dei 16 ms**, con la correzione necessaria per ciascuno:

1. **Cammini (sterzati e frontali, ~90 % delle release ≥ 15 ms).**
   - Media ~10–11 ms per release, coda fra 15 e 20 ms.
   - Nel rig, il 5 % più pesante spende il 55 % delle istruzioni nella soluzione nominale e il 37 % nella compensazione (`walk_heavy_scopes_qemu.txt`). Sul chip lo stallo istruzioni vale il 42–46 % dei cicli N+C.
   - **Nessuna ripetizione esatta trovata:** la struttura T2/T1 alterna già soluzione nominale e compensazione, e i Newton per IK sono già ~1.
   - Per portare la coda sotto 16 ms serve un −20 % sul costo della release. L'unica leva individuata con quell'ordine di grandezza e risultati identici è portare in IRAM il percorso caldo del cammino (soluzione nominale e IK inclinata), oltre le 6 funzioni di IRAM G1, con lo stesso metodo (miss evitati per byte).
   - **Non implementata:** non è una semplificazione del calcolo, richiede un budget IRAM (oggi 87 215 B di IRAM text; heap libero ~70 KB, minimo ~46,6 KB) e una misura sul chip. È la proposta per il passo successivo.
2. **Attenuate STAND, PRONE, HEIGHT (16–22 overrun per suite, 20–24 ms; solo ~3 % delle release ≥ 15 ms).**
   - Le release in overrun hanno 2 soluzioni: sonda, poi proiezione quando la scala voluta supera la proiezione.
   - **Nessuna semplificazione esatta:** la proiezione dipende dal risultato della sonda, e i Newton sono già ~1 per IK (8 IK, 9 Newton).
   - Riduzioni possibili ma non esatte (non implementate): dividere sonda e proiezione su due release, con il riferimento in ritardo di una release, da ridimostrare per STOP e box; oppure l'IRAM del punto 1.
   - Alcune HEIGHT hanno anche ~5 ms di REST alla partenza del segmento (pianificazione).
3. **WALK STOP (2–6 overrun, 20,6–24,8 ms):** la release più pesante (E8, 23,7 ms) aveva stallo istruzioni al 68 % nella soluzione nominale e al 58 % nella compensazione. È un percorso freddo in cache; la correzione è la stessa del punto 1.
4. **Ripresa dopo STOP (0–3 overrun, 19,8–22,5 ms):** nel rig il 67 % delle istruzioni del 5 % più pesante è nella soluzione nominale; stessa correzione.
5. **WALK_SETTLE:** risolto offline da P5 per le release rifiutate dalla prova STOP; da verificare sul chip.

**Fuori dalle release (non overrun):** lettura servo 2,2–4,1 ms per release dentro la finestra. Toglierla dal rilascio è un cambio di architettura: non proposto senza necessità dimostrata.

## D. Qualità e accettazione

- **Target invariati:** 0 perse operative, 0 overrun, massimo ≤ 16 ms, nessuna regressione.
- **Attese ACC a coppia OFF**, separate dalle perse operative:
  - con i servo appena alimentati (avvio a freddo) la prima attivazione verifica il registro 85 in 126–128 ms: 5–6 perse;
  - dopo un reset del solo ESP (flash) i servo tengono 220: 3,9 ms, 0 perse;
  - la protezione resta; i contatori non sono corretti.
- **Dump diagnostiche:** 6–21 ms, a coppia OFF, 0 perse a riposo (D3).
- **IMU:**
  - massimo sterzato a 1 Hz: 13,87 (`82c1af9`) e 13,66 mrad (`2638204`) contro il riferimento 13,30 di `46b3175`: **criterio non soddisfatto**, non rilassato;
  - la media (1,39 e 1,11) lo soddisfa;
  - i campioni massimi cadono nel transitorio di slew all'avvio dei cammini; nel rig il massimo a 1 Hz varia di un fattore ~2 con la sola fase (§C.7). Non è distinguibile una regressione reale dalla variabilità del campionamento.
  - **Misura più rappresentativa proposta, non implementata:** massimo e media di \|voluto − applicato\| aggiornati a ogni release nel record del comando già pubblicato via HTTP (due numeri, nessun log).

## E. DALY

- Episodio delle 11:51Z documentato in `MATDOG_1_0_2_DIAG_2638204_HW_20261116/daly_episode/`: 14/129 byte, poi RX ferma per 130 s, rientro spontaneo; causa UNKNOWN.
- D1 e D2 restano nella candidata, che deriva da `82c1af9`. Nessuna modifica al BMS, ai timeout o al polling; nessuna nuova campagna.

## F. Nuova candidata

- **`a86c680`** (`cand/diag-p0-p5-20261116`) = `82c1af9` + P5 + nota di composizione: 2 file, +13/−2 righe.
- La base è `82c1af9`, il miglior timing misurato sul chip. P4 è esclusa.
- Firmware installato e candidata sono distinti: sul robot c'è `2638204fb4ef`, la candidata non è flashata.
- Dettagli, criteri e sequenza in `candidate_a86c680/NEXT_SESSION_PLAN.md`.
