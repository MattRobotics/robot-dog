> Copia del rapporto offline. I percorsi relativi (`rig/`, `tools/`, `out/`, `trees/`, `suites/`, `audit/`, `p3a/`, `loco/`, `candidate_82c1af9/`, `candidate_2638204/`) si riferiscono alla cartella delle evidenze `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/` (sigillata, `SHA256SUMS`), non a questo repository. Il codice è sui branch locali `research/p0-peaks-20261116`, `cand/diag-p0-peaks-20261116` e `cand/diag-p0-p4-20261116` (bundle `backups/git/robot-dog-p0-peaks-20261116.bundle`).

# Motion Timing P0 — picchi residui dopo T2R1M, sviluppo offline (16/11/2026)

Lavoro offline svolto durante e dopo la validazione hardware di `46b3175` (`../MATDOG_1_0_2_DIAG_46B3175_HW_20261116/REPORT.md`). Nessun flash di questi candidati.

Le istruzioni Xtensa contate in QEMU (`-icount 3`, ×3,088) **non sono millisecondi**. P0 si chiude solo sul chip: **non è chiuso**.

## Sintesi per stato (aggiornata dopo la validazione di `82c1af9`, 16/11 sera)

**1. Risultati hardware verificati**
- **`46b3175`** (fixture, 16/11 mattina): suite A–G 65/65 + blocco H 19/19; RECOVERY 15,1–16,4 → 10,9–12,0 ms; picco 36,1 → 27,0 ms; overrun 323 → 157.
- **`82c1af9` = E1 + E1b + P3a + P3b + D3** (fixture, 16/11 pomeriggio; `../MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116/REPORT.md`):
  - un solo flash; A–G 65/65 + H 19/19; NVS, calibrazione e STOP invariati;
  - WALK −10/−12 % per release (9,8–10,7 ms), RECOVERY −10/−11 % (9,8–10,6 ms);
  - overrun **161 → 31** (suite + H);
  - picco WALK 27,0 → 23,7 ms, RECOVERY 22,5 → 19,8 ms, OTHER 25,5 → 26,8 ms;
  - **0 perse operative** (6 dell'attesa ACC);
  - D3: dump grande 20,7–21,4 ms sul chip, passaggio del registro 85 ~4 ms, 0 perse a riposo.
- **DALY:** cinque avvii il 16/11 senza silenzio. Causa UNKNOWN, non risolta.

**2. Miglioramenti dimostrati soltanto offline**
- **P4** (`e57dd89`; candidato `2638204` = `82c1af9` + P4; §9):
  - una sola soluzione nelle release attenuate quando la scala accettata non supera la proiezione;
  - release OTHER a 2 soluzioni: host 5 376 → 904, QEMU 3 917 → 697;
  - stesse sequenze di stato, RAW ≤ 1 tick, STOP e IMU invariati;
  - `static_audit` e quattro suite Motion PASS;
  - **non flashato**.

**3. Cause ancora ipotetiche**
- **WALK_SETTLE dopo un rilascio sterzato** (fino a 26,8 ms; compensazione 21 ms con sole 4 IK):
  - QEMU, con lo stesso lavoro IK, dà ~0,7 M istruzioni;
  - l'eccesso sul chip non è spiegato. Il rig non riproduce la fase lunga del chip (correzione fino a 58 mrad);
  - ipotesi non dimostrate: ricerca lunga della prova STOP (`StopPlan::build`) o ramo della posa caricata con correzioni grandi.
- **Perse a riposo dopo la dump `@MOTION TIMING`:** compatibili con i dati (la dump grande supera il periodo), non confermate da un evento.
- **Silenzi DALY.**

**4. Problemi residui per chiudere P0** (0 perse operative, 0 overrun, ≤ 16 ms)
- Release attenuate di STAND/PRONE/HEIGHT (18 overrun su 31): affrontate da P4, da misurare sul chip.
- WALK_SETTLE dopo rilasci sterzati (8): causa da misurare prima di qualsiasi patch.
- Cammini sterzati (4) e WALK STOP (1): soluzione e compensazione nella stessa release.
- Attesa ACC di 100 ms alla prima attivazione dopo il boot (6 perse a ogni avvio).
- Stallo di istruzioni dominante; ~3,6–4 ms di lettura per release.

## Esito

| # | Punto | Esito | Stato |
|---|---|---|---|
| 0 | T2R1M sul chip (16/11) | RECOVERY −27/−28 % per release (istruzioni −40/−45 %); WALK invariato; picco 36,1 → 27,0 ms; overrun 323 → 157; 0 perse in WALK/RECOVERY | VERIFIED HARDWARE |
| 1 | Compensazione piena con IK di bordo | **E1** `8346b48`: seme di Newton = nominale della release + offset di correzione accettato. WALK max 1 430 → 1 172 k; release > 1,27 M 73 → 0 | IMPLEMENTED NOT VALIDATED (VERIFIED OFFLINE) |
| 2 | Picchi di resync | sul chip oggi **nessuna release persa in WALK/RECOVERY**, quindi nessun resync dopo una persa. Restano l'inizio della ripresa (posizione 0) e la posizione 2 del calendario T2. **E1b** `1b17674` (seme anche nella ripresa): RECOVERY max 2 156 → 1 562 k; posizione 2 1 808 → 1 243 k (effetto di E1) | IMPLEMENTED NOT VALIDATED |
| 3 | ALIGN / STAND_DEPLOY | **P3a** `157c95f`: `ViaPlan::plan` ricalcola solo le blend dei segmenti cambiati, **identico bit per bit**. **P3b** `719912a`: la release che costruisce un piano tiene la sonda alla scala accettata se non supera la proiezione. OTHER max 2 045 → 1 376 k; > 1,5 M 92 → 0 | IMPLEMENTED NOT VALIDATED |
| 4 | Altra IRAM | non proposta oggi (§4) | — |
| 5 | Target ≤ 16 ms | **non dimostrato**. Il modello dà WALK 15,0–18,8 ms, RECOVERY 18,8–25,0, OTHER 17,0–22,0; ma oggi sul chip il picco WALK (27,0 ms) supera il massimo del modello per T2R1M (22,9 ms) | modello |
| 6 | Nuova sorgente di perse | 2 perse all'attivazione da PRONE_REST (C1, E1), fuori da WALK/RECOVERY e fuori dal conteggio del comando | VERIFIED HARDWARE, causa UNKNOWN |

Branch di ricerca locale `research/p0-peaks-20261116` su `0fb22b0` (T2R1M). Worktree `~/MATDOG/worktrees/matdog-p0-peaks-20261116`. Quattro commit, uno per candidato, misurati separatamente e cumulativamente. Diff complessivo: `SimpleMotion.cpp` +27 −3, `ViaPlan.h` +15 −2. Nessuno stato nuovo nei membri, nessuno scheduler, nessun limite o gate toccato.

## Metodo

Il rig del rapporto P0 strutturale (`rig/`, `tools/`, `run.sh`), con il `SimpleMotion` di produzione e la conversione gen-1 reale:
- host: 64 scenari, ogni release tracciata;
- QEMU Xtensa: 46 scenari, costo per release, frame confrontati con l'host per nome di scenario.

Il riferimento T2R1M ricostruito oggi riproduce bit per bit i digest del 15/11 (64/64).

Misure aggiuntive, solo in copie di misura e mai nel firmware:
- `run_e1sh.sh` / `trees/e1sh`: shadow ad anello aperto di E1 e punto fisso del modello di bordo;
- `run_p0only.sh` / `trees/*_m`: solo lo scenario fixture in QEMU, con scope `PLAN`, `PLANCHK`, `GENSTAND`, `VIAS`;
- `trees/planprobe`, `trees/cnt`: dimensioni dei piani e conteggio delle valutazioni `cruise`.

## 1. Compensazione piena con IK di bordo (E1)

- **Osservazione:** il massimo WALK di T2R1M in QEMU (1 430 k) è una release interpolata con compensazione piena nel modello di bordo: 8 IK, 12 passi di Newton, 23 FK. Nell'host 713 release WALK hanno ≥ 7 IK.
- **Prima divergenza:**
  - il seme di Newton era `goal_`, la posa della release precedente, che è indietro di 1–2 release di moto;
  - `worldEdgeContactInverseKinematicsTilted` calcola il sollevamento del fillet dall'HIP del seme e risolve una seconda volta se dopo la prima soluzione cambia di oltre 20 µm;
  - con un seme vecchio questo succede su tutte e quattro le gambe.
- **Patch minima:**
  - il seme, gamba per gamba, è il nominale della release più l'offset della correzione accettata, cioè la stessa previsione della release leggera T1, quando è recente e dentro i limiti URDF;
  - altrimenti si usa `goal_` come prima;
  - cambia solo il punto di partenza: modello di contatto, tolleranza, limiti, box e prova STOP sono invariati.
- **Regressione (host):**
  - stesse sequenze stato/causa ed esiti in 64/64 scenari;
  - ad anello chiuso il RAW diverge al massimo di 3 tick, salvo `JOYSTICK_HELD_SEQUENCE` (55 tick, 20 mrad di correzione applicata), lo scenario in cui il riferimento esatto non è unico e dove già T2R1M divergeva di 35 tick dal baseline.
- **Shadow ad anello aperto** (stesso ingresso, due semi; build inerte: digest identici a T2R1M):
  - 54 044 soluzioni piene, validità identica nel 100 %;
  - RAW diverso di **al massimo 1 tick** nel 6,1 % dei frame (3 571 giunti); |Δq| massimo 0,34 mrad;
  - rispetto al punto fisso esatto del modello di bordo (sollevamento coerente a 1e-9 m), E1 è più vicino nel 78 % delle gambe: errore medio 27,4 → 4,3 µrad, massimo 304 → 218 µrad.
- **Costo:** sotto.

## 2. Picchi di resync (E1b)

- **Osservazione sul chip** (16/11, T2R1M):
  - 0 perse in WALK e RECOVERY (15/11: 3), quindi nessun resync dopo una persa;
  - il picco WALK di oggi (27,0 ms, release 29281 in D10, yaw orario) porta **NOMINAL 11,2 ms e COMPENSATE 11,7 ms nella stessa release**. Il caso «soluzione + compensazione piena» esiste ancora sul chip nei cammini sterzati, anche fuori dai resync. La diagnostica conserva solo la release massima e non dice perché la release leggera non sia stata presa (rifiuto di `compensationSafe` sull'offset riusato oppure posizione 2 del calendario).
- **In QEMU** (T2R1M) i picchi RECOVERY sono:
  - posizione 0, l'inizio della ripresa: soluzione esatta + compensazione piena, fino a 2 156 k con 8 IK di bordo;
  - posizione 2: look-ahead + compensazione piena, fino a 1 808 k.
- **E1b:**
  - all'inizio della ripresa l'offset accettato ha più di 2,5 periodi (è di prima dello STOP), quindi E1 non si applicava;
  - nella ripresa (`walkRecover_`) il seme usa anche quell'offset, perché la ripresa trattiene la stessa correzione;
  - se la soluzione fallisce da quel seme, si ripete dal seme precedente: il seme non può aggiungere un rifiuto;
  - host: rispetto a E1 differiscono 35 release di inizio ripresa (≤ 1 tick); release RECOVERY con ≥ 7 IK 3 → 0 (T2R1M 86).
- **Calendario T2 «pigro»** (0, 1, 3, 5…), che eliminerebbe la posizione 2: **non implementato**. Dopo E1 la posizione 2 vale 1 243 k, non è più il massimo di classe, e cambierebbe il primo bracket di ogni resync.

## 3. ALIGN / STAND_DEPLOY (P3a, P3b)

- **Osservazione:** i 92 picchi OTHER > 1,5 M sono esattamente 2 per ogni STAND da PRONE (46 scenari): le release ALIGN → DEPLOY e DEPLOY → LIFT.
- **Scomposizione** (QEMU, scope di misura, release ALIGN → DEPLOY):
  - `ViaPlan::plan` ~920 k;
  - verifica dei via ~70 k;
  - `generateStand` con compensazione attenuata a 2 soluzioni (sonda + proiezione) ~800 k.
- **P3a, esatta:**
  - `b[k]` dipende solo da `v[k]` e `v[k+1]`, ma ogni passaggio ricalcolava tutte le blend, con due righe `cruise` ciascuna (divisioni double emulate in software);
  - ora si ricalcolano solo dove una velocità è cambiata, ogni riga una volta, con le stesse espressioni;
  - piano DEPLOY: 2 112 → 900 valutazioni `cruise`; `ViaPlan::plan` ~920 → 484 k;
  - **identità bit per bit:** 200 000 piani casuali (`p3a/test_viaplan_equiv.cpp`, -O0 e -O2: `v`, `b`, `t`, `duration`, `iterations` uguali) e tutti i frame RAW dei 64 scenari; in QEMU frame E1 = E1+P3a in 46/46.
- **P3b:**
  - nella release che ha appena costruito un piano (`planned_ && segmentStartUs_ == now`, stato esistente) la sonda alla scala accettata è tenuta se non supera la proiezione: una correzione non più grande di quanto l'attenuazione consente, per quella sola release;
  - una proiezione che si restringe risolve subito come prima;
  - host: nessun frame RAW diverso, correzione applicata entro 0,04 mrad;
  - QEMU: ALIGN → DEPLOY 1 606 → < 1 208 k.
- **Resta:** DEPLOY → LIFT (1 376 k) = piano 374 k + `heightTarget` + 1 soluzione di compensazione 462 k.
  - Una variante di `cruise` senza divisione (prodotto per v/d precalcolato) toglierebbe ~200 k, ma non è identica bit per bit: non implementata.

## 4. IRAM ulteriore

- Non proposta oggi: le patch del §1–3 riducono le istruzioni, l'IRAM riduce gli stalli. Lo stallo di istruzioni (cache) resta DOMINANTE in ogni classe sul chip (0,43–0,47).
- Il minimo di heap nella suite di oggi (40 440 B) è più basso del 15/11 pomeriggio (45 216 B), con carico HTTP diverso.
- G2 (+4,7 KB) va valutato solo dopo una misura sul chip delle patch di istruzioni, con un margine di heap misurato.

## Costo per release (QEMU, migliaia di istruzioni, 46 scenari)

| Classe | T2R1M `0fb22b0` | E1 | E1+P3a | **E1+P3a+E1b+P3b `719912a`** |
|---|---|---|---|---|
| WALK media / p99 / max | 722 / 1 151 / 1 430 | 683 / 996 / 1 172 | 683 / 997 / 1 170 | **684 / 996 / 1 172** |
| WALK > 1,27 M | 73 | 0 | 0 | **0** |
| RECOVERY media / p99 / max | 727 / 1 237 / 2 156 | 685 / 1 012 / 2 162 | 685 / 1 011 / 2 164 | **685 / 1 013 / 1 562** |
| RECOVERY > 1,27 M / > 1,5 M | 87 / 6 | 8 / 3 | 8 / 3 | **5 / 1** |
| OTHER media / p99 / max | 370 / 966 / 2 045 | invariato | 369 / 966 / 1 605 | **369 / 966 / 1 376** |
| OTHER > 1,27 M / > 1,5 M | 92 / 92 | 92 / 92 | 92 / 46 | **46 / 0** |
| Stack minimo del task Motion (rig) | 7 392 B | 7 376 B | 7 376 B | 7 376 B |

RECOVERY per posizione dall'inizio della ripresa (media / max):

| Candidato | 0 | 1 | 2 | 3 | ≥ 4 |
|---|---|---|---|---|---|
| T2R1M | 1 235 / 2 156 | 607 / 676 | 1 231 / 1 808 | 773 / 1 311 | 723 / 1 440 |
| `719912a` | 1 144 / 1 562 | 607 / 677 | 999 / 1 243 | 515 / 703 | 682 / 1 119 |

**Proiezione sul chip** (modello del rapporto P0, calibrato il 15/11; **sottostima** i picchi reali di oggi):

| Classe | Basso | Alto |
|---|---|---|
| WALK max | 15,0 ms | 18,8 ms |
| RECOVERY max | 18,8 ms | 25,0 ms |
| OTHER max | 17,0 ms | 22,0 ms |

Basso = N+C / 102,5 k + 3,6 ms; alto = istruzioni × 16 µs/k.

## 5. Vincoli residui per i 16 ms

1. **Coincidenza soluzione + compensazione piena nei cammini sterzati sul chip** (27,0 ms oggi), non riprodotta dagli scenari QEMU a questa ampiezza. Serve sapere sul chip perché la release leggera non è presa (rifiuto o calendario): oggi la diagnostica non lo registra.
2. **Inizio ripresa** (posizione 0): soluzione esatta + compensazione, 1 562 k massimo dopo E1b.
3. **DEPLOY → LIFT:** 1 376 k.
4. **Perse all'attivazione da PRONE_REST:** 1 per STAND in 2 casi su 5 oggi, 1 il 15/11. Non è calcolo Motion: sta prima del comando, nel passaggio IDLE → ACQUIRE/ALIGN. Da isolare con la strumentazione esistente (`first_miss`), registrando il record anche dopo il primo.
5. Lo stallo di istruzioni (cache) resta dominante: le istruzioni in meno non si traducono 1:1 in tempo (oggi in WALK −6/−12 % di istruzioni, tempo invariato).

## 6. Regressioni e test

**Host e suite Motion** sul branch di ricerca `719912a` (`suites/result_719912a.txt`): esito nel §6.1.

`static_audit` non è stato eseguito: il gate dei sorgenti Motion richiede di ripinnare `SimpleMotion.cpp` e `ViaPlan.h` (`motion_final_release_sources.json`, con controlli negativi), e va fatto solo per un candidato da flashare.

### 6.1 Esito dei test

| Suite (sul branch `719912a`) | Esito | Durata |
|---|---|---|
| `run_host_tests.sh` | PASS | 429 s |
| `simple_motion` | PASS | 157 s |
| `stop_support` | PASS | 54 s |
| `prone` | PASS | 145 s |
| `night_recovery` | PASS | 286 s |

Log in `suites/719912a/`. Le altre 7 suite Motion rimandate falliscono già sul baseline per cause preesistenti (rapporto P0 strutturale §4) e non sono state rieseguite.

## 7. Prosecuzione del 16/11: verifiche e candidato

### 7.1 Ripinnatura e `static_audit`

- **Nessuna ripinnatura necessaria.** Il gate dei sorgenti Motion (`motion_final_release_sources.json`) pinna solo 13 file di `src/motion/`. `SimpleMotion.cpp` e `ViaPlan.h` stanno in `src/simple/` e non sono pinnati: E1, E1b, P3a e P3b non toccano file pinnati.
- `static_audit` sul branch di ricerca `719912a`: **STATIC_AUDIT = PASS**, `MOTION_CONVERGENCE_SOURCE = PASS`, `HOST_TESTS = PASS` (`audit/static_audit_719912a.txt`).
- Le righe `FAIL` nel log sono i test di mutazione che devono fallire (`USB_FRAMING_MUTATIONS = PASS, both historical defects detected`), identiche nell'audit di `46b3175`.
- `static_audit` sul candidato `82c1af9`: **PASS** (§7.5).

### 7.2 Correttezza IK e convergenza di E1/E1b (VERIFIED OFFLINE)

Copia di misura `trees/conv` (= `719912a` + shadow). A ogni soluzione piena risolve ogni gamba sia dal seme vecchio (`goal_`) sia dal seme E1/E1b, sugli stessi ingressi. La build è inerte: digest identici a `719912a` in 64/64 scenari.

| | WALK | RECOVERY |
|---|---|---|
| Soluzioni di gamba confrontate | 185 960 | 30 236 |
| Stato diverso tra i due semi | **0** | **0** |
| Converge solo con il seme nuovo / solo con il vecchio | 0 / 0 | 0 / 0 |
| Cambio di ramo cinematico (\|Δq\| > 50 mrad) | **0** | **0** |
| \|Δq\| massimo tra le due soluzioni | 0,33 mrad | 0,31 mrad |
| Residuo massimo (tolleranza 1e-6 m) | 9,999e-7 → 9,995e-7 m | 1,000e-6 → 9,999e-7 m |
| Condizionamento massimo | 7,0 → 7,0 | 6,3 → 6,3 |
| Passi di Newton | 257 811 → 181 961 (−29 %) | 39 445 → 25 634 (−35 %) |
| Valutazioni FK | 449 469 → 367 921 | 70 155 → 55 870 |
| Istogramma passi (vecchio → nuovo) | 1 passo 64 % → 98 %; 2 passi 33 % → < 0,01 %; 3 passi 3 % → 0 | analogo |

- **Fallback di E1b:** scatta 87 volte, tutte nello scenario fixture (fase HOLD, causa `SLEW`), dove anche il seme vecchio fallisce identicamente. In quelle release il fallback aggiunge una seconda soluzione (~0,4 M istruzioni) senza soluzione nominale nella stessa release: non è un picco. Non cambia mai l'esito.
- **Rispetto al punto fisso esatto del modello di bordo** (§1): E1 è più vicino nel 78 % delle gambe, con errore medio 27,4 → 4,3 µrad.

### 7.3 Regressioni contro il Motion installato (T2R1M) (VERIFIED OFFLINE, 64 scenari host)

| Verifica | T2R1M `0fb22b0` | `719912a` |
|---|---|---|
| Sequenze stato/causa diverse, esiti finali diversi | — | **0 / 0** |
| Fault | 1 (scenario di latch previsto) | lo stesso |
| STOP: durata / posa tenuta / assestamento | — | durata identica, posa ≤ 6 tick, assestamento identico (`seg_719912a_vs_t2r1m.txt`) |
| Ripresa: lunghezza / seconda differenza | — | identica / 11 → 11 tick |
| Velocità e accelerazione massime dei target | 28/31, 11/14 tick | identiche |
| IMU voluta − applicata, WALK media / p99 / max | 0,926 / 9,456 / 32,971 mrad | 0,922 / 9,441 / 32,971 |
| IMU, RECOVERY media / p99 / max | 0,494 / 3,162 / 15,450 | 0,492 / 3,124 / 15,450 |
| Limiti `WORKSPACE` WALK / altri | 10 / 287 | 10 / 288 |
| Frazione leggera T1 WALK / ripresa | 49,9 % / 49,4 % | 49,9 % / 49,4 % |
| Suite host e Motion (§6.1) e `static_audit` | PASS | PASS |

Ad anello chiuso il RAW differisce al massimo di 3 tick, salvo lo scenario a riferimento non unico `JOYSTICK_HELD_SEQUENCE` (55 tick); ad anello aperto, al massimo di 1 tick (§1).

### 7.4 Picchi residui e le 2 perse «da PRONE_REST»

**Perse all'attivazione: probabile artefatto della diagnostica** (VERIFIED OFFLINE dai log; da confermare sul chip).
- `timing::state.armed` non torna mai falso. Una persa a riposo dopo un comando resta nel conteggio di quel comando e il `arm()` della STAND successiva la cancella: per questo C1 ed E1 hanno `command_timing.missed = 0` mentre il contatore dall'avvio sale di 1.
- Ogni STAND da PRONE_REST della suite è preceduta di ~100 ms da una dump `@MOTION TIMING`. È diagnostica, accettata solo a coppia OFF, e stampa in un'unica volta nel loop.
- Durata della dump (dal log dell'host):
  - 12,6–12,9 KB in 24–36 ms dopo blocchi con WALK/RECOVERY;
  - 2,7–6,1 KB in 9–18 ms dopo il solo blocco A.
- Sul 15 e 16/11 le perse seguono **3 delle 6 dump grandi e nessuna delle 4 piccole**.
- Un blocco di B ms fa perdere una release con probabilità ~(B − 20)/20 secondo la fase: ~30 % a 26 ms, ~80 % a 36 ms. È coerente: 36 ms → persa, 28 → persa, 26/26/30 → 1 su 3.
- Il passaggio sincrono del registro 85 (`configureAccCeiling`, 12 letture) è visibile ~10 ms dopo ogni STAND in tutti e 5 i casi: da solo non spiega perché solo C1 ed E1.
- **D3** (nel candidato, §7.5) registra l'ultima persa a Motion non attivo e la durata della dump: la prossima sessione può confermare o smentire.
- In una build operativa la dump non esiste.

**Picchi di calcolo residui** (§5, invariati): il caso sterzato con soluzione + compensazione piena visto sul chip (27,0 ms), l'inizio della ripresa (1 562 k), DEPLOY → LIFT (1 376 k).

**Nessuna nuova ottimizzazione aggiunta:** la necessità di ulteriori patch va dimostrata sul chip con il candidato.

### 7.5 Candidato minimo per il prossimo collaudo hardware (IMPLEMENTED NOT VALIDATED, non flashato)

| Voce | Valore |
|---|---|
| Branch | `cand/diag-p0-peaks-20261116` (locale), worktree `~/MATDOG/worktrees/matdog-diag-p0-peaks-20261116` |
| Commit | **`82c1af9`** = `46b3175` (installato) + E1 `ae9b6d1` + P3a `db6d621` + E1b `447082c` + P3b `e6ee46b` (cherry-pick, diff identici a `8346b48..719912a`) + D3 `82c1af9` (solo diagnostica: ultima persa a Motion non attivo, durata della dump, durata del passaggio del registro 85; commento di composizione in `BuildConfig.h`) |
| Build | CLEAN `82c1af95e0a5`, ROBOT_POWERED, OTA ingest 0. APP `93c61562fdcf1d79206565a9dc440debd2bc8dbd7e2fa91176d7aefd2f966ace`, 1 783 744 B; tabella `8f756ecb…`, bootloader e sdkconfig identici a `46b3175` |
| Sezioni contro `46b3175` | IRAM identica; `.dram0.data` +32 B (record D3); `.dram0.bss` invariata; `.flash.text` +1 088 B; `.flash.rodata` +212 B (`audit/sections_46b3175_vs_82c1af9.txt`) |
| `static_audit` | **PASS** (`audit/static_audit_diag_82c1af9.txt`) |
| Suite Motion | `simple_motion` PASS 144 s, `stop_support` PASS 49 s, `prone` PASS 141 s, `night_recovery` PASS 280 s (`audit/motion_suites_82c1af9.txt`, log `audit/suite_82c1af9_*.log`) |
| Pre-flash (preparato, **non autorizzato**) | `candidate_82c1af9/preflash/`: script derivato da quello di `46b3175` cambiando solo i valori pinnati, autorizzazione `MATDOG_DIAG82C_FLASH_AUTHORIZED`; dry gates PASS; controlli negativi SHA, dimensione, commit, NVS → REFUSE; rifiuto senza autorizzazione; `image-info` valido |
| Rollback | `46b3175` con il suo script pinnato; dry gates ripetuti oggi: PASS |
| Strumenti della sessione | quelli del 16/11: A–G con `EXPECT_BUILD='82c1af95e0a5'` e blocco H. `analyze_diag.py` ignora le righe D3 nuove; `fixture_suite.py` attende sempre `MOTION_TIMING hist`, e la riga `dump_us` arriva dopo |
| Criteri | P0 chiuso solo con 0 perse operative, 0 overrun, massimo ≤ 16 ms (invariato). Da misurare: picco WALK sterzato, inizio ripresa, DEPLOY → LIFT, natura delle perse a riposo (D3) |

## 8. Prossima prova fisica strettamente necessaria (P0)

Una sola sessione fixture, con un nuovo mandato e l'operatore presente:
- un solo flash APP-only del candidato `82c1af9` (§7.5), già costruito, auditato e con il pre-flash pronto;
- stessa suite A–G + blocco H, stessi criteri;
- obiettivi:
  - misurare sul chip il picco WALK sterzato, l'inizio della ripresa e DEPLOY → LIFT;
  - confermare con D3 che le perse a riposo vengono dalla dump diagnostica.

Rollback: `46b3175`.

## 9. Dopo la validazione sul chip di `82c1af9`: P4

### 9.1 Effetto misurato di E1/E1b/P3a/P3b (VERIFIED HARDWARE, stessi blocchi del 16/11)

- **Overrun** 161 → 31 (suite + H):
  - STAND/PRONE/HEIGHT 78 → 18;
  - cammini sterzati con rilascio e WALK_SETTLE 73 → 12;
  - riprese 9 → 0;
  - altro 1 → 1 (`../MATDOG_1_0_2_DIAG_82C1AF9_HW_20261116/overrun_causes_vs_46b3175.txt`).
- **Per release:** WALK −10/−12 %, RECOVERY −10/−11 %. Istruzioni WALK −5/−8 % (E1: meno Newton e FK), IPC +0,02/+0,05 e `istall_mem` −0,03/−0,11.
- **Effetto di layout:** le release OTHER, con istruzioni invariate, sono più veloci del 7–9 %. Una parte del guadagno viene dal layout del codice nella cache, non dalle patch.
- **Picchi:** sterzati 25,7–27,0 → 18,8–20,9 ms; WALK 23,7 ms (E8 WALK STOP); RECOVERY 19,8 ms (la soluzione esatta all'inizio della ripresa vale da sola 13,1 ms); OTHER 26,8 ms (WALK_SETTLE).
- **Perse:** 6 dell'attesa ACC, 0 operative.

### 9.2 Prima causa residua misurata: compensazione attenuata a 2 soluzioni

- **Sul chip:** 18 overrun su 31 sono release di STAND/PRONE/HEIGHT con `max_work_solves = 2`, 13,5–16 ms di compensazione e 20–22 ms per release (`residual_by_step.txt`).
- **Meccanismo:**
  - nel ramo non WALK di `compensate` la sonda alla scala accettata (`correctionScale_`) basta solo se la proiezione resta entro ±2 %;
  - altrimenti si risolve una seconda volta alla proiezione.
  - Durante l'innesto C2 di 1 s e mentre il segmento si avvicina ai limiti, la proiezione si sposta a ogni release.
- **Offline:** 5 376 release a 2 soluzioni sull'host, 3 917 in QEMU.

### 9.3 P4 (`e57dd89` sul branch di ricerca; candidato `2638204`)

- **Patch:** la sonda è tenuta quando la sua scala **non supera** la proiezione; la proiezione diventa la scala della sonda della release successiva.
  - Un'attenuazione che cresce segue con una release di ritardo, sempre con una correzione non più grande di quanto l'attenuazione consente.
  - Una proiezione che si restringe risolve subito come prima.
  - Nessuno stato nuovo; generalizza P3b.
- **Verifiche offline:**

| Verifica | Esito |
|---|---|
| Host, 64 scenari, contro `719912a` | stesse sequenze stato/causa ed esiti; RAW diverso in 1 680 release, al massimo di 1 tick; STOP (durata, posa tenuta) e ripresa identici; statistiche IMU invariate (`cmp_p4_vs_719912a.txt`, `seg_p4_vs_719912a.txt`, `imu_track_p4_vs_719912a.txt`) |
| Release OTHER a 2 soluzioni | host 5 376 → 904; QEMU 3 917 → 697 |
| Costo QEMU OTHER | media 369 → 338 k, p99 966 → 852 k, massimo invariato (1 376 k, la release DEPLOY → LIFT); WALK e RECOVERY invariati (`qemu_cost_p4.txt`) |
| QEMU = host | 46/46 digest |
| Stack Motion (rig) | 7 376 → 7 392 B |
| `static_audit` su `e57dd89` | PASS |
| Suite Motion su `e57dd89` | `simple_motion`, `stop_support`, `prone`, `night_recovery` PASS |
| Candidato `2638204` | build CLEAN `2638204fb4ef`, APP `79018a28…c6f4`; sezioni IRAM/DRAM identiche a `82c1af9`; `static_audit` e suite: §9.4 |

- **Costo di qualità:** nelle fasi attenuate la correzione applicata può arrivare con una release di ritardo, quando cresce. Sull'host la differenza della correzione applicata è ≤ 0,04 mrad.

### 9.4 Candidato `2638204`

| Voce | Valore |
|---|---|
| Branch | `cand/diag-p0-p4-20261116` (locale) = `82c1af9` + P4 `785bcec` (cherry-pick identico a `e57dd89`) + commento di composizione `2638204` |
| Build | CLEAN `2638204fb4ef`, APP `79018a28eb7876b7fbc768b8558d9d8bb31788b31f9d961739633054be39c6f4`, 1 783 680 B; IRAM e DRAM identiche a `82c1af9`, `.flash.text` −60 B |
| `static_audit` | **PASS** |
| Suite Motion | `simple_motion` PASS 139 s, `stop_support` PASS 47 s, `prone` PASS 137 s, `night_recovery` PASS 275 s |
| Pre-flash | `candidate_2638204/` (`NEXT_SESSION_PLAN.md`, `preflash/`, `tools/`, `image/`): script derivato solo nei valori pinnati; dry gates PASS; 4 controlli negativi REFUSE; rifiuto senza autorizzazione; `image-info` valido |
| Rollback | `82c1af9` (installato, script che l'ha installato, dry gates PASS) |
| Stato | **non flashato, non autorizzato** |

### 9.5 Prossima patch realmente necessaria

1. **P4**, già sviluppata: è l'unica giustificata da una causa misurata e completamente spiegata.
2. **WALK_SETTLE dopo rilasci sterzati:** nessuna patch prima di misurare dove va il tempo.
   - Il record `MOTION_PMU_HEAVIEST` di un'altra sessione (gruppi PMU a rotazione) può dare le istruzioni di quel tipo di release.
   - Se sono ~0,7 M come in QEMU, la causa sono gli stalli; se sono molte di più, è un percorso di codice che il rig non riproduce (ipotesi: prova STOP o posa caricata con correzioni grandi).
3. **Attesa ACC alla prima attivazione** (6 perse a ogni avvio):
   - nota dal 15/11, di 100 ms sincroni, nel loop;
   - va valutata a parte con l'operatore, perché tocca la verifica del registro 85: è una protezione e non si cambia senza mandato.

## File

- **Rig e strumenti:** `rig/`, `tools/`, `run.sh`, `run_e1sh.sh`, `run_p0only.sh`.
- **Uscite:** `out/<albero>/host.txt`, `target.txt`.
- **Confronti:** `cmp_*.txt`, `qemu_cost_summary.txt`.
- **P3a:** `p3a/` (test di equivalenza).
- **Locomozione:** `loco/` (`LOCO_EVAL.md`, `GROUND_MEASUREMENT_PLAN.md`, `workspace_use.cpp`, `workspace_use_e1b.txt`).
- **Test:** `suites/` (branch di ricerca); `audit/` (`static_audit` di `719912a` e `82c1af9`, suite Motion e sezioni del candidato).
- **Convergenza e regressioni:**
  - `trees/conv` + `run_conv.sh`, uscite in `out/conv/host.txt`;
  - `seg_719912a_vs_t2r1m.txt`, `cmp_719912a_vs_t2r1m.txt`, `imu_track_719912a_vs_t2r1m.txt`.
- **Candidati:** `candidate_82c1af9/` (installato il 16/11) e `candidate_2638204/` (P4, non flashato): `NEXT_SESSION_PLAN.md`, `preflash/`, `tools/`, `TOOLS.sha256`, `image/`.
- **WALK_SETTLE (tentativo di riproduzione):** `rig/settle_scenarios.h`, `run_settle.sh`, `out/settle_*`.
