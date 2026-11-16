> Copia del rapporto offline. I percorsi relativi (`rig/`, `tools/`, `out/`, `trees/`, `suites/`, `audit/`, `p3a/`, `loco/`, `candidate_82c1af9/`) si riferiscono alla cartella delle evidenze `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/` (sigillata, `SHA256SUMS`), non a questo repository. Il codice è sui branch locali `research/p0-peaks-20261116` e `cand/diag-p0-peaks-20261116` (bundle `backups/git/robot-dog-p0-peaks-20261116.bundle`).

# Motion Timing P0 — picchi residui dopo T2R1M, sviluppo offline (16/11/2026)

Lavoro offline svolto durante e dopo la validazione hardware di `46b3175` (`../MATDOG_1_0_2_DIAG_46B3175_HW_20261116/REPORT.md`). Nessun flash di questi candidati.

Le istruzioni Xtensa contate in QEMU (`-icount 3`, ×3,088) **non sono millisecondi**. P0 si chiude solo sul chip: **non è chiuso**.

## Sintesi per stato (fine giornata 16/11)

**1. Risultati hardware verificati** (`46b3175`, fixture, 16/11):
- Suite A–G 65/65 + blocco H (STOP durante la ripresa) 19/19; STOP, calibrazioni e NVS invariati.
- RECOVERY 15,1–16,4 → 10,9–12,0 ms per release (istruzioni −40/−45 %); WALK invariato in tempo.
- Picco 36,1 → 27,0 ms; overrun 323 → 157; 0 perse in WALK/RECOVERY (15/11: 3).
- Costo IMU di T2R1M: errore sterzato medio +16 %.
- DALY: tre avvii senza silenzio (freddo `e321e86`, boot dopo il flash, freddo B8 `46b3175`: 601/601 poll ciascuno a freddo); D2 attiva con baseline registrata.

**2. Miglioramenti dimostrati soltanto offline** (non flashati):
- **E1/E1b:** seme dell'IK di compensazione.
  - QEMU: WALK max 1 430 → 1 172 k; release WALK > 1,27 M 73 → 0; RECOVERY max 2 156 → 1 562 k.
  - Stato IK identico su 216 196 soluzioni, nessun cambio di ramo, Newton −29/−35 %.
- **P3a:** `ViaPlan::plan` identico bit per bit, ~440 k in meno nella release del piano.
- **P3b:** release del piano con una sola soluzione. Con P3a, OTHER max 2 045 → 1 376 k.
- **Nessuna regressione su 64 scenari:** stesse sequenze; STOP, IMU, WORKSPACE e traiettorie entro i valori di T2R1M; `static_audit` e quattro suite Motion PASS.
- **Candidato:** `82c1af9`, costruito e auditato, pre-flash pronto e non autorizzato (§7.5).

**3. Cause ancora ipotetiche:**
- **Perse «all'attivazione da PRONE_REST»:** probabile artefatto della dump diagnostica `@MOTION TIMING` a Motion non attivo (§7.4). Il legame è forte nei log ma non misurato sul chip; D3 lo decide.
- **Picco WALK sterzato di 27,0 ms** (soluzione + compensazione piena nella stessa release): sul chip non si sa se la release leggera manca per un rifiuto di `compensationSafe` o per il calendario T2.
- **Silenzi DALY:** causa UNKNOWN.

**4. Problemi residui per chiudere P0** (0 perse operative, 0 overrun, ≤ 16 ms):
- stallo di istruzioni dominante;
- inizio della ripresa 1 562 k;
- DEPLOY → LIFT 1 376 k;
- release sterzate con doppia soluzione sul chip;
- ~3,6 ms per release fuori da N+C;
- conferma D3 sulle perse a riposo.

Il modello QEMU sottostima i picchi del chip: ogni guadagno va misurato con `82c1af9` (`candidate_82c1af9/NEXT_SESSION_PLAN.md`).

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
- **Candidato:** `candidate_82c1af9/` (`NEXT_SESSION_PLAN.md`, `preflash/`, `tools/`, `TOOLS.sha256`, `d3_selftest/`).
