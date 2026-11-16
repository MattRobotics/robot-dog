> Copia del rapporto offline. I percorsi relativi (`rig/`, `tools/`, `out/`, `trees/`, `suites/`, `p3a/`, `loco/`) si riferiscono alla cartella delle evidenze `verification-artifacts/MATDOG_1_0_2_P0_PEAKS_20261116/` (sigillata, `SHA256SUMS`), non a questo repository. Il codice dei candidati è sul branch locale `research/p0-peaks-20261116` (bundle `backups/git/robot-dog-p0-peaks-20261116.bundle`).

# Motion Timing P0 — picchi residui dopo T2R1M, sviluppo offline (16/11/2026)

Lavoro offline svolto durante e dopo la validazione hardware di `46b3175` (`../MATDOG_1_0_2_DIAG_46B3175_HW_20261116/REPORT.md`). Nessun flash di questi candidati.

Le istruzioni Xtensa contate in QEMU (`-icount 3`, ×3,088) **non sono millisecondi**. P0 si chiude solo sul chip: **non è chiuso**.

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

## 7. Prossima prova fisica strettamente necessaria (P0)

Una sola sessione fixture, se l'operatore la decide, con una diagnostica = `46b3175` + E1 + E1b + P3a + P3b:
- stessa suite A–G + blocco H;
- stessi criteri;
- obiettivo: misurare sul chip il picco WALK sterzato, l'inizio della ripresa e DEPLOY → LIFT.

Richiede un nuovo mandato, una build CLEAN, `static_audit` con il nuovo pin e le suite Motion.

## File

- **Rig e strumenti:** `rig/`, `tools/`, `run.sh`, `run_e1sh.sh`, `run_p0only.sh`.
- **Uscite:** `out/<albero>/host.txt`, `target.txt`.
- **Confronti:** `cmp_*.txt`, `qemu_cost_summary.txt`.
- **P3a:** `p3a/` (test di equivalenza).
- **Locomozione:** `loco/` (`LOCO_EVAL.md`, `workspace_use.cpp`, `workspace_use_e1b.txt`).
- **Test:** `suites/`.
