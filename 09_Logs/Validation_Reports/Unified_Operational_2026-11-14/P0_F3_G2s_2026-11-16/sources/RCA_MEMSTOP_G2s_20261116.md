# MEMORY_STOP di `cb9ed85`: diagnosi e recupero (offline, 16/11/2026 sera)

Mandato «Recupero dopo validazione G2 interrotta». Robot spento; nessun flash, nessuna prova hardware, firmware operativo non toccato. Sessione analizzata: `../MATDOG_1_0_2_P0F3G2_HW_20261116/`.

## Diagnosi breve

1. **Costo statico di G2, dimostrato: 10 240 B di heap interno.**
   - Dagli ELF: `.iram0.text` passa da 87 215 a 97 515 B (+10 300); `_heap_start` si sposta di 0x2800, cioè 10 240 B; `.dram0.data` e `.dram0.bss` sono identiche.
   - Sul robot, a parità di uptime: −10 204 B di `heap_free`.
2. **Il calo a 9,7 KB è un transitorio dell'heap interno dovuto al traffico HTTP, non a Motion.**
   - È avvenuto fra 18:24:42.44 e 42.53Z, all'avvio della prova, quasi un secondo prima del primo comando Motion (43,4 s).
   - In quel momento tre client chiedevano dati insieme. `heap_free` corrente è sceso a 43 KB ed è risalito a ≈ 60 KB.
   - Motion e il codice spostato da G2 non c'entrano col transitorio: G2 conta solo per il suo costo statico.
3. **Il terzo watcher spiega il calo in modo coerente, ma non dimostrato: causa formalmente UNKNOWN**, ipotesi principale H1 (§2).
4. **Margine:**
   - **G2 (G2c, 10 KB):** non ha margine adeguato sulla build diagnostica di misura. Con il peggior transitorio a 2 client già osservato (36,2 KB) scenderebbe a 23,9 KB; con dashboard e watcher a 18,7 KB.
   - **G2s, la variante preparata (6 KB):** conserva +4 KB nel caso peggiore a 2 client.
   - Nessuna regressione funzionale è stata osservata o dimostrata per G2.
5. **Strumenti corretti e provati offline:**
   - un solo proprietario per acquisizione, con PID reale e chiusura verificata;
   - controllo di memoria M0 prima di ogni comando Motion;
   - STOP e protezioni invariati.
6. **Controllo hardware minimo discriminante:** M0 sul candidato G2 con esattamente 2 client, prima di ogni Motion (§5).

## 1. Profili di memoria `a86c680` / `cb9ed85`

Heap interno (`MALLOC_CAP_INTERNAL`): le allocazioni oltre 4 KB vanno in PSRAM, quindi anche il buffer da 20 KB delle capabilities. `heap_min_free` è il minimo storico dal boot.

| Voce | `a86c680` | `cb9ed85` | Fonte |
|---|---|---|---|
| Statico: `.iram0.text` | 87 215 B | 97 515 B (+10 300) | ELF |
| Statico: `_heap_start` | 0x3fcbe148 | 0x3fcc0948 (+10 240) | ELF |
| `heap_free` a ≈ 22–25 s di uptime | 71 860 | 61 656 (−10 204) | USB, sessione del 16/11 |
| `heap_free` a regime (mediana) | 70 300–70 500 | 60 100–60 500 | HTTP |
| Minimo storico, 1 client (watcher) a riposo | 50 264 | 39 424 | HTTP |
| Minimo storico, 2 client (prove) | 41 888 | — | HTTP |
| Minimo storico, 3 client (avvio della prova) | — | **11 144 → 9 748** | HTTP |

**Transitorio** = `heap_free` a regime − minimo storico:

| Client contemporanei | Transitorio | Dati |
|---|---|---|
| 1 (watcher + USB) | 20,3 / 21,1 KB | `a86c680` / `cb9ed85` |
| 2 (strumento + watcher) | **26,8–28,5 KB**; una volta 36,2 KB | 11 prove su 6 build (82c1af9, 2638204, 46b3175, a86c680 ×2…); 36,2 KB in `8530d3a` il 15/11, non spiegato |
| Dashboard dell'operatore + watcher | 41,4 KB | dev.6, 10/11 (`450d2c7`) |
| 3 (strumento + 2 watcher) | **50,4 KB** | solo `cb9ed85`, 16/11 |

- **Come cresce il minimo:** con 2 client cala a gradini di 2–6 KB nell'arco di minuti. Con 3 client è sceso di 9,7 + 18,5 KB in 100 ms (`out/min_drops_*.txt`, `out/heap_survey_all.txt`).
- **`heap_min_free`** è la somma dei minimi delle singole regioni, presi in istanti diversi: è pessimistico. `heap_largest` resta 31 732 B anche a 9,7 KB di minimo, quindi non misura la frammentazione.
- **Da dove viene il transitorio:** è memoria di lwIP e Wi-Fi in RAM interna.
  - Ogni richiesta apre una connessione nuova (urllib).
  - La risposta di `/api/motion/status` (4 865 B) occupa i pbuf del buffer TCP di invio (`TCP_SND_BUF` 5 744) finché il PC non conferma la ricezione; `/status` ne occupa 2,1 KB.
  - Le richieste in attesa trattengono buffer RX dinamici del Wi-Fi da ≈ 1,6 KB ciascuno, più pcb e socket.
  - Con `max_open_sockets` 7 il caso limite vale ≈ 50–60 KB. È una stima: qualunque build da 70 KB a regime potrebbe scendere sotto 24 KB con 7 risposte contemporanee.

## 2. Il terzo watcher spiega il calo a 9,7 KB?

- **A favore (dai log):**
  - il calo coincide al millisecondo con l'avvio contemporaneo dei tre client: watcher rimasto attivo 42,439, watcher del run 42,442, poller e letture iniziali della suite nello stesso istante;
  - nessun comando Motion era ancora partito;
  - `heap_free` istantaneo a 43 168 B prova che c'erano ≈ 17 KB di risposte in volo;
  - in 6 build e 11 prove a 2 client il minimo non è mai sceso oltre 36,2 KB sotto il regime.
- **Il conto:** il minimo all'avvio della prova vale 41 888 B su `a86c680` e 9 748 B su `cb9ed85`, cioè 32 140 B di differenza.
  - 10 240 B sono il costo statico di G2, dimostrato.
  - Restano ≈ 21,9 KB di transitorio in più, che corrispondono a 2–3 risposte in volo in più (≈ 6–8 KB ciascuna).
- **Perché non è dimostrato:**
  - non esiste alcuna misura con 3 client su un'altra build;
  - il poller della suite non registra i propri istanti di richiesta, quindi la sovrapposizione esatta non si può ricostruire;
  - l'ingombro per risposta è stimato, non misurato;
  - con 2 client il transitorio varia da 26,8 a 36,2 KB, quindi un'estrapolazione lineare dà 35–45 KB, non 50.
- **Esito: causa UNKNOWN**, ipotesi principale H1 (terzo client sopra il costo statico di G2).
  - Un effetto dinamico proprio di G2 non ha un meccanismo noto (G2 sposta solo funzioni Motion, e Motion non era attivo), ma non si può escludere senza la misura del §5.

## 3. G2 e la dashboard

Minimo previsto = `heap_free` a regime − transitorio osservato; soglia 24 KB:

| Uso | Transitorio | `a86c680` (70,3) | G2s (64,2) | G2c (60,1) |
|---|---|---|---|---|
| 1 client | 21 KB | 49 | 43 | 39,4 (misurato) |
| 2 client, tipico | 26,8–28,5 KB | 41,9 (misurato) | 35,7–37,4 | 31,6–33,3 |
| 2 client, peggiore visto | 36,2 KB | 34,1 | **28,0** | **23,9** ✗ |
| Dashboard + watcher (dev.6) | 41,4 KB | 28,9 | 22,8 ✗ | 18,7 ✗ |
| 3 client | 50,4 KB | 19,9 ✗ | 13,8 ✗ | 9,7 (misurato) ✗ |

- **Sulla build diagnostica di misura:** G2c **non** ha margine adeguato (sotto soglia già nel peggior caso a 2 client); G2s sì, con +4 KB, a patto di 2 client al massimo e nessuna dashboard aperta durante le prove.
- **Dashboard da sola sulla dev.8:** mai misurata; sta fra 21 e 41 KB.
- **Sul firmware operativo**, che aveva ≈ 84,7 KB a regime con dev.7 (il branch unificato non è misurato):
  - G2c resterebbe ≈ 33 KB con dashboard + watcher;
  - G2s ≈ 37 KB.
- Con 3 o più client contemporanei nessuna build, `a86c680` compreso, ha margine garantito. È un limite del trasporto HTTP che G2 riduce di 10 KB, non un difetto di G2.

**Variante G2s (preparata):** G2 senza `contactInverseKinematicsGroundNormal` (3 231 B) e `validGaitParameters` (891 B).

| | G2c (`cb9ed85`) | **G2s** (`6be4690`) |
|---|---|---|
| Funzioni in IRAM | 19 | 17 (sottoinsieme stretto) |
| Testo del rig | 9 966 B | 5 844 B |
| `.iram0.text` / costo heap (ELF) | +10 300 / 10 240 B | **+6 148 / 6 144 B** |
| Miss medi K1 WALK / K2 compensazione / K3 WALK STOP / K4 STAND-HEIGHT / K5 RECOVERY | −34/−34/−36/−71/−39 % | −26/−27/−28/−45/−28 % |

- **Scelta del set:** una selezione golosa a 6 KB fra le sole funzioni collocabili (escluse le inline COMDAT) dà quasi lo stesso risultato (−26…−29 %, K4 −47 %; `out/eval3.txt`). G2s è preferibile perché non introduce nessuna funzione nuova in IRAM.
- **Sorgenti:** privi dell'attributo, sono identici ad `a86c680`; cambiano solo due pin del controllo dei sorgenti.
- **Non verificato:** il QEMU di G2s non è stato ripetuto (identità dei frame per costruzione, come G2c verificato).

## 4. Strumenti (`hw_next/tools`, `TOOLS.sha256`)

- **`session_proc.py` (nuovo), il registro dei processi:**
  - avvio con `Popen` e PID reale verificato su `/proc`;
  - una sola istanza per ruolo (USB, watch, tool, guard);
  - rifiuto se esiste un processo di sessione non registrato o se la porta USB è occupata;
  - chiusura solo per PID registrato (USB con `QUIT`) e **verificata**;
  - la scansione di `/proc` elenca soltanto, non uccide mai per pattern.
- **`run_acquisition.py` (nuovo, sostituisce `run_with_watch.sh`):**
  - controlla la sessione pulita e `heap_min_free` ≥ 24 KB prima di partire;
  - avvia strumento, unico watcher e guardia;
  - attende la fine, chiude con verifica e stampa `ACQUISITION_CLOSED`.
- **`memgate.py` (nuovo), passo M0 in `fixture_settle.py`, `fixture_blocks.py` e `fixture_block_h.py`:**
  - prima di ogni comando Motion attende 20 s con i due client attivi;
  - richiede `heap_min_free` ≥ 24 KB su ogni campione e nessuna allocazione fallita;
  - sotto soglia si ferma subito senza STOP né Motion.
- **Altri adattamenti:**
  - `preflight.py`: aggiunta la voce di memoria (9/9);
  - `cold_start.sh` e `boot_and_record.sh`: passano dal registro;
  - `fixture_suite.py` pinnato su `6be46906d069`, più le copie `fixture_suite_cb9ed85.py` e `fixture_suite_a86c680.py`;
  - strumenti ROM con la variabile `MATDOG_P0F3G2S_FLASH_AUTHORIZED`.
- **Invariati:** `fixture_suite*.py` (sequenze, controlli, limiti, `safe_end` con STOP), `heap_guard.py`, `http_watch.py`, `usb_session.py`.
- **Prove offline** sul simulatore (`tests/RESULTS.md`), tutte superate:
  - T1 blocco H completo, 20/20 PASS;
  - T2 M0 si ferma senza nessun comando Motion;
  - T3 rifiuto con il watcher non registrato (il caso del 16/11);
  - T4 la guardia durante Motion e lo STOP inviato;
  - T5 cold con rifiuto fino alla chiusura;
  - T6 registratore di boot e chiusure verificate.

## 5. Controllo hardware minimo per qualificare G2

Dettagli: `hw_next/NEXT_SESSION_PLAN.md`. Richiede un nuovo mandato e un solo flash.

- **Il controllo discriminante:** con il candidato installato, prima di ogni Motion, il passo M0 con esattamente 2 client.
  - **Previsto con H1:** G2s 28–37 KB, G2c 24–33 KB.
  - **Sopra 24 KB:** G2 è qualificabile; nella stessa sessione si misurano il beneficio IRAM (contro il riferimento `a86c680` della stessa sessione) e F3.
  - **Sotto 24 KB con 2 client:** H1 non basta; stop senza Motion e rollback ad `a86c680`.
- **Candidato consigliato: `6be4690` (F3 + G2s).** Ha margine positivo in tutti i casi a 2 client osservati e dà la stessa calibrazione fra miss e ms. Il transitorio misurato nella sessione dirà poi se G2c è accettabile.
- **Facoltativo, solo per chiudere la causa del 16/11:** 2 minuti di un terzo client di sola lettura su `a86c680` a riposo, prima del flash. H1 prevede ≈ 20 KB.

## 6. Candidati recuperabili (nessuno pubblicato, nessuno flashato ora)

| Commit | Branch | Contenuto | Immagine |
|---|---|---|---|
| `6146ef1` | `cand/f3-fast74-20261116` | F3 | APP `afe0747c…` |
| `dfc1156` | `cand/iram-g2-20261116` | G2 + ri-pin | `3fd87581a5cf` (prima del ri-pin) |
| `cb9ed85` | `cand/p0-f3-g2-20261116` | F3 + G2 | APP `254eca05…` (MEMSTOP 16/11) |
| `b5e6691` | `cand/iram-g2s-20261116` | G2s (su `dfc1156`) | APP `ab99ea1d…`, 1 784 368 B, CLEAN |
| **`6be4690`** | `cand/p0-f3-g2s-20261116` | F3 + G2s (su `cb9ed85`) | APP `be678d83…`, 1 784 368 B, CLEAN; pacchetto `g2s/preflash` (script `6f369ae1…`, variante DRY `730d7b2a…`, dry gates PASS, rifiuto senza autorizzazione; rollback `c77428e1…`) |

Audit statico di `6be4690` e `b5e6691`: `STATIC_AUDIT = PASS`, `MOTION_CONVERGENCE_SOURCE = PASS`, nessuna voce FAIL (`g2s/static_audit_*.txt`). Log di build in `g2s/`.
