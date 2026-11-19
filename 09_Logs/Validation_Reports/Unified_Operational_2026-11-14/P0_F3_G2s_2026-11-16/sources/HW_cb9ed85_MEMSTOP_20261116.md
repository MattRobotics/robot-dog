# Validazione hardware F3 + IRAM G2 — rapporto (16/11/2026 sera)

Mandato «Validazione hardware F3 + IRAM G2». Piano: `README.md`; diario con orari: `WORKLOG.md`.

**Esito: validazione INTERROTTA dalla soglia di memoria e rollback ad `a86c680` eseguito e verificato.**
- Sul candidato `cb9ed85` `heap_min_free` è sceso a **11 144 B** alle 18:24:42.5Z (poi 9 748 B), all'avvio della prima prova e prima del primo comando Motion.
- La guardia ha fermato la prova, il robot è andato in FAULT con coppie OFF, poi rollback come da piano; Motion non è ripreso.
- Un errore di procedura mio (un watcher HTTP rimasto attivo, quindi tre client contemporanei) è la causa più probabile del calo, ma non è dimostrata.
- Né IRAM G2 né F3 sono stati misurati in movimento.

## 1. Risposte ai punti del mandato

| Voce | Esito |
|---|---|
| Beneficio reale IRAM G2 in ms | **NON MISURATO.** Le prove del candidato si sono fermate in S1. Disponibile solo il lavoro a riposo, non confrontabile: massimo 4,49 ms per release contro 4,38 ms su `a86c680`, 0 perse e 0 overrun in entrambi. |
| FAST corretto e margine | **NON PROVATO su hardware** (blocchi E–F e H non eseguiti). Il margine resta quello offline: secante 29 tick contro il limite 31,36 (+1,36 tick contro +1 richiesto), `../MATDOG_1_0_2_F3FIX_IRAM2_20261116/REPORT.md`. |
| Heap minimo e blocco massimo | `cb9ed85`: a riposo 39 424 B; all'avvio della suite con 3 client HTTP **11 144 → 9 748 B**; `heap_free` corrente minima 43 168 B. `heap_largest` 31 732 B costante (vedi §3). Nessuna riga di allocazione fallita. |
| Regressioni | Nessuna regressione funzionale osservata. Identità, NVS, restore 12/12, census, IMU, BMS e preflight 8/8 passano su `cb9ed85`. Regressione di **margine di memoria**: −10,2 KB a riposo, come previsto offline (−10 240 B). |
| P0 | **APERTO.** Percorsi residui misurati stasera su `a86c680`: §4. Sul candidato nessuna misura. |
| Stato finale | `a86c6800b543` installato (rollback verificato); READY/IDLE `PRONE_ENTRY`; coppie 0 ×12; SAFE_OFF 8191; nessun fault; BMS OK 11,2 V, SOC 46,9 %; stream BMS OFF; nessun registratore attivo. **SAFE_TO_POWER_OFF.** |

## 2. Sequenza

1. **QON 18:02:43.549Z**, registratore dal boot.
   - Silenzio DALY dal primo poll per 182 s; rientro spontaneo a 187 s di uptime (§5).
   - Gate PASS alle 18:11:02Z; firma, NVS e preflight `a86c680` 8/8.
2. **Riferimento `a86c680`, nessun FAST:**
   - prova mirata D8–D11 `ref_settle_20261116_1813`: **11/11 PASS**;
   - blocchi A e B `ref_ab_20261116_1814`: **17/17 PASS**.
3. **ROM e flash unico `cb9ed85`:**
   - `rom_checks.sh` PASS (layout, digest `a86c680`);
   - flash APP-only PASS con verify-flash;
   - boot pulito (18:18:18Z);
   - gate BMS 175/175 dal boot, uptime > 6 min;
   - preflight `cb9ed85` 8/8.
4. **`cand_settle_20261116_1824`:** S0 PASS (dump TIMING).
   - S1 STAND 120 fermato dalla guardia di memoria alle 18:24:44Z;
   - lo STOP di `safe_end` durante l'acquisizione ha dato FAULT `STOP_REFERENCE_UNAVAILABLE`, coppie 0 ×12, SAFE_OFF 8191.
5. **Rollback** (regola di memoria del mandato):
   - ROM: layout invariato, `cb9ed85` confermato a 0x010000;
   - script di rollback `c77428e1…` (quello che aveva installato `a86c680`): PASS con verify-flash;
   - boot `a86c6800b543` pulito, restore 12/12, NVS `LOAD_COUNT=4` gen 1.
6. **Chiusura:** `final_state_readonly.json`, `@BMS STREAM OFF`, registratore chiuso con QUIT e PID.

Flash eseguiti: due, cioè il flash autorizzato più il rollback giustificato dal criterio di memoria. Nessun altro.

## 3. Memoria

| Misura | `a86c680` (stasera) | `cb9ed85` |
|---|---|---|
| `heap_free` / `heap_min_free` a ≈ 22–25 s di uptime (un client USB) | 71 860 / 63 692 (dopo il rollback) | 61 656 / 53 768 |
| Costo statico | — | **−10 204 B / −9 924 B** (offline: −10 240 B) |
| `heap_min_free` a riposo, un client HTTP | 50 264 (cold) | 39 424 (`usb_boot`) |
| `heap_min_free` durante le prove, 2 client HTTP (suite + watcher) | 41 888 (settle e A/B) | — |
| `heap_min_free` all'avvio della suite, **3 client HTTP** | non misurato | **11 144 → 9 748** |
| `heap_largest` | 31 732 costante | 31 732 costante |

- **Calo del candidato.** Fra 18:24:41.3 e 18:24:43.3 `heap_min_free` passa da 39 424 a 29 680 e poi a 11 144 B; `heap_free` corrente scende a 43 168 B a 18:24:42.442 e torna a ≈ 60 100 B. Coincide con l'avvio del poller HTTP della suite e del watcher del run, prima del `@MOTION TIMING` (18:24:43.320) e dello STAND (18:24:43.415). Dopo lo STOP: 10 728, poi 9 748 B.
- **Errore di procedura (mio).** Il watcher `http_watch.py` del registratore `usb_boot` non era stato chiuso.
  - Il PID salvato (288117) era il processo `setsid`, che si era biforcato; il watcher reale era 288118.
  - Così, all'avvio della suite, tre client leggevano `/api/motion/status` insieme (risposta da alcuni KB); nelle prove di riferimento erano due.
  - Chiuso per PID esplicito alle 18:26Z. Gli altri watcher della sessione erano chiusi correttamente: i loro ultimi campioni coincidono con la fine dei run.
  - Il registratore del rollback è stato avviato registrando il PID reale (`bash -c 'echo $$; exec …'`).
- **Ipotesi, non dimostrata:** il picco viene dalle connessioni HTTP contemporanee sopra un heap più piccolo di ≈ 10 KB. Su `a86c680` il passaggio da 1 a 2 client costa ≈ 8,4 KB (50 264 → 41 888); il caso a 3 client su `a86c680` non è stato misurato. Con 2 client il candidato sarebbe stato atteso intorno a 31 KB, ma anche questo non è misurato.
- **`heap_largest`** resta 31 732 B anche quando `heap_free` scende a 43 KB e `heap_min_free` a 9,7 KB: non descrive il blocco massimo dell'heap generale in quei momenti. Non lo uso come misura di frammentazione.
- **Regola applicata senza rilassamenti:** sotto 24 KB → stop e rollback. L'ipotesi sui 3 client non cambia l'esito.

## 4. P0: percorsi sopra 16 ms (riferimento `a86c680`, stessa sessione)

P0 resta aperto: 22 overrun in 28 passi e 6 release perse nell'acquisizione S1.

| Percorso | Massimo | Note |
|---|---|---|
| WALK_SETTLE → HEIGHT (B6, ripresa dopo STOP) | 22,58 ms | COMPENSATE 14,1 ms, 2 soluzioni |
| STAND/HEIGHT/PRONE attenuati, 2 soluzioni (A2–A8, B2, S1, S8) | 20,2–22,4 ms | COMPENSATE 14,2–15,3 ms; STAND_LIFT 21,8 ms |
| WALK sterzato, joystick STANDARD (B3, S3, S5) | 20,6 ms | COMPENSATE 10,8–14,3 ms |
| RECOVERY (B6) | 17,2 ms | — |

- Medie: WALK 10,67–10,73 ms/release (734 k istruzioni, IPC 0,43, `istall_mem` 0,40 dominante); RECOVERY 10,56; OTHER 8,65 nel blocco A.
- S1 settle: 6 release perse durante ACQUIRING_REAL_POSE, come `t263r` alle 14:29 (su `2638204`); `tp5` alle 14:42 non ne aveva.
- STOP B4 verificato in 2,6 s, come il 16/11 pomeriggio.

## 5. DALY

- **QON 18:02:43Z:** silenzio dal primo poll (18:02:48.098) al 18:05:50.154Z, 92 poll `NO_RESPONSE` con `rx_bytes=0`; D2 `RX_IDLE_HIGH` in tutti i poll (0 fronti RX, FIFO TX vuota). Rientro spontaneo a 187 s di uptime.
  - È il primo silenzio registrato che inizia al boot senza alcun poll riuscito prima.
  - Rispettato il gate: Motion solo dopo 150 poll dal rientro e QON + 6 min.
- **Boot dopo il flash e dopo il rollback:** DALY risponde dal primo poll; 0 errori in 175 poll dopo il flash.
- **LED dell'XY-017 (osservazione dell'operatore):** un solo LED, lampeggio ogni 2–5 s anche a controller spento, nel FAULT e dopo, senza cambiamenti al rientro.
  - Non segue le richieste dell'ESP, quindi non separa DALY, XY-017 e bus.
  - Resta decisiva la misura RS485 su A/B in solo ascolto (mandato separato).
- **Causa: UNKNOWN.**

## 6. Cosa resta aperto (nessuna campagna avviata)

- IRAM G2 e F3 su hardware: da rifare, con nuovo mandato e flash. **Prerequisito:**
  - al massimo 2 client HTTP, cioè poller della suite più un watcher;
  - controllo prima di ogni run che nessun altro processo legga il robot.
- Se l'operatore vuole prima chiarire il margine di memoria senza nuovi flash: misurare su `a86c680`, a riposo, `heap_min_free` con 1, 2 e 3 client HTTP. È una lettura in sola GET, alla prossima accensione.
- Alternativa offline: un insieme IRAM più piccolo di G2c (meno di 10 KB) se il margine con 2 client risultasse sotto soglia.

## 7. File

- `provenance/`: verifiche a robot spento.
- `cold/` e `cold_*`: QON, D1/D2, preflight `a86c680` (`pre/`).
- `ref_settle_20261116_1813*`, `ref_ab_20261116_1814*`: riferimento, con analisi `*_analyze_diag.txt`, `*_overrun_timeline.txt` e `*_settle_analysis.txt`.
- `flashlog/install_cb9ed85/`, `usb_boot/`, `pre/preflight_cb9ed85.json`: flash e boot del candidato.
- `cand_settle_20261116_1824*`, `memstop/`: arresto per memoria (`*_heap_guard.txt`).
- `flashlog/rollback_a86c680/`, `usb_boot_rb/`, `final_state_readonly.json`: rollback e stato finale.
- Strumenti aggiunti in sessione: `fixture_blocks.py`, `heap_check.py`, `heap_guard.py`, `run_with_watch.sh`, `boot_and_record_rollback.sh`.
