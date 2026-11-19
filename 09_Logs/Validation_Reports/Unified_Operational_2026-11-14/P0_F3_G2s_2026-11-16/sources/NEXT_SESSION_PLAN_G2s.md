# Prossima sessione: qualifica di G2 con il controllo di memoria (preparata il 16/11/2026 sera)

Preparata a robot spento dopo il MEMSTOP di `cb9ed85`. **Non autorizzata:** serve un nuovo mandato con autorizzazione al flash. Il robot ha `a86c680` installato.

## Cosa cambia rispetto al 16/11 sera

- **Due client HTTP al massimo**, imposti dagli strumenti:
  - il poller dello strumento di prova più un solo `http_watch.py`;
  - nessuna dashboard o pagina Twin aperta, nessun altro programma che legga il robot.
- **Registro dei processi** (`tools/session_proc.py`):
  - ogni registratore, watcher, strumento e guardia parte con il PID reale e si chiude con verifica;
  - `run_acquisition.py` rifiuta di partire se è vivo qualunque altro processo di sessione: il caso del 16/11 non può ripetersi (prova T3).
- **M0, controllo di memoria prima di Motion**, primo passo di ogni prova:
  - attende 20 s con i due client attivi;
  - richiede `heap_min_free` ≥ 24 KB su ogni campione e nessuna riga d'allocazione fallita;
  - altrimenti si ferma senza alcun comando Motion e il robot resta READY con coppie OFF.
  - Anche il preflight ha la voce di memoria (9/9).
  - La guardia viva (`heap_guard.py`) e lo STOP restano invariati durante le prove (prova T4).

## Candidato

Consigliato **`6be4690`** (F3 + G2s); `cb9ed85` (F3 + G2) resta recuperabile. Motivo e numeri: `../REPORT.md` §3–4. Immagine e pacchetto: `../g2s/`.

## Sequenza minima (un solo flash)

1. QON, `cold_start.sh` (D1/D2), gate DALY (QON + 6 min, 150 poll). Chiusura del registratore cold con `session_proc.py stop` prima di ogni prova.
2. Preflight `a86c680` **9/9**.
3. **Riferimento `a86c680`** nella stessa sessione, per il fattore di sessione: `run_acquisition.py` con `fixture_settle.py` e con `fixture_blocks.py --blocks A,B` (M0 incluso).
4. ROM, `rom_checks.sh`, flash unico del candidato, boot con il registratore dal registro; gate DALY di nuovo (uptime ≥ 6 min, 150 poll); preflight 9/9.
5. **Controllo discriminante: M0 sul candidato**, prima di ogni Motion e con i due client.
   - **Atteso con G2s:** 28–37 KB. Con G2c sarebbe 24–34 KB, perché il transitorio a 2 client misurato su 6 build va da 26,8 a 36,2 KB.
   - **Sopra 24 KB:** proseguire con gli stessi passi del punto 3. Qui si misura il beneficio IRAM nella stessa sessione.
   - **Sotto 24 KB:** fermarsi senza Motion e fare il rollback ad `a86c680`. Vorrebbe dire che 2 client non bastano a spiegare il calo: G2 non è qualificabile con questo margine.
6. **FAST (F3):** `fixture_blocks.py --blocks E,F,G`, poi `fixture_block_h.py`.
7. Chiusura: PRONE_REST, SAFE_OFF, coppie OFF, stream BMS OFF, `session_proc.py check --expect-none` → `SAFE_TO_POWER_OFF`.

**Facoltativo**, solo se l'operatore vuole chiudere la causa del 16/11. Fra il punto 3 e il punto 4, su `a86c680` a riposo (Motion inattivo, coppie OFF), aggiungere per 2 minuti un terzo client di sola lettura.
- **Prevista con l'ipotesi H1:** `heap_min_free` vicino a 20 KB (70,3 − 50,4).
- **Se resta sopra 30 KB:** H1 cade e la causa del 16/11 resta da cercare.
- Il riavvio del flash azzera il minimo storico prima del candidato. Non serve per qualificare G2.

## Criteri (fissati prima)

| Voce | Criterio |
|---|---|
| Memoria | `heap_min_free` ≥ 24 KB in M0 e per tutta la sessione; sotto, o con un'allocazione fallita: stop e rollback, nessuna ripresa di Motion |
| IRAM | Stessa sessione, per classe: ms/release, `istall_mem`, overrun, massimi |
| F3 | E → F3 e F1–F9 senza FAULT; STOP nel FAST verificato; nessun `refused_by=STOP_PLAN` |
| P0 | Chiuso solo con 0 perse operative, 0 overrun, massimo ≤ 16 ms |
