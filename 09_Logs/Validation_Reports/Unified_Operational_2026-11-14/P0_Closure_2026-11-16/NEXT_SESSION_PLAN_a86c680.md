# Prossima validazione hardware — candidato `a86c680` = `82c1af9` + P5

Preparato il 16/11/2026 a robot spento. **Non autorizzato:** richiede un nuovo mandato. Procedura e strumenti sono quelli della sessione `2638204` del 16/11 (`MATDOG_1_0_2_DIAG_2638204_HW_20261116/README.md`); qui solo le differenze.

## Identità (VERIFIED OFFLINE)

| Voce | Valore |
|---|---|
| Installato (atteso) | `2638204fb4ef`, APP `79018a28…c6f4`, validato sulla fixture il 16/11 (A–G 65/65, H 19/19) |
| Candidato | `a86c680` (`cand/diag-p0-p5-20261116`) = `82c1af9` + P5 `b104f00` (cherry-pick identico di `87338f8`) + nota di composizione. **P4 esclusa.** |
| APP | `f7bc8b627fe2ebd1cb4bc13373a230adf684c1917cc18ff12e43bbb6fb77e55b`, 1 783 744 B, CLEAN, ROBOT_POWERED, OTA 0; `image-info` valido. Bootloader, tabella e sdkconfig identici a `82c1af9` e `2638204`; IRAM e DRAM identiche a `82c1af9`; `.flash.text` +8 B (solo `StopPlan::build` cambia dimensione) |
| Script | `preflash/flash-qualified-app0-1.0.2-dev.8-a86c680.sh` (`c77428e1…`), autorizzazione `MATDOG_DIAGP5_FLASH_AUTHORIZED`. Derivato da quello di `2638204` solo nei valori pinnati (`script_diff_vs_2638204.txt`); dry gates PASS; 4 controlli negativi REFUSE; rifiuto senza autorizzazione |
| Rollback esatto | `2638204` con lo script che l'ha installato (`rollback-…-2638204.sh`, `561c64d1…`, identico byte per byte), dry gates PASS; secondo livello `82c1af9` (`aa8e16db…`). Solo se fallisce un criterio di sicurezza |
| Strumenti | `tools/` (`TOOLS.sha256`): quelli del 16/11 con `EXPECT_BUILD='a86c6800b543'`. `rom_checks.sh` verifica `2638204` come installato; `fixture_suite_2638204.py` per la prova mirata sulla build installata; `overrun_timeline.py` per gli overrun per singola release |

## Perché questa e una sola validazione

1. **P5 sul chip.** Il picco WALK_SETTLE (24–27 ms in 4 dei 6 rilasci sterzati misurati il 16/11, su entrambe le build) ha questa firma: 1 soluzione, 4 IK, 4 Newton, 11 FK, nessun frame, compensazione 19–21 ms. Il rig con il regime del chip lo riproduce come prova STOP rifiutata dopo la ricerca completa. P5 porta quelle release da 1,5–2,6 M a 0,66–0,69 M istruzioni, con esiti identici. Serve vedere se la firma sparisce sul chip.
2. **Confronto nella stessa sessione.** Il 16/11 il tempo per istruzione è cambiato del 5–7 % fra due sessioni, a parità di codice. Per questo la stessa prova mirata (D8–D11) si esegue prima su `2638204` e poi su `a86c680`, nella stessa sessione.
3. **Misura di riferimento** per il passo successivo di P0: la fascia 15–20 ms dei cammini, il 90 % delle release sopra i 15 ms.

## Sequenza

Come il 16/11, nello stesso ordine:
1. QON e `cold_start.sh` (D1+D2), gate BMS ≥ 150 poll consecutivi OK.
   - Un silenzio si registra fino al rientro e ferma le prove.
   - Il gate riparte dal rientro; un singolo poll valido non basta.
2. Preflight in sola lettura di `2638204`: `EXPECT_BUILD=2638204fb4ef`, firma USB, `LOAD_COUNT=4` gen-1.
3. **Prova mirata su `2638204`:** `fixture_settle.py fixture_suite_2638204.py t263r_<ora>` + `http_watch.py`.
4. `rom_entry.sh`, `rom_checks.sh` (digest di `2638204` uguale), flash unico `a86c680`, `boot_and_record.sh`.
5. Verifica passiva:
   - build `a86c6800b543`, `LOAD_COUNT=4` gen-1, restore 12/12 `be8d5902bb4e37da`;
   - census, IMU, SAFE_OFF 8191, coppie OFF, D2 e D3;
   - gate BMS 150, preflight 8/8.
6. **Prova mirata su `a86c680`:** `fixture_settle.py fixture_suite.py tp5_<ora>`.
7. Suite A–G, poi blocco H se 65/65.
8. Chiusura: PRONE_REST, SAFE_OFF, coppie OFF, stream BMS OFF → `SAFE_TO_POWER_OFF`.

## Criteri (fissati prima)

| Voce | Criterio | Come |
|---|---|---|
| Regressione funzionale | 65/65 + 19/19, nessun fault; ogni passo finisce come il 16/11 | `results.json`, `compare_steps.py` |
| STOP | verifica dello STOP entro 2,2–2,6 s come il 15–16/11; ripresa a STAND_REACHED | blocco H e passi STOP |
| **P5** | nei rilasci sterzati (D5, D7, D9, D11 della suite; S4, S6 della prova mirata) **nessuna release WALK_SETTLE sopra 20 ms con la firma 1 soluzione e compensazione ≥ 15 ms**. La release più pesante dal boot non ha quella firma | `analyze_settle.py`, `overrun_timeline.py` |
| P5, esito negativo | se la firma compare ancora, la causa del picco non è la ricerca della prova STOP (ipotesi H1 respinta per quella release), anche se l'equivalenza resta dimostrata | idem |
| Confronto | per classe, nella stessa sessione: ms per release, istruzioni, IPC, stalli, ns per istruzione (`2638204` prova mirata contro `a86c680` prova mirata). Contro il 16/11 solo con il fattore di sessione dichiarato | `session_compare.py`, `analyze_diag.py` |
| Overrun | per singola release e per classe; nessuna soglia nuova | `overrun_timeline.py` |
| Qualità IMU | sterzato: media e massimo contro `46b3175` (**1,39 / 13,30 mrad a 1 Hz**), invariato. Si riporta accanto l'incertezza del campionamento: media ±30 %, massimo ×2 nel rig | `session_compare.py` |
| Release perse | 0 operative. Attese ACC (prima attivazione con i servo appena alimentati) e dump diagnostiche classificate a parte; nessun contatore corretto | `command_timing`, D3 |
| **P0** | chiuso solo con **0 perse operative, 0 overrun, massimo ≤ 16 ms** e nessuna regressione. **Atteso: non chiuso**, per la fascia 15–20 ms dei cammini (`../REPORT.md` §C) | — |

## Che cosa non misura

Non chiude P0. Anche con P5 restano:
- le release attenuate a 2 soluzioni (20–22 ms);
- WALK STOP e cammini sterzati con soluzione nominale pesante (20–25 ms);
- migliaia di release dei cammini fra 15 e 20 ms.
