# MATDOG 1.0.2-dev.6 `450d2c7` — validazione hardware del 10/11/2026: rapporto di chiusura

**Fase chiusa dall'operatore il 10/11.** Il robot è spento con KEY OFF e riposto sulla fixture; non è autorizzato nessun altro accesso hardware. Questo rapporto sostituisce la versione delle 21:02Z; le correzioni sono elencate al §7.

| | |
|---|---|
| Build installata | `450d2c793dcb`: commit `450d2c793dcb76d08ea341eb5c4b1e50215cee40` (branch locale `proposal/acc85-runtime`), FW `1.0.2-dev.6` |
| APP | 1 751 040 B, SHA256 `82122e06b6708a9d687f4525f612182c227134f3abcac27f32ed7e266be04256` |
| Rollback (non usato) | `1fbee7b`, con lo script `MATDOG_1_0_2_DEV6_ACCCAUSAL_FLASH_PREP_20261110/flash-qualified-app0-1.0.2-dev.6-1fbee7b.sh` |
| Esito | Fixture Q1–Q8 qualificata. Prima prova a terra positiva ma fuori protocollo. **0 fault.** **14 STOP d'emergenza su 14 verificati** (8 sulla fixture, 6 a terra) |

## 1. Qualificato: criteri del piano rispettati, evidenza completa

| Voce | Risultato | Evidenza |
|---|---|---|
| Flash APP-only (19:16–19:18Z) | Una sola scrittura su `app0` (0x010000–0x1bb800); hash e verify-flash corrispondenti. Non toccati: NVS (0xfe0000–0xff0000), otadata, tabella delle partizioni, bootloader, EEPROM dei servo | `flash/` |
| Boot e H0 | Un solo watchdog reset; banner `build_id=450d2c793dcb`; BMS pronto a 40 s; preflight 8/8 | `flash/`, `h0/` |
| Registro 85 | `MOTION_ACC_CEILING verified=220 target=220` a ogni attivazione; ACC letta 220 su tutti i 12 servo in FAST; lettura diretta finale **220 su 12/12** | `usb/serial.log` |
| Q1 STAND 120 | STAND_REACHED in 10 s | `usb/summary_full_session.txt` |
| Q2 SLOW | Drive 1000/50, 20 mm/s; WALK STOP → STANDING | idem |
| Q3 e Q3b STANDARD | Drive 1800/180, 30 mm/s; 2 STOP verificati in 2 s a coppia ON; riprese con WALK e con STAND | idem |
| Q4 FAST | Drive 1950/220, 32 mm/s; 2 STOP verificati (3 e 2 s); riprese riuscite | idem |
| Q5 cambi di profilo in camminata | Sempre WALKING. Transizione completa, fino alla velocità del nuovo profilo: SLOW→STANDARD 8 s, STANDARD→FAST 4 s, FAST→SLOW 13 s, SLOW→FAST 13 s (attesi circa 8 / 4,5 / 15 s). Il drive cambia prima, dopo 1–10 s. Superati anche una selezione sostituita da un'altra e un WALK STOP durante un cambio | idem |
| Q6 STOP nelle transizioni | 4 STOP verificati in 1–2 s. Selezione rifiutata con STOP attivo (`STOP_LATCHED`). Ripresa con STAND e WALK nel profilo selezionato | idem |
| Q7 joystick in avanti | Fondo scala 30 / 32 / 20 mm/s (STANDARD / FAST / SLOW); al rilascio → STANDING | idem |
| Q8 SAFE_OFF | SAFE_OFF verificato con mask 8191 e coppia OFF, ottenuto con PRONE perché la dashboard non ha un pulsante SAFE_OFF; robot in MAINTENANCE | idem |

**Criteri di interruzione nella finestra della fixture (fino alle 19:43Z): nessuno** (`closure/summary_fixture_q1_q7.txt`). Alle 19:24:52Z una finestra termica (servo 22, 150 °C letti due volte) si è aperta e chiusa tra due campioni: a 1 Hz non è osservabile (vedi D3).

## 2. Verificato fuori protocollo: prima prova a terra (19:43–19:58Z)

Le prove le ha decise l'operatore. **Non sono una qualifica G0–G5:**
- batteria sotto gli 11,0 V richiesti: da 10,6 a 10,2 V;
- anticaduta non confermata per iscritto;
- robot trasferito a terra in STAND, a coppia ON;
- sequenza libera ed esposizioni brevi;
- **un criterio di interruzione del runbook è stato colpito e non applicato.** Alle 19:57:21Z `hot_pending_mask` = 1 per un campione: servo 13 a 80 °C, confermati dalla rilettura diretta. Il firmware ha chiuso la finestra al campione normale successivo, senza fault. La sessione è proseguita per 85 s, fino a PRONE_REST.

| Prova | Risultato |
|---|---|
| STAND | 150 mm per 7,5 min consecutivi (19:48:54–19:56:22Z) e 120 mm a tratti. In STANDING: carichi ≤ 176 ‰, errori ≤ 21 tick, anche ≥ 7,3 tick dalla box |
| Camminata (campioni a 1 Hz) | Circa 2 min in tutto: SLOW 73 s (19 avanti, 18 indietro, 36 in sterzata), FAST 34 s, STANDARD 11 s |
| Margine delle anche dalla box `kHeight` | SLOW avanti 5,3 tick; SLOW indietro 4,3 (come dev.4 in SLOW); FAST 7,3; STANDARD 7,3, ma su soli 11 s. In sterzata vale la box `kSteer`, quindi il margine `kHeight` non si applica |
| Carichi in camminata | Fino a 460 ‰ (UPPER, STANDARD) e 452 ‰ (FAST) |
| STOP d'emergenza | **6 su 6 verificati** a coppia ON in 2,0–2,4 s (tracce dense): 1 in SLOW, 2 in STANDARD, 1 in FAST, 2 durante la discesa PRONE. In STOP: anche a 7,3–16,3 tick, carichi ≤ 248 ‰, errori ≤ 30 tick (soglie 400 ‰ e 48 tick). Riprese con STAND e WALK riuscite |
| PRONE | Discesa completa fino a PRONE_REST, poi SAFE_OFF verificato |

In queste condizioni il rischio della RCA (anca fuori box o STOP non assestato sotto carico) non si è manifestato: superficie piana, camminate brevi, STOP in fasi non scelte.

## 3. Non verificato

| Caso | Stato |
|---|---|
| Qualifica a terra G0–G5 | Da fare con batteria ≥ 11,0 V, anticaduta confermata e progressione condizionata a un margine delle anche ≥ 4 tick |
| STOP a terra a fine swing anteriore (tre appoggi) | Rischio principale della RCA; mai provocato |
| Mantenimenti lunghi dopo uno STOP; camminate lunghe a terra | A terra STANDARD solo 11 s, FAST 34 s |
| Frenata con ACC 180/220 sotto carico | Provata solo indirettamente (STOP verificati), non misurata |
| Cambio di profilo con la leva tenuta | Col mouse non si può fare: serve il multitouch (telefono o tablet). Verificato solo offline (controllo DOM 21/21) |
| SAFE_OFF dalla dashboard | Il pulsante non esiste. Alternative: PRONE, USB `@SERVO SAFE_OFF 51` oppure KEY OFF |
| Joystick laterale, in rotazione e all'indietro in STANDARD e FAST | Non implementato, solo in SLOW: vedi l'handoff |
| Avviso batteria durante il moto | A coppia ON il LED non si aggiorna (§4) |

## 4. Segnalazioni LED della batteria

**Osservato dall'operatore:** prima il LED arancione di batteria bassa, poi il rosso critico lampeggiante; poi KEY OFF.

**Registrato dal firmware**, fino alle 20:01:35Z (fine della registrazione):
- LED `READY` fino alle 19:58:45Z, poi `BATTERY_WARNING` fino all'ultimo campione;
- `BATTERY_CRITICAL` mai registrato: il rosso è comparso dopo la chiusura della registrazione;
- BMS DALY: allarmi `0000 0000 0000 0000`, stato STATIONARY;
- ultimo campione: pacco 10,2 V; cella minima 3 375 mV, in calo di circa 9 mV/min a −0,3 A; SOC 41,3 %.

**Soglie del firmware** (`LedStatusPolicy.cpp`). Servono solo alla presentazione: non bloccano il moto e non scrivono su KEY o MOS.

| Stato | Ingresso | Uscita | LED |
|---|---|---|---|
| `BATTERY_WARNING` | Cella minima ≤ 3 500 mV o pacco ≤ 10,50 V su 2 campioni DALY distinti consecutivi, oppure SOC ≤ 30 % | Cella ≥ 3 600 mV e pacco ≥ 10,80 V (2 campioni); SOC ≥ 33 % | Arancione, respiro lento (3 s) |
| `BATTERY_CRITICAL` | Cella minima ≤ 3 350 mV o pacco ≤ 10,05 V, subito; oppure SOC ≤ 15 % | Cella ≥ 3 450 mV e pacco ≥ 10,35 V (2 campioni); SOC ≥ 18 % | Rosso, lampeggio 250/250 ms |

Le protezioni del BMS sono separate da queste soglie (3,20 V/cella e 9,60 V/pacco secondo i commenti del firmware) e non sono state raggiunte.

**Correlazione.**
- In camminata il pacco è sceso a 10,50 V alle 19:46:50Z e la cella minima a 3 500 mV alle 19:48:45Z. Il LED è diventato arancione solo alle 19:58:50Z, entro 4 s dal rilascio della coppia (PRONE_REST alle 19:58:46Z).
- Il motivo è nel codice: con Motion attivo la presentazione LED è sospesa (`Controller.cpp:1075` e `:1198`, «Presentation can wait for torque OFF»). È una scelta di temporizzazione, non un guasto.
- **Conseguenza: a coppia ON l'avviso batteria non compare né sul LED né sulla scheda batteria del portale**, che usa gli stessi fatti oppure il SOC; la dashboard Motion non mostra la batteria. La tensione resta leggibile in `/status` (`bms_pack_v`, `bms_cell_min_mv`).
- Il rosso lampeggiante è compatibile con la soglia di cella a 3 350 mV, 25 mV sotto l'ultimo valore registrato, raggiungibile pochi minuti dopo. Non è registrato. Le soglie SOC non possono averlo causato.
- **Il SOC DALY non descrive l'energia residua:** è sceso solo da 45 a 41 % mentre la cella minima calava da 3,65 a 3,375 V. La corrente letta dal DALY è rimasta tra −0,2 e −0,7 A per tutta la sessione, con stato STATIONARY anche in camminata. Per l'autonomia conta la tensione di cella.

Nessun intervento sul firmware. Per la prossima sessione: ricaricare, partire da almeno 11,0 V e, durante il moto, sorvegliare la tensione invece del LED.

## 5. Questioni diagnostiche aperte (registrate, nessun intervento sul firmware)

### D1 — Release perse e overrun

**Fatti.** In tutta la sessione: 131 release perse (periodi da 20 ms saltati) e 2 657 overrun (release oltre 20 ms); 0 fault. Attribuzione a 1 Hz:

| Dove | Perse | Note |
|---|---|---|
| 12 riprese dopo STOP (`STAND_FROM_STOPPED_WALK`) | 101, più 5 a cavallo di STOPPED | 6–12 per ripresa, con 29–66 overrun ciascuna e lavoro fino a 36 ms per release |
| Camminata SLOW sterzata col joystick, 30 s (19:47:06–19:47:36Z) | 16 | Lavoro fino a 45,97 ms, il massimo della sessione; overrun su circa il 22 % delle release. Come su dev.4 (43 ms, 15 perse) |
| Prima attivazione: registro 85 configurato a coppia OFF | 5 | 116 ms in ACQUIRE |
| Isolate (camminate FAST e STANDARD, cambio di quota) | 4 | |

Overrun nelle camminate lunghe: 5–9 %. Sono più alti nelle camminate brevi, dove pesa l'avvio, e nella sterzata.

- **Ipotesi da verificare:** costo di calcolo per release (IK e compensazione) della legge di ripresa e di quella sterzata, non il bus.
- **Prossimo passo, offline:** scomporre le release più pesanti per parte (READ, NOMINAL, COMPENSATE, CONVERT, FRAME). In questa sessione la scomposizione non è stata raccolta.

### D2 — HTTP 503 `MOTION_BUDGET_UNAVAILABLE`

**Fatto.** Un solo caso in tutta la sessione: il contatore `transport.late` passa da 0 a 1 tra le 19:47:35 e le 19:47:36Z, alla fine della camminata SLOW sterzata. La dashboard ha mostrato il 503. Le sue tracce non lo contengono perché conservano solo gli ultimi 48 comandi, dalle 19:48:32Z.

**Meccanismo** (`HttpTransport.cpp:395–495`):
- con Motion attivo, una richiesta non esente ha bisogno di 5 ms liberi prima della release successiva e ha un budget di 100 ms (`kMotionBusyBudgetMs`);
- se il Controller non la prende in tempo, la richiesta viene abbandonata con 503 e **non viene eseguita**, nemmeno in ritardo;
- sono esenti: lo stato Motion, la velocità nulla (WALK STOP e rilascio della leva), login e sessione, `/status` e `/wifi/status`.

**Legame con D1:** in quella camminata le release arrivavano a 46 ms, quindi il loop non lasciava quasi mai 5 ms liberi.

**Effetto.** Un aggiornamento joystick con velocità non nulla non è stato eseguito. La camminata ha proseguito col comando precedente e si è conclusa al rilascio della leva (WALK_STOPPING alle 19:47:36Z). Nessun effetto sulla sicurezza, perché STOP e velocità nulla sono esenti.

**Aperto:** quanto spesso succede in camminate sterzate più lunghe; come la dashboard deve segnalare il rifiuto (decisione futura).

### D3 — Temperature spurie

**Fatti.**
- 46 letture bulk ≥ 70 °C, ognuna riletta subito in diretta dal firmware, entro 2 ms.
- In 9 casi la rilettura ha confermato il valore anomalo, fino a 150 °C, sui servo 11, 13, 21, 22 e 31. Le finestre aperte (1 sulla fixture, 8 a terra) sono state tutte chiuse dal campione normale successivo.
- Negli altri casi la rilettura era normale (per esempio 87 → 29 °C) e ha sostituito il valore bulk.
- Nelle istantanee a 1 Hz: 89 letture isolate (29–80 °C) su 11 servo su 12, tutti tranne il 23. **Tutte a coppia ON; nessuna su 2 856 letture a coppia OFF.**
- Frequenza: 0,66–0,82 % in camminata, 0,13–0,29 % in STAND o in movimento. Non dipende dall'ACC del profilo: SLOW, con ACC 50, ha la frequenza più alta.
- Le altre letture restano tra 23 e 33 °C. Nessun fault.

**Già noto** dalla RCA del 29/10: 46–83 °C su 3 servo, solo a coppia ON, con checksum valido. Ora coinvolge più servo e valori più alti.

**Ipotesi da verificare:** la misura di temperatura interna al servo viene disturbata quando il motore assorbe corrente. Lo suggeriscono il fatto che succede solo a coppia ON e la rilettura diretta coerente; un errore del bus è poco probabile.

**Il filtro funziona e non maschera un surriscaldamento reale:** 70 °C per 500 ms continui portano comunque al fault, e resta la protezione del servo.

**Prossimo passo:** analisi offline dei valori (per esempio la ricorrenza di 150). In una futura sessione autorizzata, una prova al banco su un singolo servo sotto carico.

## 6. Integrità e conservazione (verifica di chiusura, sola lettura)

| Controllo | Esito | File |
|---|---|---|
| Controlli offline dopo la sessione | **20/20 PASS:** candidato a `450d2c7`, pulito, non pubblicato; manifest, APP e tabella delle partizioni; script di flash e di rollback, che rifiutano senza autorizzazione e superano i gate a secco; asset di rollback 12/12; worktree di rollback pulito | `closure/offline_gates_post_session.txt` |
| Checksum | RCA 9/9, cartella ACC220 operativa 58/58, ACCCAUSAL 4/4. In questa cartella il manifesto delle 21:02Z è invariato, tranne i file aggiornati alla chiusura; è conservato in `closure/`. Il nuovo `SHA256SUMS` copre tutta la cartella | `closure/integrity_checks.txt` |
| NVS | Il flash ha scritto solo `app0`. Durante la sessione nessun comando di calibrazione o di rete, gli unici che scrivono in NVS. Il firmware scrive il registro 85 solo in RAM, con il Lock chiuso: EEPROM invariata. Non riletta sul robot (accesso non autorizzato) | idem |
| Worktree | Candidato `450d2c7` pulito; rollback `1fbee7b` pulito; MuJoCo `fe66610` non toccato, 0 modifiche. `/tmp/mp-a1a2` è un residuo staccato del 07/11, il cui commit è già nei branch | idem |
| Raggiungibilità | `450d2c7` esiste solo sul branch locale `proposal/acc85-runtime`, senza remoti né tag: **commit, build ed evidenze sono solo su questo PC**. Pubblicarli è una decisione dell'operatore. `1fbee7b` è anche su `origin/sim/mujoco-twin-20261110` | idem |

Modifiche fatte alla chiusura (nessuna a firmware, worktree, script di flash o binari):
- `session_summary.py`: il cambio di profilo ora è misurato fino alla velocità del nuovo profilo; il criterio termico è tornato quello del runbook §5;
- i riepiloghi rigenerati;
- questo rapporto;
- la cartella `closure/`;
- l'handoff.

## 7. Correzioni rispetto alla versione delle 21:02Z e al messaggio di chiusura in chat

- **Criterio termico.** Dopo la sessione avevo modificato il riepilogo perché contasse una finestra termica solo se aperta per 2 campioni. Così il criterio del runbook (§5: `hot_pending_mask` ≠ 0) colpito alle 19:57:21Z risultava «nessuno». Ho ripristinato il criterio scritto: ora i riepiloghi dell'intera sessione e della parte a terra riportano 1 criterio colpito (exit 2).
- **STOP sulla fixture:** sono 8; in chat avevo scritto 10. Nella sessione sono 14 in tutto, tutti verificati.
- **STAND a terra:** 150 mm per 7,5 min, non «120 mm per circa 5 min».
- **Margine delle anche in SLOW a terra:** i 4,3 tick sono all'indietro; in avanti sono 5,3.
- **Riprese dopo STOP:** 6–12 release perse ciascuna, non «circa 25».
- **Cambi di profilo:** il riepilogo misurava il cambio di drive (1–10 s), non la transizione completa (4–13 s); ora le riporta entrambe.
- **HTTP 503:** datato e attribuito a un aggiornamento joystick con velocità non nulla.
- **Batteria:** all'inizio 10,9 V, non 11,0.

## Evidenze (questa cartella)

| Percorso | Contenuto |
|---|---|
| `pre/`, `flash/`, `h0/` | Preflight, log di ROM, layout, flash, boot e H0 |
| `usb/serial.log`, `usb/events.jsonl` | USB completo e comandi inviati |
| `usb/http.jsonl` | Stato Motion a 1 Hz e `/status` ogni 5 s |
| `usb/summary_full_session.txt`, `terra/summary_ground.txt` | Riepiloghi di `session_summary.py` |
| `terra/dashboard_trace_47.json`, `terra/dashboard_trace_48.json`, `terra/hip_margins_dashboard_traces.txt` | Tracce dense della dashboard e margini delle anche |
| `closure/` | Controlli di chiusura, riepilogo della sola fixture, manifesto delle 21:02Z |
| `HANDOFF_JOYSTICK_MULTIDIREZIONALE.md` | Handoff per il prossimo sviluppo |
| `READY_REPORT.md` | Runbook della sessione |
| `SHA256SUMS` | Checksum di tutta la cartella |
