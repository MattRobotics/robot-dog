> Copia del rapporto di sessione. I percorsi relativi (`cold/`, `suite_20261115_1220/`, `preflash/`, `bms_rca/`, …) si riferiscono alla cartella delle evidenze `verification-artifacts/MATDOG_1_0_2_DIAG_IRAM_E321E86_HW_20261115/` (sigillata, `SHA256SUMS.txt` 363 file), non a questo repository.

# Validazione hardware IRAM `e321e86` e RCA DALY — 15/11/2026, pomeriggio

Ore locali CET; nei log gli orari sono in UTC (un'ora in meno). Mandato: flash APP-only del candidato diagnostico IRAM, suite fixture automatica A–G e RCA DALY prioritaria, senza strumentazione aggiuntiva. L'operatore era presente alla fixture, robot sostenuto e zampe libere. Piano e criteri fissati prima dell'accensione: `README.md`.

## Esito

| Punto | Esito | Stato |
|---|---|---|
| Flash APP-only di `e321e86` | **PASS**, un solo flash, tutti i gate superati, `verify-flash` con digest uguale | VERIFIED HARDWARE |
| Verifica passiva della nuova build | **PASS**: build `e321e8668139`, NVS gen-1 invariata, restore 12/12, census 13/13, IMU, DALY, SAFE_OFF 8191, diagnostica Timing | VERIFIED HARDWARE |
| Suite fixture A–G | **65/65 PASS in un'unica esecuzione**, senza interruzioni | VERIFIED HARDWARE |
| Timing (PMU reale, stessi blocchi del 15/11) | WALK −1,0…−1,3 ms per release (−8/−9 %), RECOVERY −1,7…−2,1 ms (−10/−11 %); N+C −13/−15 %; `istall_mem` 0,47–0,50 → 0,37–0,45; IPC 0,35–0,38 → 0,40–0,44 | VERIFIED HARDWARE |
| Perse, overrun, picchi | perse 28 → 10 (6 sono l'attesa ACC alla prima STAND in entrambi i casi: operative 22 → 4); overrun 648 → 323; picco 39,67 → 36,11 ms | VERIFIED HARDWARE |
| **Timing P0** | **NON chiuso**: il criterio dev.8 §8.3 (0 perse, 0 overrun, ≤ 16 ms) resta lontano | — |
| Memoria interna | −4 600 B a riposo a parità di fase (atteso −4 608). Minimo nella suite 45 216 B, blocco massimo 31 732 B invariato | VERIFIED HARDWARE |
| Regressioni | nessuna funzionale. Overrun leggermente più alti in tre riprese dopo STOP (D15, D19, F5), con 0 perse | VERIFIED HARDWARE |
| **BMS DALY** | guasto a freddo catturato con D1: due silenzi (213 s e 662 s), iniziati entrambi da una **risposta troncata a metà frame**; rientri spontanei. **Causa UNKNOWN circoscritta**, nessun difetto software dimostrato, nessuna patch | §A |
| Stato finale | **PRONE_REST**, SAFE_OFF 13/13 (mask 8191) verificato, coppie OFF, `e321e86` installato, porta USB libera, nessun processo attivo | VERIFIED HARDWARE |

Non eseguiti: reset durante il guasto; scritture NVS, EEPROM, calibrazione o BMS; modifiche a firmware, limiti o criteri; FAST 40; Ground Contact probing; movimenti a terra.

## Identità

| Voce | Valore |
|---|---|
| Prima | `8530d3a11ac9` (APP `75fdd3ac…0c4d4b`), confermato via USB e con `verify-flash` in ROM (digest uguale) |
| Ora | **`e321e8668139`**: `8530d3a` + IRAM G1 (`277f3c1`, `b570dd9`, `e321e86`), branch locale `cand/diag-8530d3a-iram-20261115`. APP `b88c448d8b95f44fb851d727bcb50d1438b6bc8b61918b3f7d7fcebb4b8847b2`, 1 781 840 B, app0 `0x010000` |
| Script | `preflash/flash-qualified-app0-1.0.2-dev.8-e321e86.sh` (`c250ace1…`): derivato da quello di `8530d3a` cambiando solo i valori pinnati; gate a secco e 4 controlli negativi offline |
| Rollback | `8530d3a`: immagine installata verificata uguale, script pinnato del 14/11 (`be19d757…`), gate a secco PASS. **Non eseguito** |
| Branch operativo | `ca0460a` = origin al momento della sessione; `main` `6838e4d` |

## 1. Cronologia

| Ora CET | Evento |
|---|---|
| 12:38:50 | registratore a freddo in attesa della porta |
| 12:52:03 | QON: porta rilevata, banner registrato, `POWERON`, build `8530d3a11ac9` |
| 12:52:29 | l'operatore disconnette il dongle Wi-Fi dell'host (rete domestica). Internet resta sul cavo (`network_check.txt`) |
| 12:53:26 | **primo silenzio DALY** (risposta 98/129), Motion in fail-closed `BMS_TELEMETRY_STALE`, SAFE_OFF |
| 12:57:02–12:57:10 | rientro di 4 poll, poi **secondo silenzio** (risposta 61/129) |
| 12:58:31 | l'host entra nell'AP del robot (operatore); dalle 13:02:08 osservatore HTTP (Fase B) |
| 13:08:10 | DALY di nuovo valido, senza alcun intervento |
| 13:13:10 | 151 poll consecutivi OK (gate BMS); preflight `8530d3a` 8/8 PASS |
| 13:13:30–13:14:58 | ROM, layout, verifica dell'immagine installata, **flash** (40 s) |
| 13:15:14 | boot con watchdog-reset, registratore aperto 1,5 s dopo; verifica passiva PASS |
| 13:20:22 | gate BMS dopo il flash (153 OK, nessun fallimento dal boot); preflight `e321e86` 8/8 PASS |
| 13:20:53–13:28:04 | **suite A–G, 65/65 PASS** |
| 13:28:20–13:29:55 | stato finale in sola lettura (HTTP 8/8, USB), stream BMS riportato a OFF, porta libera |

### A. RCA BMS DALY senza strumentazione aggiuntiva — 15/11/2026

Fonti: D1 per poll (`@BMS STREAM ON`), log USB, `/status` HTTP, journal NetworkManager dell'host, storico delle sessioni e Git. Nessun analizzatore, ricevitore o modifica elettrica; nessuna scrittura BMS; nessun reset durante il guasto. Ore UTC nei log, CET (UTC+1) nel testo dove indicato.

### A.1 Cattura a freddo di oggi (VERIFIED HARDWARE)

Installato `8530d3a11ac9`. Accensione con QON alle 12:52:03 CET. Robot fermo sulla fixture, AP del robot con 0 client, nessun HTTP fino alle 12:58:31 CET, nessuna app Smart BMS (dichiarazione dell'operatore).

| Uptime | UTC | Evento (D1) |
|---|---|---|
| 0 | 11:52:04,5 | POWERON; banner e motivo del reset registrati; primo poll OK a 3,2 s |
| 79,2 s | 11:53:23,7 | ultimo poll OK (129 B) |
| ~81,2 s | 11:53:26 | **risposta troncata 98/129**, primo byte puntuale a 13 ms; poi `rx 0`, `stale 0` a ogni poll |
| 297,3 s | 11:57:02 | rientro spontaneo: 4 poll completi |
| ~303,3 s | 11:57:10 | **risposta troncata 61/129**, primo byte a 17 ms; poi di nuovo `rx 0`, `stale 0` |
| — | 11:58:31 | l'host entra nell'AP (operatore); HTTP della Fase B dalle 12:02:08 |
| 965,5 s | 12:08:10 | rientro spontaneo con risposta completa (129 B, primo byte a 20 ms) |
| — | 12:13:10 | 151 poll consecutivi OK: gate BMS superato; poi preflight 8/8 |

- **Silenzi:** 213 s e 662 s.
- **Firmware all'inizio di ciascun silenzio:** nessun evento Wi-Fi, Motion o LED. Il monitor manda comandi di stato ogni 30 s, l'ultimo 12 s prima.
- **Il BMS non si è riavviato:** il suo conteggio di carica continua durante il silenzio. SOC 74,1 % → 73,8 % → 73,0 %, cioè −72 mAh in 692 s su 9 Ah, pari a ~0,37 A medi contro 0,3–0,5 A misurati.
- **Dopo il flash:** dal boot di `e321e86` alla fine della suite nessun poll fallito, nemmeno durante il Motion (`/status` ogni 5 s).

### A.2 Prima divergenza verificata

> All'inizio di ciascun guasto la UART2 dell'ESP32 riceve **l'inizio di una risposta regolare**: indirizzo corretto, primo byte a 13–17 ms, cadenza normale. Il flusso **si ferma a metà frame**, dopo 98 e 61 byte (~102 e ~64 ms di trasmissione). Da quel momento al pin RX non arriva più alcun byte, nemmeno tardivo o spurio (`stale_bytes=0`), per 213–662 s. Poi le risposte riprendono intere senza alcun intervento.

Il 15/11 mattina il rientro era iniziato con 128/129. Il punto di troncamento varia (98, 61, 128), quindi l'evento è asincrono rispetto alla risposta.

**Ciò che la diagnostica esistente non vede:** se i byte mancanti siano stati trasmessi sul bus RS485. Restano indistinguibili tre punti:
- il trasmettitore del DALY si ferma;
- il bus viene occupato, per esempio dal bridge in trasmissione;
- l'uscita RO dello XY-017 smette di riportare il bus.

**Escluso lato ESP, dal codice e dai dati:**
- **la scadenza:** la risposta completa termina a ~150 ms su 750, e la coda non arriva nemmeno dopo;
- **un blocco del driver RX:** sarebbe permanente, perché il ripristino richiede una lettura che l'applicazione fa solo con byte presenti;
- **l'overflow della FIFO:** resetta la FIFO, ma i byte successivi continuano ad arrivare;
- **un ritardo del loop o dell'ISR:** i byte restano nella UART;
- **una trasmissione ESP durante la risposta:** la scrittura è unica e il prossimo invio arriva dopo ≥ 2 s, con 300 ms di silenzio obbligato.

### A.3 Confronto degli episodi

| Data | Build (`DalyBms.cpp`) | Avvio | Inizio (uptime) | Fine | Condizioni note |
|---|---|---|---|---|---|
| 06/11 | dev.14 `4ff2aad` (`b6ba4db`) | non registrato | ignoto | ciclo di alimentazione dell'operatore | FAULT, `BMS_TELEMETRY_STALE`, LED rossi prima della sessione |
| 08/11 | 1.0.2-dev.3 `94a260c` (`b6ba4db`) | dopo flash (watchdog-reset) | ~9 s, dopo un campione OK | non rientrato a 204 s; KEY OFF/ON | USB non aperta dal boot |
| 09/11 | dev.6 `dd76fad` (`b6ba4db`) | dopo flash | dal boot | spontaneo a ~124–150 s | |
| 09/11 | dev.6 `975fe51` (`b6ba4db`) | dopo flash | dal boot | spontaneo a ~290 s | |
| 14/11 | dev.7 `b561062` (`b6ba4db`) | a freddo | 165,3 s | spontaneo a 741,0 s | host non associato, USB chiusa |
| 15/11 mattina | `8530d3a` (`b15988d`, D1) | a freddo | ~305 s | spontaneo a 740,3 s, primo frame 128/129 | host associato, HTTP 1 Hz |
| 15/11 pomeriggio | `8530d3a` (`b15988d`, D1) | a freddo (QON) | ~81 s, frame 98/129 | 297 s; ricaduta a ~303 s (61/129); rientro a 965,5 s | 0 client AP, nessun HTTP nel primo silenzio |

- **Inizio:** sempre entro ~305 s dall'avvio dell'ESP, sia a freddo sia dopo un flash. Nessun inizio registrato più tardi nelle sessioni lunghe (10/11 ×2, 11/11, 14/11, 15/11).
- **Durata:** da 2 a 11 min, poi rientro spontaneo; due volte è servito un ciclo di alimentazione o KEY.
- **Build:** le stesse build hanno anche avvii buoni (dev.6, dev.7, `8530d3a`).
- **Uptime fissi:** i due rientri a ~740 s del 14 e 15/11 non si ripetono oggi (silenzio fino a 965 s). Non c'è un temporizzatore fisso.
- **Correlazioni negative:** nessuna correlazione con l'associazione dell'host, con l'HTTP o con la USB. Il 14/11 e il primo silenzio di oggi avvengono senza client.

### A.4 Cronologia firmware verificata (blob Git e immagini installate)

| Cambio | Primo commit | Sul robot |
|---|---|---|
| FAULT di salute e LED rosso su DALY REQUIRED `NO_RESPONSE` | `d8b6786`, `34afbc7` (15–16/09) | da settembre |
| SOC e policy LED | `410a25c` (13/10): solo `LedStatusPolicy` | da metà ottobre |
| Capacità 9 Ah per la presentazione | `9512816` (17/10): +5 righe in `DalyProtocol.h`, frame invariato | da metà ottobre |
| Motion rifiutato con campione BMS più vecchio di 3 s | `5469f4e` (22/10, `BMS_OR_POWER_UNAVAILABLE`) | build SIMPLE dal 22/10 |
| **Trasporto:** rimozione di `flush()`, drain limitato, gate `availableForWrite` | sorgenti registrati dal 25/10 23:04; commit `8e20c63` (30/10) | build rc.5 sporche dal 26/10 |
| Nomi `BMS_TELEMETRY_STALE` / `BMS_COMMUNICATION_LOST` | `8e20c63` (30/10) | rc.11 r2 dal 30/10 |
| **Primo guasto documentato** | — | 06/11, dev.14 |
| Battery P1 (avvisi per tensione di cella) | 11/11 | dopo il primo guasto |
| D1 (campi di trasporto in sola lettura) | `9b848b8` (14/11) | `0526cf6`, `8530d3a`, `e321e86` |

- **`DalyProtocol.cpp`:** blob `cdb9290` identico dal 13/10 a oggi. Il frame D2 (8 B, FC03, 62 registri, risposta di 129 B), il poll di 2 s, la scadenza di 750 ms e il silenzio di 300 ms risalgono al 19/09.
- **`DalyBms.cpp`:** `06ce077` fino al 22/10, poi `b6ba4db` dal 30/10 (in tutte le build con guasto, dev.14 compresa), poi `b15988d` (solo D1).
- **Configurazione KEY del DALY:** già `DISCHARGE` (`0x005A`) il 06–07/10. Nessuna scrittura successiva registrata.
- **Visibilità:** dal 15/09 un silenzio DALY accende il LED rosso, dal 22/10 blocca il Motion. Una nuova politica di visibilità **non** spiega la comparsa a novembre. La rinomina del 30/10 cambia solo la causa riportata.
- **Ottobre:** ha registrazioni continue lunghe e tutte OK (03/10: 1 235 letture in 110 min; 06/10; 17/10; 25/10). Nessuna però copre i primi 5 minuti dopo un avvio (il 03/10 l'uptime parte da 17 min, il 17/10 da 45 min). La finestra in cui avvengono tutti gli inizi di novembre non era mai stata osservata con continuità a ottobre.

### A.5 `flush()`, `markSent`, loop e bridge

1. **Byte e tempi di bit sul filo:** identici, perché la UART li sposta da sola a 9600 baud.
2. **`markSent`:** la base della scadenza è anticipata di 8,3 ms, cioè 742 ms dalla fine della TX invece di 750. Irrilevante contro ~150 ms di risposta. Anche `first_byte_ms` include ora la TX: 11–21 ms osservati.
3. **Loop:** senza `flush()` non resta più fermo 8,3 ms a ogni poll. Le iterazioni successive girano durante la richiesta, quindi traffico servo UART1 a 1 Mbit/s su GPIO17/18 (adiacenti a GPIO15/16), RMT del LED, HTTP e stampe USB si sovrappongono alla richiesta DALY sul filo. Prima del 26/10 non accadeva.
4. **Bridge XY-017:** riceve sul TXD la stessa forma d'onda. Cambia solo la concomitanza elettrica.

Valutazione:
- Il punto 3 è l'unico effetto indiretto reale. Ma l'evento d'inizio osservato **non** riguarda la richiesta: il BMS l'ha ricevuta e ha iniziato a rispondere. Riguarda la risposta, che si sovrappone al traffico servo da sempre.
- Le modifiche SOC non toccano lo scheduler DALY (cadenza propria di 2 s) né la UART2 o i suoi pin.

**Un nesso tra rimozione di `flush()` e guasto non è dimostrato né escluso:** richiederebbe un meccanismo elettrico invisibile alla D1.

### A.6 Causa: UNKNOWN, circoscritta

**Difetto software dimostrabile: nessuno.** Il trasporto è identico in tutte le build con guasto. Le stesse build hanno avvii buoni. Il silenzio rientra da solo con lo stesso stato software, mentre un blocco lato ESP sarebbe permanente. Il primo evento è una risposta del BMS interrotta a metà frame. **Nessuna patch.** Non si allungano scadenze, non si maschera `NO_RESPONSE`, non si aggiungono reset.

**Punto non osservabile:** a valle del pin GPIO16, che cosa succede sul bus RS485 nell'istante del troncamento e durante il silenzio. Tre possibilità indistinguibili con D1:
1. il DALY smette di trasmettere (UART o task di comunicazione del BMS; il nucleo resta vivo, come mostra il SOC continuo);
2. il bus è occupato, per esempio lo XY-017 commutato in trasmissione dal suo TXD;
3. l'uscita RO dello XY-017 non riporta il bus.

Il fatto che gli inizi cadano solo nei primi ~5 minuti dopo l'avvio, a freddo o dopo flash, è compatibile con un effetto di riscaldamento o di transitorio elettrico. Non è dimostrato.

**Prossimi passi possibili, solo software e senza nuovo hardware, da decidere:**
- **D2, diagnostica passiva di registro:** campionare in `update()` i bit `rxd`/`txd` e `rxfifo_cnt` del registro di stato UART2 (`uart_struct.h`: `status.rxd`, `status.txd`). Distinguerebbe una linea RX ferma alta (nessuno trasmette, punti 1 o 3 con RO a riposo) da una ferma bassa (bus o XY-017 in stato anomalo, punto 2). Verificherebbe anche che il TXD dell'ESP sia a riposo alto tra le richieste. Nessuna transazione sul bus. Richiede una build e un flash autorizzati.
- **Quiete radio (`@WIFI OFF`, esistente, ripristino automatico dopo 10 min):** alternata tra un avvio e l'altro nei primi 10 minuti, distingue l'ipotesi di accoppiamento RF. Spegne l'HTTP, quindi va decisa dall'operatore.

## B. Validazione IRAM `e321e86` (VERIFIED HARDWARE)

### B.1 Flash e verifica passiva

| Controllo | Esito |
|---|---|
| Preflight prima del flash (HTTP 8/8, USB) | `8530d3a11ac9`, READY, DALY fresco, NVS `VALID_ACKNOWLEDGED` gen-1 (`LOAD_COUNT=4`), census 13/13, IMU, SAFE_OFF 8191, coppie 0 |
| ROM | ESP32-S3 (QFN56) rev v0.2, MAC `14:c1:9f:22:75:94`; tabella `8f756ecb…`, otadata `[1, 0]`, app0 attiva a `0x010000`, rollback del bootloader abilitato |
| Immagine installata | `verify-flash` contro `8530d3a`: digest uguale |
| Flash | `0x010000..0x1c3050` (cancellazione fino a `0x1c4000`), 1 781 840 B in 40 s, hash e `verify-flash` OK. Non toccati: bootloader, tabella, otadata, NVS `0xfe0000..0xff0000`, EEPROM, BMS |
| Boot | `build_id` e `ota_running_build_id` `e321e8668139`; restore `ALLOW_AUTOMATIC` gen-1, fingerprint `be8d5902bb4e37da`, `mapped=12/12`, coppie `UNCHANGED_OFF`; census 13/13 (761 ms); `IMU_INIT=PASS`; DALY ONLINE; LED a READY a 5,3 s; nessun panic, brownout o watchdog |
| Stato | READY/IDLE, SAFE_OFF 8191 verificato, `torque_active=false`, coppia 0 su 12, servo ≤ 27 °C, nessun fault |
| Diagnostica Timing | `@MOTION TIMING`: `diag_preemption=1`, elenco dei task PMU |
| Preflight dopo il flash | 8/8 PASS (`pre/preflight_e321e86.json`) |

### B.2 Suite A–G (`suite_20261115_1220/`, una sola esecuzione)

| Blocco | Passi | Esito | Perse (15/11 → oggi) | Overrun (15/11 → oggi) |
|---|---|---|---|---|
| A: STOP a riposo, STAND 120/150/80/120, PRONE, riarmo | 9 | PASS | 6 → 6 (attesa ACC in A2) | 37 → 5 |
| B: STANDARD, yaw orario 22 s, STOP, 4 s, STAND 120 (WALK_SETTLE), PRONE | 8 | PASS | 7 → 2 | 121 → 71 |
| C: WALK STANDARD 30 s, WALK STOP, PRONE | 6 | PASS | 0 → 1 | 55 → 12 |
| D: indietro, laterali, yaw ±, STOP in due fasi, riprese STAND e WALK, PRONE | 23 | PASS | 14 → 1 | 333 → 180 |
| E: cambi di modo STANDARD → SLOW → STANDARD in cammino | 8 | PASS | 0 → 0 | 25 → 17 |
| F: FAST 32 mm/s (preset 2), STOP, ripresa, WALK STOP, SLOW | 9 | PASS | 1 → 0 | 67 → 37 |
| G: PRONE_REST, dump finale | 2 | PASS | 0 → 0 | 10 → 1 |
| **Totale** | **65** | **65/65** | **28 → 10** | **648 → 323** |

Confronto passo per passo: `compare_steps_vs_20261115.txt`. Il 15/11 la suite aveva richiesto due riprese dopo interruzioni dello strumento; oggi non ce ne sono state. DALY: 90/90 letture OK durante la suite (`suite_summary.txt`).

### B.3 Timing prima/dopo (`analyze_diag.py` invariato, stessi blocchi)

| Blocco | Classe | ms per release, 15/11 → oggi | N+C (ms) | IPC | `istall_mem` |
|---|---|---|---|---|---|
| B | WALK | 13,53 → **12,26** | 10,01 → 8,62 | 0,36 → 0,44 | 0,48 → 0,41 |
| B | RECOVERY | 18,46 → **16,37** | 14,97 → 12,73 | 0,38 → 0,40 | 0,47 → 0,37 |
| C | WALK | 11,78 → **10,77** | 8,26 → 7,15 | 0,36 → 0,42 | 0,49 → 0,45 |
| D | WALK | 12,92 → **11,68** | 9,40 → 8,05 | 0,35 → 0,43 | 0,49 → 0,42 |
| D | RECOVERY | 16,99 → **15,07** | 13,49 → 11,48 | 0,36 → 0,42 | 0,50 → 0,40 |
| E–F | WALK | 11,85 → **10,85** | 8,35 → 7,24 | 0,36 → 0,42 | 0,49 → 0,41 |
| E–F | RECOVERY | 17,13 → **15,41** | 13,65 → 11,85 | 0,36 → 0,44 | 0,47 → 0,42 |

- **Istruzioni per release:** invariate (rapporto QEMU uguale). Il guadagno viene solo dagli stalli d'istruzione in meno: è l'effetto atteso dell'IRAM, ora misurato sul chip.
- **Proiezione offline:** prevedeva N+C −1,6…−1,9 ms (WALK) e −3,0…−3,3 ms (RECOVERY). Misurati −1,1…−1,4 e −1,8…−2,2: circa due terzi.
- **Picchi dal boot:** WALK 38,96 → 36,11 ms; RECOVERY 39,67 → 30,90 ms.
- **Gap tra le visite Motion per finestra:** 26,6–29,1 → 20,9–24,5 ms. Tempo fuori visita 3,8–4,2 → 3,2–3,3 ms. Preemption da altri task: 0 in entrambe le sessioni.
- **Unico gap lungo:** 152,8 ms alla prima STAND dopo il boot (A2, coppie OFF), cioè l'attesa della soglia ACC (registro 85) già spiegata offline. Riportato a parte e **non** tolto da alcun conteggio; il controllo resta invariato.
- **`istall_mem`:** resta il componente dominante in ogni classe (`analyze_diag`: DOMINANT), ma la sua quota scende di 4–10 punti.

### B.4 Memoria interna

| Misura | `8530d3a` | `e321e86` |
|---|---|---|
| `heap_free` a riposo dopo il boot, senza client | 79 436 B (oggi, freddo) | **74 836 B** (−4 600; atteso −4 608) |
| `heap_min_free` nella suite | 38 136 B (15/11) | **45 216 B** |
| `heap_free` minimo nella suite | 60 892 B | 63 480 B |
| Blocco massimo allocabile | 31 732 B | 31 732 B |
| Stack minimo loop / httpd | 6 224 / 9 972 B | 6 028 / 9 972 B |

- **Costo statico:** −4,6 KB, come previsto.
- **Minimo nella suite:** dipende dai picchi transitori dell'HTTP (il 15/11 tre esecuzioni e più client), quindi non è un confronto a parità di carico. Oggi è rimasto sopra 45 KB.
- **Blocco massimo** invariato: nessun segno di carenza. Non introduco soglie nuove.

### B.5 Regressioni

- **Funzionali:** nessuna. 65/65 passi, STOP mantenuti, riprese, cambi di modo, FAST 32, PRONE_REST.
- **DALY** OK in movimento, IMU valida, nessun fault.
- **Tre riprese dopo STOP** (D15 STAND, D19 WALK, F5 STAND) hanno 30/29/26 overrun contro 24/23/19, ma 0 perse contro 3/2/1. Sono release RECOVERY con NOMINAL e COMPENSATE pieni nella stessa release: il residuo strutturale.

### B.6 Residuo P0 e prossima decisione tecnica

- **Il §8.3 non è soddisfatto:** 4 perse operative, 323 overrun, picchi 25–36 ms. L'IRAM G1 da solo non può chiudere P0: anche annullando del tutto gli stalli d'istruzione resterebbero i picchi da NOMINAL + COMPENSATE nella stessa release (RECOVERY per progetto, 9 % delle release WALK).
- **Decisione proposta, in ordine:**
  1. **Pubblicare l'IRAM G1 operativo `f349af9`** sul branch operativo. È lo stesso cambiamento di `e321e86`, ora con beneficio misurato sul chip, heap verificato e nessuna regressione. Va fatto con un commit dedicato e senza flash nella stessa decisione.
  2. **Scegliere la semantica per i picchi** (rapporto offline del 15/11, A2): ripresa sulla griglia T2 oppure compensazione leggera mentre la correzione scorre. Da qualificare offline con i gate STOP prima di qualsiasi flash.
  3. **Estendere l'IRAM a G2** (+4,7 KB, simulato −51/−59 % sui miss) solo dopo il punto 2 e se il margine heap misurato lo consente.

## Stato finale e chiusura

- **Robot:** acceso, `e321e86` installato in app0, READY/IDLE `PRONE_REST`, SAFE_OFF 13/13 (8191) verificato, coppia 0 su 12, nessun fault. DALY OK, NVS gen-1 `VALID_ACKNOWLEDGED`. Stream BMS riportato a OFF (default al boot).
- **Acquisizioni:** registratori chiusi, porta USB libera, nessun processo attivo.
- **Rete dell'host:** il dongle è ancora sull'AP `MATDOG-227594`; Internet passa dal cavo. Per tornare alla rete domestica basta riattivare il suo profilo NetworkManager (nome omesso in questa copia pubblica).
- **Rollback a `8530d3a`:** pronto e verificato, non eseguito. Nessun altro flash o prova senza un nuovo mandato.

## File

- **Piano e strumenti:** `README.md`, `TOOLS.sha256`, `TOOL_DIFFS_vs_qualified.txt`, `cold_start.sh`, `monitor.py`, `usb_session.py`, `ctl.py`, `http_watch.py`, `preflight.py`, `fixture_suite.py`, `rom_entry.sh`, `boot_and_record.sh`, `analyze_diag.py`, `suite_summary.py`, `compare_steps.py`, `analyze_passive.py`.
- **Prima del flash:** `preflash/` (script pinnato e variante a secco, controlli negativi, identità del candidato, `image-info`, rollback).
- **Cattura a freddo:** `cold/` (USB D1 per poll, HTTP Fase B), `cold_*utc.txt`, `network_check.txt`, `nm_journal_session.txt`.
- **Preflight:** `pre/`.
- **Flash e boot:** `flashlog/`, `usb_boot/`.
- **Suite:** `suite_20261115_1220/` (`serial.log`, `http.jsonl`, `events.jsonl`, `results.json`, `analyze_diag.txt`), `suite_summary.txt`, `compare_steps_vs_20261115.txt`.
- **Stato finale:** `final_state_readonly.json`, `final_usb/`, `final_stream_off/`.
- **RCA DALY:** `bms_rca/` (`RCA_NOTES.md`, `hist_census.py`, `hist_census.tsv`).
- **Registro:** `WORKLOG.md`.
