# Episodio DALY del 16/11/2026, 11:51:24Z (12:51 CET) — analisi in sola lettura

Firmware installato al momento: `82c1af95e0a5` (D1 + D2), boot a freddo con QON, robot in PRONE_REST a coppia OFF, nessun comando Motion. Fonti:
- `../cold/serial.log`: registratore USB passivo dal QON, `@BMS STREAM ON`, un blocco D1+D2 ogni 2 s;
- `daly_timeline.tsv`, `d2_by_phase.txt`, `episode_timeline.txt` (generati da `daly_timeline.py`);
- journal dell'host (USB e Wi-Fi).

Nessun firmware, timeout, polling, protezione, hardware o parametro BMS modificato. Nessun comando al BMS.

## 1. Classificazione D2

Classificatore esistente (`../analyze_d2.py`, classi fissate il 16/11 mattina):
- **1 periodo PARTIAL `RX_ACTIVITY`** (14 byte su 129);
- **63 periodi SILENT `RX_IDLE_HIGH`**: RX a 1 per tutto il periodo, 0 fronti, FIFO RX sempre vuota, `tx_low_idle` 0;
- nessun `RX_HELD_LOW`, `FIFO_UNREAD` o `TX_LOW_IDLE`.

## 2. Sequenza temporale

Uptime del dispositivo: zero a 11:49:58.639Z sul clock dell'host (stima da 199 poll OK, dispersione 9 ms). Una richiesta ogni 2 s, scadenza 750 ms.

| Evento | UTC | Uptime |
|---|---|---|
| Ultimo poll valido (129 byte, primo byte 16 ms) | chiuso 11:51:19.873 | 81,23 s |
| **Primo frame parziale:** richiesta, primo byte dopo 15 ms, 14 byte, poi linea ferma | ≈ 11:51:21.715 → ≈ 11:51:21.745 | ≈ 83,08 → 83,11 s |
| Primo poll fallito (scadenza, TIMEOUT con 14 byte) | 11:51:22.465 | 83,83 s |
| Inizio del silenzio sul filo: primo poll con 0 byte | richiesta ≈ 11:51:23.71, chiuso 11:51:24.462 | 85,82 s |
| Ultimo poll fallito (65°, non stampato: chiuso 90 ms dopo la stampa precedente) | ≈ 11:53:30.51 | ≈ 211,9 s |
| **Primo frame valido:** 129 byte, primo byte 21 ms | richiesta ≈ 11:53:31.77, chiuso 11:53:31.927 | 213,29 s |
| Poll consecutivi ristabiliti: 150° consecutivo OK (gate) | 11:58:31 | ≈ 513 s |

**Durate:**
- senza telemetria valida: **132,1 s** (81,23 → 213,29 s);
- linea RX ferma: ≈ **130,0 s** (fine del frame parziale → primo byte del rientro);
- stato FAULT (LED rosso): **129,5 s** (primo TIMEOUT → primo OK). L'operatore ha osservato circa 120 s.
- Poll falliti: **65** (1 parziale + 64 a 0 byte).

## 3. D1 (driver, per poll)

- Byte ricevuti: 129 in tutti i poll prima; **14** nel poll d'inizio; **0** nei 64 successivi; 129 dal rientro.
- Ultimo frame completo prima del silenzio: 129 byte, CRC valido (`comm=OK`).
- Errori: solo TIMEOUT (il parziale è TIMEOUT perché incompleto alla scadenza). `stale_bytes` 0 in tutti i poll: nessun byte arrivato dopo una scadenza, quindi nessuna risposta in ritardo.
- Latenza del primo byte: prima mediana 18 ms (10–21), parziale 15 ms, dopo il rientro mediana 17 ms (11–23).
- Richieste: `fail_streak` sale di 1 a ogni chiusura, ogni 2 s, da 1 a 64 nei blocchi stampati più il 65° non stampato. Il driver ha aperto e chiuso una transazione a ogni periodo. Non c'è un contatore di byte trasmessi.
- SOC 58,7 % prima, 58,5 % al rientro, corrente −0,3 A: continuo; allarmi 0000.

## 4. D2 (status register di UART2, lato ESP dell'XY-017)

| Fase | Periodi | Campioni | RX basso (mediana [min–max]) | Fronti RX | TX basso a riposo | FIFO RX non vuota |
|---|---|---|---|---|---|---|
| Prima (boot → 11:51:22) | 41 | 877 | 45 [0–51] | 26 [0–34] | 0 | 0 |
| Periodo del parziale | 1 | 875 | 4 | 2 | 0 | 0 |
| Silenzio | 64 | 878 | **0** | **0** | 0 | 0 |
| Dopo il rientro (→ 11:58:46) | 157 | 864 | 46 [39–55] | 26 [12–34] | 0 | 0 |

- Il minimo 0 della fase «prima» è il periodo che precede la prima risposta dopo il boot.
- Il periodo del parziale (4 campioni bassi, 2 fronti) è proporzionale a 14 byte su 129 (≈ 4,9 e 2,8 attesi): la linea ha portato esattamente quei byte, poi niente.
- D2 legge livelli sostenuti a frequenza del loop (~440 Hz). Non vede impulsi brevi né il bus RS485 oltre l'XY-017.

## 5. L'ESP ha continuato a trasmettere le richieste?

- **Registrato:**
  - una transazione aperta e chiusa ogni 2 s (`fail_streak`);
  - TX a 1 in tutti i campioni a riposo (`tx_low_idle` 0);
  - campionamento del loop regolare (~875 campioni per periodo): il loop non si è fermato.
- **Dedotto dal codice** (core arduino-esp32 3.3.11): `HardwareSerial(2)` non ha buffer TX, quindi `availableForWrite()` è lo spazio libero nella FIFO TX hardware (128 byte). `DalyBms::update()` avvia una richiesta solo se c'è posto per 8 byte. Se UART2 avesse smesso di trasmettere, dopo ~16 richieste la FIFO sarebbe stata piena e `fail_streak` si sarebbe fermato. È salito fino a 65, quindi **UART2 ha svuotato la FIFO a ogni richiesta**: le richieste sono uscite dal pin TX di UART2.
- **Non registrato:** il segnale sul pin TX durante la richiesta (D2 non lo conta), il passaggio nell'XY-017 e sul bus RS485, la ricezione da parte del DALY.

## 6. Prima divergenza e attribuzione

**Prima divergenza osservabile:** il poll delle ≈ 11:51:21.715Z (uptime 83,08 s).
- La risposta parte puntuale (primo byte a 15 ms, come i poll validi) e si interrompe dopo 14 byte, ≈ 30 ms dopo la richiesta. Poi il pin RX di UART2 resta a riposo per ≈ 130 s.
- Nessuna anomalia sul lato ESP prima o durante: poll precedente valido con latenza normale, FIFO vuota, TX a riposo regolare, loop regolare.

| Elemento | Esito |
|---|---|
| Firmware (driver DALY) | **Non è l'origine della divergenza:** ha continuato a interrogare ogni 2 s, non ha perso byte (FIFO vuota, nessun byte tardivo), il rientro è avvenuto senza reinizializzazioni (VERIFIED HARDWARE) |
| UART2 ricezione | **Non ha perso dati:** sul pin non c'era segnale (0 fronti) (VERIFIED HARDWARE) |
| UART2 trasmissione | Ha trasmesso le richieste (dedotto dalla FIFO, §5) |
| XY-017 | **Non distinguibile:** la sua uscita verso l'ESP era ferma. Può non aver ricevuto nulla dal bus, o aver smesso di pilotare l'uscita (anche la direzione automatica bloccata darebbe questo quadro). Il passaggio delle richieste verso il bus non è registrato |
| Bus RS485 e DALY | **Non distinguibili** dall'XY-017. Il SOC continuo indica che il nucleo del BMS non si è riavviato (deduzione) |

## 7. Sequenza USB

| UTC | Evento |
|---|---|
| 11:47:11.214 | registratore armato, in attesa della porta |
| 11:49:57.194 | enumerazione USB (kernel: «USB JTAG/serial debug unit»); 11:49:57.221 `ttyACM0` |
| 11:49:57.829 | link by-id rilevato; logger avviato subito |
| 11:49:58.591 | primo comando inviato (`@BMS STREAM ON`): il firmware è ancora in avvio e lo perde |
| ≈ 11:49:58.639 | uptime 0 dell'applicazione |
| 11:49:58.6–11:50:01.7 | 10 letture di stato (perse in gran parte) |
| 11:50:02.3, 11:50:34.1, 11:51:05.9 | rilettura periodica del monitor (stato, LED, IMU, Wi-Fi, stream BMS) |
| 11:51:08.685–11:51:12.427 | 4 letture manuali (firma, persistenza, IMU, modo) |
| 11:51:16.9–11:51:17.3 | associazione del PC all'AP del robot, handshake, DHCP |
| 11:51:19.72 | richiesta: risposta valida |
| **≈ 11:51:21.715** | **frame parziale** |
| 11:51:37.7 | rilettura successiva del monitor (già nel silenzio) |

- Il logger era aperto da 84 s.
- L'ultimo comando USB precede la troncatura di **9,3 s**, l'associazione Wi-Fi di **4,4 s**, con un poll valido in mezzo.
- Il cavo USB è sempre collegato: l'enumerazione compare a ogni accensione, anche in quelle senza silenzio.

## 8. Confronto con gli altri avvii

| Avvio | Logger | Associazione Wi-Fi (dopo il boot) | Silenzio |
|---|---|---|---|
| 08/11 dev.3 | **nessuna USB aperta** | — | inizio ≈ 9 s |
| 14/11 dev.7 13:48:30 CET | porta aperta per la prima volta ≈ 3 s prima dell'inizio; chiusa durante il rientro | nessuna nella finestra | inizio 165 s, 9,6 min, rientro a porta chiusa |
| 15/11 `8530d3a` 08:32:37 CET | aperto 4 min dopo il boot, ≈ 64 s prima dell'inizio | 60 s (con disconnessione e riconnessione) | inizio ≈ 305 s, 440 s |
| 15/11 `e321e86` 12:52:03 CET | dal boot | **6,5 min: dopo l'inizio di entrambi** | inizio ≈ 81 s (98 byte), 218 s; poi ≈ 303 s (61 byte), 11 min |
| 16/11 `e321e86` 07:48:52 CET | dal boot | 11 min | nessuno (601/601 in 20 min) |
| 16/11 `46b3175` 09:04:52 CET (B8) | dal boot | 17 s | nessuno (601/601) |
| 16/11 `46b3175` 10:12:38 CET | dal boot | 102 s | nessuno (gate 152/0) |
| **16/11 `82c1af9` 12:49:57 CET** | **dal boot** | **80 s** | **inizio ≈ 83 s (14 byte), 130 s** |
| Boot dopo flash 16/11 (08:10, 10:19, 13:04 CET) | dopo 1–2 s | 15–33 s | nessuno |

- **Logger:** presente dal boot in 3 avvii senza silenzio e in 2 con silenzio; assente in un silenzio (08/11). Non è né necessario né sufficiente, quindi non è una differenza ricorrente.
- **Associazione Wi-Fi:** il 15/11 pomeriggio entrambi i silenzi iniziano senza alcun client associato. Non è necessaria.
- **Regolarità osservata, non spiegata:** gli inizi con D1 cadono a ≈ 81–83 s (15/11 pomeriggio, oggi) e ≈ 303–305 s (15/11 mattina e pomeriggio), sempre a un poll di distanza tra loro. Il 14/11 invece a 165 s. Campione piccolo: è un indizio da verificare, non una causa.
- Nei tre poll d'inizio registrati (15/11 pomeriggio ×2, oggi) la risposta è tagliata a metà (98, 61, 14 byte). Il 15/11 mattina il poll d'inizio non è campionato (HTTP ogni 5 s, primo campione a `fail_streak` 15); il rientro comincia con un poll da 128 byte.

## 9. Stato dopo il rientro (sola lettura)

- **Boot originale:** 259 campioni (blocchi USB + `/status` HTTP) dalle 11:53:32 alle 12:02:47, **0 non OK**, intervallo massimo 8,6 s; poi l'ingresso in ROM per il flash alle 12:03:07.
- **Boot dopo il flash:** 156 blocchi USB OK e tutti i campioni HTTP OK fino alle 12:20.
- **Lettura alle 12:27:55Z:** `bms_comm=OK`, 129 byte, primo byte 14 ms, `fail_streak` 0, allarmi 0, SOC 55,8 %; controller READY, PRONE_ENTRY, SAFE_OFF 8191 verificato, coppie OFF, nessun fault.

## 10. Cause

- **Dimostrata:** nessuna.
- **Concausa dimostrata:** nessuna.
- **Ipotesi aperte**, tutte a monte del pin RX di UART2:
  - il trasmettitore RS485 del DALY (o il suo modulo di comunicazione) si ferma a metà frame e riprende dopo 2–11 min;
  - l'XY-017 smette di passare la ricezione (direzione automatica bloccata o alimentazione);
  - un'interruzione del collegamento RS485;
  - un evento temporizzato comune a ≈ 82 s e ≈ 304 s dall'accensione (§8).
- **Escluse come necessarie:** il logger USB e l'associazione Wi-Fi. La causa resta **UNKNOWN**; la comunicazione intermittente non è risolta.

## 11. Prossimo controllo minimo

Senza modifiche, al prossimo avvio a freddo che avvenga comunque per un motivo diagnostico o operativo (non un ciclo dedicato), con il registratore D1/D2 dal QON. Se compare un silenzio, guardare durante la finestra:
1. **I LED TX/RX dell'XY-017, se il modulo li ha:**
   - TX che lampeggia ogni 2 s con RX spento: le richieste arrivano al bus e manca la risposta (lato bus/DALY);
   - TX spento: il guasto è fra il pin TX dell'ESP e l'XY-017.
2. **Solo se l'operatore lo ritiene innocuo,** l'app Smart BMS in sola visualizzazione via Bluetooth: dati vivi indicano un nucleo del BMS attivo con la porta RS485 muta. Il 14/11 l'app è stata aperta durante un silenzio e non è noto se lo abbia prolungato.

Una misura elettrica sul bus o una diagnostica TX nel firmware sarebbero più dirette, ma richiedono un mandato esplicito.
