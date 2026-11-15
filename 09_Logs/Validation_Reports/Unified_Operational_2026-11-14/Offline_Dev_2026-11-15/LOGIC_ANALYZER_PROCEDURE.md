# DALY: acquisizione passiva sul filo durante un guasto (procedura)

**Scopo.** Distinguere in modo definitivo le classi di guasto 1–4 dell'RCA (`REPORT.md` §3). Oggi il firmware vede una cosa sola: per 440–576 s non arriva nessun byte sull'RX di UART2, mentre la FIFO TX si svuota. Quella osservazione non distingue «richiesta non partita», «richiesta non arrivata», «BMS muto» e «risposta persa prima del pin».

**Vincoli.**
- Solo sonde passive. Nessuna trasmissione aggiunta e nessun comando BMS, KEY o MOS.
- Niente app Bluetooth Smart BMS.
- Robot fermo in PRONE_REST, nessun Motion.
- Nessun flash, reset o modifica del firmware installato.
- Il ricevitore RS485 di ascolto deve essere ad alta impedenza, con DE e RE fissi in ricezione. Non deve caricare né terminare il bus.

## Strumenti

| Canale | Punto | Cosa vede |
|---|---|---|
| CH0 | GPIO15 dell'ESP32-S3, cioè TXD verso l'XY-017 | la richiesta come lascia il pin |
| CH1 | GPIO16 dell'ESP32-S3, cioè RXD dall'XY-017 | la risposta come arriva al pin |
| CH2 | uscita RO di un secondo transceiver RS485 in sola ricezione, su A/B | tutto ciò che passa sul bus: richiesta e risposta |
| CH3 (opzionale) | pin DE/RE dell'XY-017, se accessibile, o il lato UART del DALY | la direzione del bridge |

- **Analizzatore:** almeno 4 canali, soglia 3,3 V, almeno 1 MS/s in streaming continuo per 20 minuti o più (per esempio sigrok/PulseView con un fx2lafw a 1–2 MS/s). A 9600 baud un bit dura 104 µs.
- **Massa:** comune all'ESP32, sul ritorno logico e non sul polo di potenza della batteria.
- **In parallelo:** USB in sola lettura con `@BMS STREAM ON` (una riga D1 ogni 2 s, nessuna transazione in più), oppure HTTP `/status` ogni 5 s.

## Sequenza

1. **Pronto prima dell'accensione:** analizzatore collegato e in acquisizione, robot spento (KEY OFF). Il guasto inizia 2–5 min dopo un'accensione a freddo (14/11: 165 s; 15/11: 303 s) oppure già al boot dopo un flash.
2. **Accensione:** KEY ON secondo la pratica dell'operatore, robot fermo.
3. **Fase A, 0–10 min:** AP attivo, nessun client HTTP. Solo l'USB in sola lettura.
4. **Fase B, 10–20 min:** un client HTTP carica la pagina `/` (115 KB) ogni 30 s, con `/api/motion/status` a 1 Hz. Alla sessione successiva invertire A e B.
5. **Fine:** a guasto rientrato, o dopo 20 min senza guasto. Spegnere secondo procedura.
6. **Ripetizione:** se non compare nessun guasto, ripetere il ciclo di accensione. Dal 06/11 si sono registrati circa 5 guasti in 10 giorni.

## Lettura

Decodifica UART 9600 8N1 su CH0, CH1 e CH2. Nella finestra in cui D1 mostra `TIMEOUT rx=0`:

| Osservazione | Classe | Conclusione |
|---|---|---|
| CH0 senza frame negli istanti di poll (ogni 2 s) | 1 | difetto lato ESP32: pin, GPIO matrix o UART |
| Frame su CH0, CH2 muto | 2 | il bridge XY-017 non trasmette sul bus: alimentazione, direzione o cablaggio |
| Richiesta da 8 B su CH2, nessuna risposta su CH2 | 3 | il DALY non risponde: lato BMS |
| Risposta da 129 B su CH2 assente o troncata su CH1 | 4 | ricevitore o direzione dell'XY-017, oppure filo verso GPIO16 |
| Su CH2 raffiche o frame spuri che coincidono con il traffico Wi-Fi o con l'inizio del guasto | H2 | accoppiamento elettrico o RF |

**Rientro con 128/129 byte.** Contare i byte su CH2 e su CH1 e vedere quale manca (primo o ultimo) e dove si perde: già sul bus o solo al pin.

**Criterio di chiusura.** Almeno un guasto acquisito con tutti e tre i canali principali. Il risultato attribuisce il guasto a una sola classe e decide se una correzione firmware ha senso (solo per la classe 1) o se l'intervento è elettrico o lato BMS.
