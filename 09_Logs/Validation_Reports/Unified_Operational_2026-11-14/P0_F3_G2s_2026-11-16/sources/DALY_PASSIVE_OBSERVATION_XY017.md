# DALY — osservazione passiva dell'XY-017 alla prossima accensione (preparata il 16/11/2026)

Mandato dell'operatore del 16/11 sera: la RCA DALY resta UNKNOWN. Nessuna modifica a protocollo, UART2, timeout, MOS, KEY o BMS. Solo osservazione passiva, alla prossima accensione comunque necessaria (non un ciclo dedicato).

## Precauzione temporanea per il prossimo collaudo

- Motion, flash e prove a terra solo dopo **QON + almeno 6 minuti e 150 poll DALY consecutivi validi** (`bms_gate.py`), oltre ai controlli di sicurezza esistenti.
- Motivo: i 10 silenzi registrati sono iniziati entro ≈ 335 s dal QON (`MATDOG_1_0_2_DIAG_A86C680_HW_20261116/daly_rca/REPORT_RCA_DALY.md`). Il gate di 150 poll contati dal boot può chiudersi a ≈ 300 s.
- È un'attesa di procedura, non un vincolo del firmware operativo: nessun codice cambia.

## Prima del QON (robot spento)

- Senza smontare, toccare cavi o connettori: guardare se il modulo XY-017 (ponte TTL–RS485 a direzione automatica fra ESP32 e DALY) ha LED.
- Annotare quali: alimentazione, TX, RX (le serigrafie possono dire `TXD`/`RXD`, `PWR`).
- Se non ci sono LED, l'osservazione si riduce alla registrazione D1/D2 come il 15–16/11. La misura decisiva (ricevitore RS485 in solo ascolto su A/B) richiede un mandato separato.

## Durante l'accensione

1. Avviare il registratore dal QON: `MATDOG_COLD_PHASE_MIN=5 ./cold_start.sh cold` (D1 + D2, stesso strumento del 16/11).
2. **Per i primi 6 minuti** guardare i LED, se presenti:
   - di norma, ogni 2 s, un lampo TX (richiesta da 8 byte) e subito dopo un lampo RX (≈ 130 ms di risposta);
   - annotare l'ora UTC di ogni anomalia.
3. Se compare un silenzio (D1 `fail_streak` > 0, D2 `RX_IDLE_HIGH`), osservare per tutta la sua durata:

| Osservazione | Significato | Ipotesi favorita |
|---|---|---|
| TX lampeggia ogni 2 s, RX spento | le richieste arrivano al bus, manca la risposta | H1 DALY o H3 bus |
| TX spento | il guasto è fra il pin TX dell'ESP e l'XY-017 | H2 XY-017 (o lato ESP) |
| TX acceso fisso, o RX acceso fisso | direzione o ricevitore bloccati | H2 XY-017 |
| PWR spento o intermittente | alimentazione del modulo | H2/H3 alimentazione |

4. Non fare nulla per «sbloccare»: nessun reset, nessuna riaccensione, nessuna app BMS, salvo decisione dell'operatore. Registrare fino al rientro spontaneo.
5. Dopo il rientro, il gate riparte da zero: 150 poll consecutivi e QON + 6 min.

## Registrazione

- Nel WORKLOG della sessione: presenza dei LED, orari, sequenza osservata, fotografia o video se l'operatore lo ritiene innocuo.
- Allegare `cold/` (D1/D2) e l'analisi `analyze_d2.py`.
