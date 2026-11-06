# MATDOG dev.15 — Sessione hardware del 06/11/2026

**Stato: PARTIAL HARDWARE VALIDATION — non stable.** Questo report è una fotografia della sessione, non una certificazione di affidabilità continuativa. Le prove sono state condotte sull'ESP32-S3 reale tramite dashboard AP MATDOG, con log Motion raccolti dalla pagina e osservazioni dell'operatore. Le fonti primarie sono catalogate in [EVIDENCE_MANIFEST.md](EVIDENCE_MANIFEST.md), incluso il video analizzato in [VIDEO_AUDIT.md](VIDEO_AUDIT.md). I file grezzi sono **LOCAL_ONLY / PENDING PUBLICATION**; l'assenza dei grezzi da GitHub è dichiarata e non occultata.

## 1. Provenienza: distinguere dispositivo, sorgente e remoto

| Elemento | Stato al collaudo |
|---|---|
| **VERIFIED HARDWARE — firmware installato** | MATDOG Controller `1.0.1-dev.15`, build `7997ad4d4497`, riconosciuta dopo flash tramite HTTP |
| **Sorgente candidato locale (non pubblicato su remoto)** | commit `7997ad4`, branch locale `matdog/dev15-joystick-motion`, worktree ASUS `~/MATDOG/worktrees/matdog-dev15-joystick-motion-20261105`; associazione alla build da manifest/preflight locale |
| **APP candidata** | 1.691.328 byte, SHA-256 `b81d3a4ffaa27a7690b196e77e7174c0cc7851563fcfd64cd3bddb40e62f9e13` |
| **Flash verificato** | 06/11, APP-only su app0 attivo, offset `0x010000`; lettura otadata/layout, una scrittura ROM esptool `--no-stub`, `verify-flash` indipendente; nessuna scrittura NVS/otadata/bootloader |
| **Calibrazione/attività** | NVS gen-1 `VALID_ACKNOWLEDGED`, census 13/13, RUN/READY, IMU/BMS/SERVO/LED PASS al boot, SAFE_OFF iniziale 13/13; nessun restore necessario |
| **GitHub main** | lineage documentale basata su dev.7, **non** codice dev.15; dev.14 è storico con checkpoint `4ff2aadf4ed9`. Non confondere main, commit locale, binario installato o questo branch docs. |

L'operatore ha riportato un timeout DALY BMS intermittente su dev.14 prima del flash; dopo riavvio fisico e preflight BMS a 0,5 Hz per 30 s, il dato era fresco e il successivo boot dev.15 non ha mostrato fault BMS. **Causa UNKNOWN.** Rimane lo storico sfarfallamento policromatico LED nonostante il normale PASS della diagnostica.

## 2. Prove Motion: risultati effettivi

| Prova | Evidenza | Valutazione |
|---|---|---|
| STAND100 da PRONE_ENTRY | 11,81 s, `STAND_REACHED`, quota stimata 98,6 mm, tracking regolare | **VERIFIED HARDWARE** nella prova |
| STAND100 → STAND150 | 3,76 s, `STAND_REACHED`, quota stimata 148,9 mm | **VERIFIED HARDWARE** |
| WALK STANDARD45 | 5,575 cicli complessivi nel campione; operatore riferisce appoggi regolari, nessuna anomalia LF; WALK STOP → STAND in 7,79 s | **VERIFIED HARDWARE** nel perimetro osservato, prestazioni non qualificate |
| Joystick 3DOF avanti/indietro, strafe sinistra/destra, yaw entrambi i sensi e combinazioni | Movimento e proporzionalità osservati; stream di comandi BASE_VELOCITY/ACK nel log; rilascio → STAND in un episodio 8,378 s | **VERIFIED HARDWARE** funzionale, lento; mantenere deadman e arresto al rilascio |
| STAND150 dopo nuovo boot | 13,09 s, `STAND_REACHED`; WALK45 e STAND100 seguenti rifiutati `HTTP 503 MOTION_BUDGET_UNAVAILABLE` (102 e 101 ms) | Rifiuti **VERIFIED LOG**; comando non arrivato al Motion executor, nessuna inferenza su servo/IK |
| STAND100 da PRONE, WALK60, WALK STOP | STAND100 12,27 s; WALK60 avviato; WALK STOP → STAND 7,28 s | **VERIFIED HARDWARE** nel perimetro della sessione |
| STOP operativo | Arresto completato; su STAND/WALK `STOPPED_SUPPORT_UNVERIFIED` può mantenere coppia ON; STOP post-FAULT conferma `SAFE_OFF_VERIFIED_FAULT_LATCHED` | Due semantiche diverse, da non confondere |
| Stabilizzazione IMU su tavola inclinata ferma | richiesta/applicata massima osservata pitch ~14,0°/~13,5°; roll ~4,8°/~1,9°; limitazioni `WORKSPACE` e `SLEW` | **VERIFIED HARDWARE** risposta IMU; non dimostra livellamento assoluto |
| Digital Twin 3D | aggiornamento in tempo reale anche su manipolazione manuale; **Reset vista** funziona e ripristina vista frontale grafica +X, secondo prova operatore | **VERIFIED HARDWARE** per funzione UI indicata; nessun reset fisico BNO085 |

La compensazione è più reattiva secondo osservazione diretta, ma non è stata quantificata una latenza A/B rigorosa contro dev.14. Le grandezze `height_estimate_mm` sono stime basate su encoder+IMU e assunzione del suolo; non sono misure di altezza indipendenti.

## 3. PRONE: intermittente, con rischio di caduta

**Non promuovere PRONE terminale a funzione robusta.** I risultati nella stessa dev.15 includono:

| Precondizione | Esito | Evidenza |
|---|---|---|
| Da STAND ~120 mm dopo WALK/recupero | **ABORTED**, 18,88 s; arresto a circa 52,2 mm in `PRONE_HEIGHT` | log (15)/(16), `NATIVE_TARGET_NOT_REACHED`, successivo SAFE_OFF 13/13 |
| Da STAND150 | **ABORTED**, 20,96 s; arresto a circa 52,4 mm in `PRONE_HEIGHT` | log (18), `NATIVE_TARGET_NOT_REACHED`, SAFE_OFF 13/13 |
| Da STAND50 | **COMPLETED**, 9,266 s | log (22), `PRONE_REST`, coppia OFF 13/13; assestamento leggero descritto |
| Da STAND100 dopo il precedente PRONE | **COMPLETED**, 12,310 s | log (22), `PRONE_REST`, coppia OFF 13/13 |
| Dopo rotazione antioraria e ritorno STAND (prova successiva riferita dall'operatore) | **ABORTED, VERIFIED BY OPERATOR OBSERVATION ONLY**; sosta ~50 mm e rilascio anticipato; robot sostenuto a mano | **nessun JSON contemporaneo allegato**; causa e durata esatte UNKNOWN |

Nei primi due fallimenti il Controller ha tolto coppia in posa ancora sospesa (circa 50 mm), causando una caduta/necessità di sostegno. Il successivo STOP verifica SAFE_OFF ma non cancella il fault latched: recupero STAND non disponibile dalla dashboard in quel fault. Questo è un **difetto operativo di sicurezza fisica e recoverability** da trattare prima di altre escalation prestazionali.

**Prima divergenza verificabile:** il segmento `PRONE_HEIGHT → PRONE_LIFT` intorno a ~53 mm non avanza. In un fallimento residuo `LF_UPPER` ~14 tick, in un altro i residui osservati <=6 tick; il tracking da solo non identifica la causa. Confrontare sul **codice esatto della build** `arrived()`, `settledUnderLoad()`, freshness, deadline, contenzione workspace e reale condizione di transizione. Non aumentare indiscriminatamente il timeout o disabilitare collisioni. In entrambe le finestre dei PRONE falliti le release perse non aumentano durante la discesa; nessuna causalità dimostrata tra mancato completamento e mancata deadline Motion.

## 4. Timing: ostacolo critico alle prestazioni

Periodo richiesto **20 ms / 50 Hz**. Dalle acquisizioni:
- IDLE prima dei movimenti, log (8): **0 missed**, max lavoro ~4,93 ms.
- STAND100 log (9): **0 missed**, max lavoro 26,96 ms: picco sopra periodo anche senza missed conteggiate.
- WALK45 log (11): **+50 missed**, max lavoro 34,32 ms.
- Joystick avanti/indietro log (12): **+235 missed** nella finestra, max lavoro 48,58 ms.
- Strafe e combinazioni log (13): **+383 missed** nella finestra, max 48,76 ms.
- Yaw log (14): **+429 missed** nella finestra, max 49,59 ms.
- Joystick combinato log (20): **+345 missed** nella finestra, max 47,62 ms, max gap 49,43 ms.
- WALK60 e HOLD della sessione log (19): **+7 missed** nella finestra, max lavoro 36,71 ms; max COMPENSATE 31,24 ms.
- Log (17) STAND150 in successiva sessione: rifiuti HTTP 503 separati dal superamento della deadline; controller rimane operativamente pronto finché non entra in PRONE fallito.

Questi valori sono **delta per finestra / massimi cumulativi**, non somme indipendenti di tutte le acquisizioni. Le tracce hanno snapshot sovrapposti e contatori di uptime che ripartono a ogni reboot. `COMPENSATE` può consumare più dei 20 ms; serve attribuzione per release e prima divergenza nel firmware locale, non benchmark che elimini controlli fisici reali.

La UI può restituire `MOTION_BUDGET_UNAVAILABLE` in ~101–102 ms. Confrontare codice del percorso HTTP admission, lock e scheduling: un rifiuto lato trasporto NON è un insuccesso dell'attuazione. Registrati anche timeout/skips di polling, senza fault associati in tutti i casi.

## 5. Prestazioni dinamiche / osservazioni operatore

La locomozione è **molto lenta** pur risultando proporzionale. Il robot compie trasferimenti laterali di peso marcati e swing delle anche ampi; percorre distanze modeste per ciclo. All'inizio dell'avanzamento è osservato un transitorio di arretramento/carico. In rotazione la cinematica appare più efficace, ma ancora lenta. Il rilascio del joystick funziona e deve restare immediato come comando di arresto; **`WALK_SETTLE → STAND` attualmente richiede circa 7,3–8,7 s** in alcune misure. Da non confondere l'arresto del comando con il termine del riassetto posturale.

Nel video 4:38,9 sono visibili avanti, indietro, strafe, yaw inverso e diagonale; la componente ciclica laterale del marker sul dorso ha periodo ~**6,5 s / 0,154 Hz**, ampiezza visiva **29–32 pixel** nelle finestre analizzate. Pixel, cicli apparenti e prospettiva **non equivalgono** a mm, velocità reale, CoM o periodo interno esatto del gait. Il video **non** contiene il PRONE fallito successivamente descritto.

I limiti *di comando* dev.15 riferiti nella sessione sono circa longitudinali 6,9 mm/s, laterali 3,1 mm/s e yaw 1,76°/s; questi **non sono misure di velocità al suolo**. Non dedurre da essi il limite effettivo dei ST3215. Il rumore/vibrazione storica della LF sotto fixture non è stato osservato a terra nella nuova sessione: **non dichiarare risolta la sua RCA**.

## 6. Altre anomalie non chiuse

- Riletture termiche isolate su alcuni ST3215 (incl. ID21/ID31) con variazioni improvvise, valori periodici usuali ~26–34°C; distinguere protocollo/feedback da riscaldamento fisico. Le protezioni termiche restano invariate; eventi `OK` non sono prova definitiva di assenza di guasto.
- BMS dev.14 `BMS_TELEMETRY_STALE` pre-flash, rientrato dopo reboot; nessuna causa verificata.
- LED occasionalmente sfarfallano in colori differenti, anche se `LED PASS`.
- Prova HostLink USB dal Linux bridge: flash e connessione USB no-reset verificati, ma `@MOTION STATUS`/invio `@MOTION STAND` non qualificati come controllo Motion hardware in questa sessione; helper interrotto da BrokenPipe. La Jetson **non è installata**.

## 7. Decisioni per la prossima candidata

Ordine consigliato: **(1)** correggere PRONE fail/recovery e impedire cadute da quota intermedia, senza cancellare i controlli reali; **(2)** RCA e patch minima di timing (`COMPENSATE`, 20 ms, priorità STOP, deadman e admission HTTP); **(3)** misurare/ottimizzare gait, sway, tempi settle e limiti cinematici realmente disponibili; **(4)** ampliare ROLL entro workspace certificato; **(5)** validare HostLink/Jetson dopo che Motion sia davvero affidabile.

Obiettivo richiesto dall'operatore: un miglioramento **anche fino a 5×** della velocità utile rispetto alla dev.15, **come target di sviluppo non ancora dimostrato**. Serve prima una baseline misurata a terra (distanza/tempo, stabilità, tracking, corrente, temperature, timing) e test progressivi coerenti con i limiti reali; evitare un tuning che mascheri il problema da 20 ms.

**Non modificati da questo report:** codice, firmware installato, attuatori, calibrazione, geometria, limiti, NVS, layout flash, release o tag.
