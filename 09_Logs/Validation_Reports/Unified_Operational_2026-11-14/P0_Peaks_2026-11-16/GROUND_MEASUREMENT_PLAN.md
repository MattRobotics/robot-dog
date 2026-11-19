# Fase a terra — piano minimo di misure reali (preparato il 16/11/2026, non eseguito)

Scopo: sostituire con misure le ipotesi su cui oggi si giudicano FAST 40, CRAWL e TROT.

Vincoli del piano:
- nessuna modifica hardware;
- nessun firmware nuovo: build installata `46b3175`, oppure la diagnostica successiva se nel frattempo viene autorizzata;
- nessun limite, box o criterio cambiato;
- solo comandi già qualificati sulla fixture;
- ogni prova a terra richiede un mandato dedicato e l'operatore presente con STOP accessibile.

## 0. Quattro limiti da tenere distinti

FAST 40 e TROT **non** vanno trattati come impossibilità fisiche assolute: oggi sono non dimostrati o non certificati per una combinazione dei limiti seguenti.

| Limite | Che cosa è | Stato oggi | Effetto su FAST 40 / TROT |
|---|---|---|---|
| **A. Meccanico effettivo** | limiti URDF dei giunti, limiti RAW dei servo, collisioni reali tra le parti | margine 550–900 mrad dalle traiettorie attuali (rig host, `workspace_use_e1b.txt`); le collisioni reali oltre i box non sono mai state misurate | non vincolante per i giunti; per le collisioni UNKNOWN fuori dai box |
| **B. Box software certificati** | intervalli di giunto indipendenti ricavati da CAD/FCL (`kHeight`, `kSteer*`), prova STOP dentro il box | vincolanti: 31–104 mrad di margine sterzato; FAST 40 esce dal box nella prova STOP a ACC 84 (offline, 396 configurazioni) | FAST 40 **non certificabile** con questi box: è un limite di certificazione, non una prova fisica |
| **C. Servo sotto carico** | velocità, decelerazione e coppia reali degli ST3215 a 11,4 V, cedevolezza, attrito, ritardo di inseguimento | decelerazione di frenata qualificata ACC 84 (12,9 rad/s²); C condivisa ~0,036 rad/N·m (parziale); C per giunto, attrito, ritardo dinamico e curva di coppia **UNKNOWN** | decide se una frenata più alta è reale (FAST 40) e se esiste la banda per un appoggio dinamico (TROT) |
| **D. Legge di gait** | quattro tempi, duty 0,80–0,825, sway, clearance comandata, prova di supporto statico | velocità dello swing v/(1−d) = 229 mm/s a 40 mm/s; supporto 1,7–2,1 mm; clearance effettiva ~1 mm contro 6–8 mm comandati (gemello) | una legge diversa (duty, swing, compensazione della cedevolezza) cambia il vincolo D, non A–C |

**Conclusione operativa:**
- FAST 40 diventa valutabile quando sono misurati la frenata reale sotto carico (C), la cedevolezza (C) e la clearance (D), e quando la traiettoria è certificata sul CAD invece che con box per giunto (B).
- TROT richiede in più il ritardo di inseguimento sotto carico (C). Il gemello (T3, 13/11) non basta né a escluderlo né ad ammetterlo, perché il modello attuatore non è identificato dinamicamente.

## 1. Misure, in ordine di necessità

| # | Dato | Limite | Come, con quello che esiste | Durata | Rischio |
|---|---|---|---|---|---|
| M1 | **Cedevolezza statica per giunto** C_HIP, C_UPPER, C_LOWER e **attrito statico** | C | Robot a terra, STAND 80 / 120 / 150 mm, HOLD 30 s ciascuno. Per ogni quota, `GROUND_CONTACT.OBSERVE` su LF, RF, LH, RH (passivo, nessun movimento): angoli misurati contro riferimento, PresentLoad, Jacobiano di contatto. Ogni quota raggiunta sia salendo sia scendendo (150 → 120, 80 → 120): la differenza dell'errore statico tra i due versi limita l'attrito | ~15 min | basso: solo STAND e HOLD già qualificati, nessun cammino |
| M2 | **Velocità reale al suolo** per SLOW, STANDARD, FAST 32: avanti, indietro, laterale, rotazione | D (+C) | Cammini da 30 s con i preset e il joystick già qualificati. Spostamento misurato con un metro sul pavimento (segni di partenza e arrivo) e video dall'alto con riferimento sul pavimento (metodo del 13/11); nessun sensore sul robot | ~25 min | medio: cammino a terra già fatto l'11/11 (dev.7); STOP accessibile |
| M3 | **Clearance reale del piede** in swing | D/C | Video laterale a 60 fps con riferimento millimetrico vicino alla zampa, durante M2 (SLOW e FAST) | incluso in M2 | come M2 |
| M4 | **Frenata reale sotto carico** | C/B | Analisi dei 4–6 STOP a terra fatti durante M2: durata della verifica e posa tenuta dalla telemetria HTTP, più il video. Niente STOP a ACC > 84: la frenata qualificata resta quella | incluso in M2 | come M2 |
| M5 | **Ritardo di inseguimento dinamico** per giunto | C | Oggi **non misurabile** con la telemetria esistente: `commanded_raw` arriva con ~0,7 s di età e non c'è un flusso sincrono ≥ 50 Hz. Serve una diagnostica read-only (riferimento e posizione per giunto a 50 Hz, solo durante WALK), da progettare e qualificare offline prima, con un mandato a parte | — | — |
| M6 | **Attrito piede–pavimento** | D | Robot in PRONE_REST su una tavola inclinabile, angolo di primo scivolamento (coppie OFF, robot passivo); in alternativa un dinamometro a molla con il robot in STAND fermo | ~5 min | basso |

## 2. Ordine e criteri

1. **M1** per primo: è il dato bloccante comune a V7, CRAWL e gate, e non richiede cammino.
2. **M2 + M3 + M4** nella stessa sessione, con i soli modi e preset già qualificati (FAST 32, non FAST 40).
3. **M6** a robot passivo.
4. **M5** solo dopo una diagnostica dedicata, qualificata offline.

**Criteri di arresto:** quelli della suite fixture (FAULT, BMS perso, stato inatteso, temperatura servo ≥ 65 °C, timeout), più l'operatore che preme STOP a qualunque contatto anomalo. Nessuna ripresa automatica.

**Uscite attese:**
- M1: C per giunto con intervallo d'incertezza e attrito limitato. Sostituisce il valore nominale 0,0336 del gemello.
- M2/M3: velocità realizzata e clearance reali, da confrontare con il gemello (49–68 % in avanti, 1–1,6 mm).
- M4: margine reale della frenata.

**Che cosa sbloccano:**
- M1 + M3 → la compensazione della cedevolezza (V7) e la decisione sui gate rigidi.
- M1 + M4 + certificazione CAD della traiettoria → la rivalutazione di FAST 40.
- M5 → la valutazione di un TROT lento.
