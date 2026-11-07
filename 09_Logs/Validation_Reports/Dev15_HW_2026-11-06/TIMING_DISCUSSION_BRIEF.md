# MATDOG — Timing Motion a 20 ms: sintesi per un confronto tecnico

**Destinatario:** Emanuele Colombo, interlocutore volontario esterno: suggerimenti, non sviluppo né accesso al repository. **Data:** 07/11/2026. **Base:** firmware realmente installato `1.0.1-dev.15` (build `7997ad4d4497`) e tracce del 06/11/2026. Analisi completa: `TIMING_RCA_2026-11-07.md`, stessa cartella.

## Architettura essenziale

- Quadrupede, 12 servo ST3215 su un bus seriale half-duplex (SYNC_WRITE delle 12 posizioni, letture di feedback per singolo servo), IMU BNO085, BMS DALY.
- **ESP32-S3 a 240 MHz.** Tutto il Motion gira nel `loop()` Arduino sul **core 1** (priorità 1). Wi-Fi, HTTP server, IMU e rete sono fissati sul core 0. La Dashboard nel browser comanda via HTTP; le richieste vengono comunque **servite dal loop del core 1**, una per giro.
- Release Motion periodica a **20 ms (50 Hz)** su griglia assoluta: se una release finisce tardi, i periodi interi saltati vengono contati come persi.
- Ogni release, in WALK:
  1. lettura di 2 servo (+1 lettura estesa ogni 6 release);
  2. IK nominale delle 4 zampe dalla legge del gait;
  3. correzione IMU: fit dell'assetto da FK di 3 zampe in appoggio, poi **IK inclinata delle 4 zampe** (8 risoluzioni con il contatto sul raccordo del piede, nel joystick);
  4. prove di sicurezza: limiti, box articolari certificate (convesse), conversione RAW, prova di arresto (StopPlan);
  5. SYNC_WRITE.
- Tutta la matematica è in **`double`**. L'ESP32-S3 ha FPU solo in singola precisione: le operazioni double sono routine software in ROM. Build `-Os`; codice eseguito da flash tramite I-cache da 16 KB condivisa tra i core.

## Valori misurati sul robot

| Misura | Valore |
|---|---|
| IDLE (solo I/O) | ≤ 4,9 ms per release |
| I/O servo per release (lettura + scrittura) | ~2,8 ms (lettura 1,5–4,1 ms, SYNC_WRITE ~1,2 ms) |
| WALK 45 mm: release più veloce / tipica / peggiore | **18,4 / ~24 / 34,3 ms** |
| Joystick (8 IK): più veloce / peggiore | **28,2 / 49,6 ms** |
| HOLD fermo con correzione IMU, **stesso lavoro contato** | da 7,4 a 33,1 ms di sola compensazione (×4,5) |
| Frequenza effettiva in WALK | 45,1 Hz (4,9 periodi persi/s); HOLD 49,4 Hz; altre fasi 50 Hz |
| Comandi rifiutati `MOTION_BUDGET_UNAVAILABLE` | 3, dopo 101–102 ms |
| Ritorno WALK STOP → STAND | 7,3–8,7 s, dovuto alla legge del gait, non al timing |

Lo stesso codice sul PC (double hardware) impiega 8 µs in HOLD, 21 µs in WALK e 23 µs nel joystick. Il minimo sul robot è ~730–1100 volte tanto, con rapporto quasi costante tra scenari diversi. Sul PC i contatori di lavoro coincidono con quelli del robot.

## Dove vanno i 20 ms (WALK, release tipica ~24 ms)

| Voce | ms | Note |
|---|---:|---|
| Letture servo | ~1,6 | entro budget |
| IK nominale gait (4 zampe) | ~9,4 | ~42 % del calcolo sul PC |
| Correzione IMU (fit + IK inclinata + prove di sicurezza) | ~12,2 | IK inclinata ~29 %, prova d'arresto ~5 % sul PC |
| Conversione RAW + SYNC_WRITE + resto | ~2,2 | |

**Colli di bottiglia dimostrati:**
1. Il calcolo per release supera 20 ms già nel caso migliore del joystick.
2. Una versione precedente (dev.7) risolveva la correzione IMU a release alterne e su hardware non perdeva periodi. Il meccanismo è stato rimosso il 05/11, quando sono stati aggiunti il fit del supporto e la prova d'arresto per release.
3. I comandi che richiedono 5 ms di margine prima della release successiva vengono rifiutati: tale margine non compare entro 100 ms.

Non risultano colpevoli: letture UART bloccanti, ritrasmissione di target identici (già soppressa), lock.

**Prima mossa proposta (non ancora implementata):** ripristinare il riuso a release alterne dell'ultimo offset di correzione accettato, mantenendo **a ogni release** i controlli di box, limiti e prova d'arresto. In più, una correzione della diagnostica: due etichette del JSON sono invertite e mancano lateness e tempi esterni.

## Domande per un parere

1. **`double` → `float` sull'ESP32-S3.** IK chiusa più Newton (tolleranza 1e-6 m), FK di contatto, condizionamento con inversa 3×3; quantizzazione dei servo 1,53 mrad. Conviene passare a `float` (FPU hardware) solo i cicli interni, lasciando in double le verifiche di sicurezza e la conversione RAW? Che guadagno ti aspetti realisticamente, e quali trappole di precisione hai visto (`atan2`/`acos` vicino ai limiti, Newton mal condizionato)? In alternativa: `-O2` mirato, funzioni calde in IRAM, I-cache da 32 KB?
2. **Struttura real-time.** Oggi la release è chiamata 3 volte per giro di loop insieme a HTTP e diagnostica, su priorità 1. Consigli un task Motion dedicato su timer hardware, oppure pianificazione a 25 Hz con interpolazione lineare a 50 Hz in spazio giunti? Le box certificate sono convesse, quindi l'interpolazione tra due target certificati resta dentro le box. Come gestiresti lo STOP, che oggi viene applicato solo nei punti di I/O, mentre un calcolo può durare fino a ~45 ms?
3. **Stesso lavoro, tempo ×4,5.** In HOLD il percorso è deterministico (stessi contatori, verificato offline), ma il tempo varia da 7,4 a 33 ms. Il core 0 serve Wi-Fi, HTTP e asset 3D gzip dalla flash. Come discrimineresti in modo economico la contesa su I-cache/flash rispetto agli interrupt sul core 1 (contatori PMU Xtensa, A/B con radio spenta, altro)?
4. **Velocità.** Crawl quasi statico: periodo 6,5 s, duty 0,825, sway laterale 14 mm, margine statico del supporto +0,55/+1,85 mm (modello). Dimezzando il periodo, lo spostamento ZMP stimato (~0,64 mm) consuma già il margine. Per arrivare a 2–3× conviene allungare lo stride, ridisegnare lo sway in base allo ZMP o passare presto a un trotto? Quale velocità ritieni ragionevole per un robot di questa taglia con ST3215?
