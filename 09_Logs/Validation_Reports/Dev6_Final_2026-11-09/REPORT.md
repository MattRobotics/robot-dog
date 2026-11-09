# MATDOG 1.0.2-dev.6 — Mandato finale di ottimizzazione (offline)

**Data:** 09/11/2026. **Branch:** `feat/gait-redesign-20261108`, pubblicato su `origin feat/motion-perf-next-20261107`. **PR:** #67, OPEN e DRAFT.

**Installato sul robot:** `1.0.2-dev.4`, build `4358f13234f9`. Partizioni e NVS non sono stati toccati.

**Nuova candidata:** `1.0.2-dev.6`, sorgente `4e62b818d5569c439e289a42de5514b9d81277aa`, build CLEAN `ROBOT_POWERED` `4e62b818d556`. APP `1 736 160` B, SHA256 `d8ba3f147cc7a1f59904b28b1b7b3abdd33b8a3acd35acb3dc15812c314cd475`.

**Riferimenti storici, non sovrascritti:**
- `5159c7f`, APP `bce9a022…` (modalità WALK, mai flashata);
- `3803960`, APP `05935919…`.

**Classi di evidenza:**
- **VO** = verificato offline;
- **VH** = verificato su hardware;
- **INH** = implementato, non validato su hardware;
- **UNKNOWN**.

Nessun flash, reset, scrittura NVS/EEPROM, calibrazione, modifica dei limiti o merge.

## Verdetto

**READY FOR SUPERVISED FLASH (SLOW).** SLOW è la legge di dev.4, identica bit per bit.

**STANDARD 30 mm/s e FAST 32 mm/s sono INH.** Vanno qualificate prima sulla fixture, poi a terra (§19).

**FAST 40 mm/s non è raggiungibile** entro i vincoli credibili; il motivo esatto è nel §4. Il massimo dimostrato offline con margini non inferiori a SLOW è **32 mm/s**.

## 1. Prima/dopo (velocità nominale del corpo = passo/periodo)

| | dev.4 (installata) | dev.6 `5159c7f` | **dev.6 finale** |
|---|---|---|---|
| SLOW | 20 mm/s (100 mm / 5 s) | 20 | **20** (50/75/100 mm, 5,0 s; invariata) |
| STANDARD | — | 25 (75 mm / 3 s) | **30** (60/75/**90** mm, 3,0 s → 20/25/30 mm/s) |
| FAST | — | 30 (75 mm / 2,5 s) | **32** (60/70/**80** mm, 2,5 s → 24/28/32 mm/s) |
| Pitch-down IMU (preset massimo) | −12,8 mrad (SLOW100) | STANDARD −36,8, FAST −14,9 | **STANDARD −17,1, FAST −34,2** |
| Joystick | 20 / 4,0 mm/s / 1,83 °/s, sempre SLOW | invariato | invariato. La pagina ora invia il 100 % del range: prima arrivava al 77 % laterale e all'80 % in rotazione |
| Preset rifiutati | — | 100 mm in STANDARD/FAST | **nessuno**: ogni modalità ha i suoi tre preset |

## 2. STANDARD 30 mm/s — VO, INH

**Configurazione:** 90 mm, 3,0 s, duty 0,825, quota 110 mm, alzata 8 mm, sway laterale 1,5 mm, longitudinale 3 mm. Swing asimmetrico 0,35/0,55. Drive `{1800,180}`, pari al picco della legge / 0,8.

**Margini al preset massimo** (terreno rigido, COM URDF, inseguimento ideale):
- supporto statico 2,18 mm (SLOW 2,12);
- capture point allo STOP 1,69 mm (SLOW 1,41);
- margine al box kHeight: HIP 8,7, UPPER 90, LOWER 192 tick (SLOW 9,3 / 31,7 / 93).

**Picchi della legge:**
- giunti in swing: 2,207 rad/s, 21,0 rad/s²;
- giunti caricati: 0,77 rad/s, 7,3 rad/s².

**Esiti:**
- prova STOP alla frenata misurata: PASS su tutti e 3 i preset, 8 fasi e T2 inclusi;
- modello del servo: 0 fault, 48/48 STOP con ripresa a 11,1 e 9,6 V.

**Alternative scartate:**

| Candidata | Esito |
|---|---|
| 75/2,5 | autorità IMU minore |
| 100/3,33 | pitch-down −18,5 mrad grezzo |
| quota 105 | UPPER 58 tick |
| sway 1,0 | capture point 1,32, sotto SLOW |

## 3. FAST — 40 mm/s non raggiunto; 32 mm/s VO, INH

**Configurazione:** 80 mm, 2,5 s, duty 0,825, quota 105 mm, alzata 6 mm, sway laterale 1,0 mm. Swing 0,35/0,55. Drive `{1950,220}`.

**Margini al preset massimo:**
- supporto statico 1,71 mm;
- capture point 1,41 mm (= SLOW);
- HIP 11,1, UPPER 70, LOWER 253 tick.

**Picchi della legge:** in swing 2,350 rad/s e 26,4 rad/s²; caricati 0,87 rad/s e 10,8 rad/s².

**Esiti:** prova STOP PASS su tutti i preset; modello del servo: 0 fault.

**Perché queste scelte:**
- **Alzata 6 mm:** con 8 mm il pitch-up crolla a 1,5 mrad.
- **Sway 1,0 mm:** con 1,5 mm l'HIP posteriore restava a 8,3 tick dal box. Il cedimento statico misurato in dev.15 (fino a ~10 tick sotto carico) lo portava fuori box durante lo STOP nel modello del servo: `STOP_MEASURED_OUTSIDE_BOX` in 2–3 seed su 6, con qualsiasi inviluppo.
- **Quota 105:** la quota più bassa dà autorità pitch (come chiesto, ho verificato che non è sempre un vantaggio): toglie margine all'HIP e supporto, ed è compensata dallo sway 1,0.

## 4. Perché non 40 mm/s: limiti distinti

Ricerca: ~1 950 configurazioni del generatore di produzione. Variabili esplorate:
- falcata 75–120 mm;
- quota 100–120 mm;
- duty 0,80–0,85;
- COM shift 11–17 mm;
- rampe di swing;
- alzata 6/8 mm;
- sway 1–2 mm.

Evidenze: [`geometry_sweeps.txt`](evidence/geometry_sweeps.txt) e [`fast_authority_grid.txt`](evidence/fast_authority_grid.txt).

| Limite | Effetto a 40 mm/s |
|---|---|
| Cinematico assoluto / IK | Nessun fallimento IK. La legge chiede 2,5–3,0 rad/s e 25–33 rad/s²: dentro il registro (velocità a vuoto 4,34 rad/s), ma ACC > 50 non è mai stato misurato |
| Box certificata `kHeight` | Non allargata. A 100 mm l'UPPER anteriore a fine swing ha margine sufficiente solo con COM shift ≥ 20 mm, che porta il supporto statico a −2,5 mm |
| Legge temporale | Lo swing a 2,0–2,5 s porta l'UPPER anteriore a ~2,5 rad/s vicino al box |
| Profilo servo | Non vincolante offline (velocità ≤ 71 % della velocità a vuoto a 11,1 V) |
| Frenata dimostrata (prova STOP a 8 400 step/s²) | **Prima causa:** l'arresto è sincrono e dura 1,5·v/a fissato dall'UPPER più veloce (0,30 s). L'HIP (box ±16 tick) viene trascinato per tutta la durata ed esce di 1–5 tick. 0 PASS su 1 344 configurazioni a 40 mm/s |
| Incertezza della frenata | Una frenata più alta, mai misurata, non basterebbe: vedi riga successiva |
| Supporto / capture point | **Limite fisico indipendente dal servo:** il capture point allo STOP vale ~0 mm a 40 mm/s e ≤ 1,1 mm a 34–36 mm/s in ogni configurazione che supera la prova STOP. Con ACC illimitata il robot fermato di colpo non resterebbe sul poligono d'appoggio |

**Massimo sostenibile con margini ≥ SLOW:** 32 mm/s.

**Fra 33 e 36 mm/s:** la prova STOP passa solo con capture point 0,4–1,1 mm, oppure con pitch-down 2–5 mrad. Non lo propongo come modalità operativa.

## 5. Speed/ACC e dati QC

**Rianalisi dalla posizione encoder** (non dalla PresentSpeed quantizzata a 50): [`st3215_brake_position.py`](evidence/st3215_brake_position.py) → [`.txt`](evidence/st3215_brake_position.txt). Dati: 22 ST3215-C018, a vuoto, 11,1 V. `a_stop = v0²/(2d)` è la frenata costante equivalente nella stessa distanza.

| Movimento | ACC comandata | a_stop mediana | p10 | Lettura |
|---|---|---|---|---|
| Lungo, velocità massima (goalSpeed 0) | 50 | 12 244 | 10 238 | Il servo frena **più** del profilo: a velocità massima la frenata non segue l'ACC. Capacità fisica a vuoto ≥ 10 000 |
| Breve (picco 300 step/s) | 50 | 5 625 | 4 131 | Segue l'ACC (0,83× al p10) |
| goalSpeed 600 | 25 / 30 | 2 427 / 2 978 | 2 027 / 2 373 | Segue il profilo a 0,8–1,0× |
| ACC > 50, sotto carico, altre tensioni | — | — | — | **UNKNOWN**: nessuna misura |

**Conseguenze:**
- un valore ACC 140–220 nel registro **non** è una frenata garantita;
- la prova continua a usare min(ACC, 84) = 8 400 step/s². **Non ho cambiato `kStopBrakeAcc`.**

**Nota sulla prova (VO, matematica):** l'arresto quartico ha decelerazione media pari a 2/3 del picco. Un servo che frena costantemente a ≥ 2/3·8 400 = 5 600 step/s² si ferma entro la distanza del riferimento. Il dato a vuoto (≥ 10 000) supera questa soglia; il dato sotto carico manca.

**Profili per modalità:** uno solo per modalità, fonte unica `kWalkModes` nel Controller.
- SLOW `{1000,50}`, invariato;
- STANDARD `{1800,180}`;
- FAST `{1950,220}`.

FAST usa il 69 % della velocità a vuoto a 11,1 V e circa l'80 % a 9,6 V. Non uso il massimo del registro.

## 6. STOP con frenata sensibile al carico (VO, modello)

**Modello del servo** (`servo_plant.inc`):
- capacità di frenata separata, `PLANT_BRAKE_LO/HI`, moltiplicata per un fattore di carico 0,6–0,95 e per la tensione;
- metrica nuova: escursione reale oltre il box durante ogni STOP.

| Frenata reale estratta (a vuoto, prima del carico) | SLOW | STANDARD | FAST |
|---|---|---|---|
| 6 000–15 000 (default) | 48/48, uscita dal box 0,0 tick | 48/48, 0,0 | 48/48, 0,0 |
| 5 600–8 400 | 48/48, 0,0 | 43/43 STOP OK; **1 fault POINT_SAFE alla ripresa** (c0,06), non nello STOP | 48/48, 0,0 |
| **3 000–5 600** (sotto i 2/3, ×carico 0,6–0,95) | 48/48, 0,0 | 48/48, 0,0 | 48/48, 0,0 |

Con frenata ridotta lo scostamento finale dal piano resta ≤ 2,2 tick e l'inseguimento peggiora fino a 210 tick.

**Interpretazione:** con frenata inferiore al valore nominale, il ritardo d'inseguimento rende la posizione reale più arretrata del riferimento. Per questo, nel modello, nessuno STOP è uscito dal box.

**Cosa il modello non copre:**
- dinamica del corpo;
- contatto;
- scivolamento;
- coppia gravitazionale.

**Corpo (VO, statico):** per le tre modalità al preset massimo, capture point e ZMP dell'arresto sono nella tabella del §2/3. Lo ZMP transitorio dell'arresto esce dal poligono in tutte le modalità, SLOW compreso (da −2,4 a −5,3 mm). Indica un possibile sollevamento di bordo di 40 ms, non una caduta: SLOW ha frenato a terra senza ribaltarsi.

**Separazione fra arresti:** STOP (coppia mantenuta) e SAFE_OFF (coppia OFF 13/13, latch) restano distinti e invariati.

**Classe:** frenata reale sotto carico **INH**.

## 7. Autorità IMU per modalità

**Strategia unica** per le tre modalità: stessa `compensationTarget`, nessun secondo controllore. Ogni inviluppo ha una sola origine:
1. ricerca geometrica sul preset massimo (prova STOP, T2, 8 fasi, ×0,85);
2. scala ridotta finché il modello del servo non dà 0 fault.

| Modalità | Pitch-down / pitch-up / roll (mrad) | Fattore di scala | Verifica sui 3 preset |
|---|---|---|---|
| SLOW 50–75 / 100 | −81,8 / −12,8 (invariata) | — | PASS |
| STANDARD | **−17,1 / +20,6 / ±9,1** | ×0,5 (×0,65: un POINT_SAFE alla ripresa) | PASS |
| FAST | **−34,2 / +26,8 / ±11,8** | ×0,65 | PASS |

**Perché la scala:** il fit d'appoggio sulle gambe in ritardo chiedeva fino a −94 mrad dopo una ripresa (il poligono geometrico dei preset corti arrivava a −95). Nel modello del servo questo produceva POINT_SAFE alla ripresa e STOP_PLAN con l'IMU al vertice.

**Richiesta reale:** dev.4 a terra chiedeva −15…−18 mrad. STANDARD (−17,1) e FAST (−34,2) la coprono o la sfiorano.

**Riduzione di passo/velocità quando l'autorità non basta:** non implementata; ne servirebbe la prova di continuità e STOP. I gate H1-RCA, WALK_SETTLE, HEIGHT e PRONE non sono toccati e la suite è PASS.

## 8. Laterale e rotazione — non migliorati

Il joystick resta sulla legge SLOW:
- avanti 20 mm/s;
- indietro 10 mm/s;
- laterale 4,0 mm/s;
- rotazione 1,83 °/s.

**Correzione fatta:** la dashboard scalava laterale e rotazione su costanti obsolete (6,5 s) e inviava al massimo il 77 % e l'80 % del range del Controller. Ora legge i limiti dalle capabilities: il laterale effettivo sale da 3,1 a 4,0 mm/s e la rotazione da 1,46 a 1,83 °/s.

**Nuova legge laterale/yaw:** non svolta nel tempo disponibile. La priorità 6 viene dopo frenata, geometria, IMU e budget. Il gate joystick (46 scenari) è rieseguito sulla legge invariata ([`joystick_envelope.txt`](evidence/joystick_envelope.txt)).

## 9. Joystick e modalità effettiva

Il joystick cammina **sempre SLOW**. Lo dicono la guida della dashboard, `walk_mode.active` nello stato e la DIAG `exit_cause`. Non ho assegnato al joystick i parametri di STANDARD o FAST: i suoi inviluppi sterzati sono certificati solo per SLOW.

## 10. Dashboard

**Struttura:**
- il selettore **SLOW | STANDARD | FAST** è il primo elemento del riquadro WALK;
- subito sotto: periodo, velocità massima, falcata, quota, alzata, Speed/ACC, frenata STOP e stato di qualifica, letti dalle capabilities;
- poi i tre preset della modalità con passo e velocità, ad esempio «🐢 60 mm · 20 mm/s»;
- il comando WALK resta compatibile: indice del preset tramite `BASE_VELOCITY`.

**Note e contratto:**
- note aggiornate: niente 6,5 s, niente 6,9/3,1 mm/s, niente qualifiche hardware non svolte;
- STOP sempre accessibile;
- contratto `GAIT.SELECT`: volatile, rifiutato con STOP latched, modalità selezionata e attiva distinte, latch durante WALK STOP e ripresa.

**Test:** fixture 79 PASS; DOM 21 envelope / 89 comandi PASS; contratto 6 114 PASS; browser reale (layout, ViewCube, STOP durante il caricamento del Twin) PASS.

## 11. Double contro float — non applicato

**Non ho fatto la conversione** né il benchmark comparativo. Preferisco non toccare la precisione delle prove di sicurezza senza la misura completa richiesta al §6.2 del mandato: differenza RAW, branch IK, prova STOP, margini.

**Dato disponibile** (QEMU, sessione precedente): la libm trascendentale double vale circa 2/3 del lavoro deterministico, ~0,34 M istruzioni per release in media e 1,13 M al massimo.

**Piano consigliato:** `sinf`/`cosf` solo nella valutazione del gait nominale. Prove STOP, box e conversione RAW restano double. Va misurato con l'harness QEMU esistente (digest RAW host contro Xtensa).

## 12. Motion budget

**Non rimisurato in questa sessione.** Restano validi i risultati precedenti:
- calcolo ≈ 4–5 ms a 240 MHz;
- picchi hardware di 29,6 ms non spiegati dal calcolo: cache flash o preemption, ipotesi non dimostrata;
- build diagnostica `MATDOG_DIAG_PREEMPTION` separata e spenta.

**Limiti noti di quella misura:**
- `uxTaskGetSystemState` due volte per release sospende lo scheduler;
- le ISR non sono attribuite ai task;
- i contatori crescono solo al cambio di contesto.

Non è una misura esatta del solo tempo CPU di Motion. La misura hardware minima è nel §19.

## 13. QEMU con carichi concorrenti — non svolto

Il banco QEMU resta quello già validato. **Non l'ho rieseguito sulla nuova tabella:** gli scenari STANDARD e FAST di `motion_scenarios.h` usano il preset 1, che ora ha un'altra falcata, quindi il loro digest cambia. Il digest SLOW è invariato per costruzione: `walkGait(i)` di SLOW è verificato identico.

L'harness di concorrenza sintetica non è stato costruito.

## 14. Modello del servo: calibrazione e incertezze

**Correzioni:**
- **Tensione:** la coppia, e quindi accelerazione e frenata, ora scala con la tensione. Prima scalava solo la velocità, che la legge non raggiungeva mai (frazione in saturazione di velocità 0,0 %). Per questo il tracking risultava identico a 9,6 e 11,1 V: era un artefatto del modello. Ora SLOW 66→70 tick, STANDARD 108→123, FAST 131→149.
- **Frenata separata** dall'accelerazione.
- **Uscita dal box durante STOP** riportata.

**Plausibilità degli errori d'inseguimento:** 66–210 tick. Al massimo arrivano con capacità estratte ≤ 6 000·0,6 contro una legge che chiede 14 000–17 000 step/s². È plausibile se ACC > 50 sotto carico è davvero così bassa. È **ignoto** perché non è mai stato misurato. Le tracce dev.4 di SLOW non sono confrontate qui.

**Limiti del modello:** non accoppia corpo, contatto e IMU. Le correzioni che produce non sono evidenza di stabilità dinamica.

## 15. Regressioni e margini

| Prova | Esito |
|---|---|
| Suite SimpleMotion (ASan/UBSan): mode latch, 8 STOP + ripresa, continuità del gait su 3 modalità × 3 preset × 5 STOP, H1-RCA, PRONE, recupero, T2, WALK100 lungo, `exit_cause` | PASS |
| Check geometrico e di legge, 3 modalità × 3 preset ([`final_tilt_support_checks.txt`](evidence/final_tilt_support_checks.txt)) | 9/9 PASS |
| Supporto statico, 3 × 3 | 9/9 PASS |
| Modello del servo, 18 celle | 0 fault |
| Sensibilità della frenata | vedi §6 |
| Test host, incluso il contratto HTTP | PASS |
| Audit statico | **PASS** (370 file) |
| Dashboard, incluso il browser | PASS |
| Gate joystick | vedi evidenza |

**Non rieseguiti** (sorgente SLOW e joystick invariati, rischio basso): version repro, matrice delle perturbazioni, QEMU, partenze in pendenza.

**Statistica:** 6 seed per cella non permettono una stima di probabilità di successo. Le prove sono correlate (stesso modello, stessi scenari): il risultato è una valutazione condizionata alle ipotesi del modello, non una percentuale.

## 16. Build

| Campo | Valore |
|---|---|
| Versione | `1.0.2-dev.6` |
| Commit sorgente | `4e62b818d5569c439e289a42de5514b9d81277aa` |
| Build ID | `4e62b818d556` |
| Stato | CLEAN `ROBOT_POWERED`, OTA ingest 0 |
| APP | `1 736 160` B, SHA256 `d8ba3f147cc7a1f59904b28b1b7b3abdd33b8a3acd35acb3dc15812c314cd475` |
| Manifest | `~/MATDOG/build-output/gait-redesign-20261108/build-4e62b81/artifacts/ROBOT_POWERED-ota0/` |

## 17. Script APP-only (preparato, NON eseguito)

Script: `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6F_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.6-4e62b81.sh`.
- Richiede `MATDOG_DEV6F_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`.
- Gate su layout, SHA, manifest e backup di rollback; NVS intatta.
- Rollback dev.4 e backup completo invariati.

## 18. PR #67

OPEN e DRAFT, aggiornata con questo report.

## 19. Rischi residui e qualifica hardware minima

**Rischi residui:**
1. **Frenata reale sopra ACC 50 e sotto carico:** UNKNOWN. La prova assume 8 400; il modello regge anche frenate di 3 000–5 600, ma è un modello.
2. **Inseguimento con ACC 180–220:** UNKNOWN. Il fit d'appoggio sulle gambe in ritardo può chiedere correzioni grandi: gli inviluppi sono ridotti apposta.
3. **POINT_SAFE alla ripresa in STANDARD** (1 seed su 6 in una fascia di frenata): l'esito è fail-closed (fault e SAFE_OFF), non un'uscita dal box.
4. **Capture point FAST** pari a SLOW (1,41 mm) e alzata 6 mm (meno luce dal terreno).
5. **Causa dei picchi temporali:** non dimostrata.

**Qualifica hardware minima** (nell'ordine, ogni passo con STOP pronto):
1. Flash APP-only e boot in sola lettura; SLOW di regressione (WALK100, STOP, PRONE).
2. **Sulla fixture, zampe libere:**
   - STANDARD poi FAST, preset 0 → 2;
   - registrare goal/present, corrente e temperatura a ogni release;
   - STOP in 8 fasi del ciclo per modalità.
   - Misura chiave: distanza d'arresto reale contro piano (`exit_cause`/DIAG) e inseguimento contro `max_tracking` del modello.
3. **Misura frenata sotto carico:** STOP da STANDARD e FAST a terra con supporto, almeno 8 per modalità, confrontati col banco (≥ 5 600 step/s² equivalenti).
4. **A terra:** STANDARD 60 → 75 → 90 mm, poi FAST. Registrare la correzione IMU richiesta e applicata (saturazione sull'inviluppo).
5. **Budget:** la build diagnostica `MATDOG_DIAG_PREEMPTION` solo dopo i punti 1–4, in una sessione separata.
