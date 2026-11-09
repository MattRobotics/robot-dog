# MATDOG 1.0.2-dev.6 consolidata — report finale (09/11/2026)

Classi usate:
- **VERIFIED HARDWARE**: misurato sul robot;
- **VERIFIED OFFLINE**: rig/strumenti host;
- **IMPLEMENTED NOT VALIDATED**: codice nuovo senza prova su hardware;
- **HISTORICAL**: evidenza di versioni precedenti;
- **UNKNOWN**: nessun dato.

Nessun flash, reset, accesso hardware, comando servo, scrittura NVS/EEPROM, calibrazione o merge in questa sessione.

## 1. Stato GitHub

- PR [#67](https://github.com/MattRobotics/robot-dog/pull/67): OPEN, DRAFT, ramo `feat/motion-perf-next-20261107`, nessun merge in `main`.
- Commit di questa consolidazione, su `d1bd158`:

| Commit | Contenuto |
|---|---|
| `d56ba04` | WALK100: prova di STOP con i punti medi T2 e correzione mantenuta su passo rifiutato |
| `bc47f3d` | Una sola legge joystick continua |
| `1424c54` | Fixture di trasporto riparate |
| `d06ed2d` | Causa della prima uscita |
| `dd16ed2` | Stack |
| `fbdf146` | Harness di riproduzione per versione e matrice di perturbazioni |
| `3803960` | Identità della build consolidata |

- La stringa di versione resta `1.0.2-dev.6`. La build `2c53ecf` (APP `eef32e61…`) è superata e non è mai stata flashata.

## 2. Banco (rig) e copertura — VERIFIED OFFLINE

Il rig è il `Rig` di `test_simple_motion.cpp`:
- SimpleMotion di produzione;
- porta servo scriptata, feedback = riferimento salvo perturbazioni;
- IMU come quaternione;
- orologio di rilascio.

| Cosa | Natura |
|---|---|
| Leggi, sequenze, prova di STOP, box, inviluppi, ritardi di rilascio | simulazione deterministica del codice di produzione |
| Sag, carico, offset per giunto, ritardo di feedback | modello (valori da RCA hardware: sag `err` e `load` della H1-RCA) |
| Supporto statico, clearance, raggi IMU | geometria (gate joystick, `foot_edge_contact.py`) |
| Tempi CPU, stack reale, corrente, coppia, attrito, dinamica | **non rappresentati** (UNKNOWN sul rig) |

Riproduzione dei difetti storici, ciascuno sulla versione che lo ha introdotto (`run_version_repro.sh` e regressioni dei commit correttivi):

| Difetto | Versioni | Misura sul rig della versione | Finale |
|---|---|---|---|
| Avanti con jitter → oscillazione sterzata di 14 mm (a terra, dev.4) | dev.3, dev.4 | escursione 14,3 mm | 1,5 mm |
| Jitter dopo una sterzata | dev.5 | 14,3 mm (dev.6 `2c53ecf`: 2,0 mm) | 1,5 mm |
| Perdita del cambio di legge dopo la sterzata | dev.5, dev.6 | 1 WALK STOP, ritorno a 6,92 mm/s | 0 stop, 20,00 mm/s |
| WALK100 lungo: FAULT `STOP_PLAN` | dev.3 … dev.6 (anche la dev.4 installata) | FAULT a 19,46 cicli | 31,5 cicli senza fault |
| Latch di ingresso (STAND rifiutato → FAULT) | dev.4 | la regressione di `789f342` fallisce (non STOPPED) | PASS |
| Deadlock H1 (srotolamento dopo settle sterzato) | dev.3 | la regressione di `004685e` fallisce (WORKSPACE) | PASS |
| Spike di tempo e stack | — | non rappresentabili sul rig | analisi statica (§12) |

## 3. Qualità del plant

- I numeri di legge e geometria sono deterministici.
- Il plant di servo è uno scenario, non un modello identificato: sag e carico sono costanti misurate una volta (H1-RCA), non un modello P o di attrito.
- La correzione IMU stazionaria del rig (circa −12…−17 mrad) è dello stesso segno e ordine di quella hardware (dev.4 a terra −15…−18 mrad).

## 4. Diagnostica causale

Audit (classe):
- `@MOTION DIAG` / `TIMING`: completo per tempi e giunti (VERIFIED HARDWARE, dev.4).
- `refused_by`: per rilascio (VERIFIED OFFLINE).
- La prima causa di un WALK veniva persa. `enterOff` azzerava la correzione e `refused_by` vale solo per un rilascio: proprio i dati che hanno identificato il fault WALK100.
- Nuovo `exit_cause` (`d06ed2d`), congelato una volta alla prima uscita, senza costo nel rilascio. Contiene:
  - il check rifiutato;
  - il ciclo;
  - la legge;
  - il flag nominal-solved (T2);
  - la correzione applicata e voluta.
- Regressione `exitCause`: dopo un FAULT e poi STOP il record resta.

Catena causale WALK100 (ROOT CAUSE VERIFIED offline):
1. A fine swing (fase .464/.968) il giunto UPPER rallenta verso il limite inferiore di kHeight.
2. T2 invia il punto medio della corda, che sta fuori dalla curva.
3. La secante punto medio → esatto vale 10 tick invece di 9 (−0,767 rad/s).
4. L'arresto quartico finisce 1,4 tick fuori dal box.
5. `STOP_PLAN` → `WALK_IK_OR_WORKSPACE`.

Contribuisce (CONTRIBUTING) la correzione IMU sul bordo pitch-down di kWalkTilt100: la dev.4 a terra c'era nel 102/103 dei campioni. Un secondo meccanismo, il passo di slew di 0,8 mrad (+1 tick), compare solo senza rilasci leggeri.

## 5. Cause degli spike di tempo

- Dev.4 (hardware, H1RCA): il rilascio più lungo, 29,6 ms, è al 73 % `compensate` (21,7 ms), con 2 solve e 8 IK. La variabilità sta nella compensazione (libm in doppia precisione software sull'ESP32-S3), non nel numero di IK. Classe: VERIFIED HARDWARE (dati) e HYPOTHESIS (attribuzione alla libm).
- Gap di visita fino a 31 ms, con esterni (USB/HTTP) di 6,3–6,8 ms: CONTRIBUTING.
- HOLD 7,4–33,1 ms, ripartizione esatta: HYPOTHESIS. Le statistiche di runtime FreeRTOS non sono attive, quindi non c'è tempo CPU separato dal tempo di parete.
- Il proxy di costo della build finale è invariato rispetto a `d1bd158`:
  - WALK100 p99: 430 → 394 chiamate libm;
  - joystick p99: 822 → 750 chiamate libm.

## 6. Nuova legge joystick a confronto con dev.4 e dev.6

| | dev.4 | dev.6 `2c53ecf` | finale |
|---|---|---|---|
| Avanti | 6,9 mm/s, legge sterzata | 20 mm/s, WALK STANDARD | 20 mm/s, regime avanti/indietro |
| Prima sterzata | — | WALK STOP `JOYSTICK_STEERING` | handover senza stop (10,5 s) |
| Ritorno in avanti | resta sterzata | resta sterzata, 6,9 mm/s | regime WALK STANDARD, 20 mm/s (7,1 s) |
| Laterale / imbardata | 3,1 mm/s / 1,76°/s | 3,1 / 1,76 | 4,0 mm/s / 1,83°/s |
| Oscillazione laterale con jitter | 14 mm | 1,5–2 mm | 1,5 mm |
| Stop | inversione | inversione e cambio di legge | solo inversione avanti/indietro |

## 7. Transizioni e margini

- Handover: un ciclo assestato nella sovrapposizione dei box (≤ 60 mm), RAW comandato nel box nuovo, correzione applicata nel nuovo inviluppo.
- Passo RAW ≤ 17 tick per rilascio nella sequenza tenuta (≤ 20 tick in tutte le prove nominali).
- Supporto statico minimo:
  - gate a 46 scenari più casuali: +0,39 mm;
  - 24 sequenze casuali: minimo +0,39, p10 +0,63, mediana +1,17 mm;
  - clearance ≥ 2,29 mm.
- Banda 20 %: dai comandi touchscreen reali (25 comandi in avanti) la quota laterale massima è 13 % (14,6 % con l'arrotondamento dell'etichetta): margine 5,4 punti.

## 8. Laterale e imbardata

- Laterale 25 mm per ciclo: un FAULT su 24 sequenze casuali e p10 +0,37 mm, quindi **scelto 20 mm** (4 mm/s, 0 fault).
- Imbardata 0,20: supporto minimo solo +0,18 mm, quindi 0,16.
- Vincoli limitanti:
  - prova di STOP con oscillazione > 10 mm;
  - supporto negativo con oscillazione ≤ 7 mm;
  - raggi IMU sul raccordo a 12°.

## 9. Matrice dei profili servo

Vedi [SERVO_PROFILE_STUDY.md](SERVO_PROFILE_STUDY.md):
- resta `{1000,50}`;
- `{1000,80}` raddoppierebbe l'inviluppo pitch-down di WALK100 ed è il candidato per un A/B hardware.

## 10. STOP, PRONE, ingresso, H1

Regressioni invariate e PASS:
- STOP in 5 istanti di un WALK avanti/indietro, poi STAND;
- PRONE 4/4;
- recupero di ingresso;
- srotolamento H1;
- matrice di perturbazioni con STOP in un istante casuale e recupero STAND: 0 fallimenti.

## 11. Regressioni

Tutte eseguite sul sorgente finale (`fbdf146`/`3803960`, lo stesso codice Motion), VERIFIED OFFLINE:

| Suite | Esito |
|---|---|
| `run_simple_motion_tests` | PASS: nuove `longWalk100`, `exitCause`, sequenza tenuta, handover, banda, rampe |
| `run_simple_dashboard_tests` | PASS: contratto 6 116 check, layout, viewcube; STOP durante il caricamento Twin 496 ms |
| `run_joystick_envelope` | PASS: 46 scenari più casuali, supporto minimo +0,39 mm, clearance 2,29 mm |
| `walk_tilt_envelope_check` 50/75/100 (T2 e 8 fasi) | PASS |
| PRONE, STOP support, servo packet/bus deadline, IMU feed, calibration restore | PASS |
| Matrice di perturbazioni (60 celle, 4 seed per le casuali) | 0 fault, 0 fallimenti di recupero |
| Riproduzione per versione | §2 |
| Static audit | PASS 364/364 |
| Build ESP32-S3 | CLEAN |
| `ap_transport` / `ap_only_transport` | PASS (85 / 134 check) |
| `ap_ingress` | esegue; exit 1 voluto (finding storico F01 BLOCKED) |

Note sulle regressioni:
- Il check STOP-durante-Twin del browser è fallito una volta a 722 ms, con la macchina saturata da prove parallele. Ripetuto a macchina scarica: PASS, 496 ms. È una misura, non un limite.
- `motion_transport_mode` e `motion_safety_boundaries` collegano sorgenti dello stack esecutore rimosso (`execution/*`): non riparabili come fixture.
- Altre 9 suite legacy falliscono identiche su `d1bd158` (preesistenti).

## 12. Stack e budget

- Xtensa, 89 unità, `-fcallgraph-info`: peggior percorso statico da `loop()` circa 11 248 B su 16 384. Era 11 024 B in dev.5 e 15 952 B nella dev.4 installata.
- `compensateLevel`: 592 → 800 B per il retry.
- Il minimo libero reale si misura solo su hardware: dev.4 1 696 B, con statico 15 952.

## 13. Identità finale dell'APP

| Campo | Valore |
|---|---|
| Versione | `1.0.2-dev.6` (stessa stringa, nuova build) |
| Sorgente | `3803960faf6352d8b11b9e0d4160c6f170e1b0d7`, CLEAN |
| Build ID | `3803960faf63` |
| Profilo | `ROBOT_POWERED`, OTA ingest 0 |
| APP | `MATDOG_Controller.ino.bin`, **1 727 536 B** |
| APP SHA256 | **`0593591938c686018141f5949713b67ac5b60bdf3ce942cd01e4134b0b89e88f`** |
| Layout | `MATDOG_16M_2x5M_NVS_V1`, tabella `8f756ecb…46e7` |
| Build UTC | 2026-11-09T09:39:09Z |
| Script APP-only | `~/MATDOG/verification-artifacts/MATDOG_1_0_2_DEV6C_FLASH_PREP_20261109/flash-qualified-app0-1.0.2-dev.6-3803960.sh` — **preparato, NON eseguito**; variabile `MATDOG_DEV6C_FLASH_AUTHORIZED=YES_ROBOT_SUPPORTED`; diff rispetto allo script dev.6 rivisto: solo le righe di identità |
| Rollback | dev.4 installata, APP `3e7ee77d…` (build `4358f13`, script in `MATDOG_1_0_2_DEV4_FLASH_PREP_20261108/`); backup full flash `5cbba0b9…`; `rollback-assets.sha256` verificato OK (8 file) |
| Superata | dev.6 `2c53ecf`, APP `eef32e61…` (mai flashata) |

## 14. Rischi

1. Le leggi sono verificate solo offline: dinamica, attrito, sag reale e tempi CPU sono UNKNOWN.
2. L'inviluppo pitch-down di WALK100 è più stretto (−12,8 mrad). A terra la correzione voluta era −15…−18 mrad, quindi resteranno ~3–5 mrad di beccheggio non livellato in WALK100: corpo leggermente più cabrato, reason `ENVELOPE`.
3. Handover lenti: 10,5 s per entrare nella sterzata, 7,1 s per tornare in avanti. Sono voluti (rampa dello stride più un ciclo assestato), ma l'operatore deve saperlo.
4. Ritardo di avvio laterale.
5. Le statistiche CPU non sono disponibili.

## 15. Test hardware minimo

Solo dopo un'autorizzazione esplicita al flash. Robot sostenuto, SAFE_OFF 8191 e coppia OFF verificati.

1. **Flash APP-only con lo script preparato, poi H0.** Verificare:
   - build `3803960faf63` e SHA dell'APP;
   - NVS gen-1;
   - census 13/13;
   - SAFE_OFF 8191.
   
   `@MOTION DIAG` deve contenere la riga `exit_cause`.
2. **Fixture.** STAND150, HOLD 10 s, WALK100 per 5 cicli, STOP, STAND.
3. **Pavimento: WALK STANDARD 100 per ≥ 3 minuti (≥ 36 cicli).** È il caso che faultava sul rig. Attese:
   - `imu_correction.reason` = ENVELOPE a circa −12,8 mrad;
   - nessun fault;
   - overrun e perse non peggiori della dev.4.
4. **Pavimento: joystick con la levetta tenuta.**
   - Avanti 30 s a 20 mm/s.
   - Diagonale destra tenuta: handover alla sterzata in circa 10–11 s, senza WALK STOP.
   - Destra, poi rotazione.
   - Avanti: ritorno a WALK STANDARD in circa 7 s, 20 mm/s, oscillazione laterale 1,5 mm.
   - Infine avanti con il jitter del dito.
   
   Nessun WALK STOP salvo le inversioni avanti/indietro.
5. **STOP durante il joystick sterzato, poi STAND.** In chiusura PRONE.

**Criteri di interruzione:**
- qualsiasi FAULT (salvare subito `@MOTION DIAG`);
- `loop_stack_min_free` < 1 500 B;
- release perse > 1 al secondo;
- trascinamento o inciampo visibili.

**Rollback:** dev.4 con lo script pinnato.

## 16. Verdetto

**READY FOR SUPERVISED FLASH.** Si parte dalla fixture, poi il pavimento secondo il §15.

Tutte le prove offline passano, compresi i sei difetti storici riprodotti e corretti e la matrice di perturbazioni. Resta tutto **IMPLEMENTED NOT VALIDATED HARDWARE**:
- la legge joystick unificata;
- l'inviluppo WALK100 più stretto;
- il retry della correzione.

Nessuna probabilità di successo hardware viene dichiarata: i dati non la sostengono. Il rischio principale noto (§14.2) è un minore livellamento in WALK100, non una perdita di sicurezza.

Nota per la dev.4 installata: il difetto WALK100 `STOP_PLAN` esiste anche lì. Finché la dev.6 non è flashata, evitare WALK STANDARD 100 prolungati a terra (> 1–2 minuti) con la dev.4.
