# dev.7 — punti aperti dopo la validazione hardware dell'11/11/2026

Ordine di priorità concordato con l'operatore. Ogni modifica parte da `b561062`, con un nuovo commit e una build CLEAN. Valgono i vincoli esistenti: nessuna modifica a box, limiti, tolleranze, deadline, STOP/StopPlan, NVS, calibrazioni o EEPROM servo senza una decisione tecnica esplicita. Riferimenti ai sorgenti relativi a `05_Firmware/MATDOG_Controller/` su `b561062`.

## P0 — Timing Motion

**Fatto (VH):**
- Fino a 43 release perse in 30 s a terra (19:44:31–19:45:01). Criterio operativo: più di 16 in 30 s.
- Release perse a terra 146→328: 101 nelle camminate sterzate col joystick, 54 nelle riprese STAND_FROM_STOPPED_WALK.
- Sulla fixture: 14–23 per ripresa dopo STOP (riferimento 10/11: 6–12).
- Picco dall'avvio 46,69 ms (fixture F3a: read 3,4 / nominal 14,3 / compensate 26,0 ms).
- A terra: compensate fino a 24,57 ms e nominal fino a 18,3 ms, con una sola chiamata solve, 7–8 IK, 11–12 Newton e 21–23 FK per release.
- Le sezioni sono tempo a muro, prelazione inclusa (`src/simple/MotionTiming.h:12–13`).
- Nessun `MOTION_DEADLINE_LOST`. `max_gap` invariato da 140,8 ms (primo ACQUIRE a coppie OFF).

**Obiettivo:** trovare la prima divergenza (quale sezione e quale release superano per prime il budget di 20 ms, `kBudgetUs`) e fare una patch minima. IK, STOP e protezioni vanno preservati.

**Non fare:** attribuire la causa a cache, CPU o servo senza una misura. Prima va separato il calcolo dalla prelazione: con così poche chiamate (una solve) il tempo a muro di 22–26 ms non è ancora spiegato. Strumento esistente solo diagnostico: `MATDOG_DIAG_PREEMPTION` (`MotionTiming.h`), mai nella build operativa.

**Riferimenti:**
- serializzazione: `src/core/MotionService.cpp:249–262` (`command_timing`, `timing.max_work`, `max_work_meta`);
- dati: `HW_REPORT.md` §6 A1 e A3; `extracts/ground2_summary.txt`;
- storico: `../Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md`, `../Motion_Perf_2026-11-07/`.

**Successo:**
- a terra, nelle stesse prove (sterzata STANDARD con yaw, STOP, ripresa): perse ≤ 16 in 30 s;
- nessuna release oltre 46,7 ms;
- STOP e ripresa invariati;
- regressioni offline (`test_simple_motion`, `servo_plant`, `static_audit`) PASS.

## P1 — Cambio da FAST verso un modo più lento

**Fatto:**
- Sulla fixture, FAST→SLOW e FAST→STANDARD non si sono mai completati: FAIL (VH).
- A terra, FAST→SLOW si è completato in 5 s (VH).
- Offline, su `450d2c7` e `b561062` con output identico, FAST→SLOW resta pendente per 80 s in 3 dei 16 punti di inclinazione statica, mentre FAST→STANDARD si completa sempre (VO, `extracts/offline_modechange/`).
- Quando il cambio è pendente non c'è un timeout, e la dashboard mostra solo `WALK_MODE_CHANGING`.

**Meccanismo:** `stepWalkMode()` (`src/simple/SimpleMotion.cpp:1088–1140`) cambia prima il clock e la quota. Cambia la forma solo al rilascio di una finestra a quattro contatti, e solo se la correzione IMU applicata, proiettata con `projectWalkTilt` (riga 76), sta nell'inviluppo del modo di destinazione (riga 1138).

**Ipotesi non verificata:** sulla fixture la correzione esce dall'inviluppo proprio all'inizio delle finestre (correlazione di fase): 15 campioni fuori su 44 durante FAST→STANDARD.

**Obiettivo:**
- evitare il cambio pendente senza fine: durante il cambio, limitare la correzione all'inviluppo del modo di destinazione, come fa HOLD con `pendingWalkTilt()` (righe 1002 e 1190);
- in alternativa, rinunciare al cambio in modo esplicito;
- mostrare in dashboard un motivo comprensibile (nuovo reason, testo in `src/network/MotionDashboard.h:142`).

**Non fare:** allargare o aggirare l'inviluppo certificato.

**Successo:**
- sonda `offline_modechange` sulla griglia: 0 pendenti oltre un limite dichiarato, oppure un motivo esplicito per ognuno;
- regressioni dei cambi di modo PASS;
- su hardware: FAST→SLOW e FAST→STANDARD sia sulla fixture sia a terra.

## P2 — STANDARD a 25 mm/s

**Fatto:** l'operatore chiede STANDARD a circa 25–26 mm/s. Oggi è 30 mm/s, troppo vicino ai 32 di FAST (VH, osservazione).

**Obiettivo:**
- valutare il riuso del preset STANDARD da 75 mm già verificato come massimo (25 mm/s), in `kWalkModes` (`SimpleMotion.cpp:88`);
- verificare la compatibilità con il joystick (`JoystickSpec` STANDARD, riga 114) e con i limiti mostrati in dashboard;
- patch mirata e regressioni pertinenti (gate joystick, `walk_support_check` e inviluppo, `servo_plant` STANDARD, DOM della dashboard).

**Successo:**
- STANDARD al massimo 25 mm/s nel preset e col joystick;
- supporto statico, cattura dello STOP e margine HIP non peggiori dei valori di `AUDIT_FIXES.md` §3.

## P3 — Spostamento laterale del baricentro

**Fatto:**
- Lo spostamento laterale è marcato e visibile nelle camminate laterali e diagonali (VH, osservazione).
- Ampiezze di progetto: `kJoystickSwayYM=.010` (`src/simple/SimpleMotion.h:92`), STANDARD .009 e FAST .0075 (`SimpleMotion.cpp:114`, `:123`), contro 1,25–1,5 mm in avanti.
- La funzione `joystickSwayYM()` (`SimpleMotion.h:114–118`) porta l'ampiezza al valore pieno già da una richiesta laterale o di rotazione del 3 % (`kJoystickSwayOnDemand`).
- In FAST, 6,5–7 mm davano supporto negativo offline e 10 mm faceva fallire STOP_PLAN all'avvio sterzato (`SimpleMotion.cpp:115–117`).

**Obiettivo:** verificare offline se l'ampiezza può crescere in proporzione alla richiesta laterale senza peggiorare supporto statico, margine HIP e STOP (gate joystick rigoroso, `servo_plant`, matrice degli STOP nelle finestre).

**Non fare:** modificare box o limiti per un miglioramento solo estetico.

**Successo:** gate e `servo_plant` PASS con margini non peggiori; spostamento ridotto alle richieste laterali basse.

## P4 — Letture spurie di temperatura

**Fatto (VH):**
- In sessione, 28 picchi isolati scartati.
- Su 105 riletture dirette, 26 hanno ripetuto il valore alto (71–150 °C), mentre il campione successivo era tra 29 e 36 °C.
- Nessuna finestra ha superato la conferma termica: `kHotC=70 °C` per `kHotConfirmUs=500 ms`, `kHotGapUs=250 ms` (`SimpleMotion.cpp:28–42`, `:1634–1635`).
- Più frequenti a terra e sotto carico.

**Obiettivo:**
- conservare le evidenze (`thermal.last` in `usb/http.jsonl`);
- trovare la causa: cosa restituisce il servo e la rilettura diretta (`Controller.h:214`, `SimpleMotion.cpp:150`).

**Non fare:** indebolire la protezione termica (soglie, tempo di conferma, continuità).

## Configurazione futura — Jetson Orin

Le prove dell'11/11 sono senza Jetson (175 g, inclusa nel modello). Quando verrà montata:
- registrare la sua posizione nel `base_link`;
- rivalutare massa e COM;
- ripetere sotto carico almeno: STAND con M1, indietro con STOP in ogni modo, STOP in avanti in ogni modo.

## Minori, non bloccanti

- F4b (selezione del modo con la leva tenuta) non eseguito: va fatto con due ingressi (touchscreen o secondo dispositivo).
- Camminata continua di 2 minuti non eseguita: la più lunga è stata di circa 76 s, con inversioni.
- STOP su tre contatti osservato e mantenuto una volta a terra, non certificato offline (`AUDIT_FIXES.md` §6).
- Cavo USB di circa 1 m: per le prove a terra serve un cavo più lungo, così da non dover spostare a mano il robot in HOLD.
