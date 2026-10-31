# MATDOG — Consolidamento Motion, dashboard e PRONE — 30/10/2026

**1.0.1-dev.4 — IMPLEMENTED NOT VALIDATED.** Una candidata integrata, solo offline. Nessun flash, movimento, accesso hardware, merge, backup o push. Il robot mantiene rc.11 r2, APP `fc0a7141f827c37cdd5bccb888951ca280bd2a981b1572de8eef7c74474170be`. Worktree indipendente dev.3 conservato; baseline rc.11 e artefatti precedenti invariati.

## 1. WALK STOP: causa verificata e limite della diagnosi

La prima divergenza dimostrata è nel **feedback**, non nel gait: lo status operativo non identificava il comando cui apparteneva il completamento; la pagina poteva mantenere «movimento da confermare» dopo COMPLETED o sovrascrivere uno STOP recente con la risposta/timeout di un comando precedente. Ora status e UI correlano **id + sequence**, ACK riporta lo stato dopo l'operazione, e sono pubblicati accepted_ms/finished_ms. Il rinnovo di una vecchia sessione non reinvia un comando dopo uno STOP successivo. Un ACK perso può essere riconciliato con uno status identificato; uno stato di un altro comando non vale come conferma.

Percorso: dashboard POST con sessione/CSRF → HTTPD con controlli origin/schema → mailbox → MotionService → `SimpleMotion::walkStop()` → ACK → status dell'esecutore. WALK STOP a velocità zero è già esente dal budget Motion: il latch **non effettua I/O servo e non attende la fine del passo**. STOP di sicurezza usa la corsia prioritaria separata e resta invariato.

Il gait STANDARD mantiene la politica esistente: punto di arresto `max(1, ceil(cicli*4)/4)`, poi un ciclo di decelerazione e 0,075 ciclo di raccordo. A regime: 6,9875 s + attesa fino a 1,625 s + assestamento native; durante il primo ciclo l'attesa può essere maggiore. Cinque prove host osservano 13,540 / 10,360 / 8,660 / 7,180 / 7,760 s da ciclo 0,01 / 0,50 / 1,01 / 1,24 / 2,90. Sono tempi del modello software, non misure del robot.

**Il timeout browser di 5 s ha causa ancora UNKNOWN.** Non viene imputato alla chiusura del passo né nascosto allungando il timeout. Nei test threaded dei percorsi mailbox/STOP, WALK STOP passa senza slack in 1–3 ms; con HTTPD occupato artificialmente per 100 ms passa in 102–103 ms. Questi test non riproducono Wi-Fi/socket/TLS ESP32. Possibili attese di socket, handler o rete richiedono una traccia reale; non sono una RCA provata.

Diagnostica pronta: ring browser limitato a 48 comandi/240 snapshot, esportabile senza token/sessioni; tempi invio/header/body, ACK/errore, status con identità e tempi Controller. `Server-Timing: handler;dur=...` misura il solo handler prima dell'invio, non coda precedente né trasferimento. Già disponibili dispatch/loop, heap e stack minimi. Non sottrarre direttamente performance.now del browser dall'uptime ESP32. Nessun task o polling servo aggiunto.

## 2. Dashboard e Twin

Comandi accettati, osservati, completati e abortiti distinti; ACK con identità errata resta esito sconosciuto. Età feedback cresce anche senza nuove risposte. `/status` sospeso con 503/MOTION_BUDGET_UNAVAILABLE resta distinto da `/api/motion/status` attivo. STOP resta disponibile con lettura/comando normale pendente; WALK STOP resta accessibile quando l'ultimo stato è WALKING/MOVING anche se la telemetria è in ritardo.

Il Twin ascolta il parent prima del caricamento modello, usa i medesimi 12 ACTUAL e quaternion IMU, senza nuove implementazioni FK/IK nel browser né polling aggiuntivo. Eliminati i 500 ms di interpolazione aggiunta; modello preparato a SAFE_OFF e mantenuto chiudendo il pannello. A cache fredda durante Motion gli asset pesanti restano intenzionalmente differiti, con messaggio esplicito. Mesh e URDF sono byte-identici; bundle ricompilato. Non è stata verificata qui la resa WebGL su dispositivo né la latenza reale end-to-end.

Il buffer JSON HTTP è fisso a 5120 byte invece di 4096, per ID/tempi e `commanded_raw` letti dalla RAM. Caso lungo provato: 4283 byte con 640 byte riservati ai campi trasporto. Aumenta di 1 KiB la capacità del buffer di risposta; lo stack HTTPD è invariato. Stack minimo, dimensione risposta, RTT e missed releases vanno osservati sul robot; nessuna pretesa di WCET hardware qualificato.

## 3. STAND: variante esclusa

L'evidenza di slittamento è riportata dall'operatore. Negli artefatti esaminati non sono stati trovati i nuovi video/screenshot del 30/10: nessuna misura ottica inventata. La variante studiata conserva la separazione fino al knot42, poi anticipa gli anteriori e ritarda i posteriori di fino a 12 knot, raccordando a100; LIFT invariato. Produzione, gen1, collision box e profilo servo non modificati.

| DEPLOY, modello offline | Attuale | Variante sovrapposta |
|---|---:|---:|
| Durata planner | 3,183 s | 2,867 s |
| Inizio estensione anteriore/posteriore, soglia10 mrad | 1,79 / 0,50 s | 1,54 / 0,50 s |
| Pitch di riferimento max | 10,198° | 10,211° |
| Corsa X piedi posteriori nel riferimento | 47,005 mm | 47,001 mm |
| Velocità tangenziale max vicino al piano nominale | 51,106 mm/s | 40,877 mm/s |
| Altezza piede max sul piano nominale | 0,0004 mm | 4,107 mm |
| Margine statico CAD minimo | 85,608 mm | 85,606 mm |

La velocità tangenziale diminuisce circa20%, ma cambiano i contatti, il pitch non diminuisce e la corsa posteriore rimane47 mm. **Non c'è beneficio sicuro dimostrato: nessuna nuova route nella candidata.** I dati completi contengono tutti12 target, corpo/pitch e FK piedi. Il riferimento nominale non è una soluzione dinamica a contatti fissi: anche la route attuale presenta quote nominali negative fino a−6,49 mm; non sono misure di penetrazione fisica. Il modello statico con CAD/URDF, tutte17 mesh e massa2,8 kg non determina attrito, compliance o redistribuzione dinamica. LIFT nominale2,164 s, pitch10,105→0°, corsa piedi anteriori2,784 mm/posteriori0,056 mm; invariato. Il test Controller con carico sintetico distingue invece durate realizzate: DEPLOY3,80 s, LIFT2,78 s. Misurare i contatti/slittamento nella prova, senza attivare questa variante.

## 4. PRONE e HOLD

SimpleMotion è **byte-identico a dev.3**: HEIGHT→53 mm, LIFT/DEPLOY inversi, floor LOWER anteriori+20 mrad, chiusura HIP±0,17, terminale torque ON `PRONE_CONTACT_UNVERIFIED`. Nessun SAFE_OFF automatico o recovery su fault. HOLD non modificato; sei ore virtuali verificano la persistenza software, non temperatura/endurance motori.

Replay69/69 al terminale. Minimi gen1/box in tick: STAND50 **17,000 / 10,009**; STAND100 **15,000 / 5,675**; STAND150 **18,000 / 9,213**. Residui HOLD mantenuti costanti su servo sintetico: non sono margini misurati in discesa. Solo da50 è proposta la prima esplorazione supervisionata. A100mm, dopo la perturbazione geometrica ±4tick dello studio precedente, restano solo1,675tick dalla box: non è una misura dell’incertezza reale e ±8tick non è coperto. Il vecchio margine di2 tick resta corretto come in dev.3, senza ulteriori modifiche.

Il ventre posteriore nominale resta20,322 mm alto, circa60% del peso sui posteriori; il distacco anteriore nominale di0,17 mm non prova l'appoggio. Encoder/IMU/carico non autorizzano torque OFF libero. SAFE_OFF non comanda il ripiegamento: la posa parzialmente estesa osservata dopo torque OFF è compatibile con assestamento passivo, ma senza foto/filmato non si può stabilirne la causa. PRONE dev.3 non era installata durante quelle osservazioni.

## 5. Consegna

Sorgente candidata CLEAN: `babb8a5a23da8f6d544cf8209c175be99cd8299e`, derivato da `f980fee5cd69f1e6bd38066981a5be40f47aa0fc`. Commit A `666d79b`, B `1ca2933`, strumenti/versione `babb8a5`. Nessun delta route C; D PRONE preesistente preservato. Dettagli build/SHA nel file BUILD.md; prove in RESULTS.md, procedura unica e criteri di congelamento in HANDOFF.md. Diff separati e completo nella directory patches. Tutti gli esiti nuovi sono offline; nessuna promozione a VERIFIED HARDWARE.
