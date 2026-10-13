# Verifica finale e limiti delle prove

La campagna offline finale esegue RESTORE/persistenza, Service/Coordinator/Executor/Safe Actuator, AdaptiveStand, sessioni/permit, prime/readback, STOP/watchdog/rearm, HTTP di produzione, AP/radio, DOM Dashboard e preflight/backup logico. I certificati continui esistenti vengono verificati per digest/status; non vengono rigenerati. I log e le identità esatte sono nel [TEST_RECEIPT](TEST_RECEIPT.json).

| Evidenza | Ambito e criterio |
|---|---|
| Planner finale | 10.929 target STAND / 251 segmenti; 19.080 target WALK; confronto con adapter STL indipendente entro 1e-7 rad, errore misurato 5,958840621e-8 rad |
| Pipeline quantizzata | Service → Coordinator → AdaptiveStand/FeedbackWalk → Executor → Safe Actuator, 360.120 scritture raw simulate con 12 trasformazioni individuali; nessun servo reale |
| Pose intermedie | segmenti 3/18/40/120/230 a progress .37 ammessi entro tube; perturbazioni opposte/asimmetriche e pose sconosciute negate; STOP richiede nuovo grant/feedback, nessun recupero arbitrario |
| Autorità | scope fixture/STAND/WALK separati; limite RF restrittivo rifiuta WALK prima del grant; ownership locale permane tra comandi; Web/LAN non concedono authority; scadenza e STOP chiudono la sessione |
| Certificati/payload | digest dei 3 certificati, 19.081 righe q/v/a, min/max e quantizzazione; originale SHA f86ad336… invariato; nessuna soglia allargata |
| Derivate/switch | smoothing dell’intero adapter, derivate complete posizione/velocità/accelerazione; quattro witness switch storici conservati; prova continua finale su box indipendenti + tube 6 mrad, sway 30 mm |
| LED | suite host e mutation test del full static audit: SOC 30/15, isteresi 33/18, UNKNOWN/stale, charging continuo 2→60, allarmi; invarianti RMT DMA/GPIO47/12/mappa; nessun write KEY/MOS aggiunto |
| Rete | veri handler e policy: cinque GET LAN, POST/config/OTA/grant negati, Host/Origin/CSRF/sessioni/radiogeneration, listener closure/AP recovery/STA_STOP; privacy nomi/scans e nessun AP+STA |
| Dashboard | DOM 167 controlli manutenzione e 48 azioni; Chrome desktop/mobile, 6 schermate, LAN 48 disabled, AP, 17 STL, stale; visual fixture separata dall’enforcement HTTP C++ |
| Preflight | PLAN/CLI/sintassi; 23 letture compatibili col sorgente storico b08322d…, decoder gen1 di produzione, ESP checksum/SHA, corruzioni/backup ambiguo/partial review negati; USB mai aperto |

Il full static audit già concluso PASS viene riusato per LED/rete/NVS/startup e boundary invariati: le ultime modifiche cambiano soltanto parametri fixture/WALK, dataset/pin, oracle/test e documentazione. Questi cambiamenti hanno regressioni finali dedicate, digest verificati e compilazione completa ESP32-S3. La campagna precedente con fixture di test obsolete FAIL rimane nei log locali; non viene trasformata retroattivamente in PASS.

**BLOCKED fisico:** attualità/integrazione gen1 e mapping, 0/12 limiti approvati sull’installazione corrente, posa/fixture/free-volume reali, power cut, clock/bus worst-case e freshness, conversioni drive/envelope/output/tracking/uncertainty totale, prime/hold senza salti. Per supporto portante: appoggi/forze, altezza assoluta/observer qualificato, massa/COM/compliance/attrito; per WALK caricato anche dinamica. Encoder/IMU/corrente e simulazione non chiudono questi gate. Il ramo originale a −92° resta NO-GO; il ramo 40 mm RF oltre limite resta storico escluso. Nessuna nuova Full Calibration richiesta offline.

**BLOCKED flash finché acceso e reviewato:** identità/layout/slot correnti, tredici OFF, gen1 integro, snapshot IMU/DALY/elettrici/termici, due full flash SHA uguali, recupero indipendente verificato e autorizzazione separata per il SHA dell’unico candidato. Il software di preflight non invoca il flash.
