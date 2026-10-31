# MATDOG — Qualificazione hardware successiva, distinta dal flash

**NON ESEGUITA. Nessun hardware PASS.** È una procedura futura per PRONE→STAND sulla fixture, subordinata alla decisione sul candidato e al completamento separato della [procedura APP-only](APP_ONLY_FLASH_PROCEDURE.md). Il NO_GO attuale del workflow è nella [review](FINAL_PREFLASH_REVIEW.md).

## Preparazione

- Fixture/supporto e taglio fisico pronti, area libera e operatore presente. Nessun WALK/FAST o comando persistente concorrente in questa campagna STAND. Non aumentare stack, timeout, tracking o limiti per superare un rifiuto.
- Batteria valutata con tensioni cella minima/pack, allarmi e freschezza, non dal solo SOC. Una situazione equivalente a 3,19 V minima/9,7 V non è adatta alla prova; va risolta prima. Preservare le protezioni DALY e servo. Il preallarme è advisory, non prova di capacità residua.
- Verificare di nuovo binario/build ID, boot/sessione OFF e safety; completare preflight servo e RESTORE automatico previsti. Nessuna riconferma ripetitiva di hardware già riconosciuto. Non scambiare Authority GRANTED/READY/RUN con torque ON.
- Un solo proprietario USB. Se serve il Twin, il bridge è quel proprietario e produce il log; altri client usano la sua console/feed. Se si usa il logger originale, il bridge non può aprire la stessa USB. Preparare prima una prova del bridge su asset host e AP telefono/PC; la CSP del candidato attuale blocca l'iframe integrato, quindi non dichiararlo disponibile senza un successore o accettazione esplicita del debito.
- Annotare UTC ISO8601, Europe/Rome con offset, fonte host, boot nonce e uptime. Il logger storico è UTC anche se mostra soltanto l'ora per riga. Non cambiare clock DALY o safety per allineare la visualizzazione.

## Verifiche prima dell'unico STAND

1. Con torque OFF, acquisire 12/12 RAW, ID/slot corretti, read-ok, timestamp per giunto, età, epoch/generation, lease, feedback mappato, geometria/calibrazione e profilo. Conservare un riferimento al nonce del boot.
2. Registrare heap libero/minimo/largest, stack minimo libero del loop, massimi gap loop/dispatch. Il watermark è minimo storico del boot, non spazio istantaneo: acquisire baseline e variazioni. Non attribuire un minimo heap basso a una perdita senza confrontare il libero recuperato e il largest.
3. Dashboard AP autenticata visibile: presenza periodica entro 3 s, capability fresche e `readiness.permit` corrente. STAND disabilitato quando il gate non ammette; motivo reale visibile. Refresh breve può riprendere la sessione cookie, ma pagina nascosta/chiusa oltre deadman deve revocare il permesso. Non testare perdita presenza su un robot non sostenuto.
4. Confermare raggiungibilità STOP globale con fixture sicura e percorso di taglio fisico indipendente. La corsia STOP del Controller è prioritaria dopo ricezione, ma HTTP single-task può ritardare la ricezione; non equiparare il pulsante al taglio hardware.

## PRONE→STAND e criteri di arresto

Inviare un solo STAND esplicito nelle condizioni ammesse e attendere il suo esito. Se HTTP timeout rende il POST non confermato, non ripeterlo automaticamente: osservare stato/sequence e usare STOP secondo la procedura sicura. Una richiesta ricevuta può essere stata accettata anche se il browser non ha ricevuto la risposta.

Acquisire la riga `MOTION_ADMISSION` con now_ms, epoch, generation, lease e tutti i giunti `slot:timestamp:signed_age:read_ok`; questa è la prova al gate. La traccia può essere troncata/scartata per backpressure: una telemetria successiva 12/12 non sostituisce il dato mancante. Non trasformare una traccia incompleta in un rifiuto inventato né in un PASS.

Registrare separatamente gate/ammissione, PRIME, energize/readback profilo, ARM, entry e STAND_VERIFIED, con errore tracking per giunto e stato BMS/IMU/supporto. La simulazione precedente non è una previsione del tempo fisico o della coppia richiesta.

Arrestare per qualunque fault reale, tracking persistente, feedback stale, supporto/presenza persi, profilo non verificato, allarme BMS, reset, corruzione o calo anomalo del margine stack/heap. Per interpretare lo stack: il percorso STAND conservativo ha 11440 B noti su 16384, ma non include tutti i costi runtime; il dato libero fisico deve risultare coerente e con margine prima di ripetere. Se emerge un margine insufficiente, analizzare i frame, non alzare lo stack sottraendo heap senza misura.

Verificare STOP e SAFE_OFF/readback, poi la perdita controllata del solo heartbeat con robot sostenuto, senza fare affidamento sul Twin per rinnovare presenza. Preservare il limite 3000 ms: un'interruzione della rete deve causare arresto sicuro, non permesso permanente.

## Esito della campagna

Solo i log fisici possono qualificare PRONE→STAND. Una prova riuscita deve legare immagine esatta, nonce, raw al gate, tracking, energia, stack/heap, tempi STOP/deadman e safe-off verificato. I casi non osservati restano OPEN. Non estendere il risultato a WALK/FAST: la catena generica WALK ha un rischio stack distinto già documentato e richiede un audit dedicato prima dell'uso.
