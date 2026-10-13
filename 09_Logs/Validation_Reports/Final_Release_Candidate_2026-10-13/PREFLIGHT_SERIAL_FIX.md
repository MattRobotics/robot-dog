# Correzione preflight seriale — 13 ottobre 2026

**PASS offline.** Il primo tentativo reale si era fermato con `installed source identity unavailable`: la firma iniziava in una risposta, mentre il GIT_SHA completo compariva nella sezione attribuita al comando successivo. Anche il primo STATUS aveva raccolto output di boot. Il vecchio criterio di silenzio tra byte non delimitava una risposta multilinea.

Il tool host ora invia una sola richiesta alla volta e attende tutti i record ordinati previsti dal CommandRouter installato `b08322dedc5564c080f691dc98047511ef2b953f`. SOURCE_SIGNATURE richiede firma, SHA completo di 40 cifre, versione/profilo/schema/flag coerenti e record terminale della partizione. Timeout totale finito: 20 s per le letture ordinarie, 60 s per SERVO PREFLIGHT; il silenzio non costituisce completamento.

Authority e calibrazione richiedono le rispettive risposte complete, senza ownership o permit attivi. PERSIST richiede gen-1 acknowledged integro e marker IDLE (lo stato normale del writer è OPEN, distinto dal marker). SERVO PREFLIGHT richiede STARTED, risultato PASS, 12 righe uniche coerenti/OFF e footer finale SERVO. Ciascuno dei 13 SERVO READ richiede l’ID corrente, tutti i campi, posizione valida e torque OFF. Righe frammentate e CRLF sono ricomposte; il log conserva anche boot e telemetria asincrona, che non soddisfano i predicate. Nessun reset del buffer RX cancella evidenze. Risposte precedenti, duplicate, fuori ordine, incomplete, errori o identità discordanti interrompono la sessione; nessun comando successivo né backup ROM viene avviato dopo un errore.

**16 test PASS**, con GIT_SHA ritardato a 1,2 s e coda a 1,4 s: AUTHORITY viene inviato soltanto dopo la coda. Coperti boot ritardato, framing dei 23 comandi, campi mancanti/discordanti, timeout, errori, ID/OFF, preflight incompleto e provenance negativa. I contratti delle 10 risposte già presenti nel log privato sono stati verificati offline; non sono nuove misure hardware. [Log test](test_logs/preflight_serial_fix.log) · [Receipt e digest](PREFLIGHT_SERIAL_FIX.json).

Il controllo host di provenance distingue questa precisa riparazione dalla sorgente compilata. Richiede checkout pulito, digest vecchi e nuovi dei soli tre file host nominati nel receipt e SHA dell’app invariato. Qualsiasi altro cambiamento a firmware, input Motion o tool viene rifiutato. Nessun gate di authority, backup, layout, identità, review o autorizzazione del flash viene rimosso. Il receipt host è separato dal manifest firmware.

**Immagine invariata:** sorgente compilata `b147f3884b227ec819e98a6f00c214a0aac2278e`, 3.643.120 byte, SHA256 `5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`. Manifest, designation, calibrazione e NVS invariati. Nessuna compilazione, apertura del dispositivo, reset, backup, flash o movimento effettuato per questa patch. Non sono state eseguite regressioni Motion estranee.

Aggiornare il checkout dell’ASUS con questa patch prima di riprovare. Preservare il precedente log privato. Con collegamento remoto del PC indipendente dalla radio del robot, rieseguire:

```bash
python3 /home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog/05_Firmware/MATDOG_Controller/scripts/release/motion_remote_preflight.py --execute-readonly --confirm-asus-independent-maintenance-link
```

Il comando mantiene il workflow esistente: prima diagnosi USB read-only; soltanto dopo tutti i gate, reset al ROM e due backup completi confrontati. Non esegue flash né movimento e lascia il robot nel ROM dopo il backup. La sessione produce un nuovo report privato; serve ancora la review del backup recuperabile e l’autorizzazione separata per l’esatto SHA designato. [Procedura completa](REMOTE_PREFLIGHT.md).
