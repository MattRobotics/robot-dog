# Note operatore — prova MATDOG a terra del 08/11/2026

**Queste sono osservazioni e richieste dell'operatore, non diagnosi dimostrate dai file.**

1. Al primo KEY ON la parte posteriore era leggermente sollevata; il primo STAND150 è stato abortito con messaggio di posizione iniziale fuori dalla regione d'ingresso verificata.
2. Dopo aver riposizionato manualmente il robot, STOP non ha eliminato lo stato di FAULT. L'operatore ha dovuto spegnere e riaccendere per riprendere. Requisito richiesto: riconoscimento autonomo, da torque OFF e sensori validi, della posa nuovamente ammissibile e ritorno a READY **senza STOP e senza avviare movimento automaticamente**. Il rifiuto iniziale della posa non va aggirato.
3. Dopo il power-cycle, STAND ha funzionato. WALK STANDARD 100 a terra è avanzato con falcata molto più ampia rispetto alla versione precedente. La velocità apparente di rotazione dei giunti è rimasta simile; l'effettivo spostamento metrico non è stato misurato.
4. Joystick a terra: avanti, diagonale, laterale, indietro e rotazione risultano operativi, ma l'operatore riferisce sway laterale marcato anche quando chiede soprattutto avanzamento frontale e velocità laterale/angolare troppo basse.
5. STOP e WALK STOP sono percepiti come pronti e consentono le ripartenze dirette da STOP; comandi STAND a quote diverse e PRONE hanno funzionato nella sessione registrata.
6. Al termine dell'attività MATDOG è stato spento e scollegato dalla USB. Nessuna ulteriore prova hardware autorizzata durante la notte.

Da verificare nel sorgente e nelle tracce, senza presumere la causa: recupero del latch di initial-pose, jitter laterale e legge di sway, rapporto tra gait joystick e WALK STANDARD, temporizzazione delle release e limiti di stabilità del joystick.
