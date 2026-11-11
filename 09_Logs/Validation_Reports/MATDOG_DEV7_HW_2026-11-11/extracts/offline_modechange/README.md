# Strumenti offline di riproduzione (copiati da /tmp/claude-recover/mv2/tools, 11/11/2026)

Sorgenti delle sonde host usate in sessione; i binari restano fuori dall'archivio (rigenerabili).

- `build_transition.sh SKETCH OUT MAIN_INC`: compila `test_simple_motion` + `servo_plant.inc` + il `main` indicato contro i sorgenti di uno sketch.
- `modechange-main.inc`: walk preset FAST a 105 mm, poi selezione di un modo più lento sotto una griglia di inclinazione statica del rig (roll 0/±8, pitch 0/±6/±10 mrad). Argomento: modo di destinazione (0 = SLOW, 1 = STANDARD).
- `modechange_run_<commit>_to<0|1>.txt`: esecuzioni rifatte l'11/11 sera sui binari costruiti da `450d2c7` e `b561062` (stesso output riga per riga). Risultato OFFLINE: FAST→SLOW resta pendente per 80 s in 3 punti su 16 (roll ±8 con pitch +6/+10 mrad), negli altri cambia in 10–30 s; FAST→STANDARD cambia in tutti i 16 punti. Output identico nei due commit: non è una regressione dev.7. Il rig statico non riproduce il FAST→STANDARD pendente visto sulla fixture (ipotesi: correlazione di fase, vedi OPEN_ITEMS P1).
- Gli altri `.inc` / `audit-plant.py` sono le sonde della campagna Motion vNext (finestre di sway, transizioni, audit plant).

Ricompilazione (esempio): `bash build_transition.sh <worktree>/05_Firmware/MATDOG_Controller /tmp/mc/c7 modechange-main.inc && /tmp/mc/c7/bin 0`.
