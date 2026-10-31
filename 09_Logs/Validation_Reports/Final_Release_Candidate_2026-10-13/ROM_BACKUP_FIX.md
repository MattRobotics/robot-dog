# Riparazione backup ROM e ripresa — 13 ottobre 2026

**PASS software: 23 test offline.** Nessuna nuova build: app `5577bd3c8da5f6c44d8fb9ca9b3d049399e73ccf37d218e716b773e3a87b5914`, source compilata `b147f388`, manifest e designation invariati. Sono modificati soltanto preflight, review, wrapper APP-ONLY/lettura partizioni, test e controllo di provenance host; digest esatti nel [receipt separato](PREFLIGHT_SERIAL_FIX.json).

La sessione privata `20261013T170557Z` ha concluso diagnosi seriale, identità `b08322d`, preflight 12 giunti e 13 torque OFF. Il successivo read-flash a 921600 baud con deadline 900 s è scaduto. Lo storico del [25 settembre](../../Development_Log/2026-09-25_RECOVERY_BACKUP_AND_CANDIDATE_V3.md) e del [2 ottobre](../../Development_Log/2026-10-02_M0_2_HARDWARE_ALIGNMENT.md) attesta invece ROM/no-stub a 115200, con tempi 1525,8 / 1541,7 / 2145,3 s. Lo stub non viene riprovato.

Il percorso corretto usa `--no-stub --baud 115200 --connect-attempts 1`, deadline totale **3600 s per immagine**, `--before no-reset --after no-reset` durante le letture. Il processo scrive progressi immediatamente nel log privato. Output parziale, exit nonzero o timeout sono conservati, con STOP e nessun retry. File storici e log non vengono sovrascritti. A/B devono avere 16.777.216 byte, SHA uguali e confronto byte per byte positivo; seguono layout, slot corrente, checksum/SHA app e gen-1 validato con codice C++ di produzione.

`--resume-rom-session <report>` rivalida le 23 risposte precedenti senza inviare comandi runtime a una ROM o a uno stub. Le etichetta `PREVIOUS_PRE_ROM_CAPTURE_NOT_FRESH` e registra origine/digest. L’identità corrente è nuovamente letta dalla ROM; il backup contiene la sorgente installata osservata. Le evidenze servo/IMU/BMS non sono rese fresche dal backup. Un reset di alimentazione servo o cambiamento d’installazione ne invalida l’uso preflash. La review ora richiede conferma esplicita di continuità dall’osservazione e presenza fisica con arresto accessibile: nessuna inferenza automatica.

`--enter-rom-once` autorizza un unico ingresso iniziale USB-JTAG nella ROM, prima del read-mac, per eliminare l’ambiguità di uno stub rimasto in RAM. Non viene ripetuto dopo errori. Il reset riguarda l’MCU, non è un readback dei servo. Senza l’opzione, la ripresa usa soltanto no-reset e si ferma se la ROM non risponde. Nessun bootstrap che conceda Motion, nessun writer flash/NVS/servo/BMS nel preflight.

Dopo la qualificazione del tool, la ripresa autorizzata sull’ASUS è:

```bash
python3 /home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog/05_Firmware/MATDOG_Controller/scripts/release/motion_remote_preflight.py --execute-readonly --confirm-asus-independent-maintenance-link --resume-rom-session /home/matteo-manicardi/MATDOG/private-preflight/20261013T170557Z/preflight_report.json --enter-rom-once
```

A/B locale non equivale a recupero indipendente. Il [runbook cifratura/recupero](../../../10_Backups/ESP32/README.md) richiede mezzo fisicamente distinto oppure GPG/AES-256 off-PC, download, decifratura e confronto. Credenziale solo pinentry sotto controllo dell’operatore, mai generata o esposta da Codex. Senza recuperabilità verificata: NO-GO; nessun flash. Anche dopo il recupero restano continuità/power/periferiche e supervisione fisica da attestare. L’autorizzazione del proprietario identifica già lo SHA esatto; non sostituisce questi gate.

[Log 23 test](test_logs/preflight_rom_backup_fix.log) · [Procedura completa](REMOTE_PREFLIGHT.md). Gli esiti hardware effettivi sono registrati separatamente nel report privato della nuova sessione; questo receipt qualifica soltanto il tool.

Il flasher canonico e il lettore delle partizioni ora applicano lo stesso trasporto ROM 115200/no-stub/no-reset, con un tentativo di connessione. Il flasher mantiene tutte le verifiche di backup, source/profile, layout/range e una sola chiamata write-flash; la verifica indipendente resta obbligatoria e il dispositivo rimane in ROM. Il riavvio post-flash è una fase supervisionata separata, soltanto dopo esito positivo. Letture tabella/otadata: timeout finito 120 s e lunghezza esatta obbligatoria. Il test di layout ha conservato tutti i controlli, adattando solo il riconoscimento della riga ai nuovi argomenti.

[Audit mirato APP-ONLY PASS](test_logs/rom_backup_fix_targeted_audit.log): quattro controlli originali e le relative suite di manifest/layout/backup. L’audit generale è stato fermato perché includeva regressioni firmware già qualificate; non è dichiarato un nuovo PASS generale. Il codice firmware congelato resta byte-identico.
