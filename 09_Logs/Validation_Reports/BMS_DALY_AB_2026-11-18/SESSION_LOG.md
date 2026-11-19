# DALY A/B — registro della sessione hardware (18/11/2026, autorizzazione esplicita dell'operatore)

Tempi UTC. Strumenti: `tools/` (copie di `MATDOG_BMS_DALY_RCA_V2_FABLE_20261118/ab_plan/`, con lettura a robot già acceso), `flash/` (catena di flash qualificata della sessione 30c894a, sole pin cambiate).

| Ora | Evento | Esito |
|---|---|---|
| 17:49:21 | accensione dell'operatore, robot spento da 13:21:31 (268 min); firmware `30c894a` | avvio a freddo del braccio B, non registrato dall'inizio |
| 17:53:40–18:07:40 | `B0_30c894a_cold_1749`: `@BMS STATUS` ogni 2 s da 260 s | TIMEOUT fino a 307 s, poi 395 campioni validi |
| 18:07:57 | storico LED di `30c894a` | READY→FAULT a 217,4 s di uptime, FAULT→READY a 308,8 s: silenzio DALY di ≈ 92 s |
| 18:07:49 | controlli prima del flash (`pre_flash_read_30c894a`) | identità `30c894a0d430`, app0 `0x010000`, READY, SAFE_OFF 8191, coppie OFF, DALY OK 11,2 V 65,6 %, allarmi 0000, NVS gen 1 VALID_ACKNOWLEDGED |
| 18:08:1x | ROM entry, layout del dispositivo, digest dell'installato | `ROM_CHECKS = PASS`: tabella `8f756ecb…`, app0 attiva, installato = `03c863cc…` (rollback esatto) |
| 18:08:29–18:09:13 | flash APP-only di `1.0.1-dev.5` (`c8b27d67…`, 1 634 240 B) a `0x010000` | `Hash of data verified`, `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS` |
| 18:09:17 | uscita dalla ROM con watchdog reset; `A0_dev5_postflash_boot` (4 min) | `1.0.1-dev.5` `df43feb94cdb`, NVS READY VALID_ACKNOWLEDGED, restore gen 1 12/12 `torque=UNCHANGED_OFF`, census 13/13, READY, SAFE_OFF 8191, coppie OFF, DALY 118/118 validi |
| 18:14 | `tools/ab_cycle.sh A1_dev5_cold 12` armato | in attesa del KEY OFF dell'operatore |
| 18:24:36.8 | KEY OFF dell'operatore (journal del PC: `usb 5-2: USB disconnect` 19:24:36+01:00) | registrato in `A1_dev5_cold_key_off_utc.txt` |
| 19:42:04–05 | KEY ON dell'operatore dopo 77,5 min di spegnimento (journal: nuova enumerazione 20:42:04+01:00) | avvio a freddo valido; `reset_reason=POWERON`, `1.0.1-dev.5` `df43feb94cdb` |
| 19:42:05–19:54:05 | `A1_dev5_cold` (12 min, `@BMS STATUS` ogni 2 s) | **CLEAN**: 356 campioni da 6 a 718 s, 356 validi, buco massimo 4 s; NVS VALID_ACKNOWLEDGED, restore gen 1 12/12 `torque=UNCHANGED_OFF`, census 13/13, READY, nessun comando Motion |
| 19:55 | `tools/ab_cycle.sh A2_dev5_cold 12` armato | in attesa del KEY OFF dell'operatore |
| 19:56:13.9 | KEY OFF dell'operatore per il ciclo 2 (journal: `usb 5-2: USB disconnect` 20:56:13+01:00) | registrato in `A2_dev5_cold_key_off_utc.txt`; registratore in attesa del KEY ON |
| 21:24:51.8 | KEY ON dell'operatore dopo 88,6 min di spegnimento (journal: enumerazione 22:24:51+01:00) | avvio a freddo valido; `reset_reason=POWERON`, `1.0.1-dev.5` `df43feb94cdb` |
| 21:24:52–21:36:52 | `A2_dev5_cold` (12 min) | **CLEAN**: 357 campioni da 6 a 718 s, 357 validi, buco massimo 2 s; restore gen 1 12/12 `torque=UNCHANGED_OFF`, census 13/13 |
| 21:25 | decisione dell'operatore: questo è l'ultimo rilievo, poi chiusura | braccio A chiuso con 2 avvii a freddo su 3 previsti |
| 21:37:1x | controlli prima del rollback (`pre_rollback_read_dev5`) | `df43feb94cdb`, READY/IDLE, SAFE_OFF 8191, coppie OFF, DALY OK 11,2 V 62,2 %, allarmi 0000, NVS gen 1 VALID_ACKNOWLEDGED, MODE RUN |
| 21:37:3x | ROM entry, layout, digest dell'installato (`flashlog_B_rollback`) | `ROM_CHECKS = PASS`: tabella `8f756ecb…`, app0 `0x010000`, installato = `c8b27d67…` |
| 21:37:47–21:38:37 | flash APP-only di `30c894a` con lo script qualificato invariato (SHA `812a2e83…`) | `Wrote 1779312 bytes`, `Hash of data verified`, `Verification successful (digest matched)`, `APPLICATION_ONLY_FLASH = PASS` |
| 21:38:47 | uscita dalla ROM con watchdog reset; `B1_30c894a_postrollback_boot` (4 min) | `1.0.2-dev.7` `30c894a0d430`, app0 `0x010000`, NVS READY VALID_ACKNOWLEDGED gen 1, restore `be8d5902bb4e37da` 12/12 `torque=UNCHANGED_OFF`, census 13/13, READY/IDLE, SAFE_OFF 8191, coppie OFF, DALY 118/118 validi (11,2 V, 61,8 %) |
| 21:43 | worktree operativo | HEAD `30c894a0…`, pulito, nessun file scritto |
