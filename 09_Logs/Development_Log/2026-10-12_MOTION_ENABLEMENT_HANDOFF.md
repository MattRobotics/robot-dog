# MATDOG — Handoff Motion — 2026-10-12

Punto di ingresso: [closeout hardware non-motion](../Validation_Reports/MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md).
**BASELINE HARDWARE NON-MOTION VALIDATED; MOTION NOT VALIDATED.**
Questo è un piano operativo per la successiva sessione autorizzata, non un permit.

## Baseline da preservare

- Firmware installato: `b08322dedc5564c080f691dc98047511ef2b953f`;
  build breve operatore `b08322dedc55`; APP-ONLY app0, ROBOT_POWERED,
  layout MATDOG_16M_2x5M_NVS_V1, OTA_INGEST=0. SHA documentale diverso dal firmware.
- NVS gen1 VALID_ACKNOWLEDGED, record integro, A valido/B assente e reconciliation
  non richiesta riferiti post-flash; vedi limiti delle prove nel closeout.
  Non equivale a calibrazione ammessa: precedente stato operativo
  STALE_PENDING_FULL_RECALIBRATION; RESTORE=NOT_IMPLEMENTED.
- MAINTENANCE, authority NONE, MOTION_AUTHORIZED=0. Stato power RUN non è mode RUN.
- Nessuna esecuzione gait/torque operativo/locomozione validata.
  Motion execution suites **DEFERRED**, anche se host tests riportano PASS.

## Invarianti e preflight della futura sessione

1. Ottenere autorizzazione distinta per ogni fase che scrive o attua. Restare
   inizialmente in MAINTENANCE, authority NONE e torque OFF; rete e OTA non
   possono concedere autorità attuatori. Non usare RUN per raggiungere la LAN.
2. Identificare firmware realmente in esecuzione e acquisire evidenza testuale
   redatta dello stato safety, del record NVS e dei fault. Non assumere i dati
   dell'11 ottobre come stato corrente. Se necessario, recuperare i log grezzi
   finali preesistenti; nessun dump NVS/flash o segreto pubblicato.
3. Verificare fixture/supporto stabile, spazio libero, integrità cablaggi e
   meccanica, corrispondenza ID/mapping/segni e disponibilità dell'arresto sicuro.
   Confermare census 13/13 e preflight zampe 12/12 mediante procedura autorizzata.
4. Confermare BMS/allarmi/celle, temperature servo con letture affidabili,
   IMU, alimentazione e condizioni charger/KEY. Charger collegato può alimentare
   bus e servo rail anche con KEY OFF: KEY OFF non prova isolamento elettrico.
5. Riesaminare margini LF UPPER MIN, residui termici e recovery nell'audit dev.3;
   non convertire un errore di telemetria in PASS. Confermare SAFE_OFF effettivo
   secondo il gate approvato prima di qualsiasi fase attuativa.

## Percorso tecnico

- Leggere [semantica calibrazione/limiti](../../01_Docs/02_Architecture/Milestone_I/MATDOG_CALIBRATION_AND_LIMIT_SEMANTICS.md),
  [joint/servo contract](../../01_Docs/02_Architecture/Milestone_I/MATDOG_JOINT_AND_SERVO_CONTRACT.md)
  e [development gates](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md).
- Analizzare offline il confine RAM/NVS: LOAD non ammette trasformazioni;
  restore richiede progetto e qualifica separati, non riuso automatico gen1.
- Definire e autorizzare la calibrazione operativa completa con fixture, margini,
  temperature, abort/recovery e safe-off misurabili; accettare esplicitamente
  il risultato e i limiti per ciascuna delle 12 articolazioni.
- Verificare la catena motion enablement: validità calibrazione runtime,
  envelope/JointLimits ammessi, authority arbiter e permit esplicito,
  inibizioni BMS/OTA/fault, watchdog e revoca/fail-closed. Nessun cablaggio
  alla servo bus della libreria pure motion finché questi gate non sono accettati.
- Solo dopo GO separato, pianificare movimento minimo e limitato su fixture;
  stand/gait e locomozione richiedono ulteriori gate e autorizzazione, non sono
  il primo test. Esecuzioni deferred e oracoli seguono il confine ADR-004.

## GO / NO-GO

| Gate | GO | NO-GO |
|---|---|---|
| Ingresso read-only | Identità/stati safety riconfermati, fixture e arresto disponibili | SHA inatteso, fault, NVS mutata, telemetry incoerente, charger/KEY non compresi |
| Calibrazione dedicata | Procedura e autorizzazione specifiche; margini e safe-off verificabili | Soltanto record gen1 valido, restore implicito o limiti non approvati |
| Primo movimento operativo | Tutti i gate calibrazione/limits/authority/inhibit/revoca approvati e permit esplicito | Un solo gate mancante; torque OFF non verificato; rischi termici/meccanici aperti |
| Stand/gait/locomozione | Validazione incrementale precedente e nuova autorizzazione | Suite DEFERRED trattata come PASS o accesso RUN richiesto soltanto per rete |

**Stato al passaggio: NO-GO per movimento operativo.** GO soltanto per preparazione
tecnica offline nel perimetro già autorizzato. Le verifiche hardware qui elencate
sono da eseguire nella futura sessione autorizzata, non eseguite nel closeout.

Decisioni rimandate: restore/persistenza operativa e veri power-cycle; envelope
ed enablement; LED soglie SOC via simulazione e breathing; LAN read-only separata
AP/OTA; charge termination/KEY/MOS; security sessioni/accessi. Registro unico:
[OPEN_ITEMS](../OPEN_ITEMS.md#closeout-2026-10-11--fase-successiva).
