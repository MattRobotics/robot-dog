# MATDOG SIMPLE V1 — rc.2, 22 ottobre 2026

Worktree `matdog-simple-motion-v1`, branch `matdog/simple-motion-v1`, base `46123741fcfc4fc393f48752f3fe156fb3e94369`. Catena invariata: Dashboard → SimpleMotion → Gait/IK → ServoBus/SCServo → ST3215. Un solo stato operativo, clock monotono e proprietario UART. Nessun nuovo task, scheduler, controller servo, monitor di tracking o formato NVS. Driver, Geometry V5/URDF, calibrazione e persistenza sono invariati.

## STAND e quota bassa

PRONE è la configurazione ripiegata d'ingresso, con possibile appoggio ventrale; **non è STAND a quota zero**. DISENGAGE e ALIGN rimangono due comandi sincronizzati di posizione, regolati internamente dai servo. L'ingresso osservato può comprendere LOWER oltre il dominio URDF: non viene interpolato né trasformato in un nuovo limite di comando.

La route recuperata conserva PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER e RISE. Le **66 celle con arrivo obbligatorio diventano 7 tratti**: quattro fasi iniziali e RISE diviso a 25%, 50%, 100%. Le prove nuove racchiudono ogni tratto come rettangolo articolare completo, inclusa la riserva CAD di 40 mrad: non presuppongono avanzamento identico dei dodici motori. Ogni tratto supera 120 verifiche continue FCL non adiacenti (≥3 mm, body ≥6 mm), più le 12 verifiche adiacenti, in parte recuperate per contenimento. Il rettangolo dell'intero RISE è rimasto UNRESOLVED; non è stato ammesso per superare il test.

I sette riferimenti usano la quintica `GaitTiming`, con velocità/accelerazione nominali massime 0,30 rad/s e 0,60 rad/s² e v=a=0 ai confini. L'arrivo nativo si valuta soltanto al termine del tratto, con deadline fissa: gli aggiornamenti IMU non possono rimandarla indefinitamente. Nessun confronto temporale contro microtarget. Il waypoint finale duplicato STAND150 è eliminato; goal RAW identici non vengono ritrasmessi.

**Workspace operativo implementato: 50–150 mm.** A 50 mm: IK dei quattro contatti canonici, limiti gen1 rispettati, supporto puntiforme nominale +93,664 mm nel modello di masse URDF (2,48 kg), distanza minima mesh non-piede/piano 2,367 mm. Il contatto piede conserva il modello cilindrico e la discrepanza radiale STL esistente ≤5,78 µm. La prova continua del rettangolo 50–150 mm, con riserva 25 mrad, supera 120+12 coppie; minimo bound non-body 3,025 mm, body 6,034 mm.

44 mm è la prima soluzione IK trovata nello screen a passi di 1 mm, ma LH_UPPER resta a soli 2,486 mrad dal limite gen1: non viene ammessa. **50 mm è la quota minima qualificata qui, non una prova del minimo globale.** Lo screen nominale 44–150 mm non sostituisce una prova continua di contatto/supporto. Le transizioni 150→50→150 usano contatti cartesiani continui e la stessa IK; niente interpolazione articolare che trascini i piedi. Da PRONE viene conservato il passaggio per STAND150: una scorciatoia diretta verso LOW STAND non è qualificata. La prova d'ingresso/route riguarda self-collision e limiti, non carico, attrito o supporto a terra lungo l'intero sollevamento.

## IMU, WALK e confronto numerico

Il driver BNO085 già richiedeva Rotation Vector ogni 20.000 µs; solo quel report pubblica lo snapshot orientamento. Timestamp RX reale, età SH2 e normalizzazione sono conservati. La compensazione viene ora valutata a ogni release durante STAND, HOLD, WALK e assestamento finale, preservando il pitch intenzionale della route. Guadagno 0,5, deadband continua 5 mrad, richiesta ≤20 mrad per asse, slew ≤20 mrad/s, innesto quintico di 1 s dopo torque enable. La stessa IK attenua dolcemente la correzione secondo la sensibilità locale e il margine geometrico; i limiti duri restano invariati. Non si sommano correzioni a una posa già compensata. Questo shaping non garantisce C2 per qualunque segnale IMU rumoroso.

Gait e GaitTiming sono **identici alla base**. Preset 20/30/40 mm, duty 0,8, lift 15 mm, ciclo e rampa iniziale **6 s**; WALK FAST escluso. Swing e raccordi temporali conservano continuità C2; campioni senza attesa d'arrivo. WALK STOP rimane la chiusura coordinata con decelerazione su due cicli (12 s), distinta dallo STOP immediato.

| Riferimento | Prima | rc.2 |
|---|---:|---:|
| Periodo Motion | 40 ms | 20 ms |
| Arrivi obbligatori della route | 66 | 7 |
| Lunghezza nominale route, somma norma infinito | 2,507631 rad | invariata |
| Massimo incremento target STAND nel replay | 39,462 mrad | 5,994 mrad |
| WALK 40 mm, velocità nominale corpo | 5,714 mm/s | 6,667 mm/s (+16,7%) |
| WALK avvio+regime+STOP, picchi analitici q̇/q̈ | 0,872705 / 3,838752 | 1,018157 / 5,224941 rad/s, rad/s² |

Profili **solo RAM**, invariati: STAND 300/8 (nominali 0,460/1,227 SI), WALK 1000/50 (1,534/7,670), manutenzione 160/8. A 5 s l'accelerazione nominale WALK arriverebbe a circa 7,524 rad/s²: 6 s conserva margine per compensazione e qualificazione fisica. 50 mm/6 s esce dal dominio WALK già certificato di 8,160 mrad, quindi il passo massimo resta 40 mm.

Il precedente esperimento 75 mm/6 s valeva 12,5 mm/s, lift 5 mm, picchi circa 0,923/3,466 con un diverso raccordo iniziale sperimentale. Aveva supporto URDF minimo −6,930 mm e non era qualificato a terra. Non viene importato. Anche il WALK attuale ha uno screen puntiforme negativo durante lo STOP (40 mm: −3,949 mm); cambiando solo il periodo la geometria resta la stessa. Non viene dichiarato staticamente stabile a terra.

Replay di protocollo con encoder **esplicitamente pubblicati**, senza modello motore: STAND150 9,28→18,10 s; il primo executor inviava goal finali e il test ne pubblicava subito l'arrivo, mentre rc.2 emette un riferimento temporizzato. Non dimostra una riduzione del tempo hardware. 150↔50 richiede 8,98 s per direzione nel replay; picchi del riferimento 0,291 rad/s e 0,134 rad/s². Nei tratti STAND campionati dopo l'ingresso, incluse correzioni IMU del replay, picchi 0,300/0,703; sull'intera sequenza il picco discreto di accelerazione del goal è 1,450 all'ingresso nativo. **Nessuno di questi valori misura velocità o accelerazione servo.**

## Budget, STOP e verifica

Motion configurato **50 Hz**, IMU richiesta **50 Hz**. Due feedback rapidi per release: tutti i 12 ogni 120 ms; un controllo a due blocchi ogni 120 ms: tutti i controlli ogni 1,44 s, come prima. UART 1 Mbaud: SYNC_WRITE 104 B =1,04 ms; occupazione teorica con risposte regolari ~8,54%, esclusi turnaround/overhead. Budget normali di attesa: 4+4+1,04=9,04 ms; con tutte e tre le conferme termiche: 15,04 ms. Restano 10,96/4,96 ms per calcolo e overhead. Acquisizione/enable hanno operazioni native ulteriori, circoscritte all'ingresso. DALY viene servito solo nello slack già previsto ≥12 ms. Calcolo IK limitato per iterazioni, nessuna attesa HTTP nel generatore. **WCET, jitter e 50 Hz effettivi ESP32 non sono misurabili con questi test host**; usare gli esistenti `max_gap_us`/`missed_releases` nel collaudo.

STOP consuma il latch prima del gate temporale, dopo le letture, dopo enable e prima/dopo il burst UART: SAFE_OFF nella stessa chiamata, senza attendere il waypoint. Le letture sono preemptibili; un burst già in trasmissione deve terminare (104 B nominali 1,04 ms). SAFE_OFF prioritario 13 ID/34 B, poi verifica torque 13/13 e retry. Latenza fisica non misurata. Heartbeat, allarmi, watchdog, limiti e protezioni native conservati.

Test essenziali: sequenza e quote, tre preset, continuità gait+IK alle giunzioni, IMU durante MOVING, timeout nativo, STOP fra release/durante lettura/acquisizione/enable/TX, SAFE_OFF; packet byte-for-byte SCServo, deadline/preemption UART, feed IMU; record reale gen1 `be8d5902bb4e37da`, mapping/round-trip dei 12 e **zero write host**. Dashboard aggiornata e JavaScript verificato. APP ESP32-S3 compilata; risorse definitive nel manifest della build pulita, stack Controller 16 KiB/IMU 8 KiB invariati.

Evidenze: `/home/matteo-manicardi/MATDOG/verification-artifacts/SIMPLE_OPT_20261022/` (certificati, sorgenti delle sonde, CSV, confronto numerico, log). Riproduzione `CollisionData.h`: `python3 scripts/recover_simple_collision_data.py --evidence-root <TOLERANCE_V1_20261018> --optimization-root <SIMPLE_OPT_20261022>`. Hash dei certificati nel sorgente.

Dashboard: STAND, quota 50–150, WALK STANDARD, WALK STOP, STOP; endpoint e autenticazione invariati. USB: `@MOTION STAND 50|150`, `@MOTION WALK STANDARD`, `@MOTION WALK STOP`, `@STOP`, STATUS/HEARTBEAT. Manutenzione conservata. Dopo autorizzazione separata: fixture, identità APP/rollback, SAFE_OFF 13/13 e READY/gen1/IMU/BMS; STAND150 e STOP; 150→50→150; STANDARD20, WALK STOP e STOP. Verificare cadenza, profili e latenza sul dispositivo prima del supporto a terra. Nessuna nuova Full Calibration.

Firmware installato `ad0a091`, APP `7ada67c82992c16326e1cc039c5a70ffb98761d350b51bdab0ebe469981144a2` e rollback preservati. Nessun accesso hardware, flash, EEPROM/NVS write, merge o push durante questa attività.
