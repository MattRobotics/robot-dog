# MATDOG SIMPLE V1 — rc.3, 22 ottobre 2026

Worktree `matdog-simple-motion-v1`, branch `matdog/simple-motion-v1`, base rc.2 `8d0b47da979e809b9f5460a6827534e81e4532bc`. Catena invariata: Dashboard → SimpleMotion → Gait/IK → ServoBus/SCServo → ST3215. Un solo stato operativo, clock monotono e proprietario UART. rc.3 cambia soltanto i generatori STAND/WALK e la loro integrazione; ServoBus, SCServo, calibrazione, NVS gen1, STOP/SAFE_OFF, IMU driver e profili RAM sono invariati.

## STAND: un riferimento continuo per gruppo

PRONE è la configurazione ripiegata d'ingresso con possibile appoggio ventrale; non è STAND a quota zero. DISENGAGE e ALIGN restano mosse native di posizione (l'inizio osservato può essere fuori URDF).

Poi la route congelata (`PairedProneData.h`) viene **seguita sui suoi nodi**, non sulle corde articolari rc.2 fra nodi di fase: le corde deviavano dai nodi fino a 69 mrad (REAR_DEPLOY) e 283 mrad (RISE); nel modello di appoggio statico mesh+URDF la corda RISE stirava di 15 mm la distanza piedi anteriori–posteriori. Nodi decimati con tolleranza di corda 1 mrad (`stand_vias.json`):

- **STAND_DEPLOY** nodi 0→107 (PRE_ALIGN, REAR_DEPLOY, FRONT_DEPLOY, LOAD_TRANSFER), 11 via-point, un solo riferimento senza arrivi intermedi;
- arrivo verificato a nodo 107: quattro piedi sotto il corpo prima che il peso lasci un eventuale appoggio ventrale (unica fermata interna, fisicamente motivata);
- **STAND_LIFT** nodi 107→160 (disimpegno, rise a contatti bloccati, livellamento) fino alla quota richiesta ≤50 mm: a nodo 160 la route è già STAND50 (z=50 mm, pitch 0, contatti canonici; nodi 160–251 = IK canonica entro 6·10⁻⁵ rad). Quindi **PRONE→STAND50 è diretto**: nessun passaggio per 150 mm;
- quote >50 mm: arrivo a 50 mm, poi **HEIGHT** sui nodi livellati 160–251 e IK esatta della quota richiesta, nel box altezze rc.2 (50–150 mm ±25 mrad). Lo stesso generatore serve i cambi di quota da HOLD.

`ViaPlan.h`: segmenti articolari rettilinei fra via-point raccordati da blend quintici di velocità centrati sui via-point. Per costruzione: |q̇|≤0,35 rad/s, |q̈|≤0,70 rad/s² per giunto, posizione C³, ogni giunto entro [min,max] dei propri via-point (i blend tagliano gli angoli verso l'interno e non si sovrappongono). Sotto il profilo RAM STAND 300/8 (0,460/1,227 SI) con margine per la correzione IMU. Pianificazione all'inizio del gruppo: ≤24 via, al massimo 64 passate a passo √ (6–7 sulle route reali), nessuna allocazione; nel loop Motion solo campionamento O(n).

Sicurezza continua senza monitor di tracking: ogni gruppo è contenuto in un box articolare certificato per **qualsiasi** combinazione di avanzamento dei dodici motori (riserva 40 mrad, 120 coppie FCL non adiacenti ≥3 mm, base ≥6 mm, + 12 adiacenti): DEPLOY 0–107 PASS, LIFT 107–160 PASS. Il box unico 107–251 è **NOT_PASS**: 4 coppie anca–tibia/piede anteriori convergono a 3,000 mm esatti nella configurazione mista "alto a metà salita, basso ancora ripiegato"; per questo la sosta a 50 mm resta obbligatoria sopra 50 mm. Ripresa da STOP: dal pose misurato verso il nodo più vicino del gruppo che lo contiene.

Contatti (modello di appoggio statico sagittale, mesh V5 + masse URDF) sui riferimenti esatti del firmware: solo i quattro piedi a contatto in DEPLOY, LIFT e HEIGHT; clearance non-piede minima 2,04 mm (tibia LF, geometria tibia/piede, presente a ogni quota); stiramento piedi in LIFT ≤3,2 mm (rotolamento, come i nodi). Il modello mesh pone il robot ripiegato sui piedi, non sul ventre, con scarto fino a 5,9 mm / 27 mrad rispetto alla posa nominale dei nodi PRONE: proprietà ereditata dalla route.

**Quota minima 50 mm, confermata.** A 50 mm: margine gen1 nominale 105 mrad, 29,6 mrad con correzione IMU massima (roll+pitch 20 mrad); clearance non-piede 2,37 mm, peggiore 2,25 mm con IMU ±20 mrad ed errore servo ±18,4 mrad. A 48 mm il margine con correzione diventa −5,6 mrad, a 46/44 mm l'IK con correzione non è risolvibile: sotto 50 mm non qualificato.

## WALK STANDARD

Stesso gait engine (`Gait.cpp`), stessa IK, clock costante 6,5 s/ciclo. Preset 30/45/60 mm/ciclo (4,62/6,92/9,23 mm/s), duty 0,825, lift 8 mm. Modifiche alla legge:

- **sway longitudinale** 10 mm (corpo avanti durante lo swing posteriore, indietro durante l'anteriore) e **laterale** 3,5 mm (verso le zampe in appoggio), che cambiano solo nelle finestre a quattro appoggi: nei tre appoggi l'offset è costante. Lo sway laterale è limitato dalla striscia nominale del piede (inclinazione ≤2°; anche ±21 mrad);
- **rampe geometriche C² del passo** (1 ciclo in partenza, 1 ciclo in arresto) al posto delle rampe del clock: nessun primo passo lungo 1,4λ, corpo fermo con velocità e accelerazione nulle agli estremi;
- **WALK STOP** dal prossimo touchdown (quarto di ciclo), decelerazione geometrica su un ciclo, ultimo touchdown in posizione canonica e finestra finale a quattro appoggi che ricentra gli sway. Piedi già appoggiati o in volo non cambiano bersaglio. STOP di emergenza invariato e separato.

Supporto (punti di contatto, COM URDF 2,48 kg, stesso criterio rc.2): rc.2 −3,947 mm a ciclo terminale +0,55. rc.3: ≥+5,954 mm su tutta la sequenza per 3 preset × 13 allineamenti di STOP (partenza, regime, arresto, ricentraggio); ZMP-proxy carrello-tavolo ≥+5,978 mm. Prova statica, non stabilità dinamica dimostrata. Picchi analitici q̇/q̈ ≤0,897/4,879 (rc.2 1,018/5,225), profilo WALK RAM 1000/50 invariato. Box WALK V2 (inviluppo di tutti i preset ±40 mrad) certificato 120+12 PASS; margine minimo ai limiti gen1 del riferimento 42,5 mrad (RF_LOWER, preset 60 mm). Il firmware coincide con lo strumento offline entro 10⁻¹² rad su tutte le release.

## IMU

Invariata: Rotation Vector 50 Hz, guadagno 0,5, deadband 5 mrad, richiesta ≤20 mrad, slew 20 mrad/s, innesto quintico 1 s, attenuazione geometrica, un solo generatore. Correzione rc.3: un solo passo di slew per release (alle giunzioni di gruppo il riferimento è rigenerato nella stessa release e lo slew avanzava due volte). La legge a slew limitato non è C² rispetto a segnali IMU arbitrari.

## Confronto numerico (replay di protocollo, encoder pubblicati, nessun modello motore)

| Riferimento | rc.2 | rc.3 |
|---|---:|---:|
| PRONE→STAND150 | 18,10 s | 12,80 s |
| PRONE→STAND50 | 27,08 s (via 150) | 7,70 s diretto |
| 150↔50 | 8,98 s | 5,12 s |
| Fermate con arrivo nella route | 7 (+2 native) | 1 a 50 mm, 2 sopra (+2 native) |
| WALK massimo | 40 mm / 6 s, 6,67 mm/s | 60 mm / 6,5 s, 9,23 mm/s |
| WALK STOP richiesta→STANDING | fino a ~18 s | 8,74 s (≤1,63 s attesa + 6,99 s) |
| Supporto statico minimo WALK | −3,947 mm | +5,954 mm |

**Nessuno di questi valori misura tempi, velocità o accelerazioni dei servo reali.**

## Budget, STOP e verifica

Motion configurato **50 Hz**, IMU richiesta **50 Hz**. Due feedback rapidi per release: tutti i 12 ogni 120 ms; un controllo a due blocchi ogni 120 ms: tutti i controlli ogni 1,44 s, come prima. UART 1 Mbaud: SYNC_WRITE 104 B =1,04 ms; occupazione teorica con risposte regolari ~8,54%, esclusi turnaround/overhead. Budget normali di attesa: 4+4+1,04=9,04 ms; con tutte e tre le conferme termiche: 15,04 ms. Restano 10,96/4,96 ms per calcolo e overhead. Acquisizione/enable hanno operazioni native ulteriori, circoscritte all'ingresso. DALY viene servito solo nello slack già previsto ≥12 ms. Calcolo IK limitato per iterazioni, nessuna attesa HTTP nel generatore. **WCET, jitter e 50 Hz effettivi ESP32 non sono misurabili con questi test host**; usare gli esistenti `max_gap_us`/`missed_releases` nel collaudo.

STOP consuma il latch prima del gate temporale, dopo le letture, dopo enable e prima/dopo il burst UART: SAFE_OFF nella stessa chiamata, senza attendere il waypoint. Le letture sono preemptibili; un burst già in trasmissione deve terminare (104 B nominali 1,04 ms). SAFE_OFF prioritario 13 ID/34 B, poi verifica torque 13/13 e retry. Latenza fisica non misurata. Heartbeat, allarmi, watchdog, limiti e protezioni native conservati.

Test essenziali: sequenza e quote, tre preset, continuità gait+IK alle giunzioni, IMU durante MOVING, timeout nativo, STOP fra release/durante lettura/acquisizione/enable/TX, SAFE_OFF; packet byte-for-byte SCServo, deadline/preemption UART, feed IMU; record reale gen1 `be8d5902bb4e37da`, mapping/round-trip dei 12 e **zero write host**. Dashboard aggiornata e JavaScript verificato. APP ESP32-S3 compilata; risorse definitive nel manifest della build pulita, stack Controller 16 KiB/IMU 8 KiB invariati.

Evidenze rc.3: `/home/matteo-manicardi/MATDOG/verification-artifacts/SIMPLE_TRAJ_V2_20261022/` (certificati, sonde, CSV, log). Evidenze rc.2: `/home/matteo-manicardi/MATDOG/verification-artifacts/SIMPLE_OPT_20261022/` (certificati, sorgenti delle sonde, CSV, confronto numerico, log). Riproduzione `CollisionData.h`: `python3 scripts/recover_simple_collision_data.py --evidence-root <TOLERANCE_V1_20261018> --optimization-root <SIMPLE_OPT_20261022> --trajectory-root <SIMPLE_TRAJ_V2_20261022>`. Hash dei certificati nel sorgente.

Dashboard: STAND, quota 50–150, WALK STANDARD, WALK STOP, STOP; endpoint e autenticazione invariati. USB: `@MOTION STAND 50|150`, `@MOTION WALK STANDARD` (45 mm), `@MOTION WALK STOP`, `@STOP`, STATUS/HEARTBEAT. Manutenzione conservata. Dopo autorizzazione separata: fixture, identità APP/rollback, SAFE_OFF 13/13 e READY/gen1/IMU/BMS; PRONE→STAND50 e STOP; PRONE→STAND150; 150→50→150; WALK 30 mm, WALK STOP, STOP; poi 45 e 60 mm. Verificare cadenza, profili e latenza sul dispositivo prima del supporto a terra. Nessuna nuova Full Calibration.

Firmware installato `ad0a091`, APP `7ada67c82992c16326e1cc039c5a70ffb98761d350b51bdab0ebe469981144a2` e rollback preservati. Nessun accesso hardware, flash, EEPROM/NVS write, merge o push durante questa attività.
