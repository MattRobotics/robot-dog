# MATDOG — Passaggio di consegna canonico
## Calibratore meccanico nativo NormaCore → nuova stand-up → gait engine

**Data:** 2026-07-21
**Repository MATDOG:** `~/MATDOG/github/robot-dog` (`MattRobotics/robot-dog`)
**Repository di integrazione:** `~/norma-core` (`MattRobotics/norma-core`)
**Upstream:** `norma-core/norma-core`
**Lingua operativa:** italiano
**Stato hardware richiesto alla ripartenza:** Station arrestata, torque OFF, alimentazione 12 V servo spenta fino al primo test esplicitamente autorizzato.

---

# 1. Mandato della nuova chat

La nuova chat deve:

1. ripulire e consolidare i due repository senza perdere lo stato canonico;
2. implementare il calibratore MATDOG **dentro il driver Rust ST3215 di NormaCore**, seguendo l’architettura già usata per SO101 ed ElRobot;
3. eseguire come primo pilot soltanto `LF_UPPER / M12 / limite MIN`;
4. estendere la procedura ai 24 contatti dei 12 giunti;
5. scrivere i contatti misurati e i limiti safe nel YAML MATDOG;
6. rigenerare da zero la traiettoria `HOME q=0 → LOW_STAND → NOMINAL_STAND`;
7. validarla offline e poi fisicamente con Station;
8. soltanto dopo riprendere body-height control, traiettorie dei piedi, trot e camminata.

La chat non deve ripartire da audit generici o da nuovi wrapper Python. Le informazioni necessarie sono già note e congelate in questo documento.

---

# 2. Decisione architetturale definitiva

## 2.1 Percorso corretto

```text
Station command / AutoCalibrate MATDOG
        ↓
NormaCore driver ST3215 nativo (Rust)
        ↓
matdog.rs — sequenza robot-specifica
        ↓
ST3215Calibrator / primitive RAM-only verificate
        ↓
telemetria InferenceState nativa
        ↓
posizione + velocità + corrente + stato + readback registri
        ↓
contatto ibrido, backoff, ripetizione, report
```

## 2.2 Percorso definitivamente escluso

```text
script Python command-capable
→ client Station Python
→ polling/interpretazione custom
→ movimenti e detector custom
```

Sono esclusi come base del calibratore:

- `matdog_m12_fast_endstop_calibrator.py`;
- `matdog_m12_fast_endstop_v2_runner.py`;
- `matdog_m12_adapter_fast_endstop.py`;
- tutti i runner M12 creati durante i tentativi del 2026-07-21;
- i probe command-capable basati su `CommandBarrier`;
- qualunque accesso diretto `pyserial`;
- qualunque secondo proprietario di `/dev/ttyACM0`.

I tentativi hanno dimostrato che Station e M12 scambiano realmente comandi e ACK, mentre il livello Python personalizzato ha prodotto stato/temporizzazione incoerenti. Non va ulteriormente rattoppato.

---

# 3. Stato canonico MATDOG già completato

## 3.1 Hardware e bus

```text
Asus Ubuntu
→ NormaCore Station
→ Waveshare Bus Servo Adapter
→ scheda distribuzione MATDOG
→ 12 × Feetech ST3215
```

Bus seriale:

```text
serial: 5B14114953
device: /dev/ttyACM0
```

Station è sempre l’unico proprietario della seriale.

## 3.2 Mapping servo

```text
LF: hip M13, upper M12, lower M11
RF: hip M23, upper M22, lower M21
RH: hip M33, upper M32, lower M31
LH: hip M43, upper M42, lower M41
```

Ordine canonico zampe:

```text
[LF, RF, RH, LH]
```

Diagonali trot:

```text
LF + RH
RF + LH
```

## 3.3 Direzioni encoder ↔ joint URDF

```text
LF: M13 hip -1, M12 upper +1, M11 lower -1
RF: M23 hip -1, M22 upper -1, M21 lower +1
RH: M33 hip +1, M32 upper -1, M31 lower +1
LH: M43 hip +1, M42 upper +1, M41 lower -1
```

`GOAL_POSITION` resta unsigned standard `0…4095`.
`signed_tick_delta()` serve soltanto per differenze circolari locali.
Sono vietati signed-wrap e target signed.

## 3.4 Digital zero

La calibrazione digitale EEPROM è completa e non va ripetuta:

```text
q = 0 meccanico visualizzato ≈ 2048 tick su tutti i 12 servo
Position Offset EEPROM: PASS 12/12
LOCK = 1
TORQUE = 0 al readback finale
massima deviazione raw dalla cattura q=0 = 3 tick
```

Offset finali:

```text
M11 +101    M12 +859    M13 -505
M21 -1986   M22 -891    M23 -1687
M31 -2021   M32 -953    M33 -470
M41 -1824   M42 +979    M43 -740
```

Artefatto canonico:

```text
09_Logs/Calibration/C5_R_digital_recenter/
2026-07-10_145457Z_final_12_offset_readback.json
```

SHA256:

```text
15619d23ddcb17651ba729a0d69309b5b56befeb3377f123ff7f131582fcf8ec
```

## 3.5 Tolleranza statica

```text
≤ 10 tick
≤ 0.87890625°
```

Valida per hold statico, micro-probe, ritorno e pose statiche. Non definisce contatto meccanico o accettazione dinamica.

---

# 4. Geometria MATDOG da non semplificare

MATDOG non è simmetrico fronte-retro.

```text
Hip anteriori LF/RF: 20 mm più alte
Hip posteriori RH/LH: quota più bassa
```

Visual zero dei piedi in `base_link`:

```text
anteriori: lowest/contact reference circa Z = -93.4 mm
posteriori: lowest/contact reference circa Z = -113.4 mm
```

Conseguenze:

- esistono quattro profili di zampa espliciti;
- non si copia una singola clearance su tutte le zampe senza trasformarla nella rispettiva catena URDF;
- la generazione della stand deve tenere il `base_link` parallelo al terreno compensando la differenza anteriore/posteriore tramite IK;
- target piede e prerequisite devono essere definiti nei frame corretti della singola zampa.

Geometria base:

```text
interasse hip anteriore-posteriore: 225 mm
larghezza tra assi hip: 95 mm
hip → knee: 90 mm
knee → interfaccia piede: 110 mm
knee → contact frame: 118.1 mm
stand nominale: circa 150 mm
```

Convenzione:

```text
X avanti
Y sinistra
Z alto
metri e radianti
sistema destrorso
```

---

# 5. Limiti URDF e prerequisite geometriche validate

Limiti canonici:

```text
hip:   [-45°, +45°]
upper: [-52.5°, +122.5°]
lower: circa [-92°, +37.5°]
```

## 5.1 Prerequisite HIP

Per tutte le zampe:

```yaml
hip: 0°
upper: +50°
lower: 0°
```

Percorso validato offline:

```text
hip 0° → -45° → 0° → +45° → 0°
```

## 5.2 Prerequisite LOWER

Per tutte le zampe:

```yaml
hip: 0°
upper: +90°
lower: 0°
```

Percorso validato offline:

```text
lower 0° → -92° → 0° → +37.5° → 0°
```

## 5.3 Parking per calibrare le zampe anteriori

Prima di LF:

```text
parcheggiare LH con upper +30°, hip 0°, lower 0°
```

Prima di RF:

```text
parcheggiare RH con upper +30°, hip 0°, lower 0°
```

Dopo la zampa anteriore, la posteriore parcheggiata torna a home.

## 5.4 Ordine completo

```text
LF → RF → RH → LH
```

Per ogni zampa:

```text
UPPER min/max → home
UPPER +50° → HIP min/max → ritorno
UPPER +90° e HIP 0° → LOWER min/max → home
```

Il giunto attivo è uno solo. I prerequisite sono tenuti in posa statica e monitorati.

---

# 6. Stato NormaCore aggiornato

Il 2026-07-21 è stato compilato l’upstream ufficiale aggiornato:

```text
NormaCore version: 0.1.0-beta.9
normfs: 0.1.0-beta.1
upstream main: 5b79422
PR #86 cherry-pick: f8efe97
```

Branch locale creato:

```text
matdog-official-st3215-20260721T173022Z
```

Backup branch precedente:

```text
backup/pre-official-st3215-20260721T173022Z
```

Stash precedente:

```text
pre-official-st3215-20260721T173022Z
```

Binario compilato:

```text
~/norma-core/target/release/station
SHA256: 7f464cf9c9f9cea594d24a3d112a6398fd2aa201d9e37aef9e1ef065c58c49f3
```

Problema upstream scoperto:

- discovery stock limitata agli ID bassi utilizzati da SO101/ElRobot;
- MATDOG usa ID sparsi fino a 43;
- è stata applicata localmente una patch di discovery fino a `43`;
- la patch deve essere ripulita, committata e pubblicata su un branch stabile della fork.

Nome branch consigliato:

```text
matdog/native-calibrator-foundation
```

Il client Python `station_py/client.py` modificato durante i tentativi deve essere ripristinato all’upstream. La modifica di shutdown non è parte dell’architettura finale.

---

# 7. Come integrare MATDOG nell’auto-calibrazione nativa

File principali upstream:

```text
software/drivers/st3215/src/auto_calibrate/mod.rs
software/drivers/st3215/src/auto_calibrate/calibrator.rs
software/drivers/st3215/src/auto_calibrate/so101.rs
software/drivers/st3215/src/auto_calibrate/elrobot.rs
software/drivers/st3215/src/protocol/units.rs
```

Aggiungere:

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
```

## 7.1 Riconoscimento robot

MATDOG viene riconosciuto solo con set esatto:

```text
{11,12,13,21,22,23,31,32,33,41,42,43}
```

Qualunque ID mancante o inatteso deve produrre `FAILED`, senza torque ON.

## 7.2 Non usare le primitive EEPROM degli altri robot

Non chiamare direttamente:

- `prepare_motor()`;
- `send_reset()`;
- `find_min()`;
- `find_max()`;
- `save_calibration()` / freeze con archi;
- funzioni che modificano `Offset`, `Mode`, PID, MaxTorque o ProtectionCurrent in EEPROM.

Per MATDOG il pilot è RAM-only:

```text
nessun reset servo
nessun cambio offset
nessuna EEPROM
nessuna freeze calibration
```

## 7.3 Primitive da aggiungere

Indicativamente:

```rust
prepare_motor_ram_only(...)
set_position_verified(...)
read_motor_observation(...)
find_limit_ram_only_hybrid(...)
backoff_and_verify(...)
repeat_contact_and_compare(...)
return_home_and_verify(...)
```

Le scritture devono usare il percorso nativo del driver:

```text
command_id univoco
→ send_tx
→ wait_for_command_result
→ readback del registro tramite InferenceState
```

---

# 8. Detector di contatto ibrido MATDOG

La corrente è una proxy dello sforzo/coppia; non è una misura diretta in N·m finché non viene caratterizzato il servo.

## 8.1 Dati per ogni campione

- timestamp individuale fresco del servo;
- present position;
- present speed;
- present current;
- goal position;
- torque enable;
- torque limit;
- status/error;
- ultimo command ID/result.

## 8.2 Baseline

Acquisire la baseline della corrente:

- sullo stesso giunto;
- nella stessa posa prerequisite;
- durante movimento libero reale;
- separatamente per direzione;
- con mediana/robust statistics, non un singolo campione.

## 8.3 Conferma contatto

```text
comando continua verso il limite previsto
AND movimento reale già iniziato
AND progresso encoder sotto il minimo atteso
AND velocità sotto soglia
AND corrente sopra baseline di almeno ΔI
AND evidenza persistente su campioni freschi
AND nessun errore servo
AND travel guard non superato
AND prerequisite stabili
= CONTACT_CONFIRMED
```

Stati:

```text
FREE_MOTION
CONTACT_SUSPECTED
CONTACT_CONFIRMED
CONTACT_REPEATABLE
AMBIGUOUS_CONTACT
HARD_ABORT
```

## 8.4 Soglie

Soglie da tenere distinte:

```text
contact_current_delta
hard_current_abort
stall_velocity_threshold
minimum_progress
minimum_travel_before_contact
telemetry_max_age
telemetry_gap_timeout
approach_max_travel
approach_timeout
repeatability_tolerance
backoff_distance
```

Non copiare soglie assolute da SO101 o ElRobot.

## 8.5 Dopo il contatto

```text
stop pressione immediato
→ goal sul present position o altra primitive nativa sicura
→ backoff controllato
→ verifica calo corrente e recupero tracking
→ secondo approccio fine
→ confronto posizione contatto
→ PASS solo se ripetibile
```

---

# 9. Primo pilot hardware: M12 MIN

Pilot unico:

```text
joint: lf_upper_leg_joint
servo: M12
direction: +1
side: URDF MIN
home: 2048
robot sospeso
```

Sequenza:

1. rilevare esattamente i 12 servo;
2. torque OFF globale verificato;
3. leggere M12 e i prerequisite;
4. priming sul present position;
5. impostare RAM di M12: torque limit, speed, acceleration;
6. verificarne il readback;
7. torque ON M12 e readback nativo;
8. se non a home, ritorno a `2048` con progress watchdog;
9. baseline durante moto libero;
10. approach coarse verso MIN;
11. contatto ibrido;
12. stop e backoff;
13. approach fine;
14. ripetibilità;
15. ritorno `2048`;
16. torque OFF globale e readback;
17. report JSON + stato NormaCore.

Criteri PASS minimi:

```text
12 ID esatti
nessuna EEPROM
M12 si muove realmente
TorqueEnable e TorqueLimit readback corretti
telemetria fresca
contatto con posizione+velocità+corrente
backoff riuscito
secondo contatto ripetibile
home finale entro 10 tick
torque globale OFF verificato
status 0x00
```

Qualunque ambiguità produce abort e nessuna prosecuzione.

---

# 10. Risultati falliti da non ripetere

## 10.1 CommandBarrier Python

I comandi raggiungevano Station e spesso il servo, ma il processo si fermava aspettando una barriera/telemetria interpretata dal wrapper. Non aumentare timeout o aggiungere altri polling.

## 10.2 Tuning indiscriminato

Sono stati cambiati deadband, torque, speed, accel, step e timeout prima di isolare il layer errato. Non ripetere modifiche multiple contemporanee.

## 10.3 Interpretazione torque del wrapper

Nel test finale Station ha ricevuto ACK hardware alle scritture RAM, incluso TorqueEnable, mentre il wrapper continuava a stampare `torque=False`. Il readback deve avvenire nativamente dal `motor.state` nel driver Rust.

## 10.4 Nuovi script rapidi

Non creare un altro `fast`, `v2`, `adapter`, `runner`, `probe` o monkey-patch Python. Il prossimo codice command-capable appartiene esclusivamente a NormaCore Rust.

---

# 11. File da conservare nel repository MATDOG

Conservare come fonti canoniche:

```text
README.md
06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md
06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_CALIBRATION_PLAN.md
06_Software/Matdog_Core/calibration/MATDOG_NORMACORE_STATION_CANONICAL.md
06_Software/Matdog_Core/calibration/matdog_joint_math.py
03_CAD/URDF/matt_robodog_rev00/
06_Software/Matdog_Core/kinematics/
09_Logs/Calibration/C5_R_digital_recenter/
```

Conservare C4-A…C4-F e il vecchio C5 come **riferimenti storici/offline**, non come target command-eligible.

---

# 12. Spazzatura da rimuovere

## 12.1 Dal repository/working tree MATDOG

Rimuovere tutti i prototipi command-capable del 2026-07-21 e i backup automatici associati, inclusi se presenti:

```text
matdog_m12_fast_endstop_calibrator.py
matdog_m12_fast_endstop_v2_runner.py
matdog_m12_adapter_fast_endstop.py
matdog_endstop_station_first_contact_probe.py
matdog_endstop_station_command.py
matdog_endstop_station_low_energy_hold.py
matdog_endstop_station_global_torque_off.py
*.pre_*
*.bak
*.orig
__pycache__/
```

Rimuovere i report dei tentativi falliti dalle directory attive:

```text
09_Logs/Calibration_Reports/M12_fast_endstop/
09_Logs/Calibration_Reports/M12_adapter_fast_endstop/
```

Non serve conservare il codice fallito: le lezioni tecniche sono già sintetizzate in questo handoff e la cronologia Git conserva quanto fosse stato committato.

## 12.2 Dal runtime locale

Rimuovere:

```text
~/MATDOG/runtime/station/station_beta9_*.log
~/MATDOG/runtime/station/station_beta9_matdog43_*.log
~/MATDOG/runtime/station/station_data_beta9_*/
~/MATDOG/runtime/station/station_data_beta9_matdog43_*/
```

Conservare/normalizzare un solo file:

```text
~/MATDOG/runtime/station/station.yaml
```

Dopo confronto, eliminare `station_endstop_calibration.yaml` se duplicato.

## 12.3 Da NormaCore locale

Ripristinare all’upstream:

```text
software/station/shared/station_py/client.py
```

Rimuovere:

```text
client.py.pre_nonblocking_stream_close
port.rs.pre_matdog_scan43
qualunque *.pre_* / *.bak generato nei tentativi
```

Conservare e committare soltanto:

- upstream beta.9;
- PR #86;
- patch pulita di discovery MATDOG fino all’ID 43 o, preferibilmente, discovery configurabile/esatta;
- futura implementazione Rust nativa MATDOG.

---

# 13. Aggiornamento documentazione richiesto

## README

Il milestone corrente deve diventare:

```text
Digital zero, URDF, FK/IK offline e geometria prerequisite completati.
Architettura calibratore congelata: implementazione nativa NormaCore Rust.
Prototipi Python command-capable ritirati.
C5 bloccato fino ai 24 contatti, limiti safe e rigenerazione target.
```

## Piano end-stop

Deve specificare:

- implementazione in `auto_calibrate/matdog.rs`;
- primitive RAM-only;
- corrente nel detector;
- asimmetria anteriore/posteriore;
- M12 MIN come pilot;
- niente EEPROM;
- rimozione del percorso Python command-capable.

## NormaCore canonical

Deve registrare:

- beta.9 + normfs beta.1;
- PR #86;
- discovery degli ID MATDOG fino a 43;
- AutoCalibrate nativo come unico command path futuro;
- read-only watcher eventualmente utilizzabile solo come diagnostica, non come motore del calibratore.

---

# 14. Roadmap operativa complessiva

## Fase A — Pulizia e foundation NormaCore

1. pulire `robot-dog` dai prototipi locali;
2. aggiornare/pushare documentazione canonica;
3. rinominare e pubblicare il branch NormaCore foundation;
4. ripristinare il client Python;
5. committare la discovery MATDOG;
6. build e test upstream.

**Criterio chiusura:** due repository puliti, working tree puliti, handoff committato.

## Fase B — Calibratore nativo M12 MIN

1. riconoscimento MATDOG nel dispatcher;
2. profilo M12;
3. primitive RAM-only;
4. readback registri;
5. detector ibrido con corrente;
6. report/stato;
7. test Rust unitari e simulati;
8. build Station;
9. pilot reale M12 MIN.

**Criterio chiusura:** due contatti ripetibili, home e torque OFF finali.

## Fase C — 12 giunti / 24 contatti

1. M12 min/max;
2. M13 min/max con upper +50°;
3. M11 min/max con upper +90°;
4. LF completa con LH parked;
5. RF completa con RH parked;
6. RH completa;
7. LH completa;
8. safe margins;
9. YAML canonico aggiornato.

**Criterio chiusura:** 24 contatti validati o lato intenzionalmente limitato documentato; nessuna EEPROM alterata.

## Fase D — Chiusura post-calibrazione

1. read-only joint-state;
2. FK quattro zampe;
3. verifica limiti e collisioni;
4. confronto contact frame;
5. regressioni encoder↔rad;
6. nuova validazione di tutti i waypoint.

## Fase E — HOME ZERO → LOW_STAND → NOMINAL_STAND

Rigenerare, non riutilizzare, i target hardware.

```text
HOME q=0
→ waypoint contact-locked
→ LOW_STAND
→ NOMINAL_STAND
```

Vincoli:

- `world Z=0`;
- quattro piedi sul piano;
- base_link parallelo al terreno;
- compensazione hip anteriori +20 mm;
- limiti safe misurati;
- collision check su ogni campione;
- support polygon;
- velocità/accelerazione conservative;
- nuovo target encoder post-recenter.

Esecuzione:

1. sospeso;
2. low stand;
3. nominal stand;
4. appoggio graduale supervisionato;
5. hold e ritorno safe.

## Fase F — Body control

- body height;
- roll/pitch statici;
- spostamenti COM controllati;
- verifica contatti e support polygon.

## Fase G — Gait engine

Architettura:

```text
cmd_vel / action
→ phase generator
→ stance/swing foot trajectories
→ per-leg target XYZ
→ IK MATDOG
→ 12 joint rad
→ trajectory/safety validator
→ actuator adapter Station
```

Prima sequenza:

1. lift di una zampa sospesa;
2. traiettoria swing singola;
3. trot in place sospeso;
4. trot in place appoggiato;
5. passo lento;
6. prima camminata.

L’ordine interno MATDOG resta `[LF, RF, RH, LH]`, con fasi diagonali `LF+RH` contro `RF+LH`.

---

# 15. Regole operative per la nuova chat

- Procedere spediti, senza re-audit generici già chiusi.
- Prima di scrivere codice, dichiarare esattamente file e criterio PASS.
- Un solo blocco terminale alla volta durante l’hardware.
- Classificare ogni comando:
  - `sola lettura`;
  - `modifica locale`;
  - `movimento motori RAM-only`;
  - `EEPROM` (non prevista).
- Nessun nuovo terminale quando non necessario.
- Nessun processo Station duplicato.
- Non aumentare torque/timeout per mascherare un errore software.
- Non dichiarare torque OFF senza readback.
- Non dichiarare contatto senza corrente + velocità + progresso + ripetibilità.
- Non riutilizzare target raw pre-recenter.
- Non iniziare gait prima della nuova stand validata.

---

# 16. Primo compito della nuova chat

La nuova chat deve partire con attività **software locale, nessun movimento**:

1. leggere questo handoff;
2. verificare i due `git status`;
3. applicare la pulizia canonica;
4. stabilizzare/pushare il branch `matdog/native-calibrator-foundation` di NormaCore;
5. creare lo skeleton compilabile:

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
```

6. aggiungere riconoscimento exact-set MATDOG in `mod.rs`;
7. aggiungere test che dimostrino:
   - nessuna EEPROM;
   - exact ID set;
   - torque OFF cleanup;
   - profilo M12 MIN;
   - detector con posizione, velocità e corrente;
8. build e test completi;
9. fermarsi prima del movimento e mostrare diff, test e comando hardware previsto.

---

# 17. Messaggio da incollare come primo prompt nella nuova chat

```text
Stiamo riprendendo MATDOG dal checkpoint del 2026-07-21.
Leggi integralmente il file:
09_Logs/Development_Log/2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md

Obiettivo immediato: ripulire e consolidare i repository, poi implementare il pilot nativo Rust LF_UPPER/M12 MIN dentro NormaCore Station, riusando ST3215Calibrator ma con primitive RAM-only e detector ibrido posizione + velocità + PresentCurrent. Non usare più gli script Python command-capable del 2026-07-21.

Vincoli permanenti:
- Station unico proprietario seriale;
- set servo esatto 11,12,13,21,22,23,31,32,33,41,42,43;
- nessun reset servo, nessuna EEPROM, nessun cambio offset;
- GOAL_POSITION unsigned;
- MATDOG ha hip anteriori 20 mm più alte delle posteriori;
- prerequisite geometriche: HIP upper +50°, LOWER upper +90°, rear parking +30° per la zampa anteriore ipsilaterale;
- ordine LF → RF → RH → LH;
- pilot hardware soltanto M12 MIN dopo test Rust e build PASS.

Procedi spedito ma mostra prima il piano dei file da modificare, poi applica il codice e i test. Non proporre nuovi audit generici né nuovi wrapper Python.
```

---

# 18. Stato di sicurezza al termine della sessione precedente

Ultimo test registrato:

```text
status: ABORT
reason: HOME_TORQUE_ENABLE_NOT_VERIFIED nel wrapper Python
Station log: ACK hardware alle scritture RAM incluso TorqueEnable
final_global_torque_off_verified: true
eeprom_writes_sent: false
```

Prima di qualunque ripartenza hardware, verificare comunque fisicamente:

```text
Station non duplicata
torque OFF
12 V servo inizialmente spenti
robot sospeso
master disconnect raggiungibile
```
