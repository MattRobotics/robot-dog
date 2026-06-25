# MATDOG — Stato del Progetto e Prossimi Passaggi

**Data stato:** 25 giugno 2026  
**Repository principale:** `MattRobotics/robot-dog`  
**Branch di sviluppo:** `matdog/foundation`  
**Integrazione Station:** `MattRobotics/norma-core`  
**Upstream di riferimento:** `norma-core/norma-core`

---

## 1. Obiettivo della fase attuale

MATDOG è un cane robot quadrupede custom, ispirato all'architettura Yahboom DOGZILLA ma progettato con meccanica, geometrie, elettronica e software propri.

La priorità attuale è arrivare nell'ordine a:

```text
URDF corretto
→ telemetria servo associata ai joint
→ calibrazione cinematica
→ stand pose stabile
→ IK singola zampa
→ IK quattro zampe
→ traiettorie dei piedi
→ trot in place
→ camminata reale
```

Batteria definitiva, Jetson, testa, AI e visione restano fuori dalla fase corrente. Il primo obiettivo pratico è far camminare MATDOG collegato all'Asus.

---

## 2. Stato hardware validato

Catena attuale:

```text
Asus Ubuntu
→ NormaCore Station
→ Waveshare Bus Servo Adapter
→ scheda di distribuzione custom
→ 12 × Feetech ST3215
```

Risultati confermati:

- tutti i 12 servo sono rilevati da Station;
- i motori rispondono immediatamente ai comandi;
- temperature e comunicazione bus sono normali;
- cablaggio e scheda di distribuzione sono validati per la fase attuale;
- il lag mostrato dal viewer è principalmente l'età del dato telemetrico, non un ritardo reale dei motori;
- non esiste oggi una criticità che giustifichi l'introduzione immediata di un ESP32.

Decisione architetturale:

```text
prima far camminare MATDOG con Station + Waveshare
→ solo dopo valutare ESP32 / coprocessore real-time
```

Il software deve restare modulare:

```text
Gait Generator
→ Inverse Kinematics
→ Servo Target Writer
```

---

## 3. Geometria e convenzioni congelate

Quote attuali:

```text
Interasse hip anteriore-posteriore: 225 mm
Larghezza tra assi hip destro-sinistro: 95 mm
Hip → knee: 90 mm
Knee → foot: 110 mm
Altezza target body in stand: 150 mm
```

Convenzione ROS/URDF:

```text
X = avanti
Y = sinistra
Z = alto
unità = metri
sistema destrorso
```

Il frame `base_link` deve stare al centro della faccia inferiore del body, sotto il centro del rettangolo formato dai quattro assi hip.

Ogni origine joint deve coincidere con il vero asse del rispettivo servo. Il frame di ogni piede deve stare nel punto nominale di contatto del gommino con il terreno, non nel centro estetico del link o del gommino.

---

## 4. Ordine gambe e mapping servo definitivo

Ordine canonico:

```text
[LF, RF, RH, LH]
```

Significato:

```text
LF = anteriore sinistra
RF = anteriore destra
RH = posteriore destra
LH = posteriore sinistra
```

Diagonali trot:

```text
coppia A = LF + RH
coppia B = RF + LH
```

Mappatura servo → joint:

```text
LF: hip M13, upper M12, lower M11
RF: hip M23, upper M22, lower M21
RH: hip M33, upper M32, lower M31
LH: hip M43, upper M42, lower M41
```

Questa mappa deve rimanere identica in URDF, configurazione cinematica, dashboard, viewer 3D, calibrazione, IK, gait, test hardware e future integrazioni ESP32.

---

## 5. Posa meccanica zero

Posa meccanica di riferimento:

```text
hip centrato lateralmente
upper leg verticale verso il basso
lower leg orizzontale
upper e lower leg a 90°
```

Nel modello cinematico:

```text
q_hip = 0 rad
q_upper = 0 rad
q_lower = 0 rad
```

Questa posa servirà per registrare gli encoder zero reali.

Valori non ancora noti e da non inventare:

- zero encoder per ogni joint;
- verso di rotazione per ogni joint;
- limiti software min/max;
- offset reali dei piedi;
- quota reale degli assi hip rispetto a `base_link`.

Questi dati verranno ricavati dall'URDF definitivo, dal CAD e da una calibrazione controllata.

---

## 6. Viewer 3D e telemetria Station

La telemetria raw ST3215 funziona. I valori `present_position` vengono aggiornati live.

Il renderer 3D provvisorio basato sulla `BusCard` generica non è affidabile: l'overlay MATDOG LIVE può aggiornarsi, ma il modello 3D non segue ancora correttamente i movimenti fisici.

Conclusione:

```text
telemetria e mapping del bus funzionano
→ il problema è nel renderer/proxy temporaneo
→ non nella comunicazione servo
```

Decisioni:

- non investire altro tempo nel renderer temporaneo della `BusCard`;
- non usare la tabella ST3215 come dashboard operativa del cane;
- costruire una dashboard MATDOG dedicata solo dopo URDF e calibrazione;
- usare la `BusCard` soltanto per diagnostica raw, test singoli motori e configurazione bus;
- non usare il pulsante generico `Calibrate` della pagina ST3215 per la calibrazione cinematica MATDOG.

---

## 7. Architettura software definitiva

La dashboard MATDOG non deve comandare direttamente i servo.

```text
MATDOG Dashboard
→ comandi semantici ad alto livello
→ MatdogControlDriver
→ calibrazione + limiti + sicurezza + IK + gait
→ MatdogSt3215Adapter
→ driver ST3215 ufficiale NormaCore
→ Waveshare Bus Servo Adapter
→ 12 ST3215
```

La dashboard invierà comandi del tipo:

```text
SET_CONTROL_MODE
SET_JOINT_TARGETS
SET_LEG_TARGET
SET_BODY_VELOCITY
REQUEST_ACTION
E_STOP
```

Non invierà normalmente target grezzi del tipo:

```text
servo M13 = 3210 tick
```

Conversione interna prevista:

```text
target joint in radianti
→ zero encoder + direzione + limiti
→ target encoder ST3215
→ sync write
```

Il driver ST3215 ufficiale deve restare l'unico proprietario della seriale.

---

## 8. Strategia di migrazione futura

Architettura attuale:

```text
Dashboard
→ MATDOG Control Driver su Asus
→ driver ST3215 Station
→ Waveshare
→ servo
```

Architettura futura possibile:

```text
Dashboard / Jetson
→ MATDOG API
→ ESP32 motion controller
→ gait + IK + IMU + watchdog
→ driver servo
→ ST3215
```

Dashboard, API semantica, gait generator e IK non dovranno essere riscritti. Dovrà cambiare solo il backend attuatore:

```text
oggi: LocalSt3215Actuator
domani: Esp32MotionActuator
```

---

## 9. Ruolo dei repository

### `MattRobotics/robot-dog`

È la sorgente di verità MATDOG. Contiene:

```text
geometria
URDF
mesh
servo mapping
calibrazione
IK
gait
test
documentazione tecnica
decisioni architetturali
```

### `MattRobotics/norma-core`

È la fork sottile di integrazione con Station. Conterrà soltanto:

```text
registrazione driver MATDOG
protobuf/comandi MATDOG
mount dashboard MATDOG
adapter verso ST3215
compatibilità con aggiornamenti upstream
```

### `norma-core/norma-core`

È l'upstream ufficiale. Va seguito e aggiornato in modo controllato.

### `MattRobotics/xgolite-low-level-reconstruction`

Rimane separato per reverse engineering XGO Lite, prove firmware, mapping, test vettoriali e analisi IK/gait Yahboom. Non è il repository operativo MATDOG.

---

## 10. Artefatti MATDOG già presenti

Nella branch `matdog/foundation` sono già presenti:

```text
01_Docs/02_Architecture/ARCHITECTURE.md
01_Docs/02_Architecture/CURRENT_STATE.md
01_Docs/02_Architecture/UPSTREAM_CONTRACT.md
04_Electronics/Servo_Mapping/MATDOG_SERVO_MAPPING.yaml
06_Software/Matdog_Core/config/MATDOG_GEOMETRY.yaml
09_Logs/Architecture_Decisions/ADR-002_MATDOG_Repository_and_Station_Integration.md
```

Questi file fissano geometria nota, ordine gambe, mapping servo, responsabilità dei repository, confine MATDOG/Station, strategia upstream e campi non ancora calibrati.

---

## 11. Prossimo input necessario

Il prossimo input fondamentale è l'URDF reale e definitivo MATDOG, derivato dal CAD finale.

Quando disponibile:

1. verificare struttura di link e joint;
2. verificare nomi canonici;
3. verificare origini dei joint sugli assi servo;
4. verificare assi di rotazione;
5. verificare i frame foot sul contatto gommino-terreno;
6. verificare `base_link`;
7. caricare il modello in un viewer;
8. validare la posa meccanica zero;
9. inserire il file nella struttura repository;
10. versionare il primo URDF ufficiale MATDOG.

---

## 12. Roadmap operativa

### Fase A — URDF e cinematica statica

1. Importare URDF definitivo.
2. Validare albero cinematico.
3. Validare mesh e frame.
4. Validare posa meccanica zero.
5. Confermare segni degli assi joint.

### Fase B — Calibrazione cinematica

1. Posizionare fisicamente una zampa nella posa zero.
2. Leggere `present_position` dei servo.
3. Registrare gli encoder zero.
4. Muovere un joint alla volta a bassa energia.
5. Determinare segno reale dei joint.
6. Misurare i limiti meccanici sicuri.
7. Salvare i risultati nella configurazione MATDOG.
8. Ripetere per tutte le gambe.

### Fase C — Viewer 3D sincronizzato

1. Convertire encoder live in radianti con calibrazione reale.
2. Applicare i radianti ai joint URDF.
3. Verificare una gamba per volta.
4. Verificare tutti i dodici joint.
5. Separare dashboard MATDOG e diagnostica ST3215.

### Fase D — Controllo sicuro dei joint

1. Implementare `DISARMED`.
2. Implementare `CALIBRATION`.
3. Implementare `JOINT_TELEOP`.
4. Consentire micro-movimenti sicuri di un joint.
5. Bloccare target fuori limite.
6. Integrare stop di emergenza e watchdog.

### Fase E — Stand pose

1. Definire una stand pose con target joint fissi.
2. Testarla senza carico.
3. Testarla con robot sostenuto.
4. Testarla a terra.
5. Registrare i target validati.

### Fase F — IK e gait

1. Implementare FK di una gamba.
2. Implementare IK di una gamba.
3. Validare una gamba sospesa.
4. Estendere a quattro gambe.
5. Implementare controllo altezza e assetto body.
6. Implementare traiettorie piede.
7. Implementare mark-time.
8. Implementare trot in place.
9. Implementare camminata lenta in avanti.
10. Implementare turning.

### Fase G — Dashboard MATDOG completa

```text
viewer 3D grande
Move X / Y / Yaw
Actions: Stand / Sit / Reset
quattro pannelli gamba
Diagnostics
telemetria temperatura / corrente / stato / batteria futura
```

Move, Actions e Gait resteranno disabilitati finché non saranno validati meccanica, calibrazione, limiti e stand pose.

---

## 13. Regole di sicurezza permanenti

```text
Non usare il generic Calibrate di Station per MATDOG.
Non inviare target encoder raw dalla dashboard normale.
Non attivare gait prima di calibrazione completa, limiti documentati,
stand pose validata, test singola zampa e test quattro zampe senza carico.
Non aprire una seconda connessione seriale ai servo.
Il driver ST3215 ufficiale resta il solo proprietario del bus seriale.
```

---

## 14. Ripresa della prossima sessione

```bash
cd ~/robot-dog
git pull --ff-only
git status --short
git log --oneline main..HEAD
```

Il primo lavoro operativo della prossima sessione è analizzare e inserire l'URDF definitivo MATDOG.
