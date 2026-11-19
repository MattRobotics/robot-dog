# Criteri numerici di Motion contro la risoluzione fisica (18/11/2026, offline, sorgente `30c894a`)

Mandato «Correzione priorità Motion: realismo fisico e tolleranze».
- **Sviluppo W2+B:** sospeso, mai iniziato.
- **Non fatto:** nessuna modifica al worktree o al firmware installato, nessun flash, commit o push.
- **Dove ho lavorato:** banchi su copie (`trees/`, `full/`).

## Scale fisiche usate (dal sorgente e dai dati CAD/URDF)

| Grandezza | Valore | Fonte |
|---|---|---|
| Tick ST3215 | 2π/4096 = 1,534 mrad (0,088°); mezzo tick 0,767 mrad | `kTickRad`, `SimpleMotion.h` |
| Piede | cilindro r 14,9 mm, battistrada 13,9 mm, raccordo 2 mm, striscia 9,9 mm, inclinazione nominale 2° | `FootContactData.h` (dallo YAML della geometria del piede) |
| Contatto sul raccordo | ammesso fino a 12°, dove la gamba inferiore resta ≥ 2 mm dal suolo | `FootContact.h`, `foot_edge_contact.py` |
| Gamba | coscia 90 mm, gamba ~118 mm, offset laterale anca 48 mm | `LegGeometryData.h` (URDF rev00) |
| Sensibilità verticale delle articolazioni (∣J⁻¹e_z∣, Jacobiano URDF sul contatto reale) | 10–15,3 rad/m nel WALK sterzato SLOW/STANDARD/FAST | banco, 11 324 soluzioni |
| Spostamento verticale del piede per tick | ≥ 100 µm; mezzo tick ↔ **50,2 µm** (caso peggiore, SLOW) | da sopra |

## Tabella dei criteri

| # | Criterio | Valore | Precisione producibile / margine necessario | Evidenza (banco host, 7 scenari × 3 fasi del rumore IMU; QEMU) | Esito |
|---|---|---|---|---|---|
| 1 | `Gait.cpp`: nuova IK di contatto per gamba se l'alzata del raccordo dal seme differisce da quella dell'HIP risolto | **10 µm** | Mezzo tick ↔ 50 µm. Protezioni che restano sulla soluzione consegnata: inclinazione dell'asse ≤ limite del raccordo, limiti articolari, ramo, box e `pointSafe` | Ricalcola nel 15–19 % delle gambe sterzate. Un'altra soluzione cambierebbe le articolazioni di ≤ 0,095 tick (fra 10 e 50 µm: ≤ 0,49 tick) | **Lavoro inutile fra 10 e 50 µm → patch P1** |
| 1b | W1: seme dell'HIP previsto (estrapolazione lineare) | – | – | Le soluzioni con tutte e 4 le gambe da ricalcolare passano da 28 a 23 (B3) a 10 µm. A 50 µm peggiora (da 8 a 20): nel trasferimento laterale gli HIP accelerano di 12–43 mrad in 40 ms | **Non adottata:** non evita la seconda IK nel caso peggiore |
| 2 | `TiltedContactIk` (compensazione): tolleranza di Newton | **1 µm**, 8 iterazioni | Fermarsi a 20 µm cambia ≤ 0,28 tick | Passi ≤ 2, mai `NO_CONVERGENCE`. A 20 µm si risparmia un passo nel 27–37 % delle IK. **Ma la sonda della compensazione divide per la scala** (fino a 1/32): l'errore entrerebbe ×32 nella stima di sensibilità che decide l'attenuazione | **Non toccato:** criterio accoppiato all'attenuazione |
| 2b | `TiltedContactIk`: ricalcolo dell'alzata del raccordo nella compensazione | **20 µm** | Stesso criterio fisico di 1 | Col seme E1 (16/11) lo scarto della prima soluzione è ≤ 10,2 µm, quindi 0 ricalcoli. Portarlo a 50 µm non cambia nulla (P1 solo `Gait.cpp` e P1 completo danno frame identici bit per bit in 21/21 esecuzioni) | **Allineato alla costante unica di P1 (SSOT), senza effetto misurabile** |
| 3 | `compensate()`: nuova soluzione se l'attenuazione cambia oltre il 2 % della scala | **2 %** | Singolarmente l'89–100 % di questi ricalcoli sposta < ½ tick | Ho provato tre varianti (½ tick; ¼ tick; ½ tick con una release di ritardo). Riducono le doppie compensazioni del 49–99 %, ma aumentano le **inversioni di 1 tick del comando**. In HEIGHT +12…+19 % in tutte e tre le varianti (somma sulle 3 fasi del rumore; in una singola fase fino a +28 %). In STAND_LIFT +53 % con la variante ½ tick. Quei ricalcoli tengono il comando regolare | **Non è lavoro inutile: resta il 2 %** |
| 4 | Restrizioni del modello di contatto (striscia nominale di 2°, `CONTACT_MODE`, `WORKSPACE`) | – | Margine reale: gamba ≥ 2 mm a 12° | **HW 18/11** (5 050 campioni, due catene): 0 `CONTACT_INVALID`, `NO_CONVERGENCE`, `WALK_IK_OR_WORKSPACE`, 0 fault. `WORKSPACE` solo in HOLD e WALK_SETTLE, rifiutato da **`LOAD_BOX`** (protezione del carico: resta). H1 dell'8/11 è già corretto (dev.4). Banco: nessun rifiuto falso | **Nessuna patch** |

## Patch P1 (`P1_PATCH.diff`, copia `full/cand`, non integrata)

**Cosa cambia:**
- `FootContact.h`: `kEdgeLiftResolveM = 5e-5`, con la derivazione fisica nel commento.
- `Gait.cpp` (10 µm) e `TiltedContactIk.cpp` (20 µm) usano la costante; commenti di `TiltedContactIk.h` aggiornati.
- Pin dei 4 file aggiornati in `motion_final_release_sources.json`.

**Invariati:** IK, geometria, gait, contatti, limiti, box, STOP e protezioni.

**Effetto misurato:**
- **Lavoro:** ricalcoli dell'alzata −45 %…−75 %.
- **QEMU** (deterministico; il tempo QEMU non è il WCET HW):
  - release sterzate sopra 320 k CCOUNT: B3 da 25 a 5, D8 da 44 a 17;
  - NOMINAL medio −5 %;
  - **massimo invariato:** il trasferimento laterale richiede ancora 8 IK, che spostano ≥ ½ tick.
- **Frame:**
  - STAND, HEIGHT e PRONE, e il WALK non sterzato: identici bit per bit;
  - WALK sterzato: entro 0–2 tick allineati per fase. Fino a 8 tick solo dove una correzione IMU tenuta diversa di 1,3 mrad (banda morta di 1 mrad) sposta la camminata successiva di 40 ms rispetto ai comandi del joystick.
- **Regolarità del comando:** inversioni di 1 tick invariate (WALK 3 492 contro 3 489, HEIGHT 916 contro 915, sulle 3 fasi del rumore).
- **Contatto** (inviluppo del joystick con il raccordo reale):
  - scenari deterministici: supporto identico;
  - penetrazione del modello +0,01…+0,03 mm (massimo da 0,214 a 0,232 mm), distanza dal suolo −0…0,05 mm (minimo da 2,27 a 2,24 mm);
  - margini di box, gen1 e inclinazione identici in 36 scenari su 45; gli altri sono sequenze casuali che divergono in entrambi i sensi.
- **Inviluppo del joystick:** **non passa già sulla baseline** (`random_3`: margine d'inclinazione −3,0 mrad, 6 fallimenti). Col candidato −1,4 mrad, 4 fallimenti. Difetto preesistente, da trattare a parte.
- **Suite:** `run_simple_motion_tests.sh` PASS, `run_stop_secant_gate.sh` PASS (stesse secanti), matrice di perturbazioni identica (0 fault); `static_audit` sul repository completo del candidato **PASS** (gate di convergenza delle sorgenti PASS con i pin aggiornati; le 6 righe FAIL di sotto-test attesi sono identiche all'audit CLEAN di `30c894a`); compilazione completa: IRAM, DATA e BSS identici a `30c894a`, `.flash.text` −8 B. La copia non è un repository git, quindi il gate finale rifiuta l'identità (`IDENTITY_MISMATCH`), come previsto: non è un'immagine installabile.

## Conclusione per P0

- **I criteri che producevano lavoro inutile riguardavano un solo punto:** il ricalcolo dell'alzata in `Gait.cpp`. P1 lo corregge.
- **Gli altri criteri non vanno toccati:**
  - la regola del 2 % fa lavoro che si vede nella regolarità del comando;
  - la tolleranza di Newton è accoppiata alla sonda dell'attenuazione;
  - i rifiuti osservati sono protezioni.
- **Le release più pesanti restanti fanno lavoro fisicamente significativo:**
  - doppia compensazione oltre mezzo tick;
  - 8 IK nel trasferimento laterale.
- **P0 non si chiude con la revisione delle tolleranze.** P1 riduce la frequenza dei picchi sterzati, non il massimo.
