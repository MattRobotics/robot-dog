# MATDOG — RCA 01: inversione della velocità FAST con l'attrito e stacco del piede (13/11/2026)

> **Revisione 2, 13/11/2026: rilievi Codex recepiti (§12).** Rispetto alla prima versione cambiano:
> - l'interpretazione del supporto (§0 n. 10, §4, §8.1): il margine di `m6_gait` **non** è un artefatto;
> - la portata di D5 e D9 (§6);
> - la tabella di clearance, ora solo su swing completi e con stato coerente (§5);
> - le affermazioni sugli attuatori (§7) e la portata della regressione (§11).
>
> La prima versione è archiviata in `report/REPORT_RCA01_MU_v1_pre_codex.md`.
>
> **Revisione 2.1, stesso giorno: difetto del controfattuale fantasma (D4), trovato nella sessione vNext.**
> - `swing_contact=0` rende non collidente solo il cilindro del piede in swing. Il guscio della tibia della stessa zampa
>   tocca il suolo nel 23,7 % dei campioni (~14 N, −8 N in avanti).
> - D4′ (`swing_leg_contact=0`, tutta la parte distale della zampa) è il controfattuale corretto: **108,1 mm/ciclo, 39,4
>   mm/s, il 97 % della legge, a μ 1,0**.
> - I deficit di clearance "fantasma" del §5 diventano −1,1 mm davanti e −2,9 mm dietro (F40T); prima −1,7 / −1,5.

Solo gemello MuJoCo e analisi offline. Nessun hardware, flash, push, merge, reset o clean. Firmware Motion, calibrazioni,
box, limiti, STOP e criteri invariati. Nessun parametro del gemello tarato: i controfattuali (§6) cambiano una condizione
alla volta, sono solo diagnostici e non sono configurazioni.

Classi: **VERIFIED OFFLINE** (dimostrato nel gemello o sui dati), **IMPLEMENTED NOT VALIDATED**, **HYPOTHESIS**, **UNKNOWN**.
Ogni risultato del gemello vale per il gemello `dev7_ground`, non per il robot.

## 0. Esito

| # | Conclusione | Classe |
|---|---|---|
| 1 | La velocità negativa a μ 1,0 è uno **spostamento reale all'indietro** del modello: origine base, COM e integrali delle velocità coincidono entro 0,05 mm/s (−6,65 mm/s = −133 mm in 20 s); 6 cicli completi su 6 sono negativi | VERIFIED OFFLINE |
| 2 | Configurazione dei contatti e solver **corretti**: μ effettivo 0,8 / 1,0 su tutti i contatti piede–suolo, condim 3, priority 0, nessuna `pair`, nessun contatto non‑piede; iterazioni p50 7 / max 12 su 100, nessun warning; risultato stabile con passo 0,25 ms (−6,50) e noslip 20 (−7,45 mm/s) | VERIFIED OFFLINE |
| 3 | **Prima divergenza** μ 0,8 / 1,0: **t = 4,5200 s** (dev.7: 4,5435 s), primo swing della camminata (RH): il punto di contatto più scarico (1,0 N) del **piede RH trascinato** raggiunge il cono 0,8; a μ 1,0 ci arriva 6 ms dopo. Prima di allora le due simulazioni differiscono solo per arrotondamento (‖ΔF‖ ≤ 1,3·10⁻¹¹ N, ‖Δpos‖ ≤ 6·10⁻¹⁷ m) | VERIFIED OFFLINE |
| 4 | **Prima divergenza significativa**: primo swing anteriore (LF, 6,46 s): COM +10,4 → −6,5 mm nello swing. Dal secondo (RF, 7,84 s) il regime è fissato: il piede anteriore avanza 13 mm invece di 81 e il corpo arretra di 36 mm per swing anteriore | VERIFIED OFFLINE |
| 5 | **Causa primaria (parte dominante della perdita, nel gemello)**: attrito sui piedi in swing che non si staccano. Nelle quattro baseline gli anteriori non superano 0,1 mm di distanza geometrica in nessun campione degli swing completi. Non vale in tutti i casi: nel controfattuale senza attrito in swing uno swing LF arriva a 0,210 mm, sopra 0,1 mm per il 10 % dello swing. Il solo attrito dei piedi in swing commuta il risultato: swing 0,8 / appoggio 1,0 → +86,6 mm/ciclo; swing 1,0 / appoggio 0,8 → −72,3. D1–D4 non isolano completamente cedimento, rollio, ritardo e chatter | VERIFIED OFFLINE |
| 6 | **Saturazione del modello attuatore**: l'UPPER anteriore in swing è al clamp del duty per il 64 % dello swing a μ 1,0, contro il 25 % a μ 0,8. Anche i LOWER posteriori in appoggio saturano (2,3 %). D5 porta il clamp da 1 a 2 su **tutti** i servo e l'inversione sparisce (+43,6 mm/ciclo, perdita −52 %). Questo dimostra la **sensibilità del risultato al limite del modello**, non che serva in esclusiva la saturazione anteriore. Il clamp 2 è oltre l'intervallo fisico (UPPER fino a 1,386). La coppia reale dell'ST3215 sotto carico non è misurata | VERIFIED OFFLINE (modello); UNKNOWN (HW) |
| 7 | **Perché il piede non si stacca** (gemello): il lift di legge (5 mm) è consumato tutto. Quando la zampa in swing si scarica, il treppiede di servo solo‑P cede: corpo −2,7 mm e rollio ~11 mrad verso l'angolo in swing; il comando sul corpo reale vale ≈ 0 mm. Il ritardo dei giunti in swing abbassa il piede di altri ~1,3 mm. Con il piede in swing senza collisione, la traiettoria reale passa **1,5–1,7 mm sotto il piano** a metà swing. Le versioni della tabella stanno su pose del corpo diverse: le differenze fra mediane **non** sono una scomposizione additiva (§5) | VERIFIED OFFLINE (gemello) / UNKNOWN (robot) |
| 8 | La compensazione IMU dei 38,9 mrad crea ±4,5 mm di divergenza fra legge e FK nominale; sul corpo reale si annulla quasi tutta. **Non è necessaria** all'inversione (offset nullo a μ 1,0: −3,50 mm/s). Che non contribuisca affatto **non** è dimostrato | VERIFIED OFFLINE |
| 9 | Con solref 0,008 a μ 1,0 la velocità lungo la direzione iniziale resta **−0,35 mm/s** (contro −6,65). Il +0,2 mm/ciclo della prima versione integra lungo la direzione istantanea, che è un'altra metrica. È dimostrato **l'avvicinamento a zero**, non l'inversione del segno della metrica principale. La frenata negli swing anteriori non cambia (−63,5 contro −67,4 mm/ciclo) | VERIFIED OFFLINE (sensibilità) |
| 10 | **Supporto (corretto):**<br>- Il margine dei piedi caricati di `m6_gait` (p05 −117…−173 mm) **non è un artefatto**: misura il supporto che le forze simulate trasmettono davvero, chatter compreso.<br>- A μ 1,0, negli swing anteriori il piede in swing di legge porta in media il 45–48 % del peso, e nel 18–22 % dei campioni i tre piedi d'appoggio di legge sono **tutti scarichi**; 220 campioni su 20 001 non hanno alcun piede caricato. Esempio: a 11,010 s Fz = 0 / 13,91 / 0 / 0 N con RF in swing di legge.<br>- Il margine rispetto al treppiede di legge (+12…+17 mm) è solo **disponibilità geometrica**, non supporto.<br>- Nel controfattuale fantasma corretto (D4′) il treppiede di legge regge da solo il corpo: 108 mm/ciclo, 97 % della legge, peso sui piedi in swing 0. Il D4 originale non lo mostrava, perché la tibia toccava.<br>- `m6_gait` ora riporta quota senza appoggio, peso sui piedi in swing di legge e margine geometrico, ciascuno con il suo nome | VERIFIED OFFLINE; strumentazione applicata (§9) |

## 1. Sorgenti e artefatti

| Voce | Valore |
|---|---|
| Worktree | `~/MATDOG/worktrees/matdog-mujoco-dev7-corr-20261112`, branch locale `sim/mujoco-dev7-correlation-20261112` |
| HEAD | `dddaeed62cf8faee257d5e60907cad8f5da7a7bc`, albero pulito all'inizio. Modifiche di questa RCA **non committate** (§11) |
| Artefatti FAST 40 | `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_FAST40_20261112/`: `SHA256SUMS.txt` (40 540 file) e `SHA256SUMS_CLOSEOUT.txt` verificati OK il 13/11, non modificati |
| Candidato | `patches/F40T_JCB66.patch` SHA256 `4325e374…ba9`; build `builds/F40T_JCB66/build/cosim` SHA256 `c1c29780…901f` (= SHA256SUMS) |
| Controllo | dev.7 `b5610620d0ce`, cosim M6 SHA256 `e68735f2…d762` (= SHA256SUMS M6), `MATDOG_FW_TREE` = `worktrees/matdog-dev7-b561062-src` |
| Gemello | `m6_dev7.py --variant dev7_ground` (noslip 5, base 1,707 kg, IMU 2,0 / 38,9 mrad, gear η 0,5 smooth), scenario `m6_gait` `fast_fwd`, finestra 11–31 s, MuJoCo 4.0.0 |
| Tracce esistenti usate | `p2/twin_final/{final,mu06,mu10}`, `p2/twin_drift/*`, `twin/final`, `p2/foot_lift/{F40T,dev7}.json`, `p2/swing_tracking_hw/` |
| Nuovi artefatti | `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_RCA01_MU_20261113/` (`SHA256SUMS.txt`) |

**Provenienza delle nuove tracce.** Le tracce archiviate contengono solo riepiloghi, quindi ho rieseguito i 4 run
`fast_fwd` necessari (F40T_JCB66 e dev.7, μ 0,8 e 1,0) con una sonda di sola lettura. Il report `m6_gait` di ciascuno è
**identico** a quello archiviato (tutti i campi tranne `wall_s`, `label`, `fast_joy`). Ne segue che:
- la sonda non altera la fisica;
- il gemello è deterministico;
- `fast_fwd` di F40T_JCB66 coincide con quello di F40T_JC e F40T (il passo indietro B66 non tocca l'avanti).

## 2. A — Audit della metrica

`m6_gait.run`: finestra 11–31 s con tutte e quattro le zampe in `WALKING`. Spostamento dell'**origine di `base_link`**
(`qpos[0:3]`) fra il primo e l'ultimo campione, proiettato sulla direzione di marcia all'inizio della finestra (yaw del
quaternione), diviso per il tempo (mm/s) o per i cicli `walk_cycles` (mm/ciclo). Nessuna lacuna nella finestra.

| Run | `m6_gait` | origine base | COM | ∫ v base | ∫ v COM | ∫ v COM, direzione istantanea | Δyaw | cicli < 0 |
|---|---|---|---|---|---|---|---|---|
| F40T_JCB66 μ 0,8 | 32,63 | 32,63 | 32,58 | 32,63 | 32,58 | 33,09 | −14,4° | 0/6 |
| F40T_JCB66 μ 1,0 | **−6,65** | −6,65 | −6,66 | −6,65 | −6,66 | −6,32 | −6,2° | **6/6** |
| dev.7 μ 0,8 | 21,89 | 21,89 | 21,89 | 21,89 | 21,89 | 21,96 | −3,1° | 0/7 |
| dev.7 μ 1,0 | 12,63 | 12,63 | 12,63 | 12,63 | 12,63 | 12,80 | −1,1° | 0/7 |

Velocità in mm/s. Le forze di contatto convertite dal contact frame (righe di `contact.frame`, forza su geom2) coincidono
**esattamente** con `cfrc_ext[foot, 3:6]` in tutti i 40 000 campioni (scarto 0): segni e frame sono verificati.
Σ Fz media 27,5 N = peso. **La metrica è corretta: a μ 1,0 il modello arretra davvero, a ogni ciclo.**

## 3. B — Prima divergenza μ 0,8 / 1,0

![prima divergenza](~/MATDOG/verification-artifacts/MATDOG_MUJOCO_RCA01_MU_20261113/fig/fig2_first_divergence.png)

Stesse condizioni iniziali e stessi comandi. La camminata parte a 3,761 s; il primo swing è RH da 4,38 s.

| t (s), F40T_JCB66 | Evento |
|---|---|
| 0 – 4,5195 | identiche a meno dell'arrotondamento (μ entra nella regolarizzazione del solver anche per contatti aderenti) |
| **4,5200** | punto di contatto più scarico del **piede RH in swing** (Fn 1,02 N, penetrazione 0,039 mm, il piede è trascinato) sul cono 0,8: scivola. A μ 1,0 lo stesso punto arriva al cono a 4,5260 |
| 4,548 | LF in appoggio, scaricato a 1,7 N dallo sway, sul cono 0,8 (a μ 1,0 servirebbe 0,974: aderisce) |
| 4,587 / 4,858 / 5,33 | piede Δxy 0,1 mm / corpo 0,1 mm / corpo 1 mm |
| 4,60 | primo frame firmware diverso (release 229): l'IMU vede un assetto diverso, il circuito chiuso propaga |
| **6,46** | **primo swing anteriore (LF)**: COM +10,4 / −6,5 mm, avanzamento del piede 76,7 / 49,3 mm, saturazione UPPER 12 % / 47 % dello swing |
| 7,84 | RF: avanzamento del piede 80,7 / **13,2 mm**, carico 57 % / 89 % dello swing, UPPER sat 33 % / 63 %, COM +8,2 / **−36,0 mm**. Regime fissato |

dev.7: stessa sequenza (4,5435 s, punto RH 0,96 N); il primo swing anteriore (6,34 s) passa da +2,9 a −3,1 mm, senza
saturazione apprezzabile (5 %).

![timeline](~/MATDOG/verification-artifacts/MATDOG_MUJOCO_RCA01_MU_20261113/fig/fig1_timeline_B66.png)

## 4. Evidenze μ 0,8 / μ 1,0 (finestra 11–31 s)

| Grandezza | F40T_JCB66 μ 0,8 | **F40T_JCB66 μ 1,0** | dev.7 μ 0,8 | dev.7 μ 1,0 |
|---|---|---|---|---|
| COM per ciclo: totale | 90,8 mm | **−17,3** | 54,9 | 32,0 |
| … negli swing LF + RF | +19,9 | **−67,4** | +6,2 | −11,7 |
| … negli swing RH + LH | +47,4 | +38,3 | +25,1 | +20,7 |
| … a quattro appoggi | +23,5 | +11,9 | +23,6 | +23,0 |
| Piede anteriore in swing: quota caricata (Fz > 0,1 N) | 0,58 | **0,87** | 0,56 | 0,64 |
| … Fn media | 10,2 N | **12,2 N** | 8,3 | 9,7 |
| … Fx media (direzione di marcia) / impulso per ciclo | −6,8 N / −3,68 Ns | −8,9 / −4,83 | −5,1 / −2,18 | −7,1 / −3,07 |
| … scivolamento in avanti quando caricato | +83 mm/s | **+25** | +70 | +58 |
| … \|Ft\|/(μFn) p50 (1 = sul cono) | 1,00 | 0,99 | 1,00 | 1,00 |
| Durante gli swing anteriori: velocità COM | +18 mm/s | **−62** | +7 | −14 |
| … scivolamento dei posteriori in appoggio | −5 / −3 mm/s | **−29 / −36** (68–73 % del tempo) | −4 / −4 | −2,5 / −4,4 |
| UPPER anteriore in swing: errore totale p50 / p95 | 65 / 148 tick | **165 / 230** | 68 / 105 | 89 / 146 |
| … di cui profilo (setpoint − goal) p50 | 24 | 23 | 21 | 21 |
| … di cui anello P (giunto − setpoint) p50 / p95 | 42 / 101 | **136 / 201** | 44 / 61 | 64 / 109 |
| … duty p50 / quota saturata | 578 ‰ / 0,25 | **1 000 ‰ / 0,64** | 606 / 0,00 | 755 / 0,29 |
| UPPER anteriore in appoggio, sat | 0 | 0 | 0 | 0 |
| Corpo: z − legge / pitch medio | −1,30 mm / −30 mrad | +0,10 / −27 | −1,42 / −30 | −0,79 / −31 |
| Disponibilità geometrica: margine COM–treppiede di legge, swing LF / RH (p50); **non è supporto** | +16,8 / +4,5 mm | +16,1 / +4,0 | +12,8 / +2,4 | +13,0 / +1,1 |
| Supporto trasmesso: peso medio sul piede in swing di legge, swing LF+RF / RH+LH | 27 / 15 % | **47 / 20 %** | 23 / 18 % | 27 / 23 % |
| … swing anteriori con i tre piedi d'appoggio di legge tutti scarichi | 0 | **18–22 %** | 0 | 0 |
| … campioni senza alcun piede caricato (finestra) | 0 | **220 / 20 001** | 0 | 0 |
| Chatter: campioni con ≤ 1 piede caricato / Σ Fz p01 / vz rms | 2,5 % / 11,7 N / 6 mm/s | **13,9 % / 0,0 N / 17 mm/s** | 2,5 % / 12,6 / 8 | 6,1 % / 13,1 / 11 |

Lettura:
- tutta la perdita sta negli **swing anteriori** (−87 mm/ciclo su −108);
- il COM sta geometricamente dentro il treppiede di legge, ma nel gemello quel treppiede **non porta** tutto il corpo: a μ 1,0
  il piede anteriore in swing ne porta in media il 47 %, e per il 18–22 % degli swing anteriori è l'unico appoggio. Che il
  treppiede da solo basterebbe lo mostra solo il controfattuale fantasma (D4);
- il piede anteriore in swing resta quasi fermo sul suolo mentre l'UPPER satura: la zampa spinge il corpo all'indietro e i
  posteriori scivolano al limite del cono;
- l'UPPER in appoggio non satura mai: la saturazione `duty p95 1000 ‰` della fase 2 viene **dallo swing**, non dall'appoggio.

## 5. C — Clearance

Mediana sugli **swing completi** della finestra (14–16 per gruppo), all'istante di picco della legge (u = 0,5), in mm.
Correzioni della revisione 2:
- gli swing tagliati dai bordi della finestra (2 per run) sono esclusi;
- `act` è la FK dello stato registrato (base e giunti), lo stesso usato da `actnom` e `geom`. `rec["feet"]` del gemello è
  calcolato prima dell'ultimo passo fisico del campione: è 0,5 ms più vecchio di `qpos`, con scarti fino a 0,06 mm.

Versioni:
- `nom` = FK dei giunti comandati sul corpo di legge livellato (legge + IK + compensazione IMU);
- `cmd` = stessi giunti sul corpo simulato;
- `actnom` = FK dei giunti **effettivi** sul corpo livellato;
- `act` = punto più basso del battistrada, giunti e corpo reali;
- `geom` = distanza segnata cilindro–piano (`mj_geomDistance`, < 0 = sotto il piano).

`nom` e `actnom` stanno sul corpo di legge, `cmd`, `act` e `geom` sul corpo simulato. Le differenze fra le colonne mescolano
pose diverse e **non** sono una scomposizione additiva.

Tabella completa: `foot_lift_v2/clearance_table.txt` (la prima versione in `foot_lift/`).

| Caso | Zampe | legge | nom | cmd | actnom | act | geom | quota caricata | gap > 0,1 mm p50 / max | corpo dz / pitch / \|roll\| |
|---|---|---|---|---|---|---|---|---|---|---|
| F40T_JCB66 μ 0,8 | ant. | 4,98 | 0,70 | 3,71 | −2,04 | 0,03 | 0,00 | 0,58 | 0 / 0 | −1,3 / −29 / 2,0 |
| | post. | 4,98 | 9,58 | 1,49 | 8,80 | 0,11 | 0,07 | 0,39 | 0,03 / 0,11 | −1,6 / −37 / 6,3 |
| F40T_JCB66 μ 1,0 | ant. | 4,98 | 0,67 | 4,74 | −2,76 | 0,00 | 0,00 | 0,87 | 0 / 0 | −0,4 / −26 / 6,8 |
| | post. | 4,98 | 9,13 | 1,97 | 7,88 | 0,10 | 0,02 | 0,47 | 0 / 0,01 | −1,4 / −33 / 5,8 |
| F40T_JCB66 μ 1,0, swing senza attrito | ant. | 4,98 | 0,54 | 0,68 | 0,26 | 0,02 | −0,01 | 0,91 | 0 / **0,10** (picco 0,210 mm) | −2,4 / −24 / 10,0 |
| **F40T_JCB66, swing fantasma** | **ant.** | 4,98 | 0,48 | **−0,34** | −0,39 | **−1,69** | **−1,74** | 0 | 0 / 0 | **−2,7 / −20 / 11,6** |
| | **post.** | 4,98 | 9,63 | **0,65** | 8,22 | **−1,53** | **−1,59** | 0 | 0 / 0 | −2,1 / −39 / 8,0 |
| dev.7 μ 0,8 | ant. | 5,96 | 1,86 | 1,80 | 0,51 | 0,02 | 0,00 | 0,56 | 0 / 0 | −2,6 / −22 / 3,2 |
| | post. | 5,99 | 9,83 | 2,46 | 8,42 | 0,18 | 0,14 | 0,54 | 0,29 / 0,33 | −1,4 / −37 / 7,2 |
| **dev.7, swing fantasma** | ant. | 5,96 | 1,94 | 0,52 | 0,89 | **−0,70** | −0,74 | 0 | 0 / 0 | −2,9 / −19 / 11,0 |
| | post. | 5,99 | 9,88 | 0,67 | 8,07 | **−1,76** | −1,82 | 0 | 0,07 / 0,13 | −2,0 / −42 / 9,6 |

Rispetto alla prima versione i valori cambiano di ≤ 0,2 mm (`nom`) e ≤ 0,07 mm (`act`); le conclusioni restano.

Lo **swing fantasma** è il controfattuale in cui il piede in swing di legge non collide col suolo. **Revisione 2.1:** in quella
versione la tibia della zampa in swing tocca ancora il suolo e regge parte del corpo, quindi i deficit qui sopra sono un
limite inferiore. Con tutta la parte distale non collidente (`foot_lift_v2/clearance_ghostleg.txt`):

| Caso | Zampe | cmd | act | geom | corpo dz / \|roll\| |
|---|---|---|---|---|---|
| F40T_JCB66 μ 0,8 | ant. | 0,02 | **−1,08** | −1,16 | −2,5 / 13,1 |
| | post. | −0,96 | **−2,93** | −3,00 | −2,9 / 8,9 |
| dev.7 μ 0,8 | ant. | 1,25 | −0,09 | −0,14 | −2,5 / 11,4 |
| | post. | −1,06 | **−3,35** | −3,41 | −2,4 / 12,9 |

Il deficit è maggiore sulle zampe **posteriori**.

Lettura, F40T_JCB66 anteriori (indicativa, non additiva):
1. **Legge → IK + IMU** (`nom` − legge): −4,3…−4,5 mm sul corpo livellato. I posteriori invece salgono di +4,6.
2. **Corpo reale** (`cmd`, stessi giunti sul corpo simulato):
   - a contatto `cmd` vale 3,7–4,7 mm, perché il piede trascinato sostiene il corpo;
   - nel caso fantasma il treppiede cede (corpo −2,7 mm, rollio 11,6 mrad verso l'angolo in swing) e `cmd` vale −0,34 mm
     davanti e +0,65 mm dietro: **il comando sul corpo reale scarico è ≈ 0 mm**.
3. **Ritardo dei giunti** (`actnom` − `nom`, corpo livellato):
   - −0,9 mm nel caso fantasma, con UPPER a 85 tick (profilo ~22, anello P);
   - −2,7 / −3,4 mm a μ 0,8 / 1,0, con UPPER a 154 / 206 tick: il ritardo cresce con l'attrito.
4. **Piede reale** (`act`): −1,7 mm nel caso fantasma; a contatto 0,00–0,03 mm, cioè appoggiato.
   - Nelle quattro baseline gli anteriori non superano mai 0,1 mm di distanza geometrica negli swing completi.
   - Senza attrito in swing uno swing LF arriva a 0,210 mm (§0 n. 5).
5. **Geometria reale** (`geom` − `act`): −0,01…−0,06 mm. È il bordo del cilindro inclinato. La quota di riferimento del
   battistrada è adeguata, quella nominale del `foot_link` non è mai stata usata.

**Compensazione IMU (38,9 mrad).**
- Divergenza cinematica legge/FK nominale: ±4,5 mm, cioè anteriori più bassi e posteriori più alti.
- Sul corpo reale si annulla quasi tutta, perché il corpo si inclina come comandato. Con offset nullo (`p2/foot_lift/F40T_imu0`,
  μ 0,8), `cmd` vale 3,0 / 1,9 mm contro 3,7 / 1,5–2,3 con offset, e `act` 0,06 / 0,14.
- **Non è necessaria** al mancato stacco né all'inversione (§6, D6). Che non contribuisca affatto non è dimostrato. Resta il
  consumo dell'inviluppo di tilt già noto.

**dev.7 contro F40T_JCB66.** Il deficit anteriore nel caso fantasma è −0,7 mm contro −1,7: la dev.7 ha 1 mm di lift in più
e un ritardo UPPER minore. Le posteriori sono simili (−1,8 / −1,5).

## 6. D — Controfattuali (F40T_JCB66 salvo indicazione, `fast_fwd`, una sola condizione ciascuno)

![bilancio per fase](~/MATDOG/verification-artifacts/MATDOG_MUJOCO_RCA01_MU_20261113/fig/fig3_phase_budget.png)

Colonna mm/ciclo: avanzamento del COM lungo la direzione **istantanea**, per ciclo (`phases`). La velocità di `m6_gait`
(mm/s lungo la direzione iniziale) è un'altra metrica: le due coincidono quasi sempre, ma non vicino a zero (D9).

| Id | Condizione cambiata | Distingue | mm/ciclo (legge 111) | Swing ant. LF+RF | Esito |
|---|---|---|---|---|---|
| — | μ 0,8 / μ 1,0 (riferimenti) | — | 90,8 / −17,3 | +19,9 / −67,4 | — |
| D1 | μ 1,0, attrito dei piedi in swing di legge 0,8 | frenata da contatto in swing | **86,6** | +22,0 | l'inversione sparisce |
| D2 | μ 0,8, attrito dei piedi in swing 1,0 | idem, verso opposto | **−72,3** | −140,0 | l'inversione compare (peggiore: appoggio a 0,8 cede di più) |
| D3 | μ 1,0, piedi in swing senza attrito | idem; ritardo servo senza attrito | 122,3 | +44,9 | UPPER anello P 14 tick, sat 0; chatter assente (picco 50 Hz); piede ancora a contatto il 93 % con 3,4 N |
| D4 | μ 1,0, swing senza collisione del **solo piede** (fantasma) | contatto in swing | 84,2 | +23,7 | nessuna inversione. **Difetto (revisione 2.1):** il guscio della tibia della zampa in swing tocca il suolo nel 23,7 % dei campioni (~14 N) |
| D4′ | μ 1,0, swing senza collisione di **tutta la parte distale** (tibia e piede) | contatto in swing, corretto | **108,1** (39,4 mm/s) | +43,3 | **97 % della legge**: senza contatto della zampa in swing il FAST 40 del gemello raggiunge la velocità di legge anche a μ 1,0. UPPER anteriore: anello P 14 tick, saturazione 0 |
| D5 | μ 1,0, clamp del duty 1 → 2 su **tutti** i servo | sensibilità al limite del modello attuatore | 43,6 | −23,7 | niente inversione, perdita −52 %. Intervento diagnostico oltre l'intervallo fisico: UPPER anteriore in swing con duty p95 1,307 e max 1,386. Rispetto al nuovo clamp l'UPPER non satura mai (0 %); supera il clamp normale (1) per il 31 % dello swing. Non isola l'UPPER anteriore, perché nella baseline saturano anche i LOWER posteriori in appoggio (2,3 %) |
| D6 | μ 1,0, offset IMU 0 / 0 | assetto IMU | −9,0 | −65,3 | arretra ancora: l'IMU non è necessaria |
| D7 | μ 1,0, passo fisico 0,25 ms | integrazione | −16,9 | −67,2 | robusto |
| D8 | μ 1,0, noslip 5 → 20 | convergenza attrito | −19,6 | −69,5 | robusto |
| D9 | solref 0,004 → 0,008, a μ 0,8 / 1,0 | rigidezza di contatto | 94,8 / 0,2 | +24,2 / −63,5 | frenata negli swing anteriori quasi invariata, recupero a quattro appoggi. A μ 1,0 la velocità lungo la direzione iniziale resta **−0,35 mm/s** (base −6,65): la risposta del contatto **avvicina a zero** il risultato, ma il cambio di segno della metrica principale non è dimostrato |
| D1′ | dev.7 μ 1,0, swing 0,8 | controllo | 49,8 (μ 0,8: 54,9) | +6,5 | stessa causa sulla dev.7 (78 % della perdita) |

Nessun controfattuale è una configurazione proposta. Il "cedimento del corpo" non ha un controfattuale a sé: rendere più rigido
il servo cambierebbe anche la zampa in swing. Lo isola il caso fantasma (§5).

## 7. Cause

**Causa primaria — VERIFIED OFFLINE (gemello), parte dominante.** I piedi in swing restano sul suolo e vengono trascinati,
gli anteriori con 10–12 N. La forza d'attrito sul piede in swing (μ·Fn) frena il corpo: D1, D2, D3 e D4 attribuiscono a
questo una parte dominante della perdita. Non separano completamente cedimento, rollio, ritardo e chatter, che agiscono insieme.

**Concausa nel modello — VERIFIED OFFLINE solo per il modello attuatore.**
- **Saturazione dell'UPPER anteriore (modello).** A μ 1,0 la resistenza del piede trascinato porta l'UPPER al clamp del duty
  del modello. Il piede quasi si ferma (+25 mm/s); la zampa, spinta in avanti, sposta il corpo indietro, e i posteriori in
  appoggio scivolano al cono.
  - D5 mostra la sensibilità a quel limite, non una necessità esclusiva della saturazione anteriore.
  - Parametri del modello: coppia di stallo da specifica del costruttore (VENDOR, `servo.py`), η 0,5 HYPOTHESIS.
  - La coppia reale sotto carico e in velocità **non è misurata**. M6 documenta ancora un duty UPPER simulato/HW ≈ 1,66.
  - Il ritardo articolare coincide con l'HW sui 12 campioni FAST anteriori disponibili, che includono retromarcia, diagonale,
    cambi di modo e arresto. È compatibilità del ritardo in quelle osservazioni, non una validazione della capacità dinamica
    del servo.
- **Retroazione positiva.** L'UPPER in ritardo (165 tick) abbassa ulteriormente il piede (−3,2 mm), che si carica di più
  (12 N) e aumenta l'attrito.
- **Effetto della legge F40.** Rispetto alla dev.7 ha 1 mm di lift in meno, passo e velocità di swing maggiori e il 25 %
  di saturazione già a μ 0,8. Per questo la stessa causa porta la dev.7 a rallentare e F40 a invertire.

**Causa del mancato stacco — VERIFIED OFFLINE nel gemello, UNKNOWN sul robot.** Il bilancio di clearance (§5) è negativo
anche senza attrito: la cedevolezza del treppiede (servo solo‑P, k 8,4 ‰/tick, MEASURED) più il ritardo di profilo e
d'anello della zampa in swing superano il lift di legge. L'attrito non crea il problema, lo amplifica.

**Conseguenze, non cause — VERIFIED OFFLINE.**
- Chatter stick‑slip a ~200 Hz: sparisce senza attrito in swing (D3).
- Scivolamento all'indietro dei posteriori in appoggio.
- Aumento di Fn sul piede anteriore (12 N contro 3,4 N senza attrito).
- Errore UPPER a 136 tick d'anello.

**Residui.**

| Residuo | Classe |
|---|---|
| Se il piede del robot si stacca: i log HW non hanno quota né forza. Il ritardo in swing del gemello coincide con l'HW (fase 2 §3.2); il cedimento in appoggio del gemello è un po' maggiore (LOWER posteriore −12 contro −8 tick), quindi il gemello potrebbe sovrastimare il deficit | UNKNOWN |
| μ reale piede–pavimento | UNKNOWN |
| Rigidezza e smorzamento reali del contatto: decidono il segno a μ 1,0 (D9), non la frenata | UNKNOWN |
| Coppia disponibile dell'UPPER sotto carico e in velocità (η 0,5 è HYPOTHESIS): fissa la soglia di μ dello stallo | UNKNOWN |
| μ di innesco fra 0,8 e 1,0: non scansionato; dipende dal margine di coppia sopra | UNKNOWN |
| Deriva di rotta a soglia (−14° in 20 s a μ 0,8): probabile asimmetria della frenata dei due anteriori | HYPOTHESIS, non indagata |

## 8. Difetti di strumentazione e di modello

1. **`m6_gait`, margine di supporto** (`support p05/min`). **Correzione della revisione 2:** la prima versione lo chiamava
   artefatto; non lo è.
   - Usa i piedi con Fz > 0,1 N nel singolo campione da 1 ms, quindi descrive il supporto che le forze simulate trasmettono
     davvero.
   - Quando legge −170 mm, il corpo è appoggiato soprattutto sul piede in swing di legge, con i piedi d'appoggio di legge
     scarichi o staccati di millimetri. Esempio Codex: a 11,010 s, Fz 0 / 13,91 / 0 / 0 N e LF/RH a 1,6 / 4,7 mm dal suolo.
   - Il campo ha due limiti:
     - scarta i campioni senza alcun piede caricato (220 su 20 001 a μ 1,0), che non contano;
     - è rumoroso per il chatter.
   - Il margine rispetto al treppiede di legge (+12…+17 mm negli swing anteriori, +1…+5 mm nei posteriori) è solo
     **disponibilità geometrica**. Leggerlo come margine fisico produrrebbe un **falso PASS**.
   - Nessuna conclusione dei rapporti FAST 40 si basa su quel campo (grep su rapporti e `compare_p2.md`).
2. **`m6_gait` `drag`** (Fz > 2 N per campione). Col chatter è un limite inferiore: per gli anteriori la quota a contatto
   è 0,58–0,87 contro `drag` p50 (tutte le zampe) 0,47–0,58. La misura robusta è la distanza geometrica, ora in `foot_lift_rca.py`.
3. **Modello.**
   - Nessun errore di configurazione: attrito, condim, priority, `pair`, solver.
   - Il chatter stick‑slip è un comportamento del modello (contatto rigido, servo P, attrito smooth) **non validato**.
   - Le conclusioni non dipendono da esso (D3, D7, D8, D9), salvo il segno a μ 1,0.

## 9. Patch minima proposta

- **Solo strumentazione, applicata nella revisione 2.** La patch proposta nella prima versione è sostituita da campi con
  nomi che non suggeriscono equilibrio. `m6_gait.py` aggiunge, accanto al campo esistente che resta invariato:
  - `no_loaded_foot_share`: quota della finestra senza alcun piede caricato;
  - `weight_on_law_swing_feet_mean`: peso portato dai piedi in swing di legge;
  - `law_stance_geometric_margin_mm_p05_min`: disponibilità geometrica, **non** supporto.

  Nessuno di questi campi è un gate PASS.
  - Regressione: i campi preesistenti sono identici al report archiviato di `fast_fwd` F40T_JC (`regress_v2/compare.txt`).
  - La patch della prima versione resta in `proposed/`, superata.
- **Nessuna patch di legge, firmware o modello fisico.** Le leve sono:
  - il bilancio di clearance (lift contro cedimento del treppiede più ritardo);
  - il margine di coppia dell'UPPER in swing.

  Appartengono alla fase di sviluppo del gait, esclusa da questo mandato. La misura HW (§10) ne decide la priorità.

## 10. Dati e misure hardware realmente necessari (dev.7 installata, nessun flash)

| # | Misura | Perché offline non basta |
|---|---|---|
| 1 | **Ripresa laterale ≥ 60 fps dei piedi anteriori e posteriori**, dev.7 FAST 32 mm/s, 3–4 cicli, riferimento millimetrato: il piede si stacca? per quanto dello swing striscia? | Decide se il meccanismo del gemello vale sul robot. È la misura principale |
| 2 | **μ statico piede–pavimento d'uso** (piano inclinato, un piede campione) | Colloca il robot sulla curva μ del gemello: fra 0,6 e 1,0 il gemello passa da +40 a −7 mm/s |
| 3 | **Cedimento e rollio del corpo durante uno swing anteriore** (video della misura 1 con riferimento sul telaio, oppure IMU a ≥ 50 Hz in una finestra breve) | Il gemello prevede −2,7 mm e 11 mrad con la zampa scarica. Gli snapshot esistenti a 8,33 Hz danno 4 campioni per swing |
| — | Non necessarie per questa RCA: ritardo dei giunti in swing (confrontato solo come compatibilità, §7), configurazione e solver (esclusi offline), offset IMU (non necessario all'inversione; resta aperto per l'inviluppo) | — |

Secondaria, solo al banco: coppia‑velocità dell'ST3215 sotto carico a 1 000–1 500 step/s. Fissa la soglia di stallo dell'UPPER.

**Misure indicate dalla revisione Codex** (complementari, non sostitutive):
- video calibrato e sincronizzato con la fase del gait: clearance, avanzamento, cedimento del telaio, rollio;
- carico ai piedi, per distinguere vicinanza al suolo, contatto e supporto;
- attrito statico e in scorrimento, più cedevolezza del contatto nelle condizioni d'uso;
- coppia–velocità dell'ST3215 sotto carico, con tensione, duty ed encoder sincronizzati, prima di assumere il limite
  dell'attuatore come vincolo di progetto.

Un video HW (`1000083764.mp4`) è stato pre‑analizzato a parte: `MATDOG_VIDEO_HARDWARE_PREANALYSIS.md`. Ha provenienza ignota
e non sostituisce la misura 1.

## 11. Modifiche al worktree e riproducibilità

Non committate, sopra `dddaeed`; il diff è salvato in `tools/worktree_tracked_changes.patch`.

| File | Modifica | Effetto sulla fisica |
|---|---|---|
| `dynamics.py` (+3) | `Twin.probe`: callback opzionale dopo ogni record | nessuno: report identici bit a bit all'archivio (§1) |
| `gait_study/fast40/foot_lift_rca.py` (+16/−2) | campi `actnom`, `geom` e `geom_gap_share` | campi preesistenti identici all'archivio `p2/foot_lift` (F40T, dev.7) |
| `gait_study/fast40/mu_rca.py` (nuovo) | `run` (sonda, `--cf`, `--compare`), `contacts` (per punto), `footlift` (C in un controfattuale), `analyse`, `plot`; revisione 2: saturazione rispetto al clamp del run, `support` (supporto trasmesso), `clearance` (tabella) | i controfattuali solo con `--cf` |
| `gait_study/fast40/foot_lift_rca.py`, revisione 2 | solo swing completi; `act` dalla FK dello stato registrato; `act_record_offset_max_mm` | cambia di proposito i valori della tabella (§5) |
| `m6_gait.py`, revisione 2 | `no_loaded_foot_share`, `weight_on_law_swing_feet_mean`, `law_stance_geometric_margin_mm_p05_min` | nessuno: campi esistenti identici |

**Test di regressione eseguiti.** Classe: VERIFIED OFFLINE.
1. `mu_rca.py run … --compare <JSON archiviato>`: IDENTICAL sui quattro confronti originali (F40T_JCB66 μ 0,8 / 1,0 e dev.7
   μ 0,8 / 1,0). Con la versione finale dello strumento, dopo il refactor, sono archiviati solo **B66 μ 1,0 e dev.7 μ 0,8**
   (`regress/final_*`).
2. Array del run prima e dopo il refactor: identici, NaN inclusi.
3. `foot_lift_rca.py` esteso (prima versione): campi esistenti identici a `p2/foot_lift/{F40T,dev7}.json`. La revisione 2 li
   cambia di proposito (swing completi, `act` coerente con `qpos`).
4. Revisione 2:
   - `m6_gait.py` con i nuovi campi: campi preesistenti identici all'archivio;
   - `analysis_v2.json` ricalcolata dagli NPZ esistenti, senza nuove simulazioni.

**Test proposti per ogni futura variante FAST** (nessuno modifica i gate firmware):
- R1: `fast_fwd` a μ 0,6 / 0,8 / 1,0 più D1 (`--cf swing_mu=0.8` a μ 1,0). Criterio: nessuna inversione e perdita da
  0,8 a 1,0 non peggiore della dev.7 (−42 %).
- R2: `mu_rca.py footlift swing_contact=0`. `act` a u = 0,5 > 0 per anteriori e posteriori: clearance positiva con il
  treppiede carico. È un criterio del gemello, subordinato alla misura HW 3.
- R3: quota saturata dell'UPPER anteriore in swing a μ 1,0.
- R4: D7 / D8 / D9 sul risultato finale.

**Comandi.** Da `06_Software/Matdog_MuJoCo`, con `~/MATDOG/tools/mujoco-venv/bin/python` e il `MATDOG_FW_TREE` della build:
- gli script `run4.sh`, `run_footlift.sh`, `run_cf.sh`, `run_footlift_cf.sh`, `run_ghost_contacts.sh` e `run_sens.sh`
  nella cartella artefatti;
- poi `mu_rca.py analyse analysis.json runs cf`;
- infine `python3 -I mu_rca.py plot fig runs contacts`, con matplotlib di sistema e `analysis.json` copiato in `fig/`.

Gli script d'esplorazione usati durante l'indagine sono in `tools/exploration/`. Le loro uscite sono incluse in
`analysis.json` e in questo rapporto.

## 12. Recepimento della revisione Codex (13/11/2026)

Fonte: `MATDOG_CODEX_RCA01_REVIEW_20261113.md`, trascrizione del responso Codex fornita dall'utente. Nessuna campagna già
validata è stata ripetuta. Rieseguite solo le 8 scomposizioni di clearance (`run_footlift_v2.sh`, stessa fisica
deterministica) e l'analisi sugli NPZ esistenti (`run_correction_v2.sh`).

| # | Rilievo | Esito Codex | Azione |
|---|---|---|---|
| 1 | Velocità, forze, prima divergenza | PASS | nessuna |
| 2 | D1–D9 e causalità | PARTIAL | §0 n. 5–6 e §7: "parte dominante" invece di isolamento completo; D5 come sensibilità al limite del modello |
| 3 | Scomposizione della clearance | PARTIAL | `foot_lift_rca.py`: swing completi, stato coerente. §5: tabella ricalcolata, lettura non additiva; corretta l'affermazione "mai sopra 0,1 mm in nessun caso" (0,210 mm senza attrito) |
| 4 | Saturazione e coppia ST3215 | PARTIAL | `mu_rca.py`: saturazione rispetto al clamp del run (`above_unit_share` a parte). §7: coppia VENDOR, η HYPOTHESIS, duty UPPER simulato/HW ≈ 1,66, portata dei 12 campioni |
| 5 | Robustezza numerica e contatti | PARTIAL | §0 n. 9 e D9: metrica unica, avvicinamento a zero e non cambio di segno |
| 6 | `m6_gait` e falso PASS (bloccante) | FAIL | §0 n. 10, §4, §8.1 riscritti. Supporto trasmesso misurato (`support`, nuovi campi `m6_gait`); il margine geometrico è etichettato come tale; contati i campioni senza appoggio |
| 7 | Dichiarazioni e artefatti | PARTIAL | §11: regressione finale circoscritta a B66 μ 1,0 e dev.7 μ 0,8; IMU "non necessaria" invece di "non è una causa" |
| — | (revisione 2.1, non rilevato da Codex) D4 fantasma | — | `mu_rca.py`: nuovo controfattuale `swing_leg_contact=0`. Rieseguiti D4′ (`cf_v2/`, `analysis_ghostleg.json`) e le due scomposizioni fantasma (`run_ghostleg_v2.sh`); corretti §0 n. 10, §5 e §6 |

Esito della revisione 2:
- **Nucleo causale invariato:** attrito sui piedi in swing che non si staccano; deficit di clearance di circa −1,7 mm anteriori
  e −1,5 mm posteriori nel caso fantasma.
- **Corrette** le conclusioni sul supporto effettivo, sulla coppia hardware e sulla portata dei controfattuali.
- **Uso consentito:** la RCA orienta le priorità di WALK vNext. Non è un criterio di PASS sul supporto né sulla capacità
  degli attuatori.

