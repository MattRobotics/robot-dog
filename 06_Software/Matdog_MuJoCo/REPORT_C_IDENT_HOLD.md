# MATDOG — Identificazione offline della cedevolezza ST3215 dalle HOLD dev.7 dell'11/11/2026

**Ambito.** Sola lettura dei log già acquisiti:
- sessione `MATDOG_1_0_2_DEV7_HW_20261111T1849Z`, `usb/http.jsonl`;
- geometria MuJoCo canonica (`twin.load_model`);
- mappa gen‑1 (`m2_stand_hold.gen1`).

Nessuna nuova acquisizione e nessuna simulazione di locomozione. Firmware, modello fisico, V7, V7L60, C2, T3, gate, box, STOP
e calibrazioni invariati. C **non** è aggiornata nel candidato.

Strumento: `gait_study/vnext/c_ident_hold.py`, con tre sottocomandi: `quality`, `extract` (MuJoCo) e `solve` (scipy).
Risultati in `~/MATDOG/verification-artifacts/MATDOG_MUJOCO_VNEXT_LOCO_20261113/c_ident/`.

## Verdetto

| Grandezza | Esito | Classe |
|---|---|---|
| **C condivisa UPPER+LOWER** (stesso servo ST3215) | stima ai minimi quadrati **0,036 rad/N·m**; insieme compatibile con i dati: **[0,031; 0,039]** se le coppie statiche non spiegate restano ≤ 0,27 N·m, [0,030; 0,058] fino a 0,30, [0,027; 0,126] fino a 0,35, nessun limite superiore oltre 0,5 | **parzialmente identificata**: limite inferiore robusto, limite superiore non identificabile |
| **C_UPPER e C_LOWER separate** | instabili (C_UPPER 0,020–0,036, C_LOWER 0,033–0,064 a seconda delle equazioni e della postura esclusa); con attrito ≥ 0,30 N·m C_UPPER non ha limite superiore | **UNKNOWN** separatamente |
| **C_HIP** | dentro o al bordo della banda morta (±1–3 tick, duty 0–32 ‰); intervallo [0,009; ∞) già a 0,27 N·m | **UNKNOWN**, non identificabile con queste HOLD |
| **C = 0,0336 di V7** | dentro l'insieme compatibile e a −8 % dalla stima ai minimi quadrati: **non contraddetta**. Però **non è una conferma indipendente**: η = 0,5 del gemello fu scelta da M6 proprio sui duty di queste HOLD | resta **HYPOTHESIS** compatibile con l'HW |

**C usabile nel gemello?** Sì, come valore **nominale coerente** con le HOLD reali. No, come valore **misurato**: in senso
stretto C resta **UNKNOWN** finché non è limitato l'attrito statico per giunto (§6).

Ciò che i dati sostengono davvero è un limite: il robot **non è più rigido** del valore di V7 oltre circa il 12 %
(C ≥ 0,030 con attrito ≤ 0,30 N·m). È sopra la soglia di tolleranza del gate joystick per V7 in FAST (C reale ≥ 0,0282). Per
attriti ≥ 0,35 N·m nemmeno questo è garantito.

## 1. Qualità, unità, segni, sincronizzazione

| HOLD | n | Lettura servo (mediana / max) | Velocità ≠ 0 | Deriva RAW | Variazione del comando | Tensione | Temperatura | IMU |
|---|---|---|---|---|---|---|---|---|
| 120 mm (da sotto) | 33 | 52 / 118 ms | 0 / 396 | 0 tick | 3 tick | 12,02 V | 28–49 °C | 106 ms |
| 110 mm (da sopra) | 34 | 58 / 119 ms | 0 / 408 | 1 tick | 5 tick | 12,02 V | 28–42 °C | 110 ms |
| 105 mm (da sopra) | 35 | 56 / 119 ms | 0 / 420 | 0 tick | 0 | 12,02 V | 28–33 °C | 111 ms |
| 120 mm (da sotto) | 19 | 57 / 118 ms | 0 / 228 | 0 tick | 6 tick | 12,02 V | 29–32 °C | 107 ms |

- 121 istantanee a 1 Hz, tutte valide.
- Si scartano i primi 2 s dopo l'arrivo, come M6.
- **Unità e segni:** RAW in tick (2π/4096); `PresentLoad` in ‰ con il segno al bit 10. In spazio giunto: errore
  e = dir · (cmd − raw), duty u = −dir · load (gen‑1, come `m5_compare`).
- **Sincronizzazione:** letture servo sfalsate di ≤ 120 ms, irrilevanti in statica perché il RAW è fermo.
- **Geometria:** i quattro contatti misurati sono complanari entro **0,31–0,38 mm**.
  - Assetto del corpo sui piedi: roll −4,6…−6,9 mrad, pitch −35…−42 mrad.
  - L'IMU differisce di ≈ 2 / ≈ 40 mrad: è l'offset di montaggio già misurato in M6. Coerente.

**Regolatore** (tutti i 1452 accoppiamenti giunto × campione):
- **u = 0 per |e| ≤ 1 tick; u = 8 · (|e| + 1) ‰ per |e| ≥ 2**: pendenza 8 ‰/tick più un "punch" di 8 ‰.
- Fanno eccezione 10 campioni isolati (0,7 %): 8 a un passo di 8 ‰ dalla legge, 2 con valori non multipli di 8 (36 e 76 ‰);
  compatibili con letture a cavallo di un cambio del comando.
- Il gemello usa invece 8,4 ‰ · e, senza punch: discrepanza di ≈ 1 tick sugli errori statici. Il modello non è stato
  toccato.

**Conseguenza.** `PresentLoad` è **l'uscita del regolatore**, funzione deterministica dell'errore: **non è una misura
indipendente di coppia**.
- Anche il registro `current` (presente nel log, 0–13 conteggi, 0 fino a 32 ‰) cresce in modo monotono con il duty e non
  aggiunge informazione a giunto fermo.
- La scala coppia/duty κ (N·m/‰) si può ottenere **solo** dall'equilibrio statico con il peso noto.

**Limiti di qualità:**
- errori interi: la quantizzazione ±0,5 tick vale 7–17 % sugli UPPER (e = 3–7) e 3–12 % sui LOWER (e = 4–19);
- quattro posture molto simili (105–120 mm), quindi poca eccitazione;
- HIP dentro la banda morta;
- **dentro le HOLD il comando varia fino a 6 tick e il duty fino a 56 ‰ (≈ 0,3 N·m) senza alcun movimento del RAW.**
  Coppie di quella taglia vengono assorbite senza moto, da attrito statico o da forze interne della catena chiusa, e questi
  dati non distinguono le due cose.

## 2. Modello e identificabilità

Per ogni HOLD w, geometria misurata, pavimento nel piano dei contatti, massa 2,807 kg:

  bias_k(q_w) = Σ_l (J_lᵀ F_l)_k + [k attuato] · (κ_tipo · u_k + φ_k)

- bias: forza generalizzata di gravità (MuJoCo);
- J_l: jacobiano del punto di contatto;
- F_l: reazione del suolo, con F_z ≥ 0 e |F_x|, |F_y| ≤ F_z;
- φ_k: coppia non spiegata dal duty (attrito statico del riduttore, banda morta, contatto del piede, errori di modello),
  limitata da |φ| ≤ φ_max.

Poi C = tick / (κ · 8 ‰/tick).

| Formulazione (4 HOLD) | Equazioni / incognite (F, κ) | Rango | Condizionamento | κ fuori dallo span delle F (0 = non identificabile) |
|---|---|---|---|---|
| κ condivisa, solo UPPER+LOWER (HIP libero) | 56 / 49 | 49 | **3,4·10⁴** (forze interne laterali quasi indeterminate) | 0,58 |
| κ UPPER / LOWER separate, HIP libero | 56 / 50 | 50 | 4,0·10⁴ | 0,49 / 0,55 |
| **κ condivisa, 12 giunti (HIP con il suo duty)** | 72 / 49 | 49 | **33** | **0,87** |
| κ HIP / UPPER / LOWER separate | 72 / 51 | 51 | 36 | 0,59 / 0,58 / 0,95 |

- **Strutturalmente** κ è identificabile, e il rango è pieno.
- Senza le equazioni dell'HIP il problema è mal condizionato. Con l'HIP è ben posto, ma le equazioni dell'HIP valgono solo se
  la sua coppia è il duty più un attrito limitato.
- **In pratica** l'identificabilità è limitata dalla coppia non spiegata. Il limite minimo di |φ| che rende i dati coerenti con
  equilibrio e cono di attrito è **0,23–0,25 N·m**. I segnali da spiegare, κ · u, valgono ≈ 0,1–0,85 N·m (LOWER fino a ≈ 0,85, UPPER ≤ ≈ 0,35).

## 3. Stime, residui, intervalli

**κ condivisa, 12 giunti.** Formulazione meglio condizionata; massa 2,807 kg, COM URDF.
- **Minimi quadrati (φ = 0):** κ = 0,00527 N·m/‰, cioè **C = 0,0364 rad/N·m**.
  - Residui: rms 0,084, max 0,31.
  - Escludendo una postura alla volta: **0,0337 / 0,0371 / 0,0379 / 0,0370**.
- **Insieme compatibile** (programmazione lineare, minimo e massimo di κ):

| Coppia non spiegata ammessa φ_max | 0,253 (minimo) | 0,27 | 0,30 | 0,35 | 0,40 | 0,50 N·m |
|---|---|---|---|---|---|---|
| C (rad/N·m) | 0,0326 (unico) | **[0,0315; 0,0386]** | [0,0297; 0,0578] | [0,0272; 0,126] | [0,0251; 0,205] | [0,0217; ∞) |

**Sensibilità della stima ai minimi quadrati:**
- COM ±5 mm in x: 0,0362 / 0,0366;
- COM ±3 mm in y: 0,0364 / 0,0364;
- massa del modello 2,928 kg: 0,0348.

Gli intervalli si spostano di ≤ 0,003.

**Quale φ_max è plausibile?**
- Al banco, a vuoto, l'attrito del servo è 0,03–0,04 dello stallo (≈ 0,09–0,12 N·m), con distacco fino a ≈ 0,09 dello stallo
  (≈ 0,27 N·m) (`servo.py`). La dipendenza dal carico **non è misurata**.
- Le variazioni senza moto viste nelle HOLD arrivano a ≈ 0,3 N·m.
- Il valore realistico cade quindi proprio fra 0,25 e 0,35 N·m, cioè dove l'insieme passa da stretto a illimitato superiormente.

**UPPER / LOWER separate (12 giunti):**
- minimi quadrati: C_UPPER 0,0317, C_LOWER 0,0367; senza HIP: 0,0232 / 0,0445;
- escludendo una postura: C_UPPER 0,025–0,036, C_LOWER 0,033–0,038; senza HIP: 0,020–0,030 / 0,033–0,064;
- con φ_max 0,27: C_UPPER [0,0135; 0,057], C_LOWER [0,030; 0,068]. Con 0,30: C_UPPER [0,012; ∞), C_LOWER [0,028; 0,091].
- Al limite minimo di attrito, con HIP libero, i due κ risultano disgiunti (UPPER 0,0065–0,0132, LOWER 0,0038–0,0058 N·m/‰);
  si sovrappongono solo ammettendo più attrito. La separazione non è sostenuta dai dati.

## 4. Confronto con V7 (C = 0,0336)

- Nelle stesse unità, κ del gemello = τ_stallo(12,02 V) / η / 1000 = 0,00589 N·m/‰ con η 0,5. Con la legge HW da 8 ‰/tick
  questo corrisponde a C = 0,0325.
- La soluzione ad attrito minimo cade esattamente lì (0,0326), e non per caso: M6 calibrò η = 0,5 sui duty LOWER di
  **queste** HOLD.
- Il confronto quindi **verifica la coerenza, non è una misura indipendente**.
- **0,0336 è compatibile** (−8 % dalla stima ai minimi quadrati, dentro l'insieme a φ ≤ 0,27), ma non è più identificato
  di prima.

## 5. Massa reale e ripartizione dei carichi

- **Massa.** κ scala con il peso usato nell'equilibrio, quindi ciò che i dati determinano è il **prodotto C · W**:
  - stima ai minimi quadrati: 1,002 N·rad/(N·m) con 2,807 kg e 1,000 con 2,928 kg, cioè **indipendente dalla massa usata**;
  - insieme a φ ≤ 0,27: [0,87; 1,06].
  - V7 usa C = 0,0336 con W = 28,714 N (2,928 kg): **C·W = 0,965**, il 4 % sotto la stima ai minimi quadrati e dentro
    l'insieme.
  - Se C venisse identificata con 2,807 kg e la compensazione restasse a 2,928 kg, la compensazione sarebbe il 4,3 % più alta.
    Vanno quindi usati la stessa massa e la stessa configurazione (con o senza Jetson).
- **Ripartizione** (soluzione ai minimi quadrati, 12 giunti; F_z LF / RF / RH / LH):

  | HOLD | Stima ai minimi quadrati | Modello a norma minima di V7 |
  |---|---|---|
  | 120 mm | 7,7 / 4,5 / 9,4 / 5,9 N | 6,1 / 6,0 / 7,7 / 7,8 N |
  | 105 mm | 7,4 / 5,2 / 8,5 / 6,5 N | 6,1 / 6,0 / 7,6 / 7,8 N |

  - La diagonale LF‑RH porta 16,6–17,1 N contro 10,6–11,7 N dell'altra (il modello: 13,8 / 13,8).
  - In appoggio a 4 piedi l'errore per gamba della compensazione è quindi dell'ordine del **20–25 %** anche con C esatta.
  - È una sola soluzione fra quelle compatibili: con φ > 0 le forze interne non sono uniche.
  - Nei treppiedi della camminata la ripartizione statica è determinata e questo errore non si applica.

## 6. Cosa manca e cosa può dare lo studio sensorless separato

Questi log non possono dare:
- **il limite d'attrito statico per giunto sotto carico** (φ_max): è l'unico parametro che chiude l'intervallo;
- la scala coppia/duty indipendente dal peso;
- l'asimmetria fra trascinare e reggere il carico (η): c'è una sola salita, non sovrapposta alle discese;
- C_HIP;
- C dinamico in camminata (a 1 Hz, con lettura di 120 ms).

Senza duplicarlo: lo studio sensorless, se misura per giunto l'attrito statico sotto carico, la coppia per unità di duty
(o di corrente) e l'asimmetria fra trascinamento e tenuta, fornisce esattamente φ_max e un κ indipendente. Con φ_max ≤ 0,15
N·m l'insieme qui sopra si ridurrebbe a un valore. La legge del regolatore (8 ‰/tick, punch 8 ‰, banda morta ±1) è già
ricavata qui e non va rifatta.

## 7. Riproduzione

Dal worktree, `H` = cartella della sessione e `O` = `c_ident/` degli artefatti:

```bash
~/MATDOG/tools/mujoco-venv/bin/python -I gait_study/vnext/c_ident_hold.py quality $H
```

```bash
~/MATDOG/tools/mujoco-venv/bin/python -I gait_study/vnext/c_ident_hold.py extract $H $O/m2807.npz base_mass_kg=1.707
```

```bash
python3 -I gait_study/vnext/c_ident_hold.py solve $O/m2807.npz
```

Varianti: `m2928` (base 1,828 kg) e `comxp5`, `comxm5`, `comyp3`, `comym3` (`com_shift_mm`). Output:
- `quality.txt`;
- `extract_*.txt`;
- `solve_*.txt`;
- `*.solve.json`.
