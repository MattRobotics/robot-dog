# MATDOG — Redesign della sequenza di calibrazione dei finecorsa

**Data:** 2026-07-30  
**Stato:** DESIGN + OFFLINE GEOMETRY AUDIT REQUIRED  
**Hardware:** BLOCCATO  
**Ambito:** sostituisce l'ordine `UPPER → HIP → LOWER` per tutte le future sessioni.

## 1. Motivo della correzione

La prova hardware `LF_HIP_M13_MIN` V19 ha usato la prerequisite statica:

```text
rear parking: M42 ≈ +30°
active upper: M12 ≈ +50°
active lower: M11 = 0°
```

Durante la rotazione HIP la catena upper/lower è rimasta estesa. L'operatore ha osservato un rischio reale di interferenza con il `base_link` e con la base esterna che sostiene MATDOG.

La stessa prova ha prodotto due falsi contatti M13 a 2405 tick:

```text
current=1
adaptive threshold=5
accepted model corridor=2496..2624
```

Questi valori sono classificati `INVALID_EARLY_STALL`, non limiti meccanici.

## 2. Decisione canonica

L'ordine per ogni zampa diventa obbligatoriamente:

```text
UPPER MIN
→ UPPER MAX
→ UPPER home
→ UPPER horizontal pose
→ LOWER MIN
→ LOWER MAX
→ LOWER home
→ LOWER compact/folded pose derived from measured LOWER travel
→ HIP MIN
→ HIP MAX
→ complete reverse-order recovery
```

Ordine delle zampe invariato:

```text
LF → RF → RH → LH
```

Per LF e RF resta obbligatorio il parcheggio della upper posteriore ipsilaterale prima di qualunque movimento della zampa anteriore.

## 3. Vincolo di dipendenza tra fasi

Un profilo HIP non può più essere armato isolatamente.

Per la stessa zampa devono esistere, nella sessione corrente o in un checkpoint verificato:

```text
UPPER_MIN = PASS
UPPER_MAX = PASS
LOWER_MIN = PASS
LOWER_MAX = PASS
```

La prerequisite HIP deve essere derivata dai risultati LOWER reali e non da una tabella statica comune.

Sono ammesse due implementazioni:

1. sessione nativa completa per zampa, con sei contatti in memoria;
2. manifest persistente firmato/hashato e verificato dal runner e dal calibratore.

La soluzione preferita è una sessione completa per zampa perché impedisce mix di risultati, profili o versioni software.

## 4. Pose geometriche

### 4.1 UPPER contact search

```text
hip = 0°
lower = 0°
probe = upper
```

Per le zampe anteriori, la posteriore ipsilaterale resta parcheggiata.

### 4.2 LOWER contact search

```text
hip = 0°
upper = horizontal-to-ground pose derived from URDF
probe = lower
```

La posa upper non deve essere codificata soltanto come `+90°`: l'audit deve verificare dal vero URDF che l'asse longitudinale del link sia orizzontale nel frame `base_link`.

### 4.3 HIP contact search

```text
upper = horizontal-to-ground pose
lower = compact/folded pose near the upper link
probe = hip
```

La posa lower compatta viene scelta soltanto dopo LOWER MIN/MAX. Deve:

- essere dentro il travel meccanico misurato;
- avere margine dal contatto reale;
- rendere upper e lower quasi paralleli e compatti;
- mantenere la catena lontana da `base_link`, altre zampe e fixture esterna;
- passare l'intera traiettoria HIP MIN↔MAX, non solo le pose terminali.

Il target candidato iniziale è vicino a `q_lower=-90°`, ma il valore finale deve essere calcolato dal modello e dai contatti misurati. Non è autorizzato un numero fisso prima dell'audit.

## 5. Base di sostegno esterna

La base che mantiene MATDOG sospeso non fa parte dell'URDF. Di conseguenza il precedente self-collision audit non può garantire la sua esclusione.

Prima dell'HIP hardware è obbligatorio un file di configurazione della fixture, espresso nel frame `base_link`, con almeno uno o più volumi AABB:

```yaml
schema_version: 1
frame: base_link
keepout_boxes:
  - name: central_support
    min_xyz_m: [x_min, y_min, z_min]
    max_xyz_m: [x_max, y_max, z_max]
safety_margin_m: 0.010
```

L'audit deve fallire se il file manca, contiene placeholder o se una collision mesh entra nel volume dilatato del margine di sicurezza.

## 6. Detector di contatto corretto

La conferma deve richiedere contemporaneamente:

```text
enough travel
AND low progress
AND low velocity
AND target still ahead
AND current >= adaptive threshold
AND position inside inner acceptance corridor
AND fresh telemetry
AND healthy status/readback
```

Classificazione obbligatoria:

- `FREE_MOTION`: tracking normale;
- `CONTACT_SUSPECTED`: evidenze parziali;
- `CONTACT_CONFIRMED`: tutte le evidenze concordano;
- `EARLY_STALL`: stallo prima del corridoio interno;
- `KINEMATIC_STALL_WITHOUT_CURRENT`: stallo senza supporto di corrente;
- `HARD_ABORT`: current limit, status, stale telemetry, guard o drift prerequisite.

`EARLY_STALL` e `KINEMATIC_STALL_WITHOUT_CURRENT` non vengono ripetuti come contatti: causano stop, backoff, recovery e FAIL diagnostico.

## 7. Corridoio di accettazione

Ogni lato deve avere:

```text
inner_limit = URDF limit - model tolerance toward home
outer_guard = URDF limit + 64 ticks toward the contact direction
```

Un contatto prima dell'inner limit rappresenta un ostacolo inatteso, attrito, fixture o collisione diversa dal finecorsa progettato.

La posizione V19 M13=2405 è fuori dal corridoio 2496..2624 e deve essere rifiutata al primo approccio.

## 8. Transizioni da verificare offline

Per ogni zampa devono essere campionate tutte le transizioni:

1. home → UPPER MIN → backoff → repeat → home;
2. home → UPPER MAX → backoff → repeat → home;
3. upper home → upper horizontal;
4. LOWER MIN → backoff → repeat → home;
5. LOWER MAX → backoff → repeat → home;
6. lower home → lower compact/folded;
7. HIP MIN → backoff → repeat → zero;
8. HIP MAX → backoff → repeat → zero;
9. lower compact → lower home;
10. upper horizontal → upper home;
11. rear parking → rear home, per LF/RF.

Campionamento massimo iniziale:

```text
0.5° per le transizioni prerequisite
1° per le contact sweeps
```

Il gate finale deve includere:

- joint limits;
- exact collision meshes;
- non-adjacent self-collision;
- body collision;
- cross-leg collision;
- fixture keep-out;
- continuità della traiettoria.

## 9. Recovery gerarchico

Il recovery non usa una sequenza unica per tutti i fallimenti.

### Fallimento UPPER

```text
probe stop → backoff → upper home → rear parking home → global torque OFF
```

### Fallimento LOWER

```text
probe stop → backoff → lower home → upper home → rear parking home → global torque OFF
```

### Fallimento HIP

```text
probe stop → backoff → hip zero → lower home → upper home → rear parking home → global torque OFF
```

Ogni movimento di recovery deve conservare la prerequisite che rende sicuro il movimento corrente. Non si rilascia una prerequisite prima che il giunto dipendente sia tornato nella posa sicura.

## 10. Contratti invariati

- Station unico proprietario della seriale;
- `GOAL_POSITION` unsigned 0..4095;
- signed-wrap vietato;
- sole scritture RAM consentite;
- niente EEPROM/reset/offset/lock/action/freeze;
- un solo probing joint in movimento verso il contatto per fase;
- stop/backoff/repeatability obbligatori;
- global torque OFF verificato su successo e fallimento;
- nessun merge automatico.

## 11. Gate prima del prossimo hardware

Il prossimo hardware è bloccato finché non risultano PASS:

1. audit URDF della nuova sequenza;
2. fixture keep-out configurata e verificata;
3. test esaustivi della phase dependency;
4. test detector con corrente realmente obbligatoria;
5. test early-stall M13=2405/current=1;
6. test recovery da ogni fase e da ogni write/readback failure;
7. suite ST3215 completa;
8. build Station senza warning;
9. replay deterministico della sessione LF usando M12 MIN/MAX già congelati;
10. runner che impedisce qualunque arm HIP isolato.

## 12. Stato dei dati hardware

Conservare:

```text
LF_UPPER_M12_MIN = 1443 / 1443, spread 0
LF_UPPER_M12_MAX = 3443 / 3442, spread 1
```

Invalidare:

```text
LF_HIP_M13_MIN = 2405 / 2405
reason = EARLY_STALL + NO_CURRENT_SUPPORT + WRONG_MECHANICAL_SEQUENCE
```

Nessun valore M13 deve essere scritto nei limiti misurati o nei checkpoint canonici.