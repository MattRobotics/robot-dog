# MATDOG — Addendum al redesign: detector e ritorno del probe

**Data:** 2026-07-30  
**Stato:** vincolante per il branch di audit  
**Riferimento:** `MATDOG_ENDSTOP_SEQUENCE_REDESIGN_2026-07-30.md`

Questo addendum sostituisce esclusivamente l'interpretazione iniziale che rendeva `PresentCurrent >= adaptive threshold` un requisito binario per confermare ogni contatto.

## 1. Evidenza hardware incrociata

### M12 MAX validato

```text
contact tick = 3442
current = 4
adaptive threshold = 5
posizione dentro il corridoio geometrico atteso
ripetibilità = spread 1
verifica fisica operatore = PASS
```

### M13 V19 invalidato

```text
contact tick = 2405
current = 1
adaptive threshold = 5
corridoio geometrico atteso = 2496..2624
posizione 91 tick prima dell'inner limit
sequenza meccanica HIP non accettabile
```

Conclusione: la corrente raw attuale non separa da sola il contatto valido dal falso contatto e non può essere resa requisito obbligatorio finché non viene caratterizzata per joint, direzione e posa.

## 2. Classificatore corretto

### FREE_MOTION

- progresso coerente;
- velocità coerente;
- target raggiunto o ancora seguito;
- nessun hard abort.

### EARLY_STALL

- target ancora davanti;
- progresso persistente insufficiente;
- velocità bassa;
- posizione prima dell'inner acceptance limit.

Azione:

```text
stop immediato
→ backoff
→ recovery gerarchico
→ FAIL diagnostico
```

Non eseguire un secondo approccio verso lo stesso ostacolo.

### EXPECTED_CORRIDOR_STALL

- target ancora davanti;
- progresso persistente insufficiente;
- velocità bassa;
- posizione dentro `inner_limit..outer_guard`;
- telemetria fresca;
- status/readback sani;
- current sotto l'hard-abort assoluto.

Azione:

```text
stop
→ backoff verificato
→ secondo approccio fine
→ verifica corridoio e repeatability
```

### CONTACT_REPEATABLE

Accettabile soltanto quando entrambi gli approcci:

- terminano nel corridoio del modello;
- sono entro la tolleranza di ripetibilità;
- effettuano backoff reale;
- non mostrano drift delle prerequisite;
- non superano limiti di tempo, travel, status o corrente assoluta.

### CURRENT_SUPPORT

`PresentCurrent` rimane:

- valore diagnostico registrato;
- supporto positivo quando supera una baseline caratterizzata;
- segnale di hard abort quando supera il limite assoluto;
- mai unico criterio di contatto;
- non ancora requisito binario per l'accettazione.

Una futura promozione a requisito più forte richiede una campagna di caratterizzazione che conservi come validi M12 MIN e M12 MAX.

## 3. Corridoio geometrico obbligatorio

Il detector deve conoscere, prima del moto:

```text
home_tick
inner_acceptance_tick
urdf_limit_tick
outer_guard_tick
probe_direction
```

Il controllo `EARLY_STALL` deve avvenire prima della logica di repeatability.

Il caso di regressione obbligatorio è:

```text
profile = LF_HIP_M13_MIN
present = 2405
inner_acceptance = 2496
outer_guard = 2624
velocity = 0
progress = 0
current = 1
threshold = 5
expected = EARLY_STALL, never CONTACT_CONFIRMED
```

## 4. Ritorno e handoff del probing joint

La tolleranza più ampia del probe può essere usata soltanto come finestra intermedia di assestamento mentre il joint è ancora attivo e le prerequisite restano mantenute.

Prima di:

- spegnere il torque del probe;
- classificare il probe come non-active;
- iniziare il restore delle prerequisite;

il probe deve risultare dentro la normale tolleranza statica di 10 tick.

Se il primo ritorno si ferma tra 11 e 16 tick:

```text
mantieni prerequisite e torque del probe
→ esegui correzione/nudge home a bassa energia
→ richiedi telemetria fresca
→ PASS solo entro 10 tick
```

Se resta oltre 10 tick:

```text
non rilasciare LOWER/UPPER prerequisite
→ recovery gerarchico
→ global torque OFF verificato
→ FAIL
```

Regressione V19:

```text
M13 present = 2065
home = 2048
error = 17
expected = active recovery failure while prerequisites remain held
```

Non deve più emergere successivamente come errore generico `non-active M13 left home`.

## 5. Test obbligatori

1. M12 MAX `3442, current=4, threshold=5`, dentro corridoio e ripetibile: non rifiutato per la sola corrente.
2. M13 V19 `2405`, prima del corridoio: `EARLY_STALL`.
3. Corrente alta senza stallo: non contatto.
4. Stallo dentro corridoio con corrente neutra: secondo approccio consentito, non accettazione immediata.
5. Due stall ripetibili dentro corridoio: accettazione.
6. Primo ritorno a 11..16 tick: nudge richiesto prima dell'handoff.
7. Ritorno a 17 tick: recovery failure con prerequisite ancora attive.
8. Nessun restore di LOWER/UPPER prima del probe entro 10 tick.
