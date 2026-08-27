# Prompt operativo per Claude — MATDOG RF calibrator

Sei incaricato di riprendere il calibratore meccanico RF di MATDOG nel repository:

```text
MattRobotics/norma-core
```

Devi considerare questo incarico un passaggio di consegne dopo una lunga serie di tentativi falliti. Non devi fidarti dei riepiloghi verbali: GitHub remoto è la sorgente di verità.

## 1. Prima di modificare qualsiasi cosa

Verifica direttamente:

```text
main
release/matdog-lf-calibrator-v25
PR #18 chiusa
commit sperimentale archiviato 9482086baba6fac0c266c3dc509352b6547d0365
```

Leggi integralmente:

```text
tools/matdog/README.md
tools/matdog/MATDOG_RF_CALIBRATOR_CLAUDE_HANDOFF_2026-08-05.md
```

Leggi poi il sorgente esatto della release immutabile:

```text
release/matdog-lf-calibrator-v25
SHA f87dd1fbc7e8100d275c74f9af448642f3429680
```

Confronta riga per riga i file LF V25 con il vecchio head RF `9482086...`. Il vecchio head è solo un archivio di errori ed esperimenti: non va mergiato e non va usato come nuova base.

Prima di scrivere codice, produci una tabella con quattro categorie:

```text
1. comportamento LF V25 da lasciare identico;
2. dati RF da sostituire;
3. correzione generica realmente necessaria e provata;
4. esperimento precedente da scartare.
```

Non procedere con modifiche finché questa classificazione non è completa.

## 2. Regole repository

Alla partenza devono esistere soltanto:

```text
main
release/matdog-lf-calibrator-v25
```

Crea esattamente una sola branch di sviluppo:

```text
matdog/rf-calibrator-from-lf-v25
```

Non creare branch temporanee, dispatcher, versioni Vxx, branch CI o branch di pulizia.

Non creare workflow one-shot o autoeliminanti. Usa soltanto i workflow durevoli già presenti su `main`.

Non fare merge in `main` senza revisione umana esplicita. Non modificare mai la release LF V25.

## 3. Obiettivo tecnico

Produrre il calibratore RF completo copiando il comportamento hardware-validato di LF V25 e sostituendo soltanto:

- mapping motori RF;
- direzioni encoder RF;
- parking RH necessario alla zampa anteriore RF;
- geometria/prerequisiti RF derivati dal mapping e dall’URDF;
- gestione della traslazione encoder RF tramite contatti misurati.

Non devi inventare un nuovo calibratore e non devi rifattorizzare LF per eleganza.

## 4. Baseline LF V25 vincolante

Contatti finali LF:

```text
M13 HIP   MIN 2535  MAX 1600  q0 affine 2067
M12 UPPER MIN 1439  MAX 3443  q0 affine 2040
M11 LOWER MIN 3093  MAX 1658  q0 affine 2074
```

Sequenza LF V25 persistente:

```text
preflight una volta
initial recovery
parking una volta
UPPER primo estremo -> secondo estremo
UPPER in posa prerequisite
LOWER primo estremo -> secondo estremo
LOWER in posa folded/parallela
HIP primo estremo -> secondo estremo
diagnostica affine/witness
ritorno HIP
ritorno LOWER mantenuta
ritorno UPPER
restore parking
cleanup
global torque OFF
```

Ogni estremo usa:

```text
coarse scout scartato
backoff
fine #1
backoff
fine #2
repeatability fine-to-fine
```

Non modificare guard, hard current, speed, torque, acceleration, temperatura, repeatability, q0 gate o timeout senza prima dimostrare che LF V25 usa già la stessa regola.

## 5. Mapping e direzioni

```text
LF: M13 HIP -1 | M12 UPPER +1 | M11 LOWER -1
RF: M23 HIP -1 | M22 UPPER -1 | M21 LOWER +1
RH: M33 HIP +1 | M32 UPPER -1 | M31 LOWER +1
LH: M43 HIP +1 | M42 UPPER +1 | M41 LOWER -1
```

RF usa:

```text
M23 HIP
M22 UPPER
M21 LOWER
M32 RH UPPER parking
```

Topologia obbligatoria:

```text
11 12 13 21 22 23 31 32 33 41 42 43
```

## 6. Sequenza fisica RF richiesta

La replica deve essere fisica, non soltanto nominale.

Prima degli HIP:

```text
M32 parking circa 1707
M22 prerequisite V25 circa 1024
M21 prerequisite V25 circa 1058
```

La lower deve assumere la stessa relazione folded/parallela osservata nella LF V25. Non sostituire questi target con `1044/1032`: quel tentativo è stato rifiutato.

HIP RF:

```text
primo movimento fisico: verso il basso, fino al primo vero finecorsa;
secondo movimento fisico: verso l’alto, fino al vero finecorsa opposto.
```

Con M23 direction `-1`, il vecchio esperimento otteneva il movimento verso il basso diminuendo i tick e quello verso l’alto aumentandoli. Verifica con l’URDF quale lato è realmente MIN e quale MAX e rendi coerenti log, profili e diagnostica. Non affidarti ai nomi dei vecchi stati.

## 7. Witness RF corretto

Non usare endpoint RF assoluti specchiati intorno a 2048.

Sono vietati come coordinate obbligatorie:

```text
UPPER 2653 / 654
LOWER 1003 / 2430
HIP   2479 / 1561
```

Il corretto witness è relativo:

```text
primo contatto RF misurato realmente
+
span meccanico LF V25 dello stesso joint
+
seconda direzione di probe
=
regione prevista del secondo contatto RF
```

Procedura:

1. misura il primo contatto RF con detector/corridoio/guard V25;
2. usa quel contatto come ancora encoder RF;
3. ricava lo span LF V25 dal sorgente/dati immutabili;
4. predici l’ingresso della regione del secondo contatto;
5. rifiuta plateau HOME-side prima di quell’ingresso;
6. esegui coarse, backoff, fine #1, backoff, fine #2;
7. confronta lo span RF misurato con lo span LF usando la tolleranza witness già congelata in LF V25;
8. deriva q0 e scale dalla coppia RF misurata;
9. non forzare q0=2048 e non centrare artificialmente gli endpoint.

Il vecchio head `9482086...` contiene idee utili con nomi simili a:

```text
lf_v25_reference_span_ticks
rf_relative_second_contact_entry_tick
configure_rf_relative_second_contact_entry
rf_span_witness_deviation
```

Non copiarle alla cieca. Confrontale con la release e reimplementa soltanto ciò che rispetta il contratto.

La tolleranza non va scelta per far passare un trace. Usa esattamente la tolleranza witness LF V25. Il tentativo incompleto stava passando da 16 a 24 tick perché il trace UPPER aveva deviazione 17; devi dimostrare dal sorgente LF che 24 è il contratto giusto, non dedurlo dal fallimento.

## 8. Errori precedenti da non ripetere

Vietato:

- `Leg::Rf => true` nel witness;
- endpoint assoluti RF intorno a 2048;
- target prerequisite affini `1044/1032` introdotti senza replica LF;
- allargare il q0 gate da 10 per accettare un errore di 11 tick;
- accettare HIP intorno a 2467 senza aver raggiunto lo span LF;
- confondere URDF MIN/MAX con direzione fisica;
- modificare un solo motore con eccezioni dedicate;
- creare nuovi workflow per applicare patch;
- produrre pacchetti con nomi riciclati;
- certificare hardware da soli test offline.

## 9. Test obbligatori

Devi aggiungere test comportamentali che riproducano:

- M32 near-HOME valido e caso fuori banda;
- ultimo step M22 che termina sul guard esistente;
- chamfer M21 dentro la banda V25 e caso un tick fuori;
- primo HIP RF verso il basso;
- secondo HIP RF verso l’alto;
- stessa posa prerequisite per entrambi gli HIP;
- rifiuto del plateau HIP anticipato circa 2467;
- accettazione di coppie RF traslate ma con span LF corretto;
- rifiuto di coppie con q0 affine plausibile ma span errato;
- limite witness esatto e rifiuto a +1 tick;
- LF V25 invariata;
- temperatura V25 invariata;
- cleanup globale torque OFF per ogni failure;
- assenza EEPROM e register writes dal runner.

I test devono chiamare logica reale. Test che cercano soltanto stringhe nel sorgente non sono sufficienti.

## 10. Gate software

```text
rustfmt --check
RUSTFLAGS='-D warnings' cargo test --package st3215 --all-targets
LF self-test
RF self-test
Python runner/observer/launcher tests
Station release build warning-free
package self-test
git diff --check
```

Usa i workflow durevoli:

```text
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-native-observer-check.yml
```

Non aggiungere workflow usa-e-getta.

## 11. Revisione umana obbligatoria

Prima del pacchetto hardware, mostra a Matteo:

1. diff completo contro `main`;
2. confronto della logica condivisa contro LF V25;
3. tabella di sole sostituzioni dati RF;
4. prova matematica della posa M22/M21 durante HIP;
5. prova della direzione fisica M23;
6. prova che il plateau anticipato non può passare;
7. elenco di tutti i parametri di sicurezza rimasti invariati.

Aspetta approvazione esplicita prima di generare il pacchetto.

## 12. Pacchetto hardware

Il pacchetto deve:

- contenere Station precompilata warning-free;
- essere pinning a un singolo SHA;
- avere nome univoco con short SHA;
- includere manifest e SHA256;
- verificare hash Station/runner/observer/launcher;
- rifiutare HEAD differente;
- avviare una sola Station;
- restare RAM-only;
- produrre log completi;
- verificare global torque OFF e shutdown;
- non promettere PASS hardware.

## 13. Pulizia permanente

Durante tutto il lavoro:

- una sola branch di sviluppo;
- una sola PR draft;
- nessun workflow temporaneo;
- nessun run inutile lasciato attivo;
- niente branch residui dopo chiusura/merge;
- nessun merge senza approvazione;
- release LF V25 intatta.

## 14. Contratto di comunicazione

Devi operare con precisione e trasparenza:

- GitHub remoto prima di ogni affermazione sullo stato;
- niente modifiche architetturali autonome;
- niente ottimizzazioni non richieste;
- niente claim di certificazione senza evidenza;
- progress update durante operazioni lunghe;
- stop immediato quando l’utente dice stop;
- spiegazione chiara di errore, causa, modifica e prova.

Obiettivo finale:

```text
replicare LF V25 sulla geometria RF
con mapping, direzioni, parking e traslazione encoder RF corretti
senza cambiare il contratto di sicurezza validato
```
