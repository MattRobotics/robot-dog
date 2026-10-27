# MATDOG — Cronologia firmware ESP32-S3 (V2)

Questo indice accompagna la **V2 pubblicabile**, il diario e il foglio Excel relativi al periodo **15 settembre – 26 ottobre 2026**. È una documentazione storica, non un'affermazione sul firmware attualmente installato nel robot.

## Documenti

- `MATDOG_DIARIO_SVILUPPO_ESP32S3_2026-09-15_2026-10-26_V2_PUBLIC.md`: cronologia integrale, registro di build, classificazione delle evidenze, decisioni e lacune.
- `MATDOG_Diario_Sviluppo_ESP32S3_V2_2026-09-15_2026-10-26_PUBLIC.xlsx`: 12 fogli con cronologia navigabile, 43 manifest, 18 criticità, 18 decisioni, 19 lacune, 156 citazioni e 130 SHA256 delle fonti.

Sono **edizioni di pubblicazione derivate dalla V2 originale**. Gli esiti e gli SHA256 delle APP non sono stati riscritti. I riferimenti ai percorsi fisici del computer locale sono stati sostituiti da identificativi `SRC-0001`–`SRC-0142` e nomi logici delle fonti. Gli ID sono coerenti nei due documenti; non garantiscono che il file di evidenza sia contenuto nel repository pubblico. Gli SHA256 delle fonti non disponibili nel catalogo originale restano esplicitamente `UNKNOWN`.

La **V2 originale non anonimizzata** è preservata separatamente, non va caricata nel repository pubblico. SHA256 originale: `787308caa0a8fdb921dfbc130a7d38f28f60a06874e600dc261779357c69a5e6`.

## Identità firmware al termine del periodo V2

| Ruolo documentale | Identificatore | Qualificazione storica |
|---|---|---|
| Ultima APP con flash riportato | SHA256 `54ade4fdf9c620194d15ed7990d2856ef1a971232988fecf6addca14bc18046f` | Prove fisiche parziali documentate; STAND completo, rete sotto movimento e STOP end-to-end non qualificati |
| Candidata finale RC5, 26 ottobre | SHA256 `fe33fab65672642b39f611911c5996d949ba2877727c89407e5daad438b2e2e3` | **VERIFIED OFFLINE; NOT HARDWARE VALIDATED**, nessun flash attribuito a questa APP nel perimetro V2 |
| Checkpoint sorgenti RC5, creato il 27 ottobre | Commit `c0e82234a202238521235dd240e6071011909115`, tree `6740e6b404d2ec55681582b5322b23436e0b9076` | Fotografia delle sorgenti della candidata RC5; non equivale a un nuovo build/flash |

Il checkpoint sorgente non promette la ricostruzione byte-identica del binario originale: metadati di build, SHA/dirty e timestamp sono incorporati nella APP. Le modifiche RCA successive al checkpoint non sono incluse nella V2.

## Lettura corretta

- `HW-LOG` e `HW-REPORT`: prove fisiche con differenti livelli di disponibilità della cattura.
- `OFFLINE` e `IMPLEMENTED`: codice, build o verifiche senza prova hardware corrispondente.
- `UNKNOWN`: dati non recuperati, non da integrare con ipotesi.
- Le righe storiche non vanno reinterpretate sulla base dello stato attuale di `main` o di un altro branch.

Per proseguire la cronologia oltre il 26 ottobre, creare una nuova versione o addendum con identificativo firmware, SHA256 APP, commit/tree sorgente, flag CLEAN/DIRTY, manifest, data e livello di evidenza; preservare questa V2 come fotografia del proprio periodo.
