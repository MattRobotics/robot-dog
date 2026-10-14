# Entry geometry — 14 ottobre 2026

**PASS_COMPUTATIONAL_ENTRY con tube uniforme ±0,004 rad**; nessuna autorizzazione hardware. Il tube pieno ±0,010 rad richiesto è stato completato e resta **BLOCKED**, con un controesempio LF UPPER–LOWER. Non è stato ritagliato con limiti fisici ipotetici. La posa sorgente proviene dal log reale del 13 ottobre, torque OFF; non costituisce una lettura fresca del 14 ottobre.

Bundle: `entry_geometry_bundle_20261014.json`, digest canonico **`b823f7a06923408876b31cea1eadc8a1e6fcb652e8aa4a29ffe58c2a4faf3efb`**. Conservati il certificato originale Claude `entry_certificate_prone_20261013.json` BLOCKED e i nuovi report completi 4/10 mrad. Tutte le 17 mesh collisione, URDF, route, sorgenti dei prover e versioni Python/NumPy/FCL sono identificati da SHA256.

| Prova continua | Risultato / confine |
|---|---|
| Due stadi nominali, 120 coppie non adiacenti | PASS 4 e 10 mrad, separazione richiesta 3 mm |
| ENTRY_DISENGAGE e PRIME con quattro LOWER indipendenti | PASS sul box intero `[q_original−4mrad, q_head+4mrad]` per ogni LOWER; HIP/UPPER ±4mrad; minimo certificato non adiacente **4,278 mm** |
| Dodici adiacenze, tube 4 mrad | 12/12 PASS_CONTINUOUS, nessun triangolo rimosso, nessuna eccezione di montaggio |
| Containment iniziale dei componenti chiusi | 12/12 PASS_NO_NESTED_SURFACE; il supplemento box controlla anche tutte le 120 coppie |
| Ventre: tutte le facce che toccano la fascia inferiore di 15 mm | Incluse anche le facce che attraversano il confine: 3052 triangoli, contro 2587 nel primo tool Claude |
| Ventre contro 12 link non adiacenti | Inflazione isotropa di 3 mm coperta: bound prima dell'espansione **11,045 / 11,014 mm**, richiesti 6 mm; tube 10 mrad **7,421 / 7,390 mm**, richiesti 6 mm |
| Ventre esteso soltanto verso il basso di 3 mm contro i quattro HIP adiacenti | Prismi triangolari completi, campioni FCL più bound Lipschitz: minimo continuo **0,957 mm** a 4 mrad e **0,490 mm** a 10 mrad; clearance adiacente positiva, senza imporre il margine non adiacente di 3 mm |
| Tube completo 10 mrad, LF UPPER–LOWER | BLOCKED. A q LOWER **−92,330774°** FCL rileva intersezione; la prima separazione nulla è circa **−92,070442°**. La posa storica **−91,757816°** ha superficie distante circa **0,090 mm** |

Il certificato bundle copre anche avanzamenti LOWER diversi per zampa dopo PRIME: il precedente segmento con clock comune, da solo, non avrebbe provato questa composizione. Per HIP/UPPER il runtime deve rispettare `startDeviation + tracking + mezzo passo + guardia numerica ≤4mrad`; il planner può allungare la quintica entro il timeout runtime. Policy candidata: tracking 3,5 mrad; la quantizzazione raw del comando è compresa nell'errore misurato rispetto al target non arrotondato solo finché il feedback resta nel dominio certificato. Accuratezza fisica della mappatura, encoder, gioco, flessione e mounting non sono dimostrate da questa prova.

La nuova cover deve essere misurata: il modello aggiunto è una sporgenza **verticale verso il basso ≤3 mm**, senza allargamento laterale. Il calcolo non garantisce un'espansione isotropa di tutti i montaggi HIP. Foto del robot e conferma della posa non attestano né la forma del sottoscocca né il volume libero della fixture. Restano esterni: cover reale entro l'envelope, fixture/cavi/batteria, contatti meccanici, timing e feedback reali, temperatura/tensione, STOP e power cut. Nessun carico, appoggio a terra, stabilità dinamica o movimento reale è certificato.

Verifica offline del bundle (solo lettura):

```bash
python3 09_Logs/Validation_Reports/Runtime_Commissioning_2026-10-14/geometry/verify_entry_bundle.py \
  --repo "$PWD" \
  --bundle 09_Logs/Validation_Reports/Runtime_Commissioning_2026-10-14/entry_geometry_bundle_20261014.json
```

Replay dei prover richiede `PYTHONPATH` della dipendenza FCL già presente sull'Asus e `MATDOG_PAIRED_OUTPUT` con il `core_adapter.so` recuperato da Claude. I due supplementi versionati in `geometry/` accettano `--repo`, `--joint-log` e `--output` nuovo; il tool principale `certify_commissioning_entry.py` richiede `--tube-rad 0.004` oppure `0.01`. Non sovrascrivono certificati esistenti. Il log storico resta in archivio privato, identificato dal digest; nessun dump NVS, credenziale TLS o materiale di calibrazione persistente è stato modificato o pubblicato.

Comando di caricamento RAM del bundle, da usare nella procedura autorizzata di commissioning e senza implicare torque/movimento:

```text
@MOTION COMMISSIONING ENTRY -39884 1398991 -1601476 41417 1388253 -1581534 -44485 1440408 -1595340 39884 1437340 -1595340 4000 b823f7a06923408876b31cea1eadc8a1e6fcb652e8aa4a29ffe58c2a4faf3efb
```

`ENTRY` è sola tracciabilità/configurazione RAM. Flash e fase attuativa richiedono le autorizzazioni specifiche dell'operatore; nessuna seriale o hardware è stata aperta da questo workstream.
