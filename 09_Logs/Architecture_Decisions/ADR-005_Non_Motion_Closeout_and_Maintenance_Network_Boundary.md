# ADR-005 — Closeout non-motion e confine rete MAINTENANCE

Data: 2026-10-11. Stato: **Accepted per il closeout; LAN futura DEFERRED**.

Il [report](../Validation_Reports/MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md)
chiude soltanto la baseline non-motion. NVS VALID_ACKNOWLEDGED non è permit;
RESTORE resta NOT_IMPLEMENTED, authority NONE e motion non autorizzato.

Decisione: conservare Dashboard/admin via AP 192.168.4.1 in MAINTENANCE;
STA temporanea «Prova e salva» validata dall'operatore e COMMITTED non rende
la Dashboard disponibile in LAN. Non usare RUN come scorciatoia LAN.

L'analisi locale `lan-maintenance-feasibility-b08322d/feasibility.md` rinvia
l'implementazione: occorre coordinare policy radio e AP recovery, barriere di
chiusura dei listener, ammissione LAN read-only server-side e route allowlist,
sessioni UI/admin AP e segregazione OTA. ADMIN_AP attuale richiede AP esclusivo;
abilitare soltanto STA o /status non crea una Dashboard LAN sicura.
OTA_INGEST=0 non sostituisce l'esclusione delle route OTA dalla futura LAN.

Target futuro: consultazione LAN read-only, amministrazione AP, OTA separata;
nessun endpoint di rete deve ottenere authority attuatori. Progetto, review e
prove isolamento/recovery richiesti prima della nuova funzione. Nessuna patch
LAN introdotta con questo ADR. Residui e handoff restano nei rispettivi documenti,
senza una seconda matrice di validazione.

## Implementazione autorizzata 2026-10-13, PR #47

La decisione storica sopra resta riferita al closeout b08322d. La richiesta esplicita del proprietario autorizza ora la LAN MAINTENANCE read-only: un ruolo RAM separato, cinque GET ammessi server-side, nessun login/config/OTA/attuatore, listener vincolato a socket locale e generazione radio. ADMIN_AP WPA2 esclusivo su 192.168.4.1; RUN_STA TLS/autenticato separato, senza grant Motion dalla Web UI. Transizioni revocano listener/sessioni e permit, recovery AP e STA_STOP preservati; nessun AP+STA introdotto. Stato hardware ancora non verificato. Fonti e prove: [report finale](../Validation_Reports/Final_Release_Candidate_2026-10-13/README.md).
