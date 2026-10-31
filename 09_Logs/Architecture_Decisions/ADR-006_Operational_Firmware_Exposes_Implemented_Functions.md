# ADR-006 — Il firmware di esercizio espone le funzioni implementate

Data: 2026-10-17. Stato: **Accepted — direttiva permanente per le revisioni MATDOG**.

## Direttiva

> Il firmware di esercizio deve esporre le funzionalità effettivamente
> implementate e tecnicamente ammissibili senza introdurre blocchi procedurali
> di validazione. Le procedure di qualificazione hardware devono essere esterne
> al normale flusso d'uso. I controlli runtime devono essere limitati alle
> condizioni necessarie per evitare fault hardware, attuazioni incontrollate e
> violazioni dei limiti fisici. L'interfaccia operativa deve richiedere il minor
> numero possibile di interventi manuali.

## Conseguenze vincolanti

1. **Disponibilità ≠ qualificazione.** Nessun flag di compilazione o di release
   (es. l'ex `kStandOnlyCandidate`) può spegnere una funzione implementata. La
   disponibilità è decisa a runtime dal Controller (limiti live, box certificati,
   feedback fresco, energia/BMS, collegamento). La qualificazione hardware è un
   dato riportato (`hardware_qualified` nelle capability), non un cancello.
   `static_audit.py` fallisce se un fence compile-time viene reintrodotto.
2. **Controlli runtime ammessi** (non negoziabili): limiti meccanici e hard
   limit giunto, Safe Actuator policy, readback di torque/goal, tracking/tube,
   feedback stale, BMS/power, SAFE_OFF/STOP, perdita di comunicazione (deadman
   3 s), bus exclusivity, fault. Nient'altro deve bloccare il normale uso.
3. **Comunicazione, non attenzione.** Il deadman rileva la perdita del canale
   (poll autenticato automatico della Dashboard), non chiede gesti manuali.
   Nessuna checkbox di presenza, nessun token ripetuto in condizioni normali
   (dispositivo ricordato + rinnovo automatico della sessione).
4. **Cause esplicite.** Ogni rifiuto riporta la causa effettiva (rete, auth,
   STALE, rifiuto del Controller con il motivo del firmware). Un esito non viene
   mai rietichettato da un effetto a valle (es. `INPUT_LOST_OR_EXPIRED` dopo un
   fallimento PRIME).
5. **Qualificazione esterna.** Runbook, fixture, logger e gate di prova vivono
   fuori dal flusso d'uso (USB/host), senza modalità nascoste nel firmware.
6. **Blocchi tecnici dichiarati.** Se una funzione non può essere resa sicura,
   il blocco tecnico preciso è documentato e mostrato; non viene coperto da una
   generica "policy di validazione".

## Revisione

Ogni review MATDOG verifica la patch contro i punti 1–6. Un nuovo controllo
runtime deve indicare quale fault, attuazione incontrollata o limite fisico
previene; altrimenti va rimosso.
