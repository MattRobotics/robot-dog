# Consultazione mirata XGO — confine ADR-004

Sono stati consultati soltanto i cinque documenti richiesti nel checkpoint XGO
`a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298`, in sola lettura. Repository,
commit, percorsi e SHA256 sono nel manifest di riferimenti; nessun contenuto
firmware, tabella, codice decompilato o parametro cinematico è stato importato.

Il catalogo di interfaccia identifica Lie Down e Stand Up come azioni. Non
dimostra la traiettoria interna di Stand Up. Le ricostruzioni G/G.2 distinguono
comandi, stato e fase, produzione di target cartesiani e ingresso IK; dichiarano
limiti nelle identità fisiche e nella qualificazione del timing. Si riutilizza
soltanto la separazione architetturale: intenzione → macchina a stati e feedback
→ target cartesiani → IK specifica MATDOG → target giunti semantici → safety.

L'implementazione MATDOG mantiene G4, ordine LF/RF/RH/LH, scheduler e fasi propri.
Pose, geometria, segni, q0, limiti, tempi, guadagni, servo e protocolli XGO non
entrano nei nuovi candidati. STOP, stale command, revoca e ritorno allo STAND
restano contratti originali MATDOG già coperti dalle regressioni.

ADR applicabile: `09_Logs/Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md`.
La posa prona e il suo percorso devono derivare dai dati MATDOG e dalle collision
mesh canoniche, indipendentemente da queste evidenze di interfaccia.
