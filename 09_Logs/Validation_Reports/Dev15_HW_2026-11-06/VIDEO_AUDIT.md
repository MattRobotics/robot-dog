# MATDOG dev.15 — audit cinematica video joystick (06/11/2026)

**Provenienza:** `1000083544.mp4` (SHA-256 `0b14b26dab5e5daeda079ec58a2274ec2dd59d984698eb1b61ecaa00e95c84bb`). Il file originale e l'analisi per-frame sono elencati nel [manifest](EVIDENCE_MANIFEST.md) e **non pubblicati in questo branch**. Questa pagina riporta il risultato verificato del precedente audit visivo, non afferma di aver ricostruito nuovi eventi non inclusi nel filmato.

**File:** H.264, 478×850 px, 16.620 fotogrammi decodificati, durata 278,913 s (ultimo PTS 278,897 s), rate nominale 60 fps, media ~59,59 fps. Analisi PTS anziché assumere frame/60; marker luminoso verde/ciano sul dorso tracciato in tutti i frame, differenze visuali e controlli di fotogrammi chiave. Stima dei confini delle azioni **±1–3 s**.

## Timeline osservata (tempo dall'inizio del video)

| Intervallo | Osservazione diretta |
|---|---|
| 00:00–00:03 | Postura eretta, quasi ferma, frontale |
| 00:03–00:32 | Avanzamento verso la camera, marker y circa 465→509 px |
| 00:32–01:00 | Arretramento, marker y circa 509→446 px |
| 01:00–01:14 | Piccoli assestamenti/transizione, deriva netta modesta |
| 01:14–01:49 | Strafe verso destra **dello schermo**, marker x ~280→335 px |
| 01:49–02:15 | Strafe inverso/termine movimento, forte oscillazione residua |
| 02:15–02:42 | Rotazione verso assetto obliquo, muso verso destra schermo |
| 02:42–03:00 | Assetto obliquo e ulteriori movimenti; non distinguibile precisamente fase comando/settle |
| 03:00–03:18 | Rotazione opposta, ritorno vista frontale |
| 03:18–03:36 | Riposizionamento/passi contenuti |
| 03:36–04:12 | Diagonale avanti + lato, marker ~308→235 px x e ~438→490 px y |
| 04:12–04:38,9 | Postura eretta sostanzialmente immobile |

Destra e sinistra **dello schermo** non sono quelle nel riferimento del robot, rivolto alla camera.

## Metriche visive e loro limiti

| Finestra | Periodicità laterale | Escursione orizzontale del marker (immagine) |
|---|---|---|
| Avanti 00:05–00:32 | ~6–7 s | ~29,7 px |
| Indietro 00:33–01:01 | ~6–7 s | ~29,7 px |
| Strafe 01:15–01:42 | 6,53 s | ~29,0 px |
| Yaw 02:13–02:40 | 6,48 s | ~31,8 px |
| Diagonale 03:35–04:10 | 6,51 s | ~31,3 px |

**VERIFIED VIDEO:** oscillazione laterale regolare del dorso di periodo dominante ~6,5 s (0,154 Hz; 9,2 oscillazioni/min) con traslazione media lenta; transitorio iniziale compatibile con spostamento/precarico in verso opposto alla progressione. Nessuna caduta visibile nella registrazione. Non è una misura indipendente della velocità a terra, baricentro, forza o periodo interno del gait: i pixel non sono mm, il marker non è il CoM, la camera non è calibrata. Non prova che sia sicuro ridurre lo sway: occorre verifica di appoggi/support polygon, margini IK e tracking.

**VERIFIED LOG separato (traccia 20):** ultima sequenza di joystick combinato avanti 5,5 mm/s + destra 1,9 mm/s, poi avanti 4,3 mm/s + destra 2,4 mm/s; ultimo rilascio joystick `STAND_REACHED` dopo **8,378 s**; successivo STOP `STOPPED_SUPPORT_UNVERIFIED` dopo **1,935 s** con coppia ON; 345 nuove release perse nell'intervallo registrato, massimo lavoro 47,618 ms contro 20 ms nominali. Le ultime 48 richieste del log (20) hanno HTTP 200; la finestra non copre i 279 s del video e **non esiste una sincronizzazione verificata** con il momento di pressione del joystick.

**Non documentato dal video:** il terzo PRONE fallito riferito dall'operatore dopo la dimostrazione: il filmato termina in STAND. Quella prova è registrata come osservazione operatore nel [report hardware](REPORT.md), non come evidenza video.

## Implicazioni per il prossimo sviluppo

La lentezza osservata dipende almeno dai limiti nominali di comando e dalla cadenza del gait; il timing Motion va diagnosticato separatamente con telemetria per-release. Ridurre i tempi di settle e il movimento laterale superfluo senza compromissioni dei veri supporti è un obiettivo, non una patch ancora qualificata. Un incremento di velocità fino a 5× è un **target**, non una prestazione già dimostrata.

L'audit originale dettagliato `MATDOG_DEV15_VIDEO_AUDIT_2026-11-06.md`, il CSV con 16.620 righe, i keyframe e il grafico hanno SHA manifestati separatamente; non sostituire i file originali con questo sunto.
