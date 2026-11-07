# Dev.15 — manifest delle evidenze grezze locali

Sessione hardware 06/11/2026. **Manifest informativo**: SHA-256 calcolati sui file originali presenti nell'ambiente della conversazione. **Stato aggiornato al 07/11:** i 15 JSON Motion sono stati pubblicati nell'archivio `raw/` con checksum verificato (vedi sotto); video e derivati restano locali. Il manifesto non prova da solo che gli originali siano presenti sul filesystem ASUS.

Il repository GitHub è pubblico: evitare di pubblicare indiscriminatamente video integrali domestici, eventuali voci/persone, credenziali e log con informazioni sensibili. Il caricamento futuro delle evidenze grezze richiede selezione e verifica dei contenuti. I report testuali aggregati sono presenti in questo branch.

## Acquisizioni Motion JSON (originali, ~2,1–2,5 MB ciascuna)

| File locale | Dimensione (byte) | SHA-256 |
|---|---:|---|
| `matdog-motion-trace (8).json` | 2170348 | `145d72f59b25a70b5b691da5286ec5f2a94fc634686497f95f6e4d6958c182a9` |
| `matdog-motion-trace (9).json` | 2220694 | `e7236b2ca536416179c56fed8b94e04155ada590193581b5f99ce4b91932b57b` |
| `matdog-motion-trace (10).json` | 2335601 | `cec030c253b07018bd3909b9f1b21d4a22c928cd6b7a4d5fe855f4ff7e494992` |
| `matdog-motion-trace (11).json` | 2380032 | `b832d9578a64b6b784bfce6d31839612c53c8e8bed28b282f657dd8a0a6b350b` |
| `matdog-motion-trace (12).json` | 2452030 | `518d38b963ef04733ad2da305d90537de0e646410ae4c28fede205cf88f091dc` |
| `matdog-motion-trace (13).json` | 2453926 | `7df483eb30fc81ba36c6bd47f7d4cb792446f970a942fc684e041cc61323e8f9` |
| `matdog-motion-trace (14).json` | 2454007 | `717ef29b182e5dd22c7da2048d452692241f71605715b4e01831df3ce545efd3` |
| `matdog-motion-trace (15).json` | 2425126 | `3b4546d926ecd3dc2c31403ddd1e20b492ddd55dadb4566080d41459f2c7ce19` |
| `matdog-motion-trace (16).json` | 2378925 | `609d05d493141a230b6f307247267669be7cda9657224dd4cf4351bae61f0ecf` |
| `matdog-motion-trace (17).json` | 2334376 | `a9e5b70852e97f9ec9ee377fdf8076f60faff41f436c629e7843d026962d6471` |
| `matdog-motion-trace (18).json` | 2378374 | `0559400a0675418967353f09eabe61860dd3e3ea8ecafca730fc2ca8149950e0` |
| `matdog-motion-trace (19).json` | 2409544 | `f5aeed3217ce442c3f2052190146665b5a86361c7d719931f51bc47cbbb95377` |
| `matdog-motion-trace (20).json` | 2454936 | `bc68789d9b4d30e31faf7b6fa50428bb69a58f28e11f607fa736085553e2dc18` |
| `matdog-motion-trace (21).json` | 2426732 | `9d47353f7319261ce32b64f094d19be349e6b835d6e06ee4527d261cbbc25614` |
| `matdog-motion-trace (22).json` | 2403337 | `561228f8c869fe845b81bf208c3b6515b6f869d757719b3f65e65c18e028adfb` |

**Corrispondenza sessioni/reboot:**
- (8)–(16): unica sessione dashboard iniziata `2026-11-06T17:06:55.333Z`; catture successive con finestre sovrapponibili;
- (17)–(18): nuova sessione `2026-11-06T17:46:35.607Z`;
- (19)–(22): sessione iniziata `2026-11-06T17:51:55.745Z`.
Ogni file conserva tipicamente gli ultimi 240 snapshot (~84–90 s) e gli ultimi 48 comandi; **non** rappresenta l'intera sessione e i delta da log sovrapposti **non** sono sommabili indiscriminatamente. L'ultimo fallimento PRONE riferito dopo la sessione video non è contenuto in un JSON di queste acquisizioni.

**Pubblicazione 07/11/2026:** i 15 JSON sopra, invariati, sono pubblicati nell'archivio [`raw/MATDOG_DEV15_TIMING_RCA_INPUTS_2026-11-07.zip`](raw/MATDOG_DEV15_TIMING_RCA_INPUTS_2026-11-07.zip): 1.379.942 byte, SHA256 `d0e52dfdb9d8a0a40df22d97def5d45c8dc824f74235d90c1db0b662678a6dc5`, ZIP integro. Contiene anche `SHA256SUMS`, `README_RCA.md` e `reference/VIDEO_AUDIT.md` (testo, SHA `d555d1f4…98cb`, uguale all'audit originale sotto). Ogni campo stringa è stato rivisto: nessuna credenziale, IP privato, cookie, token o dato personale; solo telemetria, nomi giunto, etichette comando e ID casuali di istanza browser. **Nessun video** nell'archivio.

## Video e analisi derivata

| File locale | Dimensione (byte) | SHA-256 |
|---|---:|---|
| `1000083544.mp4` (filmato originale, 278,913 s, 16.620 frame) | 47660200 | `0b14b26dab5e5daeda079ec58a2274ec2dd59d984698eb1b61ecaa00e95c84bb` |
| `MATDOG_DEV15_VIDEO_AUDIT_2026-11-06.md` | 12016 | `d555d1f4a55f0ca4673c846b5002d979289961942a5742f2862dd396577798cb` |
| `MATDOG_DEV15_VIDEO_PER_FRAME_16620.csv` | 1642333 | `588b38b9039e1d3333aa9f1661e935d0b237a21e8e8a3672c2ef00c5e79a3884` |
| `MATDOG_DEV15_VIDEO_TRAJECTORY.png` | 307339 | `509c6cac463b7f781a9af1a8b2edceca0b8c990804a174932aeac622db4be5cf` |
| `MATDOG_DEV15_VIDEO_KEYFRAMES.jpg` | 427068 | `95c84f344cf37e8d50b0e512bdb749ea4726624239d8fb4ccafa8e47b9db1859` |
| `MATDOG_DEV15_VIDEO_EVIDENZE_2026-11-06.zip` | 1271955 | `23dc13c25e730b6ef1ebef461ac50227be6e8727428c148c16eae4a4a2a4270b` |

L'analisi ricostruita nel repository è [VIDEO_AUDIT.md](VIDEO_AUDIT.md); non equivale al file originale sopra e non ne condivide lo SHA. CSV per-frame e video sono conservati localmente, non sincronizzati da questa PR.

## Archivio integrale di consegna (conversazione, non GitHub)

È stato creato e verificato un archivio ZIP unico di **51.647.794 byte**, 22 elementi (15 JSON grezzi, video originale, sei derivati/manifest), con test di integrità ZIP superato:

- Nome: `MATDOG_DEV15_HW_EVIDENZE_COMPLETE_2026-11-06.zip`
- SHA256: `32e280e3a83c02bb8d86c80c917f4394d73e872a51affc76ae4231c613d992aa`
- Collocazione: allegato scaricabile dalla conversazione ChatGPT, **non presente nel repository e non attestato sul filesystem ASUS**.

L'operatore può conservarlo come pacchetto grezzo prima della successiva integrazione dal computer ASUS. Contiene un filmato domestico: non spostarlo in un repository pubblico senza revisione privacy.

## Firmware: stato aggiornato al 07/11/2026

Il sorgente esatto **è ora pubblicato su questo branch** (PR #64, draft) tramite il merge `e5b0644` con secondo genitore `7997ad4`; il tree firmware è identico a `7997ad4` (verifica blob per blob in [TIMING_RCA_2026-11-07.md](TIMING_RCA_2026-11-07.md) §1). APP, ELF, manifest e `sdkconfig` della build installata restano **solo locali** sull'ASUS (`build-output/dev15-joystick-motion-20261105/hostlink-build-7997ad4/artifacts/ROBOT_POWERED-ota0/`, `SHA256SUMS` verificato): nessuna release firmware è stata creata. La sezione seguente è conservata come stato al 06/11.

## Firmware da pubblicare in fase successiva (stato al 06/11, superato)

- Sorgente locale `7997ad4`; worktree `~/MATDOG/worktrees/matdog-dev15-joystick-motion-20261105`.
- APP flashata e verificata su app0: 1.691.328 byte, SHA256 `b81d3a4ffaa27a7690b196e77e7174c0cc7851563fcfd64cd3bddb40e62f9e13`; profilo `ROBOT_POWERED`.
- Il branch **documentale** non incorpora quel sorgente, binario o manifest eseguibile. Per la pubblicazione servono confronto del commit locale, stato dei worktree, esclusione di file sensibili, test e PR firmware separata o estensione esplicitamente autorizzata; non assumere che main corrisponda all'hardware.

## Lettura delle prove

- `VERIFIED HARDWARE` = evento misurato/riportato durante prove reali, nel loro perimetro.
- `VERIFIED LOG` = presente nel JSON; il log non mostra da solo ogni dettaglio meccanico.
- `VERIFIED VIDEO` = evento osservato nei fotogrammi; nessun timestamp dei comandi.
- `HYPOTHESIS/UNKNOWN` = spiegazione non ancora verificata.
