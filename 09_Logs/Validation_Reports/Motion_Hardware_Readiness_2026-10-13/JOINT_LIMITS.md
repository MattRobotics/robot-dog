# Matrice di review dei 12 limiti — nessuna approvazione

Fonte comune: Full Calibration 1 ottobre (identità/direction) e audit 6 ottobre gen-1 (q0/contatti aggiornati). Non sono misure attuali. Proposta: intersezione URDF con contatti gen-1, inset candidato 16 tick + spread fine; approvazione 0/12. Un tick = 2π/4096 = 0,087890625°.

| Joint | ID/unit | dir | Contatti gen-1 ° | Proposta ° | Range route ° | Margine proposta ° | Min margine URDF ° | Stato |
|---|---|---:|---|---|---|---:|---:|---|
| lf_hip_joint | 13/M22 | -1 | -46.054688 / 45.527344 | -44.472656 / 43.945312 | -1.000000 / -0.000000 | 43.472656 | 44.000000 | CANDIDATE_REVIEW_REQUIRED |
| lf_upper_leg_joint | 12/ELR01 | +1 | -56.337891 / 121.464844 | -52.500000 / 119.882812 | 23.484152 / 86.821178 | 33.061635 | 35.678822 | CANDIDATE_REVIEW_REQUIRED |
| lf_lower_leg_joint | 11/M33 | -1 | -92.021484 / 34.277344 | -90.527344 / 32.783203 | -92.000000 / 20.029055 | -1.472656 | 0.000000 | ROUTE_INCOMPATIBLE_WITH_PROPOSAL |
| rf_hip_joint | 23/NEW01 | -1 | -45.395508 / 46.801758 | -43.857422 / 45.000000 | 0.000000 / 1.000000 | 43.857422 | 44.000000 | CANDIDATE_REVIEW_REQUIRED |
| rf_upper_leg_joint | 22/ELR03 | -1 | -55.546875 / 120.805664 | -52.500000 / 119.135742 | 23.484152 / 86.821178 | 32.314564 | 35.678822 | CANDIDATE_REVIEW_REQUIRED |
| rf_lower_leg_joint | 21/NEW03 | +1 | -90.922852 / 33.750000 | -89.384766 / 32.211914 | -92.000000 / 20.029055 | -2.615234 | 0.000000 | ROUTE_INCOMPATIBLE_WITH_PROPOSAL |
| rh_hip_joint | 33/NEW06 | +1 | -45.922852 / 44.648438 | -44.384766 / 43.110352 | 0.000000 / 1.000000 | 42.110352 | 44.000000 | CANDIDATE_REVIEW_REQUIRED |
| rh_upper_leg_joint | 32/ELR02 | -1 | -53.173828 / 123.222656 | -51.591797 / 121.640625 | 51.470975 / 118.145906 | 3.494719 | 4.354094 | CANDIDATE_REVIEW_REQUIRED |
| rh_lower_leg_joint | 31/NEW05 | +1 | -91.625977 / 34.365234 | -90.043945 / 32.783203 | -92.000000 / 0.044440 | -1.956055 | 0.000000 | ROUTE_INCOMPATIBLE_WITH_PROPOSAL |
| lh_hip_joint | 43/M43 | +1 | -46.538086 / 45.219727 | -45.000000 / 43.681641 | -1.000000 / -0.000000 | 43.681641 | 44.000000 | CANDIDATE_REVIEW_REQUIRED |
| lh_upper_leg_joint | 42/M42 | +1 | -53.613281 / 122.387695 | -52.031250 / 120.805664 | 51.470975 / 118.145906 | 2.659758 | 4.354094 | CANDIDATE_REVIEW_REQUIRED |
| lh_lower_leg_joint | 41/M41 | -1 | -91.538086 / 34.145508 | -89.912109 / 32.519531 | -92.000000 / 0.044440 | -2.087891 | 0.000000 | ROUTE_INCOMPATIBLE_WITH_PROPOSAL |

Per ogni joint restano obbligatori record corrente/CRC/generation/provenance, identità fisica e mounting, zero e linkage osservati, repeatability/backlash, margine caricato, tracking e STOP. LF UPPER MAX richiede il controllo specifico riportato nel README. LOWER: bordo URDF −92° e proposta incompatibile non possono essere approvati mediante un flag. I contatti non qualificano velocità, accelerazione, tracking, envelope elettrico/termico o torque ceiling.

Tutti i min/max per fase sono in `joint_limits_review.json`; l’interpolazione articolare monotona quintica rende gli estremi dei knot validi per l’intero segmento. Rischio e verifica richiesta sono inclusi per ciascuna riga.
