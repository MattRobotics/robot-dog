# Proposed commit

`Integrate gated Motion service and restore; classify partial support and document prone geometry blockers`

# Proposed PR description

MATDOG's existing Motion Core now has a versioned semantic Controller service,
feedback-paced STAND/WALK coordinator and radian executor using the existing Safe
Actuator Layer. Runtime calibration restore, independent limit/permit admission,
48 capability-gated actions, joystick deadman/STOP and a host Three.js twin are
included. Existing six dashboard tabs, bus ownership and safety protections are
preserved.

The classifier handles partial support through explicitly matched route/contact
facts and refuses unknown or unqualified acquisitions. Four-foot intermediate
117mm and already-standing 153mm starts reach 150mm in software simulation; G4
WALK transitions to coordinated STOP/STAND and rearm. Their synthetic facts do
not establish measured load, tracking or stability.

Natural belly-to-stand is not solved by this candidate. The historical successful
stand is documented but lacks its measured joint/contact trace. Full canonical
V5 evaluation rejects the preserved fixture acquisition with 212 self-intersecting
frames. Three folded geometric hypotheses, two deployment recipes and one bounded
rock-first correction are supplied with real 17-link/STL renders, projections,
COM/contact plots and exact floor-interference witnesses. No failed candidate
is promoted to executable capability.

Validation: fresh AdaptiveStand 11484 checks and Coordinator/Service/Executor
2422 checks pass; unchanged RESTORE, lifecycle, HTTP/AP, DOM/browser, viewer and
legacy static/host/mutation evidence is retained. The 286 deferred historical
oracles are not counted as execution PASS. The latest aggregate's missing-jsdom
environment error is explicitly recorded. ESP32-S3 ROBOT_POWERED build, source
inventory, complete patch and software receipt accompany delivery externally.

MOTION_AUTHORIZED=0 and OTA_INGEST=0. Production measured fact/assessment/permit
and arm producers, normal RUN_STA authenticated motion transport, approved
workspace limits, autonomous acquisition, live nonzero-velocity changes and
loaded support/tracking/stability remain open. The normal flash gate refuses
TREE_NOT_CLEAN. This change is a software review candidate, not physical trial
readiness.

No commit, PR creation, merge, push, flash/OTA, torque, real motion or NVS/EEPROM
change occurred. Review the complete diff and new artifacts before publication.
