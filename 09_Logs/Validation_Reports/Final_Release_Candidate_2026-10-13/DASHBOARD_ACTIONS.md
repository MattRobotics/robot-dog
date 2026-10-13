# 48 azioni Dashboard — catalogo effettivo

ID e parametri estratti da MotionContract.cpp. `software_tested` identifica la presenza del percorso software, mai qualifica hardware. LIE_DOWN/BODY_POSE/leg/joint/funzioni future restano planned e disabilitate; l’admission specifica impone limiti più stretti dei massimi generali qui esposti. WALK runtime qualificato per una prima falcata avanti, vy/yaw zero; le altre transizioni restano bloccate fisicamente. USB operatore gestisce grant e avanzamenti; la LAN è sempre read-only.

| ID | Categoria | Parametri | Capability | Intent | Stato |
|---|---|---|---|---|---|
| posture.stand | Posture | target_height_m:0.06..0.15 | stand | STAND | software_tested; gate richiesti |
| posture.lie_down | Posture | target_height_m | lie_down | LIE_DOWN | planned; disabled |
| posture.sit | Posture | none | sit | ACTION_EXECUTE | planned; disabled |
| posture.crouch | Posture | target_height_m | crouch | ACTION_EXECUTE | planned; disabled |
| posture.height | Posture | target_height_m:0.06..0.15 | stand | STAND | software_tested; gate richiesti |
| posture.confirm | Posture | target_height_m:0.06..0.15 | stand | STAND | software_tested; gate richiesti |
| gait.walk | Gait | gait:WALK | walk | GAIT_SELECT | software_tested; gate richiesti |
| gait.trot | Gait | gait:TROT | trot | GAIT_SELECT | planned; disabled |
| gait.pace | Gait | gait:PACE | pace | GAIT_SELECT | planned; disabled |
| gait.bound | Gait | gait:BOUND | bound | GAIT_SELECT | planned; disabled |
| gait.amble | Gait | gait:AMBLE | amble | GAIT_SELECT | planned; disabled |
| gait.crawl | Gait | gait:CRAWL | crawl | GAIT_SELECT | planned; disabled |
| body.shift_forward | Body | x_m | body_pose | BODY_POSE | planned; disabled |
| body.shift_backward | Body | x_m | body_pose | BODY_POSE | planned; disabled |
| body.shift_left | Body | y_m | body_pose | BODY_POSE | planned; disabled |
| body.shift_right | Body | y_m | body_pose | BODY_POSE | planned; disabled |
| body.shift_up | Body | z_m | body_pose | BODY_POSE | planned; disabled |
| body.shift_down | Body | z_m | body_pose | BODY_POSE | planned; disabled |
| body.roll | Body | roll_rad | body_pose | BODY_POSE | planned; disabled |
| body.pitch | Body | pitch_rad | body_pose | BODY_POSE | planned; disabled |
| body.yaw | Body | yaw_rad | body_pose | BODY_POSE | planned; disabled |
| body.neutral | Body | none | body_pose | BODY_POSE | planned; disabled |
| action.bow | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.stretch | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.shake | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.wave | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.salute | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.look_left | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.look_right | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.scan | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.dance | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| action.turn_left | Actions | yaw_rate | preset | ACTION_EXECUTE | planned; disabled |
| action.turn_right | Actions | yaw_rate | preset | ACTION_EXECUTE | planned; disabled |
| action.step_over | Actions | none | preset | ACTION_EXECUTE | planned; disabled |
| leg.lf.position | Legs | xyz_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.rf.position | Legs | xyz_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.lh.position | Legs | xyz_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.rh.position | Legs | xyz_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.lf.lift | Legs | height_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.rf.lift | Legs | height_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.lh.lift | Legs | height_m | leg_control | ACTION_EXECUTE | planned; disabled |
| leg.rh.lift | Legs | height_m | leg_control | ACTION_EXECUTE | planned; disabled |
| recovery.observe | Recovery | none | observe | ACTION_EXECUTE | planned; disabled |
| recovery.classify | Recovery | none | classify | ACTION_EXECUTE | planned; disabled |
| recovery.acquire_support | Recovery | none | acquire_support | ACTION_EXECUTE | planned; disabled |
| recovery.stand | Recovery | target_height_m:0.06..0.15 | stand | STAND | software_tested; gate richiesti |
| motion.stop | Safety | none | stop | STOP | software_tested; gate richiesti |
| motion.status | Safety | none | status | STATUS | software_tested; gate richiesti |
