# Dashboard completion — 2026-10-14T13:21:17Z

`MotionDashboard.h` extends the preserved Claude dirty patch; no viewer/bridge files were changed in this workstream. No serial port, robot, torque or flash was accessed.

## Delivered

- Separate `HTTP_TIMEOUT`, network/HTTP/schema failures, `TELEMETRY_STALE`, `CAPABILITIES_STALE`, `RESTORE_REQUIRED`, `MOTION_NOT_PERMITTED` and `MOTION_FAULT`. A fresh policy refusal or fresh fault no longer claims stale telemetry. Network failures are kept per endpoint, so a successful STOP cannot hide a failed status read.
- Observe response RTT (last/max/mean), timeouts, network errors and read retries. The default 750 ms timeout, 1 Hz visible polling, 1.5 s status freshness and 350 ms velocity TTL are preserved pending actual hardware timing. The existing bounded browser `motion_timeout_ms` override remains 300–5000 ms.
- At most one retry after 250 ms, only for the two GET read endpoints on timeout/network failure. POST commands are never replayed. A failed command revokes cached capabilities. Changed gates invalidate old capabilities before refresh completes; Controller clock rewind or changed `boot_nonce` ends held input.
- Show RAM commissioning mode, scoped joint count, entry/phase and `prime_clamp_mask` separately from formal limit approval. Missing actual joints remain UNKNOWN.
- Twin URL is browser-local and user configurable; parsed HTTP(S) URLs reject loopback/unspecified hosts and embedded credentials. The link label does not claim a LIVE hardware feed. No Three.js/STL assets enter the ESP32 application.

## Executed checks

```bash
NODE_PATH=/tmp/matdog-dashboard-dom-deps/node_modules node 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_dashboard_dom.cjs
NODE_PATH=/tmp/matdog-dashboard-dom-deps/node_modules node 05_Firmware/MATDOG_Controller/scripts/tests/test_portal_dom.cjs
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_http_transport.py
git diff --check -- 05_Firmware/MATDOG_Controller/src/network/MotionDashboard.h 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_dashboard_dom.cjs
```

- Motion DOM PASS: all 48 real catalog IDs; 6 preserved tabs; semantic command schema; disabled gates; TTL, release/focus/in-flight STOP; split diagnoses; measured fixture RTT; bounded read retry; timeout while reading JSON body; no POST replay; gate change/reboot; RAM commissioning display; browser LAN URL validation. Requests are test fixtures (`FIXTURE_ONLY`).
- Portal DOM PASS: 167 checks.
- HTTP transport PASS: 71 production-handler/security/service checks; HTTPD/socket/mailbox/driver observations use test doubles.
- Whitespace diff check PASS.
- Ancillary `test_motion_safety_boundaries.py` at this checkpoint: 75 checks, 1 FAIL (`Controller.cpp: missing ready.support_observable=reviewed_motion_bound_`). Controller was already dirty and is owned by the parallel Motion workstream; this workstream did not change it. Final Motion audit must resolve or document the exact new support predicate and rerun that check.

Test dependency: jsdom 26.1.0 installed outside the repository in `/tmp/matdog-dashboard-dom-deps`.

Tested file identities:

```text
08a8691cb873e3de56867e7473c6adec4a05ebffee1a444311b046064f35961a  MotionDashboard.h
c65bde3a2c6533fd2d07224e064845e30c9ced8fabbc39ddca0c3881bd647671  test_motion_dashboard_dom.cjs
```

## Hardware acceptance still outstanding

The configurable ESP32 link is supporting integration, not Android LIVE acceptance. The recovered host Dashboard/bridge/viewer commits and their reported FAKE_DEVICE E2E remain separate evidence. Determine the actual host LAN IP and Android reachability; start the unified host UI using the sole serial owner; verify real BNO085 quaternion, actual encoder joints, separate target, source/freshness and measured cadence with torque OFF first. No real RTT, AP-client reachability, hardware LIVE feed or physical commissioning was claimed by these DOM tests.
