# Motion Digital Twin — 12 October 2026

The existing host Three.js viewer and canonical collision URDF now render Controller motion telemetry. The embedded portal preserves its six tabs and appends Motion Control, with 48 actions from `MotionService::capabilitiesJson`. Availability always comes from the Controller. Browser controls send version 1 semantic intentions; they cannot grant an actuator permit or access the bus.

## Run the offline demonstration

```sh
# Repository root: shipped embedded portal, localhost fixtures, no hardware.
python3 05_Firmware/MATDOG_Controller/scripts/motion_dashboard_demo.py
# Use the tested Motion Core JSONL output:
# append --trace 09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_synthetic.jsonl

cd 06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer
npm ci
# Optional staging of the actual offline trace (run before dev):
node scripts/sync_canonical_assets.mjs --motion-trace ../../../../09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/adaptive_motion_synthetic.jsonl
npm run dev
```

Open `http://127.0.0.1:5185`, then Motion Control. All 48 cards and all controls are visible. Physical actions remain disabled; STOP only acknowledges the local fixture. The demo imports no device drivers. Existing Wi-Fi and OTA pages remain visible and make no writes automatically; configuration requests in this offline server are rejected.

Open `http://127.0.0.1:5183/?motion=synthetic` for the full visual fixture: prone → observe/classify → acquire support → rise 150 mm → verify/stand → WALK → STOP → restart at 105 mm. The synthetic display values exercise composition and UI behavior. They are **not** a collision/contact/stability qualification of a trajectory. Use “Replay verified offline trace” to display the Motion Core test output without generating new joint commands. The accepted file is JSONL, an array of status objects, or `{ "frames": [...] }`, capped at 10 MiB and 10,000 samples. Timestamps and target/actual radians are preserved. The offline adapter translates the test export's `stamp_ms` to `sampled_at_ms` and keyed feet with declared `STANCE`/`SWING` to foot entries with `support`/`swing`; the live Controller parser remains strict. Source evidence stays visibly labelled `SYNTHETIC_NOT_HARDWARE_QUALIFIED`. After staging, open `http://127.0.0.1:5183/?motion=trace` to replay the 2,319-sample test output. Replay stops at the last frame and becomes STALE; it never wraps timestamps backwards. If asset staging/tests recreate public files while Vite is running, restart Vite before the demo.

## Telemetry contract

`GET /api/motion/status` contains `version:1`, Controller uptime `now_ms`, `executor_state`, `stand_state`, `owner`, `fault`, `reason`, `stale`, and `telemetry`. Telemetry includes `sampled_at_ms`, `actual_rad[12]`, `target_rad[12]`, `quaternion[w,x,y,z]`, nullable `base_height_m`, and optional `feet`. Arrays use **LF, RF, RH, LH**, each hip, upper leg, lower leg. An explicit `joint_names[12]` overrides ordering only if it names each canonical joint exactly once. Missing channels are `null`, not zeros. Invalid or out-of-order timestamps and unsupported versions are rejected; unavailable actual joints are hidden rather than shown as measured neutral joints.

A foot entry is `{leg:"LF",position_m:[x,y,z],contact:"support"|"swing"|"unknown",phase:0..1,trajectory_m:[[x,y,z],...]}`. Foot positions and trajectories must use the same reported support-plane frame as the body height. At most 64 trail points per foot are retained. Contact is reported by the source and never inferred from IMU or joint geometry. `base_height_m` is the distance of the `base_link` origin to the support plane; no height is computed from the IMU position.

Grey collision meshes show actual articulation; cyan wireframe collision meshes show targets. Both receive the same base quaternion; the two joint channels remain independent. Full motion playback clears/disables IMU recenter so the body, support plane, feet and trajectories share one world frame; the original recenter controls remain available for standalone IMU inspection after pausing the motion feed. Feet display reported contact and gait phase. Freshness combines Controller sample age with local transport age. On feed pause, at 1.5 seconds, or an explicit stale flag the UI says STALE and hides target/contact overlays. Faults, tracking error and sample timestamps remain visible. The canonical model is unchanged; all 17 collision STLs are shared by actual and ghost instances.

Rendering is capped at 30 Hz. Synthetic/trace sampling is 10 Hz and Controller HTTP polling is 1 Hz. Hidden tabs suspend polling and rendering. Telemetry and rendering clocks are independent. The embedded page polls compact status at 1 Hz only while its Motion section is active and fetches the full action catalog about once every 4 seconds; held teleoperation intentions renew at 120 ms with a 350 ms TTL. Input release, blur, hidden page, offline events, stale capability and revocation send STOP. An in-flight request queues STOP immediately behind the request. Controller expiry and executor watchdog remain the authoritative protection if the browser cannot deliver STOP.

## Optional real read-only feed

The viewer does not open a serial port automatically. Its existing user-operated BNO085 Web Serial reader remains read-only. Full Controller HTTP telemetry can use the host Vite proxy:

```sh
MATDOG_CONTROLLER_URL=http://192.168.4.1 npm run dev
```

Choose “Controller read-only” or open `?motion=live`. Only `/api/motion/status` is proxied; no command route is exposed by the viewer. The production server retains its own admission checks. This requires network reachability and a Controller exposing the status service. No real hardware telemetry was available for this change. A production static viewer needs a same-origin reverse proxy for the same read-only route; browser CORS or mixed-content restrictions must not be bypassed.

## RUN transport gate

`TRANSPORT_MOTION_RUN_STA` remains an open software requirement. In normal RUN with a configured router, production Wi-Fi policy selects CLIENT_STA, revokes AdminAccess and closes the AP. The existing TLS listener excludes all Motion routes. In MAINTENANCE the administrative AP is reachable, while MotionService rejects motion with `MODE_RUN_REQUIRED`.

This is not a universal RUN/AP incompatibility: production RUN_NO_PROFILE_AP and RUN_RECOVERY_AP legitimately open the exclusive AP and its generation. A composed offline test runs actual WifiManager, NetworkPolicy, AdminAccess, MotionService and the extracted production admission body using a single OperatingMode input. It passes 31 semantic assertions, 2,077 including driver/queue/fence checks, under ASan/UBSan. Driver/events, scheduler, NVS and isolated readiness facts are explicit doubles; no executor or actuator is bound. Configuration mutations are refused in all RUN cases and write no NVS-double records after fixture seeding. See `motion_transport_mode.json` and `.log` in the session validation directory.

The minimum future change is an independently authenticated operational transport reachable in RUN_STA, delegating the existing versioned semantic contract to Controller/MotionService. Its authentication must never grant actuator authority. Wi-Fi/config mutations retain MAINTENANCE AdminAccess. No endpoint or admission protection is changed in this candidate; TTL, STOP, watchdog and revocation must be qualified on that channel before operational use.

## Embedded asset budget

The embedded portal includes only HTML/CSS/JS for controls and a host-viewer link. It does not fetch 3D assets from the ESP32. `localhost:5183` refers to the computer running Vite, not a phone opening the robot AP; a phone needs a reachable host viewer address.

Build measurements are in `MOTION_ASSET_BUDGET.json`. The core host build is 6,609,980 bytes, including approximately 6.0 MB of canonical collision assets. With the optional offline trace staged, the measured demo build is 8,907,951 bytes; `host_dist_core_bytes` and `offline_trace_assets_bytes` distinguish those assets. This exceeds the 5 MiB APP partition and the existing 0x5D0000 FAT partition after the full host bundle is counted. No flash-layout change or asset partition write is performed. Production integration remains an external host link or a host-served iframe with separately configured origins and CSP. The AP portal intentionally keeps its current strict CSP and embeds no external iframe.

## Verification

`npm run verify` checks TypeScript, all existing viewer regressions, motion schema/freshness/composition, canonical asset identity and the read-only safety boundary, then builds the host bundle. Dashboard tests execute shipped embedded scripts with synthetic Controller fixtures:

```sh
# jsdom may be installed in an isolated directory rather than the firmware tree.
NODE_PATH=/tmp/matdog-dashboard-deps/node_modules node 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_dashboard_dom.cjs
NODE_PATH=/tmp/matdog-dashboard-deps/node_modules node 05_Firmware/MATDOG_Controller/scripts/tests/test_portal_dom.cjs
```

The Motion DOM test covers 48 actions, disabled future gait/actions, six preserved tabs, semantic units/envelope, clock/ID/sequence/TTL, input release, blur, stale status, offline events and STOP queued during an outstanding command. Offline fixtures do not establish a hardware permit. The viewer regression suite passes 67 tests. A fresh owned headless Chrome browser with SwiftShader rendering also verified desktop/mobile layout, all 48 action cards, 46 disabled physical actions, six preserved tabs, 17 loaded canonical collision meshes, 12 movable joints, actual offline trace and synthetic WALK rendering, and STALE target hiding. It issued zero writes. Screenshots and `browser_checks.json` are saved under `09_Logs/Validation_Reports/Motion_Enablement_2026-10-12/browser/`; no hardware or real telemetry was used.

The extracted-production HTTP harness compiles the unmodified `handleMotion`, Controller service cases, admission and heap snapshot copy, with real PortalSession, JSON parser, MotionService and catalog. It passes 56 assertions under ASan/UBSan covering status/catalog, AP/STA fencing, origin, bounded receive/timeout, browser owner, cookies/CSRF and backend rejections. HTTPD/socket delivery, mailbox scheduling and radio observations are explicit host doubles; it does not establish firmware timing or hardware validity.

```sh
python3 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_http_transport.py
# playwright-core must be available in NODE_PATH; MATDOG_TEST_CHROME can select a local Chromium executable.
NODE_PATH=/home/matteo-manicardi/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules node 05_Firmware/MATDOG_Controller/scripts/tests/test_motion_browser.cjs
```
