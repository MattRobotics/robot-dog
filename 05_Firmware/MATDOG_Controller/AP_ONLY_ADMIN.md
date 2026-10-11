# AP-only administration contract

Baseline: `b7516c39c9fcba09992acd07377facd5ae6e8588`.
The accepted replacement for concurrent AP+STA administration is exclusive
`ADMIN_AP` / `CLIENT_STA`. Historical F01 interface-ingress diagnostics remain
negative for the old AP+STA requirement; this change does not close that
abandoned requirement or claim a veto over physical radio failures.

## State and authorization

`CLOSED → ENTER_ADMIN → ADMIN_AP → LEAVE_ADMIN → CLIENT_STA`.
`FAULT` prevents automatic reopening after an observed AP stop or an invalid
radio observation. A new explicit AP request must pass the closure barrier and
repeat driver/AP/STA verification. Startup/fallback may request ENTER_ADMIN,
but cannot open the portal until verification completes.

ADMIN_AP requires successful `esp_wifi_get_mode` with WIFI_MODE_AP, AP started,
STA not started, no connected STA/IP, and a driver response documenting no STA
association. Unknown driver observations deny. Generation exhaustion denies.
The worker suppresses connect, scan, roaming, retry and radio-recovery policies
outside CLIENT_STA. There is no production AP+STA mode command.

All `/wifi/` requests and the portal page check admission at HTTP receipt.
Queued dispatch and Controller service check the captured generation again.
Controller authentication and the last worker mutation take the same nonblocking
mutex lease. The worker rechecks generation and radio before accepting the
intent or writing configuration. The digest used for login is protected by the
same fence, rather than the delayed Controller snapshot. Sessions are reset
when their generation becomes unauthorized; a new generation requires login.
Cookie, CSRF, monotonic 64-bit expiry, peer binding, Host/Origin and throttling
remain in the existing security implementation.

For an owned transition: revoke admission, drain leases, stop HTTP and HTTPS on
`md_http_life`, acknowledge successful closure of both, then change mode on
`md_network`. Failed stops retain their handles and cannot acknowledge closure.
A blocked stop cannot authorize a timeout-based transition. HTTPD, TCP/IP and
Arduino event callbacks never execute listener stop. HTTPS registers only
nonadministrative status/OTA routes; shared admin handlers also deny outside AP.
OTA ingest remains disabled; this change adds no motion authorization.

The event callback revokes admission without waiting. The worker completes fault
handling and the lifecycle task drains/stops listeners. An operation whose lease
was already admitted can finish before the drain barrier; no subsequent lease
or queued old-generation request can mutate after that barrier. A physical
failure may precede its notification, and an admitted operation may complete in
that unobserved interval. Neither event latency nor the duration of an in-flight
commit has an offline physical bound. These are observed-revocation guarantees,
not preventive radio-failure guarantees. A spontaneous AP_START never authorizes
administration by itself.

## Operator procedure

1. Use the existing quiet MAINTENANCE state, with no calibration/OTA activity.
   Request AP through the project's existing physical command if needed.
2. Connect to the protected MATDOG AP and open `http://192.168.4.1`.
   Login with the existing unique 64-hex administrator token.
3. Enter SSID/password manually, retain the existing DHCP/static-IP validation,
   choose the enabled test profile and confirm **Prova e salva**.
4. The response reports `ADMIN_CLOSING_STA_TEST_RELOGIN_REQUIRED`. The accepted
   merged candidate is initially RAM-only. The worker allows a short response
   interval, revokes administration and waits for confirmed listener shutdown.
   Browser/page continuity is not promised. Competing Wi-Fi ON/OFF/AP commands
   are refused while the validated test intent is pending or testing.
5. In exclusive CLIENT_STA, the existing transaction writes PENDING and tests
   fresh association plus IP against the selected SSID. Success activates the
   existing ACTIVE blob; timeout, critical context or failed validation rolls
   back. Storage errors retain their existing fail-closed handling. Neither the
   record format nor the NVS namespace changes.
6. The firmware returns to a freshly verified AP generation to expose the result.
   Reconnect and login again; read COMMITTED, ROLLBACK or STORAGE_ERROR. For an AP
   identity change, successful commit uses the new AP credentials, otherwise the
   old ones remain. There is no simultaneous STA connection while viewing this
   result.
7. When finished, deactivate AP (`/wifi/ap`, `on=0`, authenticated cookie/CSRF),
   or use the existing physical AP OFF command. This closes administration and
   resumes STA operation using ACTIVE. `ap_always` must be false to request AP
   OFF. The AP idle timeout now counts in exclusive AP mode, without a STA link.

Manual AP scan returns `SCAN_DISABLED_ADMIN_AP`; automatic scan cannot run there.
Authorized physical scans and the existing roaming policy remain in CLIENT_STA.
Initial physical key provisioning is possible in CLOSED after explicit Wi-Fi
OFF and confirmed closure, then Wi-Fi ON / AP ON. Key writes are refused in
CLIENT_STA or transitions. No HTTP endpoint exposes this CLOSED bootstrap.

## Offline checks and hardware follow-up

Run `scripts/tests/run_host_tests.sh`, `scripts/static_audit.py`, and
`scripts/tests/test_ap_only_transport.py`. The new tests execute productive
manager code and extracted, unmodified HTTP admission/auth/lifecycle/routing
paths. Driver, HTTPD/TLS, sockets, event scheduling and NVS transport are declared
doubles; they do not prove physical radio behavior, TCP/IP scheduling or TLS.
F02 additionally runs `scripts/tests/test_ap_session_clock.py`.
`test_ap_transport.py` and `test_ap_ingress.py` default to the frozen baseline;
the latter must retain its historical FAIL, not be relabeled AP-only PASS.

On hardware, in a separately authorized session, verify exclusive driver mode,
STA inactivity, IPv4/mapped browser login and authenticated POST; scan refusal
without mode change; stale cookie/CSRF and queued-request rejection after revoke;
HTTP/HTTPS closure before every commanded mode change; stop-failure hold;
unexpected AP stop/start requiring explicit re-verification and login; both
credential success and rollback; AP identity change; subsequent STA connection,
manual scan/roaming and OTA-ingest rejection. Measure event-to-revocation and
lease-drain intervals, without interpreting them as physical-failure prevention.
No hardware steps are authorized or executed by this offline implementation.

`MOTION_AUTHORIZED=0` · `RESTORE=NOT_IMPLEMENTED` ·
`HARDWARE_FLASH_AUTHORIZED=NO`.

IR-01 lifecycle acknowledgment: each owned driver mode command arms the expected
AP_STOP, STA_START, STA_STOP and AP_START role changes before calling WiFi.mode.
The event callback consumes each acknowledgment once with atomic CAS, without
waiting or taking the mutation mutex. Another mode command cannot start while
any acknowledgment is pending. WAIT_AP_CLOSE retains revocation until the old
AP closure and new STA start are observed and driver/interface state is checked.
ENTER_ADMIN similarly requires STA closure and AP start plus the existing radio
verification before opening a fresh generation. Events of the same command may
arrive in either order; duplicate or unrelated role events revoke immediately.
Missing acknowledgments time out after 3000 ms in the worker and leave FAULT;
late delivery alone never reopens administration. Arduino events carry no
generation identifier: attribution relies on serialized commands and completion
of their role-event set, not a fixed delay. Physical faults remain observable
only after notification, as described above.

The IR-01 regression harness preserves the independent probe's storage/task
doubles and production manager linkage. Its assertions now require WAIT_AP_CLOSE
before reentry, and explicitly deliver queued stop/start acknowledgments for
each command. Baseline vulnerable reproduction and all original independent
review inputs remain separate and unchanged.

## AP profile acknowledgment and request confirmation (2026-10-10)

Hardware showed `config_error=11` after every boot of `41753ac`. The stock
`WiFi.softAP()` writes the AP profile with `esp_wifi_set_config()`; on a started
soft-AP the driver stops and restarts the interface and reports AP_STOP then
AP_START. Those events were outside the armed set of the mode command and were
treated as an unsolicited radio change.

ENTER_ADMIN now issues two owned commands. Command 1 is the role change
(STA_STOP/AP_START, or AP_START alone from a closed driver). Command 2 is the
profile, issued with `esp_wifi_set_config(WIFI_IF_AP)` only after command 1 is
fully acknowledged and the AP interface reports started; it arms AP_STOP and
AP_START before the call. When the driver already holds the identical profile
nothing is written and nothing is armed. A generation opens only after both
sets are consumed, the exclusive AP state is verified and the profile read back
from the driver equals the stored SSID and key with WPA2-PSK
(`AP_CONFIG_MISMATCH`, `config_error=12`, otherwise). Duplicate, unrelated or
missing role events fault exactly as before.

FAULT closes the radio once (after confirmed listener closure, with no
acknowledgment outstanding), so the network is not left visible without
administration. A missing acknowledgment still leaves FAULT with the radio
untouched, because no further mode command may start while one is pending.

Requests are confirmed by sequence number, not by phase. `WIFI_PROVISION=QUEUED
kind=… seq=N` is an intent; `WIFI_CONFIG_TX … seq=N kind=… result=COMMITTED`
is its outcome. `WIFI_PROVISION=REFUSED kind=… reason=…` never advances the
sequence. A key write accepted while the AP is active closes and reopens
administration; a second key sent during that window is refused
(`NOT_ADMITTED`) and must be repeated after `WIFI_ADMIN phase=ADMIN_AP`.
`@WIFI AP VERIFY <passphrase>` compares a supplied key with the stored one and
with the live driver profile and prints only MATCH/MISMATCH.

## Dashboard, remembered device and STA test report (2026-10-11)

Nothing in this section changes the exclusive `ADMIN_AP` / `CLIENT_STA`
contract above: one radio, and the portal exists only in a verified AP
generation.

**Reading is open inside the AP, writing is not.** `GET /status` and
`GET /wifi/status` need no login; any client that knows the WPA2 key can read
them. Every `/wifi/` POST that changes anything needs the session cookie, the
CSRF header, the same peer address, the AP Host/Origin and an admitted
generation. Knowing the AP key is not administrative permission.

**Session.** Unchanged: opened by the 64-hex administrator token, bound to the
peer, 5 minutes, ended by a new generation, an administrator-digest change or
`POST /wifi/logout`.

**Remembered device.** With "Ricorda questo dispositivo" the login also sets
`md_device`, a credential derived from the administrator digest with
HMAC-SHA256 and a random 16-byte id. The controller stores nothing per device
and writes nothing to NVS; verification is a recomputation. Consequences, all
intended:

- it survives a controller restart and needs no clock;
- it is void when no administrator is provisioned;
- provisioning a new administrator token (`@WIFI ADMIN KEY`, USB) revokes every
  remembered device at once. There is no per-device revocation; "Esci" removes
  the credential from that browser and ends its session;
- it is sent only to `POST /wifi/session` (`Path=/wifi/session`), which answers
  with a fresh ordinary session. It is never accepted as a session cookie or as
  a CSRF token, so by itself it authorizes no write;
- wrong credentials are throttled (1 s, 60 s after five), independently of the
  token throttle. A request carrying no credential is not an attempt.

Trust boundary, stated plainly: the portal is plain HTTP inside WPA2-PSK. A
client that holds the AP key and captures another client's handshake can read
that client's traffic, which already included the administrator token at
login; the remembered credential is exposed to the same observer, less often
than the session cookie. Keep the AP key private; change the administrator
token to revoke.

**Dashboard.** The page polls `/status` then `/wifi/status`, sequentially, every
4 s while visible, and renders cached controller state only. A BMS value is
shown as current only while `bms_fresh=YES` (the controller's
`kDalyTelemetryFreshnessMs` contract); otherwise N/D or STALE. Battery bar
colours use page constants (`BAT_LOW` 30 %, `BAT_CRIT` 15 %) or the
controller's own warning/critical facts. The page has no actuator, calibration
or motion endpoint.

**STA test ("Prova e salva").** Sequence seen by the operator:

1. The request is accepted in RAM; about one second later the AP and the portal
   close. The phone is disconnected. This is expected and unavoidable.
2. In exclusive `CLIENT_STA` the candidate is written as `PENDING` and tested
   for at most 30 s. A failed attempt is retried (at most every 4 s). The
   candidate becomes `ACTIVE` only if association to the tested SSID and an
   address are held for 1.5 s.
3. Whatever the result, the firmware returns to a freshly verified AP with the
   stored AP key. On failure `ACTIVE` was never written: the known-good
   configuration is intact, also across a power cut during the test (`PENDING`
   is never promoted at boot).
4. Reconnect the phone to the AP. A remembered device is recognised without the
   token. The portal shows the outcome; `@WIFI STATUS` prints it as
   `WIFI_STA_TEST`.

Outcomes: `CONNECTED` (committed, with address and RSSI); `AUTH_REJECTED`
(driver reason 15, 202 or 204: the router refused the WPA2 handshake);
`AP_NOT_FOUND` (201, 210–212); `NO_IP` (associated, no address);
`TIMEOUT`; `ABORTED` (critical context or radio fault); `STORAGE_ERROR`.

Not recoverable without USB or a power cycle, as before: a radio `FAULT`
(fail closed, radio off) needs `@WIFI AP ON` or a restart. After a successful
commit with `ap_always` false, the next boot joins the router and opens the AP
only as fallback (no credentials, or two connect timeouts) or on `@WIFI AP ON`.
With no TLS identity provisioned the controller serves nothing in `CLIENT_STA`.
With `ap_always` true the controller returns to its AP at every boot and keeps
the router profile stored but unused; AP OFF is then refused. The Wi-Fi form
states this choice ("Dopo il salvataggio") and, when it differs from the stored
policy, sends it in the same request as the router credentials, so both are
committed or neither is. A first router setup preselects "AP sempre attivo".

