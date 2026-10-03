"""Fail-closed observer of the existing @BMS STREAM, on the runner's link.

No transport, UART access, DALY writes or movement lives here. Protocol age
is the firmware's last communication result age; only OK is admissible.
"""
import re

MAX_AGE_S = 5.0  # DalyProtocol.h: 2 s polls + 750 ms timeout, existing 5 s freshness
BLOCK_TIMEOUT_S = 1.0


class DalyGuardFailure(Exception):
    pass


class DalyGuard:
    def __init__(self):
        self.stage = 0
        self.block_started = None
        self.comm_at = None
        self.age_s = 0
        self.last = None
        self.cursor = 0
        self.fault = None
        self.blocks = 0
        self.values = {}

    def fail(self, reason):
        self.fault = reason
        raise DalyGuardFailure(reason)

    def feed(self, line, received):
        if self.fault:
            raise DalyGuardFailure(self.fault)
        if line.startswith("DALY "):
            if self.stage > 1:
                self.fail("DALY incomplete telemetry block")
            if not re.fullmatch(r"DALY\s+init=OK detected=ONLINE expected=REQUIRED result=PASS", line):
                self.fail("DALY unavailable: " + line)
            self.stage = 1
            self.block_started = received
            self.values = {}
            return
        if not self.stage:
            return
        patterns = {
            1: r"  comm=(\S+) age_ms=(\d+)",
            2: r"  pack_v=(\d+\.\d+) current_a=(-?\d+\.\d+) soc=(\d+\.\d+)% cells=(\d+)",
            3: r"  cell_max_mv=(\d+) cell_min_mv=(\d+) delta_mv=(\d+)",
            4: r"  charge_mos=(ON|OFF) discharge_mos=(ON|OFF) state=(\S+) alarms=([0-9A-F]{4}) ([0-9A-F]{4}) ([0-9A-F]{4}) ([0-9A-F]{4})",
        }
        match = re.fullmatch(patterns[self.stage], line)
        if not match:
            # @STATUS also prints a DALY availability-only line, without a
            # telemetry body. It neither refreshes nor invalidates a sample.
            if self.stage == 1 and not line.startswith("  comm="):
                self.stage = 0
                return
            self.fail("DALY malformed/incomplete telemetry: " + line)
        if self.stage == 1:
            if match[1] != "OK":
                self.fail("DALY communication " + match[1])
            self.age_s = int(match[2]) / 1000
            if self.age_s > MAX_AGE_S:
                self.fail("DALY stale firmware sample")
            self.comm_at = received
        elif self.stage == 2:
            pack = float(match[1])
            if int(match[4]) != 3:
                self.fail("DALY cell topology is not the current 3S pack")
            if pack < 10.8:
                self.fail("DALY pack below 10.8 V")
            self.values.update(pack_v=pack, cells=3, soc_percent=float(match[3]))
        elif self.stage == 3:
            maximum, minimum, delta = map(int, match.groups())
            if minimum < 3600:
                self.fail("DALY minimum cell below 3.60 V")
            if maximum < minimum or maximum - minimum != delta:
                self.fail("DALY inconsistent cell extrema")
            self.values.update(cell_min_mv=minimum, cell_max_mv=maximum, delta_mv=delta)
        else:
            if match[2] != "ON" or any(int(v, 16) for v in match.groups()[3:]):
                self.fail("DALY discharge disabled or alarms present")
            self.last = self.comm_at - self.age_s
            self.blocks += 1
            self.stage = 0
            return
        self.stage += 1

    def check(self, now):
        if self.fault:
            raise DalyGuardFailure(self.fault)
        if self.stage > 1 and now - self.block_started > BLOCK_TIMEOUT_S:
            self.fail("DALY incomplete telemetry deadline")
        if self.last is None:
            raise DalyGuardFailure("DALY no complete valid telemetry yet")
        if now - self.last > MAX_AGE_S:
            self.fail("DALY telemetry missing or stale on the host link")

    def snapshot(self):
        return dict(self.values, blocks=self.blocks, max_age_s=MAX_AGE_S)
