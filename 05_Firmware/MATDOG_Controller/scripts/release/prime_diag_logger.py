#!/usr/bin/env python3
"""One-open, no-reset USB PRIME_DIAG acquisition. No flasher or servo API.

Run only AFTER the separately supervised flash/boot and physical setup check.
TX/RX byte chunks (including partial lines) are durably logged as base64 JSONL.
No reconnect, START retry, CLEAR, reset, heartbeat, or torque command exists.
"""
import argparse
import base64
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import select
import signal
import time

SOURCE = '8b9ee70a60bfa777ed50484c5e9546ceffd72f3b'
APP_SHA = '45843799db102804627674d7ed51ae9a6def9ce5d6a850cc842cc95773fa2601'
PORT = '/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00'
IDS = (13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41)
STATUS = '@MOTION VALIDATION PRIME_DIAG STATUS'
STOP = '@MOTION VALIDATION STOP'
START_TOKEN = 'CONFIRM_FIXTURE_PRESENT_POWER_CUT_TORQUE_OFF_NO_ENERGIZE'


class Refusal(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise Refusal(message)


def fields(line):
    result = {}
    for word in line.split():
        if '=' in word:
            k, v = word.split('=', 1)
            require(k not in result, 'DUPLICATE_FIELD:' + k)
            result[k] = v
    return result


def integer(d, key):
    try:
        return int(d[key], 16 if d[key].startswith('0x') else 10)
    except (ValueError, KeyError) as exc:
        raise Refusal('MISSING_OR_INVALID_FIELD:' + key) from exc


def validate_record(line, index):
    require(line.startswith('PRIME_RECORD '), 'NOT_A_RECORD')
    d = fields(line)
    for key in ('reason', 'validity', 'rx'):
        require(key in d, 'MISSING_FIELD:' + key)
    for key in ('index', 'slot', 'id', 'attempt', 'session_ms', 'target_position',
                'capture', 'target_speed', 'target_acc', 'ack', 'write_start_ms',
                'write_done_ms', 'read_start_ms', 'read_done_ms', 'done_ms',
                'address', 'width', 'rx_size', 'drained', 'decoded_valid',
                'position_read', 'speed_read', 'acc_read', 'raw_payload',
                'raw_position', 'raw_speed', 'raw_acc', 'mismatch'):
        integer(d, key)
    require(integer(d, 'index') == index, 'RECORD_INDEX')
    slot = integer(d, 'slot')
    require(0 <= slot < 12 and integer(d, 'id') == IDS[slot], 'RECORD_SLOT_ID')
    require(1 <= integer(d, 'attempt') <= 3, 'RECORD_ATTEMPT')
    try:
        raw = bytes.fromhex(d['rx'])
    except ValueError as exc:
        raise Refusal('RECORD_HEX') from exc
    require(len(d['rx']) == len(raw) * 2 and len(raw) == integer(d, 'rx_size') <= 38,
            'RECORD_RAW_SIZE')
    capture = integer(d, 'capture')
    require(capture in (0, 1), 'RECORD_CAPTURE_FLAG')
    if not capture:
        require(not raw and d['validity'] == 'NOT_ATTEMPTED' and
                integer(d, 'decoded_valid') == 0 and integer(d, 'mismatch') == 0 and
                all(integer(d, k + '_read') == -1 for k in ('position', 'speed', 'acc')),
                'STALE_RESPONSE')
        return d
    require(integer(d, 'address') == 40 and integer(d, 'width') == 10,
            'ORIGINAL_BLOCK_CHANGED')
    require(integer(d, 'target_speed') == 20 and integer(d, 'target_acc') == 5,
            'PRIME_PROFILE_CHANGED')
    require(0 <= integer(d, 'target_position') <= 4095, 'TARGET_RANGE')
    require(0 <= integer(d, 'drained') <= 64, 'RX_DRAIN_RANGE')
    for key in ('write_start_ms', 'write_done_ms', 'read_start_ms', 'read_done_ms', 'done_ms', 'session_ms'):
        require(0 <= integer(d, key) <= 0xffffffff, 'TIMESTAMP_RANGE')
    validity = ('NO_RESPONSE' if not raw else 'SHORT_PACKET' if len(raw) != 16 else
                'BAD_HEADER' if raw[:2] != b'\xff\xff' else
                'WRONG_ID' if raw[2] != IDS[slot] else
                'WRONG_LENGTH' if raw[3] != 12 else
                'SERVO_STATUS' if raw[4] else
                'VALID' if sum(raw[2:]) % 256 == 255 else 'BAD_CHECKSUM')
    # These failures occur before sending READ and legitimately have no bytes.
    if d['validity'] not in ('RX_FLOOD', 'INVALID_ARGUMENT'):
        require(d['validity'] == validity, 'PACKET_VALIDITY_DISAGREES')
    else:
        require(not raw, 'PRE_READ_ERROR_HAS_BYTES')
    valid = d['validity'] == 'VALID'
    require(integer(d, 'decoded_valid') == int(valid), 'DECODED_VALID_FLAG')
    payload = len(raw) >= 15
    require(integer(d, 'raw_payload') == int(payload), 'RAW_PAYLOAD_FLAG')
    values = (int.from_bytes(raw[7:9], 'little'), int.from_bytes(raw[11:13], 'little'), raw[6]) if payload else (-1, -1, -1)
    require(tuple(integer(d, 'raw_' + k) for k in ('position', 'speed', 'acc')) == values,
            'RAW_DECODE_DISAGREES')
    requested = tuple(integer(d, 'target_' + k) for k in ('position', 'speed', 'acc'))
    decoded = tuple(integer(d, k + '_read') for k in ('position', 'speed', 'acc'))
    require(decoded == (values if valid else (-1, -1, -1)), 'TRUSTED_DECODE_DISAGREES')
    mismatch = sum((1 << i) for i in range(3) if values[i] != requested[i]) if valid else 0
    require(integer(d, 'mismatch') == mismatch, 'MISMATCH_MASK_DISAGREES')
    return d


class Journal:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.path = self.directory / 'usb-bytes.jsonl'
        self.file = self.path.open('x', buffering=1)

    def event(self, kind, data):
        item = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    monotonic_ns=time.monotonic_ns(), kind=kind)
        if isinstance(data, bytes):
            item['base64'] = base64.b64encode(data).decode('ascii')
        else:
            item['data'] = data
        self.file.write(json.dumps(item, sort_keys=True) + '\n')
        self.file.flush()
        os.fsync(self.file.fileno())

    def save(self, name, data):
        path = self.directory / name
        temporary = path.with_suffix(path.suffix + '.tmp')
        with temporary.open('w') as f:
            json.dump(data, f, indent=2)
            f.write('\n'); f.flush(); os.fsync(f.fileno())
        os.replace(temporary, path)

    def close(self):
        self.file.close()


class RawUSB:
    """Exactly one open, exclusive; no modem-bit or baud-rate operation."""
    def __init__(self, port):
        import fcntl
        import termios
        self.fd = None
        try:
            self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            a = termios.tcgetattr(self.fd)
            a[0] = 0; a[1] = 0; a[3] = 0
            a[2] = (a[2] & ~termios.HUPCL) | termios.CLOCAL | termios.CREAD
            a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
        except BaseException:
            self.close()
            raise

    def read(self, timeout):
        if not select.select([self.fd], [], [], timeout)[0]:
            return b''
        data = os.read(self.fd, 65536)
        require(bool(data), 'USB_EOF_NO_REOPEN')
        return data

    def write(self, data):
        return os.write(self.fd, data)

    def close(self):
        if self.fd is not None:
            # Closing with HUPCL disabled. Kernel releases exclusivity on close.
            os.close(self.fd)
            self.fd = None


class Acquisition:
    def __init__(self, usb, journal, clock=time.monotonic):
        self.usb, self.journal, self.clock = usb, journal, clock
        self.buffer = b''; self.pending = []
        self.started = False; self.stop_sent = False; self.closed = False
        self.export_attempted = False
        self.boot_nonce = None; self.uptime = None; self.boot_banners = 0
        self.records = []; self.last_frame = None; self.terminal = None
        self.result = dict(status='IN_PROGRESS', source_commit=SOURCE,
                           application_sha256=APP_SHA, reset_commands=0,
                           start_commands=0, records=[], safety_errors=[])

    def send(self, command):
        require(command in (STATUS, STOP, '@SYSTEM SOURCE_SIGNATURE',
                '@STATUS',
                '@MOTION TELEMETRY 0', '@MOTION TELEMETRY 2', '@IMU STREAM OFF',
                '@BMS STREAM OFF', '@MOTION VALIDATION SENSORS', '@MOTION COMMISSIONING STATUS') or
                re.fullmatch(r'@MOTION VALIDATION PRIME_DIAG READ \d+', command) or
                re.fullmatch(r'@MOTION VALIDATION PRIME_DIAG START \d+ ' + START_TOKEN, command),
                'COMMAND_NOT_ALLOWLISTED')
        data = (command + '\n').encode('ascii')
        # INTENT is separate from TX; only successful write bytes are TX evidence.
        self.journal.event('TX_INTENT', command)
        n = self.usb.write(data)
        self.journal.event('TX', data[:n])
        require(n == len(data), 'SHORT_USB_WRITE_NO_RETRY')

    def inspect(self, line):
        if re.search(r'Guru Meditation|panic(?:ed|!)|Brownout detector|Rebooting|abort\(\)|assert failed|stack smashing|CORRUPT HEAP|Stack canary|watchdog got triggered', line, re.I):
            raise Refusal('CRITICAL_LOG:' + line)
        require(not self.started or not re.search(r'ESP-ROM:|rst:0x', line), 'UNEXPECTED_ROM_OR_RESET')
        if 'MATDOG Controller 0.2.0-dev.5' in line and not line.startswith('SOURCE_SIGNATURE'):
            self.boot_banners += 1
            require(self.boot_banners == 1 and not self.started, 'UNEXPECTED_BOOT')
        if 'GIT_SHA=' in line:
            require('GIT_SHA=' + SOURCE in line, 'WRONG_SOURCE')
        if line.startswith('PRIME_DIAG '):
            d = fields(line)
            require(d.get('mode') == '1' and d.get('torque_inhibit') == '1', 'DIAGNOSTIC_INHIBIT_LOST')
            require(d.get('session') in ('OFF', 'PRIMING', 'SAFE_OFF_REQUIRED'), 'ENERGIZE_OR_ARMED')
        if line.startswith('@TEL '):
            try:
                frame = json.loads(line[5:])
            except ValueError as exc:
                raise Refusal('INVALID_TELEMETRY_JSON') from exc
            if self.boot_nonce is not None:
                require(frame.get('boot_nonce') == self.boot_nonce, 'BOOT_NONCE_CHANGED')
                require(isinstance(frame.get('now_ms'), int) and frame['now_ms'] >= self.uptime, 'UPTIME_REGRESSED')
            self.boot_nonce = frame.get('boot_nonce'); self.uptime = frame.get('now_ms')
            require(isinstance(self.boot_nonce, int) and isinstance(self.uptime, int), 'BOOT_ID_MISSING')
            require(frame.get('local_session', {}).get('state') in ('OFF', 'PRIMING', 'SAFE_OFF_REQUIRED') and frame.get('executor_state') == 'OFF', 'ENERGIZE_OR_EXECUTOR_ACTIVE')
            for j in frame.get('servos', []):
                require(j.get('energized') == 0 and j.get('speed') == 0, 'TORQUE_OR_MOTION_PHYSICAL_CUTOFF')
            self.last_frame = frame

    def next_line(self, deadline):
        while self.clock() < deadline:
            if self.pending:
                line = self.pending.pop(0)
                self.journal.event('RX_LINE', line)
                self.inspect(line)
                return line
            data = self.usb.read(min(0.05, max(0, deadline - self.clock())))
            if data:
                self.journal.event('RX', data)
                self.buffer += data
                require(len(self.buffer) <= 131072, 'USB_UNTERMINATED_LINE_LIMIT')
                while b'\n' in self.buffer:
                    raw, self.buffer = self.buffer.split(b'\n', 1)
                    try:
                        self.pending.append(raw.decode('utf-8').rstrip('\r'))
                    except UnicodeError as exc:
                        raise Refusal('USB_INVALID_UTF8_RETAINED_RAW') from exc
        raise Refusal('RESPONSE_TIMEOUT_PARTIAL_RETAINED')

    def wait(self, predicate, timeout=3):
        deadline = self.clock() + timeout
        while True:
            line = self.next_line(deadline)
            if predicate(line):
                return line

    def exchange(self, command, prefix, timeout=3):
        self.send(command)
        return self.wait(lambda s: s.startswith(prefix), timeout)

    def status(self):
        d = fields(self.exchange(STATUS, 'PRIME_DIAG '))
        require(d.get('overflow') == '0' and integer(d, 'capacity') == 36 and
                0 <= integer(d, 'records') <= 36, 'RECORD_COUNT_OR_OVERFLOW')
        require(0 < integer(d, 'nonce') <= 0xffffffff, 'NONCE_INVALID')
        return d

    def stream_off(self):
        for command, prefix in (('@MOTION TELEMETRY 0', 'TELEMETRY=OFF'),
                                ('@IMU STREAM OFF', 'IMU_STREAM=OFF'),
                                ('@BMS STREAM OFF', 'BMS_STREAM=OFF')):
            self.exchange(command, prefix)

    def snapshot(self):
        self.last_frame = None
        self.exchange('@MOTION TELEMETRY 2', 'TELEMETRY=ON')
        self.wait(lambda s: s.startswith('@TEL '), 3)
        frame = self.last_frame
        self.exchange('@MOTION TELEMETRY 0', 'TELEMETRY=OFF')
        require(frame.get('fault') is False and frame.get('local_session', {}).get('state') == 'OFF', 'SNAPSHOT_NOT_SAFE_IDLE')
        require(frame.get('restore', {}).get('generation') == 1 and
                frame['restore'].get('fingerprint') == 'be8d5902bb4e37da' and
                frame.get('commissioning', {}).get('restore') == 'RESTORED', 'RESTORE_MISMATCH')
        servos = frame.get('servos', [])
        require(len(servos) == 12 and tuple(j.get('id') for j in servos) == IDS, 'SERVO_ID_VECTOR')
        require(all(j.get('mapped') is True and j.get('status') == 0 and
                    isinstance(j.get('age_ms'), int) and 0 <= j['age_ms'] <= 400 for j in servos), 'SERVO_FEEDBACK_INVALID')
        require(frame.get('channels', {}).get('raw', {}).get('count') == 12 and
                frame['channels'].get('mapped', {}).get('count') == 12 and
                frame['channels'].get('imu', {}).get('valid') is True, 'SENSOR_CHANNELS_INVALID')
        runtime = frame.get('runtime', {})
        require(all(isinstance(runtime.get(k), (int, float)) and runtime[k] > 0 for k in
                    ('heap_free', 'heap_min_free', 'loop_stack_min_free')), 'HEAP_STACK_INVALID')
        self.journal.save('snapshot-' + ('after' if self.started else 'before') + '.json', frame)
        return frame

    def stop(self):
        if not self.stop_sent:
            self.stop_sent = True
            self.send(STOP)

    def wait_safe_off(self, timeout=8):
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            d = self.status()
            if d['session'] == 'OFF':
                require(d.get('reason') == 'SAFE_OFF_VERIFIED', 'OFF_WITHOUT_SAFE_OFF_VERIFICATION')
                return d
            require(d['session'] in ('PRIMING', 'SAFE_OFF_REQUIRED'), 'UNSAFE_SESSION')
        raise Refusal('SAFE_OFF_TIMEOUT_NO_RESET')

    def export(self, d):
        require(not self.export_attempted, 'EXPORT_RETRY_FORBIDDEN')
        self.export_attempted = True
        n = integer(d, 'records')
        require(d['session'] == 'OFF', 'EXPORT_NOT_IDLE')
        self.terminal = d
        for i in range(n):
            line = self.exchange('@MOTION VALIDATION PRIME_DIAG READ ' + str(i), 'PRIME_RECORD ')
            record = validate_record(line, i)
            self.records.append(record)
            # Persist every successful record immediately, not only the final set.
            self.journal.save('records.json', self.records)
        end = self.status()
        require(end['session'] == 'OFF' and end['records'] == d['records'] and
                end['nonce'] == d['nonce'] and end.get('used') == '1', 'RECORD_SET_CHANGED')
        pairs = [(integer(r, 'slot'), integer(r, 'attempt')) for r in self.records]
        require(len(set(pairs)) == n, 'DUPLICATE_RECORD_ATTEMPT')
        for slot in range(12):
            attempts = [a for s, a in pairs if s == slot]
            require(attempts == list(range(1, len(attempts) + 1)), 'ATTEMPT_SEQUENCE')
        self.journal.save('terminal-status.json', end)
        return end

    def run(self):
        failure = None
        try:
            self.stream_off()
            self.send('@SYSTEM SOURCE_SIGNATURE')
            signature = self.wait(lambda s: s.startswith('SOURCE_SIGNATURE '))
            require('profile=ROBOT_POWERED' in signature and 'build_id=' + SOURCE[:12] in signature, 'SOURCE_SIGNATURE_MISMATCH')
            identity = fields(self.wait(lambda s: 'GIT_SHA=' in s))
            require(identity.get('GIT_SHA') == SOURCE and identity.get('GIT_DIRTY') == 'NO' and identity.get('HARDWARE_PROFILE') == 'ROBOT_POWERED', 'BUILD_IDENTITY')
            layout = fields(self.wait(lambda s: 'FLASH_LAYOUT=' in s))
            require(layout.get('FLASH_LAYOUT') == 'MATDOG_16M_2x5M_NVS_V1' and layout.get('OTA_INGEST') == '0', 'BUILD_LAYOUT')
            self.wait(lambda s: 'partition=app0 address=0x010000 size=0x500000' in s)
            census = fields(self.exchange('@STATUS', 'SERVO_POP '))
            require(census.get('last_census') == 'PASS' and census.get('expected_now') == '13',
                    'BOOT_CENSUS_NOT_PASS')
            frame = self.snapshot()
            require(frame.get('operational', {}).get('system_health') == 'READY', 'NOT_READY')
            sensors = fields(self.exchange('@MOTION VALIDATION SENSORS', 'SENSORS '))
            require(sensors.get('boot_safe_off_pending') == '0' and sensors.get('power') == 'RUN' and sensors.get('fault') == '0' and sensors.get('imu_valid') == '1' and sensors.get('bms_valid') == '1', 'BOOT_POWER_SENSOR_GATE')
            now = integer(sensors, 'uptime_ms')
            require(0 <= now - integer(sensors, 'bms_stamp_ms') <= 3000 and
                    0 <= now - integer(sensors, 'imu_stamp_ms') <= 400, 'SENSORS_STALE')
            before = self.status()
            require(before.get('used') == '0' and before['records'] == '0' and before['session'] == 'OFF', 'FRESH_TRIAL_REQUIRED_NO_CLEAR')
            self.started = True
            self.result['start_commands'] = 1
            self.journal.save('receipt.json', self.result)
            self.send('@MOTION VALIDATION PRIME_DIAG START ' + before['nonce'] + ' ' + START_TOKEN)
            response = fields(self.wait(lambda s: s.startswith('VALIDATION=')))
            require(response.get('VALIDATION') == 'PASS' and response.get('REASON') == 'PRIME_DIAGNOSTIC_STARTED_NO_ENERGIZE', 'START_REFUSED_NO_RETRY')
            end = self.wait_safe_off()
            self.export(end)
            self.snapshot()
            require(end.get('last_revoke') == 'PRIME_DIAGNOSTIC_COMPLETE_NO_ENERGIZE' or
                    (bool(self.records) and end.get('last_revoke', '').startswith('PRIME_')), 'INCONCLUSIVE_TERMINATION')
            self.result['status'] = 'PASS_ACQUISITION'
            self.result['prime_result'] = 'COMPLETE_NO_ENERGIZE' if end.get('last_revoke') == 'PRIME_DIAGNOSTIC_COMPLETE_NO_ENERGIZE' else 'REJECTION_CAPTURED'
        except BaseException as exc:
            failure = exc
            self.result['status'] = 'FAIL'
            self.result['error'] = str(exc)
            self.journal.event('ERROR', str(exc))
            if self.started:
                try:
                    self.stop()
                    end = self.wait_safe_off()
                    # Do not overwrite a partially saved export or repeat START.
                    if not self.export_attempted:
                        self.export(end)
                except BaseException as recovery:
                    self.result['recovery_error'] = str(recovery)
                    self.journal.event('RECOVERY_ERROR', str(recovery))
        finally:
            try:
                self.stream_off()
            except BaseException as cleanup:
                self.result['cleanup_error'] = str(cleanup)
                self.result['status'] = 'FAIL'
            self.result['records'] = self.records
            self.result['terminal'] = self.terminal
            self.result['unparsed_partial_bytes'] = len(self.buffer)
            self.journal.save('receipt.json', self.result)
            self.usb.close(); self.closed = True
            self.journal.event('USB_CLOSED', 'NO_RESET_NO_REOPEN')
            self.journal.save('log-digest.json', {'sha256': hashlib.sha256(self.journal.path.read_bytes()).hexdigest()})
            self.journal.close()
        if failure:
            raise Refusal(str(failure)) from failure
        require(self.result['status'] == 'PASS_ACQUISITION', 'CLEANUP_FAILED')
        return self.result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--port', default=PORT)
    p.add_argument('--physical-check', type=Path, required=True,
                   help='fresh operator setup record; separate flash authorization remains valid')
    a = p.parse_args()
    physical = json.loads(a.physical_check.read_text())
    require(physical.get('fixture') is True and physical.get('operator_present') is True and
            physical.get('key_on') is True and physical.get('cutoff_accessible') is True,
            'CURRENT_LOCAL_PHYSICAL_SETUP_REQUIRED')
    stamp = datetime.datetime.fromisoformat(physical['utc'])
    require(0 <= (datetime.datetime.now(datetime.timezone.utc) - stamp).total_seconds() <= 300,
            'PHYSICAL_CHECK_STALE')
    def interrupt(signum, frame):
        raise Refusal('LOCAL_INTERRUPT_STOP_NO_RESET')
    signal.signal(signal.SIGINT, interrupt); signal.signal(signal.SIGTERM, interrupt)
    journal = Journal(a.output)
    journal.event('PHYSICAL_CHECK', physical)
    try:
        usb = RawUSB(a.port)
    except BaseException as exc:
        journal.event('OPEN_FAILED_NO_RETRY', str(exc)); journal.close(); raise
    journal.event('USB_OPENED', 'ONE_OPEN_EXCLUSIVE_NO_MODEM_CONTROL')
    Acquisition(usb, journal).run()


if __name__ == '__main__':
    main()
