#!/usr/bin/env python3
"""Process registry of a MATDOG hardware session (after the 16/11 MEMSTOP: a stray http_watch.py survived because the
PID saved for it was the forking `setsid`, not the watcher; three HTTP clients then read the robot together).

Every recorder, watcher, fixture tool and guard is started here with subprocess.Popen (start_new_session=True): the PID
recorded is the process itself, checked against /proc/<pid>/cmdline. Each start refuses when a live session process
of the same role already exists (one USB owner, one HTTP watcher, one fixture tool, one guard), and when any live session
process is not in the registry. Processes are stopped only through their registered PID (the USB recorder through its
own QUIT first), and the stop is verified: the PID is gone, or /proc shows another program under it (reuse).
Never a kill by pattern: the /proc scan only lists.

Usage: session_proc.py start <name> <role> <run-dir> <console> [--also <exec'd program>] -- <cmd...>
       session_proc.py stop <name>
       session_proc.py check [--expect-none]
       session_proc.py list
Roles: usb (usb_session.py recorder; stop = ctl.py <run> QUIT), watch, tool, guard, other."""
import datetime, json, os, signal, subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REG = ROOT / 'procs'
NAMES = ('http_watch.py', 'usb_session.py', 'fixture_suite', 'fixture_settle.py', 'fixture_blocks.py', 'fixture_block_h.py',
         'heap_guard.py', 'monitor.py', 'preflight.py', 'cold_start.sh', 'boot_and_record', 'run_acquisition.py')
SINGLE = ('usb', 'watch', 'tool', 'guard')


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')


def cmdline(pid):
    try:
        return Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
    except OSError:
        return None


def ident(argv):
    """The program's identity: its first .py/.sh argument (python3 -I x.py, bash x.sh), else argv[0]."""
    return next((os.path.basename(x) for x in argv if x.endswith(('.py', '.sh'))), os.path.basename(argv[0]))


def alive_as(pid, argv, also=()):
    """True while <pid> runs the registered program, or the program it exec's into (also=, e.g. a boot script that
    ends with exec python3 usb_session.py keeps its PID)."""
    c = cmdline(pid)
    if not c:
        return False
    names = {os.path.basename(x.decode(errors='replace')) for x in c}
    return ident(argv) in names or any(a in names for a in also)


def ancestors():
    out, pid = set(), os.getpid()
    while pid > 1:
        out.add(pid)
        try:
            pid = int(Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def entries():
    REG.mkdir(exist_ok=True)
    return [json.loads(p.read_text()) for p in sorted(REG.glob('*.json'))]


def save(e):
    REG.mkdir(exist_ok=True)
    (REG / (e['name'] + '.json')).write_text(json.dumps(e, indent=1))
    (REG / (e['name'] + '.pid')).write_text('%d\n' % e['pid'])


def live_entries():
    return [e for e in entries() if not e.get('closed_utc') and alive_as(e['pid'], e['argv'], e.get('also', ()))]


def scan():
    """Every live process whose command line names a session tool (listing only); the caller and its parents
    (e.g. cold_start.sh starting its recorder through this file) are not listed."""
    out, mine = [], ancestors()
    for d in Path('/proc').iterdir():
        if not d.name.isdigit() or int(d.name) in mine:
            continue
        c = cmdline(int(d.name))
        if not c or not c[0]:
            continue
        args = [x.decode(errors='replace') for x in c if x]
        if os.path.basename(args[0]) in ('python3', 'bash', 'sh') and any(any(n in a for n in NAMES) for a in args[1:3]):
            if 'session_proc.py' not in ' '.join(args[:2]):
                out.append((int(d.name), args))
    return out


def tty_holders():
    out = []
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():
            continue
        try:
            for fd in (d / 'fd').iterdir():
                try:
                    if os.readlink(fd).startswith('/dev/ttyACM'):
                        out.append(int(d.name))
                        break
                except OSError:
                    pass
        except OSError:
            pass
    return out


def check(expect_none=False, quiet=False):
    reg = {e['pid']: e for e in live_entries()}
    found = scan()
    problems = []
    for pid, args in found:
        if pid not in reg:
            problems.append('UNREGISTERED pid %d: %s' % (pid, ' '.join(args)[:160]))
    roles = {}
    for e in reg.values():
        roles.setdefault(e['role'], []).append(e['name'])
    for r, n in roles.items():
        if r in SINGLE and len(n) > 1:
            problems.append('ROLE %s held by %d processes: %s' % (r, len(n), n))
    if expect_none and (reg or found):
        problems.append('EXPECTED NONE, live: %s' % sorted({e['name'] for e in reg.values()} | {str(p) for p, _ in found if p not in reg}))
    holders = [p for p in tty_holders() if p not in reg and p not in ancestors()]
    if holders:
        problems.append('USB port held by unregistered pid(s) %s' % holders)
    if not quiet:
        for e in reg.values():
            print('LIVE %-28s role %-5s pid %d run %s' % (e['name'], e['role'], e['pid'], e['run']))
        for p in problems:
            print('PROBLEM', p)
        print('CHECK', 'PASS' if not problems else 'FAIL')
    return not problems


def start(name, role, run, console, argv, also=()):
    if (REG / (name + '.json')).exists():
        raise SystemExit('REFUSE: %s already registered' % name)
    if not check(quiet=True):
        check()
        raise SystemExit('REFUSE: session processes not clean')
    if role in SINGLE and any(e['role'] == role for e in live_entries()):
        raise SystemExit('REFUSE: a live %s process exists' % role)
    with open(ROOT / console, 'a') as out:
        p = subprocess.Popen(argv, cwd=str(ROOT), stdout=out, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             start_new_session=True)
    e = {'name': name, 'role': role, 'run': run, 'pid': p.pid, 'argv': argv, 'also': list(also), 'console': console, 'start_utc': utc()}
    save(e)
    time.sleep(0.3)
    if p.poll() is None and not alive_as(p.pid, argv, also):
        raise SystemExit('REFUSE: pid %d does not run %s' % (p.pid, argv))
    print('STARTED %s role %s pid %d' % (name, role, p.pid))
    return p, e


def stop(name, timeout=10.0):
    e = json.loads((REG / (name + '.json')).read_text())
    if e.get('closed_utc'):
        print('ALREADY CLOSED', name)
        return True
    pid, argv, also = e['pid'], e['argv'], e.get('also', ())
    if alive_as(pid, argv, also):
        if e['role'] == 'usb':
            subprocess.run([sys.executable, '-I', str(ROOT / 'ctl.py'), e['run'], 'QUIT'], cwd=str(ROOT), check=False)
        else:
            os.kill(pid, signal.SIGTERM)
        t0 = time.time()
        while time.time() - t0 < timeout and alive_as(pid, argv, also):
            time.sleep(0.1)
        if alive_as(pid, argv, also) and e['role'] in ('watch', 'guard', 'other'):
            os.kill(pid, signal.SIGKILL)          # read-only programs only, through the registered PID
            time.sleep(0.5)
    ok = not alive_as(pid, argv, also)
    e['closed_utc'] = utc() if ok else None
    e['close_verified'] = ok
    save(e)
    print('%s %s pid %d' % ('CLOSED' if ok else 'STILL ALIVE', name, pid))
    return ok


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        return 2
    if a[0] == 'start':
        k = a.index('--')
        also = a[a.index('--also') + 1].split(',') if '--also' in a[:k] else ()
        start(a[1], a[2], a[3], a[4], a[k + 1:], also)
        return 0
    if a[0] == 'stop':
        return 0 if stop(a[1]) else 1
    if a[0] == 'check':
        return 0 if check(expect_none='--expect-none' in a) else 1
    if a[0] == 'list':
        for e in entries():
            print(json.dumps(e))
        return 0
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main())
