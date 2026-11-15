"""The emulation's sockets and process, probed.

A console's answer, a bootloader's, bytes until a pattern or a quiet, and Renode found, tied to
this process's life and bound to cores of its own (`Cored`).
"""
import contextlib
import ctypes
import glob
import os
import re
import shutil
import socket
import time

from coaxial.comm import protocol
from coaxial.comm.protocol import BootOp
from coaxial.devices.boot import BLANK_UNIT
from tools.emu.protocol_frames import frame

#: The monitor's colour codes, stripped before its output is read.
ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


class _Limits(ctypes.Structure if os.name == 'nt' else object):
    """JOBOBJECT_EXTENDED_LIMIT_INFORMATION, its basic limits and I/O counters inline."""
    if os.name == 'nt':
        _fields_ = [('per_process_time', ctypes.c_int64), ('per_job_time', ctypes.c_int64),
                    ('flags', ctypes.c_uint32), ('min_ws', ctypes.c_size_t),
                    ('max_ws', ctypes.c_size_t), ('processes', ctypes.c_uint32),
                    ('affinity', ctypes.c_size_t), ('priority', ctypes.c_uint32),
                    ('scheduling', ctypes.c_uint32), ('io', ctypes.c_uint64 * 6),
                    ('process_memory', ctypes.c_size_t), ('job_memory', ctypes.c_size_t),
                    ('peak_process', ctypes.c_size_t), ('peak_job', ctypes.c_size_t)]


def tied(process):
    """On Windows, a job that ends `process` when this one ends, killed or not - a script
    killed mid-run left its Renode running for hours. The job's handle, kept."""
    if os.name != 'nt':
        return None
    kernel = ctypes.windll.kernel32
    job = kernel.CreateJobObjectW(None, None)
    limits = _Limits()
    limits.flags = 0x2400                                  # KILL_ON_JOB_CLOSE | DIE_ON_UNHANDLED_EXCEPTION
    kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits))
    kernel.AssignProcessToJobObject(job, ctypes.c_void_p(int(process._handle)))
    return job


def find_renode():
    """Renode's executable, or None."""
    named = os.environ.get('RENODE') or shutil.which('renode')
    if named and os.path.exists(named):
        return named
    local = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'renode', 'renode_*', 'renode.exe')
    found = sorted(glob.glob(local))
    return found[-1] if found else None


class Cored:
    """Renode `pid`, a CPU thread a machine (docs/findings/emulation.md), on `machines` cores while it runs.
    Left where Renode's host is held - a relay's job on one core, a loop on a core of its own - Renode's threads
    and the host shared that core's processors and the boards answered late: a limb of three a one-byte NoReply,
    one board at full pace a CrcError. Renode's cores are those the relay sent along with a job whose suite
    asked (`focus.SENT`, or `sent`, a mask). With none sent - a page's, a flight's, a tool's - Renode takes so
    many where they are free (`placed.claim`), else any of the processors given, and books them in the
    machine's ledger (or `ledger`) while it runs: the relays count them."""

    def __init__(self, pid, machines, port, sent=None, ledger=None):
        from machine.process import placed
        from tools.dev import focus
        cores = [mask for _klass, mask in placed.topology()[0]]
        sent = int(os.environ.get(focus.SENT, '0'), 16) if sent is None else sent
        self.holds, self.booking = [], None
        if sum(1 for core in cores if core & sent == core) >= machines:
            placed.bind(pid, sent)
            return
        claims = [placed.claim() for _ in range(machines)]
        if all(mask for mask, _held in claims):
            self.holds = [held for _mask, held in claims]
        else:
            for _mask, held in claims:
                placed.free(held)
        placed.bind(pid, sum(mask for mask, _held in claims) if self.holds else sum(cores))
        self.booking = os.path.join(ledger or focus.LEDGER, '%d-renode-%d' % (os.getpid(), port))
        # Laid whole (`booked`), outside the ledger's lock: every relay's batons hold it in turn, and a page's
        # emulator would wait seconds behind them.
        with contextlib.suppress(OSError):
            focus.booked(self.booking, 0.0, pid, os.getpid(), machines)

    def free(self):
        """Renode's cores let go and the booking gone."""
        from machine.process import placed
        for held in self.holds:
            placed.free(held)
        self.holds = []
        if self.booking is not None:
            with contextlib.suppress(OSError):
                os.remove(self.booking)
            self.booking = None


#: The latest run's log in a Renode's work, the only one kept (`swept`).
LAST_LOG = 'renode_last.log'


def swept(work, port, log=None):
    """What the run on `port` left in `work` gone - its script, config and its lock, history and temp - and its
    own log `log` kept as the latest run's alone (LAST_LOG): every run's kept, Renode's work came to 15 GB over
    725 runs, five of their logs past 1 GB."""
    for name in ('run_%d.resc', 'renode_%d.config', 'renode_%d.config.lock', 'history_%d'):
        with contextlib.suppress(OSError):
            os.remove(os.path.join(work, name % port))
    shutil.rmtree(os.path.join(work, 'temp_%d' % port), ignore_errors=True)
    if log:
        with contextlib.suppress(OSError):
            os.replace(log, os.path.join(work, LAST_LOG))


def own_temp(path):
    """`path` made an empty directory: a Renode's temp, TMP and TEMP for it. Compiling its first
    plugin Renode sweeps the temp's renode-<pid> of runs gone, asking whether each pid lives - one
    since a protected process's refused it, and Renode died."""
    shutil.rmtree(path, ignore_errors=True)
    os.makedirs(path)
    return dict(os.environ, TMP=path, TEMP=path)


def answers(port):
    """Whether the console on `port` answers its help key."""
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            s.settimeout(0.5)
            for _ in range(10):
                s.sendall(frame(b'?'))
                if b'commands:' in heard(s, 0.5):
                    return True
    except OSError:
        pass
    return False


def boot_answers(port):
    """Whether a bootloader on `port` answers device 11's state as a blank node."""
    from machine.rig.rtu import crc16

    body = bytes([BLANK_UNIT, 0x6E, protocol.DEVICE_BOOT, BootOp.STATE])
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=1.0) as s:
            s.settimeout(0.5)
            s.sendall(frame(body + crc16(body).to_bytes(2, 'little')))
            return heard(s, 2.0)[:2] == body[:2]
    except OSError:
        return False


def heard_until(s, pattern, seconds):
    """Bytes until `pattern` matches the tail, or `seconds` pass."""
    got = b''
    end = time.time() + seconds
    while time.time() < end and not pattern.search(ANSI.sub('', got.decode('utf-8', 'replace'))):
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
    return got


def heard_quiet(s, quiet, seconds):
    """Bytes until none arrive for `quiet` seconds after the first, or `seconds` pass."""
    got = b''
    end = time.time() + seconds
    last = end
    while time.time() < end and not (got and time.time() - last > quiet):
        try:
            chunk = s.recv(65536)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
        last = time.time()
    return got


def heard(s, seconds):
    """Bytes for `seconds`, or until the socket closes."""
    got = b''
    end = time.time() + seconds
    while time.time() < end:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        got += chunk
    return got
