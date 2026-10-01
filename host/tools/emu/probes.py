"""The emulation's sockets and process, probed.

A console's answer, a bootloader's, bytes until a pattern or a quiet, and Renode tied to this
process's life.
"""
import ctypes
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
    killed mid-run left its Renode running for hours (2026-09-25). Its handle, kept."""
    if os.name != 'nt':
        return None
    kernel = ctypes.windll.kernel32
    job = kernel.CreateJobObjectW(None, None)
    limits = _Limits()
    limits.flags = 0x2000                                  # KILL_ON_JOB_CLOSE
    kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits))
    kernel.AssignProcessToJobObject(job, ctypes.c_void_p(int(process._handle)))
    return job


def own_temp(path):
    """`path` made an empty directory: a Renode's temp, TMP and TEMP for it. Compiling its first
    plugin Renode sweeps the temp's renode-<pid> of runs gone, asking whether each pid lives - one
    since a protected process's refused it, and Renode died (2026-10-02)."""
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
    from machine.rtu import crc16

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
