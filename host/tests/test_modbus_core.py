#!/usr/bin/env python3
"""The portable Modbus core, on this machine, with no board and no cable.

Invariant 1 keeps `modbus_crc.c`, `modbus_slave.c` and `modbus_rtu.c` free of
HAL, CMSIS and every hardware header, and says that is what makes them
host-testable. Nothing tested them: their only verification was
test_conformance.py, which needs a board on the other end of a serial link.
The framing state machine, the span checks and the half-written-register guard
- the code where a defect does the most damage - were the least covered in the
repository.

This builds them with the host gcc, together with `modbus/test/harness.c`, and
drives the result through ctypes. The clock is injected, so t1.5, t3.5 and the
2^32 wrap are tested by arithmetic rather than by waiting.

A missing compiler is not a failing suite, the same way a missing cable is not:
it says what it skipped and passes nothing. `setup.ps1` installs one.

Run from the host directory:  python tests/test_modbus_core.py
"""
import ctypes
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
REPO = os.path.dirname(HOST)
CORE = os.path.join(REPO, 'modbus')
OUT = os.path.join(REPO, 'build', 'hosttest')

# The oracle's Python mirror lives in coaxial.protocol. On a bench with the
# editable install this resolves anyway; a fresh clone (the CI runner) has
# only what the suite puts on the path - and this suite, unlike its
# siblings, never had to before it grew a mirror to hold the C to.
sys.path.insert(0, HOST)

SOURCES = [os.path.join(CORE, 'test', 'harness.c'),
           os.path.join(CORE, 'src', 'modbus_crc.c'),
           os.path.join(CORE, 'src', 'modbus_slave.c'),
           os.path.join(CORE, 'src', 'modbus_rtu.c'),
           # The request-length oracle rides along: the harness installs
           # it on the RTU under test, and the suite drives it directly
           # for the prefix sweep. The stub stands in for the dispatch
           # tables, which live beside HAL handlers.
           os.path.join(REPO, 'comms', 'src', 'cmd_length.c'),
           os.path.join(REPO, 'comms', 'test', 'cmd_find_stub.c')]

INCLUDES = [os.path.join(CORE, 'inc'), os.path.join(REPO, 'comms', 'inc'),
            os.path.join(REPO, 'board', 'inc')]

# The same warnings the firmware build puts on these three files. A host
# compiler is a second opinion on them, not just a way to run them.
FLAGS = ['-std=c11', '-O1', '-Wall', '-Wextra', '-Wconversion', '-Wshadow']

EX = {0: 'NONE', 1: 'ILLEGAL FUNCTION', 2: 'ILLEGAL DATA ADDRESS',
      3: 'ILLEGAL DATA VALUE', 4: 'SERVER DEVICE FAILURE'}


def find_cc():
    """A host C compiler, or None. PATH first, then where winget puts one."""
    from shutil import which
    found = which('gcc') or which('clang') or which('cc')
    if found:
        return found
    packages = os.path.join(os.environ.get('LOCALAPPDATA', ''),
                            'Microsoft', 'WinGet', 'Packages')
    if not os.path.isdir(packages):
        return None
    for root, _dirs, files in os.walk(packages):
        if 'gcc.exe' in files and root.endswith(os.path.join('mingw64', 'bin')):
            return os.path.join(root, 'gcc.exe')
    return None


def build(cc, sources=None, includes=None, name='mbcore'):
    """(path, warnings) for a shared library, built fresh every run.

    Takes its sources so the SHTP suite can reuse it rather than copy the
    compiler discovery and the flag list - there is one answer to "how is
    portable C built for a test on this machine".
    """
    os.makedirs(OUT, exist_ok=True)
    lib = os.path.join(OUT, name + ('.dll' if os.name == 'nt' else '.so'))
    flags = []
    for path in (includes or INCLUDES):
        flags += ['-I', path]
    done = subprocess.run([cc, '-shared', '-o', lib] + FLAGS +
                          (sources or SOURCES) + flags,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace')
    if done.returncode != 0:
        raise RuntimeError(done.stderr.strip()[-2000:])
    return lib, [l for l in (done.stderr or '').splitlines() if 'warning:' in l]


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-58s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-58s %s' % (name, detail))


class Core:
    """One harness instance: a slave, its bank, and an RTU transport."""

    def __init__(self, lib):
        self.lib = lib
        lib.mbh_new.restype = ctypes.c_void_p
        self.h = ctypes.c_void_p(lib.mbh_new())

    def close(self):
        self.lib.mbh_free(self.h)

    def execute(self, req, cap=253):
        buf = (ctypes.c_ubyte * max(cap, 1))()
        n = self.lib.mbh_execute(self.h, bytes(req), ctypes.c_size_t(len(req)),
                                 buf, ctypes.c_size_t(cap))
        return bytes(buf[:n])

    def exception(self, req, cap=253):
        """(fc, code) for an exception reply, or (None, None) for anything else."""
        rsp = self.execute(req, cap)
        if len(rsp) == 2 and rsp[0] & 0x80:
            return rsp[0] & 0x7F, rsp[1]
        return None, None

    # -- the model's own switches ----------------------------------------
    def drop(self, which):
        self.lib.mbh_drop(self.h, ctypes.c_int(which))

    def accept_all(self, on=1):
        self.lib.mbh_accept_all(self.h, ctypes.c_int(on))

    def fail_write_at(self, addr):
        self.lib.mbh_fail_write_at(self.h, ctypes.c_int(addr))

    def bad_value(self, value):
        self.lib.mbh_bad_value(self.h, ctypes.c_int(value))

    def set_id(self, text):
        self.lib.mbh_set_id(self.h, text.encode())

    def hold(self, addr):
        self.lib.mbh_hold.restype = ctypes.c_uint16
        return self.lib.mbh_hold(self.h, ctypes.c_uint16(addr))

    def coil(self, addr):
        self.lib.mbh_coil.restype = ctypes.c_ubyte
        return self.lib.mbh_coil(self.h, ctypes.c_uint16(addr))

    # -- the transport ----------------------------------------------------
    def rtu_init(self, unit=1, baud=115200, bits=11, ticks_per_us=475):
        self.lib.mbh_rtu_init(self.h, ctypes.c_ubyte(unit), ctypes.c_uint32(baud),
                              ctypes.c_ubyte(bits), ctypes.c_uint32(ticks_per_us))

    def byte(self, b, ticks):
        self.lib.mbh_rtu_byte(self.h, ctypes.c_ubyte(b), ctypes.c_uint32(ticks))

    def rtu_error(self, ticks):
        self.lib.mbh_rtu_error(self.h, ctypes.c_uint32(ticks))

    def busy(self):
        return bool(self.lib.mbh_rtu_busy(self.h))

    def t35(self):
        self.lib.mbh_rtu_t35.restype = ctypes.c_uint32
        return self.lib.mbh_rtu_t35(self.h)

    def t15(self):
        self.lib.mbh_rtu_t15.restype = ctypes.c_uint32
        return self.lib.mbh_rtu_t15(self.h)

    def service(self, ticks):
        buf = (ctypes.c_ubyte * 256)()
        n = self.lib.mbh_rtu_service(self.h, ctypes.c_uint32(ticks), buf,
                                     ctypes.c_size_t(256))
        return bytes(buf[:n])

    def hint(self, on):
        self.lib.mbh_rtu_hint(self.h, ctypes.c_int(1 if on else 0))

    def counters(self):
        six = (ctypes.c_uint32 * 6)()
        self.lib.mbh_rtu_counters(self.h, six)
        return dict(zip(('bus_message', 'bus_comm_error', 'server_message',
                         'server_exception', 'server_no_response',
                         'char_overrun'), list(six)))

    def feed(self, frame, start=1000000, step=100):
        """A whole frame at line rate; returns the last character's tick.

        The last character, not one step past it: the silence that ends a
        frame is measured from the last byte, and returning the step after
        it put every "just before t3.5" check a hundred ticks the wrong side
        of the boundary it was testing.
        """
        for i, b in enumerate(frame):
            self.byte(b, (start + i * step) & 0xFFFFFFFF)
        return (start + (len(frame) - 1) * step) & 0xFFFFFFFF

    def crc(self, data):
        self.lib.mbh_crc16.restype = ctypes.c_uint16
        return self.lib.mbh_crc16(bytes(data), ctypes.c_size_t(len(data)))

    def adu(self, unit, pdu):
        body = bytes([unit]) + bytes(pdu)
        return body + self.crc(body).to_bytes(2, 'little')


def test_crc(report, lib):
    """The standard CRC-16/MODBUS check value, then the round trip."""
    core = Core(lib)
    report.check('CRC-16/MODBUS of "123456789" is 0x4B37',
                 core.crc(b'123456789') == 0x4B37,
                 hex(core.crc(b'123456789')))

    frame = bytearray(b'\x01\x03\x00\x00\x00\x02')
    good = bytes(frame) + core.crc(frame).to_bytes(2, 'little')
    report.check('a frame with its own CRC appended checks out',
                 lib.mbh_crc_check(good, ctypes.c_size_t(len(good))) != 0)

    bad = bytearray(good)
    bad[3] ^= 0x01
    report.check('one flipped bit fails the check',
                 lib.mbh_crc_check(bytes(bad), ctypes.c_size_t(len(bad))) == 0)
    core.close()


def test_quantities(report, lib):
    """Illegal quantities are a value error, never an address error.

    The distinction is modbus_slave.c's own: the request is badly formed and
    the addresses were never consulted.
    """
    core = Core(lib)
    for name, pdu in (('read holding, qty 0', b'\x03\x00\x00\x00\x00'),
                      ('read holding, qty 126', b'\x03\x00\x00\x00\x7E'),
                      ('read input, qty 126', b'\x04\x00\x00\x00\x7E'),
                      ('read coils, qty 0', b'\x01\x00\x00\x00\x00'),
                      ('read coils, qty 2001', b'\x01\x00\x00\x07\xD1'),
                      ('read discrete, qty 2001', b'\x02\x00\x00\x07\xD1')):
        _fc, code = core.exception(pdu)
        report.check('%s -> ILLEGAL DATA VALUE' % name, code == 3,
                     EX.get(code, code))
    core.close()


def test_span(report, lib):
    """A span straddling 0xFFFF must not wrap past the range check.

    Computed in 32 bits on purpose - in 16 it would wrap to a small number
    and the request would sail through.
    """
    # With the model saying yes to every address, span_overflows is the
    # only thing left that can refuse this - which is the point. Measured:
    # with the harness applying its own 32-bit check first, the engine
    # rewritten to check in 16 bits still passed this test, and only the
    # compiler warning caught it.
    wide = Core(lib)
    wide.accept_all()
    _fc, code = wide.exception(b'\x03\xFF\xFF\x00\x02')
    report.check('addr 0xFFFF qty 2, with every address acceptable to '
                 'the model -> ILLEGAL DATA ADDRESS', code == 2,
                 EX.get(code, code))
    rsp = wide.execute(b'\x03\x00\x00\x00\x02')
    report.check('and a span that does not straddle is still answered',
                 len(rsp) == 6, rsp.hex(' '))
    wide.close()

    core = Core(lib)
    _fc, code = core.exception(b'\x03\xFF\xFF\x00\x02')
    report.check('addr 0xFFFF qty 2 -> ILLEGAL DATA ADDRESS', code == 2,
                 EX.get(code, code))
    _fc, code = core.exception(b'\x03\x00\x3C\x00\x08')
    report.check('addr 60 qty 8 runs off a 64-entry bank -> DATA ADDRESS',
                 code == 2, EX.get(code, code))
    _fc, code = core.exception(b'\x03\x00\x3C\x00\x04')
    report.check('addr 60 qty 4 fits it exactly -> answered', code is None,
                 EX.get(code, ''))
    core.drop(7)
    _fc, code = core.exception(b'\x03\x00\x00\x00\x02')
    report.check('a model with no validate_range refuses every address',
                 code == 2, EX.get(code, code))
    core.close()


def test_reads(report, lib):
    """Layout: big-endian registers, LSB-first bits, byte count first."""
    core = Core(lib)
    rsp = core.execute(b'\x03\x00\x00\x00\x03')
    report.check('read holding 0..2 returns fc, byte count, then the data',
                 rsp[:2] == b'\x03\x06' and len(rsp) == 8, rsp.hex(' '))
    report.check('and the registers are big-endian, 0x1000 upward',
                 rsp[2:] == b'\x10\x00\x10\x01\x10\x02', rsp[2:].hex(' '))

    rsp = core.execute(b'\x04\x00\x00\x00\x02')
    report.check('input registers come from their own table',
                 rsp == b'\x04\x04\x20\x00\x20\x01', rsp.hex(' '))

    rsp = core.execute(b'\x01\x00\x00\x00\x0A')
    report.check('ten alternating coils pack LSB-first into two bytes',
                 rsp == b'\x01\x02\xAA\x02', rsp.hex(' '))
    core.close()


def test_capacity(report, lib):
    """A response that will not fit the buffer given is the server's failure.

    Unreachable over RTU, which always passes 253 bytes - but the signature
    promises a capacity, and until it was checked eight of the nine handlers
    discarded it.
    """
    core = Core(lib)
    rsp = core.execute(b'\x03\x00\x00\x00\x03', cap=253)
    report.check('a read that fits is answered', len(rsp) == 8, rsp.hex(' '))
    _fc, code = core.exception(b'\x03\x00\x00\x00\x03', cap=6)
    report.check('the same read into six bytes -> SERVER DEVICE FAILURE',
                 code == 4, EX.get(code, code))
    _fc, code = core.exception(b'\x01\x00\x00\x00\x28', cap=6)
    report.check('forty coils into six bytes -> SERVER DEVICE FAILURE',
                 code == 4, EX.get(code, code))
    report.check('a capacity below the smallest exception is met with silence',
                 core.execute(b'\x03\x00\x00\x00\x03', cap=4) == b'')
    core.close()


def test_writes(report, lib):
    """Echoes, and the two ways a write is refused."""
    core = Core(lib)
    rsp = core.execute(b'\x05\x00\x00\xFF\x00')
    report.check('write single coil echoes the request verbatim',
                 rsp == b'\x05\x00\x00\xFF\x00' and core.coil(0) == 1,
                 rsp.hex(' '))
    _fc, code = core.exception(b'\x05\x00\x00\x12\x34')
    report.check('a coil value neither 0x0000 nor 0xFF00 -> DATA VALUE',
                 code == 3, EX.get(code, code))

    rsp = core.execute(b'\x06\x00\x05\xBE\xEF')
    report.check('write single register echoes and stores',
                 rsp == b'\x06\x00\x05\xBE\xEF' and core.hold(5) == 0xBEEF,
                 hex(core.hold(5)))

    _fc, code = core.exception(b'\x10\x00\x00\x00\x02\x03\x00\x01\x00\x02')
    report.check('a byte count contradicting the quantity -> DATA VALUE',
                 code == 3, EX.get(code, code))
    _fc, code = core.exception(b'\x10\x00\x00\x00\x02\x04\x00\x01')
    report.check('a byte count contradicting the frame length -> DATA VALUE',
                 code == 3, EX.get(code, code))
    _fc, code = core.exception(b'\x0F\x00\x00\x00\x00\x00')
    report.check('zero coils is a well-formed 6-byte PDU and must reach its '
                 'handler, not be met with silence', code == 3,
                 EX.get(code, code))
    core.close()


def test_half_write(report, lib):
    """The guard that stops a multi-register write applying a prefix.

    Both halves, because the second proves the first is doing the work: with
    validate_reg_value wired a bad value leaves every register untouched;
    with it dropped the registers before the failure are written and the
    client still sees one exception for the whole request.
    """
    req = b'\x10\x00\x00\x00\x03\x06\xAA\xAA\xBB\xBB\xDE\xAD'

    guarded = Core(lib)
    guarded.bad_value(0xDEAD)
    _fc, code = guarded.exception(req)
    report.check('a bad value anywhere in the span refuses the whole write',
                 code == 3, EX.get(code, code))
    report.check('and nothing before it was applied',
                 guarded.hold(0) == 0x1000 and guarded.hold(1) == 0x1001,
                 '%04X %04X' % (guarded.hold(0), guarded.hold(1)))
    guarded.close()

    unguarded = Core(lib)
    unguarded.drop(4)
    unguarded.fail_write_at(2)
    _fc, code = unguarded.exception(req)
    report.check('without the guard the same shape leaves the device half '
                 'written - which is why the guard exists',
                 code == 4 and unguarded.hold(0) == 0xAAAA,
                 '%s, reg0=%04X' % (EX.get(code, code), unguarded.hold(0)))
    unguarded.close()

    ok = Core(lib)
    rsp = ok.execute(req)
    report.check('an acceptable span writes every register and echoes '
                 'address and quantity',
                 rsp == b'\x10\x00\x00\x00\x03' and ok.hold(2) == 0xDEAD,
                 rsp.hex(' '))
    ok.close()


def test_dispatch(report, lib):
    """Unknown, user-defined, absent callbacks, and the length rule."""
    core = Core(lib)
    _fc, code = core.exception(b'\x63\x00\x00\x00\x01')
    report.check('an unimplemented function code -> ILLEGAL FUNCTION',
                 code == 1, EX.get(code, code))

    rsp = core.execute(b'\x41\x01\x02\x03')
    report.check('a user-defined code (65) reaches the application',
                 rsp == b'\x41\x01\x02\x03', rsp.hex(' '))
    rsp = core.execute(b'\x64\xAB')
    report.check('and so does the second user range (100)',
                 rsp == b'\x64\xAB', rsp.hex(' '))

    report.check('a length contradicting the function code is answered with '
                 'silence, not an exception',
                 core.execute(b'\x03\x00\x00\x00') == b'' and
                 core.execute(b'\x03\x00\x00\x00\x01\x00') == b'')

    core.drop(6)
    _fc, code = core.exception(b'\x41\x01')
    report.check('with no user_function the same code -> ILLEGAL FUNCTION',
                 code == 1, EX.get(code, code))
    core.drop(0)
    _fc, code = core.exception(b'\x03\x00\x00\x00\x01')
    report.check('a model that cannot read registers -> ILLEGAL FUNCTION',
                 code == 1, EX.get(code, code))
    core.drop(3)
    _fc, code = core.exception(b'\x05\x00\x00\xFF\x00')
    report.check('a model that cannot write bits -> ILLEGAL FUNCTION',
                 code == 1, EX.get(code, code))
    core.close()


def test_server_id(report, lib):
    """FC 0x11, including the clamp that keeps a long id inside the buffer."""
    core = Core(lib)
    rsp = core.execute(b'\x11')
    report.check('report server id is fc, byte count, run indicator, then id',
                 rsp[:3] == b'\x11\x0E\xFF' and rsp[3:] == b'coaxial_63100',
                 rsp[3:].decode('ascii', 'replace'))

    core.set_id('x' * 60)
    rsp = core.execute(b'\x11', cap=20)
    report.check('a long id is clamped to the buffer, not written past it',
                 len(rsp) == 20 and rsp[1] == 18, '%d bytes' % len(rsp))

    core.drop(5)
    _fc, code = core.exception(b'\x11')
    report.check('with no server_id callback -> ILLEGAL FUNCTION', code == 1,
                 EX.get(code, code))
    core.close()


def test_rtu_frame(report, lib):
    """A frame ends when the line has been quiet for t3.5, and only then."""
    core = Core(lib)
    core.rtu_init()
    frame = core.adu(1, b'\x03\x00\x00\x00\x02')

    at = core.feed(frame)
    report.check('a frame in progress reports busy', core.busy())
    report.check('and is not delivered before t3.5 of silence',
                 core.service(at + core.t35() - 1) == b'', 'still framing')

    rsp = core.service(at + core.t35() + 1)
    report.check('after t3.5 the response comes back addressed to this unit',
                 len(rsp) > 4 and rsp[0] == 1 and rsp[1] == 3, rsp.hex(' '))
    report.check('and it carries a CRC that checks out',
                 lib.mbh_crc_check(rsp, ctypes.c_size_t(len(rsp))) != 0)
    report.check('the transport is idle again', not core.busy())

    counters = core.counters()
    report.check('one bus message, one server message, no errors',
                 counters['bus_message'] == 1 and
                 counters['server_message'] == 1 and
                 counters['bus_comm_error'] == 0, str(counters))
    core.close()


def test_rtu_gap(report, lib):
    """A gap longer than t1.5 inside a frame means it was never a frame.

    It must be drained and discarded, not truncated and parsed - a truncated
    frame that happened to pass CRC would be acted on.
    """
    core = Core(lib)
    core.rtu_init()
    frame = core.adu(1, b'\x03\x00\x00\x00\x02')

    for i, b in enumerate(frame[:3]):
        core.byte(b, 1000000 + i * 100)
    late = 1000000 + 3 * 100 + core.t15() + 10
    for i, b in enumerate(frame[3:]):
        core.byte(b, late + i * 100)

    report.check('a frame with a t1.5 gap in it produces no response',
                 core.service(late + len(frame) * 100 + core.t35() + 1) == b'')
    counters = core.counters()
    report.check('and is counted as a bus communication error',
                 counters['bus_comm_error'] == 1 and counters['bus_message'] == 1,
                 str(counters))
    core.close()


def test_rtu_wrap(report, lib):
    """The tick counter wraps at exactly 2^32, and the arithmetic must hold.

    This is invariant 2: the elapsed-time subtraction is done in raw uint32
    for this reason, and dividing cycles down to microseconds would move the
    wrap off a power of two and break it silently. Here the frame starts
    before the wrap and finishes after it.
    """
    core = Core(lib)
    core.rtu_init()
    frame = core.adu(1, b'\x03\x00\x00\x00\x01')

    start = (1 << 32) - (len(frame) * 100) // 2
    at = core.feed(frame, start=start)
    report.check('the frame straddles the wrap', at < start, 'ticks %d -> %d'
                 % (start, at))
    report.check('nothing is delivered before t3.5, across the wrap',
                 core.service((at + core.t35() - 1) & 0xFFFFFFFF) == b'')
    rsp = core.service((at + core.t35() + 1) & 0xFFFFFFFF)
    report.check('and the response arrives once t3.5 has really elapsed',
                 len(rsp) > 4 and rsp[1] == 3, rsp.hex(' '))
    core.close()


def test_rtu_addressing(report, lib):
    """Whose frame it is, and the two ways that answer changes the reply."""
    core = Core(lib)
    core.rtu_init(unit=1)

    other = core.adu(9, b'\x03\x00\x00\x00\x01')
    at = core.feed(other)
    report.check("a frame for another unit gets no response",
                 core.service(at + core.t35() + 1) == b'')
    counters = core.counters()
    report.check('and is not counted against this server',
                 counters['server_message'] == 0 and counters['bus_message'] == 1,
                 str(counters))

    cast = core.adu(0, b'\x06\x00\x07\x12\x34')
    at = core.feed(cast, start=at + core.t35() + 1000)
    report.check('a broadcast is never answered',
                 core.service(at + core.t35() + 1) == b'')
    report.check('but it was executed', core.hold(7) == 0x1234,
                 hex(core.hold(7)))
    counters = core.counters()
    report.check('and counted as a frame handled with no reply',
                 counters['server_no_response'] == 1, str(counters))
    core.close()


def test_rtu_rejects(report, lib):
    """Bad CRC, a runt, an over-long ADU and a receiver error."""
    core = Core(lib)
    core.rtu_init()

    frame = bytearray(core.adu(1, b'\x03\x00\x00\x00\x01'))
    frame[-1] ^= 0xFF
    at = core.feed(bytes(frame))
    report.check('a bad CRC is answered with silence, never an exception',
                 core.service(at + core.t35() + 1) == b'')
    report.check('and counted as a communication error',
                 core.counters()['bus_comm_error'] == 1,
                 str(core.counters()))

    at = core.feed(b'\x01\x03', start=at + core.t35() + 1000)
    report.check('a frame shorter than the smallest legal ADU is discarded',
                 core.service(at + core.t35() + 1) == b'')

    long_frame = bytes([1]) + bytes(300)
    at = core.feed(long_frame, start=at + core.t35() + 1000)
    report.check('an ADU longer than 256 bytes is discarded, not truncated',
                 core.service(at + core.t35() + 1) == b'')

    core.rtu_error(at + core.t35() + 2000)
    at = core.feed(core.adu(1, b'\x03\x00\x00\x00\x01'),
                   start=at + core.t35() + 2100)
    report.check('a receiver error discards the frame it interrupted',
                 core.service(at + core.t35() + 1) == b'')
    report.check('and is counted as a character overrun',
                 core.counters()['char_overrun'] == 1, str(core.counters()))
    core.close()


def test_rtu_exception(report, lib):
    """An exception is a reply like any other, and is counted as one."""
    core = Core(lib)
    core.rtu_init()
    at = core.feed(core.adu(1, b'\x03\x00\x00\x00\x00'))     # qty 0
    rsp = core.service(at + core.t35() + 1)
    report.check('an illegal quantity comes back as an exception frame',
                 len(rsp) == 5 and rsp[1] == 0x83 and rsp[2] == 3,
                 rsp.hex(' '))
    report.check('counted in server_exception',
                 core.counters()['server_exception'] == 1,
                 str(core.counters()))
    core.close()


ROSTER = (test_crc, test_quantities, test_span, test_reads, test_capacity,
          test_writes, test_half_write, test_dispatch, test_server_id,
          test_rtu_frame, test_rtu_gap, test_rtu_wrap, test_rtu_addressing,
          test_rtu_rejects, test_rtu_exception)


def test_rtu_early_dispatch(report, lib):
    """A proven request is delivered on its own CRC, not after t3.5.

    The oracle names the length, the CRC checks, the address is ours -
    the silence had nothing left to say. Everything the oracle cannot
    prove, and every damaged frame, waits it out exactly as before:
    the early path is an escape from the delimiter, never from a rule.
    """
    h = Core(lib)
    h.rtu_init()
    h.hint(True)

    # FC 0x03, a fixed five-byte PDU the host-built oracle proves. The
    # very tick the last byte lands, service hands the frame over.
    frame = h.adu(1, [0x03, 0x00, 0x00, 0x00, 0x01])
    last = h.feed(frame)
    reply = h.service(last)
    report.check('a proven frame is served the tick its last byte lands',
                 len(reply) > 0 and reply[1] == 0x03, reply.hex(' '))
    report.check('and the transport is idle again, mid-silence',
                 not h.busy(), h.busy())

    # The same frame with its CRC broken: the early path must NOT consume
    # it - the t3.5 judge sees the same bytes and counts the error.
    bad = frame[:-2] + bytes([frame[-2] ^ 0xFF, frame[-1]])
    last = h.feed(bad)
    early = h.service(last)
    before = h.counters()['bus_comm_error']
    late = h.service((last + h.t35() + 1) & 0xFFFFFFFF)
    after = h.counters()['bus_comm_error']
    report.check('a broken CRC is never taken early',
                 early == b'' and late == b'', (early + late).hex(' '))
    report.check('and the silence-path judges it, once',
                 after == before + 1, '%d -> %d' % (before, after))

    # An unproven shape - FC 0x42's optional tail - waits the silence out
    # even though its CRC is fine.
    frame = h.adu(1, [0x42])
    last = h.feed(frame)
    report.check('an unproven shape still waits out t3.5',
                 h.service(last) == b'' and h.busy(), h.busy())
    h.service((last + h.t35() + 1) & 0xFFFFFFFF)

    # Another unit's frame is another unit's shape: never proven here.
    frame = h.adu(9, [0x03, 0x00, 0x00, 0x00, 0x01])
    last = h.feed(frame)
    report.check("another node's frame is not early-delivered",
                 h.service(last) == b'' and h.busy(), h.busy())
    h.service((last + h.t35() + 1) & 0xFFFFFFFF)

    # With no oracle installed the old world is byte-for-byte back.
    h.hint(False)
    frame = h.adu(1, [0x03, 0x00, 0x00, 0x00, 0x01])
    last = h.feed(frame)
    report.check('without the oracle every frame waits, as it always did',
                 h.service(last) == b'' and h.busy(), h.busy())
    h.close()


def test_oracle_prefixes(report, lib):
    """The C oracle and the Python mirror, over every prefix of every
    hinted shape - and over the shapes that must never prove.

    THE INVARIANT (cmd_length.c): a non-zero answer must equal the full
    length of every real request it can match. A row that fires at or
    under the bytes in hand is the corruption class this sweep exists
    to fail.
    """
    from coaxial.protocol import request_length as mirror

    fn = lib.mbh_request_length
    fn.restype = ctypes.c_uint16

    def oracle(pdu, have):
        return fn(bytes(pdu), ctypes.c_uint16(have))

    # Real request PDUs, encoded exactly as the host classes send them.
    proven = [
        bytes([0x03, 0x00, 0x10, 0x00, 0x02]),            # FC03 read
        bytes([0x06, 0x00, 0x01, 0x00, 0x01]),            # FC06 write
        bytes([0x6E, 3, 1, 45, 0, 1, 0xC2, 0x00]),        # cal set_param
        bytes([0x6E, 4, 2, 0, 100, 0, 0, 0, 0,
               0, 0, 1, 0xF4]),                           # duty + periods
        bytes([0x6E, 6, 4, 15]),                          # daq read, want
        bytes([0x6E, 6, 0]),                              # daq state
        bytes([0x6E, 7, 0]),                              # clock latch
        bytes([0x6E, 10, 1, 3]),                          # drive mode
        bytes([0x6E, 10, 2, 1, 0, 0, 0x03, 0xE8]),        # drive setpoint
        bytes([0x6E, 10, 4, 0, 0x0F, 0x42, 0x40]),        # drive theta
    ]
    never = [
        bytes([0x6E, 4, 2, 0, 100, 0, 0, 0, 0]),          # duty, short form
        bytes([0x6E, 6, 4]),                              # daq read, no want
        bytes([0x42, 3]),                                 # adc table, paged
        bytes([0x6E, 5, 2, 4]),                           # ring take: unhinted
    ]

    bad = []
    for pdu in proven + never:
        full = len(pdu)
        expect_full = full if pdu in proven else 0
        for have in range(1, full + 1):
            c = oracle(pdu[:have], have)
            m = mirror(pdu[:have], have)
            if c != m:
                bad.append('%s@%d: C %d mirror %d'
                           % (pdu.hex(), have, c, m))
            # A shape may name its full length before the tail arrives -
            # the early path only fires once the bytes are all IN HAND -
            # so the one forbidden answer is a length at or under `have`
            # that is not the true end, and at the end, anything but the
            # truth.
            if c != 0:
                if c < have or (have == full and c != expect_full
                                and expect_full != 0):
                    bad.append('%s@%d fires wrong: %d'
                               % (pdu.hex(), have, c))
                if expect_full == 0 and c == have:
                    bad.append('%s@%d proves a never-shape'
                               % (pdu.hex(), have))
    report.check('the C oracle and the mirror agree on every prefix',
                 not bad, '; '.join(bad[:3]))


def test_fixed_dict_matches_tables(report, lib):
    """The mirror's non-0x6E lengths against the dispatch tables' own
    req_len rows, read out of the C source - the same binding
    test_simulated uses on s_adcTable. The firmware's oracle asks
    cmd_find at runtime and cannot drift; this holds the MIRROR to it.
    """
    import io
    import re
    from coaxial.protocol import request_length as mirror

    rows = {}
    names = {}
    for line in io.open(os.path.join(REPO, 'comms', 'inc', 'cmd.h'),
                        encoding='utf-8'):
        m = re.match(r'#define (CMD_\w+)\s+(0x[0-9A-Fa-f]+)U', line)
        if m:
            names[m.group(1)] = int(m.group(2), 16)
    for fname in ('cmd_board.c', 'cmd_test.c'):
        for line in io.open(os.path.join(REPO, 'comms', 'src', fname),
                            encoding='utf-8'):
            m = re.match(r'\s*\{ (CMD_\w+),\s*"[^"]+",\s*'
                         r'(CMD_LEN_VARIABLE|\d+)U?,', line)
            if m and m.group(1) in names:
                code = names[m.group(1)]
                rows[code] = (None if m.group(2) == 'CMD_LEN_VARIABLE'
                              else int(m.group(2)))

    bad = []
    for code, req_len in sorted(rows.items()):
        if code == 0x6E:
            continue
        probe = bytes([code]) + bytes(9)
        got = mirror(probe, len(probe))
        want = 0 if req_len is None else 1 + req_len
        if got != want:
            bad.append('0x%02X: mirror %d, table %s' % (code, got, want))
    report.check('the mirror matches the dispatch tables row for row',
                 len(rows) >= 15 and not bad,
                 '; '.join(bad[:3]) or '%d rows' % len(rows))


ROSTER = ROSTER + (test_rtu_early_dispatch, test_oracle_prefixes,
                   test_fixed_dict_matches_tables)


def main():
    cc = find_cc()
    if '--which-cc' in sys.argv[1:]:
        # setup.ps1 asks this rather than looking on PATH itself: winget puts
        # the compiler somewhere only new shells see, and a second answer here
        # is how -Check offers to install one that is already installed.
        print(cc or '')
        return 0 if cc else 1
    if cc is None:
        print('no host C compiler found - skipping. '
              'setup.ps1 installs one (BrechtSanders.WinLibs.POSIX.UCRT).')
        print('\n0 passed, 0 failed')
        return 0

    print('-- %s --' % cc)
    try:
        lib_path, warnings = build(cc)
    except RuntimeError as exc:
        print('  FAIL  %-58s %s' % ('the portable core builds on this host',
                                    str(exc).splitlines()[-1][:80]))
        print('\n0 passed, 1 failed')
        return 1

    report = Report()
    report.check('the portable core builds with no warnings on a second '
                 'compiler', not warnings,
                 warnings[0][:60] if warnings else '')

    lib = ctypes.CDLL(lib_path)
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
