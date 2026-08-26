#!/usr/bin/env python3
"""SHTP framing and SH-2 decoding, on this machine, with no IMU.

`Shtp/Src/shtp.c` is hardware-free for the same reason the Modbus core is: it
turns a byte buffer into a header and a cargo and nothing else, so it can be
built here and driven through ctypes. Every buffer in this file is a frame the
BNO08X datasheet writes out byte by byte, so what is asserted is the document
rather than the implementation's opinion of it.

Datasheet references are to BNO080_085-Datasheet v1.17 in datasheets/.

A missing compiler is not a failing suite: it says so and passes nothing.
`setup.ps1` installs one.

Run from the host directory:  python tests/test_shtp_core.py
"""
import ctypes
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
SHTP = os.path.join(REPO, 'Shtp')
SOURCES = [os.path.join(SHTP, 'Src', 'shtp.c')]

CH_CONTROL = 2
CH_INPUT = 3


class Header(ctypes.Structure):
    _fields_ = [('length', ctypes.c_uint16), ('continuation', ctypes.c_bool),
                ('channel', ctypes.c_ubyte), ('seq', ctypes.c_ubyte)]


class ProductId(ctypes.Structure):
    _fields_ = [('reset_cause', ctypes.c_ubyte), ('sw_major', ctypes.c_ubyte),
                ('sw_minor', ctypes.c_ubyte), ('sw_part', ctypes.c_uint32),
                ('sw_build', ctypes.c_uint32), ('sw_patch', ctypes.c_uint16)]


class Rpt(ctypes.Structure):
    _fields_ = [('report_id', ctypes.c_ubyte), ('seq', ctypes.c_ubyte),
                ('status', ctypes.c_ubyte), ('delay', ctypes.c_ubyte),
                ('x', ctypes.c_int16), ('y', ctypes.c_int16),
                ('z', ctypes.c_int16), ('w', ctypes.c_int16),
                ('count', ctypes.c_ubyte)]


def header(lib, raw):
    """(ok, Header) for four bytes."""
    out = Header()
    ok = lib.shtp_parse_header(bytes(raw), ctypes.byref(out))
    return bool(ok), out


def reports(lib, cargo, room=8):
    got = (Rpt * room)()
    n = lib.shtp_parse_reports(bytes(cargo), ctypes.c_size_t(len(cargo)),
                               got, ctypes.c_size_t(room))
    return [got[i] for i in range(n)]


def test_header(report, lib):
    """Four bytes: length LSB, length MSB, channel, sequence. Figure 1-26."""
    ok, h = header(lib, b'\x13\x00\x03\x07')
    report.check('length is little-endian and counts its own header',
                 ok and h.length == 19, '%s len=%d' % (ok, h.length))
    report.check('channel and sequence come straight off bytes 2 and 3',
                 h.channel == 3 and h.seq == 7, '%d %d' % (h.channel, h.seq))
    report.check('and a plain cargo is not a continuation', not h.continuation)

    ok, h = header(lib, b'\x13\x80\x03\x00')
    report.check('bit 15 of the length marks a continuation, and is not '
                 'counted as length', ok and h.continuation and h.length == 19,
                 'len=%d cont=%s' % (h.length, h.continuation))

    # "A length of 65535 (0xFFFF) is reserved because a failed peripheral can
    # too easily produce 0xFFFF" - Figure 1-26. An unpowered BNO08X holds MISO
    # high and every read comes back like this.
    ok, _h = header(lib, b'\xFF\xFF\xFF\xFF')
    report.check('0xFFFF is refused rather than read as a 32 kB cargo', not ok)

    ok, h = header(lib, b'\x00\x00\x00\x00')
    report.check('an idle bus reads zero length, which is not an error',
                 ok and h.length == 0, 'len=%d' % h.length)

    ok, _h = header(lib, b'\x02\x00\x02\x00')
    report.check('a length below the header it counts is refused', not ok)


def test_build(report, lib):
    """The worked example, Figure 5-1: a set feature under its header."""
    payload = (b'\xFD\x01\x00\x00\x00\x60\xEA\x00\x00' + bytes(8))
    buf = (ctypes.c_ubyte * 64)()
    n = lib.shtp_build(buf, ctypes.c_size_t(64), ctypes.c_ubyte(CH_CONTROL),
                       ctypes.c_ubyte(5), bytes(payload),
                       ctypes.c_size_t(len(payload)))
    got = bytes(buf[:n])
    report.check('a 17-byte payload is framed as 21, and the length says so',
                 n == 21 and got[0] == 0x15 and got[1] == 0x00, got[:4].hex(' '))
    report.check('on the channel and sequence it was given',
                 got[2] == CH_CONTROL and got[3] == 5, got[:4].hex(' '))
    report.check('with the payload behind the header, unaltered',
                 got[4:] == payload, got[4:9].hex(' '))

    small = (ctypes.c_ubyte * 8)()
    report.check('a payload that will not fit is refused, not truncated',
                 lib.shtp_build(small, ctypes.c_size_t(8),
                                ctypes.c_ubyte(CH_CONTROL), ctypes.c_ubyte(0),
                                bytes(payload),
                                ctypes.c_size_t(len(payload))) == 0)


def test_set_feature(report, lib):
    """Figure 1-33, checked against the worked example in Figure 5-1.

    60 ms is the datasheet's own example and it says the interval reads
    0x0000EA60, so the byte order of that field is asserted rather than
    assumed.
    """
    buf = (ctypes.c_ubyte * 32)()
    n = lib.shtp_set_feature(buf, ctypes.c_size_t(32), ctypes.c_ubyte(0x01),
                             ctypes.c_uint32(60000))
    got = bytes(buf[:n])
    report.check('a set feature command is seventeen bytes', n == 17, str(n))
    report.check('report id 0xFD, then the feature it configures',
                 got[0] == 0xFD and got[1] == 0x01, got[:2].hex(' '))
    report.check('60 ms lands as 0x0000EA60, little-endian, at bytes 5..8',
                 got[5:9] == b'\x60\xEA\x00\x00', got[5:9].hex(' '))
    report.check('and everything this firmware does not set is zero - no '
                 'change sensitivity, no batching, no sensor-specific word',
                 got[2:5] == bytes(3) and got[9:] == bytes(8),
                 got.hex(' '))

    report.check('an interval of zero is a disable, not a refusal',
                 lib.shtp_set_feature(buf, ctypes.c_size_t(32),
                                      ctypes.c_ubyte(0x01),
                                      ctypes.c_uint32(0)) == 17)
    small = (ctypes.c_ubyte * 16)()
    report.check('sixteen bytes of room is refused rather than half written',
                 lib.shtp_set_feature(small, ctypes.c_size_t(16),
                                      ctypes.c_ubyte(0x01),
                                      ctypes.c_uint32(0)) == 0)


def test_product_id(report, lib):
    """Figure 1-29: sixteen fixed bytes behind report id 0xF8."""
    cargo = bytes([0xF8, 0x01, 3, 2,
                   0x78, 0x56, 0x34, 0x12,
                   0x21, 0x43, 0x65, 0x87,
                   0x0A, 0x00, 0, 0])
    out = ProductId()
    ok = lib.shtp_parse_product_id(cargo, ctypes.c_size_t(len(cargo)),
                                   ctypes.byref(out))
    report.check('the response decodes', ok)
    report.check('reset cause and software version come off bytes 1..3',
                 out.reset_cause == 1 and out.sw_major == 3
                 and out.sw_minor == 2,
                 '%d %d.%d' % (out.reset_cause, out.sw_major, out.sw_minor))
    report.check('part and build numbers are 32-bit little-endian',
                 out.sw_part == 0x12345678 and out.sw_build == 0x87654321,
                 '%08X %08X' % (out.sw_part, out.sw_build))
    report.check('and the patch is the 16-bit field at 12',
                 out.sw_patch == 10, str(out.sw_patch))

    report.check('a cargo one byte short is refused, not padded',
                 not lib.shtp_parse_product_id(cargo[:15],
                                               ctypes.c_size_t(15),
                                               ctypes.byref(out)))
    other = bytes([0xF1]) + cargo[1:]
    report.check('and another report id is not read as a product id',
                 not lib.shtp_parse_product_id(other,
                                               ctypes.c_size_t(len(other)),
                                               ctypes.byref(out)))


def test_reports(report, lib):
    """Figure 5-2: a timebase reference then an accelerometer report."""
    cargo = bytes([0xFB, 0x0A, 0x00, 0x00, 0x00,
                   0x01, 0x2A, 0x03, 0x00,
                   0x10, 0x27, 0xF0, 0xD8, 0x00, 0x01])
    got = reports(lib, cargo)
    report.check('a cargo carries its timebase and its sensor report',
                 len(got) == 2, '%d reports' % len(got))
    if len(got) != 2:
        return
    report.check('the timebase is kept rather than skipped, with no axes',
                 got[0].report_id == 0xFB and got[0].count == 0,
                 '0x%02X count=%d' % (got[0].report_id, got[0].count))
    acc = got[1]
    report.check('the accelerometer report is identified and sequenced',
                 acc.report_id == 0x01 and acc.seq == 0x2A,
                 '0x%02X seq=%d' % (acc.report_id, acc.seq))
    report.check('status carries the accuracy in bits 1:0 - 3 is high',
                 (acc.status & 3) == 3, 'status=0x%02X' % acc.status)
    report.check('the axes are signed little-endian counts, unscaled',
                 (acc.x, acc.y, acc.z) == (10000, -10000, 256),
                 '%d %d %d' % (acc.x, acc.y, acc.z))
    report.check('and three of the four fields are meaningful',
                 acc.count == 3, str(acc.count))


def test_walk_stops(report, lib):
    """An unknown report id ends the walk instead of guessing its length.

    The reports are not self-delimiting: a wrong length does not lose one
    report, it mis-frames every byte after it, so a length is only entered
    here once something states it.
    """
    # The datasheet does not tabulate these two - it refers to the SH-2
    # Reference Manual. They come from CEVA's own decoder instead,
    # github.com/ceva-dsp/sh2, sh2_SensorValue.c.
    report.check('the rotation vector is fourteen: the four components and '
                 'an accuracy estimate behind the common header',
                 lib.shtp_report_len(ctypes.c_ubyte(0x05)) == 14,
                 str(lib.shtp_report_len(ctypes.c_ubyte(0x05))))
    report.check('the game rotation vector is twelve - the same without the '
                 'accuracy estimate',
                 lib.shtp_report_len(ctypes.c_ubyte(0x08)) == 12,
                 str(lib.shtp_report_len(ctypes.c_ubyte(0x08))))
    report.check('and a report id nothing states a length for is still '
                 'refused rather than guessed',
                 lib.shtp_report_len(ctypes.c_ubyte(0x99)) == 0)
    report.check('while the two the datasheet writes out byte by byte are '
                 'ten - Figure 1-34 and Figure 5-2',
                 lib.shtp_report_len(ctypes.c_ubyte(0x01)) == 10
                 and lib.shtp_report_len(ctypes.c_ubyte(0x02)) == 10)
    report.check('and the timebase is five - Figure 5-2 bytes 4..8',
                 lib.shtp_report_len(ctypes.c_ubyte(0xFB)) == 5)

    cargo = bytes([0x01, 0x00, 0x03, 0x00, 1, 0, 2, 0, 3, 0,
                   0x05, 0x00, 0x03, 0x00, 9, 9, 9, 9, 9, 9, 9, 9])
    got = reports(lib, cargo)
    report.check('a known report before an unknown one is still delivered',
                 len(got) == 1 and got[0].report_id == 0x01,
                 '%d reports' % len(got))

    truncated = bytes([0x01, 0x00, 0x03, 0x00, 1, 0, 2, 0])
    report.check('a report cut short by the end of the cargo is dropped, '
                 'not read past', not reports(lib, truncated))

    report.check('and a cargo of nothing decodes to nothing',
                 not reports(lib, b''))


ROSTER = (test_header, test_build, test_set_feature, test_product_id,
          test_reports, test_walk_stops)


def main():
    cc = find_cc()
    if cc is None:
        print('no host C compiler found - skipping. '
              'setup.ps1 installs one (BrechtSanders.WinLibs.POSIX.UCRT).')
        print('\n0 passed, 0 failed')
        return 0

    print('-- %s --' % cc)
    try:
        lib_path, warnings = build(cc, SOURCES,
                                   [os.path.join(SHTP, 'Inc')], 'shtpcore')
    except RuntimeError as exc:
        print('  FAIL  %-58s %s' % ('the SHTP layer builds on this host',
                                    str(exc).splitlines()[-1][:80]))
        print('\n0 passed, 1 failed')
        return 1

    report = Report()
    report.check('the SHTP layer builds with no warnings on a second compiler',
                 not warnings, warnings[0][:60] if warnings else '')

    lib = ctypes.CDLL(lib_path)
    lib.shtp_parse_header.restype = ctypes.c_bool
    lib.shtp_parse_product_id.restype = ctypes.c_bool
    lib.shtp_build.restype = ctypes.c_size_t
    lib.shtp_set_feature.restype = ctypes.c_size_t
    lib.shtp_report_len.restype = ctypes.c_size_t
    lib.shtp_parse_reports.restype = ctypes.c_size_t

    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
