"""The board's memcpy and memset (board/src/board_mem.c), run as the C the board links in place of
newlib-nano's: every alignment of both ends and every length to past four words, against
Python's bytes, the guard bytes about the destination untouched."""
import ctypes
import os
import sys

from tools.cores.build import build, find_cc

from test_modbus_core import Report

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
SOURCES = [os.path.join(REPO, 'board', 'src', 'board_mem.c')]
INCLUDES = [os.path.join(REPO, 'comms', 'inc')]
#: Renamed for the host, whose own the library keeps.
RENAMED = ('-Dmemcpy=board_memcpy', '-Dmemset=board_memset')

GUARD, SPAN = 8, 72


def library():
    lib = ctypes.CDLL(build(find_cc(), SOURCES, INCLUDES, 'board_mem', RENAMED)[0])
    lib.board_memcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t]
    lib.board_memcpy.restype = ctypes.c_void_p
    lib.board_memset.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_size_t]
    lib.board_memset.restype = ctypes.c_void_p
    return lib


def test_every_copy(report, lib):
    """Each of 4 x 4 alignments and lengths 0 .. SPAN: the copy its source's, nothing past it."""
    source = bytes((i * 37 + 11) & 0xFF for i in range(SPAN + 2 * GUARD + 4))
    wrong = []
    for into in range(4):
        for at in range(4):
            for n in range(SPAN + 1):
                buf = (ctypes.c_uint8 * (SPAN + 2 * GUARD + 4))(*([0xEE] * (SPAN + 2 * GUARD + 4)))
                src = (ctypes.c_uint8 * len(source)).from_buffer_copy(source)
                dst = ctypes.addressof(buf) + GUARD + into
                back = lib.board_memcpy(dst, ctypes.addressof(src) + at, n)
                got = bytes(buf)
                want = (b'\xee' * (GUARD + into) + source[at:at + n]
                        + b'\xee' * (len(got) - GUARD - into - n))
                if got != want or back != dst:
                    wrong.append((into, at, n))
    report.check('memcpy: 16 alignments x %d lengths its source\'s, the guards whole' % (SPAN + 1),
                 not wrong, '%d wrong, the first %s' % (len(wrong), wrong[:3]))


def test_every_fill(report, lib):
    """Each alignment and length 0 .. SPAN, a byte with its top bits set: that byte, nothing
    past it."""
    wrong = []
    for into in range(4):
        for n in range(SPAN + 1):
            buf = (ctypes.c_uint8 * (SPAN + 2 * GUARD + 4))(*([0x11] * (SPAN + 2 * GUARD + 4)))
            dst = ctypes.addressof(buf) + GUARD + into
            back = lib.board_memset(dst, 0x1A5, n)
            got = bytes(buf)
            want = b'\x11' * (GUARD + into) + b'\xa5' * n + b'\x11' * (len(got) - GUARD - into - n)
            if got != want or back != dst:
                wrong.append((into, n))
    report.check('memset: 4 alignments x %d lengths the byte, the guards whole' % (SPAN + 1),
                 not wrong, '%d wrong, the first %s' % (len(wrong), wrong[:3]))


def main():
    report = Report()
    if find_cc() is None:
        print('no C compiler: board_mem.c cannot be built here')
        print('\n0 passed, 0 failed')
        return 0
    lib = library()
    for test in (test_every_copy, test_every_fill):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
