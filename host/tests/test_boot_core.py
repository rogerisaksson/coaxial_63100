#!/usr/bin/env python3
"""The bootloader's state machine as the C that will run, on a RAM flash.

`boot/src/boot_core.c` is hardware-free: the flash, the console and the
identity arrive through four calls, which `boot/test/harness.c` wires to
a two-megabyte byte array with 128 K sectors and a fault a test can
script. Built here with the host gcc like the modbus core and driven
through ctypes, so the whole exchange - hold, who, assign, erase, the
chunk stream, missing, verify, record, seal, go - is proven before any
register is touched, and so the things that must never happen are
proven not to: a word programmed twice, an image valid before its seal,
a node answering a question that was not its own.

docs/BOOT.md is the design; PROTOCOL.md's device 11 is the table.
"""
import ctypes
import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_modbus_core import Report, build, find_cc          # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
BOOT = os.path.join(REPO, 'boot')
SOURCES = [os.path.join(BOOT, 'test', 'harness.c'),
           os.path.join(BOOT, 'src', 'boot_core.c'),
           os.path.join(REPO, 'comms', 'src', 'wire.c')]
INCLUDES = [os.path.join(BOOT, 'inc'), os.path.join(REPO, 'comms', 'inc')]

#: The layout the harness builds, docs/BOOT.md's flash map.
APP_BASE, APP_BYTES = 0x08020000, 14 * 128 * 1024
RECORD_BASE = 0x081E0000
CHUNK, WORD = 224, 32
HEADER_OFFSET, MAGIC = 0x400, 0x50415843
UNIT_BLANK = 247
#: The ops, boot.h's numbers.
HOLD, WHO, ASSIGN, ERASE, CHUNK_OP, MISSING, VERIFY, RECORD, SEAL, GO, STATE, DUMP, STAY = range(13)
BLANK, HELD, ASSIGNED, ERASED, VERIFIED, SEALED = range(6)
TYPE = 1
UID = bytes(range(0x10, 0x1C))
SESSION = 0x5EED0001


def image(size, type_=TYPE, version=7):
    """An application image: a stack pointer in DTCM, a thumb reset
    vector inside the application, the header, then a pattern."""
    body = bytearray((i * 7 + 3) & 0xFF for i in range(size))
    struct.pack_into('<II', body, 0, 0x20010000, APP_BASE + 0x1001)
    struct.pack_into('<IIII', body, HEADER_OFFSET, MAGIC, size, version, type_)
    return bytes(body)


class Node:
    """One blank node in the harness, spoken to op by op."""

    def __init__(self, lib, type_=TYPE, uid=UID):
        self.lib = lib
        lib.boot_h_said.restype = ctypes.c_char_p
        lib.boot_h_reset(type_, uid)

    def op(self, op, payload=b''):
        """The reply's payload, b'' for silence, None for a reply that
        overflowed."""
        out = ctypes.create_string_buffer(253)
        n = self.lib.boot_h_op(op, payload, len(payload), out, 253)
        return None if n < 0 else out.raw[:n]

    def took(self, op, payload=b''):
        got = self.op(op, payload)
        return (got[0] == 1, got[2:2 + got[1]].decode() if got[0] == 0 else '')

    def read(self, address, n):
        out = ctypes.create_string_buffer(n)
        self.lib.boot_h_read(address, out, n)
        return out.raw

    def write(self, address, data):
        self.lib.boot_h_write(address, data, len(data))

    state = property(lambda self: self.lib.boot_h_state())
    unit = property(lambda self: self.lib.boot_h_unit())
    go = property(lambda self: bool(self.lib.boot_h_go()))
    valid = property(lambda self: bool(self.lib.boot_h_app_valid()))
    said = property(lambda self: self.lib.boot_h_said().decode())
    programs = property(lambda self: self.lib.boot_h_programs())
    erases = property(lambda self: self.lib.boot_h_erases())
    ignored = property(lambda self: self.lib.boot_h_ignored())


def chunks_of(img):
    return [img[i:i + CHUNK] for i in range(0, len(img), CHUNK)]


def erase_request(img):
    return struct.pack('>BIIH', TYPE, len(img), zlib.crc32(img), len(chunks_of(img)))


def stream(node, img, skip=()):
    for i, piece in enumerate(chunks_of(img)):
        if i not in skip:
            node.op(CHUNK_OP, struct.pack('>H', i) + piece)


def missing(node):
    got = node.op(MISSING)
    first, count = struct.unpack('>HH', got[:4])
    bitmap = got[4:]
    return [i for i in range(first, count) if not bitmap[i // 8] & (1 << (i % 8))]


def session(node, img, record=b''):
    """The master's sequence on one node, up to the seal."""
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img)
    ok, crc = struct.unpack('>BI', node.op(VERIFY))
    for at in range(0, len(record), CHUNK):
        node.took(RECORD, struct.pack('>H', at) + record[at:at + CHUNK])
    return ok, crc


# ------------------------------------------------------------------ tests

def test_a_blank_node(report, node):
    """Fresh from reset: blank, unit 247, no application, silent to a
    broadcast and to a question that is not its own."""
    report.check('a fresh node is blank at unit 247, no application',
                 node.state == BLANK and node.unit == UNIT_BLANK and not node.valid)
    report.check('hold is a broadcast: no reply, and the node stays',
                 node.op(HOLD, struct.pack('>I', SESSION)) == b'' and node.state == HELD)
    report.check('who with no prefix is answered by every node: uid, type, state, unit',
                 node.op(WHO, b'\x00') == UID + bytes([TYPE, HELD, UNIT_BLANK]))
    report.check('who with a prefix that is not this node is silence',
                 node.op(WHO, bytes([12, 0xFF, 0x00])) == b'')
    report.check('who with 13 bits of the right prefix is answered',
                 node.op(WHO, bytes([13, 0x10, 0x11])) != b'')
    report.check('and 13 bits with the last one wrong is silence',
                 node.op(WHO, bytes([13, 0x10, 0x19])) == b'')
    report.check('assign to another uid is silence',
                 node.op(ASSIGN, bytes(12) + bytes([2, 2, 0])) == b'' and node.unit == UNIT_BLANK)
    report.check('assign to unit 0 is refused in words',
                 node.took(ASSIGN, UID + bytes([0, 2, 0])) == (False, 'a unit is 1..246 - 0 is broadcast and 247 is every blank node'))
    report.check('assign to this uid takes, and the node answers to its unit',
                 node.took(ASSIGN, UID + bytes([2, 2, 1])) == (True, '') and node.unit == 2
                 and node.state == ASSIGNED and node.lib.boot_h_terminates() == 1)
    report.check('state says so: state, type, unit, position, 0 of 0 chunks, no app, the uid',
                 node.op(STATE) == bytes([ASSIGNED, TYPE, 2, 2]) + struct.pack('>II', 0, 0) + b'\x00' + UID)


def test_an_image_streamed(report, node):
    """The happy path: erase, every chunk once, verify, record, seal, go."""
    img = image(3 * CHUNK * 100 + 17)               # 301 chunks, a ragged last one
    record = bytes(range(256)) * 3
    ok, crc = session(node, img, record)
    report.check('every chunk landed and the crc is the master\'s',
                 ok == 1 and crc == zlib.crc32(img) and node.state == VERIFIED)
    report.check('the image is still INVALID before the seal: its first word is erased',
                 not node.valid and node.read(APP_BASE, 4) == b'\xff' * 4)
    report.check('the rest of the image is in flash already',
                 node.read(APP_BASE + WORD, len(img) - WORD) == img[WORD:])
    report.check('nothing was programmed twice: one word per 32 bytes less the first',
                 node.programs == (len(img) + WORD - 1) // WORD - 1)
    report.check('seal programs the record and the first word; the image is valid',
                 node.took(SEAL) == (True, '') and node.valid and node.state == SEALED
                 and node.read(APP_BASE, WORD) == img[:WORD]
                 and node.read(RECORD_BASE, len(record)) == record)
    report.check('go for the session sets the jump; nothing is answered',
                 node.op(GO, struct.pack('>I', SESSION)) == b'' and node.go)
    report.check('dump pages the record back, 224 bytes at a time',
                 node.op(DUMP, struct.pack('>H', 224)) == struct.pack('>H', 224) + record[224:448])


def test_chunks_lost_and_resent(report, node):
    """Broadcast loses frames: the bitmap names them, the master re-sends
    only those, a repeat is not programmed again."""
    img = image(50 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img, skip=(0, 7, 49))
    report.check('missing names exactly the chunks that were skipped',
                 missing(node) == [0, 7, 49])
    report.check('verify says no while chunks are missing',
                 node.op(VERIFY)[0] == 0 and node.state == ERASED)
    programs = node.programs
    stream(node, img)                                  # everything again
    report.check('the repeats were not programmed again: only the three missing cost words',
                 node.programs == programs + 7 * 2 + 7 - 1 and missing(node) == [])
    ok, crc = struct.unpack('>BI', node.op(VERIFY))
    report.check('and now it verifies', ok == 1 and crc == zlib.crc32(img))


def test_seal_before_verify_is_refused(report, node):
    img = image(20 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img, skip=(3,))
    report.check('seal before verify is refused in words, and the first word stays erased',
                 node.took(SEAL) == (False, 'the image is not verified - verify first, and it must say ok')
                 and node.read(APP_BASE, 4) == b'\xff' * 4 and not node.valid)
    report.check('go before seal is refused on the console, and no jump',
                 node.op(GO, struct.pack('>I', SESSION)) == b'' and not node.go
                 and node.said == 'boot: not sealed - staying')


def test_the_master_dies(report, node):
    """A stream that stops leaves an invalid image; the next erase starts
    over, and a chunk before an erase is counted and ignored."""
    img = image(30 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img, skip=range(15, 30))
    report.check('half an image is not an application',
                 not node.valid and node.state == ERASED and len(missing(node)) == 15)
    node.op(ERASE, erase_request(img))
    report.check('the next erase forgets it all: two erases, nothing held',
                 node.erases == 2 and missing(node) == list(range(30)))
    fresh = Node(node.lib)
    fresh.op(CHUNK_OP, struct.pack('>H', 0) + img[:CHUNK])
    report.check('a chunk with no erase behind it is ignored and counted',
                 fresh.ignored == 1 and fresh.state == BLANK)


def test_a_wrong_crc_and_a_wrong_type(report, node):
    img = image(10 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, struct.pack('>BIIH', TYPE, len(img), zlib.crc32(img) ^ 1, 10))
    stream(node, img)
    ok, crc = struct.unpack('>BI', node.op(VERIFY))
    report.check('a wrong crc: verify says no and reports what it summed',
                 ok == 0 and crc == zlib.crc32(img) and node.state == ERASED)
    other = Node(node.lib)
    other.op(HOLD, struct.pack('>I', SESSION))
    other.op(ASSIGN, UID + bytes([2, 2, 0]))
    other.op(ERASE, struct.pack('>BIIH', TYPE + 1, len(img), zlib.crc32(img), 10))
    report.check('an erase for another type is ignored: no erase, still assigned',
                 other.erases == 0 and other.state == ASSIGNED)
    other.op(ERASE, struct.pack('>BIIH', TYPE, APP_BYTES + 1, 0, 1))
    report.check('an image that does not fit is refused on the console',
                 other.erases == 0 and other.said == 'boot: an erase named an image that does not fit')


def test_the_debuggers_way_in(report, node):
    """An image written straight to flash - the programmer's path - is
    valid by the four tests, and a wrong one is not."""
    img = image(5 * CHUNK)
    node.write(APP_BASE, img)
    report.check('an image the debugger wrote is valid', node.valid)
    for what, bad in (('a stack pointer outside DTCM', struct.pack('<I', 0x30000000)),
                      ('an even reset vector', struct.pack('<II', 0x20010000, APP_BASE + 0x1000)),):
        node.write(APP_BASE, bad)
        report.check('%s is not an application' % what, not node.valid)
        node.write(APP_BASE, img[:8])
    node.write(APP_BASE + HEADER_OFFSET + 12, struct.pack('<I', TYPE + 1))
    report.check('another type\'s header is not an application', not node.valid)
    node.write(APP_BASE + HEADER_OFFSET + 12, struct.pack('<I', TYPE))
    report.check('and stay from the bootloader itself is refused in words',
                 node.took(STAY) == (False, 'this node is in its bootloader already'))


def test_faults(report, node):
    img = image(6 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img)
    node.op(VERIFY)
    node.lib.boot_h_fail_program(APP_BASE)
    report.check('a first word that will not program leaves the image invalid, in words',
                 node.took(SEAL) == (False, 'the first word did not program - the image stays invalid')
                 and not node.valid)
    node.lib.boot_h_fail_program(0)
    report.check('a record for an unassigned node is refused in words',
                 Node(node.lib).took(RECORD, b'\x00\x00' + b'x' * 8)
                 == (False, 'a record goes to an assigned node - assign first'))
    report.check('an unknown op is refused in words',
                 node.took(99) == (False, 'no such boot op - PROTOCOL.md lists 0..12'))


def test_crc32_is_the_ieee_one(report, node):
    data = bytes(range(256)) * 9
    report.check('the core\'s CRC-32 is zlib\'s',
                 node.lib.boot_h_crc32(data, len(data)) == zlib.crc32(data))


ROSTER = (test_a_blank_node, test_an_image_streamed, test_chunks_lost_and_resent,
          test_seal_before_verify_is_refused, test_the_master_dies,
          test_a_wrong_crc_and_a_wrong_type, test_the_debuggers_way_in, test_faults,
          test_crc32_is_the_ieee_one)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, INCLUDES, name='bootcore')
    lib = ctypes.CDLL(lib_path)
    lib.boot_h_crc32.restype = ctypes.c_uint32
    lib.boot_h_ignored.restype = ctypes.c_uint32
    lib.boot_h_programs.restype = ctypes.c_uint32
    lib.boot_h_erases.restype = ctypes.c_uint32
    report = Report()
    report.check('boot/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, Node(lib))
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
