#!/usr/bin/env python3
"""The bootloader's state machine as the C that will run, on byte-array flash and RAM."""
import ctypes
import os
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_modbus_core import Report, build, find_cc          # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.dirname(HERE))
BOOT = os.path.join(REPO, 'boot')
SOURCES = [os.path.join(BOOT, 'test', 'harness.c'),
           os.path.join(BOOT, 'src', 'boot_core.c'),
           os.path.join(REPO, 'comms', 'src', 'wire.c')]
INCLUDES = [os.path.join(BOOT, 'inc'), os.path.join(REPO, 'comms', 'inc')]

#: The layout the harness builds, docs/BOOT.md's map: the image runs from
#: D2 SRAM, flash keeps a sealed copy and the record.
RUN_BASE, RUN_BYTES = 0x30000000, 288 * 1024
STORE_BASE = 0x08020000
RECORD_BASE = 0x081E0000
SEAL_MAGIC, PERSIST = 0x4C414553, 1
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
    struct.pack_into('<II', body, 0, 0x20010000, RUN_BASE + 0x1001)
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

    def reboot(self):
        """A reset: the state gone, flash and RAM kept."""
        self.lib.boot_h_reboot()

    def power_cycle(self):
        """The state gone, RAM noise, flash kept."""
        self.lib.boot_h_power_cycle()

    def ready(self, bytes_=0, crc=0):
        """The reset's gate, with what the slot names."""
        return bool(self.lib.boot_h_ready(bytes_, crc))

    def image(self):
        out = (ctypes.c_uint32 * 2)()
        self.lib.boot_h_image(out)
        return tuple(out)

    state = property(lambda self: self.lib.boot_h_state())
    unit = property(lambda self: self.lib.boot_h_unit())
    go = property(lambda self: bool(self.lib.boot_h_go()))
    valid = property(lambda self: bool(self.lib.boot_h_app_valid()))
    said = property(lambda self: self.lib.boot_h_said().decode())
    programs = property(lambda self: self.lib.boot_h_programs())
    writes = property(lambda self: self.lib.boot_h_writes())
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
                 node.op(STATE) == bytes([ASSIGNED, TYPE, 2, 2]) + struct.pack('>II', 0, 0) + b'\x00' + UID
                 + struct.pack('>II', 0, 0) + b'\x01')


def test_an_image_streamed(report, node):
    """The happy path: erase, every chunk once, verify, record, seal, go."""
    img = image(3 * CHUNK * 100 + 17)               # 301 chunks, a ragged last one
    record = bytes(range(256)) * 3
    ok, crc = session(node, img, record)
    report.check('every chunk landed and the crc is the master\'s',
                 ok == 1 and crc == zlib.crc32(img) and node.state == VERIFIED)
    report.check('the image is still INVALID before the seal: its first word is erased',
                 not node.valid and node.read(RUN_BASE, 4) == b'\xff' * 4)
    report.check('the rest of the image is in RAM already',
                 node.read(RUN_BASE + WORD, len(img) - WORD) == img[WORD:])
    report.check('nothing was written twice: one RAM word per 32 bytes less the first, no flash',
                 node.writes == (len(img) + WORD - 1) // WORD - 1 and node.programs == 0
                 and node.erases == 0)
    report.check('seal programs the record and writes the first word; the image is valid',
                 node.took(SEAL) == (True, '') and node.valid and node.state == SEALED
                 and node.read(RUN_BASE, WORD) == img[:WORD]
                 and node.read(RECORD_BASE, len(record)) == record)
    report.check('without persist the store is untouched: the record costs the only flash',
                 node.programs == len(record) // WORD and node.erases == 1
                 and node.read(STORE_BASE, 4) == b'\xff' * 4)
    report.check('state names the image RAM holds verified: its size and crc',
                 node.op(STATE)[-9:] == struct.pack('>IIB', len(img), zlib.crc32(img), 0))
    report.check('go for the session sets the jump; nothing is answered',
                 node.op(GO, struct.pack('>I', SESSION)) == b'' and node.go)
    report.check('dump pages the record back, 224 bytes at a time',
                 node.op(DUMP, struct.pack('>H', 224)) == struct.pack('>H', 224) + record[224:448])


def test_the_same_image_offered_again(report, node):
    """Nothing on the node is versioned: the master offers an image by
    its checksum and a record by its bytes, and a node holding exactly
    those keeps them and programs nothing."""
    img = image(5 * CHUNK + 40)
    record = bytes(range(200))
    session(node, img, record)
    node.took(SEAL)
    node.reboot()
    ok, crc = session(node, img, record)
    report.check('the image offered is the one RAM held through the reset: verified at once',
                 node.erases == 0 and ok == 1 and crc == zlib.crc32(img)
                 and node.state == VERIFIED and 'kept' in node.said, node.said)
    report.check('the stream landed on a whole image: nothing missing, nothing written',
                 missing(node) == [] and node.programs == 0 and node.writes == 0)
    report.check('seal with the record already in flash writes nothing and seals',
                 node.took(SEAL) == (True, '') and node.state == SEALED
                 and node.programs == 0 and node.erases == 0 and node.valid)
    report.check('go for the session sets the jump',
                 node.op(GO, struct.pack('>I', SESSION)) == b'' and node.go)
    node.reboot()
    session(node, img, bytes(range(200, 0, -1)))
    report.check('a different record with the same image: the record sector alone is rewritten',
                 node.took(SEAL) == (True, '') and node.erases == 1
                 and node.programs == (200 + WORD - 1) // WORD
                 and node.read(RECORD_BASE, 200) == bytes(range(200, 0, -1)))
    node.reboot()
    other = image(5 * CHUNK + 40, version=8)
    ok, crc = session(node, other, record)
    report.check('a different image of the same size: streamed into RAM as ever, no flash',
                 node.erases == 0 and ok == 1 and crc == zlib.crc32(other)
                 and node.writes == (len(other) + WORD - 1) // WORD - 1)


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
    writes = node.writes
    stream(node, img)                                  # everything again
    report.check('the repeats were not written again: only the three missing cost words',
                 node.writes == writes + 7 * 2 + 7 - 1 and missing(node) == [])
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
                 and node.read(RUN_BASE, 4) == b'\xff' * 4 and not node.valid)
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
    report.check('the next erase forgets it all: nothing held, no flash touched',
                 node.erases == 0 and missing(node) == list(range(30)) and node.state == ERASED)
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
    other.op(ERASE, struct.pack('>BIIH', TYPE, RUN_BYTES + 1, 0, 1))
    report.check('an image that does not fit is refused on the console',
                 other.erases == 0 and other.said == 'boot: an erase named an image that does not fit')


def test_the_debuggers_way_in(report, node):
    """An image written straight to RAM - a debugger's load - is valid by
    the four tests, and a wrong one is not."""
    img = image(5 * CHUNK)
    node.write(RUN_BASE, img)
    report.check('an image the debugger wrote is valid', node.valid)
    for what, bad in (('a stack pointer outside DTCM', struct.pack('<I', 0x30000000)),
                      ('an even reset vector', struct.pack('<II', 0x20010000, RUN_BASE + 0x1000)),):
        node.write(RUN_BASE, bad)
        report.check('%s is not an application' % what, not node.valid)
        node.write(RUN_BASE, img[:8])
    node.write(RUN_BASE + HEADER_OFFSET + 12, struct.pack('<I', TYPE + 1))
    report.check('another type\'s header is not an application', not node.valid)
    node.write(RUN_BASE + HEADER_OFFSET + 12, struct.pack('<I', TYPE))
    report.check('and stay from the bootloader itself is refused in words',
                 node.took(STAY) == (False, 'this node is in its bootloader already'))


def test_faults(report, node):
    img = image(6 * CHUNK)
    node.op(HOLD, struct.pack('>I', SESSION))
    node.op(ASSIGN, UID + bytes([2, 2, 0]))
    node.op(ERASE, erase_request(img))
    stream(node, img)
    node.op(VERIFY)
    node.lib.boot_h_fail_program(RUN_BASE)
    report.check('a first word that will not program leaves the image invalid, in words',
                 node.took(SEAL) == (False, 'the first word did not program - the image stays invalid')
                 and not node.valid)
    node.lib.boot_h_fail_program(0)
    report.check('a record for an unassigned node is refused in words',
                 Node(node.lib).took(RECORD, b'\x00\x00' + b'x' * 8)
                 == (False, 'a record goes to an assigned node - assign first'))
    report.check('an unknown op is refused in words',
                 node.took(99) == (False, 'no such boot op - PROTOCOL.md lists 0..12'))


def sealed(node, img, flags=b''):
    """An image through seal on a fresh session; the took pair."""
    session(node, img)
    return node.took(SEAL, flags)


def test_the_store(report, node):
    """Flash is written only where its seal names another image: persist
    copies RAM behind a seal written last, and a power-up with no master
    runs the store's copy after its CRC says it is whole."""
    img = image(40 * CHUNK + 9)
    words = (len(img) + WORD - 1) // WORD
    report.check('seal with persist writes the store: the image, then its seal',
                 sealed(node, img, bytes([PERSIST])) == (True, '')
                 and node.read(STORE_BASE + WORD, len(img)) == img
                 and struct.unpack('<IIII', node.read(STORE_BASE, 16))
                 == (SEAL_MAGIC, len(img), zlib.crc32(img), TYPE)
                 and node.programs == words + 1, '%d programs' % node.programs)
    node.reboot()
    report.check('the same image sealed with persist again writes nothing: the checksum is the gate',
                 sealed(node, img, bytes([PERSIST])) == (True, '')
                 and node.programs == 0 and node.erases == 0 and node.writes == 0)
    node.power_cycle()
    report.check('a power-up: RAM is noise, so the slot names nothing, and the store is copied',
                 not node.valid and node.ready() and node.valid
                 and node.read(RUN_BASE, len(img)) == img and node.writes == words
                 and node.image() == (len(img), zlib.crc32(img)))
    node.power_cycle()
    ok, _crc = session(node, img)
    report.check('a master offering the stored image: copied, verified at once, nothing streamed',
                 ok == 1 and 'stored' in node.said and node.writes == words
                 and node.programs == 0, node.said)
    node.write(STORE_BASE + WORD + 700, b'\x00')
    node.power_cycle()
    report.check('a torn store is not an image: nothing runs', not node.ready())
    node.write(STORE_BASE + WORD + 700, img[700:701])
    node.write(STORE_BASE, b'\xff' * WORD)
    node.power_cycle()
    report.check('a store with no seal holds nothing', not node.ready())


def test_a_warm_reset(report, node):
    """RAM survives a reset: the slot's size and crc name the image, and a
    CRC over RAM decides - no copy, no master."""
    img = image(12 * CHUNK)
    sealed(node, img)
    node.reboot()
    report.check('the slot names RAM\'s image: ready without a word written',
                 node.ready(len(img), zlib.crc32(img)) and node.writes == 0
                 and node.image() == (len(img), zlib.crc32(img)))
    node.reboot()
    report.check('a slot naming another crc falls to the store, and an empty store runs nothing',
                 not node.ready(len(img), zlib.crc32(img) ^ 1))


def test_persist_fails(report, node):
    img = image(8 * CHUNK)
    session(node, img)
    node.lib.boot_h_fail_program(STORE_BASE)
    report.check('a seal word that will not program is refused in words; the node stays verified',
                 node.took(SEAL, bytes([PERSIST]))
                 == (False, 'the store did not program - seal again, or without persist to run '
                            'from RAM alone') and node.state == VERIFIED)
    node.lib.boot_h_fail_program(0)
    report.check('and sealed again it persists, the RAM image already whole',
                 node.took(SEAL, bytes([PERSIST])) == (True, '') and node.state == SEALED
                 and struct.unpack('<I', node.read(STORE_BASE, 4))[0] == SEAL_MAGIC)


class Wire:
    """A board for the host's own client, coaxial.boot.Boot: each 0x6E
    request handed to boot_pdu as the bootloader's link hands it over."""

    unit = 2

    def __init__(self, lib):
        self.lib = lib

    def _pdu(self, payload):
        out = ctypes.create_string_buffer(253)
        n = self.lib.boot_h_pdu(bytes(payload), len(payload), out, 253)
        return n, out.raw[:max(n, 0)]

    def request(self, function, payload=b'', exact_payload=None, timeout=None,
                reply_shape=None):
        from coaxial import errors
        n, reply = self._pdu(payload)
        if n == -1:
            raise errors.NoReplyError('silence')
        if n < 0:
            raise errors.ModbusException(self.unit, function, 1 if n == -2 else 4)
        return reply

    def broadcast(self, function, payload=b'', settle=0.05):
        self._pdu(payload)


def test_the_hosts_client(report, node):
    """The master's client, byte for byte against the C: the replies carry
    the fields alone, as the application's do, and every shape parses."""
    from coaxial import errors
    from coaxial.boot import Boot
    boot = Boot(Wire(node.lib))
    img = image(30 * CHUNK + 5)
    record = bytes(range(250))
    boot.hold(SESSION)
    who = boot.who()
    report.check('who parses: the uid, the type, held, the blank unit',
                 who == {'uid': UID.hex(), 'type': TYPE, 'state': 'held', 'unit': UNIT_BLANK}, who)
    report.check('a prefix that is not this node is silence, None to the client',
                 boot.who(8, b'\xff') is None)
    report.check('assign takes', boot.assign(UID.hex(), 2, 2) is True)
    state = boot.flash(TYPE, img, record, persist=True)
    report.check('the whole sequence through the client, persisted: sealed, valid, the image named',
                 state['state'] == 'sealed' and state['valid'] and state['unit'] == 2
                 and state['image'] == (len(img), zlib.crc32(img)) and state['flags'] == 0, state)
    report.check('missing parses a bitmap: nothing missing', boot.missing() == [])
    report.check('dump parses a page of the record', boot.dump(224) == (224, record[224:250] + b'\xff' * 198))
    try:
        boot.stay()
        said = None
    except errors.RigError as exc:
        said = str(exc)
    report.check('stay is refused in the node\'s own words', said == 'this node is in its bootloader already', said)


class Bench:
    """A transport with one node on it: its application answering `unit`
    (device 11 state and stay) until stay, then the C bootloader, and after
    go the application again, running what the bootloader verified."""

    port, baud = 'bench', 115200

    def __init__(self, lib, unit, running):
        self.lib, self.unit, self.running = lib, unit, running
        self.in_app, self.frames = True, 0

    def _app(self, unit, payload):
        from coaxial import errors
        if unit != self.unit or payload[:1] != b'\x0b':
            raise errors.NoReplyError('silence')
        if payload[1] == STATE:
            return (bytes([SEALED, TYPE, self.unit, 3]) + struct.pack('>II', 0, 0) + b'\x01'
                    + UID + struct.pack('>II', *self.running) + b'\x01')
        self.in_app = False                     # stay: the reset, RAM kept
        self.lib.boot_h_reboot()
        return b'\x01'

    def _core(self, payload):
        out = ctypes.create_string_buffer(253)
        return self.lib.boot_h_pdu(bytes(payload), len(payload), out, 253), out

    def request(self, unit, function, payload=b'', exact_payload=None, timeout=None,
                reply_shape=None):
        from coaxial import errors
        self.frames += 1
        if self.in_app:
            return self._app(unit, payload)
        n, out = self._core(payload) if unit == self.lib.boot_h_unit() else (-1, None)
        if n == -1:
            raise errors.NoReplyError('silence')
        if n < 0:
            raise errors.ModbusException(unit, function, 4)
        return out.raw[:n]

    def broadcast(self, function, payload=b'', settle=0.05):
        self.frames += 1
        if not self.in_app:
            self._core(payload)
            if self.lib.boot_h_go():
                got = (ctypes.c_uint32 * 2)()
                self.lib.boot_h_image(got)
                self.running, self.in_app = tuple(got), True


def test_the_host_loads_its_image(report, node):
    """The old-firmware, new-host case: a node running another image takes
    the host's through its bootloader - its unit, position and termination
    given back, the store keeping it - and one running it already, or one
    a debugger started, is left alone."""
    from coaxial import boot
    from coaxial.board import Board
    boot.STAY_S, boot.GO_S = 0.0, 1.0
    img, old = image(20 * CHUNK + 3), image(20 * CHUNK + 3, version=6)
    bench = Bench(node.lib, 3, (len(old), zlib.crc32(old)))
    board = Board(bench, unit=3)
    report.check('another image running: loaded, and the application names the host\'s',
                 boot.ensure(board, img) is True
                 and bench.running == (len(img), zlib.crc32(img)) and bench.in_app)
    report.check('its unit, position and termination given back through assign',
                 node.unit == 3 and node.lib.boot_h_terminates() == 1
                 and node.op(STATE)[3] == 3)
    report.check('and the store keeps it, sealed',
                 struct.unpack('<III', node.read(STORE_BASE, 12))
                 == (SEAL_MAGIC, len(img), zlib.crc32(img)))
    frames = bench.frames
    report.check('running the host\'s image already: one state read, nothing loaded',
                 boot.ensure(board, img) is False and bench.frames == frames + 1)
    bench.running = (0, 0)
    report.check('started by a debugger (no image named): left alone',
                 boot.ensure(board, img) is False and bench.in_app)


def test_the_front_door_owns_the_image(report, node):
    """Coaxial63100.open()'s step on a real board: another image is loaded
    and said on stderr; a shared board is refused in words, not reset."""
    import contextlib
    import io as _io
    from coaxial import Coaxial63100, boot
    from coaxial.board import Board
    from coaxial.errors import RigError
    from coaxial.session import Origin
    boot.STAY_S, boot.GO_S = 0.0, 1.0
    img, old = image(12 * CHUNK), image(12 * CHUNK, version=5)
    real_host_image = boot.host_image
    boot.host_image = lambda: ('build/Debug/coaxial_63100.elf', img)
    try:
        for label, shared in (('COM9 - shared', True), ('COM9', False)):
            rig = Coaxial63100(port='COM9', unit=3)
            bench = Bench(node.lib, 3, (len(old), zlib.crc32(old)))
            rig._board = Board(bench, unit=3)
            rig._origin = Origin(True, 'COM9', 115200, None, label, 'debug probe', 3)
            said = _io.StringIO()
            try:
                with contextlib.redirect_stderr(said):
                    rig._board.probe = lambda: None
                    rig._own_image()
                refused = None
            except RigError as exc:
                refused = str(exc)
            if shared:
                report.check('a shared board running another image: refused in words, not reset',
                             refused is not None and 'other sessions share' in refused
                             and bench.in_app and bench.running[1] == zlib.crc32(old), refused)
            else:
                report.check('an unshared one: loaded, said on stderr, recorded',
                             refused is None and rig.image_loaded == ('build/Debug/coaxial_63100.elf', True)
                             and bench.running == (len(img), zlib.crc32(img))
                             and 'loading this host\'s build' in said.getvalue(), said.getvalue())
    finally:
        boot.host_image = real_host_image


def test_crc32_is_the_ieee_one(report, node):
    data = bytes(range(256)) * 9
    report.check('the core\'s CRC-32 is zlib\'s',
                 node.lib.boot_h_crc32(data, len(data)) == zlib.crc32(data))


ROSTER = (test_a_blank_node, test_an_image_streamed, test_the_same_image_offered_again,
          test_chunks_lost_and_resent,
          test_seal_before_verify_is_refused, test_the_master_dies,
          test_a_wrong_crc_and_a_wrong_type, test_the_debuggers_way_in, test_faults,
          test_the_store, test_a_warm_reset, test_persist_fails, test_the_hosts_client,
          test_the_host_loads_its_image, test_the_front_door_owns_the_image,
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
    lib.boot_h_writes.restype = ctypes.c_uint32
    lib.boot_h_ready.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
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
