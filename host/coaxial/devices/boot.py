"""Device 11: the bootloader, as the master speaks it (docs/BOOT.md)."""
import os
import struct
import time
import zlib
from abc import ABC, abstractmethod
from pathlib import Path

from coaxial import errors
from coaxial.comm import protocol
from coaxial.comm.protocol import BootOp
from coaxial.devices.subsystem import Device
from coaxial.comm.transport import ACK
from coaxial.comm.wire import Reader

#: The wire's shapes, boot.h's numbers.
CHUNK = 224
UID_BYTES = 12
WORD = 32
HEADER = 0x400
MAGIC = 0x50415843
#: Where the image runs (D2 SRAM) and where its sealed copy is kept (flash):
#: the seal word first, the image behind it.
RUN_BASE = 0x30000000
RUN_BYTES = 0x48000
STORE_BASE = 0x08020000
STORE_BYTES = 0x1C0000
SEAL_MAGIC = 0x4C414553
PERSIST = 0x01
RECORD_BASE = 0x081E0000
RECORD_MAX = 2048
#: A blank node's unit, before assign gives it its own; the bootloader's
#: line rate, the one the master's transport opens at.
BLANK_UNIT = 247
BOOT_BAUD = 10000000
STATES = ('blank', 'held', 'assigned', 'erased', 'verified', 'sealed')
#: How many times the missing chunks are re-sent before the node is named.
MISSING_ROUNDS = 3
#: The bootloader works inside its receive path and hears nothing meanwhile,
#: so the master waits (estimates, seconds): erase's two CRCs over RAM and
#: the store and a copy between them, a chunk's frame gap (t3.5 is 1.75 ms
#: above 19 200 baud), the CRC over a full image, seal's record-sector erase,
#: and persist's store - three sectors at the datasheet's worst 4 s each.
SECTOR = 128 * 1024
ERASE_S = 0.3
CHUNK_S = 0.002
VERIFY_S = 2.0
SEAL_S = 3.0
PERSIST_S = 12.0
#: A running node sent `stay`: its reply out, 50 ms, the reset, and the
#: bootloader's gate over RAM (~0.1 s at 160 MHz). After `go`: the jump and
#: the application's init, polled for this long until it answers.
STAY_S = 0.5
GO_S = 5.0
#: The image this host was built with: $COAXIAL_IMAGE (an ELF or a raw
#: image), else the newest build/<preset>/coaxial_63100.elf beside host/.
IMAGE_ENV = 'COAXIAL_IMAGE'
BUILD = Path(__file__).resolve().parents[3] / 'build'
APP_ELF = 'coaxial_63100.elf'


def chunks_of(image):
    return [image[i:i + CHUNK] for i in range(0, len(image), CHUNK)]


def image_of(elf):
    """The image the bootloader takes, cut from a linked ELF: every loaded
    segment inside RUN at its load address, gaps 0xFF, as long as its
    header says - exact where objcopy's binary would span an empty
    section's DTCM address."""
    data = Path(elf).read_bytes()
    if data[:4] != b'\x7fELF' or data[4] != 1 or data[5] != 1:
        raise errors.RigError('%s is not a 32-bit little-endian ELF' % elf)
    phoff = struct.unpack_from('<I', data, 0x1C)[0]
    size, count = struct.unpack_from('<HH', data, 0x2A)
    body = bytearray(b'\xff' * RUN_BYTES)
    for i in range(count):
        kind, offset, _vaddr, paddr, filesz = struct.unpack_from('<5I', data, phoff + i * size)
        if kind == 1 and filesz and RUN_BASE <= paddr < RUN_BASE + RUN_BYTES:
            body[paddr - RUN_BASE:paddr - RUN_BASE + filesz] = data[offset:offset + filesz]
    magic, length = struct.unpack_from('<II', body, HEADER)
    if magic != MAGIC or not HEADER < length <= RUN_BYTES:
        raise errors.RigError('%s holds no image linked at 0x%08x - an application built '
                              'before it ran from RAM?' % (elf, RUN_BASE))
    return bytes(body[:length])


def store_of(image):
    """What flash keeps at STORE_BASE: the seal word (magic, size, crc, the
    header's type), then the image - what a debugger programs."""
    type_ = struct.unpack_from('<I', image, HEADER + 12)[0]
    seal = struct.pack('<IIII', SEAL_MAGIC, len(image), zlib.crc32(image), type_)
    return seal.ljust(WORD, b'\xff') + bytes(image)


class BootControl(ABC):
    """Device 11's ops, as a node in its bootloader and a running application
    serve them - the seam both implementations meet.
    """

    @abstractmethod
    def state(self) -> dict:
        """{'state', 'type', 'unit', 'position', 'held', 'of', 'valid', 'uid',
        'image', 'flags'} - image is (bytes, crc) of what RAM holds verified,
        (0, 0) for none; image and flags None from a node older than MINOR 19."""

    @abstractmethod
    def stay(self):
        """A running application back into its bootloader."""

    @abstractmethod
    def hold(self, session):
        """Broadcast: every node in its window stays."""

    @abstractmethod
    def who(self, bits=0, prefix=b''):
        """The node whose uid begins with the prefix, or None for silence."""

    @abstractmethod
    def assign(self, uid, unit, position, terminate=False):
        """This uid its unit and position; the last node closes the termination."""

    @abstractmethod
    def erase(self, type_, image):
        """Broadcast: every node of the type takes the image's shape."""

    @abstractmethod
    def chunk(self, index, data):
        """Broadcast: one chunk."""

    @abstractmethod
    def missing(self) -> list:
        """The chunk indices not yet held."""

    @abstractmethod
    def verify(self) -> tuple:
        """(ok, crc) over the image as it will stand."""

    @abstractmethod
    def record(self, offset, data):
        """One page of the record into the node's RAM."""

    @abstractmethod
    def seal(self, persist=False):
        """The record and the first word programmed; the image valid - and
        with `persist`, kept in the store unless it holds this image."""

    @abstractmethod
    def go(self, session):
        """Broadcast: every sealed node of the session jumps."""

    @abstractmethod
    def dump(self, offset):
        """(offset, bytes) - one page of the record sector."""

    def flash(self, type_, image, record=b'', persist=False):
        """The master's sequence on one node, after assign; returns the
        state.
        """
        self.erase(type_, image)
        for index, piece in enumerate(chunks_of(image)):
            self.chunk(index, piece)
        for _round in range(MISSING_ROUNDS):
            left = self.missing()
            if not left:
                break
            for index in left:
                self.chunk(index, chunks_of(image)[index])
        ok, crc = self.verify()
        if not ok:
            raise errors.DeviceStateError(
                'the image did not verify: the node summed %08x against %08x'
                % (crc, zlib.crc32(image)))
        for at in range(0, len(record), CHUNK):
            self.record(at, record[at:at + CHUNK])
        self.seal(persist)
        return self.state()


class Boot(Device, BootControl, device=protocol.DEVICE_BOOT):
    """The real one: each op one 0x6E frame on this unit's link."""

    def _data(self, op, payload=b'', fits=len, **kwargs):
        """A bootloader op's data, where it `fits` its shape. The application refuses the
        op in words (`u8 took` 0, a string - MINOR 18): raised in them; any other misfit is
        a PayloadError."""
        reply = self._op(op, payload, **kwargs)
        if fits(reply):
            return Reader(reply)
        if reply[:1] == b'\x00' and len(reply) > 1 and len(reply) == 2 + reply[1]:
            raise errors.DeviceStateError(Reader(reply[1:]).string())
        raise errors.PayloadError('boot op %d answered %d bytes it cannot be: %s'
                                  % (op, len(reply), reply[:16].hex(' ')))

    def state(self):
        r = Reader(self._op(BootOp.STATE))
        state, type_, unit, position = r.u8(), r.u8(), r.u8(), r.u8()
        held, of, valid = r.u32(), r.u32(), bool(r.u8())
        uid = r.take(UID_BYTES).hex()
        # MINOR 19: the image RAM holds verified, and assign's flags; an
        # older node says nothing.
        image = (r.maybe('u32'), r.maybe('u32'))
        return {'state': STATES[state], 'type': type_, 'unit': unit,
                'position': position, 'held': held, 'of': of, 'valid': valid,
                'uid': uid, 'image': image, 'flags': r.maybe('u8')}

    def stay(self):
        return self._ack(BootOp.STAY)

    def hold(self, session):
        self._broadcast(BootOp.HOLD, struct.pack('>I', session))

    def who(self, bits=0, prefix=b''):
        try:
            r = self._data(BootOp.WHO, bytes([bits]) + bytes(prefix),
                           fits=lambda reply: len(reply) == UID_BYTES + 3)
        except errors.NoReplyError:
            return None                     # no node's uid begins so
        uid = r.take(UID_BYTES).hex()
        return {'uid': uid, 'type': r.u8(), 'state': STATES[r.u8()], 'unit': r.u8()}

    def assign(self, uid, unit, position, terminate=False):
        return self._ack(BootOp.ASSIGN, bytes.fromhex(uid)
                         + bytes([unit, position, 1 if terminate else 0]))

    def erase(self, type_, image):
        self._broadcast(BootOp.ERASE, struct.pack(
            '>BIIH', type_, len(image), zlib.crc32(image), len(chunks_of(image))),
            settle=ERASE_S)

    def chunk(self, index, data):
        self._broadcast(BootOp.CHUNK, struct.pack('>H', index) + bytes(data),
                        settle=CHUNK_S)

    def missing(self):
        r = self._data(BootOp.MISSING, fits=lambda reply: len(reply) >= 4 and len(reply) >= (
            4 + (int.from_bytes(reply[2:4], 'big') + 7) // 8))
        first, count = r.u16(), r.u16()
        bitmap = r.take(r.remaining)
        return [i for i in range(first, count) if not bitmap[i // 8] >> (i % 8) & 1]

    def verify(self):
        r = self._data(BootOp.VERIFY, fits=lambda reply: len(reply) == 5, timeout=VERIFY_S)
        return bool(r.u8()), r.u32()

    def record(self, offset, data):
        return self._ack(BootOp.RECORD, struct.pack('>H', offset) + bytes(data))

    def seal(self, persist=False):
        return self.took(self._op(BootOp.SEAL, bytes([PERSIST]) if persist else b'',
                                  reply_shape=ACK,
                                  timeout=SEAL_S + (PERSIST_S if persist else 0.0)))

    def go(self, session):
        self._broadcast(BootOp.GO, struct.pack('>I', session))

    def dump(self, offset):
        r = self._data(BootOp.DUMP, struct.pack('>H', offset),
                       fits=lambda reply: reply[:2] == struct.pack('>H', offset))
        return r.u16(), r.take(r.remaining)


class Segment(ABC):
    """One serial segment as the master sees it: the blank nodes at unit 247
    as one voice, and any node by its unit.
    """

    @abstractmethod
    def blank(self):
        """A BootControl addressed to unit 247 - every blank node hears it, and
        only the node a prefix names answers.
        """

    @abstractmethod
    def at(self, unit):
        """A BootControl addressed to one unit."""


class TransportSegment(Segment):
    """A segment over one transport: a Board per unit, built on demand."""

    def __init__(self, transport):
        self._transport = transport
        self._boards = {}

    def _boot(self, unit):
        if unit not in self._boards:
            from coaxial.devices.board import Board
            self._boards[unit] = Board(self._transport, unit=unit)
        return self._boards[unit].boot

    def blank(self):
        return self._boot(BLANK_UNIT)

    def at(self, unit):
        return self._boot(unit)


def enumerate_blank(blank):
    """Every blank node on a segment, by the prefix search on the unique id:
    `who` with no prefix first; two nodes answering at once is a CRC
    error or a frame error on the wire, and either splits the prefix one
    bit deeper, 96 at most.
    """
    found, todo = [], [(0, 0)]
    while todo:
        bits, prefix = todo.pop()
        head = (prefix << (UID_BYTES * 8 - bits)).to_bytes(UID_BYTES, 'big')[:(bits + 7) // 8]
        try:
            node = blank.who(bits, head)
        except (errors.CrcError, errors.FrameError):
            if bits < UID_BYTES * 8:
                todo += [(bits + 1, prefix << 1), (bits + 1, prefix << 1 | 1)]
            continue
        if node is not None:
            found.append(node)
    return sorted(found, key=lambda n: n['uid'])


class Master:
    """The master on one segment (docs/BOOT.md): hold, enumerate, assign off
    the table, one erase and one stream per type, then each node's
    missing, verify, record and seal, then go.
    """

    def __init__(self, segment, table, images, records, session=1, persist=False):
        self.segment = segment
        self.table = table
        self.images = images
        self.records = records
        self.session = session
        self.persist = persist
        self.unknown = []

    def run(self):
        """Every node through to go; returns {unit: state}."""
        blank = self.segment.blank()
        blank.hold(self.session)
        nodes = enumerate_blank(blank)
        assigned = {}
        for node in nodes:
            row = self.table.get(node['uid'])
            if row is None:
                self.unknown.append(node['uid'])
                continue
            blank.assign(node['uid'], row['unit'], row['position'], row.get('terminate', False))
            assigned[row['unit']] = row['type']
        for type_ in sorted(set(assigned.values())):
            image = self.images[type_]
            blank.erase(type_, image)
            for index, piece in enumerate(chunks_of(image)):
                blank.chunk(index, piece)
        states = {}
        for unit, type_ in sorted(assigned.items()):
            node = self.segment.at(unit)
            image = self.images[type_]
            for _round in range(MISSING_ROUNDS):
                left = node.missing()
                if not left:
                    break
                for index in left:
                    node.chunk(index, chunks_of(image)[index])
            ok, crc = node.verify()
            if not ok:
                raise errors.DeviceStateError(
                    'unit %d did not verify: it summed %08x against %08x'
                    % (unit, crc, zlib.crc32(image)))
            record = self.records.get(unit, b'')
            for at in range(0, len(record), CHUNK):
                node.record(at, record[at:at + CHUNK])
            node.seal(self.persist)
        blank.go(self.session)
        for unit in assigned:
            states[unit] = self.segment.at(unit).state()
        return states


def host_image():
    """(path, image) - what this host was built with, or None where no
    build is at hand (a fresh clone, CI's host job)."""
    named = os.environ.get(IMAGE_ENV)
    if named:
        path = Path(named)
        return path, image_of(path) if path.suffix == '.elf' else path.read_bytes()
    built = sorted(BUILD.glob('*/' + APP_ELF), key=lambda p: p.stat().st_mtime)
    return (built[-1], image_of(built[-1])) if built else None


def load(board, image, persist=True, session=0x10AD):
    """The node behind `board` onto `image` through its bootloader: `stay`,
    then the master's sequence on this one node at the blank unit - its
    unit, position and flags given back - then `go`, and the application
    polled until it names the image. With `persist` the store keeps it for a
    power-up with no host. Returns the application's state."""
    from coaxial.devices.board import Board
    was = board.boot.state()
    board.boot.stay()
    time.sleep(STAY_S)
    blank = Board(board.transport, unit=BLANK_UNIT).boot
    blank.hold(session)
    node = blank.who()
    if node is None:
        raise errors.NoReplyError('unit %d sent stay, and no bootloader answered at unit %d'
                                  % (board.unit, BLANK_UNIT))
    blank.assign(node['uid'], was['unit'], was['position'],
                 terminate=bool((was.get('flags') or 0) & 1))
    Board(board.transport, unit=was['unit']).boot.flash(was['type'], image, persist=persist)
    blank.go(session)
    want = (len(image), zlib.crc32(image))
    until = time.monotonic() + GO_S
    while True:
        try:
            state = board.boot.state()
            if state['image'] == want:
                return state
        except errors.RigError:
            pass                          # still booting
        if time.monotonic() > until:
            raise errors.DeviceStateError(
                'unit %d took the image (%d B, crc %08x) and did not come back running it '
                'within %.0f s' % (board.unit, want[0], want[1], GO_S))
        time.sleep(0.1)


def stale(board, image):
    """The image `board` runs when it is not `image` - None when it is, or
    when a debugger started it (no bootloader named one: (0, 0))."""
    running = board.boot.state()['image']
    return None if running in ((len(image), zlib.crc32(image)), (0, 0)) else running


def ensure(board, image, persist=True):
    """Whether `board` had to be loaded with `image` (`stale`, then `load`)."""
    if stale(board, image) is None:
        return False
    load(board, image, persist)
    return True
