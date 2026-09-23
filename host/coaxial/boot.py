"""Device 11: the bootloader, as the master speaks it (docs/BOOT.md)."""
import struct
import zlib
from abc import ABC, abstractmethod

from . import errors, protocol
from .protocol import BootOp
from .subsystem import Device
from .wire import Reader

#: The wire's shapes, boot.h's numbers.
CHUNK = 224
UID_BYTES = 12
WORD = 32
HEADER = 0x400
MAGIC = 0x50415843
APP_BASE = 0x08020000
APP_BYTES = 0x1C0000
RECORD_BASE = 0x081E0000
RECORD_MAX = 2048
#: A blank node's unit, before assign gives it its own; the bootloader's
#: line rate, the one the master's transport opens at.
BLANK_UNIT = 247
BOOT_BAUD = 10000000
STATES = ('blank', 'held', 'assigned', 'erased', 'verified', 'sealed')
#: How many times the missing chunks are re-sent before the node is named.
MISSING_ROUNDS = 3


def chunks_of(image):
    return [image[i:i + CHUNK] for i in range(0, len(image), CHUNK)]


class BootControl(ABC):
    """Device 11's ops, as a node in its bootloader and a running application
    serve them - the seam both implementations meet.
    """

    @abstractmethod
    def state(self):
        """{'state', 'type', 'unit', 'position', 'held', 'of', 'valid', 'uid'}."""

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
    def missing(self):
        """The chunk indices not yet held."""

    @abstractmethod
    def verify(self):
        """(ok, crc) over the image as it will stand."""

    @abstractmethod
    def record(self, offset, data):
        """One page of the record into the node's RAM."""

    @abstractmethod
    def seal(self):
        """The record and the first word programmed; the image valid."""

    @abstractmethod
    def go(self, session):
        """Broadcast: every sealed node of the session jumps."""

    @abstractmethod
    def dump(self, offset):
        """(offset, bytes) - one page of the record sector."""

    def flash(self, type_, image, record=b''):
        """The master's sequence on one node, after assign; returns the state.
        Raises with the node's words where it refuses.
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
        self.seal()
        return self.state()


class Boot(Device, BootControl, device=protocol.DEVICE_BOOT):
    """The real one: each op one 0x6E frame on this unit's link."""

    def state(self):
        r = Reader(self._op(BootOp.STATE))
        state, type_, unit, position = r.u8(), r.u8(), r.u8(), r.u8()
        held, of, valid = r.u32(), r.u32(), bool(r.u8())
        return {'state': STATES[state], 'type': type_, 'unit': unit,
                'position': position, 'held': held, 'of': of, 'valid': valid,
                'uid': r.take(UID_BYTES).hex()}

    def stay(self):
        return self._ack(BootOp.STAY)

    def hold(self, session):
        self._broadcast(BootOp.HOLD, struct.pack('>I', session))

    def who(self, bits=0, prefix=b''):
        try:
            r = Reader(self._op(BootOp.WHO, bytes([bits]) + bytes(prefix)))
        except errors.NoReplyError:
            return None                     # no node's uid begins so
        uid = r.take(UID_BYTES).hex()
        return {'uid': uid, 'type': r.u8(), 'state': STATES[r.u8()], 'unit': r.u8()}

    def assign(self, uid, unit, position, terminate=False):
        return self._ack(BootOp.ASSIGN, bytes.fromhex(uid)
                         + bytes([unit, position, 1 if terminate else 0]))

    def erase(self, type_, image):
        self._broadcast(BootOp.ERASE, struct.pack(
            '>BIIH', type_, len(image), zlib.crc32(image), len(chunks_of(image))))

    def chunk(self, index, data):
        self._broadcast(BootOp.CHUNK, struct.pack('>H', index) + bytes(data))

    def missing(self):
        r = Reader(self._op(BootOp.MISSING))
        first, count = r.u16(), r.u16()
        bitmap = r.remaining()
        return [i for i in range(first, count) if not bitmap[i // 8] >> (i % 8) & 1]

    def verify(self):
        r = Reader(self._op(BootOp.VERIFY))
        return bool(r.u8()), r.u32()

    def record(self, offset, data):
        return self._ack(BootOp.RECORD, struct.pack('>H', offset) + bytes(data))

    def seal(self):
        return self._ack(BootOp.SEAL)

    def go(self, session):
        self._broadcast(BootOp.GO, struct.pack('>I', session))

    def dump(self, offset):
        r = Reader(self._op(BootOp.DUMP, struct.pack('>H', offset)))
        return r.u16(), r.remaining()


class Segment(ABC):
    """One serial segment as the master sees it: the blank nodes at unit 247 as
    one voice, and any node by its unit. The real one is a transport; the
    stand-in's is a list of nodes.
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
            from .board import Board
            self._boards[unit] = Board(self._transport, unit=unit)
        return self._boards[unit].boot

    def blank(self):
        return self._boot(BLANK_UNIT)

    def at(self, unit):
        return self._boot(unit)


def enumerate_blank(blank):
    """Every blank node on a segment, by the prefix search on the unique id:
    `who` with no prefix first; two nodes answering at once is a CRC error
    or a frame error on the wire, and either splits the prefix one bit
    deeper, 96 at most. Known uids are one round trip; a bus of N unknown
    nodes is about 2N.
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
    the table, one erase and one stream per type, then each node's missing,
    verify, record and seal, then go. `table` maps a uid to {'unit',
    'position', 'type', 'terminate'}; `images` maps a type to its bytes;
    `records` maps a unit to the record's bytes.
    """

    def __init__(self, segment, table, images, records, session=1):
        self.segment = segment
        self.table = table
        self.images = images
        self.records = records
        self.session = session
        self.unknown = []

    def run(self):
        """Every node through to go; returns {unit: state}. A uid not in the
        table is left blank and listed in `unknown`.
        """
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
            node.seal()
        blank.go(self.session)
        for unit in assigned:
            states[unit] = self.segment.at(unit).state()
        return states
