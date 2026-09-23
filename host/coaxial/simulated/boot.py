"""A blank node in its bootloader: boot_core.c's rules over bytearray RAM and store (docs/BOOT.md).
"""
import struct
import zlib

from coaxial.devices.boot import (BLANK_UNIT, CHUNK, HEADER, MAGIC, RECORD_MAX, RUN_BASE,
                                  RUN_BYTES, SEAL_MAGIC, STATES, WORD, BootControl, Segment,
                                  chunks_of)
from coaxial.errors import CrcError, DeviceStateError

UID = bytes(range(0x10, 0x1C))
TYPE = 1


def _refused(words):
    raise DeviceStateError(words)


class SimulatedBoot(BootControl):
    """One blank node: the state machine over bytearray RAM, where the image
    runs, and a store - the seal word first, the image behind it."""

    def __init__(self, type_=TYPE, uid=UID):
        self.type = type_
        self.uid = uid
        self.ram = bytearray(b'\xff' * RUN_BYTES)
        self.store = bytearray(b'\xff' * (WORD + RUN_BYTES))
        self.record_sector = bytearray(b'\xff' * RECORD_MAX)
        self.stores = 0                   # how often the store was written
        self.reboot()

    def reboot(self):
        """A reset: the state gone, RAM and flash kept."""
        self._state = 0
        self.unit = BLANK_UNIT
        self.position = 0
        self.flags = 0
        self.session = 0
        self.size = self.crc = 0
        self.held = set()
        self.chunks = 0
        self.first = None                 # the first word, held back
        self.verified = False
        self.record_bytes = bytearray()
        self.jumped = False
        self.image = (0, 0)               # what RAM holds verified

    # -- what the node holds ---------------------------------------------

    def valid(self, memory=None):
        memory = self.ram if memory is None else memory
        sp, reset = struct.unpack_from('<II', memory, 0)
        magic, size, _v, type_ = struct.unpack_from('<IIII', memory, HEADER)
        return (0x20000000 < sp <= 0x20020000 and reset & 1
                and RUN_BASE < reset < RUN_BASE + RUN_BYTES
                and magic == MAGIC and type_ == self.type
                and HEADER < size <= RUN_BYTES)

    def _image_crc(self):
        head = self.first if self.first is not None else self.ram[:WORD]
        return zlib.crc32(bytes(head) + bytes(self.ram[WORD:self.size]))

    def stored(self):
        """(size, crc) the store's seal names over a whole copy, or None."""
        magic, size, crc, type_ = struct.unpack_from('<IIII', self.store, 0)
        copy = memoryview(self.store)[WORD:WORD + size]
        if (magic != SEAL_MAGIC or type_ != self.type or not HEADER < size <= RUN_BYTES
                or not self.valid(copy) or zlib.crc32(copy) != crc):
            return None
        return size, crc

    # -- the ops -----------------------------------------------------------

    def state(self):
        return {'state': STATES[self._state], 'type': self.type, 'unit': self.unit,
                'position': self.position, 'held': len(self.held), 'of': self.chunks,
                'valid': bool(self.valid()), 'uid': self.uid.hex(),
                'image': self.image, 'flags': self.flags}

    def stay(self):
        _refused('this node is in its bootloader already')

    def hold(self, session):
        if self._state == 0:
            self._state = 1
        self.session = session

    def who(self, bits=0, prefix=b''):
        mine = int.from_bytes(self.uid, 'big') >> (96 - bits) if bits else 0
        theirs = int.from_bytes(bytes(prefix).ljust(12, b'\0'), 'big') >> (96 - bits) if bits else 0
        if mine != theirs:
            return None
        return {'uid': self.uid.hex(), 'type': self.type,
                'state': STATES[self._state], 'unit': self.unit}

    def assign(self, uid, unit, position, terminate=False):
        if bytes.fromhex(uid) != self.uid:
            return None
        if unit == 0 or unit >= BLANK_UNIT:
            _refused('a unit is 1..246 - 0 is broadcast and 247 is every blank node')
        self.unit, self.position = unit, position
        self.flags = 1 if terminate else 0
        self._state = 2

    def erase(self, type_, image):
        if type_ != self.type or self._state < 2:
            return
        self.size, self.crc, self.chunks = len(image), zlib.crc32(image), len(chunks_of(image))
        self.first, self.record_bytes = None, bytearray()
        held_size = struct.unpack_from('<I', self.ram, HEADER + 4)[0]
        kept = self.valid() and held_size == self.size and self._image_crc() == self.crc
        if not kept and self.stored() == (self.size, self.crc):
            self.ram[:self.size] = self.store[WORD:WORD + self.size]
            kept = True                                 # copied: nothing streams
        if kept:
            self.held = set(range(self.chunks))
            self.verified, self._state = True, 4
            return
        self.ram[:self.size] = b'\xff' * self.size
        self.held, self.verified, self._state = set(), False, 3

    def chunk(self, index, data):
        if self._state not in (3, 4) or index >= self.chunks or index in self.held:
            return
        at = index * CHUNK
        if at == 0:
            self.first = bytes(data[:WORD]).ljust(WORD, b'\xff')
            self.ram[WORD:at + len(data)] = data[WORD:]
        else:
            self.ram[at:at + len(data)] = data
        self.held.add(index)

    def missing(self):
        return [i for i in range(self.chunks) if i not in self.held]

    def verify(self):
        if self._state < 3 or len(self.held) != self.chunks:
            return False, 0
        crc = self._image_crc()
        self.verified = crc == self.crc
        if self.verified:
            self._state = 4
        return self.verified, crc

    def record(self, offset, data):
        if self._state < 2:
            _refused('a record goes to an assigned node - assign first')
        if offset + len(data) > RECORD_MAX:
            _refused('the record is at most 2048 bytes')
        self.record_bytes[offset:offset + len(data)] = data

    def seal(self, persist=False):
        if not self.verified:
            _refused('the image is not verified - verify first, and it must say ok')
        if self.record_bytes:
            padded = bytes(self.record_bytes).ljust(-(-len(self.record_bytes) // WORD) * WORD, b'\xff')
            if self.record_sector[:len(padded)] != padded:
                self.record_sector[:] = b'\xff' * RECORD_MAX
                self.record_sector[:len(padded)] = padded
        if self.first is not None:
            self.ram[:WORD] = self.first
            self.first = None
        if persist and self.stored() != (self.size, self.crc):
            self.store[:] = b'\xff' * len(self.store)
            self.store[WORD:WORD + self.size] = self.ram[:self.size]
            self.store[:16] = struct.pack('<IIII', SEAL_MAGIC, self.size, self.crc, self.type)
            self.stores += 1
        self.image = (self.size, self.crc)
        self._state = 5

    def go(self, session):
        if session == self.session and self._state == 5 and self.valid():
            self.jumped = True

    def dump(self, offset):
        return offset, bytes(self.record_sector[offset:offset + CHUNK])


class SimulatedSegment(Segment):
    """A bus of blank nodes."""

    def __init__(self, nodes):
        self.nodes = list(nodes)

    def blank(self):
        return _Blank(self.nodes)

    def at(self, unit):
        for node in self.nodes:
            if node.unit == unit:
                return node
        raise DeviceStateError('no node answers to unit %d on this segment' % unit)


class _Blank(BootControl):
    """Unit 247's voice: the broadcasts reach every node, `who` and `assign`
    the nodes the prefix or the uid names, and the rest whichever nodes
    are still blank - one answers, two collide.
    """

    def __init__(self, nodes):
        self.nodes = nodes

    def _one(self, answers):
        answers = [a for a in answers if a is not None]
        if len(answers) > 1:
            raise CrcError('%d nodes answered at once' % len(answers))
        return answers[0] if answers else None

    def _blank(self):
        return self._one([n for n in self.nodes if n.unit == BLANK_UNIT])

    def state(self):
        return self._blank().state()

    def stay(self):
        return self._blank().stay()

    def hold(self, session):
        for node in self.nodes:
            node.hold(session)

    def who(self, bits=0, prefix=b''):
        return self._one([n.who(bits, prefix) for n in self.nodes])

    def assign(self, uid, unit, position, terminate=False):
        for node in self.nodes:
            node.assign(uid, unit, position, terminate)

    def erase(self, type_, image):
        for node in self.nodes:
            node.erase(type_, image)

    def chunk(self, index, data):
        for node in self.nodes:
            node.chunk(index, data)

    def missing(self):
        return self._blank().missing()

    def verify(self):
        return self._blank().verify()

    def record(self, offset, data):
        return self._blank().record(offset, data)

    def seal(self, persist=False):
        return self._blank().seal(persist)

    def go(self, session):
        for node in self.nodes:
            node.go(session)

    def dump(self, offset):
        return self._blank().dump(offset)
