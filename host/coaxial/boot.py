"""Device 11: the bootloader, as the master speaks it (docs/BOOT.md).

`BootControl` is the interface - the ops a node in its bootloader
serves, and the two a running application serves (`state`, `stay`) -
with a real and a simulated implementation, so a name drifting between
them fails at construction. `Boot` is the real one: each op is one 0x6E
frame, the broadcasts answered by nobody. `flash()` is the master's
sequence on one assigned node: erase, every chunk, the missing ones
again, verify, the record, seal - the node keeps an image it already
holds and programs nothing, which is the principle the design rests on.
"""
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
#: A blank node's unit, before assign gives it its own.
BLANK_UNIT = 247
STATES = ('blank', 'held', 'assigned', 'erased', 'verified', 'sealed')
#: How many times the missing chunks are re-sent before the node is named.
MISSING_ROUNDS = 3


def chunks_of(image):
    return [image[i:i + CHUNK] for i in range(0, len(image), CHUNK)]


class BootControl(ABC):
    """Device 11's ops, as a node in its bootloader and a running
    application serve them - the seam both implementations meet."""

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
        """The master's sequence on one node, after assign; returns the
        state. Raises with the node's words where it refuses."""
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
