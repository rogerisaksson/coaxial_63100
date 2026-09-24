"""The board's measurement ring, drained in bursts."""
from coaxial.comm import protocol
from coaxial.comm.protocol import LogOp
from coaxial.comm.wire import Reader, pack
from coaxial.devices.subsystem import Device
from coaxial.errors import RigError
from machine.roles import Stream

#: Source ids, and what `v` means for each.
PHASES = 0      #: v = U, V, W, TIM1->CNT at latch
ANGLE = 1       #: v = value, crc, register
IMU = 2         #: v = quaternion i, j, k, real
DRIVE = 3       #: v = id, iq in 10 mA, theta_hat as a turn in 65536,
                #: the innovation in 0.1 mrad

NAMES = {PHASES: 'phases', ANGLE: 'angle', IMU: 'imu', DRIVE: 'drive'}
BY_NAME = {v: k for k, v in NAMES.items()}

MAX_BURST = 15

#: The four i16 every source fills, whatever it means by them.
WORDS = 4

#: Wire size of one record - u32 at, u8 source, u8 seq, 4x i16 - which
#: is what `take` parses below and not what the struct occupies in the
#: board's RAM. 15 of them plus the count is 211 bytes, inside 253.
RECORD_BYTES = 14


def _mask(sources):
    """A source mask from names, or the mask itself when given one."""
    if isinstance(sources, int):
        return sources
    unknown = [s for s in sources if s not in BY_NAME]
    if unknown:
        raise ValueError('no such source: %s - have %s'
                         % (', '.join(unknown), ', '.join(BY_NAME)))
    mask = 0
    for s in sources:
        mask |= 1 << BY_NAME[s]
    return mask


class Ring(Stream):

    """The board's capture ring: sources started, records read oldest first."""

    DEPTH = 1024

    def start(self, *sources):
        """Arm `sources` (names, or one raw mask) and empty the ring."""
        one = len(sources) == 1 and not isinstance(sources[0], str)
        return self._arm(_mask(sources[0] if one else list(sources)))

    def stop(self):
        """Disarm every source; the ring empties with them."""
        return self._arm(0)

    def read(self, count=None, timeout=None):
        """What is waiting, oldest first, up to `count` (default: a ring's depth)."""
        cap = self.DEPTH if count is None else count
        out = []
        while len(out) < cap:
            batch = self.take()
            if not batch:
                break
            out.extend(batch)
        return out[:cap]


class Capture(Device, Ring, device=protocol.DEVICE_LOG):

    """The ring on the board: `0x6E` device LOG."""

    def state(self):
        """What is armed, how much is waiting, and how much did not make it."""
        r = Reader(self._op(LogOp.STATE))
        mask = r.u8()
        return {
            'sources': [NAMES[i] for i in sorted(NAMES) if mask >> i & 1],
            'mask': mask,
            'count': r.u16(),
            'depth': r.u16(),
            'dropped': r.u32(),
            'thinned': r.u32(),
        }

    def _arm(self, mask):
        if not Reader(self._op(LogOp.ARM, pack(('u8', mask)))).u8():
            raise RigError('the board refused to arm the capture ring')
        return True

    def take(self, want=MAX_BURST):
        """Up to `want` records, oldest first, freed from the ring as they go."""
        want = max(1, min(int(want), MAX_BURST))
        r = Reader(self._op(LogOp.TAKE, pack(('u8', want))))
        return [self._record(r) for _ in range(r.u8())]

    @staticmethod
    def _record(r):
        return {'at': r.u32(),
                'source': NAMES.get(r.u8(), '?'),
                'seq': r.u8(),
                'v': tuple(r.i16() for _ in range(WORDS))}

