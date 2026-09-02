"""The board's measurement ring, drained in bursts.

One sample per round trip caps a host at a couple of hundred samples a
second whatever the board managed - a 53-byte reply at 115200 is 4.6 ms.
This takes fifteen at a time, so the board's own rate is the only limit
left.

Every record carries the raw codes and the cycle counter that stamped them.
Nothing is converted here: `at` is raw CYCCNT because dividing cycles down
moves the wrap off a power of two, and `v` is whatever the source put there.
"""
from . import protocol
from .errors import RigError
from .subsystem import Subsystem
from .wire import Reader

#: Source ids, and what `v` means for each.
PHASES = 0      #: v = U, V, W, TIM1->CNT at latch
ANGLE = 1       #: v = value, crc, register
IMU = 2         #: v = quaternion i, j, k, real
DRIVE = 3       #: v = id, iq in 10 mA, theta_hat as a turn in 65536,
                #: the innovation in 0.1 mrad

NAMES = {PHASES: 'phases', ANGLE: 'angle', IMU: 'imu', DRIVE: 'drive'}
BY_NAME = {v: k for k, v in NAMES.items()}

# Named as the firmware names them, in Comms/Inc/cmd.h: the bare OP_STATE
# is gate_drivers.py's, and one definition per name across the package is a rule
# the structure suite enforces.
LOG_OP_STATE = 0
LOG_OP_ARM = 1
LOG_OP_TAKE = 2

MAX_BURST = 15

#: Wire size of one record - u32 at, u8 source, u8 seq, 4x i16 - which
#: is what `take` parses below and not what the struct occupies in the
#: board's RAM. 15 of them plus the count is 211 bytes, inside 253.
RECORD_BYTES = 14


class Capture(Subsystem):

    """Arm a set of sources, then drain what they produced."""

    def _op(self, op, payload=b'', **kwargs):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_LOG, op]) + bytes(payload), **kwargs)

    def state(self):
        """What is armed, how much is waiting, and how much did not make it.

        `dropped` and `thinned` mean opposite things and a view that adds
        them up says nothing. Dropped is a sample the ring had no room for.
        Thinned is one the board declined to take, because that source had
        already used its share of what the link can drain - which is what
        stops the angle loop, at about 24 000 pushes a second, from locking
        the IMU's fifty out of a ring that holds 1024.
        """
        r = Reader(self._op(LOG_OP_STATE))
        mask = r.u8()
        return {
            'sources': [NAMES[i] for i in sorted(NAMES) if mask >> i & 1],
            'mask': mask,
            'count': r.u16(),
            'depth': r.u16(),
            'dropped': r.u32(),
            'thinned': r.u32(),
        }

    def arm(self, sources):
        """Arm a list of source names (or a raw mask) and empty the ring.

        Emptying is not optional on the board's side either: a burst whose
        first records predate the run is worse than an empty one, and no
        field in the record would say so.
        """
        if isinstance(sources, int):
            mask = sources
        else:
            unknown = [s for s in sources if s not in BY_NAME]
            if unknown:
                raise ValueError('no such source: %s - have %s'
                                 % (', '.join(unknown), ', '.join(BY_NAME)))
            mask = 0
            for s in sources:
                mask |= 1 << BY_NAME[s]

        if self._op(LOG_OP_ARM, bytes([mask]))[0] != 1:
            raise RigError('the board refused to arm the capture ring')
        return True

    def stop(self):
        """Disarm every source. The ring is emptied with them."""
        return self.arm(0)

    def take(self, want=MAX_BURST):
        """Up to `want` records, oldest first, freed from the ring as they go."""
        want = max(1, min(int(want), MAX_BURST))
        r = Reader(self._op(LOG_OP_TAKE, bytes([want])))
        out = []
        for _ in range(r.u8()):
            rec = {'at': r.u32()}
            rec['source'] = NAMES.get(r.u8(), '?')
            rec['seq'] = r.u8()
            rec['v'] = tuple(r.i16() for _ in range(4))
            out.append(rec)
        return out

    def drain(self, limit=None):
        """Everything waiting, in order, stopping at `limit` records.

        Returns when the ring reports empty rather than when a burst comes
        back short: a producer can fill a slot between the board counting
        and the reply going out.
        """
        out = []
        while limit is None or len(out) < limit:
            batch = self.take()
            if not batch:
                break
            out.extend(batch)
        return out[:limit] if limit is not None else out
