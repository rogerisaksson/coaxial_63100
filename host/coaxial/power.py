"""Who is holding a rail, behind `0x6E` device 9.

A rail is switched by whoever needs it and switched back when the last user
lets go. The bitmask is what makes a leak diagnosable: a count that will not
reach zero says only that something is holding on, and the mask says which
subsystem.

**`on` is the pin, read back - not what the count implies.** They should
agree, and the case worth catching is the one where they do not.

Every hold but the host's is a LEASE and expires on its own. Measured
2026-08-28: the thermal observer took AFE_ON, the host then talked hard enough that
`link_busy()` starved the poll holding the release, and the rail stayed high
indefinitely. The host's own hold does not expire - it was asked for by name
over the wire, and only the wire takes it back.
"""
from . import protocol
from .subsystem import Subsystem
from .wire import Reader

POWER_OP_STATE = 0
POWER_OP_RELEASE_ALL = 1

#: Rails, in the order the board reports them.
RAILS = ('afe',)

#: Bit positions in the users mask, matching `board_user_t`.
USERS = ('host', 'thermal', 'imu', 'angle', 'daq')


def named(mask):
    """The users in a mask, as names. Unknown bits keep their number."""
    got = []
    for bit in range(8):
        if mask & (1 << bit):
            got.append(USERS[bit] if bit < len(USERS) else 'bit%d' % bit)
    return got


class Power(Subsystem):

    """The rail reference counts, and a way out of a leaked hold."""

    def _op(self, op, payload=b'', **kwargs):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_POWER, op]) + payload, **kwargs)

    def state(self):
        """{rail: {...}} for every rail the board switches.

        `blocked` is whether an acquire would be refused right now. For the
        AFE rail that means the gate stage is armed: AFE_ON high takes the
        drivers' supply away, and six inputs switching into unpowered drivers
        is not a measurement worth having.
        """
        r = Reader(self._op(POWER_OP_STATE))
        got = {}
        for i in range(r.u8()):
            name = RAILS[i] if i < len(RAILS) else 'rail%d' % i
            on = bool(r.u8())
            users = r.u8()
            got[name] = {
                'on': on,
                'users': named(users),
                'mask': users,
                'count': r.u8(),
                'blocked': bool(r.u8()),
                'leased': named(r.u8()),
            }
        return got

    def release_all(self):
        """Drop every hold on every rail.

        Blunt: anything mid-measurement loses its supply. It exists because a
        leaked hold otherwise needs a power cycle. Safe while the gate stage
        is armed - it switches AFE_ON off, which gives the drivers their
        supply rather than taking it away.
        """
        return self._ack(POWER_OP_RELEASE_ALL)
