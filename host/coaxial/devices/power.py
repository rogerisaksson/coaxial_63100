"""Who is holding a rail, behind `0x6E` device 9."""
from coaxial.comm import protocol
from coaxial.comm.protocol import PowerOp
from coaxial.comm.wire import Reader, label
from coaxial.devices.subsystem import Device
from machine.roles import Output

#: Rails, in the order the board reports them.
RAILS = ('afe',)

#: Bit positions in the users mask, matching `board_user_t`.
USERS = ('host', 'thermal', 'imu', 'angle', 'daq')

#: The mask is one byte.
MASK_BITS = 8


def named(mask):
    """The users in a mask, as names. Unknown bits keep their number."""
    return [label(USERS, bit, 'bit') for bit in range(MASK_BITS)
            if mask >> bit & 1]


class Power(Device, Output, device=protocol.DEVICE_POWER):

    """The rail reference counts, and a way out of a leaked hold."""

    def state(self):
        """{rail: {...}} for every rail the board switches."""
        r = Reader(self._op(PowerOp.STATE))
        return {label(RAILS, i, 'rail'): self._rail(r) for i in range(r.u8())}

    @staticmethod
    def _rail(r):
        on = bool(r.u8())
        users = r.u8()
        return {
            'on': on,
            'users': named(users),
            'mask': users,
            'count': r.u8(),
            'blocked': bool(r.u8()),
            'leased': named(r.u8()),
        }

    def off(self):
        """Drop every hold on every rail."""
        return self._ack(PowerOp.RELEASE_ALL)
