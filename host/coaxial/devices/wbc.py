"""The whole-body loop behind `0x6E` device 13: ticked on her stand on the board, its cycles."""
from coaxial.comm import protocol
from coaxial.comm.protocol import WbcOp
from coaxial.comm.wire import Reader, pack
from coaxial.devices.subsystem import Device

#: Reply budget a tick, s: 1.4 s wall on Renode at its own 100 MIPS.
TICK_S = 3.0


class Wbc(Device, device=protocol.DEVICE_WBC):

    """The loop in C on the board (`wbc/`), ticked on the model's own stand."""

    def tick(self, ticks=3):
        """{cycles_first, cycles_last, sysclk_hz, us_first, us_last, held, passes, stuck, tau}:
        `ticks` ticks from a cleared memory, the first cold and the rest warm; tau the first
        three drives' torques, N m."""
        r = Reader(self._op(WbcOp.TICK, pack(('u8', ticks)), timeout=TICK_S * ticks))
        first, last, hz = r.u32(), r.u32(), r.u32()
        return {'cycles_first': first, 'cycles_last': last, 'sysclk_hz': hz,
                'us_first': 1e6 * first / hz, 'us_last': 1e6 * last / hz,
                'held': r.u8(), 'passes': r.u8(), 'stuck': r.u8(),
                'tau': [r.i32() / 1000.0 for _ in range(3)]}
