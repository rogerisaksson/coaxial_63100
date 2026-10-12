"""The loop and the step without an active set behind `0x6E` device 13: ticked on her stand."""
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

    def step(self, ticks=3):
        """{cycles_first, cycles_last, sysclk_hz, us_first, us_last, tau, bears}: `ticks` ticks of
        the step without an active set (wbc_wrench.c) on the same stand; tau the first three
        drives' torques, N m, bears each sole's load, N."""
        return self._stepped(self._op(WbcOp.STEP, pack(('u8', ticks)), timeout=TICK_S * ticks))

    def swung(self, ticks=3, strides=0.0):
        """As `step`, her right sole swung where it is with its knee asked folded and the
        left's moment past its room: the step's four rounds; striding `strides` (0 to 1) what
        the floor cannot give is her inertia's (wbc_cone.c) - the step's longest ticks."""
        return self._stepped(self._op(WbcOp.SWUNG, pack(('u8', ticks), ('u8', int(round(100.0 * strides)))),
                                      timeout=TICK_S * ticks))

    @staticmethod
    def _stepped(reply):
        r = Reader(reply)
        first, last, hz = r.u32(), r.u32(), r.u32()
        return {'cycles_first': first, 'cycles_last': last, 'sysclk_hz': hz,
                'us_first': 1e6 * first / hz, 'us_last': 1e6 * last / hz,
                'tau': [r.i32() / 1000.0 for _ in range(3)],
                'bears': [r.i32() / 1000.0 for _ in range(2)]}
