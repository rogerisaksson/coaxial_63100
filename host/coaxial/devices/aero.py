"""The frame's core behind `0x6E` device 14: a frame laid, a pass stepped on the board."""
from coaxial.comm import protocol
from coaxial.comm.protocol import AeroOp
from coaxial.comm.wire import MICRO, MILLI, NANO, Reader, pack
from coaxial.devices.subsystem import Device

#: A frame's params in the wire's order, aero_params_t's: (name, how many, the unit's scale -
#: none a signed byte, the way a rotor turns).
PARAMS = (('mass', 1, MICRO), ('gravity', 1, MICRO), ('inertia', 3, NANO), ('at_x', 4, MICRO),
          ('at_z', 4, MICRO), ('spin', 4, None), ('kappa', 1, MICRO), ('weigh', 4, MILLI),
          ('wo_turn', 1, MILLI), ('wo_pull', 1, MILLI), ('rise', 1, MILLI), ('lean', 1, MICRO),
          ('drag', 1, MICRO), ('inflow', 1, MICRO), ('wo_wind', 1, MILLI))

#: A reading's fields before its pass's seconds, aero_in_t's: (name, how many), millionths.
READING = (('r', 9), ('spin', 3), ('vel', 3), ('height', 1), ('given', 4), ('lo', 4), ('hi', 4))

#: Reply budget a pass, s: Renode at its own pace.
STEP_S = 2.0


def _wire(values, names):
    """([(width, the wire's integer)], {name: its values as the board has them - that
    integer by its unit, in doubles as the board's -}) of `values` by `names` - (name, how
    many, scale)."""
    fields, had = [], {}
    for name, count, scale in names:
        given = values[name]
        given = list(given) if count > 1 else [given]
        whole = [int(round(float(v) * (scale or 1))) for v in given]
        fields += [('i32' if scale else 'i8', n) for n in whole]
        had[name] = [n * (1.0 / (scale or 1)) for n in whole]
    return fields, had


class Aero(Device, device=protocol.DEVICE_AERO):

    """The frame's core in C on the board (`aero/`): its frame laid, a pass a request."""

    def lay(self, params):
        """A frame laid, the core's memory as before a first pass: `params` by aero_params_t's
        names, SI. The params as the board has them, each its wire's integer of its unit."""
        fields, had = _wire(params, PARAMS)
        self._ack(AeroOp.LAY, pack(*fields))
        return had

    def step(self, payload, reading):
        """One pass on the board: `payload` the compiled intent's bytes
        (`machine.aircraft.aerolisp`'s pack), `reading` by aero_in_t's names, SI, its `dt` s. {cycles,
        sysclk_hz, us: what the pass took; thrust, N a rotor; bound: the rotors at an end;
        asked, gave: the wrench; axis, miss; turned, pushed: what the observers hold; head;
        wind; read: the reading as the board had it}."""
        fields, had = _wire(reading, [(name, count, MICRO) for name, count in READING])
        micros = int(round(float(reading['dt']) * MICRO))
        reply = self._op(AeroOp.STEP, pack(*fields, ('u32', micros)) + bytes(payload),
                         timeout=STEP_S)
        self.took(reply)
        r = Reader(reply)
        r.u8()
        cycles, hz = r.u32(), r.u32()
        return {'cycles': cycles, 'sysclk_hz': hz, 'us': 1e6 * cycles / hz,
                'thrust': [r.micro() for _ in range(4)], 'bound': r.u32(),
                'asked': [r.micro() for _ in range(4)], 'gave': [r.micro() for _ in range(4)],
                'axis': [r.micro() for _ in range(3)], 'miss': [r.micro() for _ in range(3)],
                'turned': [r.micro() for _ in range(3)], 'pushed': [r.micro() for _ in range(3)],
                'head': [r.micro() for _ in range(2)], 'wind': [r.micro() for _ in range(3)],
                'read': dict(had, dt=micros * (1.0 / MICRO))}
