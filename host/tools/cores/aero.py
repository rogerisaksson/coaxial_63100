"""aero/ through ctypes: the frame's core, a compiled payload in, each rotor's thrust out.

    core = aero.Core(aero.frame())                       # machine.quad's frame, laid once
    out = core.step(payload, aero.reading(state, given, lo, hi, dt))

The core the firmware builds, on this host; one library a process.
"""
import ctypes
import functools
import math
import os
import struct

from machine import quad
from tools import REPO
from tools.cores.build import cached

SOURCES = [os.path.join(REPO, 'aero', name)
           for name in ('test/harness.c', 'src/aero.c', 'src/aero_turn.c', 'src/aero_share.c')]
INCLUDES = [os.path.join(REPO, 'aero', 'inc'), os.path.join(REPO, 'aero', 'src')]
#: Built as the firmware builds it (CMakeLists.txt), every symbol kept for ctypes.
BUILT = ('-O2', '-fno-math-errno') + (('-Wl,--export-all-symbols',) if os.name == 'nt' else ())

#: aero.h's sizes.
MODES, ROWS, TASKS, ROTORS, AXES, WRENCH = 6, 5, 2, 4, 3, 4
TARGET, WEIGHT, STIFF, DAMP, SWING, ASK = range(ROWS + 1)

#: aero_payload_t as bytes, native: `machine.aerolisp`'s pack.
PACK = 'I I %df %df %df I f %df I %df' % (MODES, MODES, MODES, MODES, TASKS)

#: The share's weights by row, of a unit thrust: the tilt's torques over the collective over the
#: heading's - at a rotor's end the tilt is kept and the collective gives way, the heading
#: first; the observers' bandwidths on the frame's rates and its speed, rad/s; the rate a
#: height's constraint is come to at, 1/s; the most the air may lean the rotors' axis off its
#: ask, a tangent; the rate a push is laid to the wind at, rad/s.
WEIGH_TILT, WEIGH_LIFT, WEIGH_HEADING = 30.0, 3.0, 1.0
WO_TURN, WO_PULL, RISE, LEAN, WO_WIND = 40.0, 10.0, 5.0, 0.6, 5.0

_f, _u = ctypes.c_float, ctypes.c_uint32


class Payload(ctypes.Structure):

    """aero_payload_t."""

    _fields_ = [('sequence_id', _u), ('blend_ms', _u), ('target', _f * MODES),
                ('stiffness', _f * MODES), ('damping', _f * MODES), ('cpg_enabled', _u),
                ('cpg_hz', _f), ('cpg_amplitude', _f * MODES), ('constraints', _u),
                ('constraint', _f * TASKS)]


class Params(ctypes.Structure):

    """aero_params_t."""

    _fields_ = [('mass', _f), ('gravity', _f), ('inertia', _f * AXES), ('at_x', _f * ROTORS),
                ('at_z', _f * ROTORS), ('spin', _f * ROTORS), ('kappa', _f),
                ('weigh', _f * WRENCH), ('wo_turn', _f), ('wo_pull', _f), ('rise', _f),
                ('lean', _f), ('drag', _f), ('inflow', _f), ('wo_wind', _f)]


class In(ctypes.Structure):

    """aero_in_t."""

    _fields_ = [('r', _f * 9), ('spin', _f * AXES), ('vel', _f * AXES), ('height', _f),
                ('given', _f * ROTORS), ('lo', _f * ROTORS), ('hi', _f * ROTORS), ('dt', _f)]


class Out(ctypes.Structure):

    """aero_out_t."""

    _fields_ = [('thrust', _f * ROTORS), ('asked', _f * WRENCH), ('gave', _f * WRENCH),
                ('bound', _u), ('axis', _f * AXES), ('miss', _f * AXES), ('turned', _f * AXES),
                ('pushed', _f * AXES), ('head', _f * 2), ('wind', _f * AXES)]


@functools.lru_cache(maxsize=None)
def library():
    """The core's C, built and typed once a process; its structs' sizes held to these."""
    lib = ctypes.CDLL(cached(SOURCES, INCLUDES, 'aero', list(BUILT))[0])
    lib.aeh_size.argtypes, lib.aeh_size.restype = [ctypes.c_int], ctypes.c_uint
    for which, kind in enumerate((None, Payload, Params, In, Out)):
        if kind is not None and ctypes.sizeof(kind) != lib.aeh_size(which):
            raise RuntimeError('%s is %d bytes here and %d in aero.h' % (
                kind.__name__, ctypes.sizeof(kind), lib.aeh_size(which)))
    if struct.calcsize(PACK) != ctypes.sizeof(Payload):
        raise RuntimeError('a payload packs to %d bytes, aero_payload_t is %d' % (
            struct.calcsize(PACK), ctypes.sizeof(Payload)))
    lib.aero_init.argtypes = [ctypes.c_void_p, ctypes.POINTER(Params)]
    lib.aero_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(Payload), ctypes.POINTER(In),
                              ctypes.POINTER(Out)]
    lib.aero_share.argtypes = [ctypes.c_void_p, ctypes.POINTER(_f * WRENCH),
                               ctypes.POINTER(ctypes.c_double * WRENCH),
                               ctypes.POINTER(_f * ROTORS), ctypes.POINTER(_f * ROTORS),
                               ctypes.POINTER(ctypes.c_double * ROTORS), ctypes.POINTER(_u)]
    lib.aeh_row.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(_f * (MODES + AXES))]
    return lib


def frame():
    """`machine.quad`'s frame as the core's, as it is sized: its mass and inertia, its rotors'
    places and turns, its propellers' drag a thrust; the share's weights at unit rows; the
    observers' bandwidths and the height's rate by its clock; its drag a kg and its rotors'
    loss to the air along their axis (`quad.inflow`)."""
    arm, clock = quad.ARM_M, quad.scales()[1]
    kappa = quad.K_DRAG / quad.K_THRUST
    return Params(
        mass=quad.MASS_KG, gravity=quad.GRAVITY, inertia=(_f * AXES)(*quad.INERTIA),
        at_x=(_f * ROTORS)(*[x for x, _z in quad.ROTOR_AT]),
        at_z=(_f * ROTORS)(*[z for _x, z in quad.ROTOR_AT]), spin=(_f * ROTORS)(*quad.SPIN),
        kappa=kappa, weigh=(_f * WRENCH)(WEIGH_LIFT, WEIGH_TILT / arm, WEIGH_HEADING / kappa,
                                        WEIGH_TILT / arm),
        wo_turn=WO_TURN / clock, wo_pull=WO_PULL / clock, rise=RISE / clock, lean=LEAN,
        drag=0.5 * quad.RHO * quad.BODY_CDA / quad.MASS_KG,
        inflow=math.tau * math.sqrt(quad.K_THRUST) / (quad.INFLOW_J0 * quad.PITCH_M),
        wo_wind=WO_WIND / clock)


def payload(compiled):
    """A `machine.aerolisp.CompiledAeroPayload`, or its bytes, as the core's."""
    return Payload.from_buffer_copy(compiled if isinstance(compiled, (bytes, bytearray))
                                    else compiled.pack())


def reach(asked, turning, dt, slew, least, top, most=None):
    """(each rotor's thrust as its speed has it, at the least and the most it reaches `dt`
    on), N - a reading's: from the speed its loop is `asked`, rad/s, `slew` rad/s^2 either
    way, up no faster than `most` where its board's current gives less, inside `least` and
    `top`; `turning` its speed as it is."""
    k = quad.K_THRUST
    up = [slew] * len(asked) if most is None else [min(slew, each) for each in most]
    return ([k * w * w for w in turning], [k * max(least, w - slew * dt) ** 2 for w in asked],
            [k * max(least, min(top, w + rise * dt)) ** 2 for w, rise in zip(asked, up)])


def speeds(thrusts):
    """Each rotor's speed for its thrust, rad/s: K w^2, in still air - what the air through
    its disc takes is the observers'."""
    return [math.sqrt(max(0.0, f) / quad.K_THRUST) for f in thrusts]


def reading(state, given, lo, hi, dt):
    """A pass's reading from the frame as `quad.Sky.state` has it: its axes, its rates about
    them, its speed, its height; each rotor's thrust as its speed has it and at the least and
    the most it reaches by the next pass, N."""
    return In(r=(_f * 9)(*[float(x) for row in state['turn'] for x in row]),
              spin=(_f * AXES)(*[float(x) for x in state['spin']]),
              vel=(_f * AXES)(*[float(x) for x in state['vel']]), height=float(state['h']),
              given=(_f * ROTORS)(*given), lo=(_f * ROTORS)(*lo), hi=(_f * ROTORS)(*hi), dt=dt)


class Core:

    """The core's memory for one frame, stepped."""

    def __init__(self, params=None):
        self.lib = library()
        self.params = params or frame()
        self.state = ctypes.create_string_buffer(self.lib.aeh_size(0))
        self.out = Out()
        self.lib.aero_init(self.state, ctypes.byref(self.params))

    def step(self, flown, read):
        """One pass on the payload `flown` and the reading `read`: the answer."""
        self.lib.aero_step(self.state, ctypes.byref(flown), ctypes.byref(read),
                           ctypes.byref(self.out))
        return self.out

    def share(self, want, lo, hi):
        """(the thrusts for the wrench `want` in `lo`..`hi`, the rotors at an end) - the
        active set alone."""
        f, bound = (ctypes.c_double * ROTORS)(), _u()
        self.lib.aero_share(self.state, ctypes.byref(self.params.weigh),
                            ctypes.byref((ctypes.c_double * WRENCH)(*want)),
                            ctypes.byref((_f * ROTORS)(*lo)), ctypes.byref((_f * ROTORS)(*hi)),
                            ctypes.byref(f), ctypes.byref(bound))
        return list(f), bound.value

    def row(self, which):
        """(a row of the modes as blended - ASK: the targets with the oscillator on them -,
        the rate modes' sums about x, y, z)."""
        out = (_f * (MODES + AXES))()
        self.lib.aeh_row(self.state, which, ctypes.byref(out))
        return list(out[:MODES]), list(out[MODES:])
