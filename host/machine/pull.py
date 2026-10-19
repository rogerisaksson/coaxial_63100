"""What the quad's rotors can pull: its action space, computed from motor, propeller and pack.

    w = pull.top(left, amps)                 # the four's steady top on the pack, rad/s
    w = pull.ceiling(volts, amps)            # a rotor's, its link held at those volts
    a = pull.reach(w, volts, amps)           # what its speed gains there at the most, rad/s^2
    p = pull.watts(w, iq, amps)              # what its stage takes, W

A rotor's top is where its clamp's current or its link's volts end: the propeller's torque
K w |w| and the rotor's own b w on the motor's 1.5 p lambda a q amp - on a clamp a root -,
the voltage that takes - R i and w lambda along q, w Lq i across it - inside V_FRAC of the
link's over sqrt 3: a row a volt (LINK) and, the four drawing alike through the pack's ohms,
a row a share of its charge (PACK), laid once and read between two. No branch: a choice is a
number of 0 or 1, a bound a least or a most.
"""
import functools
import math
from typing import NamedTuple

from machine import flightpack, quad


class Rotor(NamedTuple):

    """A motor and what turns on it: ohm, H across its magnets, V s, pole pairs; kg m^2 and
    N m s with its propeller on, and that propeller's drag, N m s^2."""

    r: float
    lq: float
    lam: float
    pairs: float
    j: float
    b: float
    drag: float

    @property
    def kt(self):
        """Its torque a q amp, N m."""
        return 1.5 * self.pairs * self.lam


#: The 63100's can and a 16x14 on it, kg m^2.
CAN_J, PROP_J = 1.2e-4, 3.3e-4
#: The quad's as built: the 63100 as its boards' record has it (the profile
#: outrunner_63100_14p), its can and a 16x14 turning.
ROTOR = Rotor(0.03, 4.1e-5, 0.00207, 14.0, CAN_J + PROP_J, 8e-4, quad.K_DRAG)

#: The clamp a flight is planned on and war emergency power's, A: the space's two shells.
I_MAX, I_WEP = 60.0, 90.0

#: The share of the link's volts over sqrt 3 the law's vector may use (the record's
#: drv_v_frac).
V_FRAC = 0.95

#: The tables' rows: a link's volts from VOLTS[0] to VOLTS[1], a row every VOLTS[2]; the
#: pack's charge from none to full in CHARGES rows, its cells' curve's corners among them.
VOLTS, CHARGES = (30.0, 70.0, 0.5), 41

#: A top is sought under this, rad/s, the span halved this often.
OVER_RAD_S, HALVINGS = 4000.0, 40


def torque(w, rotor=ROTOR):
    """What a rotor takes to turn at `w`, N m: its propeller's drag and its own."""
    return rotor.drag * w * abs(w) + rotor.b * w


def room(w, volts, rotor=ROTOR):
    """The q current a link's `volts` leave a rotor at `w`, A: the most whose vector - R i and
    w lambda along q, w Lq i across it - is inside V_FRAC of them over sqrt 3; none where its
    back-EMF is past that."""
    we, most = rotor.pairs * w, V_FRAC * volts / math.sqrt(3.0)
    a, b = (we * rotor.lq) ** 2 + rotor.r ** 2, rotor.r * we * rotor.lam
    return max(0.0, (math.sqrt(max(0.0, b * b - a * ((we * rotor.lam) ** 2 - most * most))) - b) / a)


def watts(w, iq, amps, rotor=ROTOR):
    """What a rotor's stage takes at `w`, W, `iq` A across its magnets of `amps` in all: its
    copper's and its back-EMF's - given back, braked. The law's own v . i is under it: its
    duties wait a period and its vector meets the rotor that much turned, 1 % at a hover, 21
    at 680 rad/s."""
    return 1.5 * (rotor.r * amps * amps + rotor.pairs * w * rotor.lam * iq)


def held(amps, rotor=ROTOR):
    """A rotor's steady top on a clamp of `amps`, rad/s: where turning takes all of them."""
    return (math.sqrt(rotor.b ** 2 + 4.0 * rotor.drag * rotor.kt * amps) - rotor.b) / (
        2.0 * rotor.drag)


def linked(volts, rotors=0, rotor=ROTOR):
    """A rotor's steady top on its link, rad/s: the fastest it turns on a current the link's
    volts leave it - `volts` held or, `rotors` drawing alike through the pack's ohms, its open
    ones (`flightpack.sagged`)."""
    low, high = 0.0, OVER_RAD_S
    for _ in range(HALVINGS):
        w = 0.5 * (low + high)
        need = torque(w, rotor) / rotor.kt
        bus = flightpack.sagged(volts, rotors * watts(w, need, need, rotor))
        fits = float(need <= room(w, bus, rotor))
        low, high = low + fits * (w - low), high - (1.0 - fits) * (high - w)
    return low


LINK = tuple(linked(VOLTS[0] + k * VOLTS[2])
             for k in range(int(round((VOLTS[1] - VOLTS[0]) / VOLTS[2])) + 1))
PACK = tuple(linked(flightpack.open_volts(k / (CHARGES - 1.0)), 4) for k in range(CHARGES))


def between(rows, x):
    """`rows` read `x` rows along, between two; an end's at it and past it."""
    x = max(0.0, min(len(rows) - 1.0, x))
    k = min(int(x), len(rows) - 2)
    return rows[k] + (rows[k + 1] - rows[k]) * (x - k)


@functools.lru_cache(maxsize=None)
def links(rotor):
    """`LINK` for `rotor`."""
    return tuple(linked(VOLTS[0] + k * VOLTS[2], 0, rotor)
                 for k in range(int(round((VOLTS[1] - VOLTS[0]) / VOLTS[2])) + 1))


def ceiling(volts, amps, rotor=ROTOR):
    """A rotor's steady top on a clamp of `amps`, its link held at `volts`, rad/s."""
    return min(held(amps, rotor), between(links(rotor), (volts - VOLTS[0]) / VOLTS[2]))


def top(left, amps):
    """The four's steady top on a clamp of `amps` each, `left` of the pack's charge in it,
    rad/s."""
    return min(held(amps), between(PACK, left * (CHARGES - 1.0)))


def reach(w, volts, amps, rotor=ROTOR):
    """What a rotor's speed gains at `w` at the most, rad/s^2: on `amps`, or what of them its
    link's `volts` leave it, over what turning takes."""
    return (rotor.kt * min(amps, room(w, volts, rotor)) - torque(w, rotor)) / rotor.j


#: The speed, rad/s, above which the loop's slew reaches WEP's; at idle it is the flight's clamp's.
W_SLEW_RAD_S = 300.0


def slew(amps=I_WEP, w=math.inf, rotor=ROTOR):
    """A rotor's loop's own slew on a clamp of `amps`, rad/s^2: half of them to turn the can
    and its propeller; at `w`, bounded by the estimate's reach on the stand-in from the
    flight's clamp at idle to WEP's at speed."""
    base = 0.5 * rotor.kt * amps / rotor.j
    least = 0.5 * rotor.kt * min(amps, I_MAX) / rotor.j
    span = max(1e-6, W_SLEW_RAD_S - quad.IDLE_RAD_S)
    frac = min(1.0, max(0.0, (w - quad.IDLE_RAD_S) / span))
    return least + frac * (base - least)


def thrust(w, along=0.0):
    """The four's thrust at `w` each, N, the air `along` their discs' axis taking its share
    (`quad.inflow`)."""
    return 4.0 * quad.K_THRUST * w * w * quad.inflow(along, w)
