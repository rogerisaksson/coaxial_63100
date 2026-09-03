"""The node network `thermal/src/thermal.c` runs in firmware, on the host.

Same six nodes and same parameters, here so they can be fitted against
measurements without a reflash. Firmware integrates continuously; this is for
calibration and for views.

**Not FEM.** A mesh and a solver do not fit in a main loop and are not needed
to answer how hot the gate driver is. Six nodes do::

    drivers ---+
    phases  ---+
    mcu     ---+--- board ---- ambient
    regs    ---+
    afe     ---+

A node is a ZONE, not a part. `regulators` is the whole supply corner: the
bucks, the LDOs **and the LED droppers** that sit there. Measured 2026-08-28
that zone ran 8 K over dead board in the passive state, eight times anything
else - but that figure is the zone's, not the LDO's.

Constant sources inside a zone matter less than they sound: the LED droppers
draw the same in all four states, so they cancel in every difference. The
campaign measures differences, which is why it tolerates a zone holding more
than its name.

Measured 2026-08-28 against a thermal camera, dead soldermask as the
reference surface, room 20 C:

===========  ======  ======  ==========  ======  ====
state          dead     mcu  regulators  bridge   afe
===========  ======  ======  ==========  ======  ====
1 passive      30.0   +15.0        +8.0    +1.0  +1.0
2 afe on       31.1   +14.2        +8.1       -  +5.9
3 traffic      31.4   +13.6        +7.6       -  +5.9
4 switching    40.0   +17.3       +20.0   +10.1   0.0
===========  ======  ======  ==========  ======  ====
"""
import math

from . import inverter

#: The room, measured. The board cannot read it itself.
AMBIENT = 20.0

#: The two camera states the NTC compensation is derived from.
MEASURED = {
    'passive': {'ntc': 36.0, 'board': 30.0},
    'switching': {'ntc': 55.6, 'board': 40.0},
}

#: The NTC's constant offset over the board, K. Mounting and the channel's
#: own calibration, not physics: taken in the passive state, where no driver
#: was warming anything.
NTC_OFFSET = MEASURED['passive']['ntc'] - MEASURED['passive']['board']

#: The four states the bench can hold. Here and not in a tool because two
#: tools drive them, and a second copy is the one that goes stale.
#:
#: The order matters: each state adds one power term to the one before, so
#: the DIFFERENCES isolate a subsystem no single state can.
STATES = ('passive', 'afe', 'traffic', 'switch')

STATE_IS = {
    'passive': 'AFE off: the drivers have supply, no PWM',
    'afe':     'AFE on: drivers unpowered, sensors alive, no traffic',
    'traffic': 'AFE on: DAQ at full tilt, data off the board',
    'switch':  'AFE off: three legs at 50 %',
}

#: PER LEG. A leg that is not switching does not get warm, and the lumped
#: pair could not say that - see `thermal_node_t`.
LEGS = ('u', 'v', 'w')
DRIVERS = tuple('driver_' + leg for leg in LEGS)
PHASES = tuple('phase_' + leg for leg in LEGS)

#: The NTC's physical neighbour. It anchors this node, not all three.
NTC_NEIGHBOUR = 'driver_v'

NODES = DRIVERS + PHASES + ('mcu', 'regulators', 'afe')

#: Firmware's `thermal_node_t` order - the sources plus the board node.
#: `0x6E` device 8 answers in that order, so it belongs here and not in a
#: second copy beside the protocol code.
ALL_NODES = NODES + ('board',)


def pretty(node):
    """`driver_u` -> `driver U`. The leg is a name, not a suffix."""
    head, _, leg = node.rpartition('_')
    return '%s %s' % (head, leg.upper()) if leg in LEGS else node

#: K/W from a leg node's surface into the board, and the driver's share of
#: the switching loss. NAMED BECAUSE THE NTC COUPLING IS SOLVED AGAINST
#: THEIR PRODUCT: the campaign's one switching state fixes
#: `ntc_sees_drivers x to_board x watt` and nothing more, so a change to
#: either of these without re-solving the coupling silently stops the model
#: reproducing its own measurement.
LEG_TO_BOARD = 45.6
DRIVER_SWITCH_WATT = 0.60 / 3

#: How far the drivers node sat above the board while switching, K.
#:
#: NOT A MEASUREMENT AND NOT A CONSTANT OF ITS OWN - it is the product
#: above, and it was written here as a bare 9.1 beside the two numbers it
#: is made of, free to drift from them.
DRIVER_RISE_SWITCHING = DRIVER_SWITCH_WATT * LEG_TO_BOARD

#: How much of the drivers' rise the NTC picks up, solved from that one
#: state. It comes out ABOVE 1, and that is not a property a sensor can
#: have: `expected_ntc` then reads the thermistor hotter than the node it
#: is coupled to at every rise - 6.0 K over it at rest and 11.5 K over it
#: at a 100 K rise, in `test_sensorless.py`. The note here used to
#: rationalise it as "closer to the heat than the point the node stands
#: for", and a cap at 1.0 was tried and dropped for costing 5.6 K in the
#: switching state.
#:
#: What is more likely is that the model has NO BOARD GRADIENT. `board` in
#: that state is the camera at one spot, the copper under the thermistor
#: need not be that spot, and a fit with nowhere else to put the
#: difference puts it in the coupling. FINDINGS has the family of
#: (to_board, coupling) pairs that fit the measurement exactly; this is
#: the only one on the curve that puts the sensor above its own source.
NTC_SEES_DRIVERS = ((MEASURED['switching']['ntc']
                     - MEASURED['switching']['board'])
                    - NTC_OFFSET) / DRIVER_RISE_SWITCHING

#: The network. **Only `board_to_ambient` and `board_capacity` have a clean
#: measurement behind them.**
#:
#: `to_board` is a **spreading resistance in the laminate**, K/W from the
#: surface at a source to the board some way off - a few K/W, not a
#: junction-to-board on tens.
CFG = {
    'board_to_ambient': 8.33,     # K/W, passive state against the supply
    'board_capacity': 49.0,       # J/K, from tau ~6.8 min
    'ntc_sees_drivers': NTC_SEES_DRIVERS,
    # Three times the lumped K/W each and a third of the J/K, so the three
    # in parallel are what the camera measured - the split moved where the
    # heat is drawn, not how much there is. One leg alone now rises three
    # times as far and three times as fast, which is the whole point.
    'to_board': dict([(n, LEG_TO_BOARD) for n in DRIVERS + PHASES]
                     + [('mcu', 22.5), ('regulators', 15.0), ('afe', 41.5)]),
    'capacity': dict([(n, 0.35 / 3) for n in DRIVERS]
                     + [(n, 1.20 / 3) for n in PHASES]
                     + [('mcu', 0.90), ('regulators', 0.80), ('afe', 0.30)]),
}

#: Power per node while three legs switch at 50 %. The 1.20 W from difference
#: 4-1 fell roughly half on the supply corner - gate charge comes out of the
#: +15V7 buck - and half on the bridge.
POWER_SWITCHING = dict([(n, DRIVER_SWITCH_WATT) for n in DRIVERS]
                       + [(n, 0.0) for n in PHASES]
                       + [('mcu', 0.666), ('regulators', 1.134), ('afe', 0.0)])


def board_from_ntc(ntc_c, driver_rise_k=0.0):
    """Board temperature from the NTC, hot spot taken out.

    `driver_rise_k` is how far the drivers node sits above the board. Zero
    with nothing switching, which is when the offset alone applies.
    """
    return ntc_c - NTC_OFFSET - NTC_SEES_DRIVERS * driver_rise_k


def expected_ntc(board_c, driver_rise_k=0.0):
    """What the NTC should read. The inverse of `board_from_ntc`."""
    return board_c + NTC_SEES_DRIVERS * driver_rise_k + NTC_OFFSET


def steady(power, cfg=CFG, ambient=AMBIENT):
    """Equilibrium temperature per node for a power split, degrees C."""
    board = ambient + sum(power.values()) * cfg['board_to_ambient']
    out = {'board': board}
    for name in NODES:
        out[name] = board + power.get(name, 0.0) * cfg['to_board'][name]
    return out


def tau_minutes(cfg=CFG):
    """The board's time constant. A run has to be several times it."""
    return cfg['board_capacity'] * cfg['board_to_ambient'] / 60.0


def settled_fraction(minutes, cfg=CFG):
    """How far toward equilibrium a run of that length gets, 0..1."""
    return 1.0 - math.exp(-minutes / tau_minutes(cfg))


#: How much of the conduction path is the FET rather than the shunt.
#: `inverter` holds both - 1.8 mohm of Rds(on) against 3.5 of shunt - and
#: this is the ratio between them, so a caller that still passes one
#: lumped `r_phase` gets it split the way the parts actually divide it.
SWITCH_SHARE = inverter.RDS_ON / (inverter.RDS_ON + inverter.SHUNT)


def phase_power(amps_rms, r_phase, switching=True, cfg_power=None):
    """Power per node at `amps_rms` a phase: the conduction split between
    the FET and the shunt it sits in series with, the drivers' switching
    share where the stage is switching, the housekeeping always.

    `r_phase` is what the current sees - the FET's Rds(on) and the shunt
    together, which `coaxial.inverter` holds. One definition, because a
    burst plan and a continuous rating that disagree about it disagree
    about everything downstream.

    SPLIT, NOT ALL ON THE PHASE NODE. Every watt of it used to land on
    `phase_*`, so the model said the shunt cooked while the FET beside it
    in the same current path stayed cold: measured, fifteen cells of
    seventeen on the phase thermometer against three on the driver's.
    They are two parts and they heat separately. The nodes keep their
    names - `driver_*` is now the FET's conduction as well as its
    switching, and `phase_*` is the shunt.
    """
    out = dict(cfg_power or POWER_SWITCHING)
    if not switching:
        for name in DRIVERS:
            out[name] = 0.0
    heat = amps_rms * amps_rms * r_phase
    for name in DRIVERS:
        out[name] = out.get(name, 0.0) + heat * SWITCH_SHARE
    for name in PHASES:
        out[name] = heat * (1.0 - SWITCH_SHARE)
    return out


def continuous_amps(r_phase, ceiling_c, cfg=CFG, ambient=AMBIENT,
                    rounds=60):
    """Amps rms a phase the board holds for ever, against `ceiling_c`.

    Where the worst node's equilibrium reaches the ceiling: below it a
    state can be held, above it every state is timed and the board's own
    envelope is what ends it. Bisected rather than solved, because the
    worst node is not always the same one - at low current it is the
    regulators and only later a phase.
    """
    def worst(amps):
        at = steady(phase_power(amps, r_phase), cfg=cfg, ambient=ambient)
        return max(at[name] for name in NODES)

    lo, hi = 0.0, 1000.0
    if worst(lo) > ceiling_c:
        return 0.0
    for _ in range(rounds):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if worst(mid) < ceiling_c else (lo, mid)
    return lo


def calibrate(camera, board_c, power=None):
    """Node resistances from a thermal camera's degrees, K/W.

    `camera` is {node: degrees} - **the board surface at each source**, not
    the part. `board_c` is the reference patch at the same moment: soldermask
    some way from anything that warms. Not the NTC, which sits in the drivers'
    hot spot.

    No fitting needed: `to_board = (T_zone - T_reference) / P_zone`. The
    result is a spreading resistance, a few K/W - tens means either the power
    or the reference surface is wrong.
    """
    power = power or POWER_SWITCHING
    out = {}
    for name, celsius in camera.items():
        if celsius is None or not power.get(name):
            continue
        out[name] = (celsius - board_c) / power[name]
    return out
