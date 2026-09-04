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

#: The leg nodes' heat capacity, J/K, LUMPED FOR THREE and divided below.
#:
#: **NOT MEASURED.** `thermal.c` has said so since the campaign - "the
#: parts' own are not measured, they respond in seconds, below what this
#: rig can resolve, and only affect the settling" - and that last clause
#: was true when this model was a steady-state fit and is FALSE NOW. The
#: envelope divides by exactly these numbers: `soak_j` is
#: `capacity x (limit - t)`, `hold_seconds` is that over the net watts,
#: and the throttle's whole reaction window is a multiple of it. Every
#: burst figure in FINDINGS rests on a number nobody took.
#:
#: HOW WRONG IT COULD BE, from Silva 2022 (Appl. Sci. 12, 12555): a
#: lumped element's EFFECTIVE transient capacity is `C* = gamma C` with
#: gamma = 1/3 less a negative term for every contact with a better
#: conductor - heat crosses a distributed body in one direction, so a
#: third of the mass is what the transient sees. If 0.35 was a guess at
#: the physical capacity, the transient one is up to three times smaller
#: and every burst is three times shorter. If it was already a guess at
#: the effective one, it stands. Nothing on record says which, so the
#: honest reading is a BAND rather than a value, and
#: `test_thermal_core.py` measures what the band is worth.
#:
#: WHAT WOULD SETTLE IT, and it is the one soft number a transient can
#: reach: a power step and the NTC's slope. With the coupling at one the
#: thermistor reads the leg lump, so `dT/dt` right after a step is
#: `P / capacity` directly - no camera, and `tools/pulse.py` already
#: makes the step.
LEG_CAPACITY_DRIVERS = 0.35
LEG_CAPACITY_PHASES = 1.20

#: Silva's leading term, for a bench that wants to see the other end of
#: the band. NOT APPLIED - applying it would be correcting an unattributed
#: number by a factor and calling the product measured.
CAPACITY_GAMMA = 1.0 / 3.0

#: How far the drivers node sat above the board while switching, K.
#:
#: NOT A MEASUREMENT AND NOT A CONSTANT OF ITS OWN - it is the product
#: above, and it was written here as a bare 9.1 beside the two numbers it
#: is made of, free to drift from them.
DRIVER_RISE_SWITCHING = DRIVER_SWITCH_WATT * LEG_TO_BOARD

#: How far the thermistor's own element sits toward the leg node, 0 to 1.
#:
#: AN ELEMENT NOW, NOT A COEFFICIENT. Silva 2022 (Appl. Sci. 12, 12555)
#: is the form: every thermal object is a resistance and a heat capacitor
#: in parallel, and objects join into a network. The thermistor is one
#: such object, tied to the leg on one side and the board on the other,
#: so its temperature is a WEIGHTED AVERAGE of the two and cannot leave
#: the interval between them whatever this number is.
#:
#: That is the property the old form could not have. It was
#: `board + c x rise + offset`, with c FITTED at 1.055 and an additive
#: offset on top, so the modelled sensor read hotter than the node
#: heating it at every load - 6.0 K over at rest, 11.5 K at a 100 K rise.
#: Capping c at one left the offset still doing it.
#:
#: NOT MEASURED, AND THE CAMPAIGN CANNOT MEASURE IT. Its one switching
#: state implies 9.6 K of thermistor rise against 9.12 K of leg rise, a
#: fraction of 1.05, which no passive body can have. Something among the
#: three inputs is wrong - the leg's spreading resistance (itself three
#: times a lumped figure the camera saw once), the driver's share of the
#: switching loss, or the camera's board reference, which reads mixed
#: copper and soldermask through an emissivity nobody corrected. The
#: model can no longer absorb that in a coupling, so it comes out as a
#: residual, which is where an inconsistency belongs.
#:
#: HALF, pending the power step. A point sensor soldered to FR4 a
#: centimetre from the pad is somewhere between a tenth and two thirds of
#: the way; this is the middle of that. No value of it can produce a
#: reading above its own source, which is the whole point of the change.
NTC_SEES_DRIVERS = 0.5

#: K/W off the board at the calibration rise, and the board's own heat
#: capacity. NAMED ABOVE THE NETWORK because `NTC_TAU_S` is derived from
#: them and a constant that reads its own dict is a constant defined
#: twice.
BOARD_TO_AMBIENT = 8.33
BOARD_CAPACITY = 49.0

#: How slowly a modelled thermistor follows, seconds.
#:
#: AN ELEMENT BETWEEN TWO NODES LAGS BETWEEN THEIR CONSTANTS, and the
#: geometric mean is what "between" means for time constants - the
#: log-midpoint, not the arithmetic one, because a lag is a ratio and not a
#: difference. The leg node is 5.3 s and the board 408 s, so this is 47 s.
#:
#: IT WAS THE LEG'S OWN, 5.32 s, which made the modelled thermistor exactly
#: as quick as the thing it watches. That is the one speed it cannot have:
#: the SOA acts on silicon in a fifth of a second to two thirds, and a
#: sensor soldered into laminate has to be far slower than that or it is not
#: a sensor in laminate, it is a second copy of the FET. At 47 s a 100 A
#: burst moves the reading about a kelvin and a half in its first second
#: while the leg node moves a hundred and forty - which is the separation
#: the bench asked for in as many words.
#:
#: WHAT SETS IT IS NOT THE THERMISTOR. The part is soldered to the board -
#: a point sensor on FR4 with copper only to its own pads - and its own
#: ceramic is a milligram, which settles in well under a second. What
#: lags is the LAMINATE AROUND IT, and the model has no node for that
#: local patch: only the leg and the bulk board. This is the pair it sits
#: between, and a bench day with a power step and the NTC's own slope
#: would replace it with a measurement.
NTC_TAU_S = math.sqrt((LEG_CAPACITY_DRIVERS / 3) * LEG_TO_BOARD
                      * BOARD_CAPACITY * BOARD_TO_AMBIENT)

#: What the campaign's switching state misses by with the element in
#: place of the old coupling. Kept as a number rather than absorbed into a slope: it is the
#: disagreement between a thermistor and a camera, and it belongs where a
#: bench can see how big it is.
NTC_CAMPAIGN_RESIDUAL_K = (MEASURED['switching']['ntc']
                           - (MEASURED['switching']['board']
                              + NTC_SEES_DRIVERS * DRIVER_RISE_SWITCHING))

#: The network. **Only `board_to_ambient` and `board_capacity` have a clean
#: measurement behind them.**
#:
#: `to_board` is a **spreading resistance in the laminate**, K/W from the
#: surface at a source to the board some way off - a few K/W, not a
#: junction-to-board on tens.
#: The rise `board_to_ambient` was measured at, K: the passive state's
#: 1.2 W over a 10 K rise.
BOARD_CAL_RISE_K = 10.0

#: How much of the board's loss at that rise is radiation.
#:
#: NOT MEASURED HERE. It is the 30 to 40 % a compendium of PCBA thermal
#: work gives for passive cooling - "stralning star for 30-40 % av den
#: totala varmeavledningen vid passiv kylning och kan inte forsummas"
#: (docs/papers) - and the split matters because the two mechanisms have
#: DIFFERENT SHAPES against the rise, so only their proportion at the
#: calibration point lets them be scaled apart.
BOARD_RAD_SHARE = 0.35

#: The room the campaign was taken in, kelvin. Radiation is a fourth
#: power, so it needs an absolute temperature and not a difference.
ROOM_K = 293.15


def board_to_ambient_at(rise_k, cfg=None):
    """K/W off the board at `rise_k` over ambient.

    A BOARD LOSES HEAT TO AIR TWO WAYS AND NEITHER IS LINEAR. Free
    convection carries `h = Nu k / L` with `Nu` a power of the Rayleigh
    number, and Ra is linear in the rise, so h goes as about the fourth
    root of it (Ziegenfelder 2022, USU: `q = h A dT` with
    `Gr = (g/nu^2) beta dT P^3`). Radiation carries
    `h_rad = eps sigma (T^2 + T0^2)(T + T0)` (Silva 2022, Eq. 5), which
    grows faster still. A single K/W is both of them frozen at one rise.

    THAT RISE WAS 10 K, and the board was then asked about loads putting
    sixty kelvin on it. Held flat the model over-predicted the copper by
    ten to fifteen kelvin at the powers a burst makes.

    Everything else - the area, the emissivity, the fluid properties, the
    characteristic length - stays inside the calibration value, so this
    reproduces the measurement exactly at its own point and only the
    shape away from it comes from the correlations.
    """
    cfg = cfg or CFG
    flat = cfg['board_to_ambient']
    cal = cfg.get('board_cal_rise_k', BOARD_CAL_RISE_K)
    if cal <= 0.0 or rise_k <= cal:
        return flat
    conv = (rise_k / cal) ** 0.25
    now, was = ROOM_K + rise_k, ROOM_K + cal
    rad = (((now * now + ROOM_K * ROOM_K) * (now + ROOM_K))
           / ((was * was + ROOM_K * ROOM_K) * (was + ROOM_K)))
    share = min(1.0, max(0.0, cfg.get('board_rad_share', BOARD_RAD_SHARE)))
    # In parallel, so their CONDUCTANCES add.
    better = (1.0 - share) * conv + share * rad
    return flat / better if better > 0.0 else flat


CFG = {
    'board_to_ambient': BOARD_TO_AMBIENT,     # K/W at BOARD_CAL_RISE_K, not a constant
    'board_cal_rise_k': BOARD_CAL_RISE_K,
    'board_rad_share': BOARD_RAD_SHARE,
    'board_capacity': BOARD_CAPACITY,   # J/K, from tau ~6.8 min
    'ntc_sees_drivers': NTC_SEES_DRIVERS,
    # Three times the lumped K/W each and a third of the J/K, so the three
    # in parallel are what the camera measured - the split moved where the
    # heat is drawn, not how much there is. One leg alone now rises three
    # times as far and three times as fast, which is the whole point.
    'to_board': dict([(n, LEG_TO_BOARD) for n in DRIVERS + PHASES]
                     + [('mcu', 22.5), ('regulators', 15.0), ('afe', 41.5)]),
    'capacity': dict([(n, LEG_CAPACITY_DRIVERS / 3) for n in DRIVERS]
                     + [(n, LEG_CAPACITY_PHASES / 3) for n in PHASES]
                     + [('mcu', 0.90), ('regulators', 0.80), ('afe', 0.30)]),
}

#: Power per node while three legs switch at 50 %. The 1.20 W from difference
#: 4-1 fell roughly half on the supply corner - gate charge comes out of the
#: +15V7 buck - and half on the bridge.
POWER_SWITCHING = dict([(n, DRIVER_SWITCH_WATT) for n in DRIVERS]
                       + [(n, 0.0) for n in PHASES]
                       + [('mcu', 0.666), ('regulators', 1.134), ('afe', 0.0)])


def board_from_ntc(ntc_c, driver_rise_k=0.0):
    """Board temperature from the NTC, the leg's share taken out.

    `driver_rise_k` is how far the leg node sits above the board. NO
    OFFSET SUBTRACTED: the 6.0 K the campaign found is a disagreement
    between a thermistor and a CAMERA, and it is the camera that reads
    mixed copper and soldermask through an uncorrected emissivity. The
    board's own sensor is the thermistor.
    """
    return ntc_c - NTC_SEES_DRIVERS * driver_rise_k


def expected_ntc(board_c, driver_rise_k=0.0):
    """Where the thermistor's element is heading: the weighted average of
    the two nodes it is tied to, and so always between them.

    The inverse of `board_from_ntc`, and the steady state the lagged
    reading relaxes toward at `NTC_TAU_S`.
    """
    return board_c + min(1.0, max(0.0, NTC_SEES_DRIVERS)) * driver_rise_k


def steady(power, cfg=CFG, ambient=AMBIENT):
    """Equilibrium temperature per node for a power split, degrees C.

    THE BOARD'S RISE IS IMPLICIT NOW: the path off it depends on the rise
    it is carrying, so this iterates rather than multiplies. A handful of
    passes is enough - the resistance moves as the fourth root, so the
    fixed point is a gentle one.
    """
    total = sum(power.values())
    rise = total * cfg['board_to_ambient']
    for _ in range(24):
        rise = total * board_to_ambient_at(rise, cfg)
    board = ambient + rise
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
