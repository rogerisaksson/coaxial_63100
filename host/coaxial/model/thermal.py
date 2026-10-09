"""The node network `thermal/src/thermal.c` runs in firmware, on the host."""
import math

from coaxial.devices.scaling import KELVIN_AT_ZERO_C
from coaxial.model import inverter

SECONDS_PER_MINUTE = 60.0
#: Of the winding's K/W, the share that is the edge into the iron; the rest
#: is the iron's own air path - board_thermal.c's WINDING_INTO_IRON.
WINDING_INTO_IRON = 0.25

#: The room, assumed: 25 C on the bench's word, 2026-09-05.
#: The board cannot read it itself; the stand-in starts every node here.
AMBIENT = 25.0

#: Thermal seconds per wall second in every world but the bench's: the stand-in's clock, the
#: emulated plant's heat and the emulated board's observer (thermal op 13). The ~7 min board
#: constant shows a load step in half a minute.
HASTE = 10.0

#: The two camera states the NTC compensation is derived from, the thermistor's
#: taken down the uncalibrated converter's 1000 codes: 1.9 and 2.6 K (2026-10-05).
MEASURED = {
    'passive': {'ntc': 34.1, 'board': 30.0},
    'switching': {'ntc': 53.0, 'board': 40.0},
}

#: The NTC's constant offset over the board, K. Mounting and the channel's
#: own calibration, not physics: taken in the passive state, where no driver
#: was warming anything.
NTC_OFFSET = MEASURED['passive']['ntc'] - MEASURED['passive']['board']

#: The bench's four states, each adding one power term to the last, so their
#: differences isolate a subsystem.
STATES = ('passive', 'afe', 'traffic', 'switch')

STATE_IS = {
    'passive': 'AFE off: the drivers have supply, no PWM',
    'afe':     'AFE on: drivers unpowered, sensors alive, no traffic',
    'traffic': 'AFE on: DAQ at full tilt, data off the board',
    'switch':  'AFE off: three legs at 50 %',
}

#: Per leg: an idle leg stays cool (`thermal_node_t`).
LEGS = ('u', 'v', 'w')
DRIVERS = tuple('driver_' + leg for leg in LEGS)
PHASES = tuple('phase_' + leg for leg in LEGS)

#: The thermistor sits in the centre patch a driver's width from the V leg's
#: patch, which it corrects.
NTC_PATCH = 'patch_v'
NTC_NEIGHBOUR = NTC_PATCH

#: The star's sources: what the camera campaign fitted, and what the
#: tools that fit it still loop over.
NODES = DRIVERS + PHASES + ('mcu', 'regulators', 'afe')

#: The laminate as seven patches - the centre keeps the name `board`, so a
#: host on an older codec reads the first ten as it did - the hot swap, and
#: the motor behind the board.
PATCHES = ('patch_u', 'patch_v', 'patch_w', 'patch_left', 'patch_bottom',
           'patch_right')
LAMINATE = ('board',) + PATCHES
MOTOR = ('winding', 'stator', 'rotor')

#: Firmware's `thermal_node_t` order. `0x6E` device 8 answers in that
#: order, so it belongs here and not in a second copy beside the protocol
#: code. Append only.
ALL_NODES = NODES + ('board', 'hotswap') + PATCHES + MOTOR


def pretty(node):
    """`driver_u` -> `driver U`. The leg is a name, not a suffix."""
    head, _, leg = node.rpartition('_')
    return '%s %s' % (head, leg.upper()) if leg in LEGS else node

#: K/W from a leg node's surface into the board: 28, the geometric mean of
#: two readings that disagree - an estimate, not a measurement.
#:
#:    camera's bridge zone, 15.2 K/W lumped, x3 legs              45.6 K/W
#:    datasheet 25.9 K/W junction-air (2s2p) - Rth JC 0.69 - 8.33  16.9 K/W
#:    geometric mean                                               27.7 K/W
#:
#: The camera read an uncorrected mixed copper/soldermask emissivity; the
#: datasheet's coupon is not this board. Continuous rating, three legs, 20 C
#: room, 125 C nodes, 105 C board: 15-18 A rms at 45.6, 20-22 A at 28, ~25 A
#: at 16.9 - the shunt binds in every case (3.5 mOhm against the FET's 1.8).
#: At 100 A a FET's 9 W puts its junction 6.2 K over its node: 131 C against
#: the sheet's 175. A camera run under load, emissivity corrected, settles it.
LEG_TO_BOARD = 28.0
DRIVER_SWITCH_WATT = 2.40 / 3

#: The leg nodes' heat capacity, J/K, lumped for three. Not measured, and the
#: envelope divides by it: `soak_j`, `hold_seconds` and the throttle's window
#: scale with it. Silva 2022 (Appl. Sci. 12, 12555): a lumped element's
#: transient capacity is ~1/3 of its physical one, so bursts may be up to 3x
#: shorter; `test_thermal_core.py` measures the band. A power step and the
#: NTC's slope settle it (`tools/bench/pulse.py` makes the step).
LEG_CAPACITY_DRIVERS = 0.35
LEG_CAPACITY_PHASES = 1.20

#: Silva's gamma, for the other end of the band. Not applied: a factor on an
#: unattributed number is not a measurement.
CAPACITY_GAMMA = 1.0 / 3.0

#: The drivers node over the board while switching, K: the product of the two
#: above, not a constant of its own.
DRIVER_RISE_SWITCHING = DRIVER_SWITCH_WATT * LEG_TO_BOARD

#: Where the thermistor sits between the board and the V leg, 0 to 1: an
#: element of the network (Silva 2022), never outside the interval. Two dry
#: runs' cooldowns beside both dies fit 0.56 (2026-10-05); the pick-and-place
#: has the V gate driver 8.2 mm off, f = 0.50, the V FETs 15-18 mm, 0.28-0.33.
NTC_SEES_DRIVERS = 0.56

#: K/W off the board at the calibration rise, and its heat capacity, J/K.
BOARD_TO_AMBIENT = 8.33
BOARD_CAPACITY = 49.0

#: How slowly the modelled thermistor follows, s: it kept the laminate's pace
#: through both cooldowns (2026-10-05).
NTC_TAU_S = 0.5

#: What the campaign's switching state misses by with the element: the
#: thermistor-against-camera disagreement, kept visible, not absorbed.
NTC_CAMPAIGN_RESIDUAL_K = (MEASURED['switching']['ntc']
                           - (MEASURED['switching']['board']
                              + NTC_SEES_DRIVERS * DRIVER_RISE_SWITCHING))

#: The rise `board_to_ambient` was measured at, K: the passive state's 1.2 W.
#: It and `board_capacity` are the network's only clean measurements;
#: `to_board` is a spreading resistance in the laminate, a few K/W.
BOARD_CAL_RISE_K = 10.0

#: Radiation's share of the board's loss at that rise: 30-40 % for passive
#: cooling (docs/papers), not measured here. The split matters because the
#: two scale differently with the rise.
BOARD_RAD_SHARE = 0.35

#: Free convection's exponent on the rise (Nu = C Ra^n): 1/4 while laminar,
#: Ra < 1e7 horizontal, 1e9 vertical. This 92 x 93 mm board runs Ra 1.3e4
#: to 4e6 over 10-85 K rises, decades short either way; `test_sensorless.py`
#: recomputes it.
CONVECTION_EXPONENT = 0.25

#: The room the campaign was taken in, kelvin. Radiation is a fourth
#: power, so it needs an absolute temperature and not a difference.
ROOM_K = 293.15


def board_to_ambient_at(rise_k, cfg=None):
    """K/W off the board at `rise_k` over ambient."""
    cfg = cfg or CFG
    flat = cfg['board_to_ambient']
    cal = cfg.get('board_cal_rise_k', BOARD_CAL_RISE_K)
    if cal <= 0.0 or rise_k <= cal:
        return flat
    conv = (rise_k / cal) ** CONVECTION_EXPONENT
    now, was = ROOM_K + rise_k, ROOM_K + cal
    rad = (((now * now + ROOM_K * ROOM_K) * (now + ROOM_K))
           / ((was * was + ROOM_K * ROOM_K) * (was + ROOM_K)))
    share = min(1.0, max(0.0, cfg.get('board_rad_share', BOARD_RAD_SHARE)))
    # In parallel, so their conductances add.
    better = (1.0 - share) * conv + share * rad
    return flat / better if better > 0.0 else flat


#: The laminate as seven patches: each one's share of the board's face,
#: off a quarter-millimetre raster of the 100 mm disc less its 10 mm bore
#: over the partition `patch_of` draws - the thermal picture's own bands.
#: 7776 mm^2 in all. The same numbers `thermal_defaults` carries.
AREA_SHARE = {'board': 0.199, 'patch_u': 0.109, 'patch_v': 0.134,
              'patch_w': 0.109, 'patch_left': 0.126, 'patch_bottom': 0.197,
              'patch_right': 0.126}


def patch_of(x_mm, y_mm):
    """Which laminate patch a point of the board is in, millimetres from the
    centre, +y up: the band under the switches and shunts above y = 12
    cut into the three legs at x = +-14, the front end's band below y =
    -25, and the middle in three at x = +-22.
    """
    if y_mm >= 12.0:
        return 'patch_u' if x_mm < -14.0 else (
            'patch_w' if x_mm > 14.0 else 'patch_v')
    if y_mm < -25.0:
        return 'patch_bottom'
    if x_mm < -22.0:
        return 'patch_left'
    if x_mm > 22.0:
        return 'patch_right'
    return 'board'


#: The sheet conductance of the laminate, W/K per unit of shared boundary
#: over centre distance: 0.020 makes the V leg's patch's three neighbours
#: in parallel the 15.2 K/W the camera measured lumped, and it is what two
#: ounces of copper on two layers at forty percent coverage compute to.
K_SHEET = 0.020

#: The edges, in `thermal.c`'s table order: `(a, b, K/W)`. The sources into
#: their patches first - a leg's switches at 12, its shunts at 8, the rest
#: the camera's zone figures, the hot swap an estimate at the driver's -
#: then the laminate's own graph as `1 / (K_SHEET * L / d)` over the
#: boundaries the raster measured, then the motor, then the mount (open
#: on a bench).
EDGES = (
    ('driver_u', 'patch_u', 12.0), ('driver_v', 'patch_v', 12.0),
    ('driver_w', 'patch_w', 12.0),
    ('phase_u', 'patch_u', 8.0), ('phase_v', 'patch_v', 8.0),
    ('phase_w', 'patch_w', 8.0),
    ('mcu', 'board', 22.5), ('regulators', 'patch_left', 15.0),
    ('afe', 'patch_bottom', 41.5), ('hotswap', 'patch_right', 12.0),
    ('patch_u', 'patch_v', 39.0), ('patch_v', 'patch_w', 39.0),
    ('patch_u', 'patch_left', 62.0), ('patch_w', 'patch_right', 62.0),
    ('patch_left', 'board', 48.0), ('board', 'patch_right', 48.0),
    ('patch_left', 'patch_bottom', 109.0), ('board', 'patch_bottom', 32.0),
    ('patch_right', 'patch_bottom', 109.0), ('patch_v', 'board', 67.0),
    ('patch_u', 'board', 263.0), ('patch_w', 'board', 263.0),
    ('winding', 'stator', 0.55), ('stator', 'rotor', 2.0),
    ('stator', 'patch_u', 0.0), ('stator', 'patch_v', 0.0),
    ('stator', 'patch_w', 0.0), ('stator', 'patch_left', 0.0),
    ('stator', 'patch_bottom', 0.0), ('stator', 'patch_right', 0.0),
)
EDGE_WINDING_STATOR, EDGE_STATOR_ROTOR, EDGE_MOUNT_FIRST = 22, 23, 24


def sink_edge(node):
    """The edge a node sheds through first, or None for a node whose first path
    is the air - the same rule as `thermal_sink_edge`.
    """
    for index, (a, _b, _r) in enumerate(EDGES):
        if a == node and index < 10 or (node == 'winding' and index == 22) \
                or (node == 'stator' and index == 23):
            return index
    return None


#: The motor's three nodes: the profile's placeholder pair split as an
#: outrunner is - a quarter of the 2.2 K/W from the copper into the iron,
#: the rest the iron's own air path; the iron twice the copper's 180 J/K,
#: the bell about the copper's, 4 K/W to still air and a unit of forced
#: convection per sqrt(krpm); the air gap 2 K/W. Estimates with a name
#: each - `thermal.c` has the derivations.
MOTOR_CAPACITY = {'winding': 180.0, 'stator': 360.0, 'rotor': 180.0}
MOTOR_TO_AMBIENT = {'stator': 1.65, 'rotor': 4.0}
FORCED = dict([(n, 0.3) for n in LAMINATE]
              + [('stator', 0.5), ('rotor', 1.0)])
#: Junction over node per watt: R_th,JC for a leg's FETs, 0.5 K at 0.13 W for
#: the A1335's die; the MCU's reads its package (bench, 2026-10-05).
RTH_DIE = dict([(n, 0.69) for n in DRIVERS]
               + [('mcu', 0.0), ('afe', 3.8)])

CFG = {
    'board_to_ambient': BOARD_TO_AMBIENT,     # K/W at BOARD_CAL_RISE_K, not a constant
    'board_cal_rise_k': BOARD_CAL_RISE_K,
    'board_rad_share': BOARD_RAD_SHARE,
    'board_capacity': BOARD_CAPACITY,   # J/K, from tau ~6.8 min
    'ntc_sees_drivers': NTC_SEES_DRIVERS,
    # The star's view of the sources, for the tools that fit the camera:
    # each source's edge into the laminate under it.
    'to_board': dict((a, r) for a, _b, r in EDGES[:10]),
    'capacity': dict([(n, LEG_CAPACITY_DRIVERS / 3) for n in DRIVERS]
                     + [(n, LEG_CAPACITY_PHASES / 3) for n in PHASES]
                     + [('mcu', 0.90), ('regulators', 0.80), ('afe', 0.30),
                        ('hotswap', 0.50)]
                     + [(n, BOARD_CAPACITY * AREA_SHARE[n]) for n in LAMINATE]
                     + list(MOTOR_CAPACITY.items())),
    # The graph: the edges' K/W in table order, each node's air path (a patch's
    # the bulk's over its share), its share of the face, its junction-per-watt
    # and its forced-convection gain.
    'edges': [r for _a, _b, r in EDGES],
    'to_ambient': dict([(n, BOARD_TO_AMBIENT / AREA_SHARE[n])
                        for n in LAMINATE] + list(MOTOR_TO_AMBIENT.items())),
    'area_share': dict(AREA_SHARE),
    'rth_die': dict(RTH_DIE),
    'forced': dict(FORCED),
    'rad_board_stator': 0.0,
}

#: Power per node while three legs switch at 50 %, dry at 24.6 V: the dump
#: and the gate's in the legs, the gate's buck's on the supply corner.
POWER_SWITCHING = dict([(n, DRIVER_SWITCH_WATT) for n in DRIVERS]
                       + [(n, 0.0) for n in PHASES]
                       + [('mcu', 0.666), ('regulators', 0.746), ('afe', 0.0)])


def board_from_ntc(ntc_c, driver_rise_k=0.0):
    """Board temperature from the NTC, the leg's share taken out."""
    return ntc_c - NTC_SEES_DRIVERS * driver_rise_k


def expected_ntc(board_c, driver_rise_k=0.0):
    """Where the thermistor's element is heading: the weighted average of the
    two nodes it is tied to, and so always between them.
    """
    return board_c + min(1.0, max(0.0, NTC_SEES_DRIVERS)) * driver_rise_k


def to_ambient_at(node, rise_k, speed_rpm=0.0, cfg=None):
    """K/W off one node's air path at a rise and a rotor speed - the same
    rule as `thermal_to_ambient_at`: a patch carries the bulk's nonlinear
    law scaled to its share, and any node the rotor's air reaches
    improves by its `forced` gain per sqrt(krpm).
    """
    cfg = cfg or CFG
    r = cfg['to_ambient'].get(node, 0.0)
    if r <= 0.0:
        return None
    if cfg['area_share'].get(node, 0.0) > 0.0:
        r = r * board_to_ambient_at(rise_k, cfg) / cfg['board_to_ambient']
    forced = cfg['forced'].get(node, 0.0)
    if forced > 0.0 and speed_rpm > 0.0:
        r /= 1.0 + forced * math.sqrt(speed_rpm / 1000.0)
    return r


def net_flows(temps, power, cfg=None, ambient=AMBIENT, speed_rpm=0.0):
    """Net watts into every node at these temperatures: what it makes, plus
    what flows in over the edges, less what it sheds to the air -
    `thermal.c`'s `net_flows`, so the stand-in integrates the same graph
    the board does.
    """
    cfg = cfg or CFG
    net = dict((n, power.get(n, 0.0)) for n in ALL_NODES)
    for (a, b, _r), r in zip(EDGES, cfg['edges']):
        if r > 0.0:
            flow = (temps[a] - temps[b]) / r
            net[a] -= flow
            net[b] += flow
    rad = cfg.get('rad_board_stator', 0.0)
    if rad > 0.0:
        def bracket(a_c, b_c):
            a, b = a_c + KELVIN_AT_ZERO_C, b_c + KELVIN_AT_ZERO_C
            return (a * a + b * b) * (a + b)
        room = bracket(26.85, 26.85)
        for n in LAMINATE:
            g = rad * cfg['area_share'][n] * bracket(temps[n], temps['stator']) / room
            flow = g * (temps[n] - temps['stator'])
            net[n] -= flow
            net['stator'] += flow
    for n in ALL_NODES:
        rise = temps[n] - ambient
        away = to_ambient_at(n, rise, speed_rpm, cfg)
        if away:
            net[n] -= rise / away
    return net


def steady(power, cfg=CFG, ambient=AMBIENT, speed_rpm=0.0, rounds=4000):
    """Equilibrium temperature per node for a power split, degrees C."""
    temps = dict((n, ambient) for n in ALL_NODES)
    joins = dict((n, []) for n in ALL_NODES)
    for (a, b, _r), r in zip(EDGES, cfg['edges']):
        if r > 0.0:
            joins[a].append((b, r))
            joins[b].append((a, r))
    for _ in range(rounds):
        moved = 0.0
        for n in ALL_NODES:
            g_total, drive = 0.0, power.get(n, 0.0)
            for other, r in joins[n]:
                g_total += 1.0 / r
                drive += temps[other] / r
            away = to_ambient_at(n, temps[n] - ambient, speed_rpm, cfg)
            if away:
                g_total += 1.0 / away
                drive += ambient / away
            if g_total > 0.0:
                fresh = drive / g_total
                moved = max(moved, abs(fresh - temps[n]))
                temps[n] = fresh
        if moved < 1e-4:
            break
    return temps


def tau_minutes(cfg=CFG):
    """The board's time constant. A run has to be several times it."""
    return cfg['board_capacity'] * cfg['board_to_ambient'] / SECONDS_PER_MINUTE


def settled_fraction(minutes, cfg=CFG):
    """How far toward equilibrium a run of that length gets, 0..1."""
    return 1.0 - math.exp(-minutes / tau_minutes(cfg))


#: `thermal_losses`: what `thermal_power_estimate` runs on, the board's before its record.
LOSSES = {
    'rds_on': inverter.RDS_ON, 'rds_alpha': 5.55e-3, 'rds_beta': 1.78e-5, 'r_shunt': inverter.SHUNT,
    'r_hotswap': 3.6e-3, 'switching_watt': 1.20, 'switch_volts': 24.6, 'driver_share': 1.0,
    'mcu_watt': 0.666, 'ldo_watt': 0.534, 'afe_watt': 0.13, 'f_sw': inverter.FSW,
    'coss_cjo': 15.6e-9, 'coss_m': 0.45, 'coss_vj': 0.7, 't_switch_s': 14.0e-9, 'v_sd': 0.85,
    'q_g': 267.0e-9, 'v_drive': 15.0, 'buck_eff': 0.85, 'r_phase': 0.05, 'k_iron': 0.0,
    'mcu_sleep_watt': 0.49,
}


def coss_energy(volts, loss=None):
    """`thermal_coss_energy`: J a transition at `volts`, the integral of v C(v)."""
    loss = loss or LOSSES
    cjo, vj, m = loss['coss_cjo'], loss['coss_vj'], loss['coss_m']
    if not (volts > 0.0 and cjo > 0.0 and vj > 0.0 and m < 1.0):
        return 0.0
    u = 1.0 + volts / vj
    e = cjo * vj * vj * ((u ** (2.0 - m) - 1.0) / (2.0 - m) - (u ** (1.0 - m) - 1.0) / (1.0 - m))
    return max(e, 0.0)


def power_estimate(load, phase_c=None, loss=None):
    """`thermal_power_estimate`: W per node for `load`, `thermal_load_t`'s fields by name
    (phase_amps, phase_sq, duty, link_volts, link_amps, switching, afe_on, speed_rpm, t_dead_s;
    absent is zero, link_amps -1), the FETs at `phase_c`'s three driver nodes."""
    loss = loss or LOSSES
    out = dict.fromkeys(ALL_NODES, 0.0)
    amps = load.get('phase_amps') or (0.0, 0.0, 0.0)
    squares = load.get('phase_sq') or (0.0, 0.0, 0.0)
    duty = load.get('duty') or (0.0, 0.0, 0.0)
    link = load.get('link_volts') or 0.0
    link = link if link > 0.0 else loss['switch_volts']
    e_cal = coss_energy(loss['switch_volts'], loss)
    scale = (coss_energy(link, loss) / e_cal if e_cal > 0.0
             else link / loss['switch_volts'] if loss['switch_volts'] > 0.0 else 1.0)
    per_leg = loss['switching_watt'] / 3.0 * scale
    from_phases = total = 0.0
    for leg, (driver, phase) in enumerate(zip(DRIVERS, PHASES)):
        rds = loss['rds_on']
        if phase_c is not None and not math.isnan(phase_c[leg]):
            d = phase_c[leg] - 25.0
            rds *= max(0.5, 1.0 + loss['rds_alpha'] * d + loss['rds_beta'] * d * d)
        sq = squares[leg] if squares[leg] > 0.0 else amps[leg] * amps[leg]
        irms = math.sqrt(sq)
        out[driver] += sq * rds
        out[phase] = sq * loss['r_shunt']
        from_phases += duty[leg] * amps[leg]
        total += sq
        if load.get('switching') and duty[leg] > 0.0:
            out[driver] += per_leg * loss['driver_share']
            out['regulators'] += per_leg * (1.0 - loss['driver_share'])
            gate = 2.0 * loss['q_g'] * loss['v_drive'] * loss['f_sw']
            out[driver] += (link * irms * loss['t_switch_s'] * loss['f_sw']
                            + 2.0 * loss['v_sd'] * 0.9 * irms * (load.get('t_dead_s') or 0.0)
                            * loss['f_sw'] + gate)
            if loss['buck_eff'] > 0.0:
                out['regulators'] += gate * (1.0 / loss['buck_eff'] - 1.0)
    link_amps = load.get('link_amps', -1.0)
    link_amps = link_amps if link_amps >= 0.0 else from_phases
    out['hotswap'] += link_amps * link_amps * loss['r_hotswap']
    out['winding'] += total * loss['r_phase']
    rpm = load.get('speed_rpm') or 0.0
    if loss['k_iron'] > 0.0 and rpm > 0.0:
        out['stator'] += loss['k_iron'] * (rpm / 1000.0) ** 2
    out['mcu'] += (loss['mcu_watt'] if load.get('afe_on') or load.get('switching')
                   else loss['mcu_sleep_watt'])
    out['regulators'] += loss['ldo_watt']
    out['afe'] += loss['afe_watt'] if load.get('afe_on') else 0.0
    return out


#: The FET's share of the conduction path: Rds(on) 1.8 against the 3.5 mOhm
#: shunt (`inverter`), for callers passing one lumped `r_phase`.
SWITCH_SHARE = inverter.RDS_ON / (inverter.RDS_ON + inverter.SHUNT)


def phase_power(amps_rms, r_phase, switching=True, cfg_power=None):
    """Power per node at `amps_rms` a phase: the conduction split between
    the FET and the shunt it sits in series with, the drivers' switching
    share where the stage is switching, the housekeeping always.
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
    """Amps rms a phase the board holds for ever, against `ceiling_c`."""
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
    """Node resistances from a thermal camera's degrees, K/W."""
    power = power or POWER_SWITCHING
    out = {}
    for name, celsius in camera.items():
        if celsius is None or not power.get(name):
            continue
        out[name] = (celsius - board_c) / power[name]
    return out


#: The online identification (`thermal_ident.h`): its scales in wire order,
#: the two a cooldown moves (spread and NTC are unobservable from one,
#: 2026-09-05), its states (words; the envelope acts on the continuous
#: `thermal_ident_margin`) and the margin's floor - the record's
#: `soa_margin_floor_ppm`, 80 % of the SOA by default when switching
#: starts.
IDENT_SCALES = ('air', 'capacity', 'spread', 'ntc')
IDENT_ONLINE = ('air', 'capacity')
IDENT_STATES = ('UNCERTAIN', 'CONVERGING', 'STABLE')
IDENT_MARGIN_FLOOR = 0.80


#: The record's default ceilings, C: laminate 105, motor 120, silicon 125, a leg
#: its FETs' junction, `inverter.T_J_MAX` (`board_cal.c`). In force: the
#: reference plus margin x span (`ceiling_of`); a board's own live in its
#: record (cal op 0).
CEILING_REF_C = 25.0
CEILING_DEFAULT_C = 125.0
CEILING_C = dict([(n, 105.0) for n in LAMINATE] + [(n, 120.0) for n in MOTOR]
                 + [(n, inverter.T_J_MAX) for n in DRIVERS])


def judged(node, celsius, watt, cfg=CFG):
    """`thermal_junction`, what the envelope judges a node on: its die where it has one
    (`rth_die`) - a leg's node two FETs and a driver, each FET half its watts - itself where
    not."""
    rth = cfg['rth_die'].get(node, 0.0)
    return celsius + (0.5 if node in DRIVERS else 1.0) * watt * rth if rth > 0.0 else celsius


def ceiling_of(node, margin=1.0, ceilings=None):
    """One node's ceiling in force: the record's, its span over the reference
    trimmed by the identification's margin.
    """
    top = (ceilings or CEILING_C).get(node, CEILING_DEFAULT_C)
    return CEILING_REF_C + margin * (top - CEILING_REF_C)
