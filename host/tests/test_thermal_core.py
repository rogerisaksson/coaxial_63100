"""The thermal envelope, run as the C that will run on the board."""
import ctypes
import math
import os
import sys

from tools.cores.build import build, find_cc
from tools.cores.thermal import (AMBIENT, BOARD_LIMIT_C, LAMINATE, LIMIT_C,
                                 LOOKAHEAD_S, NODES, SCALES, SOURCES, THERMAL,
                                 THROTTLE_AT, WINDING_LIMIT_C, GroundTruth,
                                 Ident, Model, edges, losses, power)

from test_modbus_core import Report

#: The edges as `thermal.c`'s table lays them, named here so a failure
#: says which two nodes and not which index.
EDGE_WINDING_STATOR, EDGE_STATOR_ROTOR, EDGE_MOUNT_FIRST = 22, 23, 24

#: How far the thermistor's element sits toward the leg node, as
#: `thermal_defaults` sets it. Named here rather than read back, because a
#: test that asked the code for its own expectation would agree with a
#: typo - and it is `electronics/`'s pick and place that says what it
#: should be, which `test_sensorless.py` checks against the file.
NTC_SEES_LEG = 0.30

#: K/W from a leg's switches into the laminate under them, as
#: `thermal_defaults` sets it. Named for the same reason as the fraction
#: above; the patch's own 15 K/W to the rest of the board is what makes
#: the leg's 27 - the record's 28 - and it is the graph's, not this
#: number's.
DRIVER_TO_PATCH = 12.0


#: The mirror's numbers are the C's. `coaxial/kalman/thermal_ident.py` says
#: every number in it has the same name and value as in the C; this
#: reads both and holds them to it, so the stand-in identifies the way
#: the board will and a constant tuned on one cannot drift from the
#: other. The bench, 2026-09-06: "check that the C on the target is a
#: one-to-one mapping of what is in Python - without the ground truth,
#: of course, which comes from real sensor values".
C_TO_MIRROR = (
    ('IDENT_DT_SLICE', 'SLICE_S'), ('IDENT_EPS_T', 'EPS_T'),
    ('IDENT_EPS_S', 'EPS_S'), ('IDENT_EPS_AMB', 'EPS_AMB'),
    ('IDENT_NOISE_GAIN', 'NOISE_GAIN'),
    ('IDENT_EXCITATION_MIN', 'EXCITATION_MIN'),
    ('IDENT_INNOVATION_FOLLOW', 'INNOVATION_FOLLOW'),
    ('IDENT_SETTLE_SAMPLES', 'SETTLE_SAMPLES'),
    ('IDENT_GATE_SIGMAS', 'GATE_SIGMAS'), ('IDENT_STILL_GAIN', 'STILL_GAIN'),
    ('IDENT_MOVE_SHARE', 'MOVE_SHARE'), ('IDENT_RATIO_STABLE', 'RATIO_STABLE'),
    ('IDENT_RATIO_UNCERTAIN', 'RATIO_UNCERTAIN'),
    ('IDENT_STABLE_RUNS', 'STABLE_RUNS'),
    ('THERMAL_IDENT_MIN_HORIZON_S', 'MIN_HORIZON_S'),
    ('THERMAL_IDENT_MAX_HORIZON_S', 'MAX_HORIZON_S'),
    ('THERMAL_IDENT_BLIND_S', 'BLIND_S'),
    ('THERMAL_IDENT_SCALE_MIN', 'SCALE_MIN'),
    ('THERMAL_IDENT_SCALE_MAX', 'SCALE_MAX'),
    ('THERMAL_IDENT_AMBIENT_MIN_C', 'AMBIENT_MIN_C'),
    ('THERMAL_IDENT_AMBIENT_MAX_C', 'AMBIENT_MAX_C'),
    ('THERMAL_IDENT_RECORD', 'RECORD'),
    ('THERMAL_ANCHOR_HZ', 'ANCHOR_HZ'),
    ('THERMAL_NTC_INVERT_MAX_S', 'NTC_INVERT_MAX_S'),
    ('THERMAL_NTC_AT_LEG_K', 'NTC_AT_LEG_K'),
)
C_TABLES = ('PRIOR_SIGMA', 'SIGMA_CONVERGING', 'SIGMA_STABLE', 'DRIFT_VAR',
            'FLOOR_SHARE', 'VAR_MAX', 'ONLINE')


def c_numbers():
    """Every `#define NAME number` and every `static const float|bool
    NAME[...] = { ...
    """
    import re

    text = ''
    for name in ('src/thermal_ident.c', 'inc/thermal_ident.h', 'src/thermal.c',
                 'inc/thermal.h'):
        with open(os.path.join(THERMAL, name), encoding='utf-8') as f:
            text += f.read() + '\n'

    def number(token):
        token = token.strip().rstrip('fFuUlL')
        if token in ('true', 'false'):
            return token == 'true'
        return float(token)

    text = re.sub(r'/\*.*?\*/', '', text, flags=re.S)     # comments carry numbers too
    scalars = {m.group(1): number(m.group(2)) for m in re.finditer(
        r'^#define\s+([A-Z_0-9]+)\s+(-?[0-9][0-9.eE+-]*[fFuUlL]?)\s*$',
        text, re.M)}
    tables = {}
    for m in re.finditer(r'static const (?:float|bool)\s+([A-Z_]+)\[[^\]]*\]'
                         r'\s*=\s*\{([^}]*)\}', text):
        try:
            tables[m.group(1)] = tuple(number(v) for v in m.group(2).split(',')
                                       if v.strip())
        except ValueError:
            continue                        # a table of names, not numbers
    return scalars, tables


def test_the_mirror_carries_the_cs_numbers(report, lib):
    """Every constant the mirror names is the C's, by name and value."""
    from coaxial.kalman import thermal_ident as mirror

    scalars, tables = c_numbers()
    missing = [c for c, _p in C_TO_MIRROR if c not in scalars]
    missing += [t for t in C_TABLES if t not in tables]
    report.check('every constant this check names is found in the C',
                 not missing, 'missing %s' % missing)
    off = []
    for c_name, py_name in C_TO_MIRROR:
        if c_name in scalars:
            c, p = scalars[c_name], getattr(mirror, py_name)
            if abs(float(c) - float(p)) > 1e-9 * max(1.0, abs(float(c))):
                off.append('%s %s != %s %s' % (c_name, c, py_name, p))
    report.check('and every scalar - slices, gains, gates, ratios, horizons, '
                 'clamps, the anchor\'s three - is the mirror\'s to the number',
                 not off, off)
    off = []
    for name in C_TABLES:
        if name in tables:
            c, p = tables[name], tuple(getattr(mirror, name))
            if len(c) != len(p) or any(
                    abs(float(a) - float(b)) > 1e-9 * max(1.0, abs(float(a)))
                    for a, b in zip(c, p)):
                off.append('%s C %s mirror %s' % (name, c, p))
    report.check('and every table over the five quantities - priors, the '
                 'two thresholds, drift, floor shares, ceilings, which are '
                 'online - is the same five numbers',
                 not off, off)


def test_the_mirror_walks_with_the_c(report, lib):
    """The C chain and the mirror chain on the same watts and the same
    readings, sample by sample: one-to-one measured on a walk, not only
    on the constants.
    """
    import copy
    from coaxial.model import thermal
    from coaxial.kalman import thermal_ident as mirror

    truth = GroundTruth(lib, air=2.0)
    observer = Model(lib)
    ident = Ident(lib, observer)
    watt = power(lib, phase_sq=(900.0, 900.0, 900.0), duty=(0.5, 0.5, 0.5),
                 link_volts=48.0, switching=True)
    quiet = power(lib)
    blind = (math.nan, math.nan, math.nan)

    # The tape, off the C chain: every second's watts and readings, and the C's
    # answer at every reading.
    tape, c_trace = [], []
    for _cycle in range(2):
        for phase, watts, seconds in (('run', watt, 600), ('cool', quiet, 1200)):
            for step in range(seconds):
                truth.model.step(watts, 1.0)
                seen = blind
                if phase == 'cool' and (step + 1) % 30 == 0:
                    seen = (truth.model.ntc() + truth.noise(),
                            truth.model.junction({}, 'afe') + truth.noise(),
                            truth.model.junction({}, 'mcu') + truth.noise())
                tape.append((watts, seen))
                ident.run(watts, 1.0, seen)
                if seen is not blind:
                    c_trace.append((ident.state(), ident.scale('air'),
                                    ident.scale('capacity'), ident.ambient(),
                                    ident.innovation(), ident.margin(),
                                    observer.ntc(), observer.at('mcu'),
                                    observer.at('board')))

    # The mirror on the tape, as `SimulatedThermal._integrate` runs it.
    base = copy.deepcopy(thermal.CFG)
    node = {n: AMBIENT for n in thermal.ALL_NODES}
    ntc, ambient, since = AMBIENT, AMBIENT, 0.0
    py = mirror.Identifier(0.1, AMBIENT)
    cfg = py.apply(base)
    p_trace = []
    for watts, seen in tape:
        net = thermal.net_flows(node, watts, cfg, ambient, 0.0)
        for name in thermal.ALL_NODES:
            cap = cfg['capacity'].get(name, 0.0)
            if cap > 0.0:
                node[name] += net[name] / cap
        since += 1.0
        sample = None
        if not math.isnan(seen[0]):
            sample = {'ntc': seen[0], 'afe': seen[1], 'mcu': seen[2]}
            ntc, _settled = mirror.anchor(node, ntc, cfg, watts, sample, 0.0,
                                          since, ambient)
            since = 0.0
        ntc = mirror.ntc_follow(node, ntc, cfg, 1.0)[0]
        if py.step(node, ntc, base, watts, 0.0, sample, 1.0):
            cfg = py.apply(base)
        ambient = py.ambient
        if sample:
            p_trace.append((py.state, py.scale[0], py.scale[1], py.ambient,
                            py.innovation_k, py.margin(0.8), ntc, node['mcu'],
                            node['board']))

    worst = [max(abs(c[k] - p[k]) for c, p in zip(c_trace, p_trace))
             for k in range(1, 9)]
    apart = sum(1 for c, p in zip(c_trace, p_trace) if c[0] != p[0])
    report.check('eighty readings on the tape, and the C and the mirror say '
                 'the same state at every one of them',
                 len(c_trace) == 80 and len(p_trace) == 80 and apart == 0,
                 '%d and %d samples, states apart at %d'
                 % (len(c_trace), len(p_trace), apart))
    report.check('the identification agrees to the hundredth - air and '
                 'capacity within 0.01, the room within 0.2 K, the judged '
                 'innovation within 0.01 K, the margin within 0.01',
                 worst[0] < 0.01 and worst[1] < 0.01 and worst[2] < 0.2
                 and worst[3] < 0.01 and worst[4] < 0.01,
                 'air %.4f cap %.4f room %.3f inn %.4f margin %.4f'
                 % tuple(worst[:5]))
    report.check('and the two observers under them within a tenth of a '
                 'kelvin on the thermistor, the MCU and the centre',
                 all(w < 0.1 for w in worst[5:]),
                 'ntc %.3f mcu %.3f board %.3f' % tuple(worst[5:]))


def test_the_room_is_identified(report, lib):
    """The board has no ambient sensor: the room is the fifth quantity."""
    truth = GroundTruth(lib, air=1.0)
    observer = Model(lib)
    ident = Ident(lib, observer)
    lib.thm_ambient.argtypes = [ctypes.c_void_p, ctypes.c_float]
    watt = power(lib, phase_sq=(900.0, 900.0, 900.0), duty=(0.5, 0.5, 0.5),
                 link_volts=48.0, switching=True)
    quiet = power(lib)
    truth.cycle(ident, watt, 600.0, 1200.0)
    # Within six kelvin after one cycle: the room's wide prior lets it take
    # some of the cooldown's early state error, which the anchored nodes do not
    # feel and the next cycles correct.
    report.check('a run and a cooldown in the room it woke in: the room is '
                 'identified within six kelvin, the air scale near one',
                 abs(ident.ambient() - 25.0) < 6.0
                 and abs(ident.scale('air') - 1.0) < 0.4,
                 'room %.1f C, air %.2f, %s'
                 % (ident.ambient(), ident.scale('air'), ident.state()))
    # Out into the cold, idling: the board cools toward a room the model does
    # not know.
    lib.thm_ambient(truth.model.h, -20.0)
    trace = []
    truth.cycle(ident, quiet, 0.0, 2400.0, trace=trace)
    states = [t[0] for t in trace]
    # The room is reset when the model stops predicting (2026-09-06): its sigma
    # back to the ten-kelvin prior at the first UNCERTAIN sample, the air
    # scale's untouched, so the step is charged to the room.
    at_reset = [t[5] for t in trace if t[0] == 'UNCERTAIN']
    report.check('carried to -20 C: UNCERTAIN, with the room\'s sigma back '
                 'at its prior on the first such sample, then the room found '
                 'within forty minutes and the air scale not blamed for it',
                 'UNCERTAIN' in states and at_reset
                 and abs(at_reset[0] - 10.0) < 1.0
                 and abs(ident.ambient() + 20.0) < 6.0
                 and abs(ident.scale('air') - 1.0) < 0.5,
                 'room %.1f C, air %.2f, %s' % (ident.ambient(),
                                               ident.scale('air'),
                                               ' > '.join(s for i, s in enumerate(states)
                                                          if i == 0 or s != states[i - 1])))
    lib.thm_ambient(truth.model.h, 25.0)
    truth.cycle(ident, quiet, 0.0, 2400.0)
    report.check('and back into the room: found again',
                 abs(ident.ambient() - 25.0) < 6.0
                 and abs(ident.scale('air') - 1.0) < 0.5,
                 'room %.1f C, air %.2f, %s' % (ident.ambient(),
                                               ident.scale('air'), ident.state()))


def test_an_idle_board_stays_uncertain(report, lib):
    """A board that is not switching gives the identification nothing."""
    truth = GroundTruth(lib, air=2.0)
    observer = Model(lib)
    ident = Ident(lib, observer)
    quiet = power(lib)                 # the housekeeping alone
    # Settle both at their idle equilibria first, blind: the warm-up from a
    # cold start is a transient of its own and not what is asked.
    blind = (math.nan, math.nan, math.nan)
    for _ in range(3600):
        truth.model.step(quiet, 1.0)
        ident.run(quiet, 1.0, blind)
    # Then forty minutes idling on the housekeeping, read every thirty seconds
    # as the board reads: `cycle` cools on no power at all, a cooldown and not
    # an idle.
    for step in range(2400):
        truth.model.step(quiet, 1.0)
        seen = blind
        if (step + 1) % 30 == 0:
            seen = (truth.model.ntc() + truth.noise(),
                    truth.model.junction(quiet, 'afe') + truth.noise(),
                    truth.model.junction(quiet, 'mcu') + truth.noise())
        ident.run(quiet, 1.0, seen)
    report.check('forty idle minutes in a box, sampled every thirty '
                 'seconds: no sample moved the scales',
                 ident.updates() == 0,
                 '%d updates, air %.2f' % (ident.updates(), ident.scale('air')))
    report.check('and the board stays UNCERTAIN - its margin in hand until '
                 'something switches',
                 ident.state() == 'UNCERTAIN', ident.state())
    # The margin is the floor, whatever the floor is set to: idle is no
    # evidence. The innovation alone would say full span here: at rest it sits
    # at the thermometers' floor.
    report.check('and the margin is the floor it was given - 0.80, 0.70 - '
                 'the model doubted whole, since idle taught it nothing',
                 abs(ident.margin(0.8) - 0.8) < 1e-6
                 and abs(ident.margin(0.7) - 0.7) < 1e-6
                 and abs(ident.doubt() - 1.0) < 1e-6,
                 'margin %.3f / %.3f, doubt %.2f'
                 % (ident.margin(0.8), ident.margin(0.7), ident.doubt()))


def test_the_scales_are_identified_against_a_ground_truth(report, lib):
    """The identification finds a board's air path from its own
    thermometers, says how sure it is, and notices when the situation
    changes.
    """
    truth = GroundTruth(lib, air=2.0)
    observer = Model(lib)
    ident = Ident(lib, observer)
    watt = power(lib, phase_sq=(900.0, 900.0, 900.0), duty=(0.5, 0.5, 0.5),
                 link_volts=48.0, switching=True)
    report.check('it starts uncertain, at the derived defaults',
                 ident.state() == 'UNCERTAIN'
                 and all(abs(ident.scale(s) - 1.0) < 1e-6 for s in SCALES),
                 '%s, %s' % (ident.state(),
                             ['%.2f' % ident.scale(s) for s in SCALES]))
    trace = []
    for _ in range(3):
        truth.cycle(ident, watt, 600.0, 1200.0, trace=trace)
    air = ident.scale('air')
    report.check('three cycles in a box: the air scale lands near two',
                 abs(air - 2.0) < 0.4,
                 '%.2f +- %.2f after %d updates'
                 % (air, ident.sigma('air'), ident.updates()))
    report.check('and it says so: CONVERGING or STABLE, the innovation '
                 'near the thermometers\' floor',
                 ident.state() in ('CONVERGING', 'STABLE')
                 and ident.innovation() < 0.5,
                 '%s, innovation %.2f K' % (ident.state(), ident.innovation()))
    states = [t[0] for t in trace]
    report.check('the state went UNCERTAIN, then CONVERGING',
                 states[0] == 'UNCERTAIN' and 'CONVERGING' in states,
                 ' > '.join(s for i, s in enumerate(states)
                            if i == 0 or s != states[i - 1]))
    report.check('the other scales stayed near one - nothing about them '
                 'in the data was allowed to move them far',
                 all(abs(ident.scale(s) - 1.0) < 0.5
                     for s in ('capacity', 'spread', 'ntc')),
                 ['%.2f' % ident.scale(s) for s in SCALES])
    # The margin is continuous (2026-09-06): the floor with the model doubted
    # whole, one when not at all, the evidence between; the floor is the
    # caller's.
    boxed = ident.margin(0.8)
    report.check('the margin has risen off its 0.80 floor to the whole span '
                 'on the evidence of the cooldowns - the doubt gone, the '
                 'floor no longer shows',
                 0.95 < boxed <= 1.0 and abs(ident.margin(0.7) - boxed) < 1e-6
                 and abs(ident.margin(1.0) - 1.0) < 1e-6,
                 'margin %.3f at 0.8, %.3f at 0.7, doubt %.2f'
                 % (boxed, ident.margin(0.7), ident.doubt()))

    # The box comes off, a fan goes on.
    settled = ident.state()
    truth.situation(0.5)
    trace = []
    truth.cycle(ident, watt, 600.0, 1200.0, trace=trace)
    states = [t[0] for t in trace]
    # The margin falls with the state: within the fan's first cooldown the
    # innovation says the model is wrong and the envelope is back near its
    # floor, as far as the evidence says.
    least = min(t[4] for t in trace)
    report.check('and within the fan\'s first cooldown the margin fell back '
                 'toward the floor',
                 least < boxed - 0.05 and least >= 0.8 - 1e-6,
                 'least %.3f after %.3f' % (least, boxed))
    report.check('a halved air path is caught: UNCERTAIN within the first '
                 'cooldown after it (from %s)' % settled,
                 'UNCERTAIN' in states,
                 ' > '.join(s for i, s in enumerate(states)
                            if i == 0 or s != states[i - 1]))
    for _ in range(3):
        truth.cycle(ident, watt, 600.0, 1200.0, trace=trace)
    air = ident.scale('air')
    report.check('and four cycles later the air scale has re-landed near '
                 'a half', abs(air - 0.5) < 0.15,
                 '%.2f +- %.2f, %s' % (air, ident.sigma('air'), ident.state()))
    report.check('with the state back to CONVERGING or STABLE',
                 ident.state() in ('CONVERGING', 'STABLE'), ident.state())

    # Nothing is resumed (2026-09-06, the bench's rule): a fresh identifier is
    # at the floor and doubted whole, whatever an earlier run found. A good
    # observer earns its span within a few samples; a resumed one runs on last
    # week's box.
    fresh = Ident(lib, Model(lib))
    report.check('a fresh identifier is UNCERTAIN at one, its margin at the '
                 'floor - 0.8, or 0.7 if that is the floor - and its doubt '
                 'whole; nothing is resumed',
                 fresh.state() == 'UNCERTAIN'
                 and abs(fresh.scale('air') - 1.0) < 1e-6
                 and abs(fresh.margin(0.8) - 0.8) < 1e-6
                 and abs(fresh.margin(0.7) - 0.7) < 1e-6
                 and abs(fresh.doubt() - 1.0) < 1e-6,
                 '%s %.2f margin %.3f' % (fresh.state(), fresh.scale('air'),
                                          fresh.margin(0.8)))


def wanted(spent, throttle_at=THROTTLE_AT):
    """The derate the ramp's own definition gives for a spend."""
    if spent <= throttle_at:
        return 1.0
    over = (spent - throttle_at) / (1.0 - throttle_at)
    return 0.0 if over >= 1.0 else 1.0 - over


def test_the_derate_is_a_ramp(report, lib):
    """One below the throttle point, zero at the ceiling, linear between."""
    model = Model(lib)
    span = LIMIT_C - AMBIENT
    worst = 0.0
    for celsius in range(int(AMBIENT), int(LIMIT_C) + 6):
        model.place('phase_u', float(celsius))
        got = model.budget()
        worst = max(worst, abs(got['derate'] - wanted(got['worst'])))
    report.check('the derate follows its own definition across the band',
                 worst < 0.01, 'worst %.4f off' % worst)

    model.place('phase_u', AMBIENT)
    report.check('at ambient the clamp is untouched',
                 model.budget()['derate'] == 1.0)

    model.place('phase_u', AMBIENT + THROTTLE_AT * span - 1.0)
    report.check('and still untouched just below the throttle point',
                 model.budget()['derate'] == 1.0,
                 '%.1f C' % model.at('phase_u'))

    model.place('phase_u', LIMIT_C)
    at_limit = model.budget()
    report.check('zero at the ceiling', at_limit['derate'] == 0.0,
                 '%.3f' % at_limit['derate'])

    model.place('phase_u', LIMIT_C + 40.0)
    over = model.budget()
    report.check('and zero past it, not negative - a clamp is a multiplier',
                 over['derate'] == 0.0, '%.3f' % over['derate'])
    report.check('the board acts once it is over: tripped', over['tripped'])


def test_derating_is_not_tripping(report, lib):
    """Inside the band the stage is still driving."""
    model = Model(lib)
    span = LIMIT_C - AMBIENT
    model.place('phase_u', AMBIENT + 0.92 * span)
    got = model.budget()
    report.check('mid-band it is throttling', got['throttling'],
                 '%.1f C, %.0f %% spent' % (model.at('phase_u'),
                                            100.0 * got['worst']))
    report.check('and NOT tripped - the stage is still driving',
                 not got['tripped'])
    report.check('at a clamp between the two, neither one nor zero',
                 0.0 < got['derate'] < 1.0, '%.3f' % got['derate'])


def test_the_lookahead_catches_a_ramp(report, lib):
    """The throttle acts on time left, not on a projected temperature."""
    watt = {'phase_u': 35.0}
    first = {}
    for name, ahead in (('now', 0.0), ('soon', LOOKAHEAD_S)):
        model = Model(lib)
        for step in range(4000):
            model.step(watt, 0.02)
            got = model.budget(watt, lookahead_s=ahead)
            if got['derate'] < 0.999:
                first[name] = (step * 0.02, model.at('phase_u'), got['worst'])
                break

    report.check('both eventually back off - a node heading there must',
                 'now' in first and 'soon' in first, str(first))
    if len(first) == 2:
        report.check('the time rule backs off EARLIER than the present does',
                     first['soon'][0] < first['now'][0],
                     '%.2f s ahead against %.2f s' % (first['soon'][0],
                                                      first['now'][0]))
        report.check('and does it with the node still cooler',
                     first['soon'][1] < first['now'][1],
                     '%.1f C against %.1f C' % (first['soon'][1],
                                                first['now'][1]))
        report.check('the present-only one waits for the throttle point '
                     'itself, which is what arrives too late',
                     first['now'][2] >= THROTTLE_AT - 0.01,
                     '%.3f spent' % first['now'][2])

    cold = Model(lib)
    for watts in (8.0, 35.0):
        got = cold.budget({'phase_u': watts}, lookahead_s=LOOKAHEAD_S)
        report.check('%.0f W from ambient leaves the clamp open - the burst '
                     'is shaped, not forbidden' % watts,
                     got['derate'] == 1.0,
                     'clamp %.2f, %.0f %% spent' % (got['derate'],
                                                    100.0 * got['worst']))

    # Up to what the node can hold for the reaction window, and no further.
    fault = cold.budget({'phase_u': 300.0}, lookahead_s=LOOKAHEAD_S)
    report.check('a power past what the node can hold for the window is '
                 'throttled from cold, and that is the rule, not a hole in it',
                 0.0 < fault['derate'] < 1.0 and not fault['tripped'],
                 'clamp %.2f at ambient' % fault['derate'])

    at = {}
    for window in (0.5, 1.0, 2.0, 4.0):
        model = Model(lib)
        for step in range(4000):
            model.step(watt, 0.02)
            if model.budget(watt, lookahead_s=window)['derate'] < 0.999:
                at[window] = model.at('phase_u')
                break
    order = [at[w] for w in sorted(at)]
    report.check('a longer window backs off earlier, and never refuses to '
                 'start',
                 len(at) == 4 and order == sorted(order, reverse=True),
                 ', '.join('%.1f s: %.0f C' % (w, at[w])
                           for w in sorted(at)))

    # One definition: the throttle acts on the hold, and `millis_to_limit`
    # reports the hold.
    model = Model(lib)
    model.place('phase_u', 100.0)
    got = model.budget(watt, lookahead_s=LOOKAHEAD_S)
    hold = got['millis'] / 1000.0
    report.check('the derate follows the same hold the board reports',
                 abs(got['derate'] - wanted(1.0 - hold / LOOKAHEAD_S)) < 0.01,
                 'clamp %.3f at %.3f s of hold' % (got['derate'], hold))

    # Zero disables it, bit for bit: a record without the field reads back as
    # zero and gets the pre-lookahead envelope.
    model = Model(lib)
    # 114 C: lower, the node holds 35 W for longer than the ramp's 0.2 s,
    # outside the window, and both rules answer an untouched 1.0.
    model.place('phase_u', 114.0)
    report.check('a cooling node has no hold to run out of',
                 model.budget(lookahead_s=LOOKAHEAD_S)['derate'] == 1.0)
    hot = model.budget(watt, lookahead_s=LOOKAHEAD_S)['derate']
    flat = model.budget(watt, lookahead_s=0.0)['derate']
    report.check('and with power on it, the window is what differs',
                 hot < flat, 'window %.3f against present-only %.3f'
                 % (hot, flat))


def test_the_step_must_land_inside_the_ramp(report, lib):
    """A throttle band is only there if something looks inside it."""
    # The real load, every node live.
    watt = power(lib, phase_amps=(100.0, 0.0, 0.0), duty=(0.5, 0.0, 0.0),
                 link_volts=48.0, switching=True)
    first = {}
    for dt in (0.1, 0.5, 1.0, 2.0):
        model = Model(lib)
        gone, acted = 0.0, None
        while gone < 2.0 - 1e-9:
            model.step(watt, dt)
            gone += dt
            got = model.budget(watt, lookahead_s=LOOKAHEAD_S)
            if acted is None and got['derate'] < 0.999:
                acted = (got['worst'], got['derate'], model.at('driver_u'))
        first[dt] = acted

    report.check('at the step the firmware takes, the throttle sees the '
                 'band while the worst node is still under its ceiling',
                 first[0.1] is not None and first[0.1][0] < 1.0,
                 '%.0f %% spent, clamp %.2f, driver %.0f C'
                 % (100.0 * first[0.1][0], first[0.1][1], first[0.1][2]))
    report.check('and the clamp is still near open when it first acts - a '
                 'ramp, not a cliff',
                 first[0.1][1] > 0.5, '%.2f' % first[0.1][1])

    # The counter-example: at a step longer than the ramp the first evaluation
    # is already past the ceiling with the clamp shut, and only the trip is
    # left.
    report.check('a step ten times longer steps over the band entirely',
                 first[1.0] is not None and first[1.0][0] >= 1.0
                 and first[1.0][1] == 0.0,
                 '%.0f %% spent, clamp %.2f, driver %.0f C'
                 % (100.0 * first[1.0][0], first[1.0][1], first[1.0][2]))
    report.check('and the longest step the core will take is worse still',
                 first[2.0][2] > first[1.0][2],
                 'driver %.0f C against %.0f C' % (first[2.0][2],
                                                   first[1.0][2]))


def test_the_soak_is_joules(report, lib):
    """Capacity times the rise still available, per node."""
    model = Model(lib)
    model.place('phase_u', 45.0)
    model.place('board', 30.0)
    got = model.budget()

    for name, celsius, limit in (('phase_u', 45.0, LIMIT_C),
                                 ('board', 30.0, BOARD_LIMIT_C)):
        want = model.capacity(name) * (limit - celsius)
        report.check('%s soak is capacity times the rise left' % name,
                     abs(got['soak_j'][name] - want) < 0.01,
                     '%.2f J against %.2f' % (got['soak_j'][name], want))

    # A node past its ceiling has no budget, not a debt: the trip says it is
    # over, and a negative joule count would divide into a negative burst
    # length.
    model.place('phase_u', LIMIT_C + 20.0)
    report.check('past the ceiling the soak is zero, never negative',
                 model.budget()['soak_j']['phase_u'] == 0.0)

    # Joules, so a burst length at any power, not only the present one.
    model.place('phase_u', 45.0)
    joules = model.budget()['soak_j']['phase_u']
    report.check('and it divides into a burst length at any power',
                 abs(joules / 35.0
                     - model.capacity('phase_u') * 80.0 / 35.0) < 0.01,
                 '%.2f s at 35 W' % (joules / 35.0))


def test_the_worst_node_is_the_one_acted_on(report, lib):
    """A stage is as close to its ceiling as its closest part."""
    model = Model(lib)
    model.place('phase_u', 100.0)
    model.place('board', 95.0)
    got = model.budget()
    report.check('the board at 95 of 105 beats a phase at 100 of 125',
                 got['worst_node'] == 'board',
                 '%s, %.0f %% against phase_u %.0f %%'
                 % (got['worst_node'], 100.0 * got['used']['board'],
                    100.0 * got['used']['phase_u']))
    report.check('the hottest node and the closest node are different here '
                 '- a temperature cannot say how close without its limit',
                 model.at('phase_u') > model.at('board'))


def test_a_throttle_weighs_only_what_a_clamp_can_cool(report, lib):
    """The housekeeping nodes are judged and not throttled on."""
    # 116 C is inside the 90 % band (114.5 C on this record); 110 C sits under
    # the point.
    model = Model(lib)
    model.place('regulators', 116.0)
    model.place('phase_u', 60.0)
    got = model.budget()
    report.check('a regulator at 116 of 125 does not become the worst node',
                 got['worst_node'] == 'phase_u',
                 '%s at %.0f %%, regulators at %.0f %%'
                 % (got['worst_node'], 100.0 * got['worst'],
                    100.0 * got['used']['regulators']))
    report.check('and it is still reported, at its own ceiling',
                 abs(got['used']['regulators']
                     - (116.0 - AMBIENT) / (LIMIT_C - AMBIENT)) < 0.01,
                 '%.3f' % got['used']['regulators'])
    report.check('so the clamp stays open on a board doing no work',
                 got['derate'] == 1.0, '%.2f' % got['derate'])

    # The mask is the record's, not the core's: hand it the other answer and
    # the same board derates.
    same = model.budget(undriven=())
    report.check('told every node is driven, the same board picks the '
                 'regulator back up', same['worst_node'] == 'regulators',
                 same['worst_node'])
    report.check('and derates on it', same['derate'] < 1.0,
                 '%.2f' % same['derate'])

    model.place('regulators', LIMIT_C + 5.0)
    hot = model.budget()
    report.check('a masked node at its ceiling still trips', hot['tripped'],
                 'worst %s at %.0f %%'
                 % (hot['worst_node'], 100.0 * hot['worst']))
    report.check('though it is not what the throttle is looking at',
                 hot['worst_node'] == 'phase_u', hot['worst_node'])


def test_the_conduction_is_split_where_it_is_made(report, lib):
    """The FET's watts on the driver node, the shunt's on the phase node."""
    loss = losses(lib)
    amps = 50.0
    got = power(lib, phase_amps=(amps, 0.0, 0.0), switching=False)
    conduction = amps * amps

    report.check('the shunt heats the phase node',
                 abs(got['phase_u'] - conduction * loss['r_shunt']) < 0.05,
                 '%.2f W against %.2f' % (got['phase_u'],
                                          conduction * loss['r_shunt']))
    report.check('and the FET heats the driver node beside it',
                 abs(got['driver_u'] - conduction * loss['rds_on']) < 0.05,
                 '%.2f W against %.2f' % (got['driver_u'],
                                          conduction * loss['rds_on']))
    report.check('the split is the resistances and nothing else',
                 abs(got['driver_u'] / got['phase_u']
                     - loss['rds_on'] / loss['r_shunt']) < 0.01,
                 '%.3f against %.3f' % (got['driver_u'] / got['phase_u'],
                                        loss['rds_on'] / loss['r_shunt']))

    # The FET's resistance climbs with its own node.
    hot = power(lib, phase_amps=(amps, 0.0, 0.0), switching=False,
                phase_c=(100.0, 25.0, 25.0))
    want = 1.0 + loss['rds_alpha'] * 75.0
    report.check('the FET share follows the node it heats, by the '
                 'datasheet chord',
                 abs(hot['driver_u'] / got['driver_u'] - want) < 0.01,
                 '%.3f against %.3f' % (hot['driver_u'] / got['driver_u'],
                                        want))
    report.check('and the shunt does not - it is a resistor, not silicon',
                 abs(hot['phase_u'] - got['phase_u']) < 0.01,
                 '%.3f W against %.3f' % (hot['phase_u'], got['phase_u']))


def test_conduction_is_a_mean_square_not_a_sample(report, lib):
    """One instant squared is the loss only if that instant was the rms."""
    peak = 100.0
    rms_sq = peak * peak / 2.0

    # The two instants a sampler can land on, and what each says the loss is
    # when it is squared on its own.
    at_peak = power(lib, phase_amps=(peak, 0.0, 0.0), switching=False)
    at_zero = power(lib, phase_amps=(0.0, 0.0, 0.0), switching=False)
    report.check('a sample at the peak claims twice the conduction',
                 abs(at_peak['phase_u'] / (rms_sq * losses(lib)['r_shunt'])
                     - 2.0) < 0.01,
                 '%.2f W against a true %.2f'
                 % (at_peak['phase_u'], rms_sq * losses(lib)['r_shunt']))
    report.check('and a sample at the zero crossing claims none at all',
                 at_zero['phase_u'] == 0.0, '%.3f W' % at_zero['phase_u'])

    # The mean square says the same thing wherever the sample fell.
    true_w = rms_sq * losses(lib)['r_shunt']
    for name, sample in (('at the peak', peak), ('at the crossing', 0.0)):
        got = power(lib, phase_amps=(sample, 0.0, 0.0), switching=False,
                    phase_sq=(rms_sq, 0.0, 0.0))
        report.check('with a mean square the conduction is the same %s'
                     % name,
                     abs(got['phase_u'] - true_w) < 0.01,
                     '%.3f W against %.3f' % (got['phase_u'], true_w))

    one_leg = power(lib, phase_amps=(0.0, 0.0, 0.0), switching=False,
                    phase_sq=(rms_sq, rms_sq, 0.0))
    report.check('an idle leg stays cold while two carry current',
                 one_leg['phase_w'] == 0.0 and one_leg['phase_u'] > 0.0,
                 'U %.2f W, W %.3f W' % (one_leg['phase_u'],
                                         one_leg['phase_w']))

    # Without a mean square the sample is squared, bit for bit: a caller with
    # only a sample (the harness, a host, a board whose sampler is not armed)
    # gets that rather than zero.
    report.check('no mean square means the sample is squared, as before',
                 power(lib, phase_amps=(peak, 0.0, 0.0), switching=False,
                       phase_sq=(0.0, 0.0, 0.0))['phase_u']
                 == at_peak['phase_u'])
    report.check('and a negative one is not measured either',
                 power(lib, phase_amps=(peak, 0.0, 0.0), switching=False,
                       phase_sq=(-1.0, 0.0, 0.0))['phase_u']
                 == at_peak['phase_u'])

    # The link estimate keeps the signed sample: a mean square has none.
    fwd = power(lib, phase_amps=(50.0, 0.0, 0.0), duty=(1.0, 0.0, 0.0),
                switching=False, phase_sq=(rms_sq, 0.0, 0.0))
    back = power(lib, phase_amps=(-50.0, 0.0, 0.0), duty=(1.0, 0.0, 0.0),
                 switching=False, phase_sq=(rms_sq, 0.0, 0.0))
    report.check('the conduction does not care which way the current went',
                 abs(fwd['phase_u'] - back['phase_u']) < 1e-6,
                 '%.3f against %.3f' % (fwd['phase_u'], back['phase_u']))
    report.check('and the hot swap still sees the same link current either '
                 'way, because it squares it too',
                 abs(fwd['hotswap'] - back['hotswap']) < 1e-6
                 and fwd['hotswap'] > 0.0,
                 '%.3f W against %.3f' % (fwd['hotswap'], back['hotswap']))


def test_the_thermistor_has_mass(report, lib):
    """A sensor a centimetre from the silicon cannot slew like silicon."""
    peak = 100.0
    watt = power(lib, phase_amps=(peak, -peak / 2, -peak / 2),
                 duty=(0.5, 0.5, 0.5), link_volts=48.0, switching=True)
    model = Model(lib)
    start = model.ntc()
    fastest, was = 0.0, start
    for _ in range(10):
        model.step(watt, 0.1)
        fastest = max(fastest, (model.ntc() - was) / 0.1)
        was = model.ntc()
    rose = model.ntc() - start
    report.check('a hard burst does not move the reading 60 K in a second',
                 rose < 60.0, '%.1f K in the first second' % rose)
    report.check('and the fastest it climbs is bounded by the copper, not '
                 'by the silicon it is watching',
                 fastest < 60.0, '%.1f K/s at its steepest' % fastest)

    # Substantially slower than the SOA acts.
    leg_rose = model.at('driver_v') - AMBIENT
    report.check('the reading trails the silicon it sits beside by a wide '
                 'margin over the same second',
                 leg_rose > 20.0 * rose,
                 'leg +%.1f K, reading +%.1f K' % (leg_rose, rose))

    # The node it watches is free to slew; only the reading is not.
    report.check('the driver node itself is not slowed by it',
                 model.at('driver_u') - AMBIENT > rose,
                 'driver +%.1f K against the reading +%.1f K'
                 % (model.at('driver_u') - AMBIENT, rose))

    for _ in range(20000):
        model.step(watt, 1.0)
    # The element's own steady state: a weighted average of the two nodes it
    # is tied to, with no additive offset.
    board = model.at('board')
    leg = model.at('patch_v')
    target = board + NTC_SEES_LEG * (leg - board)
    report.check('given time it lands on the weighted average of the two '
                 'patches it is tied to',
                 abs(model.ntc() - target) < 0.1,
                 '%.2f C against %.2f' % (model.ntc(), target))
    report.check('and it is between them, which no weight can break',
                 board - 1e-6 <= model.ntc() <= leg + 1e-6,
                 'board %.1f, ntc %.1f, leg %.1f'
                 % (board, model.ntc(), leg))


#: Silva 2022 (Appl. Sci. 12, 12555), Eq. 12-14: a lumped element's
#: effective transient capacity is `gamma C`, gamma = 1/3 less a negative
#: term per contact with a better conductor, because heat crosses a
#: distributed body in one direction.
GAMMA = 1.0 / 3.0


def test_the_reading_lags_between_the_two_nodes(report, lib):
    """Its constant is the geometric mean of the pair it sits between."""
    model = Model(lib)
    # Off the model, not off a number typed here: the pair is the V leg's patch
    # and the centre, and their constants are their capacities across the paths
    # `thermal_defaults` quotes for them - 15 K/W from the leg's patch to the
    # rest of the board, 48 from the centre.
    leg = model.capacity('patch_v') * 15.0
    board = model.capacity('board') * 48.0

    # The two nodes held, so the target does not move while the reading walks
    # toward it.
    hot, cold = 120.0, 40.0
    target = cold + NTC_SEES_LEG * (hot - cold)
    zero = {}
    start = None
    for step in range(400):
        model.place('patch_v', hot)
        model.place('board', cold)
        if start is None:
            model.step(zero, 1e-4)
            start = model.ntc()
        model.step(zero, 0.05)
        if (step + 1) * 0.05 >= 1.0:
            break
    if start is None:
        raise AssertionError('the model never stepped')
    share = (model.ntc() - start) / max(1e-9, target - start)
    tau = -1.0 / math.log(max(1e-9, 1.0 - min(0.999999, share)))

    report.check('the reading lags past the patch it watches',
                 tau > leg, '%.1f s against the leg patch %.1f s'
                 % (tau, leg))
    report.check('and short of the centre, which is the other end of what '
                 'it sits between',
                 tau < board, '%.1f s against the centre %.0f s'
                 % (tau, board))
    report.check('the geometric mean of the two, near enough',
                 abs(tau / math.sqrt(leg * board) - 1.0) < 0.15,
                 '%.1f s against %.1f' % (tau, math.sqrt(leg * board)))


def test_the_thermistor_never_reads_above_its_source(report, lib):
    """The reading stays between the leg and the board - on the way down
    too.
    """
    dt = 0.05
    for amps, read_over in ((25.0, 5.96), (60.0, 28.8)):
        watt = power(lib, phase_amps=(0.0, amps, 0.0), duty=(0.0, 0.5, 0.0),
                     link_volts=48.0, switching=True)
        model = Model(lib)
        lagged = False
        worst = -1e9
        for _ in range(int(120.0 / dt)):
            model.step(watt, dt)
            leg, board = model.at('patch_v'), model.at('board')
            ntc = model.ntc()
            lagged = lagged or ntc < board + NTC_SEES_LEG * (leg - board) - 1.0
            worst = max(worst, ntc - max(leg, board))
        for _ in range(int(120.0 / dt)):
            model.step({}, dt)
            leg, board = model.at('patch_v'), model.at('board')
            ntc = model.ntc()
            worst = max(worst, ntc - max(leg, board), min(leg, board) - ntc)
        report.check('%.0f A for two minutes then off: the reading never '
                     'leaves the pair it sits between (it read %.1f K over '
                     'the leg)' % (amps, read_over),
                     worst <= 1e-3, '%+.3f K outside' % worst)
        report.check('and it still lags on the way up at %.0f A' % amps,
                     lagged)


def test_the_burst_budget_rests_on_an_unmeasured_capacity(report, lib):
    """What the leg capacity is worth, since nobody measured it."""
    watt = power(lib, phase_amps=(100.0, 0.0, 0.0), duty=(0.5, 0.0, 0.0),
                 link_volts=48.0, switching=True)
    seen = {}
    for name, scale in (('on record', 1.0), ('at gamma', GAMMA)):
        model = Model(lib)
        base = model.capacity('driver_u')
        report.check('the capacity moves when a bench moves it (%s)' % name,
                     model.set_node('driver_u', DRIVER_TO_PATCH, base * scale),
                     '%.4f J/K' % (base * scale))
        got = model.budget(watt, lookahead_s=LOOKAHEAD_S)
        # Seconds from ambient to the ceiling at this power, which is what a
        # burst is spending.
        seen[name] = (got['soak_j']['driver_u'],
                      got['soak_j']['driver_u'] / watt['driver_u'])

    report.check('the soak scales with it exactly - it IS the capacity '
                 'times the rise left',
                 abs(seen['at gamma'][0] / seen['on record'][0] - GAMMA)
                 < 1e-3,
                 '%.2f J against %.2f J' % (seen['at gamma'][0],
                                            seen['on record'][0]))
    report.check('and so does the burst, one for one',
                 abs(seen['at gamma'][1] / seen['on record'][1] - GAMMA)
                 < 1e-3,
                 '%.2f s against %.2f s at 100 A'
                 % (seen['at gamma'][1], seen['on record'][1]))

    # The band, as a number a bench can act on.
    report.check('so the 100 A burst budget is a band, not a figure',
                 seen['at gamma'][1] < seen['on record'][1],
                 'between %.2f s and %.2f s on the driver node'
                 % (seen['at gamma'][1], seen['on record'][1]))

    first = {}
    for name, scale in (('on record', 1.0), ('at gamma', GAMMA)):
        model = Model(lib)
        model.set_node('driver_u', DRIVER_TO_PATCH,
                       model.capacity('driver_u') * scale)
        for step in range(4000):
            model.step(watt, 0.02)
            if model.budget(watt, lookahead_s=LOOKAHEAD_S)['derate'] < 0.999:
                first[name] = step * 0.02
                break
    report.check('the throttle acts sooner on a lighter node, by the same '
                 'factor the capacity moved',
                 'at gamma' in first and first['at gamma'] < first['on record'],
                 '%.2f s against %.2f s' % (first.get('at gamma', -1),
                                            first.get('on record', -1)))


def test_it_refuses_nothing_and_returns_no_codes(report, lib):
    """No limit set is not an error, it is a node nobody constrained."""
    model = Model(lib)
    model.place('phase_u', 90.0)
    none = model.budget(limits=[0.0] * len(NODES))
    report.check('with no ceilings at all nothing is spent and nothing trips',
                 none['worst'] == 0.0 and not none['tripped'],
                 '%.3f spent' % none['worst'])
    report.check('and the clamp stays open',
                 none['derate'] == 1.0, '%.3f' % none['derate'])

    # A ceiling below ambient is the same case, and the one that would
    # otherwise divide by a negative span.
    model.place('phase_u', 90.0)
    below = model.budget(limits=[AMBIENT - 5.0] * len(NODES))
    report.check('a ceiling below ambient is skipped, not divided by',
                 below['worst'] == 0.0 and below['derate'] == 1.0,
                 '%.3f spent, clamp %.3f' % (below['worst'], below['derate']))

    # A throttle point of 1.0 leaves no band.
    model.place('phase_u', 120.0)
    edge = model.budget(throttle_at=1.0)
    report.check('a throttle point at the ceiling leaves no band to ramp in, '
                 'and the clamp stays open rather than dividing by zero',
                 edge['derate'] == 1.0, '%.3f' % edge['derate'])


def test_the_time_left_is_reported_or_not_claimed(report, lib):
    """Milliseconds while it is heading somewhere, -1 while it is not."""
    model = Model(lib)
    model.place('phase_u', 60.0)
    idle = model.budget()
    report.check('a node with no power on it claims no time',
                 idle['millis'] < 0.0, '%.0f ms' % idle['millis'])

    hot = model.budget({'phase_u': 35.0})
    report.check('and with 35 W on it, a time in milliseconds',
                 hot['millis'] > 0.0, '%.0f ms' % hot['millis'])
    report.check('which is the capacity over the net power, near enough',
                 abs(hot['millis'] / 1000.0
                     - model.capacity('phase_u') * (LIMIT_C - 60.0) / 35.0)
                 < 1.0,
                 '%.2f s' % (hot['millis'] / 1000.0))


def test_a_ceiling_pulled_in_under_a_node_does_not_trip(report, lib):
    """The trip is judged on the record's ceiling; the throttle on the
    trimmed one.
    """
    model = Model(lib)
    model.place('driver_u', AMBIENT + 0.92 * (LIMIT_C - AMBIENT))
    whole = model.budget()
    report.check('at 92 % of the record\'s span a driver throttles and does '
                 'not trip',
                 whole['throttling'] and 0.0 < whole['derate'] < 1.0
                 and not whole['tripped'],
                 'clamp %.3f, tripped %s' % (whole['derate'], whole['tripped']))
    trimmed = [AMBIENT + 0.82 * (top - AMBIENT) for top in model.limits()]
    pulled = model.budget(limits=trimmed, trip=model.limits())
    report.check('every ceiling pulled in to 82 % of its span: the node '
                 'reads 100 %, the clamp is closed, and it is NOT tripped',
                 pulled['used']['driver_u'] >= 0.999 and pulled['derate'] == 0.0
                 and not pulled['tripped'],
                 'used %.3f, clamp %.3f, tripped %s'
                 % (pulled['used']['driver_u'], pulled['derate'],
                    pulled['tripped']))
    before = model.budget(limits=trimmed)
    report.check('a caller giving no trip ceilings is judged on the trimmed '
                 'ones, as every caller before', before['tripped'],
                 before['tripped'])
    model.place('driver_u', LIMIT_C + 0.5)
    at = model.budget(limits=trimmed, trip=model.limits())
    report.check('and at the record\'s ceiling it trips, the clamp closed',
                 at['tripped'] and at['derate'] == 0.0,
                 'tripped %s, clamp %.3f' % (at['tripped'], at['derate']))


def test_the_winding_is_an_envelope_of_its_own(report, lib):
    """The motor's copper, a node of the graph: judged like a node, by the
    same ramp, shedding through the iron and the bell to the air and not
    into the laminate.
    """
    # `3 i_rms^2 R` off the mean squares when they are there ...
    got = power(lib, phase_sq=(100.0, 100.0, 100.0), r_phase=0.05,
                switching=False)
    report.check('10 A rms on every phase through 50 mOhm is 15 W of copper',
                 abs(got['winding'] - 15.0) < 1e-3, '%.3f W' % got['winding'])
    # ... and the instantaneous sample squared when they are not.
    got = power(lib, phase_amps=(10.0, 0.0, 0.0), r_phase=0.05,
                switching=False)
    report.check('and with no mean squares the sample is squared instead',
                 abs(got['winding'] - 5.0) < 1e-3, '%.3f W' % got['winding'])

    # The steady state is the graph's: the copper into the iron, the iron to
    # the air directly and through the bell, in parallel - read off the model's
    # own edges, not typed here.
    model = Model(lib)
    r_ws = model.edge_r(EDGE_WINDING_STATOR)
    r_sr = model.edge_r(EDGE_STATOR_ROTOR)
    r_sa = model.to_ambient_at('stator', 0.0)
    r_ra = model.to_ambient_at('rotor', 0.0)
    expect = 15.0 * (r_ws + 1.0 / (1.0 / r_sa + 1.0 / (r_sr + r_ra)))
    for _ in range(6000):
        model.step({'winding': 15.0}, 1.0)
    rose = model.at('winding') - AMBIENT
    report.check('15 W settles the winding where its paths to the air say',
                 abs(rose - expect) < 0.5,
                 '%.1f K against %.1f' % (rose, expect))
    report.check('and the heat runs copper to iron to bell to air',
                 model.at('winding') > model.at('stator') > model.at('rotor')
                 > AMBIENT + 0.1,
                 '%.1f > %.1f > %.1f' % (model.at('winding'),
                                         model.at('stator'),
                                         model.at('rotor')))
    report.check('with the mount open, none of it reaches the board',
                 all(abs(model.at(n) - AMBIENT) < 1e-3 for n in LAMINATE),
                 str([round(model.at(n) - AMBIENT, 3) for n in LAMINATE]))

    # The same ramp as a board node's: a node and the winding the same fraction
    # up their own scales get the same factor - one definition.
    node = Model(lib)
    node.place('phase_u', AMBIENT + 0.947 * (LIMIT_C - AMBIENT))
    b = node.budget()
    spend = b['used']['phase_u']
    hot = Model(lib)
    hot.place('winding', AMBIENT + spend * (WINDING_LIMIT_C - AMBIENT))
    a = hot.budget()
    report.check('at %.3f of its ceiling the winding is the worst node and '
                 'throttles' % spend,
                 a['worst_node'] == 'winding' and a['throttling']
                 and 0.0 < a['derate'] < 1.0 and not a['tripped'],
                 '%s, clamp %.3f' % (a['worst_node'], a['derate']))
    report.check('by exactly the factor a node at %.3f of its own gets'
                 % spend, abs(a['derate'] - b['derate']) < 2e-3,
                 'winding %.3f, node %.3f' % (a['derate'], b['derate']))
    report.check('and its OWN factor is what the wire reports beside the '
                 'whole',
                 abs(hot.node_derate('winding') - a['derate']) < 1e-6,
                 '%.3f' % hot.node_derate('winding'))

    cooked = Model(lib)
    cooked.place('winding', WINDING_LIMIT_C + 1.0)
    got = cooked.budget()
    report.check('past its ceiling it trips and the clamp is closed',
                 got['tripped'] and got['derate'] == 0.0,
                 'clamp %.3f' % got['derate'])

    # The hold, not the temperature: a cold winding of one joule per kelvin
    # with a hundred watts on it has under a second to its ceiling.
    thin = Model(lib)
    thin.set_node('winding', r_ws, 1.0)
    soon = thin.budget({'winding': 100.0}, lookahead_s=20.0)
    now = thin.budget({'winding': 100.0}, lookahead_s=0.0)
    report.check('a cold winding whose hold has fallen into the window '
                 'is throttled on the hold',
                 soon['derate'] < 1.0 and now['derate'] == 1.0,
                 'with lookahead %.3f, without %.3f'
                 % (soon['derate'], now['derate']))

    off = Model(lib)
    off.place('winding', 200.0)
    limits = off.limits()
    limits[NODES.index('winding')] = 0.0
    got = off.budget(limits=limits)
    report.check('a zero ceiling disables the winding: nothing spent, '
                 'nothing tripped, the clamp open',
                 got['used']['winding'] == 0.0 and not got['tripped']
                 and got['derate'] == 1.0,
                 '%.3f spent, clamp %.3f' % (got['used']['winding'],
                                             got['derate']))


def test_the_laminate_is_a_graph_that_reproduces_the_bulk(report, lib):
    """Seven patches whose capacities and air paths sum to the one board the
    camera measured, joined by the copper's own conductances - and a leg
    that warms its neighbour.
    """
    model = Model(lib)
    cap = sum(model.capacity(n) for n in LAMINATE)
    report.check('the patches\' capacities sum to the measured 49 J/K',
                 abs(cap - 49.0) < 0.05, '%.2f J/K' % cap)
    g_air = sum(1.0 / model.to_ambient_at(n, 5.0) for n in LAMINATE)
    report.check('and their air paths in parallel to the measured 8.33 K/W',
                 abs(1.0 / g_air - 8.33) < 0.1, '%.2f K/W' % (1.0 / g_air))
    report.check('each patch\'s path is the bulk\'s over its share of the '
                 'face',
                 abs(model.to_ambient_at('board', 5.0) * 0.199 - 8.33) < 0.05,
                 '%.1f K/W x 0.199' % model.to_ambient_at('board', 5.0))

    joins = edges(lib)
    g = sum(1.0 / model.edge_r(e) for e, (a, b) in enumerate(joins)
            if 'patch_v' in (a, b) and {a, b} <= set(LAMINATE))
    report.check('the V patch\'s neighbours in parallel are the camera\'s '
                 'lumped bridge-to-board', abs(1.0 / g - 15.2) < 0.5,
                 '%.1f K/W' % (1.0 / g))
    report.check('and a leg\'s switches into that patch make the record\'s '
                 '28 a leg',
                 abs(model.edge_r(0) + 1.0 / g - 28.0) < 1.5,
                 '%.1f K/W' % (model.edge_r(0) + 1.0 / g))

    for _ in range(20000):
        model.step({'driver_u': 20.0}, 1.0)
    report.check('20 W on U warms U\'s patch most, V\'s next, W\'s least',
                 model.at('patch_u') > model.at('patch_v')
                 > model.at('patch_w') > AMBIENT + 1.0,
                 'U %.1f, V %.1f, W %.1f' % (model.at('patch_u'),
                                             model.at('patch_v'),
                                             model.at('patch_w')))
    report.check('and the corner beside U more than the corner beside W',
                 model.at('patch_left') > model.at('patch_right') + 0.5,
                 'left %.1f, right %.1f' % (model.at('patch_left'),
                                            model.at('patch_right')))
    lost = sum((model.at(n) - AMBIENT)
               / model.to_ambient_at(n, model.at(n) - AMBIENT)
               for n in LAMINATE)
    report.check('settled, the face loses to the air what the leg makes',
                 abs(lost - 20.0) < 0.1, '%.2f W' % lost)


def test_the_switching_loss_follows_the_coss_law(report, lib):
    """The no-load switching scales as the stored C_oss energy, not
    linearly; with current, the overlap, the body diode and the gate
    charge join it, each with a datasheet behind it.
    """
    loss = losses(lib)
    e_cal = lib.thm_coss_energy(loss['switch_volts'])
    e_63 = lib.thm_coss_energy(63.0)
    m, vj = loss['coss_m'], loss['coss_vj']

    def law(v):
        u = 1.0 + v / vj
        return ((u ** (2.0 - m) - 1.0) / (2.0 - m)
                - (u ** (1.0 - m) - 1.0) / (1.0 - m))

    report.check('the C_oss energy is the closed form of the model\'s law',
                 abs(e_63 / e_cal - law(63.0) / law(loss['switch_volts']))
                 < 1e-3,
                 '%.3f against %.3f'
                 % (e_63 / e_cal, law(63.0) / law(loss['switch_volts'])))
    report.check('and at 63 V it is over four times the 24.6 V figure, '
                 'where a line gave 2.6',
                 e_63 / e_cal > 4.0 and e_63 / e_cal > 63.0 / 24.6,
                 '%.2fx' % (e_63 / e_cal))

    gate = 2.0 * loss['q_g'] * loss['v_drive'] * loss['f_sw']
    drivers = ('driver_u', 'driver_v', 'driver_w')
    at_cal = power(lib, duty=(0.5, 0.5, 0.5),
                   link_volts=loss['switch_volts'], switching=True)
    at_63 = power(lib, duty=(0.5, 0.5, 0.5), link_volts=63.0,
                  switching=True)
    sw_cal = sum(at_cal[n] for n in drivers) - 3.0 * gate
    sw_63 = sum(at_63[n] for n in drivers) - 3.0 * gate
    report.check('no load at the calibration link: the measured 1.2 W, its '
                 'driver share on the switches',
                 abs(sw_cal - loss['switching_watt'] * loss['driver_share'])
                 < 1e-3, '%.3f W' % sw_cal)
    report.check('no load at 63 V: the same times the C_oss ratio',
                 abs(sw_63 / sw_cal - e_63 / e_cal) < 1e-3,
                 '%.2fx' % (sw_63 / sw_cal))

    # With current: 100 A rms on U at 48 V, dead time 30 ns.
    sq = 100.0 ** 2
    t_dead = 30e-9
    loaded = power(lib, phase_sq=(sq, 0.0, 0.0), duty=(0.5, 0.0, 0.0),
                   link_volts=48.0, switching=True, t_dead_s=t_dead)
    still = power(lib, phase_sq=(sq, 0.0, 0.0), duty=(0.5, 0.0, 0.0),
                  link_volts=48.0, switching=False, t_dead_s=t_dead)
    e_48 = lib.thm_coss_energy(48.0)
    noload = ((loss['switching_watt'] / 3.0) * (e_48 / e_cal)
              * loss['driver_share'])
    overlap = 48.0 * 100.0 * loss['t_switch_s'] * loss['f_sw']
    diode = 2.0 * loss['v_sd'] * 0.9 * 100.0 * t_dead * loss['f_sw']
    extra = loaded['driver_u'] - still['driver_u']
    report.check('switching 100 A adds the C_oss dump, the overlap, the body '
                 'diode across the dead time and the gate charge',
                 abs(extra - (noload + overlap + diode + gate)) < 0.02,
                 '%.3f W against %.3f'
                 % (extra, noload + overlap + diode + gate))
    report.check('the overlap is the biggest of them at this current',
                 overlap > noload and overlap > diode and overlap > gate,
                 'overlap %.2f, C_oss %.2f, diode %.2f, gate %.2f'
                 % (overlap, noload, diode, gate))
    report.check('and none of it without switching - conduction only',
                 abs(still['driver_u'] - sq * loss['rds_on']) < 1e-3,
                 '%.3f W' % still['driver_u'])
    report.check('the buck pays the gate charge\'s conversion loss on the '
                 'regulators',
                 abs((loaded['regulators'] - still['regulators'])
                     - ((loss['switching_watt'] / 3.0) * (e_48 / e_cal)
                        * (1.0 - loss['driver_share'])
                        + gate * (1.0 / loss['buck_eff'] - 1.0))) < 1e-3,
                 '%.3f W' % (loaded['regulators'] - still['regulators']))


def test_the_junction_rides_the_node(report, lib):
    """A die is its node plus its own power through R_th - not a constant."""
    model = Model(lib)
    model.place('driver_u', 100.0)
    model.place('mcu', 45.0)
    watt = {'driver_u': 18.0, 'mcu': 0.666}
    report.check('a FET\'s junction is its node plus half the leg\'s watts '
                 'through R_th,JC',
                 abs(model.junction(watt, 'driver_u') - (100.0 + 9.0 * 0.69))
                 < 1e-3, '%.2f C' % model.junction(watt, 'driver_u'))
    report.check('the MCU\'s die at its static watts is the campaign\'s 27 K '
                 'over its package',
                 abs(model.junction(watt, 'mcu') - 45.0 - 27.0) < 0.05,
                 '%.2f C' % model.junction(watt, 'mcu'))
    report.check('and no power, no rise',
                 model.junction({}, 'driver_u') == 100.0)

    # The anchor takes it off: a die seen at 72 C with 0.666 W in it is a
    # package at 45, and the patch under it 0.666 x 22.5 lower again.
    seen = Model(lib)
    for _ in range(3000):
        seen.step({'mcu': 0.666}, 1.0, (math.nan, math.nan, 72.0))
    report.check('a die read at 72 C anchors its node about 27 K under it',
                 abs((72.0 - seen.at('mcu')) - 27.0) < 3.0,
                 'node %.1f C' % seen.at('mcu'))
    report.check('and the patch under it lower by its watts through its edge',
                 seen.at('mcu') > seen.at('board') + 10.0,
                 'node %.1f, centre %.1f' % (seen.at('mcu'), seen.at('board')))


def test_the_motor_is_the_boards_boundary(report, lib):
    """The rotor's air, the mount and the faces: what a bench does not have
    and a motor does.
    """
    model = Model(lib)
    report.check('the bell in still air is its still-air figure',
                 abs(model.to_ambient_at('rotor', 5.0, 0.0) - 4.0) < 1e-3,
                 '%.2f K/W' % model.to_ambient_at('rotor', 5.0, 0.0))
    report.check('and at 4000 rpm a third of it - one unit per sqrt(krpm)',
                 abs(model.to_ambient_at('rotor', 5.0, 4000.0) - 4.0 / 3.0)
                 < 1e-3,
                 '%.2f K/W' % model.to_ambient_at('rotor', 5.0, 4000.0))
    report.check('a patch behind the rotor gets a third of that improvement',
                 abs(model.to_ambient_at('board', 5.0, 4000.0)
                     / model.to_ambient_at('board', 5.0, 0.0) - 1.0 / 1.6)
                 < 1e-3)

    rest, spun = Model(lib), Model(lib)
    for _ in range(6000):
        rest.step({'winding': 15.0}, 1.0)
        spun.step({'winding': 15.0}, 1.0, speed_rpm=4000.0)
    report.check('15 W in the copper settles lower at 4000 rpm than at rest',
                 spun.at('winding') < rest.at('winding') - 5.0,
                 '%.1f C against %.1f' % (spun.at('winding'),
                                          rest.at('winding')))

    # Mounted: six standoffs at 30 K/W each and the faces at 0.034 W/K.
    mounted = Model(lib)
    for m in range(6):
        mounted.set_edge(EDGE_MOUNT_FIRST + m, 30.0)
    mounted.radiate_to_stator(0.034)
    mounted.place('stator', 100.0)
    mounted.set_node('stator', 1e6, 1e9)       # held hot: a motor running
    for _ in range(600):
        mounted.step({}, 1.0)
    report.check('mounted, a hot stator warms the rim patches through the '
                 'standoffs and the faces',
                 all(mounted.at(n) > AMBIENT + 2.0
                     for n in ('patch_u', 'patch_v', 'patch_w', 'patch_left',
                               'patch_bottom', 'patch_right')),
                 str([round(mounted.at(n) - AMBIENT, 1) for n in LAMINATE]))
    report.check('and the centre through them',
                 mounted.at('board') > AMBIENT + 1.0,
                 '%.1f C' % mounted.at('board'))


def test_a_long_step_is_sub_stepped(report, lib):
    """An explicit step longer than a node's own constant oscillates; the
    core slices any gap into quarter seconds, so a stalled main loop
    lands where the fine integration does.
    """
    fine, coarse = Model(lib), Model(lib)
    watt = {'driver_u': 18.0}
    for _ in range(100):
        fine.step(watt, 0.1)
    for _ in range(5):
        coarse.step(watt, 2.0)
    report.check('ten seconds at 2 s steps lands where ten seconds at 0.1 s '
                 'does, on a node whose constant is under two seconds',
                 abs(fine.at('driver_u') - coarse.at('driver_u')) < 3.0
                 and math.isfinite(coarse.at('driver_u')),
                 '%.1f C against %.1f' % (coarse.at('driver_u'),
                                          fine.at('driver_u')))


ROSTER = (test_the_derate_is_a_ramp, test_derating_is_not_tripping,
          test_a_ceiling_pulled_in_under_a_node_does_not_trip,
          test_the_winding_is_an_envelope_of_its_own,
          test_the_laminate_is_a_graph_that_reproduces_the_bulk,
          test_the_switching_loss_follows_the_coss_law,
          test_the_junction_rides_the_node,
          test_the_motor_is_the_boards_boundary,
          test_a_long_step_is_sub_stepped,
          test_the_scales_are_identified_against_a_ground_truth,
          test_an_idle_board_stays_uncertain,
          test_the_room_is_identified,
          test_the_mirror_carries_the_cs_numbers,
          test_the_mirror_walks_with_the_c,
          test_the_lookahead_catches_a_ramp,
          test_the_step_must_land_inside_the_ramp, test_the_soak_is_joules,
          test_the_worst_node_is_the_one_acted_on,
          test_a_throttle_weighs_only_what_a_clamp_can_cool,
          test_the_conduction_is_split_where_it_is_made,
          test_conduction_is_a_mean_square_not_a_sample,
          test_the_thermistor_has_mass,
          test_the_reading_lags_between_the_two_nodes,
          test_the_thermistor_never_reads_above_its_source,
          test_the_burst_budget_rests_on_an_unmeasured_capacity,
          test_it_refuses_nothing_and_returns_no_codes,
          test_the_time_left_is_reported_or_not_claimed)


def main():
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    lib_path, warnings = build(cc, SOURCES, [os.path.join(THERMAL, 'inc')],
                               name='thermalcore')
    lib = ctypes.CDLL(lib_path)
    report = Report()
    report.check('thermal/ builds warning-free with the firmware flags',
                 not warnings, '; '.join(warnings[:3]))
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, lib)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
