"""The thermal envelope, run as the C that will run on the board.

`thermal/` is hardware-free like the Modbus core, and the part of it that
matters most is the part nothing was checking: the SOA envelope. `check.c`
beside the source is the calibration campaign's own report - does the
network reproduce the four camera-measured states - and it predates the
envelope entirely. The derate ramp, the lookahead and the soak joules had
a tested Python mirror in `coaxial.thermal` and `coaxial.simulated.power`,
and no test at all on the C that decides whether a real stage backs off.

That is the wrong way round. The Python is a stand-in; this is the thing
that gates the gates. So it is built here with the host gcc and driven
through ctypes, the same way the Modbus, SHTP, drive and filter cores are.

WHAT IT DOES NOT DO IS JUDGE A BOARD. Every ceiling below is stated by the
test, because the ceilings live in the calibration record and there is no
compiled-in copy to ask for - invariant 10, and the reason `thermal.h`
carries no `thermal_soa_defaults`. What is checked is the arithmetic on
whatever limits it was given.

    cd host && python tests/test_thermal_core.py
"""
import ctypes
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

from test_modbus_core import Report, build, find_cc          # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
THERMAL = os.path.join(REPO, 'thermal')
SOURCES = [os.path.join(THERMAL, 'test', 'harness.c'),
           os.path.join(THERMAL, 'src', 'thermal.c')]

#: The nodes, in the order `thermal.h` declares them. Named here so a
#: failure says `phase_u` and not `3`; the count is asked of the C.
NODES = ('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v', 'phase_w',
         'mcu', 'regulators', 'afe', 'board')

#: BUDGET_ORDER, as `harness.c` flattens it.
BUDGET = ('worst', 'worst_node', 'millis', 'throttling', 'tripped', 'derate')

#: The envelope this tree's calibration record carries, stated rather than
#: read: 125 C on every node but the board's copper, 105 there, throttling
#: from 85 % of the span and looking two seconds ahead. A bench with a
#: different board writes different numbers into its own record and this
#: test would still be testing the same arithmetic.
LIMIT_C, BOARD_LIMIT_C = 125.0, 105.0
THROTTLE_AT, LOOKAHEAD_S = 0.85, 2.0

AMBIENT = 20.0


class Model:

    """One observer behind the harness, standing where a test puts it."""

    def __init__(self, lib, celsius=AMBIENT):
        lib.thm_new.restype = ctypes.c_void_p
        lib.thm_new.argtypes = [ctypes.c_float]
        lib.thm_at.restype = ctypes.c_float
        lib.thm_at.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.thm_capacity.restype = ctypes.c_float
        lib.thm_capacity.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self.lib = lib
        self.n = lib.thm_nodes()
        self.slots = lib.thm_budget_slots()
        self.h = ctypes.c_void_p(lib.thm_new(ctypes.c_float(celsius)))

    def _floats(self, values):
        return (ctypes.c_float * len(values))(*values)

    def place(self, node, celsius):
        self.lib.thm_place(self.h, ctypes.c_int(NODES.index(node)),
                           ctypes.c_float(celsius))

    def at(self, node):
        return self.lib.thm_at(self.h, NODES.index(node))

    def capacity(self, node):
        return self.lib.thm_capacity(self.h, NODES.index(node))

    def step(self, watt, dt_s, seen=(math.nan, math.nan, math.nan)):
        self.lib.thm_step(self.h, self._floats(self._watt(watt)),
                          ctypes.c_float(seen[0]), ctypes.c_float(seen[1]),
                          ctypes.c_float(seen[2]), ctypes.c_float(dt_s))

    def _watt(self, watt):
        return [float(watt.get(name, 0.0)) for name in NODES]

    def limits(self):
        return [BOARD_LIMIT_C if name == 'board' else LIMIT_C
                for name in NODES]

    def budget(self, watt=None, throttle_at=THROTTLE_AT, lookahead_s=0.0,
               limits=None):
        out = (ctypes.c_float * self.slots)()
        self.lib.thm_budget(self.h, self._floats(self._watt(watt or {})),
                            self._floats(limits or self.limits()),
                            ctypes.c_float(throttle_at),
                            ctypes.c_float(lookahead_s), out)
        got = dict(zip(BUDGET, list(out)[:len(BUDGET)]))
        got['worst_node'] = NODES[int(got['worst_node'])]
        got['throttling'] = bool(got['throttling'])
        got['tripped'] = bool(got['tripped'])
        got['used'] = dict(zip(NODES, list(out)[6:6 + self.n]))
        got['soak_j'] = dict(zip(NODES, list(out)[6 + self.n:]))
        return got


def power(lib, phase_amps=(0.0, 0.0, 0.0), duty=(0.0, 0.0, 0.0),
          link_volts=24.0, link_amps=-1.0, switching=True, afe_on=False,
          phase_c=None):
    """The estimator's answer, per node, watts."""
    load = list(phase_amps) + list(duty) + [link_volts, link_amps,
                                           1.0 if switching else 0.0,
                                           1.0 if afe_on else 0.0]
    out = (ctypes.c_float * len(NODES))()
    temps = None
    if phase_c is not None:
        temps = (ctypes.c_float * 3)(*phase_c)
    lib.thm_power((ctypes.c_float * len(load))(*load), temps, out)
    return dict(zip(NODES, list(out)))


def losses(lib):
    names = ('rds_on', 'rds_alpha', 'r_shunt', 'r_hotswap', 'switching_watt',
             'switch_volts', 'driver_share', 'mcu_watt', 'ldo_watt',
             'afe_watt')
    out = (ctypes.c_float * len(names))()
    lib.thm_losses(out)
    return dict(zip(names, list(out)))


def wanted(spent, throttle_at=THROTTLE_AT):
    """The derate the ramp's own definition gives for a spend."""
    if spent <= throttle_at:
        return 1.0
    over = (spent - throttle_at) / (1.0 - throttle_at)
    return 0.0 if over >= 1.0 else 1.0 - over


def test_the_derate_is_a_ramp(report, lib):
    """One below the throttle point, zero at the ceiling, linear between.

    THE WHOLE POINT OF IT, and the reason `tripped` alone was not enough:
    a stage that runs at full current until the ceiling and then stops is
    a cliff. Walked across the band a degree at a time rather than checked
    at three places, because a ramp that is right at its ends and wrong in
    the middle is the shape that would go unnoticed.
    """
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
    """Inside the band the stage is still driving.

    The distinction the bench asked for in as many words: do not kill the
    drive, throttle it down in time. A budget that reported `tripped` the
    moment it reported `throttling` would have thrown that away.
    """
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
    """Derating on where a node is going, not where it is.

    THE DEFECT THIS EXISTS FOR, measured on the stand-in: a phase node at
    45 A crossed from a fifth of its budget to over the ceiling inside
    three polls, so the whole 85-to-100 band went past between two looks
    and the derate never left 1.0. Reproduced here in the C.

    AT A POWER THAT DOES NOT SATURATE THE PROJECTION. Written first with
    35 W - the 100 A conduction - both budgets were compared and the
    lookahead one closed the clamp on its FIRST step, from 24 C. It was
    not wrong, it was saturated: 35 W into a 0.40 J/K node is 87 K/s, so
    two seconds ahead is 175 K and over any ceiling from anywhere. The
    check passed and measured nothing. 8 W, about 48 A a leg, is a ramp
    steep enough to outrun a poll and shallow enough that WHEN the
    projection fires is a fact about the projection.
    """
    watt = {'phase_u': 8.0}
    first = {}
    for name, ahead in (('now', 0.0), ('soon', LOOKAHEAD_S)):
        model = Model(lib)
        for step in range(4000):
            model.step(watt, 0.05)
            got = model.budget(watt, lookahead_s=ahead)
            if got['derate'] < 0.999:
                first[name] = (step * 0.05, model.at('phase_u'), got['worst'])
                break

    report.check('both eventually back off - a node heading there must',
                 'now' in first and 'soon' in first, str(first))
    if len(first) == 2:
        report.check('the lookahead backs off EARLIER than the present does',
                     first['soon'][0] < first['now'][0],
                     '%.2f s ahead against %.2f s' % (first['soon'][0],
                                                      first['now'][0]))
        report.check('and does it with the node still cooler',
                     first['soon'][1] < first['now'][1],
                     '%.1f C against %.1f C' % (first['soon'][1],
                                                first['now'][1]))
        report.check('but not from cold - at this power it is a projection, '
                     'not a refusal to run',
                     first['soon'][0] > 0.0 and first['soon'][2] > 0.2,
                     '%.2f s in, %.0f %% spent' % (first['soon'][0],
                                                   100.0 * first['soon'][2]))
        report.check('the present-only one waits for the throttle point '
                     'itself, which is what arrives too late',
                     first['now'][2] >= THROTTLE_AT - 0.01,
                     '%.3f spent' % first['now'][2])

    # AND AT THE FULL 100 A CONDUCTION IT CLOSES FROM COLD, which is the
    # envelope's own arithmetic and not a bug: 35 W into 0.40 J/K crosses
    # the ceiling inside two seconds from ambient, so a stage asking for
    # that current is asking for something the node cannot hold. The clamp
    # scales the current, which lowers the power, which lowers the
    # projection - it settles where two seconds ahead lands on the
    # ceiling, and `board_thermal.c` slews the recovery so it cannot
    # chatter there. Written down because a bench reading `derate 0.00` at
    # 100 A should find it recorded rather than investigate it.
    cold = Model(lib)
    deep = cold.budget({'phase_u': 35.0}, lookahead_s=LOOKAHEAD_S)
    report.check('35 W from ambient closes the clamp on the projection '
                 'alone - the node cannot hold 100 A for the lookahead',
                 deep['derate'] == 0.0 and not deep['tripped'],
                 'clamp %.2f at %.1f C, %.0f %% spent'
                 % (deep['derate'], cold.at('phase_u'), 100.0 * deep['worst']))

    # ZERO DISABLES IT, bit for bit: the pre-lookahead behaviour has to
    # remain reachable, because a record that never had the field reads
    # back as zero and must still get the old envelope.
    model = Model(lib)
    # AT 85 C, NOT 60: two seconds of 8 W is about 36 K on this node, so
    # from 60 the projection lands at 96 - short of the throttle point,
    # both budgets answer 1.0, and the check compared two untouched
    # clamps. From 85 the projection crosses and the present does not,
    # which is the only place the two can be told apart.
    model.place('phase_u', 85.0)
    report.check('a cooling node is not projected anywhere warmer',
                 model.budget(lookahead_s=LOOKAHEAD_S)['derate'] == 1.0)
    hot = model.budget({'phase_u': 8.0}, lookahead_s=LOOKAHEAD_S)['derate']
    flat = model.budget({'phase_u': 8.0}, lookahead_s=0.0)['derate']
    report.check('and with power on it, the projection is what differs',
                 hot < flat, 'ahead %.3f against now %.3f' % (hot, flat))


def test_the_soak_is_joules(report, lib):
    """Capacity times the rise still available, per node.

    `used` is where a node is and `millis_to_limit` is how long AT THIS
    POWER; neither answers "how much work is left in it", which is what a
    control system asking for a burst wants. Joules do, and they divide by
    any planned power rather than only the present one.
    """
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

    # A NODE PAST ITS CEILING HAS NO BUDGET, not a debt: the trip is what
    # says it is over, and a negative joule count would divide into a
    # negative burst length.
    model.place('phase_u', LIMIT_C + 20.0)
    report.check('past the ceiling the soak is zero, never negative',
                 model.budget()['soak_j']['phase_u'] == 0.0)

    # The whole reason it is joules: seconds at a power nobody is at yet.
    model.place('phase_u', 45.0)
    joules = model.budget()['soak_j']['phase_u']
    report.check('and it divides into a burst length at any power',
                 abs(joules / 35.0
                     - model.capacity('phase_u') * 80.0 / 35.0) < 0.01,
                 '%.2f s at 35 W' % (joules / 35.0))


def test_the_worst_node_is_the_one_acted_on(report, lib):
    """A stage is as close to its ceiling as its closest part.

    An average would hide the one leg that is cooking behind five that are
    not, and the ceilings differ - the board's copper is 105 and the
    silicon 125 - so the hottest node and the worst node need not be the
    same one. That is not a defect; it is why both are reported.
    """
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


def test_the_conduction_is_split_where_it_is_made(report, lib):
    """The FET's watts on the driver node, the shunt's on the phase node.

    Booked entirely on the phase node the model said the shunt cooked
    while the FET beside it in the same current path stayed cold. They are
    two parts. Checked against `thermal_losses` rather than against a
    number typed twice: the split is a ratio of resistances the parts
    list gives.
    """
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

    # And the FET is the one whose resistance climbs with its own node.
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


def test_it_refuses_nothing_and_returns_no_codes(report, lib):
    """No limit set is not an error, it is a node nobody constrained.

    A ceiling of zero means the record did not carry one for that node.
    The budget skips it rather than reporting it at its ceiling, which is
    what a span of zero would arithmetically give.
    """
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

    # A throttle point of 1.0 leaves no band. The ramp has to survive it
    # rather than divide by the width it does not have.
    model.place('phase_u', 120.0)
    edge = model.budget(throttle_at=1.0)
    report.check('a throttle point at the ceiling leaves no band to ramp in, '
                 'and the clamp stays open rather than dividing by zero',
                 edge['derate'] == 1.0, '%.3f' % edge['derate'])


def test_the_time_left_is_reported_or_not_claimed(report, lib):
    """Milliseconds while it is heading somewhere, -1 while it is not.

    A large number where there is no answer reads like a promise. The
    board says -1 instead, and a host that plans a burst on it knows the
    difference.
    """
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


ROSTER = (test_the_derate_is_a_ramp, test_derating_is_not_tripping,
          test_the_lookahead_catches_a_ramp, test_the_soak_is_joules,
          test_the_worst_node_is_the_one_acted_on,
          test_the_conduction_is_split_where_it_is_made,
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
