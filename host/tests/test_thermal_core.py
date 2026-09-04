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

#: How far the thermistor's element sits toward the leg node, as
#: `thermal_defaults` sets it. Named here rather than read back, because a
#: test that asked the code for its own expectation would agree with a
#: typo - and it is `electronics/`'s pick and place that says what it
#: should be, which `test_sensorless.py` checks against the file.
NTC_SEES_LEG = 0.30


class Model:

    """One observer behind the harness, standing where a test puts it."""

    def __init__(self, lib, celsius=AMBIENT):
        lib.thm_new.restype = ctypes.c_void_p
        lib.thm_new.argtypes = [ctypes.c_float]
        lib.thm_at.restype = ctypes.c_float
        lib.thm_at.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.thm_capacity.restype = ctypes.c_float
        lib.thm_capacity.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.thm_ntc.restype = ctypes.c_float
        lib.thm_ntc.argtypes = [ctypes.c_void_p]
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

    def set_node(self, node, to_board, capacity):
        return bool(self.lib.thm_set_node(self.h,
                                          ctypes.c_int(NODES.index(node)),
                                          ctypes.c_float(to_board),
                                          ctypes.c_float(capacity)))

    def ntc(self):
        return self.lib.thm_ntc(self.h)

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
          phase_c=None, phase_sq=(0.0, 0.0, 0.0)):
    """The estimator's answer, per node, watts.

    `phase_sq` is the mean of the squared current a leg has carried since
    the last estimate. Zero means none was measured and the estimator
    squares the instantaneous sample instead.
    """
    load = list(phase_amps) + list(duty) + [link_volts, link_amps,
                                           1.0 if switching else 0.0,
                                           1.0 if afe_on else 0.0]         + list(phase_sq)
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
    """The throttle acts on time left, not on a projected temperature.

    THE DEFECT IT EXISTS FOR, measured on the stand-in: a phase node at
    45 A crossed from a fifth of its budget to over the ceiling inside
    three polls, so the whole 85-to-100 band went past between two looks
    and the derate never left 1.0.

    AND THE DEFECT ITS FIRST SHAPE HAD. It projected each node forward
    `lookahead_s` at its present rate and derated on where that landed,
    which fails on the case this board is for. 100 A puts 18.4 W into a
    driver node of 0.12 J/K - 0.67 s from ambient to its ceiling - and a
    two second projection lands past it from a COLD board: the clamp went
    to 0.00 before the burst began. Measured here 2026-09-03, and the
    reason the rule is now `hold / lookahead_s`.
    """
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

    # THE BURST RUNS. This is the whole difference from the projection,
    # and the reason it was changed: a node at ambient has its entire soak
    # in front of it however much power is on it, so full current is
    # allowed to start. What closes the clamp is the hold falling into the
    # window, not the size of the power.
    cold = Model(lib)
    for watts in (8.0, 35.0):
        got = cold.budget({'phase_u': watts}, lookahead_s=LOOKAHEAD_S)
        report.check('%.0f W from ambient leaves the clamp open - the burst '
                     'is shaped, not forbidden' % watts,
                     got['derate'] == 1.0,
                     'clamp %.2f, %.0f %% spent' % (got['derate'],
                                                    100.0 * got['worst']))

    # UP TO WHAT THE NODE CAN HOLD FOR THE REACTION WINDOW, and no
    # further. 35 W is the 100 A rating and the node holds it 1.2 s; 200 W
    # would be some 240 A, which it cannot hold for the 0.3 s the ramp
    # needs, so the clamp is not open even from ambient. That is the rule
    # doing its job rather than an exception to it - a power a part cannot
    # survive the reaction to is not a burst, it is a fault.
    fault = cold.budget({'phase_u': 200.0}, lookahead_s=LOOKAHEAD_S)
    report.check('a power past what the node can hold for the window is '
                 'throttled from cold, and that is the rule, not a hole in it',
                 0.0 < fault['derate'] < 1.0 and not fault['tripped'],
                 'clamp %.2f at ambient' % fault['derate'])

    # AND THE KNOB IS MONOTONE. It used to be fatal past the burst budget
    # - 2000 ms against a 670 ms node stopped the drive dead - so raising
    # it made the envelope qualitatively different rather than earlier. A
    # longer window now only means a longer, gentler ramp.
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

    # ONE DEFINITION: the throttle acts on the hold, and `millis_to_limit`
    # reports the hold. A board backing off on one number while a host
    # plans a burst on another would be two envelopes.
    model = Model(lib)
    model.place('phase_u', 100.0)
    got = model.budget(watt, lookahead_s=LOOKAHEAD_S)
    hold = got['millis'] / 1000.0
    report.check('the derate follows the same hold the board reports',
                 abs(got['derate'] - wanted(1.0 - hold / LOOKAHEAD_S)) < 0.01,
                 'clamp %.3f at %.3f s of hold' % (got['derate'], hold))

    # ZERO DISABLES IT, bit for bit: the pre-lookahead behaviour has to
    # remain reachable, because a record that never had the field reads
    # back as zero and must still get the old envelope.
    model = Model(lib)
    # AT 105 C: at 85 the node still holds 35 W for half a second, which is
    # outside the window, so both rules answered 1.0 and the check compared
    # two untouched clamps. The window binds from about 100 C at this power.
    model.place('phase_u', 105.0)
    report.check('a cooling node has no hold to run out of',
                 model.budget(lookahead_s=LOOKAHEAD_S)['derate'] == 1.0)
    hot = model.budget(watt, lookahead_s=LOOKAHEAD_S)['derate']
    flat = model.budget(watt, lookahead_s=0.0)['derate']
    report.check('and with power on it, the window is what differs',
                 hot < flat, 'window %.3f against present-only %.3f'
                 % (hot, flat))


def test_the_step_must_land_inside_the_ramp(report, lib):
    """A throttle band is only there if something looks inside it.

    THE DEFECT THIS GUARDS, found 2026-09-03 by the bench in the rotor
    observer: the envelope peaked and then collapsed, because the
    integration ran a whole poll gap and evaluated the budget once at the
    end. The ramp is the last `lookahead_s * (1 - throttle_at)` of a
    node's hold - 300 ms at the record's numbers - so a step longer than
    that lands on the far side of it and the only thing left to do is
    trip. `Board_ThermalPoll` consumes a late gap in THERMAL_STEP_MS
    slices for exactly this reason, and the stand-in does the same.

    The same two seconds of model time at every step size, and what the
    throttle saw the first time it looked.
    """
    # THE REAL LOAD, every node live. Fed only the phase node the case is
    # too gentle to show it: 35 W into 0.40 J/K is slow enough that even a
    # one second step lands at 108 C with the clamp at 0.71. It is the
    # DRIVER node that makes it - 18.4 W into 0.12 J/K - and that is the
    # node a bench actually has.
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

    # THE COUNTER-EXAMPLE, kept because it is what the defect looked like:
    # at a step longer than the ramp the first evaluation is already past
    # the ceiling with the clamp shut, and the only thing left is the trip.
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


def test_conduction_is_a_mean_square_not_a_sample(report, lib):
    """One instant squared is the loss only if that instant was the rms.

    THE DEFECT, and it is the board's rather than the model's: `load_now`
    handed the estimator ONE sample per 100 ms and it squared it. A single
    instant of a rotating three-phase current says where the vector is
    pointing, not how big it has been - squared, it runs from zero to
    twice the true loss depending only on where in the electrical period
    the sample landed.

    Worse here than a coin toss would be, because the sampler is
    SYNCHRONOUS: the trigger is a tick inside the PWM period, so at a
    speed whose electrical period divides the poll interval the alias
    LOCKS and a leg at its peak reads as a leg at zero for as long as the
    speed holds. `Board_SyncMeanSquare` accumulates in the injected
    callback instead, and `phase_sq` is what it hands over.
    """
    peak = 100.0
    rms_sq = peak * peak / 2.0

    # The two instants a sampler can land on, and what each says the loss
    # is when it is squared on its own.
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

    # AND IT IS PER LEG. A three-phase sum would be constant and could be
    # shared out, but this board also drives one leg against another -
    # `tools/pulse.py` does exactly that - and spreading U's heat over an
    # idle W would be a model that could not represent its own bench test.
    one_leg = power(lib, phase_amps=(0.0, 0.0, 0.0), switching=False,
                    phase_sq=(rms_sq, rms_sq, 0.0))
    report.check('an idle leg stays cold while two carry current',
                 one_leg['phase_w'] == 0.0 and one_leg['phase_u'] > 0.0,
                 'U %.2f W, W %.3f W' % (one_leg['phase_u'],
                                         one_leg['phase_w']))

    # THE FALLBACK IS THE OLD BEHAVIOUR, bit for bit: a caller with only a
    # sample - the harness, a host, a board whose sampler is not armed -
    # gets what it always got rather than zero.
    report.check('no mean square means the sample is squared, as before',
                 power(lib, phase_amps=(peak, 0.0, 0.0), switching=False,
                       phase_sq=(0.0, 0.0, 0.0))['phase_u']
                 == at_peak['phase_u'])
    report.check('and a negative one is not measured either',
                 power(lib, phase_amps=(peak, 0.0, 0.0), switching=False,
                       phase_sq=(-1.0, 0.0, 0.0))['phase_u']
                 == at_peak['phase_u'])

    # The link estimate keeps the SIGNED sample: a mean square has none.
    fwd = power(lib, phase_amps=(50.0, 0.0, 0.0), duty=(1.0, 0.0, 0.0),
                switching=False, phase_sq=(rms_sq, 0.0, 0.0))
    back = power(lib, phase_amps=(-50.0, 0.0, 0.0), duty=(1.0, 0.0, 0.0),
                 switching=False, phase_sq=(rms_sq, 0.0, 0.0))
    report.check('the conduction does not care which way the current went',
                 abs(fwd['phase_u'] - back['phase_u']) < 1e-6,
                 '%.3f against %.3f' % (fwd['phase_u'], back['phase_u']))
    report.check('and the hot swap still sees the same link current either '
                 'way, because it squares it too',
                 abs(fwd['regulators'] - back['regulators']) < 1e-6)


def test_the_thermistor_has_mass(report, lib):
    """A sensor a centimetre from the silicon cannot slew like silicon.

    THE SANITY RULE THE ALGEBRA HAD NONE OF. `thermal_expected_ntc` was a
    function of the driver node alone, so the modelled reading followed a
    small fast lump instantly - 18 W into 0.12 J/K is 150 K a second, and
    the page showed an NTC doing exactly that. Heat has to cross copper
    that has its own mass, and what arrives is low passed.

    The lag is the leg node's own RC, on the argument that a sensor in a
    lump is not quicker than the lump - a floor, not a fit. Steady state
    is untouched, which is the point: this bounds the RATE and nothing
    else.
    """
    # ALL THREE LEGS. The thermistor watches leg V, so a load on U alone
    # leaves the target where it started and the check measures nothing -
    # which is what the first version of it did.
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

    # SUBSTANTIALLY SLOWER THAN THE SOA ACTS, which is the whole point.
    # The envelope moves on the leg node in a fifth of a second to two
    # thirds at 100 A; a sensor soldered into laminate that kept up with
    # that would not be a sensor in laminate, it would be a second copy of
    # the FET. Its lag was exactly the leg's own until 2026-09-04.
    leg_rose = model.at('driver_v') - AMBIENT
    report.check('the reading trails the node it watches by a wide margin '
                 'over the same second',
                 leg_rose > 20.0 * rose,
                 'leg +%.1f K, reading +%.1f K' % (leg_rose, rose))

    # THE NODE IT WATCHES IS FREE TO SLEW - only the reading is not. A lag
    # that slowed the model itself would be a slower envelope, and the
    # envelope is the one thing that must not be.
    report.check('the driver node itself is not slowed by it',
                 model.at('driver_u') - AMBIENT > rose,
                 'driver +%.1f K against the reading +%.1f K'
                 % (model.at('driver_u') - AMBIENT, rose))

    # AND IT ARRIVES. A lag is not a cap: given time the reading reaches
    # the algebra, so the campaign's steady state is untouched.
    # Long enough for the BOARD to settle too - its own constant is
    # 49 J/K across 8.33 K/W, near seven minutes, and a reading chasing a
    # target that is itself still climbing lands behind it.
    for _ in range(20000):
        model.step(watt, 1.0)
    # THE ELEMENT'S OWN STEADY STATE: a weighted average of the two nodes
    # it is tied to, with no additive offset. `NTC_SEES_DRIVERS` is the
    # weight, and the point of the form is that it lands BETWEEN them for
    # any weight at all.
    board = model.at('board')
    leg = model.at('driver_v')
    target = board + NTC_SEES_LEG * (leg - board)
    report.check('given time it lands on the weighted average of the two '
                 'nodes it is tied to',
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
    """Its constant is the geometric mean of the pair it sits between.

    AN ELEMENT BETWEEN TWO NODES LAGS BETWEEN THEIR CONSTANTS, and the
    geometric mean is what "between" means for a time constant - the
    log-midpoint, because a lag is a ratio and not a difference. Checked
    against the two the model already carries rather than against a
    number typed here, so moving either moves this and nothing drifts.
    """
    model = Model(lib)
    leg = model.capacity('driver_v') * 45.6
    board = model.capacity('board') * 8.33

    # THE TWO NODES HELD, so the target does not move while the reading
    # walks toward it. Measured any other way this reads the leg's own
    # constant as well as the thermistor's, and the first version of this
    # check did exactly that - it stepped once with the nodes still at
    # ambient and divided by a target that had not appeared yet.
    hot, cold = 120.0, 40.0
    target = cold + NTC_SEES_LEG * (hot - cold)
    zero = {}
    start = None
    for step in range(400):
        model.place('driver_v', hot)
        model.place('board', cold)
        if start is None:
            model.step(zero, 1e-4)
            start = model.ntc()
        model.step(zero, 0.05)
        if (step + 1) * 0.05 >= 1.0:
            break
    share = (model.ntc() - start) / max(1e-9, target - start)
    tau = -1.0 / math.log(max(1e-9, 1.0 - min(0.999999, share)))

    report.check('the reading lags well past the node it watches',
                 tau > 5.0 * leg, '%.1f s against the leg %.2f s'
                 % (tau, leg))
    report.check('and well short of the bulk board, which is the other '
                 'end of what it sits between',
                 tau < board / 5.0, '%.1f s against the board %.0f s'
                 % (tau, board))
    report.check('the geometric mean of the two, near enough',
                 abs(tau / math.sqrt(leg * board) - 1.0) < 0.15,
                 '%.1f s against %.1f' % (tau, math.sqrt(leg * board)))


def test_the_burst_budget_rests_on_an_unmeasured_capacity(report, lib):
    """What the leg capacity is worth, since nobody measured it.

    `thermal.c` has said so since the campaign: "the parts' own are not
    measured - they respond in seconds, below what this rig can resolve,
    and only affect the settling". THE LAST CLAUSE IS NO LONGER TRUE. The
    envelope divides by exactly these numbers - `soak_j` is
    `capacity x (limit - t)`, `hold_seconds` is that over the net watts,
    and the throttle's reaction window is a multiple of it.

    So this does not assert a value. It measures the BAND: what the burst
    budget becomes at the capacity on record and at Silva's gamma, which
    is the other end of what the number could honestly be. A test that
    pinned one of them would be claiming a measurement nobody took.
    """
    watt = power(lib, phase_amps=(100.0, 0.0, 0.0), duty=(0.5, 0.0, 0.0),
                 link_volts=48.0, switching=True)
    seen = {}
    for name, scale in (('on record', 1.0), ('at gamma', GAMMA)):
        model = Model(lib)
        base = model.capacity('driver_u')
        report.check('the capacity moves when a bench moves it (%s)' % name,
                     model.set_node('driver_u', 45.6, base * scale),
                     '%.4f J/K' % (base * scale))
        got = model.budget(watt, lookahead_s=LOOKAHEAD_S)
        # Seconds from ambient to the ceiling at this power, which is what
        # a burst is spending.
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

    # THE BAND, stated as a number a bench can act on. Nothing here says
    # which end is right; that is what the power step would answer.
    report.check('so the 100 A burst budget is a band, not a figure',
                 seen['at gamma'][1] < seen['on record'][1],
                 'between %.2f s and %.2f s on the driver node'
                 % (seen['at gamma'][1], seen['on record'][1]))

    # AND THE THROTTLE MOVES WITH IT. The lookahead window is a fixed time,
    # so a lighter node crosses it sooner - the reaction the envelope gets
    # is shorter by the same factor, which is the part that costs silicon.
    first = {}
    for name, scale in (('on record', 1.0), ('at gamma', GAMMA)):
        model = Model(lib)
        model.set_node('driver_u', 45.6, model.capacity('driver_u') * scale)
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
          test_the_lookahead_catches_a_ramp,
          test_the_step_must_land_inside_the_ramp, test_the_soak_is_joules,
          test_the_worst_node_is_the_one_acted_on,
          test_the_conduction_is_split_where_it_is_made,
          test_conduction_is_a_mean_square_not_a_sample,
          test_the_thermistor_has_mass,
          test_the_reading_lags_between_the_two_nodes,
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
