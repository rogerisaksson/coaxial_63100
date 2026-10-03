"""The gynoid shoved, tripped and down: the parry, the fall past saving, the pads, the get-up's
roll and its planner, and up and walking again after the page's P."""
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report


#: Shoves of SHOVE_N for SHOVE_S along her side after SHOVE_AFTER_S of walking, at 8 phases of
#: the left leg's stride, a test a side: each one caught within PARRY_SEEN_S or held without, the
#: arms raised, the feet never further apart than APART_M; HELD_OF_8 held. Measured: as a 55 kg
#: woman at 60 N, held 3 to her left and 4 to her right, caught 0.07-0.25 s on; as built, 35 kg,
#: at 38 N - the same 0.13 m/s - 5 and 4, the falls caught 0.09-0.10 s on, two held caught 0.41
#: s on, the feet 0.58 m apart at most (2026-10-01). A test a side: 16 took 185 s, past CI's 300.
SHOVE_N, SHOVE_S, SHOVE_AFTER_S, PARRY_SEEN_S, APART_M, HELD_OF_8 = 38.0, 0.12, 6.0, 0.25, 0.6, 2


#: The page's shove past saving, after SHOVE_AFTER_S of walking at 8 phases, a test a side: her
#: head never meeting the floor faster than HEAD_MS, a tap; her body's peak on the floor over
#: LANDING_S from the fall, feet aside, at the median under PEAK_KN; a limb or her seat on the
#: floor first, never her trunk or her head. Measured over 16, as a 55 kg woman: limp 8.7 kN, a
#: thigh first 8 times; crouched 6.2, a shank first 14. As built, 35 kg: 4.2 kN, a hand first 6
#: times, a shank 10; her head down 3 times, two at rest, 0.09-0.12 m/s, one at 0.72 (2026-10-01).
LANDING_S, PEAK_KN, HEAD_MS = 1.5, 7.0, 1.0


#: Felled by the page's P walking (`tools.sim.getup_search`'s falls), she is up and walking again
#: UP_S after she lay down, a test a side (the user, 2026-10-03: she must get up after a fall).
#: Before the drives went to modules 5 falls of 5 walked at 20.5 s, after them none of 5 - the
#: gate had nothing that walked her up from the floor (2026-10-03).
UP_S = 40.0


#: The roll onto her front from flat on her back: no foot past ROLL_FOOT_MS, no hip or knee past
#: ROLL_LEG_DEG_S. Its leg thrown, a foot flew 4.9 m/s and the hip and knee 803 deg/s (2026-10-01).
ROLL_FOOT_MS, ROLL_LEG_DEG_S = 2.0, 300.0


def _parried(report, side):
    """Shoved toward `side` (1 her left) walking, she parries (`machine.landing`,
    `machine.parry`): the shove seen as the capture point leaves her feet, a hurried step that
    may cross over toward it, the arms raised - choreographed, her feet kept within a step - and
    held as often as measured."""
    from machine import Machine
    from machine.director import Director
    from machine.modes import DYNAMIC
    rows = []
    for phase in [k / 8.0 + 0.01 for k in range(8)]:
        body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        body.arm()
        director = Director(body, 0.85)
        director.walker.start()
        director.stage = 'walk'
        body.loop.step(0.0)
        bus, world = body.loop.bus, body.nodes['pelvis'].world
        feet = [world.model.body(s + '_foot').id for s in ('left', 'right')]
        pushed, was, fell, seen, apart, arms = None, 0.0, False, None, 0.0, 0.0
        while bus['t'] < (pushed or 99.0) + 4.0 and not fell:
            if pushed is None and bus['t'] >= SHOVE_AFTER_S and was < phase <= director.walker.phase:
                world.push((side * SHOVE_N, 0.0, 0.0), SHOVE_S)
                pushed = bus['t']
            was = director.walker.phase
            body.loop.write(**director.step(0.001))
            body.loop.step(0.001)
            fell = director.stage in ('falling', 'fallen')
            if pushed is not None:
                seen = seen if seen is not None or director.stage != 'catch' else bus['t'] - pushed
                apart = max(apart, abs(world.data.xpos[feet[0]][0] - world.data.xpos[feet[1]][0]))
                arms = max(arms, min(bus.get(s + '_shoulder.deg', 0.0) for s in ('left', 'right')))
        body.close()
        rows.append((not fell, seen, apart, arms))
    late = [r[1] for r in rows if not r[0] and (r[1] is None or r[1] > PARRY_SEEN_S)]
    report.check('every shove caught within %.2f s or held' % PARRY_SEEN_S, not late,
                 '%.2f-%.2f s' % (min(r[1] or 9.0 for r in rows), max(r[1] or 9.0 for r in rows)))
    report.check('the arms raised in the parry', min(r[3] for r in rows) > 0.0,
                 '%.0f-%.0f deg' % (min(r[3] for r in rows), max(r[3] for r in rows)))
    report.check('her feet never more than %.1f m apart' % APART_M,
                 max(r[2] for r in rows) <= APART_M, '%.2f m' % max(r[2] for r in rows))
    report.check('%.0f N for %.2f s held %d of 8 at least' % (SHOVE_N, SHOVE_S, HELD_OF_8),
                 sum(r[0] for r in rows) >= HELD_OF_8, '%d of 8' % sum(r[0] for r in rows))


def test_a_shove_to_her_left_parried(report):
    """`_parried` toward her left."""
    _parried(report, 1.0)


def test_a_shove_to_her_right_parried(report):
    """`_parried` toward her right."""
    _parried(report, -1.0)


def test_a_trip_lands_her_shorted(report):
    """A lace snagged shoe to shoe (`machine.events`, `World.lace`) trips her past recovery, a
    catch step tried or not: her head not the first of her on the floor; down, her legs' and
    trunk's drives shorted, her arms and neck not - she settles, not held stiff nor flailing;
    lain still, every drive cut and checked (`machine.down`), she begins to get up
    (`machine.getup`)."""
    from machine import Machine, drives, events, figure, getup, heat
    from machine.director import Director
    from machine.falls import SHORT_FALLING
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    director.walker.start()
    director.stage = 'walk'
    body.loop.step(0.0)
    world, bus = body.nodes['pelvis'].world, body.loop.bus
    ours = {world.model.body(seg[0]).id: seg[0] for seg in figure.SEGMENTS}
    laid, was, first, stages, down, shorted, cut = None, 0.0, None, [], None, None, None
    while bus['t'] < 10.0 and director.stage not in getup.STAGES:
        if laid is None and bus['t'] >= 1.0 and was < events.at('lace') <= director.walker.phase:
            events.lay('lace', director, world)
            laid = bus['t']
        was = director.walker.phase
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if not stages or stages[-1] != director.stage:
            stages.append(director.stage)
        d, m = world.data, world.model
        for i in range(d.ncon):
            a, b = (m.geom_bodyid[g] for g in (d.contact[i].geom1, d.contact[i].geom2))
            for mine, other in ((a, b), (b, a)):
                name = ours.get(mine, '')
                if name and other not in ours and not name.endswith(('foot', 'toes')):
                    first = first or name
        if director.stage == 'fallen' and down is None:
            down = bus['t']
        if down is not None and shorted is None and bus['t'] > down + 0.2:
            shorted = {j: bool(int(bus[name + 'status']) & heat.SHORTED)
                       for j, name in director.drives.items()}
        if cut is None and director.down is not None and director.down.at > 0.1:
            cut = [j for j, name in director.drives.items()
                   if int(bus[name + 'status']) & (heat.GATES_ON | heat.SHORTED)]
    report.check('the lace tripped her past recovery: falling, then down',
                 laid is not None and [s for s in stages[1:] if s not in ('catch', 'walk')][:2]
                 == ['falling', 'fallen'], ' '.join(stages))
    report.check('her head not the first of her on the floor', first not in (None, 'head'),
                 'first %s' % first)
    wrong = [j for j, s in (shorted or {}).items() if s != (drives.kind(j) in SHORT_FALLING)]
    report.check('down, the legs\' and trunk\'s phases shorted, the arms and neck not',
                 shorted is not None and not wrong,
                 '%d of %d as asked, else: %s' % (len(director.drives) - len(wrong),
                                                  len(director.drives), ' '.join(wrong) or 'none'))
    report.check('lain still, every drive cut: its gates off, its phases open',
                 cut == [], 'on or shorted: %s' % (' '.join(cut) if cut else 'none' if cut == []
                                                   else 'never cut'))
    report.check('cut, she is checked before she gets up', 'check' in stages, ' '.join(stages))
    report.check('lain still, she begins to get up', director.stage in getup.STAGES,
                 '%s at %.1f s, down at %s' % (director.stage, bus['t'],
                                              '%.1f s' % down if down else '-'))
    body.disarm()


def test_lying_still_she_holds_nothing(report):
    """Shoved past saving with no get-up left (`director.GETUP_TRIES`), she brakes her fall and
    lies cut: down, not still falling - the page lands her again (`running.RECOVER_S`) - every
    drive's gates off and its phases open, drawing its board's own (`World.drawn`). Lying, the
    page drew 120 W: her tuck held her curled, and given up she stayed falling (2026-10-02)."""
    from machine import Machine, events, heat
    from machine.director import GETUP_TRIES, Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    director.walker.start()
    director.stage = 'walk'
    director.tries = GETUP_TRIES
    body.loop.step(0.0)
    bus, world = body.loop.bus, body.nodes['pelvis'].world
    laid, was, lain = False, 0.0, None
    while bus['t'] < (lain or 20.0) + 0.5:
        if not laid and bus['t'] >= 1.0 and was < events.at('shove') <= director.walker.phase:
            events.lay('shove', director, world)
            laid = True
        was = director.walker.phase
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        if lain is None and director.down is not None and director.down.mode == 'checked':
            lain = bus['t']
    on = [j for j, name in director.drives.items()
          if int(bus[name + 'status']) & (heat.GATES_ON | heat.SHORTED)]
    own = heat.HOUSEKEEPING_W * int(world.driven.sum())
    report.check('shoved past saving, she lies down given up, not falling',
                 lain is not None and director.stage == 'fallen' and director.given_up,
                 '%s, given up %s' % (director.stage, director.given_up))
    report.check('lying, every drive cut: its gates off, its phases open',
                 lain is not None and not on, 'on or shorted: %s' % (' '.join(on) or 'none'))
    report.check("lying, she draws her boards' own", abs(world.drawn() - own) < 1e-6,
                 '%.1f W, her %d boards %.1f' % (world.drawn(), int(world.driven.sum()), own))
    body.disarm()


def _crouched(report, side):
    """Shoved past saving toward `side` (`events.SHOVES`), she goes down into a crouch
    (`falls.crouch`): a knee and a shin take the floor first, her head never, her landing softer
    than limp."""
    import numpy as np
    from machine import Machine, figure
    from machine.director import Director
    from machine.events import SHOVE_S, SHOVES
    from machine.modes import DYNAMIC
    rows = []
    for phase in [k / 8.0 + 0.01 for k in range(8)]:
        body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        body.arm()
        director = Director(body, 0.85)
        director.walker.start()
        director.stage = 'walk'
        body.loop.step(0.0)
        bus, world = body.loop.bus, body.nodes['pelvis'].world
        m, d = world.model, world.data
        ours = {m.body(seg[0]).id: seg[0] for seg in figure.SEGMENTS}
        f, pushed, was, falling, first, peak, head = np.zeros(6), None, 0.0, None, None, 0.0, None
        v6, skull = np.zeros(6), m.body('head').id
        while bus['t'] < (falling or pushed or 99.0) + (LANDING_S if falling else 4.0):
            if pushed is None and bus['t'] >= SHOVE_AFTER_S and was < phase <= director.walker.phase:
                world.push((side * SHOVES['shove'], 0.0, 0.0), SHOVE_S)
                pushed = bus['t']
            was = director.walker.phase
            body.loop.write(**director.step(0.001))
            body.loop.step(0.001)
            falling = falling or (director.stage == 'falling' and bus['t'])
            if not falling:
                continue
            total = 0.0
            for i in range(d.ncon):
                mine = [b for b in (m.geom_bodyid[d.contact[i].geom1],
                                    m.geom_bodyid[d.contact[i].geom2]) if b in ours]
                if len(mine) == 1 and not ours[mine[0]].endswith(('foot', 'toes')):
                    world._mj.mj_contactForce(m, d, i, f)
                    total += abs(f[0])
                    if ours[mine[0]] == 'head' and head is None:
                        world._mj.mj_objectVelocity(m, d, world._mj.mjtObj.mjOBJ_BODY, skull, v6, 0)
                        head = float(np.linalg.norm(v6[3:]))
                    first = first or ours[mine[0]]
            peak = max(peak, total / 1e3)
        body.close()
        rows.append((bool(falling), first or '', peak, head))
    fell = [r for r in rows if r[0]]
    report.check('the shove past saving felled her every time', len(fell) == len(rows),
                 '%d of %d' % (len(fell), len(rows)))
    heads = [r[3] for r in fell if r[3] is not None]
    report.check('her head never on the floor faster than %.1f m/s' % HEAD_MS,
                 all(v <= HEAD_MS for v in heads),
                 '%d times, %.2f m/s at most' % (len(heads), max(heads or [0.0])))
    peaks = sorted(r[2] for r in fell) or [0.0]
    report.check('her landing\'s peak under %.0f kN at the median' % PEAK_KN,
                 peaks[len(peaks) // 2] < PEAK_KN,
                 'median %.1f kN, worst %.1f' % (peaks[len(peaks) // 2], peaks[-1]))
    report.check('a limb or her seat on the floor first, never her trunk or her head',
                 all(r[1] and r[1] not in ('torso', 'neck', 'head') for r in fell),
                 ', '.join(sorted({r[1] or '-' for r in fell})))


def test_a_fall_to_her_left_crouches(report):
    """`_crouched` toward her left."""
    _crouched(report, 1.0)


def test_a_fall_to_her_right_crouches(report):
    """`_crouched` toward her right."""
    _crouched(report, -1.0)


def test_her_pads(report):
    """Her pads (`figure.PADS`, where `tools/sim/landings.py where` finds her falls land), 5 mm of
    gel (`mjcf.PAD_*`, its `gel` the spread of falls): dropped onto her knees she lands on them,
    softer than bare, the gel giving no further than its own thickness past her bare skin."""
    from machine import Machine, figure
    from machine.modes import DYNAMIC

    def drop():
        body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        body.arm()
        world = body.nodes['pelvis'].world
        m, d = world.model, world.data
        angles = dict({j: 0.0 for j in figure.JOINTS}, left_knee=90.0, right_knee=90.0)
        world.reset(angles, where=(0.0, 0.55, 0.0), turn=(1.0, 0.0, 0.0, 0.0))
        body.loop.step(0.0)
        shanks = {m.body(s + '_shank').id for s in ('left', 'right')}
        peak, deep, padded, total, f = 0.0, 0.0, 0.0, 0.0, world._np.zeros(6)
        for _ in range(300):
            body.loop.write(**angles)
            body.loop.step(0.001)
            force = 0.0
            for i in range(d.ncon):
                c = d.contact[i]
                for g in (c.geom1, c.geom2):
                    if m.geom_bodyid[g] in shanks:
                        world._mj.mj_contactForce(m, d, i, f)
                        force += abs(f[0])
                        deep = max(deep, -c.dist)
                        padded += abs(f[0]) * ('_pad' in (m.geom(g).name or ''))
            total += force
            peak = max(peak, force / 2.0)
        body.close()
        return peak, deep, padded / max(total, 1e-9)
    gel = drop()
    pads, figure.PADS = figure.PADS, ()
    try:
        bare = drop()
    finally:
        figure.PADS = pads
    report.check('dropped onto her knees, she lands on their pads', gel[2] > 0.5,
                 '%.0f %% of the knees\' load through them' % (100 * gel[2]))
    report.check('the gel lands softer than her bare skin', gel[0] < bare[0],
                 '%.0f N a knee, bare %.0f' % (gel[0], bare[0]))
    report.check('the gel gives no further than its %.0f mm past her bare skin'
                 % (1e3 * figure.PAD_M), gel[1] - bare[1] <= figure.PAD_M,
                 '%.1f mm, bare %.1f' % (1e3 * gel[1], 1e3 * bare[1]))


def test_the_roll_pushes_her_over(report):
    """From her back onto her front (`getup.FRONTS_BY`), pushed over by a planted foot and her
    arms into the recovery position - no leg thrown: face down at its end."""
    import math
    from machine import Machine, getup, observer
    from machine.director import Director
    from machine.figure import JOINTS
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    world = body.nodes['pelvis'].world
    flat, h = getup._pose(0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 90.0), math.pi / 4.0
    world.reset(flat, where=(0.0, 0.14, 0.0), turn=(math.cos(h), -math.sin(h), 0.0, 0.0))
    body.loop.step(0.0)
    bus = body.loop.bus
    for _ in range(600):
        body.loop.write(**flat)
        body.loop.step(0.001)
    roll = getup.FRONTS_BY[1]
    director.getup.begin(roll)
    director.stage = 'roll'
    feet = [world.model.body(s + '_foot').id for s in ('left', 'right')]
    legs = [world.vadr[i] for i, j in enumerate(JOINTS) if j.split('_', 1)[-1] in ('hip', 'knee')]
    v6, foot, spin = world._np.zeros(6), 0.0, 0.0
    end = bus['t'] + sum(step[2] for step in roll)
    while bus['t'] < end:
        body.loop.write(**director.getup.step(0.001))
        body.loop.step(0.001)
        for f in feet:
            world._mj.mj_objectVelocity(world.model, world.data, world._mj.mjtObj.mjOBJ_BODY, f, v6, 0)
            foot = max(foot, math.sqrt(v6[3] ** 2 + v6[4] ** 2 + v6[5] ** 2))
        spin = max(spin, max(abs(math.degrees(world.data.qvel[i])) for i in legs))
    lying = observer.lying(bus, world)
    body.close()
    report.check('rolled face down', lying == 'face down', lying)
    report.check('no foot past %.1f m/s' % ROLL_FOOT_MS, foot <= ROLL_FOOT_MS, '%.1f m/s' % foot)
    report.check('no hip or knee past %.0f deg/s' % ROLL_LEG_DEG_S, spin <= ROLL_LEG_DEG_S,
                 '%.0f deg/s' % spin)


def _up(report, k):
    """Felled by the page's P toward her left (`k` even) or right, she gets up and walks."""
    from tools.sim import getup_search
    lying = getup_search.start(('shove', k))
    report.check("the page's P felled her", lying is not None, 'she stayed up')
    if lying is None:
        return
    _cost, walked, tries = getup_search.trial(({}, lying))
    report.check('up and walking again within %.0f s of lying down' % UP_S,
                 walked is not None and walked <= UP_S,
                 '%s, %d tries' % ('at %.1f s' % walked if walked else 'down', tries))


def test_up_after_a_fall_to_her_left(report):
    """Felled toward her left by the page's P, she gets up and walks on (`machine.getup`)."""
    _up(report, 0)


def test_up_after_a_fall_to_her_right(report):
    """Felled toward her right by the page's P, she gets up and walks on (`machine.getup`)."""
    _up(report, 1)


def test_the_planner(report):
    """Her get-up's plan (`machine.planner`): a model's answer read and checked, a server asked
    once the local model has failed LOCAL_TRIES times, the house's own when neither answers."""
    import json
    from machine import planner

    class Model:
        def __init__(self, steps=None, fails=False):
            self.steps, self.fails, self.asked = steps, fails, 0

        def chat(self, messages, fmt=None, think=None, num_predict=None):
            self.asked += 1
            if self.fails:
                raise OSError('no daemon')
            return {'content': json.dumps({'steps': self.steps, 'why': 'test'})}
    now = {'lying': 'face down', 'left_up': 0.0}
    up = ['knees under', 'sit back on heels', 'onto feet']
    report.check("a local model's plan is hers", planner.plan(now, Model(up)) == (tuple(up), 'local'))
    made_up = planner.plan(now, Model(['cartwheel', 'onto feet']))
    report.check('a step the model made up falls to the default', made_up[1] == 'default',
                 ' > '.join(made_up[0]))
    report.check('a plan not ending on her feet falls to the default',
                 planner.plan(now, Model(['knees under']))[1] == 'default')
    report.check('a step from where the one before does not leave her falls to the default',
                 planner.plan(now, Model(['knees under', 'onto feet']))[1] == 'default')
    report.check('a step cannot begin where the observer says she lies otherwise',
                 not planner.fits('knees under', 'on her back')
                 and planner.fits('knees under', 'face down'))
    report.check('a model that fails falls to the default',
                 planner.plan(now, Model(fails=True))[1] == 'default')
    server = Model(up)
    tried = [(tuple(up), 'failed')] * planner.LOCAL_TRIES
    report.check('the server is asked once the local model failed %d times' % planner.LOCAL_TRIES,
                 planner.plan(now, Model(up), server, tried)[1] == 'server' and server.asked == 1)
    lying = ('face down', 'on her back', 'on her left side', 'on her right side', 'sitting',
             'kneeling')
    report.check('every default plan ends on her feet, each step from where the last left her',
                 all(planner.plan({'lying': w})[0][-1] in planner.UP
                     and planner.chained(planner.plan({'lying': w})[0], w) for w in lying))
    stream, marks = planner.stream(planner.plan(now)[0], now)
    report.check('its stream marks each step where it ends',
                 [s for _i, s in marks] == list(planner.plan(now)[0]) and marks[-1][0] == len(stream))


ROSTER = (test_a_shove_to_her_left_parried, test_a_shove_to_her_right_parried,
          test_a_trip_lands_her_shorted, test_lying_still_she_holds_nothing,
          test_a_fall_to_her_left_crouches, test_a_fall_to_her_right_crouches,
          test_her_pads, test_the_roll_pushes_her_over, test_the_planner,
          test_up_after_a_fall_to_her_left, test_up_after_a_fall_to_her_right)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
