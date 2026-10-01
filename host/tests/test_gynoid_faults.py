#!/usr/bin/env python3
"""The gynoid on her boards as built: their envelopes derating and tripping them, a gate drive
failed into its SOA, a lace caught - and fantasy boards, whose envelope never binds
(`physics.ENVELOPE`). Her walk and her look are test_gynoid.py's."""
import sys

from tools.dev.focus import chosen

class Report:
    def __init__(self):
        self.passed = self.failed = 0

    def check(self, name, ok, detail=''):
        self.passed += bool(ok)
        self.failed += (not ok)
        print('  %s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail))


#: A gate drive failed low for FAILED_S from FAILED_AT of the left leg's stride - the knee taking
#: her weight as it lands - its switches FAILED_RDS times their on-resistance. At mid-stance, 0.25,
#: the knee straight, it asked too little of them to trip in 0.1 s (2026-09-28).
FAILED_RDS, FAILED_S, FAILED_AT = 500.0, 0.15, 0.05


def test_fantasy_boards_never_bind(report):
    """A fantasy board (`heat.Heat.envelope` False) counts its envelope and never derates or
    trips; the world's ENVELOPE 0 reaches every board on its bus."""
    from machine import Machine, drives, heat, physics
    h = heat.Heat([drives.heat('left_knee')])
    h.envelope = False
    stall = drives.kt('left_knee') * drives.of('left_knee')[1].amps
    for _ in range(3000):
        for _ in range(10):
            h.load(0, stall)
        h.step(0.01, [1.0], [1.0])
    report.check('stalled at 100 A for 300 heat s: spent, never derated, its gates on',
                 h.spent[0] >= 1.0 and h.derate[0] == 1.0 and h.gates[0],
                 '%.0f C, spent %.2f' % (h.t[0][h.worst[0]], h.spent[0]))
    from machine.director import Director
    from machine.modes import DYNAMIC
    physics.ENVELOPE, was = 0.0, physics.ENVELOPE
    physics.SOA_RDS, was_rds = FAILED_RDS, physics.SOA_RDS
    try:
        body = Machine.discover('gynoid', execution_mode=DYNAMIC)
        body.arm()
        director = Director(body, 0.85)
        director.walker.start()
        director.stage = 'walk'
        body.loop.step(0.0)
        world, bus = body.nodes['pelvis'].world, body.loop.bus
        knee, off, least = director.drives['left_knee'], False, 1.0
        while bus['t'] < 1.5:
            if 0.5 <= bus['t'] < 0.501:
                world.glitch('left_knee', 'soa', FAILED_S)
            body.loop.write(**director.step(0.001))
            body.loop.step(0.001)
            off = off or not int(bus[knee + 'status']) & heat.GATES_ON
            least = min(least, bus[knee + 'derate'])
        report.check('the same glitch on a fantasy knee: its gates on, never derated',
                     not off and least == 1.0, 'derate %.2f, gates %s' % (
                         least, 'dropped' if off else 'on'))
        body.disarm()
    finally:
        physics.ENVELOPE, physics.SOA_RDS = was, was_rds


def test_a_drive_keeps_its_heat(report):
    """A drive's board keeps its heat (`machine.heat`): stalled at its board's 100 A it derates
    from THROTTLE_AT of its envelope and drops its gates at a ceiling; armed and idle it cools,
    its derate given back at RECOVER_PER_S; its report goes on the wire and back whole."""
    from machine import drives, heat, rtu
    h = heat.Heat([drives.heat('left_knee')])
    stall = drives.kt('left_knee') * drives.of('left_knee')[1].amps
    air, rds, derated_at, tripped_at = [1.0], [1.0], None, None
    for k in range(3000):
        for _ in range(10):
            h.load(0, stall if h.gates[0] else 0.0)
        h.step(0.01, air, rds)
        if h.derate[0] < 1.0 and derated_at is None:
            derated_at = (k, h.spent[0])
        if not h.gates[0] and tripped_at is None:
            tripped_at = (k, h.derate[0], h.trips[0])
    report.check('stalled at 100 A: derated past %.2f spent, then its gates dropped' %
                 heat.THROTTLE_AT,
                 derated_at is not None and tripped_at is not None
                 and derated_at[0] < tripped_at[0] and derated_at[1] > heat.THROTTLE_AT
                 and tripped_at[2] == 1, '%s %s' % (derated_at, tripped_at))
    h.arm(0)
    for _ in range(6000):
        h.step(0.01, air, rds)
    celsius, spent, derate, status = h.report(0)
    report.check('armed and idle 60 s, cool: its gates on, its derate back to 1',
                 h.gates[0] and derate == 1.0 and spent < heat.THROTTLE_AT
                 and status & heat.GATES_ON, '%.1f C, spent %.2f, derate %.2f' % (
                     celsius, spent, derate))
    frame = rtu.reply(3, 12345, -6789, round(celsius * 100.0), round(spent * 1e4),
                      round(derate * 1e4), status)
    (unit, fc, body), = rtu.replies(frame + rtu.echo(rtu.gate(3)))[0][:1]
    got = rtu.state(body)
    report.check('its state on the wire: the angle, the rate and the heat back whole',
                 (unit, fc) == (3, rtu.READ) and len(frame) == rtu.REPLY_B
                 and got[:2] == (12345, -6789) and abs(got[2] / 100.0 - celsius) < 0.01
                 and got[5] == status, got)
    report.check('a gate write parsed as the host sends it and the board echoes it',
                 rtu.requests(rtu.gate(2))[0] == [(2, rtu.WRITE_ONE, rtu.gate(2)[2:-2])]
                 and rtu.replies(rtu.echo(rtu.gate(2)))[0][0][:2] == (2, rtu.WRITE_ONE))
    report.check('its op read back: the gates on, the phases shorted',
                 (rtu.gate_op(rtu.gate(2)), rtu.gate_op(rtu.gate(2, rtu.GATE_SHORT)))
                 == (rtu.GATE_ON, rtu.GATE_SHORT))


def test_a_drive_in_its_soa(report):
    """A knee's board run hard into its SOA as it takes her weight (its gate drive failed a
    moment): its gates drop under load, the director hears it on the bus and arms it again, its
    derate given back as it cools, and she walks on. At `physics.SOA_RDS` its envelope derates it
    before its ceiling."""
    from machine import Machine, heat, physics
    physics.SOA_RDS, was_rds = FAILED_RDS, physics.SOA_RDS
    from machine.director import Director
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, 0.85)
    director.walker.start()
    director.stage = 'walk'
    body.loop.step(0.0)
    world, bus = body.nodes['pelvis'].world, body.loop.bus
    knee = director.drives['left_knee']
    dropped, armed, least, lowest, laid, was = None, None, 1.0, 9.0, None, 0.0
    while bus['t'] < 4.0:
        if laid is None and bus['t'] >= 1.0 and was < FAILED_AT <= director.walker.phase:
            world.glitch('left_knee', 'soa', FAILED_S)
            laid = bus['t']
        was = director.walker.phase
        body.loop.write(**director.step(0.001))
        body.loop.step(0.001)
        on = int(bus[knee + 'status']) & heat.GATES_ON
        dropped = bus['t'] if not on and dropped is None else dropped
        armed = bus['t'] if on and dropped is not None and armed is None else armed
        least, lowest = min(least, bus[knee + 'derate']), min(lowest, bus['pelvis.pose.y'])
    report.check('in its SOA, laid in its stance, its gates dropped, heard, and armed again '
                 'within 0.1 s', laid is not None and dropped is not None
                 and laid <= dropped < laid + 0.5 and armed is not None and armed - dropped < 0.1,
                 'laid %s, dropped %s, armed %s' % (laid, dropped, armed))
    report.check('derated to %.2f and given back to %.2f by 4 s, walking on over 2 m' % (
                     least, bus[knee + 'derate']),
                 least < 0.5 < bus[knee + 'derate'] and lowest > 0.7
                 and bus['pelvis.pose.z'] > 2.0, 'lowest %.2f m, %.2f m on' % (
                     lowest, bus['pelvis.pose.z']))
    physics.SOA_RDS = was_rds
    body.disarm()


def test_a_trip_lands_her_shorted(report):
    """A lace snagged shoe to shoe (`machine.events`, `World.lace`) trips her past recovery, a
    catch step tried or not: her head not the first of her on the floor; down, her legs' and
    trunk's drives shorted, her arms and neck not - she settles, not held stiff nor flailing;
    lain still, she begins to get up (`machine.getup`)."""
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
    laid, was, first, stages, down, shorted = None, 0.0, None, [], None, None
    while bus['t'] < 8.0 and director.stage not in getup.STAGES:
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
    report.check('lain still, she begins to get up', director.stage in getup.STAGES,
                 '%s at %.1f s, down at %s' % (director.stage, bus['t'],
                                              '%.1f s' % down if down else '-'))
    body.disarm()


#: Shoves of SHOVE_N for SHOVE_S along her side after SHOVE_AFTER_S of walking, at 8 phases of
#: the left leg's stride, to either side: each one caught within PARRY_SEEN_S, the arms raised,
#: the feet never further apart than APART_M; HELD_OF_16 held. Measured: HEAD held 1 and caught
#: 0.10-0.34 s on; the parry held 7, caught 0.07-0.25 s on, the feet 0.53 m apart at most
#: (2026-10-01).
SHOVE_N, SHOVE_S, SHOVE_AFTER_S, PARRY_SEEN_S, APART_M, HELD_OF_16 = 60.0, 0.12, 6.0, 0.25, 0.6, 5


def test_a_shove_parried(report):
    """Shoved sideways walking, she parries (`machine.landing`, `machine.parry`): the shove seen
    as the capture point leaves her feet, a hurried step that may cross over toward it, the arms
    raised - choreographed, her feet kept within a step - and held as often as measured."""
    from machine import Machine
    from machine.director import Director
    from machine.modes import DYNAMIC
    rows = []
    for phase in [k / 8.0 + 0.01 for k in range(8)]:
        for side in (1.0, -1.0):
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
    late = [r[1] for r in rows if r[1] is None or r[1] > PARRY_SEEN_S]
    report.check('every shove caught within %.2f s' % PARRY_SEEN_S, not late,
                 '%.2f-%.2f s' % (min(r[1] or 9.0 for r in rows), max(r[1] or 9.0 for r in rows)))
    report.check('the arms raised in the parry', min(r[3] for r in rows) > 0.0,
                 '%.0f-%.0f deg' % (min(r[3] for r in rows), max(r[3] for r in rows)))
    report.check('her feet never more than %.1f m apart' % APART_M,
                 max(r[2] for r in rows) <= APART_M, '%.2f m' % max(r[2] for r in rows))
    report.check('%.0f N for %.2f s held %d of 16 at least' % (SHOVE_N, SHOVE_S, HELD_OF_16),
                 sum(r[0] for r in rows) >= HELD_OF_16, '%d of 16' % sum(r[0] for r in rows))


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


def test_her_pads(report):
    """Her pads (`figure.PADS`, where `tools/sim/landings.py where` finds her falls land), 5 mm of
    gel (`physics.PAD_*`, its `gel` the spread of falls): dropped onto her knees she lands on them,
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


ROSTER = (test_a_drive_keeps_its_heat, test_a_drive_in_its_soa, test_a_trip_lands_her_shorted,
          test_fantasy_boards_never_bind, test_a_shove_parried, test_the_planner, test_her_pads)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
