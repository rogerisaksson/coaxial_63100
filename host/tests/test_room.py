"""The gynoid's turns on the one law and her errands in the living room (`machine.room`,
`machine.errands`): walking, her heading turned 0.3 rad/s, she walks on round and lands as
straight; standing, turned 0.5 rad/s, she steps round where she stands (`hold.staying`);
asked out she walks to the door, through it and stands. Her sit, her lie and the lamp are
docs/TODO.md's."""
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The law's heading turned from TURNED_AT s, the walk's row asked at WALKS_AT; a word asked at
#: ASKED_AT and given WORD_S.
WALKS_AT, TURNED_AT, ASKED_AT, WORD_S = 1.5, 3.0, 1.5, 45.0


def _body(room):
    from machine import physics
    physics.ROOM = float(room)
    from machine import Machine
    from machine.modes import DYNAMIC
    return Machine.discover('gynoid', execution_mode=DYNAMIC)


def _director(body):
    from machine.director import Director
    director = Director(body, stand_s=1e9)
    director.pace = -1.0
    director.begin(drop=0.004, stage='stand')
    body.loop.step(0.0)
    return director


def _run(body, director, until, each=None, done=None):
    """Her passes to `until` s, `each(bus, out)` before every one; stopped as `done(bus)` says
    or as she falls. Whether she stands."""
    bus = body.loop.bus
    while bus['t'] < until and director.stage != 'fallen':
        out = director.step(0.001)
        if each is not None:
            each(bus, out)
        body.loop.write(**out)
        body.loop.step(0.001)
        if done is not None and done(bus):
            break
    return director.stage != 'fallen'


def _law(director):
    """The law as the director has her (`machine.going.Going`), None before it takes her."""
    return getattr(director, 'going')


def _facing(bus):
    from machine import errands
    return math.degrees(errands.facing_of(bus))


def test_the_law_turns_walking(report):
    """Her walk's heading turned TURN_RAD_S walking for 13 s: she walks on, round 220 deg, her
    pelvis following her heading, her landing knee as straight as walking straight (10.7 deg,
    2026-10-11)."""
    from machine import errands
    rate = errands.TURN_RAD_S[0]
    body = _body(0)
    try:
        director = _director(body)

        def each(bus, _out):
            if bus['t'] >= WALKS_AT:
                director.pace = 0.0
            if _law(director) is not None and bus['t'] >= TURNED_AT:
                _law(director).heading += rate * 0.001

        up = _run(body, director, 16.0, each)
        bus, law = body.loop.bus, _law(director)
        steps = [r for r in law.steps if r['t'] >= TURNED_AT + 1.0]
        knee = sum(r['knee'] for r in steps) / max(1, len(steps))
        turned = math.degrees(law.heading)
        lag = abs((turned - _facing(bus) + 180.0) % 360.0 - 180.0)
        report.check('turned %.1f rad/s walking she walks 16 s on' % rate, up and len(steps) >= 15,
                     '%s, %d landings' % ('stands' if up else 'fell at %.1f s' % bus['t'], len(steps)))
        report.check('her heading came round 220 deg and her pelvis followed within 15',
                     turned > 220.0 and lag < 15.0, '%.0f deg, the pelvis %.0f behind' % (turned, lag))
        report.check('her landing knee stays under 15 deg, 20 at most', knee < 15.0
                     and max((r['knee'] for r in steps), default=0.0) < 20.0,
                     'mean %.1f, most %.1f deg' % (knee, max((r['knee'] for r in steps), default=0.0)))
    finally:
        body.close()


def test_she_steps_round_standing(report):
    """Standing on the law, her heading turned TURN_RAD_S standing for 9 s: she steps round
    where she stands, her pelvis following her heading, her feet within 0.3 m of where they
    were."""
    from machine import errands, pace
    rate = errands.TURN_RAD_S[1]
    body = _body(0)
    try:
        director = _director(body)
        start = {}

        def each(bus, out):
            if bus['t'] >= WALKS_AT and _law(director) is None and director.stage == 'stand':
                pace.take(director, bus, out)
                start['at'] = (bus['pelvis.pose.x'], bus['pelvis.pose.z'])
            if _law(director) is not None and bus['t'] >= TURNED_AT:
                _law(director).heading += rate * 0.001

        up = _run(body, director, 12.0, each)
        bus, law = body.loop.bus, _law(director)
        turned = math.degrees(law.heading)
        lag = abs((turned - _facing(bus) + 180.0) % 360.0 - 180.0)
        moved = math.dist(start['at'], (bus['pelvis.pose.x'], bus['pelvis.pose.z']))
        report.check('turned %.1f rad/s standing she stands, stepping round' % rate,
                     up and len(law.steps) >= 8, '%s, %d steps' % (
                         'stands' if up else 'fell at %.1f s' % bus['t'], len(law.steps)))
        report.check('her pelvis follows her heading within 20 deg', lag < 20.0,
                     '%.0f deg round, the pelvis %.0f behind' % (turned, lag))
        report.check('she stays within 0.3 m of where she stood', moved < 0.3, '%.2f m' % moved)
    finally:
        body.close()


def _errand(report, word, judge):
    """`word` asked from her stand in the room, given WORD_S; `judge(bus, world, director)`."""
    from machine import errands
    body = _body(1)
    try:
        director = _director(body)
        world = director.world

        def each(bus, _out):
            if bus['t'] >= ASKED_AT and director.errand is None and director.doing is None and (
                    bus['t'] < ASKED_AT + 0.002):
                errands.ask(director, word)

        def done(bus):
            return (bus['t'] > ASKED_AT + 1.0 and director.errand is None
                    and director.doing is None and director.k[0] <= -0.99)

        up = _run(body, director, ASKED_AT + WORD_S, each, done)
        bus = body.loop.bus
        report.check('asked %s she stands at the end' % word, up,
                     'stands at %.1f s' % bus['t'] if up else 'fell at %.1f s' % bus['t'])
        judge(bus, world, director)
    finally:
        body.close()


def test_out_through_the_door(report):
    """Asked out she walks to the door, pushes it open and stands beyond it."""
    from machine import room

    def judge(bus, world, _director):
        target = room.WORDS['out'][0]
        off = math.dist((bus['pelvis.pose.x'], bus['pelvis.pose.z']), target)
        report.check('she stands beyond the door, within 0.5 m of where out is', off < 0.5,
                     '%.2f m off, facing %.0f deg' % (off, _facing(bus)))
        report.check('the door is open past 60 deg', room.door_open(world) > 60.0,
                     '%.0f deg' % room.door_open(world))

    _errand(report, 'out', judge)


ROSTER = [test_out_through_the_door, test_the_law_turns_walking, test_she_steps_round_standing]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
