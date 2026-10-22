"""The HUMANOID page's floor: her horizons, an externality each; the arrows of her commands."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_her_horizon(report):
    """Z draws her horizon from her law's reading, which her state carries: an externality each,
    the floor a leg reaches and the room her law allows it, each arm's band; Z cycles the modes."""
    import math
    import numpy as np
    from coaxial.graphics import gynoid, horizon
    from machine import reach, sprung, stride
    from machine.figure import ANKLE_H
    from terminal.views import show_humanoid, viewpoint
    z3 = np.zeros(27)
    seen = sprung.state(sprung.laid(every=3), z3, z3, 0.0, 1.0, None)['horizon']
    report.check('her state: centre of mass, capture point under it, soles, both standing, her shoulders\' range',
                 abs(seen['xi'][0] - seen['com'][0]) < 0.02 and abs(seen['xi'][1] - seen['com'][2]) < 0.02
                 and len(seen['soles']) == 2 and seen['stance'] == (1.0, 1.0) and seen['shoulder'][0] < 0.0 < seen['shoulder'][1], seen)
    hip = (0.095, 0.80, 0.0)
    ring = horizon.reachable(0, hip, 0.0)
    far = max(math.hypot(a[0] - hip[0], a[2] - hip[2]) for a, _b in ring)
    turn, floor = horizon._turn(0.0), horizon.surface_at(hip[0], hip[2], ())[0]
    on = all(min(reach.margins(reach.over(0, hip, (0.98 * (a[0] - hip[0]) + hip[0], floor + ANKLE_H, 0.98 * (a[2] - hip[2]) + hip[2]),
                                          turn))) >= 0.0 for a, _b in ring)
    straight = horizon.reachable(0, (0.095, ANKLE_H + reach.THIGH + reach.SHANK, 0.0), 0.0)
    report.check('the floor her left leg reaches from a hip 0.80 m up, a double support\'s: a ring of %d, as far as %.2f m, '
                 'each point within her stops; from a straight leg\'s, under its sole' % (len(ring), far),
                 len(ring) == horizon.RING_N and 0.2 < far < stride.REACH_M and on
                 and max(math.hypot(a[0] - 0.095, a[2]) for a, _b in straight) < 0.1)
    arm = horizon.band((0.148, 1.345, 0.0), 0.0, -1.13, 2.62)
    report.check("an arm's band: two arcs of %d and their ends, the outer the arm's length from the shoulder" % horizon.RING_N,
                 len(arm) == 2 * horizon.RING_N + 2 and abs(math.dist(arm[0][0], (0.148, 1.345, 0.0)) - (horizon.UPPER_ARM + horizon.FOREARM)) < 1e-9)
    xs = [pt[0] for seg in horizon.room((0.0, 0.0), (-0.1, 0.0), 1.0, 0.0) for pt in seg]
    report.check("the left sole's room: REACH_M about her centre of mass, APART_M past the right",
                 abs(max(xs) - stride.REACH_M) < 1e-9 and abs(min(xs)) < 1e-9 and not horizon.room(
                     (0.0, 0.0), (0.5, 0.0), 1.0, 0.0), (min(xs), max(xs)))
    now = {'heading': 0.0, 'where': (0.0, 0.855, 0.0), 'turn': (1, 0, 0, 0), 'angles': {},
           'horizon': {'com': (0.0, 0.85, 0.0), 'xi': (0.05, 0.1), 'soles': ((0.1, 0.0, 0.0), (-0.1, 0.0, 0.0)),
                       'stance': (1.0, 0.0)}}
    got = horizon.segments(now)
    report.check("all: each leg's reach dim, a standing sole's room dim, a swinging one's bright, capture point, arms",
                 [ink for _segs, ink in got] == [horizon.REACH_INK, horizon.REACH_INK, horizon.ROOM_INK, horizon.SWING_INK,
                                                  horizon.CAPTURE_INK, horizon.ARM_INK, horizon.ARM_INK], [ink for _s, ink in got])
    report.check('soles alone, arms alone, off',
                 horizon.ARM_INK not in [ink for _s, ink in horizon.segments(now, show='soles')]
                 and {ink for _s, ink in horizon.segments(now, show='arms')} == {horizon.ARM_INK}
                 and horizon.segments(now, show='off') == [])
    st = {**viewpoint.HOME, 'orbit': False}
    modes = []
    for _ in range(4):
        show_humanoid.KEYS['z'](st)
        modes.append(st['horizon'])
    show_humanoid.KEYS['Z'](st)
    report.check('z cycles soles, arms, all, off; Z back; and it draws on her',
                 modes == ['soles', 'arms', 'all', 'off'] and st['horizon'] == 'all' and gynoid.render(
                     {}, 64, 28, colour=False) != gynoid.render({}, 64, 28, colour=False, horizon=got), modes)


def test_surface_arrows_setpoint_actual_and_landing_zones(report):
    """The arrows on the floor: asked, done, a hop, a stop, a fall."""
    import numpy as np
    from coaxial.graphics import arrows, gynoid
    from machine import physics, sprung
    from terminal.views import humanoid_keys
    z3 = np.zeros(27)
    b0, w = sprung.state(sprung.laid(every=3), z3, z3, 0.0, 1.0, None), physics.World()
    w.scenario('slope')
    y0, _ = arrows.surface_at(0.0, 1.2, w.props())
    y1, n1 = arrows.surface_at(0.0, 2.8, w.props())
    report.check('the arrow on the slope', y1 > y0 + 0.03 and n1[2] < -0.01)
    w.scenario('hole')
    report.check('and in the hole', arrows.surface_at(0.0, 0.55, w.props())[0] < -0.08)

    def held(word, setpoint, actual=(0.0, 0.0, 0.0)):
        keys = dict({k: k in word.lower() for k in 'wsadqe'}, shift=word != word.lower())
        return dict(humanoid_keys.pad(), keys=keys, setpoint=setpoint, actual=actual)

    nw0 = dict(b0, twist=(0.69, 0.0, 0.0), velocity=(0.0, 0.02), speed=0.02, doing='walk')
    sw0 = arrows.describe(nw0, held('w', (0.69, 0.0, 0.0)))
    sw_mid = arrows.describe(dict(nw0, velocity=(0.0, 0.35), speed=0.35), held('w', (0.69, 0.0, 0.0)))
    sw_blk = arrows.describe(dict(nw0, velocity=(0.0, 0.0), speed=0.0), held('w', (0.69, 0.0, 0.0)))
    sw_rev = arrows.describe(dict(nw0, velocity=(0.0, -0.25), speed=-0.25), held('w', (0.69, 0.0, 0.0)))
    nw_fast = dict(b0, twist=(1.4, 0.0, 0.0), velocity=(0.0, 1.35), speed=1.35, doing='jog')
    sw1 = arrows.describe(nw_fast, held('W', (1.4, 0.0, 0.0)))
    report.check('W / Shift+W arrow hollow at first, empty blocked, filled by speed',
                 sw0['kind'] == 'locomotion' and abs(sw0['origin'][0]) < 0.05 and sw_blk['fill'] == 0.0 and sw_rev['fill'] == 0.0
                 and 0.0 < sw0['fill'] < 0.20 < sw_mid['fill'] < 0.85 < sw1['fill'] <= 1.0 and sw1['setpoint'][0] > sw0['setpoint'][0]
                 and len(sw0['frame_segments']) > 10 and len(sw1['fill_segments']) > len(sw0['fill_segments']))

    swa = arrows.describe(dict(b0, doing='turn_left', heading=0.35, velocity=(0.0, 0.69), twist_actual=(0.69, 0.0, 0.70)),
                          held('wa', (0.69, 0.0, 0.7)))
    sq_side = arrows.describe(dict(b0, doing='strafe_left', twist_actual=(0.0, 0.45, 0.0)), held('a', (0.0, 0.5, 0.0)))
    s_spin = arrows.describe(dict(b0, doing='turn_left'), held('q', (0.0, 0.0, 0.7), (0.0, 0.0, 0.6)))
    mx = lambda sp, i: max(p[i] for s in sp['frame_segments'] for p in s)
    report.check('an arc left, a sidestep and a turn in place render cleanly',
                 swa['kind'] == 'locomotion' and mx(swa, 0) > 0.15 and swa['setpoint'][1] == 0.0
                 and abs(swa['pelvis_yaw']) < 1e-6 and 0.10 < swa['heading'] < 0.15
                 and swa['fill_segments'][-1][0] == swa['frame_segments'][-1][1]
                 and arrows.pelvis_yaw({'turn': (1, 0, 0, 0), 'heading': 0.8}) == 0.0
                 and sq_side['kind'] == 'locomotion' and mx(sq_side, 0) > 0.25
                 and s_spin['kind'] == 'locomotion' and len(s_spin['frame_segments']) > 10 and s_spin['fill'] > 0.5)

    shop = arrows.describe(dict(b0, doing='throw', where=(0.0, 1.02, 0.0)))
    report.check('a hop: an arrow up, framed and filled',
                 shop['kind'] == 'jump' and mx(shop, 1) > 0.75 and len(shop['fill_segments']) > 5)

    sstop = arrows.describe(dict(b0, doing='brake', velocity=(0.0, 0.45), speed=0.45))
    report.check('a stop: its bar and its arrow',
                 sstop['kind'] == 'stop' and len(sstop['target_segments']) >= 8 and sstop['landing'][2] > 0.25)

    sfall = arrows.describe(dict(b0, doing='controlled_fall', velocity=(0.35, 0.50), where=(0.0, 0.82, 0.0)))
    report.check('a fall: its touch-down and its ring',
                 sfall['kind'] == 'fall' and sfall['landing'] is not None and sfall['landing'][2] > 0.20 and len(sfall['target_segments']) >= 16)

    report.check('the arrows drawn on her',
                 gynoid.render(b0['angles'], 64, 28, colour=False)
                 != gynoid.render(b0['angles'], 64, 28, colour=False, arrows=(nw_fast, held('W', (1.4, 0.0, 0.0)))))


def test_balloons_on_what_she_sees(report):
    """Each thing she has seen a balloon's offer, the one she faces first (`gym.offered`), keyed
    F1 on - one alone too; a chair's seat and back one thing, a mattress out of her reach none,
    what bears her alone; turned away still offered, laid anew forgotten -; F opens the choice, a
    digit within `console.HOLD_WINDOW_S` says that one at its thing, F again or the window lapsed
    the first's, the function key at once, a click the first's; a digit with no F the gym's
    entry; a balloon on the viewport over each."""
    import math
    import time
    from types import SimpleNamespace as SN
    from terminal.ui import console
    from terminal.views import gym, scanner, show_humanoid, viewpoint
    up = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    room = [('wood', (1.5, 0.45, 0.8), (0.22, 0.02, 0.22), up), ('bed', (-1.8, 0.35, 1.8), (0.5, 0.12, 1.0), up),
            ('door', (0.75, 1.05, 3.5), (0.45, 1.05, 0.03), up)]
    sent = []
    st = {'body': SN(send=lambda **kw: sent.append(kw)), 'scanner': scanner.VisionScanner(), 'tour': None,
          **viewpoint.HOME, 'now': {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': room}}
    shown = [(key, verb) for key, verb, *_rest in gym.offered(st)]
    report.check('the door ahead, the bed and the chair aside she sees, the one she faces first, F1 F2 F3',
                 shown == [('F1', 'exit_door'), ('F2', 'lie_bed'), ('F3', 'sit_chair')]
                 and gym.prompt(st) == '[F1] exit door  [F2] lie bed  [F3] sit chair', (shown, gym.prompt(st)))
    for typed in ('f', '3', 'ff', ('f2',), 'f'):
        for k in typed:
            show_humanoid.KEYS[k](st)
    opened = st.get('choosing') is not None
    st['choosing'] -= 2.0 * console.HOLD_WINDOW_S
    st['f_at'] -= 2.0 * console.HOLD_WINDOW_S
    gym.lapsed(st, time.monotonic())
    show_humanoid.KEYS['2'](st)
    viewpoint.mouse(st, lambda: (100, 40))['on_click'](20, 10)
    said = [(c['say'], tuple(round(v, 9) for v in c.get('at', ()))) for c in sent]
    door = ('exit_door', (0.75, 3.5))
    report.check('F then 3 the chair, F F the door at once, F2 the bed, F lapsed the door, 2 with no F the '
                 "gym's second, a click the door",
                 opened and said == [('sit_chair', (1.5, 0.8)), door, ('lie_bed', (-1.8, 1.8)), door, (1, ()), door],
                 said)
    st['now'] = dict(st['now'], heading=math.pi)
    behind = [o[1] for o in gym.offered(st)]
    st['now'] = dict(st['now'], epoch=1)
    report.check('turned away, what she has seen still offered; laid anew, none of it',
                 sorted(behind) == ['exit_door', 'lie_bed', 'sit_chair'] and gym.offered(st) == [], behind)
    st['now'] = {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': room + [
        ('chair', (0.0, 0.43, 0.88), (0.22, 0.02, 0.22), up), ('chair', (0.0, 0.65, 1.10), (0.22, 0.2, 0.02), up)]}
    ahead = gym.offered(st)
    st['now'] = {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'scenario': 'mattress',
                 'props': [('bed', (0.0, 0.08, 2.5), (0.45, 0.08, 0.95), up)]}
    far = gym.offered(st)
    st['now'] = dict(st['now'], props=[('bed', (0.0, 0.08, 0.6), (0.45, 0.08, 0.95), up)])
    near = gym.offered(st)
    st['now'] = {'where': (1.5, 0.6, 0.8), 'heading': 0.0, 'doing': 'seated', 'props': room}
    report.check('a chair ahead, seat and back, one thing first; a mattress out of her reach none, in it '
                 'lie_floor; seated, her rise alone',
                 [o[1] for o in ahead] == ['sit_chair', 'exit_door', 'lie_bed', 'sit_chair'] and abs(ahead[0][3][0]) < 1e-9
                 and far == [] and [o[1] for o in near] == ['lie_floor']
                 and [o[:2] for o in gym.offered(st)] == [('F1', 'rise_chair')], (ahead, far, near))
    st['now'] = {'where': (0.0, 0.9, 0.0), 'heading': 0.0, 'props': room}
    drawn = gym.ballooned(st, gym.offered(st), 110, 36, (0.0, 0.0))
    text = ''.join(chr(code) for _at, (code, _key) in sorted(drawn.items()))
    report.check('a balloon on the viewport over each', all(w in text for w in ('F1 go out', 'F2 lie down', 'F3 sit')),
                 text)


ROSTER = (test_her_horizon, test_surface_arrows_setpoint_actual_and_landing_zones, test_balloons_on_what_she_sees)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
