"""Her errands on her loop (`worker.session`): a walk and a stop that settles, F to sit and to
rise, a scene laid anew and a walk at once."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from views_kit import Report

WALK, STOP = (0.69, 0.0, 0.0), (0.0, 0.0, 0.0)


def _run(s, seconds, until=None, after=0.0):
    """`seconds` of passes: (the flips of her doing from `after` s on, the seconds to `until`'s
    doing, or None)."""
    from machine import sprung, worker
    g, last, flips, got, t0 = s['g'], None, 0, None, s['g']['t']
    for _ in range(int(round(seconds / worker.DT))):
        worker.stepped(s)
        doing = sprung.doing(g)
        if doing != last:
            flips, last = flips + (last is not None and g['t'] - t0 >= after), doing
            if got is None and doing == until:
                got = g['t'] - t0
    return flips, got


def _speed(s):
    v = s['g']['est']['vel']
    return math.hypot(float(v[0]), float(v[2]))


def test_a_stop_settles_and_f_is_heard(report):
    """Walked 2 s and told to stop, her doing comes to the stand within the brake's own 1.2 s
    and two of her rhythm's swings more, and holds - read between her law's passes it came at 1.2 s
    and went; told to sit she is seated within 5 s, told to rise she stands within 6 s; a scene
    laid anew with a walk asked at once, she walks at once."""
    from machine import floor, worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    _run(s, 0.5)
    worker.apply(s, {'twist': WALK})
    _run(s, 2.0)
    worker.apply(s, {'twist': STOP})
    flips, stood = _run(s, 4.0, until='stand', after=2.0)
    report.check('the stop: the stand within 2.3 s, under 10 flips of her doing from 2 s on',
                 stood is not None and stood < 2.3 and flips < 10 and _speed(s) < 0.2,
                 'stand after %s, %d flips, %.2f m/s at the end' % (stood, flips, _speed(s)))
    x, _, z = s['g']['est']['pos']
    floor.mocap(s['g']['world'], 'chair', (float(x), 0.0, float(z) - 0.30), math.pi)
    worker.apply(s, {'say': 'sit_chair'})
    _flips, seated = _run(s, 5.0, until='seated')
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand = _run(s, 6.0, until='stand')
    report.check('F sits her within 5 s and rises her within 6 s',
                 seated is not None and stand is not None, (seated, stand))
    worker.apply(s, {'scenario': 'bed'})
    worker.apply(s, {'twist': WALK})
    _flips, walking = _run(s, 1.0, until='init_walk')
    report.check('a scene laid anew, the walk asked at once begins within 0.1 s',
                 walking is not None and walking < 0.1, walking)


def test_a_halting_row_waits_for_a_swing_only_while_her_rhythm_turns(report):
    """A row halting her rhythm, due while her capture point asks a step, waits a pass while her
    rhythm turns; at rest no swing comes and it comes at once (`motion.held_back`): sitting, her
    centre of mass behind her heels, a seat asked was held for ever."""
    import numpy as np
    from machine import latent, modal, motion

    def waited(turns):
        tab = np.zeros((1, latent.TABLE))
        tab[0, latent.SAID][latent.GAIN], tab[0, latent.AT] = 1.0, 1.0
        g = {'tab': tab, 't': 1.0, 'a': np.zeros(modal.ASK), 'out': np.zeros(latent.OUT), 'labels': [(1.0, 'seated')]}
        g['a'][modal.NEED], g['out'][latent.TURNS] = 1.0, turns
        motion.held_back(g, 0.003)
        return float(g['tab'][0, latent.AT]) - 1.0
    report.check('a halting row waits a pass while her rhythm turns, none at rest',
                 waited(np.pi) > 0.0 and waited(0.0) == 0.0, (waited(np.pi), waited(0.0)))


def test_a_chair_a_hair_off_is_sat_on_and_risen_from(report):
    """The chair 0.4 um off where her stop left her: a sole let go as she sat, the other standing,
    rose where her folded legs put it and she was thrown off the chair at 4.1 s (`impedance._swings`);
    she sits within 5 s and rises within 6 s."""
    from machine import floor, worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    _run(s, 0.5)
    worker.apply(s, {'twist': WALK})
    _run(s, 2.0)
    worker.apply(s, {'twist': STOP})
    _run(s, 4.0)
    x, _, z = s['g']['est']['pos']
    floor.mocap(s['g']['world'], 'chair', (float(x) + 4e-7, 0.0, float(z) - 0.30), math.pi)
    worker.apply(s, {'say': 'sit_chair'})
    _flips, seated = _run(s, 5.0, until='seated')
    height = float(s['g']['world'].data.qpos[1])
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand = _run(s, 6.0, until='stand')
    report.check('the chair 0.4 um off, F sits her within 5 s and rises her within 6 s',
                 seated is not None and stand is not None, 'seated at %s s, her pelvis %.3f m up, stood at %s s'
                 % (seated, height, stand))


def test_doorway_arm_yields_and_she_walks_on(report):
    """Walking through the room's doorway (`DOOR_X = (0.3, 1.2)` at `z = 3.5 m`) from her initial lay
    near either jamb so her hand or forearm strikes the door frame, her arm yields backward (`<= -0.40 rad`)
    and she walks on past the doorway without braking to `stand`."""
    import numpy as np
    from machine import edge, physics, touch, worker
    for side_name, hd, sh_idx, arm_skin in (('right jamb', 0.148, 13, touch.ARMS[1]),
                                            ('left jamb', 0.265, 5, touch.ARMS[0])):
        physics.ROOM = 1.0
        s = worker.session()
        worker.apply(s, {'scenario': 'flat'})
        _run(s, 0.4)
        worker.apply(s, {'heading': hd, 'twist': WALK})
        g = s['g']
        min_sh, max_arm_f = 0.0, 0.0
        for _ in range(int(round(7.5 / worker.DT))):
            worker.stepped(s)
            min_sh = min(min_sh, float(g['s']['q'][sh_idx]))
            t_arr = g['sense'][edge.TOUCH].reshape(-1, 3)
            max_arm_f = max(max_arm_f, max(float(np.linalg.norm(t_arr[k])) for k in arm_skin))
        z_end = float(g['est']['pos'][2])
        report.check('doorway %s: arm touched (%.0f N), yielded (%.2f rad) and walked on past z=3.5m'
                     % (side_name, max_arm_f, min_sh),
                     max_arm_f > edge.TOUCHED_N and min_sh < -0.40 and z_end > 3.68
                     and g.get('active_entry') == 'walk' and s['fallen_at'] is None,
                     'z_end=%.2fm min_sh=%.2f max_f=%.1fN entry=%s fallen=%s'
                     % (z_end, min_sh, max_arm_f, g.get('active_entry'), s['fallen_at']))


def test_chair_and_bed_dangling_legs_and_rising(report):
    """Seated on a chair (`sit_chair`) or reclined on a bed (`lie_bed`), her prop bears her weight
    (`y[PROP] > 0.85`), both soles leave stance (`stance == (False, False)`), and on `rise_chair` /
    `rise_bed` she plants her feet and rises to stand on her soles (`sum(loads) >= 0.85 * weight`)."""
    from machine import modal, physics, worker
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    for kind, scene, sit_cmd, sit_phase, rise_cmd in (
        ('chair', 'chair_behind', 'sit_chair', 'seated', 'rise_chair'),
        ('bed', 'bed_behind', 'lie_bed', 'bed_rest', 'rise_bed'),
    ):
        s = worker.session()
        worker.apply(s, {'scenario': scene})
        g = s['g']
        _run(s, 1.0)
        worker.apply(s, {'say': sit_cmd})
        _flips, sat_t = _run(s, 6.0, until=sit_phase)
        stance_sat = tuple(bool(v) for v in g['ask']['stance'])
        prop_sat = float(g['y'][modal.PROP])
        loads_sat = tuple(round(float(v), 1) for v in g['loads'])
        worker.apply(s, {'say': rise_cmd})
        _flips, stand_t = _run(s, 6.0, until='stand')
        _run(s, 1.0)
        weight = float(g['fr'][modal.WEIGHT])
        soles_end = sum(g['loads']) / weight
        report.check('%s: borne on prop (%.2f) with soles out of stance %s, then rises to stand (%.2f on soles)'
                     % (kind, prop_sat, stance_sat, soles_end),
                     sat_t is not None and stand_t is not None and stance_sat == (False, False)
                     and prop_sat > 0.85 and soles_end >= 0.85 and s['fallen_at'] is None,
                     'sat=%s loads=%s prop=%.2f stand=%s soles=%.2f fallen=%s'
                     % (sat_t, loads_sat, prop_sat, stand_t, soles_end, s['fallen_at']))


ROSTER = (test_a_stop_settles_and_f_is_heard, test_a_halting_row_waits_for_a_swing_only_while_her_rhythm_turns,
          test_a_chair_a_hair_off_is_sat_on_and_risen_from, test_doorway_arm_yields_and_she_walks_on,
          test_chair_and_bed_dangling_legs_and_rising)


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
