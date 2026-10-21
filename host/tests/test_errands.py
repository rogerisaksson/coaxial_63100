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


def test_chair_and_bed_seated_and_rising(report):
    """Seated on the chair by its text, the seat bears more of her than her soles do (`y[PROP]`
    over her soles' share) with both soles on the floor (`y[LOWEST]` within `leap.TOUCH_M` of
    it), as a human's are (tools/sim/seats.py), and told the rise she stands on her soles
    (`sum(loads) >= 0.85 * weight`); seated on the bed by the take's rows (`scenarios.STUNTS`)
    the bed bears more of her than her soles and she stays - her feet hang (a seat of 0.45 m
    over shanks of 0.455 with her thighs level) and no rise from it stands her yet
    (docs/TODO.md)."""
    from machine import leap, modal, physics, worker
    from tools.sim import scenarios
    physics.ROOM, physics.SIGHT = 0.0, 0.0

    def seated(scene, told):
        s = worker.session()
        worker.apply(s, {'scenario': scene})
        g = s['g']
        _run(s, 1.0)
        worker.apply(s, told)
        _run(s, 6.0)
        weight = float(g['fr'][modal.WEIGHT])
        down = tuple(bool(h <= leap.TOUCH_M) for h in g['y'][modal.LOWEST] - g['fr'][modal.FLOOR])
        return s, g, weight, float(g['y'][modal.PROP]), sum(g['loads']) / weight, down

    s, g, weight, prop_sat, soles_sat, down = seated('chair_behind', {'say': 'sit_chair'})
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand_t = _run(s, 6.0, until='stand')
    _run(s, 1.0)
    soles_end = sum(g['loads']) / weight
    report.check('chair: the seat bears %.2f of her, her soles %.2f, both on the floor %s; told to rise she stands (%.2f on soles)'
                 % (prop_sat, soles_sat, down, soles_end),
                 prop_sat > soles_sat and all(down) and stand_t is not None and soles_end >= 0.85 and s['fallen_at'] is None,
                 'prop=%.2f soles=%.2f down=%s stand=%s soles_end=%.2f fallen=%s'
                 % (prop_sat, soles_sat, down, stand_t, soles_end, s['fallen_at']))
    s, g, weight, prop_sat, soles_sat, down = seated('bed_behind', scenarios._told({'take': 'sitting_on_bed'}))
    report.check('bed: by the take the bed bears %.2f of her, her soles %.2f, and she stays on it' % (prop_sat, soles_sat),
                 prop_sat > soles_sat and s['fallen_at'] is None,
                 'prop=%.2f soles=%.2f down=%s fallen=%s' % (prop_sat, soles_sat, down, s['fallen_at']))


def test_viewport_f_errands_chair_bed_and_door(report):
    """Walking toward the chair, bed, or door shows `[F]` in the viewport, and clicking `[F]` makes her
    autonomously walk to the chair and sit, walk to the bed and lie down, or walk to the door and exit."""
    from types import SimpleNamespace as SN
    from machine import modal, physics, sprung, worker
    from terminal.views import scanner, show_humanoid, viewpoint
    sc = scanner.VisionScanner()
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    for label, hd0, walk_s, expected_prompt, goal_phase in (
        ('chair', 1.0, 0.6, '[F] sit chair', 'seated'),
        ('bed', -0.65, 1.0, '[F] lie bed', 'bed_rest'),
        ('door', 0.22, 1.5, '[F] exit door', 'stand'),
    ):
        s = worker.session()
        g = s['g']
        if walk_s > 0.0:
            worker.apply(s, {'heading': hd0, 'twist': WALK})
            _run(s, walk_s)
        else:
            g['heading'] = hd0
        now = worker.told(s)
        st = {'body': SN(send=lambda **cmd: worker.apply(s, cmd)), 'scanner': sc,
              'now': now, 'yaw': 180.0, 'tour': None, **viewpoint.HOME}
        shown_prompt = show_humanoid.prompt(st)
        viewpoint.mouse(st, lambda: (100, 40))['on_click'](20, 10)
        max_prop = 0.0
        for _ in range(40000):
            worker.stepped(s)
            max_prop = max(max_prop, float(g['y'][modal.PROP]))
            if s['fallen_at'] is not None:
                break
            if label == 'door' and g.get('_errand') is None and float(g['world'].data.qpos[2]) > 4.1 and sprung.doing(g) in ('stand', 'rest'):
                break
            if label != 'door' and sprung.doing(g) == goal_phase and max_prop > 0.45 and g['t'] > 9.0:
                break
        gx, gy, gz = (float(g['world'].data.qpos[i]) for i in range(3))
        if label == 'door':
            dq = float(g['world'].data.qpos[g['world'].model.jnt_qposadr[g['world'].model.joint('door').id]])
            ok = (shown_prompt == expected_prompt and gz > 4.1 and dq > 1.2 and s['fallen_at'] is None)
            detail = 'prompt=%r z=%.2f door=%.2f fallen=%s' % (shown_prompt, gz, dq, s['fallen_at'])
        else:
            sr_rest = float(g.get('surety', 1.0))
            ok = (shown_prompt == expected_prompt and sprung.doing(g) == goal_phase
                  and max_prop > 0.45 and gy < 0.72 and sr_rest > 0.65 and s['fallen_at'] is None)
            if label == 'bed' and ok:
                _run(s, 0.8)
                st['now'] = worker.told(s)
                rise_prompt = show_humanoid.prompt(st)
                viewpoint.mouse(st, lambda: (100, 40))['on_click'](20, 10)
                _flips, stood_t = _run(s, 6.0, until='stand')
                _run(s, 0.5)
                ok = (ok and rise_prompt == '[F] rise bed' and stood_t is not None
                      and sum(g['loads']) > 250.0 and float(g.get('surety', 1.0)) > 0.85 and s['fallen_at'] is None)
            detail = 'prompt=%r doing=%s pos=(%.2f,%.2f,%.2f) prop=%.2f sure=%.2f fallen=%s' % (
                shown_prompt, sprung.doing(g), gx, gy, gz, max_prop, float(g.get('surety', 1.0)), s['fallen_at'])
        report.check('viewport [F] click for %s: %s and completes autonomous errand' % (label, expected_prompt), ok, detail)


def test_multi_pose_bed_settle_and_randomized_scenarios(report):
    """From arbitrary starting poses and headings (including the recorded `(-0.62, 1.86, -1.50)` pose),
    `lie_bed` walks to the bed's edge, turns to face away, backs onto it with her heels within a
    sole's half-length of the edge and begins to sit, no fall; seeded randomized scenario
    perturbations stay upright without falling. Lying from the dock: scenarios.md item 1."""
    import random
    from machine import edge, figure, modal, physics, se2, sprung, worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    for name, x0, z0, yaw0 in (('csv_rec', -0.62, 1.86, -1.50), ('south_face', -0.80, 1.50, 3.10),
                               ('llm_rec', -0.37, 1.45, 2.365)):
        s = worker.session()
        g = s['g']
        d = g['world'].data
        sprung.relaid(g, (x0, z0), yaw0)
        _run(s, 0.4)
        worker.apply(s, {'say': 'lie_bed'})
        er = dict(g['_errand'])
        while g['t'] < 40.0 and s['fallen_at'] is None and g.get('_errand'):
            worker.stepped(s)
        docked_t, dx, dz = g['t'], float(d.qpos[0]) - er['x_dock'], float(d.qpos[2]) - er['z_dock']
        s_par, s_perp = dx * er['nx'] + dz * er['nz'], dx * er['tx'] + dz * er['tz']
        yaw_off = se2.wrap_angle(er['hdd'] - se2.wrap_angle(edge.yaw(g['sense'])))
        _flips, sat_t = _run(s, 3.0, until='sit_edge')
        ok = (g.get('_errand') is None and abs(s_par) <= figure.HEEL + figure.SOLE_HALF and abs(s_perp) <= er['lat_tol']
              and abs(yaw_off) <= modal.LIST_RAD and sat_t is not None and s['fallen_at'] is None)
        report.check('bed errand from %s (%.2f, %.2f, %.2f rad): docks at the edge facing away and begins to sit'
                     % (name, x0, z0, yaw0), ok,
                     'docked at %.1f s: heels %.2f m from the edge, %.2f m along it, yaw %.2f rad off; sits=%s fallen=%s'
                     % (docked_t, s_par, s_perp, yaw_off, sat_t, s['fallen_at']))
    rng = random.Random(63100)
    rand_ok = True
    rand_details = []
    for i in range(3):
        s = worker.session()
        scen = ('flat', 'bed', 'chair_behind')[i]
        worker.apply(s, {'scenario': scen})
        _run(s, 0.3)
        hd = rng.uniform(-0.8, 0.8)
        vx = rng.uniform(0.45, 0.69)
        worker.apply(s, {'heading': hd, 'twist': (vx, 0.0, 0.0)})
        _run(s, 1.2)
        worker.apply(s, {'twist': STOP})
        _flips, stood = _run(s, 3.0, until='stand')
        ok_i = stood is not None and s['fallen_at'] is None and float(s['g']['surety']) > 0.75
        rand_ok = rand_ok and ok_i
        rand_details.append((scen, round(hd, 2), round(vx, 2), stood, s['fallen_at']))
    report.check('seeded randomized scenario walk-and-stop perturbations stay upright and settle',
                 rand_ok, rand_details)


def test_continuous_outer_loop_told_and_walk_again(report):
    """Zero discrete delay tables in `se2`/`worker`; `W` at any moment of a stop reaches `> 0.3 m/s` in `< 1.0 s`."""
    from machine import se2, sprung, worker
    from tools.sim import told
    no_tables = not any(hasattr(se2, k) or hasattr(worker, k) for k in ('INIT_DELAYS', 'BRAKE_DELAYS', 'RISES'))
    by = {e[0]: e for e in sprung.entries()}
    r_horizons = (se2.rise_lead(by['rise_chair'][1]), se2.rise_lead(by['rise_bed'][1]), se2.rise_lead(by['rise_floor'][1]))
    report.check('zero discrete delay tables in se2/worker and continuous rise_lead from wire tables',
                 no_tables and r_horizons == (2.2, 3.6, 3.3), r_horizons)
    times = []
    fell = False
    for after in (0.0, 0.5, 1.0, 2.0, 3.0, 4.0):
        r = told.Run()
        r.tell(twist=told.WALK)
        r.go(3.0)
        r.tell(twist=told.STOP)
        r.go(after)
        r.tell(twist=told.WALK)
        dt = r.go(3.0, told.moving)
        times.append((after, dt))
        fell = fell or (r.s['fallen_at'] is not None)
    report.check('W at any moment of a stop reaches > 0.3 m/s within < 1.0 s without falling',
                 not fell and all(dt is not None and dt < 1.0 for _, dt in times), times)


ROSTER = (test_a_stop_settles_and_f_is_heard, test_a_halting_row_waits_for_a_swing_only_while_her_rhythm_turns,
          test_a_chair_a_hair_off_is_sat_on_and_risen_from, test_doorway_arm_yields_and_she_walks_on,
          test_chair_and_bed_seated_and_rising, test_viewport_f_errands_chair_bed_and_door,
          test_multi_pose_bed_settle_and_randomized_scenarios,
          test_continuous_outer_loop_told_and_walk_again)


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
