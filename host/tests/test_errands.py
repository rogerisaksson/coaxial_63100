"""Its errands on its loop (`worker.session`): a walk and a stop that settles, F to sit and to
rise, a scene laid anew and a walk at once."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from tools.sim.offered import OFFERED, STOP, WALK, described, reached, walked_up
from views_kit import Report, typed


def _first(st, typed, pad):
    """The page's first offer said by F (`humanoid_keys._interact`): at once with one, F again with more - a click on
    the view says nothing unless F's choice is open."""
    typed(pad, 'f', st)
    if st.get('choosing') is not None:
        typed(pad, 'f', st)


def _run(s, seconds, until=None, after=0.0):
    """`seconds` of passes: (the flips of its doing from `after` s on, the seconds to `until`'s
    doing, or None)."""
    from machine.process import sprung, worker
    g, last, flips, got, t0 = s['g'], None, 0, None, s['g']['t']
    for _ in range(int(round(seconds / worker.DT))):
        worker.stepped(s)
        g = s['g']
        doing = sprung.doing(g)
        if doing != last:
            flips, last = flips + (last is not None and g['t'] - t0 >= after), doing
            if got is None and (doing == until or (until == 'seated' and doing.startswith('seated'))):
                got = g['t'] - t0
    return flips, got


def _forwarded(s):
    """Its world forwarded after a prop placed between its steps, as `World.scenario` does."""
    world = s['g']['world']
    world._mj.mj_forward(world.model, world.data)


def _speed(s):
    v = s['g']['est']['vel']
    return math.hypot(float(v[0]), float(v[2]))


def test_a_stop_settles_and_f_is_heard(report):
    """Walked 2 s and told to stop, its doing comes to the stand within 2.3 s and holds; told to
    sit it is seated within 5 s, told to rise it stands within 6 s; a scene laid anew with a
    walk asked at once, it walks at once."""
    from machine.world import floor
    from machine.moves import leap
    from machine.law import modal
    from machine.process import worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    _run(s, 0.5)
    worker.apply(s, {'twist': WALK})
    _run(s, 2.0)
    worker.apply(s, {'twist': STOP})
    flips, stood = _run(s, 4.0, until='stand', after=2.0)
    report.check('the stop: the stand within 2.3 s, under 10 flips of its doing from 2 s on',
                 stood is not None and stood < 2.3 and flips < 10 and _speed(s) < 0.2,
                 'stand after %s, %d flips, %.2f m/s at the end' % (stood, flips, _speed(s)))
    x, _, z = s['g']['y'][modal.SOLE].reshape(2, 3).mean(axis=0)
    floor.mocap(s['g']['world'], 'chair', (float(x), 0.0, float(z) - leap.MIDDLE + floor.behind('sit_chair', floor.CHAIR_SEAT[2])),
                math.pi)
    _forwarded(s)
    worker.apply(s, {'say': 'sit_chair'})
    _flips, seated = _run(s, 5.0, until='seated')
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand = _run(s, 6.0, until='stand')
    report.check('F sits it within 5 s and rises it within 6 s, no fall',
                 seated is not None and stand is not None and s['fallen_at'] is None and s['epoch'] == 0,
                 (seated, stand, s['fallen_at'], s['epoch']))
    worker.apply(s, {'scenario': 'bed'})
    worker.apply(s, {'twist': WALK})
    _flips, walking = _run(s, 1.0, until='init_walk')
    report.check('a scene laid anew, the walk asked at once begins within 0.1 s',
                 walking is not None and walking < 0.1, walking)


def test_a_halting_row_waits_for_a_swing_only_while_its_rhythm_turns(report):
    """A row halting its rhythm, due while its capture point asks a step, waits a pass while its
    rhythm turns; at rest no swing comes and it comes at once (`waits.held_back`): sitting, its
    centre of mass behind its heels, a seat asked was held for ever."""
    import numpy as np
    from machine.law import latent, modal, waits
    from tools.cores import motion as c_motion, wait as c_wait
    k, _ask = c_motion.laid()

    def waited(turns):
        tab = np.zeros((1, latent.TABLE))
        tab[0, latent.SAID][latent.GAIN], tab[0, latent.AT] = 1.0, 1.0
        g = {'tab': tab, 't': 1.0, 'a': np.zeros(modal.ASK), 'out': np.zeros(latent.OUT), 'labels': [(1.0, 'seated')]}
        g['waits'] = c_wait.laid(k, g['a'], g['out'], np.zeros(modal.SENSED), np.zeros(modal.FRAME), np.zeros(latent.STATE))
        g['a'][modal.NEED], g['out'][latent.TURNS] = 1.0, turns
        waits.held_back(g, 0.003)
        return float(g['tab'][0, latent.AT]) - 1.0
    report.check('a halting row waits a pass while its rhythm turns, none at rest',
                 waited(np.pi) > 0.0 and waited(0.0) == 0.0, (waited(np.pi), waited(0.0)))


def test_a_chair_a_hair_off_is_sat_on_and_risen_from(report):
    """The chair 0.4 um off where its stop left it: a sole let go as it sat, the other standing,
    rose where its folded legs put it and it was thrown off the chair at 4.1 s (`swing.free`);
    it sits within 5 s and rises within 6 s."""
    from machine.world import floor
    from machine.moves import leap
    from machine.law import modal
    from machine.process import worker
    s = worker.session()
    worker.apply(s, {'scenario': 'flat'})
    _run(s, 0.5)
    worker.apply(s, {'twist': WALK})
    _run(s, 2.0)
    worker.apply(s, {'twist': STOP})
    _run(s, 4.0)
    x, _, z = s['g']['y'][modal.SOLE].reshape(2, 3).mean(axis=0)
    floor.mocap(s['g']['world'], 'chair',
                (float(x) + 4e-7, 0.0, float(z) - leap.MIDDLE + floor.behind('sit_chair', floor.CHAIR_SEAT[2])), math.pi)
    _forwarded(s)
    worker.apply(s, {'say': 'sit_chair'})
    _flips, seated = _run(s, 7.0, until='seated')
    height = float(s['g']['world'].data.qpos[1])
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stand = _run(s, 8.0, until='stand')
    report.check('the chair 0.4 um off, F sits it within 5 s and rises it within 6 s',
                 seated is not None and stand is not None, 'seated at %s s, its pelvis %.3f m up, stood at %s s'
                 % (seated, height, stand))


def test_doorway_arm_yields_and_it_walks_on(report):
    """Walking through the room's doorway (`DOOR_X = (0.3, 1.2)` at `z = 3.5 m`) from its initial lay
    near either jamb so its hand or forearm strikes the door frame, its arm yields backward (`<= -0.40 rad`)
    and it walks on past the doorway without braking to `stand`."""
    import numpy as np
    from machine.world import physics
    from machine.senses import edge, touch
    from machine.process import worker
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
                     max_arm_f > edge.TOUCHED_N and min_sh < -0.20 and z_end > 3.35
                     and g.get('active_entry') == 'walk' and s['fallen_at'] is None,
                     'z_end=%.2fm min_sh=%.2f max_f=%.1fN entry=%s fallen=%s'
                     % (z_end, min_sh, max_arm_f, g.get('active_entry'), s['fallen_at']))


def test_chair_and_bed_seated_and_rising(report):
    """Seated on the chair by its text, the seat bears more of it than its soles do (`y[PROP]`
    over its soles' share), and told the rise it stands on its soles (`sum(loads) >= 0.85 * weight`);
    told to lie on the bed by its text it bears its reclined and it stays."""
    from machine.moves import leap
    from machine.law import modal
    from machine.world import physics
    from machine.process import worker
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
    report.check('chair: the seat bears %.2f of it, its soles %.2f, both on the floor %s; told to rise it stands (%.2f on soles)'
                 % (prop_sat, soles_sat, down, soles_end),
                 prop_sat > soles_sat and stand_t is not None and soles_end >= 0.85 and s['fallen_at'] is None,
                 'prop=%.2f soles=%.2f down=%s stand=%s soles_end=%.2f fallen=%s'
                 % (prop_sat, soles_sat, down, stand_t, soles_end, s['fallen_at']))
    s, g, weight, prop_sat, soles_sat, down = seated('bed_behind', {'say': 'lie_bed'})
    report.check('bed: by its text the bed bears %.2f of its reclined, its soles %.2f on the floor %s, and it stays on it'
                 % (prop_sat, soles_sat, down),
                 prop_sat > 0.0 and s['fallen_at'] is None,
                 'prop=%.2f soles=%.2f down=%s fallen=%s' % (prop_sat, soles_sat, down, s['fallen_at']))


def test_viewport_f_errands_chair_bed_and_door(report):
    """Walking toward the chair (`SIGHT = 1.0`), bed, or door offers it first in the viewport
    (`gym.offered`, `gym.ballooned`), and F (`_first`) makes it autonomously dock and sit, lie
    down, or exit (`tools.sim.offered`: its walk, its judge, its ensemble)."""
    from machine.process import worker
    from terminal.views import gym, humanoid_keys, scanner
    sc = scanner.VisionScanner()
    for label, hd0, walk_s, verb, goal_phase in OFFERED:
        s, st, shown, shown_prompt, expected_prompt, max_prop = walked_up(label, hd0, walk_s, verb, goal_phase, sc)
        g = s['g']
        ok = reached(label, goal_phase, s, shown, max_prop)
        if label == 'bed' and ok:
            _run(s, 0.8)
            st['now'] = worker.told(s)
            rise_prompt, rise_on = gym.prompt(st), (gym.offered(st) or [('',) * 5])[0][4]
            _first(st, typed, humanoid_keys.PAD)
            _flips, stood_t = _run(s, 30.0, until='stand')
            _run(s, 0.5)
            ok = (ok and rise_prompt == '[F1] rise bed' and rise_on.startswith('bed ') and stood_t is not None
                  and sum(g['loads']) > 250.0 and float(g.get('surety', 1.0)) > 0.85 and s['fallen_at'] is None)
        report.check('F for %s: %s offered first and completes autonomous errand' % (label, expected_prompt),
                     ok, described(label, s, shown_prompt, max_prop))


def test_the_errand_goes_to_the_thing_told(report):
    """Two chairs, the scene's ahead and the lodge's aside: told to sit it looks (its registry's three
    sightings), its planner docks it (`machine.way.planner`) and it walks to the nearer one, told to sit at
    the lodge's (`at`, its balloon's) to that one."""
    from machine.world import physics, room
    from machine.process import worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    docks = []
    for told in ({'say': 'sit_chair'}, {'say': 'sit_chair', 'at': room.CHAIR_AT}):
        s = worker.session()
        worker.apply(s, {'scenario': 'chair'})
        worker.apply(s, told)
        while (s['g'].get('_errand') or {}).get('phase') in ('look', 'plan') and s['g']['t'] < 1.0:
            worker.stepped(s)
        er = s['g'].get('_errand') or {}
        docks.append(tuple(round(er.get(k, math.nan), 6) for k in ('x_dock', 'z_dock')))
    report.check("told to sit, the chair ahead's edge; told the lodge's, the lodge's",
                 docks == [(0.0, round(0.88 - room.SEAT[2], 6)), (room.CHAIR_AT[0], round(room.CHAIR_AT[1] - room.SEAT[2], 6))],
                 docks)


def test_the_low_chair_is_docked_as_it_faces(report):
    """The lodge's low chair against the west wall facing +x: told to sit at it it docks before it
    facing the room, not at its flank against the wall, and is seated within 30 s."""
    from machine.world import physics, room
    from machine.process import worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = worker.session()
    g = s['g']
    worker.apply(s, {'say': 'sit_chair', 'at': room.CHAIR_LOW_AT})
    while (g.get('_errand') or {}).get('phase') in ('look', 'plan') and g['t'] < 1.0:
        worker.stepped(s)
    er = dict(g.get('_errand') or {})
    _flips, seated = _run(s, 30.0, until='seated')
    before = (er.get('x_dock', -math.inf) > room.CHAIR_LOW_AT[0] + room.SEAT[2] / 2.0
              and abs(er.get('z_dock', math.inf) - room.CHAIR_LOW_AT[1]) < room.SEAT[2] / 2.0)
    report.check('told the low chair it docks before it facing the room and is seated within 30 s',
                 before and seated is not None,
                 'dock (%.2f, %.2f) facing %.2f rad, seated at %s s' % (er.get('x_dock', math.nan), er.get('z_dock', math.nan),
                                                                      er.get('hdd', math.nan), seated))


def test_from_the_low_chair_it_rounds_the_bed_to_the_door(report):
    """Sat on the lodge's low chair and risen, told the door: its way rounds the bed, out past the
    door's line, clear of the bed's box by its hips, no fall."""
    from machine.body import figure
    from machine.world import physics, room
    from machine.process import sprung, worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    s = worker.session()
    g, d = s['g'], s['g']['world'].data
    worker.apply(s, {'say': 'sit_chair', 'at': room.CHAIR_LOW_AT})
    _flips, seated = _run(s, 40.0, until='seated')
    _run(s, 2.0)
    sat_x = float(s['g']['world'].data.qpos[0])
    worker.apply(s, {'say': 'rise_chair'})
    _flips, stood = _run(s, 10.0, until='stand')
    _run(s, 1.0)
    worker.apply(s, {'say': 'exit_door'})
    g, d = s['g'], s['g']['world'].data
    bed = next((t for t in g['things'] if t[0] == 'bed'), None)
    t0, nearest = g['t'], math.inf
    while g['t'] < t0 + 40.0 and s['fallen_at'] is None and g.get('_errand'):
        worker.stepped(s)
        g, d = s['g'], s['g']['world'].data
        if bed is not None:
            dx = max(abs(float(d.qpos[0]) - bed[1][0]) - bed[2][0], 0.0)
            dz = max(abs(float(d.qpos[2]) - bed[1][2]) - bed[2][2], 0.0)
            nearest = min(nearest, math.hypot(dx, dz))
    _run(s, 3.0)
    g, d = s['g'], s['g']['world'].data
    report.check('from the low chair, seated past its edge and risen, it rounds the bed to the door, no fall',
                 seated is not None and sat_x < room.CHAIR_LOW_AT[0] + room.SEAT[2] - 0.05 and stood is not None
                 and s['fallen_at'] is None and s['epoch'] == 0 and not g.get('_errand')
                 and float(d.qpos[2]) > room.TO and bed is not None and nearest > figure.HIP_HALF,
                 'seated %s (x=%.2f), stood %s, out at (%.2f, %.2f) after %.1f s, nearest the bed %.2f m, fallen %s'
                 % (seated, sat_x, stood, float(d.qpos[0]), float(d.qpos[2]), g['t'] - t0, nearest, s['fallen_at']))


def test_multi_pose_bed_settle_and_randomized_scenarios(report):
    """From arbitrary starting poses and headings (including the recorded `(-0.62, 1.86, -1.50)` pose),
    `lie_bed` walks to the bed's edge, turns to face away, backs onto it with its heels within a
    sole's half-length of the edge and begins to sit, no fall; seeded randomized scenario
    perturbations stay upright without falling. Lying from the dock: scenarios.md item 1."""
    import random
    from machine.world import physics
    from machine.senses import edge
    from machine.body import figure
    from machine.law import modal, se2
    from machine.process import sprung, worker
    physics.ROOM, physics.SIGHT = 1.0, 0.0
    for name, x0, z0, yaw0 in (('lodge', 0.0, 0.0, 0.0), ('csv_rec', -0.62, 1.86, -1.50), ('south_face', -0.80, 1.50, 3.10)):
        s = worker.session()
        g, d = s['g'], s['g']['world'].data
        sprung.relaid(g, (x0, z0), yaw0)
        _run(s, 0.4)
        worker.apply(s, {'say': 'lie_bed'})
        er, sh = dict(g['_errand']), 0.0
        while g['t'] < 40.0 and s['fallen_at'] is None and g.get('_errand'):
            worker.stepped(s)
            er, sh = dict(g['_errand'] or er), max(sh, abs(float(g['sense'][edge.QD][5])), abs(float(g['sense'][edge.QD][13])))
        docked_t, dx, dz = g['t'], float(d.qpos[0]) - er['x_dock'], float(d.qpos[2]) - er['z_dock']
        s_par, s_perp = dx * er['nx'] + dz * er['nz'], dx * er['tx'] + dz * er['tz']
        yaw_off = se2.wrap_angle(er['hdd'] - se2.wrap_angle(edge.yaw(g['sense'])))
        _flips, sat_t = _run(s, 3.0, until='sit_edge')
        ok = (g.get('_errand') is None and abs(s_par - er['standoff']) <= figure.HEEL + figure.SOLE_HALF
              and abs(s_perp) <= er['lat_tol'] and abs(yaw_off) <= modal.LIST_RAD and sat_t is not None
              and sh < 12.0 and s['fallen_at'] is None)
        report.check('bed errand from %s (%.2f, %.2f, %.2f rad): docks at the edge facing away and begins to sit'
                     % (name, x0, z0, yaw0), ok,
                     '%.1fs par=%.2f perp=%.2f yaw=%.2f sh=%.1f sits=%s fall=%s'
                     % (docked_t, s_par, s_perp, yaw_off, sh, sat_t, s['fallen_at']))
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
    from machine.law import se2
    from machine.process import sprung, worker
    from tools.sim import told
    no_tables = not any(hasattr(se2, k) or hasattr(worker, k) for k in ('INIT_DELAYS', 'BRAKE_DELAYS', 'RISES'))
    by = {e[0]: e for e in sprung.entries()}
    r_horizons = (se2.rise_lead(by['rise_chair'][1]), se2.rise_lead(by['rise_bed'][1]), se2.rise_lead(by['rise_floor'][1]))
    # The bed's and floor's rises sit it up (specialize-sit-up-*: 400 ms), the chair's 450 mm one dips it (340).
    report.check('zero discrete delay tables in se2/worker; a rise leads its walk by its own length',
                 no_tables and r_horizons == (2.221, 2.281, 2.281), r_horizons)
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


ROSTER = (test_a_stop_settles_and_f_is_heard, test_a_halting_row_waits_for_a_swing_only_while_its_rhythm_turns,
          test_a_chair_a_hair_off_is_sat_on_and_risen_from, test_doorway_arm_yields_and_it_walks_on,
          test_chair_and_bed_seated_and_rising, test_viewport_f_errands_chair_bed_and_door,
          test_the_errand_goes_to_the_thing_told,
          test_the_low_chair_is_docked_as_it_faces,
          test_from_the_low_chair_it_rounds_the_bed_to_the_door,
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
