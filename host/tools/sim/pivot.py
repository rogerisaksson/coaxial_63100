"""Her pivot on an open floor: walked to her pace, a half turn by a way of turning, measured.

    python tools/sim/pivot.py                      # each way of turning, a line each
    python tools/sim/pivot.py --way spin --trace   # and a line a frame through the turn

A way of turning (WAYS): `spin` - her walk's stop, then her keys' turn in place till she faces back; `heading` -
her walk let go and her heading told back at once (`target_heading`, its slew, as an errand tells it). It says the
seconds from the turn's start to facing back (within `modal.LIST_RAD`), how far she went meanwhile, the most a
standing sole turned on the floor while it bore her (rad: a heel's pivot turns it, a planted sole holds), her
lowest surety, her hardest landing (N), her fastest driven joint (deg/s, the spasm judge's), her stumbles, a fall -
the kernel's `pivot-single-anchor-arc` its mark: a half turn in a stride, no zero-velocity pause.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Her walk to her pace, then the turn's time at most, s; her page's frame for a trace, s.
WALK_S, OVER_S, FRAME_S = 4.0, 10.0, 0.05
WAYS = ('spin', 'heading')


def _yaw(d, body):
    """A body's yaw, rad, as her heading reads one (its own z along the floor: (sin, cos) on x, z)."""
    r = d.xmat[body].reshape(3, 3)
    return math.atan2(r[0, 2], r[2, 2])


def turned(way, walk_s=WALK_S, over_s=OVER_S, trace=False):
    """Way `way`'s half turn after `walk_s` of her walk: {way, faced_s, went_m, slip_rad, surety, landing_n,
    fastest, stumbles, fell}."""
    from machine.world import physics
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    from machine.law import modal, se2
    from machine.moves import leap
    from machine.process import introspect, sprung, watch, worker
    s = worker.session()
    g = s['g']
    d, m = g['world'].data, g['world'].model
    feet, pelvis = [m.body(n).id for n in ('left_foot', 'right_foot')], m.body('pelvis').id
    worker.apply(s, {'twist': (se2.WALK_SPEED, 0.0, 0.0)})
    while g['t'] < walk_s and s['fallen_at'] is None:
        worker.stepped(s)
    t0, back = float(g['t']), _yaw(d, pelvis) + math.pi
    at0, was = (float(d.qpos[0]), float(d.qpos[2])), [_yaw(d, f) for f in feet]
    worker.apply(s, {'twist': (0.0, 0.0, 0.0)})
    if way == 'heading':
        g['has_heading'], g['target_heading'] = 1.0, math.remainder(back, math.tau)
    tap, watched, driven = introspect.Tap(), watch.Watch(), set(watch.meta()['rated'])
    out = {'way': way, 'faced_s': None, 'slip_rad': 0.0, 'surety': 1.0, 'landing_n': 0.0, 'fastest': (0.0, ''),
           'stumbles': 0, 'fell': None}
    spun, frame = False, -math.inf
    while g['t'] - t0 < over_s and s['fallen_at'] is None:
        worker.stepped(s)
        t, off = float(g['t']), math.remainder(_yaw(d, pelvis) - back, math.tau)
        if way == 'spin' and not spun and sprung.doing(g) == 'stand':
            spun = True
            worker.apply(s, {'twist': (0.0, 0.0, se2.YAW_RATE)})
        if out['faced_s'] is None and abs(off) <= modal.LIST_RAD:
            out['faced_s'] = round(t - t0, 2)
            worker.apply(s, {'twist': (0.0, 0.0, 0.0)})
        yaws = [_yaw(d, f) for f in feet]
        for k in range(2):
            if float(g['loads'][k]) > leap.TOUCH_N:
                out['slip_rad'] += abs(math.remainder(yaws[k] - was[k], math.tau))
        was = yaws
        out['surety'] = min(out['surety'], float(g['surety']))
        out['landing_n'] = max(out['landing_n'], max(float(x) for x in g['loads']))
        snap = tap.passed(s)
        if snap is not None:
            peaks = snap['body']['peaks']
            out['fastest'] = max(out['fastest'], max((v, k) for k, v in peaks.items() if k in driven))
            out['stumbles'] += sum(e['event'] == 'stumble' for e in watched.step(snap))
        if trace and t - frame >= FRAME_S:
            frame = t
            print('t %5.2f %-10s off %+6.1f deg  v %.2f  soles %4.0f/%4.0f  feet %+6.1f/%+6.1f deg  sure %.2f' % (
                t, sprung.doing(g), math.degrees(off), math.hypot(float(d.qvel[0]), float(d.qvel[2])),
                *g['loads'][:2], *(math.degrees(y) for y in yaws), float(g['surety'])))
    out['went_m'] = round(math.hypot(float(d.qpos[0]) - at0[0], float(d.qpos[2]) - at0[1]), 2)
    out['fell'] = None if s['fallen_at'] is None else round(s['fallen_at'], 2)
    out['fastest'] = '%s %.0f' % (out['fastest'][1], out['fastest'][0])
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description='Her pivot on an open floor: a half turn after her walk, measured')
    p.add_argument('--way', choices=WAYS, nargs='+', default=list(WAYS))
    p.add_argument('--walk', type=float, default=WALK_S, help='her walk before the turn, s')
    p.add_argument('--trace', action='store_true', help='a line a frame through the turn')
    a = p.parse_args(argv)
    for way in a.way:
        r = turned(way, a.walk, trace=a.trace)
        print('%s: faced back in %s s, went %.2f m, standing sole turned %.2f rad, surety %.2f, landing %.0f N, '
              'fastest %s deg/s, stumbles %d, fell %s' % (way, r['faced_s'], r['went_m'], r['slip_rad'], r['surety'],
                                                         r['landing_n'], r['fastest'], r['stumbles'], r['fell']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
