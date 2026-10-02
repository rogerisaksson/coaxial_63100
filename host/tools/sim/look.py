#!/usr/bin/env python3
"""The gynoid as the eye sees her, measured a stage a row: simulated, or a HUMANOID recording.

From the squat as the page runs her, or a recording (R, build/recordings/*.csv) - the same rows
either way (`show_humanoid.row`).

    python tools/sim/look.py                        # from the squat, 16 s
    python tools/sim/look.py --last                 # the newest recording
    python tools/sim/look.py --csv build/recordings/humanoid_20260928_070724.csv
    python tools/sim/look.py SOFT_KNEE=6            # a knob moved (tools/sim/gait_montecarlo)
    python tools/sim/look.py --to 24 --halt 15      # halted at 15 s: 11 halt, 12 settle, ..
    python tools/sim/look.py --to 20 --event lace   # a floor event laid at 12 s, the fall's look

A row a stage (the walk's first second apart): the pelvis and the head under the stand (the
dip), the torso ahead of plumb, the torso against the left shin (under 0 it leans back over bent
knees), the hips-to-shoulders line, the left knee (under 0 bent back), the head's pitch (its
range the nod), the left hip's roll, the thighs' gap as drawn, bare and in the jeans (under 0
they meet), the trunk's and arms' to the legs (simulated), how far the pelvis's and the head's
forward point above level (90 on her back), falling until an arm lands her reach off the way she
tips, the seat's drives past her skin, what her drives draw - least and most over the stage.
"""
import argparse
import csv
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402
from tools.sim.strides import (_faces, _gap, _lean, _mid, _out, _p, _reach_off,  # noqa: E402
                               _roll, walked)

#: The head's rest pitch in the neck's offset (`figure.SEGMENTS`: 0.09 up, 0.012 on), deg.
HEAD_REST = math.degrees(math.atan2(0.012, 0.09))

#: The rows' rate simulated, Hz, as the page hears her; the walk's first seconds apart; when a
#: floor event is laid from, s.
RATE_HZ, FIRST_S, EVENT_S = 60.0, 1.0, 12.0


#: The drives a LEG_GAIN knob stiffens: kp times it, kd times its root (the damping ratio kept).
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')


def simulated(to_s, values, cadence=0.85, halt_s=None, event=None, event_s=EVENT_S, pushes=()):
    """The rows from the squat, `to_s` seconds, the director as the page runs her - halted at
    `halt_s`, an `event` laid from `event_s` (`machine.events`), pushed at each of `pushes` (s)
    as the page's P pushes, its side swapped each time; LEG_GAIN among `values` stiffening
    LEG_KINDS; a row what of her touches the floor (`down`), when the event was laid (`laid`)."""
    from machine.events import SHOVE_S as PUSH_S, SHOVES
    PUSH_N = SHOVES['shove']
    from machine import physics
    from tools.sim.gait_montecarlo import _set
    values = dict(values)
    gain = values.pop('LEG_GAIN', 1.0)
    for kind in LEG_KINDS:
        peak, kp, kd, armature = physics.SERVO[kind]
        physics.SERVO[kind] = (peak, kp * gain, kd * math.sqrt(gain), armature)
    _set(values)
    from machine import Machine, events
    from machine.director import Director
    from machine.figure import SEGMENTS
    from machine.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    body.arm()
    director = Director(body, cadence)
    director.begin()
    body.loop.step(0.0)
    bus, out, said = body.loop.bus, [], -1.0
    world = body.nodes['pelvis'].world
    ours = {world.model.body(seg[0]).id: seg[0] for seg in SEGMENTS}
    laid, was, side, due = None, 0.0, 1.0, sorted(pushes)
    while bus['t'] < to_s and (event or pushes or director.stage != 'fallen'):
        if due and bus['t'] >= due[0]:
            due.pop(0)
            side = -side
            world.push((side * PUSH_N, 0.0, 0.0), PUSH_S)
        if (event and laid is None and bus['t'] >= event_s
                and was < events.at(event) <= director.walker.phase):
            events.lay(event, director, world)
            laid = bus['t']
        was = director.walker.phase
        if halt_s is not None and halt_s <= bus['t'] < halt_s + 0.001:
            director.halt()
        asked = director.step(0.001)
        body.loop.write(**asked)
        body.loop.step(0.001)
        if bus['t'] - said >= 1.0 / RATE_HZ:
            said = bus['t']
            out.append(dict(sample(bus, director, world, asked), down=_down(world, ours),
                            laid=laid))
    body.disarm()
    return out


def sample(bus, director, world, asked=None):
    """A row as the page records it (`show_humanoid.row`), from the loop's bus: the loose hinges
    beside it, and how near her feet come."""
    from machine.figure import JOINTS
    from terminal.views.show_humanoid import HEADER, row
    now = {'t': bus['t'], 'stage': director.stage, 'speed': bus['pelvis.pose.vz'],
           'phase': director.walker.phase,
           'loads': (bus['pelvis.pose.left_load'], bus['pelvis.pose.right_load']),
           'where': (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z']),
           'turn': tuple(bus['pelvis.pose.q' + k] for k in 'wxyz'),
           'angles': {j: bus.get(j + '.deg', 0.0) for j in JOINTS}, 'set': asked or {}}
    now['watts'] = world.drawn()
    return dict(zip(HEADER, row(now, 60.0)), loose=world.loose(),
                feet=world.gap(LEFT_FOOT, RIGHT_FOOT), lifted=world.lifted(LEFT_FOOT),
                landed=director.touched_at is not None,
                trunk=world.gap(('torso', 'head', 'upper_arm', 'forearm', 'hand'),
                                ('thigh', 'shank')))


def _down(world, ours):
    """Her segments touching anything not hers but the soles, by name."""
    d, m = world.data, world.model
    out = set()
    for i in range(d.ncon):
        a, b = (m.geom_bodyid[g] for g in (d.contact[i].geom1, d.contact[i].geom2))
        for mine, other in ((a, b), (b, a)):
            if mine in ours and other not in ours and not ours[mine].endswith(('foot', 'toes')):
                out.add(ours[mine])
    return ' '.join(sorted(out))


def recorded(path):
    """The rows of a recording."""
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


#: (name, unit, of a row and the stand's heights {pelvis, head, hip L, hip R}): each column.
MEASURES = (
    ('pelvis dy', 'mm', lambda r, ref: (float(r['y']) - ref['pelvis']) * 1e3),
    ('head dy', 'mm', lambda r, ref: (_p(r, 'head')[1] - ref['head']) * 1e3),
    ('hip L dy', 'mm', lambda r, ref: (_p(r, 'left_thigh')[1] - ref['hip L']) * 1e3),
    ('hip R dy', 'mm', lambda r, ref: (_p(r, 'right_thigh')[1] - ref['hip R']) * 1e3),
    ('pelvis roll', 'deg', lambda r, ref: _roll(r)),
    ('torso', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))),
    ('torso-shin', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))
     - _lean(_p(r, 'left_foot'), _p(r, 'left_shank'))),
    ('hips>shoulders', 'deg', lambda r, ref: _lean(_mid(_p(r, 'left_thigh'), _p(r, 'right_thigh')),
                                                   _mid(_p(r, 'left_upper_arm'),
                                                        _p(r, 'right_upper_arm')))),
    ('bow', 'deg', lambda r, ref: _lean(_p(r, 'torso'), _p(r, 'neck'))
     - _lean(_p(r, 'left_foot'), _p(r, 'left_thigh'))),
    ('knee L', 'deg', lambda r, ref: float(r['left_knee'])),
    ('head pitch', 'deg', lambda r, ref: _lean(_p(r, 'neck'), _p(r, 'head')) - HEAD_REST),
    ('hip roll L', 'deg', lambda r, ref: float(r['left_hip_roll'])),
    ('thigh gap', 'mm', lambda r, ref: _gap(r, False)),
    ('jeans gap', 'mm', lambda r, ref: _gap(r, True)),
    ('trunk gap', 'mm', lambda r, ref: 1e3 * float(r.get('trunk', 'nan'))),
    ('pelvis faces', 'deg', lambda r, ref: _faces(r, 'pelvis')),
    ('head faces', 'deg', lambda r, ref: _faces(r, 'head')),
    ('reach off fall', 'deg', lambda r, ref: _reach_off(r)),
    ('hands out', 'mm', lambda r, ref: 1e3 * max(
        math.dist(_p(r, side + '_fingers'), _mid(_p(r, 'torso'), _p(r, 'neck')))
        for side in ('left', 'right'))),
    ('seat out', 'mm', lambda r, ref: _out(r, SEAT)),
    ('drawing', 'W', lambda r, ref: float(r.get('watts') or 'nan')),
)

#: The seat's drives, `seat out` their worst reach past her skin.
SEAT = tuple(side + k for side in ('left_', 'right_') for k in ('hip', 'hip_roll', 'hip_yaw'))

#: A seam's measures, by name, and the director's ask beside them; the times read about it, s.
SEAM = ('pelvis dy', 'head dy', 'pelvis roll', 'torso', 'bow', 'knee L')
SEAM_AT = (-0.3, -0.1, 0.0, 0.1, 0.3)


def staged(rows):
    """[(moment, rows)] in order, numbered as the page numbers them (`director.moment`), the
    walk's first FIRST_S apart; the stand's heights."""
    from machine.director import moment
    stands = [r for r in rows if r['stage'] == 'stand'] or rows[:1]
    mid = stands[len(stands) // 2]
    ref = {'pelvis': float(mid['y']), 'head': _p(mid, 'head')[1],
           'hip L': _p(mid, 'left_thigh')[1], 'hip R': _p(mid, 'right_thigh')[1]}
    walk_at, groups = None, []
    for r in rows:
        stage = moment(r['stage'])
        if r['stage'] == 'walk':
            walk_at = float(r['t']) if walk_at is None else walk_at
            stage += ', 1st s' if float(r['t']) - walk_at < FIRST_S else ''
        if not groups or groups[-1][0] != stage:
            groups.append((stage, []))
        groups[-1][1].append(r)
    return groups, ref


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--csv', help='a HUMANOID recording (R) to measure')
    parser.add_argument('--last', action='store_true', help='the newest recording')
    parser.add_argument('--to', type=float, default=16.0, help='seconds simulated from the squat')
    parser.add_argument('--cadence', type=float, default=0.85, help='strides a second asked')
    parser.add_argument('--halt', type=float, help='halted at this second, into the squat')
    parser.add_argument('--event', help='a floor event laid: hole, sill, slip, rug, lace, soa, hot')
    parser.add_argument('--event-at', type=float, default=EVENT_S, help='laid from this second')
    parser.add_argument('--push', type=float, nargs='*', default=[], metavar='S',
                        help="pushed at these seconds as the page's P pushes")
    parser.add_argument('--brief', action='store_true',
                        help="the stages in a line and the walk's measures: no tables")
    parser.add_argument('knobs', nargs='*', metavar='NAME=V', help='constants moved')
    args = parser.parse_args(argv)
    found = glob.glob(os.path.join(REPO, 'build', 'recordings', '*.csv')) if args.last else []
    if args.last and not found:
        print('no recording in build/recordings: R on the HUMANOID page records one')
        return 1
    path = args.csv or (max(found, key=os.path.getmtime) if found else None)
    values = {k: float(v) for k, v in (kv.split('=') for kv in args.knobs)}
    rows = recorded(path) if path else simulated(args.to, values, args.cadence, args.halt,
                                                 args.event, args.event_at, args.push)
    print(path or 'simulated from the squat, %.1f s %s' % (
        args.to, ' '.join(args.knobs)))
    groups, ref = staged(rows)
    if args.brief:
        print(' '.join('%s@%.2f' % (stage, float(mine[0]['t'])) for stage, mine in groups))
        walked(rows)
        return 0
    print('%-14s %6s | %s' % ('moment', 'from s', ' | '.join(
        '%-15s' % ('%s %s' % (name, unit)) for name, unit, _f in MEASURES)))
    for stage, mine in groups:
        cells = []
        for _name, _unit, f in MEASURES:
            v = [f(r, ref) for r in mine]
            cells.append('%+6.1f..%+6.1f' % (min(v), max(v)))
        print('%-14s %6.2f | %s' % (stage, float(mine[0]['t']), ' | '.join(cells)))
    seams(rows, groups, ref)
    walked(rows)
    fell(rows)
    return 0


#: A foot and its toes, each side's: how near the feet come (`World.gap`).
LEFT_FOOT, RIGHT_FOOT = ('left_foot', 'left_toes'), ('right_foot', 'right_toes')

#: The joints whose speed is her flailing: the limbs'.
LIMBS = tuple(side + k for side in ('left_', 'right_') for k in (
    'shoulder', 'elbow', 'wrist', 'hip', 'knee', 'ankle'))


def fell(rows):
    """From a floor event on: how far she tipped, her limbs' fastest joint (the flail), what of
    her touched the floor first and when, and the head's speed as it came down."""
    laid = next((float(r['laid']) for r in rows if r.get('laid') is not None), None)
    if laid is None:
        return
    after = [r for r in rows if float(r['t']) >= laid]
    tipped = max(math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (
        float(r['qx']) ** 2 + float(r['qz']) ** 2))))) for r in after)
    flail, fastest = 0.0, ''
    for a, b in zip(after, after[1:]):
        dt = max(1e-6, float(b['t']) - float(a['t']))
        for j in LIMBS:
            v = abs(float(b[j]) - float(a[j])) / dt
            if v > flail:
                flail, fastest = v, '%s at %.2f s' % (j, float(b['t']))
    first = next(((float(r['t']), r['down']) for r in after if r['down']), None)
    head = next((r for r in after if 'head' in r['down'].split()), None)
    hit = ''
    if head is not None:
        k = rows.index(head)
        a, b = rows[k - 1], head
        hit = ', the head down at %.2f s at %.2f m/s' % (float(b['t']), (
            _p(a, 'head')[1] - _p(b, 'head')[1]) / max(1e-6, float(b['t']) - float(a['t'])))
    print('\nthe event at %.2f s: tipped %.0f deg, stages %s' % (
        laid, tipped, ' '.join(dict.fromkeys(r['stage'] for r in after))))
    print('  flail %.0f deg/s (%s); first down: %s%s' % (
        flail, fastest, '%s at %.2f s' % (first[1], first[0]) if first else 'nothing', hit))


def seams(rows, groups, ref):
    """Each hand-off between stages: the SEAM measures SEAM_AT seconds about it, the left knee
    as the director asked it beside the knee as it is."""
    f = {name: g for name, _u, g in MEASURES}
    ts = [float(r['t']) for r in rows]
    for (before, _mine), (after, theirs) in zip(groups, groups[1:]):
        at = float(theirs[0]['t'])
        print()
        print('%s -> %s at %.2f s' % (before, after, at))
        print('   dt s | %s | knee L asked' % ' | '.join('%14s' % n for n in SEAM))
        for dt in SEAM_AT:
            r = rows[min(range(len(ts)), key=lambda i: abs(ts[i] - at - dt))]
            asked = r.get('set_left_knee', '')
            print('  %+5.2f | %s | %s' % (dt, ' | '.join('%+14.1f' % f[n](r, ref) for n in SEAM),
                                          '%12.1f' % float(asked) if asked else '           -'))


if __name__ == '__main__':
    sys.exit(main())
