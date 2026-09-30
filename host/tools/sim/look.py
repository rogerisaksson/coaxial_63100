#!/usr/bin/env python3
"""The gynoid as the eye sees her, measured a stage a row: simulated, or a HUMANOID recording.

From the squat as the page runs her, or a recording (R, build/recordings/*.csv) - the same rows
either way (`show_humanoid.row`).

    python tools/sim/look.py                        # from the squat, 12 s
    python tools/sim/look.py --last                 # the newest recording
    python tools/sim/look.py --csv build/recordings/humanoid_20260928_070724.csv
    python tools/sim/look.py SOFT_KNEE=6            # a knob moved (tools/sim/gait_montecarlo)
    python tools/sim/look.py --to 24 --halt 15      # halted at 15 s: 11 halt, 12 settle, ..
    python tools/sim/look.py --to 20 --event lace   # a floor event laid at 12 s, the fall's look

A row a stage (the walk's first second apart): the pelvis and the head under the stand (the
dip), the torso ahead of plumb, the torso against the left shin (under 0 it leans back over bent
knees), the hips-to-shoulders line, the left knee (under 0 bent back), the head's pitch (its
range the nod), the left hip's roll, the thighs' gap as drawn, bare and in the jeans (under 0
they meet) - least and most over the stage, deg and mm.
"""
import argparse
import csv
import functools
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

#: The head's rest pitch in the neck's offset (`figure.SEGMENTS`: 0.09 up, 0.012 on), deg.
HEAD_REST = math.degrees(math.atan2(0.012, 0.09))

#: The rows' rate simulated, Hz, as the page hears her; the walk's first seconds apart; when a
#: floor event is laid from, s.
RATE_HZ, FIRST_S, EVENT_S = 60.0, 1.0, 12.0


#: The drives a LEG_GAIN knob stiffens: kp times it, kd times its root (the damping ratio kept).
LEG_KINDS = ('hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll')


def simulated(to_s, values, cadence=0.85, halt_s=None, event=None, event_s=EVENT_S):
    """The rows from the squat, `to_s` seconds, the director as the page runs her - halted at
    `halt_s`, an `event` laid from `event_s` (`machine.events`); LEG_GAIN among `values`
    stiffens the legs' drives
    (`physics.SERVO`). A simulated row says what of her touches the floor (`down`) and when
    the event was laid (`laid`)."""
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
    laid, was = None, 0.0
    while bus['t'] < to_s and (event or director.stage != 'fallen'):
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
    return dict(zip(HEADER, row(now, 60.0)), loose=world.loose(),
                feet=world.gap(LEFT_FOOT, RIGHT_FOOT), lifted=world.lifted(LEFT_FOOT))


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


def _p(r, seg):
    return tuple(float(r['%s_%s' % (seg, axis)]) for axis in 'xyz')


def _mid(a, b):
    return tuple((u + v) / 2.0 for u, v in zip(a, b))


def _lean(a, b):
    """The line a -> b, deg ahead of plumb (her forward z)."""
    return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))


def _roll(r):
    """The pelvis's roll, deg, + her left hip up."""
    w, x, y, z = (float(r[k]) for k in ('qw', 'qx', 'qy', 'qz'))
    return math.degrees(math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z)))


@functools.lru_cache(None)
def _thigh(dressed):
    """((depth under the hip joint, radius), ..) down a thigh as drawn, m, crotch to knee: the
    crotch the pelvis's lowest point, or the jeans' seat's (`coaxial.graphics.gynoid`)."""
    from coaxial.graphics import gynoid
    from machine.gait import HIP_DROP, THIGH
    meshes, parts = gynoid._meshes()
    worn = {part[0]: part[-1] for part in parts}
    thigh, seat = ((worn['cloth_left_thigh'], worn['cloth_seat']) if dressed
                   else (meshes['left_thigh'], meshes['pelvis']))
    rings = {}
    for x, y, z in thigh[0]:
        rings[-round(float(y), 5)] = max(rings.get(-round(float(y), 5), 0.0), math.hypot(x, z))
    ring = sorted(rings.items())
    crotch = -float(min(seat[0][:, 1])) - HIP_DROP
    out = []
    for d in (crotch + (THIGH - crotch) * i / 11.0 for i in range(12)):
        (d0, r0), (d1, r1) = next(p for p in zip(ring, ring[1:]) if p[0][0] <= d <= p[1][0])
        out.append((d, r0 + (r1 - r0) * (d - d0) / (d1 - d0)))
    return tuple(out)


def _gap(r, dressed):
    """The least distance between the thighs' surfaces as drawn, crotch to knee, mm."""
    def down(side):
        hip, knee = _p(r, side + '_thigh'), _p(r, side + '_shank')
        n = math.dist(hip, knee)
        return [(tuple(h + (k - h) * d / n for h, k in zip(hip, knee)), rad)
                for d, rad in _thigh(dressed)]
    return 1e3 * min(math.dist(a, b) - ra - rb for a, ra in down('left') for b, rb in down('right'))


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
)

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
                                                 args.event, args.event_at)
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

#: The walk measured from WALK_FROM_S after it begins: its look, each (name, unit, of the rows).
WALK_FROM_S = 2.0
WALK = (
    ('hips across', 'mm', lambda rs: _ptp(float(r['x']) for r in rs) * 1e3),
    ('shoulders across', 'mm', lambda rs: _ptp(
        _mid(_p(r, 'left_upper_arm'), _p(r, 'right_upper_arm'))[0] for r in rs) * 1e3),
    ('pelvis roll', 'deg', lambda rs: _ptp(_roll(r) for r in rs)),
    ('pelvis turn', 'deg', lambda rs: _ptp(_turn(r) for r in rs)),
    ('head bob', 'mm', lambda rs: _ptp(_p(r, 'head')[1] for r in rs) * 1e3),
    ('head fore-aft', 'mm', lambda rs: _ptp(_surge(rs, lambda r: _p(r, 'head')[2])) * 1e3),
    ('pelvis fore-aft', 'mm', lambda rs: _ptp(_surge(rs, lambda r: float(r['z']))) * 1e3),
    ('torso pitch', 'deg', lambda rs: _ptp(_lean(_p(r, 'torso'), _p(r, 'neck')) for r in rs)),
    ('hip punch', 'cm/s', lambda rs: 100.0 * max(abs(float(b['x']) - float(a['x']))
                                                 / max(1e-6, float(b['t']) - float(a['t']))
                                                 for a, b in zip(rs, rs[1:]))),
    ('head nod', 'deg', lambda rs: _ptp(_lean(_p(r, 'neck'), _p(r, 'head')) for r in rs)),
    ('arm', 'deg', lambda rs: _ptp(float(r['left_shoulder']) for r in rs)),
    ('elbow', 'deg', lambda rs: _ptp(float(r['left_elbow']) for r in rs)),
    ('strike', 'N', lambda rs: max(max(float(r['left_load']), float(r['right_load'])) for r in rs)),
    ('feet apart', 'mm', lambda rs: _mean(abs(_p(r, 'left_foot')[0] - _p(r, 'right_foot')[0])
                                          for r in rs if float(r['left_load']) > 60.0
                                          and float(r['right_load']) > 60.0) * 1e3),
    ('thigh gap', 'mm', lambda rs: min(_gap(r, False) for r in rs)),
    ('jeans gap', 'mm', lambda rs: min(_gap(r, True) for r in rs)),
    ('ankle ahead at landing', 'mm', lambda rs: _mean(
        _p(b, 'left_foot')[2] - _p(b, 'left_thigh')[2] for a, b in _landings(rs)) * 1e3),
    ('toes behind at lift', 'mm', lambda rs: _mean(
        _p(a, 'left_thigh')[2] - _p(a, 'left_toes')[2] for a, b in _lifts(rs)) * 1e3),
    ('thigh behind at lift', 'deg', lambda rs: _mean(
        _lean(_p(a, 'left_shank'), _p(a, 'left_thigh')) for a, b in _lifts(rs))),
    ('thigh ahead at landing', 'deg', lambda rs: _mean(
        -_lean(_p(b, 'left_shank'), _p(b, 'left_thigh')) for a, b in _landings(rs))),
    ('thigh most ahead', 'deg', lambda rs: _mean(max(
        -_lean(_p(r, 'left_shank'), _p(r, 'left_thigh')) for r in s) for s in _swings(rs))),
    ('feet clear', 'mm', lambda rs: min((r.get('feet', math.nan) for r in rs),
                                        default=math.nan) * 1e3),
    ('touchdown', 'm/s', lambda rs: _touchdown(rs)),
    ('swing clear', 'mm', lambda rs: min((r['lifted'] for r in _mid_swings(rs)),
                                         default=math.nan) * 1e3),
    ('swing height', 'mm', lambda rs: max((r['lifted'] for r in _mid_swings(rs)),
                                          default=math.nan) * 1e3),
    ('knee swinging', 'deg', lambda rs: max((float(r['left_knee']) for r in _mid_swings(rs)),
                                            default=math.nan)),
    ('knee at landing', 'deg', lambda rs: _mean(float(b['left_knee']) for a, b in _landings(rs))),
    ('toe out', 'deg', lambda rs: _mean(_foot(r)[0] for r in _alone(rs))),
    ('foot roll', 'deg', lambda rs: _mean(_foot(r)[1] for r in _alone(rs))),
    ('ankle roll', 'deg', lambda rs: _mean(float(r['left_ankle_roll']) for r in _alone(rs))),
    ('toe out swinging', 'deg', lambda rs: min((_foot(r)[0] for r in _mid_swings(rs)),
                                               default=math.nan)),
    ('hair fore-aft', 'deg', lambda rs: _ptp(r.get('loose', {}).get('hair_x', 0.0) for r in rs)),
    ('hair aside', 'deg', lambda rs: _ptp(r.get('loose', {}).get('hair_z', 0.0) for r in rs)),
)

#: A sole bears past BEARS_N: its landing and its lift are the rows either side of it, the one
#: after holding HELD_S - the load flickers under it late in the stance.
BEARS_N, HELD_S = 60.0, 0.1


def _held(rs, k, bears):
    t = float(rs[k]['t'])
    return all((float(r['left_load']) > BEARS_N) == bears for r in rs[k:]
               if float(r['t']) - t < HELD_S)


def _landings(rs):
    return [(a, b) for k, (a, b) in enumerate(zip(rs, rs[1:]), 1)
            if float(a['left_load']) <= BEARS_N < float(b['left_load']) and _held(rs, k, True)]


def _lifts(rs):
    return [(a, b) for k, (a, b) in enumerate(zip(rs, rs[1:]), 1)
            if float(a['left_load']) > BEARS_N >= float(b['left_load']) and _held(rs, k, False)]


def _alone(rs):
    """The rows the left foot bears her alone."""
    return [r for r in rs if float(r['left_load']) > 250.0 and float(r['right_load']) < BEARS_N]


def _foot(r):
    """(toe-out, roll) of the left foot, degrees: its toes out of the walk's line, and its outer
    edge up off the floor (everted, pronation)."""
    from machine import figure
    turn = figure.quat(*(float(r['q' + k]) for k in 'wxyz'))
    _at, foot = figure.foot_of(1.0, (0.0, 0.0, 0.0), turn,
                               [math.radians(float(r['left' + k])) for k in figure.LEG])
    ahead, out = figure.apply(foot, (0.0, 0.0, 1.0)), figure.apply(foot, (1.0, 0.0, 0.0))
    return (math.degrees(math.atan2(ahead[0], ahead[2])),
            math.degrees(math.asin(max(-1.0, min(1.0, out[1])))))


def _touchdown(rs):
    """The left ankle's fall, m/s, over the two rows before each landing, meaned."""
    lands = {id(b) for _a, b in _landings(rs)}
    return _mean((_p(rs[k - 2], 'left_foot')[1] - _p(rs[k], 'left_foot')[1])
                 / max(1e-6, float(rs[k]['t']) - float(rs[k - 2]['t']))
                 for k in range(2, len(rs)) if id(rs[k]) in lands)


def _swings(rs):
    """Each of the left foot's swings, its rows from its lift to its landing."""
    lifts, lands = {id(b) for _a, b in _lifts(rs)}, {id(b) for _a, b in _landings(rs)}
    out, start = [], None
    for k, r in enumerate(rs):
        if id(r) in lifts:
            start = k
        elif id(r) in lands and start is not None:
            out.append(rs[start:k + 1])
            start = None
    return out


def _mid_swings(rs):
    """The rows through the middle half of each of the left foot's swings."""
    out = []
    for s in _swings(rs):
        span = len(s) - 1
        out += [x for x in s[span // 4:span - span // 4] if 'lifted' in x]
    return out


def _ptp(values):
    v = list(values)
    return max(v) - min(v) if v else float('nan')


def _mean(values):
    v = list(values)
    return sum(v) / len(v) if v else float('nan')


#: A stride's seconds at the walk's cadence, the window her surge is read against.
STRIDE_S = 1.0 / 0.85


def _surge(rs, along):
    """Each row's place along the walk less its mean over the stride about it: the surge alone,
    whatever her pace does over seconds."""
    ts, vs = [float(r['t']) for r in rs], [along(r) for r in rs]
    out = []
    for t, v in zip(ts, vs):
        near = [u for s, u in zip(ts, vs) if abs(s - t) <= STRIDE_S / 2.0]
        if ts[0] <= t - STRIDE_S / 2.0 and t + STRIDE_S / 2.0 <= ts[-1]:
            out.append(v - sum(near) / len(near))
    return out


def _turn(r):
    """The pelvis's turn about the vertical, deg, + her left hip back."""
    w, x, y, z = (float(r[k]) for k in ('qw', 'qx', 'qy', 'qz'))
    return math.degrees(math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y)))


def walked(rows):
    """The steady walk's look: WALK over the rows from WALK_FROM_S after the walk begins."""
    walk = [r for r in rows if r['stage'] == 'walk']
    if not walk:
        return
    from_t = float(walk[0]['t']) + WALK_FROM_S
    steady = [r for r in walk if float(r['t']) >= from_t]
    if len(steady) < 2:
        return
    print('\n9 walk from %.2f s to %.2f s' % (from_t, float(steady[-1]['t'])))
    print('  ' + ' | '.join('%s %.1f %s' % (name, f(steady), unit) for name, unit, f in WALK))


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
