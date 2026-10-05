"""Her rows as `look.py` reads them: their geometry, and the steady walk's columns (WALK)."""
import functools
import math


def _p(r, seg):
    return tuple(float(r['%s_%s' % (seg, axis)]) for axis in 'xyz')


def _faces(r, seg):
    """How far `seg`'s forward points above level, deg: 90 lying on her back, -90 face down."""
    from machine import figure
    turn = figure.quat(*(float(r['q' + k]) for k in 'wxyz'))
    placed = figure.frames({j: float(r[j]) for j in figure.JOINTS}, _p(r, 'pelvis'), turn)
    return math.degrees(math.asin(max(-1.0, min(1.0, placed[seg][1][1][2]))))


def _reach_off(r):
    """Falling until an arm lands (a recording: falling), how far her reach - the hands from the
    shoulders, across the floor - points off the way she tips, deg; else nan."""
    landed = r.get('landed')
    if (r['stage'] != 'falling' if landed is None
            else r['stage'] not in ('falling', 'fallen') or landed):
        return math.nan
    from machine import figure
    turn = figure.quat(*(float(r['q' + k]) for k in 'wxyz'))
    tip = (turn[0][1], turn[2][1])
    hands = _mid(_p(r, 'left_hand'), _p(r, 'right_hand'))
    shoulders = _mid(_p(r, 'left_upper_arm'), _p(r, 'right_upper_arm'))
    reach = (hands[0] - shoulders[0], hands[2] - shoulders[2])
    if math.hypot(*tip) < 0.1 or math.hypot(*reach) < 0.05:
        return math.nan
    a = math.atan2(reach[0], reach[1]) - math.atan2(tip[0], tip[1])
    return abs(math.degrees(math.atan2(math.sin(a), math.cos(a))))


def _mid(a, b):
    return tuple((u + v) / 2.0 for u, v in zip(a, b))


def _out(r, joints):
    """How far the drives of `joints` reach past her skin as `r` poses her, mm: the worst of their
    drums' rims and faces (`drums`) outside every capsule of hers (`figure.BODY`)."""
    from coaxial.model.blocks import numpy as np
    from machine import figure
    placed = figure.frames({j: float(r[j]) for j in figure.JOINTS}, (0.0, 0.0, 0.0),
                           ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    turn = {k: (np.array(R), np.array(p)) for k, (p, R) in placed.items()}
    ends = np.array([turn[seg][1] + turn[seg][0] @ np.array(e) for seg, _r, a, b in _skin()
                     for e in (a, b)]).reshape(-1, 2, 3)
    radii = np.array([radius for _seg, radius, _a, _b in _skin()])
    worst = -np.inf
    for joint in joints:
        parent, dots = _drum(joint)
        if parent not in figure.STAGES and not any(seg == parent for seg, *_rest in _skin()):
            continue                   # no capsule of hers: the shank's put the foot's 87 mm out
        pts = turn[parent][1] + dots @ turn[parent][0].T
        a, ab = ends[:, 0][None], (ends[:, 1] - ends[:, 0])[None]
        t = np.clip(((pts[:, None] - a) * ab).sum(-1) / np.maximum((ab * ab).sum(-1), 1e-12),
                    0.0, 1.0)
        gap = np.linalg.norm(pts[:, None] - (a + t[..., None] * ab), axis=-1) - radii[None]
        worst = max(worst, float(gap.min(axis=1).max()))
    return 1e3 * worst


@functools.lru_cache(None)
def _skin():
    """[(segment, radius, end, end)]: her capsules (`figure.BODY`), each segment's, its frame."""
    from machine import figure
    out = []
    for seg in figure.SEGMENTS:
        bare = seg[0].split('_', 1)[1] if seg[0].startswith(('left_', 'right_')) else seg[0]
        x = -1.0 if seg[0].startswith('right_') else 1.0
        out += [(seg[0], radius, (a[0] * x, a[1], a[2]), (b[0] * x, b[1], b[2]))
                for part, radius, a, b in figure.BODY if part == bare]
    return tuple(out)


@functools.lru_cache(None)
def _drum(joint):
    """(part, dots (n, 3) in its frame): `joint`'s drum as drawn, its rims and faces."""
    from coaxial.graphics import drums
    from coaxial.model.blocks import numpy as np
    from machine import drives
    parent, at = next((p, o) for n, p, o, *_m in drums.drums() if n == 'drive_' + joint)
    axis, half = (np.array(v, float) if i == 0 else v for i, v in enumerate(drums.AXES[joint]))
    w = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, w)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    t = np.linspace(0.0, 2.0 * np.pi, 36, endpoint=False)
    rim = drives.of(joint)[1].diameter / 2.0 * (np.cos(t)[:, None] * u + np.sin(t)[:, None] * v)
    return parent, np.concatenate([np.array(at) + s * half * axis + f * rim
                                   for s in (-1.0, 0.0, 1.0) for f in (1.0, 0.5, 0.0)])


def _lean(a, b):
    """The line a -> b, deg ahead of plumb (her forward z)."""
    return math.degrees(math.atan2(b[2] - a[2], b[1] - a[1]))


def _roll(r):
    """The pelvis's roll, deg, + her left hip up."""
    w, x, y, z = (float(r[k]) for k in ('qw', 'qx', 'qy', 'qz'))
    return math.degrees(math.atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z)))


@functools.lru_cache(None)
def _thigh(dressed):
    """((depth under the hip joint, reach toward the other thigh), ..) down the left thigh as drawn,
    m, crotch to knee: the crotch the pelvis's lowest point, or the jeans' seat's
    (`coaxial.graphics.gynoid`)."""
    from coaxial.graphics import gynoid
    from machine.gait import HIP_DROP, THIGH
    meshes, parts = gynoid._meshes()
    worn = {part[0]: part[-1] for part in parts}
    thigh, seat = ((worn['cloth_left_thigh'], worn['cloth_seat']) if dressed
                   else (meshes['left_thigh'], meshes['pelvis']))
    rings = {}
    for x, y, _z in thigh[0]:
        rings[-round(float(y), 5)] = max(rings.get(-round(float(y), 5), 0.0), -float(x))
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


#: The walk measured from WALK_FROM_S after it begins: its look, each (name, unit, of the rows).
WALK_FROM_S = 2.0
WALK = (
    ('pelvis roll', 'deg', lambda rs: _ptp(_roll(r) for r in rs)),
    ('pelvis turn', 'deg', lambda rs: _ptp(_turn(r) for r in rs)),
    # a stride's own: each row less its stride's mean, her path's drift out
    ('hips wag', 'mm', lambda rs: _ptp(_surge(rs, lambda r: float(r['x']))) * 1e3),
    ('shoulders wag', 'mm', lambda rs: _ptp(_surge(rs, lambda r: _mid(
        _p(r, 'left_upper_arm'), _p(r, 'right_upper_arm'))[0])) * 1e3),
    ('pelvis swing', 'deg', lambda rs: _ptp(_surge(rs, _turn))),
    ('torso turn', 'deg', lambda rs: _ptp(_surge(rs, _shoulders_turn))),
    ('torso roll', 'deg', lambda rs: _ptp(_shoulders_roll(r) for r in rs)),
    ('head bob', 'mm', lambda rs: _ptp(_p(r, 'head')[1] for r in rs) * 1e3),
    ('head fore-aft', 'mm', lambda rs: _ptp(_surge(rs, lambda r: _p(r, 'head')[2])) * 1e3),
    ('head aside', 'mm', lambda rs: _ptp(_surge(rs, lambda r: _p(r, 'head')[0])) * 1e3),
    ('pelvis fore-aft', 'mm', lambda rs: _ptp(_surge(rs, lambda r: float(r['z']))) * 1e3),
    ('torso pitch', 'deg', lambda rs: _ptp(_lean(_p(r, 'torso'), _p(r, 'neck')) for r in rs)),
    ('hip punch', 'cm/s', lambda rs: 100.0 * max(abs(float(b['x']) - float(a['x']))
                                                 / max(1e-6, float(b['t']) - float(a['t']))
                                                 for a, b in zip(rs, rs[1:]))),
    ('head nod', 'deg', lambda rs: _ptp(_lean(_p(r, 'neck'), _p(r, 'head')) for r in rs)),
    ('arm', 'deg', lambda rs: _ptp(float(r['left_shoulder']) for r in rs)),
    ('elbow', 'deg', lambda rs: _ptp(float(r['left_elbow']) for r in rs)),
    ('elbow bent', 'deg', lambda rs: _mean(float(r['left_elbow']) for r in rs)),
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
    ('toes back at lift', 'mm', lambda rs: _mean(_slips(rs)) * 1e3),
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
    ('knee at lift', 'deg', lambda rs: _mean(float(a['left_knee']) for a, b in _lifts(rs))),
    ('heel at lift', 'deg', lambda rs: _mean(_heel(a) for a, b in _lifts(rs))),
    # the stance leg a strut: its knee while her foot bears her alone, its ankle behind its hip
    ('knee behind plumb', 'deg', lambda rs: max((float(r['left_knee']) for r in _alone(rs)
                                                 if _p(r, 'left_foot')[2] < _p(r, 'left_thigh')[2]),
                                                default=math.nan)),
    ('leg behind plumb', 'deg', lambda rs: max((_lean(_p(r, 'left_foot'), _p(r, 'left_thigh'))
                                                for r in _alone(rs)), default=math.nan)),
    ('hip over stance', 'deg', lambda rs: _mean(_roll(r) for r in _alone(rs))),
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
    """The left foot's landings: it comes to bear with its ankle ahead of its hip - its toes'
    load coming and going as the leg gives way behind her is no landing (2026-10-05)."""
    return [(a, b) for k, (a, b) in enumerate(zip(rs, rs[1:]), 1)
            if float(a['left_load']) <= BEARS_N < float(b['left_load']) and _held(rs, k, True)
            and _p(b, 'left_foot')[2] > _p(b, 'left_thigh')[2]]


def _lifts(rs):
    """Its lifts: the last time it stops bearing before each landing after the first."""
    downs = [(k, a, b) for k, (a, b) in enumerate(zip(rs, rs[1:]), 1)
             if float(a['left_load']) > BEARS_N >= float(b['left_load']) and _held(rs, k, False)]
    lands = [float(b['t']) for _a, b in _landings(rs)] + [math.inf]
    out = []
    for start, end in zip([-math.inf] + lands, lands):
        mine = [(a, b) for _k, a, b in downs if start < float(b['t']) < end]
        if mine and (start > -math.inf or end < math.inf):
            out.append(mine[-1])
    return out


def _slips(rs):
    """How far back the left toes go, each stride, from where they stood as the foot last bore
    her alone before they go ahead of it, m: a foot let go still pushing slides back along the
    floor (the user, 2026-10-04). From the row its load read under BEARS_N, a load read through
    20 ms came after the slide and the measure said 0 of 93 mm."""
    out, alone = [], {id(r) for r in _alone(rs)}
    for k, r in enumerate(rs[:-1]):
        if id(r) in alone and id(rs[k + 1]) not in alone:
            z0, back = _p(r, 'left_toes')[2], 0.0
            for n in rs[k + 1:]:
                z = _p(n, 'left_toes')[2] - z0
                if z > 0.05 or float(n['t']) - float(r['t']) > 0.6:
                    break
                back = min(back, z)
            out.append(-back)
    return out


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


def _heel(r):
    """The left heel's rise, deg: the ankle's line to the toes' joint off where the sole is flat."""
    from machine import gait
    ankle, toes = _p(r, 'left_foot'), _p(r, 'left_toes')
    return math.degrees(math.asin(max(-1.0, min(1.0, (ankle[1] - toes[1]) / math.dist(ankle, toes))))
                        - math.atan2(gait.ANKLE_H - gait.TOE_RY, gait.BALL))


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


def _shoulders_turn(r):
    """The shoulders' line about the vertical, deg, + her left shoulder back."""
    a, b = _p(r, 'left_upper_arm'), _p(r, 'right_upper_arm')
    return math.degrees(math.atan2(a[2] - b[2], a[0] - b[0]))


def _shoulders_roll(r):
    """The shoulders' line from level, deg, + her left shoulder up."""
    a, b = _p(r, 'left_upper_arm'), _p(r, 'right_upper_arm')
    return math.degrees(math.atan2(a[1] - b[1], math.hypot(a[0] - b[0], a[2] - b[2])))


def _turn(r):
    """The pelvis's turn about the vertical, deg, + her left hip back."""
    w, x, y, z = (float(r[k]) for k in ('qw', 'qx', 'qy', 'qz'))
    return math.degrees(math.atan2(2.0 * (x * z + w * y), 1.0 - 2.0 * (x * x + y * y)))


def measured(rows):
    """{measure: value}: WALK over a walk's `rows`, and what no row says alone - the parries she
    needed (`catches`) and, where the rows carry her power, the energy a metre (J/m)."""
    out = {name: f(rows) for name, _unit, f in WALK}
    out['catches'] = sum(1 for a, b in zip(rows, rows[1:]) if b['stage'] == 'catch' != a['stage'])
    watts = [float(r['watts']) for r in rows if r.get('watts') not in (None, '')]
    on = float(rows[-1]['z']) - float(rows[0]['z'])
    if watts and on > 0.1:
        out['energy'] = _mean(watts) * (float(rows[-1]['t']) - float(rows[0]['t'])) / on
    return out


def steady(rows):
    """The steady walk's rows: from WALK_FROM_S after the walk begins."""
    walk = [r for r in rows if r['stage'] in ('walk', 'catch')]
    return [r for r in walk if float(r['t']) >= float(walk[0]['t']) + WALK_FROM_S]


def walked(rows, take=None):
    """The steady walk's look: WALK over its rows, then how far off a woman's normal walk it
    is and what of it is out of her band (`normal.BAND`: by place, where WALK's are by the
    soles' loads); written a take at `take` (`fbx.wrote`)."""
    from tools.sim import fbx, normal
    walk = steady(rows)
    if len(walk) < 2:
        return
    got = measured(walk)
    print('\n9 walk from %.2f s to %.2f s' % (float(walk[0]['t']), float(walk[-1]['t'])))
    print('  ' + ' | '.join('%s %.1f %s' % (name, got[name], unit) for name, unit, _f in WALK)
          + ' | catches %d | energy %.0f J/m' % (got['catches'], got.get('energy', math.nan)))
    far, out = normal.off(normal.measured(*fbx.joints(walk)))
    print("  off a woman's walk %.2f%s" % (far, ''.join(
        ' | %s %.3g (%g)' % o for o in out)))
    if take:
        fbx.wrote(take, walk)
        print('  written %s: %d frames' % (take, len(walk)))
