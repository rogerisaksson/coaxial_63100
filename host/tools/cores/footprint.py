"""Her body's footprint as static arrays: her reach each way by height, standing, walking, turning.

    python tools/cores/footprint.py --write     # machine/body/footprint_rows.py
    laid = footprint.laid()                     # what the file holds, laid anew on her loop

Her own motions on her loop in the open (`physics.ROOM` 0). Each pass every geom of her body, its
shape's extreme points, into each band of BAND_M - her map's cell - it spans over the floor: the support
h(phi) = max p . (cos phi forward + sin phi left) + its radius at K directions, the most over the motion.
Standing, walking, her walk back and her sidesteps, each with its stop: about her pelvis in her own
heading's frame. A turn in place by each of TURN_DEG either way, from still after a walk's stop - her
soles bearing alike - as an errand turns: about her pelvis's place and heading as the turn began, its
stop's settling with it, the
most over WALKS walked before; its carry the mean of her pelvis's end there. `machine.body.footprint`
reads them.

STEP_M: the least her swinging soles clear the floor at the top of a swing over her steady walk.
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine.moves import leap
from machine.law import modal, se2
from machine.eyes import voxels
from tools import REPO
from tools.cores import written

PATH = os.path.join(REPO, 'host', 'machine', 'body', 'footprint_rows.py')
NOTICE = "Her body's footprint as static arrays: written by host/tools/cores/footprint.py, not by hand."

#: Directions about her, bands over the floor, m; her bodies (`mjcf`'s, the pelvis down).
K, BAND_M = 32, voxels.CELL_M
HERS = ('pelvis', 'spine_roll', 'torso', 'neck', 'head', 'hair', 'left_', 'right_')
#: Her motions in her own frame, by mode: (seconds, twist told) in turn, after SETTLE_S standing.
SETTLE_S = 1.0
MOTIONS = {
    'stand': ((2.0, (0.0, 0.0, 0.0)),),
    'walk': ((5.0, (se2.WALK_SPEED, 0.0, 0.0)), (3.0, (0.0, 0.0, 0.0))),
    'back': ((3.0, (-se2.BACK_SPEED, 0.0, 0.0)), (2.0, (0.0, 0.0, 0.0))),
    'side': ((2.0, (0.0, se2.STRAFE_SPEED, 0.0)), (2.0, (0.0, 0.0, 0.0)), (2.0, (0.0, -se2.STRAFE_SPEED, 0.0)),
             (2.0, (0.0, 0.0, 0.0))),
}
#: Her steady walk's swings read for STEP_M: from this many seconds into the walk to its stop.
STEADY_S = 1.5
#: Her turns in place, deg; the walks before each - seconds, and straight or on her arc either way -, her
#: gait's phase and her heading's swing at its stop varied; the most a turn and its settling may take, s.
TURN_DEG, TURN_MOST_S = (45, 90, 135, 180), 16.0
WALKS = ((1.0, 0.0), (1.4, 0.0), (1.8, 0.0), (1.0, se2.YAW_RATE), (1.4, -se2.YAW_RATE))


def _points(m, d, i):
    """(k, 3) the extreme points of geom `i`'s shape and the radius about them."""
    import numpy as np
    c, R, sz, t = d.geom_xpos[i], d.geom_xmat[i].reshape(3, 3), m.geom_size[i], int(m.geom_type[i])
    corners = np.array([[sx, sy, sz_] for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)], float)
    shapes = {2: (c[None, :], float(sz[0])), 3: (np.stack([c - R[:, 2] * sz[1], c + R[:, 2] * sz[1]]), float(sz[0])),
              5: (np.stack([c - R[:, 2] * sz[1], c + R[:, 2] * sz[1]]), float(sz[0])), 6: (c + (corners * sz[:3]) @ R.T, 0.0)}
    return shapes.get(t, (c[None, :], float(m.geom_rbound[i])))


def _session():
    from machine.world import physics
    from machine.process import sprung, worker
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    sprung.relaid(s['g'], (0.0, 0.0), 0.0)
    m = s['g']['world'].model
    body = [m.body(int(m.geom_bodyid[i])).name for i in range(m.ngeom)]
    return s, [i for i in range(m.ngeom) if body[i].startswith(HERS)], body


def _frame(g, d):
    """(her pelvis (x, z), her heading)."""
    import numpy as np
    from machine.senses import edge
    return np.array([float(d.qpos[0]), float(d.qpos[2])]), edge.yaw(g['sense'])


def _reach(m, d, hers, root, yaw, h):
    """`h` (bands, K) widened by her body about `root` facing `yaw`, bands added as her body spans them."""
    import numpy as np
    phi = 2.0 * np.pi * np.arange(K) / K
    f, left = np.array([math.sin(yaw), math.cos(yaw)]), np.array([math.cos(yaw), -math.sin(yaw)])
    dirs = np.cos(phi)[:, None] * f + np.sin(phi)[:, None] * left
    for i in hers:
        pts, r = _points(m, d, i)
        lo, hi = float(pts[:, 1].min()) - r, float(pts[:, 1].max()) + r
        first, last = max(0, int(math.floor(lo / BAND_M))), int(math.floor(max(hi, 0.0) / BAND_M))
        h = np.vstack([h, np.full((last + 1 - len(h), K), -np.inf)]) if last + 1 > len(h) else h
        h[first:last + 1] = np.maximum(h[first:last + 1], ((pts[:, [0, 2]] - root) @ dirs.T).max(axis=0) + r)
    return h


def _go(s, seconds, each=lambda: None, until=lambda: False):
    """`seconds` of passes or till `until`, `each` every ten passes."""
    from machine.process import worker
    g, t0 = s['g'], s['g']['t']
    while g['t'] - t0 < seconds and not until():
        worker.stepped(s)
        if s['fallen_at'] is not None:
            raise RuntimeError('fell at %.2f s' % s['fallen_at'])
        _ = g['n'] % 10 or each()


def _still(s):
    """Still as an errand turns her: settled, her rhythm still, her soles bearing alike (`dock.even`)."""
    from machine.way import dock, errand
    from machine.law import latent
    g = s['g']
    return errand._settled(g) > 0.0 and float(g['st'][latent.NOW][latent.GAIN]) < 1e-6 and dock.even(g)


def moved(mode):
    """((bands, K) her reach about her pelvis in her own frame over `mode`'s motion, [each swing's top
    over her steady walk, m])."""
    import numpy as np
    from machine.process import worker
    s, hers, body = _session()
    g, m, d = s['g'], s['g']['world'].model, s['g']['world'].data
    feet = {side: [i for i in hers if body[i] in ('%s_foot' % side, '%s_toes' % side)] for side in ('left', 'right')}
    out = {'h': np.full((1, K), -np.inf)}
    lows = {side: [] for side in feet}
    _go(s, SETTLE_S)
    for k, (seconds, twist) in enumerate(MOTIONS[mode]):
        worker.apply(s, {'twist': twist})
        t0 = g['t']

        def each():
            out['h'] = _reach(m, d, hers, *_frame(g, d), out['h'])
            steady = mode == 'walk' and k == 0 and g['t'] - t0 >= STEADY_S
            for side, ids in feet.items():
                lows[side] += [min(float(p[:, 1].min()) - r for p, r in (_points(m, d, i) for i in ids))] * steady
        _go(s, seconds, each)
    # each swing's top, the walk's first and last cut off by its window
    tops = [top for side in lows for top in [max(run) for up, run in itertools.groupby(
        lows[side], lambda v: v > leap.TOUCH_M) if up][1:-1]]
    return out['h'], tops


def turned(deg, way, walked_s, arc=0.0):
    """((bands, K) her reach over a turn in place by `deg` toward `way` (+1 left) and its settling, about
    her pelvis's place and heading as it began - walked `walked_s` turning `arc` rad/s, and still -,
    (forward, left) m her pelvis's end there, s it took)."""
    import numpy as np
    from machine.process import worker
    s, hers, _body = _session()
    g, m, d = s['g'], s['g']['world'].model, s['g']['world'].data
    _go(s, SETTLE_S)
    worker.apply(s, {'heading': 0.0, 'twist': (se2.WALK_SPEED, 0.0, arc)})
    _go(s, walked_s)
    worker.apply(s, {'twist': (0.0, 0.0, 0.0)})
    _go(s, TURN_MOST_S, until=lambda: _still(s))
    root, yaw = _frame(g, d)
    to = se2.wrap_angle(yaw + way * math.radians(deg))
    out = {'h': np.full((1, K), -np.inf), 'told': False}
    worker.apply(s, {'twist': (0.0, 0.0, way * se2.YAW_RATE)})

    def each():
        out['h'] = _reach(m, d, hers, root, yaw, out['h'])
        near = abs(se2.wrap_angle(to - _frame(g, d)[1])) <= modal.LIST_RAD
        _ = near and not out['told'] and (worker.apply(s, {'heading': to, 'twist': (0.0, 0.0, 0.0)}) or out.update(told=True))
    t0 = g['t']
    _go(s, TURN_MOST_S, each, until=lambda: out['told'] and _still(s))
    end = _frame(g, d)[0] - root
    return (out['h'], (float(end @ (math.sin(yaw), math.cos(yaw))), float(end @ (math.cos(yaw), -math.sin(yaw)))),
            g['t'] - t0)


def _relayed(specs):
    """{(deg, way, walked): `turned`'s} each its own process on the relay's batons (`focus.relay`)."""
    import json
    import numpy as np
    from tools.dev import focus
    jobs = [focus.Job('footprint_%d' % k, [sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--turn']
                      + [str(v) for v in spec], 0.6, 1800.0, None) for k, spec in enumerate(specs)]
    out = {}
    for job, text_, _code, _s in focus.relay(jobs):
        line = [ln for ln in (text_ or '').splitlines() if ln.startswith('{')]
        if not line:
            raise RuntimeError('%s: %s' % (job.name, (text_ or '')[-400:]))
        got = json.loads(line[-1])
        out[specs[int(job.name.split('_')[1])]] = (np.array(got['h']), tuple(got['carry']), got['took'])
    return out


def laid():
    """{'K', 'BAND_M', 'STEP_M', 'TURN_DEG', 'STAND', 'WALK', 'BACK', 'SIDE', 'LEFT', 'RIGHT', 'CARRY_*', 'TOOK_*'}:
    her footprint on her loop, as the file holds it."""
    import numpy as np
    out = {'K': K, 'BAND_M': BAND_M, 'TURN_DEG': TURN_DEG}
    swings = []
    for mode in MOTIONS:
        h, tops = moved(mode)
        out[mode.upper()], swings = h, swings + tops
    out['STEP_M'] = round(min(swings), 4)
    turns = _relayed([(deg, way, w, a) for way in (1.0, -1.0) for deg in TURN_DEG for w, a in WALKS])
    for name, way in (('LEFT', 1.0), ('RIGHT', -1.0)):
        runs = [[turns[(deg, way, w, a)] for w, a in WALKS] for deg in TURN_DEG]
        bands = max(len(h) for row in runs for h, _c, _t in row)
        pad = [[np.vstack([h, np.full((bands - len(h), K), -np.inf)]) for h, _c, _t in row] for row in runs]
        out[name] = [np.max(row, axis=0) for row in pad]
        out['CARRY_' + name] = tuple(tuple(round(float(v), 4) for v in np.mean([c for _h, c, _t in row], axis=0)) for row in runs)
        out['SPREAD_' + name] = tuple(round(float(max(np.hypot(*(np.array(c) - np.mean([c_ for _h, c_, _t in row], axis=0)))
                                                      for _h, c, _t in row)), 4) for row in runs)
        out['TOOK_' + name] = tuple(round(float(max(t for _h, _c, t in row)), 2) for row in runs)
    return out


def _rows(h):
    import numpy as np
    return tuple(tuple(round(float(v), 4) for v in np.maximum(row, 0.0)) for row in h)


def text(rows):
    """The file `rows` (`laid`) is written as."""
    lines = ['"""%s"""' % NOTICE, '',
             '#: Directions about her, K: the k-th at 2 pi k / K from her heading toward her left; her bands',
             '#: BAND_M m apart over the floor; her turns in place, deg.',
             'K, BAND_M, TURN_DEG = %d, %r, %r' % (rows['K'], rows['BAND_M'], rows['TURN_DEG']),
             "#: The least her swinging soles clear the floor at a swing's top over her steady walk, m.",
             'STEP_M = %r' % rows['STEP_M']]
    for name, what in (('STAND', 'Standing'), ('WALK', 'Walking and its stop'), ('BACK', 'Her walk back and its stop'),
                       ('SIDE', 'Her sidesteps either way and their stops')):
        lines += ['#: %s: each band from the floor up, her reach about her pelvis along each direction, m.' % what,
                  '%s = (' % name] + ['    %s,' % (row,) for row in _rows(rows[name])] + [')']
    for name in ('LEFT', 'RIGHT'):
        lines += ['#: A turn in place to her %s by each of TURN_DEG and its settling: each band, her reach about her' % name.lower(),
                  "#: pelvis's place and heading as it began, m.", '%s = (' % name]
        lines += ['    (%s),' % ''.join('\n        %s,' % (row,) for row in _rows(h)) for h in rows[name]]
        lines += [')', "#: Her pelvis's end after each, forward and left of where it began, m.",
                  'CARRY_%s = %r' % (name, rows['CARRY_' + name]),
                  '#: The most an end lay off that mean, m.', 'SPREAD_%s = %r' % (name, rows['SPREAD_' + name]),
                  '#: The most seconds each took from its word to still.', 'TOOK_%s = %r' % (name, rows['TOOK_' + name])]
    return '\n'.join(lines) + '\n'


def main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--write', action='store_true', help='write %s' % PATH)
    p.add_argument('--turn', nargs=4, type=float, metavar=('DEG', 'WAY', 'WALKED', 'ARC'), help='one turn, as JSON')
    args = p.parse_args(argv)
    if args.turn:
        import json
        h, carry, took = turned(*args.turn)
        print(json.dumps({'h': [[float(v) for v in row] for row in h], 'carry': carry, 'took': took}), flush=True)
        return 0
    rows = laid()
    print(text(rows) if not args.write else 'written %s' % PATH)
    if args.write:
        written(PATH, text(rows))
    return 0


if __name__ == '__main__':
    sys.exit(main())
