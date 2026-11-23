"""The robot's leg worn: its guards on the leg's sketch, its jeans laid over them, what cuts through.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/leg_worn.py OUT.png

The leg as leg_clamped draws it, the hip at rest above its thigh, the thigh's tube ahead of the knee's axis as its
build states it (`leg_clamped.stated`). The robot's guards (`coaxial.graphics.guards.laid`) lie where its page lays them - on the
figure's body beside its likeness (`form.worn`): a thigh's outside, a knee's cup, a shin - clear of what the sketch
stands there, each strut stood on the first part it meets on its way to its tube (`worn`). The jeans are the
likeness's as its page dresses the robot (`gynoid.body`), laid over the guards and the leg (`draped`). The leg's cables
(`knee_hose`) are the leg's too: the guards clear of the knee's however far it bends. What still cuts through what is
counted a triangle pair at a time (`clips.crossed`): none, or the sheet says how many; the cables against all else
through the knee's travel (`bent_cables`).
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clips  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_hose  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leg_clamped as lc  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: Cloth told in rows ROW_M up its limb and BEARINGS round it, split to FINE_M, laid GAP_M off what it covers - a
#: regular fit, not a slim one - and its push eased over EASE rows and bearings about; what it covers sampled STEP_M
#: apart, a strut stood that off a part.
ROW_M, BEARINGS, FINE_M, GAP_M, EASE, STEP_M = 0.005, 72, 0.004, 0.008, 8, 0.002
#: The jeans' legs as the page's likeness cuts them (`plated.BLENDS`).
JEANS = ('like_left_thigh', 'like_left_shank')
#: The leg's cables (`knee_hose`), and the parts each runs into - its channel's - never through.
CABLES = ('knee pipes', 'knee hose', 'ankle pipe', 'ankle hose')
CHANNELS = {'knee pipes': ('knee board cover', 'knee collar', 'knee saddle', 'knee shank seat'),
            'ankle pipe': ('fork',),
            'ankle hose': ('foot box',)}


def _rgb(material):
    """A corner's colour: its paint's where `lit.paint` packed one, else its palette's."""
    from coaxial.graphics.lit import PAINTED, PALETTE
    k = int(material)
    return ((k >> 16) & 255, (k >> 8) & 255, k & 255) if k & PAINTED else tuple(int(255 * c) for c in PALETTE[k & 3])


def _tris(corners, triangles, materials):
    return [(corners[t], _rgb(materials[t[0]])) for t in triangles]


def jeans(flex=0.0):
    """{part: tris}: the jeans' left legs as the page dresses the robot, the knee's frame (its thigh's, at the knee's
    pivot), the knee bent `flex` deg the way the sketch bends it."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness
    body = gynoid.body(dressed=True, like=likeness.FOUND, plates=True)
    root, names = ((0.0, 0.0, 0.0), np.eye(3)), [p[0] for p in body.parts]

    def posed(deg):
        angles = {'left_knee': deg} if deg else {}
        frames = body._frames(angles, root)
        pos, _n = body.pose(angles, root=root, _frames=frames)
        turn, spot = frames[names.index('left_thigh')][0], frames[names.index('left_shank')][1]
        return (pos - spot) @ turn
    owner, index = np.repeat(np.arange(len(names)), [len(p[5][1]) for p in body.parts]), body.index.astype(int)
    shin, deg = np.unique(index[owner == names.index(JEANS[1])]), flex
    if flex:                                             # the page's sign for the knee's bend, told by the shin's way
        ahead = posed(flex)[shin, 2].mean() - posed(0.0)[shin, 2].mean()
        sketch = km._turn([(np.array([[0.0, -0.3, 0.0]] * 3), (0, 0, 0))], flex)[0][0][0, 2]
        deg = flex if (ahead > 0.0) == (sketch > 0.0) else -flex
    at = posed(deg)
    return {n: _tris(at, index[owner == names.index(n)], body.materials) for n in JEANS}


def _standing(tree):
    """A strut's stand (`guards._strutted`): on from its foot toward its tube to where it first comes within its own
    radius and STEP_M of a part it passes."""
    import numpy as np
    from machine.body.guards import STRUT_R

    def stand(foot, on):
        way = np.asarray(on, float) - foot
        length = float(np.linalg.norm(way))
        return next((foot + way * (s / length) for s in np.arange(STEP_M, length, 0.5 * STEP_M)
                     if tree.query(foot + way * (s / length))[0] < STRUT_R + STEP_M), on)
    return stand


def worn(parts, lifts, tubes, swept=None):
    """{segment: tris}: the robot's guards on each segment `lifts` names - {segment: its frame's origin in the
    joint's frame} - on the figure's body, clear of `parts` {name: tris} and of what moves by it (`swept` {segment:
    tris}, never stood on) by the page's rule (`plated.cleared`), each strut toward its segment's tube among the parts
    (`tubes` {segment: name}) stood on what it meets first; the joint's frame."""
    import numpy as np
    from scipy.spatial import KDTree
    from coaxial.graphics import form, likeness, plated
    from coaxial.graphics import guards as gd
    from machine.body.guards import NEAR_M
    pts = km._points([t for v in parts.values() for t in v], STEP_M)
    body = {part[1]: part[-1][0] for part in form.worn(likeness.FOUND) if part[0].startswith('form_')}
    surfaces, runs, stands = {}, {}, {}
    turns = [np.array([[math.cos(a), 0.0, -math.sin(a)], [0.0, 1.0, 0.0], [math.sin(a), 0.0, math.cos(a)]])
             for a in (-math.pi / gd.SECTORS, 0.0, math.pi / gd.SECTORS)]
    for seg, up in lifts.items():
        at, corners, moving = pts - up, body[seg], (swept or {}).get(seg)
        near = np.vstack([at, km._points(moving, STEP_M) - up]) if moving else at
        low, high = float(corners[:, 1].min()) - plated.DRUM_M, float(corners[:, 1].max()) + plated.DRUM_M
        reach = 0.25 if seg == 'torso' else NEAR_M
        by = near[(near[:, 1] > low) & (near[:, 1] < high) & (np.hypot(near[:, 0], near[:, 2]) < reach)]
        # Each part told in the half-cells about its own too: a guard's face, read between its cells' middles, keeps its
        # air off a part as thin as a hose wherever between them it stands without smearing inboard clips across ahead.
        by = np.vstack([by @ t.T + (0.0, dy, 0.0) for t in turns for dy in (-0.5 * gd.ROW_M, 0.0, 0.5 * gd.ROW_M)])
        surfaces[seg] = (corners, by)
        tube = np.concatenate([tri for tri, _c in parts[tubes[seg]]]) - up
        mid = tube.mean(axis=0)
        way = np.linalg.svd(tube - mid)[2][0]
        reach = (tube - mid) @ way
        runs[seg], stands[seg] = [(mid + way * reach.min(), mid + way * reach.max(), 0.0)], _standing(KDTree(at))
    out = {}
    for _name, seg, _offset, (corners, triangles, _uv, materials) in gd.laid(surfaces, runs, stands):
        out[seg] = _tris(np.asarray(corners) + lifts[seg], np.asarray(triangles, int), np.asarray(materials))
    return out


def _travel(step):
    """The knee's turns from straight `step` deg apart to the end of its travel (`drives.TRAVEL`), deg."""
    import numpy as np
    from machine.body import drives
    end = drives.TRAVEL['knee'][1]
    return np.append(np.arange(0.0, end, step), end)


@functools.lru_cache(maxsize=None)
def guards(ahead, step=15.0):
    """{segment: tris}: the robot's guards on the leg's thigh and shank, the knee's frame (`worn`): clear of the leg,
    of its cranks swept and of its cables, the knee's bent `step` deg at a time - as the thigh sees them on the
    thigh's, as the shank does on the shank's."""
    import numpy as np
    from machine.body.figure import THIGH
    _p, _c, still, turning = kc.bodies(ahead=ahead)
    parts = dict({'knee ' + k: v for k, v in dict(still, **turning).items()}, **lc.thigh(ahead), **lc.hip(), **lc.shank())
    bent = [(knee_hose.knee(float(d)), float(d)) for d in _travel(step)]
    moving = {'left_thigh': [t for c, _d in bent for v in c.values() for t in v],
              'left_shank': [t for c, d in bent for v in c.values() for t in km._turn(v, -d)] + lc.swept()
              + [t for v in knee_hose.ankle(lc.under(), lc.shank()['fork']).values() for t in v]}
    return worn(parts, {'left_thigh': np.array([0.0, THIGH, 0.0]), 'left_shank': np.zeros(3)},
                {'left_thigh': 'thigh tube', 'left_shank': 'shank tube'}, moving)


def draped(cloth, under, worn=(), fall=None):
    """(`cloth` laid over what is `under` it and what is `worn` over that, the farthest a corner moved, m): split to
    FINE_M, each corner pushed out from its limb's axis (the frame's y) till it stands GAP_M past the farthest of them at
    its row and bearing round it, the push eased over EASE rows and bearings about - cloth stretched over a guard falls
    back to the limb -; under the height `fall`, where a trouser leg hangs, each bearing as wide as the widest it passes
    on its way down. Of `under` only what stands by the limb counts, as the page's guards count it
    (`machine.body.guards.NEAR_M`); what is `worn` - the guards - wherever it stands."""
    import numpy as np
    from scipy.ndimage import maximum_filter1d, uniform_filter1d
    from machine.body.guards import NEAR_M
    tris = clips.finer(cloth, FINE_M)
    c, p = np.array([t for t, _c in tris]).reshape(-1, 3), km._points(under, STEP_M)
    p = np.vstack([p[np.hypot(p[:, 0], p[:, 2]) < NEAR_M]] + ([km._points(list(worn), STEP_M)] if worn else []))
    row = np.floor(c[:, 1] / ROW_M).astype(int)
    r0, n = int(row.min()), int(row.max() - row.min()) + 1
    row -= r0

    def polar(q):
        d = q[:, [0, 2]]
        bear = ((np.arctan2(d[:, 1], d[:, 0]) + math.pi) / (2.0 * math.pi) * BEARINGS).astype(int) % BEARINGS
        return np.hypot(d[:, 0], d[:, 1]), bear, d
    rows = np.floor(p[:, 1] / ROW_M).astype(int) - r0
    by = (rows >= 0) & (rows < n)
    p, rows = p[by], rows[by]
    need, push = np.zeros((n, BEARINGS)), np.zeros((n, BEARINGS))
    if len(p):
        r_p, b_p, _d = polar(p)
        np.maximum.at(need, (rows, b_p), r_p + GAP_M)
    r_c, b_c, d = polar(c)
    want = np.maximum(need[row, b_c] - r_c, 0.0)
    np.maximum.at(push, (row, b_c), want)
    size, eased = 2 * EASE + 1, push
    for ease in (maximum_filter1d, uniform_filter1d):     # up the limb to its ends, round it all the way
        eased = ease(ease(eased, size, axis=0, mode='nearest'), size, axis=1, mode='wrap')
    move = np.maximum(eased[row, b_c], want)
    hang = int(math.floor(fall / ROW_M)) - r0 if fall is not None else 0
    if hang > 0:                                         # from the knee down it falls straight
        wide = np.zeros((n, BEARINGS))
        np.maximum.at(wide, (row, b_c), r_c + move)
        wide[:hang] = np.maximum.accumulate(wide[:hang][::-1], axis=0)[::-1]
        move = np.where(row < hang, np.maximum(move, wide[row, b_c] - r_c), move)
    scale = (r_c + move) / np.maximum(r_c, 1e-9)
    c[:, 0], c[:, 2] = d[:, 0] * scale, d[:, 1] * scale
    return [(t, colour) for t, (_old, colour) in zip(c.reshape(-1, 3, 3), tris)], float(move.max())


def dressed(ahead, flex=0.0):
    """({name: tris} the leg's sketch and its guards ('guards ...'), the jeans laid over them, the farthest they
    moved, m), the knee bent `flex` deg: each of the jeans' legs laid in its own segment's frame - straight, both
    one frame, as one cloth, its push eased across their seam."""
    up, down = _bare(ahead, flex)
    (ups, downs), (on_up, on_down) = ([[t for k, v in d.items() if k.startswith('guards ') == worn for t in v]
                                       for d in (up, down)] for worn in (False, True))
    cloth, bare = jeans(flex), dict(up, **{k: km._turn(v, flex) for k, v in down.items()})
    if not flex:                                         # the legs fall straight from the knee's axis down
        return (bare, *draped(cloth[JEANS[0]] + cloth[JEANS[1]], ups + downs, on_up + on_down, 0.0))
    upper, moved_u = draped(cloth[JEANS[0]], ups + km._turn(downs, flex), on_up + km._turn(on_down, flex))
    lower, moved_l = draped(km._turn(cloth[JEANS[1]], -flex), km._turn(ups, -flex) + downs,
                            km._turn(on_up, -flex) + on_down, 0.0)
    return bare, upper + km._turn(lower, flex), max(moved_u, moved_l)


def _bare(ahead, flex=0.0):
    """({name: tris} the thigh's side, {name: tris} the shank's, its own frame): the leg's sketch, its guards and its
    cables, the knee's bent `flex` deg on the thigh's side."""
    _p, _c, still, turning = kc.bodies(ahead=ahead)
    put = guards(ahead)
    up = dict({'knee ' + k: v for k, v in still.items()}, **lc.thigh(ahead), **lc.hip(), **knee_hose.knee(flex),
              **{'guards thigh': put['left_thigh']})
    down = dict({'knee ' + k: v for k, v in turning.items()}, **lc.shank(), **knee_hose.ankle(lc.under(), lc.shank()['fork']),
                **{'guards shank': put['left_shank']})
    return up, down


def bent_cables(ahead, step=15.0):
    """{cable: triangle pairs crossing}: each of the leg's cables (`knee_hose`) against all else of the leg and its
    guards but the channels it runs into (`CHANNELS`), the knee bent `step` deg at a time through its travel."""
    out = {}
    for d in _travel(step):
        up, down = _bare(ahead, float(d))
        parts = dict(up, **{k: km._turn(v, float(d)) for k, v in down.items()})
        for cable in CABLES:
            rest = [t for k, v in parts.items() if k not in CABLES + CHANNELS.get(cable, ()) for t in v]
            out[cable] = out.get(cable, 0) + clips.crossed(parts[cable], rest)
    return out


def bent_guards(step=10.0):
    """{joint: triangle pairs crossing}: the page's robot bare, each elbow's and knee's guards against those across
    it, the joint bent `step` deg at a time to the end of its travel (`drives.TRAVEL`)."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness
    from machine.body import drives
    body = gynoid.body(dressed=False, like=likeness.FOUND, plates=True)
    root, names = ((0.0, 0.0, 0.0), np.eye(3)), [p[0] for p in body.parts]
    owner, index = np.repeat(np.arange(len(names)), [len(p[5][1]) for p in body.parts]), body.index.astype(int)
    out = {}
    for joint, a, b in (('left_elbow', 'upper_arm', 'forearm'), ('left_knee', 'thigh', 'shank')):
        end, out[joint] = drives.TRAVEL[joint.partition('_')[2]][1], 0
        for deg in np.append(np.arange(step, end, step), end):
            frames = body._frames({joint: float(deg)}, root)
            pos, _n = body.pose({joint: float(deg)}, root=root, _frames=frames)
            a_tris, b_tris = ([(t, 0) for t in pos[index[owner == names.index('guard_left_' + seg)]]] for seg in (a, b))
            out[joint] += clips.crossed(a_tris, b_tris)
    return out


def counted(bare, cloth):
    """{'cloth x guards', 'cloth x limb', 'guards x limb', 'guards x guards': triangle pairs crossing} of a worn
    limb (`clips.crossed`), the last its segments' guards against each other."""
    guards = [v for k, v in bare.items() if k.startswith('guards ')]
    guard = [t for v in guards for t in v]
    limb = [t for k, v in bare.items() if not k.startswith('guards ') for t in v]
    return {'cloth x guards': clips.crossed(cloth, guard), 'cloth x limb': clips.crossed(cloth, limb),
            'guards x limb': clips.crossed(guard, limb),
            'guards x guards': sum(clips.crossed(a, b) for i, a in enumerate(guards) for b in guards[i + 1:])}


def sheet(path, title, rows, px=600):
    """Lay the views `rows` [[(name, tris, yaw, pitch)]] - each row framed on its own - under `title`, at `path`."""
    from PIL import Image, ImageDraw
    out = Image.new('RGB', (4 * px, len(rows) * (px + 30) + 30), (18, 18, 18))
    pen = ImageDraw.Draw(out)
    pen.text((12, 8), title, fill=(235, 235, 235))
    for j, row in enumerate(rows):
        centre, half = kc._framed([t for _n, tris, _y, _p in row for t in tris])
        for i, (name, tris, yaw, pitch) in enumerate(row):
            out.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=px, half=half, centre=centre), (px * i, 60 + (px + 30) * j))
            pen.text((px * i + 12, 40 + (px + 30) * j), name, fill=(200, 200, 200))
    out.save(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    lead = lc.stated()
    bare, cloth, push = dressed(lead)
    told = counted(bare, cloth)
    bent = [t for v in dressed(lead, 90.0)[0].values() for t in v]     # bare: no cloth laid over a bent knee here
    every = [t for v in bare.values() for t in v]
    views = (('from ahead', 35.0, 12.0), ('from the side', 90.0, 6.0), ('from behind', -145.0, 12.0))
    rows = [[('guards on, ' + name, every, yaw, pitch) for name, yaw, pitch in views]
            + [('the knee bent 90 deg', bent, 70.0, 12.0)],
            [('in its jeans, ' + name, every + cloth, yaw, pitch) for name, yaw, pitch in views]
            + [('in its jeans, from its inside', every + cloth, -35.0, 12.0)]]
    sheet(args.out, "leg worn: the robot's guards on the sketch, the likeness's jeans laid over them - out %.0f mm at"
          " the most; cutting through: %s" % (1e3 * push, told), rows)
    print(args.out, 'thigh %.0f mm ahead; jeans out %.1f mm; crossing %s' % (1e3 * lead, 1e3 * push, told))
    return 0


if __name__ == '__main__':
    sys.exit(main())
