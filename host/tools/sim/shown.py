#!/usr/bin/env python3
"""What shows through the robot as drawn: its drives past their covers, its body past its clothes.

    python tools/sim/shown.py                 # dressed and as its shell, a pose a row
    python tools/sim/shown.py --look shell
    python tools/sim/shown.py --out           # its likeness: what stands out of its clothes or tears, a pose a row

Each drive's drum (`drums.drums`) is sampled on its faces and posed with the body as drawn
(`gynoid.Body`): what of it stands outside every drawn part is summed, cm2 - a point inside a
closed part by its winding number. The drums stood 88-216 cm2 a joint out of the shoulders and
the elbows in each of 43 poses, 37 mm past the shoulder's shell. Dressed, the bare body's shell
that its clothes cover standing is posed beside the clothes as they bend (`drape`), and what of
it stands outside them summed: what the clothes fail to follow. Blended between two frames the
cloth left the cheeks 145 and the pelvis 105 cm2 out at a hip bent 120 deg.
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np

#: A triangle's samples: its 4 quarters' middles, as its corners' weights.
QUARTERS = np.array([(4.0, 1.0, 1.0), (1.0, 4.0, 1.0), (1.0, 1.0, 4.0), (2.0, 2.0, 2.0)]) / 6.0
#: Points a pass of the winding sum takes at once.
AT_ONCE = 1500
#: The joints told a side each, and the poses: every joint over its turn, and the robot's own.
SIDED = ('shoulder', 'elbow', 'hip', 'knee', 'ankle', 'hip_yaw', 'hip_roll', 'ankle_roll', 'wrist')
TURNS = (('shoulder', (-60, 60, 90, 135, 180)), ('elbow', (45, 90, 150)), ('hip', (-120, -90, -60, -30, 20)),
         ('knee', (45, 90, 150)), ('spine', (-30, 30, 60, 85)), ('spine_roll', (-35, 35)), ('waist', (-50, 50)),
         ('hip_yaw', (-25, 25)), ('hip_roll', (-20, 20)), ('ankle', (-50, 45)), ('neck', (-45, 60)),
         ('head', (-80, 80)))
ROOT = ((0.0, 0.0, 0.0), np.eye(3))


def told(**turns):
    """A stand with `turns` {joint kind: deg}, a sided kind both sides'."""
    from machine.body import figure
    out = figure.stand()
    for kind, deg in turns.items():
        out.update({side + kind: float(deg) for side in (('left_', 'right_') if kind in SIDED else ('',))})
    return out


def poses():
    """[(name, {joint: deg})]: the stand, the squat, a seat, a bow, a stride, and each joint alone."""
    from machine.body import figure
    stand = figure.stand()
    stride = dict(stand, left_hip=-35.0, left_knee=25.0, right_hip=20.0, right_knee=35.0, left_shoulder=-25.0,
                  right_shoulder=30.0, left_elbow=30.0, right_elbow=40.0)
    return [('stand', stand), ('squat', dict(figure.SQUAT)), ('sit', told(hip=-90, knee=90, ankle=0, spine=10)),
            ('bow', told(spine=60, hip=-70, knee=20, ankle=-10, shoulder=40)), ('stride', stride)] + [
                ('%s %+d' % (kind, deg), told(**{kind: deg})) for kind, degs in TURNS for deg in degs]


def _sampled(corners, triangles):
    """(points, m2 each) on a mesh's faces: QUARTERS of each triangle."""
    a, b, c = (corners[triangles[:, k]] for k in range(3))
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1) / len(QUARTERS)
    return np.einsum('sk,tkx->tsx', QUARTERS, corners[triangles]).reshape(-1, 3), np.repeat(area, len(QUARTERS))


def _winding(points, a, b, c):
    """How many times the closed mesh of triangles (a, b, c) winds about each point."""
    out = np.empty(len(points))
    for k in range(0, len(points), AT_ONCE):
        pa, pb, pc = (v[None] - points[k:k + AT_ONCE, None] for v in (a, b, c))
        la, lb, lc = (np.linalg.norm(v, axis=2) for v in (pa, pb, pc))
        solid = np.arctan2(np.einsum('ptk,ptk->pt', pa, np.cross(pb, pc)),
                           la * lb * lc + np.einsum('ptk,ptk->pt', pa, pb) * lc
                           + np.einsum('ptk,ptk->pt', pb, pc) * la + np.einsum('ptk,ptk->pt', pc, pa) * lb)
        out[k:k + AT_ONCE] = solid.sum(axis=1) / (2.0 * math.pi)
    return out


class Drawn:

    """A body as drawn at a pose: each of its parts' triangles, world, and whether a point is under any."""

    def __init__(self, body, angles):
        self.body, self.frames = body, body._frames(angles, ROOT)
        self.pos, _normals = body.pose(angles, root=ROOT, _frames=self.frames)
        index, part = body.index.astype(int), np.repeat(
            np.arange(len(body.parts)), [len(p[5][1]) for p in body.parts])
        self.parts = [(p[0], self.pos[index[part == i]]) for i, p in enumerate(body.parts) if len(p[5][1])]

    def under(self, points, among=lambda name: True):
        """Whether each of `points` stands inside a part `among` names."""
        inside = np.zeros(len(points), bool)
        for name, tri in self.parts:
            box = tri.reshape(-1, 3)
            near = ~inside & ((points >= box.min(0)) & (points <= box.max(0))).all(axis=1)
            if among(name) and near.any():
                inside[near] = np.abs(_winding(points[near], tri[:, 0], tri[:, 1], tri[:, 2])) > 0.5
        return inside


def drives(body, angles):
    """{joint: cm2}: each drive's drum's faces outside every part of `body` as drawn at `angles`."""
    from coaxial.graphics import drums
    drawn, out = Drawn(body, angles), {}
    for name, _parent, _at, (corners, triangles, _uv, _m) in drums.drums():
        turn, spot = drawn.frames[body.drums[name[len('drive_'):]]]
        points, area = _sampled(corners @ turn.T + spot, triangles)
        out[name[len('drive_'):]] = 1e4 * float(area[~drawn.under(points)].sum())
    return out


def _cloth(name):
    return name.startswith('cloth_')


def followed():
    """`past(angles)` -> {part: cm2}: of the bare body's shell that its clothes cover standing,
    what stands outside the dressed body as drawn at `angles`."""
    from coaxial.graphics import gynoid
    from machine.body import figure
    bare, dressed = gynoid.body(dressed=False), gynoid.body()
    shell = [i for i, p in enumerate(bare.parts) if len(p[5][1]) and not p[0].startswith(('drive_', 'release_'))]

    def at(angles):
        pos, _normals = bare.pose(angles, root=ROOT)
        return [_sampled(pos[slice(*bare.spans[i])], bare.parts[i][5][1]) for i in shell]

    clothes = Drawn(dressed, figure.stand())
    worn = [clothes.under(points, _cloth) for points, _area in at(figure.stand())]

    def past(angles):
        drawn = Drawn(dressed, angles)
        return {bare.parts[i][0]: 1e4 * float(area[under][~drawn.under(points[under])].sum())
                for i, under, (points, area) in zip(shell, worn, at(angles)) if under.any()}
    return past


#: What stands out of a garment, from no side: about a segment's bone as it is posed, cloth
#: girds a slab SLAB_M thick where it stands in OF of WAYS ways round within FAR_M; a point in
#: a slab girt with both its neighbours - a slab and more within a garment's end - stands out
#: where it reaches OUT_M past the furthest cloth its way. Told from a side - a part's dots
#: with cloth close behind them - an arm hung beside its trunk was counted for the trunk's
#: tee behind it, 1011 dots in 42 poses of which the eye found none. What stands by a
#: garment's opening - in its last girt slab or the one past it - within the cloth's reach
#: there is held too: a shoe under a hem. Of what stands past a girt slab no more is held: an
#: arm bent into a hip is the pose's.
SLAB_M, WAYS, OF, OUT_M, FAR_M = 0.02, 24, 18, 0.006, 0.25
#: Cloth tears where an edge of it grows past TORN times its own length - its length standing
#: times how folded its corners lie there (a likeness's: its uv's second, `likeness.worn`), a
#: dot's at the least (`shapes.DENSE_M`): in the tried poses a bend stretched one 4.4 times at
#: the most (a knee at 150 deg; a trunk bowed 85 deg 4.3), and a corner its neighbours left
#: behind 7.6 to 15 times - a knee's cloth masked by its side, a seat's middle. Told by its
#: length standing alone, a sleeve folded at its armpit where the figure's arms were hung
#: opened 7.6 to 10.9 times as its arm rose - the fold opening, no tear.
TORN = 6.0
#: The poses a garment is tried in beside `poses`: a walk's arm swings, an arm raised, a reach.
MOVES = (('swing', dict(left_shoulder=-35.0, right_shoulder=35.0, left_elbow=25.0, right_elbow=45.0, left_hip=-30.0,
                        right_hip=25.0, left_knee=20.0, right_knee=40.0)),
         ('raise', dict(left_shoulder=160.0, right_shoulder=100.0, left_elbow=10.0, right_elbow=90.0)),
         ('reach', dict(left_shoulder=75.0, right_shoulder=75.0, left_elbow=20.0, right_elbow=20.0, spine=20.0)))


def _boned(body, frames):
    """{segment: (its joint, its bone's way)} of `body` as its `frames` pose it: a limb's bone
    from its joint to the next's, a hand's and a foot's their limb's on, the trunk's its own
    upright."""
    from machine.body import figure
    placed = {part[0]: frames[k] for k, part in enumerate(body.parts) if frames[k] is not None}
    kids, out = {}, {}
    for name, parent, *_rest in figure.SEGMENTS:
        kids.setdefault(parent, []).append(name)
    for name, parent, *_rest in figure.SEGMENTS:
        on = [kid for kid in kids.get(name, ()) if kids.get(kid)]
        if not name.startswith(('left_', 'right_')):
            way = placed[name][0][:, 1]
        elif on:
            way = placed[on[0]][1] - placed[name][1]
            way = way / np.linalg.norm(way)
        else:
            way = out[parent][1]
        out[name] = (placed[name][1], way)
    return out


def girded(cover, joint, way):
    """`told(points)` -> (within, past, edge, beyond) about the bone from `joint` along `way`:
    whether each of `points` stands in a slab the cloth `cover` (m, 3) girds with both its
    neighbours (SLAB_M, OF of WAYS ways within FAR_M), how far past the furthest cloth its way it
    reaches, m (-inf with none that way); whether it stands by a girt run's end instead - its
    last slab or the one past it -, and how far past the furthest cloth its way in its slab and
    the two beside it (inf with none)."""
    n = int(2.0 / SLAB_M)
    aside = np.cross(way, (0.0, 0.0, 1.0) if abs(way[2]) < 0.9 else (1.0, 0.0, 0.0))
    aside = aside / np.linalg.norm(aside)

    def laid(points):
        off = points - joint
        t, x, z = off @ way, off @ aside, off @ np.cross(way, aside)
        return (np.clip((t + 1.0) / SLAB_M, 0, n - 1).astype(int),
                np.floor(np.arctan2(z, x) / (2.0 * math.pi) * WAYS).astype(int) % WAYS, np.hypot(x, z))
    slab, ways, reach = laid(np.asarray(cover, float).reshape(-1, 3))
    far = np.full((n, WAYS), -np.inf)
    np.maximum.at(far, (slab[reach < FAR_M], ways[reach < FAR_M]), reach[reach < FAR_M])
    girt = np.isfinite(far).sum(axis=1) >= OF
    within = girt & np.roll(girt, 1) & np.roll(girt, -1)
    edge = (girt | np.roll(girt, 1) | np.roll(girt, -1)) & ~within
    near = np.maximum(far, np.maximum(np.roll(far, 1, axis=0), np.roll(far, -1, axis=0)))

    def told(points):
        slab, ways, reach = laid(points)
        return (within[slab], np.where(np.isfinite(far[slab, ways]), reach - far[slab, ways], -np.inf), edge[slab],
                np.where(np.isfinite(near[slab, ways]), reach - near[slab, ways], np.inf))
    return told


def _held(body, angles, covers):
    """[(part, its dense span, {cover: `girded`'s four})] of `body` at `angles`: each part that
    hangs on a segment, its dense points told (`girded`) about its segment's bone as posed by
    each of the cloths `covers` {name: which dense points}."""
    from machine.body import figure
    frames = body._frames(angles, ROOT)
    pos, _normals = body.pose(angles, dense=True, root=ROOT, _frames=frames)
    segs = {seg[0]: seg[1] for seg in figure.SEGMENTS}
    turned = {joint: seg[0] for seg in figure.SEGMENTS for joint, *_rest in seg[2]}
    up, bones, told, out = {part[0]: part[1] for part in body.parts}, _boned(body, frames), {}, []
    for part, (lo, hi) in zip(body.parts, body.dense_spans):
        seg = part[0]
        if part[1] == '*':                     # a crank, a pin, a rod: along the segment over the one its joint turns
            seg = segs.get(turned.get(seg.split('_', 1)[1]))
        else:
            while seg not in segs:             # up its stages to the segment it hangs on
                seg = up[seg]
        if hi <= lo or seg is None:
            continue
        for name, cover in covers.items():
            if (seg, name) not in told:
                told[(seg, name)] = girded(pos[cover], *bones[seg])
        out.append((part[0], (lo, hi), {name: told[(seg, name)](pos[lo:hi]) for name in covers}))
    return out


_WORN = {}


def held(body):
    """Which dense points of `body` its clothes are held to: of each part that is no cloth and
    none the robot wears over all (`plated.WORN`), those a garment girds as it stands and those
    by its opening within its cloth's reach there; of a garment worn under another
    (`lit.UNDER`), all. Told once a body."""
    if id(body) not in _WORN:
        from coaxial.graphics import lit, plated
        inks = body.dense[3]
        cloth = (inks & lit.WOVEN) > 0
        worn = cloth & ((inks & lit.UNDER) > 0)
        for name, (lo, hi), told in _held(body, {}, {'cloth': cloth}):
            if not cloth[lo] and name not in plated.WORN:
                within, _past, edge, _beyond = told['cloth']
                worn[lo:hi] = within | edge
        _WORN[id(body)] = worn
    return _WORN[id(body)]


def stood_out(body, angles):
    """{part: points}: what of `body` at `angles` stands out of its clothes. Of what its clothes
    are held to (`held`): a part's points that reach past the cloth as it is posed (SLAB_M,
    WAYS, OF, OUT_M), a garment's under another those that reach past the one over it. A
    point no garment girds standing, nor stands by its opening - a hand, a forearm under a
    short sleeve, a shoe's toes - is held to none: bent into a hip it is the pose's fault,
    no garment's. From no side: what a view would have to find in a silhouette is told where
    it stands."""
    from coaxial.graphics import lit
    inks = body.dense[3]
    cloth = (inks & lit.WOVEN) > 0
    worn, out = held(body), {}
    for name, (lo, hi), told in _held(body, angles, {'cloth': cloth, 'over': cloth & ((inks & lit.UNDER) == 0)}):
        within, past, _edge, _beyond = told['over' if cloth[lo] else 'cloth']
        count = int((within & (past > OUT_M) & worn[lo:hi]).sum())
        if count:
            out[name] = count
    return out


def stretched(body, angles):
    """{garment: (edges torn, how many times its own length the worst grew)} of `body` at
    `angles`: of each garment's triangles (a part that is cloth and none the robot wears over
    all), its edges grown past TORN times their own length (TORN), and the most any grew."""
    from coaxial.graphics import lit, plated
    from coaxial.graphics.shapes import DENSE_M
    from machine.body import figure
    rest, _normals = body.pose(figure.stand())
    pos, _normals = body.pose(angles)
    out = {}
    for (name, *_head, (_corners, triangles, uv, inks)), (lo, _hi) in zip(body.parts, body.spans):
        if not len(triangles) or name in plated.WORN or not ((inks & lit.WOVEN) > 0).all():
            continue
        t, folded = triangles + lo, np.maximum(uv[:, 1], 1.0)
        b = np.roll(t, 1, axis=1)
        own = np.linalg.norm(rest[t] - rest[b], axis=2) * np.maximum(folded[triangles], folded[np.roll(triangles, 1, axis=1)])
        grown = np.linalg.norm(pos[t] - pos[b], axis=2) / np.maximum(own, DENSE_M)
        out[name] = (int((grown > TORN).sum()), float(grown.max()))
    return out


def torn(body, angles):
    """{garment: (edges, how many times its length the worst grew)}: where the clothes of `body`
    at `angles` tear (`stretched`)."""
    return {name: got for name, got in stretched(body, angles).items() if got[0]}


def tried():
    """[(name, {joint: deg})]: the poses a garment is tried in - `poses` and MOVES."""
    from machine.body import figure
    return poses() + [(name, dict(figure.stand(), **turns)) for name, turns in MOVES]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--look', choices=('dressed', 'shell'), action='append', help='one look alone')
    parser.add_argument('--out', nargs='?', const='', help="what stands out of the likeness's clothes and where they "
                        "tear, a pose a row: a figure's file, else the one found")
    args = parser.parse_args(argv)
    from coaxial.graphics import gynoid
    if args.out is not None:
        from coaxial.graphics import likeness
        body = gynoid.body(like=args.out or likeness.FOUND)
        got = [(name, stood_out(body, angles), stretched(body, angles)) for name, angles in tried()]
        print('the likeness: points of the robot that stand out of its clothes, and edges of cloth torn, a pose a row')
        for name, parts_, grown in got:
            tears = {k: v for k, v in grown.items() if v[0]}
            if parts_ or tears:
                print('  %-14s %5d  %s' % (name, sum(parts_.values()), ', '.join(
                    ['%s %d' % kv for kv in sorted(parts_.items(), key=lambda kv: -kv[1])[:6]]
                    + ['%s torn %d, x%.1f' % (k, n, worst) for k, (n, worst) in tears.items()])))
        worst = max(((most, name, garment) for name, _p, grown in got for garment, (_n, most) in grown.items()), default=(0.0, '', ''))
        print('  %d points in all of %d held, %d poses of %d stood out of; %d edges torn, an edge stretched x%.1f at the most '
              '(%s, %s; torn past x%.0f)' % (
                  sum(sum(parts_.values()) for _n, parts_, _g in got), int(held(body).sum()),
                  sum(bool(parts_) for _n, parts_, _g in got), len(got),
                  sum(n for _n, _p, grown in got for n, _w in grown.values()), worst[0], worst[1], worst[2], TORN))
        return 1 if any(parts_ or any(n for n, _w in grown.values()) for _n, parts_, grown in got) else 0
    for look in args.look or ('dressed', 'shell'):
        body, past = gynoid.body(dressed=look == 'dressed'), followed() if look == 'dressed' else None
        print('%s, cm2 a pose: its drives past what covers them%s' % (
            look, '; its body, as its clothes cover it standing, past them' if past else ''))
        seen = {}
        for name, angles in poses():
            out = {'drive ' + k: v for k, v in drives(body, angles).items()}
            out.update(past(angles) if past else {})
            rows = sorted(((v, k) for k, v in out.items() if v > 0.05), reverse=True)
            seen[name] = sum(v for v, _k in rows)
            print('  %-14s %6.1f  %s' % (name, seen[name], ', '.join('%s %.0f' % (k, v) for v, k in rows[:6])))
        clean = sum(v < 0.05 for v in seen.values())
        print('  %d of %d poses clean, the worst %s %.0f cm2' % (
            clean, len(seen), *max(seen.items(), key=lambda kv: kv[1])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
