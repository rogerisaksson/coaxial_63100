#!/usr/bin/env python3
"""A sewn garment's FBX thinned to an OBJ a likeness wears: its pieces welded, a garment a group.

    python tools/sim/sewn.py GARMENTS.fbx                    # under worn/ beside the figure found
    python tools/sim/sewn.py GARMENTS.fbx --beside FIGURE.obj --pitch 0.014
    python tools/sim/sewn.py MANNEQUIN.fbx --body            # under formed/ beside it: the robot's form

A cloth simulator's file holds a garment as its pattern's pieces, a mesh each, sewn where their
corners meet, and its stitches as threads' meshes. Pieces sharing SEAM corners or more are one
garment; a garment is thinned - its corners within a cell `pitch` a side made one, their mean,
which welds its seams - and written its pieces a group each, `sewn0.` the highest's. What is
under SHARE of the largest's area - a stitch's thread, a button - is left. The likeness lays the
garments on its figure (`coaxial.graphics.garment`) and bends them as the rest of it: the folds
a simulator laid, at no cost a frame. A mannequin's body (`--body`) is stood upright (`stood`),
thinned and written whole: the form the robot's shell is made of (`coaxial.graphics.form`).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np

#: Corners two pieces share to be one garment's: a stitch's thread shares 4-11 with the cloth it
#: lies on, a seam 31-343 (a tee and jeans, 53 pieces); the cell two corners are the same in, m;
#: the least share of the largest garment's area that is a garment.
SEAM, SAME_M, SHARE = 16, 0.0005, 0.1
#: A thinned corner's cell, m: a tee and jeans of 148 542 triangles come to 18 190 at it, a frame
#: of the robot wearing them 21.8 ms beside 20.9 as built (23 625 triangles at 12 mm); a fold
#: 30 mm across keeps two corners. A body stands HEIGHT_M tall where none is told: a shop's
#: mannequin's.
PITCH_M, HEIGHT_M = 0.014, 1.75


def pieces(path):
    """[(corners (n, 3) m, triangles (k, 3))] of an FBX's meshes, a pattern's piece each: polygons
    fanned, centimetres read as such."""
    from tools.sim import fbx
    with open(path, 'rb') as file:
        tree = fbx.nodes(file.read())
    out = []
    for node in next(n for n in tree if n[0] == 'Objects')[2]:
        if node[0] != 'Geometry':
            continue
        kids = {kid[0]: kid for kid in node[2]}
        corners = np.asarray(fbx._keys(node, 'Vertices'), float).reshape(-1, 3)
        ids = np.asarray(fbx._keys(node, 'PolygonVertexIndex'), np.int64)
        ends = np.flatnonzero(ids < 0)
        ids, first = np.where(ids < 0, -ids - 1, ids), np.concatenate([[0], ends[:-1] + 1])
        fans = [(ids[a], ids[b], ids[b + 1]) for a, end in zip(first, ends) for b in range(a + 1, end)]
        if len(corners) and fans and 'Vertices' in kids:
            out.append((corners, np.array(fans)))
    high = max(float(np.ptp(c[:, 1])) for c, _t in out)
    return [(c * (0.01 if high > 10.0 else 1.0), t) for c, t in out]


def _area(corners, triangles):
    a, b, c = (corners[triangles[:, k]] for k in range(3))
    return float(np.linalg.norm(np.cross(b - a, c - a), axis=1).sum()) / 2.0


def garments(cut):
    """[(corners, triangles, each triangle's piece's number in `cut`)]: the pieces `cut` sewn -
    those sharing SEAM corners or more one garment -, the highest first, what is under SHARE of
    the largest's area left."""
    keys = [set(map(tuple, np.round(c / SAME_M).astype(np.int64))) for c, _t in cut]
    of = list(range(len(cut)))

    def root(k):
        while of[k] != k:
            of[k] = of[of[k]]
            k = of[k]
        return k
    for a in range(len(cut)):
        for b in range(a):
            if root(a) != root(b) and len(keys[a] & keys[b]) >= SEAM:
                of[root(a)] = root(b)
    out = []
    for lead in sorted({root(k) for k in range(len(cut))}):
        corners, triangles, piece, base = [], [], [], 0
        for k, (c, t) in enumerate(cut):
            if root(k) == lead:
                corners.append(c)
                triangles.append(t + base)
                piece.append(np.full(len(t), k))
                base += len(c)
        out.append((np.vstack(corners), np.vstack(triangles), np.concatenate(piece)))
    most = max(_area(c, t) for c, t, _p in out)
    return sorted((g for g in out if _area(g[0], g[1]) >= SHARE * most), key=lambda g: -float(g[0][:, 1].max()))


def thinned(corners, triangles, piece, pitch):
    """(corners, triangles, piece) with the corners within a cell `pitch` a side made one, their
    mean: a triangle left with two corners the same is no triangle, one laid twice is laid once;
    `piece` each triangle's piece."""
    cells, of = np.unique(np.floor(corners / pitch).astype(np.int64), axis=0, return_inverse=True)
    of = of.ravel()
    mean = np.zeros((len(cells), 3))
    np.add.at(mean, of, corners)
    mean /= np.bincount(of, minlength=len(cells))[:, None]
    t = of[triangles]
    whole = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 2] != t[:, 0])
    _same, first = np.unique(np.sort(t[whole], axis=1), axis=0, return_index=True)
    return mean, t[whole][np.sort(first)], piece[whole][np.sort(first)]


def stood(corners, height):
    """`corners` of a body stood upright, m: its longest side up, y, its feet - two, apart -
    down on the floor; across its wider side, its left +x; its toes ahead, +z; `height` tall -
    a file's unit is its maker's, 0.019 a mannequin's height. The body is turned, none of it mirrored."""
    up, wide, deep = (int(k) for k in np.argsort(-np.ptp(corners, axis=0)))
    y = (corners[:, up] - corners[:, up].min()) / np.ptp(corners[:, up])

    def parted(low, high):                     # the widest gap across between two heights: a leg from a leg
        return float(np.diff(np.sort(corners[(y > low) & (y < high), wide])).max(initial=0.0))
    raised = np.eye(3)[up] * (-1.0 if parted(0.8, 0.9) > parted(0.1, 0.2) else 1.0)
    y = y if raised[up] > 0.0 else 1.0 - y
    toes = corners[y < 0.03, deep].mean() - corners[(y > 0.05) & (y < 0.1), deep].mean()
    ahead = np.eye(3)[deep] * (1.0 if toes > 0.0 else -1.0)
    out = corners @ np.stack([np.cross(raised, ahead), raised, ahead], 1)
    out -= (np.median(out[:, 0]), out[:, 1].min(), np.median(out[:, 2]))
    return out * height / np.ptp(out[:, 1])


def wrote(path, sewn):
    """The garments `sewn` [(corners, triangles, piece)] written an OBJ: a garment's pieces a
    group each, `sewn0.` and the piece's number the first's - a sleeve is told by its piece."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    lines, base = [], 1
    for k, (corners, triangles, piece) in enumerate(sewn):
        lines += ['v %.5f %.5f %.5f' % tuple(c) for c in corners]
        for p in np.unique(piece):
            lines.append('g sewn%d.%d' % (k, p))
            lines += ['f %d %d %d' % tuple(t) for t in triangles[piece == p] + base]
        base += len(corners)
    with open(path, 'w', encoding='ascii', newline='\n') as file:
        file.write('\n'.join(lines) + '\n')


def main():
    from coaxial.graphics import likeness
    told = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    told.add_argument('garments', help="a sewn garment's FBX")
    told.add_argument('--beside', default=likeness.FOUND, help='the figure it is worn by (default: the one found)')
    told.add_argument('--pitch', type=float, default=PITCH_M, help="a thinned corner's cell, m")
    told.add_argument('--body', action='store_true', help="a mannequin's body: stood, thinned, under formed/ beside it")
    told.add_argument('--height', type=float, default=HEIGHT_M, help='how tall a body stands, m')
    said = told.parse_args()
    if said.body:
        corners, triangles = max(pieces(said.garments), key=lambda piece: len(piece[1]))
        body = thinned(stood(corners, said.height), triangles, np.zeros(len(triangles), int), said.pitch)
        out = os.path.join(os.path.dirname(said.garments), 'formed',
                           os.path.splitext(os.path.basename(said.garments))[0] + '.obj')
        wrote(out, [body])
        print('%d triangles: a body %.2f m tall, %d triangles -> %s' % (len(triangles), said.height, len(body[1]), out))
        return 0
    if not said.beside:
        raise SystemExit('no figure found to wear it: say one with --beside')
    cut = pieces(said.garments)
    sewn = [thinned(c, t, p, said.pitch) for c, t, p in garments(cut)]
    out = os.path.join(os.path.dirname(said.beside), 'worn',
                       os.path.splitext(os.path.basename(said.garments))[0] + '.obj')
    wrote(out, sewn)
    print('%d pieces, %d triangles: %d garments, %s triangles -> %s' % (
        len(cut), sum(len(t) for _c, t in cut), len(sewn), ' + '.join(str(len(t)) for _c, t, _p in sewn), out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
