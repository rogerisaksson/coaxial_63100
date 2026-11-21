"""The abduction's hood shaped by its loads, printable: material only where they ask it.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_shaped.py [OUT.png]

The hood `shoulder_rolled` draws is a prism; here the optimiser (`topo`) is given all the hood may fill (`_room`):
from the pitch's guard's outer face to the foot of the housing's cone, from the band round the abduction's drive up
to the roof, from the pin's boss back to the motor's end - less what turns by the hood as the arm is raised (the
yoke's plates and the stop's rib swept through the travel, the drive's output ring, AIR_M off), what turns with
it but is another part (the pitch's housing, guard, lid and drive, the drive's housing and motor, the bush, the
stops' horns, the bus's hose), and with the boss round the bush, the band round the drive and the foot that sits on
the pitch's housing kept - held there. The hood's loads at the margin (`shoulder_rolled.carried`), a case each: the
drive's own torque on the band; the yoke's two supports' pushes - the arm's twist and swing as couples across them,
half its pull each - on the boss and on the band. Printed up the abduction's axis from the boss's front face: the
bush's bore, the boss and the band rings round it, each stands on itself. `shaped` runs the case once (.cache/topo
keeps it); `drawn` gives the solid's faces in the sketch's frame - None until it is shaped, or where it does not
carry its loads or lost a voxel told to stay: the hood then stays as drawn.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import knee_topo as kt  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import topo  # noqa: E402

#: The voxel, mm; the share of what is the optimiser's to shape that the hood may keep; how far past the bush's
#: wall the boss is kept, mm; the step the yoke is swept through its travel at, deg, and the grid it is drawn on, m.
H_MM, SHARE, ROUND_MM, STEP, FINE_M = 3.0, 0.25, 3.0, 1.0, 0.0005


def _swept(outline, xs, ys):
    """The least distance, m, from each (x, y) of the grids `xs`, `ys` to the outline (2-d points round, the
    sketch's frame, a convex piece of the yoke) turned through the abduction's travel."""
    import numpy as np
    from scipy import ndimage
    from scipy.spatial import Delaunay
    outline = np.asarray(outline, float)
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    g = np.stack(np.meshgrid(np.arange(lo[0], hi[0], FINE_M), np.arange(lo[1], hi[1], FINE_M)), -1).reshape(-1, 2)
    inside = g[Delaunay(outline).find_simplex(g) >= 0]
    x0, y0 = xs[0] - 0.01, ys[0] - 0.01
    shape = (int((xs[-1] - x0 + 0.02) / FINE_M) + 1, int((ys[-1] - y0 + 0.02) / FINE_M) + 1)
    hit = np.zeros(shape, bool)
    for roll in np.append(np.arange(sr.TRAVEL[0], sr.TRAVEL[1], STEP), sr.TRAVEL[1]):
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        p = inside @ np.array([[c, s], [-s, c]]).T
        i, j = np.floor((p[:, 0] - x0) / FINE_M).astype(int), np.floor((p[:, 1] - y0) / FINE_M).astype(int)
        ok = (i >= 0) & (i < shape[0]) & (j >= 0) & (j < shape[1])
        hit[i[ok], j[ok]] = True
    far = np.asarray(ndimage.distance_transform_edt(~hit), float) * FINE_M
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    return far[np.floor((gx - x0) / FINE_M).astype(int), np.floor((gy - y0) / FINE_M).astype(int)]


@functools.lru_cache(maxsize=None)
def _room():
    """(room, kept, held [across, up, deep], the boss's and the band's voxels, the bed's corner mm): the hood's
    voxels, its bed's rows across along y, up the abduction's axis z and deep along x."""
    import numpy as np
    from scipy.spatial import KDTree
    from tools.render import shoulder_hosed as sh, shoulder_mount as sm, tees
    lay, pitch, st, h, air = sr.laid(), sl.laid(), sr.stack(), 1e-3 * H_MM, sr.AIR_M
    xs = np.arange(pitch['face'], pitch['cant'] - 0.5 * h, h) + 0.5 * h
    ys = np.arange(-lay['band'] - h, lay['roof'] - 0.5 * h, h) + 0.5 * h
    zs = np.arange(lay['boss'][0], lay['motor'][1] - 0.5 * h, h) + 0.5 * h
    y, z, x = np.meshgrid(ys, zs, xs, indexing='ij')
    r_x, r_z, edge = np.hypot(y, z), np.hypot(x, y), air + 0.5 * h
    pts = km._points([t for n in ('housing', 'guard', 'lid', 'drive', 'leads') for t in sm.turning()[n]], 0.0015)
    reach = np.array([np.hypot(*pts[np.abs(pts[:, 0] - a) <= 0.5 * h + air][:, 1:].T).max(initial=0.0)
                      for a in xs])[None, None, :]
    foot = (x >= sr._hood_x()[0]) & (np.abs(z) <= sr.FOOT_M)                       # on the pitch's housing
    out = r_x < reach + np.where(foot, 0.0, edge)
    hull = np.r_[[(sr.HUB_M * math.cos(a), sr.HUB_M * math.sin(a)) for a in np.linspace(0.0, 2.0 * math.pi, 49)[:-1]],
                 [(-0.5 * sr.WIDE_M, -lay['drop'] - 0.5 * lay['saddle']), (0.5 * sr.WIDE_M, -lay['drop'] - 0.5 * lay['saddle'])]]
    plates, (p0, p1) = _swept(hull, xs, ys).T[:, None, :] < edge, lay['plate']
    out |= plates & ((np.abs(np.abs(z) - 0.5 * (p0 + p1)) <= 0.5 * (p1 - p0) + edge))
    rib = [(-0.5 * sr.HORN_M, -lay['rib'] - sr.HORN_M), (0.5 * sr.HORN_M, -lay['rib'] - sr.HORN_M),
           (0.5 * sr.HORN_M, -lay['rib']), (-0.5 * sr.HORN_M, -lay['rib'])]
    out |= (_swept(rib, xs, ys).T[:, None, :] < edge) & (z >= p1 - edge) & (z <= p1 + sr.LAP_M + edge)
    (r0, r1), (h0, h1), (m0, m1) = lay['ring'], lay['house'], lay['motor']
    bore = np.where(z < h1, st['gear'][0], st['motor'][0])
    out |= (r_z < st['gear'][0] + edge) & (z >= r0 - edge) & (z < r1) | (r_z < bore) & (z >= r1)
    for deg, way in ((sr.TRAVEL[0], -1.0), (sr.TRAVEL[1], 1.0)):        # the stops' horns, as `shoulder_rolled`
        r = lay['rib'] + 0.5 * sr.HORN_M
        b = math.radians(deg) + way * sr.HORN_M / r
        out |= ((np.hypot(x + r * math.sin(b), y + r * math.cos(b)) < 0.5 * sr.HORN_M + edge) & (z >= p1)
                & (z < h0 + 0.002 + edge))
    (_b0, b1), hole = lay['boss'], lay['pin'] + sr.BUSH_M
    out |= (r_z < hole) & (z <= -p0)                                       # the pin in its bush, on to the plate
    line = sh.laid()['up']
    out |= KDTree(line).query(np.c_[x.ravel(), y.ravel(), z.ravel()])[0].reshape(x.shape) < tees.HOSE_R + edge
    room = ~out
    boss = room & (r_z <= hole + sr.BOSS_WALL + 1e-3 * ROUND_MM) & (z <= b1)
    band = room & (r_z <= lay['band']) & (z >= h0) & (z <= m1)
    held = room & foot & (r_x < reach + 1.5 * h)
    corner = 1e3 * np.array([ys[0], zs[0], xs[0]]) - 0.5 * H_MM
    return room, held | boss | band, held, boss, band, corner


@functools.lru_cache(maxsize=None)
def case():
    """The hood's case for `topo`: its room, what must stay, what holds it, its loads - in its bed's frame, mm."""
    import numpy as np
    room, kept, held, boss, band, corner = _room()
    lay = sr.laid()
    ld, own, _push, _twist = sr.carried()
    span = 2e3 * (sr.gap() + 0.5 * sr.SHEET_M)
    tw, sw, half = ld['torsion'] / span, ld['swing'] / span, 0.5 * ld['axial']

    def load(patch, force, moment, about):                # the sketch's (x, y, z) as the bed's (y, z, x)
        return (patch, np.roll(np.asarray(force, float), -1), np.roll(np.asarray(moment, float), -1),
                1e3 * np.roll(np.asarray(about, float), -1) - corner)
    front, back, none = (0.0, 0.0, 0.5 * sum(lay['boss'])), (0.0, 0.0, lay['house'][0]), (0.0, 0.0, 0.0)
    loads = [[load(band, none, (0.0, 0.0, own), back)]]
    for ax, ay, by in ((tw, 0.0, 0.0), (0.0, sw - half, -sw - half), (0.0, -sw - half, sw - half),
                       (tw, sw - half, -sw - half), (tw, -sw - half, sw - half)):
        loads.append([load(boss, (ax, ay, 0.0), none, front), load(band, (-ax, by, 0.0), none, back)])
    return topo.case('shoulder hood', room, kept, held, loads, H_MM, SHARE)


def shaped(log=None):
    """The hood shaped (`topo.shaped`): run once, then from .cache/topo."""
    return topo.shaped(case(), log)


def drawn():
    """[(tri, colour)] of the shaped hood's voxels' faces, the sketch's frame, m - None where it is not shaped yet,
    or its solid does not carry its loads or lost a voxel told to stay."""
    import numpy as np
    if not topo.ready(case()):
        return None
    got, corner = topo.shaped(case()), _room()[-1]
    if got['margin'] < 1.0 or got['lost']:
        return None
    return [(1e-3 * np.roll(tri + corner, 1, axis=1), em.FRAME) for tri in kt.faces(got['points'], got['cells'])]


def told(log=None):
    """{'room', 'grams': g before and after, 'margin', 'level', 'lost': kept voxels that hung, 'hung': voxels of the
    solid with none of the nine under them} of the hood shaped."""
    got = shaped(log)
    layers = got['solid'].transpose(1, 0, 2)
    return {'room': round(float(got['room']), 1), 'grams': round(float(got['grams']), 1),
            'margin': round(float(got['margin']), 2), 'level': float(got['level']), 'lost': int(got['lost']),
            'hung': int((layers & ~topo.stood(layers)).sum())}


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import mounting as m, shoulder_mount as sm
    out = (sys.argv[1:] if argv is None else argv)[:1]
    got = told(log=print)
    print('hood %(grams).1f g of %(room).1f g at %(level).1f, margin x%(margin).2f, %(hung)d hung, %(lost)d lost' % got)
    if out:
        views = [(0.0, 35.0, 18.0), (0.0, -150.0, 14.0), (sr.TRAVEL[1], 35.0, 18.0), (sr.TRAVEL[0], -60.0, 10.0)]
        sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
        for i, (roll, yaw, tilt) in enumerate(views):
            tris = [t for part in sm.posed(0.0, roll).values() for t in part]
            sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.14, centre=(0.0, -0.04, 0.0)), (760 * i, 40))
        ImageDraw.Draw(sheet).text((12, 10), "the abduction's hood shaped: %(grams).0f g, margin x%(margin).2f" % got,
                                   fill=(235, 235, 235))
        sheet.save(out[0])
        print(out[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
