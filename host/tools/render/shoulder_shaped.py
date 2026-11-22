"""The abduction's cradle shaped by its loads: folded under and inboard, no arch over the shoulder.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_shaped.py [OUT.png]

The cradle `shoulder_rolled` draws sits under and inboard of the pitch's housing; here the optimiser (`topo`) is given
all the cradle may fill (`_room`): from the inboard spine beside the torso's side out under the housing's band and cone,
from the pin's boss ahead back to the abduction motor's end - less what turns by the cradle as the arm is raised (the
yoke's plates and the stop's rib swept through the travel, the drive's output ring, AIR_M off), what turns with the
pitch or stands on the torso (the pitch's housing, guard, lid, drive, leads, coil stem and torso block, the abduction's
housing, motor and board guard, the bush, the stops' horns, the bus's hose), and with the boss round the bush, the band
round the drive and the seat on the pitch's housing kept - held there. The cradle's loads at the margin
(`shoulder_rolled.carried`), a case each: the drive's own torque on the band; the yoke's two supports' pushes - the
arm's twist and swing as couples across them, half its pull each - on the boss and on the band. Printed from its flat
inboard spine out along -x (`up > 0`). `shaped` runs the case once (.cache/topo keeps it); `drawn` gives the solid's
smooth surface (`topo.faced`) in the sketch's frame - None until shaped, or where a solid does not carry or lost a
voxel told to stay: the cradle then stays as drawn.
"""
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import shoulder_rolled as sr  # noqa: E402
from tools.render import topo  # noqa: E402

#: The voxel, mm; the share of what is the optimiser's to shape that the cradle may keep; the way it grows off its
#: bed (+1 from its inboard spine out along -x, -1 along -z); how far past the bush's and band's walls seats stay, mm;
#: the step the yoke is swept through its travel at, deg, and the grid it is drawn on, m.
H_MM, SHARE, UP, ROUND_MM, STEP, FINE_M = 3.0, 0.60, 1.0, 3.0, 1.0, 0.0005


def _axes(up):
    """(3, 3): the bed's rows across, up and deep in the sketch's frame - for `up > 0`, grown along -x off the inboard
    spine `(y, -x, z)`; for `up < 0`, grown along -z `(y, -z, -x)`."""
    import numpy as np
    return np.array([(0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)] if up > 0
                    else [(0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0)])


def _bed(v, up):
    """The sketch's vector `v` in the bed's frame (`_axes`), its parts moved and signed - no sum: a zero keeps its
    sign, and a case's content its name (`topo._digest`)."""
    import numpy as np
    v = np.asarray(v, float)
    return np.array([v[1], -v[0], v[2]]) if up > 0 else np.array([v[1], -v[2], -v[0]])


def _swept(outline, x, y):
    """The least distance, m, from each point (`x`, `y` arrays) to the outline (2-d points round, the sketch's
    frame, a convex piece of the yoke) turned through the abduction's travel."""
    import numpy as np
    from scipy import ndimage
    from scipy.spatial import Delaunay
    outline = np.asarray(outline, float)
    lo, hi = outline.min(axis=0), outline.max(axis=0)
    g = np.stack(np.meshgrid(np.arange(lo[0], hi[0], FINE_M), np.arange(lo[1], hi[1], FINE_M)), -1).reshape(-1, 2)
    inside = g[Delaunay(outline).find_simplex(g) >= 0]
    x0, y0 = float(x.min()) - 0.01, float(y.min()) - 0.01
    shape = (int((float(x.max()) - x0 + 0.02) / FINE_M) + 1, int((float(y.max()) - y0 + 0.02) / FINE_M) + 1)
    hit = np.zeros(shape, bool)
    for roll in np.append(np.arange(sr.TRAVEL[0], sr.TRAVEL[1], STEP), sr.TRAVEL[1]):
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        p = inside @ np.array([[c, s], [-s, c]]).T
        i, j = np.floor((p[:, 0] - x0) / FINE_M).astype(int), np.floor((p[:, 1] - y0) / FINE_M).astype(int)
        ok = (i >= 0) & (i < shape[0]) & (j >= 0) & (j < shape[1])
        hit[i[ok], j[ok]] = True
    far = np.asarray(ndimage.distance_transform_edt(~hit), float) * FINE_M
    return far[np.floor((x - x0) / FINE_M).astype(int), np.floor((y - y0) / FINE_M).astype(int)]


@functools.lru_cache(maxsize=None)
def _room(up=UP):
    """(room, kept, held [across, up, deep], the boss's and the band's voxels, the bed's corner mm): the cradle's
    voxels folded under and inboard of the pitch's housing, with no arch over the shoulder's top."""
    import numpy as np
    from scipy import ndimage
    from scipy.spatial import KDTree
    from tools.render import shoulder_hosed as sh, shoulder_mount as sm, tees
    lay, st, h, air = sr.laid(), sr.stack(), 1e-3 * H_MM, sr.AIR_M
    xs = np.arange(0.066, -lay['band'] - 0.5 * h, -h) - 0.5 * h
    ys = np.arange(-0.062, lay['band'] + 0.5 * h, h) + 0.5 * h
    zs = np.arange(lay['boss'][0], lay['motor'][1] - 0.5 * h, h) + 0.5 * h
    if up > 0:
        y, x, z = np.meshgrid(ys, xs, zs, indexing='ij')
    else:
        zs = zs[::-1]
        y, z, x = np.meshgrid(ys, zs, xs, indexing='ij')
    r_x, r_z, edge = np.hypot(y, z), np.hypot(x, y), air + 0.9 * h
    pts = km._points([t for n in ('housing', 'guard', 'lid', 'drive', 'leads', 'coil stem')
                      for t in sm.turning()[n]], 0.0015)
    still = km._points([t for k, v in sm.still().items() if k != 'output ring' for t in v], 0.002)
    rm = np.array([np.hypot(*pts[np.abs(pts[:, 0] - a) <= 0.5 * h + air][:, 1:].T).max(initial=0.0) for a in xs])
    st_r = np.array([np.hypot(*still[np.abs(still[:, 0] - a) <= 0.5 * h + air][:, 1:].T).max(initial=0.0) for a in xs])
    reach = np.maximum(rm, st_r)[(None, slice(None), None) if up > 0 else (None, None, slice(None))]
    foot = (x >= -0.006) & (x <= 0.056) & (np.abs(z) <= 0.052) & (y <= 0.030) & ((x >= 0.012) | (y <= -0.010))
    cap = np.clip(0.015 + 0.24 * np.maximum(x, 0.0), 0.015, 0.030)
    y_top = np.where(z >= lay['house'][0] - 0.010,
                     np.maximum(0.030, np.sqrt(np.maximum((lay['band'] + 0.004) ** 2 - x ** 2, 0.0))), cap)
    env = ((y > y_top) | (y < -0.060)
           | ((z < lay['house'][0] - 0.010) & (x < -0.016) & (r_z > lay['pin'] + sr.BUSH_M + sr.BOSS_WALL + 0.005)))
    out = env | (r_x < reach + np.where(foot, 0.0, edge))
    bot_c = -lay['drop'] - 0.5 * lay['saddle'] + 0.5 * sr.WIDE_M
    hull = np.r_[[(sr.HUB_M * math.cos(a), sr.HUB_M * math.sin(a)) for a in np.linspace(0.0, 2.0 * math.pi, 49)[:-1]],
                 [(0.5 * sr.WIDE_M * math.cos(a), bot_c + 0.5 * sr.WIDE_M * math.sin(a))
                  for a in np.linspace(math.pi, 2.0 * math.pi, 25)]]
    plates, (p0, p1) = _swept(hull, x, y) < edge, lay['plate']
    out |= plates & ((np.abs(np.abs(z) - 0.5 * (p0 + p1)) <= 0.5 * (p1 - p0) + edge))
    rib = [(-0.5 * sr.HORN_M, -lay['rib'] - sr.HORN_M), (0.5 * sr.HORN_M, -lay['rib'] - sr.HORN_M),
           (0.5 * sr.HORN_M, -lay['rib']), (-0.5 * sr.HORN_M, -lay['rib'])]
    out |= (_swept(rib, x, y) < edge) & (z >= p1 - edge) & (z <= p1 + sr.LAP_M + edge)
    out |= (np.hypot(x, z) < st['board'][0] + sr.GUARD_M + edge) & (y <= -lay['board'][0] + edge)
    out |= (np.hypot(x + 0.0262, z) < 0.010) & (y < -0.045)
    (r0, r1), (h0, h1), (_m0, m1) = lay['ring'], lay['house'], lay['motor']
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
    room = topo.stood((~out).transpose(1, 0, 2)).transpose(1, 0, 2)
    boss = room & (r_z <= hole + sr.BOSS_WALL + 0.004) & (z <= b1)
    band = room & (r_z <= lay['band'] + 1e-3 * ROUND_MM) & (z >= h0) & (z <= m1)
    held = room & foot & (r_x < reach + 1.5 * h)
    seat = room & np.asarray(ndimage.binary_dilation(held), bool)
    corner = 1e3 * _bed((xs[0], ys[0], zs[0]), up) - 0.5 * H_MM
    return room, seat | boss | band, held, boss, band, corner


@functools.lru_cache(maxsize=None)
def case(share=SHARE, up=UP):
    """The cradle's case for `topo` - its room, what must stay, what holds it, its loads, in its bed's frame, mm -
    keeping `share` of what is the optimiser's, grown `up`."""
    room, kept, held, boss, band, corner = _room(up)
    lay = sr.laid()
    ld, own, _push, _twist = sr.carried()
    span = 2e3 * (sr.gap() + 0.5 * sr.SHEET_M)
    tw, sw, half = ld['torsion'] / span, ld['swing'] / span, 0.5 * ld['axial']

    def load(patch, force, moment, about):
        return patch, _bed(force, up), _bed(moment, up), 1e3 * _bed(about, up) - corner
    front, back, none = (0.0, 0.0, 0.5 * sum(lay['boss'])), (0.0, 0.0, lay['house'][0]), (0.0, 0.0, 0.0)
    loads = [[load(band, none, (0.0, 0.0, own), back)]]
    for ax, ay, by in ((tw, 0.0, 0.0), (0.0, sw - half, -sw - half), (0.0, -sw - half, sw - half),
                       (tw, sw - half, -sw - half), (tw, -sw - half, sw - half)):
        loads.append([load(boss, (ax, ay, 0.0), none, front), load(band, (-ax, by, 0.0), none, back)])
    return topo.case('shoulder hood', room, kept, held, loads, H_MM, share)


def shaped(log=None, share=SHARE, up=UP):
    """The cradle shaped (`topo.shaped`): run once, then from .cache/topo."""
    return topo.shaped(case(share, up), log)


@functools.lru_cache(maxsize=None)
def drawn(share=SHARE, up=UP):
    """[(tri, colour)] of the shaped cradle's faces, the sketch's frame, m - None where not shaped yet, or where a
    solid does not carry its loads or lost a voxel told to stay."""
    if not topo.ready(case(share, up)):
        return None
    got, corner = topo.shaped(case(share, up)), _room(up)[5]
    if got['margin'] < 1.0 or got['lost']:
        return None
    return [(1e-3 * (tri + corner) @ _axes(up), em.FRAME) for tri in topo.faced(got, case(share, up))]


def told(log=None, share=SHARE, up=UP):
    """{'room', 'grams': g before and after, 'margin', 'level', 'lost': kept voxels that hung, 'hung': voxels of the
    solid with none of the nine under them} of the cradle shaped."""
    got = shaped(log, share, up)
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
        ImageDraw.Draw(sheet).text((12, 10), "the abduction's cradle shaped: %(grams).0f g, margin x%(margin).2f" % got,
                                   fill=(235, 235, 235))
        sheet.save(out[0])
        print(out[0])
    return 0


if __name__ == '__main__':
    sys.exit(main())
