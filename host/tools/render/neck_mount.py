"""The neck's mount, sketched: the torso's block, the neck's drive and yoke, the head's drive, the bay.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/neck_mount.py OUT.png

`neck_laid` sizes it; here its parts are drawn, the torso's frame. What stays with the torso: the block - its band
round the crossbar, its band round the neck's motor over that, its saddle down to the seat the spine's tube's foot is
screwed under -, the neck's motor and its gearbox's housing, the bay on the spine's tube - the two drives' boards, a
twin under one guard and lid (`hip_clamped._drive`) -, and the two shoulders' mounts as they stand on the crossbar
(`shoulder_mount`, each drum turned its rib up: the worst the yoke passes). What nods with the neck: its output ring,
the yoke - a carbon plate on the ring's hub, inside the shoulders' drums' mouths, and a printed crown screwed on it:
a floor on the head's axis, a slit band on that floor round the head's motor, under the floor the shoe that bears
the head's thrust on the neck's band - and the head's motor and its gearbox's housing in that band. What turns with
the head: its output ring and on the ring's hub the carbon carrier the skull is screwed to. The leads and their
hose are `neck_hosed`'s.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import neck_laid as nl  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import sized  # noqa: E402

#: The head's parts' frame in the torso's: x up the head's axis, y ahead, z to the robot's left; the bay's: x
#: ahead - its boards behind its puck -, y up, z to the right. Rows: where each of a frame's axes points.
UP = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
BAY = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
#: Where the slits of the block's two bands sit, rad from up toward ahead: behind, under the shoe's sweep; the
#: bay's clamp's half height and wall, m; what stays, what nods and what turns of the parts the checks take.
SLITS, CLAMP_M = (-0.5 * math.pi, -1.9), (0.012, 0.006)
FIXED = ('block', 'spine', 'bay', 'bay clamp', 'bay guard', 'bay lid', 'left shoulder', 'right shoulder')
NODS, TURNS = ('neck plate', 'crown', 'head drive'), ('head ring', 'carrier')
#: The skin's cells about the head's axis: a row's height, m, and the bearings round.
CELL_M, BEARINGS = 0.0025, 96


def _put(tris, at, turn=None):
    """`tris` moved to `at`, first turned by the rows `turn` (a frame's axes in the torso's)."""
    import numpy as np
    spin = np.eye(3) if turn is None else np.array(turn, float)
    return [(tri @ spin + np.asarray(at, float), colour) for tri, colour in tris]


@functools.lru_cache(maxsize=None)
def still(socket='torx', low=None):
    """{name: tris} of what stays with the torso; the spine's tube from `low` up, m over the torso's anchor, where
    told - a trunk that seats it higher (`trunk_clamped.seat`) -, else from its low end as the skeleton has it."""
    lay = nl.laid()
    (yc, zc, bar), (ny, nz) = lay['bar'], lay['neck']
    (m0, m1, can), (h0, h1, ring), (b0, b1, out) = lay['motor'], lay['house'], lay['band']
    seat_y, spine_z, tube = lay['seat']
    bored = bar + 1e-3 * kl.BOND                         # the crossbar's band: lower past the neck's housing
    block = _put(cl.turned([(-lay['side'], bored, lay['bore']), (h1, bored, lay['bore']), (h1, bored, lay['under']),
                            (lay['side'], bored, lay['under'])], SLITS[0]), (0.0, yc, zc))
    block += _put(cl.turned([(b0, can, out), (m1, can, out), (m1, ring, out), (b1, ring, out)], SLITS[1]), (0.0, ny, nz))
    half_x, half_z = tube + em.OVAL_PAST[0], tube + em.OVAL_PAST[1]
    waist = 1e-3 * sized.neck(2e3 * half_x, 'spine')
    block += em._rounded(cl.filleted(cl.necked((yc, zc), lay['bore'] - 0.001, seat_y, (spine_z - half_z, spine_z + half_z),
                                               waist)), -half_x, half_x, em.FRAME, axis=0, r=0.002)
    foot = seat_y - nl.SEAT_M
    e = tube - 0.0012 - em.BORE_R - 0.003
    dress, limb = em._limb(foot, -1.0, tube, -e, socket, False)
    low = float(_spine()[1]) if low is None else low
    spine = (_put(em._rounded(em._oval(tube), seat_y, foot, em.PLATE, r=0.002) + dress + limb, (0.0, 0.0, spine_z))
             + m._prism(m._circle(tube, 40, (0.0, spine_z)), low, foot - em.FLANGE_M, em.CARBON))
    neck = _put(em._drum(can, m0, m1, em.SERVO) + em._drum(ring, h0, h1, em.GEAR), (0.0, ny, nz))
    return {'block': block, 'spine': spine, 'neck drive': neck, **_bay(), **_shoulders()}


def _spine():
    """(The spine's tube's low end's (x, y, z), its y): the skeleton's."""
    from machine.body import skeleton
    low = skeleton.TRUNK['torso'][0][0][0]
    return low, float(low[1])


def bayed():
    """((x, y, z) where the bay's frame stands, the neck's board's (from, to) along its x, the head's): the twin
    as the build lays it (`drives.BOARDS`), a puck from the nearer board to the clamp round the spine's tube."""
    lay, at = nl.laid(), nl.placed()
    _seat, spine_z, tube = lay['seat']
    by, far, near = at['bay']
    z0, half = spine_z - tube - CLAMP_M[1] - 0.001, 0.5 * nl.stacks()['board'][1]
    return (0.0, by, z0), (far - z0 - half, far - z0 + half), (near - z0 - half, near - z0 + half)


def _bay():
    """{'bay', 'bay clamp', 'bay guard', 'bay lid', 'bay leads'}: the twin - the neck's board behind the head's, a
    guard round both and a lid behind them - on a printed puck, the puck's nose on a band round the spine's tube,
    under the flange of that tube's foot."""
    from tools.render import boards, covered as cv, hip_clamped as hc
    lay, st = nl.laid(), nl.stacks()
    _seat, spine_z, tube = lay['seat']
    at, rear, front = bayed()
    r_b, can, ring = st['board'][0], lay['head']['can'], lay['head']['gear']
    wall = 1e-3 * kl.WALL + 0.0045
    reach = em.OVAL_PAST[1] - CLAMP_M[1] - 0.001 + nl.AIR_M              # the puck's face clear of the spine's foot's
    nose = min(ring, lay['seat'][0] - nl.SEAT_M - em.FLANGE_M - nl.AIR_M - at[1])     # flange, its nose under it
    drive = hc._drive({'board': rear + (r_b,), 'motor': (front[1], -reach, can), 'gear': (-reach, 0.0, nose)}, wall,
                      math.pi, leave=math.pi)
    puck = [(tri, em.FRAME if colour in (em.SERVO, em.GEAR, em.OUTPUT) else colour)
            for tri, colour in drive['still'] + drive['ring'] + drive['band']]
    turn = cv.turned((-cv.edge(r_b, can + wall), 0.0))   # the head's board's lugs across from the neck's
    twin = (boards.placed(front[0], front[1], (0.0, 0.0), front[1], turn)
            or em._drum(r_b, front[0], front[1], em.BOARD))
    clamp = cl.turned([(-CLAMP_M[0], tube + 1e-3 * kl.BOND, tube + CLAMP_M[1]),
                       (CLAMP_M[0], tube + 1e-3 * kl.BOND, tube + CLAMP_M[1])], 0.5 * math.pi)
    band = _put(clamp, (0.0, at[1], spine_z), ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)))
    return {'bay': _put(puck + twin, at, BAY), 'bay clamp': band, 'bay guard': _put(drive['guard'], at, BAY),
            'bay lid': _put(drive['lid'], at, BAY),
            'bay leads': _put(drive['leads'] + boards.leads(front[0], front[1], (0.0, 0.0), front[1], turn, can + wall),
                              at, BAY)}


def _shoulders():
    """{'left shoulder', 'right shoulder'}: each shoulder's mount where it stands on the crossbar - the crossbar,
    the block's side with its horn, and its housing turned its rib up -, the right one the left's mirrored."""
    import numpy as np
    from tools.render import shoulder_mount as sm
    at = nl.placed()
    c, s = math.cos(at['lean']), math.sin(at['lean'])
    turn = np.array([[-c, -s, 0.0], [-s, c, 0.0], [0.0, 0.0, -1.0]])    # the shoulder's sketch's axes in the torso's
    drum = km._turn(sm.turning()['housing'], -sl.drummed())
    left = [(tri @ turn + np.array(at['arm']), colour)
            for tri, colour in sm.still()['crossbar'] + sm.still()['block'] + drum]
    return {'left shoulder': left, 'right shoulder': [((tri * (-1.0, 1.0, 1.0))[::-1], colour) for tri, colour in left]}


def _plate(ring, top):
    """The yoke's carbon plate's outline, (y, z) about the neck's axis: the hub's disc `ring` round, and up from it
    to `top` WEB_M wide - over the tee's collar its rear edge in at NAPE_M, inside the neck's skin."""
    import numpy as np
    half = 0.5 * nl.WEB_M
    meet = math.atan2(half, math.sqrt(ring * ring - half * half))         # where the disc leaves the web's edge
    arc = [(ring * math.cos(t), ring * math.sin(t)) for t in np.linspace(meet, 2.0 * math.pi - meet, 49)]
    return np.array(arc + [(nl.COLLAR_M, -half), (nl.COLLAR_M + half - nl.NAPE_M, -nl.NAPE_M), (top, -nl.NAPE_M),
                           (top, half)])


def _hub(x, way, at=(0.0, 0.0)):
    """A hub's screws' heads on the face at `x`, standing `way` (+1, -1) off it, about `at`."""
    count, circle = kl.HUB
    tris = []
    for k in range(count):
        a = 2.0 * math.pi * (k + 0.5) / count
        spot = (at[0] + 1e-3 * circle * math.cos(a), at[1] + 1e-3 * circle * math.sin(a))
        tris += em._drum(em.HEAD[0], min(x, x + way * em.HEAD[1]), max(x, x + way * em.HEAD[1]), em.BOLT, spot)
    return tris


@functools.lru_cache(maxsize=None)
def nodding():
    """{name: tris} of what nods with the neck, upright: the neck's output ring, the yoke - its carbon plate on the
    ring's hub and the printed crown on that: a cheek screwed to the plate, a floor, on the floor a band round the
    head's motor, slit behind and free of the floor either side of its slit -, the crown's shoe on the neck's
    band, the head's motor and its gearbox's housing."""
    from tools.render import shoulder_mount as sm
    lay = nl.laid()
    (ny, nz), (r0, r1, ring), (web0, web1), (c0, c1) = lay['neck'], lay['ring'], lay['web'], lay['cheek']
    (p0, p1), out, yaw, head = lay['plate'], lay['band'][2], lay['yaw'], lay['head']
    plate = em._rounded(_plate(ring, p0 - ny), web0, web1, em.CARBON, axis=0, r=0.001) + _hub(web1, 1.0)
    (low, high), (rear, front) = lay['patch']
    cheek = em._rounded([(low, rear), (low, front), (high, front), (high, rear)], c0, c1, em.FRAME, axis=0, r=0.002)
    for at in ((y, z) for y in (low + nl.EDGE_M, high - nl.EDGE_M) for z in (rear + nl.EDGE_M, front - nl.EDGE_M)):
        plate += em._drum(em.HEAD[0], web1, web1 + 0.001, em.BOLT, at)
    (b0, b1, wall), can, gear = head['band'], head['can'], head['gear']
    crown = em._drum(lay['round'], p0, p1, em.FRAME)
    crown += cl.turned([(b0, can, wall), (head['motor'][1], can, wall), (head['motor'][1], gear, wall),
                        (b1, gear, wall)], math.pi)
    crown += sm._arc(can, wall, p1, b0 + 0.001, -0.6 * math.pi, 0.6 * math.pi, em.FRAME, k=24)
    shoe = em._rounded(cl.cupped((0.0, 0.0), out + sl.PLAY_M, p0 - ny, (-0.5 * nl.SHOE_M, 0.5 * nl.SHOE_M), below=False),
                       lay['band'][0], lay['band'][1], em.FRAME, axis=0, r=0.0015)
    (m0, m1), (h0, h1) = head['motor'], head['house']
    drive = em._drum(can, m0, m1, em.SERVO) + em._drum(gear, h0, h1, em.GEAR)
    return {'neck ring': _put(em._drum(ring, r0, r1, em.OUTPUT), (0.0, ny, nz)),
            'neck plate': _put(plate, (0.0, ny, nz)), 'crown': _put(cheek, (0.0, ny, nz)) + _put(crown, (0.0, 0.0, yaw), UP),
            'shoe': _put(shoe, (0.0, ny, nz)), 'head drive': _put(drive, (0.0, 0.0, yaw), UP)}


@functools.lru_cache(maxsize=None)
def turning():
    """{name: tris} of what turns with the head: its output ring, and on the ring's hub the carbon carrier the
    skull is screwed to."""
    lay = nl.laid()
    head, at = lay['head'], (0.0, 0.0, lay['yaw'])
    (r0, r1), (c0, c1), ring = head['ring'], head['carrier'], head['gear']
    carrier = em._drum(ring, c0, c1, em.CARBON, soft=0.001) + _hub(c1, 1.0)
    return {'head ring': _put(em._drum(ring, r0, r1, em.OUTPUT), at, UP), 'carrier': _put(carrier, at, UP)}


def _spun(tris, deg, about, way):
    """`tris` turned `deg` about the axis through `about` along `way` ('x' the neck's, 'y' the head's)."""
    import numpy as np
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    spin = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]] if way == 'x' else
                    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    at = np.asarray(about, float)
    return [((tri - at) @ spin.T + at, colour) for tri, colour in tris]


def posed(nod=0.0, turn=0.0, low=None):
    """{name: tris}: the neck nodded `nod` deg and the head turned `turn` deg - the head's parts about its axis,
    then all that nods about the neck's; the spine's tube from `low` where told (`still`)."""
    lay = nl.laid()
    (ny, nz), yaw = lay['neck'], lay['yaw']
    out = dict(still(low=low))
    for name, tris in nodding().items():
        out[name] = _spun(tris, nod, (0.0, ny, nz), 'x')
    for name, tris in turning().items():
        out[name] = _spun(_spun(tris, turn, (0.0, 0.0, yaw), 'y'), nod, (0.0, ny, nz), 'x')
    return out


@functools.lru_cache(maxsize=None)
def aired(nods=8):
    """The least air between what nods - the yoke, the head's drive on it and what turns on that - and what stays,
    through the neck's travel, m: the shoe on its band and the neck's output ring by its housing apart - they run
    there."""
    import numpy as np
    from machine.body import drives
    (ny, nz), h = nl.laid()['neck'], 0.006
    fixed = km._points([t for name in FIXED for t in still()[name]])
    moved = km._points([t for name in NODS for t in nodding()[name]]
                       + [t for name in TURNS for t in turning()[name]]) - (0.0, ny, nz)
    keys = km._key(np.floor(fixed / h).astype(np.int64))
    order = np.argsort(keys)
    return min(float(km._least(keys[order], fixed[order], moved @ km._rx(float(deg)).T + (0.0, ny, nz), h))
               for deg in np.linspace(*drives.TRAVEL['neck'], nods))


def checked():
    """{'held': the least of `neck_laid.held`'s margins, 'air': what nods off what stays through the neck's travel,
    mm, 'crossing': triangle pairs of what nods or turns through what stays, upright, and of the bay through the
    block, the spine and the shoulders beside it - its clamp on the spine's tube apart -, 'skin': how far inside the
    likeness's head's outer skin (what nods and does not turn keeps - the skin turned past it through the head's
    travel -, what turns with the head keeps), mm - None with no likeness beside the tree}: what the suite holds."""
    from coaxial.graphics import likeness
    from tools.render import clips
    fixed = [t for name in FIXED for t in still()[name]]
    nods = [t for name in NODS for t in nodding()[name]]
    turns = [t for name in TURNS for t in turning()[name]]
    bay = [t for name in ('bay', 'bay guard', 'bay lid', 'bay leads') for t in still()[name]]
    beside = [t for name in ('block', 'spine', 'neck drive', 'left shoulder', 'right shoulder') for t in still()[name]]
    return {'held': round(min(nl.held().values()), 3), 'air': round(1e3 * aired(), 2),
            'crossing': clips.crossed(nods + turns, fixed) + clips.crossed(bay, beside),
            'skin': tuple(round(1e3 * d, 2) for d in skin(nods, turns)) if likeness.FOUND else None}


def _far(points, yaw, rows):
    """(rows, BEARINGS): the farthest of `points` from the head's axis (z = `yaw`) in each cell - a row CELL_M of
    height from `rows`[0] on, a bearing about the axis -, -inf where none."""
    import numpy as np
    out = np.full((rows[1] - rows[0], BEARINGS), -np.inf)
    turn = np.arctan2(points[:, 0], points[:, 2] - yaw) / (2.0 * math.pi) + 0.5
    i = np.floor(points[:, 1] / CELL_M).astype(int) - rows[0]
    j = np.floor(turn * BEARINGS).astype(int) % BEARINGS
    keep = (i >= 0) & (i < len(out))
    np.maximum.at(out, (i[keep], j[keep]), np.hypot(points[keep, 0], points[keep, 2] - yaw))
    return out


def skin(nods, turns):
    """(How far inside the page's likeness's head's outer skin `nods` keep - parts the head turns past, the skin
    the nearest it comes through the head's travel -, how far `turns` keep, that turn with it), m, the robot
    upright: about the head's axis, by cells of height and bearing, the skin's farthest point less the parts' -
    the least over the cells both have points in. The skin's mouth is a bag inside the head, open at the lips: no
    part is inside or outside it by crossings, and the outer skin is the farthest."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness
    from machine.body import drives
    from tools.render import leg_worn as lw
    body = gynoid.body(dressed=True, like=likeness.FOUND, plates=True)
    every = [part[0] for part in body.parts]
    frames = body._frames({}, ((0.0, 0.0, 0.0), np.eye(3)))
    pos, _n = body.pose({}, root=((0.0, 0.0, 0.0), np.eye(3)), _frames=frames)
    torso = frames[every.index('torso')]
    local = (pos - torso[1]) @ torso[0]
    owner, index = np.repeat(np.arange(len(every)), [len(part[5][1]) for part in body.parts]), body.index.astype(int)
    cloud = km._points(lw._tris(local, index[owner == every.index('like_head')], body.materials), 0.0005)
    yaw = nl.laid()['yaw']
    rows = (int(cloud[:, 1].min() / CELL_M), int(cloud[:, 1].max() / CELL_M) + 1)
    table = _far(cloud, yaw, rows)
    table = np.where(np.isinf(table), np.inf, table)
    reach = int(math.ceil(max(abs(t) for t in drives.TRAVEL['head']) / (360.0 / BEARINGS)))
    swept = np.min([np.roll(table, k, axis=1) for k in range(-reach, reach + 1)], axis=0)

    def depth(parts, within):
        own = _far(km._points(parts, 0.001), yaw, rows)
        both = np.isfinite(own) & np.isfinite(within)
        return float((within - own)[both].min()) if both.any() else math.inf
    return depth(nods, swept), depth(turns, table)


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import neck_hosed as nh
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    views = [(0.0, 0.0, 25.0, 12.0), (0.0, 0.0, 200.0, 12.0), (60.0, 0.0, 90.0, 6.0), (-45.0, 60.0, 250.0, 12.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (nod, turn, yaw, pitch) in enumerate(views):
        tris = [t for part in list(posed(nod, turn).values()) + list(nh.posed(nod).values()) for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=pitch, px=760, half=0.2, centre=(0.0, 0.37, 0.0)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'neck and head: upright from ahead | from behind | nodded 60 | back 45, turned 60',
                               fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
