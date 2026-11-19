"""The hip's printed frame: the yaw's yoke and block, the thigh's arm and knuckle, a print each.

    from tools.render import hip_frame

The hip's members where `hip_laid` lays them, each a piece that stands on one flat face and hangs nothing over its
bed (`prints`, `solids`), every root rounded. The yoke: the crown under the yaw drive's hub and its two cheeks in
one, grown off the crown's under face, a fin up each cheek's back end. The block: the slit band round the roll
drive's motor and gearbox, the pin's bearings' housing under it and an ear either side, grown off its front face -
every bore upright in print. The thigh's arm: a head on the pitch drive's hub - the hub's eight screws through it,
steel sleeves round them and a steel ring under their heads -, a skirt round the output ring, the arm down the
hip's outer side and a pad on its foot's inner face, grown off its outer face. The thigh's knuckle: the jog in
under the cradle with the tube's collar and spigot, a chin of 45 degrees ahead under the tube's front half and a
swell behind where nothing passes, grown down the tube off its top face. Where two prints meet, screws go through
both (`hip_laid.JOINT`) in steel sleeves, steel under their heads and nuts or a cross dowel for a nut: the block's
ears on the cheeks' and fins' back ends, the knuckle's end on the arm's pad - the screws pull, the sleeves shear,
no print squeezed by a screw's preload (`hip_laid.checks`).
"""
import math

from tools.render import clamped as cl
from tools.render import elbow_mount as em
from tools.render import foot
from tools.render import hip_laid as hl
from tools.render import knee_clamped as kc
from tools.render import knee_loads as kl
from tools.render import knee_mount as km
from tools.render import mounting as m
from tools.render import tees

#: Steel's ink.
STEEL = (200, 204, 212)
#: The pin's housing's cap behind the pin's end, where the bus turns into its bore, as a share of the housing's
#: wall; where round the roll drive's band its slit stands, rad from over it toward the hip's inner side: under the
#: inner ear, clear of the web, of the riser on its top and of the trunk down its outer side.
CAP, SLIT = 1.5, math.radians(130.0)
#: A root's round, m; the crown's back edge and the cheeks' front ends, z m.
ROOT_M, CROWN_AFT, CHEEK_FORE = 0.004, -0.045, -0.005


def _along(which):
    """The turn laying a stack drawn along x (its gearbox at +x) along the yaw's axis, its gearbox down, or along the
    roll's, its gearbox ahead."""
    import numpy as np
    return np.array({'yaw': [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                     'roll': [[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]}[which])


def _put(tris, turn, shift=(0.0, 0.0, 0.0)):
    return [(tri @ turn.T + shift, colour) for tri, colour in tris]


def _prism(outline, a, b, colour, axis):
    return em._rounded(outline, a, b, colour, axis=axis, r=cl.EDGE_M, steps=1)


def _box(u, v):
    import numpy as np
    return np.array([(u[0], v[0]), (u[1], v[0]), (u[1], v[1]), (u[0], v[1])])


def _slab(outline, a, b, colour, axis=0, rounded=True):
    """A profile - simple, not convex (`foot._slab`), its corners rounded ROOT_M unless not `rounded` - from `a` to
    `b`: its outline (y, z) along x, or (x, y) along z (`axis` 2)."""
    import numpy as np
    tris = foot._slab(cl.filleted(outline, ROOT_M) if rounded else np.asarray(outline, float), a, b, colour)
    return tris if axis == 0 else [(tri[:, (1, 2, 0)], c) for tri, c in tris]


def _arc(centre, r, a0, a1, n):
    import numpy as np
    return [(centre[0] + r * math.cos(a), centre[1] + r * math.sin(a)) for a in np.linspace(a0, a1, n + 1)]


def _touch(p, r, turn):
    """Where the tangent from `p` touches the circle `r` round the origin: the one `turn` (+1 or -1) round from it."""
    a = math.atan2(p[1], p[0]) + turn * math.acos(r / math.hypot(*p))
    return r * math.cos(a), r * math.sin(a)


def _root(r, d, rho, n=8):
    """[(u, v)]: the round `rho` in the corner where the line v = `d` leaves a circle `r` round the origin, the open
    side past the line: the corner itself, then from where the round touches the line to where it touches the circle."""
    c = (math.sqrt((r + rho) ** 2 - (d + rho) ** 2), d + rho)
    end = math.atan2(-c[1], -c[0])
    return [(math.sqrt(r * r - d * d), d)] + _arc(c, rho, -0.5 * math.pi, end - (2.0 * math.pi if end > 0.0 else 0.0), n)


def _sunk(spots, face, way, axis, head, kind='torx'):
    """[(tri, colour)]: screws' ends sunk flush in a face across `axis` at `face`, the open side `way` (+1 or -1)
    of it, each at its spot (the two other coordinates, in order): a head (`head`: its radius, its height) with its
    Torx socket; a 'nut', six-sided, the screw's end in it; or a 'barrel' - a cross dowel's end, slotted."""
    import numpy as np
    out, skin = [], face + way * 0.0003
    for at in spots:
        out += m._prism(m._circle(head[0], 6 if kind == 'nut' else 16, at), face - way * head[1], skin, em.BOLT, axis)
        tip = {'torx': m._socket('torx', 0.5 * head[0]), 'nut': m._circle(0.5 * head[0], 12),
               'barrel': np.array([(-0.8, -0.12), (0.8, -0.12), (0.8, 0.12), (-0.8, 0.12)]) * head[0]}[kind] + at
        out += m._prism(tip, skin, skin + way * 0.0002, STEEL if kind == 'nut' else em.VOID, axis)
    return out


def _screwed(spots, face, way, axis, kind='torx', plate=None):
    """A joint's screws' ends at `spots` on a face (`_sunk`'s): their heads (`hip_laid.JOINT`'s) - or their nuts -
    on steel let flush into the face: a strap `hip_laid.STRAP_M` wide from the first to the last, or a `plate` -
    the face's own outline, drawn 2 mm inside its edge."""
    import numpy as np
    (_preload, r, tall), a, b = kl.SCREWS[hl.JOINT], face - way * 0.002, face + way * 0.0002
    if plate is None:
        p, half = np.asarray(spots, float), 0.5 * hl.STRAP_M
        v = np.array([p[0][1] - p[-1][1], p[-1][0] - p[0][0]]) * half / float(np.linalg.norm(p[-1] - p[0]))
        steel = m._prism(np.array([p[0] - v, p[-1] - v, p[-1] + v, p[0] + v]), a, b, STEEL, axis)
        steel += [t for at in (p[0], p[-1]) for t in m._prism(m._circle(half, 16, at), a, b, STEEL, axis)]
    else:
        edge = cl.filleted(plate, ROOT_M)
        t1, t2 = edge - np.roll(edge, 1, axis=0), np.roll(edge, -1, axis=0) - edge
        n = (np.c_[t1[:, 1], -t1[:, 0]] / np.linalg.norm(t1, axis=1, keepdims=True)
             + np.c_[t2[:, 1], -t2[:, 0]] / np.linalg.norm(t2, axis=1, keepdims=True))
        turn = np.sign(np.sum(edge[:, 0] * np.roll(edge[:, 1], -1) - np.roll(edge[:, 0], -1) * edge[:, 1]))
        steel = _slab(edge - 0.002 * turn * n / np.linalg.norm(n, axis=1, keepdims=True), a, b, STEEL, axis, False)
    return steel + _sunk(spots, face + way * 0.0002, way, axis, (1e-3 * r, 1e-3 * tall), kind)


def _ring():
    """The hub's screws' spots round its axis, m."""
    count, circle = kl.HUB
    return [(1e-3 * circle * math.cos((k + 0.5) * 2.0 * math.pi / count),
             1e-3 * circle * math.sin((k + 0.5) * 2.0 * math.pi / count)) for k in range(count)]


def yoke(low, back):
    """{'yoke', 'hub screws'}: the crown under the yaw drive's hub's face at `low`, the hub's screws' heads sunk in
    its under face; its two cheeks either side of the yaw's ring back to the block's front face at `back`, a fin up
    each's back end to take the block's upper screw; a root's round where a cheek leaves the crown - no more than
    the ring's air - and where a fin leaves its cheek. One print, grown off the crown's under face."""
    import numpy as np
    floor = low - hl.CROWN_M
    out = _prism(np.array([(-hl.CHEEK[1], CROWN_AFT), (hl.CHEEK[1], CROWN_AFT), (hl.CHEEK[1], 0.0),
                           (hl.HUB_R, hl.CROWN_Z), (-hl.HUB_R, hl.CROWN_Z), (-hl.CHEEK[1], 0.0)]), floor, low, em.FRAME, 1)
    for way in (1.0, -1.0):
        side, x = sorted((way * hl.CHEEK[0], way * hl.CHEEK[1])), way * hl.CHEEK[0]
        out += _prism(_box(side, (floor, hl.CHEEK[2])), back, CHEEK_FORE, em.FRAME, 2)
        out += _prism(_box(side, (hl.CHEEK[2] - ROOT_M, hl.FIN[0])), back, hl.FIN[1], em.FRAME, 2)
        out += m._prism(np.array([(x, low), (x - way * hl.AIR_M, low), (x, low + hl.AIR_M)]), CROWN_AFT, CHEEK_FORE,
                        em.FRAME, 2)
        out += m._prism(np.array([(hl.CHEEK[2], hl.FIN[1]), (hl.CHEEK[2], hl.FIN[1] + ROOT_M),
                                  (hl.CHEEK[2] + ROOT_M, hl.FIN[1])]), side[0], side[1], em.FRAME, 0)
    return {'yoke': out, 'hub screws': _sunk(_ring(), floor, -1.0, 1, em.HEAD)}


def block(stack, wall, low):
    """{'block', 'block screws'}: the roll drive's (`stack`, its band's `wall`) slit band round its motor and its
    gearbox's housing, the pin's two needle bearings' housing under it - its cap behind the pin's end apart, the
    bus's way in -, a web between and an ear either side, its roots on the band rounded; the ears lie on the back
    ends of the yoke's cheeks and fins (the yaw drive's hub's face at `low`), two screws through each: their heads
    on a strap on the ear's back, each into a cross dowel in the yoke - the upper one's in from the fin's outer
    side, the lower one's up from the yoke's under face. One print, grown off its front face."""
    import numpy as np
    (m0, m1, can), (_g0, g1, r_g) = stack['motor'], stack['gear']
    at, outside, floor, rear = (hl.ROLL_AT[0], hl.ROLL_AT[1], 0.0), can + wall, low - hl.CROWN_M, g1 - hl.EAR_M
    out = _put(cl.turned([(m0 + 0.002, can, outside), (m1, can, outside), (m1, r_g + 0.001, outside),
                          (g1, r_g + 0.001, outside)], SLIT), _along('roll'), at)
    (_f0, f1), (b0, _b1) = hl.needles()
    boss = hl.NEEDLE[0] + hl.HOUSED_M
    out += _prism(m._circle(boss, 48), b0, f1 + hl.AIR_M, em.FRAME, 2)
    out += _prism(m._circle(boss, 48), b0 - CAP * hl.HOUSED_M, b0, tees.PIPE_INK, 2)
    out += _prism(cl.filleted(cl.bridged((0.0, 0.0), boss - 0.001, hl.ROLL_AT, outside - 0.001, 1.6 * boss), 0.003),
                  m0 + 0.002, g1, em.FRAME, 2)
    screws = []
    for way in (1.0, -1.0):
        side = sorted((way * hl.CHEEK[0], way * hl.CHEEK[1]))
        out += _prism(_box(side, (floor, hl.FIN[0])), rear, g1, em.FRAME, 2)
        over = [(-way * v, at[1] + u) for u, v in _root(outside, -hl.CHEEK[0], ROOT_M)]      # at the ear's inner face
        under = [(way * u, at[1] - v) for u, v in _root(outside, at[1] - floor, ROOT_M)]     # at its under face
        out += _slab(over, rear, g1, em.FRAME, 2, False) + _slab(under, rear, g1, em.FRAME, 2, False)
        spots, dowel = [(way * x, y) for x, y in hl.BLOCK_AT], (hl.BARREL[0], hl.BARREL[0])
        screws += _screwed(spots, rear, -1.0, 2)
        screws += _sunk([(spots[1][1], hl.FIN[1] - hl.BARREL[0] - 0.001)], way * hl.CHEEK[1], way, 0, dowel, 'barrel')
        screws += _sunk([(spots[0][0], CHEEK_FORE - 2.0 * hl.BARREL[0])], floor, -1.0, 1, dowel, 'barrel')
    return {'block': out, 'block screws': screws}


def thigh(stack, riser):
    """{part: [(tri, colour)]} of the thigh on the pitch drive's (`stack`) output, upright, the hip's frame: 'neck'
    - the head on the hub's face, the skirt round the output ring, the arm down the hip's outer side, its front's
    root on the head rounded, the pad on its foot's inner face and the pad's ramp -, 'neck screws' - the hub's
    eight, on a steel ring let into the head -, 'knuckle' - the jog, its chin and swell, the tube's collar and its
    spigot -, 'knuckle screws' - four through the arm's foot, its pad and the knuckle's length, heads on the arm's
    outer face and nuts on the knuckle's inner end, steel under each -, 'thigh tube' and 'riser': the hose's fitting at
    `riser` (x, its reach off the pitch's axis, its w) and its pipe down into the knuckle's inner end."""
    import numpy as np
    from machine.body.figure import THIGH
    (q0, q1, r_q) = stack['gear']
    split, face, x0 = q0 + em.OUTPUT_SHARE * (q1 - q0), q1 + 0.0005, q1 + hl.ARM_X
    x1, head, cup, pad = x0 + hl.ARM_M, hl.HEAD_R, r_q + 0.0005, x0 - hl.PAD_M
    (aft, fore), (ahead, up), (top, low) = hl.ARM_Z, hl.ARM_CUT, hl.JOG
    tube_r, bone = kc._stack()[3], kc.laid()['bone']
    collar, skirt = tube_r + em.COLLAR_M + 0.0002, r_q + 1e-3 * kl.WALL
    xi, start, chin = bone - collar, (hl.ARM_FROM, hl.ARM_NECK), collar + hl.NOSE_M

    def flat(points):                                    # (reach, w) to a slab's outline (y, z)
        return np.array([(-r, w) for r, w in points])
    sole = ([start, (start[0] + start[1] - aft, aft), (low - up, aft), (low, aft + ahead)] + hl.chin(chin, hl.CHIN_M)
            + [(top, hl.JOG_Z[1]), (top, fore)])
    root = _root(head, fore, 1.5 * ROOT_M)[1:]           # the front's root on the head, then round the head
    over = _arc((0.0, 0.0), head, math.atan2(root[-1][1], root[-1][0]),
                math.atan2(*_touch(start, head, -1.0)[::-1]) % (2.0 * math.pi), 48)[1:]
    under = _arc((0.0, 0.0), cup, math.asin(fore / cup), math.atan2(*_touch(start, cup, -1.0)[::-1]), 32)
    neck = _slab(flat(sole + root + over), face, x1, em.FRAME) + _slab(flat(sole + under), x0, face, em.FRAME)
    neck += cl.turned([(split + 0.0025, r_q, skirt), (face - 2.0 * (skirt - head), r_q, skirt), (face, r_q, head),
                       (face + cl.EDGE_M, r_q, head)], None)                    # its foot in the head, no groove
    neck += _slab(flat(hl.jog(chin, hl.CHIN_M)), pad, x0, em.FRAME)
    ramp = [(x0, -(top - hl.PAD_M)), (x0, -top), (pad, -top)]                       # (x, y): 45 degrees up to the pad
    neck += _slab(ramp, hl.JOG_Z[0] + hl.JOG_CUT, hl.JOG_Z[1], em.FRAME, 2, False)
    circle = 1e-3 * kl.HUB[1]
    ring = m._band_x(circle - 0.5 * hl.STRAP_M, circle + 0.5 * hl.STRAP_M, x1 - 0.002, x1 + 0.0002, STEEL)
    knuckle = _slab(flat(hl.jog(chin, collar)), xi, pad, em.FRAME)
    knuckle += km._ring_y(tube_r + 0.0002, collar, -low - em.COLLAR_SHARE * 2.0 * tube_r, -low, bone, em.FRAME)
    knuckle += m._prism(m._circle(1e-3 * kl.spigot()[0], 32, (bone, 0.0)), -low - kl.SPIGOT_SHARE * 2.0 * tube_r, -low,
                        em.FRAME)
    spots = [(-r, w) for r, w in hl.KNUCKLE_AT]
    screws = (_screwed(spots, x1, 1.0, 0, plate=flat(hl.jog(chin, hl.CHIN_M)))
              + _screwed(spots, xi, -1.0, 0, 'nut', flat(hl.jog(chin, collar))))
    knee = -THIGH * math.cos(kc.tilt(kl.stated())) + kc.seats(kl.stated())[0] + em.SEAT_M + em.FLANGE_M
    x_f, off, w = riser
    way = [(xi, -(top + hl.ENTRY_M), w), (x_f, -(top + hl.ENTRY_M), w), (x_f, -off - 0.007, w), (x_f, -off, w)]
    pipe = [t for a, b in zip(way[:-2], way[1:-1]) for t in km._rod(np.array(a), np.array(b), tees.PIPE[1], tees.PIPE_INK)]
    pipe += km._rod(np.array(way[-2]), np.array(way[-1]), tees.PIPE[1] + 0.0016, tees.PIPE_INK)
    return {'neck': neck, 'neck screws': ring + _sunk(_ring(), x1 + 0.0002, 1.0, 0, em.HEAD), 'knuckle': knuckle,
            'knuckle screws': screws, 'riser': pipe,
            'thigh tube': m._prism(m._circle(tube_r, 40, (bone, 0.0)), knee, -low, em.CARBON)}
