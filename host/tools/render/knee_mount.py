"""Knee mounts in saddle blocks: the cables' way, the output's limb, the gaps through the travel.

    python -X utf8 tools/render/knee_mount.py OUT_DIR [--look N] [--h MM]

The knee's stack (`skeleton.cans`) sits in a printed saddle block screwed through its board to the stator's base, a
carbon clamp on the gearbox's ring; the thigh's tube's printed spigot screws into the block's boss, the shank's into a
printed arm on the gearbox's output hub, so nothing that stays reaches where the shank swings. The sizes are
knee_loads' (the hand calculation, raised where the finite elements find it short). The cables come down the thigh's
tube in the spigot's bore, set off along the knee's axis onto the swing's neutral axis, down a channel in the block's
end wall past the knee's board, along a duct under the block into a cassette round the clamp where a flat cable rolls
up and off as the knee bends, and out of it through the arm's boss into the shank's spigot's bore. Each look is drawn
apart, together straight, together at the knee's last bend and cut through its middle, with its margins and the least
gap between what turns and what stays through the knee's travel (`drives.TRAVEL`), sampled a millimetre apart.
"""
import argparse
import json
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_loads as kl  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: Each look: the block ('block' square, 'round' rounded, 'half' a saddle over a guard), honeycomb pockets on it,
#: the screws' socket, their heads sunk, and the output arm ('solid', 'pocket').
LOOKS = {11: ('block', False, 'hex', False, 'solid'), 12: ('block', False, 'torx', False, 'pocket'),
         13: ('round', False, 'hex', False, 'solid'), 14: ('block', True, 'torx', False, 'solid'),
         15: ('round', True, 'hex', False, 'pocket'), 16: ('half', True, 'torx', False, 'solid'),
         17: ('block', False, 'hex', True, 'solid'), 18: ('round', True, 'torx', True, 'solid'),
         19: ('half', True, 'hex', False, 'pocket'), 20: ('round', False, 'torx', True, 'pocket')}
#: The knee's board on the stator's base; the block's end wall over it (a 10 mm channel and its walls); the cassette's
#: room for the rolling cable and its ring's wall; an M5's head (8.5 mm across, 5 high); the bundle's radius (2 x
#: AWG16, an RS485 pair, the foot's sensors: 8 mm) and the knee's branch's; the flat cable's thickness; m.
BOARD_M, END_M, CASSETTE = 0.004, 0.014, (0.008, 0.002)
CABLE_R, BRANCH_R, FLAT_M = 0.004, 0.002, 0.0006
CABLE, BOARD = (232, 112, 40), (40, 128, 70)


def _oval():
    import numpy as np
    t = np.arange(96) * 2.0 * math.pi / 96
    c, s, p = np.cos(t), np.sin(t), kl.POWER
    return np.c_[np.sign(c) * np.abs(c) ** (2 / p) * kl.oval()[0], np.sign(s) * np.abs(s) ** (2 / p) * kl.oval()[1]] / 1e3


def _rod(a, b, r, colour, k=12):
    """A round rod from `a` to `b`, `r` round."""
    import numpy as np
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    e = np.cross(d, [0.0, 0.0, 1.0] if abs(d[2]) < 0.9 else [1.0, 0.0, 0.0])
    e /= np.linalg.norm(e)
    f = np.cross(d, e)
    t = np.arange(k) * 2.0 * math.pi / k
    ring = r * (np.cos(t)[:, None] * e + np.sin(t)[:, None] * f)
    lo, hi = a + ring, b + ring
    return [(np.array(tri), colour) for i in range(k) for tri in
            ((lo[i], lo[(i + 1) % k], hi[(i + 1) % k]), (lo[i], hi[(i + 1) % k], hi[i]))]


def _wire(points, r, colour=CABLE):
    return [tri for a, b in zip(points[:-1], points[1:]) for tri in _rod(a, b, r, colour)]


def _ribbon(yz, x0, x1, colour=CABLE):
    """A flat cable along the (y, z) path across x0..x1."""
    import numpy as np
    tris = []
    for (ya, za), (yb, zb) in zip(yz[:-1], yz[1:]):
        p = [np.array(q) for q in ((x0, ya, za), (x1, ya, za), (x1, yb, zb), (x0, yb, zb))]
        tris += [(np.array((p[0], p[1], p[2])), colour), (np.array((p[0], p[2], p[3])), colour)]
    return tris


def _ring_y(inner, outer, y0, y1, cx, colour):
    import numpy as np
    return [(t[:, [1, 0, 2]] + (cx, 0.0, 0.0), c) for t, c in m._band_x(inner, outer, y0, y1, colour)
            if np.all(np.isfinite(t))]


def _limb(cx, face, way, sz, socket, sunk, tube, dx, apart):
    """The boss's face dressed at height `face` about x = cx, and the tube's flange, spigot, collar, tube and screws
    off it toward `way` (+1 up, -1 down), the cables' bore `dx` off the tube's axis: (dressing, limb)."""
    import numpy as np
    rs, rb, _e = (v / 1e3 for v in kl.spigot())
    outline, at0, bore = _oval(), (cx, 0.0), (cx + dx, 0.0)
    holes = [np.array(b) / 1e3 + at0 for b in kl.bolts()]
    thick, collar, e = sz['flange'] / 1e3, sz['collar'] / 1e3, way * 0.0002
    size = sz['screws']['flange']
    head_r, head_h, shank_r = kl.SCREWS[size][1] / 1e3, kl.SCREWS[size][2] / 1e3, int(size[1:]) / 2e3
    dress = m._prism(outline * 0.9 + at0, face - e, face + e, m.STEEL)
    dress += m._prism(m._circle(m.RING_R, 48, at0), face + e, face + 1.5 * e, m.PRINTED)
    for at in holes:
        dress += m._prism(m._circle(shank_r, 16, at), face + 1.5 * e, face + 2 * e, m.DARK)
    dress += m._prism(m._circle(rb, 32, bore), face + 1.5 * e, face + 2 * e, m.DARK)
    base = face + way * (m.OFF_M if apart else 0.0)
    deck = base + way * thick
    limb = m._prism(outline + at0, base, deck, m.PRINTED)
    for at in holes:
        if sunk:
            limb += m._prism(m._circle(head_r * 1.12, 24, at), deck, deck + e, m.DARK)
        else:
            limb += m._prism(m._circle(head_r, 20, at), deck, deck + e, m.STEEL)
            limb += m._prism(m._circle(shank_r * 1.1, 16, at), deck + e, deck + 1.5 * e, m.DARK)
    tip = deck + way * (kl.lengths()[0] / 1e3 - m.TIP_M)
    limb += m._prism(m._circle(rs, 48, at0), deck, tip, m.PRINTED)
    limb += m._prism(m._circle(rs - 0.0012, 48, at0), tip, deck + way * kl.lengths()[0] / 1e3, m.PRINTED)
    limb += m._prism(m._circle(rb, 32, bore), tip + way * m.TIP_M, tip + way * m.TIP_M + e, m.DARK)
    if collar:
        lo, hi = sorted((deck, deck + way * kl.lengths()[1] / 1e3))
        limb += _ring_y(tube + kl.BOND / 1e3, tube + kl.BOND / 1e3 + collar, lo, hi, cx, m.PRINTED)
    pull = deck + way * (m.PULL_M if apart else 0.0)
    end = pull + way * m.TUBE_M
    limb += m._prism(m._circle(tube, 48, at0), pull, end, m.CARBON)
    limb += m._prism(m._circle(tube - m.WALL_M, 48, at0), end, end + e, m.DARK)
    head = deck - way * (head_h if sunk else 0.0) + way * (m.LIFT_M if apart else 0.0)
    for at in holes:
        limb += m._prism(m._circle(shank_r, 16, at), head - way * (thick + 0.008), head, m.SCREW)
        limb += m._prism(m._circle(head_r, 24, at), head, head + way * head_h, m.SCREW)
        limb += m._prism(m._socket(socket, head_r * 0.5) + at, head + way * head_h,
                         head + way * (head_h + 0.0002), m.DARK)
    return dress, limb


def _stack():
    from machine.body import skeleton
    parts = [(float(c[0]), r, h) for j, _rides, c, _ax, r, h in skeleton.cans() if j == 'left_knee']
    mid = (min(x - h for x, _r, h in parts) + max(x + h for x, _r, h in parts)) / 2.0
    (mx, can, mh), (gx, ring, gh) = sorted(((x - mid, r, h) for x, r, h in parts), key=lambda p: -p[1])
    way = 1.0 if gx > mx else -1.0
    thigh, shank = skeleton.HUNG['thigh'][1], skeleton.HUNG['shank'][1]
    return (can, ring, way * mx - mh, way * gx - gh, way * gx + gh,
            float(thigh[-1] if isinstance(thigh, tuple) else thigh), float(shank[0] if isinstance(shank, tuple) else shank))


def _block(shape, comb, over, top, x0, x1):
    """The saddle block from x0 to x1 about a can `over` round, `top` its outside, with its pockets if `comb`."""
    import numpy as np
    t = np.linspace(0.0, 2.0 * math.pi, 65)[:-1]
    if shape == 'block':
        tris = m._box(x0, x1, top, -top, top, m.PRINTED)
    elif shape == 'round':
        c, s = np.cos(t), np.sin(t)
        tris = m._prism(np.c_[top * np.sign(c) * np.abs(c) ** 0.5, top * np.sign(s) * np.abs(s) ** 0.5], x0, x1,
                        m.PRINTED, axis=0)
    else:
        tris = m._box(x0, x1, top, 0.0, top, m.PRINTED) + m._band_x(0.008, top, x0, x0 + END_M, m.PRINTED)
        tris += [tri for tri in m._band_x(over, top, x0 + END_M, x1, m.PRINTED, gap=(-1.0, 0.5 * math.pi))
                 if min(tri[0][:, 1]) < 1e-9]
    if not comb:
        return tris
    r = top + 0.0003
    for s in (-1.0, 1.0):
        if shape == 'block':
            def lay(u, v, s=s):
                return u, v, s * r
        else:
            def lay(u, v, s=s):
                return u, v, s * ((top ** 4 - min(abs(v), top) ** 4) ** 0.25 + 0.0003)
        tris += _comb(x0 + END_M + 0.002, x1 - 0.002, -0.72 * top if shape != 'half' else 0.08 * top,
                      0.72 * top, lay)
    return tris


def _comb(u0, u1, v0, v1, place, cell=0.0042, bar=0.0016):
    """Dark hex pockets over u0..u1, v0..v1, each corner laid by `place(u, v)` -> point."""
    import numpy as np
    w = math.sqrt(3.0) * cell
    du, dv = w + bar, (w + bar) * math.sqrt(3.0) / 2.0
    corners = [(cell * math.cos(math.pi / 2 + k * math.pi / 3), cell * math.sin(math.pi / 2 + k * math.pi / 3))
               for k in range(6)]
    tris, v, row = [], v0 + cell, 0
    while v <= v1 - cell:
        u = u0 + w / 2 + (du / 2 if row % 2 else 0.0)
        while u <= u1 - w / 2:
            mid, ring = np.asarray(place(u, v)), [np.asarray(place(u + a, v + b)) for a, b in corners]
            tris += [(np.array((mid, ring[k], ring[(k + 1) % 6])), m.DARK) for k in range(6)]
            u += du
        v, row = v + dv, row + 1
    return tris


def _turn(tris, deg):
    return [(tri @ _rx(deg).T, colour) for tri, colour in tris]


def _rx(deg):
    import numpy as np
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def bodies(look, sz, apart=False, flex=0.0, shaped=None):
    """(still, hub, turning, cables): the knee's parts with the stator, the output's hub, those on it turned by
    `flex` degrees, and the cables, laid with the gearbox at +x; the arm as `shaped` (knee_topo's voxels' faces,
    m) where given, else a plate."""
    import numpy as np
    shape, comb, socket, sunk, arm = LOOKS[look]
    can, ring, m0, g0, g1, thigh, shank = _stack()
    over, top = can + m.AIR_M, can + m.AIR_M + m.BAND_M
    off, e = kl.apart() / 1e3, kl.spigot()[2] / 1e3
    xt, xs = -off, off                                   # the tubes either side of the joint's middle
    x0, x1 = m0 - BOARD_M - END_M, xs - kl.oval()[0] / 1e3 - 0.001   # the block: its end wall over the board, its
    channel = x0 + END_M / 2.0                           # far end short of where the shank's boss swings
    still = m._cylinder_x(can, m0, g0, m.SERVO) + m._cylinder_x(ring, g0, g1, m.GEAR)
    still += m._cylinder_x(0.040, m0 - BOARD_M, m0, BOARD) + _block(shape, comb, over, top, x0, x1)
    n, radius = kl.BASE
    head_r, head_h = (v / 1e3 for v in kl.SCREWS[sz['screws']['base']][1:])
    for a in np.arange(n) * 2.0 * math.pi / n + math.pi / n:
        at = (radius / 1e3 * math.cos(a), radius / 1e3 * math.sin(a))
        still += m._cylinder_x(head_r, x0 - head_h, x0, m.SCREW, k=20, at=at)
        still += m._prism(m._socket(socket, head_r * 0.5) + at, x0 - head_h - 0.0002, x0 - head_h, m.DARK, 0)
    band = max(x1, g0 + 0.001)                           # the clamp on the gearbox's ring, clear of the can
    still += m._band_x(ring, ring + m.BAND_M, band, g1 - 0.002, m.CARBON)
    duct = -top - 0.003                                  # the duct's middle
    still += m._box(channel - 0.006, x1, 0.007, -top - 0.007, -top + 0.001, m.PRINTED)
    still += m._prism(_oval() + (xt, 0.0), top - 0.004, top + 0.012, m.PRINTED)
    face_t = top + 0.012
    dress, limb = _limb(xt, face_t, 1.0, sz, socket, sunk, thigh, -e, apart)
    still += dress + limb
    ra = ring + m.BAND_M + 0.001                         # the cassette: from the clamp out to its turning ring
    rb = ra + CASSETTE[0]
    inner = rb + CASSETTE[1]
    c0, c1 = band + 0.0015, g1 - 0.001
    arm_m = sz['arm'] / 1e3
    face_s = -(inner + 0.012)
    hub = m._cylinder_x(0.024, g1, g1 + 0.003, m.GEAR)
    turning = m._band_x(rb, inner, c0, g1 + 0.003, m.PRINTED) + m._band_x(ring + 0.002, inner, c1, g1 + 0.003,
                                                                          m.PRINTED)
    n, radius = kl.HUB
    if shaped is not None:                               # the arm the loads laid, its hub's face where it ends
        turning += [(tri, m.PRINTED) for tri in shaped]
        near = shaped.reshape(-1, 3)
        cap = float(near[np.hypot(near[:, 1], near[:, 2]) <= radius / 1e3, 0].max())
    else:
        turning += m._cylinder_x(inner, g1 + 0.003, g1 + 0.003 + arm_m, m.PRINTED)
        turning += m._box(g1 + 0.003, g1 + 0.003 + arm_m, kl.oval()[1] / 1e3, face_s, 0.0, m.PRINTED)
        if arm == 'pocket':
            turning += m._prism(np.array([[-0.030, -0.022], [-0.030, 0.022], [-0.046, 0.022], [-0.046, -0.022]]),
                                g1 + 0.003 + arm_m, g1 + 0.0033 + arm_m, m.DARK, axis=0)
        turning += m._prism(_oval() + (xs, 0.0), -inner, face_s, m.PRINTED)
        cap = g1 + 0.003 + arm_m
    head_r, head_h = (v / 1e3 for v in kl.SCREWS[sz['screws']['hub']][1:])
    cap += 0.02 if apart else 0.0
    for a in np.arange(n) * 2.0 * math.pi / n:
        at = (radius / 1e3 * math.cos(a), radius / 1e3 * math.sin(a))
        turning += m._cylinder_x(head_r, cap, cap + head_h, m.SCREW, k=20, at=at)
        turning += m._prism(m._socket(socket, head_r * 0.5) + at, cap + head_h, cap + head_h + 0.0002, m.DARK, 0)
    dress, limb = _limb(xs, face_s, -1.0, sz, socket, sunk, shank, -e, apart)
    turning += dress + limb
    cmid = 0.5 * (c0 + c1)
    sleeve = sz['flange'] / 1e3 + 0.06                   # the cables run on up and down the tubes
    cables = _wire([(xt - e, face_t + sleeve, 0.0), (xt - e, face_t, 0.0), (channel, top - 0.012, 0.0),
                    (channel, -top + 0.004, 0.0), (channel + 0.006, duct, 0.0),
                    (x1 - 0.003, duct, 0.0), (x1 - 0.003, -(ra + rb) / 2.0, 0.0), (c0 + 0.002, -(ra + rb) / 2.0, 0.0),
                    (c0 + 0.002, -ra - 0.0005, 0.0)], CABLE_R)
    cables += _wire([(channel, 0.03, 0.0), (m0 - BOARD_M / 2.0, 0.03, 0.0)], BRANCH_R)      # the knee's own branch
    s = np.linspace(0.0, 1.0, 160)
    t, rr = math.pi + s * (2.0 * math.pi + math.radians(flex)), ra + 0.0008 + s * (rb - ra - 0.0016)
    cables += _ribbon(list(zip(rr * np.cos(t), rr * np.sin(t))), c0 + 0.001, c1 - 0.001)
    out = _wire([(cmid, -rb + 0.0005, 0.0), (cmid, -(inner + 0.006), 0.0), (xs - e, -(inner + 0.006), 0.0),
                 (xs - e, face_s - sleeve, 0.0)], CABLE_R)
    if apart:
        turning = [(tri + (0.03, 0.0, 0.0), c) for tri, c in turning]
        return still, hub, turning, []
    return still, hub, _turn(turning, flex), cables + _turn(out, flex)


def _points(tris, step=0.001):
    """Points over the triangles' surfaces about `step` apart."""
    import numpy as np
    rng, out = np.random.default_rng(0), []
    for tri, _c in tris:
        area = 0.5 * float(np.linalg.norm(np.cross(tri[1] - tri[0], tri[2] - tri[0])))
        u = rng.random((max(3, int(2.0 * area / step ** 2)), 2))
        u[u.sum(axis=1) > 1] = 1 - u[u.sum(axis=1) > 1]
        out.append(tri[0] + u[:, :1] * (tri[1] - tri[0]) + u[:, 1:] * (tri[2] - tri[0]))
    return np.concatenate(out)


def _key(cells):
    big = 1 << 20
    return ((cells[:, 0] + big) * (2 * big) + cells[:, 1] + big) * (2 * big) + cells[:, 2] + big


def _least(keys, ordered, pts, h):
    """The least distance from `pts` to the sorted voxel-keyed points (`keys`, `ordered`), or `h` past it."""
    import numpy as np
    best, cells = h, np.floor(pts / h).astype(np.int64)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                k = _key(cells + (dx, dy, dz))
                lo, hi = np.searchsorted(keys, k, 'left'), np.searchsorted(keys, k, 'right')
                n = hi - lo
                if n.any():
                    which = np.repeat(np.arange(len(pts)), n)
                    start = np.repeat(lo - np.r_[0, np.cumsum(n)[:-1]], n) + np.arange(int(n.sum()))
                    best = min(best, float(np.sqrt(((pts[which] - ordered[start]) ** 2).sum(axis=1)).min()))
    return best


def gap(look, sz, h=0.006, shaped=None):
    """The least distance, m, between what stays and what turns through the knee's travel (`h` if none nearer)."""
    import numpy as np
    from machine.body import drives
    still, _hub, turning, _cables = bodies(look, sz, shaped=shaped)
    a, pts = _points(still), _points(turning)
    keys = _key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    lo, hi = drives.TRAVEL['knee']
    return min(_least(keys[order], a[order], pts @ _rx(d).T, h) for d in np.arange(lo, hi + 0.1, 5.0))


def main(argv=None):
    from PIL import Image, ImageDraw
    from machine.body import drives
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--look', type=int, action='append')
    parser.add_argument('--h', type=float, default=1.5, help='the finite elements\' voxel, mm')
    parser.add_argument('--sizes', help='the sizes knee_loads.py --fem --json kept, instead of settling anew')
    parser.add_argument('--arm', help='the arm knee_topo.py laid (its .npz), instead of a plate')
    args = parser.parse_args(argv)
    if args.sizes:
        with open(args.sizes, encoding='utf-8') as f:
            sz = json.load(f)
    else:
        sz = kl.settled(args.h)
    last, ld, fem = drives.TRAVEL['knee'][1], sz['loads'], sz['fem']
    shaped, laid = None, None
    if args.arm:
        import numpy as np
        from tools.render import knee_topo
        laid = np.load(args.arm)
        shaped = knee_topo.faces(laid['points'], laid['cells']) / 1e3
    rs, rb, e = kl.spigot()
    for look in args.look or sorted(LOOKS):
        shape, comb, socket, sunk, arm = LOOKS[look]
        still, hub, turning, _c = bodies(look, sz, True, shaped=shaped)
        apart = m.draw(still + hub + turning, px=600, half=0.27, centre=(0.02, 0.0, 0.0))
        still, hub, turning, cables = bodies(look, sz, shaped=shaped)
        straight = m.draw(still + hub + turning + cables, px=600, half=0.17, centre=(0.01, 0.0, 0.0))
        cut = m.draw(still + hub + turning + cables, yaw=-15.0, pitch=10.0, px=600, half=0.17,
                     centre=(0.01, 0.0, 0.0), cut=0.0)
        still, hub, turning, cables = bodies(look, sz, flex=last, shaped=shaped)
        bent = m.draw(still + hub + turning + cables, yaw=60.0, pitch=22.0, px=600, half=0.17,
                      centre=(0.01, 0.03, 0.0))
        least = gap(look, sz, shaped=shaped)
        sheet = Image.new('RGB', (2400, 692), (18, 18, 18))
        for i, img in enumerate((apart, straight, bent, cut)):
            sheet.paste(img, (600 * i, 92))
        pen = ImageDraw.Draw(sheet)
        screws = sz['screws']
        armed = ('arm %d mm %s' % (sz['arm'], arm) if laid is None else
                 'arm laid by topology optimisation, %.0f g, x%.2f with layers across %s' % (
                     1e3 * float(laid['mass']), float(laid['margin']), str(laid['layers'])))
        lines = [('%d: %s saddle block%s; tubes +-%.1f mm off the middle; oval flanges %d mm, collar %d mm, 4 x %s %s%s;'
                  ' %s, hub %s, base %s | apart | straight | bent %.0f deg | cut' % (
                      look, shape, ' with honeycomb pockets' if comb else '', kl.apart(), sz['flange'], sz['collar'],
                      screws['flange'], socket, ' heads sunk' if sunk else ' via steel sleeves', armed,
                      screws['hub'], screws['base'], last), (235, 235, 235)),
                 ('hand, measured x1.5 (swing %.0f, lateral %.0f, torsion %.0f N m, axial %.0f N): %s' % (
                     ld['swing'] / 1e3, ld['lateral'] / 1e3, ld['torsion'] / 1e3, ld['axial'],
                     ' | '.join('%s x%.2f' % kv for kv in sz['margins'].items())), (200, 200, 200)),
                 ('finite elements (%.1f mm voxels, printed with its layers across %s): %s' % (
                     fem['h'], fem['layers'], ' | '.join('%s x%.2f' % kv for kv in fem['margins'].items())),
                  (200, 200, 200)),
                 ('least gap, still vs turning, %.0f..%.0f deg: %s | cables: 8 mm bundle in %.0f mm bores %.1f mm off '
                  'the axis, a flat cable rolling in the cassette' % (
                      drives.TRAVEL['knee'] + ('%.1f mm' % (least * 1e3) if least >= 0.0005 else 'COLLIDES',
                                                2 * rb, e)),
                  (150, 220, 150) if least >= 0.0005 else (235, 120, 120))]
        for i, (text, colour) in enumerate(lines):
            pen.text((12, 6 + 21 * i), text, fill=colour)
        path = os.path.join(args.out, 'montering_%02d.png' % look)
        sheet.save(path)
        print(path, '%.1f mm' % (least * 1e3))
    return 0


if __name__ == '__main__':
    sys.exit(main())
