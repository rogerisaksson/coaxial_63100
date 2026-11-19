"""Exploded sketches of a joint's mount: the servo's holder, the tube's printed spigot and its screws.

    python -X utf8 tools/render/mounting.py OUT_DIR [--look N]

One joint (the left knee: its servo's stack as `skeleton.cans` lays it, the thigh's tube's radius as the build's
`[skeleton] hung` has it) drawn alone, apart and put together side by side, a look a picture (LOOKS). The holder,
printed, keeps a guard's air off the turning can: its plate screwed to the stator's base, a carbon clamp on the
gearbox's ring, and a flat boss whose face bears a tapped steel ring (or brass inserts) and the cables' hole. The
tube's spigot, printed with its flange, is bonded into the tube; steel sleeves in the flange take the M4 socket
screws' preload. The sizes are the knee's at its gearbox's shock torque, 128 N m, times 1.5: the spigot's root
44 MPa of the printed polymer's 50 (`materials.toml` paht_cf, printed lying), the 0.2 mm epoxy bond yielding at the
tube's mouth with 1.75 to spare, the flange 12 mm across its layers, 4 x M4 12.9 on steel unparted with 2.0 to spare.
"""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Each look: the holder ('cradle' a bridge over the can, 'sleeve' a sleeve with a window, 'saddle' a block), the
#: flange ('round', 'square', 'lobes'), its screws, their socket ('hex' or 'torx'), whether the heads sink into the
#: flange, and what the screws thread into ('ring' a tapped steel ring, 'inserts' brass inserts).
LOOKS = {1: ('cradle', 'round', 4, 'hex', False, 'ring'), 2: ('cradle', 'round', 4, 'torx', False, 'ring'),
         3: ('cradle', 'square', 4, 'hex', False, 'ring'), 4: ('cradle', 'round', 6, 'torx', False, 'ring'),
         5: ('cradle', 'lobes', 4, 'hex', False, 'ring'), 6: ('saddle', 'round', 4, 'torx', False, 'ring'),
         7: ('saddle', 'square', 4, 'hex', False, 'ring'), 8: ('sleeve', 'round', 4, 'hex', True, 'ring'),
         9: ('sleeve', 'lobes', 6, 'torx', False, 'ring'), 10: ('cradle', 'round', 8, 'hex', False, 'inserts')}
CARBON, PRINTED, STEEL, SCREW = (70, 74, 84), (112, 106, 98), (168, 172, 180), (96, 98, 108)
BRASS, DARK, SERVO, GEAR = (200, 162, 74), (10, 10, 12), (255, 48, 200), (196, 36, 152)
#: An M4 cap screw (head 7 mm across, 4 high, shank through the sleeve and 8 mm of thread), the steel sleeves, the
#: bolt circle, the flange's reach and thickness, the spigot (0.2 mm bond in the tube), its tip's taper, the cables'
#: bore, the tube's wall, air over the turning can, the holder's band, the steel ring; the sketch's spreads, m.
HEAD_R, HEAD_H, SHANK_R, SHANK_L = 0.007 / 2.0, 0.004, 0.002, 0.020
SLEEVE_R, CIRCLE_R, REACH_M, FLANGE_M = 0.004, 0.026, 0.033, 0.012
SPIGOT_R, SPIGOT_M, TIP_M, BORE_R = 0.0178, 0.050, 0.003, 0.007
WALL_M, AIR_M, BAND_M, RING_R = 0.002, 0.002, 0.005, 0.022
LIFT_M, OFF_M, PULL_M, TUBE_M = 0.03, 0.045, 0.06, 0.07
NAMES = {'cradle': 'bridge + clamp', 'sleeve': 'sleeve + clamp', 'saddle': 'saddle block + clamp'}


def _prism(outline, a, b, colour, axis=1):
    """[(triangle (3, 3), colour)]: the prism of the 2-d `outline` (k, 2) from `a` to `b` along `axis`."""
    import numpy as np
    k = len(outline)

    def lay(h):
        cols = [outline[:, 0], outline[:, 1]]
        cols.insert(axis, np.full(k, h))
        return np.c_[cols[0], cols[1], cols[2]]
    low, high = lay(a), lay(b)
    middle_low, middle_high = low.mean(axis=0), high.mean(axis=0)
    tris = []
    for i in range(k):
        j = (i + 1) % k
        tris += [(low[i], low[j], high[j]), (low[i], high[j], high[i]),
                 (middle_low, low[j], low[i]), (middle_high, high[i], high[j])]
    return [(np.array(t), colour) for t in tris]


def _box(x0, x1, w, y0, y1, colour):
    """A block over x0..x1, `w` either side of the middle, from height y0 to y1."""
    import numpy as np
    return _prism(np.array([[x0, -w], [x1, -w], [x1, w], [x0, w]]), y0, y1, colour)


def _circle(r, k=32, at=(0.0, 0.0)):
    import numpy as np
    t = np.arange(k) * 2.0 * math.pi / k
    return np.c_[at[0] + r * np.cos(t), at[1] + r * np.sin(t)]


def _outline(kind, r, n, offset):
    """The flange's outline about the tube, `r` its reach, lobes (if any) on the `n` screws from `offset`."""
    import numpy as np
    t = np.arange(96) * 2.0 * math.pi / 96
    if kind == 'square':
        p, s = 6.0, 0.88 * r
        return np.c_[np.sign(np.cos(t)) * np.abs(np.cos(t)) ** (2 / p),
                     np.sign(np.sin(t)) * np.abs(np.sin(t)) ** (2 / p)] * s
    if kind == 'lobes':
        lobe = np.clip(np.cos(n * (t - offset)), 0.0, None) ** 0.7
        reach = 0.024 + (r + 0.002 - 0.024) * lobe
        return np.c_[reach * np.cos(t), reach * np.sin(t)]
    return _circle(r, 96)


def _socket(kind, r):
    """A screw's socket's outline: a hexagon or a six-lobed star."""
    import numpy as np
    if kind == 'hex':
        return _circle(r, 6)
    t = np.arange(48) * 2.0 * math.pi / 48
    reach = r * (0.72 + 0.28 * np.cos(6 * t))
    return np.c_[reach * np.cos(t), reach * np.sin(t)]


def _cylinder_x(r, x0, x1, colour, k=40, at=(0.0, 0.0)):
    """[(triangle, colour)]: a drum along x from `x0` to `x1`, `r` round, its middle at y, z = `at`."""
    return _prism(_circle(r, k, at), x0, x1, colour, axis=0)


def _band_x(inner, outer, x0, x1, colour, k=48, gap=None):
    """[(triangle, colour)]: a band round the x axis between radii, open over `gap` (rad, rad) where a window is."""
    import numpy as np
    t = np.arange(k + 1) * 2.0 * math.pi / k
    keep = [i for i in range(k) if gap is None or not (gap[0] < t[i] < gap[1] or gap[0] < t[i + 1] < gap[1])]
    tris = []
    for i in keep:
        for r in (outer, inner):
            p = [np.array([x, r * math.cos(t[n]), r * math.sin(t[n])]) for x, n in ((x0, i), (x0, i + 1),
                                                                                    (x1, i + 1), (x1, i))]
            tris += [(np.array((p[0], p[1], p[2])), colour), (np.array((p[0], p[2], p[3])), colour)]
        for x in (x0, x1):
            q = [np.array([x, r * math.cos(t[n]), r * math.sin(t[n])]) for r, n in ((inner, i), (outer, i),
                                                                                    (outer, i + 1), (inner, i + 1))]
            tris += [(np.array((q[0], q[1], q[2])), colour), (np.array((q[0], q[2], q[3])), colour)]
    return tris


def _holder(holder, can, ring, m0, g0, g1):
    """The holder about a stack laid with its gearbox toward +x: plate, clamp, body; and the height of its top."""
    import numpy as np
    over = can + AIR_M                                     # the holder keeps off the turning can
    top = over + BAND_M
    plate = (m0 - 0.006, m0 - 0.001)                       # on the stator's base, the drive's still end
    tris = _band_x(0.008, top, plate[0], plate[1], PRINTED)
    for t in np.arange(8) * math.pi / 4.0 + math.pi / 8.0:
        at = (0.022 * math.cos(t), 0.022 * math.sin(t))
        tris += _cylinder_x(HEAD_R, plate[0] - HEAD_H, plate[0], SCREW, k=20, at=at)
        tris += _prism(_socket('hex', HEAD_R * 0.5) + at, plate[0] - HEAD_H - 0.0002, plate[0] - HEAD_H, DARK, 0)
    clamp = (g0 + (0.008 if holder == 'sleeve' else 0.003), g1 - 0.002)
    tris += _band_x(ring, ring + BAND_M, clamp[0], clamp[1], CARBON)
    ear = -(ring + BAND_M + 0.003)
    tris += _cylinder_x(0.0055, clamp[0], clamp[1], CARBON, at=(ear, 0.0))
    pinch = (0.5 * (clamp[0] + clamp[1]), ear)             # the clamp's screw across its ear
    tris += _prism(_circle(HEAD_R, 20, pinch), 0.0055, 0.0055 + HEAD_H, SCREW, axis=2)
    tris += _prism(_socket('hex', HEAD_R * 0.5) + pinch, 0.0055 + HEAD_H, 0.0055 + HEAD_H + 0.0002, DARK, axis=2)
    if holder == 'cradle':
        tris += _box(plate[0], clamp[1], 0.024, over, top, PRINTED)
        tris += _box(clamp[0], clamp[1], 0.024, ring + BAND_M - 0.001, over, PRINTED)
    elif holder == 'sleeve':
        tris += _band_x(over, top, plate[1], g0 + 0.001, PRINTED, gap=(math.radians(120), math.radians(240)))
        tris += _band_x(ring + BAND_M - 0.001, top, g0 + 0.001, g0 + 0.005, PRINTED)
        tris += _box(-0.022, 0.022, 0.022, over - 0.006, top, PRINTED)
    else:
        tris += _box(plate[0], clamp[1], top, 0.0, top, PRINTED)
    return tris, top


def scene(look, apart):
    """[(triangle, colour)]: the knee's servo, its holder, the thigh tube's spigot and its screws, `apart` or not."""
    import numpy as np
    from machine.body import skeleton
    holder, flange, screws, socket, sunk, threads = LOOKS[look]
    parts = [(float(c[0]), r, h) for j, _rides, c, _ax, r, h in skeleton.cans() if j == 'left_knee']
    mid = (min(x - h for x, _r, h in parts) + max(x + h for x, _r, h in parts)) / 2.0
    (mx, can, mh), (gx, ring, gh) = sorted(((x - mid, r, h) for x, r, h in parts), key=lambda p: -p[1])
    way = 1.0 if gx > mx else -1.0                         # laid with the gearbox toward +x, mirrored back after
    mx, gx = way * mx, way * gx
    radii = skeleton.HUNG['thigh'][1]
    tube = float(radii[-1] if isinstance(radii, tuple) else radii)
    tris = _cylinder_x(can, mx - mh, mx + mh, SERVO) + _cylinder_x(ring, gx - gh, gx + gh, GEAR)
    held, top = _holder(holder, can, ring, mx - mh, gx - gh, gx + gh)
    tris += held
    offset = math.pi / screws
    outline = _outline(flange, REACH_M, screws, offset)
    tris += _prism(outline, top - 0.002, top + 0.012, PRINTED)            # the boss: flat, no flange of its own
    face = top + 0.012
    turns = np.arange(screws) * 2.0 * math.pi / screws + offset
    holes = [CIRCLE_R * np.array([math.cos(t), math.sin(t)]) for t in turns]
    if threads == 'ring':
        tris += _prism(outline * 0.9, face - 0.0002, face + 0.0002, STEEL)   # the ring, the boss's shape
        tris += _prism(_circle(RING_R, 64), face + 0.0002, face + 0.0003, PRINTED)
    else:
        for at in holes:
            tris += _prism(_circle(0.003, 16, at), face - 0.0002, face + 0.0002, BRASS)   # an M4 insert, 6 mm
    for at in holes:
        tris += _prism(_circle(0.0021, 16, at), face + 0.0003, face + 0.0004, DARK)
    tris += _prism(_circle(BORE_R, 32), face + 0.0003, face + 0.0004, DARK)   # the cables' hole
    base = face + (OFF_M if apart else 0.0)
    tris += _prism(outline, base, base + FLANGE_M, PRINTED)
    deck = base + FLANGE_M
    for at in holes:
        if sunk:
            tris += _prism(_circle(HEAD_R * 1.12, 24, at), deck, deck + 0.0002, DARK)
        else:
            tris += _prism(_circle(SLEEVE_R, 20, at), deck, deck + 0.0002, STEEL)
            tris += _prism(_circle(0.0022, 16, at), deck + 0.0002, deck + 0.0003, DARK)
    tris += _prism(_circle(SPIGOT_R, 48), deck, deck + SPIGOT_M - TIP_M, PRINTED)
    tris += _prism(_circle(SPIGOT_R - 0.0012, 48), deck + SPIGOT_M - TIP_M, deck + SPIGOT_M, PRINTED)
    tris += _prism(_circle(BORE_R, 32), deck + SPIGOT_M, deck + SPIGOT_M + 0.0002, DARK)
    pull = deck + (PULL_M if apart else 0.0)
    tris += _prism(_circle(tube, 48), pull, pull + TUBE_M, CARBON)
    tris += _prism(_circle(tube - WALL_M, 48), pull + TUBE_M, pull + TUBE_M + 0.0002, DARK)
    head = deck - (HEAD_H if sunk else 0.0) + (LIFT_M if apart else 0.0)
    for at in holes:
        tris += _prism(_circle(SHANK_R, 16, at), head - SHANK_L, head, SCREW)
        tris += _prism(_circle(HEAD_R, 24, at), head, head + HEAD_H, SCREW)
        tris += _prism(_socket(socket, HEAD_R * 0.5) + at, head + HEAD_H, head + HEAD_H + 0.0002, DARK)
    if way < 0:
        tris = [(t * np.array([-1.0, 1.0, 1.0]), colour) for t, colour in tris]
    return tris


def draw(tris, yaw=35.0, pitch=28.0, px=760, half=0.16, centre=(0.0, 0.10, 0.0), cut=None):
    """The triangles drawn lit, a dot a pixel, nearest wins; past z = `cut` (if given) nothing drawn, a section."""
    import numpy as np
    from PIL import Image
    cy, sy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    cp, sp = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    R = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]) @ np.array([[cy, 0, -sy], [0, 1, 0], [sy, 0, cy]])
    light = np.array([0.4, 0.75, 0.55]) / np.linalg.norm([0.4, 0.75, 0.55])
    img, depth = np.full((px * px, 3), 18.0), np.full(px * px, -np.inf)
    pix = 2.0 * half / px
    for t, colour in tris:
        p = (t - np.asarray(centre)) @ R.T
        n = np.cross(p[1] - p[0], p[2] - p[0])
        area = float(np.linalg.norm(n))
        if area < 1e-14:
            continue
        shade = 0.35 + 0.65 * abs(float(n @ (R @ light))) / area
        count = int(min(120000, max(4, area / 2.0 / pix ** 2 * 4)))
        u = np.random.default_rng(0).random((count, 2))
        m = u.sum(axis=1) > 1
        u[m] = 1 - u[m]
        q = p[0] + u[:, :1] * (p[1] - p[0]) + u[:, 1:] * (p[2] - p[0])
        if cut is not None:
            q = q[(t[0] + u[:, :1] * (t[1] - t[0]) + u[:, 1:] * (t[2] - t[0]))[:, 2] <= cut]
        x = ((q[:, 0] / half + 1) / 2 * (px - 1)).astype(int)
        y = ((1 - (q[:, 1] / half + 1) / 2) * (px - 1)).astype(int)
        ok = (x >= 0) & (x < px) & (y >= 0) & (y < px)
        idx, z = y[ok] * px + x[ok], q[ok, 2]
        better = z > depth[idx]
        depth[idx[better]] = z[better]
        img[idx[better]] = np.array(colour, float) * shade
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8).reshape(px, px, 3))


def main(argv=None):
    from PIL import Image, ImageDraw
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--look', type=int, action='append')
    args = parser.parse_args(argv)
    for look in args.look or sorted(LOOKS):
        holder, flange, screws, socket, sunk, threads = LOOKS[look]
        apart = draw(scene(look, True))
        together = draw(scene(look, False), yaw=-40.0, half=0.115, centre=(0.0, 0.05, 0.0))
        sheet = Image.new('RGB', (apart.size[0] * 2, apart.size[1] + 52), (18, 18, 18))
        sheet.paste(apart, (0, 52))
        sheet.paste(together, (apart.size[0], 52))
        pen = ImageDraw.Draw(sheet)
        pen.text((12, 8), '%d: holder %s, flange %s 66 x 12 mm, %d x M4 %s%s into %s - apart | together, from the '
                 'stator' % (look, NAMES[holder], flange, screws, socket,
                             ' (heads sunk)' if sunk else ' via steel sleeves',
            'a tapped steel ring' if threads == 'ring' else 'brass inserts'), fill=(235, 235, 235))
        pen.text((12, 28), 'grey: printed CF polymer (holder, spigot 35.6 x 50 mm, bonded) | dark: carbon (tube 40 x 2,'
                 ' clamp) | bright: steel | magenta: servo', fill=(170, 170, 170))
        path = os.path.join(args.out, 'montering_%02d.png' % look)
        sheet.save(path)
        print(path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
