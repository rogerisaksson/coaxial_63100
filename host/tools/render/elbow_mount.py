"""The elbow's mount, exploded: the upper arm's and the forearm's tubes in line through the joint.

    python -X utf8 tools/render/elbow_mount.py OUT.png

The elbow's stack as `skeleton.cans` lays it (its board, its motor, its gearbox, the gearbox's end its turning
output ring) and the arm's tubes as the build's `[skeleton] hung` has them, the two tubes on one line through the
joint's middle. Each tube's seat is on a saddle necked to the least that carries the elbow's loads (`sized.neck`),
one width to print, the saddle the web of a clamp: the stator's over the stack onto its band round the motor's still
housing, the rotor's under it onto its band round the output ring - thin bands, each wall the least that grips
and hiding its two pinch screws, only its slit seen (`sized.band`). The board rides its motor's back under its
guard (`covered`), the stator's saddle on over the guard's rim, no wider than its seat. The cables
go down each tube's bore into a channel through its seat and its saddle, no hole in the carbon: the stator's - the
bus's trunk - out of its saddle's back end in a black pipe and on in a quarter circle of hose, round outside the
guard's lid, into a T-cross on the hose's holder on the guard's ring (`tees.fed`, `covered.across`; the guard's lid
off with the hoses in place), the board a branch off that cross, its bus's terminals beside the holder, the bus's
way in and out marked on the cross's body (`chain`); the rotor's to its saddle's back end. From the cross and from the
rotor's saddle a black pipe bends to aim across the elbow's axis (`tees.pipe`), the two facing each other with the
elbow at the middle of its travel, and a slim magenta hose hangs between their fittings, nothing turning: straight
there, bowing either way from it under its own stiffness and weight (`span`, `hung`). The rotor's channel's other
end, plugged, marks where the part mirrored for the body's other side takes its pipe. In the robot's colours,
every edge rounded. Where the two saddles meet is the elbow's stop: found by turning until what turns comes within
a millimetre of what stays, and drawn there.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import hung  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import tees  # noqa: E402

#: Drawn, not sized: the flange about a tube r round, near square and its longer way across the axis so the clamps
#: print without overhang (half-widths r + these along the axis and across it, power), its
#: screws (r + these off the axis), its thickness, the collar's wall and the spigot's and collar's lengths as shares
#: of the tube's width, the bore, the least gap kept turning, m.
OVAL_PAST, POWER, BOLTS_PAST, FLANGE_M, COLLAR_M = (0.008, 0.010), 4.0, (0.0015, 0.002), 0.009, 0.003
SPIGOT_SHARE, COLLAR_SHARE, BORE_R, CLEAR_M = 1.25, 0.375, 0.007 / 2.0, 0.001
#: The stator's saddle's height over the stack, the seats' height, the rotor's saddle's height under it and its air
#: over the stator's board - drawn in, its saddle ending near where the board begins, the servo's plugs clear behind;
#: an M4's head; m.
SADDLE_M, SEAT_M, ROTOR_M, ROTOR_AIR = 0.010, 0.006, 0.014, 0.002
#: The gearbox's still housing's length as a share of it, its turning output ring the rest.
OUTPUT_SHARE = 0.25
#: The clamps' slit, m, and where round the slits sit (the side the rotor's saddle never sweeps), rad.
SLIT_M, SLIT = 0.0025, 0.5 * math.pi
HEAD = (0.007 / 2.0, 0.004)
#: The robot's livery (`coaxial.graphics.livery`): gunmetal frames, light plates, carbon, bright bolts, a board's black
#: laminate, black honeycomb; the cable.
FRAME, PLATE, CARBON, BOLT = (112, 116, 126), (236, 240, 246), (64, 66, 68), (226, 230, 236)
BOARD, VOID, CABLE = (44, 46, 54), (8, 8, 10), (232, 112, 40)
#: The servo white, its gearbox a light grey.
SERVO, GEAR = (236, 240, 246), (190, 194, 202)
#: The servo's output ring, turning.
OUTPUT = (150, 154, 164)


def axis():
    """(way, mid): the sketch's x is `way` times the elbow drive's own (`skeleton.cans`) less `mid` - its gearbox at
    +x, its stack's middle at 0."""
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_elbow')]
    parts = [(float(c[2][0]), c[5]) for c in skeleton.cans() if c[0] == 'left_elbow']
    way = 1.0 if parts[names.index('gear')][0] > sum(x for x, _h in parts) / len(parts) else -1.0
    return way, 0.5 * (min(way * x - h for x, h in parts) + max(way * x + h for x, h in parts))


def _stack():
    """(still parts [(x0, x1, r)], gear (x0, x1, r), upper arm's tube r, forearm's tube r), the gearbox at +x."""
    from machine.body import drives, skeleton
    names = [p for p, *_ in drives.along('left_elbow')]
    parts = [(float(c[2][0]), c[4], c[5]) for c in skeleton.cans() if c[0] == 'left_elbow']
    way, mid = axis()
    laid = [(way * x - h - mid, way * x + h - mid, r) for x, r, h in parts]

    def radius(seg, end):
        r = skeleton.HUNG[seg][1]
        return float(r[end] if isinstance(r, tuple) else r)
    still = [p for p, n in zip(laid, names) if n != 'gear']
    return still, laid[names.index('gear')], radius('upper_arm', -1), radius('forearm', 1)


def _oval(r):
    import numpy as np
    t = np.arange(96) * 2.0 * math.pi / 96
    c, s = np.cos(t), np.sin(t)
    a, b = r + OVAL_PAST[0], r + OVAL_PAST[1]
    return np.c_[np.sign(c) * np.abs(c) ** (2 / POWER) * a, np.sign(s) * np.abs(s) ** (2 / POWER) * b]


def _rounded(outline, a, b, colour, axis=1, r=0.002, steps=4):
    """The prism of the 2-d `outline` from `a` to `b` along `axis`, its edges rounded `r` round: the mechanism soft."""
    import numpy as np
    pts = np.asarray(outline, float)
    a, b = min(a, b), max(a, b)
    r = min(r, 0.45 * (b - a))
    area = 0.5 * float(np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1]))
    t1, t2 = pts - np.roll(pts, 1, axis=0), np.roll(pts, -1, axis=0) - pts
    n = (np.c_[t1[:, 1], -t1[:, 0]] / np.linalg.norm(t1, axis=1, keepdims=True)
         + np.c_[t2[:, 1], -t2[:, 0]] / np.linalg.norm(t2, axis=1, keepdims=True))
    n *= (1.0 if area > 0 else -1.0) / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rings = ([(a + r * (1 - math.sin(th)), r * (1 - math.cos(th))) for th in np.linspace(0.5 * math.pi, 0.0, steps + 1)]
             + [(b - r * (1 - math.sin(th)), r * (1 - math.cos(th))) for th in np.linspace(0.0, 0.5 * math.pi, steps + 1)])

    def lay(flat, h):
        cols = [flat[:, 0], flat[:, 1]]
        cols.insert(axis, np.full(len(flat), h))
        return np.c_[cols[0], cols[1], cols[2]]
    layers = [lay(pts - inset * n, h) for h, inset in rings]
    k, tris = len(pts), []
    for lo, hi in zip(layers[:-1], layers[1:]):
        for i in range(k):
            j = (i + 1) % k
            tris += [(np.array((lo[i], lo[j], hi[j])), colour), (np.array((lo[i], hi[j], hi[i])), colour)]
    for cap in (layers[0], layers[-1]):
        mid = cap.mean(axis=0)
        tris += [(np.array((mid, cap[i], cap[(i + 1) % k])), colour) for i in range(k)]
    return tris


def _drum(r, x0, x1, colour, at=(0.0, 0.0), soft=0.0015):
    """A drum along x with its rims rounded."""
    return _rounded(m._circle(r, 48, at), x0, x1, colour, axis=0, r=soft)


def _limb(face, way, tube, dx, socket, apart):
    """The seat's face dressed at `face` on the joint's middle - a brass insert's mouth round each screw's hole - and
    the tube's flange, spigot, collar, tube and screws off it toward `way`, the bore `dx` off the axis: (dressing,
    limb)."""
    import numpy as np
    outline, at0, bore = _oval(tube), (0.0, 0.0), (dx, 0.0)
    bx, bz = tube + BOLTS_PAST[0], tube + BOLTS_PAST[1]
    holes = [np.array(p) for p in ((bx, bz), (-bx, bz), (-bx, -bz), (bx, -bz))]
    e, rs = way * 0.0002, tube - 0.001 - 0.0002
    dress = []
    for at in holes:
        dress += m._prism(m._circle(0.003, 16, at), face - 0.5 * e, face + 0.5 * e, m.BRASS)
        dress += m._prism(m._circle(0.002, 16, at), face - e, face + e, VOID)
    dress += m._prism(m._circle(BORE_R, 32, bore), face - e, face + e, VOID)
    base = face + way * (0.03 if apart else 0.0)
    deck = base + way * FLANGE_M
    limb = _rounded(outline, base, deck, FRAME, r=0.0025)
    length = SPIGOT_SHARE * 2.0 * tube
    limb += m._prism(m._circle(rs, 40, at0), deck, deck + way * (length - 0.002), FRAME)
    limb += m._prism(m._circle(rs - 0.001, 40, at0), deck + way * (length - 0.002), deck + way * length, FRAME)
    lo, hi = sorted((deck, deck + way * COLLAR_SHARE * 2.0 * tube))
    limb += km._ring_y(tube + 0.0002, tube + 0.0002 + COLLAR_M, lo, hi, 0.0, FRAME)
    pull = deck + way * (length + 0.012 if apart else 0.0)
    end = pull + way * 0.06
    limb += m._prism(m._circle(tube, 40, at0), pull, end, CARBON)
    limb += m._prism(m._circle(tube - 0.001, 40, at0), end, end + e, VOID)
    head = deck + way * (0.02 if apart else 0.0)
    for at in holes:
        limb += m._prism(m._circle(0.002, 16, at), head - way * (FLANGE_M + 0.008), head, BOLT)
        limb += m._prism(m._circle(HEAD[0], 24, at), head, head + way * HEAD[1], BOLT)
        limb += m._prism(m._socket(socket, HEAD[0] * 0.5) + at, head + way * HEAD[1], head + way * (HEAD[1] + 0.0002),
                         VOID)
    return dress, limb


def _mark(at, normal):
    """A symbolic outlet: the channel's other end, plugged, where the same part mirrored for the body's other side
    takes its pipe."""
    import numpy as np
    at, normal = np.asarray(at, float), np.asarray(normal, float)
    return (km._rod(at, at + 0.0006 * normal, tees.PIPE[1] + 0.0018, tees.PIPE_INK)
            + km._rod(at + 0.0006 * normal, at + 0.0008 * normal, tees.PIPE[1] * 0.7, FRAME))


def _band(inner, outer, x0, x1, slit, colour=FRAME, r=0.002, k=48, steps=3):
    """A thick clamp band round the stack between radii, from x0 to x1, its edges rounded `r`: only its slit at
    `slit` (y = r cos) shows - its pinch screws lie in its wall across the slit, out of sight."""
    import numpy as np
    gap = SLIT_M / inner
    turns = np.linspace(slit + 0.5 * gap, slit + 2.0 * math.pi - 0.5 * gap, k)

    def outline(inset):
        rim = [((outer - inset) * math.cos(a), (outer - inset) * math.sin(a)) for a in turns]
        bore = [((inner + inset) * math.cos(a), (inner + inset) * math.sin(a)) for a in turns[::-1]]
        return np.array(rim + bore)
    rings = ([(x0 + r * (1 - math.sin(th)), r * (1 - math.cos(th))) for th in np.linspace(0.5 * math.pi, 0.0, steps + 1)]
             + [(x1 - r * (1 - math.sin(th)), r * (1 - math.cos(th))) for th in np.linspace(0.0, 0.5 * math.pi, steps + 1)])
    layers = [np.c_[np.full(2 * k, x), outline(inset)] for x, inset in rings]
    tris, n = [], 2 * k
    for lo, hi in zip(layers[:-1], layers[1:]):
        for i in range(n):
            j = (i + 1) % n
            tris += [(np.array((lo[i], lo[j], hi[j])), colour), (np.array((lo[i], hi[j], hi[i])), colour)]
    for cap in (layers[0], layers[-1]):
        for j in range(k - 1):
            q = (cap[j], cap[j + 1], cap[n - 2 - j], cap[n - 1 - j])
            tris += [(np.array((q[0], q[1], q[2])), colour), (np.array((q[0], q[2], q[3])), colour)]
    return tris


def _laid():
    """The elbow's layout, m: (still parts, gear, upper arm's tube r, forearm's, its largest can, motor, board, the
    seat's half widths (narrow, reach), where the output ring begins, the stator band's ends and outside, the board's
    cover (`covered.cover`'s) and its guard's (face, end, outside), the saddles' height over the stack, the rotor's
    band's start and outside, the elbow's swing load)."""
    from tools.render import covered as cv
    from tools.render import knee_loads as kl
    from tools.render import sized
    still_parts, (g0, g1, ring), arm_in, arm_out = _stack()
    can = max(r for _a, _b, r in still_parts)
    motor = next(p for p in still_parts if p[2] < can)
    board = next(p for p in still_parts if p[2] >= can)
    split = g0 + OUTPUT_SHARE * (g1 - g0)                # the gearbox's still housing, then its turning output ring
    back, x1 = motor[0] + 0.002, motor[1] - 0.002        # one band round the stator, as the rotor's
    swing = kl.loads('elbow')['swing']
    band = motor[2] + 1e-3 * sized.band(swing, 1e3 * motor[2], 1e3 * (x1 - back))
    cover = (board[0] - ROTOR_AIR - cv.WALL_M, motor[0], board[2], band)    # the board's, sat on the band
    guard = cv.span(*cover)
    over = max(can + 0.002, guard[2] + cv.FIT_M)         # the saddles clear of the cover
    first = split + 0.0025                               # the rotor's band clear of the still housing
    rotor = ring + 1e-3 * sized.band(swing, 1e3 * ring, 1e3 * (g1 - first))
    seat = (arm_in + OVAL_PAST[1], arm_in + OVAL_PAST[0])
    return (still_parts, (g0, g1, ring), arm_in, arm_out, can, motor, board, seat, split, (back, x1, band), cover,
            guard, over, (first, rotor), swing)


def span():
    """The elbow's hose's span in the elbow's frame, the arm straight (`hung`): from the fitting on the stator's
    guard to the rotor's saddle's, each aimed across the axis; the board's guard and the two bands as cylinders
    CLEAR_M grown; its travel from the drive's own end to the elbow's stop."""
    return _fitted()[0]


@functools.lru_cache(maxsize=None)
def _fitted():
    """(the elbow's hose's span, the stator's pipes' tris, the rotor's, the guard's marks [(kind, (y, z) shares of
    its bore)], its outlet's pad (y, z)): the rotor's pipe out of its saddle's back end under the stack, aimed up
    at the axis; the stator's a T-cross out of its guard's face (`covered.across`), one arm aimed at the axis too,
    the trunk into the other out of the stator's saddle's back end, the middle of its height (`tees.fed`)."""
    from machine.body import drives
    from tools.render import covered as cv
    lay = _laid()
    g1, reach, (back, x1, band), cover, guard, over, (first, rotor) = (lay[1][1], lay[7][1], lay[9], lay[10], lay[11],
                                                                       lay[12], lay[13])
    face, low, travel = guard[0], over + ROTOR_AIR, (drives.TRAVEL['elbow'][0], stop())
    pipe_s, fit_s = tees.outlet((-reach, -(low + ROTOR_M / 2.0), 0.0), 1.0, -reach - face + tees.STUB_M)
    pad, marks = cv.across(math.pi, travel, cover[2], cover[3])   # across from the rotor's, under the stack
    off = math.hypot(*pad)
    aim = (-pad[0] / off, -pad[1] / off)
    pipe_t, fit_t, _line = tees.fed((face,) + pad, aim, (-reach, over + 0.5 * (SADDLE_M + SEAT_M), 0.0))
    drums = [('still', guard[0], guard[1], guard[2]), ('still', back, x1, band), ('turning', first, g1, rotor)]
    span_ = {'name': 'elbow', 'kind': 'bus', 'travel': travel,
             'from': ('still', tuple(fit_t), (0.0,) + aim), 'to': ('turning', tuple(fit_s), (0.0, 1.0, 0.0)),
             'solids': [(side, (0.5 * (a + b), 0.0, 0.0), (1.0, 0.0, 0.0), r + CLEAR_M, 0.5 * (b - a))
                        for side, a, b, r in drums]}
    return span_, pipe_t, pipe_s, marks, pad


def bodies(apart=False, flex=0.0, socket='torx', hosed=True):
    """(parts, cables, still, turning) of the elbow, the forearm's side turned by `flex` degrees; its hose hung
    unless not `hosed`."""
    from tools.render import boards, clamped as cl, covered as cv
    from tools.render import sized
    (still_parts, (g0, g1, ring), arm_in, arm_out, can, motor, board, (narrow, reach), split, (back, x1, band), cover,
     (face, end, outside), over, (first, rotor), swing) = _laid()
    e = arm_in - 0.0012 - BORE_R - 0.003                 # the bore off the axis along it: the swing's neutral axis
    seat = (-narrow, narrow)
    fitted = _fitted() if hosed else None                # the bus on out through the guard's face, in a pipe
    turn = (cv.turned(fitted[4]) if fitted               # its bus's terminals beside the holder, its lid off
            else cv.HEATSINK - boards.band(round(board[1] - board[0], 6)))
    stack = []
    for a, b, r in still_parts:
        brd = r >= can and boards.placed(a, b, (0.0, 0.0), b, turn)   # the board itself, its bare side on the motor
        stack += brd or _drum(r, a, b, BOARD if r >= can else SERVO)
        if brd and fitted:                               # the phases' leads, the bus's in under the saddle and on
            stack += boards.leads(a, b, (0.0, 0.0), b, turn, band) + boards.wired(
                a, b, (0.0, 0.0), b, turn, 0.0, fitted[4], b - cover[0], True)
    stack += _drum(ring, g0, split, GEAR)
    top = over + SADDLE_M
    face_t = top + SEAT_M
    up = (0.0, 0.03, 0.0) if apart else (0.0, 0.0, 0.0)

    def clamp(r, x0, x2, y, span, past, start=None):
        """A slit band round a drum `r` from x0 to x2, its screws hid, and its saddle to the seat's edge at `y`,
        necked to the least that holds over its width `span`, round the band from `start` (else x0) and on `past`
        clear of what it passes there."""
        grip = (swing, 1e3 * r, 1e3 * (x2 - x0))
        wall, waist = 1e-3 * sized.band(*grip), 1e-3 * sized.neck(1e3 * (span[1] - span[0]), 'elbow')
        tris = _band(r, r + wall, x0, x2, SLIT)
        for a, b, round_ in [(x0 if start is None else start, x2, r + wall - 0.001)] + past:
            tris += _rounded(cl.filleted(cl.necked((0.0, 0.0), round_, y, seat, waist)), a, b, FRAME, axis=0, r=0.002)
        return tris
    holder = clamp(motor[2], back, x1, top, (-reach, reach), [(-reach, end, outside + cv.FIT_M),   # over the cover,
                                                            (x1, reach, rotor + ROTOR_AIR)], end)   # the rotor's band
    holder += _rounded(_oval(arm_in), top - 0.002, face_t, PLATE, r=0.002)
    stack += (cv.cover(*cover, outlet=fitted[4], window=boards.window(turn))
              + cv.lid(cover[0], cover[2], cover[3], fitted[3], fitted[4]) + fitted[1] if fitted else cv.cover(*cover))
    dress, limb = _limb(face_t, 1.0, arm_in, -e, socket, apart)
    holder += dress + limb
    low = over + ROTOR_AIR                               # the rotor's saddle under the stack, the servo's own
    bottom = low + ROTOR_M                               # plugs at its board clear of it
    face_s = -(bottom + SEAT_M)
    output = _drum(ring, split, g1, OUTPUT)                                    # the output ring, turning
    tail = -reach                                        # the rotor's saddle from its seat's back to its clamp
    turning = clamp(ring, first, g1, -bottom, (tail, g1), [(tail, first, low)])   # under the stack, clear of it
    turning += _rounded(_oval(arm_out), -(bottom - 0.002), face_s, PLATE, r=0.002)
    pipe_s = tees.outlet((tail, -(low + ROTOR_M / 2.0), 0.0), 1.0, tail - face + tees.STUB_M)[0]   # past the cover
    turning += pipe_s + _mark((g1, -(low + ROTOR_M / 2.0), 0.0), (1.0, 0.0, 0.0))
    dress, limb = _limb(face_s, -1.0, arm_out, -e, socket, apart)
    turning += dress + limb
    cables = tees.hose(hung.path(fitted[0], flex)) if fitted else []   # hung from its fittings, its own lie
    run = FLANGE_M + 0.07
    cables += km._wire([(-e, face_t + run, 0.0), (-e, face_t, 0.0)], 0.002, CABLE)
    cables += km._turn(km._wire([(-e, face_s, 0.0), (-e, face_s - run, 0.0)], 0.002, CABLE), flex)
    still, turned = stack + holder, turning              # the output ring rides its bearing
    if apart:
        down = (0.0, -0.03, 0.0)
        return (stack + output + [(tri + up, c) for tri, c in holder]
                + [(tri + down, c) for tri, c in turning]), [], still, turned
    return still + km._turn(output + turned, flex), cables, still, turned


def stop(step=2.0):
    """The elbow's stop, deg: the last turn from straight at which what turns stays CLEAR_M off what stays."""
    import numpy as np
    from machine.body import drives
    _p, _c, still, turning = bodies(hosed=False)
    h = 0.006
    a, pts = km._points(still), km._points(turning)
    keys = km._key(np.floor(a / h).astype(np.int64))
    order = np.argsort(keys)
    last = 0.0
    for deg in np.arange(0.0, drives.TRAVEL['elbow'][1] + 0.1, step):
        if km._least(keys[order], a[order], pts @ km._rx(deg).T, h) < CLEAR_M:
            break
        last = float(deg)
    return last


def main(argv=None):
    from PIL import Image, ImageDraw
    from machine.body import drives
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    limit = stop()
    parts, _c, _s, _t = bodies(True)
    apart = m.draw(parts, px=900, half=0.19, centre=(0.0, 0.0, 0.0))
    parts, cables, _s, _t = bodies()
    front = m.draw(parts + cables, px=600, half=0.13, centre=(0.005, 0.0, 0.0))
    behind = m.draw(parts + cables, yaw=-50.0, pitch=18.0, px=600, half=0.13, centre=(-0.01, 0.0, 0.0))
    parts, cables, _s, _t = bodies(flex=limit)
    bent = m.draw(parts + cables, yaw=-70.0, pitch=18.0, px=600, half=0.13, centre=(-0.01, 0.02, 0.0))
    sheet = Image.new('RGB', (2700, 940), (18, 18, 18))
    sheet.paste(apart, (0, 40))
    for i, img in enumerate((front, behind, bent)):
        sheet.paste(img, (900 + 600 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'elbow, tubes in line: apart | together | from behind | bent, from behind, to its'
                               ' stop %.0f deg (the spec\'s %.0f)' % (limit, drives.TRAVEL['elbow'][1]),
                               fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out, 'stop %.0f deg' % limit)
    return 0


if __name__ == '__main__':
    sys.exit(main())
