"""The hip, sketched: its three drives in printed clamps, the cradle on its pin, the thigh's neck.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/hip_clamped.py OUT.png

The hip as `hip_laid` lays it, a link a dictionary of parts: the pelvis's (the yaw drive in its slit band), the
yaw's (the yoke - the crown under the yaw drive's hub, its two cheeks back past the pinion - and the block screwed
on it, clamping the roll drive's motor over the pin's two needle bearings: `hip_frame`), the roll's (the cradle: a
slit band round the pitch drive's motor, a saddle behind it, the steel plate with its toothed sector and its pin)
and the thigh's (the arm - its head on the pitch drive's hub, a skirt round the output ring -, the knuckle screwed
on its foot - the jog in under the cradle -, the tube on its spigot: `hip_frame`), drawn upright and leant as the
thigh's tube leans (`knee_clamped.tilt`). Each board rides its
motor's back under its guard, its lid off beside its bus's terminals, its three phase leads on their lugs (`boards`,
`covered`). The bus crosses the yaw in a hose over the yaw drive's lid, from its guard's holder to a riser on the
block, and the pitch in a hose from the pitch drive's guard to a riser on the thigh's jog - each fitting aimed at
its joint's axis, across it from the other at the middle of its travel (`hung`); the roll it crosses inside the
hollow pin, twisting, since nothing but the pin is free on the roll's axis. The trunk between: a T-cross on the yaw
drive's holder (`tees`); a cross at the riser's foot, a branch to the roll's guard, an arm aside to the hip's outer
side and the hose on from it laid still - a helix down round outside the roll's guard to where its tangent runs
through the pin's axis, straight on into the nipple on the pin's housing's cap, clipped to the block and the
guard's ring on its way (`tees.clipped`); out of the pin's bore through the cradle's saddle, hid, into the pitch
drive's guard's wall - that board the chain's junction, the trunk in and on at its terminals, the cross's arm
having no room between saddle and guard. `posed` turns the links about the hip's middle - the yaw's about y, the
roll's about z, the thigh's about x; `hip_checked` holds them to the poses asked.
"""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import boards  # noqa: E402
from tools.render import clamped as cl  # noqa: E402
from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import hip_laid as hl  # noqa: E402
from tools.render import hung  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import sized as ks  # noqa: E402
from tools.render import tees  # noqa: E402

#: The yaw's travel the hose is hung through, deg; its riser's fitting off the yaw's axis and the thigh's off the
#: pitch's, m; the thigh's riser's z behind the hip's axis, m.
YAWED, RISER_M, THIGH_M, RISER_Z = (-30.0, 30.0), 0.091, 0.086, -0.006
#: How far the yaw's hose's two pipes lean back, rad: their mouths rise, the hose an arch over the lid, never a
#: straight slack one that may bow down onto it.
LEANT = math.radians(15.0)
#: The trunk's hose in from the pelvis as far as it is drawn, m; the parts a look at the harness alone shows, by
#: their names' ends.
TRUNK_M, HARNESS = 0.06, ('leads', 'pipe', 'riser', 'trunk', 'guard', 'clips')
#: The nipple on the pin's housing's cap (`hip_frame.CAP`), where the bus turns into the pin's bore: its reach and
#: radius, m.
NIP_M = (0.007, 0.004)


def _local(stack, flip):
    """`stack` {part: (from, to, r)} along x with its gearbox at +x: as it lies, or mirrored (`flip`)."""
    return {p: ((-b, -a, r) if flip else (a, b, r)) for p, (a, b, r) in stack.items()}


def _drive(stack, wall, entry, far=None, travel=None, leave=0.0, come=None, through=False):
    """{'still', 'ring', 'band', 'guard', 'lid', 'leads' and, with a hose, 'pipe', 'fitting', 'drum'} of a drive drawn
    along x, its gearbox at +x: its board (the model where it lies beside the tree) on its motor's back, the motor,
    the gearbox's still housing; its turning output ring; a slit band `wall` thick round its motor; its guard's
    ring, its lid, the phases' leads and the bus's - in through the guard's wall at the bearing `entry`, rad, onto
    the board's terminals and out by the holder's pad across the axis from the hose's far fitting at the bearing
    `far` (the joint at the middle of `travel`), a T-cross on the pad (`tees`), one arm aimed at the axis, the
    other `come`'s way ('inlet' its end) - the board a branch off the trunk, all its leads through the pad; or -
    `through` - the trunk itself in through the wall at `entry` and on by a pipe on the pad, the board the chain's
    junction; or, with no hose, all through the wall at `entry`. The chain's marks where the bus goes in and out:
    on a cross's body, on the lid hard by the wall where the bus comes through that (`covered.beside`)."""
    (b0, b1, r_b), (m0, m1, can), (g0, g1, ring) = stack['board'], stack['motor'], stack['gear']
    split, band_r, face = g0 + em.OUTPUT_SHARE * (g1 - g0), can + wall, b0 - kc.AIR_M - cv.WALL_M
    if far is None:
        off = cv.edge(r_b, band_r)
        pad, at = None, (off * math.cos(leave), off * math.sin(leave))
        marks: tuple = cv.beside(entry, r_b, band_r, ('in', 'out'))
    else:
        pad, marks = cv.across(far, travel, r_b, band_r, entry if through else None)
        at = pad
    turn = cv.turned(at)
    laid = (b0, b1, (0.0, 0.0), b1, turn)
    out = {'still': ((boards.placed(*laid) or em._drum(r_b, b0, b1, em.BOARD)) + em._drum(can, m0, m1, em.SERVO)
                     + em._drum(ring, g0, split, em.GEAR)),
           'ring': em._drum(ring, split, g1, em.OUTPUT),
           'band': cl.turned([(m0 + 0.002, can, band_r), (m1 - 0.002, can, band_r)], 0.5 * math.pi),
           'leads': boards.leads(*laid, band_r) + boards.wired(*laid, entry, pad, b1 - face, not through),
           'guard': cv.cover(face, m0, r_b, band_r, outlet=pad, window=boards.window(turn), lidded=True),
           'lid': cv.lid(face, r_b, band_r, marks, pad, turn=turn)}
    if pad:
        reach = math.hypot(*pad)
        aim, at = (-pad[0] / reach, -pad[1] / reach), (face,) + pad
        cross = tees.pipe(at, aim, tees.STUB_M, 'out') if through else tees.tee(at, aim, tees.STUB_M, come)
        (out['pipe'], out['fitting']), out['at'] = cross, at
        out['inlet'] = tees.mouth(at, aim, tees.STUB_M, come)
        out['drum'] = cv.span(face, m0, r_b, band_r)
    return out


def _leant(pipe, fitting, at, toward, tilt):
    """A pipe (`tees.pipe`: its triangles, its fitting's end) leant `tilt` rad back about its foot `at`, its
    mouth - aimed `toward` (y, z) across the axis - rising that much off the board: (triangles, the fitting's end,
    the mouth's way)."""
    import numpy as np
    way, at = np.array([0.0, toward[0], toward[1]]), np.asarray(at, float)
    u = np.cross(way, (-1.0, 0.0, 0.0))
    k = np.array([[0.0, -u[2], u[1]], [u[2], 0.0, -u[0]], [-u[1], u[0], 0.0]])
    turn = np.eye(3) + math.sin(tilt) * k + (1.0 - math.cos(tilt)) * (k @ k)
    return [((tri - at) @ turn.T + at, colour) for tri, colour in pipe], (fitting - at) @ turn.T + at, turn @ way


def _spiral(off, reach, z1, boss):
    """(k, 3): the trunk's hose laid still from its arm's fitting at `off`, leaving along +x: a helix `reach` round
    the roll's axis down the hip's outer side to `z1`, to where its tangent runs through the pin's axis, and straight
    on along that to the end of the nipple on the pin's housing's cap, `boss` round - the line's last point."""
    import numpy as np
    t0, t1, z0 = math.atan2(off[1] - hl.ROLL_AT[1], off[0]), -math.asin(reach / hl.ROLL_AT[1]), float(off[2])

    def round_(s):                                       # on the helix a share `s` of its way: where, which way
        t, dz = t0 + s * (t1 - t0), (z1 - z0) * 6.0 * s * (1.0 - s)
        at = (reach * math.cos(t), hl.ROLL_AT[1] + reach * math.sin(t), z0 + (z1 - z0) * s * s * (3.0 - 2.0 * s))
        way = np.array([-reach * math.sin(t) * (t1 - t0), reach * math.cos(t) * (t1 - t0), dz])
        return np.array(at), way / np.linalg.norm(way)
    last, into = round_(1.0)
    nipple = np.array([0.0, 0.0, z1]) - (boss + NIP_M[0]) * into
    return tees.run([(off, np.array([1.0, 0.0, 0.0])), round_(1.0 / 3.0), round_(2.0 / 3.0), (last, into), (nipple, into)])


def _built():
    """(links {'pelvis', 'yaw', 'roll', 'thigh': {part: [(tri, colour)]}} in the hip's frame, every joint at 0,
    hoses {'yaw', 'pitch': (span as `hung` takes it, the turn laying its frame in the hip's)})."""
    import numpy as np
    stacks, walls, lean = hl.stacks(), hl.walls(), kc.tilt(kl.stated())
    out = {'pelvis': {}, 'yaw': {}, 'roll': {}, 'thigh': {}}
    # the yaw drive: on the pelvis, its ring the yaw's; its hose over its lid to the riser on the block
    stack = _local(stacks['yaw'], True)
    wall = 1e-3 * ks.band(hl.loads()['moment'][1], 1e3 * stack['motor'][2],
                          1e3 * (stack['motor'][1] - stack['motor'][0] - 0.004))
    yaw = _drive(stack, wall, math.pi, far=-0.5 * math.pi, travel=YAWED, come=(-1.0, 0.0))
    yaw['pipe'], mouth_y, way_y = _leant(yaw['pipe'], yaw['fitting'], yaw['at'], (0.0, -1.0), LEANT)
    inlet = _leant([], yaw['inlet'], yaw['at'], (0.0, -1.0), LEANT)[1]
    yaw['trunk'] = tees.hose([inlet + (0.0, -TRUNK_M, 0.0), inlet])                 # in from the pelvis's hub
    out['pelvis'].update({'yaw ' + k: hf._put(yaw[k], hf._along('yaw'))
                          for k in ('still', 'band', 'guard', 'lid', 'leads', 'pipe', 'trunk')})
    out['yaw']['yaw ring'] = hf._put(yaw['ring'], hf._along('yaw'))
    low = min(a for a, _b, _r in stacks['yaw'].values())                      # the hub's face
    # the roll drive in the yaw's block, over the pin's bearings; its bus in from the riser, out down to the pin
    stack, at = stacks['roll'], (hl.ROLL_AT[0], hl.ROLL_AT[1], 0.0)
    roll = _drive(stack, walls['roll'], 0.0, leave=math.pi)
    out['yaw'].update({'roll ' + k: hf._put(roll[k], hf._along('roll'), at)
                       for k in ('still', 'ring', 'guard', 'lid', 'leads')})
    out['yaw'].update(hf.yoke(low, stack['gear'][1]), **hf.block(stack, walls['roll'], low))
    block, outside = out['yaw']['block'], stack['motor'][2] + walls['roll']
    (_f0, _f1), (b0, _b1) = hl.needles()
    boss, cap = hl.NEEDLE[0] + hl.HOUSED_M, hf.CAP * hl.HOUSED_M
    small, big, tip = hl.teeth()
    front, back = hl.plate()
    foot = (-(hl.ROLL_AT[1] + outside), 0.0, -(RISER_M + tees.PIPE[0] + tees.PIPE_LEG))    # in the yaw drive's frame
    riser, fit_r, way_r = _leant(*tees.pipe(foot, (0.0, 1.0), foot[0] - tees.PIPE[0] - float(yaw['fitting'][0])), foot,
                                 (0.0, 1.0), LEANT)
    top = hf._along('yaw') @ np.array(foot)              # the riser's foot on the block: the trunk's cross there,
    trunk = km._rod(top, np.array([top[0], top[1], stack['board'][1]]), tees.PIPE[1], tees.PIPE_INK)   # a branch to
    aside, off = tees.pipe(foot, (1.0, 0.0), 0.0)        # the roll's guard, an arm aside - the hip's outer side - for
    reach = cv.span(0.0, 0.0, stack['board'][2], outside)[2] + tees.HOSE_R + kc.AIR_M     # the trunk on (`_spiral`)
    line = _spiral(hf._along('yaw') @ off, reach, b0 - 0.5 * cap, boss)
    into = (line[-1] - line[-2]) / np.linalg.norm(line[-1] - line[-2])
    trunk += hf._put(aside, hf._along('yaw')) + km._rod(line[-1] + NIP_M[0] * into, line[-1], NIP_M[1], tees.PIPE_INK)
    trunk += tees.hose(line)                             # held on the block and the roll's guard's ring, not its lid
    held = tees.clipped(line, block + hf._put(roll['guard'], hf._along('roll'), at))
    out['yaw'].update({'riser': hf._put(riser, hf._along('yaw')), 'trunk': trunk, 'trunk clips': held,
                       'pinion': hf._prism(m._circle(small + tip, 30, hl.ROLL_AT), back, front, hf.STEEL, 2)})
    # the cradle: the pitch drive in its band, the saddle, the plate and its pin; its hose to the thigh's riser
    stack = stacks['pitch']
    mouth = (0.0, -THIGH_M * math.cos(lean) + RISER_Z * math.sin(lean), THIGH_M * math.sin(lean) + RISER_Z * math.cos(lean))
    pitch = _drive(stack, walls['cradle'], -0.5 * math.pi, far=math.atan2(mouth[2], mouth[1]),
                   travel=(hl.TURNED[0] - 1.0, hl.TURNED[1] + 1.0), through=True)   # the trunk out of the pin's bore
    p0, p1, can = stack['motor']                                                    # through the saddle, hid
    rim = can + walls['cradle'] - 0.001
    ys = np.linspace(hl.SADDLE_Y, -hl.SADDLE_Y, 25)
    saddle = np.array([(-hl.SADDLE_Y, front), (hl.SADDLE_Y, front)] + [(y, -math.sqrt(rim * rim - y * y)) for y in ys])
    turns = np.radians(np.linspace(38.0, 142.0, 27))
    sector = np.array([((big + tip) * math.cos(t), (big + tip) * math.sin(t)) for t in turns]
                      + [((big - 0.010) * math.cos(t), (big - 0.010) * math.sin(t)) for t in turns[::-1]])
    plate = (hf._prism(hf._box((-hl.STEM_M, hl.STEM_M), (-hl.SADDLE_Y, hl.SADDLE_Y)), back, front, hf.STEEL, 2)
             + hf._prism(sector, back, front, hf.STEEL, 2))
    out['roll'].update({'pitch ' + k: pitch[k] for k in ('still', 'guard', 'lid', 'leads', 'pipe')})
    out['roll'].update({'cradle band': km._turn(pitch['band'], 90.0), 'plate': plate,
                        'saddle': hf._prism(saddle, p0 + 0.002, p1 - 0.002, em.FRAME, 0),
                        'kingpin': hf._prism(m._circle(hl.KINGPIN[0], 32), b0, back, hf.STEEL, 2)})
    # the thigh: its arm on the pitch drive's hub, its knuckle, the tube, the hose's riser - upright, then leant
    x_f = float(pitch['fitting'][0])
    upright = dict(hf.thigh(stack, (x_f, THIGH_M, RISER_Z)), **{'pitch ring': pitch['ring']})
    out['thigh'] = {k: cl.tilted(v, -lean, (0.0, 0.0)) for k, v in upright.items()}

    def drum(side, span):
        return (side, (0.5 * (span[0] + span[1]), 0.0, 0.0), (1.0, 0.0, 0.0), span[2] + em.CLEAR_M, 0.5 * (span[1] - span[0]))
    aim = np.array(pitch['fitting'], float) * (0.0, 1.0, 1.0)
    aim = -aim / float(np.linalg.norm(aim))
    hoses = {'yaw': ({'name': 'hip yaw', 'kind': 'bus', 'travel': YAWED,
                      'from': ('still', tuple(mouth_y), tuple(way_y)),
                      'to': ('turning', tuple(fit_r), tuple(way_r)), 'solids': [drum('still', yaw['drum'])]},
                     hf._along('yaw')),
             'pitch': ({'name': 'hip', 'kind': 'bus', 'travel': (hl.TURNED[0] - 1.0, hl.TURNED[1] + 1.0),
                        'from': ('still', tuple(pitch['fitting']), tuple(aim)),
                        'to': ('turning', (x_f,) + mouth[1:], (0.0, math.cos(lean), -math.sin(lean))),
                        'solids': [drum('still', pitch['drum'])]}, np.eye(3))}
    return out, hoses


_HIP = []


def links():
    """{'pelvis', 'yaw', 'roll', 'thigh': {part: [(tri, colour)]}} in the hip's frame, every joint at 0."""
    if not _HIP:
        _HIP.append(_built())
    return _HIP[0][0]


def hoses():
    """{'yaw', 'pitch': (span, turn)}: the hip's two hoses as `hung` takes them, each in its drive's frame - the
    drive's axis its x - and the turn laying that frame in the hip's."""
    links()
    return _HIP[0][1]


def turn(yaw=0.0, roll=0.0, pitch=0.0):
    """(yaw's, roll's, thigh's) turns, 3 by 3 each, at the hip's angles, deg: about y, then z, then x."""
    import numpy as np
    (cy, sy), (cr, sr), (cp, sp) = ((math.cos(math.radians(a)), math.sin(math.radians(a))) for a in (yaw, roll, pitch))
    about_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    about_z = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    about_x = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])
    return about_y, about_y @ about_z, about_y @ about_z @ about_x


def slung(yaw=0.0, roll=0.0, pitch=0.0):
    """[(tri, colour)]: the hip's two hoses hung (`hung`), the hip at its angles, deg, in the hip's frame."""
    (over, laid), (under, _same) = hoses()['yaw'], hoses()['pitch']
    return (hf._put(tees.hose(hung.path(over, -yaw)), laid)                     # the yaw's about the drive's -x
            + hf._put(tees.hose(hung.path(under, pitch)), turn(yaw, roll, pitch)[1]))


def posed(yaw=0.0, roll=0.0, pitch=0.0, hosed=True, only=None):
    """[(tri, colour)] of the hip at its angles, deg: its links - the parts whose names end as `only` tells, where
    told - and, unless not `hosed`, its two hoses hung (`slung`)."""
    import numpy as np
    turns = dict(zip(('yaw', 'roll', 'thigh'), turn(yaw, roll, pitch)))
    tris = [t for name, parts in links().items() for key, part in parts.items() if not only or key.endswith(only)
            for t in hf._put(part, turns.get(name, np.eye(3)))]
    return tris + (slung(yaw, roll, pitch) if hosed else [])


def main(argv=None):
    from PIL import Image, ImageDraw
    from machine.body import mechatronics
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    fold, back = hl.TURNED
    looks = [((0.0, 0.0, 0.0), 0.0, 5.0), ((0.0, 0.0, 0.0), 90.0, 5.0), ((0.0, 0.0, 0.0), 180.0, 5.0),
             ((0.0, 0.0, 0.0), 150.0, 15.0), ((0.0, 0.0, 0.0), 40.0, 25.0), ((0.0, 0.0, fold), 90.0, 5.0),
             ((0.0, 0.0, back), 90.0, 5.0), ((0.0, 40.0, -20.0), 0.0, 5.0)]
    px = 620
    sheet = Image.new('RGB', (px * 4, px * 2 + 30), (18, 18, 18))
    for i, (pose, yaw, pitch) in enumerate(looks):
        sheet.paste(m.draw(posed(*pose, only=HARNESS if i == 3 else None), yaw=yaw, pitch=pitch, px=px, half=0.21,
                           centre=(0.0, -0.02, -0.03)),
                    (px * (i % 4), 30 + px * (i // 4)))
    ImageDraw.Draw(sheet).text((12, 8), 'hip: ahead | from its side | behind | its harness alone | above | folded %.0f | back'
                               ' %.0f | out 40' % (-fold, back), fill=(235, 235, 235))
    sheet.save(args.out)
    print(args.out)
    own = 1e3 * float(mechatronics.materials()['hoses']['bus'][4])
    for name, (span, _laid) in hoses().items():
        print('%-5s hose %.0f mm, its least bend %.1f mm (the hose\'s own %.0f)' % (
            name, 1e3 * hung.long(span), 1e3 * hung.least(span), own))
    return 0


if __name__ == '__main__':
    sys.exit(main())
