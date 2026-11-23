"""The knee's mount, sketched: printed clamps and a black cage round the calf's inverters.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/knee_sheet.py OUT.png [--ahead M]

The knee's stack and the ankle's two drives under it as `skeleton.cans` lays them - the two back to back on one axis,
each inverter on its motor's back between them -, the leg's tubes' radii as `[skeleton] hung` has them. Printed
pieces, each laid on a bed face across the knee's axis and grown from it with nothing past 45 degrees. The stator
clamp (the thigh's side), printed from its front face: a slit band round the motor and on round the gearbox's still
housing to that face, the saddle up to the thigh's seat, its waist the least that holds over what of it stands on
the band; where the knee's inverter rides its motor's back, the board's cover behind it (`covered`) - its face flush
with the clamp's back, an open black honeycomb, the bus's hose in a holder of its ring and its lid off beside it
(`fittings`), the saddle over it - else a lip the motor is pushed back against. The multi-clamp (the shank's side),
printed from its front face and pushed onto the knee's output ring: a sleeve round that ring, webs, a slit band
round the pitch drive's motor and on from its back a solid black cage round both inverters, a narrow slot for each
phase, and the shank's seat on a foot under it - the foot on the cage and on past it on the pitch drive's band,
its waist the least that holds over both. The pitch drive goes in through the cage's open top, its inverter resting on the cage's floor; the roll
drive, its inverter first, after it, held by a collar - the third piece, printed from its flange - screwed onto the
cage's top. The drives let down the shank just so far that the cage keeps AIR_M off the knee's stator (`laid`); the
thigh's tube on a line through the hip's axis, leant (`tilt`) to pass `ahead` of the knee's by the least that lets the
knee bend to the end of its travel (`knee_stop.ahead`), the shank's tube on its own line (`shank_line`).
"""
import functools
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import boards  # noqa: E402
from tools.render import calf_wired as cw  # noqa: E402
from tools.render import clamped as cl  # noqa: E402
from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import sized as ks  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import tees  # noqa: E402

#: The air each clamp keeps off what it passes, how far a lip overlaps the face of the drive pushed against it and
#: how thick the stator clamp's lip is along the axis, m.
AIR_M, LIP_M, LIP_X = 0.002, 0.004, 0.007
#: The depth of the foot the shank's seat stands on, m (its waist and the web's from the loads, `sized.neck`).
FOOT_M = 0.012
#: The collar's flange's thickness, m, and the screws holding it to the cage's top.
COLLAR_M, SCREWS = 0.006, 6
#: How far the exploded view pulls each part along the way it goes on, m.
APART_M = 0.05
#: The printed pieces: (still's, turning's, turning's, still's) parts each takes in.
PIECES = (('stator band', 'saddle', 'thigh seat'), ('output band', 'web', 'pitch band', 'cage', 'shank seat'),
          ('collar',), ('board cover',), ('board lid',))


def _named():
    """{joint: ({part: (x0, x1, r)}, (y, z))} of the knee's and the ankle's drives as `skeleton.cans` lays them."""
    from machine.body import drives, skeleton
    stacks = {}
    for joint, _rides, at, _axis, r, half in skeleton.cans():
        if joint in ('left_knee', 'left_ankle', 'left_ankle_roll'):
            parts, _yz = stacks.setdefault(joint, ([], (float(at[1]), float(at[2]))))
            parts.append((float(at[0]) - half, float(at[0]) + half, float(r)))
    return {j: ({p: s for (p, *_), s in zip(drives.along(j), parts)}, yz) for j, (parts, yz) in stacks.items()}


def knee_board():
    """The knee's inverter (x0, x1, r) where it rides its motor's back; None where it lies apart."""
    return _named()['left_knee'][0].get('board')


def _stack():
    """(knee's motor (x0, x1, r), knee's gear (x0, x1, r), {'pitch'/'roll': ({part: (x0, x1, r)}, (y, z))}, thigh's
    tube r, shank's tube r): the knee's gearbox at +x, each ankle drive where `skeleton.cans` lays it on the shank."""
    from machine.body import skeleton
    named = _named()

    def radius(seg, end):
        r = skeleton.HUNG[seg][1]
        return float(r[end] if isinstance(r, tuple) else r)
    knee = named['left_knee'][0]
    return (knee['motor'], knee['gear'], {'pitch': named['left_ankle'], 'roll': named['left_ankle_roll']},
            radius('thigh', -1), radius('shank', 0))


def _need(a, b):
    """The least distance between the axes of two stacks of drums [(x0, x1, r)] that leaves AIR_M between each two
    sharing a span along x, m (0 where none do)."""
    return max([ra + rb + AIR_M for a0, a1, ra in a for b0, b1, rb in b if min(a1, b1) > max(a0, b0)], default=0.0)


def laid():
    """The trial's layout, m: {'bone': the tubes' line along x, 'bed': (stator clamp's, multi-clamp's, collar's) bed
    faces along x, 'stator', 'output', 'pitch_band', 'collar': profiles [(x, bore, outside)], 'cage': (x0, x1, bore,
    outside), 'pitch'/'roll': ({part: (x0, x1, r)}, (y, z)) the ankle's drives back to back on one axis, let down the
    shank just so far that the cage and the collar keep AIR_M off the knee's stator, 'drop': that move, 'walls':
    each band's wall, gripping its drum's torque and hiding its pinch screws (`sized.band`), 'cover': the knee's
    board's cover's (face, seat, board's radius, band's outside) or None, 'rim': the cage's rim's (wall, its
    screws' thread mm), the collar screwed into it (`sized.rim`), 'saddle': the stator clamp's saddle's (from, to)
    along x - from the clamp's back to AIR_M short of the multi-clamp, the stator's band on round the gearbox's
    still housing to its front: the clamp grows off that face, nothing hung}."""
    (m0, m1, can), (g0, g1, ring), ankles, thigh, shank = _stack()
    (pitch, (py, pz)), (roll, (ry, rz)) = ankles['pitch'], ankles['roll']
    assert abs(py - ry) < 1e-9 and abs(pz - rz) < 1e-9, "the ankle's drives on one axis"
    (pm0, _pm1, r_m), (rm0, rm1, _rm) = pitch['motor'], roll['motor']
    (pg0, pg1, _pgr) = pitch['gear']
    assert rm1 <= pm0 and 'board' in pitch and 'board' in roll, 'the inverters on the motors\' backs, facing'
    k_split = g0 + em.OUTPUT_SHARE * (g1 - g0)           # the knee's gearbox's still housing, then its output ring
    bed_s = max(g1, pg0 + em.OUTPUT_SHARE * (pg1 - pg0) - 0.0025)   # the pitch drive's housing's end
    board = knee_board()
    bed_t = board[0] - AIR_M - cv.WALL_M if board else m0 - LIP_X   # the cover's face, else the lip behind the motor
    bone = bed_t + thigh + em.OVAL_PAST[0]               # the thigh's seat flush with the stator clamp's back
    front = min(bone + thigh + em.OVAL_PAST[0], min(k_split + 0.0025, pm0) - AIR_M)   # its saddle's front, AIR_M
    knee, ankle = kl.loads('knee')['swing'], ks.torque('ankle')                       # short of the multi-clamp
    grips = {'stator': (knee, 1e3 * can, 1e3 * (m1 - 0.002 - m0)), 'output': (knee, 1e3 * ring, 1e3 * (
        g1 - k_split - 0.0025)), 'pitch': (ankle, 1e3 * r_m, 1e3 * (bed_s - pm0)), 'collar': (ankle, 1e3 * r_m, 1e3 * (
            rm1 - COLLAR_M - rm0 - 0.002))}
    walls = {k: 1e-3 * ks.band(*grip) for k, grip in grips.items()}
    t_s, t_o, t_p, t_l = walls['stator'], walls['output'], walls['pitch'], walls['collar']
    stator = ([] if board else [(bed_t, can - LIP_M, can + t_s), (m0, can - LIP_M, can + t_s)]) + [
        (m0, can, can + t_s), (max(m1 - 0.002, front), can, can + t_s)]   # on round the gearbox's housing to it
    cover = (bed_t, m0, board[2], can + t_s) if board else None
    output = [(k_split + 0.0025, ring, ring + t_o), (bed_s, ring, ring + t_o)]   # on past the ring, a sleeve
    r_c = max(pitch['board'][2], roll['board'][2]) + boards.LEAD_M   # round the inverters and their leads
    rim = ks.rim(ankle, 1e3 * r_c, SCREWS)               # the collar screwed into its rim, carrying the roll's torque
    o_c, o_m = r_c + 1e-3 * rim[0], r_m + t_p
    pitch_band = [(pm0, r_m, o_c), (pm0 + o_c - o_m, r_m, o_m), (bed_s, r_m, o_m)]   # its outside a 45-degree cone
    cage = (rm1, pm0, r_c, o_c)                          # from the roll motor's back to the pitch motor's
    collar = [(rm0 + 0.002, r_m, r_m + t_l), (rm1 - COLLAR_M, r_m, r_m + t_l), (rm1 - COLLAR_M, r_m, o_c),
              (rm1, r_m, o_c)]
    still = [(m0, m1, can), (g0, k_split, ring)] + cl.hull(stator) + ([board, cv.span(*cover)] if board and cover else [])
    held = (list(pitch.values()) + list(roll.values()) + cl.hull(pitch_band) + [(cage[0], cage[1], o_c)]
            + cl.hull(collar))
    cy = min(py, -math.sqrt(max(_need(held, still) ** 2 - pz ** 2, 0.0)))
    return {'bone': bone, 'bed': (bed_t, bed_s, rm1), 'stator': stator, 'output': output, 'pitch_band': pitch_band,
            'cage': cage, 'collar': collar, 'pitch': (pitch, (cy, pz)), 'roll': (roll, (cy, rz)), 'drop': py - cy,
            'walls': walls, 'cover': cover, 'rim': rim, 'saddle': (bed_t, front)}


@functools.lru_cache(maxsize=None)
def fittings():
    """(pad, spot, marks): the knee's hose's two outlets, (y, z) m about the knee's axis with the leg straight - `spot`
    on the calf's collar's flange midway across it, `pad` on the knee's board's guard's face - and the bus's marks
    on that guard [(kind, (y, z) shares of its bore)]. The collar's ahead of the calf's drives, off their rods
    behind, as far round as its pipe still runs to the knee's axis clear of the collar - where that way grazes
    the flange's circle -, the nearest there clear of the flange's screws' heads; the guard's across the knee's
    axis from it with the knee at the middle of its travel (`covered.across`). None with no board on the knee."""
    import numpy as np
    from machine.body import drives
    lay = laid()
    if not lay['cover']:
        return None
    (cy, cz), (_c0, _c1, r_c, o_c) = lay['roll'][1], lay['cage']
    ring, rim = 0.5 * (lay['collar'][0][2] + o_c), tees.PIPE[1] + 0.0018   # midway across; the outlet's collar
    head = 1e-3 * ks.PINCHES[ks.THREADS.index(lay['rim'][1])][1]         # a screw's head, on the rim's circle
    screws = np.arange(SCREWS) * 2.0 * math.pi / SCREWS
    clear = math.asin(min(1.0, (rim + head + AIR_M) / (0.5 * (r_c + o_c))))
    turns = np.radians(np.arange(0.0, 180.0, 0.5))                       # ahead of the calf's axis
    free = turns[np.abs((turns[:, None] - screws[None, :] + math.pi) % (2.0 * math.pi) - math.pi).min(axis=1) >= clear]
    graze = math.acos(ring / math.hypot(cy, cz))                         # its way to the knee's axis a tangent
    turn = float(free[free <= graze].max())
    spot = (cy + ring * math.cos(turn), cz + ring * math.sin(turn))
    pad, marks = cv.across(math.atan2(spot[1], spot[0]), drives.TRAVEL['knee'], lay['cover'][2], lay['cover'][3])
    return pad, spot, marks


def seats(ahead=0.0):
    """(top, low), m: the stator clamp's saddle's top on its band, the seat's depth `ahead` of the axis, and the
    multi-clamp's foot's bottom under the cage - the levels the two tubes' seats sit on."""
    (_m0, _m1, can), _gear, _ankles, thigh, _shank = _stack()
    lay = laid()
    over = can + lay['walls']['stator'] - 0.001
    over = max(over, cv.span(*lay['cover'])[2] + cv.FIT_M) if lay['cover'] else over   # over the board's cover
    deep_t = thigh + em.OVAL_PAST[1]
    z_t = (ahead - deep_t, ahead + deep_t)
    near = 0.0 if z_t[0] <= 0.0 <= z_t[1] else min(abs(z_t[0]), abs(z_t[1]))
    top = math.sqrt(max(over ** 2 - near ** 2, 0.0)) + em.SADDLE_M
    return top, lay['pitch'][1][0] - (lay['cage'][3] - 0.001) - FOOT_M


def tilt(ahead):
    """The thigh's tube's lean back from upright about the knee's axis, rad: its line through the hip's axis - radial
    there, where nothing folds against it - and `ahead` of the knee's, m."""
    from machine.body.figure import THIGH
    return math.asin(ahead / THIGH)


def shank_line():
    """How far ahead of the ankle's drives' axis the shank's tube runs, m: nearly through it - the slimmest shin -,
    as far forward as keeps its fork AIR_M off the rods' lower ends where the toes are farthest down (`leaves`)."""
    return leaves.reach()[1] + AIR_M + _stack()[4] + em.OVAL_PAST[1]


def _saddle(x_t, over, top, z_t, cover):
    """[(tri, colour)] of the stator clamp's saddle along x: C1-smooth transition and 37.6-deg end chamfers."""
    import numpy as np
    from tools.render import csg
    r_hi, on = over, x_t[0]
    if cover:                                            # over the cover the saddle keeps off it, on the band it holds
        _a, on, outside = cv.span(*cover)
        r_hi = max(over, outside + cv.FIT_M)
    angs = np.radians(np.linspace(90.0, -75.0, 37))

    def ring_2d(r):
        arc = [(r * math.cos(a), r * math.sin(a)) for a in angs]
        pts = [(top, -r_hi * math.sin(math.radians(75.0))), (top, z_t[1] + 0.004), (0.0, r_hi + 0.008)] + arc
        return csg.filleted2d(pts, r=0.003)

    step = on + 1.25 * max(0.0, r_hi - over)
    xs = sorted({x_t[0], x_t[0] + 0.00156, min(on, x_t[1] - 0.002),
                 *np.linspace(min(on, x_t[1] - 0.002), min(step, x_t[1] - 0.002), 8)[1:],
                 x_t[1] - 0.00156, x_t[1]})
    rings = []
    for k, x in enumerate(xs):
        t = 0.0 if x <= on or step <= on + 1e-9 else float(np.clip((x - on) / (step - on), 0.0, 1.0))
        r = r_hi - (r_hi - over) * 0.5 * (1.0 - math.cos(math.pi * t))
        p2 = ring_2d(r)
        if k in (0, len(xs) - 1):
            p2 = csg.inset2d(p2, 0.0012, outer=True)
        rings.append(np.c_[np.full(len(p2), x), p2])
    return csg.loft(rings, colour=em.FRAME).tris()


def bodies(apart=False, flex=0.0, socket='torx', ahead=0.0):
    """(parts, cables, still {name: tris}, turning {name: tris}) of the knee's trial, the shank's side turned by
    `flex` degrees, the thigh's tube's line through the hip's axis and `ahead` of the knee's, the shank's on its own
    (`shank_line`), m."""
    import numpy as np
    (m0, m1, can), (g0, g1, ring), _ankles, thigh, shank = _stack()
    lay = laid()
    (pitch, (py, pz)), (roll, (ry, rz)) = lay['pitch'], lay['roll']
    bone, (bed_t, bed_s, bed_c), (c0, c1, r_c, o_c) = lay['bone'], lay['bed'], lay['cage']
    k_split = g0 + em.OUTPUT_SHARE * (g1 - g0)
    inks = {'board': em.BOARD, 'motor': em.SERVO, 'gear': em.GEAR}

    def moved(tris, x=0.0, y=0.0, z=0.0):
        return [(tri + (x, y, z), c) for tri, c in tris]

    def drums(stack, y, z, turn=0.0):                    # an inverter the board itself where its model is beside
        out = []
        for part, (x0, x1, r) in stack.items():
            board = part == 'board' and boards.placed(x0, x1, (y, z), x1 if stack['motor'][0] >= x1 - 1e-9 else x0, turn)
            out += board or moved(em._drum(r, x0, x1, inks[part]), y=y, z=z)
        return out

    e_t = thigh - 0.0012 - em.BORE_R - 0.003             # the bores off the tubes' axis, as the elbow's
    e_s = shank - 0.0012 - em.BORE_R - 0.003
    still, turning = {}, {}
    still['knee drive'] = em._drum(can, m0, m1, em.SERVO) + em._drum(ring, g0, k_split, em.GEAR)
    board = knee_board()
    if board:                                            # its bus's terminals beside the hose's holder, its lid off
        fitted: Any = fittings()
        pad, _spot, marks = fitted
        still['knee drive'] += boards.placed(board[0], board[1], (0.0, 0.0), board[1], cv.turned(pad)) or em._drum(
            board[2], board[0], board[1], em.BOARD)
        face, _seat, board_r, band_r = lay['cover']
        laid_at = (board[0], board[1], (0.0, 0.0), board[1], cv.turned(pad))
        still['knee leads'] = (boards.leads(*laid_at, band_r) + boards.wired(     # the phases' leads; the bus's,
            *laid_at, 0.0, pad, board[1] - face, True))                             # a branch off the holder's T-cross
        still['board cover'] = cv.cover(*lay['cover'], outlet=pad, window=boards.window(cv.turned(pad)))
        still['board lid'] = cv.lid(face, board_r, band_r, marks, pad)
    still['stator band'] = cl.turned(lay['stator'], math.pi)
    over = can + lay['walls']['stator'] - 0.001          # the saddle on the band, its waist the least that holds
    reach_t, deep_t = thigh + em.OVAL_PAST[0], thigh + em.OVAL_PAST[1]
    z_t = (ahead - deep_t, ahead + deep_t)
    top, low = seats(ahead)
    first = lay['output'][0][0]                          # the multi-clamp's web starts at the cage's end or its band
    x_t = lay['saddle']                                  # the saddle AIR_M short of it, however bent
    still['saddle'] = _saddle(x_t, over, top, z_t, lay['cover'])
    still['thigh seat'] = em._rounded(cl.oval(reach_t, deep_t, (bone, ahead)), top - 0.002, top + em.SEAT_M,
                                      em.PLATE, r=cl.EDGE_M, steps=1)
    face_t = top + em.SEAT_M
    dress, limb = em._limb(face_t, 1.0, thigh, -e_t, socket, apart)
    still['thigh tube'] = moved(dress + limb, x=bone, z=ahead)
    lean = tilt(ahead)                                   # the thigh's side drawn upright, then leant to the hip
    for k in ('saddle', 'thigh seat', 'thigh tube'):
        still[k] = cl.tilted(still[k], -lean, (0.0, 0.0))
    turning['output ring'] = em._drum(ring, k_split, g1, em.OUTPUT)
    turning['output band'] = cl.turned(lay['output'], em.SLIT)
    turning['pitch drive'] = drums(pitch, py, pz)
    turning['roll drive'] = drums(roll, ry, rz, cw.TURNS['roll'])
    spot: Any = fittings()                               # the calf's boards branches off the collar's T-cross
    turning['calf leads'] = cw.leads(lay, spot[1]) if spot else []
    turning['pitch band'] = moved(cl.turned(lay['pitch_band'], em.SLIT), y=py, z=pz)
    turning['cage'] = moved(cl.slotted(r_c, o_c, c0, c1, cw.slots(lay), cv.BLACK), y=py, z=pz)
    turning['collar'] = moved(cl.turned(lay['collar'], em.SLIT), y=py, z=pz)
    heads, bolt = [], 0.5 * (r_c + o_c)                  # through the collar's flange into the cage's top rim
    head_r, head_h = 1e-3 * ks.PINCHES[ks.THREADS.index(lay['rim'][1])][1], 1e-3 * lay['rim'][1]
    for t in np.arange(SCREWS) * 2.0 * math.pi / SCREWS:
        c = (py + bolt * math.cos(t), pz + bolt * math.sin(t))
        x_f = bed_c - COLLAR_M
        heads += m._cylinder_x(head_r, x_f - head_h, x_f, em.BOLT, k=16, at=c)
        heads += m._prism(m._socket(socket, head_r * 0.5) + c, x_f - head_h - 0.0002, x_f - head_h, em.VOID, axis=0)
        heads += m._cylinder_x(0.8 * head_h, bed_c - 0.0001, bed_c + 0.0001, m.BRASS, k=16, at=c)    # its insert
    turning['collar screws'] = heads
    depth = 1e-3 * ks.neck(1e3 * (bed_s - first))       # from the knee's output band down to the cage, then the band
    web = []
    for x0, x1, outside in ((first, c1, o_c), (c1, bed_s, lay['pitch_band'][-1][2])):
        if x1 > x0:
            outline = cl.bridged((0.0, 0.0), lay['output'][-1][2] - 0.001, (py, pz), outside - 0.001, depth)
            web += em._rounded(cl.filleted(outline, 0.003), x0, x1, em.FRAME, axis=0, r=cl.EDGE_M, steps=1)
    turning['web'] = web
    reach_s, deep_s = shank + em.OVAL_PAST[0], shank + em.OVAL_PAST[1]
    ahead_s = shank_line()
    z_s = (ahead_s - deep_s, ahead_s + deep_s)
    x_s = (bone - reach_s, bed_s)
    girth = 1e-3 * ks.neck(1e3 * (x_s[1] - max(x_s[0], c0)))   # the foot's waist: the least that holds over what
    foot = em._rounded(cl.filleted(cl.necked((py, pz), o_c - 0.001, low, z_s, girth)), c0, c1, em.FRAME, axis=0,
                       r=cl.EDGE_M, steps=1)                  # stands on the cage and on past it on the pitch's band
    foot += em._rounded(cl.filleted(cl.necked((py, pz), lay['pitch_band'][-1][2] - 0.001, low, z_s, girth)), c1,
                        bed_s, em.FRAME, axis=0, r=cl.EDGE_M, steps=1)
    foot += em._rounded(cl.filleted(cl.necked((py, pz), o_c + AIR_M, low, z_s, girth)), x_s[0], c0, em.FRAME,
                        axis=0, r=cl.EDGE_M, steps=1)
    turning['shank seat'] = foot + em._rounded(cl.oval(reach_s, deep_s, (bone, ahead_s)), low + 0.002,
                                               low - em.SEAT_M, em.PLATE, r=cl.EDGE_M, steps=1)
    face_s = low - em.SEAT_M
    dress, limb = em._limb(face_s, -1.0, shank, -e_s, socket, apart)
    turning['shank tube'] = moved(dress + limb, x=bone, z=ahead_s)
    run = em.FLANGE_M + 0.07
    cables = cl.tilted(km._wire([(bone - e_t, face_t + run, ahead), (bone - e_t, face_t, ahead)], 0.002, em.CABLE),
                       -lean, (0.0, 0.0))
    cables += km._turn(km._wire([(bone - e_s, face_s, ahead_s), (bone - e_s, face_s - run, ahead_s)], 0.002, em.CABLE),
                       flex)
    if apart:
        parts = still['knee drive'] + turning['output ring']
        parts += [t for k in PIECES[0] + ('thigh tube',) for t in moved(still[k], x=-APART_M)]   # off the motor
        parts += moved(still.get('board cover', []), x=-2.0 * APART_M)                  # off the board behind it
        parts += moved(still.get('board lid', []), x=-2.6 * APART_M)                    # its lid off it
        parts += [t for k in PIECES[1] + ('shank tube',) for t in moved(turning[k], x=APART_M)]  # off the output
        parts += moved(turning['pitch drive'], x=-1.8 * APART_M)          # out through the cage's open top
        parts += [t for k in ('collar', 'collar screws') for t in moved(turning[k], x=-2.8 * APART_M)]
        parts += moved(turning['roll drive'], x=-3.8 * APART_M)
        return parts, [], still, turning
    parts = [t for v in still.values() for t in v] + km._turn([t for v in turning.values() for t in v], flex)
    return parts, cables, still, turning


def _framed(parts, pad=1.08):
    """(centre, half) taking in every part, m."""
    import numpy as np
    pts = np.concatenate([tri for tri, _c in parts])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return tuple(0.5 * (lo + hi)), 0.5 * pad * float((hi - lo).max())
