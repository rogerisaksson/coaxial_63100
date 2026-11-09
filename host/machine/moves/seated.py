"""Her legs and centre of mass sat or sitting down: the knee and ankle a seat asks, where they put her.

    knee, ankle = seated.landed(hips, edge, top, trunk)   # her hips `hips` m up onto a seat `top` up, `edge` behind
    z = seated.com(knee, ankle, trunk, modes, body)     # her centre of mass in her sagittal unit

Her figure's links and segments (`machine.body.figure`); her modes' units her manifest's (`defrobot.read`).
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import figure
from machine.moves import leap
from machine.law import modal, poise
from machine.stack import rounds
from machine.texts import kernel

#: Halvings of her thigh's length that find a landing's hips (`landed`): 1e-13 m.
HALVINGS = 40
#: Her soles' back edge ahead of her ankles, m: her ZMP's margin inside her heels (`poise.EXTENT`).
HEELS = leap.MIDDLE - float(poise.EXTENT[0])
#: Her thigh's capsules (`figure.BODY`): radius, from, to, its frame - down it -y, ahead z.
THIGHS = tuple((r, a, b) for seg, r, a, b in figure.BODY if seg == 'thigh')
#: Her hand's capsule (`figure.BODY`): radius, its middle in the hand's frame.
HAND = next((r, a) for seg, r, a, _b in figure.BODY if seg == 'hand')
#: Her thigh's underside at her knee over the floor, m, her shank plumb on a flat sole: a seat over it her
#: feet do not set her on (`acts._sought`).
POPLITEAL = figure.ANKLE_H + figure.SHANK - THIGHS[-1][0]
#: Her leg below its knee, her mass's share (her figure's shank, foot and toes): what a foot she lets go bears
#: resting, 22.7 N of her.
RESTS = sum(figure.shares()[part][0] for part in ('left_shank', 'left_foot', 'left_toes'))
#: Newton's steps that set her fist where it is asked (`fisted`): within 1e-16 m from her arm hanging; the
#: turn its slope is read over, rad.
NEWTON, NUDGE = 12, 1e-6
#: Her fist's centre on the seat she sits on, up off her pelvis's middle, m (`sat`).
ON = figure.FIST_R + figure.SEAT_Y - figure.SEAT_R
#: Her shoulders' and elbows' indices into her driven joints, her left's first (`kept`).
IDX = np.array([rounds.DRIVEN.index(s + j) for s in ('left_', 'right_') for j in ('shoulder', 'elbow')])
_STAND = figure.stand()
_HUNG = dict(_STAND, left_shoulder=0.0, left_elbow=0.0)
_AT = figure.frames(_HUNG, (0.0, 0.0, 0.0), figure.IDENT)
#: Her arm in her sagittal plane (up, ahead of her pelvis's middle), m: her shoulder's seat on her trunk as she
#: stands, its hinge on her pelvis (her spine's seat), her upper arm and her forearm to her fist hanging straight
#: (her shoulder and elbow at 0) - the
#: two links `reach` sets, within 1.6 cm of `figure` over her arm's travel -, her shoulder aside of her middle,
#: m, and her elbow standing, rad.
SHOULDER = np.array(_AT['left_upper_arm'][0][1:])
HINGE = np.array(next(seg[3] for seg in figure.SEGMENTS if seg[0] == 'torso')[1:])
LINKS = (np.array(_AT['left_forearm'][0][1:]) - SHOULDER,
         np.array(figure.fist(_HUNG)[1:]) - np.array(_AT['left_forearm'][0][1:]))
APART, STAND_ELBOW = float(_AT['left_upper_arm'][0][0]), math.radians(_STAND['left_elbow'])
#: Her hips' seats on her pelvis, their middle, m (`figure.SEGMENTS`): her pelvis's middle off her hips' (`kept`).
HIPS = tuple(0.5 * (a + b) for a, b in zip(*(seg[3] for seg in figure.SEGMENTS if seg[0] in ('left_thigh', 'right_thigh'))))
_ADMITTANCE = kernel.node('specialize-with-tactile-arm-sweep', 'appendage-admittance')
#: The kernel's seek onto a surface (`drive-appendage-admittance`): m/s down, and the yield its force error
#: goes at, N s/m (`kept`).
SEEK_M_S, YIELD_NS_M = -float(_ADMITTANCE['seek-velocity-mps'][2]), float(_ADMITTANCE['contact-yield-damping'])


def axial(knee, modes, body):
    """Her height's mode at a knee's bend, rad: her legs' shortening (`modal.shortened`) as her
    posture bends her knees for it (`impedance._poses`), a leg's mass off hers (`statics.frame`)."""
    hips_by = body['mass'] / (body['mass'] - modes['mode_limb_lift'][3])
    return ((math.cos(0.5 * math.radians(body['knee'])) - math.cos(0.5 * knee)) * (figure.THIGH + figure.SHANK)
            / (modes['mode_axial_compression'][0] * hips_by))


def knee(hips, ankle):
    """The knee's bend, rad, her hip joints `hips` m over the floor at, the ankle `ankle` bent."""
    rise = (hips - figure.ANKLE_H - figure.SHANK * math.cos(ankle)) / figure.THIGH
    return ankle + math.acos(max(-1.0, min(1.0, rise)))


def hips(knee, ankle):
    """Her hip joint's place over her ankle, (ahead, up) m, her knee and ankle bent `knee`, `ankle`."""
    return (figure.SHANK * math.sin(ankle) - figure.THIGH * math.sin(knee - ankle),
            figure.SHANK * math.cos(ankle) + figure.THIGH * math.cos(knee - ankle))


def posed(knee, ankle, trunk):
    """{joint: deg}: her knees and ankles bent `knee`, `ankle`, her pelvis and trunk `trunk` rad forward
    - her pitch turns both (`impedance._poses`) -, her soles flat, the rest of her as she stands."""
    deg = figure.stand()
    for side in ('left_', 'right_'):
        deg[side + 'hip'] = math.degrees(ankle - knee - trunk)
        deg[side + 'knee'], deg[side + 'ankle'] = math.degrees(knee), -math.degrees(ankle)
    return deg


def sagittal(p, knee, ankle, trunk):
    """A point `p` of her `posed` (her pelvis's frame, her pelvis at its origin turned `trunk` forward)
    as (ahead of her ankles, up off the floor), m."""
    hx, hy = hips(knee, ankle)
    return (p[2] + figure.HIP_DROP * math.sin(trunk) + hx, p[1] + figure.HIP_DROP * math.cos(trunk) + hy + figure.ANKLE_H)


def ahead(knee, ankle, trunk):
    """Her centre of mass ahead of her ankles, m, `posed`: her figure's segments (`figure.com`), within
    3 mm of her model's. Her manifest's masses lumped, each leg at its thigh's and shank's middles, put
    it 4 mm off: her hips stopped 6 mm over a seat they were asked 5.7 mm into."""
    return sagittal(figure.com(posed(knee, ankle, trunk), (0.0, 0.0, 0.0), figure.rx(trunk)), knee, ankle, trunk)[0]


def com(knee, ankle, trunk, modes, body):
    """Her centre of mass as `ahead` has it in her sagittal unit, off her soles' middle - her
    sagittal mode's zero (`modal.reading`, `wbc`'s sole point). Off her stand's instead, 2.9 cm
    behind it, a descent asked her 2.4 cm ahead of its knee and ankle: her ankle bent 0.41 for 0.32
    and her hips stopped 21 mm over the seat."""
    return (ahead(knee, ankle, trunk) - leap.MIDDLE) / modes['mode_sagittal_shift'][0]


def clear(knee, ankle, edge, top):
    """m her thighs clear a seat's front edge `edge` m behind her ankles, its top `top` m up, her
    knee and ankle bent `knee`, `ankle` - negative, in it: her thigh's capsules (THIGHS) in her
    sagittal plane, the edge a line across her."""
    hx, hy = hips(knee, ankle)
    ux, uy = (figure.SHANK * math.sin(ankle) - hx) / figure.THIGH, (figure.SHANK * math.cos(ankle) - hy) / figure.THIGH
    ex, ey = -edge - hx, top - figure.ANKLE_H - hy
    gaps = []
    for r, a, b in THIGHS:
        ax, ay, bx, by = -a[1] * ux - a[2] * uy, -a[1] * uy + a[2] * ux, -b[1] * ux - b[2] * uy, -b[1] * uy + b[2] * ux
        t = max(0.0, min(1.0, ((ex - ax) * (bx - ax) + (ey - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
        gaps.append(math.hypot(ex - ax - t * (bx - ax), ey - ay - t * (by - ay)) - r)
    return min(gaps)


def landed(hips, edge, top, trunk):
    """(knee, ankle), rad, that land her hip joints `hips` m over the floor as far behind her ankles
    as sets her centre of mass at her soles' back edge (HEELS), her trunk `trunk` rad forward -
    halved for till HALVINGS -, as her thighs clear the edge of a seat `top` m up and `edge` m
    behind her ankles (`clear`), and no further than a buttock past it (`figure.SEAT_Z`). Over her
    ankles her hips met a seat 0.19 m behind her soles' middle 2.6 cm onto it, her pelvis alone on
    its edge, and it bore 0.39 of her: 20 docks of 25 never borne. A buttock past an edge 0.12 m
    behind her soles' middle her thighs were 14 mm into it: they met it 2-5 cm over the seat, one
    first, her pelvis hung off it and fell back. At the suite's ankle, 0.32, her centre of mass
    stood 7 mm inside her heels' edge, her balance held it 28 mm behind her ankles and her hips
    stopped 21 mm over the seat; landed a buttock's length behind the seen edge whatever her centre
    of mass, she rose 9 times of 14."""
    lo, hi = 0.0, figure.THIGH
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        knee, ankle = legs(hips, mid)
        back = ahead(knee, ankle, trunk) < HEELS or clear(knee, ankle, edge, top) < 0.0
        lo, hi = (lo, mid) if back else (mid, hi)
    return legs(hips, min(0.5 * (lo + hi), edge - figure.SEAT_Z[0]))


def legs(hips, behind):
    """(knee, ankle), rad, that put her hip joints `hips` m over the floor and `behind` m behind her
    ankles: her leg's two links from the ankle (`figure.SHANK`, `THIGH`), its knee by the cosines."""
    up, shank, thigh = hips - figure.ANKLE_H, figure.SHANK, figure.THIGH
    knee = math.acos(max(-1.0, min(1.0, (behind * behind + up * up - shank * shank - thigh * thigh) / (2.0 * shank * thigh))))
    return knee, math.atan2(-behind, up) + math.atan2(thigh * math.sin(knee), shank + thigh * math.cos(knee))


def sat(seat):
    """Her hip joints' height sat on a surface `seat` m up: her buttocks under her pelvis, her hips
    under its middle (`figure`)."""
    return seat - figure.SEAT_Y + figure.SEAT_R - figure.HIP_DROP



def braced(trunk):
    """Her shoulders' flexion, rad (back negative), that sets her fists on the seat she sits on behind her,
    her spine `trunk` rad forward (back negative), her elbows as she stands - halved for till
    HALVINGS: sat, her pelvis's middle SEAT_R - SEAT_Y over the seat's top (`sat`)."""
    lo, hi, deg = -0.5 * math.pi, 0.0, dict(figure.stand(), spine=math.degrees(trunk))
    r, (hx, hy, hz) = HAND
    for _ in range(HALVINGS):
        mid = 0.5 * (lo + hi)
        deg['left_shoulder'] = math.degrees(mid)
        at, here = figure.frames(deg, (0.0, figure.SEAT_R - figure.SEAT_Y, 0.0), figure.IDENT)['left_hand']
        low = at[1] + sum(here[1][k] * c for k, c in enumerate((hx, hy, hz))) - r
        lo, hi = (lo, mid) if low < 0.0 else (mid, hi)
    return 0.5 * (lo + hi)


def fisted(up, ahead, trunk):
    """(shoulder, elbow) flexion, rad, that set her fist `up` m over and `ahead` m ahead of her pelvis's middle,
    her spine `trunk` rad forward (`figure.fist`, her right's mirroring her left's): NEWTON steps from her arm
    hanging, its slope read NUDGE apart, her elbow no straighter than straight."""
    deg = dict(figure.stand(), spine=math.degrees(trunk))
    q = (0.0, math.radians(deg['left_elbow']))

    def off(q):
        deg['left_shoulder'], deg['left_elbow'] = math.degrees(q[0]), math.degrees(q[1])
        at = figure.fist(deg)
        return at[1] - up, at[2] - ahead
    for _ in range(NEWTON):
        e = off(q)
        (a, b), (c, d) = (((f[0] - e[0]) / NUDGE, (f[1] - e[1]) / NUDGE)
                          for f in (off((q[0] + NUDGE, q[1])), off((q[0], q[1] + NUDGE))))
        det = a * d - b * c
        q = (q[0] - (d * e[0] - c * e[1]) / det, max(0.0, q[1] - (a * e[1] - b * e[0]) / det))
    return q


#: Her elbows this much straighter than the reach to a planted fist, rad (`kept`): her arms press on it and never
#: draw her trunk down onto it (folded to the reach ahead of her pitch they hauled her back into a fall, 853 deg/s).
FOLD_MARGIN = 0.15


def planted(ahead, modes=None):
    """(shoulder flexion, elbow fold past standing), rad - in her arm modes' units given `modes` -, that set her fist
    on the seat she sits on, `ahead` m of hers (at her SCALE) ahead of her pelvis's middle (behind negative), her pelvis upright and her
    spine straight (`fisted`): the point her text plants it at, read back off the ask and kept there, pressed to its
    told share, as her pelvis pitches by `kept`. Planted for the stage's told pitch instead, hers sat 0.68 rad forward
    of it, they hung 0.11-0.15 m over the foam and the first fist down bore alone."""
    shoulder, elbow = fisted(ON, figure.SCALE * ahead, 0.0)
    arm, fold = (1.0, 1.0) if modes is None else (modes['mode_arm_reach'][0], modes['mode_arm_fold'][0])
    return shoulder / arm, (elbow - STAND_ELBOW) / fold


def _turned(a, up, ahead):
    """(up, ahead) turned by `a` rad in her sagittal plane, up toward ahead."""
    return np.cos(a) * up - np.sin(a) * ahead, np.sin(a) * up + np.cos(a) * ahead


def fist(shoulder, elbow):
    """(up, ahead) of her fist of her pelvis's middle, m, her pelvis upright and her spine straight, at the flexions
    `shoulder` and `elbow` (rad): the links LINKS, each turned back by the flexions above it."""
    u1, a1 = _turned(-shoulder, LINKS[0][0], LINKS[0][1])
    u2, a2 = _turned(-shoulder - elbow, LINKS[1][0], LINKS[1][1])
    return SHOULDER[0] + u1 + u2, SHOULDER[1] + a1 + a2


def _pitch(w, x, j, k, rot):
    """The pitch along her way, rad, of a body's quaternion (w, x, j, k): its up toward her way (`modal.pitched`)."""
    up0, up1, up2 = 2.0 * (x * j - w * k), 1.0 - 2.0 * (x * x + k * k), 2.0 * (j * k + w * x)
    return np.arctan2(up0 * rot[modal.ALONG][0] + up1 * rot[modal.ALONG][1] + up2 * rot[modal.ALONG][2], up1)


def reach(point, spine, high):
    """(shoulder, elbow) flexions, rad, (2, n): her fists at `point` (up, ahead) of her pelvis's middle in its frame, her
    spine `spine` rad forward, each shoulder `high` (n,) m up of its seat: the elbow's fold from the length of the
    reach from the shoulder, the shoulder's from its direction, her arm two links (LINKS); past her reach the arm
    points straight at it."""
    at = np.stack(_turned(spine, SHOULDER[0] - HINGE[0], SHOULDER[1] - HINGE[1])) + HINGE
    du, da = _turned(-spine, point[0] - at[0] - high, point[1] - at[1] + 0.0 * high)
    l1, l2 = (np.sqrt(v[0] * v[0] + v[1] * v[1]) for v in LINKS)
    hang = np.arctan2(LINKS[0][1], LINKS[0][0]) - np.arctan2(LINKS[1][1], LINKS[1][0])
    elbow = np.arccos(np.clip((du * du + da * da - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0)) - hang
    vu, va = _turned(-elbow, LINKS[1][0], LINKS[1][1])
    shoulder = np.arctan2(LINKS[0][1] + va, LINKS[0][0] + vu) - np.arctan2(da, du)
    return np.arctan2(np.sin(shoulder), np.cos(shoulder)), elbow


def _at(spine, shoulder, elbow):
    """(up, ahead) of her fist of her pelvis's middle in its frame, m, at the flexions `shoulder` and `elbow` (rad)
    with her spine `spine` rad forward: the links LINKS from her shoulder's seat turned with her spine."""
    au, aa = _turned(spine, SHOULDER[0] - HINGE[0], SHOULDER[1] - HINGE[1])
    u1, a1 = _turned(spine - shoulder, LINKS[0][0], LINKS[0][1])
    u2, a2 = _turned(spine - shoulder - elbow, LINKS[1][0], LINKS[1][1])
    return HINGE[0] + au + u1 + u2, HINGE[1] + aa + a1 + a2


#: How far the two links' fist (`fist`, `_at`) is from `figure`'s at worst over her arm's travel, m: how near
#: a text's point must lie within of the seat to be a fist planted on it (`kept`).
LEAD = max(float(np.hypot(*(np.array(figure.fist(dict(_HUNG, spine=sp, left_shoulder=sh, left_elbow=el))[1:])
                              - np.array(_at(math.radians(sp), math.radians(sh), math.radians(el))))))
           for sp in (-30.0, 0.0, 30.0) for sh in (-60.0, -30.0, 0.0, 20.0) for el in (10.0, 50.0, 90.0, 130.0))


#: How far off the seat `fist` reads a fist `planted` on it at worst, m: a text's point is on the seat within it and
#: not pressed below it (the hitch presses its own a centimetre under, `acts._sought`: its arms are its own).
READ = max(abs(fist(sh, el + STAND_ELBOW)[0] - ON) for sh, el in (planted(a) for a in (-0.2, -0.14, -0.05, 0.0, 0.1)))


def kept(y, fr, palmar, posture, q, fists):
    """(`posture`, her fists' latch (14,): each fist's point (up, ahead of her pelvis's middle), her hips along her
    way and up then (her pelvis's middle: her hips' moved 3-4 cm against it as it rolled back), whether it has borne, how far it is pressed, and where it was at its touch) with her shoulders and
    elbows keeping her fists on the seat she is told to bear on (`palmar`, her palms' told share, past 0). A fist not
    yet borne is sought to the point her text set (`planted`, read back off `posture` by `fist`); one that has borne
    (`modal.PALMS`, felt at all) keeps that point, fixed along her way as her hips scoot past it (the take's hands
    stay planted as the hips pass them; pelvis-relative in height the chair's rise lifted them off its seat), and is pressed into the seat to its told share by the kernel's admittance
    (SEEK_M_S, YIELD_NS_M) at a gain bounded by the seat's stiffness as identified - the secant of its felt force over
    its sink since the touch (`_at` of her arm as sensed), the admittance's critical stiffness YIELD_NS_M / dt before
    any -, the press leading the fist by a seek step at most, so it waits for the arm and never fights a floor (led by
    the reach's error LEAD instead it pressed her off the lean, the solve shrunk to 0.01): dead-beat on a rigid floor, a slow sink on foam that creeps (a mattress: 1 cm in bore 0 N, 2-7 cm 30-50 N
    and crept 1-3 cm/s; pressed a fixed centimetre instead the fists bore nothing there and fought the rigid floor,
    and the floor scoot fell back). Either point is reached under her pelvis's pitch as sensed, her spine's bend and
    her trunk's roll (each shoulder's height, `reach`), her elbow FOLD_MARGIN straighter, no straighter than standing;
    told no share the latch lets go; a point off the seat (the sled gaits' anchors, LEAD) or pressed under it (the
    hitch's, READ) is never taken up, and a fist once borne is kept though the ask drifts a millimetre out of that band
    (gated each tick the latch flickered off mid-stage and the scenario was hauled up). Held at its joints instead the arm pulled the fist back as her trunk reclined
    and dropped her at 75 deg/s; planted for the stage's told pitch (`planted` before) they hung 0.11 m over the
    foam, the first down bore alone and 13 deg of roll and yaw onto it held her feet at 70 N and the lean at -16."""
    rot = fr[modal.ROT].reshape(3, 3)
    pitch = np.stack([_pitch(y[modal.QUAT][0], y[modal.QUAT][1], y[modal.QUAT][2], y[modal.QUAT][3], rot),
                      _pitch(y[modal.QUAT][4], y[modal.QUAT][5], y[modal.QUAT][6], y[modal.QUAT][7], rot)])
    high = np.array([1.0, -1.0]) * (2.0 * (y[modal.QUAT][5] * y[modal.QUAT][6] + y[modal.QUAT][4] * y[modal.QUAT][7]) * APART)
    hips = y[modal.HIP]
    along = 0.5 * ((hips[0] + hips[3]) * rot[modal.ALONG][0] + (hips[1] + hips[4]) * rot[modal.ALONG][1]
                   + (hips[2] + hips[5]) * rot[modal.ALONG][2])
    w, x, j, k = y[modal.QUAT][0], y[modal.QUAT][1], y[modal.QUAT][2], y[modal.QUAT][3]
    tall = 0.5 * (hips[1] + hips[4]) - ((1.0 - 2.0 * (x * x + k * k)) * HIPS[1] + 2.0 * (j * k - w * x) * HIPS[2])
    weight, dt = fr[modal.WEIGHT], fr[modal.DT]
    felt = y[modal.PALMS] * weight
    spine = pitch[1] - pitch[0]
    where = np.stack([_turned(pitch[0], *_at(spine, q[IDX[0]], q[IDX[1]]))[0],
                      _turned(pitch[0], *_at(spine, q[IDX[2]], q[IDX[3]]))[0]])
    was = np.stack([fists[3], fists[10]])
    pt_up, pt_ahead = fist(posture[IDX[0]], posture[IDX[1]])
    on = (palmar > 0.0) * np.maximum(was, (pt_up >= ON - READ) * (pt_up <= ON + LEAD) * 1.0)
    up = was * np.stack([fists[0], fists[7]]) + (1.0 - was) * pt_up
    ahead = was * np.stack([fists[1], fists[8]]) + (1.0 - was) * pt_ahead
    at = was * np.stack([fists[2], fists[9]]) + (1.0 - was) * along
    high_at = was * np.stack([fists[6], fists[13]]) + (1.0 - was) * tall
    touched = was * np.stack([fists[5], fists[12]]) + (1.0 - was) * where
    borne = on * np.maximum(was, (felt > 0.0) * 1.0)
    sink = np.maximum(touched - where, 0.0)
    known = (felt > 0.0) * (sink > 0.0) * 1.0
    stiff = known * felt / np.maximum(sink, 1e-6) + (1.0 - known) * YIELD_NS_M / dt
    rate = np.clip((0.5 * palmar * weight - felt) / np.maximum(YIELD_NS_M, stiff * dt), -SEEK_M_S, SEEK_M_S)
    press = on * np.maximum(np.minimum(np.stack([fists[4], fists[11]]) + rate * dt,
                                       np.maximum(up - where, 0.0) + SEEK_M_S * dt), 0.0)
    pu, pa = _turned(-pitch[0], up - (tall - high_at) - press, ahead - (along - at))
    shoulder, elbow = reach(np.stack([pu, pa]), spine, high)
    elbow = np.maximum(elbow - FOLD_MARGIN, STAND_ELBOW)
    out = posture.copy()
    out[IDX] = posture[IDX] + np.stack([on[0], on[0], on[1], on[1]]) * (np.stack([shoulder[0], elbow[0], shoulder[1], elbow[1]]) - posture[IDX])
    return out, np.stack([up[0], ahead[0], at[0], borne[0], press[0], touched[0], high_at[0],
                          up[1], ahead[1], at[1], borne[1], press[1], touched[1], high_at[1]])


def placed(up, trunk):
    """(shoulder, elbow, aside, levers) of her fist set `up` m over her pelvis's middle under her shoulder, her
    spine `trunk` rad forward: its flexions, rad (`fisted`); how far aside of her middle it stands, m; and the
    reach along her way from her shoulder and her elbow to it, m - what a push up on her fist turns them by."""
    deg = dict(figure.stand(), spine=math.degrees(trunk))
    shoulder = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0]
    flexed = fisted(up, shoulder[2], trunk)
    deg['left_shoulder'], deg['left_elbow'] = (math.degrees(v) for v in flexed)
    at, elbow = figure.fist(deg), figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_forearm'][0]
    return flexed[0], flexed[1], abs(at[0]), (abs(at[2] - shoulder[2]), abs(at[2] - elbow[2]))


def reaches(top):
    """Whether her legs reach the floor from her hips on a seat `top` m up (`sat`), plumb and straight: past
    it nothing her soles stand on bears her up onto it, her hands no help."""
    return sat(top) - figure.ANKLE_H <= figure.THIGH + figure.SHANK
