"""The leg's cables across its joints: channels, black pipes and the robot's hose, as the elbow's.

At the knee, as at the elbow (`elbow_mount`): the bus's trunk down the thigh's tube's bore into a channel through the
stator clamp's saddle, out of the saddle's back face into a T-cross on the knee's board's guard's holder (`tees`) -
the cross's arm a stiff bridge on into that outlet, the two too near for a hose -, the board a branch off the cross
(`knee_clamped.knee_board`); from the cross's other arm, aimed at the knee's axis, the robot's hose to a second
T-cross on the calf's collar's flange, aimed at the knee's axis too - the two across the knee's axis from each
other with the knee at the middle of its travel (`knee_clamped.fittings`) -, the chain's marks on the crosses'
bodies, toward the arm the trunk comes by and the one it goes on from (`chain`). Between them the hose hangs from its fittings over the
guard's face, nothing swivelling: straight where they face, bowing either way from there under its own stiffness and
weight, kept AIR_M off the knee's stack and the calf's drives (`span`, `hung`). The calf's two boards are branches
off the collar's cross (`calf_wired`); the trunk goes on from that cross's other arm in a hose laid still, a helix
round the cage to the shank's seat (`tees.run`), clipped on its way (`tees.clipped`). At the ankle the foot's own
cable out of the fork's yoke ahead, the bus's way out of the calf marked by it, laid still too: down past the cross
between the fork's arm and the foot's cheek, a quarter turn forward along the keel's inside, clipped above the joint
and to the keel, into a coupling on the soles' pressure sensors' box at the ball, the way in marked on its lid
(`ankle`).
"""
import functools
import math
from typing import Any

from tools.render import chain
from tools.render import clamped as cl
from tools.render import covered as cv
from tools.render import elbow_mount as em
from tools.render import foot
from tools.render import hung
from tools.render import knee_clamped as kc
from tools.render import knee_loads as kl
from tools.render import knee_mount as km
from tools.render import mounting as m
from tools.render import tees

#: The air the hose keeps off what it passes, m (the clamps' own).
AIR_M = kc.AIR_M
#: The foot's box (across, up, along the foot) on its plate beside the keel at the forefoot, m: room for the soles'
#: pressure sensors' board; the ankle's hose's radius, and a chain mark's offset beside the pipe it tells of, m.
BOX_M, ANKLE_R, MARK_M = (0.016, 0.010, 0.026), 0.7 * tees.HOSE_R, 0.012


def knee(flex=0.0):
    """{'knee pipes', 'knee hose', 'calf clips': tris} of the knee's cables, the knee's frame, the shank's side
    turned `flex` deg."""
    span_, pipes, pipe_s, clips = _fitted()
    return {'knee pipes': pipes + km._turn(pipe_s, flex), 'knee hose': tees.hose(hung.path(span_, flex)),
            'calf clips': km._turn(clips, flex)}


def span():
    """The knee's hose's span in the knee's frame with the leg straight (`hung`): from the guard's pipe's fitting to
    the collar's, each aimed at the knee's axis; the cylinders it keeps AIR_M off; its travel the knee's."""
    return _fitted()[0]


def _drums(side, rows, at=(0.0, 0.0)):
    """[(side, centre, axis, radius, half)]: a turned profile [(x, bore, outside)] about `at` (y, z) as cylinders,
    one between each two of its rows, AIR_M grown."""
    return [(side, (0.5 * (x0 + x1), at[0], at[1]), (1.0, 0.0, 0.0), max(o0, o1) + AIR_M, 0.5 * (x1 - x0))
            for (x0, _b0, o0), (x1, _b1, o1) in zip(rows[:-1], rows[1:]) if x1 > x0]


@functools.lru_cache(maxsize=None)
def _fitted():
    """(the knee's hose's span, its still pipes' tris, its turning pipes', the calf's clips): its outlets as
    `knee_clamped.fittings` lays them, a T-cross each (`tees`), one arm aimed at the knee's axis - the guard's other
    arm a stiff bridge on into the thigh's saddle's outlet, the trunk in, that outlet's collar under the seat's
    plate; the collar's out to the guard's fitting's plane, the trunk on from its other arm a hose laid still, a
    helix round the cage to the shank's seat, clipped to the collar and the cage (`tees.clipped`); the chain's
    marks on the crosses' bodies (`tees.body`) - in toward the bridge and out toward the guard's fitting, in toward
    the collar's fitting where the knee's hose comes and out toward its other arm. The knee's stator band and guard
    still, its output band and the calf's drives, cage, pitch band and collar turning - each a cylinder."""
    import numpy as np
    from machine.body import drives
    lay = kc.laid()
    (cy, cz), o_c = lay['roll'][1], lay['cage'][3]
    fitted: Any = kc.fittings()
    pad, spot = (np.array(q) for q in fitted[:2])
    aim = -pad / np.linalg.norm(pad)
    at, lead = (lay['cover'][0], float(pad[0]), float(pad[1])), kl.stated()
    lean, top = kc.tilt(lead), kc.seats(lead)[0] - 0.002 - tees.COLLAR_R       # under the seat's plate's 2 mm lap
    left = (at[0], top * math.cos(lean) + lead * math.sin(lean), lead * math.cos(lean) - top * math.sin(lean))
    pipes, fit_t = tees.pipe(at, aim, tees.STUB_M)
    up = (np.array(left[1:]) - pad) / np.linalg.norm(np.array(left[1:]) - pad)     # the trunk in, out of the thigh's
    pipes = pipes + tees.bridge(at, left) + tees.body(at, (aim, up))               # saddle; the cross's body
    x_f = lay['collar'][2][0]                                       # the collar's flange's face
    toward = -spot / np.linalg.norm(spot)
    low, stub = (x_f, float(spot[0]), float(spot[1])), x_f - fit_t[0] - tees.PIPE[0]
    pipe_s, fit_s = tees.tee(low, toward, stub, marks=('in', 'out'))
    on, reach = tees.mouth(low, toward, stub), o_c + tees.HOSE_R + AIR_M      # the trunk on: off the cross's arm
    turns, seat_x = np.linspace(math.atan2(on[2] - cz, on[1] - cy), math.pi, 3), lay['cage'][0] + 0.008

    def knot(t, x, rise):                                           # on the helix round the cage: where, which way
        way = np.array([rise, -reach * math.sin(t), reach * math.cos(t)])
        return np.array([x, cy + reach * math.cos(t), cz + reach * math.sin(t)]), way / np.linalg.norm(way)
    line = tees.run([(on, np.r_[0.0, -toward]),
                     knot(turns[1], 0.5 * (on[0] + seat_x), (seat_x - on[0]) / (turns[2] - turns[0])),
                     knot(turns[2], seat_x, 0.0)])
    pipe_s = pipe_s + tees.hose(line)
    carry = cl.turned(lay['collar'], em.SLIT) + m._band_x(o_c - 0.001, o_c, lay['cage'][0], lay['cage'][1], em.FRAME)
    clips = tees.clipped(line, [(tri + (0.0, cy, cz), c) for tri, c in carry])
    calf = list(lay['pitch'][0].values()) + list(lay['roll'][0].values()) + [(lay['cage'][0], lay['cage'][1], o_c)]

    def drum(side, x0, x1, r, at=(0.0, 0.0)):
        return _drums(side, [(x0, 0.0, r), (x1, 0.0, r)], at)
    solids = (_drums('still', lay['stator']) + drum('still', *cv.span(*lay['cover'])) + _drums('turning', lay['output'])
              + [t for q in calf for t in drum('turning', *q, at=(cy, cz))]
              + _drums('turning', lay['pitch_band'], (cy, cz)) + _drums('turning', lay['collar'], (cy, cz)))
    span_ = {'name': 'knee', 'kind': 'bus', 'travel': tuple(drives.TRAVEL['knee']),
             'from': ('still', tuple(fit_t), (0.0, aim[0], aim[1])),
             'to': ('turning', tuple(fit_s), (0.0, toward[0], toward[1])), 'solids': solids}
    return span_, pipes, pipe_s, clips


def ankle(under, fork):
    """{'ankle pipe', 'ankle hose', 'foot box', 'ankle clips', 'ankle marks': tris}: the foot's cable down out of the
    fork's yoke - its underside `under` m up the knee's frame - ahead, inside the fork's arm (`fork` [(tri, colour)]),
    in a short pipe; its hose (ANKLE_R) laid still (`tees.run`): down past the cross between the arm and the foot's
    cheek (`foot.KEEL_X`), a quarter turn forward onto the level of the coupling on the box beside the keel at the
    ball, along the keel's inside into it; clipped to the fork above the joint and to the keel below it
    (`tees.clipped`), free across the joint between;
    the bus's way out of the calf marked by the pipe, its way into the box on the box's lid (`chain`)."""
    import numpy as np
    from machine.body import skeleton
    from machine.body.figure import BALL, SHANK
    shank = kc._stack()[4]
    ahead, deep = kc.shank_line(), shank + em.OVAL_PAST[1]
    pts = km._points(list(fork), 0.002)
    arm = float(np.abs(pts[pts[:, 1] < under - 0.002][:, 0]).min())   # the fork's arms' inner faces
    top = np.array([-(arm - AIR_M - tees.PIPE[1]), under, ahead + deep - tees.PIPE[1] - AIR_M])   # under its front
    end = top - (0.0, tees.PIPE_LEG, 0.0)
    ring = tees.PIPE[1] + 0.0016
    pipe = (km._rod(top + (0.0, 0.002, 0.0), end, tees.PIPE[1], tees.PIPE_INK)
            + km._rod(end + (0.0, 0.007, 0.0), end, ring, tees.PIPE_INK))
    _stage, pin_r, _half = skeleton.CROSS
    keel = foot.parts()['foot']
    beside = -(foot.KEEL_X + AIR_M + ANKLE_R)                         # along the keel's inside, on the plate
    z = BALL                                                          # the box at the ball, by its sole's sensors
    x = -(foot.KEEL_X + AIR_M + 0.5 * BOX_M[0])                      # beside the keel
    floor = float(foot.plate(z + np.linspace(-0.5, 0.5, 9) * BOX_M[2]).max()) + 0.0005   # the plate's highest under it
    lid = floor + BOX_M[1]
    box = em._rounded(cl.filleted(cl.box((x - 0.5 * BOX_M[0], x + 0.5 * BOX_M[0]),
                                         (z - 0.5 * BOX_M[2], z + 0.5 * BOX_M[2])), 0.003), floor, lid, em.FRAME,
                      r=cl.EDGE_M, steps=1)
    back = z - 0.5 * BOX_M[2]
    socket = np.array([x, floor + 0.5 * BOX_M[1], back - 0.007])     # the coupling on its back, toward the ankle
    box += km._rod(np.array([x, socket[1], back + 0.001]), socket, ring, tees.PIPE_INK)
    past = np.array([-0.5 * (foot.KEEL_X + arm), -SHANK, pin_r + 2.0 * AIR_M + ANKLE_R])   # between the arm and the
    level = np.array([beside, socket[1], past[2] + past[1] - socket[1]])     # cheek, ahead of the pins; as far ahead
    down, fore = np.array([0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0])       # as down, the coupling's level reached
    path = tees.run([(end, down), (past, down), (level, fore), (socket, fore)])
    joint = int(np.argmin(np.abs(path[:, 1] + SHANK)))               # where it passes the ankle's axis: free across
    clips = (tees.clipped(path[:joint + 1], list(fork), ANKLE_R, (False, True))      # it, held on the fork above and
             + tees.clipped(path[joint:], keel, ANKLE_R, (True, False)))             # on the keel below
    marks = (chain.mark('out', top + (MARK_M, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
             + chain.mark('in', (x, lid, z), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    return {'ankle pipe': pipe, 'ankle hose': tees.hose(path, ANKLE_R), 'foot box': box, 'ankle clips': clips,
            'ankle marks': marks}
