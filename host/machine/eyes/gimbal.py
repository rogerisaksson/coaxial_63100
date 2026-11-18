"""The robot's eye gimbals as built and as its calibration told it them: each camera's turn and place.

    a = gimbal.angles('right', gz, told=True)   # (across, down) rad the robot's right gimbal turns at its servos' gaze `gz`
    R, c = gimbal.eye('right', a, True), gimbal.place('right', a, True)   # its camera's turn and middle in the robot's head
    gimbal.cameras()                            # its eyes' MJCF at rest, as built
    a = gimbal.played(was, gimbal.angles('left', gz), 'left')   # its left gimbal behind its pushrods' play

Two servos turn both eyes alike through pushrods to each eye's own gimbal, across about the robot's head's up
and down about the line between its eyes; a third turns them across against each other, the robot's
vergence - converging, its left eye to its right and its right to its left,
half of it each. Two gimbals so coupled squint as a law once built: each camera sits turned on its gimbal off
square (SQUINT), its optical middle ahead of the gimbal's pivot (LEVER: turned, it moves), each gimbal turned
by its pushrods that share of a servo's turn (RODS), each pushrod's play (PLAY) leaving its gimbal behind the
servo as the servo turns and at the play's far side as it turns back (`played`). The robot's calibration - a
church's, automotive's hall of targets - told it each of them within CHURCH of how it is built off
square; what plays the robot tells look by look from its pictures (`machine.eyes.squint`), the robot's
eyes' line held to their build.
"""
import functools
import math

from machine import libm
from machine.body import peripherals
from machine.eyes.stereo import BASELINE_M, FOVY_DEG, _np
from machine.eyes import epipolar

_eyes = peripherals.eyes()
EYES, DOWN_DEG = _eyes['left_at'], _eyes['down_deg']
SQUINT = _eyes['squint']
RODS = _eyes['rods']
PLAY = _eyes['play']
LEVER = _eyes['lever']
CHURCH = _eyes['church']

#: The robot's eyes' frame at rest in the head's (`cameras`' xyaxes, the rows): x its right, y up and DOWN_DEG
#: forward, z back.
_EYE_R = ((-1.0, 0.0, 0.0), (0.0, libm.cos(math.radians(DOWN_DEG)), libm.sin(math.radians(DOWN_DEG))),
          (0.0, libm.sin(math.radians(DOWN_DEG)), -libm.cos(math.radians(DOWN_DEG))))


def turned(a):
    """(3, 3) a gimbal turned (across, down) `a` rad in the robot's head's frame: across about its up, then
    down."""
    np = _np()
    (cy, sy), (cp, sp) = (libm.cos(a[0]), libm.sin(a[0])), (libm.cos(a[1]), libm.sin(a[1]))
    return np.array([[cy, sy * sp, sy * cp], [0.0, cp, -sp], [-sy, cy * sp, cy * cp]])


@functools.cache
def built(side, told=False):
    """(squint deg (3,), rods' shares (across, down), lever m, play deg (across, down)) of the robot's eye `side` as
    built, or as its calibration told it them (`told`, CHURCH)."""
    k = 1.0 + CHURCH[side] * float(told)
    return (tuple(a * k for a in SQUINT[side]), tuple(1.0 + (r - 1.0) * k for r in RODS[side]), LEVER[side] * k,
            tuple(p * k for p in PLAY[side]))


def mounted(side=None, told=False):
    """(3, 3) the robot's eye `side`'s camera at rest in its head, its axes the columns, its squint `built`
    or `told`; None square and unsquinted (`_EYE_R`) - the robot's build's, laid once each and read only: laid at
    each call, six a law pass, its gaze's turns were 0.13 ms of its 1.46 ms pass."""
    return _mounted(side, bool(told))


@functools.cache
def _mounted(side, told):
    np = _np()
    R = np.array(_EYE_R).T @ (np.eye(3) if side is None else epipolar.turn(np.radians(built(side, told)[0])))
    R.setflags(write=False)
    return R


def angles(side, gz, told=False):
    """(across, down) rad the robot's eye `side`'s gimbal turns at its servos' gaze `gz` (across, down,
    vergence; a vergence none without its third) through its pushrods (`built`), its play none: across the robot's
    version less half its vergence its left eye's, more its right's."""
    rods, verge = built(side, told)[1], (gz[2] if len(gz) > 2 else 0.0) * (0.5 if side == 'right' else -0.5)
    return (gz[0] + verge) * rods[0], gz[1] * rods[1]


def played(was, to, side, told=False):
    """(across, down) rad the robot's eye `side`'s gimbal turns to, at `was`, its pushrods asking `to`: within its
    play's half of it where it was, else at that half's edge behind them - as built, or as the robot's calibration
    told it its play: the robot's servos its own to steer, where each gimbal stands in its play the
    robot tells by how it turned them."""
    half = [math.radians(p) / 2.0 for p in built(side, told)[3]]
    return tuple(min(max(w, t - h), t + h) for w, t, h in zip(was, to, half))


def eye(side, a, told=False):
    """(3, 3) the robot's eye `side`'s camera in its head's frame, its gimbal turned `a` (across, down)
    rad, its axes the columns, as built or `told`."""
    return turned(a) @ mounted(side, told)


@functools.cache
def pivot(side):
    """(3,) the robot's eye `side`'s gimbal's pivot in its head's frame: its camera's middle at rest (EYES,
    BASELINE_M to the robot's right the right one's) its lever behind it - the robot's build's, laid once and read
    only."""
    np = _np()
    at = np.asarray(EYES, float) - np.array([BASELINE_M if side == 'right' else 0.0, 0.0, 0.0])
    p = at - mounted(side) @ np.array([0.0, 0.0, -LEVER[side]])
    p.setflags(write=False)
    return p


def place(side, a, told=False, R=None):
    """(3,) the robot's eye `side`'s camera's optical middle in its head's frame, its gimbal turned `a`: its
    lever ahead of its pivot along its axis, as built or `told` - `R` its camera's turn there (`eye`'s) where it is in
    hand."""
    np = _np()
    return pivot(side) + (eye(side, a, told) if R is None else R) @ np.array([0.0, 0.0, -built(side, told)[2]])


def cameras():
    """The robot's eyes in the head's body at rest: each looks along the head's z, DOWN_DEG under it, x to the
    robot's right - each turned and placed as built (`mounted`, `place`)."""
    out = []
    for side in ('left', 'right'):
        R = mounted(side)
        out.append('<camera name="eye_%s" pos="%.9g %.9g %.9g" xyaxes="%.9g %.9g %.9g %.9g %.9g %.9g" fovy="%g"/>'
                   % ((side,) + tuple(place(side, (0.0, 0.0)).tolist()) + tuple(R[:, :2].T.reshape(-1).tolist()) + (FOVY_DEG,)))
    return out
