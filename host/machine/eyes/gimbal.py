"""Her eyes' gimbals as built and as her calibration told her them: each camera's turn and place.

    a = gimbal.angles('right', gz, told=True)   # (across, down) rad her right gimbal turns at her servos' gaze `gz`
    R, c = gimbal.eye('right', a, True), gimbal.place('right', a, True)   # its camera's turn and middle in her head
    gimbal.cameras()                            # her eyes' MJCF at rest, as built
    a = gimbal.played(was, gimbal.angles('left', gz), 'left')   # her left gimbal behind its pushrods' play

Two servos turn both eyes alike through pushrods to each eye's own gimbal, across about her head's up and down about
the line between her eyes; a third turns them across against each other, her vergence - converging, her left eye to her
right and her right to her left, half of it each. Two gimbals so coupled squint as a law once built: each camera sits turned on its gimbal off
square (SQUINT), its optical middle ahead of the gimbal's pivot (LEVER: turned, it moves), each gimbal turned by its
pushrods that share of a servo's turn (RODS), each pushrod's play (PLAY) leaving its gimbal behind the servo as the
servo turns and at the play's far side as it turns back (`played`). Her calibration - a church's, automotive's hall of
targets - told her each of them within CHURCH of how it is built off square; what plays she tells look by look from
her pictures (`machine.eyes.squint`), her eyes' line held to their build.
"""
import functools
import math

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

#: Her eyes' frame at rest in the head's (`cameras`' xyaxes, the rows): x her right, y up and DOWN_DEG forward, z back.
_EYE_R = ((-1.0, 0.0, 0.0), (0.0, math.cos(math.radians(DOWN_DEG)), math.sin(math.radians(DOWN_DEG))),
          (0.0, math.sin(math.radians(DOWN_DEG)), -math.cos(math.radians(DOWN_DEG))))


def turned(a):
    """(3, 3) a gimbal turned (across, down) `a` rad in her head's frame: across about its up, then down."""
    np = _np()
    (cy, sy), (cp, sp) = (math.cos(a[0]), math.sin(a[0])), (math.cos(a[1]), math.sin(a[1]))
    return np.array([[cy, sy * sp, sy * cp], [0.0, cp, -sp], [-sy, cy * sp, cy * cp]])


@functools.cache
def built(side, told=False):
    """(squint deg (3,), rods' shares (across, down), lever m, play deg (across, down)) of her eye `side` as built, or
    as her calibration told her them (`told`, CHURCH)."""
    k = 1.0 + CHURCH[side] * float(told)
    return (tuple(a * k for a in SQUINT[side]), tuple(1.0 + (r - 1.0) * k for r in RODS[side]), LEVER[side] * k,
            tuple(p * k for p in PLAY[side]))


def mounted(side=None, told=False):
    """(3, 3) her eye `side`'s camera at rest in her head, its axes the columns, its squint `built` or `told`; None
    square and unsquinted (`_EYE_R`) - her build's, laid once each and read only: laid at each call, six a law pass,
    her gaze's turns were 0.13 ms of her 1.46 ms pass."""
    return _mounted(side, bool(told))


@functools.cache
def _mounted(side, told):
    np = _np()
    R = np.array(_EYE_R).T @ (np.eye(3) if side is None else epipolar.turn(np.radians(built(side, told)[0])))
    R.setflags(write=False)
    return R


def angles(side, gz, told=False):
    """(across, down) rad her eye `side`'s gimbal turns at her servos' gaze `gz` (across, down, vergence; a vergence
    none without its third) through its pushrods (`built`), its play none: across her version less half her vergence
    her left eye's, more her right's."""
    rods, verge = built(side, told)[1], (gz[2] if len(gz) > 2 else 0.0) * (0.5 if side == 'right' else -0.5)
    return (gz[0] + verge) * rods[0], gz[1] * rods[1]


def played(was, to, side, told=False):
    """(across, down) rad her eye `side`'s gimbal turns to, at `was`, its pushrods asking `to`: within its play's half
    of it where it was, else at that half's edge behind them - as built, or as her calibration told her its play: her
    servos hers to steer, where each gimbal stands in its play she tells by how she turned them."""
    half = [math.radians(p) / 2.0 for p in built(side, told)[3]]
    return tuple(min(max(w, t - h), t + h) for w, t, h in zip(was, to, half))


def eye(side, a, told=False):
    """(3, 3) her eye `side`'s camera in her head's frame, its gimbal turned `a` (across, down) rad, its axes the
    columns, as built or `told`."""
    return turned(a) @ mounted(side, told)


@functools.cache
def pivot(side):
    """(3,) her eye `side`'s gimbal's pivot in her head's frame: its camera's middle at rest (EYES, BASELINE_M to her
    right the right one's) its lever behind it - her build's, laid once and read only."""
    np = _np()
    at = np.asarray(EYES, float) - np.array([BASELINE_M if side == 'right' else 0.0, 0.0, 0.0])
    p = at - mounted(side) @ np.array([0.0, 0.0, -LEVER[side]])
    p.setflags(write=False)
    return p


def place(side, a, told=False, R=None):
    """(3,) her eye `side`'s camera's optical middle in her head's frame, its gimbal turned `a`: its lever ahead of its
    pivot along its axis, as built or `told` - `R` its camera's turn there (`eye`'s) where it is in hand."""
    np = _np()
    return pivot(side) + (eye(side, a, told) if R is None else R) @ np.array([0.0, 0.0, -built(side, told)[2]])


def cameras():
    """Her eyes in the head's body at rest: each looks along the head's z, DOWN_DEG under it, x to her right - each
    turned and placed as built (`mounted`, `place`)."""
    out = []
    for side in ('left', 'right'):
        R = mounted(side)
        out.append('<camera name="eye_%s" pos="%.9g %.9g %.9g" xyaxes="%.9g %.9g %.9g %.9g %.9g %.9g" fovy="%g"/>'
                   % ((side,) + tuple(place(side, (0.0, 0.0)).tolist()) + tuple(R[:, :2].T.reshape(-1).tolist()) + (FOVY_DEG,)))
    return out
