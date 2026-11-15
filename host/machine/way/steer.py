"""The robot's errand's words to it: a word told once, a turn in place toward its way while off it."""
import functools
import math


def tell(g, gym, heard_fn, cmd):
    """`cmd` as the errand's word to the robot, once: the same twist again would restart its gait."""
    tw = cmd.get('twist')
    if g.get('asked') != tw or (tw is not None and g.get('governed') != tw and _mods()[3].doing(g) != 'brake'):
        heard_fn(g, gym, dict(cmd, _errand_cmd=True))


@functools.cache
def _mods():
    """What `steer` reads, imported at its first call and not looked up again each pass."""
    from machine.senses import edge
    from machine.law import latent, modal, se2
    from machine.way import route
    return edge, latent, modal, se2, route


def _away(a, b):
    """rad outside the minor arc between `a` and `b` from 0 (`0` inside it)."""
    return min(abs(a), abs(b)) * float(a * b > 0.0 or abs(a) + abs(b) > math.pi)


def steer(g, gym, heard_fn, err, wz_dir, speed, off, aim_m=1.0, pushing=False):
    """Turn in place toward `err` rad while the robot lies outside the minor arc between its aim
    (`err`) and its way (`off`) past its list, standing, or past `route.CONE` as far as the
    robot's rhythm runs (`latent.GAIN`) toward `speed` - from its walking `heading` while held
    (`has_heading`), else its pelvis -; within it, walk `speed` on the robot's turn told, pure
    pursuit's to its aim `aim_m` on - 2 sin(err) / aim_m its bend, from its heading -
    over its stance's share of a turn, YAW_RATE at most; `pushing` through a door, by `err` the
    robot's heading held instead: its turn told, its heading follows its pelvis and
    the panel's shove turned it in the doorway. Held by the robot's heading to its aim's
    bearing on a straight leg too its frame turned 1.6 rad/s (`se2.twisted`'s slew) and the robot's
    gait not: it walked 0.3-0.65 rad askew of it, and of the robot's errand's stops from the robot's
    walk's speed 64 % fell where it went 0.15 m/s or more across its heading for 16 % under
    it - a straight walk's stop none of 33; round a bend, told 25-40 deg/s, the robot crabbed 26-46 deg and
    fell; told the turn, 2-5 deg."""
    edge, latent, modal, se2, route = _mods()
    pyaw = float(edge.yaw(g['sense']))
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    dyaw = w * se2.wrap_angle(float(g['heading']) - pyaw) if g.get('has_heading') else 0.0
    turn = _away(se2.wrap_angle(err - dyaw), se2.wrap_angle(off - dyaw)) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and speed > 0.0 and not pushing
    err_f = se2.wrap_angle(pyaw + err - float(g['heading']))
    wz = max(-se2.YAW_RATE, min(se2.YAW_RATE, speed * 2.0 * math.sin(err_f) / max(aim_m, 1e-6) / se2.STEER['stance_turn']))
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(pyaw + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, wz)), (True, (speed, 0.0, 0.0)))
    tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})
